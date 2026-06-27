//! `pedalkernel diag` — compile-time MNA/scattering diagnostics (IPC Phase A).
//!
//! Two modes:
//!
//! * `pedalkernel diag <file.pedal> [--ipc <out>] [--knob K=V]…` — compile the
//!   pedal, capture the MNA/scattering snapshot, optionally write it to a
//!   memory-mapped IPC file, and print it.
//! * `pedalkernel diag --ipc <file>` (no pedal) — mmap an existing IPC file and
//!   print the snapshot.
//!
//! The diagnostics path only runs when this subcommand is invoked — the audio
//! engine and `process`/`tui` paths are untouched (Rule 3, additive).

use pedalkernel::compiler::compile_pedal;
use pedalkernel::diag_ipc;
use pedalkernel::dsl::parse_pedal_file;
use pedalkernel::PedalProcessor;
use pedalkernel_rt::diag::{DiagSnapshot, MnaStageSnapshot};
use std::path::Path;
use std::process;

/// Entry point for `pedalkernel diag`.
///
/// * `file` — `.pedal` source to compile (None → read-only from `--ipc`).
/// * `ipc` — IPC file to write (with `file`) or read (without `file`).
/// * `knobs` — `Name=Value` control overrides applied before capture.
pub fn run(file: Option<&str>, ipc: Option<&str>, knobs: &[String]) {
    match (file, ipc) {
        // ── Read-only: mmap an existing IPC file and print it. ───────────────
        (None, Some(ipc_path)) => {
            let snapshot = diag_ipc::read_snapshot(Path::new(ipc_path)).unwrap_or_else(|e| {
                eprintln!("Error reading IPC file {ipc_path}: {e}");
                process::exit(1);
            });
            print_snapshot(&snapshot);
        }

        // ── Compile a pedal → capture → (optionally write) → print. ──────────
        (Some(file_path), ipc_opt) => {
            let snapshot = compile_and_capture(file_path, knobs);
            if let Some(ipc_path) = ipc_opt {
                diag_ipc::write_snapshot(Path::new(ipc_path), &snapshot).unwrap_or_else(|e| {
                    eprintln!("Error writing IPC file {ipc_path}: {e}");
                    process::exit(1);
                });
                eprintln!("wrote diagnostics IPC → {ipc_path}");
            }
            print_snapshot(&snapshot);
        }

        (None, None) => {
            eprintln!("error: `diag` needs either a <file.pedal> or --ipc <file>");
            process::exit(2);
        }
    }
}

/// Parse + (expand) + compile a `.pedal`, apply knobs, return its snapshot.
fn compile_and_capture(file_path: &str, knobs: &[String]) -> DiagSnapshot {
    let source = std::fs::read_to_string(file_path).unwrap_or_else(|e| {
        eprintln!("Error reading {file_path}: {e}");
        process::exit(1);
    });

    let mut pedal = parse_pedal_file(&source).unwrap_or_else(|e| {
        eprintln!("Parse error: {e}");
        process::exit(1);
    });

    // Expand file-based `use(...)` subcircuit instances into a flat pedal.
    if !pedal.uses.is_empty() {
        let base_dir = Path::new(file_path)
            .parent()
            .unwrap_or_else(|| Path::new("."));
        pedal = pedalkernel::dsl_expand::expand_uses(&pedal, base_dir).unwrap_or_else(|e| {
            eprintln!("Subcircuit expansion error: {e}");
            process::exit(1);
        });
    }

    let mut compiled = compile_pedal(&pedal, 48000.0).unwrap_or_else(|e| {
        eprintln!("Compile error: {e}");
        process::exit(1);
    });

    // Apply `Name=Value` knob overrides so the operating point reflects them.
    for kv in knobs {
        if let Some((name, val)) = kv.split_once('=') {
            match val.trim().parse::<f64>() {
                Ok(v) => compiled.set_control(name.trim(), v),
                Err(_) => eprintln!("  ignoring malformed knob `{kv}` (expected Name=Value)"),
            }
        } else {
            eprintln!("  ignoring malformed knob `{kv}` (expected Name=Value)");
        }
    }

    compiled.diag_snapshot(&pedal.name, file_path)
}

/// Pretty-print a diagnostics snapshot to stdout.
fn print_snapshot(snap: &DiagSnapshot) {
    println!("════════════════════════════════════════════════════════════════════");
    println!("Diagnostics MNA/Scattering Snapshot (IPC v{})", diag_ipc::VERSION);
    println!("════════════════════════════════════════════════════════════════════");
    println!("pedal       : {}", snap.pedal_name);
    println!("source      : {}", snap.source_path);
    println!("sample rate : {} Hz", snap.sample_rate);
    println!("stages      : {} total", snap.stage_count);
    print!("stage kinds :");
    for (i, k) in snap.stage_kinds.iter().enumerate() {
        print!(" [{i}]{k}");
    }
    println!();
    println!("MultiNl     : {} stage(s)", snap.multinl.len());

    if snap.multinl.is_empty() {
        println!("\n(no MultiNl stages — nothing to dump)");
        return;
    }

    for st in &snap.multinl {
        print_stage(st);
    }
}

fn print_stage(st: &MnaStageSnapshot) {
    println!("\n────────────────────────────────────────────────────────────────────");
    println!(
        "[Stage {}] MultiNl  {}",
        st.stage_index,
        if st.label.is_empty() {
            String::new()
        } else {
            format!("({})", st.label)
        }
    );
    println!("────────────────────────────────────────────────────────────────────");
    println!(
        "  n_nl={}  n_passive={}  n_ports_total={}  output_port={}",
        st.n_nl, st.n_passive, st.n_ports_total, st.output_port
    );
    println!(
        "  compensation={:.6}  supply={:.2}V  bypass_serial={}",
        st.compensation, st.supply_voltage, st.bypass_serial
    );

    // ── Port labels + resistances ────────────────────────────────────────
    println!("\n  ports (adaptor order):");
    for (i, lbl) in st.port_labels.iter().enumerate() {
        let r = st.port_resistances.get(i).copied().unwrap_or(f64::NAN);
        println!("    [{i:2}] {lbl:<22} R = {}", fmt_ohms(r));
    }

    // ── Full standard scattering matrix S ────────────────────────────────
    if st.s_dim > 0 && st.s_full.len() == st.s_dim * st.s_dim {
        println!("\n  S (standard, {0}×{0}):", st.s_dim);
        // Column header.
        print!("        ");
        for j in 0..st.s_dim {
            print!("{:>10}", short_label(&st.port_labels, j));
        }
        println!();
        for i in 0..st.s_dim {
            print!("  {:>6}", short_label(&st.port_labels, i));
            for j in 0..st.s_dim {
                print!("{:>10.4}", st.s_full[i * st.s_dim + j]);
            }
            println!();
        }
    }

    // ── Per-NL-port linearization (gm / companion) ───────────────────────
    println!("\n  NL-port linearization (operating point):");
    println!(
        "    {:<22} {:>10} {:>12} {:>12} {:>12} {:>12}",
        "port", "v_op", "gm", "1/gm", "cross_gm", "port_R"
    );
    for nl in &st.nl_linearization {
        let cross = nl
            .cross_gm
            .map(|c| format!("{c:>12.6e}"))
            .unwrap_or_else(|| format!("{:>12}", "-"));
        println!(
            "    {:<22} {:>10.4} {:>12.6e} {:>12} {} {:>12.1}",
            nl.label,
            nl.v_op,
            nl.gm,
            fmt_ohms_short(nl.r_companion),
            cross,
            nl.port_resistance
        );
    }

    // ── Extraction + VS injection ────────────────────────────────────────
    println!("\n  output extraction:");
    match &st.extraction_coeffs {
        Some(c) => {
            println!("    extract_coeffs = {}", fmt_vec(c));
            println!("    extract_vs     = {:.6}", st.extraction_vs);
        }
        None => println!("    (none — output read via WDF port {})", st.output_port),
    }
    match &st.vs_injection {
        Some(v) => println!("    vs_injection   = {}", fmt_vec(v)),
        None => println!("    vs_injection   = (none — adapted WDF input port)"),
    }
}

/// Short per-column header label: take the device id portion after any `:`.
fn short_label(labels: &[String], idx: usize) -> String {
    let full = labels.get(idx).map(String::as_str).unwrap_or("?");
    let s = full.rsplit(':').next().unwrap_or(full);
    if s.len() > 9 {
        s[..9].to_string()
    } else {
        s.to_string()
    }
}

fn fmt_vec(v: &[f64]) -> String {
    let parts: Vec<String> = v.iter().map(|x| format!("{x:+.4}")).collect();
    format!("[{}]", parts.join(", "))
}

fn fmt_ohms(r: f64) -> String {
    if !r.is_finite() {
        return "∞".to_string();
    }
    if r >= 1e6 {
        format!("{:.3} MΩ", r / 1e6)
    } else if r >= 1e3 {
        format!("{:.3} kΩ", r / 1e3)
    } else {
        format!("{r:.2} Ω")
    }
}

fn fmt_ohms_short(r: f64) -> String {
    if !r.is_finite() {
        "∞".to_string()
    } else if r >= 1e6 {
        format!("{:.2}M", r / 1e6)
    } else if r >= 1e3 {
        format!("{:.2}k", r / 1e3)
    } else {
        format!("{r:.1}")
    }
}
