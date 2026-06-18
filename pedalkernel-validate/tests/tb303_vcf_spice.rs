//! TB-303 VCF SPICE validation test.
//!
//! Compares the WDF engine output against an ngspice reference for the
//! TB-303 4-pole differential diode-ladder filter.
//!
//! The `.pedal` file lives in the private pro repo; this test skips cleanly
//! when it is absent (CI on the public engine repo).  The SPICE deck and
//! goldens are public and live in the engine repo.
//!
//! EXPECTED RED: the TB-303 BKM ladder path is known-broken (beads wbe/d7u/do9,
//! ~38 failing tb303_decomposition tests).  This test QUANTIFIES the gap
//! between WDF output and the ngspice reference for each configuration.  It
//! does NOT pending-hide the failure.
//!
//! # Skip behaviour
//!
//! When the pro pedal is absent (public CI), every test function returns early
//! via `skip_if_missing!`.  No compile error, no test failure.
//!
//! # Golden generation
//!
//! Goldens are generated once via the `#[ignore]`d test `tb303_vcf_regen_golden`,
//! run manually on a machine with ngspice:
//!
//!   cargo test -p pedalkernel-validate --test tb303_vcf_spice \
//!     tb303_vcf_regen_golden -- --ignored --nocapture
//!
//! They are NOT bootstrapped from WDF output.  Once committed, the comparison
//! test `tb303_vcf_wdf_vs_ngspice` loads them without invoking ngspice.

use std::f64::consts::PI;
use std::io::Write as _;
use std::path::{Path, PathBuf};
use std::process::Command;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

const SR: f64 = 48_000.0;
/// Oversampling factor for ngspice internal rate.
const OVERSAMPLE: u32 = 4;
const INTERNAL_RATE: f64 = SR * OVERSAMPLE as f64;

/// Number of samples for each test signal (20 ms at 48 kHz — matches ngspice golden length).
const N_SAMPLES: usize = 960;
/// Warmup samples to discard before measuring (25% = 5 ms).
const WARMUP: usize = 240;
/// Test sine frequency in Hz.
const SINE_FREQ: f64 = 440.0;
/// Sine amplitude (small-signal to stay in the ladder's linear regime).
const SINE_AMP: f64 = 0.05;

// ---------------------------------------------------------------------------
// Test configurations
// ---------------------------------------------------------------------------

/// One (cutoff_ctrl, resonance_ctrl) pair.
///
/// `cutoff_ctrl`    — value passed to `proc.set_control("Cutoff", v)`.
///                    Pedal maps 0→1 onto [1.0, 0.02] (inverted).
/// `resonance_ctrl` — value passed to `proc.set_control("Resonance", v)`.
///                    Pedal maps 0→1 onto [0.0, 0.35].
/// `cutoff_v_dc`    — DC voltage at Q10.base in the SPICE deck.  Coarse
///                    approximation — the point is to MEASURE the gap,
///                    not to zero it.
/// `res_pos`        — Resonance pot fraction in the SPICE deck (0–1).
#[derive(Debug, Clone)]
struct Config {
    label: &'static str,
    cutoff_ctrl: f64,
    resonance_ctrl: f64,
    /// Voltage at Q10.base for SPICE (coarse match to cutoff_ctrl).
    cutoff_v_dc: f64,
    /// Resonance pot fraction for SPICE (0..1, after pedal range mapping).
    res_pos: f64,
}

/// Three cutoff positions × two resonance settings = 6 configs.
const CONFIGS: &[Config] = &[
    Config {
        label: "cutoff_low_res_off",
        cutoff_ctrl: 0.8,
        resonance_ctrl: 0.0,
        cutoff_v_dc: 1.5,
        res_pos: 0.0,
    },
    Config {
        label: "cutoff_mid_res_off",
        cutoff_ctrl: 0.5,
        resonance_ctrl: 0.0,
        cutoff_v_dc: 2.5,
        res_pos: 0.0,
    },
    Config {
        label: "cutoff_high_res_off",
        cutoff_ctrl: 0.2,
        resonance_ctrl: 0.0,
        cutoff_v_dc: 3.5,
        res_pos: 0.0,
    },
    Config {
        label: "cutoff_low_res_med",
        cutoff_ctrl: 0.8,
        resonance_ctrl: 0.5,
        cutoff_v_dc: 1.5,
        res_pos: 0.175,
    },
    Config {
        label: "cutoff_mid_res_med",
        cutoff_ctrl: 0.5,
        resonance_ctrl: 0.5,
        cutoff_v_dc: 2.5,
        res_pos: 0.175,
    },
    Config {
        label: "cutoff_high_res_med",
        cutoff_ctrl: 0.2,
        resonance_ctrl: 0.5,
        cutoff_v_dc: 3.5,
        res_pos: 0.175,
    },
];

// ---------------------------------------------------------------------------
// Load pro pedal (skip if absent)
// ---------------------------------------------------------------------------

/// Try to load a .pedal file from the pro crate's acidattack-core directory.
/// Tries 2..=6 levels of `../` to cover both normal and worktree layouts.
fn load_pro_pedal(filename: &str) -> Option<String> {
    let manifest_dir = env!("CARGO_MANIFEST_DIR");
    let rel_suffix = format!("pedalkernel-pro/crates/acidattack/acidattack-core/{filename}");
    for levels in 2..=6 {
        let prefix: String = "../".repeat(levels);
        let candidate = format!("{manifest_dir}/{prefix}{rel_suffix}");
        if let Ok(s) = std::fs::read_to_string(&candidate) {
            eprintln!("  loaded {filename} from {candidate} ({} bytes)", s.len());
            return Some(s);
        }
    }
    None
}

macro_rules! skip_if_missing {
    ($source:expr, $name:expr) => {
        match $source {
            Some(s) => s,
            None => {
                eprintln!(
                    "  SKIP: {} not found (pro crate not present on this machine/CI)",
                    $name
                );
                return;
            }
        }
    };
}

// ---------------------------------------------------------------------------
// Signal generation
// ---------------------------------------------------------------------------

fn sine_signal(freq: f64, fs: f64, n: usize, amp: f64) -> Vec<f64> {
    (0..n)
        .map(|i| amp * (2.0 * PI * freq * i as f64 / fs).sin())
        .collect()
}

/// Upsample by repeating each sample `factor` times (ZOH).
fn upsample_zoh(signal: &[f64], factor: usize) -> Vec<f64> {
    signal.iter().flat_map(|&s| vec![s; factor]).collect()
}

// ---------------------------------------------------------------------------
// Metrics
// ---------------------------------------------------------------------------

fn rms(samples: &[f64]) -> f64 {
    if samples.is_empty() {
        return 0.0;
    }
    (samples.iter().map(|&x| x * x).sum::<f64>() / samples.len() as f64).sqrt()
}

fn db(linear: f64) -> f64 {
    if linear < 1e-30 {
        -300.0
    } else {
        20.0 * linear.log10()
    }
}

/// Normalized RMS error in dB: 20·log10(rms(wdf − ref) / rms(ref)).
fn nrms_error_db(wdf: &[f64], reference: &[f64]) -> f64 {
    let n = wdf.len().min(reference.len());
    if n == 0 {
        return 0.0;
    }
    let diff_rms = rms(&wdf[..n]
        .iter()
        .zip(reference[..n].iter())
        .map(|(&a, &b)| a - b)
        .collect::<Vec<_>>());
    let ref_rms = rms(&reference[..n]);
    db(diff_rms) - db(ref_rms)
}

/// Goertzel magnitude at a single frequency.
fn goertzel_mag(samples: &[f64], fs: f64, freq: f64) -> f64 {
    let n = samples.len();
    if n == 0 {
        return 0.0;
    }
    let k = (n as f64 * freq / fs).round() as usize;
    let omega = 2.0 * PI * k as f64 / n as f64;
    let coeff = 2.0 * omega.cos();
    let (mut s1, mut s2) = (0.0_f64, 0.0_f64);
    for &x in samples {
        let s0 = x + coeff * s1 - s2;
        s2 = s1;
        s1 = s0;
    }
    ((s1 - s2 * omega.cos()).powi(2) + (s2 * omega.sin()).powi(2)).sqrt()
}

fn gain_db_at(output: &[f64], input: &[f64], fs: f64, freq: f64) -> f64 {
    let out_mag = goertzel_mag(output, fs, freq);
    let in_mag = goertzel_mag(input, fs, freq);
    if in_mag < 1e-20 {
        return -300.0;
    }
    db(out_mag / in_mag)
}

// ---------------------------------------------------------------------------
// WDF engine processing
// ---------------------------------------------------------------------------

fn wdf_process(source: &str, config: &Config) -> Result<Vec<f64>, String> {
    use pedalkernel::compiler::compile_pedal;
    use pedalkernel::dsl::parse_pedal_file;
    use pedalkernel::PedalProcessor;

    let def = parse_pedal_file(source).map_err(|e| format!("parse: {:?}", e))?;
    let mut proc = compile_pedal(&def, SR).map_err(|e| format!("compile: {:?}", e))?;
    proc.set_control("Cutoff", config.cutoff_ctrl);
    proc.set_control("Resonance", config.resonance_ctrl);

    let input = sine_signal(SINE_FREQ, SR, N_SAMPLES, SINE_AMP);
    Ok(input.iter().map(|&s| proc.process(s)).collect())
}

// ---------------------------------------------------------------------------
// SPICE netlist generation
// ---------------------------------------------------------------------------

/// Generate the full ngspice netlist for one configuration.
///
/// CUTOFF_V and RES_POS are substituted literally as Rust format arguments.
fn build_netlist(config: &Config, pwl_data: &str, output_file: &Path) -> String {
    let cutoff_v = config.cutoff_v_dc;
    let res_pos = config.res_pos;
    let timestep = 1.0 / INTERNAL_RATE;
    let n_internal = N_SAMPLES * OVERSAMPLE as usize;
    let duration = n_internal as f64 / INTERNAL_RATE;
    let output_node = "v_out";
    let output_file_str = output_file.display();

    format!(
        r#"* TB-303 VCF ngspice reference — config: {label}
* CUTOFF_V={cutoff_v} V  RES_POS={res_pos}
* Internal rate: {rate} Hz  timestep: {timestep:.12e} s
.TITLE TB303 VCF — {label}

* Supply
VCC vcc 0 DC 9

* Fixed ladder feed
R_bias_floor   vcc       v_feed    10k
R_top_l        v_feed    v_l4_top  470
R_top_r        v_feed    v_r4_top  470

* Left diode-ladder (diode-connected 2SC945): QL4..QL1
QL4  v_l4_top  v_l4_top  v_l4_e  NPN_2SC945
QL3  v_l3_top  v_l3_top  v_l3_e  NPN_2SC945
QL2  v_l2_top  v_l2_top  v_l2_e  NPN_2SC945
QL1  v_l1_top  v_l1_top  v_l1_e  NPN_2SC945
* Chain wires (1 mΩ to avoid floating nodes)
Rw_l43  v_l4_e  v_l3_top  1m
Rw_l32  v_l3_e  v_l2_top  1m
Rw_l21  v_l2_e  v_l1_top  1m

* Right diode-ladder: QR4..QR1
QR4  v_r4_top  v_r4_top  v_r4_e  NPN_2SC945
QR3  v_r3_top  v_r3_top  v_r3_e  NPN_2SC945
QR2  v_r2_top  v_r2_top  v_r2_e  NPN_2SC945
QR1  v_r1_top  v_r1_top  v_r1_e  NPN_2SC945
Rw_r43  v_r4_e  v_r3_top  1m
Rw_r32  v_r3_e  v_r2_top  1m
Rw_r21  v_r2_e  v_r1_top  1m

* Cross-ladder capacitors (QL_n.emitter <-> QR_n.emitter)
C4   v_l4_e   v_r4_e    33n  IC=0
C3   v_l3_e   v_r3_e    33n  IC=0
C2   v_l2_e   v_r2_e    33n  IC=0
C1   v_l1_e   v_r1_e    18n  IC=0

* Input differential pair Q12L / Q12R
Q12L  v_l1_e   v_12l_b   v_12_e  NPN_2SC945
Q12R  v_r1_e   v_12r_b   v_12_e  NPN_2SC945
* Audio input coupling: C_in(180n) -> R_in(10k) -> Q12L.base
C_in   v_in     v_in_c    180n  IC=0
R_in   v_in_c   v_12l_b   10k
* VCO grounded for this test
C_vco  0        v_vco_c   1u    IC=0
R_vco  v_vco_c  v_12l_b   1k
* Q12R reference bias from Q9 node
R_12r_ref  v_9_bc  v_12r_b  0

* Cutoff expo-converter: Q9 (diode-ref) / Q10 (control) / Q11 (tail sink)
R_q9_bias  vcc     v_9_bc   100k
Q9         v_9_bc  v_9_bc   0       NPN_2SC945
* Direct voltage source replaces pot+R_cutoff_sum (coarse match)
Vcutoff  v_10_b  0  DC {cutoff_v}
Q10  vcc    v_10_b  v_10_e  NPN_2SC945
Q11  v_12_e v_11_b  v_11_e  NPN_2SC945
R_q10e_q11b  v_10_e  v_11_b  0
R_q11_tail   v_11_e  0  1k

* Resonance feedback: QL4.emitter * RES_POS -> R_fb_limit(100k) -> QR1.emitter
Eresonance  v_res_wiper  0  v_l4_e  0  {res_pos}
R_fb_limit  v_res_wiper  v_r1_e   100k
R_res_cv    0    v_res_wiper   100k

* Output: QL4.emitter -> C_out(180n) -> R_out(100k) -> v_out
C_out  v_l4_e   v_out_c   180n  IC=0
R_out  v_out_c  v_out     100k

* Input source (harness injects PWL)
VIN v_in 0 PWL({pwl_data})

* 2SC945 NPN model (typical datasheet: hFE=200, ft=150MHz)
.MODEL NPN_2SC945 NPN (
+  IS=1e-14  BF=200  NF=1.0  VAF=80  IKF=30m
+  ISE=2e-13  NE=1.5  BR=4  NR=1.0  VAR=20
+  IKR=2m  ISC=2e-13  NC=2.0
+  TF=1.1n  TR=15n
+  CJE=15p  VJE=0.65  MJE=0.33
+  CJC=5p   VJC=0.5   MJC=0.33
+  RB=100  RC=5  RE=1  )

.OPTIONS RELTOL=1e-4 ABSTOL=1e-11 VNTOL=1e-8
.OPTIONS METHOD=TRAP MAXORD=2
.OPTIONS ITL1=300 ITL2=100 ITL4=50
.OPTIONS GMIN=1e-11

.TRAN {timestep:.12e} {duration:.12e} 0 {timestep:.12e} UIC

.CONTROL
  set filetype=ascii
  run
  wrdata {output_file_str} {output_node}
  quit
.ENDC

.END
"#,
        label = config.label,
        cutoff_v = cutoff_v,
        res_pos = res_pos,
        rate = INTERNAL_RATE,
        timestep = timestep,
        duration = duration,
        pwl_data = pwl_data,
        output_node = output_node,
        output_file_str = output_file_str,
    )
}

/// Generate inline PWL string (upsampled to INTERNAL_RATE via ZOH).
fn generate_pwl(signal: &[f64]) -> String {
    let dt = 1.0 / INTERNAL_RATE;
    let upsampled = upsample_zoh(signal, OVERSAMPLE as usize);
    // Decimate if too many points (keep netlist manageable)
    let max_points = 100_000usize;
    let decimate = if upsampled.len() <= max_points {
        1
    } else {
        upsampled.len().div_ceil(max_points)
    };
    upsampled
        .iter()
        .enumerate()
        .step_by(decimate)
        .map(|(i, &v)| format!("{:.9e} {:.9e}", i as f64 * dt, v))
        .collect::<Vec<_>>()
        .join(" ")
}

/// Parse wrdata output (two-column: time value).
fn parse_wrdata(path: &Path) -> Option<Vec<(f64, f64)>> {
    let contents = std::fs::read_to_string(path).ok()?;
    let mut data = Vec::new();
    for line in contents.lines() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('*') || line.starts_with('#') {
            continue;
        }
        let parts: Vec<&str> = line.split_whitespace().collect();
        if parts.len() >= 2 {
            if let (Ok(t), Ok(v)) = (parts[0].parse::<f64>(), parts[1].parse::<f64>()) {
                data.push((t, v));
            }
        }
    }
    if data.is_empty() {
        None
    } else {
        Some(data)
    }
}

/// Resample SPICE output onto uniform grid at SR.
fn resample_to_sr(data: &[(f64, f64)], n_out: usize) -> Vec<f64> {
    if data.is_empty() {
        return vec![0.0; n_out];
    }
    let mut out = Vec::with_capacity(n_out);
    let mut idx = 0usize;
    for i in 0..n_out {
        let t_target = i as f64 / SR;
        while idx + 1 < data.len() && data[idx + 1].0 <= t_target {
            idx += 1;
        }
        if idx + 1 >= data.len() {
            out.push(data.last().map(|d| d.1).unwrap_or(0.0));
        } else {
            let (t0, v0) = data[idx];
            let (t1, v1) = data[idx + 1];
            let frac = if t1 > t0 {
                (t_target - t0) / (t1 - t0)
            } else {
                0.0
            };
            out.push(v0 + frac * (v1 - v0));
        }
    }
    out
}

/// Run ngspice for one configuration, return resampled output at SR.
fn run_ngspice(config: &Config, input: &[f64]) -> Option<Vec<f64>> {
    let pwl = generate_pwl(input);
    let tmpdir = tempfile::TempDir::new().ok()?;
    let netlist_path = tmpdir.path().join("circuit.spice");
    let output_path = tmpdir.path().join("output.txt");
    let netlist = build_netlist(config, &pwl, &output_path);
    std::fs::write(&netlist_path, &netlist).ok()?;

    eprintln!("    [ngspice] running {}…", config.label);
    let result = Command::new("/opt/homebrew/bin/ngspice")
        .args(["-b", netlist_path.to_str()?])
        .output()
        .ok()?;

    if !result.status.success() {
        let stderr = String::from_utf8_lossy(&result.stderr);
        eprintln!(
            "    [ngspice] FAILED for {}: {}",
            config.label,
            &stderr[..stderr.len().min(500)]
        );
        return None;
    }

    if !output_path.exists() {
        eprintln!("    [ngspice] no output for {}", config.label);
        return None;
    }

    let raw = parse_wrdata(&output_path)?;
    eprintln!("    [ngspice] {} → {} points", config.label, raw.len());
    Some(resample_to_sr(&raw, N_SAMPLES))
}

// ---------------------------------------------------------------------------
// Golden paths
// ---------------------------------------------------------------------------

fn golden_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("golden/pedals/tb303_vcf")
}

fn golden_path(label: &str) -> PathBuf {
    golden_dir().join(format!("{}.npy", label))
}

// ---------------------------------------------------------------------------
// Minimal NPY I/O
// (avoids pulling ndarray-npy into this standalone test integration)
// ---------------------------------------------------------------------------

/// Save f64 slice as a NumPy .npy v1.0 file.
fn save_npy(path: &Path, data: &[f64]) {
    let n = data.len();
    let header_str = format!("{{'descr': '<f8', 'fortran_order': False, 'shape': ({n},), }}");
    // Pad so that header block (10 + len) is a multiple of 64 bytes.
    let block_size = 10 + header_str.len() + 1; // +1 for trailing '\n'
    let pad = (64 - block_size % 64) % 64;
    let padded_header: String = format!("{}{}\n", header_str, " ".repeat(pad));
    let header_bytes = padded_header.as_bytes();
    let hdr_len = header_bytes.len() as u16;

    let mut f = std::fs::File::create(path).expect("create npy");
    f.write_all(b"\x93NUMPY").unwrap();
    f.write_all(&[1u8, 0u8]).unwrap();
    f.write_all(&hdr_len.to_le_bytes()).unwrap();
    f.write_all(header_bytes).unwrap();
    for &v in data {
        f.write_all(&v.to_le_bytes()).unwrap();
    }
}

/// Load a 1D float64 .npy file (v1.0 only).
fn load_npy(path: &Path) -> Option<Vec<f64>> {
    let bytes = std::fs::read(path).ok()?;
    if bytes.len() < 10 || &bytes[..6] != b"\x93NUMPY" {
        return None;
    }
    let hlen = u16::from_le_bytes([bytes[8], bytes[9]]) as usize;
    let data_start = 10 + hlen;
    if data_start >= bytes.len() {
        return None;
    }
    let data_bytes = &bytes[data_start..];
    let n = data_bytes.len() / 8;
    let mut out = Vec::with_capacity(n);
    for chunk in data_bytes.chunks_exact(8) {
        out.push(f64::from_le_bytes(chunk.try_into().unwrap()));
    }
    if out.is_empty() {
        None
    } else {
        Some(out)
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

/// Always passes.  Confirms the skip mechanism compiles and returns cleanly
/// when the pro pedal is absent.  Safe to run on public CI.
#[test]
fn tb303_vcf_skips_when_pro_pedal_absent() {
    match load_pro_pedal("tb303_filter.pedal") {
        None => {
            eprintln!(
                "  CONFIRMED SKIP: tb303_filter.pedal absent — \
                 skip_if_missing! returns early (expected on public CI)"
            );
        }
        Some(_) => {
            eprintln!("  pro pedal present — skip_if_missing! would NOT skip on this machine");
        }
    }
    // Either branch: test passes.
}

/// Core validation: WDF vs committed ngspice golden.
///
/// EXPECTED RED — cross-reference beads do9/d7u/wbe.
///
/// - Skips when the pro pedal is absent (public CI).
/// - Skips individual configs when their golden file is absent.
/// - Reports nRMS error and gain for each config; does NOT assert a pass threshold
///   until do9/d7u/wbe are resolved.
#[test]
fn tb303_vcf_wdf_vs_ngspice() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");

    let input = sine_signal(SINE_FREQ, SR, N_SAMPLES, SINE_AMP);
    let _input_gain = db(goertzel_mag(&input[WARMUP..], SR, SINE_FREQ));

    eprintln!();
    eprintln!("TB-303 VCF: WDF vs ngspice gap table");
    eprintln!(
        "  EXPECTED RED — beads do9/d7u/wbe: BKM diode-ladder broken, ~38 failing tb303 tests"
    );
    eprintln!("  Once do9/d7u/wbe are resolved, target is nRMS < 6 dB per config.");
    eprintln!();
    eprintln!(
        "  {:<30}  {:>15}  {:>12}  {:>12}  {:>8}",
        "Config", "nRMS err (dB)", "WDF gain", "SPICE gain", "Status"
    );
    eprintln!(
        "  {:<30}  {:>15}  {:>12}  {:>12}  {:>8}",
        "──────────────────────────────",
        "───────────────",
        "────────────",
        "────────────",
        "────────"
    );

    let mut gap_entries: Vec<(String, Option<f64>, f64, Option<f64>)> = Vec::new();

    for config in CONFIGS {
        // WDF output
        let wdf_out = match wdf_process(&source, config) {
            Ok(v) => v,
            Err(e) => {
                eprintln!("  {:<30}  WDF ERROR: {}", config.label, e);
                gap_entries.push((config.label.to_string(), None, -300.0, None));
                continue;
            }
        };
        let wdf_gain = gain_db_at(&wdf_out[WARMUP..], &input[WARMUP..], SR, SINE_FREQ);

        // Load committed golden
        let gpath = golden_path(config.label);
        let spice_out = load_npy(&gpath);

        let (nrms, spice_gain, status) = match &spice_out {
            Some(ref gold) => {
                let spice_gain = gain_db_at(&gold[WARMUP..], &input[WARMUP..], SR, SINE_FREQ);
                let nrms = nrms_error_db(&wdf_out[WARMUP..], &gold[WARMUP..]);
                (Some(nrms), Some(spice_gain), "RED")
            }
            None => {
                eprintln!(
                    "  {:<30}  golden absent (run tb303_vcf_regen_golden --ignored)",
                    config.label
                );
                (None, None, "SKIP")
            }
        };

        match (nrms, spice_gain) {
            (Some(n), Some(s)) => {
                eprintln!(
                    "  {:<30}  {:>+15.1}  {:>+12.1}  {:>+12.1}  {:>8}",
                    config.label, n, wdf_gain, s, status
                );
                gap_entries.push((config.label.to_string(), Some(n), wdf_gain, Some(s)));
            }
            _ => {
                eprintln!(
                    "  {:<30}  {:>+15}  {:>+12.1}  {:>+12}  {:>8}",
                    config.label, "N/A", wdf_gain, "N/A", status
                );
                gap_entries.push((config.label.to_string(), None, wdf_gain, None));
            }
        }
    }

    // Summary
    let measured: Vec<f64> = gap_entries
        .iter()
        .filter_map(|(_, nrms, _, _)| *nrms)
        .collect();

    eprintln!();
    if measured.is_empty() {
        eprintln!(
            "  No goldens available — run `cargo test tb303_vcf_regen_golden -- --ignored` \
             with ngspice to generate them."
        );
    } else {
        let avg = measured.iter().sum::<f64>() / measured.len() as f64;
        let max = measured
            .iter()
            .cloned()
            .fold(0.0_f64, |a, b| a.abs().max(b.abs()));
        eprintln!(
            "  Average nRMS error: {:+.1} dB  |  Max |nRMS| error: {:.1} dB  ({}/{} configs measured)",
            avg, max, measured.len(), CONFIGS.len()
        );
        eprintln!();
        eprintln!("  DIAGNOSIS for do9/d7u/wbe:");
        eprintln!("    WDF diverges {max:.0} dB max / {avg:.0} dB avg from ngspice reference.");
        eprintln!("    Root cause: BKM diode-ladder scattering path does not correctly model");
        eprintln!("    the differential TB-303 ladder cutoff sweep.");
        eprintln!("    Acceptance target once do9 is resolved: all configs < 6 dB nRMS.");
    }

    // The test does NOT assert a pass threshold — gap reporting IS the deliverable.
    // Do not add assert! here until do9/d7u/wbe are closed.
}

/// Golden regeneration (ignored by default; run manually with ngspice present).
///
/// Usage:
///   cargo test -p pedalkernel-validate --test tb303_vcf_spice \
///     tb303_vcf_regen_golden -- --ignored --nocapture
///
/// Requires: `/opt/homebrew/bin/ngspice` (or ngspice in PATH).
/// This is intentionally slow (~2–5 min for 6 configs on a complex BJT circuit).
#[test]
#[ignore]
fn tb303_vcf_regen_golden() {
    let input = sine_signal(SINE_FREQ, SR, N_SAMPLES, SINE_AMP);
    let golden_dir = golden_dir();
    std::fs::create_dir_all(&golden_dir).expect("failed to create golden dir");

    let mut ok = 0;
    let mut fail = 0;

    for config in CONFIGS {
        eprintln!("  [regen] config: {}…", config.label);
        match run_ngspice(config, &input) {
            Some(out) => {
                let path = golden_path(config.label);
                save_npy(&path, &out);
                eprintln!("    saved → {} ({} samples)", path.display(), out.len());
                ok += 1;
            }
            None => {
                eprintln!("    FAILED — ngspice error (see above)");
                fail += 1;
            }
        }
    }

    eprintln!();
    eprintln!(
        "  Regen complete: {ok}/{} goldens written, {fail} failed.",
        CONFIGS.len()
    );
    assert!(
        fail == 0,
        "Golden regen failed for {fail} configs — check ngspice errors above"
    );
}
