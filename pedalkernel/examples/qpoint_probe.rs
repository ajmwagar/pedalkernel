//! Compile `.pedal` fixtures and dump every triode DC Q-point solve.
//!
//! Run with `PK_BIAS_QPOINT_DEBUG=1` so the compiler's bias solvers print
//! their per-instance `[bias-qpoint]` lines (solved Vgk/Vpk/Vcathode/Ia, or
//! UNDETERMINABLE) to stderr:
//!
//! ```sh
//! PK_BIAS_QPOINT_DEBUG=1 cargo run -p pedalkernel --example qpoint_probe -- \
//!     pedalkernel-validate/circuits/nonlinear/common_cathode_12ax7.pedal
//! ```
//!
//! This is the refactor-gating harness for the ko5g bias unification beads:
//! run it on the base commit and on the refactor branch over the audited
//! fixture list, then diff the tables — a pure refactor must produce
//! byte-identical solved values (pedalkernel-ko5g.3 gate 2).
//!
//! Uses `CompileOptions::debug()` (skips K-table generation) — the Q-point
//! solve happens during stage building, well before K-tables.

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.is_empty() {
        eprintln!("usage: qpoint_probe <file.pedal> [...]");
        std::process::exit(2);
    }
    if std::env::var("PK_BIAS_QPOINT_DEBUG").is_err() {
        eprintln!("note: PK_BIAS_QPOINT_DEBUG is not set — no [bias-qpoint] lines will print");
    }
    for path in &args {
        eprintln!("=== {path}");
        let source = match std::fs::read_to_string(path) {
            Ok(s) => s,
            Err(e) => {
                eprintln!("  read error: {e}");
                continue;
            }
        };
        let mut pedal = match pedalkernel::dsl::parse_pedal_file(&source) {
            Ok(p) => p,
            Err(e) => {
                eprintln!("  parse error: {e}");
                continue;
            }
        };
        if !pedal.uses.is_empty() {
            let base_dir = std::path::Path::new(path)
                .parent()
                .unwrap_or_else(|| std::path::Path::new("."));
            pedal = match pedalkernel::dsl_expand::expand_uses(&pedal, base_dir) {
                Ok(p) => p,
                Err(e) => {
                    eprintln!("  subcircuit expansion error: {e}");
                    continue;
                }
            };
        }
        match pedalkernel::compiler::compile_pedal_with_options(
            &pedal,
            48_000.0,
            pedalkernel::compiler::CompileOptions::debug(),
        ) {
            Ok(_) => {}
            Err(e) => eprintln!("  compile error: {e}"),
        }
    }
}
