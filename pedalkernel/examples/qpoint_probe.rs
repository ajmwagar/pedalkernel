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
            Ok(compiled) => stage_map(&compiled),
            Err(e) => eprintln!("  compile error: {e}"),
        }
    }
}

/// `PK_BIAS_QPOINT_DEBUG=1`: dump where every tube/pentode landed at runtime —
/// the stage container + the per-instance bias values the compiler actually
/// stamped. This is the ko5g.5 gate-3 "after" column source for devices whose
/// bias is a runtime field (WDF `PentodeRoot::vg1k_bias` / `vg2k`) rather than
/// a solver print.
fn stage_map(compiled: &pedalkernel_rt::processor::CompiledPedal) {
    if std::env::var("PK_BIAS_QPOINT_DEBUG").is_err() {
        return;
    }
    for (i, stage) in compiled.stages.iter().enumerate() {
        print_stage(&format!("stage {i}"), stage);
    }
}

fn print_stage(tag: &str, stage: &pedalkernel_rt::processor::Stage) {
    use pedalkernel_rt::processor::Stage;
    use pedalkernel_rt::stage::{NlDeviceGroupKind, NlDeviceKind, RootKind};
    {
        let i = tag;
        match stage {
            Stage::Wdf(w) => match &w.root {
                RootKind::Pentode(p) => eprintln!(
                    "[stage-map] {i} Wdf root=Pentode vg1k_bias={:.6} vg2k={:.6} v_max={:.1}",
                    p.vg1k_bias(),
                    p.vg2k(),
                    p.v_max()
                ),
                RootKind::Triode(t) => eprintln!(
                    "[stage-map] {i} Wdf root=Triode vgk_bias={:.6} v_max={:.1}",
                    t.vgk_bias(),
                    t.v_max()
                ),
                other => eprintln!("[stage-map] {i} Wdf root={}", other.kind_name()),
            },
            Stage::MultiNl(m) => {
                let devices: Vec<String> = m
                    .nl_devices
                    .iter()
                    .map(|d| match d {
                        NlDeviceKind::Pentode(p) => format!(
                            "Pentode(vg1k_bias={:.6} vg2k={:.6})",
                            p.vg1k_bias(),
                            p.vg2k()
                        ),
                        other => other.debug_name().to_owned(),
                    })
                    .collect();
                let groups: Vec<String> = m
                    .device_groups
                    .as_ref()
                    .map(|g| {
                        g.groups
                            .iter()
                            .map(|k| match k {
                                NlDeviceGroupKind::PentodeThreePort(p) => format!(
                                    "PentodeThreePort(vg2k={:.6} v_max={:.1})",
                                    p.vg2k(),
                                    p.v_max()
                                ),
                                NlDeviceGroupKind::TriodeThreePort(_) => {
                                    "TriodeThreePort".to_owned()
                                }
                                NlDeviceGroupKind::VariMuThreePort(_) => {
                                    "VariMuThreePort".to_owned()
                                }
                                NlDeviceGroupKind::BjtTwoPort(_) => "BjtTwoPort".to_owned(),
                                NlDeviceGroupKind::EbersMollTwoPort(_) => {
                                    "EbersMollTwoPort".to_owned()
                                }
                                NlDeviceGroupKind::SinglePort(d) => {
                                    format!("SinglePort({})", d.debug_name())
                                }
                            })
                            .collect()
                    })
                    .unwrap_or_default();
                eprintln!(
                    "[stage-map] {i} MultiNl devices=[{}] groups=[{}]",
                    devices.join(", "),
                    groups.join(", ")
                );
            }
            Stage::Blockwise(b) => {
                eprintln!(
                    "[stage-map] {i} Blockwise sub_stages={}",
                    b.sub_stages.len()
                );
                for (j, sub) in b.sub_stages.iter().enumerate() {
                    print_stage(&format!("{i}.sub{j}"), sub);
                }
            }
            Stage::Iir(_) => eprintln!("[stage-map] {i} Iir"),
            Stage::StateSpace(_) => eprintln!("[stage-map] {i} StateSpace"),
            other => eprintln!("[stage-map] {i} {other:?}"),
        }
    }
}
