mod regression_harness;

use pedalkernel::compiler::compile_pedal;
use pedalkernel::dsl::parse_pedal_file;
use pedalkernel::PedalProcessor;
use pedalkernel_rt::processor::Stage;
use std::panic::{self, AssertUnwindSafe};

use regression_harness::{
    assert_circuit_specs_parse_and_controls_mapped, load_pro_pedal_sub, run_circuit_matrix_named,
    CircuitRegressionSpec, CompileVariant, ControlCheck,
};

const RESONANCE_CHECKS: &[ControlCheck] = &[ControlCheck::moves("Resonance", 0.005)];
const VCA_CHECKS: &[ControlCheck] = &[ControlCheck::output("VCA_Gain", 6.0)];
const FILTER_CHECKS: &[ControlCheck] = &[
    ControlCheck::tone("Cutoff", 0.5),
    ControlCheck::moves("Resonance", 0.005),
];
const DECAY_CHECKS: &[ControlCheck] = &[ControlCheck::decay("Decay", 0.5)];

/// Load all product circuits from the sibling pro repo at runtime.
/// Returns `None` when the pro repo is absent (public CI).
fn load_product_circuits() -> Option<Vec<CircuitRegressionSpec>> {
    let tb303_filter = load_pro_pedal_sub("crates/acidattack/acidattack-core/tb303_filter.pedal")?;
    let tb303_vca = load_pro_pedal_sub("crates/acidattack/acidattack-core/tb303_vca.pedal")?;
    let ms20_lpf = load_pro_pedal_sub("crates/bivalve/bivalve-core/ms20_lpf.pedal")?;
    let ms20_hpf = load_pro_pedal_sub("crates/bivalve/bivalve-core/ms20_hpf.pedal")?;
    let sallen_key_lpf = load_pro_pedal_sub("crates/bivalve/bivalve-core/sallen_key_lpf.pedal")?;
    let sallen_key_hpf = load_pro_pedal_sub("crates/bivalve/bivalve-core/sallen_key_hpf.pedal")?;
    let mfb_lpf = load_pro_pedal_sub("crates/bivalve/bivalve-core/mfb_lpf.pedal")?;
    let kick_808 = load_pro_pedal_sub("crates/drummerboy/drummerboy-core/pedals/808_kick.pedal")?;
    let snare_606 = load_pro_pedal_sub("crates/drummerboy/drummerboy-core/pedals/606_snare.pedal")?;
    let tom_808 = load_pro_pedal_sub("crates/drummerboy/drummerboy-core/pedals/808_tom.pedal")?;
    let rimshot_808 =
        load_pro_pedal_sub("crates/drummerboy/drummerboy-core/pedals/808_rimshot.pedal")?;
    let claves_808 =
        load_pro_pedal_sub("crates/drummerboy/drummerboy-core/pedals/808_claves.pedal")?;

    Some(vec![
        CircuitRegressionSpec {
            name: "AcidBath TB303 Filter",
            source: tb303_filter,
            checks: RESONANCE_CHECKS,
        },
        CircuitRegressionSpec {
            name: "AcidBath TB303 VCA",
            source: tb303_vca,
            checks: VCA_CHECKS,
        },
        CircuitRegressionSpec {
            name: "BiValve MS-20 LPF",
            source: ms20_lpf,
            checks: FILTER_CHECKS,
        },
        CircuitRegressionSpec {
            name: "BiValve MS-20 HPF",
            source: ms20_hpf,
            checks: FILTER_CHECKS,
        },
        CircuitRegressionSpec {
            name: "BiValve Sallen-Key LPF",
            source: sallen_key_lpf,
            checks: FILTER_CHECKS,
        },
        CircuitRegressionSpec {
            name: "BiValve Sallen-Key HPF",
            source: sallen_key_hpf,
            checks: FILTER_CHECKS,
        },
        CircuitRegressionSpec {
            name: "BiValve MFB LPF",
            source: mfb_lpf,
            checks: FILTER_CHECKS,
        },
        CircuitRegressionSpec {
            name: "DrummerBoy 808 Kick",
            source: kick_808,
            checks: DECAY_CHECKS,
        },
        CircuitRegressionSpec {
            name: "DrummerBoy 606 Snare",
            source: snare_606,
            checks: DECAY_CHECKS,
        },
        CircuitRegressionSpec {
            name: "DrummerBoy 808 Tom",
            source: tom_808,
            checks: DECAY_CHECKS,
        },
        CircuitRegressionSpec {
            name: "DrummerBoy 808 Rimshot",
            source: rimshot_808,
            checks: DECAY_CHECKS,
        },
        CircuitRegressionSpec {
            name: "DrummerBoy 808 Claves",
            source: claves_808,
            checks: DECAY_CHECKS,
        },
    ])
}

#[test]
fn product_pedal_controls_are_mapped_from_pro_pedals() {
    let circuits = match load_product_circuits() {
        Some(v) => v,
        None => {
            eprintln!("  SKIP: product circuits not found (pro repo not present)");
            return;
        }
    };
    assert_circuit_specs_parse_and_controls_mapped(&circuits);
}

#[test]
fn product_controls_bind_to_runtime_stages() {
    let circuits = match load_product_circuits() {
        Some(v) => v,
        None => {
            eprintln!("  SKIP: product circuits not found (pro repo not present)");
            return;
        }
    };
    for spec in &circuits {
        let pedal = parse_pedal_file(&spec.source).expect("parse product pedal");
        let compiled = compile_pedal(&pedal, 48_000.0).expect("compile product pedal");
        let control_labels = compiled
            .controls
            .iter()
            .map(|control| {
                (
                    control.label.as_str(),
                    control.component_id.as_str(),
                    format!("{:?}", control.target),
                )
            })
            .collect::<Vec<_>>();
        let stage_kinds = compiled
            .stages
            .iter()
            .map(|stage| match stage {
                Stage::Wdf(_) => "WDF",
                Stage::MultiNl(_) => "MultiNL",
                Stage::Iir(_) => "IIR",
                Stage::StateSpace(_) => "StateSpace",
                Stage::BlackFeedback(_) => "BlackFeedback",
                Stage::Blockwise(_) => "BKM",
                Stage::KMethod { .. } => "KMethod",
                Stage::SerialDelayedFeedback(_) => "SerialDelayedFeedback",
            })
            .collect::<Vec<_>>();

        eprintln!(
            "{}: stages={stage_kinds:?} controls={control_labels:?}",
            spec.name
        );

        for check in spec.checks {
            assert!(
                compiled
                    .controls
                    .iter()
                    .any(|control| control.label == check.label),
                "{}: control {:?} did not bind to runtime",
                spec.name,
                check.label
            );
        }
    }
}

#[test]
#[ignore = "captures current AcidBath/BiValve/DrummerBoy audio behavior gaps; run explicitly while fixing them"]
fn product_audio_regression_matrix_captures_known_gaps() {
    let circuits = match load_product_circuits() {
        Some(v) => v,
        None => {
            eprintln!("  SKIP: product circuits not found (pro repo not present)");
            return;
        }
    };
    const VARIANTS: &[CompileVariant] = &[CompileVariant::Default];
    run_circuit_matrix_named("audio_matrix_products", &circuits, VARIANTS);
}

#[test]
#[ignore = "monolithic BJT VCA path is currently too slow for the default test tier"]
fn acidbath_vca_gain_moves_output_regression() {
    let tb303_vca = match load_pro_pedal_sub("crates/acidattack/acidattack-core/tb303_vca.pedal") {
        Some(s) => s,
        None => {
            eprintln!("  SKIP: tb303_vca.pedal not found (pro repo not present)");
            return;
        }
    };
    const VARIANTS: &[CompileVariant] = &[CompileVariant::Default];
    let specs = vec![CircuitRegressionSpec {
        name: "AcidBath TB303 VCA",
        source: tb303_vca,
        checks: VCA_CHECKS,
    }];
    run_circuit_matrix_named("audio_matrix_acidbath_vca", &specs, VARIANTS);
}

#[test]
fn acidbath_filter_short_render_does_not_panic() {
    let src = match load_pro_pedal_sub("crates/acidattack/acidattack-core/tb303_filter.pedal") {
        Some(s) => s,
        None => {
            eprintln!("  SKIP: tb303_filter.pedal not found (pro repo not present)");
            return;
        }
    };
    let pedal = parse_pedal_file(&src).expect("parse filter");
    let mut proc = compile_pedal(&pedal, 48_000.0).expect("compile filter");
    proc.set_control("Resonance", 0.75);

    let result = panic::catch_unwind(AssertUnwindSafe(|| {
        let mut energy = 0.0;
        for n in 0..1024 {
            let input = 0.05 * (2.0 * std::f64::consts::PI * 440.0 * n as f64 / 48_000.0).sin();
            let y = proc.process(input);
            assert!(y.is_finite(), "filter output must remain finite");
            energy += y * y;
        }
        energy
    }));

    let energy = result.expect("filter render must not panic");
    assert!(energy > 0.0, "filter should produce nonzero output");
}
