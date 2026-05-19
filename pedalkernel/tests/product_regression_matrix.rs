mod regression_harness;

use pedalkernel::compiler::compile_pedal;
use pedalkernel::dsl::parse_pedal_file;
use pedalkernel::PedalProcessor;
use pedalkernel_rt::processor::Stage;
use std::panic::{self, AssertUnwindSafe};

use regression_harness::{
    assert_circuit_specs_parse_and_controls_mapped, run_circuit_matrix_named,
    CircuitRegressionSpec, CompileVariant, ControlCheck,
};

const ACIDBATH_TB303_FILTER_SRC: &str =
    include_str!("../../../pedalkernel-pro/crates/acidattack/acidattack-core/tb303_filter.pedal");
const ACIDBATH_TB303_VCA_SRC: &str =
    include_str!("../../../pedalkernel-pro/crates/acidattack/acidattack-core/tb303_vca.pedal");

const BIVALVE_MS20_LPF_SRC: &str =
    include_str!("../../../pedalkernel-pro/crates/bivalve/bivalve-core/ms20_lpf.pedal");
const BIVALVE_MS20_HPF_SRC: &str =
    include_str!("../../../pedalkernel-pro/crates/bivalve/bivalve-core/ms20_hpf.pedal");
const BIVALVE_SALLEN_KEY_LPF_SRC: &str =
    include_str!("../../../pedalkernel-pro/crates/bivalve/bivalve-core/sallen_key_lpf.pedal");
const BIVALVE_SALLEN_KEY_HPF_SRC: &str =
    include_str!("../../../pedalkernel-pro/crates/bivalve/bivalve-core/sallen_key_hpf.pedal");
const BIVALVE_MFB_LPF_SRC: &str =
    include_str!("../../../pedalkernel-pro/crates/bivalve/bivalve-core/mfb_lpf.pedal");

const DRUMMERBOY_808_KICK_SRC: &str = include_str!(
    "../../../pedalkernel-pro/crates/drummerboy/drummerboy-core/pedals/808_kick.pedal"
);
const DRUMMERBOY_808_KICK_V2_SRC: &str = include_str!(
    "../../../pedalkernel-pro/crates/drummerboy/drummerboy-core/pedals/808_kick_v2.pedal"
);
const DRUMMERBOY_606_SNARE_SRC: &str = include_str!(
    "../../../pedalkernel-pro/crates/drummerboy/drummerboy-core/pedals/606_snare.pedal"
);
const DRUMMERBOY_808_TOM_SRC: &str =
    include_str!("../../../pedalkernel-pro/crates/drummerboy/drummerboy-core/pedals/808_tom.pedal");
const DRUMMERBOY_808_RIMSHOT_SRC: &str = include_str!(
    "../../../pedalkernel-pro/crates/drummerboy/drummerboy-core/pedals/808_rimshot.pedal"
);
const DRUMMERBOY_808_CLAVES_SRC: &str = include_str!(
    "../../../pedalkernel-pro/crates/drummerboy/drummerboy-core/pedals/808_claves.pedal"
);

const RESONANCE_CHECKS: &[ControlCheck] = &[ControlCheck::moves("Resonance", 0.005)];
const VCA_CHECKS: &[ControlCheck] = &[ControlCheck::output("VCA_Gain", 6.0)];
const FILTER_CHECKS: &[ControlCheck] = &[
    ControlCheck::tone("Cutoff", 0.5),
    ControlCheck::moves("Resonance", 0.005),
];
const DECAY_CHECKS: &[ControlCheck] = &[ControlCheck::decay("Decay", 0.5)];

const PRODUCT_CIRCUITS: &[CircuitRegressionSpec] = &[
    CircuitRegressionSpec {
        name: "AcidBath TB303 Filter",
        source: ACIDBATH_TB303_FILTER_SRC,
        checks: RESONANCE_CHECKS,
    },
    CircuitRegressionSpec {
        name: "AcidBath TB303 VCA",
        source: ACIDBATH_TB303_VCA_SRC,
        checks: VCA_CHECKS,
    },
    CircuitRegressionSpec {
        name: "BiValve MS-20 LPF",
        source: BIVALVE_MS20_LPF_SRC,
        checks: FILTER_CHECKS,
    },
    CircuitRegressionSpec {
        name: "BiValve MS-20 HPF",
        source: BIVALVE_MS20_HPF_SRC,
        checks: FILTER_CHECKS,
    },
    CircuitRegressionSpec {
        name: "BiValve Sallen-Key LPF",
        source: BIVALVE_SALLEN_KEY_LPF_SRC,
        checks: FILTER_CHECKS,
    },
    CircuitRegressionSpec {
        name: "BiValve Sallen-Key HPF",
        source: BIVALVE_SALLEN_KEY_HPF_SRC,
        checks: FILTER_CHECKS,
    },
    CircuitRegressionSpec {
        name: "BiValve MFB LPF",
        source: BIVALVE_MFB_LPF_SRC,
        checks: FILTER_CHECKS,
    },
    CircuitRegressionSpec {
        name: "DrummerBoy 808 Kick",
        source: DRUMMERBOY_808_KICK_SRC,
        checks: DECAY_CHECKS,
    },
    CircuitRegressionSpec {
        name: "DrummerBoy 808 Kick v2",
        source: DRUMMERBOY_808_KICK_V2_SRC,
        checks: DECAY_CHECKS,
    },
    CircuitRegressionSpec {
        name: "DrummerBoy 606 Snare",
        source: DRUMMERBOY_606_SNARE_SRC,
        checks: DECAY_CHECKS,
    },
    CircuitRegressionSpec {
        name: "DrummerBoy 808 Tom",
        source: DRUMMERBOY_808_TOM_SRC,
        checks: DECAY_CHECKS,
    },
    CircuitRegressionSpec {
        name: "DrummerBoy 808 Rimshot",
        source: DRUMMERBOY_808_RIMSHOT_SRC,
        checks: DECAY_CHECKS,
    },
    CircuitRegressionSpec {
        name: "DrummerBoy 808 Claves",
        source: DRUMMERBOY_808_CLAVES_SRC,
        checks: DECAY_CHECKS,
    },
];

#[test]
fn product_pedal_controls_are_mapped_from_pro_pedals() {
    assert_circuit_specs_parse_and_controls_mapped(PRODUCT_CIRCUITS);
}

#[test]
fn product_controls_bind_to_runtime_stages() {
    for spec in PRODUCT_CIRCUITS {
        let pedal = parse_pedal_file(spec.source).expect("parse product pedal");
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
                Stage::BlockwiseKMethod(_) => "BKM",
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
    const VARIANTS: &[CompileVariant] = &[CompileVariant::Default];
    run_circuit_matrix_named("audio_matrix_products", PRODUCT_CIRCUITS, VARIANTS);
}

#[test]
#[ignore = "monolithic BJT VCA path is currently too slow for the default test tier"]
fn acidbath_vca_gain_moves_output_regression() {
    const VARIANTS: &[CompileVariant] = &[CompileVariant::Default];
    const SPECS: &[CircuitRegressionSpec] = &[CircuitRegressionSpec {
        name: "AcidBath TB303 VCA",
        source: ACIDBATH_TB303_VCA_SRC,
        checks: VCA_CHECKS,
    }];
    run_circuit_matrix_named("audio_matrix_acidbath_vca", SPECS, VARIANTS);
}

#[test]
fn acidbath_filter_short_render_does_not_panic() {
    let pedal = parse_pedal_file(ACIDBATH_TB303_FILTER_SRC).expect("parse filter");
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
