mod regression_harness;

use regression_harness::{
    run_circuit_matrix, run_circuit_matrix_named, CircuitRegressionSpec, CompileVariant,
    ControlCheck, STANDARD_VARIANTS,
};

const GOLDENROD_SRC: &str = include_str!("../../../pedalkernel-pro/pedals/legends/goldenrod.pedal");
const RATKING_SRC: &str =
    include_str!("../../../pedalkernel-pro/pedals/legends/ratking_non_invert_v1a.pedal");
const SCREAMER_SRC: &str = include_str!("../../../pedalkernel-pro/pedals/legends/screamer.pedal");
const SD1_SRC: &str = include_str!("../../../pedalkernel-pro/pedals/legends/sd1.pedal");

const GOLDENROD_CHECKS: &[ControlCheck] = &[
    ControlCheck::drive("Gain", 0.0),
    ControlCheck::tone("Treble", 0.75),
    ControlCheck::output("Output", 8.0),
];

const RATKING_CHECKS: &[ControlCheck] = &[
    ControlCheck::dirtier_drive("Distortion", 3.0),
    ControlCheck::tone("Filter", 2.0),
    ControlCheck::output("Volume", 8.0),
];

const SCREAMER_CHECKS: &[ControlCheck] = &[
    ControlCheck::drive("Drive", 2.0),
    ControlCheck::tone("Tone", 0.25),
    ControlCheck::output("Level", 8.0),
];

const SD1_CHECKS: &[ControlCheck] = &[
    ControlCheck::drive("Drive", 2.0),
    ControlCheck::tone("Tone", 0.25),
    ControlCheck::output("Level", 8.0),
];

const LEGENDS: &[CircuitRegressionSpec] = &[
    CircuitRegressionSpec {
        name: "Goldenrod",
        source: GOLDENROD_SRC,
        checks: GOLDENROD_CHECKS,
    },
    CircuitRegressionSpec {
        name: "Ratking",
        source: RATKING_SRC,
        checks: RATKING_CHECKS,
    },
    CircuitRegressionSpec {
        name: "Screamer",
        source: SCREAMER_SRC,
        checks: SCREAMER_CHECKS,
    },
    CircuitRegressionSpec {
        name: "SD-1",
        source: SD1_SRC,
        checks: SD1_CHECKS,
    },
];

#[test]
fn legends_audio_regression_matrix() {
    run_circuit_matrix(LEGENDS, STANDARD_VARIANTS);
}

#[test]
fn legends_stage_impl_differential_smoke() {
    const VARIANTS: &[CompileVariant] = &[
        CompileVariant::Default,
        CompileVariant::DisableIir,
        CompileVariant::SkipBlockwise,
    ];
    run_circuit_matrix_named("audio_matrix_stage_differential", LEGENDS, VARIANTS);
}
