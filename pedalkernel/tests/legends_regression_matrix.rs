mod regression_harness;

use regression_harness::{
    load_pro_pedal_sub, run_circuit_matrix, run_circuit_matrix_named, CircuitRegressionSpec,
    CompileVariant, ControlCheck, STANDARD_VARIANTS,
};

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

/// Load all four legend pedals at runtime.  Returns `None` when the pro repo
/// is absent (public CI), allowing callers to skip.
fn load_legends() -> Option<Vec<CircuitRegressionSpec>> {
    let goldenrod = load_pro_pedal_sub("pedals/legends/goldenrod.pedal")?;
    let ratking = load_pro_pedal_sub("pedals/legends/ratking_non_invert_v1a.pedal")?;
    let screamer = load_pro_pedal_sub("pedals/legends/screamer.pedal")?;
    let sd1 = load_pro_pedal_sub("pedals/legends/sd1.pedal")?;

    Some(vec![
        CircuitRegressionSpec {
            name: "Goldenrod",
            source: goldenrod,
            checks: GOLDENROD_CHECKS,
        },
        CircuitRegressionSpec {
            name: "Ratking",
            source: ratking,
            checks: RATKING_CHECKS,
        },
        CircuitRegressionSpec {
            name: "Screamer",
            source: screamer,
            checks: SCREAMER_CHECKS,
        },
        CircuitRegressionSpec {
            name: "SD-1",
            source: sd1,
            checks: SD1_CHECKS,
        },
    ])
}

#[test]
fn legends_audio_regression_matrix() {
    let legends = match load_legends() {
        Some(v) => v,
        None => {
            eprintln!("  SKIP: legends not found (pro repo not present)");
            return;
        }
    };
    run_circuit_matrix(&legends, STANDARD_VARIANTS);
}

#[test]
fn legends_stage_impl_differential_smoke() {
    let legends = match load_legends() {
        Some(v) => v,
        None => {
            eprintln!("  SKIP: legends not found (pro repo not present)");
            return;
        }
    };
    const VARIANTS: &[CompileVariant] = &[
        CompileVariant::Default,
        CompileVariant::DisableIir,
        CompileVariant::SkipBlockwise,
    ];
    run_circuit_matrix_named("audio_matrix_stage_differential", &legends, VARIANTS);
}
