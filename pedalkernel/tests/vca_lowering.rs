//! VCA lowering acceptance tests (audit gap G4, architecture debt §4).
//!
//! `vca()` was a GraphRole::Virtual stub: no MNA stamp, `CompiledPedal.vcas`
//! never populated, and `EF.out -> VCA.cv` parsed but inert. These tests pin
//! the fix (compiler/vca_lowering.rs):
//!
//! 1. a minimal inline vca circuit compiles with a populated `vcas` instance
//!    and passes audio through the behavioral gap,
//! 2. the runtime VCA's CV 0→1 sweep changes gain monotonically by far more
//!    than 20 dB (SSM2164 dB-linear law: 0..1 spans 0..-80 dB),
//! 3. the sidechain (`EF.out -> VCA.cv`) produces real gain reduction, and
//! 4. the mandatory-lowering compile error fires: a `vca()` whose audio port
//!    cannot be lowered, and any `vco()` (no VCO lowering exists), are
//!    COMPILE ERRORS naming the component — behavioral islands never ship
//!    silent (reports/architecture-debt-2026-06-12.md §4).
//!
//! Run: cargo test -p pedalkernel --no-default-features --test vca_lowering

mod audio_analysis;

use audio_analysis::*;
use pedalkernel::compiler::compile_pedal;
use pedalkernel::dsl::parse_pedal_file;
use pedalkernel::PedalProcessor;
use pedalkernel_rt::processor::ModulationTarget;

/// Minimal VCA-in-the-audio-path fixture: passive divider -> VCA -> passive
/// divider, with a feed-forward envelope follower on the VCA's cv port.
/// Both sides of the VCA's behavioral gap must compile into live stages.
const MINIMAL_VCA: &str = r#"
pedal "Minimal VCA" {
  supply 9V
  components {
    R_in: resistor(10k)
    R_sh1: resistor(100k)
    VCA1: vca(ssm2164)
    R_out: resistor(10k)
    R_sh2: resistor(100k)
    EF1: envelope_follower(10k, 1u, 100k, 1u, 20k)
  }
  nets {
    in -> R_in.a
    R_in.b -> R_sh1.a, VCA1.in
    R_sh1.b -> gnd
    VCA1.out -> R_out.a
    R_out.b -> R_sh2.a, out
    R_sh2.b -> gnd
    EF1.in -> R_in.a
    EF1.out -> VCA1.cv
  }
  controls {}
}
"#;

/// Same audio path but NO envelope on cv: the VCA must still lower (unity
/// gain) so the test can drive the CV port directly.
const MINIMAL_VCA_NO_CV: &str = r#"
pedal "Minimal VCA No CV" {
  supply 9V
  components {
    R_in: resistor(10k)
    R_sh1: resistor(100k)
    VCA1: vca(ssm2164)
    R_out: resistor(10k)
    R_sh2: resistor(100k)
  }
  nets {
    in -> R_in.a
    R_in.b -> R_sh1.a, VCA1.in
    R_sh1.b -> gnd
    VCA1.out -> R_out.a
    R_out.b -> R_sh2.a, out
    R_sh2.b -> gnd
  }
  controls {}
}
"#;

// ===========================================================================
// (1) Lowering populates vcas and the gap passes audio
// ===========================================================================

#[test]
fn minimal_vca_circuit_lowers_and_passes_audio() {
    let pedal = parse_pedal_file(MINIMAL_VCA).expect("parse");
    let mut compiled = compile_pedal(&pedal, SAMPLE_RATE).expect("compile");

    // Lowering contract: one runtime VCA per vca() component, declaration
    // order, CV-driven mode (no trigger).
    assert_eq!(compiled.vcas.len(), 1, "vcas not populated by lowering");
    assert_eq!(compiled.vcas[0].comp_id, "VCA1");
    assert!(compiled.vcas[0].trigger_idx.is_none());

    // Audio must cross the behavioral gap (historically a stage spanning the
    // gap probed exact 0 and the pedal was silent).
    let input = sine_at(1_000.0, 0.1, 1.0, SAMPLE_RATE);
    let output: Vec<f64> = input.iter().map(|&s| compiled.process(s)).collect();
    assert!(output.iter().all(|x| x.is_finite()), "NaN/inf in output");
    let steady_peak = peak(&output[(0.5 * SAMPLE_RATE) as usize..]);
    eprintln!("minimal vca steady-state peak: {steady_peak:.6}");
    assert!(
        steady_peak > 0.005,
        "VCA circuit is silent across the behavioral gap: peak {steady_peak:.6}"
    );
}

#[test]
fn vca_cv_envelope_binding_exists() {
    let pedal = parse_pedal_file(MINIMAL_VCA).expect("parse");
    let compiled = compile_pedal(&pedal, SAMPLE_RATE).expect("compile");

    let vca_cv_bindings = compiled
        .envelopes
        .iter()
        .filter(|b| matches!(b.target, ModulationTarget::VcaCv { vca_idx: 0 }))
        .count();
    assert_eq!(
        vca_cv_bindings, 1,
        "expected exactly one EF -> VCA1.cv envelope binding"
    );
}

// ===========================================================================
// (2) CV sweep: dB-linear law, monotonic, > 20 dB span
// ===========================================================================

/// Sweep the runtime element directly: gain must fall monotonically with CV
/// and span well over 20 dB (the SSM2164 law covers 80 dB over cv 0..1).
#[test]
fn vca_element_cv_sweep_is_monotonic_and_spans_over_20_db() {
    let mut vca = pedalkernel::elements::Vca::new();
    let mut prev_gain = f64::INFINITY;
    let mut gains_db = Vec::new();
    for i in 0..=10 {
        let cv = i as f64 / 10.0;
        vca.set_cv_normalized(cv);
        let gain = vca.gain();
        gains_db.push(lin_to_db(gain));
        assert!(
            gain < prev_gain,
            "VCA gain not strictly decreasing at cv={cv}: {gain} >= {prev_gain}"
        );
        prev_gain = gain;
    }
    let span = gains_db.first().unwrap() - gains_db.last().unwrap();
    eprintln!("vca element cv sweep: {gains_db:.1?} dB, span {span:.1} dB");
    assert!(
        (gains_db[0]).abs() < 1e-9,
        "cv=0 must be unity gain, got {:.3} dB",
        gains_db[0]
    );
    assert!(
        span > 20.0,
        "cv 0->1 gain span {span:.1} dB, expected > 20 dB"
    );
    assert!(
        (span - 80.0).abs() < 1e-6,
        "cv 0->1 should span the documented 80 dB, got {span:.3} dB"
    );
}

/// Same sweep through the COMPILED circuit: drive the lowered instance's CV
/// port between runs and measure steady-state throughput.
#[test]
fn compiled_vca_cv_sweep_changes_circuit_gain_monotonically() {
    let gain_at_cv = |cv: f64| -> f64 {
        let pedal = parse_pedal_file(MINIMAL_VCA_NO_CV).expect("parse");
        let mut compiled = compile_pedal(&pedal, SAMPLE_RATE).expect("compile");
        assert_eq!(compiled.vcas.len(), 1);
        compiled.vcas[0].vca.set_cv_normalized(cv);
        let input = sine_at(1_000.0, 0.05, 1.0, SAMPLE_RATE);
        let output: Vec<f64> = input.iter().map(|&s| compiled.process(s)).collect();
        let tail = (0.5 * SAMPLE_RATE) as usize;
        lin_to_db(rms(&output[tail..]) / rms(&input[tail..]))
    };

    let cvs = [0.0, 0.25, 0.5, 0.75, 1.0];
    let gains: Vec<f64> = cvs.iter().map(|&cv| gain_at_cv(cv)).collect();
    eprintln!("compiled vca cv sweep: cv {cvs:?} -> gain {gains:.2?} dB");

    for w in gains.windows(2) {
        assert!(
            w[1] < w[0] - 1.0,
            "circuit gain not monotonically decreasing with CV: {gains:?}"
        );
    }
    let span = gains[0] - gains[gains.len() - 1];
    assert!(
        span > 20.0,
        "CV 0->1 changed circuit gain by {span:.1} dB, expected > 20 dB"
    );
}

// ===========================================================================
// (3) Sidechain compresses (loud vs quiet program)
// ===========================================================================

#[test]
fn minimal_vca_envelope_sidechain_reduces_gain_when_loud() {
    let steady_gain = |amplitude: f64| -> f64 {
        let mut process = pedal_processor(MINIMAL_VCA, SAMPLE_RATE, &[]);
        let input = sine_at(1_000.0, amplitude, 2.0, SAMPLE_RATE);
        let output: Vec<f64> = input.iter().map(|&x| process(x)).collect();
        let tail = (1.5 * SAMPLE_RATE) as usize;
        lin_to_db(rms(&output[tail..]) / rms(&input[tail..]))
    };

    let quiet = steady_gain(0.01);
    let loud = steady_gain(1.0);
    let gr = quiet - loud;
    eprintln!("minimal vca sidechain: quiet {quiet:+.3} dB, loud {loud:+.3} dB, GR {gr:.3} dB");
    assert!(quiet.is_finite() && loud.is_finite());
    assert!(
        gr > 1.0,
        "EF -> VCA1.cv must reduce gain on loud program: GR {gr:.3} dB, expected > 1 dB"
    );
}

// ===========================================================================
// (4) Mandatory-lowering compile errors (never ship silent)
// ===========================================================================

/// A vca() whose audio port is not wired cannot be lowered into the audio
/// path — compilation must FAIL naming the component, not ship a silently
/// bypassed processor.
#[test]
fn unwired_vca_is_a_compile_error_naming_the_component() {
    let src = r#"
pedal "Dangling VCA" {
  supply 9V
  components {
    R1: resistor(10k)
    R2: resistor(100k)
    VCA9: vca(ssm2164)
  }
  nets {
    in -> R1.a
    R1.b -> R2.a, out
    R2.b -> gnd
  }
  controls {}
}
"#;
    let pedal = parse_pedal_file(src).expect("parse");
    let err = match compile_pedal(&pedal, SAMPLE_RATE) {
        Err(e) => e,
        Ok(_) => {
            panic!("a vca() that lowering cannot place in the audio path must be a compile error")
        }
    };
    eprintln!("unwired vca error: {err}");
    assert!(
        err.contains("VCA9"),
        "compile error must name the component: {err}"
    );
    assert!(
        err.contains("behavioral island"),
        "compile error should explain the mandatory-lowering contract: {err}"
    );
}

/// A vco() whose wave output (saw/tri/pulse) is NOT wired into the netlist
/// cannot be placed as a source in the audio path — it would silently produce
/// nothing. The VcoBlock's `bind_runtime` rejects it via the same
/// mandatory-lowering mechanism (architecture debt §4), naming the component.
/// (A *wired* vco() now lowers — see tests/vco_lowering.rs.)
#[test]
fn unlowered_vco_is_a_compile_error_naming_the_component() {
    let src = r#"
pedal "Bare VCO" {
  supply 12V
  components {
    VCO1: vco(as3340)
    R_out: resistor(1k)
  }
  nets {
    in -> R_out.a
    R_out.b -> out
  }
  controls {}
}
"#;
    let pedal = parse_pedal_file(src).expect("parse");
    let err = match compile_pedal(&pedal, SAMPLE_RATE) {
        Err(e) => e,
        Ok(_) => {
            panic!("a vco() with no wired wave output must be a compile error, not silence")
        }
    };
    eprintln!("unlowered vco error: {err}");
    assert!(
        err.contains("VCO1"),
        "compile error must name the component: {err}"
    );
    assert!(
        err.contains("behavioral island"),
        "compile error should explain the mandatory-lowering contract: {err}"
    );
}
