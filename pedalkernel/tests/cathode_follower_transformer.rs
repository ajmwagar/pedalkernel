//! Mask-7 regression: a cathode follower driving an output transformer.
//!
//! RCA (`reports/signal-routing-formation-layer-2026-06-13.md` §1): a
//! low-output-impedance source (a 12BH7 cathode follower) feeding a downstream
//! output transformer collapses to numerical zero, while a high-impedance
//! (plate-driven) source into the SAME transformer is healthy. The difference
//! is ONLY the source impedance.
//!
//! Root cause is in group FORMATION (`compiler/signal_flow.rs`): for a cathode
//! follower the active device's OUTPUT node IS its cathode bias node, so
//! `claim_passive_edges` swallows the FORWARD-SIGNAL branch leaving that node
//! (coupling cap -> transformer -> load) into the device's nonlinear group.
//! The MultiNL builder then discards the transformer -> the realized stage's
//! `output_port` reads a dead node -> silence. The plate-driven topology
//! escapes only because the plate output node != the cathode bias node, so the
//! forward branch splits cleanly into its own `PassiveRType` transformer stage.
//!
//! This file isolates mask 7 from the over-fusion mask (mask 8): there is NO
//! feedback / detector tap here, just a forward chain.
//!
//! Run (matches the RCA reproduction conditions):
//!   cargo test -p pedalkernel --no-default-features \
//!     --test cathode_follower_transformer -- --nocapture

mod audio_analysis;

use audio_analysis::*;

use pedalkernel::compiler::compile_pedal;
use pedalkernel::dsl::parse_pedal_file;
use pedalkernel::PedalProcessor;

const PROBE_HZ: f64 = 1_000.0;

/// 12BH7 cathode follower -> coupling cap -> 4:1 transformer -> 600 Ω load.
/// The cathode node is BOTH the device operating point (R_k3 to gnd) and the
/// forward-signal source (C_c3 -> T_out -> R_load -> out). This is the mask-7
/// trigger: shared output/bias node.
const CF_INTO_TRANSFORMER: &str = r#"
pedal "CF into transformer" {
  supply 250V {
    impedance: 50
    filter_cap: 47u
    rectifier: solid_state
  }
  components {
    C1: cap(100n)
    R_g3: resistor(1M)
    V3: triode(12bh7)
    R_k3: resistor(10k)
    C_c3: cap(10u, electrolytic)
    T_out: transformer(4:1, 10H)
    R_load: resistor(600)
  }
  nets {
    in -> C1.a
    C1.b -> V3.grid, R_g3.a
    R_g3.b -> gnd
    vcc -> V3.plate
    V3.cathode -> R_k3.a, C_c3.a
    R_k3.b -> gnd
    C_c3.b -> T_out.a
    T_out.b -> R_load.a, out
    R_load.b -> gnd
  }
}
"#;

/// Same 12BH7 cathode follower driving the SAME 600 Ω load directly (no
/// transformer). This is healthy regardless of the bug — it is the level
/// reference that proves the CF stage itself works.
const CF_INTO_LOAD: &str = r#"
pedal "CF into load" {
  supply 250V {
    impedance: 50
    filter_cap: 47u
    rectifier: solid_state
  }
  components {
    C1: cap(100n)
    R_g3: resistor(1M)
    V3: triode(12bh7)
    R_k3: resistor(10k)
    C_c3: cap(10u, electrolytic)
    R_load: resistor(600)
  }
  nets {
    in -> C1.a
    C1.b -> V3.grid, R_g3.a
    R_g3.b -> gnd
    vcc -> V3.plate
    V3.cathode -> R_k3.a, C_c3.a
    R_k3.b -> gnd
    C_c3.b -> R_load.a, out
    R_load.b -> gnd
  }
}
"#;

/// Count `[Stage N` entries in the debug dump.
fn stage_count(dump: &str) -> usize {
    dump.matches("[Stage ").count()
}

/// Forward gain at PROBE_HZ in dB.
fn forward_gain_db(src: &str) -> f64 {
    let resp = frequency_response_db(
        || pedal_processor(src, SAMPLE_RATE, &[]),
        &[PROBE_HZ],
        0.05,
        SAMPLE_RATE,
    );
    resp[0].1
}

/// MASK 7 — STRUCTURAL: the cathode follower into a transformer must NOT
/// compile to a single MultiNL stage that drops the transformer. It must
/// realize the transformer/load as its own R-type (PassiveRType) stage.
///
/// RED before the fix: 1 stage, no PassiveRType (transformer swallowed).
#[test]
fn cf_into_transformer_keeps_transformer_as_its_own_stage() {
    let def = parse_pedal_file(CF_INTO_TRANSFORMER).expect("parse CF->xfmr");
    let compiled = compile_pedal(&def, SAMPLE_RATE).expect("compile CF->xfmr");
    let dump = compiled.debug_dump();
    let stages = stage_count(&dump);
    eprintln!("CF->transformer: {stages} stage(s)\n{dump}");

    assert!(
        stages >= 2,
        "CF->transformer must compile to >= 2 stages (NL stage + transformer \
         R-type), got {stages} — the transformer was swallowed into the NL group"
    );
    assert!(
        dump.contains("PassiveRType"),
        "CF->transformer must realize the transformer as a PassiveRType stage; \
         dump had none (transformer absorbed/discarded by the MultiNL builder)"
    );
}

/// MASK 7 — LEVEL: forward gain through the cathode follower into the
/// transformer must be healthy (> -40 dB), not collapsed to the silence floor.
///
/// RED before the fix: ~-220 dB (truly zero). The CF-into-load reference is
/// printed for context (proves the CF stage itself is alive either way).
#[test]
fn cf_into_transformer_passes_signal() {
    let load_gain = forward_gain_db(CF_INTO_LOAD);
    let xfmr_gain = forward_gain_db(CF_INTO_TRANSFORMER);
    eprintln!(
        "mask-7 probe @ {PROBE_HZ} Hz: CF->load {load_gain:+.1} dB (reference), \
         CF->transformer {xfmr_gain:+.1} dB"
    );

    assert!(
        load_gain > -40.0,
        "reference CF->load should be healthy, got {load_gain:+.1} dB — \
         fixture broken, not the bug under test"
    );
    assert!(
        xfmr_gain > -40.0,
        "MASK 7: CF->transformer forward gain {xfmr_gain:+.1} dB collapsed \
         (expected > -40 dB) — transformer swallowed into the CF's NL group"
    );
}

// ── Transformer-core policy gate (task #3) ─────────────────────────────────

/// DC-biased single-ended output transformer carrying explicit JA core data
/// (the OT-DEMO-SE model: IDC=45m, A=1100, full geometry). Under the default
/// `Derived` policy this core is excited past its knee and runs the JA root.
const SE_OT_PEDAL: &str = r#"
pedal "SE OT" {
    supply 9V
    components {
        T1: transformer(26:1, OT-DEMO-SE)
        R_load: resistor(8)
    }
    nets {
        in -> T1.a
        T1.b -> gnd
        T1.c -> out, R_load.a
        T1.d -> gnd
        R_load.b -> gnd
    }
}
"#;

/// Bare coupled-inductor transformer — no model, no core geometry, no JA loop.
/// Declares no core physics, so the `Derived` gate keeps it LINEAR (byte-stable
/// vs `ForceLinear`); `ForceJa` still overrides it to the JA root.
const BARE_XFMR_PEDAL: &str = r#"
pedal "Bare transformer" {
    components {
        T1: transformer(10:1, 2H)
        R_load: resistor(1k)
    }
    nets {
        in -> T1.a
        T1.b -> gnd
        T1.c -> out, R_load.a
        T1.d -> gnd
        R_load.b -> gnd
    }
}
"#;

fn render_policy(src: &str, policy: pedalkernel::compiler::TransformerCorePolicy) -> Vec<f64> {
    use pedalkernel::compiler::{compile_pedal_with_options, CompileOptions};
    let pedal = parse_pedal_file(src).expect("parse");
    let mut opts = CompileOptions::default();
    opts.transformer_core = policy;
    let mut c = compile_pedal_with_options(&pedal, 48_000.0, opts).expect("compile");
    (0..4800)
        .map(|i| {
            let t = i as f64 / 48_000.0;
            c.process(0.1 * (2.0 * std::f64::consts::PI * 220.0 * t).sin())
        })
        .collect()
}

fn max_abs_diff(a: &[f64], b: &[f64]) -> f64 {
    a.iter()
        .zip(b)
        .map(|(x, y)| (x - y).abs())
        .fold(0.0, f64::max)
}

/// `ForceLinear` and `ForceJa` must actually override the derived decision:
/// for a real (excited) core they produce DIFFERENT output.
#[test]
fn force_policies_override_derived_decision() {
    use pedalkernel::compiler::TransformerCorePolicy::*;
    let lin = render_policy(SE_OT_PEDAL, ForceLinear);
    let ja = render_policy(SE_OT_PEDAL, ForceJa);
    let diff = max_abs_diff(&lin, &ja);
    assert!(
        diff > 1e-3,
        "ForceJa must differ from ForceLinear for the excited SE OT, got {diff:.3e}"
    );
}

/// Default `Derived` runs JA for the DC-biased SE OT (explicit knee a=1100,
/// h_peak≈1033 > a·(1-0.2)=880): Derived output matches ForceJa, not ForceLinear.
#[test]
fn derived_runs_ja_for_dc_biased_se_ot() {
    use pedalkernel::compiler::TransformerCorePolicy::*;
    let der = render_policy(
        SE_OT_PEDAL,
        Derived {
            margin: pedalkernel::compiler::DEFAULT_CORE_MARGIN,
        },
    );
    let lin = render_policy(SE_OT_PEDAL, ForceLinear);
    let ja = render_policy(SE_OT_PEDAL, ForceJa);
    assert!(
        max_abs_diff(&der, &ja) < 1e-9,
        "Derived should match ForceJa for the excited SE OT"
    );
    assert!(
        max_abs_diff(&der, &lin) > 1e-3,
        "Derived (JA) should differ from the linear tangent for the excited SE OT"
    );
}

/// A bare transformer declares no core physics: `Derived` keeps it byte-identical
/// to `ForceLinear`, but `ForceJa` can still force the JA root onto it.
#[test]
fn bare_transformer_stays_linear_under_derived() {
    use pedalkernel::compiler::TransformerCorePolicy::*;
    let der = render_policy(
        BARE_XFMR_PEDAL,
        Derived {
            margin: pedalkernel::compiler::DEFAULT_CORE_MARGIN,
        },
    );
    let lin = render_policy(BARE_XFMR_PEDAL, ForceLinear);
    let ja = render_policy(BARE_XFMR_PEDAL, ForceJa);
    assert!(
        max_abs_diff(&der, &lin) < 1e-12,
        "Derived must be byte-identical to ForceLinear for a no-core-physics transformer"
    );
    assert!(
        max_abs_diff(&ja, &lin) > 1e-6,
        "ForceJa must still override a bare transformer to the JA root"
    );
}
