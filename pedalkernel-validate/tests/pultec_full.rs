//! Pultec EQP-1A FULL circuit (transformers + tube makeup) — BA283-playbook
//! validation run (bead pedalkernel-q3mf).
//!
//! Circuit: `circuits/eq/pultec_eqp1a.pedal` — the circuit-accurate reference
//! (T_in 1:1 bridging input, passive LCR program EQ, 12AX7 → 12AU7 makeup,
//! T_out 4:1 step-down into the 600 Ω line termination).
//!
//! Layers (each mirrors the BA283 playbook in `neve1073_spice.rs` /
//! `ba283_ib_diagnostic.rs`):
//!   1. bias  — settled per-device op (V1/V2 Vgk/Vpk/Ip) vs ngspice settled
//!              `.tran` reference (bare `.op` is DEGENERATE for Koren triodes).
//!   2. boundary — the boundary-load table row for the makeup stage must be
//!              `ReflectedThroughTransformer` once the secondary is terminated.
//!   3. AC    — settled single-bin gain vs ngspice at program-EQ settings.
//!   4. guard — Pultec compiles must produce zero edge-guard warnings.
//!
//! Probe (stage dump + boundary table + settled bias):
//! ```text
//! PK_PULTEC_PROBE=1 cargo test -p pedalkernel-validate --test pultec_full \
//!   probe_full_pultec -- --nocapture
//! ```

use pedalkernel::compiler::{compile_pedal, CompiledPedal};
use pedalkernel::dsl::parse_pedal_file;
use pedalkernel::PedalProcessor;
use pedalkernel_rt::processor::{BoundaryLoadDisposition, BoundaryLoadSummary, Stage};
use pedalkernel_validate::metrics;
use pedalkernel_validate::npy;
use pedalkernel_validate::report::render_boundary_loads;
use pedalkernel_validate::spice::{SpiceConfig, SpiceRunner};
use std::f64::consts::PI;
use std::path::PathBuf;

const SR: f64 = 48_000.0;
const OVERSAMPLE: u32 = 4;
const FULL: &str = "circuits/eq/pultec_eqp1a.pedal";

/// Steady-state settle. Slowest constants: Cc·Rg2 = 47 ms (→ 1 s ≈ 21 τ),
/// T_out Lm charge τ ≈ 5 ms, Ck1 τ ≈ 11 ms. Proofs below verify.
const SETTLE_S: f64 = 1.0;
/// Measurement window AFTER settling. 0.2 s = integer cycles at every test
/// frequency (60·0.2=12, 100·0.2=20, 1k/10k trivially).
const MEASURE_S: f64 = 0.2;
const SINE_AMP: f64 = 0.1;

/// (config, controls, test frequencies) — the program-EQ identity set:
/// flat reference + LF boost max (the famous low shelf) + HF atten max.
const AC_CONFIGS: &[(&str, &[(&str, f64)], &[f64])] = &[
    ("flat", &[], &[100.0, 1_000.0, 10_000.0]),
    ("lf_boost", &[("LF Boost", 1.0)], &[60.0, 1_000.0]),
    ("hf_atten", &[("HF Atten", 1.0)], &[1_000.0, 10_000.0]),
];

fn freq_tag(freq: f64) -> String {
    if freq >= 1_000.0 {
        format!("{}k", (freq / 1_000.0) as u32)
    } else {
        format!("{}", freq as u32)
    }
}

fn deck_path(config: &str) -> PathBuf {
    PathBuf::from(format!(
        "{}/spice-circuits/eq/pultec_eqp1a_full_{config}.spice",
        env!("CARGO_MANIFEST_DIR")
    ))
}

fn golden_path(config: &str, freq: f64) -> PathBuf {
    PathBuf::from(format!(
        "{}/golden/eq/pultec_full_{config}/sine_{}.npy",
        env!("CARGO_MANIFEST_DIR"),
        freq_tag(freq)
    ))
}

fn load(rel: &str) -> String {
    let path = format!("{}/{}", env!("CARGO_MANIFEST_DIR"), rel);
    std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {path}: {e}"))
}

fn compile_full() -> CompiledPedal {
    let def = parse_pedal_file(&load(FULL)).expect("parse pultec_eqp1a");
    compile_pedal(&def, SR).expect("compile pultec_eqp1a")
}

/// Print per-device op points from every MultiNl stage (triodes: v_be slot =
/// Vgk, v_ce slot = Vpk, i_c slot = Ip — same convention as runtime_op_points).
fn dump_device_ops(proc: &CompiledPedal, label: &str) {
    for (si, st) in proc.stages.iter().enumerate() {
        if let Stage::MultiNl(mnl) = st {
            for d in mnl.device_op_points() {
                eprintln!(
                    "    [{label}] stage {si} {}/{}: Vgk={:+.4} V  Vpk={:+.3} V  Ig={:+.3e} A  Ip={:+.4} mA",
                    d.ref_name, d.device_type, d.v_be, d.v_ce, d.i_b, d.i_c * 1e3
                );
            }
        }
    }
}

#[test]
fn probe_full_pultec() {
    if std::env::var("PK_PULTEC_PROBE").as_deref() != Ok("1") {
        eprintln!("  SKIP probe_full_pultec: set PK_PULTEC_PROBE=1 to run");
        return;
    }
    let mut proc = compile_full();

    eprintln!("=== stage dump ===");
    eprintln!("{}", proc.debug_dump());

    eprintln!("=== boundary-load table ===");
    eprintln!(
        "{}",
        render_boundary_loads(&[("pultec_eqp1a".to_string(), proc.boundary_loads.clone())])
    );

    eprintln!("=== cold (compile-time) device op ===");
    dump_device_ops(&proc, "cold");

    // Settle on silence 2 s and re-read.
    for _ in 0..(2.0 * SR) as usize {
        proc.process(0.0);
    }
    eprintln!("=== settled (2 s silence) device op ===");
    dump_device_ops(&proc, "settled");

    // Small AC probe at 1 kHz for level sanity.
    let n = (0.5 * SR) as usize;
    let out: Vec<f64> = (0..n)
        .map(|i| proc.process(0.1 * (2.0 * PI * 1_000.0 * i as f64 / SR).sin()))
        .collect();
    let tail = &out[n / 2..];
    let rms = (tail.iter().map(|x| x * x).sum::<f64>() / tail.len() as f64).sqrt();
    eprintln!("=== 1 kHz AC probe: out RMS {rms:.6} ({:+.2} dBFS) ===", 20.0 * rms.log10());

    // Flat frequency response (single-bin gain re input) — is there a spurious
    // HPF from the input-source port resistance (VoltageSource Rp=10k) against
    // T_in's 0.5 H magnetizing inductance (corner would be ~3.2 kHz)?
    eprintln!("=== WDF flat response (fresh compile per freq, 1 s settle) ===");
    for &freq in &[30.0, 60.0, 100.0, 300.0, 1_000.0, 3_000.0, 10_000.0] {
        let mut p = compile_full();
        let settle = SR as usize;
        for i in 0..settle {
            p.process(0.05 * (2.0 * PI * freq * i as f64 / SR).sin());
        }
        let m = (0.2 * SR) as usize;
        let out: Vec<f64> = (0..m)
            .map(|j| {
                let i = settle + j;
                p.process(0.05 * (2.0 * PI * freq * i as f64 / SR).sin())
            })
            .collect();
        let om = goertzel_mag(&out, freq);
        let gain_db = 20.0 * (om / 0.05).log10();
        eprintln!("    {freq:>8.0} Hz: {gain_db:+7.2} dB");
    }

    // Control A/B at the program frequencies — do the EQ pots shape the WDF
    // response at all? (ngspice: LF Boost moves 60 Hz by tens of dB.)
    eprintln!("=== WDF control A/B (0.05 amp, 1 s settle) ===");
    for (label, freq, ctrls) in [
        ("flat      @60", 60.0, vec![]),
        ("LF Boost=1 @60", 60.0, vec![("LF Boost", 1.0)]),
        ("flat      @10k", 10_000.0, vec![]),
        ("HF Atten=1 @10k", 10_000.0, vec![("HF Atten", 1.0)]),
    ] {
        let mut p = compile_full();
        for &(c, v) in &ctrls {
            p.set_control(c, v);
        }
        let settle = SR as usize;
        for i in 0..settle {
            p.process(0.05 * (2.0 * PI * freq * i as f64 / SR).sin());
        }
        let m = (0.2 * SR) as usize;
        let out: Vec<f64> = (0..m)
            .map(|j| p.process(0.05 * (2.0 * PI * freq * (settle + j) as f64 / SR).sin()))
            .collect();
        let g = 20.0 * (goertzel_mag(&out, freq) / 0.05).log10();
        eprintln!("    {label:<16}: {g:+7.2} dB");
    }
}

fn goertzel_mag(buf: &[f64], freq: f64) -> f64 {
    let w = 2.0 * PI * freq / SR;
    let coeff = 2.0 * w.cos();
    let (mut s1, mut s2) = (0.0, 0.0);
    for &x in buf {
        let s0 = x + coeff * s1 - s2;
        s2 = s1;
        s1 = s0;
    }
    let power = (s1 * s1 + s2 * s2 - coeff * s1 * s2) / (buf.len() as f64 / 2.0).powi(2);
    power.max(0.0).sqrt()
}

// ═══════════════════════════════════════════════════════════════════════
// Layer 2 — boundary: the 600 Ω termination must reflect through T_out
// ═══════════════════════════════════════════════════════════════════════

/// Gate for the bead-q3mf netlist fix: with `R_term` (600 Ω) across the T_out
/// secondary, the C4 reflection machinery must engage — the makeup stage's
/// output boundary carries a `GroundedResistive{600Ω}` load reflected through
/// the 4:1 transformer as n²·600 = 9.6 kΩ. Before the termination the table
/// was EMPTY (nothing hung on `out` — magnetizing-only was correct absence).
#[test]
fn pultec_full_output_boundary_reflects_through_transformer() {
    let proc = compile_full();
    println!(
        "{}",
        render_boundary_loads(&[("pultec_eqp1a".to_string(), proc.boundary_loads.clone())])
    );
    assert_eq!(
        proc.boundary_loads.len(),
        1,
        "expected exactly one analyzed output boundary, got {:?}",
        proc.boundary_loads
    );
    let b = &proc.boundary_loads[0];
    match &b.model {
        BoundaryLoadSummary::GroundedResistive {
            r_total: r,
            component_ids,
        } => {
            assert!((r - 600.0).abs() < 1e-6, "R_term = 600 Ω, got {r}");
            assert_eq!(component_ids, &["R_term".to_string()]);
        }
        other => panic!("expected GroundedResistive{{600Ω; R_term}}, got {other:?}"),
    }
    match &b.disposition {
        BoundaryLoadDisposition::ReflectedThroughTransformer {
            turns_ratio: n,
            r_reflected,
        } => {
            assert!((n - 4.0).abs() < 1e-9, "T_out is 4:1, got n={n}");
            assert!(
                (r_reflected - 9_600.0).abs() < 1e-6,
                "reflected load must be n²·600 = 9.6 kΩ, got {r_reflected}"
            );
        }
        other => panic!("expected ReflectedThroughTransformer, got {other:?}"),
    }
}

/// Layer 4 — edge guard: the full Pultec must compile with ZERO dropped
/// edges even in `PK_EDGE_GUARD=error` mode (post-#196 the transformer stamp
/// exists in the triode-context MNA; a failure here = new formation bead).
#[test]
fn pultec_full_compiles_clean_under_edge_guard_error() {
    std::env::set_var("PK_EDGE_GUARD", "error");
    let def = parse_pedal_file(&load(FULL)).expect("parse pultec_eqp1a");
    let result = compile_pedal(&def, SR);
    std::env::remove_var("PK_EDGE_GUARD");
    let proc = result.expect("pultec_eqp1a must compile clean under PK_EDGE_GUARD=error");
    assert_eq!(proc.stages.len(), 3, "expected 3 stages (T_in WDF, EQ+V1, V2+T_out)");
}

// ═══════════════════════════════════════════════════════════════════════
// Layer 1 — bias: settled WDF op vs ngspice settled-.tran reference
// ═══════════════════════════════════════════════════════════════════════

/// ngspice settled reference (flat config). Provenance: the full flat deck +
/// `VIN v_in 0 DC 0`, `.OPTIONS RELTOL=1e-4 ABSTOL=1e-10 VNTOL=1e-7`,
/// `tran 20u 3.0`, means over 2.6–3.0 s (ngspice-45.2). Settle proof: the
/// 2.0–2.4 s window equals the 2.6–3.0 s window to 7 digits. Root-uniqueness
/// proof: identical result with and without the deck's `.ic` hints. Bare
/// `.op` is NOT used (degenerate for Koren triodes — bead q3mf caution).
///
///   v(n_amp)    = 1.0001e-4   (V1 grid — Ig·R leak, engine-consistent)
///   v(n_cath1)  = 1.190881
///   v(n_plate1) = 205.6078
///   v(n_grid2)  = 1.0000e-3   (V2 grid — Ig·Rg2 leak)
///   v(n_cath2)  = 0.8921617
///   i(Ltop)     = 22.30404 mA (T_out primary DC — the magnetizing short)
const NG_V1_VGK: f64 = 1.0001e-4 - 1.190881; //  −1.190781 V
const NG_V1_VPK: f64 = 205.6078 - 1.190881; //  204.416919 V
const NG_V1_IP: f64 = 1.190881 / 1_500.0; //  0.793921 mA (Ik1; Ig ≈ −1 nA)
const NG_V2_VGK: f64 = 1.0000e-3 - 0.8921617; //  −0.891162 V
const NG_V2_VPK: f64 = 285.0 - 0.8921617; //  284.107838 V
const NG_V2_IP: f64 = 22.30404e-3 + 0.8921617 / 10_000.0; // 22.39326 mA (i_Ltop + Ik2)

/// Settled per-device op vs the ngspice settled-tran reference. The COLD
/// compile-time solve puts V2 at Vgk ≈ −13.27 V (it does not see the T_out
/// primary DC short) — the runtime walks to the true root within ~2 s; this
/// test grades the SETTLED root, and prints the cold one for the record.
#[test]
fn pultec_full_bias_dashboard() {
    let mut proc = compile_full();
    eprintln!("── Pultec full bias dashboard (flat) ──");
    eprintln!("  cold (compile-time) op:");
    dump_device_ops(&proc, "cold");
    for _ in 0..(3.0 * SR) as usize {
        proc.process(0.0);
    }
    eprintln!("  settled (3 s silence) op:");
    dump_device_ops(&proc, "settled");

    let mut ops = std::collections::BTreeMap::new();
    for st in proc.stages.iter() {
        if let Stage::MultiNl(mnl) = st {
            for d in mnl.device_op_points() {
                if !d.ref_name.is_empty() {
                    ops.insert(d.ref_name.clone(), (d.v_be, d.v_ce, d.i_c));
                }
            }
        }
    }
    let refs = [
        ("V1", NG_V1_VGK, NG_V1_VPK, NG_V1_IP),
        ("V2", NG_V2_VGK, NG_V2_VPK, NG_V2_IP),
    ];
    eprintln!(
        "  {:<4} {:>10} {:>10} {:>8} | {:>10} {:>10} {:>8} | {:>9} {:>9} {:>7}",
        "dev", "ng Vgk", "our Vgk", "Δ mV", "ng Vpk", "our Vpk", "Δ mV", "ng Ip mA", "our Ip", "Δ %"
    );
    for (name, ng_vgk, ng_vpk, ng_ip) in refs {
        let (vgk, vpk, ip) = *ops
            .get(name)
            .unwrap_or_else(|| panic!("no settled op for {name}"));
        let dvgk = (vgk - ng_vgk) * 1e3;
        let dvpk = (vpk - ng_vpk) * 1e3;
        let dip = (ip - ng_ip) / ng_ip * 100.0;
        eprintln!(
            "  {:<4} {:>10.5} {:>10.5} {:>8.2} | {:>10.4} {:>10.4} {:>8.1} | {:>9.4} {:>9.4} {:>7.3}",
            name, ng_vgk, vgk, dvgk, ng_vpk, vpk, dvpk, ng_ip * 1e3, ip * 1e3, dip
        );
        assert!(
            dvgk.abs() < 5.0,
            "{name} settled Vgk {vgk:.5} vs ngspice {ng_vgk:.5} (>5 mV) — bias root regressed"
        );
        assert!(
            dvpk.abs() < 100.0,
            "{name} settled Vpk {vpk:.4} vs ngspice {ng_vpk:.4} (>100 mV)"
        );
        assert!(
            dip.abs() < 1.0,
            "{name} settled Ip {:.4} mA vs ngspice {:.4} mA (>1 %)",
            ip * 1e3,
            ng_ip * 1e3
        );
    }
}

// ═══════════════════════════════════════════════════════════════════════
// Layer 3 — AC: settled goldens (ngspice-derived, never WDF-bootstrapped)
// ═══════════════════════════════════════════════════════════════════════

fn internal_input(freq: f64, settle_s: f64) -> (SpiceRunner, Vec<f64>) {
    let config = SpiceConfig {
        sample_rate: SR as u32,
        oversample: OVERSAMPLE,
    };
    let internal_rate = config.internal_rate();
    let n = ((settle_s + MEASURE_S) * internal_rate) as usize;
    let input: Vec<f64> = (0..n)
        .map(|i| SINE_AMP * (2.0 * PI * freq * i as f64 / internal_rate).sin())
        .collect();
    (SpiceRunner::new(config), input)
}

fn generate_settled_golden(config: &str, freq: f64, settle_s: f64) -> Vec<f64> {
    let circuit = deck_path(config);
    assert!(circuit.exists(), "deck not found: {circuit:?}");
    let (runner, input) = internal_input(freq, settle_s);
    runner
        .simulate_settled_window(&circuit, &input, "v_out", settle_s)
        .unwrap_or_else(|e| panic!("ngspice failed for pultec_full_{config}@{freq}: {e}"))
}

/// Bootstrap the settled AC goldens (all configs × frequencies). Proofs per
/// golden: base-rate length, settled mean ≈ 0, half-to-half drift < 0.5 dB;
/// plus an extended-settle stability proof on flat@1k.
///
/// ```text
/// PEDALKERNEL_BOOTSTRAP_PULTEC=1 cargo test -p pedalkernel-validate \
///   --test pultec_full bootstrap_pultec_full_goldens -- --nocapture
/// ```
#[test]
fn bootstrap_pultec_full_goldens() {
    if std::env::var("PEDALKERNEL_BOOTSTRAP_PULTEC").as_deref() != Ok("1") {
        eprintln!("  SKIP bootstrap_pultec_full_goldens: set PEDALKERNEL_BOOTSTRAP_PULTEC=1");
        return;
    }
    for (config, _, freqs) in AC_CONFIGS {
        for &freq in *freqs {
            let golden = generate_settled_golden(config, freq, SETTLE_S);
            let expected = (MEASURE_S * SR) as usize;
            assert!(
                golden.len().abs_diff(expected) <= OVERSAMPLE as usize,
                "{config}@{freq}: golden must be base-rate 48 kHz (len {} vs {expected})",
                golden.len()
            );
            let mean = metrics::dc_offset(&golden);
            let amp = metrics::single_bin_amplitude(&golden, freq, SR);
            let drift = metrics::ac_amplitude_drift_db(&golden, freq, SR);
            eprintln!(
                "  pultec_full_{config} sine_{}: amp {amp:.5} V  mean {mean:+.5} V  drift {drift:+.3} dB",
                freq_tag(freq)
            );
            assert!(
                mean.abs() < 0.05 * amp.max(1e-9) + 1e-3,
                "{config}@{freq}: mean {mean:+.5} not ≈0 — not settled"
            );
            assert!(
                drift.abs() < 0.5,
                "{config}@{freq}: AC level drifting {drift:+.3} dB — extend settle"
            );
            if *config == "flat" && freq == 1_000.0 {
                let long = generate_settled_golden(config, freq, SETTLE_S * 1.5);
                let amp_long = metrics::single_bin_amplitude(&long, freq, SR);
                let delta = 20.0 * (amp_long / amp).log10();
                eprintln!("    flat@1k extended-settle Δ = {delta:+.3} dB (must ≈ 0)");
                assert!(delta.abs() < 0.3, "flat@1k still transient: {delta:+.3} dB");
            }
            let path = golden_path(config, freq);
            std::fs::create_dir_all(path.parent().unwrap()).expect("create golden dir");
            npy::write_f64(&path, &golden).expect("write golden");
            eprintln!("    wrote {} samples → {path:?}", golden.len());
        }
    }
}

/// Run the WDF at base rate with `controls`, warm `SETTLE_S`, then record the
/// `MEASURE_S` window — same rate/length/phase as the golden.
fn wdf_settled_window(controls: &[(&str, f64)], freq: f64) -> Vec<f64> {
    let mut proc = compile_full();
    for &(label, val) in controls {
        proc.set_control(label, val);
    }
    let settle_n = (SETTLE_S * SR) as usize;
    let measure_n = (MEASURE_S * SR) as usize;
    for i in 0..settle_n {
        proc.process(SINE_AMP * (2.0 * PI * freq * i as f64 / SR).sin());
    }
    (0..measure_n)
        .map(|j| {
            let i = settle_n + j;
            proc.process(SINE_AMP * (2.0 * PI * freq * i as f64 / SR).sin())
        })
        .collect()
}

/// Regression gate for pedalkernel-n0yy: the program EQ must PARTICIPATE in
/// the transfer, not merely be stamped.
///
/// Pre-fix, the stage's adapted input port injected at V1.grid (the
/// amplifier-pin fallback ran before the input-boundary search), bypassing
/// the entire LCR network: flat@1k measured +23.3 dB re input (no EQ
/// insertion loss, V1 overdriven) and the EQ controls moved the response by
/// exactly 0.00 dB. Post-fix the port lands on the T_in-secondary boundary
/// node (`n_lf`) and both invariants flip:
///
/// * flat@1k overall gain is NEGATIVE (EQ insertion loss + makeup < unity
///   for this netlist; measured ≈ −21.9 dB — assert < −5 dB, wide margin).
/// * LF Boost 0→1 RAISES 60 Hz (measured +3.4 dB; assert > +1 dB —
///   authority magnitude itself is the pedalkernel-5ty3 netlist question).
#[test]
fn pultec_full_eq_participates_in_transfer() {
    let flat_1k = wdf_settled_window(&[], 1_000.0);
    let gain_1k_db =
        20.0 * (metrics::single_bin_amplitude(&flat_1k, 1_000.0, SR) / SINE_AMP).log10();
    println!("  flat@1k overall gain: {gain_1k_db:+.2} dB (pre-n0yy: +23.3 dB)");
    assert!(
        gain_1k_db < -5.0,
        "EQ insertion loss missing: flat@1k gain {gain_1k_db:+.2} dB — the \
         adapted input port is bypassing the program EQ again (pedalkernel-n0yy)"
    );

    let flat_60 = wdf_settled_window(&[], 60.0);
    let boost_60 = wdf_settled_window(&[("LF Boost", 1.0)], 60.0);
    let delta_db = 20.0
        * (metrics::single_bin_amplitude(&boost_60, 60.0, SR)
            / metrics::single_bin_amplitude(&flat_60, 60.0, SR))
        .log10();
    println!("  LF Boost 0->1 moves 60 Hz by {delta_db:+.2} dB (pre-n0yy: 0.00 dB)");
    assert!(
        delta_db > 1.0,
        "LF Boost has no authority at 60 Hz ({delta_db:+.2} dB) — the program \
         EQ is inert again (pedalkernel-n0yy)"
    );
}

/// AC dashboard: single-bin gain (WDF/golden) per config × frequency, plus
/// THD on both sides at flat@1k. REPORTS the gap per the bead charter (this
/// is a measurement run); hard assertions only on measurement validity:
/// goldens settled, both sides non-silent.
#[test]
fn pultec_full_ac_dashboard_vs_golden() {
    let mut rows: Vec<String> = Vec::new();
    let mut missing = 0usize;
    for (config, controls, freqs) in AC_CONFIGS {
        for &freq in *freqs {
            let gpath = golden_path(config, freq);
            if !gpath.exists() {
                eprintln!(
                    "  SKIP {config}@{freq}: golden missing at {gpath:?} \
                     (run bootstrap_pultec_full_goldens)"
                );
                missing += 1;
                continue;
            }
            let golden = npy::read_f64(&gpath).expect("read golden");
            let wdf = wdf_settled_window(controls, freq);
            let n = wdf.len().min(golden.len());
            let (wdf, golden) = (&wdf[..n], &golden[..n]);

            let gold_amp = metrics::single_bin_amplitude(golden, freq, SR);
            let wdf_amp = metrics::single_bin_amplitude(wdf, freq, SR);
            let gold_mean = metrics::dc_offset(golden);
            let wdf_drift = metrics::ac_amplitude_drift_db(wdf, freq, SR);
            let gain_gap = metrics::ac_gain_db(wdf, golden, freq, SR);

            // Measurement-validity guards (not accuracy gates).
            assert!(
                gold_mean.abs() < 0.05 * gold_amp.max(1e-9) + 1e-3,
                "{config}@{freq}: golden not settled (mean {gold_mean:+.5})"
            );
            assert!(gold_amp > 1e-6, "{config}@{freq}: golden silent");
            assert!(wdf_amp > 1e-9, "{config}@{freq}: WDF silent");
            assert!(
                wdf_drift.abs() < 0.5,
                "{config}@{freq}: WDF still drifting {wdf_drift:+.3} dB"
            );

            let mut row = format!(
                "{config:<9} {:>6} Hz  golden {gold_amp:>9.5} V  wdf {wdf_amp:>9.5} V  gap {gain_gap:>+7.2} dB",
                freq_tag(freq)
            );
            if *config == "flat" && freq == 1_000.0 {
                let thd_g = metrics::thd_db(golden, freq, SR, 8);
                let thd_w = metrics::thd_db(wdf, freq, SR, 8);
                row.push_str(&format!("  | THD golden {thd_g:+.1} dB  wdf {thd_w:+.1} dB"));
            }
            rows.push(row);
        }
    }
    eprintln!("── Pultec full AC dashboard (WDF vs ngspice settled goldens) ──");
    for r in &rows {
        eprintln!("  {r}");
    }
    assert!(
        missing == 0 || !rows.is_empty(),
        "no goldens present at all — bootstrap first"
    );
}
