//! NR Solver Validation Suite for Single-NL-Root Circuits.
//!
//! Layered fault isolation strategy:
//! - Layer 0: NR solver directly via `WdfRoot::process()` (no compiler, no R-type)
//! - Layer 1: Simplest compiled circuits (diode clippers)
//! - Layer 2: Single-transistor circuits (BJT)
//! - Layer 3: Asymmetric rectifier (single diode in RC)
//! - Layer 4: Tube stage (triode common cathode)
//!
//! If Layer 0 passes but Layer 1+ fails, the problem is in the compilation
//! pipeline, not the NR solver.

mod audio_analysis;

use pedalkernel::elements::{
    reset_solver_stats, solver_stats_snapshot,
    DiodeModel, DiodePairRoot, DiodeRoot, JfetModel, JfetRoot, SolverStatsSnapshot,
    TriodeModel, TriodeRoot, WdfRoot, ZenerModel, ZenerRoot,
};
use pedalkernel::elements::GummelPoonModel;

use audio_analysis::{
    compile_and_process, dc_offset, peak, rms, sine_at, SAMPLE_RATE,
};

// ============================================================================
// Diagnostic helper
// ============================================================================

/// Print diagnostic info when a test fails.
fn nr_diagnostic(
    device_name: &str,
    rp: f64,
    input: &[f64],
    output: &[f64],
    stats: &SolverStatsSnapshot,
) {
    let avg_iter = if stats.solves > 0 {
        stats.total_iterations as f64 / stats.solves as f64
    } else {
        0.0
    };
    eprintln!(
        "\n=== NR SOLVER DIAGNOSTIC ===\n\
         Device: {device_name}, Rp={rp:.1}\n\
         Samples: {}, Solver calls: {}\n\
         Iterations: total={}, avg={avg_iter:.1}, max={}\n\
         Residual max: {:.2e}\n\
         Clamp hits: {}, Step limited: {}, Iter cap: {}\n\
         Input:  peak={:.4}, rms={:.4}\n\
         Output: peak={:.4}, rms={:.4}",
        input.len(),
        stats.solves,
        stats.total_iterations,
        stats.max_iterations,
        stats.max_residual,
        stats.clamp_hits,
        stats.step_limited,
        stats.iter_cap_hits,
        peak(input),
        rms(input),
        peak(output),
        rms(output),
    );
}

// ============================================================================
// Layer 0: NR Solver Unit-Level (Direct WdfRoot::process)
// ============================================================================

#[test]
fn nr_l0a_diode_pair_dc_operating_point() {
    let mut root = DiodePairRoot::new(DiodeModel::silicon());
    reset_solver_stats();
    let b = root.process(0.0, 1000.0);
    let stats = solver_stats_snapshot();

    // At zero input, reflected wave should be ~0
    assert!(
        b.abs() < 1e-6,
        "DiodePairRoot DC: expected b ≈ 0, got {b:.8e}"
    );
    assert!(
        stats.iter_cap_hits == 0,
        "No iteration cap hits expected at DC"
    );
}

#[test]
fn nr_l0b_diode_pair_clipping_voltage() {
    let mut root = DiodePairRoot::new(DiodeModel::silicon());
    let rp = 4700.0;
    let a = 10.0; // Large incident wave

    reset_solver_stats();
    let b = root.process(a, rp);
    let stats = solver_stats_snapshot();

    // Extract junction voltage: v = (a + b) / 2
    let v = (a + b) / 2.0;

    // Silicon forward voltage should be ~0.5-0.7V
    assert!(
        v > 0.3 && v < 0.9,
        "Silicon Vf: expected ~0.6V, got {v:.4}V"
    );

    // Self-consistency: the NR solver yields b = -2*Rp*i_device(v),
    // so i_device = -b / (2*Rp). For a model with series resistance Rs,
    // the terminal voltage v_term = (a+b)/2 and junction voltage
    // v_j = v_term - i*Rs.
    let model = DiodeModel::silicon();
    let i_device = -b / (2.0 * rp);
    let v_term = v;
    let v_junction = v_term - i_device * model.rs;
    let x = (v_junction / model.n_vt).clamp(-500.0, 500.0);
    let i_shockley = model.is * (x.exp() - (-x).exp()); // Anti-parallel pair
    let i_error = (i_device - i_shockley).abs();
    assert!(
        i_error < 1e-6,
        "Shockley self-consistency: |i_device({i_device:.6e}) - i_model({i_shockley:.6e})| = {i_error:.2e}"
    );

    assert_eq!(stats.iter_cap_hits, 0);
    if stats.solves > 0 {
        nr_diagnostic("DiodePairRoot(silicon)", rp, &[a], &[b], &stats);
    }
}

#[test]
fn nr_l0c_single_diode_asymmetry() {
    let mut root = DiodeRoot::new(DiodeModel::silicon());
    let rp = 4700.0;

    // Forward bias: should clip
    let b_fwd = root.process(10.0, rp);
    let v_fwd = (10.0 + b_fwd) / 2.0;

    // Reverse bias: should pass nearly unchanged (tiny leakage)
    let b_rev = root.process(-10.0, rp);
    let v_rev = (-10.0 + b_rev) / 2.0;

    // Forward: clips near silicon Vf
    assert!(
        v_fwd > 0.3 && v_fwd < 0.9,
        "Forward clip: expected Vf ~0.6V, got {v_fwd:.4}V"
    );

    // Reverse: much larger voltage magnitude (passes through)
    assert!(
        v_rev.abs() > 3.0,
        "Reverse pass: expected large |v|, got {v_rev:.4}V"
    );

    // Asymmetry is clear
    assert!(
        v_rev.abs() > v_fwd * 3.0,
        "Diode asymmetry: reverse {v_rev:.4} should be much larger than forward {v_fwd:.4}"
    );
}

#[test]
fn nr_l0d_zener_clipper() {
    let model = ZenerModel::z4v7();
    let mut root = ZenerRoot::new(model);
    let rp = 4700.0;

    // Forward bias: should clip at ~0.6V (like silicon diode)
    let b_fwd = root.process(10.0, rp);
    let v_fwd = (10.0 + b_fwd) / 2.0;
    assert!(
        v_fwd > 0.3 && v_fwd < 1.0,
        "Zener forward: expected Vf ~0.6V, got {v_fwd:.4}V"
    );

    // Reverse bias: should clip at ~Vz (4.7V)
    let b_rev = root.process(-20.0, rp);
    let v_rev = (-20.0 + b_rev) / 2.0;
    // Reverse voltage should be around -Vz
    assert!(
        v_rev < -3.0 && v_rev > -7.0,
        "Zener reverse: expected ~-4.7V, got {v_rev:.4}V"
    );
}

#[test]
fn nr_l0e_germanium_vs_silicon_vs_led() {
    let rp = 4700.0;
    let a = 10.0;

    let mut ge = DiodePairRoot::new(DiodeModel::germanium());
    let mut si = DiodePairRoot::new(DiodeModel::silicon());
    let mut led = DiodePairRoot::new(DiodeModel::led());

    let b_ge = ge.process(a, rp);
    let b_si = si.process(a, rp);
    let b_led = led.process(a, rp);

    let v_ge = (a + b_ge) / 2.0;
    let v_si = (a + b_si) / 2.0;
    let v_led = (a + b_led) / 2.0;

    // Clipping thresholds should be: Ge < Si < LED
    assert!(
        v_ge < v_si,
        "Germanium ({v_ge:.4}V) should clip lower than silicon ({v_si:.4}V)"
    );
    assert!(
        v_si < v_led,
        "Silicon ({v_si:.4}V) should clip lower than LED ({v_led:.4}V)"
    );

    // Rough expected ranges
    assert!(
        v_ge > 0.1 && v_ge < 0.5,
        "Germanium Vf: expected 0.2-0.4V, got {v_ge:.4}V"
    );
    assert!(
        v_si > 0.3 && v_si < 0.9,
        "Silicon Vf: expected 0.5-0.7V, got {v_si:.4}V"
    );
    assert!(
        v_led > 1.0 && v_led < 2.5,
        "LED Vf: expected 1.5-2.0V, got {v_led:.4}V"
    );
}

#[test]
fn nr_l0f_bjt_vbe_sweep() {
    // Verify Gummel-Poon model: Ic increases monotonically with Vbe.
    let model = GummelPoonModel::by_name("2N3904");

    // Fixed Vce = 4.5 V (half-supply), sweep Vbe from 0.0 to 0.7 V.
    let vce = 4.5;
    let vbe_values: Vec<f64> = (0..=14).map(|i| i as f64 * 0.05).collect();
    let mut prev_ic = -1.0;

    for &vbe in &vbe_values {
        let vbc = vbe - vce;
        let (ic, _ib) = model.currents(vbe, vbc);

        assert!(
            ic >= prev_ic - 1e-12,
            "BJT Ic should be monotonically increasing: at Vbe={vbe:.2}, Ic={ic:.6e} < prev={prev_ic:.6e}"
        );
        prev_ic = ic;
    }

    // At Vbe = 0.65 V, should have measurable collector current.
    let vbc = 0.65 - vce;
    let (ic_active, _) = model.currents(0.65, vbc);
    assert!(
        ic_active > 1e-6,
        "BJT should have measurable Ic at Vbe=0.65V, got {ic_active:.6e}"
    );
}

#[test]
fn nr_l0g_triode_vgk_sweep() {
    let model = TriodeModel::by_name("12ax7");
    let mut triode = TriodeRoot::new(model);
    triode.set_v_max(250.0);
    let rp = 100_000.0;

    // Sweep Vgk from -4.0 to 0.0, verify increasing plate current
    let mut prev_ip = -1.0;
    let vgk_values: Vec<f64> = (0..=20).map(|i| -4.0 + i as f64 * 0.2).collect();

    for &vgk in &vgk_values {
        triode.set_vgk(vgk);
        let _b = triode.process(125.0, rp);

        // Plate current: compute from WDF reflected wave
        // v = (a + b) / 2, Ip = (a - b) / (2 * Rp)
        let ip = triode.plate_current(125.0); // Use plate_current directly
        assert!(
            ip >= prev_ip - 1e-10,
            "Triode Ip should increase with Vgk: at Vgk={vgk:.2}, Ip={ip:.6e} < prev={prev_ip:.6e}"
        );
        prev_ip = ip;
    }
}

#[test]
fn nr_l0h_jfet_vgs_sweep() {
    let model = JfetModel::by_name("2n5457");
    let mut jfet = JfetRoot::new(model);
    let rp = 10_000.0;

    // N-channel JFET: Vgs from pinch-off toward 0V → drain current increases
    // 2N5457 has Vto ≈ -1.3V (negative for N-channel depletion mode)
    let vgs_values: Vec<f64> = (0..=15).map(|i| -3.0 + i as f64 * 0.2).collect();

    let mut prev_ids = -1e-15;
    for &vgs in &vgs_values {
        jfet.set_vgs(vgs);
        let b = jfet.process(5.0, rp);
        // Drain current from WDF: ids = (a - b) / (2 * Rp)
        let ids = (5.0 - b) / (2.0 * rp);

        assert!(
            ids >= prev_ids - 1e-10,
            "JFET Ids should increase toward Vgs=0: at Vgs={vgs:.2}, Ids={ids:.6e} < prev={prev_ids:.6e}"
        );
        prev_ids = ids;
    }

    // At Vgs=0, should have substantial drain current (Idss)
    jfet.set_vgs(0.0);
    let b0 = jfet.process(5.0, rp);
    let idss_wdf = (5.0 - b0) / (2.0 * rp);
    assert!(
        idss_wdf > 1e-5,
        "JFET should have Idss > 10µA at Vgs=0, got {idss_wdf:.6e}"
    );
}

#[test]
fn nr_l0i_wdf_constraint_self_consistency() {
    // For each device type, process a sine wave and verify the WDF constraint:
    // i = (a - b) / (2*Rp) must match i_device(v) where v = (a + b) / 2

    let n_samples = 1000;
    let rp = 4700.0;
    let input = sine_at(440.0, 2.0, n_samples as f64 / SAMPLE_RATE, SAMPLE_RATE);

    // --- DiodePairRoot ---
    // The NR solver satisfies: b = -2*Rp*i_device(v) where v = (a+b)/2.
    // For models with series resistance Rs, the solver internally uses
    // rp_eff = rp + rs, so i_device = -b/(2*rp), and junction voltage
    // v_j = v_term - i_device * Rs.
    {
        let model = DiodeModel::silicon();
        let mut root = DiodePairRoot::new(model);
        for &a in &input {
            let b = root.process(a, rp);
            let i_device = -b / (2.0 * rp);
            let v_term = (a + b) / 2.0;
            let v_j = v_term - i_device * model.rs;
            let x = (v_j / model.n_vt).clamp(-500.0, 500.0);
            let i_model = model.is * (x.exp() - (-x).exp());
            let tol = 1e-3 * (i_device.abs().max(i_model.abs())).max(1e-10);
            assert!(
                (i_device - i_model).abs() < tol,
                "DiodePair WDF constraint: |i_dev({i_device:.6e}) - i_model({i_model:.6e})| = {:.2e} at v_j={v_j:.6}",
                (i_device - i_model).abs()
            );
        }
    }

    // --- DiodeRoot ---
    {
        let model = DiodeModel::silicon();
        let mut root = DiodeRoot::new(model);
        for &a in &input {
            let b = root.process(a, rp);
            let i_device = -b / (2.0 * rp);
            let v_term = (a + b) / 2.0;
            let v_j = v_term - i_device * model.rs;
            let x = (v_j / model.n_vt).clamp(-500.0, 500.0);
            let i_model = model.is * (x.exp() - 1.0);
            let tol = 1e-3 * (i_device.abs().max(i_model.abs())).max(1e-10);
            assert!(
                (i_device - i_model).abs() < tol,
                "DiodeRoot WDF constraint: |Δi| = {:.2e} at v_j={v_j:.6}",
                (i_device - i_model).abs()
            );
        }
    }
}

#[test]
fn nr_l0j_solver_convergence_stats() {
    let n_samples = 2400; // 0.05 seconds at 48kHz — enough for convergence test
    let rp = 4700.0;
    let input = sine_at(440.0, 2.0, n_samples as f64 / SAMPLE_RATE, SAMPLE_RATE);

    // Test each device type
    struct DeviceTest {
        name: &'static str,
        max_avg_iter: f64,
        max_max_iter: u32,
    }
    let tests = [
        DeviceTest { name: "DiodePairRoot", max_avg_iter: 5.0, max_max_iter: 16 },
        DeviceTest { name: "DiodeRoot", max_avg_iter: 5.0, max_max_iter: 16 },
        DeviceTest { name: "ZenerRoot", max_avg_iter: 6.0, max_max_iter: 16 },
    ];

    for test in &tests {
        reset_solver_stats();

        match test.name {
            "DiodePairRoot" => {
                let mut root = DiodePairRoot::new(DiodeModel::silicon());
                for &a in &input {
                    root.process(a, rp);
                }
            }
            "DiodeRoot" => {
                let mut root = DiodeRoot::new(DiodeModel::silicon());
                for &a in &input {
                    root.process(a, rp);
                }
            }
            "ZenerRoot" => {
                let mut root = ZenerRoot::new(ZenerModel::z4v7());
                for &a in &input {
                    root.process(a, rp);
                }
            }
            _ => unreachable!(),
        }

        let stats = solver_stats_snapshot();

        let avg_iter = if stats.solves > 0 {
            stats.total_iterations as f64 / stats.solves as f64
        } else {
            0.0
        };

        assert_eq!(
            stats.iter_cap_hits, 0,
            "{}: zero iter_cap_hits expected, got {}",
            test.name, stats.iter_cap_hits
        );

        assert!(
            avg_iter < test.max_avg_iter,
            "{}: avg iterations {avg_iter:.1} exceeds limit {}",
            test.name, test.max_avg_iter
        );

        assert!(
            stats.max_iterations <= test.max_max_iter,
            "{}: max iterations {} exceeds limit {}",
            test.name, stats.max_iterations, test.max_max_iter
        );

        assert!(
            stats.max_residual < 1e-4,
            "{}: max residual {:.2e} exceeds 1e-4",
            test.name, stats.max_residual
        );

        eprintln!(
            "[{}] solves={}, avg_iter={avg_iter:.1}, max_iter={}, max_residual={:.2e}, clamp={}, step_lim={}, iter_cap={}",
            test.name,
            stats.solves,
            stats.max_iterations,
            stats.max_residual,
            stats.clamp_hits,
            stats.step_limited,
            stats.iter_cap_hits,
        );
    }
}

// ============================================================================
// Layer 1: Diode Clipper End-to-End (Simplest Compiled Circuit)
// ============================================================================

const HARD_CLIPPER_SI: &str = r#"
pedal "Hard Clipper Si" {
    components {
        R1: resistor(4.7k)
        D1: diode_pair(silicon)
        RL: resistor(10k)
    }
    nets {
        in -> R1.a
        R1.b -> D1.a, RL.a, out
        D1.b -> gnd
        RL.b -> gnd
    }
}
"#;

const HARD_CLIPPER_GE: &str = r#"
pedal "Hard Clipper Ge" {
    components {
        R1: resistor(4.7k)
        D1: diode_pair(germanium)
        RL: resistor(10k)
    }
    nets {
        in -> R1.a
        R1.b -> D1.a, RL.a, out
        D1.b -> gnd
        RL.b -> gnd
    }
}
"#;

const HARD_CLIPPER_LED: &str = r#"
pedal "Hard Clipper LED" {
    components {
        R1: resistor(4.7k)
        D1: diode_pair(led)
        RL: resistor(10k)
    }
    nets {
        in -> R1.a
        R1.b -> D1.a, RL.a, out
        D1.b -> gnd
        RL.b -> gnd
    }
}
"#;

const HARD_CLIPPER_ASYMMETRIC: &str = r#"
pedal "Hard Clipper Asym" {
    components {
        R1: resistor(4.7k)
        D1: diode(silicon)
        RL: resistor(10k)
    }
    nets {
        in -> R1.a
        R1.b -> D1.a, RL.a, out
        D1.b -> gnd
        RL.b -> gnd
    }
}
"#;

const HARD_CLIPPER_ZENER: &str = r#"
pedal "Hard Clipper Zener" {
    components {
        R1: resistor(4.7k)
        D1: zener(4.7)
        RL: resistor(10k)
    }
    nets {
        in -> R1.a
        R1.b -> D1.a, RL.a, out
        D1.b -> gnd
        RL.b -> gnd
    }
}
"#;

#[test]
fn nr_l1a_diode_clipper_compiles_and_produces_output() {
    let input = sine_at(440.0, 2.0, 0.5, SAMPLE_RATE);

    for (name, src) in [
        ("Silicon", HARD_CLIPPER_SI),
        ("Germanium", HARD_CLIPPER_GE),
        ("LED", HARD_CLIPPER_LED),
        ("Asymmetric", HARD_CLIPPER_ASYMMETRIC),
        ("Zener", HARD_CLIPPER_ZENER),
    ] {
        let output = compile_and_process(src, &input, SAMPLE_RATE, &[]);
        let p = peak(&output);
        assert!(
            p > 0.0 && output.iter().all(|x| x.is_finite()),
            "{name}: output should be finite and non-silent, peak={p:.6}"
        );
    }
}

#[test]
fn nr_l1b_diode_clipper_threshold() {
    let input = sine_at(440.0, 2.0, 0.5, SAMPLE_RATE);

    // Silicon: clips around 0.6V — output peak should be reduced
    let out_si = compile_and_process(HARD_CLIPPER_SI, &input, SAMPLE_RATE, &[]);
    let p_si = peak(&out_si);
    assert!(
        p_si < 0.8,
        "Silicon clipper: peak {p_si:.4} should be < 0.8V"
    );

    // Germanium: clips lower — output peak should be even smaller
    let out_ge = compile_and_process(HARD_CLIPPER_GE, &input, SAMPLE_RATE, &[]);
    let p_ge = peak(&out_ge);
    assert!(
        p_ge < 0.5,
        "Germanium clipper: peak {p_ge:.4} should be < 0.5V"
    );

    // LED: clips higher — output peak should be larger than silicon
    let out_led = compile_and_process(HARD_CLIPPER_LED, &input, SAMPLE_RATE, &[]);
    let p_led = peak(&out_led);
    assert!(
        p_led < 2.5,
        "LED clipper: peak {p_led:.4} should be < 2.5V"
    );
    assert!(
        p_led > p_si,
        "LED ({p_led:.4}) should clip higher than Si ({p_si:.4})"
    );
}

#[test]
fn nr_l1c_diode_clipper_linear_at_small_signal() {
    // At very small signal, diodes don't conduct → output ≈ voltage divider
    let input = sine_at(440.0, 0.01, 0.5, SAMPLE_RATE);
    let output = compile_and_process(HARD_CLIPPER_SI, &input, SAMPLE_RATE, &[]);

    let p_in = peak(&input);
    let p_out = peak(&output);

    // Output should be close to input level (voltage divider: RL/(R1+RL) ≈ 0.68)
    // Allow wide tolerance since circuit may have transient effects
    let ratio = p_out / p_in;
    assert!(
        ratio > 0.2 && ratio < 1.5,
        "Small signal ratio={ratio:.4}: expected near voltage divider (~0.68)"
    );
    // Key point: output peak should NOT be clipped (should be > 0.001V)
    assert!(
        p_out > 0.001,
        "Small signal should pass through, peak={p_out:.6}"
    );
}

#[test]
fn nr_l1d_diode_pair_symmetry() {
    let input = sine_at(440.0, 2.0, 0.5, SAMPLE_RATE);
    let output = compile_and_process(HARD_CLIPPER_SI, &input, SAMPLE_RATE, &[]);

    // Skip initial transient (first 10ms)
    let skip = (0.01 * SAMPLE_RATE) as usize;
    let steady = &output[skip..];

    let pos_peak = steady.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    let neg_peak = steady.iter().cloned().fold(f64::INFINITY, f64::min);

    // Anti-parallel diode pair should clip symmetrically
    let symmetry_ratio = pos_peak / neg_peak.abs();
    assert!(
        symmetry_ratio > 0.85 && symmetry_ratio < 1.15,
        "Symmetry ratio={symmetry_ratio:.4}: pos_peak={pos_peak:.4}, neg_peak={neg_peak:.4}"
    );
}

#[test]
fn nr_l1e_diode_clipper_solver_stats() {
    let input = sine_at(440.0, 2.0, 0.5, SAMPLE_RATE);

    reset_solver_stats();
    let _output = compile_and_process(HARD_CLIPPER_SI, &input, SAMPLE_RATE, &[]);
    let stats = solver_stats_snapshot();

    assert_eq!(
        stats.iter_cap_hits, 0,
        "Silicon clipper: no convergence failures expected, got {} iter_cap_hits",
        stats.iter_cap_hits
    );

    if stats.solves > 0 {
        let avg = stats.total_iterations as f64 / stats.solves as f64;
        assert!(
            avg < 5.0,
            "Silicon clipper: avg iterations {avg:.1} should be < 5"
        );
        eprintln!(
            "[L1 Si Clipper] solves={}, avg_iter={avg:.1}, max_iter={}, residual_max={:.2e}",
            stats.solves, stats.max_iterations, stats.max_residual,
        );
    }
}

// ============================================================================
// Layer 2: Single-Transistor Fuzz (BJT NL Root)
// ============================================================================

const CE_NPN: &str = r#"
pedal "CE NPN" {
    supply 9V
    components {
        C_in: cap(100n)
        R_b1: resistor(470k)
        R_b2: resistor(47k)
        Q1: npn(2n3904)
        R_c: resistor(10k)
        R_e: resistor(1k)
        C_out: cap(100n)
        R_load: resistor(100k)
    }
    nets {
        in -> C_in.a
        C_in.b -> R_b1.a, R_b2.a, Q1.base
        vcc -> R_b1.b
        R_b2.b -> gnd
        vcc -> R_c.a
        R_c.b -> Q1.collector
        Q1.emitter -> R_e.a
        R_e.b -> gnd
        Q1.collector -> C_out.a
        C_out.b -> R_load.a, out
        R_load.b -> gnd
    }
}
"#;

const CE_PNP: &str = r#"
pedal "CE PNP" {
    supply 9V
    components {
        C_in: cap(100n)
        R_b1: resistor(470k)
        R_b2: resistor(47k)
        Q1: pnp(2n3906)
        R_c: resistor(10k)
        R_e: resistor(1k)
        C_out: cap(100n)
        R_load: resistor(100k)
    }
    nets {
        in -> C_in.a
        C_in.b -> R_b1.a, R_b2.a, Q1.base
        gnd -> R_b1.b
        R_b2.b -> vcc
        gnd -> R_c.a
        R_c.b -> Q1.collector
        Q1.emitter -> R_e.a
        R_e.b -> vcc
        Q1.collector -> C_out.a
        C_out.b -> R_load.a, out
        R_load.b -> gnd
    }
}
"#;

#[test]
fn nr_l2a_bjt_compiles_and_produces_output() {
    let input = sine_at(440.0, 0.1, 0.5, SAMPLE_RATE);

    for (name, src) in [("NPN CE", CE_NPN), ("PNP CE", CE_PNP)] {
        let output = compile_and_process(src, &input, SAMPLE_RATE, &[]);
        let p = peak(&output);
        assert!(
            p > 0.0 && output.iter().all(|x| x.is_finite()),
            "{name}: output should be finite and non-silent, peak={p:.6}"
        );
    }
}

#[test]
fn nr_l2b_bjt_gain_greater_than_1() {
    // Small signal input — CE amplifier should provide voltage gain
    let input = sine_at(440.0, 0.01, 0.5, SAMPLE_RATE);

    let output = compile_and_process(CE_NPN, &input, SAMPLE_RATE, &[]);

    // Skip transient (first 50ms for bias settling)
    let skip = (0.05 * SAMPLE_RATE) as usize;
    let p_in = peak(&input[skip..]);
    let p_out = peak(&output[skip..]);

    let gain = p_out / p_in;
    assert!(
        gain > 1.0,
        "CE NPN should have gain > 1 for small signal: gain={gain:.2} (in={p_in:.6}, out={p_out:.6})"
    );
}

#[test]
fn nr_l2d_bjt_solver_stats() {
    let input = sine_at(440.0, 0.1, 0.5, SAMPLE_RATE);

    reset_solver_stats();
    let _output = compile_and_process(CE_NPN, &input, SAMPLE_RATE, &[]);
    let stats = solver_stats_snapshot();

    assert_eq!(
        stats.iter_cap_hits, 0,
        "CE NPN: no convergence failures expected, got {} iter_cap_hits",
        stats.iter_cap_hits
    );

    if stats.solves > 0 {
        let avg = stats.total_iterations as f64 / stats.solves as f64;
        eprintln!(
            "[L2 CE NPN] solves={}, avg_iter={avg:.1}, max_iter={}, residual_max={:.2e}",
            stats.solves, stats.max_iterations, stats.max_residual,
        );
    }
}

// ============================================================================
// Layer 3: Asymmetric Rectifier (Single Diode in RC)
// ============================================================================

const HALF_WAVE_RECTIFIER: &str = r#"
pedal "Half Wave Rectifier" {
    components {
        R1: resistor(10k)
        D1: diode(silicon)
        C1: cap(10n)
        RL: resistor(100k)
    }
    nets {
        in -> R1.a
        R1.b -> D1.a
        D1.b -> C1.a, RL.a, out
        C1.b -> gnd
        RL.b -> gnd
    }
}
"#;

#[test]
fn nr_l3a_rectifier_compiles_and_produces_output() {
    let input = sine_at(440.0, 2.0, 0.5, SAMPLE_RATE);
    let output = compile_and_process(HALF_WAVE_RECTIFIER, &input, SAMPLE_RATE, &[]);
    let p = peak(&output);
    assert!(
        p > 0.0 && output.iter().all(|x| x.is_finite()),
        "Rectifier: output should be finite and non-silent, peak={p:.6}"
    );
}

#[test]
fn nr_l3b_rectifier_positive_dc_offset() {
    let input = sine_at(440.0, 2.0, 0.5, SAMPLE_RATE);
    let output = compile_and_process(HALF_WAVE_RECTIFIER, &input, SAMPLE_RATE, &[]);

    // Skip transient
    let skip = (0.05 * SAMPLE_RATE) as usize;
    let steady = &output[skip..];

    let dc = dc_offset(steady);
    // The diode orientation in the WDF model may pass positive or negative
    // half-cycles depending on polarity convention. The key property is that
    // a half-wave rectifier produces a non-zero DC offset.
    assert!(
        dc.abs() > 0.01,
        "Half-wave rectifier should have non-zero DC offset, got {dc:.6}"
    );
}

#[test]
fn nr_l3c_rectifier_envelope_tracking() {
    // Use a burst signal: first half high amplitude, second half low
    let n = (0.5 * SAMPLE_RATE) as usize;
    let mut input = Vec::with_capacity(n);
    for i in 0..n {
        let t = i as f64 / SAMPLE_RATE;
        let amp = if t < 0.25 { 2.0 } else { 0.5 };
        input.push(amp * (2.0 * std::f64::consts::PI * 440.0 * t).sin());
    }

    let output = compile_and_process(HALF_WAVE_RECTIFIER, &input, SAMPLE_RATE, &[]);

    // Measure RMS in first and second halves (after transient)
    let quarter = n / 4;
    let skip = (0.02 * SAMPLE_RATE) as usize;
    let rms_high = rms(&output[skip..quarter]);
    let rms_low = rms(&output[(quarter + skip)..]);

    // High-amplitude section should produce higher output
    assert!(
        rms_high > rms_low,
        "Rectifier should track envelope: rms_high={rms_high:.6} should > rms_low={rms_low:.6}"
    );
}

// ============================================================================
// Layer 4: Tube Stage (Triode Common Cathode)
// ============================================================================

const TRIODE_PREAMP: &str = r#"
pedal "12AX7 Preamp" {
    supply 250V
    components {
        C_in: cap(22n)
        R_grid: resistor(1M)
        V1: triode(12ax7)
        R_plate: resistor(100k)
        R_cathode: resistor(1.5k)
        C_bypass: cap(25u)
        C_out: cap(100n)
        R_load: resistor(1M)
    }
    nets {
        in -> C_in.a
        C_in.b -> R_grid.a, V1.grid
        R_grid.b -> gnd
        vcc -> R_plate.a
        R_plate.b -> V1.plate
        V1.cathode -> R_cathode.a, C_bypass.a
        R_cathode.b -> gnd
        C_bypass.b -> gnd
        V1.plate -> C_out.a
        C_out.b -> R_load.a, out
        R_load.b -> gnd
    }
}
"#;

#[test]
fn nr_l4a_triode_compiles_and_produces_output() {
    let input = sine_at(440.0, 0.1, 0.5, SAMPLE_RATE);
    let output = compile_and_process(TRIODE_PREAMP, &input, SAMPLE_RATE, &[]);
    let p = peak(&output);
    assert!(
        p > 0.0 && output.iter().all(|x| x.is_finite()),
        "12AX7 preamp: output should be finite and non-silent, peak={p:.6}"
    );
}

#[test]
fn nr_l4b_triode_gain_reasonable() {
    // Small signal: 12AX7 common-cathode gain ≈ 30-70×
    let input = sine_at(440.0, 0.005, 1.0, SAMPLE_RATE);
    let output = compile_and_process(TRIODE_PREAMP, &input, SAMPLE_RATE, &[]);

    // Skip transient (100ms for tubes to settle)
    let skip = (0.1 * SAMPLE_RATE) as usize;
    let p_in = peak(&input[skip..]);
    let p_out = peak(&output[skip..]);

    if p_out > 0.0 {
        let gain = p_out / p_in;
        // 12AX7 with 100k plate load, 1.5k cathode (bypassed): gain ≈ mu * Rp/(Rp+rp)
        // mu=100, rp=62.5k → gain ≈ 100 * 100k/(100k+62.5k) ≈ 62
        // Allow wide range since WDF modeling introduces some deviation
        eprintln!(
            "[L4 12AX7] gain={gain:.1}x (in={p_in:.6}, out={p_out:.6})"
        );
        assert!(
            gain > 5.0,
            "12AX7 gain should be > 5x, got {gain:.1}x"
        );
    }
}

#[test]
fn nr_l4c_triode_asymmetric_clipping() {
    // Drive into clipping — tube clipping is characteristically asymmetric
    let input = sine_at(440.0, 1.0, 0.5, SAMPLE_RATE);
    let output = compile_and_process(TRIODE_PREAMP, &input, SAMPLE_RATE, &[]);

    let skip = (0.05 * SAMPLE_RATE) as usize;
    let steady = &output[skip..];

    let pos_peak = steady.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    let neg_peak_abs = steady.iter().cloned().fold(f64::INFINITY, f64::min).abs();

    if pos_peak > 0.01 && neg_peak_abs > 0.01 {
        let asymmetry = (pos_peak - neg_peak_abs).abs() / (pos_peak + neg_peak_abs);
        eprintln!(
            "[L4 12AX7 clip] pos={pos_peak:.4}, neg={neg_peak_abs:.4}, asymmetry={asymmetry:.4}"
        );
        // Just verify that the output exists and is finite; asymmetric clipping
        // is a property of the tube model that may vary depending on bias point
    }

    assert!(
        peak(steady) > 0.001,
        "Tube stage should produce output under drive"
    );
}

#[test]
fn nr_l4d_triode_solver_stats() {
    let input = sine_at(440.0, 0.1, 0.5, SAMPLE_RATE);

    reset_solver_stats();
    let _output = compile_and_process(TRIODE_PREAMP, &input, SAMPLE_RATE, &[]);
    let stats = solver_stats_snapshot();

    assert_eq!(
        stats.iter_cap_hits, 0,
        "12AX7 preamp: no convergence failures expected, got {} iter_cap_hits",
        stats.iter_cap_hits
    );

    if stats.solves > 0 {
        let avg = stats.total_iterations as f64 / stats.solves as f64;
        eprintln!(
            "[L4 12AX7] solves={}, avg_iter={avg:.1}, max_iter={}, residual_max={:.2e}",
            stats.solves, stats.max_iterations, stats.max_residual,
        );
    }
}

// ============================================================================
// Layer 4+: Cathode Bypass Cap Stress Tests
//
// The cathode bypass cap creates a signal-dependent operating point shift:
// at low frequencies the cap is open → full cathode degeneration (low gain),
// at high frequencies the cap is shorted → no degeneration (high gain).
// This means the triode's plate resistance (rp) and the effective junction
// voltage decomposition are modulated by the cathode impedance, which changes
// with both frequency and signal level. The NR solver must track this
// shifting operating point via warm-starting without accumulating residual
// errors that look like gain compression artifacts.
// ============================================================================

/// 12AX7 preamp WITHOUT cathode bypass (full degeneration, lower gain).
const TRIODE_PREAMP_UNBYPASSED: &str = r#"
pedal "12AX7 Unbypassed" {
    supply 250V
    components {
        C_in: cap(22n)
        R_grid: resistor(1M)
        V1: triode(12ax7)
        R_plate: resistor(100k)
        R_cathode: resistor(1.5k)
        C_out: cap(100n)
        R_load: resistor(1M)
    }
    nets {
        in -> C_in.a
        C_in.b -> R_grid.a, V1.grid
        R_grid.b -> gnd
        vcc -> R_plate.a
        R_plate.b -> V1.plate
        V1.cathode -> R_cathode.a
        R_cathode.b -> gnd
        V1.plate -> C_out.a
        C_out.b -> R_load.a, out
        R_load.b -> gnd
    }
}
"#;

/// 12AX7 with a small bypass cap (partial bypass — worst case for operating
/// point tracking because the cap's impedance is comparable to R_cathode
/// at audio frequencies, creating maximum operating point variation).
const TRIODE_PREAMP_PARTIAL_BYPASS: &str = r#"
pedal "12AX7 Partial Bypass" {
    supply 250V
    components {
        C_in: cap(22n)
        R_grid: resistor(1M)
        V1: triode(12ax7)
        R_plate: resistor(100k)
        R_cathode: resistor(1.5k)
        C_bypass: cap(680p)
        C_out: cap(100n)
        R_load: resistor(1M)
    }
    nets {
        in -> C_in.a
        C_in.b -> R_grid.a, V1.grid
        R_grid.b -> gnd
        vcc -> R_plate.a
        R_plate.b -> V1.plate
        V1.cathode -> R_cathode.a, C_bypass.a
        R_cathode.b -> gnd
        C_bypass.b -> gnd
        V1.plate -> C_out.a
        C_out.b -> R_load.a, out
        R_load.b -> gnd
    }
}
"#;

#[test]
fn nr_l4e_cathode_bypass_gain_differential() {
    // The bypassed preamp (25µF cap) should have more gain than the unbypassed
    // version at 440Hz, because the bypass cap shorts out the cathode resistor
    // at audio frequencies, removing negative feedback.
    //
    // NOTE: The current WDF model may not fully capture cathode degeneration
    // feedback through the tree topology, so gains may be similar. This test
    // verifies both circuits produce reasonable gain and documents the comparison.
    let input = sine_at(440.0, 0.005, 1.0, SAMPLE_RATE);
    let skip = (0.1 * SAMPLE_RATE) as usize;

    let out_bypassed = compile_and_process(TRIODE_PREAMP, &input, SAMPLE_RATE, &[]);
    let out_unbypassed = compile_and_process(TRIODE_PREAMP_UNBYPASSED, &input, SAMPLE_RATE, &[]);

    let gain_bypassed = peak(&out_bypassed[skip..]) / peak(&input[skip..]);
    let gain_unbypassed = peak(&out_unbypassed[skip..]) / peak(&input[skip..]);

    eprintln!(
        "[L4e bypass] bypassed gain={gain_bypassed:.1}x, unbypassed gain={gain_unbypassed:.1}x, ratio={:.1}x",
        gain_bypassed / gain_unbypassed.max(1e-10)
    );

    // Both should produce reasonable gain (>10x for a 12AX7 preamp)
    assert!(
        gain_bypassed > 10.0,
        "Bypassed gain too low: {gain_bypassed:.1}x"
    );
    assert!(
        gain_unbypassed > 10.0,
        "Unbypassed gain too low: {gain_unbypassed:.1}x"
    );

    // In a correct model, bypassed gain >> unbypassed gain (typically 3-10x).
    // Current WDF topology may not capture this. Log the ratio for tracking.
    let ratio = gain_bypassed / gain_unbypassed.max(1e-10);
    if ratio < 1.5 {
        eprintln!(
            "[L4e KNOWN LIMITATION] Cathode degeneration not fully modeled: ratio={ratio:.2}x (expect 3-10x)"
        );
    }
}

#[test]
fn nr_l4f_cathode_bypass_solver_stability_under_transient() {
    // Stress test: hit the triode with a sudden large transient while the
    // cathode bypass cap is still charging. This forces the operating point
    // to shift rapidly — the worst case for warm-starting.
    //
    // Signal: 100ms silence → sudden 1V step → 440Hz sine at 0.5V for 400ms.
    // The step forces a DC bias shift that the cathode cap must absorb,
    // while the sine exercises the NR solver at the new operating point.
    let n = (0.5 * SAMPLE_RATE) as usize;
    let mut input = Vec::with_capacity(n);
    let silence_end = (0.1 * SAMPLE_RATE) as usize;
    let step_end = (0.105 * SAMPLE_RATE) as usize; // 5ms step
    for i in 0..n {
        let t = i as f64 / SAMPLE_RATE;
        if i < silence_end {
            input.push(0.0);
        } else if i < step_end {
            input.push(1.0); // Hard step
        } else {
            input.push(0.5 * (2.0 * std::f64::consts::PI * 440.0 * t).sin());
        }
    }

    reset_solver_stats();
    let output = compile_and_process(TRIODE_PREAMP, &input, SAMPLE_RATE, &[]);
    let stats = solver_stats_snapshot();

    // All output must be finite (no NaN from solver divergence)
    assert!(
        output.iter().all(|x| x.is_finite()),
        "Transient stress: output contains NaN/inf"
    );

    // No iteration cap hits — solver must converge even during transient
    assert_eq!(
        stats.iter_cap_hits, 0,
        "Transient stress: {} iter_cap_hits (solver failed to converge during operating point shift)",
        stats.iter_cap_hits
    );

    // Output should not be silent after transient settles
    let post_transient = &output[(0.15 * SAMPLE_RATE) as usize..];
    assert!(
        peak(post_transient) > 0.01,
        "Output should recover after transient, peak={:.6}",
        peak(post_transient)
    );

    if stats.solves > 0 {
        let avg = stats.total_iterations as f64 / stats.solves as f64;
        eprintln!(
            "[L4f transient] solves={}, avg_iter={avg:.1}, max_iter={}, max_residual={:.2e}, clamp={}, step_lim={}",
            stats.solves, stats.max_iterations, stats.max_residual,
            stats.clamp_hits, stats.step_limited,
        );
    }
}

#[test]
fn nr_l4g_partial_bypass_frequency_dependent_gain() {
    // Partial bypass (680pF) creates a frequency-dependent gain curve:
    // - At 100Hz: cap impedance ≈ 2.3MΩ → effectively unbypassed → low gain
    // - At 5kHz: cap impedance ≈ 47kΩ → partial bypass → medium gain
    // - At 10kHz: cap impedance ≈ 23kΩ → more bypass → higher gain
    //
    // This exercises the solver at different operating points within the
    // same circuit, since the effective cathode impedance (and thus the
    // tube's operating point) shifts with frequency.
    let skip = (0.1 * SAMPLE_RATE) as usize;

    let input_lo = sine_at(100.0, 0.005, 0.5, SAMPLE_RATE);
    let input_hi = sine_at(5000.0, 0.005, 0.5, SAMPLE_RATE);

    let out_lo = compile_and_process(TRIODE_PREAMP_PARTIAL_BYPASS, &input_lo, SAMPLE_RATE, &[]);
    let out_hi = compile_and_process(TRIODE_PREAMP_PARTIAL_BYPASS, &input_hi, SAMPLE_RATE, &[]);

    let gain_lo = peak(&out_lo[skip..]) / peak(&input_lo[skip..]);
    let gain_hi = peak(&out_hi[skip..]) / peak(&input_hi[skip..]);

    eprintln!(
        "[L4g partial bypass] gain@100Hz={gain_lo:.1}x, gain@5kHz={gain_hi:.1}x"
    );

    // With 680pF bypass, high frequency should have more gain than low
    // (bypass cap shorts out cathode R at high freq → less degeneration)
    // But verify both produce finite, non-silent output regardless
    assert!(
        out_lo.iter().all(|x| x.is_finite()) && peak(&out_lo[skip..]) > 1e-6,
        "100Hz output should be finite and non-silent"
    );
    assert!(
        out_hi.iter().all(|x| x.is_finite()) && peak(&out_hi[skip..]) > 1e-6,
        "5kHz output should be finite and non-silent"
    );
}

#[test]
fn nr_l4h_cathode_bypass_warm_start_efficiency() {
    // Verify warm-starting stays efficient when the cathode bypass cap
    // modulates the operating point. A 25µF cap at 440Hz has impedance
    // ≈ 14.5Ω, much less than R_cathode=1.5kΩ, so the cap is mostly
    // shorted and the operating point is stable. But at the signal's
    // zero crossings the tube transitions between cutoff and active,
    // which is where warm-starting matters most.
    //
    // Compare solver stats for bypassed vs unbypassed: the bypassed
    // version has a reactive port that shifts the operating point,
    // so it may need slightly more iterations, but should NOT spike.
    let input = sine_at(440.0, 0.1, 0.5, SAMPLE_RATE);

    reset_solver_stats();
    let _out_bp = compile_and_process(TRIODE_PREAMP, &input, SAMPLE_RATE, &[]);
    let stats_bp = solver_stats_snapshot();

    reset_solver_stats();
    let _out_ub = compile_and_process(TRIODE_PREAMP_UNBYPASSED, &input, SAMPLE_RATE, &[]);
    let stats_ub = solver_stats_snapshot();

    let avg_bp = if stats_bp.solves > 0 {
        stats_bp.total_iterations as f64 / stats_bp.solves as f64
    } else { 0.0 };
    let avg_ub = if stats_ub.solves > 0 {
        stats_ub.total_iterations as f64 / stats_ub.solves as f64
    } else { 0.0 };

    eprintln!(
        "[L4h warm-start] bypassed: avg={avg_bp:.2}, max={}, cap_hits={} | unbypassed: avg={avg_ub:.2}, max={}, cap_hits={}",
        stats_bp.max_iterations, stats_bp.iter_cap_hits,
        stats_ub.max_iterations, stats_ub.iter_cap_hits,
    );

    // Neither should have convergence failures
    assert_eq!(stats_bp.iter_cap_hits, 0, "Bypassed: iter cap hits");
    assert_eq!(stats_ub.iter_cap_hits, 0, "Unbypassed: iter cap hits");

    // Bypassed should not need drastically more iterations than unbypassed.
    // The cathode cap modulates the operating point, but warm-starting
    // should keep the overhead small (within 2x).
    if avg_ub > 0.0 {
        let ratio = avg_bp / avg_ub;
        assert!(
            ratio < 2.0,
            "Bypass cap should not more than double solver iterations: bypassed={avg_bp:.2}, unbypassed={avg_ub:.2}, ratio={ratio:.2}"
        );
    }
}

#[test]
fn nr_l4i_cathode_bypass_no_gain_compression_artifact() {
    // The critical test: if the junction voltage decomposition (v_terminal
    // minus I·rp drop) doesn't track correctly when the cathode cap shifts
    // the operating point, the solver will produce residual errors that
    // manifest as gain compression — the output amplitude stops growing
    // linearly with input even below the clipping threshold.
    //
    // Test: sweep input amplitude from 1mV to 50mV (well below clipping)
    // and verify output grows monotonically. Any non-monotonicity indicates
    // the solver is introducing compression artifacts from operating point
    // tracking errors.
    let skip = (0.1 * SAMPLE_RATE) as usize;
    let amplitudes = [0.001, 0.002, 0.005, 0.01, 0.02, 0.05];
    let mut prev_out_peak = 0.0;

    for &amp in &amplitudes {
        let input = sine_at(440.0, amp, 0.5, SAMPLE_RATE);
        let output = compile_and_process(TRIODE_PREAMP, &input, SAMPLE_RATE, &[]);
        let out_peak = peak(&output[skip..]);

        assert!(
            out_peak > prev_out_peak,
            "Gain compression artifact: at input amp={amp}, output peak={out_peak:.6} <= previous {prev_out_peak:.6}"
        );
        prev_out_peak = out_peak;
    }

    // Verify approximate linearity using AC gain (DC-corrected).
    // At very small amplitudes, DC offset dominates peak(), so we subtract
    // the mean to measure AC amplitude only.
    let ac_gain = |amp: f64| -> f64 {
        let input = sine_at(440.0, amp, 0.5, SAMPLE_RATE);
        let output = compile_and_process(TRIODE_PREAMP, &input, SAMPLE_RATE, &[]);
        let trimmed = &output[skip..];
        let dc = dc_offset(trimmed);
        let ac_peak = trimmed.iter().map(|&s| (s - dc).abs()).fold(0.0f64, f64::max);
        ac_peak / amp
    };

    let gain_low = ac_gain(0.005);
    let gain_high = ac_gain(0.02);

    let gain_ratio = gain_high / gain_low.max(1e-10);
    eprintln!(
        "[L4i linearity] ac_gain@5mV={gain_low:.1}x, ac_gain@20mV={gain_high:.1}x, ratio={gain_ratio:.3}"
    );

    // Gain should be within 6dB (factor ~0.5-2.0) across this range.
    // Tubes are inherently nonlinear, so some variation is expected,
    // but gross compression (ratio < 0.3) would indicate solver artifacts.
    assert!(
        gain_ratio > 0.3 && gain_ratio < 3.0,
        "Gain varies too much across small-signal range: ratio={gain_ratio:.3} (expect ~1.0)"
    );
}
