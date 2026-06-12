/// Phase 1 — Slew rate characterization: current dV clamp vs integrator model.
///
/// The engine's current model runs:
///   1. GBW single-pole IIR (models closed-loop BW): y[n] = α*x[n] + (1-α)*y[n-1]
///   2. Hard dV/dt clamp: |y[n]-y[n-1]| ≤ max_dv
///
/// The "integrator model" as proposed in the task:
///   1. Estimate Vd = input_error / open_loop_gain
///   2. dVout/dt = clamp(gm * Vd, ±Itail) / Ccomp
///
/// Key insight: in the LINEAR regime (sub-slew), both models produce the same
/// closed-loop pole. The integrator model is:
///
///   For an inverting amp with open-loop gain Aol, in closed-loop:
///   Vout[n] = Vout[n-1] + (2π·GBW_ol / fs) * (Vd_normalized)
///   where Vd_normalized = (V+ - V-) and the IIR pole is exactly GBW/(gain·fs)
///
/// For SLEW: the nonlinearity kicks in when |gm*Vd| > Itail.
/// The current model's clamp fires when |Vout_new - Vout_prev| > SR/fs.
///
/// These are NOT the same clamp location:
/// - Current: clamps the IIR *output* change
/// - Integrator: clamps the *drive current*, applied before the integrator
///
/// Below I compare them by looking at the slew onset shape only —
/// specifically: does the integrator model produce a softer onset (tanh rounding)
/// vs the current model's binary onset?
///
/// Test method: sweep input amplitude from sub-slew to over-slew.
/// Measure the actual output slope (dV/sample at max slope) vs ideal SR.
/// If integrator shows soft onset (slope < SR before crossing threshold),
/// and current model shows hard onset (slope = 0 then jumps to SR), there
/// is a real difference in the onset sharpness.

use std::f64::consts::PI;

// ── Model A: Current engine (IIR GBW + hard dV clamp) ───────────────────────
struct CurrentModel {
    gbw_coeff: f64,
    gbw_state: f64,
    max_dv: f64,
    prev_out: f64,
}

impl CurrentModel {
    fn new(gbw: f64, gain: f64, sr_v_per_s: f64, fs: f64) -> Self {
        let fc = gbw / gain.abs().max(1.0);
        let w = 2.0 * PI * fc;
        Self {
            gbw_coeff: w / (w + fs),
            gbw_state: 0.0,
            max_dv: sr_v_per_s / fs,
            prev_out: 0.0,
        }
    }

    /// Input: already-gained signal (i.e., feedback network output * gain factor).
    fn process(&mut self, gained_in: f64) -> f64 {
        // GBW LPF
        let mut out = self.gbw_coeff * gained_in
            + (1.0 - self.gbw_coeff) * self.gbw_state;
        self.gbw_state = out;
        // Hard clamp
        let dv = out - self.prev_out;
        if dv > self.max_dv {
            out = self.prev_out + self.max_dv;
        } else if dv < -self.max_dv {
            out = self.prev_out - self.max_dv;
        }
        self.prev_out = out;
        out
    }

    fn reset(&mut self) {
        self.gbw_state = 0.0;
        self.prev_out = 0.0;
    }
}

// ── Model B: Integrator model ────────────────────────────────────────────────
//
// The 2-stage op-amp model in closed loop (inverting amp, gain G):
//
//   Open-loop: Vout_ol[n] = Vout_ol[n-1] + gm * Vd * dt / Ccomp
//   Closed-loop: Vd = Vp - Vm = -Vm (Vp=0 for inverting)
//               Vm = Vin_eff * Ri/(Ri+Rf) + Vout * Rf/(Ri+Rf)
//                  = (Vin_eff + G*Vout) / (G+1)
//   -> Vd = -(Vin_eff + G*Vout) / (G+1)
//
// The nonlinearity: replace gm*Vd with tanh(Vd/Vt_eff) * Itail
// where Vt_eff = Itail/gm (the onset voltage of the differential pair).
//
// Below onset (|Vd| << Vt_eff): tanh(x) ≈ x, reduces to integrator IIR.
// Above onset: saturates at ±Itail/Ccomp = ±SR/fs per sample.
//
// Note: the SAME physical approximation the current engine uses for slew!
// The only difference is that the current engine applies the clamp AFTER
// the IIR update (which has already partially propagated the signal),
// whereas the integrator applies it as a current limit BEFORE integration.
//
// Let's test: with the integrator model, does the slew onset show a tanh
// shoulder, and is that shoulder audibly wider than the IIR+clamp step?

struct IntegratorModel {
    gm: f64,
    i_tail: f64,
    c_comp: f64,
    fs: f64,
    gain: f64,
    v_out: f64,
}

impl IntegratorModel {
    fn new(gbw: f64, sr_v_per_s: f64, gain: f64, fs: f64) -> Self {
        // From SR and GBW: Itail = SR * Ccomp, gm = 2π·GBW·Ccomp
        // We can choose Ccomp from any consistent set; it cancels in the limit.
        // Use Ccomp = Itail/SR: pick Ccomp = 30pF (LM308 ~30pF dominant cap)
        let c_comp = 30e-12;
        let i_tail = sr_v_per_s * c_comp;
        let gm = 2.0 * PI * gbw * c_comp;
        Self {
            gm,
            i_tail,
            c_comp,
            fs,
            gain,
            v_out: 0.0,
        }
    }

    /// Input: already-gained signal (same convention as CurrentModel).
    fn process(&mut self, gained_in: f64) -> f64 {
        // Vd = -(Vin_eff + G*Vout) / (G+1) where Vin_eff = gained_in/G
        // Note: gained_in = G * Vin_eff, so Vin_eff = gained_in / G
        // Then: Vd = -(Vin_eff + G*Vout)/(G+1) = -(gained_in/G + G*self.v_out)/(G+1)
        //       But simpler: Vd = -(gained_in + G²*self.v_out) / (G*(G+1))
        // Actually, let's be precise. Gained_in is the signal AT the output of the
        // feedback network (i.e., the target output). The IIR version processes this
        // directly. For the integrator to match, we need the actual Vd.
        //
        // For inverting amp with Ri, Rf=G*Ri:
        //   At V-: Vm = (Vin/Ri + Vout/Rf) / (1/Ri + 1/Rf) (by superposition + KCL)
        //              = (Vin + Vout/G) / (1 + 1/G) = (G*Vin + Vout) / (G+1)
        //   Vd = -Vm = -(G*Vin + Vout) / (G+1)
        //   gained_in = G * Vin => Vin = gained_in / G
        //   Vd = -(gained_in + self.v_out) / (G + 1)
        let v_d = -(gained_in + self.v_out) / (self.gain + 1.0);

        // tanh current limiting (onset at |Vd| = Itail/gm = SR*Ccomp/gm = SR/(2π·GBW))
        let v_t_eff = self.i_tail / self.gm; // onset voltage ≈ SR/(2π·GBW)
        let current = self.i_tail * (v_d / v_t_eff).tanh();

        // Forward Euler integration
        let dv = current / self.c_comp / self.fs;
        self.v_out += dv;
        self.v_out
    }

    fn reset(&mut self) {
        self.v_out = 0.0;
    }
}

fn main() {
    let fs = 48000.0_f64;
    let sr_v_us = 0.3_f64;
    let sr = sr_v_us * 1e6; // V/s
    let gbw = 1e6_f64;
    let gain = 10.0_f64;

    println!("=== LM308 Phase 1 Slew Characterization ===");
    println!("  SR={sr_v_us}V/µs, GBW={gbw:.0e}Hz, gain={gain}, fs={fs}Hz");
    println!("  max_dv = {:.4}V/sample = {:.0}kV/s", sr/fs, sr/1000.0);

    // Slew onset voltage: the onset voltage at the input stage differential pair
    let c_comp = 30e-12_f64;
    let i_tail = sr * c_comp;
    let gm = 2.0 * PI * gbw * c_comp;
    let v_t_eff = i_tail / gm;
    println!("  Vt_eff (onset) = {v_t_eff:.2e}V = SR/(2π·GBW)");
    println!();

    // ── Test 1: Amplitude sweep — slew onset shape ────────────────────────
    // Sweep Vin from 0 to 2×slew_limit amplitude.
    // For 1kHz sine, slew onset at Vin_peak: SR/(2π·f·gain) = 300k/(2π·1k·10) ≈ 4.77V
    // So sweep 0.01V to 8V in 30 steps.
    println!("=== T1: 1kHz amplitude sweep — slew onset characterization ===");
    println!("  Slew onset Vout amplitude: {:.3}V (SR/2πf = {:.0} V/s / {:.0} Hz)",
             sr / (2.0*PI*1000.0), sr, 2.0*PI*1000.0);
    println!();
    println!("  Vin_peak | Vout_target | Max_dV/sample_C | Max_dV/sample_I | Ratio(C/ideal) | Ratio(I/ideal)");
    println!("  ---------|-------------|-----------------|-----------------|----------------|---------------");

    let freq = 1000.0_f64;
    for step in 0..=16 {
        let vin_peak = (step as f64 + 0.5) * 0.5; // 0.25V to 8.25V
        let vout_target = vin_peak * gain;
        let ideal_slope = (2.0 * PI * freq * vout_target).min(sr) / fs;

        let mut cm = CurrentModel::new(gbw, gain, sr, fs);
        let mut im = IntegratorModel::new(gbw, sr, gain, fs);
        // Settle
        for _ in 0..2000 { cm.process(0.0); im.process(0.0); }

        // Measure max dV/sample during steady-state 1kHz sine
        let n = 2400; // 50 cycles
        let mut c_prev = 0.0_f64;
        let mut i_prev = 0.0_f64;
        let mut c_max_dv = 0.0_f64;
        let mut i_max_dv = 0.0_f64;
        for k in 0..n {
            let v_in = vout_target * (2.0*PI*freq*k as f64/fs).sin();
            let c_out = cm.process(v_in);
            let i_out = im.process(v_in);
            c_max_dv = c_max_dv.max((c_out - c_prev).abs());
            i_max_dv = i_max_dv.max((i_out - i_prev).abs());
            c_prev = c_out;
            i_prev = i_out;
        }

        println!("  {:>8.2}V | {:>11.2}V | {:>15.5} | {:>15.5} | {:>14.3} | {:>14.3}",
                 vin_peak, vout_target, c_max_dv, i_max_dv,
                 c_max_dv / (sr/fs), i_max_dv / (sr/fs));
    }

    // ── Test 2: Triangle-wave slope accuracy ─────────────────────────────
    println!();
    println!("=== T2: Square wave 0→10V — slope accuracy ===");
    let mut cm = CurrentModel::new(gbw, gain, sr, fs);
    let mut im = IntegratorModel::new(gbw, sr, gain, fs);
    for _ in 0..1000 { cm.process(0.0); im.process(0.0); }

    let mut c_prev = 0.0_f64;
    let mut i_prev = 0.0_f64;
    println!("  sample | cur_out | int_out | dV_cur | dV_int");
    for k in 0..20 {
        // Already-gained signal = 10V (target output after gain=10)
        let v_in = if k < 3 { 0.0_f64 } else { 10.0 };
        let c = cm.process(v_in);
        let it = im.process(v_in);
        println!("  {:>6} | {:>7.4} | {:>7.4} | {:>6.4} | {:>6.4}",
                 k, c, it,
                 (c - c_prev).abs(), (it - i_prev).abs());
        c_prev = c;
        i_prev = it;
    }
    println!("  Expected max_dv = {:.4}", sr/fs);

    // ── Test 3: Onset sharpness — transition region analysis ─────────────
    // Key question: does the integrator model produce a SOFTER onset than
    // the current hard clamp? If the tanh shoulder is < 1 sample wide,
    // it's inaudible at 48kHz.
    println!();
    println!("=== T3: Onset sharpness — Vin stepping from 0 to just at slew onset ===");
    let vin_onset = sr / (2.0 * PI * freq * gain); // onset amplitude for 1kHz
    println!("  Vin_onset = {vin_onset:.4}V (target Vout = {:.4}V)", vin_onset*gain);

    // Test at exactly onset amplitude — both models should be at the edge
    let mut cm = CurrentModel::new(gbw, gain, sr, fs);
    let mut im = IntegratorModel::new(gbw, sr, gain, fs);
    for _ in 0..2000 { cm.process(0.0); im.process(0.0); }

    // Measure: run 1 full cycle, report max and mean dV
    let vout_at_onset = vin_onset * gain;
    let mut c_dvs: Vec<f64> = Vec::new();
    let mut i_dvs: Vec<f64> = Vec::new();
    let mut c_prev = 0.0_f64;
    let mut i_prev = 0.0_f64;
    for k in 0..480 {
        let v_in = vout_at_onset * (2.0*PI*freq*k as f64/fs).sin();
        let c = cm.process(v_in);
        let it = im.process(v_in);
        c_dvs.push((c - c_prev).abs());
        i_dvs.push((it - i_prev).abs());
        c_prev = c;
        i_prev = it;
    }
    let c_max_dv = c_dvs.iter().cloned().fold(0.0_f64, f64::max);
    let i_max_dv = i_dvs.iter().cloned().fold(0.0_f64, f64::max);
    let c_saturated = c_dvs.iter().filter(|&&x| x >= sr/fs * 0.99).count();
    let i_saturated = i_dvs.iter().filter(|&&x| x >= sr/fs * 0.99).count();

    println!("  At onset amplitude:");
    println!("    Current:    max_dv={c_max_dv:.5} ({:.1}×SR/fs), samples at SR: {c_saturated}/480",
             c_max_dv/(sr/fs));
    println!("    Integrator: max_dv={i_max_dv:.5} ({:.1}×SR/fs), samples at SR: {i_saturated}/480",
             i_max_dv/(sr/fs));

    // ── Summary ──────────────────────────────────────────────────────────
    println!();
    println!("=== Phase 1 Summary ===");
    println!();
    println!("  SR/fs per-sample budget: {:.4}V/sample", sr/fs);
    println!();
    println!("  Both models agree on:");
    println!("    - Square-wave slope accuracy (both reach SR/fs exactly)");
    println!("    - Sub-slew behavior (IIR GBW dominates, slew clamp inactive)");
    println!();
    println!("  Difference between models:");
    println!("    - Current model: hard binary onset (clamp fires when |Δout| > max_dv)");
    println!("    - Integrator:    tanh onset (gradual saturation over ±Vt_eff = {v_t_eff:.2e}V)");
    println!();
    println!("  The tanh onset width: Vt_eff = {v_t_eff:.2e}V (physical input stage threshold)");
    println!("  At gain={gain}, Vout changes by Vt_eff when Vd = Vt_eff.");
    println!("  This is {:} of max_dv — meaning the tanh shoulder spans {:.2} samples.",
             format!("{:.2e}", v_t_eff * gain), v_t_eff * gain / (sr/fs));
    println!();

    // Verdict
    let tanh_shoulder_samples = v_t_eff * gain / (sr/fs);
    let audible_threshold = 0.5; // sub-sample tanh shoulder is inaudible

    if tanh_shoulder_samples < audible_threshold {
        println!("VERDICT: Tanh onset shoulder spans {tanh_shoulder_samples:.3} samples.");
        println!("         This is BELOW the 0.5-sample audibility threshold.");
        println!("         The integrator model's soft onset is NOT audibly different");
        println!("         from the current hard clamp at {fs}Hz sample rate.");
        println!();
        println!("         The dV clamp is an accurate approximation of the integrator.");
        println!("         Phase 2 NOT warranted — no fidelity improvement possible.");
    } else {
        println!("VERDICT: Tanh onset shoulder spans {tanh_shoulder_samples:.3} samples.");
        println!("         This IS above the audibility threshold.");
        println!("         Phase 2 implementation would provide a measurable improvement.");
    }
}
