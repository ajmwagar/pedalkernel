//! Spring reverb tank model (Välimäki/Parker parametric structure).
//!
//! A spring's impulse response is a train of repeating **chirps**: a pulse
//! transits the spring (~30–40 ms) and dispersion makes high frequencies
//! arrive *later* than low frequencies (positive group-delay dispersion below
//! the transition frequency fC). We model this with the parametric structure
//! from Välimäki, Parker & Abel, *Parametric Spring Reverberation Effect*,
//! JAES 58(7/8), 2010 (AES e-lib 15511) and Parker's DAFx follow-up
//! *Efficient Dispersion Generation Structures for Spring Reverb Emulation*:
//!
//! Per spring line, inside a feedback loop:
//!   1. a cascade of **M identical stretched first-order allpasses** —
//!      `y[n] = −a·x[n] + x[n−K] + a·y[n−K]` (Schroeder allpass stretched by
//!      K = fs/(2·fC) to pack the dispersion into the audio band; group delay
//!      is maximal at DC, so LOW frequencies are delayed more than high — the
//!      spring chirp). Each section keeps its own K-sample state ring, so a
//!      section costs one MAC + two buffer reads.
//!   2. a **loop delay** Td (the spring transit time, fractional via linear
//!      interpolation);
//!   3. a **damping one-pole LPF** (darkens the tail as it recirculates);
//!   4. **loop feedback** g derived from the model's RT60:
//!      `g = 10^(−3·Td/RT60)` (negative — alternating echo polarity).
//!
//! Springs in a tank are slightly detuned (different transit times) to
//! decorrelate; their outputs are summed. A 40 Hz DC blocker sits on the
//! output.
//!
//! ## Realtime contract
//!
//! Constant per-sample cost, allocation-free `process()`, bounded state. All
//! buffers are sized in [`SpringTank::new`]; `process()` does no alloc, no
//! locks, no syscalls (same standard as [`super::bbd::BbdDelayLine::process`]).
//! A NaN/Inf guard flushes loop state, mirroring the delay-line allpass guard.
//!
//! The HF-chirp cascade (the weak chirp above fC, JAES §II) is intentionally
//! **not** in the hot path (realtime budget — OQ3); the field/flag is reserved
//! and documented but the low-chirp path is what sounds.

use alloc::vec;
use alloc::vec::Vec;

/// Maximum springs in any tank.
const MAX_SPRINGS: usize = 6;

/// Hard cap on stretched-allpass sections per spring line (bounds state).
const MAX_ALLPASSES: usize = 128;

/// Named spring-tank model parameters (Accutronics / RE-201 grounded).
///
/// Transit times, fC and RT60 are partly inferred (the primary datasheet
/// tables were 403-blocked at authoring time — flagged `[A]` in the example
/// provenance) and chosen from the published-class ranges in the design doc.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct SpringModel {
    /// Number of spring lines summed at the output.
    pub num_springs: usize,
    /// Per-spring transit time (ms) — the loop delay Td of each line. Slightly
    /// detuned across springs to decorrelate. `num_springs` entries are used.
    pub transit_ms: [crate::Wave; MAX_SPRINGS],
    /// Dispersion transition frequency fC (Hz): group delay disperses below it.
    pub fc_hz: crate::Wave,
    /// Reverberation time RT60 (s) — sets the loop feedback gain.
    pub rt60_s: crate::Wave,
    /// Number of stretched allpasses per spring (dispersion strength ∝ M·K).
    /// The JAES reference uses M=100; we ship 32 (≈11 ms group-delay
    /// dispersion across 300 Hz–2.4 kHz, a clear spring chirp) to hold a
    /// comfortable realtime margin (≥30x release — fidelity/cost OQ3).
    pub num_allpasses: usize,
    /// Stretched-allpass coefficient a (≈0.75 per JAES).
    pub ap_coeff: crate::Wave,
    /// Default damping-LPF cutoff (Hz) — how fast the tail darkens.
    pub damping_fc_hz: crate::Wave,
    /// Reserved: HF-chirp cascade gain (OFF / 0.0 in v1 — OQ3, not hot path).
    pub hf_chirp_gain: crate::Wave,
}

impl SpringModel {
    /// Accutronics Type 4 — 17" tank, Fender standard, long decay.
    /// transit ~33/37.5 ms [A], fC 4200 Hz, RT60 2.85 s.
    pub fn type4() -> Self {
        Self {
            num_springs: 2,
            transit_ms: [33.0, 37.5, 0.0, 0.0, 0.0, 0.0],
            fc_hz: 4200.0,
            rt60_s: 2.85,
            num_allpasses: 32,
            ap_coeff: 0.75,
            damping_fc_hz: 4000.0,
            hf_chirp_gain: 0.0,
        }
    }

    /// Accutronics Type 8 — 9.25" short tank, 3 springs, shorter decay.
    /// transit ~30/32/34 ms [A], fC 3500 Hz, RT60 1.8 s.
    pub fn type8() -> Self {
        Self {
            num_springs: 3,
            transit_ms: [30.0, 32.0, 34.0, 0.0, 0.0, 0.0],
            fc_hz: 3500.0,
            rt60_s: 1.8,
            num_allpasses: 32,
            ap_coeff: 0.75,
            damping_fc_hz: 3200.0,
            hf_chirp_gain: 0.0,
        }
    }

    /// Accutronics Type 9 — 17", 6-spring (3 coupled pairs), densest.
    /// transit ~33/36.5/40 ms [A], fC 4000 Hz, RT60 2.5 s.
    pub fn type9() -> Self {
        Self {
            num_springs: 3,
            transit_ms: [33.0, 36.5, 40.0, 0.0, 0.0, 0.0],
            fc_hz: 4000.0,
            rt60_s: 2.5,
            num_allpasses: 32,
            ap_coeff: 0.75,
            damping_fc_hz: 3800.0,
            hf_chirp_gain: 0.0,
        }
    }

    /// Roland RE-201 tank — small dark 2-spring tank, fast "boing".
    /// transit ~31/35 ms [A], fC 3000 Hz, RT60 1.6 s.
    pub fn re201_tank() -> Self {
        Self {
            num_springs: 2,
            transit_ms: [31.0, 35.0, 0.0, 0.0, 0.0, 0.0],
            fc_hz: 3000.0,
            rt60_s: 1.6,
            num_allpasses: 32,
            ap_coeff: 0.75,
            damping_fc_hz: 2800.0,
            hf_chirp_gain: 0.0,
        }
    }
}

/// One dispersive spring line: a stretched-allpass cascade in a feedback loop
/// with a transit delay and damping LPF. All buffers sized in `new`.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
struct SpringLine {
    /// Active stretched-allpass sections (≤ MAX_ALLPASSES).
    num_ap: usize,
    /// Stretched-allpass coefficient a.
    ap_coeff: crate::Wave,
    /// Integer stretch K (samples) per allpass section.
    stretch_k: usize,
    /// Per-section input history rings, length K each, packed contiguously:
    /// section s owns `ap_x[s*K .. (s+1)*K]`. Holds x[n-K].
    ap_x: Vec<crate::Wave>,
    /// Per-section output history rings (holds y[n-K]).
    ap_y: Vec<crate::Wave>,
    /// Rolling write index within each section's K-length ring (shared; the
    /// same modular position is used for every section).
    ap_idx: usize,
    /// Transit (loop) delay ring buffer; read at `transit_samples` fractional.
    loop_buf: Vec<crate::Wave>,
    /// Loop delay write position.
    loop_pos: usize,
    /// Transit delay in (fractional) samples.
    transit_samples: crate::Wave,
    /// Transit time in seconds (for RT60→gain recompute under control).
    transit_s: crate::Wave,
    /// Damping one-pole LPF state + coefficient.
    damping_state: crate::Wave,
    damping_coef: crate::Wave,
    /// Loop feedback gain (negative — alternating polarity).
    loop_gain: crate::Wave,
}

impl SpringLine {
    fn new(
        transit_ms: crate::Wave,
        num_ap: usize,
        ap_coeff: crate::Wave,
        stretch_k: usize,
        rt60_s: crate::Wave,
        damping_fc_hz: crate::Wave,
        sample_rate: crate::Wave,
    ) -> Self {
        let num_ap = num_ap.clamp(1, MAX_ALLPASSES);
        let stretch_k = stretch_k.max(1);
        let transit_s = transit_ms * 1e-3;
        let transit_samples = transit_s * sample_rate;
        let loop_len = (transit_samples as usize) + 4;

        let damping_coef = crate::math::exp(
            (-2.0 * crate::math::PI as crate::Wave * damping_fc_hz / sample_rate) as crate::Wave,
        ) as crate::Wave;

        let loop_gain = -(rt60_to_gain(transit_s, rt60_s));

        Self {
            num_ap,
            ap_coeff,
            stretch_k,
            ap_x: vec![0.0; num_ap * stretch_k],
            ap_y: vec![0.0; num_ap * stretch_k],
            ap_idx: 0,
            loop_buf: vec![0.0; loop_len.max(2)],
            loop_pos: 0,
            transit_samples,
            transit_s,
            damping_state: 0.0,
            damping_coef,
            loop_gain,
        }
    }

    fn set_damping_fc(&mut self, fc_hz: crate::Wave, sample_rate: crate::Wave) {
        let fc = fc_hz.clamp(200.0, sample_rate * 0.45);
        self.damping_coef = crate::math::exp(
            (-2.0 * crate::math::PI as crate::Wave * fc / sample_rate) as crate::Wave,
        ) as crate::Wave;
    }

    fn set_rt60(&mut self, rt60_s: crate::Wave) {
        self.loop_gain = -(rt60_to_gain(self.transit_s, rt60_s));
    }

    /// Process one sample through this spring line.
    #[inline]
    fn process(&mut self, input: crate::Wave) -> crate::Wave {
        // ── Loop sum: input + feedback of the damped, transit-delayed loop ──
        let delayed = self.read_loop();
        let damped = self.damping_coef * self.damping_state + (1.0 - self.damping_coef) * delayed;
        self.damping_state = damped;
        let mut x = input + self.loop_gain * damped;

        // ── Stretched-allpass cascade (dispersion) ─────────────────────────
        // section s: y = a·x + x[n-K] − a·y[n-K]; each section keeps its own
        // K-sample ring at the shared rolling index `ap_idx`.
        let a = self.ap_coeff;
        let k = self.stretch_k;
        let idx = self.ap_idx;
        for s in 0..self.num_ap {
            let base = s * k;
            let slot = base + idx;
            let x_km = self.ap_x[slot];
            let y_km = self.ap_y[slot];
            // Stretched Schroeder allpass H(z) = (−a + z^−K)/(1 − a·z^−K):
            //   y[n] = −a·x[n] + x[n−K] + a·y[n−K]  (JAES structure, §4).
            // Group delay is maximal at DC and falls with frequency, so low
            // frequencies are delayed MORE than high — the spring chirp.
            let y = -a * x + x_km + a * y_km;
            // store this section's current input/output at the K-delayed slot.
            self.ap_x[slot] = x;
            self.ap_y[slot] = y;
            x = y;
        }
        self.ap_idx = (idx + 1) % k;

        // ── Transit (loop) delay write ─────────────────────────────────────
        self.write_loop(x);

        // NaN/Inf guard — flush loop state.
        if !x.is_finite() {
            self.damping_state = 0.0;
            for v in &mut self.loop_buf {
                *v = 0.0;
            }
            for v in &mut self.ap_x {
                *v = 0.0;
            }
            for v in &mut self.ap_y {
                *v = 0.0;
            }
            return 0.0;
        }
        x
    }

    #[inline]
    fn read_loop(&self) -> crate::Wave {
        let buf_len = self.loop_buf.len();
        let d = self.transit_samples;
        let di = (d as usize).min(buf_len - 1);
        let frac = d - di as crate::Wave;
        let i0 = (self.loop_pos + buf_len - di) % buf_len;
        let i1 = (i0 + buf_len - 1) % buf_len;
        let x0 = self.loop_buf[i0];
        let x1 = self.loop_buf[i1];
        x0 * (1.0 - frac) + x1 * frac
    }

    #[inline]
    fn write_loop(&mut self, v: crate::Wave) {
        self.loop_buf[self.loop_pos] = v;
        self.loop_pos = (self.loop_pos + 1) % self.loop_buf.len();
    }

    fn reset(&mut self) {
        self.ap_idx = 0;
        self.loop_pos = 0;
        self.damping_state = 0.0;
        for v in &mut self.ap_x {
            *v = 0.0;
        }
        for v in &mut self.ap_y {
            *v = 0.0;
        }
        for v in &mut self.loop_buf {
            *v = 0.0;
        }
    }
}

/// RT60 → loop-gain magnitude: `g = 10^(−3·Td/RT60)`, clamped < 1 for
/// stability. (Caller negates for alternating echo polarity.)
#[inline]
fn rt60_to_gain(transit_s: crate::Wave, rt60_s: crate::Wave) -> crate::Wave {
    let rt60 = rt60_s.max(0.05);
    let g = crate::math::exp(
        (-3.0 * crate::math::LN_10 as crate::Wave * transit_s / rt60) as crate::Wave,
    ) as crate::Wave;
    g.min(0.999)
}

/// Spring reverb tank: a sum of detuned dispersive [`SpringLine`]s with an
/// output DC blocker and a per-instance dwell/drive input trim.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct SpringTank {
    pub model: SpringModel,
    springs: Vec<SpringLine>,
    sample_rate: crate::Wave,
    /// DC blocker (40 Hz HPF) state.
    dc_x1: crate::Wave,
    dc_y1: crate::Wave,
    dc_coef: crate::Wave,
    /// Input dwell/drive trim (1.0 = unity), set by the dwell control.
    dwell: crate::Wave,
    /// Output normalization (1/num_springs).
    out_scale: crate::Wave,
}

impl SpringTank {
    /// Create a spring tank from a named model.
    pub fn new(model: SpringModel, sample_rate: crate::Wave) -> Self {
        // Stretch K = round(fs / (2·fC)) − 1 (JAES integer part). ≥ 1.
        let k = crate::math::round((sample_rate / (2.0 * model.fc_hz)) as crate::Wave) as i64 - 1;
        let stretch_k = (k.max(1)) as usize;

        let n = model.num_springs.clamp(1, MAX_SPRINGS);
        let mut springs = Vec::with_capacity(n);
        for i in 0..n {
            springs.push(SpringLine::new(
                model.transit_ms[i],
                model.num_allpasses,
                model.ap_coeff,
                stretch_k,
                model.rt60_s,
                model.damping_fc_hz,
                sample_rate,
            ));
        }

        let dc_fc = 40.0;
        let dc_coef = crate::math::exp(
            (-2.0 * crate::math::PI as crate::Wave * dc_fc / sample_rate) as crate::Wave,
        ) as crate::Wave;

        Self {
            model,
            sample_rate,
            dc_x1: 0.0,
            dc_y1: 0.0,
            dc_coef,
            dwell: 1.0,
            out_scale: 1.0 / n as crate::Wave,
            springs,
        }
    }

    /// Dwell/drive input trim. Normalized 0..1 → 0..2 (1 = unity at 0.5).
    pub fn set_dwell_normalized(&mut self, norm: crate::Wave) {
        self.dwell = (norm.clamp(0.0, 1.0) * 2.0).max(0.0);
    }

    /// Decay → RT60 authority: rebuilds each line's loop gain from a scaled
    /// RT60 (norm 0..1 → 0.3·rt60 .. 1.6·rt60). Higher = longer tail.
    pub fn set_decay_normalized(&mut self, norm: crate::Wave) {
        let norm = norm.clamp(0.0, 1.0);
        let scale = 0.3 + 1.3 * norm;
        let rt60 = (self.model.rt60_s * scale).max(0.05);
        for spring in &mut self.springs {
            spring.set_rt60(rt60);
        }
    }

    /// Damping cutoff control: norm 0..1 → base/4 .. base·2 (dark→bright).
    pub fn set_damping_normalized(&mut self, norm: crate::Wave) {
        let norm = norm.clamp(0.0, 1.0);
        let base = self.model.damping_fc_hz;
        // log interpolation: base · 4^(norm − 0.5) wait — explicit endpoints.
        let lo = base * 0.25;
        let hi = base * 2.0;
        let log_lo = crate::math::ln(lo as crate::Wave) as crate::Wave;
        let log_hi = crate::math::ln(hi as crate::Wave) as crate::Wave;
        let fc =
            crate::math::exp((log_lo + norm * (log_hi - log_lo)) as crate::Wave) as crate::Wave;
        let sr = self.sample_rate;
        for spring in &mut self.springs {
            spring.set_damping_fc(fc, sr);
        }
    }

    /// Process one sample (input → dispersive tank → DC-blocked wet output).
    #[inline]
    pub fn process(&mut self, input: crate::Wave) -> crate::Wave {
        let x = input * self.dwell;
        let mut sum = 0.0;
        for spring in &mut self.springs {
            sum += spring.process(x);
        }
        let wet = sum * self.out_scale;

        // DC blocker: y = wet − x1 + coef·y1.
        let y = wet - self.dc_x1 + self.dc_coef * self.dc_y1;
        self.dc_x1 = wet;
        self.dc_y1 = if y.is_finite() { y } else { 0.0 };
        self.dc_y1
    }

    /// Reset all state.
    pub fn reset(&mut self) {
        self.dc_x1 = 0.0;
        self.dc_y1 = 0.0;
        for spring in &mut self.springs {
            spring.reset();
        }
    }

    /// Update sample rate (rebuilds tank). Allocation happens here, not in
    /// `process()`.
    pub fn set_sample_rate(&mut self, sample_rate: crate::Wave) {
        *self = Self::new(self.model.clone(), sample_rate);
    }

    /// Total active allpass sections across all springs (cost diagnostics).
    pub fn total_allpasses(&self) -> usize {
        self.springs.iter().map(|s| s.num_ap).sum()
    }
}
