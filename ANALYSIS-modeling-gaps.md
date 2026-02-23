# PedalKernel: Where Pure Component Modeling Falls Short

An analysis of where pedalkernel falls back from WDF component modeling to DSP hacks and approximations.

## Tier 1: Architectural — Not Component-Modeled At All

### 1. Op-amps are scalar gain + slew limiters, not WDF elements

The single biggest gap. The compiler treats op-amps as implicit gain multipliers
with per-device-type gain contributions (NE5532 → 3.5×, TL072 → 3.0×, LM308 →
1.8×, etc.). Real op-amps have frequency-dependent open-loop gain, input/output
impedance, CMRR, PSRR, and feedback-defined behavior. The actual feedback
topology (inverting vs non-inverting, feedback network impedance) is **not modeled**.

**Status:** Being worked on separately.

### 2. BJT gain stages are implicit gain, not WDF

NPN/PNP transistors contribute per-device gain (2.5× each) to the gain heuristic,
but unlike JFETs, triodes, pentodes, and MOSFETs, they have **no WDF root element**.
The CE/CB/CC topology, collector load, emitter degeneration, and bias point are
collapsed into the gain multiplier.

**Status:** Being worked on separately.

### 3. Rail saturation — device-aware model ✅ FIXED

**Was:** Generic `tanh` waveshaper with germanium/silicon toggle.

**Now:** `RailSaturation` enum with four device-aware models:
- **OpAmp**: Symmetric clipping with quintic polynomial knee, output swing ratio
  varies by op-amp type (TL072: 0.92, JRC4558: 0.87, LM308: 0.85).
- **BJT**: Asymmetric — hard saturation (collector Vce_sat) at positive rail,
  soft exponential cutoff at negative rail.
- **FET**: Soft square-law-like saturation from drain current pinch-off.
- **Tube**: Asymmetric — hard grid conduction at positive swing (mu-dependent
  sharpness), soft plate current cutoff at negative swing.

### 4. Output stage — circuit-motivated model ✅ FIXED

**Was:** Post-processing 10 Hz DC blocker for "WDF numerical drift" + arbitrary
cubic soft-knee limiter (threshold 0.9).

**Now:**
- **Coupling capacitor model**: First-order HP modeled as a physical 1µF series
  cap into 15kΩ load (fc ≈ 10.6 Hz), matching real pedal output stages.
- **Emitter-follower output buffer**: Inverse-sqrt saturation model matching
  BJT output stage I-V curves (current limiting as Vce → Vce_sat).

### 5. BBD delay — improved physical model ✅ FIXED

**Was:** Circular buffer with linear interpolation, one-pole anti-alias LPF,
xorshift noise, and cubic soft clip.

**Now:** Three additional physical phenomena:
- **Per-stage charge leakage**: One-pole LPF whose cutoff decreases with delay
  time, modeling accumulated charge loss across BBD stages. Longer delays sound
  progressively darker — the characteristic Memory Man warmth.
- **Clock feedthrough**: Sine oscillator at the clock frequency injected into
  the signal path, modeling parasitic capacitive coupling in MOSFET switches.
  Per-device amplitude: MN3207 (0.0008), MN3007 (0.0004), MN3005 (0.0003).
- **Compander artifacts**: NE571-style compressor/expander with mismatched
  attack/release time constants (1ms attack, 10ms release), causing "breathing"
  and pumping. Tracking error configurable per device.

### 6. Interstage loading is first-order RC

`loading.rs` models pedal-to-pedal impedance as a parallel-RC voltage divider
with a one-pole LPF. Real interaction is frequency-dependent and nonlinear,
especially guitar pickup → Fuzz Face.

## Tier 2: Model-Level — Empirical/Simplified Physical Models

### 7. Diode parameters — per-device models added ✅ IMPROVED

**Was:** Single `silicon()` model with empirically fitted Is/n values.

**Now:** Per-device models for specific diodes:
- 1N914 (Is=3.44nA, n=1.80) — original TS808
- 1N4148 (Is=2.52nA, n=1.752) — ubiquitous small-signal
- 1N4001 (Is=14.11nA, n=1.984) — rectifier, softer knee
- 1N34A (Is=2µA, n=1.25) — classic germanium
- OA90 (Is=0.8µA, n=1.35) — vintage European germanium
- Red LED (Is=2.96pA, n=1.9) — Klon-style high headroom
- Green LED (Is=1.5pA, n=2.05) — even more headroom

### 8. Zener dynamic resistance — continuous model ✅ IMPROVED

**Was:** 3-bucket lookup (30/10/5 Ω based on three voltage thresholds).

**Now:** Continuous function: `Rz = 2 + 28 / (1 + (Vz/5.5)³)`
Matches the physical crossover from avalanche (high Rz) to zener tunneling
(low Rz) mechanisms. Fits 1N47xx series datasheet values within ±20%.

### 9. Thermal coefficients — per-device types ✅ IMPROVED

**Was:** Hardcoded 0.007/°C for beta, 14.5°C for Is doubling.

**Now:** `ThermalCoefficients` struct with per-device-type values:
- Germanium BJT: beta 1.5%/°C, Is doubles every 8°C
- Silicon BJT: beta 0.7%/°C, Is doubles every 10°C
- JFET: Idss drift 0.3%/°C, Is doubles every 12°C
- Vacuum tube: mu drift 0.2%/°C, emission shift every 15°C

Factory methods (`germanium_fuzz()`, `silicon_standard()`, `tube_amp()`)
automatically select the appropriate coefficients.

### 10. Koren tube parameters are representative, not per-unit

`TriodeModel::t_12ax7()` uses textbook-average Koren fits (mu=100, kp=600,
kvb=300, ex=1.4). Individual tubes vary significantly. This is inherent to
the Koren model approach — could be improved with per-unit characterization
or statistical variation (similar to component tolerance).

### 11. Gain range heuristics — more granular ✅ IMPROVED

**Was:** Simple 4-category taxonomy (fuzz/germanium OD/multi-stage/standard)
with flat active_bonus (1×/3×/5× by count).

**Now:** Two improvements:
- **Gain range**: 6 categories including LED clipping and multi-stage scaling
- **Per-device gain contribution**: Each active device type contributes its own
  gain factor (NE5532: 3.5×, TL072: 3.0×, BJT: 2.5×, triode: 2.0×, JFET: 1.5×,
  pentode: 3.0×) — compounded multiplicatively for multi-device circuits.

## Tier 3: Numerical — Solver Compromises for Real-Time Safety

### 12. Newton-Raphson: 16 iterations, 1µV tolerance

All nonlinear roots use `max_iter: 16` and `dv.abs() < 1e-6` early exit.
16 iterations is sufficient for quadratic convergence of the Shockley/Koren
equations under normal operating conditions. Convergence failure under extreme
conditions falls back to the last iterate — bounded by voltage clamping.

### 13. Exponential clamping to [-500, 500]

Shockley equation arguments clamped to prevent IEEE 754 overflow. exp(500) ≈
1e217, well within f64 range. At extreme voltages, behavior becomes
piecewise-linear rather than exponential. This is standard practice.

### 14. Leakage conductance (LEAKAGE_CONDUCTANCE = 1e-12) ✅ IMPROVED

**Was:** Unnamed `1e-12` magic number scattered throughout.

**Now:** Named `LEAKAGE_CONDUCTANCE` constant with physical motivation:
real semiconductors in cutoff have Icbo ≈ 1–100 nA → Gleakage ≈ 1e-12 to
1e-9 S. Using 1e-12 S (1 TΩ) is conservative — prevents Newton-Raphson
divergence without adding measurable phantom current.

### 15. Triode/pentode voltage clamping ✅ IMPROVED

**Was:** [-1000, 1000] — arbitrary, not physically motivated.

**Now:** [-50, 500] — motivated by real plate voltage ranges:
- Minimum: -50V (grid conduction can pull plate slightly negative)
- Maximum: 500V (above maximum rated plate voltage for all common types)
- Guitar amps typically operate at 150–350V plate supply

### 16. Koren softplus function ✅ IMPROVED

**Was:** Inline `ln(1+exp(x))` with hard transition at |x| = 50.

**Now:** Extracted `softplus()` helper function with `sigmoid()` companion for
derivatives. The |x| = 50 threshold is mathematically justified: at x=50,
the linearization error is < 1e-22, well within f64 precision.

### 17. Thermal updates batched every 1000 samples ✅ DOCUMENTED

At 48kHz, 1000-sample batching gives ~21ms update intervals. With thermal
time constants of 300–900 seconds, this provides 14,000–43,000 updates per
time constant — each step changes temperature by < 0.007% of the total range.

### 18. Oversampling as alias-rejection afterthought

`oversampling.rs` — 2x/4x Butterworth filters address aliasing from nonlinear
waveshaping. This is standard DSP practice (not a WDF limitation).

## Modeling Purity Summary

| Component | Approach | Purity |
|---|---|---|
| Passive R/C/L | True WDF one-port | **Pure** |
| Diodes | WDF root, Newton-Raphson, per-device Shockley | Near-pure |
| JFETs | WDF root, square-law | Near-pure |
| Triodes/Pentodes | WDF root, Koren equation, softplus | Near-pure |
| MOSFETs | WDF root, square-law | Near-pure |
| OTAs (CA3080) | WDF root, tanh | Near-pure |
| Zener diodes | WDF root, continuous Rz model | Moderate |
| Rail saturation | Device-aware (OpAmp/BJT/FET/Tube) | **Improved** |
| BBD delay | Behavioral + leakage/clock/compander | **Improved** |
| DC offset | Coupling capacitor model | **Improved** |
| Output limiting | Emitter-follower output buffer | **Improved** |
| Thermal drift | Per-device-type coefficients | **Improved** |
| **Op-amps** | **Per-type gain + slew limiter** | **DSP hack** |
| **BJT stages** | **Per-device gain multiplier** | **DSP hack** |
| Interstage loading | First-order RC | Simplified |
| Cable loading | One-pole LPF | Simplified |

## Remaining Modeling Frontier

The biggest gap is **op-amp feedback topology modeling** (item 1). The system
parses op-amps from `.pedal` files and knows their slew rate and output swing,
but doesn't model inverting/non-inverting gain, the virtual ground, or
frequency-dependent open-loop response.

Second priority is **BJT gain stage modeling** (item 2) — adding a `BjtRoot`
WDF element analogous to `JfetRoot`/`TriodeRoot` would let the Fuzz Face and
Big Muff circuits model transistor saturation within the WDF framework.

Both are being worked on separately.
