# Physical Realism in PedalKernel

PedalKernel uses Wave Digital Filters (WDFs) to simulate guitar pedal circuits at the component level. While WDF theory handles the physics within a single circuit accurately, a naive implementation misses several real-world phenomena that occur between circuits, across time, and at the boundaries of digital audio. This document describes the physical realism features PedalKernel provides beyond the core WDF engine.

---

## Antialiasing via Oversampling

**The problem.** Nonlinear operations — diode clippers, transistor saturation, tanh soft limiting — generate harmonics. At standard sample rates (44.1/48 kHz), harmonics above Nyquist fold back into the audible spectrum as inharmonic artifacts. This is the single biggest difference between a naive WDF implementation and a production-quality plugin.

**The solution.** The `oversampling` module wraps each nonlinear WDF stage with configurable 2x or 4x oversampling:

1. **Upsample**: Zero-stuff and filter with a Butterworth anti-imaging filter
2. **Process**: Run the full WDF cycle (scatter-up → Newton-Raphson root solve → scatter-down) at the elevated sample rate
3. **Downsample**: Filter with a matched anti-aliasing filter and decimate

The filters are designed analytically using the bilinear transform — 4th-order Butterworth for 2x, 6th-order for 4x — providing ~48 dB and ~72 dB of alias rejection respectively.

```rust
use pedalkernel::compiler::{compile_pedal_with_options, CompileOptions};
use pedalkernel::oversampling::OversamplingFactor;

let options = CompileOptions {
    oversampling: OversamplingFactor::X4,
    ..Default::default()
};
let pedal = compile_pedal_with_options(&def, 44100.0, options)?;
```

**Cost.** 4x oversampling means 4x the Newton-Raphson iterations per sample. For a single diode clipper stage this is typically 4-8 iterations x 4 = 16-32 iterations per output sample. The filter overhead is negligible by comparison.

**When to use it.** Always use at least X2 for any pedal with hard clipping (Tube Screamer, RAT, Big Muff). X4 is recommended for high-gain settings and fuzz pedals. Clean boost and compression circuits with soft nonlinearities can get away with X1.

---

## Impedance Loading Between Stages

**The problem.** Real pedals load each other. A Fuzz Face (10k input impedance) after a buffered pedal sounds different than directly after a guitar pickup, because the low input impedance creates a voltage divider with the source. WDF handles impedance within a circuit, but between pedals in a chain, the source/load interaction must be modeled explicitly.

**The model.** The `loading` module provides:

- **`InterstageLoading`** — A frequency-dependent voltage divider between adjacent pedals. Models DC attenuation (`R_load / (R_src + R_load)`) and HF rolloff from parallel capacitances.
- **`CableModel`** — Guitar cable as an RC low-pass filter. A 20-foot cable at 30 pF/foot with a 7k pickup source has a -3 dB point around 38 kHz — meaningful treble rolloff that interacts with pickup impedance.
- **`ImpedanceModel`** — Presets for common impedance profiles: high-Z input (1M buffer), low-Z input (10k Fuzz Face), guitar pickup (7k), buffered output (1k).

```rust
use pedalkernel::loading::{ImpedanceModel, InterstageLoading};

// Guitar pickup -> Fuzz Face: significant loading
// DC gain = 10k / (7k + 10k) = 0.588 -- 41% signal loss
board.set_interstage_loading(
    0, // junction between pedal 0 and pedal 1
    ImpedanceModel::guitar_pickup(),
    ImpedanceModel::low_z_input(),
    48000.0,
);
```

**Default behavior.** All interstage junctions default to transparent loading (low-Z output -> high-Z input), which is the most common modern pedalboard configuration. Users opt in to impedance-sensitive connections where they matter.

**Why this matters.** The classic example: a Fuzz Face after a Wah pedal. The wah's buffered output presents ~1k to the fuzz, while a guitar pickup presents ~7k. The fuzz's input transistor biasing shifts depending on the source impedance, changing the clipping character entirely. Without this model, every fuzz sounds the same regardless of what precedes it.

### Cable Capacitance

Guitar cables act as a low-pass filter due to distributed capacitance (typically 30-40 pF/ft). Combined with a high-impedance pickup source, a 20-foot cable creates a resonant peak followed by rolloff — this is why true-bypass pedals can sound darker than buffered ones with long cable runs.

```rust
use pedalkernel::loading::CableModel;

// 20-foot cable from guitar (7k pickup) to first pedal
let mut cable = CableModel::long_20ft(7000.0, 48000.0);
// Cutoff: ~38 kHz (audible treble rolloff with pickup resonance)

// Cheap cable with higher capacitance
let mut cheap = CableModel::cheap_20ft(7000.0, 48000.0);
// Even more treble loss
```

---

## Component Tolerance

**The problem.** Real resistors are +/-5-20% from nominal. Two "identical" Tube Screamers from the same production run sound slightly different because their 4.7k resistors are actually 4.5k and 4.9k. This variation is part of what gives analog gear its character — and why some units are considered "magic."

**The model.** The `tolerance` module provides seeded deterministic randomization:

```rust
use pedalkernel::tolerance::{ToleranceEngine, ToleranceGrade};
use pedalkernel::compiler::{compile_pedal_with_options, CompileOptions};

// Two "units" of the same pedal with different component spreads
let unit_a = CompileOptions {
    tolerance: ToleranceEngine::with_grades(
        42,                        // seed = unit serial number
        ToleranceGrade::Standard,  // +/-5% resistors (metal film)
        ToleranceGrade::Loose,     // +/-10% capacitors (ceramic)
    ),
    ..Default::default()
};

let unit_b = CompileOptions {
    tolerance: ToleranceEngine::with_grades(
        43,                        // different seed = different unit
        ToleranceGrade::Standard,
        ToleranceGrade::Loose,
    ),
    ..Default::default()
};
```

**Key properties:**

- **Deterministic**: Same seed always produces the same component spread. A saved preset always sounds like the same physical pedal.
- **Per-component**: Each R/C/pot in the circuit gets its own independent variation based on a hash of (seed, component_index).
- **Bounded**: Values stay within the specified tolerance band. A +/-10% 4.7k resistor will be between 4.23k and 5.17k.
- **Centered**: The distribution is uniform around nominal, so the average across many units converges to the nominal value.

**Tolerance grades:**

| Grade     | Spread | Typical Use                                  |
|-----------|--------|----------------------------------------------|
| Precision | +/-1%  | Metal film resistors, precision circuits      |
| Standard  | +/-5%  | Metal film resistors, quality ceramic caps    |
| Loose     | +/-10% | Carbon film resistors, electrolytic caps      |
| Wide      | +/-20% | Carbon composition resistors, vintage parts   |
| Ideal     | 0%     | No variation (default)                        |

Tolerance is applied during compilation. The engine modifies resistor, capacitor, and potentiometer values in the circuit graph before WDF tree construction, so the tolerance flows through into all derived quantities (port resistances, adaptor coefficients, etc.) automatically.

---

## Thermal Drift

**The problem.** A germanium Fuzz Face played cold sounds gated and sputtery. After 15-20 minutes it warms up and becomes smooth and fat. This is because:

- **BJT current gain (beta)** increases ~0.7%/C -- a cold transistor has less gain
- **Diode saturation current (Is)** doubles every ~10C -- warm diodes clip earlier and softer
- **Thermal voltage (Vt = kT/q)** increases linearly with temperature -- shifts the clipping knee

These are real, measurable effects. Germanium is far more sensitive than silicon, which is why germanium fuzz pedals are notoriously temperature-dependent.

**The model.** The `thermal` module provides a first-order thermal system:

```
T(t) = T_ambient + (T_steady - T_ambient) * (1 - e^(-t/tau))
```

where tau is the thermal time constant (typically 300-600 seconds for a small pedal enclosure).

```rust
use pedalkernel::compiler::{compile_pedal_with_options, CompileOptions};

let options = CompileOptions {
    thermal: true,  // enable thermal drift
    ..Default::default()
};

// The compiler automatically selects the right thermal model:
// - Germanium circuits: starts at 15C, warms to 40C, tau = 600s
// - Silicon circuits: starts at 25C, warms to 35C, tau = 300s
```

**Presets:**

| Model             | Ambient | Steady State | tau  | Character                            |
|-------------------|---------|-------------|------|--------------------------------------|
| Germanium Fuzz    | 15C     | 40C         | 600s | Cold -> warm transition is dramatic  |
| Silicon Standard  | 25C     | 35C         | 300s | Subtle drift, mostly stable          |
| Tube Amp          | 25C     | 60C         | 900s | Classic "tube warm-up" period        |

**Implementation detail.** The thermal state updates every ~1000 audio samples (every ~21 ms at 48 kHz). This is more than sufficient since thermal changes happen on the scale of minutes, and it keeps the per-sample overhead near zero. Base diode model parameters are stored separately per stage so the thermal multiplier is applied fresh each update cycle without accumulation drift.

---

## How It All Fits Together

All four features can be combined:

```rust
use pedalkernel::compiler::{compile_pedal_with_options, CompileOptions};
use pedalkernel::oversampling::OversamplingFactor;
use pedalkernel::tolerance::{ToleranceEngine, ToleranceGrade};

let options = CompileOptions {
    oversampling: OversamplingFactor::X4,
    tolerance: ToleranceEngine::with_grades(
        42,
        ToleranceGrade::Standard,
        ToleranceGrade::Loose,
    ),
    thermal: true,
};

let mut pedal = compile_pedal_with_options(&def, 48000.0, options)?;
```

The original `compile_pedal()` function is unchanged and uses the defaults (no oversampling, ideal tolerance, no thermal), so existing code is unaffected.

At the pedalboard level, impedance loading is configured separately since it operates between pedals rather than within them:

```rust
use pedalkernel::loading::ImpedanceModel;

// After building the pedalboard:
board.set_interstage_loading(
    0,
    ImpedanceModel::guitar_pickup(),
    ImpedanceModel::low_z_input(), // Fuzz Face
    48000.0,
);
```

---

## What's Not Yet Modeled

These are real phenomena that could be added in the future:

- **Power supply sag** -- A shared 9V supply sagging under current draw causes compression and harmonic softening. Would need a simple PSU model (ideal Vs + series R + filter C) shared across pedals drawing from the same supply rail. Also 60 Hz / 120 Hz hum injection from unregulated supplies.
- **Ground loops and crosstalk** -- Multiple pedals sharing a ground path can create ground loop noise. In a digital model this manifests as inter-channel leakage if you're not careful about signal isolation between parallel processing paths.
- **Speaker/cabinet physics** -- Thiele-Small parameters, voice coil compression, cone breakup resonances, cabinet resonance/port tuning. Important for full-chain-to-speaker modeling.
- **Latency compensation** -- Oversampling introduces latency that needs to be reported to the DAW host for dry/wet alignment. The current oversampling implementation does not report latency.

---

## Priority Guide

If you're configuring PedalKernel for a specific use case, here's the recommended priority order:

1. **Oversampling (X2 minimum)** -- Highest ROI. Without it, hard clipping at 44.1 kHz produces audible inharmonic artifacts that make everything sound "digital." This is probably the single biggest quality improvement you can make.

2. **Impedance loading** -- Critical for Fuzz Face, vintage wahs, and any impedance-sensitive circuit in a chain. Negligible overhead. Can be skipped for all-buffered modern boards.

3. **Component tolerance** -- Adds realism and uniqueness. Two instances of the same pedal model will sound subtly different, like real hardware. Zero runtime cost (applied at compile time).

4. **Thermal drift** -- Nice-to-have for authenticity. Most audible on germanium fuzz pedals where the cold-to-warm transition is dramatic. Can be skipped for silicon circuits where the drift is subtle. Near-zero runtime cost.
