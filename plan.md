# Integration Test Suite Plan: End-to-End Audio Validation

## Overview

Build a comprehensive integration test suite that validates each component/effect type
by processing audio (both synthesized test signals and the existing `clean_guitar.wav`)
through purpose-built `.pedal` circuits, then analyzing the output WAV characteristics
with dedicated analysis utilities.

## Architecture

```
tests/
├── audio_analysis.rs         # Shared analysis utilities (Goertzel, THD, modulation, etc.)
├── test_pedals/              # Purpose-built .pedal files — one per component feature
│   ├── clip_silicon.pedal
│   ├── clip_germanium.pedal
│   ├── clip_led.pedal
│   ├── clip_zener.pedal
│   ├── clip_asymmetric.pedal
│   ├── bbd_chorus.pedal
│   ├── bbd_long_delay.pedal
│   ├── bbd_fast_clock.pedal
│   ├── cap_leaky.pedal
│   ├── cap_da.pedal
│   ├── cap_ideal_reference.pedal
│   ├── triode_clean.pedal
│   ├── triode_overdrive.pedal
│   ├── pentode_clean.pedal
│   ├── pentode_power.pedal
│   ├── opamp_buffer.pedal
│   ├── opamp_gain.pedal
│   ├── opamp_slew.pedal
│   ├── jfet_allpass.pedal
│   ├── jfet_switch.pedal
│   ├── lfo_sine.pedal
│   ├── lfo_triangle.pedal
│   ├── lfo_square.pedal
│   └── lfo_modulated_jfet.pedal
├── integration_clipping.rs   # Diode/clipping tests
├── integration_bbd.rs        # BBD delay + chorus tests
├── integration_capacitor.rs  # Leakage + DA tests
├── integration_tubes.rs      # Triode + pentode tests
├── integration_opamp_jfet.rs # Op-amp and JFET tests
└── integration_lfo.rs        # LFO modulation tests
```

Each `integration_*.rs` file is a standalone integration test file that uses the
shared `audio_analysis` module. Rust's test harness runs them all in parallel.

---

## Step 1: Audio Analysis Utilities (`tests/audio_analysis.rs`)

A `#[allow(dead_code)]` shared module providing measurement functions. No external
FFT crate — use Goertzel algorithm for efficient single-bin DFT.

### Functions

```rust
/// RMS level of a buffer.
pub fn rms(buf: &[f64]) -> f64

/// Peak absolute amplitude.
pub fn peak(buf: &[f64]) -> f64

/// DC offset (mean of buffer).
pub fn dc_offset(buf: &[f64]) -> f64

/// Goertzel algorithm: magnitude² of a single DFT bin.
/// O(N) per bin — efficient when we only need a handful of frequencies.
pub fn goertzel_mag(buf: &[f64], sample_rate: f64, target_hz: f64) -> f64

/// Energy in a frequency band [lo_hz, hi_hz] relative to total energy.
/// Steps through band at 10 Hz resolution using Goertzel.
pub fn band_energy_ratio(buf: &[f64], sample_rate: f64, lo_hz: f64, hi_hz: f64) -> f64

/// THD (Total Harmonic Distortion) of a signal at a known fundamental.
/// Measures energy in harmonics 2–8 relative to fundamental.
/// Returns ratio (0.0 = pure sine, 1.0 = harmonics equal fundamental).
pub fn thd(buf: &[f64], sample_rate: f64, fundamental_hz: f64) -> f64

/// Normalized cross-correlation between two buffers.
pub fn correlation(a: &[f64], b: &[f64]) -> f64

/// Spectral centroid — perceptual "brightness" in Hz.
/// Computed over linearly-spaced Goertzel bins from 20 Hz to Nyquist.
pub fn spectral_centroid(buf: &[f64], sample_rate: f64) -> f64

/// Detect dominant modulation rate by envelope analysis.
/// 1. Compute amplitude envelope (abs + lowpass)
/// 2. Remove DC from envelope
/// 3. Find peak frequency in envelope spectrum (0.1–20 Hz range)
pub fn detect_modulation_rate(buf: &[f64], sample_rate: f64) -> f64

/// Crest factor = peak / RMS. Low = compressed/clipped, high = dynamic.
pub fn crest_factor(buf: &[f64]) -> f64

/// Assert signal is finite, non-silent, and bounded.
pub fn assert_healthy(buf: &[f64], name: &str, max_peak: f64)

/// Compile a .pedal source string and process an input buffer through it.
/// Returns the output buffer. Convenience wrapper for tests.
pub fn compile_and_process(
    pedal_src: &str,
    input: &[f64],
    sample_rate: f64,
    controls: &[(&str, f64)],
) -> Vec<f64>

/// Same but loads from a .pedal file path.
pub fn compile_file_and_process(
    pedal_path: &str,
    input: &[f64],
    sample_rate: f64,
    controls: &[(&str, f64)],
) -> Vec<f64>

/// Generate a sine wave test signal.
pub fn sine(freq_hz: f64, duration_secs: f64, sample_rate: f64) -> Vec<f64>

/// Generate a multi-harmonic guitar-like pluck.
pub fn guitar_pluck(freq_hz: f64, duration_secs: f64, sample_rate: f64) -> Vec<f64>

/// Load the shared clean_guitar.wav (cached via LazyLock).
pub fn clean_guitar() -> &'static (Vec<f64>, u32)

/// Optionally dump a WAV to disk when PEDALKERNEL_DUMP_WAV=1.
pub fn maybe_dump_wav(samples: &[f64], name: &str, sample_rate: u32)
```

### Why Goertzel instead of FFT

- No extra dependency (no `rustfft` or `realfft`)
- O(N) per frequency bin — we only check ~10 bins per test
- Perfectly adequate for THD and band energy checks
- Keeps the test crate dependency-free beyond what's already in Cargo.toml

---

## Step 2: Test .pedal Files (24 files)

Each file isolates a **single component behavior** with minimal surrounding circuit
(typically just input coupling cap + the component under test + load resistor to output).
This makes failures directly attributable to a specific model.

### 2a. Clipping Circuits (5 files)

Each is a simple "input → coupling cap → resistor → clipping element → ground, output
tapped before clipper" topology.

| File | Key Component | What It Validates |
|------|---------------|-------------------|
| `clip_silicon.pedal` | `diode_pair(silicon)` | Vf ≈ 0.6V symmetric hard clip |
| `clip_germanium.pedal` | `diode_pair(germanium)` | Vf ≈ 0.3V softer knee |
| `clip_led.pedal` | `diode_pair(led)` | Vf ≈ 1.7V wide headroom before clipping |
| `clip_zener.pedal` | `zener(4.7)` | Asymmetric: forward ~0.6V, reverse ~4.7V |
| `clip_asymmetric.pedal` | `diode(silicon)` | Single-diode half-wave rectification |

### 2b. BBD Circuits (3 files)

| File | Key Component | What It Validates |
|------|---------------|-------------------|
| `bbd_chorus.pedal` | `bbd(mn3207)` + `lfo(triangle)` | Modulated delay, chorus character |
| `bbd_long_delay.pedal` | `bbd(mn3005)` | 4096-stage, leakage accumulation |
| `bbd_fast_clock.pedal` | `bbd(mn3207)` + fast LFO | Fast sweep, anti-alias filtering |

### 2c. Capacitor Parasitics (3 files)

| File | Key Component | What It Validates |
|------|---------------|-------------------|
| `cap_leaky.pedal` | `cap(1u, electrolytic, leakage: 1k)` | Accelerated state decay |
| `cap_da.pedal` | `cap(10u, electrolytic, da: 0.1)` | Dielectric absorption memory |
| `cap_ideal_reference.pedal` | `cap(1u)` | Baseline ideal behavior for comparison |

### 2d. Tube Circuits (4 files)

Each uses a realistic single-stage amp topology with appropriate supply voltage.

| File | Key Component | What It Validates |
|------|---------------|-------------------|
| `triode_clean.pedal` | `triode(12au7)` at 250V | Low-mu clean gain |
| `triode_overdrive.pedal` | `triode(12ax7)` at 250V | High-mu overdrive, asymmetric clip |
| `pentode_clean.pedal` | `pentode(ef86)` at 300V | Pentode preamp character |
| `pentode_power.pedal` | `pentode(el34)` at 450V | Power tube saturation |

### 2e. Op-Amp and JFET Circuits (5 files)

| File | Key Component | What It Validates |
|------|---------------|-------------------|
| `opamp_buffer.pedal` | `opamp(tl072)` unity-gain | Low distortion, unity gain |
| `opamp_gain.pedal` | `opamp(jrc4558)` with Rf/Ri feedback | Closed-loop gain |
| `opamp_slew.pedal` | `opamp(lm308)` | Slew-rate limiting at HF |
| `jfet_allpass.pedal` | `njfet(j201)` + R/C | All-pass phase shift |
| `jfet_switch.pedal` | `njfet(2n5457)` as variable R | Vgs-controlled resistance |

### 2f. LFO Circuits (4 files)

Each routes an LFO to modulate a JFET gate, with a steady sine input.

| File | Key Component | What It Validates |
|------|---------------|-------------------|
| `lfo_sine.pedal` | `lfo(sine, 100k, 47n)` | Smooth sinusoidal modulation |
| `lfo_triangle.pedal` | `lfo(triangle, 100k, 47n)` | Triangle modulation |
| `lfo_square.pedal` | `lfo(square, 100k, 47n)` | Hard on/off tremolo |
| `lfo_modulated_jfet.pedal` | LFO → 4× JFET allpass | Full phaser sweep |

---

## Step 3: Integration Tests

### 3a. Clipping Tests (`tests/integration_clipping.rs`)

**Tests (~12):**

1. **`silicon_thd_increases_with_level`** — Process 440 Hz sine at amplitudes 0.1, 0.3, 0.5, 0.8. THD should increase monotonically.

2. **`germanium_clips_before_silicon`** — At same input level (0.3), Ge THD > Si THD (lower Vf).

3. **`led_clips_after_silicon`** — At moderate level (0.5), LED THD < Si THD (higher Vf headroom).

4. **`symmetric_pair_odd_harmonics`** — DiodePair output: 3rd harmonic > 2nd harmonic (symmetric clipping produces odd harmonics).

5. **`asymmetric_diode_even_harmonics`** — Single Diode output: significant 2nd harmonic (half-wave rectification breaks symmetry).

6. **`zener_asymmetric_clipping`** — Forward clips at ~0.6V, reverse at ~4.7V. Verify asymmetric waveform.

7. **`all_clippers_finite_with_guitar`** — Process `clean_guitar.wav` through each clipper. All outputs finite, non-silent, bounded.

8. **`clipping_reduces_crest_factor`** — Crest factor of clipped output < crest factor of input (clipping compresses peaks).

9. **`silicon_vs_germanium_spectral_centroid`** — Ge output brighter than Si at same gain (softer knee creates different harmonic balance).

10. **`high_drive_heavy_distortion`** — At input amplitude 0.8, all clippers have THD > 10%.

11. **`low_drive_minimal_distortion`** — At input amplitude 0.05, all clippers have THD < 5% (below clipping threshold).

12. **`clipping_outputs_bounded`** — No clipper produces output > 2.0 from input amplitude 0.5.

### 3b. BBD Tests (`tests/integration_bbd.rs`)

**Tests (~8):**

1. **`bbd_chorus_produces_modulation`** — Process steady sine through BBD chorus. `detect_modulation_rate()` > 0 (non-zero modulation detected).

2. **`bbd_modulation_rate_matches_lfo`** — Detected modulation rate within ±30% of expected LFO frequency (f = 1/(2π×R×C)).

3. **`bbd_long_delay_leakage_decay`** — MN3005 (4096 stages): output RMS decays with delay length. Late output quieter than early output.

4. **`bbd_chorus_finite_with_guitar`** — Process `clean_guitar.wav`. Output finite, non-silent.

5. **`bbd_adds_spectral_richness`** — Chorus output has wider spectral spread than dry input (pitch modulation creates sidebands).

6. **`bbd_fast_clock_hf_limited`** — Fast-clock BBD: high-frequency energy (>8kHz) should be attenuated (anti-alias filter).

7. **`bbd_output_has_wet_signal`** — Output differs from input (correlation < 0.99).

8. **`bbd_all_models_stable`** — MN3207, MN3007, MN3005 all produce finite, non-silent output.

### 3c. Capacitor Tests (`tests/integration_capacitor.rs`)

**Tests (~6):**

1. **`leaky_cap_decays_faster`** — Charge both ideal and leaky caps with DC, then switch to zero input. Leaky cap discharge energy < ideal cap discharge energy.

2. **`da_cap_has_residual_voltage`** — After charging DA cap and fully discharging, residual late-stage energy is detectable (dielectric absorption recovery).

3. **`cap_types_all_compile`** — Film, electrolytic, ceramic, tantalum all compile and produce finite output.

4. **`leaky_cap_affects_low_frequency`** — Leaky cap acts as a lossy HPF. Very low frequencies (10 Hz) attenuated more than with ideal cap.

5. **`da_coefficient_scales_effect`** — DA=0.1 cap shows more residual than DA=0.01 cap.

6. **`ideal_cap_matches_reference`** — Ideal cap circuit output matches expected RC highpass behavior (energy above cutoff freq > energy below).

### 3d. Tube Tests (`tests/integration_tubes.rs`)

**Tests (~10):**

1. **`triode_12au7_clean_at_low_level`** — 12AU7 (low mu) with small input: THD < 10%.

2. **`triode_12ax7_more_gain_than_12au7`** — At same input, 12AX7 output RMS > 12AU7 output RMS (higher mu = more gain).

3. **`triode_12ax7_more_thd_than_12au7`** — At same input level, 12AX7 THD > 12AU7 THD (clips earlier due to higher gain).

4. **`triode_thd_increases_with_level`** — Process 12AX7 at increasing input levels. THD monotonically increases.

5. **`triode_asymmetric_clipping`** — Tube clipping produces even + odd harmonics (asymmetric transfer function). 2nd harmonic should be significant.

6. **`pentode_ef86_higher_gain`** — EF86 pentode produces higher output than 12AU7 triode at same input (pentode has higher gain).

7. **`pentode_different_harmonic_profile`** — Pentode harmonic spectrum differs from triode (more odd harmonics in pentode).

8. **`tube_amps_with_guitar`** — Process `clean_guitar.wav` through Tweed Deluxe, Bassman, JTM45. All produce finite, non-silent output with character.

9. **`tube_amps_distinct_from_each_other`** — Correlation between amp outputs < 0.99 (they sound different).

10. **`pentode_power_saturates`** — EL34 at high input: crest factor drops significantly (power tube compression).

### 3e. Op-Amp / JFET Tests (`tests/integration_opamp_jfet.rs`)

**Tests (~10):**

1. **`opamp_buffer_near_unity_gain`** — TL072 buffer: output correlation with input > 0.90.

2. **`opamp_buffer_low_thd`** — THD < 5% at moderate levels.

3. **`opamp_buffer_bounded_by_rails`** — Output peak < supply/2 (rail saturation).

4. **`opamp_slew_lm308_vs_tl072`** — LM308 (0.3V/µs) reduces HF more than TL072 (13V/µs). Spectral centroid of LM308 output < TL072 output with same HF-rich input.

5. **`opamp_slew_affects_square_wave`** — Square wave through LM308: output has rounded edges (lower crest factor than input).

6. **`jfet_allpass_flat_magnitude`** — JFET allpass: RMS at 200 Hz ≈ RMS at 2 kHz (±3dB). All-pass has flat magnitude.

7. **`jfet_produces_finite_output`** — All JFET circuits produce finite, non-silent output.

8. **`opamp_gain_stage_amplifies`** — JRC4558 with Rf/Ri=10: output RMS > 5× input RMS (with headroom margin).

9. **`opamp_all_models_compile`** — TL072, JRC4558, LM308, LM741, NE5532 all compile into working circuits.

10. **`jfet_switch_attenuates`** — JFET as switch: at Vgs near pinchoff, output significantly attenuated vs Vgs=0.

### 3f. LFO Tests (`tests/integration_lfo.rs`)

**Tests (~8):**

1. **`lfo_sine_produces_modulation`** — Sine LFO modulating JFET: amplitude envelope varies over time.

2. **`lfo_rate_matches_rc`** — Detected modulation rate within ±30% of expected f=1/(2π×R×C).

3. **`lfo_sine_smooth_envelope`** — Sine LFO envelope: low high-frequency content in envelope spectrum (smooth modulation).

4. **`lfo_square_sharp_transitions`** — Square LFO: envelope has sharp transitions (high HF content in envelope).

5. **`lfo_all_waveforms_stable`** — Sine, triangle, square all produce finite, bounded output.

6. **`lfo_speed_control_changes_rate`** — Phase 90 at Speed=0.2 vs Speed=0.8: faster speed → higher detected modulation rate.

7. **`lfo_modulated_phaser_sweeps`** — 4-stage phaser: spectral centroid varies over time (notches sweeping through spectrum).

8. **`lfo_depth_affects_modulation_amount`** — Higher LFO depth → greater amplitude variation in output envelope.

---

## Step 4: Parallelization Strategy

### Built-in Rust test parallelism

Rust's `cargo test` already runs `#[test]` functions in parallel across threads.
With ~54 independent tests across 6 files:

- Each test compiles its own pedal and processes its own audio — no shared mutable state
- `clean_guitar.wav` loaded once via `std::sync::LazyLock` (immutable, shared safely)
- `cargo test -j$(nproc)` naturally saturates available cores

### Batch processing helper for comparison tests

Tests that compare multiple pedals (e.g., "Ge clips before Si") use `std::thread::scope`
to compile and process pedals concurrently within a single test:

```rust
fn process_pedals_parallel(
    configs: &[(&str, &[(&str, f64)])],
    input: &[f64],
    sample_rate: f64,
) -> Vec<Vec<f64>> {
    std::thread::scope(|s| {
        let handles: Vec<_> = configs.iter().map(|(src, controls)| {
            s.spawn(|| compile_and_process(src, input, sample_rate, controls))
        }).collect();
        handles.into_iter().map(|h| h.join().unwrap()).collect()
    })
}
```

### Test duration tiers

- **Fast tests** (default): Use 0.1s buffers (4800 samples at 48 kHz). Run in <5s total.
- **Full tests** (`cargo test -- --ignored`): Use 1s+ buffers for accurate low-frequency
  analysis. Gated behind `#[ignore]` attribute.

### Expected timing

| Category | # Tests | Est. parallel time |
|----------|---------|-------------------|
| Clipping | 12 | ~2s |
| BBD | 8 | ~3s |
| Capacitor | 6 | ~2s |
| Tubes | 10 | ~3s |
| Op-Amp/JFET | 10 | ~2s |
| LFO | 8 | ~3s |
| **Total** | **~54** | **~5s parallel** |

---

## Step 5: WAV Output for Manual Inspection

Each test optionally writes output WAV to `test_output/` when `PEDALKERNEL_DUMP_WAV=1`:

```rust
pub fn maybe_dump_wav(samples: &[f64], name: &str, sample_rate: u32) {
    if std::env::var("PEDALKERNEL_DUMP_WAV").is_ok() {
        let dir = Path::new("test_output");
        std::fs::create_dir_all(dir).ok();
        write_wav(samples, &dir.join(format!("{name}.wav")), sample_rate).unwrap();
    }
}
```

Usage: `PEDALKERNEL_DUMP_WAV=1 cargo test` to generate WAV files for listening.

---

## Implementation Order

1. **Audio analysis module** (`tests/audio_analysis.rs`) — Goertzel, THD, correlation, modulation detection, helpers
2. **Test .pedal files** (24 files in `tests/test_pedals/`)
3. **Clipping tests** — most straightforward, validates the analysis utilities work
4. **Tube tests** — validates triode/pentode Koren models
5. **Op-Amp/JFET tests** — validates active device models
6. **BBD tests** — validates delay + modulation + leakage
7. **Capacitor tests** — validates leakage and DA parasitic models
8. **LFO tests** — validates modulation routing and waveform generation

---

## Dependencies

- **No new crate dependencies** — Goertzel + analysis utilities are pure Rust math
- Uses existing `hound` (already in deps) for WAV I/O
- Uses existing `pedalkernel` public API: `parse_pedal_file`, `compile_pedal`, `PedalProcessor`, `read_wav_mono`, `write_wav`
- Uses `std::sync::LazyLock` (stable since Rust 1.80) for shared test data
