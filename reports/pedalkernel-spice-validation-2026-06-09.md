# PedalKernel SPICE Validation Report - 2026-06-09

## Scope

This report covers the current `pedalkernel` workspace at git commit `7f14baf`, using the `pedalkernel-validate` crate to compare WDF output against freshly generated ngspice references.

Validation settings:

- Sample rate: 96 kHz
- Oversampling: 4x
- Internal SPICE/WDF rate: 384 kHz
- Circuit inputs: `pedalkernel-validate/circuits`
- SPICE netlists: `pedalkernel-validate/spice-circuits`
- Fresh SPICE golden output: `/private/tmp/pedalkernel-spice-golden`
- WDF output: `/private/tmp/pedalkernel-spice-output`
- JSON report: `/private/tmp/pedalkernel-spice-report.json`

Commands run:

```bash
cargo run -p pedalkernel-validate -- check-spice
cargo run -p pedalkernel-validate -- --golden /private/tmp/pedalkernel-spice-golden --output /private/tmp/pedalkernel-spice-output generate-spice --suite all --spice-dir pedalkernel-validate/spice-circuits
cargo run -p pedalkernel-validate -- --circuits pedalkernel-validate/circuits --golden /private/tmp/pedalkernel-spice-golden --output /private/tmp/pedalkernel-spice-output --report /private/tmp/pedalkernel-spice-report.json --detailed run --suite all
cargo test -p pedalkernel-validate
cargo test --workspace
```

## Executive Summary

Fresh SPICE reference generation succeeded for every configured validation signal:

- Generated: 75
- Skipped: 0
- Failed: 0

WDF versus fresh SPICE comparison:

- Total tests: 42
- Passed: 21
- Failed: 21
- Pass rate: 50.0%

Suite results:

| Suite | Passed | Failed | Status |
| --- | ---: | ---: | --- |
| active | 7 | 3 | FAIL |
| canonical | 3 | 5 | FAIL |
| linear | 0 | 4 | FAIL |
| nonlinear | 0 | 5 | FAIL |
| opamp | 6 | 0 | PASS |
| pedals | 2 | 2 | FAIL |
| reactive | 2 | 2 | FAIL |
| stress | 1 | 0 | PASS |

The strongest areas are op-amp topologies, DC stability, BJT diff pair, op-amp diode clipping, single-diode canonical tests, LC resonance, transformer step-up, BJT/MOSFET/PJFET-ish active smoke coverage, photocoupler, pentode push-pull, TS808, and fuzz core.

The weakest areas are linear passive SPICE agreement, standalone nonlinear diode/zener/triode suite agreement, transformer step-down/load scaling, delay-line compilation, OTA compilation, and some spectral-threshold-only failures where time-domain metrics are otherwise excellent.

## Component Type Coverage

The DSL component reference lists these component types. Current SPICE validation coverage is uneven:

| Component family | DSL types | SPICE validation status |
| --- | --- | --- |
| Passives | resistor, capacitor, inductor, potentiometer, zener | Resistor/capacitor/inductor/zener covered. Potentiometer not directly covered in `pedalkernel-validate`. Linear passive tests currently fail against fresh SPICE. |
| Diodes/clipping | diode, diode_pair | Covered by single diode, diode clipper, diode-no-cap, opamp diode clipper, RAT/TS808/Big Muff cores. Canonical single diode passes; standalone nonlinear diode suite fails thresholds against fresh SPICE. |
| BJTs | npn, pnp, matched_npn, matched_pnp | NPN, PNP, BJT diff pair, Fuzz Face, fuzz core covered. NPN/PNP common emitter and canonical BJT diff pair pass; saturated Fuzz Face fails. Matched pair component names are not directly covered. |
| JFETs | njfet, pjfet | N-JFET source follower and phase all-pass stage covered. Source follower passes under loose limited-support criteria; phase all-pass sweep fails spectral threshold. P-JFET is not directly covered. |
| MOSFETs | nmos, pmos | NMOS common source and PMOS source follower covered and passing. |
| Op-amps | opamp variants, OTA CA3080 | Standard op-amp configurations all pass. OTA CA3080 fails compile in rigid fallback: unsupported nonlinear device kind in general MNA. |
| Vacuum tubes | triode, pentode | Triode common cathode covered and failing; push-pull 6L6 pentode covered and passing. Other tube model variants are not directly covered in `pedalkernel-validate`. |
| Delay/modulation | BBD, delay_line, tap, LFO, envelope_follower | Simple delay line is covered but fails compile with `No circuit edges found`. BBD, tap, LFO, and envelope follower are not covered by SPICE validation suite. |
| Opto/neon | photocoupler, neon | Photocoupler optical attenuator covered and passing. Neon is not covered. |
| Transformers/switching | transformer, switched cap/inductor/resistor, rotary_switch, switch | Transformer step-up passes; step-down and transformer load fail. Switched components and switches are not directly covered. |
| Synth ICs | VCO, VCF, VCA, comparator, analog_switch, tempco | Not covered by `pedalkernel-validate` SPICE suite. |

## Detailed Results

### Active Devices

Status: 7/10 passed.

Passing:

- `jfet_source_follower`: sine passes with near-zero RMS and peak error. This is marked limited-support in the suite and uses very loose criteria.
- `nmos_common_source`: sine passes with -0.13 dB RMS error and near-zero peak error.
- `npn_common_emitter`: clean and driven both pass; driven RMS error is -0.36 dB.
- `optical_attenuator`: passes despite -23.1 dB RMS and -14.1 dB peak because suite tolerance is intentionally loose for photocoupler dynamics.
- `pmos_source_follower`: sine passes with 0.03 dB RMS error.
- `pnp_common_emitter`: clean and driven both pass; peak error remains below 0.43 dB.
- `push_pull_6l6`: sine passes with zero reported RMS/peak error.

Failing:

- `fuzz_face_pnp`: clean passes, saturated fails with 16.10 dB RMS and 18.39 dB peak error.
- `ota_ca3080`: compile failure: `Group 0 (rigid fallback): Unsupported NL device kind in general MNA`.
- `phase_allpass_stage`: sine passes, sweep fails spectral threshold only. RMS and peak are effectively zero, spectral error is 265.69 dB against a 260 dB threshold.

### Canonical Circuits

Status: 3/8 passed.

Passing:

- `bjt_diff_pair`: linear, soft clipping, and sweep all pass.
- `opamp_diode_clipper`: linear, clipping, and sweep all pass.
- `single_diode`: mild clipping, hard clipping, and sweep all pass.

Failing:

- `rc_highpass`: impulse, sine, and sweep all fail. Sine is -17.87 dB RMS and -12.28 dB peak; sweep peak error is 5.80 dB.
- `rc_lowpass`: impulse, sine, and sweep all fail. Sine is -21.02 dB RMS and -21.15 dB peak; sweep peak error is 1.19 dB.
- `series_rlc`: impulse, sine, and sweep all fail. Sine is -7.42 dB RMS and -7.28 dB peak.
- `transformer_load`: sine and sweep both fail. Sine is -6.17 dB RMS/peak.
- `twin_t_notch`: impulse, sine, and sweep all fail. Sine is -13.08 dB RMS and -3.26 dB peak.

### Linear Passives

Status: 0/4 passed.

Failing:

- `resistor_divider`: sine fails with -21.01 dB RMS and -20.70 dB peak against strict -60/-50 dB criteria.
- `rc_lowpass`: impulse, sine, and sweep fail. This mirrors canonical `rc_lowpass`.
- `rc_highpass`: sine and sweep fail. Sine is -23.07 dB RMS and -14.11 dB peak.
- `rl_lowpass`: sine fails with -21.02 dB RMS and -21.15 dB peak.

Interpretation: because simple resistor divider and first-order filters all miss strict SPICE thresholds, there is likely either a baseline comparison/scaling/reference mismatch or a systematic passive extraction/output-node mismatch, not just isolated filter math.

### Nonlinear Devices

Status: 0/5 passed.

Failing:

- `single_diode`: sine fails with 6.19 dB RMS and 4.87 dB peak error.
- `diode_no_cap`: low-level and clipping both fail around 7 dB RMS/peak error.
- `diode_clipper`: low-level, clipping, and sweep all fail. The sweep is slightly over peak threshold at 8.05 dB.
- `zener_clipper`: low-level fails at 7.18 dB RMS, clipping fails hard at 16.62 dB RMS and 20.72 dB peak.
- `common_cathode_12ax7`: clean and driven both fail. Clean is 8.71 dB RMS and 15.16 dB peak; driven is 7.19 dB RMS and 10.27 dB peak.

Note: canonical `single_diode` passes perfectly against its SPICE reference, while the standalone nonlinear `single_diode` fails. That points to circuit/topology/reference differences between the canonical and nonlinear fixtures rather than a universal single-diode solver failure.

### Op-Amps

Status: 6/6 passed.

Passing:

- `unity_buffer`
- `inverting_gain10`
- `noninverting_gain10`
- `summing_inverting`
- `difference_amp`
- `integrator`

This is the cleanest component family in the SPICE validation suite. The integrator passes with 1.98 dB RMS and 2.28 dB peak under the suite's 3/5 dB tolerances; the other gain/buffer stages are comfortably within their configured thresholds.

### Pedal Core Circuits

Status: 2/4 passed.

Passing:

- `fuzz_core`: clean and saturated both pass.
- `ts808_clipper`: clean and clipping both pass.

Failing:

- `bigmuff_stage`: clean fails while clipping passes. Clean has tiny RMS/peak error but fails due to THD error of 134.80 dB against a 100 dB threshold.
- `rat_clipper`: clean and clipping both fail due to THD error, despite zero RMS and peak error. Clean THD error is 176.80 dB; clipping THD error is 189.44 dB.

Interpretation: RAT and Big Muff failures are primarily metric/threshold or harmonic-content mismatches, not amplitude/time-domain mismatches.

### Reactive Components

Status: 2/4 passed.

Passing:

- `lc_resonant`: resonant and sweep both pass.
- `transformer_stepup`: sine passes with -2.55 dB RMS/peak.

Failing:

- `delay_simple`: compile failure: `No circuit edges found`.
- `transformer_stepdown`: sine and sweep both fail badly. Sine is 19.21 dB RMS and 19.19 dB peak; sweep is 20.14 dB RMS and 20.90 dB peak.

### Stress

Status: 1/1 passed.

Passing:

- `dc_stability`: silence passes with -200 dB RMS and peak error.

## Additional Test Runs

`cargo test -p pedalkernel-validate`:

- Library/unit tests: 19 passed.
- Failing integration test: `debug_delay_simple`.
- Failure: `Failed to compile: "No circuit edges found"`.

`cargo test --workspace`:

- Did not complete cleanly.
- First real compile error observed: `pedalkernel/tests/product_regression_matrix.rs:140` does not handle `Stage::KMethod { .. }` in a match over `Stage`.
- The subsequent build filled the local target directory and produced multiple `No space left on device` linker/write failures.
- I ran `cargo clean` afterward, which removed 3.8 GiB from `target/`.

## Prioritized Findings

1. Fix baseline passive/SPICE agreement before trusting higher-level analog deltas. `resistor_divider`, RC, RL, and canonical passive filters all fail, suggesting systematic gain/output/reference mismatch.
2. Add `Stage::KMethod { .. }` handling in `product_regression_matrix.rs`; the workspace test suite currently cannot compile.
3. Fix delay-line compilation in `pedalkernel-validate/circuits/reactive/delay_simple.pedal`; it fails with `No circuit edges found` in both validation and debug tests.
4. Add OTA support to the rigid/general MNA fallback or prevent CA3080 from being routed there.
5. Investigate transformer scaling/loading. Step-up passes, but step-down and transformer-load fail by roughly 6-20 dB depending on fixture.
6. Revisit nonlinear standalone fixtures versus canonical fixtures. Canonical single diode passes exactly, but nonlinear single diode and diode clippers fail around 6-7 dB.
7. Reassess THD-only pedal-core thresholds for RAT and Big Muff clean. Their time-domain amplitude errors are essentially zero, but harmonic metrics fail by a large margin.

## Artifacts

- Fresh SPICE golden directory: `/private/tmp/pedalkernel-spice-golden`
- WDF output directory: `/private/tmp/pedalkernel-spice-output`
- Structured JSON report: `/private/tmp/pedalkernel-spice-report.json`
