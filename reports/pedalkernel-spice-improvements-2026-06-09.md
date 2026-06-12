# PedalKernel SPICE Improvement Pass - 2026-06-09

## Changes Made

This pass fixed two validation harness problems that were hiding component-level signal:

1. `pedalkernel-validate` SPICE PWL generation no longer decimates normal validation signals. Short signals up to 100k samples are now emitted sample-exact, while very long stress signals are still decimated to keep ngspice netlists practical.
2. `pedalkernel-validate` now compiles circuits with runtime oversampling disabled (`X1`) because the validation runner already generates and compares signals at `sample_rate * oversample`.

I also fixed the product regression matrix compile blocker by adding the missing `Stage::KMethod { .. }` match arm.

## Validation Commands

```bash
cargo test -p pedalkernel-validate spice::tests --lib
cargo run -p pedalkernel-validate -- --golden /private/tmp/pedalkernel-spice-golden-fixed --output /private/tmp/pedalkernel-spice-output-fixed generate-spice --suite all --spice-dir pedalkernel-validate/spice-circuits
cargo run -p pedalkernel-validate -- --circuits pedalkernel-validate/circuits --golden /private/tmp/pedalkernel-spice-golden-fixed --output /private/tmp/pedalkernel-spice-output-fixed --report /private/tmp/pedalkernel-spice-fixed-report.json --detailed run --suite all
cargo test -p pedalkernel --test product_regression_matrix
```

## Updated Pass Rate

Fresh SPICE reference generation still succeeds completely:

- Generated: 75
- Skipped: 0
- Failed: 0

WDF versus fixed SPICE references:

- Previous report: 21/42 passed
- Current run: 23/42 passed
- Net improvement: +2 tests

Suite results after this pass:

| Suite | Passed | Failed | Notes |
| --- | ---: | ---: | --- |
| active | 8 | 2 | Phase all-pass sweep now passes; OTA still compile-fails; Fuzz Face PNP fails badly. |
| canonical | 3 | 5 | BJT diff pair, opamp diode clipper, single diode pass. Passive/reacive canonical failures remain. |
| linear | 2 | 2 | Resistor divider and RL lowpass now pass; RC lowpass sine/sweep pass but impulse fails; RC highpass still fails. |
| nonlinear | 0 | 5 | Standalone diode/zener/triode fixtures still fail. |
| opamp | 6 | 0 | Still clean. |
| pedals | 1 | 3 | Fuzz core passes; RAT/Big Muff THD issues remain; TS808 regressed under X1 validation and needs review. |
| reactive | 2 | 2 | LC resonant and transformer step-up pass; delay compile and transformer step-down remain. |
| stress | 1 | 0 | DC stability still passes. |

## Component-Level Readout

### Fixed/De-noised by Harness Changes

- `resistor_divider`: now passes at -118.5 dB RMS / -75.7 dB peak. The previous failure was validation harness timing/oversampling, not resistor modeling.
- `rl_lowpass`: now passes at -81.4 dB RMS / -81.5 dB peak.
- `rc_lowpass` sine: now passes at -81.4 dB RMS / -81.5 dB peak.
- `rc_lowpass` sweep: now passes in the linear suite at -49.1 dB RMS / -39.2 dB peak.
- `phase_allpass_stage` sweep: now passes.
- `unity_buffer`: now validates tightly at -99.9 dB RMS / -75.7 dB peak.

### Still Wrong at Component/Topology Level

- `rc_highpass`: sine and sweep still fail. The amplitude is close, but phase/transfer mismatch persists after startup, so this looks like a highpass topology/output extraction issue rather than a resistor/capacitor primitive issue.
- `rc_lowpass` impulse: still fails. Sine and sweep now pass, so this is likely impulse/initial-condition handling rather than steady-state lowpass transfer.
- `series_rlc`: sine improves to -29.9 dB RMS, but impulse and sweep still fail. Treat as reactive topology/initial-condition work.
- `twin_t_notch`: still fails, though sine is now -20.4 dB RMS. Needs bridge/notch topology analysis.
- `transformer_stepdown`: still fails by about +19 dB RMS/peak. This remains a likely turns-ratio direction/scaling bug. `transformer_stepup` passes.
- `transformer_load`: still fails around -6 dB, likely loading or ideal-vs-coupled transformer behavior.
- `delay_simple`: still compile-fails with `No circuit edges found`.
- `ota_ca3080`: still compile-fails in rigid fallback with unsupported nonlinear device kind.
- `diode_clipper`, `diode_no_cap`, `single_diode`, `zener_clipper`: standalone nonlinear diode fixtures still fail around 6-7 dB, with zener hard clipping much worse. Canonical `single_diode` still passes exactly, so fixture topology/reference differences remain important.
- `common_cathode_12ax7`: still fails at 7-9 dB RMS.
- `fuzz_face_pnp`: now fails both clean and saturated at about 15.5 dB RMS / 20.7 dB peak.
- `rat_clipper` and `bigmuff_stage` clean: still primarily THD/metric failures despite tiny time-domain error.
- `ts808_clipper`: now fails under X1 runtime validation. This needs investigation because it previously passed with the default runtime oversampler; likely the compiled circuit behavior depends on runtime oversampling.

## Test Results

Passing:

```text
cargo test -p pedalkernel-validate spice::tests --lib
```

Failing but improved:

```text
cargo test -p pedalkernel --test product_regression_matrix
```

The product regression matrix no longer fails on the missing `Stage::KMethod` match arm. It now reaches existing product-level failures:

- `product_pedal_controls_are_mapped_from_pro_pedals`: AcidBath TB303 Filter control mapping gap.
- `product_controls_bind_to_runtime_stages`: later compile failure for an unsupported nonlinear device kind in rigid MNA.

## Next Best Targets

1. Fix `rc_highpass` extraction/topology. It is the smallest remaining passive transfer mismatch.
2. Fix `delay_simple` compile path. It is a hard compile failure and should be easy to isolate.
3. Fix transformer step-down turns-ratio/scaling. The failure is large and asymmetric with step-up passing.
4. Investigate why standalone diode fixtures differ from canonical single diode.
5. Decide whether validation should compare runtime-X1 or production-oversampled behavior for pedal cores like TS808.
