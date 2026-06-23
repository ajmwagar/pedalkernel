# Screamer vs TS9 Hardware Reference

Date: 2026-06-23

Hardware reference:

- Interface: Scarlett 4i4 4th Gen
- Routing: output 3 -> TS9 -> input 3
- Hardware setting: knobs at noon
- Captures: `hardware-goldens/ts9_noon`

Model under test:

- `../pedalkernel-pro/pedals/legends/screamer.pedal`
- Render controls: `Drive=0.5 Tone=0.5 Level=0.5`

## Baseline

The capture path adds about 2,780 samples of latency, or roughly 58 ms at 48 kHz.

With the pro `calibrate` keyword enabled, the legend Screamer model is much hotter than the hardware return:

| Stimulus | Model vs hardware RMS | Best-fit gain | Correlation | Relative fit error |
| --- | ---: | ---: | ---: | ---: |
| `sine_1k` | +11.82 dB | -12.02 dB | 0.9783 | -13.67 dB |
| `sweep_20_20k` | +13.60 dB | -16.24 dB | 0.7379 | -3.42 dB |
| `pink_seed_1592598566` | +18.00 dB | -19.47 dB | 0.8443 | -5.42 dB |

The fuller `real/overdrive/ts808_tubescreamer.pedal` is a worse match for these captures: it is 22-32 dB hotter and has negative correlation on the sine/pink checks, so `legends/screamer.pedal` is the better starting point.

## No-Calibrate Test

Removing `calibrate` in a temporary copy does not change the aligned shape metrics, but it removes about 10.6 dB of product output normalization:

| Stimulus | Model vs hardware RMS | Best-fit gain | Correlation | Relative fit error |
| --- | ---: | ---: | ---: | ---: |
| `sine_1k` | +1.19 dB | -1.38 dB | 0.9783 | -13.67 dB |
| `sweep_20_20k` | +2.97 dB | -5.61 dB | 0.7379 | -3.42 dB |
| `pink_seed_1592598566` | +7.37 dB | -8.84 dB | 0.8443 | -5.42 dB |

This is a better reference-render level, but it should not be treated as a circuit gain-stage fix. `calibrate` is a product/output-normalization layer, not part of the TS9 circuit.

## No-Calibrate With TS9 `.pedalhw` Mod

Using `--preset "Classic TS" --mod "TS9 Conversion" --no-calibrate` against the original `legends/screamer.pedal` and companion `.pedalhw` avoids a temporary edited `.pedal` file. It confirms that disabling `calibrate` removes the product normalization, but the TS9 output-network mod still leaves the software render hotter than the hardware return:

| Stimulus | Best latency | Model vs hardware RMS | Best-fit gain | Correlation | Relative fit error |
| --- | ---: | ---: | ---: | ---: | ---: |
| `sine_1k` | 2,785 samples | +7.06 dB | -7.24 dB | 0.9795 | -13.91 dB |
| `sweep_20_20k` | 2,785 samples | +8.87 dB | -10.97 dB | 0.7858 | -4.17 dB |
| `pink_seed_1592598566` | 2,780 samples | +13.25 dB | -14.74 dB | 0.8420 | -5.36 dB |

The higher level versus the no-mod no-calibrate check is expected directionally: `TS9 Conversion` changes `R_out_g` from `10k` to `100k`, reducing the output shunt loss in the simplified model.

## Tone Mapping Check

Using the no-calibrate temporary copy with pink noise:

| Tone control | Model vs hardware RMS | Best-fit gain | Correlation | Relative fit error |
| ---: | ---: | ---: | ---: | ---: |
| 0.20 | +2.32 dB | -3.65 dB | 0.8575 | -5.77 dB |
| 0.35 | +5.60 dB | -7.00 dB | 0.8506 | -5.58 dB |
| 0.50 | +7.37 dB | -8.84 dB | 0.8443 | -5.42 dB |
| 0.65 | +8.50 dB | -10.03 dB | 0.8385 | -5.27 dB |
| 0.80 | +9.30 dB | -10.89 dB | 0.8332 | -5.15 dB |

Lower tone positions are closer in absolute level and correlation, which suggests the model's electrical tone midpoint may be brighter than the hardware at physical noon.

## Recommendation

Do not retune the pro circuit gain stages from these captures alone. Without a reamp/interface calibration, absolute level still includes Scarlett output level, pedal input loading, and return gain.

For hardware-reference renders today:

- Use `legends/screamer.pedal`, not `real/overdrive/ts808_tubescreamer.pedal`.
- Compare with product `calibrate` disabled or apply an external output trim around -10.6 dB before absolute level metrics.
- Keep shape comparisons latency-aligned at about 2,780 samples.
- Treat tone mapping as the likely next circuit/control issue, not raw clipping-stage gain.

Follow-up bead: `pedalkernel-ay6e`.
