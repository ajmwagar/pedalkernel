# Performance & benchmarks

PedalKernel circuits are designed for real-time performance. This page shows indicative CPU usage and scaling behavior.

Numbers should be read as order-of-magnitude, not promises. They depend on the host CPU, compile flags, sample rate, and how the circuit is cascaded. Run `cargo bench --bench wdf_bench` on your machine for the authoritative measurement.

## Pedal benchmarks

Measurements on an Apple M-series chip at 48 kHz, with default compile options (no oversampling, ideal tolerance):

| Pedal | CPU Budget | Samples/sec | Realtime × |
|-------|------------|-------------|------------|
| Fuzz Face | 0.04% | 122M | 2500× |
| ProCo RAT | 0.3% | 16M | 330× |
| Dyna Comp | 0.4% | 12M | 252× |
| Boss CE-2 (BBD) | 0.5% | 9.9M | 207× |
| Tube Screamer | 1.2% | 3.8M | 80× |
| Klon Centaur | 1.4% | 3.5M | 72× |
| Big Muff | 1.7% | 2.8M | 59× |
| Blues Driver | 5.9% | 818K | 17× |

**CPU Budget** = fraction of the real-time budget consumed. Under 100% means glitch-free audio.

Pedals not shown in the table (SD1, Fulltone OCD, Boss DM-2, Memory Man, Phase 90, and the tube amps and synth modules) have not been benchmarked in this pass but fall within the same envelope.

## FLOPS breakdown

WDF processing cost per sample depends on circuit complexity:

| Element | Est. FLOPs |
|---------|------------|
| Series adaptor | 9 |
| Parallel adaptor | 10 |
| Capacitor | 2 |
| Diode Newton solve | 220 (55 × 4 iterations) |
| **Typical clipper stage** | **~250 total** |

A Tube Screamer (1 clipping stage) needs ~12 MFLOPS at 48 kHz. A Big Muff (4 cascaded stages) needs ~48 MFLOPS.

## BBD depth

The BBD model (Boss CE-2) includes full analog bucket-brigade emulation:

- **Companding** — NE571-style 2:1 compression/expansion with mismatched attack/release (causes "breathing")
- **Charge leakage** — per-stage loss accumulates across 1024+ stages, darkening long delays (the Memory Man warmth)
- **Clock feedthrough** — parasitic capacitance couples switching clock into signal (the characteristic BBD whine)
- **Bandwidth limiting** — Nyquist limit at half clock frequency with anti-alias LPF
- **Soft clipping** — BBD voltage swing saturation with cubic waveshaping

Despite this, BBD overhead is modest (~0.5% CPU) because most operations are simple one-pole filters.

## Scalability

Cascaded WDF stages scale linearly:

| Stages | ns/sample | CPU @ 48kHz |
|--------|-----------|-------------|
| 1 | 173 | 0.8% |
| 2 | 268 | 1.3% |
| 4 | 592 | 2.8% |
| 8 | 1249 | 6.0% |

## Sample rate scaling

The Tube Screamer at various sample rates:

| Sample Rate | Realtime × |
|-------------|------------|
| 44.1 kHz | 90× |
| 48 kHz | 82× |
| 96 kHz | 42× |
| 192 kHz | 21× |

## Running benchmarks

Run the full benchmark suite yourself:

```bash
cargo bench --bench wdf_bench
```

This gives you precise CPU timing on your machine, sample rates, and pedals of your choice.
