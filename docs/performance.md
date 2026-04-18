# Performance & benchmarks

PedalKernel circuits are designed for real-time performance. This page shows actual CPU usage and scaling behavior.

> **Last updated: 2026-04-18** — Engine: nullor-in-R-type op-amp formulation + Wright Omega diode solver.
> Op-amp stages now use the exact nullor admittance matrix, eliminating the virtual-ground heuristic.
> Diode/BJT nonlinear solves use the Wright Omega function for faster convergence on shallow slopes.

## Pedal benchmarks

Measurements on an Apple M-series chip at 48 kHz (criterion median, per-sample):

| Pedal | ns/sample | Samples/sec | CPU @ 48kHz | Realtime × |
|-------|-----------|-------------|-------------|------------|
| Tube Screamer | 2,412 | 414K | 11.6% | 9× |
| Blues Driver | 2,753 | 363K | 13.2% | 8× |
| Klon Centaur | 1,347 | 743K | 6.5% | 15× |
| ProCo RAT | 1,870 | 535K | 9.0% | 11× |
| Dyna Comp | 514 | 1.95M | 2.5% | 41× |
| Big Muff | 21,125 | 47K | 101% | <1× |
| Fuzz Face | 110,110 | 9K | >500% | <1× |
| Boss CE-2 (BBD) | — | 1.72M | 2.8% | 36× |

**CPU @ 48kHz** = fraction of 1-second wall-clock budget used to process 1 second of audio.
Values over 100% mean the circuit cannot run in real-time on this CPU without optimization.

> **Note on Big Muff and Fuzz Face:** Both pedals contain multi-NR nonlinear stages (4-stage
> clipping tree for Big Muff; BJT Ebers-Moll pair for Fuzz Face). Their solve time is
> dominated by Newton-Raphson iteration across coupled ports, which scales with convergence
> difficulty rather than circuit size. These are the hardest circuits in the suite.

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

Despite this, BBD overhead is modest (~2.8% CPU) because most operations are simple one-pole filters.

## Scalability

Cascaded WDF stages scale linearly:

| Stages | ns/sample | CPU @ 48kHz |
|--------|-----------|-------------|
| 1 | 151 | 0.7% |
| 2 | 237 | 1.1% |
| 4 | 514 | 2.5% |
| 8 | 1,053 | 5.1% |

## Sample rate scaling

The Tube Screamer at various sample rates (throughput remains ~415-422K samples/sec):

| Sample Rate | Samples/sec | Realtime × |
|-------------|-------------|------------|
| 44.1 kHz | 419K | 9.5× |
| 48 kHz | 416K | 8.7× |
| 96 kHz | 422K | 4.4× |
| 192 kHz | 415K | 2.2× |

The near-constant samples/sec across rates confirms the bottleneck is per-sample NR solve cost,
not memory bandwidth. CPU% scales proportionally because higher sample rates demand more
solves per second.

## Running benchmarks

Run the full benchmark suite yourself:

```bash
cargo bench --bench wdf_bench
```

For just the compiled pedal group (faster):

```bash
cargo bench --bench wdf_bench -- compiled
```

This gives you precise CPU timing on your machine, sample rates, and pedals of your choice.
