# PedalKernel SPICE Validation Harness

Dockerized validation framework for the PedalKernel WDF compiler. Runs deterministic ngspice simulations as ground truth and compares against WDF compiler output across multiple test layers.

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│  PedalKernel Compiler                                    │
│  (circuit netlist → WDF tree → sample output)            │
│                                    │                     │
│                                    ▼                     │
│                            wdf_output/*.npy              │
└────────────────────────────────┬────────────────────────┘
                                 │
                    ┌────────────▼────────────┐
                    │   Docker Container       │
                    │                          │
                    │  ┌──────────────────┐    │
                    │  │ ngspice (pinned) │    │
                    │  └────────┬─────────┘    │
                    │           ▼              │
                    │    golden/*.npy          │
                    │           │              │
                    │  ┌────────▼─────────┐   │
                    │  │ Comparison Engine │   │
                    │  │ • time domain    │   │
                    │  │ • THD            │   │
                    │  │ • spectral       │   │
                    │  │ • envelope       │   │
                    │  │ • energy balance │   │
                    │  └────────┬─────────┘   │
                    │           ▼              │
                    │   results/report.json    │
                    └──────────────────────────┘
```

## Quick Start

```bash
# 1. Build the harness
docker compose -f docker/docker-compose.yml build

# 2. Generate golden references (run once, or when circuits change)
docker compose -f docker/docker-compose.yml run --rm golden-regen

# 3. Compile your circuits with PedalKernel and place output in wdf_output/
#    (directory structure must mirror golden/ — suite/test/signal.npy)

# 4. Run validation
docker compose -f docker/docker-compose.yml run --rm validate --suite all

# 5. Check results/
cat results/validation_report_*.json | python3 -m json.tool
```

## Test Layers

| Layer | What | Reference | Catches |
|-------|------|-----------|---------|
| **0** | Primitive elements (R, C, L, adaptors) | Analytical | Adaptor math, discretization |
| **1** | Linear circuits (RC, RLC, tone stacks) | Analytical BLT / SPICE .AC | Topology decomposition, R-type scattering |
| **3** | Nonlinear circuits (diode clipper, triode stages) | ngspice .TRAN | Nonlinear model, solver convergence |
| **4** | Fairchild behavioral (gain curves, timing, M/S) | ngspice + manual specs | System integration, feedback loops |
| **5** | Stress (DC stability, rate invariance, energy) | Physics constraints | Edge cases, numerical stability |

## Determinism Guarantees

The Docker container ensures:

- **Pinned ngspice version** (built from source, version 43)
- **Fixed timestep** matching WDF sample rate (no adaptive stepping)
- **Single-threaded** execution (`OMP_NUM_THREADS=1`)
- **Fixed RNG seed** (`.OPTIONS SEED=42`)
- **Tight tolerances** (`RELTOL=1e-6, ABSTOL=1e-12`)
- **GEAR integration method** (BDF, stable for stiff tube circuits)

Golden references include SHA256 hashes of their source circuits. If a circuit file changes, the golden ref is automatically regenerated.

## Adding Test Circuits

1. Add your `.spice` file to the appropriate `circuits/` subdirectory
2. Add a test entry in `config/validation_suite.yml`
3. Run `golden-regen` to generate the SPICE reference
4. Compile with PedalKernel and place output in `wdf_output/`
5. Run `validate`

Circuit templates must use `v_in` and `v_out` as input/output nodes. The harness adds the input source and simulation control automatically.

## WDF Output Format

PedalKernel must produce `.npy` files (numpy arrays, float64) at the configured sample rate, organized as:

```
wdf_output/
├── linear/
│   └── rc_lowpass/
│       └── impulse.npy
├── nonlinear/
│   ├── diode_clipper/
│   │   ├── low_level.npy
│   │   ├── clipping.npy
│   │   └── sweep.npy
│   └── common_cathode_12ax7/
│       ├── clean.npy
│       ├── driven.npy
│       ├── sweep.npy
│       └── imd.npy
└── fairchild/
    ├── static_gain_curves/
    │   └── ...
    └── time_constant_positions/
        └── ...
```

## CI Integration

The included GitHub Actions workflow (`.github/workflows/validate.yml`) runs automatically on any PR that touches compiler code. It posts a pass/fail summary as a PR comment.
