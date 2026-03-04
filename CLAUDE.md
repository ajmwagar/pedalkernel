# PedalKernel Development Guide

## Core Philosophy: True-to-Circuit Modeling

PedalKernel models guitar pedal circuits using Wave Digital Filters (WDF). The fundamental principle is:

**DSL → Circuit Understanding → Accurate Tone**

We do NOT add shortcuts, hacks, or "fake DSP fixes" to make things sound right. If the output doesn't match the expected behavior, the fix is to improve how we model the actual circuit components and topology—not to special-case certain pedal types.

### What This Means in Practice

**ACCEPTABLE:**
- Improving WDF element models (diodes, transistors, op-amps, JFETs)
- Adding new component types with accurate electrical models
- Fixing how circuit topology is analyzed and converted to WDF trees
- Correcting component parameter values to match datasheets

**NOT ACCEPTABLE:**
- Adding `if is_phaser` or `if is_fuzz` special cases
- Hardcoding DSP effects that bypass the circuit model
- Using component IDs to trigger special behavior (e.g., `if comp.id.contains("lfo")`)
- Any "it sounds wrong so let's add a filter/gain/effect" fixes

### The Pipeline

```
.pedal file (DSL)
    ↓
Parse → PedalDef (components + nets)
    ↓
CircuitGraph (nodes, edges, topology analysis)
    ↓
WDF Tree (series/parallel decomposition)
    ↓
Nonlinear Roots (diodes, JFETs, transistors)
    ↓
CompiledPedal (real-time processor)
```

Each stage must faithfully represent the circuit. If a pedal doesn't sound right, trace back through this pipeline to find where the model diverges from reality.

## Current Limitations (To Be Fixed Properly)

### Op-Amp Modeling

Op-amps are currently treated as "bridge edges" for graph connectivity but don't process signal in the WDF tree. This breaks circuits that depend on op-amp behavior:

- **Unity-gain buffers** (used in all-pass filters, phasers)
- **Inverting amplifiers** (gain stages)
- **Integrators/differentiators** (filters, oscillators)

**Proper fix:** Add op-amp WDF elements that model:
- Input impedance (very high)
- Output impedance (very low)
- Voltage gain (open-loop → closed-loop via feedback network)
- Slew rate limiting (already partially implemented)

### JFET Variable Resistance

JFETs in phasers act as voltage-controlled resistors, not clipping elements. The current JFET model focuses on the Shockley equation for drain current, but for phaser applications we need:

- Accurate Rds(on) vs Vgs relationship
- Proper triode-region modeling for small Vds

**Proper fix:** Extend JfetRoot to handle variable-resistance mode when Vds is small.

## Testing Practices

### Unit Tests

Every component model should have unit tests verifying:

1. **DC behavior** - Correct operating point
2. **AC behavior** - Frequency response matches theory
3. **Nonlinear behavior** - Clipping, saturation at correct levels
4. **Edge cases** - Extreme parameter values don't cause NaN/infinity

Example:
```rust
#[test]
fn jfet_variable_resistance_region() {
    // When Vds << Vp, JFET should act as voltage-controlled resistor
    let model = JfetModel::n_2n5952();
    // Test Rds varies with Vgs as expected
}
```

### Integration Tests

Each pedal type should have tests verifying:

1. **Compiles without error**
2. **Produces finite output** (no NaN/infinity)
3. **Signal passes through** (output RMS > threshold)
4. **Controls affect output** (changing knobs changes sound)

Example:
```rust
#[test]
fn phase90_speed_controls_sweep_rate() {
    let pedal = parse("phase90.pedal");
    let mut proc = compile_pedal(&pedal, 48000.0).unwrap();

    // Measure sweep rate at Speed=0.2 vs Speed=0.8
    // Verify higher speed = faster modulation
}
```

### Running Tests

#### Library Unit Tests (685 tests)
```bash
cargo test -p pedalkernel --lib
```
Core compiler, WDF tree, element models, NR solver, DSL parser.

#### Integration Tests

```bash
# Control binding pipeline — verifies 6-step bind produces correct pot/switch mappings
cargo test --test control_binding          # 34 tests

# Stage structure — checks compiled stage counts, types, control bindings
cargo test --test stage_investigation      # 5 tests

# Scattering sanity — property-style adaptor KCL/KVL validation
cargo test --test scattering_sanity        # 2 tests

# Pedal diagnostics — aliasing, sample-rate scaling, solver convergence
cargo test --test pedal_diagnostics        # 7 tests

# Golden regression — deterministic output locked to golden .npy snapshots
cargo test --test golden_regression        # 1 test (+ 1 ignored regenerate helper)

# BJT circuits (Fuzz Face multi-NL)
cargo test --test integration_bjt          # 21 tests

# Tube circuits (triodes, pentodes, push-pull)
cargo test --test integration_tubes        # 48 tests

# Full pedal compile-and-process
cargo test --test integration_pedals       # 15 tests

# Other integration suites
cargo test --test integration_engine       # Engine processing tests
cargo test --test integration_clipping     # Diode clipping circuits
cargo test --test integration_opamp_jfet   # Op-amp and JFET circuits
cargo test --test integration_compressor   # Compressor circuits
cargo test --test integration_lfo          # LFO and modulation
cargo test --test integration_bbd          # Bucket Brigade Device (requires bbd feature)
cargo test --test nonlinear_solver         # NR solver convergence tests
cargo test --test invariants               # Compiler invariant checks
```

#### SPICE Golden Reference Validation (36 tests)

The `pedalkernel-validate` binary compares WDF output against SPICE golden references with dB/RMS/spectral metrics. **Must be run from the `pedalkernel-validate/` directory** (circuits and golden .npy files are relative to CWD).

```bash
cd pedalkernel-validate

# Run all 7 suites with detailed metrics table (RMS dB, Peak dB, THD, Spectral)
cargo run --release --bin pedalkernel-validate -- --detailed run --suite all

# Run a specific suite
cargo run --release --bin pedalkernel-validate -- --detailed run --suite nonlinear
cargo run --release --bin pedalkernel-validate -- --detailed run --suite pedals

# Run a single test within a suite
cargo run --release --bin pedalkernel-validate -- --detailed run --suite active --test fuzz_face_pnp

# List all available suites and tests
cargo run --release --bin pedalkernel-validate -- list

# Generate JSON report
cargo run --release --bin pedalkernel-validate -- --report report.json run --suite all
```

**Validation suites:**
| Suite | Tests | Description |
|-------|-------|-------------|
| `linear` | 4 | RC/RL filter transfer functions |
| `nonlinear` | 5 | Diode clipper, cathode 12AX7, zener |
| `opamp` | 6 | Unity buffer, inverting/noninverting gain, integrator |
| `active` | 10 | BJT, JFET, MOSFET, OTA, triode, push-pull |
| `pedals` | 4 | Fuzz core, RAT clipper, Big Muff, TS808 |
| `reactive` | 4 | Delay, LC resonant, transformer step-up/down |
| `stress` | 1 | DC stability |
| `canonical` | 8 | Systematic WDF failure mode coverage (RC LP/HP, RLC, Twin-T, transformer, diode, op-amp clipper, BJT diff pair) |

**Metrics reported:**
- **RMS Error (dB)** — `20*log10(rms(wdf-ref)/rms(ref))`
- **Peak Error (dB)** — `20*log10(max|wdf-ref|/max|ref|)`
- **THD Error (dB)** — Total Harmonic Distortion difference
- **Spectral Error (dB)** — Frequency-domain magnitude difference (bins >-80dB of peak)

**Golden reference management:**
```bash
# Bootstrap golden refs from current WDF output (for regression locking)
cargo run --release --bin pedalkernel-validate -- bootstrap --suite all

# Generate analytical refs for linear circuits (no external deps)
cargo run --release --bin pedalkernel-validate -- generate-linear

# Generate from ngspice (requires `brew install ngspice`)
cargo run --release --bin pedalkernel-validate -- generate-spice --suite all --spice-dir spice-circuits

# Check ngspice availability
cargo run --release --bin pedalkernel-validate -- check-spice
```

**Data locations:**
- Circuit definitions: `pedalkernel-validate/circuits/{suite}/{test}.pedal`
- Golden references: `pedalkernel-validate/golden/{suite}/{test}/{signal}.npy`
- SPICE netlists: `pedalkernel-validate/spice-circuits/{suite}/{test}.spice`

#### pedalkernel-pro Tests (run from pedalkernel-pro/)
```bash
# Fairchild 670 compression tests
cargo test -p fairchild-670

# Full workspace tests (may fail if resvg dep is broken)
cargo test --workspace
```

### A/B Testing Against Reference

For critical pedals, record reference audio from:
- SPICE simulation of the exact circuit
- Actual hardware (if available)

Compare PedalKernel output using the `pedalkernel-validate` binary (see above) or manually:
- Frequency spectrum analysis
- THD measurements at various input levels
- Transient response comparison

## Adding New Components

When adding a new component type:

1. **Research the electronics** - Find datasheets, application notes, SPICE models
2. **Understand the WDF representation** - How does this element fit in the WDF framework?
3. **Implement the model** - Add to `elements/` with proper documentation
4. **Add to DSL parser** - Update `dsl.rs` with syntax
5. **Add to compiler** - Handle in `compiler.rs` graph building
6. **Write tests** - Unit tests for the element, integration test with a pedal
7. **Document** - Update DSL.md with syntax and usage

## Debugging Circuit Issues

When a pedal doesn't sound right:

1. **Verify the .pedal file** - Does it match the schematic exactly?
2. **Check component values** - Are resistors/caps in correct units?
3. **Trace the signal path** - Use debug output to see signal at each stage
4. **Compare to SPICE** - Simulate the same circuit, compare waveforms
5. **Isolate the problem** - Simplify the circuit to find which element is wrong

**Never** add a hack to fix the symptom. Find and fix the root cause.

## Code Style

- Use descriptive names that match electronics terminology
- Comment with circuit theory, not just code explanation
- Reference datasheets and application notes where relevant
- Keep WDF math separate from circuit topology logic

## File Structure

```
pedalkernel/
├── src/
│   ├── dsl.rs          # DSL parser
│   ├── compiler.rs     # Circuit → WDF compilation
│   ├── elements/
│   │   ├── mod.rs
│   │   ├── nonlinear.rs   # Diodes, JFETs, transistors
│   │   ├── modulation.rs  # LFO, envelope followers
│   │   └── ...
│   └── tree.rs         # WDF tree structures
├── examples/
│   └── pedals/         # Example .pedal files for testing
└── tests/
```

## Remember

The goal is to understand circuits well enough that the DSL description naturally produces the correct tone. If you find yourself writing `if pedal_type == "phaser"`, stop and ask: "What's actually different about this circuit that my model isn't capturing?"
