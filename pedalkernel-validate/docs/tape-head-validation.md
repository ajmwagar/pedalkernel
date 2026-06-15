# Tape-head Jiles-Atherton saturation validation

Validates the shipped **voltage-driven** Jiles-Atherton tape-head element
(`pedalkernel-rt/.../jiles_atherton.rs::TapeHeadVoltageRoot`, DSL
`tape_head()` / `compiler/components/tape_head.rs::studio_head`) against an
ngspice behavioural J-A reference.

Suite: **`tape`** · test: **`tape_head_saturation`**.

| Artifact | Path |
| --- | --- |
| Engine circuit | `circuits/tape/tape_head_saturation.pedal` |
| ngspice netlist | `spice-circuits/tape/tape_head_saturation.spice` |
| Golden (generated) | `golden/tape/tape_head_saturation/<label>.npy` (`clean`, `saturated`) |
| Registry | `src/config.rs` → `default_suites()` → `tape` suite |

The head is the only nonlinear element: `in -[R_pri 470]-> tape_head -> out`,
`R_load 2.2k` to ground. It colours on its gap **voltage** (`H = kv·V`), so it
reaches the J-A knee at ~1 V line level and THD climbs with drive — unlike the
current-driven transformer core (`magnetics_external` suite).

## Status: ACTIVE — known engine-side divergence (2026-06-15)

The ngspice golden was generated on 2026-06-15 (ngspice now available locally;
the deck was ported off LTspice-only constructs — `sdt()` → a native 1 F
capacitor integrator, and the monolithic nested-ternary `Bslope` decomposed into
intermediate behavioural nodes). The test is now **active** (`pending_reference:
false`) and **deliberately failing the gate** rather than hidden — we don't gate
known failures.

Measured WDF-vs-ngspice divergence (96 kHz × 4):

| Signal | RMS err | Peak err | Spectral | THD err |
| --- | --- | --- | --- | --- |
| clean (0.1 V, below knee) | −4.1 dB | −3.7 dB | 9.4 dB | 9.57 dB |
| saturated (2.0 V) | −4.8 dB | −4.3 dB | 33.4 dB | 17.74 dB |

The reference deck faithfully models the ground-truth physics (explicit
`R_pri`/`R_load` divider + head current law `i = Gp·V + Isat·M/Ms`, same M
integral as the element), so the divergence is **engine-side**. Suspected
mechanisms (build-confirm pending): the WDF port-resistance (`Rp`) adaptation vs
the divider loading (clean-regime level error), and the element's
one-implicit-field-step-per-sample H-domain integration vs ngspice's continuous
integration through the `sign(dH)` branch kink (saturated harmonics). Tracked in
bead **pedalkernel-x0mv**; the test passes once the element matches the golden.

## Regenerating the golden (requires ngspice)

ngspice is **not** installed in the CI/dev sandbox, so the `.npy` cannot be
produced here. On a machine with ngspice on `PATH`:

```bash
cd pedalkernel-validate
cargo run -p pedalkernel-validate -- generate-spice \
  --suite tape --test tape_head_saturation
```

This simulates `spice-circuits/tape/tape_head_saturation.spice` for each test
signal (`clean`, `saturated`), measures `V(v_out)`, and writes
`golden/tape/tape_head_saturation/clean.npy` and `.../saturated.npy`.

Verify availability first with `cargo run -p pedalkernel-validate -- check-spice`.

After the goldens exist, run the gate and the test auto-activates:

```bash
cargo run -p pedalkernel-validate --no-default-features -- run
```

> **Provisional tolerances.** The `pass_criteria` registered for this test
> (`normalized_rms_error_db: -6`, `peak_error_db: -6`, THD diff disabled) are
> **placeholders sized like the other WDF-vs-SPICE nonlinear tests** — they have
> **not** been measured against a real golden. Tighten them to honest values
> once the golden is generated and the WDF-vs-ngspice deltas are measured.

## Parameter mapping (element ↔ ngspice)

The `.spice` `.param` block mirrors the `studio_head()` defaults 1:1:

| Element field | ngspice `.param` | Value |
| --- | --- | --- |
| `Ms` | `Ms` | 3.5e5 A/m |
| `a` | `Aja` | 1500 A/m |
| `alpha` | `Alpha` | 1.6e-3 |
| `k` | `Kja` | 30 A/m |
| `c` | `Cja` | 0.65 |
| `kv` | `Kv` | 8000 A/m/V |
| `Isat` | `Isat` | 200 µA |
| `Gp` | `Gp` | 80 µS |
| `h_bias` | `Hbias` | 400 A/m |
| `Rp` | `Rp` | 1 kΩ (WDF port; informational in SPICE) |

**Field-domain vs time-domain.** The element advances `M` rate-*independently*
in the field domain (`dM = (dM/dH)·dH`). The ngspice model realizes the
mathematically identical trajectory in time via the chain rule
`dM/dt = (dM/dH)·(dH/dt)` with an `sdt()` integrator. Because the J-A slope
`dM/dH` is rate-independent, `M(H)` — and therefore `V(out)` — follows the same
curve; only the integration variable changes. The J-A core math
(`Lang`/`LangD`, branch-direction `delta`, wall-motion `deltaM`, sign-preserving
denominator guard) matches the element and the `oracles/ja_validation_oracle.py`
reference.
