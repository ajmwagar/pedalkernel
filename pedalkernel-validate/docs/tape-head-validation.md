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

## Status: PENDING until the golden is generated

The test ships with `pending_reference: true`. While the golden `.npy` is
missing, the runner reports it as **`[PEND]`** and **excludes it from both the
passed count and the total** in the pass-rate gate — so committing the test
before its golden exists does **not** move the `N/N passed` denominator.

Once the golden files are dropped into
`golden/tape/tape_head_saturation/{clean,saturated}.npy`, the test runs and is
compared normally **regardless of the flag** — it auto-activates into the gate
with no code change.

(A *non*-`pending_reference` test with a missing golden still **fails** loudly,
so accidental golden deletion elsewhere is still caught.)

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
