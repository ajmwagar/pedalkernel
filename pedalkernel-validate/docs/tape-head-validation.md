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

The head is the only nonlinear element, wired as a **shunt** load on the output
node: `in -[R_pri 470]-> out`, with `R_load 2.2k` and the `tape_head` BOTH from
`out` to ground (the head sinks `i(V) = Gp·V + Isat·M/Ms` at `out`, exactly as
the ngspice deck models it). It colours on its gap **voltage** (`H = kv·V`), so
it reaches the J-A knee at ~1 V line level and THD climbs into the knee — unlike
the current-driven transformer core (`magnetics_external` suite).

## Status: ACTIVE — PASSES the reference (2026-06-15)

The ngspice golden was generated on 2026-06-15 (the deck was ported off
LTspice-only constructs — `sdt()` → a native 1 F capacitor integrator, and the
monolithic nested-ternary `Bslope` decomposed into intermediate behavioural
nodes). The test is **active** (`pending_reference: false`) and **passes** the
gate at the match tolerances in `src/config.rs`.

Measured WDF-vs-ngspice agreement (96 kHz × 4, shunt topology):

| Signal | RMS err | Peak err | Spectral | THD err |
| --- | --- | --- | --- | --- |
| clean (0.1 V, below knee) | −72 dB | −73 dB | 0.2 dB | 0.01 dB |
| saturated (2.0 V) | −70 dB | −70 dB | 0.0 dB | 0.0 dB |

### History — the earlier "engine-side divergence" was a netlist bug

Before 2026-06-15 this test reported a large divergence (clean RMS −4.1 dB,
saturated spectral 33 dB) and was suspected to be an engine/model defect (WDF
port-`Rp` adaptation; discrete H-step integration vs ngspice's continuous
integral). Bead **pedalkernel-x0mv** Phase 0 instrumented the compiled circuit
against the golden sample-by-sample and **disproved both hypotheses**: the
`.pedal` was wiring the head in **series** (`R_pri.b → TH.a`, `TH.b → out`) while
the golden deck models it as a **shunt** at `v_out`. The series wiring's clean
gain (0.459 = 2200/(470+2125+2200), head incremental R ≈ 2.1 kΩ) matched the WDF
output exactly, while the shunt gain (0.697) matched the golden — pinning the
failure on the netlist, not the element. Rewiring the `.pedal` to the shunt
topology the deck documents made the WDF output match the golden to ~70 dB on
both labels, with no change to the J-A element.

Because this 470/2.2k divider is stiff (≈390 Ω at `out`), the head is a light
shunt load and output THD is modest (≤~1.2%, peaking at the ~1 V knee and
receding at 3 V as the magnetic current clamps to a near-constant offset). This
is faithful to the ngspice ground truth for the same circuit — it is a
mild-saturation reference, not a hard-clipping one.

## Regenerating the golden (requires ngspice)

ngspice may not be preinstalled in the sandbox (`apt-get install -y ngspice`).
With ngspice on `PATH`:

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

> **Tolerances.** The `pass_criteria` registered for this test
> (`normalized_rms_error_db: -6`, `peak_error_db: -6`, `thd_error_db: 3`) are
> sized like the other WDF-vs-SPICE nonlinear tests. The element clears them with
> ~64 dB of margin (measured −70 dB RMS), so they are deliberately left as-is
> rather than tightened to the measured floor — the head matches the reference.

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
