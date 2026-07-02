# DC Operating-Point Solve — literature review + design guidance

Research distilled for the pedalkernel WDF engine. Motivating problem: a multi-BJT
discrete amp (Neve BA283) whose WDF DC operating point disagrees with ngspice —
ngspice's `.op` is provably **not** a fixed point of our DC equations (seeding it
makes our solver walk away to a different root).

## 0. The diagnostic that reframes everything

Newton-Raphson **cannot leave a true zero**: if `F(x*) = 0`, a full NR step from
`x*` has length zero. So if our solver moves away from a seeded ngspice op-point,
then **ngspice's operating point is not a zero of *our* residual `F`** — our DC
equations differ from ngspice's (device model, a Norton/source term, a stamp sign,
or a Vt/temperature mismatch). This is exactly the Norton/source-term confusion
flagged in our solver-convention memory (`I_port = a/(2Rp) + i_device`).

**Consequence:** root-selection aids (`.nodeset`/`.ic`, homotopy) only steer NR
among the roots of *your own* equations; they cannot rescue a formulation whose
root set doesn't contain the physical bias. **Verify `F(ngspice_op) ≈ 0` first**;
if not, fix the equations before touching convergence/root strategy.

## 1. The core algorithm

- **MNA** (Ho–Ruehli–Brennan 1975): node voltages + a few branch currents.
- At DC, caps open / inductors short → nonlinear algebraic `F(x)=0`.
- **Newton-Raphson** (Nagel SPICE2, 1975): each nonlinear device → **companion
  model** = conductance `G=∂i/∂v` ∥ current source `I_eq = i(x_k) − G·v_k`;
  assemble → linear MNA system `J·Δx = −F`; solve (LU); update; relinearize.
  Iteration count (not matrix size) dominates DC cost. SPICE NR cap ITL1=150.

## 2. Why plain NR fails on junctions, and the aids (in order tried)

Junction current `i = Is(exp(v/Vt)−1)` changes by `e` per ~26 mV → linear
extrapolation overshoots / overflows → oscillation/divergence. SPICE layers:

1. **Junction/voltage limiting** (always on; `pnjlim`, `limvds`): clamp per-iter
   ΔVbe near the critical voltage `vcrit = Vt·ln(Vt/(√2·Is))`. *(This is #185's
   pnjlim.)* Cures single-step overshoot; not global convergence by itself.
2. **Gmin stepping** (homotopy): shunt every node to ground with `gmin`, ramp it
   `large → tiny` warm-starting each solve. Cures high-Z high-gain nets with no
   good cold basin.
3. **Source/supply stepping** (continuation): ramp independent sources `0 →
   nominal`, warm-starting each step. At 0 V everything is 0 (trivial root); the
   bias propagates stage-by-stage as rails rise. **Textbook fit for multi-stage
   discrete amps.** Doubles as the correct-branch selector (tracks the power-on
   branch).
4. **Pseudo-transient continuation** ("dynamic gmin"): attach fictitious 1 F caps,
   integrate a damped transient to steady state. Most robust, slowest; last
   resort. Convergence theory: Kelley & Keyes, SIAM J. Numer. Anal. 35(2), 1998.

Representative order (SIMetrix): junction-init NR → source stepping → diagonal
gmin → junction gmin → pseudo-transient. ngspice: NR(+limiting) → gmin → source →
transient op. All are **continuation/homotopy** methods parameterizing `F` by a
scalar λ and tracking from an easy λ to the real circuit.

## 3. Root selection in multi-stable circuits

NR lands the root in its starting basin. Bistable circuits have ≥2 stable nodes +
an unstable saddle between them; SPICE can report a saddle or the wrong branch.
- **`.nodeset`**: soft initial guess — stiff resistor to the value for the first
  solve, then **removed and re-solved** → a true equilibrium near the hint.
- **`.ic`**: harder clamp (held in `.tran UIC`); can pin a chosen state.
- **Homotopy from 0** (source stepping) is the principled way to reach the
  *physically intended* (power-on) bias. Saddle vs stable is decided by the
  Jacobian eigenvalues / small-signal dynamics.

## 4. Compile-time vs runtime (the VA/WDF consensus)

The DC operating point is a property of **topology + bias network**, not the audio
signal. **Solve it once (offline/at instantiation); seed it; do NOT re-find it at
runtime.** Runtime = a *local* per-sample correction warm-started from the previous
sample (~1 NR step), not a global homotopy.

- **K-method** (Borin/De Poli/Rocchesso, IEEE TSAP 2000): tabulate/solve the
  instantaneous nonlinearity around a state embedding the bias.
- **DK-method** (Yeh/Abel/Smith TASLP 2010; Holters/Zölzer EUSIPCO 2015, DAFx-17):
  per-sample NR warm-started from the previous sample → 1–2 iters once biased;
  expensive global DC find only at init.
- **Nonlinear WDF** (Werner; Bernardini/Sarti): prefer explicit/closed-form
  (Lambert-W) scattering to avoid per-sample iteration; multiport → per-sample
  local NR at the root, warm-started.

### Recommendation for pedalkernel

1. **Compile/instantiation time:** solve the group DC op-point with full robust
   machinery — NR + junction limiting (have pnjlim) + **source stepping from 0 V**
   (primary globalizer *and* correct-branch selector for discrete amps), gmin /
   pseudo-transient as fallbacks. Afford 150+ iters here; it's off the audio
   thread. This is `solve_bjt_group_dc_qpoint`'s job.
2. **Persist** that op-point as the seed: initial wave variables, cap states, and
   the linearization/tabulation point.
3. **Runtime:** never re-find DC. Run audio as a perturbation; if a port needs
   per-sample iteration, warm-start from last sample (limiting still on); prefer
   explicit scattering where possible.
4. **Trade-off:** solve-once-and-seed is the sweet spot for fixed-topology pedals.
   A control that changes the *bias network* (sag, bias trim) → **off-thread
   re-solve + re-seed**, not a runtime continuation that risks the audio deadline.

### Two-layer plan for the BA283 (and the discrete-amp family)

- **Layer A — formulation:** make `F(ngspice_op) ≈ 0`. The `op{}` block emits this
  residual automatically; the per-port value localizes the wrong term (suspect:
  the Norton/`dc_bias` source term at the feedback-coupled base). **Nothing else
  matters until this holds.**
- **Layer B — solve:** once A holds, add source stepping from 0 to the compile-time
  solve so it lands the right root; validate per-device against ngspice `.op`.

### 2026-07-01 — the op is a discrete-time REPELLER; confirmed cause + cure design

Follow-up to the map-Jacobian probe (`ba283_persample_map_jacobian`,
`PK_BA283_JMAP=1`). The probe linearizes the WDF per-sample map at ngspice's
correct op (servo OFF, full `op{}`+`nodes{}` seed) and prints the cap-state
Jacobian `Jcap`; offline `numpy.linalg.eig` gives the spectral radius **ρ**.

**Baseline reproduced (this branch):** `ρ(Jcap) = 2743`, dominant eigenvector on
**Cmil (0.67) ⊕ Ccmp (0.74)** (the 220p/330p HF caps around TR1's Miller loop);
the other two caps are benign (Cfb small; Cin = 10µF gives λ ≈ 1.000, a marginal
coupling-cap mode). One-sample drift of the seeded op ≈ 4.7 V; static per-port
DC-balance residual `max|F(seed)| = 0.0404 V` at the TR1-collector port. So the
correct op is a **stable fixed point of the continuous circuit but a repeller of
the WDF sample-to-sample map**, localized to the two stiff HF caps.

**Mechanism, pinned to code (confirmed, not hypothesised):**

- A WDF capacitor is a *wave unit delay*: `wdf_reflected` returns
  `wave_cache[slot].wave_state` = the previous incident `a[n−1]`
  (`pedalkernel-rt/src/boundary_math.rs:225-231`), with reference resistance
  `Rp = 1/(2·fs·C)` (`boundary_math.rs:138`) — trapezoidal. For Cmil 220p that is
  ~52 kΩ; Ccmp 330p ~34 kΩ.
- In `MultiNlStage::process` the caps are treated **semi-explicitly**: each sample
  reads `b_passive[k] = wdf_reflected(...)` (the delayed `a[n−1]`) and feeds it as
  a **fixed** term into `known_a[i]` via `s_nl_passive`
  (`pedalkernel-rt/src/stage.rs:6383-6429`). The grouped-NR
  (`solver.rs::multi_port_nr_solve_grouped_into`) solves **only the BJT device
  port voltages** — the caps are *not* unknowns. After the solve, scatter-down
  gives the new incident `a`, and `wdf_set_incident` stores `wave_state = a`
  (`stage.rs:6620-6622`).
- Net: the map `wave_state[n] → wave_state[n+1]` closes a **z⁻¹ (wave delay)**
  around TR1's high-gm common-emitter Miller feedback. With the two stiff caps'
  huge `Rp` sitting on that high-impedance node, the discrete loop gain is > 1 →
  `ρ = 2743`. This is the classic "delay-in-the-loop" instability.

**Why the earlier Backward-Euler damping only reached ρ ≈ 261 (10×, insufficient):**
softening the cap's discretization constant while it *remains a delayed wave
source feeding `known_a`* leaves the z⁻¹ in the loop — it only lowers the gain,
it does not remove the pole. The delay must be **removed**, i.e. the stiff caps
must be solved *simultaneously* with the devices.

**Cure — Part 1, implicit stiff-cap fold (runtime-feasible; NO adaptor rebuild):**

Move the stiff caps (`C < 1 nF`) out of the delayed `passive_one_ports` set and
into the grouped-NR as **coupled unknowns with a linear Backward-Euler companion
device**: `i_c(v) = G_c·(v − v_prev)`, `G_c = C·fs_eff`, `di/dv = G_c`. Key point
that makes this tractable without rebuilding the R-adaptor scattering `S`:

- The cap's **reference resistance may stay at the existing `1/(2·fs·C)`** — a WDF
  port's reference resistance only scales its wave variables; the *solved* node
  voltage `v_c` and current `i_c` are physically correct BE values regardless of
  the reference `Rp`. So the fold is a **port re-partition** ([NL ∪ stiff] become
  the solved set), not a new `S`.
- Because `G_c` now enters the Newton Jacobian **at sample n** (solved with the
  BJTs), the stiff conductance is *in the loop instantaneously* → the discrete
  pole moves to the stable continuous location. The z⁻¹ survives only on the
  companion source `I_eq = G_c·v_prev`, wrapped by the stabilizing conductance ⇒
  ρ expected < 1.

Concrete implementation surface:

1. **Extended system** size `m = n_nl + n_stiff`. The solver needs the `S`
   sub-blocks over the *stiff rows* too — `S[stiff][nl]`, `S[stiff][stiff]`,
   `S[stiff][other_passive]`, `S[stiff][adapted]` — which `MultiNlStage` does
   **not** currently retain (only the `n_nl` rows: `s_nl`, `s_nl_passive`,
   `s_nl_adapted`, `stage.rs:4686-4693`). Either (a) retain the extra rows at
   build time from the full matrix before `from_full_matrix`, or (b) reconstruct
   raw `S` at runtime from `RTypeAdaptor.power_scattering` + `sqrt_r`/`inv_sqrt_r`
   (`tree.rs:207-214`, currently private — add accessors; confirm the adaptor's
   port ordering is `[NL, passive, (vcc?), adapted]`, the same order
   `from_full_matrix` assumes).
2. **New solver path** `multi_port_nr_solve_grouped_with_caps_into` (or append a
   `LinearCapCompanion` 1-port `NlDeviceGroupIv` after the real device groups).
   Keep it a *separate* function reached only when stiff caps are present ⇒ the
   whole non-BA283 corpus stays byte-identical.
3. **State**: keep `v_c` in the cap's existing `wave_cache[slot].wave_state`
   (re-interpreted as `v_prev`), updated to the solved `v_c` after the NR. This
   leaves the map-Jacobian probe's cap enumeration unchanged, so re-running
   `PK_BA283_JMAP=1` measures the *new* map's ρ directly (before→after
   comparable).
4. **Gate**: a multi-BJT (≥2 BJT device groups) DC-coupled-feedback group that
   contains ≥1 stiff cap (`C < 1 nF`). Non-matching stages take the existing
   path. Verify byte-identical corpus with the gate off.

**Cure — Part 2, bias reformulation (unchanged direction):** make `F(ngspice_op) =
0` by fixing the Norton `dc_bias` source-term extraction at the feedback-coupled
base (localized by the probe residual to the TR1 collector / Q3 base, R2-56k).
HARD CONSTRAINT (documented regression trap): do **not** fold device small-signal
Jacobians into the linear source term. With `F(op)=0` *and* the implicit caps
(ρ<1), the op is a stable discrete fixed point ⇒ the DC servo becomes removable.

### 2026-07-01 (final) — BA283 DC bias CLOSED (ΔIc 0.0%); AC gap localized to output loading

The "base-current precision" hypothesis was tested and the DC gap fully closed —
but the mechanism was NOT a 1-2µA Ib formulation error. Diagnostic-first
(`pedalkernel-validate/tests/ba283_ib_diagnostic.rs`, permanent):

1. **Ib diagnostic** (Layer 1/2): our Gummel-Poon Ib at ngspice's exact `.op`
   junction voltages. The Ib EQUATIONS were already SPICE-exact (Ibe1/BF + ISE·
   (exp(vbe/(NE·Vt))−1) + Ibc1/BR + ISC-term, no qb division; RB/RE/RC terminal
   decomposition correct). The entire diff was **thermal voltage**: `vt: 0.02585`
   (25 °C) vs ngspice's TEMP=TNOM=27 °C with CODATA-2018 constants ⇒
   `Vt = 1.380649e-23·300.15/1.602176634e-19 ≈ 25.8649 mV`. With that Vt the
   per-device diff collapses to |ΔIb| ≤ 0.0006 µA, |ΔIc| ≤ 0.001 % (gate was
   0.2 µA). Fixed in `model_lookup::bjt_from_spice` (`SPICE_VT_27C`).
2. **Deck-KCL audit + standalone deck DC solve** (Layer 3/4): with the exact
   model, a 6-node Newton solve of the DECK's DC lands ngspice's op to <0.1 mV —
   so the remaining compile/runtime offset was NETWORK, not device. Two bugs:
   - `solve_bjt_group_dc_qpoint` stamped the pot RU1 at the FULL 4.7k track
     (`Potentiometer::resistance()` = max_r) instead of position-scaled 2350
     (the `make_leaf`/spqr_build 0.5 convention) → q-point ~4 mV off.
   - `apply_bjt_dc_qpoint`'s `resolve()` treated the input node as floating ⇒
     **Cin seeded + inverted at 0 V instead of −1.03 V**. F(seed) was
     self-consistently 0 at compile but NOT a steady state of the cap dynamics:
     Cin charged over ~0.2 s and dragged the stage from the exact q-point to the
     starved root (Q3 0.6417→0.663, ΔIc +46/−75 %) — the cold-path twin of the
     `nodes{}`-seed Cin bug fixed in `apply_cap_seed`. Fixed: in/out nodes
     DC-reference ground; caps seeded with `wdf_seed_dc_voltage` (a=b=v) and
     `passive_b` re-read from `wdf_reflected` (mirrors the proven hold path).

**Result:** cold compile + silence, servo untouched OR `PK_SERVO_DISABLE=1`:
Q3 Vbe pinned at 0.64167 bit-stable for 48 000 samples; `bias_accuracy` MATCH
**ΔVbe 0.0000 V, ΔIc 0.0 %** on all three devices (verdict **BIAS OK**);
F(ngspice_op) = 0.0001 V; ρ(Jcap) = 0.9994 < 1; deck-KCL at the settled root
≤ 0.016 µA.

**AC verdict — the THD gap is NOT bias/device/solver: it is the missing output
load.** ngspice tran (0.1 V @ 1 kHz) shows the −16 dB THD comes from Q1/Q2 being
driven into **cutoff clipping** every cycle (vbe1 swings +0.547→−0.152 V) by the
10k load current demand through Cout. Our compiled BA283 puts Cout+RL in a
downstream stage whose impedance never reflects back into the MultiNl stage —
and ngspice with **RL=1G reproduces our sim to 3-4 decimals** (vbe swings
identical; vout ±0.94 vs our ±0.92; THD −52.4 vs our −52.9 dB; fundamental 0.938
vs 0.926 V). So the WDF engine is now quantitatively exact at the topology it
simulates; the remaining `ac_accuracy` LEVEL +2.7 dB / ΔTHD ~37 dB / tilt ~4.8 dB
is entirely the **cross-stage output-loading architecture gap** (same family as
the LA-2A GAP F transformer step-down). Fix path: reflect the downstream stage's
input impedance into the NL group (or fuse Cout/RL into it) — a compiler
partitioning/boundary change, out of scope for the bias workstream.

### 2026-07-01 (superseded) — DC-solve root: Early-effect model fix + residual localization

Pursuing the remaining Layer-B gap (compile-time BJT DC solve lands Q3 over- /
Q1-Q2 under-conducting, ΔIc 46-74% vs ngspice). Decisive diagnostics (temporary,
now removed):

1. **Model vs ngspice at ngspice's exact op** (`bjt_currents_terminal` at the
   `.op` voltages): our Ic matched ngspice to 1-13%. The 13% (BC184C @ Vce=18.8)
   traced to the **base-charge Early factor being linearized**:
   `q1 = 1 + vbc/VAF + vbe/VAR` instead of SPICE's `q1 = 1/(1 − vbc/VAF − vbe/VAR)`.
   At vbc/VAF≈−0.37 the linear form over-predicts Ic ~15%. **Fixed** (commit
   `fix(bjt): SPICE Gummel-Poon base-charge q1`): all three devices now match
   ngspice Ic to ~1%; corpus lib failure set byte-identical (79); BA283 fold
   ρ=0.9994 and servo-OFF hold (ΔVbe 0.0054V) preserved; cap-seeded residual
   0.0402→0.0344V.

2. **Nodal KCL residual at ngspice's node voltages** (`solve_bjt_group_dc_qpoint`,
   `PK_DC_DIAG_NODES`): after the q1 fix, ngspice's op is a **~2µA root** of our
   nodal equations (the resistor network — incl. RU1 4.7k pot as a single a-b edge
   and Rfb 56k — is complete and correct; the earlier "1mA" output-node imbalance
   was a diagnostic node-mapping artifact at R7's midpoint). Our solve converges to
   a TRUE root (KCL 2.5e-11) that sits ~40mV away, starving the Q1/Q2 Darlington.

**Why it's not fully closed:** the residual is dominated by **base-current
(ISE/ISC recombination)** — BC184C runs β≈2-6 here, so Ib is recombination- not
transport-dominated — and a ~1-2µA Ib discrepancy is amplified by the **56k (Rfb)
/ 68k (R3) bias resistors** and the shunt feedback into the ~40mV node offset (a
2µA error × 56k ≈ 110mV). Matching ngspice's bias to <10% ΔIc therefore requires
matching the base current to <~0.2µA — a base-current-recombination calibration
that is a deeper, global-BJT change with regression risk, deferred. The collector
model and the nodal formulation are now correct; the gap is base-current precision.

### 2026-07-01 (later) — LANDED: implicit stiff-cap fold + seed self-consistency → ρ<1

Both parts implemented and validated against the probe. **ρ(Jcap) = 0.9994 < 1**
(was 2743). The dominant eigenvalues collapsed from `[2743, −312, 154, 0.9998]` to
`[0.002, 0.014, 0.881, 0.9994]`: the Cmil ⊕ Ccmp stiff modes are gone; the surviving
0.9994 is the benign Cin coupling-cap mode (the marginal `λ≈1.000` the baseline
already had).

- **Part 1 — implicit fold** (`pedalkernel-rt/src/stage.rs`): `StiffCapFold` +
  `CapCompanion` + `solve_stiff_fold`. Stiff caps (`C<1 nF`: Cmil 220p, Ccmp 330p)
  are re-partitioned out of the delayed `passive_one_ports` set into the grouped-NR
  as 1-port Backward-Euler companions `i_c=G_c·(v−v_prev)`, `G_c=1/(2·R_c)`, solved
  coupled with the BJTs. The R-adaptor `S` is NOT rebuilt — the full standard `S` is
  reconstructed from the adaptor (`RTypeAdaptor::scattering_matrix`) and sliced into
  the extended `[NL ∪ stiff]` system reusing the existing
  `multi_port_nr_solve_grouped_into`. Gated to ≥2 BJT groups + ≥1 stiff cap; every
  other stage takes the byte-identical legacy path. `v_c` is kept in the cap's
  `wave_state` slot so the probe stays before/after comparable.
- **Part 2 — seed self-consistency** (`pedalkernel/src/compiler/rigid/general.rs`,
  `apply_cap_seed`): the blocker was NOT the fold but that the probe seeds caps HOT
  to ngspice's `nodes{}` voltages while `dc_bias` was derived (`apply_dc_qpoint_seed`)
  against the compiler's own DC cap solve `dc_qpoint_passive_b`, whose **input
  coupling cap Cin sat COLD at 0 V instead of its real −1.03 V**. That inconsistency
  injected ~0.6 V of spurious `known_a` residual (`F(seed)=0.615` in BOTH the legacy
  and folded paths) so the grouped-NR could not converge at the seeded op — and
  folding caps into a NON-converging NR *amplified* ρ (2743→23611). The fix
  re-points `dc_qpoint_passive_b` at the seeded (ngspice) cap waves and re-runs the
  wave-domain Norton inversion → `F(ngspice_op)` drops **0.615 → 0.0402 V** (uses
  only `i(v*)`, no small-signal Jacobians; fires only when `nodes{}` is present, so
  production/corpus is byte-identical). This is the Part-2 directive (fix the Norton
  `dc_bias` source so the op is a fixed point), realized as removing a cap-seed↔bias
  inconsistency rather than new source math.

**Gate results:** ρ=0.9994 (✓<1); **servo-OFF hold: HOLDS**, max|ΔVbe|=0.0064 V over
48000 samples (was 4.73 V drift) ⇒ servo removable; **AC LEVEL +2.40 dB** (was
−25.13); **bias residual 0.0402 V** (Layer A→B); **corpus 1050/79 unregressed**
(79 = baseline, no fold/seed-related failures). **Still open (Layer B / deeper):**
AC **THD −36 vs −16 dB** and **tilt 5 dB**, and bias **MATCH ΔIc 46-73%** (Q1/Q2
starved vs ngspice) — the compiler's own BJT DC solve (`dc_qpoint_v`) still lands a
root ~0.04 V off ngspice's device operating point; the fold makes that op *stable*
and the seed *self-consistent*, but closing ΔIc needs the compile-time DC solve to
match ngspice (source-stepping / a better group DC solve), not the runtime map. Do
not tune `Rp`/`k_p`/BE-damping to move THD; that masks.

### 2026-06-30 — the "curvature loss" is the DC bias, not the runtime embedding

Investigation of the BA283 large-signal under-distortion (THD −36 vs ngspice
−16 dB @1 kHz, h2 ~20 dB weak, ratio `dIc/(gm·dVbe)=0.547`). The hypothesis under
test was that the **runtime WDF port embedding attenuates the reflected nonlinear
wave** (`MultiNlStage`/grouped-NR). **Instrumented and DISPROVEN as a runtime
solver bug:**

- **NR converges exactly.** Per-sample grouped-NR over the drive cycle:
  `iters ≤ 4`, `max_resid = 4.4e-5 V`, zero budget/adaptive-X2 skips. At tolerance
  `1e-6…1e-4 V` the converged `(v, i)` satisfy the exact device + port equations,
  so `b = v − Rp·i` uses the **true** Gummel-Poon current — no wave attenuation.
- **Device model matches ngspice byte-for-byte** (`BC184C`/`2N3055` params in
  `models/transistors.model` == the `.spice` deck `.MODEL` cards). Same topology,
  same models, exact solve ⇒ the runtime cannot be the divergence source.

The divergence is entirely the **quiescent DC operating point** (`bias_accuracy`):

| dev | Vbe ours/ng | Vce ours/ng | Ic ours/ng | ΔIc |
|-----|-------------|-------------|------------|-----|
| Q3 (TR1 in) | 0.663 / 0.642 | 3.61 / 5.35 | 379 / 259 µA | **+46 %** |
| Q1 (TR2 drv)| 0.498 / 0.540 | 19.8 / 18.8 | 5.0 / 19.1 µA | **−74 %** |
| Q2 (TR3 out)| 0.376 / 0.406 | 19.9 / 19.2 | 12.1 / 36.7 µA | **−67 %** |

Q3 (input) **over-conducts**; the output Darlington Q1/Q2 **starve**. `F(ngspice_op)`
is ~40 mV/port — "small", but on an exp junction 40 mV ≈ 4.6× current, which *is*
the ±50–70 % Ic error. So ngspice's op is **not** a fixed point of our DC eqns
(§0), confirmed empirically: with the DC servo OFF the solver walks away and the
stage collapses (AC level −38 dB). At the wrong bias the large-signal
curvature/compression is ~10× too weak while small-signal nearly matches (a
gm-starvation cancellation, per the servo note in `rigid/general.rs`).

- **The DC servo masks, it does not fix.** Bracketed at 1 kHz: `k_p` OFF → level
  collapses (−38 dB); `k_p=20` → THD −36; `k_p=200` → THD −50 (holding the bias
  *harder* suppresses the drive-induced compression *further*). The rectified
  bias excursion that generates the Class-A even harmonics is exactly what a
  proportional DC spring rejects — so no `k_p` reproduces ngspice's −16 dB.
- **The `0.547` ratio is a symptom of the wrong bias + servo hold**, not a
  reflected-wave scaling. All three devices read <1 (0.547 / 0.743 / 0.789).

**Verdict:** DEFER — not a clean localized runtime fix. Root cause = **Layer A**
above (the DC-bias formulation / Norton `dc_bias` source term at the
feedback-coupled base). Fix path unchanged: make `F(ngspice_op) ≈ 0`, then Layer B
source-stepping. Do **not** tune `k_p`/`Rp` to move the AC metric — it masks a
starved bias (tracked: bd `pedalkernel-opi6`).

## Primary sources

- Ho/Ruehli/Brennan, MNA, IEEE TCAS 1975 — https://ieeexplore.ieee.org/document/1084079
- Nagel, SPICE2 (ERL-M520), 1975 — https://www2.eecs.berkeley.edu/Pubs/TechRpts/1975/ERL-m-520.pdf
- Najm, *Circuit Simulation*, Wiley/IEEE 2010
- Pillage/Rohrer/Visweswariah, *Electronic Circuit and System Simulation Methods*, 1995
- SIMetrix DC Operating Point Algorithms — https://help.simetrix.co.uk/8.0/simetrix/mergedProjects/simulator_reference/topics/simref_convergence_accuracyandperformance_dcoperatingpointalgorithms.htm
- Efabless, Overcoming SPICE Convergence Issues — https://efabless.com/kb-articles/overcoming-spice-convergence-issues
- SPICE OPUS pnjlim/osdi — https://www.spiceopus.si/osdi.html
- Kelley & Keyes, Pseudo-Transient Continuation, SIAM J. Numer. Anal. 1998 — https://epubs.siam.org/doi/abs/10.1137/S0036142996304796
- Multiple DC solutions (SPICE-oriented), Applied Sciences 2023 — https://www.mdpi.com/2076-3417/13/4/2369
- Homotopy for DC op-points (Oregon State) — https://web.engr.oregonstate.edu/~karti/ece521/hom3.pdf
- HSPICE ch.6 (.nodeset/.ic) — https://web.engr.oregonstate.edu/~moon/ece323/hspice98/files/chapter_6.pdf
- Yeh/Abel/Smith DK-method, IEEE TASLP 2010 — https://ccrma.stanford.edu/~dtyeh/papers/yeh10_taslp.pdf
- Holters/Zölzer, generalized state-space, EUSIPCO 2015; automatic decomposition, DAFx-17 — https://www.dafx17.eca.ed.ac.uk/papers/DAFx17_paper_29.pdf
- Borin/De Poli/Rocchesso, K-method, IEEE TSAP 2000 — https://ieeexplore.ieee.org/document/861380
- Werner et al., Resolving WDF with Multiple Nonlinearities, DAFx-15
- Bernardini/Werner/Sarti/Smith, Lambert-W WDF, IEEE TCAS-I 2016
- Bernardini/Sarti, WDF nonlinear 3-terminal devices, CSSP 2020
