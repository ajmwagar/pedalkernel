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

**Status / honest verdict (2026-07-01):** baseline (ρ=2743) and the exact
mechanism are confirmed at code level, and the runtime-feasible cure (no `S`
rebuild) is designed above. The extended-NR itself — the coupled solve the task
flagged as *possibly too invasive for one pass* — is **not yet landed/validated to
ρ<1**: it is a multi-file change (solver path + `S`-block retention/reconstruction
+ `MultiNlStage` fields with serde defaults + build-time detection + gating)
requiring iterative build/probe/corpus cycles that cannot be responsibly validated
in a single pass without risking a masked or regressing diff. **Remaining unstable
mode: the Cmil ⊕ Ccmp z⁻¹ wave-delay loop (ρ=2743)** — unchanged until the
extended-NR lands. Do not tune `Rp`/`k_p`/BE-damping to move ρ or the AC metric;
that masks, per the servo note above.

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
