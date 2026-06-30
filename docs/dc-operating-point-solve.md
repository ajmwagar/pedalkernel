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
