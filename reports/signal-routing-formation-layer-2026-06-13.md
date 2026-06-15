# Signal-Routing, the Formation Layer — Addendum 2026-06-13

Extends `reports/signal-routing-design-2026-06-12.md` (the F9+F13 `SignalFlowPlan`
design) with two newly-confirmed masks of the §3 architectural gap
(`reports/architecture-debt-2026-06-12.md`). Both were exposed by the faithful
LA-2A rebuild (`reports/la2a-1to1-rebuild-plan-2026-06-13.md`) and proven by
reproduction during the 2026-06-13 RCA session. No code written. For review,
then fold into the phasing of the parent design.

## 0. The realization: §3 has two layers, not one

The 2026-06-12 design plans **routing within already-formed groups** — it runs
"once after `find_flow_groups` (`signal_flow.rs:1220`) … before any stage is
built" and assigns each group a driven port, a probe port, and a role. It
*assumes group formation is correct*.

The two LA-2A blockers are **upstream of that** — they are **group-formation**
bugs: the wrong edges land in the wrong group *before* the plan ever runs. So
the §3 gap is two layers wearing the same heuristic-vs-principled face:

| Layer | Question | Today's heuristic | Masks |
|---|---|---|---|
| **Formation** | which edges belong to which group? | `claim_passive_edges` "claim all local passives"; `has_feedback` "any cycle ⇒ fuse rigid" | **7, 8 (new)** |
| **Routing** | within a group: where does signal enter/leave, what role? | the five local heuristics (parent doc §1) | 1–6 |

The same directed signal-flow model the parent design builds for routing
(rail-blocked **directed** BFS + active-edge direction, parent §2 inputs) is
exactly what the formation layer needs too — applied one step earlier, to
*membership* instead of *orientation*. This is an argument for one model, two
consumers, not two models.

## 1. Mask 7 — cathode-follower claim-swallow (LA-2A blocker "b")

**Symptom.** A low-output-impedance source (12BH7 cathode follower) driving the
output transformer collapses to numerical zero; a high-impedance (plate-driven)
source into the *same* transformer is healthy.

**Reproduction (full compiler, `--no-default-features`, 1 kHz, RCA 2026-06-13):**
- cathode-follower → `T_out`(4:1) → 600 Ω: **−220 dB** (test floor; truly zero)
- plate-driven → `T_out`(4:1) → 600 Ω: −43.9 dB (healthy)
- difference is **only** the source impedance.

**Mechanism (confirmed, not the transformer — `pedalkernel-rt` verified clean).**
The two circuits compile to *structurally different* processors
(`CompiledPedal::debug_dump`):
- plate-driven → **2 stages**: the transformer becomes its own 4-port
  `PassiveRType` adaptor and passes signal correctly.
- cathode-follower → **1 stage**: a single `MultiNlStage` whose surviving
  passives are two caps — **the transformer and the 600 Ω load are absent from
  the realized stage**; `output_port` reads a node the signal never reaches.

Root cause in formation: `claim_passive_edges` (`signal_flow.rs:794`) seeds its
claim set with the active device's nodes **including its output node**
(`signal_flow.rs:805-814`), triggered by `should_claim_local_passives` for any
tube/BJT/JFET/MOSFET (`signal_flow.rs:1297-1303`). For a **cathode follower the
device output node *is* the bias/operating-point node** (the cathode). The
claim heuristic — designed to pull an emitter/source/cathode operating-point
chain into the device's NL group — cannot distinguish that bias network from
the **downstream forward-signal branch** that leaves the same cathode node
(coupling cap → `T_out` → load). It claims both. The downstream transformer
branch is absorbed as "local passives," and the MultiNL builder discards the
R-type → silence.

The plate-driven topology escapes *by accident of node identity*: the plate
output node ≠ the cathode bias node, so the forward branch is not a group node
and splits cleanly into its own `PassiveRType` stage. (Confirmed: inserting a
100 Ω series resistor at the cathode does **not** help — the trigger is the
shared output/bias node, not the absence of a series R.)

**This is not a transformer, gamma-conditioning, or runtime bug** — all three
were tested and cleared during the RCA. It is purely which-edge-in-which-group.

## 2. Mask 8 — detector-loop over-fusion (LA-2A blocker "a")

**Symptom.** With the faithful detector wired (`out -> C_sc.a` … →
`EL_drive.b -> PC1.led`), SPQR fuses the entire output network — `T_in`,
makeup pot, V3, **and** `T_out` — into one rigid `has_feedback` R-type group
(one stage's `debug_label` contains both `T_in` and `T_out`; 12 feedback
edges). PR-1's de-fusion-via-LED-barrier was **falsified**: the LED pin has no
electrical edge, so it is not in the fused group; the loop closes through the
**sidechain feedback tap** `out -> C_sc.a` bridged back to the `in -> fork ->
R_ff -> R37a` node. Cutting that tap de-fuses; cutting the LED net is a no-op.

**Mechanism.** `FlowGroup::has_feedback()` (`signal_flow.rs:52`) is true whenever
`feedback_edges` is non-empty; any detected cycle routes the group to the rigid
R-type builder, which fuses every reactive element in the loop. But the LA-2A's
output→sidechain path is a **detector tap**, not an instantaneous electrical
feedback — it is the program-dependent gain-reduction loop, whose physical
realization is *delayed* (the photocoupler's CdS cells integrate over
ms-to-s). The engine already has the right tool for this: the **one-sample-delay
convention** used by `SidechainProcessor::cv_delayed` and F5's feedback taps.

**The fix for mask 8 is the delayed cross-network coupling we already specced**
(`reports/cross-network-coupler-impl-plan-2026-06-13.md`). The detector tap
should be a **formation-time cut**: classified as a delayed coupling boundary,
removed from `feedback_edges` so the output network does not fuse, and resolved
by the tree reading last sample's value. So mask 8 ≡ the CrossNetworkCoupler
delayed path; they converge on the same change, and the LED was always a red
herring — the boundary is the tap.

## 3. Design: the formation classifier

Add a **branch classifier** consumed by `find_flow_groups` /
`claim_passive_edges`, built from the parent design's directed-flow inputs
(directed rail-blocked BFS from `in`/to `out`, active-edge direction). Every
passive branch leaving an active device's node is classified as exactly one of:

1. **Operating-point / bias** — terminates on a rail (B+, gnd, AC-ground) with
   no directed path to `out` and no onward active device. → **claim** into the
   device's NL group (today's behavior, now *justified* rather than swept).
2. **Forward-signal** — has a directed path toward `out` (or to a downstream
   group's driven port). → **do not claim**; it is its own downstream stage
   (the transformer R-type, a tone network, a coupling stage). Fixes mask 7.
3. **Delayed-feedback tap** — closes a cycle back to an upstream group AND is a
   detector/sidechain coupling (sourced from `out`/a probe node, sinking into a
   modulation or gain-control element). → **cut** at formation; register as a
   delayed coupling boundary (1-sample delay), not a `feedback_edge`. Fixes
   mask 8 and *is* the delayed CrossNetworkCoupler.

The discriminator for mask 7 (cases 1 vs 2 at a shared output/bias node) is the
one the heuristic lacks: **a rail-terminated branch with no forward path is
bias; a branch with a directed path to `out` is signal** — even when both leave
the same cathode node. The discriminator for mask 8 (case 3) is: **a cycle edge
whose source is `out`/a probe and whose sink is a gain-control/modulation
element is a detector, not instantaneous feedback.**

`feedback_input_is_barrier` (`signal_flow.rs:1432`) and the
`split_groups_at_behavioral_gaps` machinery (`bbd_lowering.rs:108`, the DspBlock
gap-cut) are the existing precedents for case 3 — the formation classifier
generalizes "cut at a behavioral gap" to "cut at a delayed coupling tap."

## 4. Phasing impact on the parent design

Insert before the parent's Phase 2 consumers (the routing layer assumes correct
groups, so formation must lead):

| Phase | Content | Size | Lands independently? |
|---|---|---|---|
| **F-0** | Acceptance: la2a forward-level floor (>−40 dB once transformer not swallowed) RED; `cathode_follower_into_transformer` minimal fixture RED; detector-tap de-fusion structural check (no single stage label holds both `T_in` and `T_out`) | S (tests only) | yes |
| **F-1** | Branch classifier (cases 1/2/3) computed in shadow mode, diffed via tracing against today's claim/fuse decisions on every green fixture | M | zero behavior change |
| **F-2** | `claim_passive_edges` consults the classifier: forward-signal branches no longer claimed (mask 7 fixed → la2a + any cathode-follower-out-transformer circuit gets its downstream stage) | M | yes (heuristic stays as dead code) |
| **F-3** | Detector taps cut as delayed couplings (mask 8 fixed = delayed CrossNetworkCoupler); LA-2A output network de-fuses, detector goes live | M-L | yes |

F-2 and F-3 are independently revertible, each with its own failure-set diff.
After F-3 the LA-2A reaches its [V] acceptance (forward level + GR direction +
attack/release), and the routing-layer phases (parent doc §7) proceed on
correctly-formed groups.

## 5. Migration safety

Formation changes are higher blast-radius than routing because they alter group
membership for *every* tube/BJT stage. Guard set (all currently green):
- **Every amp** (`bassman_5f6a`, `marshall_jtm45`, `tweed_deluxe_5e3[_full]`) —
  cathode/emitter operating-point chains MUST still be claimed (case 1). These
  are the regression frontline for the classifier; `golden_regression.rs`,
  `legends_regression_matrix.rs`.
- **fet_leveler / opto_leveler / vca_bus_comp** — their `envelope_follower`
  detector is already cut at the follower (not a circuit-node tap), so case 3
  must be a *no-op* for them; only the faithful la2a tap is newly cut.
- **Klon/TS/RAT** — feedback op-amp groups must keep fusing (instantaneous
  feedback, case 3 must NOT misfire on a non-detector cycle).
- Determinism fingerprints (`engine_determinism.rs`) churn once on the
  membership change — same one-time re-baseline question as parent Q3.

Discipline: failing-first (F-0), shadow-diff (F-1), per-phase failure-set diff
against a reverted baseline (F-2/F-3), goldens regenerated only with written
justification — the F1–F12 protocol.

## 6. Open questions for review

- **Q-F1 — Classifier home.** Build the formation classifier as part of the
  parent's `flow_plan.rs` (one directed-flow model, run before
  `find_flow_groups` feeds it back)? Or a smaller standalone pass feeding
  `claim_passive_edges`? Leaning: same model, but formation must run first,
  which inverts the parent's "plan after find_flow_groups" ordering — needs a
  two-pass shape (classify → form → plan).
- **Q-F2 — Bias vs signal at a shared node.** Is "rail-terminated, no directed
  path to `out`" a sufficient definition of an operating-point branch, or are
  there bias networks that pass through a non-rail node before a rail (e.g.
  cathode → Rk → bypass-cap → gnd with the cap node tapped)? Need the cathode
  bypass / partially-bypassed-cathode case enumerated against the amps.
- **Q-F3 — Detector detection.** Identify a delayed-coupling tap structurally
  (source = `out`/probe, sink = modulation/gain element) or require a DSL
  annotation? Leaning structural, to keep faithful netlists un-annotated, with
  the DspBlock/`feedback_input_is_barrier` precedent.
- **Q-F4 — Output transformer as its own stage.** Once mask 7 is fixed, a
  cathode-follower → transformer is a 2-stage compile (NL stage probes the
  cathode; transformer R-type stage driven from it). Confirm the cathode node
  is a valid driven port for the downstream `PassiveRType` (it is a low-Z
  source — the parent design's driven-port polarity/Thevenin note, §3 known
  approximation, applies).
- **Q-F5 — Sequencing vs parent.** Do F-0…F-3 land as a precursor PR series on
  `la2a-1to1`, then the parent routing phases rebase on top? Or one combined
  epic? Leaning: precursor series — LA-2A is the forcing function and its
  acceptance is the gate, so it should drive and land first.
