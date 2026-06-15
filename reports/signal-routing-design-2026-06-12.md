# Signal-Routing Design: F9 + F13 as One Change — 2026-06-12

Design document for human review. No code has been written. Companion to
`reports/architecture-debt-2026-06-12.md` §3 (the six masks) and
`reports/outboard-fix-and-product-plan-2026-06-12.md` (F9 :147, F13 :195).

## 1. Problem

The compiler decides where audio ENTERS a group, where it LEAVES, and how the
group participates in the chain via at least five independent local heuristics:

| Site | Decision | Mechanism today |
|---|---|---|
| `spqr_build.rs:2273-2311` (`build_passive_rtype_stage`) | input/output node | terminal iteration order (NodeId), plus F5's swap-on-proven-inversion patch :2304-2311 |
| `rigid/general.rs:739-752` (`build_wdf_ports`) | injection port | fallback chain: `in_node` → amplifier pin → F12's `find_input_boundary_node` (:777-826) |
| `spqr_build.rs:1216-1346` + :1530-1650 | feedforward vs serial | two separate bridge classifiers; flags applied by matching `signal_flow_distance` + `debug_label` (:1628, :1685 — release builds match blindly or not at all) |
| `spqr_build.rs:3609-3786` (`compute_group_flow_distances`) | execution order | undirected BFS min-hop + special-case bumps (sink :3730-3737, feedback :3738-3748, ground-clip :3755-3783) |
| `processor.rs:3203-3241` | runtime input | serial `signal` var, with `is_trigger_voice` / `is_feedforward` bolt-ons; ff reads fall back to stale serial signal (:3234) |

The six masks (architecture-debt §3) are all consequences: arbitrary terminal
order (F1/B1), NodeId orientation flips (F5b), grounded injection (F12b),
mid-network injection silence (F13a, `kick_808_wav.rs:137` — `in` enters the
bridged-T midpoint `R1.b`), parallel branches collapsing (F13b, snare), and
serial passive stages hard-zeroing the chain (F13c, moog_ladder_vcf — its
resonance path `Resonance+R_fb` and CV chain `R_cv+Cutoff` compile as serial
stages with no in→out transfer).

## 2. Design: a single `SignalFlowPlan` pass

New module `pedalkernel/src/compiler/flow_plan.rs`, run once after
`find_flow_groups` (`signal_flow.rs:1220`) + `split_groups_at_behavioral_gaps`
(`bbd_lowering.rs:108`), before any stage is built. Every downstream builder
consults the plan instead of recomputing locally.

```rust
pub(crate) struct SignalFlowPlan {
    pub groups: Vec<GroupPlan>,
    pub order: Vec<usize>,              // topological execution order
}
pub(crate) struct GroupPlan {
    pub role: GroupRole,                // Serial | ParallelBranch { converge: NodeId }
                                        //   | Bypass | Feedback (solver attribute, orthogonal)
    pub driven_ports: Vec<DrivenPort>,  // node + source (GlobalIn | Probe(group)) + polarity
    pub probe: ProbePort,               // node + extraction polarity
}
```

Inputs (all already exist):
- **Rail-blocked BFS distance from `in`** — `bfs_distances_from_in_node`
  (`signal_flow.rs:759`), extended to be *directed*: active edges traverse
  input→output only, using `SignalTerminals` (`signal_flow.rs:100-153`) and
  `graph.nullor_pins`; passive edges bidirectional; rails block (as today).
  A reverse pass from `out` gives distance-to-output.
- **Active-edge direction** — `graph.active_edge_indices` + per-element
  input/output pins from `find_active_elements` (`signal_flow.rs:93`).
- **Group boundary nodes** — the discovery half of `compute_group_terminals`
  (`spqr_build.rs:3792-3848`), kept; its ordering/merging half retired (§5).

Per-group derivation:
1. **Driven port(s).** Boundary nodes with an incoming directed path from `in`
   or from an upstream group's probe node. Preference order (the F9 recipe,
   architecture-debt E4): (a) node driven by an upstream active-edge OUTPUT
   (op-amp out is a voltage source — strongest claim), (b) minimal directed
   distance from `in`, (c) NodeId as tiebreak ONLY. If `in_node` (or a trigger
   port) is *interior* to the group, the driven port is that exact interior
   node — see §4.
2. **Probe port.** Boundary node with minimal directed distance to `out`
   (or to the downstream group's driven port). Polarity is derived from edge
   orientation at the extraction element, eliminating the sign-blind
   `b_tree / 2.0` fallback (`pedalkernel-rt/src/stage.rs:2196`) as the default
   path — the probe names a leaf and a sign.
3. **Role.**
   - `Bypass`: no directed path probe→out AND not a convergence input
     (confirms, and eventually replaces, `classify_group_bias`,
     `spqr_build.rs:304-307`). This is what fixes F13c: moog_ladder's CV
     chain and any tap-off network stop overwriting the serial signal.
   - `ParallelBranch { converge }`: §3.
   - `Feedback`: `FlowGroup::has_feedback()` carried through — affects solver
     choice, not routing.
   - `Serial`: everything else.
4. **Order** = topological sort of the "probe feeds driven-port" DAG; ties
   broken by directed distance then group index. Replaces every special case
   in `compute_group_flow_distances` (sink bump, feedback span arithmetic,
   ground-clip reordering — all become emergent properties of the DAG).

Runtime contract: the ad-hoc `node_signals: Vec<(nid, v)>` becomes the single
routing bus. EVERY non-bypass stage writes its probe value at its probe node;
every driven port reads the bus by node. The serial `signal` variable in
`processor.rs:3130-3300` degenerates to "bus value at the previous probe
node". Plan order guarantees writes-before-reads; the stale-serial fallback at
`processor.rs:3234` (root cause of `feedforward_stage_receives_signal` red) is
deleted — an unsatisfiable read becomes a compile error, not silence.

## 3. Parallel summation at convergence nodes (F13b, 808 snare)

Evidence: `examples/eurorack/drums/snare_808.pedal` SIMPLIFICATIONS 4b
(lines 78-97) — each resonator rings correctly alone; together the engine
picks one branch to own `out` regardless of mixer formulation (four tried);
blend pot inert. Today's only summing mechanism is `signal += stage_output`
for `is_feedforward` stages (`processor.rs:3279`), with sign and ordering
broken (the 5 red lib tests).

Design: **summation happens inside the convergence group's MNA, not on the
bus.** A convergence node is detected when ≥2 groups' probe paths terminate
at boundary nodes of one downstream group (the mixer: resistive summer, pot
crossfade, or inverting summing amp). That group's plan gets one driven port
per branch; its builder (`build_passive_rtype_stage`, blockwise, or rigid
MNA) stamps one voltage source per driven port instead of exactly one
(`spqr_build.rs:2355-2356` currently hardcodes a single `vs_node`;
`MnaSystem::new(..., 1 + …)` at :2351 already parameterizes source count for
transformers — the extension is mechanical). For linear mixers the multi-input
transfer is exact by superposition inside one solve; for NL convergence groups
all sources enter the same NR solve — no superposition assumption needed.

Known approximation (documented, not hidden): branch probe values are treated
as ideal voltage sources at the convergence group's driven ports. Exact for
op-amp-output branches (808 snare, Klon blend — they ARE voltage sources);
Thevenin loading error for passive-output branches, same class as today's
stage-boundary approximation.

The additive `is_feedforward` blend and both bridge classifiers are retired
(§5); a Klon-style branch is just a `ParallelBranch` group whose convergence
group is U2's feedback stage.

## 4. Mid-network injection (F13a, 808 kick)

`kick_808_wav.rs:136-137`: `in -> R_trig -> R1.b` — the bridged-T midpoint,
interior to U1's feedback group. Today every builder assumes drive arrives at
a terminal or an op-amp pin, so the nine voices compile silent (snare's
SIMPLIFICATIONS 3b measured peak = 0.0 and moved the injection to `U1.neg`
as a workaround, changing the transfer-function zeros vs hardware).

Design: the plan records the driven port as the *interior* node, and group
builders stamp the source there:
- Rigid/MultiNL MNA: `build_wdf_ports` (`rigid/general.rs:681`) takes the
  plan's node instead of the :739-752 fallback chain — the MNA machinery is
  node-agnostic already; only the selection is wrong.
- IIR/OpAmp derivations: coefficient derivation must use the driven node as
  the source location (this is why injection-at-pin "works" but injection
  mid-network silently zeroes — the derivation never sees the source).
  Where a builder cannot honor an interior driven port, it must REFUSE
  (compile error) rather than fall back — same principle as expedient E3.

Acceptance: bridged-T driven at the midpoint matches a SPICE golden (new
fixture), then the un-workaround'd kick/snare netlists ring.

## 5. What gets deleted vs adapted

Deleted (replaced by plan lookups):
- Input/output selection + F5 swap in `build_passive_rtype_stage`
  (`spqr_build.rs:2273-2311`).
- Injection fallback chain + `find_input_boundary_node`
  (`rigid/general.rs:739-752, 777-826` — F12's third fallback subsumed).
- Feedforward bridge site 1 (`spqr_build.rs:1216-1346`, incl. F2's
  `genuinely_parallel` guard :1267-1293) and the phase-2 detector + flag
  application (`spqr_build.rs:1530-1714`, incl. the debug_label matching).
- The distance special cases in `compute_group_flow_distances`
  (`spqr_build.rs:3717-3786`); the plain BFS helper folds into the plan.
- Runtime: `is_feedforward` additive blend + stale fallback
  (`processor.rs:3224-3234, 3277-3279`); `is_trigger_voice` node routing
  (:3203-3223, :3280-3297) generalizes into the bus (trigger voices become
  ordinary groups driven from the trigger port node).

Adapted (kept, but consult the plan):
- `compute_group_terminals` boundary *discovery* (`spqr_build.rs:3803-3848`)
  feeds the plan; the convergence-merge heuristic (:3853-3894) and the
  F1 BTreeSet ordering contract become irrelevant once orientation is
  plan-driven (ordering stays deterministic but no longer load-bearing).
- Claiming (`signal_flow.rs:794`) is a *membership* concern and stays; the
  E2 BJT carve-out (:899-908) becomes removable once BJT groups without `in`
  get a real driven port (its exit plan, architecture-debt E2).
- Triode-context path (`spqr_build.rs:1139-1207`) keeps its MNA but takes
  grid-node ordering from the plan instead of its private BFS (:1190).
- BBD boundary terminals (`spqr_build.rs:324-331`) become plan inputs:
  behavioral-gap edges are "probe here / re-drive there" pairs.

## 6. Migration safety and test strategy

Behaviors that could shift (all currently green):
- **TS/Klon/RAT post-F2 calibrations** — bridge-classifier retirement changes
  the Klon blend path mechanism. Guards: `tests/golden_regression.rs` +
  `tests/golden/`, `legends_regression_matrix.rs`, `ratking_regression.rs`,
  `screamer_drive_sweep.rs`, `tone_pot_blend.rs`.
- **Stage-order ties** — plan ordering may permute equal-distance stages:
  `engine_determinism.rs` fingerprints will churn once (see Q3).
- **F5-dependent fixtures** — orientation now plan-derived:
  `envelope_taps.rs`, `outboard_acceptance.rs` (opto/fet levelers).
- **Trigger-voice routing rewrite** — `kick_808_wav.rs`,
  `eurorack_examples.rs` kick test.

Strategy (same discipline as F1-F12): failing-first tests, then per-phase
failure-set diff against a reverted baseline; goldens regenerated only with a
written justification.

Existing red/ignored tests that should flip:
1. `tests/eurorack_examples.rs:212` `snare_808_dual_modes_and_tone_blend`
   (un-ignore; the audit prints at :188-199 become asserts).
2. `tests/kick_808_wav.rs:202` — 9 silent voices ring at target frequencies.
3. `examples/synths/moog_ladder_vcf.pedal` — no test exists today; write one
   (nonzero output, lowpass slope, resonance pot authority) BEFORE phase 2.
4. dyna_comp electrically-correct variant (F1/E4 exit): input-terminal
   selection by upstream-active-edge preference — `outboard_acceptance.rs`.
5. The 5 red lib tests in `compiler/feedforward_tests.rs`
   (`two_path_feedforward_produces_output`, `pot_crossfade_…`,
   `goldenrod_feedforward_produces_signal`, `goldenrod_gain_crossfade_…`,
   `feedforward_stage_receives_signal` / `…_ordering`).
6. TB303/eurorack_acid resonance — **evidence file not found**, see Q1.

New acceptance tests to write FIRST (phase 0):
- `flow_plan` unit tests on minimal netlists: divider orientation under node
  renumbering (F5b regression), fan-out→converge with analytic resistive-sum
  check, interior-node injection vs SPICE, CV-chain → role=Bypass.
- Shadow-mode invariant: on every currently-green fixture, the plan's
  decisions match the heuristics' (divergences logged, triaged, then gated).

## 7. Sizing and phasing

| Phase | Content | Size | Blast radius | Lands independently? |
|---|---|---|---|---|
| 0 | Acceptance tests + golden baseline snapshot | S | none (tests only) | yes |
| 1 | `flow_plan.rs` in shadow mode (computed, diffed via tracing, unused) + stable stage↔group IDs (kills debug_label matching) | M (~600 loc new) | zero behavior change | yes |
| 2 | Consumers switch: orientation (`build_passive_rtype_stage`), injection (`build_wdf_ports`, incl. mid-network), ordering, role=Bypass | M-L | whole fixture suite; flips moog ladder, kick, dyna_comp | yes (heuristics still present as dead code) |
| 3 | Multi-driven-port MNA + node-bus runtime + parallel sum | L | compiler + rt processor loop; flips snare blend + 5 feedforward tests | yes |
| 4 | Delete retired heuristics (§5), remove E2 carve-out, fold trigger voices into bus | S-M | cleanup; failure-set diff must be empty | yes |

Phases 2 and 3 are separately revertible; each carries its own failure-set
diff. Estimated total: the largest single compiler change since blockwise —
which is the argument for shadow mode, not for splitting the design.

## 8. Open questions for the reviewer

- **Q1 — Missing evidence:** `tests/eurorack_acid.rs` and the held
  `tb303_vcf` example are cited as failing evidence but exist neither in this
  branch's history (`git log --all -- '*eurorack_acid*'` returns nothing) nor
  as untracked files. Where do they live, and what is the 303-resonance
  acceptance criterion? (TB303 work exists at `tb303_decomposition_tests.rs`
  / BKM commits 6357709, f36a8fc — is that the intended reference?)
- **Q2 — Sum semantics:** is in-MNA summation with ideal-source branch
  coupling acceptable for v1 (exact for op-amp branches, Thevenin error for
  passive branches), or must v1 model branch output impedance at convergence?
- **Q3 — Determinism vs P1:** plan ordering will churn bit-exact fingerprints
  once (`engine_determinism.rs`). Accept a one-time re-baseline, or pin
  legacy order on ties at the cost of carrying the old distance code?
- **Q4 — Refusal policy:** when a builder cannot honor a plan (e.g. IIR with
  an interior driven port it can't derive), hard compile error (E3 spirit) or
  warn-and-fall-back during phases 2-3 only?
- **Q5 — Bypass definition:** no *directed AC* path to out, or no DC path?
  Affects CV chains (pot to gnd) vs bias dividers — proposal: AC, with
  `classify_group_bias` kept as a cross-check during migration.
- **Q6 — Scope:** fold behavioral-island ports (architecture-debt §4) into
  the plan now (BBD pairs as first-class probe/drive entries), or keep the
  current additive BBD mechanism until the VCO/island formalization?
