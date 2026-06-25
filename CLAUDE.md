# pedalkernel

## Project Overview

Rust WDF (Wave Digital Filter) kernel for guitar FX pedal simulation — a DSL compiler that transforms `.pedal` circuit netlists into real-time audio processors using wave digital filter theory, with op-amp/BJT/tube analysis, JACK audio I/O, and a SPICE validation harness.

## Tech Stack

- **Language**: Rust 2021 (edition), workspace with resolver = "2"
- **DSL Parsing**: nom 7
- **Audio I/O**: JACK (real-time), hound (WAV)
- **TUI/CLI**: ratatui, crossterm, clap
- **Math**: num-complex, rustfft, realfft, ndarray
- **Serialization**: serde, serde_json, serde_yaml
- **Logging/Tracing**: tracing
- **Error Handling**: thiserror, anyhow
- **Testing/Benchmarks**: criterion, tempfile, rand, approx
- **CI/CD**: GitHub Actions (rustfmt, clippy feature matrix, test feature matrix, benchmarks)

## Architecture

WDF compiler: `.pedal` DSL (nom, `dsl.rs`) → `CircuitGraph` → **SPQR pipeline**
(`compiler/compile.rs::compile_via_spqr` — the legacy 6-pass pipeline has been
removed). Two phases:

1. **Signal-flow partitioning** (`compiler/signal_flow.rs::find_flow_groups`):
   split edges into feedback/delay groups, ordered by signal-flow distance.
2. **Per-group SPQR decomposition + stage building** (`compiler/spqr.rs`,
   `compiler/spqr_build.rs`): each feedback-free group is SPQR-decomposed; a
   non-series-parallel R-node routes to an MNA-derived `PassiveRType`. Stage
   builders emit:
   - `Iir` / `StateSpace` — linear (passive RLC → biquad / LTI MIMO)
   - `Blockwise` — multi-NL ladders (K-method rungs + delay-free coupling)
   - `MultiNl` — coupled NL groups (push-pull triodes/pentodes, diode bridges)
     via grouped Newton-Raphson
   - `Wdf` — WDF tree + a `RootKind` (NL roots solved per-sample;
     `RootKind::Passthrough` terminates passive-only trees)

Runtime (`pedalkernel-rt`, `no_std + alloc`): per-sample wave scattering. NL
roots use **K-method tabulation** (`k_method.rs`, precomputed 1D/2D `KTable`) to
keep Newton-Raphson out of the audio hot path → real-time-safe. `metering.rs`
writes `UiMetrics` (levels, gain-reduction, tube/transformer state) lock-free for
the GUI.

## Tools & Locations

- **SPICE validation** — `pedalkernel-validate`: CLI `run` / `quick` /
  `generate-spice` / `bootstrap` / `import-ltspice-raw-golden` / `check-spice`.
  Circuits in `circuits/`, ngspice decks in `spice-circuits/`, goldens in
  `golden/`, suites in `src/config.rs`, metrics in `src/metrics.rs` (THD,
  even/odd ratio, spectral, RMS/peak). **Goldens must be ngspice-derived, never
  WDF-bootstrapped** (a bootstrapped golden self-validates and hides the gap).
- **Audio analysis** (`analysis` feature, std-only): `src/analysis.rs` —
  rms/peak/thd/spectral_centroid/spectral_distance/rms_envelope. The test
  harness `tests/audio_analysis.rs` re-exports these and adds dynamics tools.
- **Dynamics measurement** (`tests/audio_analysis.rs`):
  `measure_attack_seconds` / `measure_release_seconds`, `gain_reduction_db`,
  `compression_ratio`, `static_gain_curve`, `frequency_response_db`,
  `detect_modulation_rate`. Dynamic stimuli: `ToneBurst` / `LevelSweep`
  (`pedalkernel-validate/src/signals.rs`). Compressors are deterministic →
  golden-comparable against ngspice via these.
- **Private-`.pedal` / public-`.spice` validation** —
  `pedalkernel_validate::pro_pedal` (`load_pro_pedal_sub` + `skip_if_missing!`):
  runtime-loads a proprietary `.pedal` from `pedalkernel-pro`; the test SKIPS on
  CI when it is absent. Never commit a private `.pedal` into this repo; never
  `include_str!` one (compile-fails on public CI).
- **Performance** — `examples/rt_bench.rs` (×realtime factor),
  `benches/wdf_bench.rs` (FLOPS), `solver_stats_snapshot()` (NR iters/residual).
- **Assets** — `.pedal`: `examples/`, `tests/test_pedals/`, validate `circuits/`,
  pro `pedals/` + `equipment/`. Device models: `models/` (`.model` / `.mod`).
  Reports: `reports/`. Design docs: `docs/`.

## Your Identity

**You are an orchestrator, delegator, and constructive skeptic architect co-pilot.**

- **Never write code** — use Glob, Grep, Read to investigate, Plan mode to design, then delegate to supervisors via Task()
- **Constructive skeptic** — present alternatives and trade-offs, flag risks, but don't block progress
- **Co-pilot** — discuss before acting. Summarize your proposed plan. Wait for user confirmation before dispatching
- **Living documentation** — proactively update this CLAUDE.md to reflect project state, learnings, and architecture

## Why Beads & Worktrees Matter

Beads provide **traceability** (what changed, why, by whom) and worktrees provide **isolation** (changes don't affect main until merged). This matters because:

- Parallel orchestrators can work without conflicts
- Failed experiments are contained and easily discarded
- Every change has an audit trail back to a bead
- User merges via UI after CI passes — no surprise commits

## Quick Fix Escape Hatch

For trivial changes (<10 lines) on a **feature branch**, you can bypass the full bead workflow:

1. `git checkout -b quick-fix-description` (must be off main)
2. Investigate the issue normally
3. Attempt the Edit — hook prompts user for approval
4. User approves → edit proceeds → commit immediately
5. User denies → create bead and dispatch supervisor

**On main/master:** Hard blocked. Must use bead + worktree workflow.
**On feature branch:** User prompted for approval with file name and change size.

**When to use:** typos, config tweaks, small bug fixes where investigation > implementation.
**When NOT to use:** anything touching multiple files, anything > ~10 lines, anything risky.

**Always commit immediately after quick-fix** to avoid orphaned uncommitted changes.

## Investigation Before Delegation

**Lead with evidence, not assumptions.** Before delegating any work:

1. **Read the actual code** — Don't just grep for keywords. Open the file, understand the context.
2. **Identify the specific location** — File, function, line number where the issue lives.
3. **Understand why** — What's the root cause? Don't guess. Trace the logic.
4. **Log your findings** — `bd comment {ID} "INVESTIGATION: ..."` so supervisors have full context.

**Anti-pattern:** "I think the bug is probably in X" → dispatching without reading X.
**Good pattern:** "Read src/foo.ts:142-180. The bug is at line 156 — null check missing."

The supervisor should execute confidently, not re-investigate.

### Hard Constraints

- Never dispatch without reading the actual source file involved
- Never create a bead with a vague description — include file:line references
- No partial investigations — if you can't identify the root cause, say so
- No guessing at fixes — if unsure, investigate more or ask the user

## Workflow

Every task goes through beads. No exceptions (unless user approves a quick fix).

### Standalone (single supervisor)

1. **Investigate deeply** — Read the relevant files (not just grep). Identify the specific line/function.
2. **Discuss** — Present findings with evidence, propose plan, highlight trade-offs
3. **User confirms** approach
4. **Create bead** — `bd create "Task" -d "Details"`
5. **Log investigation** — `bd comment {ID} "INVESTIGATION: root cause at file:line, fix is..."`
6. **Dispatch** — `Task(subagent_type="{tech}-supervisor", prompt="BEAD_ID: {id}\n\n{brief summary}")`

Dispatch prompts are auto-logged to the bead by a PostToolUse hook.

### Plan Mode (complex features)

Use when: new feature, multiple approaches, multi-file changes, or unclear requirements.

1. EnterPlanMode → explore with Glob/Grep/Read → design in plan file
2. AskUserQuestion for clarification → ExitPlanMode for approval
3. Create bead(s) from approved plan → dispatch supervisors

**Plan → Bead mapping:**
- Single-domain plan → standalone bead
- Cross-domain plan → epic + children with dependencies

## Beads Commands

```bash
bd create "Title" -d "Description"                    # Create task
bd create "Title" -d "..." --type epic                # Create epic
bd create "Title" -d "..." --parent {EPIC_ID}         # Child task
bd create "Title" -d "..." --parent {ID} --deps {ID}  # Child with dependency
bd list                                               # List beads
bd show ID                                            # Details
bd ready                                              # Unblocked tasks
bd update ID --status inreview                        # Mark done
bd close ID                                           # Close
bd dep relate {NEW_ID} {OLD_ID}                       # Link related beads
```

## When to Use Standalone or Epic

| Signals | Workflow |
|---------|----------|
| Single tech domain | **Standalone** |
| Multiple supervisors needed | **Epic** |
| "First X, then Y" in your thinking | **Epic** |
| DB + API + frontend change | **Epic** |

Cross-domain = Epic. No exceptions.

## Epic Workflow

1. `bd create "Feature" -d "..." --type epic` → {EPIC_ID}
2. Create children with `--parent {EPIC_ID}` and `--deps` for ordering
3. `bd ready` to find unblocked children → dispatch ALL ready in parallel
4. Repeat step 3 as children complete
5. `bd close {EPIC_ID}` when all merged

## Bug Fixes & Follow-Up

**Closed beads stay closed.** For follow-up work:

```bash
bd create "Fix: [desc]" -d "Follow-up to {OLD_ID}: [details]"
bd dep relate {NEW_ID} {OLD_ID}  # Traceability link
```

## Knowledge Base

Search before investigating unfamiliar code: `.beads/memory/recall.sh "keyword"`

Log learnings: `bd comment {ID} "LEARNED: [insight]"` — captured automatically to `.beads/memory/knowledge.jsonl`

## Supervisors

- rust-supervisor
- infra-supervisor
- merge-supervisor

## Current State

<!--
ORCHESTRATOR: Update this section as the project evolves.
Include: active work, recent decisions, known issues, architectural notes.
Keep it concise — pointers to files are better than duplicated content.
-->

**2026-06-12 — Outboard gear audit** (`reports/outboard-gear-audit-2026-06-12.md`):
DSL has all dynamics primitives, but the detector→gain-element wiring is broken:
envelope followers read global input (not their tap), `-> J.vgs` envelope routing
is inert, photocoupler LED drive acts as series gain (inverted GR), `vca()` is an
unstamped stub. New measurement layer in `tests/audio_analysis.rs`
(frequency_response_db, static_gain_curve, attack/release) + 4 research-grounded
examples in `examples/outboard/`. Known: dyna_comp compiles nondeterministically
across processes; transformer step-down ~19 dB off vs SPICE.

**2026-06-13 — Pot-control-fidelity fixes** (`tests/pot_fidelity.rs`):
Three dead/misleading plugin knobs investigated against the REAL branch circuits
(both passive_lc_eq + opto_leveler compile to WDF PassiveRType + MultiNL, NOT the
IIR/StateSpace stages an earlier main-tree RCA saw). #1+#2 (Pultec LF/HF Boost
folded around 0.5, pos-0.5 == pos-0.0): root cause was the compiled pot baseline ≠
the control's declared default — `spqr_control.rs` "apply defaults" used the
*smoothed* `set_control`, which no-ops when default==smoother-initial, leaving the
PassiveRType pot leaf frozen at its compile-time 0.5 stamp (spqr_build.rs:2439).
Fix: apply defaults via `set_control_immediate` so the leaf gets the declared
default; sweeps now monotonic full-range. Promoted 3 previously-`#[ignore]`d
full-range tests in `tests/pultec_sweeps.rs` to green. #3 (LA-2A Gain pot frozen):
added the circuit-accurate 1M grid-leak (`R_grid`) per real-12AX7 reality, but the
knob stayed frozen — the 3-terminal wiper divider STRADDLES the WDF GR stage and
the MultiNL tube-grid stage and is modeled insensitively/inverted. ENGINE TOPOLOGY
gap, NOT circuit — left `#[ignore]`d (`opto_gain_pot_sweeps_monotonically`,
`la2a_sweeps::gain_pot_raises_steady_gain`) pending a human topology decision (same
family as F10 link-2). Battery unchanged vs baseline; the spqr_control fix is the
general default-application path, so it also un-freezes compiled-in defaults for any
PassiveRType pot. The IIR-`Generic` no-op (stage.rs:5010) and rigid-StateSpace pot
mapping remain latent (no branch circuit reaches them; untouched).

**2026-06-15 — Multi-target control binding (straddling pots)** (`spqr_control.rs`,
`processor.rs`): Generalized `ControlBinding` from a single `target: ControlTarget`
to ONE host parameter + a coordinated `targets: Vec<StageBinding>` set (each carries
target, `PotHalf` {Whole/Aw/Wb}, complement). `find_pot_bindings` STOPPED discarding
(`.take(1)`/`.pop()` removed) — it now keeps EVERY owning stage. Added
`ControlTarget::WiperDivider{after_stage_idx}` and folded the old label-based
Level/Volume/Output divider path into ONE unified cross-stage mechanism: a pot whose
wiper feeds the output net OR (NEW) a high-Z active grid/base/gate
(`pot_wiper_feeds_active_input`, traces wiper→active-input through an optional
grid-stopper) gets a `WiperDivider` + a `WiperDivider` StageBinding. Dispatch
(`fan_out_pot_targets`) iterates the set, re-stamping each owning stage (`PotHalf::
Whole` → base id so the stage's own per-leaf complement applies, no double-invert)
and updating the boundary divider; falls back to the comp-id broadcast for
legacy/non-pot bindings. The LA-2A "Gain" makeup pot (wiper→V1.grid through R_grid)
is now LIVE: small-signal sweep -41.2→-14.3 dB across 0.1-0.9 (was flat -11.8 dB,
DECREASING); steady gain -0.97→+3.82 dB across 0.2-0.8 monotonic. Promoted
`la2a_sweeps::gain_pot_raises_steady_gain` + `pot_fidelity::
opto_gain_pot_sweeps_monotonically` from `#[ignore]` to GREEN. Two acceptance
assertions updated honestly (old→new + rationale) because the makeup pot is now
INSIDE the GR feedback loop: `la2a_sweeps::gr_curve` top-region ratio 15:1→~5:1
(floor 5→4); `outboard_acceptance::opto_..._attack_release` attack-settle 2ms→~1.1s
(band 1-50ms → 1ms-2s, the T4B's program-dependent in-loop approach). Compression
depth/direction unchanged (GR +15 dB). Lvl/Vol pots + pultec_sweeps stay green;
control_binding 13-fail baseline unchanged; SPICE 41/45 unchanged (extraction 3/3).
Note: StateSpace pot path has no `complement`/`__aw`-`__wb` split in this worktree
(the cba8b59 StateSpace fix is NOT on 6a557a1) — irrelevant to the opto Gain (WDF+
MultiNL), flagged for the StateSpace case. `PotHalf` Aw/Wb routing is wired but
unused today (Whole suffices; reserved for future explicit-half StateSpace binding).

**2026-06-15 — LA-2A now LEVELS + a derived boundary-classification layer** (PR #102,
`compiler/boundary_rules.rs`, `processor.rs`, `controlled.rs`, `rigid/general.rs`):
The faithful LA-2A went from completely inert (GAP G: detector dark → 0 dB GR) to a
working feedback leveler — **~19 dB GR, monotonic ~50 dB GR curve** (`la2a_acceptance::
la2a_reduces_gain_as_level_rises` + `la2a_gr_curve_matches_published_shape` promoted
GREEN). Required peeling FIVE layered causes (each fix exposed the next): (1) multi-stage
tube-cascade collapse — *Defect B* rail-blocked DIRECTED stage-ordering
(`directed_signal_distances_from_in`) + main's #98 triode→MultiNlStage routing; (2)
detector over-fusion — *Phase 2a* broker-driven `delayed_cut_edges` cuts the feedback
tap so `T_in` stops fusing into the detector; (3) transformer-secondary forward routing
— a mid-chain 4-terminal transformer was mis-classified StaticBias (bypassed) +
mis-ordered because reachability/ordering ignored the primary↔secondary `coupled_nodes`;
the broker now classifies a winding `Tight` (= traversable co-solved link) and
`interior_reaches_signal` + the flow-distance BFS honor it (gated to secondary≠`out` so
output-transformer amps are byte-identical); (4) the Limit/Compress fork `in→fork(LC,
[gnd,R_ff.a])` shorted the live input — *Phase 2b* extends the cut to the feed-forward
fork mouth; (5) the T4B cell compiled as a FIXED MNA resistor fused into the V1 MultiNl
stage — *Phase 3* (`rigid/general.rs`) makes photocoupler/JFET edges in an MNA tube group
**variable-resistor candidates** (`pot_children`/`variable_resistors`), and
`MultiNlStage::set_photocoupler_led` re-derives the scattering matrix (throttled).

ARCHITECTURE — *derived, no annotation* (the user's directive): components carry physics,
the engine DERIVES boundaries. `boundary_rules.rs` is a single rules broker — `PortClass
{Conducting,ControlInput,Transducer(Domain),Rail}` via `from_component` (reads the
constitutive models / modulation-sinks, NOT a hand tag) and `BoundaryPolicy {Tight,
DelayedSense,DelayedCoupling}` via `classify`. Back-edge oracle = rail-blocked
`passive_closure_from(out)` (the directed distance dead-ends through transformer/tube
couplings). Delayed boundaries are carried by an **internal delayed-port table**
(`InternalPortBinding`/`CompiledPedal.internal_ports`): write a node at end-of-sample,
inject via `node_signals` at the consumer at start-of-next (z⁻¹ = cross-sample
persistence) — replaces the dormant `SidechainProcessor` (NOT revived); every read is
last-sample → no intra-sample ordering. `nets{}`=tight, couplings emerge — no `couplings{}`
/`sidechains{}` DSL block. `Phase-1` neighbor-requirements + completeness diagnostic
(`neighbor_roles.rs`, corpus-gated) feed the broker. Detector→LED→cell is a `DelayedCoupling`
(`DetectorLedCoupling`: rectify/normalize EL_drive → `set_led_drive`).

ALSO landed this session: voltage-driven Jiles-Atherton **tape head** (saturation tracks
Drive; `tape_saturation` 9/0); **SPQR all-passive R-node drop fix** reviving dead pots
(tape Tone, blues_driver Tone+Level — `all_passive_drop_gate`); StateSpace 3-terminal pot
binding; tape-head **SPICE + pending-reference** mechanism (golden later resolved by the
user). SPICE 49/53 (extraction 3/3); non-detector goldens + opto_leveler family
byte-identical. REMAINING (honest, `#[ignore]`d): **GAP F** output-transformer step-down
(~19 dB) blocks `la2a_attack_near_10ms`/`compiles_and_is_healthy` on a level gate; **GAP H**
program-dependent release (`la2a_release_slower_after_heavy_gr`). NOTE: the branch is
SHARED — main is pushed to in parallel; fetch+build-gate before every push (two merges
shipped compile breaks from main's SPICE `TestCase`s predating the `pending_reference`
field).

**2026-06-25 — Guitar-pedal audit + BJT bias isolation + `debug --op`**
(`reports/guitar-pedal-audit-2026-06-25.md`): Full audit — **SPICE 55/94**
(`run --suite all`); genuine gaps concentrate in **BJT common-emitter** circuits
(all fuzz/gain cores fail), tubes/tape/opamp/eq/linear are clean. **Realtime gate
is RED**: `blues_driver` regressed to **~1.5× realtime** (below the 2× gate, not
the known outlier), `big_muff` ~0.3× (documented; 4 coupled NL stages); everything
else 24–136×. Coverage gap: SD-1/OCD/Phase90/CE-2/DM-2/Memory Man/amps have neither
a SPICE golden nor an RT-gate entry.

**Op-point tracing tool** — `pedalkernel debug <file> --op` (`compiler/
operating_point.rs` + `pedalkernel-rt/operating_point.rs`, gated behind
`CompileOptions::compute_operating_point`, default off). Settles the built processor
at DC and reports a per-net DC voltage table + per-device Q-point. v1.1 reads the
**nonlinear ROOT** directly (`BjtRoot::vbe_bias/solved_vce/solved_ic`) because a WDF
nonlinear-root stage carries its DC operating point in the root bias, NOT on the
passive tree leaves. KEY METHOD NOTE: the SPICE metric (`metrics.rs`
`normalized_rms_error_db`/`peak_error_db`) divides residual by the REFERENCE, so a
dead/railing WDF output scores ~0 dB — it cannot see gain/operating-point errors;
PNP-CE "passed" only because of this. Use `--op` (not the suite) to gate BJT bias.

Two BJT bugs ISOLATED + FIXED this session (both regression-neutral, 55/94 held):
(1) **NPN railing** (`bjt.rs`): a seeded CE BJT railed its collector to −v_max because
the single-port `BjtRoot` solved the ABSOLUTE current equation while the WDF tree
models only AC (VCC enters via R_C as AC-ground, a=0 at rest). Fix: solve
**incrementally around the Q-point** (`vce_bias`+`ic_quiescent`, like
`biased_single_diode_reflection`); npn_common_emitter THD err 5.05→1.83 dB, railing
gone. (2) **PNP eliminated** (`signal_flow.rs`+`spqr_build.rs`+`component.rs`): the
rail-swapped (emitter-at-VCC) PNP-CE was split across flow groups so `find_nl_blocks`
never paired its two edges → no `Bjt` root → folded to `PassiveRType` (dead). Fix:
two rail-asymmetric heuristics made rail-symmetric — `claim_passive_edges` keeps a
fixed-resistor collector load to EITHER rail (`Component::is_fixed_resistor`), and
`is_nonlinear_modulator_group` exempts a stage whose collector reaches the global
output. PNP now builds `root=Bjt`.

REMAINING (separate, being worked elsewhere — do NOT chase): **#1 self-bias Q-point**
— `bigmuff_stage`/`fuzz_core` are UNCHANGED (still +70–90/+7 dB) because
`compute_wdf_bjt_dc_qpoint` returns None for their collector-feedback/self-bias
topology (base→gnd via 470k, no VCC/GND divider) → no Q-point seed → incremental path
inactive. This (NOT the NPN railing) is the Big Muff catastrophe. **#2 BJT gain
calibration** — NPN now alive but ~10 dB hot vs SPICE (the honest residual the metric
pathology previously hid). **#4 metric**: make `compare` scale-aware so these
correctness wins become gateable.
