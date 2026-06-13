# Architecture Debt & Open Issues — 2026-06-12

Companion to `reports/outboard-fix-and-product-plan-2026-06-12.md` (the fix
plan) and `docs/circuit-provenance.md` (the accuracy registry). That plan
tracks *what* is broken; this report is the honest assessment of *how* it was
fixed — which of today's ten landed fixes are proper, which are tracked
expedients with exit plans, and what architectural weakness the whole session
exposed but did not fix. Written after F1–F7, F10–F12 landed (21 commits,
branch `claude/kind-brahmagupta-7tjsyw`).

## 1. What is solid

Every landed fix carried: an independent root-cause analysis with file:line
evidence (most confirmed by reproduction, several by multiple independent
RCAs), a failing-first test that defined "fixed" before any code changed, and
a failure-set diff against a reverted baseline proving zero collateral. No
fix was calibrated around; no acceptance threshold was weakened (two tests
remain `#[ignore]`d rather than lowered). The fixes that are architecturally
clean — fully root-cause, right mechanism, no residue:

- **F2** — bridge classifier now requires genuine parallelism (mirrors the
  site-2 guard the codebase already had).
- **F3** — the inversion-detection BFS models electrical reality (op-amp
  outputs are signal sources).
- **F5** — detector taps resolved at compile time; feedback taps use the
  engine's own one-sample-delay convention (SidechainProcessor precedent).
- **F6** — leaf-XOR-input-path classification plus the controlled-resistance
  MNA stamping photocouplers always needed.
- **F11 (lowering half)** — splitting flow groups at behavioral gaps is
  correct compiler topology work.
- **F4 (binding chain)** — the envelope binding pipeline now actually exists
  end-to-end.

## 2. Tracked expedients (deliberate, documented, need their exit plans run)

| # | Expedient | Why it happened | Exit plan |
|---|---|---|---|
| E1 | **F12 hosts the MOSFET square law inside `JfetRoot`** (`JfetRoot::from_mosfet`, embedded `Option<MosfetModel>`) | `NlDeviceKind` lives in `stage.rs`, owned by a concurrent agent at fix time; adding a variant breaks exhaustive matches in `processor.rs` | Add `NlDeviceKind::Mosfet` properly; the `NlDeviceIv for MosfetRoot` impl was pre-built for exactly this. Bundle Zener support (needs a real variant — no host can express forward-diode + reverse-breakdown) |
| E2 | **F10 exempts BJT-containing SCCs from the claiming bound** | A BJT makeup stage whose group lacks `in` compiles to a dead NlWdf — the old swallow behavior is load-bearing for it | Fix the underlying dead-NlWdf compile (likely the same injection weakness as §3), then remove the carve-out |
| E3 | **F7's silent stage-drop became `tracing::warn!`, not a compile error** | `spqr_to_stages` is infallible at every call site | Make the SPQR stage-collection path fallible; a passive group that fails conversion should refuse to compile, not warn |
| E4 | **F1 froze the dyna_comp on the electrically wrong variant** (flat-linear; VS at U1.neg) | BTreeSet ordering was the validated minimal determinism fix; correctness was scoped out deliberately | F9: input-terminal selection by upstream-active-edge preference (`graph.active_edge_indices`), sorted order as tiebreak only — recipe verified by gdb-level RCA |
| E5 | **F4 activates `resolve_components` only for envelope-driven JFETs** (`resolve_components_by`) | Protecting LFO-path behavior during parallel fixes; the general pass remains dead code | Decide the general reclassification policy and either activate or delete the dead pass |
| E6 | **F7 phase 2 (live switched-component positions) unimplemented** | Phase 1 (stop the collapse, frozen at position 0) was the validated scope | Wire `SwitchPosition` controls through `spqr_control` + leaf recompute; an `#[ignore]`d red test already defines done |

## 3. The central architectural gap: heuristic signal routing

At least **six independently-discovered defects are the same weakness wearing
different masks**: the compiler chooses input/output/injection nodes by local
searches and fallback heuristics rather than a principled, directed
signal-flow model of the circuit graph.

The masks found this session:

1. F1/B1 — group terminal order decided which node got the MNA voltage
   source (was: hash order; now: stable but electrically arbitrary).
2. F5's second root cause — `build_passive_rtype_stage` oriented
   input/output terminals by NodeId order; a virtual pin joining a node
   flipped a divider's transfer.
3. F12's second root cause — NL groups containing neither `in` nor an
   amplifier got a grounded injection port: structurally deaf, device-agnostic.
4. F13a — audio injected at an interior node of an op-amp feedback network
   compiles to silence (why `kick_808_wav.rs` has been silent since its
   introduction).
5. F13b — parallel active branches do not sum (808 snare's two resonators
   collapse to one; four mixer formulations tried; blend pots inert).
6. F13c — a serial passive stage carrying no source→out flow hard-zeros the
   chain (moog_ladder_vcf, boss_ce2's wet path, the held 303 VCF example).

Four of these have now been patched individually (1–3 plus part of the
feedforward story). **Recommendation: stop patching masks.** F9 + F13 should
be one design, not two more patches: a single signal-flow pass that assigns
every group a driven port, a probe port, and a role (serial / parallel-sum /
bypass) from global topology — rail-blocked distance from `in`, active-edge
direction, and explicit parallel-branch summation at convergence nodes. This
is the next piece of work that deserves a design review rather than an
autonomous fix; it gates the Eurorack line (303 VCF resonance, snare blend,
Moog ladder) and the dyna_comp's correct variant.

## 4. The behavioral-island boundary is implicit

BBD, VCA, and VCO are behavioral blocks beside the circuit-exact WDF core — a
legitimate hybrid (a clocked charge-transfer IC is not WDF-able) — but the
boundary is implicit: `GraphRole::Virtual` + `StampResult::Skip` + a hoped-for
lowering pass. The census found all three the same way: **by silence**. F11
fixed BBD; F8 (VCA) is open; VCO lowering is open (cem3340_vco and minisynth
compile silent). The F11 fix also revealed the runtime contract is global
rather than per-instance (one `bbd_wet_mix` for all BBDs, serial post-stage
application — a behavioral approximation of circuit placement).

**Recommendation:** make "behavioral island" a first-class compiler concept —
declared ports, mandatory lowering (compile error if a Virtual component has
no lowering pass), per-instance runtime contracts. That single change would
have prevented BBD/VCA/VCO from ever shipping silent and prevents the next
behavioral element from repeating the pattern.

**Status — DONE (2026-06-13).** Principle *and* mechanism are now both
realized. "Behavioral island" is a first-class compiler concept: the
`DspBlock` trait (`compiler/dsp_block.rs`) — a `GraphRole::Virtual` component
that lowers to a per-instance runtime DSP block bridged across a galvanic gap.
A single `dsp_blocks()` registry is the *only* place a block is declared, and
that one line wires it into group splitting, terminal injection, runtime
binding, **and** the mandatory-lowering gate at once. The gate
(`reject_unlowered_behavioral`) is registry-driven: a runtime-DSP island whose
`type_tag` no registered block `handles()` fails to compile naming the
component — so the gate is impossible to forget when adding a block (the
`vco()` rejection is now produced by the registry, not a hand-written arm).
`bbd_lowering`/`vca_lowering` were migrated to `impl DspBlock for
BbdBlock`/`VcaBlock` (keeping their type-specific guts; each writes its own
`compiled.bbds`/`compiled.vcas` field), collapsing the ~6 hand-wired call
sites in `spqr_build` into one registry pass each. This was a pure refactor
(zero behavior change: identical battery failure set, zero golden drift,
determinism intact). The spring-reverb (roadmap) and delay-line (F15) blocks
implement this contract — one impl + one registry line each — instead of
hand-copying a third and fourth lowering module. The remaining open piece is
the per-instance *runtime* contract for BBD (one global `bbd_wet_mix` for all
BBDs — a behavioral approximation of circuit placement); the compile-side
boundary is now formalized.

## 5. Other standing debts (pre-existing, confirmed this session)

- **Feedforward sign/ordering** — the legit parallel-path feature emits −x
  and executes before its injection node is written (falls back to stale
  serial signal). 5 lib tests still red
  (`feedforward_tests::ordering/receives_signal`, `goldenrod_*`). Fix B from
  the F2 RCA: Passthrough fallback sign at `stage.rs:2196` + consistent
  distance-scale ordering.
- **F10 link 2** — MultiNL stages inject at the device pin when `in` is
  absent from the group (`rigid/general.rs:735-746` — partially improved by
  F12's third fallback) and IIR stages silently ignore `Generic`-role pot
  writes (`pedalkernel-rt/src/stage.rs:4768-4770`). Blocks the op-amp makeup
  variant (test `#[ignore]`d with numbers).
- **F14** — residual LSB-scale (~1e-13) in-process nondeterminism in
  tweed_deluxe_5e3(_full) and opto_leveler; candidate sites
  `rigid/general.rs:277/402/421/447` HashSets, `signal_flow.rs:77`
  `rail_nodes`. Inaudible; breaks bit-exact determinism (the P1 brand
  promise) and build-cache keys.
- **Op-amp closed-loop dynamics post-processed** (GBW/slew/rails outside the
  scattering matrix) — long-documented in `docs/modeling-limits.md`; matters
  for precision active EQ (API 550-class) and the F12 OCD follow-ups.
- **OCD drive direction inverted** (gain falls as Drive rises — feedback
  rheostat misclassification family) and **diode-connected MOSFET (vgs=vds)
  unsupported** — both noted in `tests/mosfet_mna.rs` comments.
- **Pots on IIR stages are inert** (`IirPotRole::Generic` silently dropped);
  known TODO, same family as F10 link 2. Blocks kick decay control.
- **golden triode_clean is "too loud"** (peak 294.85 vs limit 40) —
  pre-existing, surfaced when stale-golden aborts stopped masking it.
- **Build hygiene:** `cargo test --lib --no-default-features` does not
  compile (`compile_pedal_cached`/postcard not gated behind `build-cache`) —
  CI's no-features row would fail; the rt `debug-trace` feature does not
  compile (bare `eprintln!` in no_std paths, `solver.rs:735/973`); local
  rustfmt 1.94.1 disagrees with the repo on ~10 files (every `cargo fmt
  --all` invocation churns them); 4 test binaries `include_str!` missing
  pedalkernel-pro files instead of being feature-gated.
- **No noise primitive in the DSL** — blocks the 808 snare's snappy path
  (avalanche-transistor source, well-characterized: 130 mV rms calibration);
  candidate new stateful element alongside the spring-reverb primitive from
  the product roadmap.
- **`envelope_follower` detector is peak-ish RC only** — dbx-class RMS
  (log-domain 35 ms averager) still missing (plan Phase 2).

## 6. Sequencing recommendation

1. **F8 (VCA)** — last silent compressor; behavioral path mirroring OTA, no
   architectural entanglement. Then the acceptance suite is 8/9.
2. **F9+F13 as one designed change** (§3) — with a design doc reviewed
   before implementation. Unblocks: 303 VCF, snare blend, Moog ladder,
   dyna_comp depth, E2's carve-out removal, likely the feedforward ordering.
3. **E1 exit** (proper `NlDeviceKind::Mosfet` + Zener) and **E3** (fallible
   stage collection) — small, independent.
4. **Behavioral-island formalization** (§4) folded into the VCO lowering
   work (the third stub becomes the test case for the concept).
5. **F14 + build hygiene** — batch of small determinism/CI items before any
   public validation claims (P1 gate).
