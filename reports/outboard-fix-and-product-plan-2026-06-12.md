# Outboard Fix & Product Plan — 2026-06-12

Companion to `reports/outboard-gear-audit-2026-06-12.md`. That report found the
gaps; this is the plan to close them and where the project goes afterward.
Every root cause below is evidence-backed (file:line, confirmed by reproduction
unless marked otherwise). The TDD contract is in
`pedalkernel/tests/outboard_acceptance.rs`: nine `#[ignore]`d red tests, one per
gap, each verified to fail today for its documented reason. **A fix is done when
its acceptance test's `#[ignore]` line is deleted and the suite is green.**

## Part 1 — Getting the compressors working

### Phase 0: hygiene (do alongside everything else)

- `cargo test -p pedalkernel --no-default-features --lib` does not compile
  (86 errors: `compile_pedal_cached`/postcard usages not gated behind the
  `build-cache` feature). CI's no-features matrix row would hit this.
- Pre-existing red tests that overlap this work and should be adopted, not
  duplicated: `zero_output_tests::minimal_opamp_with_input_network_produces_audio`
  (B2, red), the six `feedforward_tests`/`goldenrod_*` cancellation tests (red,
  same arithmetic as B2), `zero_output_tests::screamer_*` (environmental —
  needs `pro-pedals` gating).
- Local rustfmt 1.94.1 disagrees with the repo on ~10 pre-existing files; don't
  commit that churn.

### Phase 1: fix sequence

Ordered by confirmed-cause × blast-radius × unblocking-power. Each item lists
root cause → fix → acceptance.

**F1. Compile determinism (B1/G9) — IN FLIGHT.**
Cause: `compute_group_terminals` (`compiler/spqr_build.rs:3380`) iterates a
`std::HashSet` to order group terminals; the >2-terminal merge keeps the first
survivor, which downstream decides `build_passive_rtype_stage`'s `input_node`
(~:2101), MNA source stamping (~:2166), port polarity (~:2290), and baked
scattering coefficients (~:2316). `RandomState` seeds per instance → flips even
in-process; the two dyna_comp variants differ by 4.21 dB (exactly the audit's
GR spread). Fix: `BTreeSet` (validated 24/24 + 20/20 deterministic on a patched
copy) + red-first tests in `engine_determinism.rs`. Caveat: deterministic ≠
correct — a second independent RCA (gdb on the unmodified binary, 8/8 runs
correlated) proved the op-amp output node (U1.out) is the physically correct
injection point, while ascending node order deterministically picks U1.neg —
the flat-linear variant. Follow-up F9 has the recipe: prefer the terminal
driven by an upstream active/VCVS edge (`graph.active_edge_indices`), with
sorted order as tiebreak only. Latent: sweep other hash-ordered iterations feeding compilation
(spqr_build.rs:1214, blockwise.rs:307, :1959 probes; hashbrown maps are
per-process-seeded — cross-process risk).

**F2. Feedforward-bridge silence (B2 + B3a/G5).**
Cause: the bridge branch (`spqr_build.rs:1122-1183`) treats any series element
spanning {`in` or an op-amp out} → {feedback-group node} as a parallel
feedforward path, lowers it to an open-circuited Passthrough WDF stage that
emits exactly `−x` (`pedalkernel-rt/src/stage.rs:2174-2196`), and the additive
blend (`processor.rs:3179-3181`) computes `x + (−x) = 0` upstream of an
otherwise-correct gain stage. Trigger: input cap before an inverting input
(B2), or R→C interstage coupling (B3a). Fix: per-group source check — only
bridge when the feedback group owning the far node also reaches the source
node through its own claimed edges; otherwise fall through to the proven
blockwise/IIR lowering. Second pre-existing bug, same arithmetic: the bridge's
own sign/weighting (the six red `feedforward_tests`). Acceptance:
`inverting_opamp_with_input_cap_passes_audio`,
`resistor_cap_coupled_opamp_cascade_passes_audio`, plus the existing
`zero_output_tests` red test; the feedforward_tests going green is the stretch
goal (sign fix). **Blast-radius warning (third RCA pass, confirmed): the loose
path is load-bearing.** Tube Screamer, Klon, RAT, and Blues Driver all compile
site-1 feedforward stages today and their calibrated outputs include partial
cancellation — they survive only because their component values make it
inexact. F2 therefore ships WITH golden re-blessing and SPICE re-validation of
those pedals (golden_regression, legends/product matrices,
screamer_drive_sweep), not as a quiet bugfix. Refined fix conditions: only
bridge when (a) the convergence node is an op-amp input pin and (b) a second
serial path source→convergence exists (mirror `sources_with_feedback`,
spqr_build.rs:1437-1461). Companion fix B (own bead): Passthrough fallback
sign (`stage.rs:2196` returns −V) + injection-node ordering via
`bfs_dist_from_in_node` with a consistent distance scale — covered by the 7
red feedforward_tests. Latent finds to bead separately: cap-coupled
non-inverting cascades blow up (+118 dB); the rt `debug-trace` feature does
not compile (bare `eprintln!` in no_std paths, solver.rs:735/973).

**F3. Inverting-topology misclassification (B3b).**
Cause: `is_inverting_topology` (`compiler/rigid/mod.rs:239-313`) recognizes
only `graph.in_node` as a signal source; a pos input fed from an upstream
op-amp output walks through the upstream feedback network to a rail and
returns "inverting", then `input_leg_resistance` picks the ground leg as Ri →
gain −1 instead of +2 (confirmed by discriminating experiment). Fix: treat any
other op-amp's `out_node` as a signal source and do not traverse through it.
Blast radius: classification of every single-VCVS stage — rerun
`opamp_gain_tests`, `black_feedback_tests`, `rigid_tests`,
`legends_regression_matrix`. Acceptance:
`dc_coupled_noninverting_cascade_has_positive_gain`.

**F4. Envelope→JFET vgs inert (G2).**
Cause: the modulation binding resolves to the stage *root*, but a JFET
variable resistor is a dynamic *leaf*; `set_jfet_vgs()` silently no-ops
(`compiler/bind.rs:593-616`, `pedalkernel-rt/src/stage.rs:2691`). The LFO path
works because it binds leaves. Fix: leaf-vs-root detection in binding; a
`ModulationTarget` variant that reaches `tree.set_jfet_vr_vgs()`; mind units —
EF output [0,1] must map to a 0→Vp (negative) gate swing. Acceptance:
`envelope_to_jfet_vgs_modulates_gain`, then `fet_leveler_compresses` (with F5).

**F5. Envelope tap nets ignored (G1).**
Cause: `EF1.in -> node` parses but is never extracted into the binding
(`bind.rs:410-441`); runtime hardwires `binding.envelope.process(input)` to
the global pedal input (`processor.rs:2813`). Fix: resolve the tap to a source
stage index at compile; feed that stage's previous-sample output (feedback
topologies) or current output (feed-forward) to the envelope. The
SidechainProcessor (one-sample-delay) is the in-engine precedent. Unlocks:
true feedback detectors — the 1176, LA-2A, and SSL architectures. Acceptance:
`fet_leveler_compresses` (with F4); a new feedback-vs-feedforward tap test.

**F6. Photocoupler series-gain override (G3).**
Cause: opto cells register both as a WDF leaf AND as an op-amp "input-path"
element; the runtime calls both setters (`processor.rs:2834-2837`), and the
input-path one applies a series gain `dc_rf/(fixed_r + R_ldr)`
(`stage.rs:3136-3151`) that swamps the correct leaf behavior → expansion.
(Static-analysis confirmed at the call site; exact misclassification point in
stage planning to pin during the fix.) Fix: bind each photocoupler as
`InWdfLeaf` xor `InOpAmpInputPath`; only call the matching setter. Acceptance:
`opto_leveler_reduces_gain_as_level_rises`, then
`opto_leveler_attack_release_in_t4b_range`.

**F7. Switched components collapse passive networks (B4/G7).**
Cause: `CapSwitched`/`InductorSwitched`/`ResistorSwitched` never override
`Component::make_leaf` (`compiler/components/passives.rs:640/719/798`; default
`component.rs:800` returns None); one None leaf poisons the S/P/Q conversion
(`spqr.rs:896/921/928`) and the AllPassive arm silently drops the stage
(`spqr.rs:1100-1109`) → zero stages, unity passthrough, dead controls. Also:
`graph.rs:4001-4003` substitutes a silent 1 kΩ resistor for failed leaves
elsewhere. Fix phase 1 (~30 lines): implement `make_leaf` mirroring each
type's `stamp_mna` math (frozen at position 0). Phase 2: live switching —
`controls()` → SwitchPosition, teach `spqr_control::bind_controls` (:19), and
extend the runtime switch handler (`dyn_node.rs:1799`) beyond
`"switched_resistor"`. Defensive: make AllPassive conversion failure a compile
error, never a silent passthrough. Acceptance: new red tests per the RCA spec
(switched-cap RC lowpass ≥15 dB cut at pos 0, ≥6 dB position separation).

**F8. VCA stub (G4).**
Cause (triple): `GraphRole::Virtual`/`StampResult::Skip`
(`components/active_ics.rs:521-533`), no `modulation_sink()` for `cv`, no
`ModulationSinkKind::VcaCv` (`component.rs:330-341`); `compile.rs:308`
hardcodes `vcas: Vec::new()`. Fix: behavioral path mirroring the working OTA
`iabc` chain — add the sink kind + target, a gain-stage root applying the
SSM2164 exponential law (−33 mV/dB-class control), `stage.set_vca_gain()`.
Full current-mode WDF two-port is the later fidelity upgrade. Acceptance:
`vca_bus_comp_compresses`.

**F9. OTA depth + input-terminal correctness (the "deterministic ≠ correct"
follow-up).** dyna_comp's OTA stage compiles to a Passthrough root; GR ceiling
~4 dB traces to port adaptation (~5 Ω port resistance vs gm) and to F1's
frozen-but-arbitrary input-node choice. Fix `build_passive_rtype_stage`'s
input-node selection to prefer signal-flow distance from `in`; revisit OTA
port adaptation; also fixes the SPICE `ota_ca3080` compile failure (rigid MNA
unsupported-kind). Acceptance: dyna_comp ratio/GR matching ElectroSmash-
documented behavior; SPICE ota fixture compiling.

**F10. Makeup stage flattens passive networks (B5/G8).**
Cause (confirmed by reproduction on triode, JFET, and op-amp variants): a
two-link chain. Link 1 — the passive-claiming pass swallows the upstream EQ
network into the active stage's group: `claim_passive_edges`
(`signal_flow.rs:743`) has a single-hop pendant pass with no barrier check
(:858-864) and a BFS expansion (:867-1009) that stops only at in/out/rails —
an EQ *between* `in` and the device input is interior and gets fully claimed
(`collect_triode_context_edges` at `spqr_build.rs:2681` is a third copy of
the same defect for pot-less groups). Link 2 — the swallowed network goes
inert: MultiNL stages fall back to injecting the source at the device's
amplifier pin when `in_node` isn't in the group (`rigid/general.rs:735-746`),
placing the network outside the source→output path (pot restamps DO fire —
mis-injection, not a stamping failure); IIR stages bake coefficients once and
`IirPotRole::Generic` writes return silently
(`pedalkernel-rt/src/stage.rs:4768-4770`). Fix: bound the claiming BFS by
signal-flow distance — never expand to a non-rail node strictly closer to
`in_node` than the device's input pin (rail-terminated bias claims stay
untouched; don't bound when distance is `None`, e.g. LFO subcircuits);
defense-in-depth: inject at the MNA node nearest `in_node` instead of the
device pin, and re-derive (or refuse) IIR for groups with Generic-role
runtime pots. Blast radius: claiming feeds every standalone tube/BJT/JFET
fixture — regression-check integration_tubes/bjt/opamp_jfet,
legends_regression_matrix, golden_regression. Acceptance: RC treble-cut +
each buffer variant must keep ≥6 dB of pot authority at 10 kHz (today ≤0.03
dB; passive baseline 11.06 dB), plus a level guard `resp(1 kHz) > −40 dB`
for the −84..−240 dB collapse mode on the full Pultec netlist.

### Phase 1b: bugs found by the census + Eurorack circuit work (2026-06-12 PM)

**F11. BBD components are never lowered** — `CompiledPedal.bbds` is only ever
`Vec::new()`; the Bbd component is Virtual/Skip with no binding pass, while
full runtime support (BbdDelayLine, ControlTarget::BbdClockRate) sits unused.
Silences boss_dm2, memory_man, boss_ce2, walrus_slo; ehx_memory_man passes
dry leakage only. Same stub pattern as F8's VCA. (Fix in flight.)

**F12. MOSFET/Zener unsupported in multi-NL MNA** — `create_nl_device`
returns None (build.rs:76); kills fulltone_ocd. Mirror the OTA dispatch arm.
(Fix in flight.)

**F13. Signal-routing bug class** (three measured manifestations, likely
shared machinery): (a) audio injected at an interior node of an op-amp
feedback network compiles to silence — also the root cause of
kick_808_wav.rs's nine silent voices, present since that test's
introduction; (b) parallel active branches do not sum — the 808 snare's two
resonators collapse to one regardless of mixer formulation, blend pots
inert; (c) a serial passive stage that carries no source→out flow hard-zeros
the chain (moog_ladder_vcf, boss_ce2 stage[2], and the 303 VCF example).
Investigate the bypass_serial decision (~spqr_build.rs:1195-1210), passive
root output extraction (stage.rs:1421-1440), and feedforward injection-node
routing. Gates the Eurorack line (303 VCF + snare blend) and synth examples.

**F14. Residual LSB-scale nondeterminism** — tweed_deluxe_5e3(_full) and
opto_leveler produce distinct in-process fingerprints (~1e-13, per-instance
RandomState at coefficient level). Candidate sites: rigid/general.rs
HashSets (:277,402,421,447), signal_flow.rs:77 rail_nodes. Breaks bit-exact
determinism (P1 brand promise) though inaudible.

Also known: pots on IIR stages are inert (existing TODO in kick_808_wav.rs
ignores; same family as F10 link-2), and golden triode_clean is 'too loud'
(peak 294.85) — pre-existing, surfaced when stale-golden aborts stopped
masking it.

### Phase 2: fidelity (after Phase 1 unblocks the topologies)

Transformer step-down ~19 dB (SPICE-confirmed; blocks line-level I/O) · RMS
detector element (dbx log-domain 35 ms averager) · stereo/linked sidechains ·
line-level test conventions (+4 dBu, 600 Ω, ±16 V) · op-amp GBW/slew inside
the loop (precision active EQ) · remaining SPICE suite failures · SPICE
fixtures for every dynamics element (stepped-Vgs JFET divider, stepped-LED
opto divider, EF timing).

### Phase 3: reference units (acceptance = published hardware behavior)

1176 (F1,F2,F4,F5) → LA-2A (F5,F6) → Pultec with makeup + switched taps
(F7,F10, transformer) → SSL/dbx VCA comps (F8 + Phase 2 RMS/stereo) → API 550A
(F2,F3 + proportional-Q) → Fairchild 670 (vari-mu + tertiary winding, mostly
exists). Each gets: static curve, ratio/knee, attack/release, THD-vs-GR, and
frequency-response acceptance tests against published specs.

### Beads (paste when `bd` is available)

```bash
bd create "Epic: outboard dynamics engine fixes" -d "reports/outboard-fix-and-product-plan-2026-06-12.md Part 1" --type epic
bd create "F1 BTreeSet terminal ordering (B1)" -d "spqr_build.rs:3380; RCA validated; tests in engine_determinism.rs" --parent {EPIC}
bd create "F2 feedforward-bridge condition (B2/B3a)" -d "spqr_build.rs:1122-1183; -x cancellation; see plan F2" --parent {EPIC}
bd create "F3 is_inverting_topology opamp-out sources (B3b)" -d "rigid/mod.rs:239-313; see plan F3" --parent {EPIC}
bd create "F4 envelope->JFET leaf binding (G2)" -d "bind.rs:593-616, stage.rs:2691; see plan F4" --parent {EPIC}
bd create "F5 envelope tap nets (G1)" -d "bind.rs:410-441, processor.rs:2813; see plan F5" --parent {EPIC} --deps {F4}
bd create "F6 photocoupler leaf-vs-input-path (G3)" -d "processor.rs:2834, stage.rs:3136; see plan F6" --parent {EPIC}
bd create "F7 switched-component make_leaf (B4)" -d "passives.rs:640/719/798, spqr.rs:1100; see plan F7" --parent {EPIC}
bd create "F8 behavioral VCA cv sink (G4)" -d "active_ics.rs:521, component.rs:330; see plan F8" --parent {EPIC}
bd create "F9 input-terminal selection + OTA depth" -d "deterministic-ne-correct follow-up; see plan F9" --parent {EPIC} --deps {F1}
bd create "F10 makeup-stage flattening (B5)" -d "RCA pending; repro fixtures preserved" --parent {EPIC}
```

## Part 2 — Future-looking: effects and formats

**Tape echo (RE-201 class) — mostly assembled already.** `delay_line` has
`tape_oxide` medium, multi-`tap` heads, `speed_mod` for LFO wow/flutter, and
`feedback` as a control target (docs/dsl.md:489-524). Work: an RE-201 example
with three heads + spring send, a wow/flutter spectrum test
(`detect_modulation_rate`), self-oscillation stability test at high feedback.
Near-term, high product value (500-series tape-echo modules are rare and
desirable).

**Spring reverb — the one missing effect primitive.** Needs a dispersive
delay element (chirped allpass cascade per spring, plus drip/boing
nonlinearity at the transducers). Candidate: new `spring(tank_model)` stateful
element following the BBD precedent (physical story, first-order). Accutronics
4AB3C1B-class tanks are well documented. This plus tape echo covers the two
big "character" sends.

**Studio EQs.** After F2/F3/F7: API 550A proportional-Q, Neve 1073-style
inductor EQ + transformer mic/line amp (gated on transformer step-down fix),
SSL E-channel parametric (gyrator-based — gyrators are op-amp+RC, exercising
exactly the topologies F2/F3 fix).

**Eurorack.** VCO/VCF/VCA/comparator/analog-switch/matched pairs/tempco all
exist; CEM3340 example already in-tree. Work: CV/gate port conventions are
in the DSL (`cv_pitch`, `gate`) — needs a module-level test harness (1 V/oct
tracking tests exist?), slew limiter + S&H + function-generator examples, and
the F8 VCA fix (SSM2164 is the Eurorack standard). A `.pedal`-defined module
rack is a credible "virtual Eurorack" product on the JACK backend.

**500-series.** Electrical conventions are researched (±16 V, +4 dBu, 130 mA
per rail, VPR spec cited in the audit). Work: a `format "500-series"`
metadata/lint layer (supply rails, current budget, pin map), the Phase-2
line-level test harness, and the reference-unit ports (Part 1 Phase 3) as the
launch module lineup.

## Part 3 — Product viability

Three pillars, all leveraging the same kernel:

**P1. Trust as the brand.** No competitor publishes SPICE-validated,
deterministic, circuit-exact models. Gates: SPICE pass rate as a tracked
headline metric (today 23/42); determinism guaranteed by F1 + a CI
determinism suite; per-unit acceptance curves vs published hardware specs
(Phase 3) published as artifacts. "Every model ships with its validation
report" is the differentiator.

**P2. Software products.** The kernel stays a library (roadmap non-goal:
no binary plugin distribution in-tree) — so the product is a *downstream*
plugin line: CLAP-first wrapper crate (CLAP is Rust-friendly; VST3/AU via
clap-wrapper), consuming the existing metering buffer (docs/metering.md UI
hooks already landed) and layout crate for panel UIs. Engine prerequisites
already on the roadmap: oversampling latency reporting; parameter smoothing
audit; preset serialization (serde already in). The JACK standalone + TUI is
the live/Linux story today.

**P3. Hardware products (the 500-series ambition).** The differentiated
pipeline: design in `.pedal` → audition in real time → SPICE-validate →
`pedalkernel-layout` → KiCad schematic → PCB. The layout crate exists with
heavy tech debt (its lib.rs opens with 17 lint suppressions) and stops at
schematic, not PCB layout. Work: harden layout crate, `.pedalhw` part/voltage
metadata (roadmap item), BOM export, and a first hardware target — the
fet_leveler as a 500-series module is the natural pilot: simple BOM, the
digital twin already in the repo, and the sim becomes the test fixture for
the physical build (compare measured curves vs the model's acceptance tests).

**Sequencing gates:** M0 = Phase 1 fixes green (acceptance suite un-ignored)
→ M1 = 1176 + LA-2A reference models validated → M2 = tape echo + spring +
EQ line, CLAP wrapper prototype → M3 = 500-series pilot module (hardware) +
plugin beta → M4 = product line decision with real usage data. Each gate is
measurable from the test suite; nothing in M1+ starts until M0's red suite
is green, because every downstream product inherits its credibility from P1.
