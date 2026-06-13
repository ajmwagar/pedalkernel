# CrossNetworkCoupler — Implementation Plan (DELAYED path)

**Date:** 2026-06-13 · **Status:** spec for review · **Branch:** la2a-1to1
**Design parent:** `reports/cross-network-coupling-design-2026-06-13.md` (READ — decisions:
subsume SidechainProcessor; tight path = transformer already done; this build is the
DELAYED path only). This doc is the next level down: trait shape, the over-fusion fix,
migration, the optocoupler client, and the test gates.

## Executive summary (8 lines)

1. The faithful `la2a.pedal` sits at -140 dB because the detector net `out -> … -> PC1.led`
   closes a real loop, `has_feedback()` (spqr_build.rs:556) lumps T_in+V3+T_out+I/O+sidechain
   into ONE rigid R-type adaptor (:597), whose output port scatters to [0,0] → silence.
2. The fix: make the optocoupler a **non-merge barrier with a 1-sample delay** so the
   detector loop is never an MNA feedback edge → de-fuses the output AND makes the detector live.
3. De-fusion, GAP G (detector-from-circuit), and GAP H (program release) are ONE problem, ONE fix.
4. `CrossNetworkCoupler` = pluggable coupling-domain state (`Cv` for sidechain, `Light`+two-rate
   for opto), per-side `contribute`/`read`, `Delayed{samples}` policy. SidechainProcessor = `Cv` flavor.
5. **CONTRADICTION TO FLAG (§3):** `build_sidechains()` (bind.rs:770) is defined but **NEVER CALLED**;
   no shipped `.pedal` uses a `sidechains {}` block; the existing CV→push-pull loop has zero live
   clients. The "migrate existing sidechain clients, byte-identical" gate has **nothing to migrate**.
6. Reuse the DspBlock gap-cut discipline (`all_boundary_nodes`, dsp_block.rs:238) + the
   `cv_delayed` eval-order loop (processor.rs:3811-3834). The bridge is a coupling-state hop, not audio.
7. The LED becomes a real electrical port (`led.a`/`led.b`) in the sidechain network; `set_led_drive`
   math → `contribute`, `port_resistance` → `read`; delete the `PhotocouplerLed` modulation-sink path.
8. Minimal first PR (§6) = the barrier + gap-cut alone, gated on the structural de-fusion check; it
   un-collapses the LA-2A output even before the optical `contribute`/`read` lands.

Report path: `reports/cross-network-coupler-impl-plan-2026-06-13.md`.

---

## 1. Trait / struct shape

The coupling domain is pluggable; the coupler owns per-side rules and a policy.

```rust
// pedalkernel-rt: new module elements/coupler.rs
pub enum CouplingDomain {                 // the persisted non-electrical state
    Cv(crate::Wave),                      // sidechain control voltage (subsumes SidechainProcessor)
    Light { state_fast: crate::Wave, state_slow: crate::Wave }, // opto two-rate cell (controlled.rs:106-118)
    // future: Thermal, Armature
}

pub enum CouplingPolicy { Tight, Delayed { samples: usize } } // transformer=Tight; opto/sc=Delayed{1}

pub struct PortSample { pub voltage: crate::Wave, pub current: crate::Wave, pub power: crate::Wave }
// answers Open-Q "what does contribute read": expose all three (LED=current-ish, EL panel=voltage)
pub enum PortParam { Resistance(crate::Wave), Cv(crate::Wave) } // what `read` writes back into a side

pub struct CouplingSide { pub id: SideId, pub network_tag: usize, pub drives: bool, pub senses: bool }

pub trait CrossNetworkCoupler {
    fn sides(&self) -> &[CouplingSide];
    fn policy(&self) -> CouplingPolicy;
    fn contribute(&mut self, side: SideId, port: PortSample, dt: crate::Wave); // drive edge → update state
    fn read(&self, side: SideId) -> PortParam;                                  // sense edge → next-solve param
    fn reset(&mut self);
}
```

**SidechainProcessor expressed as `Cv`** — its current shape is
`{ circuit: CompiledPedal, cv_delayed: Wave }` (stage.rs:6620; `process()`/`set_control()`/`reset()`
at :6627). It already IS a delayed coupler: `contribute(drive_side, port, dt)` runs
`self.circuit.process(port.voltage)` and stores into `coupling_state = Cv`; `read(sense_side)`
returns `PortParam::Cv(last_state)`; `policy() == Delayed{1}` is the existing eval-order delay
(processor.rs:3825 `sc.cv_delayed = cv`). The wrapper is mechanical — see §3.

**Optocoupler expressed as `Light`** — side `led` is a drive edge: `contribute` is exactly the
`set_led_drive` two-rate math (controlled.rs:166-205: asymmetric `alpha_fast/alpha_slow`,
`effective_light = (1-w)·fast + w·slow`). Side `ldr` is a sense edge: `read` is the log-interp
resistance (controlled.rs:188-203) returned as `PortParam::Resistance`. `policy()==Delayed{1}`.
`Photocoupler` (controlled.rs:103-128) already holds the entire `Light` state; the trait just
re-homes its two existing entry points.

**Transformer (Tight, already shipped — no new lowering)** — `policy()==Tight` lowers to the
existing `DynNode::TransformerNode` (dyn_node.rs:509, `reflected_with_state` at :651). The trait
only *describes* it; `contribute`/`read` collapse into the single multi-port scatter. Per the
design decision, we do not route the transformer through the new evaluator.

## 2. Compiler change — the over-fusion fix (THE de-fusion)

**Where group membership is decided.** `find_flow_groups` (signal_flow.rs:1220) builds SCCs over a
directed flow-dependency graph (Tarjan, :475), then merges via `merge_same_component_sccs` (:618)
and `merge_shared_active_node_sccs` (:678). A `FlowGroup` (signal_flow.rs:23) is a set of edges;
`group.has_feedback()` true → spqr_build.rs:556 builds ONE rigid adaptor over `group.all_edges()`
at :597. The detector loop makes the whole output network one feedback group.

**The fix is to stop the loop from ever closing through the electrical graph.** The optocoupler's two
sides are *galvanically isolated* — they should never have shared an electrical group. Make the
coupler a **non-merge barrier**, identical in mechanism to the DspBlock galvanic cut:

- Photocoupler component declares its electrical edges with `EdgeKind::Behavioral` + `StampResult::Skip`
  (the BBD/delay pattern, delay.rs:56-65). A `Behavioral` edge is **never claimed into any FlowGroup**
  (find_flow_groups only claims Linear/Reactive/Nonlinear/Vccs/Vcvs), so it cannot union the led-side
  network with the ldr-side network. This is the opposite of `merge_shared_active_node_sccs`.
- Register the coupler's side nodes as global terminals. Extend `all_boundary_nodes` (dsp_block.rs:238)
  (or add a sibling `coupler_boundary_nodes`) so SPQR forces stage ports at the led.a/led.b and ldr.a/ldr.b
  nodes — the same treatment as `in_node`/`out_node`. Now the output network (T_in…T_out) and the
  sidechain front-end are separate groups; the output group is NOT a feedback group; its scattering
  returns to ~[+0.97, -0.69] (PROVEN by cutting the two feedback nets).
- Reuse `bbd_lowering::split_groups_at_behavioral_gaps` (bbd_lowering.rs:152) to split any residual
  group at the gap; the coupler edges are already absent from those groups.

**Bridge = coupling-state hop, NOT audio.** Unlike DspBlock (which carries audio across the cut via
`node_signals` reads + `b = 2v−a` writes, dsp_block.rs:94-101), the coupler carries only the
non-electrical scalar. Runtime order: solve net1 (sidechain) → `contribute` (drive node sample →
Light state) → solve net2 (audio), whose `ldr` leaf `read`s **last sample's** state. The 1-sample
delay is the F5 eval-order convention (processor.rs:3663-3668, 3811-3834) — free, no joint matrix.

## 3. SidechainProcessor migration (zero-behavior-change) — **see CONTRADICTION**

**Findings (verified):** `build_sidechains()` (bind.rs:770-878) is **never called**. Every
`CompiledPedal` initializes `sidechains: Vec::new()`. `pedal.sidechains` is read only at
classify.rs:111 (to mark `sidechain_edge_set`). No shipped `.pedal` contains a `sidechains {}` block —
every "sidechain" hit in examples is a comment. The "Dyna Comp OTA sidechain" uses an
`envelope_follower` element bound at compile time, NOT a SidechainProcessor; no Fairchild 670 `.pedal`
exists. **There are zero live clients to migrate.** The design doc's "migrate existing clients,
byte-identical" gate is vacuous as written.

**Recommended migration steps (de-risked given the above):**
1. Keep `SidechainProcessor` as-is; wrap it (don't rewrite) so it `impl CrossNetworkCoupler` with
   `coupling_state = Cv`, `policy = Delayed{1}`. Pure adapter — same fields, same `process()` body.
2. Verification protocol: because nothing exercises the old path, the "byte-identical" bar is met by
   a **construction unit test** (assert the wrapped coupler's per-sample `contribute`/`read` produces
   the same `cv_delayed` sequence as the raw `SidechainProcessor::process` loop on a synthetic CV
   input). This is the DspBlock-generalization discipline applied to a dormant path.
3. Do NOT spend a PR "migrating clients" that don't exist. Fold this into the first delayed-path PR.

## 4. Optocoupler optical client (resolves GAP G)

- **LED becomes a real electrical port-pair.** Photocoupler `pin_config` (modulation.rs:~210) gains
  `led.a`/`led.b`; `graph_role` stays `Edge{a,b}` for the LDR side, plus the new led edge declared
  `EdgeKind::Behavioral` (so it does not stamp and does not fuse). The la2a.pedal net
  `EL_drive.b -> PC1.led` (la2a.pedal:237) becomes `EL_drive.b -> PC1.led.a` / `gnd -> PC1.led.b`
  (LED across the EL-drive winding) — now a genuine node in the sidechain network.
- **Light = coupling state.** The drive sample at the led node feeds `contribute` (the two-rate cell,
  controlled.rs:166-205). The LDR leaf in the audio network calls `read` → `Resistance`, throttled like
  any controlled resistance (stage.rs:1565-1575). Loop closed by `Delayed{1}`.
- **Delete the modulation-sink path.** Remove `ModulationSink{PhotocouplerLed}` from
  `modulation_sink()` (modulation.rs) and the `PhotocouplerLed` arm of `build_envelope_jfet_bindings`
  (bind.rs:716-755) and its runtime dispatch (processor.rs:2799-2810). GAP G dies: the LED is no longer
  bound only from an `envelope_follower.out`; any circuit net into `led` drives the cell.
- **GAP H (note, do not fully spec).** Once `contribute` owns the cell, program-dependent τ is a
  state extension on `CouplingDomain::Light` (add a slow GR-history accumulator that lengthens
  `tau_*_fall` after heavier/longer GR; t4b() fixed taus at controlled.rs:85-94). Follow-on bead.

## 5. Test gates

- **Forward-level floor (the RCA gate).** `la2a` steady gain `> -40 dB` (RED today ~-125/-140 dB).
  Promote `la2a_compiles_and_is_healthy` (la2a_acceptance.rs:55, `assert_healthy` floor 100.0).
- **De-fusion structural check.** No single stage's `debug_label` contains BOTH `T_in` and `T_out`.
  `debug_label` is the comma-joined `group_comp_ids` (`group_label`, spqr_build.rs:517 → :355). A new
  test compiles `la2a.pedal --no-default-features`, inspects `CompiledPedal` stages, asserts the
  output group is no longer one fused R-type spanning both transformers.
- **GAP G dynamics.** `la2a_reduces_gain_as_level_rises` (la2a_acceptance.rs:74): loud_gain < quiet_gain,
  GR > 3 dB (correct downward direction — shunt cell, dark=high R). Plus `la2a_gr_curve_matches_published_shape`
  (monotonic, total GR > 20 dB) and `la2a_attack_near_10ms` (1-50 ms).
- **Zero-change on the dormant sidechain path.** Unit test from §3.2 (synthetic CV sequence identical
  through wrapped coupler vs raw SidechainProcessor).
- **Realtime.** Each side solved in its own network (no joint matrix); coupling = one scalar
  `contribute`/`read` + free eval-order delay (design §9). Assert stage count does not regress to a
  single mega-group; bench the per-sample cost ≈ today's `set_led_drive` + one node read.
- **GAP H** stays `#[ignore]` (la2a_acceptance.rs:137) — follow-on.

## 6. Phasing (independent landings, each with its own gate)

- **PR-1 (the de-fuse — minimal first PR).** Photocoupler led edge → `EdgeKind::Behavioral` +
  boundary-node registration (§2). No `contribute`/`read` yet (LED still effectively dark). Gate:
  **de-fusion structural check** + `la2a_compiles_and_is_healthy` flips green (output > -40 dB). This
  alone un-collapses the LA-2A output — the keystone.
- **PR-2 (trait + Cv wrapper).** Add `CrossNetworkCoupler`, `CouplingDomain::Cv`, `Delayed{1}`; wrap
  SidechainProcessor. Gate: §3.2 construction unit test (dormant-path equivalence).
- **PR-3 (optical client).** `Light` domain; `contribute`/`read` from controlled.rs math; delete the
  modulation-sink path; re-wire la2a.pedal:237 to `led.a`/`led.b`. Gate: GAP G dynamics tests green.
- **PR-4 (GAP H, separate bead).** GR-history τ on `CouplingDomain::Light`.

## 7. Open implementation questions

1. **PR-1 with a dark LED:** does cutting the led edge to `Behavioral` and removing the loop, but NOT
   yet driving the cell, leave the LDR at `r_dark` (3 MΩ)? If so the divider is near-bypass — confirm
   `assert_healthy` passes at the static operating point before `contribute` exists.
2. **PortSample drive quantity at the led node:** voltage, current, or power? `set_led_drive` takes a
   normalized 0..1 `led_drive` — what maps the solved EL-drive node sample onto that 0..1? (Peak
   Reduction pot scaling + a rectify/normalize step must live in `contribute` or a pre-stage.)
3. **Boundary-node registration scope:** extend `all_boundary_nodes` (dsp_block.rs:238) vs a new
   `coupler_boundary_nodes` — which keeps the DspBlock and coupler concerns cleanly separated?
4. **Galvanic-short validation:** should the compiler reject a user net that shares a node across the
   coupler's two sides (defeating the isolation)? Design Open-Q; recommend a warn, not a hard error, in v1.
5. **Migration scope:** given §3 (no live clients), is PR-2 worth landing now, or defer the trait until
   a second delayed client exists and ship PR-1+PR-3 against the concrete Photocoupler first?

## Contradictions vs the design doc's assumptions

- Design §Decisions and §10.2 assume **existing sidechain clients** (Dyna Comp OTA, 670/push-pull) to
  migrate byte-identically. **Verified false:** `build_sidechains()` is never called; no `.pedal` uses
  `sidechains {}`; the CV→push-pull loop (processor.rs:3811-3834) has no live producer. The
  zero-behavior-change bar has no live subject — re-scope as a dormant-path equivalence unit test (§3.2).
- Design §6 implies the coupler needs new "per-side, no-fusion" graph machinery. **It already exists**
  as the `Behavioral`-edge + `all_boundary_nodes` gap-cut (DspBlock); the coupler reuses it verbatim,
  differing only in the bridge (scalar hop, not audio). No new merge-prevention pass is required.
