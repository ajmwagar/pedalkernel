# DspBlock Generalization — Spec (2026-06-13)

Supersedes the narrow `DspBlock` trait landed in `d64dfe1`. Companion to
`reports/architecture-debt-2026-06-12.md` §4 (which records the narrow trait as
DONE — this generalizes it) and the realtime-or-bust standard. For review, then
implement.

## 1. Why

The trait we just landed models a **serial audio insert**: `boundary_nodes ->
Vec<usize>` is a flat node list, the BBD runtime splices into the main path
through a *global* `bbd_wet_mix` (`processor.rs:3787`), and "controls" only
arrive via control-rate bindings. That shape was fit to its two examples (BBD,
VCA). It cannot express a generator (VCO: no input side), multi-output blocks
(VCO saw/tri/pulse; stereo), CV-as-audio-rate-input, or per-instance mix.

Those are **not fundamental limits**. A `DspBlock` runs its own DSP *outside the
WDF/MNA scattering solver*, so the cost rule that governs WDF elements does not
apply to it (see §3). Generalizing the trait to a structured **port signature**
turns `DspBlock` from "effect insert" into the engine's general **behavioral DSP
node** — the clean hybrid boundary: WDF/MNA solves the analog signal path; a
`DspBlock` hosts anything better *computed* than circuit-simulated (PolyBLEP
oscillators, digital delay/reverb, lookahead, FFT-domain work); voltage ports
are the connective tissue. PolyBLEP is then just *the algorithm inside* a VCO
block — the same relation `tanh` has to the OTA model.

## 2. The port signature (replaces `boundary_nodes`)

Harmonize with the existing port vocabulary (`Component::ports`,
`SignalTerminals` at component.rs:191, `EdgeKind::Behavioral` at :255). A block
declares a typed signature; the lowering machinery consumes it generically.

```rust
/// How a behavioral block connects to the surrounding circuit.
pub(super) struct BlockIo {
    /// Nodes whose voltage the block READS each sample. Empty ⇒ generator.
    pub inputs:  Vec<BlockPort>,
    /// Nodes the block DRIVES (as a voltage source) each sample. Empty ⇒ sink.
    pub outputs: Vec<BlockPort>,
}
pub(super) struct BlockPort {
    pub node: NodeId,
    pub pin:  &'static str,   // component pin this maps to (.in/.out/.cv/.clock/.saw…)
    pub role: PortRole,       // Audio | Cv | Gate | Clock | Sync
}
```

`PortRole` is metadata for the binder/UI/validation (e.g. a `Cv` input may be
1 V/oct-scaled); the runtime treats every input as "read node voltage" and every
output as "write node voltage." The trait method:

```rust
fn io(&self, pedal: &PedalDef, graph: &CircuitGraph) -> Vec<(ComponentId, BlockIo)>;
```

replaces `boundary_nodes`; `all_boundary_nodes` (dsp_block.rs) becomes "flatten
every block's input+output nodes" — same downstream use (SPQR terminals + gap
split), now structured.

## 3. The cost model, written as law

This is the invariant that makes the generalization safe — it must live as a
doc-comment on the trait and as the reviewer's acceptance criterion:

> **A `DspBlock` never participates in the WDF/MNA scattering solve.** Its input
> ports are node *reads* (`node_signals[n]`); its output ports are node *writes*
> (`b = 2v − a`, the existing port-injection path, processor.rs:1334/2862).
> Neither grows or re-derives any per-sample matrix. Therefore a block may have
> **any number of ports** and **audio-rate internal controls** at constant
> per-sample cost.

Corollary — the two "virtual control" cost classes that were conflated:

| Control modulates… | Cost | Rate | Example |
|---|---|---|---|
| a **WDF-stamped element** (pot wiper, JFET Rds, opto R_ldr) | re-derive scattering (matrix) | control-rate / gated | F6 photocoupler, pots |
| a **DspBlock internal parameter** (phase_inc, gain, clock) | float update | **audio-rate, free** | VCO pitch, VCA CV |

So a block control may be driven by *either* a control-rate binding (a pot) *or*
an **audio-rate CV input port** — both legal, both cheap. The expensive path is
only ever a WDF element reacting to a changing coefficient, which a block, by
definition, is not.

## 4. Generator / source placement

The current `split_groups_at_behavioral_gaps` (bbd_lowering.rs:108) assumes a
two-sided island (audio in ↔ audio out, galvanically separated). Generalize by
input-port count:

- **N ≥ 1 inputs** → gap-bridge as today (split groups at the block; each input
  node terminates one side, each output node seeds the next).
- **N = 0 (generator: VCO)** → no gap to split; the block is a **source** placed
  at the head of the flow group its output node belongs to, injecting like a
  voltage source. Reuses the port-write path; no new machinery, just the N=0
  branch in placement.

## 5. Output→tree feedback: the one real constraint

An output port that feeds back into the WDF tree and influences the tree within
the *same* sample is a delay-free loop. Resolve with the **1-sample-delay
convention** already used by sidechain CV (`SidechainProcessor::cv_delayed`) and
F5's feedback taps: the block writes its output, the tree reads last sample's
value. Document per block whether an output is feed-forward (same-sample) or
feedback (delayed); default feedback when a cycle is detected.

## 6. Per-instance state

Kill the global `bbd_wet_mix` (processor.rs:1320). Each block instance owns its
mix/state; the binder maps `BBD2.mix` to instance 2, not a single field. This is
required for two BBDs/springs with different settings (walrus_slo already has 2)
and is the honest model of circuit placement.

## 7. Migration (the dispatch: trait + BBD + VCA, one pass, zero-behavior-change)

- `BbdBlock`: `io` = 1 Audio input (BBD.in node), 1 Audio output (BBD.out node);
  controls clock/feedback/mix (keep current bindings); per-instance mix.
- `VcaBlock`: `io` = 1 Audio input, 1 Audio output, 1 Cv input *or* control-rate
  CV binding (keep current envelope-routed CV for zero-change; the Cv-port path
  is new capability, tested separately).
- Acceptance: **byte-identical** test battery + golden zero-drift + determinism
  fingerprints unchanged, exactly as the d64dfe1 refactor was verified. The
  migration is a no-op for existing circuits; it only *adds* expressible shapes.

## 8. The remaining three, as port signatures (land after §7)

- **VCO** (`reject_unlowered` currently errors it): `io` = **0 inputs**, outputs
  = {saw, tri, pulse} Audio + the block reads CV via `cv`/`cv_pitch`/`pwm`/`sync`
  **input ports**. PolyBLEP `Vco` element (synth.rs, already implemented) is the
  body. Revives `cem3340_vco`, `minisynth`. This is the proof the generalization
  works — VCO stops being a special case.
- **delay_line + tap (F15)**: `io` = 1 Audio in, 1 Audio out + tap outputs;
  controls delay_time/feedback/mix as CV-or-pot; fixes the dead control surface
  (taps hardcoded `vec![1.0]` at spqr_build.rs:1928, pots/speed_mod unbound).
  Un-ignores the RE-201 + Dimension D gated tests and the 4 red `control_binding`
  delay tests.
- **spring** (`reports/spring-reverb-design-2026-06-13.md`): `io` = 1 Audio in,
  1 Audio out; dwell/damping controls. The dispersion algorithm is the body.

## 9. Verification

- §7 migration: zero-behavior-change battery (the d64dfe1 protocol).
- Each §8 block: red-first acceptance test (the existing `#[ignore]`d ones where
  they exist), realtime bench ≥ target per the realtime standard, determinism.
- A new `dsp_block` unit test asserting the cost law structurally: a block with
  ≥2 ports and a CV control compiles and runs allocation-free at audio rate
  (process-path allocation audit).

## 10. Open questions for review

1. **`PortRole` taxonomy** — is `Audio | Cv | Gate | Clock | Sync` enough, or do
   we want it open-ended? (Leaning closed enum, matching `EdgeKind`/`ModulationSinkKind` style.)
2. **VCO graph role** — it is currently `GraphRole::ActiveIc` in the comment at
   component.rs:358 yet treated as Virtual by the gate. Reconcile: a generator is
   `Virtual` (no WDF edge) that happens to be a source. Confirm no `ActiveIc`
   logic depends on VCO.
3. **CV-port vs control-binding for the same parameter** — allow both wired
   simultaneously (summed), or error? (Leaning: sum, like a real CV + offset knob.)
4. **Feedback detection** — auto-detect output→tree cycles and default to
   delayed, or require the block to declare per-output? (Leaning: auto-detect,
   declare to override.)
5. Does generalizing `boundary_nodes → io` disturb the d64dfe1 zero-drift
   guarantee for BBD/VCA? (Must not — §7 is the gate.)
