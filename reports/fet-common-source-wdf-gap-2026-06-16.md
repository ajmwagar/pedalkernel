# FET common-source / source-follower WDF gap

**Date:** 2026-06-16
**Status:** Characterized, not fixed. Two prerequisites landed separately
(BJT Q-point `ee55e10`, spectral metric `b2c099c`); the FET fix itself is
blocked by an SPQR tree-construction limitation documented below.

## Symptom

Active-device validation circuits with a JFET or enhancement MOSFET produce
~zero output despite "passing" the lenient validate thresholds (a null is as
close to the reference as it is far — see the dB-sign note in the metrics).

Ground-truthed peak vs ngspice (sine, oversampled):

| circuit | ngspice peak | WDF peak | ratio |
|---|---|---|---|
| `nmos_common_source` | 7.38 | ~0.0002 | ~0 |
| `jfet_source_follower` | 1.31 | ~0.0001 | ~0 |
| `pmos_source_follower` | 8.54 | 0.50 | 0.06 |

(For contrast, `npn_common_emitter` amplifies — but note it has a *separate,
pre-existing* 2× overshoot: bootstraps to 8.999 vs ref 4.499, present in the
committed code independent of any FET work. Worth a follow-up.)

## Root cause (definitive)

The drain-source WDF root sees the **wrong port resistance** — the gate-bias
network folded into its port through the supply rails:

| stage | device | port `rp` | should be ≈ |
|---|---|---|---|
| J1 | jfet | 1,022,213 (= RG, the 1 MΩ gate resistor) | ~Rd |
| M1 | nmos | 159,683 (= Rd+R1+R2+Rs) | ~Rd |
| Q1 | npn (works) | 20,013 (≈ 2·Rc) | ✓ |

The nmos passive tree is a pure **series chain**:
`R1(100k) → R2(47k) → Rs(470) → Cs → Rd(2200) → C1`, i.e. the SP
decomposition routed the drain-source port straight through the gate divider:
`drain → Rd → vcc → R1 → gate → R2 → gnd → Rs → source`.

Why the BJT is immune and the FET is not:

- A **BJT declares two ports** in `Component::ports()`:
  `[("base","emitter"), ("collector","emitter")]`. The base-emitter edge is
  *nonlinear* and its `make_leaf` is `None`, so it is **dropped** from the
  passive tree, which disconnects the base-bias network from the
  collector-emitter port. Result: port = Rc+Re.
- A **FET declares one port**: `[("drain","source")]`. There is no
  gate-source edge to break the gate off, so the gate-bias network stays
  electrically in series with the drain-source port through the rails.

The group terminals are `[in, out]` (verified), and vcc/gnd are correctly
*non*-terminals; `extract_pendants` treats rails as dead-ends — but with only
one NL edge there is nothing to split the in→out chain at the gate, so the
whole bias network folds into the single drain-source port.

## Fix approaches tried (and why each failed)

1. **Treat `vcc_node` as a DC reference in `compute_group_terminals`**
   (symmetric with `gnd_node`, which *is* skipped). Result: **no effect** on
   the FET `rp` (1M / 160k unchanged) and it spawned a spurious extra stage.
   vcc was already a non-terminal here; this is not the lever.

2. **Add a gate-source port to the FET (mirror the BJT's two-edge topology).**
   Blocked by the FET's `make_leaf` returning `Some(JfetVrNode)` (the device
   is *dual-purpose* — it is also the passive variable-resistor used by LFO
   phasers). So a second gate-source edge would become a leaf *in* the tree
   rather than being dropped like the BJT's base-emitter edge — it would not
   isolate the gate. Mirroring the BJT cleanly would require coordinated
   changes across `ports()`, `classify_nonlinear`, `make_leaf`, the SingleNl
   representative-edge selection, and the JfetVr dual-use — high blast radius.

3. **Prune the gate subnetwork and re-decompose with `[drain, source]` as the
   port** (contained, in the NlWdf builder; the gate is physically a high-Z
   control input driven via `set_control_voltage` + the DC Q-point, not the
   WDF tree). The prune fired correctly (gate edges removed), but the
   re-decomposition produced a **wrong tree**: nmos `rp`=0.13 (the source
   bypass cap Cs dominated and the drain load Rd was lost in pendant
   reassembly), jfet `rp`=1803. The SP pendant-reinsertion does not cleanly
   assemble a two-terminal drain-source port from two pendants sitting at the
   two different port nodes.

## What the fix needs (roadmap)

The bias side is fully solved and verified (kept out of tree for now):

- `vgs_bias` on `JfetRoot`/`MosfetRoot` + `set_bias`/`vgs_bias()` accessors.
- `set_control_voltage` Jfet/Mosfet arms (`vgs = vgs_bias + input`).
- `solver_control` = `vgs() - vgs_bias()` (AC axis, matching the K-table).
- `compute_wdf_fet_dc_qpoint` (square-law self-bias load-line; solved
  nmos Vgs = 2.218 V, matching hand calculation) returning a shared
  `DcQPoint { input_bias, output_warm_start, bypass_drop }`.

The K-DBG confirmed these are correct and *sufficient* — the K-table swings
fully across the control axis *once `rp` is the drain load*. The remaining
work is purely the **tree/port construction**:

> Make the FET drain-source root see only its drain/source loads, with the
> gate-bias network excluded — the WDF equivalent of the BJT's dropped
> base-emitter edge.

The cleanest principled path is approach (3) done correctly: prune the gate
subnetwork **and** fix the pendant-reassembly so the rebuilt tree is
`Series(Rd-side, Rs‖Cs-side)` with `rp ≈ Rd` (source bypassed). That requires
understanding `insert_pendant_at_junction` for the two-terminal-port case, or
building the drain/source port tree directly rather than via the generic SP
pendant path. This is dedicated SPQR-decomposition work and should land with
a full regression gate (tubes, diodes, opamp, pedals, the JfetVr phaser and
source-follower paths) — not a surgical change.

## OTA (`ota_ca3080`) — deeper, separate

Out of scope here but related: the CA3080 is a 4-terminal transconductor
(sense `pos−neg`, inject `Iout` at the load) but is modeled as a 1-port
`Vccs` across `pos→neg`, so the output current is never injected at the load
node → null. This is an architectural WDF limitation (current-output
injection), not a netlist or bias fix.
