//! VCA lowering: turn `vca()` components into runtime gain elements plus
//! their CV modulation bindings (audit gap G4).
//!
//! The `Vca` component is `GraphRole::Virtual` — it contributes no electrical
//! edges, so the circuit graph is galvanically split at `VCA.in`/`VCA.out`,
//! exactly like `bbd()` (see `super::bbd_lowering`, the precedent this module
//! mirrors). Before this pass existed the SPQR pipeline left
//! `CompiledPedal.vcas` empty and `EF.out -> VCA.cv` nets were inert, so VCA
//! circuits either went silent (gap in the audio path) or shipped with a
//! documented hard-wired bypass and zero gain reduction. This module:
//!
//! 1. builds one runtime `Vca` per `vca()` component (`vcas[idx]` order =
//!    component declaration order — the index contract used by
//!    `ModulationTarget::VcaCv`),
//! 2. reuses `bbd_lowering::split_groups_at_behavioral_gaps` so the passives
//!    on each side of the behavioral gap become separate serial stages (the
//!    runtime applies the VCA gain in the serial chain after the stages —
//!    gain commutes with the linear stage chain),
//! 3. binds envelope followers wired to `VCA.cv` as
//!    `ModulationTarget::VcaCv` modulators (the F4/F5 detector-tap machinery
//!    resolves where the detector reads its signal), and
//! 4. enforces the architecture-debt §4 principle: **behavioral islands must
//!    lower or fail to compile**. If a `vca()` exists and lowering produces
//!    no runtime instance for it, compilation FAILS with an error naming the
//!    component — never ship silent. The same mechanism rejects `vco()`
//!    components outright (VCO lowering does not exist yet; previously
//!    `cem3340_vco.pedal` compiled to a silent processor).

use hashbrown::HashMap;

use crate::dsl::{PedalDef, Pin};

use super::compiled::{CompiledPedal, EnvelopeBinding, ModulationTarget, VcaBinding};
use super::components::Vca;
use super::dsp_block::{behavioral_island_error, BlockIo, BlockPort, DspBlock, PortRole};
use super::graph::CircuitGraph;

/// Pin names a `vca()` accepts on its audio input side.
const VCA_IN_PINS: &[&str] = &["in", "input", "in1", "in2", "in3", "in4"];
/// Pin names a `vca()` accepts on its audio output side.
const VCA_OUT_PINS: &[&str] = &["out", "output", "out1", "out2", "out3", "out4"];

/// The VCA [`DspBlock`]: lowers `vca()` components to runtime `VcaBinding`s
/// (`compiled.vcas`) plus their CV-modulation bindings. The type-specific
/// guts live as `pub(super)` helpers in this module.
pub(super) struct VcaBlock;

impl DspBlock for VcaBlock {
    fn handles(&self, type_tag: &str) -> bool {
        type_tag == "VCA"
    }

    fn component_ids(&self, pedal: &PedalDef) -> Vec<String> {
        vca_component_ids(pedal)
    }

    fn io(&self, pedal: &PedalDef, graph: &CircuitGraph) -> Vec<(String, BlockIo)> {
        vca_io(pedal, graph)
    }

    fn bind_runtime(
        &self,
        pedal: &PedalDef,
        compiled: &mut CompiledPedal,
        sample_rate: f64,
    ) -> Result<(), String> {
        compiled.vcas = lower_vcas(pedal, sample_rate)?;
        bind_vca_runtime(pedal, compiled, sample_rate);
        Ok(())
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// (1) Runtime Vca construction
// ═══════════════════════════════════════════════════════════════════════════

/// All `vca()` component IDs in declaration order. `vcas[i]` corresponds to
/// `vca_component_ids(pedal)[i]` — the index `ModulationTarget::VcaCv`
/// refers to.
pub(super) fn vca_component_ids(pedal: &PedalDef) -> Vec<String> {
    pedal
        .components
        .iter()
        .filter(|c| c.kind.as_any().downcast_ref::<Vca>().is_some())
        .map(|c| c.id.clone())
        .collect()
}

/// True if any net references `comp_id.<pin>` for one of `pins`.
fn pin_wired(pedal: &PedalDef, comp_id: &str, pins: &[&str]) -> bool {
    pedal.nets.iter().any(|net| {
        std::iter::once(&net.from).chain(net.to.iter()).any(|p| {
            matches!(p, Pin::ComponentPin { component, pin }
                if component == comp_id && pins.contains(&pin.as_str()))
        })
    })
}

/// Build one runtime `Vca` per `vca()` component (declaration order).
///
/// The runtime element sits in the serial chain (`CompiledPedal::process`
/// applies it after the stage loop, mirroring the BBD wet path), reading the
/// serial signal (`input_node_id == usize::MAX`) and writing it back. Gain
/// is unity until `ModulationTarget::VcaCv` routing drives the `cv` port.
///
/// Errors (never ship silent — see module docs): a `vca()` whose audio pins
/// are not wired into the netlist cannot be placed in the audio path, so
/// lowering would produce a dangling instance while the circuit's behavioral
/// gap stays unbridged.
pub(super) fn lower_vcas(pedal: &PedalDef, sample_rate: f64) -> Result<Vec<VcaBinding>, String> {
    let mut vcas = Vec::new();
    for comp in &pedal.components {
        if comp.kind.as_any().downcast_ref::<Vca>().is_none() {
            continue;
        }
        let in_wired = pin_wired(pedal, &comp.id, VCA_IN_PINS);
        let out_wired = pin_wired(pedal, &comp.id, VCA_OUT_PINS);
        if !in_wired || !out_wired {
            return Err(behavioral_island_error(
                &comp.id,
                "vca()",
                &format!(
                    "its audio port is not wired into the netlist (in wired: {in_wired}, \
                     out wired: {out_wired}), so no runtime gain element can bridge the \
                     behavioral gap"
                ),
            ));
        }
        vcas.push(VcaBinding {
            vca: pedalkernel_rt::elements::Vca::new(),
            // ADSR is only used in trigger-gated mode; `trigger_idx: None`
            // selects CV-driven mode (gain owned by VcaCv routing).
            envelope: pedalkernel_rt::elements::AdsrEnvelope::new(sample_rate as crate::Wave),
            trigger_idx: None,
            input_node_id: usize::MAX,  // serial chain input
            output_node_id: usize::MAX, // serial chain output
            comp_id: comp.id.clone(),
        });
    }
    Ok(vcas)
}

// ═══════════════════════════════════════════════════════════════════════════
// (2) Behavioral gap boundaries
// ═══════════════════════════════════════════════════════════════════════════

/// Typed port signature (spec §2) of each `vca()`: 1 Audio input (the VCA `in`
/// node), 1 Audio output (the VCA `out` node), in component declaration order.
/// CV is bound separately (control-rate envelope routing, kept for zero
/// behavior change); the Cv-port path is new capability that lands with VCO.
///
/// These nodes are behavioral stage boundaries, exactly like
/// `bbd_lowering::bbd_io`: the netlist is galvanically cut there, so each one
/// must be treated like a global terminal when computing group terminals —
/// otherwise the dangling side of a passive group has no port and compiles into
/// a stage whose probe reads 0. `all_boundary_nodes` flattens inputs-then-
/// outputs, recovering exactly the node set the previous flat
/// `vca_boundary_nodes` produced for the 1-in/1-out circuits that exist today.
/// Group splitting itself is shared with the BBD pass
/// (`bbd_lowering::split_groups_at_behavioral_gaps`).
pub(super) fn vca_io(pedal: &PedalDef, graph: &CircuitGraph) -> Vec<(String, BlockIo)> {
    let resolve = |id: &str, pins: &[&'static str]| -> Option<BlockPort> {
        for &pin in pins {
            if let Some(&node) = graph.node_names.get(&format!("{id}.{pin}")) {
                return Some(BlockPort {
                    node,
                    pin,
                    role: PortRole::Audio,
                });
            }
        }
        None
    };
    vca_component_ids(pedal)
        .into_iter()
        .map(|id| {
            let inputs = resolve(&id, VCA_IN_PINS).into_iter().collect();
            let outputs = resolve(&id, VCA_OUT_PINS).into_iter().collect();
            (id, BlockIo { inputs, outputs })
        })
        .collect()
}

// ═══════════════════════════════════════════════════════════════════════════
// (3) CV modulation binding
// ═══════════════════════════════════════════════════════════════════════════

/// Bind envelope followers wired to `VCA.cv` as `ModulationTarget::VcaCv`.
///
/// Must run after the stage list is final (it resolves detector taps against
/// `compiled.stages` via the F4/F5 machinery in `bind::resolve_envelope_tap`)
/// and only appends to `compiled.envelopes` — existing bindings keep their
/// indices. Bias/range come from the VCA component's `modulation_sink("cv")`
/// declaration (see `components::active_ics::Vca` for the scaling rationale).
///
/// Scope: direct `EF.out -> VCA.cv` nets, mirroring
/// `bind::build_envelope_jfet_bindings`. LFO-driven `cv` (tremolo) routes
/// through the same `ModulationTarget::VcaCv` arm but is not auto-bound yet.
pub(super) fn bind_vca_runtime(pedal: &PedalDef, compiled: &mut CompiledPedal, sample_rate: f64) {
    let vca_ids = vca_component_ids(pedal);
    if vca_ids.is_empty() {
        return;
    }
    let vca_id_to_idx: HashMap<&str, usize> = vca_ids
        .iter()
        .enumerate()
        .map(|(i, id)| (id.as_str(), i))
        .collect();

    for comp in &pedal.components {
        let Some(ef) = comp
            .kind
            .as_any()
            .downcast_ref::<super::components::EnvelopeFollower>()
        else {
            continue;
        };

        for net in &pedal.nets {
            let Pin::ComponentPin { component, pin } = &net.from else {
                continue;
            };
            if component != &comp.id || (pin != "out" && pin != "output") {
                continue;
            }
            for to_pin in &net.to {
                let Pin::ComponentPin {
                    component: tcomp,
                    pin: tpin,
                } = to_pin
                else {
                    continue;
                };
                let Some(&vca_idx) = vca_id_to_idx.get(tcomp.as_str()) else {
                    continue;
                };
                let Some(sink) = pedal
                    .components
                    .iter()
                    .find(|c| &c.id == tcomp)
                    .and_then(|c| c.kind.modulation_sink(tpin))
                else {
                    continue;
                };

                let envelope = crate::elements::EnvelopeFollower::from_rc(
                    ef.attack_r as crate::Wave,
                    ef.attack_c as crate::Wave,
                    ef.release_r as crate::Wave,
                    ef.release_c as crate::Wave,
                    ef.sensitivity_r as crate::Wave,
                    sample_rate as crate::Wave,
                );
                // Detector tap (audit gap G1 machinery): feed-forward taps
                // read interior/input nodes, feedback taps the chain output.
                let tap = super::bind::resolve_envelope_tap(pedal, &compiled.stages, &comp.id);
                compiled.envelopes.push(EnvelopeBinding {
                    envelope,
                    target: ModulationTarget::VcaCv { vca_idx },
                    bias: sink.bias as crate::Wave,
                    range: sink.range as crate::Wave,
                    env_id: comp.id.clone(),
                    tap,
                });
            }
        }
    }
}
