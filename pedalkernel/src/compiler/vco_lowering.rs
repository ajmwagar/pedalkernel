//! VCO lowering: turn `vco()` components into runtime `VcoBinding`s — the
//! proof case for the generalized [`DspBlock`] (spec §8,
//! reports/dsp-block-generalization-2026-06-13.md).
//!
//! The `Vco` component is `GraphRole::Virtual` (no MNA stamp). Unlike `bbd()`
//! and `vca()` — 1-in/1-out *inserts* that bridge a galvanic gap — a VCO is a
//! **generator**: it has **zero audio inputs** and one-or-more audio outputs
//! (saw/tri/pulse). It is the first block to exercise:
//!
//! 1. the **N=0 source-placement branch** (spec §4): with no input port there
//!    is no gap to split; the block is a source seeded at the head of its
//!    output node's flow group, injecting its sample into `node_signals` like
//!    an input-port voltage source (processor.rs tick loop), where downstream
//!    stages read it.
//! 2. the **audio-rate CV-port path** (spec §3): the runtime reads the voltage
//!    at the VCO's `cv`/`cv_pitch` node each sample and maps it to frequency at
//!    1 V/oct via `Vco::set_cv_pitch` — a float update to `phase_inc`, **no
//!    scattering recompute**. When no CV pin is wired the VCO runs at its
//!    declared base frequency.
//!
//! The PolyBLEP `Vco` element (pedalkernel-rt `elements::synth`) is the body;
//! this module only *wires it in*. As with `bbd()`/`vca()`, `vcos[i]`
//! corresponds to `vco_component_ids(pedal)[i]` (declaration order = the index
//! contract), and a `vco()` whose wave output cannot be placed in the audio
//! path is a **compile error** naming the component (architecture debt §4 —
//! never ship a silently-bypassed circuit).

use crate::dsl::{PedalDef, Pin, VcoWaveformDsl};

use super::compiled::CompiledPedal;
use super::components::Vco;
use super::dsp_block::{behavioral_island_error, BlockIo, BlockPort, DspBlock, PortRole};
use super::graph::CircuitGraph;
use pedalkernel_rt::elements::VcoWaveform;
use pedalkernel_rt::processor::VcoBinding;

/// Wave-output pins a `vco()` can drive (declaration-order preference per
/// waveform is resolved from the component's declared default waveform).
const VCO_SAW_PINS: &[&str] = &["saw"];
const VCO_TRI_PINS: &[&str] = &["tri", "triangle"];
const VCO_PULSE_PINS: &[&str] = &["pulse", "square"];
/// 1 V/oct pitch CV input pins.
const VCO_CV_PINS: &[&str] = &["cv", "cv_pitch"];
/// Hard-sync input pins (metadata only this pass — `Vco` has no sync port yet).
const VCO_SYNC_PINS: &[&str] = &["sync"];

/// The VCO [`DspBlock`]: lowers `vco()` components to runtime `VcoBinding`s
/// (`compiled.vcos`). A generator (0 audio inputs) — the N=0 source branch.
pub(super) struct VcoBlock;

impl DspBlock for VcoBlock {
    fn handles(&self, type_tag: &str) -> bool {
        type_tag == "VCO"
    }

    fn component_ids(&self, pedal: &PedalDef) -> Vec<String> {
        vco_component_ids(pedal)
    }

    fn io(&self, pedal: &PedalDef, graph: &CircuitGraph) -> Vec<(String, BlockIo)> {
        vco_io(pedal, graph)
    }

    fn bind_runtime(
        &self,
        pedal: &PedalDef,
        compiled: &mut CompiledPedal,
        sample_rate: f64,
    ) -> Result<(), String> {
        compiled.vcos = lower_vcos(pedal, &compiled_graph_node_lookup(pedal), sample_rate)?;
        Ok(())
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Component enumeration
// ═══════════════════════════════════════════════════════════════════════════

/// All `vco()` component IDs in declaration order. `vcos[i]` corresponds to
/// `vco_component_ids(pedal)[i]`.
pub(super) fn vco_component_ids(pedal: &PedalDef) -> Vec<String> {
    pedal
        .components
        .iter()
        .filter(|c| c.kind.as_any().downcast_ref::<Vco>().is_some())
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

// ═══════════════════════════════════════════════════════════════════════════
// Port signature (spec §2/§4) — a generator: 0 inputs, ≥1 audio output
// ═══════════════════════════════════════════════════════════════════════════

/// Resolve `id.pin` (trying each candidate pin name) to a graph node.
fn resolve_port(
    graph: &CircuitGraph,
    id: &str,
    pins: &[&'static str],
    role: PortRole,
) -> Option<BlockPort> {
    for &pin in pins {
        if let Some(&node) = graph.node_names.get(&format!("{id}.{pin}")) {
            return Some(BlockPort { node, pin, role });
        }
    }
    None
}

/// Pick the wave-output pins for a VCO's declared default waveform.
fn waveform_pins(wf: VcoWaveformDsl) -> &'static [&'static str] {
    match wf {
        VcoWaveformDsl::Saw => VCO_SAW_PINS,
        VcoWaveformDsl::Triangle => VCO_TRI_PINS,
        VcoWaveformDsl::Pulse => VCO_PULSE_PINS,
    }
}

/// Typed port signature (spec §2/§4) of each `vco()`: a **generator** —
/// `inputs` are the CV/sync pins present in the netlist (PortRole::Cv / Sync),
/// possibly empty (CV inputs do NOT count as a galvanic gap to bridge);
/// `outputs` are the wave-output pins wired in the netlist (PortRole::Audio),
/// preferring the component's declared default waveform but listing every wired
/// wave output so each becomes a SPQR terminal.
///
/// `all_boundary_nodes` flattens inputs-then-outputs to recover the behavioral
/// stage boundaries: each wired wave-output node must be a SPQR terminal so the
/// downstream passive group has a port there. A VCO with `inputs.is_empty()`
/// (no CV/sync wired) is the pure-source N=0 case the placement branch handles.
pub(super) fn vco_io(pedal: &PedalDef, graph: &CircuitGraph) -> Vec<(String, BlockIo)> {
    vco_component_ids(pedal)
        .into_iter()
        .map(|id| {
            let mut inputs = Vec::new();
            if let Some(p) = resolve_port(graph, &id, VCO_CV_PINS, PortRole::Cv) {
                inputs.push(p);
            }
            if let Some(p) = resolve_port(graph, &id, VCO_SYNC_PINS, PortRole::Sync) {
                inputs.push(p);
            }

            // Outputs: every wired wave-output pin, default waveform first.
            let wf = pedal
                .components
                .iter()
                .find(|c| c.id == id)
                .and_then(|c| c.kind.as_any().downcast_ref::<Vco>())
                .map(|v| v.waveform)
                .unwrap_or(VcoWaveformDsl::Saw);
            let mut outputs = Vec::new();
            let mut push_out = |pins: &[&'static str], outputs: &mut Vec<BlockPort>| {
                if let Some(p) = resolve_port(graph, &id, pins, PortRole::Audio) {
                    if !outputs.iter().any(|o: &BlockPort| o.node == p.node) {
                        outputs.push(p);
                    }
                }
            };
            push_out(waveform_pins(wf), &mut outputs);
            for pins in [VCO_SAW_PINS, VCO_TRI_PINS, VCO_PULSE_PINS] {
                push_out(pins, &mut outputs);
            }

            (id, BlockIo { inputs, outputs })
        })
        .collect()
}

// ═══════════════════════════════════════════════════════════════════════════
// Runtime VcoBinding construction
// ═══════════════════════════════════════════════════════════════════════════

/// Reconstruct the graph (node_names) the same way `from_pedal` does, so the
/// runtime-binding pass can resolve VCO output/cv pins to node ids. The
/// `bind_runtime_all` driver does not pass the graph through, so we rebuild it
/// here (cheap, once per compile, and matches how `bbd_io`/`vca_io` resolve
/// node names from the same `CircuitGraph::from_pedal`).
fn compiled_graph_node_lookup(pedal: &PedalDef) -> CircuitGraph {
    CircuitGraph::from_pedal(pedal)
}

/// Map the DSL waveform to the runtime `VcoWaveform`.
fn rt_waveform(wf: VcoWaveformDsl) -> VcoWaveform {
    match wf {
        VcoWaveformDsl::Saw => VcoWaveform::Saw,
        VcoWaveformDsl::Triangle => VcoWaveform::Triangle,
        VcoWaveformDsl::Pulse => VcoWaveform::Pulse,
    }
}

/// Build one runtime `VcoBinding` per `vco()` component (declaration order).
///
/// Errors (never ship silent — architecture debt §4): a `vco()` with no wired
/// wave output (saw/tri/pulse) cannot be placed as a source in the audio path,
/// so it would silently produce nothing — a compile error naming the component.
pub(super) fn lower_vcos(
    pedal: &PedalDef,
    graph: &CircuitGraph,
    sample_rate: f64,
) -> Result<Vec<VcoBinding>, String> {
    let mut vcos = Vec::new();
    for comp in &pedal.components {
        let Some(vco) = comp.kind.as_any().downcast_ref::<Vco>() else {
            continue;
        };

        // A VCO must have at least one wave output wired into the netlist;
        // otherwise it is an unplaceable behavioral island (§4).
        let any_out_wired = pin_wired(pedal, &comp.id, VCO_SAW_PINS)
            || pin_wired(pedal, &comp.id, VCO_TRI_PINS)
            || pin_wired(pedal, &comp.id, VCO_PULSE_PINS);
        if !any_out_wired {
            return Err(behavioral_island_error(
                &comp.id,
                "vco()",
                "no wave output (saw/tri/pulse) is wired into the netlist, so the \
                 oscillator cannot be placed as a source and would silently produce \
                 nothing",
            ));
        }

        // Output node: the declared default waveform's pin, falling back to any
        // wired wave output (so a `tri`-only patch on a default-saw VCO still
        // injects somewhere live).
        let output_node_id = resolve_port(
            graph,
            &comp.id,
            waveform_pins(vco.waveform),
            PortRole::Audio,
        )
        .or_else(|| resolve_port(graph, &comp.id, VCO_SAW_PINS, PortRole::Audio))
        .or_else(|| resolve_port(graph, &comp.id, VCO_TRI_PINS, PortRole::Audio))
        .or_else(|| resolve_port(graph, &comp.id, VCO_PULSE_PINS, PortRole::Audio))
        .map(|p| p.node)
        .unwrap_or(usize::MAX);

        // 1 V/oct pitch CV node (read each sample by the runtime). `usize::MAX`
        // when no CV pin is wired — the VCO then holds its base frequency.
        let cv_node_id = resolve_port(graph, &comp.id, VCO_CV_PINS, PortRole::Cv)
            .map(|p| p.node)
            .unwrap_or(usize::MAX);

        // The cem3340/as3340/v3340 model variants share the PolyBLEP body
        // (1 V/oct, saw/tri/pulse); the variant is metadata for layout/BOM.
        let mut rt_vco = pedalkernel_rt::elements::Vco::new(sample_rate);
        // Base frequency = the vco()'s declared frequency; CV maps relative to
        // it at 1 V/oct (cv=0 -> base_freq, +1 V -> octave up).
        rt_vco.set_base_freq(vco.base_freq);

        vcos.push(VcoBinding {
            vco: rt_vco,
            waveform: rt_waveform(vco.waveform),
            output_node_id,
            cv_node_id,
            comp_id: comp.id.clone(),
        });
    }
    Ok(vcos)
}
