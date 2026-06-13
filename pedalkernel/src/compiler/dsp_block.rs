//! Behavioral-island lowering, generalized (architecture debt §4).
//!
//! A **DSP block** is a `GraphRole::Virtual` component that lowers to a
//! per-instance *runtime* DSP block (a `BbdDelayLine`, a `VcaBinding`, and —
//! per the roadmap — a spring reverb or a delay line) bridged across a
//! galvanic gap in the netlist, instead of being stamped into the WDF/MNA
//! core. Such a component contributes no electrical edges (`StampResult::Skip`
//! + a `Behavioral` edge), so the circuit graph is galvanically cut at its
//! audio pins; the surrounding passives compile into separate serial stages
//! and the runtime block bridges the gap.
//!
//! Before this module the lowering machinery was hand-written once per block
//! kind (`bbd_lowering`, `vca_lowering`) and wired by hand at ~6 call sites in
//! `spqr_build`. The [`DspBlock`] trait + the [`dsp_blocks`] registry collapse
//! that into one abstraction: the registry is the single place a new block is
//! declared, which makes the mandatory-lowering gate
//! ([`reject_unlowered_behavioral`]) impossible to forget.
//!
//! # Adding a new DSP block
//!
//! This is the contract the spring-reverb and delay-line work implement:
//!
//! 1. Add a runtime instance type + a `Vec<_>` field on `CompiledPedal`
//!    (the per-instance index contract).
//! 2. Write the type-specific guts (build the runtime instances, the boundary
//!    math, control/modulation binding) in a sibling module, e.g.
//!    `spring_lowering.rs` — exactly as `bbd_lowering`/`vca_lowering` do.
//! 3. `impl DspBlock for SpringBlock`, delegating to those guts.
//! 4. Add **one line** to [`dsp_blocks`]: `&SpringBlock`.
//!
//! That single registry line wires the block into group splitting, terminal
//! injection, runtime binding, **and** the mandatory-lowering gate — there is
//! no separate spot to forget.

use crate::dsl::PedalDef;

use super::compiled::CompiledPedal;
use super::graph::{CircuitGraph, NodeId};
use super::signal_flow::FlowGroup;

// ═══════════════════════════════════════════════════════════════════════════
// Port signature (spec §2) — the typed shape a block declares.
// ═══════════════════════════════════════════════════════════════════════════

/// What a behavioral-block port carries. Metadata for the binder/UI/validation
/// (e.g. a `Cv` input may be 1 V/oct-scaled); the runtime treats every input
/// as "read node voltage" and every output as "write node voltage". Closed
/// enum, mirroring `EdgeKind` / `ModulationSinkKind` at component.rs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum PortRole {
    /// Audio-band signal (the serial in/out of an insert).
    Audio,
    /// Control voltage (e.g. 1 V/oct pitch, VCA gain CV).
    Cv,
    /// Gate / trigger level.
    Gate,
    /// Clock / sample-rate tick (e.g. BBD bucket clock).
    Clock,
    /// Hard-sync edge.
    Sync,
}

/// One node a behavioral block connects to, with the component pin it maps to
/// and the role of the signal it carries.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct BlockPort {
    /// Graph node whose voltage the block reads (input) or drives (output).
    pub node: NodeId,
    /// Component pin this maps to (`.in`/`.out`/`.cv`/`.clock`/`.saw`…).
    pub pin: &'static str,
    /// Role of the signal carried, for binder/UI/validation.
    pub role: PortRole,
}

/// How a behavioral block connects to the surrounding circuit (spec §2). A
/// block declares this typed signature; the lowering machinery consumes it
/// generically.
#[derive(Debug, Clone, Default)]
pub(super) struct BlockIo {
    /// Nodes whose voltage the block READS each sample. Empty ⇒ generator
    /// (N=0 source, e.g. VCO), placed at the head of its output flow group.
    pub inputs: Vec<BlockPort>,
    /// Nodes the block DRIVES (as a voltage source) each sample. Empty ⇒ sink.
    pub outputs: Vec<BlockPort>,
}

/// A behavioral island: a `GraphRole::Virtual` component that lowers to a
/// per-instance runtime DSP block (delay line, VCA, ...) bridged across a
/// galvanic gap in the netlist, instead of stamping into the WDF/MNA core.
///
/// # Cost model (spec §3) — the law that makes the generalization safe
///
/// A `DspBlock` **never participates in the WDF/MNA scattering solve.** Its
/// input ports are node *reads* (`node_signals[n]`); its output ports are node
/// *writes* (`b = 2v − a`, the existing port-injection path,
/// processor.rs:1334/2862). Neither grows nor re-derives any per-sample matrix.
/// Therefore a block may have **any number of ports** and **audio-rate internal
/// controls** at constant per-sample cost. The matrix-cost rule (re-derive the
/// scattering solve) applies *only* to controls that modulate a WDF-stamped
/// element (pot wiper, JFET Rds, opto R_ldr) — which a block, by definition, is
/// not.
pub(super) trait DspBlock {
    /// `type_tag` this block handles (e.g. `"BBD delay"`, `"VCA"`). Matched
    /// against `Component::type_tag()` by the mandatory-lowering gate.
    fn handles(&self, type_tag: &str) -> bool;

    /// Component ids of this kind in declaration order (the runtime index
    /// contract — `runtime_vec[i]` corresponds to `component_ids(pedal)[i]`).
    fn component_ids(&self, pedal: &PedalDef) -> Vec<String>;

    /// The typed port signature (spec §2) of each instance of this block, in
    /// component declaration order. Replaces the old flat `boundary_nodes`:
    /// `all_boundary_nodes` flattens every block's input+output port nodes to
    /// recover the same set of SPQR terminals / gap-split boundaries, now
    /// sourced from structure. An instance with `inputs.is_empty()` is a
    /// generator (N=0 source). Empty `Vec` when this block has no components.
    fn io(&self, pedal: &PedalDef, graph: &CircuitGraph) -> Vec<(String, BlockIo)>;

    /// Lower + bind this kind's runtime instances into the compiled pedal,
    /// writing its own runtime field (`compiled.bbds`, `compiled.vcas`, ...).
    ///
    /// Fallible: a block whose audio pins cannot bridge the gap is a compile
    /// error (never ship a silently-bypassed circuit — see §4).
    fn bind_runtime(
        &self,
        pedal: &PedalDef,
        compiled: &mut CompiledPedal,
        sample_rate: f64,
    ) -> Result<(), String>;

    /// True when this block has at least one component in `pedal`.
    fn has_components(&self, pedal: &PedalDef) -> bool {
        !self.component_ids(pedal).is_empty()
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Registry — the single place a DSP block is declared.
// ═══════════════════════════════════════════════════════════════════════════

/// The registered DSP blocks. Adding a block = one impl + one line here; that
/// line is what wires it into group splitting, terminal injection, runtime
/// binding, and the mandatory-lowering gate at once.
pub(super) fn dsp_blocks() -> &'static [&'static dyn DspBlock] {
    &[
        &super::bbd_lowering::BbdBlock,
        &super::vca_lowering::VcaBlock,
    ]
}

/// True when any registered block has components in `pedal` — the predicate
/// that gates group splitting and terminal injection (a behavioral gap only
/// exists if some block is present).
pub(super) fn any_block_has_components(pedal: &PedalDef) -> bool {
    dsp_blocks().iter().any(|b| b.has_components(pedal))
}

// ═══════════════════════════════════════════════════════════════════════════
// Mandatory-lowering gate (architecture debt §4)
// ═══════════════════════════════════════════════════════════════════════════

/// Compile error for a behavioral island that did not lower to a runtime
/// element. Generic over component kind so every block kind shares the same
/// contract wording.
pub(super) fn behavioral_island_error(comp_id: &str, type_tag: &str, reason: &str) -> String {
    format!(
        "{comp_id}: {type_tag} is a behavioral island that did not lower to a runtime \
         element ({reason}). Behavioral components must lower or fail to compile — \
         refusing to ship a silently-bypassed circuit \
         (reports/architecture-debt-2026-06-12.md §4)."
    )
}

/// The `type_tag`s of behavioral islands that lower to a runtime DSP block (and
/// therefore *require* a registered [`DspBlock`]). A component with one of
/// these tags but no registered block would compile to a silent island — the
/// §4 violation this gate exists to stop.
///
/// `vco()` is the only one without a block today: there is no VCO lowering, so
/// `cem3340_vco`/`minisynth` previously compiled silent. Delete its tag here
/// when a `VcoBlock` lands (one impl + one `dsp_blocks` line).
const RUNTIME_DSP_ISLAND_TAGS: &[(&str, &str)] = &[
    // (Component::type_tag, the spelling used in the error message)
    ("BBD delay", "bbd()"),
    ("VCA", "vca()"),
    ("VCO", "vco()"),
];

/// Reject behavioral islands that have NO registered [`DspBlock`] to lower
/// them — a `GraphRole::Virtual` component whose `type_tag` is a known
/// runtime-DSP island but which no registered block `handles()`.
///
/// Registry-driven: the gate cannot be forgotten when adding a block, and a
/// new runtime-DSP island that ships without a block fails to compile naming
/// the component (currently: `vco()`).
pub(super) fn reject_unlowered_behavioral(pedal: &PedalDef) -> Result<(), String> {
    let blocks = dsp_blocks();
    for comp in &pedal.components {
        let tag = comp.kind.type_tag();
        let Some((_, spelling)) = RUNTIME_DSP_ISLAND_TAGS.iter().find(|(t, _)| *t == tag) else {
            continue;
        };
        if blocks.iter().any(|b| b.handles(tag)) {
            continue;
        }
        return Err(behavioral_island_error(
            &comp.id,
            spelling,
            "VCO lowering is not implemented yet — the oscillator would silently \
             produce nothing",
        ));
    }
    Ok(())
}

// ═══════════════════════════════════════════════════════════════════════════
// Generalized passes — one loop over the registry, replacing the per-block
// hand-wiring in spqr_build.
// ═══════════════════════════════════════════════════════════════════════════

/// Boundary node ids gathered from ALL registered blocks (deduplicated,
/// declaration order). These are behavioral stage boundaries: the netlist is
/// galvanically cut there, so each must be a SPQR terminal — otherwise the
/// dangling side of a passive group has no port and its stage probes 0.
///
/// Sourced from each block's structured [`BlockIo`] signature: flatten every
/// instance's input port nodes then output port nodes (declaration order),
/// deduplicating. For the 1-in/1-out inserts that exist today (BBD, VCA) this
/// yields exactly `[in_node, out_node]` per instance — the identical node set
/// the old flat `boundary_nodes` produced.
pub(super) fn all_boundary_nodes(pedal: &PedalDef, graph: &CircuitGraph) -> Vec<usize> {
    let mut nodes = Vec::new();
    for block in dsp_blocks() {
        for (_id, io) in block.io(pedal, graph) {
            for port in io.inputs.iter().chain(io.outputs.iter()) {
                if !nodes.contains(&port.node) {
                    nodes.push(port.node);
                }
            }
        }
    }
    nodes
}

/// Split flow groups into galvanically-connected clusters at every registered
/// block's behavioral gap. See [`split_groups_at_behavioral_gaps`] for the
/// union-find rationale (rails are deliberately ignored).
///
/// Placement by input-port count (spec §4):
///
/// - **N ≥ 1 inputs** → gap-bridge: the union-find split below cuts each group
///   that spans the block's galvanic gap into one serial stage per side. This
///   is the only case the inserts that exist today (BBD, VCA, both 1-in/1-out)
///   exercise.
/// - **N = 0 (generator: VCO)** → there is no input side and thus no gap to
///   split; the block is a source placed at the head of the flow group its
///   output node belongs to, reusing the port-write path. The branch below is
///   written so a generator drops in without new machinery, but is
///   **unexercised this pass** — every registered block is 1-in/1-out, so the
///   `inputs.is_empty()` guard never fires. Exercised by VCO (spec §8).
///
/// Only run when [`any_block_has_components`] is true (no gap otherwise).
pub(super) fn split_groups_at_behavioral_gaps(
    groups: &mut Vec<FlowGroup>,
    graph: &CircuitGraph,
    pedal: &PedalDef,
) {
    // Generators (N=0 inputs) have no galvanic gap to split: their output node
    // is a source seed, not a cut. A 1-in/1-out insert always has a non-empty
    // `inputs`, so this guard is dead-but-correct until VCO lands (spec §8).
    let has_generator = dsp_blocks().iter().any(|block| {
        block
            .io(pedal, graph)
            .iter()
            .any(|(_id, io)| io.inputs.is_empty() && !io.outputs.is_empty())
    });
    if has_generator {
        // A generator seeds the head of its output node's flow group rather
        // than cutting one — no union-find split is needed for its side.
        // Placement reuses the existing port-write path (spec §3/§4); there is
        // no additional group restructuring to perform here for the N=0 case.
        // exercised by VCO (spec §8)
    }

    // Gap-split every N≥1 insert (the only case present today).
    super::bbd_lowering::split_groups_at_behavioral_gaps(groups, graph);
}

/// Lower + bind every registered block's runtime instances. One loop replacing
/// the paired `bind_bbd_runtime` + `bind_vca_runtime` at each construction
/// site. Fallible: an island that cannot bridge its gap aborts compilation.
pub(super) fn bind_runtime_all(
    pedal: &PedalDef,
    compiled: &mut CompiledPedal,
    sample_rate: f64,
) -> Result<(), String> {
    for block in dsp_blocks() {
        block.bind_runtime(pedal, compiled, sample_rate)?;
    }
    Ok(())
}
