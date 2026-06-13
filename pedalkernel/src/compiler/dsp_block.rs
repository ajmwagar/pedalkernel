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
use super::graph::CircuitGraph;
use super::signal_flow::FlowGroup;

/// A behavioral island: a `GraphRole::Virtual` component that lowers to a
/// per-instance runtime DSP block (delay line, VCA, ...) bridged across a
/// galvanic gap in the netlist, instead of stamping into the WDF/MNA core.
pub(super) trait DspBlock {
    /// `type_tag` this block handles (e.g. `"BBD delay"`, `"VCA"`). Matched
    /// against `Component::type_tag()` by the mandatory-lowering gate.
    fn handles(&self, type_tag: &str) -> bool;

    /// Component ids of this kind in declaration order (the runtime index
    /// contract — `runtime_vec[i]` corresponds to `component_ids(pedal)[i]`).
    fn component_ids(&self, pedal: &PedalDef) -> Vec<String>;

    /// Boundary node ids (the Virtual audio pins) to add as SPQR terminals and
    /// to split flow groups at. Empty when this block has no components.
    fn boundary_nodes(&self, pedal: &PedalDef, graph: &CircuitGraph) -> Vec<usize>;

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
pub(super) fn all_boundary_nodes(pedal: &PedalDef, graph: &CircuitGraph) -> Vec<usize> {
    let mut nodes = Vec::new();
    for block in dsp_blocks() {
        for n in block.boundary_nodes(pedal, graph) {
            if !nodes.contains(&n) {
                nodes.push(n);
            }
        }
    }
    nodes
}

/// Split flow groups into galvanically-connected clusters at every registered
/// block's behavioral gap. See [`split_groups_at_behavioral_gaps`] for the
/// union-find rationale (rails are deliberately ignored).
///
/// Only run when [`any_block_has_components`] is true (no gap otherwise).
pub(super) fn split_groups_at_behavioral_gaps(groups: &mut Vec<FlowGroup>, graph: &CircuitGraph) {
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
