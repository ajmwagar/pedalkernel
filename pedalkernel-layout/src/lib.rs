//! Automatic schematic layout engine for PedalKernel `.pedal` circuits.
//!
//! Converts a parsed [`pedalkernel::dsl::PedalDef`] into a positioned layout
//! suitable for:
//!
//! 1. **Plugin UI** — real-time signal flow visualization with monitor-driven glow
//! 2. **KiCad export** — properly placed `.kicad_sch` files for PCB design
//!
//! # Pipeline
//!
//! ```text
//! PedalDef
//!   → LayoutGraph        (directed graph with pin-direction inference)
//!   → FunctionalGroups   (gain stages, tone stacks, push-pull, etc.)
//!   → Column assignment  (Sugiyama layering for left-to-right signal flow)
//!   → Vertical placement (supply → signal path → ground convention)
//!   → Wire routing       (orthogonal Manhattan paths)
//!   → Optimization       (crossing minimization, grid snap, symmetry)
//!   → Layout             (JSON-serializable output)
//! ```

pub mod graph;
pub mod groups;
pub mod layering;
pub mod placement;
pub mod routing;
pub mod optimize;
pub mod symbols;
pub mod types;
pub mod kicad;

use pedalkernel::dsl::PedalDef;
use types::Layout;

/// Generate a complete schematic layout from a parsed pedal definition.
///
/// This is the main entry point. It runs all six layout phases and returns
/// a [`Layout`] that can be serialized to `.pedal_layout` JSON or exported
/// to KiCad `.kicad_sch` format.
pub fn generate_layout(pedal: &PedalDef, width: f32, height: f32) -> Layout {
    // Phase 1: Build directed graph from netlist
    let lg = graph::LayoutGraph::from_pedal(pedal);

    // Phase 2: Detect functional groups
    let groups = groups::detect_groups(&lg);

    // Phase 3: Assign columns via Sugiyama layering
    let columns = layering::assign_columns(&lg, &groups);

    // Phase 4: Vertical placement within columns
    let mut layout = placement::place_components(&lg, &groups, &columns, width, height);

    // Phase 5: Route wires
    routing::route_wires(&mut layout, &lg);

    // Phase 6: Aesthetic optimization
    optimize::optimize_layout(&mut layout);

    layout
}

/// Serialize a layout to `.pedal_layout` JSON format.
pub fn to_json(layout: &Layout) -> String {
    serde_json::to_string_pretty(layout).expect("layout serialization should not fail")
}

/// Export a layout to KiCad `.kicad_sch` schematic format.
pub fn to_kicad_schematic(layout: &Layout, pedal: &PedalDef) -> String {
    kicad::export_kicad_schematic(layout, pedal)
}
