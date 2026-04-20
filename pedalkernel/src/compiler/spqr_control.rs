//! Control binding for the SPQR pipeline.
//!
//! Maps .pedal `controls {}` declarations to runtime parameter targets.
//! One path per control type. Component ID lookup, no special cases.
//!
//! Current: pots.
//! Future: CV inputs, switches, buttons, MIDI CC.

use super::compiled::{CompiledPedal, ControlBinding, ControlTarget};
use crate::dsl::{PedalDef, PotTaper};

/// Bind control declarations from the .pedal file to compiled stage parameters.
///
/// For each control in `pedal.controls`, finds the matching component
/// in the compiled stages and creates a runtime binding.
pub(super) fn bind_controls(pedal: &PedalDef, compiled: &mut CompiledPedal) {
    for ctrl in &pedal.controls {
        if let Some(binding) = bind_pot(ctrl, pedal, compiled) {
            compiled.controls.push(binding);
        }
    }

    // Apply default positions
    for ctrl in &pedal.controls {
        compiled.set_control(&ctrl.label, ctrl.default);
    }
}

/// Try to bind a control declaration as a pot.
///
/// Searches WDF stages for a pot leaf matching the component ID.
fn bind_pot(
    ctrl: &crate::dsl::ControlDef,
    pedal: &PedalDef,
    compiled: &CompiledPedal,
) -> Option<ControlBinding> {
    let comp_id = &ctrl.component;

    // Get pot metadata (taper, max_r) from the PedalDef component list
    let pot_comp = pedal.components.iter().find(|c| c.id == *comp_id)?;
    let (max_r, taper) = pot_comp
        .kind
        .as_any()
        .downcast_ref::<super::components::Potentiometer>()
        .map(|p| (p.max_r, p.taper))
        .unwrap_or((100_000.0, PotTaper::B));

    // Search WDF stages for the pot leaf (synthetic names: {id}__aw, {id}__wb)
    for (stage_idx, stage) in compiled.stages.iter().enumerate() {
        let aw_id = format!("{comp_id}__aw");
        let wb_id = format!("{comp_id}__wb");
        let has_pot = stage.tree.get_pot_position(comp_id).is_some()
            || stage.tree.get_pot_position(&aw_id).is_some()
            || stage.tree.get_pot_position(&wb_id).is_some();
        if has_pot {
            return Some(ControlBinding {
                label: ctrl.label.clone(),
                target: ControlTarget::PotInStage(stage_idx),
                component_id: comp_id.clone(),
                component_id_aw: aw_id,
                component_id_wb: wb_id,
                max_resistance: max_r,
                taper,
                range: ctrl.range,
            });
        }
    }

    // TODO: search IIR/StateSpace/MultiNl stages for pots
    // TODO: handle non-pot controls (switches, LFO rate, etc.)

    None
}
