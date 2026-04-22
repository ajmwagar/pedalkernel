//! Control binding for the SPQR pipeline.
//!
//! Pots are components. Each stage knows how to handle its own pot changes.
//! The control module just routes: find stage → tell it the new position.
//!
//! Smoothing spreads parameter changes over ~32 samples to avoid zipper
//! noise and CPU spikes from recomputation.

use super::compiled::{CompiledPedal, ControlBinding, ControlTarget, Stage};
use crate::dsl::{PedalDef, PotTaper};

/// Bind control declarations to compiled stages.
///
/// For each pot control in the .pedal file, finds which stage contains
/// that pot and creates a binding. The stage handles recomputation
/// internally — WDF trees recompute impedances, OpAmpRoot recalcs
/// gain, IIR recalcs coefficients.
pub(super) fn bind_controls(pedal: &PedalDef, compiled: &mut CompiledPedal) {
    for ctrl in &pedal.controls {
        if let Some(binding) = find_pot_binding(ctrl, pedal, compiled) {
            compiled.controls.push(binding);
        }
    }

    // Apply defaults
    for ctrl in &pedal.controls {
        compiled.set_control(&ctrl.label, ctrl.default);
    }
}

/// Find which stage owns a pot and create the binding.
///
/// Searches all stage types. The pot may be:
/// - A WdfPot leaf in a WdfStage tree (tree.set_pot works)
/// - Consumed by OpAmpRoot at compile time (needs gain recompute)
/// - In a MultiNlStage (delta-update scattering matrix)
fn find_pot_binding(
    ctrl: &crate::dsl::ControlDef,
    pedal: &PedalDef,
    compiled: &CompiledPedal,
) -> Option<ControlBinding> {
    let comp_id = &ctrl.component;

    // Get pot metadata
    let pot_comp = pedal.components.iter().find(|c| c.id == *comp_id)?;
    let (max_r, taper) = pot_comp
        .kind
        .as_any()
        .downcast_ref::<super::components::Potentiometer>()
        .map(|p| (p.max_r, p.taper))
        .unwrap_or((100_000.0, PotTaper::B));

    let aw_id = format!("{comp_id}__aw");
    let wb_id = format!("{comp_id}__wb");

    // Search all stages in the unified vec
    for (idx, stage) in compiled.stages.iter().enumerate() {
        match stage {
            Stage::Wdf(wdf) => {
                let in_tree = wdf.tree.get_pot_position(comp_id).is_some()
                    || wdf.tree.get_pot_position(&aw_id).is_some()
                    || wdf.tree.get_pot_position(&wb_id).is_some();
                let in_children = wdf.opamp_children.iter().any(|c| {
                    c.get_pot_position(comp_id).is_some()
                        || c.get_pot_position(&aw_id).is_some()
                        || c.get_pot_position(&wb_id).is_some()
                });
                let is_feedback_pot = wdf.feedback_pot_id.as_deref() == Some(comp_id);

                if in_tree || in_children || is_feedback_pot {
                    return Some(make_binding(
                        ctrl,
                        comp_id,
                        &aw_id,
                        &wb_id,
                        max_r,
                        taper,
                        ControlTarget::PotInStage(idx),
                    ));
                }
            }
            Stage::MultiNl(mnl) => {
                for (pi, child) in mnl.passive_children.iter().enumerate() {
                    if child.get_pot_position(comp_id).is_some()
                        || child.get_pot_position(&aw_id).is_some()
                        || child.get_pot_position(&wb_id).is_some()
                    {
                        return Some(make_binding(
                            ctrl,
                            comp_id,
                            &aw_id,
                            &wb_id,
                            max_r,
                            taper,
                            ControlTarget::PotInMultiNlStage(idx, pi),
                        ));
                    }
                }
                for child in &mnl.pot_children {
                    if child.get_pot_position(comp_id).is_some()
                        || child.get_pot_position(&aw_id).is_some()
                        || child.get_pot_position(&wb_id).is_some()
                    {
                        return Some(make_binding(
                            ctrl,
                            comp_id,
                            &aw_id,
                            &wb_id,
                            max_r,
                            taper,
                            ControlTarget::PotInMultiNlStage(idx, 0),
                        ));
                    }
                }
            }
            Stage::Iir(iir) => {
                if iir.has_pot(comp_id) {
                    return Some(make_binding(
                        ctrl,
                        comp_id,
                        &aw_id,
                        &wb_id,
                        max_r,
                        taper,
                        ControlTarget::PotInIirStage(idx),
                    ));
                }
            }
            Stage::BlackFeedback(bf) => {
                if bf.has_pot(comp_id) {
                    return Some(make_binding(
                        ctrl,
                        comp_id,
                        &aw_id,
                        &wb_id,
                        max_r,
                        taper,
                        ControlTarget::PotInBlackFeedbackStage(idx),
                    ));
                }
            }
            Stage::StateSpace(_) => {
                // TODO: StateSpaceStage G-matrix delta
            }
        }
    }

    None
}

fn make_binding(
    ctrl: &crate::dsl::ControlDef,
    comp_id: &str,
    aw_id: &str,
    wb_id: &str,
    max_r: f64,
    taper: PotTaper,
    target: ControlTarget,
) -> ControlBinding {
    ControlBinding {
        label: ctrl.label.clone(),
        target,
        component_id: comp_id.to_string(),
        component_id_aw: aw_id.to_string(),
        component_id_wb: wb_id.to_string(),
        max_resistance: max_r,
        taper,
        range: ctrl.range,
    }
}
