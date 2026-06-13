//! Control binding for the SPQR pipeline.
//!
//! Pots are components. Each stage knows how to handle its own pot changes.
//! The control module just routes: find stage → tell it the new position.
//!
//! Smoothing spreads parameter changes over ~32 samples to avoid zipper
//! noise and CPU spikes from recomputation.

use super::compiled::{CompiledPedal, ControlBinding, ControlTarget};
use crate::dsl::{PedalDef, PotTaper};
use std::collections::BTreeMap;

/// Bind control declarations to compiled stages.
///
/// For each pot control in the .pedal file, finds which stage contains
/// that pot and creates a binding. The stage handles recomputation
/// internally — WDF trees recompute impedances, OpAmpRoot recalcs
/// gain, IIR recalcs coefficients.
pub(super) fn bind_controls(pedal: &PedalDef, compiled: &mut CompiledPedal) {
    for ctrl in &pedal.controls {
        compiled
            .controls
            .extend(find_pot_bindings(ctrl, pedal, compiled));
    }

    // Detect output divider pots and create wiper dividers.
    // A pot is an output divider if:
    // - Its label is "Level", "Volume", "Output", or "Mix" (convention)
    // - It's bound to a WDF stage (PotInStage)
    // The wiper divider applies `signal *= position` between stages,
    // bypassing the WDF tree output probe issue.
    let mut output_dividers = BTreeMap::new();
    for ctrl in &compiled.controls {
        let label_lower = ctrl.label.to_lowercase();
        let is_output_divider = label_lower.contains("level")
            || label_lower.contains("volume")
            || label_lower.contains("output");
        if is_output_divider {
            if pot_wiper_connects_directly_to_out(pedal, &ctrl.component_id) {
                continue;
            }
            if let super::compiled::ControlTarget::PotInStage(stage_idx) = &ctrl.target {
                let default = pedal
                    .controls
                    .iter()
                    .find(|c| c.label == ctrl.label && c.component == ctrl.component_id)
                    .map(|c| c.default)
                    .unwrap_or(0.5);
                let position = ctrl
                    .taper
                    .apply(ctrl.range.0 + default * (ctrl.range.1 - ctrl.range.0));
                output_dividers
                    .entry(ctrl.component_id.clone())
                    .and_modify(|(existing_stage_idx, existing_position, existing_taper)| {
                        if *stage_idx >= *existing_stage_idx {
                            *existing_stage_idx = *stage_idx;
                            *existing_position = position;
                            *existing_taper = ctrl.taper;
                        }
                    })
                    .or_insert((*stage_idx, position, ctrl.taper));
            }
        }
    }
    compiled
        .wiper_dividers
        .extend(output_dividers.into_iter().map(
            |(pot_comp_id, (after_stage_idx, position, taper))| super::compiled::WiperDivider {
                after_stage_idx,
                pot_comp_id,
                position,
                taper,
            },
        ));

    // Create pot smoothers (one per control) for zipper-free updates.
    for (i, binding) in compiled.controls.iter().enumerate() {
        if let Some(ctrl) = pedal
            .controls
            .iter()
            .find(|ctrl| ctrl.label == binding.label && ctrl.component == binding.component_id)
        {
            compiled
                .pot_smoothers
                .push(super::compiled::SmoothedParam::new(
                    ctrl.default,
                    i,
                    compiled.sample_rate,
                ));
        }
    }

    // Apply defaults.
    //
    // Use the *immediate* path, not the smoothed `set_control`. The smoother's
    // `current` is initialized to `ctrl.default` above, so `set_control` here
    // would see `is_settled()` == true and never fire `set_control_pot`, leaving
    // the underlying pot leaf at its compile-time stamp (position 0.5 for
    // PassiveRType children — see spqr_build.rs). That mismatch makes the
    // declared default inert and folds the runtime pot sweep around 0.5
    // (pos-0.5 measures identical to pos-0.0, sub-0.5 positions alias). Pushing
    // the default through `set_control_immediate` writes it straight into the
    // leaf and re-derives the scattering, so the compiled baseline IS the
    // declared default and the sweep is monotonic across the full range.
    for ctrl in &pedal.controls {
        compiled.set_control_immediate(&ctrl.label, ctrl.default);
    }
}

fn pot_wiper_connects_directly_to_out(pedal: &PedalDef, comp_id: &str) -> bool {
    fn is_wiper(pin: &crate::dsl::Pin, comp_id: &str) -> bool {
        matches!(
            pin,
            crate::dsl::Pin::ComponentPin { component, pin }
                if component == comp_id && (pin == "w" || pin == "wiper")
        )
    }

    fn is_out(pin: &crate::dsl::Pin) -> bool {
        matches!(pin, crate::dsl::Pin::Reserved(name) if name == "out")
    }

    pedal.nets.iter().any(|net| {
        (is_wiper(&net.from, comp_id) && net.to.iter().any(is_out))
            || (is_out(&net.from) && net.to.iter().any(|pin| is_wiper(pin, comp_id)))
    })
}

/// Find every stage that owns a pot and create bindings.
///
/// Searches all stage types. The pot may be:
/// - A WdfPot leaf in a WdfStage tree (tree.set_pot works)
/// - Consumed by OpAmpRoot at compile time (needs gain recompute)
/// - In a MultiNlStage (delta-update scattering matrix)
///
/// A split physical pot can appear in multiple passive branches/stages. Binding
/// only the first owner leaves later PassiveRType extraction matrices stale.
fn find_pot_bindings(
    ctrl: &crate::dsl::ControlDef,
    pedal: &PedalDef,
    compiled: &CompiledPedal,
) -> Vec<ControlBinding> {
    let comp_id = &ctrl.component;

    // Get pot metadata
    let Some(pot_comp) = pedal.components.iter().find(|c| c.id == *comp_id) else {
        return Vec::new();
    };
    let (max_r, taper) = pot_comp
        .kind
        .as_any()
        .downcast_ref::<super::components::Potentiometer>()
        .map(|p| (p.max_r, p.taper))
        .unwrap_or((100_000.0, PotTaper::B));

    let mut bindings: Vec<_> = compiled
        .stages
        .iter()
        .enumerate()
        .filter_map(|(idx, stage)| {
            stage
                .control_target_for_pot(idx, comp_id)
                .map(|target| make_binding(ctrl, comp_id, max_r, taper, target))
        })
        .collect();

    if bindings.len() <= 1 {
        return bindings;
    }

    // One physical pot can be present in multiple extracted stages. Runtime
    // control application fans out by component id to every owning stage, so a
    // single UI/control binding is enough and avoids duplicate host parameters.
    let label = ctrl.label.to_lowercase();
    if label.contains("level") || label.contains("volume") || label.contains("output") {
        bindings.pop().into_iter().collect()
    } else {
        bindings.into_iter().take(1).collect()
    }
}

fn make_binding(
    ctrl: &crate::dsl::ControlDef,
    comp_id: &str,
    max_r: f64,
    taper: PotTaper,
    target: ControlTarget,
) -> ControlBinding {
    ControlBinding {
        label: ctrl.label.clone(),
        target,
        component_id: comp_id.to_string(),
        max_resistance: max_r,
        taper,
        range: ctrl.range,
    }
}
