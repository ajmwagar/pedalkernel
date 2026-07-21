//! Control binding for the SPQR pipeline.
//!
//! Pots are components. Each stage knows how to handle its own pot changes.
//! The control module just routes: find stage → tell it the new position.
//!
//! Smoothing spreads parameter changes over ~32 samples to avoid zipper
//! noise and CPU spikes from recomputation.

use super::compiled::{CompiledPedal, ControlBinding, ControlTarget, PotHalf, StageBinding};
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

    // Unified cross-stage wiper-divider detection.
    //
    // A 3-terminal pot whose wiper feeds a load OUTSIDE its own stage divides
    // the signal by its position at that boundary. There are two such cases,
    // now handled by ONE mechanism (no label special-case):
    //   (1) Output level pot — wiper feeds the downstream output network
    //       (the classic Level/Volume/Output knob). Recognized by label OR by
    //       the wiper reaching a later passive group.
    //   (2) Makeup/drive pot into a high-Z active grid — wiper feeds a tube
    //       grid / BJT base / FET gate that lives in a LATER stage (the LA-2A
    //       "Gain" case). The grid is near-open, so the wiper voltage is a
    //       clean `position * V(wiper_top)` ratio: a wiper divider is the
    //       physically-faithful model (we do NOT jointly solve the shared
    //       wiper node across the boundary).
    // For each detected pot we emit a `WiperDivider` entry (carrying the live
    // position) and append a `ControlTarget::WiperDivider` to the owning
    // control's coordinated target set, so the SINGLE multi-target dispatch
    // path drives both the per-stage leaf re-stamps and the boundary divider.
    let mut output_dividers: BTreeMap<String, (usize, f64, PotTaper)> = BTreeMap::new();
    for ctrl in &compiled.controls {
        // Only pots that landed in a WDF stage can carry an inter-stage
        // divider (the divider applies after a serial WDF stage).
        let super::compiled::ControlTarget::PotInStage(stage_idx) = &ctrl.target else {
            continue;
        };
        let label_lower = ctrl.label.to_lowercase();
        let label_says_output = label_lower.contains("level")
            || label_lower.contains("volume")
            || label_lower.contains("output");
        let feeds_active_grid = pot_wiper_feeds_active_input(pedal, &ctrl.component_id);
        if !(label_says_output || feeds_active_grid) {
            continue;
        }
        // A wiper wired DIRECTLY to `out` needs no inter-stage divider (the
        // output probe already sees the wiper node).
        if pot_wiper_connects_directly_to_out(pedal, &ctrl.component_id) {
            continue;
        }
        let default = pedal
            .controls
            .iter()
            .find(|c| c.label == ctrl.label && c.component == ctrl.component_id)
            .map(|c| c.default)
            .unwrap_or(0.5);
        let position = ctrl
            .taper
            .apply(
                (ctrl.range.0 as f64 + default * (ctrl.range.1 as f64 - ctrl.range.0 as f64))
                    as crate::Wave,
            );
        output_dividers
            .entry(ctrl.component_id.clone())
            .and_modify(|(existing_stage_idx, existing_position, existing_taper)| {
                if *stage_idx >= *existing_stage_idx {
                    *existing_stage_idx = *stage_idx;
                    *existing_position = position as f64;
                    *existing_taper = ctrl.taper;
                }
            })
            .or_insert((*stage_idx, position as f64, ctrl.taper));
    }

    // Materialize the dividers AND register a WiperDivider target on each
    // owning control so dispatch fans the position into the divider too.
    for (pot_comp_id, (after_stage_idx, position, taper)) in &output_dividers {
        compiled
            .wiper_dividers
            .push(super::compiled::WiperDivider {
                after_stage_idx: *after_stage_idx,
                pot_comp_id: pot_comp_id.clone(),
                position: *position as crate::Wave,
                taper: *taper,
            });
        for ctrl in compiled.controls.iter_mut() {
            if &ctrl.component_id == pot_comp_id
                && !ctrl
                    .targets
                    .iter()
                    .any(|t| matches!(t.target, ControlTarget::WiperDivider { .. }))
            {
                ctrl.targets.push(StageBinding {
                    target: ControlTarget::WiperDivider {
                        after_stage_idx: *after_stage_idx,
                    },
                    half: PotHalf::Whole,
                    complement: false,
                });
            }
        }
    }

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
                    ctrl.default as crate::Wave,
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
        compiled.set_control_immediate(&ctrl.label, ctrl.default as crate::Wave);
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

/// Whether the pot's wiper feeds a high-impedance active-device control input
/// — a tube grid, BJT base, or FET gate — directly or through a grid-leak /
/// series resistor.
///
/// Such a load is near-open, so the wiper acts as a clean voltage divider into
/// it: turning the pot scales the drive into the downstream active stage. The
/// pot's two halves get extracted into different stages (the source-side half
/// loads the upstream passive network, the wiper-side half sits at the grid),
/// and only a wiper divider at the boundary delivers the divided voltage. This
/// detector recognizes the topology so the binding layer can add that divider
/// without relying on a label convention.
fn pot_wiper_feeds_active_input(pedal: &PedalDef, comp_id: &str) -> bool {
    use crate::dsl::Pin;
    /// Control-input pins of active devices.
    const ACTIVE_INPUT_PINS: [&str; 3] = ["grid", "base", "gate"];

    let is_active_input = |pin: &Pin| -> bool {
        matches!(pin, Pin::ComponentPin { pin, .. } if ACTIVE_INPUT_PINS.contains(&pin.as_str()))
    };

    // Every pin sharing a net with `pin` (the electrical node it sits on).
    let net_peers = |pin: &Pin| -> Vec<Pin> {
        let mut peers: Vec<Pin> = Vec::new();
        for net in &pedal.nets {
            let all: Vec<&Pin> = core::iter::once(&net.from).chain(net.to.iter()).collect();
            if all.iter().any(|p| *p == pin) {
                for &p in &all {
                    if p != pin && !peers.contains(p) {
                        peers.push(p.clone());
                    }
                }
            }
        }
        peers
    };

    // If a pin belongs to a 2-terminal SERIES passive (resistor or cap), return
    // the OTHER terminal of that element — the far end of the series link. `None`
    // for active-device pins, pot pins, 3+ terminal parts, or unknown ids. The
    // wiper→base coupling is a chain of such series R/C (uberdrive
    // wiper→R12→C8→base, lgsm wiper→C8→base — GAP 4b, pedalkernel-xki7).
    let series_far_pin = |pin: &Pin| -> Option<Pin> {
        let Pin::ComponentPin { component, pin: pn } = pin else {
            return None;
        };
        let comp = pedal.components.iter().find(|c| &c.id == component)?;
        let tag = comp.kind.type_tag();
        if tag != "resistor" && tag != "capacitor" {
            return None;
        }
        // 2-terminal a/b element: the far terminal is the opposite lug.
        let other = match pn.as_str() {
            "a" => "b",
            "b" => "a",
            _ => return None,
        };
        Some(Pin::ComponentPin {
            component: component.clone(),
            pin: other.to_string(),
        })
    };

    // BFS from the wiper node across series R/C chains; succeed on reaching an
    // active control input. Frontier holds NODE-representative pins (any pin on a
    // node); we expand to the node's peers, then hop each peer that is a series
    // R/C terminal to its far terminal (a new node), bounded by `visited`.
    let wiper = Pin::ComponentPin {
        component: comp_id.to_string(),
        pin: "w".to_string(),
    };
    let wiper_alt = Pin::ComponentPin {
        component: comp_id.to_string(),
        pin: "wiper".to_string(),
    };

    let mut visited: Vec<Pin> = Vec::new();
    let mut frontier: Vec<Pin> = vec![wiper.clone(), wiper_alt.clone()];
    // Guard against pathological nets; the corpus chains are ≤ a few hops.
    for _hop in 0..8 {
        if frontier.is_empty() {
            break;
        }
        let mut next: Vec<Pin> = Vec::new();
        for start in &frontier {
            for peer in net_peers(start) {
                if is_active_input(&peer) {
                    return true;
                }
                // Continue only through a series R/C to its far terminal.
                if let Some(far) = series_far_pin(&peer) {
                    if !visited.contains(&far) {
                        visited.push(far.clone());
                        next.push(far);
                    }
                }
            }
        }
        frontier = next;
    }
    false
}

/// Find every stage that owns a pot and create ONE coordinated binding.
///
/// Searches all stage types. The pot may be:
/// - A WdfPot leaf in a WdfStage tree (tree.set_pot works)
/// - Consumed by OpAmpRoot at compile time (needs gain recompute)
/// - In a MultiNlStage (delta-update scattering matrix)
///
/// A split physical pot can appear in MULTIPLE extracted stages — e.g. a
/// 3-terminal makeup pot whose two halves land in a WDF gain-reduction stage
/// AND a MultiNL tube-grid stage (the LA-2A "Gain" case). Earlier code kept
/// only the FIRST owner and discarded the rest, leaving the straddled half
/// frozen. This now produces ONE [`ControlBinding`] (= one host parameter)
/// whose `targets: Vec<StageBinding>` carries EVERY owning stage, so dispatch
/// fans a single position out to all of them. When the pot's wiper feeds a
/// downstream high-impedance load (a later-stage tube grid / BJT base, the
/// near-open-circuit divider case) a [`ControlTarget::WiperDivider`] target is
/// added too so the divided wiper voltage actually reaches the load — the
/// physically-faithful model when the wiper node is not jointly solved across
/// the boundary.
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

    // Collect a stage binding for EVERY stage that owns this pot. The runtime
    // re-stamps each leaf (base id + `__aw`/`__wb` halves) with its own
    // compile-time `complement` flag, so each binding uses `PotHalf::Whole`
    // and lets the stage's `set_control_pot` expand to the correct halves —
    // this avoids double-inverting the GND-touching half.
    let stage_targets: Vec<ControlTarget> = compiled
        .stages
        .iter()
        .enumerate()
        .filter_map(|(idx, stage)| stage.control_target_for_pot(idx, comp_id))
        .collect();

    if stage_targets.is_empty() {
        return Vec::new();
    }

    let mut targets: Vec<StageBinding> = stage_targets
        .iter()
        .cloned()
        .map(|target| StageBinding {
            target,
            half: PotHalf::Whole,
            complement: false,
        })
        .collect();

    // The representative/primary target (first owning stage) is kept on
    // `target` for the many back-compat readers of `.target`.
    let primary = stage_targets[0].clone();

    vec![ControlBinding {
        label: ctrl.label.clone(),
        target: primary,
        targets: std::mem::take(&mut targets),
        component_id: comp_id.to_string(),
        max_resistance: max_r as crate::Wave,
        taper,
        range: (ctrl.range.0 as crate::Wave, ctrl.range.1 as crate::Wave),
    }]
}
