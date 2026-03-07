//! Validation layer for `.pedal` files.
//!
//! Runs at compile time (not runtime) to catch common errors in `.pedal`
//! circuit definitions. Collects warnings without hard-failing — the goal
//! is to surface issues that would otherwise be silently ignored by the
//! compiler pipeline.
//!
//! # Categories of warnings
//!
//! - **Component reference errors**: nets/controls referencing components that don't exist
//! - **Invalid pin names**: using pins that don't exist for a given component type
//! - **Unrecognized modulation targets**: LFO/envelope outputs going to pins the compiler skips
//! - **Suspicious component values**: zero-ohm resistors, negative caps, etc.
//! - **Duplicate component IDs**: two components sharing the same name
//! - **Orphaned components**: components declared but never wired into any net
//! - **Missing signal path**: no route from `in` to `out`
//! - **Control binding issues**: controls targeting nonexistent components

use std::collections::{HashMap, HashSet};

use crate::dsl::*;
use crate::models;

/// Severity of a validation warning.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Severity {
    /// Something looks off but probably works (e.g., very high resistance).
    Info,
    /// Likely a mistake — the compiler will silently skip or fall back.
    Warning,
    /// Almost certainly a bug — broken signal path, missing component, etc.
    Error,
}

/// A single validation warning from a `.pedal` file.
#[derive(Debug, Clone)]
pub struct PedalWarning {
    pub severity: Severity,
    /// Short machine-readable code (e.g., "unknown-pin", "orphaned-component").
    pub code: &'static str,
    /// Human-readable explanation.
    pub message: String,
}

impl std::fmt::Display for PedalWarning {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let tag = match self.severity {
            Severity::Info => "info",
            Severity::Warning => "warning",
            Severity::Error => "error",
        };
        write!(f, "[{}] {}: {}", tag, self.code, self.message)
    }
}

/// Validate a parsed `PedalDef` and return all warnings.
///
/// This runs *after* parsing and *before* (or at the start of) compilation.
/// It never mutates the `PedalDef` and has no runtime cost.
pub fn validate_pedal(pedal: &PedalDef) -> Vec<PedalWarning> {
    let mut warnings = Vec::new();

    check_duplicate_component_ids(pedal, &mut warnings);
    check_component_values(pedal, &mut warnings);
    check_net_references(pedal, &mut warnings);
    check_pin_validity(pedal, &mut warnings);
    check_orphaned_components(pedal, &mut warnings);
    check_signal_path(pedal, &mut warnings);
    check_controls(pedal, &mut warnings);
    check_modulation_targets(pedal, &mut warnings);
    check_supply_config(pedal, &mut warnings);
    check_monitor_references(pedal, &mut warnings);
    check_model_names(pedal, &mut warnings);
    check_mirrors(pedal, &mut warnings);

    warnings
}

// ═══════════════════════════════════════════════════════════════════════════
// Individual checks
// ═══════════════════════════════════════════════════════════════════════════

/// Duplicate component IDs.
fn check_duplicate_component_ids(pedal: &PedalDef, w: &mut Vec<PedalWarning>) {
    let mut seen: HashMap<&str, usize> = HashMap::new();
    for (i, comp) in pedal.components.iter().enumerate() {
        if let Some(&prev) = seen.get(comp.id.as_str()) {
            w.push(PedalWarning {
                severity: Severity::Error,
                code: "duplicate-component-id",
                message: format!(
                    "Component '{}' declared at index {} and again at index {}",
                    comp.id, prev, i
                ),
            });
        } else {
            seen.insert(&comp.id, i);
        }
    }
}

/// Suspicious component values.
fn check_component_values(pedal: &PedalDef, w: &mut Vec<PedalWarning>) {
    for comp in &pedal.components {
        let issues = comp.kind.validate_values(&comp.id);
        for (severity, message) in issues {
            let code = match severity {
                Severity::Error => "invalid-value",
                Severity::Warning | Severity::Info => "suspicious-value",
            };
            w.push(PedalWarning {
                severity,
                code,
                message,
            });
        }
    }
}

/// Nets referencing component IDs that don't exist.
fn check_net_references(pedal: &PedalDef, w: &mut Vec<PedalWarning>) {
    let comp_ids: HashSet<&str> = pedal.components.iter().map(|c| c.id.as_str()).collect();
    let reserved: HashSet<&str> = [
        "in",
        "out",
        "gnd",
        "vcc",
        "fx_send",
        "fx_return",
        // Synth-specific CV/Gate nodes (from dsl.rs RESERVED_NODES)
        "gate",
        "cv_pitch",
        "cv_mod",
        "cv_filter",
    ]
    .iter()
    .copied()
    .collect();
    let supply_names: HashSet<&str> = pedal.supplies.iter().map(|s| s.name.as_str()).collect();
    let supply_list: Vec<&str> = pedal.supply_names();

    fn check_pin_ref(
        pin: &Pin,
        comp_ids: &HashSet<&str>,
        reserved: &HashSet<&str>,
        supply_names: &HashSet<&str>,
        supply_list: &[&str],
        w: &mut Vec<PedalWarning>,
    ) {
        match pin {
            Pin::ComponentPin { component, .. } => {
                if !comp_ids.contains(component.as_str()) {
                    w.push(PedalWarning {
                        severity: Severity::Error,
                        code: "unknown-component-ref",
                        message: format!(
                            "Net references component '{}' which is not declared in components",
                            component
                        ),
                    });
                }
            }
            Pin::Reserved(name) => {
                if !reserved.contains(name.as_str()) && !supply_names.contains(name.as_str()) {
                    w.push(PedalWarning {
                        severity: Severity::Warning,
                        code: "unknown-reserved-pin",
                        message: format!(
                            "Net uses reserved pin '{}' — expected one of: in, out, gnd, vcc{}",
                            name,
                            if supply_names.is_empty() {
                                String::new()
                            } else {
                                format!(", {}", supply_list.join(", "))
                            }
                        ),
                    });
                }
            }
            Pin::Fork {
                switch,
                destinations,
            } => {
                if !comp_ids.contains(switch.as_str()) {
                    w.push(PedalWarning {
                        severity: Severity::Error,
                        code: "unknown-component-ref",
                        message: format!(
                            "Fork references switch component '{}' which is not declared",
                            switch
                        ),
                    });
                }
                for dest in destinations {
                    check_pin_ref(dest, comp_ids, reserved, supply_names, supply_list, w);
                }
            }
        }
    }

    for net in &pedal.nets {
        check_pin_ref(
            &net.from,
            &comp_ids,
            &reserved,
            &supply_names,
            &supply_list,
            w,
        );
        for to_pin in &net.to {
            check_pin_ref(to_pin, &comp_ids, &reserved, &supply_names, &supply_list, w);
        }
    }
}

/// Valid pin names for a component type (delegates to Component trait).
fn valid_pins_for(kind: &dyn super::component::Component) -> &'static [&'static str] {
    kind.pin_config().valid_pins
}

/// Check that pin names used in nets are valid for their component type.
fn check_pin_validity(pedal: &PedalDef, w: &mut Vec<PedalWarning>) {
    use super::component::Component;

    let comp_map: HashMap<&str, &Box<dyn Component>> = pedal
        .components
        .iter()
        .map(|c| (c.id.as_str(), &c.kind))
        .collect();

    // Modulation target pins are valid on specific component types even though
    // they're not circuit pins (they're resolved during binding, not graph building).
    let is_valid_modulation_pin = |kind: &dyn Component, pin: &str| -> bool {
        kind.modulation_pins().contains(&pin)
    };

    let check_pin = |pin: &Pin, w: &mut Vec<PedalWarning>| {
        if let Pin::ComponentPin { component, pin } = pin {
            if let Some(kind) = comp_map.get(component.as_str()) {
                let valid = valid_pins_for(kind.as_ref());
                // Don't warn for component types that have no pin list
                // (switches, etc.) since they're not circuit elements
                if !valid.is_empty()
                    && !valid.contains(&pin.as_str())
                    && !is_valid_modulation_pin(kind.as_ref(), pin)
                {
                    w.push(PedalWarning {
                        severity: Severity::Error,
                        code: "unknown-pin",
                        message: format!(
                            "Pin '{}.{}' is not a recognized pin — expected one of: {}",
                            component,
                            pin,
                            valid.join(", ")
                        ),
                    });
                }
            }
            // If component doesn't exist, check_net_references already flagged it
        }
    };

    for net in &pedal.nets {
        check_pin(&net.from, w);
        for to_pin in &net.to {
            check_pin(to_pin, w);
        }
    }
}

/// Components declared but never referenced in any net.
fn check_orphaned_components(pedal: &PedalDef, w: &mut Vec<PedalWarning>) {
    let mut referenced: HashSet<String> = HashSet::new();

    fn collect_refs(pin: &Pin, refs: &mut HashSet<String>) {
        match pin {
            Pin::ComponentPin { component, .. } => {
                refs.insert(component.clone());
            }
            Pin::Fork {
                switch,
                destinations,
            } => {
                refs.insert(switch.clone());
                for d in destinations {
                    if let Pin::ComponentPin { component, .. } = d {
                        refs.insert(component.clone());
                    }
                }
            }
            Pin::Reserved(_) => {}
        }
    }

    for net in &pedal.nets {
        collect_refs(&net.from, &mut referenced);
        for to_pin in &net.to {
            collect_refs(to_pin, &mut referenced);
        }
    }

    // Controls also reference components
    for ctrl in &pedal.controls {
        referenced.insert(ctrl.component.clone());
    }
    for trim in &pedal.trims {
        referenced.insert(trim.component.clone());
    }

    for comp in &pedal.components {
        // Skip internal/virtual types that don't need net connections
        if comp.kind.is_control_only() {
            continue;
        }
        if !referenced.contains(&comp.id) {
            w.push(PedalWarning {
                severity: Severity::Warning,
                code: "orphaned-component",
                message: format!(
                    "Component '{}' is declared but never referenced in any net or control",
                    comp.id
                ),
            });
        }
    }
}

/// Check that there's a plausible signal path from `in` to `out`.
fn check_signal_path(pedal: &PedalDef, w: &mut Vec<PedalWarning>) {
    // Synths (circuits with VCOs) generate their own signal — they don't need
    // an in → out path since audio originates from the oscillator.
    let is_synth = pedal
        .components
        .iter()
        .any(|c| c.kind.type_tag() == "VCO");
    if is_synth {
        return;
    }

    // Build adjacency based on net connections (node = pin key, edges = same net).
    let mut adj: HashMap<String, HashSet<String>> = HashMap::new();

    let pin_key = |pin: &Pin| -> String {
        match pin {
            Pin::Reserved(s) => s.clone(),
            Pin::ComponentPin { component, pin } => format!("{}.{}", component, pin),
            Pin::Fork { switch, .. } => format!("__fork_{}", switch),
        }
    };

    // Also add implicit component-internal connections via signal_adjacencies().
    for comp in &pedal.components {
        for (pa, pb) in comp.kind.signal_adjacencies() {
            let ka = format!("{}.{}", comp.id, pa);
            let kb = format!("{}.{}", comp.id, pb);
            adj.entry(ka.clone()).or_default().insert(kb.clone());
            adj.entry(kb).or_default().insert(ka);
        }
    }

    // Add net connections
    for net in &pedal.nets {
        let from = pin_key(&net.from);
        for to_pin in &net.to {
            let to = pin_key(to_pin);
            adj.entry(from.clone()).or_default().insert(to.clone());
            adj.entry(to).or_default().insert(from.clone());
        }
    }

    // BFS from "in"
    let mut visited: HashSet<String> = HashSet::new();
    let mut queue = std::collections::VecDeque::new();
    visited.insert("in".to_string());
    queue.push_back("in".to_string());

    while let Some(node) = queue.pop_front() {
        if let Some(neighbors) = adj.get(&node) {
            for n in neighbors {
                if visited.insert(n.clone()) {
                    queue.push_back(n.clone());
                }
            }
        }
    }

    if !visited.contains("out") {
        w.push(PedalWarning {
            severity: Severity::Error,
            code: "no-signal-path",
            message: "No signal path found from 'in' to 'out'".to_string(),
        });
    }
}

/// Check control definitions reference valid components and properties.
fn check_controls(pedal: &PedalDef, w: &mut Vec<PedalWarning>) {
    use super::component::Component;

    let comp_map: HashMap<&str, &Box<dyn Component>> = pedal
        .components
        .iter()
        .map(|c| (c.id.as_str(), &c.kind))
        .collect();

    let check_ctrl = |ctrl: &ControlDef, w: &mut Vec<PedalWarning>, is_trim: bool| {
        let section = if is_trim { "trims" } else { "controls" };
        if !comp_map.contains_key(ctrl.component.as_str()) {
            w.push(PedalWarning {
                severity: Severity::Error,
                code: "unknown-control-component",
                message: format!(
                    "{} entry '{}' references component '{}' which is not declared",
                    section, ctrl.label, ctrl.component
                ),
            });
            return;
        }

        let kind = comp_map[ctrl.component.as_str()].as_ref();
        // Controls typically target potentiometers or switches
        let tag = kind.type_tag();
        if !matches!(tag, "potentiometer" | "switch" | "rotary switch") {
            w.push(PedalWarning {
                severity: Severity::Warning,
                code: "unusual-control-target",
                message: format!(
                    "{} entry '{}' targets '{}' which is a {:?} — expected a pot or switch",
                    section,
                    ctrl.label,
                    ctrl.component,
                    tag
                ),
            });
        }

        // Check range validity
        if ctrl.range.0 >= ctrl.range.1 {
            w.push(PedalWarning {
                severity: Severity::Warning,
                code: "invalid-control-range",
                message: format!(
                    "{} entry '{}' has inverted or empty range [{}, {}]",
                    section, ctrl.label, ctrl.range.0, ctrl.range.1
                ),
            });
        }

        // Check default is within range
        if ctrl.default < ctrl.range.0 || ctrl.default > ctrl.range.1 {
            w.push(PedalWarning {
                severity: Severity::Warning,
                code: "default-out-of-range",
                message: format!(
                    "{} entry '{}' default {} is outside range [{}, {}]",
                    section, ctrl.label, ctrl.default, ctrl.range.0, ctrl.range.1
                ),
            });
        }
    };

    // Check for duplicate control labels
    let mut seen_labels: HashSet<&str> = HashSet::new();
    for ctrl in &pedal.controls {
        if !seen_labels.insert(&ctrl.label) {
            w.push(PedalWarning {
                severity: Severity::Warning,
                code: "duplicate-control-label",
                message: format!(
                    "Control label '{}' appears more than once — only the last binding will take effect",
                    ctrl.label
                ),
            });
        }
        check_ctrl(ctrl, w, false);
    }

    for ctrl in &pedal.trims {
        check_ctrl(ctrl, w, true);
    }
}

/// Check LFO/envelope follower modulation targets.
///
/// This catches the `_ => continue` silent-skip in compile.rs lines 1409 and 1536.
fn check_modulation_targets(pedal: &PedalDef, w: &mut Vec<PedalWarning>) {
    use super::component::Component;

    let comp_map: HashMap<&str, &Box<dyn Component>> = pedal
        .components
        .iter()
        .map(|c| (c.id.as_str(), &c.kind))
        .collect();

    // Valid modulation target pins that the compiler recognizes
    let valid_mod_targets: HashSet<&str> = [
        "vgs",
        "gate",
        "led",
        "vgk",
        "vg1k",
        "iabc",
        "clock",
        "speed_mod",
        "delay_time",
    ]
    .iter()
    .copied()
    .collect();

    // Pins that are "destination" pins on passive components — these are valid net
    // connections but NOT modulation targets. When an LFO connects to, e.g., Depth.a,
    // the signal flows through the passive network but doesn't create a modulation binding.
    let passive_connection_pins: HashSet<&str> = ["a", "b", "w", "wiper", "in", "out", "rate"]
        .iter()
        .copied()
        .collect();

    // Check each modulation source's output connections
    for comp in &pedal.components {
        let source_type = match comp.kind.type_tag() {
            "LFO" => "LFO",
            "envelope follower" => "EnvelopeFollower",
            _ => continue,
        };

        for net in &pedal.nets {
            // Look for nets where this mod source's .out is the source
            if let Pin::ComponentPin { component, pin } = &net.from {
                if component == &comp.id && pin == "out" {
                    for target_pin in &net.to {
                        if let Pin::ComponentPin {
                            component: target_comp,
                            pin: target_prop,
                        } = target_pin
                        {
                            // Check if the target component exists
                            if !comp_map.contains_key(target_comp.as_str()) {
                                // Already caught by check_net_references
                                continue;
                            }

                            let target_kind = comp_map[target_comp.as_str()].as_ref();

                            // If the target pin is a known modulation target, great
                            if valid_mod_targets.contains(target_prop.as_str()) {
                                // Verify the target component type makes sense for this pin
                                check_mod_target_compatibility(
                                    source_type,
                                    &comp.id,
                                    target_comp,
                                    target_prop,
                                    target_kind,
                                    w,
                                );
                            } else if passive_connection_pins.contains(target_prop.as_str()) {
                                // This is a valid passive connection (e.g., LFO1.out -> Depth.a)
                                // but it won't create a modulation binding. Check if the user
                                // probably intended a modulation binding.
                                let tag = target_kind.type_tag();
                                let target_is_pot = tag == "potentiometer";
                                let target_is_passive = matches!(tag, "resistor" | "capacitor" | "inductor");
                                if !target_is_pot
                                    && !target_is_passive
                                    && tag != "LFO"
                                {
                                    // Connecting a modulation source to a non-passive, non-pot
                                    // component's generic pin is suspicious
                                    w.push(PedalWarning {
                                        severity: Severity::Info,
                                        code: "passive-mod-connection",
                                        message: format!(
                                            "{} '{}'.out -> '{}.{}' creates a passive net connection, not a modulation binding. \
                                             The compiler won't create an LFO/envelope modulation for this. \
                                             Use a recognized modulation pin (vgs, led, clock, etc.) for direct modulation.",
                                            source_type, comp.id, target_comp, target_prop
                                        ),
                                    });
                                }
                            } else {
                                // Unknown target pin — this is what the compiler's
                                // `_ => continue` silently skips
                                w.push(PedalWarning {
                                    severity: Severity::Warning,
                                    code: "unrecognized-mod-target",
                                    message: format!(
                                        "{} '{}'.out -> '{}.{}' uses unrecognized modulation target '{}'. \
                                         This connection will be silently ignored by the compiler. \
                                         Valid modulation targets are: {}",
                                        source_type, comp.id, target_comp, target_prop, target_prop,
                                        valid_mod_targets.iter().copied().collect::<Vec<_>>().join(", ")
                                    ),
                                });
                            }
                        }
                    }
                }
            }
        }
    }
}

/// Check that a modulation target pin makes sense for the target component type.
fn check_mod_target_compatibility(
    source_type: &str,
    source_id: &str,
    target_comp: &str,
    target_pin: &str,
    target_kind: &dyn super::component::Component,
    w: &mut Vec<PedalWarning>,
) {
    // The pin is a mismatch if this component doesn't list it as a modulation pin.
    let mismatch = !target_kind.modulation_pins().contains(&target_pin);

    if mismatch {
        w.push(PedalWarning {
            severity: Severity::Warning,
            code: "mod-target-type-mismatch",
            message: format!(
                "{} '{}'.out -> '{}.{}': pin '{}' is typically used with {} components, \
                 but '{}' is a {}",
                source_type,
                source_id,
                target_comp,
                target_pin,
                target_pin,
                expected_component_for_pin(target_pin),
                target_comp,
                target_kind.type_tag()
            ),
        });
    }
}

fn expected_component_for_pin(pin: &str) -> &'static str {
    match pin {
        "vgs" | "gate" => "JFET/MOSFET",
        "led" => "photocoupler",
        "vgk" => "triode",
        "vg1k" => "pentode",
        "iabc" => "OTA (CA3080)",
        "clock" => "BBD",
        "speed_mod" | "delay_time" => "delay line",
        _ => "unknown",
    }
}

fn component_type_name(kind: &dyn super::component::Component) -> &'static str {
    kind.type_tag()
}

/// Check supply configuration.
fn check_supply_config(pedal: &PedalDef, w: &mut Vec<PedalWarning>) {
    for supply in &pedal.supplies {
        if supply.config.voltage == 0.0 {
            w.push(PedalWarning {
                severity: Severity::Warning,
                code: "zero-supply-voltage",
                message: format!(
                    "Supply rail '{}' has 0V — circuit won't bias correctly",
                    supply.name
                ),
            });
        }
        if let Some(imp) = supply.config.impedance {
            if imp < 0.0 {
                w.push(PedalWarning {
                    severity: Severity::Error,
                    code: "invalid-value",
                    message: format!(
                        "Supply rail '{}' has negative impedance {:.1} Ω",
                        supply.name, imp
                    ),
                });
            }
        }
        if let Some(cap) = supply.config.filter_cap {
            if cap <= 0.0 {
                w.push(PedalWarning {
                    severity: Severity::Error,
                    code: "invalid-value",
                    message: format!(
                        "Supply rail '{}' has non-positive filter cap {:.2e} F",
                        supply.name, cap
                    ),
                });
            }
        }
    }

    // Check for duplicate supply names
    let mut seen: HashSet<&str> = HashSet::new();
    for supply in &pedal.supplies {
        if !seen.insert(&supply.name) {
            w.push(PedalWarning {
                severity: Severity::Error,
                code: "duplicate-supply-name",
                message: format!(
                    "Supply rail name '{}' is declared more than once",
                    supply.name
                ),
            });
        }
    }
}

/// Check monitor definitions reference valid components.
fn check_monitor_references(pedal: &PedalDef, w: &mut Vec<PedalWarning>) {
    let comp_ids: HashSet<&str> = pedal.components.iter().map(|c| c.id.as_str()).collect();
    let reserved_monitors = ["input", "output"];

    for monitor in &pedal.monitors {
        if !comp_ids.contains(monitor.component.as_str())
            && !reserved_monitors.contains(&monitor.component.as_str())
        {
            w.push(PedalWarning {
                severity: Severity::Warning,
                code: "unknown-monitor-component",
                message: format!(
                    "Monitor '{}' references component '{}' which is not declared",
                    monitor.label, monitor.component
                ),
            });
        }
    }
}

/// Check mirrored pot declarations.
///
/// Validates:
/// - Mirror target exists and is a potentiometer
/// - No mirror chains (A mirrors B mirrors C)
/// - Mirrored pots do not appear in the controls block
/// - Warns if mirrored and source pots have different resistances
fn check_mirrors(pedal: &PedalDef, w: &mut Vec<PedalWarning>) {
    use super::component::Component;

    let comp_map: HashMap<&str, &Box<dyn Component>> = pedal
        .components
        .iter()
        .map(|c| (c.id.as_str(), &c.kind))
        .collect();

    for (mirrored_id, source_id) in &pedal.mirrors {
        // 1. Source must exist
        let Some(source_kind) = comp_map.get(source_id.as_str()) else {
            w.push(PedalWarning {
                severity: Severity::Error,
                code: "mirror-target-missing",
                message: format!(
                    "Mirrored pot '{}' references '{}' which is not declared",
                    mirrored_id, source_id
                ),
            });
            continue;
        };

        // 2. Source must be a potentiometer
        if !source_kind.is_pot() {
            w.push(PedalWarning {
                severity: Severity::Error,
                code: "mirror-target-not-pot",
                message: format!(
                    "Mirrored pot '{}' references '{}' which is a {}, not a potentiometer",
                    mirrored_id,
                    source_id,
                    component_type_name(source_kind.as_ref())
                ),
            });
            continue;
        }

        // 3. Mirrored component must itself be a pot
        if let Some(mirrored_kind) = comp_map.get(mirrored_id.as_str()) {
            if !mirrored_kind.is_pot() {
                w.push(PedalWarning {
                    severity: Severity::Error,
                    code: "mirror-not-pot",
                    message: format!(
                        "'{}' uses `mirrors` but is a {}, not a potentiometer",
                        mirrored_id,
                        component_type_name(mirrored_kind.as_ref())
                    ),
                });
            }
        }

        // 4. No chains: source must not itself be mirrored
        if pedal.mirrors.contains_key(source_id) {
            w.push(PedalWarning {
                severity: Severity::Error,
                code: "mirror-chain",
                message: format!(
                    "'{}' mirrors '{}' which itself mirrors '{}' — mirror chains are not allowed",
                    mirrored_id,
                    source_id,
                    pedal.mirrors.get(source_id).unwrap()
                ),
            });
        }

        // 5. Mirrored pot must not appear in controls
        if pedal.controls.iter().any(|c| c.component == *mirrored_id) {
            w.push(PedalWarning {
                severity: Severity::Error,
                code: "mirror-in-controls",
                message: format!(
                    "Mirrored pot '{}' must not appear in the controls block — \
                     its position is derived from '{}'",
                    mirrored_id, source_id
                ),
            });
        }

        // 6. Warn if resistances differ
        if let (Some(mirrored_kind), Some(source_kind_inner)) = (
            comp_map.get(mirrored_id.as_str()),
            comp_map.get(source_id.as_str()),
        ) {
            if let (Some(r_mirrored), Some(r_source)) =
                (mirrored_kind.resistance(), source_kind_inner.resistance())
            {
                if (r_mirrored - r_source).abs() > f64::EPSILON {
                    w.push(PedalWarning {
                        severity: Severity::Warning,
                        code: "mirror-resistance-mismatch",
                        message: format!(
                            "Mirrored pot '{}' ({:.0} Ω) has different resistance than source '{}' ({:.0} Ω) — \
                             dual-gang pots typically have matched gangs",
                            mirrored_id, r_mirrored, source_id, r_source
                        ),
                    });
                }
            }
        }
    }
}

/// Check that SPICE model names referenced by components actually exist in
/// the model registry. Emits `"unknown-model"` errors with suggestions for
/// near-matches.
fn check_model_names(pedal: &PedalDef, w: &mut Vec<PedalWarning>) {
    for comp in &pedal.components {
        let kind = comp.kind.as_ref();
        let name = match kind.model_name() {
            Some(n) => n,
            None => continue,
        };

        let (kind_label, lookup, all_names): (
            &str,
            fn(&str) -> bool,
            fn() -> Vec<&'static str>,
        ) = if kind.is_bjt() {
            (
                "BJT",
                |n| models::bjt_by_name(n).is_some(),
                models::bjt_model_names,
            )
        } else if kind.is_jfet() {
            (
                "JFET",
                |n| models::jfet_by_name(n).is_some(),
                models::jfet_model_names,
            )
        } else if kind.is_tube() {
            // Distinguish pentode vs triode/varimu for the correct model registry
            if kind.type_tag() == "pentode" {
                (
                    "pentode",
                    |n| models::pentode_by_name(n).is_some(),
                    models::pentode_model_names,
                )
            } else {
                (
                    "triode",
                    |n| models::triode_by_name(n).is_some(),
                    models::triode_model_names,
                )
            }
        } else {
            continue;
        };

        if !lookup(name) {
            let available = all_names();
            let suggestions = find_similar_names(name, &available, 3);
            let suggestion_text = if suggestions.is_empty() {
                String::new()
            } else {
                format!(". Did you mean: {}?", suggestions.join(", "))
            };
            w.push(PedalWarning {
                severity: Severity::Error,
                code: "unknown-model",
                message: format!(
                    "Component '{}' references unknown {} model '{}'{} ({} models available)",
                    comp.id,
                    kind_label,
                    name,
                    suggestion_text,
                    available.len()
                ),
            });
        }
    }
}

/// Find model names similar to `query` using case-insensitive edit distance.
/// Returns up to `max` suggestions sorted by distance.
fn find_similar_names<'a>(query: &str, candidates: &[&'a str], max: usize) -> Vec<&'a str> {
    let query_upper = query.to_uppercase();
    let mut scored: Vec<(&str, usize)> = candidates
        .iter()
        .filter_map(|&candidate| {
            let dist = edit_distance(&query_upper, &candidate.to_uppercase());
            // Only suggest if reasonably close (within half the query length + 2)
            let threshold = query_upper.len() / 2 + 2;
            if dist <= threshold {
                Some((candidate, dist))
            } else {
                None
            }
        })
        .collect();
    scored.sort_by_key(|&(_, d)| d);
    scored.into_iter().take(max).map(|(name, _)| name).collect()
}

/// Simple Levenshtein edit distance.
fn edit_distance(a: &str, b: &str) -> usize {
    let a: Vec<char> = a.chars().collect();
    let b: Vec<char> = b.chars().collect();
    let (m, n) = (a.len(), b.len());
    let mut dp = vec![vec![0usize; n + 1]; m + 1];
    for i in 0..=m {
        dp[i][0] = i;
    }
    for j in 0..=n {
        dp[0][j] = j;
    }
    for i in 1..=m {
        for j in 1..=n {
            let cost = if a[i - 1] == b[j - 1] { 0 } else { 1 };
            dp[i][j] = (dp[i - 1][j] + 1)
                .min(dp[i][j - 1] + 1)
                .min(dp[i - 1][j - 1] + cost);
        }
    }
    dp[m][n]
}

/// Validate `.pedal` files for use in build scripts.
///
/// Reads each file, parses it, runs full validation, and prints
/// `cargo::warning=` directives. Panics (failing the build) if any
/// errors are found.
///
/// Call this from your crate's `build.rs`:
/// ```no_run
/// // build.rs
/// fn main() {
///     pedalkernel::compiler::validate_pedal_files(&[
///         "pedals/my_overdrive.pedal",
///         "pedals/my_phaser.pedal",
///     ]);
/// }
/// ```
pub fn validate_pedal_files(paths: &[&str]) {
    // Re-run if any model file changes
    for model_file in [
        "models/transistors.model",
        "models/jfets.model",
        "models/triodes.model",
        "models/pentodes.model",
        "models/diodes.model",
        "models/leds.model",
        "models/schottky.model",
        "models/zeners.model",
    ] {
        println!("cargo:rerun-if-changed={model_file}");
    }

    let mut all_errors = Vec::new();

    for path in paths {
        println!("cargo:rerun-if-changed={path}");
        let src =
            std::fs::read_to_string(path).unwrap_or_else(|e| panic!("Cannot read {path}: {e}"));
        let pedal = crate::dsl::parse_pedal_file(&src)
            .unwrap_or_else(|e| panic!("{path}: parse error: {e}"));
        let warnings = validate_pedal(&pedal);
        for w in &warnings {
            match w.severity {
                Severity::Error => {
                    println!("cargo::warning={path}: error[{}]: {}", w.code, w.message);
                    all_errors.push(format!("{path}: {w}"));
                }
                Severity::Warning => {
                    println!("cargo::warning={path}: warn[{}]: {}", w.code, w.message);
                }
                Severity::Info => {
                    println!("cargo::warning={path}: info[{}]: {}", w.code, w.message);
                }
            }
        }
    }

    if !all_errors.is_empty() {
        panic!(
            "\n{} .pedal validation error(s):\n  {}\n",
            all_errors.len(),
            all_errors.join("\n  ")
        );
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Tests
// ═══════════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod tests {
    use super::*;
    use crate::compiler::component::Component;
    use crate::compiler::components::*;

    /// Build a minimal valid pedal for testing.
    fn minimal_pedal() -> PedalDef {
        PedalDef {
            name: "Test".to_string(),
            subtitle: None,
            supplies: vec![],
            components: vec![ComponentDef {
                id: "R1".to_string(),
                kind: Box::new(Resistor { value: 10_000.0 }),
            }],
            nets: vec![
                NetDef {
                    from: Pin::Reserved("in".to_string()),
                    to: vec![Pin::ComponentPin {
                        component: "R1".to_string(),
                        pin: "a".to_string(),
                    }],
                },
                NetDef {
                    from: Pin::ComponentPin {
                        component: "R1".to_string(),
                        pin: "b".to_string(),
                    },
                    to: vec![Pin::Reserved("out".to_string())],
                },
            ],
            controls: vec![],
            trims: vec![],
            monitors: vec![],
            sidechains: vec![],
            mirrors: std::collections::HashMap::new(),
            midi_bindings: vec![],
        }
    }

    fn has_code(warnings: &[PedalWarning], code: &str) -> bool {
        warnings.iter().any(|w| w.code == code)
    }

    fn warnings_with_code<'a>(warnings: &'a [PedalWarning], code: &str) -> Vec<&'a PedalWarning> {
        warnings.iter().filter(|w| w.code == code).collect()
    }

    #[test]
    fn minimal_valid_pedal_has_no_errors() {
        let pedal = minimal_pedal();
        let warnings = validate_pedal(&pedal);
        let errors: Vec<_> = warnings
            .iter()
            .filter(|w| w.severity == Severity::Error)
            .collect();
        assert!(errors.is_empty(), "Unexpected errors: {:?}", errors);
    }

    #[test]
    fn duplicate_component_id() {
        let mut pedal = minimal_pedal();
        pedal.components.push(ComponentDef {
            id: "R1".to_string(),
            kind: Box::new(Resistor { value: 1000.0 }),
        });
        let warnings = validate_pedal(&pedal);
        assert!(has_code(&warnings, "duplicate-component-id"));
    }

    #[test]
    fn zero_resistance() {
        let mut pedal = minimal_pedal();
        pedal.components[0].kind = Box::new(Resistor { value: 0.0 });
        let warnings = validate_pedal(&pedal);
        assert!(has_code(&warnings, "invalid-value"));
    }

    #[test]
    fn very_large_capacitor_warns() {
        let mut pedal = minimal_pedal();
        pedal.components.push(ComponentDef {
            id: "C1".to_string(),
            kind: Box::new(Capacitor { config: CapConfig::new(100.0) }), // 100 F!
        });
        // Wire it in
        pedal.nets.push(NetDef {
            from: Pin::ComponentPin {
                component: "C1".to_string(),
                pin: "a".to_string(),
            },
            to: vec![Pin::Reserved("gnd".to_string())],
        });
        pedal.nets.push(NetDef {
            from: Pin::ComponentPin {
                component: "C1".to_string(),
                pin: "b".to_string(),
            },
            to: vec![Pin::Reserved("gnd".to_string())],
        });
        let warnings = validate_pedal(&pedal);
        assert!(has_code(&warnings, "suspicious-value"));
    }

    #[test]
    fn net_references_nonexistent_component() {
        let mut pedal = minimal_pedal();
        pedal.nets.push(NetDef {
            from: Pin::ComponentPin {
                component: "NOPE".to_string(),
                pin: "a".to_string(),
            },
            to: vec![Pin::Reserved("gnd".to_string())],
        });
        let warnings = validate_pedal(&pedal);
        assert!(has_code(&warnings, "unknown-component-ref"));
    }

    #[test]
    fn unknown_pin_name() {
        let mut pedal = minimal_pedal();
        pedal.nets[0] = NetDef {
            from: Pin::Reserved("in".to_string()),
            to: vec![Pin::ComponentPin {
                component: "R1".to_string(),
                pin: "cathode".to_string(), // resistors don't have cathodes
            }],
        };
        let warnings = validate_pedal(&pedal);
        assert!(has_code(&warnings, "unknown-pin"));
    }

    #[test]
    fn orphaned_component() {
        let mut pedal = minimal_pedal();
        pedal.components.push(ComponentDef {
            id: "R_lonely".to_string(),
            kind: Box::new(Resistor { value: 10_000.0 }),
        });
        let warnings = validate_pedal(&pedal);
        assert!(has_code(&warnings, "orphaned-component"));
    }

    #[test]
    fn no_signal_path() {
        let pedal = PedalDef {
            name: "Broken".to_string(),
            subtitle: None,
            supplies: vec![],
            components: vec![ComponentDef {
                id: "R1".to_string(),
                kind: Box::new(Resistor { value: 10_000.0 }),
            }],
            nets: vec![
                // in -> R1.a but R1.b -> gnd (not out)
                NetDef {
                    from: Pin::Reserved("in".to_string()),
                    to: vec![Pin::ComponentPin {
                        component: "R1".to_string(),
                        pin: "a".to_string(),
                    }],
                },
                NetDef {
                    from: Pin::ComponentPin {
                        component: "R1".to_string(),
                        pin: "b".to_string(),
                    },
                    to: vec![Pin::Reserved("gnd".to_string())],
                },
            ],
            controls: vec![],
            trims: vec![],
            monitors: vec![],
            sidechains: vec![],
            mirrors: std::collections::HashMap::new(),
            midi_bindings: vec![],
        };
        let warnings = validate_pedal(&pedal);
        assert!(has_code(&warnings, "no-signal-path"));
    }

    #[test]
    fn control_references_nonexistent_component() {
        let mut pedal = minimal_pedal();
        pedal.controls.push(ControlDef {
            component: "Ghost".to_string(),
            property: "position".to_string(),
            label: "Volume".to_string(),
            range: (0.0, 1.0),
            default: 0.5,
        });
        let warnings = validate_pedal(&pedal);
        assert!(has_code(&warnings, "unknown-control-component"));
    }

    #[test]
    fn control_default_out_of_range() {
        let mut pedal = minimal_pedal();
        pedal.components.push(ComponentDef {
            id: "Vol".to_string(),
            kind: Box::new(Potentiometer { max_r: 100_000.0, taper: PotTaper::B }),
        });
        pedal.nets.push(NetDef {
            from: Pin::ComponentPin {
                component: "Vol".to_string(),
                pin: "a".to_string(),
            },
            to: vec![Pin::Reserved("gnd".to_string())],
        });
        pedal.controls.push(ControlDef {
            component: "Vol".to_string(),
            property: "position".to_string(),
            label: "Volume".to_string(),
            range: (0.0, 1.0),
            default: 1.5, // out of range
        });
        let warnings = validate_pedal(&pedal);
        assert!(has_code(&warnings, "default-out-of-range"));
    }

    #[test]
    fn unrecognized_modulation_target() {
        let mut pedal = minimal_pedal();
        pedal.components.push(ComponentDef {
            id: "LFO1".to_string(),
            kind: Box::new(Lfo { waveform: LfoWaveformDsl::Triangle, timing_r: 150_000.0, timing_c: 1e-6 }),
        });
        pedal.components.push(ComponentDef {
            id: "Depth".to_string(),
            kind: Box::new(Potentiometer { max_r: 50_000.0, taper: PotTaper::B }),
        });
        // LFO1.out -> Depth.a — this is a passive connection, not a modulation target
        // The compiler silently skips it at line 1409
        pedal.nets.push(NetDef {
            from: Pin::ComponentPin {
                component: "LFO1".to_string(),
                pin: "out".to_string(),
            },
            to: vec![Pin::ComponentPin {
                component: "R1".to_string(),
                pin: "modulate".to_string(), // not a real pin
            }],
        });
        let warnings = validate_pedal(&pedal);
        assert!(
            has_code(&warnings, "unrecognized-mod-target"),
            "Expected 'unrecognized-mod-target' warning. Got: {:?}",
            warnings
        );
    }

    #[test]
    fn lfo_to_wrong_component_type() {
        let mut pedal = minimal_pedal();
        pedal.components.push(ComponentDef {
            id: "LFO1".to_string(),
            kind: Box::new(Lfo { waveform: LfoWaveformDsl::Sine, timing_r: 100_000.0, timing_c: 1e-6 }),
        });
        // LFO1.out -> R1.vgs — resistors don't have vgs
        pedal.nets.push(NetDef {
            from: Pin::ComponentPin {
                component: "LFO1".to_string(),
                pin: "out".to_string(),
            },
            to: vec![Pin::ComponentPin {
                component: "R1".to_string(),
                pin: "vgs".to_string(),
            }],
        });
        let warnings = validate_pedal(&pedal);
        assert!(
            has_code(&warnings, "mod-target-type-mismatch"),
            "Expected 'mod-target-type-mismatch' warning. Got: {:?}",
            warnings
        );
    }

    #[test]
    fn valid_lfo_to_jfet_vgs_no_warning() {
        let mut pedal = minimal_pedal();
        pedal.components.push(ComponentDef {
            id: "LFO1".to_string(),
            kind: Box::new(Lfo { waveform: LfoWaveformDsl::Triangle, timing_r: 1_000_000.0, timing_c: 220e-9 }),
        });
        pedal.components.push(ComponentDef {
            id: "J1".to_string(),
            kind: Box::new(NJfet { model: "2N5952".to_string() }),
        });
        pedal.nets.push(NetDef {
            from: Pin::ComponentPin {
                component: "LFO1".to_string(),
                pin: "out".to_string(),
            },
            to: vec![Pin::ComponentPin {
                component: "J1".to_string(),
                pin: "vgs".to_string(),
            }],
        });
        pedal.nets.push(NetDef {
            from: Pin::ComponentPin {
                component: "J1".to_string(),
                pin: "drain".to_string(),
            },
            to: vec![Pin::Reserved("gnd".to_string())],
        });
        pedal.nets.push(NetDef {
            from: Pin::ComponentPin {
                component: "J1".to_string(),
                pin: "source".to_string(),
            },
            to: vec![Pin::Reserved("gnd".to_string())],
        });
        let warnings = validate_pedal(&pedal);
        assert!(
            !has_code(&warnings, "mod-target-type-mismatch"),
            "Got unexpected mod-target-type-mismatch: {:?}",
            warnings_with_code(&warnings, "mod-target-type-mismatch")
        );
        assert!(
            !has_code(&warnings, "unrecognized-mod-target"),
            "Got unexpected unrecognized-mod-target: {:?}",
            warnings_with_code(&warnings, "unrecognized-mod-target")
        );
    }

    #[test]
    fn lfo_to_pot_is_passive_no_hard_warning() {
        let mut pedal = minimal_pedal();
        pedal.components.push(ComponentDef {
            id: "LFO1".to_string(),
            kind: Box::new(Lfo { waveform: LfoWaveformDsl::Triangle, timing_r: 150_000.0, timing_c: 1e-6 }),
        });
        pedal.components.push(ComponentDef {
            id: "Depth".to_string(),
            kind: Box::new(Potentiometer { max_r: 50_000.0, taper: PotTaper::B }),
        });
        pedal.nets.push(NetDef {
            from: Pin::ComponentPin {
                component: "LFO1".to_string(),
                pin: "out".to_string(),
            },
            to: vec![Pin::ComponentPin {
                component: "Depth".to_string(),
                pin: "a".to_string(),
            }],
        });
        pedal.nets.push(NetDef {
            from: Pin::ComponentPin {
                component: "Depth".to_string(),
                pin: "b".to_string(),
            },
            to: vec![Pin::Reserved("gnd".to_string())],
        });
        let warnings = validate_pedal(&pedal);
        // LFO -> pot.a is a valid passive connection, should NOT be flagged as unrecognized
        assert!(!has_code(&warnings, "unrecognized-mod-target"));
    }

    #[test]
    fn duplicate_control_label() {
        let mut pedal = minimal_pedal();
        pedal.components.push(ComponentDef {
            id: "P1".to_string(),
            kind: Box::new(Potentiometer { max_r: 100_000.0, taper: PotTaper::B }),
        });
        pedal.components.push(ComponentDef {
            id: "P2".to_string(),
            kind: Box::new(Potentiometer { max_r: 100_000.0, taper: PotTaper::B }),
        });
        pedal.nets.push(NetDef {
            from: Pin::ComponentPin {
                component: "P1".to_string(),
                pin: "a".to_string(),
            },
            to: vec![Pin::Reserved("gnd".to_string())],
        });
        pedal.nets.push(NetDef {
            from: Pin::ComponentPin {
                component: "P2".to_string(),
                pin: "a".to_string(),
            },
            to: vec![Pin::Reserved("gnd".to_string())],
        });
        pedal.controls.push(ControlDef {
            component: "P1".to_string(),
            property: "position".to_string(),
            label: "Drive".to_string(),
            range: (0.0, 1.0),
            default: 0.5,
        });
        pedal.controls.push(ControlDef {
            component: "P2".to_string(),
            property: "position".to_string(),
            label: "Drive".to_string(), // duplicate!
            range: (0.0, 1.0),
            default: 0.5,
        });
        let warnings = validate_pedal(&pedal);
        assert!(has_code(&warnings, "duplicate-control-label"));
    }

    #[test]
    fn validate_real_tube_screamer() {
        let src = std::fs::read_to_string("examples/pedals/overdrive/tube_screamer.pedal")
            .expect("tube_screamer.pedal not found");
        let pedal = parse_pedal_file(&src).expect("parse failed");
        let warnings = validate_pedal(&pedal);
        let errors: Vec<_> = warnings
            .iter()
            .filter(|w| w.severity == Severity::Error)
            .collect();
        assert!(
            errors.is_empty(),
            "Tube Screamer should have no errors: {:?}",
            errors
        );
    }

    #[test]
    fn validate_real_phase90() {
        let src = std::fs::read_to_string("examples/pedals/phaser/phase90.pedal")
            .expect("phase90.pedal not found");
        let pedal = parse_pedal_file(&src).expect("parse failed");
        let warnings = validate_pedal(&pedal);
        let errors: Vec<_> = warnings
            .iter()
            .filter(|w| w.severity == Severity::Error)
            .collect();
        assert!(
            errors.is_empty(),
            "Phase 90 should have no errors: {:?}",
            errors
        );
    }

    #[test]
    fn validate_real_boss_ce2() {
        let src = std::fs::read_to_string("examples/pedals/modulation/boss_ce2.pedal")
            .expect("boss_ce2.pedal not found");
        let pedal = parse_pedal_file(&src).expect("parse failed");
        let warnings = validate_pedal(&pedal);
        let errors: Vec<_> = warnings
            .iter()
            .filter(|w| w.severity == Severity::Error)
            .collect();
        assert!(
            errors.is_empty(),
            "Boss CE-2 should have no errors: {:?}",
            errors
        );
    }

    /// Scan ALL .pedal files and print every warning found.
    /// This test always passes — it's for visibility, not assertions.
    #[test]
    fn scan_all_pedal_files() {
        fn walk(dir: &std::path::Path, results: &mut Vec<(String, Vec<PedalWarning>)>) {
            if let Ok(entries) = std::fs::read_dir(dir) {
                for entry in entries.flatten() {
                    let path = entry.path();
                    if path.is_dir() {
                        walk(&path, results);
                    } else if path.extension().map_or(false, |e| e == "pedal") {
                        let src = std::fs::read_to_string(&path).unwrap();
                        if let Ok(pedal) = parse_pedal_file(&src) {
                            let warnings = validate_pedal(&pedal);
                            if !warnings.is_empty() {
                                results.push((path.display().to_string(), warnings));
                            }
                        }
                    }
                }
            }
        }

        let mut results = Vec::new();
        walk(std::path::Path::new("examples"), &mut results);
        walk(std::path::Path::new("tests/test_pedals"), &mut results);

        if results.is_empty() {
            eprintln!("\n=== All .pedal files validated clean ===\n");
        } else {
            let mut total = 0;
            for (path, warnings) in &results {
                eprintln!("\n--- {} ---", path);
                for w in warnings {
                    let tag = match w.severity {
                        Severity::Info => "INFO",
                        Severity::Warning => "WARN",
                        Severity::Error => "ERR ",
                    };
                    eprintln!("  [{}] {}: {}", tag, w.code, w.message);
                    total += 1;
                }
            }
            eprintln!(
                "\n=== {} warnings across {} files ===\n",
                total,
                results.len()
            );
        }
    }

    // ── Model name validation tests ─────────────────────────────────────

    #[test]
    fn valid_bjt_model_name_no_warning() {
        let mut pedal = minimal_pedal();
        pedal.components.push(ComponentDef {
            id: "Q1".to_string(),
            kind: Box::new(Npn { model: "2N3904".to_string() }),
        });
        pedal.nets.push(NetDef {
            from: Pin::ComponentPin {
                component: "Q1".to_string(),
                pin: "collector".to_string(),
            },
            to: vec![Pin::Reserved("gnd".to_string())],
        });
        pedal.nets.push(NetDef {
            from: Pin::ComponentPin {
                component: "Q1".to_string(),
                pin: "emitter".to_string(),
            },
            to: vec![Pin::Reserved("gnd".to_string())],
        });
        pedal.nets.push(NetDef {
            from: Pin::ComponentPin {
                component: "Q1".to_string(),
                pin: "base".to_string(),
            },
            to: vec![Pin::Reserved("gnd".to_string())],
        });
        let warnings = validate_pedal(&pedal);
        assert!(
            !has_code(&warnings, "unknown-model"),
            "Valid BJT '2N3904' should not trigger unknown-model: {:?}",
            warnings_with_code(&warnings, "unknown-model")
        );
    }

    #[test]
    fn invalid_bjt_model_name_emits_error() {
        let mut pedal = minimal_pedal();
        pedal.components.push(ComponentDef {
            id: "Q1".to_string(),
            kind: Box::new(Npn { model: "2N3904X".to_string() }),
        });
        pedal.nets.push(NetDef {
            from: Pin::ComponentPin {
                component: "Q1".to_string(),
                pin: "collector".to_string(),
            },
            to: vec![Pin::Reserved("gnd".to_string())],
        });
        pedal.nets.push(NetDef {
            from: Pin::ComponentPin {
                component: "Q1".to_string(),
                pin: "emitter".to_string(),
            },
            to: vec![Pin::Reserved("gnd".to_string())],
        });
        pedal.nets.push(NetDef {
            from: Pin::ComponentPin {
                component: "Q1".to_string(),
                pin: "base".to_string(),
            },
            to: vec![Pin::Reserved("gnd".to_string())],
        });
        let warnings = validate_pedal(&pedal);
        let model_errors = warnings_with_code(&warnings, "unknown-model");
        assert!(
            !model_errors.is_empty(),
            "Invalid BJT '2N3904X' should emit unknown-model error"
        );
        assert_eq!(model_errors[0].severity, Severity::Error);
        // Should suggest the real model name
        assert!(
            model_errors[0].message.contains("2N3904"),
            "Should suggest '2N3904', got: {}",
            model_errors[0].message
        );
    }

    #[test]
    fn valid_triode_model_name_no_warning() {
        let mut pedal = minimal_pedal();
        pedal.components.push(ComponentDef {
            id: "V1".to_string(),
            kind: Box::new(Triode { model: "12AX7".to_string() }),
        });
        pedal.nets.push(NetDef {
            from: Pin::ComponentPin {
                component: "V1".to_string(),
                pin: "plate".to_string(),
            },
            to: vec![Pin::Reserved("gnd".to_string())],
        });
        pedal.nets.push(NetDef {
            from: Pin::ComponentPin {
                component: "V1".to_string(),
                pin: "cathode".to_string(),
            },
            to: vec![Pin::Reserved("gnd".to_string())],
        });
        let warnings = validate_pedal(&pedal);
        assert!(
            !has_code(&warnings, "unknown-model"),
            "Valid triode '12AX7' should not trigger unknown-model: {:?}",
            warnings_with_code(&warnings, "unknown-model")
        );
    }

    #[test]
    fn invalid_jfet_model_name_emits_error() {
        let mut pedal = minimal_pedal();
        pedal.components.push(ComponentDef {
            id: "J1".to_string(),
            kind: Box::new(NJfet { model: "NOTREAL".to_string() }),
        });
        pedal.nets.push(NetDef {
            from: Pin::ComponentPin {
                component: "J1".to_string(),
                pin: "drain".to_string(),
            },
            to: vec![Pin::Reserved("gnd".to_string())],
        });
        pedal.nets.push(NetDef {
            from: Pin::ComponentPin {
                component: "J1".to_string(),
                pin: "source".to_string(),
            },
            to: vec![Pin::Reserved("gnd".to_string())],
        });
        let warnings = validate_pedal(&pedal);
        assert!(
            has_code(&warnings, "unknown-model"),
            "Invalid JFET 'NOTREAL' should emit unknown-model error"
        );
    }

    #[test]
    fn invalid_pentode_model_name_emits_error() {
        let mut pedal = minimal_pedal();
        pedal.components.push(ComponentDef {
            id: "V1".to_string(),
            kind: Box::new(Pentode { model: "FAKETUBE".to_string() }),
        });
        pedal.nets.push(NetDef {
            from: Pin::ComponentPin {
                component: "V1".to_string(),
                pin: "plate".to_string(),
            },
            to: vec![Pin::Reserved("gnd".to_string())],
        });
        pedal.nets.push(NetDef {
            from: Pin::ComponentPin {
                component: "V1".to_string(),
                pin: "cathode".to_string(),
            },
            to: vec![Pin::Reserved("gnd".to_string())],
        });
        let warnings = validate_pedal(&pedal);
        assert!(
            has_code(&warnings, "unknown-model"),
            "Invalid pentode 'FAKETUBE' should emit unknown-model error"
        );
    }

    #[test]
    fn edit_distance_basic() {
        assert_eq!(super::edit_distance("abc", "abc"), 0);
        assert_eq!(super::edit_distance("abc", "abd"), 1);
        assert_eq!(super::edit_distance("", "abc"), 3);
        assert_eq!(super::edit_distance("kitten", "sitting"), 3);
    }

    #[test]
    fn find_similar_names_suggests_close_matches() {
        let candidates = ["2N3904", "2N3906", "BC184C", "2N5088"];
        let suggestions = super::find_similar_names("2N3904X", &candidates, 3);
        assert!(
            suggestions.contains(&"2N3904"),
            "Should suggest '2N3904' for '2N3904X', got: {:?}",
            suggestions
        );
    }

    // ── Mirror validation tests ─────────────────────────────────────

    fn dual_gang_pedal() -> PedalDef {
        let mut pedal = minimal_pedal();
        pedal.components.push(ComponentDef {
            id: "Gain_A".to_string(),
            kind: Box::new(Potentiometer { max_r: 100_000.0, taper: PotTaper::B }),
        });
        pedal.components.push(ComponentDef {
            id: "Gain_B".to_string(),
            kind: Box::new(Potentiometer { max_r: 100_000.0, taper: PotTaper::B }),
        });
        // Wire pots into the circuit
        pedal.nets.push(NetDef {
            from: Pin::ComponentPin {
                component: "Gain_A".to_string(),
                pin: "a".to_string(),
            },
            to: vec![Pin::Reserved("gnd".to_string())],
        });
        pedal.nets.push(NetDef {
            from: Pin::ComponentPin {
                component: "Gain_B".to_string(),
                pin: "a".to_string(),
            },
            to: vec![Pin::Reserved("gnd".to_string())],
        });
        pedal
            .mirrors
            .insert("Gain_B".to_string(), "Gain_A".to_string());
        pedal
    }

    #[test]
    fn valid_mirror_no_errors() {
        let pedal = dual_gang_pedal();
        let warnings = validate_pedal(&pedal);
        assert!(
            !has_code(&warnings, "mirror-target-missing"),
            "Valid mirror should not trigger errors: {:?}",
            warnings_with_code(&warnings, "mirror-target-missing")
        );
        assert!(!has_code(&warnings, "mirror-target-not-pot"));
        assert!(!has_code(&warnings, "mirror-chain"));
        assert!(!has_code(&warnings, "mirror-in-controls"));
    }

    #[test]
    fn mirror_target_missing() {
        let mut pedal = minimal_pedal();
        pedal
            .mirrors
            .insert("Gain_B".to_string(), "NONEXISTENT".to_string());
        pedal.components.push(ComponentDef {
            id: "Gain_B".to_string(),
            kind: Box::new(Potentiometer { max_r: 100_000.0, taper: PotTaper::B }),
        });
        pedal.nets.push(NetDef {
            from: Pin::ComponentPin {
                component: "Gain_B".to_string(),
                pin: "a".to_string(),
            },
            to: vec![Pin::Reserved("gnd".to_string())],
        });
        let warnings = validate_pedal(&pedal);
        assert!(has_code(&warnings, "mirror-target-missing"));
    }

    #[test]
    fn mirror_target_not_pot() {
        let mut pedal = minimal_pedal();
        // R1 exists but is a resistor, not a pot
        pedal.mirrors.insert("P1".to_string(), "R1".to_string());
        pedal.components.push(ComponentDef {
            id: "P1".to_string(),
            kind: Box::new(Potentiometer { max_r: 100_000.0, taper: PotTaper::B }),
        });
        pedal.nets.push(NetDef {
            from: Pin::ComponentPin {
                component: "P1".to_string(),
                pin: "a".to_string(),
            },
            to: vec![Pin::Reserved("gnd".to_string())],
        });
        let warnings = validate_pedal(&pedal);
        assert!(has_code(&warnings, "mirror-target-not-pot"));
    }

    #[test]
    fn mirror_chain_not_allowed() {
        let mut pedal = dual_gang_pedal();
        // Add a third pot that mirrors Gain_B (which itself mirrors Gain_A)
        pedal.components.push(ComponentDef {
            id: "Gain_C".to_string(),
            kind: Box::new(Potentiometer { max_r: 100_000.0, taper: PotTaper::B }),
        });
        pedal.nets.push(NetDef {
            from: Pin::ComponentPin {
                component: "Gain_C".to_string(),
                pin: "a".to_string(),
            },
            to: vec![Pin::Reserved("gnd".to_string())],
        });
        pedal
            .mirrors
            .insert("Gain_C".to_string(), "Gain_B".to_string());
        let warnings = validate_pedal(&pedal);
        assert!(has_code(&warnings, "mirror-chain"));
    }

    #[test]
    fn mirrored_pot_in_controls_is_error() {
        let mut pedal = dual_gang_pedal();
        // Add Gain_B to controls — this should be an error
        pedal.controls.push(ControlDef {
            component: "Gain_B".to_string(),
            property: "position".to_string(),
            label: "Gain B".to_string(),
            range: (0.0, 1.0),
            default: 0.5,
        });
        let warnings = validate_pedal(&pedal);
        assert!(has_code(&warnings, "mirror-in-controls"));
    }

    #[test]
    fn mirror_resistance_mismatch_warns() {
        let mut pedal = minimal_pedal();
        pedal.components.push(ComponentDef {
            id: "P_A".to_string(),
            kind: Box::new(Potentiometer { max_r: 100_000.0, taper: PotTaper::B }),
        });
        pedal.components.push(ComponentDef {
            id: "P_B".to_string(),
            kind: Box::new(Potentiometer { max_r: 50_000.0, taper: PotTaper::B }), // different!
        });
        pedal.nets.push(NetDef {
            from: Pin::ComponentPin {
                component: "P_A".to_string(),
                pin: "a".to_string(),
            },
            to: vec![Pin::Reserved("gnd".to_string())],
        });
        pedal.nets.push(NetDef {
            from: Pin::ComponentPin {
                component: "P_B".to_string(),
                pin: "a".to_string(),
            },
            to: vec![Pin::Reserved("gnd".to_string())],
        });
        pedal.mirrors.insert("P_B".to_string(), "P_A".to_string());
        let warnings = validate_pedal(&pedal);
        assert!(has_code(&warnings, "mirror-resistance-mismatch"));
    }
}
