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
        match &comp.kind {
            ComponentKind::Resistor(r) => {
                if *r <= 0.0 {
                    w.push(PedalWarning {
                        severity: Severity::Error,
                        code: "invalid-value",
                        message: format!(
                            "Resistor '{}' has non-positive value {:.2} Ω",
                            comp.id, r
                        ),
                    });
                } else if *r < 1.0 {
                    w.push(PedalWarning {
                        severity: Severity::Info,
                        code: "suspicious-value",
                        message: format!(
                            "Resistor '{}' is {:.2} Ω — extremely low, did you mean {}k?",
                            comp.id, r, r
                        ),
                    });
                } else if *r > 100e6 {
                    w.push(PedalWarning {
                        severity: Severity::Info,
                        code: "suspicious-value",
                        message: format!(
                            "Resistor '{}' is {:.1} MΩ — unusually high",
                            comp.id, r / 1e6
                        ),
                    });
                }
            }
            ComponentKind::Capacitor(cfg) => {
                if cfg.value <= 0.0 {
                    w.push(PedalWarning {
                        severity: Severity::Error,
                        code: "invalid-value",
                        message: format!(
                            "Capacitor '{}' has non-positive value {:.2e} F",
                            comp.id, cfg.value
                        ),
                    });
                } else if cfg.value > 1.0 {
                    w.push(PedalWarning {
                        severity: Severity::Warning,
                        code: "suspicious-value",
                        message: format!(
                            "Capacitor '{}' is {:.2} F — over 1 farad, did you forget a unit suffix (n, u, p)?",
                            comp.id, cfg.value
                        ),
                    });
                }
                if let Some(leak) = cfg.leakage {
                    if leak <= 0.0 {
                        w.push(PedalWarning {
                            severity: Severity::Error,
                            code: "invalid-value",
                            message: format!(
                                "Capacitor '{}' has non-positive leakage resistance {:.2} Ω",
                                comp.id, leak
                            ),
                        });
                    }
                }
                if let Some(da) = cfg.da {
                    if !(0.0..=1.0).contains(&da) {
                        w.push(PedalWarning {
                            severity: Severity::Warning,
                            code: "invalid-value",
                            message: format!(
                                "Capacitor '{}' has dielectric absorption {:.3} — should be 0.0 to 1.0",
                                comp.id, da
                            ),
                        });
                    }
                }
            }
            ComponentKind::Inductor(l) => {
                if *l <= 0.0 {
                    w.push(PedalWarning {
                        severity: Severity::Error,
                        code: "invalid-value",
                        message: format!(
                            "Inductor '{}' has non-positive value {:.2e} H",
                            comp.id, l
                        ),
                    });
                }
            }
            ComponentKind::Potentiometer(r) => {
                if *r <= 0.0 {
                    w.push(PedalWarning {
                        severity: Severity::Error,
                        code: "invalid-value",
                        message: format!(
                            "Potentiometer '{}' has non-positive max resistance {:.2} Ω",
                            comp.id, r
                        ),
                    });
                }
            }
            ComponentKind::Zener(v) => {
                if *v <= 0.0 {
                    w.push(PedalWarning {
                        severity: Severity::Error,
                        code: "invalid-value",
                        message: format!(
                            "Zener '{}' has non-positive breakdown voltage {:.2} V",
                            comp.id, v
                        ),
                    });
                }
            }
            ComponentKind::Lfo(_, timing_r, timing_c) => {
                if *timing_r <= 0.0 || *timing_c <= 0.0 {
                    w.push(PedalWarning {
                        severity: Severity::Error,
                        code: "invalid-value",
                        message: format!(
                            "LFO '{}' has non-positive timing values (R={:.2}, C={:.2e})",
                            comp.id, timing_r, timing_c
                        ),
                    });
                }
                let freq = 1.0 / (2.0 * std::f64::consts::PI * timing_r * timing_c);
                if freq > 100.0 {
                    w.push(PedalWarning {
                        severity: Severity::Warning,
                        code: "suspicious-value",
                        message: format!(
                            "LFO '{}' base frequency is {:.1} Hz — unusually fast for modulation",
                            comp.id, freq
                        ),
                    });
                }
            }
            ComponentKind::EnvelopeFollower(ar, ac, rr, rc, sr) => {
                if *ar <= 0.0 || *ac <= 0.0 || *rr <= 0.0 || *rc <= 0.0 || *sr <= 0.0 {
                    w.push(PedalWarning {
                        severity: Severity::Error,
                        code: "invalid-value",
                        message: format!(
                            "EnvelopeFollower '{}' has non-positive parameter value(s)",
                            comp.id
                        ),
                    });
                }
            }
            ComponentKind::Switch(n) => {
                if *n < 2 {
                    w.push(PedalWarning {
                        severity: Severity::Warning,
                        code: "suspicious-value",
                        message: format!(
                            "Switch '{}' has {} position(s) — a switch needs at least 2",
                            comp.id, n
                        ),
                    });
                }
            }
            _ => {}
        }
    }
}

/// Nets referencing component IDs that don't exist.
fn check_net_references(pedal: &PedalDef, w: &mut Vec<PedalWarning>) {
    let comp_ids: HashSet<&str> = pedal.components.iter().map(|c| c.id.as_str()).collect();
    let reserved: HashSet<&str> = ["in", "out", "gnd", "vcc", "fx_send", "fx_return"]
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
            Pin::Fork { switch, destinations } => {
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
        check_pin_ref(&net.from, &comp_ids, &reserved, &supply_names, &supply_list, w);
        for to_pin in &net.to {
            check_pin_ref(to_pin, &comp_ids, &reserved, &supply_names, &supply_list, w);
        }
    }
}

/// Valid pin names for each component type.
fn valid_pins_for(kind: &ComponentKind) -> &'static [&'static str] {
    match kind {
        // 2-terminal passive elements
        ComponentKind::Resistor(_)
        | ComponentKind::Capacitor(_)
        | ComponentKind::Inductor(_)
        | ComponentKind::DiodePair(_)
        | ComponentKind::Diode(_)
        | ComponentKind::Zener(_)
        | ComponentKind::Neon(_)
        | ComponentKind::Tempco(_, _)
        | ComponentKind::CapSwitched(_)
        | ComponentKind::InductorSwitched(_)
        | ComponentKind::ResistorSwitched(_) => &["a", "b"],

        // Potentiometer: 2-terminal (a, b) or 3-terminal (a, wiper/w, b)
        ComponentKind::Potentiometer(_) => &["a", "b", "w", "wiper", "position"],

        // BJT
        ComponentKind::Npn(_) | ComponentKind::Pnp(_) => {
            &["base", "collector", "emitter"]
        }
        ComponentKind::MatchedNpn(_) | ComponentKind::MatchedPnp(_) => {
            &["base", "collector", "emitter", "base1", "base2", "collector1", "collector2", "emitter1", "emitter2"]
        }

        // Op-amp
        ComponentKind::OpAmp(_) => &["pos", "neg", "out", "vp", "in"],

        // JFET
        ComponentKind::NJfet(_) | ComponentKind::PJfet(_) => {
            &["gate", "drain", "source", "vgs"]
        }

        // MOSFET
        ComponentKind::Nmos(_) | ComponentKind::Pmos(_) => {
            &["gate", "drain", "source", "vgs"]
        }

        // Photocoupler: LDR (a, b) + LED drive
        ComponentKind::Photocoupler(_) => &["a", "b", "led"],

        // Triode tube
        ComponentKind::Triode(_) => &["plate", "cathode", "grid", "vgk"],

        // Pentode tube
        ComponentKind::Pentode(_) => &["plate", "cathode", "g1", "g2", "grid", "screen", "vg1k"],

        // LFO: output + rate control
        ComponentKind::Lfo(..) => &["out", "rate"],

        // Envelope follower: output + input
        ComponentKind::EnvelopeFollower(..) => &["out", "in"],

        // BBD delay
        ComponentKind::Bbd(_) => &["in", "out", "clock"],

        // Delay line
        ComponentKind::DelayLine(..) => &["input", "output", "rate", "speed_mod", "delay_time", "feedback"],

        // Tap
        ComponentKind::Tap(..) => &["output"],

        // Transformer: primary (a, b) + secondary (c, d) + optional center taps
        ComponentKind::Transformer(_) => {
            &["a", "b", "c", "d", "pri_a", "pri_b", "sec_a", "sec_b", "pri_ct", "sec_ct", "ct"]
        }

        // Synth ICs
        ComponentKind::Vco(_) => &["cv", "saw", "tri", "pulse", "sync", "out"],
        ComponentKind::Vcf(_) => &["in", "out", "cv", "res"],
        ComponentKind::Vca(_) => &["in", "out", "cv"],
        ComponentKind::Comparator(_) => &["pos", "neg", "out"],
        ComponentKind::AnalogSwitch(_) => {
            &["in1", "out1", "ctrl1", "in2", "out2", "ctrl2",
              "in3", "out3", "ctrl3", "in4", "out4", "ctrl4"]
        }

        // Switches are control elements, not circuit elements with pins
        ComponentKind::RotarySwitch(_) | ComponentKind::Switch(_) => &[],
    }
}

/// Check that pin names used in nets are valid for their component type.
fn check_pin_validity(pedal: &PedalDef, w: &mut Vec<PedalWarning>) {
    let comp_map: HashMap<&str, &ComponentKind> = pedal
        .components
        .iter()
        .map(|c| (c.id.as_str(), &c.kind))
        .collect();

    let check_pin = |pin: &Pin, w: &mut Vec<PedalWarning>| {
        if let Pin::ComponentPin { component, pin } = pin {
            if let Some(kind) = comp_map.get(component.as_str()) {
                let valid = valid_pins_for(kind);
                // Don't warn for component types that have no pin list
                // (switches, etc.) since they're not circuit elements
                if !valid.is_empty() && !valid.contains(&pin.as_str()) {
                    w.push(PedalWarning {
                        severity: Severity::Warning,
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
            Pin::Fork { switch, destinations } => {
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
        if matches!(
            &comp.kind,
            ComponentKind::RotarySwitch(_) | ComponentKind::Switch(_)
        ) {
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
    // Build adjacency based on net connections (node = pin key, edges = same net).
    let mut adj: HashMap<String, HashSet<String>> = HashMap::new();

    let pin_key = |pin: &Pin| -> String {
        match pin {
            Pin::Reserved(s) => s.clone(),
            Pin::ComponentPin { component, pin } => format!("{}.{}", component, pin),
            Pin::Fork { switch, .. } => format!("__fork_{}", switch),
        }
    };

    // Also add implicit component-internal connections:
    // For a 2-terminal element, .a and .b are connected through the element.
    for comp in &pedal.components {
        let pins = valid_pins_for(&comp.kind);
        // For circuit elements, connect their terminal pairs
        match &comp.kind {
            ComponentKind::Resistor(_)
            | ComponentKind::Capacitor(_)
            | ComponentKind::Inductor(_)
            | ComponentKind::DiodePair(_)
            | ComponentKind::Diode(_)
            | ComponentKind::Zener(_)
            | ComponentKind::Neon(_)
            | ComponentKind::Tempco(_, _)
            | ComponentKind::CapSwitched(_)
            | ComponentKind::InductorSwitched(_)
            | ComponentKind::ResistorSwitched(_) => {
                let ka = format!("{}.a", comp.id);
                let kb = format!("{}.b", comp.id);
                adj.entry(ka.clone()).or_default().insert(kb.clone());
                adj.entry(kb).or_default().insert(ka);
            }
            ComponentKind::Potentiometer(_) => {
                // a-wiper and wiper-b are connected through the pot
                let ka = format!("{}.a", comp.id);
                let kb = format!("{}.b", comp.id);
                let kw = format!("{}.wiper", comp.id);
                let kw2 = format!("{}.w", comp.id);
                adj.entry(ka.clone()).or_default().insert(kb.clone());
                adj.entry(kb.clone()).or_default().insert(ka.clone());
                adj.entry(ka.clone()).or_default().insert(kw.clone());
                adj.entry(kw.clone()).or_default().insert(ka);
                adj.entry(kb.clone()).or_default().insert(kw2.clone());
                adj.entry(kw2).or_default().insert(kb);
            }
            ComponentKind::NJfet(_) | ComponentKind::PJfet(_)
            | ComponentKind::Nmos(_) | ComponentKind::Pmos(_) => {
                let kd = format!("{}.drain", comp.id);
                let ks = format!("{}.source", comp.id);
                adj.entry(kd.clone()).or_default().insert(ks.clone());
                adj.entry(ks).or_default().insert(kd);
            }
            ComponentKind::Triode(_) | ComponentKind::Pentode(_) => {
                let kp = format!("{}.plate", comp.id);
                let kk = format!("{}.cathode", comp.id);
                adj.entry(kp.clone()).or_default().insert(kk.clone());
                adj.entry(kk).or_default().insert(kp);
            }
            ComponentKind::Npn(_) | ComponentKind::Pnp(_) => {
                let kc = format!("{}.collector", comp.id);
                let ke = format!("{}.emitter", comp.id);
                adj.entry(kc.clone()).or_default().insert(ke.clone());
                adj.entry(ke).or_default().insert(kc);
            }
            ComponentKind::OpAmp(_) => {
                // Op-amp: pos/neg are inputs, out is output.
                // For signal path check, connect through the op-amp.
                let kp = format!("{}.pos", comp.id);
                let kn = format!("{}.neg", comp.id);
                let ko = format!("{}.out", comp.id);
                adj.entry(kp.clone()).or_default().insert(ko.clone());
                adj.entry(kn.clone()).or_default().insert(ko.clone());
                adj.entry(ko.clone()).or_default().insert(kp);
                adj.entry(ko).or_default().insert(kn);
            }
            ComponentKind::Photocoupler(_) => {
                let ka = format!("{}.a", comp.id);
                let kb = format!("{}.b", comp.id);
                adj.entry(ka.clone()).or_default().insert(kb.clone());
                adj.entry(kb).or_default().insert(ka);
            }
            ComponentKind::Bbd(_) => {
                let ki = format!("{}.in", comp.id);
                let ko = format!("{}.out", comp.id);
                adj.entry(ki.clone()).or_default().insert(ko.clone());
                adj.entry(ko).or_default().insert(ki);
            }
            ComponentKind::DelayLine(..) => {
                let ki = format!("{}.input", comp.id);
                let ko = format!("{}.output", comp.id);
                adj.entry(ki.clone()).or_default().insert(ko.clone());
                adj.entry(ko).or_default().insert(ki);
            }
            ComponentKind::Transformer(_) => {
                // Primary and secondary are magnetically coupled
                let ka = format!("{}.a", comp.id);
                let kb = format!("{}.b", comp.id);
                let kc = format!("{}.c", comp.id);
                let kd = format!("{}.d", comp.id);
                adj.entry(ka.clone()).or_default().insert(kb.clone());
                adj.entry(kb).or_default().insert(ka);
                adj.entry(kc.clone()).or_default().insert(kd.clone());
                adj.entry(kd).or_default().insert(kc);
            }
            // LFO, EnvelopeFollower, Switches, Taps, Synth ICs — not passive signal path
            _ => {
                // Connect any declared pins to ensure adjacency
                if pins.len() >= 2 {
                    let first = format!("{}.{}", comp.id, pins[0]);
                    for p in &pins[1..] {
                        let other = format!("{}.{}", comp.id, p);
                        adj.entry(first.clone()).or_default().insert(other.clone());
                        adj.entry(other).or_default().insert(first.clone());
                    }
                }
            }
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
    let comp_map: HashMap<&str, &ComponentKind> = pedal
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

        let kind = comp_map[ctrl.component.as_str()];
        // Controls typically target potentiometers or switches
        if !matches!(
            kind,
            ComponentKind::Potentiometer(_)
                | ComponentKind::Switch(_)
                | ComponentKind::RotarySwitch(_)
        ) {
            w.push(PedalWarning {
                severity: Severity::Warning,
                code: "unusual-control-target",
                message: format!(
                    "{} entry '{}' targets '{}' which is a {:?} — expected a pot or switch",
                    section, ctrl.label, ctrl.component,
                    component_type_name(kind)
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
    let comp_map: HashMap<&str, &ComponentKind> = pedal
        .components
        .iter()
        .map(|c| (c.id.as_str(), &c.kind))
        .collect();

    // Valid modulation target pins that the compiler recognizes
    let valid_mod_targets: HashSet<&str> =
        ["vgs", "gate", "led", "vgk", "vg1k", "iabc", "clock", "speed_mod", "delay_time"]
            .iter()
            .copied()
            .collect();

    // Pins that are "destination" pins on passive components — these are valid net
    // connections but NOT modulation targets. When an LFO connects to, e.g., Depth.a,
    // the signal flows through the passive network but doesn't create a modulation binding.
    let passive_connection_pins: HashSet<&str> =
        ["a", "b", "w", "wiper", "in", "out", "rate"]
            .iter()
            .copied()
            .collect();

    // Check each modulation source's output connections
    for comp in &pedal.components {
        let is_mod_source = matches!(
            &comp.kind,
            ComponentKind::Lfo(..) | ComponentKind::EnvelopeFollower(..)
        );
        if !is_mod_source {
            continue;
        }

        let source_type = if matches!(&comp.kind, ComponentKind::Lfo(..)) {
            "LFO"
        } else {
            "EnvelopeFollower"
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

                            let target_kind = comp_map[target_comp.as_str()];

                            // If the target pin is a known modulation target, great
                            if valid_mod_targets.contains(target_prop.as_str()) {
                                // Verify the target component type makes sense for this pin
                                check_mod_target_compatibility(
                                    source_type, &comp.id, target_comp, target_prop,
                                    target_kind, w,
                                );
                            } else if passive_connection_pins.contains(target_prop.as_str()) {
                                // This is a valid passive connection (e.g., LFO1.out -> Depth.a)
                                // but it won't create a modulation binding. Check if the user
                                // probably intended a modulation binding.
                                let target_is_pot = matches!(target_kind, ComponentKind::Potentiometer(_));
                                let target_is_passive = matches!(
                                    target_kind,
                                    ComponentKind::Resistor(_)
                                        | ComponentKind::Capacitor(_)
                                        | ComponentKind::Inductor(_)
                                );
                                if !target_is_pot && !target_is_passive
                                    && !matches!(target_kind, ComponentKind::Lfo(..)) {
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
    target_kind: &ComponentKind,
    w: &mut Vec<PedalWarning>,
) {
    let mismatch = match target_pin {
        "vgs" | "gate" => !matches!(
            target_kind,
            ComponentKind::NJfet(_) | ComponentKind::PJfet(_)
                | ComponentKind::Nmos(_) | ComponentKind::Pmos(_)
        ),
        "led" => !matches!(target_kind, ComponentKind::Photocoupler(_)),
        "vgk" => !matches!(target_kind, ComponentKind::Triode(_)),
        "vg1k" => !matches!(target_kind, ComponentKind::Pentode(_)),
        "iabc" => !matches!(target_kind, ComponentKind::OpAmp(OpAmpType::Ca3080)),
        "clock" => !matches!(target_kind, ComponentKind::Bbd(_)),
        "speed_mod" | "delay_time" => !matches!(target_kind, ComponentKind::DelayLine(..)),
        _ => false,
    };

    if mismatch {
        w.push(PedalWarning {
            severity: Severity::Warning,
            code: "mod-target-type-mismatch",
            message: format!(
                "{} '{}'.out -> '{}.{}': pin '{}' is typically used with {} components, \
                 but '{}' is a {}",
                source_type, source_id, target_comp, target_pin, target_pin,
                expected_component_for_pin(target_pin),
                target_comp, component_type_name(target_kind)
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

fn component_type_name(kind: &ComponentKind) -> &'static str {
    match kind {
        ComponentKind::Resistor(_) => "resistor",
        ComponentKind::Capacitor(_) => "capacitor",
        ComponentKind::Inductor(_) => "inductor",
        ComponentKind::DiodePair(_) => "diode pair",
        ComponentKind::Diode(_) => "diode",
        ComponentKind::Zener(_) => "zener diode",
        ComponentKind::Potentiometer(_) => "potentiometer",
        ComponentKind::Npn(_) => "NPN transistor",
        ComponentKind::Pnp(_) => "PNP transistor",
        ComponentKind::OpAmp(ot) if ot.is_ota() => "OTA",
        ComponentKind::OpAmp(_) => "op-amp",
        ComponentKind::NJfet(_) => "N-channel JFET",
        ComponentKind::PJfet(_) => "P-channel JFET",
        ComponentKind::Nmos(_) => "N-channel MOSFET",
        ComponentKind::Pmos(_) => "P-channel MOSFET",
        ComponentKind::Photocoupler(_) => "photocoupler",
        ComponentKind::Triode(_) => "triode",
        ComponentKind::Pentode(_) => "pentode",
        ComponentKind::Lfo(..) => "LFO",
        ComponentKind::EnvelopeFollower(..) => "envelope follower",
        ComponentKind::Bbd(_) => "BBD delay",
        ComponentKind::DelayLine(..) => "delay line",
        ComponentKind::Tap(..) => "tap",
        ComponentKind::Neon(_) => "neon bulb",
        ComponentKind::Vco(_) => "VCO",
        ComponentKind::Vcf(_) => "VCF",
        ComponentKind::Vca(_) => "VCA",
        ComponentKind::Comparator(_) => "comparator",
        ComponentKind::AnalogSwitch(_) => "analog switch",
        ComponentKind::MatchedNpn(_) => "matched NPN pair",
        ComponentKind::MatchedPnp(_) => "matched PNP pair",
        ComponentKind::Tempco(_, _) => "tempco resistor",
        ComponentKind::Transformer(_) => "transformer",
        ComponentKind::CapSwitched(_) => "switched capacitor",
        ComponentKind::InductorSwitched(_) => "switched inductor",
        ComponentKind::ResistorSwitched(_) => "switched resistor",
        ComponentKind::RotarySwitch(_) => "rotary switch",
        ComponentKind::Switch(_) => "switch",
    }
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
                message: format!("Supply rail name '{}' is declared more than once", supply.name),
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

// ═══════════════════════════════════════════════════════════════════════════
// Tests
// ═══════════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a minimal valid pedal for testing.
    fn minimal_pedal() -> PedalDef {
        PedalDef {
            name: "Test".to_string(),
            supplies: vec![],
            components: vec![
                ComponentDef {
                    id: "R1".to_string(),
                    kind: ComponentKind::Resistor(10_000.0),
                },
            ],
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
            kind: ComponentKind::Resistor(1000.0),
        });
        let warnings = validate_pedal(&pedal);
        assert!(has_code(&warnings, "duplicate-component-id"));
    }

    #[test]
    fn zero_resistance() {
        let mut pedal = minimal_pedal();
        pedal.components[0].kind = ComponentKind::Resistor(0.0);
        let warnings = validate_pedal(&pedal);
        assert!(has_code(&warnings, "invalid-value"));
    }

    #[test]
    fn very_large_capacitor_warns() {
        let mut pedal = minimal_pedal();
        pedal.components.push(ComponentDef {
            id: "C1".to_string(),
            kind: ComponentKind::Capacitor(CapConfig::new(100.0)), // 100 F!
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
            kind: ComponentKind::Resistor(10_000.0),
        });
        let warnings = validate_pedal(&pedal);
        assert!(has_code(&warnings, "orphaned-component"));
    }

    #[test]
    fn no_signal_path() {
        let pedal = PedalDef {
            name: "Broken".to_string(),
            supplies: vec![],
            components: vec![
                ComponentDef {
                    id: "R1".to_string(),
                    kind: ComponentKind::Resistor(10_000.0),
                },
            ],
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
            kind: ComponentKind::Potentiometer(100_000.0),
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
            kind: ComponentKind::Lfo(LfoWaveformDsl::Triangle, 150_000.0, 1e-6),
        });
        pedal.components.push(ComponentDef {
            id: "Depth".to_string(),
            kind: ComponentKind::Potentiometer(50_000.0),
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
            kind: ComponentKind::Lfo(LfoWaveformDsl::Sine, 100_000.0, 1e-6),
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
            kind: ComponentKind::Lfo(LfoWaveformDsl::Triangle, 1_000_000.0, 220e-9),
        });
        pedal.components.push(ComponentDef {
            id: "J1".to_string(),
            kind: ComponentKind::NJfet(JfetType::N2n5952),
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
            kind: ComponentKind::Lfo(LfoWaveformDsl::Triangle, 150_000.0, 1e-6),
        });
        pedal.components.push(ComponentDef {
            id: "Depth".to_string(),
            kind: ComponentKind::Potentiometer(50_000.0),
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
            kind: ComponentKind::Potentiometer(100_000.0),
        });
        pedal.components.push(ComponentDef {
            id: "P2".to_string(),
            kind: ComponentKind::Potentiometer(100_000.0),
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
}
