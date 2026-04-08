//! Subcircuit resolution and routing for equipment-level circuit partitioning.
//!
//! Provides `resolve_subcircuits()` to convert `SubcircuitDef` blocks into
//! compilable `ResolvedSubcircuit` sub-`PedalDef`s, and `build_routing()` to
//! produce a topologically-sorted signal graph connecting them.

use crate::dsl::{PedalDef, Pin, SubcircuitDef};
use std::collections::{HashMap, HashSet};

/// A resolved subcircuit ready for separate compilation.
pub struct ResolvedSubcircuit {
    /// Subcircuit name.
    pub name: String,
    /// Inferred port names.
    pub ports: Vec<String>,
    /// Sub-`PedalDef` ready for `compile_pedal_with_options()`.
    pub pedal_def: PedalDef,
    /// Decimation factor (1 = full rate).
    pub rate_divisor: u32,
}

/// Routing step in the inter-subcircuit signal graph.
#[derive(Debug, Clone)]
pub enum RoutingStep {
    /// Process subcircuit at index, feeding it signal from source.
    ProcessSubcircuit {
        subcircuit_idx: usize,
        input_source: SignalSource,
    },
}

/// Where a subcircuit gets its input signal.
#[derive(Debug, Clone)]
pub enum SignalSource {
    /// The equipment-level input.
    EquipmentInput,
    /// Output of subcircuit at given index.
    SubcircuitOutput(usize),
    /// Sum of multiple sources (fan-in).
    /// Currently unused but reserved for multi-input routing in a future pass.
    #[allow(dead_code)]
    Sum(Vec<SignalSource>),
}

/// Resolve subcircuit definitions into compilable sub-`PedalDef`s.
///
/// Infers port names from node names in subcircuit nets that do not resolve
/// to component pins or reserved nodes (`gnd`, `vcc`, supply rails).
/// `in` and `out` are always treated as ports when used inside a subcircuit.
pub fn resolve_subcircuits(pedal: &PedalDef) -> Result<Vec<ResolvedSubcircuit>, String> {
    if pedal.subcircuits.is_empty() {
        return Ok(Vec::new());
    }

    let supply_rails: HashSet<String> = pedal.supplies.iter().map(|s| s.name.clone()).collect();
    let reserved = ["gnd", "vcc", "in", "out"];

    let mut resolved = Vec::new();

    for sc in &pedal.subcircuits {
        let ports = infer_ports(sc, &supply_rails, &reserved);

        // Build sub-PedalDef
        let sub_def = PedalDef {
            name: format!("{}::{}", pedal.name, sc.name),
            subtitle: None,
            supplies: pedal.supplies.clone(),
            components: sc.components.clone(),
            nets: sc.nets.clone(),
            controls: sc.controls.clone(),
            trims: sc.trims.clone(),
            monitors: sc.monitors.clone(),
            sidechains: Vec::new(),
            subcircuits: Vec::new(),
            mirrors: sc.mirrors.clone(),
            midi_bindings: sc.midi_bindings.clone(),
            calibrate: false,
        };

        resolved.push(ResolvedSubcircuit {
            name: sc.name.clone(),
            ports,
            pedal_def: sub_def,
            rate_divisor: sc.rate.unwrap_or(1),
        });
    }

    // Validate: check parent nets reference valid ports
    validate_routing(pedal, &resolved)?;

    Ok(resolved)
}

/// Infer the port names of a subcircuit from its net declarations.
///
/// Ports are node names that appear in subcircuit nets but are not:
/// - component pin IDs
/// - reserved words (gnd, vcc)
/// - supply rail names
///
/// `in` and `out` are always inserted first/last when the subcircuit
/// uses them, because they become interface points with the parent routing layer.
fn infer_ports(
    sc: &SubcircuitDef,
    supply_rails: &HashSet<String>,
    reserved: &[&str],
) -> Vec<String> {
    let component_ids: HashSet<String> = sc.components.iter().map(|c| c.id.clone()).collect();
    let mut ports: Vec<String> = Vec::new();
    let mut seen_ports: HashSet<String> = HashSet::new();

    for net in &sc.nets {
        for pin in std::iter::once(&net.from).chain(net.to.iter()) {
            if let Pin::Reserved(name) = pin {
                // Skip standard reserved nodes — they are not ports
                if name == "gnd" || name == "vcc" || supply_rails.contains(name) {
                    continue;
                }
                // `in` and `out` are handled separately below
                if name == "in" || name == "out" {
                    continue;
                }
                // Any other name that is not a component ID is an inferred port
                if !component_ids.contains(name) && !seen_ports.contains(name) {
                    // Only expose as a port if it's not a standard reserved keyword
                    if !reserved.contains(&name.as_str()) {
                        ports.push(name.clone());
                        seen_ports.insert(name.clone());
                    }
                }
            }
        }
    }

    // `in` and `out` are always ports when used inside the subcircuit
    let uses_in = sc.nets.iter().any(|n| {
        std::iter::once(&n.from)
            .chain(n.to.iter())
            .any(|p| matches!(p, Pin::Reserved(s) if s == "in"))
    });
    let uses_out = sc.nets.iter().any(|n| {
        std::iter::once(&n.from)
            .chain(n.to.iter())
            .any(|p| matches!(p, Pin::Reserved(s) if s == "out"))
    });

    // Insert `in` at front, `out` at end (conventional ordering)
    let mut ordered = Vec::new();
    if uses_in {
        ordered.push("in".to_string());
    }
    ordered.extend(ports);
    if uses_out {
        ordered.push("out".to_string());
    }

    ordered
}

/// Build the routing order from parent nets (topological sort).
///
/// Returns `(steps, output_subcircuit_idx)` where `output_subcircuit_idx` is
/// the index into `resolved` of the subcircuit whose output reaches `out`.
pub fn build_routing(
    pedal: &PedalDef,
    resolved: &[ResolvedSubcircuit],
) -> Result<(Vec<RoutingStep>, usize), String> {
    // Map subcircuit names → indices
    let name_to_idx: HashMap<&str, usize> = resolved
        .iter()
        .enumerate()
        .map(|(i, r)| (r.name.as_str(), i))
        .collect();

    let mut output_sc_idx: Option<usize> = None;
    let mut sc_inputs: HashMap<usize, SignalSource> = HashMap::new();

    for net in &pedal.nets {
        // Determine source signal
        let source = match &net.from {
            Pin::Reserved(n) if n == "in" => SignalSource::EquipmentInput,
            Pin::SubcircuitPort { subcircuit, port } if port == "out" => {
                match name_to_idx.get(subcircuit.as_str()) {
                    Some(&idx) => SignalSource::SubcircuitOutput(idx),
                    None => return Err(format!("unknown subcircuit '{}' in routing", subcircuit)),
                }
            }
            // Skip non-routing net origins (component pins, gnd, etc.)
            _ => continue,
        };

        for dest in &net.to {
            match dest {
                Pin::Reserved(n) if n == "out" => {
                    // Source feeds the equipment output — track which subcircuit
                    if let SignalSource::SubcircuitOutput(idx) = &source {
                        output_sc_idx = Some(*idx);
                    }
                }
                Pin::SubcircuitPort { subcircuit, port } if port == "in" => {
                    if let Some(&idx) = name_to_idx.get(subcircuit.as_str()) {
                        sc_inputs.insert(idx, source.clone());
                    }
                }
                // Non-standard ports (ctrl, etc.) — reserved for Phase 2 control routing
                Pin::SubcircuitPort { .. } => {}
                _ => {}
            }
        }
    }

    let output_idx = output_sc_idx
        .ok_or_else(|| "no subcircuit output connected to equipment 'out'".to_string())?;

    // Topological sort — visit in dependency order
    let mut steps: Vec<RoutingStep> = Vec::new();
    let mut processed: HashSet<usize> = HashSet::new();

    // Visit the output subcircuit first (it will pull in its dependencies recursively)
    topo_visit(output_idx, &sc_inputs, &mut steps, &mut processed);

    // Visit any remaining subcircuits (side-effect paths, unused for output)
    for i in 0..resolved.len() {
        topo_visit(i, &sc_inputs, &mut steps, &mut processed);
    }

    Ok((steps, output_idx))
}

fn topo_visit(
    idx: usize,
    sc_inputs: &HashMap<usize, SignalSource>,
    steps: &mut Vec<RoutingStep>,
    processed: &mut HashSet<usize>,
) {
    if processed.contains(&idx) {
        return;
    }

    // Visit dependencies before this node
    if let Some(source) = sc_inputs.get(&idx) {
        visit_source_deps(source, sc_inputs, steps, processed);
    }

    processed.insert(idx);
    steps.push(RoutingStep::ProcessSubcircuit {
        subcircuit_idx: idx,
        input_source: sc_inputs
            .get(&idx)
            .cloned()
            .unwrap_or(SignalSource::EquipmentInput),
    });
}

fn visit_source_deps(
    source: &SignalSource,
    sc_inputs: &HashMap<usize, SignalSource>,
    steps: &mut Vec<RoutingStep>,
    processed: &mut HashSet<usize>,
) {
    match source {
        SignalSource::SubcircuitOutput(idx) => {
            topo_visit(*idx, sc_inputs, steps, processed);
        }
        SignalSource::Sum(sources) => {
            for s in sources {
                visit_source_deps(s, sc_inputs, steps, processed);
            }
        }
        SignalSource::EquipmentInput => {}
    }
}

fn validate_routing(pedal: &PedalDef, resolved: &[ResolvedSubcircuit]) -> Result<(), String> {
    let name_to_sc: HashMap<&str, &ResolvedSubcircuit> =
        resolved.iter().map(|r| (r.name.as_str(), r)).collect();

    // Check all SubcircuitPort references in parent nets
    for net in &pedal.nets {
        for pin in std::iter::once(&net.from).chain(net.to.iter()) {
            if let Pin::SubcircuitPort { subcircuit, port } = pin {
                let sc = name_to_sc.get(subcircuit.as_str()).ok_or_else(|| {
                    format!(
                        "no subcircuit named '{}' — referenced in top-level nets",
                        subcircuit
                    )
                })?;

                // `in` and `out` are always valid ports; other ports must be inferred
                if port != "in" && port != "out" && !sc.ports.contains(port) {
                    return Err(format!(
                        "no port '{}' on subcircuit '{}' — available ports: {}",
                        port,
                        subcircuit,
                        sc.ports
                            .iter()
                            .map(|s| s.as_str())
                            .collect::<Vec<_>>()
                            .join(", ")
                    ));
                }
            }
        }
    }

    // Rate divisors must be powers of two (parser validates, but double-check)
    for r in resolved {
        let d = r.rate_divisor;
        if d > 1 && (d & (d - 1)) != 0 {
            return Err(format!(
                "subcircuit '{}' rate divisor {} is not a power of two",
                r.name, d
            ));
        }
    }

    Ok(())
}

// ═══════════════════════════════════════════════════════════════════════════
// Tests
// ═══════════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dsl::parse_pedal_file;

    #[test]
    fn resolve_basic_subcircuits() {
        let src = r#"
equipment "Test" {
    supply 9V
    subcircuit audio {
        components { R1: resistor(1k) }
        nets { in -> R1.a  R1.b -> out }
    }
    nets { in -> audio.in  audio.out -> out }
}"#;
        let pedal = parse_pedal_file(src).unwrap();
        let resolved = resolve_subcircuits(&pedal).unwrap();
        assert_eq!(resolved.len(), 1);
        assert_eq!(resolved[0].name, "audio");
        assert!(
            resolved[0].ports.contains(&"in".to_string()),
            "expected 'in' port, got: {:?}",
            resolved[0].ports
        );
        assert!(
            resolved[0].ports.contains(&"out".to_string()),
            "expected 'out' port, got: {:?}",
            resolved[0].ports
        );
    }

    #[test]
    fn routing_two_subcircuits() {
        let src = r#"
equipment "Test" {
    supply 9V
    subcircuit sidechain {
        rate: 1/64
        components { R1: resistor(10k) }
        nets { in -> R1.a  R1.b -> out }
    }
    subcircuit audio {
        components { R2: resistor(1k) }
        nets { in -> R2.a  R2.b -> out }
    }
    nets {
        in -> audio.in, sidechain.in
        audio.out -> out
    }
}"#;
        let pedal = parse_pedal_file(src).unwrap();
        let resolved = resolve_subcircuits(&pedal).unwrap();
        let (routing, output_idx) = build_routing(&pedal, &resolved).unwrap();
        assert_eq!(
            resolved[output_idx].name, "audio",
            "output subcircuit should be 'audio'"
        );
        assert!(
            routing.len() >= 2,
            "both subcircuits should appear in routing"
        );
    }

    #[test]
    fn resolve_empty_subcircuits_returns_empty() {
        let src = r#"
pedal "Simple" {
    supply 9V
    components { R1: resistor(1k) }
    nets { in -> R1.a  R1.b -> out }
}"#;
        let pedal = parse_pedal_file(src).unwrap();
        let resolved = resolve_subcircuits(&pedal).unwrap();
        assert!(resolved.is_empty());
    }

    #[test]
    fn validate_rejects_unknown_subcircuit_reference() {
        // Build a minimal PedalDef with a SubcircuitPort referencing a nonexistent subcircuit
        let src = r#"
equipment "Test" {
    supply 9V
    subcircuit audio {
        components { R1: resistor(1k) }
        nets { in -> R1.a  R1.b -> out }
    }
    nets { in -> audio.in  audio.out -> out }
}"#;
        let mut pedal = parse_pedal_file(src).unwrap();
        // Inject a bad reference
        pedal.nets.push(crate::dsl::NetDef {
            from: Pin::SubcircuitPort {
                subcircuit: "nonexistent".to_string(),
                port: "in".to_string(),
            },
            to: vec![Pin::Reserved("out".to_string())],
        });

        let resolved_ok = resolve_subcircuits(&pedal);
        // validate_routing is called inside resolve_subcircuits; it should fail
        assert!(
            resolved_ok.is_err(),
            "should reject reference to nonexistent subcircuit"
        );
    }
}
