//! Phase 2: Functional group detection.
//!
//! Automatically identifies circuit subcircuits by pattern matching on the
//! layout graph. Groups include gain stages, tone stacks, push-pull outputs,
//! phase inverters, and generic groups for unmatched components.

use crate::graph::LayoutGraph;
use pedalkernel::dsl::ComponentKind;
use std::collections::HashSet;

/// The type of functional group detected.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum GroupKind {
    /// Triode/BJT/JFET gain stage: active device + plate/collector load +
    /// optional cathode/emitter bias + coupling caps.
    GainStage,
    /// Tone stack: cluster of pots + caps + resistors between gain stages.
    ToneStack,
    /// Push-pull output: 2 or 4 pentodes driving a transformer.
    PushPullOutput,
    /// Phase inverter: two triodes sharing a tail resistor.
    PhaseInverter,
    /// Op-amp stage: op-amp with feedback network.
    OpAmpStage,
    /// Input section: components connected directly to the input.
    InputSection,
    /// Output section: components connected directly to the output.
    OutputSection,
    /// Generic group for components that don't match a specific pattern.
    Generic,
}

/// A detected functional group.
#[derive(Debug, Clone)]
pub struct FunctionalGroup {
    /// Unique group name (e.g., `"gain_stage_1"`, `"tone_stack"`).
    pub name: String,
    /// Human-readable label (e.g., `"V1a Preamp"`, `"Tone Stack"`).
    pub label: String,
    /// Group type.
    pub kind: GroupKind,
    /// Node indices (in the LayoutGraph) belonging to this group.
    pub members: Vec<usize>,
    /// The "primary" active device node index (if any).
    pub primary_device: Option<usize>,
}

/// Detect all functional groups in the layout graph.
pub fn detect_groups(graph: &LayoutGraph) -> Vec<FunctionalGroup> {
    let mut assigned: HashSet<usize> = HashSet::new();
    let mut groups = Vec::new();

    // Mark anchor nodes as assigned (they don't belong to groups)
    assigned.insert(graph.in_node);
    assigned.insert(graph.out_node);
    assigned.insert(graph.vcc_node);
    assigned.insert(graph.gnd_node);

    // Detect gain stages (triodes, BJTs, JFETs with surrounding passives)
    let gain_stages = detect_gain_stages(graph, &assigned);
    for gs in &gain_stages {
        for &m in &gs.members {
            assigned.insert(m);
        }
    }
    groups.extend(gain_stages);

    // Detect op-amp stages
    let opamp_stages = detect_opamp_stages(graph, &assigned);
    for os in &opamp_stages {
        for &m in &os.members {
            assigned.insert(m);
        }
    }
    groups.extend(opamp_stages);

    // Detect push-pull output sections
    let pp = detect_push_pull(graph, &assigned);
    for g in &pp {
        for &m in &g.members {
            assigned.insert(m);
        }
    }
    groups.extend(pp);

    // Detect tone stacks (clusters of pots + passives with no active devices)
    let tone_stacks = detect_tone_stacks(graph, &assigned);
    for ts in &tone_stacks {
        for &m in &ts.members {
            assigned.insert(m);
        }
    }
    groups.extend(tone_stacks);

    // Assign remaining components to input/output/generic groups
    let remaining = assign_remaining(graph, &assigned);
    groups.extend(remaining);

    groups
}

// ---------------------------------------------------------------------------
// Gain stage detection
// ---------------------------------------------------------------------------

fn detect_gain_stages(graph: &LayoutGraph, assigned: &HashSet<usize>) -> Vec<FunctionalGroup> {
    let mut groups = Vec::new();
    let mut stage_num = 0;
    let mut locally_assigned: HashSet<usize> = HashSet::new();

    for node in &graph.nodes {
        if assigned.contains(&node.id) || locally_assigned.contains(&node.id) {
            continue;
        }

        let is_gain_device = matches!(
            node.comp.as_ref().map(|c| &c.kind),
            Some(
                ComponentKind::Triode(_)
                    | ComponentKind::Pentode(_)
                    | ComponentKind::Npn(_)
                    | ComponentKind::Pnp(_)
                    | ComponentKind::NJfet(_)
                    | ComponentKind::PJfet(_)
                    | ComponentKind::Nmos(_)
                    | ComponentKind::Pmos(_)
            )
        );

        if !is_gain_device {
            continue;
        }

        stage_num += 1;
        let mut members = vec![node.id];

        // Walk neighbors to find associated passives:
        // - Plate/collector load resistors (connect to vcc)
        // - Cathode/emitter bias R+C (connect to gnd)
        // - Input coupling cap
        // - Grid/base leak resistor
        let neighbors = graph.neighbors(node.id);
        for &nid in &neighbors {
            if assigned.contains(&nid) || locally_assigned.contains(&nid) || nid == node.id {
                continue;
            }
            if !graph.is_passive(nid) {
                continue;
            }

            // Check if this passive connects to vcc or gnd (supply/bias component)
            if graph.connects_to_vcc(nid) || graph.connects_to_gnd(nid) {
                members.push(nid);
                continue;
            }

            // Check if this passive connects to only this active device and one other
            // (coupling cap or grid leak)
            let passive_neighbors = graph.neighbors(nid);
            if passive_neighbors.len() <= 3 {
                // Small connectivity — likely part of this stage
                let connects_to_other_active = passive_neighbors.iter().any(|&pid| {
                    pid != node.id
                        && !assigned.contains(&pid)
                        && !locally_assigned.contains(&pid)
                        && graph.is_active_device(pid)
                });
                if !connects_to_other_active {
                    members.push(nid);
                }
            }
        }

        for &m in &members {
            locally_assigned.insert(m);
        }

        let device_name = &node.comp_id;
        let kind_name = match node.comp.as_ref().map(|c| &c.kind) {
            Some(ComponentKind::Triode(_)) => "Triode",
            Some(ComponentKind::Pentode(_)) => "Pentode",
            Some(ComponentKind::Npn(_) | ComponentKind::Pnp(_)) => "BJT",
            Some(ComponentKind::NJfet(_) | ComponentKind::PJfet(_)) => "JFET",
            Some(ComponentKind::Nmos(_) | ComponentKind::Pmos(_)) => "MOSFET",
            _ => "Gain",
        };

        groups.push(FunctionalGroup {
            name: format!("gain_stage_{stage_num}"),
            label: format!("{device_name} {kind_name} Stage"),
            kind: GroupKind::GainStage,
            members,
            primary_device: Some(node.id),
        });
    }

    groups
}

// ---------------------------------------------------------------------------
// Op-amp stage detection
// ---------------------------------------------------------------------------

fn detect_opamp_stages(graph: &LayoutGraph, assigned: &HashSet<usize>) -> Vec<FunctionalGroup> {
    let mut groups = Vec::new();
    let mut stage_num = 0;
    let mut locally_assigned: HashSet<usize> = HashSet::new();

    for node in &graph.nodes {
        if assigned.contains(&node.id) || locally_assigned.contains(&node.id) {
            continue;
        }

        let is_opamp = matches!(
            node.comp.as_ref().map(|c| &c.kind),
            Some(ComponentKind::OpAmp(_))
        );

        if !is_opamp {
            continue;
        }

        stage_num += 1;
        let mut members = vec![node.id];

        // Collect feedback network components (connect between neg input and output)
        let neighbors = graph.neighbors(node.id);
        for &nid in &neighbors {
            if assigned.contains(&nid) || locally_assigned.contains(&nid) || nid == node.id {
                continue;
            }
            // Include passives and diodes in the feedback network
            let is_feedback_component = matches!(
                graph.node_kind(nid),
                Some(
                    ComponentKind::Resistor(_)
                        | ComponentKind::Capacitor(_)
                        | ComponentKind::Diode(_)
                        | ComponentKind::DiodePair(_)
                        | ComponentKind::Potentiometer(_)
                )
            );
            if is_feedback_component {
                members.push(nid);
            }
        }

        for &m in &members {
            locally_assigned.insert(m);
        }

        let device_name = &node.comp_id;
        groups.push(FunctionalGroup {
            name: format!("opamp_stage_{stage_num}"),
            label: format!("{device_name} Op-Amp Stage"),
            kind: GroupKind::OpAmpStage,
            members,
            primary_device: Some(node.id),
        });
    }

    groups
}

// ---------------------------------------------------------------------------
// Push-pull output detection
// ---------------------------------------------------------------------------

fn detect_push_pull(graph: &LayoutGraph, assigned: &HashSet<usize>) -> Vec<FunctionalGroup> {
    let mut groups = Vec::new();

    // Find transformers — push-pull outputs always have one
    let transformers: Vec<usize> = graph
        .nodes
        .iter()
        .filter(|n| {
            !assigned.contains(&n.id)
                && matches!(n.comp.as_ref().map(|c| &c.kind), Some(ComponentKind::Transformer(_)))
        })
        .map(|n| n.id)
        .collect();

    for tx_id in transformers {
        let mut members = vec![tx_id];

        // Find pentodes connected to the transformer
        let neighbors = graph.neighbors(tx_id);
        let pentodes: Vec<usize> = neighbors
            .iter()
            .filter(|&&nid| {
                !assigned.contains(&nid)
                    && matches!(
                        graph.node_kind(nid),
                        Some(ComponentKind::Pentode(_))
                    )
            })
            .copied()
            .collect();

        if pentodes.len() >= 2 {
            // This is a push-pull output
            for &pid in &pentodes {
                members.push(pid);
                // Also grab bias components connected to each pentode
                for &nid in &graph.neighbors(pid) {
                    if !assigned.contains(&nid) && graph.is_passive(nid) {
                        if graph.connects_to_vcc(nid) || graph.connects_to_gnd(nid) {
                            members.push(nid);
                        }
                    }
                }
            }

            groups.push(FunctionalGroup {
                name: "push_pull_output".into(),
                label: "Push-Pull Output".into(),
                kind: GroupKind::PushPullOutput,
                members,
                primary_device: Some(tx_id),
            });
        }
    }

    groups
}

// ---------------------------------------------------------------------------
// Tone stack detection
// ---------------------------------------------------------------------------

fn detect_tone_stacks(graph: &LayoutGraph, assigned: &HashSet<usize>) -> Vec<FunctionalGroup> {
    let mut groups = Vec::new();

    // Find pots that aren't already assigned
    let unassigned_pots: Vec<usize> = graph
        .nodes
        .iter()
        .filter(|n| {
            !assigned.contains(&n.id)
                && matches!(n.comp.as_ref().map(|c| &c.kind), Some(ComponentKind::Potentiometer(_)))
        })
        .map(|n| n.id)
        .collect();

    if unassigned_pots.is_empty() {
        return groups;
    }

    // Group pots that are connected to each other (or to shared passives)
    let mut pot_clusters: Vec<HashSet<usize>> = Vec::new();

    for &pot_id in &unassigned_pots {
        // Find which existing cluster this pot belongs to (via shared neighbors)
        let pot_neighbors = graph.neighbors(pot_id);
        let mut matching_cluster = None;

        for (ci, cluster) in pot_clusters.iter().enumerate() {
            // Check if this pot shares a neighbor with any pot in the cluster
            for &existing_pot in cluster.iter() {
                let existing_neighbors = graph.neighbors(existing_pot);
                if pot_neighbors.intersection(&existing_neighbors).next().is_some() {
                    matching_cluster = Some(ci);
                    break;
                }
            }
            if matching_cluster.is_some() {
                break;
            }
        }

        match matching_cluster {
            Some(ci) => {
                pot_clusters[ci].insert(pot_id);
            }
            None => {
                let mut cluster = HashSet::new();
                cluster.insert(pot_id);
                pot_clusters.push(cluster);
            }
        }
    }

    let mut ts_num = 0;
    for cluster in pot_clusters {
        ts_num += 1;
        let mut members: Vec<usize> = cluster.iter().copied().collect();

        // Add passives that connect between the pots
        for &pot_id in &cluster {
            for &nid in &graph.neighbors(pot_id) {
                if assigned.contains(&nid) || members.contains(&nid) {
                    continue;
                }
                if graph.is_passive(nid) && !graph.is_active_device(nid) {
                    members.push(nid);
                }
            }
        }

        let pot_names: Vec<&str> = members
            .iter()
            .filter_map(|&m| {
                if matches!(graph.node_kind(m), Some(ComponentKind::Potentiometer(_))) {
                    Some(graph.nodes[m].comp_id.as_str())
                } else {
                    None
                }
            })
            .collect();

        let label = if pot_names.len() == 1 {
            format!("{} Control", pot_names[0])
        } else {
            "Tone Stack".into()
        };

        groups.push(FunctionalGroup {
            name: format!("tone_stack_{ts_num}"),
            label,
            kind: GroupKind::ToneStack,
            members,
            primary_device: None,
        });
    }

    groups
}

// ---------------------------------------------------------------------------
// Remaining components
// ---------------------------------------------------------------------------

fn assign_remaining(graph: &LayoutGraph, assigned: &HashSet<usize>) -> Vec<FunctionalGroup> {
    let mut groups = Vec::new();
    let mut input_members = Vec::new();
    let mut output_members = Vec::new();
    let mut generic_members = Vec::new();

    for node in &graph.nodes {
        if assigned.contains(&node.id) || node.is_anchor {
            continue;
        }

        // Check if this component connects to input or output
        let connects_to_in = graph
            .edges
            .iter()
            .any(|e| (e.from == graph.in_node && e.to == node.id) || (e.from == node.id && e.to == graph.in_node));
        let connects_to_out = graph
            .edges
            .iter()
            .any(|e| (e.from == graph.out_node && e.to == node.id) || (e.from == node.id && e.to == graph.out_node));

        if connects_to_in {
            input_members.push(node.id);
        } else if connects_to_out {
            output_members.push(node.id);
        } else {
            generic_members.push(node.id);
        }
    }

    if !input_members.is_empty() {
        groups.push(FunctionalGroup {
            name: "input".into(),
            label: "Input".into(),
            kind: GroupKind::InputSection,
            members: input_members,
            primary_device: None,
        });
    }

    if !output_members.is_empty() {
        groups.push(FunctionalGroup {
            name: "output".into(),
            label: "Output".into(),
            kind: GroupKind::OutputSection,
            members: output_members,
            primary_device: None,
        });
    }

    if !generic_members.is_empty() {
        groups.push(FunctionalGroup {
            name: "misc".into(),
            label: "Misc".into(),
            kind: GroupKind::Generic,
            members: generic_members,
            primary_device: None,
        });
    }

    groups
}

#[cfg(test)]
mod tests {
    use super::*;
    use pedalkernel::dsl::*;

    #[test]
    fn detect_triode_gain_stage() {
        let pedal = PedalDef {
            name: "Test".into(),
            supplies: vec![],
            components: vec![
                ComponentDef { id: "C1".into(), kind: ComponentKind::Capacitor(CapConfig::new(100e-9)) },
                ComponentDef { id: "R1".into(), kind: ComponentKind::Resistor(1e6) },
                ComponentDef { id: "V1".into(), kind: ComponentKind::Triode(TriodeType::T12ax7) },
                ComponentDef { id: "R2".into(), kind: ComponentKind::Resistor(100e3) },
                ComponentDef { id: "R3".into(), kind: ComponentKind::Resistor(1500.0) },
                ComponentDef { id: "C2".into(), kind: ComponentKind::Capacitor(CapConfig::new(25e-6)) },
            ],
            nets: vec![
                NetDef {
                    from: Pin::Reserved("in".into()),
                    to: vec![Pin::ComponentPin { component: "C1".into(), pin: "a".into() }],
                },
                NetDef {
                    from: Pin::ComponentPin { component: "C1".into(), pin: "b".into() },
                    to: vec![
                        Pin::ComponentPin { component: "R1".into(), pin: "a".into() },
                        Pin::ComponentPin { component: "V1".into(), pin: "grid".into() },
                    ],
                },
                NetDef {
                    from: Pin::Reserved("vcc".into()),
                    to: vec![Pin::ComponentPin { component: "R2".into(), pin: "a".into() }],
                },
                NetDef {
                    from: Pin::ComponentPin { component: "R2".into(), pin: "b".into() },
                    to: vec![Pin::ComponentPin { component: "V1".into(), pin: "plate".into() }],
                },
                NetDef {
                    from: Pin::ComponentPin { component: "V1".into(), pin: "cathode".into() },
                    to: vec![
                        Pin::ComponentPin { component: "R3".into(), pin: "a".into() },
                        Pin::ComponentPin { component: "C2".into(), pin: "a".into() },
                    ],
                },
                NetDef {
                    from: Pin::ComponentPin { component: "R3".into(), pin: "b".into() },
                    to: vec![Pin::Reserved("gnd".into())],
                },
                NetDef {
                    from: Pin::ComponentPin { component: "C2".into(), pin: "b".into() },
                    to: vec![Pin::Reserved("gnd".into())],
                },
                NetDef {
                    from: Pin::ComponentPin { component: "R1".into(), pin: "b".into() },
                    to: vec![Pin::Reserved("gnd".into())],
                },
                NetDef {
                    from: Pin::ComponentPin { component: "V1".into(), pin: "plate".into() },
                    to: vec![Pin::Reserved("out".into())],
                },
            ],
            controls: vec![],
            trims: vec![],
            monitors: vec![],
        };

        let graph = LayoutGraph::from_pedal(&pedal);
        let groups = detect_groups(&graph);

        // Should detect at least one gain stage
        let gain_stages: Vec<_> = groups.iter().filter(|g| g.kind == GroupKind::GainStage).collect();
        assert!(!gain_stages.is_empty(), "Should detect a gain stage");

        // The gain stage should contain the triode
        let gs = &gain_stages[0];
        let has_triode = gs.members.iter().any(|&m| graph.nodes[m].comp_id == "V1");
        assert!(has_triode, "Gain stage should contain the triode");
    }
}
