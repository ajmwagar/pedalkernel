//! Model conversion helpers and WDF tree utilities.

use crate::dsl::*;
use crate::elements::*;

use super::dyn_node::{BinaryKind, DynNode};

pub(super) fn diode_model(dt: DiodeType) -> DiodeModel {
    match dt {
        DiodeType::Silicon => DiodeModel::silicon(),
        DiodeType::Germanium => DiodeModel::germanium(),
        DiodeType::Led => DiodeModel::led(),
    }
}

pub(super) fn jfet_model(name: &str, is_n_channel: bool) -> JfetModel {
    let model = JfetModel::by_name(name);
    // Handle polarity mismatch: if DSL says N-channel but model is P-channel
    if is_n_channel != model.is_n_channel {
        JfetModel::by_name("2N5457")
    } else {
        model
    }
}

pub(super) fn triode_model(name: &str) -> TriodeModel {
    TriodeModel::by_name(name)
}

pub(super) fn pentode_model(name: &str) -> PentodeModel {
    PentodeModel::by_name(name)
}

pub(super) fn vari_mu_model(name: &str) -> VariMuModel {
    VariMuModel::by_name(name)
}

/// Look up a Gummel-Poon BJT model by name from the embedded SPICE model library.
pub(super) fn gummel_poon_model(name: &str) -> GummelPoonModel {
    GummelPoonModel::by_name(name)
}

pub(super) fn mosfet_model(mt: MosfetType, is_n_channel: bool) -> MosfetModel {
    match mt {
        MosfetType::N2n7000 => MosfetModel::n_2n7000(),
        MosfetType::Irf520 => MosfetModel::n_irf520(),
        MosfetType::Bs250 => {
            if is_n_channel {
                // Mismatch: N-channel requested with P-channel type
                MosfetModel::n_2n7000()
            } else {
                MosfetModel::p_bs250()
            }
        }
        MosfetType::Irf9520 => {
            if is_n_channel {
                MosfetModel::n_2n7000()
            } else {
                MosfetModel::p_irf9520()
            }
        }
    }
}

pub(super) fn has_vs(node: &DynNode) -> bool {
    match node {
        DynNode::Leaf(leaf) if leaf.type_tag() == "voltage_source" => true,
        DynNode::Binary { left, right, .. } => {
            has_vs(left) || has_vs(right)
        }
        _ => false,
    }
}

/// Walk the tree and balance any adaptor where one branch contains the
/// VoltageSource and the other has much higher impedance.
///
/// - **Parallel**: high-Z sibling causes gamma->1, attenuating the Vs signal
///   (silent output, e.g. Big Muff).
/// - **Series**: high-Z sibling causes gamma->0, dumping all scattered-down
///   energy into the passives and causing oscillation/instability (e.g. ProCo RAT).
pub(super) fn balance_parallel_vs(node: &mut DynNode) {
    match node {
        DynNode::Binary { left, right, .. } => {
            let left_has_vs = has_vs(left);
            let right_has_vs = has_vs(right);

            if left_has_vs && !right_has_vs {
                let target = right.port_resistance();
                adjust_vs_branch_rp(left, target);
            } else if right_has_vs && !left_has_vs {
                let target = left.port_resistance();
                adjust_vs_branch_rp(right, target);
            }
            // Recurse into children.
            balance_parallel_vs(left);
            balance_parallel_vs(right);
        }
        // Recurse through Transformer nodes so VS balancing reaches
        // inner trees (e.g., push-pull trees wrapped with transformer load).
        DynNode::Transformer { secondary, .. } => {
            balance_parallel_vs(secondary);
        }
        _ => {}
    }
}

/// Adjust the Vs port resistance inside `branch` so that `branch.port_resistance() ~ target_rp`.
///
/// If the branch already has comparable impedance, do nothing.
pub(super) fn adjust_vs_branch_rp(branch: &mut DynNode, target_rp: f64) {
    let current_rp = branch.port_resistance();
    if current_rp >= target_rp * 0.5 {
        return; // Already reasonably balanced.
    }
    match branch {
        DynNode::Leaf(leaf) if leaf.type_tag() == "voltage_source" => {
            leaf.set_resistance(target_rp.max(1.0));
        }
        DynNode::Binary { kind: BinaryKind::Series, left, right, .. } => {
            // Series(Vs_branch, other): set Vs rp so that series total ~ target.
            if has_vs(left) {
                let other_rp = right.port_resistance();
                let vs_target = (target_rp - other_rp).max(1.0);
                set_vs_rp(left, vs_target);
            } else if has_vs(right) {
                let other_rp = left.port_resistance();
                let vs_target = (target_rp - other_rp).max(1.0);
                set_vs_rp(right, vs_target);
            }
        }
        DynNode::Binary { kind: BinaryKind::Parallel, left, right, .. } => {
            // Recurse -- the Vs is deeper.
            if has_vs(left) {
                adjust_vs_branch_rp(left, target_rp);
            } else {
                adjust_vs_branch_rp(right, target_rp);
            }
        }
        _ => {}
    }
}

pub(super) fn set_vs_rp(node: &mut DynNode, rp_val: f64) {
    match node {
        DynNode::Leaf(leaf) if leaf.type_tag() == "voltage_source" => {
            leaf.set_resistance(rp_val);
        }
        DynNode::Binary { left, right, .. } => {
            if has_vs(left) {
                set_vs_rp(left, rp_val);
            } else {
                set_vs_rp(right, rp_val);
            }
        }
        _ => {}
    }
}

/// Collect all pot component IDs from a DynNode tree (for debugging).
pub(super) fn collect_pot_ids(node: &DynNode, out: &mut Vec<String>) {
    match node {
        DynNode::Leaf(leaf) if leaf.type_tag() == "pot" => {
            if let Some(id) = leaf.comp_id() {
                out.push(id.to_string());
            }
        }
        DynNode::Binary { left, right, .. } => {
            collect_pot_ids(left, out);
            collect_pot_ids(right, out);
        }
        DynNode::Transformer { secondary, .. } => collect_pot_ids(secondary, out),
        DynNode::RType { children, .. } => {
            for c in children {
                collect_pot_ids(c, out);
            }
        }
        _ => {}
    }
}

pub(super) fn has_pot(node: &DynNode, comp_id: &str) -> bool {
    match node {
        DynNode::Leaf(leaf) if leaf.type_tag() == "pot" => {
            match leaf.comp_id() {
                Some(id) => {
                    // Match the pot itself or the synthetic __aw half of a 3-terminal pot.
                    // (Finding either half means the control is in this stage.)
                    id == comp_id || id == format!("{comp_id}__aw")
                }
                None => false,
            }
        }
        DynNode::Binary { left, right, .. } => {
            has_pot(left, comp_id) || has_pot(right, comp_id)
        }
        DynNode::Transformer { secondary, .. } => has_pot(secondary, comp_id),
        DynNode::RType { children, .. } => children.iter().any(|c| has_pot(c, comp_id)),
        _ => false,
    }
}
