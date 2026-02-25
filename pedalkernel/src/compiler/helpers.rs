//! Model conversion helpers and WDF tree utilities.

use crate::dsl::*;
use crate::elements::*;

use super::dyn_node::DynNode;
use super::graph::{SpTree, make_leaf};

/// Convert SP tree to DynNode, inserting a VoltageSource for the virtual leaf.
pub(super) fn sp_to_dyn_with_vs(
    tree: &SpTree,
    components: &[ComponentDef],
    sample_rate: f64,
    vs_idx: usize,
) -> DynNode {
    match tree {
        SpTree::Leaf(idx) if *idx == vs_idx => DynNode::VoltageSource {
            voltage: 0.0,
            rp: 1.0,
        },
        SpTree::Leaf(idx) => make_leaf(&components[*idx], sample_rate),
        SpTree::Series(left, right) => {
            let l = sp_to_dyn_with_vs(left, components, sample_rate, vs_idx);
            let r = sp_to_dyn_with_vs(right, components, sample_rate, vs_idx);
            let r1 = l.port_resistance();
            let r2 = r.port_resistance();
            let rp = r1 + r2;
            DynNode::Series {
                left: Box::new(l),
                right: Box::new(r),
                rp,
                gamma: r1 / rp,
                b1: 0.0,
                b2: 0.0,
            }
        }
        SpTree::Parallel(left, right) => {
            let l = sp_to_dyn_with_vs(left, components, sample_rate, vs_idx);
            let r = sp_to_dyn_with_vs(right, components, sample_rate, vs_idx);
            let r1 = l.port_resistance();
            let r2 = r.port_resistance();
            let rp = r1 * r2 / (r1 + r2);
            DynNode::Parallel {
                left: Box::new(l),
                right: Box::new(r),
                rp,
                gamma: r2 / (r1 + r2),
                b1: 0.0,
                b2: 0.0,
            }
        }
    }
}

pub(super) fn diode_model(dt: DiodeType) -> DiodeModel {
    match dt {
        DiodeType::Silicon => DiodeModel::silicon(),
        DiodeType::Germanium => DiodeModel::germanium(),
        DiodeType::Led => DiodeModel::led(),
    }
}

pub(super) fn jfet_model(jt: JfetType, is_n_channel: bool) -> JfetModel {
    match jt {
        JfetType::J201 => JfetModel::n_j201(),
        JfetType::N2n5457 => JfetModel::n_2n5457(),
        JfetType::N2n5952 => JfetModel::n_2n5952(),
        // 2SK30A variants - classic phaser JFETs
        JfetType::N2sk30a => JfetModel::n_2sk30a(),
        JfetType::N2sk30aGr => JfetModel::n_2sk30a_gr(),
        JfetType::N2sk30aY => JfetModel::n_2sk30a_y(),
        JfetType::N2sk30aBl => JfetModel::n_2sk30a_bl(),
        JfetType::P2n5460 => {
            if is_n_channel {
                // Mismatch: N-channel requested with P-channel type
                // Fall back to a reasonable N-channel model
                JfetModel::n_2n5457()
            } else {
                JfetModel::p_2n5460()
            }
        }
    }
}

pub(super) fn triode_model(tt: TriodeType) -> TriodeModel {
    match tt {
        TriodeType::T12ax7 => TriodeModel::t_12ax7(),
        TriodeType::T12at7 => TriodeModel::t_12at7(),
        TriodeType::T12au7 => TriodeModel::t_12au7(),
        TriodeType::T12ay7 => TriodeModel::t_12ay7(),
        TriodeType::T12bh7 => TriodeModel::t_12bh7(),
        TriodeType::T6386 => TriodeModel::t_6386(),
    }
}

pub(super) fn pentode_model(pt: PentodeType) -> PentodeModel {
    match pt {
        PentodeType::Ef86 => PentodeModel::p_ef86(),
        PentodeType::El84 => PentodeModel::p_el84(),
        // 6AQ5A is a beam power tube similar to EL84, use that model as approximation
        PentodeType::A6aq5a => PentodeModel::p_el84(),
        // 6973 is a 9W beam power tube similar to EL84, used in Fairchild 670 sidechain
        PentodeType::A6973 => PentodeModel::p_el84(),
        PentodeType::A6l6gc => PentodeModel::p_6l6gc(),
        PentodeType::El34 => PentodeModel::p_el34(),
        PentodeType::A6550 => PentodeModel::p_6550(),
    }
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
        DynNode::VoltageSource { .. } => true,
        DynNode::Series { left, right, .. } | DynNode::Parallel { left, right, .. } => {
            has_vs(left) || has_vs(right)
        }
        _ => false,
    }
}

/// Walk the tree and balance any adaptor where one branch contains the
/// VoltageSource and the other has much higher impedance.
///
/// - **Parallel**: high-Z sibling causes gamma→1, attenuating the Vs signal
///   (silent output, e.g. Big Muff).
/// - **Series**: high-Z sibling causes gamma→0, dumping all scattered-down
///   energy into the passives and causing oscillation/instability (e.g. ProCo RAT).
pub(super) fn balance_parallel_vs(node: &mut DynNode) {
    match node {
        DynNode::Parallel { left, right, .. } | DynNode::Series { left, right, .. } => {
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
        _ => {}
    }
}

/// Adjust the Vs port resistance inside `branch` so that `branch.port_resistance() ≈ target_rp`.
///
/// If the branch already has comparable impedance, do nothing.
pub(super) fn adjust_vs_branch_rp(branch: &mut DynNode, target_rp: f64) {
    let current_rp = branch.port_resistance();
    if current_rp >= target_rp * 0.5 {
        return; // Already reasonably balanced.
    }
    match branch {
        DynNode::VoltageSource { rp, .. } => {
            *rp = target_rp.max(1.0);
        }
        DynNode::Series { left, right, .. } => {
            // Series(Vs_branch, other): set Vs rp so that series total ≈ target.
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
        DynNode::Parallel { left, right, .. } => {
            // Recurse — the Vs is deeper.
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
        DynNode::VoltageSource { rp, .. } => *rp = rp_val,
        DynNode::Series { left, right, .. } | DynNode::Parallel { left, right, .. } => {
            if has_vs(left) {
                set_vs_rp(left, rp_val);
            } else {
                set_vs_rp(right, rp_val);
            }
        }
        _ => {}
    }
}

pub(super) fn has_pot(node: &DynNode, comp_id: &str) -> bool {
    match node {
        DynNode::Pot { comp_id: id, .. } => {
            // Match the pot itself or the synthetic __aw half of a 3-terminal pot.
            // (Finding either half means the control is in this stage.)
            id == comp_id || *id == format!("{comp_id}__aw")
        }
        DynNode::Series { left, right, .. } | DynNode::Parallel { left, right, .. } => {
            has_pot(left, comp_id) || has_pot(right, comp_id)
        }
        _ => false,
    }
}

