//! Runtime helper functions for WDF tree manipulation.
//!
//! Extracted from pedalkernel's compiler/helpers.rs — only the functions
//! needed at runtime (balance_parallel_vs, has_pot, etc.).

extern crate alloc;
use alloc::{
    string::{String, ToString},
    vec::Vec,
};

use crate::dyn_node::{BinaryKind, DynNode};
use crate::wdf_leaf::WdfLeaf;

pub fn has_vs(node: &DynNode) -> bool {
    match node.structural() {
        DynNode::Leaf(leaf) if leaf.type_tag() == "voltage_source" => true,
        DynNode::Binary { left, right, .. } => has_vs(left) || has_vs(right),
        _ => false,
    }
}

/// Walk the tree and balance any adaptor where one branch contains the
/// VoltageSource and the other has much higher impedance.
pub fn balance_parallel_vs(node: &mut DynNode) {
    match node.structural_mut() {
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
        DynNode::Transformer { secondary, .. } => {
            balance_parallel_vs(secondary);
        }
        _ => {}
    }
}

/// Adjust the Vs port resistance inside `branch` so that `branch.port_resistance() ~ target_rp`.
pub fn adjust_vs_branch_rp(branch: &mut DynNode, target_rp: crate::Wave) {
    let current_rp = branch.port_resistance();
    if current_rp >= target_rp * 0.5 {
        return;
    }
    match branch.structural_mut() {
        DynNode::Leaf(leaf) if leaf.type_tag() == "voltage_source" => {
            leaf.set_resistance(target_rp.max(1.0));
        }
        DynNode::Binary {
            kind: BinaryKind::Series,
            left,
            right,
            ..
        } => {
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
        DynNode::Binary {
            kind: BinaryKind::Parallel,
            left,
            right,
            ..
        } => {
            if has_vs(left) {
                adjust_vs_branch_rp(left, target_rp);
            } else {
                adjust_vs_branch_rp(right, target_rp);
            }
        }
        _ => {}
    }
}

pub fn set_vs_rp(node: &mut DynNode, rp_val: crate::Wave) {
    match node.structural_mut() {
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

/// Collect all pot component IDs from a DynNode tree.
pub fn collect_pot_ids(node: &DynNode, out: &mut Vec<String>) {
    match node.structural() {
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

pub fn has_pot(node: &DynNode, comp_id: &str) -> bool {
    match node.structural() {
        DynNode::Leaf(leaf) if leaf.type_tag() == "pot" => match leaf.comp_id() {
            Some(id) => {
                id == comp_id
                    || id
                        .strip_prefix(comp_id)
                        .is_some_and(|suffix| suffix == "__aw" || suffix == "__wb")
            }
            None => false,
        },
        DynNode::Binary { left, right, .. } => has_pot(left, comp_id) || has_pot(right, comp_id),
        DynNode::Transformer { secondary, .. } => has_pot(secondary, comp_id),
        DynNode::RType { children, .. } => children.iter().any(|c| has_pot(c, comp_id)),
        _ => false,
    }
}
