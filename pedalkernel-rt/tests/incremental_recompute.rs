//! Tests for the incremental recompute system (Phases 1+2).
//!
//! Validates: is_dynamic(), compute_dynamic_flags(), set_pot_dirty(),
//! recompute_incremental() — the infrastructure for O(log N) pot updates.

use pedalkernel_rt::dyn_node::DynNode;
use pedalkernel_rt::pot_taper::PotTaper;
use pedalkernel_rt::wdf_leaf::*;

// ═══════════════════════════════════════════════════════════════════════════
// 1. is_dynamic() correctness
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn wdf_pot_is_dynamic() {
    let pot = WdfPot {
        comp_id: "Drive".to_string(),
        max_resistance: 100_000.0,
        position: 0.5,
        taper: PotTaper::B,
        rp: 50_000.0,
        last_a: 0.0,
        complement: false,
    };
    assert!(pot.is_dynamic(), "WdfPot should be dynamic");
}

#[test]
fn wdf_resistor_is_not_dynamic() {
    let r = WdfResistor {
        comp_id: Some("R1".to_string()),
        rp: 1000.0,
        last_a: 0.0,
    };
    assert!(!r.is_dynamic(), "WdfResistor should NOT be dynamic");
}

#[test]
fn wdf_capacitor_is_not_dynamic() {
    let c = LeafKind::capacitor(Some("C1".to_string()), 100e-9, 48_000.0);
    assert!(!c.is_dynamic(), "capacitor one-port should NOT be dynamic");
}

#[test]
fn wdf_voltage_source_is_not_dynamic() {
    let vs = WdfVoltageSource {
        voltage: 0.0,
        rp: 1.0,
        is_cathode_bias: false,
        port_name: None,
    };
    assert!(!vs.is_dynamic(), "WdfVoltageSource should NOT be dynamic");
}

// ═══════════════════════════════════════════════════════════════════════════
// 2. compute_dynamic_flags()
// ═══════════════════════════════════════════════════════════════════════════

/// Helper: build a Resistor(1kΩ) DynNode.
fn resistor_1k() -> DynNode {
    DynNode::Resistor(Some("R1".to_string()), 1000.0)
}

/// Helper: build a Capacitor(100nF @ 48kHz) DynNode.
fn cap_100n() -> DynNode {
    let fs = 48000.0;
    let c = 100e-9;
    DynNode::Capacitor(Some("C1".to_string()), c, 1.0 / (2.0 * fs * c))
}

/// Helper: build a Pot(100kΩ) DynNode at position 0.5.
fn pot_100k() -> DynNode {
    DynNode::Pot("Gain".to_string(), 100_000.0, 0.5, PotTaper::B)
}

#[test]
fn static_tree_has_no_dynamic_flags() {
    // Series(Resistor(1k), Capacitor(100n, 48000)) — all static
    let mut tree = DynNode::Series(Box::new(resistor_1k()), Box::new(cap_100n()));

    let result = tree.compute_dynamic_flags();
    assert!(
        !result,
        "Static tree should return false from compute_dynamic_flags"
    );

    match tree.structural() {
        DynNode::Binary { has_dynamic, .. } => {
            assert!(
                !has_dynamic,
                "Static tree root should have has_dynamic=false"
            );
        }
        _ => panic!("Expected Binary node"),
    }
}

#[test]
fn tree_with_pot_has_dynamic_flag() {
    // Series(Resistor(1k), Pot(100k))
    let mut tree = DynNode::Series(Box::new(resistor_1k()), Box::new(pot_100k()));

    let result = tree.compute_dynamic_flags();
    assert!(
        result,
        "Tree with pot should return true from compute_dynamic_flags"
    );

    match tree.structural() {
        DynNode::Binary { has_dynamic, .. } => {
            assert!(has_dynamic, "Root should have has_dynamic=true");
        }
        _ => panic!("Expected Binary node"),
    }
}

#[test]
fn mixed_tree_dynamic_flag_only_on_pot_side() {
    // Series(Resistor(1k), Series(Capacitor(100n), Pot(100k)))
    let inner = DynNode::Series(Box::new(cap_100n()), Box::new(pot_100k()));
    let mut tree = DynNode::Series(Box::new(resistor_1k()), Box::new(inner));

    tree.compute_dynamic_flags();

    // Root Series: has_dynamic = true
    match tree.structural() {
        DynNode::Binary {
            has_dynamic,
            left,
            right,
            ..
        } => {
            assert!(has_dynamic, "Root should have has_dynamic=true");

            // Left is a Resistor leaf — not dynamic
            assert!(
                matches!(left.as_ref(), DynNode::Leaf(l) if !l.is_dynamic()),
                "Left child (Resistor) should not be dynamic"
            );

            // Right is inner Series: has_dynamic = true
            match right.structural() {
                DynNode::Binary { has_dynamic, .. } => {
                    assert!(has_dynamic, "Inner Series should have has_dynamic=true");
                }
                _ => panic!("Expected inner Binary node"),
            }
        }
        _ => panic!("Expected Binary node at root"),
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// 3. set_pot_dirty() marks only ancestor path
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn set_pot_dirty_marks_ancestors() {
    // Series(Resistor(1k), Pot(100k))
    let mut tree = DynNode::Series(Box::new(resistor_1k()), Box::new(pot_100k()));
    tree.compute_dynamic_flags();

    // Dirty should be false after compute_dynamic_flags
    match tree.structural() {
        DynNode::Binary { dirty, .. } => assert!(!dirty, "dirty should be false initially"),
        _ => panic!("Expected Binary"),
    }

    let found = tree.set_pot_dirty("Gain", 0.7);
    assert!(found, "set_pot_dirty should find the pot");

    match tree.structural() {
        DynNode::Binary { dirty, .. } => assert!(dirty, "Root should be dirty after set_pot_dirty"),
        _ => panic!("Expected Binary"),
    }
}

#[test]
fn set_pot_dirty_returns_false_for_missing_pot() {
    let mut tree = DynNode::Series(Box::new(resistor_1k()), Box::new(pot_100k()));
    tree.compute_dynamic_flags();

    let found = tree.set_pot_dirty("nonexistent", 0.5);
    assert!(!found, "set_pot_dirty should return false for missing pot");

    match tree.structural() {
        DynNode::Binary { dirty, .. } => {
            assert!(!dirty, "Root should NOT be dirty when pot not found");
        }
        _ => panic!("Expected Binary"),
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// 4. recompute_incremental() correctness
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn incremental_matches_full_recompute() {
    // Build: Series(Resistor(1k), Pot(100k))
    let mut tree_full = DynNode::Series(Box::new(resistor_1k()), Box::new(pot_100k()));
    let mut tree_incr = tree_full.clone();

    // Set pot to 0.5 and do full recompute (initial state)
    tree_full.set_pot("Gain", 0.5);
    tree_full.recompute();

    tree_incr.set_pot("Gain", 0.5);
    tree_incr.recompute();
    tree_incr.compute_dynamic_flags();

    // Now change pot to 0.7
    // Full recompute path:
    tree_full.set_pot("Gain", 0.7);
    tree_full.recompute();
    let rp_full = tree_full.port_resistance();

    // Incremental path:
    tree_incr.set_pot_dirty("Gain", 0.7);
    tree_incr.recompute_incremental();
    let rp_incr = tree_incr.port_resistance();

    assert!(
        (rp_full - rp_incr).abs() < 1e-10,
        "Incremental and full recompute should produce identical port_resistance: full={rp_full}, incr={rp_incr}"
    );

    // Check gamma too
    let gamma_full = match tree_full.structural() {
        DynNode::Binary { gamma, .. } => *gamma,
        _ => panic!("Expected Binary"),
    };
    let gamma_incr = match tree_incr.structural() {
        DynNode::Binary { gamma, .. } => *gamma,
        _ => panic!("Expected Binary"),
    };
    assert!(
        (gamma_full - gamma_incr).abs() < 1e-10,
        "Gamma mismatch: full={gamma_full}, incr={gamma_incr}"
    );
}

#[test]
fn incremental_skips_static_subtree() {
    // Build: Series(Capacitor(100n, 48000), Pot(100k))
    let mut tree = DynNode::Series(Box::new(cap_100n()), Box::new(pot_100k()));
    tree.recompute();
    tree.compute_dynamic_flags();

    let rp_before = tree.port_resistance();

    // Change pot
    tree.set_pot_dirty("Gain", 0.8);
    tree.recompute_incremental();

    let rp_after = tree.port_resistance();

    // Port resistance should change (pot changed from 0.5 to 0.8)
    assert!(
        (rp_before - rp_after).abs() > 1.0,
        "Port resistance should change after pot update: before={rp_before}, after={rp_after}"
    );

    // Verify the cap's rp is unchanged (it's static, should not be touched)
    match tree.structural() {
        DynNode::Binary { left, .. } => {
            let cap_rp = left.port_resistance();
            let expected_rp = 1.0 / (2.0 * 48000.0 * 100e-9);
            assert!(
                (cap_rp - expected_rp).abs() < 1e-6,
                "Cap rp should be unchanged: got={cap_rp}, expected={expected_rp}"
            );
        }
        _ => panic!("Expected Binary"),
    }
}

#[test]
fn incremental_noop_when_no_pot_changed() {
    let mut tree = DynNode::Series(Box::new(resistor_1k()), Box::new(pot_100k()));
    tree.recompute();
    tree.compute_dynamic_flags();

    let rp_before = tree.port_resistance();
    let gamma_before = match tree.structural() {
        DynNode::Binary { gamma, .. } => *gamma,
        _ => panic!("Expected Binary"),
    };

    // Don't change any pot, just call recompute_incremental
    tree.recompute_incremental();

    let rp_after = tree.port_resistance();
    let gamma_after = match tree.structural() {
        DynNode::Binary { gamma, .. } => *gamma,
        _ => panic!("Expected Binary"),
    };

    assert_eq!(
        rp_before, rp_after,
        "Port resistance should be unchanged when no pot changed"
    );
    assert_eq!(
        gamma_before, gamma_after,
        "Gamma should be unchanged when no pot changed"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 5. Split pot (aw/wb) dirty marking
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn split_pot_dirty_marks_both_halves() {
    // Series(Pot("Drive__aw", 100k), Pot("Drive__wb", 100k))
    let aw = DynNode::Pot("Drive__aw".to_string(), 100_000.0, 0.5, PotTaper::B);
    let wb = DynNode::Pot("Drive__wb".to_string(), 100_000.0, 0.5, PotTaper::B);
    let mut tree = DynNode::Series(Box::new(aw), Box::new(wb));

    tree.compute_dynamic_flags();

    // Set via base name "Drive" — should match both __aw and __wb
    let found = tree.set_pot_dirty("Drive", 0.3);
    assert!(
        found,
        "set_pot_dirty should find split pot halves via prefix"
    );

    match tree.structural() {
        DynNode::Binary { dirty, .. } => {
            assert!(dirty, "Root should be dirty after split pot update");
        }
        _ => panic!("Expected Binary"),
    }

    // Verify both halves updated their position
    let pos_aw = tree.get_pot_position("Drive__aw");
    let pos_wb = tree.get_pot_position("Drive__wb");
    assert_eq!(pos_aw, Some(0.3), "Drive__aw position should be 0.3");
    assert_eq!(pos_wb, Some(0.3), "Drive__wb position should be 0.3");
}
