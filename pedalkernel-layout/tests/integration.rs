//! Integration tests for the layout engine.
//!
//! Tests the full pipeline: PedalDef → Layout → JSON + KiCad export.

use pedalkernel::dsl::*;
use pedalkernel_layout::{generate_layout, to_json, to_kicad_schematic};

/// Build a simple Tube Screamer-like circuit for testing.
fn tube_screamer_pedal() -> PedalDef {
    PedalDef {
        name: "Tube Screamer".into(),
        supplies: vec![],
        components: vec![
            ComponentDef { id: "C1".into(), kind: ComponentKind::Capacitor(CapConfig::new(100e-9)) },
            ComponentDef { id: "R1".into(), kind: ComponentKind::Resistor(510e3) },
            ComponentDef { id: "U1".into(), kind: ComponentKind::OpAmp(OpAmpType::Jrc4558) },
            ComponentDef { id: "R3".into(), kind: ComponentKind::Resistor(4700.0) },
            ComponentDef { id: "U2".into(), kind: ComponentKind::OpAmp(OpAmpType::Jrc4558) },
            ComponentDef { id: "R4".into(), kind: ComponentKind::Resistor(51e3) },
            ComponentDef { id: "Drive".into(), kind: ComponentKind::Potentiometer(500e3, PotTaper::B) },
            ComponentDef { id: "D1".into(), kind: ComponentKind::Diode(DiodeType::Silicon) },
            ComponentDef { id: "D2".into(), kind: ComponentKind::Diode(DiodeType::Silicon) },
            ComponentDef { id: "Tone".into(), kind: ComponentKind::Potentiometer(20e3, PotTaper::B) },
            ComponentDef { id: "C4".into(), kind: ComponentKind::Capacitor(CapConfig::new(220e-9)) },
            ComponentDef { id: "Level".into(), kind: ComponentKind::Potentiometer(100e3, PotTaper::B) },
            ComponentDef { id: "R7".into(), kind: ComponentKind::Resistor(10e3) },
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
                    Pin::ComponentPin { component: "U1".into(), pin: "pos".into() },
                ],
            },
            NetDef {
                from: Pin::ComponentPin { component: "R1".into(), pin: "b".into() },
                to: vec![Pin::Reserved("gnd".into())],
            },
            NetDef {
                from: Pin::ComponentPin { component: "U1".into(), pin: "out".into() },
                to: vec![Pin::ComponentPin { component: "R3".into(), pin: "a".into() }],
            },
            NetDef {
                from: Pin::ComponentPin { component: "R3".into(), pin: "b".into() },
                to: vec![Pin::ComponentPin { component: "U2".into(), pin: "neg".into() }],
            },
            NetDef {
                from: Pin::ComponentPin { component: "U2".into(), pin: "neg".into() },
                to: vec![
                    Pin::ComponentPin { component: "R4".into(), pin: "a".into() },
                    Pin::ComponentPin { component: "Drive".into(), pin: "a".into() },
                    Pin::ComponentPin { component: "D1".into(), pin: "a".into() },
                    Pin::ComponentPin { component: "D2".into(), pin: "b".into() },
                ],
            },
            NetDef {
                from: Pin::ComponentPin { component: "R4".into(), pin: "b".into() },
                to: vec![Pin::ComponentPin { component: "U2".into(), pin: "out".into() }],
            },
            NetDef {
                from: Pin::ComponentPin { component: "Drive".into(), pin: "b".into() },
                to: vec![Pin::ComponentPin { component: "U2".into(), pin: "out".into() }],
            },
            NetDef {
                from: Pin::ComponentPin { component: "D1".into(), pin: "b".into() },
                to: vec![Pin::ComponentPin { component: "U2".into(), pin: "out".into() }],
            },
            NetDef {
                from: Pin::ComponentPin { component: "D2".into(), pin: "a".into() },
                to: vec![Pin::ComponentPin { component: "U2".into(), pin: "out".into() }],
            },
            NetDef {
                from: Pin::ComponentPin { component: "U2".into(), pin: "out".into() },
                to: vec![Pin::ComponentPin { component: "Tone".into(), pin: "a".into() }],
            },
            NetDef {
                from: Pin::ComponentPin { component: "Tone".into(), pin: "b".into() },
                to: vec![Pin::ComponentPin { component: "C4".into(), pin: "a".into() }],
            },
            NetDef {
                from: Pin::ComponentPin { component: "C4".into(), pin: "b".into() },
                to: vec![Pin::Reserved("gnd".into())],
            },
            NetDef {
                from: Pin::ComponentPin { component: "Tone".into(), pin: "b".into() },
                to: vec![
                    Pin::ComponentPin { component: "Level".into(), pin: "a".into() },
                    Pin::ComponentPin { component: "R7".into(), pin: "a".into() },
                ],
            },
            NetDef {
                from: Pin::ComponentPin { component: "R7".into(), pin: "b".into() },
                to: vec![Pin::Reserved("gnd".into())],
            },
            NetDef {
                from: Pin::ComponentPin { component: "Level".into(), pin: "b".into() },
                to: vec![Pin::Reserved("out".into())],
            },
        ],
        controls: vec![
            ControlDef {
                component: "Drive".into(),
                property: "position".into(),
                label: "Drive".into(),
                range: (0.0, 1.0),
                default: 0.5,
            },
            ControlDef {
                component: "Tone".into(),
                property: "position".into(),
                label: "Tone".into(),
                range: (0.0, 1.0),
                default: 0.5,
            },
            ControlDef {
                component: "Level".into(),
                property: "position".into(),
                label: "Level".into(),
                range: (0.0, 1.0),
                default: 0.7,
            },
        ],
        trims: vec![],
        monitors: vec![],
        sidechains: vec![],
    }
}

/// Build a simple triode gain stage circuit for testing.
fn triode_gain_stage() -> PedalDef {
    PedalDef {
        name: "Triode Stage".into(),
        supplies: vec![NamedSupply::new("vcc", 350.0)],
        components: vec![
            ComponentDef { id: "C1".into(), kind: ComponentKind::Capacitor(CapConfig::new(20e-9)) },
            ComponentDef { id: "R1".into(), kind: ComponentKind::Resistor(1e6) },
            ComponentDef { id: "V1".into(), kind: ComponentKind::Triode("12AX7".into()) },
            ComponentDef { id: "R2".into(), kind: ComponentKind::Resistor(100e3) },
            ComponentDef { id: "R3".into(), kind: ComponentKind::Resistor(1500.0) },
            ComponentDef { id: "C2".into(), kind: ComponentKind::Capacitor(CapConfig::new(25e-6)) },
            ComponentDef { id: "C3".into(), kind: ComponentKind::Capacitor(CapConfig::new(22e-9)) },
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
                from: Pin::ComponentPin { component: "R1".into(), pin: "b".into() },
                to: vec![Pin::Reserved("gnd".into())],
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
                from: Pin::ComponentPin { component: "V1".into(), pin: "plate".into() },
                to: vec![Pin::ComponentPin { component: "C3".into(), pin: "a".into() }],
            },
            NetDef {
                from: Pin::ComponentPin { component: "C3".into(), pin: "b".into() },
                to: vec![Pin::Reserved("out".into())],
            },
        ],
        controls: vec![],
        trims: vec![],
        monitors: vec![],
        sidechains: vec![],
    }
}

// ─── Full pipeline tests ────────────────────────────────────────────────────

#[test]
fn tube_screamer_produces_valid_layout() {
    let pedal = tube_screamer_pedal();
    let layout = generate_layout(&pedal, 2048.0, 1024.0);

    assert_eq!(layout.version, 1);
    assert_eq!(layout.bounds.width, 2048.0);
    assert_eq!(layout.bounds.height, 1024.0);

    // All 13 DSL components should be placed (each exactly once)
    assert_eq!(
        layout.components.len(),
        pedal.components.len(),
        "Layout should have exactly one placed component per DSL component"
    );

    // Should have supply rails
    assert_eq!(layout.supply_rails.len(), 2);

    // Should have wires
    assert!(!layout.wires.is_empty(), "Should route wires");

    // Should have groups
    assert!(!layout.groups.is_empty(), "Should detect groups");

    // Signal path order should include all components
    assert_eq!(layout.signal_path_order.len(), layout.components.len());
}

#[test]
fn triode_stage_produces_valid_layout() {
    let pedal = triode_gain_stage();
    let layout = generate_layout(&pedal, 2048.0, 1024.0);

    assert_eq!(layout.components.len(), 7);
    assert!(!layout.wires.is_empty());
    assert!(!layout.groups.is_empty());

    // Check that the triode is placed
    let triode = layout.components.iter().find(|c| c.name == "V1");
    assert!(triode.is_some(), "Triode V1 should be placed");
    let triode = triode.unwrap();
    assert_eq!(triode.kind, "triode");
    assert_eq!(triode.symbol, "triode");
}

#[test]
fn no_component_positions_are_nan() {
    let pedal = tube_screamer_pedal();
    let layout = generate_layout(&pedal, 2048.0, 1024.0);

    for comp in &layout.components {
        assert!(!comp.x.is_nan(), "Component {} x is NaN", comp.name);
        assert!(!comp.y.is_nan(), "Component {} y is NaN", comp.name);
        assert!(comp.x.is_finite(), "Component {} x is infinite", comp.name);
        assert!(comp.y.is_finite(), "Component {} y is infinite", comp.name);
    }
}

#[test]
fn no_wire_points_are_nan() {
    let pedal = tube_screamer_pedal();
    let layout = generate_layout(&pedal, 2048.0, 1024.0);

    for wire in &layout.wires {
        for pt in &wire.points {
            assert!(!pt[0].is_nan(), "Wire {} has NaN x", wire.net);
            assert!(!pt[1].is_nan(), "Wire {} has NaN y", wire.net);
        }
    }
}

#[test]
fn components_within_bounds() {
    let pedal = tube_screamer_pedal();
    let layout = generate_layout(&pedal, 2048.0, 1024.0);

    for comp in &layout.components {
        assert!(
            comp.x >= -50.0 && comp.x <= layout.bounds.width + 50.0,
            "Component {} x={} is outside bounds",
            comp.name,
            comp.x
        );
        assert!(
            comp.y >= -50.0 && comp.y <= layout.bounds.height + 50.0,
            "Component {} y={} is outside bounds",
            comp.name,
            comp.y
        );
    }
}

// ─── JSON output tests ──────────────────────────────────────────────────────

#[test]
fn json_roundtrips() {
    let pedal = tube_screamer_pedal();
    let layout = generate_layout(&pedal, 2048.0, 1024.0);
    let json = to_json(&layout);

    // Should be valid JSON
    let parsed: serde_json::Value = serde_json::from_str(&json).expect("JSON should parse");

    // Check top-level structure
    assert_eq!(parsed["version"], 1);
    assert!(parsed["components"].is_array());
    assert!(parsed["wires"].is_array());
    assert!(parsed["groups"].is_array());
    assert!(parsed["supply_rails"].is_array());
    assert!(parsed["signal_path_order"].is_array());
}

#[test]
fn json_components_have_required_fields() {
    let pedal = tube_screamer_pedal();
    let layout = generate_layout(&pedal, 2048.0, 1024.0);
    let json = to_json(&layout);
    let parsed: serde_json::Value = serde_json::from_str(&json).unwrap();

    for comp in parsed["components"].as_array().unwrap() {
        assert!(comp["name"].is_string(), "component missing name");
        assert!(comp["kind"].is_string(), "component missing kind");
        assert!(comp["x"].is_number(), "component missing x");
        assert!(comp["y"].is_number(), "component missing y");
        assert!(comp["orientation"].is_number(), "component missing orientation");
        assert!(comp["symbol"].is_string(), "component missing symbol");
        assert!(comp["group"].is_string(), "component missing group");
    }
}

// ─── KiCad export tests ─────────────────────────────────────────────────────

#[test]
fn kicad_export_produces_valid_schematic() {
    let pedal = tube_screamer_pedal();
    let layout = generate_layout(&pedal, 2048.0, 1024.0);
    let kicad = to_kicad_schematic(&layout, &pedal);

    assert!(kicad.starts_with("(kicad_sch"), "Should start with kicad_sch header");
    assert!(kicad.contains("(version 20230121)"), "Should have version");
    assert!(kicad.contains("(generator pedalkernel-layout)"), "Should credit generator");
    assert!(kicad.contains("(wire"), "Should have wire definitions");
    assert!(kicad.contains("(symbol"), "Should have symbol definitions");
}

#[test]
fn kicad_export_includes_all_components() {
    let pedal = triode_gain_stage();
    let layout = generate_layout(&pedal, 2048.0, 1024.0);
    let kicad = to_kicad_schematic(&layout, &pedal);

    // Each component should appear as a symbol in the KiCad output
    for comp in &pedal.components {
        assert!(
            kicad.contains(&comp.id),
            "KiCad output should contain component {}",
            comp.id
        );
    }
}

// ─── Layout quality tests ───────────────────────────────────────────────────

#[test]
fn signal_flows_left_to_right() {
    let pedal = triode_gain_stage();
    let layout = generate_layout(&pedal, 2048.0, 1024.0);

    // Find input-connected and output-connected components
    let c1 = layout.components.iter().find(|c| c.name == "C1").unwrap();
    let c3 = layout.components.iter().find(|c| c.name == "C3").unwrap();

    // Input coupling cap should be to the left of or at the same position as
    // output coupling cap. When both are in the same gain stage group they
    // may share a column, which is acceptable.
    assert!(
        c1.x <= c3.x,
        "Input cap C1 (x={}) should be left of or equal to output cap C3 (x={})",
        c1.x,
        c3.x
    );
}

#[test]
fn layout_score_is_finite() {
    let pedal = tube_screamer_pedal();
    let layout = generate_layout(&pedal, 2048.0, 1024.0);
    let score = pedalkernel_layout::optimize::layout_score(&layout);

    assert!(score.is_finite(), "Layout score should be finite, got {}", score);
}

#[test]
fn empty_pedal_produces_empty_layout() {
    let pedal = PedalDef {
        name: "Empty".into(),
        supplies: vec![],
        components: vec![],
        nets: vec![],
        controls: vec![],
        trims: vec![],
        monitors: vec![],
        sidechains: vec![],
    };

    let layout = generate_layout(&pedal, 1024.0, 512.0);

    assert_eq!(layout.components.len(), 0);
    assert!(layout.wires.is_empty());
    assert_eq!(layout.supply_rails.len(), 2);
}

// ─── Symbol tests ───────────────────────────────────────────────────────────

#[test]
fn all_symbol_types_have_paths() {
    use pedalkernel_layout::symbols::{symbol_paths, SymbolType};

    let types = [
        SymbolType::Resistor,
        SymbolType::Capacitor,
        SymbolType::ElectrolyticCap,
        SymbolType::Inductor,
        SymbolType::Diode,
        SymbolType::Zener,
        SymbolType::Triode,
        SymbolType::Pentode,
        SymbolType::NJfet,
        SymbolType::PJfet,
        SymbolType::NpnBjt,
        SymbolType::PnpBjt,
        SymbolType::Nmos,
        SymbolType::Pmos,
        SymbolType::OpAmp,
        SymbolType::Pot,
        SymbolType::Transformer,
        SymbolType::Photocoupler,
        SymbolType::IcChip,
        SymbolType::Ground,
        SymbolType::Supply,
        SymbolType::Speaker,
        SymbolType::Input,
        SymbolType::Output,
        SymbolType::Lfo,
        SymbolType::Neon,
        SymbolType::Switch,
    ];

    for sym in types {
        let paths = symbol_paths(sym);
        assert!(!paths.is_empty(), "Symbol {:?} should have path commands", sym);
    }
}
