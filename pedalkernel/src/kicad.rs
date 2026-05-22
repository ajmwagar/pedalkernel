//! KiCad netlist export from the parsed `.pedal` AST.
//!
//! Generates a KiCad-compatible netlist file (S-expression format) so
//! the same `.pedal` file can drive both tone-prototyping via WDF and
//! PCB layout via KiCad.

use crate::compiler::component::Component;
use crate::compiler::components::*;
use crate::dsl::*;
use std::fmt::Write;

/// Human-readable value string for a component.
fn value_str(kind: &dyn Component) -> String {
    // Switched components (check before generic resistance/capacitance/inductance)
    if let Some(rs) = kind.as_any().downcast_ref::<ResistorSwitched>() {
        let values = &rs.values;
        if values.is_empty() {
            return "R_switched".into();
        }
        return format!(
            "R_switched_{}-{}",
            format_eng(
                *values
                    .iter()
                    .min_by(|a, b| a.partial_cmp(b).unwrap())
                    .unwrap(),
                "Ω"
            ),
            format_eng(
                *values
                    .iter()
                    .max_by(|a, b| a.partial_cmp(b).unwrap())
                    .unwrap(),
                "Ω"
            )
        );
    }
    if let Some(cs) = kind.as_any().downcast_ref::<CapSwitched>() {
        let values = &cs.values;
        if values.is_empty() {
            return "C_switched".into();
        }
        return format!(
            "C_switched_{}-{}",
            format_eng(
                *values
                    .iter()
                    .min_by(|a, b| a.partial_cmp(b).unwrap())
                    .unwrap(),
                "F"
            ),
            format_eng(
                *values
                    .iter()
                    .max_by(|a, b| a.partial_cmp(b).unwrap())
                    .unwrap(),
                "F"
            )
        );
    }
    if let Some(ls) = kind.as_any().downcast_ref::<InductorSwitched>() {
        let values = &ls.values;
        if values.is_empty() {
            return "L_switched".into();
        }
        return format!(
            "L_switched_{}-{}",
            format_eng(
                *values
                    .iter()
                    .min_by(|a, b| a.partial_cmp(b).unwrap())
                    .unwrap(),
                "H"
            ),
            format_eng(
                *values
                    .iter()
                    .max_by(|a, b| a.partial_cmp(b).unwrap())
                    .unwrap(),
                "H"
            )
        );
    }

    // Tempco (check before generic resistance)
    if let Some(tc) = kind.as_any().downcast_ref::<Tempco>() {
        return format!("{}_Tempco_{:.0}ppm", format_eng(tc.resistance, "Ω"), tc.ppm);
    }

    // Diodes (check before generic diode_type to handle Zener and DiodePair)
    if let Some(z) = kind.as_any().downcast_ref::<Zener>() {
        return format!("{}V_Zener", z.breakdown_voltage);
    }
    if let Some(dt) = kind.diode_type() {
        if kind.as_any().downcast_ref::<DiodePair>().is_some() {
            return format!("{dt:?}_pair");
        }
        return format!("{dt:?}");
    }

    // Passive components with scalar values
    if let Some(r) = kind.resistance() {
        return format_eng(r, "Ω");
    }
    if let Some(c) = kind.capacitance() {
        return format_eng(c, "F");
    }
    if let Some(l) = kind.inductance() {
        return format_eng(l, "H");
    }

    // BJTs
    if kind.is_bjt() {
        if let Some(name) = kind.model_name() {
            return name.to_string();
        }
    }

    // Op-amp
    if let Some(ot) = kind.op_amp_type() {
        return match ot {
            OpAmpType::Generic => "OpAmp".into(),
            OpAmpType::Tl072 => "TL072".into(),
            OpAmpType::Tl082 => "TL082".into(),
            OpAmpType::Jrc4558 => "JRC4558D".into(),
            OpAmpType::Rc4558 => "RC4558".into(),
            OpAmpType::Lm308 => "LM308N".into(),
            OpAmpType::Lm741 => "LM741".into(),
            OpAmpType::Ne5532 => "NE5532".into(),
            OpAmpType::Ca3080 => "CA3080".into(),
            OpAmpType::Op07 => "OP07".into(),
        };
    }

    // JFETs
    if let Some(jfet) = kind.as_any().downcast_ref::<NJfet>() {
        return format!("N-JFET_{}", jfet.model);
    }
    if let Some(jfet) = kind.as_any().downcast_ref::<PJfet>() {
        return format!("P-JFET_{}", jfet.model);
    }

    // Photocoupler
    if let Some(pc) = kind.as_any().downcast_ref::<PhotocouplerComp>() {
        return format!("Vactrol_{:?}", pc.coupler_type);
    }

    // LFO
    if let Some(lfo) = kind.as_any().downcast_ref::<Lfo>() {
        return format!(
            "LFO_{:?}_R{}_C{}",
            lfo.waveform,
            format_eng(lfo.timing_r, ""),
            format_eng(lfo.timing_c, "")
        );
    }

    // Tubes
    if let Some(t) = kind.as_any().downcast_ref::<Triode>() {
        return format!("Triode_{:?}", t.model);
    }
    if let Some(vm) = kind.as_any().downcast_ref::<VariMu>() {
        return format!("VariMu_{}", vm.model);
    }
    if let Some(p) = kind.as_any().downcast_ref::<Pentode>() {
        return format!("Pentode_{:?}", p.model);
    }

    // Envelope follower
    if let Some(ef) = kind.as_any().downcast_ref::<EnvelopeFollower>() {
        return format!(
            "ENV_atk{}/{}_rel{}/{}_sens{}",
            format_eng(ef.attack_r, ""),
            format_eng(ef.attack_c, ""),
            format_eng(ef.release_r, ""),
            format_eng(ef.release_c, ""),
            format_eng(ef.sensitivity_r, ""),
        );
    }

    // MOSFETs
    if let Some(m) = kind.as_any().downcast_ref::<Nmos>() {
        return format!("N-MOS_{:?}", m.mosfet_type);
    }
    if let Some(m) = kind.as_any().downcast_ref::<Pmos>() {
        return format!("P-MOS_{:?}", m.mosfet_type);
    }

    // BBD
    if let Some(b) = kind.as_any().downcast_ref::<Bbd>() {
        return format!("BBD_{:?}", b.bbd_type);
    }

    // Delay line
    if let Some(dl) = kind.as_any().downcast_ref::<DelayLineComp>() {
        return format!(
            "DelayLine_{}-{}_{:?}_{:?}",
            format_eng(dl.min_delay, "s"),
            format_eng(dl.max_delay, "s"),
            dl.interpolation,
            dl.medium
        );
    }

    // Tap
    if let Some(tap) = kind.as_any().downcast_ref::<Tap>() {
        return format!("Tap_{}_{:.1}x", tap.parent_id, tap.ratio);
    }

    // Neon
    if let Some(n) = kind.as_any().downcast_ref::<Neon>() {
        return match n.neon_type {
            NeonType::Ne2 => "NE-2".into(),
            NeonType::Ne51 => "NE-51".into(),
            NeonType::Ne83 => "NE-83".into(),
        };
    }

    // Synth ICs
    if let Some(v) = kind.as_any().downcast_ref::<Vco>() {
        return format!("VCO_{:?}", v.vco_type);
    }
    if let Some(v) = kind.as_any().downcast_ref::<Vcf>() {
        return format!("VCF_{:?}", v.vcf_type);
    }
    if let Some(v) = kind.as_any().downcast_ref::<Vca>() {
        return format!("VCA_{:?}", v.vca_type);
    }
    if let Some(c) = kind.as_any().downcast_ref::<Comparator>() {
        return format!("Comparator_{:?}", c.comp_type);
    }
    if let Some(s) = kind.as_any().downcast_ref::<AnalogSwitch>() {
        return format!("Switch_{:?}", s.switch_type);
    }

    // Matched transistors
    if let Some(m) = kind.as_any().downcast_ref::<MatchedNpn>() {
        return format!("Matched_NPN_{:?}", m.matched_type);
    }
    if let Some(m) = kind.as_any().downcast_ref::<MatchedPnp>() {
        return format!("Matched_PNP_{:?}", m.matched_type);
    }

    // Transformer
    if let Some(cfg) = kind.transformer_config() {
        let pri = match cfg.primary_type {
            crate::dsl::WindingType::Standard => "",
            crate::dsl::WindingType::CenterTap => "CT",
            crate::dsl::WindingType::PushPull => "PP",
        };
        let sec = match cfg.secondary_type {
            crate::dsl::WindingType::Standard => "",
            crate::dsl::WindingType::CenterTap => "CT",
            crate::dsl::WindingType::PushPull => "PP",
        };
        return format!(
            "XFMR_{:.1}:1_{}{}_{}H",
            cfg.turns_ratio,
            pri,
            sec,
            format_eng(cfg.primary_inductance, "")
        );
    }

    // Rotary switch
    if let Some(sw) = kind.as_any().downcast_ref::<RotarySwitch>() {
        return format!("Rotary_{}pos", sw.linked_ids.len());
    }

    // Switch
    if let Some(sw) = kind.as_any().downcast_ref::<Switch>() {
        return format!("Switch_{}pos", sw.positions);
    }

    // Trigger input
    if kind.as_any().downcast_ref::<TriggerInputComp>().is_some() {
        return "TriggerInput".into();
    }

    // Fallback
    kind.type_tag().to_string()
}

/// Format a value with engineering suffix for display.
pub fn format_eng(val: f64, unit: &str) -> String {
    if val >= 1e6 {
        format!("{:.1}M{unit}", val / 1e6)
    } else if val >= 1e3 {
        format!("{:.1}k{unit}", val / 1e3)
    } else if val >= 1.0 {
        format!("{:.1}{unit}", val)
    } else if val >= 1e-3 {
        format!("{:.1}m{unit}", val * 1e3)
    } else if val >= 1e-6 {
        format!("{:.1}u{unit}", val * 1e6)
    } else if val >= 1e-9 {
        format!("{:.1}n{unit}", val * 1e9)
    } else {
        format!("{:.1}p{unit}", val * 1e12)
    }
}

/// Build a net map from the DSL net definitions.
/// Each unique junction point gets a net number.
fn build_net_map(nets: &[NetDef]) -> Vec<(usize, String, Vec<Pin>)> {
    // Collect all connected groups.  Each NetDef says "from connects to all of to".
    // We merge nets that share a pin.
    let mut groups: Vec<Vec<Pin>> = Vec::new();

    for net in nets {
        let mut all_pins: Vec<Pin> = vec![net.from.clone()];
        all_pins.extend(net.to.iter().cloned());

        // Find existing groups that share any of these pins
        let mut merge_indices: Vec<usize> = Vec::new();
        for (i, group) in groups.iter().enumerate() {
            if all_pins.iter().any(|p| group.contains(p)) {
                merge_indices.push(i);
            }
        }

        if merge_indices.is_empty() {
            groups.push(all_pins);
        } else {
            // Merge all matching groups + new pins into the first matching group
            merge_indices.sort();
            let target = merge_indices[0];
            for &idx in merge_indices.iter().skip(1).rev() {
                let g = groups.remove(idx);
                groups[target].extend(g);
            }
            for p in all_pins {
                if !groups[target].contains(&p) {
                    groups[target].push(p);
                }
            }
        }
    }

    // Assign net numbers (0 = unconnected, 1+ = nets)
    groups
        .into_iter()
        .enumerate()
        .map(|(i, pins)| {
            let net_num = i + 1;
            // Name the net after the first reserved node or first pin
            let name = pins
                .iter()
                .find_map(|p| {
                    if let Pin::Reserved(n) = p {
                        Some(n.clone())
                    } else {
                        None
                    }
                })
                .unwrap_or_else(|| format!("Net{net_num}"));
            (net_num, name, pins)
        })
        .collect()
}

fn pin_to_string(pin: &Pin) -> String {
    match pin {
        Pin::Reserved(n) => n.clone(),
        Pin::ComponentPin { component, pin } => format!("{component}.{pin}"),
        Pin::Fork {
            switch,
            destinations,
        } => {
            let dests: Vec<_> = destinations.iter().map(pin_to_string).collect();
            format!("fork({}, [{}])", switch, dests.join(", "))
        }
        Pin::SubcircuitPort { subcircuit, port } => format!("{subcircuit}.{port}"),
    }
}

/// Export a parsed pedal definition to KiCad netlist format (S-expression).
pub fn export_kicad_netlist(pedal: &PedalDef) -> String {
    let mut out = String::with_capacity(4096);

    writeln!(out, "(export (version D)").unwrap();
    writeln!(out, "  (design").unwrap();
    writeln!(out, "    (source \"{}.pedal\")", pedal.name).unwrap();
    writeln!(out, "    (tool \"PedalKernel DSL\"))").unwrap();

    // Components section
    writeln!(out, "  (components").unwrap();
    for comp in &pedal.components {
        if let Some(lfo) = comp.kind.as_any().downcast_ref::<Lfo>() {
            // LFO components expand into timing R and C for the physical circuit
            let waveform = &lfo.waveform;
            // Timing resistor
            writeln!(
                out,
                "    (comp (ref R_{id}) (value \"{val}\") (libsource (lib \"Device:R\")) (field (name \"Note\") \"LFO {wf:?} timing\"))",
                id = comp.id, val = format_eng(lfo.timing_r, "Ω"), wf = waveform
            )
            .unwrap();
            // Timing capacitor
            writeln!(
                out,
                "    (comp (ref C_{id}) (value \"{val}\") (libsource (lib \"Device:C\")) (field (name \"Note\") \"LFO {wf:?} timing\"))",
                id = comp.id, val = format_eng(lfo.timing_c, "F"), wf = waveform
            )
            .unwrap();
        } else if let Some(ef) = comp.kind.as_any().downcast_ref::<EnvelopeFollower>() {
            // Envelope follower expands into 5 physical components
            // Attack timing resistor
            writeln!(
                out,
                "    (comp (ref R_{id}_ATK) (value \"{val}\") (libsource (lib \"Device:R\")) (field (name \"Note\") \"Envelope attack timing\"))",
                id = comp.id, val = format_eng(ef.attack_r, "Ω")
            )
            .unwrap();
            // Attack timing capacitor
            writeln!(
                out,
                "    (comp (ref C_{id}_ATK) (value \"{val}\") (libsource (lib \"Device:C\")) (field (name \"Note\") \"Envelope attack timing\"))",
                id = comp.id, val = format_eng(ef.attack_c, "F")
            )
            .unwrap();
            // Release timing resistor
            writeln!(
                out,
                "    (comp (ref R_{id}_REL) (value \"{val}\") (libsource (lib \"Device:R\")) (field (name \"Note\") \"Envelope release timing\"))",
                id = comp.id, val = format_eng(ef.release_r, "Ω")
            )
            .unwrap();
            // Release timing capacitor
            writeln!(
                out,
                "    (comp (ref C_{id}_REL) (value \"{val}\") (libsource (lib \"Device:C\")) (field (name \"Note\") \"Envelope release timing\"))",
                id = comp.id, val = format_eng(ef.release_c, "F")
            )
            .unwrap();
            // Sensitivity resistor
            writeln!(
                out,
                "    (comp (ref R_{id}_SENS) (value \"{val}\") (libsource (lib \"Device:R\")) (field (name \"Note\") \"Envelope sensitivity\"))",
                id = comp.id, val = format_eng(ef.sensitivity_r, "Ω")
            )
            .unwrap();
            // Rectifier diode (part of the physical envelope detector)
            writeln!(
                out,
                "    (comp (ref D_{id}) (value \"1N4148\") (libsource (lib \"Device:D\")) (field (name \"Note\") \"Envelope rectifier\"))",
                id = comp.id
            )
            .unwrap();
        } else {
            let (lib_ref, _prefix) = comp.kind.footprint_ref();
            let val = value_str(&*comp.kind);
            writeln!(
                out,
                "    (comp (ref {id}) (value \"{val}\") (libsource (lib \"{lib_ref}\")))",
                id = comp.id
            )
            .unwrap();
        }
    }
    writeln!(out, "  )").unwrap();

    // Nets section
    let net_map = build_net_map(&pedal.nets);
    writeln!(out, "  (nets").unwrap();
    for (num, name, pins) in &net_map {
        write!(out, "    (net (code {num}) (name \"{name}\")").unwrap();
        for pin in pins {
            write!(out, " (node (ref \"{}\"))", pin_to_string(pin)).unwrap();
        }
        writeln!(out, ")").unwrap();
    }
    writeln!(out, "  )").unwrap();

    writeln!(out, ")").unwrap();
    out
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn format_eng_values() {
        assert_eq!(format_eng(4700.0, "Ω"), "4.7kΩ");
        assert_eq!(format_eng(220e-9, "F"), "220.0nF");
        assert_eq!(format_eng(0.1, "H"), "100.0mH");
        assert_eq!(format_eng(1e6, "Ω"), "1.0MΩ");
    }

    #[test]
    fn export_tube_screamer() {
        let pedal = PedalDef {
            name: "Tube Screamer".into(),
            subtitle: None,
            components: vec![
                ComponentDef {
                    id: "R1".into(),
                    kind: Box::new(Resistor { value: 4700.0 }),
                },
                ComponentDef {
                    id: "C1".into(),
                    kind: Box::new(Capacitor {
                        config: CapConfig::new(220e-9),
                    }),
                },
                ComponentDef {
                    id: "D1".into(),
                    kind: Box::new(DiodePair {
                        diode_type: DiodeType::Silicon,
                    }),
                },
            ],
            nets: vec![
                NetDef {
                    from: Pin::Reserved("in".into()),
                    to: vec![Pin::ComponentPin {
                        component: "C1".into(),
                        pin: "a".into(),
                    }],
                },
                NetDef {
                    from: Pin::ComponentPin {
                        component: "C1".into(),
                        pin: "b".into(),
                    },
                    to: vec![
                        Pin::ComponentPin {
                            component: "R1".into(),
                            pin: "a".into(),
                        },
                        Pin::ComponentPin {
                            component: "D1".into(),
                            pin: "a".into(),
                        },
                    ],
                },
            ],
            controls: vec![],
            supplies: vec![],
            trims: vec![],
            monitors: vec![],
            sidechains: vec![],
            mirrors: hashbrown::HashMap::new(),
            calibrate: false,
            subcircuits: vec![],
            ports: vec![],
            init_hints: vec![],
        };

        let netlist = export_kicad_netlist(&pedal);
        assert!(netlist.contains("(export (version D)"));
        assert!(netlist.contains("4.7kΩ"));
        assert!(netlist.contains("220.0nF"));
        assert!(netlist.contains("(net (code"));
    }

    #[test]
    fn net_merging() {
        // Two nets that share a pin should merge
        let nets = vec![
            NetDef {
                from: Pin::Reserved("in".into()),
                to: vec![Pin::ComponentPin {
                    component: "C1".into(),
                    pin: "a".into(),
                }],
            },
            NetDef {
                from: Pin::ComponentPin {
                    component: "C1".into(),
                    pin: "a".into(),
                },
                to: vec![Pin::ComponentPin {
                    component: "R1".into(),
                    pin: "a".into(),
                }],
            },
        ];
        let map = build_net_map(&nets);
        // Should be a single net group
        assert_eq!(map.len(), 1);
        assert_eq!(map[0].2.len(), 3); // in, C1.a, R1.a
    }
}
