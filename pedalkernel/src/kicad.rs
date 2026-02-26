//! KiCad netlist export from the parsed `.pedal` AST.
//!
//! Generates a KiCad-compatible netlist file (S-expression format) so
//! the same `.pedal` file can drive both tone-prototyping via WDF and
//! PCB layout via KiCad.

use crate::dsl::*;
use std::fmt::Write;

/// Map a DSL component kind to a KiCad symbol library reference.
fn footprint_ref(kind: &ComponentKind) -> (&str, &str) {
    match kind {
        ComponentKind::Resistor(_) => ("Device:R", "R"),
        ComponentKind::Capacitor(_) => ("Device:C", "C"),
        ComponentKind::Inductor(_) => ("Device:L", "L"),
        ComponentKind::DiodePair(_) => ("Device:D", "D"), // pair shown as single symbol + note
        ComponentKind::Diode(_) => ("Device:D", "D"),
        ComponentKind::Zener(_) => ("Device:D_Zener", "D"),
        ComponentKind::Potentiometer(_) => ("Device:R_Potentiometer", "RV"),
        ComponentKind::Npn(_) => ("Device:Q_NPN_BCE", "Q"),
        ComponentKind::Pnp(_) => ("Device:Q_PNP_BCE", "Q"),
        ComponentKind::OpAmp(ot) => {
            let lib = match ot {
                OpAmpType::Generic => "Amplifier_Operational:TL072",
                OpAmpType::Tl072 => "Amplifier_Operational:TL072",
                OpAmpType::Tl082 => "Amplifier_Operational:TL082",
                OpAmpType::Jrc4558 => "Amplifier_Operational:JRC4558",
                OpAmpType::Rc4558 => "Amplifier_Operational:RC4558",
                OpAmpType::Lm308 => "Amplifier_Operational:LM308",
                OpAmpType::Lm741 => "Amplifier_Operational:LM741",
                OpAmpType::Ne5532 => "Amplifier_Operational:NE5532",
                OpAmpType::Ca3080 => "Amplifier_Operational:CA3080",
                OpAmpType::Op07 => "Amplifier_Operational:OP07",
            };
            (lib, "U")
        }
        ComponentKind::NJfet(_) => ("Device:Q_NJFET_DGS", "J"),
        ComponentKind::PJfet(_) => ("Device:Q_PJFET_DGS", "J"),
        ComponentKind::Photocoupler(_) => ("Isolator:PC817", "OC"),
        ComponentKind::Lfo(..) => ("", "LFO"), // Expands to timing R + C
        ComponentKind::Triode(tt) => match tt {
            TriodeType::T12ax7 => ("Valve:ECC83", "V"),
            TriodeType::T12at7 => ("Valve:ECC81", "V"),
            TriodeType::T12au7 => ("Valve:ECC82", "V"),
            TriodeType::T12ay7 => ("Valve:12AY7", "V"),
            TriodeType::T12bh7 => ("Valve:12BH7", "V"),
            TriodeType::T6386 => ("Valve:6386", "V"),
        },
        ComponentKind::Pentode(pt) => match pt {
            PentodeType::Ef86 => ("Valve:EF86", "V"),
            PentodeType::El84 => ("Valve:EL84", "V"),
            PentodeType::A6aq5a => ("Valve:6AQ5", "V"),
            PentodeType::A6973 => ("Valve:6973", "V"),
            PentodeType::A6l6gc => ("Valve:6L6GC", "V"),
            PentodeType::El34 => ("Valve:EL34", "V"),
            PentodeType::A6550 => ("Valve:6550", "V"),
        },
        ComponentKind::EnvelopeFollower(..) => ("", "ENV"), // Expands to RC timing components
        ComponentKind::Nmos(_) => ("Device:Q_NMOS_DGS", "Q"),
        ComponentKind::Pmos(_) => ("Device:Q_PMOS_DGS", "Q"),
        ComponentKind::Bbd(bt) => match bt {
            BbdType::Mn3207 => ("Analog_Delay:MN3207", "IC"),
            BbdType::Mn3007 => ("Analog_Delay:MN3007", "IC"),
            BbdType::Mn3005 => ("Analog_Delay:MN3005", "IC"),
        },
        // DelayLine and Tap are virtual WDF elements — no physical schematic symbol.
        // They map to a comment/annotation in KiCad.
        ComponentKind::DelayLine(..) => ("", "DL"),
        ComponentKind::Tap(..) => ("", "TAP"),
        ComponentKind::Neon(nt) => match nt {
            NeonType::Ne2 => ("Device:Lamp_Neon", "NE"),
            NeonType::Ne51 => ("Device:Lamp_Neon", "NE"),
            NeonType::Ne83 => ("Device:Lamp_Neon", "NE"),
        },
        // ── Synth ICs ──────────────────────────────────────────────────
        ComponentKind::Vco(vt) => match vt {
            VcoType::Cem3340 => ("Oscillator:CEM3340", "U"),
            VcoType::As3340 => ("Oscillator:AS3340", "U"),
            VcoType::V3340 => ("Oscillator:V3340", "U"),
        },
        ComponentKind::Vcf(vt) => match vt {
            VcfType::Cem3320 => ("Analog:CEM3320", "U"),
            VcfType::As3320 => ("Analog:AS3320", "U"),
        },
        ComponentKind::Vca(vt) => match vt {
            VcaType::Ssm2164 => ("Analog:SSM2164", "U"),
            VcaType::V2164 => ("Analog:V2164", "U"),
        },
        ComponentKind::Comparator(ct) => match ct {
            ComparatorType::Lm311 => ("Comparator:LM311", "U"),
            ComparatorType::Lm393 => ("Comparator:LM393", "U"),
        },
        ComponentKind::AnalogSwitch(st) => match st {
            AnalogSwitchType::Cd4066 => ("Analog_Switch:CD4066", "U"),
            AnalogSwitchType::Dg411 => ("Analog_Switch:DG411", "U"),
        },
        ComponentKind::MatchedNpn(mt) | ComponentKind::MatchedPnp(mt) => match mt {
            MatchedTransistorType::Ssm2210 => ("Transistor_BJT:SSM2210", "Q"),
            MatchedTransistorType::Ca3046 => ("Transistor_Array:CA3046", "Q"),
            MatchedTransistorType::Lm394 => ("Transistor_BJT:LM394", "Q"),
            MatchedTransistorType::That340 => ("Transistor_BJT:THAT340", "Q"),
        },
        ComponentKind::Tempco(_, _) => ("Device:R", "RT"),
        // ── Studio Equipment ─────────────────────────────────────────────
        ComponentKind::Transformer(cfg) => {
            // Select transformer symbol based on configuration
            match (cfg.primary_type, cfg.secondary_type) {
                (WindingType::PushPull, WindingType::Standard) => ("Transformer:Transformer_1P_1S", "T"),
                (WindingType::Standard, WindingType::CenterTap) => ("Transformer:Transformer_1P_1S_CT", "T"),
                (WindingType::PushPull, WindingType::CenterTap) => ("Transformer:Transformer_1P_CT_1S_CT", "T"),
                _ => ("Transformer:Transformer_1P_1S", "T"),
            }
        }
        ComponentKind::CapSwitched(values) => {
            // Switched capacitor - show as regular cap with note about switch positions
            if values.is_empty() {
                ("Device:C", "C")
            } else {
                ("Device:C", "C")
            }
        }
        ComponentKind::InductorSwitched(values) => {
            // Multi-tap inductor - show as regular inductor with note about taps
            if values.is_empty() {
                ("Device:L", "L")
            } else {
                ("Device:L", "L")
            }
        }
        ComponentKind::RotarySwitch(ids) => {
            // Rotary switch - positions determined by linked components
            let positions = ids.len().max(2); // Minimum 2 positions
            match positions {
                2..=4 => ("Switch:SW_Rotary4", "SW"),
                5..=6 => ("Switch:SW_Rotary6", "SW"),
                7..=8 => ("Switch:SW_Rotary8", "SW"),
                _ => ("Switch:SW_Rotary12", "SW"),
            }
        }
        ComponentKind::ResistorSwitched(values) => {
            // Switched resistor - show as regular resistor with note about switch positions
            if values.is_empty() {
                ("Device:R", "R")
            } else {
                ("Device:R", "R")
            }
        }
        ComponentKind::Switch(positions) => {
            // Simple mechanical switch
            match positions {
                2 => ("Switch:SW_SPDT", "SW"),
                3 => ("Switch:SW_Rotary4", "SW"),
                _ => ("Switch:SW_Rotary4", "SW"),
            }
        }
    }
}

/// Human-readable value string for a component.
fn value_str(kind: &ComponentKind) -> String {
    match kind {
        ComponentKind::Resistor(v) => format_eng(*v, "Ω"),
        ComponentKind::Capacitor(cfg) => format_eng(cfg.value, "F"),
        ComponentKind::Inductor(v) => format_eng(*v, "H"),
        ComponentKind::DiodePair(dt) => format!("{dt:?}_pair"),
        ComponentKind::Diode(dt) => format!("{dt:?}"),
        ComponentKind::Zener(v) => format!("{v}V_Zener"),
        ComponentKind::Potentiometer(v) => format_eng(*v, "Ω"),
        ComponentKind::Npn(bt) => match bt {
            BjtType::Generic => "NPN".into(),
            BjtType::N2n3904 => "2N3904".into(),
            BjtType::N2n2222 => "2N2222".into(),
            BjtType::Bc108 => "BC108".into(),
            BjtType::Bc109 => "BC109".into(),
            BjtType::N2n5088 => "2N5088".into(),
            BjtType::N2n5089 => "2N5089".into(),
            BjtType::N2n3906 => "2N3906".into(),
            BjtType::Ac128 => "AC128".into(),
            BjtType::Oc44 => "OC44".into(),
            BjtType::Nkt275 => "NKT275".into(),
        },
        ComponentKind::Pnp(bt) => match bt {
            BjtType::Generic => "PNP".into(),
            BjtType::N2n3904 => "2N3904".into(),
            BjtType::N2n2222 => "2N2222".into(),
            BjtType::Bc108 => "BC108".into(),
            BjtType::Bc109 => "BC109".into(),
            BjtType::N2n5088 => "2N5088".into(),
            BjtType::N2n5089 => "2N5089".into(),
            BjtType::N2n3906 => "2N3906".into(),
            BjtType::Ac128 => "AC128".into(),
            BjtType::Oc44 => "OC44".into(),
            BjtType::Nkt275 => "NKT275".into(),
        },
        ComponentKind::OpAmp(ot) => match ot {
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
        },
        ComponentKind::NJfet(jt) => format!("N-JFET_{jt:?}"),
        ComponentKind::PJfet(jt) => format!("P-JFET_{jt:?}"),
        ComponentKind::Photocoupler(pt) => format!("Vactrol_{pt:?}"),
        ComponentKind::Lfo(wf, timing_r, timing_c) => {
            // Show waveform and timing components for BOM reference
            format!(
                "LFO_{wf:?}_R{}_C{}",
                format_eng(*timing_r, ""),
                format_eng(*timing_c, "")
            )
        }
        ComponentKind::Triode(tt) => format!("Triode_{tt:?}"),
        ComponentKind::Pentode(pt) => format!("Pentode_{pt:?}"),
        ComponentKind::EnvelopeFollower(attack_r, attack_c, release_r, release_c, sens_r) => {
            format!(
                "ENV_atk{}/{}_rel{}/{}_sens{}",
                format_eng(*attack_r, ""),
                format_eng(*attack_c, ""),
                format_eng(*release_r, ""),
                format_eng(*release_c, ""),
                format_eng(*sens_r, ""),
            )
        }
        ComponentKind::Nmos(mt) => format!("N-MOS_{mt:?}"),
        ComponentKind::Pmos(mt) => format!("P-MOS_{mt:?}"),
        ComponentKind::Bbd(bt) => format!("BBD_{bt:?}"),
        ComponentKind::DelayLine(min, max, interp, medium) => {
            format!("DelayLine_{}-{}_{:?}_{:?}", format_eng(*min, "s"), format_eng(*max, "s"), interp, medium)
        }
        ComponentKind::Tap(parent, ratio) => format!("Tap_{}_{:.1}x", parent, ratio),
        ComponentKind::Neon(nt) => match nt {
            NeonType::Ne2 => "NE-2".into(),
            NeonType::Ne51 => "NE-51".into(),
            NeonType::Ne83 => "NE-83".into(),
        },
        // ── Synth ICs ──────────────────────────────────────────────────
        ComponentKind::Vco(vt) => format!("VCO_{vt:?}"),
        ComponentKind::Vcf(vt) => format!("VCF_{vt:?}"),
        ComponentKind::Vca(vt) => format!("VCA_{vt:?}"),
        ComponentKind::Comparator(ct) => format!("Comparator_{ct:?}"),
        ComponentKind::AnalogSwitch(st) => format!("Switch_{st:?}"),
        ComponentKind::MatchedNpn(mt) => format!("Matched_NPN_{mt:?}"),
        ComponentKind::MatchedPnp(mt) => format!("Matched_PNP_{mt:?}"),
        ComponentKind::Tempco(r, ppm) => format!(
            "{}_Tempco_{:.0}ppm",
            format_eng(*r, "\u{2126}"),
            ppm
        ),
        // ── Studio Equipment Components ──────────────────────────────
        ComponentKind::Transformer(cfg) => {
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
            format!(
                "XFMR_{:.1}:1_{}{}_{}H",
                cfg.turns_ratio,
                pri,
                sec,
                format_eng(cfg.primary_inductance, "")
            )
        }
        ComponentKind::CapSwitched(values) => {
            // Show range for BOM reference
            if values.is_empty() {
                "C_switched".into()
            } else {
                format!(
                    "C_switched_{}-{}",
                    format_eng(*values.iter().min_by(|a, b| a.partial_cmp(b).unwrap()).unwrap(), "F"),
                    format_eng(*values.iter().max_by(|a, b| a.partial_cmp(b).unwrap()).unwrap(), "F")
                )
            }
        }
        ComponentKind::InductorSwitched(values) => {
            // Show range for BOM reference
            if values.is_empty() {
                "L_switched".into()
            } else {
                format!(
                    "L_switched_{}-{}",
                    format_eng(*values.iter().min_by(|a, b| a.partial_cmp(b).unwrap()).unwrap(), "H"),
                    format_eng(*values.iter().max_by(|a, b| a.partial_cmp(b).unwrap()).unwrap(), "H")
                )
            }
        }
        ComponentKind::RotarySwitch(positions) => {
            format!("Rotary_{}pos", positions.len())
        }
        ComponentKind::ResistorSwitched(values) => {
            // Show range for BOM reference
            if values.is_empty() {
                "R_switched".into()
            } else {
                format!(
                    "R_switched_{}-{}",
                    format_eng(*values.iter().min_by(|a, b| a.partial_cmp(b).unwrap()).unwrap(), "Ω"),
                    format_eng(*values.iter().max_by(|a, b| a.partial_cmp(b).unwrap()).unwrap(), "Ω")
                )
            }
        }
        ComponentKind::Switch(positions) => {
            format!("Switch_{}pos", positions)
        }
    }
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
        match &comp.kind {
            // LFO components expand into timing R and C for the physical circuit
            ComponentKind::Lfo(waveform, timing_r, timing_c) => {
                // Timing resistor
                writeln!(
                    out,
                    "    (comp (ref R_{id}) (value \"{val}\") (libsource (lib \"Device:R\")) (field (name \"Note\") \"LFO {wf:?} timing\"))",
                    id = comp.id, val = format_eng(*timing_r, "Ω"), wf = waveform
                )
                .unwrap();
                // Timing capacitor
                writeln!(
                    out,
                    "    (comp (ref C_{id}) (value \"{val}\") (libsource (lib \"Device:C\")) (field (name \"Note\") \"LFO {wf:?} timing\"))",
                    id = comp.id, val = format_eng(*timing_c, "F"), wf = waveform
                )
                .unwrap();
            }
            // Envelope follower expands into 5 physical components
            ComponentKind::EnvelopeFollower(attack_r, attack_c, release_r, release_c, sens_r) => {
                // Attack timing resistor
                writeln!(
                    out,
                    "    (comp (ref R_{id}_ATK) (value \"{val}\") (libsource (lib \"Device:R\")) (field (name \"Note\") \"Envelope attack timing\"))",
                    id = comp.id, val = format_eng(*attack_r, "Ω")
                )
                .unwrap();
                // Attack timing capacitor
                writeln!(
                    out,
                    "    (comp (ref C_{id}_ATK) (value \"{val}\") (libsource (lib \"Device:C\")) (field (name \"Note\") \"Envelope attack timing\"))",
                    id = comp.id, val = format_eng(*attack_c, "F")
                )
                .unwrap();
                // Release timing resistor
                writeln!(
                    out,
                    "    (comp (ref R_{id}_REL) (value \"{val}\") (libsource (lib \"Device:R\")) (field (name \"Note\") \"Envelope release timing\"))",
                    id = comp.id, val = format_eng(*release_r, "Ω")
                )
                .unwrap();
                // Release timing capacitor
                writeln!(
                    out,
                    "    (comp (ref C_{id}_REL) (value \"{val}\") (libsource (lib \"Device:C\")) (field (name \"Note\") \"Envelope release timing\"))",
                    id = comp.id, val = format_eng(*release_c, "F")
                )
                .unwrap();
                // Sensitivity resistor
                writeln!(
                    out,
                    "    (comp (ref R_{id}_SENS) (value \"{val}\") (libsource (lib \"Device:R\")) (field (name \"Note\") \"Envelope sensitivity\"))",
                    id = comp.id, val = format_eng(*sens_r, "Ω")
                )
                .unwrap();
                // Rectifier diode (part of the physical envelope detector)
                writeln!(
                    out,
                    "    (comp (ref D_{id}) (value \"1N4148\") (libsource (lib \"Device:D\")) (field (name \"Note\") \"Envelope rectifier\"))",
                    id = comp.id
                )
                .unwrap();
            }
            _ => {
                let (lib_ref, _prefix) = footprint_ref(&comp.kind);
                let val = value_str(&comp.kind);
                writeln!(
                    out,
                    "    (comp (ref {id}) (value \"{val}\") (libsource (lib \"{lib_ref}\")))",
                    id = comp.id
                )
                .unwrap();
            }
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
            components: vec![
                ComponentDef {
                    id: "R1".into(),
                    kind: ComponentKind::Resistor(4700.0),
                },
                ComponentDef {
                    id: "C1".into(),
                    kind: ComponentKind::Capacitor(CapConfig::new(220e-9)),
                },
                ComponentDef {
                    id: "D1".into(),
                    kind: ComponentKind::DiodePair(DiodeType::Silicon),
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
            supply: None,
            trims: vec![],
            monitors: vec![],
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
