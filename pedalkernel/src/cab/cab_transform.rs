//! Domain transform: CabDef → synthetic PedalDef → CompiledPedal.
//!
//! Converts the multi-domain cabinet model (electrical, mechanical, acoustic)
//! into a unified equivalent electrical circuit using standard impedance
//! analogies, then compiles it through the existing WDF pipeline.
//!
//! ## Impedance Analogy (BL Gyrator)
//!
//! The voice coil motor (BL transducer) acts as a gyrator between electrical
//! and mechanical domains. A series mechanical impedance reflects as a
//! parallel electrical admittance:
//!
//!   R_es  = BL² / Rms           (mechanical loss)
//!   C_mes = Mms / BL²           (moving mass)
//!   L_ces = Cms × BL²           (driver compliance)
//!
//! The acoustic domain reflects through Sd² into the mechanical domain,
//! then through BL² into the electrical domain.

use std::f64::consts::PI;

use crate::compiler::{compile_pedal, CompiledPedal};
use crate::dsl::{CapConfig, CapType, ControlDef, NetDef, PedalDef, Pin};

use super::cab_def::*;
use super::cab_validate::{resolved_ts, validate_cab};

// ═══════════════════════════════════════════════════════════════════════════
// Public API
// ═══════════════════════════════════════════════════════════════════════════

/// Compile a `.cab` definition into a real-time audio processor.
///
/// Validates the cab, resolves presets, derives T-S parameters, generates
/// a synthetic equivalent circuit as a PedalDef, and compiles it through
/// the existing WDF pipeline.
pub fn compile_cab(cab: &CabDef, sample_rate: f64) -> Result<CompiledPedal, String> {
    let mut cab = cab.clone();
    let _warnings = validate_cab(&mut cab)?;

    // Generate one processor per mic (or a single default if no mics)
    // For now, compile the first mic or default
    let pedal_def = generate_pedal_def(&cab, 0)?;

    compile_pedal(&pedal_def, sample_rate)
}

// ═══════════════════════════════════════════════════════════════════════════
// Synthetic PedalDef generation
// ═══════════════════════════════════════════════════════════════════════════

/// Generate a synthetic PedalDef representing the T-S equivalent circuit.
fn generate_pedal_def(cab: &CabDef, mic_index: usize) -> Result<PedalDef, String> {
    let driver = &cab.drivers[0]; // TODO: multi-driver summation
    let ts = resolved_ts(driver);
    let env = &cab.environment;

    let bl2 = ts.bl * ts.bl;
    let rho = env.air_density();
    let c = env.speed_of_sound();
    let a = piston_radius(ts.sd); // effective piston radius

    // ── Electrical domain (direct) ──────────────────────────────────────
    let r_e = ts.re;
    let l_e = ts.le;

    // ── Motional impedance (BL gyrator reflection) ──────────────────────
    // Series mechanical → parallel electrical
    let r_es = bl2 / ts.rms; // mechanical loss
    let c_mes = ts.mms / bl2; // moving mass
    let l_ces = ts.cms * bl2; // driver compliance

    // ── Enclosure compliance ────────────────────────────────────────────
    let l_box = match cab.enclosure.enclosure_type {
        EnclosureType::InfiniteBaffle => None,
        _ => {
            let v_box = cab.enclosure.volume.unwrap_or(0.045); // default 45L
            let c_box_mech = v_box / (rho * c * c * ts.sd * ts.sd);
            Some(c_box_mech * bl2)
        }
    };

    // ── Radiation impedance ─────────────────────────────────────────────
    // At high frequencies (ka >> 1): Z_rad ≈ ρc / (π·a²)
    // Reflected through Sd² and BL²:
    // R_rad_elec = (ρ·c / (π·a²)) × (BL² / Sd²)...
    // Actually, radiation impedance for a piston in an infinite baffle:
    //   Z_rad_acoustic = ρ·c / (π·a²) × (R1(2ka) + j·X1(2ka))
    // At low freq: R1 ≈ (ka)²/2, X1 ≈ 8ka/(3π)
    // We model this as parallel R_rad + L_rad where:
    //   R_rad represents the real (resistive) part at high freq
    //   L_rad represents the reactive (mass loading) part
    let r_rad_acoustic = rho * c / (PI * a * a);
    let l_rad_acoustic = 8.0 * rho / (3.0 * PI * PI * a);
    // Reflect to electrical domain: Z_elec = Z_acoustic × Sd² × BL²...
    // Wait — the reflection path is:
    //   acoustic → mechanical: multiply by Sd²
    //   mechanical → electrical: divide by Z (gyrator), so multiply admittance by BL²
    // For a gyrator: Z_elec = BL² / Z_mech, and Z_mech = Z_acoustic / Sd²
    // So Z_elec = BL² × Sd² / Z_acoustic
    // For parallel R_rad + L_rad in acoustic domain:
    //   Y_acoustic = 1/R_rad + 1/(jωL_rad)
    //   Z_acoustic = 1/Y_acoustic
    //   Z_elec = BL² × Sd² / Z_acoustic = BL² × Sd² × Y_acoustic
    //          = BL² × Sd² × (1/R_rad + 1/(jωL_rad))
    // This is a parallel R + L in the electrical domain:
    //   R_rad_elec = R_rad_acoustic / (BL² × Sd²)... hmm, that inverts.
    //
    // Let me reconsider. In the T-S model, the motional impedance Z_mot
    // appears as a parallel RLC. The radiation load adds to the mechanical
    // impedance (series with the mechanical branch). Through the gyrator:
    //   Z_mot = BL² / (Z_mech + Z_rad_mech)
    //   Z_rad_mech = Z_rad_acoustic × Sd²
    //
    // The radiation impedance adds R_rad and M_rad (mass) IN SERIES with
    // the mechanical elements. Through the gyrator, these become:
    //   Additional terms in the parallel motional branch:
    //   - Actually no — adding series impedance to the mechanical branch
    //     changes the overall Z_mot, but doesn't simply add parallel elements.
    //
    // For a first approximation, we model radiation as:
    // 1. Radiation mass adds to Mms: Mms_total = Mms + M_rad×Sd²... wait.
    //    Radiation mass M_rad_mech = (8*rho*a) / (3*PI) × Sd (one-sided)
    //    This adds to Mms, increasing it by ~5-15%.
    // 2. Radiation resistance provides the acoustic output — this is how
    //    power is transferred to air. In the equivalent circuit, we model
    //    it as a resistor in the motional branch.
    //
    // Simplification for the T-S model:
    //   C_mes_total = (Mms + M_rad) / BL²
    //   R_rad_elec = BL² / R_rad_mech  (as parallel element in motional branch)
    //   where R_rad_mech = R_rad_acoustic × Sd²
    //
    // R_rad_mech at HF (ka>>1) = rho*c*Sd (mechanical radiation resistance)
    // R_rad_mech at LF: much smaller, proportional to (ka)²
    //
    // For the WDF model, we use the HF value as a constant resistor.
    // The frequency-dependent behavior will be approximate but captures
    // the essential physics.
    let m_rad = 8.0 * rho * a / (3.0 * PI); // radiation mass per unit area
    let m_rad_total = m_rad * ts.sd; // total radiation mass on one side
    let r_rad_mech = rho * c * ts.sd; // radiation resistance at HF

    // Corrected motional parameters including radiation
    let c_mes_total = (ts.mms + m_rad_total) / bl2;
    let r_rad_elec = bl2 / r_rad_mech;

    // ── Enclosure-type-specific components ──────────────────────────────
    let open_back_leak = if cab.enclosure.enclosure_type == EnclosureType::OpenBack {
        let open_frac = cab.enclosure.open_area.unwrap_or(0.75);
        // Rear radiation through opening: lower impedance = more leakage
        // Model as a resistor in parallel with box compliance
        // Leak resistance decreases with opening size
        let r_leak = bl2 / (rho * c * ts.sd * open_frac.max(0.01));
        Some(r_leak)
    } else {
        None
    };

    // ── Build component list and netlist ────────────────────────────────
    let mut components = Vec::new();
    let mut nets = Vec::new();

    // Helper: add a component and return its ID
    let mut comp_id = 0usize;
    let mut next_id = |prefix: &str| -> String {
        comp_id += 1;
        format!("{prefix}_{comp_id}")
    };

    // Node names for the circuit
    // in -> Re -> Le -> junction_motional -> (parallel branch) -> gnd
    //                                    -> R_rad -> out

    // Re: voice coil resistance
    let re_id = next_id("Re");
    components.push(crate::dsl::ComponentDef {
        id: re_id.clone(),
        kind: Box::new(crate::compiler::components::Resistor { value: r_e }),
    });

    // Le: voice coil inductance
    let le_id = next_id("Le");
    components.push(crate::dsl::ComponentDef {
        id: le_id.clone(),
        kind: Box::new(crate::compiler::components::Inductor { value: l_e }),
    });

    // R_es: mechanical loss (parallel element)
    let res_id = next_id("Res");
    components.push(crate::dsl::ComponentDef {
        id: res_id.clone(),
        kind: Box::new(crate::compiler::components::Resistor { value: r_es }),
    });

    // C_mes: moving mass (parallel element, includes radiation mass)
    let cmes_id = next_id("Cmes");
    components.push(crate::dsl::ComponentDef {
        id: cmes_id.clone(),
        kind: Box::new(crate::compiler::components::Capacitor {
            config: CapConfig {
                value: c_mes_total,
                cap_type: CapType::Film,
                leakage: None,
                da: None,
            },
        }),
    });

    // L_ces: driver compliance (+ box compliance if sealed)
    let l_total = if let Some(lb) = l_box {
        l_ces + lb // series inductors add
    } else {
        l_ces
    };

    if cab.enclosure.enclosure_type == EnclosureType::OpenBack {
        // Open back: L_ces in parallel branch, L_box separately with leak
        let lces_id = next_id("Lces");
        components.push(crate::dsl::ComponentDef {
            id: lces_id.clone(),
            kind: Box::new(crate::compiler::components::Inductor { value: l_ces }),
        });

        if let Some(lb) = l_box {
            let lbox_id = next_id("Lbox");
            components.push(crate::dsl::ComponentDef {
                id: lbox_id.clone(),
                kind: Box::new(crate::compiler::components::Inductor { value: lb }),
            });

            // L_box in parallel with R_leak
            if let Some(r_leak) = open_back_leak {
                let rleak_id = next_id("Rleak");
                components.push(crate::dsl::ComponentDef {
                    id: rleak_id.clone(),
                    kind: Box::new(crate::compiler::components::Resistor {
                        value: r_leak,
                    }),
                });

                // Net: L_box and R_leak both from junction to ground
                nets.push(NetDef {
                    from: Pin::ComponentPin {
                        component: lbox_id.clone(),
                        pin: "a".to_string(),
                    },
                    to: vec![
                        Pin::ComponentPin {
                            component: rleak_id.clone(),
                            pin: "a".to_string(),
                        },
                        Pin::ComponentPin {
                            component: lces_id.clone(),
                            pin: "a".to_string(),
                        },
                    ],
                });
                nets.push(NetDef {
                    from: Pin::ComponentPin {
                        component: lbox_id,
                        pin: "b".to_string(),
                    },
                    to: vec![Pin::Reserved("gnd".to_string())],
                });
                nets.push(NetDef {
                    from: Pin::ComponentPin {
                        component: rleak_id,
                        pin: "b".to_string(),
                    },
                    to: vec![Pin::Reserved("gnd".to_string())],
                });
                nets.push(NetDef {
                    from: Pin::ComponentPin {
                        component: lces_id,
                        pin: "b".to_string(),
                    },
                    to: vec![Pin::Reserved("gnd".to_string())],
                });
            }
        }
    } else {
        // Sealed / other: combined L_ces + L_box
        let lces_id = next_id("Lces");
        components.push(crate::dsl::ComponentDef {
            id: lces_id.clone(),
            kind: Box::new(crate::compiler::components::Inductor {
                value: l_total,
            }),
        });

        // L_ces from junction_motional to gnd
        nets.push(NetDef {
            from: Pin::ComponentPin {
                component: lces_id,
                pin: "b".to_string(),
            },
            to: vec![Pin::Reserved("gnd".to_string())],
        });
    }

    // R_rad: radiation resistance (output extraction point)
    let rrad_id = next_id("Rrad");
    components.push(crate::dsl::ComponentDef {
        id: rrad_id.clone(),
        kind: Box::new(crate::compiler::components::Resistor {
            value: r_rad_elec,
        }),
    });

    // ── Cone breakup mode RLC branches ──────────────────────────────────
    let cone = driver.cone.as_ref().map(|c| c.clone()).unwrap_or_default();
    let breakup_freq = cone.breakup_freq.unwrap_or_else(|| {
        let diam = 2.0 * piston_radius(ts.sd);
        let ref_diam = 0.30; // 12 inch reference
        cone.material.default_breakup_freq_12in() * (ref_diam / diam)
    });

    let mut breakup_ids = Vec::new();
    for mode_n in 0..cone.breakup_modes.min(4) {
        let f_mode = breakup_freq * (1.0 + mode_n as f64 * 0.6); // modes spaced ~60% apart
        let q_mode = cone.breakup_q / (1.0 + mode_n as f64 * 0.3);
        let level = 10f64.powf(cone.breakup_level / 20.0) / (1.0 + mode_n as f64);

        // RLC values for a parallel resonator at f_mode with Q = q_mode
        // Normalized to match the motional impedance level
        let r_bp = r_es * level;
        let c_bp = 1.0 / (2.0 * PI * f_mode * r_bp * q_mode);
        let l_bp = r_bp * q_mode / (2.0 * PI * f_mode);

        let rbp_id = next_id("Rbp");
        let cbp_id = next_id("Cbp");
        let lbp_id = next_id("Lbp");

        components.push(crate::dsl::ComponentDef {
            id: rbp_id.clone(),
            kind: Box::new(crate::compiler::components::Resistor { value: r_bp }),
        });
        components.push(crate::dsl::ComponentDef {
            id: cbp_id.clone(),
            kind: Box::new(crate::compiler::components::Capacitor {
                config: CapConfig {
                    value: c_bp,
                    cap_type: CapType::Film,
                    leakage: None,
                    da: None,
                },
            }),
        });
        components.push(crate::dsl::ComponentDef {
            id: lbp_id.clone(),
            kind: Box::new(crate::compiler::components::Inductor { value: l_bp }),
        });

        breakup_ids.push((rbp_id, cbp_id, lbp_id));
    }

    // ── Build the netlist ───────────────────────────────────────────────

    // in -> Re.a
    nets.push(NetDef {
        from: Pin::Reserved("in".to_string()),
        to: vec![Pin::ComponentPin {
            component: re_id.clone(),
            pin: "a".to_string(),
        }],
    });

    // Re.b -> Le.a
    nets.push(NetDef {
        from: Pin::ComponentPin {
            component: re_id,
            pin: "b".to_string(),
        },
        to: vec![Pin::ComponentPin {
            component: le_id.clone(),
            pin: "a".to_string(),
        }],
    });

    // Le.b -> junction_motional (R_es.a, C_mes.a, R_rad.a + breakup branches)
    let mut junction_pins = vec![
        Pin::ComponentPin {
            component: res_id.clone(),
            pin: "a".to_string(),
        },
        Pin::ComponentPin {
            component: cmes_id.clone(),
            pin: "a".to_string(),
        },
        Pin::ComponentPin {
            component: rrad_id.clone(),
            pin: "a".to_string(),
        },
    ];

    // Add breakup branches to junction
    for (rbp_id, cbp_id, lbp_id) in &breakup_ids {
        junction_pins.push(Pin::ComponentPin {
            component: rbp_id.clone(),
            pin: "a".to_string(),
        });
        junction_pins.push(Pin::ComponentPin {
            component: cbp_id.clone(),
            pin: "a".to_string(),
        });
        junction_pins.push(Pin::ComponentPin {
            component: lbp_id.clone(),
            pin: "a".to_string(),
        });
    }

    // For open_back, L_ces is already connected above; for sealed, add it here
    if cab.enclosure.enclosure_type != EnclosureType::OpenBack {
        // L_ces.a is also at the junction
        let lces_existing_id = format!("Lces_{}", comp_id - breakup_ids.len() * 3 - 1);
        // Actually we need to track the L_ces ID properly. Let me use the component list.
        // The L_ces component was added with next_id("Lces"), so its ID is deterministic.
        // Let me find it from components.
        if let Some(lces_comp) = components.iter().find(|c| c.id.starts_with("Lces")) {
            junction_pins.push(Pin::ComponentPin {
                component: lces_comp.id.clone(),
                pin: "a".to_string(),
            });
        }
    }

    nets.push(NetDef {
        from: Pin::ComponentPin {
            component: le_id,
            pin: "b".to_string(),
        },
        to: junction_pins,
    });

    // Parallel elements .b -> gnd
    nets.push(NetDef {
        from: Pin::ComponentPin {
            component: res_id,
            pin: "b".to_string(),
        },
        to: vec![Pin::Reserved("gnd".to_string())],
    });
    nets.push(NetDef {
        from: Pin::ComponentPin {
            component: cmes_id,
            pin: "b".to_string(),
        },
        to: vec![Pin::Reserved("gnd".to_string())],
    });

    // Breakup branch .b -> gnd
    for (rbp_id, cbp_id, lbp_id) in &breakup_ids {
        nets.push(NetDef {
            from: Pin::ComponentPin {
                component: rbp_id.clone(),
                pin: "b".to_string(),
            },
            to: vec![Pin::Reserved("gnd".to_string())],
        });
        nets.push(NetDef {
            from: Pin::ComponentPin {
                component: cbp_id.clone(),
                pin: "b".to_string(),
            },
            to: vec![Pin::Reserved("gnd".to_string())],
        });
        nets.push(NetDef {
            from: Pin::ComponentPin {
                component: lbp_id.clone(),
                pin: "b".to_string(),
            },
            to: vec![Pin::Reserved("gnd".to_string())],
        });
    }

    // R_rad.b -> out
    nets.push(NetDef {
        from: Pin::ComponentPin {
            component: rrad_id,
            pin: "b".to_string(),
        },
        to: vec![Pin::Reserved("out".to_string())],
    });

    // ── Assemble PedalDef ───────────────────────────────────────────────
    Ok(PedalDef {
        name: format!("{} [cab]", cab.name),
        subtitle: None,
        supplies: Vec::new(),
        components,
        nets,
        controls: Vec::new(),
        trims: Vec::new(),
        monitors: Vec::new(),
        sidechains: Vec::new(),
        mirrors: std::collections::HashMap::new(),
        midi_bindings: Vec::new(),
        calibrate: false,
    })
}

// ═══════════════════════════════════════════════════════════════════════════
// Tests
// ═══════════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cab::cab_dsl::parse_cab_file;

    #[test]
    fn generate_sealed_cab_pedal_def() {
        let src = r#"
            cab "Test Sealed" {
                driver { preset: v30 }
                enclosure { type: sealed, volume: 45L }
            }
        "#;
        let cab = parse_cab_file(src).unwrap();
        let mut cab = cab;
        validate_cab(&mut cab).unwrap();
        let pedal = generate_pedal_def(&cab, 0).unwrap();

        // Should have components for Re, Le, R_es, C_mes, L_ces, R_rad + breakup modes
        assert!(
            pedal.components.len() >= 6,
            "Expected at least 6 components, got {}",
            pedal.components.len()
        );

        // Should have net connections
        assert!(!pedal.nets.is_empty());

        // Name should include [cab]
        assert!(pedal.name.contains("[cab]"));
    }

    #[test]
    fn compile_sealed_cab() {
        let src = r#"
            cab "Test Sealed" {
                driver { preset: v30 }
                enclosure { type: sealed, volume: 45L }
            }
        "#;
        let cab = parse_cab_file(src).unwrap();
        let result = compile_cab(&cab, 48000.0);
        assert!(result.is_ok(), "Sealed cab should compile: {}", result.err().unwrap_or_default());
    }

    #[test]
    fn compile_open_back_cab() {
        let src = r#"
            cab "Test Open" {
                driver { preset: c12n }
                enclosure {
                    type: open_back
                    volume: 42L
                    open_area: 80%
                    depth: 23cm
                }
            }
        "#;
        let cab = parse_cab_file(src).unwrap();
        let result = compile_cab(&cab, 48000.0);
        assert!(
            result.is_ok(),
            "Open back cab should compile: {}",
            result.err().unwrap_or_default()
        );
    }

    #[test]
    fn cab_produces_output() {
        use crate::PedalProcessor;

        let src = r#"
            cab "Test" {
                driver { preset: v30 }
                enclosure { type: sealed, volume: 45L }
            }
        "#;
        let cab = parse_cab_file(src).unwrap();
        let mut proc = compile_cab(&cab, 48000.0).unwrap();

        // Feed a sine wave
        let mut max_out = 0.0f64;
        for i in 0..4800 {
            let t = i as f64 / 48000.0;
            let input = 0.5 * (2.0 * PI * 440.0 * t).sin();
            let output = proc.process(input);
            if output.is_finite() {
                max_out = max_out.max(output.abs());
            }
        }

        assert!(
            max_out > 1e-6,
            "Cab should produce non-zero output, got max={max_out}"
        );
        assert!(
            max_out < 100.0,
            "Cab output should be bounded, got max={max_out}"
        );
    }
}
