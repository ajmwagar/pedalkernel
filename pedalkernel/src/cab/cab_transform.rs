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

use crate::compiler::components::TransformerComp;
use crate::compiler::{compile_pedal, CompiledPedal};
use crate::dsl::{CapConfig, CapType, ControlDef, NetDef, PedalDef, Pin, TransformerConfig};

use super::cab_def::*;
use super::cab_validate::{calculate_panel_modes, resolved_ts, validate_cab};

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

    // ── Sealed enclosure leakage ────────────────────────────────────────
    // All sealed enclosures have some air leakage. This is modeled as a
    // high-value resistor in parallel with the box compliance, representing
    // the slow pressure equalization through gaps. Default: ~10 MΩ (well sealed).
    let sealed_leak = if cab.enclosure.enclosure_type == EnclosureType::Sealed {
        // User-specified leakage or default 10M acoustic ohms
        let r_leak_acoustic = cab.enclosure.leakage.unwrap_or(10e6);
        // Reflect to electrical domain: R_leak_elec = BL² / (R_leak_acoustic × Sd²)
        // This is a very high resistance, minimal effect except at very low frequencies
        Some(bl2 / (r_leak_acoustic * ts.sd * ts.sd).max(1e-12))
    } else {
        None
    };

    // ── Bass reflex port (Helmholtz resonator) ────────────────────────
    // The port acts as an acoustic mass (inductor) with viscous losses (resistor)
    // in series, placed in parallel with the box compliance.
    // Tuning freq: fb = (c / 2π) × √(Sp / (lp × Vb))
    // where Sp = port area, lp = effective port length, Vb = box volume
    let port_rl = if cab.enclosure.enclosure_type == EnclosureType::BassReflex {
        if let Some(ref port) = cab.enclosure.port {
            let v_box = cab.enclosure.volume.unwrap_or(0.045);
            // Port area
            let sp = if let Some(d) = port.diameter {
                PI * (d / 2.0) * (d / 2.0)
            } else if let (Some(w), Some(h)) = (port.width, port.height) {
                w * h
            } else {
                PI * 0.025 * 0.025 // default ~50mm diameter
            };
            // Effective port length includes end corrections (~0.85 × diameter per end)
            let port_radius = (sp / PI).sqrt();
            let l_eff = port.length + 1.7 * port_radius;
            // Acoustic mass of port air: Ma = ρ × l_eff / Sp
            let m_port_acoustic = rho * l_eff / sp;
            // Reflect to electrical domain via Sd² and BL²
            // Port mechanical mass = Ma × Sp² (acoustic → mechanical)
            // Port electrical capacitance = M_port_mech / BL²
            let c_port_elec = m_port_acoustic * sp * sp / bl2;
            // Viscous losses in port: Rv ≈ 8πμl / Sp (Poiseuille-like)
            // μ_air ≈ 1.8e-5 Pa·s
            let mu_air = 1.8e-5;
            let r_port_acoustic = 8.0 * PI * mu_air * l_eff / (sp * sp);
            let r_port_elec = bl2 / (r_port_acoustic * ts.sd * ts.sd).max(1e-12);
            Some((r_port_elec, c_port_elec))
        } else {
            None
        }
    } else {
        None
    };

    // ── Grille filtering ──────────────────────────────────────────────
    // Grille cloth/metal adds a small series R + L between the radiation
    // resistance and the output, creating a gentle low-pass filter.
    let grille = cab.enclosure.baffle.as_ref().map(|b| b.grille).unwrap_or_default();
    let grille_r = grille.resistance_factor() * r_rad_elec;
    let grille_l = grille.mass_factor() * l_e; // scale relative to voice coil Le

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
    // Cone material affects breakup: stiffer materials push breakup higher,
    // higher damping factor reduces Q of breakup resonances.
    let cone = driver.cone.as_ref().cloned().unwrap_or_default();
    let breakup_freq = cone.breakup_freq.unwrap_or_else(|| {
        let diam = 2.0 * piston_radius(ts.sd);
        let ref_diam = 0.30; // 12 inch reference
        // Dust cap shifts breakup: aluminum cap raises it, felt lowers it
        // Surround HF loss effectively lowers the audible breakup contribution
        let base_freq = cone.material.default_breakup_freq_12in() * (ref_diam / diam);
        base_freq * cone.dust_cap.breakup_freq_multiplier()
    });

    // Material properties modulate the user-specified (or default) Q and level:
    //   - Higher damping_factor → lower Q (more damped breakup peaks)
    //   - Dust cap HF damping adds to overall breakup damping
    //   - Surround HF loss reduces effective breakup level
    // Reference: paper cone (damping=0.08, stiffness=1.0)
    let mat_damping = cone.material.damping_factor() + cone.dust_cap.hf_damping();
    let ref_damping = ConeMaterial::Paper.damping_factor(); // 0.08
    let damping_ratio = ref_damping / mat_damping.max(1e-6);
    let surround_atten = 1.0 - cone.surround.hf_loss(); // surround HF loss reduces breakup level

    let mut breakup_ids = Vec::new();
    for mode_n in 0..cone.breakup_modes.min(4) {
        let f_mode = breakup_freq * (1.0 + mode_n as f64 * 0.6);
        // Scale Q by inverse damping ratio: aluminum (low damping) → higher Q peaks
        let q_mode = (cone.breakup_q * damping_ratio) / (1.0 + mode_n as f64 * 0.3);
        let level =
            10f64.powf(cone.breakup_level / 20.0) * surround_atten / (1.0 + mode_n as f64);

        // RLC values for a parallel resonator at f_mode with Q = q_mode
        let r_bp = r_es * level;
        let c_bp = 1.0 / (2.0 * PI * f_mode * r_bp * q_mode.max(0.1));
        let l_bp = r_bp * q_mode.max(0.1) / (2.0 * PI * f_mode);

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

    // ── Panel vibration mode RLC branches ─────────────────────────────
    // Cabinet panels vibrate at resonant frequencies determined by material,
    // thickness, dimensions, and covering. These add coloration (the "woody"
    // character of a cab) as parallel RLC branches on the motional junction.
    let mut panel_mode_ids: Vec<(String, String, String)> = Vec::new();
    if let Some(ref construction) = cab.enclosure.construction {
        // Need width and height to compute panel modes
        if let (Some(w), Some(h)) = (construction.width, construction.height) {
            let modes = calculate_panel_modes(
                construction.material,
                construction.thickness,
                w,
                h,
                construction.covering,
                4, // up to 4 panel modes
            );
            for pm in &modes {
                // Scale panel mode level: lower modes couple more strongly
                let pm_level = 10f64.powf(pm.level_db / 20.0) * 0.3; // panels are weaker than cone breakup
                let r_pm = r_es * pm_level.max(1e-6);
                let q_pm = pm.q;
                let f_pm = pm.frequency;

                let c_pm = 1.0 / (2.0 * PI * f_pm * r_pm * q_pm.max(0.1));
                let l_pm = r_pm * q_pm.max(0.1) / (2.0 * PI * f_pm);

                let rpm_id = next_id("Rpm");
                let cpm_id = next_id("Cpm");
                let lpm_id = next_id("Lpm");

                components.push(crate::dsl::ComponentDef {
                    id: rpm_id.clone(),
                    kind: Box::new(crate::compiler::components::Resistor { value: r_pm }),
                });
                components.push(crate::dsl::ComponentDef {
                    id: cpm_id.clone(),
                    kind: Box::new(crate::compiler::components::Capacitor {
                        config: CapConfig {
                            value: c_pm,
                            cap_type: CapType::Film,
                            leakage: None,
                            da: None,
                        },
                    }),
                });
                components.push(crate::dsl::ComponentDef {
                    id: lpm_id.clone(),
                    kind: Box::new(crate::compiler::components::Inductor { value: l_pm }),
                });

                panel_mode_ids.push((rpm_id, cpm_id, lpm_id));
            }
        }
    }

    // ── Internal damping material ─────────────────────────────────────
    // Damping material (polyester fill, fiberglass, etc.) absorbs acoustic
    // energy inside the enclosure, reducing the Q of the box resonance.
    // Modeled as a resistor in parallel with the box compliance (L_box).
    // Higher absorption → lower resistance → more damping.
    let damping_r = if let Some(ref damping_def) = cab.enclosure.damping {
        let alpha = damping_def.material.absorption_coefficient();
        if alpha > 0.0 && damping_def.coverage > 0.0 {
            // Effective absorption scales with coverage, material coefficient,
            // and thickness (thicker fill absorbs more, especially at lower frequencies).
            // Reference thickness: 25mm. Thicker = more absorption (diminishing returns).
            let thickness_factor = (damping_def.thickness / 0.025).sqrt().min(2.0);
            let effective_alpha = alpha * damping_def.coverage * thickness_factor;
            Some(r_es / effective_alpha.max(0.01))
        } else {
            None
        }
    } else {
        None
    };

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

    // Add panel mode branches to junction
    for (rpm_id, cpm_id, lpm_id) in &panel_mode_ids {
        junction_pins.push(Pin::ComponentPin {
            component: rpm_id.clone(),
            pin: "a".to_string(),
        });
        junction_pins.push(Pin::ComponentPin {
            component: cpm_id.clone(),
            pin: "a".to_string(),
        });
        junction_pins.push(Pin::ComponentPin {
            component: lpm_id.clone(),
            pin: "a".to_string(),
        });
    }

    // Add damping material resistor to junction (parallel with box compliance)
    if let Some(r_damp) = damping_r {
        let rdamp_id = next_id("Rdamp");
        components.push(crate::dsl::ComponentDef {
            id: rdamp_id.clone(),
            kind: Box::new(crate::compiler::components::Resistor { value: r_damp }),
        });
        junction_pins.push(Pin::ComponentPin {
            component: rdamp_id.clone(),
            pin: "a".to_string(),
        });
        nets.push(NetDef {
            from: Pin::ComponentPin {
                component: rdamp_id,
                pin: "b".to_string(),
            },
            to: vec![Pin::Reserved("gnd".to_string())],
        });
    }

    // Add sealed enclosure leakage resistor to junction
    if let Some(r_leak) = sealed_leak {
        let rleak_id = next_id("Rleak");
        components.push(crate::dsl::ComponentDef {
            id: rleak_id.clone(),
            kind: Box::new(crate::compiler::components::Resistor { value: r_leak }),
        });
        junction_pins.push(Pin::ComponentPin {
            component: rleak_id.clone(),
            pin: "a".to_string(),
        });
        nets.push(NetDef {
            from: Pin::ComponentPin {
                component: rleak_id,
                pin: "b".to_string(),
            },
            to: vec![Pin::Reserved("gnd".to_string())],
        });
    }

    // Add bass reflex port (series R + C from junction to gnd)
    // The port capacitor (acoustic mass) resonates with the box compliance (inductor)
    // creating the Helmholtz resonance that extends bass response.
    if let Some((r_port, c_port)) = port_rl {
        let rport_id = next_id("Rport");
        let cport_id = next_id("Cport");
        components.push(crate::dsl::ComponentDef {
            id: rport_id.clone(),
            kind: Box::new(crate::compiler::components::Resistor { value: r_port }),
        });
        components.push(crate::dsl::ComponentDef {
            id: cport_id.clone(),
            kind: Box::new(crate::compiler::components::Capacitor {
                config: CapConfig {
                    value: c_port,
                    cap_type: CapType::Film,
                    leakage: None,
                    da: None,
                },
            }),
        });
        // Port R.a at junction, R.b -> C.a, C.b -> gnd
        junction_pins.push(Pin::ComponentPin {
            component: rport_id.clone(),
            pin: "a".to_string(),
        });
        nets.push(NetDef {
            from: Pin::ComponentPin {
                component: rport_id,
                pin: "b".to_string(),
            },
            to: vec![Pin::ComponentPin {
                component: cport_id.clone(),
                pin: "a".to_string(),
            }],
        });
        nets.push(NetDef {
            from: Pin::ComponentPin {
                component: cport_id,
                pin: "b".to_string(),
            },
            to: vec![Pin::Reserved("gnd".to_string())],
        });
    }

    // For open_back, L_ces is already connected above; for sealed, add it here
    if cab.enclosure.enclosure_type != EnclosureType::OpenBack {
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

    // Panel mode branch .b -> gnd
    for (rpm_id, cpm_id, lpm_id) in &panel_mode_ids {
        nets.push(NetDef {
            from: Pin::ComponentPin {
                component: rpm_id.clone(),
                pin: "b".to_string(),
            },
            to: vec![Pin::Reserved("gnd".to_string())],
        });
        nets.push(NetDef {
            from: Pin::ComponentPin {
                component: cpm_id.clone(),
                pin: "b".to_string(),
            },
            to: vec![Pin::Reserved("gnd".to_string())],
        });
        nets.push(NetDef {
            from: Pin::ComponentPin {
                component: lpm_id.clone(),
                pin: "b".to_string(),
            },
            to: vec![Pin::Reserved("gnd".to_string())],
        });
    }

    // ── Grille filtering ────────────────────────────────────────────────
    // Grille cloth/metal adds series R + L between R_rad and output,
    // creating a gentle low-pass that attenuates high frequencies.
    // Inserted into the signal path before the mic circuit.
    let mut post_rad_pin = Pin::ComponentPin {
        component: rrad_id.clone(),
        pin: "b".to_string(),
    };

    if grille_r > 0.0 {
        let rgrille_id = next_id("Rgrille");
        components.push(crate::dsl::ComponentDef {
            id: rgrille_id.clone(),
            kind: Box::new(crate::compiler::components::Resistor { value: grille_r }),
        });
        nets.push(NetDef {
            from: post_rad_pin,
            to: vec![Pin::ComponentPin {
                component: rgrille_id.clone(),
                pin: "a".to_string(),
            }],
        });
        post_rad_pin = Pin::ComponentPin {
            component: rgrille_id,
            pin: "b".to_string(),
        };
    }

    if grille_l > 0.0 {
        let lgrille_id = next_id("Lgrille");
        components.push(crate::dsl::ComponentDef {
            id: lgrille_id.clone(),
            kind: Box::new(crate::compiler::components::Inductor { value: grille_l }),
        });
        nets.push(NetDef {
            from: post_rad_pin,
            to: vec![Pin::ComponentPin {
                component: lgrille_id.clone(),
                pin: "a".to_string(),
            }],
        });
        post_rad_pin = Pin::ComponentPin {
            component: lgrille_id,
            pin: "b".to_string(),
        };
    }

    // ── Microphone circuit ──────────────────────────────────────────────
    // Determine if we have a mic with an equivalent circuit to model.
    let mic_params = if !cab.mics.is_empty() && mic_index < cab.mics.len() {
        cab.mics[mic_index].model.circuit_params()
    } else {
        None
    };

    if let Some(ref mp) = mic_params {
        // Build mic transducer equivalent circuit:
        //   post_rad → [coupling_cap →] [R_fet →] R_vc → L_vc → Transformer → out
        //
        // For condensers: coupling cap + FET output R replace voice coil R/L.
        // For dynamic/ribbon: voice coil R + L feed the transformer primary.

        let mut prev_pin = post_rad_pin;

        // Condenser: coupling capacitor (capsule equivalent)
        if let Some(cc) = mp.coupling_cap {
            let cc_id = next_id("Ccap");
            components.push(crate::dsl::ComponentDef {
                id: cc_id.clone(),
                kind: Box::new(crate::compiler::components::Capacitor {
                    config: CapConfig {
                        value: cc,
                        cap_type: CapType::Film,
                        leakage: None,
                        da: None,
                    },
                }),
            });
            nets.push(NetDef {
                from: prev_pin,
                to: vec![Pin::ComponentPin {
                    component: cc_id.clone(),
                    pin: "a".to_string(),
                }],
            });
            prev_pin = Pin::ComponentPin {
                component: cc_id,
                pin: "b".to_string(),
            };
        }

        // Condenser: FET output impedance
        if let Some(r_fet) = mp.fet_output_r {
            let rfet_id = next_id("Rfet");
            components.push(crate::dsl::ComponentDef {
                id: rfet_id.clone(),
                kind: Box::new(crate::compiler::components::Resistor { value: r_fet }),
            });
            nets.push(NetDef {
                from: prev_pin,
                to: vec![Pin::ComponentPin {
                    component: rfet_id.clone(),
                    pin: "a".to_string(),
                }],
            });
            prev_pin = Pin::ComponentPin {
                component: rfet_id,
                pin: "b".to_string(),
            };
        }

        // Dynamic/ribbon: voice coil resistance (skip if zero for condensers)
        if mp.voice_coil_re > 0.0 {
            let rvc_id = next_id("Rvc");
            components.push(crate::dsl::ComponentDef {
                id: rvc_id.clone(),
                kind: Box::new(crate::compiler::components::Resistor {
                    value: mp.voice_coil_re,
                }),
            });
            nets.push(NetDef {
                from: prev_pin,
                to: vec![Pin::ComponentPin {
                    component: rvc_id.clone(),
                    pin: "a".to_string(),
                }],
            });
            prev_pin = Pin::ComponentPin {
                component: rvc_id,
                pin: "b".to_string(),
            };
        }

        // Dynamic/ribbon: voice coil inductance (skip if zero for condensers)
        if mp.voice_coil_le > 0.0 {
            let lvc_id = next_id("Lvc");
            components.push(crate::dsl::ComponentDef {
                id: lvc_id.clone(),
                kind: Box::new(crate::compiler::components::Inductor {
                    value: mp.voice_coil_le,
                }),
            });
            nets.push(NetDef {
                from: prev_pin,
                to: vec![Pin::ComponentPin {
                    component: lvc_id.clone(),
                    pin: "a".to_string(),
                }],
            });
            prev_pin = Pin::ComponentPin {
                component: lvc_id,
                pin: "b".to_string(),
            };
        }

        // Step-up transformer
        let xfmr_id = next_id("Tmic");
        let xfmr_config = TransformerConfig::new(1.0 / mp.transformer_ratio, mp.transformer_l_pri)
            .with_dcr(mp.transformer_dcr_pri, mp.transformer_dcr_sec)
            .with_capacitance(mp.transformer_cap);
        components.push(crate::dsl::ComponentDef {
            id: xfmr_id.clone(),
            kind: Box::new(TransformerComp {
                config: xfmr_config,
            }),
        });

        // prev_pin → transformer primary.a
        nets.push(NetDef {
            from: prev_pin,
            to: vec![Pin::ComponentPin {
                component: xfmr_id.clone(),
                pin: "pri.a".to_string(),
            }],
        });
        // transformer primary.b → gnd
        nets.push(NetDef {
            from: Pin::ComponentPin {
                component: xfmr_id.clone(),
                pin: "pri.b".to_string(),
            },
            to: vec![Pin::Reserved("gnd".to_string())],
        });
        // transformer secondary.a → out
        nets.push(NetDef {
            from: Pin::ComponentPin {
                component: xfmr_id.clone(),
                pin: "sec.a".to_string(),
            },
            to: vec![Pin::Reserved("out".to_string())],
        });
        // transformer secondary.b → gnd
        nets.push(NetDef {
            from: Pin::ComponentPin {
                component: xfmr_id,
                pin: "sec.b".to_string(),
            },
            to: vec![Pin::Reserved("gnd".to_string())],
        });
    } else {
        // No mic circuit — direct pickup: post_rad → out
        nets.push(NetDef {
            from: post_rad_pin,
            to: vec![Pin::Reserved("out".to_string())],
        });
    }

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

    #[test]
    fn compile_cab_with_sm57_mic() {
        let src = r#"
            cab "SM57 Test" {
                driver { preset: v30 }
                enclosure { type: sealed, volume: 45L }
                mic close {
                    model: sm57
                    position: [0cm, 0cm, 2.5cm]
                }
            }
        "#;
        let cab = parse_cab_file(src).unwrap();
        let result = compile_cab(&cab, 48000.0);
        assert!(
            result.is_ok(),
            "Cab with SM57 should compile: {}",
            result.err().unwrap_or_default()
        );
    }

    #[test]
    fn compile_cab_with_ribbon_mic() {
        let src = r#"
            cab "Ribbon Test" {
                driver { preset: g12m_greenback }
                enclosure { type: sealed, volume: 45L }
                mic ribbon {
                    model: r121
                    position: [0cm, 0cm, 5cm]
                }
            }
        "#;
        let cab = parse_cab_file(src).unwrap();
        let result = compile_cab(&cab, 48000.0);
        assert!(
            result.is_ok(),
            "Cab with R121 should compile: {}",
            result.err().unwrap_or_default()
        );
    }

    #[test]
    fn compile_cab_with_condenser_mic() {
        let src = r#"
            cab "Condenser Test" {
                driver { preset: v30 }
                enclosure { type: sealed, volume: 45L }
                mic room {
                    model: u87
                    position: [0cm, 0cm, 30cm]
                }
            }
        "#;
        let cab = parse_cab_file(src).unwrap();
        let result = compile_cab(&cab, 48000.0);
        assert!(
            result.is_ok(),
            "Cab with U87 should compile: {}",
            result.err().unwrap_or_default()
        );
    }

    #[test]
    fn compile_cab_with_flat_mic() {
        let src = r#"
            cab "Flat Test" {
                driver { preset: v30 }
                enclosure { type: sealed, volume: 45L }
                mic direct {
                    model: flat
                    position: [0cm, 0cm, 2.5cm]
                }
            }
        "#;
        let cab = parse_cab_file(src).unwrap();
        let result = compile_cab(&cab, 48000.0);
        assert!(
            result.is_ok(),
            "Cab with Flat mic should compile: {}",
            result.err().unwrap_or_default()
        );
    }

    #[test]
    fn mic_cab_produces_output() {
        use crate::PedalProcessor;

        let src = r#"
            cab "SM57 Output Test" {
                driver { preset: v30 }
                enclosure { type: sealed, volume: 45L }
                mic close {
                    model: sm57
                    position: [0cm, 0cm, 2.5cm]
                }
            }
        "#;
        let cab = parse_cab_file(src).unwrap();
        let mut proc = compile_cab(&cab, 48000.0).unwrap();

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
            "Cab with mic should produce non-zero output, got max={max_out}"
        );
    }

    #[test]
    fn pedal_def_with_mic_has_transformer() {
        let src = r#"
            cab "Transformer Check" {
                driver { preset: v30 }
                enclosure { type: sealed, volume: 45L }
                mic close {
                    model: sm57
                    position: [0cm, 0cm, 2.5cm]
                }
            }
        "#;
        let cab = parse_cab_file(src).unwrap();
        let mut cab = cab;
        validate_cab(&mut cab).unwrap();
        let pedal = generate_pedal_def(&cab, 0).unwrap();

        // Should have transformer component
        assert!(
            pedal.components.iter().any(|c| c.id.starts_with("Tmic")),
            "Pedal def with mic should contain a transformer"
        );
        // Should have voice coil resistor
        assert!(
            pedal.components.iter().any(|c| c.id.starts_with("Rvc")),
            "Pedal def with dynamic mic should have voice coil R"
        );
    }

    #[test]
    fn pedal_def_without_mic_has_no_transformer() {
        let src = r#"
            cab "No Mic" {
                driver { preset: v30 }
                enclosure { type: sealed, volume: 45L }
            }
        "#;
        let cab = parse_cab_file(src).unwrap();
        let mut cab = cab;
        validate_cab(&mut cab).unwrap();
        let pedal = generate_pedal_def(&cab, 0).unwrap();

        assert!(
            !pedal.components.iter().any(|c| c.id.starts_with("Tmic")),
            "Pedal def without mic should not contain a transformer"
        );
    }
}
