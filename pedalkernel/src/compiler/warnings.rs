//! Voltage compatibility checking for pedal circuits.

use crate::dsl::*;

// ═══════════════════════════════════════════════════════════════════════════
// Voltage compatibility check
// ═══════════════════════════════════════════════════════════════════════════

/// Severity of a voltage compatibility warning.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WarningSeverity {
    /// Informational — the component should be fine, but behavior may differ
    /// from the canonical 9V version (e.g. slightly different bias point).
    Info,
    /// The component is likely operating outside its typical ratings and may
    /// clip, distort differently, or wear out faster in a real build.
    Caution,
    /// The component would almost certainly fail or be damaged at this voltage
    /// in a physical build.
    Danger,
}

/// A single voltage compatibility warning for a component.
#[derive(Debug, Clone)]
pub struct VoltageWarning {
    pub component_id: String,
    pub severity: WarningSeverity,
    pub message: String,
}

/// Check a pedal definition for voltage compatibility at a given supply voltage.
///
/// Returns a list of warnings for components that may not tolerate the target
/// voltage.  These are heuristic — the DSL doesn't carry voltage ratings, so
/// we infer from component types and typical part specs:
///
/// - **Germanium transistors**: Vce(max) typically 15–32V.  Caution above 15V.
/// - **Electrolytic capacitors** (≥ 1 µF): often rated 10–16V.  Caution above 12V.
/// - **Op-amps**: common audio opamps (TL072, NE5532) rated to ±18V (36V total).
///   Caution above 18V.
/// - **Silicon transistors**: usually Vce(max) ≥ 40V.  Caution above 24V.
/// - **Germanium diodes**: forward behaviour is voltage-independent, but reverse
///   breakdown is lower (~50V Ge vs ~100V+ Si).  Only flagged above 24V.
///
/// Note: passive elements (R, L) and silicon diodes are unaffected in the WDF
/// model and are not flagged.
pub fn check_voltage_compatibility(pedal: &PedalDef, voltage: f64) -> Vec<VoltageWarning> {
    let mut warnings = Vec::new();

    for comp in &pedal.components {
        match &comp.kind {
            ComponentKind::Npn(bt) | ComponentKind::Pnp(bt) => {
                let is_ge = bt.is_germanium();
                if is_ge {
                    // Germanium PNPs: typical Vce(max) = 15–32V
                    if voltage > 18.0 {
                        warnings.push(VoltageWarning {
                            component_id: comp.id.clone(),
                            severity: WarningSeverity::Danger,
                            message: format!(
                                "Germanium transistor {} likely exceeds Vce(max) at {:.0}V \
                                 (typical Ge PNP rated 15–32V)",
                                comp.id, voltage
                            ),
                        });
                    } else if voltage > 12.0 {
                        warnings.push(VoltageWarning {
                            component_id: comp.id.clone(),
                            severity: WarningSeverity::Caution,
                            message: format!(
                                "Germanium transistor {} may run hot at {:.0}V \
                                 — bias point shifts, tone will differ from 9V",
                                comp.id, voltage
                            ),
                        });
                    }
                } else if voltage > 24.0 {
                    // Silicon transistors: typically Vce(max) >= 40V
                    warnings.push(VoltageWarning {
                        component_id: comp.id.clone(),
                        severity: WarningSeverity::Caution,
                        message: format!(
                            "Transistor {} at {:.0}V — verify Vce(max) rating of actual part",
                            comp.id, voltage
                        ),
                    });
                }
            }
            ComponentKind::OpAmp(ot) => {
                // Use the op-amp type's known supply_max for accurate warnings
                let max_supply = ot.supply_max();
                if voltage > max_supply * 0.5 {
                    // Operating above half the max total supply (typical single-supply limit)
                    let severity = if voltage > max_supply {
                        WarningSeverity::Danger
                    } else {
                        WarningSeverity::Caution
                    };
                    warnings.push(VoltageWarning {
                        component_id: comp.id.clone(),
                        severity,
                        message: format!(
                            "Op-amp {} ({:?}) at {:.0}V — max total supply {:.0}V (±{:.0}V split)",
                            comp.id, ot, voltage, max_supply, max_supply / 2.0
                        ),
                    });
                }
            }
            ComponentKind::Capacitor(farads) => {
                // Electrolytics (≥ 1µF) often have low voltage ratings.
                // 10µF caps commonly rated 10V or 16V.
                if *farads >= 1e-6 {
                    if voltage > 16.0 {
                        warnings.push(VoltageWarning {
                            component_id: comp.id.clone(),
                            severity: WarningSeverity::Danger,
                            message: format!(
                                "Electrolytic cap {} ({:.0}µF) may exceed voltage rating at {:.0}V \
                                 — common ratings are 10V, 16V, 25V",
                                comp.id, farads * 1e6, voltage
                            ),
                        });
                    } else if voltage > 12.0 {
                        warnings.push(VoltageWarning {
                            component_id: comp.id.clone(),
                            severity: WarningSeverity::Caution,
                            message: format!(
                                "Electrolytic cap {} ({:.0}µF) — ensure voltage rating ≥ {:.0}V",
                                comp.id,
                                farads * 1e6,
                                voltage
                            ),
                        });
                    }
                }
            }
            ComponentKind::DiodePair(DiodeType::Germanium)
            | ComponentKind::Diode(DiodeType::Germanium) => {
                // Ge diode forward behaviour is voltage-independent in the WDF,
                // but reverse breakdown is lower (~50V vs 100V+ Si).
                // Also: more temperature-sensitive at higher power dissipation.
                if voltage > 18.0 {
                    warnings.push(VoltageWarning {
                        component_id: comp.id.clone(),
                        severity: WarningSeverity::Info,
                        message: format!(
                            "Germanium diode {} — higher power dissipation at {:.0}V \
                             may shift forward voltage (temperature dependent)",
                            comp.id, voltage
                        ),
                    });
                }
            }
            ComponentKind::Bbd(_) => {
                // BBDs are typically rated 10-15V. They're sensitive to
                // over-voltage which causes excess clock noise and distortion.
                if voltage > 15.0 {
                    warnings.push(VoltageWarning {
                        component_id: comp.id.clone(),
                        severity: WarningSeverity::Danger,
                        message: format!(
                            "BBD {} may exceed max supply at {:.0}V — typical rating 10-15V",
                            comp.id, voltage
                        ),
                    });
                }
            }
            // Synth ICs: CEM/AS/V series typically run ±5V to ±9V (10-18V total)
            ComponentKind::Vco(_) | ComponentKind::Vcf(_) => {
                if voltage > 18.0 {
                    warnings.push(VoltageWarning {
                        component_id: comp.id.clone(),
                        severity: WarningSeverity::Danger,
                        message: format!(
                            "Synth IC {} exceeds max supply at {:.0}V — CEM/AS/V series rated ±9V (18V max)",
                            comp.id, voltage
                        ),
                    });
                }
            }
            ComponentKind::AnalogSwitch(_) => {
                if voltage > 20.0 {
                    warnings.push(VoltageWarning {
                        component_id: comp.id.clone(),
                        severity: WarningSeverity::Danger,
                        message: format!(
                            "Analog switch {} exceeds max supply at {:.0}V — CD4066/DG411 rated 20V max",
                            comp.id, voltage
                        ),
                    });
                }
            }
            // Resistors, inductors, Si/LED diodes, pots, VCA, comparator, matched
            // transistors, tempco: no voltage concerns within the 5–24V range
            // (or handled by supply_max spec if provided).
            _ => {}
        }
    }

    warnings
}

