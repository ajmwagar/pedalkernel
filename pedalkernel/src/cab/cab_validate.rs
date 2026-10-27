//! Validation and derived parameter calculation for `.cab` definitions.
//!
//! Resolves presets, derives Thiele-Small parameters from the 7 primitives,
//! checks physical plausibility, and computes panel vibration modes.

use std::f64::consts::PI;

use super::cab_def::*;
use super::presets::DriverTsParams;

// ═══════════════════════════════════════════════════════════════════════════
// Warning types
// ═══════════════════════════════════════════════════════════════════════════

#[derive(Debug, Clone)]
pub enum CabWarningSeverity {
    Info,
    Warning,
    Error,
}

#[derive(Debug, Clone)]
pub struct CabWarning {
    pub severity: CabWarningSeverity,
    pub message: String,
}

// ═══════════════════════════════════════════════════════════════════════════
// Main validation entry point
// ═══════════════════════════════════════════════════════════════════════════

/// Validate and complete a CabDef: resolve presets, derive T-S params,
/// check plausibility.
pub fn validate_cab(cab: &mut CabDef) -> Result<Vec<CabWarning>, String> {
    let mut warnings = Vec::new();

    if cab.drivers.is_empty() {
        return Err("Cabinet must have at least one driver".to_string());
    }

    // Resolve presets and derive parameters for each driver
    for (i, driver) in cab.drivers.iter_mut().enumerate() {
        resolve_preset(driver);

        // Check that we have all 7 primitive T-S params
        let missing = check_required_ts(driver);
        if !missing.is_empty() {
            return Err(format!(
                "Driver {} missing required T-S parameters: {}",
                driver.id.as_deref().unwrap_or(&format!("#{}", i)),
                missing.join(", ")
            ));
        }

        derive_ts_parameters(driver, &cab.environment);
        warnings.extend(plausibility_check(driver, i));
    }

    // Derive volume from construction dimensions if not explicitly set
    derive_volume(&mut cab.enclosure);

    // Enclosure validation
    warnings.extend(validate_enclosure(&cab.enclosure));

    // Mic validation
    warnings.extend(validate_mics(&cab.mics));

    Ok(warnings)
}

// ═══════════════════════════════════════════════════════════════════════════
// Preset resolution
// ═══════════════════════════════════════════════════════════════════════════

/// Fill unspecified T-S parameters from the preset database.
fn resolve_preset(driver: &mut DriverDef) {
    if let Some(preset) = driver.preset {
        let p = preset.params();
        if driver.re.is_none() {
            driver.re = Some(p.re);
        }
        if driver.le.is_none() {
            driver.le = Some(p.le);
        }
        if driver.bl.is_none() {
            driver.bl = Some(p.bl);
        }
        if driver.mms.is_none() {
            driver.mms = Some(p.mms);
        }
        if driver.cms.is_none() {
            driver.cms = Some(p.cms);
        }
        if driver.rms.is_none() {
            driver.rms = Some(p.rms);
        }
        if driver.sd.is_none() {
            driver.sd = Some(p.sd);
        }
    }
}

/// Check which of the 7 required primitive T-S parameters are missing.
fn check_required_ts(driver: &DriverDef) -> Vec<&'static str> {
    let mut missing = Vec::new();
    if driver.re.is_none() {
        missing.push("Re");
    }
    if driver.le.is_none() {
        missing.push("Le");
    }
    if driver.bl.is_none() {
        missing.push("Bl");
    }
    if driver.mms.is_none() {
        missing.push("Mms");
    }
    if driver.cms.is_none() {
        missing.push("Cms");
    }
    if driver.rms.is_none() {
        missing.push("Rms");
    }
    if driver.sd.is_none() {
        missing.push("Sd");
    }
    missing
}

// ═══════════════════════════════════════════════════════════════════════════
// Derived T-S parameter calculation
// ═══════════════════════════════════════════════════════════════════════════

/// Compute derived T-S parameters from the 7 primitives.
pub fn derive_ts_parameters(driver: &mut DriverDef, env: &EnvironmentDef) {
    let re = driver.re.unwrap();
    let bl = driver.bl.unwrap();
    let mms = driver.mms.unwrap();
    let cms = driver.cms.unwrap();
    let rms = driver.rms.unwrap();
    let sd = driver.sd.unwrap();
    let rho = env.air_density();
    let c = env.speed_of_sound();

    // Resonant frequency
    if driver.fs.is_none() {
        driver.fs = Some(1.0 / (2.0 * PI * (mms * cms).sqrt()));
    }
    let fs = driver.fs.unwrap();

    // Mechanical Q
    if driver.qms.is_none() {
        driver.qms = Some(2.0 * PI * fs * mms / rms);
    }

    // Electrical Q
    if driver.qes.is_none() {
        driver.qes = Some(2.0 * PI * fs * mms * re / (bl * bl));
    }

    // Total Q
    if driver.qts.is_none() {
        let qms = driver.qms.unwrap();
        let qes = driver.qes.unwrap();
        driver.qts = Some(qms * qes / (qms + qes));
    }

    // Equivalent compliance volume
    if driver.vas.is_none() {
        driver.vas = Some(rho * c * c * sd * sd * cms);
    }

    // Reference efficiency (Small, 1972)
    // η₀ = (4π²·Fs³·Vas) / c³ = (4π²·Fs³·ρ·Sd²·Cms) / c
    if driver.eta0.is_none() {
        let eta0 = (4.0 * PI * PI * fs * fs * fs * rho * sd * sd * cms) / c;
        driver.eta0 = Some(eta0);
    }

    // Sensitivity (dB SPL, 1W/1m)
    if driver.spl.is_none() {
        let eta0 = driver.eta0.unwrap();
        if eta0 > 0.0 {
            driver.spl = Some(112.1 + 10.0 * eta0.log10());
        }
    }
}

/// Extract resolved T-S params from a validated driver. Panics if not validated.
pub fn resolved_ts(driver: &DriverDef) -> DriverTsParams {
    DriverTsParams {
        re: driver.re.unwrap(),
        le: driver.le.unwrap(),
        bl: driver.bl.unwrap(),
        mms: driver.mms.unwrap(),
        cms: driver.cms.unwrap(),
        rms: driver.rms.unwrap(),
        sd: driver.sd.unwrap(),
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Plausibility checks
// ═══════════════════════════════════════════════════════════════════════════

fn plausibility_check(driver: &DriverDef, index: usize) -> Vec<CabWarning> {
    let mut warnings = Vec::new();
    let label = driver
        .id
        .as_deref()
        .map(|s| s.to_string())
        .unwrap_or_else(|| format!("driver #{index}"));

    if let Some(qts) = driver.qts {
        if qts < 0.15 || qts > 2.0 {
            warnings.push(CabWarning {
                severity: CabWarningSeverity::Warning,
                message: format!(
                    "{label}: Qts = {qts:.2} is outside typical range (0.2–1.5)"
                ),
            });
        }
    }

    if let Some(re) = driver.re {
        if re < 1.0 || re > 20.0 {
            warnings.push(CabWarning {
                severity: CabWarningSeverity::Warning,
                message: format!(
                    "{label}: Re = {re:.1} Ω is outside typical range (2–16 Ω)"
                ),
            });
        }
    }

    if let Some(fs) = driver.fs {
        if fs < 20.0 || fs > 300.0 {
            warnings.push(CabWarning {
                severity: CabWarningSeverity::Warning,
                message: format!(
                    "{label}: Fs = {fs:.0} Hz is outside typical range (30–200 Hz)"
                ),
            });
        }
    }

    if let Some(sd) = driver.sd {
        let diam_inches = 2.0 * (sd / PI).sqrt() / 0.0254;
        if diam_inches < 3.0 || diam_inches > 20.0 {
            warnings.push(CabWarning {
                severity: CabWarningSeverity::Warning,
                message: format!(
                    "{label}: Sd implies {diam_inches:.0}\" diameter — outside typical range (4\"–18\")"
                ),
            });
        }
    }

    warnings
}

/// Derive internal volume from construction dimensions when not explicitly specified.
///
/// If `volume` is already set, it is left unchanged (user override wins).
/// Otherwise, if `construction` provides width, height, and depth, the internal
/// volume is computed by subtracting 2× panel thickness per axis.
fn derive_volume(enc: &mut EnclosureDef) {
    if enc.volume.is_some() {
        return; // User explicitly specified volume, respect it
    }
    if let Some(ref construction) = enc.construction {
        if let (Some(w), Some(h), Some(d)) =
            (construction.width, construction.height, construction.depth)
        {
            let t = construction.thickness;
            // Internal dimensions = external minus 2× panel thickness per axis
            let w_int = (w - 2.0 * t).max(0.01);
            let h_int = (h - 2.0 * t).max(0.01);
            let d_int = (d - 2.0 * t).max(0.01);
            enc.volume = Some(w_int * h_int * d_int);
        }
    }
}

fn validate_enclosure(enc: &EnclosureDef) -> Vec<CabWarning> {
    let mut warnings = Vec::new();

    match enc.enclosure_type {
        EnclosureType::Sealed | EnclosureType::OpenBack | EnclosureType::BassReflex => {
            if enc.volume.is_none() {
                warnings.push(CabWarning {
                    severity: CabWarningSeverity::Error,
                    message: "Enclosure volume is required for sealed/open_back/bass_reflex types"
                        .to_string(),
                });
            }
        }
        EnclosureType::InfiniteBaffle => {}
        _ => {
            if enc.volume.is_none() {
                warnings.push(CabWarning {
                    severity: CabWarningSeverity::Info,
                    message: "Enclosure volume not specified — will be estimated".to_string(),
                });
            }
        }
    }

    if enc.enclosure_type == EnclosureType::OpenBack && enc.open_area.is_none() {
        warnings.push(CabWarning {
            severity: CabWarningSeverity::Info,
            message: "Open-back cabinet without open_area specified — defaulting to 75%"
                .to_string(),
        });
    }

    if enc.enclosure_type == EnclosureType::BassReflex && enc.port.is_none() {
        warnings.push(CabWarning {
            severity: CabWarningSeverity::Error,
            message: "Bass reflex enclosure requires a port { } block".to_string(),
        });
    }

    warnings
}

fn validate_mics(mics: &[MicDef]) -> Vec<CabWarning> {
    let mut warnings = Vec::new();
    for mic in mics {
        let dist = (mic.position[0].powi(2) + mic.position[1].powi(2) + mic.position[2].powi(2))
            .sqrt();
        if dist > 5.0 {
            warnings.push(CabWarning {
                severity: CabWarningSeverity::Info,
                message: format!(
                    "Mic '{}' is {dist:.1}m from driver — very far, may need longer IR",
                    mic.id
                ),
            });
        }
    }
    warnings
}

// ═══════════════════════════════════════════════════════════════════════════
// Panel mode calculation
// ═══════════════════════════════════════════════════════════════════════════

/// Calculate panel vibration modes for a rectangular panel.
///
/// Uses thin plate theory: f_mn = (π/2) × √(D / (ρ_s)) × ((m/Lx)² + (n/Ly)²)
/// where D = flexural rigidity, ρ_s = surface mass density, Lx/Ly = dimensions.
pub fn calculate_panel_modes(
    material: PanelMaterial,
    thickness: f64,
    width: f64,
    height: f64,
    covering: CoveringType,
    max_modes: usize,
) -> Vec<PanelMode> {
    let d = material.flexural_rigidity(thickness);
    let rho_s = material.density() * thickness + covering.added_mass();
    let damping = material.damping_ratio() * covering.damping_multiplier();

    let mut modes = Vec::new();
    let f_11 = (PI / 2.0) * (d / rho_s).sqrt() * (1.0 / (width * width) + 1.0 / (height * height));

    for m in 1..=4 {
        for n in 1..=4 {
            let f_mn = (PI / 2.0)
                * (d / rho_s).sqrt()
                * ((m as f64 / width).powi(2) + (n as f64 / height).powi(2));

            let q = 1.0 / damping;
            // Higher modes are progressively weaker
            let level_db = -6.0 * ((m * n) as f64 - 1.0).max(0.0);

            modes.push(PanelMode {
                mode: (m, n),
                frequency: f_mn,
                q,
                level_db,
            });

            if modes.len() >= max_modes {
                break;
            }
        }
        if modes.len() >= max_modes {
            break;
        }
    }

    modes.sort_by(|a, b| a.frequency.partial_cmp(&b.frequency).unwrap());
    modes.truncate(max_modes);
    modes
}

// ═══════════════════════════════════════════════════════════════════════════
// Tests
// ═══════════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod tests {
    use super::*;

    fn make_test_cab() -> CabDef {
        let mut cab = CabDef {
            name: "Test".to_string(),
            drivers: vec![DriverDef {
                preset: Some(DriverPreset::V30),
                ..DriverDef::default()
            }],
            enclosure: EnclosureDef {
                enclosure_type: EnclosureType::Sealed,
                volume: Some(0.045),
                ..EnclosureDef::default()
            },
            crossover: None,
            mics: Vec::new(),
            environment: EnvironmentDef::default(),
            controls: Vec::new(),
            monitors: Vec::new(),
        };
        cab
    }

    #[test]
    fn validate_resolves_preset() {
        let mut cab = make_test_cab();
        let _warnings = validate_cab(&mut cab).unwrap();
        let d = &cab.drivers[0];
        assert!(d.re.is_some());
        assert!(d.le.is_some());
        assert!(d.bl.is_some());
        assert!(d.mms.is_some());
        assert!(d.cms.is_some());
        assert!(d.rms.is_some());
        assert!(d.sd.is_some());
    }

    #[test]
    fn validate_derives_fs() {
        let mut cab = make_test_cab();
        validate_cab(&mut cab).unwrap();
        let fs = cab.drivers[0].fs.unwrap();
        assert!(
            (fs - 75.0).abs() < 15.0,
            "V30 Fs should be ~75 Hz, got {fs:.1}"
        );
    }

    #[test]
    fn validate_derives_qts() {
        let mut cab = make_test_cab();
        validate_cab(&mut cab).unwrap();
        let qts = cab.drivers[0].qts.unwrap();
        assert!(
            (qts - 0.79).abs() < 0.2,
            "V30 Qts should be ~0.79, got {qts:.2}"
        );
    }

    #[test]
    fn validate_derives_sensitivity() {
        let mut cab = make_test_cab();
        validate_cab(&mut cab).unwrap();
        let spl = cab.drivers[0].spl.unwrap();
        assert!(
            spl > 90.0 && spl < 105.0,
            "V30 SPL should be ~100 dB, got {spl:.1}"
        );
    }

    #[test]
    fn validate_missing_driver_fails() {
        let mut cab = CabDef {
            name: "Empty".to_string(),
            drivers: Vec::new(),
            enclosure: EnclosureDef::default(),
            crossover: None,
            mics: Vec::new(),
            environment: EnvironmentDef::default(),
            controls: Vec::new(),
            monitors: Vec::new(),
        };
        assert!(validate_cab(&mut cab).is_err());
    }

    #[test]
    fn validate_missing_ts_params_fails() {
        let mut cab = CabDef {
            name: "Incomplete".to_string(),
            drivers: vec![DriverDef {
                re: Some(5.6),
                // Missing other params, no preset
                ..DriverDef::default()
            }],
            enclosure: EnclosureDef {
                volume: Some(0.045),
                ..EnclosureDef::default()
            },
            crossover: None,
            mics: Vec::new(),
            environment: EnvironmentDef::default(),
            controls: Vec::new(),
            monitors: Vec::new(),
        };
        assert!(validate_cab(&mut cab).is_err());
    }

    #[test]
    fn volume_derived_from_construction() {
        let mut enc = EnclosureDef {
            enclosure_type: EnclosureType::Sealed,
            volume: None,
            construction: Some(ConstructionDef {
                width: Some(0.60),    // 60cm
                height: Some(0.70),   // 70cm
                depth: Some(0.30),    // 30cm
                thickness: 0.018,     // 18mm birch ply
                ..ConstructionDef::default()
            }),
            ..EnclosureDef::default()
        };
        derive_volume(&mut enc);
        let vol = enc.volume.expect("Volume should be derived");
        // Internal: (0.60-0.036) × (0.70-0.036) × (0.30-0.036) = 0.564 × 0.664 × 0.264 ≈ 0.0988 m³
        assert!(
            (vol - 0.0988).abs() < 0.005,
            "Derived volume should be ~0.099 m³, got {vol:.4}"
        );
    }

    #[test]
    fn explicit_volume_overrides_construction() {
        let mut enc = EnclosureDef {
            enclosure_type: EnclosureType::Sealed,
            volume: Some(0.045),
            construction: Some(ConstructionDef {
                width: Some(0.60),
                height: Some(0.70),
                depth: Some(0.30),
                thickness: 0.018,
                ..ConstructionDef::default()
            }),
            ..EnclosureDef::default()
        };
        derive_volume(&mut enc);
        assert_eq!(
            enc.volume,
            Some(0.045),
            "Explicit volume should not be overridden"
        );
    }

    #[test]
    fn volume_not_derived_without_all_dimensions() {
        let mut enc = EnclosureDef {
            volume: None,
            construction: Some(ConstructionDef {
                width: Some(0.60),
                height: None, // missing
                depth: Some(0.30),
                thickness: 0.018,
                ..ConstructionDef::default()
            }),
            ..EnclosureDef::default()
        };
        derive_volume(&mut enc);
        assert!(enc.volume.is_none(), "Volume should not be derived without all 3 dimensions");
    }

    #[test]
    fn panel_modes_birch_ply() {
        let modes = calculate_panel_modes(
            PanelMaterial::BirchPly,
            0.018,
            0.76,
            0.76,
            CoveringType::Tolex,
            4,
        );
        assert_eq!(modes.len(), 4);
        // First mode should be above 100 Hz for a 76cm panel
        assert!(
            modes[0].frequency > 50.0,
            "First panel mode should be > 50 Hz, got {:.0}",
            modes[0].frequency
        );
        // Modes should increase in frequency
        for w in modes.windows(2) {
            assert!(w[1].frequency >= w[0].frequency);
        }
    }
}
