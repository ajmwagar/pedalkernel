//! Speaker preset database and microphone response data.
//!
//! All Thiele-Small parameters sourced from manufacturer datasheets.
//! Mic response data based on published frequency response curves.

use super::cab_def::*;

// ═══════════════════════════════════════════════════════════════════════════
// Driver T-S parameter sets
// ═══════════════════════════════════════════════════════════════════════════

/// Resolved Thiele-Small parameters (all required fields present).
#[derive(Debug, Clone)]
pub struct DriverTsParams {
    pub re: f64,
    pub le: f64,
    pub bl: f64,
    pub mms: f64,
    pub cms: f64,
    pub rms: f64,
    pub sd: f64,
}

impl DriverPreset {
    /// Return the full Thiele-Small parameter set for this preset speaker.
    pub fn params(&self) -> DriverTsParams {
        match self {
            // ── Celestion ────────────────────────────────────────────────
            Self::V30 => DriverTsParams {
                re: 5.6,
                le: 0.62e-3,
                bl: 10.2,
                mms: 34.5e-3,
                cms: 1.305e-4,
                rms: 2.39,
                sd: 531e-4,
            },
            Self::G12mGreenback => DriverTsParams {
                re: 5.5,
                le: 0.55e-3,
                bl: 8.8,
                mms: 20.0e-3,
                cms: 2.252e-4,
                rms: 1.71,
                sd: 531e-4,
            },
            Self::G12hAnniversary => DriverTsParams {
                re: 5.7,
                le: 0.65e-3,
                bl: 11.5,
                mms: 30.0e-3,
                cms: 2.345e-4,
                rms: 1.62,
                sd: 531e-4,
            },
            Self::G12t75 => DriverTsParams {
                re: 5.6,
                le: 0.60e-3,
                bl: 10.0,
                mms: 28.0e-3,
                cms: 1.608e-4,
                rms: 2.20,
                sd: 531e-4,
            },
            Self::G12_65 => DriverTsParams {
                re: 5.5,
                le: 0.58e-3,
                bl: 10.8,
                mms: 32.0e-3,
                cms: 1.874e-4,
                rms: 2.01,
                sd: 531e-4,
            },
            Self::G12Blue => DriverTsParams {
                re: 12.5,
                le: 0.80e-3,
                bl: 8.5,
                mms: 12.5e-3,
                cms: 2.026e-4,
                rms: 1.75,
                sd: 531e-4,
            },
            Self::G12Creamback => DriverTsParams {
                re: 5.5,
                le: 0.58e-3,
                bl: 9.2,
                mms: 22.0e-3,
                cms: 2.047e-4,
                rms: 1.79,
                sd: 531e-4,
            },
            Self::G12Evh => DriverTsParams {
                re: 5.6,
                le: 0.62e-3,
                bl: 11.0,
                mms: 36.0e-3,
                cms: 1.436e-4,
                rms: 2.11,
                sd: 531e-4,
            },

            // ── Jensen ───────────────────────────────────────────────────
            Self::P12r => DriverTsParams {
                re: 6.8,
                le: 0.45e-3,
                bl: 7.2,
                mms: 14.0e-3,
                cms: 2.234e-4,
                rms: 1.98,
                sd: 531e-4,
            },
            Self::C12n => DriverTsParams {
                re: 6.5,
                le: 0.50e-3,
                bl: 8.0,
                mms: 18.0e-3,
                cms: 2.199e-4,
                rms: 1.81,
                sd: 531e-4,
            },
            Self::C12q => DriverTsParams {
                re: 6.8,
                le: 0.42e-3,
                bl: 6.8,
                mms: 12.0e-3,
                cms: 2.339e-4,
                rms: 1.59,
                sd: 531e-4,
            },
            Self::P10r => DriverTsParams {
                re: 6.5,
                le: 0.38e-3,
                bl: 5.5,
                mms: 8.0e-3,
                cms: 3.166e-4,
                rms: 1.44,
                sd: 346e-4,
            },

            // ── Eminence ─────────────────────────────────────────────────
            Self::SwampThang => DriverTsParams {
                re: 5.4,
                le: 0.70e-3,
                bl: 12.0,
                mms: 42.0e-3,
                cms: 1.994e-4,
                rms: 2.64,
                sd: 531e-4,
            },
            Self::ManOWar => DriverTsParams {
                re: 5.5,
                le: 0.65e-3,
                bl: 13.0,
                mms: 48.0e-3,
                cms: 2.111e-4,
                rms: 2.51,
                sd: 531e-4,
            },
            Self::Cv75 => DriverTsParams {
                re: 5.5,
                le: 0.55e-3,
                bl: 9.0,
                mms: 22.0e-3,
                cms: 2.725e-4,
                rms: 1.63,
                sd: 531e-4,
            },
            Self::Legend1258 => DriverTsParams {
                re: 5.6,
                le: 0.65e-3,
                bl: 12.5,
                mms: 40.0e-3,
                cms: 2.093e-4,
                rms: 2.51,
                sd: 531e-4,
            },
            Self::TexasHeat => DriverTsParams {
                re: 5.5,
                le: 0.60e-3,
                bl: 11.0,
                mms: 35.0e-3,
                cms: 2.393e-4,
                rms: 2.42,
                sd: 531e-4,
            },
            Self::CannabisRex => DriverTsParams {
                re: 5.6,
                le: 0.58e-3,
                bl: 9.5,
                mms: 28.0e-3,
                cms: 2.991e-4,
                rms: 1.94,
                sd: 531e-4,
            },

            // ── EV / JBL ─────────────────────────────────────────────────
            Self::Ev12l => DriverTsParams {
                re: 5.5,
                le: 0.80e-3,
                bl: 15.0,
                mms: 65.0e-3,
                cms: 2.436e-4,
                rms: 2.04,
                sd: 531e-4,
            },
            Self::JblD120 => DriverTsParams {
                re: 5.4,
                le: 0.75e-3,
                bl: 16.0,
                mms: 70.0e-3,
                cms: 2.262e-4,
                rms: 2.20,
                sd: 531e-4,
            },

            // ── WGS ─────────────────────────────────────────────────────
            Self::WbEt65 => DriverTsParams {
                re: 5.5,
                le: 0.58e-3,
                bl: 9.8,
                mms: 25.0e-3,
                cms: 1.801e-4,
                rms: 2.14,
                sd: 531e-4,
            },
            Self::WbRetro30 => DriverTsParams {
                re: 5.5,
                le: 0.60e-3,
                bl: 10.0,
                mms: 30.0e-3,
                cms: 1.501e-4,
                rms: 2.36,
                sd: 531e-4,
            },

            // ── Bass speakers ────────────────────────────────────────────
            Self::BassliteS2010 => DriverTsParams {
                re: 5.5,
                le: 0.90e-3,
                bl: 14.0,
                mms: 50.0e-3,
                cms: 1.675e-4,
                rms: 2.88,
                sd: 346e-4,
            },
            Self::Kappalite3015 => DriverTsParams {
                re: 5.2,
                le: 0.95e-3,
                bl: 18.0,
                mms: 85.0e-3,
                cms: 1.863e-4,
                rms: 2.67,
                sd: 855e-4,
            },
            Self::Ampeg10 => DriverTsParams {
                re: 5.5,
                le: 0.50e-3,
                bl: 8.5,
                mms: 16.0e-3,
                cms: 1.955e-4,
                rms: 2.01,
                sd: 346e-4,
            },
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Microphone equivalent circuit data
// ═══════════════════════════════════════════════════════════════════════════

/// Electrical equivalent circuit parameters for a microphone transducer.
///
/// Dynamic mics: voice coil (R_vc, L_vc) → step-up transformer.
/// Ribbon mics: ribbon element (R_rib, L_rib) → step-up transformer.
/// Condenser mics: capsule coupling cap → FET output impedance → transformer.
/// Flat/Measurement: no circuit (direct pickup).
#[derive(Debug, Clone)]
pub struct MicCircuitParams {
    /// Voice coil / ribbon DC resistance (Ω).
    pub voice_coil_re: f64,
    /// Voice coil / ribbon inductance (H).
    pub voice_coil_le: f64,
    /// Transformer step-up ratio (1:N, stored as N).
    pub transformer_ratio: f64,
    /// Transformer primary inductance (H).
    pub transformer_l_pri: f64,
    /// Transformer primary DCR (Ω).
    pub transformer_dcr_pri: f64,
    /// Transformer secondary DCR (Ω).
    pub transformer_dcr_sec: f64,
    /// Transformer parasitic capacitance (F).
    pub transformer_cap: f64,
    /// Condenser capsule coupling capacitance (F). None for dynamic/ribbon.
    pub coupling_cap: Option<f64>,
    /// FET output impedance for condensers (Ω). None for dynamic/ribbon.
    pub fet_output_r: Option<f64>,
}

impl MicModel {
    /// Return equivalent circuit parameters for this mic model.
    /// Returns `None` for Flat, Measurement, and Custom (direct pickup).
    pub fn circuit_params(&self) -> Option<MicCircuitParams> {
        match self {
            // ── Dynamic mics ────────────────────────────────────────────
            Self::Sm57 => Some(MicCircuitParams {
                voice_coil_re: 11.0,
                voice_coil_le: 200e-6,
                transformer_ratio: 28.0,
                transformer_l_pri: 0.5,
                transformer_dcr_pri: 2.0,
                transformer_dcr_sec: 150.0,
                transformer_cap: 200e-12,
                coupling_cap: None,
                fet_output_r: None,
            }),
            Self::Sm7b => Some(MicCircuitParams {
                voice_coil_re: 10.0,
                voice_coil_le: 250e-6,
                transformer_ratio: 30.0,
                transformer_l_pri: 0.8,
                transformer_dcr_pri: 2.5,
                transformer_dcr_sec: 180.0,
                transformer_cap: 150e-12,
                coupling_cap: None,
                fet_output_r: None,
            }),
            Self::Md421 => Some(MicCircuitParams {
                voice_coil_re: 8.0,
                voice_coil_le: 150e-6,
                transformer_ratio: 25.0,
                transformer_l_pri: 0.8,
                transformer_dcr_pri: 1.8,
                transformer_dcr_sec: 120.0,
                transformer_cap: 150e-12,
                coupling_cap: None,
                fet_output_r: None,
            }),
            Self::E906 => Some(MicCircuitParams {
                voice_coil_re: 12.0,
                voice_coil_le: 180e-6,
                transformer_ratio: 26.0,
                transformer_l_pri: 0.6,
                transformer_dcr_pri: 2.2,
                transformer_dcr_sec: 140.0,
                transformer_cap: 180e-12,
                coupling_cap: None,
                fet_output_r: None,
            }),
            Self::E609 => Some(MicCircuitParams {
                voice_coil_re: 12.0,
                voice_coil_le: 180e-6,
                transformer_ratio: 26.0,
                transformer_l_pri: 0.55,
                transformer_dcr_pri: 2.0,
                transformer_dcr_sec: 130.0,
                transformer_cap: 200e-12,
                coupling_cap: None,
                fet_output_r: None,
            }),
            Self::Re20 => Some(MicCircuitParams {
                voice_coil_re: 9.0,
                voice_coil_le: 220e-6,
                transformer_ratio: 30.0,
                transformer_l_pri: 1.0,
                transformer_dcr_pri: 2.8,
                transformer_dcr_sec: 200.0,
                transformer_cap: 120e-12,
                coupling_cap: None,
                fet_output_r: None,
            }),
            Self::M88 => Some(MicCircuitParams {
                voice_coil_re: 10.0,
                voice_coil_le: 200e-6,
                transformer_ratio: 28.0,
                transformer_l_pri: 0.7,
                transformer_dcr_pri: 2.0,
                transformer_dcr_sec: 160.0,
                transformer_cap: 160e-12,
                coupling_cap: None,
                fet_output_r: None,
            }),
            Self::M201 => Some(MicCircuitParams {
                voice_coil_re: 9.5,
                voice_coil_le: 170e-6,
                transformer_ratio: 27.0,
                transformer_l_pri: 0.65,
                transformer_dcr_pri: 2.0,
                transformer_dcr_sec: 140.0,
                transformer_cap: 170e-12,
                coupling_cap: None,
                fet_output_r: None,
            }),

            // ── Ribbon mics ─────────────────────────────────────────────
            Self::R121 => Some(MicCircuitParams {
                voice_coil_re: 0.5,
                voice_coil_le: 10e-6,
                transformer_ratio: 37.0,
                transformer_l_pri: 0.1,
                transformer_dcr_pri: 0.3,
                transformer_dcr_sec: 80.0,
                transformer_cap: 300e-12,
                coupling_cap: None,
                fet_output_r: None,
            }),
            Self::R101 => Some(MicCircuitParams {
                voice_coil_re: 0.6,
                voice_coil_le: 12e-6,
                transformer_ratio: 35.0,
                transformer_l_pri: 0.12,
                transformer_dcr_pri: 0.3,
                transformer_dcr_sec: 75.0,
                transformer_cap: 280e-12,
                coupling_cap: None,
                fet_output_r: None,
            }),
            Self::Fathead => Some(MicCircuitParams {
                voice_coil_re: 0.8,
                voice_coil_le: 15e-6,
                transformer_ratio: 35.0,
                transformer_l_pri: 0.08,
                transformer_dcr_pri: 0.4,
                transformer_dcr_sec: 70.0,
                transformer_cap: 350e-12,
                coupling_cap: None,
                fet_output_r: None,
            }),
            Self::M160 => Some(MicCircuitParams {
                voice_coil_re: 0.3,
                voice_coil_le: 5e-6,
                transformer_ratio: 30.0,
                transformer_l_pri: 0.15,
                transformer_dcr_pri: 0.2,
                transformer_dcr_sec: 60.0,
                transformer_cap: 250e-12,
                coupling_cap: None,
                fet_output_r: None,
            }),

            // ── Condenser mics ──────────────────────────────────────────
            Self::U87 => Some(MicCircuitParams {
                voice_coil_re: 0.0,     // no voice coil; coupling cap + FET instead
                voice_coil_le: 0.0,
                transformer_ratio: 5.0,
                transformer_l_pri: 2.0,
                transformer_dcr_pri: 8.0,
                transformer_dcr_sec: 40.0,
                transformer_cap: 100e-12,
                coupling_cap: Some(40e-12),
                fet_output_r: Some(200.0),
            }),
            Self::C414 => Some(MicCircuitParams {
                voice_coil_re: 0.0,
                voice_coil_le: 0.0,
                transformer_ratio: 8.0,
                transformer_l_pri: 1.5,
                transformer_dcr_pri: 6.0,
                transformer_dcr_sec: 50.0,
                transformer_cap: 80e-12,
                coupling_cap: Some(35e-12),
                fet_output_r: Some(180.0),
            }),
            Self::Km84 => Some(MicCircuitParams {
                voice_coil_re: 0.0,
                voice_coil_le: 0.0,
                transformer_ratio: 6.0,
                transformer_l_pri: 1.8,
                transformer_dcr_pri: 7.0,
                transformer_dcr_sec: 45.0,
                transformer_cap: 90e-12,
                coupling_cap: Some(30e-12),
                fet_output_r: Some(150.0),
            }),
            Self::C451 => Some(MicCircuitParams {
                voice_coil_re: 0.0,
                voice_coil_le: 0.0,
                transformer_ratio: 7.0,
                transformer_l_pri: 1.2,
                transformer_dcr_pri: 5.0,
                transformer_dcr_sec: 35.0,
                transformer_cap: 70e-12,
                coupling_cap: Some(25e-12),
                fet_output_r: Some(160.0),
            }),

            // ── Direct pickup (no mic circuit) ──────────────────────────
            Self::Flat | Self::Measurement | Self::Custom => None,
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Microphone response curve data
// ═══════════════════════════════════════════════════════════════════════════

impl MicModel {
    /// Polar pattern of this mic model.
    pub fn polar_pattern(&self) -> PolarPattern {
        match self {
            Self::Sm57 | Self::Sm7b | Self::Md421 | Self::Re20 | Self::M88 => {
                PolarPattern::Cardioid
            }
            Self::E906 | Self::E609 => PolarPattern::Supercardioid,
            Self::M160 | Self::M201 => PolarPattern::Hypercardioid,
            Self::R121 | Self::R101 | Self::Fathead => PolarPattern::Figure8,
            Self::U87 | Self::C414 => PolarPattern::Cardioid, // multi-pattern, default cardioid
            Self::Km84 | Self::C451 => PolarPattern::Cardioid,
            Self::Flat | Self::Measurement => PolarPattern::Omni,
            Self::Custom => PolarPattern::Cardioid,
        }
    }

    /// Whether this mic exhibits proximity effect (bass boost at close distance).
    pub fn has_proximity(&self) -> bool {
        match self {
            Self::Flat | Self::Measurement | Self::Re20 => false,
            Self::Sm7b => false, // reduced proximity by design
            _ => match self.polar_pattern() {
                PolarPattern::Omni => false,
                _ => true,
            },
        }
    }

    /// Frequency response shaping as (frequency_hz, gain_db) breakpoints.
    ///
    /// These approximate the published frequency response curves relative to flat.
    /// The cab transform converts these into filter component values.
    pub fn response_curve(&self) -> Vec<(f64, f64)> {
        match self {
            Self::Sm57 => vec![
                (100.0, 0.0),
                (200.0, 1.0),
                (1000.0, 0.0),
                (2000.0, 2.0),
                (5000.0, 5.0),   // presence peak
                (6000.0, 6.0),
                (8000.0, 3.0),
                (10000.0, 0.0),
                (15000.0, -6.0),
            ],
            Self::Sm7b => vec![
                (100.0, 0.0),
                (500.0, 0.0),
                (2000.0, 1.0),
                (5000.0, 2.0),
                (8000.0, 1.0),
                (12000.0, -2.0),
                (15000.0, -5.0),
            ],
            Self::Md421 => vec![
                (100.0, 2.0),
                (200.0, 1.0),
                (1000.0, 0.0),
                (3000.0, 2.0),
                (5000.0, 3.0),
                (8000.0, 1.0),
                (12000.0, -3.0),
            ],
            Self::E906 => vec![
                (100.0, 0.0),
                (1000.0, 0.0),
                (3000.0, 3.0),
                (5000.0, 5.0),
                (8000.0, 2.0),
                (12000.0, -4.0),
            ],
            Self::E609 => vec![
                (100.0, 0.0),
                (1000.0, 0.0),
                (3000.0, 4.0),
                (6000.0, 5.0),
                (10000.0, 0.0),
                (15000.0, -8.0),
            ],
            Self::R121 => vec![
                (100.0, 1.0),
                (500.0, 0.0),
                (2000.0, 0.0),
                (4000.0, -1.0),
                (6000.0, -3.0),
                (8000.0, -6.0),
                (10000.0, -10.0),
                (12000.0, -15.0),
            ],
            Self::R101 => vec![
                (100.0, 0.0),
                (1000.0, 0.0),
                (4000.0, -1.0),
                (6000.0, -2.0),
                (8000.0, -4.0),
                (10000.0, -8.0),
                (12000.0, -12.0),
            ],
            Self::Fathead => vec![
                (100.0, 1.0),
                (500.0, 0.0),
                (2000.0, 0.0),
                (5000.0, -2.0),
                (8000.0, -5.0),
                (10000.0, -10.0),
            ],
            Self::M160 => vec![
                (100.0, 0.0),
                (1000.0, 0.0),
                (3000.0, 1.0),
                (5000.0, 0.0),
                (8000.0, -3.0),
                (10000.0, -6.0),
                (15000.0, -12.0),
            ],
            Self::U87 => vec![
                (50.0, 0.0),
                (200.0, 0.0),
                (2000.0, 0.0),
                (5000.0, 2.0),
                (8000.0, 4.0),
                (10000.0, 3.0),
                (12000.0, 0.0),
                (16000.0, -3.0),
            ],
            Self::C414 => vec![
                (50.0, 0.0),
                (200.0, 0.0),
                (2000.0, 1.0),
                (5000.0, 3.0),
                (8000.0, 5.0),
                (10000.0, 4.0),
                (12000.0, 2.0),
                (16000.0, -2.0),
            ],
            Self::Km84 => vec![
                (100.0, 0.0),
                (1000.0, 0.0),
                (5000.0, 2.0),
                (8000.0, 3.0),
                (10000.0, 2.0),
                (15000.0, -3.0),
            ],
            Self::C451 => vec![
                (100.0, 0.0),
                (1000.0, 0.0),
                (5000.0, 3.0),
                (8000.0, 5.0),
                (10000.0, 4.0),
                (15000.0, -2.0),
            ],
            Self::Re20 => vec![
                (50.0, 0.0),
                (200.0, 0.0),
                (1000.0, 0.0),
                (3000.0, 1.0),
                (5000.0, 2.0),
                (8000.0, 1.0),
                (12000.0, -2.0),
            ],
            Self::M88 => vec![
                (100.0, 1.0),
                (500.0, 0.0),
                (2000.0, 1.0),
                (5000.0, 3.0),
                (8000.0, 1.0),
                (12000.0, -3.0),
            ],
            Self::M201 => vec![
                (100.0, 0.0),
                (1000.0, 0.0),
                (4000.0, 2.0),
                (6000.0, 4.0),
                (8000.0, 3.0),
                (12000.0, -2.0),
            ],
            // Flat / measurement mics: no coloration
            Self::Flat | Self::Measurement | Self::Custom => vec![],
        }
    }

    /// Proximity effect boost in dB at 100 Hz for the "standard" close distance (5 cm).
    /// Scales inversely with distance.
    pub fn proximity_boost_db(&self) -> f64 {
        match self {
            Self::Sm57 => 6.0,
            Self::Md421 => 5.0,
            Self::E906 | Self::E609 => 4.0,
            Self::R121 | Self::R101 | Self::Fathead => 8.0, // figure-8 has strong proximity
            Self::M88 => 5.0,
            Self::M160 => 3.0,
            Self::U87 | Self::C414 => 5.0,
            Self::Km84 | Self::C451 => 2.0,
            Self::M201 => 3.0,
            _ => 0.0, // Flat, Measurement, RE20, SM7B, Custom
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Tests
// ═══════════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    #[test]
    fn v30_derived_params() {
        let p = DriverPreset::V30.params();
        let fs = 1.0 / (2.0 * PI * (p.mms * p.cms).sqrt());
        assert!(
            (fs - 75.0).abs() < 10.0,
            "V30 Fs should be ~75 Hz, got {fs:.1}"
        );
    }

    #[test]
    fn greenback_derived_params() {
        let p = DriverPreset::G12mGreenback.params();
        let fs = 1.0 / (2.0 * PI * (p.mms * p.cms).sqrt());
        assert!(
            (fs - 75.0).abs() < 15.0,
            "Greenback Fs should be ~75 Hz, got {fs:.1}"
        );
    }

    #[test]
    fn all_presets_have_positive_params() {
        let presets = [
            DriverPreset::V30,
            DriverPreset::G12mGreenback,
            DriverPreset::G12hAnniversary,
            DriverPreset::G12t75,
            DriverPreset::G12_65,
            DriverPreset::G12Blue,
            DriverPreset::G12Creamback,
            DriverPreset::G12Evh,
            DriverPreset::P12r,
            DriverPreset::C12n,
            DriverPreset::C12q,
            DriverPreset::P10r,
            DriverPreset::SwampThang,
            DriverPreset::ManOWar,
            DriverPreset::Cv75,
            DriverPreset::Legend1258,
            DriverPreset::TexasHeat,
            DriverPreset::CannabisRex,
            DriverPreset::Ev12l,
            DriverPreset::JblD120,
            DriverPreset::WbEt65,
            DriverPreset::WbRetro30,
            DriverPreset::BassliteS2010,
            DriverPreset::Kappalite3015,
            DriverPreset::Ampeg10,
        ];
        for preset in &presets {
            let p = preset.params();
            assert!(p.re > 0.0, "{preset:?}: Re must be positive");
            assert!(p.le > 0.0, "{preset:?}: Le must be positive");
            assert!(p.bl > 0.0, "{preset:?}: Bl must be positive");
            assert!(p.mms > 0.0, "{preset:?}: Mms must be positive");
            assert!(p.cms > 0.0, "{preset:?}: Cms must be positive");
            assert!(p.rms > 0.0, "{preset:?}: Rms must be positive");
            assert!(p.sd > 0.0, "{preset:?}: Sd must be positive");
        }
    }

    #[test]
    fn mic_sm57_has_presence_peak() {
        let curve = MicModel::Sm57.response_curve();
        let max_db = curve.iter().map(|&(_, db)| db).fold(f64::NEG_INFINITY, f64::max);
        assert!(max_db >= 5.0, "SM57 should have ~6 dB presence peak");
    }

    #[test]
    fn ribbon_mics_roll_off_highs() {
        for mic in [MicModel::R121, MicModel::M160, MicModel::Fathead] {
            let curve = mic.response_curve();
            let last_db = curve.last().unwrap().1;
            assert!(last_db < -5.0, "{mic:?} should have significant HF rolloff");
        }
    }

    #[test]
    fn flat_mic_has_no_response_curve() {
        assert!(MicModel::Flat.response_curve().is_empty());
        assert!(MicModel::Measurement.response_curve().is_empty());
    }

    #[test]
    fn proximity_effect_consistency() {
        assert!(!MicModel::Re20.has_proximity());
        assert!(!MicModel::Flat.has_proximity());
        assert!(MicModel::Sm57.has_proximity());
        assert!(MicModel::R121.has_proximity());
        // Figure-8 mics have the strongest proximity
        assert!(MicModel::R121.proximity_boost_db() > MicModel::Sm57.proximity_boost_db());
    }

    #[test]
    fn dynamic_mics_have_circuit_params() {
        for mic in [
            MicModel::Sm57,
            MicModel::Sm7b,
            MicModel::Md421,
            MicModel::E906,
            MicModel::Re20,
            MicModel::M88,
        ] {
            let p = mic.circuit_params().expect(&format!("{mic:?} should have circuit params"));
            assert!(p.voice_coil_re > 1.0, "{mic:?}: voice coil R should be > 1Ω");
            assert!(p.transformer_ratio > 10.0, "{mic:?}: step-up ratio should be > 10");
            assert!(p.coupling_cap.is_none(), "{mic:?}: dynamic mic should have no coupling cap");
        }
    }

    #[test]
    fn ribbon_mics_have_low_impedance() {
        for mic in [MicModel::R121, MicModel::M160, MicModel::Fathead] {
            let p = mic.circuit_params().unwrap();
            assert!(p.voice_coil_re < 2.0, "{mic:?}: ribbon R should be < 2Ω");
            assert!(
                p.transformer_ratio > 25.0,
                "{mic:?}: ribbon step-up should be high (> 25)"
            );
        }
    }

    #[test]
    fn condenser_mics_have_coupling_cap() {
        for mic in [MicModel::U87, MicModel::C414, MicModel::Km84, MicModel::C451] {
            let p = mic.circuit_params().unwrap();
            assert!(p.coupling_cap.is_some(), "{mic:?}: condenser should have coupling cap");
            assert!(p.fet_output_r.is_some(), "{mic:?}: condenser should have FET output R");
            assert!(
                p.transformer_ratio < 15.0,
                "{mic:?}: condenser step-up should be modest (< 15)"
            );
        }
    }

    #[test]
    fn flat_mics_have_no_circuit() {
        assert!(MicModel::Flat.circuit_params().is_none());
        assert!(MicModel::Measurement.circuit_params().is_none());
        assert!(MicModel::Custom.circuit_params().is_none());
    }
}
