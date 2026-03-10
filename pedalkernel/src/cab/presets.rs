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
// Microphone data
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
}
