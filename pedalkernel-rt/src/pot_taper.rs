/// Potentiometer taper type.
///
/// Following the Asian/European convention:
/// - A (Audio/Logarithmic): Slow start, fast finish. Used for volume controls.
/// - B (Linear): Proportional change. Used for tone/blend controls.
/// - C (Reverse Log): Fast start, slow finish. Rare, for specific compensation.
///
/// Note: Some American manufacturers flip A and B. Always check datasheets.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum PotTaper {
    /// Audio/Logarithmic taper - slow at start, fast at end (volume controls)
    A,
    /// Linear taper - proportional change (default for most controls)
    #[default]
    B,
    /// Reverse logarithmic - fast at start, slow at end (rare)
    C,
}

impl PotTaper {
    /// Apply the taper curve to a linear position (0.0 to 1.0).
    /// Returns the effective resistance ratio (0.0 to 1.0).
    #[inline]
    pub fn apply(self, pos: crate::Wave) -> crate::Wave {
        let pos = pos.clamp(0.0, 1.0);
        match self {
            // Linear: direct mapping
            PotTaper::B => pos,
            // Audio/Log: slow start, fast end
            // Using (10^pos - 1) / 9 which gives ~24% at 50% rotation
            PotTaper::A => {
                (crate::math::powf(10.0 as crate::Wave, pos as crate::Wave) as crate::Wave - 1.0)
                    / 9.0
            }
            // Reverse log: fast start, slow end (inverse of A)
            PotTaper::C => {
                1.0 - (crate::math::powf(10.0 as crate::Wave, (1.0 - pos) as crate::Wave)
                    as crate::Wave
                    - 1.0)
                    / 9.0
            }
        }
    }
}
