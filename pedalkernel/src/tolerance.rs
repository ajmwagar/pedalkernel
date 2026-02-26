//! Component tolerance randomization.
//!
//! Real resistors and capacitors are ±5–20% from their nominal values.
//! Two identical pedals sound slightly different because of this.
//!
//! This module provides a seeded pseudo-random tolerance system:
//! - Each "unit" (physical pedal instance) gets a seed
//! - Component values are deterministically varied from nominal
//! - A saved preset always sounds like the same physical pedal
//! - Two instances of the same model diverge slightly
//!
//! The randomization uses a simple but deterministic hash-based approach
//! so that the same seed + component index always produces the same offset.

/// Tolerance grade for electronic components.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ToleranceGrade {
    /// ±1% (precision resistors, film caps). Tight, predictable.
    Precision,
    /// ±5% (standard metal film resistors, quality ceramic caps).
    Standard,
    /// ±10% (carbon film resistors, electrolytic caps). Vintage character.
    Loose,
    /// ±20% (carbon composition resistors, cheap ceramics). Maximum variation.
    Wide,
    /// No tolerance applied. Ideal components.
    Ideal,
}

impl ToleranceGrade {
    /// Get the tolerance as a fractional multiplier (e.g., 0.05 for ±5%).
    pub fn fraction(self) -> f64 {
        match self {
            ToleranceGrade::Precision => 0.01,
            ToleranceGrade::Standard => 0.05,
            ToleranceGrade::Loose => 0.10,
            ToleranceGrade::Wide => 0.20,
            ToleranceGrade::Ideal => 0.0,
        }
    }
}

/// Component tolerance engine.
///
/// Given a seed (representing a specific physical pedal unit), generates
/// deterministic variations for each component. The same seed always
/// produces the same set of component variations.
///
/// Usage:
/// ```ignore
/// let tol = ToleranceEngine::new(42, ToleranceGrade::Loose);
/// let actual_r = tol.apply(nominal_r, 0);  // component index 0
/// let actual_c = tol.apply(nominal_c, 1);  // component index 1
/// ```
#[derive(Debug, Clone)]
pub struct ToleranceEngine {
    /// Seed for this specific pedal unit instance.
    seed: u64,
    /// Tolerance grade (how much variation to apply).
    resistor_grade: ToleranceGrade,
    /// Tolerance grade for capacitors (often looser than resistors).
    capacitor_grade: ToleranceGrade,
}

impl ToleranceEngine {
    /// Create a new tolerance engine with uniform grade for all components.
    pub fn new(seed: u64, grade: ToleranceGrade) -> Self {
        Self {
            seed,
            resistor_grade: grade,
            capacitor_grade: grade,
        }
    }

    /// Create with separate grades for resistors and capacitors.
    ///
    /// Capacitors typically have looser tolerances than resistors in
    /// vintage pedals. A common combination: ±5% resistors, ±10% caps.
    pub fn with_grades(
        seed: u64,
        resistor_grade: ToleranceGrade,
        capacitor_grade: ToleranceGrade,
    ) -> Self {
        Self {
            seed,
            resistor_grade,
            capacitor_grade,
        }
    }

    /// Create an engine that applies no tolerance (ideal components).
    pub fn ideal() -> Self {
        Self::new(0, ToleranceGrade::Ideal)
    }

    /// Apply tolerance to a resistor value.
    ///
    /// Returns the actual value after applying a deterministic random
    /// offset based on the seed and component index.
    pub fn apply_resistor(&self, nominal: f64, component_index: usize) -> f64 {
        self.apply_internal(nominal, component_index, self.resistor_grade)
    }

    /// Apply tolerance to a capacitor value.
    pub fn apply_capacitor(&self, nominal: f64, component_index: usize) -> f64 {
        self.apply_internal(nominal, component_index, self.capacitor_grade)
    }

    /// Core tolerance application.
    fn apply_internal(&self, nominal: f64, component_index: usize, grade: ToleranceGrade) -> f64 {
        let fraction = grade.fraction();
        if fraction == 0.0 {
            return nominal;
        }

        // Generate a deterministic pseudo-random value in [-1, 1]
        // using a hash of (seed, component_index).
        let random = self.deterministic_random(component_index);

        // Apply tolerance: nominal * (1 + fraction * random)
        // This gives a value in [nominal * (1-fraction), nominal * (1+fraction)]
        nominal * (1.0 + fraction * random)
    }

    /// Deterministic pseudo-random number in [-1, 1] from seed + index.
    ///
    /// Uses a simple but effective hash (SplitMix64-derived) to ensure
    /// good distribution across different seeds and indices.
    fn deterministic_random(&self, component_index: usize) -> f64 {
        // Combine seed and index
        let mut z = self.seed.wrapping_add(component_index as u64).wrapping_mul(0x9E3779B97F4A7C15);
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58476D1CE4E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D049BB133111EB);
        z ^= z >> 31;

        // Convert to [-1, 1] range
        (z as f64 / u64::MAX as f64) * 2.0 - 1.0
    }

    /// Get the seed.
    pub fn seed(&self) -> u64 {
        self.seed
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ideal_tolerance_unchanged() {
        let tol = ToleranceEngine::ideal();
        let nominal = 4700.0;
        let actual = tol.apply_resistor(nominal, 0);
        assert!(
            (actual - nominal).abs() < 1e-10,
            "Ideal should not change value: {actual}"
        );
    }

    #[test]
    fn tolerance_deterministic() {
        let tol1 = ToleranceEngine::new(42, ToleranceGrade::Loose);
        let tol2 = ToleranceEngine::new(42, ToleranceGrade::Loose);

        for i in 0..20 {
            let v1 = tol1.apply_resistor(1000.0, i);
            let v2 = tol2.apply_resistor(1000.0, i);
            assert!(
                (v1 - v2).abs() < 1e-10,
                "Same seed should give same result: i={i}, v1={v1}, v2={v2}"
            );
        }
    }

    #[test]
    fn tolerance_varies_with_seed() {
        let tol1 = ToleranceEngine::new(1, ToleranceGrade::Loose);
        let tol2 = ToleranceEngine::new(2, ToleranceGrade::Loose);

        let v1 = tol1.apply_resistor(1000.0, 0);
        let v2 = tol2.apply_resistor(1000.0, 0);

        assert!(
            (v1 - v2).abs() > 0.01,
            "Different seeds should give different values: v1={v1}, v2={v2}"
        );
    }

    #[test]
    fn tolerance_within_bounds() {
        let tol = ToleranceEngine::new(123, ToleranceGrade::Loose);
        let nominal = 10_000.0;
        let fraction = ToleranceGrade::Loose.fraction(); // 0.10

        for i in 0..1000 {
            let actual = tol.apply_resistor(nominal, i);
            let lo = nominal * (1.0 - fraction);
            let hi = nominal * (1.0 + fraction);
            assert!(
                actual >= lo && actual <= hi,
                "Component {i} out of bounds: {actual}, expected [{lo}, {hi}]"
            );
        }
    }

    #[test]
    fn tolerance_varies_across_components() {
        let tol = ToleranceEngine::new(42, ToleranceGrade::Standard);
        let nominal = 1000.0;

        let values: Vec<f64> = (0..100).map(|i| tol.apply_resistor(nominal, i)).collect();

        // Not all values should be the same
        let unique: std::collections::HashSet<u64> =
            values.iter().map(|&v| v.to_bits()).collect();
        assert!(
            unique.len() > 50,
            "Should have good variety across components: {} unique values",
            unique.len()
        );
    }

    #[test]
    fn tolerance_different_grades() {
        let precision = ToleranceEngine::new(42, ToleranceGrade::Precision);
        let wide = ToleranceEngine::new(42, ToleranceGrade::Wide);
        let nominal = 1000.0;

        // Collect deviations
        let precision_dev: f64 = (0..100)
            .map(|i| (precision.apply_resistor(nominal, i) - nominal).abs())
            .sum::<f64>()
            / 100.0;
        let wide_dev: f64 = (0..100)
            .map(|i| (wide.apply_resistor(nominal, i) - nominal).abs())
            .sum::<f64>()
            / 100.0;

        assert!(
            wide_dev > precision_dev * 3.0,
            "Wide should deviate much more than precision: wide={wide_dev:.2}, precision={precision_dev:.2}"
        );
    }

    #[test]
    fn separate_resistor_capacitor_grades() {
        let tol = ToleranceEngine::with_grades(
            42,
            ToleranceGrade::Standard,  // ±5% for resistors
            ToleranceGrade::Wide,      // ±20% for caps
        );
        let nominal = 1000.0;

        // Capacitor deviation should be larger
        let r_dev: f64 = (0..100)
            .map(|i| (tol.apply_resistor(nominal, i) - nominal).abs())
            .sum::<f64>()
            / 100.0;
        let c_dev: f64 = (0..100)
            .map(|i| (tol.apply_capacitor(nominal, i) - nominal).abs())
            .sum::<f64>()
            / 100.0;

        assert!(
            c_dev > r_dev * 2.0,
            "Capacitor deviation should be larger: c={c_dev:.2}, r={r_dev:.2}"
        );
    }

    #[test]
    fn tolerance_distribution_centered() {
        // The distribution should be roughly centered around the nominal value
        let tol = ToleranceEngine::new(777, ToleranceGrade::Loose);
        let nominal = 1000.0;

        let mean: f64 = (0..10000)
            .map(|i| tol.apply_resistor(nominal, i))
            .sum::<f64>()
            / 10000.0;

        // Mean should be close to nominal (within 2% for uniform distribution)
        assert!(
            (mean - nominal).abs() < nominal * 0.02,
            "Mean should be near nominal: mean={mean:.2}, nominal={nominal}"
        );
    }
}
