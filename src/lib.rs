//! PedalKernel — compile guitar pedal circuit definitions into real-time
//! audio DSP kernels using Wave Digital Filters (WDFs).
//!
//! # Modules
//!
//! - [`dsl`] — nom-based parser for `.pedal` circuit definition files
//! - [`elements`] — WDF one-port elements (R, C, L) and nonlinear roots (diodes)
//! - [`tree`] — WDF adaptors (series, parallel) and processing engine
//! - [`pedals`] — ready-to-use pedal implementations (Overdrive, Fuzz, Delay)
//! - [`kicad`] — KiCad netlist export from the parsed AST
//! - [`wav`] — WAV file I/O for offline rendering and testing

pub mod dsl;
pub mod elements;
pub mod kicad;
pub mod pedals;
pub mod tree;
pub mod wav;

/// Audio processor trait for pedals.
pub trait PedalProcessor {
    /// Process a single sample.
    fn process(&mut self, input: f64) -> f64;

    /// Set sample rate (call before processing).
    fn set_sample_rate(&mut self, rate: f64);

    /// Reset all internal state.
    fn reset(&mut self);
}

// ---------------------------------------------------------------------------
// Integration tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dsl_to_kicad_roundtrip() {
        let src = r#"
pedal "Test Pedal" {
  components {
    R1: resistor(10k)
    C1: cap(100n)
    D1: diode_pair(silicon)
  }
  nets {
    in -> C1.a
    C1.b -> R1.a, D1.a
    D1.b -> gnd
    R1.b -> out
  }
  controls {
    R1.position -> "Tone" [0.0, 1.0] = 0.5
  }
}
"#;
        let pedal = dsl::parse_pedal_file(src).unwrap();
        assert_eq!(pedal.name, "Test Pedal");
        assert_eq!(pedal.components.len(), 3);
        assert_eq!(pedal.nets.len(), 4);
        assert_eq!(pedal.controls.len(), 1);

        let netlist = kicad::export_kicad_netlist(&pedal);
        assert!(netlist.contains("(export (version D)"));
        assert!(netlist.contains("10.0kΩ"));
    }

    #[test]
    fn wdf_overdrive_pipeline() {
        use pedals::Overdrive;

        let mut od = Overdrive::new(48000.0);
        od.set_gain(0.7);

        // Process a short burst
        let input = wav::sine_wave(440.0, 0.1, 48000);
        let output: Vec<f64> = input.iter().map(|&s| od.process(s)).collect();

        // Should have nonzero output
        let max_out = output.iter().copied().fold(0.0_f64, |a, b| a.max(b.abs()));
        assert!(max_out > 0.001);

        // Should be clipped relative to input
        let max_in = input.iter().copied().fold(0.0_f64, |a, b| a.max(b.abs()));
        // At high gain the output peak may be less than input peak (clipping)
        // or at least the waveform is modified
        assert!(max_out < max_in * 5.0, "output shouldn't blow up");
    }

    #[test]
    fn wdf_fuzz_pipeline() {
        use pedals::FuzzFace;

        let mut ff = FuzzFace::new(48000.0);
        ff.set_fuzz(0.8);

        let input = wav::sine_wave(196.0, 0.1, 48000);
        let output: Vec<f64> = input.iter().map(|&s| ff.process(s)).collect();
        let max_out = output.iter().copied().fold(0.0_f64, |a, b| a.max(b.abs()));
        assert!(max_out > 0.0001, "fuzz should produce output");
    }
}
