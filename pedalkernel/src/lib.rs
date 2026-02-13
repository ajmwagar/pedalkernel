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

pub mod board;
pub mod compiler;
pub mod dsl;
pub mod elements;
pub mod kicad;
pub mod pedalboard;
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

    /// Set a named control parameter (0.0–1.0). Default: no-op.
    fn set_control(&mut self, _label: &str, _value: f64) {}
}

// ---------------------------------------------------------------------------
// JACK real-time audio engine (requires `jack-rt` feature)
// ---------------------------------------------------------------------------

#[cfg(feature = "jack-rt")]
mod jack_engine {
    use crate::PedalProcessor;
    use jack::{AudioIn, AudioOut, Client, ClientOptions, Control, ProcessHandler, ProcessScope};
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::sync::{Arc, Mutex};

    /// JACK process handler wrapping a PedalProcessor.
    pub struct JackProcessor<P: PedalProcessor> {
        processor: P,
        in_port: jack::Port<AudioIn>,
        out_port: jack::Port<AudioOut>,
    }

    impl<P: PedalProcessor + Send> ProcessHandler for JackProcessor<P> {
        fn process(&mut self, _client: &Client, ps: &ProcessScope) -> Control {
            let input = self.in_port.as_slice(ps);
            let output = self.out_port.as_mut_slice(ps);

            for (out, &inp) in output.iter_mut().zip(input.iter()) {
                *out = self.processor.process(inp as f64) as f32;
            }

            Control::Continue
        }
    }

    // ── Shared controls for real-time TUI ↔ JACK communication ──────────

    /// Shared control state for real-time parameter updates between a UI
    /// thread and the JACK process callback.
    pub struct SharedControls {
        pending: Mutex<Vec<(String, f64)>>,
        bypassed: AtomicBool,
    }

    impl SharedControls {
        pub fn new() -> Self {
            Self {
                pending: Mutex::new(Vec::new()),
                bypassed: AtomicBool::new(false),
            }
        }

        /// Queue a control update (called from the UI thread).
        pub fn set_control(&self, label: &str, value: f64) {
            if let Ok(mut pending) = self.pending.lock() {
                pending.push((label.to_string(), value));
            }
        }

        pub fn set_bypassed(&self, bypassed: bool) {
            self.bypassed.store(bypassed, Ordering::Relaxed);
        }

        pub fn is_bypassed(&self) -> bool {
            self.bypassed.load(Ordering::Relaxed)
        }
    }

    /// JACK process handler with live control updates from a shared state.
    pub struct JackProcessorLive<P: PedalProcessor> {
        processor: P,
        in_port: jack::Port<AudioIn>,
        out_port: jack::Port<AudioOut>,
        controls: Arc<SharedControls>,
    }

    impl<P: PedalProcessor + Send> ProcessHandler for JackProcessorLive<P> {
        fn process(&mut self, _client: &Client, ps: &ProcessScope) -> Control {
            // Drain pending control updates (non-blocking).
            if let Ok(mut pending) = self.controls.pending.try_lock() {
                for (label, value) in pending.drain(..) {
                    self.processor.set_control(&label, value);
                }
            }

            let input = self.in_port.as_slice(ps);
            let output = self.out_port.as_mut_slice(ps);

            if self.controls.bypassed.load(Ordering::Relaxed) {
                for (out, &inp) in output.iter_mut().zip(input.iter()) {
                    *out = inp;
                }
            } else {
                for (out, &inp) in output.iter_mut().zip(input.iter()) {
                    *out = self.processor.process(inp as f64) as f32;
                }
            }

            Control::Continue
        }
    }

    /// A `PedalProcessor` wrapper that ignores JACK input and feeds samples
    /// from a pre-loaded circular buffer (looping WAV file) into the inner processor.
    pub struct WavLoopProcessor<P: PedalProcessor> {
        samples: Vec<f64>,
        position: usize,
        inner: P,
    }

    impl<P: PedalProcessor> WavLoopProcessor<P> {
        pub fn new(samples: Vec<f64>, inner: P) -> Self {
            Self {
                samples,
                position: 0,
                inner,
            }
        }
    }

    impl<P: PedalProcessor + Send> PedalProcessor for WavLoopProcessor<P> {
        fn process(&mut self, _input: f64) -> f64 {
            let sample = if self.samples.is_empty() {
                0.0
            } else {
                let s = self.samples[self.position];
                self.position = (self.position + 1) % self.samples.len();
                s
            };
            self.inner.process(sample)
        }

        fn set_sample_rate(&mut self, rate: f64) {
            self.inner.set_sample_rate(rate);
        }

        fn reset(&mut self) {
            self.position = 0;
            self.inner.reset();
        }

        fn set_control(&mut self, label: &str, value: f64) {
            self.inner.set_control(label, value);
        }
    }

    /// JACK-based real-time audio engine.
    ///
    /// Connects a `PedalProcessor` to the system audio graph via JACK.
    /// Target: sub-5 ms total latency through a Scarlett 2i2 on Linux
    /// (48 kHz, 64-sample buffer).
    ///
    /// Enable with: `cargo build --features jack-rt`
    pub struct AudioEngine;

    impl AudioEngine {
        /// Create a JACK client, register ports, and start processing.
        ///
        /// Blocks until the returned `AsyncClient` is dropped.
        pub fn run<P: PedalProcessor + Send + 'static>(
            name: &str,
            mut processor: P,
        ) -> Result<jack::AsyncClient<(), JackProcessor<P>>, jack::Error> {
            let (client, _status) = Client::new(name, ClientOptions::NO_START_SERVER)?;
            processor.set_sample_rate(client.sample_rate() as f64);

            let in_port = client.register_port("in", AudioIn)?;
            let out_port = client.register_port("out", AudioOut)?;

            let handler = JackProcessor {
                processor,
                in_port,
                out_port,
            };
            client.activate_async((), handler)
        }

        /// Create a JACK client (for port enumeration before activation).
        ///
        /// Tries to connect to an existing JACK server first; falls back to
        /// allowing the server to auto-start if `NO_START_SERVER` fails.
        pub fn create_client(name: &str) -> Result<Client, jack::Error> {
            match Client::new(name, ClientOptions::NO_START_SERVER) {
                Ok((client, _status)) => Ok(client),
                Err(_) => {
                    let (client, _status) = Client::new(name, ClientOptions::empty())?;
                    Ok(client)
                }
            }
        }

        /// List available audio input sources (JACK ports that produce audio).
        /// These are ports you can read from — typically `system:capture_*`.
        pub fn list_input_sources(client: &Client) -> Vec<String> {
            client.ports(
                None,
                Some("32 bit float mono audio"),
                jack::PortFlags::IS_OUTPUT,
            )
        }

        /// List available audio output destinations (JACK ports that consume audio).
        /// These are ports you can write to — typically `system:playback_*`.
        pub fn list_output_destinations(client: &Client) -> Vec<String> {
            client.ports(
                None,
                Some("32 bit float mono audio"),
                jack::PortFlags::IS_INPUT,
            )
        }

        /// Activate a processor with shared controls for live parameter updates.
        ///
        /// Returns the async client and the full names of the registered
        /// input and output ports (for connecting to other JACK ports).
        pub fn start<P: PedalProcessor + Send + 'static>(
            client: Client,
            mut processor: P,
            controls: Arc<SharedControls>,
        ) -> Result<
            (jack::AsyncClient<(), JackProcessorLive<P>>, String, String),
            jack::Error,
        > {
            processor.set_sample_rate(client.sample_rate() as f64);

            let in_port = client.register_port("in", AudioIn)?;
            let out_port = client.register_port("out", AudioOut)?;

            // Build full port names: "<client_name>:<port_name>"
            let client_name = client.name().to_string();
            let in_name = format!("{client_name}:in");
            let out_name = format!("{client_name}:out");

            let handler = JackProcessorLive {
                processor,
                in_port,
                out_port,
                controls,
            };
            let async_client = client.activate_async((), handler)?;

            Ok((async_client, in_name, out_name))
        }
    }
}

#[cfg(feature = "jack-rt")]
pub use jack_engine::{AudioEngine, SharedControls, WavLoopProcessor};

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

    // -----------------------------------------------------------------------
    // Per-pedal .pedal file parse tests
    // -----------------------------------------------------------------------

    /// Helper: read and parse a .pedal example file, panicking with context on failure.
    fn parse_example(filename: &str) -> dsl::PedalDef {
        let path = format!("examples/{filename}");
        let src =
            std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("failed to read {path}: {e}"));
        dsl::parse_pedal_file(&src).unwrap_or_else(|e| panic!("failed to parse {path}: {e}"))
    }

    #[test]
    fn pedal_tube_screamer() {
        let p = parse_example("tube_screamer.pedal");
        assert_eq!(p.name, "Tube Screamer");
        assert_eq!(p.components.len(), 4);
        assert_eq!(p.nets.len(), 4);
        assert_eq!(p.controls.len(), 2);
        assert_eq!(p.controls[0].label, "Drive");
    }

    #[test]
    fn pedal_fuzz_face() {
        let p = parse_example("fuzz_face.pedal");
        assert_eq!(p.name, "Fuzz Face");
        assert_eq!(p.components.len(), 9);
        assert_eq!(p.nets.len(), 13);
        assert_eq!(p.controls.len(), 2);
        let labels: Vec<&str> = p.controls.iter().map(|c| c.label.as_str()).collect();
        assert!(labels.contains(&"Fuzz"));
        assert!(labels.contains(&"Volume"));
    }

    #[test]
    fn pedal_big_muff() {
        let p = parse_example("big_muff.pedal");
        assert_eq!(p.name, "Big Muff");
        assert_eq!(p.components.len(), 13);
        assert_eq!(p.nets.len(), 13);
        assert_eq!(p.controls.len(), 3);
        let labels: Vec<&str> = p.controls.iter().map(|c| c.label.as_str()).collect();
        assert!(labels.contains(&"Sustain"));
        assert!(labels.contains(&"Tone"));
        assert!(labels.contains(&"Volume"));
    }

    #[test]
    fn pedal_dyna_comp() {
        let p = parse_example("dyna_comp.pedal");
        assert_eq!(p.name, "MXR Dyna Comp");
        assert_eq!(p.components.len(), 9);
        assert_eq!(p.nets.len(), 11);
        assert_eq!(p.controls.len(), 2);
        let labels: Vec<&str> = p.controls.iter().map(|c| c.label.as_str()).collect();
        assert!(labels.contains(&"Sensitivity"));
        assert!(labels.contains(&"Output"));
        // Verify opamp is present (OTA topology)
        assert!(p
            .components
            .iter()
            .any(|c| c.kind == dsl::ComponentKind::OpAmp));
    }

    #[test]
    fn pedal_proco_rat() {
        let p = parse_example("proco_rat.pedal");
        assert_eq!(p.name, "ProCo RAT");
        assert_eq!(p.components.len(), 12);
        assert_eq!(p.nets.len(), 15);
        assert_eq!(p.controls.len(), 3);
        let labels: Vec<&str> = p.controls.iter().map(|c| c.label.as_str()).collect();
        assert!(labels.contains(&"Distortion"));
        assert!(labels.contains(&"Filter"));
        assert!(labels.contains(&"Volume"));
        // Verify opamp + hard clipping diode pair
        assert!(p
            .components
            .iter()
            .any(|c| c.kind == dsl::ComponentKind::OpAmp));
        assert!(p
            .components
            .iter()
            .any(|c| c.kind == dsl::ComponentKind::DiodePair(dsl::DiodeType::Silicon)));
    }

    #[test]
    fn pedal_blues_driver() {
        let p = parse_example("blues_driver.pedal");
        assert_eq!(p.name, "Boss Blues Driver");
        assert_eq!(p.components.len(), 15);
        assert_eq!(p.nets.len(), 18);
        assert_eq!(p.controls.len(), 3);
        let labels: Vec<&str> = p.controls.iter().map(|c| c.label.as_str()).collect();
        assert!(labels.contains(&"Gain"));
        assert!(labels.contains(&"Tone"));
        assert!(labels.contains(&"Level"));
        // Verify NPN transistor + asymmetric single diode
        assert!(p
            .components
            .iter()
            .any(|c| c.kind == dsl::ComponentKind::Npn));
        assert!(p
            .components
            .iter()
            .any(|c| c.kind == dsl::ComponentKind::Diode(dsl::DiodeType::Silicon)));
    }

    #[test]
    fn pedal_klon_centaur() {
        let p = parse_example("klon_centaur.pedal");
        assert_eq!(p.name, "Klon Centaur");
        assert_eq!(p.components.len(), 15);
        assert_eq!(p.nets.len(), 19);
        assert_eq!(p.controls.len(), 3);
        let labels: Vec<&str> = p.controls.iter().map(|c| c.label.as_str()).collect();
        assert!(labels.contains(&"Gain"));
        assert!(labels.contains(&"Treble"));
        assert!(labels.contains(&"Output"));
        // Verify dual opamps + germanium diode pair in feedback
        let opamp_count = p
            .components
            .iter()
            .filter(|c| c.kind == dsl::ComponentKind::OpAmp)
            .count();
        assert_eq!(opamp_count, 2, "Klon uses dual opamps");
        assert!(p
            .components
            .iter()
            .any(|c| c.kind == dsl::ComponentKind::DiodePair(dsl::DiodeType::Germanium)));
    }

    #[test]
    fn all_pedal_files_export_kicad() {
        let files = [
            "tube_screamer.pedal",
            "fuzz_face.pedal",
            "big_muff.pedal",
            "dyna_comp.pedal",
            "proco_rat.pedal",
            "blues_driver.pedal",
            "klon_centaur.pedal",
        ];
        for f in files {
            let p = parse_example(f);
            let netlist = kicad::export_kicad_netlist(&p);
            assert!(
                netlist.contains("(export (version D)"),
                "{f} KiCad export missing header"
            );
            assert!(
                netlist.contains(&p.name),
                "{f} KiCad export missing pedal name"
            );
        }
    }
}
