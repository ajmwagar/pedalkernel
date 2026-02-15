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
#[cfg(feature = "hardware")]
pub mod hw;
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

    /// Set supply voltage in volts (default 9.0).
    ///
    /// Real pedals run at 9V (standard battery), but many players and boutique
    /// builders run them at 12V or 18V for more headroom.  Higher voltage means
    /// the active gain stages (opamps / transistors) can swing further before
    /// hitting their supply rails, preserving more dynamics before the diode
    /// clipping stage.  The diode clipping threshold itself is unchanged.
    fn set_supply_voltage(&mut self, _voltage: f64) {}
}

// ---------------------------------------------------------------------------
// JACK real-time audio engine (requires `jack-rt` feature)
// ---------------------------------------------------------------------------

#[cfg(feature = "jack-rt")]
mod jack_engine {
    use crate::PedalProcessor;
    use jack::{AudioIn, AudioOut, Client, ClientOptions, Control, ProcessHandler, ProcessScope};
    use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
    use std::sync::{Arc, Mutex};

    // ── DC blocking filter ─────────────────────────────────────────────
    //
    // First-order high-pass at ~10 Hz removes DC offset that accumulates
    // from WDF numerical drift, reclaiming headroom that would otherwise
    // push one side of the waveform into DAC hard clipping.

    /// First-order DC blocking filter (high-pass, ~10 Hz cutoff).
    struct DcBlocker {
        x_prev: f64,
        y_prev: f64,
        coeff: f64, // R / (R + 1) where R = 1 / (2π * fc / fs)
    }

    impl DcBlocker {
        fn new(sample_rate: f64) -> Self {
            let fc = 10.0; // 10 Hz cutoff — well below guitar range
            let r = 1.0 / (2.0 * std::f64::consts::PI * fc / sample_rate);
            Self {
                x_prev: 0.0,
                y_prev: 0.0,
                coeff: r / (r + 1.0),
            }
        }

        #[inline]
        fn process(&mut self, x: f64) -> f64 {
            // y[n] = R * (y[n-1] + x[n] - x[n-1])
            self.y_prev = self.coeff * (self.y_prev + x - self.x_prev);
            self.x_prev = x;
            self.y_prev
        }
    }

    // ── Output stage ──────────────────────────────────────────────────
    //
    // Applies DC blocking + soft limiting to prevent DAC hard clipping.
    // DAC hard clipping (output > 1.0) sounds like harsh static/buzz
    // that's unrelated to the musical soft-clipping the WDF diodes produce.

    /// Output stage: DC blocker + soft-knee limiter to prevent DAC hard clipping.
    struct OutputStage {
        dc_blocker: DcBlocker,
    }

    /// Soft-knee limiter threshold.  Signals below this amplitude pass
    /// through *unchanged* — no coloration, no dynamics loss.  Only
    /// signals between the threshold and the DAC ceiling are compressed.
    const LIMITER_THRESHOLD: f64 = 0.9;

    /// Reciprocal of `(1.0 - THRESHOLD)`, precomputed.
    const LIMITER_INV_KNEE: f64 = 1.0 / (1.0 - LIMITER_THRESHOLD);

    impl OutputStage {
        fn new(sample_rate: f64) -> Self {
            Self {
                dc_blocker: DcBlocker::new(sample_rate),
            }
        }

        /// Process a sample: remove DC offset, then soft-limit to [-1.0, 1.0].
        ///
        /// Uses a soft-knee limiter modeled after analog brickwall limiters:
        /// - Below ±0.9: **passthrough** — no coloration whatsoever.
        /// - 0.9 to 1.0: smooth cubic compression toward ±1.0.
        /// - Above 1.0: hard ceiling at ±1.0 (should rarely reach here).
        ///
        /// This preserves 100% of the WDF circuit's tonal character for
        /// signals in the normal range, and only intervenes for peaks that
        /// would otherwise hit the DAC hard clipper.
        #[inline]
        fn process(&mut self, sample: f64) -> f64 {
            let x = self.dc_blocker.process(sample);
            let abs_x = x.abs();
            if abs_x <= LIMITER_THRESHOLD {
                // Below threshold: clean passthrough, zero coloration.
                x
            } else if abs_x >= 1.0 {
                // Above ceiling: hard limit (safety net).
                x.signum()
            } else {
                // Knee region: cubic ease-out from threshold to 1.0.
                // Maps [threshold, 1.0] input → [threshold, 1.0] output
                // with zero derivative at the ceiling (smooth tangent).
                let t = (abs_x - LIMITER_THRESHOLD) * LIMITER_INV_KNEE; // 0..1
                let compressed = LIMITER_THRESHOLD + (1.0 - LIMITER_THRESHOLD) * (2.0 * t - t * t);
                compressed.copysign(x)
            }
        }
    }

    // ── Lock-free control slot ────────────────────────────────────────
    //
    // Replaces the `Mutex<Vec<(String, f64)>>` on the hot path.  The UI
    // thread writes control updates into fixed slots keyed by label.
    // The RT callback reads them without locking — no priority inversion,
    // no heap allocation, no xruns.

    /// A single lock-free control value slot using atomic f64.
    struct ControlSlot {
        label: String,
        /// Atomic storage for the f64 value (bit-cast via u64).
        value: AtomicU64,
        /// Generation counter: bumped on each write so the reader knows
        /// whether a new value is available without a separate dirty flag.
        generation: AtomicU64,
        /// Last generation the reader saw.
        last_read_gen: std::cell::UnsafeCell<u64>,
    }

    // SAFETY: The AtomicU64 fields are inherently thread-safe.
    // `last_read_gen` is only accessed from the RT thread (reader side).
    unsafe impl Send for ControlSlot {}
    unsafe impl Sync for ControlSlot {}

    impl ControlSlot {
        fn new(label: String, initial: f64) -> Self {
            Self {
                label,
                value: AtomicU64::new(initial.to_bits()),
                generation: AtomicU64::new(0),
                last_read_gen: std::cell::UnsafeCell::new(0),
            }
        }

        /// Write a new value (UI thread).
        fn store(&self, value: f64) {
            self.value.store(value.to_bits(), Ordering::Release);
            self.generation.fetch_add(1, Ordering::Release);
        }

        /// Check if a new value is available and read it (RT thread).
        /// Returns `Some(value)` if updated since last read.
        #[inline]
        fn load_if_changed(&self) -> Option<f64> {
            let gen = self.generation.load(Ordering::Acquire);
            // SAFETY: only called from the single RT thread
            let last = unsafe { &mut *self.last_read_gen.get() };
            if gen != *last {
                *last = gen;
                Some(f64::from_bits(self.value.load(Ordering::Acquire)))
            } else {
                None
            }
        }
    }

    /// Result of activating a JACK client: the async handle plus registered
    /// input/output port names.
    pub type ActiveJackClient<P> = (jack::AsyncClient<(), JackProcessorLive<P>>, String, String);

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
    ///
    /// Uses lock-free atomic slots instead of a `Mutex<Vec<>>` to avoid
    /// priority inversion and heap allocation in the RT callback — the
    /// most common cause of xruns (crackle/static) in JACK applications.
    pub struct SharedControls {
        slots: Vec<ControlSlot>,
        /// Fallback for controls not pre-registered as slots.
        pending: Mutex<Vec<(String, f64)>>,
        bypassed: AtomicBool,
    }

    impl Default for SharedControls {
        fn default() -> Self {
            Self::new()
        }
    }

    impl SharedControls {
        pub fn new() -> Self {
            Self {
                slots: Vec::new(),
                pending: Mutex::new(Vec::new()),
                bypassed: AtomicBool::new(false),
            }
        }

        /// Create SharedControls with pre-registered lock-free slots for
        /// known control labels.  This is the preferred constructor — it
        /// eliminates all locking from the RT callback for these controls.
        pub fn with_controls(labels: &[(String, f64)]) -> Self {
            let slots = labels
                .iter()
                .map(|(label, default)| ControlSlot::new(label.clone(), *default))
                .collect();
            Self {
                slots,
                pending: Mutex::new(Vec::new()),
                bypassed: AtomicBool::new(false),
            }
        }

        /// Set a control value (called from the UI thread).
        ///
        /// If the label matches a pre-registered slot, uses lock-free
        /// atomic write.  Otherwise falls back to the Mutex path.
        pub fn set_control(&self, label: &str, value: f64) {
            for slot in &self.slots {
                if slot.label == label {
                    slot.store(value);
                    return;
                }
            }
            // Fallback for unknown labels
            if let Ok(mut pending) = self.pending.lock() {
                pending.push((label.to_string(), value));
            }
        }

        /// Drain changed controls into the processor (called from RT thread).
        /// Lock-free for pre-registered slots; `try_lock` for the fallback.
        #[inline]
        fn drain_into(&self, processor: &mut dyn PedalProcessor) {
            // Lock-free path: check each atomic slot
            for slot in &self.slots {
                if let Some(value) = slot.load_if_changed() {
                    processor.set_control(&slot.label, value);
                }
            }
            // Fallback path (rare): drain any Mutex-queued controls
            if let Ok(mut pending) = self.pending.try_lock() {
                for (label, value) in pending.drain(..) {
                    processor.set_control(&label, value);
                }
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
        output_stage: OutputStage,
    }

    impl<P: PedalProcessor + Send> ProcessHandler for JackProcessorLive<P> {
        fn process(&mut self, _client: &Client, ps: &ProcessScope) -> Control {
            // Drain pending control updates (lock-free for registered slots).
            self.controls.drain_into(&mut self.processor);

            let input = self.in_port.as_slice(ps);
            let output = self.out_port.as_mut_slice(ps);

            if self.controls.bypassed.load(Ordering::Relaxed) {
                for (out, &inp) in output.iter_mut().zip(input.iter()) {
                    *out = inp;
                }
            } else {
                for (out, &inp) in output.iter_mut().zip(input.iter()) {
                    let processed = self.processor.process(inp as f64);
                    // DC-block + soft-limit to prevent DAC hard clipping.
                    *out = self.output_stage.process(processed) as f32;
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
        ) -> Result<ActiveJackClient<P>, jack::Error> {
            let sample_rate = client.sample_rate() as f64;
            processor.set_sample_rate(sample_rate);

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
                output_stage: OutputStage::new(sample_rate),
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
        assert_eq!(p.components.len(), 20);
        assert_eq!(p.nets.len(), 26);
        assert_eq!(p.controls.len(), 3);
        let labels: Vec<&str> = p.controls.iter().map(|c| c.label.as_str()).collect();
        assert!(labels.contains(&"Drive"));
        assert!(labels.contains(&"Tone"));
        assert!(labels.contains(&"Level"));
        // Verify dual JRC4558 opamps + feedback clipping diodes
        let opamp_count = p
            .components
            .iter()
            .filter(|c| matches!(c.kind, dsl::ComponentKind::OpAmp(dsl::OpAmpType::Jrc4558)))
            .count();
        assert_eq!(opamp_count, 2, "TS808 uses dual JRC4558D");
        assert!(p
            .components
            .iter()
            .any(|c| c.kind == dsl::ComponentKind::Diode(dsl::DiodeType::Silicon)));
    }

    #[test]
    fn pedal_fuzz_face() {
        let p = parse_example("fuzz_face.pedal");
        assert_eq!(p.name, "Fuzz Face");
        assert_eq!(p.components.len(), 16);
        assert_eq!(p.nets.len(), 22);
        assert_eq!(p.controls.len(), 2);
        let labels: Vec<&str> = p.controls.iter().map(|c| c.label.as_str()).collect();
        assert!(labels.contains(&"Fuzz"));
        assert!(labels.contains(&"Volume"));
        // Verify PNP transistors (germanium)
        let pnp_count = p
            .components
            .iter()
            .filter(|c| c.kind == dsl::ComponentKind::Pnp)
            .count();
        assert_eq!(pnp_count, 2, "Fuzz Face uses 2 PNP transistors");
    }

    #[test]
    fn pedal_big_muff() {
        let p = parse_example("big_muff.pedal");
        assert_eq!(p.name, "Big Muff");
        assert_eq!(p.components.len(), 37);
        assert_eq!(p.nets.len(), 49);
        assert_eq!(p.controls.len(), 3);
        let labels: Vec<&str> = p.controls.iter().map(|c| c.label.as_str()).collect();
        assert!(labels.contains(&"Sustain"));
        assert!(labels.contains(&"Tone"));
        assert!(labels.contains(&"Volume"));
        // Verify 4 NPN gain stages + 2 clipping diode pairs
        let npn_count = p
            .components
            .iter()
            .filter(|c| c.kind == dsl::ComponentKind::Npn)
            .count();
        assert_eq!(npn_count, 4, "Big Muff uses 4 NPN transistor stages");
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
            .any(|c| matches!(c.kind, dsl::ComponentKind::OpAmp(_))));
    }

    #[test]
    fn pedal_proco_rat() {
        let p = parse_example("proco_rat.pedal");
        assert_eq!(p.name, "ProCo RAT");
        assert_eq!(p.components.len(), 22);
        assert_eq!(p.nets.len(), 28);
        assert_eq!(p.controls.len(), 3);
        let labels: Vec<&str> = p.controls.iter().map(|c| c.label.as_str()).collect();
        assert!(labels.contains(&"Distortion"));
        assert!(labels.contains(&"Filter"));
        assert!(labels.contains(&"Volume"));
        // Verify LM308 opamp + hard clipping diodes
        assert!(p
            .components
            .iter()
            .any(|c| matches!(c.kind, dsl::ComponentKind::OpAmp(dsl::OpAmpType::Lm308))));
        let diode_count = p
            .components
            .iter()
            .filter(|c| c.kind == dsl::ComponentKind::Diode(dsl::DiodeType::Silicon))
            .count();
        assert_eq!(diode_count, 2, "RAT uses 2 silicon clipping diodes");
    }

    #[test]
    fn pedal_blues_driver() {
        let p = parse_example("blues_driver.pedal");
        assert_eq!(p.name, "Boss Blues Driver");
        assert_eq!(p.components.len(), 32);
        assert_eq!(p.nets.len(), 39);
        assert_eq!(p.controls.len(), 3);
        let labels: Vec<&str> = p.controls.iter().map(|c| c.label.as_str()).collect();
        assert!(labels.contains(&"Gain"));
        assert!(labels.contains(&"Tone"));
        assert!(labels.contains(&"Level"));
        // Verify JFET input buffer + dual TL072 + asymmetric diodes
        assert!(p
            .components
            .iter()
            .any(|c| matches!(c.kind, dsl::ComponentKind::NJfet(_))));
        let opamp_count = p
            .components
            .iter()
            .filter(|c| matches!(c.kind, dsl::ComponentKind::OpAmp(_)))
            .count();
        assert_eq!(opamp_count, 2, "Blues Driver uses dual TL072");
        let diode_count = p
            .components
            .iter()
            .filter(|c| c.kind == dsl::ComponentKind::Diode(dsl::DiodeType::Silicon))
            .count();
        assert_eq!(diode_count, 3, "Blues Driver uses 3 asymmetric clipping diodes");
    }

    #[test]
    fn pedal_klon_centaur() {
        let p = parse_example("klon_centaur.pedal");
        assert_eq!(p.name, "Klon Centaur");
        assert_eq!(p.components.len(), 29);
        assert_eq!(p.nets.len(), 37);
        assert_eq!(p.controls.len(), 3);
        let labels: Vec<&str> = p.controls.iter().map(|c| c.label.as_str()).collect();
        assert!(labels.contains(&"Gain"));
        assert!(labels.contains(&"Treble"));
        assert!(labels.contains(&"Output"));
        // Verify 3 TL072 opamps + germanium feedback clipping diodes
        let opamp_count = p
            .components
            .iter()
            .filter(|c| matches!(c.kind, dsl::ComponentKind::OpAmp(dsl::OpAmpType::Tl072)))
            .count();
        assert_eq!(opamp_count, 3, "Klon uses 3 opamps (2 gain + 1 output buffer)");
        let ge_diode_count = p
            .components
            .iter()
            .filter(|c| c.kind == dsl::ComponentKind::Diode(dsl::DiodeType::Germanium))
            .count();
        assert_eq!(ge_diode_count, 2, "Klon uses 2 germanium clipping diodes in feedback");
    }

    #[test]
    fn pedal_fulltone_ocd() {
        let p = parse_example("fulltone_ocd.pedal");
        assert_eq!(p.name, "Fulltone OCD");
        assert_eq!(p.components.len(), 18);
        assert_eq!(p.nets.len(), 23);
        assert_eq!(p.controls.len(), 3);
        let labels: Vec<&str> = p.controls.iter().map(|c| c.label.as_str()).collect();
        assert!(labels.contains(&"Drive"));
        assert!(labels.contains(&"Tone"));
        assert!(labels.contains(&"Volume"));
        // Verify MOSFET clipping (the OCD's signature)
        let mosfet_count = p
            .components
            .iter()
            .filter(|c| matches!(c.kind, dsl::ComponentKind::Nmos(_)))
            .count();
        assert_eq!(mosfet_count, 2, "OCD uses 2 NMOS MOSFETs for clipping");
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
            "fulltone_ocd.pedal",
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
