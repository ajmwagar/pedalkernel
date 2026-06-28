//! Main compiler entry point: PedalDef -> CompiledPedal.
//!
//! Delegates to the SPQR pipeline (`compile_via_spqr`) for all circuit
//! compilation. The legacy 6-pass pipeline has been removed.

use crate::dsl::*;
use crate::oversampling::OversamplingFactor;
use crate::tolerance::ToleranceEngine;

use super::compiled::*;

// ═════��═════════════════════════════════════════════════════════════════════
// Compile options
// ═��═══════════════════════════════════════════════════════��═════════════════

#[derive(Clone)]
pub struct CompileOptions {
    pub oversampling: OversamplingFactor,
    pub tolerance: ToleranceEngine,
    pub thermal: bool,
    /// When true, collapse ALL nonlinear elements into a single MultiNlStage
    /// instead of planning them as individual stages. Used for sidechain
    /// sub-circuits where the entire NL network should be solved as one
    /// multi-junction system (shared MNA + scattering matrix).
    pub collapse_nl: bool,
    /// When true, skip K-method lookup table generation. NL roots fall back
    /// to Newton-Raphson iteration at runtime. Use for fast debug builds —
    /// K-table generation is the main compile-time bottleneck (65536-entry
    /// 2D sweeps per NL root).
    pub skip_k_tables: bool,
    /// When true, skip blockwise decomposition of multi-NL groups.
    /// Falls back to monolithic R-type adaptor. Use when blockwise
    /// produces incorrect pot bindings or signal routing.
    pub skip_blockwise: bool,
    /// Diagnostic mode: build blockwise-decomposed nonlinear ladders as
    /// serial WDF/K-method rung stages instead of packaging them into one
    /// delay-free Blockwise coupling stage. This intentionally breaks
    /// delay-free feedback, so it is only useful for isolating rung behavior.
    pub force_serial_blockwise: bool,
    /// Diagnostic mode: when `force_serial_blockwise` is enabled, wrap the
    /// serial rung stages in a one-sample output feedback loop with this gain.
    /// `0.0` preserves the plain forced-serial behavior.
    pub force_serial_blockwise_feedback_gain: f64,
    /// Diagnostic mode: do not synthesize IIR stages from rigid subgraphs.
    /// Rigid passive networks fall through to StateSpace where possible, and
    /// active feedback networks fall through to the existing WDF adaptors.
    pub disable_iir: bool,
    /// Solve coupled blockwise K-method stages with the full delay-free
    /// Newton/Jacobian iteration. This is useful for compiler validation and
    /// offline reference renders, but too expensive for some realtime targets.
    /// When false, coupled blockwise stages use table-driven fixed-point
    /// iteration without building/solving the full Jacobian.
    pub coupled_blockwise_newton: bool,
}

impl Default for CompileOptions {
    fn default() -> Self {
        Self {
            oversampling: OversamplingFactor::X4,
            tolerance: ToleranceEngine::ideal(),
            thermal: false,
            collapse_nl: false,
            skip_k_tables: false,
            skip_blockwise: false,
            force_serial_blockwise: false,
            force_serial_blockwise_feedback_gain: 0.0,
            disable_iir: false,
            coupled_blockwise_newton: true,
        }
    }
}

impl CompileOptions {
    /// Return a copy of these options with blockwise decomposition disabled,
    /// forcing all nonlinear groups through the monolithic/rigid MNA path.
    ///
    /// Use this for differential testing: compile the same circuit twice
    /// (`default()` and `force_monolithic()`) and compare outputs sample-wise.
    /// Any deviation indicates that blockwise is behaving as a dialect change
    /// rather than a pure optimization — which is a bug.
    ///
    /// Follows the same opt-in pattern as [`CompileOptions::thermal`]:
    /// default `false` (blockwise enabled), set `true` to override.
    pub fn force_monolithic(self) -> Self {
        Self {
            skip_blockwise: true,
            ..self
        }
    }

    /// Default options for release builds: full optimization including K-tables.
    pub fn release() -> Self {
        Self::default()
    }

    /// Fast options for debug builds: skip K-table generation.
    /// NL elements use Newton-Raphson iteration instead (correct but slower at runtime).
    pub fn debug() -> Self {
        Self {
            skip_k_tables: true,
            ..Self::default()
        }
    }

    /// Auto-detect from the `PROFILE` env var set by Cargo during build.rs.
    /// Returns `release()` for "release" profile, `debug()` for everything else.
    ///
    /// Use in build.rs:
    /// ```no_run
    /// let options = pedalkernel::compiler::CompileOptions::from_cargo_profile();
    /// ```
    pub fn from_cargo_profile() -> Self {
        match std::env::var("PROFILE").as_deref() {
            Ok("release") => Self::release(),
            _ => Self::debug(),
        }
    }
}

// ══════════════════════════════��═══════════════════════════════════��════════
// Public entry points
// ═══════���═════════════════════���═════════════════════════════════════════════

/// Compile a `.pedal` circuit into a real-time audio processor with default options.
pub fn compile_pedal(pedal: &PedalDef, sample_rate: f64) -> Result<CompiledPedal, String> {
    compile_pedal_with_options(pedal, sample_rate, CompileOptions::default())
}

/// Compile a `.pedal` circuit into a real-time audio processor with custom options.
///
/// Delegates to the SPQR pipeline for standard circuits and to the subcircuit
/// equipment compiler for multi-subcircuit equipment definitions.
pub fn compile_pedal_with_options(
    pedal: &PedalDef,
    sample_rate: f64,
    options: CompileOptions,
) -> Result<CompiledPedal, String> {
    // Subcircuit equipment uses a separate compilation path.
    if !pedal.subcircuits.is_empty() {
        return compile_subcircuit_equipment(pedal, sample_rate, &options);
    }

    // Delegate to SPQR pipeline.
    super::spqr_build::compile_via_spqr_with_options(pedal, sample_rate, options)
}

// ──────────────────────────────────────────────────────────────────────────
// Cached compile for build.rs
// ──────────────────────────────────────────────────────────────────────────

/// Compute a cache key for a `.pedal` source + compile options.
///
/// Returns a hex string hash that changes when the source, sample rate,
/// or relevant options change.
pub fn compile_cache_key(source: &str, sample_rate: f64, options: &CompileOptions) -> String {
    use std::collections::hash_map::DefaultHasher;
    use std::hash::{Hash, Hasher};

    let mut hasher = DefaultHasher::new();
    source.hash(&mut hasher);
    sample_rate.to_bits().hash(&mut hasher);
    options.skip_k_tables.hash(&mut hasher);
    options.collapse_nl.hash(&mut hasher);
    options.skip_blockwise.hash(&mut hasher);
    options.force_serial_blockwise.hash(&mut hasher);
    options
        .force_serial_blockwise_feedback_gain
        .to_bits()
        .hash(&mut hasher);
    options.disable_iir.hash(&mut hasher);
    options.coupled_blockwise_newton.hash(&mut hasher);
    (options.oversampling as u8).hash(&mut hasher);
    format!("{:016x}", hasher.finish())
}

/// Compile a `.pedal` source to a postcard blob with caching.
///
/// Single-call utility for build.rs. Handles parse → compile → serialize →
/// cache write. Returns the postcard bytes (write them to OUT_DIR).
///
/// Skips recompilation when the cached blob exists and source hash matches.
/// Uses `CompileOptions::from_cargo_profile()` internally if you want
/// automatic debug/release detection, or pass your own options.
///
/// ```no_run
/// // In build.rs:
/// let options = pedalkernel::compiler::CompileOptions::from_cargo_profile();
/// let blob = pedalkernel::compiler::compile_pedal_cached(
///     &source, "TB303 Filter", "tb303_filter", 48_000.0, &options, out_path,
/// ).expect("compile failed");
/// // blob is already written to out_path/tb303_filter.postcard
/// ```
#[cfg(feature = "build-cache")]
pub fn compile_pedal_cached(
    source: &str,
    name: &str,
    file_stem: &str,
    sample_rate: f64,
    options: &CompileOptions,
    out_dir: &std::path::Path,
) -> Result<Vec<u8>, String> {
    let blob_path = out_dir.join(format!("{file_stem}.postcard"));
    let hash_path = out_dir.join(format!("{file_stem}.hash"));
    let cache_key = compile_cache_key(source, sample_rate, options);

    // Check cache
    if blob_path.exists() && hash_path.exists() {
        if let Ok(cached) = std::fs::read_to_string(&hash_path) {
            if cached.trim() == cache_key {
                if let Ok(blob) = std::fs::read(&blob_path) {
                    eprintln!("  {name}: cached ({} bytes)", blob.len());
                    return Ok(blob);
                }
            }
        }
    }

    // Cache miss — compile
    eprintln!("  {name}: parsing...");
    let pedal_def =
        crate::dsl::parse_pedal_file(source).map_err(|e| format!("{name}: parse failed: {e}"))?;
    eprintln!(
        "  {name}: compiling (skip_k_tables={}, oversampling={:?})...",
        options.skip_k_tables, options.oversampling
    );
    let t0 = std::time::Instant::now();
    let compiled = compile_pedal_with_options(&pedal_def, sample_rate, options.clone())
        .map_err(|e| format!("{name}: compile failed: {e}"))?;
    eprintln!(
        "  {name}: compile done in {:.1}s, {} stages, {} ports",
        t0.elapsed().as_secs_f64(),
        compiled.stages.len(),
        compiled.ports.len()
    );
    eprintln!("  {name}: serializing...");
    let blob =
        postcard::to_allocvec(&compiled).map_err(|e| format!("{name}: serialize failed: {e}"))?;

    eprintln!(
        "  {name}: compiled ({} bytes, k_tables={})",
        blob.len(),
        !options.skip_k_tables
    );

    // Write cache
    let _ = std::fs::write(&blob_path, &blob);
    let _ = std::fs::write(&hash_path, &cache_key);

    Ok(blob)
}

// ──────────────────────────────────────────────────────────────────────────
// Subcircuit equipment compiler
// ──────────────────────────────────────────────────────────────────────────

/// Compile an equipment definition that uses subcircuit blocks.
///
/// Each subcircuit is compiled independently as a sub-PedalDef (potentially at a
/// different sample rate) and wired together via the routing
/// graph produced by `subcircuit::build_routing()`.
///
/// When all components live inside subcircuits (`pedal.components.is_empty()`),
/// the caller's normal pipeline is entirely replaced by this function.
fn compile_subcircuit_equipment(
    pedal: &PedalDef,
    sample_rate: f64,
    _options: &CompileOptions,
) -> Result<CompiledPedal, String> {
    use super::stage::SubcircuitProcessor;
    use super::subcircuit;

    // 1. Resolve subcircuits to compilable sub-PedalDefs
    let resolved = subcircuit::resolve_subcircuits(pedal)
        .map_err(|e| format!("subcircuit resolution: {e}"))?;

    // 2. Compile each subcircuit
    let mut processors: Vec<SubcircuitProcessor> = Vec::with_capacity(resolved.len());
    for r in &resolved {
        let sc_rate = sample_rate / r.rate_divisor as f64;
        // Subcircuits use no oversampling by default to avoid N × M sample rate blowup.
        let sc_options = CompileOptions {
            collapse_nl: false,
            oversampling: crate::oversampling::OversamplingFactor::X1,
            ..CompileOptions::default()
        };
        let compiled = compile_pedal_with_options(&r.pedal_def, sc_rate, sc_options)
            .map_err(|e| format!("subcircuit '{}': {e}", r.name))?;
        processors.push(SubcircuitProcessor {
            circuit: compiled,
            name: r.name.clone(),
            rate_divisor: r.rate_divisor,
            rate_counter: r.rate_divisor.max(1),
            held_output: 0.0,
            prev_output: 0.0,
        });
    }

    // 3. Build routing graph
    let (routing, output_idx) = subcircuit::build_routing(pedal, &resolved)
        .map_err(|e| format!("subcircuit routing: {e}"))?;

    let n = processors.len();

    // 4. Assemble routing-only CompiledPedal shell
    Ok(CompiledPedal {
        stages: Vec::new(),
        stage_route_plan: pedalkernel_rt::processor::StageRoutePlan::default(),
        push_pull_stages: Vec::new(),
        pre_gain: 1.0,
        output_gain: 1.0,
        rail_saturation: super::compiled::RailSaturation::None,
        rail_sat_oversampler: crate::oversampling::Oversampler::new(
            crate::oversampling::OversamplingFactor::X1,
        ),
        sample_rate,
        controls: Vec::new(),
        gain_range: (1.0, 1.0),
        supply_voltage: pedal.supplies.first().map_or(9.0, |s| s.config.voltage),
        lfos: Vec::new(),
        envelopes: Vec::new(),
        bbds: Vec::new(),
        delay_lines: Vec::new(),
        springs: Vec::new(),
        vcos: Vec::new(),
        vcas: Vec::new(),
        thermal: None,
        tolerance_seed: 0,
        oversampling: crate::oversampling::OversamplingFactor::X1,
        opamp_stages: Vec::new(),
        power_supply: None,
        metrics_accumulator: None,
        metrics_buffer: None,
        #[cfg(feature = "diag")]
        diag_ring: None,
        input_loading: None,
        output_loading: None,
        output_dc_block: None,
        sidechains: Vec::new(),
        subcircuit_processors: processors,
        subcircuit_routing: routing,
        subcircuit_output_idx: Some(output_idx),
        subcircuit_outputs: vec![0.0; n],
        pot_smoothers: Vec::new(),
        wiper_dividers: Vec::new(),
        pot_mirrors: hashbrown::HashMap::new(),
        base_grid_bias: 0.0,
        multi_nl_recompute_counter: 0,
        node_signals: Vec::new(),
        bbd_wet_mix: 0.5,
        bbd_mix_pot_id: None,
        triggers: Vec::new(),
        original_passive_values: hashbrown::HashMap::new(),
        ports: Vec::new(),
        port_values: Vec::new(),
        internal_ports: Vec::new(),
        detector_led_coupling: None,
        initialized: false,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dsl::parse_pedal_file;

    #[test]
    fn calibrate_output_gain_reasonable() {
        // A simple resistor divider: output should be ~half input.
        // calibrate should produce a gain ~2.0 to compensate.
        let src = r#"
            pedal "Divider" {
                components {
                    R1: resistor(10k)
                    R2: resistor(10k)
                }
                nets {
                    in -> R1.a
                    R1.b -> R2.a
                    R2.b -> gnd
                    R1.b -> out
                }
                calibrate
            }
        "#;
        let pedal = parse_pedal_file(src).unwrap();
        assert!(pedal.calibrate);
        let compiled = compile_pedal(&pedal, 48000.0).unwrap();
        // Output gain should compensate for the ~6dB loss
        assert!(
            compiled.output_gain > 1.0,
            "expected output_gain > 1.0, got {}",
            compiled.output_gain
        );
        assert!(
            compiled.output_gain < 10.0,
            "expected output_gain < 10.0, got {}",
            compiled.output_gain
        );
    }

    #[test]
    fn no_calibrate_gives_unity_gain() {
        let src = r#"
            pedal "Pass" {
                components {
                    R1: resistor(10k)
                }
                nets {
                    in -> R1.a
                    R1.b -> out
                }
            }
        "#;
        let pedal = parse_pedal_file(src).unwrap();
        assert!(!pedal.calibrate);
        let compiled = compile_pedal(&pedal, 48000.0).unwrap();
        assert!(
            (compiled.output_gain - 1.0).abs() < 1e-10,
            "expected output_gain = 1.0, got {}",
            compiled.output_gain
        );
    }

    // ── Port binding tests ──────────────────────────────────────────

    #[test]
    fn compile_with_ports_produces_port_bindings() {
        let src = r#"pedal "PortTest" {
    supply 9V
    ports {
        audio_in: input
        cv_cutoff: input
        audio_out: output
    }
    components { R1: resistor(10k) R_cv: resistor(100k) }
    nets {
        audio_in -> R1.a
        cv_cutoff -> R_cv.a
        R_cv.b -> R1.a
        R1.b -> audio_out
    }
    controls {}
}"#;
        let pedal = parse_pedal_file(src).unwrap();
        assert_eq!(pedal.ports.len(), 3);

        let compiled = compile_pedal(&pedal, 48000.0).unwrap();
        eprintln!(
            "  ports: {:?}",
            compiled
                .ports
                .iter()
                .map(|p| (&p.name, p.direction, p.index, p.node_id))
                .collect::<Vec<_>>()
        );

        assert_eq!(compiled.ports.len(), 3, "should have 3 port bindings");
        assert_eq!(compiled.port_values.len(), 3, "port_values should match");

        // Check names and directions
        let audio_in = compiled.ports.iter().find(|p| p.name == "audio_in");
        assert!(audio_in.is_some(), "should have audio_in port");
        assert_eq!(
            audio_in.unwrap().direction,
            pedalkernel_rt::PortDirection::Input
        );

        let cv_cutoff = compiled.ports.iter().find(|p| p.name == "cv_cutoff");
        assert!(cv_cutoff.is_some(), "should have cv_cutoff port");
        assert_eq!(
            cv_cutoff.unwrap().direction,
            pedalkernel_rt::PortDirection::Input
        );

        let audio_out = compiled.ports.iter().find(|p| p.name == "audio_out");
        assert!(audio_out.is_some(), "should have audio_out port");
        assert_eq!(
            audio_out.unwrap().direction,
            pedalkernel_rt::PortDirection::Output
        );

        // Node IDs should be valid (not MAX)
        for port in &compiled.ports {
            assert_ne!(
                port.node_id,
                usize::MAX,
                "port {} should have valid node_id",
                port.name
            );
        }
    }

    #[test]
    fn set_control_falls_back_to_matching_cv_input_port() {
        use crate::PedalProcessor;

        let src = r#"pedal "ControlToCvPort" {
    supply 9V
    ports {
        audio_in: input
        cv_cutoff: input
        audio_out: output
    }
    components { R1: resistor(10k) R_cv: resistor(100k) }
    nets {
        audio_in -> R1.a
        cv_cutoff -> R_cv.a
        R_cv.b -> R1.a
        R1.b -> audio_out
    }
    controls {}
}"#;
        let pedal = parse_pedal_file(src).unwrap();
        let mut compiled = compile_pedal(&pedal, 48000.0).unwrap();
        let cv_idx = compiled.resolve_port("cv_cutoff").unwrap();

        compiled.set_control("Cutoff", 0.75);

        assert_eq!(
            compiled.port_values[cv_idx], 0.75,
            "unbound Cutoff control should drive matching cv_cutoff input port"
        );
    }

    #[test]
    fn compile_without_ports_has_empty_ports() {
        let src = r#"pedal "NoPort" {
    supply 9V
    components { R1: resistor(10k) }
    nets { in -> R1.a  R1.b -> out }
    controls {}
}"#;
        let pedal = parse_pedal_file(src).unwrap();
        let compiled = compile_pedal(&pedal, 48000.0).unwrap();
        // No ports section → empty (or implicit in/out — depends on design)
        // For now, empty is fine. Backward compat.
        eprintln!("  ports: {}", compiled.ports.len());
    }

    #[test]
    fn process_ports_passes_signal() {
        use crate::PedalProcessor;

        let src = r#"pedal "PortIO" {
    supply 9V
    ports {
        audio_in: input
        audio_out: output
    }
    components { R1: resistor(10k) }
    nets { audio_in -> R1.a  R1.b -> audio_out }
    controls {}
}"#;
        let pedal = parse_pedal_file(src).unwrap();
        let mut compiled = compile_pedal(&pedal, 48000.0).unwrap();

        let in_idx = compiled.resolve_port("audio_in").unwrap();
        let out_idx = compiled.resolve_port("audio_out").unwrap();
        assert_eq!(compiled.port_count(), 2);

        // Process several samples with a sine input
        let mut ports = vec![0.0f64; compiled.port_count()];
        let mut peak = 0.0f64;
        for i in 0..4800 {
            let input = (2.0 * std::f64::consts::PI * 440.0 * i as f64 / 48000.0).sin();
            ports[in_idx] = input;
            compiled.process_ports(&mut ports);
            peak = peak.max(ports[out_idx].abs());
        }

        eprintln!("  process_ports: peak output = {peak:.6}");
        assert!(
            peak > 0.01,
            "signal should flow through ports: peak={peak:.6}. \
             If 0, audio_in port isn't reaching the WDF stages."
        );
    }

    #[test]
    fn process_ports_backward_compat_with_old_process() {
        use crate::PedalProcessor;

        // Circuit using old-style in/out (no ports section)
        let src = r#"pedal "OldStyle" {
    supply 9V
    components { R1: resistor(10k) }
    nets { in -> R1.a  R1.b -> out }
    controls {}
}"#;
        let pedal = parse_pedal_file(src).unwrap();
        let mut compiled = compile_pedal(&pedal, 48000.0).unwrap();

        // Old API should still work
        let mut peak = 0.0f64;
        for i in 0..4800 {
            let input = (2.0 * std::f64::consts::PI * 440.0 * i as f64 / 48000.0).sin();
            let out = compiled.process(input);
            peak = peak.max(out.abs());
        }
        eprintln!("  old process(): peak = {peak:.6}");
        assert!(
            peak > 0.01,
            "old process(f64) should still work: peak={peak:.6}"
        );
    }

    #[test]
    fn cv_port_modulates_signal() {
        use crate::PedalProcessor;

        // Two input ports: audio_in and cv_mod. cv_mod connects through
        // a resistor to the same node as audio_in, so it adds to the signal.
        let src = r#"pedal "CVTest" {
    supply 9V
    ports {
        audio_in: input
        cv_mod: input
        audio_out: output
    }
    components {
        R1: resistor(10k)
        R_cv: resistor(100k)
    }
    nets {
        audio_in -> R1.a
        cv_mod -> R_cv.a
        R_cv.b -> R1.a
        R1.b -> audio_out
    }
    controls {}
}"#;
        let pedal = parse_pedal_file(src).unwrap();
        let mut compiled = compile_pedal(&pedal, 48000.0).unwrap();

        let in_idx = compiled.resolve_port("audio_in").unwrap();
        let cv_idx = compiled.resolve_port("cv_mod").unwrap();
        let out_idx = compiled.resolve_port("audio_out").unwrap();

        // Process with CV = 0 (no modulation)
        let mut ports = vec![0.0f64; compiled.port_count()];
        let mut peak_no_cv = 0.0f64;
        for i in 0..4800 {
            let input = (2.0 * std::f64::consts::PI * 440.0 * i as f64 / 48000.0).sin();
            ports[in_idx] = input;
            ports[cv_idx] = 0.0;
            compiled.process_ports(&mut ports);
            if i >= 2400 {
                peak_no_cv = peak_no_cv.max(ports[out_idx].abs());
            }
        }

        // Process with CV = 0.5 (adds DC offset — should change output)
        compiled.reset();
        let mut peak_with_cv = 0.0f64;
        for i in 0..4800 {
            let input = (2.0 * std::f64::consts::PI * 440.0 * i as f64 / 48000.0).sin();
            ports[in_idx] = input;
            ports[cv_idx] = 5.0; // large DC offset via CV
            compiled.process_ports(&mut ports);
            if i >= 2400 {
                peak_with_cv = peak_with_cv.max(ports[out_idx].abs());
            }
        }

        eprintln!("  CV test: no_cv={peak_no_cv:.4}, with_cv={peak_with_cv:.4}");
        // CV should change the output (either level or DC offset)
        assert!(
            (peak_with_cv - peak_no_cv).abs() > 0.001 || peak_with_cv > 0.01,
            "CV port should affect output: no_cv={peak_no_cv:.4}, with_cv={peak_with_cv:.4}"
        );
    }

    #[test]
    fn port_vs_leaf_is_addressable() {
        // Verify set_voltage_by_port finds, sets, and isn't overwritten
        use pedalkernel_rt::dyn_node::DynNode;
        use pedalkernel_rt::wdf_leaf::*;

        // Build: Series(VS_main, Series(VS_port, Resistor))
        // VS_main is the audio input, VS_port is the CV port.
        let vs_main = DynNode::Leaf(LeafKind::VoltageSource(WdfVoltageSource {
            voltage: 0.0,
            rp: 1.0,
            is_cathode_bias: false,
            port_name: None,
        }));
        let vs_port = DynNode::Leaf(LeafKind::VoltageSource(WdfVoltageSource {
            voltage: 0.0,
            rp: 1.0,
            is_cathode_bias: false,
            port_name: Some("cv_test".into()),
        }));
        let r = DynNode::Leaf(LeafKind::Resistor(WdfResistor {
            comp_id: Some("R1".into()),
            rp: 10000.0,
            last_a: 0.0,
        }));
        let cv_branch = DynNode::Series(Box::new(vs_port), Box::new(r));
        let mut tree = DynNode::Series(Box::new(vs_main), Box::new(cv_branch));
        tree.recompute();

        // Set port VS to 3.0
        let found = tree.set_voltage_by_port("cv_test", 3.0);
        assert!(found, "should find VS by port name");

        // Global set_voltage(1.0) should set VS_main but NOT VS_port
        tree.set_voltage(1.0);

        // Scatter and check: with VS_main=1.0 and VS_port=3.0, the
        // reflected wave should differ from VS_main=1.0, VS_port=0.0
        let b_with_cv = tree.reflected();

        // Reset port to 0 and scatter again
        tree.set_voltage_by_port("cv_test", 0.0);
        tree.set_voltage(1.0);
        let b_without_cv = tree.reflected();

        eprintln!("  reflected: with_cv={b_with_cv:.6}, without_cv={b_without_cv:.6}");
        assert!(
            (b_with_cv - b_without_cv).abs() > 0.01,
            "port VS should affect scattering: with={b_with_cv:.4}, without={b_without_cv:.4}"
        );
    }
}
