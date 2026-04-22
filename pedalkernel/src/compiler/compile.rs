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

pub struct CompileOptions {
    pub oversampling: OversamplingFactor,
    pub tolerance: ToleranceEngine,
    pub thermal: bool,
    /// When true, collapse ALL nonlinear elements into a single MultiNlStage
    /// instead of planning them as individual stages. Used for sidechain
    /// sub-circuits where the entire NL network should be solved as one
    /// multi-junction system (shared MNA + scattering matrix).
    pub collapse_nl: bool,
}

impl Default for CompileOptions {
    fn default() -> Self {
        Self {
            oversampling: OversamplingFactor::X4,
            tolerance: ToleranceEngine::ideal(),
            thermal: false,
            collapse_nl: false,
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

// ──────────��────────────────────────────────���─────────────────────────────────
// Subcircuit equipment compiler
// ───────────��─────────���───────────────────────────────────────────────────────

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
        slew_limiters: Vec::new(),
        bbds: Vec::new(),
        delay_lines: Vec::new(),
        vcos: Vec::new(),
        vcas: Vec::new(),
        thermal: None,
        tolerance_seed: 0,
        oversampling: crate::oversampling::OversamplingFactor::X1,
        opamp_stages: Vec::new(),
        power_supply: None,
        #[cfg(debug_assertions)]
        debug_stats: None,
        metrics_accumulator: None,
        metrics_buffer: None,
        input_loading: None,
        output_loading: None,
        output_dc_block: None,
        sidechains: Vec::new(),
        subcircuit_processors: processors,
        subcircuit_routing: routing,
        subcircuit_output_idx: Some(output_idx),
        subcircuit_outputs: vec![0.0; n],
        pot_smoothers: Vec::new(),
        pot_mirrors: std::collections::HashMap::new(),
        base_grid_bias: 0.0,
        multi_nl_recompute_counter: 0,
        node_signals: Vec::new(),
        bbd_wet_mix: 0.5,
        bbd_mix_pot_id: None,
        triggers: Vec::new(),
        original_passive_values: std::collections::HashMap::new(),
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
}
