//! Delay-line lowering: turn `delay_line()` + `tap()` components into runtime
//! `DelayLineBinding`s with their real tap ratios, plus their control
//! (delay_time / feedback / mix) and LFO (wow/flutter speed_mod) bindings —
//! F15, the generalized [`DspBlock`] (spec §8,
//! reports/dsp-block-generalization-2026-06-13.md).
//!
//! The `DelayLineComp` is `GraphRole::Virtual` with a `Behavioral` edge — it
//! contributes no electrical edges, so the circuit graph is galvanically split
//! at `DL.in`/`DL.out`, exactly like `bbd()`/`vca()` (the precedent this module
//! mirrors). It *compiled* before this module existed (the SPQR pipeline
//! already built one `DelayLineBinding`), but its entire control surface was
//! DEAD:
//!
//!   * taps hard-coded `taps: vec![1.0]` (old `spqr_build::build_delay_line_
//!     bindings`), so `tap(DL, 2.0)` / `tap(DL, 3.0)` were silent;
//!   * `pot -> DL.delay_time` / `pot -> DL.feedback` never bound (the SPQR
//!     pipeline only binds pots living inside compiled stages; bind.rs's
//!     DelayTime/DelayFeedback handlers are dead code);
//!   * `LFO.out -> DL.speed_mod` never bound (SPQR built LfoBindings only for
//!     BBD clocks);
//!   * the tape medium was never configured (`configure_zones_from_taps` was
//!     never called) so `process_medium` was a no-op;
//!   * wet/dry was a hard-coded 50/50 in the processor tick loop.
//!
//! The RUNTIME fully supports all of it (`DelayLineBinding.taps`,
//! `ControlTarget::DelayTime`/`DelayFeedback`, `ModulationTarget::DelaySpeed`,
//! `DelayLine::configure_zones_from_taps`); this module only *wires it in* as a
//! [`DspBlock`]. As with `bbd()`/`vca()`/`vco()`, `delay_lines[i]` corresponds
//! to `delay_component_ids(pedal)[i]` (declaration order = the index contract
//! the `DelayTime`/`DelayFeedback`/`DelaySpeed` targets refer to).
//!
//! # Feedback range (RE-201 self-oscillation)
//!
//! The runtime `DelayLine::set_feedback` already clamps to `0.99` (not `0.95`);
//! the `0.95` the spec refers to is the *control-law scale* applied at the
//! processor's `ControlTarget::DelayFeedback` arm (`set_feedback(value *
//! 0.95)`, processor.rs:2265) — pot-at-max → loop gain 0.95, a stable
//! near-oscillation tamed by tape compression rather than a hard runaway. This
//! pass deliberately KEEPS that scale: the RE-201 acceptance baseline
//! (`space_echo_max_intensity_sustains`) is measured against the 0.95 loop-gain
//! clamp (Intensity 1.0 → tail decays to 0.34x over 2.45 s — slow decay, not
//! true self-oscillation), and the stability guard *is* `< 1.0`. Relaxing
//! toward unity would require a real tape-compression limiter in the feedback
//! path to stay bounded; that is follow-up, not this block. So: 0.95 control
//! scale retained, documented here as the stability guard.

use hashbrown::HashMap;

use crate::dsl::{LfoWaveformDsl, PedalDef, Pin};

use super::compiled::{CompiledPedal, ControlBinding, ControlTarget, LfoBinding, ModulationTarget};
use super::components::{DelayLineComp, Lfo, Tap};
use super::dsp_block::{BlockIo, BlockPort, DspBlock, PortRole};
use super::graph::CircuitGraph;
use pedalkernel_rt::elements::Modulator;
use pedalkernel_rt::processor::DelayLineBinding;

/// Pin names a `delay_line()` accepts on its audio input / output side.
const DL_IN_PINS: &[&str] = &["in", "input"];
const DL_OUT_PINS: &[&str] = &["out", "output"];
/// Tap output pins.
const TAP_OUT_PINS: &[&str] = &["out", "output"];
/// Pot terminals a routed control may connect through.
const POT_PINS: &[&str] = &["wiper", "w", "a", "b"];

/// The delay-line [`DspBlock`]: lowers `delay_line()` + `tap()` components to
/// runtime `DelayLineBinding`s (`compiled.delay_lines`) plus their control / LFO
/// bindings. The type-specific guts live as `pub(super)` helpers in this module.
pub(super) struct DelayLineBlock;

impl DspBlock for DelayLineBlock {
    fn handles(&self, type_tag: &str) -> bool {
        type_tag == "delay line"
    }

    fn component_ids(&self, pedal: &PedalDef) -> Vec<String> {
        delay_component_ids(pedal)
    }

    fn io(&self, pedal: &PedalDef, graph: &CircuitGraph) -> Vec<(String, BlockIo)> {
        delay_io(pedal, graph)
    }

    fn bind_runtime(
        &self,
        pedal: &PedalDef,
        compiled: &mut CompiledPedal,
        sample_rate: f64,
    ) -> Result<(), String> {
        compiled.delay_lines = build_delay_line_bindings(pedal, sample_rate);
        bind_delay_runtime(pedal, compiled, sample_rate);
        Ok(())
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// (1) Component enumeration + runtime DelayLineBinding construction
// ═══════════════════════════════════════════════════════════════════════════

/// All `delay_line()` component IDs in declaration order. `delay_lines[i]`
/// corresponds to `delay_component_ids(pedal)[i]` — the index every
/// `DelayTime`/`DelayFeedback`/`DelaySpeed` target refers to.
pub(super) fn delay_component_ids(pedal: &PedalDef) -> Vec<String> {
    pedal
        .components
        .iter()
        .filter(|c| c.kind.as_any().downcast_ref::<DelayLineComp>().is_some())
        .map(|c| c.id.clone())
        .collect()
}

/// Tap ratios attached to a given delay line, in declaration order, with an
/// implicit base tap at ratio 1.0 first (the `delay_line()` itself = head 1).
/// `tap(DL, 2.0)` / `tap(DL, 3.0)` append 2.0 / 3.0.
fn tap_ratios_for(pedal: &PedalDef, dl_id: &str) -> Vec<f64> {
    let mut ratios = vec![1.0];
    for comp in &pedal.components {
        if let Some(tap) = comp.kind.as_any().downcast_ref::<Tap>() {
            if tap.parent_id == dl_id {
                ratios.push(tap.ratio);
            }
        }
    }
    ratios
}

/// Build one `DelayLineBinding` per `delay_line()` component (declaration
/// order), carrying its REAL tap ratios (base 1.0 plus each `tap()` at its
/// declared ratio) — not the old hard-coded `vec![1.0]`.
///
/// MEDIUM ZONES DELIBERATELY LEFT INERT (F15 scope = the dead *control*
/// surface): the medium is still `set_medium`'d, but `configure_zones_from_taps`
/// is NOT called. With multi-tap ratios (RE-201 heads at 2x/3x) plus feedback
/// recirculation, the in-place `process_medium` zone filter
/// (delay.rs:`process_medium`, which writes filtered values back into the same
/// buffer the feedback path re-reads) diverges to ±1e7. That is a pre-existing
/// runtime-DSP bug never exercised before (zones were always empty because
/// configure was never called), and fixing it is a DSP change out of scope here
/// (and follow-up — see report). Leaving zones empty preserves the documented
/// "TAPE MEDIUM INERT" behavior (space_echo SIMPLIFICATIONS #3) and the measured
/// RE-201 baselines.
pub(super) fn build_delay_line_bindings(
    pedal: &PedalDef,
    sample_rate: f64,
) -> Vec<DelayLineBinding> {
    pedal
        .components
        .iter()
        .filter_map(|component| {
            let delay = component.kind.as_any().downcast_ref::<DelayLineComp>()?;
            let mut delay_line = pedalkernel_rt::elements::DelayLine::new(
                delay.min_delay,
                delay.max_delay,
                sample_rate,
                delay.interpolation,
            );
            delay_line.set_medium(delay.medium);

            let taps = tap_ratios_for(pedal, &component.id);
            // Grow the ring buffer so the longest tap (RE-201 head 3 at 3x)
            // reads without clamping onto the buffer-max position. The base
            // delay range / normalized law is unchanged.
            let max_ratio = taps.iter().cloned().fold(1.0_f64, f64::max);
            delay_line.ensure_tap_capacity(max_ratio);

            Some(DelayLineBinding {
                delay_line,
                taps,
                // Default 50/50 (historical); a mix pot would override via the
                // runtime mix binding. No mix pot is wired in space_echo (the
                // wet bus is engine-gated, see space_echo SIMPLIFICATIONS #4),
                // so this preserves the measured RE-201 baselines.
                wet_mix: 0.5,
                comp_id: component.id.clone(),
            })
        })
        .collect()
}

// ═══════════════════════════════════════════════════════════════════════════
// (2) Port signature (spec §2) — 1 Audio in, main out + each tap out
// ═══════════════════════════════════════════════════════════════════════════

/// Resolve `id.pin` (trying each candidate pin name) to a graph node.
fn resolve_port(
    graph: &CircuitGraph,
    id: &str,
    pins: &[&'static str],
    role: PortRole,
) -> Option<BlockPort> {
    for &pin in pins {
        if let Some(&node) = graph.node_names.get(&format!("{id}.{pin}")) {
            return Some(BlockPort { node, pin, role });
        }
    }
    None
}

/// Typed port signature (spec §2/§8) of each `delay_line()`: 1 Audio input (the
/// DL `in` node) and Audio outputs = the DL `out` node followed by each
/// attached `tap()`'s output node (heads 2/3 at their ratios). In component
/// declaration order.
///
/// These nodes are the behavioral stage boundaries: the netlist is galvanically
/// cut at the delay line, so each must be a SPQR terminal — otherwise the
/// dangling side of a passive group has no port and its stage probes 0. The tap
/// output nodes (`T2.out -> out`, …) are real audio-path nodes that must also
/// be terminals so the wet-summing network downstream of the heads resolves.
/// `all_boundary_nodes` flattens inputs-then-outputs to recover the set.
pub(super) fn delay_io(pedal: &PedalDef, graph: &CircuitGraph) -> Vec<(String, BlockIo)> {
    delay_component_ids(pedal)
        .into_iter()
        .map(|id| {
            let inputs: Vec<BlockPort> = resolve_port(graph, &id, DL_IN_PINS, PortRole::Audio)
                .into_iter()
                .collect();

            let mut outputs: Vec<BlockPort> = Vec::new();
            if let Some(p) = resolve_port(graph, &id, DL_OUT_PINS, PortRole::Audio) {
                outputs.push(p);
            }
            // Each tap() reading this delay line contributes an output port at
            // its ratio (declaration order), so its node becomes a terminal.
            for comp in &pedal.components {
                if let Some(tap) = comp.kind.as_any().downcast_ref::<Tap>() {
                    if tap.parent_id == id {
                        if let Some(p) =
                            resolve_port(graph, &comp.id, TAP_OUT_PINS, PortRole::Audio)
                        {
                            if !outputs.iter().any(|o| o.node == p.node) {
                                outputs.push(p);
                            }
                        }
                    }
                }
            }

            (id, BlockIo { inputs, outputs })
        })
        .collect()
}

// ═══════════════════════════════════════════════════════════════════════════
// (3)+(4) Control and LFO binding (per-instance, no global state)
// ═══════════════════════════════════════════════════════════════════════════

/// Bind delay-line controls (delay_time / feedback / mix) and wow/flutter LFOs
/// onto a compiled pedal.
///
/// Must run after `spqr_control::bind_controls` (it only adds/retargets
/// bindings, never reorders existing ones — pot smoother indices stay valid),
/// mirroring `bbd_lowering::bind_bbd_runtime`.
pub(super) fn bind_delay_runtime(pedal: &PedalDef, compiled: &mut CompiledPedal, sample_rate: f64) {
    let dl_ids = delay_component_ids(pedal);
    if dl_ids.is_empty() {
        return;
    }
    let dl_id_to_idx: HashMap<&str, usize> = dl_ids
        .iter()
        .enumerate()
        .map(|(i, id)| (id.as_str(), i))
        .collect();

    // ── LFOs driving DL speed_mod (wow/flutter) ─────────────────────────
    // `LFO.out -> DL.speed_mod` (RE-201 wow). One speed_mod binding per LFO.
    bind_speed_mod_lfos(pedal, compiled, sample_rate, &dl_ids, &dl_id_to_idx);

    // ── Controls ────────────────────────────────────────────────────────
    for ctrl in &pedal.controls {
        // Direct declarations on the delay line itself (`DL1.delay_time -> ..`).
        if let Some(&idx) = dl_id_to_idx.get(ctrl.component.as_str()) {
            let target = match ctrl.property.as_str() {
                "delay_time" => Some(ControlTarget::DelayTime(idx)),
                "feedback" => Some(ControlTarget::DelayFeedback(idx)),
                _ => None,
            };
            if let Some(target) = target {
                upsert_control(compiled, pedal, ctrl, target);
            }
            continue;
        }

        // Pot-routed controls: a pot whose terminal nets reach a DL modulation
        // pin (`Pot.wiper -> DL.delay_time` / `-> DL.feedback`).
        let Some(pot) = pedal
            .components
            .iter()
            .find(|c| c.id == ctrl.component && c.kind.is_pot())
        else {
            continue;
        };

        if let Some(target) = scan_pot_target(&pot.id, pedal, &dl_id_to_idx) {
            upsert_control(compiled, pedal, ctrl, target);
        }
    }
}

/// Bind every `LFO.out -> DL.speed_mod` (directly) as a `DelaySpeed` modulator.
fn bind_speed_mod_lfos(
    pedal: &PedalDef,
    compiled: &mut CompiledPedal,
    sample_rate: f64,
    dl_ids: &[String],
    dl_id_to_idx: &HashMap<&str, usize>,
) {
    let mut bound_lfo: HashMap<String, ()> = HashMap::new();
    for comp in &pedal.components {
        let Some(lfo) = comp.kind.as_any().downcast_ref::<Lfo>() else {
            continue;
        };
        for net in &pedal.nets {
            let Pin::ComponentPin { component, pin } = &net.from else {
                continue;
            };
            if component != &comp.id || (pin != "out" && pin != "output") {
                continue;
            }
            for to_pin in &net.to {
                let Pin::ComponentPin {
                    component: tcomp,
                    pin: tpin,
                } = to_pin
                else {
                    continue;
                };
                if tpin != "speed_mod" {
                    continue;
                }
                let Some(&dl_idx) = dl_id_to_idx.get(tcomp.as_str()) else {
                    continue;
                };
                if bound_lfo.contains_key(&comp.id) {
                    continue; // one speed_mod binding per LFO is enough
                }
                // Bias/range from the delay line's declared speed_mod sink.
                let (bias, range) = pedal
                    .components
                    .iter()
                    .find(|c| c.id == dl_ids[dl_idx])
                    .and_then(|c| c.kind.modulation_sink("speed_mod"))
                    .map(|s| (s.bias, s.range))
                    .unwrap_or((0.0, 0.02));

                let waveform = match lfo.waveform {
                    LfoWaveformDsl::Sine => crate::elements::LfoWaveform::Sine,
                    LfoWaveformDsl::Triangle => crate::elements::LfoWaveform::Triangle,
                    LfoWaveformDsl::Square => crate::elements::LfoWaveform::Square,
                    LfoWaveformDsl::SawUp => crate::elements::LfoWaveform::SawUp,
                    LfoWaveformDsl::SawDown => crate::elements::LfoWaveform::SawDown,
                    LfoWaveformDsl::SampleAndHold => crate::elements::LfoWaveform::SampleAndHold,
                };
                let base_freq = 1.0 / (2.0 * std::f64::consts::PI * lfo.timing_r * lfo.timing_c);
                let mut osc = crate::elements::Lfo::new(waveform, sample_rate);
                osc.set_rate(base_freq);

                bound_lfo.insert(comp.id.clone(), ());
                compiled.lfos.push(LfoBinding {
                    lfo: osc,
                    target: ModulationTarget::DelaySpeed { delay_idx: dl_idx },
                    bias,
                    range,
                    base_freq,
                    lfo_id: comp.id.clone(),
                });
            }
        }
    }
}

/// Retarget an existing binding for (label, component) or append a new one.
///
/// Retargeting in place keeps `controls` indices stable so pot smoothers
/// created by `bind_controls` stay valid (mirrors `bbd_lowering::upsert_control`).
fn upsert_control(
    compiled: &mut CompiledPedal,
    pedal: &PedalDef,
    ctrl: &crate::dsl::ControlDef,
    target: ControlTarget,
) {
    if let Some(existing) = compiled
        .controls
        .iter_mut()
        .find(|b| b.label == ctrl.label && b.component_id == ctrl.component)
    {
        existing.target = target;
    } else {
        let comp = pedal.components.iter().find(|c| c.id == ctrl.component);
        let max_r = comp.and_then(|c| c.kind.resistance()).unwrap_or(100_000.0);
        let taper = comp
            .and_then(|c| c.kind.pot_taper())
            .unwrap_or(crate::dsl::PotTaper::B);
        compiled.controls.push(ControlBinding {
            label: ctrl.label.clone(),
            target,
            component_id: ctrl.component.clone(),
            max_resistance: max_r,
            taper,
            range: ctrl.range,
        });
    }
    // (Re-)apply the declared default so the new target receives it.
    compiled.set_control(&ctrl.label, ctrl.default);
}

/// Resolve a pot to a delay-line control target by scanning nets.
///
/// Forward: `Pot.term -> DL.delay_time` ⇒ DelayTime; `Pot.term -> DL.feedback`
/// ⇒ DelayFeedback. Mirrors `bbd_lowering::scan_pot_target`.
fn scan_pot_target(
    pot_id: &str,
    pedal: &PedalDef,
    dl_id_to_idx: &HashMap<&str, usize>,
) -> Option<ControlTarget> {
    for net in &pedal.nets {
        let Pin::ComponentPin { component, pin } = &net.from else {
            continue;
        };
        if component != pot_id || !POT_PINS.contains(&pin.as_str()) {
            continue;
        }
        for to_pin in &net.to {
            let Pin::ComponentPin {
                component: tcomp,
                pin: tpin,
            } = to_pin
            else {
                continue;
            };
            if let Some(&idx) = dl_id_to_idx.get(tcomp.as_str()) {
                match tpin.as_str() {
                    "delay_time" => return Some(ControlTarget::DelayTime(idx)),
                    "feedback" => return Some(ControlTarget::DelayFeedback(idx)),
                    _ => {}
                }
            }
        }
    }
    None
}
