//! Compile-time operating-envelope estimation.
//!
//! A small, *device-agnostic* estimator that answers "how hard can a driver
//! push a port, under worst-case (supply-limited) drive?". It is deliberately
//! kept free of transformer (or tube, or diode) specifics so a future
//! nonlinearity gate for any of those can reuse it.
//!
//! The only consumer today is the transformer core-excitation estimator
//! ([`transformer_core_excitation`]), which decides whether a magnetic core is
//! driven out of its linear region and therefore needs the (expensive)
//! Jiles-Atherton root instead of the cheap linear-inductor tangent. The gate
//! wiring + predicate call that consumes [`CoreExcitation`] lands in a later
//! task; this module produces the raw field-excursion *estimate* only.
//!
//! ## Honest-uncertainty contract
//!
//! When the driver Thévenin source / available swing genuinely can't be read
//! off the graph, the estimators return a *conservatively large* excursion so
//! the eventual gate prefers the physical (nonlinear) model. We fail toward
//! physics, never silently toward the cheap tangent.

use super::graph::{CircuitGraph, NodeId};
use crate::dsl::TransformerConfig;

/// Conservative lowest passband frequency, in Hz, used to size the peak
/// magnetizing current `I_mag = V_peak / (2·π·f_low·Lm)`.
///
/// Magnetizing current — and therefore core field H — is largest at the low
/// end of the passband (the magnetizing reactance `2·π·f·Lm` shrinks as `f`
/// falls). 20 Hz is the bottom of audible range and a standard transformer
/// low-frequency power-handling reference; using it makes `h_ac_peak` a
/// worst-case (largest) estimate over the audio band rather than a nominal
/// mid-band one.
pub(crate) const F_LOW_HZ: f64 = 20.0;

/// Fallback magnetic path length (metres) when core geometry is absent.
///
/// `le_eff = path_len + gap`. When a transformer carries no geometry we fall
/// back to the silicon-steel demo core's path length
/// ([`JaCoreModel::si_steel_demo`] uses `path_len = 0.10 m`), which is a
/// representative small audio-transformer core. Documented so the estimate is
/// reproducible rather than magic.
pub(crate) const DEFAULT_PATH_LEN_M: f64 = 0.10;

/// Fallback magnetizing inductance (H) when neither an explicit `Lm` nor a
/// linear primary inductance is available. Matches the
/// `SpiceTransformerModel` library default (`magnetizing_inductance = 0.99`).
pub(crate) const DEFAULT_LM_H: f64 = 0.99;

/// A conservatively-large field excursion (A/m) returned when the worst-case
/// drive across the primary cannot be determined from the graph.
///
/// This is comfortably above silicon-steel knee scales (the Jiles-Atherton
/// shape parameter `a` is order `1e3 A/m`), so a gate comparing `h_peak`
/// against the knee will classify the core NONLINEAR — the safe default.
pub(crate) const UNKNOWN_DRIVE_H_PEAK: f64 = 1.0e6;

/// Peak excursion an electrical port can be driven to under worst-case drive.
///
/// This captures a Thévenin driver (source resistance + maximum available
/// open-circuit swing, taken from supply rails / bias — *not* a nominal signal
/// level) feeding a port load. It is intentionally primitive (volts, ohms,
/// henries, hertz) so it is reusable across transformer / tube / diode gates.
///
/// A future tube or diode gate would build one of these from its own driver
/// (e.g. a plate-load Thévenin for a triode, or the supply-limited swing ahead
/// of a clipping diode pair) and read [`OperatingEnvelope::peak_port_voltage`]
/// or [`OperatingEnvelope::peak_reactive_current`] to size its own excursion
/// — without this module knowing anything about tubes or diodes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct OperatingEnvelope {
    /// Driver Thévenin source resistance (Ω). `0` for an ideal source.
    pub source_resistance: f64,
    /// Maximum open-circuit swing the driver can impose (V, peak), derived
    /// from supply rails / bias headroom — the worst case, not nominal level.
    pub available_swing_peak: f64,
    /// Port load resistance (Ω) seen by the driver. `f64::INFINITY` for an
    /// open / purely reactive load (no resistive divider attenuation).
    pub load_resistance: f64,
}

impl OperatingEnvelope {
    /// Construct an envelope from primitive driver/load quantities.
    pub(crate) fn new(
        source_resistance: f64,
        available_swing_peak: f64,
        load_resistance: f64,
    ) -> Self {
        Self {
            source_resistance: source_resistance.max(0.0),
            available_swing_peak: available_swing_peak.abs(),
            load_resistance: if load_resistance > 0.0 {
                load_resistance
            } else {
                f64::INFINITY
            },
        }
    }

    /// Peak voltage actually delivered across the port, after the resistive
    /// divider formed by the source resistance and the load.
    ///
    /// With an infinite (open / reactive) load this is just the available
    /// swing; with a finite resistive load the source resistance attenuates it.
    pub(crate) fn peak_port_voltage(&self) -> f64 {
        if !self.load_resistance.is_finite() {
            return self.available_swing_peak;
        }
        let denom = self.source_resistance + self.load_resistance;
        if denom <= 0.0 {
            return self.available_swing_peak;
        }
        self.available_swing_peak * (self.load_resistance / denom)
    }

    /// Peak current through a magnetizing inductance `lm` at frequency `f_low`,
    /// driven by the peak port voltage: `I = V_peak / (2·π·f·Lm)`.
    ///
    /// Returns `None` when the inductance or frequency is non-physical, so the
    /// caller can fall back to the honest-uncertainty path.
    pub(crate) fn peak_reactive_current(&self, lm: f64, f_low: f64) -> Option<f64> {
        if !(lm.is_finite() && lm > 0.0 && f_low.is_finite() && f_low > 0.0) {
            return None;
        }
        let reactance = 2.0 * core::f64::consts::PI * f_low * lm;
        if !(reactance.is_finite() && reactance > 0.0) {
            return None;
        }
        Some(self.peak_port_voltage() / reactance)
    }
}

/// Peak magnetic field H (A/m) seen by a transformer core, split into a DC
/// standing component and an AC excursion.
///
/// `h_peak = |h_dc| + h_ac_peak` is the worst-case superposition: a gapped
/// single-ended output transformer's standing bias adds directly to the
/// audio-rate swing.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct CoreExcitation {
    /// Standing DC field from primary bias current (A/m). `0` for a cleanly
    /// AC-coupled line transformer.
    pub h_dc: f64,
    /// Peak AC field from the magnetizing current at `F_LOW_HZ` (A/m).
    pub h_ac_peak: f64,
    /// Worst-case total field `|h_dc| + h_ac_peak` (A/m).
    pub h_peak: f64,
    /// `true` when the AC excursion could not be estimated from the graph and
    /// `h_peak` was forced to [`UNKNOWN_DRIVE_H_PEAK`] (fail toward physics).
    pub drive_unknown: bool,
}

/// DC voltage at a node, consulting (in order) ground, hard supply rails, AC
/// grounds, then the solved DC-bias map. Returns `None` for a node with no
/// known DC potential.
///
/// Relocated here from `spqr_build.rs` so the bias-inference primitives are
/// shared by the operating-envelope estimators.
pub(crate) fn node_dc_voltage(
    node: NodeId,
    bias_node_voltages: &std::collections::BTreeMap<NodeId, f64>,
    graph: &CircuitGraph,
) -> Option<f64> {
    if node == graph.gnd_node {
        return Some(0.0);
    }
    if let Some(&v) = graph.supply_voltages.get(&node) {
        return Some(v);
    }
    if graph.ac_ground_nodes.contains(&node) {
        return Some(0.0);
    }
    bias_node_voltages.get(&node).copied()
}

/// Infer the standing DC bias current through a transformer primary as
/// `(Vp − Vn) / R_primary`, using the DC potentials at the primary terminals.
///
/// Returns `None` when the primary DCR is unusable or either terminal has no
/// known DC potential. Relocated from `spqr_build.rs`; callers there re-export
/// through this module.
pub(crate) fn infer_transformer_dc_bias_current(
    cfg: &TransformerConfig,
    primary_pos: NodeId,
    primary_neg: NodeId,
    bias_node_voltages: &std::collections::BTreeMap<NodeId, f64>,
    graph: &CircuitGraph,
) -> Option<f64> {
    let r_primary = cfg.primary_dcr;
    if !(r_primary.is_finite() && r_primary > 1.0e-9) {
        return None;
    }

    let vp = node_dc_voltage(primary_pos, bias_node_voltages, graph)?;
    let vn = node_dc_voltage(primary_neg, bias_node_voltages, graph)?;
    let current = (vp - vn) / r_primary;
    (current.is_finite() && current.abs() > 1.0e-12).then_some(current)
}

/// Effective magnetic path length `le_eff = path_len + gap` (metres), falling
/// back to the documented default when geometry is absent.
fn effective_path_length(cfg: &TransformerConfig) -> f64 {
    let path_len = cfg
        .core_path_length
        .filter(|p| p.is_finite() && *p > 0.0)
        .unwrap_or(DEFAULT_PATH_LEN_M);
    let gap = cfg
        .core_gap
        .filter(|g| g.is_finite() && *g >= 0.0)
        .unwrap_or(0.0);
    (path_len + gap).max(1.0e-6)
}

/// Magnetizing inductance to use for the AC excursion: explicit `Lm`, else the
/// linear-skeleton default (`primary_inductance · coupling`, matching
/// `stamp_linear_transformer_skeleton`), else the library default.
fn magnetizing_inductance(cfg: &TransformerConfig) -> f64 {
    if let Some(lm) = cfg
        .magnetizing_inductance
        .filter(|l| l.is_finite() && *l > 0.0)
    {
        return lm;
    }
    let k = cfg.coupling.clamp(0.0, 1.0);
    let from_linear = cfg.primary_inductance * k.max(1.0e-6);
    if from_linear.is_finite() && from_linear > 0.0 {
        from_linear
    } else {
        DEFAULT_LM_H
    }
}

/// Number of primary turns for the Ampère-law field `H = N·I / le_eff`,
/// falling back to the silicon-steel demo core's turn count when absent
/// (`JaCoreModel::si_steel_demo().n_turns == 2000`, a representative small
/// audio-transformer winding).
const DEFAULT_PRIMARY_TURNS: f64 = 2000.0;

fn primary_turns(cfg: &TransformerConfig) -> f64 {
    cfg.core_primary_turns
        .filter(|n| n.is_finite() && *n > 0.0)
        .unwrap_or(DEFAULT_PRIMARY_TURNS)
}

/// Estimate the peak magnetic field a transformer core sees, given the
/// circuit's DC bias map and a worst-case primary-drive [`OperatingEnvelope`].
///
/// - `h_dc = N · I_dc / le_eff`, where `I_dc` is the standing primary current
///   (the model's explicit `dc_bias_current` if present, else inferred from the
///   primary-terminal DC potentials via [`infer_transformer_dc_bias_current`]).
/// - `h_ac_peak = N · I_mag / le_eff`, where
///   `I_mag = V_peak / (2·π·F_LOW_HZ·Lm)` and `V_peak` is the worst-case swing
///   the driver can impose across the primary.
/// - `h_peak = |h_dc| + h_ac_peak`.
///
/// When `driver` is `None` (the driver Thévenin / available swing could not be
/// read from the graph) the AC excursion is unknown: `h_peak` is forced to
/// [`UNKNOWN_DRIVE_H_PEAK`] and `drive_unknown` is set, so a downstream gate
/// prefers the physical (nonlinear) model.
pub(crate) fn transformer_core_excitation(
    cfg: &TransformerConfig,
    primary_pos: NodeId,
    primary_neg: NodeId,
    bias_node_voltages: &std::collections::BTreeMap<NodeId, f64>,
    graph: &CircuitGraph,
    driver: Option<OperatingEnvelope>,
) -> CoreExcitation {
    let le_eff = effective_path_length(cfg);
    let n_turns = primary_turns(cfg);

    // ── DC standing field ────────────────────────────────────────────────
    let i_dc = cfg.dc_bias_current.or_else(|| {
        infer_transformer_dc_bias_current(cfg, primary_pos, primary_neg, bias_node_voltages, graph)
    });
    let h_dc = match i_dc {
        Some(i) if i.is_finite() => n_turns * i / le_eff,
        _ => 0.0,
    };

    // ── AC excursion field ───────────────────────────────────────────────
    let lm = magnetizing_inductance(cfg);
    let i_mag = driver.and_then(|d| d.peak_reactive_current(lm, F_LOW_HZ));

    match i_mag {
        Some(i) if i.is_finite() => {
            let h_ac_peak = n_turns * i / le_eff;
            CoreExcitation {
                h_dc,
                h_ac_peak,
                h_peak: h_dc.abs() + h_ac_peak,
                drive_unknown: false,
            }
        }
        // Honest uncertainty: drive couldn't be determined → fail toward
        // physics with a conservatively large field. We still report the DC
        // standing field we *do* know about for diagnostics.
        _ => CoreExcitation {
            h_dc,
            h_ac_peak: UNKNOWN_DRIVE_H_PEAK,
            h_peak: h_dc.abs() + UNKNOWN_DRIVE_H_PEAK,
            drive_unknown: true,
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dsl::parse_pedal_file;

    /// Reproduce the compiler's DC-bias map (spqr_build.rs lines ~340-352):
    /// build the graph, partition into flow groups, classify, and collect the
    /// StaticBias DC voltages.
    fn bias_map_for(
        pedal: &crate::dsl::PedalDef,
    ) -> (CircuitGraph, std::collections::BTreeMap<NodeId, f64>) {
        let graph = CircuitGraph::from_pedal(pedal);
        let all_edges: Vec<usize> = (0..graph.edges.len()).collect();
        let groups = super::super::signal_flow::find_flow_groups(&all_edges, &graph);
        let mut bias: std::collections::BTreeMap<NodeId, f64> = std::collections::BTreeMap::new();
        for g in &groups {
            if let super::super::bias_analysis::GroupBiasKind::StaticBias { dc_voltages } =
                super::super::bias_analysis::classify_group_bias(g, &graph)
            {
                bias.extend(dc_voltages);
            }
        }
        (graph, bias)
    }

    /// Locate a transformer component's resolved config + primary terminal
    /// nodes (`<id>.a` / `<id>.b`) in a compiled graph.
    fn transformer_primary(
        graph: &CircuitGraph,
        pedal: &crate::dsl::PedalDef,
    ) -> (TransformerConfig, NodeId, NodeId) {
        let comp = pedal
            .components
            .iter()
            .find(|c| c.kind.is_transformer())
            .expect("a transformer component");
        let cfg = crate::model_lookup::transformer_config_from_dsl(
            comp.kind.transformer_config().unwrap(),
        );
        let pos = graph
            .node_names
            .get(&format!("{}.a", comp.id))
            .copied()
            .expect("primary .a");
        let neg = graph
            .node_names
            .get(&format!("{}.b", comp.id))
            .copied()
            .expect("primary .b");
        (cfg, pos, neg)
    }

    /// DC-biased single-ended output transformer (OT-DEMO-SE: IDC=45m,
    /// N1=2000, LE=0.10, GAP=1e-3). A standing field must be present and the
    /// worst-case total field must reach silicon-steel knee scale (J-A
    /// `a ≈ 1.1e3 A/m`) → would classify NONLINEAR.
    #[test]
    fn dc_biased_output_transformer_reaches_knee() {
        let src = r#"
pedal "OT SE" {
    supply 9V
    components {
        T1: transformer(26:1, OT-DEMO-SE)
        R_load: resistor(8)
    }
    nets {
        in -> T1.a
        T1.b -> gnd
        T1.c -> out, R_load.a
        T1.d -> gnd
        R_load.b -> gnd
    }
}
"#;
        let pedal = parse_pedal_file(src).expect("parse");
        let (graph, bias) = bias_map_for(&pedal);
        let (cfg, pos, neg) = transformer_primary(&graph, &pedal);

        // A single-ended OT is push-pull-class B0: even a modest worst-case
        // primary swing here. Use a supply-limited drive of a few volts into a
        // reactive (open-resistive) primary.
        let driver = OperatingEnvelope::new(
            /* r_src */ 270.0,
            /* swing */ 9.0,
            /* r_load */ f64::INFINITY,
        );
        let exc = transformer_core_excitation(&cfg, pos, neg, &bias, &graph, Some(driver));

        // le_eff = 0.10 + 1e-3 = 0.101; H_dc = 2000 * 0.045 / 0.101 ≈ 891 A/m.
        assert!(
            exc.h_dc.abs() > 500.0,
            "expected a substantial standing field, got h_dc = {} A/m",
            exc.h_dc
        );
        // J-A `a` for this core is 1100 A/m; total worst-case field must reach
        // that knee scale.
        assert!(
            exc.h_peak >= 1.0e3,
            "expected h_peak at/above silicon-steel knee (~1e3 A/m), got {} A/m",
            exc.h_peak
        );
        assert!(!exc.drive_unknown);
    }

    /// Clean line transformer (GENERIC_600_600, no DC bias) driven at line
    /// level. No standing field; the worst-case field stays well below the
    /// calibrated J-A knee (a≈656 A/m) → classifies LINEAR.
    #[test]
    fn clean_line_transformer_stays_linear() {
        let src = r#"
pedal "Line Iso" {
    supply 9V
    components {
        T1: transformer(1:1, GENERIC_600_600)
        R_load: resistor(600)
    }
    nets {
        in -> T1.a
        T1.b -> gnd
        T1.c -> out, R_load.a
        T1.d -> gnd
        R_load.b -> gnd
    }
}
"#;
        let pedal = parse_pedal_file(src).expect("parse");
        let (graph, bias) = bias_map_for(&pedal);
        let (cfg, pos, neg) = transformer_primary(&graph, &pedal);

        // Normal line-level drive: ~1.23 V peak (+4 dBu) from a low-impedance
        // op-amp-class source into the 600 Ω primary.
        let driver = OperatingEnvelope::new(
            /* r_src */ 50.0, /* swing */ 1.23, /* r_load */ 600.0,
        );
        let exc = transformer_core_excitation(&cfg, pos, neg, &bias, &graph, Some(driver));

        assert!(
            exc.h_dc.abs() < 1.0e-6,
            "expected no DC standing field, got h_dc = {} A/m",
            exc.h_dc
        );
        // N1=1200, Lm≈0.995 H: h_peak ≈ 182 A/m at line level — well below the
        // calibrated knee a≈656 (and its 0.8·knee≈524 linear band), so the core
        // classifies LINEAR at line level. (Was `< 100` when N1=1; the threshold
        // moved with the calibrated turn count, the LINEAR verdict did not.)
        let knee = crate::model_lookup::ja_core_from_transformer_cfg(&cfg).a as f64;
        assert!(
            exc.h_peak < 0.8 * knee,
            "expected h_peak in the linear band, got {} A/m vs 0.8·knee = {}",
            exc.h_peak,
            0.8 * knee
        );
        assert!(!exc.drive_unknown);
    }

    /// When the driver is unknown, the estimate fails toward physics: a
    /// conservatively large `h_peak` and the `drive_unknown` flag set.
    #[test]
    fn unknown_drive_fails_toward_physics() {
        let src = r#"
pedal "Line Iso" {
    supply 9V
    components {
        T1: transformer(1:1, GENERIC_600_600)
        R_load: resistor(600)
    }
    nets {
        in -> T1.a
        T1.b -> gnd
        T1.c -> out, R_load.a
        T1.d -> gnd
        R_load.b -> gnd
    }
}
"#;
        let pedal = parse_pedal_file(src).expect("parse");
        let (graph, bias) = bias_map_for(&pedal);
        let (cfg, pos, neg) = transformer_primary(&graph, &pedal);

        let exc = transformer_core_excitation(&cfg, pos, neg, &bias, &graph, None);
        assert!(exc.drive_unknown);
        assert!(
            exc.h_peak >= UNKNOWN_DRIVE_H_PEAK,
            "expected conservatively large h_peak, got {}",
            exc.h_peak
        );
    }
}
