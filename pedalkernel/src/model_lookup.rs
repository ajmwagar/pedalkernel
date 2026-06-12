//! Model lookup functions — bridges pedalkernel's model DB to pedalkernel-rt types.
//!
//! pedalkernel-rt defines the model structs (JfetModel, TriodeModel, etc.) but
//! cannot contain the SPICE model database (it's no_std). These free functions
//! look up models by name from pedalkernel's embedded model DB and convert them
//! to the runtime types.

use crate::dsl::TransformerConfig;
use crate::models::{
    bjt_by_name, jfet_by_name, opamp_by_name, pentode_by_name, transformer_by_name, triode_by_name,
    SpiceBjtModel, SpiceJfetModel, SpiceOpAmpModel, SpicePentodeModel, SpiceTransformerModel,
    SpiceTriodeModel,
};
use pedalkernel_rt::elements::nonlinear::{
    GummelPoonModel, JfetModel, OpAmpModel, PentodeModel, TriodeModel,
};

// ── JFET ────────────────────────────────────────────────────────────────

/// Look up a JFET model by name from the embedded SPICE model file.
///
/// Panics if the model name is not found.
pub fn jfet_model_by_name(name: &str) -> JfetModel {
    jfet_try_by_name(name).unwrap_or_else(|| panic!("Unknown JFET model: '{name}'"))
}

/// Try to look up a JFET model by name. Returns `None` if not found.
pub fn jfet_try_by_name(name: &str) -> Option<JfetModel> {
    jfet_by_name(name).map(|s| jfet_from_spice(&s))
}

fn jfet_from_spice(s: &SpiceJfetModel) -> JfetModel {
    JfetModel {
        vto: s.vto,
        beta: s.beta,
        lambda: s.lambda,
        gate_is: s.is,
        n: s.n,
        rd: s.rd,
        rs: s.rs,
        cgs: s.cgs,
        cgd: s.cgd,
        is_n_channel: s.is_n_channel,
    }
}

// ── Triode ──────────────────────────────────────────────────────────────

/// Look up a triode model by name from the model registry.
/// Panics if the name is not found.
pub fn triode_model_by_name(name: &str) -> TriodeModel {
    triode_try_by_name(name).unwrap_or_else(|| {
        panic!(
            "Unknown triode model: '{}'. Use triode_model_names() to list available models.",
            name
        )
    })
}

/// Look up a triode model by name, returning None if not found.
pub fn triode_try_by_name(name: &str) -> Option<TriodeModel> {
    triode_by_name(name).map(|s| triode_from_spice(&s))
}

fn triode_from_spice(spice: &SpiceTriodeModel) -> TriodeModel {
    TriodeModel {
        mu: spice.mu,
        kp: spice.kp,
        kvb: spice.kvb,
        ex: spice.ex,
        kg1: spice.kg1,
        rp: spice.rp,
    }
}

// ── Pentode ─────────────────────────────────────────────────────────────

/// Look up a pentode model by name from the model registry.
/// Panics if the name is not found.
pub fn pentode_model_by_name(name: &str) -> PentodeModel {
    pentode_try_by_name(name).unwrap_or_else(|| {
        panic!(
            "Unknown pentode model: '{}'. Use pentode_model_names() to list available models.",
            name
        )
    })
}

/// Look up a pentode model by name, returning None if not found.
pub fn pentode_try_by_name(name: &str) -> Option<PentodeModel> {
    pentode_by_name(name).map(|s| pentode_from_spice(&s))
}

fn pentode_from_spice(spice: &SpicePentodeModel) -> PentodeModel {
    PentodeModel {
        mu: spice.mu,
        kp: spice.kp,
        kvb: spice.kvb,
        ex: spice.ex,
        kvb2: spice.kvb2,
        vg2_default: spice.vg2_default,
        kg1: spice.kg1,
        kg2: spice.kg2,
        rp: spice.rp,
    }
}

// ── BJT (Gummel-Poon) ──────────────────────────────────────────────────

/// Look up a Gummel-Poon BJT model by name from the model registry.
/// Panics if the model name is not found.
pub fn bjt_model_by_name(name: &str) -> GummelPoonModel {
    bjt_try_by_name(name).unwrap_or_else(|| panic!("Unknown BJT model: '{name}'"))
}

/// Try to look up a Gummel-Poon model by name. Returns `None` if not found.
pub fn bjt_try_by_name(name: &str) -> Option<GummelPoonModel> {
    bjt_by_name(name).map(|s| bjt_from_spice(&s))
}

fn bjt_from_spice(spice: &SpiceBjtModel) -> GummelPoonModel {
    GummelPoonModel {
        is: spice.is,
        bf: spice.bf,
        br: spice.br,
        nf: spice.nf,
        nr: spice.nr,
        vt: 0.02585, // kT/q at 25°C
        vaf: spice.vaf,
        var: spice.var,
        ikf: spice.ikf,
        ikr: spice.ikr,
        ise: spice.ise,
        ne: spice.ne,
        isc: spice.isc,
        nc: spice.nc,
        cje: spice.cje,
        vje: spice.vje,
        mje: spice.mje,
        cjc: spice.cjc,
        vjc: spice.vjc,
        mjc: spice.mjc,
        rb: spice.rb,
        re: spice.re,
        rc: spice.rc,
        tf: spice.tf,
        tr: spice.tr,
        is_pnp: spice.is_pnp,
    }
}

// ── Op-Amp ──────────────────────────────────────────────────────────────

/// Look up a compact op-amp model by name from the model registry.
/// Panics if the model name is not found.
pub fn opamp_model_by_name(name: &str) -> OpAmpModel {
    opamp_try_by_name(name).unwrap_or_else(|| panic!("Unknown op-amp model: '{name}'"))
}

/// Try to look up a compact op-amp model by name. Returns `None` if not found.
pub fn opamp_try_by_name(name: &str) -> Option<OpAmpModel> {
    opamp_by_name(name).map(opamp_from_spice)
}

fn opamp_from_spice(spice: &SpiceOpAmpModel) -> OpAmpModel {
    OpAmpModel {
        open_loop_gain: spice.open_loop_gain,
        gbw: spice.gbw,
        slew_rate: spice.slew_rate,
        v_rail_pos: spice.v_rail_pos,
        v_rail_neg: spice.v_rail_neg,
        output_impedance: spice.output_impedance,
        output_capacitance: spice.output_capacitance,
    }
}

/// Convert from DSL OpAmpType to runtime OpAmpModel.
pub fn opamp_model_from_type(ot: &crate::dsl::OpAmpType) -> OpAmpModel {
    opamp_model_by_name(opamp_type_model_name(ot))
}

/// Return an OTA transconductance from the model registry.
pub fn ota_gm_from_type(ot: &crate::dsl::OpAmpType) -> Option<f64> {
    let model = opamp_by_name(opamp_type_model_name(ot))?;
    model.is_ota.then_some(model.ota_gm)
}

fn opamp_type_model_name(ot: &crate::dsl::OpAmpType) -> &'static str {
    use crate::dsl::OpAmpType;
    match ot {
        OpAmpType::Generic => "GENERIC",
        OpAmpType::Tl072 => "TL072",
        OpAmpType::Tl082 => "TL082",
        OpAmpType::Jrc4558 => "JRC4558",
        OpAmpType::Rc4558 => "RC4558",
        OpAmpType::Lm308 => "LM308",
        OpAmpType::Lm741 => "LM741",
        OpAmpType::Ne5532 => "NE5532",
        OpAmpType::Op07 => "OP07",
        OpAmpType::Ca3080 => "CA3080",
    }
}

// ── Transformer ─────────────────────────────────────────────────────────

/// Resolve a DSL transformer instance against the embedded model registry.
///
/// Explicit scalar fields on `cfg` override model-library defaults. The
/// current DSL cannot distinguish explicit default zeros for DCR/Cp from
/// unspecified values, so zero-valued DCR/Cp intentionally remain explicit
/// only for generic `transformer(ratio, Lp, ...)` instances; model-backed
/// instances inherit model DCR/Cp unless a non-zero override is present.
pub fn transformer_config_from_dsl(cfg: &TransformerConfig) -> TransformerConfig {
    let Some(model_name) = cfg.model.as_deref() else {
        return cfg.clone();
    };

    let model = transformer_by_name(model_name)
        .unwrap_or_else(|| panic!("Unknown transformer model: '{model_name}'"));
    let mut resolved = transformer_config_from_model(model, cfg.turns_ratio);

    resolved.primary_type = cfg.primary_type;
    resolved.secondary_type = cfg.secondary_type;
    resolved.tertiary_turns_ratio = cfg.tertiary_turns_ratio;

    // Instance overrides.
    if cfg.primary_inductance > 0.0 {
        resolved.primary_inductance = cfg.primary_inductance;
    }
    if cfg.primary_dcr > 0.0 {
        resolved.primary_dcr = cfg.primary_dcr;
    }
    if cfg.secondary_dcr > 0.0 {
        resolved.secondary_dcr = cfg.secondary_dcr;
    }
    if cfg.capacitance > 0.0 {
        resolved.capacitance = cfg.capacitance;
    }
    if cfg.coupling > 0.0 {
        resolved.coupling = cfg.coupling;
    }
    if cfg.primary_leakage.is_some() {
        resolved.primary_leakage = cfg.primary_leakage;
    }
    if cfg.secondary_leakage.is_some() {
        resolved.secondary_leakage = cfg.secondary_leakage;
    }
    if cfg.magnetizing_inductance.is_some() {
        resolved.magnetizing_inductance = cfg.magnetizing_inductance;
    }
    if cfg.core_loss_resistance.is_some() {
        resolved.core_loss_resistance = cfg.core_loss_resistance;
    }
    if cfg.core_primary_turns.is_some() {
        resolved.core_primary_turns = cfg.core_primary_turns;
    }
    if cfg.core_area.is_some() {
        resolved.core_area = cfg.core_area;
    }
    if cfg.core_path_length.is_some() {
        resolved.core_path_length = cfg.core_path_length;
    }
    if cfg.core_gap.is_some() {
        resolved.core_gap = cfg.core_gap;
    }
    if cfg.dc_bias_current.is_some() {
        resolved.dc_bias_current = cfg.dc_bias_current;
    }
    if cfg.ja_ms.is_some() {
        resolved.ja_ms = cfg.ja_ms;
    }
    if cfg.ja_a.is_some() {
        resolved.ja_a = cfg.ja_a;
    }
    if cfg.ja_alpha.is_some() {
        resolved.ja_alpha = cfg.ja_alpha;
    }
    if cfg.ja_k.is_some() {
        resolved.ja_k = cfg.ja_k;
    }
    if cfg.ja_c.is_some() {
        resolved.ja_c = cfg.ja_c;
    }

    resolved.model = Some(model.name.clone());
    resolved
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn transformer_model_values_merge_with_overrides() {
        let mut cfg = TransformerConfig::with_model(10.0, "JT11P1".to_string());
        cfg.magnetizing_inductance = Some(8.0);
        cfg.core_loss_resistance = Some(250_000.0);
        cfg.capacitance = 100e-12;

        let resolved = transformer_config_from_dsl(&cfg);
        assert_eq!(resolved.model.as_deref(), Some("JT11P1"));
        assert!((resolved.turns_ratio - 10.0).abs() < 1e-12);
        assert!((resolved.primary_inductance - 2.0).abs() < 1e-12);
        assert!((resolved.primary_dcr - 75.0).abs() < 1e-12);
        assert!((resolved.magnetizing_inductance.unwrap() - 8.0).abs() < 1e-12);
        assert!((resolved.core_loss_resistance.unwrap() - 250_000.0).abs() < 1e-6);
        assert!((resolved.capacitance - 100e-12).abs() < 1e-21);
    }

    #[test]
    fn transformer_model_propagates_ja_core_fields() {
        let cfg = TransformerConfig::with_model(26.0, "OT-DEMO-SE".to_string());
        let resolved = transformer_config_from_dsl(&cfg);

        assert_eq!(resolved.model.as_deref(), Some("OT-DEMO-SE"));
        assert!((resolved.core_primary_turns.unwrap() - 2000.0).abs() < 1e-12);
        assert!((resolved.core_area.unwrap() - 2.0e-4).abs() < 1e-18);
        assert!((resolved.core_path_length.unwrap() - 0.10).abs() < 1e-12);
        assert!((resolved.core_gap.unwrap() - 1.0e-3).abs() < 1e-18);
        assert!((resolved.dc_bias_current.unwrap() - 45.0e-3).abs() < 1e-18);
        assert!((resolved.ja_ms.unwrap() - 1.6e6).abs() < 1e-6);
        assert!((resolved.ja_a.unwrap() - 1100.0).abs() < 1e-12);
        assert!((resolved.ja_alpha.unwrap() - 1.6e-3).abs() < 1e-18);
        assert!((resolved.ja_k.unwrap() - 400.0).abs() < 1e-12);
        assert!((resolved.ja_c.unwrap() - 0.2).abs() < 1e-12);
    }
}

fn transformer_config_from_model(
    model: &SpiceTransformerModel,
    turns_ratio: f64,
) -> TransformerConfig {
    TransformerConfig {
        model: Some(model.name.clone()),
        turns_ratio,
        primary_inductance: model.primary_inductance,
        primary_dcr: model.primary_dcr,
        secondary_dcr: model.secondary_dcr,
        capacitance: model.capacitance,
        coupling: model.coupling,
        primary_leakage: Some(model.primary_leakage),
        secondary_leakage: Some(model.secondary_leakage),
        magnetizing_inductance: Some(model.magnetizing_inductance),
        core_loss_resistance: (model.core_loss_resistance > 0.0)
            .then_some(model.core_loss_resistance),
        core_primary_turns: (model.primary_turns > 0.0).then_some(model.primary_turns),
        core_area: (model.core_area > 0.0).then_some(model.core_area),
        core_path_length: (model.magnetic_path_length > 0.0).then_some(model.magnetic_path_length),
        core_gap: (model.gap_length > 0.0).then_some(model.gap_length),
        dc_bias_current: (model.dc_bias_current != 0.0).then_some(model.dc_bias_current),
        ja_ms: (model.ja_ms > 0.0).then_some(model.ja_ms),
        ja_a: (model.ja_a > 0.0).then_some(model.ja_a),
        ja_alpha: (model.ja_alpha != 0.0).then_some(model.ja_alpha),
        ja_k: (model.ja_k > 0.0).then_some(model.ja_k),
        ja_c: (model.ja_c > 0.0).then_some(model.ja_c),
        ..Default::default()
    }
}
