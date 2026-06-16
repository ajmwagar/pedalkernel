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
    CoreMaterial, GummelPoonModel, JaCoreModel, JfetModel, OpAmpModel, PentodeModel, TriodeModel,
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
    jfet_by_name(name).map(|s| jfet_from_spice(s))
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
    triode_by_name(name).map(|s| triode_from_spice(s))
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
    pentode_by_name(name).map(|s| pentode_from_spice(s))
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
    bjt_by_name(name).map(|s| bjt_from_spice(s))
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

// ── Transformer core → JA model adapter ───────────────────────────────────

/// Default primary turns for a derived core when `core_primary_turns` is absent.
/// Matches the operating-point estimator's fallback
/// (`operating_point::DEFAULT_PRIMARY_TURNS == 2000`) so the geometry the gate
/// estimates the field against agrees with the geometry the JA core is built
/// from. A representative small audio-transformer winding.
const DEFAULT_CORE_TURNS: f64 = 2000.0;
/// Default core cross-sectional area (m²) — matches `JaCoreModel::si_steel_demo`.
const DEFAULT_CORE_AREA_M2: f64 = 2.0e-4;
/// Default magnetic path length (m) — matches the estimator's
/// `operating_point::DEFAULT_PATH_LEN_M` and `JaCoreModel::si_steel_demo`.
const DEFAULT_CORE_PATH_LEN_M: f64 = 0.10;

/// True when a transformer config carries enough physical core data to ground a
/// nonlinear-core decision: real core geometry (`core_area` +
/// `core_path_length` + `core_primary_turns`), which — together with the
/// material from `MS` — places the derived saturation knee at the part's rated
/// level (see `models/transformers.model`).
///
/// A bare `transformer(ratio, L)` declares NO core physics — only an ideal
/// coupled-inductor model. Deriving a saturation knee for it from fallback
/// geometry would fabricate a nonlinearity the netlist never declared, so the
/// gate treats such transformers as linear (see the architecture directive:
/// components carry physics, the engine derives boundaries — no declared core
/// physics ⇒ no saturation).
///
/// Geometry is authoritative because the knee is ALWAYS derived from `Lm` +
/// geometry + material (never an explicit `a`); a model with a material (`MS`)
/// but no geometry has no grounded knee scale and stays linear.
pub fn transformer_has_core_physics(cfg: &TransformerConfig) -> bool {
    cfg.core_area.is_some_and(|a| a.is_finite() && a > 0.0)
        && cfg
            .core_path_length
            .is_some_and(|p| p.is_finite() && p > 0.0)
        && cfg
            .core_primary_turns
            .is_some_and(|n| n.is_finite() && n > 0.0)
}

/// Build the Jiles-Atherton core model for a (resolved) transformer config.
///
/// # The load-bearing invariant
///
/// The JA branch's small-signal tangent MUST equal today's linear magnetizing
/// inductance `Lm`, so a circuit that never saturates produces byte-identical
/// output whether the gate runs JA or the linear tangent. Therefore the shape
/// parameter `a` (= the saturation knee field) is ALWAYS derived from `Lm` +
/// geometry + material via [`JaCoreModel::from_small_signal`] — it is never
/// hand-set, because an explicit `a` would override the `Lm`-derived
/// inductance and break the invariant.
///
/// # What the model fields contribute
///
/// - **Material** comes from the model's saturation magnetization `MS`
///   ([`CoreMaterial::from_ms`] picks the nearest soft-magnetic class). The
///   class fixes `Ms` and the *default* loop-shape `(alpha, k, c)`.
/// - **Geometry** (`N1`/`AE`/`LE`/`GAP`) sets the knee scale: with `Lm` fixed,
///   the geometry/material determine where saturation onsets. A part's
///   geometry is calibrated (see `models/transformers.model`) so the derived
///   knee sits at its datasheet rated max level, above line-level stimulus.
/// - **Explicit loop params** (`ALPHA`/`JA_K`/`JA_C`) are treated ONLY as
///   hysteresis-SHAPE overrides on top of the material defaults — coercivity,
///   remanence, reversibility. They never set the inductance-fixing `a`.
///
/// `lm` is the explicit `magnetizing_inductance`, else `primary_inductance ·
/// coupling` (matching `stamp_linear_transformer_skeleton`). Geometry falls
/// back to the documented defaults above (matching `operating_point`'s
/// field-estimate fallbacks) when a model omits a dimension.
pub fn ja_core_from_transformer_cfg(cfg: &TransformerConfig) -> JaCoreModel {
    let n_turns = cfg
        .core_primary_turns
        .filter(|n| n.is_finite() && *n > 0.0)
        .unwrap_or(DEFAULT_CORE_TURNS);
    let area = cfg
        .core_area
        .filter(|a| a.is_finite() && *a > 0.0)
        .unwrap_or(DEFAULT_CORE_AREA_M2);
    let path_len = cfg
        .core_path_length
        .filter(|p| p.is_finite() && *p > 0.0)
        .unwrap_or(DEFAULT_CORE_PATH_LEN_M);
    let gap = cfg
        .core_gap
        .filter(|g| g.is_finite() && *g >= 0.0)
        .unwrap_or(0.0);

    // Magnetizing inductance: explicit `Lm`, else the linear-skeleton default
    // `l_primary · coupling` (matching `stamp_linear_transformer_skeleton`).
    let lm = cfg
        .magnetizing_inductance
        .filter(|l| l.is_finite() && *l > 0.0)
        .unwrap_or_else(|| {
            let k = cfg.coupling.clamp(0.0, 1.0).max(1.0e-6);
            cfg.primary_inductance * k
        });

    // Material from the model's saturation magnetization (loop-shape family);
    // default silicon steel when no `MS` was declared.
    let material = match cfg.ja_ms.filter(|m| m.is_finite() && *m > 0.0) {
        Some(ms) => CoreMaterial::from_ms(ms as pedalkernel_rt::Wave),
        None => CoreMaterial::SiliconSteel,
    };

    // ALWAYS derive `a` from Lm so the small-signal tangent == today's `Lm`.
    let mut model = JaCoreModel::from_small_signal(
        lm as pedalkernel_rt::Wave,
        n_turns as pedalkernel_rt::Wave,
        area as pedalkernel_rt::Wave,
        path_len as pedalkernel_rt::Wave,
        gap as pedalkernel_rt::Wave,
        material,
    );

    // Explicit loop params are hysteresis-SHAPE overrides only — never `a`.
    // (`ALPHA` also feeds `a`'s reversible-coefficient derivation, but here it
    // post-overrides the *runtime* loop shape after `a` is fixed; we leave
    // `from_small_signal`'s `a` untouched so the tangent stays pinned to `Lm`.)
    if let Some(alpha) = cfg.ja_alpha.filter(|v| v.is_finite()) {
        model.alpha = alpha as pedalkernel_rt::Wave;
    }
    if let Some(k) = cfg.ja_k.filter(|v| v.is_finite() && *v > 0.0) {
        model.k = k as pedalkernel_rt::Wave;
    }
    if let Some(c) = cfg
        .ja_c
        .filter(|v| v.is_finite() && (0.0..=1.0).contains(v))
    {
        model.c = c as pedalkernel_rt::Wave;
    }

    model
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
        // AE made self-consistent (4.364e-3) so the DERIVED knee = the validated
        // a≈1100 without an explicit `A` field (which was removed from the model).
        assert!((resolved.core_area.unwrap() - 4.364e-3).abs() < 1e-9);
        assert!((resolved.core_path_length.unwrap() - 0.10).abs() < 1e-12);
        assert!((resolved.core_gap.unwrap() - 1.0e-3).abs() < 1e-18);
        assert!((resolved.dc_bias_current.unwrap() - 45.0e-3).abs() < 1e-18);
        assert!((resolved.ja_ms.unwrap() - 1.6e6).abs() < 1e-6);
        // No explicit `A` any more — the knee is derived from Lm + geometry.
        assert!(resolved.ja_a.is_none());
        assert!((resolved.ja_alpha.unwrap() - 1.6e-3).abs() < 1e-18);
        assert!((resolved.ja_k.unwrap() - 400.0).abs() < 1e-12);
        assert!((resolved.ja_c.unwrap() - 0.2).abs() < 1e-12);
    }

    #[test]
    fn bare_transformer_has_no_core_physics() {
        // `transformer(10:1, 2H)` — no model, no geometry, no JA params.
        let mut cfg = TransformerConfig::default();
        cfg.turns_ratio = 10.0;
        cfg.primary_inductance = 2.0;
        assert!(
            !transformer_has_core_physics(&cfg),
            "a bare coupled-inductor transformer declares no core physics"
        );
    }

    #[test]
    fn modelled_se_ot_has_core_physics_and_derives_validated_knee() {
        let cfg = transformer_config_from_dsl(&TransformerConfig::with_model(
            26.0,
            "OT-DEMO-SE".to_string(),
        ));
        assert!(transformer_has_core_physics(&cfg));
        // The knee is DERIVED from Lm + geometry + material (no explicit `a`).
        // AE was made self-consistent (4.364e-3) so the derived knee reproduces
        // the validated a≈1100 — uniform with the rest of the library.
        let core = ja_core_from_transformer_cfg(&cfg);
        assert!(
            (core.a as f64 - 1100.0).abs() < 2.0,
            "derived JA knee should reproduce the validated ~1100, got a = {}",
            core.a
        );
        assert!((core.ms as f64 - 1.6e6).abs() < 1.0);
    }

    #[test]
    fn derived_core_tangent_reproduces_lm() {
        // No explicit JA: `from_small_signal` picks `a` so the tangent matches
        // a physical Lm. Use real geometry consistent with the target Lm.
        let mut cfg = TransformerConfig::default();
        cfg.primary_inductance = 0.5;
        cfg.coupling = 0.99;
        cfg.core_primary_turns = Some(800.0);
        cfg.core_area = Some(2.0e-4);
        cfg.core_path_length = Some(0.10);
        assert!(transformer_has_core_physics(&cfg));
        let core = ja_core_from_transformer_cfg(&cfg);
        // A finite, physical knee (not the degenerate clamp).
        assert!(core.a.is_finite() && (core.a as f64) > 0.0 && (core.a as f64) < 1.0e7);
    }

    /// Peak core field `H = N·I/le_eff` for a primary swing `v_peak` at
    /// `F_LOW = 20 Hz`, matching `operating_point::transformer_core_excitation`.
    fn h_peak_for(cfg: &TransformerConfig, v_peak: f64) -> f64 {
        let core = ja_core_from_transformer_cfg(cfg);
        let n = core.n_turns as f64;
        let le_eff = (core.path_len + core.gap) as f64;
        let lm = cfg.magnetizing_inductance.unwrap_or(cfg.primary_inductance);
        let i_mag = v_peak / (2.0 * std::f64::consts::PI * 20.0 * lm);
        n * i_mag / le_eff
    }

    fn dbu_to_vpeak(dbu: f64) -> f64 {
        0.774_596_669_241_483 * 10f64.powf(dbu / 20.0) * std::f64::consts::SQRT_2
    }

    /// A calibrated library transformer (a) stays LINEAR at line level and
    /// (b) flips to JA when driven near its rated max @20Hz. This is the
    /// permanent acceptance anchor for the per-part core-data calibration
    /// (`models/transformers.model`): the derived knee `a` must sit ABOVE the
    /// line-level field and AT/BELOW the rated-max field.
    #[test]
    fn calibrated_library_transformer_linear_at_line_ja_at_rated() {
        // GENERIC_10K_10K: NickelIron, rated ~+20 dBu @20Hz, no DC bias.
        let cfg = transformer_config_from_dsl(&TransformerConfig::with_model(
            1.0,
            "GENERIC_10K_10K".to_string(),
        ));
        assert!(transformer_has_core_physics(&cfg));
        let knee = ja_core_from_transformer_cfg(&cfg).a as f64;

        // Line level (+4 dBu): well inside the linear band (h < 0.8·knee).
        let h_line = h_peak_for(&cfg, dbu_to_vpeak(4.0));
        let band = 0.8 * knee;
        assert!(
            h_line < band,
            "expected LINEAR at line level: h_line={h_line:.1} vs 0.8·knee={band:.1}"
        );

        // Rated max (+20 dBu): field reaches/exceeds the knee → JA territory.
        let h_rated = h_peak_for(&cfg, dbu_to_vpeak(20.0));
        assert!(
            h_rated >= knee,
            "expected JA at rated max: h_rated={h_rated:.1} vs knee={knee:.1}"
        );
    }

    /// JT11P1 carries a Jensen [DATASHEET] rated level (+20 dBu @20Hz); its
    /// derived small-signal tangent must still reproduce `Lm` (the load-bearing
    /// invariant) — `from_small_signal` pins `a` so `χ_i` gives `Lm` to ~0.01%.
    #[test]
    fn calibrated_jt11p1_derived_tangent_matches_lm() {
        let cfg =
            transformer_config_from_dsl(&TransformerConfig::with_model(1.0, "JT11P1".to_string()));
        let core = ja_core_from_transformer_cfg(&cfg);
        // Reconstruct the small-signal inductance the JA tangent delivers and
        // compare to the model's Lm (the invariant that keeps goldens stable).
        let lm = cfg.magnetizing_inductance.unwrap();
        let (n, ae, le, gap) = (
            core.n_turns as f64,
            core.area as f64,
            core.path_len as f64,
            core.gap as f64,
        );
        let mu0 = 1.256_637_061_435_917_3e-6;
        let k0 = n * n * mu0 * ae;
        let le_eff = le + gap;
        // Runtime small-signal χ_i = r/(1−α·r), r = c·ms/(3a).
        let r = (core.c as f64) * (core.ms as f64) / (3.0 * core.a as f64);
        let chi = r / (1.0 - core.alpha as f64 * r);
        let lm_tangent = if gap > 0.0 {
            k0 * (1.0 + chi) / (le_eff + chi * gap)
        } else {
            k0 * (1.0 + chi) / le_eff
        };
        let rel = (lm_tangent - lm).abs() / lm;
        assert!(
            rel < 1.0e-3,
            "derived JA tangent must reproduce Lm: lm_tangent={lm_tangent:.6} vs Lm={lm:.6} (rel={rel:.2e})"
        );
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
