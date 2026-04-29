//! Model lookup functions — bridges pedalkernel's model DB to pedalkernel-rt types.
//!
//! pedalkernel-rt defines the model structs (JfetModel, TriodeModel, etc.) but
//! cannot contain the SPICE model database (it's no_std). These free functions
//! look up models by name from pedalkernel's embedded model DB and convert them
//! to the runtime types.

use crate::models::{
    bjt_by_name, jfet_by_name, pentode_by_name, triode_by_name, SpiceBjtModel, SpiceJfetModel,
    SpicePentodeModel, SpiceTriodeModel,
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

/// Convert from DSL OpAmpType to runtime OpAmpModel.
pub fn opamp_model_from_type(ot: &crate::dsl::OpAmpType) -> OpAmpModel {
    use crate::dsl::OpAmpType;
    match ot {
        OpAmpType::Generic => OpAmpModel::generic(),
        OpAmpType::Tl072 => OpAmpModel::tl072(),
        OpAmpType::Tl082 => OpAmpModel::tl082(),
        OpAmpType::Jrc4558 => OpAmpModel::jrc4558(),
        OpAmpType::Rc4558 => OpAmpModel::rc4558(),
        OpAmpType::Lm308 => OpAmpModel::lm308(),
        OpAmpType::Lm741 => OpAmpModel::lm741(),
        OpAmpType::Ne5532 => OpAmpModel::ne5532(),
        OpAmpType::Op07 => OpAmpModel::op07(),
        // OTAs (CA3080) are handled separately as OtaRoot, not OpAmpRoot
        OpAmpType::Ca3080 => OpAmpModel::generic(),
    }
}
