//! Model conversion helpers and WDF tree utilities.
//!
//! Runtime helpers (balance_parallel_vs, has_pot, etc.) are re-exported
//! from pedalkernel-rt. Compiler-only model helpers stay here.

// Re-export runtime helpers
pub use pedalkernel_rt::helpers::*;

// ── Compiler-only model helpers ─────────────────────────────────────────

use crate::dsl::*;
use crate::elements::*;
use crate::model_lookup;

pub(super) fn diode_model(dt: DiodeType) -> DiodeModel {
    match dt {
        DiodeType::Silicon => DiodeModel::silicon(),
        DiodeType::Germanium => DiodeModel::germanium(),
        DiodeType::Led => DiodeModel::led(),
        DiodeType::Schottky => DiodeModel::schottky(),
    }
}

pub(super) fn jfet_model(name: &str, is_n_channel: bool) -> JfetModel {
    let model = model_lookup::jfet_model_by_name(name);
    // Handle polarity mismatch: if DSL says N-channel but model is P-channel
    if is_n_channel != model.is_n_channel {
        model_lookup::jfet_model_by_name("2N5457")
    } else {
        model
    }
}

pub(super) fn triode_model(name: &str) -> TriodeModel {
    model_lookup::triode_model_by_name(name)
}

pub(super) fn pentode_model(name: &str) -> PentodeModel {
    model_lookup::pentode_model_by_name(name)
}

pub(super) fn vari_mu_model(name: &str) -> VariMuModel {
    VariMuModel::by_name(name)
}

/// Look up a Gummel-Poon BJT model by name from the embedded SPICE model library.
pub(super) fn gummel_poon_model(name: &str) -> GummelPoonModel {
    model_lookup::bjt_model_by_name(name)
}

pub(super) fn mosfet_model(mt: MosfetType, is_n_channel: bool) -> MosfetModel {
    match mt {
        MosfetType::N2n7000 => MosfetModel::n_2n7000(),
        MosfetType::Irf520 => MosfetModel::n_irf520(),
        MosfetType::Bs250 => {
            if is_n_channel {
                // Mismatch: N-channel requested with P-channel type
                MosfetModel::n_2n7000()
            } else {
                MosfetModel::p_bs250()
            }
        }
        MosfetType::Irf9520 => {
            if is_n_channel {
                MosfetModel::n_2n7000()
            } else {
                MosfetModel::p_irf9520()
            }
        }
    }
}
