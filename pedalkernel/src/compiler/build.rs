//! Tree builder utilities shared across pipeline stages.
//!
//! Contains `create_root()` and `create_nl_device()` which are used by the
//! SPQR pipeline (spqr_build.rs, rigid/general.rs).

use crate::elements::*;

use super::classify::NonlinearKind;
use super::helpers::*;
use super::stage::{NlDeviceKind, RootKind};

// ═══════════════════════════════════════════════════════════════════════════
// NL device creation for multi-NL stages
// ═══════════════════════════════════════════════════════════════════════════

pub(super) fn create_nl_device(kind: &NonlinearKind) -> Option<NlDeviceKind> {
    match kind {
        NonlinearKind::Triode {
            model_name,
            parallel_count,
            is_vari_mu,
            ..
        } => {
            if *is_vari_mu {
                let model = vari_mu_model(model_name);
                Some(NlDeviceKind::VariMu(
                    VariMuTriodeRoot::new(model).with_parallel_count(*parallel_count),
                ))
            } else {
                let model = triode_model(model_name);
                Some(NlDeviceKind::Triode(
                    TriodeRoot::new(model).with_parallel_count(*parallel_count),
                ))
            }
        }
        NonlinearKind::SingleDiode(dt) => {
            let model = diode_model(*dt);
            Some(NlDeviceKind::Diode(DiodeRoot::new(model)))
        }
        NonlinearKind::DiodePair(dt) => {
            let model = diode_model(*dt);
            Some(NlDeviceKind::DiodePair(DiodePairRoot::new(model)))
        }
        NonlinearKind::Pentode { model_name, .. } => {
            let model = pentode_model(model_name);
            Some(NlDeviceKind::Pentode(PentodeRoot::new(model)))
        }
        NonlinearKind::Jfet {
            model_name,
            is_n_channel,
            ..
        } => {
            let model = crate::model_lookup::jfet_model_by_name(model_name);
            Some(NlDeviceKind::Jfet(JfetRoot::new(model)))
        }
        _ => None, // Mosfet, Zener, Ota not yet supported in multi-NL
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Root creation factory
// ═══════════════════════════════════════════════════════════════════════════

/// Create the RootKind for a nonlinear element.
/// Returns (root, base_diode_model) — diode stages store their model.
///
/// `use_jfet_vr` overrides JFET creation: when true, builds a
/// `JfetVr` (variable resistance, no NR) instead of `Jfet` (full NR solver).
pub(super) fn create_root(
    kind: &NonlinearKind,
    use_jfet_vr: bool,
) -> (RootKind, Option<DiodeModel>) {
    match kind {
        NonlinearKind::DiodePair(dt) => {
            let model = diode_model(*dt);
            // Use Wright Omega explicit solver (O(1), no iteration)
            (
                RootKind::ExplicitDiodePair(ExplicitDiodePairRoot::new(model)),
                Some(model),
            )
        }
        NonlinearKind::SingleDiode(dt) => {
            let model = diode_model(*dt);
            // Use Wright Omega explicit solver (O(1), no iteration)
            (
                RootKind::ExplicitSingleDiode(ExplicitDiodeRoot::new(model)),
                Some(model),
            )
        }
        NonlinearKind::Jfet {
            model_name,
            is_n_channel,
        } => {
            let model = jfet_model(model_name, *is_n_channel);
            if use_jfet_vr {
                (RootKind::JfetVr(JfetVariableResistor::new(model)), None)
            } else {
                (RootKind::Jfet(JfetRoot::new(model)), None)
            }
        }
        NonlinearKind::Triode {
            model_name,
            parallel_count,
            is_vari_mu,
            ..
        } => {
            if *is_vari_mu {
                let model = vari_mu_model(model_name);
                (
                    RootKind::VariMu(
                        VariMuTriodeRoot::new(model).with_parallel_count(*parallel_count),
                    ),
                    None,
                )
            } else {
                let model = triode_model(model_name);
                (
                    RootKind::Triode(TriodeRoot::new(model).with_parallel_count(*parallel_count)),
                    None,
                )
            }
        }
        NonlinearKind::Pentode { model_name, .. } => {
            let model = pentode_model(model_name);
            (RootKind::Pentode(PentodeRoot::new(model)), None)
        }
        NonlinearKind::Mosfet {
            mosfet_type,
            is_n_channel,
        } => {
            let model = mosfet_model(*mosfet_type, *is_n_channel);
            (RootKind::Mosfet(MosfetRoot::new(model)), None)
        }
        NonlinearKind::Zener { voltage } => {
            let model = ZenerModel::new(*voltage);
            (RootKind::Zener(ZenerRoot::new(model)), None)
        }
        NonlinearKind::Ota => {
            let model = OtaModel::ca3080();
            (RootKind::Ota(OtaRoot::new(model)), None)
        }
        NonlinearKind::BjtNpn {
            model_name,
            base_node,
            collector_node,
            ..
        } => {
            let model = gummel_poon_model(model_name);
            if base_node == collector_node {
                // Diode-connected BJT: base=collector → functions as a diode.
                // Use DiodeRoot with the BJT's junction parameters (IS, NF·Vt).
                // BjtRoot computes Ic(Vce) at fixed Vbe → flat b-axis (no signal
                // response). DiodeRoot computes Id(Vd) = Is·(exp(V/nVt)-1) where
                // V IS the port voltage — signal enters through b_tree naturally.
                let diode_model = DiodeModel::from_bjt_base_emitter(&model);
                (
                    RootKind::ExplicitSingleDiode(ExplicitDiodeRoot::new(diode_model)),
                    Some(diode_model),
                )
            } else {
                (RootKind::Bjt(BjtRoot::new(model, false)), None)
            }
        }
        NonlinearKind::BjtPnp {
            model_name,
            base_node,
            collector_node,
            ..
        } => {
            let model = gummel_poon_model(model_name);
            if base_node == collector_node {
                let diode_model = DiodeModel::from_bjt_base_emitter(&model);
                (
                    RootKind::ExplicitSingleDiode(ExplicitDiodeRoot::new(diode_model)),
                    Some(diode_model),
                )
            } else {
                (RootKind::Bjt(BjtRoot::new(model, true)), None)
            }
        }
    }
}
