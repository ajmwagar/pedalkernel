//! Compiled pedal processor — re-exported from pedalkernel-rt.
pub use pedalkernel_rt::processor::*;

/// Extract precomputed scattering data from a compiled pedal.
///
/// Called by [`crate::precompute::extract_precomputed`]; lives here so it
/// has access to the `pub(crate)` fields of [`CompiledPedal`] and
/// [`MultiNlStage`].
pub(crate) fn extract_precomputed_from_compiled(
    compiled: &CompiledPedal,
    pedal_source: &str,
    sample_rate: f64,
) -> crate::precompute::PrecomputedScattering {
    use crate::precompute::{
        pedal_hash, PrecomputedInterpTable, PrecomputedScattering, PrecomputedStage,
    };

    let hash = pedal_hash(pedal_source);
    let mut stages = Vec::new();
    let mut interp_tables = Vec::new();

    for (stage_idx, stage) in compiled.stages.iter().enumerate() {
        let mnl = match stage {
            Stage::MultiNl(m) => m,
            _ => continue,
        };
        let adaptor = mnl.adaptor();
        let n = adaptor.num_ports();
        let scattering = adaptor.power_scattering().to_vec();
        let port_resistances = adaptor.port_resistances();
        let vs_injection = mnl.vs_injection().map(ToOwned::to_owned);
        let extract_coeffs = mnl.extract_coeffs().map(ToOwned::to_owned);
        let extract_vs = mnl.extract_vs();

        stages.push(PrecomputedStage {
            stage_type: 1, // MultiNL
            n_ports: n,
            scattering,
            vs_injection,
            port_resistances,
            extract_coeffs,
            extract_vs,
        });

        if let Some(table) = mnl.interp_table() {
            interp_tables.push(PrecomputedInterpTable {
                stage_index: stage_idx,
                n_ports: n,
                has_vs_injection: table.has_vs_injection(),
                resistances: table.resistances().to_vec(),
                matrices: table.matrices().to_vec(),
                injections: table.injections().to_vec(),
            });
        }
    }

    PrecomputedScattering {
        pedal_hash: hash,
        sample_rate,
        stages,
        interp_tables,
    }
}
