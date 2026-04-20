//! IIR rigid stage building (stub).
//!
//! All-linear rigid stages → biquad from MNA eigenvalues. O(1)/sample.

use super::super::stage::WdfStage;

pub(super) fn build_iir_stage() -> Result<WdfStage, String> {
    Err("IIR rigid stages not yet implemented".to_string())
}
