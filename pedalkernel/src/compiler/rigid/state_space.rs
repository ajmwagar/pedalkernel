//! State-space rigid stage building (stub).
//!
//! VCVS + reactive feedback, no NL → Schur complement. O(N)/sample.

use super::super::stage::WdfStage;

pub(super) fn build_state_space_stage() -> Result<WdfStage, String> {
    Err("StateSpace rigid stages not yet implemented".to_string())
}
