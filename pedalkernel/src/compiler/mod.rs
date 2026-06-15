//! Netlist-to-WDF compiler.
//!
//! Compiles a parsed `.pedal` file into a real-time audio processor by:
//! 1. Building a circuit graph from the netlist
//! 2. Identifying clipping stages (around diode elements)
//! 3. For each stage, building a WDF binary tree via series-parallel decomposition
//! 4. Modeling active elements (transistors, opamps) as gain stages
//! 5. Chaining everything into a cascaded `PedalProcessor`

mod bbd_lowering;
mod bias_analysis;
mod bind;
mod blockwise;
mod build;
mod calibrate;
mod classify;
mod compile;
mod compiled;
pub mod component;
pub mod components;
mod coupling;
mod delay_lowering;
mod dsp_block;
mod dyn_node;
mod graph;
mod helpers;
mod k_method;
pub mod neighbor_roles;
mod rigid;
mod signal_flow;
mod split;
mod spqr;
mod spqr_build;
mod spqr_control;
mod spring_lowering;
mod stage;
pub mod stage_test_helpers;
pub(crate) mod subcircuit;
pub mod validate;
mod vca_lowering;
mod vco_lowering;
mod warnings;
pub(crate) mod wdf_leaf;

#[cfg(feature = "build-cache")]
pub use compile::compile_pedal_cached;
pub use compile::{compile_cache_key, compile_pedal, compile_pedal_with_options, CompileOptions};
pub(crate) use compiled::extract_precomputed_from_compiled;
pub use compiled::CompiledPedal;
pub use component::{
    Cardinality, Component, NeighborReq, NeighborRole, PinDirection,
};
pub use neighbor_roles::{
    completeness_errors, infer_neighbor_roles, InferredNeighbor,
};
pub use split::{compile_split_pedal, SplitCompiledPedal};
pub use spqr_build::{compile_via_spqr, compile_via_spqr_with_options};
pub use validate::{validate_pedal, validate_pedal_files, PedalWarning, Severity};
pub use warnings::{check_voltage_compatibility, VoltageWarning, WarningSeverity};

#[cfg(test)]
mod bias_analysis_tests;
#[cfg(test)]
mod bias_application_tests;
#[cfg(test)]
mod bind_tests;
#[cfg(test)]
mod black_feedback_tests;
#[cfg(test)]
mod diode_polarity_tests;
#[cfg(test)]
mod extraction_candidate_tests;
#[cfg(test)]
mod feedforward_tests;
#[cfg(test)]
mod general_mna_tests;
#[cfg(test)]
mod goldenrod_pot_tests;
#[cfg(test)]
mod goldenrod_signal_chain_tests;
#[cfg(test)]
mod graph_role_tests;
#[cfg(test)]
mod ground_clip_tests;
#[cfg(test)]
mod group_terminal_tests;
#[cfg(test)]
mod harmonic_tests;
#[cfg(test)]
mod mna_accuracy_tests;
#[cfg(test)]
mod nonideal_tests;
#[cfg(test)]
mod opamp_drive_tests;
#[cfg(test)]
mod passive_extraction_tests;
#[cfg(test)]
mod passive_wdf_tests;
#[cfg(test)]
mod pot_binding_investigation_tests;
#[cfg(test)]
mod pot_binding_tests;
#[cfg(test)]
mod pot_sweep_tests;
#[cfg(test)]
mod pot_tests;
#[cfg(test)]
mod pot_wiper_tests;
#[cfg(test)]
mod ratking_signal_chain_tests;
#[cfg(test)]
mod rigid_tests;
#[cfg(test)]
mod split_point_tests;
#[cfg(test)]
mod spqr_build_tests;
#[cfg(test)]
mod spqr_tests;
#[cfg(test)]
mod state_space_pot_tests;
#[cfg(test)]
mod tone_stage_tests;
#[cfg(test)]
mod tree_build_tests;
#[cfg(test)]
mod voltage_extraction_tests;
#[cfg(test)]
mod zero_output_tests;
// incremental_recompute_tests moved to pedalkernel-rt/tests/incremental_recompute.rs
#[cfg(test)]
mod biquad_table_tests;
#[cfg(test)]
mod blockwise_detection_tests;
#[cfg(test)]
mod bridged_t_bias_loading_tests;
#[cfg(test)]
mod coupling_tests;
#[cfg(test)]
mod ground_leg_tests;
#[cfg(test)]
mod k_method_non_bjt_tests;
#[cfg(test)]
mod k_method_tests;
#[cfg(test)]
mod level_pot_tests;
#[cfg(test)]
mod mfb_debug_test;
#[cfg(test)]
mod opamp_gain_tests;
#[cfg(test)]
mod tb303_decomposition_tests;
