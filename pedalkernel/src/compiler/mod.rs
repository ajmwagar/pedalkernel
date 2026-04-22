//! Netlist-to-WDF compiler.
//!
//! Compiles a parsed `.pedal` file into a real-time audio processor by:
//! 1. Building a circuit graph from the netlist
//! 2. Identifying clipping stages (around diode elements)
//! 3. For each stage, building a WDF binary tree via series-parallel decomposition
//! 4. Modeling active elements (transistors, opamps) as gain stages
//! 5. Chaining everything into a cascaded `PedalProcessor`

mod bind;
mod bjt_bias_analysis;
mod build;
mod calibrate;
mod classify;
mod compile;
mod compiled;
pub mod component;
pub mod components;
mod dyn_node;
mod graph;
mod signal_flow;
mod spqr_control;
mod helpers;
mod rigid;
mod spqr;
mod spqr_build;
mod split;
mod stage;
pub mod stage_test_helpers;
pub(crate) mod subcircuit;
pub mod validate;
mod warnings;
pub(crate) mod wdf_leaf;

pub use compile::{compile_pedal, compile_pedal_with_options, CompileOptions};
pub use spqr_build::{compile_via_spqr, compile_via_spqr_with_options};
pub(crate) use compiled::extract_precomputed_from_compiled;
pub use compiled::CompiledPedal;
pub use component::{Component, PinDirection};
pub use split::{compile_split_pedal, SplitCompiledPedal};
pub use validate::{validate_pedal, validate_pedal_files, PedalWarning, Severity};
pub use warnings::{check_voltage_compatibility, VoltageWarning, WarningSeverity};

#[cfg(test)]
mod voltage_extraction_tests;
#[cfg(test)]
mod spqr_tests;
#[cfg(test)]
mod spqr_build_tests;
#[cfg(test)]
mod rigid_tests;
#[cfg(test)]
mod bind_tests;
#[cfg(test)]
mod general_mna_tests;
#[cfg(test)]
mod pot_tests;
#[cfg(test)]
mod graph_role_tests;
#[cfg(test)]
mod nonideal_tests;
#[cfg(test)]
mod split_point_tests;
#[cfg(test)]
mod pot_sweep_tests;
#[cfg(test)]
mod zero_output_tests;
#[cfg(test)]
mod pot_binding_tests;
#[cfg(test)]
mod black_feedback_tests;
