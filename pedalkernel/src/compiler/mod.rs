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
mod classify;
mod compile;
mod compiled;
pub mod component;
pub mod components;
mod dyn_node;
mod graph;
mod helpers;
mod opamp_analysis;
mod plan;
mod rigid;
mod spqr;
mod spqr_build;
mod split;
mod stage;
pub mod stage_test_helpers;
pub(crate) mod subcircuit;
mod topology;
pub mod validate;
mod warnings;
pub(crate) mod wdf_leaf;

pub use compile::{compile_pedal, compile_pedal_with_options, CompileOptions};
pub use spqr_build::compile_via_spqr;
pub(crate) use compiled::extract_precomputed_from_compiled;
pub use compiled::CompiledPedal;
pub use component::{Component, PinDirection};
pub use split::{compile_split_pedal, SplitCompiledPedal};
pub use validate::{validate_pedal, validate_pedal_files, PedalWarning, Severity};
pub use warnings::{check_voltage_compatibility, VoltageWarning, WarningSeverity};

#[cfg(test)]
mod tests;
#[cfg(test)]
mod spqr_tests;
#[cfg(test)]
mod spqr_build_tests;
#[cfg(test)]
mod rigid_tests;
