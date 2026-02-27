//! Netlist-to-WDF compiler.
//!
//! Compiles a parsed `.pedal` file into a real-time audio processor by:
//! 1. Building a circuit graph from the netlist
//! 2. Identifying clipping stages (around diode elements)
//! 3. For each stage, building a WDF binary tree via series-parallel decomposition
//! 4. Modeling active elements (transistors, opamps) as gain stages
//! 5. Chaining everything into a cascaded `PedalProcessor`

mod compile;
mod compiled;
mod dyn_node;
mod graph;
mod helpers;
mod split;
mod stage;
pub mod validate;
mod warnings;

pub use compile::{compile_pedal, compile_pedal_with_options, CompileOptions};
pub use compiled::CompiledPedal;
pub use split::{compile_split_pedal, SplitCompiledPedal};
pub use validate::{validate_pedal, PedalWarning, Severity};
pub use warnings::{check_voltage_compatibility, VoltageWarning, WarningSeverity};

#[cfg(test)]
mod tests;
