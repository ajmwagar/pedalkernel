//! Subcircuit routing types for inter-subcircuit signal graph.
//!
//! These are the runtime-only types needed by the processor.
//! The compile-time resolution logic lives in the pedalkernel crate.

extern crate alloc;
use alloc::vec::Vec;

/// Routing step in the inter-subcircuit signal graph.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum RoutingStep {
    /// Process subcircuit at index, feeding it signal from source.
    ProcessSubcircuit {
        subcircuit_idx: usize,
        input_source: SignalSource,
    },
}

/// Where a subcircuit gets its input signal.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum SignalSource {
    /// The equipment-level input.
    EquipmentInput,
    /// Output of subcircuit at given index.
    SubcircuitOutput(usize),
    /// Sum of multiple sources (fan-in).
    #[allow(dead_code)]
    Sum(Vec<SignalSource>),
}
