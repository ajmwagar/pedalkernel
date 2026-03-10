//! Speaker cabinet modeling — `.cab` DSL → WDF → IR export.
//!
//! Describes cabinets through physical properties (driver Thiele-Small
//! parameters, enclosure geometry, mic placement) and compiles them into
//! real-time WDF processors via a synthetic equivalent circuit.

pub mod cab_def;
pub mod cab_dsl;
pub mod cab_transform;
pub mod cab_validate;
pub mod ir_export;
pub mod presets;

// Re-exports for convenience
pub use cab_def::CabDef;
pub use cab_dsl::parse_cab_file;
pub use cab_transform::compile_cab;
pub use cab_validate::{validate_cab, CabWarning, CabWarningSeverity};
pub use ir_export::{capture_ir, export_ir_wav, IrExportConfig};
