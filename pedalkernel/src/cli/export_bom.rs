//! `pedalkernel export-bom` — export a bill of materials from a `.pedal` file.

use std::path::Path;
use std::process;

use pedalkernel::bom::{build_bom, to_csv, to_table};
use pedalkernel::dsl_expand::parse_and_expand_pedal_file;

/// BOM output format.
#[derive(Debug, Clone, Copy, PartialEq, Eq, clap::ValueEnum)]
pub enum BomFormat {
    /// Human-readable aligned table: per-refdes rows + aggregated view.
    Table,
    /// Machine-readable CSV of the per-refdes rows
    /// (`refdes,category,value,model,qty`).
    Csv,
}

/// Load `file` (expanding `use(...)` subcircuits), build the BOM, and write
/// it in `format` to `output`, or stdout when no path is given.
pub fn run(file: &str, format: BomFormat, output: Option<&str>) {
    let pedal = parse_and_expand_pedal_file(Path::new(file)).unwrap_or_else(|e| {
        eprintln!("Error: {e}");
        process::exit(1);
    });
    let bom = build_bom(&pedal);
    let rendered = match format {
        BomFormat::Table => to_table(&bom),
        BomFormat::Csv => to_csv(&bom),
    };
    super::export_kicad::write_output(&rendered, output);
}
