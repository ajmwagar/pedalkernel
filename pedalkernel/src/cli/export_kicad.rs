//! `pedalkernel export-kicad` — export a KiCad netlist from a `.pedal` file.

use std::path::Path;
use std::process;

use pedalkernel::dsl_expand::parse_and_expand_pedal_file;
use pedalkernel::kicad::export_kicad_netlist;

/// Load `file` (expanding `use(...)` subcircuits) and write the KiCad
/// S-expression netlist to `output`, or stdout when no path is given.
pub fn run(file: &str, output: Option<&str>) {
    let pedal = parse_and_expand_pedal_file(Path::new(file)).unwrap_or_else(|e| {
        eprintln!("Error: {e}");
        process::exit(1);
    });
    let netlist = export_kicad_netlist(&pedal);
    write_output(&netlist, output);
}

/// Write `content` to `path`, or stdout when `None`. Exits on I/O error.
pub(crate) fn write_output(content: &str, output: Option<&str>) {
    match output {
        Some(path) => {
            if let Err(e) = std::fs::write(path, content) {
                eprintln!("Error writing {path}: {e}");
                process::exit(1);
            }
        }
        None => print!("{content}"),
    }
}
