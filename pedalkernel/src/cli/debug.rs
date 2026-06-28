use pedalkernel::compiler::compile_pedal;
use pedalkernel::dsl::parse_pedal_file;
use std::process;

/// Debug a `.pedal` file.
///
/// When `json` is false (default) this prints the prose stage/structure dump.
/// When `json` is true it prints ONLY a structured JSON `StageGraph` (no prose
/// preamble) so the output is machine-parseable by tooling such as the
/// schematic-trace IDE.
pub fn run(file_path: &str, json: bool) {
    // Parse the .pedal file
    let source = std::fs::read_to_string(file_path).unwrap_or_else(|e| {
        eprintln!("Error reading {file_path}: {e}");
        process::exit(1);
    });

    let mut pedal = parse_pedal_file(&source).unwrap_or_else(|e| {
        eprintln!("Parse error: {e}");
        process::exit(1);
    });

    // Expand file-based subcircuit `use(...)` instances into a flat pedal.
    if !pedal.uses.is_empty() {
        let base_dir = std::path::Path::new(file_path)
            .parent()
            .unwrap_or_else(|| std::path::Path::new("."));
        pedal = pedalkernel::dsl_expand::expand_uses(&pedal, base_dir).unwrap_or_else(|e| {
            eprintln!("Subcircuit expansion error: {e}");
            process::exit(1);
        });
    }

    if !json {
        // Print parsed component summary (prose mode only).
        println!("file: {file_path}");
        println!("name: {}", pedal.name);
        if let Some(ref sub) = pedal.subtitle {
            println!("subtitle: {sub}");
        }
        println!("components: {}", pedal.components.len());
        println!("nets: {}", pedal.nets.len());
        println!();
    }

    // Compile
    let compiled = match compile_pedal(&pedal, 48000.0) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("Compile error: {e}");
            process::exit(1);
        }
    };

    if json {
        // Structured StageGraph — additive, machine-readable.
        print!("{}", compiled.stage_graph_json());
    } else {
        // Prose debug dump (unchanged behavior).
        print!("{}", compiled.debug_dump());
    }
}
