use pedalkernel::compiler::compile_pedal;
use pedalkernel::dsl::parse_pedal_file;
use std::process;

pub fn run(file_path: &str) {
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

    // Print parsed component summary
    println!("file: {file_path}");
    println!("name: {}", pedal.name);
    if let Some(ref sub) = pedal.subtitle {
        println!("subtitle: {sub}");
    }
    println!("components: {}", pedal.components.len());
    println!("nets: {}", pedal.nets.len());
    println!();

    // Compile
    let compiled = match compile_pedal(&pedal, 48000.0) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("Compile error: {e}");
            process::exit(1);
        }
    };

    // Print the debug dump
    print!("{}", compiled.debug_dump());
}
