// Engine frequency probe — measures absolute dBFS levels and relative ratios
// Run: cargo test -p pedalkernel-validate --test probe_engine --release -- --nocapture
use pedalkernel::compiler::compile_pedal;
use pedalkernel::dsl::parse_pedal_file;
use pedalkernel::PedalProcessor;
use std::f64::consts::PI;

const SR: f64 = 96_000.0;

fn load(path: &str) -> String {
    std::fs::read_to_string(path).unwrap_or_else(|e| panic!("read {path}: {e}"))
}

/// Returns (input_dBFS, output_dBFS) at frequency, using RMS over steady-state portion.
fn rms_at(pedal_path: &str, freq: f64) -> (f64, f64) {
    let contents = load(pedal_path);
    let def = parse_pedal_file(&contents).unwrap_or_else(|e| panic!("parse: {e:?}"));
    let mut proc = compile_pedal(&def, SR).expect("compile");
    let n = 32_768;
    let warmup = 8_192;
    let amp = 0.5;
    let input: Vec<f64> = (0..n)
        .map(|i| amp * (2.0 * PI * freq * i as f64 / SR).sin())
        .collect();
    let output: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();
    let in_rms = rms(&input[warmup..]);
    let out_rms = rms(&output[warmup..]);
    let in_db = if in_rms < 1e-20 {
        -200.0
    } else {
        20.0 * in_rms.log10()
    };
    let out_db = if out_rms < 1e-20 {
        -200.0
    } else {
        20.0 * out_rms.log10()
    };
    (in_db, out_db)
}

fn rms(buf: &[f64]) -> f64 {
    (buf.iter().map(|x| x * x).sum::<f64>() / buf.len() as f64).sqrt()
}

#[test]
fn probe_all_configs() {
    let base = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("circuits/eq");
    let configs = [
        ("flat", "pultec_eqp1a_passive_flat.pedal"),
        ("lf_boost", "pultec_eqp1a_passive_lf_boost.pedal"),
        ("lf_trick", "pultec_eqp1a_passive_lf_trick.pedal"),
        ("hf_boost", "pultec_eqp1a_passive_hf_boost.pedal"),
        ("hf_atten", "pultec_eqp1a_passive_hf_atten.pedal"),
    ];
    // Key probe frequencies
    let freqs: &[(f64, &str)] = &[
        (60.0, "60"),
        (100.0, "100"),
        (1000.0, "1k"),
        (3100.0, "3.1k"),
        (5000.0, "5k"),
        (10000.0, "10k"),
        (16000.0, "16k"),
    ];

    println!("\n=== Engine Output Level (dBFS, input = 0.5 = -6dBFS) ===");
    println!(
        "{:<12} {:>7} {:>7} {:>7} {:>7} {:>7} {:>7} {:>7}",
        "config", "60Hz", "100Hz", "1kHz", "3.1kHz", "5kHz", "10kHz", "16kHz"
    );

    for (name, file) in &configs {
        let path = base.join(file);
        let mut row = format!("{:<12}", name);
        for &(freq, _) in freqs {
            let (_, out_db) = rms_at(path.to_str().unwrap(), freq);
            row.push_str(&format!(" {:>7.1}", out_db));
        }
        println!("{}", row);
    }

    println!("\n=== ngspice AC Reference (dBFS, input=0dBFS) ===");
    println!("(these are ABSOLUTE levels from SPICE AC analysis with 1V input)");
    println!(
        "{:<12} {:>7} {:>7} {:>7} {:>7} {:>7} {:>7} {:>7}",
        "config", "60Hz", "100Hz", "1kHz", "3.1kHz", "5kHz", "10kHz", "16kHz"
    );
    println!(
        "{:<12} {:>7} {:>7} {:>7} {:>7} {:>7} {:>7} {:>7}",
        "flat", "-76.5", "-72.0", "-52.0", "-44.3", "-40.9", "-42.9", "-53.9"
    );
    println!(
        "{:<12} {:>7} {:>7} {:>7} {:>7} {:>7} {:>7} {:>7}",
        "lf_boost", "-75.4", "-71.7", "-52.2", "-44.3", "-40.9", "-42.9", "-53.9"
    );
    println!(
        "{:<12} {:>7} {:>7} {:>7} {:>7} {:>7} {:>7} {:>7}",
        "lf_trick", "-75.5", "-71.8", "-52.2", "-41.1", "-34.8", "-31.5", "-38.7"
    );
    println!(
        "{:<12} {:>7} {:>7} {:>7} {:>7} {:>7} {:>7} {:>7}",
        "hf_boost", "-33.3", "-32.0", "-31.7", "-35.1", "-38.0", "-42.9", "-50.4"
    );
    println!(
        "{:<12} {:>7} {:>7} {:>7} {:>7} {:>7} {:>7} {:>7}",
        "hf_atten", "-76.5", "-72.0", "-52.0", "-44.5", "-42.1", "-42.7", "-50.7"
    );

    println!("\n=== Engine vs ngspice gap (engine_out - ngspice_ref) ===");
    println!("(negative = engine is quieter; positive = engine is louder)");
    println!("Note: engine uses -6dBFS input (0.5), ngspice uses 0dBFS (1.0)");
    println!("Gap shown is (engine_dBFS + 6) - ngspice_dBFS = relative transfer function error");
}
