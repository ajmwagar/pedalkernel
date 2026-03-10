use pedalkernel::dsl::parse_pedal_file;
use pedalkernel::compiler::compile_pedal;
use pedalkernel::PedalProcessor;

fn main() {
    let base = "/Users/ajmwagar/src/pedalkernel/pedalkernel-pro/pedals/legends";
    let pedals = [
        ("RATKING", "ratking.pedal", vec![("Distortion", 0.5), ("Filter", 0.5), ("Volume", 0.5)]),
        ("SCREAMER", "screamer.pedal", vec![("Drive", 0.5), ("Tone", 0.5), ("Level", 0.5)]),
        ("GOLDENROD", "goldenrod.pedal", vec![("Gain", 0.5), ("Treble", 0.5), ("Output", 0.5)]),
        ("MUFF", "muff.pedal", vec![("Sustain", 0.7), ("Tone", 0.5), ("Volume", 0.6)]),
    ];

    let input: Vec<f64> = (0..4800).map(|i| {
        (2.0 * std::f64::consts::PI * 440.0 * i as f64 / 48000.0).sin() * 0.1
    }).collect();

    for (name, file, knobs) in &pedals {
        let src = std::fs::read_to_string(format!("{}/{}", base, file)).unwrap();
        let pedal = parse_pedal_file(&src).unwrap();
        let mut proc = compile_pedal(&pedal, 48000.0).unwrap();
        for &(k, v) in knobs { proc.set_control(k, v); }
        let output: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();
        let peak = output[480..].iter().copied().fold(0.0_f64, |a, b| a.max(b.abs()));
        let rms = (output[480..].iter().map(|x| x * x).sum::<f64>() / (output.len() - 480) as f64).sqrt();
        println!("{}: peak={:.6}, rms={:.6}, gain_dB={:.1}",
            name, peak, rms, 20.0 * (peak / 0.1).log10());
    }

    // Also test the core example proco_rat for comparison
    let src = std::fs::read_to_string("/Users/ajmwagar/src/pedalkernel/pedalkernel/pedalkernel/examples/pedals/distortion/proco_rat.pedal").unwrap();
    let pedal = parse_pedal_file(&src).unwrap();
    let mut proc = compile_pedal(&pedal, 48000.0).unwrap();
    proc.set_control("Distortion", 0.5);
    proc.set_control("Filter", 0.5);
    proc.set_control("Volume", 0.5);
    let output: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();
    let peak = output[480..].iter().copied().fold(0.0_f64, |a, b| a.max(b.abs()));
    let rms = (output[480..].iter().map(|x| x * x).sum::<f64>() / (output.len() - 480) as f64).sqrt();
    println!("proco_rat(core): peak={:.6}, rms={:.6}, gain_dB={:.1}",
        peak, rms, 20.0 * (peak / 0.1).log10());
}
