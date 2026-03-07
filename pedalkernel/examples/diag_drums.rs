use pedalkernel::dsl::parse_pedal_file;
use pedalkernel::compiler::compile_pedal;
use pedalkernel::PedalProcessor;

fn test_pedal(name: &str, src: &str, notes: &[u8]) {
    eprintln!("\n{:=<60}", "");
    eprintln!("  {name}");
    eprintln!("{:=<60}", "");

    let pedal = match parse_pedal_file(src) {
        Ok(p) => p,
        Err(e) => { eprintln!("  PARSE ERROR: {e}"); return; }
    };

    let mut proc = match compile_pedal(&pedal, 48000.0) {
        Ok(p) => p,
        Err(e) => { eprintln!("  COMPILE ERROR: {e}"); return; }
    };

    eprintln!("{}", proc.debug_dump());

    for &note in notes {
        // Process 480 samples silence first
        for _ in 0..480 { proc.process(0.0); }

        // Fire note
        proc.note_on(note, 127);

        // Collect 4800 samples (100ms)
        let mut output = Vec::new();
        for _ in 0..4800 {
            output.push(proc.process(0.0));
        }

        let peak = output.iter().map(|x| x.abs()).fold(0.0_f64, f64::max);
        let rms = (output.iter().map(|x| x * x).sum::<f64>() / output.len() as f64).sqrt();
        let zc: usize = output.windows(2)
            .filter(|w| w[0].signum() != w[1].signum() && w[0] != 0.0)
            .count();

        // Estimate frequency from zero crossings (2 crossings per period)
        let est_freq = (zc as f64 / 2.0) / (4800.0 / 48000.0);

        eprintln!("  note {note}: peak={peak:.6}, rms={rms:.6}, zc={zc}, est_freq={est_freq:.1}Hz");
        eprintln!("    first 5: {:?}", &output[..5]);

        // Show envelope: samples at key points
        let checkpoints = [0, 48, 240, 480, 960, 2400, 4799];
        let envelope: Vec<String> = checkpoints.iter()
            .filter(|&&i| i < output.len())
            .map(|&i| format!("[{}]={:.4}", i, output[i]))
            .collect();
        eprintln!("    envelope: {}", envelope.join(", "));

        // Show peak amplitude per ~10ms window to see decay
        let window = 480; // 10ms at 48kHz
        let mut peaks_per_window: Vec<String> = Vec::new();
        for chunk_start in (0..output.len()).step_by(window) {
            let chunk_end = (chunk_start + window).min(output.len());
            let chunk_peak = output[chunk_start..chunk_end]
                .iter()
                .map(|x| x.abs())
                .fold(0.0_f64, f64::max);
            peaks_per_window.push(format!("{:.4}", chunk_peak));
        }
        eprintln!("    peak/10ms: [{}]", peaks_per_window.join(", "));
    }
}

fn main() {
    // Test simplified RLC tank (from unit test)
    let simple_rlc = r#"
pedal "SIMPLE RLC" subtitle "test" {
  supply 9V
  components {
    T: trigger_input()
    R_damp: resistor(2.2)
    L: inductor(100m)
    C: cap(82u)
    R_out: resistor(10k)
  }
  nets {
    T.out -> R_damp.a
    R_damp.b -> L.a
    L.b -> C.a
    C.b -> gnd
    R_damp.b -> R_out.a
    R_out.b -> out
  }
  controls {
    T.trigger -> midi(36)
  }
}
"#;
    test_pedal("SIMPLE RLC TANK", simple_rlc, &[36]);

    // Test tr808_kit
    let kit_src = std::fs::read_to_string(
        "/Users/ajmwagar/src/pedalkernel/pedalkernel-pro/pedals/new/synth/tr808_kit.pedal"
    ).unwrap();
    test_pedal("TR-808 KIT", &kit_src, &[36, 56]);

    // Test sine_drum (just 2 voices for speed)
    let sine_src = std::fs::read_to_string(
        "/Users/ajmwagar/src/pedalkernel/pedalkernel-pro/pedals/new/synth/sine_drum.pedal"
    ).unwrap();
    test_pedal("SINE DRUM", &sine_src, &[48, 55]);
}
