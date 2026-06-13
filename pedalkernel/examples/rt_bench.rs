//! Realtime-factor micro-bench for compiled pedals.
//!
//! Compiles a fixed set of circuits, processes 5 s of a 0.2-amplitude
//! 220 Hz sine at 48 kHz through each, and prints the achieved
//! x-realtime factor (audio seconds rendered per wall-clock second).
//!
//! Used to track the per-sample cost of controlled-resistor (photocoupler
//! LDR / JFET VR) scattering re-derivation versus circuits without
//! controlled resistors (dyna_comp, tube_screamer).
//!
//! Run with:
//!   cargo run --release --no-default-features --example rt_bench

use std::time::Instant;

use pedalkernel::compiler::compile_pedal;
use pedalkernel::dsl::parse_pedal_file;
use pedalkernel::PedalProcessor;

const SAMPLE_RATE: f64 = 48_000.0;
const DURATION_S: f64 = 5.0;
const FREQ_HZ: f64 = 220.0;
const AMP: f64 = 0.2;

fn main() {
    let circuits: &[(&str, &str)] = &[
        (
            "opto_leveler",
            "examples/outboard/compressor/opto_leveler.pedal",
        ),
        (
            "fet_leveler",
            "examples/outboard/compressor/fet_leveler.pedal",
        ),
        ("dyna_comp", "examples/pedals/compressor/dyna_comp.pedal"),
        (
            "tube_screamer",
            "examples/pedals/overdrive/tube_screamer.pedal",
        ),
        // Generator DspBlock (VCO): no WDF scattering, cheap (spec §3/§8).
        ("cem3340_vco", "examples/synths/cem3340_vco.pedal"),
        ("minisynth", "examples/synths/minisynth.pedal"),
        // Delay/tape DspBlock (F15): multi-tap delay line + wow LFO.
        ("space_echo", "examples/rack/space_echo.pedal"),
        // Spring-reverb DspBlock: dispersive allpass tank + op-amp driver/
        // recovery. The full pedal's factor here (~12x) is dominated by the
        // op-amp stages + oversampler + node routing (like opto_leveler ~13x),
        // NOT by the spring. The spring BLOCK's own per-sample cost clears
        // ~75x release at 64 allpasses (asserted directly on SpringTank::
        // process in tests/spring_reverb.rs — the realtime budget the design
        // doc specifies, isolated from the surrounding circuit stages).
        ("spring_reverb", "examples/rack/spring_reverb.pedal"),
    ];

    let num_samples = (SAMPLE_RATE * DURATION_S) as usize;
    println!(
        "rt_bench: {DURATION_S} s of {AMP}-amp {FREQ_HZ} Hz sine at {} Hz ({num_samples} samples)",
        SAMPLE_RATE as u64
    );

    for (name, rel_path) in circuits {
        let path = format!("{}/{rel_path}", env!("CARGO_MANIFEST_DIR"));
        let src = match std::fs::read_to_string(&path) {
            Ok(s) => s,
            Err(e) => {
                println!("{name:>16}: SKIP (read {path}: {e})");
                continue;
            }
        };
        let pedal = match parse_pedal_file(&src) {
            Ok(p) => p,
            Err(e) => {
                println!("{name:>16}: SKIP (parse: {e:?})");
                continue;
            }
        };
        let mut proc = match compile_pedal(&pedal, SAMPLE_RATE) {
            Ok(p) => p,
            Err(e) => {
                println!("{name:>16}: SKIP (compile: {e:?})");
                continue;
            }
        };

        // Warm-up: let auto-init, NR solver caches, and DC ramps settle
        // outside the timed region (0.25 s).
        let warmup = (SAMPLE_RATE * 0.25) as usize;
        for i in 0..warmup {
            let t = i as f64 / SAMPLE_RATE;
            let x = AMP * (2.0 * std::f64::consts::PI * FREQ_HZ * t).sin();
            std::hint::black_box(proc.process(x));
        }

        let start = Instant::now();
        let mut acc = 0.0_f64;
        for i in 0..num_samples {
            let t = i as f64 / SAMPLE_RATE;
            let x = AMP * (2.0 * std::f64::consts::PI * FREQ_HZ * t).sin();
            acc += proc.process(x).abs();
        }
        let elapsed = start.elapsed().as_secs_f64();
        std::hint::black_box(acc);

        let xrt = DURATION_S / elapsed;
        println!(
            "{name:>16}: {xrt:8.1}x realtime  ({elapsed:.3} s wall, mean |out| {:.4})",
            acc / num_samples as f64
        );
    }
}
