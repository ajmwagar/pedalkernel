use pedalkernel::PedalProcessor;
use std::f64::consts::PI;

const GOLDENROD: &str =
    include_str!("/Users/ajmwagar/src/pedalkernel/pedalkernel-pro/pedals/legends/goldenrod.pedal");

#[test]
fn goldenrod_gain_affects_harmonics() {
    let pedal = pedalkernel::dsl::parse_pedal_file(GOLDENROD).expect("parse");
    let sr = 48000.0;
    let freq = 440.0;
    let amplitude = 1.0; // 0dBFS — calibrate_input_level attenuates internally
    let n_samples = 4096;

    let mut thd_values = Vec::new();

    for gain in [0.0, 0.25, 0.5, 0.75, 1.0] {
        let mut engine = pedalkernel::compiler::compile_pedal(&pedal, sr).expect("compile");
        engine.set_control("Gain", gain);
        engine.set_control("Output", 0.7);
        engine.set_control("Treble", 0.5);

        // Settle smoothers + DC
        for i in 0..8192 {
            let x = amplitude * (2.0 * PI * freq * i as f64 / sr).sin();
            engine.process(x);
        }

        // Capture
        let mut output = vec![0.0f64; n_samples];
        for i in 0..n_samples {
            let x = amplitude * (2.0 * PI * freq * (i as f64 + 8192.0) / sr).sin();
            output[i] = engine.process(x);
        }

        // DFT at harmonics
        let fundamental_bin = (freq * n_samples as f64 / sr).round() as usize;
        let mut power_fundamental = 0.0;
        let mut power_harmonics = 0.0;

        eprint!("Gain={:.2}: ", gain);
        for h in 1..=8 {
            let bin = fundamental_bin * h;
            if bin >= n_samples / 2 {
                break;
            }

            let mut re = 0.0;
            let mut im = 0.0;
            for (i, &s) in output.iter().enumerate() {
                let angle = 2.0 * PI * bin as f64 * i as f64 / n_samples as f64;
                re += s * angle.cos();
                im += s * angle.sin();
            }
            let mag = (re * re + im * im).sqrt() / n_samples as f64 * 2.0;

            if h == 1 {
                power_fundamental = mag * mag;
                eprint!("H1={:.4e} ", mag);
            } else {
                power_harmonics += mag * mag;
                if h <= 5 {
                    eprint!("H{}={:.4e} ", h, mag);
                }
            }
        }

        let thd = if power_fundamental > 0.0 {
            (power_harmonics / power_fundamental).sqrt() * 100.0
        } else {
            0.0
        };
        let rms = (output.iter().map(|x| x * x).sum::<f64>() / n_samples as f64).sqrt();
        let peak = output.iter().map(|x| x.abs()).fold(0.0f64, f64::max);
        eprintln!("RMS={:.4e} peak={:.4e} THD={:.1}%", rms, peak, thd);
        thd_values.push(thd);
    }

    // Dump pot state at full gain
    eprintln!("\n--- Engine pot state at Gain=1.0 ---");
    let mut engine = pedalkernel::compiler::compile_pedal(&pedal, sr).expect("compile");
    engine.set_control("Gain", 1.0);
    for _ in 0..1024 {
        engine.process(0.0);
    }
    for (label, target, smoothed) in engine.control_debug_info() {
        eprintln!("  {}: target={:.4} val={:.4}", label, target, smoothed);
    }

    // Check gain stage output amplitude (before diodes) by using very low output
    eprintln!("\n--- Signal level investigation ---");
    for gain in [0.0, 1.0] {
        let mut engine = pedalkernel::compiler::compile_pedal(&pedal, sr).expect("compile");
        engine.set_control("Gain", gain);
        engine.set_control("Output", 1.0);
        engine.set_control("Treble", 0.5);
        for i in 0..8192 {
            let x = amplitude * (2.0 * PI * freq * i as f64 / sr).sin();
            engine.process(x);
        }
        // Capture a few samples and print peak
        let mut peak = 0.0f64;
        let mut min_val = f64::MAX;
        let mut max_val = f64::MIN;
        for i in 0..4096 {
            let x = amplitude * (2.0 * PI * freq * (i as f64 + 8192.0) / sr).sin();
            let y = engine.process(x);
            peak = peak.max(y.abs());
            min_val = min_val.min(y);
            max_val = max_val.max(y);
        }
        eprintln!(
            "Gain={:.1}: peak={:.4} min={:.4} max={:.4}",
            gain, peak, min_val, max_val
        );
    }

    // Check opamp gain at different pot settings
    eprintln!("\n--- Opamp gain investigation ---");
    for gain_knob in [0.0, 0.5, 1.0] {
        let mut engine = pedalkernel::compiler::compile_pedal(&pedal, sr).expect("compile");
        engine.set_control("Gain", gain_knob);
        for _ in 0..512 {
            engine.process(0.0);
        }
        for (si, g, pot_id) in engine.opamp_debug_info() {
            eprintln!(
                "  gain_knob={:.1} stage[{}]: opamp_gain={:.2} feedback_pot={:?}",
                gain_knob, si, g, pot_id
            );
        }
    }

    // Check multi-NL scattering coefficients
    eprintln!("\n--- Multi-NL stage scattering ---");
    {
        let engine = pedalkernel::compiler::compile_pedal(&pedal, sr).expect("compile");
        for (i, n_nl, s_adapted, comp, port_r) in engine.multi_nl_debug_info() {
            eprintln!(
                "  mnl[{}]: n_nl={} compensation={:.4} s_nl_adapted={:.6?} port_R={:.1?}",
                i, n_nl, comp, s_adapted, port_r
            );
        }
    }

    // Check with higher amplitude to see if clipping behavior changes
    eprintln!("\n--- Amplitude sweep at Gain=1.0 ---");
    for amp in [0.01, 0.05, 0.1, 0.2, 0.5, 1.0] {
        let mut engine = pedalkernel::compiler::compile_pedal(&pedal, sr).expect("compile");
        engine.set_control("Gain", 1.0);
        engine.set_control("Output", 0.7);
        engine.set_control("Treble", 0.5);
        for i in 0..8192 {
            let x = amp * (2.0 * PI * freq * i as f64 / sr).sin();
            engine.process(x);
        }
        let mut out = vec![0.0f64; n_samples];
        for i in 0..n_samples {
            let x = amp * (2.0 * PI * freq * (i as f64 + 8192.0) / sr).sin();
            out[i] = engine.process(x);
        }
        let rms = (out.iter().map(|x| x * x).sum::<f64>() / n_samples as f64).sqrt();
        let peak = out.iter().map(|x| x.abs()).fold(0.0f64, f64::max);
        let bin = (freq * n_samples as f64 / sr).round() as usize;
        let mut h1_pow = 0.0;
        let mut harm_pow = 0.0;
        for h in 1..=8 {
            let b = bin * h;
            if b >= n_samples / 2 {
                break;
            }
            let (mut re, mut im) = (0.0, 0.0);
            for (i, &s) in out.iter().enumerate() {
                let a = 2.0 * PI * b as f64 * i as f64 / n_samples as f64;
                re += s * a.cos();
                im += s * a.sin();
            }
            let mag = (re * re + im * im).sqrt() / n_samples as f64 * 2.0;
            if h == 1 {
                h1_pow = mag * mag;
            } else {
                harm_pow += mag * mag;
            }
        }
        let thd = if h1_pow > 0.0 {
            (harm_pow / h1_pow).sqrt() * 100.0
        } else {
            0.0
        };
        eprintln!(
            "amp={:.2}: rms={:.4e} peak={:.4e} THD={:.1}%",
            amp, rms, peak, thd
        );
    }

    // Check v_prev (NR converged voltage) at diode junction
    eprintln!("\n--- NR solver converged voltages ---");
    for gain_knob in [0.0, 0.5, 1.0] {
        let mut engine = pedalkernel::compiler::compile_pedal(&pedal, sr).expect("compile");
        engine.set_control("Gain", gain_knob);
        engine.set_control("Output", 0.7);
        engine.set_control("Treble", 0.5);
        // Process and capture peak NR voltage
        let mut max_v = 0.0f64;
        for i in 0..8192 {
            let x = amplitude * (2.0 * PI * freq * i as f64 / sr).sin();
            engine.process(x);
        }
        // Read v_prev from multi-NL stages
        for (i, n_nl, _s, _c, _r) in engine.multi_nl_debug_info() {
            eprintln!("  gain_knob={:.1} mnl[{}]: n_nl={}", gain_knob, i, n_nl);
        }
    }

    // Also dump at gain=0.0
    eprintln!("\n--- Engine pot state at Gain=0.0 ---");
    let mut engine2 = pedalkernel::compiler::compile_pedal(&pedal, sr).expect("compile");
    engine2.set_control("Gain", 0.0);
    for _ in 0..1024 {
        engine2.process(0.0);
    }
    for (label, target, smoothed) in engine2.control_debug_info() {
        eprintln!("  {}: target={:.4} val={:.4}", label, target, smoothed);
    }

    // THD should increase with gain
    let thd_low = thd_values[0];
    let thd_high = thd_values[4];
    eprintln!(
        "\nTHD at gain=0: {:.1}%, THD at gain=1: {:.1}%",
        thd_low, thd_high
    );
    assert!(
        thd_high > thd_low * 1.5,
        "Gain pot should increase THD: low={:.1}%, high={:.1}%",
        thd_low,
        thd_high
    );
}

#[test]
fn goldenrod_fft_spectrum() {
    let pedal = pedalkernel::dsl::parse_pedal_file(GOLDENROD).expect("parse");
    let sr = 48000.0;
    let freq = 440.0;
    let amplitude = 0.03;
    let n_samples = 8192;

    for gain in [0.0, 0.5, 1.0] {
        let mut engine = pedalkernel::compiler::compile_pedal(&pedal, sr).expect("compile");
        engine.set_control("Gain", gain);
        engine.set_control("Output", 0.7);
        engine.set_control("Treble", 0.5);

        // Settle
        for i in 0..8192 {
            let x = amplitude * (2.0 * PI * freq * i as f64 / sr).sin();
            engine.process(x);
        }

        // Capture
        let mut output = vec![0.0f64; n_samples];
        for i in 0..n_samples {
            let x = amplitude * (2.0 * PI * freq * (i as f64 + 8192.0) / sr).sin();
            output[i] = engine.process(x);
        }

        // DFT at all harmonics up to Nyquist
        let fundamental_bin = (freq * n_samples as f64 / sr).round() as usize;
        let max_harmonic = (sr / 2.0 / freq).floor() as usize;

        eprintln!(
            "\n=== Gain={:.1} — Harmonic spectrum (440Hz fundamental) ===",
            gain
        );
        eprintln!("{:<6} {:<10} {:<10} {:<10}", "H#", "Freq(Hz)", "Mag", "dB");

        let mut noise_floor = 0.0f64;
        let mut harmonics_above_noise = 0usize;
        let mut total_harmonic_power = 0.0f64;
        let mut fundamental_mag = 0.0f64;

        for h in 1..=max_harmonic.min(20) {
            let bin = fundamental_bin * h;
            if bin >= n_samples / 2 {
                break;
            }

            let mut re = 0.0;
            let mut im = 0.0;
            for (i, &s) in output.iter().enumerate() {
                let angle = 2.0 * PI * bin as f64 * i as f64 / n_samples as f64;
                re += s * angle.cos();
                im += s * angle.sin();
            }
            let mag = (re * re + im * im).sqrt() / n_samples as f64 * 2.0;
            let db = if mag > 1e-12 {
                20.0 * (mag).log10()
            } else {
                -240.0
            };

            if h == 1 {
                fundamental_mag = mag;
            } else {
                total_harmonic_power += mag * mag;
            }

            // Count harmonics above -60dB relative to fundamental
            let rel_db = if fundamental_mag > 1e-12 {
                20.0 * (mag / fundamental_mag).log10()
            } else {
                -240.0
            };
            if h > 1 && rel_db > -60.0 {
                harmonics_above_noise += 1;
            }

            if mag > 1e-6 {
                eprintln!(
                    "{:<6} {:<10.0} {:<10.4e} {:<10.1}",
                    format!("H{}", h),
                    h as f64 * freq,
                    mag,
                    db
                );
            }
        }

        let thd = if fundamental_mag > 0.0 {
            (total_harmonic_power / (fundamental_mag * fundamental_mag)).sqrt() * 100.0
        } else {
            0.0
        };

        eprintln!(
            "--- Summary: {} harmonics above -60dB, THD={:.1}% ---",
            harmonics_above_noise, thd
        );
    }
}

#[test]
fn goldenrod_gain_at_different_input_levels() {
    let pedal = pedalkernel::dsl::parse_pedal_file(GOLDENROD).expect("parse");
    let sr = 48000.0;
    let freq = 440.0;
    let n_samples = 4096;

    eprintln!("\n=== Input level vs Gain pot THD response ===");
    eprintln!(
        "{:<12} {:<12} {:<12} {:<10}",
        "Input(dBFS)", "Gain=0 THD", "Gain=1 THD", "Ratio"
    );

    for db in [-40.0, -30.0, -20.0, -12.0, -6.0, 0.0] {
        let amplitude = 10.0_f64.powf(db / 20.0);

        let mut thd_at = |gain: f64| -> f64 {
            let mut engine = pedalkernel::compiler::compile_pedal(&pedal, sr).expect("compile");
            engine.set_control("Gain", gain);
            engine.set_control("Output", 0.7);
            engine.set_control("Treble", 0.5);
            for i in 0..8192 {
                let x = amplitude * (2.0 * PI * freq * i as f64 / sr).sin();
                engine.process(x);
            }
            let mut output = vec![0.0f64; n_samples];
            for i in 0..n_samples {
                let x = amplitude * (2.0 * PI * freq * (i as f64 + 8192.0) / sr).sin();
                output[i] = engine.process(x);
            }
            let bin = (freq * n_samples as f64 / sr).round() as usize;
            let mut h1_pow = 0.0;
            let mut harm_pow = 0.0;
            for h in 1..=8 {
                let b = bin * h;
                if b >= n_samples / 2 {
                    break;
                }
                let (mut re, mut im) = (0.0, 0.0);
                for (i, &s) in output.iter().enumerate() {
                    let a = 2.0 * PI * b as f64 * i as f64 / n_samples as f64;
                    re += s * a.cos();
                    im += s * a.sin();
                }
                let mag = (re * re + im * im).sqrt() / n_samples as f64 * 2.0;
                if h == 1 {
                    h1_pow = mag * mag;
                } else {
                    harm_pow += mag * mag;
                }
            }
            if h1_pow > 0.0 {
                (harm_pow / h1_pow).sqrt() * 100.0
            } else {
                0.0
            }
        };

        let thd_low = thd_at(0.0);
        let thd_high = thd_at(1.0);
        let ratio = if thd_low > 0.1 {
            thd_high / thd_low
        } else {
            f64::INFINITY
        };
        eprintln!(
            "{:<12.0} {:<12.1}% {:<12.1}% {:<10.1}x",
            db, thd_low, thd_high, ratio
        );
    }
}

#[test]
fn goldenrod_calibrated_pre_gain() {
    let pedal = pedalkernel::dsl::parse_pedal_file(GOLDENROD).expect("parse");
    let sr = 48000.0;
    let engine = pedalkernel::compiler::compile_pedal(&pedal, sr).expect("compile");

    // pre_gain and output_gain are pub(super) — check via control_debug_info proxy
    // Instead, test the actual behavior: send 0dBFS and measure THD
    let freq = 440.0;
    let n_samples = 4096;

    let mut thd_at = |gain_knob: f64| -> f64 {
        let mut e = pedalkernel::compiler::compile_pedal(&pedal, sr).expect("compile");
        e.set_control("Gain", gain_knob);
        e.set_control("Output", 0.7);
        e.set_control("Treble", 0.5);
        for i in 0..8192 {
            // 0dBFS input — the calibration should attenuate internally
            let x = 1.0 * (2.0 * PI * freq * i as f64 / sr).sin();
            e.process(x);
        }
        let mut output = vec![0.0f64; n_samples];
        for i in 0..n_samples {
            let x = 1.0 * (2.0 * PI * freq * (i as f64 + 8192.0) / sr).sin();
            output[i] = e.process(x);
        }
        let bin = (freq * n_samples as f64 / sr).round() as usize;
        let mut h1_pow = 0.0;
        let mut harm_pow = 0.0;
        for h in 1..=8 {
            let b = bin * h;
            if b >= n_samples / 2 {
                break;
            }
            let (mut re, mut im) = (0.0, 0.0);
            for (i, &s) in output.iter().enumerate() {
                let a = 2.0 * PI * b as f64 * i as f64 / n_samples as f64;
                re += s * a.cos();
                im += s * a.sin();
            }
            let mag = (re * re + im * im).sqrt() / n_samples as f64 * 2.0;
            if h == 1 {
                h1_pow = mag * mag;
            } else {
                harm_pow += mag * mag;
            }
        }
        if h1_pow > 0.0 {
            (harm_pow / h1_pow).sqrt() * 100.0
        } else {
            0.0
        }
    };

    let thd_low = thd_at(0.0);
    let thd_high = thd_at(1.0);
    eprintln!(
        "Calibrated @ 0dBFS: Gain=0 THD={:.1}%, Gain=1 THD={:.1}%, ratio={:.1}x",
        thd_low,
        thd_high,
        thd_high / thd_low.max(0.1)
    );

    assert!(
        thd_high > thd_low * 2.0,
        "After calibration, gain pot should still affect THD at 0dBFS: low={:.1}% high={:.1}%",
        thd_low,
        thd_high
    );
}

#[test]
fn goldenrod_treble_topology_debug() {
    let src = GOLDENROD;
    let pedal = pedalkernel::dsl::parse_pedal_file(src).expect("parse");
    let sr = 48000.0;

    // Compile and check what opamp stages were created
    let engine = pedalkernel::compiler::compile_pedal(&pedal, sr).expect("compile");

    eprintln!("=== Opamp stages ===");
    for (si, g, pot_id) in engine.opamp_debug_info() {
        eprintln!("  stage[{}]: gain={:.2} feedback_pot={:?}", si, g, pot_id);
    }

    // The treble pot should be a feedback pot on one of the stages
    let has_treble = engine
        .opamp_debug_info()
        .iter()
        .any(|(_, _, pot)| pot.as_ref().map_or(false, |p| p.contains("Treble")));
    eprintln!("Has Treble feedback pot: {}", has_treble);
    assert!(
        has_treble,
        "Treble pot should be detected as a feedback pot on U4's stage"
    );
}

#[test]
fn goldenrod_treble_changes_spectrum() {
    let pedal = pedalkernel::dsl::parse_pedal_file(GOLDENROD).expect("parse");
    let sr = 48000.0;
    let freq = 10000.0; // 10kHz — where shelving EQ should be most visible
    let amplitude = 1.0;
    let n_samples = 4096;

    let rms_at = |treble: f64| -> f64 {
        let mut engine = pedalkernel::compiler::compile_pedal(&pedal, sr).expect("compile");
        engine.set_control("Gain", 0.5);
        engine.set_control("Treble", treble);
        engine.set_control("Output", 0.7);
        for i in 0..8192 {
            let x = amplitude * (2.0 * PI * freq * i as f64 / sr).sin();
            engine.process(x);
        }
        let mut sum_sq = 0.0;
        for i in 0..n_samples {
            let x = amplitude * (2.0 * PI * freq * (i as f64 + 8192.0) / sr).sin();
            let y = engine.process(x);
            sum_sq += y * y;
        }
        (sum_sq / n_samples as f64).sqrt()
    };

    let dark = rms_at(0.0);
    let bright = rms_at(1.0);
    let ratio_db = 20.0 * (bright / dark).log10();
    eprintln!(
        "Treble sweep at 10kHz: dark={:.6}, bright={:.6}, ratio={:.3}x, dB={:.2}",
        dark,
        bright,
        bright / dark,
        ratio_db
    );
    assert!(
        ratio_db.abs() > 1.0,
        "Treble should affect 10kHz by ≥1dB, got {:.2}dB",
        ratio_db
    );
}

#[test]
fn goldenrod_tone_iir_unit_test() {
    // Directly test ToneFeedback IIR with known parameters
    // Klon: rf=1.8k, ri=1.8k, c_tone=3.9nF, r_shelf=4.7k, max_pot_r=10k
    use pedalkernel::compiler::stage_test_helpers::ToneFeedback;

    let sr = 48000.0;
    let freq = 10000.0;
    let n = 4096;

    for pot_pos in [0.0, 0.5, 1.0] {
        let mut tf = ToneFeedback::new(
            1800.0,
            1800.0,
            3.9e-9,
            4700.0,
            10000.0,
            "Treble".into(),
            sr,
            pot_pos,
        );

        // Warm up
        for i in 0..1024 {
            let x = 0.1 * (2.0 * PI * freq * i as f64 / sr).sin();
            tf.process(x);
        }
        // Measure
        let mut sum_sq = 0.0;
        for i in 0..n {
            let x = 0.1 * (2.0 * PI * freq * (i + 1024) as f64 / sr).sin();
            let y = tf.process(x);
            sum_sq += y * y;
        }
        let rms = (sum_sq / n as f64).sqrt();
        eprintln!(
            "ToneFeedback pot={:.1}: rms={:.6} b0={:.6} b1={:.6} a1={:.6} dc_gain={:.6}",
            pot_pos, rms, tf.b0, tf.b1, tf.a1, tf.dc_gain
        );
    }
}
