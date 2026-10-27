//! Integration tests for speaker cabinet modeling (.cab DSL → WDF → IR export).

use pedalkernel::cab::{compile_cab, ir_export, parse_cab_file, validate_cab, IrExportConfig};
use pedalkernel::PedalProcessor;
use std::f64::consts::PI;

// ═══════════════════════════════════════════════════════════════════════════
// Parsing
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn parse_fender_deluxe() {
    let src = include_str!("../examples/cabs/fender_deluxe_1x12.cab");
    let cab = parse_cab_file(src).expect("Fender Deluxe should parse");
    assert_eq!(cab.name, "Fender Deluxe 1x12");
    assert_eq!(cab.drivers.len(), 1);
    assert_eq!(cab.mics.len(), 2);
}

#[test]
fn parse_marshall_1960a() {
    let src = include_str!("../examples/cabs/marshall_1960a.cab");
    let cab = parse_cab_file(src).expect("Marshall 1960A should parse");
    assert_eq!(cab.name, "Marshall 1960A");
    assert_eq!(cab.drivers.len(), 1);
    // Has copies: 4, but still one driver definition
}

#[test]
fn parse_mesa_recto() {
    let src = include_str!("../examples/cabs/mesa_recto_4x12.cab");
    let cab = parse_cab_file(src).expect("Mesa Recto should parse");
    assert_eq!(cab.name, "Mesa Recto 4x12");
}

#[test]
fn parse_vox_ac30() {
    let src = include_str!("../examples/cabs/vox_ac30_2x12.cab");
    let cab = parse_cab_file(src).expect("Vox AC30 should parse");
    assert_eq!(cab.name, "Vox AC30 2x12");
}

// ═══════════════════════════════════════════════════════════════════════════
// Validation
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn validate_all_examples() {
    let examples = [
        include_str!("../examples/cabs/fender_deluxe_1x12.cab"),
        include_str!("../examples/cabs/marshall_1960a.cab"),
        include_str!("../examples/cabs/mesa_recto_4x12.cab"),
        include_str!("../examples/cabs/vox_ac30_2x12.cab"),
    ];

    for (i, src) in examples.iter().enumerate() {
        let mut cab = parse_cab_file(src).unwrap_or_else(|e| panic!("Example {i} parse failed: {e}"));
        let warnings = validate_cab(&mut cab).unwrap_or_else(|e| panic!("Example {i} validation failed: {e}"));

        // Check that Fs was derived
        let driver = &cab.drivers[0];
        assert!(driver.fs.is_some(), "Example {i}: Fs should be derived");
        assert!(driver.qts.is_some(), "Example {i}: Qts should be derived");
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Compilation
// ═══════════════════════════════════════════════════════════════════════════

fn compile_example(src: &str) -> Box<dyn PedalProcessor> {
    let cab = parse_cab_file(src).unwrap();
    Box::new(compile_cab(&cab, 48000.0).unwrap())
}

#[test]
fn compile_sealed_cab() {
    let src = r#"
        cab "Test Sealed" {
            driver { preset: v30 }
            enclosure { type: sealed, volume: 45L }
        }
    "#;
    let _proc = compile_example(src);
}

#[test]
fn compile_open_back_cab() {
    let src = r#"
        cab "Test Open" {
            driver { preset: c12n }
            enclosure { type: open_back, volume: 42L, open_area: 80%, depth: 23cm }
        }
    "#;
    let _proc = compile_example(src);
}

#[test]
fn compile_infinite_baffle() {
    let src = r#"
        cab "Test Baffle" {
            driver { preset: g12_blue }
            enclosure { type: infinite_baffle }
        }
    "#;
    let _proc = compile_example(src);
}

// ═══════════════════════════════════════════════════════════════════════════
// Signal health
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn cab_signal_passes_through() {
    let src = r#"
        cab "Test" {
            driver { preset: v30 }
            enclosure { type: sealed, volume: 45L }
        }
    "#;
    let cab = parse_cab_file(src).unwrap();
    let mut proc = compile_cab(&cab, 48000.0).unwrap();

    // Process a 440 Hz sine for 100ms
    let mut max_out = 0.0f64;
    let mut finite_count = 0usize;
    let n = 4800;
    for i in 0..n {
        let t = i as f64 / 48000.0;
        let input = 0.5 * (2.0 * PI * 440.0 * t).sin();
        let output = proc.process(input);
        if output.is_finite() {
            max_out = max_out.max(output.abs());
            finite_count += 1;
        }
    }

    assert_eq!(finite_count, n, "All samples should be finite");
    assert!(max_out > 1e-6, "Output should be non-zero, got {max_out}");
    assert!(max_out < 100.0, "Output should be bounded, got {max_out}");
}

#[test]
fn cab_no_nan_or_inf() {
    let src = r#"
        cab "NaN Test" {
            driver { preset: g12m_greenback }
            enclosure { type: sealed, volume: 70L }
        }
    "#;
    let cab = parse_cab_file(src).unwrap();
    let mut proc = compile_cab(&cab, 48000.0).unwrap();

    // Feed various signals including silence, impulse, and loud signal
    let signals: Vec<f64> = (0..48000)
        .map(|i| {
            let t = i as f64 / 48000.0;
            if i == 0 {
                1.0 // impulse
            } else if i < 1000 {
                0.0 // silence
            } else {
                0.5 * (2.0 * PI * 200.0 * t).sin() // sine
            }
        })
        .collect();

    for (i, &s) in signals.iter().enumerate() {
        let out = proc.process(s);
        assert!(
            out.is_finite(),
            "Output at sample {i} is not finite: {out}"
        );
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// IR export
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn ir_export_produces_wav() {
    let src = r#"
        cab "IR Test" {
            driver { preset: v30 }
            enclosure { type: sealed, volume: 45L }
        }
    "#;
    let cab = parse_cab_file(src).unwrap();
    let mut proc = compile_cab(&cab, 48000.0).unwrap();

    let config = IrExportConfig {
        sample_rate: 48_000,
        length: 4800, // 100ms
        bit_depth: 32,
        normalize: true,
        fade_out: 480,
    };

    let ir = ir_export::capture_ir(&mut proc, &config);
    assert_eq!(ir.len(), 4800);

    // Normalized peak should be 1.0
    let peak = ir.iter().map(|s| s.abs()).fold(0.0f64, f64::max);
    assert!(
        (peak - 1.0).abs() < 0.01,
        "Normalized IR peak should be ~1.0, got {peak}"
    );

    // Write to temp file
    let tmp = std::env::temp_dir().join("pedalkernel_cab_ir_test.wav");
    ir_export::export_ir_wav(&ir, &tmp, &config).unwrap();

    let reader = hound::WavReader::open(&tmp).unwrap();
    assert_eq!(reader.spec().sample_rate, 48_000);
    assert_eq!(reader.len(), 4800);
    let _ = std::fs::remove_file(&tmp);
}

// ═══════════════════════════════════════════════════════════════════════════
// T-S parameter validation
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn ts_parameters_physically_plausible() {
    let src = r#"
        cab "T-S Test" {
            driver { preset: v30 }
            enclosure { type: sealed, volume: 45L }
        }
    "#;
    let mut cab = parse_cab_file(src).unwrap();
    let warnings = validate_cab(&mut cab).unwrap();

    let d = &cab.drivers[0];
    let fs = d.fs.unwrap();
    let qts = d.qts.unwrap();
    let spl = d.spl.unwrap();

    // V30: Fs ~75 Hz, Qts ~0.78, SPL ~95-100 dB
    assert!(fs > 60.0 && fs < 90.0, "V30 Fs={fs:.0} should be 60-90 Hz");
    assert!(qts > 0.5 && qts < 1.2, "V30 Qts={qts:.2} should be 0.5-1.2");
    assert!(spl > 85.0 && spl < 105.0, "V30 SPL={spl:.1} should be 85-105 dB");

    // Should have no error-level warnings for a preset speaker
    let errors: Vec<_> = warnings
        .iter()
        .filter(|w| matches!(w.severity, pedalkernel::cab::CabWarningSeverity::Error))
        .collect();
    assert!(errors.is_empty(), "Should have no errors: {errors:?}");
}
