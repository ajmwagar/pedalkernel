//! Probe: BA283 output level + per-device op-point vs time, to characterize servo.
#[cfg(feature = "diag")]
#[test]
fn ba283_servo_drift() {
    use pedalkernel::compiler::compile_pedal;
    use pedalkernel::dsl::parse_pedal_file;
    use pedalkernel::PedalProcessor;
    use pedalkernel_rt::diag_ring::OpPointRecord;
    use pedalkernel_rt::processor::Stage;
    use pedalkernel_validate::pro_pedal::load_pro_pedal_sub;
    use pedalkernel_validate::skip_if_missing;
    use std::f64::consts::PI;

    const SR: f64 = 48_000.0;
    const AMP: f64 = 0.1;
    const FREQ: f64 = 1_000.0;
    let source = skip_if_missing!(load_pro_pedal_sub("pedals/500/1073/sub/ba283.pedal"), "ba283");
    let def = parse_pedal_file(&source).unwrap();
    let mut proc = compile_pedal(&def, SR).unwrap();
    proc.set_control("Gain", 0.5);

    let total = 96_000usize; // 2 seconds
    let mut win: Vec<f64> = Vec::new();
    for i in 0..total {
        let x = AMP * (2.0 * PI * FREQ * i as f64 / SR).sin();
        let y = proc.process(x);
        win.push(y);
        if (i + 1) % 4000 == 0 {
            let rms = (win.iter().map(|v| v * v).sum::<f64>() / win.len() as f64).sqrt();
            let pk = win.iter().fold(0.0_f64, |a, &v| a.max(v.abs()));
            let mut recs = [OpPointRecord::default(); 8];
            let mut n = 0usize;
            for (si, st) in proc.stages.iter().enumerate() {
                if let Stage::MultiNl(mnl) = st {
                    n += mnl.runtime_op_points(si, &mut recs[n..]);
                }
            }
            let mut devs = String::new();
            for r in &recs[..n] {
                devs.push_str(&format!(
                    " [{}/{} Vbe={:.3} Ic={:.2e}]",
                    r.ref_str(),
                    r.type_str(),
                    r.v_be,
                    r.i_c
                ));
            }
            eprintln!(
                "  t={:>6} ({:.2}s) RMS={:.4}V pk={:.4}V{}",
                i + 1,
                (i + 1) as f64 / SR,
                rms,
                pk,
                devs
            );
            win.clear();
        }
    }
}
