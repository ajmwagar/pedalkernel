// Compare Ebers-Moll vs Gummel-Poon BJT models
// Run with: cargo run --example bjt_model_compare

#[allow(deprecated)]
use pedalkernel::elements::{BjtModel, BjtNpnRoot, GummelPoonModel, WdfRoot};

fn main() {
    println!("=== BJT Model Comparison: Ebers-Moll vs Gummel-Poon ===\n");

    let sample_rate = 96000.0;
    let rp = 10000.0; // 10k port resistance (typical collector load)

    // Test at various Vbe points
    let vbe_points = [0.5, 0.55, 0.6, 0.65, 0.7, 0.75];

    println!("2N3904 NPN - Collector Current vs Vbe");
    println!(
        "{:>6} | {:>12} | {:>12} | {:>8}",
        "Vbe", "Ebers-Moll", "Gummel-Poon", "Ratio"
    );
    println!("{:-<6}-+-{:-<12}-+-{:-<12}-+-{:-<8}", "", "", "", "");

    let gp_model = GummelPoonModel::by_name("2N3904");

    for vbe in vbe_points {
        // Ebers-Moll (via BjtNpnRoot WDF root — deprecated, Phase 2 will replace)
        #[allow(deprecated)]
        let ic_em = {
            let em_model = BjtModel::by_name("2N3904");
            let mut em_bjt = BjtNpnRoot::new(em_model);
            em_bjt.set_vbe(vbe);
            em_bjt.set_v_max(9.0);
            let a = 4.5; // ~Vcc/2 incident wave
            let _ = em_bjt.process(a, rp);
            em_bjt.collector_current()
        };

        // Gummel-Poon (direct model evaluation at typical Vce=4.5V)
        let vce = 4.5;
        let vbc = vbe - vce;
        let (ic_gp, _) = gp_model.currents(vbe, vbc);

        let ratio = if ic_em.abs() > 1e-15 {
            ic_gp / ic_em
        } else {
            0.0
        };
        println!(
            "{:>6.2} | {:>12.6} | {:>12.6} | {:>8.3}",
            vbe,
            ic_em * 1000.0, // mA
            ic_gp * 1000.0, // mA
            ratio
        );
    }

    println!("\n(Ic in mA)\n");

    // Dynamic test - process sine wave through Ebers-Moll root
    println!("=== Dynamic Response Test (Ebers-Moll WDF root) ===\n");
    println!("Processing 1kHz sine through Ebers-Moll model...\n");

    let freq = 1000.0;
    let num_samples = (sample_rate / freq * 10.0) as usize; // 10 cycles

    #[allow(deprecated)]
    let em_rms = {
        let em_model = BjtModel::by_name("2N3904");
        let mut em_bjt = BjtNpnRoot::new(em_model);
        em_bjt.set_v_max(9.0);

        let mut rms = 0.0;
        let mut _em_max = 0.0f64;

        for i in 0..num_samples {
            let t = i as f64 / sample_rate;
            let vbe = 0.65 + 0.02 * (2.0 * std::f64::consts::PI * freq * t).sin();
            em_bjt.set_vbe(vbe);
            let a = 4.5;
            let b_em = em_bjt.process(a, rp);
            let v_em = (a + b_em) / 2.0;
            rms += v_em * v_em;
            _em_max = _em_max.max(v_em.abs());
        }
        (rms / num_samples as f64).sqrt()
    };

    let _ = sample_rate; // used above

    println!("Output Vce Statistics:");
    println!("  Ebers-Moll:   RMS={em_rms:.4}V");

    // Base charge comparison
    println!("\n=== Gummel-Poon Base Charge (Qb) ===\n");
    println!("Qb models high-injection effects and Early effect.");
    println!("When Qb > 1, effective beta drops.\n");

    println!("{:>6} | {:>6} | {:>8}", "Vbe", "Vbc", "Qb");
    println!("{:-<6}-+-{:-<6}-+-{:-<8}", "", "", "");

    for vbe in [0.5, 0.6, 0.7, 0.8] {
        for vbc in [-5.0, -2.0, 0.0] {
            let qb = gp_model.base_charge(vbe, vbc);
            println!("{vbe:>6.2} | {vbc:>6.1} | {qb:>8.4}");
        }
    }

    println!("\n=== Summary ===");
    println!("- Ebers-Moll: Simpler, 4 params (Is, Bf, Vt, Va)");
    println!("- Gummel-Poon: Full SPICE model, ~15 params");
    println!("  - Base charge modulation (Qb)");
    println!("  - High-injection effects (Ikf, Ikr)");
    println!("  - Junction capacitances (Cje, Cjc)");
    println!("  - Better match to physical transistor behavior");
}
