// Gummel-Poon BJT model showcase.
// Run with: cargo run --example bjt_model_compare

use pedalkernel::model_lookup::bjt_model_by_name;

fn main() {
    println!("=== Gummel-Poon BJT Model ===\n");

    let rp = 10000.0; // 10k port resistance (typical collector load)

    // Test at various Vbe points
    let vbe_points = [0.5, 0.55, 0.6, 0.65, 0.7, 0.75];

    println!("2N3904 NPN - Collector Current vs Vbe");
    println!("{:>6} | {:>12}", "Vbe", "Ic (mA)");
    println!("{:-<6}-+-{:-<12}", "", "");

    let gp_model = bjt_model_by_name("2N3904");

    for vbe in vbe_points {
        // Evaluate Gummel-Poon model at typical Vce = 4.5 V
        let vce = 4.5;
        let vbc = vbe - vce;
        let (ic_gp, _) = gp_model.currents(vbe, vbc);
        println!("{:>6.2} | {:>12.6}", vbe, ic_gp * 1000.0);
    }

    println!("\n(Ic in mA)\n");

    // ── Base charge comparison ──────────────────────────────────────────────
    println!("=== Gummel-Poon Base Charge (Qb) ===\n");
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

    // ── Dynamic Ic sweep via Gummel-Poon direct model evaluation ─────────────
    println!("\n=== Dynamic Response: Ic vs modulated Vbe ===\n");
    let sample_rate = 96000.0;
    let freq = 1000.0;
    let num_samples = (sample_rate / freq * 10.0) as usize;
    let vce = 4.5;

    let mut rms = 0.0;
    for i in 0..num_samples {
        let t = i as f64 / sample_rate;
        let vbe = 0.65 + 0.02 * (2.0 * std::f64::consts::PI * freq * t).sin();
        let vbc = vbe - vce;
        let (ic, _ib) = gp_model.currents(vbe, vbc);
        rms += ic * ic;
    }
    let rms = (rms / num_samples as f64).sqrt();
    let _ = rp; // kept for context
    println!(
        "Ic RMS over 10 cycles of 1 kHz at Vce={vce} V: {:.4} mA",
        rms * 1000.0
    );

    println!("\n=== Summary ===");
    println!("- Gummel-Poon: Full SPICE model (~15 params)");
    println!("  - Base charge modulation (Qb)");
    println!("  - High-injection effects (Ikf, Ikr)");
    println!("  - Junction capacitances (Cje, Cjc)");
    println!("  - Two-port solver (BjtTwoPort) via grouped NR loop");
}
