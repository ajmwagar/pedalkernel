//! CLI subcommand for listing and searching available component models.

use pedalkernel::models::{
    bjt_by_name, jfet_by_name, opamp_by_name, pentode_by_name, transformer_by_name, triode_by_name,
    BJT_MODELS, JFET_MODELS, OPAMP_MODELS, PENTODE_MODELS, TRANSFORMER_MODELS, TRIODE_MODELS,
};

/// Run the `models` subcommand.
pub fn run(type_filter: Option<&str>, search: Option<&str>) {
    let search_upper = search.map(|s| s.to_uppercase());

    match type_filter.map(|s| s.to_lowercase()).as_deref() {
        Some("bjt") | Some("npn") | Some("pnp") => {
            print_bjts(search_upper.as_deref(), type_filter);
        }
        Some("jfet") | Some("njf") | Some("pjf") => {
            print_jfets(search_upper.as_deref(), type_filter);
        }
        Some("triode") => {
            print_triodes(search_upper.as_deref());
        }
        Some("pentode") => {
            print_pentodes(search_upper.as_deref());
        }
        Some("opamp") | Some("op-amp") | Some("ota") => {
            print_opamps(search_upper.as_deref(), type_filter);
        }
        Some("transformer") | Some("xfmr") => {
            print_transformers(search_upper.as_deref());
        }
        Some(other) => {
            eprintln!("Unknown type filter: '{other}'");
            eprintln!(
                "Available types: bjt, npn, pnp, jfet, njf, pjf, triode, pentode, opamp, ota, transformer"
            );
            std::process::exit(1);
        }
        None => {
            // Show all types
            print_bjts(search_upper.as_deref(), None);
            println!();
            print_jfets(search_upper.as_deref(), None);
            println!();
            print_triodes(search_upper.as_deref());
            println!();
            print_pentodes(search_upper.as_deref());
            println!();
            print_opamps(search_upper.as_deref(), None);
            println!();
            print_transformers(search_upper.as_deref());
        }
    }
}

fn print_bjts(search: Option<&str>, polarity_filter: Option<&str>) {
    let mut models: Vec<_> = BJT_MODELS.values().collect();
    models.sort_by(|a, b| a.name.cmp(&b.name));

    // Filter by polarity if requested
    if let Some(pf) = polarity_filter {
        let pf = pf.to_lowercase();
        if pf == "npn" {
            models.retain(|m| !m.is_pnp);
        } else if pf == "pnp" {
            models.retain(|m| m.is_pnp);
        }
    }

    // Filter by search term
    if let Some(term) = search {
        models.retain(|m| m.name.contains(term));
    }

    if models.is_empty() {
        println!("No BJT models found.");
        return;
    }

    println!("BJT Models ({} found)", models.len());
    println!("{:-<78}", "");
    println!(
        "{:<20} {:>5} {:>10} {:>10} {:>10} {:>10}",
        "Name", "Type", "IS (A)", "BF", "VAF (V)", "NF"
    );
    println!("{:-<78}", "");

    for m in &models {
        let polarity = if m.is_pnp { "PNP" } else { "NPN" };
        let vaf = if m.vaf.is_infinite() {
            "inf".to_string()
        } else {
            format!("{:.1}", m.vaf)
        };
        println!(
            "{:<20} {:>5} {:>10.3e} {:>10.1} {:>10} {:>10.3}",
            m.name, polarity, m.is, m.bf, vaf, m.nf
        );
    }
}

fn print_jfets(search: Option<&str>, polarity_filter: Option<&str>) {
    let mut models: Vec<_> = JFET_MODELS.values().collect();
    models.sort_by(|a, b| a.name.cmp(&b.name));

    // Filter by polarity if requested
    if let Some(pf) = polarity_filter {
        let pf = pf.to_lowercase();
        if pf == "njf" {
            models.retain(|m| m.is_n_channel);
        } else if pf == "pjf" {
            models.retain(|m| !m.is_n_channel);
        }
    }

    // Filter by search term
    if let Some(term) = search {
        models.retain(|m| m.name.contains(term));
    }

    if models.is_empty() {
        println!("No JFET models found.");
        return;
    }

    println!("JFET Models ({} found)", models.len());
    println!("{:-<78}", "");
    println!(
        "{:<20} {:>5} {:>10} {:>10} {:>10} {:>10}",
        "Name", "Type", "VTO (V)", "BETA(A/V2)", "LAMBDA", "Idss (A)"
    );
    println!("{:-<78}", "");

    for m in &models {
        let polarity = if m.is_n_channel { "NJF" } else { "PJF" };
        let idss = m.beta * m.vto * m.vto;
        println!(
            "{:<20} {:>5} {:>10.3} {:>10.3e} {:>10.3e} {:>10.3e}",
            m.name, polarity, m.vto, m.beta, m.lambda, idss
        );
    }
}

fn print_triodes(search: Option<&str>) {
    let mut models: Vec<_> = TRIODE_MODELS.values().collect();
    models.sort_by(|a, b| a.name.cmp(&b.name));

    if let Some(term) = search {
        models.retain(|m| m.name.contains(term));
    }

    if models.is_empty() {
        println!("No triode models found.");
        return;
    }

    println!("Triode Models ({} found)", models.len());
    println!("{:-<78}", "");
    println!(
        "{:<20} {:>8} {:>8} {:>8} {:>8} {:>8}",
        "Name", "MU", "EX", "KG1", "KP", "KVB"
    );
    println!("{:-<78}", "");

    for m in &models {
        println!(
            "{:<20} {:>8.1} {:>8.2} {:>8.0} {:>8.0} {:>8.0}",
            m.name, m.mu, m.ex, m.kg1, m.kp, m.kvb
        );
    }
}

fn print_pentodes(search: Option<&str>) {
    let mut models: Vec<_> = PENTODE_MODELS.values().collect();
    models.sort_by(|a, b| a.name.cmp(&b.name));

    if let Some(term) = search {
        models.retain(|m| m.name.contains(term));
    }

    if models.is_empty() {
        println!("No pentode models found.");
        return;
    }

    println!("Pentode Models ({} found)", models.len());
    println!("{:-<88}", "");
    println!(
        "{:<12} {:>6} {:>6} {:>6} {:>6} {:>6} {:>6} {:>6} {:>6}",
        "Name", "MU", "EX", "KG1", "KG2", "KP", "KVB", "KVB2", "VG2"
    );
    println!("{:-<88}", "");

    for m in &models {
        println!(
            "{:<12} {:>6.1} {:>6.2} {:>6.0} {:>6.0} {:>6.0} {:>6.0} {:>6.0} {:>6.0}",
            m.name, m.mu, m.ex, m.kg1, m.kg2, m.kp, m.kvb, m.kvb2, m.vg2_default
        );
    }
}

fn print_opamps(search: Option<&str>, type_filter: Option<&str>) {
    let mut models: Vec<_> = OPAMP_MODELS.values().collect();
    models.sort_by(|a, b| a.name.cmp(&b.name));

    if let Some(tf) = type_filter {
        if tf.eq_ignore_ascii_case("ota") {
            models.retain(|m| m.is_ota);
        } else if tf.eq_ignore_ascii_case("opamp") || tf.eq_ignore_ascii_case("op-amp") {
            models.retain(|m| !m.is_ota);
        }
    }

    if let Some(term) = search {
        models.retain(|m| m.name.contains(term));
    }

    if models.is_empty() {
        println!("No op-amp models found.");
        return;
    }

    println!("Op-Amp Models ({} found)", models.len());
    println!("{:-<88}", "");
    println!(
        "{:<12} {:>6} {:>10} {:>10} {:>8} {:>8} {:>8} {:>10}",
        "Name", "Type", "A0", "GBW", "SR", "RO", "COUT", "GM"
    );
    println!("{:-<88}", "");

    for m in &models {
        println!(
            "{:<12} {:>6} {:>10.1} {:>10.3e} {:>8.2} {:>8.1} {:>8.3e} {:>10.3e}",
            m.name,
            if m.is_ota { "OTA" } else { "OPAMP" },
            m.open_loop_gain,
            m.gbw,
            m.slew_rate,
            m.output_impedance,
            m.output_capacitance,
            m.ota_gm
        );
    }
}

fn print_transformers(search: Option<&str>) {
    let mut models: Vec<_> = TRANSFORMER_MODELS.values().collect();
    models.sort_by(|a, b| a.name.cmp(&b.name));

    if let Some(term) = search {
        models.retain(|m| m.name.contains(term));
    }

    if models.is_empty() {
        println!("No transformer models found.");
        return;
    }

    println!("Transformer Models ({} found)", models.len());
    println!("{:-<96}", "");
    println!(
        "{:<18} {:>10} {:>8} {:>8} {:>8} {:>10} {:>10} {:>10}",
        "Name", "LP (H)", "K", "RP", "RS", "LLP (H)", "LLS (H)", "CP (F)"
    );
    println!("{:-<96}", "");

    for m in &models {
        println!(
            "{:<18} {:>10.3e} {:>8.4} {:>8.1} {:>8.1} {:>10.3e} {:>10.3e} {:>10.3e}",
            m.name,
            m.primary_inductance,
            m.coupling,
            m.primary_dcr,
            m.secondary_dcr,
            m.primary_leakage,
            m.secondary_leakage,
            m.capacitance
        );
    }
}

/// Print details for a specific model.
pub fn show(name: &str) {
    let name_upper = name.to_uppercase();

    if let Some(m) = bjt_by_name(&name_upper) {
        println!("BJT: {} ({})", m.name, if m.is_pnp { "PNP" } else { "NPN" });
        println!("{:-<50}", "");
        println!("  IS   = {:.4e} A   (saturation current)", m.is);
        println!("  BF   = {:.2}        (forward beta)", m.bf);
        println!("  BR   = {:.4}      (reverse beta)", m.br);
        println!("  NF   = {:.3}       (forward ideality)", m.nf);
        println!("  NR   = {:.3}       (reverse ideality)", m.nr);
        let vaf = if m.vaf.is_infinite() {
            "inf".to_string()
        } else {
            format!("{:.2}", m.vaf)
        };
        let var = if m.var.is_infinite() {
            "inf".to_string()
        } else {
            format!("{:.2}", m.var)
        };
        println!("  VAF  = {} V     (forward Early voltage)", vaf);
        println!("  VAR  = {} V     (reverse Early voltage)", var);
        let ikf = if m.ikf.is_infinite() {
            "inf".to_string()
        } else {
            format!("{:.4e}", m.ikf)
        };
        let ikr = if m.ikr.is_infinite() {
            "inf".to_string()
        } else {
            format!("{:.4e}", m.ikr)
        };
        println!("  IKF  = {} A  (high-injection knee)", ikf);
        println!("  IKR  = {} A  (reverse knee)", ikr);
        println!("  RB   = {:.2} Ohm    (base resistance)", m.rb);
        println!("  RE   = {:.2} Ohm    (emitter resistance)", m.re);
        println!("  RC   = {:.2} Ohm    (collector resistance)", m.rc);
        println!("  CJE  = {:.4e} F   (B-E capacitance)", m.cje);
        println!("  CJC  = {:.4e} F   (B-C capacitance)", m.cjc);
        println!("  TF   = {:.4e} s   (forward transit time)", m.tf);
        println!("  TR   = {:.4e} s   (reverse transit time)", m.tr);
        return;
    }

    if let Some(m) = jfet_by_name(&name_upper) {
        println!(
            "JFET: {} ({})",
            m.name,
            if m.is_n_channel {
                "N-channel"
            } else {
                "P-channel"
            }
        );
        println!("{:-<50}", "");
        println!("  VTO    = {:.3} V      (threshold voltage)", m.vto);
        println!("  BETA   = {:.4e} A/V^2 (transconductance coeff)", m.beta);
        println!("  LAMBDA = {:.4e} 1/V   (channel-length mod)", m.lambda);
        println!(
            "  Idss   = {:.4e} A     (= BETA * VTO^2)",
            m.beta * m.vto * m.vto
        );
        println!("  IS     = {:.4e} A     (gate junction sat current)", m.is);
        println!("  N      = {:.3}        (gate ideality factor)", m.n);
        println!("  RD     = {:.2} Ohm    (drain resistance)", m.rd);
        println!("  RS     = {:.2} Ohm    (source resistance)", m.rs);
        println!("  CGS    = {:.4e} F   (gate-source capacitance)", m.cgs);
        println!("  CGD    = {:.4e} F   (gate-drain capacitance)", m.cgd);
        return;
    }

    if let Some(m) = triode_by_name(&name_upper) {
        println!("Triode: {}", m.name);
        println!("{:-<50}", "");
        println!("  MU   = {:.1}       (amplification factor)", m.mu);
        println!("  EX   = {:.2}       (exponent)", m.ex);
        println!("  KG1  = {:.0}       (plate current scaling)", m.kg1);
        println!("  KP   = {:.0}       (plate resistance factor)", m.kp);
        println!("  KVB  = {:.0}       (knee voltage)", m.kvb);
        return;
    }

    if let Some(m) = pentode_by_name(&name_upper) {
        println!("Pentode: {}", m.name);
        println!("{:-<50}", "");
        println!("  MU   = {:.1}       (amplification factor)", m.mu);
        println!("  EX   = {:.2}       (exponent)", m.ex);
        println!("  KG1  = {:.0}       (plate current scaling)", m.kg1);
        println!("  KG2  = {:.0}       (screen current scaling)", m.kg2);
        println!("  KP   = {:.0}       (plate resistance factor)", m.kp);
        println!("  KVB  = {:.0}       (knee voltage)", m.kvb);
        println!("  KVB2 = {:.0}       (plate saturation knee)", m.kvb2);
        println!(
            "  VG2  = {:.0} V     (default screen voltage)",
            m.vg2_default
        );
        return;
    }

    if let Some(m) = opamp_by_name(&name_upper) {
        println!(
            "Op-Amp: {} ({})",
            m.name,
            if m.is_ota { "OTA" } else { "voltage op-amp" }
        );
        println!("{:-<50}", "");
        println!("  A0    = {:.2}       (open-loop gain)", m.open_loop_gain);
        println!("  GBW   = {:.4e} Hz    (gain-bandwidth product)", m.gbw);
        println!("  SR    = {:.3} V/us   (slew rate)", m.slew_rate);
        println!(
            "  VPOS  = {:.3} V      (positive output swing)",
            m.v_rail_pos
        );
        println!(
            "  VNEG  = {:.3} V      (negative output swing)",
            m.v_rail_neg
        );
        println!(
            "  RO    = {:.2} Ohm    (output impedance)",
            m.output_impedance
        );
        println!(
            "  COUT  = {:.4e} F   (output capacitance)",
            m.output_capacitance
        );
        if m.is_ota {
            println!("  IABC  = {:.4e} A     (bias current)", m.ota_iabc);
            println!("  VT    = {:.4e} V     (thermal voltage)", m.ota_vt);
            println!("  GM    = {:.4e} S     (transconductance)", m.ota_gm);
            println!("  RLOAD = {:.2} Ohm    (nominal load)", m.ota_r_load);
        }
        return;
    }

    if let Some(m) = transformer_by_name(&name_upper) {
        println!("Transformer: {}", m.name);
        println!("{:-<50}", "");
        println!(
            "  LP   = {:.4e} H     (primary inductance)",
            m.primary_inductance
        );
        println!("  K    = {:.5}        (coupling coefficient)", m.coupling);
        println!("  RP   = {:.2} Ohm    (primary DCR)", m.primary_dcr);
        println!("  RS   = {:.2} Ohm    (secondary DCR)", m.secondary_dcr);
        println!("  LLP  = {:.4e} H     (primary leakage)", m.primary_leakage);
        println!(
            "  LLS  = {:.4e} H     (secondary leakage)",
            m.secondary_leakage
        );
        println!(
            "  LM   = {:.4e} H     (magnetizing inductance)",
            m.magnetizing_inductance
        );
        println!(
            "  RC   = {:.4e} Ohm   (core-loss resistance)",
            m.core_loss_resistance
        );
        println!(
            "  CP   = {:.4e} F     (interwinding capacitance)",
            m.capacitance
        );
        return;
    }

    eprintln!("Model '{}' not found in any model library.", name);
    eprintln!("Use 'pedalkernel models' to see all available models.");
    std::process::exit(1);
}
