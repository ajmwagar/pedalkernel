//! CLI subcommand for listing and searching available component models.

use pedalkernel::models::{bjt_by_name, jfet_by_name, BJT_MODELS, JFET_MODELS};

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
        Some(other) => {
            eprintln!("Unknown type filter: '{other}'");
            eprintln!("Available types: bjt, npn, pnp, jfet, njf, pjf");
            std::process::exit(1);
        }
        None => {
            // Show all types
            print_bjts(search_upper.as_deref(), None);
            println!();
            print_jfets(search_upper.as_deref(), None);
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
        let vaf = if m.vaf.is_infinite() { "inf".to_string() } else { format!("{:.2}", m.vaf) };
        let var = if m.var.is_infinite() { "inf".to_string() } else { format!("{:.2}", m.var) };
        println!("  VAF  = {} V     (forward Early voltage)", vaf);
        println!("  VAR  = {} V     (reverse Early voltage)", var);
        let ikf = if m.ikf.is_infinite() { "inf".to_string() } else { format!("{:.4e}", m.ikf) };
        let ikr = if m.ikr.is_infinite() { "inf".to_string() } else { format!("{:.4e}", m.ikr) };
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
            if m.is_n_channel { "N-channel" } else { "P-channel" }
        );
        println!("{:-<50}", "");
        println!("  VTO    = {:.3} V      (threshold voltage)", m.vto);
        println!("  BETA   = {:.4e} A/V^2 (transconductance coeff)", m.beta);
        println!("  LAMBDA = {:.4e} 1/V   (channel-length mod)", m.lambda);
        println!("  Idss   = {:.4e} A     (= BETA * VTO^2)", m.beta * m.vto * m.vto);
        println!("  IS     = {:.4e} A     (gate junction sat current)", m.is);
        println!("  N      = {:.3}        (gate ideality factor)", m.n);
        println!("  RD     = {:.2} Ohm    (drain resistance)", m.rd);
        println!("  RS     = {:.2} Ohm    (source resistance)", m.rs);
        println!("  CGS    = {:.4e} F   (gate-source capacitance)", m.cgs);
        println!("  CGD    = {:.4e} F   (gate-drain capacitance)", m.cgd);
        return;
    }

    eprintln!("Model '{}' not found in any model library.", name);
    eprintln!("Use 'pedalkernel models' to see all available models.");
    std::process::exit(1);
}
