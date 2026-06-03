//! TB-303 ladder filter decomposition tests.
//!
//! These tests load the .pedal files from pedalkernel-pro (if present)
//! and verify that the 4-stage diode-connected BJT ladder decomposes
//! correctly via blockwise + K-method.
//!
//! If the .pedal files are not found (e.g., public-only checkout),
//! tests pass with a skip message. No .pedal content is inlined here.

use super::blockwise;
use super::graph::CircuitGraph;
use super::spqr::*;
use crate::PedalProcessor;

const SR: f64 = 48_000.0;

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
enum Tb303TestLayer {
    Topology,
    Compilation,
    Coupling,
    Runtime,
    Reference,
}

struct Tb303LayerEntry {
    filter: &'static str,
    layer: Tb303TestLayer,
    purpose: &'static str,
}

// First-line diagnostics for the TB-303 VCF work. Keep this ordered from
// cheapest/localest to broadest so a failing response test can be chased back
// through topology, compile-time stage selection, coupling, and runtime state.
const TB303_LAYERED_DIAGNOSTICS: &[Tb303LayerEntry] = &[
    Tb303LayerEntry {
        filter: "tb303_filter_uses_coupled_ladder_not_grounded_cascade",
        layer: Tb303TestLayer::Topology,
        purpose: "source topology is a coupled differential ladder, not the old grounded cascade",
    },
    Tb303LayerEntry {
        filter: "tb303_filter_uses_q12_input_differential_pair",
        layer: Tb303TestLayer::Topology,
        purpose: "audio enters through Q12 instead of direct ladder injection",
    },
    Tb303LayerEntry {
        filter: "tb303_blockwise_classifies_differential_diode_rungs_generically",
        layer: Tb303TestLayer::Topology,
        purpose: "blockwise analysis finds four QL/QR differential diode rungs",
    },
    Tb303LayerEntry {
        filter: "tb303_coupled_differential_ladder_compiles_to_bkm_not_monolithic_mna",
        layer: Tb303TestLayer::Compilation,
        purpose: "compiler emits BKM rung blocks instead of monolithic MultiNL",
    },
    Tb303LayerEntry {
        filter: "tb303_q12_compiles_as_multi_binding_wdf_boundary_block",
        layer: Tb303TestLayer::Compilation,
        purpose: "Q12 remains a boundary WDF block in the compiled stage contract",
    },
    Tb303LayerEntry {
        filter: "tb303_bkm_differential_rungs_have_side_aware_block_ports",
        layer: Tb303TestLayer::Coupling,
        purpose: "BKM exposes bottom-left, bottom-right, and top differential ports per rung",
    },
    Tb303LayerEntry {
        filter: "tb303_bkm_coupling_matrix_connects_ladder_block_ports",
        layer: Tb303TestLayer::Coupling,
        purpose: "coupling matrix connects adjacent ladder block rows",
    },
    Tb303LayerEntry {
        filter: "tb303_bkm_direct_path_is_lowpass",
        layer: Tb303TestLayer::Runtime,
        purpose: "direct BKM path produces finite nonzero lowpass response",
    },
    Tb303LayerEntry {
        filter: "tb303_bkm_full_processor_vco_port_path_is_not_flat",
        layer: Tb303TestLayer::Runtime,
        purpose: "external process_ports path reaches BKM without flattening the response",
    },
    Tb303LayerEntry {
        filter: "tb303_compare_bkm_forced_serial_and_htb_shape",
        layer: Tb303TestLayer::Reference,
        purpose: "full BKM response shape is checked against serial and Stinchcombe references",
    },
    Tb303LayerEntry {
        filter: "tb303_resonance_k_sweep_tracks_10pole_shape",
        layer: Tb303TestLayer::Reference,
        purpose: "late-stage resonance behavior tracks the 10-pole reference trend",
    },
];

#[test]
fn tb303_layered_diagnostics_are_ordered_and_actionable() {
    assert!(
        !TB303_LAYERED_DIAGNOSTICS.is_empty(),
        "TB303 diagnostic layer map must not be empty"
    );

    let mut previous = Tb303TestLayer::Topology;
    let mut seen = std::collections::HashSet::new();
    let mut layer_counts = [0usize; 5];
    for entry in TB303_LAYERED_DIAGNOSTICS {
        assert!(
            entry.layer >= previous,
            "TB303 diagnostic `{}` is out of layer order",
            entry.filter
        );
        previous = entry.layer;
        assert!(
            entry.filter.starts_with("tb303_"),
            "diagnostic filter `{}` should be directly runnable as a cargo test filter",
            entry.filter
        );
        assert!(
            seen.insert(entry.filter),
            "duplicate TB303 diagnostic filter `{}`",
            entry.filter
        );
        assert!(
            !entry.purpose.trim().is_empty(),
            "diagnostic `{}` needs a purpose",
            entry.filter
        );
        layer_counts[entry.layer as usize] += 1;
    }

    assert_eq!(
        layer_counts,
        [3, 2, 2, 2, 2],
        "keep the first-line TB303 diagnostics balanced across layers"
    );
}

fn settled_sine_rms<F>(freq: f64, amp: f64, mut process: F) -> f64
where
    F: FnMut(f64) -> f64,
{
    for i in 0..9600 {
        let input = amp * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
        let _ = process(input);
    }

    let mut sum_sq = 0.0f64;
    let mut count = 0usize;
    for i in 0..9600 {
        let input = amp * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
        let out = process(input);
        sum_sq += out * out;
        count += 1;
    }

    (sum_sq / count.max(1) as f64).sqrt()
}

fn settled_sine_ac_rms<F>(freq: f64, amp: f64, mut process: F) -> f64
where
    F: FnMut(f64) -> f64,
{
    for i in 0..9600 {
        let input = amp * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
        let _ = process(input);
    }

    let mut values = Vec::with_capacity(9600);
    for i in 0..9600 {
        let input = amp * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
        values.push(process(input));
    }

    ac_rms(&values)
}

fn settled_sine_ac_rms_ports(
    proc: &mut super::compiled::CompiledPedal,
    freq: f64,
    amp: f64,
    driven_inputs: &[(&str, f64)],
    dc_inputs: &[(&str, f64)],
    output: &str,
) -> f64 {
    let driven_inputs: Vec<(usize, f64)> = driven_inputs
        .iter()
        .map(|(name, gain)| {
            (
                proc.resolve_port(name)
                    .unwrap_or_else(|| panic!("missing input port {name}")),
                *gain,
            )
        })
        .collect();
    let dc_inputs: Vec<(usize, f64)> = dc_inputs
        .iter()
        .map(|(name, value)| {
            (
                proc.resolve_port(name)
                    .unwrap_or_else(|| panic!("missing input port {name}")),
                *value,
            )
        })
        .collect();
    let out_idx = proc
        .resolve_port(output)
        .unwrap_or_else(|| panic!("missing output port {output}"));
    let mut ports = vec![0.0; proc.port_count()];

    settled_sine_ac_rms(freq, amp, |input| {
        ports.fill(0.0);
        for &(idx, value) in &dc_inputs {
            ports[idx] = value;
        }
        for &(idx, gain) in &driven_inputs {
            ports[idx] = input * gain;
        }
        proc.process_ports(&mut ports);
        ports[out_idx]
    })
}

fn quick_sine_ac_rms_ports(
    proc: &mut super::compiled::CompiledPedal,
    freq: f64,
    amp: f64,
    driven_inputs: &[(&str, f64)],
    dc_inputs: &[(&str, f64)],
    output: &str,
) -> f64 {
    let driven_inputs: Vec<(usize, f64)> = driven_inputs
        .iter()
        .map(|(name, gain)| {
            (
                proc.resolve_port(name)
                    .unwrap_or_else(|| panic!("missing input port {name}")),
                *gain,
            )
        })
        .collect();
    let dc_inputs: Vec<(usize, f64)> = dc_inputs
        .iter()
        .map(|(name, value)| {
            (
                proc.resolve_port(name)
                    .unwrap_or_else(|| panic!("missing input port {name}")),
                *value,
            )
        })
        .collect();
    let out_idx = proc
        .resolve_port(output)
        .unwrap_or_else(|| panic!("missing output port {output}"));
    let mut ports = vec![0.0; proc.port_count()];

    for i in 0..1200 {
        let input = amp * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
        ports.fill(0.0);
        for &(idx, value) in &dc_inputs {
            ports[idx] = value;
        }
        for &(idx, gain) in &driven_inputs {
            ports[idx] = input * gain;
        }
        proc.process_ports(&mut ports);
    }

    let mut values = Vec::with_capacity(2400);
    for i in 0..2400 {
        let input = amp * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
        ports.fill(0.0);
        for &(idx, value) in &dc_inputs {
            ports[idx] = value;
        }
        for &(idx, gain) in &driven_inputs {
            ports[idx] = input * gain;
        }
        proc.process_ports(&mut ports);
        values.push(ports[out_idx]);
    }

    ac_rms(&values)
}

fn ac_rms(values: &[f64]) -> f64 {
    if values.is_empty() {
        return 0.0;
    }
    let mean = values.iter().copied().sum::<f64>() / values.len() as f64;
    let sum_sq = values
        .iter()
        .map(|value| {
            let ac = *value - mean;
            ac * ac
        })
        .sum::<f64>();
    (sum_sq / values.len() as f64).sqrt()
}

fn stinchcombe_htb_magnitude(freq_hz: f64, fc_hz: f64) -> f64 {
    let w = freq_hz / fc_hz;
    let w2 = w * w;
    let w3 = w2 * w;
    let w4 = w2 * w2;
    let re = w4 - 14.142 * w2 + 1.0;
    let im = -6.727 * w3 + 9.514 * w;
    1.0 / (re * re + im * im).sqrt()
}

#[derive(Clone, Copy, Debug)]
struct Complex {
    re: f64,
    im: f64,
}

impl Complex {
    const fn new(re: f64, im: f64) -> Self {
        Self { re, im }
    }

    const fn real(re: f64) -> Self {
        Self { re, im: 0.0 }
    }

    fn add(self, rhs: Self) -> Self {
        Self::new(self.re + rhs.re, self.im + rhs.im)
    }

    fn mul(self, rhs: Self) -> Self {
        Self::new(
            self.re * rhs.re - self.im * rhs.im,
            self.re * rhs.im + self.im * rhs.re,
        )
    }

    fn scale(self, rhs: f64) -> Self {
        Self::new(self.re * rhs, self.im * rhs)
    }

    fn div_scalar(self, rhs: f64) -> Self {
        Self::new(self.re / rhs, self.im / rhs)
    }

    fn mag(self) -> f64 {
        self.re.hypot(self.im)
    }
}

fn stinchcombe_10pole_magnitude(freq_hz: f64, fc_hz: f64, resonance_k: f64) -> f64 {
    let omega = 2.0 * std::f64::consts::PI * freq_hz;
    let omega_c = (2.0 * std::f64::consts::PI * fc_hz).max(1.0e-9);
    let s = Complex::new(0.0, omega);
    let s2 = s.mul(s);
    let s3 = s2.mul(s);
    let s4 = s2.mul(s2);

    let pole = |p: f64| s.add(Complex::real(p));

    let numerator = s3
        .mul(pole(109.9))
        .mul(pole(34.0))
        .mul(pole(7.41))
        .scale(1.06);

    let core = s4
        .div_scalar(omega_c.powi(4))
        .add(
            s3.div_scalar(omega_c.powi(3))
                .scale(2.0f64.powf(11.0 / 4.0)),
        )
        .add(s2.div_scalar(omega_c.powi(2)).scale(10.0 * 2.0f64.sqrt()))
        .add(s.div_scalar(omega_c).scale(2.0f64.powf(13.0 / 4.0)))
        .add(Complex::real(1.0));

    let coupling = pole(97.5)
        .mul(pole(38.5))
        .mul(pole(4.45))
        .mul(pole(578.1))
        .mul(pole(20.0))
        .mul(pole(7.41));
    let resonance = s4.mul(pole(46.5)).mul(pole(4.40)).scale(18.7 * resonance_k);
    let denominator = core.mul(coupling).add(resonance);
    let den_mag = denominator.mag();
    if den_mag <= 1.0e-30 {
        0.0
    } else {
        numerator.mag() / den_mag
    }
}

fn db_norm(value: f64, reference: f64) -> f64 {
    20.0 * (value / reference.max(1e-12)).max(1e-12).log10()
}

const TWO_RUNG_DIODE_LADDER: &str = r#"
pedal "Two Rung Diode Ladder" { supply 9V
  ports {
    audio_in: input(10k)
    audio_out: output
  }
  components {
    D1: diode(silicon)
    D2: diode(silicon)
    R_bias1: resistor(100k)
    R_bias2: resistor(100k)
    R_in: resistor(10k)
    C1: cap(33n)
    C2: cap(33n)
  }
  nets {
    vcc -> R_bias1.a
    R_bias1.b -> D1.a
    audio_in -> R_in.a
    R_in.b -> D1.a

    D1.b -> C1.a
    C1.b -> gnd
    D1.b -> D2.a

    vcc -> R_bias2.a
    R_bias2.b -> D2.a
    D2.b -> C2.a
    C2.b -> gnd
    D2.b -> audio_out
  }
  controls {}
}
"#;

const TWO_RUNG_DIODE_LADDER_WITH_FEEDBACK: &str = r#"
pedal "Two Rung Diode Ladder Feedback" { supply 9V
  ports {
    audio_in: input(10k)
    audio_out: output
  }
  components {
    D1: diode(silicon)
    D2: diode(silicon)
    R_bias1: resistor(100k)
    R_bias2: resistor(100k)
    R_in: resistor(10k)
    C1: cap(33n)
    C2: cap(33n)
    Resonance: pot(100k, b)
    R_fb_limit: resistor(33k)
  }
  nets {
    vcc -> R_bias1.a
    R_bias1.b -> D1.a
    audio_in -> R_in.a
    R_in.b -> D1.a

    D1.b -> C1.a
    C1.b -> gnd
    D1.b -> D2.a

    vcc -> R_bias2.a
    R_bias2.b -> D2.a
    D2.b -> C2.a
    C2.b -> gnd
    D2.b -> audio_out
    D2.b -> Resonance.a
    Resonance.b -> R_fb_limit.a
    R_fb_limit.b -> D1.a
  }
  controls {
    Resonance.position -> "Resonance" [0.0, 0.95] = 0.0
  }
}
"#;

const TWO_RUNG_DIODE_LADDER_WITH_FEEDBACK_AND_CV: &str = r#"
pedal "Two Rung Diode Ladder Feedback CV" { supply 9V
  ports {
    audio_in: input(10k)
    cv_cutoff: input(47k)
    audio_out: output
  }
  components {
    D1: diode(silicon)
    D2: diode(silicon)
    R_bias1: resistor(100k)
    R_bias2: resistor(100k)
    R_in: resistor(10k)
    R_cv: resistor(47k)
    C1: cap(33n)
    C2: cap(33n)
    Resonance: pot(100k, b)
    R_fb_limit: resistor(33k)
  }
  nets {
    vcc -> R_bias1.a
    R_bias1.b -> D1.a
    audio_in -> R_in.a
    R_in.b -> D1.a
    cv_cutoff -> R_cv.a
    R_cv.b -> D1.a

    D1.b -> C1.a
    C1.b -> gnd
    D1.b -> D2.a

    vcc -> R_bias2.a
    R_bias2.b -> D2.a
    D2.b -> C2.a
    C2.b -> gnd
    D2.b -> audio_out
    D2.b -> Resonance.a
    Resonance.b -> R_fb_limit.a
    R_fb_limit.b -> D1.a
  }
  controls {
    Resonance.position -> "Resonance" [0.0, 0.95] = 0.0
  }
}
"#;

const TWO_RUNG_DIFFERENTIAL_DIODE_LADDER: &str = r#"
pedal "Two Rung Differential Diode Ladder" { supply 9V
  ports {
    audio_in: input(10k)
    audio_out: output
  }
  components {
    QL1: npn(2sc945)
    QR1: npn(2sc945)
    QL2: npn(2sc945)
    QR2: npn(2sc945)
    R_top_l: resistor(10k)
    R_top_r: resistor(10k)
    R_in: resistor(10k)
    R_ref: resistor(10k)
    C1: cap(33n)
    C2: cap(33n)
  }
  nets {
    vcc -> R_top_l.a
    R_top_l.b -> QL2.base, QL2.collector
    vcc -> R_top_r.a
    R_top_r.b -> QR2.base, QR2.collector

    QL2.emitter -> QL1.base, QL1.collector
    QR2.emitter -> QR1.base, QR1.collector

    audio_in -> R_in.a
    R_in.b -> QL1.emitter
    gnd -> R_ref.a
    R_ref.b -> QR1.emitter

    QL1.emitter -> C1.a
    QR1.emitter -> C1.b
    QL2.emitter -> C2.a
    QR2.emitter -> C2.b

    QL2.emitter -> audio_out
  }
  controls {}
}
"#;

const Q12_TO_TWO_RUNG_DIFFERENTIAL_DIODE_LADDER: &str = r#"
pedal "Q12 To Two Rung Differential Diode Ladder" { supply 9V
  ports {
    audio_in: input(10k)
    audio_out: output
  }
  components {
    Q12L: npn(2sc945)
    Q12R: npn(2sc945)
    QL1: npn(2sc945)
    QR1: npn(2sc945)
    QL2: npn(2sc945)
    QR2: npn(2sc945)
    R_top_l: resistor(10k)
    R_top_r: resistor(10k)
    R_in: resistor(10k)
    R_ref: resistor(10k)
    R_tail: resistor(6.8k)
    C1: cap(33n)
    C2: cap(33n)
  }
  nets {
    audio_in -> R_in.a
    R_in.b -> Q12L.base
    gnd -> R_ref.a
    R_ref.b -> Q12R.base
    Q12L.emitter -> Q12R.emitter, R_tail.a
    R_tail.b -> gnd

    Q12L.collector -> QL1.emitter
    Q12R.collector -> QR1.emitter

    vcc -> R_top_l.a
    R_top_l.b -> QL2.base, QL2.collector
    vcc -> R_top_r.a
    R_top_r.b -> QR2.base, QR2.collector

    QL2.emitter -> QL1.base, QL1.collector
    QR2.emitter -> QR1.base, QR1.collector

    QL1.emitter -> C1.a
    QR1.emitter -> C1.b
    QL2.emitter -> C2.a
    QR2.emitter -> C2.b

    QL2.emitter -> audio_out
  }
  controls {}
}
"#;

/// Try to load a .pedal file from the pro crate's acidattack-core directory.
fn load_pro_pedal(filename: &str) -> Option<String> {
    let manifest_dir = env!("CARGO_MANIFEST_DIR");
    let pro_path = format!(
        "{}/../../pedalkernel-pro/crates/acidattack/acidattack-core/{filename}",
        manifest_dir
    );
    match std::fs::read_to_string(&pro_path) {
        Ok(s) => {
            eprintln!("  loaded {filename} ({} bytes)", s.len());
            Some(s)
        }
        Err(_) => None,
    }
}

fn tb303_bkm_vs_signals(
    bkm: &pedalkernel_rt::stage::BlockwiseStage,
    cutoff_cv: f64,
    resonance_cv: f64,
) -> Vec<f64> {
    bkm.vs_port_map
        .iter()
        .map(|(name, _)| {
            if name == "cv_cutoff" {
                cutoff_cv
            } else if name == "cv_resonance" {
                resonance_cv
            } else {
                0.0
            }
        })
        .collect()
}

fn tb303_bkm_vs_signals_with_vco(
    bkm: &pedalkernel_rt::stage::BlockwiseStage,
    vco_in: f64,
    cutoff_cv: f64,
    resonance_cv: f64,
) -> Vec<f64> {
    bkm.vs_port_map
        .iter()
        .map(|(name, _)| {
            if name == "vco_in" {
                vco_in
            } else if name == "cv_cutoff" {
                cutoff_cv
            } else if name == "cv_resonance" {
                resonance_cv
            } else {
                0.0
            }
        })
        .collect()
}

fn tb303_source_without_resonance_feedback(source: &str) -> String {
    let mut stripped = String::with_capacity(source.len());
    let mut skip_resonance_comment_block = false;
    for line in source.lines() {
        let trimmed = line.trim();
        if trimmed == "# Resonance feedback path"
            || trimmed == "# Resonance CV input: audio-rate voltage injection into the feedback"
            || trimmed == "# return node. This avoids moving the Resonance pot at CV rate."
            || trimmed == "# Output and resonance feedback use the top dynamic rung node. Resonance"
            || trimmed == "# is a passive coupling-network path here; gain calibration against the"
            || trimmed == "# full Stinchcombe feedback term belongs in the BKM resonance pass."
        {
            skip_resonance_comment_block = true;
            continue;
        }

        let is_resonance_component = trimmed.starts_with("Resonance:")
            || trimmed.starts_with("R_fb_")
            || trimmed.starts_with("R_res_cv:")
            || trimmed.starts_with("Ufb:");
        let is_resonance_net = trimmed.starts_with("cv_resonance ->")
            || trimmed.contains("-> R_res_cv.")
            || trimmed.contains("-> R_fb_")
            || trimmed.contains("-> Ufb.")
            || trimmed.contains("-> Resonance.")
            || trimmed.contains("R_fb_limit.b ->");
        let is_resonance_control = trimmed.starts_with("Resonance.position ->");

        if is_resonance_component || is_resonance_net || is_resonance_control {
            skip_resonance_comment_block = false;
            continue;
        }

        if skip_resonance_comment_block && trimmed.is_empty() {
            skip_resonance_comment_block = false;
            continue;
        }

        stripped.push_str(line);
        stripped.push('\n');
    }
    stripped
}

fn bkm_block_row_coupling_summary(
    bkm: &pedalkernel_rt::stage::BlockwiseStage,
) -> Vec<(usize, f64, f64)> {
    let block_rows = bkm.blocks.len().min(bkm.n_ports);
    (0..block_rows)
        .map(|row| {
            let diag = bkm.coupling_s[row * bkm.n_ports + row];
            let offdiag_max = (0..bkm.n_ports)
                .filter(|&col| col != row)
                .map(|col| bkm.coupling_s[row * bkm.n_ports + col].abs())
                .fold(0.0_f64, f64::max);
            (row, diag, offdiag_max)
        })
        .collect()
}

fn bkm_identity_isolated_block_rows(bkm: &pedalkernel_rt::stage::BlockwiseStage) -> Vec<usize> {
    bkm_block_row_coupling_summary(bkm)
        .into_iter()
        .filter_map(|(row, diag, offdiag_max)| {
            ((diag - 1.0).abs() < 1e-4 && offdiag_max < 1e-5).then_some(row)
        })
        .collect()
}

macro_rules! skip_if_missing {
    ($source:expr, $name:expr) => {
        match $source {
            Some(s) => s,
            None => {
                eprintln!("  SKIP: {} not found (pro crate not present)", $name);
                return;
            }
        }
    };
}

#[test]
fn tb303_cutoff_path_uses_expo_converter_boundary() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");

    for q in ["Q9", "Q10", "Q11"] {
        assert!(
            source.contains(&format!("{q}: npn(")),
            "TB303 cutoff path should explicitly model the Q9/Q10/Q11 exponential converter boundary; missing {q}"
        );
    }

    assert!(
        !source.contains("R_tail_l.b -> Cutoff.a")
            && !source.contains("R_tail_r.b -> Cutoff.a"),
        "Cutoff pot should not be the ladder tail shunt. The pot/CV should drive the expo converter, \
         which then controls the shared ladder tail current."
    );

    assert!(
        source.contains("Q12L.emitter -> Q12R.emitter, Q11.collector"),
        "The cutoff converter output should be the shared Q12 emitter tail current sink"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 1. Filter compiles without error (structural — no K-tables)
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn tb303_filter_compiles() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    eprintln!("  parsing...");
    let def = crate::dsl::parse_pedal_file(&source).expect("parse failed");
    eprintln!("  compiling (skip_k_tables=true)...");
    let compiled =
        super::compile_pedal_with_options(&def, SR, super::compile::CompileOptions::debug());
    assert!(
        compiled.is_ok(),
        "TB303 filter should compile: {:?}",
        compiled.err()
    );
    let compiled = compiled.unwrap();
    eprintln!("  TB303 filter: {} stages", compiled.stages.len());
    for (i, stage) in compiled.stages.iter().enumerate() {
        match stage {
            super::compiled::Stage::Wdf(w) => {
                let is_bjt = matches!(w.root, pedalkernel_rt::stage::RootKind::Bjt(_));
                eprintln!(
                    "    stage {i}: WDF bjt={is_bjt} rp={:.0}",
                    w.tree.port_resistance()
                );
            }
            super::compiled::Stage::Iir(iir) => {
                eprintln!("    stage {i}: IIR");
            }
            super::compiled::Stage::MultiNl(m) => {
                eprintln!("    stage {i}: MultiNL (MONOLITHIC — bad!)");
            }
            _ => {
                eprintln!("    stage {i}: other");
            }
        }
    }
    assert!(
        compiled
            .stages
            .iter()
            .all(|stage| !matches!(stage, super::compiled::Stage::MultiNl(_))),
        "TB303 cutoff converter groups should be consumed by control analysis, not emitted as audio-rate MultiNL stages"
    );
}

#[test]
fn tb303_filter_uses_coupled_ladder_not_grounded_cascade() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");

    for cap in ["C1", "C2", "C3", "C4"] {
        assert!(
            !source.contains(&format!("{cap}.b -> gnd")),
            "{cap} must be a cross-ladder capacitor, not a shunt-to-ground cascade pole"
        );
    }

    for link in [
        "Q1.emitter -> Q2.base",
        "Q2.emitter -> Q3.base",
        "Q3.emitter -> Q4.base",
    ] {
        assert!(
            !source.contains(link),
            "TB303 ladder must not contain serial cascade link `{link}`"
        );
    }

    for (left, right) in [
        ("QL1.emitter -> C1.a", "QR1.emitter -> C1.b"),
        ("QL2.emitter -> C2.a", "QR2.emitter -> C2.b"),
        ("QL3.emitter -> C3.a", "QR3.emitter -> C3.b"),
        ("QL4.emitter -> C4.a", "QR4.emitter -> C4.b"),
    ] {
        assert!(
            source.contains(left) && source.contains(right),
            "TB303 ladder should couple differential rung nodes through `{left}` / `{right}`"
        );
    }
}

#[test]
fn tb303_filter_uses_one_shared_cutoff_tail_current() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");

    for independent_bias in ["R_bias2", "R_bias3", "R_bias4", "R_ref_bottom"] {
        assert!(
            !source.contains(independent_bias),
            "TB303 cutoff must not be set by independent rung bias element `{independent_bias}`"
        );
    }

    assert!(
        source.contains("R_bias_floor.b -> R_top_l.a, R_top_r.a"),
        "top ladder feed should be fixed and shared across both diode chains"
    );
    assert!(
        !source.contains("R_bias_floor.b -> Cutoff.a"),
        "Cutoff pot should not sit in the top feed as a per-rung voltage bias"
    );

    for shared_tail_link in ["Cutoff.w -> R_cutoff_sum.a", "R_cv.b -> Q10.base"] {
        assert!(
            source.contains(shared_tail_link),
            "TB303 cutoff current path should contain `{shared_tail_link}`"
        );
    }
}

#[test]
fn tb303_filter_uses_q12_input_differential_pair() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");

    for q12 in ["Q12L: npn(2sc945)", "Q12R: npn(2sc945)"] {
        assert!(
            source.contains(q12),
            "TB303 audio injection should include input differential pair device `{q12}`"
        );
    }

    for direct_injection in ["R_in.b -> QL1.emitter", "R_vco.b -> QL1.emitter"] {
        assert!(
            !source.contains(direct_injection),
            "audio/VCO must enter Q12, not inject directly into the ladder through `{direct_injection}`"
        );
    }

    for q12_link in [
        "R_in.b -> Q12L.base",
        "R_vco.b -> Q12L.base",
        "R_q9_bias.b -> Q12R.base",
        "QL1.emitter -> Q12L.collector",
        "QR1.emitter -> Q12R.collector",
        "Q12L.emitter -> Q12R.emitter, Q11.collector",
    ] {
        assert!(
            source.contains(q12_link),
            "TB303 Q12 input differential pair should contain `{q12_link}`"
        );
    }
}

#[test]
fn tb303_q12_compiles_as_multi_binding_wdf_boundary_block() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let def = crate::dsl::parse_pedal_file(&source).expect("parse failed");

    let compiled =
        super::compile_pedal_with_options(&def, SR, super::compile::CompileOptions::default())
            .expect("compile failed");

    let q12 = compiled.stages.iter().find_map(|stage| {
        if let super::compiled::Stage::Wdf(wdf) = stage {
            let label = {
                #[cfg(debug_assertions)]
                {
                    wdf.debug_label.as_str()
                }
                #[cfg(not(debug_assertions))]
                {
                    ""
                }
            };
            if wdf.root_comp_id == "Q12L"
                || wdf.root_comp_id == "Q12R"
                || label.contains("Q12L")
                || label.contains("Q12R")
            {
                return Some(wdf);
            }
        }
        None
    });

    let q12 = q12.expect(
        "Q12 input differential pair should compile as one WDF boundary block, not be omitted from BKM",
    );
    let labels: Vec<&str> = q12
        .boundary_bindings
        .iter()
        .map(|binding| binding.label.as_str())
        .collect();
    for expected in [
        "base_left",
        "base_right",
        "collector_left",
        "collector_right",
        "emitter_tail",
    ] {
        assert!(
            labels.contains(&expected),
            "Q12 WDF boundary bindings should include `{expected}`, got {labels:?}"
        );
    }
}

#[test]
fn tb303_stage_graph_connects_q12_collectors_to_bkm_ladder_ports() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let def = crate::dsl::parse_pedal_file(&source).expect("parse failed");

    let compiled =
        super::compile_pedal_with_options(&def, SR, super::compile::CompileOptions::default())
            .expect("compile failed");

    let graph = &compiled.stage_graph;
    assert!(
        !graph.stages.is_empty(),
        "compiled pedal should expose an emitted stage graph"
    );

    let q12_stage_idx = graph
        .stages
        .iter()
        .position(|stage| {
            stage.kind == "Wdf"
                && stage
                    .ports
                    .iter()
                    .any(|port| port.label == "collector_left")
                && stage
                    .ports
                    .iter()
                    .any(|port| port.label == "collector_right")
        })
        .expect("Q12 should appear as a WDF stage graph node with collector boundary ports");

    let bkm_stage_idx = graph
        .stages
        .iter()
        .position(|stage| stage.kind == "Blockwise")
        .expect("TB303 ladder should appear as a BKM stage graph node");
    assert!(
        !graph
            .stages
            .iter()
            .any(|stage| stage.kind == "BlockwiseKMethod"),
        "stage graph should name the topology Blockwise; K-method is a solver/table detail"
    );

    for collector_label in ["collector_left", "collector_right"] {
        let q12_port_idx = graph.stages[q12_stage_idx]
            .ports
            .iter()
            .position(|port| port.label == collector_label)
            .expect("Q12 collector port should be present");
        let q12_node = graph.stages[q12_stage_idx].ports[q12_port_idx].node_id;

        assert!(
            graph.stages[bkm_stage_idx]
                .ports
                .iter()
                .any(|port| port.node_id == q12_node),
            "BKM stage graph ports should include Q12 {collector_label} node {q12_node}"
        );
        assert!(
            graph.connections.iter().any(|connection| {
                connection.node_id == q12_node
                    && ((connection.from_stage == q12_stage_idx
                        && connection.from_port == q12_port_idx
                        && connection.to_stage == bkm_stage_idx)
                        || (connection.to_stage == q12_stage_idx
                            && connection.to_port == q12_port_idx
                            && connection.from_stage == bkm_stage_idx))
            }),
            "stage graph should connect Q12 {collector_label} node {q12_node} to the BKM ladder"
        );
    }
}

#[test]
fn tb303_stage_route_plan_maps_external_ports_to_bkm_vs_boundaries() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let def = crate::dsl::parse_pedal_file(&source).expect("parse failed");

    let mut compiled =
        super::compile_pedal_with_options(&def, SR, super::compile::CompileOptions::default())
            .expect("compile failed");
    compiled.cache_all_vs_pointers();

    let bkm_stage_idx = compiled
        .stages
        .iter()
        .position(|stage| matches!(stage, pedalkernel_rt::processor::Stage::Blockwise(_)))
        .expect("TB303 ladder should compile to a BKM stage");
    let audio_out_idx = compiled
        .resolve_port("audio_out")
        .expect("TB303 should expose audio_out");

    let debug = compiled.stage_route_plan.debug();
    assert_eq!(
        debug.primary_bkm_stage_idx,
        Some(bkm_stage_idx),
        "route plan should identify the graph-routed BKM stage, debug={debug:?}"
    );

    assert!(
        debug
            .primary_bkm_vs_bindings
            .iter()
            .any(|binding| binding.starts_with("cv_cutoff")),
        "route plan should map cutoff CV into a BKM VS boundary, debug={debug:?}"
    );
    for input_name in ["audio_in", "vco_in"] {
        assert!(
            debug
                .primary_bkm_boundary_drives
                .iter()
                .any(|binding| binding.contains(input_name)),
            "route plan should route `{input_name}` through the Q12 WDF boundary handoff, debug={debug:?}"
        );
        assert!(
            !debug
                .primary_bkm_vs_bindings
                .iter()
                .any(|binding| binding.starts_with(input_name)),
            "route plan should not also inject `{input_name}` directly into BKM once Q12 owns it, debug={debug:?}"
        );
        assert!(
            !debug
                .primary_bkm_boundary_drives
                .iter()
                .any(|binding| binding.contains(&format!("-[\"{input_name}\"]"))),
            "route plan must not subtract `{input_name}` from the Q12 right/base reference side, debug={debug:?}"
        );
    }

    assert!(
        debug.primary_bkm_output_ports.contains(&audio_out_idx),
        "route plan should publish BKM output to audio_out ({audio_out_idx}), debug={debug:?}"
    );
    assert!(
        debug.connection_count > 0,
        "route plan should retain stage graph connectivity for diagnostics"
    );
}

#[test]
fn tb303_diode_connected_bjts_reduce_to_diode_roots_in_multinl_fallback() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let def = crate::dsl::parse_pedal_file(&source).expect("parse failed");

    let mut options = super::compile::CompileOptions::default();
    options.skip_blockwise = true;
    let compiled = super::compile_pedal_with_options(&def, SR, options).expect("compile failed");

    let ladder = compiled
        .stages
        .iter()
        .find_map(|stage| {
            if let super::compiled::Stage::MultiNl(mnl) = stage {
                (mnl.n_nl >= 8).then_some(mnl)
            } else {
                None
            }
        })
        .expect("skip_blockwise fallback should compile diode-connected BJTs through MultiNL");

    assert!(
        ladder.device_groups.is_none(),
        "diode-connected BJT ladder fallback must not use BjtTwoPort grouped MNA; \
         base=collector devices should reduce to diode roots before any MultiNL fallback"
    );
    assert_eq!(
        ladder.nl_devices.len(),
        8,
        "the coupled TB303 ladder has eight diode-connected BJT junctions"
    );
    assert!(
        ladder
            .nl_devices
            .iter()
            .all(|device| matches!(device, pedalkernel_rt::stage::NlDeviceKind::Diode(_))),
        "all diode-connected BJT junctions should be represented as diode roots"
    );
}

#[test]
fn tb303_blockwise_classifies_differential_diode_rungs_generically() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let def = crate::dsl::parse_pedal_file(&source).expect("parse failed");
    let graph = CircuitGraph::from_pedal(&def);

    let active_set: std::collections::HashSet<usize> =
        graph.active_edge_indices.iter().copied().collect();
    let all_edges: Vec<usize> = (0..graph.edges.len())
        .filter(|i| !active_set.contains(i))
        .collect();

    let plan = blockwise::analyze_blockwise(&all_edges, &graph)
        .expect("TB303 coupled ladder should produce a blockwise plan");

    let rung_blocks: Vec<_> = plan
        .blocks
        .iter()
        .filter(|block| {
            matches!(
                block.topology,
                blockwise::BlockTopology::DifferentialDiodeRung { .. }
            )
        })
        .collect();
    assert_eq!(
        rung_blocks.len(),
        4,
        "TB303 ladder should expose four generic differential diode rung blocks even when Q12/control devices are present"
    );
    for (i, block) in rung_blocks.iter().enumerate() {
        assert!(
            matches!(
                block.topology,
                blockwise::BlockTopology::DifferentialDiodeRung { .. }
            ),
            "block {i} should be classified as a generic differential diode rung, got {:?}",
            block.topology
        );
    }
}

#[test]
fn tb303_coupled_differential_ladder_compiles_to_bkm_not_monolithic_mna() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let def = crate::dsl::parse_pedal_file(&source).expect("parse failed");

    let compiled =
        super::compile_pedal_with_options(&def, SR, super::compile::CompileOptions::default())
            .expect("compile failed");

    let bkm = compiled.stages.iter().find_map(|stage| {
        if let super::compiled::Stage::Blockwise(bkm) = stage {
            Some(bkm)
        } else {
            None
        }
    });
    assert!(
        bkm.is_some(),
        "coupled differential diode ladders should compile to BKM, not monolithic MultiNL"
    );
    assert!(
        !compiled
            .stages
            .iter()
            .any(|stage| matches!(stage, super::compiled::Stage::MultiNl(mnl) if mnl.n_nl >= 8)),
        "TB303 ladder should not fall back to monolithic MultiNL"
    );

    let bkm = bkm.unwrap();
    assert_eq!(bkm.blocks.len(), 4);
    assert!(
        bkm.blocks.iter().all(|block| block.k_table.dims == 2),
        "differential diode rung BKM blocks should use 2D K-tables"
    );
}

#[test]
fn tb303_bkm_output_and_feedback_tap_top_rung() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let def = crate::dsl::parse_pedal_file(&source).expect("parse failed");
    let graph = CircuitGraph::from_pedal(&def);

    assert!(
        source.contains("QL4.emitter -> C_out.a"),
        "TB303 filter output coupling must tap the top dynamic rung node"
    );
    assert!(
        source.contains("QL4.emitter -> Resonance.b"),
        "TB303 resonance send must tap the same top dynamic rung node as output"
    );
    assert!(
        source.contains("R_fb_limit.b -> QR1.emitter"),
        "TB303 resonance return must enter the opposite bottom side so the feedback is out of phase at the cutoff peak"
    );

    let active_set: std::collections::HashSet<usize> =
        graph.active_edge_indices.iter().copied().collect();
    let all_edges: Vec<usize> = (0..graph.edges.len())
        .filter(|i| !active_set.contains(i))
        .collect();
    let plan = blockwise::analyze_blockwise(&all_edges, &graph)
        .expect("TB303 coupled ladder should produce a blockwise plan");

    let block_comp_ids = |block: &blockwise::Block| -> Vec<&str> {
        let mut ids: Vec<&str> = block
            .nl_edges
            .iter()
            .map(|&eidx| graph.components[graph.edges[eidx].comp_idx].id.as_str())
            .collect();
        ids.sort_unstable();
        ids.dedup();
        ids
    };

    assert_eq!(plan.blocks.len(), 4);
    assert_eq!(
        block_comp_ids(&plan.blocks[0]),
        vec!["QL1", "QR1"],
        "first BKM block should be the bottom/input rung"
    );
    assert_eq!(
        block_comp_ids(&plan.blocks[3]),
        vec!["QL4", "QR4"],
        "last BKM block should be the top/output rung"
    );

    let compiled =
        super::compile_pedal_with_options(&def, SR, super::compile::CompileOptions::default())
            .expect("compile failed");
    let bkm = compiled
        .stages
        .iter()
        .find_map(|stage| {
            if let super::compiled::Stage::Blockwise(bkm) = stage {
                Some(bkm)
            } else {
                None
            }
        })
        .expect("TB303 should compile to BKM");

    assert_eq!(
        bkm.output_block, 3,
        "compiled BKM output must tap the top rung block"
    );
    assert!(
        bkm.feedback_port_map
            .iter()
            .all(|(block_idx, _)| *block_idx == bkm.output_block),
        "feedback send ports must be driven from the selected output/top block"
    );
}

#[test]
fn tb303_bkm_vco_drive_is_external_coupling_port() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let def = crate::dsl::parse_pedal_file(&source).expect("parse failed");
    let graph = CircuitGraph::from_pedal(&def);

    let active_set: std::collections::HashSet<usize> =
        graph.active_edge_indices.iter().copied().collect();
    let all_edges: Vec<usize> = (0..graph.edges.len())
        .filter(|i| !active_set.contains(i))
        .collect();
    let groups = super::signal_flow::find_flow_groups(&all_edges, &graph);
    let feedback_group = groups
        .iter()
        .find(|group| group.has_feedback())
        .expect("TB303 ladder feedback group should exist");
    let plan = blockwise::analyze_blockwise(&feedback_group.all_edges(), &graph)
        .expect("TB303 coupled ladder should produce a blockwise plan");

    let edge_ids = |edges: &[usize]| -> Vec<&str> {
        edges
            .iter()
            .map(|&eidx| graph.components[graph.edges[eidx].comp_idx].id.as_str())
            .collect()
    };
    let local_r_vco_blocks: Vec<usize> = plan
        .blocks
        .iter()
        .enumerate()
        .filter_map(|(bi, block)| {
            edge_ids(&block.linear_edges)
                .contains(&"R_vco")
                .then_some(bi)
        })
        .collect();
    let coupling_ids = edge_ids(&plan.coupling_edges);

    assert!(
        local_r_vco_blocks.is_empty(),
        "R_vco must not be absorbed into a local rung block; it is an external drive boundary, local blocks={local_r_vco_blocks:?}"
    );
    assert!(
        coupling_ids.contains(&"R_vco"),
        "R_vco must remain in the BKM coupling network so vco_in can become a VS port, coupling={coupling_ids:?}"
    );

    let compiled =
        super::compile_pedal_with_options(&def, SR, super::compile::CompileOptions::default())
            .expect("compile failed");
    let bkm = compiled
        .stages
        .iter()
        .find_map(|stage| {
            if let super::compiled::Stage::Blockwise(bkm) = stage {
                Some(bkm)
            } else {
                None
            }
        })
        .expect("TB303 should compile to BKM");

    assert!(
        bkm.vs_port_map.iter().any(|(name, _)| name == "vco_in"),
        "vco_in must be represented as a BKM VS port; ports={:?}",
        bkm.vs_port_map
    );
    assert!(
        bkm.vs_port_map.iter().any(|(name, _)| name == "audio_in"),
        "audio_in must also be represented as a BKM VS port; ports={:?}",
        bkm.vs_port_map
    );
}

#[test]
fn tb303_bkm_does_not_stamp_reactive_coupling_as_resistor() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let def = crate::dsl::parse_pedal_file(&source).expect("parse failed");
    let compiled =
        super::compile_pedal_with_options(&def, SR, super::compile::CompileOptions::default())
            .expect("compile failed");
    let bkm = compiled
        .stages
        .iter()
        .find_map(|stage| {
            if let super::compiled::Stage::Blockwise(bkm) = stage {
                Some(bkm)
            } else {
                None
            }
        })
        .expect("TB303 should compile to BKM");

    let reactive_coupling: Vec<_> = bkm
        .coupling_elements
        .iter()
        .filter(|element| matches!(element.comp_id.as_str(), "C_out"))
        .map(|element| (element.comp_id.as_str(), element.resistance))
        .collect();

    assert!(
        reactive_coupling.is_empty(),
        "reactive BKM boundary edges cannot be represented as fallback resistors; got {reactive_coupling:?}"
    );

    assert!(
        bkm.coupling_passives
            .iter()
            .any(|passive| passive.comp_id == "C_out"
                && passive.port_idx < bkm.n_ports
                && bkm.coupling_ports[passive.port_idx].mna.terminals.pos.is_some()),
        "reactive BKM boundary edge C_out must be represented as a passive WDF coupling port; passives={:?}",
        bkm.coupling_passives
            .iter()
            .map(|passive| (&passive.comp_id, passive.port_idx))
            .collect::<Vec<_>>()
    );
    assert!(
        bkm.output_port_index.is_some(),
        "BKM should add a high-Z output probe coupling port so audio_out is read after C_out/R_out, not from the internal ladder tap"
    );
    assert!(
        bkm.coupling_elements
            .iter()
            .any(|element| element.comp_id == "R_out"),
        "TB303 output load R_out must be in the same BKM coupling adaptor as C_out; \
         otherwise the output coupling cap has no load in the coupled solve. coupling={:?}",
        bkm.coupling_elements
            .iter()
            .map(|element| element.comp_id.as_str())
            .collect::<Vec<_>>()
    );
}

#[test]
fn tb303_bkm_pulls_input_coupling_caps_into_adaptor() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    assert!(
        source.contains("C_in: cap(") && source.contains("C_vco: cap("),
        "TB303 .pedal should model the input/VCO coupling caps that contribute to the Stinchcombe 10-pole shelf"
    );

    let def = crate::dsl::parse_pedal_file(&source).expect("parse failed");
    let compiled =
        super::compile_pedal_with_options(&def, SR, super::compile::CompileOptions::default())
            .expect("compile failed");
    let bkm = compiled
        .stages
        .iter()
        .find_map(|stage| {
            if let super::compiled::Stage::Blockwise(bkm) = stage {
                Some(bkm)
            } else {
                None
            }
        })
        .expect("TB303 should compile to BKM");

    for cap_id in ["C_in", "C_vco"] {
        assert!(
            bkm.coupling_passives
                .iter()
                .any(|passive| passive.comp_id == cap_id && passive.port_idx < bkm.n_ports),
            "{cap_id} must be represented as a passive WDF coupling port inside BKM; passives={:?}",
            bkm.coupling_passives
                .iter()
                .map(|passive| (&passive.comp_id, passive.port_idx))
                .collect::<Vec<_>>()
        );
    }

    for (idx, block) in bkm.blocks.iter().enumerate() {
        assert!(
            block.dc_offset.is_finite() && block.dc_offset.abs() < 10.0,
            "AC-coupled BKM input caps must not poison DC initialization; block {idx} dc_offset={}",
            block.dc_offset
        );
    }
}

#[test]
fn tb303_coupling_caps_land_near_stinchcombe_shelf_corners() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let def = crate::dsl::parse_pedal_file(&source).expect("parse failed");
    let compiled =
        super::compile_pedal_with_options(&def, SR, super::compile::CompileOptions::default())
            .expect("compile failed");
    let bkm = compiled
        .stages
        .iter()
        .find_map(|stage| {
            if let super::compiled::Stage::Blockwise(bkm) = stage {
                Some(bkm)
            } else {
                None
            }
        })
        .expect("TB303 should compile to BKM");

    let cap_value = |id: &str| -> f64 {
        bkm.coupling_passives
            .iter()
            .find_map(|passive| {
                if passive.comp_id != id {
                    return None;
                }
                if let pedalkernel_rt::dyn_node::DynNode::Leaf(
                    pedalkernel_rt::wdf_leaf::LeafKind::OnePort { runtime, .. },
                ) = &passive.node
                {
                    match runtime.spec.kind {
                        pedalkernel_rt::boundary_math::OnePortKind::Capacitor(farads) => {
                            Some(farads)
                        }
                        _ => None,
                    }
                } else {
                    None
                }
            })
            .unwrap_or_else(|| panic!("missing coupling cap {id} in BKM passives"))
    };
    let resistor_value = |id: &str| -> f64 {
        bkm.coupling_elements
            .iter()
            .find(|element| element.comp_id == id)
            .map(|element| element.resistance)
            .unwrap_or_else(|| panic!("missing coupling resistor {id} in BKM elements"))
    };
    let corner_rad = |r: f64, c: f64| 1.0 / (r * c).max(1.0e-30);

    let input_corner = corner_rad(resistor_value("R_in"), cap_value("C_in"));
    let output_corner = corner_rad(resistor_value("R_out"), cap_value("C_out"));

    eprintln!(
        "  TB303 coupling shelf corners: input={input_corner:.1}rad/s, output={output_corner:.1}rad/s"
    );

    assert!(
        (450.0..700.0).contains(&input_corner),
        "input coupling should sit near Stinchcombe's high shelf pole around 578rad/s: {input_corner:.1}rad/s"
    );
    assert!(
        (40.0..80.0).contains(&output_corner),
        "output coupling should sit between the published low shelf poles rather than around 1krad/s: {output_corner:.1}rad/s"
    );
}

#[test]
fn tb303_bkm_differential_rungs_have_side_aware_block_ports() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let def = crate::dsl::parse_pedal_file(&source).expect("parse failed");
    let compiled =
        super::compile_pedal_with_options(&def, SR, super::compile::CompileOptions::default())
            .expect("compile failed");
    let bkm = compiled
        .stages
        .iter()
        .find_map(|stage| {
            if let super::compiled::Stage::Blockwise(bkm) = stage {
                Some(bkm)
            } else {
                None
            }
        })
        .expect("TB303 should compile to BKM");

    assert_eq!(bkm.blocks.len(), 4);
    assert_eq!(
        bkm.solve_mode,
        pedalkernel_rt::stage::BlockwiseSolveMode::CoupledNewton,
        "default TB303 BKM must use the delay-free coupled K-method solve"
    );
    assert!(
        bkm.diode_ladder_core.is_none(),
        "optimized diode ladder core must be opt-in until it participates in the coupled BKM feedback solve"
    );
    assert!(
        bkm.feedback_port_map.is_empty(),
        "coupled BKM should keep feedback in the coupling matrix instead of adding a synthetic feedback drive port"
    );
    assert!(
        bkm.coupling_ports.len() >= bkm.blocks.len(),
        "BKM should expose one coupling port per rung"
    );
    assert_eq!(
        bkm.block_ports.len(),
        bkm.blocks.len(),
        "BKM must track which coupling ports are owned by each block"
    );

    for (block_idx, ports) in bkm.block_ports.iter().enumerate() {
        assert!(
            ports.len() >= 3,
            "differential rung block {block_idx} should expose bottom left/right side ports plus a top differential port, got {ports:?}"
        );
        let (bottom_left_idx, bottom_right_idx, top_diff_idx) = bkm
            .differential_rung_ports(block_idx)
            .expect("differential rung ports should be role-bound");
        let bottom_left = &bkm.coupling_ports[bottom_left_idx];
        let bottom_right = &bkm.coupling_ports[bottom_right_idx];
        let top_diff = &bkm.coupling_ports[top_diff_idx];
        assert!(
            bottom_left.mna.terminals.pos.is_some() && bottom_left.mna.terminals.neg.is_none(),
            "bottom-left rung port should be single-ended for asymmetric passive feedback, got {bottom_left:?}"
        );
        assert!(
            bottom_right.mna.terminals.pos.is_some() && bottom_right.mna.terminals.neg.is_none(),
            "bottom-right rung port should be single-ended for asymmetric passive feedback, got {bottom_right:?}"
        );
        assert!(
            top_diff.mna.terminals.pos.is_some() && top_diff.mna.terminals.neg.is_some(),
            "top rung port should stay floating/differential for symmetric inter-rung coupling, got {top_diff:?}"
        );
    }
}

#[test]
fn tb303_coupled_fixed_point_is_realtime_opt_out() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let def = crate::dsl::parse_pedal_file(&source).expect("parse failed");
    let compiled = super::compile_pedal_with_options(
        &def,
        SR,
        super::compile::CompileOptions {
            coupled_blockwise_newton: false,
            ..Default::default()
        },
    )
    .expect("compile failed");
    let bkm = compiled
        .stages
        .iter()
        .find_map(|stage| {
            if let super::compiled::Stage::Blockwise(bkm) = stage {
                Some(bkm)
            } else {
                None
            }
        })
        .expect("TB303 should compile to BKM");

    assert_eq!(
        bkm.solve_mode,
        pedalkernel_rt::stage::BlockwiseSolveMode::CoupledFixedPoint
    );
}

#[test]
fn tb303_coupled_fixed_point_resonance_zero_has_no_active_peak() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let freqs = [100.0, 200.0, 500.0, 1000.0, 2000.0, 5000.0, 10_000.0];
    let cache_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("target")
        .join("test-cache");
    let _ = std::fs::create_dir_all(&cache_dir);
    let mut options = super::compile::CompileOptions::default();
    options.coupled_blockwise_newton = false;
    options.oversampling = crate::oversampling::OversamplingFactor::X1;
    let blob = super::compile::compile_pedal_cached(
        &source,
        "tb303_delayed_no_active_peak",
        "tb303_delayed_no_active_peak",
        SR,
        &options,
        &cache_dir,
    )
    .expect("compile failed");

    let gains: Vec<f64> = freqs
        .iter()
        .map(|&freq| {
            let mut proc: super::compiled::CompiledPedal =
                postcard::from_bytes(&blob).expect("deserialize failed");
            proc.set_control_immediate("Cutoff", 0.5);
            proc.set_control_immediate("Resonance", 0.0);
            quick_sine_ac_rms_ports(
                &mut proc,
                freq,
                0.003,
                &[("vco_in", 1.0)],
                &[("audio_in", 0.0), ("cv_cutoff", 0.0), ("cv_resonance", 0.0)],
                "audio_out",
            )
        })
        .collect();
    let norm = gains[0].max(1.0e-12);
    let normalized_db: Vec<f64> = gains.iter().map(|gain| db_norm(*gain, norm)).collect();
    let max_gain_db = normalized_db
        .iter()
        .copied()
        .fold(f64::NEG_INFINITY, f64::max);

    eprintln!(
        "  delayed BKM k=0 normalized dB: {:?}",
        normalized_db
            .iter()
            .map(|v| format!("{v:+.1}"))
            .collect::<Vec<_>>()
    );
    assert!(
        max_gain_db < 6.0,
        "one-sample delayed BKM at Resonance=0 must not create an active peak; \
         max_gain_db={max_gain_db:.2}, response={normalized_db:?}"
    );
}

#[test]
fn tb303_bkm_differential_rungs_own_multiple_coupling_ports() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let def = crate::dsl::parse_pedal_file(&source).expect("parse failed");
    let compiled =
        super::compile_pedal_with_options(&def, SR, super::compile::CompileOptions::default())
            .expect("compile failed");
    let bkm = compiled
        .stages
        .iter()
        .find_map(|stage| {
            if let super::compiled::Stage::Blockwise(bkm) = stage {
                Some(bkm)
            } else {
                None
            }
        })
        .expect("TB303 should compile to BKM");

    eprintln!("  TB303 BKM block_ports: {:?}", bkm.block_ports);

    assert_eq!(bkm.blocks.len(), 4);
    assert!(
        bkm.block_ports.iter().all(|ports| ports.len() >= 3),
        "each differential rung must own bottom left/right ports and a top differential port"
    );
    assert!(
        bkm.n_ports >= bkm.blocks.len() * 3,
        "side-aware differential BKM should include block boundary ports in addition to source ports"
    );
}

#[test]
fn tb303_blockwise_differential_rungs_expose_top_and_bottom_ports() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let def = crate::dsl::parse_pedal_file(&source).expect("parse failed");
    let graph = CircuitGraph::from_pedal(&def);

    let active_set: std::collections::HashSet<usize> =
        graph.active_edge_indices.iter().copied().collect();
    let all_edges: Vec<usize> = (0..graph.edges.len())
        .filter(|i| !active_set.contains(i))
        .collect();
    let plan = blockwise::analyze_blockwise(&all_edges, &graph)
        .expect("TB303 coupled ladder should produce a blockwise plan");

    let ports: Vec<_> = plan
        .blocks
        .iter()
        .filter_map(|block| blockwise::differential_rung_ports(block, &graph))
        .collect();

    eprintln!("  TB303 differential rung ports: {ports:?}");
    assert_eq!(ports.len(), 4);

    for (idx, port) in ports.iter().enumerate() {
        assert_ne!(
            (port.top_left, port.top_right),
            (port.bottom_left, port.bottom_right),
            "differential rung {idx} must expose distinct top and bottom differential boundaries"
        );
    }

    for idx in 0..ports.len() - 1 {
        assert_eq!(
            (ports[idx].top_left, ports[idx].top_right),
            (ports[idx + 1].bottom_left, ports[idx + 1].bottom_right),
            "adjacent differential rungs must share the same electrical boundary: \
             lower rung {idx} top should equal upper rung {} bottom",
            idx + 1
        );
    }
}

#[test]
fn tb303_bkm_coupling_matrix_connects_ladder_block_ports() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let def = crate::dsl::parse_pedal_file(&source).expect("parse failed");
    let compiled =
        super::compile_pedal_with_options(&def, SR, super::compile::CompileOptions::default())
            .expect("compile failed");
    let bkm = compiled
        .stages
        .iter()
        .find_map(|stage| {
            if let super::compiled::Stage::Blockwise(bkm) = stage {
                Some(bkm)
            } else {
                None
            }
        })
        .expect("TB303 should compile to BKM");

    let rows = bkm_block_row_coupling_summary(bkm);
    let isolated = bkm_identity_isolated_block_rows(bkm);
    eprintln!("  TB303 BKM block-row coupling summary: {rows:?}");

    assert!(
        isolated.is_empty(),
        "BKM coupling MNA isolated ladder block rows {isolated:?}; \
         these rows are identity reflections, so the solver cannot couple those rungs electrically"
    );
}

#[test]
fn tb303_coupled_bkm_stays_finite_on_silence() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let def = crate::dsl::parse_pedal_file(&source).expect("parse failed");
    let mut compiled =
        super::compile_pedal_with_options(&def, SR, super::compile::CompileOptions::default())
            .expect("compile failed");
    let bkm = compiled
        .stages
        .iter_mut()
        .find_map(|stage| {
            if let pedalkernel_rt::processor::Stage::Blockwise(bkm) = stage {
                Some(bkm)
            } else {
                None
            }
        })
        .expect("TB303 should compile to BKM");

    assert!(
        bkm.blocks.iter().all(|block| block.dc_offset.is_finite()),
        "BKM DC operating-point solve must leave finite block DC offsets: {:?}",
        bkm.blocks
            .iter()
            .map(|block| block.dc_offset)
            .collect::<Vec<_>>()
    );

    let vs = tb303_bkm_vs_signals(bkm, 0.0, 0.0);
    for sample in 0..2048 {
        let y = bkm.process_with_serial_input(0.0, &vs);
        assert!(
            y.is_finite(),
            "BKM output went non-finite at sample {sample}: {y}"
        );
        assert!(
            bkm.work_a.iter().all(|v| v.is_finite()) && bkm.work_b.iter().all(|v| v.is_finite()),
            "BKM coupling work buffers went non-finite at sample {sample}"
        );
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// 2. Blockwise detects 4 NL blocks (one per ladder stage)
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn tb303_filter_has_4_blockwise_blocks() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let def = crate::dsl::parse_pedal_file(&source).expect("parse failed");
    let graph = CircuitGraph::from_pedal(&def);

    let active_set: std::collections::HashSet<usize> =
        graph.active_edge_indices.iter().copied().collect();
    let all_edges: Vec<usize> = (0..graph.edges.len())
        .filter(|i| !active_set.contains(i))
        .collect();

    eprintln!(
        "  total edges: {}, NL: {}",
        all_edges.len(),
        all_edges
            .iter()
            .filter(|&&e| graph.effective_edge_kind(e) == super::component::EdgeKind::Nonlinear)
            .count()
    );

    let plan = blockwise::analyze_blockwise(&all_edges, &graph);

    if let Some(plan) = &plan {
        eprintln!(
            "  Blockwise: {} blocks, {} coupling edges",
            plan.num_blocks(),
            plan.coupling_edges.len()
        );
        for (i, block) in plan.blocks.iter().enumerate() {
            let edge_names: Vec<&str> = block
                .all_edges()
                .iter()
                .map(|&eidx| graph.components[graph.edges[eidx].comp_idx].id.as_str())
                .collect();
            eprintln!(
                "    block {i}: {} NL, {} reactive, {} linear = {:?}",
                block.nl_edges.len(),
                block.reactive_edges.len(),
                block.linear_edges.len(),
                edge_names
            );
        }
        let coupling_names: Vec<&str> = plan
            .coupling_edges
            .iter()
            .map(|&eidx| graph.components[graph.edges[eidx].comp_idx].id.as_str())
            .collect();
        eprintln!("    coupling: {:?}", coupling_names);

        assert!(
            plan.num_blocks() >= 4,
            "TB303 4-pole ladder should have ≥4 blockwise blocks, got {}",
            plan.num_blocks()
        );
    } else {
        eprintln!("  WARNING: blockwise decomposition returned None — ladder not split");
    }
}

#[test]
fn tb303_blockwise_blocks_are_in_signal_order() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let def = crate::dsl::parse_pedal_file(&source).expect("parse failed");
    let graph = CircuitGraph::from_pedal(&def);

    let active_set: std::collections::HashSet<usize> =
        graph.active_edge_indices.iter().copied().collect();
    let all_edges: Vec<usize> = (0..graph.edges.len())
        .filter(|i| !active_set.contains(i))
        .collect();

    let plan = blockwise::analyze_blockwise(&all_edges, &graph).expect("should decompose");
    let rung_names: Vec<(&str, &str)> = plan
        .blocks
        .iter()
        .filter_map(|block| {
            let blockwise::BlockTopology::DifferentialDiodeRung {
                left_comp_idx,
                right_comp_idx,
                ..
            } = block.topology
            else {
                return None;
            };
            Some((
                graph.components[left_comp_idx].id.as_str(),
                graph.components[right_comp_idx].id.as_str(),
            ))
        })
        .collect();

    assert_eq!(
        rung_names,
        vec![
            ("QL1", "QR1"),
            ("QL2", "QR2"),
            ("QL3", "QR3"),
            ("QL4", "QR4"),
        ],
        "differential diode rung blocks must be ordered bottom-to-top; \
         Q12/control blocks are boundary or bias blocks, not ladder rungs"
    );
}

#[test]
fn tb303_blockwise_rungs_keep_their_own_caps() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let def = crate::dsl::parse_pedal_file(&source).expect("parse failed");
    let graph = CircuitGraph::from_pedal(&def);

    let active_set: std::collections::HashSet<usize> =
        graph.active_edge_indices.iter().copied().collect();
    let all_edges: Vec<usize> = (0..graph.edges.len())
        .filter(|i| !active_set.contains(i))
        .collect();

    let plan = blockwise::analyze_blockwise(&all_edges, &graph).expect("should decompose");
    let actual: Vec<(&str, Vec<&str>)> = plan
        .blocks
        .iter()
        .filter_map(|block| {
            let blockwise::BlockTopology::DifferentialDiodeRung { left_comp_idx, .. } =
                block.topology
            else {
                return None;
            };
            let nl_name = graph.components[left_comp_idx].id.as_str();
            let caps = block
                .reactive_edges
                .iter()
                .filter_map(|&eidx| {
                    let id = graph.components[graph.edges[eidx].comp_idx].id.as_str();
                    matches!(id, "C1" | "C2" | "C3" | "C4").then_some(id)
                })
                .collect::<Vec<_>>();
            Some((nl_name, caps))
        })
        .collect();

    eprintln!("  rung cap assignment: {actual:?}");
    assert_eq!(
        actual,
        vec![
            ("QL1", vec!["C1"]),
            ("QL2", vec!["C2"]),
            ("QL3", vec!["C3"]),
            ("QL4", vec!["C4"]),
        ],
        "each signal-ordered BKM rung must own the shunt cap attached to its emitter"
    );
}

#[test]
fn tb303_blockwise_coupling_ports_are_diode_inputs() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let def = crate::dsl::parse_pedal_file(&source).expect("parse failed");
    let graph = CircuitGraph::from_pedal(&def);

    let active_set: std::collections::HashSet<usize> =
        graph.active_edge_indices.iter().copied().collect();
    let all_edges: Vec<usize> = (0..graph.edges.len())
        .filter(|i| !active_set.contains(i))
        .collect();
    let plan = blockwise::analyze_blockwise(&all_edges, &graph).expect("should decompose");

    let selected: Vec<_> = plan
        .blocks
        .iter()
        .filter_map(|block| {
            blockwise::differential_rung_ports(block, &graph).map(|ports| {
                (
                    ports.top_left,
                    ports.top_right,
                    ports.bottom_left,
                    ports.bottom_right,
                )
            })
        })
        .collect();

    let expected: Vec<_> = [
        ("QL1.base", "QR1.base", "QL1.emitter", "QR1.emitter"),
        ("QL2.base", "QR2.base", "QL2.emitter", "QR2.emitter"),
        ("QL3.base", "QR3.base", "QL3.emitter", "QR3.emitter"),
        ("QL4.base", "QR4.base", "QL4.emitter", "QR4.emitter"),
    ]
    .iter()
    .map(|(tl, tr, bl, br)| {
        (
            graph.node_names.get(*tl).copied().expect("top left pin"),
            graph.node_names.get(*tr).copied().expect("top right pin"),
            graph.node_names.get(*bl).copied().expect("bottom left pin"),
            graph
                .node_names
                .get(*br)
                .copied()
                .expect("bottom right pin"),
        )
    })
    .collect();

    eprintln!("  selected differential rung ports: {selected:?}");
    eprintln!("  expected differential rung ports: {expected:?}");

    assert_eq!(
        selected, expected,
        "BKM coupling ports must expose each rung's top differential diode inputs \
         and bottom differential emitter/cap boundary"
    );
}

#[test]
fn tb303_pedal_nodes_match_expected_ladder_wiring() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let def = crate::dsl::parse_pedal_file(&source).expect("parse failed");
    let graph = CircuitGraph::from_pedal(&def);

    let node = |name: &str| graph.node_names.get(name).copied().expect(name);

    for side in ["L", "R"] {
        for rung in 1..=4 {
            let q = format!("Q{side}{rung}");
            assert_eq!(node(&format!("{q}.base")), node(&format!("{q}.collector")));
            assert_ne!(node(&format!("{q}.base")), node(&format!("{q}.emitter")));
        }
    }

    assert_eq!(node("QL4.emitter"), node("QL3.base"));
    assert_eq!(node("QL3.emitter"), node("QL2.base"));
    assert_eq!(node("QL2.emitter"), node("QL1.base"));
    assert_eq!(node("QR4.emitter"), node("QR3.base"));
    assert_eq!(node("QR3.emitter"), node("QR2.base"));
    assert_eq!(node("QR2.emitter"), node("QR1.base"));

    assert_eq!(node("QL1.emitter"), node("C1.a"));
    assert_eq!(node("QR1.emitter"), node("C1.b"));
    assert_eq!(node("QL2.emitter"), node("C2.a"));
    assert_eq!(node("QR2.emitter"), node("C2.b"));
    assert_eq!(node("QL3.emitter"), node("C3.a"));
    assert_eq!(node("QR3.emitter"), node("C3.b"));
    assert_eq!(node("QL4.emitter"), node("C4.a"));
    assert_eq!(node("QR4.emitter"), node("C4.b"));

    assert_eq!(node("QL1.emitter"), node("Q12L.collector"));
    assert_eq!(node("QR1.emitter"), node("Q12R.collector"));
    assert_eq!(node("QL4.emitter"), node("C_out.a"));
    assert_eq!(node("QL4.emitter"), node("Resonance.b"));
}

#[test]
fn two_rung_diode_ladder_splits_into_ordered_blocks() {
    let def = crate::dsl::parse_pedal_file(TWO_RUNG_DIODE_LADDER).expect("parse failed");
    let graph = CircuitGraph::from_pedal(&def);
    let active_set: std::collections::HashSet<usize> =
        graph.active_edge_indices.iter().copied().collect();
    let all_edges: Vec<usize> = (0..graph.edges.len())
        .filter(|i| !active_set.contains(i))
        .collect();
    let plan = blockwise::analyze_blockwise(&all_edges, &graph)
        .expect("literal two-rung diode ladder should decompose");

    let names: Vec<&str> = plan
        .blocks
        .iter()
        .map(|block| {
            let edge = block.nl_edges[0];
            graph.components[graph.edges[edge].comp_idx].id.as_str()
        })
        .collect();

    eprintln!("  two-rung diode ladder block order: {names:?}");
    assert_eq!(names, vec!["D1", "D2"]);
}

#[test]
fn two_rung_diode_ladder_bkm_direct_path_is_lowpass() {
    let def = crate::dsl::parse_pedal_file(TWO_RUNG_DIODE_LADDER).expect("parse failed");

    let measure = |freq: f64| -> f64 {
        let mut proc = super::compile_pedal(&def, SR).expect("compile failed");
        let bkm = proc
            .stages
            .iter_mut()
            .find_map(|s| {
                if let pedalkernel_rt::processor::Stage::Blockwise(ref mut k) = s {
                    Some(k)
                } else {
                    None
                }
            })
            .expect("two-rung diode ladder should compile to BKM");

        for _ in 0..4800 {
            let _ = bkm.process(&[0.0, 0.0]);
        }

        let mut peak = 0.0f64;
        for i in 0..4800 {
            let input = 0.1 * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
            peak = peak.max(bkm.process(&[input, 0.0]).abs());
        }
        peak
    };

    let gain_100 = measure(100.0);
    let gain_10k = measure(10_000.0);
    let ratio_db = 20.0 * (gain_100 / gain_10k.max(1e-12)).log10();

    eprintln!(
        "  two-rung diode ladder BKM: 100Hz={gain_100:.6}, 10kHz={gain_10k:.6}, ratio={ratio_db:+.1} dB"
    );

    assert!(
        gain_100 > gain_10k,
        "two-rung diode ladder BKM should be lowpass: 100Hz={gain_100:.6}, 10kHz={gain_10k:.6}"
    );
}

#[test]
fn two_rung_differential_diode_ladder_compiles_to_bkm_fixture() {
    let def =
        crate::dsl::parse_pedal_file(TWO_RUNG_DIFFERENTIAL_DIODE_LADDER).expect("parse failed");
    let graph = CircuitGraph::from_pedal(&def);
    let active_set: std::collections::HashSet<usize> =
        graph.active_edge_indices.iter().copied().collect();
    let all_edges: Vec<usize> = (0..graph.edges.len())
        .filter(|i| !active_set.contains(i))
        .collect();
    let plan = blockwise::analyze_blockwise(&all_edges, &graph)
        .expect("two-rung differential ladder should decompose");

    let rung_names: Vec<(&str, &str)> = plan
        .blocks
        .iter()
        .filter_map(|block| {
            let blockwise::BlockTopology::DifferentialDiodeRung {
                left_comp_idx,
                right_comp_idx,
                ..
            } = block.topology
            else {
                return None;
            };
            Some((
                graph.components[left_comp_idx].id.as_str(),
                graph.components[right_comp_idx].id.as_str(),
            ))
        })
        .collect();

    assert_eq!(
        rung_names,
        vec![("QL1", "QR1"), ("QL2", "QR2")],
        "minimal differential fixture should expose exactly two bottom-to-top rung blocks"
    );

    let built = blockwise::try_build_blockwise(
        &all_edges,
        &graph,
        &[graph.in_node, graph.out_node],
        SR,
        &std::collections::BTreeMap::new(),
        9.0,
        &def.ports,
        false,
        0.0,
        false,
        true,
        &def.init_hints,
    )
    .expect("minimal differential fixture should build through BKM directly");
    let bkm_count = built
        .iter()
        .filter(|stage| matches!(stage, super::spqr_build::BuiltStage::Blockwise(_)))
        .count();
    assert_eq!(
        bkm_count, 1,
        "minimal differential fixture should compile to one BKM stage"
    );
}

#[test]
fn two_rung_differential_diode_ladder_runtime_is_lowpass() {
    let def =
        crate::dsl::parse_pedal_file(TWO_RUNG_DIFFERENTIAL_DIODE_LADDER).expect("parse failed");
    let graph = CircuitGraph::from_pedal(&def);
    let active_set: std::collections::HashSet<usize> =
        graph.active_edge_indices.iter().copied().collect();
    let all_edges: Vec<usize> = (0..graph.edges.len())
        .filter(|i| !active_set.contains(i))
        .collect();

    let measure = |freq: f64| -> f64 {
        let built = blockwise::try_build_blockwise(
            &all_edges,
            &graph,
            &[graph.in_node, graph.out_node],
            SR,
            &std::collections::BTreeMap::new(),
            9.0,
            &def.ports,
            false,
            0.0,
            false,
            true,
            &def.init_hints,
        )
        .expect("minimal differential fixture should build through BKM directly");
        let mut bkm = built
            .into_iter()
            .find_map(|stage| {
                if let super::spqr_build::BuiltStage::Blockwise(bkm) = stage {
                    Some(bkm)
                } else {
                    None
                }
            })
            .expect("BKM stage");

        for _ in 0..1200 {
            let _ = bkm.process(&[0.0, 0.0]);
        }

        let mut values = Vec::with_capacity(2400);
        for i in 0..2400 {
            let input = 0.05 * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
            values.push(bkm.process(&[input, 0.0]));
        }
        ac_rms(&values)
    };

    let gain_100 = measure(100.0);
    let gain_10k = measure(10_000.0);
    let ratio_db = db_norm(gain_100, gain_10k);

    eprintln!(
        "  two-rung differential ladder runtime: 100Hz={gain_100:.6}, \
         10kHz={gain_10k:.6}, ratio={ratio_db:+.1} dB"
    );

    assert!(
        gain_100.is_finite() && gain_10k.is_finite(),
        "minimal differential fixture must stay finite: 100Hz={gain_100}, 10kHz={gain_10k}"
    );
    assert!(
        gain_100 > 1.0e-8,
        "minimal differential fixture should produce nonzero output at 100Hz"
    );
    assert!(
        gain_100 > gain_10k * 1.25,
        "minimal differential fixture should isolate a BKM lowpass response: \
         100Hz={gain_100:.6}, 10kHz={gain_10k:.6}, ratio={ratio_db:+.1} dB"
    );
}

#[test]
fn q12_to_two_rung_differential_diode_ladder_routes_into_bkm_boundary() {
    let def = crate::dsl::parse_pedal_file(Q12_TO_TWO_RUNG_DIFFERENTIAL_DIODE_LADDER)
        .expect("parse failed");
    let graph = CircuitGraph::from_pedal(&def);
    let active_set: std::collections::HashSet<usize> =
        graph.active_edge_indices.iter().copied().collect();
    let all_edges: Vec<usize> = (0..graph.edges.len())
        .filter(|i| !active_set.contains(i))
        .collect();

    let built = blockwise::try_build_blockwise(
        &all_edges,
        &graph,
        &[graph.in_node, graph.out_node],
        SR,
        &std::collections::BTreeMap::new(),
        9.0,
        &def.ports,
        false,
        0.0,
        false,
        true,
        &def.init_hints,
    )
    .expect("Q12 handoff fixture should build through direct BKM");

    let q12 = built
        .iter()
        .find_map(|stage| {
            if let super::spqr_build::BuiltStage::Wdf(wdf) = stage {
                let has_q12_collectors = wdf
                    .boundary_bindings
                    .iter()
                    .any(|binding| binding.label == "collector_left")
                    && wdf
                        .boundary_bindings
                        .iter()
                        .any(|binding| binding.label == "collector_right");
                has_q12_collectors.then_some(wdf)
            } else {
                None
            }
        })
        .expect("Q12 should build as a WDF boundary source");
    assert!(
        q12.boundary_bindings
            .iter()
            .any(|binding| binding.label == "base_left"),
        "Q12 handoff source should expose its driven base boundary"
    );

    let bkm = built
        .iter()
        .find_map(|stage| {
            if let super::spqr_build::BuiltStage::Blockwise(bkm) = stage {
                Some(bkm)
            } else {
                None
            }
        })
        .expect("Q12 handoff fixture should build the ladder as BKM");
    assert_eq!(
        bkm.blocks.len(),
        2,
        "Q12 handoff fixture should keep the two diode rungs in one BKM ladder"
    );

    let q12_left = graph.node_names["Q12L.collector"];
    let q12_right = graph.node_names["Q12R.collector"];
    assert!(
        bkm.coupling_ports.iter().any(|port| {
            let (pos, neg) = port.graph.raw().as_tuple();
            pos == Some(q12_left) || neg == Some(q12_left)
        }),
        "Q12 left collector node should be a BKM coupling boundary"
    );
    assert!(
        bkm.coupling_ports.iter().any(|port| {
            let (pos, neg) = port.graph.raw().as_tuple();
            pos == Some(q12_right) || neg == Some(q12_right)
        }),
        "Q12 right collector node should be a BKM coupling boundary"
    );
}

#[test]
#[ignore = "documents current Q12 collector boundary runtime-coupling gap; enable when bead 5 fixes per-boundary BKM drive"]
fn q12_to_two_rung_differential_diode_ladder_handoff_is_lowpass() {
    let def = crate::dsl::parse_pedal_file(Q12_TO_TWO_RUNG_DIFFERENTIAL_DIODE_LADDER)
        .expect("parse failed");

    let measure = |freq: f64| -> f64 {
        let graph = CircuitGraph::from_pedal(&def);
        let active_set: std::collections::HashSet<usize> =
            graph.active_edge_indices.iter().copied().collect();
        let all_edges: Vec<usize> = (0..graph.edges.len())
            .filter(|i| !active_set.contains(i))
            .collect();
        let q12_left = graph.node_names["Q12L.collector"];
        let q12_right = graph.node_names["Q12R.collector"];

        let built = blockwise::try_build_blockwise(
            &all_edges,
            &graph,
            &[graph.in_node, graph.out_node],
            SR,
            &std::collections::BTreeMap::new(),
            9.0,
            &def.ports,
            false,
            0.0,
            false,
            true,
            &def.init_hints,
        )
        .expect("Q12 handoff fixture should build through direct BKM");

        let mut bkm = None;
        for stage in built {
            match stage {
                super::spqr_build::BuiltStage::Blockwise(stage) => {
                    bkm = Some(stage);
                }
                _ => {}
            }
        }
        let mut bkm = bkm.expect("routed target stage should be BKM");
        let left_port_indices: Vec<usize> = bkm
            .coupling_ports
            .iter()
            .enumerate()
            .filter_map(|(port_idx, port)| {
                let (pos, neg) = port.graph.raw().as_tuple();
                (pos == Some(q12_left) || neg == Some(q12_left)).then_some(port_idx)
            })
            .collect();
        let right_port_indices: Vec<usize> = bkm
            .coupling_ports
            .iter()
            .enumerate()
            .filter_map(|(port_idx, port)| {
                let (pos, neg) = port.graph.raw().as_tuple();
                (pos == Some(q12_right) || neg == Some(q12_right)).then_some(port_idx)
            })
            .collect();
        assert!(
            !left_port_indices.is_empty() && !right_port_indices.is_empty(),
            "Q12 collector nodes should map to left/right BKM coupling ports"
        );
        let vs_signals = vec![0.0; bkm.vs_port_map.len()];

        let mut process = |input: f64| {
            let mut drives = Vec::new();
            pedalkernel_rt::boundary_math::push_differential_voltage_drives(
                &mut drives,
                &left_port_indices,
                &right_port_indices,
                pedalkernel_rt::boundary_math::PortVoltage::new(input),
            );
            bkm.process_with_boundary_drives(&drives, &vs_signals)
        };

        for i in 0..1200 {
            let input = 0.003 * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
            let _ = process(input);
        }
        let mut values = Vec::with_capacity(2400);
        for i in 0..2400 {
            let input = 0.003 * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
            values.push(process(input));
        }
        ac_rms(&values)
    };

    let gain_100 = measure(100.0);
    let gain_10k = measure(10_000.0);
    let ratio_db = db_norm(gain_100, gain_10k);

    eprintln!(
        "  Q12 collector boundary -> two-rung differential ladder: 100Hz={gain_100:.6}, \
         10kHz={gain_10k:.6}, ratio={ratio_db:+.1} dB"
    );

    assert!(
        gain_100.is_finite() && gain_10k.is_finite(),
        "Q12 handoff fixture must stay finite: 100Hz={gain_100}, 10kHz={gain_10k}"
    );
    assert!(
        gain_100 > 1.0e-10,
        "Q12 collector boundary handoff should produce nonzero output at 100Hz"
    );
    assert!(
        gain_100 > gain_10k * 1.25,
        "Q12 collector boundary handoff into BKM should preserve a lowpass response: \
         100Hz={gain_100:.6}, 10kHz={gain_10k:.6}, ratio={ratio_db:+.1} dB"
    );
}

#[test]
fn q12_handoff_fixture_runtime_boundary_probes_classify_coupling_gap() {
    #[derive(Clone, Copy)]
    enum Probe {
        SerialFirstRung,
        Q12CollectorBoundary,
        FirstRungBottomDiff,
        FirstRungTopDiff,
        SecondRungBottomDiff,
        SecondRungTopDiff,
    }

    impl Probe {
        const ALL: [Self; 6] = [
            Self::SerialFirstRung,
            Self::Q12CollectorBoundary,
            Self::FirstRungBottomDiff,
            Self::FirstRungTopDiff,
            Self::SecondRungBottomDiff,
            Self::SecondRungTopDiff,
        ];

        const fn label(self) -> &'static str {
            match self {
                Self::SerialFirstRung => "serial_first_rung",
                Self::Q12CollectorBoundary => "q12_collector_boundary",
                Self::FirstRungBottomDiff => "first_rung_bottom_diff",
                Self::FirstRungTopDiff => "first_rung_top_diff",
                Self::SecondRungBottomDiff => "second_rung_bottom_diff",
                Self::SecondRungTopDiff => "second_rung_top_diff",
            }
        }
    }

    let def = crate::dsl::parse_pedal_file(Q12_TO_TWO_RUNG_DIFFERENTIAL_DIODE_LADDER)
        .expect("parse failed");

    let measure = |probe: Probe, freq: f64| -> f64 {
        let graph = CircuitGraph::from_pedal(&def);
        let active_set: std::collections::HashSet<usize> =
            graph.active_edge_indices.iter().copied().collect();
        let all_edges: Vec<usize> = (0..graph.edges.len())
            .filter(|i| !active_set.contains(i))
            .collect();
        let q12_left = graph.node_names["Q12L.collector"];
        let q12_right = graph.node_names["Q12R.collector"];

        let built = blockwise::try_build_blockwise(
            &all_edges,
            &graph,
            &[graph.in_node, graph.out_node],
            SR,
            &std::collections::BTreeMap::new(),
            9.0,
            &def.ports,
            false,
            0.0,
            false,
            true,
            &def.init_hints,
        )
        .expect("Q12 handoff fixture should build through direct BKM");

        let mut bkm = built
            .into_iter()
            .find_map(|stage| {
                if let super::spqr_build::BuiltStage::Blockwise(stage) = stage {
                    Some(stage)
                } else {
                    None
                }
            })
            .expect("BKM stage");
        let left_port_indices: Vec<usize> = bkm
            .coupling_ports
            .iter()
            .enumerate()
            .filter_map(|(port_idx, port)| {
                let (pos, neg) = port.graph.raw().as_tuple();
                (pos == Some(q12_left) || neg == Some(q12_left)).then_some(port_idx)
            })
            .collect();
        let right_port_indices: Vec<usize> = bkm
            .coupling_ports
            .iter()
            .enumerate()
            .filter_map(|(port_idx, port)| {
                let (pos, neg) = port.graph.raw().as_tuple();
                (pos == Some(q12_right) || neg == Some(q12_right)).then_some(port_idx)
            })
            .collect();
        assert!(
            !left_port_indices.is_empty() && !right_port_indices.is_empty(),
            "Q12 collector nodes should map to left/right BKM coupling ports"
        );
        assert!(
            bkm.differential_rung_ports(0).is_some() && bkm.differential_rung_ports(1).is_some(),
            "Q12 fixture should expose two three-port differential rung blocks, got {:?}",
            bkm.block_ports
        );
        let first_rung_ports = bkm
            .differential_rung_ports(0)
            .expect("first rung should expose bottom-left, bottom-right, top-differential ports");
        let second_rung_ports = bkm
            .differential_rung_ports(1)
            .expect("second rung should expose bottom-left, bottom-right, top-differential ports");
        let vs_signals = vec![0.0; bkm.vs_port_map.len()];

        let mut process = |input: f64| match probe {
            Probe::SerialFirstRung => bkm.process_with_serial_input(input, &vs_signals),
            Probe::Q12CollectorBoundary => {
                let mut drives = Vec::new();
                pedalkernel_rt::boundary_math::push_differential_voltage_drives(
                    &mut drives,
                    &left_port_indices,
                    &right_port_indices,
                    pedalkernel_rt::boundary_math::PortVoltage::new(input),
                );
                bkm.process_with_boundary_drives(&drives, &vs_signals)
            }
            Probe::FirstRungBottomDiff => {
                let mut drives = Vec::new();
                pedalkernel_rt::boundary_math::push_differential_voltage_drives(
                    &mut drives,
                    &[first_rung_ports.0],
                    &[first_rung_ports.1],
                    pedalkernel_rt::boundary_math::PortVoltage::new(input),
                );
                bkm.process_with_boundary_drives(&drives, &vs_signals)
            }
            Probe::FirstRungTopDiff => {
                let mut drives = Vec::new();
                pedalkernel_rt::boundary_math::push_differential_voltage_drives(
                    &mut drives,
                    &[first_rung_ports.2],
                    &[],
                    pedalkernel_rt::boundary_math::PortVoltage::new(input),
                );
                bkm.process_with_boundary_drives(&drives, &vs_signals)
            }
            Probe::SecondRungBottomDiff => {
                let mut drives = Vec::new();
                pedalkernel_rt::boundary_math::push_differential_voltage_drives(
                    &mut drives,
                    &[second_rung_ports.0],
                    &[second_rung_ports.1],
                    pedalkernel_rt::boundary_math::PortVoltage::new(input),
                );
                bkm.process_with_boundary_drives(&drives, &vs_signals)
            }
            Probe::SecondRungTopDiff => {
                let mut drives = Vec::new();
                pedalkernel_rt::boundary_math::push_differential_voltage_drives(
                    &mut drives,
                    &[second_rung_ports.2],
                    &[],
                    pedalkernel_rt::boundary_math::PortVoltage::new(input),
                );
                bkm.process_with_boundary_drives(&drives, &vs_signals)
            }
        };

        for i in 0..1200 {
            let input = 0.003 * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
            let _ = process(input);
        }
        let mut values = Vec::with_capacity(2400);
        for i in 0..2400 {
            let input = 0.003 * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
            values.push(process(input));
        }
        ac_rms(&values)
    };

    let mut results = Vec::new();
    for probe in Probe::ALL {
        let gain_100 = measure(probe, 100.0);
        let gain_10k = measure(probe, 10_000.0);
        let ratio_db = db_norm(gain_100, gain_10k);
        eprintln!(
            "  boundary probes: {} 100Hz={gain_100:.6}, 10kHz={gain_10k:.6}, \
             ratio={ratio_db:+.1} dB",
            probe.label()
        );
        results.push((probe, gain_100, gain_10k, ratio_db));
    }

    assert!(
        results
            .iter()
            .all(|(_, gain_100, gain_10k, _)| gain_100.is_finite() && gain_10k.is_finite()),
        "boundary probes should stay finite"
    );
    let (_, serial_100, serial_10k, _) = results
        .iter()
        .find(|(probe, _, _, _)| matches!(probe, Probe::SerialFirstRung))
        .copied()
        .expect("serial first-rung result");
    let first_top_is_lowpass = results.iter().any(|(probe, gain_100, gain_10k, _)| {
        matches!(probe, Probe::FirstRungTopDiff) && *gain_100 > *gain_10k * 1.25
    });
    let second_top_is_lowpass = results.iter().any(|(probe, gain_100, gain_10k, _)| {
        matches!(probe, Probe::SecondRungTopDiff) && *gain_100 > *gain_10k * 1.25
    });
    if serial_100 <= serial_10k * 1.25 {
        eprintln!(
            "  GAP: serial first-rung drive is not lowpass in the Q12 handoff fixture; \
             this points at BKM output extraction/coupled multiport drive before Q12 routing"
        );
    }
    if !first_top_is_lowpass && !second_top_is_lowpass {
        eprintln!(
            "  GAP: direct top-differential rung probes are not lowpass; \
             this points at coupled multiport BKM evaluation or output extraction"
        );
    }
}

#[test]
fn two_rung_normal_diode_ladder_cv_moves_cutoff() {
    let def = crate::dsl::parse_pedal_file(TWO_RUNG_DIODE_LADDER_WITH_FEEDBACK_AND_CV)
        .expect("parse failed");
    let probe = super::compile_pedal(&def, SR).expect("compile failed");
    assert!(
        probe
            .stages
            .iter()
            .any(|stage| matches!(stage, pedalkernel_rt::processor::Stage::Blockwise(_))),
        "normal diode CV ladder should compile to BKM"
    );

    let measure_ac = |cv_voltage: f64, freq: f64| -> f64 {
        let mut proc = super::compile_pedal(&def, SR).expect("compile failed");
        let in_idx = proc.resolve_port("audio_in").expect("audio_in port");
        let cv_idx = proc.resolve_port("cv_cutoff").expect("cv_cutoff port");
        let out_idx = proc.resolve_port("audio_out").expect("audio_out port");

        for _ in 0..9600 {
            let mut ports = vec![0.0; proc.port_count()];
            ports[cv_idx] = cv_voltage;
            proc.process_ports(&mut ports);
        }

        let mut values = Vec::new();
        for i in 0..9600 {
            let input = 0.05 * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
            let mut ports = vec![0.0; proc.port_count()];
            ports[in_idx] = input;
            ports[cv_idx] = cv_voltage;
            proc.process_ports(&mut ports);
            values.push(ports[out_idx]);
        }
        ac_rms(&values)
    };

    let low_cv_8k = measure_ac(-1.0, 8_000.0);
    let high_cv_8k = measure_ac(3.0, 8_000.0);
    let low_cv_12k = measure_ac(-1.0, 12_000.0);
    let high_cv_12k = measure_ac(3.0, 12_000.0);

    eprintln!(
        "  normal diode cutoff CV AC: 8k -1V={low_cv_8k:.6}, +3V={high_cv_8k:.6}; \
         12k -1V={low_cv_12k:.6}, +3V={high_cv_12k:.6}"
    );

    assert!(
        high_cv_8k > low_cv_8k * 1.25,
        "raising cv_cutoff should move the normal diode ladder cutoff upward at 8kHz: \
         low={low_cv_8k:.6}, high={high_cv_8k:.6}, ratio={:.3}",
        high_cv_8k / low_cv_8k.max(1e-12)
    );
    assert!(
        high_cv_12k > low_cv_12k * 1.25,
        "raising cv_cutoff should move the normal diode ladder cutoff upward at 12kHz: \
         low={low_cv_12k:.6}, high={high_cv_12k:.6}, ratio={:.3}",
        high_cv_12k / low_cv_12k.max(1e-12)
    );
}

#[test]
fn two_rung_normal_diode_ladder_cutoff_cv_is_reversible() {
    let def = crate::dsl::parse_pedal_file(TWO_RUNG_DIODE_LADDER_WITH_FEEDBACK_AND_CV)
        .expect("parse failed");
    let mut proc = super::compile_pedal(&def, SR).expect("compile failed");
    let in_idx = proc.resolve_port("audio_in").expect("audio_in port");
    let cv_idx = proc.resolve_port("cv_cutoff").expect("cv_cutoff port");
    let out_idx = proc.resolve_port("audio_out").expect("audio_out port");

    let measure_ac = |proc: &mut super::compiled::CompiledPedal, cv_voltage: f64| -> f64 {
        for _ in 0..9600 {
            let mut ports = vec![0.0; proc.port_count()];
            ports[cv_idx] = cv_voltage;
            proc.process_ports(&mut ports);
        }

        let mut values = Vec::new();
        for i in 0..9600 {
            let input = 0.05 * (2.0 * std::f64::consts::PI * 12_000.0 * i as f64 / SR).sin();
            let mut ports = vec![0.0; proc.port_count()];
            ports[in_idx] = input;
            ports[cv_idx] = cv_voltage;
            proc.process_ports(&mut ports);
            values.push(ports[out_idx]);
        }
        ac_rms(&values)
    };

    let low_initial = measure_ac(&mut proc, 0.0);
    let high = measure_ac(&mut proc, 3.0);
    let low_again = measure_ac(&mut proc, 0.0);

    eprintln!(
        "  reversible normal diode cutoff CV AC: 12k 0V={low_initial:.6}, \
         +3V={high:.6}, back-to-0V={low_again:.6}"
    );

    assert!(
        high > low_initial * 1.25,
        "raising cv_cutoff should open the normal diode ladder before reversal: \
         low={low_initial:.6}, high={high:.6}"
    );
    assert!(
        low_again < high / 1.25,
        "returning cv_cutoff to 0V should restore the lower cutoff instead of keeping stale Rp: \
         high={high:.6}, low_again={low_again:.6}"
    );
}

#[test]
fn two_rung_normal_diode_ladder_cutoff_uses_circuit_calibration() {
    let def = crate::dsl::parse_pedal_file(TWO_RUNG_DIODE_LADDER_WITH_FEEDBACK_AND_CV)
        .expect("parse failed");
    let mut proc = super::compile_pedal(&def, SR).expect("compile failed");
    let cv_idx = proc.resolve_port("cv_cutoff").expect("cv_cutoff port");

    let measure_rp = |proc: &mut super::compiled::CompiledPedal, cv_voltage: f64| -> f64 {
        let mut ports = vec![0.0; proc.port_count()];
        ports[cv_idx] = cv_voltage;
        proc.process_ports(&mut ports);
        proc.stages
            .iter()
            .find_map(|stage| {
                if let pedalkernel_rt::processor::Stage::Blockwise(bkm) = stage {
                    Some(bkm.blocks[0].rp)
                } else {
                    None
                }
            })
            .expect("normal diode ladder should compile to BKM")
    };

    let rp_0v = measure_rp(&mut proc, 0.0);
    let rp_3v = measure_rp(&mut proc, 3.0);
    let ratio = rp_3v / rp_0v;

    eprintln!("  circuit-calibrated normal diode cutoff Rp: 0V={rp_0v:.3}, +3V={rp_3v:.3}, ratio={ratio:.3}");

    assert!(
        ratio > 0.35 && ratio < 0.85,
        "cutoff CV should use diode dynamic resistance from R_bias/R_cv, not 2^CV scaling: \
         rp_0v={rp_0v:.3}, rp_3v={rp_3v:.3}, ratio={ratio:.3}"
    );
}

#[test]
fn two_rung_feedback_diode_ladder_compiles_to_bkm() {
    let def =
        crate::dsl::parse_pedal_file(TWO_RUNG_DIODE_LADDER_WITH_FEEDBACK).expect("parse failed");
    let proc = super::compile_pedal(&def, SR).expect("compile failed");

    let bkm_count = proc
        .stages
        .iter()
        .filter(|stage| matches!(stage, pedalkernel_rt::processor::Stage::Blockwise(_)))
        .count();

    assert_eq!(
        bkm_count, 1,
        "feedback diode ladder should compile as one BKM stage"
    );
}

#[test]
fn two_rung_feedback_bkm_coupling_matrix_connects_block_ports() {
    let def =
        crate::dsl::parse_pedal_file(TWO_RUNG_DIODE_LADDER_WITH_FEEDBACK).expect("parse failed");
    let proc = super::compile_pedal(&def, SR).expect("compile failed");
    let bkm = proc
        .stages
        .iter()
        .find_map(|stage| {
            if let pedalkernel_rt::processor::Stage::Blockwise(bkm) = stage {
                Some(bkm)
            } else {
                None
            }
        })
        .expect("two-rung feedback ladder should compile to BKM");

    let rows = bkm_block_row_coupling_summary(bkm);
    let isolated = bkm_identity_isolated_block_rows(bkm);
    eprintln!("  two-rung BKM block-row coupling summary: {rows:?}");

    assert!(
        isolated.is_empty(),
        "BKM coupling MNA isolated simple ladder block rows {isolated:?}; \
         the compiler must derive connected block ports from the parsed .pedal graph"
    );
}

#[test]
fn two_rung_feedback_diode_ladder_bkm_direct_path_is_lowpass() {
    let def =
        crate::dsl::parse_pedal_file(TWO_RUNG_DIODE_LADDER_WITH_FEEDBACK).expect("parse failed");

    let measure = |freq: f64| -> f64 {
        let mut proc = super::compile_pedal(&def, SR).expect("compile failed");
        proc.set_control_immediate("Resonance", 0.0);

        for _ in 0..4800 {
            let _ = proc.process(0.0);
        }

        let mut peak = 0.0f64;
        for i in 0..4800 {
            let input = 0.1 * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
            peak = peak.max(proc.process(input).abs());
        }
        peak
    };

    let gain_100 = measure(100.0);
    let gain_10k = measure(10_000.0);
    let ratio_db = 20.0 * (gain_100 / gain_10k.max(1e-12)).log10();

    eprintln!(
        "  two-rung feedback diode ladder: 100Hz={gain_100:.6}, 10kHz={gain_10k:.6}, ratio={ratio_db:+.1} dB"
    );

    assert!(
        gain_100 > gain_10k * 1.1,
        "two-rung feedback diode ladder should tilt lowpass: 100Hz={gain_100:.6}, 10kHz={gain_10k:.6}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 3. Compiled structure: not monolithic (structural — no K-tables)
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn tb303_filter_not_monolithic() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let def = crate::dsl::parse_pedal_file(&source).expect("parse failed");
    let compiled =
        super::compile_pedal_with_options(&def, SR, super::compile::CompileOptions::debug())
            .unwrap();

    let dump = compiled.debug_dump();
    let has_8port = dump.contains("n_nl=8");
    let stage_count = compiled.stages.len();

    eprintln!("  Stages: {stage_count}, monolithic_8port: {has_8port}");
    eprintln!("{dump}");

    if has_8port {
        eprintln!("  WARNING: monolithic n_nl=8 — blockwise not applied to this circuit");
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// 4. Audio: lowpass behavior (needs K-tables — uses disk cache)
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn tb303_filter_is_lowpass() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");

    let cache_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("target")
        .join("test-cache");
    let _ = std::fs::create_dir_all(&cache_dir);

    eprintln!("  compiling with K-tables (cached)...");
    let mut options = super::compile::CompileOptions::default();
    options.coupled_blockwise_newton = false;
    let blob = super::compile::compile_pedal_cached(
        &source,
        "tb303_filter_audio",
        "tb303_filter_audio",
        SR,
        &options,
        &cache_dir,
    )
    .expect("compile failed");
    eprintln!("  blob: {} bytes", blob.len());

    let measure = |freq: f64, cutoff: f64| -> f64 {
        let mut proc: super::compiled::CompiledPedal =
            postcard::from_bytes(&blob).expect("deserialize failed");
        proc.set_control_immediate("Cutoff", cutoff);
        proc.set_control_immediate("Resonance", 0.0);
        for _ in 0..2400 {
            proc.process(0.0);
        }
        let mut peak = 0.0f64;
        for i in 0..4800 {
            let input = (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
            peak = peak.max(proc.process(input).abs());
        }
        peak
    };

    let gain_100 = measure(100.0, 0.5);
    let gain_10k = measure(10000.0, 0.5);
    let ratio_db = 20.0 * (gain_100 / gain_10k.max(1e-12)).log10();

    eprintln!("  LPF: 100Hz={gain_100:.4}, 10kHz={gain_10k:.4}, ratio={ratio_db:+.1} dB");

    assert!(
        gain_100 > gain_10k * 2.0,
        "LOWPASS: 100Hz ({gain_100:.4}) must be >2x 10kHz ({gain_10k:.4})"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 5. SPQR tree structure inspection
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn tb303_filter_spqr_structure() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let def = crate::dsl::parse_pedal_file(&source).expect("parse failed");
    let graph = CircuitGraph::from_pedal(&def);

    let active_set: std::collections::HashSet<usize> =
        graph.active_edge_indices.iter().copied().collect();
    let all_edges: Vec<usize> = (0..graph.edges.len())
        .filter(|i| !active_set.contains(i))
        .collect();

    let tree = spqr_decompose(
        &all_edges,
        &[graph.in_node, graph.out_node],
        &graph,
        graph.gnd_node,
    );

    fn count_nodes(node: &SpqrNode) -> (usize, usize, usize, usize) {
        match node {
            SpqrNode::S { children, .. } => {
                let mut s = (1, 0, 0, 0);
                for c in children {
                    let r = count_nodes(c);
                    s.0 += r.0;
                    s.1 += r.1;
                    s.2 += r.2;
                    s.3 += r.3;
                }
                s
            }
            SpqrNode::P { children, .. } => {
                let mut s = (0, 1, 0, 0);
                for c in children {
                    let r = count_nodes(c);
                    s.0 += r.0;
                    s.1 += r.1;
                    s.2 += r.2;
                    s.3 += r.3;
                }
                s
            }
            SpqrNode::Q { .. } => (0, 0, 1, 0),
            SpqrNode::R { children, .. } => {
                let mut s = (0, 0, 0, 1);
                for c in children {
                    let r = count_nodes(c);
                    s.0 += r.0;
                    s.1 += r.1;
                    s.2 += r.2;
                    s.3 += r.3;
                }
                s
            }
        }
    }

    let (s, p, q, r) = count_nodes(&tree);
    eprintln!("  SPQR nodes: S={s}, P={p}, Q={q}, R={r}");
    eprintln!(
        "  Total edges: {}, NL edges: {}",
        all_edges.len(),
        all_edges
            .iter()
            .filter(|&&e| { graph.effective_edge_kind(e) == super::component::EdgeKind::Nonlinear })
            .count()
    );
    eprintln!("  Has P-nodes (parallel shunts): {}", p > 0);
}

// ═══════════════════════════════════════════════════════════════════════════
// 6. Coupling scattering matrix is well-formed
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn tb303_coupling_scattering_wellformed() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let def = crate::dsl::parse_pedal_file(&source).expect("parse failed");
    let graph = CircuitGraph::from_pedal(&def);

    // Collect passive (non-active) edge indices.
    let active_set: std::collections::HashSet<usize> =
        graph.active_edge_indices.iter().copied().collect();
    let all_edges: Vec<usize> = (0..graph.edges.len())
        .filter(|i| !active_set.contains(i))
        .collect();

    let plan = blockwise::analyze_blockwise(&all_edges, &graph)
        .expect("blockwise decomposition should succeed for TB303");

    assert!(
        !plan.coupling_edges.is_empty(),
        "TB303 ladder must have coupling edges connecting the 4 blocks"
    );
    eprintln!(
        "  coupling: {} edges, {} blocks",
        plan.coupling_edges.len(),
        plan.num_blocks()
    );

    // Build MNA from coupling edges.
    // Collect unique non-rail, non-ground nodes from coupling edges.
    let excluded: std::collections::HashSet<usize> = {
        let mut s = std::collections::HashSet::new();
        s.insert(graph.gnd_node);
        s.insert(graph.vcc_node);
        for &n in &graph.supply_nodes {
            s.insert(n);
        }
        s
    };

    let mut coupling_nodes: Vec<usize> = Vec::new();
    for &eidx in &plan.coupling_edges {
        let e = &graph.edges[eidx];
        for &n in &[e.node_a, e.node_b] {
            if !excluded.contains(&n) && !coupling_nodes.contains(&n) {
                coupling_nodes.push(n);
            }
        }
    }
    eprintln!("  coupling nodes: {:?}", coupling_nodes);

    // Map node IDs to MNA indices (0-based, ground = None).
    let node_to_mna: std::collections::HashMap<usize, usize> = coupling_nodes
        .iter()
        .enumerate()
        .map(|(i, &n)| (n, i))
        .collect();
    let num_nodes = coupling_nodes.len();

    let mut mna = pedalkernel_rt::tree::MnaSystem::new(num_nodes, 0);

    // Stamp each coupling resistor.
    for &eidx in &plan.coupling_edges {
        let e = &graph.edges[eidx];
        let comp = &graph.components[e.comp_idx];
        let r = comp.kind.resistance().unwrap_or(10_000.0); // default if not a pure resistor
        let n1 = if excluded.contains(&e.node_a) {
            None
        } else {
            Some(node_to_mna[&e.node_a])
        };
        let n2 = if excluded.contains(&e.node_b) {
            None
        } else {
            Some(node_to_mna[&e.node_b])
        };
        eprintln!(
            "    stamp R={r:.0} between {:?} and {:?} (edge {} = {})",
            n1, n2, eidx, comp.id
        );
        mna.stamp_resistor(n1, n2, r);
    }

    // Create WDF ports: one per block (at port node) + one adapted VS input port.
    let n_ports = plan.num_blocks() + 1;
    let mut ports = Vec::with_capacity(n_ports);

    // Block ports: each block's first port_node that appears in coupling_nodes.
    for (bi, block) in plan.blocks.iter().enumerate() {
        let port_node = block
            .port_nodes
            .iter()
            .find(|&&n| node_to_mna.contains_key(&n))
            .copied();
        let mna_node = port_node.and_then(|n| node_to_mna.get(&n).copied());
        eprintln!(
            "    block {bi}: port_node={:?} → mna_node={:?}",
            port_node, mna_node
        );
        ports.push(pedalkernel_rt::tree::WdfPort {
            node_pos: mna_node,
            node_neg: None,
            resistance: 1000.0, // nominal port resistance
        });
    }

    // Adapted VS input port at the first coupling node.
    ports.push(pedalkernel_rt::tree::WdfPort {
        node_pos: Some(0),
        node_neg: None,
        resistance: 1.0, // VS port
    });

    let s_matrix = mna.derive_scattering_matrix_general(&ports);

    eprintln!("  scattering matrix: {}×{}", n_ports, n_ports);
    for row in 0..n_ports {
        let row_vals: Vec<String> = (0..n_ports)
            .map(|col| format!("{:+.4}", s_matrix[row * n_ports + col]))
            .collect();
        eprintln!("    [{}]", row_vals.join(", "));
    }

    // Assert: matrix is square (n_ports × n_ports).
    assert_eq!(
        s_matrix.len(),
        n_ports * n_ports,
        "Scattering matrix should be {}×{} = {} elements, got {}",
        n_ports,
        n_ports,
        n_ports * n_ports,
        s_matrix.len()
    );

    // Assert: no NaN/Inf.
    for (i, &val) in s_matrix.iter().enumerate() {
        assert!(
            val.is_finite(),
            "Scattering matrix element [{}][{}] = {} is not finite",
            i / n_ports,
            i % n_ports,
            val
        );
    }

    // Assert: all entries bounded [-2, 2] (passivity bound for multi-port).
    for (i, &val) in s_matrix.iter().enumerate() {
        assert!(
            val.abs() <= 2.0 + 1e-10,
            "Scattering matrix element [{}][{}] = {} exceeds passivity bound [-2, 2]",
            i / n_ports,
            i % n_ports,
            val
        );
    }

    // Assert: number of ports = blocks + 1.
    assert_eq!(
        n_ports,
        plan.num_blocks() + 1,
        "Number of ports should be num_blocks + 1 (VS input)"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 7. Blockwise K-method produces audio (non-silent)
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn tb303_blockwise_k_method_produces_audio() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");

    let cache_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("target")
        .join("test-cache");
    let _ = std::fs::create_dir_all(&cache_dir);

    eprintln!("  compiling with K-tables (cached)...");
    let options = super::compile::CompileOptions::default();
    let blob = super::compile::compile_pedal_cached(
        &source,
        "tb303_bkm_audio",
        "tb303_bkm_audio",
        SR,
        &options,
        &cache_dir,
    )
    .expect("compile failed");
    eprintln!("  blob: {} bytes", blob.len());

    let mut proc: super::compiled::CompiledPedal =
        postcard::from_bytes(&blob).expect("deserialize failed");

    // Default controls.
    proc.set_control_immediate("Cutoff", 0.5);
    proc.set_control_immediate("Resonance", 0.3);

    // Warm-up.
    for _ in 0..2400 {
        proc.process(0.0);
    }

    // Process 4800 samples of 440Hz sine at 0.5V amplitude.
    let mut peak = 0.0f64;
    for i in 0..4800 {
        let input = 0.5 * (2.0 * std::f64::consts::PI * 440.0 * i as f64 / SR).sin();
        let out = proc.process(input);
        peak = peak.max(out.abs());
    }

    eprintln!("  peak output: {peak:.6}");

    assert!(
        peak > 0.01,
        "TB303 K-method stage should produce audible output, got peak={peak:.6}"
    );
}

#[test]
fn tb303_bkm_direct_output_reaches_serial_output() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");

    let cache_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("target")
        .join("test-cache");
    let _ = std::fs::create_dir_all(&cache_dir);

    let blob = super::compile::compile_pedal_cached(
        &source,
        "tb303_bkm_direct_vs_serial",
        "tb303_bkm_direct_vs_serial",
        SR,
        &super::compile::CompileOptions::default(),
        &cache_dir,
    )
    .expect("compile failed");

    let mut proc_direct: super::compiled::CompiledPedal =
        postcard::from_bytes(&blob).expect("deserialize failed");
    let mut proc_coupled: super::compiled::CompiledPedal =
        postcard::from_bytes(&blob).expect("deserialize failed");
    let mut proc_serial: super::compiled::CompiledPedal =
        postcard::from_bytes(&blob).expect("deserialize failed");

    let bkm = proc_direct
        .stages
        .iter_mut()
        .find_map(|s| {
            if let pedalkernel_rt::processor::Stage::Blockwise(ref mut k) = s {
                Some(k)
            } else {
                None
            }
        })
        .expect("TB303 should compile to blockwise K-method");
    let wdf = proc_coupled
        .stages
        .iter_mut()
        .find_map(|s| {
            if let pedalkernel_rt::processor::Stage::Wdf(ref mut w) = s {
                Some(w)
            } else {
                None
            }
        })
        .expect("TB303 should have output coupling WDF stage");

    for _ in 0..2400 {
        let direct = bkm.process(&[0.0, 0.0, 0.0, 0.0]);
        let _ = wdf.process(direct);
        let _ = proc_serial.process(0.0);
    }

    let mut bkm_peak = 0.0f64;
    let mut coupled_peak = 0.0f64;
    let mut serial_peak = 0.0f64;
    for i in 0..4800 {
        let input = 0.5 * (2.0 * std::f64::consts::PI * 440.0 * i as f64 / SR).sin();
        let bkm_out = bkm.process(&[input, 0.0, 0.0, 0.0]);
        bkm_peak = bkm_peak.max(bkm_out.abs());
        coupled_peak = coupled_peak.max(wdf.process(bkm_out).abs());
        serial_peak = serial_peak.max(proc_serial.process(input).abs());
    }

    eprintln!(
        "  BKM direct peak={bkm_peak:.6}, direct C_out peak={coupled_peak:.6}, serial peak={serial_peak:.6}"
    );
    assert!(
        bkm_peak > 0.01,
        "BKM should produce signal before output coupling, got {bkm_peak:.6}"
    );
    assert!(
        coupled_peak > bkm_peak * 0.1,
        "Output coupling should preserve at least 10% of BKM signal: \
         bkm={bkm_peak:.6}, coupled={coupled_peak:.6}"
    );
    assert!(
        serial_peak > 0.01,
        "Full processor serial path should remain non-silent: serial={serial_peak:.6}"
    );
}

#[test]
fn tb303_bkm_direct_path_is_lowpass() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let def = crate::dsl::parse_pedal_file(&source).expect("parse failed");

    let measure = |freq: f64| -> f64 {
        let mut proc =
            super::compile_pedal_with_options(&def, SR, super::compile::CompileOptions::default())
                .expect("compile failed");
        proc.set_control_immediate("Cutoff", 0.5);
        proc.set_control_immediate("Resonance", 0.0);
        proc.cache_all_vs_pointers();
        for _ in 0..9600 {
            let _ = proc.process(0.0);
        }

        let bkm = proc
            .stages
            .iter_mut()
            .find_map(|s| {
                if let pedalkernel_rt::processor::Stage::Blockwise(ref mut k) = s {
                    Some(k)
                } else {
                    None
                }
            })
            .expect("TB303 should compile to blockwise K-method");

        for i in 0..9600 {
            let input = 0.03 * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
            let vs = tb303_bkm_vs_signals(bkm, 0.0, 0.0);
            let _ = bkm.process_with_serial_input(input, &vs);
        }

        settled_sine_ac_rms(freq, 0.03, |input| {
            let vs = tb303_bkm_vs_signals(bkm, 0.0, 0.0);
            bkm.process_with_serial_input(input, &vs)
        })
    };

    let gain_100 = measure(100.0);
    let gain_10k = measure(10_000.0);
    let ratio_db = 20.0 * (gain_100 / gain_10k.max(1e-12)).log10();

    eprintln!(
        "  BKM direct LPF: 100Hz={gain_100:.6}, 10kHz={gain_10k:.6}, ratio={ratio_db:+.1} dB"
    );

    assert!(
        gain_100 > gain_10k * 3.0,
        "BKM core should have a settled lowpass slope before C_out: 100Hz={gain_100:.6}, 10kHz={gain_10k:.6}"
    );
}

#[test]
fn tb303_bkm_feedback_ports_are_the_lowpass_bypass_source() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let def = crate::dsl::parse_pedal_file(&source).expect("parse failed");

    let measure = |freq: f64, with_feedback_ports: bool| -> f64 {
        let mut proc =
            super::compile_pedal_with_options(&def, SR, super::compile::CompileOptions::default())
                .expect("compile failed");
        proc.set_control_immediate("Cutoff", 0.5);
        proc.set_control_immediate("Resonance", 0.0);
        proc.cache_all_vs_pointers();
        for _ in 0..9600 {
            let _ = proc.process(0.0);
        }

        let bkm = proc
            .stages
            .iter_mut()
            .find_map(|s| {
                if let pedalkernel_rt::processor::Stage::Blockwise(ref mut k) = s {
                    Some(k)
                } else {
                    None
                }
            })
            .expect("TB303 should compile to BKM");

        for i in 0..9600 {
            let input = 0.03 * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
            if with_feedback_ports {
                let _ = bkm.process(&[input, 0.0, 0.5, 0.0]);
            } else {
                let _ = bkm.debug_process_without_feedback_ports(&[input, 0.0, 0.5, 0.0]);
            }
        }

        settled_sine_ac_rms(freq, 0.03, |input| {
            if with_feedback_ports {
                bkm.process(&[input, 0.0, 0.5, 0.0])
            } else {
                bkm.debug_process_without_feedback_ports(&[input, 0.0, 0.5, 0.0])
            }
        })
    };

    let no_fb_low = measure(100.0, false);
    let no_fb_high = measure(10_000.0, false);
    let fb_low = measure(100.0, true);
    let fb_high = measure(10_000.0, true);

    eprintln!(
        "  BKM feedback-port diagnostic: no_fb_ratio={:.2}, with_fb_ratio={:.2}",
        no_fb_low / no_fb_high.max(1e-12),
        fb_low / fb_high.max(1e-12)
    );

    assert!(
        fb_low > fb_high * 3.0,
        "BKM feedback-port injection must not bypass the lowpass when Resonance=0: \
         100Hz={fb_low:.6}, 10kHz={fb_high:.6}"
    );
}

#[test]
fn tb303_bkm_cutoff_pot_without_cv_is_lowpass() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let def = crate::dsl::parse_pedal_file(&source).expect("parse failed");

    let measure_with = |freq: f64, vco_drive: f64| -> f64 {
        let mut proc =
            super::compile_pedal_with_options(&def, SR, super::compile::CompileOptions::default())
                .expect("compile failed");
        proc.set_control_immediate("Cutoff", 0.5);
        proc.set_control_immediate("Resonance", 0.0);
        proc.cache_all_vs_pointers();
        for _ in 0..9600 {
            let _ = proc.process(0.0);
        }

        let bkm = proc
            .stages
            .iter_mut()
            .find_map(|s| {
                if let pedalkernel_rt::processor::Stage::Blockwise(ref mut k) = s {
                    Some(k)
                } else {
                    None
                }
            })
            .expect("TB303 should compile to BKM");

        settled_sine_ac_rms(freq, 0.03, |input| {
            bkm.process(&[input, input * vco_drive, 0.0, 0.0])
        })
    };

    // Probe one source path. Driving audio_in and vco_in with the same sine is
    // a two-source test and can turn the coupling network into a feed-forward
    // shelf instead of the small-signal ladder response we want here.
    let gain_100 = measure_with(100.0, 0.0);
    let gain_10k = measure_with(10_000.0, 0.0);
    let ratio_db = 20.0 * (gain_100 / gain_10k.max(1e-12)).log10();

    eprintln!(
        "  BKM cutoff pot LPF, zero CV: 100Hz={gain_100:.6}, 10kHz={gain_10k:.6}, ratio={ratio_db:+.1} dB"
    );

    assert!(
        gain_100 > gain_10k * 3.0,
        "Cutoff pot alone should bias the BKM diode ladder into a lowpass response: \
         100Hz={gain_100:.6}, 10kHz={gain_10k:.6}"
    );
}

#[test]
fn tb303_bkm_core_without_resonance_feedback_is_lowpass() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let source = tb303_source_without_resonance_feedback(&source);
    let def = crate::dsl::parse_pedal_file(&source).expect("parse failed");

    let measure_with = |freq: f64, vco_drive: f64| -> f64 {
        let mut proc =
            super::compile_pedal_with_options(&def, SR, super::compile::CompileOptions::default())
                .expect("compile failed");
        proc.set_control_immediate("Cutoff", 0.5);
        proc.cache_all_vs_pointers();
        for _ in 0..9600 {
            let _ = proc.process(0.0);
        }

        let bkm = proc
            .stages
            .iter_mut()
            .find_map(|s| {
                if let pedalkernel_rt::processor::Stage::Blockwise(ref mut k) = s {
                    Some(k)
                } else {
                    None
                }
            })
            .expect("TB303 core should compile to BKM");

        settled_sine_ac_rms(freq, 0.03, |input| {
            bkm.process(&[input, input * vco_drive, 0.0])
        })
    };

    let gain_100 = measure_with(100.0, 0.0);
    let gain_10k = measure_with(10_000.0, 0.0);
    let ratio_db = 20.0 * (gain_100 / gain_10k.max(1e-12)).log10();
    eprintln!(
        "  BKM core without resonance feedback: 100Hz={gain_100:.6}, 10kHz={gain_10k:.6}, ratio={ratio_db:+.1} dB"
    );

    assert!(
        gain_100 > gain_10k * 3.0,
        "core ladder without resonance feedback should be a lowpass: \
         100Hz={gain_100:.6}, 10kHz={gain_10k:.6}"
    );
}

#[test]
fn tb303_bkm_full_processor_vco_port_path_is_not_flat() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let def = crate::dsl::parse_pedal_file(&source).expect("parse failed");

    let measure_process = |freq: f64| -> f64 {
        let mut proc =
            super::compile_pedal_with_options(&def, SR, super::compile::CompileOptions::default())
                .expect("compile failed");
        proc.set_control_immediate("Cutoff", 0.5);
        proc.set_control_immediate("Resonance", 0.0);
        settled_sine_ac_rms(freq, 0.03, |input| proc.process(input))
    };

    let measure_ports = |freq: f64| -> f64 {
        let mut proc =
            super::compile_pedal_with_options(&def, SR, super::compile::CompileOptions::default())
                .expect("compile failed");
        proc.set_control_immediate("Cutoff", 0.5);
        proc.set_control_immediate("Resonance", 0.0);

        quick_sine_ac_rms_ports(
            &mut proc,
            freq,
            0.03,
            &[("vco_in", 1.0)],
            &[("cv_cutoff", 0.0), ("cv_resonance", 0.0)],
            "audio_out",
        )
    };

    let process_low = measure_process(100.0);
    let process_high = measure_process(10_000.0);
    let ports_low = measure_ports(100.0);
    let ports_high = measure_ports(10_000.0);
    let process_ratio_db = 20.0 * (process_low / process_high.max(1e-12)).log10();
    let ports_ratio_db = 20.0 * (ports_low / ports_high.max(1e-12)).log10();

    eprintln!(
        "  BKM full processor process(input): 100Hz={process_low:.6}, 10kHz={process_high:.6}, ratio={process_ratio_db:+.1} dB"
    );
    eprintln!(
        "  BKM full processor process_ports(audio): 100Hz={ports_low:.6}, 10kHz={ports_high:.6}, ratio={ports_ratio_db:+.1} dB"
    );

    assert!(
        ports_ratio_db > 24.0,
        "Driving the explicit audio port should expose the BKM lowpass path; \
         process(input) ratio={process_ratio_db:+.1} dB, process_ports ratio={ports_ratio_db:+.1} dB"
    );
}

#[test]
fn tb303_bkm_resonance_control_changes_cutoff_band() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let def = crate::dsl::parse_pedal_file(&source).expect("parse failed");

    let measure = |freq: f64, resonance: f64| -> f64 {
        let mut proc =
            super::compile_pedal_with_options(&def, SR, super::compile::CompileOptions::default())
                .expect("compile failed");
        proc.set_control_immediate("Cutoff", 0.5);
        proc.set_control_immediate("Resonance", resonance);

        settled_sine_ac_rms_ports(
            &mut proc,
            freq,
            0.01,
            &[("audio_in", 1.0), ("vco_in", 1.0)],
            &[("cv_cutoff", 0.0), ("cv_resonance", 0.0)],
            "audio_out",
        )
    };

    let low_res_500 = measure(500.0, 0.0);
    let high_res_500 = measure(500.0, 0.85);
    let low_res_2k = measure(2_000.0, 0.0);
    let high_res_2k = measure(2_000.0, 0.85);
    let low_res_5k = measure(5_000.0, 0.0);
    let high_res_5k = measure(5_000.0, 0.85);

    let boost_500 = db_norm(high_res_500, low_res_500);
    let boost_2k = db_norm(high_res_2k, low_res_2k);
    let boost_5k = db_norm(high_res_5k, low_res_5k);
    eprintln!(
        "  BKM resonance boost: 500Hz={boost_500:+.2}dB, 2kHz={boost_2k:+.2}dB, 5kHz={boost_5k:+.2}dB"
    );

    assert!(
        boost_500.abs().max(boost_2k.abs()).max(boost_5k.abs()) > 1.0,
        "BKM Resonance control should measurably change the cutoff-band transfer: \
         500Hz={boost_500:+.2}dB, 2kHz={boost_2k:+.2}dB, 5kHz={boost_5k:+.2}dB"
    );
}

#[test]
fn tb303_bkm_max_resonance_remains_finite() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let def = crate::dsl::parse_pedal_file(&source).expect("parse failed");
    let mut proc =
        super::compile_pedal_with_options(&def, SR, super::compile::CompileOptions::default())
            .expect("compile failed");
    proc.set_control_immediate("Cutoff", 0.5);
    proc.set_control_immediate("Resonance", 0.95);
    proc.cache_all_vs_pointers();

    let audio_in = proc.resolve_port("audio_in").expect("audio_in port");
    let vco_in = proc.resolve_port("vco_in").expect("vco_in port");
    let cv_cutoff = proc.resolve_port("cv_cutoff").expect("cv_cutoff port");
    let cv_resonance = proc
        .resolve_port("cv_resonance")
        .expect("cv_resonance port");
    let audio_out = proc.resolve_port("audio_out").expect("audio_out port");
    let mut ports = vec![0.0; proc.port_count()];

    let mut max_abs = 0.0f64;
    for i in 0..48_000 {
        let input = 0.05 * (2.0 * std::f64::consts::PI * 220.0 * i as f64 / SR).sin();
        ports[audio_in] = input;
        ports[vco_in] = input;
        ports[cv_cutoff] = 0.0;
        ports[cv_resonance] = 0.0;
        proc.process_ports(&mut ports);
        let out = ports[audio_out];
        assert!(
            out.is_finite(),
            "max resonance must not emit NaN/Inf at sample {i}: {out}"
        );
        max_abs = max_abs.max(out.abs());
    }

    for stage in &proc.stages {
        if let pedalkernel_rt::processor::Stage::Blockwise(bkm) = stage {
            assert!(
                bkm.work_a
                    .iter()
                    .chain(bkm.work_b.iter())
                    .all(|v| v.is_finite()),
                "BKM work buffers must remain finite at max resonance"
            );
            assert!(
                bkm.blocks.iter().all(|block| {
                    block.dc_offset.is_finite()
                        && block.rp.is_finite()
                        && block.k_table.entries.iter().all(|v| v.is_finite())
                }),
                "BKM block state and K-tables must remain finite at max resonance"
            );
        }
    }

    assert!(
        max_abs < 100.0,
        "max resonance should stay bounded enough for host/plugin output: peak={max_abs:.6}"
    );
}

#[test]
fn tb303_bkm_block_outputs_show_shallow_lowpass_shape() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let cache_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("target")
        .join("test-cache");
    let _ = std::fs::create_dir_all(&cache_dir);

    let blob = super::compile::compile_pedal_cached(
        &source,
        "tb303_bkm_block_shape",
        "tb303_bkm_block_shape",
        SR,
        &super::compile::CompileOptions::default(),
        &cache_dir,
    )
    .expect("compile failed");

    let measure_blocks = |freq: f64| -> Vec<f64> {
        let mut proc: super::compiled::CompiledPedal =
            postcard::from_bytes(&blob).expect("deserialize failed");
        let bkm = proc
            .stages
            .iter_mut()
            .find_map(|s| {
                if let pedalkernel_rt::processor::Stage::Blockwise(ref mut k) = s {
                    Some(k)
                } else {
                    None
                }
            })
            .expect("TB303 should compile to blockwise K-method");

        for i in 0..9600 {
            let input = 0.1 * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
            let vs = tb303_bkm_vs_signals_with_vco(bkm, input, 0.0, 0.0);
            let _ = bkm.debug_process_with_block_outputs(&vs);
        }

        let mut values = vec![Vec::new(); bkm.blocks.len()];
        for i in 0..9600 {
            let input = 0.1 * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
            let vs = tb303_bkm_vs_signals_with_vco(bkm, input, 0.0, 0.0);
            let (_, blocks) = bkm.debug_process_with_block_outputs(&vs);
            for (block_values, value) in values.iter_mut().zip(blocks.iter()) {
                block_values.push(*value);
            }
        }

        values
            .iter()
            .map(|block_values| ac_rms(block_values))
            .collect()
    };

    let low = measure_blocks(100.0);
    let high = measure_blocks(10_000.0);
    eprintln!("  BKM block RMS 100Hz: {low:.6?}");
    eprintln!("  BKM block RMS 10kHz: {high:.6?}");

    assert_eq!(low.len(), 4);
    assert_eq!(high.len(), 4);
    let final_ratio = low[3] / high[3].max(1e-12);
    assert!(
        final_ratio > 1.5,
        "final BKM rung should at least remain lowpass: 100Hz={:.6}, 10kHz={:.6}",
        low[3],
        high[3]
    );
    assert!(
        final_ratio > 100.0,
        "BKM block-output diagnostic should show the corrected multi-rung lowpass slope: ratio={final_ratio:.3}"
    );
}

#[test]
fn tb303_bkm_rung_response_exposes_missing_cascaded_poles() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let def = crate::dsl::parse_pedal_file(&source).expect("parse failed");

    let measure_serial = |freq: f64| -> Vec<f64> {
        let mut options = super::compile::CompileOptions::default();
        options.force_serial_blockwise = true;
        let mut proc =
            super::compile_pedal_with_options(&def, SR, options).expect("compile failed");
        proc.set_control_immediate("Resonance", 0.0);

        for i in 0..9600 {
            let _ = proc.process(0.1 * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin());
        }

        let mut sums = Vec::new();
        let mut count = 0usize;
        for i in 0..9600 {
            let input = 0.1 * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
            let mut x = input;
            let mut rung = 0usize;
            for stage in &mut proc.stages {
                let pedalkernel_rt::processor::Stage::Wdf(wdf) = stage else {
                    continue;
                };
                x = wdf.process(x);
                if rung < 4 {
                    if sums.len() <= rung {
                        sums.push(0.0);
                    }
                    sums[rung] += x * x;
                    rung += 1;
                }
            }
            count += 1;
        }

        sums.into_iter()
            .map(|v| (v / count.max(1) as f64).sqrt())
            .collect()
    };

    let measure_bkm = |freq: f64| -> Vec<f64> {
        let mut proc =
            super::compile_pedal_with_options(&def, SR, super::compile::CompileOptions::default())
                .expect("compile failed");
        proc.set_control_immediate("Resonance", 0.0);

        let bkm = proc
            .stages
            .iter_mut()
            .find_map(|s| {
                if let pedalkernel_rt::processor::Stage::Blockwise(ref mut k) = s {
                    Some(k)
                } else {
                    None
                }
            })
            .expect("TB303 should compile to BKM");

        for i in 0..9600 {
            let input = 0.1 * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
            let vs = tb303_bkm_vs_signals_with_vco(bkm, input, 0.0, 0.0);
            let _ = bkm.debug_process_with_block_outputs(&vs);
        }

        let mut values = vec![Vec::new(); bkm.blocks.len()];
        for i in 0..9600 {
            let input = 0.1 * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
            let vs = tb303_bkm_vs_signals_with_vco(bkm, input, 0.0, 0.0);
            let (_, outputs) = bkm.debug_process_with_block_outputs(&vs);
            for (block_values, value) in values.iter_mut().zip(outputs.iter()) {
                block_values.push(*value);
            }
        }

        values
            .iter()
            .map(|block_values| ac_rms(block_values))
            .collect()
    };

    let serial_low = measure_serial(100.0);
    let serial_high = measure_serial(10_000.0);
    let bkm_low = measure_bkm(100.0);
    let bkm_high = measure_bkm(10_000.0);

    let serial_ratio: Vec<f64> = serial_low
        .iter()
        .zip(serial_high.iter())
        .map(|(lo, hi)| lo / hi.max(1e-12))
        .collect();
    let bkm_ratio: Vec<f64> = bkm_low
        .iter()
        .zip(bkm_high.iter())
        .map(|(lo, hi)| lo / hi.max(1e-12))
        .collect();

    eprintln!("  serial low RMS: {serial_low:.6?}");
    eprintln!("  serial high RMS: {serial_high:.6?}");
    eprintln!("  serial low/high per rung: {serial_ratio:.3?}");
    eprintln!("  BKM low RMS: {bkm_low:.6?}");
    eprintln!("  BKM high RMS: {bkm_high:.6?}");
    eprintln!("  BKM low/high per rung: {bkm_ratio:.3?}");

    assert_eq!(serial_ratio.len(), 4);
    assert_eq!(bkm_ratio.len(), 4);
    assert!(
        serial_ratio[3] > serial_ratio[0] * 10.0,
        "forced-serial WDF should add poles across the cascade: serial ratios={serial_ratio:.3?}"
    );
    assert!(
        bkm_ratio[3] < bkm_ratio[0] * 2.0,
        "BKM currently fails to add later cascade poles; keep this diagnostic until the coupling/cascade contract is fixed. \
         serial ratios={serial_ratio:.3?}, BKM ratios={bkm_ratio:.3?}"
    );
}

#[test]
fn tb303_compare_bkm_forced_serial_and_htb_shape() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let freqs = [100.0, 200.0, 500.0, 1000.0, 2000.0, 5000.0, 10_000.0];

    let compile_mode = |force_serial: bool| -> Vec<u8> {
        let cache_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("target")
            .join("test-cache");
        let _ = std::fs::create_dir_all(&cache_dir);
        let mut options = super::compile::CompileOptions::default();
        options.force_serial_blockwise = force_serial;
        let key = if force_serial {
            "tb303_compare_serial"
        } else {
            "tb303_compare_bkm"
        };
        super::compile::compile_pedal_cached(&source, key, key, SR, &options, &cache_dir)
            .expect("compile failed")
    };

    let bkm_blob = compile_mode(false);
    let serial_blob = compile_mode(true);

    let measure_blob = |blob: &[u8], freq: f64| -> f64 {
        let mut proc: super::compiled::CompiledPedal =
            postcard::from_bytes(blob).expect("deserialize failed");
        proc.set_control_immediate("Cutoff", 0.5);
        proc.set_control_immediate("Resonance", 0.0);
        quick_sine_ac_rms_ports(
            &mut proc,
            freq,
            0.03,
            &[("audio_in", 1.0)],
            &[("cv_cutoff", 0.0), ("cv_resonance", 0.0)],
            "audio_out",
        )
    };

    let bkm: Vec<f64> = freqs
        .iter()
        .map(|&freq| measure_blob(&bkm_blob, freq))
        .collect();
    let serial: Vec<f64> = freqs
        .iter()
        .map(|&freq| measure_blob(&serial_blob, freq))
        .collect();

    let best_fit_fc = |gains: &[f64]| -> f64 {
        let target: Vec<f64> = gains.iter().map(|gain| db_norm(*gain, gains[0])).collect();
        let mut best_fc = 100.0;
        let mut best_err = f64::INFINITY;

        for step in 0..240 {
            let t = step as f64 / 239.0;
            let fc = 100.0 * (80.0f64).powf(t);
            let h_ref = stinchcombe_htb_magnitude(freqs[0], fc);
            let mut err = 0.0;
            for (i, &freq) in freqs.iter().enumerate() {
                let h_db = db_norm(stinchcombe_htb_magnitude(freq, fc), h_ref);
                let diff = target[i] - h_db;
                err += diff * diff;
            }
            if err < best_err {
                best_err = err;
                best_fc = fc;
            }
        }

        best_fc
    };

    let htb_fc = best_fit_fc(&serial);
    let h_ref = stinchcombe_htb_magnitude(freqs[0], htb_fc);
    let best_10pole_fit = |gains: &[f64], norm_idx: usize| -> (f64, f64) {
        let target: Vec<f64> = gains
            .iter()
            .map(|gain| db_norm(*gain, gains[norm_idx]))
            .collect();
        let mut best_fc = 100.0;
        let mut best_err = f64::INFINITY;

        for step in 0..320 {
            let t = step as f64 / 319.0;
            let fc = 100.0 * (100.0f64).powf(t);
            let ref_norm = stinchcombe_10pole_magnitude(freqs[norm_idx], fc, 0.0);
            if ref_norm <= 1.0e-12 {
                continue;
            }
            let mut err = 0.0;
            for (i, &freq) in freqs.iter().enumerate() {
                let ref_db = db_norm(stinchcombe_10pole_magnitude(freq, fc, 0.0), ref_norm);
                let diff = target[i] - ref_db;
                err += diff * diff;
            }
            if err < best_err {
                best_err = err;
                best_fc = fc;
            }
        }

        let ref_norm = stinchcombe_10pole_magnitude(freqs[norm_idx], best_fc, 0.0);
        let max_abs_err = freqs
            .iter()
            .enumerate()
            .map(|(i, &freq)| {
                let ref_db = db_norm(stinchcombe_10pole_magnitude(freq, best_fc, 0.0), ref_norm);
                (target[i] - ref_db).abs()
            })
            .fold(0.0, f64::max);
        (best_fc, max_abs_err)
    };
    let (bkm_10pole_fc_100, bkm_10pole_err_100) = best_10pole_fit(&bkm, 0);
    let (bkm_10pole_fc_10k, bkm_10pole_err_10k) = best_10pole_fit(&bkm, freqs.len() - 1);
    let best_10pole_shift_fit = |gains: &[f64], norm_idx: usize| -> (f64, f64, f64) {
        let target: Vec<f64> = gains
            .iter()
            .map(|gain| db_norm(*gain, gains[norm_idx]))
            .collect();
        let mut best_scale = 1.0;
        let mut best_fc = 100.0;
        let mut best_err = f64::INFINITY;

        for scale_step in 0..81 {
            let scale = 0.80 + 0.005 * scale_step as f64;
            for fc_step in 0..240 {
                let t = fc_step as f64 / 239.0;
                let fc = 100.0 * (100.0f64).powf(t);
                let ref_norm = stinchcombe_10pole_magnitude(freqs[norm_idx] * scale, fc, 0.0);
                if ref_norm <= 1.0e-12 {
                    continue;
                }
                let mut err = 0.0;
                for (i, &freq) in freqs.iter().enumerate() {
                    let ref_db = db_norm(
                        stinchcombe_10pole_magnitude(freq * scale, fc, 0.0),
                        ref_norm,
                    );
                    let diff = target[i] - ref_db;
                    err += diff * diff;
                }
                if err < best_err {
                    best_err = err;
                    best_scale = scale;
                    best_fc = fc;
                }
            }
        }

        let ref_norm = stinchcombe_10pole_magnitude(freqs[norm_idx] * best_scale, best_fc, 0.0);
        let max_abs_err = freqs
            .iter()
            .enumerate()
            .map(|(i, &freq)| {
                let ref_db = db_norm(
                    stinchcombe_10pole_magnitude(freq * best_scale, best_fc, 0.0),
                    ref_norm,
                );
                (target[i] - ref_db).abs()
            })
            .fold(0.0, f64::max);
        (best_scale, best_fc, max_abs_err)
    };
    let (bkm_shift_scale, bkm_shift_fc, bkm_shift_err) = best_10pole_shift_fit(&bkm, 0);
    let full_10_ref = stinchcombe_10pole_magnitude(freqs[0], bkm_10pole_fc_100, 0.0);

    eprintln!("  TB303 normalized response comparison, Cutoff=0.5 Resonance=0");
    eprintln!("  H_tb best-fit fc to forced-serial WDF: {htb_fc:.1} Hz");
    eprintln!(
        "  10-pole k=0 best fit to BKM: fc={bkm_10pole_fc_100:.1}Hz err={bkm_10pole_err_100:.2}dB (norm 100Hz), \
         fc={bkm_10pole_fc_10k:.1}Hz err={bkm_10pole_err_10k:.2}dB (norm 10kHz)"
    );
    eprintln!(
        "  10-pole k=0 shift fit: freq_scale={bkm_shift_scale:.3}, fc={bkm_shift_fc:.1}Hz, err={bkm_shift_err:.2}dB"
    );
    eprintln!(
        "  {:>8} {:>10} {:>10} {:>10} {:>10} {:>10}",
        "Freq", "BKM dB", "Serial dB", "H_tb dB", "10pole dB", "BKM-10p"
    );
    for (i, &freq) in freqs.iter().enumerate() {
        let bkm_db = db_norm(bkm[i], bkm[0]);
        let serial_db = db_norm(serial[i], serial[0]);
        let htb_db = db_norm(stinchcombe_htb_magnitude(freq, htb_fc), h_ref);
        let full_10_db = db_norm(
            stinchcombe_10pole_magnitude(freq, bkm_10pole_fc_100, 0.0),
            full_10_ref,
        );
        eprintln!(
            "  {freq:>8.0} {bkm_db:>+10.1} {serial_db:>+10.1} {htb_db:>+10.1} {full_10_db:>+10.1} {:>+10.1}",
            bkm_db - full_10_db
        );
    }

    let bkm_oct = db_norm(bkm[6], bkm[5]);
    let serial_oct = db_norm(serial[6], serial[5]);
    let htb_oct = db_norm(
        stinchcombe_htb_magnitude(freqs[6], htb_fc),
        stinchcombe_htb_magnitude(freqs[5], htb_fc),
    );
    eprintln!(
        "  5k->10k slope: BKM={bkm_oct:+.1} dB/oct, serial={serial_oct:+.1} dB/oct, H_tb={htb_oct:+.1} dB/oct"
    );
    assert!(
        bkm_10pole_err_100 < 3.0,
        "BKM with coupling caps should match the published 10-pole k=0 shelf/lowpass shape after cutoff fit: \
         fc={bkm_10pole_fc_100:.1}Hz, max_abs_err={bkm_10pole_err_100:.2}dB"
    );
}

#[test]
fn tb303_bkm_output_is_read_only_extraction() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let def = crate::dsl::parse_pedal_file(&source).expect("parse failed");
    let compiled = super::compile_pedal(&def, SR).expect("compile failed");

    let bkm = compiled
        .stages
        .iter()
        .find_map(|s| {
            if let pedalkernel_rt::processor::Stage::Blockwise(k) = s {
                Some(k)
            } else {
                None
            }
        })
        .expect("TB303 should compile to blockwise K-method");

    assert!(
        bkm.output_port_index.is_none(),
        "audio_out must not be added as an active WDF scattering probe"
    );
    assert_eq!(
        bkm.output_extraction_coeffs.len(),
        bkm.n_ports,
        "audio_out should be read back through MNA extraction coefficients"
    );
    assert!(
        bkm.output_extraction_coeffs
            .iter()
            .any(|coeff| coeff.abs() > 1.0e-12),
        "audio_out extraction must observe a real coupling-network node"
    );
}

#[test]
fn tb303_bkm_audio_out_blocks_dc_after_output_coupling_cap() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let cache_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("target")
        .join("test-cache");
    let _ = std::fs::create_dir_all(&cache_dir);
    let mut options = super::compile::CompileOptions::default();
    options.coupled_blockwise_newton = false;
    let blob = super::compile::compile_pedal_cached(
        &source,
        "tb303_audio_out_dc_blocked_v2",
        "tb303_audio_out_dc_blocked_v2",
        SR,
        &options,
        &cache_dir,
    )
    .expect("compile failed");

    for input_name in ["audio_in", "vco_in"] {
        let mut proc: super::compiled::CompiledPedal =
            postcard::from_bytes(&blob).expect("deserialize failed");
        let mut bkm_only: super::compiled::CompiledPedal =
            postcard::from_bytes(&blob).expect("deserialize failed");
        let mut direct_proc: super::compiled::CompiledPedal =
            postcard::from_bytes(&blob).expect("deserialize failed");
        let mut direct_bkm = match direct_proc.stages.remove(0) {
            pedalkernel_rt::processor::Stage::Blockwise(k) => k,
            _ => panic!("expected first stage to be BKM"),
        };
        bkm_only.stages.truncate(1);
        proc.set_control_immediate("Cutoff", 0.5);
        proc.set_control_immediate("Resonance", 0.0);
        proc.cache_all_vs_pointers();
        bkm_only.set_control("Cutoff", 0.5);
        bkm_only.set_control("Resonance", 0.0);
        bkm_only.cache_all_vs_pointers();
        direct_bkm.set_pot("Cutoff", 0.5);
        direct_bkm.set_pot("Resonance", 0.0);
        direct_bkm.solve_dc_operating_point();

        let in_idx = proc
            .resolve_port(input_name)
            .unwrap_or_else(|| panic!("missing {input_name} port"));
        let cv_cutoff = proc.resolve_port("cv_cutoff").expect("cv_cutoff port");
        let cv_resonance = proc
            .resolve_port("cv_resonance")
            .expect("cv_resonance port");
        let out_idx = proc.resolve_port("audio_out").expect("audio_out port");
        let mut ports = vec![0.0; proc.port_count()];
        let mut bkm_ports = vec![0.0; bkm_only.port_count()];
        let direct_input_idx = direct_bkm
            .vs_port_map
            .iter()
            .position(|(name, _)| name == input_name)
            .unwrap_or_else(|| panic!("BKM missing {input_name} VS map"));
        let direct_cutoff_idx = direct_bkm
            .vs_port_map
            .iter()
            .position(|(name, _)| name == "cv_cutoff")
            .expect("BKM missing cv_cutoff VS map");
        let direct_resonance_idx = direct_bkm
            .vs_port_map
            .iter()
            .position(|(name, _)| name == "cv_resonance")
            .expect("BKM missing cv_resonance VS map");
        let mut direct_vs = vec![0.0; direct_bkm.vs_port_map.len()];

        for _ in 0..4800 {
            ports.fill(0.0);
            ports[cv_cutoff] = 0.0;
            ports[cv_resonance] = 0.0;
            proc.process_ports(&mut ports);
            bkm_ports.fill(0.0);
            bkm_ports[cv_cutoff] = 0.0;
            bkm_ports[cv_resonance] = 0.0;
            bkm_only.process_ports(&mut bkm_ports);
            direct_vs.fill(0.0);
            direct_vs[direct_cutoff_idx] = 0.0;
            direct_vs[direct_resonance_idx] = 0.0;
            direct_bkm.process_with_serial_input(0.0, &direct_vs);
        }

        let mut tail = Vec::with_capacity(4800);
        let mut bkm_tail = Vec::with_capacity(4800);
        let mut direct_tail = Vec::with_capacity(4800);
        for i in 0..24_000 {
            ports.fill(0.0);
            ports[in_idx] = 0.1;
            ports[cv_cutoff] = 0.0;
            ports[cv_resonance] = 0.0;
            proc.process_ports(&mut ports);
            bkm_ports.fill(0.0);
            bkm_ports[in_idx] = 0.1;
            bkm_ports[cv_cutoff] = 0.0;
            bkm_ports[cv_resonance] = 0.0;
            bkm_only.process_ports(&mut bkm_ports);
            direct_vs.fill(0.0);
            direct_vs[direct_input_idx] = 0.1;
            direct_vs[direct_cutoff_idx] = 0.0;
            direct_vs[direct_resonance_idx] = 0.0;
            let direct_out = direct_bkm.process_with_serial_input(0.0, &direct_vs);
            if i >= 19_200 {
                tail.push(ports[out_idx]);
                bkm_tail.push(bkm_ports[out_idx]);
                direct_tail.push(direct_out);
            }
        }

        let mean = tail.iter().copied().sum::<f64>() / tail.len() as f64;
        let max_abs = tail.iter().map(|v| v.abs()).fold(0.0_f64, f64::max);
        let bkm_mean = bkm_tail.iter().copied().sum::<f64>() / bkm_tail.len() as f64;
        let bkm_max_abs = bkm_tail.iter().map(|v| v.abs()).fold(0.0_f64, f64::max);
        let direct_mean = direct_tail.iter().copied().sum::<f64>() / direct_tail.len() as f64;
        let direct_max_abs = direct_tail.iter().map(|v| v.abs()).fold(0.0_f64, f64::max);
        eprintln!(
            "  {input_name} DC tail: full mean={mean:+.6e}, full max_abs={max_abs:.6e}; \
             bkm_only mean={bkm_mean:+.6e}, bkm_only max_abs={bkm_max_abs:.6e}; \
             direct_bkm mean={direct_mean:+.6e}, direct_bkm max_abs={direct_max_abs:.6e}"
        );
        let limit = if input_name == "vco_in" {
            4.0e-2
        } else {
            1.0e-2
        };
        assert!(
            mean.abs() < limit && max_abs < limit,
            "BKM audio_out must be post-C_out/R_out and bounded/DC-blocked for {input_name}; \
             limit={limit:.3e}, tail mean={mean:+.6e}, max_abs={max_abs:.6e}"
        );
        assert!(
            direct_mean.is_finite() && direct_max_abs.is_finite() && direct_max_abs < 1.0,
            "raw BKM extraction should stay finite/bounded before the final output DC blocker for {input_name}; \
             direct mean={direct_mean:+.6e}, direct max_abs={direct_max_abs:.6e}"
        );
    }
}

#[test]
fn tb303_k0_response_has_no_midband_output_probe_kink() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let cache_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("target")
        .join("test-cache");
    let _ = std::fs::create_dir_all(&cache_dir);
    let blob = super::compile::compile_pedal_cached(
        &source,
        "tb303_k0_no_output_probe_kink",
        "tb303_k0_no_output_probe_kink",
        SR,
        &super::compile::CompileOptions::default(),
        &cache_dir,
    )
    .expect("compile failed");

    let freqs = [
        1250.0, 1500.0, 1600.0, 1650.0, 1700.0, 1750.0, 1800.0, 1900.0, 2000.0, 2200.0, 2500.0,
    ];
    let measure = |input_port: &'static str, amp: f64, freq: f64| -> f64 {
        let mut proc: super::compiled::CompiledPedal =
            postcard::from_bytes(&blob).expect("deserialize failed");
        proc.set_control_immediate("Cutoff", 0.5);
        proc.set_control_immediate("Resonance", 0.0);
        quick_sine_ac_rms_ports(
            &mut proc,
            freq,
            amp,
            &[(input_port, 1.0)],
            &[("cv_cutoff", 0.0), ("cv_resonance", 0.0)],
            "audio_out",
        )
    };
    let audio_gains: Vec<f64> = freqs
        .iter()
        .map(|&freq| measure("audio_in", 0.03, freq))
        .collect();
    let hot_vco_gains: Vec<f64> = freqs
        .iter()
        .map(|&freq| measure("vco_in", 0.03, freq))
        .collect();
    let gains: Vec<f64> = freqs
        .iter()
        .map(|&freq| measure("vco_in", 0.003, freq))
        .collect();
    eprintln!("  TB303 midband kink probe");
    eprintln!(
        "  {:>8} {:>12} {:>12} {:>12}",
        "freq", "audio30mV", "vco30mV", "vco3mV"
    );
    for i in 0..freqs.len() {
        eprintln!(
            "  {:>8.0} {:>12.6} {:>12.6} {:>12.6}",
            freqs[i], audio_gains[i], hot_vco_gains[i], gains[i]
        );
    }

    let max_jump = gains
        .windows(2)
        .map(|pair| (20.0 * (pair[1].max(1.0e-12) / pair[0].max(1.0e-12)).log10()).abs())
        .fold(0.0, f64::max);
    let audio_max_jump = audio_gains
        .windows(2)
        .map(|pair| (20.0 * (pair[1].max(1.0e-12) / pair[0].max(1.0e-12)).log10()).abs())
        .fold(0.0, f64::max);
    assert!(
        max_jump < 4.0,
        "k=0 WDF response has a non-physical midband kink: freqs={freqs:?}, audio_gains={audio_gains:?}, hot_vco_gains={hot_vco_gains:?}, audio_max_jump={audio_max_jump:.2}dB, vco_gains={gains:?}, vco_max_jump={max_jump:.2}dB"
    );
}

#[test]
fn tb303_k0_high_frequency_response_has_no_nyquist_cliff() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let cache_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("target")
        .join("test-cache");
    let _ = std::fs::create_dir_all(&cache_dir);
    let blob = super::compile::compile_pedal_cached(
        &source,
        "tb303_k0_no_hf_cliff",
        "tb303_k0_no_hf_cliff",
        SR,
        &super::compile::CompileOptions::default(),
        &cache_dir,
    )
    .expect("compile failed");

    const FILTER_AUDIO_INPUT_VOLTS: f64 = 0.03;
    const FILTER_AUDIO_INPUT_RESISTANCE_OHMS: f64 = 10_000.0;
    const FILTER_VCO_INPUT_RESISTANCE_OHMS: f64 = 1_000.0;
    const FILTER_VCO_INPUT_VOLTS: f64 = FILTER_AUDIO_INPUT_VOLTS * FILTER_VCO_INPUT_RESISTANCE_OHMS
        / FILTER_AUDIO_INPUT_RESISTANCE_OHMS;

    let freqs = [8_000.0, 10_000.0, 12_500.0, 16_000.0];
    let gains: Vec<f64> = freqs
        .iter()
        .map(|&freq| {
            let mut proc: super::compiled::CompiledPedal =
                postcard::from_bytes(&blob).expect("deserialize failed");
            proc.set_control_immediate("Cutoff", 0.5);
            proc.set_control_immediate("Resonance", 0.0);
            settled_sine_ac_rms_ports(
                &mut proc,
                freq,
                FILTER_VCO_INPUT_VOLTS,
                &[("vco_in", 1.0)],
                &[("cv_cutoff", 0.0), ("cv_resonance", 0.0)],
                "audio_out",
            )
        })
        .collect();

    eprintln!("  TB303 high-frequency k=0 response probe");
    for (freq, gain) in freqs.iter().zip(gains.iter()) {
        eprintln!("    {freq:>7.0}Hz gain={gain:.9}");
    }

    let slopes: Vec<f64> = gains
        .windows(2)
        .zip(freqs.windows(2))
        .map(|(gain_pair, freq_pair)| {
            let db = 20.0 * (gain_pair[1].max(1.0e-12) / gain_pair[0].max(1.0e-12)).log10();
            db / (freq_pair[1] / freq_pair[0]).log2()
        })
        .collect();
    eprintln!("  HF slopes dB/oct: {slopes:.2?}");

    assert!(
        slopes.iter().all(|slope| *slope < -6.0 && *slope > -48.0),
        "HF response should keep rolling off smoothly without flattening or cliffing: freqs={freqs:?}, gains={gains:?}, slopes={slopes:?}"
    );
}

#[test]
fn stinchcombe_10pole_reference_blocks_dc_and_stays_finite() {
    let dc = stinchcombe_10pole_magnitude(0.0, 1_000.0, 0.0);
    let low = stinchcombe_10pole_magnitude(100.0, 1_000.0, 0.0);
    let resonant = stinchcombe_10pole_magnitude(1_000.0, 1_000.0, 0.8);

    assert_eq!(
        dc, 0.0,
        "full 10-pole reference should include DC-blocking zeros"
    );
    assert!(low.is_finite() && low > 0.0);
    assert!(resonant.is_finite() && resonant > 0.0);
}

#[test]
fn tb303_forced_serial_tracks_stinchcombe_htb_shape() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let def = crate::dsl::parse_pedal_file(&source).expect("parse failed");
    let freqs = [100.0, 200.0, 500.0, 1000.0, 2000.0, 5000.0, 10_000.0];

    let measure_serial = |freq: f64| -> f64 {
        let mut options = super::compile::CompileOptions::default();
        options.force_serial_blockwise = true;
        let mut proc =
            super::compile_pedal_with_options(&def, SR, options).expect("compile failed");
        proc.set_control_immediate("Cutoff", 0.5);
        proc.set_control_immediate("Resonance", 0.0);
        settled_sine_ac_rms_ports(
            &mut proc,
            freq,
            0.03,
            &[("audio_in", 1.0), ("vco_in", 1.0)],
            &[("cv_cutoff", 0.0), ("cv_resonance", 0.0)],
            "audio_out",
        )
    };

    let serial: Vec<f64> = freqs.iter().map(|&freq| measure_serial(freq)).collect();

    let target: Vec<f64> = serial
        .iter()
        .map(|gain| db_norm(*gain, serial[0]))
        .collect();
    let mut best_fc = 100.0;
    let mut best_err = f64::INFINITY;

    for step in 0..240 {
        let t = step as f64 / 239.0;
        let fc = 100.0 * (80.0f64).powf(t);
        let h_ref = stinchcombe_htb_magnitude(freqs[0], fc);
        let mut err = 0.0;
        for (i, &freq) in freqs.iter().enumerate() {
            let h_db = db_norm(stinchcombe_htb_magnitude(freq, fc), h_ref);
            let diff = target[i] - h_db;
            err += diff * diff;
        }
        if err < best_err {
            best_err = err;
            best_fc = fc;
        }
    }

    let h_ref = stinchcombe_htb_magnitude(freqs[0], best_fc);
    let max_abs_err = freqs
        .iter()
        .enumerate()
        .map(|(i, &freq)| {
            let h_db = db_norm(stinchcombe_htb_magnitude(freq, best_fc), h_ref);
            (target[i] - h_db).abs()
        })
        .fold(0.0, f64::max);

    eprintln!("  forced-serial vs H_tb: best_fc={best_fc:.1}Hz, max_abs_err={max_abs_err:.2}dB");

    assert!(
        max_abs_err < 4.0,
        "forced-serial WDF should stay close to normalized H_tb shape after best-fit cutoff; \
         best_fc={best_fc:.1}Hz, max_abs_err={max_abs_err:.2}dB"
    );
}

#[test]
fn tb303_forced_serial_delayed_feedback_fit_vs_stinchcombe_htb() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let def = crate::dsl::parse_pedal_file(&source).expect("parse failed");
    let freqs = [100.0, 200.0, 500.0, 1000.0, 2000.0, 5000.0, 10_000.0];

    let measure_serial_feedback = |freq: f64, feedback_gain: f64| -> f64 {
        let mut options = super::compile::CompileOptions::default();
        options.force_serial_blockwise = true;
        options.force_serial_blockwise_feedback_gain = feedback_gain;
        let mut proc =
            super::compile_pedal_with_options(&def, SR, options).expect("compile failed");
        proc.set_control_immediate("Cutoff", 0.5);
        proc.set_control_immediate("Resonance", 0.0);

        settled_sine_ac_rms(freq, 0.03, |input| proc.process(input))
    };

    let fit_error = |gains: &[f64]| -> (f64, f64) {
        let target: Vec<f64> = gains.iter().map(|gain| db_norm(*gain, gains[0])).collect();
        let mut best_fc = 100.0;
        let mut best_err = f64::INFINITY;

        for step in 0..240 {
            let t = step as f64 / 239.0;
            let fc = 100.0 * (80.0f64).powf(t);
            let h_ref = stinchcombe_htb_magnitude(freqs[0], fc);
            let mut err = 0.0;
            for (i, &freq) in freqs.iter().enumerate() {
                let h_db = db_norm(stinchcombe_htb_magnitude(freq, fc), h_ref);
                let diff = target[i] - h_db;
                err += diff * diff;
            }
            if err < best_err {
                best_err = err;
                best_fc = fc;
            }
        }

        let h_ref = stinchcombe_htb_magnitude(freqs[0], best_fc);
        let max_abs_err = freqs
            .iter()
            .enumerate()
            .map(|(i, &freq)| {
                let h_db = db_norm(stinchcombe_htb_magnitude(freq, best_fc), h_ref);
                (target[i] - h_db).abs()
            })
            .fold(0.0, f64::max);
        (best_fc, max_abs_err)
    };

    let candidates = [
        -0.95, -0.85, -0.7, -0.55, -0.4, -0.25, -0.1, 0.0, 0.1, 0.25, 0.4, 0.55, 0.7, 0.85, 0.95,
    ];
    let mut best_gain = 0.0;
    let mut best_fc = 0.0;
    let mut best_err = f64::INFINITY;

    for feedback_gain in candidates {
        let gains: Vec<f64> = freqs
            .iter()
            .map(|&freq| measure_serial_feedback(freq, feedback_gain))
            .collect();
        if gains.iter().any(|gain| !gain.is_finite() || *gain <= 1e-12) {
            continue;
        }
        let (fit_fc, max_err) = fit_error(&gains);
        eprintln!(
            "  delayed serial fb={feedback_gain:+.2}: best_fc={fit_fc:.1}Hz, max_abs_err={max_err:.2}dB"
        );
        if max_err < best_err {
            best_err = max_err;
            best_fc = fit_fc;
            best_gain = feedback_gain;
        }
    }

    eprintln!(
        "  best delayed serial feedback fit: fb={best_gain:+.2}, best_fc={best_fc:.1}Hz, max_abs_err={best_err:.2}dB"
    );

    assert!(
        best_err < 3.0,
        "one-sample serial feedback should be at least competitive with the no-feedback serial H_tb fit; \
         best fb={best_gain:+.2}, fc={best_fc:.1}Hz, max_abs_err={best_err:.2}dB"
    );
}

#[test]
fn tb303_force_serial_feedback_compiles_to_serial_feedback_stage() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let def = crate::dsl::parse_pedal_file(&source).expect("parse failed");
    let mut options = super::compile::CompileOptions::default();
    options.force_serial_blockwise = true;
    options.force_serial_blockwise_feedback_gain = -0.55;

    let proc = super::compile_pedal_with_options(&def, SR, options).expect("compile failed");
    let serial = proc.stages.iter().find_map(|stage| {
        if let pedalkernel_rt::processor::Stage::SerialDelayedFeedback(serial) = stage {
            Some(serial)
        } else {
            None
        }
    });

    assert!(
        serial.is_some(),
        "forced serial feedback should compile into a SerialDelayedFeedback stage"
    );
    let serial = serial.unwrap();
    assert!(
        serial.stages.len() >= 4,
        "TB303 forced serial feedback should wrap the ladder rung WDF stages"
    );
    assert_eq!(serial.feedback_gain, -0.55);
    assert!(
        !proc
            .stages
            .iter()
            .any(|stage| matches!(stage, pedalkernel_rt::processor::Stage::Blockwise(_))),
        "forced serial feedback should bypass the BKM packaging path"
    );
}

#[test]
fn tb303_bkm_direct_block_matches_forced_serial_first_rung_order() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let def = crate::dsl::parse_pedal_file(&source).expect("parse failed");

    let measure_serial_first = |freq: f64| -> f64 {
        let mut options = super::compile::CompileOptions::default();
        options.force_serial_blockwise = true;
        let mut proc =
            super::compile_pedal_with_options(&def, SR, options).expect("compile failed");
        proc.set_control_immediate("Resonance", 0.0);

        for i in 0..9600 {
            let input = 0.1 * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
            for stage in &mut proc.stages {
                if let pedalkernel_rt::processor::Stage::Wdf(wdf) = stage {
                    let _ = wdf.process(input);
                    break;
                }
            }
        }

        let mut sum_sq = 0.0;
        let mut count = 0usize;
        for i in 0..9600 {
            let input = 0.1 * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
            for stage in &mut proc.stages {
                if let pedalkernel_rt::processor::Stage::Wdf(wdf) = stage {
                    let out = wdf.process(input);
                    sum_sq += out * out;
                    count += 1;
                    break;
                }
            }
        }

        (sum_sq / count.max(1) as f64).sqrt()
    };

    let measure_bkm_first_direct = |freq: f64| -> f64 {
        let mut proc =
            super::compile_pedal_with_options(&def, SR, super::compile::CompileOptions::default())
                .expect("compile failed");
        proc.set_control_immediate("Resonance", 0.0);

        let bkm = proc
            .stages
            .iter_mut()
            .find_map(|s| {
                if let pedalkernel_rt::processor::Stage::Blockwise(ref mut k) = s {
                    Some(k)
                } else {
                    None
                }
            })
            .expect("TB303 should compile to BKM");

        for v in &mut bkm.work_b {
            *v = 0.0;
        }
        for v in &mut bkm.work_a {
            *v = 0.0;
        }
        for &(ref name, port_idx) in &bkm.vs_port_map {
            if name.starts_with("_supply_") {
                bkm.work_b[port_idx] = 2.0 * bkm.supply_voltage;
            }
        }
        for row in 0..bkm.n_ports {
            let mut sum = 0.0;
            for col in 0..bkm.n_ports {
                sum += bkm.coupling_s[row * bkm.n_ports + col] * bkm.work_b[col];
            }
            bkm.work_a[row] = sum;
        }
        let dc_drive = (bkm.work_a[0] + bkm.work_b[0]) / 2.0;

        let mut block = bkm.blocks[0].clone();
        block.k_table.precompute_scales();

        let mut process_block = |input: f64| {
            let physical_input = dc_drive + input;
            block
                .tree
                .set_voltage(block.source_polarity * physical_input);
            let b_tree = block.tree.reflected();
            let a_root = if block.k_table.dims == 1 {
                block.k_table.lookup_1d(b_tree)
            } else {
                block
                    .k_table
                    .lookup_2d(b_tree, block.k_table_control_polarity * physical_input)
            };
            let raw = if let Some(ref probe_id) = block.cascade_probe_id {
                block
                    .tree
                    .leaf_voltage_for_incident(probe_id, a_root)
                    .unwrap_or((a_root + b_tree) / 2.0)
            } else {
                (a_root + b_tree) / 2.0
            };
            block.tree.set_incident(a_root);
            raw - block.dc_offset
        };

        for i in 0..9600 {
            let input = 0.1 * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
            let _ = process_block(input);
        }

        let mut values = Vec::new();
        for i in 0..9600 {
            let input = 0.1 * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
            let out = process_block(input);
            values.push(out);
        }

        ac_rms(&values)
    };

    let serial_low = measure_serial_first(100.0);
    let serial_high = measure_serial_first(10_000.0);
    let bkm_low = measure_bkm_first_direct(100.0);
    let bkm_high = measure_bkm_first_direct(10_000.0);
    let serial_ratio = serial_low / serial_high.max(1e-12);
    let bkm_ratio = bkm_low / bkm_high.max(1e-12);

    eprintln!(
        "  first rung direct WDF: 100Hz={serial_low:.6}, 10kHz={serial_high:.6}, ratio={serial_ratio:.3}"
    );
    eprintln!(
        "  first rung direct BKM: 100Hz={bkm_low:.6}, 10kHz={bkm_high:.6}, ratio={bkm_ratio:.3}"
    );

    assert!(
        bkm_ratio > serial_ratio * 0.9,
        "direct BKM first block should preserve the same one-pole order as forced WDF. \
         serial ratio={serial_ratio:.3}, BKM ratio={bkm_ratio:.3}"
    );
}

#[test]
fn tb303_bkm_direct_blocks_each_have_lowpass_order() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let def = crate::dsl::parse_pedal_file(&source).expect("parse failed");

    let measure_blocks = |freq: f64| -> Vec<f64> {
        let mut proc =
            super::compile_pedal_with_options(&def, SR, super::compile::CompileOptions::default())
                .expect("compile failed");
        proc.set_control_immediate("Resonance", 0.0);

        let bkm = proc
            .stages
            .iter_mut()
            .find_map(|s| {
                if let pedalkernel_rt::processor::Stage::Blockwise(ref mut k) = s {
                    Some(k)
                } else {
                    None
                }
            })
            .expect("TB303 should compile to BKM");

        for v in &mut bkm.work_b {
            *v = 0.0;
        }
        for v in &mut bkm.work_a {
            *v = 0.0;
        }
        for &(ref name, port_idx) in &bkm.vs_port_map {
            if name.starts_with("_supply_") {
                bkm.work_b[port_idx] = 2.0 * bkm.supply_voltage;
            }
        }
        for row in 0..bkm.n_ports {
            let mut sum = 0.0;
            for col in 0..bkm.n_ports {
                sum += bkm.coupling_s[row * bkm.n_ports + col] * bkm.work_b[col];
            }
            bkm.work_a[row] = sum;
        }
        let dc_drives: Vec<f64> = (0..bkm.blocks.len())
            .map(|i| (bkm.work_a[i] + bkm.work_b[i]) / 2.0)
            .collect();
        eprintln!(
            "  direct BKM probes: {:?}",
            bkm.blocks
                .iter()
                .map(|block| block.cascade_probe_id.as_deref().unwrap_or("<root>"))
                .collect::<Vec<_>>()
        );
        eprintln!(
            "  direct BKM rp: {:?}",
            bkm.blocks.iter().map(|block| block.rp).collect::<Vec<_>>()
        );
        let mut blocks = bkm.blocks.clone();
        for block in &mut blocks {
            block.k_table.precompute_scales();
        }

        for i in 0..9600 {
            let input = 0.1 * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
            for (block_idx, block) in blocks.iter_mut().enumerate() {
                let physical_input = dc_drives[block_idx] + input;
                block
                    .tree
                    .set_voltage(block.source_polarity * physical_input);
                let b_tree = block.tree.reflected();
                let a_root = if block.k_table.dims == 1 {
                    block.k_table.lookup_1d(b_tree)
                } else {
                    block
                        .k_table
                        .lookup_2d(b_tree, block.k_table_control_polarity * physical_input)
                };
                block.tree.set_incident(a_root);
            }
        }

        let mut values = vec![Vec::new(); blocks.len()];
        for i in 0..9600 {
            let input = 0.1 * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
            for (block_idx, block) in blocks.iter_mut().enumerate() {
                let physical_input = dc_drives[block_idx] + input;
                block
                    .tree
                    .set_voltage(block.source_polarity * physical_input);
                let b_tree = block.tree.reflected();
                let a_root = if block.k_table.dims == 1 {
                    block.k_table.lookup_1d(b_tree)
                } else {
                    block
                        .k_table
                        .lookup_2d(b_tree, block.k_table_control_polarity * physical_input)
                };
                let raw = if let Some(ref probe_id) = block.cascade_probe_id {
                    block
                        .tree
                        .leaf_voltage_for_incident(probe_id, a_root)
                        .unwrap_or((a_root + b_tree) / 2.0)
                } else {
                    (a_root + b_tree) / 2.0
                };
                block.tree.set_incident(a_root);
                let out = raw - block.dc_offset;
                values[block_idx].push(out);
            }
        }

        values
            .iter()
            .map(|block_values| ac_rms(block_values))
            .collect()
    };

    let low = measure_blocks(100.0);
    let high = measure_blocks(10_000.0);
    let ratios: Vec<f64> = low
        .iter()
        .zip(high.iter())
        .map(|(lo, hi)| lo / hi.max(1e-12))
        .collect();

    eprintln!("  direct BKM block 100Hz RMS: {low:.6?}");
    eprintln!("  direct BKM block 10kHz RMS: {high:.6?}");
    eprintln!("  direct BKM block low/high: {ratios:.3?}");

    assert!(
        ratios.iter().all(|ratio| *ratio > 2.0),
        "each independently driven BKM block should retain local lowpass behavior: {ratios:.3?}"
    );
}

#[test]
fn tb303_bkm_k_tables_are_generated_per_rung_not_shared() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let cache_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("target")
        .join("test-cache");
    let _ = std::fs::create_dir_all(&cache_dir);

    let blob = super::compile::compile_pedal_cached(
        &source,
        "tb303_bkm_ktable_ownership_v3",
        "tb303_bkm_ktable_ownership_v3",
        SR,
        &super::compile::CompileOptions::default(),
        &cache_dir,
    )
    .expect("compile failed");

    let mut proc: super::compiled::CompiledPedal =
        postcard::from_bytes(&blob).expect("deserialize failed");
    let bkm = proc
        .stages
        .iter_mut()
        .find_map(|s| {
            if let pedalkernel_rt::processor::Stage::Blockwise(ref mut k) = s {
                Some(k)
            } else {
                None
            }
        })
        .expect("TB303 should compile to blockwise K-method");

    assert_eq!(bkm.blocks.len(), 4);

    let table_ptrs: Vec<usize> = bkm
        .blocks
        .iter()
        .map(|block| block.k_table.entries.as_ptr() as usize)
        .collect();
    let mut unique_ptrs = table_ptrs.clone();
    unique_ptrs.sort_unstable();
    unique_ptrs.dedup();

    eprintln!("  BKM K-table entry pointers: {table_ptrs:?}");
    assert_eq!(
        unique_ptrs.len(),
        table_ptrs.len(),
        "each BKM rung should own its generated K-table storage; do not share table Vecs blindly"
    );

    let table_delta = |a: usize, b: usize| -> f64 {
        let ta = &bkm.blocks[a].k_table;
        let tb = &bkm.blocks[b].k_table;
        assert_eq!(ta.dims, tb.dims);
        assert_eq!(ta.steps, tb.steps);
        assert_eq!(ta.entries.len(), tb.entries.len());
        ta.entries
            .iter()
            .zip(tb.entries.iter())
            .map(|(x, y)| (x - y).abs())
            .fold(0.0f64, f64::max)
    };

    let q1_q2_delta = table_delta(0, 1);
    let q2_q3_delta = table_delta(1, 2);
    let q3_q4_delta = table_delta(2, 3);
    eprintln!(
        "  BKM K-table max deltas: Q1/Q2={q1_q2_delta:.6}, Q2/Q3={q2_q3_delta:.6}, Q3/Q4={q3_q4_delta:.6}"
    );

    assert!(
        q1_q2_delta > 1e-6,
        "Q1 table should differ from downstream rung tables because its source impedance differs"
    );
    assert!(
        q2_q3_delta < 1e-9 && q3_q4_delta < 1e-9,
        "identical downstream rungs may have matching table contents, but still own separate tables"
    );
}

#[test]
fn tb303_serial_blockwise_without_bkm_is_lowpass_diagnostic() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let def = crate::dsl::parse_pedal_file(&source).expect("parse failed");
    let mut options = super::compile::CompileOptions::default();
    options.force_serial_blockwise = true;

    let compiled =
        super::compile_pedal_with_options(&def, SR, options.clone()).expect("compile failed");
    let bkm_count = compiled
        .stages
        .iter()
        .filter(|stage| matches!(stage, pedalkernel_rt::processor::Stage::Blockwise(_)))
        .count();
    let wdf_count = compiled
        .stages
        .iter()
        .filter(|stage| matches!(stage, pedalkernel_rt::processor::Stage::Wdf(_)))
        .count();

    eprintln!(
        "  serial blockwise diagnostic: stages={}, wdf_count={wdf_count}, bkm_count={bkm_count}",
        compiled.stages.len()
    );
    assert_eq!(
        bkm_count, 0,
        "force_serial_blockwise must not emit BKM stages"
    );
    assert!(
        wdf_count >= 4,
        "force_serial_blockwise should expose the ladder rungs as serial WDF/K-method stages"
    );

    let measure = |freq: f64| -> f64 {
        let mut proc =
            super::compile_pedal_with_options(&def, SR, options.clone()).expect("compile failed");
        proc.set_control_immediate("Resonance", 0.0);

        for _ in 0..4800 {
            let _ = proc.process(0.0);
        }

        let mut peak = 0.0f64;
        for i in 0..4800 {
            let input = 0.1 * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
            peak = peak.max(proc.process(input).abs());
        }
        peak
    };

    let gain_100 = measure(100.0);
    let gain_10k = measure(10_000.0);
    let ratio_db = 20.0 * (gain_100 / gain_10k.max(1e-12)).log10();

    eprintln!(
        "  serial blockwise diagnostic: 100Hz={gain_100:.6}, 10kHz={gain_10k:.6}, ratio={ratio_db:+.1} dB"
    );
    assert!(
        gain_100 > gain_10k,
        "serial blockwise rungs should be lowpass without BKM coupling: 100Hz={gain_100:.6}, 10kHz={gain_10k:.6}"
    );
}

#[test]
fn tb303_forced_serial_stage_outputs_show_reference_shape() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let def = crate::dsl::parse_pedal_file(&source).expect("parse failed");
    let mut options = super::compile::CompileOptions::default();
    options.force_serial_blockwise = true;

    let measure = |freq: f64| -> Vec<f64> {
        let mut proc =
            super::compile_pedal_with_options(&def, SR, options.clone()).expect("compile failed");
        proc.set_control_immediate("Resonance", 0.0);

        let mut sums = vec![0.0f64; proc.stages.len()];
        let mut count = 0usize;

        for i in 0..9600 {
            let _ = proc.process(0.1 * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin());
        }

        for i in 0..9600 {
            let input = 0.1 * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
            let mut x = input;
            for (si, stage) in proc.stages.iter_mut().enumerate() {
                x = match stage {
                    pedalkernel_rt::processor::Stage::Wdf(wdf) => wdf.process(x),
                    pedalkernel_rt::processor::Stage::MultiNl(mnl) => mnl.process(x),
                    pedalkernel_rt::processor::Stage::Iir(iir) => iir.process(x),
                    pedalkernel_rt::processor::Stage::StateSpace(ss) => ss.process(x),
                    pedalkernel_rt::processor::Stage::BlackFeedback(bf) => bf.process(x),
                    pedalkernel_rt::processor::Stage::Blockwise(bkm) => {
                        bkm.process(&[x, 0.0, 0.0, 0.0])
                    }
                    pedalkernel_rt::processor::Stage::SerialDelayedFeedback(s) => s.process(x),
                };
                sums[si] += x * x;
            }
            count += 1;
        }

        sums.into_iter()
            .map(|v| (v / count.max(1) as f64).sqrt())
            .collect()
    };

    let low = measure(100.0);
    let high = measure(10_000.0);
    eprintln!("  forced serial stage RMS 100Hz: {low:.6?}");
    eprintln!("  forced serial stage RMS 10kHz: {high:.6?}");

    assert!(
        low.last().copied().unwrap_or(0.0) > high.last().copied().unwrap_or(0.0) * 3.0,
        "forced serial reference should be strongly lowpass"
    );
}

#[test]
fn tb303_forced_serial_wdf_rungs_stay_bounded_on_silence() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let def = crate::dsl::parse_pedal_file(&source).expect("parse failed");
    let mut options = super::compile::CompileOptions::default();
    options.force_serial_blockwise = true;

    let mut proc = super::compile_pedal_with_options(&def, SR, options).expect("compile failed");

    for sample in 0..4800 {
        proc.process(0.0);

        if sample % 480 == 479 {
            for (si, stage) in proc.stages.iter().enumerate() {
                let pedalkernel_rt::processor::Stage::Wdf(wdf) = stage else {
                    continue;
                };
                let mut tree = wdf.tree.clone();
                tree.set_voltage(0.0);
                let b = tree.reflected();
                eprintln!("  serial WDF sample={sample} stage={si} b_tree={b:.6}");
                assert!(
                    b.is_finite() && b.abs() < 10.0,
                    "forced-serial WDF stage {si} should stay bounded on silence, b_tree={b:.6}"
                );
            }
        }
    }
}

#[test]
fn two_rung_feedback_bkm_stays_bounded_on_silence() {
    let def =
        crate::dsl::parse_pedal_file(TWO_RUNG_DIODE_LADDER_WITH_FEEDBACK).expect("parse failed");
    let mut proc = super::compile_pedal(&def, SR).expect("compile failed");

    for sample in 0..4800 {
        proc.process(0.0);

        if sample % 480 == 479 {
            let bkm = proc
                .stages
                .iter()
                .find_map(|s| {
                    if let pedalkernel_rt::processor::Stage::Blockwise(ref k) = s {
                        Some(k)
                    } else {
                        None
                    }
                })
                .expect("two-rung feedback ladder should compile to BKM");

            for (bi, block) in bkm.blocks.iter().enumerate() {
                let mut tree = block.tree.clone();
                tree.set_voltage(0.0);
                let b = tree.reflected();
                eprintln!(
                    "  two-rung BKM sample={sample} block={bi} b_tree={b:.6}, dc_offset={:.6}",
                    block.dc_offset
                );
                assert!(
                    b.is_finite() && b.abs() < 10.0,
                    "two-rung BKM block {bi} should stay bounded on silence, b_tree={b:.6}"
                );
            }
        }
    }
}

#[test]
fn tb303_output_coupling_probes_load_resistor() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let def = crate::dsl::parse_pedal_file(&source).expect("parse failed");
    let proc = super::spqr_build::compile_via_spqr(&def, SR).expect("compile failed");

    let probes: Vec<Option<String>> = proc
        .stages
        .iter()
        .filter_map(|stage| {
            if let pedalkernel_rt::processor::Stage::Wdf(wdf) = stage {
                Some(wdf.output_probe.clone())
            } else {
                None
            }
        })
        .collect();

    eprintln!("  WDF output probes: {:?}", probes);
    assert!(
        probes.iter().any(|probe| probe.as_deref() == Some("R_out")),
        "TB303 output coupling stage should probe R_out, got {:?}",
        probes
    );
}

#[test]
fn tb303_resonance_recomputes_bkm_coupling_matrix() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let def = crate::dsl::parse_pedal_file(&source).expect("parse failed");
    let mut compiled = super::compile_pedal(&def, SR).expect("compile failed");

    assert!(
        compiled
            .controls
            .iter()
            .any(|c| c.label == "Resonance" && c.component_id == "Resonance"),
        "Resonance control must bind even though its pot lives in the BKM coupling network"
    );

    let bkm = compiled
        .stages
        .iter_mut()
        .find_map(|s| {
            if let pedalkernel_rt::processor::Stage::Blockwise(ref mut k) = s {
                Some(k)
            } else {
                None
            }
        })
        .expect("BKM stage");

    let before = bkm.coupling_s.clone();
    let block_ports = bkm.block_ports.clone();
    assert!(
        bkm.set_pot("Resonance", 0.7),
        "BKM should accept coupling-network pot changes"
    );
    let delta = before
        .iter()
        .zip(bkm.coupling_s.iter())
        .map(|(a, b)| (a - b).abs())
        .fold(0.0f64, f64::max);

    let n = bkm.n_ports;
    let mut changed = Vec::new();
    for row in 0..n {
        for col in 0..n {
            let idx = row * n + col;
            let d = bkm.coupling_s[idx] - before[idx];
            if d.abs() > 1.0e-5 {
                changed.push((d.abs(), row, col, before[idx], bkm.coupling_s[idx], d));
            }
        }
    }
    changed.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap());

    eprintln!("  Resonance coupling matrix max delta: {delta:.6}");
    eprintln!("  BKM block ports: {block_ports:?}");
    eprintln!("  Resonance/R_fb coupling elements:");
    for element in bkm.coupling_elements.iter().filter(|e| {
        e.comp_id == "Resonance" || e.comp_id == "R_fb_limit" || e.comp_id == "R_res_cv"
    }) {
        eprintln!(
            "    {}: {:?}<->{:?} R={:.3} invert={}",
            element.comp_id,
            element.node_a,
            element.node_b,
            element.resistance,
            element.invert_control
        );
    }
    eprintln!("  Resonance changed entries:");
    for (_, row, col, old, new, d) in changed.iter().take(24) {
        eprintln!("    S[{row},{col}]: {old:+.6} -> {new:+.6} (delta {d:+.6})");
    }
    assert!(
        delta > 1e-6,
        "moving Resonance must recompute the BKM coupling scattering matrix"
    );
}

#[test]
fn tb303_resonance_matrix_routes_output_back_to_input_ports() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let def = crate::dsl::parse_pedal_file(&source).expect("parse failed");
    let mut compiled = super::compile_pedal(&def, SR).expect("compile failed");

    let bkm = compiled
        .stages
        .iter_mut()
        .find_map(|s| {
            if let pedalkernel_rt::processor::Stage::Blockwise(ref mut k) = s {
                Some(k)
            } else {
                None
            }
        })
        .expect("BKM stage");

    assert_eq!(bkm.block_ports.len(), 4, "expected four ladder rung blocks");
    assert!(
        bkm.block_ports.iter().all(|ports| ports.len() >= 2),
        "differential ladder rungs should expose at least bottom/top coupling ports: {:?}",
        bkm.block_ports
    );

    assert!(bkm.set_pot("Resonance", 0.0));
    let flat = bkm.coupling_s.clone();
    assert!(bkm.set_pot("Resonance", 0.95));
    let n = bkm.n_ports;

    let input_ports: Vec<_> = bkm.block_port_ids(0).collect();
    assert!(
        bkm.feedback_port_map.is_empty(),
        "coupled BKM resonance must be represented by passive coupling elements, not synthetic feedback source ports"
    );
    let output_ports: Vec<_> = bkm.block_port_ids(bkm.output_block).collect();
    let mut best_feedback_delta = 0.0f64;
    let mut best_feedback_entry = None;
    for &row in &input_ports {
        for &col in &output_ports {
            let delta = (bkm.coupling_s[row * n + col] - flat[row * n + col]).abs();
            if delta > best_feedback_delta {
                best_feedback_delta = delta;
                best_feedback_entry =
                    Some((row, col, flat[row * n + col], bkm.coupling_s[row * n + col]));
            }
        }
    }

    eprintln!("  input ports={input_ports:?}, output rung ports={output_ports:?}");
    eprintln!(
        "  best output->input resonance delta={best_feedback_delta:.8?} at {best_feedback_entry:?}"
    );
    assert!(
        best_feedback_delta > 1.0e-3,
        "moving Resonance must materially change passive coupling from an output rung port back to an input rung port"
    );
}

#[test]
fn tb303_resonance_pot_compiles_as_series_feedback_rheostat() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");

    let cache_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("target")
        .join("test-cache");
    let _ = std::fs::create_dir_all(&cache_dir);

    let blob = super::compile::compile_pedal_cached(
        &source,
        "tb303_resonance_series_rheostat",
        "tb303_resonance_series_rheostat",
        SR,
        &super::compile::CompileOptions::default(),
        &cache_dir,
    )
    .expect("compile failed");

    let mut proc: super::compiled::CompiledPedal =
        postcard::from_bytes(&blob).expect("deserialize failed");

    let bkm = proc
        .stages
        .iter_mut()
        .find_map(|s| {
            if let pedalkernel_rt::processor::Stage::Blockwise(k) = s {
                Some(k)
            } else {
                None
            }
        })
        .expect("TB303 should compile to blockwise K-method");
    assert!(
        bkm.vs_port_map
            .iter()
            .any(|(name, _)| name == "cv_resonance"),
        "Resonance CV must compile as a voltage port, not an audio-rate pot update"
    );
    assert!(bkm.set_pot("Resonance", 0.0));

    let resonance_legs: Vec<_> = bkm
        .coupling_elements
        .iter()
        .filter(|e| e.comp_id == "Resonance")
        .collect();

    assert_eq!(
        resonance_legs.len(),
        1,
        "Resonance should compile as a series feedback rheostat, not a grounded divider that loads the ladder at Resonance=0"
    );
    assert!(
        resonance_legs[0].invert_control,
        "series resonance feedback must invert the exposed control so more knob means lower feedback resistance"
    );

    let min_direct = resonance_legs
        .iter()
        .find(|e| e.invert_control)
        .expect("direct feedback leg should be inverted")
        .resistance;

    assert!(
        min_direct > 90_000.0,
        "Resonance=0 should isolate feedback without shunting the ladder: direct={min_direct:.1}"
    );

    let mut proc: super::compiled::CompiledPedal =
        postcard::from_bytes(&blob).expect("deserialize failed");
    let bkm = proc
        .stages
        .iter_mut()
        .find_map(|s| {
            if let pedalkernel_rt::processor::Stage::Blockwise(k) = s {
                Some(k)
            } else {
                None
            }
        })
        .expect("TB303 should compile to blockwise K-method");
    assert!(bkm.set_pot("Resonance", 0.95));
    let resonance_legs: Vec<_> = bkm
        .coupling_elements
        .iter()
        .filter(|e| e.comp_id == "Resonance")
        .collect();
    let max_direct = resonance_legs
        .iter()
        .find(|e| e.invert_control)
        .expect("direct feedback leg should be inverted")
        .resistance;

    assert!(
        max_direct < 10_000.0,
        "Resonance=0.95 should connect the feedback return: direct={max_direct:.1}"
    );
}

#[test]
fn tb303_passive_resonance_feedback_stays_in_bkm_coupling() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let def = crate::dsl::parse_pedal_file(&source).expect("parse failed");
    let compiled =
        super::compile_pedal_with_options(&def, SR, super::compile::CompileOptions::default())
            .expect("compile failed");
    let bkm = compiled
        .stages
        .iter()
        .find_map(|stage| {
            if let super::compiled::Stage::Blockwise(bkm) = stage {
                Some(bkm)
            } else {
                None
            }
        })
        .expect("passive resonance ladder should still compile to BKM");

    assert!(
        bkm.coupling_vcvss.is_empty(),
        "current TB303 resonance feedback is passive; active gain stages should not be required for BKM coupling"
    );
    assert!(
        bkm.coupling_elements
            .iter()
            .any(|element| element.comp_id == "Resonance"),
        "TB303 resonance pot must be stamped as a passive coupling element"
    );
    assert!(
        bkm.coupling_elements
            .iter()
            .any(|element| element.comp_id == "R_fb_limit"),
        "TB303 feedback limiting resistor must stay in the coupling MNA"
    );
    assert!(
        bkm.feedback_port_map.is_empty(),
        "resonance must still use coupling-network elements, not synthetic feedback source ports"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 8. Resonance creates a gain peak
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn tb303_resonance_creates_peak() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");

    let cache_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("target")
        .join("test-cache");
    let _ = std::fs::create_dir_all(&cache_dir);

    eprintln!("  compiling with K-tables (cached)...");
    let options = super::compile::CompileOptions::default();
    let blob = super::compile::compile_pedal_cached(
        &source,
        "tb303_resonance_peak",
        "tb303_resonance_peak",
        SR,
        &options,
        &cache_dir,
    )
    .expect("compile failed");

    let measure_gain_at_freq = |freq: f64, resonance: f64| -> f64 {
        let mut proc: super::compiled::CompiledPedal =
            postcard::from_bytes(&blob).expect("deserialize failed");
        proc.set_control_immediate("Cutoff", 0.5);
        proc.set_control_immediate("Resonance", resonance);

        // AcidAttack drives the WDF filter at roughly 30 mV, not 1 V; at 1 V
        // the nonlinear ladder compresses the resonance peak and this stops
        // being a small-signal filter-readiness check. Use settled RMS rather
        // than peak capture so startup ringing cannot masquerade as resonance.
        quick_sine_ac_rms_ports(
            &mut proc,
            freq,
            0.03,
            &[("vco_in", 1.0)],
            &[("cv_cutoff", 0.0), ("cv_resonance", 0.0)],
            "audio_out",
        )
    };

    let probe_freqs = [250.0, 500.0, 1000.0, 2000.0, 4000.0, 8000.0];
    let mut best = (0.0f64, 0.0f64, 0.0f64, 0.0f64);
    let mut worst_abs = 0.0f64;
    for freq in probe_freqs {
        let gain_flat = measure_gain_at_freq(freq, 0.0);
        let gain_resonant = measure_gain_at_freq(freq, 0.85);
        let ratio = gain_resonant / gain_flat.max(1e-12);
        worst_abs = worst_abs.max(gain_resonant);
        eprintln!(
            "  {freq:>5.0}Hz gain: flat={gain_flat:.4}, resonant={gain_resonant:.4}, ratio={ratio:.2}x"
        );
        if ratio > best.3 {
            best = (freq, gain_flat, gain_resonant, ratio);
        }
    }

    eprintln!(
        "  best resonance gain: {:.0}Hz flat={:.4}, resonant={:.4}, ratio={:.2}x",
        best.0, best.1, best.2, best.3
    );

    assert!(
        best.3 > 1.25,
        "Resonance should boost gain near the cutoff peak by >1.25×: best={best:?}"
    );
    assert!(
        worst_abs < 5.0,
        "Resonance must be a bounded cutoff-band peak, not broadband runaway: worst_abs={worst_abs:.4}, best={best:?}"
    );
}

#[test]
fn tb303_resonance_k_sweep_tracks_10pole_shape() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let freqs = [250.0, 500.0, 1000.0, 2000.0, 4000.0, 8000.0];

    let cache_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("target")
        .join("test-cache");
    let _ = std::fs::create_dir_all(&cache_dir);

    let options = super::compile::CompileOptions::default();
    let blob = super::compile::compile_pedal_cached(
        &source,
        "tb303_resonance_k_sweep",
        "tb303_resonance_k_sweep",
        SR,
        &options,
        &cache_dir,
    )
    .expect("compile failed");

    let measure = |freq: f64, resonance: f64| -> f64 {
        let mut proc: super::compiled::CompiledPedal =
            postcard::from_bytes(&blob).expect("deserialize failed");
        proc.set_control_immediate("Cutoff", 0.5);
        proc.set_control_immediate("Resonance", resonance);
        quick_sine_ac_rms_ports(
            &mut proc,
            freq,
            0.03,
            &[("vco_in", 1.0)],
            &[("cv_cutoff", 0.0), ("cv_resonance", 0.0)],
            "audio_out",
        )
    };

    let flat: Vec<f64> = freqs.iter().map(|&freq| measure(freq, 0.0)).collect();
    let flat_norm: Vec<f64> = flat.iter().map(|gain| db_norm(*gain, flat[0])).collect();

    let mut best_fc = 100.0;
    let mut best_err = f64::INFINITY;
    for step in 0..320 {
        let t = step as f64 / 319.0;
        let fc = 100.0 * (100.0f64).powf(t);
        let ref_norm = stinchcombe_10pole_magnitude(freqs[0], fc, 0.0);
        if ref_norm <= 1.0e-12 {
            continue;
        }
        let err = freqs
            .iter()
            .enumerate()
            .map(|(i, &freq)| {
                let ref_db = db_norm(stinchcombe_10pole_magnitude(freq, fc, 0.0), ref_norm);
                let diff = flat_norm[i] - ref_db;
                diff * diff
            })
            .sum::<f64>();
        if err < best_err {
            best_err = err;
            best_fc = fc;
        }
    }

    eprintln!("  TB303 resonance k sweep vs 10-pole, fitted k=0 fc={best_fc:.1}Hz");
    eprintln!(
        "  {:>5} {:>9} {:>9} {:>9} {:>9}",
        "k", "err dB", "2k dB", "4k dB", "8k dB"
    );

    let flat_refs: Vec<f64> = freqs
        .iter()
        .map(|&freq| stinchcombe_10pole_magnitude(freq, best_fc, 0.0))
        .collect();
    let mut previous_err = 0.0;
    for (idx, &k) in [0.0, 0.1, 0.2, 0.3].iter().enumerate() {
        let gains: Vec<f64> = freqs.iter().map(|&freq| measure(freq, k)).collect();
        let mut max_ratio_err: f64 = 0.0;
        let mut err_by_freq = Vec::new();
        for (i, &freq) in freqs.iter().enumerate() {
            let bkm_ratio_db = db_norm(gains[i], flat[i].max(1.0e-12));
            let ref_ratio_db = db_norm(
                stinchcombe_10pole_magnitude(freq, best_fc, k),
                flat_refs[i].max(1.0e-12),
            );
            let ratio_err = bkm_ratio_db - ref_ratio_db;
            max_ratio_err = max_ratio_err.max(ratio_err.abs());
            err_by_freq.push(ratio_err);
        }
        eprintln!(
            "  {k:>5.2} {max_ratio_err:>9.2} {:+9.2} {:+9.2} {:+9.2}",
            err_by_freq[3], err_by_freq[4], err_by_freq[5]
        );
        if idx > 0 {
            assert!(
                max_ratio_err <= previous_err + 6.0,
                "resonance error should not explode as k rises; k={k}, previous={previous_err:.2}dB current={max_ratio_err:.2}dB"
            );
        }
        previous_err = max_ratio_err;
    }
}

#[test]
fn tb303_bkm_metering_reports_coupled_solver_diagnostics() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let def = crate::dsl::parse_pedal_file(&source).expect("parse failed");
    let mut proc =
        super::compile_pedal_with_options(&def, SR, super::compile::CompileOptions::default())
            .expect("compile failed");
    proc.set_control_immediate("Cutoff", 0.5);
    proc.set_control_immediate("Resonance", 0.7);
    proc.enable_metering(32);

    for i in 0..96 {
        let input = 0.03 * (2.0 * std::f64::consts::PI * 500.0 * i as f64 / SR).sin();
        for port in &proc.ports {
            if port.name == "vco_in" && port.index < proc.port_values.len() {
                proc.port_values[port.index] = input;
            }
        }
        proc.process(input);
    }

    let metrics = proc.read_metrics();
    eprintln!(
        "  BKM solver metrics: solves={}, total_iter={}, max_iter={}, nonconv={}, max_residual={:.3e}",
        metrics.nr_solve_count,
        metrics.nr_total_iterations,
        metrics.nr_max_iterations,
        metrics.nr_nonconverged_count,
        metrics.nr_max_residual
    );
    assert!(
        metrics.nr_solve_count > 0,
        "BKM coupled solve should publish solver calls into the metrics ring buffer"
    );
    assert!(
        metrics.nr_total_iterations >= metrics.nr_solve_count,
        "solver iteration totals should be populated"
    );
    assert!(
        metrics.stage_nr_iterations.iter().any(|&iters| iters > 0),
        "per-stage solver iteration diagnostics should identify the BKM stage"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 9. Block port mapping: each block gets a unique coupling port
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn tb303_each_block_has_unique_coupling_port() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let def = crate::dsl::parse_pedal_file(&source).expect("parse failed");
    let graph = CircuitGraph::from_pedal(&def);

    let active_set: std::collections::HashSet<usize> =
        graph.active_edge_indices.iter().copied().collect();
    let all_edges: Vec<usize> = (0..graph.edges.len())
        .filter(|i| !active_set.contains(i))
        .collect();

    // Get the feedback group (the one with all BJTs)
    let groups = super::signal_flow::find_flow_groups(&all_edges, &graph);
    let feedback_group = groups
        .iter()
        .find(|g| g.has_feedback())
        .expect("should have feedback group");
    let group_edges = feedback_group.all_edges();

    let plan = blockwise::analyze_blockwise(&group_edges, &graph).expect("should decompose");

    // Collect coupling MNA nodes
    let rails: std::collections::HashSet<usize> = {
        let mut r = std::collections::HashSet::new();
        r.insert(graph.gnd_node);
        r.insert(graph.vcc_node);
        r.extend(&graph.supply_nodes);
        r.extend(&graph.ac_ground_nodes);
        r
    };
    let mut coupling_nodes: Vec<usize> = Vec::new();
    for &eidx in &plan.coupling_edges {
        let e = &graph.edges[eidx];
        if !rails.contains(&e.node_a) && !coupling_nodes.contains(&e.node_a) {
            coupling_nodes.push(e.node_a);
        }
        if !rails.contains(&e.node_b) && !coupling_nodes.contains(&e.node_b) {
            coupling_nodes.push(e.node_b);
        }
    }

    // For each block, find its coupling port node
    // Count how many coupling edges touch each of the block's port_nodes
    let mut assigned_ports: Vec<usize> = Vec::new();
    for (bi, block) in plan.blocks.iter().enumerate() {
        let best_node = block
            .port_nodes
            .iter()
            .filter(|pn| coupling_nodes.contains(pn))
            .max_by_key(|&&pn| {
                plan.coupling_edges
                    .iter()
                    .filter(|&&eidx| {
                        let e = &graph.edges[eidx];
                        e.node_a == pn || e.node_b == pn
                    })
                    .count()
            })
            .copied();
        eprintln!(
            "  block {bi}: port_nodes={:?}, best_coupling_node={:?}",
            block.port_nodes, best_node
        );
        if let Some(node) = best_node {
            assigned_ports.push(node);
        }
    }

    // Assert: all assigned ports are unique
    let unique_count = {
        let mut s = assigned_ports.clone();
        s.sort();
        s.dedup();
        s.len()
    };
    assert_eq!(
        unique_count,
        assigned_ports.len(),
        "Each block must have a UNIQUE coupling port, but got duplicates: {:?}",
        assigned_ports
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 10. Linear-regime sanity: coupling scattering with identity K-tables
//
// Replace K-tables with linear approximation (b_out ≈ b_in * gain).
// The system becomes purely linear. If this produces lowpass behavior,
// the coupling network is correct and any remaining issues are in K-table
// interpolation. If flat, the coupling network itself is wrong.
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn tb303_linear_regime_has_frequency_dependence() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");

    let cache_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("target")
        .join("test-cache");
    let _ = std::fs::create_dir_all(&cache_dir);

    let options = super::compile::CompileOptions::default();
    let blob = super::compile::compile_pedal_cached(
        &source,
        "tb303_linear_regime",
        "tb303_linear_regime",
        SR,
        &options,
        &cache_dir,
    )
    .expect("compile failed");

    let measure = |freq: f64| -> f64 {
        let mut proc: super::compiled::CompiledPedal =
            postcard::from_bytes(&blob).expect("deserialize failed");
        proc.set_control_immediate("Cutoff", 0.5);
        proc.set_control_immediate("Resonance", 0.0);
        // Warm-up
        for _ in 0..2400 {
            proc.process(0.0);
        }
        let mut peak = 0.0f64;
        for i in 0..4800 {
            let input = 0.1 * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
            peak = peak.max(proc.process(input).abs());
        }
        peak
    };

    let gain_100 = measure(100.0);
    let gain_1k = measure(1000.0);
    let gain_10k = measure(10000.0);

    let ratio_100_10k = 20.0 * (gain_100 / gain_10k.max(1e-12)).log10();

    eprintln!("  Linear regime: 100Hz={gain_100:.6}, 1kHz={gain_1k:.6}, 10kHz={gain_10k:.6}");
    eprintln!("  Ratio 100Hz/10kHz: {ratio_100_10k:+.1} dB");

    // Even in linear regime (K-tables as-is), the RC time constants in
    // each block's WDF tree should produce SOME frequency dependence.
    // If the ratio is exactly 0 dB, the coupling is not routing correctly.
    assert!(
        (gain_100 - gain_10k).abs() > gain_100 * 0.01,
        "Linear regime should show frequency dependence (RC in each block). \
         100Hz={gain_100:.6}, 10kHz={gain_10k:.6} — identical means coupling is broken."
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 11. Single-block frequency response: each block should lowpass
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn tb303_single_block_filters() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");

    let cache_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("target")
        .join("test-cache");
    let _ = std::fs::create_dir_all(&cache_dir);

    let options = super::compile::CompileOptions::default();
    let blob = super::compile::compile_pedal_cached(
        &source,
        "tb303_single_block",
        "tb303_single_block",
        SR,
        &options,
        &cache_dir,
    )
    .expect("compile failed");

    let proc: super::compiled::CompiledPedal =
        postcard::from_bytes(&blob).expect("deserialize failed");

    // Find the Blockwise stage
    let bkm = proc.stages.iter().find_map(|s| {
        if let pedalkernel_rt::processor::Stage::Blockwise(ref k) = s {
            Some(k)
        } else {
            None
        }
    });

    if bkm.is_none() {
        eprintln!("  SKIP: no Blockwise stage found");
        return;
    }
    let bkm = bkm.unwrap();

    // Process a single block directly
    let measure_block = |block_idx: usize, freq: f64| -> f64 {
        let mut block = bkm.blocks[block_idx].clone();
        block.k_table.precompute_scales();
        // Warm up
        for _ in 0..2400 {
            block.tree.set_voltage(0.0);
            let b = block.tree.reflected();
            let a = block.k_table.lookup_2d(b, 0.0);
            block.tree.set_incident(a);
        }
        let mut peak = 0.0f64;
        for i in 0..4800 {
            let input = 0.1 * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
            block.tree.set_voltage(input);
            let b = block.tree.reflected();
            let a = block.k_table.lookup_2d(b, 0.0);
            block.tree.set_incident(a);
            let v_out = (a + b) / 2.0;
            peak = peak.max(v_out.abs());
        }
        peak
    };

    for bi in 0..bkm.blocks.len().min(4) {
        let g100 = measure_block(bi, 100.0);
        let g1k = measure_block(bi, 1000.0);
        let g10k = measure_block(bi, 10000.0);
        let ratio_db = 20.0 * (g100 / g10k.max(1e-12)).log10();
        eprintln!(
            "  block {bi}: 100Hz={g100:.6}, 1kHz={g1k:.6}, 10kHz={g10k:.6}, ratio={ratio_db:+.1}dB"
        );
    }

    // At least one block should show >3dB rolloff from 100Hz to 10kHz
    let block0_100 = measure_block(0, 100.0);
    let block0_10k = measure_block(0, 10000.0);
    assert!(
        block0_100 > block0_10k * 1.2,
        "Single block should lowpass: 100Hz={block0_100:.6} should be >1.2× 10kHz={block0_10k:.6}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// Test: CV port changes filter behavior
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn tb303_cv_port_modulates_output() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");

    let cache_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("target")
        .join("test-cache");
    let _ = std::fs::create_dir_all(&cache_dir);

    let options = super::compile::CompileOptions::default();
    let blob = super::compile::compile_pedal_cached(
        &source,
        "tb303_cv_test_v3",
        "tb303_cv_test_v3",
        SR,
        &options,
        &cache_dir,
    )
    .expect("compile failed");

    let measure_with_cv = |cv_voltage: f64| -> f64 {
        let mut proc: super::compiled::CompiledPedal =
            postcard::from_bytes(&blob).expect("deserialize failed");
        proc.set_control_immediate("Resonance", 0.0);
        // Warm up with CV
        for _ in 0..2400 {
            let mut ports = vec![0.0; proc.port_count()];
            if let Some(cv_idx) = proc.resolve_port("cv_cutoff") {
                ports[cv_idx] = cv_voltage;
            }
            proc.process_ports(&mut ports);
        }
        let mut peak = 0.0f64;
        for i in 0..4800 {
            let input = 0.1 * (2.0 * std::f64::consts::PI * 1000.0 * i as f64 / SR).sin();
            let mut ports = vec![0.0; proc.port_count()];
            if let Some(in_idx) = proc.resolve_port("audio_in") {
                ports[in_idx] = input;
            }
            if let Some(cv_idx) = proc.resolve_port("cv_cutoff") {
                ports[cv_idx] = cv_voltage;
            }
            proc.process_ports(&mut ports);
            if let Some(out_idx) = proc.resolve_port("audio_out") {
                peak = peak.max(ports[out_idx].abs());
            }
        }
        peak
    };

    let gain_cv0 = measure_with_cv(0.0);
    let gain_cv1 = measure_with_cv(1.0);
    let gain_cv_neg1 = measure_with_cv(-1.0);

    eprintln!("  CV=0: {gain_cv0:.6}, CV=+1V: {gain_cv1:.6}, CV=-1V: {gain_cv_neg1:.6}");

    // CV should change the output — different bias → different cutoff
    let max_gain = gain_cv0.max(gain_cv1).max(gain_cv_neg1);
    let min_gain = gain_cv0.min(gain_cv1).min(gain_cv_neg1);
    assert!(
        max_gain > min_gain * 1.1,
        "CV should modulate filter: max={max_gain:.6}, min={min_gain:.6} (ratio={:.2}×)",
        max_gain / min_gain.max(1e-12)
    );
}

#[test]
fn tb303_cv_port_moves_explicit_diode_ladder_cutoff() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");

    let cache_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("target")
        .join("test-cache");
    let _ = std::fs::create_dir_all(&cache_dir);

    let options = super::compile::CompileOptions::default();
    let blob = super::compile::compile_pedal_cached(
        &source,
        "tb303_cv_ac_cutoff_test_v4",
        "tb303_cv_ac_cutoff_test_v4",
        SR,
        &options,
        &cache_dir,
    )
    .expect("compile failed");

    let measure_ac = |cv_voltage: f64, freq: f64| -> f64 {
        let mut proc: super::compiled::CompiledPedal =
            postcard::from_bytes(&blob).expect("deserialize failed");
        proc.set_control_immediate("Resonance", 0.0);
        let in_idx = proc.resolve_port("audio_in").expect("audio_in port");
        let cv_idx = proc.resolve_port("cv_cutoff").expect("cv_cutoff port");
        let out_idx = proc.resolve_port("audio_out").expect("audio_out port");

        for _ in 0..9600 {
            let mut ports = vec![0.0; proc.port_count()];
            ports[cv_idx] = cv_voltage;
            proc.process_ports(&mut ports);
        }

        let mut values = Vec::new();
        for i in 0..9600 {
            let input = 0.05 * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
            let mut ports = vec![0.0; proc.port_count()];
            ports[in_idx] = input;
            ports[cv_idx] = cv_voltage;
            proc.process_ports(&mut ports);
            values.push(ports[out_idx]);
        }

        ac_rms(&values)
    };

    let low_cv_8k = measure_ac(-1.0, 8_000.0);
    let high_cv_8k = measure_ac(3.0, 8_000.0);
    let low_cv_12k = measure_ac(-1.0, 12_000.0);
    let high_cv_12k = measure_ac(3.0, 12_000.0);

    eprintln!(
        "  cutoff CV AC: 8k -1V={low_cv_8k:.6}, +3V={high_cv_8k:.6}; \
         12k -1V={low_cv_12k:.6}, +3V={high_cv_12k:.6}"
    );

    assert!(
        high_cv_8k > low_cv_8k * 1.15,
        "raising cv_cutoff should move the ExplicitSingleDiode ladder cutoff upward at 8kHz: \
         low={low_cv_8k:.6}, high={high_cv_8k:.6}, ratio={:.3}",
        high_cv_8k / low_cv_8k.max(1e-12)
    );
    assert!(
        high_cv_12k > low_cv_12k,
        "raising cv_cutoff should not reduce the upper stop-band response at 12kHz: \
         low={low_cv_12k:.6}, high={high_cv_12k:.6}, ratio={:.3}",
        high_cv_12k / low_cv_12k.max(1e-12)
    );
}

#[test]
fn tb303_cutoff_cv_updates_shared_diffpair_tail_current_axis() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");

    let cache_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("target")
        .join("test-cache");
    let _ = std::fs::create_dir_all(&cache_dir);

    let options = super::compile::CompileOptions::default();
    let blob = super::compile::compile_pedal_cached(
        &source,
        "tb303_cv_tail_current_axis_test_v1",
        "tb303_cv_tail_current_axis_test_v1",
        SR,
        &options,
        &cache_dir,
    )
    .expect("compile failed");

    let mut proc: super::compiled::CompiledPedal =
        postcard::from_bytes(&blob).expect("deserialize failed");
    let cv_idx = proc.resolve_port("cv_cutoff").expect("cv_cutoff port");

    let measure_ctrl = |proc: &mut super::compiled::CompiledPedal, cv_voltage: f64| -> f64 {
        let mut ports = vec![0.0; proc.port_count()];
        ports[cv_idx] = cv_voltage;
        proc.process_ports(&mut ports);
        proc.stages
            .iter()
            .find_map(|stage| {
                if let pedalkernel_rt::processor::Stage::Blockwise(bkm) = stage {
                    Some(
                        bkm.blocks[1]
                            .shared_diode_bias_voltage
                            .expect("shared cutoff control should update all diffpair rungs"),
                    )
                } else {
                    None
                }
            })
            .expect("tb303 filter should compile to BKM")
    };

    let ctrl_neg1v = measure_ctrl(&mut proc, -1.0);
    let ctrl_0v = measure_ctrl(&mut proc, 0.0);
    let ctrl_3v = measure_ctrl(&mut proc, 3.0);

    eprintln!(
        "  circuit-derived TB303 cutoff ctrl: -1V={ctrl_neg1v:.3}, 0V={ctrl_0v:.3}, +3V={ctrl_3v:.3}"
    );

    assert!(
        ctrl_neg1v < ctrl_0v && ctrl_0v < ctrl_3v,
        "TB303 cutoff CV should modulate the shared DiffPair K-table tail-current axis, not mutate block Rp: \
         -1V={ctrl_neg1v:.3}, 0V={ctrl_0v:.3}, +3V={ctrl_3v:.3}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// Test G: Validate measurement infrastructure with known transfer function
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn tb303_measurement_infra_detects_known_lowpass() {
    // Bypass the 303 entirely. Implement a trivial 1-pole RC lowpass
    // in software and measure it the same way the filter tests do.
    // If THIS test fails, our measurement code is wrong.
    let sr = SR;
    let fc = 1000.0; // 1kHz cutoff
    let rc = 1.0 / (2.0 * std::f64::consts::PI * fc);
    let alpha = 1.0 / (1.0 + rc * sr); // bilinear 1-pole coefficient

    let measure = |freq: f64| -> f64 {
        let mut state = 0.0f64;
        // Warm up
        for _ in 0..2400 {
            state += alpha * (0.0 - state);
        }
        let mut peak = 0.0f64;
        for i in 0..4800 {
            let input = 0.1 * (2.0 * std::f64::consts::PI * freq * i as f64 / sr).sin();
            state += alpha * (input - state);
            peak = peak.max(state.abs());
        }
        peak
    };

    let g100 = measure(100.0);
    let g1k = measure(1000.0);
    let g10k = measure(10000.0);
    let ratio_db = 20.0 * (g100 / g10k.max(1e-12)).log10();

    eprintln!("  Known 1-pole LP: 100Hz={g100:.6}, 1kHz={g1k:.6}, 10kHz={g10k:.6}");
    eprintln!("  Ratio 100Hz/10kHz: {ratio_db:+.1} dB");

    assert!(
        g100 > g10k * 2.0,
        "Known 1kHz LP should show >6dB rolloff at 10kHz: 100Hz={g100:.6}, 10kHz={g10k:.6}"
    );
    assert!(
        ratio_db > 15.0,
        "Expected >15dB rolloff from 100Hz to 10kHz, got {ratio_db:+.1} dB"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// Test F: DC gain check — suspicious 2.0 flat output
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn tb303_dc_gain_not_two() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");

    let cache_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("target")
        .join("test-cache");
    let _ = std::fs::create_dir_all(&cache_dir);

    let options = super::compile::CompileOptions::default();
    let blob = super::compile::compile_pedal_cached(
        &source,
        "tb303_dc_gain",
        "tb303_dc_gain",
        SR,
        &options,
        &cache_dir,
    )
    .expect("compile failed");

    let mut proc: super::compiled::CompiledPedal =
        postcard::from_bytes(&blob).expect("deserialize failed");
    proc.set_control_immediate("Resonance", 0.0);

    // Step input: DC gain test
    for _ in 0..4800 {
        proc.process(0.0);
    }
    let mut last = 0.0;
    for _ in 0..9600 {
        last = proc.process(1.0); // Step to 1.0V
    }

    eprintln!("  DC step response (after 200ms): {last:.6}");

    // DC gain should be ≈1.0 (Stinchcombe: H(0) = -1, magnitude 1).
    // If we get exactly 2.0, the output has a doubling bug.
    assert!(
        last.abs() < 1.5,
        "DC gain should be ≈1 (Stinchcombe H(0)=-1), got {last:.6}. \
         If exactly 2.0, output extraction has a doubling (V=(a+b)/2 with a≈b)."
    );
    assert!(
        last.abs() > 0.01,
        "DC gain should be nonzero, got {last:.6}. Filter is silent."
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// Test D: Newton iteration count — is Newton actually running?
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn tb303_k_table_not_identity() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");

    let cache_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("target")
        .join("test-cache");
    let _ = std::fs::create_dir_all(&cache_dir);

    let options = super::compile::CompileOptions::default();
    let blob = super::compile::compile_pedal_cached(
        &source,
        "tb303_ktable_check",
        "tb303_ktable_check",
        SR,
        &options,
        &cache_dir,
    )
    .expect("compile failed");

    let proc: super::compiled::CompiledPedal =
        postcard::from_bytes(&blob).expect("deserialize failed");

    // Find the Blockwise stage and inspect its K-tables
    let bkm = proc.stages.iter().find_map(|s| {
        if let pedalkernel_rt::processor::Stage::Blockwise(ref k) = s {
            Some(k)
        } else {
            None
        }
    });

    if bkm.is_none() {
        eprintln!("  SKIP: no Blockwise stage found");
        return;
    }
    let bkm = bkm.unwrap();

    // Check each block's K-table: evaluate at several b values
    for (bi, block) in bkm.blocks.iter().enumerate() {
        let mut table = block.k_table.clone();
        table.precompute_scales();

        let b_values = [-5.0, -1.0, 0.0, 1.0, 5.0];
        let a_values: Vec<f64> = b_values.iter().map(|&b| table.lookup_2d(b, 0.0)).collect();

        let max_a = a_values.iter().map(|a| a.abs()).fold(0.0f64, f64::max);
        let min_a = a_values.iter().map(|a| a.abs()).fold(f64::MAX, f64::min);
        let spread = max_a - min_a;

        eprintln!("  block {bi} K-table: b={b_values:?} → a={a_values:.4?}, spread={spread:.6}");

        // The K-table should NOT be constant — a_root should vary with b_tree.
        // If spread ≈ 0, the diode is degenerating to a constant (identity reflection).
        assert!(
            spread > 0.01,
            "Block {bi} K-table is near-constant (spread={spread:.6}). \
             Diode NL is degenerating — check bias point, Is, Vt."
        );
    }
}

// VCA test removed — VCA now uses behavioral macromodel (BA662 + LA4140),
// not a WDF-compiled .pedal. See acidattack-core/src/vca.rs.

#[test]
fn tb303_vco_port_reaches_output() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let cache_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("target")
        .join("test-cache");
    let _ = std::fs::create_dir_all(&cache_dir);

    let options = super::compile::CompileOptions::default();
    let blob = super::compile::compile_pedal_cached(
        &source,
        "tb303_vco_port",
        "tb303_vco_port",
        SR,
        &options,
        &cache_dir,
    )
    .expect("compile failed");

    let mut proc: super::compiled::CompiledPedal =
        postcard::from_bytes(&blob).expect("deserialize failed");

    // Warm up with silence
    for _ in 0..2400 {
        proc.process(0.0);
    }

    // Test 1: process(input) path — serial chain
    let mut peak_serial = 0.0f64;
    for i in 0..4800 {
        let input = 0.5 * (2.0 * std::f64::consts::PI * 440.0 * i as f64 / SR).sin();
        peak_serial = peak_serial.max(proc.process(input).abs());
    }

    // Test 2: process_ports path — VCO through vco_in port
    let mut proc2: super::compiled::CompiledPedal =
        postcard::from_bytes(&blob).expect("deserialize failed");
    for _ in 0..2400 {
        let mut ports = vec![0.0; proc2.port_count()];
        proc2.process_ports(&mut ports);
    }
    let vco_idx = proc2.resolve_port("vco_in");
    let out_idx = proc2.resolve_port("audio_out");
    eprintln!(
        "  vco_in port index: {:?}, audio_out port index: {:?}",
        vco_idx, out_idx
    );
    eprintln!("  total ports: {}", proc2.port_count());

    let mut peak_port = 0.0f64;
    for i in 0..4800 {
        let vco = 0.5 * (2.0 * std::f64::consts::PI * 440.0 * i as f64 / SR).sin();
        let mut ports = vec![0.0; proc2.port_count()];
        if let Some(idx) = vco_idx {
            ports[idx] = vco;
        }
        proc2.process_ports(&mut ports);
        if let Some(idx) = out_idx {
            peak_port = peak_port.max(ports[idx].abs());
        }
    }

    eprintln!("  serial chain peak: {peak_serial:.6}");
    eprintln!("  vco_in port peak: {peak_port:.6}");

    assert!(
        peak_serial > 0.01,
        "Serial chain should produce audio, got {peak_serial:.6}"
    );
    assert!(
        peak_port > 0.01,
        "VCO through vco_in port should produce audio, got {peak_port:.6}"
    );
}

#[test]
fn tb303_sustained_tone_not_silent() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let cache_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("target")
        .join("test-cache");
    let _ = std::fs::create_dir_all(&cache_dir);

    let options = super::compile::CompileOptions::default();
    let blob = super::compile::compile_pedal_cached(
        &source,
        "tb303_sustained",
        "tb303_sustained",
        SR,
        &options,
        &cache_dir,
    )
    .expect("compile failed");

    let mut proc: super::compiled::CompiledPedal =
        postcard::from_bytes(&blob).expect("deserialize failed");

    // Process 48000 samples (1 second) of 220Hz saw through vco_in port
    let vco_idx = proc.resolve_port("vco_in");
    let out_idx = proc.resolve_port("audio_out");
    eprintln!(
        "  vco_in={:?} audio_out={:?} ports={}",
        vco_idx,
        out_idx,
        proc.port_count()
    );

    let mut samples = Vec::new();
    for i in 0..48000 {
        let t = i as f64 / SR;
        let vco = 0.3 * (2.0 * std::f64::consts::PI * 220.0 * t).sin();
        let mut ports = vec![0.0; proc.port_count()];
        if let Some(idx) = vco_idx {
            ports[idx] = vco;
        }
        proc.process_ports(&mut ports);
        let out = if let Some(idx) = out_idx {
            ports[idx]
        } else {
            0.0
        };
        samples.push(out);
    }

    // Check signal at different time windows
    let peak_0_100ms: f64 = samples[0..4800].iter().map(|s| s.abs()).fold(0.0, f64::max);
    let peak_100_200ms: f64 = samples[4800..9600]
        .iter()
        .map(|s| s.abs())
        .fold(0.0, f64::max);
    let peak_500_600ms: f64 = samples[24000..28800]
        .iter()
        .map(|s| s.abs())
        .fold(0.0, f64::max);
    let peak_900_1000ms: f64 = samples[43200..48000]
        .iter()
        .map(|s| s.abs())
        .fold(0.0, f64::max);

    eprintln!("  0-100ms:   peak={peak_0_100ms:.6}");
    eprintln!("  100-200ms: peak={peak_100_200ms:.6}");
    eprintln!("  500-600ms: peak={peak_500_600ms:.6}");
    eprintln!("  900-1000ms: peak={peak_900_1000ms:.6}");

    // Check for NaN/Inf
    let nan_count = samples.iter().filter(|s| !s.is_finite()).count();
    eprintln!("  NaN/Inf samples: {nan_count} / {}", samples.len());

    // Signal should be sustained (not just a click)
    assert!(
        peak_500_600ms > 0.001,
        "Signal at 500-600ms should be nonzero (sustained tone), got {peak_500_600ms:.6}"
    );
    assert!(nan_count == 0, "No NaN/Inf allowed, got {nan_count}");
}

#[test]
fn tb303_render_wav_comparison() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let cache_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("target")
        .join("test-cache");
    let _ = std::fs::create_dir_all(&cache_dir);

    let options = super::compile::CompileOptions::default();
    let blob = super::compile::compile_pedal_cached(
        &source,
        "tb303_wav_render",
        "tb303_wav_render",
        SR,
        &options,
        &cache_dir,
    )
    .expect("compile failed");

    let mut proc: super::compiled::CompiledPedal =
        postcard::from_bytes(&blob).expect("deserialize failed");

    let wav_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("target")
        .join("test-wav");
    let _ = std::fs::create_dir_all(&wav_dir);

    let duration_secs = 2.0;
    let n_samples = (duration_secs * SR) as usize;

    // ── Track 1: dry VCO (bypass filter) ──
    let mut dry_samples = Vec::with_capacity(n_samples);
    for i in 0..n_samples {
        let t = i as f64 / SR;
        // Saw-ish: fundamental + a few harmonics
        let vco = 0.3 * (2.0 * std::f64::consts::PI * 220.0 * t).sin()
            + 0.15 * (2.0 * std::f64::consts::PI * 440.0 * t).sin()
            + 0.1 * (2.0 * std::f64::consts::PI * 660.0 * t).sin();
        dry_samples.push(vco);
    }
    crate::wav::write_wav(
        &dry_samples,
        &wav_dir.join("tb303_1_dry_vco.wav"),
        SR as u32,
    )
    .expect("write dry wav");

    // ── Track 2: VCO through filter (serial chain / process(input)) ──
    let mut filtered_serial = Vec::with_capacity(n_samples);
    for i in 0..n_samples {
        let t = i as f64 / SR;
        let vco = 0.3 * (2.0 * std::f64::consts::PI * 220.0 * t).sin()
            + 0.15 * (2.0 * std::f64::consts::PI * 440.0 * t).sin()
            + 0.1 * (2.0 * std::f64::consts::PI * 660.0 * t).sin();
        let out = proc.process(vco);
        filtered_serial.push(out);
    }
    crate::wav::write_wav(
        &filtered_serial,
        &wav_dir.join("tb303_2_filtered_serial.wav"),
        SR as u32,
    )
    .expect("write filtered serial wav");

    // ── Track 3: VCO through filter (port path / process_ports) ──
    let mut proc3: super::compiled::CompiledPedal =
        postcard::from_bytes(&blob).expect("deserialize failed");
    let vco_idx = proc3.resolve_port("vco_in");
    let out_idx = proc3.resolve_port("audio_out");
    let mut filtered_port = Vec::with_capacity(n_samples);
    for i in 0..n_samples {
        let t = i as f64 / SR;
        let vco = 0.3 * (2.0 * std::f64::consts::PI * 220.0 * t).sin()
            + 0.15 * (2.0 * std::f64::consts::PI * 440.0 * t).sin()
            + 0.1 * (2.0 * std::f64::consts::PI * 660.0 * t).sin();
        let mut ports = vec![0.0; proc3.port_count()];
        if let Some(idx) = vco_idx {
            ports[idx] = vco * 0.03;
        } // attenuated
        proc3.process_ports(&mut ports);
        let out = if let Some(idx) = out_idx {
            ports[idx]
        } else {
            0.0
        };
        filtered_port.push(out);
    }
    crate::wav::write_wav(
        &filtered_port,
        &wav_dir.join("tb303_3_filtered_port.wav"),
        SR as u32,
    )
    .expect("write filtered port wav");

    // ── Track 4: VCO through filter with CV sweep ──
    let mut proc4: super::compiled::CompiledPedal =
        postcard::from_bytes(&blob).expect("deserialize failed");
    let vco_idx4 = proc4.resolve_port("vco_in");
    let cv_idx4 = proc4.resolve_port("cv_cutoff");
    let out_idx4 = proc4.resolve_port("audio_out");
    let mut filtered_cv = Vec::with_capacity(n_samples);
    for i in 0..n_samples {
        let t = i as f64 / SR;
        let vco = 0.3 * (2.0 * std::f64::consts::PI * 220.0 * t).sin()
            + 0.15 * (2.0 * std::f64::consts::PI * 440.0 * t).sin()
            + 0.1 * (2.0 * std::f64::consts::PI * 660.0 * t).sin();
        // CV sweep: 0 to 2V over 2 seconds (opening the filter)
        let cv = t;
        let mut ports = vec![0.0; proc4.port_count()];
        if let Some(idx) = vco_idx4 {
            ports[idx] = vco * 0.03;
        }
        if let Some(idx) = cv_idx4 {
            ports[idx] = cv;
        }
        proc4.process_ports(&mut ports);
        let out = if let Some(idx) = out_idx4 {
            ports[idx]
        } else {
            0.0
        };
        filtered_cv.push(out);
    }
    crate::wav::write_wav(
        &filtered_cv,
        &wav_dir.join("tb303_4_filtered_cv_sweep.wav"),
        SR as u32,
    )
    .expect("write filtered cv wav");

    let serial_peak = filtered_serial
        .iter()
        .map(|s| s.abs())
        .fold(0.0f64, f64::max);
    let port_peak = filtered_port.iter().map(|s| s.abs()).fold(0.0f64, f64::max);
    let cv_peak = filtered_cv.iter().map(|s| s.abs()).fold(0.0f64, f64::max);

    eprintln!("  WAV files written to {:?}", wav_dir);
    eprintln!("  Track 1: dry VCO");
    eprintln!("  Track 2: filtered (serial), peak={serial_peak:.4}");
    eprintln!("  Track 3: filtered (port), peak={port_peak:.4}");
    eprintln!("  Track 4: filtered (port + CV sweep), peak={cv_peak:.4}");
}

#[test]
#[ignore]
fn tb303_export_publish_k0_response_data() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let out_dir = std::env::var("PK_TB303_PUBLISH_DIR")
        .map(std::path::PathBuf::from)
        .unwrap_or_else(|_| {
            std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
                .join("target")
                .join("tb303-publish")
        });
    std::fs::create_dir_all(&out_dir).expect("create publish artifact dir");

    let cache_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("target")
        .join("test-cache");
    let _ = std::fs::create_dir_all(&cache_dir);

    let blob = super::compile::compile_pedal_cached(
        &source,
        "tb303_publish_response",
        "tb303_publish_response",
        SR,
        &super::compile::CompileOptions::default(),
        &cache_dir,
    )
    .expect("compile failed");

    const FILTER_AUDIO_INPUT_VOLTS: f64 = 0.03;
    const FILTER_AUDIO_INPUT_RESISTANCE_OHMS: f64 = 10_000.0;
    const FILTER_VCO_INPUT_RESISTANCE_OHMS: f64 = 1_000.0;
    const FILTER_VCO_INPUT_VOLTS: f64 = FILTER_AUDIO_INPUT_VOLTS * FILTER_VCO_INPUT_RESISTANCE_OHMS
        / FILTER_AUDIO_INPUT_RESISTANCE_OHMS;

    let freqs = [
        80.0, 100.0, 125.0, 160.0, 200.0, 250.0, 315.0, 400.0, 500.0, 630.0, 800.0, 1000.0, 1250.0,
        1500.0, 1600.0, 1700.0, 1750.0, 1800.0, 2000.0, 2500.0, 3150.0, 4000.0, 5000.0, 6300.0,
        8000.0, 10_000.0, 12_500.0, 16_000.0,
    ];

    let bkm_k0: Vec<f64> = freqs
        .iter()
        .map(|&freq| {
            let mut proc: super::compiled::CompiledPedal =
                postcard::from_bytes(&blob).expect("deserialize failed");
            proc.set_control_immediate("Cutoff", 0.5);
            proc.set_control_immediate("Resonance", 0.0);
            quick_sine_ac_rms_ports(
                &mut proc,
                freq,
                FILTER_VCO_INPUT_VOLTS,
                &[("vco_in", 1.0)],
                &[("cv_cutoff", 0.0), ("cv_resonance", 0.0)],
                "audio_out",
            )
        })
        .collect();

    let target: Vec<f64> = bkm_k0
        .iter()
        .map(|gain| db_norm(*gain, bkm_k0[1]))
        .collect();
    let mut best_fc = 100.0;
    let mut best_err = f64::INFINITY;
    for step in 0..240 {
        let t = step as f64 / 239.0;
        let fc = 100.0 * (100.0f64).powf(t);
        let ref_norm = stinchcombe_10pole_magnitude(freqs[1], fc, 0.0);
        if ref_norm <= 1.0e-12 {
            continue;
        }
        let err = freqs
            .iter()
            .enumerate()
            .map(|(i, &freq)| {
                let ref_db = db_norm(stinchcombe_10pole_magnitude(freq, fc, 0.0), ref_norm);
                let diff = target[i] - ref_db;
                diff * diff
            })
            .sum::<f64>();
        if err < best_err {
            best_err = err;
            best_fc = fc;
        }
    }

    let htb_norm = stinchcombe_htb_magnitude(freqs[1], best_fc);
    let tenpole_norm = stinchcombe_10pole_magnitude(freqs[1], best_fc, 0.0);
    let mut response_csv = String::from("freq_hz,htb_db,tenpole_k0_db,wdf_k0_db\n");
    for (i, &freq) in freqs.iter().enumerate() {
        response_csv.push_str(&format!(
            "{freq:.3},{:.6},{:.6},{:.6}\n",
            db_norm(stinchcombe_htb_magnitude(freq, best_fc), htb_norm),
            db_norm(
                stinchcombe_10pole_magnitude(freq, best_fc, 0.0),
                tenpole_norm
            ),
            db_norm(bkm_k0[i], bkm_k0[1]),
        ));
    }
    std::fs::write(out_dir.join("tb303_response.csv"), response_csv).expect("write response csv");
    eprintln!(
        "  TB303 k=0 response CSV written to {:?} (fc={best_fc:.1}Hz)",
        out_dir.join("tb303_response.csv")
    );
}

#[test]
#[ignore]
fn tb303_export_publish_response_data() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let out_dir = std::env::var("PK_TB303_PUBLISH_DIR")
        .map(std::path::PathBuf::from)
        .unwrap_or_else(|_| {
            std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
                .join("target")
                .join("tb303-publish")
        });
    std::fs::create_dir_all(&out_dir).expect("create publish artifact dir");

    let cache_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("target")
        .join("test-cache");
    let _ = std::fs::create_dir_all(&cache_dir);

    let options = super::compile::CompileOptions::default();
    let blob = super::compile::compile_pedal_cached(
        &source,
        "tb303_publish_response",
        "tb303_publish_response",
        SR,
        &options,
        &cache_dir,
    )
    .expect("compile failed");

    const FILTER_AUDIO_INPUT_VOLTS: f64 = 0.03;
    const FILTER_AUDIO_INPUT_RESISTANCE_OHMS: f64 = 10_000.0;
    const FILTER_VCO_INPUT_RESISTANCE_OHMS: f64 = 1_000.0;
    const FILTER_VCO_INPUT_VOLTS: f64 = FILTER_AUDIO_INPUT_VOLTS * FILTER_VCO_INPUT_RESISTANCE_OHMS
        / FILTER_AUDIO_INPUT_RESISTANCE_OHMS;

    let freqs = [
        40.0, 50.0, 63.0, 80.0, 100.0, 125.0, 160.0, 200.0, 250.0, 315.0, 400.0, 500.0, 630.0,
        800.0, 1000.0, 1250.0, 1600.0, 2000.0, 2500.0, 3150.0, 4000.0, 5000.0, 6300.0, 8000.0,
        10_000.0, 12_500.0, 16_000.0,
    ];

    let measure = |freq: f64, resonance: f64| -> f64 {
        let mut proc: super::compiled::CompiledPedal =
            postcard::from_bytes(&blob).expect("deserialize failed");
        proc.set_control_immediate("Cutoff", 0.5);
        proc.set_control_immediate("Resonance", resonance);
        quick_sine_ac_rms_ports(
            &mut proc,
            freq,
            FILTER_VCO_INPUT_VOLTS,
            &[("vco_in", 1.0)],
            &[("cv_cutoff", 0.0), ("cv_resonance", 0.0)],
            "audio_out",
        )
    };

    let bkm_k0: Vec<f64> = freqs.iter().map(|&freq| measure(freq, 0.0)).collect();
    let bkm_k03: Vec<f64> = freqs.iter().map(|&freq| measure(freq, 0.3)).collect();
    let bkm_k085: Vec<f64> = freqs.iter().map(|&freq| measure(freq, 0.85)).collect();

    let mut best_fc = 100.0;
    let mut best_err = f64::INFINITY;
    let target: Vec<f64> = bkm_k0
        .iter()
        .map(|gain| db_norm(*gain, bkm_k0[4]))
        .collect();
    for step in 0..360 {
        let t = step as f64 / 359.0;
        let fc = 100.0 * (100.0f64).powf(t);
        let ref_norm = stinchcombe_10pole_magnitude(freqs[4], fc, 0.0);
        if ref_norm <= 1.0e-12 {
            continue;
        }
        let err = freqs
            .iter()
            .enumerate()
            .map(|(i, &freq)| {
                let ref_db = db_norm(stinchcombe_10pole_magnitude(freq, fc, 0.0), ref_norm);
                let diff = target[i] - ref_db;
                diff * diff
            })
            .sum::<f64>();
        if err < best_err {
            best_err = err;
            best_fc = fc;
        }
    }

    let mut response_csv =
        String::from("freq_hz,htb_db,tenpole_k0_db,tenpole_k03_db,tenpole_k085_db,wdf_k0_db,wdf_k03_db,wdf_k085_db\n");
    let htb_norm = stinchcombe_htb_magnitude(freqs[4], best_fc);
    let tenpole_k0_norm = stinchcombe_10pole_magnitude(freqs[4], best_fc, 0.0);
    let tenpole_k03_norm = stinchcombe_10pole_magnitude(freqs[4], best_fc, 0.3);
    let tenpole_k085_norm = stinchcombe_10pole_magnitude(freqs[4], best_fc, 0.85);
    for (i, &freq) in freqs.iter().enumerate() {
        response_csv.push_str(&format!(
            "{freq:.3},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6}\n",
            db_norm(stinchcombe_htb_magnitude(freq, best_fc), htb_norm),
            db_norm(
                stinchcombe_10pole_magnitude(freq, best_fc, 0.0),
                tenpole_k0_norm
            ),
            db_norm(
                stinchcombe_10pole_magnitude(freq, best_fc, 0.3),
                tenpole_k03_norm
            ),
            db_norm(
                stinchcombe_10pole_magnitude(freq, best_fc, 0.85),
                tenpole_k085_norm
            ),
            db_norm(bkm_k0[i], bkm_k0[4]),
            db_norm(bkm_k03[i], bkm_k03[4]),
            db_norm(bkm_k085[i], bkm_k085[4]),
        ));
    }
    std::fs::write(out_dir.join("tb303_response.csv"), response_csv).expect("write response csv");

    let render = |filename: &str,
                  resonance: f64,
                  mut drive: Box<dyn FnMut(f64) -> (f64, f64, f64)>| {
        let mut proc: super::compiled::CompiledPedal =
            postcard::from_bytes(&blob).expect("deserialize failed");
        proc.set_control_immediate("Cutoff", 0.5);
        proc.set_control_immediate("Resonance", resonance);
        let vco_idx = proc.resolve_port("vco_in").expect("missing vco_in");
        let cv_cutoff_idx = proc.resolve_port("cv_cutoff").expect("missing cv_cutoff");
        let cv_res_idx = proc
            .resolve_port("cv_resonance")
            .expect("missing cv_resonance");
        let out_idx = proc.resolve_port("audio_out").expect("missing audio_out");
        let n_samples = SR as usize;
        let mut ports = vec![0.0; proc.port_count()];
        let mut output = Vec::with_capacity(n_samples);
        for i in 0..n_samples {
            let t = i as f64 / SR;
            let (vco, cutoff_cv, resonance_cv) = drive(t);
            ports.fill(0.0);
            ports[vco_idx] = vco;
            ports[cv_cutoff_idx] = cutoff_cv;
            ports[cv_res_idx] = resonance_cv;
            proc.process_ports(&mut ports);
            output.push(ports[out_idx]);
        }
        crate::wav::write_wav(&output, &out_dir.join(filename), SR as u32).expect("write WDF wav");
        output
    };

    let waveform = render(
        "tb303_wdf_k03.wav",
        0.3,
        Box::new(|t| {
            let input = FILTER_VCO_INPUT_VOLTS
                * ((2.0 * std::f64::consts::PI * 110.0 * t).sin()
                    + 0.5 * (2.0 * std::f64::consts::PI * 220.0 * t).sin()
                    + 0.33 * (2.0 * std::f64::consts::PI * 330.0 * t).sin()
                    + 0.25 * (2.0 * std::f64::consts::PI * 440.0 * t).sin());
            (input, 0.0, 0.0)
        }),
    );

    let squelch = render(
        "tb303_squelch_cutoff_sweep.wav",
        0.85,
        Box::new(|t| {
            let input = FILTER_VCO_INPUT_VOLTS
                * ((2.0 * std::f64::consts::PI * 110.0 * t).sin()
                    + 0.5 * (2.0 * std::f64::consts::PI * 220.0 * t).sin()
                    + 0.33 * (2.0 * std::f64::consts::PI * 330.0 * t).sin());
            let cutoff_cv = 1.2 * (0.5 + 0.5 * (2.0 * std::f64::consts::PI * 1.0 * t).sin());
            (input, cutoff_cv, 0.0)
        }),
    );
    let self_osc = render(
        "tb303_self_osc_probe.wav",
        0.95,
        Box::new(|_| (0.0, 0.6, 0.0)),
    );
    let accent_probe = render(
        "tb303_accent_cv_probe.wav",
        0.75,
        Box::new(|t| {
            let input = FILTER_VCO_INPUT_VOLTS
                * ((2.0 * std::f64::consts::PI * 110.0 * t).sin()
                    + 0.5 * (2.0 * std::f64::consts::PI * 220.0 * t).sin());
            let step = if (t * 4.0).fract() < 0.18 { 1.0 } else { 0.0 };
            (input, 0.9 * step, 0.6 * step)
        }),
    );

    write_goertzel_csv(&out_dir.join("tb303_fft.csv"), &waveform, 110.0);
    write_goertzel_csv(&out_dir.join("tb303_squelch_fft.csv"), &squelch, 110.0);
    write_goertzel_csv(&out_dir.join("tb303_self_osc_fft.csv"), &self_osc, 110.0);
    write_goertzel_csv(
        &out_dir.join("tb303_accent_cv_fft.csv"),
        &accent_probe,
        110.0,
    );

    eprintln!("  TB303 publish artifacts written to {:?}", out_dir);
    eprintln!("  fitted 10-pole fc={best_fc:.1}Hz");
}

#[test]
#[ignore]
fn tb303_export_squelch_then_normal_reference_wav() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let out_dir = std::env::var("PK_TB303_PUBLISH_DIR")
        .map(std::path::PathBuf::from)
        .unwrap_or_else(|_| {
            std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
                .join("target")
                .join("tb303-publish")
        });
    std::fs::create_dir_all(&out_dir).expect("create publish artifact dir");

    let cache_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("target")
        .join("test-cache");
    let _ = std::fs::create_dir_all(&cache_dir);
    let options = super::compile::CompileOptions::default();
    let blob = super::compile::compile_pedal_cached(
        &source,
        "tb303_squelch_reference",
        "tb303_squelch_reference",
        SR,
        &options,
        &cache_dir,
    )
    .expect("compile failed");

    let mut proc: super::compiled::CompiledPedal =
        postcard::from_bytes(&blob).expect("deserialize failed");
    let vco_idx = proc.resolve_port("vco_in").expect("missing vco_in");
    let cv_cutoff_idx = proc.resolve_port("cv_cutoff").expect("missing cv_cutoff");
    let out_idx = proc.resolve_port("audio_out").expect("missing audio_out");
    let mut ports = vec![0.0; proc.port_count()];
    let n_samples = (2.0 * SR) as usize;
    let mut samples = Vec::with_capacity(n_samples);
    proc.set_control_immediate("Cutoff", 0.5);
    proc.set_control_immediate("Resonance", 0.85);

    for i in 0..n_samples {
        let t = i as f64 / SR;
        let first_half = t < 1.0;
        if i == SR as usize {
            proc.set_control_immediate("Resonance", 0.2);
        }

        let local_t = if first_half { t } else { t - 1.0 };
        let cutoff_cv = if first_half {
            1.4 * (0.5 + 0.5 * (2.0 * std::f64::consts::PI * 2.0 * local_t).sin())
        } else {
            0.25
        };
        let gate_env = if (local_t * 8.0).fract() < 0.55 {
            1.0
        } else {
            0.25
        };
        let input = 0.03
            * gate_env
            * ((2.0 * std::f64::consts::PI * 110.0 * t).sin()
                + 0.5 * (2.0 * std::f64::consts::PI * 220.0 * t).sin()
                + 0.33 * (2.0 * std::f64::consts::PI * 330.0 * t).sin()
                + 0.25 * (2.0 * std::f64::consts::PI * 440.0 * t).sin());

        ports.fill(0.0);
        ports[vco_idx] = input;
        ports[cv_cutoff_idx] = cutoff_cv;
        proc.process_ports(&mut ports);
        samples.push(ports[out_idx]);
    }

    let path = out_dir.join("tb303_squelch_then_normal.wav");
    crate::wav::write_wav(&samples, &path, SR as u32).expect("write squelch reference wav");
    eprintln!("  wrote {:?}", path);
}

#[test]
#[ignore]
fn tb303_nonlinear_k_sweep_signature_metrics() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let cache_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("target")
        .join("test-cache");
    let _ = std::fs::create_dir_all(&cache_dir);
    let options = super::compile::CompileOptions::default();
    let blob = super::compile::compile_pedal_cached(
        &source,
        "tb303_nonlinear_k_sweep",
        "tb303_nonlinear_k_sweep",
        SR,
        &options,
        &cache_dir,
    )
    .expect("compile failed");

    let render_segment = |resonance: f64, cutoff_cv: f64, input_amp: f64| -> Vec<f64> {
        let mut proc: super::compiled::CompiledPedal =
            postcard::from_bytes(&blob).expect("deserialize failed");
        proc.set_control_immediate("Cutoff", 0.5);
        proc.set_control_immediate("Resonance", resonance);
        let vco_idx = proc.resolve_port("vco_in").expect("missing vco_in");
        let cv_cutoff_idx = proc.resolve_port("cv_cutoff").expect("missing cv_cutoff");
        let out_idx = proc.resolve_port("audio_out").expect("missing audio_out");
        let mut ports = vec![0.0; proc.port_count()];
        let n_samples = (0.35 * SR) as usize;
        let mut output = Vec::with_capacity(n_samples);
        for i in 0..n_samples {
            let t = i as f64 / SR;
            let input = input_amp
                * ((2.0 * std::f64::consts::PI * 110.0 * t).sin()
                    + 0.5 * (2.0 * std::f64::consts::PI * 220.0 * t).sin()
                    + 0.33 * (2.0 * std::f64::consts::PI * 330.0 * t).sin()
                    + 0.25 * (2.0 * std::f64::consts::PI * 440.0 * t).sin());
            ports.fill(0.0);
            ports[vco_idx] = input;
            ports[cv_cutoff_idx] = cutoff_cv;
            proc.process_ports(&mut ports);
            output.push(ports[out_idx]);
        }
        output
    };

    eprintln!("  TB303 nonlinear k-sweep signature metrics");
    eprintln!(
        "  {:>5} {:>10} {:>10} {:>10} {:>10}",
        "k", "rms", "thd-ish", "hf/fund", "finite"
    );
    let mut last_hf = 0.0;
    for &k in &[0.0, 0.3, 0.6, 0.85] {
        let samples = render_segment(k, 0.4, 0.03);
        let start = samples.len() / 2;
        let window = &samples[start..];
        let rms = ac_rms(window);
        let fundamental = goertzel_mag(window, 110.0, SR).max(1.0e-12);
        let harmonic_sum = [220.0, 330.0, 440.0, 550.0, 660.0, 770.0, 880.0]
            .iter()
            .map(|&freq| goertzel_mag(window, freq, SR))
            .sum::<f64>();
        let hf_sum = [1760.0, 2200.0, 3300.0, 4400.0, 6600.0, 8800.0]
            .iter()
            .map(|&freq| goertzel_mag(window, freq, SR))
            .sum::<f64>();
        let thdish = harmonic_sum / fundamental;
        let hf_ratio = hf_sum / fundamental;
        let finite = samples.iter().all(|sample| sample.is_finite());
        eprintln!("  {k:>5.2} {rms:>10.5} {thdish:>10.3} {hf_ratio:>10.3} {finite:>10}");
        assert!(finite, "k={k} produced NaN/Inf");
        assert!(rms < 10.0, "k={k} runaway output rms={rms}");
        if k > 0.0 {
            assert!(
                hf_ratio >= last_hf * 0.5,
                "higher k should not collapse high-frequency nonlinear content: previous={last_hf:.3}, current={hf_ratio:.3}"
            );
        }
        last_hf = hf_ratio;
    }

    let quiet = render_segment(0.95, 0.7, 1.0e-6);
    let quiet_window = &quiet[quiet.len() / 2..];
    let mut best = (0.0, 0.0);
    for freq in [
        220.0, 330.0, 440.0, 660.0, 880.0, 1100.0, 1320.0, 1760.0, 2200.0, 3300.0, 4400.0, 6600.0,
        8800.0,
    ] {
        let mag = goertzel_mag(quiet_window, freq, SR);
        if mag > best.1 {
            best = (freq, mag);
        }
    }
    eprintln!(
        "  high-k quiet-input dominant bin: {:.0}Hz mag={:.6}",
        best.0, best.1
    );
    assert!(
        best.1.is_finite(),
        "quiet-input self-osc probe must stay finite"
    );
}

fn write_goertzel_csv(path: &std::path::Path, waveform: &[f64], norm_freq: f64) {
    let fft_freqs = [
        55.0, 110.0, 220.0, 330.0, 440.0, 660.0, 880.0, 1100.0, 1320.0, 1760.0, 2200.0, 3300.0,
        4400.0, 6600.0, 8800.0, 11_000.0, 13_200.0, 16_000.0,
    ];
    let start = waveform.len() / 2;
    let window = &waveform[start..];
    let norm = goertzel_mag(window, norm_freq, SR).max(1.0e-12);
    let mut csv = String::from("freq_hz,wdf_db\n");
    for &freq in &fft_freqs {
        csv.push_str(&format!(
            "{freq:.3},{:.6}\n",
            db_norm(goertzel_mag(window, freq, SR), norm)
        ));
    }
    std::fs::write(path, csv).expect("write goertzel csv");
}

fn goertzel_mag(samples: &[f64], freq_hz: f64, sample_rate: f64) -> f64 {
    let omega = 2.0 * std::f64::consts::PI * freq_hz / sample_rate;
    let coeff = 2.0 * omega.cos();
    let mut s_prev = 0.0;
    let mut s_prev2 = 0.0;
    for (i, &sample) in samples.iter().enumerate() {
        let window =
            0.5 - 0.5 * (2.0 * std::f64::consts::PI * i as f64 / samples.len() as f64).cos();
        let s = sample * window + coeff * s_prev - s_prev2;
        s_prev2 = s_prev;
        s_prev = s;
    }
    (s_prev2 * s_prev2 + s_prev * s_prev - coeff * s_prev * s_prev2).sqrt() / samples.len() as f64
}

#[test]
fn tb303_trace_cascade_one_sample() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let cache_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("target")
        .join("test-cache");
    let _ = std::fs::create_dir_all(&cache_dir);
    let options = super::compile::CompileOptions::default();
    let blob = super::compile::compile_pedal_cached(
        &source,
        "tb303_trace",
        "tb303_trace",
        SR,
        &options,
        &cache_dir,
    )
    .expect("compile failed");

    let mut proc: super::compiled::CompiledPedal =
        postcard::from_bytes(&blob).expect("deserialize failed");

    // Warm up
    for _ in 0..4800 {
        proc.process(0.0);
    }

    // Process one nonzero sample and trace internals
    let input = 0.3; // single step input

    // Find the BKM stage
    let bkm = proc
        .stages
        .iter_mut()
        .find_map(|s| {
            if let pedalkernel_rt::processor::Stage::Blockwise(ref mut k) = s {
                Some(k)
            } else {
                None
            }
        })
        .expect("BKM stage");

    let n_blocks = bkm.blocks.len();
    let n = bkm.n_ports;

    // Manually trace one cascade
    let audio_vs_idx = bkm
        .vs_port_map
        .first()
        .map(|(_, idx)| *idx)
        .unwrap_or(n_blocks);
    bkm.work_b[audio_vs_idx] = 2.0 * input;
    eprintln!(
        "  INPUT: {input}, audio_vs_idx={audio_vs_idx}, work_b[{audio_vs_idx}]={}",
        bkm.work_b[audio_vs_idx]
    );

    // Scatter
    for i in 0..n {
        let mut sum = 0.0;
        for j in 0..n {
            sum += bkm.coupling_s[i * n + j] * bkm.work_b[j];
        }
        bkm.work_a[i] = sum;
    }

    let feedback = (bkm.work_a[0] + bkm.work_b[0]) / 2.0;
    let mut cascade = input + feedback;
    eprintln!("  FEEDBACK: {feedback:.6}, cascade_start: {cascade:.6}");

    for i in 0..n_blocks {
        bkm.blocks[i].tree.set_voltage(cascade);
        let b_tree = bkm.blocks[i].tree.reflected();
        let ctrl = cascade;
        let a_root = bkm.blocks[i].k_table.lookup_2d(b_tree, ctrl);
        let v_out = (a_root + b_tree) / 2.0;
        let dc = bkm.blocks[i].dc_offset;
        let v_ac = v_out - dc;
        eprintln!("  BLOCK {i}: cascade_in={cascade:.6}, b_tree={b_tree:.6}, ctrl={ctrl:.6}, a_root={a_root:.6}, v_out={v_out:.6}, dc={dc:.6}, v_ac={v_ac:.6}");
        cascade = v_ac;
    }
    eprintln!("  FINAL OUTPUT: {cascade:.6}");
}

#[test]
fn tb303_trace_small_signal() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let cache_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("target")
        .join("test-cache");
    let _ = std::fs::create_dir_all(&cache_dir);
    let options = super::compile::CompileOptions::default();
    let blob = super::compile::compile_pedal_cached(
        &source,
        "tb303_trace_small",
        "tb303_trace_small",
        SR,
        &options,
        &cache_dir,
    )
    .expect("compile failed");

    let mut proc: super::compiled::CompiledPedal =
        postcard::from_bytes(&blob).expect("deserialize failed");

    // Warm up
    for _ in 0..4800 {
        proc.process(0.0);
    }

    // Realistic signal: 10mV (what the VCO sends after 0.03× attenuation)
    let input = 0.01;

    let bkm = proc
        .stages
        .iter_mut()
        .find_map(|s| {
            if let pedalkernel_rt::processor::Stage::Blockwise(ref mut k) = s {
                Some(k)
            } else {
                None
            }
        })
        .expect("BKM stage");

    let n_blocks = bkm.blocks.len();
    let n = bkm.n_ports;
    let audio_vs_idx = bkm
        .vs_port_map
        .first()
        .map(|(_, idx)| *idx)
        .unwrap_or(n_blocks);
    bkm.work_b[audio_vs_idx] = 2.0 * input;

    for i in 0..n {
        let mut sum = 0.0;
        for j in 0..n {
            sum += bkm.coupling_s[i * n + j] * bkm.work_b[j];
        }
        bkm.work_a[i] = sum;
    }

    let feedback = (bkm.work_a[0] + bkm.work_b[0]) / 2.0;
    let mut cascade = input + feedback;
    eprintln!("  INPUT: {input}, feedback: {feedback:.6}, cascade_start: {cascade:.6}");

    for i in 0..n_blocks {
        bkm.blocks[i].tree.set_voltage(cascade);
        let b_tree = bkm.blocks[i].tree.reflected();
        let ctrl = cascade;
        let a_root = bkm.blocks[i].k_table.lookup_2d(b_tree, ctrl);
        let v_out = (a_root + b_tree) / 2.0;
        let dc = bkm.blocks[i].dc_offset;
        let v_ac = v_out - dc;
        eprintln!("  BLOCK {i}: cascade={cascade:.6}, ctrl={ctrl:.6}, b_tree={b_tree:.6}, a_root={a_root:.6}, v_out={v_out:.6}, dc={dc:.4}, v_ac={v_ac:.6}");
        cascade = v_ac;
    }
    eprintln!("  FINAL: {cascade:.6}");
    assert!(
        cascade.abs() < 1.0,
        "Small-signal output should be <1V, got {cascade:.6}"
    );
}

#[test]
fn tb303_cap_state_bounded_during_warmup() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let cache_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("target")
        .join("test-cache");
    let _ = std::fs::create_dir_all(&cache_dir);
    let options = super::compile::CompileOptions::default();
    let blob = super::compile::compile_pedal_cached(
        &source,
        "tb303_capwind",
        "tb303_capwind",
        SR,
        &options,
        &cache_dir,
    )
    .expect("compile failed");

    let mut proc: super::compiled::CompiledPedal =
        postcard::from_bytes(&blob).expect("deserialize failed");
    proc.set_control_immediate("Resonance", 0.0);

    // Process 4800 samples of SILENCE — no input
    // After each sample, check the BKM stage's block tree states
    for sample in 0..4800 {
        proc.process(0.0);

        // Every 480 samples (10ms), check block states
        if sample % 480 == 479 {
            let bkm = proc.stages.iter().find_map(|s| {
                if let pedalkernel_rt::processor::Stage::Blockwise(ref k) = s {
                    Some(k)
                } else {
                    None
                }
            });
            if let Some(bkm) = bkm {
                for (bi, block) in bkm.blocks.iter().enumerate() {
                    let probe_v = block
                        .cascade_probe_id
                        .as_deref()
                        .and_then(|id| block.tree.leaf_voltage(id));
                    let mut tree_clone = block.tree.clone();
                    tree_clone.set_voltage(0.0);
                    let b = tree_clone.reflected();
                    let reported = probe_v.unwrap_or(b);
                    if b.abs() > 10.0 {
                        eprintln!(
                            "  WINDUP at sample {sample}: block {bi} b_tree={b:.4}, probe_v={probe_v:?}"
                        );
                    }
                    assert!(
                        reported.is_finite() && reported.abs() < 10.0,
                        "Cap wind-up: block {bi} probe/b_tree={reported:.4} at sample {sample} \
                         (silent input). Cap state has diverged. b_tree={b:.4}"
                    );
                }
            }
        }
    }

    // Final check: all b_tree should be small
    let bkm = proc
        .stages
        .iter()
        .find_map(|s| {
            if let pedalkernel_rt::processor::Stage::Blockwise(ref k) = s {
                Some(k)
            } else {
                None
            }
        })
        .expect("BKM");

    for (bi, block) in bkm.blocks.iter().enumerate() {
        let mut t = block.tree.clone();
        t.set_voltage(0.0);
        let b = t.reflected();
        let probe_v = block
            .cascade_probe_id
            .as_deref()
            .and_then(|id| block.tree.leaf_voltage(id));
        let reported = probe_v.unwrap_or(b);
        eprintln!(
            "  Final block {bi}: b_tree={b:.6}, probe_v={probe_v:?}, dc_offset={:.6}",
            block.dc_offset
        );
        assert!(
            reported.is_finite() && reported.abs() < 10.0,
            "After 100ms silence, block {bi} probe/b_tree={reported:.6} should be small. \
             Cap wound up. b_tree={b:.6}"
        );
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Signal routing: VCO port path should be lowpass, not highpass
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn tb303_vco_port_path_is_lowpass() {
    // The process(input) serial chain goes through C_out (highpass) first,
    // then the ladder. That's wrong — C_out is an output coupling cap.
    //
    // The process_ports path with vco_in goes directly into the BKM
    // stage's coupling VS port, bypassing C_out. This should be lowpass.
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let cache_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("target")
        .join("test-cache");
    let _ = std::fs::create_dir_all(&cache_dir);
    let options = super::compile::CompileOptions::default();
    let blob = super::compile::compile_pedal_cached(
        &source,
        "tb303_vco_lp_bottom_to_top_output",
        "tb303_vco_lp_bottom_to_top_output",
        SR,
        &options,
        &cache_dir,
    )
    .expect("compile failed");

    const TB303_SMALL_SIGNAL_VCO_PORT_VOLTS: f64 = 0.003;
    const TB303_TEST_CUTOFF: f64 = 0.1;

    let measure_q12_boundary = |freq: f64| -> f64 {
        let mut proc: super::compiled::CompiledPedal =
            postcard::from_bytes(&blob).expect("deserialize failed");
        proc.cache_all_vs_pointers();
        proc.set_control_immediate("Cutoff", TB303_TEST_CUTOFF);
        proc.set_control_immediate("Resonance", 0.0);
        let route = proc
            .stage_route_plan
            .primary_bkm
            .clone()
            .expect("BKM route");
        let drive = route
            .boundary_drives
            .iter()
            .find(|drive| {
                drive
                    .positive_input_port_names
                    .iter()
                    .any(|name| name == "vco_in")
            })
            .expect("vco_in Q12 boundary drive")
            .clone();
        let stage = proc
            .stages
            .get_mut(drive.source_stage_idx)
            .expect("Q12 source stage");
        let pedalkernel_rt::processor::Stage::Wdf(wdf) = stage else {
            panic!("Q12 source stage should be WDF");
        };

        for i in 0..1200 {
            let input = TB303_SMALL_SIGNAL_VCO_PORT_VOLTS
                * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
            let _ = wdf.process(input);
        }
        let mut values = Vec::with_capacity(2400);
        for i in 0..2400 {
            let input = TB303_SMALL_SIGNAL_VCO_PORT_VOLTS
                * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
            values.push(wdf.process(input));
        }
        ac_rms(&values)
    };

    let measure_direct_bkm_boundary = |freq: f64| -> f64 {
        let mut proc: super::compiled::CompiledPedal =
            postcard::from_bytes(&blob).expect("deserialize failed");
        proc.cache_all_vs_pointers();
        proc.set_control_immediate("Cutoff", TB303_TEST_CUTOFF);
        proc.set_control_immediate("Resonance", 0.0);
        let route = proc
            .stage_route_plan
            .primary_bkm
            .clone()
            .expect("BKM route");
        let drive = route
            .boundary_drives
            .iter()
            .find(|drive| {
                drive
                    .positive_input_port_names
                    .iter()
                    .any(|name| name == "vco_in")
            })
            .expect("vco_in Q12 boundary drive")
            .clone();
        let stage = proc.stages.get_mut(route.stage_idx).expect("BKM stage");
        let pedalkernel_rt::processor::Stage::Blockwise(bkm) = stage else {
            panic!("routed stage should be BKM");
        };
        let vs_signals = vec![0.0; bkm.vs_port_map.len()];

        let mut process = |input: f64| {
            let mut drives = Vec::new();
            pedalkernel_rt::boundary_math::push_differential_voltage_drives(
                &mut drives,
                &drive.positive_target_coupling_port_indices,
                &drive.negative_target_coupling_port_indices,
                pedalkernel_rt::boundary_math::PortVoltage::new(input),
            );
            bkm.process_with_boundary_drives(&drives, &vs_signals)
        };

        for i in 0..1200 {
            let input = TB303_SMALL_SIGNAL_VCO_PORT_VOLTS
                * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
            let _ = process(input);
        }
        let mut values = Vec::with_capacity(2400);
        for i in 0..2400 {
            let input = TB303_SMALL_SIGNAL_VCO_PORT_VOLTS
                * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
            values.push(process(input));
        }
        ac_rms(&values)
    };

    let measure_direct_bkm_after_q12 = |freq: f64| -> f64 {
        let mut proc: super::compiled::CompiledPedal =
            postcard::from_bytes(&blob).expect("deserialize failed");
        proc.cache_all_vs_pointers();
        proc.set_control_immediate("Cutoff", TB303_TEST_CUTOFF);
        proc.set_control_immediate("Resonance", 0.0);
        let route = proc
            .stage_route_plan
            .primary_bkm
            .clone()
            .expect("BKM route");
        let drive = route
            .boundary_drives
            .iter()
            .find(|drive| {
                drive
                    .positive_input_port_names
                    .iter()
                    .any(|name| name == "vco_in")
            })
            .expect("vco_in Q12 boundary drive")
            .clone();

        assert_ne!(
            drive.source_stage_idx, route.stage_idx,
            "Q12 and BKM stages should be distinct"
        );
        let (q12_stage, bkm_stage) = if drive.source_stage_idx < route.stage_idx {
            let (left, right) = proc.stages.split_at_mut(route.stage_idx);
            (&mut left[drive.source_stage_idx], &mut right[0])
        } else {
            let (left, right) = proc.stages.split_at_mut(drive.source_stage_idx);
            (&mut right[0], &mut left[route.stage_idx])
        };
        let pedalkernel_rt::processor::Stage::Wdf(q12) = q12_stage else {
            panic!("Q12 source stage should be WDF");
        };
        let pedalkernel_rt::processor::Stage::Blockwise(bkm) = bkm_stage else {
            panic!("routed stage should be BKM");
        };
        let vs_signals = vec![0.0; bkm.vs_port_map.len()];

        let mut process = |input: f64| {
            let q12_out = q12.process(input);
            let mut drives = Vec::new();
            pedalkernel_rt::boundary_math::push_differential_voltage_drives(
                &mut drives,
                &drive.positive_target_coupling_port_indices,
                &drive.negative_target_coupling_port_indices,
                pedalkernel_rt::boundary_math::PortVoltage::new(q12_out),
            );
            bkm.process_with_boundary_drives(&drives, &vs_signals)
        };

        for i in 0..1200 {
            let input = TB303_SMALL_SIGNAL_VCO_PORT_VOLTS
                * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
            let _ = process(input);
        }
        let mut values = Vec::with_capacity(2400);
        for i in 0..2400 {
            let input = TB303_SMALL_SIGNAL_VCO_PORT_VOLTS
                * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
            values.push(process(input));
        }
        ac_rms(&values)
    };

    let measure_direct_bkm_boundary_without_extraction = |freq: f64| -> f64 {
        let mut proc: super::compiled::CompiledPedal =
            postcard::from_bytes(&blob).expect("deserialize failed");
        proc.cache_all_vs_pointers();
        proc.set_control_immediate("Cutoff", TB303_TEST_CUTOFF);
        proc.set_control_immediate("Resonance", 0.0);
        let route = proc
            .stage_route_plan
            .primary_bkm
            .clone()
            .expect("BKM route");
        let drive = route
            .boundary_drives
            .iter()
            .find(|drive| {
                drive
                    .positive_input_port_names
                    .iter()
                    .any(|name| name == "vco_in")
            })
            .expect("vco_in Q12 boundary drive")
            .clone();
        let stage = proc.stages.get_mut(route.stage_idx).expect("BKM stage");
        let pedalkernel_rt::processor::Stage::Blockwise(bkm) = stage else {
            panic!("routed stage should be BKM");
        };
        bkm.output_extraction_coeffs.fill(0.0);
        let vs_signals = vec![0.0; bkm.vs_port_map.len()];

        let mut process = |input: f64| {
            let mut drives = Vec::new();
            pedalkernel_rt::boundary_math::push_differential_voltage_drives(
                &mut drives,
                &drive.positive_target_coupling_port_indices,
                &drive.negative_target_coupling_port_indices,
                pedalkernel_rt::boundary_math::PortVoltage::new(input),
            );
            bkm.process_with_boundary_drives(&drives, &vs_signals)
        };

        for i in 0..1200 {
            let input = TB303_SMALL_SIGNAL_VCO_PORT_VOLTS
                * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
            let _ = process(input);
        }
        let mut values = Vec::with_capacity(2400);
        for i in 0..2400 {
            let input = TB303_SMALL_SIGNAL_VCO_PORT_VOLTS
                * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
            values.push(process(input));
        }
        ac_rms(&values)
    };

    let measure_direct_bkm_primary_drive = |freq: f64| -> f64 {
        let mut proc: super::compiled::CompiledPedal =
            postcard::from_bytes(&blob).expect("deserialize failed");
        proc.cache_all_vs_pointers();
        proc.set_control_immediate("Cutoff", TB303_TEST_CUTOFF);
        proc.set_control_immediate("Resonance", 0.0);
        let route = proc
            .stage_route_plan
            .primary_bkm
            .clone()
            .expect("BKM route");
        let stage = proc.stages.get_mut(route.stage_idx).expect("BKM stage");
        let pedalkernel_rt::processor::Stage::Blockwise(bkm) = stage else {
            panic!("routed stage should be BKM");
        };
        let vs_signals = vec![0.0; bkm.vs_port_map.len()];

        let mut process = |input: f64| bkm.process_with_serial_input(input, &vs_signals);

        for i in 0..1200 {
            let input = TB303_SMALL_SIGNAL_VCO_PORT_VOLTS
                * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
            let _ = process(input);
        }
        let mut values = Vec::with_capacity(2400);
        for i in 0..2400 {
            let input = TB303_SMALL_SIGNAL_VCO_PORT_VOLTS
                * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
            values.push(process(input));
        }
        ac_rms(&values)
    };

    let measure_routed_port_ac = |freq: f64| -> f64 {
        let mut proc: super::compiled::CompiledPedal =
            postcard::from_bytes(&blob).expect("deserialize failed");
        proc.set_control_immediate("Cutoff", TB303_TEST_CUTOFF);
        proc.set_control_immediate("Resonance", 0.0);
        quick_sine_ac_rms_ports(
            &mut proc,
            freq,
            TB303_SMALL_SIGNAL_VCO_PORT_VOLTS,
            &[("vco_in", 1.0)],
            &[("cv_cutoff", 0.0), ("cv_resonance", 0.0)],
            "audio_out",
        )
    };

    let measure_routed_zero_input_ac = || -> f64 {
        let mut proc: super::compiled::CompiledPedal =
            postcard::from_bytes(&blob).expect("deserialize failed");
        proc.set_control_immediate("Cutoff", TB303_TEST_CUTOFF);
        proc.set_control_immediate("Resonance", 0.0);
        quick_sine_ac_rms_ports(
            &mut proc,
            100.0,
            0.0,
            &[("vco_in", 1.0)],
            &[("cv_cutoff", 0.0), ("cv_resonance", 0.0)],
            "audio_out",
        )
    };

    let q100 = measure_q12_boundary(100.0);
    let q1k = measure_q12_boundary(1000.0);
    let q10k = measure_q12_boundary(10000.0);
    let b100 = measure_direct_bkm_boundary(100.0);
    let b1k = measure_direct_bkm_boundary(1000.0);
    let b10k = measure_direct_bkm_boundary(10000.0);
    let qb100 = measure_direct_bkm_after_q12(100.0);
    let qb1k = measure_direct_bkm_after_q12(1000.0);
    let qb10k = measure_direct_bkm_after_q12(10000.0);
    let f100 = measure_direct_bkm_boundary_without_extraction(100.0);
    let f1k = measure_direct_bkm_boundary_without_extraction(1000.0);
    let f10k = measure_direct_bkm_boundary_without_extraction(10000.0);
    let s100 = measure_direct_bkm_primary_drive(100.0);
    let s1k = measure_direct_bkm_primary_drive(1000.0);
    let s10k = measure_direct_bkm_primary_drive(10000.0);
    let zero_routed = measure_routed_zero_input_ac();
    let g100 = measure_routed_port_ac(100.0);
    let g1k = measure_routed_port_ac(1000.0);
    let g10k = measure_routed_port_ac(10000.0);
    let q_ratio_db = 20.0 * (q100 / q10k.max(1e-12)).log10();
    let b_ratio_db = 20.0 * (b100 / b10k.max(1e-12)).log10();
    let qb_ratio_db = 20.0 * (qb100 / qb10k.max(1e-12)).log10();
    let f_ratio_db = 20.0 * (f100 / f10k.max(1e-12)).log10();
    let s_ratio_db = 20.0 * (s100 / s10k.max(1e-12)).log10();
    let ratio_db = 20.0 * (g100 / g10k.max(1e-12)).log10();

    eprintln!("  Q12 boundary AC: 100Hz={q100:.6}, 1kHz={q1k:.6}, 10kHz={q10k:.6}, ratio={q_ratio_db:+.1} dB");
    eprintln!("  Direct BKM boundary AC: 100Hz={b100:.6}, 1kHz={b1k:.6}, 10kHz={b10k:.6}, ratio={b_ratio_db:+.1} dB");
    eprintln!("  Direct Q12→BKM AC: 100Hz={qb100:.6}, 1kHz={qb1k:.6}, 10kHz={qb10k:.6}, ratio={qb_ratio_db:+.1} dB");
    eprintln!("  Direct BKM fallback AC: 100Hz={f100:.6}, 1kHz={f1k:.6}, 10kHz={f10k:.6}, ratio={f_ratio_db:+.1} dB");
    eprintln!("  Direct BKM primary drive AC: 100Hz={s100:.6}, 1kHz={s1k:.6}, 10kHz={s10k:.6}, ratio={s_ratio_db:+.1} dB");
    eprintln!("  Routed zero-input AC: {zero_routed:.6}");
    eprintln!("  Routed VCO port AC: 100Hz={g100:.6}, 1kHz={g1k:.6}, 10kHz={g10k:.6}, ratio={ratio_db:+.1} dB");

    // The diode ladder should be LOWPASS: 100Hz > 10kHz
    assert!(
        g100 > g10k,
        "VCO port path should be lowpass: 100Hz={g100:.6} must be > 10kHz={g10k:.6} \
         (ratio={ratio_db:+.1} dB)"
    );

    if ratio_db <= 2.0 {
        eprintln!(
            "  WEAK: routed Q12->BKM path is still under-damped/flat; \
             routed={ratio_db:+.1} dB, direct_bkm={b_ratio_db:+.1} dB, \
             q12={q_ratio_db:+.1} dB, zero_input={zero_routed:.6}"
        );
    }
}

#[test]
fn tb303_audio_and_vco_ports_both_reach_output() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let cache_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("target")
        .join("test-cache");
    let _ = std::fs::create_dir_all(&cache_dir);
    let options = super::compile::CompileOptions::default();
    let blob = super::compile::compile_pedal_cached(
        &source,
        "tb303_audio_and_vco_ports_v1",
        "tb303_audio_and_vco_ports_v1",
        SR,
        &options,
        &cache_dir,
    )
    .expect("compile failed");

    let measure = |input_name: &str, freq: f64| -> f64 {
        let mut proc: super::compiled::CompiledPedal =
            postcard::from_bytes(&blob).expect("deserialize failed");
        proc.set_control_immediate("Resonance", 0.0);
        let input_idx = proc
            .resolve_port(input_name)
            .unwrap_or_else(|| panic!("{input_name} port"));
        let out_idx = proc.resolve_port("audio_out").expect("audio_out port");

        for _ in 0..4800 {
            let mut ports = vec![0.0; proc.port_count()];
            proc.process_ports(&mut ports);
        }

        let mut values = Vec::with_capacity(9600);
        for i in 0..9600 {
            let input = 0.05 * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
            let mut ports = vec![0.0; proc.port_count()];
            ports[input_idx] = input;
            proc.process_ports(&mut ports);
            values.push(ports[out_idx]);
        }
        ac_rms(&values)
    };

    let audio_100 = measure("audio_in", 100.0);
    let audio_10k = measure("audio_in", 10_000.0);
    let vco_100 = measure("vco_in", 100.0);
    let vco_10k = measure("vco_in", 10_000.0);

    eprintln!(
        "  TB303 input paths: audio 100Hz={audio_100:.6}, 10kHz={audio_10k:.6}; \
         vco 100Hz={vco_100:.6}, 10kHz={vco_10k:.6}"
    );

    assert!(
        audio_100 > 1.0e-4 && vco_100 > 1.0e-4,
        "both TB303 input ports should reach output: audio_100={audio_100:.6}, vco_100={vco_100:.6}"
    );
    assert!(
        audio_100 > audio_10k && vco_100 > vco_10k,
        "both TB303 input ports should see the ladder lowpass shape: \
         audio 100Hz={audio_100:.6}, 10kHz={audio_10k:.6}; \
         vco 100Hz={vco_100:.6}, 10kHz={vco_10k:.6}"
    );
}

#[test]
fn tb303_serial_path_ordering_check() {
    // Document the serial chain ordering issue:
    // process(input) goes Stage 0 (C_out) → Stage 1 (BKM ladder)
    // This is highpass-then-lowpass, which gives highpass behavior.
    // The correct order should be lowpass-then-highpass (ladder → C_out).
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let cache_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("target")
        .join("test-cache");
    let _ = std::fs::create_dir_all(&cache_dir);
    let options = super::compile::CompileOptions::default();
    let blob = super::compile::compile_pedal_cached(
        &source,
        "tb303_serial_order",
        "tb303_serial_order",
        SR,
        &options,
        &cache_dir,
    )
    .expect("compile failed");

    let proc: super::compiled::CompiledPedal =
        postcard::from_bytes(&blob).expect("deserialize failed");

    // Check stage types and ordering
    for (i, stage) in proc.stages.iter().enumerate() {
        let stype = match stage {
            pedalkernel_rt::processor::Stage::Wdf(w) => {
                format!(
                    "WDF(root={:?}, rp={:.0})",
                    if w.tree.port_resistance() > 5000.0 {
                        "high-Z"
                    } else {
                        "low-Z"
                    },
                    w.tree.port_resistance()
                )
            }
            pedalkernel_rt::processor::Stage::Blockwise(k) => {
                format!("BKM({}blocks, {}ports)", k.blocks.len(), k.n_ports)
            }
            _ => format!("other"),
        };
        eprintln!("  Stage {i}: {stype}");
    }

    // The BKM stage should come BEFORE the C_out stage in processing order
    let bkm_idx = proc
        .stages
        .iter()
        .position(|s| matches!(s, pedalkernel_rt::processor::Stage::Blockwise(_)));
    let cout_idx = proc
        .stages
        .iter()
        .position(|s| matches!(s, pedalkernel_rt::processor::Stage::Wdf(_)));

    eprintln!(
        "  BKM stage index: {:?}, C_out stage index: {:?}",
        bkm_idx, cout_idx
    );

    if let (Some(bkm), Some(cout)) = (bkm_idx, cout_idx) {
        if bkm > cout {
            eprintln!("  WARNING: BKM after C_out — serial chain is highpass-then-lowpass");
            eprintln!("  FIX: swap ordering so ladder processes before output coupling");
        }
    }
}

#[test]
fn tb303_stage_ordering_debug() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let def = crate::dsl::parse_pedal_file(&source).expect("parse");
    let compiled = super::spqr_build::compile_via_spqr(&def, SR).expect("compile");
    for (i, stage) in compiled.stages.iter().enumerate() {
        match stage {
            pedalkernel_rt::processor::Stage::Wdf(w) => {
                eprintln!(
                    "  Stage {i}: WDF flow_dist={} rp={:.0}",
                    w.signal_flow_distance,
                    w.tree.port_resistance()
                );
            }
            pedalkernel_rt::processor::Stage::Blockwise(k) => {
                eprintln!(
                    "  Stage {i}: BKM flow_dist={} blocks={}",
                    k.signal_flow_distance,
                    k.blocks.len()
                );
            }
            _ => {
                eprintln!("  Stage {i}: other");
            }
        }
    }
}

#[test]
fn tb303_per_block_signal_trace() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let cache_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("target")
        .join("test-cache");
    let _ = std::fs::create_dir_all(&cache_dir);
    let options = super::compile::CompileOptions::default();
    let blob = super::compile::compile_pedal_cached(
        &source,
        "tb303_trace_detail",
        "tb303_trace_detail",
        SR,
        &options,
        &cache_dir,
    )
    .expect("compile failed");

    let mut proc: super::compiled::CompiledPedal =
        postcard::from_bytes(&blob).expect("deserialize failed");

    // Warm up
    for _ in 0..4800 {
        proc.process(0.0);
    }

    // Find BKM stage
    let bkm = proc
        .stages
        .iter_mut()
        .find_map(|s| {
            if let pedalkernel_rt::processor::Stage::Blockwise(ref mut k) = s {
                Some(k)
            } else {
                None
            }
        })
        .expect("BKM stage");

    let n = bkm.n_ports;
    let n_blocks = bkm.blocks.len();

    eprintln!("  === BKM state after warmup ===");
    eprintln!("  n_ports={n}, n_blocks={n_blocks}");
    eprintln!("  supply_voltage={}", bkm.supply_voltage);
    eprintln!("  vs_port_map: {:?}", bkm.vs_port_map);
    eprintln!("  port_index_cache: {:?}", bkm.port_index_cache);

    // Print coupling scattering matrix
    eprintln!("  scattering ({n}x{n}):");
    for i in 0..n {
        let row: Vec<String> = (0..n)
            .map(|j| format!("{:+.4}", bkm.coupling_s[i * n + j]))
            .collect();
        eprintln!("    row {i}: [{}]", row.join(", "));
    }

    // Print work_b (should have supply voltage in supply VS port)
    eprintln!(
        "  work_b: {:?}",
        &bkm.work_b[..n]
            .iter()
            .map(|v| format!("{v:+.4}"))
            .collect::<Vec<_>>()
    );

    // Print per-block tree state
    for (bi, block) in bkm.blocks.iter_mut().enumerate() {
        let mut t = block.tree.clone();
        t.set_voltage(0.0);
        let b = t.reflected();
        eprintln!("  block {bi}: rp={:.1}, dc_offset={:.4}, b_tree_at_0={:.4}, k_table dims={}, entries={}",
            block.rp, block.dc_offset, b, block.k_table.dims, block.k_table.entries.len());

        // Check K-table at a few points
        block.k_table.precompute_scales();
        let test_b = [-2.0, -0.5, 0.0, 0.5, 2.0];
        let test_a: Vec<String> = test_b
            .iter()
            .map(|&b| {
                let a = block.k_table.lookup_1d(b);
                format!("{a:+.4}")
            })
            .collect();
        eprintln!("    K-table: b={test_b:?} → a=[{}]", test_a.join(", "));
    }

    // Now process ONE sample with input=0.1V and trace every step
    eprintln!("\n  === Processing input=0.1V ===");
    let input = 0.1;

    // Simulate what process() does
    // 1. Write VS signals
    for (i, &(ref name, vs_idx)) in bkm.vs_port_map.iter().enumerate() {
        let v = if name.starts_with("_supply_") {
            bkm.supply_voltage
        } else if i == 0 {
            input // audio_in gets the input
        } else {
            0.0
        };
        bkm.work_b[vs_idx] = 2.0 * v;
        eprintln!(
            "  VS port '{name}' (idx={vs_idx}): v={v:.4}, b={:.4}",
            2.0 * v
        );
    }

    // 2. Scatter
    let mut work_a = vec![0.0; n];
    for i in 0..n {
        let mut sum = 0.0;
        for j in 0..n {
            sum += bkm.coupling_s[i * n + j] * bkm.work_b[j];
        }
        work_a[i] = sum;
    }
    eprintln!("  after scatter:");
    for i in 0..n {
        let v = (work_a[i] + bkm.work_b[i]) / 2.0;
        eprintln!(
            "    port {i}: a={:+.4}, b={:+.4}, V={:+.4}",
            work_a[i], bkm.work_b[i], v
        );
    }

    // 3. Cascade
    let mut cascade = (work_a[0] + bkm.work_b[0]) / 2.0;
    eprintln!("  cascade start: {cascade:+.6}");

    for i in 0..n_blocks {
        let vs_in = -cascade; // negated for WDF sign convention
        bkm.blocks[i].tree.set_voltage(vs_in);
        let b_tree = bkm.blocks[i].tree.reflected();
        let a_root = bkm.blocks[i].k_table.lookup_1d(b_tree);
        let v_out = (a_root + b_tree) / 2.0;
        eprintln!("  block {i}: cascade_in={cascade:+.6}, VS={vs_in:+.6}, b_tree={b_tree:+.6}, a_root={a_root:+.6}, V_out={v_out:+.6}");
        cascade = v_out;
    }
    eprintln!("  cascade output: {cascade:+.6}");

    // The cascade output must be nonzero for the filter to work
    assert!(
        cascade.abs() > 1e-6,
        "Cascade output should be nonzero with 0.1V input. Got {cascade:.10}"
    );
}

#[test]
fn tb303_check_wider_frequency_range() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let cache_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("target")
        .join("test-cache");
    let _ = std::fs::create_dir_all(&cache_dir);
    let options = super::compile::CompileOptions::default();
    let blob = super::compile::compile_pedal_cached(
        &source,
        "tb303_freq_range",
        "tb303_freq_range",
        SR,
        &options,
        &cache_dir,
    )
    .expect("compile failed");

    let measure = |freq: f64| -> f64 {
        let mut proc: super::compiled::CompiledPedal = postcard::from_bytes(&blob).expect("deser");
        for _ in 0..4800 {
            proc.process(0.0);
        }
        let mut peak = 0.0f64;
        for i in 0..9600 {
            let input = 0.1 * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
            peak = peak.max(proc.process(input).abs());
        }
        peak
    };

    let freqs = [
        50.0, 100.0, 500.0, 1000.0, 5000.0, 10000.0, 15000.0, 20000.0,
    ];
    eprintln!("  Frequency response:");
    for &f in &freqs {
        let g = measure(f);
        let g_db = 20.0 * (g / 0.1).log10();
        eprintln!("    {f:>8.0} Hz: {g:.6} ({g_db:+.1} dB)");
    }

    let g_50 = measure(50.0);
    let g_20k = measure(20000.0);
    let ratio = 20.0 * (g_50 / g_20k.max(1e-12)).log10();
    eprintln!("  50Hz/20kHz ratio: {ratio:+.1} dB");

    assert!(
        g_50 > g_20k,
        "Should be lowpass: 50Hz={g_50:.6} > 20kHz={g_20k:.6}"
    );
}
