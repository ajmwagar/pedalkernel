//! K-method table generation for WDF nonlinear roots.
//!
//! Replaces Newton-Raphson iteration with precomputed lookup tables.
//! The table maps incident wave (b_tree) → reflected wave (a_root)
//! for a fixed port resistance, by evaluating the NL device's I-V
//! characteristic across the input domain at compile time.
//!
//! 1D tables: diodes, zeners (single junction voltage)
//! 2D tables: BJTs, JFETs, triodes (incident wave × control voltage)

use pedalkernel_rt::stage::{KTable, RootKind, WdfStage};

/// Default number of table entries per dimension.
const DEFAULT_STEPS_1D: usize = 1024;
const DEFAULT_STEPS_2D: usize = 256;

/// Input domain bounds (volts in wave domain).
/// Most guitar pedal signals stay within ±10V.
const B_RANGE: f64 = 25.0;

/// Control voltage domain for 2D devices.
/// BJT Vbe: roughly -0.5 to 0.8V. We go wider for safety.
const CTRL_MIN_BJT: f64 = -2.0;
const CTRL_MAX_BJT: f64 = 2.0;

/// Try to generate a K-method table for a WDF stage's NL root.
///
/// Returns `Some(KTable)` if the root is K-method eligible (memoryless,
/// low-dimensional). Returns `None` for linear roots, high-dimensional
/// devices, or roots with internal state.
pub(super) fn generate_k_table(stage: &mut WdfStage) -> Option<KTable> {
    let rp = stage.tree.port_resistance();
    #[cfg(test)]
    {
        let root_name = match &stage.root {
            RootKind::DiodePair(_) => "DiodePair",
            RootKind::SingleDiode(_) => "SingleDiode",
            RootKind::ExplicitDiodePair(_) => "ExplicitDiodePair",
            RootKind::ExplicitSingleDiode(_) => "ExplicitSingleDiode",
            RootKind::Zener(_) => "Zener",
            RootKind::Jfet(_) => "Jfet",
            RootKind::Triode(_) => "Triode",
            RootKind::Mosfet(_) => "Mosfet",
            RootKind::Bjt(_) => "Bjt",
            RootKind::ShortCircuit => "ShortCircuit",
            RootKind::Passthrough => "Passthrough",
            RootKind::OpAmp(_) => "OpAmp",
            _ => "other",
        };
        eprintln!("  K-method: rp={rp:.1}, root={root_name}");
    }
    if rp <= 0.0 {
        return None;
    }

    // Check K-method eligibility via RootKind::k_method_candidacy()
    let (eligible, _dims) = stage.root.k_method_candidacy();
    if !eligible {
        return None;
    }

    // Use dimensionality from candidacy check
    if _dims == 1 {
        // 1D: sweep b_tree, use RootKind's process method directly
        let root = &mut stage.root;
        Some(sweep_1d(|b| root.process(b, rp), DEFAULT_STEPS_1D))
    } else {
        // 2D devices need control voltage sweep — use set_control_voltage + process
        let (ctrl_min, ctrl_max) = match &stage.root {
            RootKind::Triode(_) => (-4.0, 1.0),
            RootKind::Jfet(_) => (-3.0, 0.5),
            RootKind::Mosfet(_) => (-1.0, 5.0),
            RootKind::Bjt(_) => (-0.2, 0.8), // Vbe range: cutoff to saturation
            RootKind::DiffPair(_) => (-0.9, 4.0), // I_tail modulation: 0.1× to 5× bias
            _ => return None,
        };
        // Build table inline (not via sweep_2d) so we can reset NR state
        // between entries. This ensures each entry is a cold-start solve,
        // making the table a pure function of (b, ctrl) — no dependence
        // on sweep order or warm-starting.
        let b_min = -B_RANGE;
        let b_max = B_RANGE;
        let steps = DEFAULT_STEPS_2D;
        let mut entries = Vec::with_capacity(steps * steps);
        let root = &mut stage.root;

        let mut nan_count = 0usize;
        for ic in 0..steps {
            let tc = ic as f64 / (steps - 1) as f64;
            let ctrl = ctrl_min + tc * (ctrl_max - ctrl_min);
            if ic % 32 == 0 || ic == steps - 1 {
                eprintln!(
                    "  K-table 2D: row {ic}/{steps} (ctrl={ctrl:.3}), {nan_count} NaN so far"
                );
                // Also print as cargo:warning so it shows in `cargo build` output
                #[cfg(feature = "build-cache")]
                println!(
                    "cargo:warning=K-table 2D: row {ic}/{steps} (ctrl={ctrl:.3}), {nan_count} NaN"
                );
            }
            for ib in 0..steps {
                let tb = ib as f64 / (steps - 1) as f64;
                let b = b_min + tb * (b_max - b_min);
                // Reset NR warm-start state for clean cold-start solve
                root.reset_nr_state();
                root.set_control_voltage(ctrl, 1.0, 0.0);
                let a = root.process(b, rp);
                if !a.is_finite() {
                    nan_count += 1;
                }
                entries.push(if a.is_finite() { a } else { 0.0 });
            }
        }
        eprintln!(
            "  K-table 2D: done, {steps}x{steps} = {} entries, {nan_count} NaN",
            steps * steps
        );
        #[cfg(feature = "build-cache")]
        println!(
            "cargo:warning=K-table 2D: done, {nan_count} NaN out of {}",
            steps * steps
        );

        let mut t = KTable {
            dims: 2,
            b_min,
            b_max,
            ctrl_min,
            ctrl_max,
            steps,
            entries,
            inv_b_scale: 0.0,
            inv_c_scale: 0.0,
        };
        t.precompute_scales();
        Some(t)
    }
}

/// Sweep a 1D NL root across the input domain.
fn sweep_1d<F: FnMut(f64) -> f64>(mut f: F, steps: usize) -> KTable {
    let b_min = -B_RANGE;
    let b_max = B_RANGE;
    let mut entries = Vec::with_capacity(steps);

    for i in 0..steps {
        let t = i as f64 / (steps - 1) as f64;
        let b = b_min + t * (b_max - b_min);
        let a = f(b);
        entries.push(if a.is_finite() { a } else { 0.0 });
    }

    let mut t = KTable {
        dims: 1,
        b_min,
        b_max,
        ctrl_min: 0.0,
        ctrl_max: 0.0,
        steps,
        entries,
        inv_b_scale: 0.0,
        inv_c_scale: 0.0,
    };
    t.precompute_scales();
    t
}

/// Sweep a 2D NL root across (b_tree, control_voltage).
fn sweep_2d<F: FnMut(f64, f64) -> f64>(
    mut f: F,
    steps: usize,
    ctrl_min: f64,
    ctrl_max: f64,
) -> KTable {
    let b_min = -B_RANGE;
    let b_max = B_RANGE;
    let mut entries = Vec::with_capacity(steps * steps);

    for ic in 0..steps {
        let tc = ic as f64 / (steps - 1) as f64;
        let ctrl = ctrl_min + tc * (ctrl_max - ctrl_min);
        for ib in 0..steps {
            let tb = ib as f64 / (steps - 1) as f64;
            let b = b_min + tb * (b_max - b_min);
            let a = f(b, ctrl);
            entries.push(if a.is_finite() { a } else { 0.0 });
        }
    }

    let mut t = KTable {
        dims: 2,
        b_min,
        b_max,
        ctrl_min,
        ctrl_max,
        steps,
        entries,
        inv_b_scale: 0.0,
        inv_c_scale: 0.0,
    };
    t.precompute_scales();
    t
}
