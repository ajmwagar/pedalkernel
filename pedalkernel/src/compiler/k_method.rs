//! K-method table generation for WDF nonlinear roots.
//!
//! Replaces Newton-Raphson iteration with precomputed lookup tables.
//! The table maps incident wave (b_tree) → reflected wave (a_root)
//! for a fixed port resistance, by evaluating the NL device's I-V
//! characteristic across the input domain at compile time.
//!
//! 1D tables: diodes, zeners (single junction voltage)
//! 2D tables: BJTs, JFETs, triodes (incident wave × control voltage)

use pedalkernel_rt::elements::nonlinear::diode::DiodeModel;
use pedalkernel_rt::math;
use pedalkernel_rt::stage::{KTable, RootKind, WdfStage};

use super::component::{KMethodAxis, KMethodSpec};

/// Default number of table entries per dimension.
const DEFAULT_STEPS_1D: usize = 8192;
const DEFAULT_STEPS_2D: usize = 256;

/// Input domain bounds (volts in wave domain).
/// Most guitar pedal signals stay within ±10V.
const B_RANGE: f64 = 25.0;
const B_RANGE_1D: f64 = 10.0;

/// Control voltage domain for 2D devices.
/// BJT Vbe: roughly -0.5 to 0.8V. We go wider for safety.
const CTRL_MIN_BJT: f64 = -2.0;
const CTRL_MAX_BJT: f64 = 2.0;
const CTRL_MIN_BIAS: f64 = -5.0;
const CTRL_MAX_BIAS: f64 = 5.0;

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
            RootKind::ZenerPair(_) => "ZenerPair",
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
                entries.push(if a.is_finite() {
                    a as pedalkernel_rt::Wave
                } else {
                    0.0 as pedalkernel_rt::Wave
                });
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

pub(super) fn generate_k_table_for_spec(stage: &mut WdfStage, spec: KMethodSpec) -> Option<KTable> {
    if spec.axes == [KMethodAxis::IncidentWave] {
        let rp = stage.tree.port_resistance();
        if rp <= 0.0 {
            return None;
        }
        let root = &mut stage.root;
        Some(sweep_1d(|b| root.process(b, rp), DEFAULT_STEPS_1D))
    } else if spec.axes == [KMethodAxis::IncidentWave, KMethodAxis::DeviceControl] {
        generate_k_table(stage)
    } else {
        None
    }
}

pub(super) fn generate_biased_single_diode_k_table(model: DiodeModel, rp: f64) -> KTable {
    sweep_2d(
        |b_tree, bias| biased_single_diode_reflection(model, rp, b_tree, bias),
        DEFAULT_STEPS_2D,
        CTRL_MIN_BIAS,
        CTRL_MAX_BIAS,
    )
}

pub(super) fn generate_differential_ladder_tanh_table(n_vt: f64) -> KTable {
    let n_vt = n_vt.max(1.0e-6);
    sweep_1d(
        |v_dm| {
            let x = (v_dm / (2.0 * n_vt)).clamp(-80.0, 80.0);
            x.tanh()
        },
        DEFAULT_STEPS_1D,
    )
}

fn biased_single_diode_reflection(model: DiodeModel, rp: f64, b_tree: f64, bias: f64) -> f64 {
    // Incremental diode equation around a DC operating point:
    //   v_ac + Rp * (I(v_bias + v_ac) - I(v_bias)) = b_tree / 2
    //
    // The root table returns the reflected AC wave. This lets BKM pass raw
    // coupling voltages between blocks while preserving the local small-signal
    // conductance set by the bias point.
    let i_bias = diode_current(model, bias);
    let target = b_tree * 0.5;
    let mut v_ac = target;

    for _ in 0..16 {
        let v = bias + v_ac;
        let i = diode_current(model, v);
        let g = diode_conductance(model, v);
        let f = v_ac + rp * (i - i_bias) - target;
        let df = 1.0 + rp * g;
        if df.abs() < 1.0e-18 {
            break;
        }
        let step = (f / df).clamp(-1.0, 1.0);
        v_ac -= step;
        if step.abs() < 1.0e-12 {
            break;
        }
    }

    2.0 * v_ac - b_tree
}

fn diode_current(model: DiodeModel, v: f64) -> f64 {
    let x = (v / model.n_vt).clamp(-80.0, 80.0);
    model.is * (math::exp(x) - 1.0)
}

fn diode_conductance(model: DiodeModel, v: f64) -> f64 {
    let x = (v / model.n_vt).clamp(-80.0, 80.0);
    model.is * math::exp(x) / model.n_vt
}

/// Sweep a 1D NL root across the input domain.
fn sweep_1d<F: FnMut(f64) -> f64>(mut f: F, steps: usize) -> KTable {
    let b_min = -B_RANGE_1D;
    let b_max = B_RANGE_1D;
    let mut entries = Vec::with_capacity(steps);

    for i in 0..steps {
        let t = i as f64 / (steps - 1) as f64;
        let b = b_min + t * (b_max - b_min);
        let a = f(b);
        entries.push(if a.is_finite() {
            a as pedalkernel_rt::Wave
        } else {
            0.0 as pedalkernel_rt::Wave
        });
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
            entries.push(if a.is_finite() {
                a as pedalkernel_rt::Wave
            } else {
                0.0 as pedalkernel_rt::Wave
            });
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

#[cfg(test)]
mod tests {
    use super::*;
    use pedalkernel_rt::elements::nonlinear::diode::{DiodeModel, ExplicitDiodeRoot};
    use pedalkernel_rt::elements::WdfRoot;

    #[test]
    fn one_dimensional_diode_table_preserves_local_small_signal_slope() {
        let model = DiodeModel::silicon();
        let rp = 346.4;
        let mut table_root = ExplicitDiodeRoot::new(model);
        let table = sweep_1d(|b| table_root.process(b, rp), DEFAULT_STEPS_1D);

        let slope = |table: &KTable, center: f64| {
            let delta = 1.0e-3;
            (table.lookup_1d(center + delta) - table.lookup_1d(center - delta)) / (2.0 * delta)
        };
        let exact_slope = |center: f64| {
            let delta = 1.0e-3;
            let mut root_hi = ExplicitDiodeRoot::new(model);
            let mut root_lo = ExplicitDiodeRoot::new(model);
            (root_hi.process(center + delta, rp) - root_lo.process(center - delta, rp))
                / (2.0 * delta)
        };

        for center in [-0.25, 0.0, 0.25, 0.5] {
            let table_slope = slope(&table, center);
            let exact = exact_slope(center);
            let rel_err = ((table_slope - exact) / exact.abs().max(1.0e-12)).abs();
            assert!(
                rel_err < 0.20,
                "1D diode K-table should preserve local slope near operating point {center:.3}V: \
                 table={table_slope:.6}, exact={exact:.6}, rel_err={rel_err:.3}"
            );
        }
    }

    #[test]
    fn biased_single_diode_table_changes_incremental_slope_with_bias_voltage() {
        let model = DiodeModel::silicon();
        let rp = 346.4;
        let table = generate_biased_single_diode_k_table(model, rp);

        let slope_at = |bias: f64| {
            let delta = 1.0e-3;
            (table.lookup_2d(delta, bias) - table.lookup_2d(-delta, bias)) / (2.0 * delta)
        };

        let slope_unbiased = slope_at(0.0).abs();
        let slope_biased = slope_at(0.55).abs();

        assert!(
            slope_biased > slope_unbiased * 100.0,
            "biased diode K-table should encode operating-point conductance: \
             slope@0V={slope_unbiased:.6}, slope@0.55V={slope_biased:.6}"
        );
    }
}
