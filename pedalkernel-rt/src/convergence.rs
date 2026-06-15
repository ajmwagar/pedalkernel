//! Parallel-branch convergence summation (F13b).
//!
//! Some circuits route several independent active branches (each driven by an
//! op-amp output — i.e. an ideal voltage source) into one downstream *linear,
//! resistive* mixer whose output is the circuit's global `out`. The classic
//! example is the TR-808 snare: two bridged-T resonators (one per op-amp half)
//! feed a blend pot whose wiper is the output.
//!
//! Such a mixer cannot be expressed as a single-input WDF/serial stage — it has
//! `N >= 2` driving inputs. Because the mixer is **linear** and every driver is
//! an ideal voltage source, the output is exactly a weighted sum by
//! superposition:
//!
//! ```text
//! V_out = Σ_i g_i · V_i
//! ```
//!
//! where each transfer coefficient `g_i` is obtained from one DC resistive solve:
//! set branch `i`'s source = 1 V, **short** every other branch source to ground
//! (the correct superposition rule for voltage sources — it captures inter-branch
//! loading exactly), and read the voltage at the output node.
//!
//! The `g_i` are pure functions of the resistor/pot values, so they are computed
//! at compile time and re-solved only when a control (e.g. the blend pot) moves —
//! never per sample. The per-sample cost of a convergence stage is therefore just
//! `N` multiply-adds against the `node_signals` bus.

use crate::pot_taper::PotTaper;
use alloc::string::String;
use alloc::vec;
use alloc::vec::Vec;

/// A resistive branch with a constant conductance (`1/R`).
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Clone, Debug)]
pub struct FixedBranch {
    /// Local node index (within the convergence network) or `None` for ground.
    pub na: Option<usize>,
    pub nb: Option<usize>,
    pub conductance: crate::Wave,
}

/// One resistive half of a potentiometer whose value tracks the wiper position.
///
/// A 3-terminal pot contributes two segments: `a→w` with `R = pos·max_r`
/// (`is_aw = true`) and `w→b` with `R = (1−pos)·max_r` (`is_aw = false`).
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Clone, Debug)]
pub struct PotBranch {
    pub comp_id: String,
    pub na: Option<usize>,
    pub nb: Option<usize>,
    pub max_r: crate::Wave,
    pub taper: PotTaper,
    /// `true` = the `a→w` half (R = pos·max_r); `false` = `w→b` (R = (1−pos)·max_r).
    pub is_aw: bool,
    /// Current normalized pot position [0, 1] (pre-taper).
    pub position: crate::Wave,
}

/// One driving branch: an upstream op-amp-output node and its transfer gain.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Clone, Debug)]
pub struct ConvergenceBranch {
    /// Circuit graph node id that this branch's driver (op-amp `out`) publishes
    /// into `node_signals`.
    pub source_node_id: usize,
    /// Local node index of `source_node_id` within the resistive network.
    pub source_local: usize,
    /// Transfer coefficient `g_i` from this driver to the output node.
    pub gain: crate::Wave,
}

/// A convergence-summation stage: `V_out = Σ_i g_i · node_signals[source_i]`.
///
/// Carries the resistive mixer network so the gains can be re-solved at
/// control-rate when a pot moves. The per-sample evaluation never touches the
/// network — it is `N` multiply-adds only.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Clone, Debug)]
pub struct ConvergenceSum {
    /// Driving branches (one per upstream active source).
    pub branches: Vec<ConvergenceBranch>,
    /// Circuit graph node id of the global output (where the sum is published).
    pub output_node_id: usize,
    /// Number of non-ground local nodes in the resistive network.
    pub num_nodes: usize,
    /// Local node index of the output node within the network.
    pub output_local: Option<usize>,
    /// Constant resistive branches (R_out, fixed mixer resistors).
    pub fixed_branches: Vec<FixedBranch>,
    /// Pot segments whose conductance varies with wiper position.
    pub pot_branches: Vec<PotBranch>,
}

impl ConvergenceSum {
    /// Evaluate the sum for the current driver voltages. `O(N)` multiply-adds.
    #[inline]
    pub fn evaluate(&self, driver_voltage: impl Fn(usize) -> crate::Wave) -> crate::Wave {
        let mut acc = 0.0;
        for b in &self.branches {
            acc += b.gain * driver_voltage(b.source_node_id);
        }
        acc
    }

    /// Update the stored position of every pot segment matching `comp_id`.
    /// Returns true if any segment changed (and thus gains must be re-solved).
    pub fn set_pot(&mut self, comp_id: &str, position: crate::Wave) -> bool {
        let mut changed = false;
        for pb in &mut self.pot_branches {
            if pb.comp_id == comp_id {
                pb.position = position;
                changed = true;
            }
        }
        changed
    }

    /// Re-solve every branch gain `g_i` from the current resistor/pot values.
    ///
    /// For branch `i`: stamp a unit voltage source at its source node, short all
    /// other source nodes to ground, solve the DC resistive system, read the
    /// output node voltage. This is the textbook superposition transfer.
    ///
    /// Runs at compile time and on control-rate pot changes only.
    pub fn recompute_gains(&mut self) {
        let n = self.num_nodes;
        let Some(out_local) = self.output_local else {
            for b in &mut self.branches {
                b.gain = 0.0;
            }
            return;
        };

        // Conductance matrix for the *passive* network (no sources yet).
        let mut g_base = vec![0.0 as crate::Wave; n * n];
        let stamp =
            |g: &mut [crate::Wave], na: Option<usize>, nb: Option<usize>, cond: crate::Wave| {
                if let Some(i) = na {
                    g[i * n + i] += cond;
                    if let Some(j) = nb {
                        g[i * n + j] -= cond;
                    }
                }
                if let Some(j) = nb {
                    g[j * n + j] += cond;
                    if let Some(i) = na {
                        g[j * n + i] -= cond;
                    }
                }
            };
        for fb in &self.fixed_branches {
            stamp(&mut g_base, fb.na, fb.nb, fb.conductance);
        }
        for pb in &self.pot_branches {
            let tapered = pb.taper.apply(pb.position);
            let frac = if pb.is_aw { tapered } else { 1.0 - tapered };
            let r = (frac * pb.max_r).max(1.0);
            stamp(&mut g_base, pb.na, pb.nb, 1.0 / r);
        }

        let driver_locals: Vec<usize> = self.branches.iter().map(|b| b.source_local).collect();

        for bi in 0..self.branches.len() {
            // Build the reduced system: every driver node is a fixed-voltage node.
            // The active branch source = 1 V, all others = 0 V (shorted to gnd).
            // Eliminate the driver rows/cols by moving their known voltages to the
            // RHS, then solve for the remaining (internal) node voltages.
            let active = driver_locals[bi];

            let is_driver = |node: usize| driver_locals.contains(&node);
            let known_v = |node: usize| -> crate::Wave {
                if node == active {
                    1.0
                } else {
                    0.0
                }
            };

            // Map unknown (non-driver) nodes to a compact index.
            let mut unknown_index = vec![usize::MAX; n];
            let mut unknowns: Vec<usize> = Vec::new();
            for (node, slot) in unknown_index.iter_mut().enumerate() {
                if !is_driver(node) {
                    *slot = unknowns.len();
                    unknowns.push(node);
                }
            }

            let v_out = if unknown_index[out_local] == usize::MAX {
                // Output is itself a driver node — its voltage is the known value.
                known_v(out_local)
            } else {
                let m = unknowns.len();
                let mut a = vec![0.0 as crate::Wave; m * m];
                let mut rhs = vec![0.0 as crate::Wave; m];
                for (ri, &node) in unknowns.iter().enumerate() {
                    for col in 0..n {
                        let gij = g_base[node * n + col];
                        if gij == 0.0 {
                            continue;
                        }
                        let ci = unknown_index[col];
                        if ci != usize::MAX {
                            a[ri * m + ci] += gij;
                        } else {
                            // Known (driver) column: move to RHS.
                            rhs[ri] -= gij * known_v(col);
                        }
                    }
                }
                let sol = solve_dense(&a, &rhs, m);
                sol.get(unknown_index[out_local]).copied().unwrap_or(0.0)
            };

            self.branches[bi].gain = if v_out.is_finite() { v_out } else { 0.0 };
        }
    }
}

/// Solve `A·x = b` for a small dense system via Gaussian elimination with
/// partial pivoting. Returns zeros if the system is singular.
fn solve_dense(a_in: &[crate::Wave], b_in: &[crate::Wave], n: usize) -> Vec<crate::Wave> {
    if n == 0 {
        return Vec::new();
    }
    let mut a = a_in.to_vec();
    let mut b = b_in.to_vec();
    for col in 0..n {
        // Partial pivot.
        let mut piv = col;
        let mut best = a[col * n + col].abs();
        for r in (col + 1)..n {
            let v = a[r * n + col].abs();
            if v > best {
                best = v;
                piv = r;
            }
        }
        if best < 1e-18 {
            return vec![0.0; n];
        }
        if piv != col {
            for c in 0..n {
                a.swap(col * n + c, piv * n + c);
            }
            b.swap(col, piv);
        }
        let inv = 1.0 / a[col * n + col];
        for r in 0..n {
            if r == col {
                continue;
            }
            let factor = a[r * n + col] * inv;
            if factor == 0.0 {
                continue;
            }
            for c in col..n {
                a[r * n + c] -= factor * a[col * n + c];
            }
            b[r] -= factor * b[col];
        }
    }
    let mut x = vec![0.0; n];
    for i in 0..n {
        x[i] = b[i] / a[i * n + i];
    }
    x
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::string::ToString;

    /// Snare-style blend pot: drivers at local 0 (a) and 1 (b), wiper at 2,
    /// output at 3 fed through R_out from the wiper (output open / no load).
    fn snare_mixer(position: crate::Wave) -> ConvergenceSum {
        let mut cs = ConvergenceSum {
            branches: vec![
                ConvergenceBranch {
                    source_node_id: 25,
                    source_local: 0,
                    gain: 0.0,
                },
                ConvergenceBranch {
                    source_node_id: 37,
                    source_local: 1,
                    gain: 0.0,
                },
            ],
            output_node_id: 44,
            num_nodes: 4,
            output_local: Some(3),
            fixed_branches: vec![FixedBranch {
                na: Some(2),
                nb: Some(3),
                conductance: 1.0 / 10_000.0,
            }],
            pot_branches: vec![
                PotBranch {
                    comp_id: "Tone".to_string(),
                    na: Some(0),
                    nb: Some(2),
                    max_r: 100_000.0,
                    taper: PotTaper::B,
                    is_aw: true,
                    position,
                },
                PotBranch {
                    comp_id: "Tone".to_string(),
                    na: Some(2),
                    nb: Some(1),
                    max_r: 100_000.0,
                    taper: PotTaper::B,
                    is_aw: false,
                    position,
                },
            ],
        };
        cs.recompute_gains();
        cs
    }

    #[test]
    fn blend_center_is_equal_split() {
        let cs = snare_mixer(0.5);
        // No load on the wiper (output open), so V_out = V_wiper exactly.
        assert!(
            (cs.branches[0].gain - 0.5).abs() < 1e-6,
            "g1={}",
            cs.branches[0].gain
        );
        assert!(
            (cs.branches[1].gain - 0.5).abs() < 1e-6,
            "g2={}",
            cs.branches[1].gain
        );
    }

    #[test]
    fn blend_authority_shifts_ratio() {
        let lo = snare_mixer(0.1);
        let hi = snare_mixer(0.9);
        // pos→0 favors one branch, pos→1 the other.
        assert!(lo.branches[0].gain != hi.branches[0].gain);
        assert!(lo.branches[0].gain > lo.branches[1].gain);
        assert!(hi.branches[0].gain < hi.branches[1].gain);
    }

    #[test]
    fn gains_sum_to_one_for_lossless_crossfade() {
        for &p in &[0.0, 0.25, 0.5, 0.75, 1.0] {
            let cs = snare_mixer(p);
            let total = cs.branches[0].gain + cs.branches[1].gain;
            assert!((total - 1.0).abs() < 1e-6, "p={p} sum={total}");
        }
    }
}
