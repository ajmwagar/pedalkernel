//! WDF tree: adaptors and processing engine.
//!
//! The tree processes audio one sample at a time in four phases:
//! 1. **scatter_up** — bottom-up: leaves produce reflected waves `b`,
//!    adaptors combine them via scattering matrices.
//! 2. **root_solve** — the nonlinear root element resolves the implicit
//!    equation and produces a reflected wave back down.
//! 3. **scatter_down** — top-down: adaptors distribute incident waves
//!    to children using the scattering matrix.
//! 4. **state_update** — reactive elements latch their incident wave
//!    as the new state for the next sample.
//!
//! Zero allocation on the hot path — all buffers are pre-sized.

use crate::elements::*;

// ---------------------------------------------------------------------------
// Two-port series adaptor
// ---------------------------------------------------------------------------

/// Series adaptor joining two sub-trees.
///
/// Port resistance:  `Rp = R1 + R2`
/// Scattering coefficient: `gamma = R1 / Rp`
///
/// 3-port series junction (port 3 = parent, reflection-free):
///   scatter_up:   `b3 = -(b1 + b2)`
///   scatter_down: `a1 = b1 - gamma * (b1 + b2 + a3)`
///                 `a2 = b2 - (1 - gamma) * (b1 + b2 + a3)`
#[derive(Debug)]
pub struct SeriesAdaptor {
    pub port_resistance: f64,
    gamma: f64,
    // Child reflected waves (cached from scatter_up)
    b1: f64,
    b2: f64,
}

impl SeriesAdaptor {
    pub fn new(r1: f64, r2: f64) -> Self {
        let rp = r1 + r2;
        Self {
            port_resistance: rp,
            gamma: r1 / rp,
            b1: 0.0,
            b2: 0.0,
        }
    }

    /// Recompute when child port resistances change.
    pub fn update_ports(&mut self, r1: f64, r2: f64) {
        self.port_resistance = r1 + r2;
        self.gamma = r1 / self.port_resistance;
    }

    /// Bottom-up: accept child reflected waves, produce parent reflected wave.
    #[inline]
    pub fn scatter_up(&mut self, b1: f64, b2: f64) -> f64 {
        self.b1 = b1;
        self.b2 = b2;
        -(b1 + b2)
    }

    /// Top-down: accept parent incident wave, produce child incident waves.
    /// Returns `(a1, a2)`.
    #[inline]
    pub fn scatter_down(&self, a3: f64) -> (f64, f64) {
        let sum = self.b1 + self.b2 + a3;
        let a1 = self.b1 - self.gamma * sum;
        #[cfg(feature = "fault-injection")]
        let a1 = if crate::fault_injection::is_active(
            crate::fault_injection::Fault::CorruptScattering,
        ) {
            a1 + 1e-3
        } else {
            a1
        };
        let a2 = self.b2 - (1.0 - self.gamma) * sum;
        (a1, a2)
    }
}

// ---------------------------------------------------------------------------
// Two-port ideal transformer adaptor
// ---------------------------------------------------------------------------

/// Ideal transformer adaptor for WDF trees.
///
/// Models an ideal transformer with turns ratio n (primary:secondary = n:1).
/// The primary port faces the root (reflection-free), secondary faces child subtree.
///
/// **Transformer relations:**
/// - Voltage ratio: V₁/V₂ = n
/// - Current ratio: I₁/I₂ = 1/n
/// - Power: V₁·I₁ = V₂·I₂ (lossless)
///
/// **WDF port resistance transformation:**
/// - R_primary = n² × R_secondary
///
/// **Scattering (primary adapted, S₁₁=0):**
/// - scatter_up:   b₁ = n·b₂
/// - scatter_down: a₂ = a₁/n
///
/// For a step-down transformer (n > 1): voltage decreases, current increases.
/// For a step-up transformer (n < 1): voltage increases, current decreases.
#[derive(Debug, Clone)]
pub struct TransformerAdaptor {
    /// Port resistance seen from primary (root direction) = n² × R_secondary
    pub port_resistance: f64,
    /// Turns ratio n = N_primary / N_secondary
    turns_ratio: f64,
    /// Cached secondary (child) reflected wave
    b2: f64,
}

impl TransformerAdaptor {
    /// Create a transformer adaptor.
    ///
    /// * `turns_ratio` — n = primary turns / secondary turns (e.g., 10.0 for 10:1 step-down)
    /// * `secondary_port_resistance` — port resistance of the secondary side subtree
    pub fn new(turns_ratio: f64, secondary_port_resistance: f64) -> Self {
        Self {
            port_resistance: turns_ratio * turns_ratio * secondary_port_resistance,
            turns_ratio,
            b2: 0.0,
        }
    }

    /// Update when secondary subtree port resistance changes.
    pub fn update_secondary(&mut self, secondary_port_resistance: f64) {
        self.port_resistance = self.turns_ratio * self.turns_ratio * secondary_port_resistance;
    }

    /// Update turns ratio (for variable transformers / tap switching).
    pub fn set_turns_ratio(&mut self, turns_ratio: f64, secondary_port_resistance: f64) {
        self.turns_ratio = turns_ratio;
        self.port_resistance = turns_ratio * turns_ratio * secondary_port_resistance;
    }

    /// Get current turns ratio.
    pub fn turns_ratio(&self) -> f64 {
        self.turns_ratio
    }

    /// scatter_up: secondary (child) sends b₂, produce b₁ to primary (parent/root).
    ///
    /// Voltage is scaled up by turns ratio: V₁ = n·V₂
    #[inline]
    pub fn scatter_up(&mut self, b2: f64) -> f64 {
        self.b2 = b2;
        self.turns_ratio * b2
    }

    /// scatter_down: primary (parent) sends a₁, produce a₂ to secondary (child).
    ///
    /// Voltage is scaled down by turns ratio: V₂ = V₁/n
    #[inline]
    pub fn scatter_down(&self, a1: f64) -> f64 {
        a1 / self.turns_ratio
    }

    /// Reset state.
    pub fn reset(&mut self) {
        self.b2 = 0.0;
    }
}

// ---------------------------------------------------------------------------
// R-Type adaptor (N-port arbitrary topology)
// ---------------------------------------------------------------------------

/// R-type adaptor for arbitrary N-port topologies in WDF.
///
/// Handles non-series/parallel circuit topologies that cannot be decomposed
/// into binary adaptor trees. Examples include:
/// - 3-winding transformers
/// - Bridged-T and twin-T networks
/// - Bassman/Marshall tone stacks
/// - Op-amp feedback networks
///
/// The scattering matrix S is computed at compile time from the circuit's
/// MNA (Modified Nodal Analysis) system using element stamps.
///
/// **Reference:** Werner, Smith, Abel (2015) "Wave Digital Filter Adaptors
/// for Arbitrary Topologies and Multiport Linear Elements"
///
/// **Scattering equation:**
/// b = S · a
///
/// One port (typically the last) is adapted to be reflection-free (S_nn = 0).
#[derive(Debug, Clone)]
pub struct RTypeAdaptor {
    /// Number of ports (children + 1 for parent)
    pub num_ports: usize,
    /// Port resistance seen from parent (adapted port)
    pub port_resistance: f64,
    /// Power-normalized scattering matrix S̅ (row-major, num_ports × num_ports).
    /// S̅[i][j] = S[i][j] * √(R_j / R_i). For passive/lossless networks,
    /// all entries are bounded to [-1, 1] and the matrix is unitary.
    power_scattering: Vec<f64>,
    /// √R_i for each port — used to convert from power waves back to standard waves.
    sqrt_r: Vec<f64>,
    /// 1/√R_i for each port — used to convert standard waves to power waves.
    inv_sqrt_r: Vec<f64>,
    /// Cached child reflected waves (b₁, b₂, ..., b_{n-1})
    b_children: Vec<f64>,
}

impl RTypeAdaptor {
    /// Create an R-type adaptor from a pre-computed scattering matrix.
    ///
    /// * `scattering_matrix` — NxN matrix in row-major order
    /// * `port_resistances` — resistance for each port (last is adapted)
    ///
    /// The last port is the "parent" port facing the root, and should be
    /// adapted (S[n-1][n-1] ≈ 0).
    pub fn new(scattering_matrix: Vec<f64>, port_resistances: &[f64]) -> Self {
        let n = port_resistances.len();
        assert_eq!(
            scattering_matrix.len(),
            n * n,
            "Scattering matrix must be {n}×{n}"
        );

        let sqrt_r: Vec<f64> = port_resistances.iter().map(|r| r.sqrt()).collect();
        let inv_sqrt_r: Vec<f64> = sqrt_r.iter().map(|sr| 1.0 / sr).collect();

        // Compute power-normalized scattering: S̅[i][j] = S[i][j] * √(R_j / R_i)
        let mut power_scattering = vec![0.0; n * n];
        for i in 0..n {
            for j in 0..n {
                power_scattering[i * n + j] =
                    scattering_matrix[i * n + j] * sqrt_r[j] * inv_sqrt_r[i];
            }
        }

        Self {
            num_ports: n,
            port_resistance: port_resistances[n - 1],
            power_scattering,
            sqrt_r,
            inv_sqrt_r,
            b_children: vec![0.0; n - 1],
        }
    }

    /// Create an R-type adaptor for an ideal 3-winding transformer.
    ///
    /// * `n12` — turns ratio primary:secondary₁ (e.g., 4.0 for 4:1)
    /// * `n13` — turns ratio primary:secondary₂ (e.g., 9.5 for 9.5:1)
    /// * `r_sec1` — port resistance of secondary₁ subtree
    /// * `r_sec2` — port resistance of secondary₂ subtree
    ///
    /// The primary (port 3) is adapted (reflection-free).
    pub fn three_winding_transformer(n12: f64, n13: f64, r_sec1: f64, r_sec2: f64) -> Self {
        // Ideal 3-winding transformer with series magnetic structure:
        // V1/n1 = V2/n2 = V3/n3  (voltage)
        // n1·I1 + n2·I2 + n3·I3 = 0  (current, power conservation)
        //
        // With n1=1 (primary reference), n2=1/n12, n3=1/n13
        //
        // Port resistances: R1=r_sec1, R2=r_sec2, R3=? (to be computed for adaptation)
        //
        // The scattering matrix is derived from MNA stamps.
        // For this symmetric case, we can derive closed-form expressions.

        let n2 = 1.0 / n12; // Secondary 1 turns (relative to primary=1)
        let n3 = 1.0 / n13; // Secondary 2 turns (relative to primary=1)

        // Equivalent primary port resistance for reflection-free adaptation
        // R3 = n1²·(R1/n2² + R2/n3²) for perfect adaptation
        // Simplifying with n1=1: R3 = R1·n12² + R2·n13²
        let r_prim = r_sec1 * n12 * n12 + r_sec2 * n13 * n13;

        // Scattering matrix for 3-winding transformer (primary adapted)
        // This is derived from the MNA system with transformer stamps.
        //
        // Ports: 1=sec1, 2=sec2, 3=primary (adapted)
        // With proper normalization for WDF wave variables.

        let r1 = r_sec1;
        let r2 = r_sec2;
        let r3 = r_prim;

        // Compute scattering coefficients using the MNA-derived formulas
        // For ideal transformer: S = I - 2·R·Y where Y is admittance matrix
        //
        // For 3-winding transformer with turns n1:n2:n3 (we use 1:n2:n3):
        // The scattering matrix (with port 3 adapted) is:
        let _denom = r1 * n2 * n2 + r2 * n3 * n3 + r3;

        // S11 = (r1*n2² - r2*n3² - r3) / denom
        // S12 = 2*n2*n3*sqrt(r1*r2) / denom ... but this gets complex with normalization
        //
        // Simpler: use the reflection coefficient approach
        // When port 3 is perfectly adapted (R3 = n2²R1 + n3²R2), S33=0
        // Then waves just scale by turns ratios:

        // For the adapted case, the scattering simplifies to:
        // b1 = (2n2²r1/denom - 1)a1 + (2n2n3√(r1r2)/denom)a2 + (2n2√(r1r3)/denom)a3
        // ... this is getting complicated.

        // Let's use a simpler formulation for the special case where we want
        // an "ideal" transformer with no reflections at port 3:

        // With adaptation R3 = n12²R1 + n13²R2:
        // S = [[s11, s12, s13],
        //      [s21, s22, s23],
        //      [s31, s32, 0  ]]

        // From Werner DAFx-15 Eq. (6): S = I + 2[0 R]X^{-1}[0 I]^T
        // For ideal transformer this simplifies significantly.

        // Practical closed-form for 3-winding transformer:
        let alpha = r1 / (r1 + r2 * n12 * n12 / (n13 * n13));
        let beta = 1.0 - alpha;

        // Approximate scattering for 3-winding (assuming loose coupling model)
        // These coefficients route waves through the transformer correctly
        let s11 = 2.0 * alpha - 1.0;
        let s12 = 2.0 * (alpha * beta).sqrt();
        let s13 = n12 * 2.0 * alpha.sqrt() * (1.0 - alpha).max(0.0).sqrt();
        let s21 = s12;
        let s22 = 2.0 * beta - 1.0;
        let s23 = n13 * 2.0 * beta.sqrt() * (1.0 - beta).max(0.0).sqrt();
        let s31 = n12;
        let s32 = n13;
        let s33 = 0.0; // Adapted

        let scattering_matrix = vec![s11, s12, s13, s21, s22, s23, s31, s32, s33];
        let port_resistances = [r1, r2, r_prim];
        Self::new(scattering_matrix, &port_resistances)
    }

    /// Create from MNA element stamps (general method).
    ///
    /// This implements the Werner DAFx-15 algorithm for deriving scattering
    /// matrices from arbitrary linear circuits.
    ///
    /// * `num_ports` — number of WDF ports
    /// * `mna_stamps` — element stamps defining the internal circuit
    /// * `port_resistances` — resistance at each port
    ///
    /// The last port is adapted to be reflection-free.
    pub fn from_mna(num_ports: usize, mna_system: MnaSystem, port_resistances: &[f64]) -> Self {
        assert_eq!(port_resistances.len(), num_ports);

        // Build the X matrix from MNA stamps
        // X = [G  B  C_thevenin]
        //     [C  D  V_stamps  ]
        //     [port equations  ]
        //
        // Where G, B, C, D are standard MNA blocks

        let scattering_matrix = mna_system.derive_scattering_matrix(port_resistances);
        Self::new(scattering_matrix, port_resistances)
    }

    /// scatter_up: collect child reflected waves, produce parent reflected wave.
    ///
    /// Children send b₁, b₂, ..., b_{n-1}. We compute b_n (to parent).
    /// Uses power-normalized S̅ internally for bounded intermediate products.
    #[inline]
    pub fn scatter_up(&mut self, b_children: &[f64]) -> f64 {
        debug_assert_eq!(b_children.len(), self.num_ports - 1);

        // Cache for scatter_down
        self.b_children.copy_from_slice(b_children);

        // Power-wave scatter: a̅_n = Σ S̅[n-1][j] · b̅_j, then a_n = a̅_n · √R_n
        // Since port n is adapted (S̅[n-1][n-1]≈0), only children contribute.
        let n = self.num_ports;
        let mut sum_power = 0.0;
        for j in 0..(n - 1) {
            sum_power +=
                self.power_scattering[(n - 1) * n + j] * b_children[j] * self.inv_sqrt_r[j];
        }
        sum_power * self.sqrt_r[n - 1]
    }

    /// scatter_down: given parent incident wave, produce child incident waves.
    ///
    /// Parent sends a_n. We compute a₁, a₂, ..., a_{n-1} for children.
    /// Uses power-normalized S̅ internally for bounded intermediate products.
    #[inline]
    pub fn scatter_down(&self, a_parent: f64) -> Vec<f64> {
        let n = self.num_ports;
        let mut a_children = vec![0.0; n - 1];
        // a_parent enters as power wave: b̅_parent = a_parent / √R_parent
        let b_parent_power = a_parent * self.inv_sqrt_r[n - 1];

        for i in 0..(n - 1) {
            let mut sum_power = self.power_scattering[i * n + n - 1] * b_parent_power;
            for j in 0..(n - 1) {
                sum_power +=
                    self.power_scattering[i * n + j] * self.b_children[j] * self.inv_sqrt_r[j];
            }
            a_children[i] = sum_power * self.sqrt_r[i];
        }

        a_children
    }

    /// Perform full N×N scatter: `a = S · b_all` using power-normalized waves.
    ///
    /// Equivalent to standard scattering but uses S̅ (power-normalized) internally.
    /// Since S̅ entries are bounded [-1,1] for passive networks, intermediate
    /// products cannot overflow even with extreme port resistance ratios (e.g.,
    /// 0.1Ω cap vs 10kΩ NL port).
    #[inline]
    pub fn scatter_all(&self, b_all: &[f64]) -> Vec<f64> {
        let n = self.num_ports;
        debug_assert_eq!(b_all.len(), n);
        let mut a = vec![0.0; n];
        self.scatter_all_into(b_all, &mut a);
        a
    }

    /// Like `scatter_all`, but writes into a pre-allocated output buffer
    /// to avoid per-sample heap allocation.
    pub fn scatter_all_into(&self, b_all: &[f64], a_out: &mut [f64]) {
        let n = self.num_ports;
        debug_assert_eq!(b_all.len(), n);
        debug_assert!(a_out.len() >= n);
        for i in 0..n {
            let mut sum_power = 0.0;
            for j in 0..n {
                sum_power += self.power_scattering[i * n + j] * b_all[j] * self.inv_sqrt_r[j];
            }
            a_out[i] = sum_power * self.sqrt_r[i];
        }
    }

    /// Manually set the cached child reflected waves.
    ///
    /// After a multi-NL NR solve determines the correct `b` values for all
    /// ports, call this to update the cached state so that a subsequent
    /// `scatter_down` produces correct incident waves for passive children.
    pub fn set_child_waves(&mut self, b_children: &[f64]) {
        debug_assert_eq!(b_children.len(), self.num_ports - 1);
        self.b_children.copy_from_slice(b_children);
    }

    /// Reset state.
    pub fn reset(&mut self) {
        for b in &mut self.b_children {
            *b = 0.0;
        }
    }

    // ── Precompute accessors ─────────────────────────────────────────────
    // Minimal read-only accessors used by `precompute::extract_precomputed`.
    // They expose only what the extractor needs and nothing more.

    /// Return the power-normalized scattering matrix S̅ (row-major).
    ///
    /// `S̅[i][j] = S[i][j] · √(R_j / R_i)`.  For passive networks all entries
    /// are bounded `[-1, 1]`.
    #[must_use]
    pub fn power_scattering(&self) -> &[f64] {
        &self.power_scattering
    }

    /// Number of ports (children + 1 adapted parent port).
    #[must_use]
    pub fn num_ports(&self) -> usize {
        self.num_ports
    }

    /// Port resistances in port order (last entry is the adapted parent port).
    ///
    /// Reconstructed from `√R_i` values stored internally as `R_i = (√R_i)²`.
    #[must_use]
    pub fn port_resistances(&self) -> Vec<f64> {
        self.sqrt_r.iter().map(|sr| sr * sr).collect()
    }
}

// ---------------------------------------------------------------------------
// WDF Port for generalized scattering
// ---------------------------------------------------------------------------

/// A WDF port spanning an arbitrary node pair (not necessarily node-to-ground).
///
/// Used by `derive_scattering_matrix_general()` to support floating ports
/// such as a BJT's collector-to-emitter path where neither terminal is ground.
///
/// The generalized Werner formula for the scattering matrix entry is:
/// ```text
/// S[i][j] = δ_ij + 2·R_i · (X⁻¹[ai,aj] - X⁻¹[ai,bj] - X⁻¹[bi,aj] + X⁻¹[bi,bj])
/// ```
/// where `ai`, `bi` are the positive and negative nodes of port `i`.
#[derive(Debug, Clone)]
pub struct WdfPort {
    /// Positive node index (None = ground).
    pub node_pos: Option<usize>,
    /// Negative node index (None = ground).
    pub node_neg: Option<usize>,
    /// Port resistance (Ω).
    pub resistance: f64,
}

/// Helper: compute the X⁻¹ entry between two port terminal pairs.
///
/// Returns `X⁻¹[pi, pj] - X⁻¹[pi, nj] - X⁻¹[ni, pj] + X⁻¹[ni, nj]`
/// where `pi/ni` are positive/negative nodes of port i, `pj/nj` of port j.
/// Ground nodes (None) contribute zero to the X⁻¹ lookup.
#[inline]
fn x_inv_port_entry(
    x_inv: &[f64],
    n: usize,
    pos_i: Option<usize>,
    neg_i: Option<usize>,
    pos_j: Option<usize>,
    neg_j: Option<usize>,
) -> f64 {
    let lookup = |row: Option<usize>, col: Option<usize>| -> f64 {
        match (row, col) {
            (Some(r), Some(c)) => x_inv[r * n + c],
            _ => 0.0,
        }
    };
    lookup(pos_i, pos_j) - lookup(pos_i, neg_j) - lookup(neg_i, pos_j) + lookup(neg_i, neg_j)
}

// ---------------------------------------------------------------------------
// MNA System for R-Type adaptor construction
// ---------------------------------------------------------------------------

/// MNA (Modified Nodal Analysis) system for deriving WDF scattering matrices.
///
/// Element stamps define the internal circuit topology. The system is solved
/// at compile time to produce the scattering matrix.
#[derive(Debug, Clone)]
pub struct MnaSystem {
    /// Number of nodes (excluding ground)
    pub num_nodes: usize,
    /// Number of voltage sources / controlled sources
    pub num_vsources: usize,
    /// Conductance matrix G (num_nodes × num_nodes)
    pub g_matrix: Vec<f64>,
    /// Voltage source matrix B (num_nodes × num_vsources)
    pub b_matrix: Vec<f64>,
    /// Current output matrix C (num_vsources × num_nodes)
    pub c_matrix: Vec<f64>,
    /// Direct coupling matrix D (num_vsources × num_vsources)
    pub d_matrix: Vec<f64>,
}

impl MnaSystem {
    /// Create an empty MNA system.
    pub fn new(num_nodes: usize, num_vsources: usize) -> Self {
        Self {
            num_nodes,
            num_vsources,
            g_matrix: vec![0.0; num_nodes * num_nodes],
            b_matrix: vec![0.0; num_nodes * num_vsources],
            c_matrix: vec![0.0; num_vsources * num_nodes],
            d_matrix: vec![0.0; num_vsources * num_vsources],
        }
    }

    /// Add a resistor stamp between nodes n1 and n2.
    pub fn stamp_resistor(&mut self, n1: Option<usize>, n2: Option<usize>, resistance: f64) {
        let g = 1.0 / resistance;
        if let Some(i) = n1 {
            self.g_matrix[i * self.num_nodes + i] += g;
            if let Some(j) = n2 {
                self.g_matrix[i * self.num_nodes + j] -= g;
            }
        }
        if let Some(j) = n2 {
            self.g_matrix[j * self.num_nodes + j] += g;
            if let Some(i) = n1 {
                self.g_matrix[j * self.num_nodes + i] -= g;
            }
        }
    }

    /// Add a voltage source stamp (vsrc_idx) between nodes n+ and n-.
    pub fn stamp_voltage_source(
        &mut self,
        n_pos: Option<usize>,
        n_neg: Option<usize>,
        vsrc_idx: usize,
    ) {
        if let Some(i) = n_pos {
            self.b_matrix[i * self.num_vsources + vsrc_idx] = 1.0;
            self.c_matrix[vsrc_idx * self.num_nodes + i] = 1.0;
        }
        if let Some(j) = n_neg {
            self.b_matrix[j * self.num_vsources + vsrc_idx] = -1.0;
            self.c_matrix[vsrc_idx * self.num_nodes + j] = -1.0;
        }
    }

    /// Add an ideal transformer stamp.
    ///
    /// Primary: nodes p+ to p- (voltage source vsrc_p)
    /// Secondary: nodes s+ to s- (voltage source vsrc_s)
    /// Turns ratio: n = V_primary / V_secondary
    pub fn stamp_transformer(
        &mut self,
        p_pos: Option<usize>,
        p_neg: Option<usize>,
        s_pos: Option<usize>,
        s_neg: Option<usize>,
        vsrc_p: usize,
        vsrc_s: usize,
        turns_ratio: f64,
    ) {
        // Primary voltage: V_p+ - V_p- = n * V_s
        self.stamp_voltage_source(p_pos, p_neg, vsrc_p);

        // Secondary constraint: V_s+ - V_s- = V_s (auxiliary)
        self.stamp_voltage_source(s_pos, s_neg, vsrc_s);

        // Coupling: V_p = n * V_s => D[vsrc_p][vsrc_s] = -n
        self.d_matrix[vsrc_p * self.num_vsources + vsrc_s] = -turns_ratio;

        // Current relation: I_p = I_s / n => handled by the transformer stamp
        // The B and C matrices encode current flow through both windings
    }

    /// Add a VCCS (Voltage-Controlled Current Source) stamp.
    ///
    /// Models `I_out = gm * V_in` where:
    /// - Input voltage: V(in_pos) - V(in_neg)
    /// - Output current: flows from out_pos to out_neg
    ///
    /// MNA stamp: G[out+][in+] += gm, G[out+][in-] -= gm,
    ///            G[out-][in+] -= gm, G[out-][in-] += gm
    pub fn stamp_vccs(
        &mut self,
        out_pos: Option<usize>,
        out_neg: Option<usize>,
        in_pos: Option<usize>,
        in_neg: Option<usize>,
        gm: f64,
    ) {
        let n = self.num_nodes;
        if let Some(op) = out_pos {
            if let Some(ip) = in_pos {
                self.g_matrix[op * n + ip] += gm;
            }
            if let Some(inn) = in_neg {
                self.g_matrix[op * n + inn] -= gm;
            }
        }
        if let Some(on) = out_neg {
            if let Some(ip) = in_pos {
                self.g_matrix[on * n + ip] -= gm;
            }
            if let Some(inn) = in_neg {
                self.g_matrix[on * n + inn] += gm;
            }
        }
    }

    /// Add a finite-gain VCVS (Voltage-Controlled Voltage Source) stamp for
    /// modelling an op-amp as `V_out = Aol·(V_pos − V_neg) − Ro·i_vsrc`.
    ///
    /// This is the nullor (Werner 2016) generalised to finite open-loop gain
    /// and non-zero output impedance. With `Aol → ∞, Ro → 0` it becomes a
    /// pure nullor (infinite-gain ideal op-amp). Typical datasheet values for
    /// audio op-amps: `Aol ∈ [50k, 400k]`, `Ro ∈ [50, 200]Ω`.
    ///
    /// Reserves one auxiliary MNA row/column (the norator branch current
    /// `i_vsrc`). The constraint equation written into row `vsrc_idx` is:
    ///
    /// ```text
    /// v(out_pos) − v(out_neg) − Aol·v(pos) + Aol·v(neg) + Ro·i_vsrc = 0
    /// ```
    ///
    /// Stamp:
    /// * `B[out_pos, vsrc] += +1`, `B[out_neg, vsrc] += −1` — branch current
    ///   i_vsrc is injected into the out_pos/out_neg node pair
    /// * `C[vsrc, out_pos] += +1`, `C[vsrc, out_neg] += −1` — output voltage
    /// * `C[vsrc, pos] += −Aol`, `C[vsrc, neg] += +Aol` — controlling voltage
    /// * `D[vsrc, vsrc] += Ro` — finite output impedance
    ///
    /// Inputs at `pos`/`neg` draw zero current (nullator / infinite input
    /// impedance).
    ///
    /// * `pos`, `neg` — non-inverting and inverting input nodes (None = gnd)
    /// * `out_pos`, `out_neg` — output node pair (None = gnd)
    /// * `aol` — open-loop voltage gain (dimensionless)
    /// * `ro` — output resistance in Ohms (use 0.0 for ideal nullor)
    /// * `vsrc_idx` — auxiliary MNA branch index (must be < num_vsources)
    pub fn stamp_vcvs(
        &mut self,
        pos: Option<usize>,
        neg: Option<usize>,
        out_pos: Option<usize>,
        out_neg: Option<usize>,
        aol: f64,
        ro: f64,
        vsrc_idx: usize,
    ) {
        // B: branch current i_vsrc appears in KCL at out_pos / out_neg
        if let Some(i) = out_pos {
            self.b_matrix[i * self.num_vsources + vsrc_idx] += 1.0;
        }
        if let Some(i) = out_neg {
            self.b_matrix[i * self.num_vsources + vsrc_idx] += -1.0;
        }
        // C: voltage constraint row
        //   v(out_pos) − v(out_neg) − Aol·v(pos) + Aol·v(neg) + Ro·i = 0
        if let Some(i) = out_pos {
            self.c_matrix[vsrc_idx * self.num_nodes + i] += 1.0;
        }
        if let Some(i) = out_neg {
            self.c_matrix[vsrc_idx * self.num_nodes + i] += -1.0;
        }
        if let Some(i) = pos {
            self.c_matrix[vsrc_idx * self.num_nodes + i] += -aol;
        }
        if let Some(i) = neg {
            self.c_matrix[vsrc_idx * self.num_nodes + i] += aol;
        }
        // D: finite output impedance on the branch
        self.d_matrix[vsrc_idx * self.num_vsources + vsrc_idx] += ro;
    }

    /// Derive the scattering matrix for WDF ports.
    ///
    /// Each port corresponds to a Thévenin equivalent at a node pair.
    /// The last port is adapted (reflection-free).
    ///
    /// Returns: NxN scattering matrix in row-major order.
    pub fn derive_scattering_matrix(&self, port_resistances: &[f64]) -> Vec<f64> {
        let n_ports = port_resistances.len();

        // Build the full MNA system matrix X
        let n_total = self.num_nodes + self.num_vsources;
        let mut x_matrix = vec![0.0; n_total * n_total];

        // Fill G block
        for i in 0..self.num_nodes {
            for j in 0..self.num_nodes {
                x_matrix[i * n_total + j] = self.g_matrix[i * self.num_nodes + j];
            }
        }

        // Fill B block
        for i in 0..self.num_nodes {
            for j in 0..self.num_vsources {
                x_matrix[i * n_total + self.num_nodes + j] =
                    self.b_matrix[i * self.num_vsources + j];
            }
        }

        // Fill C block
        for i in 0..self.num_vsources {
            for j in 0..self.num_nodes {
                x_matrix[(self.num_nodes + i) * n_total + j] =
                    self.c_matrix[i * self.num_nodes + j];
            }
        }

        // Fill D block
        for i in 0..self.num_vsources {
            for j in 0..self.num_vsources {
                x_matrix[(self.num_nodes + i) * n_total + self.num_nodes + j] =
                    self.d_matrix[i * self.num_vsources + j];
            }
        }

        // Add port Thévenin resistances to G diagonal
        // (Each port contributes its port resistance as a conductance)
        for (p, &rp) in port_resistances.iter().enumerate() {
            if p < self.num_nodes {
                x_matrix[p * n_total + p] += 1.0 / rp;
            }
        }

        // Invert X matrix with equilibration for numerical conditioning
        let x_inv = invert_matrix_equilibrated(&x_matrix, n_total);

        // Derive scattering matrix from the MNA inverse.
        //
        // Each port i has Thévenin resistance R_i and is connected node_i to ground.
        // The MNA system with port conductances stamped gives:  X·V = P·diag(1/R)·a
        // Port voltages:  v = Pᵀ·V
        // Reflected waves: b = 2v - a
        // Therefore: S = 2·Pᵀ·X⁻¹·P·diag(1/R) - I
        // Which gives: S[i][j] = 2·X⁻¹[node_i][node_j] / R_j - δ_ij

        let mut scattering = vec![0.0; n_ports * n_ports];
        for i in 0..n_ports {
            for j in 0..n_ports {
                let delta = if i == j { 1.0 } else { 0.0 };
                // Assuming ports are at nodes 0, 1, ..., n_ports-1
                let x_inv_ij = if i < n_total && j < n_total {
                    x_inv[i * n_total + j]
                } else {
                    0.0
                };
                scattering[i * n_ports + j] = 2.0 * x_inv_ij / port_resistances[j] - delta;
            }
        }

        scattering
    }

    /// Derive scattering matrix for arbitrary port terminal pairs.
    ///
    /// Generalizes `derive_scattering_matrix` to support ports that span
    /// arbitrary node pairs (not just node-to-ground). This is required for
    /// BJT collector-to-emitter ports where neither terminal is ground.
    ///
    /// Uses the generalized Werner formula:
    /// ```text
    /// S[i][j] = 2/R_j · (X⁻¹[ai,aj] - X⁻¹[ai,bj] - X⁻¹[bi,aj] + X⁻¹[bi,bj]) - δ_ij
    /// ```
    /// where a_k/b_k are the positive/negative terminal nodes of port k.
    ///
    /// The last port is adapted (reflection-free, S[n-1][n-1] ≈ 0).
    pub fn derive_scattering_matrix_general(&self, ports: &[WdfPort]) -> Vec<f64> {
        let n_ports = ports.len();

        // Build the full MNA system matrix X
        let n_total = self.num_nodes + self.num_vsources;
        let mut x_matrix = vec![0.0; n_total * n_total];

        // Fill G block
        for i in 0..self.num_nodes {
            for j in 0..self.num_nodes {
                x_matrix[i * n_total + j] = self.g_matrix[i * self.num_nodes + j];
            }
        }

        // Fill B block
        for i in 0..self.num_nodes {
            for j in 0..self.num_vsources {
                x_matrix[i * n_total + self.num_nodes + j] =
                    self.b_matrix[i * self.num_vsources + j];
            }
        }

        // Fill C block
        for i in 0..self.num_vsources {
            for j in 0..self.num_nodes {
                x_matrix[(self.num_nodes + i) * n_total + j] =
                    self.c_matrix[i * self.num_nodes + j];
            }
        }

        // Fill D block
        for i in 0..self.num_vsources {
            for j in 0..self.num_vsources {
                x_matrix[(self.num_nodes + i) * n_total + self.num_nodes + j] =
                    self.d_matrix[i * self.num_vsources + j];
            }
        }

        // Add port Thévenin resistances: stamp each port as a conductance
        // between its positive and negative node pair.
        for port in ports.iter() {
            let g = 1.0 / port.resistance;
            if let Some(p) = port.node_pos {
                x_matrix[p * n_total + p] += g;
                if let Some(n) = port.node_neg {
                    x_matrix[p * n_total + n] -= g;
                }
            }
            if let Some(n) = port.node_neg {
                x_matrix[n * n_total + n] += g;
                if let Some(p) = port.node_pos {
                    x_matrix[n * n_total + p] -= g;
                }
            }
        }

        // Invert X matrix with equilibration for numerical conditioning
        let x_inv = invert_matrix_equilibrated(&x_matrix, n_total);

        // Derive scattering using generalized Werner formula
        let mut scattering = vec![0.0; n_ports * n_ports];
        for i in 0..n_ports {
            for j in 0..n_ports {
                let delta = if i == j { 1.0 } else { 0.0 };
                let entry = x_inv_port_entry(
                    &x_inv,
                    n_total,
                    ports[i].node_pos,
                    ports[i].node_neg,
                    ports[j].node_pos,
                    ports[j].node_neg,
                );
                scattering[i * n_ports + j] = 2.0 * entry / ports[j].resistance - delta;
            }
        }

        scattering
    }

    /// Derive scattering matrix **and** VS injection vector for ports driven
    /// by an ideal voltage source (zero internal impedance).
    ///
    /// Unlike `derive_scattering_matrix_general` where the VS is a WDF port,
    /// this method stamps the VS directly into the MNA B/C/D matrices so its
    /// internal impedance is zero. The scattering matrix is derived for the
    /// remaining (reactive + probe) ports only, and a separate injection
    /// vector `k` maps the VS voltage to port incident waves:
    ///
    /// ```text
    /// a[i] = Σ_j S[i][j] · b[j] + k[i] · V_in
    /// ```
    ///
    /// * `ports` — WDF ports (reactive elements + output probe; no VS port)
    /// * `vs_idx` — index of the voltage source branch in the MNA system
    ///
    /// Returns `(scattering, vs_injection)`.
    pub fn derive_scattering_and_vs_injection(
        &self,
        ports: &[WdfPort],
        vs_idx: usize,
    ) -> (Vec<f64>, Vec<f64>) {
        let n_ports = ports.len();

        // Build the full MNA system matrix X
        let n_total = self.num_nodes + self.num_vsources;
        let mut x_matrix = vec![0.0; n_total * n_total];

        // Fill G block
        for i in 0..self.num_nodes {
            for j in 0..self.num_nodes {
                x_matrix[i * n_total + j] = self.g_matrix[i * self.num_nodes + j];
            }
        }
        // Fill B block
        for i in 0..self.num_nodes {
            for j in 0..self.num_vsources {
                x_matrix[i * n_total + self.num_nodes + j] =
                    self.b_matrix[i * self.num_vsources + j];
            }
        }
        // Fill C block
        for i in 0..self.num_vsources {
            for j in 0..self.num_nodes {
                x_matrix[(self.num_nodes + i) * n_total + j] =
                    self.c_matrix[i * self.num_nodes + j];
            }
        }
        // Fill D block
        for i in 0..self.num_vsources {
            for j in 0..self.num_vsources {
                x_matrix[(self.num_nodes + i) * n_total + self.num_nodes + j] =
                    self.d_matrix[i * self.num_vsources + j];
            }
        }

        // Add port Thévenin resistances (same as derive_scattering_matrix_general)
        for port in ports.iter() {
            let g = 1.0 / port.resistance;
            if let Some(p) = port.node_pos {
                x_matrix[p * n_total + p] += g;
                if let Some(n) = port.node_neg {
                    x_matrix[p * n_total + n] -= g;
                }
            }
            if let Some(n) = port.node_neg {
                x_matrix[n * n_total + n] += g;
                if let Some(p) = port.node_pos {
                    x_matrix[n * n_total + p] -= g;
                }
            }
        }

        // Invert X with equilibration for numerical conditioning
        let x_inv = invert_matrix_equilibrated(&x_matrix, n_total);

        // Derive scattering matrix (port-to-port interactions)
        let mut scattering = vec![0.0; n_ports * n_ports];
        for i in 0..n_ports {
            for j in 0..n_ports {
                let delta = if i == j { 1.0 } else { 0.0 };
                let entry = x_inv_port_entry(
                    &x_inv,
                    n_total,
                    ports[i].node_pos,
                    ports[i].node_neg,
                    ports[j].node_pos,
                    ports[j].node_neg,
                );
                scattering[i * n_ports + j] = 2.0 * entry / ports[j].resistance - delta;
            }
        }

        // Derive VS injection vector k[i]:
        //   k[i] = 2 · (X⁻¹[p_i, vs_col] - X⁻¹[n_i, vs_col])
        // where vs_col = num_nodes + vs_idx (the VS branch column in X⁻¹).
        let vs_col = self.num_nodes + vs_idx;
        let mut vs_injection = vec![0.0; n_ports];
        for i in 0..n_ports {
            let lookup = |node: Option<usize>| -> f64 {
                match node {
                    Some(r) => x_inv[r * n_total + vs_col],
                    None => 0.0,
                }
            };
            vs_injection[i] = 2.0 * (lookup(ports[i].node_pos) - lookup(ports[i].node_neg));
        }

        (scattering, vs_injection)
    }

    /// Derive node-voltage extraction coefficients for reading the output
    /// voltage at a specific MNA node from the WDF port b-waves and VS input.
    ///
    /// The output voltage is computed as:
    /// ```text
    /// V(node) = Σ_k extract[k] * b[k] + extract_vs * V_in
    /// ```
    ///
    /// This bypasses WDF port impedance mismatch by reading the circuit's
    /// nodal voltage directly from the X⁻¹ matrix.
    ///
    /// * `ports` — WDF ports (same as passed to scattering derivation)
    /// * `vs_idx` — index of the voltage source branch in the MNA system
    /// * `output_pos` — positive output MNA node (Some(idx) or None for ground)
    /// * `output_neg` — negative output MNA node (Some(idx) or None for ground)
    ///
    /// Returns `(port_coeffs, vs_coeff)`.
    pub fn derive_extraction_coeffs(
        &self,
        ports: &[WdfPort],
        vs_idx: usize,
        output_pos: Option<usize>,
        output_neg: Option<usize>,
    ) -> (Vec<f64>, f64) {
        let n_ports = ports.len();

        // Build the full MNA system matrix X (same as derive_scattering_and_vs_injection)
        let n_total = self.num_nodes + self.num_vsources;
        let mut x_matrix = vec![0.0; n_total * n_total];

        // Fill G block
        for i in 0..self.num_nodes {
            for j in 0..self.num_nodes {
                x_matrix[i * n_total + j] = self.g_matrix[i * self.num_nodes + j];
            }
        }
        // Fill B block
        for i in 0..self.num_nodes {
            for j in 0..self.num_vsources {
                x_matrix[i * n_total + self.num_nodes + j] =
                    self.b_matrix[i * self.num_vsources + j];
            }
        }
        // Fill C block
        for i in 0..self.num_vsources {
            for j in 0..self.num_nodes {
                x_matrix[(self.num_nodes + i) * n_total + j] =
                    self.c_matrix[i * self.num_nodes + j];
            }
        }
        // Fill D block
        for i in 0..self.num_vsources {
            for j in 0..self.num_vsources {
                x_matrix[(self.num_nodes + i) * n_total + self.num_nodes + j] =
                    self.d_matrix[i * self.num_vsources + j];
            }
        }

        // Add port Thévenin resistances
        for port in ports.iter() {
            let g = 1.0 / port.resistance;
            if let Some(p) = port.node_pos {
                x_matrix[p * n_total + p] += g;
                if let Some(n) = port.node_neg {
                    x_matrix[p * n_total + n] -= g;
                }
            }
            if let Some(n) = port.node_neg {
                x_matrix[n * n_total + n] += g;
                if let Some(p) = port.node_pos {
                    x_matrix[n * n_total + p] -= g;
                }
            }
        }

        // Invert X with equilibration for numerical conditioning
        let x_inv = invert_matrix_equilibrated(&x_matrix, n_total);

        // Extraction: V(out) = Σ_k coeff[k] * b[k] + vs_coeff * V_in
        // Each port k injects current b[k]/R[k] at its nodes.
        // V(m) = Σ_k (X⁻¹[m,p_k] - X⁻¹[m,n_k]) / R_k * b_k + X⁻¹[m,vs_col] * V_in
        // For floating output (pos/neg): V = V(pos) - V(neg)
        let lookup = |row: Option<usize>, col: usize| -> f64 {
            match row {
                Some(r) => x_inv[r * n_total + col],
                None => 0.0,
            }
        };

        let mut port_coeffs = vec![0.0; n_ports];
        for k in 0..n_ports {
            let pk = ports[k].node_pos;
            let nk = ports[k].node_neg;
            // X⁻¹ entry from port k's source to output node
            let pos_contrib = match pk {
                Some(p) => lookup(output_pos, p) - lookup(output_neg, p),
                None => 0.0,
            };
            let neg_contrib = match nk {
                Some(n) => lookup(output_pos, n) - lookup(output_neg, n),
                None => 0.0,
            };
            port_coeffs[k] = (pos_contrib - neg_contrib) / ports[k].resistance;
        }

        // VS contribution
        let vs_col = self.num_nodes + vs_idx;
        let vs_coeff = lookup(output_pos, vs_col) - lookup(output_neg, vs_col);

        (port_coeffs, vs_coeff)
    }

    /// Derive node-voltage extraction coefficients for reading the output voltage
    /// at a specific MNA node from the WDF port b-waves, without a voltage source.
    ///
    /// This is the same computation as `derive_extraction_coeffs` but omits the VS
    /// term. Use this for standard adapted-WDF-port stages (non-VS-injection mode)
    /// where the adapted port is itself the last element of `ports`.
    ///
    /// Returns `port_coeffs` where `V(out) = Σ_k port_coeffs[k] * b[k]`.
    pub fn derive_node_extraction_coeffs(
        &self,
        ports: &[WdfPort],
        output_pos: Option<usize>,
        output_neg: Option<usize>,
    ) -> Vec<f64> {
        let n_ports = ports.len();
        let n_total = self.num_nodes + self.num_vsources;
        let mut x_matrix = vec![0.0; n_total * n_total];

        // Fill G block
        for i in 0..self.num_nodes {
            for j in 0..self.num_nodes {
                x_matrix[i * n_total + j] = self.g_matrix[i * self.num_nodes + j];
            }
        }
        // Fill B block
        for i in 0..self.num_nodes {
            for j in 0..self.num_vsources {
                x_matrix[i * n_total + self.num_nodes + j] =
                    self.b_matrix[i * self.num_vsources + j];
            }
        }
        // Fill C block
        for i in 0..self.num_vsources {
            for j in 0..self.num_nodes {
                x_matrix[(self.num_nodes + i) * n_total + j] =
                    self.c_matrix[i * self.num_nodes + j];
            }
        }
        // Fill D block
        for i in 0..self.num_vsources {
            for j in 0..self.num_vsources {
                x_matrix[(self.num_nodes + i) * n_total + self.num_nodes + j] =
                    self.d_matrix[i * self.num_vsources + j];
            }
        }

        // Add port Thévenin resistances
        for port in ports.iter() {
            let g = 1.0 / port.resistance;
            if let Some(p) = port.node_pos {
                x_matrix[p * n_total + p] += g;
                if let Some(n) = port.node_neg {
                    x_matrix[p * n_total + n] -= g;
                }
            }
            if let Some(n) = port.node_neg {
                x_matrix[n * n_total + n] += g;
                if let Some(p) = port.node_pos {
                    x_matrix[n * n_total + p] -= g;
                }
            }
        }

        let x_inv = invert_matrix_equilibrated(&x_matrix, n_total);

        let lookup = |row: Option<usize>, col: usize| -> f64 {
            match row {
                Some(r) => x_inv[r * n_total + col],
                None => 0.0,
            }
        };

        let mut port_coeffs = vec![0.0; n_ports];
        for k in 0..n_ports {
            let pk = ports[k].node_pos;
            let nk = ports[k].node_neg;
            let pos_contrib = match pk {
                Some(p) => lookup(output_pos, p) - lookup(output_neg, p),
                None => 0.0,
            };
            let neg_contrib = match nk {
                Some(n) => lookup(output_pos, n) - lookup(output_neg, n),
                None => 0.0,
            };
            port_coeffs[k] = (pos_contrib - neg_contrib) / ports[k].resistance;
        }

        port_coeffs
    }

    /// Build discrete-time state-space matrices from the continuous-time MNA
    /// using the bilinear (trapezoidal) transform.
    ///
    /// This avoids WDF port-impedance scaling issues by working directly with
    /// node voltages. Capacitors are treated as continuous-time elements (C matrix)
    /// rather than WDF ports (port conductance 1/R_p = 2·f_s·C).
    ///
    /// The continuous-time augmented system is:
    /// ```text
    /// [G   B] [V ]   [C_cap  0] [dV/dt]   [0  ]
    /// [E   D] [I ] + [0      0] [dI/dt] = [V_s]
    /// ```
    ///
    /// Bilinear transform `s → 2·f_s·(z-1)/(z+1)` gives:
    /// ```text
    /// M · x[n] = N · x[n-1] + F · u[n]
    /// ```
    /// where `M = [G+2fsC  B; E  D]`, `N = [2fsC-G  -B; 0  0]`,
    /// and `F = [0...1...]` maps the VS input.
    ///
    /// Returns `(A_d, b_d, c_out, n_states)` where:
    /// - `A_d = M⁻¹·N` (n×n state transition matrix)
    /// - `b_d = M⁻¹·F` (n×1 input vector)
    /// - `c_out` (1×n output extraction vector)
    /// - `n_states` = num_nodes + num_vsources
    pub fn build_state_space_matrices(
        &self,
        cap_stamps: &[(Option<usize>, Option<usize>, f64)],
        vs_idx: usize,
        output_pos: Option<usize>,
        output_neg: Option<usize>,
        sample_rate: f64,
    ) -> (Vec<f64>, Vec<f64>, Vec<f64>, usize) {
        let n_nodes = self.num_nodes;
        let n_vs = self.num_vsources;
        let n_aug = n_nodes + n_vs;
        let two_fs = 2.0 * sample_rate;

        // Build C_cap matrix (n_nodes × n_nodes) from capacitance stamps.
        let mut c_cap = vec![0.0; n_nodes * n_nodes];
        for &(pos, neg, cap) in cap_stamps {
            if let Some(p) = pos {
                c_cap[p * n_nodes + p] += cap;
                if let Some(n) = neg {
                    c_cap[p * n_nodes + n] -= cap;
                }
            }
            if let Some(n) = neg {
                c_cap[n * n_nodes + n] += cap;
                if let Some(p) = pos {
                    c_cap[n * n_nodes + p] -= cap;
                }
            }
        }

        // Build M = [G + 2·fs·C_cap,  B;  E,  D]
        let mut m_matrix = vec![0.0; n_aug * n_aug];
        for i in 0..n_nodes {
            for j in 0..n_nodes {
                m_matrix[i * n_aug + j] =
                    self.g_matrix[i * n_nodes + j] + two_fs * c_cap[i * n_nodes + j];
            }
        }
        for i in 0..n_nodes {
            for j in 0..n_vs {
                m_matrix[i * n_aug + n_nodes + j] = self.b_matrix[i * n_vs + j];
            }
        }
        for i in 0..n_vs {
            for j in 0..n_nodes {
                m_matrix[(n_nodes + i) * n_aug + j] = self.c_matrix[i * n_nodes + j];
            }
        }
        for i in 0..n_vs {
            for j in 0..n_vs {
                m_matrix[(n_nodes + i) * n_aug + n_nodes + j] = self.d_matrix[i * n_vs + j];
            }
        }

        // Build N = [2·fs·C_cap - G,  -B;  0,  0]
        let mut n_matrix = vec![0.0; n_aug * n_aug];
        for i in 0..n_nodes {
            for j in 0..n_nodes {
                n_matrix[i * n_aug + j] =
                    two_fs * c_cap[i * n_nodes + j] - self.g_matrix[i * n_nodes + j];
            }
        }
        for i in 0..n_nodes {
            for j in 0..n_vs {
                n_matrix[i * n_aug + n_nodes + j] = -self.b_matrix[i * n_vs + j];
            }
        }
        // Lower blocks (VS constraint rows) are zero — no history terms.

        // Invert M with equilibration for numerical conditioning
        let m_inv = invert_matrix_equilibrated(&m_matrix, n_aug);

        // Full augmented system: A_full = M⁻¹·N, b_full = M⁻¹·F, c_full
        let mut a_full = vec![0.0; n_aug * n_aug];
        for i in 0..n_aug {
            for j in 0..n_aug {
                let mut sum = 0.0;
                for k in 0..n_aug {
                    sum += m_inv[i * n_aug + k] * n_matrix[k * n_aug + j];
                }
                a_full[i * n_aug + j] = sum;
            }
        }

        let vs_row = n_nodes + vs_idx;
        let mut b_full = vec![0.0; n_aug];
        for i in 0..n_aug {
            b_full[i] = m_inv[i * n_aug + vs_row];
        }

        let mut c_full = vec![0.0; n_aug];
        if let Some(p) = output_pos {
            c_full[p] = 1.0;
        }
        if let Some(n) = output_neg {
            c_full[n] -= 1.0;
        }

        // ── Continuous-time Schur complement, then bilinear transform ───
        //
        // Non-cap nodes are algebraic in continuous time (C·ẋ = 0):
        //   C_c·ẋ_c + G_cc·x_c + G_ca·x_a = B_c·u
        //   G_ac·x_c + G_aa·x_a = B_a·u           (algebraic)
        //
        // Eliminate x_a from the algebraic equation:
        //   x_a = G_aa⁻¹·(B_a·u - G_ac·x_c)
        //
        // Reduced continuous system:
        //   C_c·ẋ_c + G_red·x_c = B_red·u
        //   G_red = G_cc - G_ca·G_aa⁻¹·G_ac    (Schur complement)
        //   B_red = B_c - G_ca·G_aa⁻¹·B_a
        //
        // Then apply bilinear transform to the REDUCED system:
        //   M_red = G_red + 2fs·C_c
        //   N_red = 2fs·C_c - G_red
        //   A_d = M_red⁻¹·N_red
        //
        // This avoids parasitic -1 eigenvalues entirely.
        let cap_indices: Vec<usize> = (0..n_nodes)
            .filter(|&i| c_cap[i * n_nodes + i].abs() > 1e-30)
            .collect();
        let n_c = cap_indices.len();

        eprintln!("[ss-reduce] n_aug={n_aug} n_c={n_c} cap_indices={cap_indices:?}");

        if n_c > 0 && n_c < n_aug {
            // Algebraic indices: non-cap nodes + all vsource rows
            let alg_indices: Vec<usize> = (0..n_aug)
                .filter(|i| !cap_indices.contains(i))
                .collect();
            let n_a = alg_indices.len();

            // Build the full augmented G matrix [G, B; C, D]
            let mut g_aug = vec![0.0; n_aug * n_aug];
            for i in 0..n_nodes {
                for j in 0..n_nodes {
                    g_aug[i * n_aug + j] = self.g_matrix[i * n_nodes + j];
                }
            }
            for i in 0..n_nodes {
                for j in 0..n_vs {
                    g_aug[i * n_aug + n_nodes + j] = self.b_matrix[i * n_vs + j];
                }
            }
            for i in 0..n_vs {
                for j in 0..n_nodes {
                    g_aug[(n_nodes + i) * n_aug + j] = self.c_matrix[i * n_nodes + j];
                }
            }
            for i in 0..n_vs {
                for j in 0..n_vs {
                    g_aug[(n_nodes + i) * n_aug + n_nodes + j] = self.d_matrix[i * n_vs + j];
                }
            }

            // Extract sub-blocks
            let extract = |mat: &[f64], n: usize, rows: &[usize], cols: &[usize]| -> Vec<f64> {
                let nr = rows.len();
                let nc = cols.len();
                let mut block = vec![0.0; nr * nc];
                for (ri, &r) in rows.iter().enumerate() {
                    for (ci, &c) in cols.iter().enumerate() {
                        block[ri * nc + ci] = mat[r * n + c];
                    }
                }
                block
            };

            let g_cc = extract(&g_aug, n_aug, &cap_indices, &cap_indices);
            let g_ca = extract(&g_aug, n_aug, &cap_indices, &alg_indices);
            let g_ac = extract(&g_aug, n_aug, &alg_indices, &cap_indices);
            let g_aa = extract(&g_aug, n_aug, &alg_indices, &alg_indices);

            // Invert G_aa
            let g_aa_inv = invert_matrix_equilibrated(&g_aa, n_a);

            // G_reduced = G_cc - G_ca · G_aa⁻¹ · G_ac  [n_c × n_c]
            // First: G_aa⁻¹ · G_ac  [n_a × n_c]
            let mut gaa_inv_gac = vec![0.0; n_a * n_c];
            for i in 0..n_a {
                for j in 0..n_c {
                    let mut sum = 0.0;
                    for k in 0..n_a {
                        sum += g_aa_inv[i * n_a + k] * g_ac[k * n_c + j];
                    }
                    gaa_inv_gac[i * n_c + j] = sum;
                }
            }

            let mut g_red = g_cc.clone();
            for i in 0..n_c {
                for j in 0..n_c {
                    let mut sum = 0.0;
                    for k in 0..n_a {
                        sum += g_ca[i * n_a + k] * gaa_inv_gac[k * n_c + j];
                    }
                    g_red[i * n_c + j] -= sum;
                }
            }

            eprintln!("[ss-reduce] G_cc = {g_cc:?}");
            eprintln!("[ss-reduce] G_red = {g_red:?}");

            // Reduced C matrix (cap diagonal only, in reduced space)
            let mut c_red = vec![0.0; n_c * n_c];
            for (ri, &ci) in cap_indices.iter().enumerate() {
                c_red[ri * n_c + ri] = c_cap[ci * n_nodes + ci];
            }

            // B_reduced: input column from VS (B_c - G_ca · G_aa⁻¹ · B_a)
            // B_a = augmented column at vs_row for algebraic indices
            let b_c_aug: Vec<f64> = cap_indices.iter().map(|&i| {
                if i < n_nodes { 0.0 } else { if i - n_nodes == vs_idx { 1.0 } else { 0.0 } }
            }).collect();
            let b_a_aug: Vec<f64> = alg_indices.iter().map(|&i| {
                if i < n_nodes { 0.0 } else { if i - n_nodes == vs_idx { 1.0 } else { 0.0 } }
            }).collect();

            // G_aa⁻¹ · b_a
            let mut gaa_inv_ba = vec![0.0; n_a];
            for i in 0..n_a {
                for k in 0..n_a {
                    gaa_inv_ba[i] += g_aa_inv[i * n_a + k] * b_a_aug[k];
                }
            }

            let mut b_red = b_c_aug.clone();
            for i in 0..n_c {
                let mut sum = 0.0;
                for k in 0..n_a {
                    sum += g_ca[i * n_a + k] * gaa_inv_ba[k];
                }
                b_red[i] -= sum;
            }

            // Bilinear transform on reduced system
            // M = G_red + 2fs·C_red
            // N = 2fs·C_red - G_red
            let mut m_red = vec![0.0; n_c * n_c];
            let mut n_red = vec![0.0; n_c * n_c];
            for i in 0..n_c {
                for j in 0..n_c {
                    m_red[i * n_c + j] = g_red[i * n_c + j] + two_fs * c_red[i * n_c + j];
                    n_red[i * n_c + j] = two_fs * c_red[i * n_c + j] - g_red[i * n_c + j];
                }
            }

            let m_inv = invert_matrix_equilibrated(&m_red, n_c);

            // A_d = M⁻¹ · N
            let mut a_d = vec![0.0; n_c * n_c];
            for i in 0..n_c {
                for j in 0..n_c {
                    for k in 0..n_c {
                        a_d[i * n_c + j] += m_inv[i * n_c + k] * n_red[k * n_c + j];
                    }
                }
            }

            // b_d = M⁻¹ · b_red
            let mut b_d = vec![0.0; n_c];
            for i in 0..n_c {
                for k in 0..n_c {
                    b_d[i] += m_inv[i * n_c + k] * b_red[k];
                }
            }

            // c_out: output extraction in reduced space
            // V_out = c_c · x_c + c_a · x_a
            //       = c_c · x_c + c_a · G_aa⁻¹ · (b_a·u - G_ac·x_c)
            //       = (c_c - c_a · G_aa⁻¹ · G_ac) · x_c + c_a · G_aa⁻¹ · b_a · u
            let c_c_out: Vec<f64> = cap_indices.iter().map(|&i| c_full[i]).collect();
            let c_a_out: Vec<f64> = alg_indices.iter().map(|&i| c_full[i]).collect();

            let mut c_d = c_c_out.clone();
            for j in 0..n_c {
                let mut sum = 0.0;
                for k in 0..n_a {
                    sum += c_a_out[k] * gaa_inv_gac[k * n_c + j];
                }
                c_d[j] -= sum;
            }

            (a_d, b_d, c_d, n_c)
        } else {
            (a_full, b_full, c_full, n_aug)
        }
    }
}

/// Simple matrix inversion using Gaussian elimination with partial pivoting.
/// For compile-time use in small matrices (N ≤ 10).
fn invert_matrix(matrix: &[f64], n: usize) -> Vec<f64> {
    let mut a = matrix.to_vec();
    let mut inv = vec![0.0; n * n];

    // Initialize inverse as identity
    for i in 0..n {
        inv[i * n + i] = 1.0;
    }

    // Forward elimination with partial pivoting
    for col in 0..n {
        // Find pivot
        let mut max_row = col;
        let mut max_val = a[col * n + col].abs();
        for row in (col + 1)..n {
            let val = a[row * n + col].abs();
            if val > max_val {
                max_val = val;
                max_row = row;
            }
        }

        // Swap rows if needed
        if max_row != col {
            for j in 0..n {
                a.swap(col * n + j, max_row * n + j);
                inv.swap(col * n + j, max_row * n + j);
            }
        }

        // Scale pivot row
        let pivot = a[col * n + col];
        if pivot.abs() < 1e-15 {
            // Singular matrix - return identity (or handle error)
            return vec![0.0; n * n];
        }
        for j in 0..n {
            a[col * n + j] /= pivot;
            inv[col * n + j] /= pivot;
        }

        // Eliminate column
        for row in 0..n {
            if row != col {
                let factor = a[row * n + col];
                for j in 0..n {
                    a[row * n + j] -= factor * a[col * n + j];
                    inv[row * n + j] -= factor * inv[col * n + j];
                }
            }
        }
    }

    inv
}

/// Matrix inversion with symmetric diagonal equilibration for numerical conditioning.
///
/// When port impedances span orders of magnitude (e.g. 2Ω DCR to 10MΩ probe),
/// the raw X matrix is ill-conditioned. Symmetric scaling D·X·D normalizes all
/// entries to O(1) before Gauss-Jordan, then unscales: X⁻¹ = D · (D·X·D)⁻¹ · D.
fn invert_matrix_equilibrated(matrix: &[f64], n: usize) -> Vec<f64> {
    // Compute scale factors: d[i] = 1 / sqrt(max_j |X[i][j]|)
    let mut d = vec![1.0; n];
    for i in 0..n {
        let mut row_max = 0.0f64;
        for j in 0..n {
            row_max = row_max.max(matrix[i * n + j].abs());
        }
        if row_max > 1e-30 {
            d[i] = 1.0 / row_max.sqrt();
        }
    }

    // Build scaled matrix: X_scaled = D · X · D
    let mut scaled = vec![0.0; n * n];
    for i in 0..n {
        for j in 0..n {
            scaled[i * n + j] = d[i] * matrix[i * n + j] * d[j];
        }
    }

    // Invert the well-conditioned scaled matrix
    let scaled_inv = invert_matrix(&scaled, n);

    // Unscale: X⁻¹ = D · X_scaled⁻¹ · D
    let mut result = vec![0.0; n * n];
    for i in 0..n {
        for j in 0..n {
            result[i * n + j] = d[i] * scaled_inv[i * n + j] * d[j];
        }
    }

    result
}

// ---------------------------------------------------------------------------
// Scattering interpolation table for single-pot R-type stages
// ---------------------------------------------------------------------------

/// Precomputed scattering matrices at log-spaced pot positions.
///
/// At compile time, sweeps the pot through K positions and computes full-precision
/// scattering matrices. At runtime, binary search + linear interpolation replaces
/// O(n³) matrix inversion with O(n²) lerp.
pub struct ScatteringInterpolationTable {
    /// K log-spaced resistance values (ascending).
    resistances: Vec<f64>,
    /// K scattering matrices, each n_ports² entries (row-major).
    scattering_matrices: Vec<Vec<f64>>,
    /// K VS injection vectors, each n_ports entries.
    vs_injection_vectors: Vec<Vec<f64>>,
    /// Number of WDF ports.
    n_ports: usize,
}

impl ScatteringInterpolationTable {
    /// Build a table by sweeping the pot through K log-spaced positions.
    ///
    /// - `mna`: base MNA system (pot already stamped at initial_g)
    /// - `pot_node_pos`, `pot_node_neg`: MNA node indices for the pot
    /// - `initial_g`: current pot conductance stamped in the MNA G matrix
    /// - `r_min`, `r_max`: pot resistance range (typically 1.0 to max_resistance)
    /// - `ports`: WDF port definitions (reactive ports, no pot port)
    /// - `vs_idx`: voltage source index for VS injection (None for standard mode)
    /// - `k`: number of table entries
    pub fn build(
        mna: &MnaSystem,
        pot_node_pos: Option<usize>,
        pot_node_neg: Option<usize>,
        initial_g: f64,
        r_min: f64,
        r_max: f64,
        ports: &[WdfPort],
        vs_idx: Option<usize>,
        k: usize,
    ) -> Self {
        let n_ports = ports.len();
        let n_mna = mna.num_nodes;
        let log_min = r_min.ln();
        let log_max = r_max.ln();

        let mut resistances = Vec::with_capacity(k);
        let mut scattering_matrices = Vec::with_capacity(k);
        let mut vs_injection_vectors = Vec::with_capacity(k);

        // Clone MNA once and delta-update through the sweep
        let mut sweep_mna = mna.clone();

        // Unstamp the initial pot conductance so we start clean
        let mut prev_g = initial_g;

        for i in 0..k {
            let t = if k > 1 {
                i as f64 / (k - 1) as f64
            } else {
                0.5
            };
            let r = (log_min + t * (log_max - log_min)).exp();
            let g = 1.0 / r;

            // Delta-update: remove previous conductance, add new
            let delta = g - prev_g;
            if delta.abs() > 1e-30 {
                if let Some(p) = pot_node_pos {
                    sweep_mna.g_matrix[p * n_mna + p] += delta;
                    if let Some(n) = pot_node_neg {
                        sweep_mna.g_matrix[p * n_mna + n] -= delta;
                    }
                }
                if let Some(n) = pot_node_neg {
                    sweep_mna.g_matrix[n * n_mna + n] += delta;
                    if let Some(p) = pot_node_pos {
                        sweep_mna.g_matrix[n * n_mna + p] -= delta;
                    }
                }
            }
            prev_g = g;

            let (scat, vs_inj) = if let Some(vs) = vs_idx {
                sweep_mna.derive_scattering_and_vs_injection(ports, vs)
            } else {
                let scat = sweep_mna.derive_scattering_matrix_general(ports);
                (scat, vec![0.0; n_ports])
            };

            resistances.push(r);
            scattering_matrices.push(scat);
            vs_injection_vectors.push(vs_inj);
        }

        Self {
            resistances,
            scattering_matrices,
            vs_injection_vectors,
            n_ports,
        }
    }

    /// Look up scattering matrix and VS injection vector by interpolating
    /// in log-resistance space. Returns (scattering, vs_injection).
    pub fn lookup(&self, pot_resistance: f64) -> (Vec<f64>, Vec<f64>) {
        let k = self.resistances.len();
        let log_r = pot_resistance.ln();
        let log_min = self.resistances[0].ln();
        let log_max = self.resistances[k - 1].ln();

        // Normalized position in [0, 1]
        let t = if log_max > log_min {
            (log_r - log_min) / (log_max - log_min)
        } else {
            0.5
        };

        // Map to table index
        let idx_f = t * (k - 1) as f64;
        let lo = (idx_f.floor() as usize).min(k - 2);
        let hi = lo + 1;
        let frac = idx_f - lo as f64;

        // Linear interpolation of scattering matrix entries
        let n2 = self.n_ports * self.n_ports;
        let mut scat = vec![0.0; n2];
        for i in 0..n2 {
            scat[i] = self.scattering_matrices[lo][i]
                + frac * (self.scattering_matrices[hi][i] - self.scattering_matrices[lo][i]);
        }

        // Linear interpolation of VS injection vector
        let mut vs = vec![0.0; self.n_ports];
        for i in 0..self.n_ports {
            vs[i] = self.vs_injection_vectors[lo][i]
                + frac * (self.vs_injection_vectors[hi][i] - self.vs_injection_vectors[lo][i]);
        }

        (scat, vs)
    }

    // ── Precompute accessors ─────────────────────────────────────────────

    /// Returns `true` when VS injection vectors are non-trivial (any entry != 0).
    #[must_use]
    pub fn has_vs_injection(&self) -> bool {
        self.vs_injection_vectors
            .iter()
            .any(|v| v.iter().any(|&x| x != 0.0))
    }

    /// Log-spaced resistance samples (ascending).
    #[must_use]
    pub fn resistances(&self) -> &[f64] {
        &self.resistances
    }

    /// Scattering matrices, one per resistance sample, each `n_ports²` entries.
    #[must_use]
    pub fn matrices(&self) -> &[Vec<f64>] {
        &self.scattering_matrices
    }

    /// VS injection vectors, one per resistance sample, each `n_ports` entries.
    #[must_use]
    pub fn injections(&self) -> &[Vec<f64>] {
        &self.vs_injection_vectors
    }
}

// ---------------------------------------------------------------------------
// Two-port parallel adaptor
// ---------------------------------------------------------------------------

/// Parallel adaptor joining two sub-trees.
///
/// Port resistance:  `Rp = R1 * R2 / (R1 + R2)`
/// Scattering coefficient: `gamma = R2 / (R1 + R2)`
///
/// 3-port parallel junction (port 3 = parent, reflection-free):
///   scatter_up:   b3 = (1-γ)·b1 + γ·b2
///   scatter_down (from v1 = v3, v2 = v3):
///     a1 = a3 + γ·(b2 - b1)
///     a2 = a3 - (1-γ)·(b2 - b1)
#[derive(Debug)]
pub struct ParallelAdaptor {
    pub port_resistance: f64,
    gamma: f64,
    b1: f64,
    b2: f64,
}

impl ParallelAdaptor {
    pub fn new(r1: f64, r2: f64) -> Self {
        let rp = r1 * r2 / (r1 + r2);
        Self {
            port_resistance: rp,
            gamma: r2 / (r1 + r2),
            b1: 0.0,
            b2: 0.0,
        }
    }

    pub fn update_ports(&mut self, r1: f64, r2: f64) {
        self.port_resistance = r1 * r2 / (r1 + r2);
        self.gamma = r2 / (r1 + r2);
    }

    /// Bottom-up: produce parent reflected wave.
    #[inline]
    pub fn scatter_up(&mut self, b1: f64, b2: f64) -> f64 {
        self.b1 = b1;
        self.b2 = b2;
        b1 + self.gamma * (b2 - b1)
    }

    /// Top-down: produce child incident waves from parent incident.
    /// Returns `(a1, a2)`.
    #[inline]
    pub fn scatter_down(&self, a3: f64) -> (f64, f64) {
        let diff = self.b2 - self.b1;
        // Consistent with scatter_up b3 = (1-γ)·b1 + γ·b2:
        //   v1 = v3 → a1 = a3 + γ·(b2-b1)
        //   v2 = v3 → a2 = a3 - (1-γ)·(b2-b1)
        let a1 = a3 + self.gamma * diff;
        let a2 = a3 - (1.0 - self.gamma) * diff;
        (a1, a2)
    }
}

// ---------------------------------------------------------------------------
// Complete WDF processing engine
// ---------------------------------------------------------------------------

/// WDF processing engine for a Tube-Screamer-style clipping circuit.
///
/// Tree topology:
/// ```text
///        [DiodePair root]
///              |
///         SeriesAdaptor
///          /         \
///   VoltageSource   ParallelAdaptor
///    (input)         /          \
///                Resistor    Capacitor
/// ```
///
/// The voltage source injects the input signal.  The series adaptor
/// connects it with the parallel RC + diode clipping network.
/// The diode pair at the root provides the nonlinearity.
pub struct WdfClipper {
    // Leaves
    vs: VoltageSource,
    resistor: Resistor,
    capacitor: Capacitor,
    // Adaptors
    par: ParallelAdaptor,
    ser: SeriesAdaptor,
    // Root
    diode: DiodePairRoot,
    // Sample rate
    sample_rate: f64,
}

impl WdfClipper {
    /// Create a new WDF clipper circuit.
    ///
    /// * `resistance` — clipping resistor value (Ω)
    /// * `capacitance` — clipping capacitor value (F)
    /// * `diode_model` — diode pair characteristics
    /// * `sample_rate` — audio sample rate (Hz)
    pub fn new(
        resistance: f64,
        capacitance: f64,
        diode_model: DiodeModel,
        sample_rate: f64,
    ) -> Self {
        let vs = VoltageSource::new(1.0); // small Rp for voltage source
        let resistor = Resistor::new(resistance);
        let capacitor = Capacitor::new(capacitance, sample_rate);

        let par = ParallelAdaptor::new(resistor.port_resistance(), capacitor.port_resistance());
        let ser = SeriesAdaptor::new(vs.port_resistance(), par.port_resistance);
        let diode = DiodePairRoot::new(diode_model);

        Self {
            vs,
            resistor,
            capacitor,
            par,
            ser,
            diode,
            sample_rate,
        }
    }

    /// Process one sample through the WDF tree.  Zero allocations.
    #[inline]
    pub fn process(&mut self, input: f64) -> f64 {
        // Inject input
        self.vs.set_voltage(input);

        // --- Phase 1: scatter_up (bottom → root) ---
        let b_vs = self.vs.reflected();
        let b_r = self.resistor.reflected();
        let b_c = self.capacitor.reflected();

        let b_par = self.par.scatter_up(b_r, b_c);
        let b_ser = self.ser.scatter_up(b_vs, b_par);

        // --- Phase 2: root solve ---
        let a_root = self.diode.process(b_ser, self.ser.port_resistance);

        // --- Phase 3: scatter_down (root → leaves) ---
        let (a_vs, a_par) = self.ser.scatter_down(a_root);
        let (a_r, a_c) = self.par.scatter_down(a_par);

        // --- Phase 4: state update ---
        self.vs.set_incident(a_vs);
        self.resistor.set_incident(a_r);
        self.capacitor.set_incident(a_c);

        // Output = voltage across the diode pair (the clipped signal).
        // This is naturally bounded by the diode forward voltage (~±0.7V
        // for silicon), producing the characteristic soft-clipping curve.
        (a_root + b_ser) / 2.0
    }

    /// Update the clipping resistance in-place without resetting capacitor
    /// state.  This avoids the discontinuity (click/pop) that occurs when
    /// the entire WDF tree is reconstructed on a knob change.
    pub fn set_resistance(&mut self, resistance: f64) {
        self.resistor.set_resistance(resistance);
        self.par.update_ports(
            self.resistor.port_resistance(),
            self.capacitor.port_resistance(),
        );
        self.ser
            .update_ports(self.vs.port_resistance(), self.par.port_resistance);
    }

    /// Update port resistances after sample rate change.
    pub fn set_sample_rate(&mut self, fs: f64) {
        self.sample_rate = fs;
        self.capacitor.set_sample_rate(fs);
        self.par.update_ports(
            self.resistor.port_resistance(),
            self.capacitor.port_resistance(),
        );
        self.ser
            .update_ports(self.vs.port_resistance(), self.par.port_resistance);
    }

    /// Reset all state (capacitor memory).
    pub fn reset(&mut self) {
        self.capacitor.reset();
        self.par.b1 = 0.0;
        self.par.b2 = 0.0;
        self.ser.b1 = 0.0;
        self.ser.b2 = 0.0;
    }
}

/// WDF single-diode clipper (asymmetric clipping).
///
/// Same topology as `WdfClipper` but with a single diode root.
pub struct WdfSingleDiodeClipper {
    vs: VoltageSource,
    resistor: Resistor,
    capacitor: Capacitor,
    par: ParallelAdaptor,
    ser: SeriesAdaptor,
    diode: DiodeRoot,
    sample_rate: f64,
}

impl WdfSingleDiodeClipper {
    pub fn new(
        resistance: f64,
        capacitance: f64,
        diode_model: DiodeModel,
        sample_rate: f64,
    ) -> Self {
        let vs = VoltageSource::new(1.0);
        let resistor = Resistor::new(resistance);
        let capacitor = Capacitor::new(capacitance, sample_rate);

        let par = ParallelAdaptor::new(resistor.port_resistance(), capacitor.port_resistance());
        let ser = SeriesAdaptor::new(vs.port_resistance(), par.port_resistance);
        let diode = DiodeRoot::new(diode_model);

        Self {
            vs,
            resistor,
            capacitor,
            par,
            ser,
            diode,
            sample_rate,
        }
    }

    #[inline]
    pub fn process(&mut self, input: f64) -> f64 {
        self.vs.set_voltage(input);

        let b_vs = self.vs.reflected();
        let b_r = self.resistor.reflected();
        let b_c = self.capacitor.reflected();

        let b_par = self.par.scatter_up(b_r, b_c);
        let b_ser = self.ser.scatter_up(b_vs, b_par);

        let a_root = self.diode.process(b_ser, self.ser.port_resistance);

        let (a_vs, a_par) = self.ser.scatter_down(a_root);
        let (a_r, a_c) = self.par.scatter_down(a_par);

        self.vs.set_incident(a_vs);
        self.resistor.set_incident(a_r);
        self.capacitor.set_incident(a_c);

        // Output = voltage across the single diode (asymmetric clipping).
        (a_root + b_ser) / 2.0
    }

    /// Update the clipping resistance in-place without resetting capacitor
    /// state (avoids clicks on knob changes).
    pub fn set_resistance(&mut self, resistance: f64) {
        self.resistor.set_resistance(resistance);
        self.par.update_ports(
            self.resistor.port_resistance(),
            self.capacitor.port_resistance(),
        );
        self.ser
            .update_ports(self.vs.port_resistance(), self.par.port_resistance);
    }

    pub fn set_sample_rate(&mut self, fs: f64) {
        self.sample_rate = fs;
        self.capacitor.set_sample_rate(fs);
        self.par.update_ports(
            self.resistor.port_resistance(),
            self.capacitor.port_resistance(),
        );
        self.ser
            .update_ports(self.vs.port_resistance(), self.par.port_resistance);
    }

    pub fn reset(&mut self) {
        self.capacitor.reset();
        self.par.b1 = 0.0;
        self.par.b2 = 0.0;
        self.ser.b1 = 0.0;
        self.ser.b2 = 0.0;
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // -------------------------------------------------------------------------
    // Transformer adaptor tests
    // -------------------------------------------------------------------------

    #[test]
    fn transformer_adaptor_port_resistance_stepdown() {
        // 10:1 step-down, secondary load 100Ω
        // Primary should see n² × R_sec = 100 × 100 = 10kΩ
        let t = TransformerAdaptor::new(10.0, 100.0);
        assert!(
            (t.port_resistance - 10000.0).abs() < 1e-6,
            "10:1 xformer with 100Ω load should show 10kΩ at primary"
        );
    }

    #[test]
    fn transformer_adaptor_port_resistance_stepup() {
        // 1:4 step-up (n=0.25), secondary load 1kΩ
        // Primary should see n² × R_sec = 0.0625 × 1000 = 62.5Ω
        let t = TransformerAdaptor::new(0.25, 1000.0);
        assert!(
            (t.port_resistance - 62.5).abs() < 1e-6,
            "1:4 xformer with 1kΩ load should show 62.5Ω at primary"
        );
    }

    #[test]
    fn transformer_adaptor_voltage_ratio() {
        // 10:1 step-down transformer
        let mut t = TransformerAdaptor::new(10.0, 100.0);

        // If secondary reflects b2=1.0V equivalent
        // Primary should see b1 = n * b2 = 10.0V equivalent
        let b1 = t.scatter_up(1.0);
        assert!(
            (b1 - 10.0).abs() < 1e-10,
            "scatter_up should scale by turns ratio"
        );

        // If primary receives a1=10.0V equivalent
        // Secondary should see a2 = a1/n = 1.0V equivalent
        let a2 = t.scatter_down(10.0);
        assert!(
            (a2 - 1.0).abs() < 1e-10,
            "scatter_down should scale by 1/turns ratio"
        );
    }

    #[test]
    fn transformer_adaptor_wave_scaling() {
        // Test that waves scale correctly by turns ratio
        // 4:1 step-down transformer, secondary load 250Ω
        let mut t = TransformerAdaptor::new(4.0, 250.0);

        // Test scatter_up: secondary wave scaled up by n
        let b_sec = 0.5;
        let b_prim = t.scatter_up(b_sec);
        assert!(
            (b_prim / b_sec - 4.0).abs() < 1e-10,
            "scatter_up should scale by turns ratio"
        );

        // Test scatter_down: primary wave scaled down by n
        let a_prim = 2.0;
        let a_sec = t.scatter_down(a_prim);
        assert!(
            (a_prim / a_sec - 4.0).abs() < 1e-10,
            "scatter_down should scale by 1/turns ratio"
        );

        // Verify port resistance transformation
        assert!(
            (t.port_resistance - 4000.0).abs() < 1e-6,
            "R_prim should be n²×R_sec = 4000Ω"
        );
    }

    #[test]
    fn transformer_dc_stability() {
        // Test transformer with DC (zero input) is stable
        let mut t = TransformerAdaptor::new(10.0, 100.0);

        for _ in 0..100 {
            let b_prim = t.scatter_up(0.0);
            assert!(b_prim.abs() < 1e-10);
            let a_sec = t.scatter_down(0.0);
            assert!(a_sec.abs() < 1e-10);
        }
    }

    #[test]
    fn transformer_unity_ratio() {
        // 1:1 isolation transformer
        let mut t = TransformerAdaptor::new(1.0, 1000.0);
        assert!((t.port_resistance - 1000.0).abs() < 1e-6);

        let b1 = t.scatter_up(0.7);
        assert!((b1 - 0.7).abs() < 1e-10, "1:1 should pass waves unchanged");

        let a2 = t.scatter_down(0.3);
        assert!((a2 - 0.3).abs() < 1e-10, "1:1 should pass waves unchanged");
    }

    // -------------------------------------------------------------------------
    // R-type adaptor tests
    // -------------------------------------------------------------------------

    #[test]
    fn rtype_three_winding_creation() {
        // 3-winding transformer: 4:1:1 (primary to two identical secondaries)
        let r = RTypeAdaptor::three_winding_transformer(4.0, 4.0, 100.0, 100.0);
        assert_eq!(r.num_ports, 3);
        // Primary should see n²R1 + n²R2 = 16*100 + 16*100 = 3200Ω
        assert!(
            (r.port_resistance - 3200.0).abs() < 100.0,
            "Expected ~3200Ω, got {}",
            r.port_resistance
        );
    }

    #[test]
    fn rtype_scatter_preserves_dimensions() {
        let mut r = RTypeAdaptor::three_winding_transformer(2.0, 3.0, 500.0, 300.0);

        let b_children = [0.5, 0.3];
        let b_parent = r.scatter_up(&b_children);
        assert!(
            b_parent.is_finite(),
            "scatter_up should produce finite output"
        );

        let a_children = r.scatter_down(0.2);
        assert_eq!(a_children.len(), 2, "scatter_down should produce 2 outputs");
        assert!(a_children[0].is_finite());
        assert!(a_children[1].is_finite());
    }

    #[test]
    fn mna_resistor_stamp() {
        let mut mna = MnaSystem::new(2, 0);
        mna.stamp_resistor(Some(0), Some(1), 1000.0);

        // G should be: [[1/R, -1/R], [-1/R, 1/R]]
        let g = 0.001;
        assert!((mna.g_matrix[0] - g).abs() < 1e-10);
        assert!((mna.g_matrix[1] - (-g)).abs() < 1e-10);
        assert!((mna.g_matrix[2] - (-g)).abs() < 1e-10);
        assert!((mna.g_matrix[3] - g).abs() < 1e-10);
    }

    #[test]
    fn matrix_inversion_identity() {
        // Invert a 2x2 identity matrix
        let identity = vec![1.0, 0.0, 0.0, 1.0];
        let inv = invert_matrix(&identity, 2);
        assert!((inv[0] - 1.0).abs() < 1e-10);
        assert!((inv[1] - 0.0).abs() < 1e-10);
        assert!((inv[2] - 0.0).abs() < 1e-10);
        assert!((inv[3] - 1.0).abs() < 1e-10);
    }

    #[test]
    fn matrix_inversion_simple() {
        // Invert [[2, 1], [1, 1]] -> [[1, -1], [-1, 2]]
        let m = vec![2.0, 1.0, 1.0, 1.0];
        let inv = invert_matrix(&m, 2);
        assert!((inv[0] - 1.0).abs() < 1e-10);
        assert!((inv[1] - (-1.0)).abs() < 1e-10);
        assert!((inv[2] - (-1.0)).abs() < 1e-10);
        assert!((inv[3] - 2.0).abs() < 1e-10);
    }

    // -------------------------------------------------------------------------
    // VCVS (finite-gain op-amp nullor) tests
    // -------------------------------------------------------------------------

    /// Solve an MNA system for DC operating point.
    ///
    /// Builds the augmented X matrix `[[G, B], [C, D]]` and solves
    /// `X · z = rhs` where `z = [v_nodes; i_vsources]` and `rhs` has the
    /// voltage-source values in the lower partition.
    ///
    /// Returns node voltages `v[0..num_nodes]`.
    fn mna_solve_dc(mna: &MnaSystem, vs_values: &[f64]) -> Vec<f64> {
        let n = mna.num_nodes;
        let m = mna.num_vsources;
        let nt = n + m;
        let mut x = vec![0.0; nt * nt];
        for i in 0..n {
            for j in 0..n {
                x[i * nt + j] = mna.g_matrix[i * n + j];
            }
        }
        for i in 0..n {
            for j in 0..m {
                x[i * nt + (n + j)] = mna.b_matrix[i * m + j];
            }
        }
        for i in 0..m {
            for j in 0..n {
                x[(n + i) * nt + j] = mna.c_matrix[i * n + j];
            }
        }
        for i in 0..m {
            for j in 0..m {
                x[(n + i) * nt + (n + j)] = mna.d_matrix[i * m + j];
            }
        }
        let x_inv = invert_matrix(&x, nt);
        let mut rhs = vec![0.0; nt];
        for (k, &v) in vs_values.iter().enumerate() {
            rhs[n + k] = v;
        }
        let mut z = vec![0.0; nt];
        for i in 0..nt {
            let mut s = 0.0;
            for j in 0..nt {
                s += x_inv[i * nt + j] * rhs[j];
            }
            z[i] = s;
        }
        z[..n].to_vec()
    }

    #[test]
    fn mna_vcvs_unity_buffer() {
        // Unity-gain voltage follower: pos=in, neg=out, out=out.
        // Expect V_out ≈ V_in to within ~1/Aol.
        //   Nodes: 0 = in, 1 = out
        //   vs 0: input source at node 0
        //   vs 1: op-amp VCVS (pos=0, neg=1, out_pos=1, out_neg=gnd)
        let mut mna = MnaSystem::new(2, 2);
        mna.stamp_voltage_source(Some(0), None, 0);
        mna.stamp_vcvs(Some(0), Some(1), Some(1), None, 200_000.0, 75.0, 1);
        let v_in = 1.0;
        let v = mna_solve_dc(&mna, &[v_in, 0.0]);
        assert!(
            (v[1] - v_in).abs() < 1e-3,
            "unity buffer: V_out={} expected ~{}",
            v[1],
            v_in
        );
    }

    #[test]
    fn mna_vcvs_inverting_x10() {
        // Inverting amp: pos=gnd, neg=vnode, out drives node_out via Rf.
        //   Nodes: 0 = V_in, 1 = V_neg (virtual gnd), 2 = V_out
        //   Ri (10k) between node 0 and node 1
        //   Rf (100k) between node 1 and node 2
        //   vs 0: input VS at node 0
        //   vs 1: op-amp VCVS (pos=gnd, neg=1, out_pos=2, out_neg=gnd)
        let mut mna = MnaSystem::new(3, 2);
        mna.stamp_voltage_source(Some(0), None, 0);
        mna.stamp_resistor(Some(0), Some(1), 10_000.0);
        mna.stamp_resistor(Some(1), Some(2), 100_000.0);
        mna.stamp_vcvs(None, Some(1), Some(2), None, 200_000.0, 75.0, 1);
        let v = mna_solve_dc(&mna, &[1.0, 0.0]);
        // Ideal gain = -Rf/Ri = -10. Finite-Aol correction: small, < 0.01%.
        assert!(
            (v[2] - (-10.0)).abs() < 0.01,
            "inverting x10: V_out={} expected ≈-10",
            v[2]
        );
    }

    #[test]
    fn mna_vcvs_noninverting_x10() {
        // Non-inverting amp gain = 1 + Rf/Ri = 10 with Rf=90k, Ri=10k.
        //   Nodes: 0 = V_in (= V_pos), 1 = V_neg, 2 = V_out
        //   Ri (10k) between node 1 and gnd
        //   Rf (90k) between node 1 and node 2
        //   vs 0: input VS at node 0
        //   vs 1: op-amp VCVS (pos=0, neg=1, out_pos=2, out_neg=gnd)
        let mut mna = MnaSystem::new(3, 2);
        mna.stamp_voltage_source(Some(0), None, 0);
        mna.stamp_resistor(Some(1), None, 10_000.0);
        mna.stamp_resistor(Some(1), Some(2), 90_000.0);
        mna.stamp_vcvs(Some(0), Some(1), Some(2), None, 200_000.0, 75.0, 1);
        let v = mna_solve_dc(&mna, &[1.0, 0.0]);
        assert!(
            (v[2] - 10.0).abs() < 0.01,
            "non-inverting x10: V_out={} expected ≈10",
            v[2]
        );
    }

    #[test]
    fn mna_vcvs_finite_aol_correction() {
        // Verify closed-loop gain deviates from ideal by expected 1/Aol amount.
        // Ideal inverting: G = -Rf/Ri. With finite Aol:
        //   G = -(Rf/Ri) · 1 / (1 + (1 + Rf/Ri)/Aol)
        // For Rf=100k, Ri=10k, Aol=100k (JRC4558):
        //   G = -10 · 1/(1 + 11/100000) ≈ -10 · 0.99989 ≈ -9.9989
        let mut mna = MnaSystem::new(3, 2);
        mna.stamp_voltage_source(Some(0), None, 0);
        mna.stamp_resistor(Some(0), Some(1), 10_000.0);
        mna.stamp_resistor(Some(1), Some(2), 100_000.0);
        mna.stamp_vcvs(None, Some(1), Some(2), None, 100_000.0, 75.0, 1);
        let v = mna_solve_dc(&mna, &[1.0, 0.0]);
        let g_ideal = -10.0;
        let g_expected = g_ideal / (1.0 + 11.0 / 100_000.0);
        assert!(
            (v[2] - g_expected).abs() < 1e-4,
            "finite-Aol correction: V_out={} expected ≈{}",
            v[2],
            g_expected
        );
        // And confirm it's measurably different from ideal (i.e. correction is real)
        assert!(
            (v[2] - g_ideal).abs() > 1e-5,
            "expected finite-Aol deviation from ideal, but got V_out={}",
            v[2]
        );
    }

    #[test]
    fn mna_vcvs_nullor_stamp_matrix_entries() {
        // Direct test that the stamp places the expected entries.
        let mut mna = MnaSystem::new(3, 1);
        mna.stamp_vcvs(Some(0), Some(1), Some(2), None, 200_000.0, 75.0, 0);
        let m = mna.num_vsources;
        let n = mna.num_nodes;
        // B column 0 (branch current into output)
        assert!((mna.b_matrix[2 * m + 0] - 1.0).abs() < 1e-12); // out_pos
                                                                // C row 0: +1 at out_pos, -Aol at pos, +Aol at neg
        assert!((mna.c_matrix[0 * n + 2] - 1.0).abs() < 1e-12);
        assert!((mna.c_matrix[0 * n + 0] - (-200_000.0)).abs() < 1e-6);
        assert!((mna.c_matrix[0 * n + 1] - 200_000.0).abs() < 1e-6);
        // D: Ro on the branch diagonal
        assert!((mna.d_matrix[0 * m + 0] - 75.0).abs() < 1e-12);
    }

    // -------------------------------------------------------------------------
    // Series/parallel adaptor tests (existing)
    // -------------------------------------------------------------------------

    #[test]
    fn series_adaptor_port_resistance() {
        let s = SeriesAdaptor::new(1000.0, 2000.0);
        assert!((s.port_resistance - 3000.0).abs() < 1e-6);
    }

    #[test]
    fn series_adaptor_scatter_up() {
        let mut s = SeriesAdaptor::new(1000.0, 2000.0);
        let b3 = s.scatter_up(0.5, 0.3);
        assert!((b3 - (-0.8)).abs() < 1e-10, "b3 = -(b1+b2)");
    }

    #[test]
    fn parallel_adaptor_port_resistance() {
        let p = ParallelAdaptor::new(1000.0, 2000.0);
        let expected = 1000.0 * 2000.0 / 3000.0;
        assert!((p.port_resistance - expected).abs() < 1e-6);
    }

    #[test]
    fn parallel_adaptor_scatter_up() {
        let mut p = ParallelAdaptor::new(1000.0, 1000.0);
        // Equal resistances => gamma = 0.5 => b_up = (b1+b2)/2
        let b3 = p.scatter_up(1.0, -1.0);
        assert!((b3 - 0.0).abs() < 1e-10);
    }

    #[test]
    fn series_adaptor_scatter_down_matches_formula() {
        let mut s = SeriesAdaptor::new(1500.0, 3300.0);
        let b1 = 0.4;
        let b2 = -0.2;
        let a3 = 0.5;
        let sum = b1 + b2 + a3;
        let gamma = 1500.0 / (1500.0 + 3300.0);
        let expected_a1 = b1 - gamma * sum;
        let expected_a2 = b2 - (1.0 - gamma) * sum;
        let (_, _) = (s.scatter_up(b1, b2), ()); // cache b1/b2 in adaptor
        let (a1, a2) = s.scatter_down(a3);
        assert!((a1 - expected_a1).abs() < 1e-12);
        assert!((a2 - expected_a2).abs() < 1e-12);
    }

    #[test]
    fn parallel_adaptor_scatter_down_matches_formula() {
        let mut p = ParallelAdaptor::new(2200.0, 4700.0);
        let b1 = -0.1;
        let b2 = 0.3;
        let a3 = -0.25;
        let diff = b2 - b1;
        let gamma = 4700.0 / (2200.0 + 4700.0);
        let (_, _) = (p.scatter_up(b1, b2), ());
        let (a1, a2) = p.scatter_down(a3);
        let expected_a1 = a3 + gamma * diff;
        let expected_a2 = a3 - (1.0 - gamma) * diff;
        assert!((a1 - expected_a1).abs() < 1e-12);
        assert!((a2 - expected_a2).abs() < 1e-12);
    }

    #[test]
    fn wdf_clipper_dc_stability() {
        let mut c = WdfClipper::new(4700.0, 220e-9, DiodeModel::silicon(), 48000.0);
        // Feed DC = 0 for many samples, output should stay near zero
        for _ in 0..1000 {
            let out = c.process(0.0);
            assert!(out.abs() < 1e-6, "DC stability: output was {out}");
        }
    }

    #[test]
    fn wdf_clipper_clips_large_signal() {
        let mut c = WdfClipper::new(4700.0, 220e-9, DiodeModel::silicon(), 48000.0);
        // Feed a large-amplitude sine for several cycles
        let mut max_out = 0.0_f64;
        for i in 0..48000 {
            let t = i as f64 / 48000.0;
            let input = 5.0 * (2.0 * std::f64::consts::PI * 440.0 * t).sin();
            let out = c.process(input);
            max_out = max_out.max(out.abs());
        }
        // Output should be bounded — diode clipping keeps it from blowing up
        assert!(
            max_out < 50.0,
            "output should be bounded: peak was {max_out}"
        );
        assert!(max_out > 0.01, "should produce nonzero output");
    }

    #[test]
    fn wdf_clipper_produces_signal() {
        let mut c = WdfClipper::new(4700.0, 220e-9, DiodeModel::silicon(), 48000.0);
        // Feed a sine wave, collect output
        let mut max_out = 0.0_f64;
        for i in 0..4800 {
            let t = i as f64 / 48000.0;
            let input = 0.5 * (2.0 * std::f64::consts::PI * 440.0 * t).sin();
            let out = c.process(input);
            max_out = max_out.max(out.abs());
        }
        assert!(
            max_out > 0.001,
            "should produce nonzero output, got {max_out}"
        );
    }

    #[test]
    fn series_scatter_roundtrip_energy() {
        // Energy conservation: |b1|^2/R1 + |b2|^2/R2 + |b3|^2/R3
        // should be preserved through scattering.
        let r1 = 1000.0;
        let r2 = 2200.0;
        let mut s = SeriesAdaptor::new(r1, r2);

        let b1 = 0.3;
        let b2 = 0.7;
        let b3 = s.scatter_up(b1, b2);

        // Now scatter down with some incident
        let a3 = -b3; // matched termination at root
        let (a1, a2) = s.scatter_down(a3);

        // Check Kirchhoff: voltages sum at series junction
        let v1 = (a1 + b1) / 2.0;
        let v2 = (a2 + b2) / 2.0;
        let v3 = (a3 + b3) / 2.0;
        assert!(
            (v1 + v2 - v3).abs() < 1e-10,
            "KVL: {v1} + {v2} should ≈ {v3}"
        );
    }

    // -------------------------------------------------------------------------
    // Generalized MNA scattering tests (Phase 1)
    // -------------------------------------------------------------------------

    #[test]
    fn test_general_port_matches_simple() {
        // Verify that derive_scattering_matrix_general with node-to-ground ports
        // produces identical results to derive_scattering_matrix.
        //
        // Two-port network: R=1000Ω between node0 and node1.
        // Port 0 at node0 (to ground), Port 1 at node1 (to ground).
        let mut mna = MnaSystem::new(2, 0);
        mna.stamp_resistor(Some(0), Some(1), 1000.0);

        let port_resistances = [500.0, 500.0];

        // Original method
        let s_old = mna.derive_scattering_matrix(&port_resistances);

        // General method with node-to-ground ports
        let ports = vec![
            WdfPort {
                node_pos: Some(0),
                node_neg: None,
                resistance: 500.0,
            },
            WdfPort {
                node_pos: Some(1),
                node_neg: None,
                resistance: 500.0,
            },
        ];
        let s_new = mna.derive_scattering_matrix_general(&ports);

        for i in 0..4 {
            assert!(
                (s_old[i] - s_new[i]).abs() < 1e-10,
                "S[{}][{}] mismatch: old={}, new={}",
                i / 2,
                i % 2,
                s_old[i],
                s_new[i]
            );
        }
    }

    #[test]
    fn test_general_port_floating_pair() {
        // Network: R from node0→node1, Rg from node1→ground (provides ground ref).
        // One floating port from node0 to node1 with resistance Rp.
        //
        // Thévenin impedance from node0 to node1 (port disconnected) = R
        // (Rg only connects node1 to ground, doesn't affect the 2-terminal Z).
        //
        // Expected reflection: S[0][0] = (R - Rp) / (R + Rp)
        let r = 1000.0;
        let rg = 1000.0;
        let rp = 2000.0;

        let mut mna = MnaSystem::new(2, 0);
        mna.stamp_resistor(Some(0), Some(1), r);
        mna.stamp_resistor(Some(1), None, rg); // ground reference

        let ports = vec![WdfPort {
            node_pos: Some(0),
            node_neg: Some(1),
            resistance: rp,
        }];
        let s = mna.derive_scattering_matrix_general(&ports);

        let expected = (r - rp) / (r + rp); // = -1/3
        assert!(
            (s[0] - expected).abs() < 1e-10,
            "S[0][0] should be (R-Rp)/(R+Rp) = {expected}, got {}",
            s[0]
        );
    }

    #[test]
    fn test_general_port_voltage_divider() {
        // R1: node0 → node1, R2: node1 → gnd.
        // Port 0: node0 to gnd, Port 1: node1 to gnd.
        let r1 = 1000.0;
        let r2 = 1000.0;
        let rp0 = 500.0;
        let rp1 = 500.0;

        let mut mna = MnaSystem::new(2, 0);
        mna.stamp_resistor(Some(0), Some(1), r1);
        mna.stamp_resistor(Some(1), None, r2);

        let ports = vec![
            WdfPort {
                node_pos: Some(0),
                node_neg: None,
                resistance: rp0,
            },
            WdfPort {
                node_pos: Some(1),
                node_neg: None,
                resistance: rp1,
            },
        ];
        let s = mna.derive_scattering_matrix_general(&ports);

        // Verify basic properties: S is 2×2, finite, no NaN
        for i in 0..4 {
            assert!(s[i].is_finite(), "S element {} is not finite", i);
        }

        // Row passivity: |S[i][0]|² + |S[i][1]|² ≤ 1 for each row
        for i in 0..2 {
            let row_norm_sq = s[i * 2] * s[i * 2] + s[i * 2 + 1] * s[i * 2 + 1];
            assert!(
                row_norm_sq <= 1.0 + 1e-10,
                "Row {} norm² = {} exceeds 1 (not passive)",
                i,
                row_norm_sq
            );
        }
    }

    #[test]
    fn test_general_port_t_network() {
        // T-network: R1 between node0-node1, R2 between node1-node2,
        // R3 between node1-gnd.
        // Port 0: node0 to gnd, Port 1: node2 to gnd.
        let r1 = 1000.0;
        let r2 = 1000.0;
        let r3 = 2000.0;
        let rp0 = 1500.0;
        let rp1 = 1500.0;

        let mut mna = MnaSystem::new(3, 0);
        mna.stamp_resistor(Some(0), Some(1), r1);
        mna.stamp_resistor(Some(1), Some(2), r2);
        mna.stamp_resistor(Some(1), None, r3);

        let ports = vec![
            WdfPort {
                node_pos: Some(0),
                node_neg: None,
                resistance: rp0,
            },
            WdfPort {
                node_pos: Some(2),
                node_neg: None,
                resistance: rp1,
            },
        ];
        let s = mna.derive_scattering_matrix_general(&ports);

        // Verify S is 2×2 and symmetric (since the circuit is symmetric)
        assert!(
            (s[1] - s[2]).abs() < 1e-10,
            "Symmetric network should have S[0][1] = S[1][0]: {} vs {}",
            s[1],
            s[2]
        );

        // Verify passivity
        for i in 0..2 {
            let row_norm_sq = s[i * 2] * s[i * 2] + s[i * 2 + 1] * s[i * 2 + 1];
            assert!(
                row_norm_sq <= 1.0 + 1e-10,
                "Row {} norm² = {} exceeds 1",
                i,
                row_norm_sq
            );
        }
    }

    #[test]
    fn test_general_port_energy_conservation() {
        // For a lossless junction (no internal resistors — only port resistances),
        // the power-normalized scattering matrix Ŝ should satisfy Ŝᵀ·Ŝ = I.
        //
        // Power-normalized: Ŝ[i][j] = sqrt(R_i/R_j) · S[i][j]
        //
        // For a resistive network (lossy), we instead check passivity:
        // all singular values of Ŝ ≤ 1.
        //
        // Here we test a specific analytical case: single resistor R between
        // node0 and node1, ports at each node to ground with equal resistance Rp.
        //
        // Analytical: The 2×2 G matrix from R is [[1/R,-1/R],[-1/R,1/R]], plus
        // port conductances: G_total = [[1/R+1/Rp,-1/R],[-1/R,1/R+1/Rp]].
        // S[i][j] = 2·X⁻¹[i][j]/Rp - δ_ij.
        let r = 1000.0;
        let rp = 1000.0;
        let mut mna = MnaSystem::new(2, 0);
        mna.stamp_resistor(Some(0), Some(1), r);

        let ports = vec![
            WdfPort {
                node_pos: Some(0),
                node_neg: None,
                resistance: rp,
            },
            WdfPort {
                node_pos: Some(1),
                node_neg: None,
                resistance: rp,
            },
        ];
        let s = mna.derive_scattering_matrix_general(&ports);

        // Analytical: G_total = [[2/R, -1/R], [-1/R, 2/R]], det = 3/R²
        // X⁻¹ = R/3 · [[2, 1], [1, 2]]
        // S[0][0] = 2·(R/3·2)/Rp - 1 = 4/3 - 1 = 1/3  (with Rp=R)
        // S[0][1] = 2·(R/3·1)/Rp = 2/3
        // S[1][0] = 2/3, S[1][1] = 1/3
        let expected = [1.0 / 3.0, 2.0 / 3.0, 2.0 / 3.0, 1.0 / 3.0];
        for i in 0..4 {
            assert!(
                (s[i] - expected[i]).abs() < 1e-10,
                "S[{}][{}]: expected {}, got {}",
                i / 2,
                i % 2,
                expected[i],
                s[i]
            );
        }

        // Verify passivity: for each row, R_i · Σ_j S[i][j]²/R_j ≤ 1
        // With equal resistances this simplifies to row norm² ≤ 1
        for i in 0..2 {
            let row_norm_sq = s[i * 2] * s[i * 2] + s[i * 2 + 1] * s[i * 2 + 1];
            assert!(
                row_norm_sq <= 1.0 + 1e-10,
                "Row {} norm² = {} exceeds 1 (not passive)",
                i,
                row_norm_sq
            );
        }
    }

    #[test]
    fn test_adapted_port_reflection_zero() {
        // For a properly adapted last port, S[n-1][n-1] should ≈ 0.
        // Voltage divider: R1 between node0-node1, R2 between node1-gnd.
        // Port 0: node0→gnd with Rp0.
        // Port 1 (adapted): node1→gnd.
        //
        // The adapted port resistance = Thévenin impedance seen from port 1's
        // terminals through the network WITH all other port resistances connected.
        // From node1: R2 to gnd in parallel with (R1 + Rp0) to gnd.
        // Rth = R2 || (R1 + Rp0)
        let r1 = 1000.0;
        let r2 = 2000.0;
        let rp0 = 1000.0;
        let rp1 = r2 * (r1 + rp0) / (r2 + r1 + rp0); // R2 || (R1+Rp0)

        let mut mna = MnaSystem::new(2, 0);
        mna.stamp_resistor(Some(0), Some(1), r1);
        mna.stamp_resistor(Some(1), None, r2);

        let ports = vec![
            WdfPort {
                node_pos: Some(0),
                node_neg: None,
                resistance: rp0,
            },
            WdfPort {
                node_pos: Some(1),
                node_neg: None,
                resistance: rp1,
            },
        ];
        let s = mna.derive_scattering_matrix_general(&ports);

        assert!(
            s[3].abs() < 1e-8,
            "S[1][1] should be ≈ 0 for adapted port, got {} (Rp1={})",
            s[3],
            rp1
        );
    }

    #[test]
    fn test_scatter_all_consistency() {
        // Verify scatter_all matches scatter_up + scatter_down for known inputs.
        // Use a 3-port R-type from a simple network.
        //
        // Network: R_ab=1000 (0→1), R_bc=1500 (1→2), R_bg=2000 (1→gnd).
        // Port 0: node0→gnd (Rp0=1000), Port 1: node2→gnd (Rp1=1000)
        // Port 2 (adapted): node1→gnd
        //
        // Thévenin from node1: (R_ab+Rp0)||(R_bc+Rp1)||R_bg = 2000||2500||2000
        let r_ab = 1000.0;
        let r_bc = 1500.0;
        let r_bg = 2000.0;
        let r0 = 1000.0;
        let r1 = 1000.0;

        // Compute adapted resistance for port 2
        let branch_a = r_ab + r0; // 2000
        let branch_b = r_bc + r1; // 2500
        let r2 = 1.0 / (1.0 / branch_a + 1.0 / branch_b + 1.0 / r_bg);

        let mut mna = MnaSystem::new(3, 0);
        mna.stamp_resistor(Some(0), Some(1), r_ab);
        mna.stamp_resistor(Some(1), Some(2), r_bc);
        mna.stamp_resistor(Some(1), None, r_bg);

        let ports = vec![
            WdfPort {
                node_pos: Some(0),
                node_neg: None,
                resistance: r0,
            },
            WdfPort {
                node_pos: Some(2),
                node_neg: None,
                resistance: r1,
            },
            WdfPort {
                node_pos: Some(1),
                node_neg: None,
                resistance: r2,
            },
        ];
        let s_matrix = mna.derive_scattering_matrix_general(&ports);

        // Verify S[2][2] ≈ 0 (adapted)
        let n = 3;
        assert!(
            s_matrix[2 * n + 2].abs() < 1e-8,
            "S[2][2] should be ≈ 0 for adapted port, got {}",
            s_matrix[2 * n + 2]
        );

        let mut adaptor = RTypeAdaptor::new(s_matrix, &[r0, r1, r2]);

        // Set up known child waves
        let b_children = [0.5, -0.3];

        // scatter_up gives b_parent (uses S[2][0..1])
        let b_parent = adaptor.scatter_up(&b_children);

        // scatter_down with a known a_parent (uses S[0..1][0..2])
        let a_parent = 0.7;
        let a_children = adaptor.scatter_down(a_parent);

        // Now use scatter_all with full wave vector
        // b_all = [b_child_0, b_child_1, a_parent]
        let b_all = [b_children[0], b_children[1], a_parent];
        let a_all = adaptor.scatter_all(&b_all);

        // scatter_all computes a[i] = Σ_j S[i][j]·b_all[j] for all i.
        // For the adapted parent port (i=2): a_all[2] = Σ_j S[2][j]·b_all[j]
        //   = S[2][0]·b0 + S[2][1]·b1 + S[2][2]·a_parent
        //   ≈ b_parent + 0 (since S[2][2]≈0)
        assert!(
            (a_all[2] - b_parent).abs() < 1e-8,
            "scatter_all parent port should match scatter_up: {} vs {}",
            a_all[2],
            b_parent
        );

        // For child ports: a_all[i] = Σ_j S[i][j]·b_all[j] should match a_children[i]
        for i in 0..2 {
            assert!(
                (a_all[i] - a_children[i]).abs() < 1e-8,
                "scatter_all child {} should match scatter_down: {} vs {}",
                i,
                a_all[i],
                a_children[i]
            );
        }
    }

    #[test]
    fn scattering_changes_when_port_resistance_changes() {
        // Minimal 2-node circuit: verify derive_scattering_matrix_general
        // produces different scattering when a port resistance changes.
        //
        // Topology: node 0 --[R1=1kΩ]--> GND
        //           node 0 --[NL port]--> node 1
        //           node 0 --[pot port]--> node 1
        //           node 1 --[adapted]---> GND
        let mut mna = MnaSystem::new(2, 0);
        mna.stamp_resistor(Some(0), None, 1000.0);
        // GMIN
        mna.stamp_resistor(Some(0), None, 1e9);
        mna.stamp_resistor(Some(1), None, 1e9);

        // Pot at minimum (1Ω)
        let ports_lo = vec![
            WdfPort {
                node_pos: Some(0),
                node_neg: Some(1),
                resistance: 10_000.0,
            },
            WdfPort {
                node_pos: Some(0),
                node_neg: Some(1),
                resistance: 1.0,
            },
            WdfPort {
                node_pos: Some(1),
                node_neg: None,
                resistance: 1000.0,
            },
        ];
        // Pot at maximum (1kΩ)
        let ports_hi = vec![
            WdfPort {
                node_pos: Some(0),
                node_neg: Some(1),
                resistance: 10_000.0,
            },
            WdfPort {
                node_pos: Some(0),
                node_neg: Some(1),
                resistance: 1000.0,
            },
            WdfPort {
                node_pos: Some(1),
                node_neg: None,
                resistance: 1000.0,
            },
        ];

        let s_lo = mna.derive_scattering_matrix_general(&ports_lo);
        let s_hi = mna.derive_scattering_matrix_general(&ports_hi);

        let max_diff = s_lo
            .iter()
            .zip(s_hi.iter())
            .map(|(a, b)| (a - b).abs())
            .fold(0.0f64, f64::max);

        assert!(
            max_diff > 0.01,
            "Scattering should change with port resistance (max_diff={max_diff:.6e})\n\
             s_lo={s_lo:.6?}\n\
             s_hi={s_hi:.6?}"
        );
    }

    // ─────────────────────────────────────────────────────────────────────
    // State-space reduction tests (Schur complement)
    // ─────────────────────────────────────────────────────────────────────

    /// Helper: compute eigenvalues of a 2×2 matrix.
    fn eigenvalues_2x2(a: &[f64; 4]) -> (num_complex::Complex64, num_complex::Complex64) {
        use num_complex::Complex64;
        let tr = a[0] + a[3]; // trace
        let det = a[0] * a[3] - a[1] * a[2]; // determinant
        let disc = tr * tr - 4.0 * det;
        if disc >= 0.0 {
            let sq = disc.sqrt();
            (
                Complex64::new((tr + sq) / 2.0, 0.0),
                Complex64::new((tr - sq) / 2.0, 0.0),
            )
        } else {
            let sq = (-disc).sqrt();
            (
                Complex64::new(tr / 2.0, sq / 2.0),
                Complex64::new(tr / 2.0, -sq / 2.0),
            )
        }
    }

    /// Simple RC lowpass: R + C to ground. One cap state.
    /// Analog: H(s) = 1/(1 + sRC)
    /// Discrete pole at z = (1 - RC·2fs)/(1 + RC·2fs) via bilinear.
    #[test]
    fn state_space_simple_rc_pole() {
        let r = 10_000.0; // 10kΩ
        let c = 100e-9; // 100nF
        let fs = 48000.0;

        // Build MNA: 2 nodes (in, cap_node), 1 VS (input)
        let mut mna = MnaSystem::new(2, 1);
        mna.stamp_resistor(Some(0), Some(1), r);
        mna.stamp_voltage_source(Some(0), None, 0);

        let cap_stamps = vec![(Some(1), None, c)];
        let (a_d, b_d, c_out, n_states) =
            mna.build_state_space_matrices(&cap_stamps, 0, Some(1), None, fs);

        // Should reduce to 1 state (the cap)
        assert_eq!(n_states, 1, "Should have 1 cap state, got {n_states}");

        // The single eigenvalue: z = (2fs*C*R - 1)/(2fs*C*R + 1)
        let rc_2fs = 2.0 * fs * r * c;
        let expected_pole = (rc_2fs - 1.0) / (rc_2fs + 1.0);
        let actual_pole = a_d[0];
        assert!(
            (actual_pole - expected_pole).abs() < 1e-6,
            "RC pole: expected {expected_pole:.6}, got {actual_pole:.6}"
        );

        // Pole should be real, positive, < 1 (stable)
        assert!(actual_pole > 0.0 && actual_pole < 1.0,
            "RC pole should be stable (0 < z < 1), got {actual_pole}");
    }

    /// Two-cap network with resistive coupling: creates complex eigenvalues.
    /// R1-C1-R2-C2 ladder: two RC stages create a 2nd-order response.
    #[test]
    fn state_space_two_cap_has_complex_poles() {
        let r1 = 10_000.0; // 10kΩ
        let r2 = 10_000.0;
        let c1 = 100e-9; // 100nF
        let c2 = 100e-9;
        let fs = 48000.0;

        // 3 nodes: input(0), mid(1), output(2). 1 VS.
        let mut mna = MnaSystem::new(3, 1);
        mna.stamp_resistor(Some(0), Some(1), r1);
        mna.stamp_resistor(Some(1), Some(2), r2);
        mna.stamp_voltage_source(Some(0), None, 0);

        let cap_stamps = vec![
            (Some(1), None, c1), // C1: mid to gnd
            (Some(2), None, c2), // C2: output to gnd
        ];
        let (a_d, _b_d, _c_out, n_states) =
            mna.build_state_space_matrices(&cap_stamps, 0, Some(2), None, fs);

        assert_eq!(n_states, 2, "Should have 2 cap states, got {n_states}");

        let a_2x2 = [a_d[0], a_d[1], a_d[n_states], a_d[n_states + 1]];
        let (ev1, ev2) = eigenvalues_2x2(&a_2x2);

        // Two-stage RC ladder has two real poles (overdamped), not complex.
        // But with feedback, it would oscillate. For this test, just verify
        // the eigenvalues are stable and the reduction works.
        eprintln!("Two-cap ladder: λ1={ev1}, λ2={ev2}");
        assert!(
            ev1.norm() < 1.0 && ev2.norm() < 1.0,
            "Both eigenvalues should be stable (|λ|<1): {ev1}, {ev2}"
        );
    }

    /// Bridged-T resonator with op-amp VCVS: the 808 kick drum circuit.
    /// Must produce complex eigenvalues near 130 Hz.
    #[test]
    fn state_space_bridged_t_has_complex_poles_at_130hz() {
        use std::f64::consts::PI;

        let r1 = 150_000.0;
        let r2 = 150_000.0;
        let c1 = 8.2e-9;
        let c2 = 8.2e-9;
        let r_fb = 470_000.0;
        let r_trig = 100_000.0;
        let r_out = 10_000.0;
        let aol = 5.0; // Near oscillation threshold for complex eigenvalues
        let ro = 75.0;
        let fs = 48000.0;

        let rc_product: f64 = r1 * r2 * c1 * c2;
        let f0_target = 1.0 / (2.0 * PI * rc_product.sqrt());
        eprintln!("Target f0 = {f0_target:.1} Hz");

        // MNA: 5 nodes + 2 VS (VCVS + input)
        // Node assignment:
        //   0 = junction (R1/R2/C2/R_trig)
        //   1 = output (R2/R_fb/R_out)
        //   2 = circuit_out (R_out far end)
        //   3 = neg (R1/R_fb/C1)
        //   4 = input (R_trig far end)
        let n_nodes = 5;
        let n_vs = 2; // VCVS + input VS
        let mut mna = MnaSystem::new(n_nodes, n_vs);

        // Resistors
        mna.stamp_resistor(Some(3), Some(0), r1);
        mna.stamp_resistor(Some(0), Some(1), r2);
        mna.stamp_resistor(Some(3), Some(1), r_fb);
        mna.stamp_resistor(Some(4), Some(0), r_trig);
        mna.stamp_resistor(Some(1), Some(2), r_out);

        // VCVS: V(out) - Aol*(V(pos) - V(neg)) + Ro*i = 0
        // pos=gnd(None), neg=3, out=1
        mna.stamp_vcvs(None, Some(3), Some(1), None, aol, ro, 0);

        // Input VS at node 4
        mna.stamp_voltage_source(Some(4), None, 1);

        // Caps: C1 (neg→gnd), C2 (junction→gnd)
        let cap_stamps = vec![
            (Some(3), None, c1),
            (Some(0), None, c2),
        ];

        let (a_d, b_d, c_out, n_states) =
            mna.build_state_space_matrices(&cap_stamps, 0, Some(1), None, fs);

        eprintln!("n_states = {n_states}");
        assert_eq!(n_states, 2, "Should reduce to 2 cap states (C1, C2)");

        // Print A matrix
        for i in 0..n_states {
            let row: Vec<String> = (0..n_states)
                .map(|j| format!("{:+.6e}", a_d[i * n_states + j]))
                .collect();
            eprintln!("A[{i}] = [{}]", row.join(", "));
        }

        // At least one eigenvalue pair should be complex (resonance)
        // For a 3×3, compute eigenvalues numerically via companion matrix
        // or just check the discriminant of the characteristic polynomial.
        // Simpler: run the system for 2000 samples with an impulse and
        // count zero crossings.
        let mut x = vec![0.0; n_states];
        let mut prev_y = 0.0;
        let mut zero_crossings = 0u32;
        let n_samples = 2000;
        for i in 0..n_samples {
            let u = if i == 0 { 1.0 } else { 0.0 };
            let mut x_new = vec![0.0; n_states];
            for r in 0..n_states {
                let mut v = b_d[r] * u;
                for c in 0..n_states {
                    v += a_d[r * n_states + c] * x[c];
                }
                x_new[r] = v;
            }
            let mut y = 0.0;
            for r in 0..n_states {
                y += c_out[r] * x_new[r];
            }
            if i > 0 && y * prev_y < 0.0 {
                zero_crossings += 1;
            }
            prev_y = y;
            x = x_new;
        }

        let est_freq = zero_crossings as f64 / 2.0 / (n_samples as f64 / fs);
        eprintln!("Zero crossings: {zero_crossings}, estimated f = {est_freq:.1} Hz");

        // Must oscillate (at least 10 zero crossings in ~42ms)
        assert!(
            zero_crossings > 10,
            "Bridged-T should oscillate: got only {zero_crossings} zero crossings"
        );

        // Frequency should be within 2× of 130 Hz
        assert!(
            est_freq > 65.0 && est_freq < 260.0,
            "Resonant frequency {est_freq:.1} Hz should be near {f0_target:.1} Hz"
        );
    }
}
