//! Precomputed scattering matrix binary format (`.pksc`).
//!
//! Stores scattering matrices, VS injection vectors, port resistances,
//! and pot interpolation tables so Cortex-M7 can skip MNA derivation
//! at runtime.
//!
//! # Binary format
//!
//! All multi-byte values are little-endian.  `f64` values are transmitted
//! as their IEEE-754 bit pattern.
//!
//! ```text
//! Header (32 bytes):
//!   magic:              [u8; 4]  = b"PKSC"
//!   version:            u16      = 1
//!   flags:              u16      = 0
//!   pedal_hash:         u32      (FNV-1a of source text)
//!   sample_rate_bits:   u64      (f64::to_bits)
//!   num_stages:         u16
//!   num_interp_tables:  u16
//!   reserved:           [u8; 8]
//!
//! Per Stage:
//!   stage_type:        u8     (0=WDF, 1=MultiNL, 2=PushPull)
//!   n_ports:           u16
//!   has_vs_injection:  u8
//!   has_extract:       u8
//!   padding:           [u8; 3]
//!   scattering:        [f64; n_ports × n_ports]  row-major
//!   vs_injection:      [f64; n_ports]             only if has_vs_injection != 0
//!   port_resistances:  [f64; n_ports]
//!   extract_coeffs:    [f64; n_ports]             only if has_extract != 0
//!   extract_vs:        f64                        only if has_extract != 0
//!
//! Per Interpolation Table:
//!   stage_index:       u16
//!   n_positions:       u16
//!   n_ports:           u16
//!   has_vs_injection:  u8
//!   padding:           u8
//!   resistances:       [f64; n_positions]
//!   matrices:          [f64; n_positions × n_ports × n_ports]
//!   injections:        [f64; n_positions × n_ports]   only if has_vs_injection != 0
//! ```

use std::io::{Cursor, Read, Write};

const MAGIC: [u8; 4] = *b"PKSC";
const VERSION: u16 = 1;

// ═══════════════════════════════════════════════════════════════════════════
// Error type
// ═══════════════════════════════════════════════════════════════════════════

/// Errors produced by `.pksc` serialization / deserialization.
#[derive(Debug)]
pub enum PrecomputeError {
    /// Underlying I/O failure during byte-level read/write.
    Io(std::io::Error),
    /// Magic bytes did not match `b"PKSC"`.
    BadMagic,
    /// File version is newer than this library knows about.
    BadVersion(u16),
    /// A field value is outside the range this library accepts.
    Consistency(String),
}

impl std::fmt::Display for PrecomputeError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Io(e) => write!(f, "I/O error: {e}"),
            Self::BadMagic => write!(f, "invalid magic bytes (expected PKSC)"),
            Self::BadVersion(v) => write!(f, "unsupported version {v}"),
            Self::Consistency(msg) => write!(f, "data consistency error: {msg}"),
        }
    }
}

impl std::error::Error for PrecomputeError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        if let Self::Io(e) = self {
            Some(e)
        } else {
            None
        }
    }
}

impl From<std::io::Error> for PrecomputeError {
    fn from(e: std::io::Error) -> Self {
        Self::Io(e)
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Public data types
// ═══════════════════════════════════════════════════════════════════════════

/// A complete precomputed scattering package for one `.pedal` file.
#[derive(Debug, Clone)]
pub struct PrecomputedScattering {
    /// FNV-1a hash of the `.pedal` source text used to produce this file.
    /// Checked at load time to detect stale `.pksc` files.
    pub pedal_hash: u32,
    /// Sample rate this package was compiled for (Hz).
    pub sample_rate: f64,
    /// One entry per compiled stage in topological processing order.
    pub stages: Vec<PrecomputedStage>,
    /// Pot interpolation tables (one per stage that has a single pot).
    pub interp_tables: Vec<PrecomputedInterpTable>,
}

/// Precomputed data for one stage (WDF, MultiNL, or PushPull).
#[derive(Debug, Clone)]
pub struct PrecomputedStage {
    /// Stage type discriminant: 0 = WDF, 1 = MultiNL, 2 = PushPull.
    pub stage_type: u8,
    /// Number of WDF / MNA ports.
    pub n_ports: usize,
    /// Scattering matrix, `n_ports × n_ports`, row-major.
    ///
    /// For `RTypeAdaptor`-based stages this is the **power-normalized** S̅
    /// matrix (as stored internally).  See the `RTypeAdaptor` docs for the
    /// normalization convention.  If the raw matrix is ever needed, it can be
    /// recovered via `S[i][j] = S̅[i][j] * √(R_i / R_j)`.
    pub scattering: Vec<f64>,
    /// VS injection vector (length `n_ports`), present when the stage is
    /// driven by an ideal voltage source input.
    pub vs_injection: Option<Vec<f64>>,
    /// Port resistances (Ω), length `n_ports`.
    pub port_resistances: Vec<f64>,
    /// Node-voltage extraction coefficients (length `n_ports`), present when
    /// the output is read directly from MNA node voltages.
    pub extract_coeffs: Option<Vec<f64>>,
    /// VS component of the extraction formula:
    ///   `V_out = Σ extract_coeffs[k]·b[k]  +  extract_vs · V_in`
    pub extract_vs: f64,
}

/// Precomputed pot-sweep interpolation table for a single-pot stage.
#[derive(Debug, Clone)]
pub struct PrecomputedInterpTable {
    /// Index into `PrecomputedScattering::stages` this table belongs to.
    pub stage_index: usize,
    /// Number of WDF ports.
    pub n_ports: usize,
    /// Whether VS injection vectors are present.
    pub has_vs_injection: bool,
    /// `K` log-spaced resistance values (ascending).
    pub resistances: Vec<f64>,
    /// `K` scattering matrices, each `n_ports²` entries, row-major.
    pub matrices: Vec<Vec<f64>>,
    /// `K` VS injection vectors, each `n_ports` entries.
    /// Empty when `has_vs_injection` is `false`.
    pub injections: Vec<Vec<f64>>,
}

// ═══════════════════════════════════════════════════════════════════════════
// FNV-1a hash helper
// ═══════════════════════════════════════════════════════════════════════════

/// Compute the 32-bit FNV-1a hash of `text`.
///
/// Used to fingerprint the `.pedal` source so stale `.pksc` files can be
/// detected at load time.
///
/// # Example
///
/// ```
/// use pedalkernel::precompute::pedal_hash;
/// let h = pedal_hash("hello");
/// assert_ne!(h, pedal_hash("world"));
/// ```
pub fn pedal_hash(text: &str) -> u32 {
    const FNV_OFFSET: u32 = 0x811c_9dc5;
    const FNV_PRIME: u32 = 0x0100_0193;
    let mut hash = FNV_OFFSET;
    for byte in text.bytes() {
        hash ^= u32::from(byte);
        hash = hash.wrapping_mul(FNV_PRIME);
    }
    hash
}

// ═══════════════════════════════════════════════════════════════════════════
// Serialization helpers
// ═══════════════════════════════════════════════════════════════════════════

fn write_u8(w: &mut impl Write, v: u8) -> std::io::Result<()> {
    w.write_all(&[v])
}

fn write_u16_le(w: &mut impl Write, v: u16) -> std::io::Result<()> {
    w.write_all(&v.to_le_bytes())
}

fn write_u32_le(w: &mut impl Write, v: u32) -> std::io::Result<()> {
    w.write_all(&v.to_le_bytes())
}

fn write_u64_le(w: &mut impl Write, v: u64) -> std::io::Result<()> {
    w.write_all(&v.to_le_bytes())
}

fn write_f64_le(w: &mut impl Write, v: f64) -> std::io::Result<()> {
    write_u64_le(w, v.to_bits())
}

fn write_f64_slice(w: &mut impl Write, s: &[f64]) -> std::io::Result<()> {
    for &v in s {
        write_f64_le(w, v)?;
    }
    Ok(())
}

fn read_u8(r: &mut impl Read) -> std::io::Result<u8> {
    let mut buf = [0u8; 1];
    r.read_exact(&mut buf)?;
    Ok(buf[0])
}

fn read_u16_le(r: &mut impl Read) -> std::io::Result<u16> {
    let mut buf = [0u8; 2];
    r.read_exact(&mut buf)?;
    Ok(u16::from_le_bytes(buf))
}

fn read_u32_le(r: &mut impl Read) -> std::io::Result<u32> {
    let mut buf = [0u8; 4];
    r.read_exact(&mut buf)?;
    Ok(u32::from_le_bytes(buf))
}

fn read_u64_le(r: &mut impl Read) -> std::io::Result<u64> {
    let mut buf = [0u8; 8];
    r.read_exact(&mut buf)?;
    Ok(u64::from_le_bytes(buf))
}

fn read_f64_le(r: &mut impl Read) -> std::io::Result<f64> {
    Ok(f64::from_bits(read_u64_le(r)?))
}

fn read_f64_vec(r: &mut impl Read, n: usize) -> std::io::Result<Vec<f64>> {
    let mut v = vec![0.0f64; n];
    for x in &mut v {
        *x = read_f64_le(r)?;
    }
    Ok(v)
}

// ═══════════════════════════════════════════════════════════════════════════
// PrecomputedScattering  —  to_bytes / from_bytes
// ═══════════════════════════════════════════════════════════════════════════

impl PrecomputedScattering {
    /// Serialize to the `.pksc` binary format.
    ///
    /// # Errors
    ///
    /// Returns an error if the internal `Vec` write fails (practically
    /// infallible, but propagated for API consistency).
    pub fn to_bytes(&self) -> Result<Vec<u8>, PrecomputeError> {
        let mut buf: Vec<u8> = Vec::new();
        self.write_to(&mut buf)?;
        Ok(buf)
    }

    fn write_to(&self, w: &mut impl Write) -> Result<(), PrecomputeError> {
        // ── Header (32 bytes) ───────────────────────────────────────────
        w.write_all(&MAGIC)?;
        write_u16_le(w, VERSION)?;
        write_u16_le(w, 0u16)?; // flags
        write_u32_le(w, self.pedal_hash)?;
        write_u64_le(w, self.sample_rate.to_bits())?;

        let num_stages = self.stages.len();
        let num_tables = self.interp_tables.len();
        if num_stages > u16::MAX as usize {
            return Err(PrecomputeError::Consistency(format!(
                "too many stages: {num_stages}"
            )));
        }
        if num_tables > u16::MAX as usize {
            return Err(PrecomputeError::Consistency(format!(
                "too many interp tables: {num_tables}"
            )));
        }
        write_u16_le(w, num_stages as u16)?;
        write_u16_le(w, num_tables as u16)?;
        w.write_all(&[0u8; 8])?; // reserved

        // ── Stages ──────────────────────────────────────────────────────
        for stage in &self.stages {
            stage.write_to(w)?;
        }

        // ── Interpolation tables ────────────────────────────────────────
        for table in &self.interp_tables {
            table.write_to(w)?;
        }

        Ok(())
    }

    /// Deserialize from the `.pksc` binary format.
    ///
    /// Returns [`PrecomputeError::BadMagic`] if the magic bytes are wrong,
    /// [`PrecomputeError::BadVersion`] if the version is unsupported, or an
    /// I/O error if the data is truncated.
    pub fn from_bytes(data: &[u8]) -> Result<Self, PrecomputeError> {
        let mut cursor = Cursor::new(data);
        Self::read_from(&mut cursor)
    }

    fn read_from(r: &mut impl Read) -> Result<Self, PrecomputeError> {
        // ── Header ───────────────────────────────────────────────────────
        let mut magic = [0u8; 4];
        r.read_exact(&mut magic)?;
        if magic != MAGIC {
            return Err(PrecomputeError::BadMagic);
        }

        let version = read_u16_le(r)?;
        if version != VERSION {
            return Err(PrecomputeError::BadVersion(version));
        }

        let _flags = read_u16_le(r)?;
        let pedal_hash = read_u32_le(r)?;
        let sample_rate = f64::from_bits(read_u64_le(r)?);
        let num_stages = read_u16_le(r)? as usize;
        let num_tables = read_u16_le(r)? as usize;
        let mut _reserved = [0u8; 8];
        r.read_exact(&mut _reserved)?;

        // ── Stages ───────────────────────────────────────────────────────
        let mut stages = Vec::with_capacity(num_stages);
        for _ in 0..num_stages {
            stages.push(PrecomputedStage::read_from(r)?);
        }

        // ── Interpolation tables ─────────────────────────────────────────
        let mut interp_tables = Vec::with_capacity(num_tables);
        for _ in 0..num_tables {
            interp_tables.push(PrecomputedInterpTable::read_from(r)?);
        }

        Ok(Self {
            pedal_hash,
            sample_rate,
            stages,
            interp_tables,
        })
    }
}

// ── PrecomputedStage ─────────────────────────────────────────────────────

impl PrecomputedStage {
    fn write_to(&self, w: &mut impl Write) -> Result<(), PrecomputeError> {
        let n = self.n_ports;
        if n > u16::MAX as usize {
            return Err(PrecomputeError::Consistency(format!(
                "n_ports too large: {n}"
            )));
        }

        write_u8(w, self.stage_type)?;
        write_u16_le(w, n as u16)?;
        write_u8(w, u8::from(self.vs_injection.is_some()))?;
        write_u8(w, u8::from(self.extract_coeffs.is_some()))?;
        w.write_all(&[0u8; 3])?; // padding

        write_f64_slice(w, &self.scattering)?;

        if let Some(ref vs) = self.vs_injection {
            write_f64_slice(w, vs)?;
        }

        write_f64_slice(w, &self.port_resistances)?;

        if let Some(ref ec) = self.extract_coeffs {
            write_f64_slice(w, ec)?;
            write_f64_le(w, self.extract_vs)?;
        }

        Ok(())
    }

    fn read_from(r: &mut impl Read) -> Result<Self, PrecomputeError> {
        let stage_type = read_u8(r)?;
        let n = read_u16_le(r)? as usize;
        let has_vs = read_u8(r)? != 0;
        let has_extract = read_u8(r)? != 0;
        let mut _pad = [0u8; 3];
        r.read_exact(&mut _pad)?;

        let scattering = read_f64_vec(r, n * n)?;
        let vs_injection = if has_vs {
            Some(read_f64_vec(r, n)?)
        } else {
            None
        };
        let port_resistances = read_f64_vec(r, n)?;
        let (extract_coeffs, extract_vs) = if has_extract {
            let ec = read_f64_vec(r, n)?;
            let ev = read_f64_le(r)?;
            (Some(ec), ev)
        } else {
            (None, 0.0)
        };

        Ok(Self {
            stage_type,
            n_ports: n,
            scattering,
            vs_injection,
            port_resistances,
            extract_coeffs,
            extract_vs,
        })
    }
}

// ── PrecomputedInterpTable ───────────────────────────────────────────────

impl PrecomputedInterpTable {
    fn write_to(&self, w: &mut impl Write) -> Result<(), PrecomputeError> {
        let k = self.resistances.len();
        let n = self.n_ports;

        if self.stage_index > u16::MAX as usize {
            return Err(PrecomputeError::Consistency(format!(
                "stage_index too large: {}",
                self.stage_index
            )));
        }
        if k > u16::MAX as usize {
            return Err(PrecomputeError::Consistency(format!(
                "n_positions too large: {k}"
            )));
        }
        if n > u16::MAX as usize {
            return Err(PrecomputeError::Consistency(format!(
                "n_ports too large: {n}"
            )));
        }

        write_u16_le(w, self.stage_index as u16)?;
        write_u16_le(w, k as u16)?;
        write_u16_le(w, n as u16)?;
        write_u8(w, u8::from(self.has_vs_injection))?;
        write_u8(w, 0u8)?; // padding

        write_f64_slice(w, &self.resistances)?;

        for mat in &self.matrices {
            write_f64_slice(w, mat)?;
        }

        if self.has_vs_injection {
            for inj in &self.injections {
                write_f64_slice(w, inj)?;
            }
        }

        Ok(())
    }

    fn read_from(r: &mut impl Read) -> Result<Self, PrecomputeError> {
        let stage_index = read_u16_le(r)? as usize;
        let k = read_u16_le(r)? as usize;
        let n = read_u16_le(r)? as usize;
        let has_vs_injection = read_u8(r)? != 0;
        let _pad = read_u8(r)?;

        let resistances = read_f64_vec(r, k)?;

        let mut matrices = Vec::with_capacity(k);
        for _ in 0..k {
            matrices.push(read_f64_vec(r, n * n)?);
        }

        let mut injections = Vec::with_capacity(k);
        if has_vs_injection {
            for _ in 0..k {
                injections.push(read_f64_vec(r, n)?);
            }
        }

        Ok(Self {
            stage_index,
            n_ports: n,
            has_vs_injection,
            resistances,
            matrices,
            injections,
        })
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Extract from CompiledPedal
// ═══════════════════════════════════════════════════════════════════════════

/// Extract a [`PrecomputedScattering`] from a compiled pedal.
///
/// Iterates over the compiled pedal's MultiNL stages and extracts scattering
/// matrices, port resistances, VS injection vectors, and pot interpolation
/// tables.
///
/// # Arguments
///
/// * `compiled`      – the compiled pedal processor
/// * `pedal_source`  – the original `.pedal` source text (used to compute the hash)
/// * `sample_rate`   – the sample rate used during compilation (Hz)
pub fn extract_precomputed(
    compiled: &crate::compiler::CompiledPedal,
    pedal_source: &str,
    sample_rate: f64,
) -> PrecomputedScattering {
    crate::compiler::extract_precomputed_from_compiled(compiled, pedal_source, sample_rate)
}

// ═══════════════════════════════════════════════════════════════════════════
// Tests
// ═══════════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod tests {
    use super::*;

    // ── round-trip test ──────────────────────────────────────────────────

    #[test]
    fn precompute_round_trip() {
        let original = PrecomputedScattering {
            pedal_hash: 0xDEAD_BEEF,
            sample_rate: 48000.0,
            stages: vec![
                PrecomputedStage {
                    stage_type: 1,
                    n_ports: 3,
                    scattering: vec![
                        1.0, 0.0, 0.0, // row 0
                        0.0, 1.0, 0.0, // row 1
                        0.0, 0.0, 0.0, // row 2 (adapted)
                    ],
                    vs_injection: Some(vec![0.5, -0.5, 1.0]),
                    port_resistances: vec![100.0, 200.0, 300.0],
                    extract_coeffs: Some(vec![0.1, 0.2, 0.3]),
                    extract_vs: 0.42,
                },
                PrecomputedStage {
                    stage_type: 0,
                    n_ports: 2,
                    scattering: vec![-1.0, 2.0, 2.0, -1.0],
                    vs_injection: None,
                    port_resistances: vec![50.0, 50.0],
                    extract_coeffs: None,
                    extract_vs: 0.0,
                },
            ],
            interp_tables: vec![PrecomputedInterpTable {
                stage_index: 0,
                n_ports: 3,
                has_vs_injection: true,
                resistances: vec![10.0, 100.0, 1000.0],
                matrices: vec![
                    vec![1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
                    vec![0.5, 0.5, 0.0, 0.5, 0.5, 0.0, 0.0, 0.0, 0.0],
                    vec![0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                ],
                injections: vec![
                    vec![1.0, 0.0, 0.0],
                    vec![0.5, 0.5, 0.0],
                    vec![0.0, 1.0, 0.0],
                ],
            }],
        };

        let bytes = original.to_bytes().expect("serialize");
        let restored = PrecomputedScattering::from_bytes(&bytes).expect("deserialize");

        assert_eq!(restored.pedal_hash, original.pedal_hash);
        assert!((restored.sample_rate - original.sample_rate).abs() < 1e-9);
        assert_eq!(restored.stages.len(), original.stages.len());

        // Stage 0 checks
        let s0 = &restored.stages[0];
        assert_eq!(s0.stage_type, 1);
        assert_eq!(s0.n_ports, 3);
        assert_eq!(s0.scattering, original.stages[0].scattering);
        assert_eq!(s0.vs_injection, original.stages[0].vs_injection);
        assert_eq!(s0.port_resistances, original.stages[0].port_resistances);
        assert_eq!(s0.extract_coeffs, original.stages[0].extract_coeffs);
        assert!((s0.extract_vs - original.stages[0].extract_vs).abs() < 1e-12);

        // Stage 1 checks
        let s1 = &restored.stages[1];
        assert_eq!(s1.stage_type, 0);
        assert!(s1.vs_injection.is_none());
        assert!(s1.extract_coeffs.is_none());

        // Interp table checks
        assert_eq!(restored.interp_tables.len(), 1);
        let t0 = &restored.interp_tables[0];
        assert_eq!(t0.stage_index, 0);
        assert_eq!(t0.n_ports, 3);
        assert!(t0.has_vs_injection);
        assert_eq!(t0.resistances, original.interp_tables[0].resistances);
        assert_eq!(t0.matrices, original.interp_tables[0].matrices);
        assert_eq!(t0.injections, original.interp_tables[0].injections);
    }

    // ── hash stability test ──────────────────────────────────────────────

    #[test]
    fn precompute_hash_changes() {
        let h1 = pedal_hash("pedal A {}");
        let h2 = pedal_hash("pedal B {}");
        let h3 = pedal_hash("pedal A {}");
        assert_ne!(h1, h2, "different sources must produce different hashes");
        assert_eq!(h1, h3, "same source must produce identical hash");
    }

    // ── bad-magic rejection ──────────────────────────────────────────────

    #[test]
    fn precompute_rejects_bad_magic() {
        let mut bad = b"NOPE\x01\x00".to_vec();
        bad.extend_from_slice(&[0u8; 26]);
        let err = PrecomputedScattering::from_bytes(&bad).unwrap_err();
        assert!(matches!(err, PrecomputeError::BadMagic));
    }

    // ── pedal integration test ───────────────────────────────────────────

    #[test]
    fn precompute_from_pedal() {
        // BJT differential pair — two coupled NPN transistors generate a MultiNL stage
        // (R-type adaptor + multi-port NR solver).
        let src = r#"
pedal "ClipTest" subtitle "precompute integration test" {
  supply 12V
  components {
    C1:     cap(100n)
    R_B1:   resistor(100k)
    R_B2:   resistor(100k)
    Q1:     npn(2n3904)
    Q2:     npn(2n3904)
    RC1:    resistor(10k)
    RC2:    resistor(10k)
    R_tail: resistor(10k)
    C_out:  cap(1u)
    RL:     resistor(10k)
  }
  nets {
    in -> C1.a
    C1.b -> R_B1.a, Q1.base
    R_B1.b -> vcc
    R_B2.a -> Q2.base
    R_B2.b -> vcc
    Q1.collector -> RC1.a
    Q2.collector -> RC2.a
    RC1.b -> vcc
    RC2.b -> vcc
    Q1.emitter -> Q2.emitter, R_tail.a
    R_tail.b -> gnd
    Q1.collector -> C_out.a
    C_out.b -> RL.a, out
    RL.b -> gnd
  }
  controls {}
}
"#;
        use crate::compiler::{compile_pedal, CompiledPedal};
        use crate::dsl::parse_pedal_file;

        let pedal_def = parse_pedal_file(src).expect("parse");
        let compiled: CompiledPedal = compile_pedal(&pedal_def, 48000.0).expect("compile");

        let precomp = extract_precomputed(&compiled, src, 48000.0);

        // The hash must match what pedal_hash() returns for the same source.
        assert_eq!(precomp.pedal_hash, pedal_hash(src));
        assert!((precomp.sample_rate - 48000.0).abs() < 1e-9);

        // At least one MultiNL stage must be present (opamp + diode uses MNA/NR).
        assert!(
            !precomp.stages.is_empty(),
            "expected at least one MultiNL stage, got none \
             (debug: compiled has {} wdf + {} multi_nl stages)",
            compiled.debug_stage_count(),
            compiled.debug_multi_nl_count(),
        );

        // Every stage must have a well-formed scattering matrix.
        for (i, stage) in precomp.stages.iter().enumerate() {
            let expected_len = stage.n_ports * stage.n_ports;
            assert_eq!(
                stage.scattering.len(),
                expected_len,
                "stage {i}: scattering matrix has wrong length"
            );
            assert!(
                stage.scattering.iter().all(|x| x.is_finite()),
                "stage {i}: scattering matrix contains non-finite values"
            );
        }
    }
}
