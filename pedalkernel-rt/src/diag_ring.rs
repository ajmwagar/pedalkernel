//! Live runtime diagnostics ring (Phase B of the diagnostics IPC channel,
//! bead pedalkernel-9u6u.2).
//!
//! Phase A ([`crate::diag`]) captures the **static** compile-time MNA /
//! scattering snapshot. This module captures the **dynamic** per-block runtime
//! behavior — the grouped Newton-Raphson convergence and, for each BJT NL port,
//! the operating-point state (`Vbe`, `Vce`, device currents `Ic`/`Ib`, the
//! transconductance `gm = ∂Ic/∂Vbe`) plus per-stage input/output level — into a
//! lock-free single-producer/single-consumer ring laid out over a **caller
//! provided byte region** (the std host mmaps the IPC file and hands the region
//! here as the ring backing; the CLI mmaps the same file and reads).
//!
//! # no_std / std split
//!
//! This module is `no_std + core` only: it writes a fixed binary layout into a
//! raw `*mut u8` region. It does **not** own a file or `mmap` — the std host
//! ([`pedalkernel`]'s `diag_ipc`) provides the backing pointer (Phase A reserved
//! the runtime-ring offset in the IPC header for exactly this). The same fixed
//! layout is parsed back on the std side by the CLI live-tail reader.
//!
//! # Layout (little-endian, version 1)
//!
//! The region begins with a fixed [`RING_HEADER_LEN`]-byte header, followed by
//! `capacity` back-to-back [`FrameRecord`]s of [`FRAME_LEN`] bytes each:
//!
//! ```text
//!  offset  size  field
//!  ------  ----  --------------------------------------------------------------
//!    0      4    ring_magic   = b"PKDR"
//!    4      4    ring_version = u32   (= 1)
//!    8      4    capacity     = u32   (number of frame slots; power of two)
//!   12      4    frame_len    = u32   (= FRAME_LEN; lets the reader self-check)
//!   16      8    write_idx    = u64   (monotonic; Release-stored by producer)
//!   24      8    reserved
//!   32      …    capacity × FrameRecord
//! ```
//!
//! `write_idx` is the only mutable header field. The producer writes a frame to
//! `slot = write_idx & (capacity-1)`, then `Release`-stores `write_idx+1`. A
//! reader `Acquire`-loads `write_idx` and reads the most recent `min(write_idx,
//! capacity)` completed slots — the classic overwrite SPSC discipline (a slow
//! reader drops old frames, never tears: every field is a plain POD copy and the
//! `Release`/`Acquire` pair publishes the slot before the index).
//!
//! Off by default: only constructed when the `diag` feature attaches a region.

use core::sync::atomic::{AtomicU64, Ordering};

/// Magic at the head of the runtime-ring region.
pub const RING_MAGIC: [u8; 4] = *b"PKDR";
/// Runtime-ring layout version.
pub const RING_VERSION: u32 = 1;
/// Fixed ring header length in bytes.
pub const RING_HEADER_LEN: usize = 32;

/// Maximum number of BJT/NL operating-point records carried per frame.
///
/// The BA283 co-solves 3 BJTs (6 NL ports); 8 device records is plenty for the
/// 1073 family and keeps a frame small.
pub const MAX_OP_RECORDS: usize = 8;
/// Maximum per-stage level records carried per frame.
pub const MAX_STAGE_LEVELS: usize = 16;

/// One nonlinear-device operating-point record (a BJT, triode, …).
///
/// All fields are `f32` so the frame layout is stable regardless of whether the
/// engine's [`crate::Wave`] is `f32` or `f64`. For a grouped BJT two-port:
/// `v_be`/`v_ce` are the NR operating-point voltages, `i_c`/`i_b` the device
/// currents at that point, and `gm = ∂Ic/∂Vbe` the transconductance. The
/// **whole BA283 question** is whether `i_c` swings with `v_be` at `gm`.
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub struct OpPointRecord {
    /// Stage index this device belongs to.
    pub stage_index: u32,
    /// Device/group index within the stage.
    pub group_index: u32,
    /// Base-emitter operating-point voltage `Vbe` (the NR `v_prev` BE port).
    pub v_be: f32,
    /// Collector-emitter operating-point voltage `Vce`.
    pub v_ce: f32,
    /// Collector current `Ic` at the operating point.
    pub i_c: f32,
    /// Base current `Ib` at the operating point.
    pub i_b: f32,
    /// Transconductance `gm = ∂Ic/∂Vbe` (S) — the textbook BJT gm.
    pub gm: f32,
    /// Self conductance on the CE port `∂Ic/∂Vce` (output conductance proxy).
    pub g_ce: f32,
    /// Component **reference** (the `.pedal` element id, e.g. `Q3`/`TR1`), NUL-
    /// padded ASCII. This is the device IDENTITY threaded from the compiler so
    /// the op-point is attributed to a physical transistor, **never** a guessed
    /// port index (`grp0`). Use [`Self::ref_str`] to read it.
    pub ref_name: [u8; Self::NAME_LEN],
    /// Device **type** (the model name, e.g. `2N3055`/`BC184C`), NUL-padded
    /// ASCII. Read via [`Self::type_str`].
    pub device_type: [u8; Self::NAME_LEN],
}

impl Default for OpPointRecord {
    fn default() -> Self {
        Self {
            stage_index: 0,
            group_index: 0,
            v_be: 0.0,
            v_ce: 0.0,
            i_c: 0.0,
            i_b: 0.0,
            gm: 0.0,
            g_ce: 0.0,
            ref_name: [0u8; Self::NAME_LEN],
            device_type: [0u8; Self::NAME_LEN],
        }
    }
}

impl OpPointRecord {
    /// Fixed inline length for the component ref / device-type ASCII fields.
    pub const NAME_LEN: usize = 16;

    /// Byte length of one op-point record.
    /// `2×u32 + 6×f32 + 2×NAME_LEN`.
    pub const LEN: usize = 4 + 4 + 4 * 6 + 2 * Self::NAME_LEN;

    /// Pack a `&str` into a fixed `[u8; NAME_LEN]`, truncating + NUL-padding.
    pub fn pack_name(s: &str) -> [u8; Self::NAME_LEN] {
        let mut out = [0u8; Self::NAME_LEN];
        let b = s.as_bytes();
        let n = b.len().min(Self::NAME_LEN);
        out[..n].copy_from_slice(&b[..n]);
        out
    }

    /// Decode a NUL-padded inline name field to `&str` (lossless for ASCII).
    fn name_str(field: &[u8; Self::NAME_LEN]) -> &str {
        let end = field.iter().position(|&b| b == 0).unwrap_or(Self::NAME_LEN);
        // Safe: producers only write valid ASCII via `pack_name`.
        core::str::from_utf8(&field[..end]).unwrap_or("")
    }

    /// The component reference (e.g. `Q3`) as a string.
    pub fn ref_str(&self) -> &str {
        Self::name_str(&self.ref_name)
    }

    /// The device type / model name (e.g. `2N3055`) as a string.
    pub fn type_str(&self) -> &str {
        Self::name_str(&self.device_type)
    }

    fn write_le(&self, out: &mut [u8]) {
        out[0..4].copy_from_slice(&self.stage_index.to_le_bytes());
        out[4..8].copy_from_slice(&self.group_index.to_le_bytes());
        out[8..12].copy_from_slice(&self.v_be.to_le_bytes());
        out[12..16].copy_from_slice(&self.v_ce.to_le_bytes());
        out[16..20].copy_from_slice(&self.i_c.to_le_bytes());
        out[20..24].copy_from_slice(&self.i_b.to_le_bytes());
        out[24..28].copy_from_slice(&self.gm.to_le_bytes());
        out[28..32].copy_from_slice(&self.g_ce.to_le_bytes());
        let nl = Self::NAME_LEN;
        out[32..32 + nl].copy_from_slice(&self.ref_name);
        out[32 + nl..32 + 2 * nl].copy_from_slice(&self.device_type);
    }

    fn read_le(buf: &[u8]) -> Self {
        let u32a = |o: usize| u32::from_le_bytes([buf[o], buf[o + 1], buf[o + 2], buf[o + 3]]);
        let f32a = |o: usize| f32::from_le_bytes([buf[o], buf[o + 1], buf[o + 2], buf[o + 3]]);
        let nl = Self::NAME_LEN;
        let mut ref_name = [0u8; Self::NAME_LEN];
        let mut device_type = [0u8; Self::NAME_LEN];
        ref_name.copy_from_slice(&buf[32..32 + nl]);
        device_type.copy_from_slice(&buf[32 + nl..32 + 2 * nl]);
        Self {
            stage_index: u32a(0),
            group_index: u32a(4),
            v_be: f32a(8),
            v_ce: f32a(12),
            i_c: f32a(16),
            i_b: f32a(20),
            gm: f32a(24),
            g_ce: f32a(28),
            ref_name,
            device_type,
        }
    }
}

/// A complete per-block runtime diagnostics frame.
///
/// Captured once per metering block (the same cadence Phase A's metrics use) and
/// pushed to the ring. Holds the block's NR convergence aggregate, per-NL-device
/// operating points, and per-stage input/output levels.
#[derive(Clone, Copy, Debug)]
#[repr(C)]
pub struct RuntimeFrame {
    /// Monotonic block counter (matches the metrics block counter).
    pub block_counter: u64,
    /// Number of NR solves recorded in this block.
    pub nr_solves: u64,
    /// Total NR iterations across all solves in this block.
    pub nr_total_iterations: u64,
    /// Max NR iterations in any single solve this block.
    pub nr_max_iterations: u32,
    /// Number of solves that hit the iteration cap (non-converged proxy).
    pub nr_iter_cap_hits: u32,
    /// Max NR residual seen this block.
    pub nr_max_residual: f32,
    /// Block input RMS (linear).
    pub input_rms: f32,
    /// Block output RMS (linear).
    pub output_rms: f32,
    /// Number of valid op-point records.
    pub n_op_records: u32,
    /// Number of valid stage-level records.
    pub n_stage_levels: u32,
    /// Per-device operating points (first `n_op_records` valid).
    pub op_records: [OpPointRecord; MAX_OP_RECORDS],
    /// Per-stage absolute output level (first `n_stage_levels` valid).
    pub stage_levels: [f32; MAX_STAGE_LEVELS],
}

impl Default for RuntimeFrame {
    fn default() -> Self {
        Self {
            block_counter: 0,
            nr_solves: 0,
            nr_total_iterations: 0,
            nr_max_iterations: 0,
            nr_iter_cap_hits: 0,
            nr_max_residual: 0.0,
            input_rms: 0.0,
            output_rms: 0.0,
            n_op_records: 0,
            n_stage_levels: 0,
            op_records: [OpPointRecord::default(); MAX_OP_RECORDS],
            stage_levels: [0.0; MAX_STAGE_LEVELS],
        }
    }
}

/// Fixed byte length of one serialized [`RuntimeFrame`] record.
pub const FRAME_LEN: usize = {
    // header scalars
    8 + 8 + 8 + 4 + 4 + 4 + 4 + 4 + 4 + 4
        + OpPointRecord::LEN * MAX_OP_RECORDS
        + 4 * MAX_STAGE_LEVELS
};

impl RuntimeFrame {
    fn write_le(&self, out: &mut [u8]) {
        out[0..8].copy_from_slice(&self.block_counter.to_le_bytes());
        out[8..16].copy_from_slice(&self.nr_solves.to_le_bytes());
        out[16..24].copy_from_slice(&self.nr_total_iterations.to_le_bytes());
        out[24..28].copy_from_slice(&self.nr_max_iterations.to_le_bytes());
        out[28..32].copy_from_slice(&self.nr_iter_cap_hits.to_le_bytes());
        out[32..36].copy_from_slice(&self.nr_max_residual.to_le_bytes());
        out[36..40].copy_from_slice(&self.input_rms.to_le_bytes());
        out[40..44].copy_from_slice(&self.output_rms.to_le_bytes());
        out[44..48].copy_from_slice(&self.n_op_records.to_le_bytes());
        out[48..52].copy_from_slice(&self.n_stage_levels.to_le_bytes());
        let mut o = 52;
        for rec in &self.op_records {
            rec.write_le(&mut out[o..o + OpPointRecord::LEN]);
            o += OpPointRecord::LEN;
        }
        for lvl in &self.stage_levels {
            out[o..o + 4].copy_from_slice(&lvl.to_le_bytes());
            o += 4;
        }
    }

    /// Parse a frame from `FRAME_LEN` bytes (used by the std-side CLI reader).
    pub fn read_le(buf: &[u8]) -> Self {
        let u64a = |o: usize| {
            u64::from_le_bytes([
                buf[o],
                buf[o + 1],
                buf[o + 2],
                buf[o + 3],
                buf[o + 4],
                buf[o + 5],
                buf[o + 6],
                buf[o + 7],
            ])
        };
        let u32a = |o: usize| u32::from_le_bytes([buf[o], buf[o + 1], buf[o + 2], buf[o + 3]]);
        let f32a = |o: usize| f32::from_le_bytes([buf[o], buf[o + 1], buf[o + 2], buf[o + 3]]);
        let mut frame = RuntimeFrame {
            block_counter: u64a(0),
            nr_solves: u64a(8),
            nr_total_iterations: u64a(16),
            nr_max_iterations: u32a(24),
            nr_iter_cap_hits: u32a(28),
            nr_max_residual: f32a(32),
            input_rms: f32a(36),
            output_rms: f32a(40),
            n_op_records: u32a(44),
            n_stage_levels: u32a(48),
            op_records: [OpPointRecord::default(); MAX_OP_RECORDS],
            stage_levels: [0.0; MAX_STAGE_LEVELS],
        };
        let mut o = 52;
        for rec in frame.op_records.iter_mut() {
            *rec = OpPointRecord::read_le(&buf[o..o + OpPointRecord::LEN]);
            o += OpPointRecord::LEN;
        }
        for lvl in frame.stage_levels.iter_mut() {
            *lvl = f32a(o);
            o += 4;
        }
        frame
    }
}

/// Total byte size a runtime ring region needs for `capacity` frames.
pub const fn region_len(capacity: usize) -> usize {
    RING_HEADER_LEN + capacity * FRAME_LEN
}

/// Lock-free SPSC runtime ring laid out over a caller-provided byte region.
///
/// The producer (audio engine) holds this and calls [`push`](Self::push) once
/// per block. The backing `*mut u8` points at the runtime-ring section of the
/// mmap'd IPC file; the std host owns the mapping's lifetime and guarantees the
/// region stays valid and is at least [`region_len`] bytes.
///
/// # Safety contract
///
/// The caller must ensure `ptr` is valid for `region_len(capacity)` bytes for
/// the lifetime of this `RuntimeRing`, and that **only one** `RuntimeRing` (one
/// producer) writes the region at a time. Readers map the same bytes read-only.
pub struct RuntimeRing {
    /// Base of the region.
    ptr: *mut u8,
    /// `capacity - 1` for fast modulo (capacity is a power of two).
    mask: usize,
    /// Mirror of the header `write_idx`, owned by the producer.
    write_idx: u64,
}

impl RuntimeRing {
    /// Initialize a ring over `ptr`/`capacity` and stamp the header.
    ///
    /// # Safety
    /// `ptr` must be valid and writable for `region_len(capacity)` bytes for the
    /// life of the returned ring, and uniquely owned by this producer.
    pub unsafe fn init(ptr: *mut u8, capacity: usize) -> Self {
        let capacity = capacity.next_power_of_two().max(2);
        // Stamp the header.
        let hdr = core::slice::from_raw_parts_mut(ptr, RING_HEADER_LEN);
        hdr[0..4].copy_from_slice(&RING_MAGIC);
        hdr[4..8].copy_from_slice(&RING_VERSION.to_le_bytes());
        hdr[8..12].copy_from_slice(&(capacity as u32).to_le_bytes());
        hdr[12..16].copy_from_slice(&(FRAME_LEN as u32).to_le_bytes());
        // write_idx + reserved start at zero.
        hdr[16..24].copy_from_slice(&0u64.to_le_bytes());
        hdr[24..32].copy_from_slice(&0u64.to_le_bytes());
        Self {
            ptr,
            mask: capacity - 1,
            write_idx: 0,
        }
    }

    /// Push a frame into the ring (producer; lock-free, wait-free).
    pub fn push(&mut self, frame: &RuntimeFrame) {
        let slot = (self.write_idx as usize) & self.mask;
        let off = RING_HEADER_LEN + slot * FRAME_LEN;
        // Safety: the region is valid for region_len(capacity) bytes and we are
        // the sole producer; the slot is exclusively ours until we publish.
        unsafe {
            let dst = core::slice::from_raw_parts_mut(self.ptr.add(off), FRAME_LEN);
            frame.write_le(dst);
            // Publish: Release-store the incremented index (header offset 16) so
            // a reader that Acquire-loads it sees the fully-written frame.
            self.write_idx = self.write_idx.wrapping_add(1);
            let widx_ptr = self.ptr.add(16) as *const AtomicU64;
            (*widx_ptr).store(self.write_idx, Ordering::Release);
        }
    }

    /// Number of frames pushed so far.
    pub fn frames_written(&self) -> u64 {
        self.write_idx
    }
}

// Safety: the ring is a single-producer handle over a byte region; the contract
// requires exclusive producer ownership. Send lets the audio thread own it.
unsafe impl Send for RuntimeRing {}

#[cfg(test)]
mod tests {
    use super::*;
    extern crate alloc;
    use alloc::vec;

    #[test]
    fn frame_roundtrip() {
        let mut f = RuntimeFrame::default();
        f.block_counter = 42;
        f.nr_solves = 7;
        f.nr_total_iterations = 21;
        f.nr_max_iterations = 5;
        f.nr_max_residual = 1.5e-6;
        f.input_rms = 0.0707;
        f.output_rms = 6.84;
        f.n_op_records = 1;
        f.n_stage_levels = 3;
        f.op_records[0] = OpPointRecord {
            stage_index: 1,
            group_index: 0,
            v_be: 0.62,
            v_ce: 4.5,
            i_c: 1.2e-3,
            i_b: 6.0e-6,
            gm: 0.033,
            g_ce: 1e-5,
            ref_name: OpPointRecord::pack_name("Q3"),
            device_type: OpPointRecord::pack_name("2N3055"),
        };
        f.stage_levels[0] = 0.1;
        f.stage_levels[1] = 0.032;
        f.stage_levels[2] = 0.033;

        let mut buf = vec![0u8; FRAME_LEN];
        f.write_le(&mut buf);
        let back = RuntimeFrame::read_le(&buf);
        assert_eq!(back.block_counter, 42);
        assert_eq!(back.nr_solves, 7);
        assert_eq!(back.n_op_records, 1);
        assert_eq!(back.op_records[0], f.op_records[0]);
        // Identity survives the binary round-trip and decodes back to &str.
        assert_eq!(back.op_records[0].ref_str(), "Q3");
        assert_eq!(back.op_records[0].type_str(), "2N3055");
        assert!((back.stage_levels[1] - 0.032).abs() < 1e-9);
    }

    #[test]
    fn ring_push_and_read_latest() {
        let cap = 4usize;
        let mut region = vec![0u8; region_len(cap)];
        let ptr = region.as_mut_ptr();
        let mut ring = unsafe { RuntimeRing::init(ptr, cap) };

        for i in 0..10u64 {
            let mut f = RuntimeFrame::default();
            f.block_counter = i;
            ring.push(&f);
        }
        assert_eq!(ring.frames_written(), 10);

        // Header reflects the published index.
        let widx = u64::from_le_bytes([
            region[16], region[17], region[18], region[19], region[20], region[21], region[22],
            region[23],
        ]);
        assert_eq!(widx, 10);

        // The latest slot holds block_counter 9.
        let slot = ((widx - 1) as usize) & (cap - 1);
        let off = RING_HEADER_LEN + slot * FRAME_LEN;
        let latest = RuntimeFrame::read_le(&region[off..off + FRAME_LEN]);
        assert_eq!(latest.block_counter, 9);
    }
}
