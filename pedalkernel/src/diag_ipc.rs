//! Diagnostics IPC file format + memory-mapped read/write (std host side).
//!
//! Phase A of the diagnostics IPC channel (bead pedalkernel-9u6u.1). The
//! `pedalkernel-rt` crate produces a [`DiagSnapshot`](pedalkernel_rt::diag::DiagSnapshot)
//! (plain `Vec`/`String`, `no_std`); this module — which lives on the **std**
//! side — serializes it into a memory-mapped file and reads it back.
//!
//! # File layout (version 1)
//!
//! A fixed 32-byte header followed by a body. All multi-byte integers are
//! little-endian.
//!
//! ```text
//! offset  size  field
//! ------  ----  -----------------------------------------------------------
//!   0      8    magic   = b"PKDIAG\x00\x01"  (ASCII "PKDIAG" + 0x00 0x01)
//!   8      4    version = u32                  (= 1 for this layout)
//!  12      4    section_count = u32            (= 1 in Phase A: MNA snapshot)
//!  16      4    mna_kind = u32                 (0 = JSON body)
//!  20      4    mna_offset = u32               (byte offset of MNA body)
//!  24      4    mna_len = u32                  (byte length of MNA body)
//!  28      4    reserved = u32                 (0; Phase B: runtime-ring offset)
//!  32      …    MNA body                        (mna_kind==0 → UTF-8 JSON of
//!                                                DiagSnapshot)
//! ```
//!
//! ## Why JSON for the MNA section (Phase A choice — Rule 1 tradeoff)
//!
//! The MNA snapshot is **static** (written once at compile time, read once by
//! the CLI). JSON keeps the matrices self-describing and trivially diffable for
//! the hand-computation we need on the BA283. The structured binary header lets
//! Phase B append a fixed-layout, lock-free runtime ring section *after* the
//! JSON body without disturbing it — the `section_count`/`reserved` fields are
//! the extension points. The runtime ring (Phase B) must be a binary layout
//! (lock-free `MetricsRingBuffer` over the mmap region); only the static MNA
//! section is JSON.

use memmap2::{Mmap, MmapMut};
use pedalkernel_rt::diag::DiagSnapshot;
use pedalkernel_rt::diag_ring::{self, RuntimeFrame};
use std::fs::OpenOptions;
use std::io::{self, Write};
use std::path::Path;

/// Magic bytes at the head of every diagnostics IPC file.
pub const MAGIC: [u8; 8] = *b"PKDIAG\x00\x01";
/// Current file-format version.
pub const VERSION: u32 = 1;
/// Total header size in bytes.
pub const HEADER_LEN: usize = 32;
/// `mna_kind` value indicating a UTF-8 JSON body.
pub const MNA_KIND_JSON: u32 = 0;

/// Serialize a [`DiagSnapshot`] to a memory-mapped IPC file at `path`.
///
/// Writes the version-1 header + a JSON MNA body. The file is created/truncated.
pub fn write_snapshot(path: &Path, snapshot: &DiagSnapshot) -> io::Result<()> {
    let body = serde_json::to_vec_pretty(snapshot)
        .map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e))?;
    let total = HEADER_LEN + body.len();

    let file = OpenOptions::new()
        .read(true)
        .write(true)
        .create(true)
        .truncate(true)
        .open(path)?;
    file.set_len(total as u64)?;

    let mut mmap = unsafe { MmapMut::map_mut(&file)? };

    // ── Header ───────────────────────────────────────────────────────────
    mmap[0..8].copy_from_slice(&MAGIC);
    write_u32(&mut mmap[8..12], VERSION);
    write_u32(&mut mmap[12..16], 1); // section_count
    write_u32(&mut mmap[16..20], MNA_KIND_JSON);
    write_u32(&mut mmap[20..24], HEADER_LEN as u32); // mna_offset
    write_u32(&mut mmap[24..28], body.len() as u32); // mna_len
    write_u32(&mut mmap[28..32], 0); // reserved (Phase B)

    // ── Body ─────────────────────────────────────────────────────────────
    (&mut mmap[HEADER_LEN..]).write_all(&body)?;
    mmap.flush()?;
    Ok(())
}

/// Read and deserialize a [`DiagSnapshot`] from a memory-mapped IPC file.
pub fn read_snapshot(path: &Path) -> io::Result<DiagSnapshot> {
    let file = OpenOptions::new().read(true).open(path)?;
    let mmap = unsafe { Mmap::map(&file)? };

    if mmap.len() < HEADER_LEN {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "diag IPC file too short for header",
        ));
    }
    if mmap[0..8] != MAGIC {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "bad magic — not a PKDIAG file",
        ));
    }
    let version = read_u32(&mmap[8..12]);
    if version != VERSION {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            format!("unsupported diag IPC version {version} (expected {VERSION})"),
        ));
    }
    let mna_kind = read_u32(&mmap[16..20]);
    let mna_offset = read_u32(&mmap[20..24]) as usize;
    let mna_len = read_u32(&mmap[24..28]) as usize;

    if mna_kind != MNA_KIND_JSON {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            format!("unsupported MNA section kind {mna_kind} (expected JSON)"),
        ));
    }
    let end = mna_offset
        .checked_add(mna_len)
        .filter(|&e| e <= mmap.len())
        .ok_or_else(|| {
            io::Error::new(io::ErrorKind::InvalidData, "MNA section out of bounds")
        })?;
    let body = &mmap[mna_offset..end];
    serde_json::from_slice(body).map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e))
}

// ═══════════════════════════════════════════════════════════════════════════
// Phase B — runtime ring section
// ═══════════════════════════════════════════════════════════════════════════
//
// The Phase-A header's `reserved` field (offset 28) is repurposed as the
// **runtime-ring byte offset** within the file: 0 means "no ring" (a Phase-A
// file), any non-zero value is the start of a `diag_ring`-formatted region. The
// region is appended *after* the JSON MNA body, so Phase-A readers (which only
// touch `mna_offset..mna_offset+mna_len`) are unaffected.

/// A writable IPC file with an appended runtime-ring section (Phase B).
///
/// Holds the `MmapMut` that owns the runtime-ring region's lifetime and exposes
/// the raw pointer/capacity the engine hands to
/// `CompiledPedal::attach_diag_ring`. Keep this alive for as long as the engine
/// is writing frames.
pub struct DiagIpcWriter {
    mmap: MmapMut,
    ring_offset: usize,
    ring_capacity: usize,
}

impl DiagIpcWriter {
    /// Raw pointer to the runtime-ring region (pass to `attach_diag_ring`).
    pub fn ring_ptr(&self) -> *mut u8 {
        // Safety: the region lives inside the mmap owned by `self`.
        unsafe { self.mmap.as_ptr().add(self.ring_offset) as *mut u8 }
    }

    /// Frame-slot capacity of the runtime ring.
    pub fn ring_capacity(&self) -> usize {
        self.ring_capacity
    }

    /// Flush the mmap to disk (so out-of-process readers see the header/JSON).
    pub fn flush(&self) -> io::Result<()> {
        self.mmap.flush()
    }
}

/// Write a [`DiagSnapshot`] **and** reserve an appended runtime-ring section,
/// returning a [`DiagIpcWriter`] whose region the engine writes per-block frames
/// into. `ring_capacity` is the number of frame slots (rounded up to a power of
/// two by the ring).
pub fn write_snapshot_with_ring(
    path: &Path,
    snapshot: &DiagSnapshot,
    ring_capacity: usize,
) -> io::Result<DiagIpcWriter> {
    let body = serde_json::to_vec_pretty(snapshot)
        .map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e))?;

    let ring_capacity = ring_capacity.next_power_of_two().max(2);
    // Align the ring section to 8 bytes so the in-file `write_idx` AtomicU64
    // (ring header offset 16) is naturally aligned for the reader's load. The
    // mmap base is page-aligned, so an 8-aligned ring_offset suffices.
    let ring_offset = (HEADER_LEN + body.len() + 7) & !7;
    let ring_len = diag_ring::region_len(ring_capacity);
    let total = ring_offset + ring_len;

    let file = OpenOptions::new()
        .read(true)
        .write(true)
        .create(true)
        .truncate(true)
        .open(path)?;
    file.set_len(total as u64)?;

    let mut mmap = unsafe { MmapMut::map_mut(&file)? };

    // ── Header (section_count = 2; reserved = ring offset) ────────────────
    mmap[0..8].copy_from_slice(&MAGIC);
    write_u32(&mut mmap[8..12], VERSION);
    write_u32(&mut mmap[12..16], 2); // section_count: MNA + runtime ring
    write_u32(&mut mmap[16..20], MNA_KIND_JSON);
    write_u32(&mut mmap[20..24], HEADER_LEN as u32); // mna_offset
    write_u32(&mut mmap[24..28], body.len() as u32); // mna_len
    write_u32(&mut mmap[28..32], ring_offset as u32); // runtime-ring offset

    // ── MNA body (padding between body end and the aligned ring stays 0) ──
    let body_end = HEADER_LEN + body.len();
    (&mut mmap[HEADER_LEN..body_end]).write_all(&body)?;
    mmap.flush()?;

    Ok(DiagIpcWriter {
        mmap,
        ring_offset,
        ring_capacity,
    })
}

/// Read-only view of an IPC file's runtime ring (Phase B live-tail).
///
/// Maps the file read-only and parses the ring header. [`latest_frame`] and
/// [`frames_written`] track the producer's `write_idx` across calls so a CLI can
/// poll for new frames (live tail).
pub struct RuntimeRingReader {
    mmap: Mmap,
    ring_offset: usize,
    capacity: usize,
    frame_len: usize,
}

impl RuntimeRingReader {
    /// Open `path`, validate the header, and locate the runtime-ring section.
    ///
    /// Returns `Ok(None)` if the file is a Phase-A file (no ring offset).
    pub fn open(path: &Path) -> io::Result<Option<Self>> {
        let file = OpenOptions::new().read(true).open(path)?;
        let mmap = unsafe { Mmap::map(&file)? };
        if mmap.len() < HEADER_LEN || mmap[0..8] != MAGIC {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "bad magic — not a PKDIAG file",
            ));
        }
        let ring_offset = read_u32(&mmap[28..32]) as usize;
        if ring_offset == 0 {
            return Ok(None); // Phase-A file: no runtime ring.
        }
        if ring_offset + diag_ring::RING_HEADER_LEN > mmap.len() {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "runtime-ring offset out of bounds",
            ));
        }
        let hdr = &mmap[ring_offset..ring_offset + diag_ring::RING_HEADER_LEN];
        if hdr[0..4] != diag_ring::RING_MAGIC {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "bad runtime-ring magic",
            ));
        }
        let capacity = read_u32(&hdr[8..12]) as usize;
        let frame_len = read_u32(&hdr[12..16]) as usize;
        if frame_len != diag_ring::FRAME_LEN {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!(
                    "runtime-ring frame_len {frame_len} != expected {}",
                    diag_ring::FRAME_LEN
                ),
            ));
        }
        Ok(Some(Self {
            mmap,
            ring_offset,
            capacity,
            frame_len,
        }))
    }

    /// Producer's published frame count (`write_idx`), Acquire-ordered.
    pub fn frames_written(&self) -> u64 {
        use std::sync::atomic::{AtomicU64, Ordering};
        let p = unsafe { self.mmap.as_ptr().add(self.ring_offset + 16) as *const AtomicU64 };
        unsafe { (*p).load(Ordering::Acquire) }
    }

    /// Frame-slot capacity.
    pub fn capacity(&self) -> usize {
        self.capacity
    }

    /// Read the frame at absolute producer index `idx` (must be within the last
    /// `capacity` published frames or it may read a torn/overwritten slot).
    pub fn frame_at(&self, idx: u64) -> RuntimeFrame {
        let slot = (idx as usize) & (self.capacity - 1);
        let off = self.ring_offset + diag_ring::RING_HEADER_LEN + slot * self.frame_len;
        RuntimeFrame::read_le(&self.mmap[off..off + self.frame_len])
    }

    /// Most recently published frame, or `None` if nothing written yet.
    pub fn latest_frame(&self) -> Option<RuntimeFrame> {
        let n = self.frames_written();
        if n == 0 {
            None
        } else {
            Some(self.frame_at(n - 1))
        }
    }
}

fn write_u32(buf: &mut [u8], v: u32) {
    buf[0..4].copy_from_slice(&v.to_le_bytes());
}

fn read_u32(buf: &[u8]) -> u32 {
    u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]])
}

#[cfg(test)]
mod tests {
    use super::*;
    use pedalkernel_rt::diag::{MnaStageSnapshot, NlPortLinearization};

    #[test]
    fn roundtrip_snapshot() {
        let snap = DiagSnapshot {
            pedal_name: "Test".into(),
            source_path: "/tmp/x.pedal".into(),
            sample_rate: 48000.0,
            stage_count: 2,
            stage_kinds: vec!["Wdf".into(), "MultiNl".into()],
            multinl: vec![MnaStageSnapshot {
                stage_index: 1,
                label: "Q1,Q2".into(),
                n_nl: 1,
                n_passive: 0,
                n_ports_total: 2,
                port_labels: vec!["NL0:Diode".into(), "adapted".into()],
                port_resistances: vec![1000.0, 2200.0],
                s_full: vec![0.1, 0.9, 0.9, 0.1],
                s_dim: 2,
                nl_linearization: vec![NlPortLinearization {
                    label: "NL0:Diode".into(),
                    device: "Diode".into(),
                    v_op: 0.6,
                    gm: 0.01,
                    r_companion: 100.0,
                    cross_gm: None,
                    port_resistance: 1000.0,
                }],
                nl_port_resistances: vec![1000.0],
                extraction_coeffs: Some(vec![0.5, 0.5]),
                extraction_vs: 1.0,
                vs_injection: None,
                output_port: 0,
                compensation: 1.0,
                supply_voltage: 9.0,
                bypass_serial: false,
            }],
        };

        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("diag.pkdiag");
        write_snapshot(&path, &snap).unwrap();
        let back = read_snapshot(&path).unwrap();
        assert_eq!(snap, back);
    }

    #[test]
    fn phase_a_file_has_no_ring() {
        // A Phase-A file (write_snapshot) must report no runtime ring.
        let snap = DiagSnapshot {
            pedal_name: "A".into(),
            source_path: "x".into(),
            sample_rate: 48000.0,
            stage_count: 0,
            stage_kinds: vec![],
            multinl: vec![],
        };
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("a.pkdiag");
        write_snapshot(&path, &snap).unwrap();
        assert!(RuntimeRingReader::open(&path).unwrap().is_none());
        // And the Phase-A reader still parses it.
        assert_eq!(read_snapshot(&path).unwrap(), snap);
    }

    #[test]
    fn ring_section_roundtrip() {
        use pedalkernel_rt::diag_ring::{OpPointRecord, RuntimeFrame, RuntimeRing};

        let snap = DiagSnapshot {
            pedal_name: "Ring".into(),
            source_path: "y".into(),
            sample_rate: 48000.0,
            stage_count: 1,
            stage_kinds: vec!["MultiNl".into()],
            multinl: vec![],
        };
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("ring.pkdiag");

        // Write file + ring, then drive frames through the rt ring over the
        // writer's region (same handoff as attach_diag_ring).
        let writer = write_snapshot_with_ring(&path, &snap, 8).unwrap();
        let mut ring =
            unsafe { RuntimeRing::init(writer.ring_ptr(), writer.ring_capacity()) };
        for i in 0..5u64 {
            let mut f = RuntimeFrame::default();
            f.block_counter = i;
            f.nr_solves = 256;
            f.n_op_records = 1;
            f.op_records[0] = OpPointRecord {
                stage_index: 0,
                group_index: 0,
                v_be: 0.36,
                v_ce: 21.6,
                i_c: 7.0e-6,
                i_b: 4.0e-7,
                gm: 0.00027,
                g_ce: 9.0e-8,
            };
            ring.push(&f);
        }
        writer.flush().unwrap();

        // The static snapshot still reads back.
        assert_eq!(read_snapshot(&path).unwrap(), snap);

        // The ring reads back the latest frame.
        let reader = RuntimeRingReader::open(&path).unwrap().unwrap();
        assert_eq!(reader.frames_written(), 5);
        let latest = reader.latest_frame().unwrap();
        assert_eq!(latest.block_counter, 4);
        assert_eq!(latest.nr_solves, 256);
        assert_eq!(latest.n_op_records, 1);
        assert!((latest.op_records[0].gm - 0.00027).abs() < 1e-9);
    }
}
