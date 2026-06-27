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
}
