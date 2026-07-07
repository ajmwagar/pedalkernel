//! Boundary-load decision table — validate-side visibility tests (bead
//! pedalkernel-lkf1.2).
//!
//! Consumes `CompiledPedal::boundary_loads` (PR #190): the compile-time record
//! of what hangs electrically downstream of each stage's OUTPUT boundary
//! (`BoundaryLoadSummary`) and what the compiler did about it
//! (`BoundaryLoadDisposition`). WHY: downstream impedance never reflects back
//! into an upstream stage's scattering — a stage whose output boundary is left
//! `Unloaded` solves against an open circuit, and unloaded boundaries and
//! marginal op-points travel together (live cases: Klon +16 dBFS over on macOS
//! / NaN-flush to silence on x86/WSL; RAT; Pultec).
//!
//! Table semantics locked here (mirrors the compiler-side tests in
//! `pedalkernel/src/compiler/boundary_load.rs`):
//!
//!   * an EMPTY table means no feedback group ever reached the general-MNA
//!     call site (e.g. the whole pedal compiled via blockwise/other paths) —
//!     that ABSENCE is itself a triage signal, distinct from a populated row
//!     with `Unloaded{reason}` (analyzed, declined, reason recorded);
//!   * BA283 (pro, `skip_if_missing!` house pattern): exactly ONE row —
//!     `SeriesCapIntoLoad{Cout → RL}` + `FusedUpstream` (the fusion that
//!     closed the loaded-AC gap to LEVEL −0.05 dB);
//!   * a public circuit documents CURRENT REALITY for the non-fused case;
//!   * the corpus sweep (`PK_BOUNDARY_TABLE=1`) prints one line per example
//!     pedal — the greppable fleet view of unloaded output boundaries. A
//!     reporting tool, NOT a gate.

use pedalkernel::compiler::{compile_pedal, CompiledPedal};
use pedalkernel::dsl::parse_pedal_file;
use pedalkernel_rt::processor::{BoundaryLoadDisposition, BoundaryLoadSummary};
use pedalkernel_validate::pro_pedal::load_pro_pedal_sub;
use pedalkernel_validate::report::{render_boundary_loads, unloaded_output_flag};
use pedalkernel_validate::skip_if_missing;
use std::path::{Path, PathBuf};

const SR: f64 = 48_000.0;

fn manifest() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

fn compile_source(name: &str, source: &str) -> CompiledPedal {
    let def = parse_pedal_file(source).unwrap_or_else(|e| panic!("parse {name}: {e}"));
    compile_pedal(&def, SR).unwrap_or_else(|e| panic!("compile {name}: {e}"))
}

fn compile_public(rel: &str) -> CompiledPedal {
    let p = manifest().join(rel);
    let source = std::fs::read_to_string(&p).unwrap_or_else(|e| panic!("read {p:?}: {e}"));
    compile_source(rel, &source)
}

/// BA283 (pro pedal): the known-fused boundary. Exactly one analyzed output
/// boundary — the classic `Cout → RL` output network — fused into the upstream
/// multi-BJT MNA stage, so the load-current demand is part of the solved
/// network (LEVEL −0.05 dB / ΔTHD 0.4 dB / tilt 0.79 dB vs the loaded golden).
/// Skips cleanly when pedalkernel-pro is absent (public CI).
#[test]
fn ba283_output_boundary_fuses_series_cap_into_load() {
    let source = skip_if_missing!(
        load_pro_pedal_sub("pedals/500/1073/sub/ba283.pedal"),
        "pedals/500/1073/sub/ba283.pedal"
    );
    let proc = compile_source("neve1073_ba283", &source);
    println!(
        "{}",
        render_boundary_loads(&[(
            "neve1073_ba283".to_string(),
            proc.boundary_loads.clone()
        )])
    );

    assert_eq!(
        proc.boundary_loads.len(),
        1,
        "BA283: exactly one analyzed output boundary expected, got {:?}",
        proc.boundary_loads
    );
    let b = &proc.boundary_loads[0];
    assert!(
        matches!(&b.model, BoundaryLoadSummary::SeriesCapIntoLoad { .. }),
        "BA283 output network is the classic Cout→RL SeriesCapIntoLoad, got {:?}",
        b.model
    );
    assert_eq!(
        b.disposition,
        BoundaryLoadDisposition::FusedUpstream,
        "BA283 output load must be fused into the upstream MNA stage"
    );
    // A fused boundary is NOT an unloaded output — no triage flag.
    assert!(
        unloaded_output_flag(&proc.boundary_loads).is_none(),
        "fused boundary must not raise the unloaded-output flag"
    );
}

/// Public circuit that does NOT fuse — documents CURRENT REALITY.
///
/// `fuzz_face_pnp` DOES reach the general-MNA call site (its 2-PNP feedback
/// group declines blockwise), and its trailing `C2 → RL` output network IS the
/// recognized SeriesCapIntoLoad shape — but the BA283 policy gate declines
/// with `gate:no-dc-qpoint-seed` (the PNP pair's compile-time DC q-point solve
/// does not produce the seed the fusion requires), so the boundary is left
/// OPEN and the row records the reason. When the gate widens to cover this
/// circuit, this test SHOULD fail — update it to lock the new disposition.
#[test]
fn fuzz_face_pnp_output_boundary_documents_unloaded_reality() {
    let proc = compile_public("circuits/active/fuzz_face_pnp.pedal");
    println!(
        "{}",
        render_boundary_loads(&[(
            "fuzz_face_pnp".to_string(),
            proc.boundary_loads.clone()
        )])
    );

    assert_eq!(
        proc.boundary_loads.len(),
        1,
        "fuzz_face_pnp: one analyzed output boundary expected, got {:?}",
        proc.boundary_loads
    );
    let b = &proc.boundary_loads[0];
    match &b.disposition {
        BoundaryLoadDisposition::Unloaded { reason } => {
            assert_eq!(
                reason, "gate:no-dc-qpoint-seed",
                "fuzz_face_pnp decline reason changed — current reality moved, \
                 update this lock"
            );
        }
        other => panic!(
            "fuzz_face_pnp output boundary is documented UNLOADED today; \
             disposition changed to {other:?} — the gate widened, update this test"
        ),
    }
    // The open boundary must raise the triage flag next to any dashboard verdict.
    let flag = unloaded_output_flag(&proc.boundary_loads)
        .expect("unloaded output boundary must raise the triage flag");
    assert!(
        flag.contains("gate:no-dc-qpoint-seed"),
        "flag should carry the decline reason, got: {flag}"
    );
}

/// Companion reality-lock for the OTHER absence mode: `si_fb_amp` (the public
/// BA283 proxy) compiles its feedback group via BLOCKWISE — the group never
/// reaches the general-MNA call site, so the table is EMPTY (measured). That
/// is DISTINCT from a populated `Unloaded{reason}` row: nothing analyzed the
/// output boundary at all, and the `Cout → RL` load hangs off a blockwise
/// stage that never feels it. The triage flag must still fire (unanalyzed ⇒
/// open boundary). If blockwise routing changes and si_fb_amp starts reaching
/// the call site, this test SHOULD fail — update it to lock the new reality.
#[test]
fn si_fb_amp_boundary_table_is_empty_blockwise_path() {
    let proc = compile_public("circuits/active/si_fb_amp.pedal");
    println!(
        "{}",
        render_boundary_loads(&[("si_fb_amp".to_string(), proc.boundary_loads.clone())])
    );

    assert!(
        proc.boundary_loads.is_empty(),
        "si_fb_amp is documented EMPTY-table today (blockwise path, no \
         general-MNA analysis); the table now reads {:?} — routing changed, \
         update this lock",
        proc.boundary_loads
    );
    // The empty table is itself a triage signal: the output boundary was
    // never analyzed, so the flag must fire (distinct wording from Unloaded).
    let flag = unloaded_output_flag(&proc.boundary_loads)
        .expect("empty table must raise the unanalyzed-output flag");
    assert!(
        flag.contains("empty table"),
        "empty-table flag should name the absence mode, got: {flag}"
    );
}

fn collect_pedals(dir: &Path, out: &mut Vec<PathBuf>) {
    let Ok(entries) = std::fs::read_dir(dir) else {
        return;
    };
    for entry in entries.flatten() {
        let path = entry.path();
        if path.is_dir() {
            collect_pedals(&path, out);
        } else if path.extension().is_some_and(|e| e == "pedal") {
            out.push(path);
        }
    }
}

/// Fleet sweep: compile every example pedal under `pedalkernel/examples/pedals/`
/// and print ONE line per pedal — name, output-boundary disposition, detail — so
/// the fleet can be grepped for silently-unloaded output boundaries (the
/// Klon/RAT/Pultec WSL-NaN triage lever). REPORTING TOOL, not a gate: compile
/// failures are printed and counted, never fail the test. Env-gated:
///
/// ```text
/// PK_BOUNDARY_TABLE=1 cargo test -p pedalkernel-validate --test boundary_load_table \
///   corpus_boundary_load_sweep -- --nocapture
/// ```
#[test]
fn corpus_boundary_load_sweep() {
    if std::env::var("PK_BOUNDARY_TABLE").as_deref() != Ok("1") {
        eprintln!("  SKIP corpus_boundary_load_sweep: set PK_BOUNDARY_TABLE=1 to run");
        return;
    }
    let root = manifest().join("../pedalkernel/examples/pedals");
    let mut pedals = Vec::new();
    collect_pedals(&root, &mut pedals);
    pedals.sort();
    assert!(!pedals.is_empty(), "no example pedals found under {root:?}");

    let mut fused = 0usize;
    let mut unloaded = 0usize;
    let mut empty = 0usize;
    let mut errors = 0usize;
    println!(
        "\n── corpus boundary-load sweep: {} pedals under examples/pedals/ ──",
        pedals.len()
    );
    for path in &pedals {
        let name = path
            .strip_prefix(&root)
            .unwrap_or(path)
            .display()
            .to_string();
        let source = std::fs::read_to_string(path)
            .unwrap_or_else(|e| panic!("read {path:?}: {e}"));
        // Fast compile: K-tables skipped (runtime-only precompute; stage
        // routing and the boundary decision are unaffected).
        let compiled = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            let def = parse_pedal_file(&source).map_err(|e| e.to_string())?;
            let options = pedalkernel::compiler::CompileOptions {
                skip_k_tables: true,
                ..Default::default()
            };
            pedalkernel::compiler::compile_pedal_with_options(&def, SR, options)
        }));
        let (disposition, detail) = match compiled {
            Err(panic) => {
                errors += 1;
                let msg = panic
                    .downcast_ref::<String>()
                    .cloned()
                    .or_else(|| panic.downcast_ref::<&str>().map(|s| s.to_string()))
                    .unwrap_or_else(|| "panic".to_string());
                ("compile-panic".to_string(), msg.replace('\n', " "))
            }
            Ok(Err(e)) => {
                errors += 1;
                ("compile-error".to_string(), e.replace('\n', " "))
            }
            Ok(Ok(proc)) => {
                let loads = &proc.boundary_loads;
                if loads.is_empty() {
                    empty += 1;
                    (
                        "empty-table".to_string(),
                        "no-analysis (no feedback group reached general-MNA)".to_string(),
                    )
                } else if loads
                    .iter()
                    .any(|b| b.disposition == BoundaryLoadDisposition::FusedUpstream)
                {
                    fused += 1;
                    let models: Vec<String> = loads
                        .iter()
                        .filter(|b| b.disposition == BoundaryLoadDisposition::FusedUpstream)
                        .map(|b| pedalkernel_validate::report::format_boundary_model(&b.model))
                        .collect();
                    ("FusedUpstream".to_string(), models.join(" | "))
                } else {
                    unloaded += 1;
                    let mut reasons: Vec<String> = loads
                        .iter()
                        .filter_map(|b| match &b.disposition {
                            BoundaryLoadDisposition::Unloaded { reason } => Some(reason.clone()),
                            BoundaryLoadDisposition::FusedUpstream => None,
                        })
                        .collect();
                    reasons.sort();
                    reasons.dedup();
                    ("Unloaded".to_string(), reasons.join(", "))
                }
            }
        };
        println!("  {name:<40} {disposition:<14} {detail}");
    }
    println!(
        "── sweep summary: {} pedals — {fused} fused, {unloaded} unloaded, \
         {empty} empty-table, {errors} compile-error ──\n   ({} of {} output \
         boundaries are OPEN: unloaded + empty-table — the WSL/NaN triage set)",
        pedals.len(),
        unloaded + empty,
        pedals.len(),
    );
}
