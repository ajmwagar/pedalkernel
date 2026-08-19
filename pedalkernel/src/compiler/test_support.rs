//! Shared test-only helpers for the `compiler::*_tests` modules.
//!
//! The engine test suite loads a number of `.pedal` fixtures from the private
//! `pedalkernel-pro` repo, which lives *next to* the `pedalkernel` checkout.
//! Hand-building that path with a fixed number of `../` levels only works for
//! one checkout layout: a plain checkout needs two levels, while a git worktree
//! under `.worktrees/<name>/` needs three.  The helpers here probe a range of
//! depths so tests behave identically in either layout.

use std::path::PathBuf;

/// Number of `../` levels probed when locating the `pedalkernel-pro` repo.
const PRO_REPO_DEPTHS: std::ops::RangeInclusive<usize> = 2..=6;

/// Resolve a path inside the `pedalkernel-pro` repo (e.g.
/// `"pedals/legends/screamer.pedal"` or `"pedals/legends"`), returning the
/// first candidate that exists.
///
/// Tries several `../` depths from `CARGO_MANIFEST_DIR` so that tests work in
/// both a normal checkout and a git-worktree layout.
pub(crate) fn pro_repo_path(pro_sub_path: &str) -> Option<PathBuf> {
    let manifest_dir = env!("CARGO_MANIFEST_DIR");
    for levels in PRO_REPO_DEPTHS {
        let prefix: String = "../".repeat(levels);
        let candidate = PathBuf::from(format!(
            "{manifest_dir}/{prefix}pedalkernel-pro/{pro_sub_path}"
        ));
        if candidate.exists() {
            return Some(candidate);
        }
    }
    None
}

/// Try to load a file at `pro_sub_path` (relative to the `pedalkernel-pro` repo
/// root, e.g. `"crates/acidattack/acidattack-core/tb303_filter.pedal"` or
/// `"pedals/legends/screamer.pedal"`).
///
/// This is the canonical shared helper for all pro-repo file loading in the
/// engine test suite.  `pedalkernel_validate::pro_pedal::load_pro_pedal_sub`
/// mirrors this logic for tests in the validate crate (circular dep prevents
/// sharing a single crate).
pub(crate) fn load_pro_pedal_sub(pro_sub_path: &str) -> Option<String> {
    let path = pro_repo_path(pro_sub_path)?;
    let s = std::fs::read_to_string(&path).ok()?;
    eprintln!(
        "  loaded {} ({} bytes) from {}",
        pro_sub_path,
        s.len(),
        path.display()
    );
    Some(s)
}

/// Convenience wrapper: load `pedals/legends/{name}.pedal` from the pro repo.
pub(crate) fn load_legend_source(name: &str) -> Option<String> {
    load_pro_pedal_sub(&format!("pedals/legends/{name}.pedal"))
}
