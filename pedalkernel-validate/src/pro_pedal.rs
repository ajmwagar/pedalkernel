//! Helpers for loading proprietary `.pedal` files from the pedalkernel-pro
//! private repository at test time.
//!
//! The `.pedal` sources are proprietary and must **never** be committed to the
//! public engine repo, nor embedded via `include_str!` (that would fail to
//! compile on public CI where the pro repo is absent).
//!
//! # Usage
//!
//! ```rust,ignore
//! use pedalkernel_validate::pro_pedal::{load_pro_pedal_sub, skip_if_missing};
//!
//! #[test]
//! fn my_test() {
//!     let source = skip_if_missing!(
//!         load_pro_pedal_sub("pedals/legends/screamer.pedal"),
//!         "pedals/legends/screamer.pedal"
//!     );
//!     // use source…
//! }
//! ```
//!
//! # CI behaviour
//!
//! When the pro repo is absent (public CI, forks, contributor machines), the
//! `load_pro_pedal_sub` function returns `None`.  The `skip_if_missing!` macro
//! then calls `return` — causing the test to exit silently with no failure.
//! Rust's test harness reports such a test as passed (not failed, not skipped
//! with the ignored attribute).
//!
//! # Mirror note
//!
//! `pedalkernel::compiler::tb303_decomposition_tests::load_pro_pedal_sub` mirrors
//! this logic for tests in the `pedalkernel` crate.  A true shared crate is not
//! possible because `pedalkernel-validate` depends on `pedalkernel`, so a
//! reverse dependency would be circular.

/// Try to load a file at `pro_sub_path` (relative to the pedalkernel-pro repo
/// root, e.g. `"crates/acidattack/acidattack-core/tb303_filter.pedal"` or
/// `"pedals/legends/screamer.pedal"`).
///
/// Tries several candidate `../` depths from `CARGO_MANIFEST_DIR` so that tests
/// work in both normal checkout and git-worktree layouts.
///
/// Returns `None` — never panics — when the pro repo is absent, so callers can
/// skip gracefully via [`skip_if_missing!`].
pub fn load_pro_pedal_sub(pro_sub_path: &str) -> Option<String> {
    let manifest_dir = env!("CARGO_MANIFEST_DIR");
    // Try 2..=6 levels of `../` to cover:
    //   2: normal checkout  (pedalkernel-pro/ and pedalkernel/ are siblings)
    //   3: worktree layout  (.worktrees/<name>/ adds one extra level)
    //   4..=6: deeper nesting in case of unusual CI layouts
    for levels in 2..=6 {
        let prefix: String = "../".repeat(levels);
        let candidate = format!("{manifest_dir}/{prefix}pedalkernel-pro/{pro_sub_path}");
        if let Ok(s) = std::fs::read_to_string(&candidate) {
            eprintln!(
                "  loaded {} ({} bytes) from {candidate}",
                pro_sub_path,
                s.len()
            );
            return Some(s);
        }
    }
    None
}

/// Early-return a test when a pro `.pedal` file is absent.
///
/// ```rust,ignore
/// let source = skip_if_missing!(
///     load_pro_pedal_sub("pedals/legends/screamer.pedal"),
///     "pedals/legends/screamer.pedal"
/// );
/// ```
///
/// Expands to `match $source { Some(s) => s, None => { eprintln!(…); return; } }`.
/// The enclosing `#[test]` function returns `()`, so the harness records a pass.
#[macro_export]
macro_rules! skip_if_missing {
    ($source:expr, $name:expr) => {
        match $source {
            Some(s) => s,
            None => {
                eprintln!("  SKIP: {} not found (pro repo absent)", $name);
                return;
            }
        }
    };
}
