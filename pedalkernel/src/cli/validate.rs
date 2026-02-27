//! CLI `validate` subcommand — lint `.pedal` files for common issues.
//!
//! Supports single files, directories (recursive), and glob patterns.
//! Optional `--fix` mode applies auto-fixable changes in place.

use std::path::{Path, PathBuf};

use pedalkernel::compiler::validate::{validate_pedal, PedalWarning, Severity};
use pedalkernel::dsl::parse_pedal_file;

/// Run the validate subcommand.
pub fn run(paths: &[String], fix: bool) {
    let files = collect_pedal_files(paths);

    if files.is_empty() {
        eprintln!("No .pedal files found.");
        std::process::exit(1);
    }

    let mut total_info = 0usize;
    let mut total_warn = 0usize;
    let mut total_err = 0usize;
    let mut files_with_issues = 0usize;
    let mut files_fixed = 0usize;

    for file in &files {
        let src = match std::fs::read_to_string(file) {
            Ok(s) => s,
            Err(e) => {
                eprintln!("\x1b[31merror\x1b[0m: cannot read {}: {}", file.display(), e);
                total_err += 1;
                continue;
            }
        };

        let pedal = match parse_pedal_file(&src) {
            Ok(p) => p,
            Err(e) => {
                eprintln!(
                    "\n\x1b[1m{}\x1b[0m",
                    file.display()
                );
                eprintln!(
                    "  \x1b[31merror\x1b[0m[parse-error]: {}",
                    e
                );
                total_err += 1;
                continue;
            }
        };

        let warnings = validate_pedal(&pedal);
        if warnings.is_empty() {
            continue;
        }

        files_with_issues += 1;
        eprintln!("\n\x1b[1m{}\x1b[0m", file.display());

        for w in &warnings {
            let (color, tag) = match w.severity {
                Severity::Info => ("\x1b[36m", "info"),
                Severity::Warning => ("\x1b[33m", "warning"),
                Severity::Error => ("\x1b[31m", "error"),
            };
            eprintln!(
                "  {color}{tag}\x1b[0m[{}]: {}",
                w.code, w.message
            );
            match w.severity {
                Severity::Info => total_info += 1,
                Severity::Warning => total_warn += 1,
                Severity::Error => total_err += 1,
            }
        }

        // Auto-fix if requested
        if fix {
            if let Some(fixed) = try_autofix(&src, &warnings) {
                if fixed != src {
                    match std::fs::write(file, &fixed) {
                        Ok(()) => {
                            eprintln!("  \x1b[32mfixed\x1b[0m: applied auto-fixes");
                            files_fixed += 1;
                        }
                        Err(e) => {
                            eprintln!("  \x1b[31merror\x1b[0m: cannot write {}: {}", file.display(), e);
                        }
                    }
                }
            }
        }
    }

    // Summary
    eprintln!();
    let clean = files.len() - files_with_issues;
    if files_with_issues == 0 {
        eprintln!(
            "\x1b[32m✓\x1b[0m {} file{} validated clean",
            files.len(),
            if files.len() == 1 { "" } else { "s" }
        );
    } else {
        eprintln!(
            "Checked {} file{}: {} clean, {} with issues",
            files.len(),
            if files.len() == 1 { "" } else { "s" },
            clean,
            files_with_issues,
        );
        if total_err > 0 {
            eprint!("  \x1b[31m{} error{}\x1b[0m", total_err, if total_err == 1 { "" } else { "s" });
        }
        if total_warn > 0 {
            if total_err > 0 { eprint!(", "); }
            eprint!("\x1b[33m{} warning{}\x1b[0m", total_warn, if total_warn == 1 { "" } else { "s" });
        }
        if total_info > 0 {
            if total_err > 0 || total_warn > 0 { eprint!(", "); }
            eprint!("\x1b[36m{} info\x1b[0m", total_info);
        }
        eprintln!();
        if fix && files_fixed > 0 {
            eprintln!(
                "  \x1b[32mFixed {} file{}\x1b[0m",
                files_fixed,
                if files_fixed == 1 { "" } else { "s" }
            );
        }
    }

    if total_err > 0 {
        std::process::exit(1);
    }
}

/// Collect `.pedal` files from paths — supports files, directories (recursive), and globs.
fn collect_pedal_files(paths: &[String]) -> Vec<PathBuf> {
    let mut files = Vec::new();

    for path_str in paths {
        // Check if it's a glob pattern
        if path_str.contains('*') || path_str.contains('?') || path_str.contains('[') {
            match glob_pedal_files(path_str) {
                Ok(matched) => files.extend(matched),
                Err(e) => eprintln!("warning: bad glob pattern '{}': {}", path_str, e),
            }
            continue;
        }

        let path = Path::new(path_str);
        if path.is_file() {
            if path.extension().map_or(false, |e| e == "pedal") {
                files.push(path.to_path_buf());
            } else {
                eprintln!("warning: skipping non-.pedal file: {}", path.display());
            }
        } else if path.is_dir() {
            walk_dir(path, &mut files);
        } else {
            eprintln!("warning: path not found: {}", path.display());
        }
    }

    files.sort();
    files.dedup();
    files
}

/// Recursively walk a directory for `.pedal` files.
fn walk_dir(dir: &Path, files: &mut Vec<PathBuf>) {
    if let Ok(entries) = std::fs::read_dir(dir) {
        for entry in entries.flatten() {
            let path = entry.path();
            if path.is_dir() {
                walk_dir(&path, files);
            } else if path.extension().map_or(false, |e| e == "pedal") {
                files.push(path);
            }
        }
    }
}

/// Simple glob matching for `.pedal` files.
/// Supports `*` (any chars in filename) and `**` (recursive directories).
fn glob_pedal_files(pattern: &str) -> Result<Vec<PathBuf>, String> {
    // Split on ** to handle recursive globbing
    let mut files = Vec::new();

    if pattern.contains("**") {
        // Extract the base directory (before **) and the filename pattern (after **)
        let parts: Vec<&str> = pattern.splitn(2, "**").collect();
        let base_dir = if parts[0].is_empty() || parts[0] == "/" {
            Path::new(".")
        } else {
            Path::new(parts[0].trim_end_matches('/'))
        };
        let file_pattern = parts.get(1).map(|s| s.trim_start_matches('/')).unwrap_or("*.pedal");

        fn walk_glob(dir: &Path, pattern: &str, files: &mut Vec<PathBuf>) {
            if let Ok(entries) = std::fs::read_dir(dir) {
                for entry in entries.flatten() {
                    let path = entry.path();
                    if path.is_dir() {
                        walk_glob(&path, pattern, files);
                    } else if matches_simple_glob(
                        path.file_name().unwrap_or_default().to_str().unwrap_or(""),
                        pattern,
                    ) {
                        files.push(path);
                    }
                }
            }
        }

        walk_glob(base_dir, file_pattern, &mut files);
    } else {
        // Simple glob in a single directory
        let dir = Path::new(pattern)
            .parent()
            .unwrap_or(Path::new("."));
        let file_pattern = Path::new(pattern)
            .file_name()
            .and_then(|f| f.to_str())
            .unwrap_or("*.pedal");

        if let Ok(entries) = std::fs::read_dir(dir) {
            for entry in entries.flatten() {
                let path = entry.path();
                if path.is_file() {
                    if let Some(name) = path.file_name().and_then(|f| f.to_str()) {
                        if matches_simple_glob(name, file_pattern) {
                            files.push(path);
                        }
                    }
                }
            }
        }
    }

    Ok(files)
}

/// Simple glob matching: `*` matches any sequence, `?` matches one char.
fn matches_simple_glob(name: &str, pattern: &str) -> bool {
    let mut ni = name.chars().peekable();
    let mut pi = pattern.chars().peekable();

    fn match_inner(
        name: &mut std::iter::Peekable<std::str::Chars>,
        pattern: &mut std::iter::Peekable<std::str::Chars>,
    ) -> bool {
        loop {
            match (pattern.peek(), name.peek()) {
                (None, None) => return true,
                (None, Some(_)) => return false,
                (Some('*'), _) => {
                    pattern.next();
                    // Try matching zero or more chars
                    let remaining_pattern: String = pattern.collect();
                    let remaining_name: String = std::iter::once(
                        name.peek().copied()
                    )
                    .flatten()
                    .chain(name.clone())
                    .collect();

                    // Try every suffix of name against remaining pattern
                    for i in 0..=remaining_name.len() {
                        if matches_simple_glob(&remaining_name[i..], &remaining_pattern) {
                            return true;
                        }
                    }
                    return false;
                }
                (Some('?'), Some(_)) => {
                    pattern.next();
                    name.next();
                }
                (Some(pc), Some(nc)) if *pc == *nc => {
                    pattern.next();
                    name.next();
                }
                _ => return false,
            }
        }
    }

    match_inner(&mut ni, &mut pi)
}

// ═══════════════════════════════════════════════════════════════════════════
// Auto-fix
// ═══════════════════════════════════════════════════════════════════════════

/// Try to auto-fix known issues in the source text.
/// Returns `Some(fixed_source)` if any fixes were applied, `None` if nothing to fix.
fn try_autofix(src: &str, warnings: &[PedalWarning]) -> Option<String> {
    let mut result = src.to_string();
    let mut changed = false;

    for w in warnings {
        match w.code {
            "unknown-pin" => {
                // Try to extract the bad pin reference and suggest a fix
                // Pattern: "Pin 'COMP.BAD_PIN' is not a recognized pin"
                if let Some(fix) = extract_pin_fix(&w.message) {
                    let old = format!("{}.{}", fix.component, fix.old_pin);
                    let new = format!("{}.{}", fix.component, fix.new_pin);
                    if result.contains(&old) {
                        result = result.replace(&old, &new);
                        changed = true;
                    }
                }
            }
            _ => {}
        }
    }

    if changed { Some(result) } else { None }
}

struct PinFix {
    component: String,
    old_pin: String,
    new_pin: String,
}

/// Parse a warning message to extract an auto-fixable pin rename.
fn extract_pin_fix(message: &str) -> Option<PinFix> {
    // "Pin 'PC1.ldr_a' is not a recognized pin — expected one of: a, b, led"
    let pin_start = message.find("Pin '")? + 5;
    let pin_end = message[pin_start..].find('\'')?;
    let full_pin = &message[pin_start..pin_start + pin_end];
    let dot = full_pin.find('.')?;
    let component = full_pin[..dot].to_string();
    let old_pin = full_pin[dot + 1..].to_string();

    // Known auto-fixable renames
    let new_pin = match old_pin.as_str() {
        "ldr_a" => "a",
        "ldr_b" => "b",
        _ => return None,
    };

    Some(PinFix {
        component,
        old_pin,
        new_pin: new_pin.to_string(),
    })
}
