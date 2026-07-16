//! Bill-of-materials (BOM) generation from the parsed `.pedal` AST.
//!
//! Walks the (already `use(...)`-expanded) [`PedalDef`] component list and
//! produces one row per refdes plus an aggregated purchase view grouped by
//! `(category, value, model)` with summed quantities.
//!
//! Semantics match `tools/mouser_bom.py`: a `diode_pair` counts as **2**
//! physical diodes (and aggregates with single diodes of the same type).
//! Vendor part-number mapping intentionally stays in the Python tool.
//!
//! Rendering: [`to_table`] is the human-readable aligned view (per-refdes
//! rows followed by the aggregated view); [`to_csv`] is a machine-readable
//! CSV of the per-refdes rows; [`to_csv_aggregated`] is a CSV of the
//! aggregated lines.

use crate::compiler::component::Component;
use crate::compiler::components::DiodePair;
use crate::dsl::{PedalDef, PotTaper};
use crate::kicad::{format_eng, value_str};
use std::fmt::Write;

/// One BOM row for a single component instance (refdes).
#[derive(Debug, Clone, PartialEq)]
pub struct BomRow {
    /// Reference designator (the component id from the `.pedal` file).
    pub refdes: String,
    /// Component category (e.g. `resistor`, `potentiometer`, `PNP transistor`).
    pub category: String,
    /// Human-readable value / description (e.g. `4.7kΩ`, `Silicon`).
    pub value: String,
    /// Device model name when the component carries one (e.g. `AC128`).
    pub model: String,
    /// Physical part count for this refdes (`diode_pair` = 2 diodes).
    pub qty: usize,
}

/// One aggregated BOM line, grouped by `(category, value, model)`.
#[derive(Debug, Clone, PartialEq)]
pub struct BomLine {
    /// Component category shared by all refs on this line.
    pub category: String,
    /// Value / description shared by all refs on this line.
    pub value: String,
    /// Model name shared by all refs on this line.
    pub model: String,
    /// Total physical part count across all refs.
    pub qty: usize,
    /// Reference designators folded into this line, in netlist order.
    pub refs: Vec<String>,
}

/// A complete BOM: per-refdes rows plus the aggregated purchase view.
#[derive(Debug, Clone, PartialEq)]
pub struct Bom {
    /// Pedal name from the `.pedal` header.
    pub pedal_name: String,
    /// One row per component instance, in declaration order.
    pub rows: Vec<BomRow>,
    /// Aggregated lines grouped by `(category, value, model)`, first-seen order.
    pub lines: Vec<BomLine>,
}

impl Bom {
    /// Total physical part count (sum of row quantities).
    pub fn total_parts(&self) -> usize {
        self.rows.iter().map(|r| r.qty).sum()
    }
}

/// Human-readable taper name for the BOM description column.
fn taper_name(taper: PotTaper) -> &'static str {
    match taper {
        PotTaper::A => "audio",
        PotTaper::B => "linear",
        PotTaper::C => "reverse-log",
    }
}

/// Build one BOM row for a component.
fn row_for(refdes: &str, kind: &dyn Component) -> BomRow {
    let model = kind.model_name().unwrap_or("").to_string();

    // diode_pair = 2 physical diodes of the underlying type (mouser_bom.py
    // semantics). Normalize category/value so pairs aggregate with singles.
    if kind.as_any().downcast_ref::<DiodePair>().is_some() {
        let value = kind
            .diode_type()
            .map(|dt| format!("{dt:?}"))
            .unwrap_or_else(|| value_str(kind));
        return BomRow {
            refdes: refdes.to_string(),
            category: "diode".to_string(),
            value,
            model,
            qty: 2,
        };
    }

    // Pots: surface the taper in the description (it is a physically
    // different part — an A500k is not a B500k).
    if kind.is_pot() {
        if let (Some(r), Some(taper)) = (kind.resistance(), kind.pot_taper()) {
            return BomRow {
                refdes: refdes.to_string(),
                category: kind.type_tag().to_string(),
                value: format!("{} ({} taper)", format_eng(r, "\u{3a9}"), taper_name(taper)),
                model,
                qty: 1,
            };
        }
    }

    BomRow {
        refdes: refdes.to_string(),
        category: kind.type_tag().to_string(),
        value: value_str(kind),
        model,
        qty: 1,
    }
}

/// Build a [`Bom`] from an expanded [`PedalDef`].
///
/// Callers should pass a pedal loaded via
/// [`crate::dsl_expand::parse_and_expand_pedal_file`] so subcircuit
/// components are included.
pub fn build_bom(pedal: &PedalDef) -> Bom {
    let rows: Vec<BomRow> = pedal
        .components
        .iter()
        .map(|c| row_for(&c.id, c.kind.as_ref()))
        .collect();

    let mut lines: Vec<BomLine> = Vec::new();
    for row in &rows {
        if let Some(line) = lines
            .iter_mut()
            .find(|l| l.category == row.category && l.value == row.value && l.model == row.model)
        {
            line.qty += row.qty;
            line.refs.push(row.refdes.clone());
        } else {
            lines.push(BomLine {
                category: row.category.clone(),
                value: row.value.clone(),
                model: row.model.clone(),
                qty: row.qty,
                refs: vec![row.refdes.clone()],
            });
        }
    }

    Bom {
        pedal_name: pedal.name.clone(),
        rows,
        lines,
    }
}

// ---------------------------------------------------------------------------
// Rendering
// ---------------------------------------------------------------------------

/// Quote a CSV field when needed (comma, quote, or newline present).
fn csv_field(s: &str) -> String {
    if s.contains(',') || s.contains('"') || s.contains('\n') {
        format!("\"{}\"", s.replace('"', "\"\""))
    } else {
        s.to_string()
    }
}

/// CSV of the per-refdes rows: `refdes,category,value,model,qty`.
pub fn to_csv(bom: &Bom) -> String {
    let mut out = String::with_capacity(1024);
    out.push_str("refdes,category,value,model,qty\n");
    for row in &bom.rows {
        let _ = writeln!(
            out,
            "{},{},{},{},{}",
            csv_field(&row.refdes),
            csv_field(&row.category),
            csv_field(&row.value),
            csv_field(&row.model),
            row.qty
        );
    }
    out
}

/// CSV of the aggregated lines: `qty,category,value,model,refs`
/// (refs are space-separated).
pub fn to_csv_aggregated(bom: &Bom) -> String {
    let mut out = String::with_capacity(1024);
    out.push_str("qty,category,value,model,refs\n");
    for line in &bom.lines {
        let _ = writeln!(
            out,
            "{},{},{},{},{}",
            line.qty,
            csv_field(&line.category),
            csv_field(&line.value),
            csv_field(&line.model),
            csv_field(&line.refs.join(" "))
        );
    }
    out
}

/// Character count (not byte count) so `Ω`/`µ` don't break column alignment.
fn width(s: &str) -> usize {
    s.chars().count()
}

/// Pad `s` to `w` display characters.
fn pad(s: &str, w: usize) -> String {
    let mut out = s.to_string();
    for _ in width(s)..w {
        out.push(' ');
    }
    out
}

/// Render an aligned text table from `(header, rows)`.
fn render_table(headers: &[&str], rows: &[Vec<String>]) -> String {
    let mut widths: Vec<usize> = headers.iter().map(|h| width(h)).collect();
    for row in rows {
        for (i, cell) in row.iter().enumerate() {
            widths[i] = widths[i].max(width(cell));
        }
    }
    let mut out = String::new();
    let header_line: Vec<String> = headers
        .iter()
        .enumerate()
        .map(|(i, h)| pad(h, widths[i]))
        .collect();
    let _ = writeln!(out, "  {}", header_line.join("  ").trim_end());
    let rule: Vec<String> = widths.iter().map(|w| "-".repeat(*w)).collect();
    let _ = writeln!(out, "  {}", rule.join("  "));
    for row in rows {
        let cells: Vec<String> = row
            .iter()
            .enumerate()
            .map(|(i, c)| pad(c, widths[i]))
            .collect();
        let _ = writeln!(out, "  {}", cells.join("  ").trim_end());
    }
    out
}

/// Display placeholder for empty model cells.
fn dash(s: &str) -> String {
    if s.is_empty() {
        "-".to_string()
    } else {
        s.to_string()
    }
}

/// Human-readable aligned table: per-refdes rows followed by the
/// aggregated `(category, value, model)` view.
pub fn to_table(bom: &Bom) -> String {
    let mut out = String::with_capacity(2048);
    let _ = writeln!(out, "BOM: {}", bom.pedal_name);
    let _ = writeln!(
        out,
        "{} components, {} physical parts",
        bom.rows.len(),
        bom.total_parts()
    );
    let _ = writeln!(out);

    let rows: Vec<Vec<String>> = bom
        .rows
        .iter()
        .map(|r| {
            vec![
                r.refdes.clone(),
                r.category.clone(),
                r.value.clone(),
                dash(&r.model),
                r.qty.to_string(),
            ]
        })
        .collect();
    out.push_str(&render_table(
        &["Ref", "Category", "Value", "Model", "Qty"],
        &rows,
    ));

    let _ = writeln!(out);
    let _ = writeln!(out, "Aggregated (category / value / model):");
    let _ = writeln!(out);
    let lines: Vec<Vec<String>> = bom
        .lines
        .iter()
        .map(|l| {
            vec![
                l.qty.to_string(),
                l.category.clone(),
                l.value.clone(),
                dash(&l.model),
                l.refs.join(" "),
            ]
        })
        .collect();
    out.push_str(&render_table(
        &["Qty", "Category", "Value", "Model", "Refs"],
        &lines,
    ));
    out
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dsl_expand::parse_and_expand_pedal_file;
    use std::path::{Path, PathBuf};

    /// Search examples/ subdirectories for a file by name.
    fn find_example_file(filename: &str) -> PathBuf {
        fn walk(dir: &Path, target: &str) -> Option<PathBuf> {
            for entry in std::fs::read_dir(dir).ok()?.flatten() {
                let path = entry.path();
                if path.is_dir() {
                    if let Some(found) = walk(&path, target) {
                        return Some(found);
                    }
                } else if path.file_name().and_then(|n| n.to_str()) == Some(target) {
                    return Some(path);
                }
            }
            None
        }
        walk(Path::new("examples"), filename)
            .unwrap_or_else(|| panic!("example file not found: {filename}"))
    }

    /// Collect every `.pedal` file under examples/.
    fn all_example_pedals() -> Vec<PathBuf> {
        fn walk(dir: &Path, acc: &mut Vec<PathBuf>) {
            let Ok(rd) = std::fs::read_dir(dir) else {
                return;
            };
            for entry in rd.flatten() {
                let path = entry.path();
                if path.is_dir() {
                    walk(&path, acc);
                } else if path.extension().and_then(|e| e.to_str()) == Some("pedal") {
                    acc.push(path);
                }
            }
        }
        let mut acc = Vec::new();
        walk(Path::new("examples"), &mut acc);
        acc.sort();
        acc
    }

    #[test]
    fn fuzz_face_bom_rows() {
        let path = find_example_file("fuzz_face.pedal");
        let pedal = parse_and_expand_pedal_file(&path)
            .unwrap_or_else(|e| panic!("failed to load {}: {e}", path.display()));
        let bom = build_bom(&pedal);

        // Real Fuzz Face BOM: 4 resistors, 3 caps, 2x AC128 PNP, 2 pots.
        assert_eq!(bom.rows.len(), 11, "11 components total");
        let count = |cat: &str| bom.rows.iter().filter(|r| r.category == cat).count();
        assert_eq!(count("resistor"), 4, "4 resistors (R1-R4)");
        assert_eq!(count("capacitor"), 3, "3 caps (C1-C3)");
        assert_eq!(count("PNP transistor"), 2, "2 PNP transistors");
        assert_eq!(count("potentiometer"), 2, "2 pots (Fuzz, Volume)");

        // Both PNPs are AC128 and aggregate into a single qty-2 line.
        let pnps: Vec<&BomRow> = bom
            .rows
            .iter()
            .filter(|r| r.category == "PNP transistor")
            .collect();
        assert!(pnps.iter().all(|r| r.model == "AC128"), "PNPs are AC128");
        let pnp_line = bom
            .lines
            .iter()
            .find(|l| l.category == "PNP transistor" && l.model == "AC128")
            .expect("aggregated AC128 line");
        assert_eq!(pnp_line.qty, 2);
        assert_eq!(pnp_line.refs, vec!["Q1", "Q2"]);

        // Pot descriptions carry the taper.
        let fuzz = bom.rows.iter().find(|r| r.refdes == "Fuzz").unwrap();
        assert!(
            fuzz.value.contains("linear taper"),
            "Fuzz pot description shows taper: {}",
            fuzz.value
        );
        let vol = bom.rows.iter().find(|r| r.refdes == "Volume").unwrap();
        assert!(
            vol.value.contains("audio taper"),
            "Volume pot description shows taper: {}",
            vol.value
        );
        assert!(
            vol.value.contains("500.0k\u{3a9}"),
            "Volume pot value shows resistance: {}",
            vol.value
        );
    }

    #[test]
    fn diode_pair_counts_as_two_diodes() {
        // mouser_bom.py semantics: diode_pair = 2 physical diodes, and it
        // aggregates with single diodes of the same type.
        let src = r#"
pedal "Pair Test" {
  components {
    D1: diode_pair(silicon)
    D2: diode(silicon)
    R1: resistor(10k)
  }
  nets {
    in -> D1.a
    D1.b -> D2.a
    D2.b -> R1.a
    R1.b -> out
  }
}
"#;
        let pedal = crate::dsl::parse_pedal_file(src).expect("parse");
        let bom = build_bom(&pedal);

        let d1 = bom.rows.iter().find(|r| r.refdes == "D1").unwrap();
        assert_eq!(d1.qty, 2, "diode_pair is 2 physical diodes");
        assert_eq!(d1.category, "diode");
        assert_eq!(d1.value, "Silicon");

        let line = bom
            .lines
            .iter()
            .find(|l| l.category == "diode" && l.value == "Silicon")
            .expect("aggregated silicon diode line");
        assert_eq!(line.qty, 3, "pair (2) + single (1) merge to one line");
        assert_eq!(line.refs, vec!["D1", "D2"]);
        assert_eq!(bom.total_parts(), 4);
    }

    /// Mirror of `all_pedal_files_export_kicad`: BOM generation must render
    /// sensibly over every example pedal — tubes, transformers, BBDs,
    /// photocouplers, tape heads, LFO/envelope followers, switched
    /// components — without panicking.
    #[test]
    fn all_example_pedals_build_bom() {
        let files = all_example_pedals();
        assert!(
            files.len() >= 25,
            "expected the full examples corpus, found {}",
            files.len()
        );
        for path in files {
            let pedal = parse_and_expand_pedal_file(&path)
                .unwrap_or_else(|e| panic!("failed to load {}: {e}", path.display()));
            let bom = build_bom(&pedal);
            assert_eq!(
                bom.rows.len(),
                pedal.components.len(),
                "{}: one row per component",
                path.display()
            );

            let table = to_table(&bom);
            let csv = to_csv(&bom);
            let agg = to_csv_aggregated(&bom);
            assert!(
                table.contains(&pedal.name),
                "{}: table header",
                path.display()
            );
            for comp in &pedal.components {
                assert!(
                    csv.contains(&comp.id),
                    "{}: refdes {} missing from CSV",
                    path.display(),
                    comp.id
                );
            }
            // Every row folds into exactly one aggregated line.
            let line_qty: usize = bom.lines.iter().map(|l| l.qty).sum();
            assert_eq!(line_qty, bom.total_parts(), "{}: qty sums", path.display());
            assert!(!agg.is_empty());

            // No row renders as an empty/placeholder value.
            for row in &bom.rows {
                assert!(
                    !row.value.trim().is_empty(),
                    "{}: {} has empty value",
                    path.display(),
                    row.refdes
                );
                assert!(
                    !row.category.trim().is_empty(),
                    "{}: {} has empty category",
                    path.display(),
                    row.refdes
                );
            }
        }
    }
}
