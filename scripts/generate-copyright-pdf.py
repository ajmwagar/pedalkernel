#!/usr/bin/env python3
"""
Generate a PDF for U.S. Copyright Office source-code registration.

The Copyright Office requires a source-code deposit to accompany software
registrations. For works longer than 50 pages, they accept a "first 25 +
last 25 pages" deposit. This script produces that deposit automatically
from the repo's core Rust sources.

File selection is priority-ordered (see FILE_GROUPS below). Globs that
match nothing are silently skipped, so the script works across branches
and releases even as the file layout evolves.

Page formatting follows the Copyright Office conventions:
  - One-sided US Letter, 1-inch margins
  - Monospace (Courier) 8pt for source, 10-11pt for headings
  - Page header shows "PedalKernel vX.Y.Z" and the current file path
  - Page footer shows the copyright notice and page number

Usage:
    scripts/generate-copyright-pdf.py [-o out.pdf] [-v VERSION]

Dependencies (pure Python, no system libs):
    pip install fpdf2 pypdf
"""
from __future__ import annotations

import argparse
import datetime
import shutil
import sys
import tempfile
import tomllib
import warnings
from pathlib import Path
from typing import Iterable

# Silence fpdf2's deprecation warnings on `ln=` — these are cosmetic and
# the alternative API (new_x / new_y) is an fpdf2-specific dep we'd rather
# not couple to in case the library's API evolves again.
warnings.filterwarnings("ignore", category=DeprecationWarning)

from fpdf import FPDF  # noqa: E402
from pypdf import PdfReader, PdfWriter  # noqa: E402


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

AUTHOR = "Avery Wagar"
TITLE = "PedalKernel"

# Priority-ordered file groups. First-in gets rendered first; if the total
# exceeds the page budget and truncation kicks in, the first-listed material
# is preferentially kept (it appears in the "first 25" pages).
#
# Patterns that match nothing are silently skipped — this keeps the script
# stable across branches where some files haven't landed yet (e.g. SPQR on
# feature/spqr-tree vs. older compiler on main).
FILE_GROUPS: list[tuple[str, list[str]]] = [
    (
        "Compiler core and SPQR decomposition",
        [
            "pedalkernel/src/compiler/mod.rs",
            "pedalkernel/src/compiler/compile.rs",
            "pedalkernel/src/compiler/spqr.rs",
            "pedalkernel/src/compiler/spqr_build.rs",
            "pedalkernel/src/compiler/spqr_control.rs",
            "pedalkernel/src/compiler/signal_flow.rs",
            "pedalkernel/src/compiler/graph.rs",
            "pedalkernel/src/compiler/topology.rs",
            "pedalkernel/src/compiler/classify.rs",
            "pedalkernel/src/compiler/plan.rs",
            "pedalkernel/src/compiler/split.rs",
            "pedalkernel/src/compiler/build.rs",
        ],
    ),
    (
        "Wave digital filter engine",
        [
            "pedalkernel/src/tree.rs",
            "pedalkernel/src/compiler/stage.rs",
            "pedalkernel/src/compiler/wdf_leaf.rs",
        ],
    ),
    (
        ".pedal DSL parser",
        [
            "pedalkernel/src/dsl.rs",
            "pedalkernel/src/dsl/**/*.rs",
        ],
    ),
    (
        "Wright Omega and nonlinear solver",
        [
            "pedalkernel/src/elements/nonlinear/solver.rs",
            "pedalkernel/src/elements/nonlinear/diode.rs",
            "pedalkernel/src/elements/nonlinear/jfet.rs",
            "pedalkernel/src/elements/nonlinear/triode.rs",
            "pedalkernel/src/elements/nonlinear/pentode.rs",
            "pedalkernel/src/elements/nonlinear/mosfet.rs",
            "pedalkernel/src/elements/nonlinear/ota.rs",
            "pedalkernel/src/elements/nonlinear/vari_mu.rs",
            "pedalkernel/src/compiler/opamp_analysis.rs",
            "pedalkernel/src/compiler/bjt_bias_analysis.rs",
        ],
    ),
    (
        "Component trait system",
        [
            "pedalkernel/src/compiler/component.rs",
            "pedalkernel/src/compiler/components/*.rs",
            "pedalkernel/src/elements/mod.rs",
            "pedalkernel/src/elements/linear.rs",
            "pedalkernel/src/elements/controlled.rs",
        ],
    ),
    (
        "IR and codegen",
        [
            "pedalkernel/src/compiler/compiled.rs",
            "pedalkernel/src/compiler/bind.rs",
            "pedalkernel/src/compiler/dyn_node.rs",
            "pedalkernel/src/compiler/subcircuit.rs",
            "pedalkernel/src/compiler/validate.rs",
            "pedalkernel/src/compiler/warnings.rs",
        ],
    ),
]

# Copyright Office: long works can deposit first 25 + last 25 pages.
MAX_PAGES_BEFORE_TRUNCATE = 50
HEAD_PAGES = 25
TAIL_PAGES = 25


# ---------------------------------------------------------------------------
# Version lookup
# ---------------------------------------------------------------------------


def read_version(repo_root: Path) -> str:
    """Extract workspace / package version from the root Cargo.toml."""
    for candidate in [repo_root / "Cargo.toml", repo_root / "pedalkernel" / "Cargo.toml"]:
        if not candidate.exists():
            continue
        with open(candidate, "rb") as f:
            data = tomllib.load(f)
        ws = data.get("workspace", {}).get("package", {})
        if "version" in ws:
            return ws["version"]
        if "package" in data and "version" in data["package"]:
            return data["package"]["version"]
    return "0.0.0"


# ---------------------------------------------------------------------------
# File collection
# ---------------------------------------------------------------------------


def collect_files(groups, repo_root: Path) -> list[tuple[str, list[Path]]]:
    """Expand globs and return [(section_name, [files]), ...] with dedup."""
    results: list[tuple[str, list[Path]]] = []
    seen: set[Path] = set()
    for section, patterns in groups:
        files: list[Path] = []
        for pattern in patterns:
            if "*" in pattern:
                matched = sorted(repo_root.glob(pattern))
            else:
                p = repo_root / pattern
                matched = [p] if p.exists() else []
            for path in matched:
                if path.is_file() and path not in seen:
                    seen.add(path)
                    files.append(path)
        if files:
            results.append((section, files))
    return results


# ---------------------------------------------------------------------------
# PDF rendering
# ---------------------------------------------------------------------------


class CopyrightPDF(FPDF):
    """FPDF subclass with per-page header/footer for the Copyright deposit."""

    def __init__(self, title: str, version: str, author: str, year: int):
        super().__init__(orientation="P", unit="in", format="Letter")
        self._title_text = title
        self._version = version
        self._author = author
        self._year = year
        self._current_file = ""
        self.set_margins(left=1.0, top=0.85, right=1.0)
        self.set_auto_page_break(auto=True, margin=0.85)

    def set_current_file(self, path: str) -> None:
        self._current_file = path

    def header(self) -> None:  # noqa: D401 — fpdf2 override name
        self.set_font("Courier", "B", 9)
        self.cell(3.5, 0.25, f"{self._title_text} v{self._version}", ln=0, align="L")
        self.set_font("Courier", "", 8)
        self.cell(3.5, 0.25, self._current_file, ln=1, align="R")
        self.ln(0.05)
        y = self.get_y()
        self.set_draw_color(180, 180, 180)
        self.line(self.l_margin, y, self.w - self.r_margin, y)
        self.set_draw_color(0, 0, 0)
        self.ln(0.15)

    def footer(self) -> None:  # noqa: D401
        self.set_y(-0.6)
        y = self.get_y()
        self.set_draw_color(180, 180, 180)
        self.line(self.l_margin, y, self.w - self.r_margin, y)
        self.set_draw_color(0, 0, 0)
        self.ln(0.1)
        self.set_font("Courier", "", 8)
        self.cell(3.5, 0.25, f"(c) {self._year} {self._author}", ln=0, align="L")
        self.cell(3.5, 0.25, f"Page {self.page_no()}", ln=0, align="R")


def render_pdf(
    groups: list[tuple[str, list[Path]]],
    repo_root: Path,
    output: Path,
    title: str,
    version: str,
    author: str,
    year: int,
) -> int:
    """Render the full PDF. Returns the page count."""
    pdf = CopyrightPDF(title=title, version=version, author=author, year=year)

    # Cover page.
    pdf.set_current_file("")
    pdf.add_page()
    pdf.ln(2.0)
    pdf.set_font("Courier", "B", 24)
    pdf.cell(0, 0.5, title, ln=1, align="C")
    pdf.set_font("Courier", "", 14)
    pdf.cell(0, 0.35, f"Version {version}", ln=1, align="C")
    pdf.ln(0.5)
    pdf.set_font("Courier", "", 11)
    pdf.cell(0, 0.25, f"(c) {year} {author}", ln=1, align="C")
    pdf.ln(0.3)
    pdf.cell(0, 0.22, "Source code deposit", ln=1, align="C")
    pdf.cell(0, 0.22, "U.S. Copyright Office registration", ln=1, align="C")
    pdf.ln(1.0)
    pdf.set_font("Courier", "", 10)
    pdf.cell(0, 0.25, "Contents:", ln=1, align="L")
    pdf.ln(0.1)
    for section, files in groups:
        pdf.set_font("Courier", "B", 10)
        pdf.cell(0, 0.22, f"  {section}", ln=1)
        pdf.set_font("Courier", "", 9)
        for path in files:
            rel = path.relative_to(repo_root).as_posix()
            pdf.cell(0, 0.18, f"    {rel}", ln=1)

    # Source pages.
    for section, files in groups:
        for path in files:
            rel = path.relative_to(repo_root).as_posix()
            pdf.set_current_file(rel)
            pdf.add_page()
            pdf.set_font("Courier", "B", 11)
            pdf.cell(0, 0.3, section, ln=1)
            pdf.set_font("Courier", "B", 10)
            pdf.cell(0, 0.25, rel, ln=1)
            pdf.ln(0.1)
            pdf.set_font("Courier", "", 8)
            _emit_source(pdf, path)

    pdf.output(str(output))
    reader = PdfReader(str(output))
    return len(reader.pages)


def _emit_source(pdf: FPDF, path: Path) -> None:
    """Write the file's source into the PDF, wrapping long lines."""
    try:
        text = path.read_text(encoding="utf-8", errors="replace")
    except OSError as e:
        pdf.cell(0, 0.18, f"[could not read file: {e}]", ln=1)
        return
    # Character width at 8pt Courier is ~0.06in. Printable width = 6.5in at 1"
    # margins on Letter, so ~108 chars per line. Round down to 100 for safety.
    max_chars = 100
    line_height = 0.13
    for raw_line in text.splitlines() or [""]:
        # Expand tabs so wrapping works predictably.
        line = raw_line.replace("\t", "    ")
        if not line:
            pdf.ln(line_height)
            continue
        # Hard-wrap long lines with a continuation indent so the reader can
        # tell a wrapped line from a new one.
        pieces = _wrap(line, max_chars)
        first = True
        for piece in pieces:
            prefix = "" if first else "    "  # visual continuation marker
            first = False
            try:
                pdf.cell(0, line_height, prefix + piece, ln=1)
            except Exception:
                # fpdf2 will complain about unprintable chars. Replace them.
                safe = (prefix + piece).encode("latin-1", "replace").decode("latin-1")
                pdf.cell(0, line_height, safe, ln=1)


def _wrap(line: str, width: int) -> Iterable[str]:
    """Hard-wrap on column boundary, preserving source shape."""
    if len(line) <= width:
        yield line
        return
    while line:
        yield line[:width]
        line = line[width:]


# ---------------------------------------------------------------------------
# Truncation (first 25 + last 25)
# ---------------------------------------------------------------------------


def truncate_pdf(src: Path, dest: Path, head: int, tail: int) -> None:
    """Produce a new PDF containing only the first `head` + last `tail` pages."""
    reader = PdfReader(str(src))
    writer = PdfWriter()

    for i in range(head):
        writer.add_page(reader.pages[i])
    for i in range(len(reader.pages) - tail, len(reader.pages)):
        writer.add_page(reader.pages[i])

    with open(dest, "wb") as f:
        writer.write(f)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=Path("pedalkernel-copyright-deposit.pdf"),
        help="Output PDF path (default: pedalkernel-copyright-deposit.pdf)",
    )
    parser.add_argument(
        "-v",
        "--version",
        default=None,
        help="Explicit version string (defaults to value in workspace Cargo.toml)",
    )
    parser.add_argument(
        "-r",
        "--repo-root",
        type=Path,
        default=Path.cwd(),
        help="Repo root (default: cwd)",
    )
    args = parser.parse_args()

    repo_root = args.repo_root.resolve()
    version = args.version or read_version(repo_root)
    year = datetime.datetime.now().year

    groups = collect_files(FILE_GROUPS, repo_root)
    if not groups:
        print("error: no source files matched the priority groups", file=sys.stderr)
        return 1

    file_count = sum(len(files) for _, files in groups)
    print(
        f"Rendering {file_count} file(s) across {len(groups)} group(s) for {TITLE} v{version}",
        file=sys.stderr,
    )

    with tempfile.TemporaryDirectory() as tmp:
        full = Path(tmp) / "full.pdf"
        page_count = render_pdf(
            groups=groups,
            repo_root=repo_root,
            output=full,
            title=TITLE,
            version=version,
            author=AUTHOR,
            year=year,
        )
        print(f"Full document: {page_count} pages", file=sys.stderr)

        if page_count > MAX_PAGES_BEFORE_TRUNCATE:
            print(
                f"Exceeds {MAX_PAGES_BEFORE_TRUNCATE} pages; truncating to first "
                f"{HEAD_PAGES} + last {TAIL_PAGES}",
                file=sys.stderr,
            )
            truncate_pdf(full, args.output, head=HEAD_PAGES, tail=TAIL_PAGES)
        else:
            shutil.copyfile(full, args.output)

    final = PdfReader(str(args.output))
    print(f"Wrote {args.output} ({len(final.pages)} pages)", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
