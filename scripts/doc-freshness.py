#!/usr/bin/env python3
"""
Check book docs for drift: compare each doc's `source_commit` front
matter to the current HEAD, and for each path under `watches` report
what's changed since the doc was written.

Front matter schema (per book page, inside the YAML block):

    source_commit: "abc1234..."   # full SHA the doc was validated against
    watches:                      # files that make this doc stale if edited
      - pedalkernel/src/compiler/component.rs
      - pedalkernel/src/compiler/components/

Usage:
    scripts/doc-freshness.py                 # report on all docs
    scripts/doc-freshness.py --fail-on-stale # exit 1 if anything is stale
    scripts/doc-freshness.py --docs docs/    # custom docs root

Output is plain text, two-column. Designed to be readable in CI logs
and embeddable in PR comments.
"""
from __future__ import annotations

import argparse
import re
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path


FRONT_MATTER_RE = re.compile(r"^---\n(.*?)\n---\n", re.DOTALL)
# Minimal YAML for just the fields we care about. Full YAML parsing
# would add a dep; we only need source_commit and watches.
SOURCE_COMMIT_RE = re.compile(r'^source_commit:\s*["\']?([a-f0-9]{7,40})["\']?\s*$', re.MULTILINE)
WATCHES_BLOCK_RE = re.compile(r"^watches:\s*\n((?:\s+-\s+.+\n)+)", re.MULTILINE)
WATCHES_ITEM_RE = re.compile(r"^\s+-\s+[\"']?(.+?)[\"']?\s*$", re.MULTILINE)


@dataclass
class DocMeta:
    path: Path
    source_commit: str | None
    watches: list[str]


def parse_front_matter(path: Path) -> DocMeta:
    text = path.read_text(encoding="utf-8", errors="replace")
    m = FRONT_MATTER_RE.match(text)
    if not m:
        return DocMeta(path=path, source_commit=None, watches=[])
    block = m.group(1)

    commit = None
    commit_match = SOURCE_COMMIT_RE.search(block)
    if commit_match:
        commit = commit_match.group(1)

    watches: list[str] = []
    watches_match = WATCHES_BLOCK_RE.search(block)
    if watches_match:
        watches = WATCHES_ITEM_RE.findall(watches_match.group(1))

    return DocMeta(path=path, source_commit=commit, watches=watches)


def git_commits_since(source_commit: str, path: str) -> list[tuple[str, str]]:
    """Return [(short_sha, subject), ...] of non-merge commits touching path
    since source_commit.

    Merge commits are skipped (--no-merges): they don't introduce new content
    relative to their parents, and counting them would flag docs as stale on
    any branch that has merged main into a feature branch even though nothing
    substantive changed. The non-merge commits they bring in still show up
    here — those are the real drift events.
    """
    try:
        out = subprocess.check_output(
            ["git", "log", "--oneline", "--no-merges",
             f"{source_commit}..HEAD", "--", path],
            text=True,
            stderr=subprocess.DEVNULL,
        )
    except subprocess.CalledProcessError:
        # source_commit unreachable from HEAD — report no drift rather than
        # crashing. Useful when previewing SPQR-tree docs on main before merge.
        return []
    rows: list[tuple[str, str]] = []
    for line in out.splitlines():
        if not line:
            continue
        sha, _, subject = line.partition(" ")
        rows.append((sha, subject))
    return rows


def collect_docs(root: Path) -> list[Path]:
    return sorted(p for p in root.rglob("*.md") if p.is_file())


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    parser.add_argument("--docs", type=Path, default=Path("docs"), help="Docs root (default: docs/)")
    parser.add_argument(
        "--fail-on-stale",
        action="store_true",
        help="Exit with non-zero status if any tracked doc is stale",
    )
    args = parser.parse_args()

    if not args.docs.is_dir():
        print(f"error: docs directory not found: {args.docs}", file=sys.stderr)
        return 2

    # Resolve current HEAD for human reference.
    head = subprocess.check_output(["git", "rev-parse", "--short", "HEAD"], text=True).strip()

    untracked: list[Path] = []
    fresh: list[DocMeta] = []
    stale: list[tuple[DocMeta, list[tuple[str, list[tuple[str, str]]]]]] = []

    for path in collect_docs(args.docs):
        meta = parse_front_matter(path)
        if meta.source_commit is None:
            untracked.append(path)
            continue
        drift: list[tuple[str, list[tuple[str, str]]]] = []
        for watched in meta.watches:
            commits = git_commits_since(meta.source_commit, watched)
            if commits:
                drift.append((watched, commits))
        if drift:
            stale.append((meta, drift))
        else:
            fresh.append(meta)

    # -------- reporting --------
    print(f"HEAD is {head}")
    print()

    if stale:
        print(f"STALE — {len(stale)} doc(s) have drift on their watched files:")
        for meta, drift in stale:
            print(f"  {meta.path}  (source: {meta.source_commit[:7]})")
            for watched, commits in drift:
                print(f"    {watched}")
                for sha, subject in commits[:8]:
                    print(f"      {sha}  {subject}")
                if len(commits) > 8:
                    print(f"      ... and {len(commits) - 8} more")
        print()

    if fresh:
        print(f"FRESH — {len(fresh)} doc(s) are current against their watched files:")
        for meta in fresh:
            print(f"  {meta.path}  (source: {meta.source_commit[:7]})")
        print()

    if untracked:
        print(f"UNTRACKED — {len(untracked)} doc(s) have no source_commit front matter:")
        for path in untracked:
            print(f"  {path}")

    if args.fail_on_stale and stale:
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
