#!/usr/bin/env python3
"""Print a human-readable dashboard summary from accuracy.json to stdout.

Used by the GitHub Actions workflow to surface results in the job log.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path


def main() -> int:
    if len(sys.argv) < 2:
        print("Usage: print_summary.py <accuracy.json>", file=sys.stderr)
        return 1

    path = Path(sys.argv[1])
    if not path.exists():
        print(f"accuracy.json not found at {path}", file=sys.stderr)
        return 1

    with path.open() as f:
        a = json.load(f)

    s = a["summary"]
    print("=" * 60)
    print("ACCURACY DASHBOARD SUMMARY")
    print("=" * 60)
    print(f"  Timestamp    : {a['run_timestamp']}")
    print(f"  Git commit   : {a.get('git_commit') or 'unknown'}")
    print(f"  Circuits     : {s['total_circuits']}")
    print(f"  Passed       : {s['passed']}")
    print(f"  Failed       : {s['failed']}")
    print(f"  WDF pending  : {s['wdf_pending']}")
    print(f"  Pass rate    : {s['pass_rate'] * 100:.1f}%")
    print("=" * 60)

    if s["failed"] > 0:
        print("\nFailed circuits:")
        for key, c in a["circuits"].items():
            if not c["passed"]:
                err = c.get("error") or ""
                print(f"  FAIL  {key}  {err}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
