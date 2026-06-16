#!/usr/bin/env python3
"""SPICE validation REGRESSION gate (base-vs-PR diff).

Compares two validation run-reports — the PR's and the base branch's — produced
by `pedalkernel-validate run` with the SAME metrics + goldens, so the only
difference is the code under test. Fails (exit 1) only when more than
--max-new-failures circuits that PASSED on the base now FAIL on the PR.

Circuits present only in the PR (newly added tests/circuits) are IGNORED, so
adding new — possibly failing — tests never trips the gate; only regressions of
pre-existing circuits do.

Usage:
  check_spice_regressions.py --base base-report.json --current pr-report.json \
      [--max-new-failures N]
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


def report_pass_map(report: dict) -> dict[str, bool]:
    """suites -> tests -> signals[{label, passed, pending}]  =>  {suite/test/label: passed}.

    Pending signals (golden missing) are neither pass nor fail and are dropped.
    """
    out: dict[str, bool] = {}
    for sn, sd in report.get("suites", {}).items():
        for tn, td in sd.get("tests", {}).items():
            for sig in td.get("signals", []):
                if sig.get("pending"):
                    continue
                out[f"{sn}/{tn}/{sig['label']}"] = bool(sig.get("passed", False))
    return out


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--base", required=True, type=Path, help="base-branch run report JSON")
    p.add_argument("--current", required=True, type=Path, help="PR run report JSON")
    p.add_argument("--max-new-failures", type=int, default=0)
    a = p.parse_args()

    if not a.current.exists():
        print(f"::error::current report not found: {a.current}", file=sys.stderr)
        return 1
    cur = report_pass_map(json.loads(a.current.read_text()))

    if not a.base.exists():
        print(f"::warning::base report {a.base} missing — skipping regression gate "
              "(no comparable base run; e.g. first commit / force-push)")
        return 0
    base = report_pass_map(json.loads(a.base.read_text()))

    regressions = sorted(k for k in cur if base.get(k) is True and cur[k] is False)
    fixed = sorted(k for k in cur if base.get(k) is False and cur[k] is True)
    new_circuits = sorted(k for k in cur if k not in base)
    removed = sorted(k for k in base if k not in cur)

    print("------------------------------------------------------------")
    print(f"SPICE regression gate  (base {len(base)} circuits, PR {len(cur)})")
    print(f"  new circuits (ignored): {len(new_circuits)}")
    print(f"  removed circuits      : {len(removed)}")
    print(f"  fixed (fail -> pass)  : {len(fixed)}")
    for k in fixed:
        print(f"      + {k}")
    print(f"  REGRESSIONS (pass -> fail): {len(regressions)}  (allowed: {a.max_new_failures})")
    for k in regressions:
        print(f"      - {k}")
    if len(regressions) > a.max_new_failures:
        print(f"::error::{len(regressions)} new SPICE regression(s) exceed the allowed "
              f"{a.max_new_failures} (set SPICE_MAX_NEW_FAILURES to adjust)")
        return 1
    print("SPICE regression gate satisfied.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
