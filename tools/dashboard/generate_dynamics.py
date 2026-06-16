#!/usr/bin/env python3
"""
PedalKernel Dynamics Dashboard Plotter

Consumes:
  - dynamics.json produced by `pedalkernel-validate dynamics`
    (--dynamics-json), schema documented in that subcommand and below.

Produces, per circuit, into <out-dir>/<suite>/<test>/:
  - static_curve.png : input dBVU vs output dBVU (WDF vs ngspice + unity ref)
                       and input dBVU vs gain-reduction dB.
  - tone_burst.png   : time (ms) vs gain-reduction dB (WDF vs ngspice), with
                       measured attack/release annotations.

Style matches tools/dashboard/generate_dashboard.py (dark theme; ngspice cyan
#00d4ff, WDF orange #ff6b35).

dynamics.json schema (schema_version 1)
---------------------------------------
{
  "schema_version": 1,
  "run_timestamp": "<str>",
  "git_commit": "<str|null>",
  "sample_rate": <int>, "oversample": <int>,
  "circuits": {
    "<suite>/<test>": {
      "suite": "...", "test": "...", "circuit": "<pedal path>",
      "frequency_hz": <float>,
      "static_curve": {                      # optional
        "input_levels_dbvu": [..],
        "wdf_output_dbvu": [..], "spice_output_dbvu": [..],
        "wdf_gr_db": [..], "spice_gr_db": [..],
        "wdf_ratio": <float>, "spice_ratio": <float>,
        "wdf_gr_total_db": <float>, "spice_gr_total_db": <float>
      },
      "tone_burst": {                        # optional
        "on_ms": <float>, "off_ms": <float>,
        "time_ms": [..], "wdf_gr_db": [..], "spice_gr_db": [..],
        "wdf_attack_ms": <float>, "wdf_release_ms": <float>,
        "spice_attack_ms": <float>, "spice_release_ms": <float>
      }
    }
  }
}
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")  # non-interactive; must precede pyplot import
import matplotlib.pyplot as plt

# --- shared dark theme (mirrors generate_dashboard.py) ----------------------
FIG_BG = "#1a1a2e"
AX_BG = "#16213e"
SPICE_COLOR = "#00d4ff"   # ngspice = cyan
WDF_COLOR = "#ff6b35"     # WDF = orange
UNITY_COLOR = "#888888"   # dashed grey unity reference
TEXT = "white"
GRID = "#333"
LEGEND_BG = "#0f3460"
BOX_BG = "#0f3460"


def _style_axis(ax) -> None:
    ax.set_facecolor(AX_BG)
    ax.tick_params(colors=TEXT)
    for spine in ax.spines.values():
        spine.set_color("#444")
    ax.grid(True, color=GRID, linewidth=0.4)
    ax.xaxis.label.set_color(TEXT)
    ax.yaxis.label.set_color(TEXT)


def _ratio_str(r: float) -> str:
    """Render a compression ratio, guarding INFINITY (perfect limiter)."""
    if r is None:
        return "—"
    if r == float("inf") or (isinstance(r, float) and r != r):  # inf or nan
        return "∞:1"
    return f"{r:.2f}:1"


def plot_static_curve(circuit: dict, out_path: Path, title: str) -> None:
    sc = circuit["static_curve"]
    x = sc["input_levels_dbvu"]

    fig, (ax_a, ax_b) = plt.subplots(1, 2, figsize=(13, 5))
    fig.patch.set_facecolor(FIG_BG)
    for ax in (ax_a, ax_b):
        _style_axis(ax)
    fig.suptitle(title + " — static gain", color=TEXT, fontsize=11, fontweight="bold")

    # ngspice traces are optional: WDF-only circuits (no committed golden) are
    # measured live through the engine and have no `spice_*` arrays.
    spice_out = sc.get("spice_output_dbvu")
    spice_gr = sc.get("spice_gr_db")
    has_spice = spice_out is not None and spice_gr is not None

    # Panel A: input vs output, with unity (y=x) reference.
    lo, hi = min(x), max(x)
    ax_a.plot([lo, hi], [lo, hi], color=UNITY_COLOR, linestyle="--",
              linewidth=0.9, label="unity (y=x)", alpha=0.8)
    if has_spice:
        ax_a.plot(x, spice_out, color=SPICE_COLOR, marker="o",
                  markersize=4, linewidth=1.2, label="ngspice", alpha=0.95)
    ax_a.plot(x, sc["wdf_output_dbvu"], color=WDF_COLOR, marker="s",
              markersize=4, linewidth=1.2, label="WDF", alpha=0.9)
    ax_a.set_xlabel("input (dBVU)")
    ax_a.set_ylabel("output (dBVU)")
    ax_a.set_title("transfer curve", color=TEXT, fontsize=9)
    ax_a.legend(facecolor=LEGEND_BG, labelcolor=TEXT, fontsize=8)

    # Panel B: input vs gain reduction.
    if has_spice:
        ax_b.plot(x, spice_gr, color=SPICE_COLOR, marker="o",
                  markersize=4, linewidth=1.2, label="ngspice", alpha=0.95)
    ax_b.plot(x, sc["wdf_gr_db"], color=WDF_COLOR, marker="s",
              markersize=4, linewidth=1.2, label="WDF", alpha=0.9)
    ax_b.set_xlabel("input (dBVU)")
    ax_b.set_ylabel("gain reduction (dB)")
    ax_b.set_title("gain reduction vs input", color=TEXT, fontsize=9)
    ax_b.legend(facecolor=LEGEND_BG, labelcolor=TEXT, fontsize=8)

    annot = (
        f"WDF:    ratio {_ratio_str(sc.get('wdf_ratio'))}, "
        f"total GR {sc.get('wdf_gr_total_db', 0.0):.2f} dB"
    )
    if has_spice:
        annot += (
            f"\nngspice: ratio {_ratio_str(sc.get('spice_ratio'))}, "
            f"total GR {sc.get('spice_gr_total_db', 0.0):.2f} dB"
        )
    else:
        annot += "\n(WDF-only — no ngspice golden yet)"
    ax_b.text(
        0.02, 0.04, annot, transform=ax_b.transAxes, color=TEXT, fontsize=8,
        va="bottom", ha="left",
        bbox=dict(facecolor=BOX_BG, alpha=0.85, pad=4, edgecolor="#444"),
    )

    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(str(out_path), dpi=140, bbox_inches="tight",
                facecolor=fig.get_facecolor())
    plt.close(fig)


def plot_tone_burst(circuit: dict, out_path: Path, title: str) -> None:
    tb = circuit["tone_burst"]
    t = tb["time_ms"]

    fig, ax = plt.subplots(figsize=(13, 5))
    fig.patch.set_facecolor(FIG_BG)
    _style_axis(ax)
    fig.suptitle(title + " — attack / release", color=TEXT, fontsize=11,
                 fontweight="bold")

    spice_gr = tb.get("spice_gr_db")
    has_spice = spice_gr is not None
    if has_spice:
        ax.plot(t, spice_gr, color=SPICE_COLOR, linewidth=1.0,
                label="ngspice", alpha=0.95)
    ax.plot(t, tb["wdf_gr_db"], color=WDF_COLOR, linewidth=1.0,
            label="WDF", alpha=0.9)
    ax.set_xlabel("time (ms)")
    ax.set_ylabel("gain reduction (dB)")
    ax.legend(facecolor=LEGEND_BG, labelcolor=TEXT, fontsize=8)

    # NOTE: scalar attack/release (ms) is intentionally NOT annotated here. The
    # single-burst envelope-settle measurement reflects audio-level settling, not
    # gain-reduction ballistics, and is unreliable on the current rough reference.
    # The TRANSIENT SHAPE is the honest takeaway.
    if has_spice:
        annot = (
            "GR-vs-time shape (qualitative).\n"
            "WDF flat ≈ no compression; reference is level-dependent."
        )
    else:
        annot = (
            "GR-vs-time shape (qualitative, WDF-only).\n"
            "Live WDF measurement; no ngspice golden yet."
        )
    ax.text(
        0.98, 0.96, annot, transform=ax.transAxes, color=TEXT, fontsize=8,
        va="top", ha="right",
        bbox=dict(facecolor=BOX_BG, alpha=0.85, pad=4, edgecolor="#444"),
    )

    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(str(out_path), dpi=140, bbox_inches="tight",
                facecolor=fig.get_facecolor())
    plt.close(fig)


def main() -> int:
    p = argparse.ArgumentParser(description="Render PedalKernel dynamics PNGs.")
    p.add_argument("--dynamics-json", required=True, type=Path,
                   help="Path to dynamics.json from pedalkernel-validate dynamics")
    p.add_argument("--out-dir", required=True, type=Path,
                   help="Output directory root for <suite>/<test>/*.png")
    args = p.parse_args()

    data = json.loads(args.dynamics_json.read_text())
    circuits = data.get("circuits", {})
    if not circuits:
        print("No circuits in dynamics.json; nothing to render.")
        return 0

    n_static = 0
    n_burst = 0
    for key, circuit in sorted(circuits.items()):
        suite = circuit.get("suite", key.split("/")[0])
        test = circuit.get("test", key.split("/")[-1])
        out_dir = args.out_dir / suite / test
        title = key

        if circuit.get("static_curve"):
            out = out_dir / "static_curve.png"
            plot_static_curve(circuit, out, title)
            print(f"  ✓ {out}")
            n_static += 1
        # Tone-burst rendering is disabled: the single-burst GR-vs-time measurement
        # is not yet robust across diverse circuits (slow-attack opto truncates the
        # window; non-compressing paths pick up onset-transient artifacts). The
        # `plot_tone_burst` helper and the JSON `tone_burst` data are retained for
        # when the ballistics measurement is hardened. Static curves ship now.
        _ = plot_tone_burst  # keep referenced

    print(f"Rendered {n_static} static-curve and {n_burst} tone-burst PNG(s).")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
