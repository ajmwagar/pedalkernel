#!/usr/bin/env python3
"""
PedalKernel Accuracy Dashboard Generator

Consumes:
  - pedalkernel-validate JSON report  (--report-json)
  - ngspice golden .npy files         (--golden-dir, layout: {suite}/{test}/{signal}.npy)
  - WDF golden .npy files             (layout: {suite}/{test}/wdf/{signal}.npy, optional)

Produces per-circuit PNGs:
  - time-domain overlay (ngspice vs WDF + difference trace)
  - FFT magnitude overlay (log-freq, dB)
  - annotated with dB-error metrics from the JSON report

Produces summary outputs:
  - matrix-<suite>.png     : per-suite RMS/Peak/Spectral error heatmaps
  - accuracy.json          : per-circuit pass/fail + metrics (schema documented below)
  - history.json           : append-only per-run drift log

JSON SCHEMAS
============

accuracy.json
-------------
{
  "schema_version": 1,
  "run_timestamp": "<ISO-8601 string passed via --timestamp>",
  "git_commit": "<string or null>",
  "sample_rate": <int>,
  "oversample": <int>,
  "summary": {
    "total_circuits": <int>,
    "passed": <int>,
    "failed": <int>,
    "wdf_pending": <int>,
    "pass_rate": <float 0-1>
  },
  "circuits": {
    "<suite>/<test>/<signal>": {
      "suite": "<string>",
      "test": "<string>",
      "signal": "<string>",
      "passed": <bool>,
      "wdf_pending": <bool>,   // true = WDF golden absent; ngspice-only plot
      "metrics": {             // null if wdf_pending=true or comparison failed
        "normalized_rms_error_db": <float>,
        "peak_error_db": <float>,
        "thd_error_db": <float or null>,
        "thd_plus_n_error_db": <float or null>,   // THD+N error (harmonics + broadband noise/intermod)
        "harmonic_mag_error_db": <float or null>, // worst per-harmonic magnitude error (−100 dBFS gated)
        "even_odd_ratio_error_db": <float or null>, // even/odd balance error (push-pull vs single-ended)
        "spectral_error_db": <float>,        // audio-band (capped at audio Nyquist); gating value
        "spectral_error_full_db": <float>,   // full-band (raw), up to the data Nyquist; informational
        "even_odd_ratio_db": <float or null>,
        "dc_drift_mv": <float or null>
      },
      "error": <string or null>
    }
  }
}

history.json
------------
Array of run records, appended on each invocation:
[
  {
    "timestamp": "<ISO-8601 string>",
    "git_commit": "<string or null>",
    "summary": { ... same as accuracy.json summary ... },
    "circuits": {
      "<suite>/<test>/<signal>": {
        "passed": <bool>,
        "wdf_pending": <bool>,
        "normalized_rms_error_db": <float or null>,
        "peak_error_db": <float or null>,
        "spectral_error_db": <float or null>
      }
    }
  },
  ...
]
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path
from typing import Any

import matplotlib
matplotlib.use("Agg")  # non-interactive backend; must be before pyplot import
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import numpy as np
from PIL import Image  # rendered-PNG dimension readback (Pillow is a matplotlib dep)


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

SCHEMA_VERSION = 1
SAMPLE_RATE_DEFAULT = 384_000  # 96kHz x4 oversample

# RMS/Peak-error colour scale (dB difference, negative = good). Diverging around
# 0: green at VMIN (deep negative = excellent), YELLOW at 0 (error == signal,
# borderline), red at VMAX_POS (large positive = the engine is badly off). The
# old scale capped at 0, so every positive error (0, +13, +90) rendered the same
# max red and severity was invisible.
HEATMAP_VMIN_DB = -100.0
HEATMAP_VMAX_DB = 0.0          # the diverging centre (neutral / yellow)
HEATMAP_VMAX_POS_DB = 75.0     # positive error mapped to deepest red

# Spectral error is a different quantity: a max-bin error that is always >= 0
# (0 dB = identical spectra), so it needs its own scale and polarity — green at
# 0, ramping to red at SPECTRAL_VMAX_DB. Tune this threshold to taste.
SPECTRAL_VMAX_DB = 60.0

# Waveform plot: trim this many seconds off the front (warmup suppression).
PLOT_WARMUP_TRIM_S = 0.005  # 5 ms

# Maximum samples shown in the time-domain panel (down-sampled for clarity).
MAX_PLOT_SAMPLES = 4096


# ---------------------------------------------------------------------------
# Utility helpers
# ---------------------------------------------------------------------------

def load_npy(path: Path) -> np.ndarray | None:
    """Load a .npy file.  Returns None (not a crash) if the file is absent."""
    if not path.exists():
        return None
    return np.load(str(path))


def compute_fft_db(signal: np.ndarray, sample_rate: float) -> tuple[np.ndarray, np.ndarray]:
    """Return (frequencies, magnitude_dB) for one-sided FFT."""
    n = len(signal)
    if n == 0:
        return np.array([]), np.array([])
    window = np.blackman(n)
    spectrum = np.abs(np.fft.rfft(signal * window)) / n
    freqs = np.fft.rfftfreq(n, d=1.0 / sample_rate)
    # Avoid log(0)
    mag_db = 20.0 * np.log10(np.maximum(spectrum, 1e-12))
    return freqs, mag_db


def trim_and_downsample(signal: np.ndarray, sample_rate: float) -> np.ndarray:
    """Trim warmup samples and down-sample to MAX_PLOT_SAMPLES."""
    trim = int(PLOT_WARMUP_TRIM_S * sample_rate)
    s = signal[trim:] if trim < len(signal) else signal
    if len(s) > MAX_PLOT_SAMPLES:
        step = len(s) // MAX_PLOT_SAMPLES
        s = s[::step]
    return s


def key_for(suite: str, test: str, signal: str) -> str:
    return f"{suite}/{test}/{signal}"


# ---------------------------------------------------------------------------
# Per-circuit plot
# ---------------------------------------------------------------------------

def plot_circuit(
    suite: str,
    test: str,
    signal: str,
    spice: np.ndarray,
    wdf: np.ndarray | None,
    metrics: dict[str, Any] | None,
    sample_rate: float,
    out_path: Path,
    base_nyquist_hz: float | None = None,
) -> None:
    """Generate a 2-panel PNG (time-domain + FFT) for one circuit/signal."""
    out_path.parent.mkdir(parents=True, exist_ok=True)

    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    fig.patch.set_facecolor("#1a1a2e")
    for ax in axes:
        ax.set_facecolor("#16213e")
        ax.tick_params(colors="white")
        ax.xaxis.label.set_color("white")
        ax.yaxis.label.set_color("white")
        ax.title.set_color("white")
        for spine in ax.spines.values():
            spine.set_edgecolor("#444")

    title = f"{suite} / {test} / {signal}"
    if wdf is None:
        title += "  [WDF pending]"

    fig.suptitle(title, color="white", fontsize=11, fontweight="bold")

    # --- Time domain ---
    ax_t = axes[0]
    spice_plot = trim_and_downsample(spice, sample_rate)
    t_spice = np.arange(len(spice_plot)) / sample_rate

    ax_t.plot(t_spice * 1e3, spice_plot, color="#00d4ff", linewidth=0.9, label="ngspice", alpha=0.9)

    if wdf is not None:
        wdf_plot = trim_and_downsample(wdf, sample_rate)
        t_wdf = np.arange(len(wdf_plot)) / sample_rate
        ax_t.plot(t_wdf * 1e3, wdf_plot, color="#ff6b35", linewidth=0.9, label="WDF", alpha=0.8)

        # Difference trace (align lengths)
        n_diff = min(len(spice_plot), len(wdf_plot))
        diff = wdf_plot[:n_diff] - spice_plot[:n_diff]
        ax_t.plot(
            t_spice[:n_diff] * 1e3, diff,
            color="#90ee90", linewidth=0.6, label="diff", alpha=0.7
        )

    ax_t.set_xlabel("Time (ms)")
    ax_t.set_ylabel("Amplitude (V)")  # linear sample value (volts), NOT dB
    ax_t.set_title("Time Domain")
    ax_t.legend(facecolor="#0f3460", labelcolor="white", fontsize=8)
    ax_t.grid(True, color="#333", linewidth=0.4)

    # --- FFT ---
    ax_f = axes[1]
    freqs_s, mag_s = compute_fft_db(spice, sample_rate)
    if len(freqs_s) > 1:
        mask = freqs_s > 0
        ax_f.semilogx(
            freqs_s[mask], mag_s[mask],
            color="#00d4ff", linewidth=0.9, label="ngspice", alpha=0.9
        )

    if wdf is not None:
        freqs_w, mag_w = compute_fft_db(wdf, sample_rate)
        if len(freqs_w) > 1:
            mask = freqs_w > 0
            ax_f.semilogx(
                freqs_w[mask], mag_w[mask],
                color="#ff6b35", linewidth=0.9, label="WDF", alpha=0.8
            )

    ax_f.set_xlabel("Frequency (Hz)")
    ax_f.set_ylabel("Magnitude (dB)")
    ax_f.set_title("FFT Magnitude")

    # Audio-Nyquist marker. Circuits run at the oversampled rate; PedalKernel
    # then anti-aliases, band-limiting ultrasonic content before downsampling,
    # so WDF output rolls off above the base-rate Nyquist. The ngspice golden is
    # a raw transient sampled at the oversampled rate and keeps the full
    # ultrasonic harmonics and aliasing up to its own Nyquist. So spectral
    # divergence to the RIGHT of this line is the anti-aliasing working as
    # intended (ultrasonic, filtered out on downsampling), not an accuracy defect.
    if base_nyquist_hz is not None and base_nyquist_hz > 0:
        ax_f.axvline(
            base_nyquist_hz, color="#ffb000", linestyle="--",
            linewidth=0.9, alpha=0.75,
            label=f"audio Nyquist ({base_nyquist_hz / 1000:.0f} kHz)",
        )

    ax_f.legend(facecolor="#0f3460", labelcolor="white", fontsize=8)
    ax_f.grid(True, which="both", color="#333", linewidth=0.4)
    ax_f.set_xlim(left=20)

    # --- Annotation box with metrics ---
    if metrics:
        rms = metrics.get("normalized_rms_error_db")
        peak = metrics.get("peak_error_db")
        spectral = metrics.get("spectral_error_db")          # audio-band (capped)
        spectral_full = metrics.get("spectral_error_full_db")  # full-band (raw)
        thd = metrics.get("thd_error_db")
        lines = []
        if rms is not None:
            lines.append(f"RMS err:   {rms:.1f} dB")
        if peak is not None:
            lines.append(f"Peak err:  {peak:.1f} dB")
        if spectral_full is not None:
            lines.append(f"Spec full: {spectral_full:.1f} dB")
        if spectral is not None:
            lines.append(f"Spec audio:{spectral:.1f} dB")
        if thd is not None:
            lines.append(f"THD err:   {thd:.2f} dB")
        if lines:
            annotation = "\n".join(lines)
            ax_f.text(
                0.02, 0.02, annotation,
                transform=ax_f.transAxes,
                fontsize=7,
                color="white",
                verticalalignment="bottom",
                bbox=dict(facecolor="#0f3460", alpha=0.8, pad=4, edgecolor="#444"),
                fontfamily="monospace",
            )

    plt.tight_layout()
    # 140 dpi: crisp at the large widths the full-width accuracy dashboard renders.
    fig.savefig(str(out_path), dpi=140, bbox_inches="tight", facecolor=fig.get_facecolor())
    plt.close(fig)


# ---------------------------------------------------------------------------
# Accuracy matrix / heatmap
# ---------------------------------------------------------------------------

def _short_row_label(r: dict[str, Any]) -> str:
    """Row label within a per-suite matrix: drop the redundant suite prefix.

    The suite is already the matrix title, so showing `test / signal` keeps the
    labels short and readable instead of repeating `<suite>/` on every row.
    """
    return f"{r['test']} / {r['signal']}"


def plot_suite_matrix(
    suite: str,
    circuit_records: list[dict[str, Any]],
    out_path: Path,
) -> tuple[int, int]:
    """Generate one compact accuracy-matrix heatmap PNG for a single suite.

    Rows = circuits/signals in this suite (suite prefix dropped from labels).
    Columns = metrics (RMS, Peak, Spectral).
    Colour = dB error: within each column, green = better (closer to ngspice),
             red = worse (further from the reference).

    Returns the pixel (width, height) of the rendered PNG so callers can report
    dimensions and confirm the matrices are no longer extreme-aspect-ratio.
    """
    out_path.parent.mkdir(parents=True, exist_ok=True)

    records = sorted(circuit_records, key=lambda r: r["key"])

    labels = [_short_row_label(r) for r in records]
    metric_cols = ["normalized_rms_error_db", "peak_error_db", "spectral_error_db"]
    col_labels = ["RMS\n(dB)", "Peak\n(dB)", "Spectral\n(audio, dB)"]
    spectral_col = metric_cols.index("spectral_error_db")

    # Build matrix: NaN = no data (wdf_pending or error)
    mat = np.full((len(labels), len(metric_cols)), np.nan)
    pass_flags = []

    for i, r in enumerate(records):
        m = r.get("metrics") or {}
        for j, col in enumerate(metric_cols):
            v = m.get(col)
            if v is not None:
                mat[i, j] = float(v)
        pass_flags.append(r.get("passed", False))

    n_rows, n_cols = mat.shape

    # Compact, legible sizing: fixed per-row height keeps labels readable and the
    # figure tightly bounded regardless of suite size (no 3764-px-tall column).
    row_h_in = 0.34            # inches per row
    fig_w = 7.0
    fig_h = max(2.6, n_rows * row_h_in + 1.8)

    fig, ax = plt.subplots(figsize=(fig_w, fig_h))
    fig.patch.set_facecolor("#1a1a2e")
    ax.set_facecolor("#1a1a2e")

    cmap = plt.cm.RdYlGn_r

    # Two scales, because the columns measure different quantities. In BOTH,
    # RdYlGn_r reads the same for the viewer: green = better (closer to ngspice),
    # red = worse. They differ only in where "perfect" sits, which is intrinsic
    # to the metric and shown by the per-cell number:
    #   RMS / Peak — signed dB difference. Diverging: green at -100 (excellent),
    #                yellow at 0 (error == signal), red at +VMAX_POS. So a +75
    #                error is now clearly redder than 0 (which is yellow).
    #   Spectral   — max-bin error, always >= 0, 0 = identical spectra.
    #                Scale [0, SPECTRAL_VMAX_DB]: green at 0, red at the cap.
    norm_diff = mcolors.TwoSlopeNorm(
        vmin=HEATMAP_VMIN_DB, vcenter=HEATMAP_VMAX_DB, vmax=HEATMAP_VMAX_POS_DB
    )
    norm_spec = mcolors.Normalize(vmin=0.0, vmax=SPECTRAL_VMAX_DB)

    # Map each column through its own norm into an RGBA image, then draw once.
    rgba = np.zeros((n_rows, n_cols, 4))
    for j in range(n_cols):
        col = mat[:, j]
        if j == spectral_col:
            normed = norm_spec(np.clip(col, 0.0, SPECTRAL_VMAX_DB))
        else:
            normed = norm_diff(np.clip(col, HEATMAP_VMIN_DB, HEATMAP_VMAX_POS_DB))
        rgba[:, j, :] = cmap(normed)

    # NaN cells (no data) -> grey.
    nan_mask = np.isnan(mat)
    rgba[nan_mask] = mcolors.to_rgba("#555555")

    ax.imshow(rgba, aspect="auto")

    # Print the dB value inside each cell so the colour is backed by a number.
    for i in range(n_rows):
        for j in range(n_cols):
            v = mat[i, j]
            txt = "—" if np.isnan(v) else f"{v:.0f}"
            ax.text(
                j, i, txt, ha="center", va="center",
                fontsize=7, color="#10101a", fontfamily="monospace",
            )

    # Axes labels
    ax.set_xticks(range(n_cols))
    ax.set_xticklabels(col_labels, color="white", fontsize=9)
    ax.set_yticks(range(n_rows))

    # Tick marks + column headers white FIRST — this must precede the per-row
    # colouring below, or it overrides the row label colours back to white.
    ax.tick_params(colors="white")
    # Colour row labels by pass/fail: failures stand out in orange, passes stay
    # legible white.
    ax.set_yticklabels(labels, fontsize=8)
    for tick, passed in zip(ax.get_yticklabels(), pass_flags):
        tick.set_color("white" if passed else "#ff6b35")

    ax.set_title(f"{suite} — RMS / Peak / Spectral error vs ngspice",
                 color="white", pad=8, fontsize=11, fontweight="bold")

    # Single shared colorbar legend: low = green = better, high = red = worse.
    # Both column families share the same green->red polarity, so one bar with a
    # "better -> worse" label communicates the semantics for the whole matrix.
    sm = plt.cm.ScalarMappable(norm=mcolors.Normalize(vmin=0.0, vmax=1.0), cmap=cmap)
    cbar = fig.colorbar(sm, ax=ax, orientation="vertical", fraction=0.045, pad=0.03)
    cbar.set_ticks([0.0, 1.0])
    cbar.set_ticklabels(["better\n(closer to\nngspice)", "worse\n(further\nfrom ref)"])
    cbar.ax.yaxis.set_tick_params(color="white")
    plt.setp(cbar.ax.yaxis.get_ticklabels(), color="white", fontsize=7)

    plt.tight_layout()
    fig.savefig(str(out_path), dpi=140, bbox_inches="tight", facecolor=fig.get_facecolor())
    plt.close(fig)

    # Read back rendered pixel dimensions for reporting.
    with Image.open(out_path) as im:
        px_w, px_h = im.size
    print(f"[dashboard] matrix-{suite} -> {out_path}  ({px_w}x{px_h})")
    return px_w, px_h


def plot_suite_matrices(
    circuit_records: list[dict[str, Any]],
    out_dir: Path,
) -> list[str]:
    """Render one compact matrix PNG per suite: matrix-<suite>.png.

    Replaces the single full-height accuracy-matrix.png. Returns the sorted list
    of suite names that produced a matrix.
    """
    by_suite: dict[str, list[dict[str, Any]]] = {}
    for r in circuit_records:
        by_suite.setdefault(r["suite"], []).append(r)

    suites = sorted(by_suite)
    for suite in suites:
        out_path = out_dir / f"matrix-{suite}.png"
        plot_suite_matrix(suite, by_suite[suite], out_path)
    return suites


# ---------------------------------------------------------------------------
# History append
# ---------------------------------------------------------------------------

def append_history(
    history_path: Path,
    timestamp: str,
    git_commit: str | None,
    summary: dict[str, Any],
    circuits: dict[str, dict[str, Any]],
) -> None:
    """Append this run to the history JSON (creates file if absent)."""
    history: list[dict[str, Any]] = []
    if history_path.exists():
        with history_path.open() as f:
            try:
                history = json.load(f)
            except json.JSONDecodeError:
                print(f"[dashboard] WARNING: could not parse {history_path}, starting fresh")

    run_record: dict[str, Any] = {
        "timestamp": timestamp,
        "git_commit": git_commit,
        "summary": summary,
        "circuits": {},
    }
    for key, info in circuits.items():
        m = info.get("metrics") or {}
        run_record["circuits"][key] = {
            "passed": info.get("passed", False),
            "wdf_pending": info.get("wdf_pending", False),
            "normalized_rms_error_db": m.get("normalized_rms_error_db"),
            "peak_error_db": m.get("peak_error_db"),
            "spectral_error_db": m.get("spectral_error_db"),
        }

    history.append(run_record)

    history_path.parent.mkdir(parents=True, exist_ok=True)
    with history_path.open("w") as f:
        json.dump(history, f, indent=2)
    print(f"[dashboard] history ({len(history)} runs) -> {history_path}")


# ---------------------------------------------------------------------------
# Main logic
# ---------------------------------------------------------------------------

def main() -> int:
    parser = argparse.ArgumentParser(
        description="Generate PedalKernel accuracy dashboard from validation report + golden .npy files.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "--report-json", type=Path,
        help="Path to the JSON report produced by `pedalkernel-validate --report <path> run`.",
    )
    parser.add_argument(
        "--golden-dir", type=Path,
        help="Root of the golden directory (contains {suite}/{test}/{signal}.npy).",
    )
    parser.add_argument(
        "--out-dir", required=True, type=Path,
        help="Output directory for generated PNGs + JSON files.",
    )
    parser.add_argument(
        "--timestamp",
        help=(
            "ISO-8601 timestamp for this run (e.g. 2026-06-15T02:00:00Z). "
            "Pass from outside — do NOT rely on datetime.now() for reproducibility. "
            "Required for a full run; ignored in --from-accuracy-json mode."
        ),
    )
    parser.add_argument(
        "--history-json", type=Path,
        help=(
            "Path to the history JSON file to append this run's errors to. "
            "Created if absent. Optional — omit to skip history."
        ),
    )
    parser.add_argument(
        "--from-accuracy-json", type=Path,
        help=(
            "Render ONLY the per-suite accuracy matrices from a committed "
            "accuracy.json (no validate report, no golden .npy, no engine "
            "rebuild). Mutually exclusive with --report-json. Per-circuit "
            "overlay/FFT PNGs are left untouched in this mode."
        ),
    )
    args = parser.parse_args()

    # -----------------------------------------------------------------------
    # Matrix-only mode: regenerate per-suite matrices from committed
    # accuracy.json without touching the engine, the validate report, or the
    # per-circuit PNGs. Used for readability/layout iteration on the matrices.
    # -----------------------------------------------------------------------
    if args.from_accuracy_json is not None:
        if args.report_json is not None:
            print("[dashboard] ERROR: --from-accuracy-json and --report-json are "
                  "mutually exclusive", file=sys.stderr)
            return 1
        if not args.from_accuracy_json.exists():
            print(f"[dashboard] ERROR: accuracy.json not found: "
                  f"{args.from_accuracy_json}", file=sys.stderr)
            return 1
        with args.from_accuracy_json.open() as f:
            acc = json.load(f)
        records = [
            {"key": key, **entry}
            for key, entry in acc.get("circuits", {}).items()
        ]
        args.out_dir.mkdir(parents=True, exist_ok=True)
        suites = plot_suite_matrices(records, args.out_dir)
        print()
        print("=" * 60)
        print("MATRIX-ONLY REGEN (from accuracy.json)")
        print("=" * 60)
        print(f"  Source      : {args.from_accuracy_json}")
        print(f"  Circuits    : {len(records)}")
        print(f"  Suites      : {len(suites)}  ({', '.join(suites)})")
        print(f"  Output dir  : {args.out_dir}")
        print("=" * 60)
        return 0

    # Full run requires the validate report + goldens + a literal timestamp.
    if args.report_json is None or args.golden_dir is None:
        print("[dashboard] ERROR: --report-json and --golden-dir are required for "
              "a full run (or use --from-accuracy-json for matrices only)",
              file=sys.stderr)
        return 1
    if args.timestamp is None:
        print("[dashboard] ERROR: --timestamp is required for a full run",
              file=sys.stderr)
        return 1

    # -----------------------------------------------------------------------
    # Load the validation report
    # -----------------------------------------------------------------------
    if not args.report_json.exists():
        print(f"[dashboard] ERROR: report-json not found: {args.report_json}", file=sys.stderr)
        return 1

    with args.report_json.open() as f:
        report = json.load(f)

    base_sample_rate = float(report.get("sample_rate", 96000))
    sample_rate = base_sample_rate * float(report.get("oversample", 4))
    base_nyquist_hz = base_sample_rate / 2.0  # audio band edge for the FFT marker
    git_commit = report.get("git_commit")

    args.out_dir.mkdir(parents=True, exist_ok=True)

    # -----------------------------------------------------------------------
    # Walk suites/tests/signals
    # -----------------------------------------------------------------------
    circuits: dict[str, dict[str, Any]] = {}
    circuit_records_for_matrix: list[dict[str, Any]] = []
    generated_pngs: list[Path] = []

    for suite_name, suite_data in report.get("suites", {}).items():
        for test_name, test_data in suite_data.get("tests", {}).items():
            for signal_data in test_data.get("signals", []):
                signal = signal_data["label"]
                key = key_for(suite_name, test_name, signal)

                # Paths to golden files
                spice_path = args.golden_dir / suite_name / test_name / f"{signal}.npy"
                wdf_path = args.golden_dir / suite_name / test_name / "wdf" / f"{signal}.npy"

                spice = load_npy(spice_path)
                wdf = load_npy(wdf_path)

                wdf_pending = wdf is None
                metrics: dict[str, Any] | None = None
                passed = signal_data.get("passed", False)
                error_msg = signal_data.get("error")

                if signal_data.get("comparison"):
                    metrics = dict(signal_data["comparison"])

                # If WDF golden absent, note it — but do NOT override `passed`.
                # The `passed` value already reflects the report's verdict (which
                # comes from a live WDF run against the ngspice golden, or from
                # compare-goldens when both committed goldens exist).
                # wdf_pending only controls the plot title and overlay waveform.
                if wdf_pending and error_msg is None:
                    error_msg = "WDF golden not yet committed (bead 85by.3); ngspice-only plot"

                circuit_entry: dict[str, Any] = {
                    "suite": suite_name,
                    "test": test_name,
                    "signal": signal,
                    "passed": passed,
                    "wdf_pending": wdf_pending,
                    "metrics": metrics,
                    "error": error_msg,
                }
                circuits[key] = circuit_entry
                circuit_records_for_matrix.append({"key": key, **circuit_entry})

                # Plot
                if spice is None:
                    print(f"[dashboard] SKIP {key}: ngspice golden missing at {spice_path}")
                    continue

                png_path = args.out_dir / "circuits" / suite_name / test_name / f"{signal}.png"
                plot_circuit(
                    suite=suite_name,
                    test=test_name,
                    signal=signal,
                    spice=spice,
                    wdf=wdf,
                    metrics=metrics,
                    sample_rate=sample_rate,
                    out_path=png_path,
                    base_nyquist_hz=base_nyquist_hz,
                )
                status = "WDF pending" if wdf_pending else ("PASS" if passed else "FAIL")
                print(f"[dashboard] {status:12s}  {key}  ->  {png_path.name}")
                generated_pngs.append(png_path)

    # -----------------------------------------------------------------------
    # Per-suite accuracy matrix PNGs (matrix-<suite>.png)
    # -----------------------------------------------------------------------
    matrix_suites = plot_suite_matrices(circuit_records_for_matrix, args.out_dir)
    generated_pngs.extend(args.out_dir / f"matrix-{s}.png" for s in matrix_suites)

    # -----------------------------------------------------------------------
    # accuracy.json
    # -----------------------------------------------------------------------
    total = len(circuits)
    wdf_pending_count = sum(1 for c in circuits.values() if c["wdf_pending"])
    # For wdf_pending circuits, `passed` reflects the report verdict (live WDF
    # run vs ngspice golden), which is valid even without committed WDF goldens.
    passed_count = sum(1 for c in circuits.values() if c["passed"])
    failed_count = sum(1 for c in circuits.values() if not c["passed"])
    pass_rate = passed_count / total if total else 0.0

    summary = {
        "total_circuits": total,
        "passed": passed_count,
        "failed": failed_count,
        "wdf_pending": wdf_pending_count,
        "pass_rate": round(pass_rate, 4),
    }

    accuracy_doc = {
        "schema_version": SCHEMA_VERSION,
        "run_timestamp": args.timestamp,
        "git_commit": git_commit,
        "sample_rate": report.get("sample_rate"),
        "oversample": report.get("oversample"),
        "summary": summary,
        "circuits": circuits,
    }

    accuracy_path = args.out_dir / "accuracy.json"
    with accuracy_path.open("w") as f:
        json.dump(accuracy_doc, f, indent=2)
    print(f"[dashboard] accuracy.json -> {accuracy_path}")

    # -----------------------------------------------------------------------
    # History JSON (optional)
    # -----------------------------------------------------------------------
    if args.history_json:
        append_history(
            history_path=args.history_json,
            timestamp=args.timestamp,
            git_commit=git_commit,
            summary=summary,
            circuits=circuits,
        )

    # -----------------------------------------------------------------------
    # Summary report to stdout
    # -----------------------------------------------------------------------
    print()
    print("=" * 60)
    print("DASHBOARD SUMMARY")
    print("=" * 60)
    print(f"  Timestamp   : {args.timestamp}")
    print(f"  Git commit  : {git_commit or 'unknown'}")
    print(f"  Circuits    : {total}")
    print(f"  Passed      : {passed_count}")
    print(f"  Failed      : {failed_count}")
    print(f"  WDF pending : {wdf_pending_count}")
    print(f"  Pass rate   : {pass_rate*100:.1f}%")
    print(f"  PNGs gen.   : {len(generated_pngs)}")
    print(f"  Output dir  : {args.out_dir}")
    print("=" * 60)

    # Return non-zero if any circuits failed (not counting WDF-pending)
    return 0 if failed_count == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
