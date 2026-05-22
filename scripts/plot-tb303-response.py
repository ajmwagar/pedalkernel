#!/usr/bin/env python3
"""Generate TB-303 response/FFT SVGs from the current compiled WDF."""

from __future__ import annotations

import argparse
import csv
import math
import os
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUT = ROOT / "pedalkernel" / "target" / "tb303-publish"


COLORS = {
    "htb_db": "#777777",
    "tenpole_k0_db": "#1f77b4",
    "tenpole_k03_db": "#17becf",
    "tenpole_k085_db": "#003f5c",
    "wdf_k0_db": "#d62728",
    "wdf_k03_db": "#ff7f0e",
    "wdf_k085_db": "#7f0000",
    "wdf_db": "#d62728",
}


LABELS = {
    "htb_db": "H_tb core",
    "tenpole_k0_db": "10-pole k=0",
    "tenpole_k03_db": "10-pole k=0.3",
    "tenpole_k085_db": "10-pole k=0.85",
    "wdf_k0_db": "PK WDF k=0",
    "wdf_k03_db": "PK WDF k=0.3",
    "wdf_k085_db": "PK WDF k=0.85",
    "wdf_db": "PK WDF FFT",
}


def run_export(out_dir: Path) -> None:
    env = os.environ.copy()
    env["PK_TB303_PUBLISH_DIR"] = str(out_dir)
    subprocess.run(
        [
            "cargo",
            "test",
            "-p",
            "pedalkernel",
            "--lib",
            "tb303_export_publish_response_data",
            "--",
            "--ignored",
            "--nocapture",
        ],
        cwd=ROOT,
        env=env,
        check=True,
    )


def read_csv(path: Path) -> list[dict[str, float]]:
    with path.open(newline="") as f:
        return [{k: float(v) for k, v in row.items()} for row in csv.DictReader(f)]


def available_keys(rows: list[dict[str, float]], keys: list[str]) -> list[str]:
    if not rows:
        return []
    return [key for key in keys if key in rows[0]]


def nice_ticks(min_v: float, max_v: float, step: float) -> list[float]:
    start = math.floor(min_v / step) * step
    end = math.ceil(max_v / step) * step
    ticks = []
    v = start
    while v <= end + step * 0.5:
        ticks.append(v)
        v += step
    return ticks


def write_svg(
    path: Path,
    title: str,
    rows: list[dict[str, float]],
    y_keys: list[str],
    y_label: str,
    y_min: float | None = None,
    y_max: float | None = None,
) -> None:
    width, height = 1100, 680
    left, right, top, bottom = 86, 32, 58, 86
    plot_w = width - left - right
    plot_h = height - top - bottom
    xs = [r["freq_hz"] for r in rows]
    x_min, x_max = min(xs), max(xs)
    lx_min, lx_max = math.log10(x_min), math.log10(x_max)
    values = [r[k] for r in rows for k in y_keys]
    y_min = min(values) if y_min is None else y_min
    y_max = max(values) if y_max is None else y_max
    pad = max(1.0, (y_max - y_min) * 0.08)
    y_min, y_max = y_min - pad, y_max + pad

    def x_px(freq: float) -> float:
        return left + (math.log10(freq) - lx_min) / (lx_max - lx_min) * plot_w

    def y_px(db: float) -> float:
        return top + (y_max - db) / (y_max - y_min) * plot_h

    def polyline(key: str) -> str:
        points = " ".join(f"{x_px(r['freq_hz']):.1f},{y_px(r[key]):.1f}" for r in rows)
        return (
            f'<polyline fill="none" stroke="{COLORS[key]}" stroke-width="3" '
            f'stroke-linejoin="round" stroke-linecap="round" points="{points}" />'
        )

    y_ticks = nice_ticks(y_min, y_max, 6.0)
    x_ticks = [50, 100, 200, 500, 1000, 2000, 5000, 10000, 16000]
    lines = [
        '<svg xmlns="http://www.w3.org/2000/svg" width="1100" height="680" viewBox="0 0 1100 680">',
        '<rect width="1100" height="680" fill="#fbfaf7"/>',
        f'<text x="{left}" y="34" font-family="Inter, Helvetica, Arial, sans-serif" font-size="26" font-weight="700" fill="#151515">{title}</text>',
        f'<text x="{left}" y="650" font-family="Inter, Helvetica, Arial, sans-serif" font-size="16" fill="#333">Frequency (Hz, log scale)</text>',
        f'<text x="24" y="{top + plot_h / 2:.1f}" font-family="Inter, Helvetica, Arial, sans-serif" font-size="16" fill="#333" transform="rotate(-90 24 {top + plot_h / 2:.1f})">{y_label}</text>',
        f'<rect x="{left}" y="{top}" width="{plot_w}" height="{plot_h}" fill="#ffffff" stroke="#d8d2c8"/>',
    ]
    for tick in y_ticks:
        y = y_px(tick)
        lines.append(f'<line x1="{left}" y1="{y:.1f}" x2="{left + plot_w}" y2="{y:.1f}" stroke="#ece6dc"/>')
        lines.append(f'<text x="{left - 12}" y="{y + 5:.1f}" text-anchor="end" font-family="Inter, Helvetica, Arial, sans-serif" font-size="13" fill="#555">{tick:.0f}</text>')
    for tick in x_ticks:
        if tick < x_min or tick > x_max:
            continue
        x = x_px(tick)
        label = f"{tick // 1000}k" if tick >= 1000 else str(tick)
        lines.append(f'<line x1="{x:.1f}" y1="{top}" x2="{x:.1f}" y2="{top + plot_h}" stroke="#f2ede6"/>')
        lines.append(f'<text x="{x:.1f}" y="{top + plot_h + 24}" text-anchor="middle" font-family="Inter, Helvetica, Arial, sans-serif" font-size="13" fill="#555">{label}</text>')
    lines.extend(polyline(k) for k in y_keys)

    legend_x = left + plot_w - 250
    legend_y = top + 24
    for i, key in enumerate(y_keys):
        y = legend_y + i * 24
        lines.append(f'<line x1="{legend_x}" y1="{y}" x2="{legend_x + 28}" y2="{y}" stroke="{COLORS[key]}" stroke-width="4"/>')
        lines.append(f'<text x="{legend_x + 38}" y="{y + 5}" font-family="Inter, Helvetica, Arial, sans-serif" font-size="14" fill="#222">{LABELS[key]}</text>')
    lines.append("</svg>")
    path.write_text("\n".join(lines))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument(
        "--generate",
        action="store_true",
        help="run the slow Rust exporter before plotting; otherwise reuse existing CSVs",
    )
    parser.add_argument("--extras", action="store_true", help="also plot optional nonlinear probe CSVs")
    args = parser.parse_args()

    args.out.mkdir(parents=True, exist_ok=True)
    if args.generate:
        run_export(args.out)

    response_path = args.out / "tb303_response.csv"
    fft_path = args.out / "tb303_fft.csv"
    if not response_path.exists() or not fft_path.exists():
        raise SystemExit(
            f"Missing {response_path} or {fft_path}; run with --generate after confirming the exporter cost is acceptable."
        )

    response = read_csv(response_path)
    fft = read_csv(fft_path)
    write_svg(
        args.out / "tb303_response_k0.svg",
        "TB-303 k=0 Response: Stinchcombe vs PK WDF",
        response,
        available_keys(response, ["htb_db", "tenpole_k0_db", "wdf_k0_db"]),
        "Normalized gain (dB @ 100 Hz)",
        -60,
        12,
    )
    write_svg(
        args.out / "tb303_response_resonance.svg",
        "TB-303 Resonance Diagnostic: 10-pole vs PK WDF",
        response,
        available_keys(response, ["tenpole_k03_db", "wdf_k03_db", "tenpole_k085_db", "wdf_k085_db"]),
        "Normalized gain (dB @ 100 Hz)",
        -60,
        36,
    )
    write_svg(
        args.out / "tb303_fft.svg",
        "TB-303 PK WDF Harmonic FFT",
        fft,
        ["wdf_db"],
        "Level (dB @ 110 Hz)",
        -90,
        6,
    )
    if args.extras:
        squelch_fft = read_csv(args.out / "tb303_squelch_fft.csv")
        self_osc_fft = read_csv(args.out / "tb303_self_osc_fft.csv")
        accent_fft = read_csv(args.out / "tb303_accent_cv_fft.csv")
        write_svg(
            args.out / "tb303_squelch_fft.svg",
            "TB-303 PK WDF Squelch Sweep FFT",
            squelch_fft,
            ["wdf_db"],
            "Level (dB @ 110 Hz)",
            -90,
            12,
        )
        write_svg(
            args.out / "tb303_self_osc_fft.svg",
            "TB-303 PK WDF High-Resonance Quiet-Input Probe",
            self_osc_fft,
            ["wdf_db"],
            "Level (dB @ 110 Hz)",
            -90,
            12,
        )
        write_svg(
            args.out / "tb303_accent_cv_fft.svg",
            "TB-303 PK WDF Accent/CV Dynamics Probe",
            accent_fft,
            ["wdf_db"],
            "Level (dB @ 110 Hz)",
            -90,
            12,
        )
        print(f"Wrote {args.out / 'tb303_squelch_fft.svg'}")
        print(f"Wrote {args.out / 'tb303_self_osc_fft.svg'}")
        print(f"Wrote {args.out / 'tb303_accent_cv_fft.svg'}")
    print(f"Wrote {args.out / 'tb303_response_k0.svg'}")
    print(f"Wrote {args.out / 'tb303_response_resonance.svg'}")
    print(f"Wrote {args.out / 'tb303_fft.svg'}")
    print(f"Wrote {args.out / 'tb303_wdf_k03.wav'}")


if __name__ == "__main__":
    main()
