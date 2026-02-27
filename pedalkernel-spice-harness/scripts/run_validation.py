#!/usr/bin/env python3
"""
PedalKernel SPICE Validation Harness — Main Runner

Orchestrates:
  1. Load test suite config
  2. Generate input signals
  3. Run ngspice simulations (or load golden references)
  4. Load WDF outputs from PedalKernel
  5. Compare using configured metrics
  6. Generate pass/fail report

Usage:
  python run_validation.py --suite all
  python run_validation.py --suite nonlinear --circuit diode_clipper
  python run_validation.py --suite fairchild --test tc_positions
  python run_validation.py --list
"""

import os
import sys
import json
import time
import subprocess
import hashlib
from pathlib import Path
from dataclasses import dataclass, field, asdict
from typing import Optional

import click
import yaml
import numpy as np
import scipy.signal as signal
import scipy.io.wavfile as wavfile
from tabulate import tabulate

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
HARNESS_ROOT = Path("/harness")
CIRCUITS_DIR = HARNESS_ROOT / "circuits"
GOLDEN_DIR = HARNESS_ROOT / "golden"
WDF_OUTPUT_DIR = HARNESS_ROOT / "wdf_output"
RESULTS_DIR = HARNESS_ROOT / "results"
CONFIG_DIR = HARNESS_ROOT / "config"

SAMPLE_RATE = int(os.environ.get("SAMPLE_RATE", 96000))
OVERSAMPLE = int(os.environ.get("OVERSAMPLE", 4))
INTERNAL_RATE = SAMPLE_RATE * OVERSAMPLE


# ---------------------------------------------------------------------------
# Data classes
# ---------------------------------------------------------------------------
@dataclass
class TestResult:
    suite: str
    test: str
    signal_label: str
    metric: str
    value: float
    threshold: float
    passed: bool
    unit: str = ""
    detail: str = ""


@dataclass
class SuiteReport:
    suite: str
    total: int = 0
    passed: int = 0
    failed: int = 0
    skipped: int = 0
    results: list = field(default_factory=list)
    wall_time_s: float = 0.0


# ---------------------------------------------------------------------------
# Signal generators
# ---------------------------------------------------------------------------
def generate_signal(sig_config: dict, sample_rate: int) -> np.ndarray:
    """Generate a test signal from config."""
    sig_type = sig_config["type"]
    duration = sig_config.get("duration", 0.1)
    n_samples = int(duration * sample_rate)
    t = np.arange(n_samples) / sample_rate

    if sig_type == "impulse":
        x = np.zeros(n_samples)
        x[0] = sig_config.get("amplitude", 1.0)
        return x

    elif sig_type == "sine":
        amp = sig_config.get("amplitude", 1.0)
        freq = sig_config["frequency"]
        return amp * np.sin(2.0 * np.pi * freq * t)

    elif sig_type == "two_tone":
        amp = sig_config.get("amplitude", 1.0)
        f1, f2 = sig_config["f1"], sig_config["f2"]
        return amp * 0.5 * (np.sin(2 * np.pi * f1 * t) + np.sin(2 * np.pi * f2 * t))

    elif sig_type == "exp_sweep":
        f_start = sig_config.get("f_start", 20)
        f_end = sig_config.get("f_end", 20000)
        amp = sig_config.get("amplitude", 1.0)
        sweep = signal.chirp(t, f_start, duration, f_end, method="logarithmic")
        return amp * sweep

    elif sig_type == "silence":
        return np.zeros(n_samples)

    elif sig_type == "tone_burst":
        freq = sig_config["frequency"]
        amp_dbvu = sig_config.get("amplitude_dbvu", 0)
        amp = 10 ** (amp_dbvu / 20.0) * 0.7746  # dBVU to Vpeak (0 dBVU ≈ +4 dBu)
        on_ms = sig_config.get("on_ms", 100)
        off_ms = sig_config.get("off_ms", 900)
        reps = sig_config.get("repetitions", 1)
        period_samples = int((on_ms + off_ms) * sample_rate / 1000)
        on_samples = int(on_ms * sample_rate / 1000)
        total = period_samples * reps
        x = np.zeros(total)
        t_local = np.arange(on_samples) / sample_rate
        burst = amp * np.sin(2 * np.pi * freq * t_local)
        for r in range(reps):
            start = r * period_samples
            x[start:start + on_samples] = burst
        return x

    elif sig_type == "level_sweep":
        freq = sig_config["frequency"]
        levels = sig_config["levels_dbvu"]
        dur_per = sig_config.get("duration_per_level", 0.5)
        segments = []
        for lvl in levels:
            amp = 10 ** (lvl / 20.0) * 0.7746
            n = int(dur_per * sample_rate)
            t_seg = np.arange(n) / sample_rate
            segments.append(amp * np.sin(2 * np.pi * freq * t_seg))
        return np.concatenate(segments)

    else:
        raise ValueError(f"Unknown signal type: {sig_type}")


# ---------------------------------------------------------------------------
# ngspice interface
# ---------------------------------------------------------------------------
def generate_spice_netlist(circuit_path: str, input_signal_file: str,
                           output_raw_file: str, params: dict = None) -> str:
    """
    Generate a complete ngspice netlist that:
    - Includes the circuit under test
    - Uses a PWL voltage source driven by the input signal file
    - Runs .TRAN with fixed timestep
    - Writes output to raw file
    """
    effective_rate = INTERNAL_RATE
    timestep = 1.0 / effective_rate

    # Read circuit template
    with open(circuit_path, "r") as f:
        circuit = f.read()

    # Parameter substitutions
    if params:
        for k, v in params.items():
            circuit = circuit.replace(f"${{{k}}}", str(v))

    # The circuit template must define:
    #   - A node 'v_in' for input
    #   - A node 'v_out' for output
    #   - Include .SUBCKT if needed
    #   - NOT include .TRAN, .END, or input source (we add those)

    netlist = f"""\
* PedalKernel Validation Harness — Auto-generated netlist
* Circuit: {circuit_path}
* Timestep: {timestep:.12e} s ({effective_rate} Hz)
* Generated: {time.strftime('%Y-%m-%dT%H:%M:%SZ')}

{circuit}

* --- Input source (file-driven PWL) ---
Vin v_in 0 PWL FILE="{input_signal_file}"

* --- Analysis ---
.OPTIONS RELTOL=1e-6 ABSTOL=1e-12 VNTOL=1e-9
.OPTIONS METHOD=GEAR MAXORD=2
.OPTIONS ITL1=500 ITL2=200 ITL4=50

.CONTROL
  set filetype=binary
  tran {timestep:.12e} {{duration}} 0 {timestep:.12e}
  write {output_raw_file} v(v_out)
  quit
.ENDC

.END
"""
    return netlist


def write_pwl_file(signal_data: np.ndarray, sample_rate: int, filepath: str):
    """Write signal as ngspice PWL (piecewise linear) text file."""
    with open(filepath, "w") as f:
        for i, val in enumerate(signal_data):
            t = i / sample_rate
            f.write(f"{t:.12e} {val:.12e}\n")


def run_ngspice(netlist_path: str, timeout: int = 300) -> bool:
    """Run ngspice in batch mode. Returns True on success."""
    result = subprocess.run(
        ["ngspice", "-b", "-o", "/dev/null", netlist_path],
        capture_output=True, text=True, timeout=timeout
    )
    if result.returncode != 0:
        print(f"  [SPICE ERROR] {result.stderr[:500]}", file=sys.stderr)
        return False
    return True


def read_raw_file(raw_path: str) -> np.ndarray:
    """Read ngspice binary raw file and return output voltage array."""
    # Use numpy to parse the binary raw format
    # ngspice raw format: header + binary doubles
    # For simplicity, we'll use the ASCII fallback or PySpice
    try:
        from PySpice.Spice.NgSpice.RawFile import RawFile
        raw = RawFile(raw_path)
        return np.array(raw.variables[0].data, dtype=np.float64)
    except ImportError:
        # Fallback: parse manually
        # This is a simplified parser for ngspice binary raw files
        with open(raw_path, "rb") as f:
            header = b""
            while True:
                line = f.readline()
                header += line
                if b"Binary:" in line:
                    break

            # Parse header for number of points and variables
            header_str = header.decode("ascii", errors="replace")
            n_points = 0
            n_vars = 0
            for line in header_str.split("\n"):
                if line.startswith("No. Points:"):
                    n_points = int(line.split(":")[1].strip())
                elif line.startswith("No. Variables:"):
                    n_vars = int(line.split(":")[1].strip())

            # Read binary data: n_points * n_vars * 8 bytes (double)
            data = np.frombuffer(f.read(), dtype=np.float64)
            data = data.reshape(n_points, n_vars)
            # Column 0 is time, column 1 is first variable (v_out)
            return data[:, 1] if n_vars > 1 else data[:, 0]


# ---------------------------------------------------------------------------
# Comparison metrics
# ---------------------------------------------------------------------------
def compute_thd(signal_data: np.ndarray, fundamental_hz: float,
                sample_rate: int, n_harmonics: int = 10) -> float:
    """Compute THD in dB from FFT."""
    N = len(signal_data)
    win = np.blackman(N)
    spectrum = np.fft.rfft(signal_data * win)
    freqs = np.fft.rfftfreq(N, 1.0 / sample_rate)
    magnitudes = np.abs(spectrum)

    # Find fundamental bin
    fund_bin = np.argmin(np.abs(freqs - fundamental_hz))
    fund_power = magnitudes[fund_bin] ** 2

    # Sum harmonic powers
    harm_power = 0.0
    for h in range(2, n_harmonics + 1):
        harm_freq = fundamental_hz * h
        if harm_freq > sample_rate / 2:
            break
        harm_bin = np.argmin(np.abs(freqs - harm_freq))
        harm_power += magnitudes[harm_bin] ** 2

    if fund_power < 1e-30:
        return -200.0
    return 10.0 * np.log10(harm_power / fund_power)


def compute_even_odd_ratio(signal_data: np.ndarray, fundamental_hz: float,
                            sample_rate: int, n_harmonics: int = 10) -> float:
    """Ratio of even harmonic power to odd harmonic power, in dB."""
    N = len(signal_data)
    win = np.blackman(N)
    spectrum = np.fft.rfft(signal_data * win)
    freqs = np.fft.rfftfreq(N, 1.0 / sample_rate)
    magnitudes = np.abs(spectrum)

    even_power = 0.0
    odd_power = 0.0

    for h in range(2, n_harmonics + 1):
        harm_freq = fundamental_hz * h
        if harm_freq > sample_rate / 2:
            break
        harm_bin = np.argmin(np.abs(freqs - harm_freq))
        power = magnitudes[harm_bin] ** 2
        if h % 2 == 0:
            even_power += power
        else:
            odd_power += power

    if odd_power < 1e-30:
        return 0.0
    return 10.0 * np.log10(even_power / (odd_power + 1e-30))


def normalized_rms_error(wdf: np.ndarray, ref: np.ndarray) -> float:
    """Normalized RMS error in dB."""
    # Align lengths
    n = min(len(wdf), len(ref))
    wdf, ref = wdf[:n], ref[:n]
    err = np.sqrt(np.mean((wdf - ref) ** 2))
    ref_rms = np.sqrt(np.mean(ref ** 2))
    if ref_rms < 1e-30:
        return -200.0
    return 20.0 * np.log10(err / ref_rms)


def peak_error_db(wdf: np.ndarray, ref: np.ndarray) -> float:
    """Peak absolute error in dB relative to reference peak."""
    n = min(len(wdf), len(ref))
    wdf, ref = wdf[:n], ref[:n]
    peak_err = np.max(np.abs(wdf - ref))
    ref_peak = np.max(np.abs(ref))
    if ref_peak < 1e-30:
        return -200.0
    return 20.0 * np.log10(peak_err / ref_peak)


def spectral_error_db(wdf: np.ndarray, ref: np.ndarray,
                       sample_rate: int, max_freq: float = None) -> float:
    """Maximum spectral magnitude error in dB."""
    n = min(len(wdf), len(ref))
    wdf, ref = wdf[:n], ref[:n]
    win = np.blackman(n)

    W = np.abs(np.fft.rfft(wdf * win))
    R = np.abs(np.fft.rfft(ref * win))
    freqs = np.fft.rfftfreq(n, 1.0 / sample_rate)

    if max_freq is None:
        max_freq = sample_rate / 4

    mask = freqs <= max_freq
    # Avoid log of zero
    W_db = 20 * np.log10(W[mask] + 1e-30)
    R_db = 20 * np.log10(R[mask] + 1e-30)

    # Only compare where reference has significant energy
    significant = R_db > (np.max(R_db) - 80)
    if not np.any(significant):
        return 0.0
    return np.max(np.abs(W_db[significant] - R_db[significant]))


# ---------------------------------------------------------------------------
# Test runner
# ---------------------------------------------------------------------------
def load_config() -> dict:
    config_path = CONFIG_DIR / "validation_suite.yml"
    with open(config_path) as f:
        return yaml.safe_load(f)


def run_test(suite_name: str, test_name: str, test_config: dict,
             global_config: dict) -> list[TestResult]:
    """Run a single test case, return list of TestResult."""
    results = []
    circuit_path = CIRCUITS_DIR / test_config["circuit"]

    if not circuit_path.exists():
        results.append(TestResult(
            suite=suite_name, test=test_name, signal_label="",
            metric="circuit_exists", value=0, threshold=1,
            passed=False, detail=f"Circuit file not found: {circuit_path}"
        ))
        return results

    criteria = test_config.get("pass_criteria", {})

    for sig_config in test_config.get("signals", []):
        label = sig_config.get("label", sig_config["type"])

        # Check for golden reference
        golden_path = GOLDEN_DIR / suite_name / test_name / f"{label}.npy"
        wdf_path = WDF_OUTPUT_DIR / suite_name / test_name / f"{label}.npy"

        if not golden_path.exists():
            results.append(TestResult(
                suite=suite_name, test=test_name, signal_label=label,
                metric="golden_exists", value=0, threshold=1,
                passed=False, detail=f"Golden reference not found: {golden_path}"
            ))
            continue

        if not wdf_path.exists():
            results.append(TestResult(
                suite=suite_name, test=test_name, signal_label=label,
                metric="wdf_output_exists", value=0, threshold=1,
                passed=False, detail=f"WDF output not found: {wdf_path}"
            ))
            continue

        ref = np.load(golden_path)
        wdf = np.load(wdf_path)

        sr = global_config.get("sample_rate", SAMPLE_RATE)

        # Time domain comparison
        if "normalized_rms_error_db" in criteria:
            val = normalized_rms_error(wdf, ref)
            thresh = criteria["normalized_rms_error_db"]
            results.append(TestResult(
                suite=suite_name, test=test_name, signal_label=label,
                metric="normalized_rms_error", value=val, threshold=thresh,
                passed=(val <= thresh), unit="dB"
            ))

        if "peak_error_db" in criteria:
            val = peak_error_db(wdf, ref)
            thresh = criteria["peak_error_db"]
            results.append(TestResult(
                suite=suite_name, test=test_name, signal_label=label,
                metric="peak_error", value=val, threshold=thresh,
                passed=(val <= thresh), unit="dB"
            ))

        # THD comparison
        if "thd_error_db" in criteria:
            for metric_cfg in test_config.get("metrics", []):
                if metric_cfg.get("type") == "thd":
                    fund = metric_cfg["fundamental"]
                    thd_wdf = compute_thd(wdf, fund, sr)
                    thd_ref = compute_thd(ref, fund, sr)
                    val = abs(thd_wdf - thd_ref)
                    thresh = criteria["thd_error_db"]
                    results.append(TestResult(
                        suite=suite_name, test=test_name, signal_label=label,
                        metric="thd_error", value=val, threshold=thresh,
                        passed=(val <= thresh), unit="dB",
                        detail=f"WDF THD={thd_wdf:.1f}dB, REF THD={thd_ref:.1f}dB"
                    ))

        # Spectral comparison
        if "spectral_error_db" in criteria:
            val = spectral_error_db(wdf, ref, sr)
            thresh = criteria["spectral_error_db"]
            results.append(TestResult(
                suite=suite_name, test=test_name, signal_label=label,
                metric="spectral_error", value=val, threshold=thresh,
                passed=(val <= thresh), unit="dB"
            ))

        # Even/odd ratio (push-pull test)
        if "even_harmonic_suppression_db" in criteria:
            ratio = compute_even_odd_ratio(wdf, 1000, sr)
            thresh = -criteria["even_harmonic_suppression_db"]  # ratio should be negative
            results.append(TestResult(
                suite=suite_name, test=test_name, signal_label=label,
                metric="even_odd_ratio", value=ratio, threshold=thresh,
                passed=(ratio <= thresh), unit="dB",
                detail=f"Even/odd ratio={ratio:.1f}dB (want < {thresh:.0f}dB)"
            ))

    return results


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
@click.command()
@click.option("--suite", default="all", help="Suite to run: all, linear, nonlinear, fairchild, stress")
@click.option("--circuit", default=None, help="Run only a specific circuit within the suite")
@click.option("--test", default=None, help="Run only a specific test within the suite")
@click.option("--list", "list_tests", is_flag=True, help="List available tests without running")
@click.option("--verbose", "-v", is_flag=True, help="Verbose output")
def main(suite, circuit, test, list_tests, verbose):
    """PedalKernel SPICE Validation Harness"""

    config = load_config()
    suites_config = config.get("suites", {})

    if list_tests:
        for s_name, s_config in suites_config.items():
            click.echo(f"\n{'='*60}")
            click.echo(f"Suite: {s_name} — {s_config.get('description', '')}")
            click.echo(f"{'='*60}")
            for t_name, t_config in s_config.get("tests", {}).items():
                click.echo(f"  {t_name}: {t_config.get('description', '')}")
        return

    # Select suites to run
    if suite == "all":
        run_suites = suites_config
    elif suite in suites_config:
        run_suites = {suite: suites_config[suite]}
    else:
        click.echo(f"Unknown suite: {suite}", err=True)
        sys.exit(1)

    # Ensure results dir exists
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)

    all_reports = []
    total_pass = 0
    total_fail = 0
    total_skip = 0

    for s_name, s_config in run_suites.items():
        report = SuiteReport(suite=s_name)
        t0 = time.time()

        click.echo(f"\n{'='*60}")
        click.echo(f"Suite: {s_name}")
        click.echo(f"{'='*60}")

        for t_name, t_config in s_config.get("tests", {}).items():
            if circuit and t_name != circuit:
                continue
            if test and t_name != test:
                continue

            click.echo(f"\n  [{t_name}] {t_config.get('description', '')}")

            results = run_test(s_name, t_name, t_config, config.get("global", {}))

            for r in results:
                report.total += 1
                if r.passed:
                    report.passed += 1
                    total_pass += 1
                    status = click.style("PASS", fg="green")
                else:
                    report.failed += 1
                    total_fail += 1
                    status = click.style("FAIL", fg="red")

                click.echo(f"    {status} {r.metric}: {r.value:.2f} {r.unit} "
                           f"(threshold: {r.threshold:.2f})")
                if verbose and r.detail:
                    click.echo(f"         {r.detail}")

                report.results.append(asdict(r))

        report.wall_time_s = time.time() - t0
        all_reports.append(report)

    # Summary
    click.echo(f"\n{'='*60}")
    click.echo(f"SUMMARY")
    click.echo(f"{'='*60}")

    table_data = []
    for r in all_reports:
        status = click.style("PASS", fg="green") if r.failed == 0 else click.style("FAIL", fg="red")
        table_data.append([r.suite, r.passed, r.failed, r.total, f"{r.wall_time_s:.1f}s", status])

    click.echo(tabulate(table_data,
                        headers=["Suite", "Pass", "Fail", "Total", "Time", "Status"],
                        tablefmt="simple"))

    click.echo(f"\nTotal: {total_pass} passed, {total_fail} failed, {total_skip} skipped")

    # Write JSON report
    report_path = RESULTS_DIR / f"validation_report_{time.strftime('%Y%m%d_%H%M%S')}.json"
    with open(report_path, "w") as f:
        json.dump({
            "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ"),
            "config": {
                "sample_rate": SAMPLE_RATE,
                "oversample": OVERSAMPLE,
                "internal_rate": INTERNAL_RATE,
            },
            "summary": {
                "total_pass": total_pass,
                "total_fail": total_fail,
                "total_skip": total_skip,
            },
            "suites": [asdict(r) if hasattr(r, '__dataclass_fields__') else
                       {"suite": r.suite, "total": r.total, "passed": r.passed,
                        "failed": r.failed, "wall_time_s": r.wall_time_s,
                        "results": r.results}
                       for r in all_reports]
        }, f, indent=2)

    click.echo(f"\nReport written to: {report_path}")

    # Exit code
    sys.exit(1 if total_fail > 0 else 0)


if __name__ == "__main__":
    main()
