#!/usr/bin/env python3
"""
PedalKernel Golden Reference Generator

Runs ngspice for every test case in the validation suite and saves
the output as .npy files in the golden/ directory.

These golden references are the SPICE ground truth against which
WDF compiler output is compared.

Usage:
  python generate_golden.py --all
  python generate_golden.py --suite nonlinear
  python generate_golden.py --suite nonlinear --circuit diode_clipper
"""

import os
import sys
import json
import time
import hashlib
import tempfile
import subprocess
from pathlib import Path

import click
import yaml
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from run_validation import (
    generate_signal, write_pwl_file,
    HARNESS_ROOT, CIRCUITS_DIR, GOLDEN_DIR, CONFIG_DIR,
    SAMPLE_RATE, OVERSAMPLE, INTERNAL_RATE,
)


def generate_ngspice_control(circuit_path: Path, pwl_path: str,
                              duration: float, output_node: str = "v_out") -> str:
    """
    Build a complete ngspice-ready netlist from a circuit template + PWL input.
    """
    timestep = 1.0 / INTERNAL_RATE

    with open(circuit_path) as f:
        circuit_body = f.read()

    netlist = f"""\
* PedalKernel Golden Reference Generation
* Circuit: {circuit_path.name}
* Internal rate: {INTERNAL_RATE} Hz (timestep: {timestep:.12e} s)
* Duration: {duration} s

.TITLE Golden Reference — {circuit_path.stem}

{circuit_body}

* Input source — driven by PWL file
VIN v_in 0 PWL file="{pwl_path}"

* Simulation control
.OPTIONS RELTOL=1e-6 ABSTOL=1e-12 VNTOL=1e-9
.OPTIONS METHOD=GEAR MAXORD=2
.OPTIONS ITL1=500 ITL2=200 ITL4=50
* Disable random number generation for determinism
.OPTIONS SEED=42

.TRAN {timestep:.12e} {duration:.12e} 0 {timestep:.12e} UIC

.END
"""
    return netlist


def run_ngspice_and_extract(netlist_str: str, duration: float,
                             node: str = "v_out") -> np.ndarray:
    """
    Run ngspice batch, parse output, return uniform-sampled numpy array.
    """
    with tempfile.TemporaryDirectory(prefix="pk_golden_") as tmpdir:
        netlist_path = os.path.join(tmpdir, "circuit.spice")
        raw_path = os.path.join(tmpdir, "output.raw")

        # Modify netlist to include .CONTROL block for raw write
        control_block = f"""
.CONTROL
  set filetype=ascii
  run
  wrdata {os.path.join(tmpdir, 'output.txt')} {node}
  quit
.ENDC
"""
        # Insert control block before .END
        netlist_with_ctrl = netlist_str.replace(".END", control_block + "\n.END")

        with open(netlist_path, "w") as f:
            f.write(netlist_with_ctrl)

        # Run ngspice
        result = subprocess.run(
            ["ngspice", "-b", netlist_path],
            capture_output=True, text=True, timeout=600
        )

        if result.returncode != 0:
            # Try to extract useful error info
            err_lines = [l for l in result.stderr.split("\n") if "error" in l.lower()]
            raise RuntimeError(
                f"ngspice failed (exit {result.returncode}):\n"
                + "\n".join(err_lines[:10])
                + f"\n\nFull stderr:\n{result.stderr[:2000]}"
            )

        # Parse wrdata output (two columns: time, value)
        data_path = os.path.join(tmpdir, "output.txt")
        if not os.path.exists(data_path):
            raise FileNotFoundError(f"ngspice did not produce output file. "
                                    f"Check circuit for convergence issues.")

        raw_data = np.loadtxt(data_path)
        if raw_data.ndim == 1:
            # Single column — just values
            return raw_data
        else:
            t_spice = raw_data[:, 0]
            v_spice = raw_data[:, 1]

        # Resample to uniform grid at INTERNAL_RATE
        n_expected = int(duration * INTERNAL_RATE)
        t_uniform = np.arange(n_expected) / INTERNAL_RATE

        # High-quality interpolation
        from scipy.interpolate import interp1d
        interp = interp1d(t_spice, v_spice, kind="cubic",
                          fill_value="extrapolate", bounds_error=False)
        v_uniform = interp(t_uniform)

        # Decimate to output sample rate (INTERNAL_RATE → SAMPLE_RATE)
        if OVERSAMPLE > 1:
            from scipy.signal import decimate
            v_output = decimate(v_uniform, OVERSAMPLE, ftype="fir", zero_phase=True)
        else:
            v_output = v_uniform

        return v_output


def compute_circuit_hash(circuit_path: Path) -> str:
    """SHA256 of circuit file for change detection."""
    with open(circuit_path, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()[:12]


@click.command()
@click.option("--all", "run_all", is_flag=True, help="Regenerate all golden references")
@click.option("--suite", default=None, help="Specific suite")
@click.option("--circuit", default=None, help="Specific circuit within suite")
@click.option("--force", is_flag=True, help="Overwrite existing golden refs")
def main(run_all, suite, circuit, force):
    """Generate golden SPICE reference outputs."""

    config_path = CONFIG_DIR / "validation_suite.yml"
    with open(config_path) as f:
        config = yaml.safe_load(f)

    suites = config.get("suites", {})

    if run_all:
        target_suites = suites
    elif suite:
        if suite not in suites:
            click.echo(f"Unknown suite: {suite}", err=True)
            sys.exit(1)
        target_suites = {suite: suites[suite]}
    else:
        click.echo("Specify --all or --suite <name>", err=True)
        sys.exit(1)

    total_generated = 0
    total_skipped = 0
    total_failed = 0

    for s_name, s_config in target_suites.items():
        click.echo(f"\n{'='*60}")
        click.echo(f"Suite: {s_name}")
        click.echo(f"{'='*60}")

        for t_name, t_config in s_config.get("tests", {}).items():
            if circuit and t_name != circuit:
                continue

            circuit_path = CIRCUITS_DIR / t_config["circuit"]
            if not circuit_path.exists():
                click.echo(f"  [{t_name}] SKIP — circuit not found: {circuit_path}")
                total_skipped += 1
                continue

            click.echo(f"\n  [{t_name}] {t_config.get('description', '')}")

            for sig_config in t_config.get("signals", []):
                label = sig_config.get("label", sig_config["type"])
                golden_dir = GOLDEN_DIR / s_name / t_name
                golden_path = golden_dir / f"{label}.npy"
                meta_path = golden_dir / f"{label}.meta.json"

                # Skip if exists and not forced
                if golden_path.exists() and not force:
                    # Check if circuit has changed
                    if meta_path.exists():
                        with open(meta_path) as f:
                            meta = json.load(f)
                        current_hash = compute_circuit_hash(circuit_path)
                        if meta.get("circuit_hash") == current_hash:
                            click.echo(f"    [{label}] exists, skipping (use --force to regenerate)")
                            total_skipped += 1
                            continue
                        else:
                            click.echo(f"    [{label}] circuit changed, regenerating...")

                # Generate input signal
                input_signal = generate_signal(sig_config, INTERNAL_RATE)
                duration = len(input_signal) / INTERNAL_RATE

                click.echo(f"    [{label}] generating... ({len(input_signal)} samples, "
                           f"{duration:.3f}s at {INTERNAL_RATE} Hz)")

                try:
                    # Write PWL file
                    with tempfile.NamedTemporaryFile(mode="w", suffix=".pwl",
                                                      delete=False) as pwl_file:
                        write_pwl_file(input_signal, INTERNAL_RATE, pwl_file.name)
                        pwl_path = pwl_file.name

                    # Generate netlist
                    netlist = generate_ngspice_control(circuit_path, pwl_path, duration)

                    # Run SPICE
                    t0 = time.time()
                    output = run_ngspice_and_extract(netlist, duration)
                    elapsed = time.time() - t0

                    # Save golden reference
                    golden_dir.mkdir(parents=True, exist_ok=True)
                    np.save(golden_path, output)

                    # Save metadata
                    with open(meta_path, "w") as f:
                        json.dump({
                            "circuit": str(t_config["circuit"]),
                            "circuit_hash": compute_circuit_hash(circuit_path),
                            "signal_config": sig_config,
                            "sample_rate": SAMPLE_RATE,
                            "oversample": OVERSAMPLE,
                            "internal_rate": INTERNAL_RATE,
                            "n_samples": len(output),
                            "duration_s": duration,
                            "generated_at": time.strftime("%Y-%m-%dT%H:%M:%SZ"),
                            "ngspice_time_s": elapsed,
                            "output_rms": float(np.sqrt(np.mean(output ** 2))),
                            "output_peak": float(np.max(np.abs(output))),
                        }, f, indent=2)

                    click.echo(f"    [{label}] {click.style('OK', fg='green')} "
                               f"({len(output)} samples, peak={np.max(np.abs(output)):.4f}, "
                               f"SPICE took {elapsed:.1f}s)")
                    total_generated += 1

                except Exception as e:
                    click.echo(f"    [{label}] {click.style('FAIL', fg='red')} — {e}")
                    total_failed += 1

                finally:
                    # Clean up PWL temp file
                    try:
                        os.unlink(pwl_path)
                    except:
                        pass

    click.echo(f"\n{'='*60}")
    click.echo(f"Golden generation complete: {total_generated} generated, "
               f"{total_skipped} skipped, {total_failed} failed")


if __name__ == "__main__":
    main()
