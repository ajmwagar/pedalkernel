#!/usr/bin/env python3
"""
Convert SPICE .mod files (multi-line format) into PedalKernel .model format.

Usage:
    python3 convert_mod.py                    # Process all .mod files in current directory
    python3 convert_mod.py new_bjts.mod       # Process a specific file
    python3 convert_mod.py --dry-run          # Show what would be done without writing

The script:
  1. Joins continuation lines (lines starting with +) into single lines
  2. Strips Q/D/J prefixes from model names
  3. Removes unsupported SPICE params (XTB, KF, AF, IRB, RBM, XTF, ITF, PTF, etc.)
  4. Deduplicates against existing .model files
  5. Appends new entries or creates new .model files as needed
"""

import re
import sys
import os
from pathlib import Path


# ─── Supported SPICE parameters per component type ─────────────────────────

BJT_KNOWN = [
    'IS', 'BF', 'BR', 'NF', 'NR', 'VAF', 'VA', 'VAR', 'VB',
    'IKF', 'JBF', 'IKR', 'JBR', 'ISE', 'NE', 'ISC', 'NC',
    'RB', 'RE', 'RC', 'CJE', 'CEB', 'VJE', 'PE', 'MJE', 'ME',
    'CJC', 'CCB', 'VJC', 'PC', 'MJC', 'MC', 'TF', 'TR',
]

JFET_KNOWN = ['VTO', 'BETA', 'LAMBDA', 'IS', 'N', 'RD', 'RS', 'CGS', 'CGD', 'PB']

DIODE_KNOWN = ['IS', 'RS', 'N', 'TT', 'CJO', 'VJ', 'M', 'BV', 'IBV', 'EG', 'XTI']


# ─── Core parsing ──────────────────────────────────────────────────────────

def join_continuation_lines(text):
    """Join SPICE continuation lines (starting with +) into single lines."""
    lines = text.split('\n')
    result = []
    for line in lines:
        stripped = line.strip()
        if stripped.startswith('+'):
            if result:
                result[-1] = result[-1].rstrip() + ' ' + stripped[1:].strip()
            continue
        result.append(line)
    return '\n'.join(result)


def extract_models(text, type_filter=None):
    """Extract .MODEL lines from SPICE text.

    Returns list of (clean_name, spice_type, param_dict).
    Deduplicates by name (first occurrence wins).
    """
    text = join_continuation_lines(text)
    models = []
    seen = set()

    for line in text.split('\n'):
        stripped = line.strip()
        if not stripped.upper().startswith('.MODEL'):
            continue

        m = re.match(
            r'\.MODEL\s+(\S+)\s+(NPN|PNP|NJF|PJF|D)\s*\(([^)]*)\)',
            stripped, re.IGNORECASE,
        )
        if not m:
            continue

        name = m.group(1)
        mtype = m.group(2).upper()
        params_str = m.group(3).strip()

        if type_filter and mtype not in type_filter:
            continue

        # Strip conventional SPICE prefix (Q for BJT, D for diode, J for JFET)
        clean_name = name
        if name.startswith('Q') and len(name) > 1:
            clean_name = name[1:]
        elif name.startswith('D') and len(name) > 1 and (name[1:2].isdigit() or name[1:3] == 'LE'):
            clean_name = name[1:]
        elif name.startswith('J') and len(name) > 1 and name[1:2].isdigit():
            clean_name = name[1:]

        # Skip template/default models (ending with ~)
        if clean_name.endswith('~'):
            continue

        clean_name = clean_name.upper()

        # Skip empty param blocks
        if not params_str.strip():
            continue

        # Deduplicate
        if clean_name in seen:
            continue
        seen.add(clean_name)

        # Parse key=value pairs
        param_dict = {}
        for pair in re.findall(r'(\w+)\s*=\s*([^\s,)]+)', params_str):
            param_dict[pair[0].upper()] = pair[1]

        models.append((clean_name, mtype, param_dict))

    return models


def get_existing_names(model_file):
    """Get set of model names already in a .model file."""
    names = set()
    if not os.path.exists(model_file):
        return names
    with open(model_file) as f:
        for line in f:
            m = re.match(r'\.MODEL\s+(\S+)', line.strip())
            if m:
                names.add(m.group(1).upper())
    return names


# ─── Formatters ────────────────────────────────────────────────────────────

def fmt_bjt(name, mtype, params):
    parts = [f'{k}={v}' for k, v in params.items() if k in BJT_KNOWN]
    return f'.MODEL {name} {mtype}({" ".join(parts)})'


def fmt_jfet(name, mtype, params):
    parts = [f'{k}={v}' for k, v in params.items() if k in JFET_KNOWN]
    return f'.MODEL {name} {mtype} {" ".join(parts)}'


def fmt_diode(name, params):
    parts = [f'{k}={v}' for k, v in params.items() if k in DIODE_KNOWN]
    return f'.MODEL {name} D({" ".join(parts)})'


# ─── File classification ──────────────────────────────────────────────────

def classify_mod_file(filepath):
    """Guess the component type from file name and contents."""
    name = Path(filepath).stem.lower()
    if 'npn' in name:
        return 'bjt', {'NPN'}
    elif 'pnp' in name:
        return 'bjt', {'PNP'}
    elif 'jfet' in name or 'njf' in name or 'pjf' in name:
        return 'jfet', {'NJF', 'PJF'}
    elif 'led' in name:
        return 'led', {'D'}
    elif 'zener' in name:
        return 'zener', {'D'}
    elif 'schottky' in name:
        return 'schottky', {'D'}
    elif 'diode' in name:
        return 'diode', {'D'}
    elif 'mosfet' in name or 'nmos' in name or 'pmos' in name:
        return 'mosfet', {'NMOS', 'PMOS'}
    else:
        # Try to detect from content
        with open(filepath) as f:
            content = f.read(500).upper()
        if 'NPN' in content or 'PNP' in content:
            return 'bjt', {'NPN', 'PNP'}
        elif 'NJF' in content or 'PJF' in content:
            return 'jfet', {'NJF', 'PJF'}
        else:
            return 'diode', {'D'}


# ─── Target .model file mapping ───────────────────────────────────────────

TARGET_FILES = {
    'bjt': 'transistors.model',
    'jfet': 'jfets.model',
    'diode': 'diodes.model',
    'led': 'leds.model',
    'schottky': 'schottky.model',
    'zener': 'zeners.model',
}

FORMATTERS = {
    'bjt': fmt_bjt,
    'jfet': fmt_jfet,
    'diode': fmt_diode,
    'led': fmt_diode,
    'schottky': fmt_diode,
    'zener': fmt_diode,
}


# ─── Main ─────────────────────────────────────────────────────────────────

def process_mod_file(filepath, dry_run=False):
    """Process a single .mod file and append new models to the target .model file."""
    comp_type, type_filter = classify_mod_file(filepath)
    target = TARGET_FILES.get(comp_type)
    formatter_key = comp_type

    if not target:
        print(f"  SKIP: Unknown component type for {filepath}")
        return 0

    with open(filepath) as f:
        text = f.read()

    models = extract_models(text, type_filter)
    if not models:
        print(f"  SKIP: No models found in {filepath}")
        return 0

    existing = get_existing_names(target)
    new_models = [(n, t, p) for n, t, p in models if n not in existing]

    if not new_models:
        print(f"  OK: All {len(models)} models from {filepath} already in {target}")
        return 0

    formatter = FORMATTERS[formatter_key]
    source_name = Path(filepath).name

    lines = []
    lines.append(f'\n* ── From {source_name} ({len(new_models)} models) ' + '─' * 40 + '\n')
    for name, mtype, params in sorted(new_models, key=lambda x: x[0]):
        if comp_type in ('diode', 'led', 'schottky', 'zener'):
            lines.append(fmt_diode(name, params))
        elif comp_type == 'jfet':
            lines.append(fmt_jfet(name, mtype, params))
        else:
            lines.append(fmt_bjt(name, mtype, params))

    if dry_run:
        print(f"  DRY RUN: Would append {len(new_models)} models to {target}")
        for l in lines[:5]:
            print(f"    {l}")
        if len(lines) > 5:
            print(f"    ... and {len(lines) - 5} more")
    else:
        with open(target, 'a') as f:
            f.write('\n'.join(lines) + '\n')
        print(f"  DONE: Appended {len(new_models)} new models to {target} (from {source_name})")

    return len(new_models)


def main():
    args = sys.argv[1:]
    dry_run = '--dry-run' in args
    args = [a for a in args if a != '--dry-run']

    if not args:
        # Process all .mod files in current directory
        mod_files = sorted(Path('.').glob('*.mod'))
    else:
        mod_files = [Path(a) for a in args]

    if not mod_files:
        print("No .mod files found.")
        return

    total = 0
    for f in mod_files:
        print(f"Processing {f.name}...")
        total += process_mod_file(str(f), dry_run=dry_run)

    print(f"\nTotal: {total} new models {'would be' if dry_run else ''} added.")


if __name__ == '__main__':
    main()
