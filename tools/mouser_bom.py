#!/usr/bin/env python3
"""mouser_bom.py — Parse .pedal files and generate a Mouser BOM.

Usage:
    python tools/mouser_bom.py examples/tube_screamer.pedal
    python tools/mouser_bom.py examples/big_muff.pedal --qty 5
    python tools/mouser_bom.py examples/big_muff.pedal --qty 3 --csv bom.csv
"""

import argparse
import csv
import os
import re
import sys

# ---------------------------------------------------------------------------
# Engineering-notation helpers
# ---------------------------------------------------------------------------

SUFFIXES = {"p": 1e-12, "n": 1e-9, "u": 1e-6, "m": 1e-3, "k": 1e3, "M": 1e6}


def parse_eng(raw):
    """Parse engineering notation: '4.7k' -> 4700.0, '47n' -> 4.7e-8."""
    raw = raw.strip()
    if not raw:
        return 0.0
    if raw[-1] in SUFFIXES:
        return float(raw[:-1]) * SUFFIXES[raw[-1]]
    return float(raw)


def format_eng(value, unit):
    """Format a value with the best SI prefix + unit, e.g. 4700 Ω -> '4.7kΩ'."""
    if value == 0:
        return f"0{unit}"
    thresholds = [
        (1e6, "M"),
        (1e3, "k"),
        (1, ""),
        (1e-3, "m"),
        (1e-6, "\u00b5"),
        (1e-9, "n"),
        (1e-12, "p"),
    ]
    for thresh, prefix in thresholds:
        if abs(value) >= thresh * 0.999:
            scaled = value / thresh
            if scaled == int(scaled):
                return f"{int(scaled)}{prefix}{unit}"
            s = f"{scaled:.2f}".rstrip("0").rstrip(".")
            return f"{s}{prefix}{unit}"
    return f"{value}{unit}"


# ---------------------------------------------------------------------------
# .pedal parser
# ---------------------------------------------------------------------------

COMPONENT_RE = re.compile(r"^\s*(\w+)\s*:\s*(\w+)\(([^)]*)\)", re.MULTILINE)


def parse_pedal(path):
    """Parse a .pedal file, returning (pedal_name, components_list).

    Each component is a dict: {id, kind, arg}
    """
    with open(path) as f:
        text = f.read()

    m = re.search(r'pedal\s+"([^"]+)"', text)
    name = m.group(1) if m else os.path.basename(path).replace(".pedal", "")

    m = re.search(r"components\s*\{([^}]+)\}", text, re.DOTALL)
    if not m:
        print(f"Error: no components block found in {path}", file=sys.stderr)
        sys.exit(1)

    block = m.group(1)
    components = []
    for match in COMPONENT_RE.finditer(block):
        components.append(
            {"id": match.group(1), "kind": match.group(2), "arg": match.group(3).strip()}
        )

    return name, components


# ---------------------------------------------------------------------------
# Curated Mouser part-number lookup
# ---------------------------------------------------------------------------

# Resistors — Yageo MFR-25 1/4W metal film
RESISTORS = {
    470: ("603-MFR-25FRF52-470R", "470\u2126 1/4W Metal Film"),
    1e3: ("603-MFR-25FRF52-1K", "1k\u2126 1/4W Metal Film"),
    1.5e3: ("603-MFR-25FRF52-1K5", "1.5k\u2126 1/4W Metal Film"),
    4.7e3: ("603-MFR-25FRF52-4K7", "4.7k\u2126 1/4W Metal Film"),
    8.2e3: ("603-MFR-25FRF52-8K2", "8.2k\u2126 1/4W Metal Film"),
    10e3: ("603-MFR-25FRF52-10K", "10k\u2126 1/4W Metal Film"),
    15e3: ("603-MFR-25FRF52-15K", "15k\u2126 1/4W Metal Film"),
    22e3: ("603-MFR-25FRF52-22K", "22k\u2126 1/4W Metal Film"),
    33e3: ("603-MFR-25FRF52-33K", "33k\u2126 1/4W Metal Film"),
    47e3: ("603-MFR-25FRF52-47K", "47k\u2126 1/4W Metal Film"),
    100e3: ("603-MFR-25FRF52-100K", "100k\u2126 1/4W Metal Film"),
    150e3: ("603-MFR-25FRF52-150K", "150k\u2126 1/4W Metal Film"),
    422e3: ("603-MFR-25FRF52-422K", "422k\u2126 1/4W Metal Film"),
    470e3: ("603-MFR-25FRF52-470K", "470k\u2126 1/4W Metal Film"),
    500e3: ("603-MFR-25FRF52-500K", "500k\u2126 1/4W Metal Film"),
    560e3: ("603-MFR-25FRF52-560K", "560k\u2126 1/4W Metal Film"),
    1e6: ("603-MFR-25FRF52-1M", "1M\u2126 1/4W Metal Film"),
}

# Capacitors — Kemet ceramic (pF), WIMA MKS2 film (nF), Nichicon electrolytic (µF)
CAPACITORS = {
    100e-12: ("80-C320C101J1G5TA", "100pF Ceramic"),
    390e-12: ("80-C320C391J1G5TA", "390pF Ceramic"),
    1e-9: ("80-C320C102K5R5TA", "1nF Ceramic"),
    3.3e-9: ("80-C320C332K5R5TA", "3.3nF Ceramic"),
    4e-9: ("80-C320C402K5R5TA", "4nF Ceramic"),
    10e-9: ("5-MKS2D031001A00JSSD", "10nF WIMA Film"),
    22e-9: ("5-MKS2D032201A00JSSD", "22nF WIMA Film"),
    47e-9: ("5-MKS2D034701A00JSSD", "47nF WIMA Film"),
    68e-9: ("5-MKS2D036801A00JSSD", "68nF WIMA Film"),
    100e-9: ("5-MKS2D041001A00JSSD", "100nF WIMA Film"),
    220e-9: ("5-MKS2D042201A00JSSD", "220nF WIMA Film"),
    1e-6: ("647-UPW1H010MDD6", "1\u00b5F 50V Electrolytic"),
    2.2e-6: ("647-UPW1H2R2MDD6", "2.2\u00b5F 50V Electrolytic"),
    10e-6: ("647-UPW1H100MDD6", "10\u00b5F 50V Electrolytic"),
    100e-6: ("647-UPW1H101MED6", "100\u00b5F 50V Electrolytic"),
}

# Potentiometers — Alpha 16mm single-gang
POTS = {
    1e3: ("317-2001F-1K", "1k\u2126 Alpha 16mm Pot"),
    4.7e3: ("317-2001F-4.7K", "4.7k\u2126 Alpha 16mm Pot"),
    10e3: ("317-2001F-10K", "10k\u2126 Alpha 16mm Pot"),
    25e3: ("317-2001F-25K", "25k\u2126 Alpha 16mm Pot"),
    50e3: ("317-2001F-50K", "50k\u2126 Alpha 16mm Pot"),
    100e3: ("317-2001F-100K", "100k\u2126 Alpha 16mm Pot"),
    250e3: ("317-2001F-250K", "250k\u2126 Alpha 16mm Pot"),
    500e3: ("317-2001F-500K", "500k\u2126 Alpha 16mm Pot"),
}

# Diodes
DIODES = {
    "silicon": ("512-1N4148", "1N4148 Silicon Diode"),
    "germanium": ("583-1N34A", "1N34A Germanium Diode"),
}
LED_PART = ("604-WP7113ID", "3mm Red LED")

# Transistors
TRANSISTORS = {
    "npn": ("512-2N3904BU", "2N3904 NPN Transistor"),
    "pnp": ("512-2N3906BU", "2N3906 PNP Transistor"),
}

# Op-amps
OPAMP_PART = ("595-TL072CP", "TL072 Dual Op-Amp")


def _find_closest(table, value, tolerance=0.02):
    """Find a table entry matching value within relative tolerance."""
    if value == 0:
        return table.get(0)
    for key, entry in table.items():
        if key == 0:
            continue
        if abs(key - value) / abs(key) < tolerance:
            return entry
    return None


def lookup_part(kind, arg):
    """Return (mouser_pn, description, qty_per_unit) or None."""
    if kind == "resistor":
        entry = _find_closest(RESISTORS, parse_eng(arg))
        if entry:
            return (*entry, 1)
    elif kind == "cap":
        entry = _find_closest(CAPACITORS, parse_eng(arg))
        if entry:
            return (*entry, 1)
    elif kind == "pot":
        entry = _find_closest(POTS, parse_eng(arg))
        if entry:
            return (*entry, 1)
    elif kind == "diode_pair":
        entry = DIODES.get(arg)
        if entry:
            return (*entry, 2)
    elif kind == "diode":
        entry = DIODES.get(arg)
        if entry:
            return (*entry, 1)
    elif kind == "led":
        return (*LED_PART, 1)
    elif kind in ("npn", "pnp"):
        entry = TRANSISTORS.get(kind)
        if entry:
            return (*entry, 1)
    elif kind == "opamp":
        return (*OPAMP_PART, 1)
    return None


def mouser_search_url(kind, arg):
    """Generate a Mouser search URL as fallback for unmapped parts."""
    query = f"{kind} {arg}".replace(" ", "+")
    return f"https://www.mouser.com/Search/Refine?Keyword={query}"


# ---------------------------------------------------------------------------
# BOM generation
# ---------------------------------------------------------------------------


def _component_display(kind, arg):
    """Human-friendly component type label."""
    labels = {
        "resistor": "Resistor",
        "cap": "Capacitor",
        "pot": "Potentiometer",
        "diode_pair": f"Diode ({arg.title()}, x2)",
        "diode": f"Diode ({arg.title()})",
        "npn": "NPN Transistor",
        "pnp": "PNP Transistor",
        "opamp": "Op-Amp",
        "led": "LED",
    }
    return labels.get(kind, kind)


def _format_value(kind, arg):
    """Format the component value for display."""
    if kind in ("resistor", "pot"):
        return format_eng(parse_eng(arg), "\u2126")
    elif kind == "cap":
        return format_eng(parse_eng(arg), "F")
    elif kind in ("diode", "diode_pair"):
        return arg.title()
    elif kind in ("npn", "pnp", "opamp", "led"):
        return "\u2014"
    return arg


def build_bom(components, qty):
    """Build BOM from parsed components list.

    Returns list of dicts with keys:
        ref, kind, display, value, mouser_pn, description, qty, warning
    """
    bom = []
    for comp in components:
        kind, arg = comp["kind"], comp["arg"]
        result = lookup_part(kind, arg)
        display = _component_display(kind, arg)
        value = _format_value(kind, arg)

        if result:
            pn, desc, qty_per = result
            bom.append(
                {
                    "ref": comp["id"],
                    "kind": kind,
                    "display": display,
                    "value": value,
                    "mouser_pn": pn,
                    "description": desc,
                    "qty": qty_per * qty,
                    "warning": None,
                }
            )
        else:
            url = mouser_search_url(kind, arg)
            bom.append(
                {
                    "ref": comp["id"],
                    "kind": kind,
                    "display": display,
                    "value": value,
                    "mouser_pn": "[search]",
                    "description": f"{kind}({arg})",
                    "qty": qty,
                    "warning": f"No curated part \u2014 search: {url}",
                }
            )
    return bom


# ---------------------------------------------------------------------------
# Output
# ---------------------------------------------------------------------------


def print_bom_table(name, bom, qty):
    """Print a formatted BOM table to stdout."""
    line = "\u2550" * 70
    thin = "\u2500" * 70

    print(f"\n{line}")
    label = f"  BOM: {name}" + (f" (x{qty})" if qty > 1 else "")
    print(label)
    print(line)

    print(
        f"  {'Ref':<8} {'Component':<20} {'Value':<10} {'Mouser P/N':<24} {'Qty':>3}"
    )
    print(
        f"  {'\u2500' * 6:<8} {'\u2500' * 9:<20} {'\u2500' * 5:<10} {'\u2500' * 10:<24} {'\u2500' * 3:>3}"
    )

    total_parts = 0
    for item in bom:
        print(
            f"  {item['ref']:<8} {item['display']:<20} {item['value']:<10} "
            f"{item['mouser_pn']:<24} {item['qty']:>3}"
        )
        total_parts += item["qty"]

    print(thin)
    print(f"  Total: {len(bom)} line items, {total_parts} parts")

    warnings = [item for item in bom if item["warning"]]
    if warnings:
        print()
        for item in warnings:
            print(f"  Warning: {item['ref']}: {item['warning']}")

    print(line)


def write_csv(bom, path):
    """Write Mouser-compatible BOM CSV."""
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["Mouser Part Number", "Quantity", "Reference", "Description"])
        for item in bom:
            if item["mouser_pn"] == "[search]":
                continue
            writer.writerow(
                [item["mouser_pn"], item["qty"], item["ref"], item["description"]]
            )
    print(f"\n  CSV written to: {path}")


def mouser_cart(bom, api_key):
    """Create a Mouser cart via API. Returns cart URL or None."""
    try:
        import requests
    except ImportError:
        print(
            "  Note: 'requests' not installed — skipping Mouser API cart creation.",
            file=sys.stderr,
        )
        return None

    items = []
    for item in bom:
        if item["mouser_pn"] == "[search]":
            continue
        items.append(
            {"MouserPartNumber": item["mouser_pn"], "Quantity": item["qty"]}
        )

    if not items:
        return None

    url = f"https://api.mouser.com/api/v1/cart?apiKey={api_key}"
    try:
        resp = requests.post(url, json={"CartItems": items}, timeout=15)
        resp.raise_for_status()
        data = resp.json()
        cart_key = data.get("CartKey", "")
        if cart_key:
            return f"https://www.mouser.com/Cart/AddToOrder?CartKey={cart_key}"
    except Exception as e:
        print(f"  Mouser API error: {e}", file=sys.stderr)
    return None


def print_cart_info(bom, api_key):
    """Print Mouser cart URL or BOM upload instructions."""
    if api_key:
        url = mouser_cart(bom, api_key)
        if url:
            print(f"\n  Mouser Cart URL:\n  {url}")
            return

    print(f"\n  Upload BOM at: https://www.mouser.com/Bom/Upload")
    print(f"  (Use --csv FILE to export, or set MOUSER_API_KEY for direct cart)")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main():
    parser = argparse.ArgumentParser(
        description="Generate a Mouser BOM from a .pedal file."
    )
    parser.add_argument("pedal_file", help="Path to .pedal file")
    parser.add_argument(
        "--qty", type=int, default=1, help="Number of pedals to build (default: 1)"
    )
    parser.add_argument(
        "--csv", metavar="FILE", dest="csv_file", help="Export Mouser-compatible BOM CSV"
    )
    args = parser.parse_args()

    if args.qty < 1:
        print("Error: --qty must be >= 1", file=sys.stderr)
        sys.exit(1)

    name, components = parse_pedal(args.pedal_file)
    bom = build_bom(components, args.qty)

    print_bom_table(name, bom, args.qty)

    if args.csv_file:
        write_csv(bom, args.csv_file)

    api_key = os.environ.get("MOUSER_API_KEY")
    print_cart_info(bom, api_key)
    print()


if __name__ == "__main__":
    main()
