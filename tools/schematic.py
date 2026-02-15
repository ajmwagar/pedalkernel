#!/usr/bin/env python3
"""schematic.py — Render .pedal circuits as EE-style schematics (PNG/PDF/SVG).

Usage:
    python tools/schematic.py examples/tube_screamer.pedal -o tube_screamer.png
    python tools/schematic.py examples/klon_centaur.pedal -o klon.pdf
    python tools/schematic.py examples/big_muff.pedal -o muff.svg

    # All pedals at once
    for f in examples/*.pedal; do
      python tools/schematic.py "$f" -o "${f%.pedal}.png"
    done
"""

import argparse
import os
import re
import sys
from collections import defaultdict
from datetime import date

try:
    import schemdraw
    import schemdraw.elements as elm
except ImportError:
    print(
        "Error: schemdraw not installed. Run: pip install -r tools/requirements.txt",
        file=sys.stderr,
    )
    sys.exit(1)

# Ensure matplotlib is available for PNG/PDF output
try:
    import matplotlib
    matplotlib.use("Agg")
    matplotlib.rcParams["font.family"] = "sans-serif"
    matplotlib.rcParams["font.sans-serif"] = [
        "Liberation Sans", "DejaVu Sans", "Source Sans 3", "Noto Sans", "sans-serif"
    ]
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False

# Pick best available font for schemdraw
_FONT_PREFS = ["Liberation Sans", "DejaVu Sans", "Source Sans 3", "Noto Sans"]
FONT_FAMILY = "sans-serif"
if HAS_MATPLOTLIB:
    import matplotlib.font_manager as _fm
    _available = {f.name for f in _fm.fontManager.ttflist}
    for _pref in _FONT_PREFS:
        if _pref in _available:
            FONT_FAMILY = _pref
            break

# ---------------------------------------------------------------------------
# Engineering-notation helpers (from mouser_bom.py)
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
    """Format a value with the best SI prefix + unit, e.g. 4700 -> '4.7k\u03a9'."""
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


def format_value(kind, arg):
    """Format component value for label."""
    if kind in ("resistor", "pot"):
        return format_eng(parse_eng(arg), "\u03a9")
    elif kind == "cap":
        return format_eng(parse_eng(arg), "F")
    elif kind in ("diode", "diode_pair"):
        return arg.title()
    elif kind in ("npn", "pnp", "led"):
        return ""
    elif kind == "opamp":
        # Op-amp type: tl072, jrc4558, lm308, ca3080, etc.
        if arg:
            type_map = {
                "tl072": "TL072",
                "tl082": "TL082",
                "jrc4558": "JRC4558D",
                "4558": "JRC4558D",
                "rc4558": "RC4558",
                "lm308": "LM308N",
                "lm741": "LM741",
                "ne5532": "NE5532",
                "ca3080": "CA3080",
                "op07": "OP07",
            }
            return type_map.get(arg.lower(), arg.upper())
        return ""
    elif kind in ("njfet", "pjfet"):
        # JFET type: j201, 2n5457, 2n5460
        return arg.upper() if arg else ""
    elif kind == "photocoupler":
        # Photocoupler type: nsl32, vtl5c1, vtl5c3
        return arg.upper() if arg else "VTL"
    elif kind == "lfo":
        # LFO args: waveform, timing_r, timing_c
        parts = [p.strip() for p in arg.split(",")]
        if len(parts) >= 3:
            waveform = parts[0].title()
            timing_r = format_eng(parse_eng(parts[1]), "\u03a9")
            timing_c = format_eng(parse_eng(parts[2]), "F")
            return f"{waveform}\nR={timing_r}, C={timing_c}"
        return arg
    return arg


# ---------------------------------------------------------------------------
# .pedal parser
# ---------------------------------------------------------------------------

COMPONENT_RE = re.compile(r"^\s*(\w+)\s*:\s*(\w+)\(([^)]*)\)", re.MULTILINE)
NET_RE = re.compile(r"^\s*([\w.]+)\s*->\s*(.+)$", re.MULTILINE)


def parse_pedal(path):
    """Parse a .pedal file, returning (name, components, nets).

    components: list of {id, kind, arg}
    nets: list of (src_pin, [dst_pin, ...])
    """
    with open(path) as f:
        text = f.read()

    m = re.search(r'pedal\s+"([^"]+)"', text)
    name = m.group(1) if m else os.path.basename(path).replace(".pedal", "")

    # Parse components
    m = re.search(r"components\s*\{([^}]+)\}", text, re.DOTALL)
    if not m:
        print(f"Error: no components block found in {path}", file=sys.stderr)
        sys.exit(1)
    components = []
    for match in COMPONENT_RE.finditer(m.group(1)):
        components.append(
            {"id": match.group(1), "kind": match.group(2), "arg": match.group(3).strip()}
        )

    # Parse nets
    m = re.search(r"nets\s*\{([^}]+)\}", text, re.DOTALL)
    if not m:
        print(f"Error: no nets block found in {path}", file=sys.stderr)
        sys.exit(1)
    nets = []
    for match in NET_RE.finditer(m.group(1)):
        src = match.group(1).strip()
        dsts = [d.strip() for d in match.group(2).split(",")]
        # Strip inline comments from destination pins
        cleaned = []
        for d in dsts:
            d = d.split("#")[0].strip()
            if d:
                cleaned.append(d)
        nets.append((src, cleaned))

    return name, components, nets


# ---------------------------------------------------------------------------
# .pedalhw parser
# ---------------------------------------------------------------------------

HWPROP_RE = re.compile(
    r'(?:part\("([^"]+)"\)|vce_max\([\d.]+\)|voltage_rating\([\d.]+\)'
    r"|supply_max\([\d.]+\)|breakdown\([\d.]+\)|power_rating\([\d.]+\))"
)
HWLINE_RE = re.compile(r"^\s*(\w+)\s*:\s*(.+)$", re.MULTILINE)


def parse_pedalhw(path):
    """Parse a .pedalhw file, returning {comp_id: part_name} for labeling.

    Only extracts part("...") values — the voltage/power specs are handled
    by the Rust `hw` module.
    """
    parts = {}
    if not os.path.exists(path):
        return parts
    with open(path) as f:
        text = f.read()
    for match in HWLINE_RE.finditer(text):
        comp_id = match.group(1)
        props = match.group(2)
        m = re.search(r'part\("([^"]+)"\)', props)
        if m:
            parts[comp_id] = m.group(1)
    return parts


def find_pedalhw(pedal_path):
    """Auto-discover the companion .pedalhw file for a .pedal file."""
    base = os.path.splitext(pedal_path)[0]
    hw_path = base + ".pedalhw"
    return hw_path


# ---------------------------------------------------------------------------
# Union-Find for junction merging
# ---------------------------------------------------------------------------


class UnionFind:
    def __init__(self):
        self.parent = {}
        self.rank = {}
        self.label = {}  # canonical label for each set

    def make(self, x, lbl=None):
        if x not in self.parent:
            self.parent[x] = x
            self.rank[x] = 0
            self.label[x] = lbl or x

    def find(self, x):
        if self.parent[x] != x:
            self.parent[x] = self.find(self.parent[x])
        return self.parent[x]

    def union(self, a, b):
        ra, rb = self.find(a), self.find(b)
        if ra == rb:
            return
        # Prefer special node labels
        special = {"in", "out", "gnd", "vcc"}
        la, lb = self.label[ra], self.label[rb]
        keep_label = la if la in special else lb if lb in special else la

        if self.rank[ra] < self.rank[rb]:
            ra, rb = rb, ra
        self.parent[rb] = ra
        if self.rank[ra] == self.rank[rb]:
            self.rank[ra] += 1
        self.label[ra] = keep_label

    def get_label(self, x):
        return self.label[self.find(x)]


# ---------------------------------------------------------------------------
# Circuit graph construction
# ---------------------------------------------------------------------------

# Pin definitions per component kind
PINS = {
    "resistor": ["a", "b"],
    "cap": ["a", "b"],
    "pot": ["a", "b"],
    "diode": ["a", "b"],
    "diode_pair": ["a", "b"],
    "led": ["a", "b"],
    "npn": ["base", "collector", "emitter"],
    "pnp": ["base", "collector", "emitter"],
    "opamp": ["pos", "neg", "out"],
    "njfet": ["gate", "drain", "source"],
    "pjfet": ["gate", "drain", "source"],
    "photocoupler": ["led", "ldr_a", "ldr_b"],
    "lfo": ["out", "rate"],
}

TWO_TERMINAL = {"resistor", "cap", "pot", "diode", "diode_pair", "led"}


def build_graph(components, nets):
    """Build a circuit graph using union-find.

    Returns:
        uf: UnionFind with all pins merged into junctions
        comp_map: {comp_id: component_dict}
        edges: list of (comp_id, junction_a, junction_b) for two-terminal
        multi_edges: list of (comp_id, {pin_name: junction}) for multi-terminal
    """
    uf = UnionFind()
    comp_map = {}

    # Create pin nodes for every component
    for comp in components:
        comp_map[comp["id"]] = comp
        pins = PINS.get(comp["kind"], ["a", "b"])
        for pin in pins:
            pin_id = f"{comp['id']}.{pin}"
            uf.make(pin_id, pin_id)

    # Create special nodes
    for special in ["in", "out", "gnd", "vcc"]:
        uf.make(special, special)

    # Apply net connections
    for src, dsts in nets:
        uf.make(src, src)
        for dst in dsts:
            uf.make(dst, dst)
            uf.union(src, dst)

    # Build edges for two-terminal components
    edges = []
    for comp in components:
        if comp["kind"] in TWO_TERMINAL:
            ja = uf.get_label(f"{comp['id']}.a")
            jb = uf.get_label(f"{comp['id']}.b")
            edges.append((comp["id"], ja, jb))

    # Build multi-terminal component info
    multi_edges = []
    for comp in components:
        if comp["kind"] not in TWO_TERMINAL:
            pins = PINS.get(comp["kind"], [])
            pin_junctions = {}
            for pin in pins:
                pin_junctions[pin] = uf.get_label(f"{comp['id']}.{pin}")
            multi_edges.append((comp["id"], pin_junctions))

    return uf, comp_map, edges, multi_edges


# ---------------------------------------------------------------------------
# Auto-layout: BFS spine + classification
# ---------------------------------------------------------------------------


def build_adjacency(edges, multi_edges, comp_map):
    """Build junction adjacency graph.

    Returns adj: {junction: [(junction, comp_id), ...]}
    """
    adj = defaultdict(list)
    for comp_id, ja, jb in edges:
        adj[ja].append((jb, comp_id))
        adj[jb].append((ja, comp_id))

    # For multi-terminal, add edges between all pin junctions
    for comp_id, pin_junctions in multi_edges:
        kind = comp_map[comp_id]["kind"]
        junctions = list(pin_junctions.values())
        if kind in ("npn", "pnp"):
            # Main path: collector <-> emitter, branch: base
            c, e, b = pin_junctions.get("collector"), pin_junctions.get("emitter"), pin_junctions.get("base")
            if c and e:
                adj[c].append((e, comp_id))
                adj[e].append((c, comp_id))
            if b and c:
                adj[b].append((c, comp_id))
                adj[c].append((b, comp_id))
            if b and e:
                adj[b].append((e, comp_id))
                adj[e].append((b, comp_id))
        elif kind == "opamp":
            p, n, o = pin_junctions.get("pos"), pin_junctions.get("neg"), pin_junctions.get("out")
            if p and o:
                adj[p].append((o, comp_id))
                adj[o].append((p, comp_id))
            if n and o:
                adj[n].append((o, comp_id))
                adj[o].append((n, comp_id))
            if p and n:
                adj[p].append((n, comp_id))
                adj[n].append((p, comp_id))

    return adj


def bfs_path(adj, start, end):
    """BFS shortest path from start to end junction. Returns list of (junction, comp_id) pairs."""
    if start == end:
        return [(start, None)]
    from collections import deque
    visited = {start}
    queue = deque([(start, [(start, None)])])

    while queue:
        node, path = queue.popleft()
        for neighbor, comp_id in adj.get(node, []):
            if neighbor in visited:
                continue
            visited.add(neighbor)
            new_path = path + [(neighbor, comp_id)]
            if neighbor == end:
                return new_path
            queue.append((neighbor, new_path))
    return None


def classify_components(edges, multi_edges, comp_map, adj):
    """Classify each component into layout roles.

    Returns:
        spine_comps: ordered list of (comp_id, junction_from, junction_to)
        branches: list of (comp_id, spine_junction, target_junction, direction)
        parallel_comps: list of (comp_id, junction_a, junction_b)
        feedback_comps: list of (comp_id, junction_a, junction_b)
    """
    path = bfs_path(adj, "in", "out")

    if not path:
        # No path found — treat all components as spine in order
        spine_comps = [(cid, ja, jb) for cid, ja, jb in edges]
        return spine_comps, [], [], []

    spine_junctions = [j for j, _ in path]
    spine_set = set(spine_junctions)
    spine_comp_ids = {cid for _, cid in path if cid is not None}

    spine_order = {j: i for i, j in enumerate(spine_junctions)}

    spine_comps = []
    branches = []
    parallel_comps = []
    feedback_comps = []

    # Classify two-terminal components
    for comp_id, ja, jb in edges:
        a_on_spine = ja in spine_set
        b_on_spine = jb in spine_set

        if comp_id in spine_comp_ids:
            spine_comps.append((comp_id, ja, jb))
        elif a_on_spine and b_on_spine:
            # Both on spine but not the spine edge itself
            ia, ib = spine_order[ja], spine_order[jb]
            if ia > ib:
                ia, ib = ib, ia
                ja, jb = jb, ja
            if ib - ia == 1:
                # Adjacent spine junctions — parallel to a spine component
                parallel_comps.append((comp_id, ja, jb))
            elif ia < ib:
                # Feedback (later back to earlier)
                feedback_comps.append((comp_id, ja, jb))
            else:
                parallel_comps.append((comp_id, ja, jb))
        elif a_on_spine and jb == "gnd":
            branches.append((comp_id, ja, jb, "gnd"))
        elif b_on_spine and ja == "gnd":
            branches.append((comp_id, jb, ja, "gnd"))
        elif a_on_spine and jb == "vcc":
            branches.append((comp_id, ja, jb, "vcc"))
        elif b_on_spine and ja == "vcc":
            branches.append((comp_id, jb, ja, "vcc"))
        elif a_on_spine:
            # Branch off spine to somewhere else
            branches.append((comp_id, ja, jb, "branch"))
        elif b_on_spine:
            branches.append((comp_id, jb, ja, "branch"))
        elif ja == "gnd" or jb == "gnd":
            # Component connected to ground from off-spine junction
            spine_j = jb if ja == "gnd" else ja
            branches.append((comp_id, spine_j, "gnd", "gnd"))
        elif ja == "vcc" or jb == "vcc":
            spine_j = jb if ja == "vcc" else ja
            branches.append((comp_id, spine_j, "vcc", "vcc"))
        else:
            # Neither end on spine — try to place as branch
            branches.append((comp_id, ja, jb, "branch"))

    # Classify multi-terminal components
    for comp_id, pin_junctions in multi_edges:
        kind = comp_map[comp_id]["kind"]
        if comp_id in spine_comp_ids:
            # On spine — determine through-pins
            if kind in ("npn", "pnp"):
                c = pin_junctions.get("collector")
                e = pin_junctions.get("emitter")
                spine_comps.append((comp_id, c, e))
            elif kind == "opamp":
                # Input is pos or neg, output is out
                p = pin_junctions.get("pos")
                n = pin_junctions.get("neg")
                o = pin_junctions.get("out")
                # Pick whichever input is on the spine path
                if p in spine_set and o in spine_set:
                    spine_comps.append((comp_id, p, o))
                elif n in spine_set and o in spine_set:
                    spine_comps.append((comp_id, n, o))
                else:
                    spine_comps.append((comp_id, n or p, o))

    # Sort spine components by path order
    def spine_sort_key(item):
        _, ja, jb = item
        ia = spine_order.get(ja, 999)
        ib = spine_order.get(jb, 999)
        return min(ia, ib)

    spine_comps.sort(key=spine_sort_key)

    return spine_comps, branches, parallel_comps, feedback_comps


# ---------------------------------------------------------------------------
# Schemdraw rendering
# ---------------------------------------------------------------------------

SCHEMDRAW_MAP = {
    "resistor": elm.Resistor,
    "cap": elm.Capacitor,
    "pot": elm.Potentiometer,
    "diode": elm.Diode,
    "diode_pair": elm.Diode,
    "led": elm.LED,
    "npn": elm.BjtNpn,
    "pnp": elm.BjtPnp,
    "opamp": elm.Opamp,
    "njfet": elm.JFetN,
    "pjfet": elm.JFetP,
    # LFO and photocoupler use Box elements with custom labels
}


def make_label(comp_id, kind, arg, part_name=None):
    """Create a label: ID on top, part name and/or value below.

    If a .pedalhw file provides a part name (e.g. "AC128"), it's shown
    instead of the generic value for active components, or alongside the
    value for passives.
    """
    val = format_value(kind, arg)
    if kind == "diode_pair":
        val = f"{val} \u00d72"
    if part_name:
        if kind in ("npn", "pnp", "opamp"):
            # Active: show part name instead of dash
            return f"{comp_id}\n{part_name}"
        elif kind in ("diode", "diode_pair"):
            return f"{comp_id}\n{part_name}" + (f" \u00d72" if kind == "diode_pair" else "")
        elif val:
            # Passive: show both
            return f"{comp_id}\n{part_name}\n{val}"
        else:
            return f"{comp_id}\n{part_name}"
    if val:
        return f"{comp_id}\n{val}"
    return comp_id


def get_spacing(n_components):
    """Adaptive spacing based on component count."""
    if n_components <= 5:
        return 2.0
    elif n_components <= 10:
        return 1.5
    else:
        return 1.2


def _draw_element(d, kind, label, direction="right", pos=None, label_loc=None):
    """Draw a schemdraw element with correct handling for all component types.

    For two-terminal components, returns (element, start_point, end_point).
    For BJT/opamp/JFET, returns (element, pin_dict) where pin_dict maps pin names to points.
    """
    elem_cls = SCHEMDRAW_MAP.get(kind, elm.Resistor)

    if kind in ("npn", "pnp"):
        e = elem_cls()
        if pos is not None:
            e = e.at(pos)
        e = e.label(label, loc=label_loc or "right")
        el = d.add(e)
        pins = {
            "base": el.absanchors["base"],
            "collector": el.absanchors["collector"],
            "emitter": el.absanchors["emitter"],
        }
        return el, pins
    elif kind in ("njfet", "pjfet"):
        e = elem_cls()
        if pos is not None:
            e = e.at(pos)
        e = e.label(label, loc=label_loc or "right")
        el = d.add(e)
        pins = {
            "gate": el.absanchors["gate"],
            "drain": el.absanchors["drain"],
            "source": el.absanchors["source"],
        }
        return el, pins
    elif kind == "opamp":
        e = elem_cls()
        if pos is not None:
            e = e.at(pos)
        e = e.label(label, loc=label_loc or "center")
        el = d.add(e)
        pins = {
            "in1": el.absanchors["in1"],
            "in2": el.absanchors["in2"],
            "out": el.absanchors["out"],
        }
        return el, pins
    elif kind == "lfo":
        # Draw LFO as a labeled box with timing component info
        e = elm.Ic(pins=[elm.IcPin("out", side="right"),
                        elm.IcPin("rate", side="left")],
                   size=(2, 1.5))
        if pos is not None:
            e = e.at(pos)
        e = e.label(label, loc="center", fontsize=8)
        el = d.add(e)
        pins = {
            "out": el.absanchors["out"],
            "rate": el.absanchors["rate"],
        }
        return el, pins
    elif kind == "photocoupler":
        # Draw photocoupler as a box with LED and LDR pins
        e = elm.Ic(pins=[elm.IcPin("led", side="left"),
                        elm.IcPin("ldr_a", side="right", anchorname="ldr_a"),
                        elm.IcPin("ldr_b", side="right", anchorname="ldr_b")],
                   size=(2, 1.5))
        if pos is not None:
            e = e.at(pos)
        e = e.label(label, loc="center", fontsize=8)
        el = d.add(e)
        pins = {
            "led": el.absanchors["led"],
            "ldr_a": el.absanchors["ldr_a"],
            "ldr_b": el.absanchors["ldr_b"],
        }
        return el, pins
    else:
        e = elem_cls()
        if pos is not None:
            e = e.at(pos)
        if direction == "right":
            e = e.right()
        elif direction == "down":
            e = e.down()
        elif direction == "up":
            e = e.up()
        elif direction == "left":
            e = e.left()
        if label_loc is None:
            label_loc = "top" if direction == "right" else "right"
        # Offset labels on vertical elements to avoid overlapping lines
        lbl_ofst = 0.15 if direction in ("down", "up") else None
        e = e.label(label, loc=label_loc, ofst=lbl_ofst)
        el = d.add(e)
        return el, {"start": el.start, "end": el.end}


def _draw_power_supply(d, origin, unit):
    """Draw the standard 9V power supply island with bias voltage divider.

    Standard guitar pedal PSU:
      9V DC in -> D_prot -> C_filter -> 9V rail
      9V rail -> R_b1 -> Vbias (4.5V) -> R_b2 -> GND
      Vbias -> C_bias -> GND
    """
    x0, y0 = origin
    fs = 9
    su = unit * 0.7  # sub-unit for compact layout

    # Section label
    d.add(elm.Label().at((x0, y0 + su * 0.6))
          .label("POWER SUPPLY", fontsize=11, halign="left"))

    # 9V DC jack
    d.add(elm.Dot(open=True).at((x0, y0)).label("9V DC", loc="left", fontsize=fs))

    # Reverse polarity protection diode
    dp = d.add(elm.Diode().right().at((x0, y0)).length(su)
               .label("D_prot", loc="top", fontsize=fs))

    # Junction dot
    j1 = dp.end
    d.add(elm.Dot().at(j1))

    # Filter cap to ground (down from junction)
    cf = d.add(elm.Capacitor().down().at(j1).length(su)
               .label("C_filt\n100\u00b5F", loc="right", ofst=0.15, fontsize=fs))
    d.add(elm.Ground().at(cf.end))

    # Continue right — 9V rail tap, then bias divider
    j2_vdd = (j1[0] + su * 0.7, y0)
    d.add(elm.Line().at(j1).to(j2_vdd))
    d.add(elm.Dot().at(j2_vdd))
    d.add(elm.Vdd().at(j2_vdd).label("9V", fontsize=fs))

    # Bias divider starts further right
    j2 = (j2_vdd[0] + su * 0.5, y0)
    d.add(elm.Line().at(j2_vdd).to(j2))
    d.add(elm.Dot().at(j2))

    # R_b1 down
    r1 = d.add(elm.Resistor().down().at(j2).length(su)
               .label("R_b1\n100k\u03a9", loc="right", ofst=0.15, fontsize=fs))
    bias_pt = r1.end
    d.add(elm.Dot().at(bias_pt))

    # 4.5V label
    d.add(elm.Label().at((bias_pt[0] - 0.5, bias_pt[1]))
          .label("4.5V", fontsize=fs, halign="right"))

    # R_b2 down to ground
    r2 = d.add(elm.Resistor().down().at(bias_pt).length(su)
               .label("R_b2\n100k\u03a9", loc="right", ofst=0.15, fontsize=fs))
    d.add(elm.Ground().at(r2.end))

    # C_bias: right from bias point, then down to ground
    cb_pt = (bias_pt[0] + su * 0.9, bias_pt[1])
    d.add(elm.Line().at(bias_pt).to(cb_pt))
    d.add(elm.Dot().at(cb_pt))
    cb = d.add(elm.Capacitor().down().at(cb_pt).length(su)
               .label("C_bias\n10\u00b5F", loc="right", ofst=0.15, fontsize=fs))
    d.add(elm.Ground().at(cb.end))


def _draw_bypass_switching(d, origin, unit):
    """Draw the standard true-bypass DPDT footswitch with LED indicator."""
    x0, y0 = origin
    fs = 9
    su = unit * 0.7

    # Section label
    d.add(elm.Label().at((x0, y0 + su * 0.6))
          .label("TRUE BYPASS", fontsize=11, halign="left"))

    # Input jack
    d.add(elm.Dot(open=True).at((x0, y0)).label("IN Jack", loc="left", fontsize=fs))

    # DPDT switch box
    sw_x = x0 + su
    sw_w = su * 1.1
    sw_h = su * 1.4
    sw_cx = sw_x + sw_w / 2
    sw_top = y0 + sw_h * 0.3
    sw_bot = y0 - sw_h * 0.7

    d.add(elm.Line().at((x0, y0)).to((sw_x, y0)))

    # Box
    d.add(elm.Line().at((sw_x, sw_top)).to((sw_x + sw_w, sw_top)))
    d.add(elm.Line().at((sw_x + sw_w, sw_top)).to((sw_x + sw_w, sw_bot)))
    d.add(elm.Line().at((sw_x + sw_w, sw_bot)).to((sw_x, sw_bot)))
    d.add(elm.Line().at((sw_x, sw_bot)).to((sw_x, sw_top)))
    d.add(elm.Label().at((sw_cx, (sw_top + sw_bot) / 2))
          .label("DPDT\nFootswitch", fontsize=fs, halign="center"))

    # Right side — output jack
    out_x = sw_x + sw_w
    d.add(elm.Line().at((out_x, y0)).to((out_x + su * 0.5, y0)))
    d.add(elm.Dot(open=True).at((out_x + su * 0.5, y0))
          .label("OUT Jack", loc="right", fontsize=fs))

    # Effect IN/OUT arrows from switch box
    arr_x = out_x + su * 0.2
    eff_in_y = sw_top - su * 0.15
    eff_out_y = sw_bot + su * 0.15

    d.add(elm.Line().at((out_x, sw_top)).to((arr_x, eff_in_y)))
    d.add(elm.Label().at((arr_x + 0.2, eff_in_y))
          .label("\u2192 Effect IN", fontsize=fs, halign="left"))

    d.add(elm.Line().at((out_x, sw_bot)).to((arr_x, eff_out_y)))
    d.add(elm.Label().at((arr_x + 0.2, eff_out_y))
          .label("\u2190 Effect OUT", fontsize=fs, halign="left"))

    # LED indicator — right of switch, going down
    led_x = out_x + su * 0.1
    led_top = sw_bot
    d.add(elm.Vdd().at((led_x, led_top)).label("9V", fontsize=fs))
    rl = d.add(elm.Resistor().down().at((led_x, led_top)).length(su * 0.8)
               .label("R_LED\n4.7k\u03a9", loc="left", ofst=0.15, fontsize=fs))
    led = d.add(elm.LED().down().at(rl.end).length(su * 0.8)
                .label("LED", loc="left", ofst=0.15, fontsize=fs))
    d.add(elm.Ground().at(led.end))


def render_schematic(name, components, edges, multi_edges, comp_map, adj, output_path, dpi=150, hw_parts=None):
    """Render the circuit schematic using schemdraw.

    Args:
        hw_parts: optional dict {comp_id: part_name} from .pedalhw file.
            When present, part names are shown on the schematic labels.
    """
    hw_parts = hw_parts or {}
    spine_comps, branches, parallel_comps, feedback_comps = classify_components(
        edges, multi_edges, comp_map, adj
    )

    n_comps = len(components)
    spacing = get_spacing(n_comps)
    unit = spacing * 2

    # Build multi-edge lookup
    multi_map = {}  # comp_id -> pin_junctions
    for comp_id, pin_junctions in multi_edges:
        multi_map[comp_id] = pin_junctions

    # Collect branch info for each spine junction
    branch_map = defaultdict(list)  # spine_junction -> [(comp_id, target, direction)]
    for comp_id, spine_j, target_j, direction in branches:
        branch_map[spine_j].append((comp_id, target_j, direction))

    # Pick backend: matplotlib for PNG/PDF, SVG for .svg
    ext = os.path.splitext(output_path)[1].lower()
    if ext == ".svg":
        schemdraw.use("svg")
    elif HAS_MATPLOTLIB:
        schemdraw.use("matplotlib")
    else:
        print(
            "Warning: matplotlib not installed. Only SVG output is supported.",
            file=sys.stderr,
        )
        schemdraw.use("svg")
        output_path = os.path.splitext(output_path)[0] + ".svg"

    with schemdraw.Drawing(show=False) as d:
        d.config(unit=unit, fontsize=11, font=FONT_FAMILY)

        # Track junction positions
        junction_pos = {}

        # Draw input terminal
        inp = d.add(elm.Dot(open=True).label("IN", loc="left"))
        junction_pos["in"] = (inp.center[0], inp.center[1])

        # Draw spine left-to-right
        for comp_id, ja, jb in spine_comps:
            comp = comp_map[comp_id]
            kind = comp["kind"]
            label = make_label(comp_id, kind, comp["arg"], hw_parts.get(comp_id))

            if kind in ("npn", "pnp"):
                # Draw a short wire, then place BJT with base at current pos
                wire = d.add(elm.Line().right().length(unit * 0.3))
                base_pos = wire.end

                bjt_el, pins = _draw_element(d, kind, label, pos=base_pos)
                junction_pos[ja] = base_pos

                # Continue from collector (signal goes base -> collector for common emitter)
                collector_pos = pins["collector"]
                emitter_pos = pins["emitter"]

                # Figure out which pin connects forward on spine
                pin_j = multi_map.get(comp_id, {})
                collector_junc = pin_j.get("collector")
                emitter_junc = pin_j.get("emitter")
                base_junc = pin_j.get("base")

                junction_pos.setdefault(base_junc, base_pos)
                junction_pos.setdefault(collector_junc, collector_pos)
                junction_pos.setdefault(emitter_junc, emitter_pos)

                # Draw wire from collector rightward to continue spine
                d.add(elm.Line().right().at(collector_pos).length(unit * 0.3))

            elif kind == "opamp":
                # Place opamp — in1 is +, in2 is -, out is output
                wire = d.add(elm.Line().right().length(unit * 0.2))
                in_pos = wire.end

                oa_el, pins = _draw_element(d, kind, label, pos=in_pos)

                pin_j = multi_map.get(comp_id, {})
                pos_junc = pin_j.get("pos")
                neg_junc = pin_j.get("neg")
                out_junc = pin_j.get("out")

                junction_pos.setdefault(pos_junc, pins["in1"])
                junction_pos.setdefault(neg_junc, pins["in2"])
                junction_pos.setdefault(out_junc, pins["out"])
                junction_pos[ja] = pins["in1"]
                junction_pos[jb] = pins["out"]

                # Continue from output
                d.add(elm.Line().right().at(pins["out"]).length(unit * 0.2))

            else:
                el, pts = _draw_element(d, kind, label, direction="right")
                junction_pos[ja] = pts["start"]
                junction_pos[jb] = pts["end"]

        # Draw output terminal
        out_elem = d.add(elm.Dot(open=True).label("OUT", loc="right"))
        junction_pos["out"] = (out_elem.center[0], out_elem.center[1])

        # Draw branches — group by junction, offset horizontally to avoid overlap
        branch_spacing = unit * 0.7
        for spine_j, branch_list in branch_map.items():
            pos = junction_pos.get(spine_j)
            if pos is None:
                continue

            # Count branches by direction for offset calculation
            gnd_idx = 0
            vcc_idx = 0
            other_idx = 0
            n_gnd = sum(1 for _, _, d in branch_list if d == "gnd")
            n_vcc = sum(1 for _, _, d in branch_list if d == "vcc")

            for comp_id, target_j, direction in branch_list:
                comp = comp_map[comp_id]
                kind = comp["kind"]
                label = make_label(comp_id, kind, comp["arg"], hw_parts.get(comp_id))

                if direction == "gnd":
                    # Center the gnd branches around the junction x-position
                    offset = (gnd_idx - (n_gnd - 1) / 2) * branch_spacing
                    branch_pos = (pos[0] + offset, pos[1])
                    # Alternate label side to reduce overlap
                    lbl_loc = "left" if gnd_idx % 2 == 0 else "right"
                    gnd_idx += 1

                    d.add(elm.Dot().at(pos))
                    if abs(offset) > 0.01:
                        d.add(elm.Line().at(pos).to(branch_pos))

                    el, pts = _draw_element(d, kind, label, direction="down",
                                            pos=branch_pos, label_loc=lbl_loc)
                    d.add(elm.Ground().at(pts["end"]))

                elif direction == "vcc":
                    offset = (vcc_idx - (n_vcc - 1) / 2) * branch_spacing
                    branch_pos = (pos[0] + offset, pos[1])
                    lbl_loc = "left" if vcc_idx % 2 == 0 else "right"
                    vcc_idx += 1

                    d.add(elm.Dot().at(pos))
                    if abs(offset) > 0.01:
                        d.add(elm.Line().at(pos).to(branch_pos))

                    el, pts = _draw_element(d, kind, label, direction="up",
                                            pos=branch_pos, label_loc=lbl_loc)
                    d.add(elm.Vdd().at(pts["end"]).label("V+"))

                else:
                    # Generic branch — draw downward with offset
                    offset = other_idx * branch_spacing
                    branch_pos = (pos[0] + offset, pos[1])
                    lbl_loc = "left" if other_idx % 2 == 0 else "right"
                    other_idx += 1

                    d.add(elm.Dot().at(pos))
                    if abs(offset) > 0.01:
                        d.add(elm.Line().at(pos).to(branch_pos))

                    el, pts = _draw_element(d, kind, label, direction="down",
                                            pos=branch_pos, label_loc=lbl_loc)
                    junction_pos[target_j] = pts["end"]

        # Draw parallel components (both ends on spine, arc above)
        for i, (comp_id, ja, jb) in enumerate(parallel_comps):
            comp = comp_map[comp_id]
            kind = comp["kind"]
            label = make_label(comp_id, kind, comp["arg"], hw_parts.get(comp_id))
            elem_cls = SCHEMDRAW_MAP.get(kind, elm.Resistor)

            pos_a = junction_pos.get(ja)
            pos_b = junction_pos.get(jb)
            if pos_a is None or pos_b is None:
                continue

            d.add(elm.Dot().at(pos_a))
            d.add(elm.Dot().at(pos_b))

            arc_h = unit * 0.5 * (i + 1)
            top_a = (pos_a[0], pos_a[1] + arc_h)
            top_b = (pos_b[0], pos_b[1] + arc_h)

            d.add(elm.Line().at(pos_a).to(top_a))
            d.add(elem_cls().endpoints(top_a, top_b).label(label, loc="top"))
            d.add(elm.Line().at(top_b).to(pos_b))

        # Draw feedback components (arc above the circuit)
        for i, (comp_id, ja, jb) in enumerate(feedback_comps):
            comp = comp_map[comp_id]
            kind = comp["kind"]
            label = make_label(comp_id, kind, comp["arg"], hw_parts.get(comp_id))
            elem_cls = SCHEMDRAW_MAP.get(kind, elm.Resistor)

            pos_a = junction_pos.get(ja)
            pos_b = junction_pos.get(jb)
            if pos_a is None or pos_b is None:
                continue

            d.add(elm.Dot().at(pos_a))
            d.add(elm.Dot().at(pos_b))

            # Higher arc for feedback
            arc_h = unit * 0.8 * (i + 1)
            top_a = (pos_a[0], pos_a[1] + arc_h)
            top_b = (pos_b[0], pos_b[1] + arc_h)

            d.add(elm.Line().at(pos_a).to(top_a))
            d.add(elem_cls().endpoints(top_a, top_b).label(label, loc="top"))
            d.add(elm.Line().at(top_b).to(pos_b))

        # --- Standard islands (power supply + bypass switching) ---
        bbox = d.get_bbox()
        island_y = bbox.ymin - 3.0  # below the main circuit
        circuit_w = bbox.xmax - bbox.xmin

        # Power supply island — bottom-left
        _draw_power_supply(d, (bbox.xmin, island_y), unit)

        # Bypass switching island — right of power supply with gap
        bypass_x = bbox.xmin + max(circuit_w * 0.5, unit * 3.5)
        _draw_bypass_switching(d, (bypass_x, island_y), unit)

        # EE-style title block (bottom-right corner)
        bbox = d.get_bbox()  # re-compute after islands
        tb_w = max(8, (bbox.xmax - bbox.xmin) * 0.4)
        tb_h = 2.0
        tb_x = bbox.xmax - tb_w
        tb_y = bbox.ymin - 2.5

        # Title block border
        d.add(elm.Line().at((tb_x, tb_y)).to((tb_x + tb_w, tb_y)))
        d.add(elm.Line().at((tb_x + tb_w, tb_y)).to((tb_x + tb_w, tb_y + tb_h)))
        d.add(elm.Line().at((tb_x + tb_w, tb_y + tb_h)).to((tb_x, tb_y + tb_h)))
        d.add(elm.Line().at((tb_x, tb_y + tb_h)).to((tb_x, tb_y)))
        # Horizontal divider
        d.add(elm.Line().at((tb_x, tb_y + tb_h * 0.5)).to((tb_x + tb_w, tb_y + tb_h * 0.5)))
        # Vertical divider in bottom half
        d.add(elm.Line().at((tb_x + tb_w * 0.5, tb_y)).to((tb_x + tb_w * 0.5, tb_y + tb_h * 0.5)))

        # Top row: PedalKernel + pedal name
        d.add(elm.Label().at((tb_x + 0.2, tb_y + tb_h * 0.75))
              .label("PedalKernel", fontsize=10, halign="left"))
        d.add(elm.Label().at((tb_x + tb_w * 0.5, tb_y + tb_h * 0.75))
              .label(name, fontsize=14, halign="center"))

        # Bottom-left: Drawn by
        d.add(elm.Label().at((tb_x + 0.2, tb_y + tb_h * 0.25))
              .label(f"Drawn by: PedalKernel", fontsize=8, halign="left"))

        # Bottom-right: Rev, Date, Page
        today = date.today().isoformat()
        d.add(elm.Label().at((tb_x + tb_w * 0.52, tb_y + tb_h * 0.35))
              .label(f"Rev: 1.0", fontsize=8, halign="left"))
        d.add(elm.Label().at((tb_x + tb_w * 0.52, tb_y + tb_h * 0.15))
              .label(f"Date: {today}   Page 1 of 1", fontsize=8, halign="left"))

        # Save — white background for raster, transparent for vector
        ext = os.path.splitext(output_path)[1].lower()
        transparent = ext in (".svg", ".pdf")
        d.save(output_path, dpi=dpi, transparent=transparent)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main():
    parser = argparse.ArgumentParser(
        description="Render a .pedal circuit as an EE-style schematic."
    )
    parser.add_argument("pedal_file", help="Path to .pedal file")
    parser.add_argument(
        "-o", "--output", default=None,
        help="Output file path (PNG/PDF/SVG, inferred from extension). "
             "Default: <pedal_name>.png",
    )
    parser.add_argument(
        "--dpi", type=int, default=150,
        help="DPI for PNG output (default: 150)",
    )
    args = parser.parse_args()

    name, components, nets = parse_pedal(args.pedal_file)

    # Auto-discover companion .pedalhw file for part name annotations
    hw_path = find_pedalhw(args.pedal_file)
    hw_parts = parse_pedalhw(hw_path)
    if hw_parts:
        print(f"  Using hardware specs from: {hw_path}")

    output_path = args.output
    if output_path is None:
        base = os.path.splitext(os.path.basename(args.pedal_file))[0]
        output_path = f"{base}.png"

    uf, comp_map, edges, multi_edges = build_graph(components, nets)
    adj = build_adjacency(edges, multi_edges, comp_map)

    render_schematic(name, components, edges, multi_edges, comp_map, adj, output_path, args.dpi, hw_parts)

    print(f"Schematic saved to: {output_path}")


if __name__ == "__main__":
    main()
