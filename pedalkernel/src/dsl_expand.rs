//! File-based subcircuit expansion (flatten pass).
//!
//! Front-end only: turns a [`PedalDef`] containing `use("path.pedal")` instances
//! (declared in the `components { ... }` block, collected into
//! [`PedalDef::uses`]) into ONE flat `PedalDef` that the existing
//! `compile_pedal` accepts unchanged. The WDF/SPQR compiler and runtime are
//! never touched.
//!
//! A `use(...)` instance is FLATTENED into the parent's single component / net /
//! control list (unlike inline `subcircuit { ... }` blocks, which partition the
//! graph into separate rate-domain processors via
//! [`crate::compiler::subcircuit`] — file-based uses must NOT route through
//! that path).
//!
//! ## Namespacing
//! For an instance `id`:
//! - component `R3` → `id.R3`; every `ComponentPin { component, .. }` is
//!   rewritten to `id.component`.
//! - a bare internal node `Reserved(name)` (not `gnd`, not a supply rail, not a
//!   declared port) → `Reserved("id.name")`.
//! - `gnd` stays global.
//! - a supply rail (`vcc` or a named supply of the sub) stays global by
//!   default, UNLESS the parent explicitly maps `id.<rail>` to a node — then it
//!   unifies with that node (implicit-global, overridable).
//! - a declared port `P` (from the sub's `ports { }`, plus `in`/`out`) unifies
//!   with whatever node the parent wired to `id.P`. A used-but-unmapped non-rail
//!   port is a loud error.
//!
//! ## Controls
//! A sub control on component `C` becomes a control on `id.C`. A parent control
//! referencing `id.<subctrl>` re-targets/renames the namespaced sub control.

use crate::compiler::components::{Capacitor, Inductor, Potentiometer, Resistor};
use crate::dsl::{ControlDef, NetDef, PedalDef, Pin, UseInstance};
use std::path::{Path, PathBuf};

/// Default reserved rail/IO names that are always global (never namespaced) when
/// they appear as bare `Reserved` nodes in a sub. `in`/`out` are NOT in here —
/// they are treated as declared ports (boundary nodes) and unified with the
/// parent.
const GLOBAL_GND: &str = "gnd";

/// Reserved names that, when used as a sub's internal node, stay global as a
/// shared rail unless explicitly mapped by the parent.
const DEFAULT_RAILS: &[&str] = &["vcc"];

/// Expand all file-based `use(...)` subcircuit instances in `pedal` into a
/// single flat `PedalDef`.
///
/// `base_dir` is the directory the parent `.pedal` file lives in; `use(...)`
/// paths are resolved relative to it. Nested subcircuits are expanded first
/// (depth-first), using each sub file's own directory as its base.
///
/// Returns a flat pedal whose `uses` is empty. Idempotent for pedals with no
/// `use(...)` instances (returns an equivalent clone).
pub fn expand_uses(pedal: &PedalDef, base_dir: &Path) -> Result<PedalDef, String> {
    let mut visited = Vec::new();
    expand_inner(pedal, base_dir, &mut visited)
}

fn expand_inner(
    pedal: &PedalDef,
    base_dir: &Path,
    visited: &mut Vec<PathBuf>,
) -> Result<PedalDef, String> {
    if pedal.uses.is_empty() {
        // Nothing to flatten — return a clone with uses cleared (idempotent).
        let mut out = pedal.clone();
        out.uses.clear();
        return Ok(out);
    }

    let mut out = pedal.clone();
    let uses = std::mem::take(&mut out.uses);

    // Collect parent-side port/rail mappings BEFORE merging, then drop the
    // routing nets that are fully consumed by unification.
    for inst in &uses {
        merge_instance(&mut out, inst, base_dir, visited)?;
    }

    out.uses.clear();
    Ok(out)
}

/// Resolve, parse, recursively expand, namespace and merge one instance.
fn merge_instance(
    parent: &mut PedalDef,
    inst: &UseInstance,
    base_dir: &Path,
    visited: &mut Vec<PathBuf>,
) -> Result<(), String> {
    let UseInstance {
        id,
        path,
        overrides: _,
    } = inst;

    // ---- Resolve + cycle check ---------------------------------------------
    let resolved = base_dir.join(path);
    let canonical = resolved
        .canonicalize()
        .map_err(|e| format!("subcircuit `{id}`: cannot resolve `{path}`: {e}"))?;
    if visited.contains(&canonical) {
        return Err(format!(
            "subcircuit cycle detected: `{}` is already being expanded",
            canonical.display()
        ));
    }

    let src = std::fs::read_to_string(&canonical).map_err(|e| {
        format!(
            "subcircuit `{id}`: cannot read `{}`: {e}",
            canonical.display()
        )
    })?;
    let sub_raw = crate::dsl::parse_pedal_file(&src)
        .map_err(|e| format!("subcircuit `{id}` (`{}`): {e}", canonical.display()))?;

    // ---- Recurse: expand nested uses first ---------------------------------
    let sub_dir = canonical
        .parent()
        .map(|p| p.to_path_buf())
        .unwrap_or_else(|| PathBuf::from("."));
    visited.push(canonical.clone());
    let mut sub = expand_inner(&sub_raw, &sub_dir, visited)?;
    visited.pop();

    // ---- Apply component-value overrides -----------------------------------
    // Each `(comp_id, new_value)` from the `with { }` block is applied to the
    // matching sub ComponentDef before namespacing. Error loudly if any id has
    // no matching component (Rule 12 — do not silently ignore).
    for (ov_id, ov_val) in &inst.overrides {
        // Collect available ids before the mutable borrow so the error message
        // can list them without conflicting borrows.
        let available: Vec<String> = sub.components.iter().map(|c| c.id.clone()).collect();

        let comp = sub
            .components
            .iter_mut()
            .find(|c| c.id == *ov_id)
            .ok_or_else(|| {
                format!(
                    "subcircuit `{id}` (`{path}`): override `{ov_id}` does not match any \
                     component in the sub (available: {})",
                    available.join(", ")
                )
            })?;

        let kind = comp.kind.as_mut();
        if let Some(r) = kind.as_any_mut().downcast_mut::<Resistor>() {
            r.value = *ov_val;
        } else if let Some(c) = kind.as_any_mut().downcast_mut::<Capacitor>() {
            c.config.value = *ov_val;
        } else if let Some(l) = kind.as_any_mut().downcast_mut::<Inductor>() {
            l.value = *ov_val;
        } else if let Some(p) = kind.as_any_mut().downcast_mut::<Potentiometer>() {
            p.max_r = *ov_val;
        } else {
            return Err(format!(
                "subcircuit `{id}` (`{path}`): override `{ov_id}` targets a component type \
                 that does not support value overrides (only resistor, cap, inductor, pot)"
            ));
        }
    }

    // ---- Classify the sub's reserved node names ----------------------------
    // Declared ports: from `ports { }` plus implicit in/out.
    let mut port_names: Vec<String> = sub.ports.iter().map(|p| p.name.clone()).collect();
    for implicit in ["in", "out"] {
        if !port_names.iter().any(|n| n == implicit) {
            port_names.push(implicit.to_string());
        }
    }
    // Rail names: the sub's declared supplies plus the default `vcc`.
    let mut rail_names: Vec<String> = sub.supplies.iter().map(|s| s.name.clone()).collect();
    for r in DEFAULT_RAILS {
        if !rail_names.iter().any(|n| n == r) {
            rail_names.push(r.to_string());
        }
    }

    // ---- Gather parent mappings for this instance --------------------------
    // For each parent net mentioning `SubcircuitPort { subcircuit: id, port }`,
    // record the node the port unifies with, and drop the routing net.
    let mut port_map: hashbrown::HashMap<String, Pin> = hashbrown::HashMap::new();
    let mut kept_nets: Vec<NetDef> = Vec::with_capacity(parent.nets.len());

    for net in std::mem::take(&mut parent.nets) {
        if let Some((port, target, leftover)) = consume_routing_net(&net, id) {
            // Unify this port with `target`. If the port already mapped to a
            // node, keep the first; emit an explicit net joining the two so the
            // extra parent endpoints aren't lost.
            match port_map.get(&port).cloned() {
                None => {
                    port_map.insert(port.clone(), target.clone());
                }
                Some(existing) => {
                    kept_nets.push(NetDef {
                        from: existing,
                        to: vec![target.clone()],
                    });
                }
            }
            // Any leftover endpoints (port wired to >1 node) are re-attached to
            // the unified node.
            if let Some(extra) = leftover {
                let anchor = port_map.get(&port).cloned().unwrap_or(target);
                kept_nets.push(NetDef {
                    from: anchor,
                    to: extra,
                });
            }
        } else {
            kept_nets.push(net);
        }
    }
    parent.nets = kept_nets;

    // ---- Validate: every used non-rail port must be mapped -----------------
    // A port is "used" if the sub references it as a Reserved node.
    let sub_used_reserved = collect_reserved_names(&sub);
    for port in &port_names {
        let is_rail = rail_names.iter().any(|r| r == port);
        if is_rail {
            continue;
        }
        if sub_used_reserved.contains(port.as_str()) && !port_map.contains_key(port) {
            return Err(format!(
                "subcircuit `{id}`: port `{port}` is used internally but the parent does not wire `{id}.{port}` to any node"
            ));
        }
    }

    // ---- Build the reserved-node rewrite map -------------------------------
    let rewrite_reserved = |name: &str| -> Pin {
        if name == GLOBAL_GND {
            return Pin::Reserved(name.to_string());
        }
        // Explicit parent mapping (port OR rail override) wins.
        if let Some(target) = port_map.get(name) {
            return target.clone();
        }
        if rail_names.iter().any(|r| r == name) {
            // Implicit-global rail.
            return Pin::Reserved(name.to_string());
        }
        if port_names.iter().any(|p| p == name) {
            // Declared, unused-but-unmapped port that is not referenced
            // internally: harmless, leave as a namespaced node.
            return Pin::Reserved(format!("{id}.{name}"));
        }
        // Bare internal node → namespace it.
        Pin::Reserved(format!("{id}.{name}"))
    };

    let ns_pin = |pin: &Pin| -> Pin { namespace_pin(pin, id, &rewrite_reserved) };

    // ---- Merge namespaced components ---------------------------------------
    for comp in &sub.components {
        let mut c = comp.clone();
        c.id = format!("{id}.{}", comp.id);
        parent.components.push(c);
    }

    // ---- Merge namespaced nets ---------------------------------------------
    for net in &sub.nets {
        parent.nets.push(NetDef {
            from: ns_pin(&net.from),
            to: net.to.iter().map(|p| ns_pin(p)).collect(),
        });
    }

    // ---- Merge namespaced mirrors ------------------------------------------
    for (k, v) in &sub.mirrors {
        parent
            .mirrors
            .insert(format!("{id}.{k}"), format!("{id}.{v}"));
    }

    // ---- Merge namespaced controls -----------------------------------------
    // Sub controls become controls on `id.C`. Parent controls that reference
    // `id.<sub-control-component>` already point at the namespaced component
    // after we rewrite the parent control component below.
    for ctrl in &sub.controls {
        parent.controls.push(ControlDef {
            component: format!("{id}.{}", ctrl.component),
            property: ctrl.property.clone(),
            label: ctrl.label.clone(),
            range: ctrl.range,
            default: ctrl.default,
        });
    }
    for trim in &sub.trims {
        parent.trims.push(ControlDef {
            component: format!("{id}.{}", trim.component),
            property: trim.property.clone(),
            label: trim.label.clone(),
            range: trim.range,
            default: trim.default,
        });
    }

    // ---- Re-expose sub controls via parent controls ------------------------
    // A parent control written as `id.SubCtrl -> "Label" ...` references the sub
    // by instance id; its `component` is the instance `id` and `property` is the
    // sub control's label/component. We resolve these to the namespaced sub
    // control: drop the auto-merged sub control with the same target and keep
    // the parent's (relabeled/ranged) one.
    rebind_parent_controls(parent, id, &sub);

    // ---- Merge supplies (dedupe by name; parent wins) ----------------------
    for sup in &sub.supplies {
        if !parent.supplies.iter().any(|p| p.name == sup.name) {
            parent.supplies.push(sup.clone());
        }
    }

    // ---- Merge init hints (namespaced device labels) -----------------------
    for hint in &sub.init_hints {
        let mut h = hint.clone();
        h.device_label = format!("{id}.{}", hint.device_label);
        parent.init_hints.push(h);
    }

    Ok(())
}

/// If `net` is a pure routing net for instance `id` (it mentions
/// `SubcircuitPort { subcircuit: id, port }`), return
/// `(port, unify_target, leftover_endpoints)` and signal the net should be
/// dropped. The common 1:1 case (`X -> id.P` or `id.P -> X`) yields a single
/// target and no leftover.
///
/// Returns `None` if the net does not reference this instance's ports (it is
/// kept verbatim).
fn consume_routing_net(net: &NetDef, id: &str) -> Option<(String, Pin, Option<Vec<Pin>>)> {
    let from_port = sub_port_of(&net.from, id);
    let to_ports: Vec<(usize, String)> = net
        .to
        .iter()
        .enumerate()
        .filter_map(|(i, p)| sub_port_of(p, id).map(|port| (i, port)))
        .collect();

    match (from_port, to_ports.is_empty()) {
        (Some(port), true) => {
            // `id.P -> [targets...]`: the targets are the unify node(s).
            let mut targets = net.to.clone();
            let first = targets.remove(0);
            let leftover = if targets.is_empty() {
                None
            } else {
                Some(targets)
            };
            Some((port, first, leftover))
        }
        (None, false) => {
            // `from -> [.., id.P, ..]`: `from` is the unify node. Other
            // non-port destinations become leftover (re-attached to the node).
            let port = to_ports[0].1.clone();
            let port_idxs: std::collections::HashSet<usize> =
                to_ports.iter().map(|(i, _)| *i).collect();
            let leftover: Vec<Pin> = net
                .to
                .iter()
                .enumerate()
                .filter(|(i, _)| !port_idxs.contains(i))
                .map(|(_, p)| p.clone())
                .collect();
            let leftover = if leftover.is_empty() {
                None
            } else {
                Some(leftover)
            };
            Some((port, net.from.clone(), leftover))
        }
        (Some(port), false) => {
            // Both sides reference instance ports — rare; treat `from`'s port as
            // the one mapped, target = first to-port's pin (already a
            // SubcircuitPort, which is unusual). Fall back to dropping with the
            // from side mapped to the first to endpoint.
            let target = net.to[0].clone();
            Some((port, target, None))
        }
        (None, true) => None,
    }
}

/// Returns the port name if `pin` is `SubcircuitPort { subcircuit: id, .. }`.
fn sub_port_of(pin: &Pin, id: &str) -> Option<String> {
    match pin {
        Pin::SubcircuitPort { subcircuit, port } if subcircuit == id => Some(port.clone()),
        _ => None,
    }
}

/// Namespace a single pin from a sub into the parent scope.
fn namespace_pin(pin: &Pin, id: &str, rewrite_reserved: &dyn Fn(&str) -> Pin) -> Pin {
    match pin {
        Pin::Reserved(name) => rewrite_reserved(name),
        Pin::ComponentPin { component, pin } => Pin::ComponentPin {
            component: format!("{id}.{component}"),
            pin: pin.clone(),
        },
        Pin::Fork {
            switch,
            destinations,
        } => Pin::Fork {
            switch: format!("{id}.{switch}"),
            destinations: destinations
                .iter()
                .map(|d| namespace_pin(d, id, rewrite_reserved))
                .collect(),
        },
        // A sub should not itself contain unexpanded SubcircuitPort references
        // (nested uses are flattened before we get here). Pass through.
        Pin::SubcircuitPort { subcircuit, port } => Pin::SubcircuitPort {
            subcircuit: format!("{id}.{subcircuit}"),
            port: port.clone(),
        },
    }
}

/// Collect the set of bare `Reserved` node names referenced anywhere in a sub's
/// nets (used to decide whether an unmapped port is actually "used").
fn collect_reserved_names(sub: &PedalDef) -> std::collections::HashSet<String> {
    let mut set = std::collections::HashSet::new();
    let mut visit = |pin: &Pin| {
        if let Pin::Reserved(name) = pin {
            set.insert(name.clone());
        }
    };
    for net in &sub.nets {
        collect_pin(&net.from, &mut visit);
        for p in &net.to {
            collect_pin(p, &mut visit);
        }
    }
    set
}

fn collect_pin(pin: &Pin, f: &mut dyn FnMut(&Pin)) {
    f(pin);
    if let Pin::Fork { destinations, .. } = pin {
        for d in destinations {
            collect_pin(d, f);
        }
    }
}

/// Re-bind parent controls that reference instance `id`.
///
/// A parent control `id.X -> "Label" [..] = ..` (parsed as
/// `component == id`, `property == X`) re-exposes a sub control. We match it to
/// the auto-merged sub control whose namespaced component is `id.X` OR whose
/// label is `X`, replace that sub control's label/range/default with the
/// parent's, and remove the now-redundant parent control entry.
fn rebind_parent_controls(parent: &mut PedalDef, id: &str, sub: &PedalDef) {
    // At this point `parent.controls` is [original parent controls...,
    // auto-merged sub controls...]. Separate the parent re-exposing controls
    // (those whose `component == id`) from the merged sub controls, then fold
    // each re-exposing control into the matching merged sub control.
    let all = std::mem::take(&mut parent.controls);
    let mut reexpose: Vec<ControlDef> = Vec::new();
    let mut merged: Vec<ControlDef> = Vec::new();
    for c in all {
        if c.component == id {
            reexpose.push(c);
        } else {
            merged.push(c);
        }
    }

    for ctrl in reexpose {
        // Parent re-exposes `id.<property>`. Match the merged sub control by
        // namespaced component name OR by sub control label.
        let want = &ctrl.property;
        let ns_comp = format!("{id}.{want}");
        if let Some(slot) = merged.iter_mut().find(|c| {
            c.component == ns_comp
                || sub
                    .controls
                    .iter()
                    .any(|sc| &sc.label == want && format!("{id}.{}", sc.component) == c.component)
        }) {
            slot.label = ctrl.label.clone();
            slot.range = ctrl.range;
            slot.default = ctrl.default;
        } else {
            // No matching sub control — keep it so the issue surfaces rather
            // than being silently dropped.
            merged.push(ctrl);
        }
    }

    parent.controls = merged;
}

/// Path-aware entry point: read `path`, parse, and expand its `use(...)`
/// instances relative to the file's own directory.
///
/// This is the entry CLI/tests should use for file-path loads so `use(...)`
/// works end-to-end. String-only callers continue to use
/// [`crate::dsl::parse_pedal_file`] (which leaves `uses` un-expanded).
pub fn parse_and_expand_pedal_file(path: &Path) -> Result<PedalDef, String> {
    let src = std::fs::read_to_string(path)
        .map_err(|e| format!("cannot read `{}`: {e}", path.display()))?;
    let def = crate::dsl::parse_pedal_file(&src)?;
    let base_dir = path.parent().unwrap_or_else(|| Path::new("."));
    expand_uses(&def, base_dir)
}
