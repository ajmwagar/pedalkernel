---
title: "The Component trait"
description: "How circuit elements plug into the compiler, and how to add a new one."
section: "Internals"
weight: 86
source_commit: "95744ce1cdd9c2cdec3550bfdce9879b1737312c"
preview: true
watches:
  - pedalkernel/src/compiler/component.rs
  - pedalkernel/src/compiler/components/
  - pedalkernel/src/dsl.rs
---

# The Component Trait

> **Preview.** This page describes the component system as it exists on the `feature/spqr-tree` branch. The underlying trait is older, but several of the dispatch points (signal-flow grouping via `output_impedance()`, edge classification via `edges()`) are specific to the SPQR compiler. See [compiler internals](./compiler-internals.md) for the pipeline context.

This is the piece of the compiler that determines "what does a circuit element do, and how does it participate in compilation?" Every resistor, op-amp, tube, pot, transformer, and delay line implements the `Component` trait. The compiler queries trait methods; it never pattern-matches on concrete types. That's the whole design: adding a new component type doesn't require editing any compiler pass.

## The trait at a glance

`Component` lives in `pedalkernel/src/compiler/component.rs`. It's a fat trait — around thirty methods — but most are defaulted. A minimal implementation has to answer four questions; the rest fall out of sensible defaults.

The methods group into these buckets:

**Identity and metadata.** `type_tag` ("resistor", "NPN transistor"), `display_value`, `footprint_ref` (KiCad library ref + designator), `model_name`. Used by the layout engine, the KiCad exporter, the BOM generator, and error messages.

**Classification.** `is_passive`, `is_nonlinear`, `is_variable`, `is_gain_device`, `is_pot`, `is_bjt`, and so on. These drive routing decisions at every compiler stage. Most are defaulted to `false`; concrete types override the handful that apply.

**Signal flow.** `signal_terminals()` returns `Passive` / `TwoPort { input, output }` / `Amplifier { input, output, control }`. `output_impedance()` returns `VoltageSource` (op-amp outputs, buffers) or `Finite` (everything else). These drive the signal-flow grouping pass — `VoltageSource` outputs act as barriers that end a passive group.

**Graph construction.** `graph_role()` returns `Edge` for ordinary two-terminal parts, `Pot` for three-terminal pots, `ActiveEdge` for devices with control pins, `Virtual` for metadata-only pins. `edges()` declares `EdgeKind::{Linear, Reactive, Nonlinear, Vcvs, Behavioral}` for each port. `resolve_edges()` lets a component change its mind based on neighbours (a JFET used as a modulated variable resistor reports `Linear` instead of `Nonlinear`).

**MNA stamping.** For elements that end up inside an R-node (rigid subgraph), `stamp_mna()` writes into a `MnaSystem`. Most passives have a tiny implementation (`stamp_resistor`, `stamp_capacitor`). Op-amps use `stamp_mna_multi` to resolve three pins (pos, neg, out) and add the VCVS constraint.

**WDF leaf creation.** `make_leaf()` returns `Some(DynNode)` for elements that become leaves in a WDF tree (resistor, capacitor, inductor, pot). Nonlinear and virtual components return `None` — they aren't leaves; they're tree roots or external.

**Non-idealities.** `nonideal_fx()` returns a `Vec<NonIdealFx>` of post-processing effects — `OpAmpBandwidth` (GBW-derived lowpass), `RailSaturation` (per-device output swing). These are applied as a shared layer on stage outputs, not in the scattering matrix.

**Controls and modulation.** `controls()` declares what the user can turn at runtime — typically a `PotPosition` on a pot, or `Rate`/`Depth` on an LFO. `modulation_sink()` says where modulation signals enter a component (JFET gate, photocoupler cell).

**Trait-object plumbing.** `clone_box`, `as_any`, `as_any_mut`, `dyn_eq`. All provided by the `impl_component_dyn!()` macro so every concrete type gets them for one line of boilerplate.

## Trait objects, not an enum

The compiler uses `Box<dyn Component>` everywhere. There is **no** `ComponentKind` enum with a variant per device. The concrete type is erased immediately after the DSL parser produces it, and the pipeline interacts with components only through the trait.

This matters for two reasons. One, it lets you add a new component without touching any pattern match — nothing in the compiler says "if this is a `Resistor`, do X; if it's a `Triode`, do Y." Two, it means the trait must cover every question the compiler needs to ask. Hence its size.

The `impl_component_dyn!()` macro handles the cross-cutting concerns trait objects need (clone, downcast, equality):

```rust
macro_rules! impl_component_dyn {
    () => {
        fn clone_box(&self) -> Box<dyn Component> { Box::new(self.clone()) }
        fn as_any(&self) -> &dyn std::any::Any { self }
        fn as_any_mut(&mut self) -> &mut dyn std::any::Any { self }
        fn dyn_eq(&self, other: &dyn Component) -> bool {
            other.as_any().downcast_ref::<Self>().map_or(false, |o| self == o)
        }
    };
}
```

Every concrete `impl Component` calls `impl_component_dyn!();` as its first line.

## Where components live

`pedalkernel/src/compiler/components/` holds one file per component family:

- `passives.rs` — resistor, capacitor, inductor, potentiometer, tempco, switched variants
- `diodes.rs` — diode, diode pair, zener, neon
- `transistors.rs` — NPN, PNP, N-/P-JFET, N-/P-MOSFET
- `tubes.rs` — triode, pentode, vari-mu
- `active_ics.rs` — op-amp (incl. OTA), VCO, VCF, VCA, comparator, analog switch, matched pairs
- `delay.rs` — BBD, delay line, tap
- `modulation.rs` — LFO, envelope follower, photocoupler
- `switches.rs` — switch, rotary switch, trigger input
- `transformer.rs` — audio transformer with multiple winding kinds

Registration is convention-over-configuration:

```rust
// components/mod.rs
mod active_ics;
mod delay;
mod diodes;
mod modulation;
mod passives;
mod switches;
mod transformer;
mod transistors;
mod tubes;

pub use active_ics::*;
pub use delay::*;
// ... and so on
```

Every sibling module is `pub use`'d. No registry, no init function, no plugin table. A struct is a component the moment it implements the trait and lives in this directory.

## Adding a new component

Say you want to add a varistor (a voltage-dependent nonlinear resistor). End-to-end walkthrough:

**1. Pick a home.** A varistor is a nonlinear two-terminal part, so it fits in `components/diodes.rs` alongside the zener. Or create `components/varistors.rs` and add `mod varistors; pub use varistors::*;` to `components/mod.rs`.

**2. Declare the struct.**

```rust
#[derive(Debug, Clone, PartialEq)]
pub struct Varistor {
    pub v_ref: f64,        // clamping threshold
    pub nonlinearity: f64, // knee sharpness exponent
}
```

**3. Implement the trait.** Only the methods specific to the device — the rest default.

```rust
impl Component for Varistor {
    impl_component_dyn!();

    fn type_tag(&self) -> &'static str { "varistor" }
    fn is_passive(&self) -> bool { false }
    fn is_nonlinear(&self) -> bool { true }

    fn pin_config(&self) -> PinConfig {
        PinConfig { valid_pins: &["a", "b"], aliases: &[] }
    }
    fn graph_role(&self) -> GraphRole {
        GraphRole::Edge { pin_a: "a", pin_b: "b" }
    }
    fn edges(&self) -> Vec<ComponentEdge> {
        vec![ComponentEdge { pin_a: "a", pin_b: "b", kind: EdgeKind::Nonlinear, port_group: None }]
    }

    fn classify_nonlinear(&self, _id: &str, a: NodeId, b: NodeId, gnd: NodeId,
                          _names: &HashMap<String, NodeId>)
        -> Option<(NonlinearKind, Vec<NodeId>)>
    {
        let junction = if b == gnd { a } else { b };
        Some((NonlinearKind::CustomVaristor(self.v_ref, self.nonlinearity), vec![junction]))
    }

    fn footprint_ref(&self) -> (&'static str, &'static str) { ("Device:MOV", "RV") }
}
```

**4. Add a DSL parser.** In `pedalkernel/src/dsl.rs`, write a nom parser that matches `varistor(v_ref, nonlinearity)` and register it in `component_kind()`.

```rust
fn parse_varistor(input: &str) -> IResult<&str, BoxComp> {
    let (input, _) = tag("varistor")(input)?;
    let (input, _) = char('(')(input)?;
    let (input, v_ref) = double(input)?;
    let (input, _) = ws_comma(input)?;
    let (input, nonlinearity) = double(input)?;
    let (input, _) = char(')')(input)?;
    Ok((input, Box::new(Varistor { v_ref, nonlinearity })))
}

// Register in component_kind's alt() — order matters if prefixes overlap
```

**5. (Optional) Add a `NonlinearKind` variant and solver kernel.** If the device needs a new Newton-Raphson kernel rather than an existing one, extend `NonlinearKind` in `compiler/classify.rs` and implement the kernel in `compiler/stage.rs`. If it fits an existing kernel (just a diode with different parameters), skip this.

That's it. No compiler pass needs editing. Signal-flow grouping will classify the new component as `Finite` output impedance and pass it through; SPQR will handle it as an edge; stage building will route it to a WDF root because `is_nonlinear()` returned true.

## How the DSL parser dispatches

The parser is `nom::alt`-driven, not table-driven. When parsing `R1: resistor(4.7k)`, the sequence is:

1. `component_def()` extracts `R1` as the component ID.
2. `component_kind()` runs `alt((parse_resistor_switched, parse_resistor, parse_cap_switched, parse_cap, ..., parse_opamp, ..., parse_varistor, ...))`.
3. Each sub-parser begins with `tag("resistor")` or similar. First match wins.
4. The matched parser returns `Box<dyn Component>`.

Two consequences. Order matters when one prefix is a substring of another — `parse_resistor_switched` must come before `parse_resistor`. And there is no central table to keep in sync; each parser owns its syntax.

## How the compiler queries the trait

Four passes, in order:

**Signal-flow grouping** (`compiler/signal_flow.rs`) uses `signal_terminals()` and `output_impedance()` to build a directed graph whose nodes are active outputs and whose edges are passive paths. Tarjan's SCC finds feedback cycles. `VoltageSource` outputs become group boundaries.

**SPQR decomposition** (`compiler/spqr.rs`) uses `edges()` via `EdgeKind`. A subgraph with only `Linear` and `Reactive` edges classifies as `AllPassive` (WDF tree). One `Nonlinear` becomes `SingleNl`. Any `Vcvs` forces R-node / MNA handling. Multiple nonlinear elements become `Complex`.

**Stage building** (`compiler/spqr_build.rs`) calls `make_leaf()` for each passive component and assembles adaptor trees. For R-nodes it calls `stamp_mna()` / `stamp_mna_multi()` to populate the MNA matrix. `classify_nonlinear()` identifies where the Newton-Raphson root lives.

**Post-processing attach** (`compiler/stage.rs`) iterates each stage's components and collects `nonideal_fx()` values. Those become a `NonIdealFxState` chain (GBW lowpass, slew limit, rail saturation) applied to stage outputs.

None of these passes names a concrete component type. Everything flows through the trait.

## Auxiliary types

A handful of enums do most of the classifying:

- **`EdgeKind`** — `Linear`, `Reactive`, `Nonlinear`, `Vcvs`, `Vccs`, `Behavioral`. Tagged on each port.
- **`OutputImpedance`** — `VoltageSource` or `Finite`. Drives signal-flow barriers.
- **`SignalTerminals`** — `Passive` / `TwoPort` / `Amplifier`. Tells the compiler which pin is input and which is output.
- **`PinDirection`** — `Input`, `Output`, `Up` (toward VCC / B+), `Down` (toward GND), `Bidirectional`. Used by the layout engine.
- **`GraphRole`** — `Edge`, `ActiveEdge`, `VcvsEdge`, `Pot`, `Virtual`. Tells the graph builder how to wire the component up.
- **`NonIdealFx`** — `OpAmpBandwidth`, `RailSaturation`. Extensible enum for post-processing effects.

The agent-written inline rustdoc at the top of `component.rs` is the source of truth for exact variants; the list above is a map, not a spec.

## Pots are ordinary components

A `Potentiometer` implements `Component` like anything else. The differences are three returns:

- `graph_role()` returns `GraphRole::Pot` rather than `Edge`. The graph builder knows to expose three terminals (`a`, `w`, `b`) instead of two.
- `ports()` returns `[(a, w), (w, b)]` — two sub-edges, not one.
- `is_variable()` returns `true`. Stages that hold variable components register for recompute when the pot is moved.

`controls()` returns a single `ControlParam { name: "position", kind: ControlParamKind::PotPosition }`. That's what the binding pass in `spqr_control.rs` wires to the user-visible control name. The runtime update path — how moving a pot propagates through port resistances, opamp gain, and post-processing — is a separate story covered in [Controls and pots](./controls-and-pots.md).

## Extending externally

The `Component` trait is public-reachable:

```rust
pub use compiler::{Component, PinDirection};
```

In principle a downstream crate can `impl Component for MyStruct`. In practice it's awkward for two reasons. First, `impl_component_dyn!()` is `pub(crate)`, so external implementations have to write the four trait-object methods manually. Second, there is no external registry — even if you implement the trait, the DSL parser won't know about your component unless it's in the `component_kind()` `alt`.

The honest design premise is "components are in-tree." If you have a component you want pedalkernel to support, open a PR; it is a fixed amount of mechanical work per device.

## Testing conventions

Tests live next to the implementation, in `component.rs` for the trait and in the per-family files for concrete types. Typical patterns:

- **Trait-object tests** — clone, downcast, `PartialEq` for `Box<dyn Component>`.
- **Edge declaration** — assert `edges()` returns the expected `EdgeKind`.
- **Resolution** — test context-dependent edge changes (JFETs that become linear under modulation).
- **Classification** — verify `classify_nonlinear` returns the right kind and junction nodes.

One test per method per component type is a good baseline. Tests verify the trait contract — what the compiler will see — not implementation details. Example from `components/passives.rs`:

```rust
#[test]
fn edges_linear_resistor() {
    let r = Resistor { value: 1000.0 };
    let edges = r.edges();
    assert_eq!(edges.len(), 1);
    assert_eq!(edges[0].kind, EdgeKind::Linear);
}
```

## Stability caveats

A few pieces of this surface are in motion:

- **`resolve_edges()` + `ResolveContext`** are recent additions to handle components whose edge kind depends on wiring (modulated JFETs as variable resistors, OTAs with current-control, etc.). The context struct may pick up more fields.
- **`NonIdealFx` is extensible.** Thermal, supply-sag, and grid-current variants are commented as placeholders. The post-processing stage builder may need refactoring as these land.
- **`SignalTerminals::Amplifier { control }`** currently only handles single control pins. Multi-CV inputs (VCAs with multiple CV) may prompt a shape change.

If you're adding a new component type and one of these edges trips, start a discussion on the PR rather than working around it silently — the refactor to unblock you is probably worth doing.
