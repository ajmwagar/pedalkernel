---
title: "Roadmap"
description: "Where the engine is going next."
section: "Project"
weight: 110
---

# Roadmap

PedalKernel is under active development. This page tracks the significant work items we want to take on next. It is intentionally short — the [modeling limits](./modeling-limits.md) page is a fuller accounting of known gaps.

## Modeling frontier

These are the largest open questions in the WDF engine.

**Op-amp feedback topology.** Op-amps are currently modeled as per-device gain multipliers with slew-rate and rail-saturation. The inverting / non-inverting feedback network, virtual-ground behaviour, and frequency-dependent open-loop response are not yet part of the WDF tree. Closing this gap is the single biggest accuracy win available.

**BJT gain stages as WDF roots.** JFETs, MOSFETs, triodes, pentodes, diodes, and zeners all have true WDF root elements. BJTs currently contribute a per-device gain factor without their own root — so CE/CB/CC topology, collector load, and emitter degeneration collapse into a scalar. A `BjtRoot` would make Fuzz Face, Big Muff, and Tonebender behave correctly under bias-point shifts.

**Power supply sag across a pedalboard.** Single pedals can opt into a supply model (see the `supplies` block in the DSL). A shared 9 V supply sagging across several pedals drawing current together is not yet modeled.

**Speaker and cabinet physics.** The chain currently ends at the pedal output. Thiele-Small driver parameters, voice-coil compression, cone breakup, and cabinet resonance are all missing, and matter for full-rig simulation.

**Oversampling latency reporting.** Oversampling adds latency that a DAW host needs to know about for dry/wet alignment. The oversampling module does not yet report it.

## Coverage

**More circuits.** There is no fixed target list — we merge new `.pedal` files whenever they pass SPICE validation. Contributions of well-documented vintage circuits are especially welcome.

**Hardware specs (`.pedalhw`).** Voltage-rating and part-number metadata exists for some circuits but is not systematically applied across the library.

## Non-goals

A few directions we have deliberately chosen not to take:

- **Neural / black-box amp models.** PedalKernel is circuit-exact. If a circuit can't be modeled component-by-component, we would rather leave it out than fit a network to it.
- **Binary plugin distribution.** The kernel is a Rust library. VST / AU packaging is left to downstream consumers.
- **Proprietary circuits.** Only publicly documented schematics belong in the open-source tree.
