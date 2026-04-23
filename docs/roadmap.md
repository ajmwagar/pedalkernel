---
title: "Roadmap"
description: "Where the engine is going next."
section: "Project"
weight: 110
source_commit: "95744ce1cdd9c2cdec3550bfdce9879b1737312c"
preview: true
---

# Roadmap

> **Preview.** This page reflects the state once the `feature/spqr-tree` branch lands. Items that have already been closed on that branch (op-amp feedback topology extraction, BJT WDF roots, and others) have been removed from this list. On `main` today, some of those items are still in progress.

PedalKernel is under active development. This page tracks the significant work items we want to take on next. It is intentionally short — the [modeling limits](./modeling-limits.md) page is a fuller accounting of known gaps.

## Modeling frontier

These are the largest open questions in the WDF engine.

**Op-amp closed-loop dynamics inside the WDF tree.** Feedback topology and gain are now derived from the circuit graph (see the [modeling limits](./modeling-limits.md) page), but gain-bandwidth product, slew rate, and rail saturation still run as a post-processing layer on top of each stage rather than as scattering matrix entries. For most guitar circuits this is inaudible; for precision topologies (servo loops, active filters where GBW is a design variable) it is the next accuracy win available.

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
