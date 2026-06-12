---
title: "Roadmap"
description: "Where the engine is going next."
section: "Project"
weight: 110
source_commit: "ba0372ed07318273d8d1a016ca9a572acc0a27df"
---

# Roadmap

PedalKernel is under active development. This page tracks the significant work items we want to take on next. It is intentionally short — the [modeling limits](./modeling-limits.md) page is a fuller accounting of known gaps.

## Modeling frontier

These are the largest open questions in the WDF engine.

**Op-amp closed-loop dynamics inside the WDF tree.** Feedback topology and gain are now derived from the circuit graph (see the [modeling limits](./modeling-limits.md) page), but gain-bandwidth product, slew rate, and rail saturation still run as a post-processing layer on top of each stage rather than as scattering matrix entries. For most guitar circuits this is inaudible; for precision topologies (servo loops, active filters where GBW is a design variable) it is the next accuracy win available.

**ADAA-LUT: antialiased nonlinear roots.** The [K-method tables](./compiler-internals.md#k-method-tables) currently store `a_root = f(b_tree)` directly, so they alias the same way Newton-Raphson does at large signal levels. Antiderivative antialiasing (ADAA) reduces that aliasing by integrating the nonlinearity over the sample interval rather than sampling it pointwise — and the LUT variant precomputes both `f` and its antiderivative `F` so the per-sample cost is two table lookups and a divide. Layering ADAA-LUT on top of the existing K-method pipeline would let high-gain stages run at audio rate without the 4× oversampling we currently fall back to.

**Power supply sag across a pedalboard.** Single pedals can opt into a supply model (see the `supplies` block in the DSL). A shared 9 V supply sagging across several pedals drawing current together is not yet modeled.

**Speaker and cabinet physics.** The chain currently ends at the pedal output. Thiele-Small driver parameters, voice-coil compression, cone breakup, and cabinet resonance are all missing, and matter for full-rig simulation.

**Oversampling latency reporting.** Oversampling adds latency that a DAW host needs to know about for dry/wet alignment. The oversampling module does not yet report it.

**3D K-tables for pentodes.** Pentodes report `(true, 3)` from `k_method_candidacy()`, but the table generator currently emits 1D and 2D only. Adding the 3D path (`b_tree` × `Vg1k` × `Vg2k`) would extend the lookup fast path to power tubes.

## Coverage

**More circuits.** There is no fixed target list — we merge new `.pedal` files whenever they pass SPICE validation. Contributions of well-documented vintage circuits are especially welcome.

**Hardware specs (`.pedalhw`).** Voltage-rating and part-number metadata exists for some circuits but is not systematically applied across the library.

## Non-goals

A few directions we have deliberately chosen not to take:

- **Neural / black-box amp models.** PedalKernel is circuit-exact. If a circuit can't be modeled component-by-component, we would rather leave it out than fit a network to it.
- **Binary plugin distribution.** The kernel is a Rust library. VST / AU packaging is left to downstream consumers.
- **Proprietary circuits.** Only publicly documented schematics belong in the open-source tree.
