---
title: "Roadmap"
description: "Where the engine is going next."
section: "Project"
weight: 110
source_commit: "222212fe33f0aa223f7d545d890a16654c17cca8"
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

**Bidirectional cross-network coupling.** The [cross-network coupling](./compiler-internals.md#cross-network-coupling) layer currently injects donor-stage voltages into recipient-stage incident waves one sample late — back-action from the recipient to the donor lags by `1/fs`. For most audio circuits this is inaudible, but for tight servo loops (high-feedback opto-compressors, push-pull pairs that share a phase inverter) the next refinement is delay-free bidirectional coupling — likely a Newton iteration across the network boundary inside one sample.

**Transformer-flux metering.** The metering `transformer_flux[4]` field has been a stub since the metering layer landed. The Jiles-Atherton transformer model now carries `M` (magnetisation) and `B` (flux density) per sample as part of `JaState`. Wiring those into the metrics ring buffer for a transformer-saturation indicator is mechanical and shovel-ready.

**Faithful LA-2A and Pultec EQP-1A.** Both circuits exist in the tree but at different fidelity levels. The LA-2A 1:1 rebuild has the right topology (faithful 4-terminal transformers, detector-tap de-fusion). The Pultec EQP-1A has an ngspice baseline for its passive EQ section. Closing the remaining gaps — feedback-loop convergence on LA-2A masks, validated harmonic spectra on Pultec — is the next milestone for the outboard-gear coverage line.

## Coverage

**More circuits.** There is no fixed target list — we merge new `.pedal` files whenever they pass SPICE validation. Contributions of well-documented vintage circuits are especially welcome.

**Hardware specs (`.pedalhw`).** Voltage-rating and part-number metadata exists for some circuits but is not systematically applied across the library.

## Non-goals

A few directions we have deliberately chosen not to take:

- **Neural / black-box amp models.** PedalKernel is circuit-exact. If a circuit can't be modeled component-by-component, we would rather leave it out than fit a network to it.
- **Binary plugin distribution.** The kernel is a Rust library. VST / AU packaging is left to downstream consumers.
- **Proprietary circuits.** Only publicly documented schematics belong in the open-source tree.
