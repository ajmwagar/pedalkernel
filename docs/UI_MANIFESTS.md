# Pedal UI manifests (FOSS)

This repo keeps `.pedal` files **circuit-only**.

- `.pedal` describes the signal path (components + wiring) and the parameter mapping used by the audio engine.
- `.pedalhw` (used in the pro toolchain) describes hardware constraints and *skin* / finish metadata.

For the VST/UI layer we also maintain **non-secret UI manifests** as JSON under:

- `pedalkernel-vst/ui/manifests/`

These manifests are intended to be safe to ship publicly and to be stable inputs for UI generation.

## Why manifests exist

The circuit description (`.pedal`) already contains control names/ranges, but UI generation needs additional, presentation-oriented data:

- a stable pedal slug/id
- faceplate enclosure preset (e.g. 125B portrait)
- the ordered list of knobs/switches with short labels
- optional styling hints (knob family/color, LED hint, label rendering style)

## How the pro pipeline uses this

In the pro pipeline (private), generation typically combines:

1. `.pedal` (circuit + parameter mapping)
2. `.pedalhw` (hardware/skin metadata)
3. `ui` manifest from this repo (control surface list + optional style hints)
4. internal assets (brand fonts, licensed graphics, etc.)

…to produce final, branded skins.

The manifests in this repo are intentionally conservative: they avoid secrets and licensed art.

## Schema

The JSON Schema lives at:

- `pedalkernel-vst/ui/ui_manifest.schema.json`

Each manifest should include `"$schema"` pointing to the relative schema path.

## Free pedal manifests

Free pedal manifests live under:

- `pedalkernel-vst/ui/manifests/free/`
