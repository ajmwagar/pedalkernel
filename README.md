# PedalKernel

PedalKernel is the macOS realtime host for VST3 instruments and effects in the
FPL studio. Synesthesia owns intent and deterministic orchestration. Isochrone
owns network audio. PedalKernel owns plugin discovery, lifecycle, parameter
automation, sample-accurate MIDI, and processing on the Mac Studio.

The initial scaffold contains the versioned control contract and tested
hardware-loop latency primitives. It does **not** yet load or execute VST3
bundles; unsupported runtime operations must fail loudly until the host exists.

See [docs/architecture.md](docs/architecture.md) for the boundary and deployment
plan.

