# Studio DSP architecture

## Ownership

- **Synesthesia (Pi):** device graph, named routes, transport, presets, and the
  bounded Lua planning API. An LLM may translate “warm piano through the WA-2A”
  into checked graph commands, but never participates in an audio callback.
- **Isochrone:** bidirectional timestamped PCM transport and clock recovery. It
  exposes named send/receive endpoints; PedalKernel does not duplicate RTP,
  jitter buffering, or ASRC.
- **PedalKernel (Mac Studio):** VST3/VSTi discovery and hosting, plugin state,
  sample-accurate MIDI/automation, CoreAudio callbacks, and latency reporting.

The JSON `HostCommand` schema in `pedalkernel-core` is the control boundary.
Synesthesia should invoke a PedalKernel control socket/client rather than link to
plugin-host internals. Audio endpoint names are opaque and resolved by their
owner.

## Signal paths

VST instrument:

```
controller MIDI -> Synesthesia -> PedalKernel control/event queue
                                 VSTi -> CoreAudio or Isochrone -> Pi/18i20
```

Remote effect:

```
Pi source -> Isochrone send -> PedalKernel VST3 chain
          <- Isochrone receive <- compensated wet output
```

Physical insert on the Pi:

```
Mac plugin/source -> Isochrone -> Pi 18i20 send -> hardware -> 18i20 return
                  <- Isochrone <- Pi capture
```

Calibration sends an impulse or chirp through the complete route, records the
return, and stores the measured frame offset with device identities, sample
rate, and route topology. A change to any of those invalidates the measurement.
Dry/parallel paths are delayed to the measured round trip; the wet return is not
advanced or guessed from nominal buffer sizes.

## Realtime rules

The audio callback performs no allocation, locking, filesystem access, network
I/O, logging, JSON, Lua, or plugin scanning. Commands arrive through a bounded
single-producer/single-consumer queue and carry block-relative sample offsets.
Immutable graph/state snapshots are prepared on the control thread and swapped
at block boundaries. Queue overflow and plugin failure are visible errors; they
do not silently bypass.

## Host implementation milestones

1. macOS VST3 discovery and class metadata cache, explicitly refreshed.
2. One in-process instrument with CoreAudio output and timestamped MIDI.
3. One effect fed by an Isochrone receive endpoint and returned through an
   Isochrone send endpoint.
4. Plugin state save/restore and deterministic parameter enumeration.
5. Hardware-loop calibration, persisted measurement identity, and dry-path
   compensation.
6. Crash isolation as a separate worker process after the callback contract is
   proven; never hide plugin crashes with an implicit bypass.

## Manual runbook

1. Start Isochrone receive/send endpoints and verify their sample rate/channel
   layout matches the session.
2. Start PedalKernel, configure the session, load a VST3 class, then connect the
   named endpoints.
3. Send a MIDI note or known audio test tone and verify local processing.
4. For hardware, patch the declared 18i20 send/return and run calibration.
5. Confirm the measured round-trip value and a phase-coherent dry/wet null test.
6. Enable the route in Synesthesia. Stop in reverse order.

