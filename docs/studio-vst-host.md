# Studio VST host

PedalKernel is the Mac Studio realtime host for VST3 instruments/effects and
its own circuit DSP. Synesthesia owns named routes and deterministic
orchestration. Isochrone owns bidirectional timestamped PCM transport and clock
recovery. The host does not duplicate either service.

## Boundaries

The versioned `pedalkernel-host-protocol` messages are the control boundary.
Synesthesia talks to a PedalKernel control socket rather than linking against
host internals. An LLM or bounded Lua plugin may produce checked commands on the
control plane; neither runs in an audio callback.

```
controller MIDI -> Synesthesia -> PedalKernel -> VSTi
Pi audio -> Isochrone receive -> PedalKernel VST/circuit chain
Pi audio <- Isochrone send    <- PedalKernel output
```

For a physical insert connected to the Pi's 18i20:

```
Mac source -> Isochrone -> Pi send -> hardware -> Pi return -> Isochrone -> Mac
```

`pedalkernel-fx-loop` owns calibration capture. A calibration sends a known
impulse/chirp through the complete route and records the measured round trip.
The `HardwareInsert` contract stores that measurement in frames together with
its artifact identity. Any device, sample-rate, channel, or topology change
invalidates it. The dry/parallel path is delayed by that measured amount; the
wet return is never “advanced” or guessed from nominal buffer sizes.

## Realtime contract

The callback performs no allocation, locking, filesystem access, network I/O,
logging, JSON, Lua, or plugin scanning. Commands cross a bounded SPSC queue with
block-relative sample offsets. Immutable graph snapshots are prepared on the
control thread and swapped only at block boundaries. Queue overflow or plugin
failure is a visible error, not a silent bypass.

## Implementation sequence

1. macOS VST3 discovery and class metadata cache with explicit refresh.
2. One in-process VSTi with CoreAudio output and timestamped MIDI.
3. One effect using named Isochrone receive/send endpoints.
4. Plugin state save/restore and deterministic parameter enumeration.
5. Extend `pedalkernel-fx-loop` to emit a measured latency artifact, then apply
   dry-path compensation and verify it with a phase/null test.
6. Add crash isolation as a worker process after the callback contract is
   proven; do not hide a crashed plugin behind implicit bypass.

## Manual runbook

1. Start Isochrone endpoints and verify sample rate and channel layout.
2. Start PedalKernel, configure a session, load a VST3 class, and connect the
   named endpoints.
3. Send a known MIDI note or test tone and verify local processing.
4. Patch the declared 18i20 send/return and run `pedalkernel-fx-loop` capture.
5. Register the measured frames and verify dry/wet alignment with a null test.
6. Enable the route in Synesthesia. Stop the route in reverse order.

## Control shim

`pedalkernel-studio-host` is the headless VST3 control shim. It listens on
`127.0.0.1:9473` by default and accepts one JSON request per line. Keep it on
loopback and use an SSH tunnel from Synesthesia; the daemon rejects LAN peers.
Pass an Isochrone destination as the second argument to render directly into
Isochrone's own RTP packet sender, avoiding a virtual CoreAudio loopback. Omit
it to use the default CoreAudio output. The shim calls Isochrone as a library;
it does not duplicate its wire format, sequencing, or receiver policy.

Configure before discovery or loading. Every request has a caller-selected
`request_id`, echoed in its response. The initial runtime intentionally permits
one active VST3 instance per process; run another process/port for isolation.

```sh
cargo run -p pedalkernel-studio-host
printf '%s\n' \
  '{"request_id":"1","command":"configure","protocol":1,"session":{"id":"surge","sample_rate_hz":48000,"block_frames":128,"input_channels":0,"output_channels":2}}' \
  '{"request_id":"2","command":"discover"}' \
  | nc 127.0.0.1 9473
```

For the home-studio Pi:

```sh
pedalkernel-studio-host 127.0.0.1:9473 192.168.2.74:50040
```
