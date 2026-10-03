# VST editor surfaces

PedalKernel captures a plugin editor as a retained native frame. Capture is
separate from presentation and transport: Canvas can import a local frame,
while a remote-display adapter can submit the same frame to a hardware encoder.
No PNG, JPEG, temporary file, or browser reload sits in this path.

The stable contract lives in `pedalkernel-editor-surface` and reports only
portable metadata. Native handles remain process-local objects:

- macOS 14 and newer uses a ScreenCaptureKit stream. Its serial callback keeps
  a bounded latest-frame queue, and every published frame retains its
  IOSurface for direct Metal/WGPU import or VideoToolbox submission. A slow
  consumer drops stale frames instead of blocking capture or accumulating
  latency.
- Linux under X11 or XWayland uses XComposite to obtain the editor pixmap,
  copies each complete presentation into a snapshot pixmap, then exports that
  snapshot through DRI3 as an owned dma-buf. This avoids sampling a pixmap while
  the plugin is repainting it.

DRI3 v1 does not report a DRM modifier. The descriptor therefore reports
`modifier: null`; an importer must negotiate or derive the layout and must not
silently assume linear memory. Native Wayland plugins will need a compositor or
portal producer, but XWayland-hosted VST3 editors use the existing Linux path.

## Host runbook

The editor must be open before capture starts. The title selector is combined
with the host PID, so another application's similarly named window cannot be
captured.

```text
pedalkernel-studio-ctl editor-open surge
pedalkernel-studio-ctl surface-start surge "Surge XT" 30
pedalkernel-studio-ctl surface-status
pedalkernel-studio-ctl surface-stop surge
pedalkernel-studio-ctl editor-close surge
```

macOS asks the signed host application for Screen Recording permission once.
Permission denial is returned by `surface-start`; periodic capture failures are
visible in `surface-status` and stderr without stopping plugin audio.

Installed macOS hosts must keep a stable signing identity and identifier across
rebuilds. The linker's ad-hoc signature is tied to the binary hash, so replacing
an ad-hoc-signed host makes TCC treat it as a new capture client. A deployment
can sign the final artifact without baking a developer identity into this repo:

```text
codesign --force --timestamp \
  --sign "$PEDALKERNEL_CODESIGN_IDENTITY" \
  --identifier dev.futurepresentlabs.pedalkernel-studio-host \
  pedalkernel-studio-host
```

The host retains the previous frame until a complete replacement has arrived.
This invariant is also required of downstream presenters: import or enqueue the
new frame first, swap it into the displayed slot, and only then release the old
lease.
