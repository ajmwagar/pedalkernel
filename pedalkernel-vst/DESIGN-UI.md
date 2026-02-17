# PedalKernel VST UI Design: Photorealistic Pedal Interface

## Goal

Replace the current host-generic parameter UI with an embedded, photorealistic
pedal GUI -- the kind you see in GarageBand's Pedalboard, Neural DSP plugins, or
Arturia's effects. Real knobs with specular highlights, powder-coated enclosures,
screw heads, LED jewels, rubber feet. Change any pedal and the entire visual
identity swaps: different enclosure, different knobs, different labelling.

---

## Current State

- **No embedded GUI.** The VST plugin exposes 7 `nih_plug` parameters (1 pedal
  selector + 6 generic knobs). The DAW renders them as generic sliders.
- **No `editor()` implementation.** The `Plugin` trait's `editor()` method is
  not overridden, so nih-plug returns `None`.
- **TUI exists for JACK mode.** Ratatui-based ASCII knob widgets exist in
  `pedalkernel/src/cli/tui_widgets.rs`, but this is terminal-only.

---

## Architectural Decision: How to Render

### Options Considered

| Approach | Pros | Cons |
|----------|------|------|
| **nih_plug_egui** (immediate-mode) | Free param binding, layout engine, image support | Not designed for bitmap-heavy skeuomorphic UIs; every frame redraws all widgets |
| **nih_plug_vizia** (declarative) | CSS-like styling, accessibility, maintained by nih-plug author | Better for clean/modern UIs; bitmap compositing is a workaround, not first-class |
| **nih_plug_iced** (Elm-style) | Declarative, wgpu used to be supported | wgpu backend was removed; glow-only; limited bitmap support |
| **BYO GUI: baseview + wgpu** | Full GPU control, texture atlases, custom shaders | More code to write; macOS window lifecycle issue (baseview #124) |
| **BYO GUI: baseview + glow (OpenGL)** | Widest DAW compatibility, mature, filmstrip pattern is trivial | OpenGL is legacy on macOS (deprecated); still plenty capable |
| **BYO GUI: baseview + softbuffer (CPU)** | Zero GPU issues, just blit pixels | CPU-bound; fine for static compositing, struggles with animations |
| **WebView (nih-plug-webview)** | HTML/CSS/JS, fastest iteration, easy bitmap layouts | Experimental; extra process/IPC overhead; platform gaps |

### Recommendation: Hybrid egui + Texture Atlas

**Primary:** Use `nih_plug_egui` with pre-rendered texture assets. egui supports
loading PNG textures, custom painting via `egui::Painter`, and gives us
parameter synchronization for free. The photorealistic elements (enclosure,
knobs, LEDs) are all pre-rendered bitmaps -- egui just composites them.

**Fallback / Future:** If egui's immediate-mode overhead or rendering quality
becomes limiting, graduate to a BYO GUI with wgpu. The asset pipeline and data
model are identical either way; only the rendering backend changes.

**Why not pure BYO from day one?** Parameter binding, DPI scaling, mouse hit
testing, and window lifecycle are all solved problems in `nih_plug_egui`. Writing
those from scratch for a BYO GUI is 2-3x more code for no visual benefit. The
visual quality comes from the *assets*, not the renderer.

---

## Asset Pipeline: From 3D Model to Plugin

This is how professional plugins (Neural DSP, Waves, Arturia) produce their UIs.
PedalKernel should follow the same proven pipeline.

### Step 1: 3D Modeling (Blender)

Model each component type as a reusable Blender asset:

```
assets/blender/
  enclosures/
    1590b.blend          # Hammond 1590B (standard pedal size)
    1590bb.blend         # 1590BB (larger, Big Muff size)
    1590a.blend          # 1590A (mini pedal)
    1590xx.blend         # 1590XX (double-wide)
  knobs/
    chicken_head.blend   # Classic chicken-head pointer knob
    davies_1900h.blend   # Davies 1900H clone (Boss-style)
    mxr_knob.blend       # MXR-style large fluted knob
    rat_knob.blend       # ProCo RAT "toothpaste cap" knob
    small_pointer.blend  # Small pointer for trim pots
    soft_touch.blend     # Rubber soft-touch (modern boutique)
  switches/
    3pdt_footswitch.blend
    toggle_spdt.blend
  leds/
    5mm_jewel.blend      # Standard LED with chrome bezel
    3mm_bare.blend       # Bare 3mm LED
  hardware/
    input_jack.blend     # 1/4" mono jack
    output_jack.blend    # 1/4" mono jack
    dc_jack.blend        # 2.1mm barrel jack
    thumbscrew.blend     # Battery compartment
    rubber_foot.blend    # Adhesive rubber foot
```

### Step 2: Texturing & Materials (PBR)

Each enclosure gets a unique material set:

| Pedal | Enclosure Color | Finish | Knob Style | LED Color |
|-------|----------------|--------|------------|-----------|
| Tube Screamer | Ibanez green (#2E7D32) | Matte powder coat | Plastic pointer (cream) | Red |
| Fuzz Face | Hammertone grey | Textured metal | Chicken head (black) | None (vintage) |
| Big Muff | Silver/black graphic | Matte enamel | MXR-style (black) | Red |
| Dyna Comp | Red (#D32F2F) | Gloss powder coat | MXR-style (black) | Red |
| ProCo RAT | Black | Matte powder coat | RAT "toothpaste" (white) | Red |
| Blues Driver | Blue (#1565C0) | Matte powder coat | Davies 1900H (black) | Red |
| Klon Centaur | Gold (#C5A03F) | Hammertone metal | Chicken head (gold) | None |

Material properties:
- **Powder coat**: Diffuse-dominant, slight roughness variation, very subtle
  orange-peel normal map
- **Brushed aluminum**: Anisotropic specular, directional roughness
- **Chrome**: Mirror-like specular, environment-mapped reflections
- **Rubber (knob grip)**: High roughness, subtle bump map
- **Acrylic (LED jewel)**: Subsurface scattering, colored transmission

### Step 3: Lighting Rig

Fixed 3-point lighting with HDRI environment:
- **Key light**: 45deg above, slightly right. Warm white (5500K).
- **Fill light**: Left side, cooler (6500K), 40% intensity of key.
- **Rim light**: Behind and above, for edge separation.
- **HDRI**: Neutral studio environment for realistic reflections on chrome.

Critical: lights are **fixed in world space**. When rendering knob filmstrips,
the knob rotates but lighting stays put. This is what makes highlights look real.

### Step 4: Rendering Filmstrips

For each knob variant, render a rotation sequence:

```
Frames:        128 (or 256 for smooth feel)
Rotation arc:  270 degrees (7 o'clock to 5 o'clock, standard pot range)
Resolution:    128x128 px per frame (256x256 for Retina/HiDPI)
Format:        PNG with alpha channel
Output:        Single vertical filmstrip (128 frames stacked = 128x16384 px)
```

Render each knob style x each color variant:
```
assets/filmstrips/
  chicken_head_black_128x128_128f.png
  chicken_head_gold_128x128_128f.png
  davies_1900h_black_128x128_128f.png
  mxr_knob_black_128x128_128f.png
  rat_knob_white_128x128_128f.png
  pointer_cream_128x128_128f.png
  soft_touch_black_128x128_128f.png
```

Tool options for filmstrip generation:
- **Blender** (Python scripting): Automate camera, rotation, render per frame
- **KnobMan3D**: Free tool specifically for knob filmstrip generation
- **Cinema 4D**: Industry standard but expensive

### Step 5: Enclosure Backgrounds

Render each pedal's complete enclosure (without knobs) as a single image:

```
assets/enclosures/
  tube_screamer_bg.png     # 400x700 px (or @2x: 800x1400)
  fuzz_face_bg.png
  big_muff_bg.png
  dyna_comp_bg.png
  proco_rat_bg.png
  blues_driver_bg.png
  klon_centaur_bg.png
  delay_bg.png
  generic_bg.png           # Fallback for user-loaded .pedal files
```

Include in the background render:
- Enclosure body with color/finish
- Silkscreen text (pedal name, control labels, manufacturer logo)
- Screw heads (corner screws)
- Input/output/DC jacks (side or top)
- Footswitch (3PDT stomp)
- Rubber feet (bottom corners, partially visible)
- **No knobs** -- those are composited at runtime

### Step 6: LED States

Render LED on/off as separate small images:
```
assets/leds/
  led_red_on.png       # 24x24, with glow halo
  led_red_off.png      # 24x24, dark jewel
  led_green_on.png
  led_green_off.png
  led_amber_on.png
  led_amber_off.png
```

---

## Runtime Architecture

### Data Model: PedalSkin

Each pedal ships with (or can load) a skin definition:

```rust
/// Visual description of a single pedal's GUI.
struct PedalSkin {
    /// Background enclosure image (PNG, pre-rendered).
    background: TextureId,

    /// Total GUI dimensions (logical pixels, before DPI scaling).
    size: (u32, u32),

    /// Knob visual definitions, one per active control.
    knobs: Vec<KnobSkin>,

    /// LED indicator position and color.
    led: Option<LedSkin>,

    /// Bypass footswitch hit area.
    footswitch_rect: Rect,
}

struct KnobSkin {
    /// Which filmstrip texture to use.
    filmstrip: TextureId,
    /// Number of frames in the filmstrip.
    frame_count: u32,
    /// Size of each frame in the filmstrip (px).
    frame_size: (u32, u32),
    /// Position on the enclosure (center point, logical px).
    position: (f32, f32),
    /// Which parameter index this knob controls (0-5).
    param_index: usize,
    /// Display label (e.g., "Drive", "Level").
    label: String,
}

struct LedSkin {
    on_texture: TextureId,
    off_texture: TextureId,
    position: (f32, f32),
}
```

### Skin Definitions

Skins can be defined in a companion `.pedalskin` file or embedded in the binary
for built-in pedals. Example format (TOML):

```toml
[skin]
background = "tube_screamer_bg.png"
size = [400, 700]

[[knobs]]
filmstrip = "pointer_cream_128x128_128f.png"
frames = 128
frame_size = [128, 128]
position = [140, 320]
param = 0
label = "Drive"

[[knobs]]
filmstrip = "pointer_cream_128x128_128f.png"
frames = 128
frame_size = [128, 128]
position = [260, 320]
param = 1
label = "Level"

[led]
on = "led_red_on.png"
off = "led_red_off.png"
position = [200, 180]

[footswitch]
rect = [160, 580, 80, 80]
```

### Hot-Swapping Knobs and Backgrounds

**Yes, this is straightforward with the filmstrip approach.** When the user
changes the pedal selector:

1. Read the new `pedal_type` parameter value.
2. Look up the `PedalSkin` for that pedal index.
3. All textures are pre-loaded at `editor()` creation time (they're small PNGs).
4. The next frame renders with the new skin's background and knob filmstrips.

There is **no recompilation, no shader swap, no texture upload delay.** The
textures are already in GPU memory (or CPU memory for softbuffer). Swapping is
just changing which texture ID and filmstrip frame offset to sample.

For user-loaded `.pedal` files without a custom skin, use the `generic_bg.png`
background with a standard knob filmstrip. The generic skin auto-positions knobs
based on how many controls the pedal defines (1-6 knobs in a grid layout).

### Rendering Loop (egui approach)

```rust
fn editor(&self, ...) -> Option<Box<dyn Editor>> {
    // Use nih_plug_egui::create_egui_editor
}

fn update(&self, egui_ctx: &egui::Context, setter: &ParamSetter, state: &mut EditorState) {
    let pedal_idx = self.params.pedal_type.value() as usize;
    let skin = &self.skins[pedal_idx];

    egui::CentralPanel::default().show(egui_ctx, |ui| {
        // 1. Draw enclosure background
        ui.image(skin.background, skin.size);

        // 2. Draw each knob at its position
        for knob in &skin.knobs {
            let value = self.params.knobs[knob.param_index].value();
            let frame = (value * (knob.frame_count - 1) as f32) as usize;

            let uv_top = frame as f32 / knob.frame_count as f32;
            let uv_bottom = (frame + 1) as f32 / knob.frame_count as f32;

            // Draw filmstrip frame at knob position
            // (actual egui API uses Image::new().uv() for sub-rect sampling)
            draw_filmstrip_frame(ui, knob.filmstrip, knob.position, frame);

            // 3. Handle mouse drag on knob area
            let knob_rect = Rect::from_center_size(knob.position, knob.frame_size);
            let response = ui.allocate_rect(knob_rect, Sense::drag());
            if response.dragged() {
                let delta = -response.drag_delta().y / 200.0;
                let new_val = (value + delta).clamp(0.0, 1.0);
                setter.set_parameter(&self.params.knobs[knob.param_index], new_val);
            }
        }

        // 4. Draw LED
        if let Some(led) = &skin.led {
            let bypassed = /* check bypass state */;
            let tex = if bypassed { led.off_texture } else { led.on_texture };
            ui.put(led_rect, egui::Image::new(tex));
        }

        // 5. Draw knob labels below each knob
        for knob in &skin.knobs {
            let label_pos = (knob.position.0, knob.position.1 + knob.frame_size.1 / 2.0 + 12.0);
            ui.put(label_rect, egui::Label::new(&knob.label));
        }
    });
}
```

### Interaction Model

| Action | Behavior |
|--------|----------|
| **Drag knob up/down** | Increase/decrease parameter value (vertical drag, industry standard) |
| **Scroll on knob** | Fine-tune parameter (smaller increments) |
| **Double-click knob** | Reset to default value |
| **Ctrl+click knob** | Type exact value |
| **Click footswitch** | Toggle bypass (LED on/off) |
| **Right-click knob** | Context menu: MIDI learn, automation, reset |
| **Change pedal selector** | Entire skin swaps (background + knobs + LED + layout) |

---

## File Structure (New)

```
pedalkernel-vst/
  Cargo.toml                    # Add: nih_plug_egui, image, serde, toml
  src/
    lib.rs                      # Add: editor() implementation
    editor.rs                   # GUI editor module
    editor/
      mod.rs                    # Editor state, texture loading
      skin.rs                   # PedalSkin, KnobSkin, LedSkin structs
      skin_loader.rs            # Parse .pedalskin TOML files
      knob_widget.rs            # Filmstrip knob rendering + interaction
      led_widget.rs             # LED on/off rendering
      pedal_view.rs             # Main pedal view compositor
      texture_atlas.rs          # Texture loading and caching
  assets/
    skins/
      tube_screamer.toml
      fuzz_face.toml
      big_muff.toml
      dyna_comp.toml
      proco_rat.toml
      blues_driver.toml
      klon_centaur.toml
      delay.toml
      generic.toml              # Fallback skin for user pedals
    enclosures/
      tube_screamer_bg.png
      fuzz_face_bg.png
      ...
      generic_bg.png
    filmstrips/
      chicken_head_black.png
      davies_1900h_black.png
      mxr_knob_black.png
      pointer_cream.png
      rat_knob_white.png
      soft_touch_black.png
    leds/
      led_red_on.png
      led_red_off.png
      led_green_on.png
      led_green_off.png
```

---

## Dependency Changes

```toml
# pedalkernel-vst/Cargo.toml additions:

[dependencies]
nih_plug = { git = "https://github.com/robbert-vdh/nih-plug.git", features = ["egui"] }
nih_plug_egui = { git = "https://github.com/robbert-vdh/nih-plug.git" }
image = { version = "0.25", default-features = false, features = ["png"] }
serde = { version = "1", features = ["derive"] }
toml = "0.8"
```

---

## DPI / HiDPI Strategy

- All skin coordinates are in **logical pixels**.
- Assets are shipped at **2x resolution** (e.g., 256x256 per knob frame).
- At 1x DPI, downsample. At 2x DPI (Retina), render 1:1.
- egui handles DPI scaling via `egui::Context::pixels_per_point()`.
- Enclosure backgrounds: ship @1x and @2x, select at load time.

---

## Performance Budget

| Component | Cost |
|-----------|------|
| Background blit | 1 texture draw call |
| Per knob | 1 filmstrip sub-rect blit |
| LED | 1 small texture blit |
| Labels | Text rendering (cached by egui) |
| **Total per frame** | ~10-15 draw calls for a 6-knob pedal |
| **Target frame rate** | 60 FPS (vsync) |
| **GPU memory** | ~20-50 MB for all filmstrips + enclosures |

This is trivially within budget. Professional plugins with far more complex UIs
(channel strips, mixer views) run at 60fps on integrated GPUs.

---

## Can Knobs and Backgrounds Be Changed on the Fly?

**Yes.** This is a first-class feature of the filmstrip architecture:

1. **Pedal selector changes** -> swap entire `PedalSkin` -> next frame renders
   the new pedal. All textures are pre-loaded, so the swap is instantaneous (one
   pointer swap, zero allocations).

2. **User-defined skins** -> users drop a `.pedalskin` TOML + PNG assets into
   `~/.pedalkernel/skins/`. The plugin scans this directory at startup and
   applies matching skins. No recompilation needed.

3. **Skin hot-reload** (development mode) -> watch the skin file for changes,
   reload textures when modified. Useful during asset development.

4. **Knob style per control** -> each knob in a skin can reference a different
   filmstrip. One pedal could have chicken-head knobs for Drive and small
   pointers for trim pots, just like real hardware.

5. **Animated LEDs** -> swap between on/off textures, or use a short filmstrip
   for glow fade-in/fade-out animation.

---

## Phase Plan

### Phase 1: Infrastructure
- Add `nih_plug_egui` dependency
- Implement `editor()` on `PedalKernelPlugin`
- Create `PedalSkin` data model and TOML loader
- Build filmstrip knob widget with drag interaction
- Render a single placeholder pedal (colored rectangle + circles for knobs)

### Phase 2: Generic Skin
- Design and render a `generic_bg.png` enclosure in Blender
- Render one standard knob filmstrip (Davies 1900H style, black)
- Render LED on/off assets
- Wire up parameter binding (knob drag -> param change -> DSP)
- Auto-layout knobs based on pedal control count (1-6)

### Phase 3: Per-Pedal Skins (The Big One)
- Model and render 7 unique enclosures matching the real pedal aesthetics
- Render knob filmstrips in each pedal's style/color
- Create `.pedalskin` definitions for all built-in pedals
- Add pedal name silkscreen text to enclosure renders
- Handle pedal-swap animation (fade or instant swap)

### Phase 4: Polish
- Double-click to reset knob
- Right-click context menu (MIDI learn, automation)
- Tooltip showing parameter value on hover
- Bypass footswitch with LED toggle
- Window resize support
- DPI-aware asset selection (@1x vs @2x)

### Phase 5: User Customization
- Scan `~/.pedalkernel/skins/` for user skins
- Document skin format for community creators
- Optional: skin hot-reload for development workflow

---

## Unified Asset Pipeline: Plugin UI to Physical Pedal

A major advantage of investing in a proper 3D asset pipeline is that the **same
Blender models, textures, and layouts produce both the plugin UI and the
manufacturing files for real hardware.** Design once, output everywhere.

### The Master File: One Blender Project Per Pedal

```
assets/blender/pedals/tube_screamer/
  tube_screamer.blend          # Complete 3D scene
  ├── Enclosure (1590B mesh)   # Exact Hammond dimensions: 112 x 60 x 31 mm
  ├── Knobs (linked from knob library)
  ├── Footswitch, jacks, LED
  ├── Materials (PBR: powder coat, chrome, rubber)
  ├── Lighting rig (3-point + HDRI)
  ├── Camera: "GUI Render"     # Top-down for plugin filmstrips
  ├── Camera: "UV Print"       # Orthographic, face-only, for manufacturing
  └── Camera: "Product Shot"   # 3/4 angle for marketing/website
```

From this single scene you export:

| Output | Purpose | Format | How |
|--------|---------|--------|-----|
| Plugin background | VST GUI enclosure | PNG @2x (800x1400) | Render "GUI Render" camera, no knobs visible |
| Knob filmstrips | VST GUI knob rotation | PNG filmstrip (128 frames) | Animate knob rotation, render sequence |
| UV print artwork | Physical enclosure printing | PDF (CMYK + spot layers) | Bake texture → export → finalize in Inkscape/Illustrator |
| Laser engrave file | Physical enclosure engraving | SVG / DXF (vector) | Export face art via Freestyle SVG or Flatterer add-on |
| Product renders | Website, marketing | PNG/JPEG @high-res | Render "Product Shot" camera |
| PCB silkscreen | Circuit board labeling | KiCad footprint (.kicad_mod) | Export label layer SVG → svg2shenzhen → KiCad |

### Physical Manufacturing Formats

#### UV Printing (Full-Color on Aluminum)

This is how EarthQuaker Devices, JHS, and most modern builders do it. A Roland
VersaUV flatbed printer sprays CMYK + white + gloss directly onto powder-coated
or bare aluminum.

**From Blender to UV print:**

1. Model enclosure at **exact real-world dimensions** (1590B = 112 x 60 x 31 mm)
2. UV unwrap the top face
3. Texture paint the design (or apply materials and bake)
4. **Bake texture** to image: minimum **4096 x 4096 px** (covers 600+ DPI on a
   1590B face which is 56 x 108.5 mm)
5. Export baked texture as PNG/TIFF
6. Import into **Inkscape or Illustrator** at exact physical dimensions
7. Convert to **CMYK** color space
8. Add required spot-color layers:
   - **WHITE layer**: RDG_WHITE spot color (where white ink prints; required for
     any white or light areas on a dark enclosure)
   - **GLOSS layer**: RDG_GLOSS spot color (varnish for shine/matte contrast)
   - **COLOR layer**: The CMYK artwork itself
9. Convert all text to outlines/paths
10. Export as **PDF** for the print service (Tayda, AmplifyFun, etc.)

**Resolution requirements:**
| Target DPI | Pixel dimensions for 1590B face (56 x 108.5 mm) |
|------------|--------------------------------------------------|
| 300 DPI | 661 x 1281 px |
| 400 DPI (minimum) | 882 x 1709 px |
| 600 DPI (recommended) | 1323 x 2563 px |
| 1440 DPI (printer native) | 3175 x 6152 px |

Since we're already rendering enclosure backgrounds at 800x1400 for the plugin
GUI (@2x), bumping to **1600x2800** or higher for the master bake gives us
print-ready resolution with zero extra modeling work.

#### Laser Engraving (Powder Coat Removal)

For a more industrial/boutique look: laser away the powder coat to expose bare
aluminum. Creates a permanent, unfadeable two-tone design.

**From Blender to laser file:**

1. Use Blender's **Freestyle SVG Exporter** (built-in) or the **Flatterer**
   add-on to export the enclosure face artwork as vector SVG
2. Open in Inkscape, set document dimensions to physical enclosure size
3. Color-code paths: black = engrave, red = score/cut
4. Set stroke width to **0.1 mm (hairline)** for cut/score operations
5. Convert all text to paths
6. Export as **SVG or DXF** for the laser cutter (LightBurn, etc.)

**What works well with engraving:**
- Control labels (text)
- Logos and line art
- Simple geometric patterns
- Knob position markers (the "0-10" scale arc)

**What needs UV printing instead:**
- Multi-color artwork
- Photographic images
- Gradients and complex shading

#### Screen Printing (Legacy, Large Runs)

Traditional method, still used for 1-3 color designs at scale (25+ units):
- Export vector artwork (SVG/AI/EPS), one color per layer
- Each color = one screen (mesh setup ~$100-150 per screen)
- Use **Nazdar epoxy ink** for durability on metal
- Economics favor UV printing for runs under ~50 units

#### PCB Silkscreen Art

The same design elements (logo, knob labels, component references) can target
PCB fabrication:

1. Design labels and logos as SVG in Inkscape
2. Use **svg2shenzhen** extension to assign Inkscape layers to KiCad PCB layers:
   - `F.SilkS` = front silkscreen (white text/logos)
   - `F.Mask` = front solder mask openings
   - `Edge.Cuts` = board outline
3. Export to `.kicad_mod` footprint files
4. Import into KiCad alongside the PedalKernel-generated netlist

Alternatively, KiCad's **bitmap2component** tool converts PNG → footprint at
300 PPI (a 300x300 px image = 1" on the board).

### Unified Inkscape Master

For maximum reuse, maintain a single **Inkscape SVG per pedal** with named
layers that export to every target:

```
tube_screamer_master.svg
  Layer: "COLOR"        → UV print color artwork
  Layer: "WHITE"        → UV print white ink areas
  Layer: "GLOSS"        → UV print varnish areas
  Layer: "ENGRAVE"      → Laser engraving paths
  Layer: "F.SilkS"      → PCB front silkscreen
  Layer: "Edge.Cuts"    → PCB board outline
  Layer: "DRILL"        → Enclosure drill template + PCB drills
  Layer: "GUI_OVERLAY"  → Plugin-only overlay elements
```

**Export targets from one file:**
- Toggle COLOR + WHITE + GLOSS visible → Export PDF → Tayda UV printing
- Toggle ENGRAVE visible → Export SVG → LightBurn laser engraving
- Toggle F.SilkS + Edge.Cuts + DRILL visible → svg2shenzhen → KiCad
- Toggle COLOR + GUI_OVERLAY visible → Export PNG @2x → Plugin background asset

### What This Means in Practice

When you design a new pedal:

1. **Model it once** in Blender (or draw it in Inkscape for 2D-only)
2. **Render it** for the plugin UI (filmstrips + background)
3. **Export it** for UV printing when you're ready to manufacture
4. **Export it** for laser engraving if you prefer that finish
5. **Export it** for PCB silkscreen labeling
6. **Render product shots** for the website / marketing

The 3D model IS the single source of truth. Change the color of the enclosure in
Blender, re-render, and both the plugin skin and the manufacturing file update.

### Asset Directory Structure (Extended for Manufacturing)

```
assets/
  blender/
    pedals/
      tube_screamer/
        tube_screamer.blend
        tube_screamer_master.svg    # Inkscape unified export file
      fuzz_face/
        ...
    knobs/
      chicken_head.blend
      davies_1900h.blend
      ...
    shared/
      lighting_rig.blend            # Linked into all pedal scenes
      hdri/studio_neutral.exr
  gui/                              # Plugin UI assets (generated from blender/)
    enclosures/
      tube_screamer_bg.png
      tube_screamer_bg@2x.png
      ...
    filmstrips/
      pointer_cream_128f.png
      ...
    leds/
      led_red_on.png
      ...
  manufacturing/                    # Physical production files (generated)
    uv_print/
      tube_screamer_tayda.pdf       # Ready for Tayda UV print service
      fuzz_face_tayda.pdf
      ...
    laser/
      tube_screamer_engrave.svg
      ...
    drill_templates/
      1590b_ts808_drill.dxf        # CNC drill template
      ...
    pcb/
      tube_screamer_silkscreen.kicad_mod
      ...
  render/                           # Marketing / product shots
    tube_screamer_hero.png
    tube_screamer_angle.png
    ...
```

---

## Open Questions

1. **Window size**: Fixed per pedal (like real hardware) or fixed viewport with
   pedal centered? GarageBand uses a fixed "pedalboard" viewport; Neural DSP
   uses per-plugin window sizes.

2. **Pedalboard view**: Eventually render a chain of pedals side-by-side when
   loading a `.board` file? This is a significant UI expansion but very
   compelling visually.

3. **Blender vs. KnobMan3D**: Blender gives full control but requires modeling
   skill. KnobMan3D is purpose-built for knob filmstrips but limited in
   enclosure rendering. Recommendation: Blender for enclosures, either tool for
   knobs.

4. **Asset embedding vs. external files**: Embed PNGs in the binary via
   `include_bytes!()` for zero-dependency distribution? Or ship assets alongside
   the `.vst3` bundle? VST3 bundles support a `Resources/` directory; CLAP does
   not have a standard resource mechanism.

5. **Font for labels**: Render labels as part of the enclosure background
   (static silkscreen look), or render dynamically with a font (flexible but
   less realistic)? Recommendation: static in the background for built-in
   pedals, dynamic for user pedals.

6. **Enclosure finish for physical builds**: UV printing (full-color, Tayda
   service, ~$3-5/unit) or laser engraving (two-tone, more "boutique" feel,
   requires a fiber/CO2 laser)? Could offer both -- the same Blender model
   exports to either format.

7. **Texture resolution master**: Render Blender bakes at 4096x4096+ so the
   same texture is print-ready (600 DPI on a 1590B) without re-rendering. The
   plugin GUI just downsamples from the master.
