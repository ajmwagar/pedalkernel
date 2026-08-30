# `.cab` DSL Reference

The `.cab` format describes a speaker cabinet's electro-mechano-acoustic system. The PedalKernel compiler transforms this into either a real-time WDF processor or a static impulse response (`.wav`).

Separate parser module from `.pedal`. Shared engineering notation, comment syntax (`#`), and block structure. All sections use `{ }` blocks. All values support standard engineering suffixes (`p`, `n`, `u`, `m`, `k`, `M`, `G`).

## Additional Unit Suffixes

| Suffix | Meaning | Conversion | Example |
|--------|---------|------------|---------|
| `g` | grams | ÷ 1000 → kg | `Mms: 34.5g` |
| `cm` | centimeters | ÷ 100 → m | `depth: 35cm` |
| `cm2` | square centimeters | ÷ 10000 → m² | `Sd: 531cm2` |
| `mm` | millimeters | ÷ 1000 → m | `thickness: 18mm` |
| `L` | liters | ÷ 1000 → m³ | `volume: 45L` |
| `C` | degrees Celsius | (used directly) | `temperature: 22C` |
| `dB` | decibels | (used directly) | `breakup_level: -12dB` |
| `Hz` | hertz | (used directly) | `breakup_freq: 4200Hz` |
| `%` | percent | ÷ 100 → ratio | `coverage: 50%` |
| `deg` | degrees | (used directly) | `angle: 45deg` |

Standard engineering suffixes (`k`, `m`, etc.) follow the same rules as `.pedal` files.

---

## File Structure

```
cab "<Name>" {
  driver { ... }              # REQUIRED — at least one
  enclosure { ... }           # REQUIRED
  crossover { ... }           # Optional — for multi-way cabs
  mic <id> { ... }            # Optional — one or more named mic positions
  environment { ... }         # Optional — defaults to standard conditions
  controls { ... }            # Optional — exposed parameters
  monitors { ... }            # Optional — real-time metering
}
```

Multi-driver cabs use named driver blocks:

```
cab "4x12" {
  driver upper_left { ... }
  driver upper_right { ... }
  driver lower_left { ... }
  driver lower_right { ... }
  enclosure { ... }
}
```

Identical drivers can use `copies`:

```
cab "4x12" {
  driver all {
    preset: g12m_greenback
    copies: 4
    positions: [[19cm, 57cm], [57cm, 57cm], [19cm, 19cm], [57cm, 19cm]]
  }
  enclosure { ... }
}
```

Mixed driver configurations:

```
cab "2x12 Mixed" {
  driver left {
    preset: v30
    position: [19cm, 30cm]
  }
  driver right {
    preset: g12m_greenback
    position: [57cm, 30cm]
  }
  enclosure { ... }
}
```

---

## Driver Block

### Thiele-Small Parameters

Primary interface. Published on every speaker datasheet.

```
driver {
  # --- Identification (optional) ---
  model: "Celestion Vintage 30"

  # --- Electrical ---
  Re: 5.6                      # DC voice coil resistance (Ω)
  Le: 0.62m                    # Voice coil inductance (H)
  Bl: 10.2                     # Force factor (T·m)

  # --- Mechanical ---
  Mms: 34.5g                   # Moving mass (kg)
  Cms: 0.34m                   # Suspension compliance (m/N)
  Rms: 2.4                     # Mechanical resistance (N·s/m)
  Sd: 531cm2                   # Effective radiating area (m²)

  # --- Derived (calculated by compiler, override-able) ---
  # Fs: 75Hz                   # Resonant freq = 1/(2π√(Mms·Cms))
  # Qms: 6.72                  # Mech Q = 2π·Fs·Mms / Rms
  # Qes: 0.89                  # Elec Q = 2π·Fs·Mms·Re / Bl²
  # Qts: 0.787                 # Total Q = Qms·Qes / (Qms+Qes)
  # Vas: 68L                   # Equiv compliance vol = ρ·c²·Sd²·Cms
  # eta0: 1.8%                 # Reference efficiency
  # SPL: 100dB                 # Sensitivity (1W/1m)
  # Xmax: 3.5mm                # Linear excursion limit (from Bl/Cms)
}
```

### Preset Speakers

```
driver {
  preset: v30                   # Load all T-S params from database
  Rms: 3.0                     # Override individual values
}
```

**Guitar speakers:**

| Preset | Speaker | Fs | Qts | Character |
|--------|---------|-----|-----|-----------|
| `v30` | Celestion Vintage 30 | 75 Hz | 0.79 | Aggressive presence peak |
| `g12m_greenback` | Celestion G12M Greenback | 75 Hz | 0.95 | Smooth, warm, classic rock |
| `g12h_anniversary` | Celestion G12H Anniversary | 55 Hz | 0.60 | Tight, balanced |
| `g12t75` | Celestion G12-T75 | 75 Hz | 0.82 | Bright, modern, scooped |
| `g12_65` | Celestion G12-65 | 65 Hz | 0.75 | Full-range, balanced |
| `g12_blue` | Celestion Blue | 100 Hz | 1.00 | Chiming, sweet alnico |
| `g12_creamback` | Celestion Creamback | 75 Hz | 0.85 | Greenback warmth + clarity |
| `g12_evh` | Celestion G12 EVH | 70 Hz | 0.70 | Tight, aggressive |
| `p12r` | Jensen P12R | 95 Hz | 1.05 | Vintage Fender clean |
| `c12n` | Jensen C12N | 80 Hz | 0.90 | Louder Fender |
| `c12q` | Jensen C12Q | 90 Hz | 1.10 | Budget vintage, soft |
| `p10r` | Jensen P10R | 100 Hz | 0.95 | Small Fender combo |
| `swamp_thang` | Eminence Swamp Thang | 83 Hz | 0.78 | Massive low end |
| `man_o_war` | Eminence Man O War | 80 Hz | 0.65 | Tight, high-gain |
| `cv75` | Eminence CV-75 | 75 Hz | 0.88 | Greenback-style |
| `legend_1258` | Eminence Legend 1258 | 55 Hz | 0.72 | All-around workhorse |
| `texas_heat` | Eminence Texas Heat | 80 Hz | 0.75 | Hot mids |
| `cannabis_rex` | Eminence Cannabis Rex | 50 Hz | 0.95 | Warm, hemp cone |
| `ev12l` | EV EVM12L | 55 Hz | 0.41 | Flat, PA-style, heavy |
| `jbl_d120` | JBL D120F | 48 Hz | 0.38 | Ultra-clean, hi-fi |
| `wb_et65` | WGS ET65 | 75 Hz | 0.82 | Smooth, balanced |
| `wb_retro30` | WGS Retro 30 | 75 Hz | 0.80 | V30 alternative |

**Bass speakers:**

| Preset | Speaker | Fs | Qts | Character |
|--------|---------|-----|-----|-----------|
| `basslite_s2010` | Eminence Basslite S2010 | 55 Hz | 0.44 | Lightweight neo bass |
| `kappalite_3015` | Eminence Kappalite 3015 | 42 Hz | 0.36 | Deep bass, lightweight |
| `ampeg_10` | Eminence for Ampeg 10" | 70 Hz | 0.55 | Classic SVT 10" |

### Voice Coil

```
driver {
  ...
  voice_coil {
    # Semi-inductance (Leach / Wright-Bailey model)
    # Real voice coils: inductance decreases with frequency due to
    # eddy currents in the pole piece. Impedance rises slower than
    # a pure inductor above Fs.
    semi_inductance: true       # Default: false
    Ke: 0.05                    # Eddy current coefficient
    n: 0.7                      # Semi-inductance exponent (0.5-1.0)
                                # 0.5 = strong eddy currents (shorting ring)
                                # 1.0 = pure inductor (no eddy currents)

    # Thermal modeling (Phase 6)
    thermal {
      capacity: 1.2             # Thermal mass (J/°C)
      R_ambient: 8.0            # Thermal resistance coil → air (°C/W)
      R_magnet: 3.5             # Thermal resistance coil → magnet (°C/W)
      tc_Re: 0.00393            # Temp coefficient of Re (copper = 0.393%/°C)
      tc_Bl: -0.0012            # Temp coefficient of Bl (negative)
    }
  }
}
```

### Cone

```
driver {
  ...
  cone {
    material: paper             # See material table below
    dust_cap: felt              # paper, felt, aluminum, screen, phase_plug, none

    # Breakup behavior — auto-derived from material if omitted
    breakup_freq: 4200Hz        # First breakup mode frequency
    breakup_modes: 4            # Number of modes to model (1-8)
    breakup_q: 4.5              # Q factor of breakup resonances
    breakup_level: -12dB        # Amplitude relative to piston band

    # Surround type affects HF rolloff
    surround: cloth             # cloth, rubber, foam, accordion
  }
}
```

**Cone materials:**

| Material | Stiffness | Damping | Breakup Freq (12") | Character |
|----------|-----------|---------|---------------------|-----------|
| `paper` | Moderate | High | ~3-5 kHz | Warm, classic breakup |
| `kevlar` | High | Moderate | ~5-7 kHz | Controlled, modern |
| `aluminum` | Very high | Low | ~7-10 kHz | Sharp, metallic breakup |
| `ceramic` | Extreme | Very low | ~8-12 kHz | Extended, harsh breakup |
| `hemp` | Moderate | High | ~3-4 kHz | Vintage warmth |
| `neo_paper` | Mod-high | Moderate | ~4-6 kHz | Modern "hot" speakers |
| `pulp` | Low | Very high | ~2-3 kHz | Very dark, vintage |
| `carbon_fiber` | Very high | Moderate | ~6-9 kHz | Stiff, controlled |

### Suspension Nonlinearity (Phase 6)

```
driver {
  ...
  suspension {
    Xmax: 3.5mm                 # Maximum linear excursion
    Xmech: 8mm                  # Mechanical limit (damage)

    # Cms(x) curve shape
    curve: hardening            # linear, hardening, softening
    # hardening: Cms(x) = Cms0 / (1 + (x/Xmax)^alpha)
    # alpha defaults to 2.0, adjustable:
    hardening_exp: 2.0

    # BL(x) profile
    bl_symmetry: 0.85           # 1.0 = symmetric, <1.0 = asymmetric
    bl_profile: gaussian        # gaussian, flat_top, tilted
    # gaussian: BL(x) = Bl0 × exp(-(x/sigma)^2)
    # flat_top: BL(x) = Bl0 for |x| < Xflat, then gaussian taper
    # tilted:  BL(x) = Bl0 × (1 + tilt×x) × exp(-(x/sigma)^2)

    # Suspension creep (break-in)
    creep: 0.0                  # 0.0 = broken in, 1.0 = brand new
  }
}
```

### Driver Position

For multi-driver cabs, each driver specifies its baffle position:

```
driver upper_left {
  preset: v30
  position: [19cm, 57cm]       # [x, y] from bottom-left of baffle
}
```

For single-driver cabs, position is specified in the `baffle` sub-block of `enclosure`.

---

## Enclosure Block

### Enclosure Types

```
enclosure {
  type: open_back               # REQUIRED — see types below
  volume: 45L                   # REQUIRED — internal air volume
}
```

**Types:**

| Type | Description | Key Parameters |
|------|-------------|---------------|
| `sealed` | Closed box | `volume` |
| `open_back` | Rear panel partially or fully open | `volume`, `open_area`, `depth` |
| `bass_reflex` | Ported enclosure | `volume`, `port { ... }` |
| `transmission_line` | Quarter-wave line | `line_length`, `taper`, `stuffing` |
| `horn` | Horn-loaded | `throat_area`, `mouth_area`, `length`, `profile` |
| `infinite_baffle` | Free-air (no enclosure) | (none) |
| `bandpass` | Bandpass enclosure | `volume_front`, `volume_rear`, `port { ... }` |

### Sealed

```
enclosure {
  type: sealed
  volume: 45L
  # System Qtc auto-calculated from driver Qts and Vas/Vb ratio
  # Qtc = Qts × √(1 + Vas/Vb)
  # Override: Qtc: 0.707       # Forces specific alignment
  leakage: 10M                 # Enclosure air leakage resistance (Ω acoustic)
                                # Default: 10M (well-sealed)
                                # Lower = leakier box (vintage builds)
}
```

### Open Back

```
enclosure {
  type: open_back
  volume: 50L
  open_area: 75%                # Rear panel open percentage
  depth: 25cm                   # Front-to-back internal depth
  # Dipole cancellation frequency derived from depth:
  # f_cancel ≈ c / (2 × path_length)
  # path_length ≈ depth + baffle_perimeter/4
}
```

### Bass Reflex

```
enclosure {
  type: bass_reflex
  volume: 55L

  port {
    shape: round                # round, slot, passive_radiator
    diameter: 10cm              # For round
    # width: 12cm              # For slot
    # height: 2cm              # For slot
    length: 18cm
    flare: none                 # none, single, double
    # Flared ports reduce turbulence noise at high SPL

    # Passive radiator (alternative to port tube):
    # shape: passive_radiator
    # Mmp: 80g                 # Passive radiator moving mass
    # Cmp: 0.5m                # Passive radiator compliance
    # Sd_pr: 531cm2            # Passive radiator area

    # Port nonlinearity (Phase 6)
    # Turbulence onset modeled as velocity-dependent resistance:
    # R(v) = R_linear + R_turb × max(0, |v| - v_threshold)²
    # v_threshold auto-calculated from port area and Xmax
  }
}
```

### Transmission Line

```
enclosure {
  type: transmission_line
  line_length: 1.2m            # Acoustic path length
  taper: exponential            # constant, exponential, conical
  stuffing: medium              # none, light, medium, heavy
  # Stuffing density affects both damping and effective line length
  # (acoustic velocity is reduced in stuffed lines)
}
```

### Horn

```
enclosure {
  type: horn
  throat_area: 100cm2           # At driver end
  mouth_area: 2000cm2           # At open end
  length: 60cm                  # Path length
  profile: exponential          # conical, exponential, hyperbolic, tractrix
  # Cutoff frequency determined by mouth area:
  # fc ≈ c / (2π × √(mouth_area/π))
}
```

### Bandpass

```
enclosure {
  type: bandpass
  volume_front: 30L             # Sealed front chamber
  volume_rear: 40L              # Ported rear chamber
  port {
    shape: round
    diameter: 8cm
    length: 15cm
  }
}
```

### Construction

```
enclosure {
  ...
  construction {
    material: birch_ply(18mm)   # <material>(<thickness>)

    # Explicit dimensions (optional — estimated from volume if omitted)
    width: 76cm
    height: 76cm
    depth: 35cm

    # Or specify material properties directly:
    # material: custom(18mm) {
    #   density: 680
    #   youngs_modulus: 12G
    #   damping_ratio: 0.025
    #   poissons_ratio: 0.3
    # }

    bracing {
      B1: brace(horizontal, 38cm)
      B2: brace(vertical, 38cm)
      B3: brace(cross, 38cm, 38cm)
      # Brace cross-section (optional, defaults to 20mm × 40mm)
      # B1: brace(horizontal, 38cm, 20mm × 40mm)
      # B1: brace(horizontal, 38cm, material: oak(20mm × 40mm))
    }

    joints: dado                # butt, dado, rabbet, mitre, finger, dovetail
    corners: metal_bracket      # butt, finger_joint, dovetail, metal_bracket
    covering: tolex             # none, tolex, tweed, vinyl, carpet, rat_fur

    # Rear panel (for sealed/bass_reflex — ignored for open_back)
    rear_panel: same            # same (same material as sides), or override:
    # rear_panel: mdf(12mm)
  }
}
```

**Wood / panel materials:**

| Material | Density (kg/m³) | Young's (GPa) | Damping | Character |
|----------|-----------------|----------------|---------|-----------|
| `birch_ply` | 680 | 12.0 | 0.025 | Tight, articulate, modern |
| `pine` | 500 | 9.0 | 0.045 | Warm, resonant, vintage |
| `mdf` | 750 | 4.0 | 0.080 | Dead, neutral, studio |
| `particle_board` | 650 | 3.0 | 0.090 | Dull, cheap |
| `poplar` | 420 | 8.0 | 0.040 | Light, open midrange |
| `marine_ply` | 700 | 13.0 | 0.020 | Bright, stiff |
| `oak` | 750 | 12.0 | 0.030 | Dense, tight |
| `mahogany` | 550 | 10.0 | 0.035 | Warm, musical |
| `cedar` | 380 | 6.0 | 0.055 | Light, very resonant |
| `bamboo_ply` | 700 | 14.0 | 0.020 | Very stiff, bright |

**Covering materials:**

| Covering | Added Mass (kg/m²) | Damping Factor | Character |
|----------|-------------------|----------------|-----------|
| `none` | 0 | 1.0× | Raw wood resonance |
| `tolex` | 0.8 | 1.3× | Slight damping, standard |
| `tweed` | 0.4 | 1.1× | Minimal damping, vintage |
| `vinyl` | 1.0 | 1.4× | Moderate damping |
| `carpet` | 1.5 | 1.8× | Heavy damping, 80s metal |
| `rat_fur` | 1.2 | 1.6× | Moderate-heavy damping |

### Internal Damping

```
enclosure {
  ...
  damping {
    material: polyester_fill
    coverage: 50%
    thickness: 25mm
  }
}
```

| Material | Absorption Profile | Character |
|----------|-------------------|-----------|
| `none` | — | Full internal reflections |
| `polyester_fill` | Moderate, strongest > 500 Hz | Standard, clean |
| `fiberglass` | High, broadband | Very absorbent |
| `cotton_batting` | Gentle, mostly HF | Soft, minimal |
| `acoustic_foam` | Tuned, profile-dependent | Controlled |
| `wool` | Excellent broadband | Premium, natural |
| `long_hair_wool` | Very high, even at LF | Maximum absorption |
| `dacron` | Similar to polyester, lighter | Light fill |

### Baffle

```
enclosure {
  ...
  baffle {
    # Single-driver position (ignored if drivers specify positions)
    driver_position: [25cm, 26cm]

    edge: rounded(2cm)          # square, rounded(<radius>), chamfered(<width>)
    grille: cloth               # none, cloth, metal_perf, expanded_metal
    grille_distance: 15mm       # Gap between grille and baffle face

    # Baffle thickness override (defaults to construction material thickness)
    # thickness: 24mm           # Thicker baffle = stiffer = less diffraction
  }
}
```

---

## Crossover Block (Multi-Way Cabs)

For cabs with separate woofer/tweeter or multi-way configurations:

```
crossover {
  type: passive                 # passive, active, none
  topology: second_order        # first_order, second_order, third_order, fourth_order

  # High-pass to tweeter
  high_pass {
    frequency: 3500Hz
    driver: tweeter
    components {
      C1: cap(10u)
      L1: inductor(0.5m)
    }
  }

  # Low-pass to woofer
  low_pass {
    frequency: 3500Hz
    driver: woofer
    components {
      L2: inductor(1.5m)
      C2: cap(22u)
    }
  }
}
```

Or simplified:

```
crossover {
  type: passive
  topology: second_order
  frequency: 3500Hz
  high: tweeter
  low: woofer
  # Compiler generates standard Butterworth component values
}
```

---

## Mic Block

Named mic positions. Multiple mics supported. Omitting mic blocks uses a default `flat` mic at 1m on-axis.

```
mic <id> {
  model: sm57                   # Mic model (see table)
  position: [x, y, z]          # Relative to target driver cone center
                                # x: left(-) / right(+)
                                # y: down(-) / up(+)
                                # z: distance from grille
  angle: 0deg                   # Off-axis angle (0 = on-axis)
  target: upper_left            # Which driver (multi-driver cabs)
                                # or: center (aims at baffle center)
}
```

```
mic close {
  model: sm57
  position: [2cm, 0cm, 3cm]
  angle: 15deg
}

mic room {
  model: u87
  position: [0cm, 0cm, 120cm]
  angle: 0deg
}

mic rear {
  model: r121
  position: [0cm, 0cm, -30cm]  # Negative z = behind cab (open-back)
  angle: 0deg
}
```

### Microphone Models

| Model | Type | Pattern | Proximity | Character |
|-------|------|---------|-----------|-----------|
| `sm57` | Dynamic | Cardioid | Yes | Presence peak ~5-6 kHz, industry standard |
| `sm7b` | Dynamic | Cardioid | Reduced | Smooth, flat, broadcast |
| `md421` | Dynamic | Cardioid | Yes | Full range, pronounced low-mid |
| `e906` | Dynamic | Supercardioid | Yes | Designed for cabs, presence switch |
| `e609` | Dynamic | Supercardioid | Yes | Flat against grille, bright |
| `m160` | Ribbon | Hypercardioid | No | Smooth HF rolloff, detailed mids |
| `r121` | Ribbon | Figure-8 | Yes | Dark, warm, smooth |
| `r101` | Ribbon | Figure-8 | Yes | Brighter ribbon |
| `fathead` | Ribbon | Figure-8 | Yes | Budget ribbon, slightly gritty |
| `u87` | Condenser | Multi | Yes | Full range, accurate room mic |
| `c414` | Condenser | Multi | Yes | Bright, detailed |
| `km84` | Condenser SD | Cardioid | Mild | Precise transients |
| `c451` | Condenser SD | Cardioid | Mild | Bright, snappy transients |
| `re20` | Dynamic | Cardioid | No | Flat, no proximity, broadcast |
| `m88` | Dynamic | Hypercardioid | Yes | Tight pattern, punchy |
| `m201` | Dynamic | Hypercardioid | Mild | Bright, detailed |
| `flat` | — | Omni | No | Mathematically flat, no coloration |
| `measurement` | — | Omni | No | Calibrated measurement mic response |

### Mic Position Effects (computed by compiler)

**Proximity effect**: bass boost at close distances for cardioid/figure-8 patterns. Modeled as a distance-dependent low shelf. Disabled for omni and mics marked `Proximity: No`.

**Cone position weighting**: at cap center, more direct HF energy. At cone edge, more body, less fizz. The compiler weights the driver's angular radiation pattern at each frequency based on mic position relative to cone geometry.

**Comb filtering**: at very close distances, path length differences from mic to different cone regions create comb filtering. Computed from cone geometry and mic position.

### Custom Mic Response

```
mic custom_mic {
  model: custom
  response_file: "my_mic_response.csv"   # Frequency (Hz), magnitude (dB), phase (deg)
  pattern: cardioid
  proximity: true
  position: [0cm, 0cm, 5cm]
}
```

---

## Environment Block

Optional. Defaults to standard conditions (20°C, 50% humidity, sea level).

```
environment {
  temperature: 22C              # Speed of sound: c = 331.3 × √(1 + T/273.15)
  humidity: 45%                 # HF air absorption coefficient
  altitude: 150m                # Air density: ρ ≈ 1.225 × (1 - 2.25577e-5 × alt)^5.25588
  # pressure: 101325            # Atmospheric pressure (Pa) — derived from altitude if omitted
}
```

Affects: port tuning frequency, transmission line tuning, baffle diffraction wavelengths, radiation impedance, HF air absorption for distant mics.

---

## Controls Block

Expose parameters as named knobs with ranges and defaults. Same syntax as `.pedal` controls.

```
controls {
  # Driver
  driver.Mms -> "Cone Weight" [15g, 60g] = 34.5g
  driver.Cms -> "Suspension" [0.1m, 1.0m] = 0.34m
  driver.Bl -> "Motor Strength" [5.0, 18.0] = 10.2
  driver.cone.breakup_freq -> "Breakup Freq" [2000Hz, 8000Hz] = 4200Hz
  driver.cone.breakup_level -> "Breakup Level" [-24dB, 0dB] = -12dB

  # Enclosure
  enclosure.volume -> "Cab Size" [15L, 120L] = 45L
  enclosure.open_area -> "Back Open" [0%, 100%] = 75%
  enclosure.damping.coverage -> "Stuffing" [0%, 100%] = 50%
  enclosure.construction.material.damping_ratio -> "Wood Resonance" [0.01, 0.15] = 0.025

  # Port (bass reflex)
  enclosure.port.length -> "Port Tune" [5cm, 40cm] = 18cm

  # Mic
  mic.close.position.x -> "Mic X" [-10cm, 10cm] = 2cm
  mic.close.position.z -> "Mic Distance" [1cm, 30cm] = 3cm
  mic.close.angle -> "Mic Angle" [0deg, 60deg] = 15deg

  # Suspension (Phase 6)
  driver.suspension.Xmax -> "Excursion Limit" [1mm, 10mm] = 3.5mm
  driver.suspension.bl_symmetry -> "BL Symmetry" [0.5, 1.0] = 0.85
  driver.suspension.creep -> "Break-In" [0.0, 1.0] = 0.0

  # Thermal (Phase 6)
  # (typically not user-exposed, but available)
}
```

---

## Monitors Block

Real-time metering for the WDF cab sim (Phase 7). Same syntax as `.pedal`.

```
monitors {
  driver.cone_displacement -> "Excursion" [peak]
  driver.voice_coil_temp -> "VC Temp" [thermal]
  driver.power -> "Power" [vu]
  output -> "Output Level" [ppm]
  enclosure.port_velocity -> "Port Velocity" [peak]
}
```

---

## CLI Usage

### IR Export

```bash
# Default: 48 kHz, 2048 samples, 24-bit, mono, normalized
pedalkernel ir <file.cab> <output.wav>

# Custom settings
pedalkernel ir <file.cab> <output.wav> \
  --sample-rate 48000 \
  --length 4096 \
  --bit-depth 24 \
  --normalize true \
  --fade-out 64                 # Cosine fade on last N samples

# Parameter overrides
pedalkernel ir <file.cab> output.wav "Mic X=4cm" "Mic Distance=5cm"

# Multi-mic: exports one WAV per mic block
pedalkernel ir <file.cab> output_dir/ --multi-mic

# Stereo: first two mic blocks → L/R channels
pedalkernel ir <file.cab> stereo.wav --stereo

# Parameter sweep
pedalkernel ir <file.cab> sweep_dir/ \
  --sweep "Mic X" -8cm 8cm 17

# Multi-dimensional sweep
pedalkernel ir <file.cab> sweep_dir/ \
  --sweep "Mic X" -8cm 8cm 17 \
  --sweep "Mic Distance" 1cm 10cm 5

# Batch all mic models at one position
pedalkernel ir <file.cab> batch_dir/ \
  --sweep-mics sm57,r121,md421,u87,m160,e906
```

### Real-Time Processing

```bash
# Process audio through cab sim (full WDF with nonlinearity)
pedalkernel process <file.cab> input.wav output.wav

# With parameter overrides
pedalkernel process <file.cab> input.wav output.wav "Cab Size=60L" "Stuffing=80%"
```

### In Board Files

```
board "Full Rig" {
  ts: "tube_screamer.pedal" { Drive = 0.7 }
  amp: "tweed_deluxe_5e3.pedal" { Volume = 0.6 }
  cab: "fender_1x12.cab" { Mic X = 2cm, Mic Distance = 3cm }
}
```

### Validation

```bash
pedalkernel validate <file.cab>
pedalkernel validate <file.cab> --fix    # Auto-fix (e.g., calculate missing derived params)
```

---

## Complete Examples

### Fender Deluxe 1×12 Open Back

```
cab "Fender Deluxe 1x12" {
  driver {
    preset: c12n
    cone { material: paper, dust_cap: paper, surround: cloth }
    suspension { Xmax: 2mm, curve: hardening, bl_symmetry: 0.82 }
  }

  enclosure {
    type: open_back
    volume: 42L
    open_area: 80%
    depth: 23cm

    construction {
      material: pine(12mm)
      width: 51cm
      height: 46cm
      depth: 23cm
      bracing {}
      joints: butt
      covering: tweed
      corners: butt
    }

    damping { material: none }

    baffle {
      driver_position: [25cm, 26cm]
      edge: square
      grille: cloth
    }
  }

  mic close {
    model: sm57
    position: [2cm, 0cm, 3cm]
    angle: 15deg
  }

  mic room {
    model: c414
    position: [0cm, 0cm, 150cm]
    angle: 0deg
  }

  controls {
    mic.close.position.x -> "SM57 Position" [-8cm, 8cm] = 2cm
    mic.close.position.z -> "SM57 Distance" [1cm, 15cm] = 3cm
    mic.close.angle -> "SM57 Angle" [0deg, 60deg] = 15deg
  }
}
```

### Marshall 1960A 4×12

```
cab "Marshall 1960A" {
  driver all {
    preset: g12m_greenback
    copies: 4
    positions: [[19cm, 57cm], [57cm, 57cm], [19cm, 19cm], [57cm, 19cm]]
    cone { material: paper, dust_cap: felt, surround: cloth }
    suspension { Xmax: 2.5mm, curve: hardening, bl_symmetry: 0.88 }
  }

  enclosure {
    type: sealed
    volume: 170L

    construction {
      material: birch_ply(18mm)
      width: 76cm
      height: 76cm
      depth: 35cm
      bracing {
        B1: brace(horizontal, 38cm)
        B2: brace(vertical, 38cm)
      }
      joints: dado
      covering: tolex
      corners: metal_bracket
    }

    damping {
      material: polyester_fill
      coverage: 30%
      thickness: 25mm
    }

    baffle {
      edge: square
      grille: cloth
    }
  }

  mic close {
    model: sm57
    position: [2cm, 0cm, 2cm]
    target: upper_left
    angle: 0deg
  }

  mic dark {
    model: r121
    position: [-3cm, 0cm, 8cm]
    target: upper_left
    angle: 0deg
  }

  controls {
    mic.close.position.x -> "SM57 Position" [-8cm, 8cm] = 2cm
    mic.close.position.z -> "SM57 Distance" [1cm, 15cm] = 2cm
    enclosure.construction.material.damping_ratio -> "Wood Resonance" [0.01, 0.08] = 0.025
    enclosure.damping.coverage -> "Stuffing" [0%, 100%] = 30%
  }
}
```

### Mesa Rectifier 4×12

```
cab "Mesa Recto 4x12" {
  driver all {
    preset: v30
    copies: 4
    positions: [[19cm, 57cm], [57cm, 57cm], [19cm, 19cm], [57cm, 19cm]]
    cone { material: paper, dust_cap: felt }
    suspension { Xmax: 3.5mm, curve: hardening, bl_symmetry: 0.85 }
  }

  enclosure {
    type: sealed
    volume: 180L

    construction {
      material: birch_ply(18mm)
      width: 76cm
      height: 76cm
      depth: 37cm
      bracing {
        B1: brace(horizontal, 38cm)
        B2: brace(vertical, 38cm)
        B3: brace(cross, 38cm, 38cm)
      }
      joints: dado
      covering: tolex
      corners: metal_bracket
    }

    damping {
      material: polyester_fill
      coverage: 40%
      thickness: 30mm
    }

    baffle {
      edge: square
      grille: cloth
    }
  }

  mic close {
    model: sm57
    position: [1cm, 0cm, 2cm]
    target: upper_left
    angle: 0deg
  }

  controls {
    mic.close.position.x -> "Mic Position" [-8cm, 8cm] = 1cm
    mic.close.position.z -> "Mic Distance" [1cm, 15cm] = 2cm
  }
}
```

### Ampeg SVT 8×10 (Bass)

```
cab "Ampeg SVT 810" {
  driver all {
    preset: ampeg_10
    copies: 8
    positions: [
      [16cm, 96cm], [40cm, 96cm],   # Top pair
      [16cm, 72cm], [40cm, 72cm],   # Upper-mid pair
      [16cm, 48cm], [40cm, 48cm],   # Lower-mid pair
      [16cm, 24cm], [40cm, 24cm]    # Bottom pair
    ]
    cone { material: paper, dust_cap: paper, surround: rubber }
    suspension { Xmax: 4mm, curve: hardening, bl_symmetry: 0.80 }
  }

  enclosure {
    type: sealed
    volume: 340L

    construction {
      material: birch_ply(18mm)
      width: 61cm
      height: 122cm
      depth: 41cm
      bracing {
        B1: brace(horizontal, 30cm)
        B2: brace(horizontal, 61cm)
        B3: brace(horizontal, 91cm)
      }
      joints: dado
      covering: tolex
      corners: metal_bracket
    }

    damping {
      material: fiberglass
      coverage: 25%
      thickness: 25mm
    }

    baffle {
      edge: square
      grille: cloth
    }
  }

  mic close {
    model: re20
    position: [0cm, 0cm, 5cm]
    target: upper_left
    angle: 0deg
  }
}
```

### Studio Monitor 1×12 Ported

```
cab "Studio 1x12 Ported" {
  driver {
    model: "Custom 12-inch"
    Re: 6.2
    Le: 0.85m
    Bl: 12.5
    Mms: 45g
    Cms: 0.42m
    Rms: 3.2
    Sd: 531cm2

    voice_coil {
      semi_inductance: true
      Ke: 0.04
      n: 0.65
      thermal {
        capacity: 1.5
        R_ambient: 7.0
        R_magnet: 3.0
        tc_Re: 0.00393
        tc_Bl: -0.0010
      }
    }

    cone { material: kevlar, dust_cap: aluminum, surround: rubber }
    suspension { Xmax: 5mm, curve: hardening, bl_symmetry: 0.92 }
  }

  enclosure {
    type: bass_reflex
    volume: 55L

    port {
      shape: round
      diameter: 10cm
      length: 18cm
      flare: double
    }

    construction {
      material: mdf(18mm)
      bracing {
        B1: brace(horizontal, 30cm)
        B2: brace(horizontal, 60cm)
      }
      joints: dado
      covering: none
    }

    damping {
      material: fiberglass
      coverage: 40%
      thickness: 50mm
    }

    baffle {
      driver_position: [20cm, 35cm]
      edge: rounded(3cm)
      grille: none
    }
  }

  mic measurement {
    model: measurement
    position: [0cm, 0cm, 100cm]
    angle: 0deg
  }

  environment {
    temperature: 22C
    humidity: 45%
  }

  controls {
    driver.Mms -> "Cone Weight" [20g, 60g] = 45g
    enclosure.volume -> "Cab Size" [30L, 80L] = 55L
    enclosure.port.length -> "Port Tune" [8cm, 35cm] = 18cm
    enclosure.damping.coverage -> "Stuffing" [0%, 100%] = 40%
  }
}
```
