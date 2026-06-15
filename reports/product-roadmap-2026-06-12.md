# PedalKernel Product Roadmap — 2026-06-12

Companion to `reports/outboard-fix-and-product-plan-2026-06-12.md` (engine
gates M0–M4) and `reports/outboard-gear-audit-2026-06-12.md`. Grounded in five
web-research streams (plugin market, licensing precedents, CLAP/Rust status,
virtual-modular market, 500-series economics + digital-twin precedents) run
2026-06-12; key citations inline, unverified items flagged in the underlying
research notes.

**Owner direction (2026-06-12):** first hardware target is **Eurorack** —
TB-303-class acid circuits and TR-x00-class analog drum voices — ahead of
500-series. Owner has a physical Eurorack for test/validation.

## 1. Thesis

**Validation is the brand.** The market research found that every major vendor
(UAD, Plugin Alliance/TMT, Softube, Arturia/TAE, Acustica) markets
"component-level" authenticity, and **none publishes validation evidence** —
no SPICE overlays, no hardware null tests; all verification is third-party
community measurement (Plugin Doctor culture, Dan Worrall). Separately, the
hardware research concluded that a simulation-first analog hardware company
with a published, validated, shared circuit model is **white space**: the
nearest occupants are Cytomic (schematic-derived plugin, no hardware),
Weiss×Softube (shared code, but digital hardware), and WesAudio/Tegeler
(plugins as remote control for analog hardware, not as its model).

PedalKernel's position: **one `.pedal` file is the product** — it compiles to
the real-time model (plugin/VCV module), generates the KiCad schematic for
the physical unit, and ships with its SPICE validation report. "Our plugin
and our hardware are the same circuit, and here's the proof" is a claim no
competitor can currently make.

## 2. License and business model

- **AGPL-3.0 + CLA is already in place** — and the research found this is
  near-novel in audio (Zrythm is the only AGPL precedent; no AGPL+commercial
  dual-licensed audio engine exists). AGPL means no one ships a proprietary
  plugin on the engine without a commercial license; the CLA means only we
  can grant one. Critically, the CLA exists *before* external contributors —
  the MuseScore pattern (fine) not the Audacity retrofit (forks, revolt).
- **Reusable templates**: HISE (GPL + ~$50/mo indie commercial tier, revenue
  gate at €50k) for engine licensing; VCV's GPLv3 §7 exception (free plugins
  = any license, commercial = negotiated) for a future circuit-library
  ecosystem; Harrison Mixbus (upstreaming + code separation) for our own
  proprietary layers if ever needed.
- **Revenue layers** (engine stays free): (a) sold plugins/modules built on
  our own copyright; (b) commercial engine licenses to third parties; (c)
  hardware (kits + assembled); (d) later: validated-circuit marketplace.
- Cautionary data: donations don't sustain (Ardour ≈$6k over 5 early years;
  Analog Obsession ~98k Patreon members but ~$9k/mo) — recurring products,
  not tips.

## 3. Market facts that bound the plan

- **Plugin pricing reality**: modeled-classic compressors street at $29–50
  (UAD/PA perpetual sales), $99–149 boutique (Pulsar), subscriptions
  $15–25/mo. List prices are fiction. "Free + verifiable, pay for the
  polished product tier" undercuts on both price and trust.
- **CLAP is viable as primary format** (2026): native in Bitwig, REAPER, FL
  Studio 2024, Studio One 7; `clap-wrapper` is feature-complete for VST3 and
  AUv2 (AAX still draft — Pro Tools deferred); Steinberg MIT-licensed VST3
  (Oct 2025), making wrapped distribution cleaner. Rust RT-audio is proven
  (cpal 8.7M downloads, Helgobox as commercial precedent) but `nih-plug` is
  in maintenance mode → treat the plugin shell as a thin replaceable adapter
  over the CLAP C API + clap-wrapper, not a framework bet.
- **Virtual modular**: VCV owns distribution (~3k modules, $149 Pro,
  $19–29/mo VCV+ with backlash); its analog modeling is widely criticized
  (aliasing threads; flagship Mutable ports are digital firmware, not analog
  models); Softube owns component-credibility but is closed and
  CPU-notorious; Cytomic's two Doepfer modules prove paid demand for genuine
  circuit simulation *inside* VCV. Customer is price-sensitive — accuracy
  must be demonstrated audibly (A/B vs hardware), not claimed.
- **Hardware economics**: Eurorack — no certification gate, ±12 V, huge DIY
  culture, ~18k modules on ModularGrid, growth +12% YoY (decelerating).
  500-series — VPR certification is free (sample module + agreement); ~500+
  modules existed by 2020; price ladder: DIY kit $150–400 (DIYRE CP5 $150,
  Hairball Lola kit $375) → assembled boutique $700–1,200 (API 525 $725,
  AML ez1073-500 ~$893) → recallable analog $2,400–3,000 (WesAudio
  ngBusComp $2,999). EU CE/EMC self-declaration for simple analog:
  ~$600–1,500. Kit economics (DIYRE, documented): retail/distribution/
  assembly dominate cost, not BOM; kits avoid returns; PCB setup fees
  amortize via batch/crowdfund.
- **Naming is hard law**: UA owns "1176"/"LA-2A" (Bomb Factory was forced to
  rename to BF76/BF-2A); "Pultec" belongs to Pulse Techniques (acquired by
  Wolff Audio, Sept 2025); UA itself renamed its Space Echo plugin "Galaxy
  Tape Echo" when the Roland license lapsed. Products use the
  digit-shuffle/"-style" convention (FET-76, Black 76, VLA-2A pattern).
  In-repo circuit files may cite real units descriptively; product SKUs may
  not.

## 4. Product lines

### Line 1 — Plugins (software twin of everything below)

CLAP-first shell over the kernel (CLAP C API + clap-wrapper → VST3/AUv2),
metering buffer already in the engine, layout crate for panel UI. Engine
prerequisites (already on the engine roadmap): oversampling latency
reporting, preset serialization, parameter smoothing audit.

First wave (post-M1): the dynamics units being fixed now — FET leveler
(1176-style), opto leveler (LA-2A-style), passive EQ (EQP-style) — each
shipping with its validation report (SPICE overlays + acceptance curves vs
published hardware behavior) as a public artifact. Free tier = engine +
`.pedal` source (AGPL); paid tier = polished binaries/presets/support
($29–79 street; subscriptions later if the catalog justifies it).

### Line 2 — Eurorack (FIRST HARDWARE; owner direction)

Why first: the engine already speaks this domain — the blockwise solver was
built for the TB-303 diode ladder (named test case), a TB303 product exists
in the pro regression matrix, an 808-kick-from-first-principles test is
in-tree, and CEM3340/AS3340 VCO + SSM2164 VCA (post-F8) + comparators/S&H
are DSL primitives. Eurorack is also the cheapest hardware on-ramp: ±12 V
single-ended, no transformers/balanced I/O (sidesteps the transformer
step-down bug), no certification gate, kit-friendly, and the owner has a
rack for ground-truth A/B.

Candidate module sequence (each = one `.pedal` source → VCV module → KiCad
→ PCB):
1. **Acid voice filter** — 303-style diode-ladder VCF (+ the 303's
   characteristic envelope/accent circuit as a companion or combined
   module). Crowded clone market (x0x heart, Behringer TD-3, AJH) but
   nobody ships "SPICE-validated, twin-in-your-DAW"; the published
   filter-curve overlays ARE the differentiation.
2. **Analog drum voices** — TR-x00-lineage circuits (808 kick bridged-T
   resonator first — already modeled in-tree — then snare/hat class
   voices). "TR-1000-class" voice bank as a multi-module family.
3. **Dynamics module** — the opto or FET leveler circuit in Eurorack format
   (compressors are underserved in Eurorack; reuses Line-1 work directly).
Format facts to encode in the engine: ±12 V rails, 1 V/oct (DSL has
cv_pitch/tempco), ~10 Vpp signal levels, Doepfer mechanical/power spec — a
`format "eurorack"` lint mirroring the planned 500-series one.

Distribution: kits first (DIYRE economics; no returns problem; batch-driven
PCB costs), assembled later via builder partners (CAPI/ZenPro pattern).

### Line 3 — VCV Rack modules (software face of Line 2)

Each Eurorack module ships a VCV twin into the exact "circuit-exact" gap the
modular research documented. Strategy: free tier modules under AGPL (VCV's
exception permits free plugins any license — ours stay AGPL) to seed
reputation; paid versions on the VCV Library once the brand carries it
(commercial license negotiated with VCV; split unpublished — contact VCV).
The marketing asset: side-by-side spectrum/scope captures of the VCV module
vs the physical module vs SPICE.

### Line 4 — 500-series (after M1 + line-level fidelity gate)

The original ambition, sequenced after Eurorack because it needs the Phase-2
engine work (transformer step-down fix, +4 dBu/±16 V conventions, balanced
I/O). VPR certification is free. Entry product: the FET leveler as kit
($200–400 target, Hairball/DIYRE band) + assembled ($700–1,000), each unit
shipping with its twin plugin license (the SSL-UC1/Chase-Bliss bundling
pattern, but with the stronger shared-model claim). EQ module follows
(passive-EQ circuit post-F7/F10).

## 5. Milestones (supersedes the plan's M-gates with hardware reality)

- **M0 — engine credible** (in progress): the 9-test acceptance suite green.
  Status today: 3 green (determinism F1, envelope→JFET F4, DC-cascade
  polarity F3), switched components fixed (F7), claiming fix in flight
  (F10); remaining F2/F5/F6/F8/F9.
- **M1 — reference models validated**: 1176-style + LA-2A-style + EQP-style
  pass acceptance curves vs published hardware specs; validation reports
  publishable. Plus: 303 ladder + 808 kick get the same acceptance-curve
  treatment (they're ahead — circuits already in-tree).
- **M2 — software products**: CLAP shell prototype hosting two units; VCV
  module prototype of the 303 filter; latency reporting + presets done.
- **M3 — hardware pilot**: 303-filter (or kick-voice) Eurorack module —
  `.pedal` → KiCad → fab → measured against its own simulation in the
  owner's rack; publish the comparison whether flattering or not (the
  honesty is the marketing). Layout crate hardening is the gating tech-debt
  (17 lint suppressions, schematic-only today; PCB layout stays manual in
  KiCad).
- **M4 — product decision**: kit launch (Eurorack), plugin sales begin,
  500-series go/no-go with real data.

## 6. Risks

- **Engine fidelity gap is real today** (audit): M0/M1 are genuinely gating;
  shipping early would burn the only differentiator (trust).
- **nih-plug maintenance-mode** → keep the shell thin on the CLAP C API.
- **Price-sensitive segments** (modular discount culture, $29 plugin
  street) → validation must convert to *audible* demos, not whitepapers
  alone.
- **Naming/trademark** — strict "-style"/original-name policy from day one.
- **One-person bandwidth** — every line leans on the same kernel and the
  same `.pedal` sources by design; sequencing is serial on purpose.
- **AGPL novelty in audio** — no precedent means educating commercial
  licensees; HISE's terms are the template to copy.
