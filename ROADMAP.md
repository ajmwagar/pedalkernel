# PedalKernel Roadmap

## Phase 2: New Component Types (Requires Code)

| Component | DSL Syntax | Use Case | Effort |
|-----------|------------|----------|--------|
| **N-MOSFET** | `nmos(2n7000)` | Fulltone OCD clipping, modern drives | Medium |
| **P-MOSFET** | `pmos(bs250)` | Some boost circuits | Medium |
| **Zener diode** | `zener(5.1v)` | Voltage regulation, some clipping | Low |

## Phase 3: Advanced Modeling

| Feature | Description |
|---------|-------------|
| **Slew rate limiting** | Use `OpAmpType::slew_rate()` in WDF to model HF compression (LM308 vs TL072) |
| **OTA behavior** | CA3080 has current-controlled gain - different from voltage-mode op-amps |
| **BBD abstraction** | `bbd(mn3207)` that expands to delay line + clock driver for DM-2/CE-2 |

## Phase 4: More Pedals to Fix

These pro pedals could be made more accurate:

| Pedal | Needs |
|-------|-------|
| **TS808** (pro) | Update to use `opamp(jrc4558)` |
| **Klon** (pro) | Update to use `opamp(tl072)` |
| **Fulltone OCD** | Needs MOSFET support for accurate clipping |
| **Keeley Compressor** | Uses Ross/Dyna Comp OTA topology |
| **Phase 90** | Could benefit from JFET VCA modeling |

## Phase 5: Verification & Testing

1. Cross-reference each pedal against published schematics
2. A/B test WDF output against real pedal recordings
3. Add schematic version comments (e.g., "Ram's Head 1973" vs "Civil War")

## Quick Wins (No Code)

- Add `.pedalhw` files to all pro pedals with real part numbers
- Update pro pedals to use typed op-amps where applicable
