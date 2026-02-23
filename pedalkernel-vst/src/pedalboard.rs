//! 6-slot pedalboard architecture for Pro UI
//!
//! Signal chain: input → Slot 1 → Slot 2 → ... → Slot 6 → [Amp] → output
//! Each slot has: pedal selector + 3 knobs + bypass
//! Amp section has: amp selector + gain + tone + volume + bypass

use atomic_float::AtomicF32;
use nih_plug::prelude::*;
use std::sync::atomic::{AtomicBool, AtomicU8, Ordering};
use std::sync::Arc;

// Re-export for UI crate to use
pub use atomic_float;

/// Number of pedal slots in the pedalboard
pub const NUM_SLOTS: usize = 6;
/// Number of knobs per pedal slot
pub const KNOBS_PER_SLOT: usize = 3;
/// Number of amp controls
pub const AMP_KNOBS: usize = 3;

/// Shared state for a single pedal slot
/// Uses Arc atomics so UI can share the same state
#[derive(Debug)]
pub struct SlotState {
    /// Which pedal is selected (index into pedal library)
    pub pedal_index: Arc<AtomicU8>,
    /// Whether this slot is bypassed
    pub bypassed: Arc<AtomicBool>,
    /// Current knob values (0.0 - 1.0)
    pub knobs: [Arc<AtomicF32>; KNOBS_PER_SLOT],
}

impl Default for SlotState {
    fn default() -> Self {
        Self {
            pedal_index: Arc::new(AtomicU8::new(0)),
            bypassed: Arc::new(AtomicBool::new(false)),
            knobs: std::array::from_fn(|_| Arc::new(AtomicF32::new(0.5))),
        }
    }
}

impl Clone for SlotState {
    fn clone(&self) -> Self {
        Self {
            pedal_index: Arc::clone(&self.pedal_index),
            bypassed: Arc::clone(&self.bypassed),
            knobs: std::array::from_fn(|i| Arc::clone(&self.knobs[i])),
        }
    }
}

/// Shared state for the amp section
/// Uses Arc atomics so UI can share the same state
#[derive(Debug)]
pub struct AmpState {
    /// Which amp is selected (index into amp library)
    pub amp_index: Arc<AtomicU8>,
    /// Whether the amp is bypassed
    pub bypassed: Arc<AtomicBool>,
    /// Amp knob values: [Gain, Tone, Volume]
    pub knobs: [Arc<AtomicF32>; AMP_KNOBS],
}

impl Default for AmpState {
    fn default() -> Self {
        Self {
            amp_index: Arc::new(AtomicU8::new(0)),
            bypassed: Arc::new(AtomicBool::new(false)),
            knobs: [
                Arc::new(AtomicF32::new(0.5)), // Gain
                Arc::new(AtomicF32::new(0.5)), // Tone
                Arc::new(AtomicF32::new(0.7)), // Volume
            ],
        }
    }
}

impl Clone for AmpState {
    fn clone(&self) -> Self {
        Self {
            amp_index: Arc::clone(&self.amp_index),
            bypassed: Arc::clone(&self.bypassed),
            knobs: std::array::from_fn(|i| Arc::clone(&self.knobs[i])),
        }
    }
}

/// Shared pedalboard state between audio thread and UI
#[derive(Debug)]
pub struct PedalboardState {
    /// State for each of the 6 slots
    pub slots: [SlotState; NUM_SLOTS],
    /// Amp section state
    pub amp: AmpState,
    /// Names of all available pedals (for UI dropdown)
    pub pedal_names: Vec<String>,
    /// Names of all available amps (for UI dropdown)
    pub amp_names: Vec<String>,
    /// Labels for each pedal's controls (indexed by pedal index)
    /// Each pedal has up to KNOBS_PER_SLOT labels
    pub pedal_labels: Vec<[Option<String>; KNOBS_PER_SLOT]>,
    /// Peak meter value for visualization
    pub peak_meter: Arc<AtomicF32>,
}

impl PedalboardState {
    pub fn new(
        pedal_names: Vec<String>,
        pedal_labels: Vec<[Option<String>; KNOBS_PER_SLOT]>,
        amp_names: Vec<String>,
    ) -> Self {
        Self {
            slots: std::array::from_fn(|_| SlotState::default()),
            amp: AmpState::default(),
            pedal_names,
            amp_names,
            pedal_labels,
            peak_meter: Arc::new(AtomicF32::new(0.0)),
        }
    }

    /// Get the knob labels for a given slot based on its selected pedal
    pub fn get_slot_labels(&self, slot: usize) -> [Option<String>; KNOBS_PER_SLOT] {
        let pedal_idx = self.slots[slot].pedal_index.load(Ordering::Relaxed) as usize;
        self.pedal_labels
            .get(pedal_idx)
            .cloned()
            .unwrap_or_default()
    }
}

/// Parameters for the 6-slot pedalboard + amp
///
/// Total params: 6 selectors + 18 knobs + 6 bypasses + 1 amp selector + 3 amp knobs + 1 amp bypass = 35 parameters
#[derive(Params)]
pub struct PedalboardParams {
    // ═══════════════════════════════════════════════════════════════════════════
    // Slot 1
    // ═══════════════════════════════════════════════════════════════════════════
    #[id = "s1_pedal"]
    pub slot1_pedal: IntParam,
    #[id = "s1_bypass"]
    pub slot1_bypass: BoolParam,
    #[id = "s1_k1"]
    pub slot1_knob1: FloatParam,
    #[id = "s1_k2"]
    pub slot1_knob2: FloatParam,
    #[id = "s1_k3"]
    pub slot1_knob3: FloatParam,

    // ═══════════════════════════════════════════════════════════════════════════
    // Slot 2
    // ═══════════════════════════════════════════════════════════════════════════
    #[id = "s2_pedal"]
    pub slot2_pedal: IntParam,
    #[id = "s2_bypass"]
    pub slot2_bypass: BoolParam,
    #[id = "s2_k1"]
    pub slot2_knob1: FloatParam,
    #[id = "s2_k2"]
    pub slot2_knob2: FloatParam,
    #[id = "s2_k3"]
    pub slot2_knob3: FloatParam,

    // ═══════════════════════════════════════════════════════════════════════════
    // Slot 3
    // ═══════════════════════════════════════════════════════════════════════════
    #[id = "s3_pedal"]
    pub slot3_pedal: IntParam,
    #[id = "s3_bypass"]
    pub slot3_bypass: BoolParam,
    #[id = "s3_k1"]
    pub slot3_knob1: FloatParam,
    #[id = "s3_k2"]
    pub slot3_knob2: FloatParam,
    #[id = "s3_k3"]
    pub slot3_knob3: FloatParam,

    // ═══════════════════════════════════════════════════════════════════════════
    // Slot 4
    // ═══════════════════════════════════════════════════════════════════════════
    #[id = "s4_pedal"]
    pub slot4_pedal: IntParam,
    #[id = "s4_bypass"]
    pub slot4_bypass: BoolParam,
    #[id = "s4_k1"]
    pub slot4_knob1: FloatParam,
    #[id = "s4_k2"]
    pub slot4_knob2: FloatParam,
    #[id = "s4_k3"]
    pub slot4_knob3: FloatParam,

    // ═══════════════════════════════════════════════════════════════════════════
    // Slot 5
    // ═══════════════════════════════════════════════════════════════════════════
    #[id = "s5_pedal"]
    pub slot5_pedal: IntParam,
    #[id = "s5_bypass"]
    pub slot5_bypass: BoolParam,
    #[id = "s5_k1"]
    pub slot5_knob1: FloatParam,
    #[id = "s5_k2"]
    pub slot5_knob2: FloatParam,
    #[id = "s5_k3"]
    pub slot5_knob3: FloatParam,

    // ═══════════════════════════════════════════════════════════════════════════
    // Slot 6
    // ═══════════════════════════════════════════════════════════════════════════
    #[id = "s6_pedal"]
    pub slot6_pedal: IntParam,
    #[id = "s6_bypass"]
    pub slot6_bypass: BoolParam,
    #[id = "s6_k1"]
    pub slot6_knob1: FloatParam,
    #[id = "s6_k2"]
    pub slot6_knob2: FloatParam,
    #[id = "s6_k3"]
    pub slot6_knob3: FloatParam,

    // ═══════════════════════════════════════════════════════════════════════════
    // Amp Section (after all pedal slots)
    // ═══════════════════════════════════════════════════════════════════════════
    #[id = "amp_type"]
    pub amp_type: IntParam,
    #[id = "amp_bypass"]
    pub amp_bypass: BoolParam,
    #[id = "amp_gain"]
    pub amp_gain: FloatParam,
    #[id = "amp_tone"]
    pub amp_tone: FloatParam,
    #[id = "amp_volume"]
    pub amp_volume: FloatParam,
}

impl PedalboardParams {
    pub fn new(state: Arc<PedalboardState>) -> Self {
        let max_pedal = (state.pedal_names.len() as i32 - 1).max(0);
        let max_amp = (state.amp_names.len() as i32 - 1).max(0);

        // Helper to create pedal selector
        let make_selector = |slot: usize, state: &Arc<PedalboardState>| {
            let s = state.clone();
            IntParam::new(
                &format!("Slot {} Pedal", slot + 1),
                0,
                IntRange::Linear { min: 0, max: max_pedal },
            )
            .with_value_to_string(Arc::new(move |v| {
                s.pedal_names
                    .get(v as usize)
                    .cloned()
                    .unwrap_or_else(|| format!("#{v}"))
            }))
        };

        // Helper to create bypass
        let make_bypass = |slot: usize| {
            BoolParam::new(&format!("Slot {} Bypass", slot + 1), false)
        };

        // Helper to create knob
        let make_knob = |slot: usize, knob: usize, state: &Arc<PedalboardState>| {
            let s = state.clone();
            FloatParam::new(
                &format!("Slot {} Knob {}", slot + 1, knob + 1),
                0.5,
                FloatRange::Linear { min: 0.0, max: 1.0 },
            )
            .with_smoother(SmoothingStyle::Linear(20.0))
            .with_value_to_string(Arc::new(move |v: f32| {
                let labels = s.get_slot_labels(slot);
                match &labels[knob] {
                    Some(label) => format!("{}: {:.2}", label, v),
                    None => format!("{:.2}", v),
                }
            }))
        };

        // Amp selector
        let amp_state = state.clone();
        let amp_selector = IntParam::new("Amp", 0, IntRange::Linear { min: 0, max: max_amp })
            .with_value_to_string(Arc::new(move |v| {
                amp_state
                    .amp_names
                    .get(v as usize)
                    .cloned()
                    .unwrap_or_else(|| format!("Amp #{v}"))
            }));

        Self {
            // Slot 1
            slot1_pedal: make_selector(0, &state),
            slot1_bypass: make_bypass(0),
            slot1_knob1: make_knob(0, 0, &state),
            slot1_knob2: make_knob(0, 1, &state),
            slot1_knob3: make_knob(0, 2, &state),
            // Slot 2
            slot2_pedal: make_selector(1, &state),
            slot2_bypass: make_bypass(1),
            slot2_knob1: make_knob(1, 0, &state),
            slot2_knob2: make_knob(1, 1, &state),
            slot2_knob3: make_knob(1, 2, &state),
            // Slot 3
            slot3_pedal: make_selector(2, &state),
            slot3_bypass: make_bypass(2),
            slot3_knob1: make_knob(2, 0, &state),
            slot3_knob2: make_knob(2, 1, &state),
            slot3_knob3: make_knob(2, 2, &state),
            // Slot 4
            slot4_pedal: make_selector(3, &state),
            slot4_bypass: make_bypass(3),
            slot4_knob1: make_knob(3, 0, &state),
            slot4_knob2: make_knob(3, 1, &state),
            slot4_knob3: make_knob(3, 2, &state),
            // Slot 5
            slot5_pedal: make_selector(4, &state),
            slot5_bypass: make_bypass(4),
            slot5_knob1: make_knob(4, 0, &state),
            slot5_knob2: make_knob(4, 1, &state),
            slot5_knob3: make_knob(4, 2, &state),
            // Slot 6
            slot6_pedal: make_selector(5, &state),
            slot6_bypass: make_bypass(5),
            slot6_knob1: make_knob(5, 0, &state),
            slot6_knob2: make_knob(5, 1, &state),
            slot6_knob3: make_knob(5, 2, &state),
            // Amp section
            amp_type: amp_selector,
            amp_bypass: BoolParam::new("Amp Bypass", false),
            amp_gain: FloatParam::new("Amp Gain", 0.5, FloatRange::Linear { min: 0.0, max: 1.0 })
                .with_smoother(SmoothingStyle::Linear(20.0))
                .with_value_to_string(Arc::new(|v| format!("Gain: {:.0}%", v * 100.0))),
            amp_tone: FloatParam::new("Amp Tone", 0.5, FloatRange::Linear { min: 0.0, max: 1.0 })
                .with_smoother(SmoothingStyle::Linear(20.0))
                .with_value_to_string(Arc::new(|v| format!("Tone: {:.0}%", v * 100.0))),
            amp_volume: FloatParam::new("Amp Volume", 0.7, FloatRange::Linear { min: 0.0, max: 1.0 })
                .with_smoother(SmoothingStyle::Linear(20.0))
                .with_value_to_string(Arc::new(|v| format!("Vol: {:.0}%", v * 100.0))),
        }
    }

    /// Get pedal selector for a slot
    pub fn get_pedal_selector(&self, slot: usize) -> &IntParam {
        match slot {
            0 => &self.slot1_pedal,
            1 => &self.slot2_pedal,
            2 => &self.slot3_pedal,
            3 => &self.slot4_pedal,
            4 => &self.slot5_pedal,
            5 => &self.slot6_pedal,
            _ => &self.slot1_pedal,
        }
    }

    /// Get bypass for a slot
    pub fn get_bypass(&self, slot: usize) -> &BoolParam {
        match slot {
            0 => &self.slot1_bypass,
            1 => &self.slot2_bypass,
            2 => &self.slot3_bypass,
            3 => &self.slot4_bypass,
            4 => &self.slot5_bypass,
            5 => &self.slot6_bypass,
            _ => &self.slot1_bypass,
        }
    }

    /// Get knobs for a slot (returns array of 3 knobs)
    pub fn get_knobs(&self, slot: usize) -> [&FloatParam; KNOBS_PER_SLOT] {
        match slot {
            0 => [&self.slot1_knob1, &self.slot1_knob2, &self.slot1_knob3],
            1 => [&self.slot2_knob1, &self.slot2_knob2, &self.slot2_knob3],
            2 => [&self.slot3_knob1, &self.slot3_knob2, &self.slot3_knob3],
            3 => [&self.slot4_knob1, &self.slot4_knob2, &self.slot4_knob3],
            4 => [&self.slot5_knob1, &self.slot5_knob2, &self.slot5_knob3],
            5 => [&self.slot6_knob1, &self.slot6_knob2, &self.slot6_knob3],
            _ => [&self.slot1_knob1, &self.slot1_knob2, &self.slot1_knob3],
        }
    }
}
