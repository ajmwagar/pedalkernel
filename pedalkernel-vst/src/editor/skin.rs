//! Skin data model: maps pedal metadata to visual properties.

use nih_plug_egui::egui::Color32;

use crate::PedalMeta;
use super::enclosure_color_for_pedal;

/// Visual description for a single pedal.
#[allow(dead_code)]
pub struct PedalSkin {
    /// Display name.
    pub name: String,
    /// Enclosure background color.
    pub enclosure_color: Color32,
    /// Labels for each knob slot (None = unused).
    pub labels: [Option<String>; crate::NUM_KNOBS],
    /// Pedal index in the engine list.
    pub pedal_index: usize,
}

impl PedalSkin {
    /// Build a skin from pedal metadata (procedural fallback — no textures).
    pub fn from_meta(index: usize, meta: &PedalMeta) -> Self {
        Self {
            name: meta.name.clone(),
            enclosure_color: enclosure_color_for_pedal(&meta.name),
            labels: meta.labels.clone(),
            pedal_index: index,
        }
    }
}
