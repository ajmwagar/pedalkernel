//! Embedded GUI editor for the PedalKernel VST plugin.
//!
//! Uses `nih_plug_egui` to render a skeuomorphic pedal interface with
//! pre-rendered textures. The visual identity swaps when the user
//! changes the pedal selector parameter.

mod skin;

use nih_plug::prelude::*;
use nih_plug_egui::egui::{self, Color32, FontId, Rect, Sense, Stroke, StrokeKind, TextureHandle, Vec2};
use nih_plug_egui::{create_egui_editor, EguiState};
use std::sync::Arc;

use crate::{PedalKernelParams, PedalMeta, NUM_KNOBS};
pub use skin::PedalSkin;

/// Persisted editor window state. Stored on the params struct with `#[persist]`.
pub fn default_editor_state() -> Arc<EguiState> {
    EguiState::from_size(400, 700)
}

/// State held by the editor between frames.
#[allow(dead_code)]
pub struct EditorState {
    /// Pre-built skins for every loaded pedal.
    skins: Vec<PedalSkin>,
    /// Cached generic knob texture (procedural, generated at startup).
    knob_texture: Option<TextureHandle>,
    /// LED on texture.
    led_on_texture: Option<TextureHandle>,
    /// LED off texture.
    led_off_texture: Option<TextureHandle>,
    /// Which knob is being dragged (pedal_idx, knob_slot).
    dragging: Option<(usize, usize)>,
}

/// Colors used throughout the editor.
mod colors {
    use nih_plug_egui::egui::Color32;

    pub const ENCLOSURE_DARK: Color32 = Color32::from_rgb(45, 45, 48);
    pub const ENCLOSURE_BORDER: Color32 = Color32::from_rgb(30, 30, 33);
    pub const KNOB_BG: Color32 = Color32::from_rgb(35, 35, 38);
    pub const KNOB_RING: Color32 = Color32::from_rgb(60, 60, 65);
    pub const KNOB_POINTER: Color32 = Color32::from_rgb(240, 240, 240);
    pub const KNOB_POINTER_ACTIVE: Color32 = Color32::from_rgb(255, 200, 60);
    pub const LABEL_TEXT: Color32 = Color32::from_rgb(200, 200, 200);
    pub const NAME_TEXT: Color32 = Color32::from_rgb(255, 255, 255);
    pub const LED_ON: Color32 = Color32::from_rgb(255, 40, 40);
    pub const LED_GLOW: Color32 = Color32::from_rgba_premultiplied(255, 40, 40, 80);
    pub const LED_OFF: Color32 = Color32::from_rgb(60, 20, 20);
    pub const FOOTSWITCH_BG: Color32 = Color32::from_rgb(80, 80, 85);
    pub const FOOTSWITCH_RIM: Color32 = Color32::from_rgb(120, 120, 125);
    pub const SCREW: Color32 = Color32::from_rgb(140, 140, 145);
    pub const VALUE_TEXT: Color32 = Color32::from_rgb(160, 160, 165);
}

/// Build the enclosure color for a given pedal index.
fn enclosure_color_for_pedal(pedal_name: &str) -> Color32 {
    let name = pedal_name.to_lowercase();
    if name.contains("tube screamer") || name.contains("ts808") || name.contains("ts-808") {
        Color32::from_rgb(46, 125, 50) // Ibanez green
    } else if name.contains("fuzz face") {
        Color32::from_rgb(100, 100, 105) // Hammertone grey
    } else if name.contains("big muff") {
        Color32::from_rgb(70, 70, 75) // Silver/dark
    } else if name.contains("dyna comp") {
        Color32::from_rgb(211, 47, 47) // Red
    } else if name.contains("rat") || name.contains("proco") {
        Color32::from_rgb(30, 30, 33) // Black
    } else if name.contains("blues driver") {
        Color32::from_rgb(21, 101, 192) // Blue
    } else if name.contains("klon") || name.contains("centaur") {
        Color32::from_rgb(197, 160, 63) // Gold
    } else if name.contains("delay") {
        Color32::from_rgb(50, 80, 120) // Slate blue
    } else {
        colors::ENCLOSURE_DARK
    }
}

/// Create the egui editor for the plugin.
pub fn create(
    params: Arc<PedalKernelParams>,
    meta: Arc<Vec<PedalMeta>>,
) -> Option<Box<dyn Editor>> {
    let skins: Vec<PedalSkin> = meta
        .iter()
        .enumerate()
        .map(|(i, m)| PedalSkin::from_meta(i, m))
        .collect();

    let egui_state = params.editor_state.clone();

    create_egui_editor(
        egui_state,
        EditorState {
            skins,
            knob_texture: None,
            led_on_texture: None,
            led_off_texture: None,
            dragging: None,
        },
        // Build callback: runs once when editor window opens.
        |_ctx, _state| {},
        // Update callback: runs every frame.
        move |ctx, setter, state| {
            let pedal_idx = params.pedal_type.value() as usize;

            egui::CentralPanel::default()
                .frame(egui::Frame::NONE.fill(Color32::from_rgb(25, 25, 28)))
                .show(ctx, |ui| {
                    let available = ui.available_size();
                    let skin = state.skins.get(pedal_idx);

                    // Draw enclosure background.
                    let enclosure_color = skin
                        .map(|s| s.enclosure_color)
                        .unwrap_or(colors::ENCLOSURE_DARK);
                    draw_enclosure(ui, available, enclosure_color);

                    // Pedal name at top.
                    let pedal_name = params
                        .pedal_type
                        .to_string();
                    draw_pedal_name(ui, available, &pedal_name);

                    // Knobs.
                    let knob_params = [
                        &params.knob1,
                        &params.knob2,
                        &params.knob3,
                        &params.knob4,
                        &params.knob5,
                        &params.knob6,
                    ];

                    // Extract labels into owned data to avoid borrow conflict.
                    let labels: [Option<String>; NUM_KNOBS] = state
                        .skins
                        .get(pedal_idx)
                        .map(|s| s.labels.clone())
                        .unwrap_or_default();

                    let active_knobs: Vec<(usize, String)> = labels
                        .iter()
                        .enumerate()
                        .filter_map(|(i, l)| l.as_ref().map(|label| (i, label.clone())))
                        .collect();

                    if !active_knobs.is_empty() {
                        draw_knobs(
                            ui,
                            available,
                            &active_knobs,
                            &knob_params,
                            setter,
                            pedal_idx,
                            state,
                        );
                    }

                    // LED indicator.
                    draw_led(ui, available, true);

                    // Footswitch.
                    draw_footswitch(ui, available);

                    // Screws.
                    draw_screws(ui, available);
                });
        },
    )
}

/// Draw the enclosure body with rounded corners and border.
fn draw_enclosure(ui: &mut egui::Ui, size: Vec2, color: Color32) {
    let painter = ui.painter();
    let rect = Rect::from_min_size(egui::pos2(0.0, 0.0), size);

    // Main body.
    painter.rect_filled(rect, 12.0, color);

    // Subtle inner shadow at top.
    let highlight = Color32::from_rgba_premultiplied(255, 255, 255, 15);
    painter.rect_stroke(rect.shrink(1.0), 11.0, Stroke::new(1.0, highlight), StrokeKind::Inside);

    // Outer border.
    painter.rect_stroke(rect, 12.0, Stroke::new(2.0, colors::ENCLOSURE_BORDER), StrokeKind::Inside);
}

/// Draw the pedal name centered near the top.
fn draw_pedal_name(ui: &mut egui::Ui, size: Vec2, name: &str) {
    let painter = ui.painter();
    let center_x = size.x / 2.0;
    let y = 50.0;

    painter.text(
        egui::pos2(center_x, y),
        egui::Align2::CENTER_CENTER,
        name,
        FontId::proportional(22.0),
        colors::NAME_TEXT,
    );
}

/// Draw knobs with labels and value display, handling drag interaction.
fn draw_knobs(
    ui: &mut egui::Ui,
    size: Vec2,
    active_knobs: &[(usize, String)],
    knob_params: &[&FloatParam; NUM_KNOBS],
    setter: &ParamSetter,
    pedal_idx: usize,
    state: &mut EditorState,
) {
    let knob_count = active_knobs.len();
    let knob_radius = 32.0;
    let pointer_len = 24.0;

    // Layout: up to 3 per row, centered.
    let cols = if knob_count <= 3 { knob_count } else { 3 };
    let rows = (knob_count + cols - 1) / cols;
    let knob_area_top = 100.0;
    let v_spacing = 130.0;

    for (ki, (slot, label)) in active_knobs.iter().enumerate() {
        let slot = *slot;
        let row = ki / cols;
        let col = ki % cols;
        let items_in_row = if row < rows - 1 {
            cols
        } else {
            knob_count - (rows - 1) * cols
        };
        let row_offset = size.x / (items_in_row as f32 + 1.0);
        let cx = row_offset * (col as f32 + 1.0);
        let cy = knob_area_top + v_spacing * row as f32;

        let value = knob_params[slot].modulated_normalized_value();

        // Knob interaction area.
        let knob_rect = Rect::from_center_size(
            egui::pos2(cx, cy),
            Vec2::splat(knob_radius * 2.0 + 10.0),
        );
        let response = ui.allocate_rect(knob_rect, Sense::click_and_drag());

        let is_active = response.dragged() || response.hovered();

        // Handle drag.
        if response.drag_started() {
            state.dragging = Some((pedal_idx, slot));
            setter.begin_set_parameter(knob_params[slot]);
        }
        if response.dragged() {
            let delta = -response.drag_delta().y / 200.0;
            let new_val = (value + delta).clamp(0.0, 1.0);
            setter.set_parameter_normalized(knob_params[slot], new_val);
        }
        if response.drag_stopped() {
            setter.end_set_parameter(knob_params[slot]);
            state.dragging = None;
        }

        // Double-click to reset.
        if response.double_clicked() {
            setter.set_parameter_normalized(knob_params[slot], 0.5);
        }

        // Scroll for fine adjustment.
        let scroll = response.hover_pos().map(|_| ui.input(|i| i.smooth_scroll_delta.y));
        if let Some(dy) = scroll {
            if dy != 0.0 {
                let delta = dy / 600.0;
                let new_val = (value + delta).clamp(0.0, 1.0);
                setter.begin_set_parameter(knob_params[slot]);
                setter.set_parameter_normalized(knob_params[slot], new_val);
                setter.end_set_parameter(knob_params[slot]);
            }
        }

        // Draw the knob.
        draw_single_knob(ui, cx, cy, knob_radius, pointer_len, value, is_active);

        // Label below knob.
        let painter = ui.painter();
        painter.text(
            egui::pos2(cx, cy + knob_radius + 18.0),
            egui::Align2::CENTER_CENTER,
            label.as_str(),
            FontId::proportional(12.0),
            colors::LABEL_TEXT,
        );

        // Value readout below label.
        painter.text(
            egui::pos2(cx, cy + knob_radius + 34.0),
            egui::Align2::CENTER_CENTER,
            format!("{:.0}%", value * 100.0),
            FontId::proportional(10.0),
            colors::VALUE_TEXT,
        );
    }
}

/// Draw a single knob (ring + pointer).
fn draw_single_knob(
    ui: &mut egui::Ui,
    cx: f32,
    cy: f32,
    radius: f32,
    pointer_len: f32,
    value: f32,
    active: bool,
) {
    let painter = ui.painter();
    let center = egui::pos2(cx, cy);

    // Knob body (dark circle).
    painter.circle_filled(center, radius, colors::KNOB_BG);
    painter.circle_stroke(center, radius, Stroke::new(2.0, colors::KNOB_RING));

    // Arc track (subtle background arc showing full range).
    let start_angle = std::f32::consts::FRAC_PI_2 + 0.75 * std::f32::consts::PI; // 7 o'clock
    let sweep = 1.5 * std::f32::consts::PI; // 270 degrees

    // Draw tick marks around the arc.
    let track_radius = radius - 4.0;
    for i in 0..=10 {
        let frac = i as f32 / 10.0;
        let angle = start_angle + sweep * frac;
        let inner = track_radius - 3.0;
        let outer = track_radius;
        let cos_a = angle.cos();
        let sin_a = angle.sin();
        painter.line_segment(
            [
                egui::pos2(cx + cos_a * inner, cy + sin_a * inner),
                egui::pos2(cx + cos_a * outer, cy + sin_a * outer),
            ],
            Stroke::new(1.0, Color32::from_rgb(80, 80, 85)),
        );
    }

    // Pointer line.
    let angle = start_angle + sweep * value;
    let pointer_color = if active {
        colors::KNOB_POINTER_ACTIVE
    } else {
        colors::KNOB_POINTER
    };

    let inner_r = 6.0;
    let outer_r = pointer_len;
    painter.line_segment(
        [
            egui::pos2(cx + angle.cos() * inner_r, cy + angle.sin() * inner_r),
            egui::pos2(cx + angle.cos() * outer_r, cy + angle.sin() * outer_r),
        ],
        Stroke::new(2.5, pointer_color),
    );

    // Center dot.
    painter.circle_filled(center, 4.0, colors::KNOB_RING);
}

/// Draw LED indicator.
fn draw_led(ui: &mut egui::Ui, size: Vec2, active: bool) {
    let painter = ui.painter();
    let cx = size.x / 2.0;
    let cy = size.y - 180.0;
    let center = egui::pos2(cx, cy);

    if active {
        // Glow halo.
        painter.circle_filled(center, 12.0, colors::LED_GLOW);
        painter.circle_filled(center, 6.0, colors::LED_ON);
        // Bright center.
        painter.circle_filled(center, 2.5, Color32::from_rgb(255, 180, 180));
    } else {
        painter.circle_filled(center, 6.0, colors::LED_OFF);
    }

    // Chrome bezel.
    painter.circle_stroke(center, 7.0, Stroke::new(1.5, Color32::from_rgb(160, 160, 165)));
}

/// Draw the footswitch.
fn draw_footswitch(ui: &mut egui::Ui, size: Vec2) {
    let painter = ui.painter();
    let cx = size.x / 2.0;
    let cy = size.y - 100.0;
    let center = egui::pos2(cx, cy);
    let radius = 28.0;

    // Shadow.
    painter.circle_filled(
        egui::pos2(cx, cy + 2.0),
        radius + 2.0,
        Color32::from_rgba_premultiplied(0, 0, 0, 40),
    );

    // Switch body.
    painter.circle_filled(center, radius, colors::FOOTSWITCH_BG);

    // Rim highlight.
    painter.circle_stroke(center, radius, Stroke::new(2.0, colors::FOOTSWITCH_RIM));

    // Cross-hatch texture on the switch.
    let inner = radius - 8.0;
    for i in -3..=3 {
        let offset = i as f32 * 5.0;
        let y1 = cy + offset;
        let dx = (inner * inner - offset * offset).max(0.0).sqrt();
        painter.line_segment(
            [egui::pos2(cx - dx, y1), egui::pos2(cx + dx, y1)],
            Stroke::new(0.5, Color32::from_rgb(95, 95, 100)),
        );
    }
}

/// Draw corner screws.
fn draw_screws(ui: &mut egui::Ui, size: Vec2) {
    let painter = ui.painter();
    let inset = 18.0;
    let screw_r = 5.0;
    let positions = [
        egui::pos2(inset, inset),
        egui::pos2(size.x - inset, inset),
        egui::pos2(inset, size.y - inset),
        egui::pos2(size.x - inset, size.y - inset),
    ];

    for pos in positions {
        painter.circle_filled(pos, screw_r, colors::SCREW);
        painter.circle_stroke(pos, screw_r, Stroke::new(0.5, Color32::from_rgb(100, 100, 105)));
        // Phillips head cross.
        painter.line_segment(
            [
                egui::pos2(pos.x - 3.0, pos.y),
                egui::pos2(pos.x + 3.0, pos.y),
            ],
            Stroke::new(1.0, Color32::from_rgb(110, 110, 115)),
        );
        painter.line_segment(
            [
                egui::pos2(pos.x, pos.y - 3.0),
                egui::pos2(pos.x, pos.y + 3.0),
            ],
            Stroke::new(1.0, Color32::from_rgb(110, 110, 115)),
        );
    }
}
