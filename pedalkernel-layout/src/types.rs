//! Output types for the schematic layout engine.
//!
//! All types derive [`serde::Serialize`] and [`serde::Deserialize`] so the
//! layout can be written to `.pedal_layout` JSON and loaded at runtime by the
//! signal-flow shader.

use serde::{Deserialize, Serialize};

/// Complete schematic layout — the output of [`crate::generate_layout`].
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Layout {
    /// Format version (currently 1).
    pub version: u32,
    /// Canvas bounds in layout units.
    pub bounds: Bounds,
    /// Positioned components with symbol and group assignments.
    pub components: Vec<PlacedComponent>,
    /// Routed wires connecting component pins.
    pub wires: Vec<Wire>,
    /// Functional groups (gain stages, tone stacks, etc.).
    pub groups: Vec<Group>,
    /// Supply rail positions.
    pub supply_rails: Vec<SupplyRail>,
    /// Component indices in signal-path order (input → output).
    pub signal_path_order: Vec<usize>,
}

/// Canvas dimensions.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct Bounds {
    pub width: f32,
    pub height: f32,
}

/// A component placed on the schematic.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PlacedComponent {
    /// Component ID from the DSL (e.g., `"V1a"`, `"R_plate"`).
    pub name: String,
    /// Component kind string (e.g., `"triode"`, `"resistor"`).
    pub kind: String,
    /// X position in layout units.
    pub x: f32,
    /// Y position in layout units.
    pub y: f32,
    /// Rotation in degrees (0, 90, 180, 270).
    pub orientation: u16,
    /// Symbol identifier for rendering.
    pub symbol: String,
    /// Functional group this component belongs to.
    pub group: String,
    /// Human-readable value label (e.g., `"100k"`, `"220nF"`).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub label: Option<String>,
    /// If this component is monitored, the monitor index for glow mapping.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub monitor_index: Option<usize>,
}

/// A routed wire (sequence of orthogonal segments).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Wire {
    /// Net name (e.g., `"net_3"`, `"vcc"`, `"gnd"`).
    pub net: String,
    /// Ordered waypoints `[x, y]` defining the wire path.
    pub points: Vec<[f32; 2]>,
    /// Whether this wire is on the primary signal path.
    pub signal_path: bool,
    /// Monitor index at the start of this wire segment (for glow interpolation).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub monitor_index_start: Option<usize>,
    /// Monitor index at the end of this wire segment.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub monitor_index_end: Option<usize>,
}

/// A functional group bounding box.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Group {
    /// Internal group name (e.g., `"preamp_1"`).
    pub name: String,
    /// Human-readable label (e.g., `"V1a Preamp"`).
    pub label: String,
    /// Bounding rectangle.
    pub bounds: GroupBounds,
}

/// Bounding rectangle for a group.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct GroupBounds {
    pub x: f32,
    pub y: f32,
    pub w: f32,
    pub h: f32,
}

/// A supply rail (horizontal line across the schematic).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SupplyRail {
    /// Rail name (`"vcc"` or `"gnd"`).
    pub name: String,
    /// Y position of this rail.
    pub y: f32,
}

// ---------------------------------------------------------------------------
// Internal placement helpers
// ---------------------------------------------------------------------------

/// Point in 2D layout space.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point {
    pub x: f32,
    pub y: f32,
}

impl Point {
    pub fn new(x: f32, y: f32) -> Self {
        Self { x, y }
    }
}

/// Axis-aligned rectangle (used for overlap detection).
#[derive(Debug, Clone, Copy)]
pub struct Rect {
    pub x: f32,
    pub y: f32,
    pub w: f32,
    pub h: f32,
}

impl Rect {
    pub fn overlaps(&self, other: &Rect) -> bool {
        self.x < other.x + other.w
            && self.x + self.w > other.x
            && self.y < other.y + other.h
            && self.y + self.h > other.y
    }

    pub fn contains_point(&self, p: Point) -> bool {
        p.x >= self.x && p.x <= self.x + self.w && p.y >= self.y && p.y <= self.y + self.h
    }
}
