pub mod process;
pub mod validate;

#[cfg(all(feature = "jack-rt", feature = "tui"))]
pub mod tui_widgets;

#[cfg(all(feature = "jack-rt", feature = "tui"))]
pub mod tui;

#[cfg(all(feature = "jack-rt", feature = "tui"))]
pub mod board_tui;

#[cfg(all(feature = "jack-rt", feature = "tui"))]
pub mod folder_tui;

#[cfg(all(feature = "jack-rt", feature = "tui"))]
pub mod pedal_folder_tui;

/// Return the stereo sibling of a JACK port based on the `_1`/`_2` suffix convention.
#[cfg(all(feature = "jack-rt", feature = "tui"))]
pub(crate) fn stereo_pair(port: &str) -> Option<String> {
    if let Some(base) = port.strip_suffix("_1") {
        Some(format!("{base}_2"))
    } else if let Some(base) = port.strip_suffix("_2") {
        Some(format!("{base}_1"))
    } else {
        None
    }
}

/// Format a port name for display, collapsing stereo pairs (e.g. `system:playback_1+2`).
#[cfg(all(feature = "jack-rt", feature = "tui"))]
pub(crate) fn stereo_display(port: &str) -> String {
    if let Some(base) = port.strip_suffix("_1") {
        format!("{base}_1+2")
    } else if let Some(base) = port.strip_suffix("_2") {
        format!("{base}_1+2")
    } else {
        port.to_string()
    }
}
