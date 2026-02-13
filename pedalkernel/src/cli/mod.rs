pub mod process;

#[cfg(all(feature = "jack-rt", feature = "tui"))]
pub mod tui_widgets;

#[cfg(all(feature = "jack-rt", feature = "tui"))]
pub mod tui;

#[cfg(all(feature = "jack-rt", feature = "tui"))]
pub mod board_tui;

#[cfg(all(feature = "jack-rt", feature = "tui"))]
pub mod folder_tui;
