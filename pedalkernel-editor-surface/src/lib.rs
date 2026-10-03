//! Retained native frames for VST editor presentation.
//!
//! This crate owns capture only. Local Canvas import and remote video encoding
//! are consumers of the same immutable frame lease and deliberately live
//! elsewhere. A consumer must retain the frame until its replacement has been
//! imported or submitted; dropping the displayed frame first causes flicker.

use std::time::{SystemTime, UNIX_EPOCH};

use serde::{Deserialize, Serialize};
use thiserror::Error;

pub const EDITOR_SURFACE_VERSION: u16 = 1;

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct EditorSelector {
    pub process_id: u32,
    pub title_contains: String,
}

impl EditorSelector {
    pub fn validate(&self) -> Result<(), CaptureError> {
        if self.process_id == 0 {
            return Err(CaptureError::InvalidSelector(
                "process_id must not be zero".into(),
            ));
        }
        if self.title_contains.trim().is_empty() {
            return Err(CaptureError::InvalidSelector(
                "title_contains must not be empty".into(),
            ));
        }
        Ok(())
    }

    pub(crate) fn title_matches(&self, title: &str) -> bool {
        title
            .to_lowercase()
            .contains(&self.title_contains.to_lowercase())
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CaptureBackend {
    ScreenCaptureKit,
    XCompositeDri3,
}

impl CaptureBackend {
    pub const fn zero_copy(self) -> bool {
        true
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PixelFormat {
    Bgra8Unorm,
    Bgrx8Unorm,
}

impl PixelFormat {
    pub const fn bytes_per_pixel(self) -> u32 {
        4
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum NativeHandleDescriptor {
    IoSurface {
        id: u32,
    },
    DmaBuf {
        drm_fourcc: u32,
        /// `None` means the DRI3 v1 server did not report a modifier. It must
        /// not be silently treated as linear by an importer.
        modifier: Option<u64>,
        offset: u32,
    },
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FrameDescriptor {
    pub protocol: u16,
    pub sequence: u64,
    pub captured_at_unix_ns: u64,
    pub width: u32,
    pub height: u32,
    pub stride: u32,
    pub pixel_format: PixelFormat,
    pub handle: NativeHandleDescriptor,
}

impl FrameDescriptor {
    pub fn validate(&self) -> Result<(), CaptureError> {
        if self.protocol != EDITOR_SURFACE_VERSION {
            return Err(CaptureError::InvalidFrame(format!(
                "unsupported editor surface protocol {}",
                self.protocol
            )));
        }
        if self.width == 0 || self.height == 0 {
            return Err(CaptureError::InvalidFrame(
                "frame dimensions must be non-zero".into(),
            ));
        }
        let packed_stride = self
            .width
            .checked_mul(self.pixel_format.bytes_per_pixel())
            .ok_or_else(|| CaptureError::InvalidFrame("frame stride overflow".into()))?;
        if self.stride < packed_stride {
            return Err(CaptureError::InvalidFrame(format!(
                "stride {} is smaller than packed row size {packed_stride}",
                self.stride
            )));
        }
        Ok(())
    }
}

pub trait FrameLease {
    fn descriptor(&self) -> &FrameDescriptor;
}

pub trait EditorCapture {
    type Frame: FrameLease;

    fn backend(&self) -> CaptureBackend;
    fn selector(&self) -> &EditorSelector;
    fn capture(&mut self) -> Result<Self::Frame, CaptureError>;
}

#[derive(Debug, Error, PartialEq, Eq)]
pub enum CaptureError {
    #[error("invalid editor selector: {0}")]
    InvalidSelector(String),
    #[error("no editor window matched PID {process_id} and title containing {title:?}")]
    WindowNotFound { process_id: u32, title: String },
    #[error("more than one editor window matched PID {process_id} and title containing {title:?}")]
    AmbiguousWindow { process_id: u32, title: String },
    #[error("invalid captured frame: {0}")]
    InvalidFrame(String),
    #[error("{backend} capture failed: {message}")]
    Platform {
        backend: &'static str,
        message: String,
    },
}

impl CaptureError {
    pub(crate) fn platform(backend: &'static str, error: impl std::fmt::Display) -> Self {
        Self::Platform {
            backend,
            message: error.to_string(),
        }
    }
}

pub(crate) fn capture_timestamp_ns() -> u64 {
    let nanos = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos();
    u64::try_from(nanos).unwrap_or(u64::MAX)
}

#[cfg(target_os = "linux")]
mod linux;
#[cfg(target_os = "linux")]
pub use linux::{LinuxEditorCapture, LinuxEditorFrame};
#[cfg(target_os = "linux")]
pub type NativeEditorCapture = LinuxEditorCapture;
#[cfg(target_os = "linux")]
pub type NativeEditorFrame = LinuxEditorFrame;

#[cfg(target_os = "macos")]
mod macos;
#[cfg(target_os = "macos")]
pub use macos::{MacEditorCapture, MacEditorFrame};
#[cfg(target_os = "macos")]
pub type NativeEditorCapture = MacEditorCapture;
#[cfg(target_os = "macos")]
pub type NativeEditorFrame = MacEditorFrame;

#[cfg(test)]
mod tests {
    use super::*;

    fn selector() -> EditorSelector {
        EditorSelector {
            process_id: 42,
            title_contains: "Surge XT".into(),
        }
    }

    #[test]
    fn selector_matching_is_case_insensitive_and_bounded_to_pid_elsewhere() {
        let selector = selector();
        assert!(selector.title_matches("surge xt - VST3"));
        assert!(!selector.title_matches("Dexed - VST3"));
    }

    #[test]
    fn selector_rejects_missing_identity() {
        assert!(EditorSelector {
            process_id: 0,
            title_contains: "Surge".into(),
        }
        .validate()
        .is_err());
        assert!(EditorSelector {
            process_id: 42,
            title_contains: "  ".into(),
        }
        .validate()
        .is_err());
    }

    #[test]
    fn frame_rejects_torn_or_misdescribed_rows() {
        let frame = FrameDescriptor {
            protocol: EDITOR_SURFACE_VERSION,
            sequence: 1,
            captured_at_unix_ns: 1,
            width: 100,
            height: 50,
            stride: 399,
            pixel_format: PixelFormat::Bgra8Unorm,
            handle: NativeHandleDescriptor::IoSurface { id: 9 },
        };
        assert!(matches!(
            frame.validate(),
            Err(CaptureError::InvalidFrame(_))
        ));
    }

    #[test]
    fn descriptors_round_trip_without_serializing_native_handles() {
        let frame = FrameDescriptor {
            protocol: EDITOR_SURFACE_VERSION,
            sequence: 7,
            captured_at_unix_ns: 11,
            width: 960,
            height: 640,
            stride: 3_840,
            pixel_format: PixelFormat::Bgra8Unorm,
            handle: NativeHandleDescriptor::IoSurface { id: 99 },
        };
        let json = serde_json::to_string(&frame).unwrap();
        assert_eq!(
            serde_json::from_str::<FrameDescriptor>(&json).unwrap(),
            frame
        );
    }
}
