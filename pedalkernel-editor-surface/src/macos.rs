use std::{ffi::c_void, sync::mpsc};

use apple_cf::iosurface::IOSurface;
use block2::RcBlock;
use objc2::{rc::Retained, AnyThread};
use objc2_core_media::CMSampleBuffer;
use objc2_foundation::NSError;
use objc2_screen_capture_kit::{
    SCContentFilter, SCRunningApplication, SCScreenshotManager, SCShareableContent,
    SCStreamConfiguration, SCWindow,
};

use crate::{
    capture_timestamp_ns, CaptureBackend, CaptureError, EditorCapture, EditorSelector,
    FrameDescriptor, FrameLease, NativeHandleDescriptor, PixelFormat, EDITOR_SURFACE_VERSION,
};

const BACKEND_NAME: &str = "ScreenCaptureKit";
const BGRA_FOURCC: u32 = u32::from_be_bytes(*b"BGRA");

unsafe extern "C" {
    fn CMSampleBufferGetImageBuffer(sample_buffer: *mut c_void) -> *mut c_void;
    fn CVPixelBufferGetIOSurface(pixel_buffer: *mut c_void) -> *mut c_void;
}

/// A complete ScreenCaptureKit frame backed by a retained IOSurface.
///
/// Consumers can import the IOSurface into Metal/WGPU or submit it directly
/// to VideoToolbox. The surface is never serialized by the control protocol.
pub struct MacEditorFrame {
    descriptor: FrameDescriptor,
    surface: IOSurface,
}

impl MacEditorFrame {
    pub fn io_surface(&self) -> &IOSurface {
        &self.surface
    }
}

impl FrameLease for MacEditorFrame {
    fn descriptor(&self) -> &FrameDescriptor {
        &self.descriptor
    }
}

pub struct MacEditorCapture {
    selector: EditorSelector,
    window_id: u32,
    filter: Retained<SCContentFilter>,
    configuration: Retained<SCStreamConfiguration>,
    sequence: u64,
}

impl MacEditorCapture {
    pub fn open(selector: EditorSelector) -> Result<Self, CaptureError> {
        selector.validate()?;
        let content = shareable_content()?;
        let window = select_window(unsafe { content.windows() }, &selector)?;
        let frame = unsafe { window.frame() };
        let width = dimension(frame.size.width, "width")?;
        let height = dimension(frame.size.height, "height")?;
        let window_id = unsafe { window.windowID() };
        let filter = unsafe {
            SCContentFilter::initWithDesktopIndependentWindow(SCContentFilter::alloc(), &window)
        };
        let configuration = unsafe { SCStreamConfiguration::new() };
        unsafe {
            configuration.setWidth(width as usize);
            configuration.setHeight(height as usize);
            configuration.setPixelFormat(BGRA_FOURCC);
            configuration.setShowsCursor(false);
        }

        Ok(Self {
            selector,
            window_id,
            filter,
            configuration,
            sequence: 0,
        })
    }

    pub const fn window_id(&self) -> u32 {
        self.window_id
    }
}

impl EditorCapture for MacEditorCapture {
    type Frame = MacEditorFrame;

    fn backend(&self) -> CaptureBackend {
        CaptureBackend::ScreenCaptureKit
    }

    fn selector(&self) -> &EditorSelector {
        &self.selector
    }

    fn capture(&mut self) -> Result<Self::Frame, CaptureError> {
        let surface = capture_surface(&self.filter, &self.configuration)?;
        if surface.pixel_format() != BGRA_FOURCC {
            return Err(CaptureError::InvalidFrame(format!(
                "expected BGRA IOSurface, received fourcc {:#010x}",
                surface.pixel_format()
            )));
        }
        self.sequence = self.sequence.wrapping_add(1);
        let descriptor = FrameDescriptor {
            protocol: EDITOR_SURFACE_VERSION,
            sequence: self.sequence,
            captured_at_unix_ns: capture_timestamp_ns(),
            width: u32::try_from(surface.width()).map_err(|_| {
                CaptureError::InvalidFrame("IOSurface width does not fit u32".into())
            })?,
            height: u32::try_from(surface.height()).map_err(|_| {
                CaptureError::InvalidFrame("IOSurface height does not fit u32".into())
            })?,
            stride: u32::try_from(surface.bytes_per_row()).map_err(|_| {
                CaptureError::InvalidFrame("IOSurface stride does not fit u32".into())
            })?,
            pixel_format: PixelFormat::Bgra8Unorm,
            handle: NativeHandleDescriptor::IoSurface { id: surface.id() },
        };
        descriptor.validate()?;
        Ok(MacEditorFrame {
            descriptor,
            surface,
        })
    }
}

fn shareable_content() -> Result<Retained<SCShareableContent>, CaptureError> {
    let (sender, receiver) = mpsc::sync_channel(1);
    let handler = RcBlock::new(
        move |content: *mut SCShareableContent, error: *mut NSError| {
            let result = if !error.is_null() {
                Err(unsafe { error_message(error) })
            } else {
                // SAFETY: ScreenCaptureKit lends a live Objective-C object for
                // the duration of this callback. Retain it before transferring
                // its +1 pointer through the Send-only channel payload.
                unsafe { Retained::retain(content) }
                    .map(Retained::into_raw)
                    .map(|pointer| pointer as usize)
                    .ok_or_else(|| "ScreenCaptureKit returned no shareable content".to_owned())
            };
            let _ = sender.send(result);
        },
    );
    unsafe { SCShareableContent::getShareableContentWithCompletionHandler(&handler) };
    let pointer = receiver
        .recv()
        .map_err(|error| CaptureError::platform(BACKEND_NAME, error))?
        .map_err(|error| CaptureError::platform(BACKEND_NAME, error))?;
    // SAFETY: The callback transferred exactly one retain through this raw
    // pointer and no other owner adopts it.
    unsafe { Retained::from_raw(pointer as *mut SCShareableContent) }
        .ok_or_else(|| CaptureError::platform(BACKEND_NAME, "shareable content vanished"))
}

fn capture_surface(
    filter: &SCContentFilter,
    configuration: &SCStreamConfiguration,
) -> Result<IOSurface, CaptureError> {
    let (sender, receiver) = mpsc::sync_channel(1);
    let handler = RcBlock::new(move |sample: *mut CMSampleBuffer, error: *mut NSError| {
        let result = if !error.is_null() {
            Err(unsafe { error_message(error) })
        } else if sample.is_null() {
            Err("ScreenCaptureKit returned no sample buffer".to_owned())
        } else {
            // SAFETY: The sample and its pixel buffer are live for this
            // callback. IOSurface::from_raw_borrowed performs the retain that
            // makes the returned frame independent of callback lifetime.
            let pixel_buffer = unsafe { CMSampleBufferGetImageBuffer(sample.cast()) };
            let surface = if pixel_buffer.is_null() {
                std::ptr::null_mut()
            } else {
                unsafe { CVPixelBufferGetIOSurface(pixel_buffer) }
            };
            unsafe { IOSurface::from_raw_borrowed(surface) }
                .ok_or_else(|| "sample buffer is not IOSurface-backed".to_owned())
        };
        let _ = sender.send(result);
    });
    unsafe {
        SCScreenshotManager::captureSampleBufferWithFilter_configuration_completionHandler(
            filter,
            configuration,
            Some(&handler),
        )
    };
    receiver
        .recv()
        .map_err(|error| CaptureError::platform(BACKEND_NAME, error))?
        .map_err(|error| CaptureError::platform(BACKEND_NAME, error))
}

fn select_window(
    windows: Retained<objc2_foundation::NSArray<SCWindow>>,
    selector: &EditorSelector,
) -> Result<Retained<SCWindow>, CaptureError> {
    let mut matches = windows.into_iter().filter(|window| unsafe {
        let pid_matches = window.owningApplication().is_some_and(
            |application: Retained<SCRunningApplication>| {
                application.processID() == selector.process_id as i32
            },
        );
        let title_matches = window
            .title()
            .is_some_and(|title| selector.title_matches(&title.to_string()));
        pid_matches && title_matches
    });
    let first = matches.next().ok_or_else(|| CaptureError::WindowNotFound {
        process_id: selector.process_id,
        title: selector.title_contains.clone(),
    })?;
    if matches.next().is_some() {
        return Err(CaptureError::AmbiguousWindow {
            process_id: selector.process_id,
            title: selector.title_contains.clone(),
        });
    }
    Ok(first)
}

unsafe fn error_message(error: *mut NSError) -> String {
    // SAFETY: Callers check for null and the callback keeps NSError alive.
    unsafe { &*error }.localizedDescription().to_string()
}

fn dimension(value: f64, label: &str) -> Result<u32, CaptureError> {
    if !value.is_finite() || value <= 0.0 || value > f64::from(u32::MAX) {
        return Err(CaptureError::InvalidFrame(format!(
            "invalid editor {label} {value}"
        )));
    }
    Ok(value.ceil() as u32)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dimensions_round_up_fractional_backing_sizes() {
        assert_eq!(dimension(639.25, "width").unwrap(), 640);
        assert!(dimension(0.0, "width").is_err());
        assert!(dimension(f64::NAN, "width").is_err());
    }
}
