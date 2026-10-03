use std::{
    ffi::c_void,
    sync::mpsc::{self, Receiver, SyncSender},
    time::Duration,
};

use apple_cf::iosurface::IOSurface;
use block2::RcBlock;
use dispatch2::{DispatchQueue, DispatchRetained};
use objc2::{
    define_class, msg_send, rc::Retained, runtime::ProtocolObject, AnyThread, DefinedClass,
};
use objc2_core_media::CMSampleBuffer;
use objc2_foundation::{NSError, NSObject, NSObjectProtocol};
use objc2_screen_capture_kit::{
    SCContentFilter, SCRunningApplication, SCShareableContent, SCStream, SCStreamConfiguration,
    SCStreamOutput, SCStreamOutputType, SCWindow,
};

use crate::{
    capture_timestamp_ns, validate_capture_fps, CaptureBackend, CaptureError, EditorCapture,
    EditorSelector, FrameDescriptor, FrameLease, NativeHandleDescriptor, PixelFormat,
    DEFAULT_CAPTURE_FPS, EDITOR_SURFACE_VERSION,
};

const BACKEND_NAME: &str = "ScreenCaptureKit";
const BGRA_FOURCC: u32 = u32::from_be_bytes(*b"BGRA");

unsafe extern "C" {
    fn CMSampleBufferGetImageBuffer(sample_buffer: *mut c_void) -> *mut c_void;
    fn CVPixelBufferGetIOSurface(pixel_buffer: *mut c_void) -> *mut c_void;
}

struct FrameOutputIvars {
    frames: SyncSender<Result<IOSurface, String>>,
}

define_class!(
    // SAFETY: NSObject has no subclassing requirements. FrameOutput does not
    // implement Drop, and its immutable ivars are safe to use on the serial
    // ScreenCaptureKit callback queue.
    #[unsafe(super = NSObject)]
    #[name = "FPLPedalKernelFrameOutput"]
    #[ivars = FrameOutputIvars]
    struct FrameOutput;

    // SAFETY: NSObjectProtocol has no additional safety requirements.
    unsafe impl NSObjectProtocol for FrameOutput {}

    // SAFETY: The callback selector and arguments match SCStreamOutput's
    // Objective-C protocol declaration.
    unsafe impl SCStreamOutput for FrameOutput {
        #[unsafe(method(stream:didOutputSampleBuffer:ofType:))]
        #[allow(non_snake_case)]
        unsafe fn stream_didOutputSampleBuffer_ofType(
            &self,
            _stream: &SCStream,
            sample_buffer: &CMSampleBuffer,
            output_type: SCStreamOutputType,
        ) {
            if output_type != SCStreamOutputType::Screen {
                return;
            }
            let result = unsafe { surface_from_sample(sample_buffer as *const _ as *mut c_void) };
            // This is a latest-frame channel. Backpressure must never block the
            // ScreenCaptureKit callback or grow unbounded.
            let _ = self.ivars().frames.try_send(result);
        }
    }
);

impl FrameOutput {
    fn new(frames: SyncSender<Result<IOSurface, String>>) -> Retained<Self> {
        let this = Self::alloc().set_ivars(FrameOutputIvars { frames });
        unsafe { msg_send![super(this), init] }
    }
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
    stream: Retained<SCStream>,
    _output: Retained<FrameOutput>,
    _queue: DispatchRetained<DispatchQueue>,
    frames: Receiver<Result<IOSurface, String>>,
    sequence: u64,
}

impl MacEditorCapture {
    pub fn open(selector: EditorSelector) -> Result<Self, CaptureError> {
        Self::open_with_fps(selector, DEFAULT_CAPTURE_FPS)
    }

    pub fn open_with_fps(selector: EditorSelector, max_fps: u16) -> Result<Self, CaptureError> {
        selector.validate()?;
        validate_capture_fps(max_fps)?;
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
            configuration
                .setMinimumFrameInterval(objc2_core_media::CMTime::new(1, i32::from(max_fps)));
        }

        let (sender, frames) = mpsc::sync_channel(2);
        let output = FrameOutput::new(sender);
        let queue = DispatchQueue::new("dev.fpl.pedalkernel.editor-surface", None);
        let stream = unsafe {
            SCStream::initWithFilter_configuration_delegate(
                SCStream::alloc(),
                &filter,
                &configuration,
                None,
            )
        };
        let protocol_output = ProtocolObject::from_ref(&*output);
        unsafe {
            stream.addStreamOutput_type_sampleHandlerQueue_error(
                protocol_output,
                SCStreamOutputType::Screen,
                Some(&queue),
            )
        }
        .map_err(|error| CaptureError::platform(BACKEND_NAME, error.localizedDescription()))?;
        start_stream(&stream)?;

        Ok(Self {
            selector,
            window_id,
            stream,
            _output: output,
            _queue: queue,
            frames,
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
        let mut result = self
            .frames
            .recv_timeout(Duration::from_secs(1))
            .map_err(|error| CaptureError::platform(BACKEND_NAME, error))?;
        while let Ok(newer) = self.frames.try_recv() {
            result = newer;
        }
        let surface = result.map_err(|error| CaptureError::platform(BACKEND_NAME, error))?;
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

impl Drop for MacEditorCapture {
    fn drop(&mut self) {
        unsafe { self.stream.stopCaptureWithCompletionHandler(None) };
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

fn start_stream(stream: &SCStream) -> Result<(), CaptureError> {
    let (sender, receiver) = mpsc::sync_channel(1);
    let handler = RcBlock::new(move |error: *mut NSError| {
        let result = if error.is_null() {
            Ok(())
        } else {
            Err(unsafe { error_message(error) })
        };
        let _ = sender.send(result);
    });
    unsafe { stream.startCaptureWithCompletionHandler(Some(&handler)) };
    receiver
        .recv()
        .map_err(|error| CaptureError::platform(BACKEND_NAME, error))?
        .map_err(|error| CaptureError::platform(BACKEND_NAME, error))
}

unsafe fn surface_from_sample(sample: *mut c_void) -> Result<IOSurface, String> {
    let pixel_buffer = unsafe { CMSampleBufferGetImageBuffer(sample) };
    let surface = if pixel_buffer.is_null() {
        std::ptr::null_mut()
    } else {
        unsafe { CVPixelBufferGetIOSurface(pixel_buffer) }
    };
    // SAFETY: ScreenCaptureKit keeps the sample and pixel buffer live for the
    // callback. This performs the retain that makes the frame independent of
    // callback lifetime.
    unsafe { IOSurface::from_raw_borrowed(surface) }
        .ok_or_else(|| "sample buffer is not IOSurface-backed".to_owned())
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
