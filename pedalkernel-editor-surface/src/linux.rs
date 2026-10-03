use std::{
    os::fd::{AsFd, BorrowedFd, OwnedFd},
    sync::Arc,
};

use x11rb::{
    connection::Connection,
    protocol::{
        composite::{ConnectionExt as CompositeExt, Redirect},
        dri3::ConnectionExt as Dri3Ext,
        xproto::{Atom, AtomEnum, ConnectionExt as XprotoExt, CreateGCAux, Pixmap, Window},
    },
    rust_connection::RustConnection,
};

use crate::{
    capture_timestamp_ns, CaptureBackend, CaptureError, EditorCapture, EditorSelector,
    FrameDescriptor, FrameLease, NativeHandleDescriptor, PixelFormat, EDITOR_SURFACE_VERSION,
};

const BACKEND_NAME: &str = "XComposite/DRI3";
const DRM_FORMAT_XRGB8888: u32 = u32::from_le_bytes(*b"XR24");
const DRM_FORMAT_ARGB8888: u32 = u32::from_le_bytes(*b"AR24");

struct Atoms {
    net_wm_pid: Atom,
    net_wm_name: Atom,
    utf8_string: Atom,
    wm_name: Atom,
}

/// A snapshot pixmap and its exported dma-buf.
///
/// The snapshot is distinct from the live XComposite window pixmap. That is
/// what makes this a stable frame lease instead of a handle to storage that
/// the plugin can repaint while the consumer is importing it.
pub struct LinuxEditorFrame {
    descriptor: FrameDescriptor,
    dma_buf: OwnedFd,
    connection: Arc<RustConnection>,
    snapshot_pixmap: Pixmap,
}

impl LinuxEditorFrame {
    pub fn dma_buf(&self) -> BorrowedFd<'_> {
        self.dma_buf.as_fd()
    }
}

impl FrameLease for LinuxEditorFrame {
    fn descriptor(&self) -> &FrameDescriptor {
        &self.descriptor
    }
}

impl Drop for LinuxEditorFrame {
    fn drop(&mut self) {
        let _ = self.connection.free_pixmap(self.snapshot_pixmap);
        let _ = self.connection.flush();
    }
}

pub struct LinuxEditorCapture {
    selector: EditorSelector,
    connection: Arc<RustConnection>,
    window: Window,
    live_pixmap: Pixmap,
    width: u16,
    height: u16,
    depth: u8,
    sequence: u64,
}

impl LinuxEditorCapture {
    pub fn open(selector: EditorSelector) -> Result<Self, CaptureError> {
        selector.validate()?;
        let (connection, screen_index) =
            x11rb::connect(None).map_err(|error| CaptureError::platform(BACKEND_NAME, error))?;
        let connection = Arc::new(connection);
        let root = connection.setup().roots[screen_index].root;

        let composite = connection
            .composite_query_version(0, 4)
            .map_err(platform)?
            .reply()
            .map_err(platform)?;
        if (composite.major_version, composite.minor_version) < (0, 2) {
            return Err(CaptureError::platform(
                BACKEND_NAME,
                format!(
                    "XComposite 0.2 is required, server reports {}.{}",
                    composite.major_version, composite.minor_version
                ),
            ));
        }
        connection
            .dri3_query_version(1, 0)
            .map_err(platform)?
            .reply()
            .map_err(platform)?;

        let atoms = intern_atoms(&connection)?;
        let window = find_window(&connection, root, &atoms, &selector)?;
        let geometry = connection
            .get_geometry(window)
            .map_err(platform)?
            .reply()
            .map_err(platform)?;
        validate_geometry(geometry.width, geometry.height, geometry.depth)?;
        connection
            .composite_redirect_window(window, Redirect::AUTOMATIC)
            .map_err(platform)?
            .check()
            .map_err(platform)?;
        let live_pixmap = connection.generate_id().map_err(platform)?;
        let name_result = connection
            .composite_name_window_pixmap(window, live_pixmap)
            .map_err(platform)
            .and_then(|cookie| cookie.check().map_err(platform));
        if let Err(error) = name_result {
            let _ = connection.composite_unredirect_window(window, Redirect::AUTOMATIC);
            return Err(error);
        }
        connection.flush().map_err(platform)?;

        Ok(Self {
            selector,
            connection,
            window,
            live_pixmap,
            width: geometry.width,
            height: geometry.height,
            depth: geometry.depth,
            sequence: 0,
        })
    }

    pub const fn window(&self) -> Window {
        self.window
    }

    fn refresh_live_pixmap_after_resize(&mut self) -> Result<(), CaptureError> {
        let geometry = self
            .connection
            .get_geometry(self.window)
            .map_err(platform)?
            .reply()
            .map_err(platform)?;
        validate_geometry(geometry.width, geometry.height, geometry.depth)?;
        if (geometry.width, geometry.height, geometry.depth)
            == (self.width, self.height, self.depth)
        {
            return Ok(());
        }

        let replacement = self.connection.generate_id().map_err(platform)?;
        self.connection
            .composite_name_window_pixmap(self.window, replacement)
            .map_err(platform)?
            .check()
            .map_err(platform)?;
        self.connection
            .free_pixmap(self.live_pixmap)
            .map_err(platform)?
            .check()
            .map_err(platform)?;
        self.live_pixmap = replacement;
        self.width = geometry.width;
        self.height = geometry.height;
        self.depth = geometry.depth;
        Ok(())
    }
}

impl EditorCapture for LinuxEditorCapture {
    type Frame = LinuxEditorFrame;

    fn backend(&self) -> CaptureBackend {
        CaptureBackend::XCompositeDri3
    }

    fn selector(&self) -> &EditorSelector {
        &self.selector
    }

    fn capture(&mut self) -> Result<Self::Frame, CaptureError> {
        self.refresh_live_pixmap_after_resize()?;
        let snapshot_pixmap = self.connection.generate_id().map_err(platform)?;
        self.connection
            .create_pixmap(
                self.depth,
                snapshot_pixmap,
                self.live_pixmap,
                self.width,
                self.height,
            )
            .map_err(platform)?
            .check()
            .map_err(platform)?;

        let result = self.capture_snapshot(snapshot_pixmap);
        if result.is_err() {
            let _ = self.connection.free_pixmap(snapshot_pixmap);
            let _ = self.connection.flush();
        }
        result
    }
}

impl LinuxEditorCapture {
    fn capture_snapshot(
        &mut self,
        snapshot_pixmap: Pixmap,
    ) -> Result<LinuxEditorFrame, CaptureError> {
        let gc = self.connection.generate_id().map_err(platform)?;
        self.connection
            .create_gc(gc, snapshot_pixmap, &CreateGCAux::new())
            .map_err(platform)?
            .check()
            .map_err(platform)?;
        let copy_result = self
            .connection
            .copy_area(
                self.live_pixmap,
                snapshot_pixmap,
                gc,
                0,
                0,
                0,
                0,
                self.width,
                self.height,
            )
            .map_err(platform)?
            .check()
            .map_err(platform);
        let free_result = self
            .connection
            .free_gc(gc)
            .map_err(platform)?
            .check()
            .map_err(platform);
        copy_result?;
        free_result?;

        // The reply is an ordering barrier: the snapshot copy has completed
        // before the server exports its storage.
        let exported = self
            .connection
            .dri3_buffer_from_pixmap(snapshot_pixmap)
            .map_err(platform)?
            .reply()
            .map_err(platform)?;
        if exported.nfd != 1 {
            return Err(CaptureError::InvalidFrame(format!(
                "DRI3 exported {} file descriptors; packed BGRA requires exactly one",
                exported.nfd
            )));
        }
        if exported.width != self.width || exported.height != self.height {
            return Err(CaptureError::InvalidFrame(format!(
                "DRI3 dimensions {}x{} do not match snapshot {}x{}",
                exported.width, exported.height, self.width, self.height
            )));
        }
        let (pixel_format, drm_fourcc) = drm_format(exported.depth, exported.bpp)?;
        self.sequence = self.sequence.wrapping_add(1);
        let descriptor = FrameDescriptor {
            protocol: EDITOR_SURFACE_VERSION,
            sequence: self.sequence,
            captured_at_unix_ns: capture_timestamp_ns(),
            width: u32::from(exported.width),
            height: u32::from(exported.height),
            stride: u32::from(exported.stride),
            pixel_format,
            handle: NativeHandleDescriptor::DmaBuf {
                drm_fourcc,
                // BufferFromPixmap is DRI3 v1 and carries no modifier. An
                // importer must negotiate/derive this instead of assuming
                // linear memory.
                modifier: None,
                offset: 0,
            },
        };
        descriptor.validate()?;
        Ok(LinuxEditorFrame {
            descriptor,
            dma_buf: exported.pixmap_fd,
            connection: Arc::clone(&self.connection),
            snapshot_pixmap,
        })
    }
}

impl Drop for LinuxEditorCapture {
    fn drop(&mut self) {
        let _ = self.connection.free_pixmap(self.live_pixmap);
        let _ = self
            .connection
            .composite_unredirect_window(self.window, Redirect::AUTOMATIC);
        let _ = self.connection.flush();
    }
}

fn intern_atoms(connection: &RustConnection) -> Result<Atoms, CaptureError> {
    Ok(Atoms {
        net_wm_pid: intern(connection, b"_NET_WM_PID")?,
        net_wm_name: intern(connection, b"_NET_WM_NAME")?,
        utf8_string: intern(connection, b"UTF8_STRING")?,
        wm_name: intern(connection, b"WM_NAME")?,
    })
}

fn intern(connection: &RustConnection, name: &[u8]) -> Result<Atom, CaptureError> {
    Ok(connection
        .intern_atom(false, name)
        .map_err(platform)?
        .reply()
        .map_err(platform)?
        .atom)
}

fn find_window(
    connection: &RustConnection,
    root: Window,
    atoms: &Atoms,
    selector: &EditorSelector,
) -> Result<Window, CaptureError> {
    let mut pending = vec![root];
    let mut matches = Vec::new();
    while let Some(window) = pending.pop() {
        let tree = connection
            .query_tree(window)
            .map_err(platform)?
            .reply()
            .map_err(platform)?;
        pending.extend(tree.children);
        if window_pid(connection, window, atoms)? == Some(selector.process_id)
            && window_title(connection, window, atoms)?
                .is_some_and(|title| selector.title_matches(&title))
        {
            matches.push(window);
        }
    }
    match matches.as_slice() {
        [window] => Ok(*window),
        [] => Err(CaptureError::WindowNotFound {
            process_id: selector.process_id,
            title: selector.title_contains.clone(),
        }),
        _ => Err(CaptureError::AmbiguousWindow {
            process_id: selector.process_id,
            title: selector.title_contains.clone(),
        }),
    }
}

fn window_pid(
    connection: &RustConnection,
    window: Window,
    atoms: &Atoms,
) -> Result<Option<u32>, CaptureError> {
    let property = connection
        .get_property(false, window, atoms.net_wm_pid, AtomEnum::CARDINAL, 0, 1)
        .map_err(platform)?
        .reply()
        .map_err(platform)?;
    Ok(property.value32().and_then(|mut values| values.next()))
}

fn window_title(
    connection: &RustConnection,
    window: Window,
    atoms: &Atoms,
) -> Result<Option<String>, CaptureError> {
    for (property, kind) in [
        (atoms.net_wm_name, atoms.utf8_string),
        (atoms.wm_name, AtomEnum::STRING.into()),
    ] {
        let value = connection
            .get_property(false, window, property, kind, 0, 16_384)
            .map_err(platform)?
            .reply()
            .map_err(platform)?
            .value;
        if !value.is_empty() {
            return Ok(Some(String::from_utf8_lossy(&value).into_owned()));
        }
    }
    Ok(None)
}

fn validate_geometry(width: u16, height: u16, depth: u8) -> Result<(), CaptureError> {
    if width == 0 || height == 0 {
        return Err(CaptureError::InvalidFrame(
            "X11 editor has zero-sized geometry".into(),
        ));
    }
    if depth != 24 && depth != 32 {
        return Err(CaptureError::InvalidFrame(format!(
            "unsupported X11 editor depth {depth}; expected 24 or 32"
        )));
    }
    Ok(())
}

fn drm_format(depth: u8, bpp: u8) -> Result<(PixelFormat, u32), CaptureError> {
    match (depth, bpp) {
        (24, 32) => Ok((PixelFormat::Bgrx8Unorm, DRM_FORMAT_XRGB8888)),
        (32, 32) => Ok((PixelFormat::Bgra8Unorm, DRM_FORMAT_ARGB8888)),
        _ => Err(CaptureError::InvalidFrame(format!(
            "unsupported DRI3 depth/bpp {depth}/{bpp}"
        ))),
    }
}

fn platform(error: impl std::fmt::Display) -> CaptureError {
    CaptureError::platform(BACKEND_NAME, error)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn maps_common_x11_visuals_to_drm_fourcc() {
        assert_eq!(
            drm_format(24, 32).unwrap(),
            (PixelFormat::Bgrx8Unorm, DRM_FORMAT_XRGB8888)
        );
        assert_eq!(
            drm_format(32, 32).unwrap(),
            (PixelFormat::Bgra8Unorm, DRM_FORMAT_ARGB8888)
        );
        assert!(drm_format(16, 16).is_err());
    }
}
