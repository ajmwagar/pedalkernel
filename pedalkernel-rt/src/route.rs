//! Minimal declarative stage routing primitives.
//!
//! Stages expose local ports through binding ids. Containers can match those
//! ids to build route data without knowing how any stage processes audio.

extern crate alloc;

use alloc::vec::Vec;

/// Declarative identifier used to join a stage output to a stage input.
///
/// The compiler decides what this means. Current adapters use original graph
/// node ids because they are already the shared currency at stage boundaries.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct BindingId(pub usize);

impl BindingId {
    pub const fn new(raw: usize) -> Self {
        Self(raw)
    }

    pub const fn get(self) -> usize {
        self.0
    }
}

/// One local stage port exposed under a route binding id.
///
/// `local_port` is intentionally just an index into the stage's own port
/// storage. The stage owns what that index means; route building only matches
/// `binding_id`s.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct PortBinding {
    pub binding_id: BindingId,
    pub local_port: usize,
}

impl PortBinding {
    pub const fn new(binding_id: BindingId, local_port: usize) -> Self {
        Self {
            binding_id,
            local_port,
        }
    }
}

/// Declarative route between two exposed stage bindings.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Route {
    pub from: BindingId,
    pub to: BindingId,
}

impl Route {
    pub const fn new(from: BindingId, to: BindingId) -> Self {
        Self { from, to }
    }
}

/// Build declarative routes for every output/input binding id match.
pub fn connect_matching(outs: &[PortBinding], ins: &[PortBinding]) -> Vec<Route> {
    let mut routes = Vec::new();
    for out in outs {
        for input in ins {
            if out.binding_id == input.binding_id {
                routes.push(Route::new(out.binding_id, input.binding_id));
            }
        }
    }
    routes
}

#[cfg(test)]
mod tests {
    use alloc::vec;

    use super::*;

    #[test]
    fn matching_bindings_create_declarative_routes() {
        let outs = [
            PortBinding::new(BindingId::new(10), 0),
            PortBinding::new(BindingId::new(20), 1),
        ];
        let ins = [
            PortBinding::new(BindingId::new(20), 0),
            PortBinding::new(BindingId::new(30), 1),
        ];

        let routes = connect_matching(&outs, &ins);

        assert_eq!(
            routes,
            vec![Route::new(BindingId::new(20), BindingId::new(20))]
        );
    }
}
