//! Individual component structs implementing the `Component` trait.
//!
//! Each struct holds the data for one circuit component type and implements
//! the full `Component` trait.

mod active_ics;
mod delay;
mod diodes;
mod modulation;
mod passives;
mod switches;
mod transformer;
mod transistors;
mod tubes;

pub use active_ics::*;
pub use delay::*;
pub use diodes::*;
pub use modulation::*;
pub use passives::*;
pub use switches::*;
pub use transformer::*;
pub use transistors::*;
pub use tubes::*;

/// Macro to implement the three trait-object support methods (clone_box, as_any, dyn_eq)
/// that every `Component` implementor needs. Requires the struct to derive Clone + PartialEq.
macro_rules! impl_component_dyn {
    () => {
        fn clone_box(&self) -> Box<dyn crate::compiler::component::Component> {
            Box::new(self.clone())
        }
        fn as_any(&self) -> &dyn std::any::Any {
            self
        }
        fn as_any_mut(&mut self) -> &mut dyn std::any::Any {
            self
        }
        fn dyn_eq(&self, other: &dyn crate::compiler::component::Component) -> bool {
            other
                .as_any()
                .downcast_ref::<Self>()
                .map_or(false, |o| self == o)
        }
    };
}
pub(crate) use impl_component_dyn;
