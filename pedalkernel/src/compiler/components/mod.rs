//! Individual component structs implementing the `Component` trait.
//!
//! Each struct holds the data for one circuit component type and implements
//! the full `Component` trait. These structs will eventually replace the
//! `ComponentKind` enum in `dsl.rs`.

mod passives;
mod diodes;
mod transistors;
mod tubes;
mod active_ics;
mod modulation;
mod delay;
mod transformer;
mod switches;

pub use passives::*;
pub use diodes::*;
pub use transistors::*;
pub use tubes::*;
pub use active_ics::*;
pub use modulation::*;
pub use delay::*;
pub use transformer::*;
pub use switches::*;

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
        fn dyn_eq(&self, other: &dyn crate::compiler::component::Component) -> bool {
            other
                .as_any()
                .downcast_ref::<Self>()
                .map_or(false, |o| self == o)
        }
    };
}
pub(crate) use impl_component_dyn;

// ═══════════════════════════════════════════════════════════════════════════
// Bridge: ComponentKind → Box<dyn Component>
// ═══════════════════════════════════════════════════════════════════════════

use super::component::Component;
use crate::dsl::ComponentKind;

impl From<ComponentKind> for Box<dyn Component> {
    fn from(kind: ComponentKind) -> Self {
        match kind {
            ComponentKind::Resistor(v) => Box::new(Resistor { value: v }),
            ComponentKind::Capacitor(cfg) => Box::new(Capacitor { config: cfg }),
            ComponentKind::Inductor(v) => Box::new(Inductor { value: v }),
            ComponentKind::Potentiometer(r, t) => Box::new(Potentiometer { max_r: r, taper: t }),
            ComponentKind::Tempco(r, p) => Box::new(Tempco { resistance: r, ppm: p }),
            ComponentKind::CapSwitched(v) => Box::new(CapSwitched { values: v }),
            ComponentKind::InductorSwitched(v) => Box::new(InductorSwitched { values: v }),
            ComponentKind::ResistorSwitched(v) => Box::new(ResistorSwitched { values: v }),
            ComponentKind::Diode(dt) => Box::new(Diode { diode_type: dt }),
            ComponentKind::DiodePair(dt) => Box::new(DiodePair { diode_type: dt }),
            ComponentKind::Zener(v) => Box::new(Zener { breakdown_voltage: v }),
            ComponentKind::Neon(nt) => Box::new(Neon { neon_type: nt }),
            ComponentKind::Npn(m) => Box::new(Npn { model: m }),
            ComponentKind::Pnp(m) => Box::new(Pnp { model: m }),
            ComponentKind::NJfet(m) => Box::new(NJfet { model: m }),
            ComponentKind::PJfet(m) => Box::new(PJfet { model: m }),
            ComponentKind::Nmos(mt) => Box::new(Nmos { mosfet_type: mt }),
            ComponentKind::Pmos(mt) => Box::new(Pmos { mosfet_type: mt }),
            ComponentKind::Triode(m) => Box::new(Triode { model: m }),
            ComponentKind::Pentode(m) => Box::new(Pentode { model: m }),
            ComponentKind::VariMu(m) => Box::new(VariMu { model: m }),
            ComponentKind::OpAmp(ot) => Box::new(OpAmp { op_type: ot }),
            ComponentKind::Vco(vt) => Box::new(Vco { vco_type: vt }),
            ComponentKind::Vcf(vt) => Box::new(Vcf { vcf_type: vt }),
            ComponentKind::Vca(vt) => Box::new(Vca { vca_type: vt }),
            ComponentKind::Comparator(ct) => Box::new(Comparator { comp_type: ct }),
            ComponentKind::AnalogSwitch(st) => Box::new(AnalogSwitch { switch_type: st }),
            ComponentKind::MatchedNpn(mt) => Box::new(MatchedNpn { matched_type: mt }),
            ComponentKind::MatchedPnp(mt) => Box::new(MatchedPnp { matched_type: mt }),
            ComponentKind::Lfo(w, r, c) => Box::new(Lfo { waveform: w, timing_r: r, timing_c: c }),
            ComponentKind::EnvelopeFollower(ar, ac, rr, rc, sr) => Box::new(EnvelopeFollower {
                attack_r: ar, attack_c: ac, release_r: rr, release_c: rc, sensitivity_r: sr,
            }),
            ComponentKind::Photocoupler(pt) => Box::new(PhotocouplerComp { coupler_type: pt }),
            ComponentKind::Bbd(bt) => Box::new(Bbd { bbd_type: bt }),
            ComponentKind::DelayLine(min, max, interp, med) => Box::new(DelayLineComp {
                min_delay: min, max_delay: max, interpolation: interp, medium: med,
            }),
            ComponentKind::Tap(id, ratio) => Box::new(Tap { parent_id: id, ratio }),
            ComponentKind::Transformer(cfg) => Box::new(TransformerComp { config: cfg }),
            ComponentKind::RotarySwitch(ids) => Box::new(RotarySwitch { linked_ids: ids }),
            ComponentKind::Switch(n) => Box::new(Switch { positions: n }),
            ComponentKind::TriggerInput => Box::new(TriggerInputComp),
        }
    }
}
