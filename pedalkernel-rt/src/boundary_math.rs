//! Typed WDF boundary-drive math.
//!
//! Keep wave/voltage conversion and port orientation rules here so runtime
//! routing code does not hand-roll signs or `2V - a` source math.

use crate::Wave;
use alloc::vec::Vec;

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct PortVoltage(pub Wave);

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct IncidentWave(pub Wave);

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ReflectedWave(pub Wave);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum PortOrientation {
    NonInverting,
    Inverting,
}

impl PortOrientation {
    pub const fn sign(self) -> Wave {
        match self {
            Self::NonInverting => 1.0,
            Self::Inverting => -1.0,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct PortTerminals<N> {
    pub pos: Option<N>,
    pub neg: Option<N>,
}

pub type WdfPortTerminals = PortTerminals<usize>;

impl<N> PortTerminals<N> {
    pub const fn grounded() -> Self {
        Self {
            pos: None,
            neg: None,
        }
    }

    pub const fn single_ended(pos: N) -> Self {
        Self {
            pos: Some(pos),
            neg: None,
        }
    }

    pub const fn maybe_single_ended(pos: Option<N>) -> Self {
        Self { pos, neg: None }
    }

    pub const fn differential(pos: N, neg: N) -> Self {
        Self {
            pos: Some(pos),
            neg: Some(neg),
        }
    }

    pub const fn maybe_differential(pos: Option<N>, neg: Option<N>) -> Self {
        Self { pos, neg }
    }

    pub fn reversed(self) -> Self {
        Self {
            pos: self.neg,
            neg: self.pos,
        }
    }

    pub fn as_tuple(self) -> (Option<N>, Option<N>) {
        (self.pos, self.neg)
    }

    pub fn voltage_with<F>(self, mut node_voltage: F) -> PortVoltage
    where
        F: FnMut(N) -> Wave,
    {
        let pos = self.pos.map(&mut node_voltage).unwrap_or(0.0);
        let neg = self.neg.map(node_voltage).unwrap_or(0.0);
        PortVoltage(pos - neg)
    }
}

impl PortTerminals<usize> {
    pub const fn to_wdf_port(self, resistance: Wave) -> crate::tree::WdfPort {
        crate::tree::WdfPort {
            node_pos: self.pos,
            node_neg: self.neg,
            resistance,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct PortSpec<N> {
    pub terminals: PortTerminals<N>,
    pub resistance: Wave,
}

impl<N> PortSpec<N> {
    pub const fn new(terminals: PortTerminals<N>, resistance: Wave) -> Self {
        Self {
            terminals,
            resistance,
        }
    }
}

impl PortSpec<usize> {
    pub const fn to_wdf_port(self) -> crate::tree::WdfPort {
        self.terminals.to_wdf_port(self.resistance)
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct MappedPort<G, M> {
    pub graph: PortTerminals<G>,
    pub mna: PortSpec<M>,
}

impl<G, M> MappedPort<G, M> {
    pub const fn new(graph: PortTerminals<G>, mna: PortSpec<M>) -> Self {
        Self { graph, mna }
    }
}

impl MappedPort<usize, usize> {
    pub const fn to_wdf_port(self) -> crate::tree::WdfPort {
        self.mna.to_wdf_port()
    }

    pub const fn resistance(self) -> Wave {
        self.mna.resistance
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct CapStamp<N> {
    pub terminals: PortTerminals<N>,
    pub capacitance: Wave,
}

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct PotStamp<N> {
    pub child_idx: usize,
    pub terminals: PortTerminals<N>,
    pub conductance: Wave,
}

impl PortVoltage {
    pub const fn new(value: Wave) -> Self {
        Self(value)
    }

    /// Reflected wave for an ideal voltage source at a port.
    ///
    /// WDF voltage sources use `b = 2V - a`. This is intentionally separate
    /// from BKM boundary incident offsets, which are direct wave offsets.
    pub fn ideal_voltage_source_reflection(self, incident: IncidentWave) -> ReflectedWave {
        ReflectedWave(2.0 * self.0 - incident.0)
    }

    pub fn as_boundary_incident_offset(self, orientation: PortOrientation) -> IncidentWave {
        IncidentWave(self.0 * orientation.sign())
    }
}

impl IncidentWave {
    pub const fn new(value: Wave) -> Self {
        Self(value)
    }

    pub fn oriented(self, orientation: PortOrientation) -> Self {
        Self(self.0 * orientation.sign())
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct BoundaryIncidentDrive {
    pub port_idx: usize,
    pub incident_offset: IncidentWave,
}

impl BoundaryIncidentDrive {
    pub const fn new(port_idx: usize, incident_offset: IncidentWave) -> Self {
        Self {
            port_idx,
            incident_offset,
        }
    }

    pub fn from_port_voltage(
        port_idx: usize,
        voltage: PortVoltage,
        orientation: PortOrientation,
    ) -> Self {
        Self::new(port_idx, voltage.as_boundary_incident_offset(orientation))
    }
}

pub fn sum_incident_offsets(port_idx: usize, drives: &[BoundaryIncidentDrive]) -> Wave {
    drives
        .iter()
        .filter_map(|drive| {
            (drive.port_idx == port_idx && drive.incident_offset.0.is_finite())
                .then_some(drive.incident_offset.0)
        })
        .sum()
}

pub fn push_differential_voltage_drives(
    drives: &mut Vec<BoundaryIncidentDrive>,
    positive_ports: &[usize],
    negative_ports: &[usize],
    voltage: PortVoltage,
) {
    for &port_idx in positive_ports {
        drives.push(BoundaryIncidentDrive::from_port_voltage(
            port_idx,
            voltage,
            PortOrientation::NonInverting,
        ));
    }
    for &port_idx in negative_ports {
        drives.push(BoundaryIncidentDrive::from_port_voltage(
            port_idx,
            voltage,
            PortOrientation::Inverting,
        ));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ideal_voltage_source_reflection_uses_two_v_minus_a() {
        let reflected =
            PortVoltage::new(0.75).ideal_voltage_source_reflection(IncidentWave::new(0.2));
        assert_eq!(reflected, ReflectedWave(1.3));
    }

    #[test]
    fn boundary_voltage_offsets_are_oriented_without_source_reflection_factor() {
        assert_eq!(
            PortVoltage::new(0.25).as_boundary_incident_offset(PortOrientation::NonInverting),
            IncidentWave(0.25)
        );
        assert_eq!(
            PortVoltage::new(0.25).as_boundary_incident_offset(PortOrientation::Inverting),
            IncidentWave(-0.25)
        );
    }

    #[test]
    fn differential_drive_expands_to_positive_and_negative_port_offsets() {
        let mut drives = Vec::new();
        push_differential_voltage_drives(&mut drives, &[1, 2], &[3], PortVoltage::new(0.1));
        assert_eq!(sum_incident_offsets(1, &drives), 0.1);
        assert_eq!(sum_incident_offsets(2, &drives), 0.1);
        assert_eq!(sum_incident_offsets(3, &drives), -0.1);
        assert_eq!(sum_incident_offsets(4, &drives), 0.0);
    }

    #[test]
    fn repeated_boundary_offsets_sum_per_port() {
        let drives = [
            BoundaryIncidentDrive::new(7, IncidentWave::new(0.2)),
            BoundaryIncidentDrive::new(7, IncidentWave::new(-0.05)),
            BoundaryIncidentDrive::new(8, IncidentWave::new(1.0)),
        ];
        assert_eq!(sum_incident_offsets(7, &drives), 0.15000000000000002);
    }

    #[test]
    fn wdf_port_terminals_make_single_ended_and_differential_orientation_explicit() {
        assert_eq!(
            WdfPortTerminals::single_ended(3).as_tuple(),
            (Some(3), None)
        );
        assert_eq!(
            WdfPortTerminals::differential(3, 4).as_tuple(),
            (Some(3), Some(4))
        );
        assert_eq!(
            WdfPortTerminals::differential(3, 4).reversed().as_tuple(),
            (Some(4), Some(3))
        );
        assert_eq!(WdfPortTerminals::grounded().as_tuple(), (None, None));
    }

    #[test]
    fn wdf_port_terminals_extract_oriented_voltage() {
        let node_voltage = |node: usize| match node {
            1 => 2.5,
            2 => 0.75,
            _ => 0.0,
        };
        assert_eq!(
            WdfPortTerminals::single_ended(1).voltage_with(node_voltage),
            PortVoltage(2.5)
        );
        assert_eq!(
            WdfPortTerminals::differential(1, 2).voltage_with(node_voltage),
            PortVoltage(1.75)
        );
        assert_eq!(
            WdfPortTerminals::differential(1, 2)
                .reversed()
                .voltage_with(node_voltage),
            PortVoltage(-1.75)
        );
    }

    #[test]
    fn wdf_port_terminals_construct_runtime_wdf_port() {
        let port = WdfPortTerminals::differential(5, 6).to_wdf_port(123.0);
        assert_eq!(port.node_pos, Some(5));
        assert_eq!(port.node_neg, Some(6));
        assert_eq!(port.resistance, 123.0);
    }

    #[test]
    fn port_spec_carries_resistance_with_oriented_terminals() {
        let spec = PortSpec::new(WdfPortTerminals::differential(2, 9), 470.0);
        let port = spec.to_wdf_port();

        assert_eq!(port.node_pos, Some(2));
        assert_eq!(port.node_neg, Some(9));
        assert_eq!(port.resistance, 470.0);
    }

    #[test]
    fn mapped_port_keeps_graph_and_mna_terminal_spaces_together() {
        let binding = MappedPort::new(
            PortTerminals::differential("left", "right"),
            PortSpec::new(WdfPortTerminals::differential(4, 5), 1000.0),
        );

        assert_eq!(binding.graph.as_tuple(), (Some("left"), Some("right")));
        assert_eq!(binding.mna.terminals.as_tuple(), (Some(4), Some(5)));
        assert_eq!(binding.mna.resistance, 1000.0);
    }

    #[test]
    fn pot_stamp_names_child_terminals_and_current_conductance() {
        let stamp = PotStamp {
            child_idx: 3,
            terminals: WdfPortTerminals::single_ended(7),
            conductance: 0.001,
        };

        assert_eq!(stamp.child_idx, 3);
        assert_eq!(stamp.terminals.as_tuple(), (Some(7), None));
        assert_eq!(stamp.conductance, 0.001);
    }
}
