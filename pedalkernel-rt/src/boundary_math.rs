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
}
