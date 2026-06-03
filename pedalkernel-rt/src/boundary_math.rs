//! Typed WDF boundary-drive math.
//!
//! Keep wave/voltage conversion and port orientation rules here so runtime
//! routing code does not hand-roll signs or `2V - a` source math.

use crate::Wave;
use alloc::{string::String, vec::Vec};
use core::marker::PhantomData;

#[derive(Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct DomainIndex<D> {
    raw: usize,
    #[cfg_attr(feature = "serde", serde(skip))]
    _domain: PhantomData<fn() -> D>,
}

impl<D> Copy for DomainIndex<D> {}

impl<D> Clone for DomainIndex<D> {
    fn clone(&self) -> Self {
        *self
    }
}

impl<D> DomainIndex<D> {
    pub const fn new(raw: usize) -> Self {
        Self {
            raw,
            _domain: PhantomData,
        }
    }

    pub const fn get(self) -> usize {
        self.raw
    }

    pub const fn is_unresolved(self) -> bool {
        self.raw == usize::MAX
    }
}

impl<D> From<usize> for DomainIndex<D> {
    fn from(raw: usize) -> Self {
        Self::new(raw)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum GraphNodeDomain {}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum MnaNodeDomain {}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum ScatteringPortDomain {}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum ProcessorPortDomain {}

pub type GraphNodeId = DomainIndex<GraphNodeDomain>;
pub type MnaNodeId = DomainIndex<MnaNodeDomain>;
pub type ScatteringPortId = DomainIndex<ScatteringPortDomain>;
pub type ProcessorPortId = DomainIndex<ProcessorPortDomain>;
pub type GraphPortTerminals = PortTerminals<GraphNodeId>;
pub type MnaPortTerminals = PortTerminals<MnaNodeId>;
pub type ScatteringPortTerminals = PortTerminals<ScatteringPortId>;
pub type CircuitMappedPort = MappedPort<GraphNodeId, MnaNodeId>;
pub type MnaOnePort = OnePort<MnaNodeId>;
pub type MnaVariableResistorBinding = VariableResistorBinding<MnaNodeId>;
pub type NamedScatteringPortBinding = NamedPortBinding<ScatteringPortId>;
pub type FeedbackScatteringPortBinding = FeedbackPortBinding<ScatteringPortId>;
pub type Ohms = Wave;
pub type Farads = Wave;
pub type Henries = Wave;

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

/// A physical WDF one-port component value.
///
/// Runtime state is stored separately: capacitors keep voltage, while inductors
/// typically keep the wave-scaled current `R_port * I_L`.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum OnePortKind {
    Resistor(Ohms),
    Capacitor(Farads),
    Inductor(Henries),
}

impl OnePortKind {
    const DEFAULT_SAMPLE_RATE: Wave = 48_000.0;

    pub const fn is_stateful(self) -> bool {
        matches!(self, Self::Capacitor(_) | Self::Inductor(_))
    }

    pub fn rp(self, sample_rate: Wave) -> Wave {
        match self {
            Self::Resistor(ohms) => ohms,
            Self::Capacitor(farads) => 1.0 / (2.0 * sample_rate * farads),
            Self::Inductor(henries) => 2.0 * sample_rate * henries,
        }
    }

    pub fn sample_rate_for_rp(self, rp: Wave) -> Wave {
        match self {
            Self::Resistor(_) => Self::DEFAULT_SAMPLE_RATE,
            Self::Capacitor(farads) if farads > 0.0 && rp > 0.0 => 1.0 / (2.0 * rp * farads),
            Self::Inductor(henries) if henries > 0.0 => rp / (2.0 * henries),
            Self::Capacitor(_) | Self::Inductor(_) => Self::DEFAULT_SAMPLE_RATE,
        }
    }
}

/// A two-terminal one-port in node domain `N`.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct OnePort<N> {
    pub terminals: PortTerminals<N>,
    pub kind: OnePortKind,
}

impl<N> OnePort<N> {
    pub const fn new(terminals: PortTerminals<N>, kind: OnePortKind) -> Self {
        Self { terminals, kind }
    }

    pub fn map<M, F>(self, f: F) -> OnePort<M>
    where
        F: FnMut(N) -> M,
    {
        OnePort {
            terminals: self.terminals.map(f),
            kind: self.kind,
        }
    }
}

impl OnePort<MnaNodeId> {
    pub fn raw_terminals(self) -> WdfPortTerminals {
        self.terminals.raw()
    }
}

/// A one-port runtime binding: physical spec plus optional shared state slot.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct RuntimeOnePort<N> {
    pub spec: OnePort<N>,
    pub state_slot: Option<StateSlot>,
}

impl<N> RuntimeOnePort<N> {
    pub const fn new(spec: OnePort<N>, state_slot: Option<StateSlot>) -> Self {
        Self { spec, state_slot }
    }

    pub fn rp(&self, sample_rate: Wave) -> Wave {
        self.spec.kind.rp(sample_rate)
    }

    pub fn physical_value(&self) -> Wave {
        match self.spec.kind {
            OnePortKind::Resistor(ohms) => ohms,
            OnePortKind::Capacitor(farads) => farads,
            OnePortKind::Inductor(henries) => henries,
        }
    }

    pub fn type_tag(&self) -> &'static str {
        match self.spec.kind {
            OnePortKind::Resistor(_) => "resistor",
            OnePortKind::Capacitor(_) => "capacitor",
            OnePortKind::Inductor(_) => "inductor",
        }
    }

    pub fn state_slot(&self) -> Option<StateSlot> {
        self.state_slot
    }

    fn required_state_slot(&self) -> StateSlot {
        self.state_slot
            .expect("stateful one-port must have a runtime state slot")
    }

    pub fn wdf_reflected(&self, state: &RuntimeState) -> Wave {
        match self.spec.kind {
            OnePortKind::Resistor(_) => 0.0,
            OnePortKind::Capacitor(_) => state.wave_cache[self.required_state_slot().0].wave_state,
            OnePortKind::Inductor(_) => -state.wave_cache[self.required_state_slot().0].wave_state,
        }
    }

    pub fn wdf_set_incident(&self, incident: Wave, state: &mut RuntimeState) {
        match self.spec.kind {
            OnePortKind::Resistor(_) => {}
            OnePortKind::Capacitor(_) => {
                let slot = self.required_state_slot().0;
                let previous_reflected = state.wave_cache[slot].wave_state;
                state.wave_cache[slot].previous_reflected = previous_reflected;
                state.wave_cache[slot].wave_state = incident;
                state.states[slot] =
                    OnePortState::CapacitorVoltage((incident + previous_reflected) / 2.0);
            }
            OnePortKind::Inductor(_) => {
                let slot = self.required_state_slot().0;
                state.wave_cache[slot].wave_state = incident;
                state.states[slot] = OnePortState::InductorScaledCurrent(incident);
            }
        }
    }

    pub fn wdf_leaf_voltage(&self, state: &RuntimeState) -> Wave {
        match self.spec.kind {
            OnePortKind::Resistor(_) => 0.0,
            OnePortKind::Capacitor(_) => match state.states[self.required_state_slot().0] {
                OnePortState::CapacitorVoltage(voltage) => voltage,
                OnePortState::InductorScaledCurrent(_) => 0.0,
            },
            OnePortKind::Inductor(_) => {
                state.wave_cache[self.required_state_slot().0].wave_state / 2.0
            }
        }
    }

    pub fn wdf_projected_leaf_voltage(&self, incident: Wave, state: &RuntimeState) -> Wave {
        match self.spec.kind {
            OnePortKind::Resistor(_) | OnePortKind::Inductor(_) => incident / 2.0,
            OnePortKind::Capacitor(_) => {
                (incident + state.wave_cache[self.required_state_slot().0].wave_state) / 2.0
            }
        }
    }

    pub fn wdf_one_port_state(&self, state: &RuntimeState) -> Option<OnePortState> {
        self.state_slot.map(|slot| state.states[slot.0])
    }

    pub fn wdf_set_one_port_state(
        &self,
        one_port_state: OnePortState,
        state: &mut RuntimeState,
    ) -> bool {
        match (self.spec.kind, one_port_state) {
            (OnePortKind::Capacitor(_), OnePortState::CapacitorVoltage(voltage)) => {
                let slot = self.required_state_slot().0;
                let previous_reflected = state.wave_cache[slot].previous_reflected;
                state.states[slot] = OnePortState::CapacitorVoltage(voltage);
                state.wave_cache[slot].wave_state = 2.0 * voltage - previous_reflected;
                true
            }
            (OnePortKind::Inductor(_), OnePortState::InductorScaledCurrent(scaled_current)) => {
                let slot = self.required_state_slot().0;
                state.states[slot] = OnePortState::InductorScaledCurrent(scaled_current);
                state.wave_cache[slot].wave_state = scaled_current;
                true
            }
            _ => false,
        }
    }

    pub fn wdf_reset(&self, state: &mut RuntimeState) {
        match self.spec.kind {
            OnePortKind::Resistor(_) => {}
            OnePortKind::Capacitor(_) => {
                let slot = self.required_state_slot().0;
                state.states[slot] = OnePortState::CapacitorVoltage(0.0);
                state.wave_cache[slot] = WdfWaveCache::default();
            }
            OnePortKind::Inductor(_) => {
                let slot = self.required_state_slot().0;
                state.states[slot] = OnePortState::InductorScaledCurrent(0.0);
                state.wave_cache[slot] = WdfWaveCache::default();
            }
        }
    }
}

/// Relationship between physical one-port memory and a transformed LTI state basis.
///
/// IIR and state-space reductions do not generally keep a one-to-one vector
/// of capacitor voltages or inductor scaled currents. They project the same
/// physical one-port set into a solver-specific basis, and may fold or
/// eliminate states when the linear model permits it.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct LtiStateMap<N> {
    /// Physical reactive one-ports that contributed to this LTI form.
    pub physical_one_ports: Vec<RuntimeOnePort<N>>,
    /// Number of canonical physical one-port slots allocated in runtime state.
    pub physical_state_count: usize,
    /// Number of entries in the transformed solver state vector.
    pub transformed_state_count: usize,
    /// Number of physical states folded, transformed away, or eliminated.
    pub folded_or_eliminated_count: usize,
}

impl<N> LtiStateMap<N> {
    pub fn empty(transformed_state_count: usize) -> Self {
        Self {
            physical_one_ports: Vec::new(),
            physical_state_count: 0,
            transformed_state_count,
            folded_or_eliminated_count: 0,
        }
    }

    pub fn new(
        physical_one_ports: Vec<RuntimeOnePort<N>>,
        physical_state_count: usize,
        transformed_state_count: usize,
    ) -> Self {
        let folded_or_eliminated_count =
            physical_state_count.saturating_sub(transformed_state_count);
        Self {
            physical_one_ports,
            physical_state_count,
            transformed_state_count,
            folded_or_eliminated_count,
        }
    }
}

impl<N> Default for LtiStateMap<N> {
    fn default() -> Self {
        Self::empty(0)
    }
}

/// Dense index into one-port runtime state arrays.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct StateSlot(pub usize);

/// Sample-time state for reactive one-ports.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum OnePortState {
    CapacitorVoltage(Wave),
    InductorScaledCurrent(Wave),
}

/// WDF working cache for a physical one-port state slot.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct WdfWaveCache {
    pub wave_state: Wave,
    pub previous_reflected: Wave,
}

impl Default for WdfWaveCache {
    fn default() -> Self {
        Self {
            wave_state: 0.0,
            previous_reflected: 0.0,
        }
    }
}

/// Runtime state for physical one-ports plus dense formulation sidecars.
///
/// `states` is the canonical physical basis. `wave_cache` is a WDF sidecar
/// indexed by the same `StateSlot`; entries for non-WDF regions are allocated
/// but unused. This keeps the hot path as direct vector indexing with the
/// invariant `states.len() == wave_cache.len()`.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct RuntimeState {
    pub states: Vec<OnePortState>,
    pub wave_cache: Vec<WdfWaveCache>,
}

impl RuntimeState {
    pub const fn new() -> Self {
        Self {
            states: Vec::new(),
            wave_cache: Vec::new(),
        }
    }

    pub fn allocate(&mut self, state: OnePortState) -> StateSlot {
        debug_assert_eq!(self.states.len(), self.wave_cache.len());
        let slot = StateSlot(self.states.len());
        self.states.push(state);
        self.wave_cache.push(WdfWaveCache::default());
        slot
    }

    pub fn allocate_one_port(&mut self, kind: OnePortKind) -> Option<StateSlot> {
        match kind {
            OnePortKind::Resistor(_) => None,
            OnePortKind::Capacitor(_) => Some(self.allocate(OnePortState::CapacitorVoltage(0.0))),
            OnePortKind::Inductor(_) => {
                Some(self.allocate(OnePortState::InductorScaledCurrent(0.0)))
            }
        }
    }

    pub fn is_empty(&self) -> bool {
        self.states.is_empty()
    }

    pub fn len(&self) -> usize {
        debug_assert_eq!(self.states.len(), self.wave_cache.len());
        self.states.len()
    }

    pub fn append(&mut self, mut other: Self) {
        debug_assert_eq!(self.states.len(), self.wave_cache.len());
        debug_assert_eq!(other.states.len(), other.wave_cache.len());
        self.states.append(&mut other.states);
        self.wave_cache.append(&mut other.wave_cache);
    }
}

impl Default for RuntimeState {
    fn default() -> Self {
        Self::new()
    }
}

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

    pub fn map<M, F>(self, mut f: F) -> PortTerminals<M>
    where
        F: FnMut(N) -> M,
    {
        PortTerminals {
            pos: self.pos.map(&mut f),
            neg: self.neg.map(f),
        }
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

impl<D> PortTerminals<DomainIndex<D>> {
    pub fn raw(self) -> PortTerminals<usize> {
        self.map(DomainIndex::get)
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

impl PortSpec<MnaNodeId> {
    pub fn to_wdf_port(self) -> crate::tree::WdfPort {
        self.terminals.raw().to_wdf_port(self.resistance)
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

impl CircuitMappedPort {
    pub fn to_wdf_port(self) -> crate::tree::WdfPort {
        self.mna.to_wdf_port()
    }

    pub const fn resistance(self) -> Wave {
        self.mna.resistance
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct VariableResistorBinding<N> {
    pub child_idx: usize,
    pub terminals: PortTerminals<N>,
    pub conductance: Wave,
}

/// Generic semantic role attached to a concrete port index.
///
/// This is the neutral version of repeated structures like BKM block-port
/// bindings: a port is a transport/index detail, while `role` says why a
/// consumer owns or observes that port.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct RolePortBinding<R, P = usize> {
    pub port: P,
    pub role: R,
}

impl<R, P> RolePortBinding<R, P> {
    pub const fn new(port: P, role: R) -> Self {
        Self { port, role }
    }
}

impl<R> RolePortBinding<R, usize> {
    pub fn port_idx(self) -> usize {
        self.port
    }
}

/// Named runtime/source binding to a network port.
#[derive(Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct NamedPortBinding<P = usize> {
    pub name: String,
    pub port: P,
}

impl<P> NamedPortBinding<P> {
    pub fn new(name: String, port: P) -> Self {
        Self { name, port }
    }
}

impl NamedPortBinding<usize> {
    pub fn typed(self) -> NamedPortBinding<ScatteringPortId> {
        NamedPortBinding {
            name: self.name,
            port: ScatteringPortId::new(self.port),
        }
    }
}

/// Internal feedback/source binding from a producer block to a network port.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct FeedbackPortBinding<P = usize> {
    pub source_idx: usize,
    pub port: P,
}

impl<P> FeedbackPortBinding<P> {
    pub const fn new(source_idx: usize, port: P) -> Self {
        Self { source_idx, port }
    }
}

impl FeedbackPortBinding<usize> {
    pub const fn port_idx(self) -> usize {
        self.port
    }

    pub fn typed(self) -> FeedbackPortBinding<ScatteringPortId> {
        FeedbackPortBinding {
            source_idx: self.source_idx,
            port: ScatteringPortId::new(self.port),
        }
    }
}

/// Read-only node extraction from a reflected-wave vector.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ExtractionProbe<N = usize> {
    pub coeffs: Vec<Wave>,
    pub terminals: Option<PortTerminals<N>>,
}

impl<N> ExtractionProbe<N> {
    pub fn new(coeffs: Vec<Wave>, terminals: Option<PortTerminals<N>>) -> Self {
        Self { coeffs, terminals }
    }

    pub fn reads_from(&self, reflected: &[Wave]) -> bool {
        self.coeffs.len() == reflected.len() && self.coeffs.iter().any(|c| c.abs() > 1.0e-15)
    }

    pub fn read_reflected(&self, reflected: &[Wave]) -> Option<Wave> {
        Self::read_coeffs(&self.coeffs, reflected)
    }

    pub fn read_coeffs(coeffs: &[Wave], reflected: &[Wave]) -> Option<Wave> {
        if coeffs.len() != reflected.len() || !coeffs.iter().any(|c| c.abs() > 1.0e-15) {
            return None;
        }
        let value = coeffs
            .iter()
            .zip(reflected.iter())
            .map(|(coeff, b)| coeff * b)
            .sum::<Wave>();
        value.is_finite().then_some(value)
    }
}

/// Shared descriptor for linear multiport networks derived from MNA.
///
/// Consumers still own their hot-path work buffers and solver details. This
/// carries the common stable pieces: port mapping, scattering matrix, source
/// drives, variable resistors, and read-only extraction probes.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct LinearMultiportNetwork<G = GraphNodeId, M = MnaNodeId> {
    pub scattering: Vec<Wave>,
    pub ports: Vec<MappedPort<G, M>>,
    pub source_drives: Vec<NamedPortBinding<ScatteringPortId>>,
    pub feedback_drives: Vec<FeedbackPortBinding<ScatteringPortId>>,
    pub variable_resistors: Vec<VariableResistorBinding<M>>,
    pub extraction: Option<ExtractionProbe<M>>,
}

impl<G, M> LinearMultiportNetwork<G, M> {
    pub fn new(scattering: Vec<Wave>, ports: Vec<MappedPort<G, M>>) -> Self {
        Self {
            scattering,
            ports,
            source_drives: Vec::new(),
            feedback_drives: Vec::new(),
            variable_resistors: Vec::new(),
            extraction: None,
        }
    }

    pub fn n_ports(&self) -> usize {
        self.ports.len()
    }

    pub fn has_square_scattering(&self) -> bool {
        let n = self.n_ports();
        self.scattering.len() == n * n
    }

    pub fn scatter_into(&self, reflected: &[Wave], incident: &mut [Wave]) -> bool {
        let n = self.n_ports();
        if !self.has_square_scattering() || reflected.len() != n || incident.len() != n {
            return false;
        }
        for i in 0..n {
            let mut sum = 0.0;
            for j in 0..n {
                sum += self.scattering[i * n + j] * reflected[j];
            }
            incident[i] = sum;
        }
        true
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
    pub port: ScatteringPortId,
    pub incident_offset: IncidentWave,
}

impl BoundaryIncidentDrive {
    pub const fn new(port_idx: usize, incident_offset: IncidentWave) -> Self {
        Self {
            port: ScatteringPortId::new(port_idx),
            incident_offset,
        }
    }

    pub const fn for_port(port: ScatteringPortId, incident_offset: IncidentWave) -> Self {
        Self {
            port,
            incident_offset,
        }
    }

    pub const fn port_idx(self) -> usize {
        self.port.get()
    }

    pub fn from_port_voltage(
        port_idx: usize,
        voltage: PortVoltage,
        orientation: PortOrientation,
    ) -> Self {
        Self::new(port_idx, voltage.as_boundary_incident_offset(orientation))
    }

    pub fn from_typed_port_voltage(
        port: ScatteringPortId,
        voltage: PortVoltage,
        orientation: PortOrientation,
    ) -> Self {
        Self::for_port(port, voltage.as_boundary_incident_offset(orientation))
    }
}

pub fn sum_incident_offsets(port_idx: usize, drives: &[BoundaryIncidentDrive]) -> Wave {
    sum_incident_offsets_for_port(ScatteringPortId::new(port_idx), drives)
}

pub fn sum_incident_offsets_for_port(
    port: ScatteringPortId,
    drives: &[BoundaryIncidentDrive],
) -> Wave {
    drives
        .iter()
        .filter_map(|drive| {
            (drive.port == port && drive.incident_offset.0.is_finite())
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
    let positive: Vec<_> = positive_ports
        .iter()
        .copied()
        .map(ScatteringPortId::new)
        .collect();
    let negative: Vec<_> = negative_ports
        .iter()
        .copied()
        .map(ScatteringPortId::new)
        .collect();
    push_differential_voltage_drives_for_ports(drives, &positive, &negative, voltage);
}

pub fn push_differential_voltage_drives_for_ports(
    drives: &mut Vec<BoundaryIncidentDrive>,
    positive_ports: &[ScatteringPortId],
    negative_ports: &[ScatteringPortId],
    voltage: PortVoltage,
) {
    for &port in positive_ports {
        drives.push(BoundaryIncidentDrive::from_typed_port_voltage(
            port,
            voltage,
            PortOrientation::NonInverting,
        ));
    }
    for &port in negative_ports {
        drives.push(BoundaryIncidentDrive::from_typed_port_voltage(
            port,
            voltage,
            PortOrientation::Inverting,
        ));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec;

    #[test]
    fn domain_indices_keep_raw_ids_in_separate_type_domains() {
        let graph_node = GraphNodeId::new(7);
        let mna_node = MnaNodeId::new(7);
        let scattering_port = ScatteringPortId::new(7);
        let processor_port = ProcessorPortId::new(7);

        assert_eq!(graph_node.get(), 7);
        assert_eq!(mna_node.get(), 7);
        assert_eq!(scattering_port.get(), 7);
        assert_eq!(processor_port.get(), 7);
        assert!(GraphNodeId::new(usize::MAX).is_unresolved());
    }

    #[test]
    fn port_terminals_map_between_index_domains() {
        let graph_terminals = PortTerminals::differential(GraphNodeId::new(2), GraphNodeId::new(5));
        let mna_terminals = graph_terminals.map(|node| MnaNodeId::new(node.get() + 10));

        assert_eq!(
            mna_terminals.as_tuple(),
            (Some(MnaNodeId::new(12)), Some(MnaNodeId::new(15)))
        );
    }

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
            BoundaryIncidentDrive::for_port(ScatteringPortId::new(7), IncidentWave::new(-0.05)),
            BoundaryIncidentDrive::new(8, IncidentWave::new(1.0)),
        ];
        assert_eq!(sum_incident_offsets(7, &drives), 0.15000000000000002);
        assert_eq!(
            sum_incident_offsets_for_port(ScatteringPortId::new(7), &drives),
            0.15000000000000002
        );
        assert_eq!(drives[0].port_idx(), 7);
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
    fn circuit_mapped_port_uses_typed_graph_and_mna_domains() {
        let binding = CircuitMappedPort::new(
            GraphPortTerminals::differential(GraphNodeId::new(10), GraphNodeId::new(11)),
            PortSpec::new(
                MnaPortTerminals::differential(MnaNodeId::new(2), MnaNodeId::new(3)),
                220.0,
            ),
        );

        assert_eq!(binding.graph.raw().as_tuple(), (Some(10), Some(11)));
        assert_eq!(binding.mna.terminals.raw().as_tuple(), (Some(2), Some(3)));
        let port = binding.to_wdf_port();
        assert_eq!(port.node_pos, Some(2));
        assert_eq!(port.node_neg, Some(3));
        assert_eq!(port.resistance, 220.0);
    }

    #[test]
    fn one_port_describes_physical_value_separately_from_runtime_state() {
        let cap = OnePort::new(
            PortTerminals::differential("out", "ref"),
            OnePortKind::Capacitor(100e-9),
        );
        let mapped = cap.map(|node| match node {
            "out" => MnaNodeId::new(4),
            "ref" => MnaNodeId::new(5),
            _ => MnaNodeId::new(usize::MAX),
        });

        assert_eq!(mapped.terminals.raw().as_tuple(), (Some(4), Some(5)));
        assert!(mapped.kind.is_stateful());
        assert_eq!(OnePortKind::Resistor(1_000.0).is_stateful(), false);
        assert_eq!(
            OnePortState::CapacitorVoltage(0.25),
            OnePortState::CapacitorVoltage(0.25)
        );
        assert_eq!(
            OnePortState::InductorScaledCurrent(0.5),
            OnePortState::InductorScaledCurrent(0.5)
        );
    }

    #[test]
    fn runtime_one_port_derives_port_resistance_from_physical_spec() {
        let sample_rate = 48_000.0;
        let cap = RuntimeOnePort::new(
            OnePort::new(
                PortTerminals::<()>::grounded(),
                OnePortKind::Capacitor(100e-9),
            ),
            Some(StateSlot(2)),
        );
        let inductor = RuntimeOnePort::new(
            OnePort::new(PortTerminals::<()>::grounded(), OnePortKind::Inductor(1e-3)),
            Some(StateSlot(3)),
        );
        let resistor = RuntimeOnePort::new(
            OnePort::new(
                PortTerminals::<()>::grounded(),
                OnePortKind::Resistor(1_000.0),
            ),
            None,
        );

        assert_eq!(cap.state_slot, Some(StateSlot(2)));
        assert!((cap.rp(sample_rate) - 104.166_666_666_666_67).abs() < 1e-9);
        assert_eq!(inductor.rp(sample_rate), 96.0);
        assert_eq!(resistor.rp(sample_rate), 1_000.0);
    }

    #[test]
    fn runtime_state_allocates_dense_physical_state_and_wdf_cache_slots() {
        let mut runtime = RuntimeState::new();

        assert_eq!(
            runtime.allocate_one_port(OnePortKind::Resistor(1_000.0)),
            None
        );
        let cap_slot = runtime
            .allocate_one_port(OnePortKind::Capacitor(100e-9))
            .unwrap();
        let inductor_slot = runtime
            .allocate_one_port(OnePortKind::Inductor(1e-3))
            .unwrap();

        assert_eq!(cap_slot, StateSlot(0));
        assert_eq!(inductor_slot, StateSlot(1));
        assert_eq!(runtime.states.len(), runtime.wave_cache.len());
        assert_eq!(
            runtime.states[cap_slot.0],
            OnePortState::CapacitorVoltage(0.0)
        );
        assert_eq!(
            runtime.states[inductor_slot.0],
            OnePortState::InductorScaledCurrent(0.0)
        );
        assert_eq!(runtime.wave_cache[cap_slot.0], WdfWaveCache::default());
    }

    #[test]
    fn variable_resistor_binding_names_child_terminals_and_current_conductance() {
        let binding = VariableResistorBinding {
            child_idx: 3,
            terminals: WdfPortTerminals::single_ended(7),
            conductance: 0.001,
        };

        assert_eq!(binding.child_idx, 3);
        assert_eq!(binding.terminals.as_tuple(), (Some(7), None));
        assert_eq!(binding.conductance, 0.001);
    }

    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    enum TestRole {
        Input,
        Feedback,
    }

    #[test]
    fn role_port_binding_separates_semantic_role_from_port_index() {
        let binding = RolePortBinding::new(ScatteringPortId::new(4), TestRole::Feedback);

        assert_eq!(binding.port.get(), 4);
        assert_eq!(binding.role, TestRole::Feedback);

        let legacy_index_binding = RolePortBinding::new(2usize, TestRole::Input);
        assert_eq!(legacy_index_binding.port_idx(), 2);
    }

    #[test]
    fn named_and_feedback_bindings_name_source_ports() {
        let named = NamedPortBinding::new(alloc::string::String::from("audio_in"), 3usize).typed();
        let feedback = FeedbackPortBinding::new(1, 7usize).typed();

        assert_eq!(named.name, "audio_in");
        assert_eq!(named.port, ScatteringPortId::new(3));
        assert_eq!(feedback.source_idx, 1);
        assert_eq!(feedback.port, ScatteringPortId::new(7));
    }

    #[test]
    fn extraction_probe_reads_reflected_wave_vector() {
        let probe = ExtractionProbe::new(vec![0.25, -0.5, 0.0], Some(WdfPortTerminals::grounded()));
        let reflected = [2.0, -1.0, 9.0];

        assert!(probe.reads_from(&reflected));
        assert_eq!(probe.read_reflected(&reflected), Some(1.0));
        assert_eq!(probe.read_reflected(&reflected[..2]), None);
    }

    #[test]
    fn linear_multiport_network_scatters_reflected_to_incident() {
        let ports = vec![
            CircuitMappedPort::new(
                GraphPortTerminals::single_ended(GraphNodeId::new(1)),
                PortSpec::new(MnaPortTerminals::single_ended(MnaNodeId::new(1)), 100.0),
            ),
            CircuitMappedPort::new(
                GraphPortTerminals::single_ended(GraphNodeId::new(2)),
                PortSpec::new(MnaPortTerminals::single_ended(MnaNodeId::new(2)), 200.0),
            ),
        ];
        let mut network = LinearMultiportNetwork::new(vec![0.0, 1.0, 1.0, 0.0], ports);
        network.source_drives.push(NamedPortBinding::new(
            alloc::string::String::from("cv"),
            ScatteringPortId::new(1),
        ));
        network.extraction = Some(ExtractionProbe::new(
            vec![0.5, 0.5],
            Some(MnaPortTerminals::single_ended(MnaNodeId::new(2))),
        ));

        let mut incident = [0.0, 0.0];
        assert!(network.scatter_into(&[0.25, -0.75], &mut incident));
        assert_eq!(incident, [-0.75, 0.25]);
        assert_eq!(
            network
                .extraction
                .as_ref()
                .unwrap()
                .read_reflected(&incident),
            Some(-0.25)
        );
    }
}
