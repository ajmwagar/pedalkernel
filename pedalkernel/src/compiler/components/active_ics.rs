//! Active IC component structs: OpAmp, Vco, Vcf, Vca, Comparator, AnalogSwitch,
//! MatchedNpn, MatchedPnp.

use hashbrown::HashMap;

use crate::compiler::classify::NonlinearKind;
use crate::compiler::component::{
    Component, ComponentEdge, EdgeKind, GraphRole, ModulationSink, ModulationSinkKind, NonIdealFx,
    PinConfig, PinDirection, ResolveContext, SignalTerminals, StampContext, StampResult,
};
use crate::compiler::graph::NodeId;
use crate::dsl::{
    AnalogSwitchType, ComparatorType, MatchedTransistorType, OpAmpType, VcaType, VcfType, VcoType,
    VcoWaveformDsl,
};
use crate::tree::MnaSystem;

use super::impl_component_dyn;

// ═══════════════════════════════════════════════════════════════════════════
// OpAmp
// ═══════════════════════════════════════════════════════════════════════════

#[derive(Debug, Clone, PartialEq)]
pub struct OpAmp {
    pub op_type: OpAmpType,
}

impl Component for OpAmp {
    impl_component_dyn!();

    fn type_tag(&self) -> &'static str {
        if self.op_type.is_ota() {
            "OTA"
        } else {
            "op-amp"
        }
    }

    fn ports(&self) -> Vec<(&'static str, &'static str)> {
        if self.op_type.is_ota() {
            vec![("pos", "neg")]
        } else {
            vec![("neg", "out")]
        }
    }

    fn output_impedance(&self) -> crate::compiler::component::OutputImpedance {
        if self.op_type.is_ota() {
            // OTA output is a current source (high impedance)
            crate::compiler::component::OutputImpedance::Finite
        } else {
            // Op-amp output is a voltage source (zero impedance via feedback)
            crate::compiler::component::OutputImpedance::VoltageSource
        }
    }

    fn nonideal_fx(&self, _sample_rate: f64) -> Vec<NonIdealFx> {
        if self.op_type.is_ota() {
            return Vec::new(); // OTA non-idealities handled differently
        }
        let model = crate::model_lookup::opamp_model_from_type(&self.op_type);
        vec![
            NonIdealFx::OpAmpBandwidth {
                gbw: model.gbw,
                slew_rate: model.slew_rate,
            },
            // Default rail saturation — overridden by apply_bias() when a
            // bias network is detected. The v_max here assumes the minimum
            // common pedal supply (9V). Pipeline patches this after bias
            // analysis with the actual supply voltage and bias point.
            NonIdealFx::RailSaturation { v_max: 3.0 },
        ]
    }

    fn signal_terminals(&self) -> SignalTerminals {
        if self.op_type.is_ota() {
            SignalTerminals::Amplifier {
                input: "pos",
                output: "neg",
                control: None,
            }
        } else {
            SignalTerminals::Amplifier {
                input: "neg",
                output: "out",
                control: Some("pos"),
            }
        }
    }

    fn is_passive(&self) -> bool {
        false
    }

    // feedback_input_is_barrier: removed — the trait default now derives it from
    // signal_terminals() (Amplifier { control: Some } ⇒ barrier). Op-amp returns
    // control: Some("pos"); OTA returns control: None — same result, no hardcode.

    fn is_nonlinear(&self) -> bool {
        self.op_type.is_ota()
    }

    fn is_active_ic(&self) -> bool {
        !self.op_type.is_ota()
    }

    fn port_semantic(&self, pin_a: &str, pin_b: &str) -> crate::compiler::component::PortSemantic {
        if self.op_type.is_ota() {
            return crate::compiler::component::PortSemantic::LinearPassive;
        }
        // VCVS output is a voltage constraint
        let pins = [pin_a, pin_b];
        if pins.contains(&"out") || pins.contains(&"output") {
            crate::compiler::component::PortSemantic::VoltageConstraint
        } else {
            crate::compiler::component::PortSemantic::LinearPassive
        }
    }

    fn pin_config(&self) -> PinConfig {
        PinConfig {
            valid_pins: &["pos", "neg", "out", "output", "vp", "in", "input"],
            aliases: &[("in", "input"), ("out", "output")],
        }
    }

    fn modulation_pins(&self) -> &'static [&'static str] {
        if self.op_type == OpAmpType::Ca3080 {
            &["iabc"]
        } else {
            &[]
        }
    }

    fn graph_role(&self) -> GraphRole {
        if self.op_type.is_ota() {
            GraphRole::Edge {
                pin_a: "pos",
                pin_b: "neg",
            }
        } else {
            // Non-OTA op-amps participate in the graph as three-pin nullors
            // (pos, neg, out). The R-type stage builder emits a VCVS stamp.
            GraphRole::VcvsEdge {
                pin_pos: "pos",
                pin_neg: "neg",
                pin_out: "out",
            }
        }
    }

    fn stamp_mna(
        &self,
        _comp_id: &str,
        _n1: Option<usize>,
        _n2: Option<usize>,
        _mna: &mut MnaSystem,
        _sample_rate: f64,
    ) -> StampResult {
        // Two-terminal path unused for op-amps. Use stamp_mna_multi.
        StampResult::Skip
    }

    fn mna_vsource_count(&self) -> usize {
        if self.op_type.is_ota() {
            0
        } else {
            1
        }
    }

    fn mna_internal_node_count(&self) -> usize {
        0 // No internal node — simple VCVS stamp
    }

    fn stamp_mna_multi(
        &self,
        _comp_id: &str,
        ctx: &mut StampContext,
        mna: &mut MnaSystem,
    ) -> StampResult {
        if self.op_type.is_ota() {
            return StampResult::Skip;
        }
        let model = crate::model_lookup::opamp_model_from_type(&self.op_type);
        let ro = model.output_impedance;

        let aol = model.open_loop_gain;

        let out_mna = (ctx.pin_to_mna)("out");
        mna.stamp_vcvs(
            (ctx.pin_to_mna)("pos"),
            (ctx.pin_to_mna)("neg"),
            out_mna,
            None,
            aol,
            ro,
            ctx.vsrc_base,
        );

        // Output capacitance: keeps the output node as a dynamic state
        // in the Schur complement reduction, preserving the feedback loop's
        // energy recirculation. Without this, the output is eliminated
        // algebraically and the Q drops to ~0.3.
        if model.output_capacitance > 0.0 {
            if let Some(ref mut reactive_one_ports) = ctx.reactive_one_ports {
                reactive_one_ports.push(pedalkernel_rt::boundary_math::OnePort::new(
                    pedalkernel_rt::boundary_math::MnaPortTerminals::maybe_single_ended(
                        out_mna.map(pedalkernel_rt::boundary_math::MnaNodeId::new),
                    ),
                    pedalkernel_rt::boundary_math::OnePortKind::Capacitor(model.output_capacitance),
                ));
            }
        }

        StampResult::Stamped
    }

    fn edges(&self) -> Vec<ComponentEdge> {
        if self.op_type.is_ota() {
            vec![ComponentEdge {
                pin_a: "pos",
                pin_b: "neg",
                kind: EdgeKind::Vccs,
                port_group: None,
            }]
        } else {
            // VCVS edge: neg→out carries the nullor constraint.
            // pos is resolved via stamp_mna_multi's pin_to_mna("pos").
            vec![ComponentEdge {
                pin_a: "neg",
                pin_b: "out",
                kind: EdgeKind::Vcvs,
                port_group: None,
            }]
        }
    }

    fn resolve_edges(&self, ctx: &ResolveContext) -> Option<Vec<ComponentEdge>> {
        if self.op_type.is_ota() && ctx.control_pin_is_modulated {
            Some(vec![ComponentEdge {
                pin_a: "pos",
                pin_b: "neg",
                kind: EdgeKind::Vccs,
                port_group: None,
            }])
        } else {
            None
        }
    }

    fn classify_nonlinear(
        &self,
        _comp_id: &str,
        _node_a: NodeId,
        node_b: NodeId,
        gnd_node: NodeId,
        _node_names: &HashMap<String, NodeId>,
    ) -> Option<(NonlinearKind, Vec<NodeId>)> {
        if self.op_type.is_ota() {
            let jn = if node_b == gnd_node { _node_a } else { node_b };
            Some((NonlinearKind::Ota, vec![jn]))
        } else {
            None
        }
    }

    fn modulation_sink(&self, pin: &str) -> Option<ModulationSink> {
        if self.op_type.is_ota() {
            match pin {
                "iabc" => Some(ModulationSink {
                    target_kind: ModulationSinkKind::OtaIabc,
                    bias: 0.5,
                    range: 0.5,
                }),
                _ => None,
            }
        } else {
            None
        }
    }

    fn footprint_ref(&self) -> (&'static str, &'static str) {
        let lib = match self.op_type {
            OpAmpType::Generic => "Amplifier_Operational:TL072",
            OpAmpType::Tl072 => "Amplifier_Operational:TL072",
            OpAmpType::Tl082 => "Amplifier_Operational:TL082",
            OpAmpType::Jrc4558 => "Amplifier_Operational:JRC4558",
            OpAmpType::Rc4558 => "Amplifier_Operational:RC4558",
            OpAmpType::Lm308 => "Amplifier_Operational:LM308",
            OpAmpType::Lm741 => "Amplifier_Operational:LM741",
            OpAmpType::Ne5532 => "Amplifier_Operational:NE5532",
            OpAmpType::Ca3080 => "Amplifier_Operational:CA3080",
            OpAmpType::Op07 => "Amplifier_Operational:OP07",
        };
        (lib, "U")
    }

    fn symbol_name(&self) -> &'static str {
        "opamp"
    }
    fn layout_class(&self) -> &'static str {
        "opamp"
    }

    fn signal_adjacencies(&self) -> Vec<(&'static str, &'static str)> {
        vec![("pos", "out"), ("neg", "out")]
    }

    fn pin_direction(&self, pin: &str) -> PinDirection {
        match pin {
            "pos" | "neg" => PinDirection::Input,
            "out" | "output" => PinDirection::Output,
            _ => PinDirection::Bidirectional,
        }
    }

    fn op_amp_type(&self) -> Option<OpAmpType> {
        Some(self.op_type)
    }

    fn apply_bias(
        &self,
        bias_voltages: &hashbrown::HashMap<String, f64>,
        supply_voltage: f64,
    ) -> crate::compiler::component::BiasResult {
        // Op-amp bias sets the DC operating point (output swing center).
        // The bias voltage at the pos/neg input determines the output's
        // midpoint. Rail limits are supply_voltage minus headroom on each side.
        //
        // For a symmetric divider (bias = supply/2 = 4.5V at 9V):
        //   v_rail_pos = supply - bias - headroom = 9 - 4.5 - 1.5 = 3.0V
        //   v_rail_neg = bias - headroom = 4.5 - 1.5 = 3.0V
        //
        // For asymmetric (bias = 2.25V at 9V):
        //   v_rail_pos = 9 - 2.25 - 1.5 = 5.25V
        //   v_rail_neg = 2.25 - 1.5 = 0.75V
        const HEADROOM: f64 = 1.5; // Typical output stage headroom

        // Use bias voltage if provided; default to supply/2 (symmetric)
        let bias_v = bias_voltages
            .get("pos")
            .or_else(|| bias_voltages.get("neg"))
            .copied()
            .unwrap_or(supply_voltage / 2.0);

        let v_rail_pos = (supply_voltage - bias_v - HEADROOM).max(0.5);
        let v_rail_neg = (bias_v - HEADROOM).max(0.5);

        crate::compiler::component::BiasResult::Applied {
            v_rail_pos,
            v_rail_neg,
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Vco
// ═══════════════════════════════════════════════════════════════════════════

#[derive(Debug, Clone, PartialEq)]
pub struct Vco {
    pub vco_type: VcoType,
    /// Base frequency in Hz (default 440.0).
    pub base_freq: f64,
    /// Default waveform output.
    pub waveform: VcoWaveformDsl,
}

impl Component for Vco {
    impl_component_dyn!();

    fn type_tag(&self) -> &'static str {
        "VCO"
    }

    fn is_passive(&self) -> bool {
        false
    }

    fn is_active_ic(&self) -> bool {
        false
    }

    fn pin_config(&self) -> PinConfig {
        PinConfig {
            valid_pins: &["cv", "saw", "tri", "pulse", "pw", "sync", "out", "output"],
            aliases: &[("out", "output")],
        }
    }

    fn graph_role(&self) -> GraphRole {
        GraphRole::Virtual
    }

    fn stamp_mna(
        &self,
        _comp_id: &str,
        _n1: Option<usize>,
        _n2: Option<usize>,
        _mna: &mut MnaSystem,
        _sample_rate: f64,
    ) -> StampResult {
        StampResult::Skip
    }

    fn footprint_ref(&self) -> (&'static str, &'static str) {
        match self.vco_type {
            VcoType::Cem3340 => ("Oscillator:CEM3340", "U"),
            VcoType::As3340 => ("Oscillator:AS3340", "U"),
            VcoType::V3340 => ("Oscillator:V3340", "U"),
        }
    }

    fn symbol_name(&self) -> &'static str {
        "ic_chip"
    }
    fn layout_class(&self) -> &'static str {
        "vco"
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Vcf
// ═══════════════════════════════════════════════════════════════════════════

#[derive(Debug, Clone, PartialEq)]
pub struct Vcf {
    pub vcf_type: VcfType,
}

impl Component for Vcf {
    impl_component_dyn!();

    fn type_tag(&self) -> &'static str {
        "VCF"
    }

    fn is_passive(&self) -> bool {
        false
    }

    fn is_active_ic(&self) -> bool {
        true
    }

    fn pin_config(&self) -> PinConfig {
        PinConfig {
            valid_pins: &["in", "input", "out", "output", "cv", "res"],
            aliases: &[("in", "input"), ("out", "output")],
        }
    }

    fn graph_role(&self) -> GraphRole {
        GraphRole::ActiveIc
    }

    fn stamp_mna(
        &self,
        _comp_id: &str,
        _n1: Option<usize>,
        _n2: Option<usize>,
        _mna: &mut MnaSystem,
        _sample_rate: f64,
    ) -> StampResult {
        StampResult::Skip
    }

    fn footprint_ref(&self) -> (&'static str, &'static str) {
        match self.vcf_type {
            VcfType::Cem3320 => ("Analog:CEM3320", "U"),
            VcfType::As3320 => ("Analog:AS3320", "U"),
        }
    }

    fn symbol_name(&self) -> &'static str {
        "ic_chip"
    }
    fn layout_class(&self) -> &'static str {
        "vcf"
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Vca
// ═══════════════════════════════════════════════════════════════════════════

#[derive(Debug, Clone, PartialEq)]
pub struct Vca {
    pub vca_type: VcaType,
}

impl Component for Vca {
    impl_component_dyn!();

    fn type_tag(&self) -> &'static str {
        "VCA"
    }

    fn is_passive(&self) -> bool {
        false
    }

    fn is_active_ic(&self) -> bool {
        false
    }

    fn pin_config(&self) -> PinConfig {
        PinConfig {
            valid_pins: &[
                "in", "input", "out", "output", "cv", "in1", "out1", "cv1", "in2", "out2", "cv2",
                "in3", "out3", "cv3", "in4", "out4", "cv4",
            ],
            aliases: &[("in", "input"), ("out", "output")],
        }
    }

    fn graph_role(&self) -> GraphRole {
        GraphRole::Virtual
    }

    fn stamp_mna(
        &self,
        _comp_id: &str,
        _n1: Option<usize>,
        _n2: Option<usize>,
        _mna: &mut MnaSystem,
        _sample_rate: f64,
    ) -> StampResult {
        StampResult::Skip
    }

    fn modulation_sink(&self, pin: &str) -> Option<ModulationSink> {
        match pin {
            // Scaling rationale (audit gap G4): the runtime Vca maps a
            // normalized CV 0..1 onto 0..-80 dB, the SSM2164-class dB-linear
            // law (see pedalkernel-rt `Vca::set_cv_normalized`). The engine's
            // envelope followers output a rectified average — ~1.3-2 for
            // full-scale program (2/π × a typical sensitivity of 2.0 from a
            // 20k sensitivity resistor, times any buffer gain ahead of the
            // tap) — so cv = bias + env × range with bias 0, range 0.05 puts
            // full-scale program at cv ≈ 0.07-0.1 ≈ 5-8 dB of gain reduction
            // and a ~2:1 ratio over the top 12 dB (SSL-bus-comp-like glue).
            // Larger ranges over-compress: because the detector is linear in
            // amplitude, GR in dB grows ~10^(in_db/20), and beyond ~11 dB of
            // full-scale GR the static curve's top end slopes DOWNWARD
            // (output falls as input rises — negative ratio).
            "cv" | "cv1" | "cv2" | "cv3" | "cv4" => Some(ModulationSink {
                target_kind: ModulationSinkKind::VcaCv,
                bias: 0.0,
                range: 0.05,
            }),
            _ => None,
        }
    }

    fn footprint_ref(&self) -> (&'static str, &'static str) {
        match self.vca_type {
            VcaType::Ssm2164 => ("Analog:SSM2164", "U"),
            VcaType::V2164 => ("Analog:V2164", "U"),
        }
    }

    fn symbol_name(&self) -> &'static str {
        "ic_chip"
    }
    fn layout_class(&self) -> &'static str {
        "vca"
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Comparator
// ═══════════════════════════════════════════════════════════════════════════

#[derive(Debug, Clone, PartialEq)]
pub struct Comparator {
    pub comp_type: ComparatorType,
}

impl Component for Comparator {
    impl_component_dyn!();

    fn type_tag(&self) -> &'static str {
        "comparator"
    }

    fn is_passive(&self) -> bool {
        false
    }

    fn is_active_ic(&self) -> bool {
        true
    }

    fn pin_config(&self) -> PinConfig {
        PinConfig {
            valid_pins: &["pos", "neg", "out", "output"],
            aliases: &[("out", "output")],
        }
    }

    fn graph_role(&self) -> GraphRole {
        GraphRole::ActiveIc
    }

    fn stamp_mna(
        &self,
        _comp_id: &str,
        _n1: Option<usize>,
        _n2: Option<usize>,
        _mna: &mut MnaSystem,
        _sample_rate: f64,
    ) -> StampResult {
        StampResult::Skip
    }

    fn footprint_ref(&self) -> (&'static str, &'static str) {
        match self.comp_type {
            ComparatorType::Lm311 => ("Comparator:LM311", "U"),
            ComparatorType::Lm393 => ("Comparator:LM393", "U"),
        }
    }

    fn symbol_name(&self) -> &'static str {
        "ic_chip"
    }
    fn layout_class(&self) -> &'static str {
        "comparator"
    }

    fn pin_direction(&self, pin: &str) -> PinDirection {
        match pin {
            "pos" | "neg" => PinDirection::Input,
            "out" | "output" => PinDirection::Output,
            _ => PinDirection::Bidirectional,
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// AnalogSwitch
// ═══════════════════════════════════════════════════════════════════════════

#[derive(Debug, Clone, PartialEq)]
pub struct AnalogSwitch {
    pub switch_type: AnalogSwitchType,
}

impl Component for AnalogSwitch {
    impl_component_dyn!();

    fn type_tag(&self) -> &'static str {
        "analog switch"
    }

    fn is_passive(&self) -> bool {
        false
    }

    fn is_active_ic(&self) -> bool {
        true
    }

    fn pin_config(&self) -> PinConfig {
        PinConfig {
            valid_pins: &[
                "in1", "out1", "ctrl1", "in2", "out2", "ctrl2", "in3", "out3", "ctrl3", "in4",
                "out4", "ctrl4",
            ],
            aliases: &[],
        }
    }

    fn graph_role(&self) -> GraphRole {
        GraphRole::ActiveIc
    }

    fn stamp_mna(
        &self,
        _comp_id: &str,
        _n1: Option<usize>,
        _n2: Option<usize>,
        _mna: &mut MnaSystem,
        _sample_rate: f64,
    ) -> StampResult {
        StampResult::Skip
    }

    fn footprint_ref(&self) -> (&'static str, &'static str) {
        match self.switch_type {
            AnalogSwitchType::Cd4066 => ("Analog_Switch:CD4066", "U"),
            AnalogSwitchType::Dg411 => ("Analog_Switch:DG411", "U"),
        }
    }

    fn symbol_name(&self) -> &'static str {
        "ic_chip"
    }
    fn layout_class(&self) -> &'static str {
        "analog_switch"
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// MatchedNpn
// ═══════════════════════════════════════════════════════════════════════════

#[derive(Debug, Clone, PartialEq)]
pub struct MatchedNpn {
    pub matched_type: MatchedTransistorType,
}

impl Component for MatchedNpn {
    impl_component_dyn!();

    fn type_tag(&self) -> &'static str {
        "matched NPN pair"
    }

    fn is_passive(&self) -> bool {
        false
    }

    fn is_active_ic(&self) -> bool {
        true
    }

    fn pin_config(&self) -> PinConfig {
        PinConfig {
            valid_pins: &[
                "base",
                "collector",
                "emitter",
                "base1",
                "base2",
                "collector1",
                "collector2",
                "emitter1",
                "emitter2",
            ],
            aliases: &[],
        }
    }

    fn graph_role(&self) -> GraphRole {
        GraphRole::ActiveIc
    }

    fn stamp_mna(
        &self,
        _comp_id: &str,
        _n1: Option<usize>,
        _n2: Option<usize>,
        _mna: &mut MnaSystem,
        _sample_rate: f64,
    ) -> StampResult {
        StampResult::Skip
    }

    fn footprint_ref(&self) -> (&'static str, &'static str) {
        match self.matched_type {
            MatchedTransistorType::Ssm2210 => ("Transistor_BJT:SSM2210", "Q"),
            MatchedTransistorType::Ca3046 => ("Transistor_Array:CA3046", "Q"),
            MatchedTransistorType::Lm394 => ("Transistor_BJT:LM394", "Q"),
            MatchedTransistorType::That340 => ("Transistor_BJT:THAT340", "Q"),
        }
    }

    fn symbol_name(&self) -> &'static str {
        "npn_bjt"
    }
    fn layout_class(&self) -> &'static str {
        "matched_npn"
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// MatchedPnp
// ═══════════════════════════════════════════════════════════════════════════

#[derive(Debug, Clone, PartialEq)]
pub struct MatchedPnp {
    pub matched_type: MatchedTransistorType,
}

impl Component for MatchedPnp {
    impl_component_dyn!();

    fn type_tag(&self) -> &'static str {
        "matched PNP pair"
    }

    fn is_passive(&self) -> bool {
        false
    }

    fn is_active_ic(&self) -> bool {
        true
    }

    fn pin_config(&self) -> PinConfig {
        PinConfig {
            valid_pins: &[
                "base",
                "collector",
                "emitter",
                "base1",
                "base2",
                "collector1",
                "collector2",
                "emitter1",
                "emitter2",
            ],
            aliases: &[],
        }
    }

    fn graph_role(&self) -> GraphRole {
        GraphRole::ActiveIc
    }

    fn stamp_mna(
        &self,
        _comp_id: &str,
        _n1: Option<usize>,
        _n2: Option<usize>,
        _mna: &mut MnaSystem,
        _sample_rate: f64,
    ) -> StampResult {
        StampResult::Skip
    }

    fn footprint_ref(&self) -> (&'static str, &'static str) {
        match self.matched_type {
            MatchedTransistorType::Ssm2210 => ("Transistor_BJT:SSM2210", "Q"),
            MatchedTransistorType::Ca3046 => ("Transistor_Array:CA3046", "Q"),
            MatchedTransistorType::Lm394 => ("Transistor_BJT:LM394", "Q"),
            MatchedTransistorType::That340 => ("Transistor_BJT:THAT340", "Q"),
        }
    }

    fn symbol_name(&self) -> &'static str {
        "pnp_bjt"
    }
    fn layout_class(&self) -> &'static str {
        "matched_pnp"
    }
}
