//! Active IC component structs: OpAmp, Vco, Vcf, Vca, Comparator, AnalogSwitch,
//! MatchedNpn, MatchedPnp.

use std::collections::HashMap;

use crate::compiler::classify::NonlinearKind;
use crate::compiler::component::{
    Component, ComponentEdge, EdgeKind, GraphRole, ModulationSink, ModulationSinkKind, PinConfig,
    PinDirection, ResolveContext, StampContext, StampResult,
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

    fn is_passive(&self) -> bool {
        false
    }

    fn is_nonlinear(&self) -> bool {
        self.op_type.is_ota()
    }

    fn is_active_ic(&self) -> bool {
        !self.op_type.is_ota()
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
        if self.op_type.is_ota() { 0 } else { 1 }
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
        let model = crate::elements::OpAmpModel::from_opamp_type(&self.op_type);
        let ro = model.output_impedance;

        // For the state-space MNA path, use GBW-limited effective gain.
        // The dominant pole at f_p = GBW/Aol creates gain rolloff.
        // At the audio band center (~130 Hz for a kick drum), the effective
        // Aol = GBW / f_audio. This keeps the loop gain near the oscillation
        // threshold, giving complex eigenvalues at the correct frequency.
        //
        // For non-resonant circuits (simple Rf/Ri feedback), the reduced
        // Aol still gives accurate closed-loop gain because Aol >> gain.
        // Use GBW-limited gain at a frequency that keeps eigenvalues complex.
        // The Schur complement modifies the effective loop gain — eigenvalues
        // go real above Aol≈55 for typical bridged-T circuits. Using
        // Aol = GBW / f_high keeps the gain in the complex-eigenvalue regime.
        let f_high = model.gbw / 50.0; // Target Aol ≈ 50
        let aol = (model.gbw / f_high).min(model.open_loop_gain);

        mna.stamp_vcvs(
            (ctx.pin_to_mna)("pos"),
            (ctx.pin_to_mna)("neg"),
            (ctx.pin_to_mna)("out"),
            None,
            aol,
            ro,
            ctx.vsrc_base,
        );
        StampResult::Stamped
    }

    fn edges(&self) -> Vec<ComponentEdge> {
        if self.op_type.is_ota() {
            vec![ComponentEdge {
                pin_a: "pos",
                pin_b: "neg",
                kind: EdgeKind::Nonlinear,
                port_group: None,
            }]
        } else {
            vec![]
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
