//! Pedalboard processor — chains multiple compiled pedals in series.
//!
//! Implements [`PedalProcessor`] so it can be used directly with
//! [`AudioEngine::start()`] — zero changes to the JACK engine.
//!
//! Control routing uses a `"idx:label"` prefix scheme:
//! - `"0:Drive"` → pedal index 0, knob "Drive"
//! - `"2:__bypass__"` → toggle bypass on pedal index 2

use crate::board::{find_global_board, BoardDef, BoardEntry};
use crate::compiler::{compile_pedal, compile_split_pedal, CompiledPedal, SplitCompiledPedal};
use crate::dsl::parse_pedal_file;
use crate::loading::InterstageLoading;
use crate::PedalProcessor;
use std::path::Path;

/// A single slot in the pedalboard chain.
///
/// When `processor` is `None`, the slot acts as dry passthrough — this
/// happens when a pedal fails to load or compile. A warning is emitted
/// at build time and the slot is marked as failed.
struct PedalSlot {
    name: String,
    processor: Option<CompiledPedal>,
    bypassed: bool,
    failed: bool,
}

/// An FX send/return loop backed by a split amp pedal.
///
/// The amp's pre-section runs first, then the FX chain, then the post-section.
struct FxLoopSlot {
    /// The split amp pedal (pre → FX chain → post).
    amp: SplitCompiledPedal,
    /// Effects inside the FX loop.
    effects: Vec<PedalSlot>,
    /// Wet/dry mix for the FX chain (0.0 = bypass loop, 1.0 = full wet).
    mix: f64,
}

/// A slot in the signal chain — either a single pedal or an FX loop group.
enum ChainSlot {
    Pedal(PedalSlot),
    FxLoop(FxLoopSlot),
}

/// Chains N `CompiledPedal` instances in series, with optional FX loop groups.
///
/// When impedance loading is enabled, interstage loading models are inserted
/// between adjacent pedals to simulate the electrical interaction between
/// source and load impedances.
pub struct PedalboardProcessor {
    chain: Vec<ChainSlot>,
    /// Interstage loading between pedals. `loading[i]` sits between
    /// `chain[i]` and `chain[i+1]`. Length = max(0, chain.len() - 1).
    loading: Vec<InterstageLoading>,
}

impl PedalboardProcessor {
    /// Build a pedalboard from a parsed `.board` definition.
    ///
    /// Each `.pedal` file is read relative to `board_dir`, compiled at the
    /// given sample rate, and any knob overrides from the board file are applied.
    ///
    /// If a pedal fails to load or compile, it is replaced with a dry
    /// passthrough slot and a warning is emitted to stderr. The board
    /// still loads — only the broken pedal is silent.
    pub fn from_board(
        board: &BoardDef,
        board_dir: &Path,
        sample_rate: f64,
    ) -> Result<Self, String> {
        let (pb, warnings) = Self::from_board_with_warnings(board, board_dir, sample_rate);
        for w in &warnings {
            eprintln!("WARNING: {w}");
        }
        Ok(pb)
    }

    /// Build a pedalboard, returning both the processor and any warnings.
    ///
    /// Pedals that fail to read, parse, or compile are replaced with dry
    /// passthrough slots. The returned warnings describe each failure.
    ///
    /// If a `.global.board` file is found (by walking up from `board_dir`),
    /// its `start` pedals are prepended and `end` pedals are appended to
    /// the signal chain.
    pub fn from_board_with_warnings(
        board: &BoardDef,
        board_dir: &Path,
        sample_rate: f64,
    ) -> (Self, Vec<String>) {
        let mut chain = Vec::new();
        let mut warnings = Vec::new();

        // Detect global board
        let global = find_global_board(board_dir);
        if let Some((ref def, ref dir)) = global {
            let n_start = def.start.len();
            let n_end = def.end.len();
            eprintln!(
                "Global: {} ({} start, {} end)",
                dir.join(".global.board").display(),
                n_start,
                n_end,
            );
        }

        // Prepend global start pedals
        if let Some((ref def, ref global_dir)) = global {
            for entry in &def.start {
                let slot = Self::compile_pedal_entry(entry, global_dir, sample_rate, &mut warnings);
                chain.push(ChainSlot::Pedal(slot));
            }
        }

        // Build main board entries
        for entry in &board.entries {
            match entry {
                BoardEntry::Pedal(pedal_entry) => {
                    let slot =
                        Self::compile_pedal_entry(pedal_entry, board_dir, sample_rate, &mut warnings);
                    chain.push(ChainSlot::Pedal(slot));
                }
                BoardEntry::FxLoop(group) => {
                    // Find the referenced amp pedal — it must have been declared
                    // earlier in the board as a regular Pedal entry.
                    // We need to load, parse, and compile it as a split pedal.
                    let amp_entry = board.entries.iter().find_map(|e| {
                        if let BoardEntry::Pedal(p) = e {
                            if p.id == group.target_id {
                                return Some(p);
                            }
                        }
                        None
                    });

                    let amp_result: Result<(String, SplitCompiledPedal), String> = match amp_entry {
                        None => Err(format!(
                            "fx_loop references '{}' but no pedal with that ID exists",
                            group.target_id
                        )),
                        Some(entry) => {
                            let pedal_path = {
                                let direct = board_dir.join(&entry.path);
                                if direct.exists() {
                                    direct
                                } else if let Some(found) =
                                    Self::find_in_subdirs(board_dir, &entry.path)
                                {
                                    found
                                } else {
                                    direct
                                }
                            };
                            (|| {
                                let source =
                                    std::fs::read_to_string(&pedal_path).map_err(|e| {
                                        format!(
                                            "Error reading amp '{}' at {}: {e}",
                                            entry.id,
                                            pedal_path.display()
                                        )
                                    })?;
                                let pedal_def = parse_pedal_file(&source).map_err(|e| {
                                    format!("Parse error in '{}': {e}", entry.id)
                                })?;

                                if !pedal_def.has_fx_loop() {
                                    return Err(format!(
                                        "Pedal '{}' has no fx_send/fx_return nodes",
                                        entry.id
                                    ));
                                }

                                let mut split = compile_split_pedal(&pedal_def, sample_rate)
                                    .map_err(|e| {
                                        format!("Failed to split-compile '{}': {e}", entry.id)
                                    })?;

                                // Apply overrides from the board
                                for ctrl in &pedal_def.controls {
                                    split.set_control(&ctrl.label, ctrl.default);
                                }
                                for (label, val) in &entry.overrides {
                                    split.set_control(label, *val);
                                }

                                Ok((pedal_def.name.clone(), split))
                            })()
                        }
                    };

                    match amp_result {
                        Ok((name, split)) => {
                            // Remove the regular pedal entry for this amp from the chain
                            // (it's now handled by the FX loop)
                            chain.retain(|slot| {
                                if let ChainSlot::Pedal(p) = slot {
                                    p.name != name || p.failed
                                } else {
                                    true
                                }
                            });

                            let mut effects = Vec::with_capacity(group.pedals.len());
                            for pedal_entry in &group.pedals {
                                let slot = Self::compile_pedal_entry(
                                    pedal_entry,
                                    board_dir,
                                    sample_rate,
                                    &mut warnings,
                                );
                                effects.push(slot);
                            }
                            chain.push(ChainSlot::FxLoop(FxLoopSlot {
                                amp: split,
                                effects,
                                mix: group.mix,
                            }));
                        }
                        Err(msg) => {
                            warnings.push(format!(
                                "FX loop for '{}' failed: {msg}",
                                group.target_id
                            ));
                            // Fall back: just add the FX loop effects as regular pedals
                            for pedal_entry in &group.pedals {
                                let slot = Self::compile_pedal_entry(
                                    pedal_entry,
                                    board_dir,
                                    sample_rate,
                                    &mut warnings,
                                );
                                chain.push(ChainSlot::Pedal(slot));
                            }
                        }
                    }
                }
            }
        }

        // Append global end pedals
        if let Some((ref def, ref global_dir)) = global {
            for entry in &def.end {
                let slot = Self::compile_pedal_entry(entry, global_dir, sample_rate, &mut warnings);
                chain.push(ChainSlot::Pedal(slot));
            }
        }

        // Create interstage loading between adjacent pedals.
        // Default: transparent (buffered output → high-Z input), which is the
        // most common modern pedal configuration and has negligible effect.
        let n = chain.len();
        let loading = if n > 1 {
            (0..n - 1)
                .map(|_| InterstageLoading::transparent(sample_rate))
                .collect()
        } else {
            Vec::new()
        };

        (Self { chain, loading }, warnings)
    }

    /// Build a pedalboard from a parsed `.board` definition, using a resolver
    /// function instead of filesystem reads.
    ///
    /// The `resolver` takes a pedal path string (as written in the `.board` file)
    /// and returns the `.pedal` source text.  This allows building boards from
    /// `include_str!`'d sources without any filesystem access — useful for
    /// embedded/factory boards in the VST plugin.
    ///
    /// Global boards and FX loops are not supported in resolver mode (boards
    /// referencing FX loops should use [`from_board_with_warnings`] instead).
    pub fn from_board_with_resolver(
        board: &BoardDef,
        sample_rate: f64,
        resolver: impl Fn(&str) -> Option<String>,
    ) -> (Self, Vec<String>) {
        let mut chain = Vec::new();
        let mut warnings = Vec::new();

        for entry in &board.entries {
            match entry {
                BoardEntry::Pedal(pedal_entry) => {
                    let slot = Self::compile_pedal_entry_from_source(
                        pedal_entry,
                        &resolver,
                        sample_rate,
                        &mut warnings,
                    );
                    chain.push(ChainSlot::Pedal(slot));
                }
                BoardEntry::FxLoop(group) => {
                    warnings.push(format!(
                        "FX loop '{}' skipped — not supported in resolver mode",
                        group.target_id,
                    ));
                    // Add the FX loop effects as regular pedals (best effort).
                    for pedal_entry in &group.pedals {
                        let slot = Self::compile_pedal_entry_from_source(
                            pedal_entry,
                            &resolver,
                            sample_rate,
                            &mut warnings,
                        );
                        chain.push(ChainSlot::Pedal(slot));
                    }
                }
            }
        }

        let n = chain.len();
        let loading = if n > 1 {
            (0..n - 1)
                .map(|_| InterstageLoading::transparent(sample_rate))
                .collect()
        } else {
            Vec::new()
        };

        (Self { chain, loading }, warnings)
    }

    /// Compile a pedal entry using a resolver instead of filesystem reads.
    fn compile_pedal_entry_from_source(
        entry: &crate::board::BoardPedalEntry,
        resolver: &impl Fn(&str) -> Option<String>,
        sample_rate: f64,
        warnings: &mut Vec<String>,
    ) -> PedalSlot {
        let result: Result<(String, CompiledPedal), String> = (|| {
            let source = resolver(&entry.path).ok_or_else(|| {
                format!("Pedal '{}' not found: {}", entry.id, entry.path)
            })?;
            let pedal_def = parse_pedal_file(&source)
                .map_err(|e| format!("Parse error in '{}' ({}): {e}", entry.id, entry.path))?;
            let mut proc = compile_pedal(&pedal_def, sample_rate).map_err(|e| {
                format!("Compilation error in '{}' ({}): {e}", entry.id, entry.path)
            })?;

            for ctrl in &pedal_def.controls {
                proc.set_control(&ctrl.label, ctrl.default);
            }
            for (label, val) in &entry.overrides {
                proc.set_control(label, *val);
            }

            Ok((pedal_def.name.clone(), proc))
        })();

        match result {
            Ok((name, proc)) => PedalSlot {
                name,
                processor: Some(proc),
                bypassed: false,
                failed: false,
            },
            Err(msg) => {
                warnings.push(format!(
                    "Pedal '{}' failed — using dry passthrough: {msg}",
                    entry.id
                ));
                PedalSlot {
                    name: format!("{} [FAILED]", entry.id),
                    processor: None,
                    bypassed: false,
                    failed: true,
                }
            }
        }
    }

    /// Compile a single pedal entry into a PedalSlot.
    fn compile_pedal_entry(
        entry: &crate::board::BoardPedalEntry,
        board_dir: &Path,
        sample_rate: f64,
        warnings: &mut Vec<String>,
    ) -> PedalSlot {
        // Try direct path first, then search subdirectories by filename.
        let pedal_path = {
            let direct = board_dir.join(&entry.path);
            if direct.exists() {
                direct
            } else if let Some(found) = Self::find_in_subdirs(board_dir, &entry.path) {
                found
            } else {
                direct // fall through to produce the original error message
            }
        };

        let result: Result<(String, CompiledPedal), String> = (|| {
            let source = std::fs::read_to_string(&pedal_path).map_err(|e| {
                format!(
                    "Error reading pedal '{}' at {}: {e}",
                    entry.id,
                    pedal_path.display()
                )
            })?;
            let pedal_def = parse_pedal_file(&source)
                .map_err(|e| format!("Parse error in '{}' ({}): {e}", entry.id, entry.path))?;
            let mut proc = compile_pedal(&pedal_def, sample_rate).map_err(|e| {
                format!("Compilation error in '{}' ({}): {e}", entry.id, entry.path)
            })?;

            // Apply default control values from the .pedal file
            for ctrl in &pedal_def.controls {
                proc.set_control(&ctrl.label, ctrl.default);
            }

            // Apply overrides from the .board file
            for (label, val) in &entry.overrides {
                proc.set_control(label, *val);
            }

            Ok((pedal_def.name.clone(), proc))
        })();

        match result {
            Ok((name, proc)) => PedalSlot {
                name,
                processor: Some(proc),
                bypassed: false,
                failed: false,
            },
            Err(msg) => {
                warnings.push(format!(
                    "Pedal '{}' failed — using dry passthrough: {msg}",
                    entry.id
                ));
                PedalSlot {
                    name: format!("{} [FAILED]", entry.id),
                    processor: None,
                    bypassed: false,
                    failed: true,
                }
            }
        }
    }

    /// Search subdirectories of `base` for a file matching `filename`.
    fn find_in_subdirs(base: &Path, filename: &str) -> Option<std::path::PathBuf> {
        let target = Path::new(filename)
            .file_name()?
            .to_str()?;
        Self::walk_for_file(base, target)
    }

    fn walk_for_file(dir: &Path, target: &str) -> Option<std::path::PathBuf> {
        for entry in std::fs::read_dir(dir).ok()?.flatten() {
            let path = entry.path();
            if path.is_dir() {
                if let Some(found) = Self::walk_for_file(&path, target) {
                    return Some(found);
                }
            } else if path.file_name().and_then(|n| n.to_str()) == Some(target) {
                return Some(path);
            }
        }
        None
    }

    /// Flat list of all pedal slots (for indexed access).
    /// FX loop effect slots are flattened — indices are stable across the board.
    fn flat_pedals(&self) -> Vec<&PedalSlot> {
        let mut out = Vec::new();
        for slot in &self.chain {
            match slot {
                ChainSlot::Pedal(p) => out.push(p),
                ChainSlot::FxLoop(lp) => {
                    for p in &lp.effects {
                        out.push(p);
                    }
                }
            }
        }
        out
    }

    /// Mutable flat list of all pedal slots.
    fn flat_pedals_mut(&mut self) -> Vec<&mut PedalSlot> {
        let mut out = Vec::new();
        for slot in &mut self.chain {
            match slot {
                ChainSlot::Pedal(p) => out.push(p),
                ChainSlot::FxLoop(lp) => {
                    for p in &mut lp.effects {
                        out.push(p);
                    }
                }
            }
        }
        out
    }

    /// Number of pedals in the chain (flattened, including FX loop contents).
    pub fn len(&self) -> usize {
        self.flat_pedals().len()
    }

    /// Whether the pedalboard is empty.
    pub fn is_empty(&self) -> bool {
        self.chain.is_empty()
    }

    /// Get the name of pedal at the given flat index.
    pub fn pedal_name(&self, index: usize) -> Option<&str> {
        self.flat_pedals().get(index).map(|s| s.name.as_str())
    }

    /// Check if a pedal is bypassed.
    pub fn is_pedal_bypassed(&self, index: usize) -> bool {
        self.flat_pedals()
            .get(index)
            .is_some_and(|s| s.bypassed)
    }

    /// Check if a pedal slot failed to compile (dry passthrough).
    pub fn is_pedal_failed(&self, index: usize) -> bool {
        self.flat_pedals()
            .get(index)
            .is_some_and(|s| s.failed)
    }

    /// Configure the impedance loading between two adjacent pedals.
    ///
    /// `junction_index` is the junction between `chain[junction_index]` and
    /// `chain[junction_index + 1]`. The `source` describes the output impedance
    /// of the preceding pedal, and `load` describes the input impedance of
    /// the following pedal.
    ///
    /// This models the electrical interaction that causes real pedals to sound
    /// different depending on what's connected before/after them. For example,
    /// a Fuzz Face after a buffered pedal sounds different than directly after
    /// a guitar, because its low input impedance loads the source differently.
    pub fn set_interstage_loading(
        &mut self,
        junction_index: usize,
        source: crate::loading::ImpedanceModel,
        load: crate::loading::ImpedanceModel,
        sample_rate: f64,
    ) {
        if junction_index < self.loading.len() {
            self.loading[junction_index] =
                InterstageLoading::new(source, load, sample_rate);
        }
    }

    /// Get the input impedance of the pedalboard (Ω).
    ///
    /// Returns the input impedance of the first pedal in the chain.
    /// If the chain is empty or the first pedal has no processor, returns high-Z (1MΩ).
    pub fn input_impedance(&self) -> f64 {
        for slot in &self.chain {
            match slot {
                ChainSlot::Pedal(p) => {
                    if !p.bypassed {
                        if let Some(ref proc) = p.processor {
                            return proc.input_impedance();
                        }
                    }
                }
                ChainSlot::FxLoop(fx) => {
                    // FX loop's input impedance is the amp's preamp input
                    return fx.amp.input_impedance();
                }
            }
        }
        1_000_000.0 // High-Z default
    }

    /// Get the output impedance of the pedalboard (Ω).
    ///
    /// Returns the output impedance of the last pedal in the chain.
    /// If the chain is empty or the last pedal has no processor, returns low-Z (1kΩ).
    pub fn output_impedance(&self) -> f64 {
        for slot in self.chain.iter().rev() {
            match slot {
                ChainSlot::Pedal(p) => {
                    if !p.bypassed {
                        if let Some(ref proc) = p.processor {
                            return proc.output_impedance();
                        }
                    }
                }
                ChainSlot::FxLoop(fx) => {
                    // FX loop's output impedance is the amp's poweramp output
                    return fx.amp.output_impedance();
                }
            }
        }
        1_000.0 // Low-Z default
    }
}

/// Process a single pedal slot, respecting bypass.
fn process_slot(slot: &mut PedalSlot, signal: f64) -> f64 {
    if slot.bypassed {
        signal
    } else if let Some(ref mut proc) = slot.processor {
        proc.process(signal)
    } else {
        signal // dry passthrough (failed pedal)
    }
}

impl PedalProcessor for PedalboardProcessor {
    fn process(&mut self, input: f64) -> f64 {
        let mut signal = input;
        let n = self.chain.len();
        for i in 0..n {
            match &mut self.chain[i] {
                ChainSlot::Pedal(slot) => {
                    signal = process_slot(slot, signal);
                }
                ChainSlot::FxLoop(fx) => {
                    // Pre-amp section → FX send
                    let send = fx.amp.process_pre(signal);
                    // FX chain with wet/dry mix
                    let dry = send;
                    let mut wet = send;
                    for slot in &mut fx.effects {
                        wet = process_slot(slot, wet);
                    }
                    let fx_return = dry * (1.0 - fx.mix) + wet * fx.mix;
                    // Post-amp section (power amp) ← FX return
                    signal = fx.amp.process_post(fx_return);
                }
            }
            // Apply interstage loading between this slot and the next.
            if i < self.loading.len() {
                signal = self.loading[i].process(signal);
            }
        }
        signal
    }

    fn set_sample_rate(&mut self, rate: f64) {
        for chain_slot in &mut self.chain {
            match chain_slot {
                ChainSlot::Pedal(slot) => {
                    if let Some(ref mut proc) = slot.processor {
                        proc.set_sample_rate(rate);
                    }
                }
                ChainSlot::FxLoop(fx) => {
                    fx.amp.set_sample_rate(rate);
                    for slot in &mut fx.effects {
                        if let Some(ref mut proc) = slot.processor {
                            proc.set_sample_rate(rate);
                        }
                    }
                }
            }
        }
        for loading in &mut self.loading {
            loading.set_sample_rate(rate);
        }
    }

    fn reset(&mut self) {
        for chain_slot in &mut self.chain {
            match chain_slot {
                ChainSlot::Pedal(slot) => {
                    if let Some(ref mut proc) = slot.processor {
                        proc.reset();
                    }
                }
                ChainSlot::FxLoop(fx) => {
                    fx.amp.reset();
                    for slot in &mut fx.effects {
                        if let Some(ref mut proc) = slot.processor {
                            proc.reset();
                        }
                    }
                }
            }
        }
        for loading in &mut self.loading {
            loading.reset();
        }
    }

    /// Control routing: `"idx:label"` dispatches to the appropriate pedal.
    ///
    /// Indices are flat across the board — FX loop pedals are numbered
    /// sequentially after pedals before the loop. Special controls:
    /// - `"idx:__bypass__"` toggles bypass (value > 0.5 = bypassed)
    /// - `"fx_loop:mix"` controls the FX loop wet/dry blend (first loop)
    /// - `"fx_loop_N:mix"` controls the Nth FX loop's mix (0-indexed)
    fn set_control(&mut self, label: &str, value: f64) {
        // FX loop controls: "fx_loop:mix", "fx_loop_N:mix", "fx_loop:Knob", "fx_loop_N:Knob"
        if let Some(rest) = label.strip_prefix("fx_loop") {
            // Determine which FX loop index and which knob label
            let (loop_idx, knob) = if let Some(knob_label) = rest.strip_prefix(':') {
                (0usize, knob_label)
            } else if let Some(idx_and_label) = rest.strip_prefix('_') {
                if let Some((idx_str, knob_label)) = idx_and_label.split_once(':') {
                    if let Ok(idx) = idx_str.parse::<usize>() {
                        (idx, knob_label)
                    } else {
                        return;
                    }
                } else {
                    return;
                }
            } else {
                return;
            };

            let mut cur_loop = 0;
            for slot in &mut self.chain {
                if let ChainSlot::FxLoop(fx) = slot {
                    if cur_loop == loop_idx {
                        if knob == "mix" {
                            fx.mix = value.clamp(0.0, 1.0);
                        } else {
                            // Route to the split amp's controls
                            fx.amp.set_control(knob, value);
                        }
                        return;
                    }
                    cur_loop += 1;
                }
            }
            return;
        }

        // Standard pedal control routing
        if let Some((prefix, remainder)) = label.split_once(':') {
            if let Ok(idx) = prefix.parse::<usize>() {
                let mut flat = self.flat_pedals_mut();
                if let Some(slot) = flat.get_mut(idx) {
                    if remainder == "__bypass__" {
                        slot.bypassed = value > 0.5;
                    } else if let Some(ref mut proc) = slot.processor {
                        proc.set_control(remainder, value);
                    }
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Board Switcher — meta-processor for switching between pre-compiled boards
// ---------------------------------------------------------------------------

/// Meta-processor wrapping multiple pre-compiled pedalboards.
///
/// Delegates `process()` to the active board. Switching is instant —
/// just an index change, no allocation, no audio gap.
pub struct BoardSwitcherProcessor {
    boards: Vec<PedalboardProcessor>,
    active: usize,
}

impl BoardSwitcherProcessor {
    /// Create a new board switcher from a vec of pre-compiled pedalboards.
    pub fn new(boards: Vec<PedalboardProcessor>) -> Self {
        Self { boards, active: 0 }
    }

    /// Number of boards.
    pub fn len(&self) -> usize {
        self.boards.len()
    }

    /// Whether there are no boards.
    pub fn is_empty(&self) -> bool {
        self.boards.is_empty()
    }

    /// Current active board index.
    pub fn active(&self) -> usize {
        self.active
    }
}

impl PedalProcessor for BoardSwitcherProcessor {
    fn process(&mut self, input: f64) -> f64 {
        if let Some(board) = self.boards.get_mut(self.active) {
            board.process(input)
        } else {
            input
        }
    }

    fn set_sample_rate(&mut self, rate: f64) {
        for board in &mut self.boards {
            board.set_sample_rate(rate);
        }
    }

    fn reset(&mut self) {
        for board in &mut self.boards {
            board.reset();
        }
    }

    fn set_control(&mut self, label: &str, value: f64) {
        if label == "__switch_board__" {
            let idx = value as usize;
            if idx < self.boards.len() {
                self.active = idx;
            }
        } else if let Some(board) = self.boards.get_mut(self.active) {
            board.set_control(label, value);
        }
    }
}

// ---------------------------------------------------------------------------
// PedalSwitcherProcessor — switches between individual compiled pedals
// ---------------------------------------------------------------------------

/// Meta-processor wrapping multiple pre-compiled individual pedals.
///
/// Delegates `process()` to the active pedal. Switching is instant —
/// just an index change, no allocation, no audio gap.
/// Unlike `BoardSwitcherProcessor`, forwards bare control labels (no prefix).
pub struct PedalSwitcherProcessor {
    pedals: Vec<CompiledPedal>,
    active: usize,
}

impl PedalSwitcherProcessor {
    /// Create a new pedal switcher from a vec of pre-compiled pedals.
    pub fn new(pedals: Vec<CompiledPedal>) -> Self {
        Self { pedals, active: 0 }
    }

    /// Number of pedals.
    pub fn len(&self) -> usize {
        self.pedals.len()
    }

    /// Whether there are no pedals.
    pub fn is_empty(&self) -> bool {
        self.pedals.is_empty()
    }

    /// Current active pedal index.
    pub fn active(&self) -> usize {
        self.active
    }
}

impl PedalProcessor for PedalSwitcherProcessor {
    fn process(&mut self, input: f64) -> f64 {
        if let Some(pedal) = self.pedals.get_mut(self.active) {
            pedal.process(input)
        } else {
            input
        }
    }

    fn set_sample_rate(&mut self, rate: f64) {
        for pedal in &mut self.pedals {
            pedal.set_sample_rate(rate);
        }
    }

    fn reset(&mut self) {
        for pedal in &mut self.pedals {
            pedal.reset();
        }
    }

    fn set_control(&mut self, label: &str, value: f64) {
        if label == "__switch_pedal__" {
            let idx = value as usize;
            if idx < self.pedals.len() {
                self.active = idx;
            }
        } else if let Some(pedal) = self.pedals.get_mut(self.active) {
            pedal.set_control(label, value);
        }
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::board::parse_board_file;

    #[test]
    fn parse_and_build_pedalboard() {
        let src = r#"
board "Test Board" {
  ts: "tube_screamer.pedal" { Drive = 0.6, Level = 0.7 }
}
"#;
        let board = parse_board_file(src).unwrap();
        let board_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("examples");
        let pb = PedalboardProcessor::from_board(&board, &board_dir, 48000.0).unwrap();
        assert_eq!(pb.len(), 1);
        assert_eq!(pb.pedal_name(0), Some("Tube Screamer"));
        assert!(!pb.is_pedal_bypassed(0));
    }

    #[test]
    fn pedalboard_processes_audio() {
        let src = r#"
board "Chain" {
  ts: "tube_screamer.pedal"
  bd: "blues_driver.pedal"
}
"#;
        let board = parse_board_file(src).unwrap();
        let board_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("examples");
        let mut pb = PedalboardProcessor::from_board(&board, &board_dir, 48000.0).unwrap();

        // Process some samples — should produce non-zero output
        let mut max_out = 0.0_f64;
        for i in 0..1000 {
            let input = (i as f64 * 440.0 * 2.0 * std::f64::consts::PI / 48000.0).sin() * 0.1;
            let output = pb.process(input);
            max_out = max_out.max(output.abs());
        }
        assert!(max_out > 0.0001, "pedalboard should produce output");
    }

    #[test]
    fn pedalboard_bypass_control() {
        let src = r#"
board "Bypass Test" {
  ts: "tube_screamer.pedal"
}
"#;
        let board = parse_board_file(src).unwrap();
        let board_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("examples");
        let mut pb = PedalboardProcessor::from_board(&board, &board_dir, 48000.0).unwrap();

        // Bypass pedal 0
        pb.set_control("0:__bypass__", 1.0);
        assert!(pb.is_pedal_bypassed(0));

        // Bypassed pedal should pass through
        let input = 0.42;
        let output = pb.process(input);
        assert!((output - input).abs() < f64::EPSILON);

        // Un-bypass
        pb.set_control("0:__bypass__", 0.0);
        assert!(!pb.is_pedal_bypassed(0));
    }

    #[test]
    fn pedalboard_knob_control_routing() {
        let src = r#"
board "Knob Test" {
  ts: "tube_screamer.pedal"
}
"#;
        let board = parse_board_file(src).unwrap();
        let board_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("examples");
        let mut pb = PedalboardProcessor::from_board(&board, &board_dir, 48000.0).unwrap();

        // Setting a control via the routing scheme should not panic
        pb.set_control("0:Drive", 0.9);
        pb.set_control("0:Level", 0.3);

        // Invalid index should be a no-op
        pb.set_control("99:Drive", 0.5);
    }

    #[test]
    fn pedalboard_missing_pedal_file_uses_dry_passthrough() {
        let src = r#"
board "Bad" {
  x: "nonexistent.pedal"
}
"#;
        let board = parse_board_file(src).unwrap();
        let board_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("examples");
        let (pb, warnings) =
            PedalboardProcessor::from_board_with_warnings(&board, &board_dir, 48000.0);

        // Board still loads — the failed pedal becomes dry passthrough
        assert_eq!(pb.len(), 1);
        assert!(pb.is_pedal_failed(0));
        assert!(!warnings.is_empty());

        // Dry passthrough: input == output
        let mut pb = pb;
        let output = pb.process(0.42);
        assert!((output - 0.42).abs() < f64::EPSILON);
    }

    // ── BoardSwitcherProcessor tests ─────────────────────────────────────

    fn make_test_board(pedal_file: &str) -> PedalboardProcessor {
        let src = format!(
            r#"board "Test" {{
  p: "{pedal_file}"
}}"#
        );
        let board = parse_board_file(&src).unwrap();
        let board_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("examples");
        PedalboardProcessor::from_board(&board, &board_dir, 48000.0).unwrap()
    }

    #[test]
    fn switcher_delegates_to_active_board() {
        let board0 = make_test_board("tube_screamer.pedal");
        let board1 = make_test_board("blues_driver.pedal");
        let mut switcher = BoardSwitcherProcessor::new(vec![board0, board1]);

        assert_eq!(switcher.len(), 2);
        assert_eq!(switcher.active(), 0);

        // Process through board 0
        let out0 = switcher.process(0.1);

        // Switch to board 1
        switcher.set_control("__switch_board__", 1.0);
        assert_eq!(switcher.active(), 1);

        // Process through board 1 — output should differ from board 0
        let out1 = switcher.process(0.1);
        // Both should produce output (not necessarily different on first sample,
        // but the switcher should not crash)
        assert!(out0.is_finite());
        assert!(out1.is_finite());
    }

    #[test]
    fn switcher_bounds_check() {
        let board0 = make_test_board("tube_screamer.pedal");
        let mut switcher = BoardSwitcherProcessor::new(vec![board0]);

        // Out-of-bounds switch should be ignored
        switcher.set_control("__switch_board__", 99.0);
        assert_eq!(switcher.active(), 0);
    }

    #[test]
    fn switcher_forwards_controls_to_active() {
        let board0 = make_test_board("tube_screamer.pedal");
        let board1 = make_test_board("blues_driver.pedal");
        let mut switcher = BoardSwitcherProcessor::new(vec![board0, board1]);

        // Forward a control to board 0 (active) — should not panic
        switcher.set_control("0:Drive", 0.9);

        // Switch and forward to board 1
        switcher.set_control("__switch_board__", 1.0);
        switcher.set_control("0:Gain", 0.8);
    }

    #[test]
    fn switcher_set_sample_rate_all_boards() {
        let board0 = make_test_board("tube_screamer.pedal");
        let board1 = make_test_board("blues_driver.pedal");
        let mut switcher = BoardSwitcherProcessor::new(vec![board0, board1]);

        // Should not panic — sets rate on all boards
        switcher.set_sample_rate(44100.0);
        switcher.reset();
    }

    // ── Global board tests ──────────────────────────────────────────────

    #[test]
    fn global_board_prepends_and_appends() {
        let examples = Path::new(env!("CARGO_MANIFEST_DIR")).join("examples");
        let ts_path = examples.join("pedals/overdrive/tube_screamer.pedal").canonicalize().unwrap();
        let comp_path = examples.join("pedals/compressor/dyna_comp.pedal").canonicalize().unwrap();
        let bd_path = examples.join("pedals/overdrive/blues_driver.pedal").canonicalize().unwrap();

        let tmp = tempfile::tempdir().unwrap();

        let global_content = format!(
            "global {{\n  start {{\n    comp: \"{}\" {{ Sensitivity = 0.3 }}\n  }}\n  end {{\n    bd: \"{}\"\n  }}\n}}",
            comp_path.display(),
            bd_path.display(),
        );
        std::fs::write(tmp.path().join(".global.board"), &global_content).unwrap();

        let sub = tmp.path().join("boards");
        std::fs::create_dir(&sub).unwrap();
        let board_content = format!(
            "board \"Test\" {{\n  ts: \"{}\" {{ Drive = 0.6 }}\n}}",
            ts_path.display(),
        );
        std::fs::write(sub.join("test.board"), &board_content).unwrap();

        let board = parse_board_file(&board_content).unwrap();
        let (pb, warnings) =
            PedalboardProcessor::from_board_with_warnings(&board, &sub, 48000.0);

        assert_eq!(pb.len(), 3, "warnings: {warnings:?}");
        assert_eq!(pb.pedal_name(0), Some("MXR Dyna Comp"));
        assert_eq!(pb.pedal_name(1), Some("Tube Screamer"));
        assert_eq!(pb.pedal_name(2), Some("Boss Blues Driver"));
    }

    #[test]
    fn global_board_start_only() {
        let examples = Path::new(env!("CARGO_MANIFEST_DIR")).join("examples");
        let ts_path = examples.join("pedals/overdrive/tube_screamer.pedal").canonicalize().unwrap();
        let comp_path = examples.join("pedals/compressor/dyna_comp.pedal").canonicalize().unwrap();

        let tmp = tempfile::tempdir().unwrap();

        let global_content = format!(
            "global {{\n  start {{\n    comp: \"{}\"\n  }}\n}}",
            comp_path.display(),
        );
        std::fs::write(tmp.path().join(".global.board"), &global_content).unwrap();

        let board_content = format!(
            "board \"Test\" {{\n  ts: \"{}\"\n}}",
            ts_path.display(),
        );

        let board = parse_board_file(&board_content).unwrap();
        let (pb, _) =
            PedalboardProcessor::from_board_with_warnings(&board, tmp.path(), 48000.0);

        assert_eq!(pb.len(), 2);
        assert_eq!(pb.pedal_name(0), Some("MXR Dyna Comp"));
        assert_eq!(pb.pedal_name(1), Some("Tube Screamer"));
    }

    #[test]
    fn global_board_signal_chain_order() {
        let examples = Path::new(env!("CARGO_MANIFEST_DIR")).join("examples");
        let ts_path = examples.join("pedals/overdrive/tube_screamer.pedal").canonicalize().unwrap();
        let comp_path = examples.join("pedals/compressor/dyna_comp.pedal").canonicalize().unwrap();
        let bd_path = examples.join("pedals/overdrive/blues_driver.pedal").canonicalize().unwrap();

        let tmp = tempfile::tempdir().unwrap();

        let global_content = format!(
            "global {{\n  start {{\n    comp: \"{}\"\n  }}\n  end {{\n    bd: \"{}\"\n  }}\n}}",
            comp_path.display(),
            bd_path.display(),
        );
        std::fs::write(tmp.path().join(".global.board"), &global_content).unwrap();

        let board_content = format!(
            "board \"Chain\" {{\n  ts: \"{}\" {{ Drive = 0.8 }}\n}}",
            ts_path.display(),
        );

        let board = parse_board_file(&board_content).unwrap();
        let (pb, _) =
            PedalboardProcessor::from_board_with_warnings(&board, tmp.path(), 48000.0);

        assert_eq!(pb.len(), 3);

        let mut pb = pb;
        let mut max_out = 0.0_f64;
        for i in 0..1000 {
            let input = (i as f64 * 440.0 * 2.0 * std::f64::consts::PI / 48000.0).sin() * 0.1;
            let output = pb.process(input);
            assert!(output.is_finite());
            max_out = max_out.max(output.abs());
        }
        assert!(max_out > 0.0001, "pedalboard with global should produce output");
    }

    #[test]
    fn no_global_board_unchanged() {
        let tmp = tempfile::tempdir().unwrap();
        let examples = Path::new(env!("CARGO_MANIFEST_DIR")).join("examples");
        let ts_path = examples.join("pedals/overdrive/tube_screamer.pedal").canonicalize().unwrap();

        let board_content = format!(
            "board \"Solo\" {{\n  ts: \"{}\"\n}}",
            ts_path.display(),
        );

        let board = parse_board_file(&board_content).unwrap();
        let (pb, warnings) =
            PedalboardProcessor::from_board_with_warnings(&board, tmp.path(), 48000.0);

        assert_eq!(pb.len(), 1);
        assert!(warnings.is_empty());
        assert_eq!(pb.pedal_name(0), Some("Tube Screamer"));
    }

    #[test]
    fn global_board_control_routing_flat_index() {
        let examples = Path::new(env!("CARGO_MANIFEST_DIR")).join("examples");
        let ts_path = examples.join("pedals/overdrive/tube_screamer.pedal").canonicalize().unwrap();
        let comp_path = examples.join("pedals/compressor/dyna_comp.pedal").canonicalize().unwrap();

        let tmp = tempfile::tempdir().unwrap();

        let global_content = format!(
            "global {{\n  start {{\n    comp: \"{}\"\n  }}\n}}",
            comp_path.display(),
        );
        std::fs::write(tmp.path().join(".global.board"), &global_content).unwrap();

        let board_content = format!(
            "board \"Test\" {{\n  ts: \"{}\"\n}}",
            ts_path.display(),
        );

        let board = parse_board_file(&board_content).unwrap();
        let (mut pb, _) =
            PedalboardProcessor::from_board_with_warnings(&board, tmp.path(), 48000.0);

        pb.set_control("0:__bypass__", 1.0);
        assert!(pb.is_pedal_bypassed(0));
        assert!(!pb.is_pedal_bypassed(1));

        pb.set_control("1:Drive", 0.9);
    }

    // ── FX loop tests ────────────────────────────────────────────────────

    #[test]
    fn pedalboard_fx_loop_processes_audio() {
        let src = r#"
board "FX Loop" {
  amp: "tweed_deluxe_5e3.pedal" { Volume = 0.7, Tone = 0.5 }
  fx_loop(amp) {
    ts: "tube_screamer.pedal" { Drive = 0.3, Level = 0.5 }
  }
}
"#;
        let board = parse_board_file(src).unwrap();
        let board_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("examples");
        let (mut pb, warnings) =
            PedalboardProcessor::from_board_with_warnings(&board, &board_dir, 48000.0);
        assert!(
            warnings.is_empty(),
            "FX loop board should load without warnings: {warnings:?}"
        );

        // Process some samples — should produce non-zero output
        let mut max_out = 0.0_f64;
        for i in 0..1000 {
            let input = (i as f64 * 440.0 * 2.0 * std::f64::consts::PI / 48000.0).sin() * 0.1;
            let output = pb.process(input);
            assert!(output.is_finite(), "FX loop output should be finite");
            max_out = max_out.max(output.abs());
        }
        assert!(max_out > 0.0001, "FX loop board should produce output");
    }

    #[test]
    fn pedalboard_fx_loop_mix_control() {
        let src = r#"
board "Mix" {
  amp: "tweed_deluxe_5e3.pedal" { Volume = 0.5, Tone = 0.5 }
  fx_loop(amp) {
    ts: "tube_screamer.pedal" { Drive = 0.5 }
  }
}
"#;
        let board = parse_board_file(src).unwrap();
        let board_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("examples");
        let mut pb = PedalboardProcessor::from_board(&board, &board_dir, 48000.0).unwrap();

        // Set FX loop mix — should not panic
        pb.set_control("fx_loop:mix", 0.5);

        // Route a control to the amp inside the FX loop
        pb.set_control("fx_loop:Volume", 0.8);

        // Process should still work
        let output = pb.process(0.1);
        assert!(output.is_finite());
    }

    #[test]
    fn pedalboard_fx_loop_missing_amp_falls_back() {
        let src = r#"
board "Bad FX Loop" {
  ts: "tube_screamer.pedal"
  fx_loop(nonexistent) {
    bd: "blues_driver.pedal"
  }
}
"#;
        let board = parse_board_file(src).unwrap();
        let board_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("examples");
        let (pb, warnings) =
            PedalboardProcessor::from_board_with_warnings(&board, &board_dir, 48000.0);

        // Should have a warning about the missing amp
        assert!(
            warnings.iter().any(|w| w.contains("nonexistent")),
            "Should warn about missing amp: {warnings:?}"
        );
        // The blues_driver should still be in the chain as a fallback pedal
        assert!(pb.len() >= 2, "Fallback should still have pedals");
    }

    #[test]
    fn from_board_with_resolver_builds_chain() {
        let board_src = r#"
board "Resolver Test" {
  ts: "tube_screamer.pedal" { Drive = 0.7 }
  bd: "blues_driver.pedal"
}
"#;
        let ts_src = std::fs::read_to_string(
            Path::new(env!("CARGO_MANIFEST_DIR"))
                .join("examples/pedals/overdrive/tube_screamer.pedal"),
        )
        .unwrap();
        let bd_src = std::fs::read_to_string(
            Path::new(env!("CARGO_MANIFEST_DIR"))
                .join("examples/pedals/overdrive/blues_driver.pedal"),
        )
        .unwrap();

        let sources: Vec<(&str, String)> = vec![
            ("tube_screamer.pedal", ts_src),
            ("blues_driver.pedal", bd_src),
        ];

        let board = parse_board_file(board_src).unwrap();
        let (pb, warnings) = PedalboardProcessor::from_board_with_resolver(
            &board,
            48000.0,
            |path| sources.iter().find(|(p, _)| *p == path).map(|(_, s)| s.clone()),
        );

        assert!(warnings.is_empty(), "Unexpected warnings: {warnings:?}");
        assert_eq!(pb.len(), 2);
        assert_eq!(pb.pedal_name(0), Some("Tube Screamer"));
        assert_eq!(pb.pedal_name(1), Some("Boss Blues Driver"));

        // Verify it processes audio
        let mut pb = pb;
        let input = crate::wav::sine_wave(330.0, 0.02, 48000);
        let output: Vec<f64> = input.iter().map(|&s| pb.process(s)).collect();
        let max_out = output.iter().copied().fold(0.0_f64, |a, b| a.max(b.abs()));
        assert!(max_out > 1e-6, "Board should produce non-silent output");
    }

    #[test]
    fn from_board_with_resolver_missing_pedal_warns() {
        let board_src = r#"
board "Missing Test" {
  ts: "nonexistent.pedal"
}
"#;
        let board = parse_board_file(board_src).unwrap();
        let (pb, warnings) = PedalboardProcessor::from_board_with_resolver(
            &board,
            48000.0,
            |_| None,
        );

        assert_eq!(pb.len(), 1);
        assert!(pb.is_pedal_failed(0));
        assert!(
            warnings.iter().any(|w| w.contains("not found")),
            "Should warn about missing pedal: {warnings:?}"
        );
    }
}
