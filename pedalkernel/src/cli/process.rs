use hound::{SampleFormat, WavWriter};
use pedalkernel::compiler::compile_pedal;
use pedalkernel::dsl::{parse_pedal_file, PedalDef};
use pedalkernel::PedalProcessor;
use std::path::{Path, PathBuf};
use std::process;

pub struct ProcessOptions<'a> {
    pub pedalhw: Option<&'a str>,
    pub preset: Option<&'a str>,
    pub mods: &'a [String],
    pub no_calibrate: bool,
}

pub fn run(
    file_path: &str,
    input_path: &str,
    output_path: &str,
    options: ProcessOptions<'_>,
    knob_args: &[String],
) {
    if file_path.ends_with(".board") {
        if options.pedalhw.is_some()
            || options.preset.is_some()
            || !options.mods.is_empty()
            || options.no_calibrate
        {
            eprintln!(
                "Error: --pedalhw, --preset, --mod, and --no-calibrate are only supported for .pedal files"
            );
            process::exit(1);
        }
        run_board(file_path, input_path, output_path, knob_args);
    } else {
        run_pedal(file_path, input_path, output_path, options, knob_args);
    }
}

fn run_pedal(
    pedal_path: &str,
    input_path: &str,
    output_path: &str,
    options: ProcessOptions<'_>,
    knob_args: &[String],
) {
    // Parse knob overrides (e.g. "Drive=0.8")
    let overrides: Vec<(&str, f64)> = knob_args
        .iter()
        .filter_map(|a| {
            let (k, v) = a.split_once('=')?;
            Some((k, v.parse::<f64>().ok()?))
        })
        .collect();

    // Parse the .pedal file
    let mut source = std::fs::read_to_string(pedal_path).unwrap_or_else(|e| {
        eprintln!("Error reading {pedal_path}: {e}");
        process::exit(1);
    });
    let pedalhw =
        if options.pedalhw.is_some() || options.preset.is_some() || !options.mods.is_empty() {
            Some(load_pedalhw(pedal_path, options.pedalhw))
        } else {
            None
        };
    if let Some(hw) = &pedalhw {
        for mod_name in options.mods {
            let pedal_mod = hw.find_mod(mod_name).unwrap_or_else(|| {
                eprintln!(
                    "Error: mod '{mod_name}' not found in {}. Available mods: {:?}",
                    hw.path.display(),
                    hw.mods.iter().map(|m| m.name.as_str()).collect::<Vec<_>>()
                );
                process::exit(1);
            });
            source = apply_pedalhw_mod(&source, pedal_mod).unwrap_or_else(|e| {
                eprintln!("Error applying mod '{mod_name}': {e}");
                process::exit(1);
            });
            eprintln!("Applied .pedalhw mod: {}", pedal_mod.name);
        }
    }
    let mut pedal = parse_pedal_file(&source).unwrap_or_else(|e| {
        eprintln!("Parse error: {e}");
        process::exit(1);
    });
    // Flatten any file-based `use("...")` subcircuit instances relative to the
    // .pedal file's directory before compilation.
    if !pedal.uses.is_empty() {
        let base_dir = Path::new(pedal_path)
            .parent()
            .unwrap_or_else(|| Path::new("."));
        pedal = pedalkernel::dsl_expand::expand_uses(&pedal, base_dir).unwrap_or_else(|e| {
            eprintln!("Subcircuit expansion error: {e}");
            process::exit(1);
        });
    }
    if options.no_calibrate && disable_calibrate(&mut pedal) {
        eprintln!("Disabled .pedal calibrate normalization for this render");
    }

    // Read input WAV
    let (input_samples, sample_rate) = pedalkernel::wav::read_wav_mono(Path::new(input_path))
        .unwrap_or_else(|e| {
            eprintln!("Error reading {input_path}: {e}");
            process::exit(1);
        });
    eprintln!(
        "Input: {} ({} samples, {} Hz)",
        input_path,
        input_samples.len(),
        sample_rate,
    );

    // Resolve control values: start from defaults, apply overrides
    let mut knob_values: Vec<(String, f64)> = pedal
        .controls
        .iter()
        .map(|c| (c.label.clone(), c.default))
        .collect();
    if let Some(preset_name) = options.preset {
        let hw = pedalhw.as_ref().unwrap_or_else(|| {
            eprintln!("Error: --preset requires a .pedalhw file");
            process::exit(1);
        });
        let preset = hw.find_preset(preset_name).unwrap_or_else(|| {
            eprintln!(
                "Error: preset '{preset_name}' not found in {}. Available presets: {:?}",
                hw.path.display(),
                hw.presets
                    .iter()
                    .map(|preset| preset.name.as_str())
                    .collect::<Vec<_>>()
            );
            process::exit(1);
        });
        for (idx, value) in preset.knobs.iter().enumerate() {
            if let Some(value) = value {
                if let Some((_, control_value)) = knob_values.get_mut(idx) {
                    *control_value = *value;
                }
            }
        }
        eprintln!("Applied .pedalhw preset: {}", preset.name);
    }
    for (name, val) in &overrides {
        if let Some(kv) = knob_values
            .iter_mut()
            .find(|(l, _)| l.eq_ignore_ascii_case(name))
        {
            kv.1 = *val;
        } else {
            eprintln!(
                "Warning: unknown knob '{name}', available: {:?}",
                knob_values
                    .iter()
                    .map(|(l, _)| l.as_str())
                    .collect::<Vec<_>>()
            );
        }
    }

    // Compile the pedal's netlist into a WDF processor
    let mut proc = compile_pedal(&pedal, sample_rate as f64).unwrap_or_else(|e| {
        eprintln!("Compilation error: {e}");
        process::exit(1);
    });
    for (label, val) in &knob_values {
        proc.set_control(label, *val);
    }
    let output_samples: Vec<f64> = input_samples.iter().map(|&s| proc.process(s)).collect();

    // Write output WAV (same sample rate, mono, 32-bit float)
    write_wav(output_path, sample_rate, &output_samples);

    eprintln!("Pedal:  \"{}\"", pedal.name);
    for (label, val) in &knob_values {
        eprintln!("  {label}: {val:.2}");
    }
    eprintln!("Output: {} ({} samples)", output_path, output_samples.len());
}

fn disable_calibrate(pedal: &mut PedalDef) -> bool {
    let was_enabled = pedal.calibrate;
    pedal.calibrate = false;
    was_enabled
}

#[derive(Debug)]
struct PedalHw {
    path: PathBuf,
    presets: Vec<PedalHwPreset>,
    mods: Vec<PedalHwMod>,
}

#[derive(Debug)]
struct PedalHwPreset {
    name: String,
    knobs: Vec<Option<f64>>,
}

#[derive(Debug)]
struct PedalHwMod {
    name: String,
    changes: Vec<PedalHwChange>,
}

#[derive(Debug)]
struct PedalHwChange {
    component: String,
    value: String,
}

impl PedalHw {
    fn find_preset(&self, name: &str) -> Option<&PedalHwPreset> {
        self.presets
            .iter()
            .find(|preset| preset.name.eq_ignore_ascii_case(name))
    }

    fn find_mod(&self, name: &str) -> Option<&PedalHwMod> {
        self.mods
            .iter()
            .find(|pedal_mod| pedal_mod.name.eq_ignore_ascii_case(name))
    }
}

fn load_pedalhw(pedal_path: &str, explicit_path: Option<&str>) -> PedalHw {
    let path = explicit_path
        .map(PathBuf::from)
        .unwrap_or_else(|| Path::new(pedal_path).with_extension("pedalhw"));
    let source = std::fs::read_to_string(&path).unwrap_or_else(|e| {
        eprintln!("Error reading .pedalhw {}: {e}", path.display());
        process::exit(1);
    });
    parse_pedalhw(&path, &source).unwrap_or_else(|e| {
        eprintln!("Error parsing .pedalhw {}: {e}", path.display());
        process::exit(1);
    })
}

fn parse_pedalhw(path: &Path, source: &str) -> Result<PedalHw, String> {
    enum Section {
        None,
        Preset,
        Mod,
    }

    let mut section = Section::None;
    let mut presets = Vec::new();
    let mut mods = Vec::new();
    let mut current_preset: Option<PedalHwPreset> = None;
    let mut current_mod: Option<PedalHwMod> = None;

    let flush_current = |presets: &mut Vec<PedalHwPreset>,
                         mods: &mut Vec<PedalHwMod>,
                         current_preset: &mut Option<PedalHwPreset>,
                         current_mod: &mut Option<PedalHwMod>| {
        if let Some(preset) = current_preset.take() {
            if !preset.name.is_empty() {
                presets.push(preset);
            }
        }
        if let Some(pedal_mod) = current_mod.take() {
            if !pedal_mod.name.is_empty() {
                mods.push(pedal_mod);
            }
        }
    };

    for raw_line in source.lines() {
        let line = strip_toml_comment(raw_line).trim().to_string();
        if line.is_empty() {
            continue;
        }
        match line.as_str() {
            "[[presets]]" => {
                flush_current(
                    &mut presets,
                    &mut mods,
                    &mut current_preset,
                    &mut current_mod,
                );
                current_preset = Some(PedalHwPreset {
                    name: String::new(),
                    knobs: vec![None; 6],
                });
                section = Section::Preset;
                continue;
            }
            "[[mods]]" => {
                flush_current(
                    &mut presets,
                    &mut mods,
                    &mut current_preset,
                    &mut current_mod,
                );
                current_mod = Some(PedalHwMod {
                    name: String::new(),
                    changes: Vec::new(),
                });
                section = Section::Mod;
                continue;
            }
            _ if line.starts_with('[') => {
                flush_current(
                    &mut presets,
                    &mut mods,
                    &mut current_preset,
                    &mut current_mod,
                );
                section = Section::None;
                continue;
            }
            _ => {}
        }

        if let Some((component, value)) = parse_change_object(&line) {
            if let Some(pedal_mod) = current_mod.as_mut() {
                pedal_mod.changes.push(PedalHwChange { component, value });
            }
            continue;
        }

        let Some((key, value)) = line.split_once('=') else {
            continue;
        };
        let key = key.trim();
        let value = value.trim().trim_end_matches(',');
        match section {
            Section::Preset => {
                if let Some(preset) = current_preset.as_mut() {
                    if key == "name" {
                        preset.name = parse_toml_string(value)
                            .ok_or_else(|| format!("invalid preset name: {value}"))?;
                    } else if let Some(idx) = parse_knob_key(key) {
                        preset.knobs[idx] = Some(
                            value
                                .parse::<f64>()
                                .map_err(|_| format!("invalid {key} value: {value}"))?,
                        );
                    }
                }
            }
            Section::Mod => {
                if key == "name" {
                    if let Some(pedal_mod) = current_mod.as_mut() {
                        pedal_mod.name = parse_toml_string(value)
                            .ok_or_else(|| format!("invalid mod name: {value}"))?;
                    }
                }
            }
            Section::None => {}
        }
    }

    flush_current(
        &mut presets,
        &mut mods,
        &mut current_preset,
        &mut current_mod,
    );

    Ok(PedalHw {
        path: path.to_path_buf(),
        presets,
        mods,
    })
}

fn strip_toml_comment(line: &str) -> String {
    let mut in_quote = false;
    let mut escaped = false;
    for (idx, ch) in line.char_indices() {
        if escaped {
            escaped = false;
            continue;
        }
        match ch {
            '\\' if in_quote => escaped = true,
            '"' => in_quote = !in_quote,
            '#' if !in_quote => return line[..idx].to_string(),
            _ => {}
        }
    }
    line.to_string()
}

fn parse_toml_string(value: &str) -> Option<String> {
    let value = value.trim();
    let value = value.strip_prefix('"')?.strip_suffix('"')?;
    Some(value.replace("\\\"", "\"").replace("\\\\", "\\"))
}

fn parse_knob_key(key: &str) -> Option<usize> {
    let number = key.strip_prefix("knob")?.parse::<usize>().ok()?;
    (1..=6).contains(&number).then_some(number - 1)
}

fn parse_change_object(line: &str) -> Option<(String, String)> {
    if !line.contains("component") || !line.contains("value") {
        return None;
    }
    Some((
        quoted_value_after(line, "component")?,
        quoted_value_after(line, "value")?,
    ))
}

fn quoted_value_after(line: &str, key: &str) -> Option<String> {
    let start = line.find(key)?;
    let after_key = &line[start + key.len()..];
    let eq = after_key.find('=')?;
    parse_toml_string_prefix(after_key[eq + 1..].trim())
}

fn parse_toml_string_prefix(value: &str) -> Option<String> {
    let value = value.strip_prefix('"')?;
    let mut escaped = false;
    let mut out = String::new();
    for ch in value.chars() {
        if escaped {
            match ch {
                '"' | '\\' => out.push(ch),
                _ => {
                    out.push('\\');
                    out.push(ch);
                }
            }
            escaped = false;
            continue;
        }
        match ch {
            '\\' => escaped = true,
            '"' => return Some(out),
            _ => out.push(ch),
        }
    }
    None
}

fn apply_pedalhw_mod(source: &str, pedal_mod: &PedalHwMod) -> Result<String, String> {
    let mut updated = source.to_string();
    for change in &pedal_mod.changes {
        updated = apply_component_change(&updated, change)?;
    }
    Ok(updated)
}

fn apply_component_change(source: &str, change: &PedalHwChange) -> Result<String, String> {
    let mut changed = false;
    let mut output = String::with_capacity(source.len());
    for line in source.lines() {
        if !changed && component_line_matches(line, &change.component) {
            output.push_str(
                &rewrite_component_value(line, &change.value)
                    .ok_or_else(|| format!("could not rewrite component {}", change.component))?,
            );
            changed = true;
        } else {
            output.push_str(line);
        }
        output.push('\n');
    }
    if changed {
        Ok(output)
    } else {
        Err(format!("component '{}' not found", change.component))
    }
}

fn component_line_matches(line: &str, component: &str) -> bool {
    let trimmed = line.trim_start();
    trimmed
        .strip_prefix(component)
        .is_some_and(|rest| rest.trim_start().starts_with(':'))
}

fn rewrite_component_value(line: &str, value: &str) -> Option<String> {
    let open = line.find('(')?;
    let close = line[open + 1..].find(')')? + open + 1;
    let mut out = String::with_capacity(line.len() + value.len());
    out.push_str(&line[..open + 1]);
    out.push_str(value);
    out.push_str(&line[close..]);
    Some(out)
}

fn run_board(board_path: &str, input_path: &str, output_path: &str, knob_args: &[String]) {
    use pedalkernel::board::parse_board_file;
    use pedalkernel::pedalboard::PedalboardProcessor;

    // Parse knob overrides for boards: "pedal_id.Knob=value"
    let overrides: Vec<(&str, &str, f64)> = knob_args
        .iter()
        .filter_map(|a| {
            let (lhs, v) = a.split_once('=')?;
            let val = v.parse::<f64>().ok()?;
            if let Some((pedal_id, knob)) = lhs.split_once('.') {
                Some((pedal_id, knob, val))
            } else {
                eprintln!(
                    "Warning: board knob override must use 'pedal_id.Knob=value' format, got: {a}"
                );
                None
            }
        })
        .collect();

    // Parse the .board file
    let source = std::fs::read_to_string(board_path).unwrap_or_else(|e| {
        eprintln!("Error reading {board_path}: {e}");
        process::exit(1);
    });
    let board = parse_board_file(&source).unwrap_or_else(|e| {
        eprintln!("Parse error: {e}");
        process::exit(1);
    });

    let board_dir = Path::new(board_path)
        .parent()
        .unwrap_or_else(|| Path::new("."));

    // Read input WAV
    let (input_samples, sample_rate) = pedalkernel::wav::read_wav_mono(Path::new(input_path))
        .unwrap_or_else(|e| {
            eprintln!("Error reading {input_path}: {e}");
            process::exit(1);
        });
    eprintln!(
        "Input: {} ({} samples, {} Hz)",
        input_path,
        input_samples.len(),
        sample_rate,
    );

    // Build pedalboard processor
    let mut proc = PedalboardProcessor::from_board(&board, board_dir, sample_rate as f64)
        .unwrap_or_else(|e| {
            eprintln!("Board compilation error: {e}");
            process::exit(1);
        });

    // Apply CLI knob overrides using the "idx:label" control routing
    for (pedal_id, knob, val) in &overrides {
        if let Some(idx) = board.pedals().iter().position(|p| p.id == *pedal_id) {
            proc.set_control(&format!("{idx}:{knob}"), *val);
        } else {
            eprintln!(
                "Warning: unknown pedal id '{pedal_id}', available: {:?}",
                board
                    .pedals()
                    .iter()
                    .map(|p| p.id.as_str())
                    .collect::<Vec<_>>()
            );
        }
    }

    let output_samples: Vec<f64> = input_samples.iter().map(|&s| proc.process(s)).collect();

    // Write output WAV
    write_wav(output_path, sample_rate, &output_samples);

    eprintln!("Board:  \"{}\"", board.name);
    for entry in &board.pedals() {
        eprintln!("  [{}] {}", entry.id, entry.path);
    }
    eprintln!("Output: {} ({} samples)", output_path, output_samples.len());
}

fn write_wav(output_path: &str, sample_rate: u32, samples: &[f64]) {
    let out_spec = hound::WavSpec {
        channels: 1,
        sample_rate,
        bits_per_sample: 32,
        sample_format: SampleFormat::Float,
    };
    let mut writer = WavWriter::create(Path::new(output_path), out_spec).unwrap_or_else(|e| {
        eprintln!("Error creating {output_path}: {e}");
        process::exit(1);
    });
    for &s in samples {
        writer.write_sample(s as f32).unwrap();
    }
    writer.finalize().unwrap();
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_pedalhw_presets_and_inline_mod_changes() {
        let source = r#"
            [[presets]]
            name = "Classic TS"
            knob1 = 0.5
            knob2 = 0.5
            knob3 = 0.5

            [[mods]]
            name = "TS9 Conversion"
            changes = [
                { component = "R_out_s", value = "470" },
                { component = "R_out_g", value = "100k" },
            ]
        "#;

        let hw = parse_pedalhw(Path::new("screamer.pedalhw"), source).unwrap();

        let preset = hw.find_preset("classic ts").unwrap();
        assert_eq!(preset.knobs[0], Some(0.5));
        assert_eq!(preset.knobs[1], Some(0.5));
        assert_eq!(preset.knobs[2], Some(0.5));

        let pedal_mod = hw.find_mod("ts9 conversion").unwrap();
        assert_eq!(pedal_mod.changes.len(), 2);
        assert_eq!(pedal_mod.changes[0].component, "R_out_s");
        assert_eq!(pedal_mod.changes[0].value, "470");
        assert_eq!(pedal_mod.changes[1].component, "R_out_g");
        assert_eq!(pedal_mod.changes[1].value, "100k");
    }

    #[test]
    fn applies_pedalhw_component_changes_without_touching_other_lines() {
        let source = r#"
  components {
    R_out_s: resistor(100)      # Series output resistor
    R_out_g: resistor(10k)      # Shunt to ground
  }
        "#;
        let pedal_mod = PedalHwMod {
            name: "TS9 Conversion".to_string(),
            changes: vec![
                PedalHwChange {
                    component: "R_out_s".to_string(),
                    value: "470".to_string(),
                },
                PedalHwChange {
                    component: "R_out_g".to_string(),
                    value: "100k".to_string(),
                },
            ],
        };

        let updated = apply_pedalhw_mod(source, &pedal_mod).unwrap();

        assert!(updated.contains("R_out_s: resistor(470)      # Series output resistor"));
        assert!(updated.contains("R_out_g: resistor(100k)      # Shunt to ground"));
    }

    #[test]
    fn no_calibrate_override_disables_parsed_calibrate_keyword() {
        let source = r#"
            pedal "Test" {
                components {
                    R1: resistor(10k)
                }
                nets {
                    in -> R1.a
                    R1.b -> out
                }
                calibrate
            }
        "#;

        let mut pedal = parse_pedal_file(source).unwrap();
        assert!(pedal.calibrate);

        assert!(disable_calibrate(&mut pedal));
        assert!(!pedal.calibrate);
        assert!(!disable_calibrate(&mut pedal));
    }
}
