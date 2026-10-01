//! Stable control contract for a PedalKernel VST3 and remote DSP host.
//!
//! Synesthesia describes a session through these messages. Isochrone remains
//! the owner of network audio. Plugin loading, CoreAudio, and network I/O live
//! outside this crate so the orchestration boundary survives host rewrites.

use serde::{Deserialize, Serialize};
use thiserror::Error;

pub const PROTOCOL_VERSION: u16 = 1;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SessionSpec {
    pub id: String,
    pub sample_rate_hz: u32,
    pub block_frames: u32,
    pub input_channels: u16,
    pub output_channels: u16,
}
impl SessionSpec {
    pub fn validate(&self) -> Result<(), ContractError> {
        if self.id.trim().is_empty() {
            return Err(ContractError::EmptySessionId);
        }
        if !(8_000..=384_000).contains(&self.sample_rate_hz) {
            return Err(ContractError::InvalidSampleRate(self.sample_rate_hz));
        }
        if self.block_frames == 0 || self.block_frames > 8_192 {
            return Err(ContractError::InvalidBlockSize(self.block_frames));
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PluginRole {
    Instrument,
    Effect,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "command", rename_all = "snake_case", deny_unknown_fields)]
pub enum HostCommand {
    Configure {
        protocol: u16,
        session: SessionSpec,
    },
    LoadVst3 {
        instance: String,
        class_id: String,
        bundle_path: String,
        role: PluginRole,
    },
    SetParameter {
        instance: String,
        parameter_id: u32,
        normalized: f64,
        sample_offset: u32,
    },
    Midi {
        instance: String,
        bytes: Vec<u8>,
        sample_offset: u32,
    },
    SetBypass {
        instance: String,
        bypassed: bool,
    },
    Unload {
        instance: String,
    },
    Stop,
}

impl HostCommand {
    pub fn validate(&self) -> Result<(), ContractError> {
        match self {
            Self::Configure { protocol, session } => {
                if *protocol != PROTOCOL_VERSION {
                    return Err(ContractError::ProtocolVersion(*protocol));
                }
                session.validate()
            }
            Self::SetParameter { normalized, .. }
                if !normalized.is_finite() || !(0.0..=1.0).contains(normalized) =>
            {
                Err(ContractError::InvalidNormalizedParameter(*normalized))
            }
            Self::Midi { bytes, .. } if bytes.is_empty() || bytes.len() > 3 => {
                Err(ContractError::InvalidMidiLength(bytes.len()))
            }
            Self::LoadVst3 {
                instance,
                class_id,
                bundle_path,
                ..
            } if instance.is_empty() || class_id.is_empty() || bundle_path.is_empty() => {
                Err(ContractError::MissingPluginIdentity)
            }
            _ => Ok(()),
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum StreamOwner {
    Isochrone,
    LocalDevice,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AudioEndpoint {
    pub owner: StreamOwner,
    /// Opaque endpoint name resolved by the owning service.
    pub endpoint: String,
    pub channels: Vec<u16>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct HardwareInsert {
    pub id: String,
    pub send: AudioEndpoint,
    pub return_: AudioEndpoint,
    /// Measured end-to-end delay; never inferred from nominal buffer sizes.
    pub round_trip_frames: u32,
    /// Identity of the calibration artifact produced by pedalkernel-fx-loop.
    pub calibration_id: String,
}

#[derive(Debug, Error, PartialEq)]
pub enum ContractError {
    #[error("session id must not be empty")]
    EmptySessionId,
    #[error("unsupported protocol version {0}")]
    ProtocolVersion(u16),
    #[error("invalid sample rate {0}")]
    InvalidSampleRate(u32),
    #[error("invalid block size {0}")]
    InvalidBlockSize(u32),
    #[error("normalized parameter must be finite and in 0..=1, got {0}")]
    InvalidNormalizedParameter(f64),
    #[error("MIDI message must contain 1 to 3 bytes, got {0}")]
    InvalidMidiLength(usize),
    #[error("plugin instance, class id, and bundle path are required")]
    MissingPluginIdentity,
}

/// Fixed-size delay line constructed off the realtime thread. `process` does
/// not allocate and aligns a dry path with a measured hardware/network return.
pub struct DelayLine {
    samples: Vec<f32>,
    cursor: usize,
}

impl DelayLine {
    pub fn new(delay_frames: usize, channels: usize) -> Self {
        Self {
            samples: vec![0.0; delay_frames.saturating_mul(channels)],
            cursor: 0,
        }
    }

    pub fn process(&mut self, sample: f32) -> f32 {
        if self.samples.is_empty() {
            return sample;
        }
        let delayed = self.samples[self.cursor];
        self.samples[self.cursor] = sample;
        self.cursor = (self.cursor + 1) % self.samples.len();
        delayed
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn delay_line_is_exact() {
        let mut delay = DelayLine::new(2, 1);
        let output: Vec<_> = [1.0, 2.0, 3.0, 4.0]
            .into_iter()
            .map(|sample| delay.process(sample))
            .collect();
        assert_eq!(output, [0.0, 0.0, 1.0, 2.0]);
    }

    #[test]
    fn rejects_out_of_range_plugin_parameter() {
        let command = HostCommand::SetParameter {
            instance: "piano".into(),
            parameter_id: 9,
            normalized: 1.1,
            sample_offset: 0,
        };
        assert!(matches!(
            command.validate(),
            Err(ContractError::InvalidNormalizedParameter(_))
        ));
    }

    #[test]
    fn command_schema_round_trips() {
        let command = HostCommand::Midi {
            instance: "piano".into(),
            bytes: vec![0x90, 60, 100],
            sample_offset: 12,
        };
        let json = serde_json::to_string(&command).unwrap();
        assert_eq!(serde_json::from_str::<HostCommand>(&json).unwrap(), command);
    }
}
