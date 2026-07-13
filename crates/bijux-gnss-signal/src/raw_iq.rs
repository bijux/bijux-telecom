//! Raw IQ metadata contracts shared by ingest surfaces.

use serde::{Deserialize, Serialize};
use std::fmt;

/// Supported raw IQ sample encodings.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum IqSampleFormat {
    /// Signed 8-bit interleaved I/Q pairs.
    Iq8,
    /// Signed 16-bit little-endian interleaved I/Q pairs.
    Iq16Le,
    /// Complex float32 little-endian interleaved I/Q pairs.
    Cf32Le,
}

impl IqSampleFormat {
    /// Number of encoded bytes for one complex I/Q sample.
    pub const fn bytes_per_complex_sample(self) -> usize {
        match self {
            Self::Iq8 => 2,
            Self::Iq16Le => 4,
            Self::Cf32Le => 8,
        }
    }
}

/// Controlled quantization profile applied before raw-IQ storage.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum IqQuantization {
    /// Preserve scaled complex samples in float32 without additional quantization.
    Float32,
    /// Apply a bipolar sign quantizer and store the result in signed 8-bit IQ.
    Bipolar1Bit,
    /// Apply a uniform 2-bit quantizer and store the result in signed 8-bit IQ.
    Signed2Bit,
    /// Apply a uniform 4-bit quantizer and store the result in signed 8-bit IQ.
    Signed4Bit,
    /// Apply a uniform 8-bit quantizer and store the result in signed 8-bit IQ.
    Signed8Bit,
    /// Apply a uniform 16-bit quantizer and store the result in signed 16-bit IQ.
    Signed16Bit,
}

impl IqQuantization {
    /// Raw IQ container format used to store this quantization profile.
    pub const fn sample_format(self) -> IqSampleFormat {
        match self {
            Self::Float32 => IqSampleFormat::Cf32Le,
            Self::Bipolar1Bit | Self::Signed2Bit | Self::Signed4Bit | Self::Signed8Bit => {
                IqSampleFormat::Iq8
            }
            Self::Signed16Bit => IqSampleFormat::Iq16Le,
        }
    }

    /// Effective quantization depth used to model this profile.
    pub const fn quantization_bits(self) -> u8 {
        match self {
            Self::Float32 => 32,
            Self::Bipolar1Bit => 1,
            Self::Signed2Bit => 2,
            Self::Signed4Bit => 4,
            Self::Signed8Bit => 8,
            Self::Signed16Bit => 16,
        }
    }

    /// Stable identifier used in reports, CLI arguments, and generated artifacts.
    pub const fn identifier(self) -> &'static str {
        match self {
            Self::Float32 => "float32",
            Self::Bipolar1Bit => "bipolar_1bit",
            Self::Signed2Bit => "signed_2bit",
            Self::Signed4Bit => "signed_4bit",
            Self::Signed8Bit => "signed_8bit",
            Self::Signed16Bit => "signed_16bit",
        }
    }
}

impl fmt::Display for IqQuantization {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.identifier())
    }
}

/// Explicit metadata required to ingest a raw IQ capture.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RawIqMetadata {
    /// Encoded sample format on disk.
    pub format: IqSampleFormat,
    /// Sample rate in Hz.
    pub sample_rate_hz: f64,
    /// Intermediate frequency in Hz. Use `0.0` for zero-IF captures.
    pub intermediate_freq_hz: f64,
    /// Capture start timestamp in UTC.
    pub capture_start_utc: String,
    /// Optional byte offset before the first I/Q pair.
    #[serde(default)]
    pub offset_bytes: u64,
    /// Optional quantization depth descriptor.
    #[serde(default)]
    pub quantization_bits: Option<u8>,
    /// Optional free-form notes.
    #[serde(default)]
    pub notes: Option<String>,
}
