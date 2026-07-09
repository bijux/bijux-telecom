//! Raw IQ metadata contracts shared by ingest surfaces.

use serde::{Deserialize, Serialize};

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
