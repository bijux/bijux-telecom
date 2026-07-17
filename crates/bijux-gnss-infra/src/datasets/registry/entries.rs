use bijux_gnss_signal::api::{IqSampleFormat, RawIqMetadata};
use serde::{Deserialize, Serialize};

/// Recorded capture provenance for a registered dataset.
#[derive(Debug, Serialize, Deserialize, Clone, PartialEq, Eq)]
pub struct RecordedCaptureProvenance {
    /// Canonical source URL for the original recording.
    pub source_url: String,
    /// License string or URL that governs redistribution.
    pub license: String,
    /// Required credit line for downstream users.
    pub attribution: String,
    /// Front-end hardware used for the original capture.
    pub hardware: Option<String>,
    /// RF center frequency in Hz when it is known exactly.
    pub center_frequency_hz: Option<u64>,
    /// Optional repository-local receiver profile tuned for this capture.
    pub recommended_config: Option<String>,
}

/// Dataset registry entry.
#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct DatasetEntry {
    /// Dataset id.
    pub id: String,
    /// Dataset path.
    pub path: String,
    /// Data format identifier.
    pub format: IqSampleFormat,
    /// Sample rate in Hz when declared directly in the registry.
    pub sample_rate_hz: Option<f64>,
    /// Intermediate frequency in Hz when declared directly in the registry.
    pub intermediate_freq_hz: Option<f64>,
    /// Capture start timestamp in UTC.
    pub capture_start_utc: Option<String>,
    /// Expected satellites.
    pub expected_sats: Vec<u8>,
    /// Expected region.
    pub expected_region: Option<String>,
    /// Expected time (UTC).
    pub expected_time_utc: Option<String>,
    /// Sidecar metadata path.
    pub sidecar: Option<String>,
    /// Recorded capture provenance when the dataset comes from a real RF recording.
    #[serde(default)]
    pub recorded_capture: Option<RecordedCaptureProvenance>,
}

impl DatasetEntry {
    /// Build explicit raw IQ metadata from a registry entry when it carries all required fields.
    pub fn raw_iq_metadata(&self) -> Option<RawIqMetadata> {
        let sample_rate_hz = self.sample_rate_hz?;
        let intermediate_freq_hz = self.intermediate_freq_hz?;
        let capture_start_utc = self.capture_start_utc.clone()?;
        Some(RawIqMetadata {
            format: self.format,
            sample_rate_hz,
            intermediate_freq_hz,
            capture_start_utc,
            offset_bytes: 0,
            quantization_bits: None,
            notes: None,
        })
    }
}
