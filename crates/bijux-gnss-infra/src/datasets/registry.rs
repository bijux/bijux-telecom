//! Dataset registry parsing and path normalization.

use bijux_gnss_receiver::api::core::InputError;
use bijux_gnss_signal::api::{IqSampleFormat, RawIqMetadata};
use serde::{Deserialize, Serialize};
use std::fs;
use std::path::Path;

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

/// Dataset registry file.
#[derive(Debug, Deserialize)]
pub struct DatasetRegistry {
    /// Registry schema version.
    pub version: u32,
    /// Dataset entries.
    pub entries: Vec<DatasetEntry>,
}

impl DatasetRegistry {
    /// Load registry from disk.
    pub fn load(path: &Path) -> Result<Self, InputError> {
        let contents = fs::read_to_string(path).map_err(map_err)?;
        let mut registry: DatasetRegistry = toml::from_str(&contents).map_err(map_err)?;
        if registry.entries.is_empty() {
            return Err(InputError { message: "dataset registry is empty".to_string() });
        }
        let base_dir = path.parent().unwrap_or_else(|| Path::new("."));
        for entry in &mut registry.entries {
            entry.path = resolve_registry_path(base_dir, &entry.path);
            entry.sidecar =
                entry.sidecar.as_ref().map(|value| resolve_registry_path(base_dir, value));
            if let Some(recorded_capture) = &mut entry.recorded_capture {
                recorded_capture.recommended_config = recorded_capture
                    .recommended_config
                    .as_ref()
                    .map(|value| resolve_registry_path(base_dir, value));
            }
        }
        Ok(registry)
    }

    /// Find entry by id.
    pub fn find(&self, id: &str) -> Option<DatasetEntry> {
        self.entries.iter().find(|entry| entry.id == id).cloned()
    }
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

fn resolve_registry_path(base_dir: &Path, value: &str) -> String {
    let path = Path::new(value);
    if path.is_absolute() {
        return value.to_string();
    }
    let anchor = match (base_dir.file_name(), base_dir.parent(), path.components().next()) {
        (Some(dir_name), Some(parent), Some(first_component))
            if first_component.as_os_str() == dir_name =>
        {
            parent
        }
        _ => base_dir,
    };
    anchor.join(path).to_string_lossy().into_owned()
}

fn map_err(err: impl std::fmt::Display) -> InputError {
    InputError { message: err.to_string() }
}

#[cfg(test)]
mod tests {
    use super::DatasetRegistry;
    use std::fs;

    #[test]
    fn dataset_registry_load_normalizes_relative_paths() {
        let temp = tempfile::tempdir().expect("tempdir");
        let root = temp.path();
        fs::create_dir_all(root.join("datasets/demo")).expect("create dataset dir");
        let registry_path = root.join("registry.toml");
        fs::write(
            &registry_path,
            r#"
version = 1

[[entries]]
id = "demo"
path = "datasets/demo/demo.iq16"
format = "iq16_le"
sample_rate_hz = 5000000.0
intermediate_freq_hz = 0.0
capture_start_utc = "2026-07-09T00:00:00Z"
expected_sats = [1]
expected_region = "N/A"
expected_time_utc = "N/A"
sidecar = "datasets/demo/demo.sidecar.toml"
"#,
        )
        .expect("write registry");

        let registry = DatasetRegistry::load(&registry_path).expect("load registry");
        let entry = registry.find("demo").expect("find dataset");

        assert!(entry.path.starts_with(root.to_string_lossy().as_ref()));
        assert!(entry
            .sidecar
            .as_deref()
            .expect("sidecar")
            .starts_with(root.to_string_lossy().as_ref()));
    }

    #[test]
    fn dataset_registry_load_preserves_recorded_capture_provenance() {
        let temp = tempfile::tempdir().expect("tempdir");
        let root = temp.path();
        fs::create_dir_all(root.join("datasets/recorded")).expect("create dataset dir");
        fs::create_dir_all(root.join("configs")).expect("create configs dir");
        let registry_path = root.join("registry.toml");
        fs::write(
            &registry_path,
            r#"
version = 1

[[entries]]
id = "gps_l1_live_sky"
path = "datasets/recorded/live_sky.iq16"
format = "iq16_le"
sample_rate_hz = 4000000.0
intermediate_freq_hz = 0.0
capture_start_utc = "2022-03-27T11:32:04.2147593125Z"
expected_sats = [31]
expected_region = "N/A"
expected_time_utc = "2022-03-27T11:32:04Z"
sidecar = "datasets/recorded/live_sky.sidecar.toml"

[entries.recorded_capture]
source_url = "https://doi.org/10.5281/zenodo.6394603"
license = "CC-BY-4.0"
attribution = "Daniel Estévez"
hardware = "USRP B205mini"
center_frequency_hz = 1575420000
recommended_config = "configs/receiver_live_sky_gps_l1.toml"
"#,
        )
        .expect("write registry");

        let registry = DatasetRegistry::load(&registry_path).expect("load registry");
        let entry = registry.find("gps_l1_live_sky").expect("find dataset");
        let recorded_capture =
            entry.recorded_capture.as_ref().expect("recorded capture provenance");

        assert_eq!(recorded_capture.source_url, "https://doi.org/10.5281/zenodo.6394603");
        assert_eq!(recorded_capture.license, "CC-BY-4.0");
        assert_eq!(recorded_capture.attribution, "Daniel Estévez");
        assert_eq!(recorded_capture.hardware.as_deref(), Some("USRP B205mini"));
        assert_eq!(recorded_capture.center_frequency_hz, Some(1_575_420_000));
        assert!(recorded_capture
            .recommended_config
            .as_deref()
            .expect("recommended config")
            .starts_with(root.to_string_lossy().as_ref()));
    }
}
