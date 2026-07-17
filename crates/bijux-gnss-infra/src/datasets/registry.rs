//! Dataset registry parsing and path normalization.

use bijux_gnss_receiver::api::core::InputError;
use serde::Deserialize;
use std::path::Path;

mod entries;
mod loading;
mod path_resolution;

pub type DatasetEntry = entries::DatasetEntry;
pub type RecordedCaptureProvenance = entries::RecordedCaptureProvenance;

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
        loading::load_registry(path)
    }

    /// Find entry by id.
    pub fn find(&self, id: &str) -> Option<DatasetEntry> {
        self.entries.iter().find(|entry| entry.id == id).cloned()
    }
}

pub(super) fn map_err(err: impl std::fmt::Display) -> InputError {
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
