//! Dataset registry parsing and validation.

use bijux_gnss_receiver::api::core::InputError;
use serde::{Deserialize, Serialize};
use std::fs;
use std::path::Path;

/// Dataset registry entry.
#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct DatasetEntry {
    /// Dataset id.
    pub id: String,
    /// Dataset path.
    pub path: String,
    /// Data format identifier.
    pub format: String,
    /// Sample rate in Hz.
    pub sample_rate_hz: f64,
    /// Intermediate frequency in Hz.
    pub intermediate_freq_hz: f64,
    /// Expected satellites.
    pub expected_sats: Vec<u8>,
    /// Expected region.
    pub expected_region: Option<String>,
    /// Expected time (UTC).
    pub expected_time_utc: Option<String>,
    /// Sidecar metadata path.
    pub sidecar: Option<String>,
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
        let registry: DatasetRegistry = toml::from_str(&contents).map_err(map_err)?;
        if registry.entries.is_empty() {
            return Err(InputError { message: "dataset registry is empty".to_string() });
        }
        Ok(registry)
    }

    /// Find entry by id.
    pub fn find(&self, id: &str) -> Option<DatasetEntry> {
        self.entries.iter().find(|e| e.id == id).cloned()
    }
}

fn map_err(err: impl std::fmt::Display) -> InputError {
    InputError { message: err.to_string() }
}
