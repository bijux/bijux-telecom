//! Fixture file loading helpers for deterministic test assets.

use std::fs;
use std::path::Path;

use bijux_gnss_infra::api::{DatasetEntry, DatasetRegistry};
use serde::de::DeserializeOwned;

/// Load a reference TOML file into a typed struct.
pub fn load_reference_config<T: DeserializeOwned>(path: &Path) -> Result<T, String> {
    let contents = fs::read_to_string(path).map_err(|error| error.to_string())?;
    toml::from_str(&contents).map_err(|error| error.to_string())
}

/// Load a dataset entry from a registry file and dataset id.
pub fn load_dataset_entry(path: &Path, id: &str) -> Result<DatasetEntry, String> {
    let contents = fs::read_to_string(path).map_err(|error| error.to_string())?;
    let registry: DatasetRegistry = toml::from_str(&contents).map_err(|error| error.to_string())?;
    registry
        .entries
        .into_iter()
        .find(|entry| entry.id == id)
        .ok_or_else(|| format!("dataset id not found: {id}"))
}

/// Load a JSON golden file into a typed struct.
pub fn load_golden_json<T: DeserializeOwned>(path: &Path) -> Result<T, String> {
    let contents = fs::read_to_string(path).map_err(|error| error.to_string())?;
    serde_json::from_str(&contents).map_err(|error| error.to_string())
}
