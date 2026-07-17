use std::fs;
use std::path::Path;

use bijux_gnss_receiver::api::core::InputError;

use super::DatasetRegistry;

pub(super) fn load_registry(path: &Path) -> Result<DatasetRegistry, InputError> {
    let contents = fs::read_to_string(path).map_err(super::map_err)?;
    let mut registry: DatasetRegistry = toml::from_str(&contents).map_err(super::map_err)?;
    if registry.entries.is_empty() {
        return Err(InputError { message: "dataset registry is empty".to_string() });
    }

    let base_dir = path.parent().unwrap_or_else(|| Path::new("."));
    for entry in &mut registry.entries {
        entry.path = super::path_resolution::resolve_registry_path(base_dir, &entry.path);
        entry.sidecar = entry
            .sidecar
            .as_ref()
            .map(|value| super::path_resolution::resolve_registry_path(base_dir, value));
        if let Some(recorded_capture) = &mut entry.recorded_capture {
            recorded_capture.recommended_config = recorded_capture
                .recommended_config
                .as_ref()
                .map(|value| super::path_resolution::resolve_registry_path(base_dir, value));
        }
    }

    Ok(registry)
}
