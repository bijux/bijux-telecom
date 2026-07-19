use std::fs;
use std::path::Path;

use bijux_gnss_receiver::api::core::InputError;
use bijux_gnss_signal::api::RawIqMetadata;

pub(super) fn load_raw_iq_metadata(path: &Path) -> Result<RawIqMetadata, InputError> {
    let contents = fs::read_to_string(path).map_err(super::map_err)?;
    let value: toml::Value = toml::from_str(&contents).map_err(super::map_err)?;
    validate_required_raw_iq_fields(&value)?;
    let metadata: RawIqMetadata = value.try_into().map_err(super::map_err)?;
    super::validate_raw_iq_metadata(&metadata)?;
    Ok(metadata)
}

fn validate_required_raw_iq_fields(value: &toml::Value) -> Result<(), InputError> {
    let table = value.as_table().ok_or_else(|| InputError {
        message: "raw IQ metadata must be a TOML table".to_string(),
    })?;

    for field in ["format", "sample_rate_hz", "intermediate_freq_hz", "capture_start_utc"] {
        if !table.contains_key(field) {
            return Err(InputError { message: format!("raw IQ metadata must declare {field}") });
        }
    }

    Ok(())
}
