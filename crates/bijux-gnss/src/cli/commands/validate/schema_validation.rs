#[cfg(feature = "schema-validate")]
use jsonschema::validator_for;

use super::*;

#[derive(Copy, Clone)]
pub(crate) enum CsvType {
    U64,
    U8,
    F64,
    Bool,
    Str,
}

pub(crate) fn validate_csv_schema(path: &Path, expected: &str, types: &[CsvType]) -> Result<()> {
    let data = fs::read_to_string(path)?;
    let mut lines = data.lines();
    let header = lines.next().unwrap_or_default();
    if header.trim() != expected {
        bail!("csv schema mismatch in {}: expected {}, got {}", path.display(), expected, header);
    }
    for (idx, line) in lines.enumerate() {
        if line.trim().is_empty() {
            continue;
        }
        let cols: Vec<_> = line.split(',').collect();
        if cols.len() != types.len() {
            bail!(
                "csv schema mismatch in {} line {}: expected {} columns, got {}",
                path.display(),
                idx + 1,
                types.len(),
                cols.len()
            );
        }
        for (value, kind) in cols.iter().zip(types.iter()) {
            match kind {
                CsvType::U64 => {
                    value.parse::<u64>().map_err(|_| {
                        eyre!("csv parse error in {} line {}", path.display(), idx + 1)
                    })?;
                }
                CsvType::U8 => {
                    value.parse::<u8>().map_err(|_| {
                        eyre!("csv parse error in {} line {}", path.display(), idx + 1)
                    })?;
                }
                CsvType::F64 => {
                    value.parse::<f64>().map_err(|_| {
                        eyre!("csv parse error in {} line {}", path.display(), idx + 1)
                    })?;
                }
                CsvType::Bool => {
                    value.parse::<bool>().map_err(|_| {
                        eyre!("csv parse error in {} line {}", path.display(), idx + 1)
                    })?;
                }
                CsvType::Str => {
                    if value.trim().is_empty() {
                        bail!(
                            "csv parse error in {} line {}: empty string",
                            path.display(),
                            idx + 1
                        );
                    }
                }
            }
        }
    }
    Ok(())
}

pub(crate) fn validate_json_schema(
    schema_path: &Path,
    data_path: &Path,
    strict: bool,
) -> Result<()> {
    #[cfg(feature = "schema-validate")]
    {
        let schema_data = fs::read_to_string(schema_path)?;
        let schema_json: serde_json::Value = serde_json::from_str(&schema_data)?;
        let compiled = validator_for(&schema_json)
            .map_err(|error| eyre!("invalid schema {}: {}", schema_path.display(), error))?;
        let data = fs::read_to_string(data_path)?;
        let json: serde_json::Value = serde_json::from_str(&data)?;
        if strict {
            match &json {
                serde_json::Value::Array(items) if items.is_empty() => {
                    bail!("{} is empty", data_path.display());
                }
                _ => {}
            }
        }
        if !compiled.is_valid(&json) {
            let mut messages = Vec::new();
            for error in compiled.iter_errors(&json) {
                messages.push(error.to_string());
            }
            bail!("schema validation failed for {}: {}", data_path.display(), messages.join(", "));
        }
        Ok(())
    }
    #[cfg(not(feature = "schema-validate"))]
    {
        let _ = schema_path;
        if strict {
            bail!(
                "schema validation disabled; enable --features schema-validate to validate {}",
                data_path.display()
            );
        }
        Ok(())
    }
}

pub(crate) fn validate_jsonl_schema(
    schema_path: &Path,
    data_path: &Path,
    strict: bool,
) -> Result<()> {
    #[cfg(feature = "schema-validate")]
    {
        let schema_data = fs::read_to_string(schema_path)?;
        let schema_json: serde_json::Value = serde_json::from_str(&schema_data)?;
        let compiled = validator_for(&schema_json)
            .map_err(|error| eyre!("invalid schema {}: {}", schema_path.display(), error))?;
        let data = fs::read_to_string(data_path)?;
        let mut count = 0usize;
        for (idx, line) in data.lines().enumerate() {
            if line.trim().is_empty() {
                continue;
            }
            let json: serde_json::Value = serde_json::from_str(line)?;
            count += 1;
            let errors: Vec<String> =
                compiled.iter_errors(&json).map(|error| error.to_string()).collect();
            if !errors.is_empty() {
                bail!(
                    "schema validation failed for {} line {}: {}",
                    data_path.display(),
                    idx + 1,
                    errors.join(", ")
                );
            }
        }
        if strict && count == 0 {
            bail!("{} is empty", data_path.display());
        }
        Ok(())
    }
    #[cfg(not(feature = "schema-validate"))]
    {
        let _ = schema_path;
        if strict {
            bail!(
                "schema validation disabled; enable --features schema-validate to validate {}",
                data_path.display()
            );
        }
        Ok(())
    }
}

pub(crate) fn validate_config_schema(profile: &ReceiverConfig) -> Result<()> {
    #[cfg(feature = "schema-validate")]
    {
        let schema_path = schema_path("receiver_profile.schema.json");
        let schema_json: serde_json::Value = if schema_path.exists() {
            let file_json: serde_json::Value =
                serde_json::from_str(&fs::read_to_string(&schema_path)?)?;
            if file_json.get("properties").is_some() {
                file_json
            } else {
                serde_json::to_value(schemars::schema_for!(ReceiverConfig))?
            }
        } else {
            serde_json::to_value(schemars::schema_for!(ReceiverConfig))?
        };
        let compiled =
            validator_for(&schema_json).map_err(|error| eyre!("invalid schema: {}", error))?;
        let json = serde_json::to_value(profile)?;
        if !compiled.is_valid(&json) {
            let mut messages = Vec::new();
            for error in compiled.iter_errors(&json) {
                messages.push(error.to_string());
            }
            bail!("config schema validation failed: {}", messages.join(", "));
        }
        Ok(())
    }
    #[cfg(not(feature = "schema-validate"))]
    {
        let _ = profile;
        Ok(())
    }
}

pub(crate) fn validate_sidecar_schema(sidecar: &RawIqMetadata) -> Result<()> {
    #[cfg(feature = "schema-validate")]
    {
        let schema_path = schema_path("sidecar.schema.json");
        let schema_data = fs::read_to_string(schema_path)?;
        let schema_json: serde_json::Value = serde_json::from_str(&schema_data)?;
        let compiled = validator_for(&schema_json)
            .map_err(|error| eyre!("invalid sidecar schema: {}", error))?;
        let json = serde_json::to_value(sidecar)?;
        if !compiled.is_valid(&json) {
            let mut messages = Vec::new();
            for error in compiled.iter_errors(&json) {
                messages.push(error.to_string());
            }
            bail!("sidecar schema validation failed: {}", messages.join(", "));
        }
        Ok(())
    }
    #[cfg(not(feature = "schema-validate"))]
    {
        let _ = sidecar;
        Ok(())
    }
}

#[cfg(all(test, feature = "schema-validate"))]
mod tests {
    use super::*;

    fn artifact_header_fixture() -> serde_json::Value {
        serde_json::json!({
            "schema_version": 1,
            "producer": "bijux-gnss",
            "producer_version": "0.1.0",
            "created_at_unix_ms": 0,
            "git_sha": "test",
            "config_hash": "test",
            "dataset_id": null,
            "toolchain": "test",
            "features": [],
            "deterministic": true,
            "git_dirty": false
        })
    }

    fn assert_schema_accepts(schema_name: &str, value: &serde_json::Value) {
        let schema_data =
            fs::read_to_string(schema_path(schema_name)).expect("schema should be readable");
        let schema: serde_json::Value =
            serde_json::from_str(&schema_data).expect("schema should contain valid JSON");
        let validator = validator_for(&schema).expect("schema should compile");
        let errors =
            validator.iter_errors(value).map(|error| error.to_string()).collect::<Vec<_>>();
        assert!(errors.is_empty(), "{schema_name} rejected fixture: {}", errors.join(", "));
    }

    #[test]
    fn acquisition_schema_accepts_absent_refinement_evidence() {
        let value = serde_json::json!({
            "header": artifact_header_fixture(),
            "payload": {
                "sat": {"constellation": "gps", "prn": 1},
                "signal_band": "l1",
                "source_time": {
                    "sample_index": 0,
                    "sample_rate_hz": 4_092_000.0,
                    "receiver_time_s": 0.0
                },
                "doppler_hz": 0.0,
                "carrier_hz": 0.0,
                "doppler_refinement": null,
                "code_phase_refinement": null,
                "uncertainty": null,
                "code_phase_samples": 0,
                "peak": 1.0,
                "peak_second_ratio": 1.0,
                "hypothesis": "accepted"
            }
        });

        assert_schema_accepts("acq_result_v1.schema.json", &value);
    }

    #[test]
    fn ephemeris_schema_accepts_generic_artifact_payload() {
        let value = serde_json::json!({
            "header": artifact_header_fixture(),
            "payload": []
        });

        assert_schema_accepts("gps_ephemeris_v1.schema.json", &value);
    }

    #[test]
    fn sidecar_schema_accepts_serialized_optional_metadata() {
        let value = serde_json::json!({
            "format": "iq8",
            "sample_rate_hz": 4_092_000.0,
            "intermediate_freq_hz": 0.0,
            "capture_start_utc": "2026-01-01T00:00:00Z",
            "offset_bytes": 0,
            "quantization_bits": null,
            "notes": null
        });

        assert_schema_accepts("sidecar.schema.json", &value);
    }
}
