//! Raw IQ metadata loading and validation.

use std::path::Path;

use bijux_gnss_receiver::api::core::InputError;
use bijux_gnss_signal::api::RawIqMetadata;

use super::DatasetEntry;

mod loading;
mod resolution;
mod validation;

/// Load and validate raw IQ metadata from a sidecar file.
pub fn load_raw_iq_metadata(path: &Path) -> Result<RawIqMetadata, InputError> {
    loading::load_raw_iq_metadata(path)
}

/// Resolve raw IQ metadata from explicit sidecar input or dataset registry metadata.
pub fn resolve_raw_iq_metadata(
    dataset: Option<&DatasetEntry>,
    explicit_sidecar: Option<&Path>,
) -> Result<RawIqMetadata, InputError> {
    resolution::resolve_raw_iq_metadata(dataset, explicit_sidecar)
}

pub(super) fn validate_dataset_metadata_match(
    dataset: Option<&DatasetEntry>,
    dataset_metadata: &RawIqMetadata,
    sidecar_metadata: &RawIqMetadata,
) -> Result<(), InputError> {
    validation::validate_dataset_metadata_match(dataset, dataset_metadata, sidecar_metadata)
}

pub(super) fn validate_raw_iq_metadata(metadata: &RawIqMetadata) -> Result<(), InputError> {
    validation::validate_raw_iq_metadata(metadata)
}

pub(super) fn map_err(err: impl std::fmt::Display) -> InputError {
    InputError { message: err.to_string() }
}

#[cfg(test)]
mod tests {
    use super::{load_raw_iq_metadata, resolve_raw_iq_metadata};
    use crate::datasets::DatasetRegistry;
    use std::fs;

    #[test]
    fn resolve_raw_iq_metadata_prefers_validated_sidecar() {
        let temp = tempfile::tempdir().expect("tempdir");
        let sidecar_path = temp.path().join("demo.sidecar.toml");
        fs::write(
            &sidecar_path,
            r#"
format = "iq16_le"
sample_rate_hz = 5000000.0
intermediate_freq_hz = 0.0
capture_start_utc = "2026-07-09T00:00:00Z"
offset_bytes = 128
quantization_bits = 16
"#,
        )
        .expect("write sidecar");
        let registry_path = temp.path().join("registry.toml");
        fs::write(
            &registry_path,
            format!(
                r#"
version = 1

[[entries]]
id = "demo"
path = "{}"
format = "iq16_le"
sample_rate_hz = 5000000.0
intermediate_freq_hz = 0.0
capture_start_utc = "2026-07-09T00:00:00Z"
expected_sats = [1]
expected_region = "N/A"
expected_time_utc = "N/A"
sidecar = "{}"
"#,
                temp.path().join("demo.iq16").display(),
                sidecar_path.display()
            ),
        )
        .expect("write registry");

        let registry = DatasetRegistry::load(&registry_path).expect("load registry");
        let entry = registry.find("demo").expect("find dataset");
        let metadata =
            resolve_raw_iq_metadata(Some(&entry), None).expect("resolve raw iq metadata");

        assert_eq!(metadata.format, bijux_gnss_signal::api::IqSampleFormat::Iq16Le);
        assert_eq!(metadata.offset_bytes, 128);
        assert_eq!(metadata.quantization_bits, Some(16));
    }

    #[test]
    fn resolve_raw_iq_metadata_uses_sidecar_when_registry_omits_sample_rate() {
        let temp = tempfile::tempdir().expect("tempdir");
        let sidecar_path = temp.path().join("demo.sidecar.toml");
        fs::write(
            &sidecar_path,
            r#"
format = "iq16_le"
sample_rate_hz = 5000000.0
intermediate_freq_hz = 0.0
capture_start_utc = "2026-07-09T00:00:00Z"
quantization_bits = 16
"#,
        )
        .expect("write sidecar");
        let registry_path = temp.path().join("registry.toml");
        fs::write(
            &registry_path,
            format!(
                r#"
version = 1

[[entries]]
id = "demo"
path = "{}"
format = "iq16_le"
intermediate_freq_hz = 0.0
capture_start_utc = "2026-07-09T00:00:00Z"
expected_sats = [1]
expected_region = "N/A"
expected_time_utc = "N/A"
sidecar = "{}"
"#,
                temp.path().join("demo.iq16").display(),
                sidecar_path.display()
            ),
        )
        .expect("write registry");

        let registry = DatasetRegistry::load(&registry_path).expect("load registry");
        let entry = registry.find("demo").expect("find dataset");
        let metadata =
            resolve_raw_iq_metadata(Some(&entry), None).expect("resolve raw iq metadata");

        assert_eq!(metadata.sample_rate_hz, 5_000_000.0);
        assert_eq!(metadata.quantization_bits, Some(16));
    }

    #[test]
    fn resolve_raw_iq_metadata_uses_sidecar_when_registry_omits_intermediate_frequency() {
        let temp = tempfile::tempdir().expect("tempdir");
        let sidecar_path = temp.path().join("demo.sidecar.toml");
        fs::write(
            &sidecar_path,
            r#"
format = "iq16_le"
sample_rate_hz = 5000000.0
intermediate_freq_hz = 250000.0
capture_start_utc = "2026-07-09T00:00:00Z"
quantization_bits = 16
"#,
        )
        .expect("write sidecar");
        let registry_path = temp.path().join("registry.toml");
        fs::write(
            &registry_path,
            format!(
                r#"
version = 1

[[entries]]
id = "demo"
path = "{}"
format = "iq16_le"
sample_rate_hz = 5000000.0
capture_start_utc = "2026-07-09T00:00:00Z"
expected_sats = [1]
expected_region = "N/A"
expected_time_utc = "N/A"
sidecar = "{}"
"#,
                temp.path().join("demo.iq16").display(),
                sidecar_path.display()
            ),
        )
        .expect("write registry");

        let registry = DatasetRegistry::load(&registry_path).expect("load registry");
        let entry = registry.find("demo").expect("find dataset");
        let metadata =
            resolve_raw_iq_metadata(Some(&entry), None).expect("resolve raw iq metadata");

        assert_eq!(metadata.intermediate_freq_hz, 250_000.0);
        assert_eq!(metadata.quantization_bits, Some(16));
    }

    #[test]
    fn load_raw_iq_metadata_rejects_signed_16bit_quantization_mismatch() {
        let temp = tempfile::tempdir().expect("tempdir");
        let sidecar_path = temp.path().join("broken.sidecar.toml");
        fs::write(
            &sidecar_path,
            r#"
format = "iq16_le"
sample_rate_hz = 5000000.0
intermediate_freq_hz = 0.0
capture_start_utc = "2026-07-09T00:00:00Z"
quantization_bits = 8
"#,
        )
        .expect("write sidecar");

        let err = load_raw_iq_metadata(&sidecar_path).expect_err("quantization mismatch must fail");
        assert!(err.message.contains("quantization_bits"));
        assert!(err.message.contains("Iq16Le"));
    }

    #[test]
    fn load_raw_iq_metadata_rejects_complex_float32_quantization_mismatch() {
        let temp = tempfile::tempdir().expect("tempdir");
        let sidecar_path = temp.path().join("broken.sidecar.toml");
        fs::write(
            &sidecar_path,
            r#"
format = "cf32_le"
sample_rate_hz = 5000000.0
intermediate_freq_hz = 0.0
capture_start_utc = "2026-07-09T00:00:00Z"
quantization_bits = 16
"#,
        )
        .expect("write sidecar");

        let err = load_raw_iq_metadata(&sidecar_path).expect_err("quantization mismatch must fail");
        assert!(err.message.contains("quantization_bits"));
        assert!(err.message.contains("Cf32Le"));
    }

    #[test]
    fn load_raw_iq_metadata_rejects_missing_sample_rate() {
        let temp = tempfile::tempdir().expect("tempdir");
        let sidecar_path = temp.path().join("broken.sidecar.toml");
        fs::write(
            &sidecar_path,
            r#"
format = "iq16_le"
intermediate_freq_hz = 0.0
capture_start_utc = "2026-07-09T00:00:00Z"
"#,
        )
        .expect("write sidecar");

        let err = load_raw_iq_metadata(&sidecar_path).expect_err("missing sample rate must fail");
        assert!(err.message.contains("sample_rate_hz"));
    }

    #[test]
    fn load_raw_iq_metadata_rejects_missing_intermediate_frequency() {
        let temp = tempfile::tempdir().expect("tempdir");
        let sidecar_path = temp.path().join("broken.sidecar.toml");
        fs::write(
            &sidecar_path,
            r#"
format = "iq16_le"
sample_rate_hz = 5000000.0
capture_start_utc = "2026-07-09T00:00:00Z"
"#,
        )
        .expect("write sidecar");

        let err = load_raw_iq_metadata(&sidecar_path)
            .expect_err("missing intermediate frequency must fail");
        assert!(err.message.contains("intermediate_freq_hz"));
    }

    #[test]
    fn load_raw_iq_metadata_rejects_missing_timestamp() {
        let temp = tempfile::tempdir().expect("tempdir");
        let sidecar_path = temp.path().join("broken.sidecar.toml");
        fs::write(
            &sidecar_path,
            r#"
format = "iq16_le"
sample_rate_hz = 5000000.0
intermediate_freq_hz = 0.0
"#,
        )
        .expect("write sidecar");

        let err = load_raw_iq_metadata(&sidecar_path).expect_err("missing timestamp must fail");
        assert!(err.message.contains("capture_start_utc"));
    }
}
