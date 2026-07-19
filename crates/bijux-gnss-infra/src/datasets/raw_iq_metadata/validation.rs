use bijux_gnss_receiver::api::core::InputError;
use bijux_gnss_signal::api::{IqSampleFormat, RawIqMetadata};

use crate::datasets::DatasetEntry;

pub(super) fn validate_dataset_metadata_match(
    dataset: Option<&DatasetEntry>,
    dataset_metadata: &RawIqMetadata,
    sidecar_metadata: &RawIqMetadata,
) -> Result<(), InputError> {
    let matches = dataset_metadata.format == sidecar_metadata.format
        && (dataset_metadata.sample_rate_hz - sidecar_metadata.sample_rate_hz).abs() < f64::EPSILON
        && (dataset_metadata.intermediate_freq_hz - sidecar_metadata.intermediate_freq_hz).abs()
            < f64::EPSILON
        && dataset_metadata.capture_start_utc == sidecar_metadata.capture_start_utc;
    if !matches {
        let dataset_id = dataset.map(|entry| entry.id.as_str()).unwrap_or("unknown");
        return Err(InputError {
            message: format!(
                "dataset registry metadata and sidecar metadata disagree for dataset {dataset_id}"
            ),
        });
    }

    Ok(())
}

pub(super) fn validate_raw_iq_metadata(metadata: &RawIqMetadata) -> Result<(), InputError> {
    if !metadata.sample_rate_hz.is_finite() || metadata.sample_rate_hz <= 0.0 {
        return Err(InputError {
            message: "raw IQ metadata must declare a positive sample_rate_hz".to_string(),
        });
    }
    if !metadata.intermediate_freq_hz.is_finite() {
        return Err(InputError {
            message: "raw IQ metadata must declare a finite intermediate_freq_hz".to_string(),
        });
    }
    if metadata.capture_start_utc.trim().is_empty() {
        return Err(InputError {
            message: "raw IQ metadata must declare capture_start_utc".to_string(),
        });
    }
    if let Some(bits) = metadata.quantization_bits {
        let expected_bits = match metadata.format {
            IqSampleFormat::Iq8 => 8,
            IqSampleFormat::Iq16Le => 16,
            IqSampleFormat::Cf32Le => 32,
        };
        if bits != expected_bits {
            return Err(InputError {
                message: format!(
                    "raw IQ metadata quantization_bits {bits} does not match format {:?}",
                    metadata.format
                ),
            });
        }
    }

    Ok(())
}
