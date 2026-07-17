//! Run provenance helpers.

use bijux_gnss_receiver::api::core::InputError;
use bijux_gnss_receiver::api::ReceiverConfig;

use crate::datasets::{resolve_raw_iq_metadata, DatasetEntry};

use crate::run_layout::directories::context::RunContextArgs;

/// Replay scope persisted in run manifests and reports.
#[derive(Debug, serde::Serialize, Clone)]
pub struct ReplayScope {
    /// Whether deterministic mode was requested.
    pub deterministic: bool,
    /// Whether run resumed from a prior run directory.
    pub resume: bool,
    /// Whether output directory was explicitly requested.
    pub explicit_output_dir: bool,
    /// Requested dataset id from CLI args.
    pub requested_dataset_id: Option<String>,
    /// Whether unregistered datasets were allowed.
    pub allow_unregistered_dataset: bool,
}

/// Front-end provenance captured at run time.
#[derive(Debug, serde::Serialize, Clone)]
pub struct FrontEndProvenance {
    /// Declared raw IQ sample format, if available.
    pub sample_format: Option<String>,
    /// Sample rate used for ingest, in Hz.
    pub sample_rate_hz: f64,
    /// Intermediate frequency used for ingest, in Hz.
    pub intermediate_freq_hz: f64,
    /// Capture start timestamp in UTC, if available.
    pub capture_start_utc: Option<String>,
    /// Quantization depth in bits.
    pub quantization_bits: u8,
    /// Code frequency basis in Hz.
    pub code_freq_basis_hz: f64,
    /// Normalization source.
    pub normalization_source: String,
    /// Calibration source descriptor.
    pub calibration_source: String,
}

pub(super) fn replay_scope(args: &RunContextArgs<'_>) -> ReplayScope {
    ReplayScope {
        deterministic: args.deterministic,
        resume: args.resume.is_some(),
        explicit_output_dir: args.out.is_some(),
        requested_dataset_id: args.dataset_id.map(str::to_string),
        allow_unregistered_dataset: args.unregistered_dataset,
    }
}

pub(super) fn front_end_provenance(
    args: &RunContextArgs<'_>,
    profile: &ReceiverConfig,
    dataset: Option<&DatasetEntry>,
) -> Result<FrontEndProvenance, InputError> {
    let raw_iq_metadata =
        resolve_raw_iq_metadata(dataset, args.sidecar.map(|path| path.as_path())).ok();
    let sidecar_used = args.sidecar.is_some();
    if sidecar_used && raw_iq_metadata.is_none() {
        resolve_raw_iq_metadata(dataset, args.sidecar.map(|path| path.as_path()))?;
    }
    Ok(FrontEndProvenance {
        sample_format: raw_iq_metadata.as_ref().map(|metadata| format!("{:?}", metadata.format)),
        sample_rate_hz: profile.sample_rate_hz,
        intermediate_freq_hz: profile.intermediate_freq_hz,
        capture_start_utc: raw_iq_metadata
            .as_ref()
            .map(|metadata| metadata.capture_start_utc.clone()),
        quantization_bits: profile.quantization_bits,
        code_freq_basis_hz: profile.code_freq_basis_hz,
        normalization_source: if raw_iq_metadata.is_some() {
            "raw_iq_metadata+receiver_profile".to_string()
        } else if sidecar_used {
            "sidecar+receiver_profile".to_string()
        } else {
            "receiver_profile".to_string()
        },
        calibration_source: args
            .sidecar
            .map(|path| format!("sidecar:{}", path.display()))
            .unwrap_or_else(|| "none_declared".to_string()),
    })
}

pub(super) fn enabled_features() -> Vec<String> {
    let mut features = Vec::new();
    if let Ok(value) = std::env::var("BIJUX_GNSS_FEATURES") {
        for item in value.split(',') {
            let trimmed = item.trim();
            if !trimmed.is_empty() {
                features.push(trimmed.to_string());
            }
        }
    }
    features
}

#[cfg(test)]
mod tests {
    use super::front_end_provenance;
    use crate::run_layout::RunContextArgs;
    use bijux_gnss_receiver::api::ReceiverConfig;
    use std::fs;

    #[test]
    fn front_end_provenance_rejects_sidecar_without_sample_rate() {
        let temp = tempfile::tempdir().expect("tempdir");
        let sidecar = temp.path().join("broken.sidecar.toml");
        fs::write(
            &sidecar,
            r#"
format = "iq16_le"
intermediate_freq_hz = 0.0
capture_start_utc = "2026-07-09T00:00:00Z"
"#,
        )
        .expect("write sidecar");

        let args = RunContextArgs {
            config: None,
            dataset_id: None,
            unregistered_dataset: true,
            out: None,
            resume: None,
            deterministic: true,
            sidecar: Some(&sidecar),
        };
        let err = front_end_provenance(&args, &ReceiverConfig::default(), None)
            .expect_err("missing sample rate must fail");
        assert!(err.message.contains("sample_rate_hz") || err.message.contains("missing field"));
    }

    #[test]
    fn front_end_provenance_rejects_sidecar_without_intermediate_frequency() {
        let temp = tempfile::tempdir().expect("tempdir");
        let sidecar = temp.path().join("broken.sidecar.toml");
        fs::write(
            &sidecar,
            r#"
format = "iq16_le"
sample_rate_hz = 5000000.0
capture_start_utc = "2026-07-09T00:00:00Z"
"#,
        )
        .expect("write sidecar");

        let args = RunContextArgs {
            config: None,
            dataset_id: None,
            unregistered_dataset: true,
            out: None,
            resume: None,
            deterministic: true,
            sidecar: Some(&sidecar),
        };
        let err = front_end_provenance(&args, &ReceiverConfig::default(), None)
            .expect_err("missing intermediate frequency must fail");
        assert!(
            err.message.contains("intermediate_freq_hz") || err.message.contains("missing field")
        );
    }
}
