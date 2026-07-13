use std::fs;
use std::path::{Path, PathBuf};

use bijux_gnss_infra::api::core::{
    sort_obs_sats, validate_obs_epochs, AcqAssumptions, AcqResult, AcqResultV1, ArtifactHeaderV1,
    ArtifactReadPolicy, Constellation, DiagnosticEvent, DiagnosticSeverity, NavSolutionEpoch,
    NavSolutionEpochV1, ObsEpoch, ObsEpochV1, SamplesFrame, SatId, SchemaVersion, TrackEpoch,
    SignalBand, TrackEpochV1, ValidateConfig,
};
use bijux_gnss_infra::api::hash_config;
use bijux_gnss_infra::api::nav::{
    write_rinex_nav, write_rinex_obs, CodeBiasProvider, GpsEphemeris, GpsEphemerisV1,
};
use bijux_gnss_infra::api::parse_ecef;
use bijux_gnss_infra::api::receiver::{
    AcquisitionEngine, FileSamples, Receiver, ReceiverConfig, ReceiverPipelineConfig,
};
use bijux_gnss_infra::api::signal::{
    ca_code_assignment, ca_code_autocorrelation_summary, ca_code_cross_correlation_summary,
    generate_ca_code_chips, samples_per_code, Prn, RawIqMetadata, SignalSource,
    CA_CODE_PERIOD_CHIPS,
};
use bijux_gnss_infra::api::RunManifest;
use bijux_gnss_infra::api::{
    apply_common_overrides, apply_overrides, apply_sweep_value, CommonOverrides,
};
use bijux_gnss_infra::api::{artifact_explain, artifact_validate, prepare_run};
use bijux_gnss_infra::api::{
    build_validation_report, build_validation_report_from_observation_artifacts,
};
use bijux_gnss_infra::api::{expand_sweep, parse_sweep};
use bijux_gnss_infra::api::{DatasetEntry, DatasetRegistry};
use bijux_gnss_infra::api::{ValidationReferenceEpoch, ValidationReport};
use clap::{Args, Parser, Subcommand, ValueEnum};
use eyre::{bail, eyre, Context, ContextCompat, Result};
use serde::{Deserialize, Serialize};

fn workspace_root() -> &'static Path {
    let crate_dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    crate_dir.parent().and_then(|p| p.parent()).expect("workspace root")
}

#[derive(Clone, Copy)]
enum CliErrorClass {
    OperatorMisconfiguration,
    UnsupportedScience,
    InternalFault,
}

fn classified_error(class: CliErrorClass, detail: impl AsRef<str>) -> eyre::Report {
    let (code, label) = match class {
        CliErrorClass::OperatorMisconfiguration => ("operator_misconfiguration", "operator"),
        CliErrorClass::UnsupportedScience => ("unsupported_science", "science"),
        CliErrorClass::InternalFault => ("internal_fault", "internal"),
    };
    eyre!("[{label}:{code}] {}", detail.as_ref())
}

pub(crate) fn schema_path(name: &str) -> PathBuf {
    workspace_root().join("schemas").join(name)
}

fn infra_args(common: &CommonArgs) -> bijux_gnss_infra::api::RunContextArgs<'_> {
    bijux_gnss_infra::api::RunContextArgs {
        config: common.config.as_ref(),
        dataset_id: common.dataset.as_deref(),
        unregistered_dataset: common.unregistered_dataset,
        out: common.out.as_ref(),
        resume: common.resume.as_ref(),
        deterministic: common.deterministic,
        sidecar: common.sidecar.as_ref(),
    }
}

fn run_dir(common: &CommonArgs, command: &str, dataset: Option<&DatasetEntry>) -> Result<PathBuf> {
    Ok(bijux_gnss_infra::api::run_dir(&infra_args(common), command, dataset)?)
}

fn artifacts_dir(
    common: &CommonArgs,
    command: &str,
    dataset: Option<&DatasetEntry>,
) -> Result<PathBuf> {
    Ok(bijux_gnss_infra::api::artifacts_dir(&infra_args(common), command, dataset)?)
}

fn artifact_header(
    common: &CommonArgs,
    profile: &ReceiverConfig,
    dataset: Option<&DatasetEntry>,
) -> Result<ArtifactHeaderV1> {
    Ok(bijux_gnss_infra::api::artifact_header(&infra_args(common), profile, dataset)?)
}

fn apply_acquisition_doppler_overrides(
    profile: &mut ReceiverConfig,
    doppler_search_hz: Option<i32>,
    doppler_step_hz: Option<i32>,
) {
    if let Some(search_hz) = doppler_search_hz {
        profile.acquisition.doppler_search_hz = search_hz;
    }
    if let Some(step_hz) = doppler_step_hz {
        profile.acquisition.doppler_step_hz = step_hz;
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
struct DopplerSearchSettings {
    max_search_hz: i32,
    bin_width_hz: i32,
    bin_count: usize,
    intermediate_freq_hz: f64,
}

fn doppler_search_settings(profile: &ReceiverConfig) -> DopplerSearchSettings {
    let max_search_hz = profile.acquisition.doppler_search_hz.max(0);
    let bin_width_hz = profile.acquisition.doppler_step_hz.max(1);
    let bin_count = ((max_search_hz / bin_width_hz) as usize * 2) + 1;
    DopplerSearchSettings {
        max_search_hz,
        bin_width_hz,
        bin_count,
        intermediate_freq_hz: profile.intermediate_freq_hz,
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct CodePhaseSearchSettings {
    start_sample: usize,
    step_samples: usize,
    bin_count: usize,
    period_samples: usize,
    mode: String,
}

fn code_phase_search_settings(profile: &ReceiverConfig) -> CodePhaseSearchSettings {
    let period_samples =
        samples_per_code(profile.sample_rate_hz, profile.code_freq_basis_hz, profile.code_length);
    CodePhaseSearchSettings {
        start_sample: 0,
        step_samples: 1,
        bin_count: period_samples,
        period_samples,
        mode: "full_code".to_string(),
    }
}

fn code_phase_search_settings_from_assumptions(
    assumptions: &AcqAssumptions,
) -> CodePhaseSearchSettings {
    CodePhaseSearchSettings {
        start_sample: assumptions.code_phase_search_start_sample,
        step_samples: assumptions.code_phase_search_step_samples,
        bin_count: assumptions.code_phase_search_bins,
        period_samples: assumptions.samples_per_code,
        mode: assumptions.code_phase_search_mode.clone(),
    }
}

fn code_phase_search_settings_from_results(
    profile: &ReceiverConfig,
    results: &[Vec<AcqResult>],
) -> CodePhaseSearchSettings {
    results
        .iter()
        .flat_map(|candidates| candidates.iter())
        .find_map(|result| result.assumptions.as_ref())
        .map(code_phase_search_settings_from_assumptions)
        .unwrap_or_else(|| code_phase_search_settings(profile))
}

fn write_manifest<T: Serialize>(
    common: &CommonArgs,
    command: &str,
    profile: &ReceiverConfig,
    dataset: Option<&DatasetEntry>,
    summary: &T,
) -> Result<RunManifest> {
    let value = serde_json::to_value(summary)?;
    Ok(bijux_gnss_infra::api::write_manifest(
        &infra_args(common),
        command,
        profile,
        dataset,
        &value,
    )?)
}
