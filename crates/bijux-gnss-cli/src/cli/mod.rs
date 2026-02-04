use std::fs;
use std::path::{Path, PathBuf};

use bijux_gnss_infra::api::core::{
    sort_obs_sats, validate_obs_epochs, AcqResultV1, ArtifactHeaderV1, ArtifactReadPolicy,
    DiagnosticEvent, DiagnosticSeverity, Constellation, NavSolutionEpochV1,
    ObsEpoch, ObsEpochV1, SamplesFrame, SatId, SchemaVersion, TrackEpoch, TrackEpochV1,
    ValidateConfig,
};
use bijux_gnss_infra::api::*;
use bijux_gnss_infra::api::nav::{
    elevation_azimuth_deg, sat_state_gps_l1ca, CodeBiasProvider, GpsEphemeris, GpsEphemerisV1,
    Matrix, NavClockModel, PhaseBiasProvider, ProcessNoiseConfig, PseudorangeMeasurement,
    WeightingConfig, write_rinex_nav, write_rinex_obs,
};
use bijux_gnss_infra::api::receiver::{acquisition::Acquisition, data::FileSamples, ReceiverConfig, ReceiverProfile};
use bijux_gnss_infra::api::signal::{generate_ca_code, samples_per_code, Prn, SignalSource};
use clap::{Args, Parser, Subcommand, ValueEnum};
use eyre::{bail, eyre, Context, ContextCompat, Result};
use serde::{Deserialize, Serialize};

fn workspace_root() -> &'static Path {
    let crate_dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    crate_dir
        .parent()
        .and_then(|p| p.parent())
        .expect("workspace root")
}

pub(crate) fn schema_path(name: &str) -> PathBuf {
    workspace_root().join("schemas").join(name)
}

fn infra_args(common: &CommonArgs) -> RunContextArgs<'_> {
    RunContextArgs {
        config: common.config.as_ref(),
        dataset_id: common.dataset.as_deref(),
        unregistered_dataset: common.unregistered_dataset,
        out: common.out.as_ref(),
        resume: common.resume.as_ref(),
        deterministic: common.deterministic,
    }
}

fn run_dir(common: &CommonArgs, command: &str, dataset: Option<&DatasetEntry>) -> Result<PathBuf> {
    Ok(bijux_gnss_infra::api::run_dir(
        &infra_args(common),
        command,
        dataset,
    )?)
}

fn artifacts_dir(
    common: &CommonArgs,
    command: &str,
    dataset: Option<&DatasetEntry>,
) -> Result<PathBuf> {
    Ok(bijux_gnss_infra::api::artifacts_dir(
        &infra_args(common),
        command,
        dataset,
    )?)
}

fn artifact_header(
    common: &CommonArgs,
    profile: &ReceiverProfile,
    dataset: Option<&DatasetEntry>,
) -> Result<ArtifactHeaderV1> {
    Ok(bijux_gnss_infra::api::artifact_header(
        &infra_args(common),
        profile,
        dataset,
    )?)
}

fn write_manifest<T: Serialize>(
    common: &CommonArgs,
    command: &str,
    profile: &ReceiverProfile,
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
