use std::fs;
use std::path::{Path, PathBuf};

use bijux_gnss_core::{
    sort_obs_sats, validate_obs_epochs, AcqResultV1, ArtifactHeaderV1, ArtifactReadPolicy,
    ArtifactValidate, DiagnosticEvent, DiagnosticSeverity, Constellation, NavSolutionEpochV1,
    ObsEpoch, ObsEpochV1, SamplesFrame, SatId, SchemaVersion, TrackEpoch, TrackEpochV1,
    ValidateConfig,
};
use bijux_gnss_infra::*;
use bijux_gnss_nav::{
    elevation_azimuth_deg, sat_state_gps_l1ca, CodeBiasProvider, GpsEphemeris, GpsEphemerisV1,
    Matrix, NavClockModel, PhaseBiasProvider, PppConfig, PppProcessNoise, ProcessNoiseConfig,
    PseudorangeMeasurement, WeightingConfig, write_rinex_nav, write_rinex_obs,
};
use bijux_gnss_receiver::{acquisition::Acquisition, data::FileSamples, ReceiverConfig, ReceiverProfile};
use bijux_gnss_signal::{generate_ca_code, samples_per_code, Prn, SignalSource};
use clap::{Args, Parser, Subcommand, ValueEnum};
use eyre::{bail, eyre, Context, ContextCompat, Result};
use jsonschema::JSONSchema;
use schemars::schema_for;
use serde::{Deserialize, Serialize};

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
    Ok(bijux_gnss_infra::run_dir(
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
    Ok(bijux_gnss_infra::artifacts_dir(
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
    Ok(bijux_gnss_infra::artifact_header(
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
    Ok(bijux_gnss_infra::write_manifest(
        &infra_args(common),
        command,
        profile,
        dataset,
        &value,
    )?)
}
