use std::fs;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::time::{SystemTime, UNIX_EPOCH};

use bijux_gnss_core::{
    check_nav_solution_finite, check_obs_epoch_finite, sort_obs_sats,
    validate_obs_epochs, AcqResultV1, ArtifactCompatibility, ArtifactHeader,
    Constellation, NavSolutionEpochV1, ObsEpoch, ObsEpochV1, SamplesFrame, SatId, SchemaVersion,
    TrackEpochV1, ValidateConfig,
};
use bijux_gnss_nav::{
    elevation_azimuth_deg, sat_state_gps_l1ca, CodeBiasProvider, GpsEphemeris, GpsEphemerisV1,
    Matrix, NavClockModel, PhaseBiasProvider, PppConfig, PppProcessNoise, ProcessNoiseConfig,
    PseudorangeMeasurement, WeightingConfig, write_rinex_nav, write_rinex_obs,
};
use bijux_gnss_receiver::{
    acquisition::Acquisition, data::FileSamples, data::SampleSource, ReceiverConfig,
    ReceiverProfile,
};
use bijux_gnss_signal::{generate_ca_code, samples_per_code, Prn};
use clap::{Args, Parser, Subcommand, ValueEnum};
use eyre::{bail, eyre, Context, ContextCompat, Result};
use jsonschema::JSONSchema;
use schemars::schema_for;
use serde::{Deserialize, Serialize};
