use std::fs;
use std::path::{Path, PathBuf};
use std::time::{SystemTime, UNIX_EPOCH};

use bijux_gnss_core::{
    validate_obs_epochs, Constellation, ObsEpoch, SamplesFrame, SatId, SchemaVersion,
    ValidateConfig,
};
use bijux_gnss_nav::{
    elevation_azimuth_deg, sat_state_gps_l1ca, CodeBiasProvider, GpsEphemeris, Matrix,
    NavClockModel, PhaseBiasProvider, PppConfig, PppProcessNoise, ProcessNoiseConfig,
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
