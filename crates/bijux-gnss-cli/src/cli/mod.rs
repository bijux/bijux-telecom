use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command as ProcessCommand;
use std::time::{SystemTime, UNIX_EPOCH};

use bijux_gnss_core::{validate_obs_epochs, Constellation, ObsEpoch, SamplesFrame, SatId};
use bijux_gnss_nav::{
    elevation_azimuth_deg, sat_state_gps_l1ca, CodeBiasProvider, GpsEphemeris, Matrix,
    NavClockModel, PhaseBiasProvider, PppConfig, PppFilter, PppProcessNoise, ProcessNoiseConfig,
    ProductsProvider, PseudorangeMeasurement, WeightingConfig, write_rinex_nav, write_rinex_obs,
};
use bijux_gnss_receiver::{
    acquisition::Acquisition, data::FileSamples, data::SampleSource, ReceiverConfig,
    ReceiverProfile,
};
use bijux_gnss_signal::codes::ca_code::{generate_ca_code, Prn};
use bijux_gnss_signal::signal::samples_per_code;
use clap::{Args, Parser, Subcommand, ValueEnum};
use eyre::{bail, eyre, Context, ContextCompat, Result};
use jsonschema::JSONSchema;
use schemars::schema_for;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
