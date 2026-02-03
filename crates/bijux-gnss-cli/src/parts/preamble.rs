use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command as ProcessCommand;
use std::time::{SystemTime, UNIX_EPOCH};

use bijux_gnss_core::{validate_obs_epochs, Constellation, ObsEpoch, SamplesFrame, SatId};
use bijux_gnss_nav::{
    elevation_azimuth_deg, sat_state_gps_l1ca, CodeBiasProvider, GpsEphemeris, Matrix,
    NavClockModel, PhaseBiasProvider, PppConfig, PppFilter, PppProcessNoise, ProcessNoiseConfig,
    ProductsProvider, PseudorangeMeasurement, WeightingConfig,
};
use bijux_gnss_receiver::{
    acquisition::Acquisition, ca_code::generate_ca_code, ca_code::Prn, data::FileSamples,
    data::SampleSource, signal::samples_per_code, ReceiverConfig, ReceiverProfile,
};
use clap::{Args, Parser, Subcommand, ValueEnum};
use eyre::{bail, eyre, Context, ContextCompat, Result};
use jsonschema::JSONSchema;
use schemars::schema_for;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

