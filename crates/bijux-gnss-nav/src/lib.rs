//! GNSS navigation domain: ephemeris parsing, corrections, and positioning.
//! See docs/GLOSSARY.md for acronym definitions.
//!
//! Module map:
//! - `orbits`: broadcast ephemeris and satellite state
//! - `formats`: LNAV parsing and precise products I/O
//! - `corrections`: iono/tropo/bias helpers and combinations
//! - `estimation`: EKF, PPP, PVT, and RTK solvers
//! - `linalg`: minimal matrix helpers for estimators
//! - `time`: navigation-time utilities
//! - Start here: `sat_state_gps_l1ca` and `PositionSolver`

#![deny(clippy::unwrap_used)]
#![deny(missing_docs)]
#![forbid(unsafe_code)]

mod corrections;
mod estimation;
mod formats;
mod linalg;
mod models;
mod orbits;
mod time;

pub use crate::formats::antex::{
    parse_antex_receiver_calibrations, parse_antex_satellite_calibrations,
};
pub use crate::models::antenna::{
    canonical_receiver_antenna_type, receiver_antenna_range_correction_m,
    satellite_antenna_range_correction_m, satellite_band_from_antex_frequency,
    ReceiverAntennaCalibration, ReceiverAntennaCalibrations, ReceiverPhaseCenterOffset,
    SatelliteAntennaCalibration, SatelliteAntennaCalibrations, SatellitePhaseCenterOffset,
};
pub use crate::models::ocean_tide_loading::{
    OceanTideConstituent, OceanTideLoadingConstituent, OceanTideLoadingModel,
};
pub use crate::estimation::ppp::config::PppTroposphereSource;
pub use crate::estimation::rtk::antenna::RtkAntennaCorrectionConfig;
pub use crate::estimation::rtk::double_difference::rtk_double_difference_residual_metrics_with_antenna_corrections;
pub use crate::estimation::rtk::single_difference::rtk_single_difference_residual_metrics_with_antenna_corrections;

/// Public API surface for this crate.
pub mod api;
