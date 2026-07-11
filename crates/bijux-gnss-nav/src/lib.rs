//! GNSS navigation domain: ephemeris parsing, corrections, and positioning.
//! See docs/GLOSSARY.md for acronym definitions.
//!
//! Module map:
//! - `orbits`: broadcast ephemeris and satellite state
//! - `formats`: LNAV parsing and precise products I/O
//! - `corrections`: iono/tropo/bias helpers and combinations
//! - `estimation`: EKF, PPP, and PVT solvers
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

/// Public API surface for this crate.
pub mod api;

pub use crate::estimation::position::solver::{
    position_dops_from_satellite_positions, PositionDops, PositionSolveRefusal,
    PositionSolveRefusalKind,
};
pub use crate::formats::rinex_obs::{
    parse_rinex_gps_observation_dataset, RinexGpsObservationChannel, RinexGpsObservationDataset,
};
pub use crate::estimation::position::raim::{
    RaimFaultDetection, RaimFaultDetectionStatus, RaimFaultExclusion,
};
