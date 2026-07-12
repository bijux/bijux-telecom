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

pub use crate::estimation::ppp::config::PppTroposphereSource;

/// Public API surface for this crate.
pub mod api;
