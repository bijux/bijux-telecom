//! GNSS navigation domain: ephemeris parsing, corrections, and positioning.
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

pub mod corrections;
pub mod estimation;
pub mod formats;
pub mod linalg;
pub mod orbits;
pub mod time;

pub use corrections::*;
pub use estimation::*;
pub use formats::*;
pub use linalg::Matrix;
pub use orbits::*;
