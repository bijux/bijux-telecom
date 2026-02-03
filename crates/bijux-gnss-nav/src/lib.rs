//! GNSS navigation domain: ephemeris parsing and positioning.

#![deny(clippy::unwrap_used)]

pub mod atmosphere;
pub mod biases;
pub mod clk;
pub mod corrections;
pub mod ekf;
pub mod linalg;
pub mod products;
pub mod sp3;

pub use linalg::Matrix;
pub mod ephemeris;
pub mod gps;
pub mod position;

pub use crate::atmosphere::*;
pub use crate::biases::*;
pub use crate::clk::*;
pub use crate::corrections::*;
pub use crate::ekf::*;
pub use crate::ephemeris::{CsvEphemerisProvider, Ephemeris, EphemerisError, EphemerisProvider};
pub use crate::gps::{
    bit_sync_from_prompt, decode_rawephem_hex, decode_subframe_hex, decode_subframes,
    find_preamble, sat_state_gps_l1ca, BitSyncResult, GpsEphemeris, GpsSatState, LnavDecodeStats,
    SubframeInfo,
};
pub use crate::position::{
    ecef_to_enu, ecef_to_geodetic, elevation_azimuth_deg, geodetic_to_ecef, weight_from_cn0_elev,
    PositionObservation, PositionSolution, PositionSolver, WeightingConfig,
};
pub use crate::products::*;
pub use crate::sp3::*;
