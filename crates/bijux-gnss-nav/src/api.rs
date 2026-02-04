//! Public API for bijux-gnss-nav.

/// Versioned artifact wrappers.
///
/// Broadcast ephemeris list artifact v1.
pub type GpsEphemerisV1 = bijux_gnss_core::ArtifactV1<Vec<crate::orbits::gps::GpsEphemeris>>;
/// PPP solution epoch artifact v1.
pub type PppEpochV1 =
    bijux_gnss_core::ppp::PppEpochV1<crate::estimation::ppp::config::PppSolutionEpoch>;
pub use crate::corrections::atmosphere::{clamp_ztd, AtmosphereConfig};
pub use crate::corrections::biases::{CodeBiasProvider, PhaseBiasProvider, ZeroBiases};
/// Corrections and combination helpers.
pub use crate::corrections::combinations::combinations_from_obs_epochs;
pub use crate::corrections::{compute_corrections, CorrectionContext, Corrections};
pub use crate::estimation::ekf::models::{
    AmbiguityManager, CarrierPhaseMeasurement, DopplerMeasurement, InterSystemBiasManager,
    NavClockModel, ProcessNoiseConfig, PseudorangeMeasurement,
};
/// EKF interface and measurements.
pub use crate::estimation::ekf::state::{Ekf, EkfConfig, MeasurementKind};
pub use crate::estimation::ekf::traits::MeasurementModel;
/// Position solver (least squares) and helpers.
pub use crate::estimation::position::solver::{
    ecef_to_enu, ecef_to_geodetic, elevation_azimuth_deg, geodetic_to_ecef, weight_from_cn0_elev,
    PositionObservation, PositionSolver, WeightingConfig,
};
/// PPP configuration and filter.
pub use crate::estimation::ppp::config::{
    PppArMode, PppConfig, PppConvergenceConfig, PppFilter, PppProcessNoise, PppSolutionEpoch,
};
/// Format parsing and output.
pub use crate::formats::lnav_bits::{bit_sync_from_prompt, decode_subframes};
pub use crate::formats::lnav_decode::{decode_rawephem_hex, decode_subframe_hex};
/// Precise product providers (SP3/CLK) and fallbacks.
pub use crate::formats::precise_products::{
    BroadcastProductsProvider, ProductDiagnostics, Products, ProductsProvider,
};
pub use crate::formats::rinex::{write_rinex_nav, write_rinex_obs};
/// Precise product parsing helpers.
pub use crate::formats::{clk::ClkProvider, sp3::Sp3Provider};
/// Linear algebra helper.
pub use crate::linalg::Matrix;
/// Ephemeris provider traits and helpers.
pub use crate::orbits::ephemeris::{CsvEphemerisProvider, Ephemeris, EphemerisProvider};
/// Broadcast ephemeris and satellite state.
pub use crate::orbits::gps::{sat_state_gps_l1ca, GpsEphemeris, GpsSatState};
/// Time helpers.
pub use crate::time::{gps_time_from_utc, gps_week_rollover, normalize_tow};

/// Navigation engine trait boundary.
pub trait NavEngine {
    /// Update navigation solution with new observation epoch.
    fn update(&mut self, obs: &bijux_gnss_core::ObsEpochV1) -> bijux_gnss_core::NavSolutionEpochV1;
}
