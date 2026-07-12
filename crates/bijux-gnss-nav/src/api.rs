//! Public API for bijux-gnss-nav.

/// Versioned artifact wrappers.
///
/// Broadcast ephemeris list artifact v1.
pub type GpsEphemerisV1 = bijux_gnss_core::api::ArtifactV1<Vec<crate::orbits::gps::GpsEphemeris>>;
/// PPP solution epoch artifact v1.
pub type PppEpochV1 =
    bijux_gnss_core::api::ppp::PppEpochV1<crate::estimation::ppp::config::PppSolutionEpoch>;
pub use crate::corrections::atmosphere::{clamp_ztd, AtmosphereConfig};
pub use crate::corrections::biases::{CodeBiasProvider, PhaseBiasProvider, ZeroBiases};
/// Corrections and combination helpers.
pub use crate::corrections::combinations::combinations_from_obs_epochs;
pub use crate::corrections::geometry_free::{
    geometry_free_diagnostics_from_obs_epochs, GeometryFreeEvent, GeometryFreeObservation,
    GeometryFreeThresholds,
};
pub use crate::corrections::iono_free_code::{
    iono_free_code_from_obs_epochs, IonoFreeCodeObservation,
};
pub use crate::corrections::iono_free_phase::{
    iono_free_phase_from_obs_epochs, IonoFreePhaseObservation,
};
pub use crate::corrections::melbourne_wubbena::{
    melbourne_wubbena_diagnostics_from_obs_epochs, MelbourneWubbenaEvent,
    MelbourneWubbenaObservation, MelbourneWubbenaThresholds,
};
pub use crate::corrections::{compute_corrections, CorrectionContext, Corrections};
pub use crate::estimation::ekf::models::{
    AmbiguityManager, CarrierPhaseMeasurement, DopplerMeasurement, InterSystemBiasManager,
    NavClockModel, ProcessNoiseConfig, PseudorangeMeasurement,
};
/// EKF interface and measurements.
pub use crate::estimation::ekf::state::{Ekf, EkfConfig, MeasurementKind};
pub use crate::estimation::ekf::traits::MeasurementModel;
pub use crate::estimation::position::raim::{
    RaimFaultDetection, RaimFaultDetectionStatus, RaimFaultExclusion,
};
/// Position solver (least squares) and helpers.
pub use crate::estimation::position::solver::{
    ecef_to_enu, ecef_to_geodetic, elevation_azimuth_deg, geodetic_to_ecef,
    position_broadcast_navigation_from_glonass_frames,
    position_broadcast_navigation_from_gps_ephemerides, position_dops_from_satellite_positions,
    position_measurement_weight, position_observation_has_valid_satellite_time,
    position_observations_from_epoch, weight_from_cn0_elev, weight_from_pseudorange_sigma,
    PositionBroadcastNavigation, PositionDops, PositionObservation, PositionSolveRefusal,
    PositionSolveRefusalKind, PositionSolver, WeightingConfig,
};
/// PPP configuration and filter.
pub use crate::estimation::ppp::config::{
    PppArMode, PppConfig, PppConvergenceConfig, PppFilter, PppProcessNoise, PppSolutionEpoch,
};
/// RTK float-baseline helpers.
pub use crate::estimation::rtk::ambiguity::{
    rtk_ambiguity_state_from_fixed_solution, rtk_candidate_ratio,
    rtk_conditioned_baseline_from_fixed_ambiguities,
    rtk_float_ambiguity_state_from_baseline_solution, rtk_float_ambiguity_state_from_filter_state,
    rtk_integer_ambiguity_candidates, rtk_lambda_decorrelate, rtk_ratio_test_acceptance,
    rtk_select_partial_ambiguity_fix, RtkAmbiguityFixAudit, RtkAmbiguityFixPolicy,
    RtkAmbiguityFixResult, RtkAmbiguityFixState, RtkAmbiguityFixStatus, RtkAmbiguityTracker,
    RtkConditionedBaselineSolution, RtkDecorrelatedAmbiguityState, RtkDoubleDifferenceAmbiguityId,
    RtkFloatAmbiguityState, RtkIntegerAmbiguityCandidate, RtkRatioTestFixer,
};
pub use crate::estimation::rtk::baseline::{
    rtk_float_baseline_from_double_differences,
    rtk_float_baseline_from_double_differences_with_rover_prior, RtkFloatAmbiguityEstimate,
    RtkFloatBaselineSolution,
};
/// RTK double-difference helpers.
pub use crate::estimation::rtk::double_difference::{
    rtk_double_difference_residual_metrics, rtk_double_differences_by_constellation,
    rtk_double_differences_from_single_differences, RtkDoubleDifferenceObservation,
    RtkDoubleDifferenceResidualMetrics,
};
/// RTK single-difference helpers.
pub use crate::estimation::rtk::single_difference::{
    choose_rtk_single_difference_reference_signal,
    choose_rtk_single_difference_reference_signals_by_constellation,
    rtk_single_difference_residual_metrics, rtk_single_differences_from_obs_epochs,
    RtkSingleDifferenceObservation, RtkSingleDifferenceResidualMetrics,
};
/// Format parsing and output.
pub use crate::formats::galileo_inav_decode::{
    decode_galileo_broadcast_navigation_data, decode_galileo_broadcast_navigation_data_payloads,
    decode_galileo_inav_clock_word, decode_galileo_inav_ephemeris_1_word,
    decode_galileo_inav_ephemeris_2_word, decode_galileo_inav_ephemeris_3_word,
    decode_galileo_inav_status_word, decode_galileo_inav_word, decode_galileo_inav_word_bytes,
    decode_galileo_inav_word_hex, GalileoInavBatchRejection, GalileoInavBatchRejectionReason,
    GalileoInavClockWord, GalileoInavEphemeris1Word, GalileoInavEphemeris2Word,
    GalileoInavEphemeris3Word, GalileoInavStatusWord, GalileoInavWord,
};
/// GLONASS navigation string verification and decoding.
pub use crate::formats::glonass_navigation_decode::{
    decode_glonass_broadcast_navigation_frame, decode_glonass_navigation_string,
    GlonassNavigationFrameRejection, GlonassNavigationFrameRejectionReason,
    GlonassNavigationString, GlonassNavigationStringRejection,
    GlonassNavigationStringRejectionReason, GlonassStringParitySummary,
};
/// Format parsing and output.
pub use crate::formats::lnav_bits::{
    align_gps_l1ca_lnav_from_prompt, align_gps_l1ca_lnav_subframes, bit_sync_from_prompt,
    decode_gps_l1ca_lnav_from_prompt, decode_gps_l1ca_lnav_subframes, decode_how_word,
    decode_subframes, decode_tlm_word, demodulate_gps_l1ca_navigation_bits,
    ephemerides_from_decoded_gps_l1ca_lnav, summarize_word_parity, GpsL1CaHowWord,
    GpsL1CaLnavAlignment, GpsL1CaLnavDecodedStream, GpsL1CaLnavDecodedSubframe,
    GpsL1CaLnavSubframeAlignment, GpsL1CaNavigationBit, GpsL1CaNavigationBits, GpsL1CaTlmWord,
    GpsL1CaWordParitySummary,
};
pub use crate::formats::lnav_decode::{
    decode_rawephem_hex, decode_subframe1_clock, decode_subframe2_orbit, decode_subframe3_orbit,
    decode_subframe_hex, GpsL1CaLnavEphemerisRejection, GpsL1CaLnavEphemerisRejectionReason,
    GpsL1CaLnavSubframe1Clock, GpsL1CaLnavSubframe2Orbit, GpsL1CaLnavSubframe3Orbit,
};
/// Precise product providers (SP3/CLK) and fallbacks.
pub use crate::formats::precise_products::{
    BroadcastProductsProvider, ProductDiagnostics, Products, ProductsProvider,
};
pub use crate::formats::rinex::{
    parse_rinex_broadcast_navigation, parse_rinex_nav, parse_rinex_obs_header,
    write_rinex_broadcast_navigation, write_rinex_nav, write_rinex_obs,
};
pub use crate::formats::rinex_obs::{
    parse_rinex_gps_observation_dataset, RinexGpsObservationChannel, RinexGpsObservationDataset,
};
/// Precise product parsing helpers.
pub use crate::formats::{
    clk::{ClkInterpolationSummary, ClkProvider},
    sp3::{Sp3InterpolationSummary, Sp3Provider},
};
/// Linear algebra helper.
pub use crate::linalg::Matrix;
/// Atmospheric model scaffolding.
pub use crate::models::atmosphere::{
    IonosphereModel, KlobucharCoefficients, KlobucharModel, SaastamoinenModel, TroposphereModel,
};
/// BeiDou broadcast navigation and satellite state helpers.
pub use crate::orbits::beidou::{
    beidou_earth_rotation_correction, beidou_navigation_age, beidou_satellite_clock_correction_b1i,
    is_beidou_navigation_valid, sat_state_beidou_b1i, sat_state_beidou_b1i_at_receive_time,
    sat_state_beidou_b1i_from_observation, select_best_beidou_navigation,
    BeidouBroadcastNavigationData, BeidouClockCorrection, BeidouEarthRotationCorrection,
    BeidouEphemeris, BeidouIonosphericCorrection, BeidouNavigationAge, BeidouSatState,
    BeidouSatelliteClockCorrection, BeidouSignalHealth, BeidouSystemTime,
};
/// Ephemeris provider traits and helpers.
pub use crate::orbits::ephemeris::{CsvEphemerisProvider, Ephemeris, EphemerisProvider};
/// Broadcast ephemeris and satellite state.
pub use crate::orbits::galileo::{
    galileo_earth_rotation_correction, galileo_navigation_age,
    galileo_satellite_clock_correction_e1, is_galileo_navigation_valid, sat_state_galileo_e1,
    sat_state_galileo_e1_at_receive_time, sat_state_galileo_e1_from_observation,
    select_best_galileo_navigation, GalileoBroadcastNavigationData, GalileoClockCorrection,
    GalileoEarthRotationCorrection, GalileoEphemeris, GalileoIonosphericCorrection,
    GalileoIonosphericDisturbanceFlags, GalileoNavigationAge, GalileoSatState,
    GalileoSatelliteClockCorrection, GalileoSignalHealth, GalileoSystemTime,
};
/// GLONASS navigation domain types.
pub use crate::orbits::glonass::{
    glonass_earth_rotation_correction, glonass_gps_minus_glonass_s, glonass_navigation_age,
    glonass_satellite_clock_correction, glonass_satellite_type_from_word,
    is_glonass_navigation_valid, sat_state_glonass_l1, sat_state_glonass_l1_at_receive_time,
    sat_state_glonass_l1_from_observation, select_best_glonass_navigation,
    semicircles_to_radians, GlonassAlmanacEntry, GlonassAlmanacTimeData,
    GlonassBroadcastNavigationFrame, GlonassEarthRotationCorrection, GlonassFrameTime,
    GlonassImmediateHealth, GlonassImmediateNavigationData, GlonassNavigationAge,
    GlonassSatState, GlonassSatelliteClockCorrection, GlonassSatelliteType, GlonassStateVector,
    GlonassSystemTime,
};
/// Broadcast ephemeris and satellite state.
pub use crate::orbits::gps::{
    gps_earth_rotation_correction, gps_ephemeris_age, gps_satellite_clock_correction,
    is_ephemeris_valid, sat_state_gps_l1ca, sat_state_gps_l1ca_at_receive_time,
    sat_state_gps_l1ca_from_observation, select_best_ephemeris, GpsBroadcastNavigationData,
    GpsEarthRotationCorrection, GpsEphemeris, GpsEphemerisAge, GpsSatState,
    GpsSatelliteClockCorrection,
};
/// Time helpers.
pub use crate::time::{gps_time_from_utc, gps_week_rollover, normalize_tow};

/// Navigation engine trait boundary.
pub trait NavEngine {
    /// Update navigation solution with new observation epoch.
    fn update(
        &mut self,
        obs: &bijux_gnss_core::api::ObsEpochV1,
    ) -> bijux_gnss_core::api::NavSolutionEpochV1;
}
