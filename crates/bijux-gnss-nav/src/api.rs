//! Public API for bijux-gnss-nav.

/// Versioned artifact wrappers.
///
/// Broadcast ephemeris list artifact v1.
pub type GpsEphemerisV1 = bijux_gnss_core::api::ArtifactV1<Vec<crate::orbits::gps::GpsEphemeris>>;
/// PPP solution epoch artifact v1.
pub type PppEpochV1 =
    bijux_gnss_core::api::ppp::PppEpochV1<crate::estimation::ppp::config::PppSolutionEpoch>;
pub use crate::corrections::atmosphere::{clamp_ztd, AtmosphereConfig};
pub use crate::corrections::biases::{
    iono_free_code_bias_m, CodeBiasProvider, PhaseBiasProvider, SignalCodeBiases, ZeroBiases,
};
pub use crate::corrections::broadcast_group_delay::{
    beidou_broadcast_group_delay_code_bias_m, galileo_broadcast_group_delay_code_bias_m,
    gps_broadcast_group_delay_code_bias_m, BroadcastGroupDelayBiases,
};
pub use crate::corrections::broadcast_ionosphere_residuals::{
    galileo_broadcast_ionosphere_residuals_from_obs_epochs,
    gps_broadcast_ionosphere_residuals_from_obs_epochs, summarize_broadcast_ionosphere_residuals,
    BroadcastIonosphereResidualObservation, BroadcastIonosphereResidualStats,
    BroadcastIonosphereResidualSummary,
};
/// Corrections and combination helpers.
pub use crate::corrections::combinations::combinations_from_obs_epochs;
pub use crate::corrections::geometry_free::{
    geometry_free_diagnostics_from_obs_epochs, GeometryFreeEvent, GeometryFreeObservation,
    GeometryFreeThresholds,
};
pub use crate::corrections::iono_free_code::{
    iono_free_code_from_obs_epochs, iono_free_code_from_obs_epochs_with_biases,
    IonoFreeCodeObservation,
};
pub use crate::corrections::iono_free_phase::{
    iono_free_phase_from_obs_epochs, IonoFreePhaseObservation,
};
pub use crate::corrections::measured_ionosphere::{
    measured_ionosphere_from_obs_epochs, MeasuredIonosphereObservation,
};
pub use crate::corrections::melbourne_wubbena::{
    melbourne_wubbena_diagnostics_from_obs_epochs, MelbourneWubbenaEvent,
    MelbourneWubbenaObservation, MelbourneWubbenaThresholds,
};
pub use crate::corrections::narrow_lane::{narrow_lane_from_obs_epochs, NarrowLaneObservation};
pub use crate::corrections::{compute_corrections, CorrectionContext, Corrections};
pub use crate::estimation::ekf::models::{
    AmbiguityManager, CarrierPhaseMeasurement, DopplerMeasurement, InterSystemBiasManager,
    NavClockModel, ProcessNoiseConfig, PseudorangeMeasurement, StaticNavClockModel,
};
pub use crate::estimation::ekf::state::{Ekf, EkfConfig, MeasurementKind};
/// EKF interface and measurements.
pub use crate::estimation::ekf::statistics::InnovationConsistencyConfig;
pub use crate::estimation::ekf::traits::MeasurementModel;
/// Position solver (least squares) and helpers.
pub use crate::estimation::position::filter::{
    PositionFilter, PositionFilterConfig, PositionFilterEpoch, PositionFilterIndices,
    PositionFilterMotionClass, PositionFilterMotionModel, PositionFilterProcessNoise,
    PositionFilterStaticPositionModel,
};
pub use crate::estimation::position::integrity::{
    clock_anomaly::{
        advance_satellite_clock_suspect_streak, detect_satellite_clock_anomaly,
        SatelliteClockAnomaly,
    },
    clock_consistency::{
        detect_constellation_clock_inconsistencies, ConstellationClockInconsistency,
    },
    code_doppler_anomaly::{detect_common_code_doppler_anomaly, CommonCodeDopplerAnomaly},
    replay_timing::{detect_replay_timing_anomaly, ReplayTimingAnomaly},
    residual_temporal_correlation::{
        advance_residual_whiteness_suspect_streak, classify_residual_temporal_correlation,
        detect_residual_temporal_correlation, residual_temporal_correlation_is_persistent,
        ResidualTemporalCorrelation, ResidualTemporalCorrelationEvidence,
    },
};
pub use crate::estimation::position::navigation::{
    position_satellite_state_from_observation, PositionSatelliteState,
};
pub use crate::estimation::position::navigation_filter::{
    NavigationFilter, NavigationFilterConfig, NavigationFilterThresholds,
};
pub use crate::estimation::position::raim::{
    formal_protection_levels, PositionProtectionLevels, RaimFaultDetection,
    RaimFaultDetectionStatus, RaimFaultExclusion, RaimSolutionSeparationCheck,
    RaimSolutionSeparationSubset,
};
pub use crate::estimation::position::runtime::{
    supported_positioning_signal, supports_positioning_signal, EkfState, NavigationEngine,
    NavigationState, PositionConstellationPolicy, PositionRuntime, PositionRuntimeConfig,
    PositionRuntimeThresholds, PositionRuntimeWeightingConfig,
};
pub use crate::estimation::position::solution_smoother::{
    PositionSolutionSmoother, PositionSolutionSmootherConfig, PositionSolutionSmootherEpoch,
};
pub use crate::estimation::position::solver::{
    ecef_to_enu, ecef_to_geodetic, elevation_azimuth_deg, geodetic_to_ecef,
    position_broadcast_navigation_from_beidou_navigations,
    position_broadcast_navigation_from_galileo_navigations,
    position_broadcast_navigation_from_glonass_frames,
    position_broadcast_navigation_from_gps_ephemerides, position_dops_from_satellite_positions,
    position_measurement_weight, position_observation_has_valid_satellite_time,
    position_observations_from_epoch, weight_from_cn0, weight_from_elevation,
    weight_from_pseudorange_sigma, ImpossibleGeometryEvidence, PositionBroadcastNavigation,
    PositionDops, PositionFilterDivergenceReason, PositionObservation, PositionRobustWeighting,
    PositionSolution, PositionSolveRefusal, PositionSolveRefusalKind, PositionSolver,
    PositionWeightingModel, ReplayTimingAnomalyEvidence, WeightingConfig,
};
pub use crate::estimation::position::trajectory::{
    trajectory_reconstruction_report, TrajectoryReconstructionError, TrajectoryReconstructionInput,
    TrajectoryReconstructionReport, TrajectoryReconstructionSample,
};
/// PPP configuration and filter.
pub use crate::estimation::ppp::config::{
    PppArMode, PppConfig, PppConvergenceConfig, PppFilter, PppProcessNoise, PppSolutionEpoch,
    PppTroposphereSource,
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
/// RTK execution, epoch alignment, and contract-adapter helpers.
pub use crate::estimation::rtk::execution::{
    build_dd, build_dd_per_constellation, build_sd, choose_ref_sat,
    choose_ref_sat_per_constellation, dd_covariance, double_difference, innovation_diagnostics,
    los_unit, single_difference, solve_baseline_dd, solve_float_baseline_dd, AlignmentDiagnostic,
    AlignmentReport, BaselineConfig, DdCovarianceModel, DdObservation, EpochAligner, RefSatPolicy,
    RefSatSelector, SdObservation, SolutionSeparation,
};
/// RTK baseline-quality, residual, and fix-guard helpers.
pub use crate::estimation::rtk::quality::{
    apply_fix_hold, baseline_from_ecef, dd_residual_metrics, enu_to_ecef,
    evaluate_rtk_fixed_baseline_guard, jitter_summary, sd_residual_metrics, solution_separation,
    BaselineSolution, JitterSummary, RtkBaselineQuality, RtkFixedBaselineGuardDecision,
    RtkFixedBaselineGuardPolicy, RtkPrecision,
};
/// RTK single-difference helpers.
pub use crate::estimation::rtk::single_difference::{
    choose_rtk_single_difference_reference_signal,
    choose_rtk_single_difference_reference_signals_by_constellation,
    rtk_single_difference_residual_metrics,
    rtk_single_difference_residual_metrics_with_antenna_corrections,
    rtk_single_differences_from_obs_epochs, RtkSingleDifferenceObservation,
    RtkSingleDifferenceResidualMetrics,
};
/// RTK double-difference helpers.
pub use crate::estimation::rtk::{
    antenna::RtkAntennaCorrectionConfig,
    double_difference::{
        rtk_double_difference_residual_metrics,
        rtk_double_difference_residual_metrics_with_antenna_corrections,
        rtk_double_differences_by_constellation, rtk_double_differences_from_single_differences,
        RtkDoubleDifferenceObservation, RtkDoubleDifferenceResidualMetrics,
    },
};
/// Advanced PPP/RTK solution claim and refusal surfaces.
pub use crate::estimation::solution_claims::{
    apply_downgrade_policy, evaluate_prerequisites, evaluate_solution_evidence,
    support_status_matrix, AdvancedClaimDecision, AdvancedMaturity, AdvancedMode,
    AdvancedPrereqDecision, AdvancedPrerequisites, AdvancedRefusalClass, AdvancedSolutionArtifact,
    AdvancedSolutionClaim, AdvancedSolutionEvidence, AdvancedSolutionMeasurements,
    AdvancedSolutionProvenance, AdvancedSupportMatrix, AdvancedSupportRow, AmbiguityStateArtifact,
    CorrectionInputArtifact, ExecutionArtifact, ExecutionStatus, ADVANCED_SUPPORT_MATRIX_VERSION,
};
pub use crate::formats::antex::{
    parse_antex_receiver_calibrations, parse_antex_satellite_calibrations,
};
/// BeiDou B1I D1 navigation decoding.
pub use crate::formats::beidou_b1i_navigation_decode::{
    decode_beidou_b1i_clock_subframe, decode_beidou_b1i_ephemeris_1_subframe,
    decode_beidou_b1i_ephemeris_2_subframe, decode_beidou_b1i_subframe,
    decode_beidou_broadcast_navigation_data,
    decode_beidou_broadcast_navigation_data_with_reference_week, BeidouD1BatchRejection,
    BeidouD1BatchRejectionReason, BeidouD1ClockSubframe, BeidouD1Ephemeris1Subframe,
    BeidouD1Ephemeris2Subframe, BeidouD1Subframe, BeidouD1SubframeRejection,
    BeidouD1SubframeRejectionReason,
};
pub use crate::formats::bias_sinex::{BiasSinexProvider, BiasSinexWindow};
/// GPS CNAV packet decoding.
pub use crate::formats::cnav_decode::{
    decode_gps_cnav_broadcast_navigation, decode_gps_cnav_clock_correction_message,
    decode_gps_cnav_ephemeris_message, decode_gps_cnav_message, decode_gps_cnav_message_hex,
    decode_gps_cnav_orbit_message, GpsCnavBroadcastNavigationData, GpsCnavClockCorrection,
    GpsCnavClockCorrectionMessage, GpsCnavCommon, GpsCnavEphemeris, GpsCnavEphemerisMessage,
    GpsCnavGroupDelayCorrection, GpsCnavIonosphericCorrection, GpsCnavMessage,
    GpsCnavMessageRejection, GpsCnavMessageRejectionReason, GpsCnavNavigationRejection,
    GpsCnavNavigationRejectionReason, GpsCnavNonElevationAccuracy, GpsCnavOrbitMessage,
    GpsCnavSignalHealth,
};
/// Galileo F/NAV page decoding.
pub use crate::formats::galileo_fnav_decode::{
    decode_galileo_fnav_page, GalileoFnavPage, GalileoFnavPageRejection,
    GalileoFnavPageRejectionReason,
};
/// Format parsing and output.
pub use crate::formats::galileo_inav_decode::{
    decode_galileo_broadcast_navigation_data, decode_galileo_broadcast_navigation_data_payloads,
    decode_galileo_broadcast_navigation_data_payloads_with_reference_week,
    decode_galileo_broadcast_navigation_data_with_reference_week, decode_galileo_inav_clock_word,
    decode_galileo_inav_ephemeris_1_word, decode_galileo_inav_ephemeris_2_word,
    decode_galileo_inav_ephemeris_3_word, decode_galileo_inav_status_word,
    decode_galileo_inav_word, decode_galileo_inav_word_bytes, decode_galileo_inav_word_hex,
    GalileoInavBatchRejection, GalileoInavBatchRejectionReason, GalileoInavClockWord,
    GalileoInavEphemeris1Word, GalileoInavEphemeris2Word, GalileoInavEphemeris3Word,
    GalileoInavStatusWord, GalileoInavWord,
};
/// GLONASS navigation string verification and decoding.
pub use crate::formats::glonass_navigation_decode::{
    decode_glonass_broadcast_navigation_frame,
    decode_glonass_broadcast_navigation_frame_with_reference_day, decode_glonass_navigation_string,
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
    parse_rinex_beidou_observation_dataset, parse_rinex_galileo_observation_dataset,
    parse_rinex_gps_observation_dataset, RinexBeidouObservationChannel,
    RinexBeidouObservationDataset, RinexCodeBiasState, RinexCodeBiasStatus,
    RinexGalileoObservationChannel, RinexGalileoObservationDataset, RinexGpsObservationChannel,
    RinexGpsObservationDataset,
};
/// Precise product parsing helpers.
pub use crate::formats::{
    clk::{ClkInterpolationSummary, ClkProvider},
    sp3::{Sp3InterpolationSummary, Sp3Provider},
};
/// Linear algebra helper.
pub use crate::linalg::Matrix;
pub use crate::models::antenna::{
    canonical_receiver_antenna_type, receiver_antenna_range_correction_m,
    satellite_antenna_range_correction_m, satellite_band_from_antex_frequency,
    ReceiverAntennaCalibration, ReceiverAntennaCalibrations, ReceiverPhaseCenterOffset,
    SatelliteAntennaCalibration, SatelliteAntennaCalibrations, SatellitePhaseCenterOffset,
};
/// Atmospheric delay models and coefficients.
pub use crate::models::atmosphere::{
    IonosphereModel, KlobucharCoefficients, KlobucharModel, SaastamoinenModel, TroposphereModel,
};
pub use crate::models::nequick::model::GalileoNequickModel;
pub use crate::models::ocean_tide_loading::{
    OceanTideConstituent, OceanTideLoadingConstituent, OceanTideLoadingModel,
};
pub use crate::models::solid_earth_tide::SolidEarthTideModel;
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
    sat_state_glonass_l1_from_observation, select_best_glonass_navigation, semicircles_to_radians,
    GlonassAlmanacEntry, GlonassAlmanacTimeData, GlonassBroadcastNavigationFrame,
    GlonassEarthRotationCorrection, GlonassFrameTime, GlonassImmediateHealth,
    GlonassImmediateNavigationData, GlonassNavigationAge, GlonassSatState,
    GlonassSatelliteClockCorrection, GlonassSatelliteType, GlonassStateVector, GlonassSystemTime,
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
pub use crate::time::{
    beidou_to_gps_with_offset, format_utc_civil_time, galileo_to_gps_with_offset,
    glonass_to_gps_with_offset, glonass_to_utc_with_offset, gps_time_from_utc,
    gps_to_beidou_with_offset, gps_to_galileo_with_offset, gps_to_glonass_with_offset,
    gps_to_utc_civil_with_offset, gps_to_utc_with_offset, gps_week_rollover, normalize_tow,
    parse_utc_civil_time, resolve_beidou_week_rollover, resolve_galileo_week_rollover,
    resolve_glonass_day_number, resolve_gps_week_rollover, resolve_truncated_week,
    tai_to_utc_civil_with_offset, tai_to_utc_with_offset, utc_civil_to_gps_with_offset,
    utc_civil_to_tai_with_offset, utc_to_glonass_with_offset, utc_to_gps_with_offset,
    utc_to_tai_with_offset, BeidouTime, GalileoGpsTimeOffset, GalileoTime, GlonassDayResolution,
    GlonassTime, GlonassUtcOffset, GnssTimeSystem, RolloverResolutionError, TimeConversion,
    TimeOffsetEvidence, TimeOffsetSource, UtcCivilTime, UtcCivilTimeError, WeekRolloverResolution,
};

/// Navigation engine trait boundary.
pub trait NavEngine {
    /// Update navigation solution with new observation epoch.
    fn update(
        &mut self,
        obs: &bijux_gnss_core::api::ObsEpochV1,
    ) -> bijux_gnss_core::api::NavSolutionEpochV1;
}
