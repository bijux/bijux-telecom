#![allow(missing_docs)]

use std::collections::BTreeMap;
use std::f32::consts::TAU;

use num_complex::Complex;

use crate::reference_validation::{reference_ecef, ValidationReferenceEpoch};
use bijux_gnss_core::api::{
    ecef_to_enu, ecef_to_geodetic, stats, AcqHypothesis, AcqResult, Constellation,
    GlonassFrequencyChannel, GpsTime, Hertz, Meters, NavQualityFlag, NavSolutionEpoch, ObsEpoch,
    ObservationStatus, ReceiverSampleTrace, SampleClock, SampleTime, SamplesFrame, SatId,
    Seconds, SigId, SignalBand, SignalCode, SignalDelayAlignment, SignalSpec, SolutionStatus,
    SolutionValidity,
};
use bijux_gnss_signal::api::SignalSource;


use crate::engine::receiver_config::ReceiverPipelineConfig;
use crate::io::data::SampleSourceError;
use crate::pipeline::doppler::carrier_hz_from_doppler_hz;
use bijux_gnss_nav::api::{sat_state_gps_l1ca, GpsEphemeris};
use bijux_gnss_signal::api::{
    first_order_ionosphere_code_delay_m, receiver_search_code_phase_samples, samples_per_code,
    IqSampleFormat, RawIqMetadata,
};
use serde::{Deserialize, Serialize};

const SYNTHETIC_IQ_TRUTH_SCHEMA_VERSION: u32 = 4;
const SYNTHETIC_GNSS_ACCURACY_ARTIFACT_SCHEMA_VERSION: u32 = 1;
const GPS_L1_CA_NAV_BIT_PERIOD_S: f64 = 0.02;
const SYNTHETIC_COMPLEX_NOISE_POWER: f64 = 1.0;
const SYNTHETIC_NOISE_STD_PER_COMPONENT: f32 = std::f32::consts::FRAC_1_SQRT_2;
const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;

include!("synthetic/scenario.rs");
include!("synthetic/ionosphere.rs");
include!("synthetic/observation_truth.rs");
include!("synthetic/stage_accuracy.rs");
include!("synthetic/pvt_profile_types.rs");
include!("synthetic/observation_validation_types.rs");
include!("synthetic/gnss_artifact_types.rs");
include!("synthetic/artifact_validation.rs");
include!("synthetic/artifact_build.rs");
include!("synthetic/pvt_profile_cases.rs");
include!("synthetic/pvt_profiles_signal_geometry.rs");
include!("synthetic/pvt_profiles_motion_clock.rs");
include!("synthetic/pvt_profiles_time_accuracy.rs");
include!("synthetic/pvt_truth.rs");
include!("synthetic/acquisition_validation_types.rs");
include!("synthetic/tracking_validation_types.rs");
include!("synthetic/capture_tracking.rs");
include!("synthetic/acquisition_validation.rs");
include!("synthetic/sensitivity_profiles.rs");
include!("synthetic/signal_generation.rs");

#[cfg(test)]
mod tests {
    use super::{
        build_iq16_capture_bundle, build_truth_bundle, expected_acquisition_code_phase_samples,
        expected_acquisition_code_phase_samples_f64, generate_l1_ca, generate_l1_ca_multi,
        generate_l1_ca_with_doppler_ramp, generate_l1_ca_with_fades,
        generate_l1_ca_with_phase_windows, measure_noise_only_acquisition_false_alarm_rate,
        measure_noise_only_acquisition_false_alarm_rates,
        measure_truth_guided_acquisition_detection_probability,
        measure_truth_guided_acquisition_detection_rate, measure_truth_guided_tracking_lock_rate,
        nav_bit_index_at_time_s, nav_bit_sign_at_time_s, signal_amplitude_from_cn0,
        summarize_observation_errors, summarize_truth_guided_accuracy_cn0_profile,
        summarize_truth_guided_pvt_clock_profile, summarize_truth_guided_pvt_cn0_profile,
        summarize_truth_guided_pvt_constellation_geometry_profile,
        summarize_truth_guided_pvt_geometry_profile, summarize_truth_guided_pvt_motion_profile,
        summarize_truth_guided_pvt_multipath_profile, summarize_truth_guided_pvt_time_profile,
        synthetic_tracking_sensitivity_report, truth_guided_receiver_accuracy_budgets,
        validate_acquisition_accuracy_budget, validate_pvt_accuracy_budget,
        validate_truth_guided_acquisition_code_phase,
        validate_truth_guided_acquisition_code_phase_refinement,
        validate_truth_guided_acquisition_coherent_integration,
        validate_truth_guided_acquisition_doppler,
        validate_truth_guided_acquisition_receiver_clock_offset,
        validate_truth_guided_acquisition_sample_rates, validate_truth_guided_cn0,
        validate_truth_guided_pvt_table, wrapped_code_phase_error_samples,
        wrapped_code_phase_error_samples_f64, SatState, SyntheticAccuracyCn0ProfileReport,
        SyntheticAcquisitionDetectionRateCase, SyntheticAcquisitionDetectionRatePoint,
        SyntheticAcquisitionDetectionRateReport, SyntheticAcquisitionFalseAlarmRateCase,
        SyntheticAcquisitionSampleRateValidationCase, SyntheticAcquisitionTruthTableReport,
        SyntheticAcquisitionTruthTableSatellite, SyntheticDopplerRampParams, SyntheticFadeWindow,
        SyntheticNavBitMode, SyntheticPhaseWindow, SyntheticPvtAccuracyEpoch,
        SyntheticPvtAccuracyReport, SyntheticPvtClockProfileCase, SyntheticPvtCn0ProfileCase,
        SyntheticPvtCn0ProfilePoint, SyntheticPvtCn0ProfileReport,
        SyntheticPvtConstellationGeometryProfileCase,
        SyntheticPvtConstellationGeometryProfileReport, SyntheticPvtGeometryProfileCase,
        SyntheticPvtGeometryProfileReport, SyntheticPvtMotionProfileCase,
        SyntheticPvtMotionProfileReport, SyntheticPvtMultipathProfileCase,
        SyntheticPvtMultipathProfileReport, SyntheticPvtTimeProfileCase,
        SyntheticPvtTimeProfileReport, SyntheticPvtTimeTrend, SyntheticPvtTruthReferenceEpoch,
        SyntheticPvtTruthTableClockBias, SyntheticPvtTruthTableDop, SyntheticPvtTruthTableEcef,
        SyntheticPvtTruthTableEnuError, SyntheticPvtTruthTableEpoch,
        SyntheticPvtTruthTableGeodetic, SyntheticPvtTruthTableReport, SyntheticScenario,
        SyntheticSignalParams, SyntheticSignalSource, SyntheticTrackingLockRateCase,
        SyntheticTrackingLockRatePoint, SyntheticTrackingLockRateReport,
        SyntheticTrackingSensitivityTrial, SyntheticTrackingTruthTableEpoch,
        SyntheticTrackingTruthTableReport, SyntheticTrackingTruthTableSatellite,
        SPEED_OF_LIGHT_MPS, SYNTHETIC_COMPLEX_NOISE_POWER, SYNTHETIC_NOISE_STD_PER_COMPONENT,
    };
    use crate::engine::receiver_config::ReceiverPipelineConfig;
    use crate::reference_validation::ValidationReferenceEpoch;
    use bijux_gnss_core::api::{
        ecef_to_geodetic, lla_to_ecef, Constellation, Cycles, Epoch, FreqHz, Hertz, LockFlags,
        Meters, NavLifecycleState, NavQualityFlag, NavSolutionEpoch, NavUncertaintyClass, ObsEpoch,
        ObsMetadata, ObsSatellite, ObservationEpochDecision, ObservationStatus, ReceiverRole,
        ReceiverSampleTrace, SampleTime, SamplesFrame, SatId, Seconds, SigId, SignalBand,
        SignalCode, SignalSpec, SolutionStatus, SolutionValidity,
        NAV_OUTPUT_STABILITY_SIGNATURE_VERSION, NAV_SOLUTION_MODEL_VERSION,
    };
    use bijux_gnss_nav::api::GpsEphemeris;
    use bijux_gnss_signal::api::{
        advance_code_phase_seconds, sample_ca_code, samples_per_code, IqSampleFormat, Prn,
        RawIqMetadata, SignalSource,
    };
    use num_complex::Complex;

    const RECEIVER_PHASE_TOLERANCE_SAMPLES: f64 = 1e-6;

    include!("synthetic/tests/foundation.rs");
    include!("synthetic/tests/pvt_profiles_signal_conditions.rs");
    include!("synthetic/tests/pvt_profiles_motion_timing.rs");
    include!("synthetic/tests/profile_fixtures.rs");
    include!("synthetic/tests/signal_model.rs");
    include!("synthetic/tests/validation_detection_reports.rs");
    include!("synthetic/tests/validation_frequency_sampling.rs");
}
