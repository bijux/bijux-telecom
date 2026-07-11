#![allow(missing_docs)]

use std::collections::BTreeMap;
use std::f32::consts::TAU;

use num_complex::Complex;

use bijux_gnss_core::api::{
    ecef_to_enu, ecef_to_geodetic, reference_ecef, stats, AcqHypothesis, AcqResult, Constellation,
    Hertz, NavQualityFlag, NavSolutionEpoch, ObsEpoch, ObservationStatus, ReceiverSampleTrace,
    SampleClock, SampleTime, SamplesFrame, SatId, Seconds, SignalBand, SolutionStatus,
    SolutionValidity, ValidationReferenceEpoch,
};
use bijux_gnss_signal::api::SignalSource;

use crate::engine::receiver_config::ReceiverPipelineConfig;
use crate::io::data::SampleSourceError;
use crate::pipeline::doppler::carrier_hz_from_doppler_hz;
use bijux_gnss_nav::api::{sat_state_gps_l1ca, GpsEphemeris};
use bijux_gnss_signal::api::{
    advance_code_phase_seconds, code_value_at_phase, generate_ca_code, samples_per_code,
    IqSampleFormat, Prn, RawIqMetadata,
};
use serde::{Deserialize, Serialize};

const SYNTHETIC_IQ_TRUTH_SCHEMA_VERSION: u32 = 3;
const SYNTHETIC_GNSS_ACCURACY_ARTIFACT_SCHEMA_VERSION: u32 = 1;
const GPS_L1_CA_NAV_BIT_PERIOD_S: f64 = 0.02;
const SYNTHETIC_COMPLEX_NOISE_POWER: f64 = 1.0;
const SYNTHETIC_NOISE_STD_PER_COMPONENT: f32 = std::f32::consts::FRAC_1_SQRT_2;
const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct SyntheticSignalParams {
    pub sat: SatId,
    pub doppler_hz: f64,
    pub code_phase_chips: f64,
    pub carrier_phase_rad: f64,
    pub cn0_db_hz: f32,
    pub data_bit_flip: bool,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct SyntheticDopplerRampParams {
    pub signal: SyntheticSignalParams,
    pub doppler_rate_hz_per_s: f64,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct SyntheticFadeWindow {
    /// Fade start time in seconds, inclusive.
    pub start_s: f64,
    /// Fade end time in seconds, exclusive.
    pub end_s: f64,
    /// Multiplicative scale applied to the synthetic signal inside the window.
    pub signal_scale: f32,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct SyntheticPhaseWindow {
    /// Phase-offset start time in seconds, inclusive.
    pub start_s: f64,
    /// Phase-offset end time in seconds, exclusive.
    pub end_s: f64,
    /// Additive carrier phase offset applied inside the window.
    pub phase_offset_rad: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SyntheticScenario {
    pub sample_rate_hz: f64,
    pub intermediate_freq_hz: f64,
    #[serde(default)]
    pub receiver_clock_frequency_bias_hz: f64,
    pub duration_s: f64,
    pub seed: u64,
    pub satellites: Vec<SyntheticSignalParams>,
    #[serde(default)]
    pub ephemerides: Vec<GpsEphemeris>,
    #[serde(default)]
    pub id: String,
}

/// Deterministic navigation-bit schedule used by a synthetic signal.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum SyntheticNavBitMode {
    /// Keep the navigation-bit sign positive for the full capture.
    ConstantPositive,
    /// Alternate the bit sign every 20 ms, starting positive at sample zero.
    AlternatingGpsLnav20ms,
}

/// Sample-aligned navigation-bit truth interval.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticNavBitSegment {
    /// Inclusive segment start sample.
    pub start_sample: u64,
    /// Exclusive segment end sample.
    pub end_sample: u64,
    /// Segment start time in seconds.
    pub start_s: f64,
    /// Segment end time in seconds.
    pub end_s: f64,
    /// Navigation-bit sign applied over this interval.
    pub bit: i8,
}

/// Per-satellite truth carried alongside a synthetic IQ export.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticSatelliteTruth {
    /// Satellite identifier.
    pub sat: SatId,
    /// Injected Doppler shift in Hz.
    pub doppler_hz: f64,
    /// Injected code phase at sample zero, in chips.
    pub code_phase_chips: f64,
    /// Injected carrier phase at sample zero, in radians.
    pub carrier_phase_rad: f64,
    /// Injected carrier-to-noise density ratio in dB-Hz.
    pub cn0_db_hz: f32,
    /// Complex sample amplitude before additive noise and output scaling.
    pub signal_amplitude: f32,
    /// Deterministic navigation-bit model used for this signal.
    pub nav_bit_mode: SyntheticNavBitMode,
    /// Sample-aligned navigation-bit truth intervals.
    pub nav_bit_segments: Vec<SyntheticNavBitSegment>,
}

/// Machine-readable truth for an exported synthetic IQ capture.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticIqTruthBundle {
    /// Truth schema version.
    pub schema_version: u32,
    /// Stable scenario identifier for this capture.
    pub scenario_id: String,
    /// Deterministic seed used for the synthetic noise realization.
    pub seed: u64,
    /// Output sample format.
    pub sample_format: IqSampleFormat,
    /// Output sample rate in Hz.
    pub sample_rate_hz: f64,
    /// Output intermediate frequency in Hz.
    pub intermediate_freq_hz: f64,
    /// Common receiver clock frequency bias added to every synthetic carrier, in Hz.
    pub receiver_clock_frequency_bias_hz: f64,
    /// Synthetic capture start timestamp in UTC.
    pub capture_start_utc: String,
    /// Output quantization depth in bits.
    pub quantization_bits: u8,
    /// Total capture duration in seconds.
    pub duration_s: f64,
    /// Total complex samples emitted into the file.
    pub sample_count: usize,
    /// Gaussian noise standard deviation applied to each I/Q component.
    pub noise_std_per_component: f32,
    /// Total noise power per complex sample before quantization.
    pub noise_power_per_complex_sample: f32,
    /// Peak absolute I/Q component before output scaling.
    pub peak_component_before_scaling: f32,
    /// Scale factor applied before quantization.
    pub output_scale_applied: f32,
    /// Per-satellite truth rows.
    pub satellites: Vec<SyntheticSatelliteTruth>,
}

/// Encoded synthetic capture bundle ready to write to disk.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticIqCaptureBundle {
    /// Raw interleaved IQ bytes in the declared output format.
    pub raw_iq_bytes: Vec<u8>,
    /// Sidecar metadata for the encoded raw IQ file.
    pub metadata: RawIqMetadata,
    /// Machine-readable truth for the emitted capture.
    pub truth: SyntheticIqTruthBundle,
}

/// Per-satellite C/N0 comparison between synthetic truth and receiver measurement.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticCn0ValidationSatellite {
    /// Satellite identifier.
    pub sat: SatId,
    /// Injected carrier-to-noise density ratio in dB-Hz.
    pub injected_cn0_db_hz: f32,
    /// Mean tracking-correlator estimate across the measured stable tracking window.
    pub measured_mean_cn0_dbhz: f64,
    /// Minimum tracking-correlator estimate across the measured stable tracking window.
    pub measured_min_cn0_dbhz: f64,
    /// Maximum tracking-correlator estimate across the measured stable tracking window.
    pub measured_max_cn0_dbhz: f64,
    /// Mean estimate minus injected truth, in dB-Hz.
    pub cn0_delta_db: f64,
    /// Injected complex amplitude before additive noise and output scaling.
    pub signal_amplitude: f32,
    /// Injected complex amplitude after the export scaling factor is applied.
    pub output_signal_amplitude: f32,
    /// Count of stable tracking epochs measured for this satellite.
    pub epochs_measured: usize,
    /// Whether the measured mean stayed within the requested tolerance.
    pub pass: bool,
}

/// Truth-guided C/N0 validation report for an exported synthetic IQ capture.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticCn0ValidationReport {
    /// Stable scenario identifier for this capture.
    pub scenario_id: String,
    /// Allowed absolute C/N0 error in dB-Hz.
    pub tolerance_db_hz: f64,
    /// Capture sample rate in Hz.
    pub sample_rate_hz: f64,
    /// Number of samples used per coherent receiver estimate.
    pub coherent_samples_per_epoch: usize,
    /// Coherent integration interval in seconds.
    pub coherent_integration_s: f64,
    /// Quantization depth declared in the truth bundle.
    pub quantization_bits: u8,
    /// Gaussian noise standard deviation applied to each I/Q component.
    pub noise_std_per_component: f32,
    /// Total noise power per complex sample before quantization.
    pub noise_power_per_complex_sample: f32,
    /// Scale factor applied before quantization.
    pub output_scale_applied: f32,
    /// Whether every measured satellite passed the requested tolerance.
    pub pass: bool,
    /// Per-satellite comparison rows.
    pub satellites: Vec<SyntheticCn0ValidationSatellite>,
}

/// Absolute synthetic reference used when comparing emitted observations against truth.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct SyntheticObservationTruthReference {
    /// Absolute receiver receive time at capture sample zero, in seconds.
    pub receive_time_s: f64,
    /// Receiver truth position in ECEF meters.
    pub receiver_ecef_m: [f64; 3],
}

/// Truth-guided value recorded for one observation observable.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticObservationTruthTableValue {
    /// Expected synthetic truth value for this observable.
    pub truth: Option<f64>,
    /// Measured receiver value for this observable.
    pub measured: f64,
    /// Reported one-sigma uncertainty for this observable, when the receiver exposes one.
    pub sigma: Option<f64>,
    /// Signed residual between the measured value and the truth value, when comparable.
    pub residual: Option<f64>,
}

/// Per-epoch observation truth-table row for synthetic validation.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticObservationTruthTableEpoch {
    /// Stable artifact identifier for the source observation epoch.
    pub artifact_id: String,
    /// Stable identity key for the source observation epoch.
    pub epoch_id: String,
    /// Receiver observation epoch index.
    pub epoch_index: u64,
    /// Absolute sample index at the start of the observation epoch.
    pub sample_index: u64,
    /// Observation status reported by the receiver.
    pub observation_status: ObservationStatus,
    /// Receiver reasons for degraded or rejected observation handling.
    pub observation_reject_reasons: Vec<String>,
    /// Pseudorange truth row in meters.
    pub pseudorange_m: SyntheticObservationTruthTableValue,
    /// Carrier-phase truth row in cycles.
    pub carrier_phase_cycles: SyntheticObservationTruthTableValue,
    /// Carrier-phase arc anchor for ambiguity alignment, when one exists.
    pub carrier_phase_arc_start_sample_index: Option<u64>,
    /// Ambiguity bias removed from the carrier-phase truth comparison, in cycles.
    pub carrier_phase_arc_bias_cycles: Option<f64>,
    /// Doppler truth row in Hz.
    pub doppler_hz: SyntheticObservationTruthTableValue,
    /// C/N0 truth row in dB-Hz.
    pub cn0_db_hz: SyntheticObservationTruthTableValue,
}

/// Per-satellite observation truth table for synthetic validation.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticObservationTruthTableSatellite {
    /// Satellite identifier.
    pub sat: SatId,
    /// Injected Doppler shift in Hz.
    pub injected_doppler_hz: f64,
    /// Expected measured Doppler after the receiver clock bias is applied, in Hz.
    pub expected_measured_doppler_hz: f64,
    /// Injected code phase at sample zero, in chips.
    pub injected_code_phase_chips: f64,
    /// Injected carrier-to-noise density ratio in dB-Hz.
    pub injected_cn0_db_hz: f32,
    /// Number of truth-table rows recorded for this satellite.
    pub epoch_count: usize,
    /// Number of carrier-phase arcs aligned before residual evaluation.
    pub carrier_phase_arcs_evaluated: usize,
    /// Reasons one or more observables could not be compared against synthetic truth.
    pub notes: Vec<String>,
    /// Per-epoch truth-table rows.
    pub epochs: Vec<SyntheticObservationTruthTableEpoch>,
}

/// Truth-guided observation truth table for a synthetic scenario.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticObservationTruthTableReport {
    /// Stable scenario identifier for this validation run.
    pub scenario_id: String,
    /// Capture sample rate in Hz.
    pub sample_rate_hz: f64,
    /// Hatch-smoothing window applied before observation comparison.
    pub hatch_window: u32,
    /// Absolute receive-time anchor used for geometric pseudorange truth.
    pub reference_receive_time_s: f64,
    /// Per-satellite truth-table rows.
    pub satellites: Vec<SyntheticObservationTruthTableSatellite>,
}

/// Synthetic truth reference for one PVT solution epoch.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SyntheticPvtTruthReferenceEpoch {
    /// Reference position and time for this navigation epoch.
    pub position: ValidationReferenceEpoch,
    /// Receiver clock bias truth in seconds for this navigation epoch.
    pub clock_bias_s: f64,
}

/// ECEF coordinates recorded in a synthetic PVT truth table.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct SyntheticPvtTruthTableEcef {
    /// X coordinate in meters.
    pub x_m: f64,
    /// Y coordinate in meters.
    pub y_m: f64,
    /// Z coordinate in meters.
    pub z_m: f64,
}

/// Geodetic coordinates recorded in a synthetic PVT truth table.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct SyntheticPvtTruthTableGeodetic {
    /// Latitude in degrees.
    pub latitude_deg: f64,
    /// Longitude in degrees.
    pub longitude_deg: f64,
    /// Altitude above the ellipsoid in meters.
    pub altitude_m: f64,
}

/// ENU error components recorded in a synthetic PVT truth table.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct SyntheticPvtTruthTableEnuError {
    /// East error in meters.
    pub east_m: f64,
    /// North error in meters.
    pub north_m: f64,
    /// Up error in meters.
    pub up_m: f64,
    /// Horizontal error magnitude in meters.
    pub horiz_m: f64,
    /// Vertical error magnitude in meters.
    pub vert_m: f64,
    /// 3D position error magnitude in meters.
    pub error_3d_m: f64,
}

/// Receiver clock-bias truth comparison recorded in a synthetic PVT truth table.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct SyntheticPvtTruthTableClockBias {
    /// Truth clock bias in seconds.
    pub truth_s: f64,
    /// Measured clock bias in seconds.
    pub measured_s: f64,
    /// Measured minus truth clock bias in seconds.
    pub error_s: f64,
    /// Truth clock bias in meters.
    pub truth_m: f64,
    /// Measured clock bias in meters.
    pub measured_m: f64,
    /// Measured minus truth clock bias in meters.
    pub error_m: f64,
}

/// DOP values recorded in a synthetic PVT truth table.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct SyntheticPvtTruthTableDop {
    /// Position dilution of precision.
    pub pdop: f64,
    /// Horizontal dilution of precision.
    pub hdop: Option<f64>,
    /// Vertical dilution of precision.
    pub vdop: Option<f64>,
    /// Geometric dilution of precision.
    pub gdop: Option<f64>,
    /// Time dilution of precision.
    pub tdop: Option<f64>,
}

/// Per-epoch synthetic PVT truth-table row.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticPvtTruthTableEpoch {
    /// Stable artifact identifier for the navigation solution epoch.
    pub artifact_id: String,
    /// Source observation epoch identifier used to produce this navigation solution.
    pub source_observation_epoch_id: String,
    /// Receiver navigation epoch index.
    pub epoch_index: u64,
    /// Receiver receive time for this solution, in seconds.
    pub receive_time_s: f64,
    /// Truth ECEF position.
    pub truth_ecef_m: SyntheticPvtTruthTableEcef,
    /// Measured ECEF position.
    pub measured_ecef_m: SyntheticPvtTruthTableEcef,
    /// Measured minus truth ECEF error, in meters.
    pub ecef_error_m: SyntheticPvtTruthTableEcef,
    /// Truth geodetic position.
    pub truth_geodetic: SyntheticPvtTruthTableGeodetic,
    /// Measured geodetic position.
    pub measured_geodetic: SyntheticPvtTruthTableGeodetic,
    /// ENU error components.
    pub enu_error_m: SyntheticPvtTruthTableEnuError,
    /// Clock-bias truth comparison.
    pub clock_bias: SyntheticPvtTruthTableClockBias,
    /// Residual RMS after the navigation solve, in meters.
    pub residual_rms_m: f64,
    /// Pre-fit residual RMS, in meters, when available.
    pub pre_fit_residual_rms_m: Option<f64>,
    /// Post-fit residual RMS, in meters, when available.
    pub post_fit_residual_rms_m: Option<f64>,
    /// Dilution-of-precision values reported for this solution.
    pub dop: SyntheticPvtTruthTableDop,
    /// Receiver solution status.
    pub solution_status: SolutionStatus,
    /// Receiver solution quality classification.
    pub solution_quality: NavQualityFlag,
    /// Receiver solution validity classification.
    pub solution_validity: SolutionValidity,
    /// Whether the receiver marked this navigation solution valid.
    pub valid: bool,
    /// Number of satellites carried in the solution artifact.
    pub sat_count: usize,
    /// Number of satellites used in the final solve.
    pub used_sat_count: usize,
    /// Number of rejected satellites reported by the receiver.
    pub rejected_sat_count: usize,
}

/// Truth-guided PVT truth table for a synthetic navigation run.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticPvtTruthTableReport {
    /// Stable scenario identifier for this validation run.
    pub scenario_id: String,
    /// Count of navigation solutions supplied to the builder.
    pub solution_count: usize,
    /// Count of rows with matching synthetic truth.
    pub matched_epoch_count: usize,
    /// Solution epochs that had no matching synthetic truth reference.
    pub unmatched_solution_epochs: Vec<u64>,
    /// Truth epochs that were not consumed by any navigation solution row.
    pub unused_reference_epochs: Vec<u64>,
    /// Per-epoch PVT truth-table rows.
    pub epochs: Vec<SyntheticPvtTruthTableEpoch>,
}

/// Hard accuracy threshold for truth-guided acquisition validation.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionAccuracyBudget {
    /// Maximum absolute Doppler error in Hz.
    pub max_doppler_error_hz: f64,
    /// Maximum wrapped code-phase error in samples.
    pub max_code_phase_error_samples: usize,
}

/// Hard accuracy threshold for truth-guided tracking validation.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct SyntheticTrackingAccuracyBudget {
    /// Maximum absolute carrier-frequency error in Hz.
    pub max_carrier_error_hz: f64,
    /// Maximum absolute Doppler error in Hz.
    pub max_doppler_error_hz: f64,
    /// Maximum wrapped code-phase error in samples.
    pub max_code_phase_error_samples: f64,
    /// Maximum absolute C/N0 error in dB-Hz.
    pub max_cn0_error_db_hz: f64,
}

/// Hard accuracy threshold for truth-guided observation validation.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct SyntheticObservationAccuracyBudget {
    /// Maximum absolute pseudorange error in meters.
    pub max_pseudorange_error_m: f64,
    /// Maximum absolute carrier-phase error in cycles.
    pub max_carrier_phase_error_cycles: f64,
    /// Maximum absolute Doppler error in Hz.
    pub max_doppler_error_hz: f64,
    /// Maximum absolute C/N0 error in dB-Hz.
    pub max_cn0_error_db_hz: f64,
}

/// Hard accuracy threshold for truth-guided PVT validation.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct SyntheticPvtAccuracyBudget {
    /// Maximum 3D position error in meters.
    pub max_position_error_3d_m: f64,
    /// Maximum absolute clock-bias error in meters.
    pub max_clock_bias_error_m: f64,
    /// Maximum residual RMS in meters.
    pub max_residual_rms_m: f64,
    /// Maximum PDOP for an in-budget solution epoch.
    pub max_pdop: f64,
}

/// Hard truth-guided accuracy thresholds for receiver stages.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct SyntheticReceiverAccuracyBudgets {
    /// Acquisition-stage hard thresholds.
    pub acquisition: SyntheticAcquisitionAccuracyBudget,
    /// Tracking-stage hard thresholds.
    pub tracking: SyntheticTrackingAccuracyBudget,
    /// Observation-stage hard thresholds.
    pub observation: SyntheticObservationAccuracyBudget,
    /// PVT-stage hard thresholds.
    pub pvt: SyntheticPvtAccuracyBudget,
}

/// Machine-checkable reason a stage accuracy report lacks sufficient synthetic truth coverage.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub struct SyntheticTruthCoverageIssue {
    /// Satellite identifier when the issue is scoped to one tracked satellite.
    pub sat: Option<SatId>,
    /// Navigation epoch index when the issue is scoped to one solution epoch.
    pub epoch_index: Option<u64>,
    /// Stable issue code describing the missing or incomplete truth coverage.
    pub code: String,
}

/// Per-satellite acquisition budget outcome.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionAccuracySatellite {
    /// Satellite identifier.
    pub sat: SatId,
    /// Measured absolute Doppler error in Hz.
    pub doppler_error_hz: f64,
    /// Measured wrapped code-phase error in samples.
    pub code_phase_error_samples: usize,
    /// Whether this satellite stayed within the configured hard budget.
    pub pass: bool,
}

/// Acquisition-stage hard-budget validation report.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionAccuracyReport {
    /// Stable scenario identifier for this capture.
    pub scenario_id: String,
    /// Maximum absolute Doppler error allowed in Hz.
    pub max_doppler_error_hz: f64,
    /// Maximum wrapped code-phase error allowed in samples.
    pub max_code_phase_error_samples: usize,
    /// Number of measured satellites.
    pub satellite_count: usize,
    /// Number of satellites that satisfied the budget.
    pub passing_satellite_count: usize,
    /// Whether this report had enough synthetic truth coverage to make a hard accuracy claim.
    pub truth_coverage_ready: bool,
    /// Machine-checkable truth-coverage issues that forced or should force validation failure.
    pub truth_coverage_issues: Vec<SyntheticTruthCoverageIssue>,
    /// Whether every measured satellite satisfied the budget.
    pub pass: bool,
    /// Per-satellite budget outcomes.
    pub satellites: Vec<SyntheticAcquisitionAccuracySatellite>,
}

/// Per-satellite tracking budget outcome.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticTrackingAccuracySatellite {
    /// Satellite identifier.
    pub sat: SatId,
    /// Number of stable tracking epochs compared against truth.
    pub stable_epoch_count: usize,
    /// Maximum carrier error across stable tracking epochs, in Hz.
    pub max_carrier_error_hz: f64,
    /// Maximum Doppler error across stable tracking epochs, in Hz.
    pub max_doppler_error_hz: f64,
    /// Maximum wrapped code-phase error across stable tracking epochs, in samples.
    pub max_code_phase_error_samples: f64,
    /// Maximum C/N0 error across stable tracking epochs, in dB-Hz.
    pub max_cn0_error_db_hz: f64,
    /// Whether this satellite stayed within the configured hard budget.
    pub pass: bool,
}

/// Tracking-stage hard-budget validation report.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticTrackingAccuracyReport {
    /// Stable scenario identifier for this capture.
    pub scenario_id: String,
    /// Maximum absolute carrier-frequency error allowed in Hz.
    pub max_carrier_error_hz: f64,
    /// Maximum absolute Doppler error allowed in Hz.
    pub max_doppler_error_hz: f64,
    /// Maximum wrapped code-phase error allowed in samples.
    pub max_code_phase_error_samples: f64,
    /// Maximum absolute C/N0 error allowed in dB-Hz.
    pub max_cn0_error_db_hz: f64,
    /// Number of measured satellites.
    pub satellite_count: usize,
    /// Number of satellites that satisfied the budget.
    pub passing_satellite_count: usize,
    /// Whether this report had enough synthetic truth coverage to make a hard accuracy claim.
    pub truth_coverage_ready: bool,
    /// Machine-checkable truth-coverage issues that forced or should force validation failure.
    pub truth_coverage_issues: Vec<SyntheticTruthCoverageIssue>,
    /// Whether every measured satellite satisfied the budget.
    pub pass: bool,
    /// Per-satellite budget outcomes.
    pub satellites: Vec<SyntheticTrackingAccuracySatellite>,
}

/// Per-satellite observation budget outcome.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticObservationAccuracySatellite {
    /// Satellite identifier.
    pub sat: SatId,
    /// Maximum absolute pseudorange error in meters.
    pub max_pseudorange_error_m: Option<f64>,
    /// Maximum absolute carrier-phase error in cycles.
    pub max_carrier_phase_error_cycles: Option<f64>,
    /// Maximum absolute Doppler error in Hz.
    pub max_doppler_error_hz: Option<f64>,
    /// Maximum absolute C/N0 error in dB-Hz.
    pub max_cn0_error_db_hz: Option<f64>,
    /// Whether this satellite stayed within the configured hard budget.
    pub pass: bool,
}

/// Observation-stage hard-budget validation report.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticObservationAccuracyReport {
    /// Stable scenario identifier for this validation run.
    pub scenario_id: String,
    /// Maximum absolute pseudorange error allowed in meters.
    pub max_pseudorange_error_m: f64,
    /// Maximum absolute carrier-phase error allowed in cycles.
    pub max_carrier_phase_error_cycles: f64,
    /// Maximum absolute Doppler error allowed in Hz.
    pub max_doppler_error_hz: f64,
    /// Maximum absolute C/N0 error allowed in dB-Hz.
    pub max_cn0_error_db_hz: f64,
    /// Number of measured satellites.
    pub satellite_count: usize,
    /// Number of satellites that satisfied the budget.
    pub passing_satellite_count: usize,
    /// Whether this report had enough synthetic truth coverage to make a hard accuracy claim.
    pub truth_coverage_ready: bool,
    /// Machine-checkable truth-coverage issues that forced or should force validation failure.
    pub truth_coverage_issues: Vec<SyntheticTruthCoverageIssue>,
    /// Whether every measured satellite satisfied the budget.
    pub pass: bool,
    /// Per-satellite budget outcomes.
    pub satellites: Vec<SyntheticObservationAccuracySatellite>,
}

/// Per-epoch PVT budget outcome.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticPvtAccuracyEpoch {
    /// Receiver navigation epoch index.
    pub epoch_index: u64,
    /// 3D position error in meters.
    pub position_error_3d_m: f64,
    /// Absolute clock-bias error in meters.
    pub clock_bias_error_m: f64,
    /// Residual RMS in meters.
    pub residual_rms_m: f64,
    /// PDOP reported for this epoch.
    pub pdop: f64,
    /// Whether this epoch stayed within the configured hard budget.
    pub pass: bool,
}

/// PVT-stage hard-budget validation report.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticPvtAccuracyReport {
    /// Stable scenario identifier for this validation run.
    pub scenario_id: String,
    /// Maximum 3D position error allowed in meters.
    pub max_position_error_3d_m: f64,
    /// Maximum absolute clock-bias error allowed in meters.
    pub max_clock_bias_error_m: f64,
    /// Maximum residual RMS allowed in meters.
    pub max_residual_rms_m: f64,
    /// Maximum PDOP allowed for an in-budget epoch.
    pub max_pdop: f64,
    /// Number of matched solution epochs.
    pub epoch_count: usize,
    /// Number of epochs that satisfied the budget.
    pub passing_epoch_count: usize,
    /// Whether this report had enough synthetic truth coverage to make a hard accuracy claim.
    pub truth_coverage_ready: bool,
    /// Machine-checkable truth-coverage issues that forced or should force validation failure.
    pub truth_coverage_issues: Vec<SyntheticTruthCoverageIssue>,
    /// Whether every matched epoch satisfied the budget.
    pub pass: bool,
    /// Per-epoch budget outcomes.
    pub epochs: Vec<SyntheticPvtAccuracyEpoch>,
}

/// One synthetic PVT accuracy measurement point indexed by observed signal strength.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticPvtCn0ProfilePoint {
    /// Stable scenario identifier for this validation run.
    pub scenario_id: String,
    /// Number of observation epochs that contributed C/N0 measurements.
    pub observation_epoch_count: usize,
    /// Mean observation C/N0 across the contributing epochs, in dB-Hz.
    pub mean_observation_cn0_dbhz: f64,
    /// Minimum per-epoch mean observation C/N0, in dB-Hz.
    pub min_observation_cn0_dbhz: f64,
    /// Maximum per-epoch mean observation C/N0, in dB-Hz.
    pub max_observation_cn0_dbhz: f64,
    /// Number of matched PVT epochs compared against truth.
    pub epoch_count: usize,
    /// Number of matched PVT epochs that satisfied the hard accuracy budget.
    pub passing_epoch_count: usize,
    /// Passing-epoch fraction across the matched PVT epochs.
    pub pass_rate: f64,
    /// RMS 3D position error across matched PVT epochs, in meters.
    pub rms_position_error_3d_m: Option<f64>,
    /// Maximum 3D position error across matched PVT epochs, in meters.
    pub max_position_error_3d_m: Option<f64>,
    /// RMS clock-bias error across matched PVT epochs, in meters.
    pub rms_clock_bias_error_m: Option<f64>,
    /// Maximum clock-bias error across matched PVT epochs, in meters.
    pub max_clock_bias_error_m: Option<f64>,
    /// RMS residual RMS across matched PVT epochs, in meters.
    pub rms_residual_rms_m: Option<f64>,
    /// Maximum residual RMS across matched PVT epochs, in meters.
    pub max_residual_rms_m: Option<f64>,
    /// Maximum PDOP across matched PVT epochs.
    pub max_pdop: Option<f64>,
    /// Whether synthetic truth coverage remained sufficient for a hard claim.
    pub truth_coverage_ready: bool,
    /// Machine-checkable truth-coverage issues that forced or should force validation failure.
    pub truth_coverage_issues: Vec<SyntheticTruthCoverageIssue>,
    /// Whether the point had both truth-ready PVT comparisons and observed C/N0 support.
    pub ready: bool,
}

/// Truth-guided PVT accuracy profile across multiple signal-strength points.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticPvtCn0ProfileReport {
    /// Scenario identifier prefix shared across the measurement points.
    pub scenario_id_prefix: String,
    /// Measurement points captured in the report.
    pub points: Vec<SyntheticPvtCn0ProfilePoint>,
}

/// One synthetic PVT accuracy measurement point indexed by satellite geometry quality.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticPvtGeometryProfilePoint {
    /// Stable scenario identifier for this validation run.
    pub scenario_id: String,
    /// Number of matched PVT epochs that contributed geometry measurements.
    pub epoch_count: usize,
    /// Number of matched PVT epochs that satisfied the hard accuracy budget.
    pub passing_epoch_count: usize,
    /// Passing-epoch fraction across the matched PVT epochs.
    pub pass_rate: f64,
    /// Mean PDOP across matched PVT epochs.
    pub mean_pdop: Option<f64>,
    /// Minimum PDOP across matched PVT epochs.
    pub min_pdop: Option<f64>,
    /// Maximum PDOP across matched PVT epochs.
    pub max_pdop: Option<f64>,
    /// RMS 3D position error across matched PVT epochs, in meters.
    pub rms_position_error_3d_m: Option<f64>,
    /// Maximum 3D position error across matched PVT epochs, in meters.
    pub max_position_error_3d_m: Option<f64>,
    /// RMS clock-bias error across matched PVT epochs, in meters.
    pub rms_clock_bias_error_m: Option<f64>,
    /// Maximum clock-bias error across matched PVT epochs, in meters.
    pub max_clock_bias_error_m: Option<f64>,
    /// RMS residual RMS across matched PVT epochs, in meters.
    pub rms_residual_rms_m: Option<f64>,
    /// Maximum residual RMS across matched PVT epochs, in meters.
    pub max_residual_rms_m: Option<f64>,
    /// Whether synthetic truth coverage remained sufficient for a hard claim.
    pub truth_coverage_ready: bool,
    /// Machine-checkable truth-coverage issues that forced or should force validation failure.
    pub truth_coverage_issues: Vec<SyntheticTruthCoverageIssue>,
    /// Whether the point had truth-ready PVT comparisons and at least one geometry measurement.
    pub ready: bool,
}

/// Truth-guided PVT accuracy profile across multiple satellite-geometry points.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticPvtGeometryProfileReport {
    /// Scenario identifier prefix shared across the measurement points.
    pub scenario_id_prefix: String,
    /// Measurement points captured in the report.
    pub points: Vec<SyntheticPvtGeometryProfilePoint>,
}

/// One synthetic PVT accuracy measurement point indexed by injected multipath severity.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticPvtMultipathProfilePoint {
    /// Stable scenario identifier for this validation run.
    pub scenario_id: String,
    /// Number of satellites carrying injected multipath bias.
    pub affected_satellite_count: usize,
    /// Mean absolute injected pseudorange bias across affected satellites, in meters.
    pub mean_abs_pseudorange_bias_m: f64,
    /// Maximum absolute injected pseudorange bias across affected satellites, in meters.
    pub max_abs_pseudorange_bias_m: f64,
    /// Number of matched PVT epochs compared against truth.
    pub epoch_count: usize,
    /// Number of matched PVT epochs that satisfied the hard accuracy budget.
    pub passing_epoch_count: usize,
    /// Passing-epoch fraction across the matched PVT epochs.
    pub pass_rate: f64,
    /// Number of truth-matched epochs whose solution validity remained stable.
    pub stable_epoch_count: usize,
    /// Stable-solution fraction across truth-matched epochs.
    pub stable_epoch_rate: f64,
    /// Number of truth-matched epochs whose solution validity diverged.
    pub diverging_epoch_count: usize,
    /// Diverging-solution fraction across truth-matched epochs.
    pub diverging_epoch_rate: f64,
    /// RMS 3D position error across matched PVT epochs, in meters.
    pub rms_position_error_3d_m: Option<f64>,
    /// Maximum 3D position error across matched PVT epochs, in meters.
    pub max_position_error_3d_m: Option<f64>,
    /// RMS residual RMS across matched PVT epochs, in meters.
    pub rms_residual_rms_m: Option<f64>,
    /// Maximum residual RMS across matched PVT epochs, in meters.
    pub max_residual_rms_m: Option<f64>,
    /// Whether synthetic truth coverage remained sufficient for a hard claim.
    pub truth_coverage_ready: bool,
    /// Machine-checkable truth-coverage issues that forced or should force validation failure.
    pub truth_coverage_issues: Vec<SyntheticTruthCoverageIssue>,
    /// Whether the point had truth-ready PVT comparisons and at least one matched epoch.
    pub ready: bool,
}

/// Truth-guided PVT accuracy profile across multiple synthetic multipath points.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticPvtMultipathProfileReport {
    /// Scenario identifier prefix shared across the measurement points.
    pub scenario_id_prefix: String,
    /// Measurement points captured in the report.
    pub points: Vec<SyntheticPvtMultipathProfilePoint>,
}

/// One synthetic PVT accuracy measurement point indexed by receiver-motion truth path.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticPvtMotionProfilePoint {
    /// Stable scenario identifier for this validation run.
    pub scenario_id: String,
    /// Number of truth epochs in the receiver-motion reference path.
    pub truth_epoch_count: usize,
    /// Whether the truth path contains measurable receiver displacement.
    pub moving: bool,
    /// Total receiver path length across the truth epochs, in meters.
    pub path_length_m: f64,
    /// Mean receiver speed across the truth epochs, in meters per second.
    pub mean_speed_mps: f64,
    /// Number of matched PVT epochs compared against truth.
    pub epoch_count: usize,
    /// Number of matched PVT epochs that satisfied the hard accuracy budget.
    pub passing_epoch_count: usize,
    /// Passing-epoch fraction across the matched PVT epochs.
    pub pass_rate: f64,
    /// Number of truth-matched epochs whose solution validity remained stable.
    pub stable_epoch_count: usize,
    /// Stable-solution fraction across truth-matched epochs.
    pub stable_epoch_rate: f64,
    /// RMS 3D position error across matched PVT epochs, in meters.
    pub rms_position_error_3d_m: Option<f64>,
    /// Maximum 3D position error across matched PVT epochs, in meters.
    pub max_position_error_3d_m: Option<f64>,
    /// RMS residual RMS across matched PVT epochs, in meters.
    pub rms_residual_rms_m: Option<f64>,
    /// Maximum residual RMS across matched PVT epochs, in meters.
    pub max_residual_rms_m: Option<f64>,
    /// Whether synthetic truth coverage remained sufficient for a hard claim.
    pub truth_coverage_ready: bool,
    /// Machine-checkable truth-coverage issues that forced or should force validation failure.
    pub truth_coverage_issues: Vec<SyntheticTruthCoverageIssue>,
    /// Whether the point had truth-ready PVT comparisons and at least one matched epoch.
    pub ready: bool,
}

/// Truth-guided PVT accuracy profile across multiple receiver-motion truth paths.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticPvtMotionProfileReport {
    /// Scenario identifier prefix shared across the measurement points.
    pub scenario_id_prefix: String,
    /// Measurement points captured in the report.
    pub points: Vec<SyntheticPvtMotionProfilePoint>,
}

/// One truth-guided PVT accuracy measurement point indexed by injected receiver clock drift.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticPvtClockProfilePoint {
    /// Stable scenario identifier for this validation run.
    pub scenario_id: String,
    /// Injected receiver clock drift carried by the scenario truth, in seconds per second.
    pub injected_clock_drift_s_per_s: f64,
    /// Expected observation-Doppler offset induced by the injected clock drift, in hertz.
    pub expected_observation_doppler_offset_hz: f64,
    /// Number of matched PVT epochs compared against truth.
    pub epoch_count: usize,
    /// Number of matched PVT epochs that satisfied the hard accuracy budget.
    pub passing_epoch_count: usize,
    /// Passing-epoch fraction across the matched PVT epochs.
    pub pass_rate: f64,
    /// Number of observation Doppler pairs compared against the stable reference observations.
    pub observation_doppler_pair_count: usize,
    /// Mean observed observation-Doppler offset relative to the stable reference observations, in hertz.
    pub observed_mean_observation_doppler_offset_hz: Option<f64>,
    /// Minimum observed observation-Doppler offset relative to the stable reference observations, in hertz.
    pub observed_min_observation_doppler_offset_hz: Option<f64>,
    /// Maximum observed observation-Doppler offset relative to the stable reference observations, in hertz.
    pub observed_max_observation_doppler_offset_hz: Option<f64>,
    /// Largest absolute observation-Doppler offset error relative to the expected clock-induced offset, in hertz.
    pub max_observation_doppler_offset_error_hz: Option<f64>,
    /// Final solved receiver clock drift from the navigation solution sequence, in seconds per second.
    pub final_solved_clock_drift_s_per_s: Option<f64>,
    /// Mean solved receiver clock drift across the navigation solution sequence, in seconds per second.
    pub mean_solved_clock_drift_s_per_s: Option<f64>,
    /// Largest absolute solved-drift error across the navigation solution sequence, in seconds per second.
    pub max_clock_drift_error_s_per_s: Option<f64>,
    /// RMS clock-bias error across matched PVT epochs, in meters.
    pub rms_clock_bias_error_m: Option<f64>,
    /// Maximum clock-bias error across matched PVT epochs, in meters.
    pub max_clock_bias_error_m: Option<f64>,
    /// RMS residual RMS across matched PVT epochs, in meters.
    pub rms_residual_rms_m: Option<f64>,
    /// Maximum residual RMS across matched PVT epochs, in meters.
    pub max_residual_rms_m: Option<f64>,
    /// Whether synthetic truth coverage remained sufficient for a hard claim.
    pub truth_coverage_ready: bool,
    /// Machine-checkable truth-coverage issues that forced or should force validation failure.
    pub truth_coverage_issues: Vec<SyntheticTruthCoverageIssue>,
    /// Whether the point had truth-ready PVT comparisons and at least one solved clock-drift estimate.
    pub ready: bool,
}

/// Truth-guided PVT accuracy profile across multiple receiver clock-drift points.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticPvtClockProfileReport {
    /// Scenario identifier prefix shared across the measurement points.
    pub scenario_id_prefix: String,
    /// Measurement points captured in the report.
    pub points: Vec<SyntheticPvtClockProfilePoint>,
}

/// Time-evolution trend classification for a truth-guided PVT accuracy run.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
pub enum SyntheticPvtTimeTrend {
    /// Errors settle or remain bounded over the measured interval.
    Stabilizing,
    /// Errors grow over time without the receiver validity diverging.
    Drifting,
    /// Errors diverge strongly enough that receiver validity becomes diverging.
    Diverging,
}

/// One truth-guided PVT accuracy measurement point indexed by time evolution.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticPvtTimeProfilePoint {
    /// Stable scenario identifier for this validation run.
    pub scenario_id: String,
    /// Number of truth-matched epochs in the time series.
    pub truth_epoch_count: usize,
    /// Time covered by the matched epochs, in seconds.
    pub duration_s: f64,
    /// Number of matched PVT epochs compared against truth.
    pub epoch_count: usize,
    /// Number of matched PVT epochs that satisfied the hard accuracy budget.
    pub passing_epoch_count: usize,
    /// Passing-epoch fraction across the matched PVT epochs.
    pub pass_rate: f64,
    /// Number of truth-matched epochs whose solution validity remained stable.
    pub stable_epoch_count: usize,
    /// Stable-solution fraction across truth-matched epochs.
    pub stable_epoch_rate: f64,
    /// Number of truth-matched epochs whose solution validity diverged.
    pub diverging_epoch_count: usize,
    /// Diverging-solution fraction across truth-matched epochs.
    pub diverging_epoch_rate: f64,
    /// Mean 3D position error over the first analysis window, in meters.
    pub first_window_mean_position_error_3d_m: Option<f64>,
    /// Mean 3D position error over the last analysis window, in meters.
    pub last_window_mean_position_error_3d_m: Option<f64>,
    /// Linear 3D position-error drift slope over receive time, in meters per second.
    pub position_error_drift_m_per_s: Option<f64>,
    /// Mean residual RMS over the first analysis window, in meters.
    pub first_window_mean_residual_rms_m: Option<f64>,
    /// Mean residual RMS over the last analysis window, in meters.
    pub last_window_mean_residual_rms_m: Option<f64>,
    /// Linear residual-RMS drift slope over receive time, in meters per second.
    pub residual_rms_drift_m_per_s: Option<f64>,
    /// Classified time-evolution trend.
    pub trend: SyntheticPvtTimeTrend,
    /// Whether synthetic truth coverage remained sufficient for a hard claim.
    pub truth_coverage_ready: bool,
    /// Machine-checkable truth-coverage issues that forced or should force validation failure.
    pub truth_coverage_issues: Vec<SyntheticTruthCoverageIssue>,
    /// Whether the point had truth-ready PVT comparisons and enough epochs to classify time trend.
    pub ready: bool,
}

/// Truth-guided PVT accuracy profile across long-run time-evolution points.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticPvtTimeProfileReport {
    /// Scenario identifier prefix shared across the measurement points.
    pub scenario_id_prefix: String,
    /// Measurement points captured in the report.
    pub points: Vec<SyntheticPvtTimeProfilePoint>,
}

/// Aggregated multi-stage accuracy summary at one signal-strength point.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAccuracyCn0ProfilePoint {
    /// Shared signal-strength coordinate for the merged stage metrics, in dB-Hz.
    pub cn0_db_hz: f64,
    /// Number of acquisition cases contributing to this point.
    pub acquisition_case_count: usize,
    /// Total synthetic acquisition trials contributing to this point.
    pub acquisition_trial_count: usize,
    /// Mean acceptance probability across acquisition cases at this point.
    pub acquisition_acceptance_probability_mean: Option<f64>,
    /// Mean detection probability across acquisition cases at this point.
    pub acquisition_detection_probability_mean: Option<f64>,
    /// Number of tracking cases contributing to this point.
    pub tracking_case_count: usize,
    /// Total synthetic tracking trials contributing to this point.
    pub tracking_trial_count: usize,
    /// Mean stable-lock probability across tracking cases at this point.
    pub tracking_lock_probability_mean: Option<f64>,
    /// Mean refused-lock rate across tracking cases at this point.
    pub tracking_refused_lock_rate_mean: Option<f64>,
    /// Number of PVT cases contributing to this point.
    pub pvt_case_count: usize,
    /// Total matched PVT epochs contributing to this point.
    pub pvt_epoch_count: usize,
    /// Mean PVT pass rate across cases at this point.
    pub pvt_pass_rate_mean: Option<f64>,
    /// Mean RMS 3D position error across cases at this point, in meters.
    pub pvt_rms_position_error_3d_m_mean: Option<f64>,
    /// Maximum 3D position error observed across cases at this point, in meters.
    pub pvt_max_position_error_3d_m_max: Option<f64>,
}

/// Merged acquisition, tracking, and PVT performance profile across signal strength.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAccuracyCn0ProfileReport {
    /// Scenario identifier prefix shared across the measurement points.
    pub scenario_id_prefix: String,
    /// Merged per-C/N0 measurement points.
    pub points: Vec<SyntheticAccuracyCn0ProfilePoint>,
}

/// Summary of one observation-error distribution.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticObservationErrorStats {
    /// Count of comparable observation rows.
    pub count: usize,
    /// Signed mean error.
    pub mean_error: f64,
    /// Median absolute error.
    pub median_abs_error: f64,
    /// Root-mean-square error magnitude.
    pub rms_error: f64,
    /// 95th percentile absolute error.
    pub p95_abs_error: f64,
    /// Maximum absolute error.
    pub max_abs_error: f64,
}

/// Per-satellite observation truth comparison summary.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticObservationValidationSatellite {
    /// Satellite identifier.
    pub sat: SatId,
    /// Aggregated pseudorange error statistics in meters.
    pub pseudorange_error_m: Option<SyntheticObservationErrorStats>,
    /// Aggregated ambiguity-aligned carrier-phase residual statistics in cycles.
    pub carrier_phase_error_cycles: Option<SyntheticObservationErrorStats>,
    /// Number of carrier-phase arcs aligned before residual aggregation.
    pub carrier_phase_arcs_evaluated: usize,
    /// Aggregated Doppler error statistics in Hz.
    pub doppler_error_hz: Option<SyntheticObservationErrorStats>,
    /// Aggregated C/N0 error statistics in dB-Hz.
    pub cn0_error_db_hz: Option<SyntheticObservationErrorStats>,
    /// Reasons a metric could not be evaluated for this satellite.
    pub notes: Vec<String>,
}

/// Truth-guided observation validation report for a synthetic scenario.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticObservationValidationReport {
    /// Stable scenario identifier for this validation run.
    pub scenario_id: String,
    /// Capture sample rate in Hz.
    pub sample_rate_hz: f64,
    /// Hatch-smoothing window applied before observation comparison.
    pub hatch_window: u32,
    /// Absolute receive-time anchor used for geometric pseudorange truth.
    pub reference_receive_time_s: f64,
    /// Per-satellite observation truth summaries.
    pub satellites: Vec<SyntheticObservationValidationSatellite>,
}

/// Data-source summary for one GNSS accuracy artifact.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticGnssAccuracyDataSource {
    /// Stable source kind used to interpret the run origin.
    pub source_kind: String,
    /// Sample rate used for the validation run, in Hz.
    pub sample_rate_hz: f64,
    /// Intermediate frequency used for the validation run, in Hz.
    pub intermediate_freq_hz: f64,
    /// Total synthetic capture duration, in seconds.
    pub duration_s: f64,
    /// Number of satellites present in the source scenario.
    pub satellite_count: usize,
}

/// Reference-truth summary for one GNSS accuracy artifact.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticGnssAccuracyReferenceTruth {
    /// Stable truth kind used to interpret the reference source.
    pub truth_kind: String,
    /// Receiver ECEF reference coordinates when available, in meters.
    pub receiver_ecef_m: Option<[f64; 3]>,
    /// Absolute receive-time anchor for the truth reference, in seconds.
    pub reference_receive_time_s: Option<f64>,
    /// Number of satellites covered by the reference truth.
    pub satellite_count: usize,
    /// Number of truth epochs available to the validation run.
    pub reference_epoch_count: usize,
}

/// Compact acquisition-stage summary carried by a final GNSS accuracy artifact.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticGnssAcquisitionStageSummary {
    /// Whether the acquisition stage satisfied its hard thresholds.
    pub pass: bool,
    /// Whether synthetic truth coverage was sufficient for the stage claim.
    pub truth_coverage_ready: bool,
    /// Number of measured satellites.
    pub satellite_count: usize,
    /// Number of satellites that satisfied the stage budget.
    pub passing_satellite_count: usize,
    /// Largest absolute Doppler error observed across measured satellites, in Hz.
    pub observed_max_doppler_error_hz: Option<f64>,
    /// Largest wrapped code-phase error observed across measured satellites, in samples.
    pub observed_max_code_phase_error_samples: Option<usize>,
    /// Maximum absolute Doppler error allowed in Hz.
    pub threshold_max_doppler_error_hz: f64,
    /// Maximum wrapped code-phase error allowed in samples.
    pub threshold_max_code_phase_error_samples: usize,
}

/// Compact tracking-stage summary carried by a final GNSS accuracy artifact.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticGnssTrackingStageSummary {
    /// Whether the tracking stage satisfied its hard thresholds.
    pub pass: bool,
    /// Whether synthetic truth coverage was sufficient for the stage claim.
    pub truth_coverage_ready: bool,
    /// Number of measured satellites.
    pub satellite_count: usize,
    /// Number of satellites that satisfied the stage budget.
    pub passing_satellite_count: usize,
    /// Largest carrier-frequency error observed across measured satellites, in Hz.
    pub observed_max_carrier_error_hz: Option<f64>,
    /// Largest Doppler error observed across measured satellites, in Hz.
    pub observed_max_doppler_error_hz: Option<f64>,
    /// Largest wrapped code-phase error observed across measured satellites, in samples.
    pub observed_max_code_phase_error_samples: Option<f64>,
    /// Largest C/N0 error observed across measured satellites, in dB-Hz.
    pub observed_max_cn0_error_db_hz: Option<f64>,
    /// Maximum absolute carrier-frequency error allowed in Hz.
    pub threshold_max_carrier_error_hz: f64,
    /// Maximum absolute Doppler error allowed in Hz.
    pub threshold_max_doppler_error_hz: f64,
    /// Maximum wrapped code-phase error allowed in samples.
    pub threshold_max_code_phase_error_samples: f64,
    /// Maximum absolute C/N0 error allowed in dB-Hz.
    pub threshold_max_cn0_error_db_hz: f64,
}

/// Compact observation-stage summary carried by a final GNSS accuracy artifact.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticGnssObservationStageSummary {
    /// Whether the observation stage satisfied its hard thresholds.
    pub pass: bool,
    /// Whether synthetic truth coverage was sufficient for the stage claim.
    pub truth_coverage_ready: bool,
    /// Number of measured satellites.
    pub satellite_count: usize,
    /// Number of satellites that satisfied the stage budget.
    pub passing_satellite_count: usize,
    /// Largest pseudorange error observed across measured satellites, in meters.
    pub observed_max_pseudorange_error_m: Option<f64>,
    /// Largest carrier-phase error observed across measured satellites, in cycles.
    pub observed_max_carrier_phase_error_cycles: Option<f64>,
    /// Largest Doppler error observed across measured satellites, in Hz.
    pub observed_max_doppler_error_hz: Option<f64>,
    /// Largest C/N0 error observed across measured satellites, in dB-Hz.
    pub observed_max_cn0_error_db_hz: Option<f64>,
    /// Maximum absolute pseudorange error allowed in meters.
    pub threshold_max_pseudorange_error_m: f64,
    /// Maximum absolute carrier-phase error allowed in cycles.
    pub threshold_max_carrier_phase_error_cycles: f64,
    /// Maximum absolute Doppler error allowed in Hz.
    pub threshold_max_doppler_error_hz: f64,
    /// Maximum absolute C/N0 error allowed in dB-Hz.
    pub threshold_max_cn0_error_db_hz: f64,
}

/// Compact PVT-stage summary carried by a final GNSS accuracy artifact.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticGnssPvtStageSummary {
    /// Whether the PVT stage satisfied its hard thresholds.
    pub pass: bool,
    /// Whether synthetic truth coverage was sufficient for the stage claim.
    pub truth_coverage_ready: bool,
    /// Number of measured navigation epochs.
    pub epoch_count: usize,
    /// Number of epochs that satisfied the stage budget.
    pub passing_epoch_count: usize,
    /// Largest 3D position error observed across measured epochs, in meters.
    pub observed_max_position_error_3d_m: Option<f64>,
    /// Largest absolute clock-bias error observed across measured epochs, in meters.
    pub observed_max_clock_bias_error_m: Option<f64>,
    /// Largest residual RMS observed across measured epochs, in meters.
    pub observed_max_residual_rms_m: Option<f64>,
    /// Largest PDOP observed across measured epochs.
    pub observed_max_pdop: Option<f64>,
    /// Maximum 3D position error allowed in meters.
    pub threshold_max_position_error_3d_m: f64,
    /// Maximum absolute clock-bias error allowed in meters.
    pub threshold_max_clock_bias_error_m: f64,
    /// Maximum residual RMS allowed in meters.
    pub threshold_max_residual_rms_m: f64,
    /// Maximum PDOP allowed for an in-budget epoch.
    pub threshold_max_pdop: f64,
}

/// Final acquisition-stage artifact payload with both compact summary and detailed report.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticGnssAcquisitionStageArtifact {
    /// Compact stage summary used for run-level pass/fail review.
    pub summary: SyntheticGnssAcquisitionStageSummary,
    /// Detailed acquisition accuracy report for per-satellite review.
    pub report: SyntheticAcquisitionAccuracyReport,
}

/// Final tracking-stage artifact payload with both compact summary and detailed report.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticGnssTrackingStageArtifact {
    /// Compact stage summary used for run-level pass/fail review.
    pub summary: SyntheticGnssTrackingStageSummary,
    /// Detailed tracking accuracy report for per-satellite review.
    pub report: SyntheticTrackingAccuracyReport,
}

/// Final observation-stage artifact payload with both compact summary and detailed report.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticGnssObservationStageArtifact {
    /// Compact stage summary used for run-level pass/fail review.
    pub summary: SyntheticGnssObservationStageSummary,
    /// Detailed observation accuracy report for per-satellite review.
    pub report: SyntheticObservationAccuracyReport,
}

/// Final PVT-stage artifact payload with both compact summary and detailed report.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticGnssPvtStageArtifact {
    /// Compact stage summary used for run-level pass/fail review.
    pub summary: SyntheticGnssPvtStageSummary,
    /// Detailed PVT accuracy report for per-epoch review.
    pub report: SyntheticPvtAccuracyReport,
}

/// Borrowed inputs for building one final GNSS accuracy artifact.
#[derive(Debug, Clone)]
pub struct SyntheticGnssAccuracyArtifactCase<'a> {
    /// Stable scenario identifier for the validation run.
    pub scenario_id: &'a str,
    /// Data-source summary for the validation run.
    pub data_source: SyntheticGnssAccuracyDataSource,
    /// Reference-truth summary for the validation run.
    pub reference_truth: SyntheticGnssAccuracyReferenceTruth,
    /// Detailed acquisition-stage accuracy report.
    pub acquisition: &'a SyntheticAcquisitionAccuracyReport,
    /// Detailed tracking-stage accuracy report.
    pub tracking: &'a SyntheticTrackingAccuracyReport,
    /// Detailed observation-stage accuracy report.
    pub observation: &'a SyntheticObservationAccuracyReport,
    /// Detailed PVT-stage accuracy report.
    pub pvt: &'a SyntheticPvtAccuracyReport,
}

/// Final truth-guided GNSS accuracy artifact for one validation run.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticGnssAccuracyArtifact {
    /// Schema version for the artifact payload.
    pub schema_version: u32,
    /// Stable scenario identifier for the validation run.
    pub scenario_id: String,
    /// Whether every stage satisfied its hard thresholds.
    pub pass: bool,
    /// Whether every stage had enough truth coverage for a hard claim.
    pub truth_coverage_ready: bool,
    /// Validation-run source summary.
    pub data_source: SyntheticGnssAccuracyDataSource,
    /// Validation-run reference-truth summary.
    pub reference_truth: SyntheticGnssAccuracyReferenceTruth,
    /// Acquisition-stage artifact payload.
    pub acquisition: SyntheticGnssAcquisitionStageArtifact,
    /// Tracking-stage artifact payload.
    pub tracking: SyntheticGnssTrackingStageArtifact,
    /// Observation-stage artifact payload.
    pub observation: SyntheticGnssObservationStageArtifact,
    /// PVT-stage artifact payload.
    pub pvt: SyntheticGnssPvtStageArtifact,
}

fn summarize_observation_errors(errors: &[f64]) -> Option<SyntheticObservationErrorStats> {
    if errors.is_empty() {
        return None;
    }

    let signed = stats(errors);
    let absolute_errors = errors.iter().map(|error| error.abs()).collect::<Vec<_>>();
    let absolute = stats(&absolute_errors);

    Some(SyntheticObservationErrorStats {
        count: signed.count,
        mean_error: signed.mean,
        median_abs_error: absolute.median,
        rms_error: signed.rms,
        p95_abs_error: absolute.p95,
        max_abs_error: absolute.max,
    })
}

fn mean_f64(values: &[f64]) -> Option<f64> {
    if values.is_empty() {
        None
    } else {
        Some(values.iter().sum::<f64>() / values.len() as f64)
    }
}

fn time_profile_window_len(epoch_count: usize) -> usize {
    epoch_count.clamp(1, 25)
}

fn linear_trend_slope(receive_times_s: &[f64], values: &[f64]) -> Option<f64> {
    if receive_times_s.len() != values.len() || values.len() < 2 {
        return None;
    }
    let mean_time_s = mean_f64(receive_times_s)?;
    let mean_value = mean_f64(values)?;
    let (numerator, denominator) =
        receive_times_s.iter().zip(values.iter()).fold((0.0, 0.0), |acc, (time_s, value)| {
            let centered_time_s = time_s - mean_time_s;
            (
                acc.0 + centered_time_s * (value - mean_value),
                acc.1 + centered_time_s * centered_time_s,
            )
        });
    (denominator > f64::EPSILON).then_some(numerator / denominator)
}

fn classify_pvt_time_trend(
    diverging_epoch_count: usize,
    first_window_mean_position_error_3d_m: Option<f64>,
    last_window_mean_position_error_3d_m: Option<f64>,
    position_error_drift_m_per_s: Option<f64>,
    first_window_mean_residual_rms_m: Option<f64>,
    last_window_mean_residual_rms_m: Option<f64>,
    residual_rms_drift_m_per_s: Option<f64>,
) -> SyntheticPvtTimeTrend {
    if diverging_epoch_count > 0 {
        return SyntheticPvtTimeTrend::Diverging;
    }

    let position_growth_m = first_window_mean_position_error_3d_m
        .zip(last_window_mean_position_error_3d_m)
        .map(|(first, last)| last - first)
        .unwrap_or(0.0);
    let residual_growth_m = first_window_mean_residual_rms_m
        .zip(last_window_mean_residual_rms_m)
        .map(|(first, last)| last - first)
        .unwrap_or(0.0);
    let position_slope_m_per_s = position_error_drift_m_per_s.unwrap_or(0.0);
    let residual_slope_m_per_s = residual_rms_drift_m_per_s.unwrap_or(0.0);

    if position_growth_m > 5.0
        || residual_growth_m > 5.0
        || position_slope_m_per_s > 1.0
        || residual_slope_m_per_s > 1.0
    {
        SyntheticPvtTimeTrend::Diverging
    } else if position_growth_m > 0.5
        || position_slope_m_per_s > 0.1
        || ((position_growth_m > 0.0 || position_slope_m_per_s >= 0.0)
            && (residual_growth_m > 0.5 || residual_slope_m_per_s > 0.1))
    {
        SyntheticPvtTimeTrend::Drifting
    } else {
        SyntheticPvtTimeTrend::Stabilizing
    }
}

fn truth_coverage_issue(
    sat: Option<SatId>,
    epoch_index: Option<u64>,
    code: impl Into<String>,
) -> SyntheticTruthCoverageIssue {
    SyntheticTruthCoverageIssue { sat, epoch_index, code: code.into() }
}

fn acquisition_truth_coverage_issues(
    report: &SyntheticAcquisitionTruthTableReport,
) -> Vec<SyntheticTruthCoverageIssue> {
    let mut issues = Vec::new();
    if report.satellites.is_empty() {
        issues.push(truth_coverage_issue(None, None, "no_truth_satellites"));
    }
    for satellite in &report.satellites {
        if !satellite.expected_measured_doppler_hz.is_finite() {
            issues.push(truth_coverage_issue(
                Some(satellite.sat),
                None,
                "non_finite_expected_doppler_truth",
            ));
        }
        if !satellite.measured_doppler_hz.is_finite() {
            issues.push(truth_coverage_issue(
                Some(satellite.sat),
                None,
                "non_finite_measured_doppler",
            ));
        }
    }
    issues
}

fn tracking_truth_coverage_issues(
    report: &SyntheticTrackingTruthTableReport,
) -> Vec<SyntheticTruthCoverageIssue> {
    let mut issues = Vec::new();
    if report.satellites.is_empty() {
        issues.push(truth_coverage_issue(None, None, "no_truth_satellites"));
    }
    for satellite in &report.satellites {
        if satellite.epoch_count == 0 {
            issues.push(truth_coverage_issue(Some(satellite.sat), None, "no_tracking_epochs"));
        }
        if satellite.stable_epoch_count == 0 {
            issues.push(truth_coverage_issue(
                Some(satellite.sat),
                None,
                "no_stable_tracking_truth_epochs",
            ));
        }
    }
    issues
}

fn observation_truth_coverage_issues(
    report: &SyntheticObservationValidationReport,
) -> Vec<SyntheticTruthCoverageIssue> {
    let mut issues = Vec::new();
    if report.satellites.is_empty() {
        issues.push(truth_coverage_issue(None, None, "no_truth_satellites"));
    }
    for satellite in &report.satellites {
        if satellite.pseudorange_error_m.is_none() {
            issues.push(truth_coverage_issue(
                Some(satellite.sat),
                None,
                "missing_pseudorange_truth",
            ));
        }
        if satellite.carrier_phase_error_cycles.is_none() {
            issues.push(truth_coverage_issue(
                Some(satellite.sat),
                None,
                "missing_carrier_phase_truth",
            ));
        }
        if satellite.doppler_error_hz.is_none() {
            issues.push(truth_coverage_issue(Some(satellite.sat), None, "missing_doppler_truth"));
        }
        if satellite.cn0_error_db_hz.is_none() {
            issues.push(truth_coverage_issue(Some(satellite.sat), None, "missing_cn0_truth"));
        }
        for note in &satellite.notes {
            issues.push(truth_coverage_issue(Some(satellite.sat), None, note.clone()));
        }
    }
    issues
}

fn pvt_truth_coverage_issues(
    report: &SyntheticPvtTruthTableReport,
) -> Vec<SyntheticTruthCoverageIssue> {
    let mut issues = Vec::new();
    if report.solution_count == 0 {
        issues.push(truth_coverage_issue(None, None, "no_navigation_solutions"));
    }
    if report.matched_epoch_count == 0 {
        issues.push(truth_coverage_issue(None, None, "no_matched_truth_epochs"));
    }
    for epoch_index in &report.unmatched_solution_epochs {
        issues.push(truth_coverage_issue(None, Some(*epoch_index), "unmatched_solution_epoch"));
    }
    for epoch_index in &report.unused_reference_epochs {
        issues.push(truth_coverage_issue(None, Some(*epoch_index), "unused_truth_reference_epoch"));
    }
    issues
}

struct ObservedSatelliteRow<'a> {
    artifact_id: String,
    epoch_id: String,
    epoch_index: u64,
    sample_index: u64,
    observation: &'a bijux_gnss_core::api::ObsSatellite,
}

/// Build a truth-guided observation table from synthetic tracking inputs.
pub fn validate_truth_guided_observation_table(
    config: &ReceiverPipelineConfig,
    tracks: &[crate::pipeline::tracking::TrackingResult],
    truth: &SyntheticScenario,
    reference: &SyntheticObservationTruthReference,
    hatch_window: u32,
) -> SyntheticObservationTruthTableReport {
    let observations = crate::pipeline::observations::observation_artifacts_from_tracking_results(
        config,
        tracks,
        hatch_window,
    )
    .output
    .epochs;
    let satellites = truth
        .satellites
        .iter()
        .map(|sat_truth| {
            let observed_rows = comparable_satellite_observations(&observations, sat_truth.sat);
            let expected_doppler_hz = sat_truth.doppler_hz + truth.receiver_clock_frequency_bias_hz;
            let mut notes = Vec::new();
            if observed_rows.is_empty() {
                notes.push("no_comparable_observation_rows".to_string());
            }
            let ephemeris =
                truth.ephemerides.iter().find(|candidate| candidate.sat == sat_truth.sat);
            if ephemeris.is_none() {
                notes.push("missing_ephemeris_for_pseudorange_truth".to_string());
            } else if !observed_rows.iter().any(|row| {
                row.observation.metadata.pseudorange_model == "tracked_code_phase_alignment"
            }) {
                notes.push("no_absolute_pseudorange_rows".to_string());
            }
            let sat_state = SatState::new_with_receiver_clock_frequency_bias_hz(
                config,
                *sat_truth,
                truth.receiver_clock_frequency_bias_hz,
            );
            let carrier_phase_arc_bias_cycles =
                carrier_phase_arc_bias_cycles_by_start_sample(config, &sat_state, &observed_rows);
            if carrier_phase_arc_bias_cycles.is_empty() {
                notes.push("no_usable_carrier_phase_rows".to_string());
            }
            let epochs = observed_rows
                .iter()
                .map(|row| {
                    let pseudorange_truth_m = ephemeris.and_then(|ephemeris| {
                        (row.observation.metadata.pseudorange_model
                            == "tracked_code_phase_alignment")
                            .then(|| {
                                let receive_time_s = reference.receive_time_s
                                    + row.sample_index as f64 / config.sampling_freq_hz;
                                synthetic_pseudorange_m(
                                    ephemeris,
                                    receive_time_s,
                                    reference.receiver_ecef_m,
                                )
                            })
                    });
                    let carrier_phase_truth_cycles = Some(
                        sat_state.carrier_phase_rad_at(
                            row.sample_index as f64 / config.sampling_freq_hz,
                        ) / std::f64::consts::TAU,
                    );
                    let carrier_phase_arc_start_sample_index =
                        carrier_phase_arc_start_sample_index(row.observation);
                    let carrier_phase_arc_bias =
                        carrier_phase_arc_start_sample_index.and_then(|arc_start| {
                            carrier_phase_arc_bias_cycles.get(&arc_start).copied()
                        });

                    SyntheticObservationTruthTableEpoch {
                        artifact_id: row.artifact_id.clone(),
                        epoch_id: row.epoch_id.clone(),
                        epoch_index: row.epoch_index,
                        sample_index: row.sample_index,
                        observation_status: row.observation.observation_status,
                        observation_reject_reasons: row
                            .observation
                            .observation_reject_reasons
                            .clone(),
                        pseudorange_m: SyntheticObservationTruthTableValue {
                            truth: pseudorange_truth_m,
                            measured: row.observation.pseudorange_m.0,
                            sigma: Some(row.observation.pseudorange_var_m2.sqrt()),
                            residual: pseudorange_truth_m
                                .map(|truth_m| row.observation.pseudorange_m.0 - truth_m),
                        },
                        carrier_phase_cycles: SyntheticObservationTruthTableValue {
                            truth: carrier_phase_truth_cycles,
                            measured: row.observation.carrier_phase_cycles.0,
                            sigma: Some(row.observation.carrier_phase_var_cycles2.sqrt()),
                            residual: carrier_phase_truth_cycles.zip(carrier_phase_arc_bias).map(
                                |(truth_cycles, arc_bias_cycles)| {
                                    row.observation.carrier_phase_cycles.0
                                        - truth_cycles
                                        - arc_bias_cycles
                                },
                            ),
                        },
                        carrier_phase_arc_start_sample_index,
                        carrier_phase_arc_bias_cycles: carrier_phase_arc_bias,
                        doppler_hz: SyntheticObservationTruthTableValue {
                            truth: Some(expected_doppler_hz),
                            measured: row.observation.doppler_hz.0,
                            sigma: Some(row.observation.doppler_var_hz2.sqrt()),
                            residual: Some(row.observation.doppler_hz.0 - expected_doppler_hz),
                        },
                        cn0_db_hz: SyntheticObservationTruthTableValue {
                            truth: Some(sat_truth.cn0_db_hz as f64),
                            measured: row.observation.cn0_dbhz,
                            sigma: row
                                .observation
                                .metadata
                                .tracking_uncertainty
                                .as_ref()
                                .map(|uncertainty| uncertainty.cn0_dbhz),
                            residual: Some(row.observation.cn0_dbhz - sat_truth.cn0_db_hz as f64),
                        },
                    }
                })
                .collect::<Vec<_>>();

            SyntheticObservationTruthTableSatellite {
                sat: sat_truth.sat,
                injected_doppler_hz: sat_truth.doppler_hz,
                expected_measured_doppler_hz: expected_doppler_hz,
                injected_code_phase_chips: sat_truth.code_phase_chips,
                injected_cn0_db_hz: sat_truth.cn0_db_hz,
                epoch_count: epochs.len(),
                carrier_phase_arcs_evaluated: carrier_phase_arc_bias_cycles.len(),
                notes,
                epochs,
            }
        })
        .collect();

    SyntheticObservationTruthTableReport {
        scenario_id: truth.id.clone(),
        sample_rate_hz: truth.sample_rate_hz,
        hatch_window,
        reference_receive_time_s: reference.receive_time_s,
        satellites,
    }
}

/// Measure emitted observation values against synthetic truth on a per-satellite basis.
pub fn validate_truth_guided_observations(
    config: &ReceiverPipelineConfig,
    tracks: &[crate::pipeline::tracking::TrackingResult],
    truth: &SyntheticScenario,
    reference: &SyntheticObservationTruthReference,
    hatch_window: u32,
) -> SyntheticObservationValidationReport {
    let table =
        validate_truth_guided_observation_table(config, tracks, truth, reference, hatch_window);
    let satellites = table
        .satellites
        .iter()
        .map(|satellite| SyntheticObservationValidationSatellite {
            sat: satellite.sat,
            pseudorange_error_m: summarize_observation_errors(
                &satellite
                    .epochs
                    .iter()
                    .filter_map(|epoch| epoch.pseudorange_m.residual)
                    .collect::<Vec<_>>(),
            ),
            carrier_phase_error_cycles: summarize_observation_errors(
                &satellite
                    .epochs
                    .iter()
                    .filter_map(|epoch| epoch.carrier_phase_cycles.residual)
                    .collect::<Vec<_>>(),
            ),
            carrier_phase_arcs_evaluated: satellite.carrier_phase_arcs_evaluated,
            doppler_error_hz: summarize_observation_errors(
                &satellite
                    .epochs
                    .iter()
                    .filter_map(|epoch| epoch.doppler_hz.residual)
                    .collect::<Vec<_>>(),
            ),
            cn0_error_db_hz: summarize_observation_errors(
                &satellite
                    .epochs
                    .iter()
                    .filter_map(|epoch| epoch.cn0_db_hz.residual)
                    .collect::<Vec<_>>(),
            ),
            notes: satellite.notes.clone(),
        })
        .collect();

    SyntheticObservationValidationReport {
        scenario_id: table.scenario_id,
        sample_rate_hz: table.sample_rate_hz,
        hatch_window: table.hatch_window,
        reference_receive_time_s: table.reference_receive_time_s,
        satellites,
    }
}

/// Return the hard truth-guided receiver accuracy budgets used in synthetic validation.
pub fn truth_guided_receiver_accuracy_budgets() -> SyntheticReceiverAccuracyBudgets {
    SyntheticReceiverAccuracyBudgets {
        acquisition: SyntheticAcquisitionAccuracyBudget {
            max_doppler_error_hz: 500.0,
            max_code_phase_error_samples: 2,
        },
        tracking: SyntheticTrackingAccuracyBudget {
            max_carrier_error_hz: 10.0,
            max_doppler_error_hz: 10.0,
            max_code_phase_error_samples: 1.0,
            max_cn0_error_db_hz: 8.0,
        },
        observation: SyntheticObservationAccuracyBudget {
            max_pseudorange_error_m: 5.0e-2,
            max_carrier_phase_error_cycles: 1.0e-6,
            max_doppler_error_hz: 1.0e-6,
            max_cn0_error_db_hz: 1.0e-6,
        },
        pvt: SyntheticPvtAccuracyBudget {
            max_position_error_3d_m: 5.0,
            max_clock_bias_error_m: 0.5,
            max_residual_rms_m: 1.0,
            max_pdop: 3.0,
        },
    }
}

/// Evaluate whether a truth-guided acquisition report satisfies a hard accuracy budget.
pub fn validate_acquisition_accuracy_budget(
    report: &SyntheticAcquisitionTruthTableReport,
    budget: SyntheticAcquisitionAccuracyBudget,
) -> SyntheticAcquisitionAccuracyReport {
    let truth_coverage_issues = acquisition_truth_coverage_issues(report);
    let truth_coverage_ready = truth_coverage_issues.is_empty();
    let satellites = report
        .satellites
        .iter()
        .map(|satellite| {
            let pass = satellite.doppler_error_hz.is_finite()
                && satellite.code_phase_error_samples <= budget.max_code_phase_error_samples
                && satellite.doppler_error_hz <= budget.max_doppler_error_hz + f64::EPSILON;
            SyntheticAcquisitionAccuracySatellite {
                sat: satellite.sat,
                doppler_error_hz: satellite.doppler_error_hz,
                code_phase_error_samples: satellite.code_phase_error_samples,
                pass,
            }
        })
        .collect::<Vec<_>>();
    let passing_satellite_count = satellites.iter().filter(|satellite| satellite.pass).count();

    SyntheticAcquisitionAccuracyReport {
        scenario_id: report.scenario_id.clone(),
        max_doppler_error_hz: budget.max_doppler_error_hz,
        max_code_phase_error_samples: budget.max_code_phase_error_samples,
        satellite_count: satellites.len(),
        passing_satellite_count,
        truth_coverage_ready,
        truth_coverage_issues,
        pass: truth_coverage_ready
            && !satellites.is_empty()
            && passing_satellite_count == satellites.len(),
        satellites,
    }
}

/// Evaluate whether a truth-guided tracking report satisfies a hard accuracy budget.
pub fn validate_tracking_accuracy_budget(
    report: &SyntheticTrackingTruthTableReport,
    budget: SyntheticTrackingAccuracyBudget,
) -> SyntheticTrackingAccuracyReport {
    let truth_coverage_issues = tracking_truth_coverage_issues(report);
    let truth_coverage_ready = truth_coverage_issues.is_empty();
    let satellites = report
        .satellites
        .iter()
        .map(|satellite| {
            let stable_epochs = satellite
                .epochs
                .iter()
                .filter(|epoch| epoch.stable_tracking_epoch)
                .collect::<Vec<_>>();
            let max_carrier_error_hz =
                stable_epochs.iter().map(|epoch| epoch.carrier_error_hz).fold(0.0_f64, f64::max);
            let max_doppler_error_hz =
                stable_epochs.iter().map(|epoch| epoch.doppler_error_hz).fold(0.0_f64, f64::max);
            let max_code_phase_error_samples = stable_epochs
                .iter()
                .map(|epoch| epoch.code_phase_error_samples)
                .fold(0.0_f64, f64::max);
            let max_cn0_error_db_hz =
                stable_epochs.iter().map(|epoch| epoch.cn0_error_db).fold(0.0_f64, f64::max);
            let pass = !stable_epochs.is_empty()
                && max_carrier_error_hz <= budget.max_carrier_error_hz + f64::EPSILON
                && max_doppler_error_hz <= budget.max_doppler_error_hz + f64::EPSILON
                && max_code_phase_error_samples
                    <= budget.max_code_phase_error_samples + f64::EPSILON
                && max_cn0_error_db_hz <= budget.max_cn0_error_db_hz + f64::EPSILON;

            SyntheticTrackingAccuracySatellite {
                sat: satellite.sat,
                stable_epoch_count: stable_epochs.len(),
                max_carrier_error_hz,
                max_doppler_error_hz,
                max_code_phase_error_samples,
                max_cn0_error_db_hz,
                pass,
            }
        })
        .collect::<Vec<_>>();
    let passing_satellite_count = satellites.iter().filter(|satellite| satellite.pass).count();

    SyntheticTrackingAccuracyReport {
        scenario_id: report.scenario_id.clone(),
        max_carrier_error_hz: budget.max_carrier_error_hz,
        max_doppler_error_hz: budget.max_doppler_error_hz,
        max_code_phase_error_samples: budget.max_code_phase_error_samples,
        max_cn0_error_db_hz: budget.max_cn0_error_db_hz,
        satellite_count: satellites.len(),
        passing_satellite_count,
        truth_coverage_ready,
        truth_coverage_issues,
        pass: truth_coverage_ready
            && !satellites.is_empty()
            && passing_satellite_count == satellites.len(),
        satellites,
    }
}

/// Evaluate whether a truth-guided observation report satisfies a hard accuracy budget.
pub fn validate_observation_accuracy_budget(
    report: &SyntheticObservationValidationReport,
    budget: SyntheticObservationAccuracyBudget,
) -> SyntheticObservationAccuracyReport {
    let truth_coverage_issues = observation_truth_coverage_issues(report);
    let truth_coverage_ready = truth_coverage_issues.is_empty();
    let satellites = report
        .satellites
        .iter()
        .map(|satellite| {
            let max_pseudorange_error_m =
                satellite.pseudorange_error_m.as_ref().map(|stats| stats.max_abs_error);
            let max_carrier_phase_error_cycles =
                satellite.carrier_phase_error_cycles.as_ref().map(|stats| stats.max_abs_error);
            let max_doppler_error_hz =
                satellite.doppler_error_hz.as_ref().map(|stats| stats.max_abs_error);
            let max_cn0_error_db_hz =
                satellite.cn0_error_db_hz.as_ref().map(|stats| stats.max_abs_error);
            let pass = max_pseudorange_error_m
                .zip(max_carrier_phase_error_cycles)
                .zip(max_doppler_error_hz.zip(max_cn0_error_db_hz))
                .map(
                    |(
                        (pseudorange_error_m, carrier_phase_error_cycles),
                        (doppler_error_hz, cn0_error_db_hz),
                    )| {
                        pseudorange_error_m <= budget.max_pseudorange_error_m + f64::EPSILON
                            && carrier_phase_error_cycles
                                <= budget.max_carrier_phase_error_cycles + f64::EPSILON
                            && doppler_error_hz <= budget.max_doppler_error_hz + f64::EPSILON
                            && cn0_error_db_hz <= budget.max_cn0_error_db_hz + f64::EPSILON
                    },
                )
                .unwrap_or(false);

            SyntheticObservationAccuracySatellite {
                sat: satellite.sat,
                max_pseudorange_error_m,
                max_carrier_phase_error_cycles,
                max_doppler_error_hz,
                max_cn0_error_db_hz,
                pass,
            }
        })
        .collect::<Vec<_>>();
    let passing_satellite_count = satellites.iter().filter(|satellite| satellite.pass).count();

    SyntheticObservationAccuracyReport {
        scenario_id: report.scenario_id.clone(),
        max_pseudorange_error_m: budget.max_pseudorange_error_m,
        max_carrier_phase_error_cycles: budget.max_carrier_phase_error_cycles,
        max_doppler_error_hz: budget.max_doppler_error_hz,
        max_cn0_error_db_hz: budget.max_cn0_error_db_hz,
        satellite_count: satellites.len(),
        passing_satellite_count,
        truth_coverage_ready,
        truth_coverage_issues,
        pass: truth_coverage_ready
            && !satellites.is_empty()
            && passing_satellite_count == satellites.len(),
        satellites,
    }
}

/// Evaluate whether a truth-guided PVT report satisfies a hard accuracy budget.
pub fn validate_pvt_accuracy_budget(
    report: &SyntheticPvtTruthTableReport,
    budget: SyntheticPvtAccuracyBudget,
) -> SyntheticPvtAccuracyReport {
    let truth_coverage_issues = pvt_truth_coverage_issues(report);
    let truth_coverage_ready = truth_coverage_issues.is_empty();
    let epochs = report
        .epochs
        .iter()
        .map(|epoch| {
            let clock_bias_error_m = epoch.clock_bias.error_m.abs();
            let pass = epoch.enu_error_m.error_3d_m
                <= budget.max_position_error_3d_m + f64::EPSILON
                && clock_bias_error_m <= budget.max_clock_bias_error_m + f64::EPSILON
                && epoch.residual_rms_m <= budget.max_residual_rms_m + f64::EPSILON
                && epoch.dop.pdop <= budget.max_pdop + f64::EPSILON
                && epoch.valid;
            SyntheticPvtAccuracyEpoch {
                epoch_index: epoch.epoch_index,
                position_error_3d_m: epoch.enu_error_m.error_3d_m,
                clock_bias_error_m,
                residual_rms_m: epoch.residual_rms_m,
                pdop: epoch.dop.pdop,
                pass,
            }
        })
        .collect::<Vec<_>>();
    let passing_epoch_count = epochs.iter().filter(|epoch| epoch.pass).count();

    SyntheticPvtAccuracyReport {
        scenario_id: report.scenario_id.clone(),
        max_position_error_3d_m: budget.max_position_error_3d_m,
        max_clock_bias_error_m: budget.max_clock_bias_error_m,
        max_residual_rms_m: budget.max_residual_rms_m,
        max_pdop: budget.max_pdop,
        epoch_count: epochs.len(),
        passing_epoch_count,
        truth_coverage_ready,
        truth_coverage_issues,
        pass: truth_coverage_ready && !epochs.is_empty() && passing_epoch_count == epochs.len(),
        epochs,
    }
}

fn summarize_gnss_acquisition_stage(
    report: &SyntheticAcquisitionAccuracyReport,
) -> SyntheticGnssAcquisitionStageSummary {
    SyntheticGnssAcquisitionStageSummary {
        pass: report.pass,
        truth_coverage_ready: report.truth_coverage_ready,
        satellite_count: report.satellite_count,
        passing_satellite_count: report.passing_satellite_count,
        observed_max_doppler_error_hz: report
            .satellites
            .iter()
            .map(|satellite| satellite.doppler_error_hz)
            .max_by(f64::total_cmp),
        observed_max_code_phase_error_samples: report
            .satellites
            .iter()
            .map(|satellite| satellite.code_phase_error_samples)
            .max(),
        threshold_max_doppler_error_hz: report.max_doppler_error_hz,
        threshold_max_code_phase_error_samples: report.max_code_phase_error_samples,
    }
}

fn summarize_gnss_tracking_stage(
    report: &SyntheticTrackingAccuracyReport,
) -> SyntheticGnssTrackingStageSummary {
    SyntheticGnssTrackingStageSummary {
        pass: report.pass,
        truth_coverage_ready: report.truth_coverage_ready,
        satellite_count: report.satellite_count,
        passing_satellite_count: report.passing_satellite_count,
        observed_max_carrier_error_hz: report
            .satellites
            .iter()
            .map(|satellite| satellite.max_carrier_error_hz)
            .max_by(f64::total_cmp),
        observed_max_doppler_error_hz: report
            .satellites
            .iter()
            .map(|satellite| satellite.max_doppler_error_hz)
            .max_by(f64::total_cmp),
        observed_max_code_phase_error_samples: report
            .satellites
            .iter()
            .map(|satellite| satellite.max_code_phase_error_samples)
            .max_by(f64::total_cmp),
        observed_max_cn0_error_db_hz: report
            .satellites
            .iter()
            .map(|satellite| satellite.max_cn0_error_db_hz)
            .max_by(f64::total_cmp),
        threshold_max_carrier_error_hz: report.max_carrier_error_hz,
        threshold_max_doppler_error_hz: report.max_doppler_error_hz,
        threshold_max_code_phase_error_samples: report.max_code_phase_error_samples,
        threshold_max_cn0_error_db_hz: report.max_cn0_error_db_hz,
    }
}

fn summarize_gnss_observation_stage(
    report: &SyntheticObservationAccuracyReport,
) -> SyntheticGnssObservationStageSummary {
    SyntheticGnssObservationStageSummary {
        pass: report.pass,
        truth_coverage_ready: report.truth_coverage_ready,
        satellite_count: report.satellite_count,
        passing_satellite_count: report.passing_satellite_count,
        observed_max_pseudorange_error_m: report
            .satellites
            .iter()
            .filter_map(|satellite| satellite.max_pseudorange_error_m)
            .max_by(f64::total_cmp),
        observed_max_carrier_phase_error_cycles: report
            .satellites
            .iter()
            .filter_map(|satellite| satellite.max_carrier_phase_error_cycles)
            .max_by(f64::total_cmp),
        observed_max_doppler_error_hz: report
            .satellites
            .iter()
            .filter_map(|satellite| satellite.max_doppler_error_hz)
            .max_by(f64::total_cmp),
        observed_max_cn0_error_db_hz: report
            .satellites
            .iter()
            .filter_map(|satellite| satellite.max_cn0_error_db_hz)
            .max_by(f64::total_cmp),
        threshold_max_pseudorange_error_m: report.max_pseudorange_error_m,
        threshold_max_carrier_phase_error_cycles: report.max_carrier_phase_error_cycles,
        threshold_max_doppler_error_hz: report.max_doppler_error_hz,
        threshold_max_cn0_error_db_hz: report.max_cn0_error_db_hz,
    }
}

fn summarize_gnss_pvt_stage(report: &SyntheticPvtAccuracyReport) -> SyntheticGnssPvtStageSummary {
    SyntheticGnssPvtStageSummary {
        pass: report.pass,
        truth_coverage_ready: report.truth_coverage_ready,
        epoch_count: report.epoch_count,
        passing_epoch_count: report.passing_epoch_count,
        observed_max_position_error_3d_m: report
            .epochs
            .iter()
            .map(|epoch| epoch.position_error_3d_m)
            .max_by(f64::total_cmp),
        observed_max_clock_bias_error_m: report
            .epochs
            .iter()
            .map(|epoch| epoch.clock_bias_error_m)
            .max_by(f64::total_cmp),
        observed_max_residual_rms_m: report
            .epochs
            .iter()
            .map(|epoch| epoch.residual_rms_m)
            .max_by(f64::total_cmp),
        observed_max_pdop: report.epochs.iter().map(|epoch| epoch.pdop).max_by(f64::total_cmp),
        threshold_max_position_error_3d_m: report.max_position_error_3d_m,
        threshold_max_clock_bias_error_m: report.max_clock_bias_error_m,
        threshold_max_residual_rms_m: report.max_residual_rms_m,
        threshold_max_pdop: report.max_pdop,
    }
}

/// Build one final truth-guided GNSS accuracy artifact from stage-level validation reports.
pub fn build_truth_guided_gnss_accuracy_artifact(
    case: SyntheticGnssAccuracyArtifactCase<'_>,
) -> SyntheticGnssAccuracyArtifact {
    let acquisition = SyntheticGnssAcquisitionStageArtifact {
        summary: summarize_gnss_acquisition_stage(case.acquisition),
        report: case.acquisition.clone(),
    };
    let tracking = SyntheticGnssTrackingStageArtifact {
        summary: summarize_gnss_tracking_stage(case.tracking),
        report: case.tracking.clone(),
    };
    let observation = SyntheticGnssObservationStageArtifact {
        summary: summarize_gnss_observation_stage(case.observation),
        report: case.observation.clone(),
    };
    let pvt = SyntheticGnssPvtStageArtifact {
        summary: summarize_gnss_pvt_stage(case.pvt),
        report: case.pvt.clone(),
    };
    let truth_coverage_ready = acquisition.summary.truth_coverage_ready
        && tracking.summary.truth_coverage_ready
        && observation.summary.truth_coverage_ready
        && pvt.summary.truth_coverage_ready;
    let pass = acquisition.summary.pass
        && tracking.summary.pass
        && observation.summary.pass
        && pvt.summary.pass;

    SyntheticGnssAccuracyArtifact {
        schema_version: SYNTHETIC_GNSS_ACCURACY_ARTIFACT_SCHEMA_VERSION,
        scenario_id: case.scenario_id.to_string(),
        pass,
        truth_coverage_ready,
        data_source: case.data_source,
        reference_truth: case.reference_truth,
        acquisition,
        tracking,
        observation,
        pvt,
    }
}

/// Persist one final truth-guided GNSS accuracy artifact as a single JSON file.
pub fn write_truth_guided_gnss_accuracy_artifact(
    path: &std::path::Path,
    artifact: &SyntheticGnssAccuracyArtifact,
) -> Result<(), std::io::Error> {
    let payload = serde_json::to_vec_pretty(artifact)
        .map_err(|error| std::io::Error::other(format!("failed to serialize gnss accuracy artifact: {error}")))?;
    std::fs::write(path, payload)
}

/// Borrowed inputs for one synthetic PVT C/N0 profile point.
#[derive(Debug, Clone, Copy)]
pub struct SyntheticPvtCn0ProfileCase<'a> {
    /// Stable scenario identifier for this validation run.
    pub scenario_id: &'a str,
    /// Observation epochs that fed the synthetic PVT solver path.
    pub observations: &'a [ObsEpoch],
    /// Truth-guided PVT accuracy report for the same scenario.
    pub accuracy: &'a SyntheticPvtAccuracyReport,
}

/// Borrowed inputs for one synthetic PVT geometry profile point.
#[derive(Debug, Clone, Copy)]
pub struct SyntheticPvtGeometryProfileCase<'a> {
    /// Stable scenario identifier for this validation run.
    pub scenario_id: &'a str,
    /// Truth-guided PVT accuracy report for the geometry case.
    pub accuracy: &'a SyntheticPvtAccuracyReport,
}

/// Borrowed inputs for one synthetic PVT multipath profile point.
#[derive(Debug, Clone, Copy)]
pub struct SyntheticPvtMultipathProfileCase<'a> {
    /// Stable scenario identifier for this validation run.
    pub scenario_id: &'a str,
    /// Number of satellites carrying injected multipath bias.
    pub affected_satellite_count: usize,
    /// Mean absolute injected pseudorange bias across affected satellites, in meters.
    pub mean_abs_pseudorange_bias_m: f64,
    /// Maximum absolute injected pseudorange bias across affected satellites, in meters.
    pub max_abs_pseudorange_bias_m: f64,
    /// Truth-guided PVT truth table for the same scenario.
    pub truth_table: &'a SyntheticPvtTruthTableReport,
    /// Truth-guided PVT accuracy report for the same scenario.
    pub accuracy: &'a SyntheticPvtAccuracyReport,
}

/// Borrowed inputs for one synthetic PVT receiver-motion profile point.
#[derive(Debug, Clone, Copy)]
pub struct SyntheticPvtMotionProfileCase<'a> {
    /// Stable scenario identifier for this validation run.
    pub scenario_id: &'a str,
    /// Truth-guided PVT truth table for the same scenario.
    pub truth_table: &'a SyntheticPvtTruthTableReport,
    /// Truth-guided PVT accuracy report for the same scenario.
    pub accuracy: &'a SyntheticPvtAccuracyReport,
}

/// Borrowed inputs for one receiver clock-drift PVT profile point.
#[derive(Debug, Clone, Copy)]
pub struct SyntheticPvtClockProfileCase<'a> {
    /// Stable scenario identifier for this validation run.
    pub scenario_id: &'a str,
    /// Injected receiver clock drift carried by the scenario truth, in seconds per second.
    pub injected_clock_drift_s_per_s: f64,
    /// Expected observation-Doppler offset induced by the injected clock drift, in hertz.
    pub expected_observation_doppler_offset_hz: f64,
    /// Observation epochs produced by the drifting or stable receiver-clock scenario.
    pub observations: &'a [ObsEpoch],
    /// Stable-reference observation epochs used to measure the clock-induced Doppler offset.
    pub reference_observations: Option<&'a [ObsEpoch]>,
    /// Navigation solutions that should estimate the receiver clock drift.
    pub solutions: &'a [NavSolutionEpoch],
    /// Truth-guided PVT accuracy report for the same scenario.
    pub accuracy: &'a SyntheticPvtAccuracyReport,
}

/// Borrowed inputs for one long-run synthetic PVT time-profile point.
#[derive(Debug, Clone, Copy)]
pub struct SyntheticPvtTimeProfileCase<'a> {
    /// Stable scenario identifier for this validation run.
    pub scenario_id: &'a str,
    /// Truth-guided PVT truth table for the same scenario.
    pub truth_table: &'a SyntheticPvtTruthTableReport,
    /// Truth-guided PVT accuracy report for the same scenario.
    pub accuracy: &'a SyntheticPvtAccuracyReport,
}

/// Summarize truth-guided PVT accuracy across multiple signal-strength points.
pub fn summarize_truth_guided_pvt_cn0_profile(
    cases: &[SyntheticPvtCn0ProfileCase<'_>],
    scenario_id_prefix: &str,
) -> SyntheticPvtCn0ProfileReport {
    let mut points = cases
        .iter()
        .map(|case| {
            let epoch_mean_cn0 = case
                .observations
                .iter()
                .filter_map(|epoch| {
                    let values = epoch
                        .sats
                        .iter()
                        .filter_map(|sat| sat.cn0_dbhz.is_finite().then_some(sat.cn0_dbhz))
                        .collect::<Vec<_>>();
                    if values.is_empty() {
                        None
                    } else {
                        Some(values.iter().sum::<f64>() / values.len() as f64)
                    }
                })
                .collect::<Vec<_>>();
            let observation_epoch_count = epoch_mean_cn0.len();
            let mean_observation_cn0_dbhz = if epoch_mean_cn0.is_empty() {
                0.0
            } else {
                epoch_mean_cn0.iter().sum::<f64>() / epoch_mean_cn0.len() as f64
            };
            let min_observation_cn0_dbhz =
                epoch_mean_cn0.iter().copied().reduce(f64::min).unwrap_or(0.0);
            let max_observation_cn0_dbhz =
                epoch_mean_cn0.iter().copied().reduce(f64::max).unwrap_or(0.0);
            let position_error_3d_m = case
                .accuracy
                .epochs
                .iter()
                .map(|epoch| epoch.position_error_3d_m)
                .collect::<Vec<_>>();
            let clock_bias_error_m = case
                .accuracy
                .epochs
                .iter()
                .map(|epoch| epoch.clock_bias_error_m)
                .collect::<Vec<_>>();
            let residual_rms_m =
                case.accuracy.epochs.iter().map(|epoch| epoch.residual_rms_m).collect::<Vec<_>>();
            let pdop = case.accuracy.epochs.iter().map(|epoch| epoch.pdop).collect::<Vec<_>>();
            let passing_epoch_count =
                case.accuracy.epochs.iter().filter(|epoch| epoch.pass).count();
            let epoch_count = case.accuracy.epoch_count;
            let pass_rate = if epoch_count == 0 {
                0.0
            } else {
                passing_epoch_count as f64 / epoch_count as f64
            };

            SyntheticPvtCn0ProfilePoint {
                scenario_id: case.scenario_id.to_string(),
                observation_epoch_count,
                mean_observation_cn0_dbhz,
                min_observation_cn0_dbhz,
                max_observation_cn0_dbhz,
                epoch_count,
                passing_epoch_count,
                pass_rate,
                rms_position_error_3d_m: (!position_error_3d_m.is_empty())
                    .then(|| stats(&position_error_3d_m).rms),
                max_position_error_3d_m: position_error_3d_m.iter().copied().reduce(f64::max),
                rms_clock_bias_error_m: (!clock_bias_error_m.is_empty())
                    .then(|| stats(&clock_bias_error_m).rms),
                max_clock_bias_error_m: clock_bias_error_m.iter().copied().reduce(f64::max),
                rms_residual_rms_m: (!residual_rms_m.is_empty())
                    .then(|| stats(&residual_rms_m).rms),
                max_residual_rms_m: residual_rms_m.iter().copied().reduce(f64::max),
                max_pdop: pdop.iter().copied().reduce(f64::max),
                truth_coverage_ready: case.accuracy.truth_coverage_ready,
                truth_coverage_issues: case.accuracy.truth_coverage_issues.clone(),
                ready: case.accuracy.truth_coverage_ready
                    && observation_epoch_count > 0
                    && epoch_count > 0,
            }
        })
        .collect::<Vec<_>>();
    points.sort_by(|left, right| {
        left.mean_observation_cn0_dbhz
            .partial_cmp(&right.mean_observation_cn0_dbhz)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then_with(|| left.scenario_id.cmp(&right.scenario_id))
    });

    SyntheticPvtCn0ProfileReport { scenario_id_prefix: scenario_id_prefix.to_string(), points }
}

/// Summarize truth-guided PVT accuracy across multiple satellite-geometry points.
pub fn summarize_truth_guided_pvt_geometry_profile(
    cases: &[SyntheticPvtGeometryProfileCase<'_>],
    scenario_id_prefix: &str,
) -> SyntheticPvtGeometryProfileReport {
    let mut points = cases
        .iter()
        .map(|case| {
            let position_error_3d_m = case
                .accuracy
                .epochs
                .iter()
                .map(|epoch| epoch.position_error_3d_m)
                .collect::<Vec<_>>();
            let clock_bias_error_m = case
                .accuracy
                .epochs
                .iter()
                .map(|epoch| epoch.clock_bias_error_m)
                .collect::<Vec<_>>();
            let residual_rms_m =
                case.accuracy.epochs.iter().map(|epoch| epoch.residual_rms_m).collect::<Vec<_>>();
            let pdop = case.accuracy.epochs.iter().map(|epoch| epoch.pdop).collect::<Vec<_>>();
            let passing_epoch_count =
                case.accuracy.epochs.iter().filter(|epoch| epoch.pass).count();
            let epoch_count = case.accuracy.epoch_count;
            let pass_rate = if epoch_count == 0 {
                0.0
            } else {
                passing_epoch_count as f64 / epoch_count as f64
            };

            SyntheticPvtGeometryProfilePoint {
                scenario_id: case.scenario_id.to_string(),
                epoch_count,
                passing_epoch_count,
                pass_rate,
                mean_pdop: mean_f64(&pdop),
                min_pdop: pdop.iter().copied().reduce(f64::min),
                max_pdop: pdop.iter().copied().reduce(f64::max),
                rms_position_error_3d_m: (!position_error_3d_m.is_empty())
                    .then(|| stats(&position_error_3d_m).rms),
                max_position_error_3d_m: position_error_3d_m.iter().copied().reduce(f64::max),
                rms_clock_bias_error_m: (!clock_bias_error_m.is_empty())
                    .then(|| stats(&clock_bias_error_m).rms),
                max_clock_bias_error_m: clock_bias_error_m.iter().copied().reduce(f64::max),
                rms_residual_rms_m: (!residual_rms_m.is_empty())
                    .then(|| stats(&residual_rms_m).rms),
                max_residual_rms_m: residual_rms_m.iter().copied().reduce(f64::max),
                truth_coverage_ready: case.accuracy.truth_coverage_ready,
                truth_coverage_issues: case.accuracy.truth_coverage_issues.clone(),
                ready: case.accuracy.truth_coverage_ready && !pdop.is_empty() && epoch_count > 0,
            }
        })
        .collect::<Vec<_>>();
    points.sort_by(|left, right| {
        left.mean_pdop
            .partial_cmp(&right.mean_pdop)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then_with(|| left.scenario_id.cmp(&right.scenario_id))
    });

    SyntheticPvtGeometryProfileReport { scenario_id_prefix: scenario_id_prefix.to_string(), points }
}

/// Summarize truth-guided PVT accuracy across multiple synthetic multipath points.
pub fn summarize_truth_guided_pvt_multipath_profile(
    cases: &[SyntheticPvtMultipathProfileCase<'_>],
    scenario_id_prefix: &str,
) -> SyntheticPvtMultipathProfileReport {
    let mut points = cases
        .iter()
        .map(|case| {
            let position_error_3d_m = case
                .accuracy
                .epochs
                .iter()
                .map(|epoch| epoch.position_error_3d_m)
                .collect::<Vec<_>>();
            let residual_rms_m =
                case.accuracy.epochs.iter().map(|epoch| epoch.residual_rms_m).collect::<Vec<_>>();
            let passing_epoch_count =
                case.accuracy.epochs.iter().filter(|epoch| epoch.pass).count();
            let epoch_count = case.accuracy.epoch_count;
            let pass_rate = if epoch_count == 0 {
                0.0
            } else {
                passing_epoch_count as f64 / epoch_count as f64
            };
            let stable_epoch_count = case
                .truth_table
                .epochs
                .iter()
                .filter(|epoch| epoch.solution_validity == SolutionValidity::Stable)
                .count();
            let diverging_epoch_count = case
                .truth_table
                .epochs
                .iter()
                .filter(|epoch| epoch.solution_validity == SolutionValidity::Diverging)
                .count();
            let truth_epoch_count = case.truth_table.epochs.len();
            let stable_epoch_rate = if truth_epoch_count == 0 {
                0.0
            } else {
                stable_epoch_count as f64 / truth_epoch_count as f64
            };
            let diverging_epoch_rate = if truth_epoch_count == 0 {
                0.0
            } else {
                diverging_epoch_count as f64 / truth_epoch_count as f64
            };

            SyntheticPvtMultipathProfilePoint {
                scenario_id: case.scenario_id.to_string(),
                affected_satellite_count: case.affected_satellite_count,
                mean_abs_pseudorange_bias_m: case.mean_abs_pseudorange_bias_m,
                max_abs_pseudorange_bias_m: case.max_abs_pseudorange_bias_m,
                epoch_count,
                passing_epoch_count,
                pass_rate,
                stable_epoch_count,
                stable_epoch_rate,
                diverging_epoch_count,
                diverging_epoch_rate,
                rms_position_error_3d_m: (!position_error_3d_m.is_empty())
                    .then(|| stats(&position_error_3d_m).rms),
                max_position_error_3d_m: position_error_3d_m.iter().copied().reduce(f64::max),
                rms_residual_rms_m: (!residual_rms_m.is_empty())
                    .then(|| stats(&residual_rms_m).rms),
                max_residual_rms_m: residual_rms_m.iter().copied().reduce(f64::max),
                truth_coverage_ready: case.accuracy.truth_coverage_ready,
                truth_coverage_issues: case.accuracy.truth_coverage_issues.clone(),
                ready: case.accuracy.truth_coverage_ready
                    && truth_epoch_count > 0
                    && epoch_count > 0,
            }
        })
        .collect::<Vec<_>>();
    points.sort_by(|left, right| {
        left.max_abs_pseudorange_bias_m
            .partial_cmp(&right.max_abs_pseudorange_bias_m)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then_with(|| left.scenario_id.cmp(&right.scenario_id))
    });

    SyntheticPvtMultipathProfileReport {
        scenario_id_prefix: scenario_id_prefix.to_string(),
        points,
    }
}

/// Summarize truth-guided PVT accuracy across multiple receiver-motion truth paths.
pub fn summarize_truth_guided_pvt_motion_profile(
    cases: &[SyntheticPvtMotionProfileCase<'_>],
    scenario_id_prefix: &str,
) -> SyntheticPvtMotionProfileReport {
    let mut points = cases
        .iter()
        .map(|case| {
            let position_error_3d_m = case
                .accuracy
                .epochs
                .iter()
                .map(|epoch| epoch.position_error_3d_m)
                .collect::<Vec<_>>();
            let residual_rms_m =
                case.accuracy.epochs.iter().map(|epoch| epoch.residual_rms_m).collect::<Vec<_>>();
            let passing_epoch_count =
                case.accuracy.epochs.iter().filter(|epoch| epoch.pass).count();
            let epoch_count = case.accuracy.epoch_count;
            let pass_rate = if epoch_count == 0 {
                0.0
            } else {
                passing_epoch_count as f64 / epoch_count as f64
            };
            let stable_epoch_count = case
                .truth_table
                .epochs
                .iter()
                .filter(|epoch| epoch.solution_validity == SolutionValidity::Stable)
                .count();
            let truth_epoch_count = case.truth_table.epochs.len();
            let stable_epoch_rate = if truth_epoch_count == 0 {
                0.0
            } else {
                stable_epoch_count as f64 / truth_epoch_count as f64
            };
            let truth_path_segments = case
                .truth_table
                .epochs
                .windows(2)
                .map(|window| {
                    let previous = &window[0].truth_ecef_m;
                    let current = &window[1].truth_ecef_m;
                    let dx = current.x_m - previous.x_m;
                    let dy = current.y_m - previous.y_m;
                    let dz = current.z_m - previous.z_m;
                    let dt_s = window[1].receive_time_s - window[0].receive_time_s;
                    ((dx * dx + dy * dy + dz * dz).sqrt(), dt_s)
                })
                .collect::<Vec<_>>();
            let path_length_m =
                truth_path_segments.iter().map(|(segment_length_m, _)| *segment_length_m).sum();
            let duration_s = truth_path_segments.iter().map(|(_, dt_s)| *dt_s).sum::<f64>();
            let mean_speed_mps = if duration_s <= 0.0 { 0.0 } else { path_length_m / duration_s };

            SyntheticPvtMotionProfilePoint {
                scenario_id: case.scenario_id.to_string(),
                truth_epoch_count,
                moving: path_length_m > 0.0,
                path_length_m,
                mean_speed_mps,
                epoch_count,
                passing_epoch_count,
                pass_rate,
                stable_epoch_count,
                stable_epoch_rate,
                rms_position_error_3d_m: (!position_error_3d_m.is_empty())
                    .then(|| stats(&position_error_3d_m).rms),
                max_position_error_3d_m: position_error_3d_m.iter().copied().reduce(f64::max),
                rms_residual_rms_m: (!residual_rms_m.is_empty())
                    .then(|| stats(&residual_rms_m).rms),
                max_residual_rms_m: residual_rms_m.iter().copied().reduce(f64::max),
                truth_coverage_ready: case.accuracy.truth_coverage_ready,
                truth_coverage_issues: case.accuracy.truth_coverage_issues.clone(),
                ready: case.accuracy.truth_coverage_ready
                    && truth_epoch_count > 0
                    && epoch_count > 0,
            }
        })
        .collect::<Vec<_>>();
    points.sort_by(|left, right| {
        left.path_length_m
            .partial_cmp(&right.path_length_m)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then_with(|| {
                left.mean_speed_mps
                    .partial_cmp(&right.mean_speed_mps)
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .then_with(|| left.scenario_id.cmp(&right.scenario_id))
    });

    SyntheticPvtMotionProfileReport { scenario_id_prefix: scenario_id_prefix.to_string(), points }
}

/// Summarize truth-guided PVT accuracy across multiple injected receiver clock-drift points.
pub fn summarize_truth_guided_pvt_clock_profile(
    cases: &[SyntheticPvtClockProfileCase<'_>],
    scenario_id_prefix: &str,
) -> SyntheticPvtClockProfileReport {
    let mut points = cases
        .iter()
        .map(|case| {
            let observation_doppler_offsets_hz = case
                .reference_observations
                .map(|reference_observations| {
                    observation_doppler_offsets_hz(case.observations, reference_observations)
                })
                .unwrap_or_default();
            let solved_clock_drift_s_per_s = case
                .solutions
                .iter()
                .map(|solution| solution.clock_drift_s_per_s)
                .filter(|value| value.is_finite())
                .collect::<Vec<_>>();
            let clock_drift_error_s_per_s = solved_clock_drift_s_per_s
                .iter()
                .map(|value| (value - case.injected_clock_drift_s_per_s).abs())
                .collect::<Vec<_>>();
            let clock_bias_error_m = case
                .accuracy
                .epochs
                .iter()
                .map(|epoch| epoch.clock_bias_error_m)
                .collect::<Vec<_>>();
            let residual_rms_m =
                case.accuracy.epochs.iter().map(|epoch| epoch.residual_rms_m).collect::<Vec<_>>();
            let passing_epoch_count =
                case.accuracy.epochs.iter().filter(|epoch| epoch.pass).count();
            let epoch_count = case.accuracy.epoch_count;
            let pass_rate = if epoch_count == 0 {
                0.0
            } else {
                passing_epoch_count as f64 / epoch_count as f64
            };

            SyntheticPvtClockProfilePoint {
                scenario_id: case.scenario_id.to_string(),
                injected_clock_drift_s_per_s: case.injected_clock_drift_s_per_s,
                expected_observation_doppler_offset_hz: case.expected_observation_doppler_offset_hz,
                epoch_count,
                passing_epoch_count,
                pass_rate,
                observation_doppler_pair_count: observation_doppler_offsets_hz.len(),
                observed_mean_observation_doppler_offset_hz: mean_f64(
                    &observation_doppler_offsets_hz,
                ),
                observed_min_observation_doppler_offset_hz: observation_doppler_offsets_hz
                    .iter()
                    .copied()
                    .reduce(f64::min),
                observed_max_observation_doppler_offset_hz: observation_doppler_offsets_hz
                    .iter()
                    .copied()
                    .reduce(f64::max),
                max_observation_doppler_offset_error_hz: observation_doppler_offsets_hz
                    .iter()
                    .map(|value| (value - case.expected_observation_doppler_offset_hz).abs())
                    .reduce(f64::max),
                final_solved_clock_drift_s_per_s: solved_clock_drift_s_per_s.last().copied(),
                mean_solved_clock_drift_s_per_s: mean_f64(&solved_clock_drift_s_per_s),
                max_clock_drift_error_s_per_s: clock_drift_error_s_per_s
                    .iter()
                    .copied()
                    .reduce(f64::max),
                rms_clock_bias_error_m: (!clock_bias_error_m.is_empty())
                    .then(|| stats(&clock_bias_error_m).rms),
                max_clock_bias_error_m: clock_bias_error_m.iter().copied().reduce(f64::max),
                rms_residual_rms_m: (!residual_rms_m.is_empty())
                    .then(|| stats(&residual_rms_m).rms),
                max_residual_rms_m: residual_rms_m.iter().copied().reduce(f64::max),
                truth_coverage_ready: case.accuracy.truth_coverage_ready,
                truth_coverage_issues: case.accuracy.truth_coverage_issues.clone(),
                ready: case.accuracy.truth_coverage_ready
                    && epoch_count > 0
                    && !solved_clock_drift_s_per_s.is_empty()
                    && (case.reference_observations.is_some()
                        || case.expected_observation_doppler_offset_hz.abs() <= f64::EPSILON),
            }
        })
        .collect::<Vec<_>>();
    points.sort_by(|left, right| {
        left.injected_clock_drift_s_per_s
            .partial_cmp(&right.injected_clock_drift_s_per_s)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then_with(|| left.scenario_id.cmp(&right.scenario_id))
    });

    SyntheticPvtClockProfileReport { scenario_id_prefix: scenario_id_prefix.to_string(), points }
}

fn observation_doppler_offsets_hz(
    observations: &[ObsEpoch],
    reference_observations: &[ObsEpoch],
) -> Vec<f64> {
    let reference_by_epoch = reference_observations
        .iter()
        .map(|epoch| {
            let sats = epoch
                .sats
                .iter()
                .map(|sat| (sat.signal_id, sat.doppler_hz.0))
                .collect::<BTreeMap<_, _>>();
            (epoch.epoch_idx, sats)
        })
        .collect::<BTreeMap<_, _>>();
    let mut offsets_hz = Vec::new();
    for epoch in observations {
        let Some(reference_sats) = reference_by_epoch.get(&epoch.epoch_idx) else {
            continue;
        };
        for sat in &epoch.sats {
            if let Some(reference_doppler_hz) = reference_sats.get(&sat.signal_id) {
                offsets_hz.push(sat.doppler_hz.0 - reference_doppler_hz);
            }
        }
    }
    offsets_hz
}

/// Summarize truth-guided PVT accuracy across long-run time-evolution points.
pub fn summarize_truth_guided_pvt_time_profile(
    cases: &[SyntheticPvtTimeProfileCase<'_>],
    scenario_id_prefix: &str,
) -> SyntheticPvtTimeProfileReport {
    let mut points = cases
        .iter()
        .map(|case| {
            let epoch_count = case.accuracy.epoch_count;
            let passing_epoch_count =
                case.accuracy.epochs.iter().filter(|epoch| epoch.pass).count();
            let pass_rate = if epoch_count == 0 {
                0.0
            } else {
                passing_epoch_count as f64 / epoch_count as f64
            };
            let stable_epoch_count = case
                .truth_table
                .epochs
                .iter()
                .filter(|epoch| epoch.solution_validity == SolutionValidity::Stable)
                .count();
            let diverging_epoch_count = case
                .truth_table
                .epochs
                .iter()
                .filter(|epoch| epoch.solution_validity == SolutionValidity::Diverging)
                .count();
            let truth_epoch_count = case.truth_table.epochs.len();
            let stable_epoch_rate = if truth_epoch_count == 0 {
                0.0
            } else {
                stable_epoch_count as f64 / truth_epoch_count as f64
            };
            let diverging_epoch_rate = if truth_epoch_count == 0 {
                0.0
            } else {
                diverging_epoch_count as f64 / truth_epoch_count as f64
            };
            let receive_times_s = case
                .truth_table
                .epochs
                .iter()
                .map(|epoch| epoch.receive_time_s)
                .collect::<Vec<_>>();
            let duration_s = match (receive_times_s.first(), receive_times_s.last()) {
                (Some(first), Some(last)) if last > first => last - first,
                _ => 0.0,
            };
            let position_error_3d_m = case
                .accuracy
                .epochs
                .iter()
                .map(|epoch| epoch.position_error_3d_m)
                .collect::<Vec<_>>();
            let residual_rms_m =
                case.accuracy.epochs.iter().map(|epoch| epoch.residual_rms_m).collect::<Vec<_>>();
            let window_len = time_profile_window_len(epoch_count);
            let first_window_mean_position_error_3d_m =
                mean_f64(&position_error_3d_m[..window_len.min(position_error_3d_m.len())]);
            let last_window_mean_position_error_3d_m = if position_error_3d_m.is_empty() {
                None
            } else {
                mean_f64(
                    &position_error_3d_m
                        [position_error_3d_m.len() - window_len.min(position_error_3d_m.len())..],
                )
            };
            let first_window_mean_residual_rms_m =
                mean_f64(&residual_rms_m[..window_len.min(residual_rms_m.len())]);
            let last_window_mean_residual_rms_m = if residual_rms_m.is_empty() {
                None
            } else {
                mean_f64(
                    &residual_rms_m[residual_rms_m.len() - window_len.min(residual_rms_m.len())..],
                )
            };
            let position_error_drift_m_per_s =
                linear_trend_slope(&receive_times_s, &position_error_3d_m);
            let residual_rms_drift_m_per_s = linear_trend_slope(&receive_times_s, &residual_rms_m);
            let trend = classify_pvt_time_trend(
                diverging_epoch_count,
                first_window_mean_position_error_3d_m,
                last_window_mean_position_error_3d_m,
                position_error_drift_m_per_s,
                first_window_mean_residual_rms_m,
                last_window_mean_residual_rms_m,
                residual_rms_drift_m_per_s,
            );

            SyntheticPvtTimeProfilePoint {
                scenario_id: case.scenario_id.to_string(),
                truth_epoch_count,
                duration_s,
                epoch_count,
                passing_epoch_count,
                pass_rate,
                stable_epoch_count,
                stable_epoch_rate,
                diverging_epoch_count,
                diverging_epoch_rate,
                first_window_mean_position_error_3d_m,
                last_window_mean_position_error_3d_m,
                position_error_drift_m_per_s,
                first_window_mean_residual_rms_m,
                last_window_mean_residual_rms_m,
                residual_rms_drift_m_per_s,
                trend,
                truth_coverage_ready: case.accuracy.truth_coverage_ready,
                truth_coverage_issues: case.accuracy.truth_coverage_issues.clone(),
                ready: case.accuracy.truth_coverage_ready
                    && truth_epoch_count >= 2
                    && epoch_count >= 2,
            }
        })
        .collect::<Vec<_>>();
    points.sort_by(|left, right| {
        left.duration_s
            .partial_cmp(&right.duration_s)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then_with(|| left.scenario_id.cmp(&right.scenario_id))
    });

    SyntheticPvtTimeProfileReport { scenario_id_prefix: scenario_id_prefix.to_string(), points }
}

/// Merge acquisition, tracking, and PVT C/N0 profiles into one stage-level output.
pub fn summarize_truth_guided_accuracy_cn0_profile(
    scenario_id_prefix: &str,
    acquisition: &SyntheticAcquisitionDetectionRateReport,
    tracking: &SyntheticTrackingLockRateReport,
    pvt: &SyntheticPvtCn0ProfileReport,
) -> SyntheticAccuracyCn0ProfileReport {
    #[derive(Default)]
    struct Accumulator {
        acquisition_trial_count: usize,
        acquisition_acceptance_probability: Vec<f64>,
        acquisition_detection_probability: Vec<f64>,
        tracking_trial_count: usize,
        tracking_lock_probability: Vec<f64>,
        tracking_refused_lock_rate: Vec<f64>,
        pvt_epoch_count: usize,
        pvt_pass_rate: Vec<f64>,
        pvt_rms_position_error_3d_m: Vec<f64>,
        pvt_max_position_error_3d_m: Vec<f64>,
    }

    let mut by_cn0 = BTreeMap::<i32, Accumulator>::new();
    for point in &acquisition.points {
        let entry = by_cn0.entry((point.cn0_db_hz * 10.0).round() as i32).or_default();
        entry.acquisition_trial_count += point.trial_count;
        entry.acquisition_acceptance_probability.push(point.acceptance_probability);
        entry.acquisition_detection_probability.push(point.detection_probability);
    }
    for point in &tracking.points {
        let entry = by_cn0.entry((point.cn0_db_hz * 10.0).round() as i32).or_default();
        entry.tracking_trial_count += point.trial_count;
        entry.tracking_lock_probability.push(point.lock_probability);
        if point.trial_count > 0 {
            entry
                .tracking_refused_lock_rate
                .push(point.refused_lock_count as f64 / point.trial_count as f64);
        }
    }
    for point in &pvt.points {
        let entry =
            by_cn0.entry((point.mean_observation_cn0_dbhz * 10.0).round() as i32).or_default();
        entry.pvt_epoch_count += point.epoch_count;
        entry.pvt_pass_rate.push(point.pass_rate);
        if let Some(rms_position_error_3d_m) = point.rms_position_error_3d_m {
            entry.pvt_rms_position_error_3d_m.push(rms_position_error_3d_m);
        }
        if let Some(max_position_error_3d_m) = point.max_position_error_3d_m {
            entry.pvt_max_position_error_3d_m.push(max_position_error_3d_m);
        }
    }

    let points = by_cn0
        .into_iter()
        .map(|(cn0_tenths_db_hz, values)| SyntheticAccuracyCn0ProfilePoint {
            cn0_db_hz: cn0_tenths_db_hz as f64 / 10.0,
            acquisition_case_count: values.acquisition_detection_probability.len(),
            acquisition_trial_count: values.acquisition_trial_count,
            acquisition_acceptance_probability_mean: mean_f64(
                &values.acquisition_acceptance_probability,
            ),
            acquisition_detection_probability_mean: mean_f64(
                &values.acquisition_detection_probability,
            ),
            tracking_case_count: values.tracking_lock_probability.len(),
            tracking_trial_count: values.tracking_trial_count,
            tracking_lock_probability_mean: mean_f64(&values.tracking_lock_probability),
            tracking_refused_lock_rate_mean: mean_f64(&values.tracking_refused_lock_rate),
            pvt_case_count: values.pvt_pass_rate.len(),
            pvt_epoch_count: values.pvt_epoch_count,
            pvt_pass_rate_mean: mean_f64(&values.pvt_pass_rate),
            pvt_rms_position_error_3d_m_mean: mean_f64(&values.pvt_rms_position_error_3d_m),
            pvt_max_position_error_3d_m_max: values
                .pvt_max_position_error_3d_m
                .iter()
                .copied()
                .reduce(f64::max),
        })
        .collect::<Vec<_>>();

    SyntheticAccuracyCn0ProfileReport { scenario_id_prefix: scenario_id_prefix.to_string(), points }
}

/// Build a truth-guided PVT table from synthetic navigation solutions.
pub fn validate_truth_guided_pvt_table(
    scenario_id: &str,
    solutions: &[NavSolutionEpoch],
    reference: &[SyntheticPvtTruthReferenceEpoch],
) -> SyntheticPvtTruthTableReport {
    let reference_by_epoch =
        reference.iter().map(|epoch| (epoch.position.epoch_idx, epoch)).collect::<BTreeMap<_, _>>();
    let mut epochs = Vec::new();
    let mut unmatched_solution_epochs = Vec::new();
    let mut matched_epoch_indices = std::collections::BTreeSet::new();

    for solution in solutions {
        let Some(reference_epoch) = reference_by_epoch.get(&solution.epoch.index) else {
            unmatched_solution_epochs.push(solution.epoch.index);
            continue;
        };

        matched_epoch_indices.insert(solution.epoch.index);
        let truth_ecef = reference_ecef(&reference_epoch.position);
        let truth_geodetic = ecef_to_geodetic(truth_ecef.0, truth_ecef.1, truth_ecef.2);
        let (east_m, north_m, up_m) = ecef_to_enu(
            solution.ecef_x_m.0,
            solution.ecef_y_m.0,
            solution.ecef_z_m.0,
            reference_epoch.position.latitude_deg,
            reference_epoch.position.longitude_deg,
            reference_epoch.position.altitude_m,
        );
        let horiz_m = (east_m * east_m + north_m * north_m).sqrt();
        let vert_m = up_m.abs();
        let error_3d_m = (horiz_m * horiz_m + up_m * up_m).sqrt();
        let clock_bias_truth_m = reference_epoch.clock_bias_s * SPEED_OF_LIGHT_MPS;

        epochs.push(SyntheticPvtTruthTableEpoch {
            artifact_id: solution.artifact_id.clone(),
            source_observation_epoch_id: solution.source_observation_epoch_id.clone(),
            epoch_index: solution.epoch.index,
            receive_time_s: solution.t_rx_s.0,
            truth_ecef_m: SyntheticPvtTruthTableEcef {
                x_m: truth_ecef.0,
                y_m: truth_ecef.1,
                z_m: truth_ecef.2,
            },
            measured_ecef_m: SyntheticPvtTruthTableEcef {
                x_m: solution.ecef_x_m.0,
                y_m: solution.ecef_y_m.0,
                z_m: solution.ecef_z_m.0,
            },
            ecef_error_m: SyntheticPvtTruthTableEcef {
                x_m: solution.ecef_x_m.0 - truth_ecef.0,
                y_m: solution.ecef_y_m.0 - truth_ecef.1,
                z_m: solution.ecef_z_m.0 - truth_ecef.2,
            },
            truth_geodetic: SyntheticPvtTruthTableGeodetic {
                latitude_deg: truth_geodetic.0,
                longitude_deg: truth_geodetic.1,
                altitude_m: truth_geodetic.2,
            },
            measured_geodetic: SyntheticPvtTruthTableGeodetic {
                latitude_deg: solution.latitude_deg,
                longitude_deg: solution.longitude_deg,
                altitude_m: solution.altitude_m.0,
            },
            enu_error_m: SyntheticPvtTruthTableEnuError {
                east_m,
                north_m,
                up_m,
                horiz_m,
                vert_m,
                error_3d_m,
            },
            clock_bias: SyntheticPvtTruthTableClockBias {
                truth_s: reference_epoch.clock_bias_s,
                measured_s: solution.clock_bias_s.0,
                error_s: solution.clock_bias_s.0 - reference_epoch.clock_bias_s,
                truth_m: clock_bias_truth_m,
                measured_m: solution.clock_bias_m.0,
                error_m: solution.clock_bias_m.0 - clock_bias_truth_m,
            },
            residual_rms_m: solution.rms_m.0,
            pre_fit_residual_rms_m: solution.pre_fit_residual_rms_m.map(|value| value.0),
            post_fit_residual_rms_m: solution.post_fit_residual_rms_m.map(|value| value.0),
            dop: SyntheticPvtTruthTableDop {
                pdop: solution.pdop,
                hdop: solution.hdop,
                vdop: solution.vdop,
                gdop: solution.gdop,
                tdop: solution.tdop,
            },
            solution_status: solution.status,
            solution_quality: solution.quality,
            solution_validity: solution.validity,
            valid: solution.valid,
            sat_count: solution.sat_count,
            used_sat_count: solution.used_sat_count,
            rejected_sat_count: solution.rejected_sat_count,
        });
    }

    let unused_reference_epochs = reference
        .iter()
        .map(|epoch| epoch.position.epoch_idx)
        .filter(|epoch_idx| !matched_epoch_indices.contains(epoch_idx))
        .collect::<Vec<_>>();

    SyntheticPvtTruthTableReport {
        scenario_id: scenario_id.to_string(),
        solution_count: solutions.len(),
        matched_epoch_count: epochs.len(),
        unmatched_solution_epochs,
        unused_reference_epochs,
        epochs,
    }
}

fn carrier_phase_arc_start_sample_index(
    observation: &bijux_gnss_core::api::ObsSatellite,
) -> Option<u64> {
    (observation.lock_flags.carrier_lock
        && observation.metadata.carrier_phase_continuity != "unusable")
        .then_some(observation.metadata.carrier_phase_arc_start_sample_index)
}

fn carrier_phase_arc_bias_cycles_by_start_sample(
    config: &ReceiverPipelineConfig,
    sat_state: &SatState,
    observed_rows: &[ObservedSatelliteRow<'_>],
) -> BTreeMap<u64, f64> {
    let mut differences_by_arc = BTreeMap::<u64, Vec<f64>>::new();
    for row in observed_rows {
        let Some(arc_start_sample_index) = carrier_phase_arc_start_sample_index(row.observation)
        else {
            continue;
        };
        let truth_cycles = sat_state
            .carrier_phase_rad_at(row.sample_index as f64 / config.sampling_freq_hz)
            / std::f64::consts::TAU;
        differences_by_arc
            .entry(arc_start_sample_index)
            .or_default()
            .push(row.observation.carrier_phase_cycles.0 - truth_cycles);
    }

    differences_by_arc
        .into_iter()
        .map(|(arc_start_sample_index, differences)| {
            (arc_start_sample_index, stats(&differences).median)
        })
        .collect()
}

fn comparable_satellite_observations<'a>(
    observations: &'a [ObsEpoch],
    sat: SatId,
) -> Vec<ObservedSatelliteRow<'a>> {
    observations
        .iter()
        .filter(|epoch| epoch.valid)
        .flat_map(|epoch| {
            let artifact_id = epoch
                .manifest
                .as_ref()
                .map(|manifest| manifest.artifact_id.clone())
                .unwrap_or_else(|| format!("obs-epoch-{:010}", epoch.epoch_idx));
            let epoch_id = epoch
                .manifest
                .as_ref()
                .map(|manifest| manifest.epoch_id.clone())
                .unwrap_or_else(|| bijux_gnss_core::api::obs_epoch_stability_key(epoch));
            epoch.sats.iter().filter_map(move |observation| {
                (observation.signal_id.sat == sat
                    && matches!(
                        observation.observation_status,
                        ObservationStatus::Accepted | ObservationStatus::Weak
                    ))
                .then_some(ObservedSatelliteRow {
                    artifact_id: artifact_id.clone(),
                    epoch_id: epoch_id.clone(),
                    epoch_index: epoch.epoch_idx,
                    sample_index: epoch.source_time.sample_index,
                    observation,
                })
            })
        })
        .collect()
}

fn synthetic_pseudorange_m(
    ephemeris: &GpsEphemeris,
    receive_time_s: f64,
    receiver_ecef_m: [f64; 3],
) -> f64 {
    let mut tau_s = 0.07;
    let mut pseudorange_m = 0.0;
    for _ in 0..10 {
        let sat = sat_state_gps_l1ca(ephemeris, receive_time_s - tau_s, tau_s);
        let dx = receiver_ecef_m[0] - sat.x_m;
        let dy = receiver_ecef_m[1] - sat.y_m;
        let dz = receiver_ecef_m[2] - sat.z_m;
        let geometric_range_m = (dx * dx + dy * dy + dz * dz).sqrt();
        pseudorange_m = geometric_range_m - sat.clock_correction.bias_s * SPEED_OF_LIGHT_MPS;
        let next_tau_s = pseudorange_m / SPEED_OF_LIGHT_MPS;
        if (next_tau_s - tau_s).abs() < 1.0e-12 {
            break;
        }
        tau_s = next_tau_s;
    }
    pseudorange_m
}

/// Per-satellite acquisition code-phase comparison between clean synthetic truth and receiver output.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionCodePhaseValidationSatellite {
    /// Satellite identifier.
    pub sat: SatId,
    /// Injected code phase at the start of the validation frame, in chips.
    pub injected_code_phase_chips: f64,
    /// Expected acquisition-reported code phase sample under the receiver search convention.
    pub expected_code_phase_samples: usize,
    /// Measured acquisition-reported code phase sample.
    pub measured_code_phase_samples: usize,
    /// Wrapped absolute error between expected and measured code phase samples.
    pub code_phase_error_samples: usize,
    /// Peak-to-mean ratio for the selected acquisition result.
    pub peak_mean_ratio: f32,
    /// Acquisition hypothesis returned by the receiver.
    pub hypothesis: String,
    /// Whether the measured code phase stayed within tolerance.
    pub pass: bool,
}

/// Truth-guided acquisition code-phase validation report for a synthetic capture.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionCodePhaseValidationReport {
    /// Stable scenario identifier for this capture.
    pub scenario_id: String,
    /// Allowed wrapped absolute error in samples.
    pub tolerance_samples: usize,
    /// Capture sample rate in Hz.
    pub sample_rate_hz: f64,
    /// Number of samples in one code period at the configured rate.
    pub period_samples: usize,
    /// Whether every measured satellite passed the requested tolerance.
    pub pass: bool,
    /// Per-satellite comparison rows.
    pub satellites: Vec<SyntheticAcquisitionCodePhaseValidationSatellite>,
}

/// Per-satellite comparison between coarse acquisition code phase and refined sub-sample phase.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionCodePhaseRefinementSatellite {
    /// Satellite identifier.
    pub sat: SatId,
    /// Injected code phase at the start of the validation frame, in chips.
    pub injected_code_phase_chips: f64,
    /// Continuous expected acquisition-reported code phase sample.
    pub expected_code_phase_samples: f64,
    /// Coarse acquisition-reported code phase sample.
    pub coarse_code_phase_samples: usize,
    /// Refined acquisition-reported code phase sample.
    pub refined_code_phase_samples: f64,
    /// Wrapped absolute coarse error in samples.
    pub coarse_error_samples: f64,
    /// Wrapped absolute refined error in samples.
    pub refined_error_samples: f64,
    /// Pseudorange initialization error from the coarse phase, in meters.
    pub coarse_pseudorange_error_m: f64,
    /// Pseudorange initialization error from the refined phase, in meters.
    pub refined_pseudorange_error_m: f64,
    /// Improvement from refinement, in samples.
    pub improvement_samples: f64,
    /// Improvement from refinement, in meters.
    pub improvement_m: f64,
    /// Peak-to-mean ratio for the selected acquisition result.
    pub peak_mean_ratio: f32,
    /// Acquisition hypothesis returned by the receiver.
    pub hypothesis: String,
    /// Whether the refined phase preserved or improved coarse pseudorange initialization.
    pub pass: bool,
}

/// Truth-guided report for acquisition code-phase refinement on a synthetic capture.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionCodePhaseRefinementReport {
    /// Stable scenario identifier for this capture.
    pub scenario_id: String,
    /// Capture sample rate in Hz.
    pub sample_rate_hz: f64,
    /// Number of samples in one code period at the configured rate.
    pub period_samples: usize,
    /// Whether every measured satellite passed the refinement criterion.
    pub pass: bool,
    /// Per-satellite refinement rows.
    pub satellites: Vec<SyntheticAcquisitionCodePhaseRefinementSatellite>,
}

/// Per-satellite acquisition Doppler comparison between clean synthetic truth and receiver output.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionDopplerValidationSatellite {
    /// Satellite identifier.
    pub sat: SatId,
    /// Injected Doppler shift in Hz.
    pub injected_doppler_hz: f64,
    /// Expected acquisition Doppler after the common receiver clock bias is applied, in Hz.
    pub expected_measured_doppler_hz: f64,
    /// Measured acquisition Doppler in Hz relative to the configured IF.
    pub measured_doppler_hz: f64,
    /// Absolute Doppler error in Hz.
    pub doppler_error_hz: f64,
    /// Effective acquisition Doppler bin width in Hz.
    pub doppler_step_hz: i32,
    /// Doppler error expressed in acquisition bins.
    pub doppler_error_bins: f64,
    /// Peak-to-mean ratio for the selected acquisition result.
    pub peak_mean_ratio: f32,
    /// Acquisition hypothesis returned by the receiver.
    pub hypothesis: String,
    /// Whether the measured Doppler stayed within tolerance.
    pub pass: bool,
}

/// Truth-guided acquisition Doppler validation report for a synthetic capture.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionDopplerValidationReport {
    /// Stable scenario identifier for this capture.
    pub scenario_id: String,
    /// Allowed Doppler error in acquisition bins.
    pub tolerance_bins: usize,
    /// Allowed Doppler error in Hz.
    pub tolerance_hz: f64,
    /// Capture sample rate in Hz.
    pub sample_rate_hz: f64,
    /// Effective acquisition Doppler bin width in Hz.
    pub doppler_step_hz: i32,
    /// Whether every measured satellite passed the requested tolerance.
    pub pass: bool,
    /// Per-satellite comparison rows.
    pub satellites: Vec<SyntheticAcquisitionDopplerValidationSatellite>,
}

/// Per-satellite acquisition truth-table row for synthetic validation.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionTruthTableSatellite {
    /// Satellite identifier.
    pub sat: SatId,
    /// Injected Doppler shift in Hz.
    pub injected_doppler_hz: f64,
    /// Expected acquisition Doppler after the common receiver clock bias is applied, in Hz.
    pub expected_measured_doppler_hz: f64,
    /// Measured acquisition Doppler in Hz relative to the configured IF.
    pub measured_doppler_hz: f64,
    /// Absolute Doppler error in Hz.
    pub doppler_error_hz: f64,
    /// Doppler error expressed in acquisition bins.
    pub doppler_error_bins: f64,
    /// Injected code phase at the start of the validation frame, in chips.
    pub injected_code_phase_chips: f64,
    /// Expected acquisition-reported code phase sample under the receiver search convention.
    pub expected_code_phase_samples: usize,
    /// Measured acquisition-reported code phase sample.
    pub measured_code_phase_samples: usize,
    /// Wrapped absolute error between expected and measured code phase samples.
    pub code_phase_error_samples: usize,
    /// Peak-to-mean ratio for the selected acquisition result.
    pub peak_mean_ratio: f32,
    /// Acquisition hypothesis returned by the receiver.
    pub hypothesis: String,
    /// Whether the measured Doppler stayed within tolerance.
    pub doppler_pass: bool,
    /// Whether the measured code phase stayed within tolerance.
    pub code_phase_pass: bool,
    /// Whether both Doppler and code phase stayed within tolerance.
    pub pass: bool,
}

/// Truth-guided acquisition truth table for a synthetic capture.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionTruthTableReport {
    /// Stable scenario identifier for this capture.
    pub scenario_id: String,
    /// Allowed Doppler error in acquisition bins.
    pub doppler_tolerance_bins: usize,
    /// Allowed Doppler error in Hz.
    pub doppler_tolerance_hz: f64,
    /// Allowed wrapped absolute code-phase error in samples.
    pub code_phase_tolerance_samples: usize,
    /// Capture sample rate in Hz.
    pub sample_rate_hz: f64,
    /// Number of samples in one code period at the configured rate.
    pub period_samples: usize,
    /// Effective acquisition Doppler bin width in Hz.
    pub doppler_step_hz: i32,
    /// Whether every measured satellite passed the requested tolerances.
    pub pass: bool,
    /// Per-satellite truth-table rows.
    pub satellites: Vec<SyntheticAcquisitionTruthTableSatellite>,
}

/// Per-satellite acquisition receiver clock-offset comparison row.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionReceiverClockOffsetSatellite {
    /// Satellite identifier.
    pub sat: SatId,
    /// Injected Doppler shift in Hz.
    pub injected_doppler_hz: f64,
    /// Receiver clock frequency bias injected into the synthetic capture, in Hz.
    pub injected_receiver_clock_frequency_bias_hz: f64,
    /// Expected Doppler measurement after the common receiver clock bias is applied, in Hz.
    pub expected_measured_doppler_hz: f64,
    /// Measured acquisition Doppler in Hz relative to the configured IF.
    pub measured_doppler_hz: f64,
    /// Receiver clock frequency bias implied by the measured acquisition Doppler, in Hz.
    pub measured_receiver_clock_frequency_bias_hz: f64,
    /// Absolute error between measured and injected receiver clock frequency bias, in Hz.
    pub receiver_clock_frequency_bias_error_hz: f64,
    /// Effective acquisition Doppler bin width in Hz.
    pub doppler_step_hz: i32,
    /// Peak-to-mean ratio for the selected acquisition result.
    pub peak_mean_ratio: f32,
    /// Acquisition hypothesis returned by the receiver.
    pub hypothesis: String,
    /// Whether the measured receiver clock bias stayed within tolerance.
    pub pass: bool,
}

/// Truth-guided acquisition receiver clock-offset validation report for a synthetic capture.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionReceiverClockOffsetValidationReport {
    /// Stable scenario identifier for this capture.
    pub scenario_id: String,
    /// Receiver clock frequency bias injected into the synthetic capture, in Hz.
    pub injected_receiver_clock_frequency_bias_hz: f64,
    /// Allowed receiver clock bias error in acquisition bins.
    pub tolerance_bins: usize,
    /// Allowed receiver clock bias error in Hz.
    pub tolerance_hz: f64,
    /// Capture sample rate in Hz.
    pub sample_rate_hz: f64,
    /// Effective acquisition Doppler bin width in Hz.
    pub doppler_step_hz: i32,
    /// Mean recovered receiver clock frequency bias across all measured satellites, in Hz.
    pub mean_measured_receiver_clock_frequency_bias_hz: f64,
    /// Minimum recovered receiver clock frequency bias across all measured satellites, in Hz.
    pub min_measured_receiver_clock_frequency_bias_hz: f64,
    /// Maximum recovered receiver clock frequency bias across all measured satellites, in Hz.
    pub max_measured_receiver_clock_frequency_bias_hz: f64,
    /// Spread between the minimum and maximum recovered receiver clock frequency bias, in Hz.
    pub measured_receiver_clock_frequency_bias_spread_hz: f64,
    /// Whether every measured satellite recovered the injected receiver clock bias consistently.
    pub pass: bool,
    /// Per-satellite comparison rows.
    pub satellites: Vec<SyntheticAcquisitionReceiverClockOffsetSatellite>,
}

/// Per-satellite acquisition validation row for a specific coherent integration profile.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionCoherentIntegrationSatellite {
    /// Satellite identifier.
    pub sat: SatId,
    /// Coherent integration length under test, in milliseconds.
    pub coherent_ms: u32,
    /// Noncoherent integration count under test.
    pub noncoherent: u32,
    /// Code-phase error in samples.
    pub code_phase_error_samples: usize,
    /// Doppler error in Hz.
    pub doppler_error_hz: f64,
    /// Doppler error in acquisition bins.
    pub doppler_error_bins: f64,
    /// Peak-to-mean ratio for the selected acquisition result.
    pub peak_mean_ratio: f32,
    /// Acquisition hypothesis returned by the receiver.
    pub hypothesis: String,
    /// Whether both code-phase and Doppler checks passed for this profile.
    pub pass: bool,
}

/// Truth-guided acquisition validation report for a coherent integration profile.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionCoherentIntegrationReport {
    /// Stable scenario identifier for this capture.
    pub scenario_id: String,
    /// Coherent integration length under test, in milliseconds.
    pub coherent_ms: u32,
    /// Noncoherent integration count under test.
    pub noncoherent: u32,
    /// Allowed code-phase error in samples.
    pub code_phase_tolerance_samples: usize,
    /// Allowed Doppler error in acquisition bins.
    pub doppler_tolerance_bins: usize,
    /// Allowed Doppler error in Hz.
    pub doppler_tolerance_hz: f64,
    /// Capture sample rate in Hz.
    pub sample_rate_hz: f64,
    /// Whether every measured satellite passed the requested tolerances.
    pub pass: bool,
    /// Per-satellite validation rows.
    pub satellites: Vec<SyntheticAcquisitionCoherentIntegrationSatellite>,
}

/// Per-trial acquisition outcome for a synthetic sensitivity profile.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionSensitivityTrial {
    /// Stable scenario identifier for this trial.
    pub scenario_id: String,
    /// Deterministic seed used for the synthetic noise realization.
    pub seed: u64,
    /// Satellite identifier under test.
    pub sat: SatId,
    /// Acquisition hypothesis returned by the receiver.
    pub hypothesis: String,
    /// Whether the receiver accepted this trial.
    pub accepted: bool,
    /// Whether the selected non-rejected result stayed within truth tolerances.
    pub detected: bool,
    /// Wrapped code-phase error in samples when truth was available.
    pub code_phase_error_samples: Option<usize>,
    /// Doppler error in acquisition bins when truth was available.
    pub doppler_error_bins: Option<f64>,
    /// Peak-to-mean ratio for the selected acquisition result.
    pub peak_mean_ratio: f32,
}

/// Detection-probability summary for a synthetic acquisition profile.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionSensitivityReport {
    /// Scenario identifier prefix shared across the trials.
    pub scenario_id_prefix: String,
    /// Satellite identifier under test.
    pub sat: SatId,
    /// Injected carrier-to-noise density ratio in dB-Hz.
    pub cn0_db_hz: f32,
    /// Coherent integration length under test, in milliseconds.
    pub coherent_ms: u32,
    /// Noncoherent integration count under test.
    pub noncoherent: u32,
    /// Allowed code-phase error in samples.
    pub code_phase_tolerance_samples: usize,
    /// Allowed Doppler error in acquisition bins.
    pub doppler_tolerance_bins: usize,
    /// Effective acquisition Doppler bin width in Hz.
    pub doppler_step_hz: i32,
    /// Number of synthetic trials measured for this profile.
    pub trial_count: usize,
    /// Number of trials that produced accepted results.
    pub accepted_count: usize,
    /// Number of trials that produced non-rejected results within truth tolerances.
    pub detected_count: usize,
    /// Accepted-trial probability across the measured trials.
    pub acceptance_probability: f64,
    /// Detection probability across the measured trials.
    pub detection_probability: f64,
    /// Mean peak-to-mean ratio across the measured trials.
    pub mean_peak_mean_ratio: f64,
    /// Per-trial acquisition rows.
    pub trials: Vec<SyntheticAcquisitionSensitivityTrial>,
}

/// Acquisition detection-rate input for one synthetic measurement point.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionDetectionRateCase {
    /// Signal configuration under test.
    pub signal: SyntheticSignalParams,
    /// Coherent integration length under test, in milliseconds.
    pub coherent_ms: u32,
    /// Noncoherent integration count under test.
    pub noncoherent: u32,
}

/// Detection-rate summary for one synthetic acquisition measurement point.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionDetectionRatePoint {
    /// Satellite identifier under test.
    pub sat: SatId,
    /// Injected carrier-to-noise density ratio in dB-Hz.
    pub cn0_db_hz: f32,
    /// Injected Doppler shift in Hz.
    pub doppler_hz: f64,
    /// Coherent integration length under test, in milliseconds.
    pub coherent_ms: u32,
    /// Noncoherent integration count under test.
    pub noncoherent: u32,
    /// Number of synthetic trials measured for this point.
    pub trial_count: usize,
    /// Number of trials that produced accepted results.
    pub accepted_count: usize,
    /// Number of trials that produced non-rejected results within truth tolerances.
    pub detected_count: usize,
    /// Accepted-trial probability across the measured trials.
    pub acceptance_probability: f64,
    /// Detection probability across the measured trials.
    pub detection_probability: f64,
    /// Mean peak-to-mean ratio across the measured trials.
    pub mean_peak_mean_ratio: f64,
}

/// Detection-rate report across multiple C/N0, Doppler, and integration settings.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionDetectionRateReport {
    /// Scenario identifier prefix shared across the measurement points.
    pub scenario_id_prefix: String,
    /// Allowed code-phase error in samples.
    pub code_phase_tolerance_samples: usize,
    /// Allowed Doppler error in acquisition bins.
    pub doppler_tolerance_bins: usize,
    /// Effective acquisition Doppler bin width in Hz.
    pub doppler_step_hz: i32,
    /// Measurement points captured in the report.
    pub points: Vec<SyntheticAcquisitionDetectionRatePoint>,
}

/// Per-trial tracking outcome for a synthetic lock-sensitivity profile.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticTrackingSensitivityTrial {
    /// Stable scenario identifier for this trial.
    pub scenario_id: String,
    /// Deterministic seed used for the synthetic noise realization.
    pub seed: u64,
    /// Satellite identifier under test.
    pub sat: SatId,
    /// Whether the receiver reached a sustained stable tracking window.
    pub stable_lock: bool,
    /// Whether the receiver explicitly refused to declare lock under weak signal conditions.
    pub refused_lock: bool,
    /// First epoch index of the sustained stable tracking window, when one exists.
    pub first_lock_epoch_index: Option<usize>,
    /// Count of epochs in the sustained stable tracking window.
    pub locked_epoch_count: usize,
    /// Final per-epoch tracking state emitted by the receiver.
    pub final_lock_state: String,
    /// Final per-epoch tracking state reason emitted by the receiver.
    pub final_lock_state_reason: Option<String>,
}

/// Lock-probability summary for one synthetic tracking profile.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticTrackingSensitivityReport {
    /// Scenario identifier prefix shared across the trials.
    pub scenario_id_prefix: String,
    /// Satellite identifier under test.
    pub sat: SatId,
    /// Injected carrier-to-noise density ratio in dB-Hz.
    pub cn0_db_hz: f32,
    /// Duration of each synthetic tracking trial in seconds.
    pub duration_s: f64,
    /// Seeded Doppler error relative to truth, in Hz.
    pub seeded_doppler_error_hz: f64,
    /// Seeded code-phase error relative to truth, in samples.
    pub seeded_code_phase_error_samples: isize,
    /// Minimum sustained locked-epoch count required to declare stable lock.
    pub min_locked_epochs: usize,
    /// Number of synthetic trials measured for this profile.
    pub trial_count: usize,
    /// Number of trials that reached a sustained stable lock window.
    pub stable_lock_count: usize,
    /// Number of trials that explicitly refused lock under weak-signal conditions.
    pub refused_lock_count: usize,
    /// Stable-lock probability across the measured trials.
    pub lock_probability: f64,
    /// Mean count of epochs inside the sustained lock window.
    pub mean_locked_epochs: f64,
    /// Per-trial tracking rows.
    pub trials: Vec<SyntheticTrackingSensitivityTrial>,
}

/// Tracking lock-rate input for one synthetic measurement point.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticTrackingLockRateCase {
    /// Signal configuration under test.
    pub signal: SyntheticSignalParams,
    /// Duration of each synthetic tracking trial in seconds.
    pub duration_s: f64,
    /// Seeded Doppler error relative to truth, in Hz.
    pub seeded_doppler_error_hz: f64,
    /// Seeded code-phase error relative to truth, in samples.
    pub seeded_code_phase_error_samples: isize,
    /// Minimum sustained locked-epoch count required to declare stable lock.
    pub min_locked_epochs: usize,
}

/// Lock-rate summary for one synthetic tracking measurement point.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticTrackingLockRatePoint {
    /// Satellite identifier under test.
    pub sat: SatId,
    /// Injected carrier-to-noise density ratio in dB-Hz.
    pub cn0_db_hz: f32,
    /// Duration of each synthetic tracking trial in seconds.
    pub duration_s: f64,
    /// Seeded Doppler error relative to truth, in Hz.
    pub seeded_doppler_error_hz: f64,
    /// Seeded code-phase error relative to truth, in samples.
    pub seeded_code_phase_error_samples: isize,
    /// Minimum sustained locked-epoch count required to declare stable lock.
    pub min_locked_epochs: usize,
    /// Number of synthetic trials measured for this point.
    pub trial_count: usize,
    /// Number of trials that reached a sustained stable lock window.
    pub stable_lock_count: usize,
    /// Number of trials that explicitly refused lock under weak-signal conditions.
    pub refused_lock_count: usize,
    /// Stable-lock probability across the measured trials.
    pub lock_probability: f64,
    /// Mean count of epochs inside the sustained lock window.
    pub mean_locked_epochs: f64,
}

/// Lock-rate report across multiple C/N0 tracking measurement points.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticTrackingLockRateReport {
    /// Scenario identifier prefix shared across the measurement points.
    pub scenario_id_prefix: String,
    /// Measurement points captured in the report.
    pub points: Vec<SyntheticTrackingLockRatePoint>,
}

/// Per-epoch tracking truth-table row for synthetic validation.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticTrackingTruthTableEpoch {
    /// Zero-based epoch index inside the satellite track.
    pub epoch_index: usize,
    /// Absolute sample index at the start of the epoch.
    pub sample_index: u64,
    /// Expected carrier frequency in Hz at this epoch.
    pub expected_carrier_hz: f64,
    /// Measured carrier frequency in Hz at this epoch.
    pub measured_carrier_hz: f64,
    /// Absolute carrier-frequency error in Hz.
    pub carrier_error_hz: f64,
    /// Expected Doppler in Hz at this epoch.
    pub expected_doppler_hz: f64,
    /// Measured Doppler in Hz at this epoch.
    pub measured_doppler_hz: f64,
    /// Absolute Doppler error in Hz.
    pub doppler_error_hz: f64,
    /// Expected code phase in samples at this epoch.
    pub expected_code_phase_samples: f64,
    /// Measured code phase in samples at this epoch.
    pub measured_code_phase_samples: f64,
    /// Wrapped absolute code-phase error in samples.
    pub code_phase_error_samples: f64,
    /// Expected C/N0 in dB-Hz at this epoch.
    pub expected_cn0_db_hz: f64,
    /// Measured C/N0 in dB-Hz at this epoch.
    pub measured_cn0_dbhz: f64,
    /// Absolute C/N0 error in dB-Hz.
    pub cn0_error_db: f64,
    /// Whether the prompt detector reported signal lock.
    pub lock: bool,
    /// Whether the PLL reported lock.
    pub pll_lock: bool,
    /// Whether the DLL reported lock.
    pub dll_lock: bool,
    /// Whether the FLL reported lock.
    pub fll_lock: bool,
    /// Whether the receiver reported a cycle slip at this epoch.
    pub cycle_slip: bool,
    /// Receiver lock-state label for this epoch.
    pub lock_state: String,
    /// Receiver lock-state reason for this epoch, when one was emitted.
    pub lock_state_reason: Option<String>,
    /// Whether this epoch belongs to a stable tracking window.
    pub stable_tracking_epoch: bool,
    /// Whether every tracked error stayed within tolerance for this stable epoch.
    pub pass: bool,
}

/// Per-satellite tracking truth table for synthetic validation.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticTrackingTruthTableSatellite {
    /// Satellite identifier.
    pub sat: SatId,
    /// Injected Doppler shift in Hz.
    pub injected_doppler_hz: f64,
    /// Expected measured Doppler after the receiver clock bias is applied, in Hz.
    pub expected_measured_doppler_hz: f64,
    /// Injected code phase at sample zero, in chips.
    pub injected_code_phase_chips: f64,
    /// Injected carrier-to-noise density ratio in dB-Hz.
    pub injected_cn0_db_hz: f32,
    /// Count of recorded epochs for this satellite.
    pub epoch_count: usize,
    /// Count of epochs inside stable tracking windows.
    pub stable_epoch_count: usize,
    /// First stable tracking epoch index, when one exists.
    pub first_stable_epoch_index: Option<usize>,
    /// Whether every stable tracking epoch stayed within tolerance.
    pub pass: bool,
    /// Per-epoch truth-table rows.
    pub epochs: Vec<SyntheticTrackingTruthTableEpoch>,
}

/// Truth-guided tracking truth table for a synthetic capture.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticTrackingTruthTableReport {
    /// Stable scenario identifier for this capture.
    pub scenario_id: String,
    /// Allowed carrier-frequency error in Hz.
    pub carrier_tolerance_hz: f64,
    /// Allowed Doppler error in Hz.
    pub doppler_tolerance_hz: f64,
    /// Allowed wrapped absolute code-phase error in samples.
    pub code_phase_tolerance_samples: f64,
    /// Allowed absolute C/N0 error in dB-Hz.
    pub cn0_tolerance_db_hz: f64,
    /// Capture sample rate in Hz.
    pub sample_rate_hz: f64,
    /// Number of samples in one code period at the configured rate.
    pub period_samples: usize,
    /// Output scale applied before quantization.
    pub output_scale_applied: f32,
    /// Whether every measured satellite passed the requested tolerances.
    pub pass: bool,
    /// Per-satellite truth-table rows.
    pub satellites: Vec<SyntheticTrackingTruthTableSatellite>,
}

/// Noise-only false-alarm summary for a synthetic acquisition profile.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionFalseAlarmReport {
    /// Scenario identifier prefix shared across the trials.
    pub scenario_id_prefix: String,
    /// Satellite identifier searched during the noise-only trials.
    pub sat: SatId,
    /// Coherent integration length under test, in milliseconds.
    pub coherent_ms: u32,
    /// Noncoherent integration count under test.
    pub noncoherent: u32,
    /// Number of synthetic noise-only trials measured for this profile.
    pub trial_count: usize,
    /// Number of trials that produced accepted results.
    pub false_alarm_count: usize,
    /// False-alarm probability across the measured trials.
    pub false_alarm_rate: f64,
    /// Mean peak-to-mean ratio across the measured trials.
    pub mean_peak_mean_ratio: f64,
    /// Per-trial acquisition rows.
    pub trials: Vec<SyntheticAcquisitionSensitivityTrial>,
}

/// Noise-only false-alarm measurement input for one acquisition profile.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
pub struct SyntheticAcquisitionFalseAlarmRateCase {
    /// Satellite identifier searched during the noise-only trials.
    pub sat: SatId,
    /// Coherent integration length under test, in milliseconds.
    pub coherent_ms: u32,
    /// Noncoherent integration count under test.
    pub noncoherent: u32,
}

/// Noise-only false-alarm summary for one acquisition measurement point.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionFalseAlarmRatePoint {
    /// Satellite identifier searched during the noise-only trials.
    pub sat: SatId,
    /// Coherent integration length under test, in milliseconds.
    pub coherent_ms: u32,
    /// Noncoherent integration count under test.
    pub noncoherent: u32,
    /// Number of synthetic noise-only trials measured for this point.
    pub trial_count: usize,
    /// Number of trials that produced accepted results.
    pub false_alarm_count: usize,
    /// False-alarm probability across the measured trials.
    pub false_alarm_rate: f64,
    /// Mean peak-to-mean ratio across the measured trials.
    pub mean_peak_mean_ratio: f64,
}

/// Noise-only false-alarm report across multiple acquisition profiles.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionFalseAlarmRateReport {
    /// Scenario identifier prefix shared across the measurement points.
    pub scenario_id_prefix: String,
    /// Configured acquisition Doppler search half-width in Hz.
    pub acquisition_doppler_search_hz: i32,
    /// Effective acquisition Doppler bin width in Hz.
    pub acquisition_doppler_step_hz: i32,
    /// Measurement points captured in the report.
    pub points: Vec<SyntheticAcquisitionFalseAlarmRatePoint>,
}

/// Per-satellite comparison between coarse acquisition Doppler bins and refined Doppler estimates.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionDopplerRefinementSatellite {
    /// Satellite identifier.
    pub sat: SatId,
    /// Injected Doppler shift in Hz.
    pub injected_doppler_hz: f64,
    /// Coarse acquisition Doppler in Hz relative to the configured IF.
    pub coarse_doppler_hz: f64,
    /// Refined acquisition Doppler in Hz relative to the configured IF.
    pub refined_doppler_hz: f64,
    /// Absolute coarse Doppler error in Hz.
    pub coarse_error_hz: f64,
    /// Absolute refined Doppler error in Hz.
    pub refined_error_hz: f64,
    /// Improvement from refinement, in Hz.
    pub improvement_hz: f64,
    /// Effective acquisition Doppler bin width in Hz.
    pub doppler_step_hz: i32,
    /// Refined Doppler offset relative to the coarse grid, in bins.
    pub refinement_bins: f64,
    /// Peak-to-mean ratio for the selected acquisition result.
    pub peak_mean_ratio: f32,
    /// Acquisition hypothesis returned by the receiver.
    pub hypothesis: String,
    /// Whether the refined Doppler stayed measurably closer than the coarse bin.
    pub pass: bool,
}

/// Truth-guided report for acquisition Doppler refinement on a synthetic capture.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionDopplerRefinementReport {
    /// Stable scenario identifier for this capture.
    pub scenario_id: String,
    /// Capture sample rate in Hz.
    pub sample_rate_hz: f64,
    /// Effective acquisition Doppler bin width in Hz.
    pub doppler_step_hz: i32,
    /// Whether every measured satellite passed the refinement criterion.
    pub pass: bool,
    /// Per-satellite refinement rows.
    pub satellites: Vec<SyntheticAcquisitionDopplerRefinementSatellite>,
}

/// Validation input for one acquisition sample-rate profile.
#[derive(Debug, Clone, Copy)]
pub struct SyntheticAcquisitionSampleRateValidationCase<'a> {
    /// Receiver configuration to validate.
    pub config: &'a ReceiverPipelineConfig,
    /// Synthetic sample frame captured at the configured rate.
    pub frame: &'a SamplesFrame,
    /// Machine-readable synthetic truth for the frame.
    pub truth: &'a SyntheticIqTruthBundle,
}

/// Per-sample-rate acquisition validation summary across code-phase and Doppler checks.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionSampleRateValidationPoint {
    /// Stable scenario identifier for this sample-rate profile.
    pub scenario_id: String,
    /// Capture sample rate in Hz.
    pub sample_rate_hz: f64,
    /// Truth-guided acquisition code-phase validation for this profile.
    pub code_phase_validation: SyntheticAcquisitionCodePhaseValidationReport,
    /// Truth-guided acquisition Doppler validation for this profile.
    pub doppler_validation: SyntheticAcquisitionDopplerValidationReport,
    /// Whether both code-phase and Doppler checks passed for this profile.
    pub pass: bool,
}

/// Aggregate acquisition validation across multiple sample-rate profiles.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionSampleRateValidationReport {
    /// Allowed code-phase error in samples for every profile.
    pub code_phase_tolerance_samples: usize,
    /// Allowed Doppler error in acquisition bins for every profile.
    pub doppler_tolerance_bins: usize,
    /// Allowed Doppler error in Hz for every profile.
    pub doppler_tolerance_hz: f64,
    /// Number of distinct sample rates covered by the report.
    pub distinct_sample_rate_count: usize,
    /// Lowest sample rate validated by this report, in Hz.
    pub min_sample_rate_hz: f64,
    /// Highest sample rate validated by this report, in Hz.
    pub max_sample_rate_hz: f64,
    /// Whether at least two distinct sample rates were validated and every profile passed.
    pub pass: bool,
    /// Per-profile validation results.
    pub points: Vec<SyntheticAcquisitionSampleRateValidationPoint>,
}

/// Build a machine-readable truth bundle for an emitted synthetic capture.
pub fn build_truth_bundle(
    scenario_id: &str,
    scenario: &SyntheticScenario,
    frame: &SamplesFrame,
    metadata: &RawIqMetadata,
    peak_component_before_scaling: f32,
    output_scale_applied: f32,
) -> SyntheticIqTruthBundle {
    SyntheticIqTruthBundle {
        schema_version: SYNTHETIC_IQ_TRUTH_SCHEMA_VERSION,
        scenario_id: scenario_id.to_string(),
        seed: scenario.seed,
        sample_format: metadata.format,
        sample_rate_hz: frame.t0.sample_rate_hz,
        intermediate_freq_hz: metadata.intermediate_freq_hz,
        receiver_clock_frequency_bias_hz: scenario.receiver_clock_frequency_bias_hz,
        capture_start_utc: metadata.capture_start_utc.clone(),
        quantization_bits: metadata.quantization_bits.unwrap_or_default(),
        duration_s: frame.len() as f64 * frame.dt_s.0,
        sample_count: frame.len(),
        noise_std_per_component: SYNTHETIC_NOISE_STD_PER_COMPONENT,
        noise_power_per_complex_sample: SYNTHETIC_COMPLEX_NOISE_POWER as f32,
        peak_component_before_scaling,
        output_scale_applied,
        satellites: scenario
            .satellites
            .iter()
            .map(|params| SyntheticSatelliteTruth {
                sat: params.sat,
                doppler_hz: params.doppler_hz,
                code_phase_chips: params.code_phase_chips,
                carrier_phase_rad: params.carrier_phase_rad,
                cn0_db_hz: params.cn0_db_hz,
                signal_amplitude: signal_amplitude_from_cn0(
                    params.cn0_db_hz,
                    frame.t0.sample_rate_hz,
                ),
                nav_bit_mode: nav_bit_mode(params),
                nav_bit_segments: nav_bit_segments(
                    frame.t0.sample_rate_hz,
                    frame.len() as u64,
                    params.data_bit_flip,
                ),
            })
            .collect(),
    }
}

/// Encode a generated synthetic frame as an IQ16 little-endian capture with truth metadata.
pub fn build_iq16_capture_bundle(
    scenario_id: &str,
    scenario: &SyntheticScenario,
    frame: &SamplesFrame,
    capture_start_utc: &str,
    notes: Option<String>,
) -> SyntheticIqCaptureBundle {
    let peak_component_before_scaling = peak_component(&frame.iq);
    let output_scale_applied = if peak_component_before_scaling <= 0.999 {
        1.0
    } else {
        0.999 / peak_component_before_scaling
    };
    let raw_iq_bytes = encode_iq16_le_bytes(&frame.iq, output_scale_applied);
    let metadata = RawIqMetadata {
        format: IqSampleFormat::Iq16Le,
        sample_rate_hz: frame.t0.sample_rate_hz,
        intermediate_freq_hz: scenario.intermediate_freq_hz,
        capture_start_utc: capture_start_utc.to_string(),
        offset_bytes: 0,
        quantization_bits: Some(16),
        notes,
    };
    let truth = build_truth_bundle(
        scenario_id,
        scenario,
        frame,
        &metadata,
        peak_component_before_scaling,
        output_scale_applied,
    );
    SyntheticIqCaptureBundle { raw_iq_bytes, metadata, truth }
}

/// Measure truth-guided C/N0 directly from a synthetic capture using receiver correlators.
pub fn validate_truth_guided_cn0(
    config: &ReceiverPipelineConfig,
    frame: &SamplesFrame,
    truth: &SyntheticIqTruthBundle,
    tolerance_db_hz: f64,
) -> SyntheticCn0ValidationReport {
    let coherent_samples_per_epoch =
        samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length)
            .min(frame.len());
    let coherent_integration_s = coherent_samples_per_epoch as f64 * frame.dt_s.0;
    let period_samples =
        samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length);
    let satellites = truth
        .satellites
        .iter()
        .map(|sat_truth| {
            let isolated_frame = regenerate_isolated_scaled_satellite_frame_with_noise(
                config, frame, truth, sat_truth,
            );
            let tracking = crate::pipeline::tracking::Tracking::new(
                config.clone(),
                crate::engine::runtime::ReceiverRuntime::default(),
            );
            let seeded_code_phase_samples = wrap_seeded_code_phase_samples(
                expected_acquisition_code_phase_samples(
                    config,
                    &isolated_frame,
                    sat_truth.code_phase_chips,
                ) as isize,
                period_samples,
            );
            let tracks = tracking.track_from_acquisition(
                &isolated_frame,
                &[seeded_tracking_acquisition(
                    sat_truth.sat,
                    synthetic_truth_measured_doppler_hz(truth, sat_truth),
                    seeded_code_phase_samples,
                    sat_truth.cn0_db_hz,
                    format!("truth_guided_cn0_tracking_seed_{}", sat_truth.sat.prn),
                )],
            );
            let cn0_values = tracks
                .first()
                .map(|track| stable_tracking_cn0_values(&track.epochs))
                .unwrap_or_default();

            let measured_mean_cn0_dbhz = if cn0_values.is_empty() {
                0.0
            } else {
                cn0_values.iter().sum::<f64>() / cn0_values.len() as f64
            };
            let measured_min_cn0_dbhz = cn0_values.iter().copied().reduce(f64::min).unwrap_or(0.0);
            let measured_max_cn0_dbhz = cn0_values.iter().copied().reduce(f64::max).unwrap_or(0.0);
            let cn0_delta_db = measured_mean_cn0_dbhz - sat_truth.cn0_db_hz as f64;
            let pass = !cn0_values.is_empty() && cn0_delta_db.abs() <= tolerance_db_hz;

            SyntheticCn0ValidationSatellite {
                sat: sat_truth.sat,
                injected_cn0_db_hz: sat_truth.cn0_db_hz,
                measured_mean_cn0_dbhz,
                measured_min_cn0_dbhz,
                measured_max_cn0_dbhz,
                cn0_delta_db,
                signal_amplitude: sat_truth.signal_amplitude,
                output_signal_amplitude: sat_truth.signal_amplitude * truth.output_scale_applied,
                epochs_measured: cn0_values.len(),
                pass,
            }
        })
        .collect::<Vec<_>>();
    let pass = !satellites.is_empty() && satellites.iter().all(|row| row.pass);

    SyntheticCn0ValidationReport {
        scenario_id: truth.scenario_id.clone(),
        tolerance_db_hz,
        sample_rate_hz: truth.sample_rate_hz,
        coherent_samples_per_epoch,
        coherent_integration_s,
        quantization_bits: truth.quantization_bits,
        noise_std_per_component: truth.noise_std_per_component,
        noise_power_per_complex_sample: truth.noise_power_per_complex_sample,
        output_scale_applied: truth.output_scale_applied,
        pass,
        satellites,
    }
}

const TRACKING_CN0_MIN_STABLE_EPOCHS: usize = 5;

fn stable_tracking_cn0_values(epochs: &[crate::api::core::TrackEpoch]) -> Vec<f64> {
    let Some((start, count)) =
        stable_tracking_window_bounds(epochs, TRACKING_CN0_MIN_STABLE_EPOCHS)
    else {
        return Vec::new();
    };
    epochs[start..start + count]
        .iter()
        .filter_map(|epoch| epoch.cn0_dbhz.is_finite().then_some(epoch.cn0_dbhz))
        .collect()
}

/// Convert a truth-model code phase into the receiver's acquisition-reported sample convention.
pub fn expected_acquisition_code_phase_samples(
    config: &ReceiverPipelineConfig,
    frame: &SamplesFrame,
    code_phase_chips: f64,
) -> usize {
    let period_samples =
        samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length)
            .max(1);
    let phase_samples = code_phase_samples_at_epoch_start(config, frame, code_phase_chips);
    let injected_sample = (phase_samples.round() as usize) % period_samples;
    (period_samples - injected_sample) % period_samples
}

/// Convert a truth-model code phase into the receiver's continuous acquisition sample convention.
pub fn expected_acquisition_code_phase_samples_f64(
    config: &ReceiverPipelineConfig,
    frame: &SamplesFrame,
    code_phase_chips: f64,
) -> f64 {
    let period_samples =
        samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length)
            .max(1) as f64;
    let phase_samples = code_phase_samples_at_epoch_start(config, frame, code_phase_chips);
    (period_samples - phase_samples.rem_euclid(period_samples)).rem_euclid(period_samples)
}

/// Measure wrapped code-phase error in samples over one code period.
pub fn wrapped_code_phase_error_samples(
    actual: usize,
    expected: usize,
    period_samples: usize,
) -> usize {
    let period_samples = period_samples.max(1);
    let forward = actual.abs_diff(expected);
    let wrapped = period_samples.saturating_sub(forward);
    forward.min(wrapped)
}

/// Measure wrapped code-phase error in samples over one code period for fractional estimates.
pub fn wrapped_code_phase_error_samples_f64(
    actual: f64,
    expected: f64,
    period_samples: usize,
) -> f64 {
    let period_samples = period_samples.max(1) as f64;
    let forward = (actual - expected).abs().rem_euclid(period_samples);
    let wrapped = (period_samples - forward).rem_euclid(period_samples);
    forward.min(wrapped)
}

fn code_phase_error_samples_to_pseudorange_m(error_samples: f64, sample_rate_hz: f64) -> f64 {
    (error_samples / sample_rate_hz) * SPEED_OF_LIGHT_MPS
}

fn tracking_epoch_is_stable(epoch: &crate::api::core::TrackEpoch) -> bool {
    epoch.lock
        && epoch.lock_state == "tracking"
        && epoch.pll_lock
        && epoch.fll_lock
        && !epoch.cycle_slip
        && epoch.lock_state_reason.as_deref() != Some("lock_lost")
}

fn code_phase_samples_at_sample_index(
    config: &ReceiverPipelineConfig,
    sample_rate_hz: f64,
    sample_index: u64,
    code_phase_chips: f64,
) -> f64 {
    let start_s = sample_index as f64 / sample_rate_hz;
    let chip_phase = advance_code_phase_seconds(
        code_phase_chips,
        config.code_freq_basis_hz,
        start_s,
        config.code_length,
    )
    .expect("synthetic epoch alignment requires a valid code phase model");
    let samples_per_chip = sample_rate_hz / config.code_freq_basis_hz;
    chip_phase * samples_per_chip
}

fn expected_tracking_code_phase_samples(
    config: &ReceiverPipelineConfig,
    sample_rate_hz: f64,
    sample_index: u64,
    code_phase_chips: f64,
) -> f64 {
    let period_samples =
        samples_per_code(sample_rate_hz, config.code_freq_basis_hz, config.code_length).max(1)
            as f64;
    let phase_samples =
        code_phase_samples_at_sample_index(config, sample_rate_hz, sample_index, code_phase_chips);
    (period_samples - phase_samples.rem_euclid(period_samples)).rem_euclid(period_samples)
}

/// Build a truth-guided tracking table from a synthetic capture.
pub fn validate_truth_guided_tracking_table(
    config: &ReceiverPipelineConfig,
    frame: &SamplesFrame,
    truth: &SyntheticIqTruthBundle,
    carrier_tolerance_hz: f64,
    doppler_tolerance_hz: f64,
    code_phase_tolerance_samples: f64,
    cn0_tolerance_db_hz: f64,
) -> SyntheticTrackingTruthTableReport {
    let period_samples =
        samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length)
            .max(1);
    let satellites = truth
        .satellites
        .iter()
        .map(|sat_truth| {
            let isolated_frame = regenerate_isolated_scaled_satellite_frame_with_noise(
                config, frame, truth, sat_truth,
            );
            let expected_measured_doppler_hz =
                synthetic_truth_measured_doppler_hz(truth, sat_truth);
            let expected_carrier_hz = synthetic_carrier_hz(
                truth.intermediate_freq_hz,
                sat_truth.sat,
                expected_measured_doppler_hz,
            );
            let seeded_code_phase_samples = wrap_seeded_code_phase_samples(
                expected_acquisition_code_phase_samples(
                    config,
                    &isolated_frame,
                    sat_truth.code_phase_chips,
                ) as isize,
                period_samples,
            );
            let tracking = crate::pipeline::tracking::Tracking::new(
                config.clone(),
                crate::engine::runtime::ReceiverRuntime::default(),
            );
            let tracks = tracking.track_from_acquisition(
                &isolated_frame,
                &[seeded_tracking_acquisition(
                    sat_truth.sat,
                    expected_measured_doppler_hz,
                    seeded_code_phase_samples,
                    sat_truth.cn0_db_hz,
                    format!("truth_guided_tracking_seed_{}", sat_truth.sat.prn),
                )],
            );
            let epochs = tracks.first().map(|track| track.epochs.clone()).unwrap_or_default();
            let epoch_rows = epochs
                .iter()
                .enumerate()
                .map(|(epoch_index, epoch)| {
                    let expected_code_phase_samples = expected_tracking_code_phase_samples(
                        config,
                        truth.sample_rate_hz,
                        epoch.sample_index,
                        sat_truth.code_phase_chips,
                    );
                    let measured_code_phase_samples = epoch.code_phase_samples.0;
                    let code_phase_error_samples = wrapped_code_phase_error_samples_f64(
                        measured_code_phase_samples,
                        expected_code_phase_samples,
                        period_samples,
                    );
                    let measured_carrier_hz = epoch.carrier_hz.0;
                    let carrier_error_hz = (measured_carrier_hz - expected_carrier_hz).abs();
                    let measured_doppler_hz = crate::pipeline::doppler::doppler_hz_from_carrier_hz(
                        config.intermediate_freq_hz,
                        measured_carrier_hz,
                    );
                    let doppler_error_hz =
                        (measured_doppler_hz - expected_measured_doppler_hz).abs();
                    let measured_cn0_dbhz = epoch.cn0_dbhz;
                    let cn0_error_db = (measured_cn0_dbhz - sat_truth.cn0_db_hz as f64).abs();
                    let stable_tracking_epoch = tracking_epoch_is_stable(epoch);
                    let pass = stable_tracking_epoch
                        && measured_carrier_hz.is_finite()
                        && measured_doppler_hz.is_finite()
                        && measured_code_phase_samples.is_finite()
                        && measured_cn0_dbhz.is_finite()
                        && carrier_error_hz <= carrier_tolerance_hz + f64::EPSILON
                        && doppler_error_hz <= doppler_tolerance_hz + f64::EPSILON
                        && code_phase_error_samples <= code_phase_tolerance_samples + f64::EPSILON
                        && cn0_error_db <= cn0_tolerance_db_hz + f64::EPSILON;

                    SyntheticTrackingTruthTableEpoch {
                        epoch_index,
                        sample_index: epoch.sample_index,
                        expected_carrier_hz,
                        measured_carrier_hz,
                        carrier_error_hz,
                        expected_doppler_hz: expected_measured_doppler_hz,
                        measured_doppler_hz,
                        doppler_error_hz,
                        expected_code_phase_samples,
                        measured_code_phase_samples,
                        code_phase_error_samples,
                        expected_cn0_db_hz: sat_truth.cn0_db_hz as f64,
                        measured_cn0_dbhz,
                        cn0_error_db,
                        lock: epoch.lock,
                        pll_lock: epoch.pll_lock,
                        dll_lock: epoch.dll_lock,
                        fll_lock: epoch.fll_lock,
                        cycle_slip: epoch.cycle_slip,
                        lock_state: epoch.lock_state.clone(),
                        lock_state_reason: epoch.lock_state_reason.clone(),
                        stable_tracking_epoch,
                        pass,
                    }
                })
                .collect::<Vec<_>>();
            let stable_epoch_count =
                epoch_rows.iter().filter(|row| row.stable_tracking_epoch).count();
            let first_stable_epoch_index =
                epoch_rows.iter().find(|row| row.stable_tracking_epoch).map(|row| row.epoch_index);
            let pass = stable_epoch_count > 0
                && epoch_rows.iter().filter(|row| row.stable_tracking_epoch).all(|row| row.pass);

            SyntheticTrackingTruthTableSatellite {
                sat: sat_truth.sat,
                injected_doppler_hz: sat_truth.doppler_hz,
                expected_measured_doppler_hz,
                injected_code_phase_chips: sat_truth.code_phase_chips,
                injected_cn0_db_hz: sat_truth.cn0_db_hz,
                epoch_count: epoch_rows.len(),
                stable_epoch_count,
                first_stable_epoch_index,
                pass,
                epochs: epoch_rows,
            }
        })
        .collect::<Vec<_>>();
    let pass = !satellites.is_empty() && satellites.iter().all(|row| row.pass);

    SyntheticTrackingTruthTableReport {
        scenario_id: truth.scenario_id.clone(),
        carrier_tolerance_hz,
        doppler_tolerance_hz,
        code_phase_tolerance_samples,
        cn0_tolerance_db_hz,
        sample_rate_hz: truth.sample_rate_hz,
        period_samples,
        output_scale_applied: truth.output_scale_applied,
        pass,
        satellites,
    }
}

/// Build a truth-guided acquisition table from a synthetic capture.
pub fn validate_truth_guided_acquisition_table(
    config: &ReceiverPipelineConfig,
    frame: &SamplesFrame,
    truth: &SyntheticIqTruthBundle,
    doppler_tolerance_bins: usize,
    code_phase_tolerance_samples: usize,
) -> SyntheticAcquisitionTruthTableReport {
    let period_samples =
        samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length)
            .max(1);
    let doppler_step_hz = config.acquisition_doppler_step_hz.max(1);
    let doppler_tolerance_hz = doppler_tolerance_bins as f64 * doppler_step_hz as f64;
    let satellites = truth
        .satellites
        .iter()
        .map(|sat_truth| {
            let isolated_frame = regenerate_isolated_scaled_satellite_signal_only_frame(
                config, frame, truth, sat_truth,
            );
            let acquisition = crate::pipeline::acquisition::Acquisition::new(
                config.clone(),
                crate::engine::runtime::ReceiverRuntime::default(),
            );
            let result = acquisition.run_fft(&isolated_frame, &[sat_truth.sat]).remove(0);
            let expected_measured_doppler_hz =
                synthetic_truth_measured_doppler_hz(truth, sat_truth);
            let measured_doppler_hz = crate::pipeline::doppler::doppler_hz_from_carrier_hz(
                config.intermediate_freq_hz,
                result.carrier_hz.0,
            );
            let doppler_error_hz = (measured_doppler_hz - expected_measured_doppler_hz).abs();
            let doppler_error_bins = doppler_error_hz / doppler_step_hz as f64;
            let doppler_pass = measured_doppler_hz.is_finite()
                && doppler_error_hz <= doppler_tolerance_hz + f64::EPSILON;

            let expected_code_phase_samples = expected_acquisition_code_phase_samples(
                config,
                &isolated_frame,
                sat_truth.code_phase_chips,
            );
            let measured_code_phase_samples = result.code_phase_samples;
            let code_phase_error_samples = wrapped_code_phase_error_samples(
                measured_code_phase_samples,
                expected_code_phase_samples,
                period_samples,
            );
            let code_phase_pass = code_phase_error_samples <= code_phase_tolerance_samples;

            SyntheticAcquisitionTruthTableSatellite {
                sat: sat_truth.sat,
                injected_doppler_hz: sat_truth.doppler_hz,
                expected_measured_doppler_hz,
                measured_doppler_hz,
                doppler_error_hz,
                doppler_error_bins,
                injected_code_phase_chips: sat_truth.code_phase_chips,
                expected_code_phase_samples,
                measured_code_phase_samples,
                code_phase_error_samples,
                peak_mean_ratio: result.peak_mean_ratio,
                hypothesis: result.hypothesis.to_string(),
                doppler_pass,
                code_phase_pass,
                pass: doppler_pass && code_phase_pass,
            }
        })
        .collect::<Vec<_>>();
    let pass = !satellites.is_empty() && satellites.iter().all(|row| row.pass);

    SyntheticAcquisitionTruthTableReport {
        scenario_id: truth.scenario_id.clone(),
        doppler_tolerance_bins,
        doppler_tolerance_hz,
        code_phase_tolerance_samples,
        sample_rate_hz: truth.sample_rate_hz,
        period_samples,
        doppler_step_hz,
        pass,
        satellites,
    }
}

/// Measure truth-guided acquisition code-phase accuracy from a synthetic capture.
pub fn validate_truth_guided_acquisition_code_phase(
    config: &ReceiverPipelineConfig,
    frame: &SamplesFrame,
    truth: &SyntheticIqTruthBundle,
    tolerance_samples: usize,
) -> SyntheticAcquisitionCodePhaseValidationReport {
    let period_samples =
        samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length)
            .max(1);
    let satellites = truth
        .satellites
        .iter()
        .map(|sat_truth| {
            let isolated_frame = regenerate_isolated_scaled_satellite_signal_only_frame(
                config, frame, truth, sat_truth,
            );
            let mut acquisition_config = config.clone();
            acquisition_config.intermediate_freq_hz = synthetic_carrier_hz(
                truth.intermediate_freq_hz,
                sat_truth.sat,
                synthetic_truth_measured_doppler_hz(truth, sat_truth),
            );
            acquisition_config.acquisition_doppler_search_hz = 0;
            acquisition_config.acquisition_doppler_step_hz = 1;
            let acquisition = crate::pipeline::acquisition::Acquisition::new(
                acquisition_config,
                crate::engine::runtime::ReceiverRuntime::default(),
            )
            .with_doppler(0, 1);
            let result = acquisition.run_fft(&isolated_frame, &[sat_truth.sat]).remove(0);
            let expected_code_phase_samples = expected_acquisition_code_phase_samples(
                config,
                &isolated_frame,
                sat_truth.code_phase_chips,
            );
            let code_phase_error_samples = wrapped_code_phase_error_samples(
                result.code_phase_samples,
                expected_code_phase_samples,
                period_samples,
            );
            let pass = matches!(
                result.hypothesis,
                crate::api::core::AcqHypothesis::Accepted
                    | crate::api::core::AcqHypothesis::Ambiguous
            ) && code_phase_error_samples <= tolerance_samples;

            SyntheticAcquisitionCodePhaseValidationSatellite {
                sat: sat_truth.sat,
                injected_code_phase_chips: sat_truth.code_phase_chips,
                expected_code_phase_samples,
                measured_code_phase_samples: result.code_phase_samples,
                code_phase_error_samples,
                peak_mean_ratio: result.peak_mean_ratio,
                hypothesis: result.hypothesis.to_string(),
                pass,
            }
        })
        .collect::<Vec<_>>();
    let pass = !satellites.is_empty() && satellites.iter().all(|row| row.pass);

    SyntheticAcquisitionCodePhaseValidationReport {
        scenario_id: truth.scenario_id.clone(),
        tolerance_samples,
        sample_rate_hz: truth.sample_rate_hz,
        period_samples,
        pass,
        satellites,
    }
}

/// Measure whether acquisition code-phase refinement improves pseudorange initialization.
pub fn validate_truth_guided_acquisition_code_phase_refinement(
    config: &ReceiverPipelineConfig,
    frame: &SamplesFrame,
    truth: &SyntheticIqTruthBundle,
) -> SyntheticAcquisitionCodePhaseRefinementReport {
    let period_samples =
        samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length)
            .max(1);
    let satellites = truth
        .satellites
        .iter()
        .map(|sat_truth| {
            let isolated_frame = regenerate_isolated_scaled_satellite_signal_only_frame(
                config, frame, truth, sat_truth,
            );
            let mut acquisition_config = config.clone();
            acquisition_config.intermediate_freq_hz = synthetic_carrier_hz(
                truth.intermediate_freq_hz,
                sat_truth.sat,
                synthetic_truth_measured_doppler_hz(truth, sat_truth),
            );
            acquisition_config.acquisition_doppler_search_hz = 0;
            acquisition_config.acquisition_doppler_step_hz = 1;
            let acquisition = crate::pipeline::acquisition::Acquisition::new(
                acquisition_config,
                crate::engine::runtime::ReceiverRuntime::default(),
            )
            .with_doppler(0, 1);
            let result = acquisition.run_fft(&isolated_frame, &[sat_truth.sat]).remove(0);
            let expected_code_phase_samples = expected_acquisition_code_phase_samples_f64(
                config,
                &isolated_frame,
                sat_truth.code_phase_chips,
            );
            let coarse_code_phase_samples = result.code_phase_samples;
            let refined_code_phase_samples = result.resolved_code_phase_samples();
            let coarse_error_samples = wrapped_code_phase_error_samples_f64(
                coarse_code_phase_samples as f64,
                expected_code_phase_samples,
                period_samples,
            );
            let refined_error_samples = wrapped_code_phase_error_samples_f64(
                refined_code_phase_samples,
                expected_code_phase_samples,
                period_samples,
            );
            let coarse_pseudorange_error_m = code_phase_error_samples_to_pseudorange_m(
                coarse_error_samples,
                config.sampling_freq_hz,
            );
            let refined_pseudorange_error_m = code_phase_error_samples_to_pseudorange_m(
                refined_error_samples,
                config.sampling_freq_hz,
            );
            let improvement_samples = coarse_error_samples - refined_error_samples;
            let improvement_m = coarse_pseudorange_error_m - refined_pseudorange_error_m;
            let pass = matches!(
                result.hypothesis,
                crate::api::core::AcqHypothesis::Accepted
                    | crate::api::core::AcqHypothesis::Ambiguous
            ) && refined_error_samples <= coarse_error_samples + f64::EPSILON;

            SyntheticAcquisitionCodePhaseRefinementSatellite {
                sat: sat_truth.sat,
                injected_code_phase_chips: sat_truth.code_phase_chips,
                expected_code_phase_samples,
                coarse_code_phase_samples,
                refined_code_phase_samples,
                coarse_error_samples,
                refined_error_samples,
                coarse_pseudorange_error_m,
                refined_pseudorange_error_m,
                improvement_samples,
                improvement_m,
                peak_mean_ratio: result.peak_mean_ratio,
                hypothesis: result.hypothesis.to_string(),
                pass,
            }
        })
        .collect::<Vec<_>>();
    let pass = !satellites.is_empty() && satellites.iter().all(|row| row.pass);

    SyntheticAcquisitionCodePhaseRefinementReport {
        scenario_id: truth.scenario_id.clone(),
        sample_rate_hz: truth.sample_rate_hz,
        period_samples,
        pass,
        satellites,
    }
}

/// Measure truth-guided acquisition Doppler accuracy from a synthetic capture.
pub fn validate_truth_guided_acquisition_doppler(
    config: &ReceiverPipelineConfig,
    frame: &SamplesFrame,
    truth: &SyntheticIqTruthBundle,
    tolerance_bins: usize,
) -> SyntheticAcquisitionDopplerValidationReport {
    let doppler_step_hz = config.acquisition_doppler_step_hz.max(1);
    let tolerance_hz = tolerance_bins as f64 * doppler_step_hz as f64;
    let satellites = truth
        .satellites
        .iter()
        .map(|sat_truth| {
            let isolated_frame = regenerate_isolated_scaled_satellite_signal_only_frame(
                config, frame, truth, sat_truth,
            );
            let acquisition = crate::pipeline::acquisition::Acquisition::new(
                config.clone(),
                crate::engine::runtime::ReceiverRuntime::default(),
            );
            let result = acquisition.run_fft(&isolated_frame, &[sat_truth.sat]).remove(0);
            let measured_doppler_hz = crate::pipeline::doppler::doppler_hz_from_carrier_hz(
                config.intermediate_freq_hz,
                result.carrier_hz.0,
            );
            let expected_measured_doppler_hz =
                synthetic_truth_measured_doppler_hz(truth, sat_truth);
            let doppler_error_hz = (measured_doppler_hz - expected_measured_doppler_hz).abs();
            let doppler_error_bins = doppler_error_hz / doppler_step_hz as f64;
            let pass =
                measured_doppler_hz.is_finite() && doppler_error_hz <= tolerance_hz + f64::EPSILON;

            SyntheticAcquisitionDopplerValidationSatellite {
                sat: sat_truth.sat,
                injected_doppler_hz: sat_truth.doppler_hz,
                expected_measured_doppler_hz,
                measured_doppler_hz,
                doppler_error_hz,
                doppler_step_hz,
                doppler_error_bins,
                peak_mean_ratio: result.peak_mean_ratio,
                hypothesis: result.hypothesis.to_string(),
                pass,
            }
        })
        .collect::<Vec<_>>();
    let pass = !satellites.is_empty() && satellites.iter().all(|row| row.pass);

    SyntheticAcquisitionDopplerValidationReport {
        scenario_id: truth.scenario_id.clone(),
        tolerance_bins,
        tolerance_hz,
        sample_rate_hz: truth.sample_rate_hz,
        doppler_step_hz,
        pass,
        satellites,
    }
}

/// Validate acquisition receiver clock-offset recovery from a synthetic capture.
pub fn validate_truth_guided_acquisition_receiver_clock_offset(
    config: &ReceiverPipelineConfig,
    frame: &SamplesFrame,
    truth: &SyntheticIqTruthBundle,
    tolerance_bins: usize,
) -> SyntheticAcquisitionReceiverClockOffsetValidationReport {
    let doppler_step_hz = config.acquisition_doppler_step_hz.max(1);
    let tolerance_hz = tolerance_bins as f64 * doppler_step_hz as f64;
    let injected_receiver_clock_frequency_bias_hz = truth.receiver_clock_frequency_bias_hz;
    let satellites = truth
        .satellites
        .iter()
        .map(|sat_truth| {
            let isolated_frame = regenerate_isolated_scaled_satellite_signal_only_frame(
                config, frame, truth, sat_truth,
            );
            let acquisition = crate::pipeline::acquisition::Acquisition::new(
                config.clone(),
                crate::engine::runtime::ReceiverRuntime::default(),
            );
            let result = acquisition.run_fft(&isolated_frame, &[sat_truth.sat]).remove(0);
            let measured_doppler_hz = crate::pipeline::doppler::doppler_hz_from_carrier_hz(
                config.intermediate_freq_hz,
                result.carrier_hz.0,
            );
            let expected_measured_doppler_hz =
                sat_truth.doppler_hz + injected_receiver_clock_frequency_bias_hz;
            let measured_receiver_clock_frequency_bias_hz =
                measured_doppler_hz - sat_truth.doppler_hz;
            let receiver_clock_frequency_bias_error_hz = (measured_receiver_clock_frequency_bias_hz
                - injected_receiver_clock_frequency_bias_hz)
                .abs();
            let pass = measured_doppler_hz.is_finite()
                && receiver_clock_frequency_bias_error_hz <= tolerance_hz + f64::EPSILON;

            SyntheticAcquisitionReceiverClockOffsetSatellite {
                sat: sat_truth.sat,
                injected_doppler_hz: sat_truth.doppler_hz,
                injected_receiver_clock_frequency_bias_hz,
                expected_measured_doppler_hz,
                measured_doppler_hz,
                measured_receiver_clock_frequency_bias_hz,
                receiver_clock_frequency_bias_error_hz,
                doppler_step_hz,
                peak_mean_ratio: result.peak_mean_ratio,
                hypothesis: result.hypothesis.to_string(),
                pass,
            }
        })
        .collect::<Vec<_>>();
    let min_measured_receiver_clock_frequency_bias_hz = satellites
        .iter()
        .map(|row| row.measured_receiver_clock_frequency_bias_hz)
        .min_by(|left, right| left.partial_cmp(right).unwrap_or(std::cmp::Ordering::Equal))
        .unwrap_or(0.0);
    let max_measured_receiver_clock_frequency_bias_hz = satellites
        .iter()
        .map(|row| row.measured_receiver_clock_frequency_bias_hz)
        .max_by(|left, right| left.partial_cmp(right).unwrap_or(std::cmp::Ordering::Equal))
        .unwrap_or(0.0);
    let measured_receiver_clock_frequency_bias_spread_hz =
        max_measured_receiver_clock_frequency_bias_hz
            - min_measured_receiver_clock_frequency_bias_hz;
    let mean_measured_receiver_clock_frequency_bias_hz = if satellites.is_empty() {
        0.0
    } else {
        satellites.iter().map(|row| row.measured_receiver_clock_frequency_bias_hz).sum::<f64>()
            / satellites.len() as f64
    };
    let pass = !satellites.is_empty()
        && satellites.iter().all(|row| row.pass)
        && measured_receiver_clock_frequency_bias_spread_hz <= tolerance_hz + f64::EPSILON;

    SyntheticAcquisitionReceiverClockOffsetValidationReport {
        scenario_id: truth.scenario_id.clone(),
        injected_receiver_clock_frequency_bias_hz,
        tolerance_bins,
        tolerance_hz,
        sample_rate_hz: truth.sample_rate_hz,
        doppler_step_hz,
        mean_measured_receiver_clock_frequency_bias_hz,
        min_measured_receiver_clock_frequency_bias_hz,
        max_measured_receiver_clock_frequency_bias_hz,
        measured_receiver_clock_frequency_bias_spread_hz,
        pass,
        satellites,
    }
}

/// Validate acquisition code phase and Doppler for a coherent integration profile.
pub fn validate_truth_guided_acquisition_coherent_integration(
    config: &ReceiverPipelineConfig,
    frame: &SamplesFrame,
    truth: &SyntheticIqTruthBundle,
    coherent_ms: u32,
    noncoherent: u32,
    code_phase_tolerance_samples: usize,
    doppler_tolerance_bins: usize,
) -> SyntheticAcquisitionCoherentIntegrationReport {
    let mut profile_config = config.clone();
    profile_config.acquisition_integration_ms = coherent_ms;
    profile_config.acquisition_noncoherent = noncoherent;

    let code_phase_report = validate_truth_guided_acquisition_code_phase(
        &profile_config,
        frame,
        truth,
        code_phase_tolerance_samples,
    );
    let doppler_report = validate_truth_guided_acquisition_doppler(
        &profile_config,
        frame,
        truth,
        doppler_tolerance_bins,
    );

    let satellites = code_phase_report
        .satellites
        .iter()
        .zip(doppler_report.satellites.iter())
        .map(|(code_phase_row, doppler_row)| {
            debug_assert_eq!(code_phase_row.sat.prn, doppler_row.sat.prn);
            SyntheticAcquisitionCoherentIntegrationSatellite {
                sat: code_phase_row.sat,
                coherent_ms,
                noncoherent,
                code_phase_error_samples: code_phase_row.code_phase_error_samples,
                doppler_error_hz: doppler_row.doppler_error_hz,
                doppler_error_bins: doppler_row.doppler_error_bins,
                peak_mean_ratio: code_phase_row.peak_mean_ratio,
                hypothesis: code_phase_row.hypothesis.clone(),
                pass: code_phase_row.pass && doppler_row.pass,
            }
        })
        .collect::<Vec<_>>();
    let pass = !satellites.is_empty() && satellites.iter().all(|row| row.pass);

    SyntheticAcquisitionCoherentIntegrationReport {
        scenario_id: truth.scenario_id.clone(),
        coherent_ms,
        noncoherent,
        code_phase_tolerance_samples,
        doppler_tolerance_bins,
        doppler_tolerance_hz: doppler_report.tolerance_hz,
        sample_rate_hz: truth.sample_rate_hz,
        pass,
        satellites,
    }
}

/// Measure low-C/N0 detection probability for an acquisition integration profile.
pub fn measure_truth_guided_acquisition_detection_probability(
    config: &ReceiverPipelineConfig,
    signal: SyntheticSignalParams,
    coherent_ms: u32,
    noncoherent: u32,
    trial_seeds: &[u64],
    scenario_id_prefix: &str,
    code_phase_tolerance_samples: usize,
    doppler_tolerance_bins: usize,
) -> SyntheticAcquisitionSensitivityReport {
    let mut profile_config = config.clone();
    profile_config.acquisition_integration_ms = coherent_ms;
    profile_config.acquisition_noncoherent = noncoherent;
    let doppler_step_hz = profile_config.acquisition_doppler_step_hz.max(1);
    let duration_s = (coherent_ms.saturating_mul(noncoherent).max(1) as f64) / 1000.0;

    let trials = trial_seeds
        .iter()
        .enumerate()
        .map(|(trial_index, seed)| {
            let scenario_id = format!("{scenario_id_prefix}_trial_{trial_index}");
            let scenario = SyntheticScenario {
                sample_rate_hz: profile_config.sampling_freq_hz,
                intermediate_freq_hz: profile_config.intermediate_freq_hz,
                receiver_clock_frequency_bias_hz: 0.0,
                duration_s,
                seed: *seed,
                satellites: vec![signal],
                ephemerides: Vec::new(),
                id: scenario_id.clone(),
            };
            let frame = generate_l1_ca_multi(&profile_config, &scenario);
            let bundle = build_iq16_capture_bundle(
                &scenario.id,
                &scenario,
                &frame,
                "2026-07-09T00:00:00Z",
                Some("synthetic acquisition sensitivity trial".to_string()),
            );
            let scaled_frame = SamplesFrame::new(
                frame.t0,
                frame.dt_s,
                frame.iq.iter().map(|sample| *sample * bundle.truth.output_scale_applied).collect(),
            );
            let acquisition = crate::pipeline::acquisition::Acquisition::new(
                profile_config.clone(),
                crate::engine::runtime::ReceiverRuntime::default(),
            );
            let result = acquisition.run_fft(&scaled_frame, &[signal.sat]).remove(0);
            let expected_code_phase_samples = expected_acquisition_code_phase_samples(
                &profile_config,
                &scaled_frame,
                signal.code_phase_chips,
            );
            let period_samples = samples_per_code(
                profile_config.sampling_freq_hz,
                profile_config.code_freq_basis_hz,
                profile_config.code_length,
            );
            let code_phase_error_samples = wrapped_code_phase_error_samples(
                result.code_phase_samples,
                expected_code_phase_samples,
                period_samples,
            );
            let measured_doppler_hz = crate::pipeline::doppler::doppler_hz_from_carrier_hz(
                profile_config.intermediate_freq_hz,
                result.carrier_hz.0,
            );
            let doppler_error_bins =
                ((measured_doppler_hz - signal.doppler_hz).abs()) / doppler_step_hz as f64;
            let accepted = matches!(result.hypothesis, crate::api::core::AcqHypothesis::Accepted);
            let detected = !matches!(result.hypothesis, crate::api::core::AcqHypothesis::Rejected)
                && code_phase_error_samples <= code_phase_tolerance_samples
                && doppler_error_bins <= doppler_tolerance_bins as f64 + f64::EPSILON;

            SyntheticAcquisitionSensitivityTrial {
                scenario_id,
                seed: *seed,
                sat: signal.sat,
                hypothesis: result.hypothesis.to_string(),
                accepted,
                detected,
                code_phase_error_samples: Some(code_phase_error_samples),
                doppler_error_bins: Some(doppler_error_bins),
                peak_mean_ratio: result.peak_mean_ratio,
            }
        })
        .collect::<Vec<_>>();

    synthetic_acquisition_sensitivity_report(
        scenario_id_prefix,
        signal.sat,
        Some(signal.cn0_db_hz),
        coherent_ms,
        noncoherent,
        code_phase_tolerance_samples,
        doppler_tolerance_bins,
        doppler_step_hz,
        trials,
    )
}

/// Measure acquisition detection rate across multiple C/N0, Doppler, and integration settings.
pub fn measure_truth_guided_acquisition_detection_rate(
    config: &ReceiverPipelineConfig,
    cases: &[SyntheticAcquisitionDetectionRateCase],
    trial_seeds: &[u64],
    scenario_id_prefix: &str,
    code_phase_tolerance_samples: usize,
    doppler_tolerance_bins: usize,
) -> SyntheticAcquisitionDetectionRateReport {
    let doppler_step_hz = config.acquisition_doppler_step_hz.max(1);
    let points = cases
        .iter()
        .map(|case| {
            let sensitivity = measure_truth_guided_acquisition_detection_probability(
                config,
                case.signal,
                case.coherent_ms,
                case.noncoherent,
                trial_seeds,
                &detection_rate_case_id(scenario_id_prefix, case),
                code_phase_tolerance_samples,
                doppler_tolerance_bins,
            );

            SyntheticAcquisitionDetectionRatePoint {
                sat: case.signal.sat,
                cn0_db_hz: case.signal.cn0_db_hz,
                doppler_hz: case.signal.doppler_hz,
                coherent_ms: case.coherent_ms,
                noncoherent: case.noncoherent,
                trial_count: sensitivity.trial_count,
                accepted_count: sensitivity.accepted_count,
                detected_count: sensitivity.detected_count,
                acceptance_probability: sensitivity.acceptance_probability,
                detection_probability: sensitivity.detection_probability,
                mean_peak_mean_ratio: sensitivity.mean_peak_mean_ratio,
            }
        })
        .collect::<Vec<_>>();

    SyntheticAcquisitionDetectionRateReport {
        scenario_id_prefix: scenario_id_prefix.to_string(),
        code_phase_tolerance_samples,
        doppler_tolerance_bins,
        doppler_step_hz,
        points,
    }
}

/// Measure stable tracking lock probability for one synthetic signal profile.
pub fn measure_truth_guided_tracking_lock_probability(
    config: &ReceiverPipelineConfig,
    signal: SyntheticSignalParams,
    duration_s: f64,
    trial_seeds: &[u64],
    scenario_id_prefix: &str,
    seeded_doppler_error_hz: f64,
    seeded_code_phase_error_samples: isize,
    min_locked_epochs: usize,
) -> SyntheticTrackingSensitivityReport {
    let period_samples =
        samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length);

    let trials = trial_seeds
        .iter()
        .enumerate()
        .map(|(trial_index, seed)| {
            let scenario_id = format!("{scenario_id_prefix}_trial_{trial_index}");
            let scenario = SyntheticScenario {
                sample_rate_hz: config.sampling_freq_hz,
                intermediate_freq_hz: config.intermediate_freq_hz,
                receiver_clock_frequency_bias_hz: 0.0,
                duration_s,
                seed: *seed,
                satellites: vec![signal],
                ephemerides: Vec::new(),
                id: scenario_id.clone(),
            };
            let frame = generate_l1_ca_multi(config, &scenario);
            let expected_code_phase_samples =
                signal.code_phase_chips * config.sampling_freq_hz / config.code_freq_basis_hz;
            let seeded_code_phase_samples = wrap_seeded_code_phase_samples(
                expected_code_phase_samples.round() as isize + seeded_code_phase_error_samples,
                period_samples,
            );
            let tracking = crate::pipeline::tracking::Tracking::new(
                config.clone(),
                crate::engine::runtime::ReceiverRuntime::default(),
            );
            let tracks = tracking.track_from_acquisition(
                &frame,
                &[seeded_tracking_acquisition(
                    signal.sat,
                    signal.doppler_hz + seeded_doppler_error_hz,
                    seeded_code_phase_samples,
                    signal.cn0_db_hz,
                    format!(
                        "synthetic_tracking_seed_doppler_error_{:+.3}_code_phase_error_{}",
                        seeded_doppler_error_hz, seeded_code_phase_error_samples
                    ),
                )],
            );
            let epochs = &tracks.first().expect("track").epochs;
            let stable_window =
                stable_tracking_window_bounds(epochs, min_locked_epochs).unwrap_or((0, 0));
            let last_epoch = epochs.last();

            SyntheticTrackingSensitivityTrial {
                scenario_id,
                seed: *seed,
                sat: signal.sat,
                stable_lock: stable_window.1 > 0,
                refused_lock: epochs.iter().any(|epoch| {
                    epoch.lock_state_reason.as_deref() == Some("cn0_below_tracking_lock_floor")
                }),
                first_lock_epoch_index: (stable_window.1 > 0).then_some(stable_window.0),
                locked_epoch_count: stable_window.1,
                final_lock_state: last_epoch
                    .map(|epoch| epoch.lock_state.clone())
                    .unwrap_or_else(|| "not_tracked".to_string()),
                final_lock_state_reason: last_epoch
                    .and_then(|epoch| epoch.lock_state_reason.clone()),
            }
        })
        .collect::<Vec<_>>();

    synthetic_tracking_sensitivity_report(
        scenario_id_prefix,
        signal.sat,
        signal.cn0_db_hz,
        duration_s,
        seeded_doppler_error_hz,
        seeded_code_phase_error_samples,
        min_locked_epochs,
        trials,
    )
}

/// Measure tracking lock rate across multiple C/N0 points.
pub fn measure_truth_guided_tracking_lock_rate(
    config: &ReceiverPipelineConfig,
    cases: &[SyntheticTrackingLockRateCase],
    trial_seeds: &[u64],
    scenario_id_prefix: &str,
) -> SyntheticTrackingLockRateReport {
    let points = cases
        .iter()
        .map(|case| {
            let sensitivity = measure_truth_guided_tracking_lock_probability(
                config,
                case.signal,
                case.duration_s,
                trial_seeds,
                &tracking_lock_rate_case_id(scenario_id_prefix, case),
                case.seeded_doppler_error_hz,
                case.seeded_code_phase_error_samples,
                case.min_locked_epochs,
            );

            SyntheticTrackingLockRatePoint {
                sat: case.signal.sat,
                cn0_db_hz: case.signal.cn0_db_hz,
                duration_s: case.duration_s,
                seeded_doppler_error_hz: case.seeded_doppler_error_hz,
                seeded_code_phase_error_samples: case.seeded_code_phase_error_samples,
                min_locked_epochs: case.min_locked_epochs,
                trial_count: sensitivity.trial_count,
                stable_lock_count: sensitivity.stable_lock_count,
                refused_lock_count: sensitivity.refused_lock_count,
                lock_probability: sensitivity.lock_probability,
                mean_locked_epochs: sensitivity.mean_locked_epochs,
            }
        })
        .collect::<Vec<_>>();

    SyntheticTrackingLockRateReport { scenario_id_prefix: scenario_id_prefix.to_string(), points }
}

/// Measure noise-only false-alarm rate for an acquisition integration profile.
pub fn measure_noise_only_acquisition_false_alarm_rate(
    config: &ReceiverPipelineConfig,
    sat: SatId,
    coherent_ms: u32,
    noncoherent: u32,
    trial_seeds: &[u64],
    scenario_id_prefix: &str,
) -> SyntheticAcquisitionFalseAlarmReport {
    let mut profile_config = config.clone();
    profile_config.acquisition_integration_ms = coherent_ms;
    profile_config.acquisition_noncoherent = noncoherent;
    let duration_s = (coherent_ms.saturating_mul(noncoherent).max(1) as f64) / 1000.0;

    let trials = trial_seeds
        .iter()
        .enumerate()
        .map(|(trial_index, seed)| {
            let scenario_id = format!("{scenario_id_prefix}_trial_{trial_index}");
            let scenario = SyntheticScenario {
                sample_rate_hz: profile_config.sampling_freq_hz,
                intermediate_freq_hz: profile_config.intermediate_freq_hz,
                receiver_clock_frequency_bias_hz: 0.0,
                duration_s,
                seed: *seed,
                satellites: Vec::new(),
                ephemerides: Vec::new(),
                id: scenario_id.clone(),
            };
            let frame = generate_l1_ca_multi(&profile_config, &scenario);
            let acquisition = crate::pipeline::acquisition::Acquisition::new(
                profile_config.clone(),
                crate::engine::runtime::ReceiverRuntime::default(),
            );
            let result = acquisition.run_fft(&frame, &[sat]).remove(0);
            let accepted = matches!(result.hypothesis, crate::api::core::AcqHypothesis::Accepted);

            SyntheticAcquisitionSensitivityTrial {
                scenario_id,
                seed: *seed,
                sat,
                hypothesis: result.hypothesis.to_string(),
                accepted,
                detected: false,
                code_phase_error_samples: None,
                doppler_error_bins: None,
                peak_mean_ratio: result.peak_mean_ratio,
            }
        })
        .collect::<Vec<_>>();
    let trial_count = trials.len();
    let false_alarm_count = trials.iter().filter(|trial| trial.accepted).count();
    let mean_peak_mean_ratio = if trial_count == 0 {
        0.0
    } else {
        trials.iter().map(|trial| trial.peak_mean_ratio as f64).sum::<f64>() / trial_count as f64
    };

    SyntheticAcquisitionFalseAlarmReport {
        scenario_id_prefix: scenario_id_prefix.to_string(),
        sat,
        coherent_ms,
        noncoherent,
        trial_count,
        false_alarm_count,
        false_alarm_rate: if trial_count == 0 {
            0.0
        } else {
            false_alarm_count as f64 / trial_count as f64
        },
        mean_peak_mean_ratio,
        trials,
    }
}

/// Measure noise-only false-alarm rate across multiple acquisition integration profiles.
pub fn measure_noise_only_acquisition_false_alarm_rates(
    config: &ReceiverPipelineConfig,
    cases: &[SyntheticAcquisitionFalseAlarmRateCase],
    trial_seeds: &[u64],
    scenario_id_prefix: &str,
) -> SyntheticAcquisitionFalseAlarmRateReport {
    let points = cases
        .iter()
        .map(|case| {
            let report = measure_noise_only_acquisition_false_alarm_rate(
                config,
                case.sat,
                case.coherent_ms,
                case.noncoherent,
                trial_seeds,
                &false_alarm_rate_case_id(scenario_id_prefix, case),
            );

            SyntheticAcquisitionFalseAlarmRatePoint {
                sat: case.sat,
                coherent_ms: case.coherent_ms,
                noncoherent: case.noncoherent,
                trial_count: report.trial_count,
                false_alarm_count: report.false_alarm_count,
                false_alarm_rate: report.false_alarm_rate,
                mean_peak_mean_ratio: report.mean_peak_mean_ratio,
            }
        })
        .collect::<Vec<_>>();

    SyntheticAcquisitionFalseAlarmRateReport {
        scenario_id_prefix: scenario_id_prefix.to_string(),
        acquisition_doppler_search_hz: config.acquisition_doppler_search_hz,
        acquisition_doppler_step_hz: config.acquisition_doppler_step_hz.max(1),
        points,
    }
}

fn synthetic_acquisition_sensitivity_report(
    scenario_id_prefix: &str,
    sat: SatId,
    cn0_db_hz: Option<f32>,
    coherent_ms: u32,
    noncoherent: u32,
    code_phase_tolerance_samples: usize,
    doppler_tolerance_bins: usize,
    doppler_step_hz: i32,
    trials: Vec<SyntheticAcquisitionSensitivityTrial>,
) -> SyntheticAcquisitionSensitivityReport {
    let trial_count = trials.len();
    let accepted_count = trials.iter().filter(|trial| trial.accepted).count();
    let detected_count = trials.iter().filter(|trial| trial.detected).count();
    let mean_peak_mean_ratio = if trial_count == 0 {
        0.0
    } else {
        trials.iter().map(|trial| trial.peak_mean_ratio as f64).sum::<f64>() / trial_count as f64
    };

    SyntheticAcquisitionSensitivityReport {
        scenario_id_prefix: scenario_id_prefix.to_string(),
        sat,
        cn0_db_hz: cn0_db_hz.unwrap_or_default(),
        coherent_ms,
        noncoherent,
        code_phase_tolerance_samples,
        doppler_tolerance_bins,
        doppler_step_hz,
        trial_count,
        accepted_count,
        detected_count,
        acceptance_probability: if trial_count == 0 {
            0.0
        } else {
            accepted_count as f64 / trial_count as f64
        },
        detection_probability: if trial_count == 0 {
            0.0
        } else {
            detected_count as f64 / trial_count as f64
        },
        mean_peak_mean_ratio,
        trials,
    }
}

fn synthetic_tracking_sensitivity_report(
    scenario_id_prefix: &str,
    sat: SatId,
    cn0_db_hz: f32,
    duration_s: f64,
    seeded_doppler_error_hz: f64,
    seeded_code_phase_error_samples: isize,
    min_locked_epochs: usize,
    trials: Vec<SyntheticTrackingSensitivityTrial>,
) -> SyntheticTrackingSensitivityReport {
    let trial_count = trials.len();
    let stable_lock_count = trials.iter().filter(|trial| trial.stable_lock).count();
    let refused_lock_count = trials.iter().filter(|trial| trial.refused_lock).count();
    let mean_locked_epochs = if trial_count == 0 {
        0.0
    } else {
        trials.iter().map(|trial| trial.locked_epoch_count as f64).sum::<f64>() / trial_count as f64
    };

    SyntheticTrackingSensitivityReport {
        scenario_id_prefix: scenario_id_prefix.to_string(),
        sat,
        cn0_db_hz,
        duration_s,
        seeded_doppler_error_hz,
        seeded_code_phase_error_samples,
        min_locked_epochs,
        trial_count,
        stable_lock_count,
        refused_lock_count,
        lock_probability: if trial_count == 0 {
            0.0
        } else {
            stable_lock_count as f64 / trial_count as f64
        },
        mean_locked_epochs,
        trials,
    }
}

fn detection_rate_case_id(
    scenario_id_prefix: &str,
    case: &SyntheticAcquisitionDetectionRateCase,
) -> String {
    format!(
        "{scenario_id_prefix}_prn_{}_cn0_{:03}_doppler_{:+05}_coherent_{}ms_noncoherent_{}",
        case.signal.sat.prn,
        (case.signal.cn0_db_hz * 10.0).round() as i32,
        case.signal.doppler_hz.round() as i32,
        case.coherent_ms,
        case.noncoherent,
    )
}

fn tracking_lock_rate_case_id(
    scenario_id_prefix: &str,
    case: &SyntheticTrackingLockRateCase,
) -> String {
    format!(
        "{scenario_id_prefix}_prn_{}_cn0_{:03}_doppler_error_{:+05}_code_error_{:+03}_duration_ms_{}",
        case.signal.sat.prn,
        (case.signal.cn0_db_hz * 10.0).round() as i32,
        case.seeded_doppler_error_hz.round() as i32,
        case.seeded_code_phase_error_samples,
        (case.duration_s * 1_000.0).round() as usize,
    )
}

fn false_alarm_rate_case_id(
    scenario_id_prefix: &str,
    case: &SyntheticAcquisitionFalseAlarmRateCase,
) -> String {
    format!(
        "{scenario_id_prefix}_prn_{}_coherent_{}ms_noncoherent_{}",
        case.sat.prn, case.coherent_ms, case.noncoherent,
    )
}

fn seeded_tracking_acquisition(
    sat: SatId,
    doppler_hz: f64,
    code_phase_samples: usize,
    cn0_db_hz: f32,
    explain_selection_reason: String,
) -> AcqResult {
    AcqResult {
        sat,
        signal_band: SignalBand::L1,
        source_time: ReceiverSampleTrace::default(),
        candidate_rank: 1,
        is_primary_candidate: true,
        doppler_hz: Hertz(doppler_hz),
        carrier_hz: Hertz(doppler_hz),
        code_phase_samples,
        peak: 1.0,
        second_peak: 0.1,
        mean: 0.01,
        peak_mean_ratio: 20.0,
        peak_second_ratio: 10.0,
        cn0_proxy: cn0_db_hz,
        score: 1.0,
        hypothesis: AcqHypothesis::Accepted,
        assumptions: None,
        evidence: Vec::new(),
        threshold_provenance: None,
        explain_selection_reason: Some(explain_selection_reason),
        doppler_refinement: None,
        code_phase_refinement: None,
        signal_delay_alignment: None,
        uncertainty: None,
    }
}

fn wrap_seeded_code_phase_samples(code_phase_samples: isize, period_samples: usize) -> usize {
    let period_samples = period_samples.max(1) as isize;
    code_phase_samples.rem_euclid(period_samples) as usize
}

fn stable_tracking_window_bounds(
    epochs: &[crate::api::core::TrackEpoch],
    min_locked_epochs: usize,
) -> Option<(usize, usize)> {
    if min_locked_epochs == 0 {
        return Some((0, epochs.len()));
    }

    let mut stable_start = None;
    for (index, epoch) in epochs.iter().enumerate() {
        let stable = tracking_epoch_is_stable(epoch);
        match (stable_start, stable) {
            (None, true) => stable_start = Some(index),
            (Some(start), false) => {
                if index - start >= min_locked_epochs {
                    return Some((start, index - start));
                }
                stable_start = None;
            }
            _ => {}
        }
    }

    if let Some(start) = stable_start {
        if epochs.len() - start >= min_locked_epochs {
            return Some((start, epochs.len() - start));
        }
    }

    None
}

/// Measure whether acquisition Doppler refinement improves on the raw search bin.
pub fn validate_truth_guided_acquisition_doppler_refinement(
    config: &ReceiverPipelineConfig,
    frame: &SamplesFrame,
    truth: &SyntheticIqTruthBundle,
) -> SyntheticAcquisitionDopplerRefinementReport {
    let doppler_step_hz = config.acquisition_doppler_step_hz.max(1);
    let satellites = truth
        .satellites
        .iter()
        .map(|sat_truth| {
            let isolated_frame = regenerate_isolated_scaled_satellite_signal_only_frame(
                config, frame, truth, sat_truth,
            );
            let acquisition = crate::pipeline::acquisition::Acquisition::new(
                config.clone(),
                crate::engine::runtime::ReceiverRuntime::default(),
            );
            let result = acquisition.run_fft(&isolated_frame, &[sat_truth.sat]).remove(0);
            let refined_doppler_hz = crate::pipeline::doppler::doppler_hz_from_carrier_hz(
                config.intermediate_freq_hz,
                result.carrier_hz.0,
            );
            let (coarse_doppler_hz, refinement_bins) = result
                .doppler_refinement
                .as_ref()
                .map(|refinement| {
                    (
                        crate::pipeline::doppler::doppler_hz_from_carrier_hz(
                            config.intermediate_freq_hz,
                            refinement.coarse_carrier_hz.0,
                        ),
                        refinement.offset_bins,
                    )
                })
                .unwrap_or((refined_doppler_hz, 0.0));
            let expected_measured_doppler_hz =
                synthetic_truth_measured_doppler_hz(truth, sat_truth);
            let coarse_error_hz = (coarse_doppler_hz - expected_measured_doppler_hz).abs();
            let refined_error_hz = (refined_doppler_hz - expected_measured_doppler_hz).abs();
            let improvement_hz = coarse_error_hz - refined_error_hz;
            let pass = result.doppler_refinement.is_some() && improvement_hz > f64::EPSILON;

            SyntheticAcquisitionDopplerRefinementSatellite {
                sat: sat_truth.sat,
                injected_doppler_hz: sat_truth.doppler_hz,
                coarse_doppler_hz,
                refined_doppler_hz,
                coarse_error_hz,
                refined_error_hz,
                improvement_hz,
                doppler_step_hz,
                refinement_bins,
                peak_mean_ratio: result.peak_mean_ratio,
                hypothesis: result.hypothesis.to_string(),
                pass,
            }
        })
        .collect::<Vec<_>>();
    let pass = !satellites.is_empty() && satellites.iter().all(|row| row.pass);

    SyntheticAcquisitionDopplerRefinementReport {
        scenario_id: truth.scenario_id.clone(),
        sample_rate_hz: truth.sample_rate_hz,
        doppler_step_hz,
        pass,
        satellites,
    }
}

/// Validate acquisition code phase and Doppler across multiple sample-rate profiles.
pub fn validate_truth_guided_acquisition_sample_rates(
    cases: &[SyntheticAcquisitionSampleRateValidationCase<'_>],
    code_phase_tolerance_samples: usize,
    doppler_tolerance_bins: usize,
) -> SyntheticAcquisitionSampleRateValidationReport {
    let mut points = cases
        .iter()
        .map(|case| {
            let code_phase_validation = validate_truth_guided_acquisition_code_phase(
                case.config,
                case.frame,
                case.truth,
                code_phase_tolerance_samples,
            );
            let doppler_validation = validate_truth_guided_acquisition_doppler(
                case.config,
                case.frame,
                case.truth,
                doppler_tolerance_bins,
            );
            let pass = code_phase_validation.pass && doppler_validation.pass;
            SyntheticAcquisitionSampleRateValidationPoint {
                scenario_id: case.truth.scenario_id.clone(),
                sample_rate_hz: case.truth.sample_rate_hz,
                code_phase_validation,
                doppler_validation,
                pass,
            }
        })
        .collect::<Vec<_>>();
    points.sort_by(|left, right| {
        left.sample_rate_hz
            .partial_cmp(&right.sample_rate_hz)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then_with(|| left.scenario_id.cmp(&right.scenario_id))
    });

    let distinct_sample_rate_count = points
        .iter()
        .map(|point| point.sample_rate_hz.to_bits())
        .collect::<std::collections::BTreeSet<_>>()
        .len();
    let min_sample_rate_hz = points.first().map(|point| point.sample_rate_hz).unwrap_or(0.0);
    let max_sample_rate_hz = points.last().map(|point| point.sample_rate_hz).unwrap_or(0.0);
    let doppler_tolerance_hz = points
        .first()
        .map(|point| point.doppler_validation.tolerance_hz)
        .unwrap_or(doppler_tolerance_bins as f64);
    let pass = distinct_sample_rate_count >= 2
        && !points.is_empty()
        && points.iter().all(|point| point.pass);

    SyntheticAcquisitionSampleRateValidationReport {
        code_phase_tolerance_samples,
        doppler_tolerance_bins,
        doppler_tolerance_hz,
        distinct_sample_rate_count,
        min_sample_rate_hz,
        max_sample_rate_hz,
        pass,
        points,
    }
}

/// Generate a synthetic GPS L1 C/A signal at the receiver sample rate.
///
/// The C/N0 control is approximate and intended for test harnesses.
pub fn generate_l1_ca(
    config: &ReceiverPipelineConfig,
    params: SyntheticSignalParams,
    seed: u64,
    duration_s: f64,
) -> SamplesFrame {
    generate_l1_ca_multi(
        config,
        &SyntheticScenario {
            sample_rate_hz: config.sampling_freq_hz,
            intermediate_freq_hz: config.intermediate_freq_hz,
            receiver_clock_frequency_bias_hz: 0.0,
            duration_s,
            seed,
            satellites: vec![params],
            ephemerides: Vec::new(),
            id: "synthetic".to_string(),
        },
    )
}

/// Generate a synthetic GPS L1 C/A signal whose Doppler evolves linearly over time.
///
/// The `doppler_rate_hz_per_s` term models a moving-receiver or moving-satellite scenario
/// where the instantaneous carrier Doppler shifts continuously during the capture.
pub fn generate_l1_ca_with_doppler_ramp(
    config: &ReceiverPipelineConfig,
    params: SyntheticDopplerRampParams,
    seed: u64,
    duration_s: f64,
) -> SamplesFrame {
    let signal_only = generate_l1_ca_with_doppler_ramp_signal_only(config, params, duration_s);
    let clock = SampleClock::new(config.sampling_freq_hz);
    let dt_s = clock.dt_s();
    let noise_std = SYNTHETIC_NOISE_STD_PER_COMPONENT;
    let mut rng = XorShift64::new(seed);
    let mut iq = Vec::with_capacity(signal_only.len());
    for sample in signal_only.iq {
        let noise_i = rng.next_gaussian() * noise_std;
        let noise_q = rng.next_gaussian() * noise_std;
        let noise = Complex::new(noise_i, noise_q);
        iq.push(sample + noise);
    }
    SamplesFrame::new(signal_only.t0, Seconds(dt_s), iq)
}

/// Generate a synthetic GPS L1 C/A signal with deterministic signal-power fades.
pub fn generate_l1_ca_with_fades(
    config: &ReceiverPipelineConfig,
    params: SyntheticSignalParams,
    fade_windows: &[SyntheticFadeWindow],
    seed: u64,
    duration_s: f64,
) -> SamplesFrame {
    let mut signal_only = generate_l1_ca_signal_only(config, params, duration_s);
    apply_synthetic_fade_windows(&mut signal_only, fade_windows);
    let clock = SampleClock::new(config.sampling_freq_hz);
    let dt_s = clock.dt_s();
    let noise_std = SYNTHETIC_NOISE_STD_PER_COMPONENT;
    let mut rng = XorShift64::new(seed);
    let mut iq = Vec::with_capacity(signal_only.len());
    for sample in signal_only.iq {
        let noise_i = rng.next_gaussian() * noise_std;
        let noise_q = rng.next_gaussian() * noise_std;
        let noise = Complex::new(noise_i, noise_q);
        iq.push(sample + noise);
    }
    SamplesFrame::new(signal_only.t0, Seconds(dt_s), iq)
}

/// Generate a synthetic GPS L1 C/A signal with deterministic carrier phase-offset windows.
pub fn generate_l1_ca_with_phase_windows(
    config: &ReceiverPipelineConfig,
    params: SyntheticSignalParams,
    phase_windows: &[SyntheticPhaseWindow],
    seed: u64,
    duration_s: f64,
) -> SamplesFrame {
    let mut signal_only = generate_l1_ca_signal_only(config, params, duration_s);
    apply_synthetic_phase_windows(&mut signal_only, phase_windows);
    let clock = SampleClock::new(config.sampling_freq_hz);
    let dt_s = clock.dt_s();
    let noise_std = SYNTHETIC_NOISE_STD_PER_COMPONENT;
    let mut rng = XorShift64::new(seed);
    let mut iq = Vec::with_capacity(signal_only.len());
    for sample in signal_only.iq {
        let noise_i = rng.next_gaussian() * noise_std;
        let noise_q = rng.next_gaussian() * noise_std;
        let noise = Complex::new(noise_i, noise_q);
        iq.push(sample + noise);
    }
    SamplesFrame::new(signal_only.t0, Seconds(dt_s), iq)
}

pub fn generate_l1_ca_multi(
    config: &ReceiverPipelineConfig,
    scenario: &SyntheticScenario,
) -> SamplesFrame {
    let signal_only = generate_l1_ca_multi_signal_only(config, scenario);
    let clock = SampleClock::new(config.sampling_freq_hz);
    let dt_s = clock.dt_s();
    let noise_std = SYNTHETIC_NOISE_STD_PER_COMPONENT;
    let mut rng = XorShift64::new(scenario.seed);
    let mut iq = Vec::with_capacity(signal_only.len());
    for sample in signal_only.iq {
        let noise_i = rng.next_gaussian() * noise_std;
        let noise_q = rng.next_gaussian() * noise_std;
        let noise = Complex::new(noise_i, noise_q);
        iq.push(sample + noise);
    }
    SamplesFrame::new(signal_only.t0, Seconds(dt_s), iq)
}

/// Streaming synthetic GPS L1 C/A signal source for long-duration receiver tests.
pub struct SyntheticSignalSource {
    sample_rate_hz: f64,
    dt_s: f64,
    remaining_samples: usize,
    next_sample_index: u64,
    noise_std: f32,
    sat_states: Vec<SatState>,
    rng: XorShift64,
}

fn generate_l1_ca_with_doppler_ramp_signal_only(
    config: &ReceiverPipelineConfig,
    params: SyntheticDopplerRampParams,
    duration_s: f64,
) -> SamplesFrame {
    let clock = SampleClock::new(config.sampling_freq_hz);
    let dt_s = clock.dt_s();
    let sample_count = (duration_s * config.sampling_freq_hz).round() as usize;
    let sat_state = SatState::new_with_doppler_rate_and_receiver_clock_frequency_bias_hz(
        config,
        params.signal,
        0.0,
        params.doppler_rate_hz_per_s,
    );
    let mut iq = Vec::with_capacity(sample_count);
    for n in 0..sample_count {
        let t = n as f64 * dt_s;
        iq.push(sat_state.sample_at(t));
    }
    let t0 = SampleTime { sample_index: 0, sample_rate_hz: config.sampling_freq_hz };
    SamplesFrame::new(t0, Seconds(dt_s), iq)
}

fn generate_l1_ca_signal_only(
    config: &ReceiverPipelineConfig,
    params: SyntheticSignalParams,
    duration_s: f64,
) -> SamplesFrame {
    generate_l1_ca_multi_signal_only(
        config,
        &SyntheticScenario {
            sample_rate_hz: config.sampling_freq_hz,
            intermediate_freq_hz: config.intermediate_freq_hz,
            receiver_clock_frequency_bias_hz: 0.0,
            duration_s,
            seed: 0,
            satellites: vec![params],
            ephemerides: Vec::new(),
            id: "synthetic".to_string(),
        },
    )
}

fn apply_synthetic_fade_windows(frame: &mut SamplesFrame, fade_windows: &[SyntheticFadeWindow]) {
    if fade_windows.is_empty() {
        return;
    }

    let sample_rate_hz = frame.t0.sample_rate_hz;
    for (offset, sample) in frame.iq.iter_mut().enumerate() {
        let sample_time_s = (frame.t0.sample_index as f64 + offset as f64) / sample_rate_hz;
        let signal_scale = synthetic_signal_scale_at_time_s(fade_windows, sample_time_s);
        *sample *= signal_scale;
    }
}

fn apply_synthetic_phase_windows(frame: &mut SamplesFrame, phase_windows: &[SyntheticPhaseWindow]) {
    if phase_windows.is_empty() {
        return;
    }

    let sample_rate_hz = frame.t0.sample_rate_hz;
    for (offset, sample) in frame.iq.iter_mut().enumerate() {
        let sample_time_s = (frame.t0.sample_index as f64 + offset as f64) / sample_rate_hz;
        let phase_offset_rad = synthetic_phase_offset_at_time_s(phase_windows, sample_time_s);
        if phase_offset_rad.abs() <= f64::EPSILON {
            continue;
        }
        let rotation = Complex::from_polar(1.0_f32, phase_offset_rad as f32);
        *sample *= rotation;
    }
}

fn synthetic_signal_scale_at_time_s(
    fade_windows: &[SyntheticFadeWindow],
    sample_time_s: f64,
) -> f32 {
    fade_windows
        .iter()
        .filter(|window| sample_time_s >= window.start_s && sample_time_s < window.end_s)
        .fold(1.0_f32, |signal_scale, window| signal_scale * window.signal_scale)
}

fn synthetic_phase_offset_at_time_s(
    phase_windows: &[SyntheticPhaseWindow],
    sample_time_s: f64,
) -> f64 {
    phase_windows
        .iter()
        .filter(|window| sample_time_s >= window.start_s && sample_time_s < window.end_s)
        .map(|window| window.phase_offset_rad)
        .sum()
}

fn generate_l1_ca_multi_signal_only(
    config: &ReceiverPipelineConfig,
    scenario: &SyntheticScenario,
) -> SamplesFrame {
    let clock = SampleClock::new(config.sampling_freq_hz);
    let dt_s = clock.dt_s();
    let sample_count = (scenario.duration_s * config.sampling_freq_hz).round() as usize;
    let sat_states: Vec<SatState> = scenario
        .satellites
        .iter()
        .map(|sat| {
            SatState::new_with_receiver_clock_frequency_bias_hz(
                config,
                *sat,
                scenario.receiver_clock_frequency_bias_hz,
            )
        })
        .collect();
    let mut iq = Vec::with_capacity(sample_count);
    for n in 0..sample_count {
        let t = n as f64 * dt_s;
        let mut sample = Complex::new(0.0f32, 0.0f32);
        for sat in &sat_states {
            sample += sat.sample_at(t);
        }
        iq.push(sample);
    }
    let t0 = SampleTime { sample_index: 0, sample_rate_hz: config.sampling_freq_hz };
    SamplesFrame::new(t0, Seconds(dt_s), iq)
}

impl SyntheticSignalSource {
    /// Build a streaming synthetic source from a scenario without materializing the full capture.
    pub fn new(config: &ReceiverPipelineConfig, scenario: &SyntheticScenario) -> Self {
        Self::with_noise_std(config, scenario, SYNTHETIC_NOISE_STD_PER_COMPONENT)
    }

    /// Build a streaming synthetic source that emits only the deterministic signal component.
    pub fn new_signal_only(config: &ReceiverPipelineConfig, scenario: &SyntheticScenario) -> Self {
        Self::with_noise_std(config, scenario, 0.0)
    }

    fn with_noise_std(
        config: &ReceiverPipelineConfig,
        scenario: &SyntheticScenario,
        noise_std: f32,
    ) -> Self {
        let sample_count = (scenario.duration_s * config.sampling_freq_hz).round() as usize;

        Self {
            sample_rate_hz: config.sampling_freq_hz,
            dt_s: 1.0 / config.sampling_freq_hz,
            remaining_samples: sample_count,
            next_sample_index: 0,
            noise_std,
            sat_states: scenario
                .satellites
                .iter()
                .map(|sat| {
                    SatState::new_with_receiver_clock_frequency_bias_hz(
                        config,
                        *sat,
                        scenario.receiver_clock_frequency_bias_hz,
                    )
                })
                .collect(),
            rng: XorShift64::new(scenario.seed),
        }
    }
}

impl SignalSource for SyntheticSignalSource {
    type Error = SampleSourceError;

    fn sample_rate_hz(&self) -> f64 {
        self.sample_rate_hz
    }

    fn next_frame(&mut self, frame_len: usize) -> Result<Option<SamplesFrame>, Self::Error> {
        if self.remaining_samples == 0 {
            return Ok(None);
        }
        let count = self.remaining_samples.min(frame_len.max(1));
        let t0 = SampleTime {
            sample_index: self.next_sample_index,
            sample_rate_hz: self.sample_rate_hz,
        };
        let mut iq = Vec::with_capacity(count);
        for offset in 0..count {
            let t = (self.next_sample_index + offset as u64) as f64 * self.dt_s;
            let mut sample = Complex::new(0.0f32, 0.0f32);
            for sat in &self.sat_states {
                sample += sat.sample_at(t);
            }
            if self.noise_std <= f32::EPSILON {
                iq.push(sample);
            } else {
                let noise_i = self.rng.next_gaussian() * self.noise_std;
                let noise_q = self.rng.next_gaussian() * self.noise_std;
                iq.push(sample + Complex::new(noise_i, noise_q));
            }
        }
        self.next_sample_index += count as u64;
        self.remaining_samples -= count;
        Ok(Some(SamplesFrame::new(t0, Seconds(self.dt_s), iq)))
    }

    fn is_done(&self) -> bool {
        self.remaining_samples == 0
    }
}

#[derive(Debug, Clone)]
struct SatState {
    doppler_hz: f64,
    doppler_rate_hz_per_s: f64,
    receiver_clock_frequency_bias_hz: f64,
    code_phase_chips: f64,
    carrier_phase_rad: f64,
    cn0_db_hz: f32,
    data_bit_flip: bool,
    code: Vec<i8>,
    code_rate_hz: f64,
    if_hz: f64,
    sample_rate_hz: f64,
}

impl SatState {
    fn new_with_receiver_clock_frequency_bias_hz(
        config: &ReceiverPipelineConfig,
        params: SyntheticSignalParams,
        receiver_clock_frequency_bias_hz: f64,
    ) -> Self {
        Self::new_with_doppler_rate_and_receiver_clock_frequency_bias_hz(
            config,
            params,
            receiver_clock_frequency_bias_hz,
            0.0,
        )
    }

    fn new_with_doppler_rate_and_receiver_clock_frequency_bias_hz(
        config: &ReceiverPipelineConfig,
        params: SyntheticSignalParams,
        receiver_clock_frequency_bias_hz: f64,
        doppler_rate_hz_per_s: f64,
    ) -> Self {
        Self {
            doppler_hz: params.doppler_hz,
            doppler_rate_hz_per_s,
            receiver_clock_frequency_bias_hz,
            code_phase_chips: params.code_phase_chips,
            carrier_phase_rad: params.carrier_phase_rad,
            cn0_db_hz: params.cn0_db_hz,
            data_bit_flip: params.data_bit_flip,
            code: generate_ca_code(Prn(params.sat.prn)).unwrap_or_else(|_| vec![1; 1023]),
            code_rate_hz: config.code_freq_basis_hz,
            if_hz: synthetic_intermediate_frequency_hz(config.intermediate_freq_hz, params.sat),
            sample_rate_hz: config.sampling_freq_hz,
        }
    }

    fn carrier_hz_at(&self, t: f64) -> f64 {
        self.if_hz
            + self.doppler_hz
            + self.receiver_clock_frequency_bias_hz
            + self.doppler_rate_hz_per_s * t
    }

    fn carrier_phase_rad_at(&self, t: f64) -> f64 {
        let initial_carrier_hz = self.carrier_hz_at(0.0);
        self.carrier_phase_rad
            + std::f64::consts::TAU
                * (initial_carrier_hz * t + 0.5 * self.doppler_rate_hz_per_s * t * t)
    }

    fn sample_at(&self, t: f64) -> Complex<f32> {
        let code_phase = advance_code_phase_seconds(
            self.code_phase_chips,
            self.code_rate_hz,
            t,
            self.code.len(),
        )
        .expect("synthetic generator requires a valid code phase model");
        let chip = code_value_at_phase(&self.code, code_phase).unwrap_or(1.0);
        let data_bit = nav_bit_value_at_time_s(self.data_bit_flip, t);

        let phase = self.carrier_phase_rad_at(t) as f32;
        let carrier = Complex::new(phase.cos(), phase.sin());

        let amplitude = signal_amplitude_from_cn0(self.cn0_db_hz, self.sample_rate_hz);

        carrier * (chip * data_bit * amplitude)
    }
}

fn signal_amplitude_from_cn0(cn0_db_hz: f32, sample_rate_hz: f64) -> f32 {
    let cn0_linear = 10.0_f64.powf(cn0_db_hz as f64 / 10.0).max(1e-12);
    ((cn0_linear * SYNTHETIC_COMPLEX_NOISE_POWER) / sample_rate_hz).sqrt() as f32
}

fn regenerate_isolated_scaled_satellite_signal_only_frame(
    config: &ReceiverPipelineConfig,
    measured_frame: &SamplesFrame,
    truth: &SyntheticIqTruthBundle,
    sat_truth: &SyntheticSatelliteTruth,
) -> SamplesFrame {
    let isolated_frame = generate_l1_ca_multi_signal_only(
        config,
        &isolated_satellite_scenario(measured_frame, truth, sat_truth),
    );
    let iq = isolated_frame
        .iq
        .iter()
        .map(|sample| *sample * truth.output_scale_applied)
        .collect::<Vec<_>>();
    SamplesFrame::new(measured_frame.t0, measured_frame.dt_s, iq)
}

fn regenerate_isolated_scaled_satellite_frame_with_noise(
    config: &ReceiverPipelineConfig,
    measured_frame: &SamplesFrame,
    truth: &SyntheticIqTruthBundle,
    sat_truth: &SyntheticSatelliteTruth,
) -> SamplesFrame {
    let isolated_frame = generate_l1_ca_multi(
        config,
        &isolated_satellite_scenario(measured_frame, truth, sat_truth),
    );
    let iq = isolated_frame
        .iq
        .iter()
        .map(|sample| *sample * truth.output_scale_applied)
        .collect::<Vec<_>>();
    SamplesFrame::new(measured_frame.t0, measured_frame.dt_s, iq)
}

fn isolated_satellite_scenario(
    measured_frame: &SamplesFrame,
    truth: &SyntheticIqTruthBundle,
    sat_truth: &SyntheticSatelliteTruth,
) -> SyntheticScenario {
    SyntheticScenario {
        sample_rate_hz: truth.sample_rate_hz,
        intermediate_freq_hz: truth.intermediate_freq_hz,
        receiver_clock_frequency_bias_hz: truth.receiver_clock_frequency_bias_hz,
        duration_s: measured_frame.len() as f64 * measured_frame.dt_s.0,
        seed: truth.seed,
        satellites: vec![SyntheticSignalParams {
            sat: sat_truth.sat,
            doppler_hz: sat_truth.doppler_hz,
            code_phase_chips: sat_truth.code_phase_chips,
            carrier_phase_rad: sat_truth.carrier_phase_rad,
            cn0_db_hz: sat_truth.cn0_db_hz,
            data_bit_flip: sat_truth.nav_bit_mode == SyntheticNavBitMode::AlternatingGpsLnav20ms,
        }],
        ephemerides: Vec::new(),
        id: sat_truth.sat.prn.to_string(),
    }
}

fn code_phase_samples_at_epoch_start(
    config: &ReceiverPipelineConfig,
    frame: &SamplesFrame,
    code_phase_chips: f64,
) -> f64 {
    code_phase_samples_at_sample_index(
        config,
        frame.t0.sample_rate_hz,
        frame.t0.sample_index,
        code_phase_chips,
    )
}

fn synthetic_intermediate_frequency_hz(intermediate_freq_hz: f64, sat: SatId) -> f64 {
    intermediate_freq_hz
        + (synthetic_constellation_carrier_hz(sat)
            - bijux_gnss_core::api::GPS_L1_CA_CARRIER_HZ.value())
}

fn synthetic_carrier_hz(intermediate_freq_hz: f64, sat: SatId, doppler_hz: f64) -> f64 {
    carrier_hz_from_doppler_hz(
        synthetic_intermediate_frequency_hz(intermediate_freq_hz, sat),
        doppler_hz,
    )
}

fn synthetic_truth_measured_doppler_hz(
    truth: &SyntheticIqTruthBundle,
    sat_truth: &SyntheticSatelliteTruth,
) -> f64 {
    sat_truth.doppler_hz + truth.receiver_clock_frequency_bias_hz
}

fn synthetic_constellation_carrier_hz(sat: SatId) -> f64 {
    match sat.constellation {
        Constellation::Galileo => bijux_gnss_core::api::GALILEO_E1_CARRIER_HZ.value(),
        Constellation::Gps => bijux_gnss_core::api::GPS_L1_CA_CARRIER_HZ.value(),
        Constellation::Glonass => bijux_gnss_core::api::GLONASS_L1_CARRIER_HZ.value(),
        _ => bijux_gnss_core::api::GPS_L1_CA_CARRIER_HZ.value(),
    }
}

fn nav_bit_mode(params: &SyntheticSignalParams) -> SyntheticNavBitMode {
    if params.data_bit_flip {
        SyntheticNavBitMode::AlternatingGpsLnav20ms
    } else {
        SyntheticNavBitMode::ConstantPositive
    }
}

fn nav_bit_value_at_time_s(data_bit_flip: bool, time_s: f64) -> f32 {
    nav_bit_sign_at_time_s(data_bit_flip, time_s) as f32
}

fn nav_bit_sign_at_time_s(data_bit_flip: bool, time_s: f64) -> i8 {
    if !data_bit_flip {
        return 1;
    }
    nav_bit_sign_for_index(nav_bit_index_at_time_s(time_s))
}

fn nav_bit_index_at_time_s(time_s: f64) -> u64 {
    if !time_s.is_finite() || time_s <= 0.0 {
        return 0;
    }
    (time_s / GPS_L1_CA_NAV_BIT_PERIOD_S).floor() as u64
}

fn nav_bit_sign_for_index(bit_index: u64) -> i8 {
    if bit_index % 2 == 0 {
        1
    } else {
        -1
    }
}

fn peak_component(samples: &[Complex<f32>]) -> f32 {
    samples.iter().flat_map(|sample| [sample.re.abs(), sample.im.abs()]).fold(0.0f32, f32::max)
}

fn encode_iq16_le_bytes(samples: &[Complex<f32>], scale: f32) -> Vec<u8> {
    let mut encoded = Vec::with_capacity(samples.len() * 4);
    for sample in samples {
        encoded.extend_from_slice(&quantize_i16_component(sample.re * scale).to_le_bytes());
        encoded.extend_from_slice(&quantize_i16_component(sample.im * scale).to_le_bytes());
    }
    encoded
}

fn quantize_i16_component(value: f32) -> i16 {
    let scaled = (value * 32768.0).round();
    scaled.clamp(-32768.0, 32767.0) as i16
}

fn nav_bit_segments(
    sample_rate_hz: f64,
    sample_count: u64,
    data_bit_flip: bool,
) -> Vec<SyntheticNavBitSegment> {
    if sample_count == 0 {
        return Vec::new();
    }
    if !data_bit_flip {
        return vec![SyntheticNavBitSegment {
            start_sample: 0,
            end_sample: sample_count,
            start_s: 0.0,
            end_s: sample_count as f64 / sample_rate_hz,
            bit: 1,
        }];
    }

    let mut segments = Vec::new();
    let mut bit_index = 0u64;
    loop {
        let start_sample =
            ((bit_index as f64 * GPS_L1_CA_NAV_BIT_PERIOD_S * sample_rate_hz).ceil()) as u64;
        if start_sample >= sample_count {
            break;
        }
        let end_sample = ((((bit_index + 1) as f64) * GPS_L1_CA_NAV_BIT_PERIOD_S * sample_rate_hz)
            .ceil()) as u64;
        let clamped_end = end_sample.min(sample_count);
        segments.push(SyntheticNavBitSegment {
            start_sample,
            end_sample: clamped_end,
            start_s: start_sample as f64 / sample_rate_hz,
            end_s: clamped_end as f64 / sample_rate_hz,
            bit: nav_bit_sign_for_index(bit_index),
        });
        bit_index += 1;
    }
    segments
}

#[derive(Debug, Clone)]
struct XorShift64 {
    state: u64,
}

impl XorShift64 {
    fn new(seed: u64) -> Self {
        let seed = if seed == 0 { 0xDEADBEEFCAFEBABE } else { seed };
        Self { state: seed }
    }

    fn next_u64(&mut self) -> u64 {
        let mut x = self.state;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.state = x;
        x
    }

    fn next_f32(&mut self) -> f32 {
        let val = (self.next_u64() >> 40) as u32;
        val as f32 / (u32::MAX as f32)
    }

    fn next_gaussian(&mut self) -> f32 {
        let u1 = self.next_f32().max(1e-12);
        let u2 = self.next_f32();
        let r = (-2.0 * u1.ln()).sqrt();
        let theta = TAU * u2;
        r * theta.cos()
    }
}

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
        summarize_truth_guided_pvt_cn0_profile, summarize_truth_guided_pvt_geometry_profile,
        summarize_truth_guided_pvt_motion_profile, summarize_truth_guided_pvt_multipath_profile,
        summarize_truth_guided_pvt_time_profile, synthetic_tracking_sensitivity_report,
        truth_guided_receiver_accuracy_budgets, validate_acquisition_accuracy_budget,
        validate_pvt_accuracy_budget, validate_truth_guided_acquisition_code_phase,
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
        SyntheticPvtAccuracyReport, SyntheticPvtCn0ProfileCase, SyntheticPvtCn0ProfilePoint,
        SyntheticPvtCn0ProfileReport, SyntheticPvtGeometryProfileCase,
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
    use bijux_gnss_core::api::{
        ecef_to_geodetic, lla_to_ecef, Constellation, Cycles, Epoch, FreqHz, Hertz, LockFlags,
        Meters, NavLifecycleState, NavQualityFlag, NavSolutionEpoch, NavUncertaintyClass, ObsEpoch,
        ObsMetadata, ObsSatellite, ObservationEpochDecision, ObservationStatus, ReceiverRole,
        ReceiverSampleTrace, SampleTime, SamplesFrame, SatId, Seconds, SigId, SignalBand,
        SignalCode, SignalSpec, SolutionStatus, SolutionValidity, ValidationReferenceEpoch,
        NAV_OUTPUT_STABILITY_SIGNATURE_VERSION, NAV_SOLUTION_MODEL_VERSION,
    };
    use bijux_gnss_signal::api::{
        advance_code_phase_seconds, sample_ca_code, samples_per_code, IqSampleFormat, Prn,
        RawIqMetadata, SignalSource,
    };
    use num_complex::Complex;

    const RECEIVER_PHASE_TOLERANCE_SAMPLES: f64 = 1e-6;

    #[test]
    fn synthetic_signal_source_matches_materialized_generator() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 1_023_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            ..ReceiverPipelineConfig::default()
        };
        let scenario = SyntheticScenario {
            sample_rate_hz: config.sampling_freq_hz,
            intermediate_freq_hz: config.intermediate_freq_hz,
            receiver_clock_frequency_bias_hz: 0.0,
            duration_s: 0.004,
            seed: 29,
            satellites: vec![SyntheticSignalParams {
                sat: SatId { constellation: Constellation::Gps, prn: 7 },
                doppler_hz: 750.0,
                code_phase_chips: 15.0,
                carrier_phase_rad: 0.0,
                cn0_db_hz: 47.0,
                data_bit_flip: false,
            }],
            ephemerides: Vec::new(),
            id: "synthetic-stream".to_string(),
        };

        let expected = generate_l1_ca_multi(&config, &scenario);
        let mut source = SyntheticSignalSource::new(&config, &scenario);
        let streamed = collect_frames(&mut source, 2 * 1_023);

        assert_eq!(expected.len(), streamed.len());
        assert_eq!(expected.t0, streamed.t0);
        assert_eq!(expected.dt_s, streamed.dt_s);
        assert_eq!(expected.iq, streamed.iq);
        assert!(source.is_done());
    }

    #[test]
    fn pvt_truth_table_records_truth_measured_values_and_errors() {
        let truth_ecef = lla_to_ecef(37.0, -122.0, 10.0);
        let measured_ecef = (truth_ecef.0 + 1.5, truth_ecef.1 - 2.0, truth_ecef.2 + 0.75);
        let measured_geodetic = ecef_to_geodetic(measured_ecef.0, measured_ecef.1, measured_ecef.2);
        let truth_clock_bias_s = 2.0e-4;
        let measured_clock_bias_s = 2.5e-4;
        let solution = NavSolutionEpoch {
            epoch: Epoch { index: 7 },
            t_rx_s: Seconds(100_000.0),
            source_time: ReceiverSampleTrace::from_sample_index(0, 1.0),
            ecef_x_m: Meters(measured_ecef.0),
            ecef_y_m: Meters(measured_ecef.1),
            ecef_z_m: Meters(measured_ecef.2),
            latitude_deg: measured_geodetic.0,
            longitude_deg: measured_geodetic.1,
            altitude_m: Meters(measured_geodetic.2),
            clock_bias_s: Seconds(measured_clock_bias_s),
            clock_bias_m: Meters(measured_clock_bias_s * SPEED_OF_LIGHT_MPS),
            clock_drift_s_per_s: 0.0,
            pdop: 1.2,
            pre_fit_residual_rms_m: Some(Meters(3.5)),
            post_fit_residual_rms_m: Some(Meters(1.25)),
            rms_m: Meters(1.25),
            status: SolutionStatus::Converged,
            quality: NavQualityFlag::Float,
            validity: SolutionValidity::Stable,
            valid: true,
            processing_ms: None,
            residuals: Vec::new(),
            health: Vec::new(),
            isb: Vec::new(),
            sigma_h_m: None,
            sigma_v_m: None,
            innovation_rms_m: None,
            normalized_innovation_rms: None,
            normalized_innovation_max: None,
            ekf_innovation_rms: None,
            ekf_condition_number: None,
            ekf_whiteness_ratio: None,
            ekf_predicted_variance: None,
            ekf_observed_variance: None,
            integrity_hpl_m: None,
            integrity_vpl_m: None,
            model_version: NAV_SOLUTION_MODEL_VERSION,
            lifecycle_state: NavLifecycleState::Converged,
            uncertainty_class: NavUncertaintyClass::Low,
            assumptions: None,
            refusal_class: None,
            artifact_id: "nav-epoch-0000000007-pvt-truth".to_string(),
            source_observation_epoch_id: "obs-epoch-0000000007-pvt-truth".to_string(),
            explain_decision: "accepted".to_string(),
            explain_reasons: vec!["navigation_solution_usable".to_string()],
            provenance: None,
            sat_count: 5,
            used_sat_count: 4,
            rejected_sat_count: 1,
            hdop: Some(0.9),
            vdop: Some(0.8),
            gdop: Some(1.3),
            tdop: Some(0.4),
            stability_signature: "navsig:v2:pvt-truth".to_string(),
            stability_signature_version: NAV_OUTPUT_STABILITY_SIGNATURE_VERSION,
        };
        let reference = SyntheticPvtTruthReferenceEpoch {
            position: ValidationReferenceEpoch {
                epoch_idx: 7,
                t_rx_s: Some(100_000.0),
                latitude_deg: 37.0,
                longitude_deg: -122.0,
                altitude_m: 10.0,
                ecef_x_m: Some(truth_ecef.0),
                ecef_y_m: Some(truth_ecef.1),
                ecef_z_m: Some(truth_ecef.2),
                vel_x_mps: None,
                vel_y_mps: None,
                vel_z_mps: None,
            },
            clock_bias_s: truth_clock_bias_s,
        };

        let report =
            validate_truth_guided_pvt_table("unit_test_pvt_truth", &[solution], &[reference]);
        let row = report.epochs.first().expect("pvt truth row");

        assert_eq!(report.solution_count, 1);
        assert_eq!(report.matched_epoch_count, 1);
        assert!(report.unmatched_solution_epochs.is_empty());
        assert!(report.unused_reference_epochs.is_empty());
        assert_eq!(row.truth_ecef_m.x_m, truth_ecef.0);
        assert_eq!(row.measured_ecef_m.y_m, measured_ecef.1);
        assert_eq!(row.ecef_error_m.x_m, 1.5);
        assert_eq!(row.ecef_error_m.y_m, -2.0);
        assert_eq!(row.ecef_error_m.z_m, 0.75);
        assert_eq!(row.truth_geodetic.latitude_deg, 37.0);
        assert_eq!(row.truth_geodetic.longitude_deg, -122.0);
        assert_eq!(row.truth_geodetic.altitude_m, 10.0);
        assert_eq!(row.clock_bias.truth_s, truth_clock_bias_s);
        assert_eq!(row.clock_bias.measured_s, measured_clock_bias_s);
        assert_eq!(row.clock_bias.error_s, measured_clock_bias_s - truth_clock_bias_s);
        assert!(
            (row.clock_bias.error_m
                - (measured_clock_bias_s - truth_clock_bias_s) * SPEED_OF_LIGHT_MPS)
                .abs()
                <= 1.0e-9
        );
        assert_eq!(row.residual_rms_m, 1.25);
        assert_eq!(row.pre_fit_residual_rms_m, Some(3.5));
        assert_eq!(row.post_fit_residual_rms_m, Some(1.25));
        assert_eq!(row.dop.pdop, 1.2);
        assert_eq!(row.solution_status, SolutionStatus::Converged);
        assert_eq!(row.solution_quality, NavQualityFlag::Float);
        assert_eq!(row.solution_validity, SolutionValidity::Stable);
        assert!(row.valid);
    }

    #[test]
    fn truth_guided_receiver_accuracy_budgets_are_hard_and_positive() {
        let budgets = truth_guided_receiver_accuracy_budgets();

        assert!(budgets.acquisition.max_doppler_error_hz > 0.0);
        assert!(budgets.acquisition.max_code_phase_error_samples > 0);
        assert!(budgets.tracking.max_carrier_error_hz > 0.0);
        assert!(budgets.tracking.max_doppler_error_hz > 0.0);
        assert!(budgets.tracking.max_code_phase_error_samples > 0.0);
        assert!(budgets.tracking.max_cn0_error_db_hz > 0.0);
        assert!(budgets.observation.max_pseudorange_error_m > 0.0);
        assert!(budgets.observation.max_carrier_phase_error_cycles > 0.0);
        assert!(budgets.observation.max_doppler_error_hz > 0.0);
        assert!(budgets.observation.max_cn0_error_db_hz > 0.0);
        assert!(budgets.pvt.max_position_error_3d_m > 0.0);
        assert!(budgets.pvt.max_clock_bias_error_m > 0.0);
        assert!(budgets.pvt.max_residual_rms_m > 0.0);
        assert!(budgets.pvt.max_pdop > 0.0);
    }

    #[test]
    fn acquisition_accuracy_budget_fails_when_truth_error_exceeds_threshold() {
        let report = SyntheticAcquisitionTruthTableReport {
            scenario_id: "acquisition_budget_failure".to_string(),
            doppler_tolerance_bins: 1,
            doppler_tolerance_hz: 500.0,
            code_phase_tolerance_samples: 2,
            sample_rate_hz: 1_023_000.0,
            period_samples: 1023,
            doppler_step_hz: 500,
            pass: true,
            satellites: vec![SyntheticAcquisitionTruthTableSatellite {
                sat: SatId { constellation: Constellation::Gps, prn: 3 },
                injected_doppler_hz: 750.0,
                expected_measured_doppler_hz: 750.0,
                measured_doppler_hz: 1_500.0,
                doppler_error_hz: 750.0,
                doppler_error_bins: 1.5,
                injected_code_phase_chips: 200.25,
                expected_code_phase_samples: 100,
                measured_code_phase_samples: 104,
                code_phase_error_samples: 4,
                peak_mean_ratio: 10.0,
                hypothesis: "accepted".to_string(),
                doppler_pass: false,
                code_phase_pass: false,
                pass: false,
            }],
        };

        let accuracy = validate_acquisition_accuracy_budget(
            &report,
            truth_guided_receiver_accuracy_budgets().acquisition,
        );
        let satellite = accuracy.satellites.first().expect("acquisition satellite");

        assert!(!accuracy.pass);
        assert_eq!(accuracy.passing_satellite_count, 0);
        assert!(!satellite.pass);
    }

    #[test]
    fn acquisition_accuracy_budget_requires_truth_satellites() {
        let report = SyntheticAcquisitionTruthTableReport {
            scenario_id: "acquisition_missing_truth".to_string(),
            doppler_tolerance_bins: 1,
            doppler_tolerance_hz: 500.0,
            code_phase_tolerance_samples: 2,
            sample_rate_hz: 1_023_000.0,
            period_samples: 1023,
            doppler_step_hz: 500,
            pass: false,
            satellites: Vec::new(),
        };

        let accuracy = validate_acquisition_accuracy_budget(
            &report,
            truth_guided_receiver_accuracy_budgets().acquisition,
        );

        assert!(!accuracy.truth_coverage_ready);
        assert_eq!(accuracy.truth_coverage_issues.len(), 1);
        assert_eq!(accuracy.truth_coverage_issues[0].code, "no_truth_satellites");
        assert!(!accuracy.pass);
    }

    #[test]
    fn tracking_accuracy_budget_requires_stable_truth_epochs() {
        let report = SyntheticTrackingTruthTableReport {
            scenario_id: "tracking_missing_truth".to_string(),
            carrier_tolerance_hz: 10.0,
            doppler_tolerance_hz: 10.0,
            code_phase_tolerance_samples: 1.0,
            cn0_tolerance_db_hz: 8.0,
            sample_rate_hz: 1_023_000.0,
            period_samples: 1023,
            output_scale_applied: 1.0,
            pass: false,
            satellites: vec![SyntheticTrackingTruthTableSatellite {
                sat: SatId { constellation: Constellation::Gps, prn: 7 },
                injected_doppler_hz: -250.0,
                expected_measured_doppler_hz: -250.0,
                injected_code_phase_chips: 100.0,
                injected_cn0_db_hz: 45.0,
                epoch_count: 1,
                stable_epoch_count: 0,
                first_stable_epoch_index: None,
                pass: false,
                epochs: vec![SyntheticTrackingTruthTableEpoch {
                    epoch_index: 0,
                    sample_index: 0,
                    expected_carrier_hz: -250.0,
                    measured_carrier_hz: -250.0,
                    carrier_error_hz: 0.0,
                    expected_doppler_hz: -250.0,
                    measured_doppler_hz: -250.0,
                    doppler_error_hz: 0.0,
                    expected_code_phase_samples: 100.0,
                    measured_code_phase_samples: 100.0,
                    code_phase_error_samples: 0.0,
                    expected_cn0_db_hz: 45.0,
                    measured_cn0_dbhz: 45.0,
                    cn0_error_db: 0.0,
                    lock: false,
                    pll_lock: false,
                    dll_lock: false,
                    fll_lock: false,
                    cycle_slip: false,
                    lock_state: "degraded".to_string(),
                    lock_state_reason: Some("no_stable_truth_window".to_string()),
                    stable_tracking_epoch: false,
                    pass: false,
                }],
            }],
        };

        let accuracy = super::validate_tracking_accuracy_budget(
            &report,
            truth_guided_receiver_accuracy_budgets().tracking,
        );

        assert!(!accuracy.truth_coverage_ready);
        assert_eq!(accuracy.truth_coverage_issues.len(), 1);
        assert_eq!(accuracy.truth_coverage_issues[0].sat, Some(report.satellites[0].sat));
        assert_eq!(accuracy.truth_coverage_issues[0].code, "no_stable_tracking_truth_epochs");
        assert!(!accuracy.pass);
    }

    #[test]
    fn pvt_accuracy_budget_fails_invalid_or_out_of_budget_epoch() {
        let report = SyntheticPvtTruthTableReport {
            scenario_id: "pvt_budget_failure".to_string(),
            solution_count: 1,
            matched_epoch_count: 1,
            unmatched_solution_epochs: Vec::new(),
            unused_reference_epochs: Vec::new(),
            epochs: vec![SyntheticPvtTruthTableEpoch {
                artifact_id: "nav-epoch-invalid".to_string(),
                source_observation_epoch_id: "obs-epoch-invalid".to_string(),
                epoch_index: 9,
                receive_time_s: 100_000.0,
                truth_ecef_m: SyntheticPvtTruthTableEcef { x_m: 0.0, y_m: 0.0, z_m: 0.0 },
                measured_ecef_m: SyntheticPvtTruthTableEcef { x_m: 0.0, y_m: 0.0, z_m: 6.0 },
                ecef_error_m: SyntheticPvtTruthTableEcef { x_m: 0.0, y_m: 0.0, z_m: 6.0 },
                truth_geodetic: SyntheticPvtTruthTableGeodetic {
                    latitude_deg: 0.0,
                    longitude_deg: 0.0,
                    altitude_m: 0.0,
                },
                measured_geodetic: SyntheticPvtTruthTableGeodetic {
                    latitude_deg: 0.0,
                    longitude_deg: 0.0,
                    altitude_m: 6.0,
                },
                enu_error_m: SyntheticPvtTruthTableEnuError {
                    east_m: 0.0,
                    north_m: 0.0,
                    up_m: 6.0,
                    horiz_m: 0.0,
                    vert_m: 6.0,
                    error_3d_m: 6.0,
                },
                clock_bias: SyntheticPvtTruthTableClockBias {
                    truth_s: 0.0,
                    measured_s: 1.0e-6,
                    error_s: 1.0e-6,
                    truth_m: 0.0,
                    measured_m: 10.0,
                    error_m: 10.0,
                },
                residual_rms_m: 2.0,
                pre_fit_residual_rms_m: Some(2.0),
                post_fit_residual_rms_m: Some(2.0),
                dop: SyntheticPvtTruthTableDop {
                    pdop: 4.0,
                    hdop: Some(2.0),
                    vdop: Some(2.0),
                    gdop: Some(4.5),
                    tdop: Some(1.0),
                },
                solution_status: SolutionStatus::Invalid,
                solution_quality: NavQualityFlag::NoFix,
                solution_validity: SolutionValidity::Invalid,
                valid: false,
                sat_count: 4,
                used_sat_count: 4,
                rejected_sat_count: 0,
            }],
        };

        let accuracy =
            validate_pvt_accuracy_budget(&report, truth_guided_receiver_accuracy_budgets().pvt);
        let epoch = accuracy.epochs.first().expect("pvt epoch");

        assert!(!accuracy.pass);
        assert_eq!(accuracy.passing_epoch_count, 0);
        assert!(!epoch.pass);
    }

    #[test]
    fn pvt_cn0_profile_summarizes_observation_levels_and_epoch_quality() {
        let weak_observations =
            vec![sample_obs_epoch(4, &[24.0, 26.0]), sample_obs_epoch(5, &[28.0, 30.0])];
        let strong_observations =
            vec![sample_obs_epoch(6, &[40.0, 42.0]), sample_obs_epoch(7, &[44.0, 46.0])];
        let weak_accuracy = SyntheticPvtAccuracyReport {
            scenario_id: "pvt_cn0_profile_weak".to_string(),
            max_position_error_3d_m: 5.0,
            max_clock_bias_error_m: 10.0,
            max_residual_rms_m: 4.0,
            max_pdop: 6.0,
            epoch_count: 2,
            passing_epoch_count: 1,
            truth_coverage_ready: true,
            truth_coverage_issues: Vec::new(),
            pass: false,
            epochs: vec![
                SyntheticPvtAccuracyEpoch {
                    epoch_index: 4,
                    position_error_3d_m: 2.0,
                    clock_bias_error_m: 1.0,
                    residual_rms_m: 0.5,
                    pdop: 2.0,
                    pass: true,
                },
                SyntheticPvtAccuracyEpoch {
                    epoch_index: 5,
                    position_error_3d_m: 4.0,
                    clock_bias_error_m: 3.0,
                    residual_rms_m: 1.5,
                    pdop: 3.0,
                    pass: false,
                },
            ],
        };
        let strong_accuracy = SyntheticPvtAccuracyReport {
            scenario_id: "pvt_cn0_profile_strong".to_string(),
            max_position_error_3d_m: 5.0,
            max_clock_bias_error_m: 10.0,
            max_residual_rms_m: 4.0,
            max_pdop: 6.0,
            epoch_count: 2,
            passing_epoch_count: 2,
            truth_coverage_ready: true,
            truth_coverage_issues: Vec::new(),
            pass: true,
            epochs: vec![
                SyntheticPvtAccuracyEpoch {
                    epoch_index: 6,
                    position_error_3d_m: 1.0,
                    clock_bias_error_m: 0.5,
                    residual_rms_m: 0.25,
                    pdop: 1.5,
                    pass: true,
                },
                SyntheticPvtAccuracyEpoch {
                    epoch_index: 7,
                    position_error_3d_m: 1.5,
                    clock_bias_error_m: 0.75,
                    residual_rms_m: 0.5,
                    pdop: 1.75,
                    pass: true,
                },
            ],
        };

        let report = summarize_truth_guided_pvt_cn0_profile(
            &[
                SyntheticPvtCn0ProfileCase {
                    scenario_id: "pvt_cn0_profile_strong",
                    observations: &strong_observations,
                    accuracy: &strong_accuracy,
                },
                SyntheticPvtCn0ProfileCase {
                    scenario_id: "pvt_cn0_profile_weak",
                    observations: &weak_observations,
                    accuracy: &weak_accuracy,
                },
            ],
            "pvt_cn0_profile",
        );

        assert_eq!(report.points.len(), 2);
        assert_eq!(report.points[0].scenario_id, "pvt_cn0_profile_weak");
        assert!((report.points[0].mean_observation_cn0_dbhz - 27.0).abs() <= 1.0e-12);
        assert_eq!(report.points[0].min_observation_cn0_dbhz, 25.0);
        assert_eq!(report.points[0].max_observation_cn0_dbhz, 29.0);
        assert_eq!(report.points[0].passing_epoch_count, 1);
        assert!((report.points[0].pass_rate - 0.5).abs() <= 1.0e-12);
        assert_eq!(report.points[0].max_position_error_3d_m, Some(4.0));
        assert!(report.points[0].ready);
        assert_eq!(report.points[1].scenario_id, "pvt_cn0_profile_strong");
        assert!((report.points[1].mean_observation_cn0_dbhz - 43.0).abs() <= 1.0e-12);
        assert_eq!(report.points[1].passing_epoch_count, 2);
        assert!((report.points[1].pass_rate - 1.0).abs() <= 1.0e-12);
        assert_eq!(report.points[1].max_position_error_3d_m, Some(1.5));
    }

    #[test]
    fn accuracy_cn0_profile_merges_stage_metrics_by_signal_level() {
        let acquisition = SyntheticAcquisitionDetectionRateReport {
            scenario_id_prefix: "accuracy_cn0_profile".to_string(),
            code_phase_tolerance_samples: 2,
            doppler_tolerance_bins: 1,
            doppler_step_hz: 500,
            points: vec![
                SyntheticAcquisitionDetectionRatePoint {
                    sat: SatId { constellation: Constellation::Gps, prn: 7 },
                    cn0_db_hz: 27.0,
                    doppler_hz: 250.0,
                    coherent_ms: 1,
                    noncoherent: 1,
                    trial_count: 4,
                    accepted_count: 2,
                    detected_count: 1,
                    acceptance_probability: 0.5,
                    detection_probability: 0.25,
                    mean_peak_mean_ratio: 3.0,
                },
                SyntheticAcquisitionDetectionRatePoint {
                    sat: SatId { constellation: Constellation::Gps, prn: 7 },
                    cn0_db_hz: 43.0,
                    doppler_hz: 250.0,
                    coherent_ms: 1,
                    noncoherent: 1,
                    trial_count: 4,
                    accepted_count: 4,
                    detected_count: 4,
                    acceptance_probability: 1.0,
                    detection_probability: 1.0,
                    mean_peak_mean_ratio: 12.0,
                },
            ],
        };
        let tracking = SyntheticTrackingLockRateReport {
            scenario_id_prefix: "accuracy_cn0_profile".to_string(),
            points: vec![
                SyntheticTrackingLockRatePoint {
                    sat: SatId { constellation: Constellation::Gps, prn: 7 },
                    cn0_db_hz: 27.0,
                    duration_s: 0.08,
                    seeded_doppler_error_hz: 60.0,
                    seeded_code_phase_error_samples: 1,
                    min_locked_epochs: 4,
                    trial_count: 4,
                    stable_lock_count: 1,
                    refused_lock_count: 2,
                    lock_probability: 0.25,
                    mean_locked_epochs: 2.0,
                },
                SyntheticTrackingLockRatePoint {
                    sat: SatId { constellation: Constellation::Gps, prn: 7 },
                    cn0_db_hz: 43.0,
                    duration_s: 0.08,
                    seeded_doppler_error_hz: 60.0,
                    seeded_code_phase_error_samples: 1,
                    min_locked_epochs: 4,
                    trial_count: 4,
                    stable_lock_count: 4,
                    refused_lock_count: 0,
                    lock_probability: 1.0,
                    mean_locked_epochs: 8.0,
                },
            ],
        };
        let pvt = SyntheticPvtCn0ProfileReport {
            scenario_id_prefix: "accuracy_cn0_profile".to_string(),
            points: vec![
                SyntheticPvtCn0ProfilePoint {
                    scenario_id: "pvt_weak".to_string(),
                    observation_epoch_count: 2,
                    mean_observation_cn0_dbhz: 27.0,
                    min_observation_cn0_dbhz: 25.0,
                    max_observation_cn0_dbhz: 29.0,
                    epoch_count: 2,
                    passing_epoch_count: 1,
                    pass_rate: 0.5,
                    rms_position_error_3d_m: Some(3.0),
                    max_position_error_3d_m: Some(4.0),
                    rms_clock_bias_error_m: Some(2.0),
                    max_clock_bias_error_m: Some(3.0),
                    rms_residual_rms_m: Some(1.0),
                    max_residual_rms_m: Some(1.5),
                    max_pdop: Some(3.0),
                    truth_coverage_ready: true,
                    truth_coverage_issues: Vec::new(),
                    ready: true,
                },
                SyntheticPvtCn0ProfilePoint {
                    scenario_id: "pvt_strong".to_string(),
                    observation_epoch_count: 2,
                    mean_observation_cn0_dbhz: 43.0,
                    min_observation_cn0_dbhz: 41.0,
                    max_observation_cn0_dbhz: 45.0,
                    epoch_count: 2,
                    passing_epoch_count: 2,
                    pass_rate: 1.0,
                    rms_position_error_3d_m: Some(1.25),
                    max_position_error_3d_m: Some(1.5),
                    rms_clock_bias_error_m: Some(0.625),
                    max_clock_bias_error_m: Some(0.75),
                    rms_residual_rms_m: Some(0.375),
                    max_residual_rms_m: Some(0.5),
                    max_pdop: Some(1.75),
                    truth_coverage_ready: true,
                    truth_coverage_issues: Vec::new(),
                    ready: true,
                },
            ],
        };

        let report: SyntheticAccuracyCn0ProfileReport = summarize_truth_guided_accuracy_cn0_profile(
            "accuracy_cn0_profile",
            &acquisition,
            &tracking,
            &pvt,
        );

        assert_eq!(report.points.len(), 2);
        assert_eq!(report.points[0].cn0_db_hz, 27.0);
        assert_eq!(report.points[0].acquisition_case_count, 1);
        assert_eq!(report.points[0].tracking_case_count, 1);
        assert_eq!(report.points[0].pvt_case_count, 1);
        assert_eq!(report.points[0].acquisition_detection_probability_mean, Some(0.25));
        assert_eq!(report.points[0].tracking_lock_probability_mean, Some(0.25));
        assert_eq!(report.points[0].pvt_pass_rate_mean, Some(0.5));
        assert_eq!(report.points[1].cn0_db_hz, 43.0);
        assert_eq!(report.points[1].acquisition_detection_probability_mean, Some(1.0));
        assert_eq!(report.points[1].tracking_lock_probability_mean, Some(1.0));
        assert_eq!(report.points[1].pvt_pass_rate_mean, Some(1.0));
        assert_eq!(report.points[1].pvt_max_position_error_3d_m_max, Some(1.5));
    }

    #[test]
    fn pvt_geometry_profile_sorts_cases_by_mean_pdop() {
        let poor_accuracy = SyntheticPvtAccuracyReport {
            scenario_id: "pvt_geometry_profile_poor".to_string(),
            max_position_error_3d_m: 5.0,
            max_clock_bias_error_m: 10.0,
            max_residual_rms_m: 4.0,
            max_pdop: 6.0,
            epoch_count: 2,
            passing_epoch_count: 1,
            truth_coverage_ready: true,
            truth_coverage_issues: Vec::new(),
            pass: false,
            epochs: vec![
                SyntheticPvtAccuracyEpoch {
                    epoch_index: 1,
                    position_error_3d_m: 3.5,
                    clock_bias_error_m: 1.75,
                    residual_rms_m: 1.2,
                    pdop: 4.0,
                    pass: false,
                },
                SyntheticPvtAccuracyEpoch {
                    epoch_index: 2,
                    position_error_3d_m: 4.5,
                    clock_bias_error_m: 2.25,
                    residual_rms_m: 1.5,
                    pdop: 5.5,
                    pass: true,
                },
            ],
        };
        let good_accuracy = SyntheticPvtAccuracyReport {
            scenario_id: "pvt_geometry_profile_good".to_string(),
            max_position_error_3d_m: 5.0,
            max_clock_bias_error_m: 10.0,
            max_residual_rms_m: 4.0,
            max_pdop: 6.0,
            epoch_count: 2,
            passing_epoch_count: 2,
            truth_coverage_ready: true,
            truth_coverage_issues: Vec::new(),
            pass: true,
            epochs: vec![
                SyntheticPvtAccuracyEpoch {
                    epoch_index: 3,
                    position_error_3d_m: 0.9,
                    clock_bias_error_m: 0.4,
                    residual_rms_m: 0.2,
                    pdop: 1.4,
                    pass: true,
                },
                SyntheticPvtAccuracyEpoch {
                    epoch_index: 4,
                    position_error_3d_m: 1.2,
                    clock_bias_error_m: 0.5,
                    residual_rms_m: 0.25,
                    pdop: 1.8,
                    pass: true,
                },
            ],
        };

        let report: SyntheticPvtGeometryProfileReport = summarize_truth_guided_pvt_geometry_profile(
            &[
                SyntheticPvtGeometryProfileCase {
                    scenario_id: "pvt_geometry_profile_poor",
                    accuracy: &poor_accuracy,
                },
                SyntheticPvtGeometryProfileCase {
                    scenario_id: "pvt_geometry_profile_good",
                    accuracy: &good_accuracy,
                },
            ],
            "pvt_geometry_profile",
        );

        assert_eq!(report.points.len(), 2);
        assert_eq!(report.points[0].scenario_id, "pvt_geometry_profile_good");
        assert_eq!(report.points[0].mean_pdop, Some(1.6));
        assert_eq!(report.points[0].max_pdop, Some(1.8));
        assert_eq!(report.points[0].passing_epoch_count, 2);
        assert_eq!(report.points[1].scenario_id, "pvt_geometry_profile_poor");
        assert_eq!(report.points[1].mean_pdop, Some(4.75));
        assert_eq!(report.points[1].max_pdop, Some(5.5));
        assert_eq!(report.points[1].pass_rate, 0.5);
        assert!(report.points[1].ready);
    }

    #[test]
    fn pvt_multipath_profile_sorts_cases_by_injected_bias_and_validity() {
        let clean_truth = SyntheticPvtTruthTableReport {
            scenario_id: "pvt_multipath_profile_clean".to_string(),
            solution_count: 1,
            matched_epoch_count: 1,
            unmatched_solution_epochs: Vec::new(),
            unused_reference_epochs: Vec::new(),
            epochs: vec![SyntheticPvtTruthTableEpoch {
                artifact_id: "clean-artifact".to_string(),
                source_observation_epoch_id: "clean-source".to_string(),
                epoch_index: 1,
                receive_time_s: 1.0,
                truth_ecef_m: SyntheticPvtTruthTableEcef { x_m: 0.0, y_m: 0.0, z_m: 0.0 },
                measured_ecef_m: SyntheticPvtTruthTableEcef { x_m: 0.5, y_m: 0.0, z_m: 0.0 },
                ecef_error_m: SyntheticPvtTruthTableEcef { x_m: 0.5, y_m: 0.0, z_m: 0.0 },
                truth_geodetic: SyntheticPvtTruthTableGeodetic {
                    latitude_deg: 0.0,
                    longitude_deg: 0.0,
                    altitude_m: 0.0,
                },
                measured_geodetic: SyntheticPvtTruthTableGeodetic {
                    latitude_deg: 0.0,
                    longitude_deg: 0.0,
                    altitude_m: 0.5,
                },
                enu_error_m: SyntheticPvtTruthTableEnuError {
                    east_m: 0.5,
                    north_m: 0.0,
                    up_m: 0.0,
                    horiz_m: 0.5,
                    vert_m: 0.0,
                    error_3d_m: 0.5,
                },
                clock_bias: SyntheticPvtTruthTableClockBias {
                    truth_s: 0.0,
                    measured_s: 0.0,
                    error_s: 0.0,
                    truth_m: 0.0,
                    measured_m: 0.0,
                    error_m: 0.0,
                },
                residual_rms_m: 0.75,
                pre_fit_residual_rms_m: Some(0.75),
                post_fit_residual_rms_m: Some(0.75),
                dop: SyntheticPvtTruthTableDop {
                    pdop: 1.5,
                    hdop: Some(1.0),
                    vdop: Some(1.0),
                    gdop: Some(1.8),
                    tdop: Some(0.6),
                },
                solution_status: SolutionStatus::Converged,
                solution_quality: NavQualityFlag::Float,
                solution_validity: SolutionValidity::Stable,
                valid: true,
                sat_count: 5,
                used_sat_count: 5,
                rejected_sat_count: 0,
            }],
        };
        let severe_truth = SyntheticPvtTruthTableReport {
            scenario_id: "pvt_multipath_profile_severe".to_string(),
            solution_count: 1,
            matched_epoch_count: 1,
            unmatched_solution_epochs: Vec::new(),
            unused_reference_epochs: Vec::new(),
            epochs: vec![SyntheticPvtTruthTableEpoch {
                artifact_id: "severe-artifact".to_string(),
                source_observation_epoch_id: "severe-source".to_string(),
                epoch_index: 2,
                receive_time_s: 2.0,
                truth_ecef_m: SyntheticPvtTruthTableEcef { x_m: 0.0, y_m: 0.0, z_m: 0.0 },
                measured_ecef_m: SyntheticPvtTruthTableEcef { x_m: 20.0, y_m: 0.0, z_m: 0.0 },
                ecef_error_m: SyntheticPvtTruthTableEcef { x_m: 20.0, y_m: 0.0, z_m: 0.0 },
                truth_geodetic: SyntheticPvtTruthTableGeodetic {
                    latitude_deg: 0.0,
                    longitude_deg: 0.0,
                    altitude_m: 0.0,
                },
                measured_geodetic: SyntheticPvtTruthTableGeodetic {
                    latitude_deg: 0.0,
                    longitude_deg: 0.0,
                    altitude_m: 20.0,
                },
                enu_error_m: SyntheticPvtTruthTableEnuError {
                    east_m: 20.0,
                    north_m: 0.0,
                    up_m: 0.0,
                    horiz_m: 20.0,
                    vert_m: 0.0,
                    error_3d_m: 20.0,
                },
                clock_bias: SyntheticPvtTruthTableClockBias {
                    truth_s: 0.0,
                    measured_s: 0.0,
                    error_s: 0.0,
                    truth_m: 0.0,
                    measured_m: 0.0,
                    error_m: 0.0,
                },
                residual_rms_m: 60.0,
                pre_fit_residual_rms_m: Some(60.0),
                post_fit_residual_rms_m: Some(60.0),
                dop: SyntheticPvtTruthTableDop {
                    pdop: 1.5,
                    hdop: Some(1.0),
                    vdop: Some(1.0),
                    gdop: Some(1.8),
                    tdop: Some(0.6),
                },
                solution_status: SolutionStatus::Degraded,
                solution_quality: NavQualityFlag::Degraded,
                solution_validity: SolutionValidity::Diverging,
                valid: true,
                sat_count: 5,
                used_sat_count: 5,
                rejected_sat_count: 0,
            }],
        };
        let clean_accuracy = SyntheticPvtAccuracyReport {
            scenario_id: "pvt_multipath_profile_clean".to_string(),
            max_position_error_3d_m: 5.0,
            max_clock_bias_error_m: 10.0,
            max_residual_rms_m: 4.0,
            max_pdop: 6.0,
            epoch_count: 1,
            passing_epoch_count: 1,
            truth_coverage_ready: true,
            truth_coverage_issues: Vec::new(),
            pass: true,
            epochs: vec![SyntheticPvtAccuracyEpoch {
                epoch_index: 1,
                position_error_3d_m: 0.5,
                clock_bias_error_m: 0.0,
                residual_rms_m: 0.75,
                pdop: 1.5,
                pass: true,
            }],
        };
        let severe_accuracy = SyntheticPvtAccuracyReport {
            scenario_id: "pvt_multipath_profile_severe".to_string(),
            max_position_error_3d_m: 5.0,
            max_clock_bias_error_m: 10.0,
            max_residual_rms_m: 4.0,
            max_pdop: 6.0,
            epoch_count: 1,
            passing_epoch_count: 0,
            truth_coverage_ready: true,
            truth_coverage_issues: Vec::new(),
            pass: false,
            epochs: vec![SyntheticPvtAccuracyEpoch {
                epoch_index: 2,
                position_error_3d_m: 20.0,
                clock_bias_error_m: 0.0,
                residual_rms_m: 60.0,
                pdop: 1.5,
                pass: false,
            }],
        };

        let report: SyntheticPvtMultipathProfileReport =
            summarize_truth_guided_pvt_multipath_profile(
                &[
                    SyntheticPvtMultipathProfileCase {
                        scenario_id: "pvt_multipath_profile_severe",
                        affected_satellite_count: 4,
                        mean_abs_pseudorange_bias_m: 22.0,
                        max_abs_pseudorange_bias_m: 35.0,
                        truth_table: &severe_truth,
                        accuracy: &severe_accuracy,
                    },
                    SyntheticPvtMultipathProfileCase {
                        scenario_id: "pvt_multipath_profile_clean",
                        affected_satellite_count: 0,
                        mean_abs_pseudorange_bias_m: 0.0,
                        max_abs_pseudorange_bias_m: 0.0,
                        truth_table: &clean_truth,
                        accuracy: &clean_accuracy,
                    },
                ],
                "pvt_multipath_profile",
            );

        assert_eq!(report.points.len(), 2);
        assert_eq!(report.points[0].scenario_id, "pvt_multipath_profile_clean");
        assert_eq!(report.points[0].stable_epoch_count, 1);
        assert_eq!(report.points[0].diverging_epoch_count, 0);
        assert_eq!(report.points[0].pass_rate, 1.0);
        assert_eq!(report.points[1].scenario_id, "pvt_multipath_profile_severe");
        assert_eq!(report.points[1].max_abs_pseudorange_bias_m, 35.0);
        assert_eq!(report.points[1].stable_epoch_count, 0);
        assert_eq!(report.points[1].diverging_epoch_count, 1);
        assert_eq!(report.points[1].max_residual_rms_m, Some(60.0));
        assert!(report.points[1].ready);
    }

    #[test]
    fn pvt_motion_profile_sorts_cases_by_truth_path_length() {
        let static_truth = SyntheticPvtTruthTableReport {
            scenario_id: "pvt_motion_profile_static".to_string(),
            solution_count: 2,
            matched_epoch_count: 2,
            unmatched_solution_epochs: Vec::new(),
            unused_reference_epochs: Vec::new(),
            epochs: vec![
                SyntheticPvtTruthTableEpoch {
                    artifact_id: "static-artifact-1".to_string(),
                    source_observation_epoch_id: "static-source-1".to_string(),
                    epoch_index: 1,
                    receive_time_s: 100.0,
                    truth_ecef_m: SyntheticPvtTruthTableEcef { x_m: 0.0, y_m: 0.0, z_m: 0.0 },
                    measured_ecef_m: SyntheticPvtTruthTableEcef { x_m: 0.2, y_m: 0.0, z_m: 0.0 },
                    ecef_error_m: SyntheticPvtTruthTableEcef { x_m: 0.2, y_m: 0.0, z_m: 0.0 },
                    truth_geodetic: SyntheticPvtTruthTableGeodetic {
                        latitude_deg: 0.0,
                        longitude_deg: 0.0,
                        altitude_m: 0.0,
                    },
                    measured_geodetic: SyntheticPvtTruthTableGeodetic {
                        latitude_deg: 0.0,
                        longitude_deg: 0.0,
                        altitude_m: 0.2,
                    },
                    enu_error_m: SyntheticPvtTruthTableEnuError {
                        east_m: 0.2,
                        north_m: 0.0,
                        up_m: 0.0,
                        horiz_m: 0.2,
                        vert_m: 0.0,
                        error_3d_m: 0.2,
                    },
                    clock_bias: SyntheticPvtTruthTableClockBias {
                        truth_s: 0.0,
                        measured_s: 0.0,
                        error_s: 0.0,
                        truth_m: 0.0,
                        measured_m: 0.0,
                        error_m: 0.0,
                    },
                    residual_rms_m: 0.4,
                    pre_fit_residual_rms_m: Some(0.4),
                    post_fit_residual_rms_m: Some(0.4),
                    dop: SyntheticPvtTruthTableDop {
                        pdop: 1.4,
                        hdop: Some(1.0),
                        vdop: Some(0.9),
                        gdop: Some(1.6),
                        tdop: Some(0.5),
                    },
                    solution_status: SolutionStatus::Converged,
                    solution_quality: NavQualityFlag::Float,
                    solution_validity: SolutionValidity::Stable,
                    valid: true,
                    sat_count: 5,
                    used_sat_count: 5,
                    rejected_sat_count: 0,
                },
                SyntheticPvtTruthTableEpoch {
                    artifact_id: "static-artifact-2".to_string(),
                    source_observation_epoch_id: "static-source-2".to_string(),
                    epoch_index: 2,
                    receive_time_s: 101.0,
                    truth_ecef_m: SyntheticPvtTruthTableEcef { x_m: 0.0, y_m: 0.0, z_m: 0.0 },
                    measured_ecef_m: SyntheticPvtTruthTableEcef { x_m: 0.3, y_m: 0.0, z_m: 0.0 },
                    ecef_error_m: SyntheticPvtTruthTableEcef { x_m: 0.3, y_m: 0.0, z_m: 0.0 },
                    truth_geodetic: SyntheticPvtTruthTableGeodetic {
                        latitude_deg: 0.0,
                        longitude_deg: 0.0,
                        altitude_m: 0.0,
                    },
                    measured_geodetic: SyntheticPvtTruthTableGeodetic {
                        latitude_deg: 0.0,
                        longitude_deg: 0.0,
                        altitude_m: 0.3,
                    },
                    enu_error_m: SyntheticPvtTruthTableEnuError {
                        east_m: 0.3,
                        north_m: 0.0,
                        up_m: 0.0,
                        horiz_m: 0.3,
                        vert_m: 0.0,
                        error_3d_m: 0.3,
                    },
                    clock_bias: SyntheticPvtTruthTableClockBias {
                        truth_s: 0.0,
                        measured_s: 0.0,
                        error_s: 0.0,
                        truth_m: 0.0,
                        measured_m: 0.0,
                        error_m: 0.0,
                    },
                    residual_rms_m: 0.45,
                    pre_fit_residual_rms_m: Some(0.45),
                    post_fit_residual_rms_m: Some(0.45),
                    dop: SyntheticPvtTruthTableDop {
                        pdop: 1.4,
                        hdop: Some(1.0),
                        vdop: Some(0.9),
                        gdop: Some(1.6),
                        tdop: Some(0.5),
                    },
                    solution_status: SolutionStatus::Converged,
                    solution_quality: NavQualityFlag::Float,
                    solution_validity: SolutionValidity::Stable,
                    valid: true,
                    sat_count: 5,
                    used_sat_count: 5,
                    rejected_sat_count: 0,
                },
            ],
        };
        let moving_truth = SyntheticPvtTruthTableReport {
            scenario_id: "pvt_motion_profile_linear_motion".to_string(),
            solution_count: 2,
            matched_epoch_count: 2,
            unmatched_solution_epochs: Vec::new(),
            unused_reference_epochs: Vec::new(),
            epochs: vec![
                SyntheticPvtTruthTableEpoch {
                    artifact_id: "moving-artifact-1".to_string(),
                    source_observation_epoch_id: "moving-source-1".to_string(),
                    epoch_index: 1,
                    receive_time_s: 100.0,
                    truth_ecef_m: SyntheticPvtTruthTableEcef { x_m: 0.0, y_m: 0.0, z_m: 0.0 },
                    measured_ecef_m: SyntheticPvtTruthTableEcef { x_m: 0.2, y_m: 0.0, z_m: 0.0 },
                    ecef_error_m: SyntheticPvtTruthTableEcef { x_m: 0.2, y_m: 0.0, z_m: 0.0 },
                    truth_geodetic: SyntheticPvtTruthTableGeodetic {
                        latitude_deg: 0.0,
                        longitude_deg: 0.0,
                        altitude_m: 0.0,
                    },
                    measured_geodetic: SyntheticPvtTruthTableGeodetic {
                        latitude_deg: 0.0,
                        longitude_deg: 0.0,
                        altitude_m: 0.2,
                    },
                    enu_error_m: SyntheticPvtTruthTableEnuError {
                        east_m: 0.2,
                        north_m: 0.0,
                        up_m: 0.0,
                        horiz_m: 0.2,
                        vert_m: 0.0,
                        error_3d_m: 0.2,
                    },
                    clock_bias: SyntheticPvtTruthTableClockBias {
                        truth_s: 0.0,
                        measured_s: 0.0,
                        error_s: 0.0,
                        truth_m: 0.0,
                        measured_m: 0.0,
                        error_m: 0.0,
                    },
                    residual_rms_m: 0.4,
                    pre_fit_residual_rms_m: Some(0.4),
                    post_fit_residual_rms_m: Some(0.4),
                    dop: SyntheticPvtTruthTableDop {
                        pdop: 1.5,
                        hdop: Some(1.1),
                        vdop: Some(0.9),
                        gdop: Some(1.7),
                        tdop: Some(0.5),
                    },
                    solution_status: SolutionStatus::Converged,
                    solution_quality: NavQualityFlag::Float,
                    solution_validity: SolutionValidity::Stable,
                    valid: true,
                    sat_count: 5,
                    used_sat_count: 5,
                    rejected_sat_count: 0,
                },
                SyntheticPvtTruthTableEpoch {
                    artifact_id: "moving-artifact-2".to_string(),
                    source_observation_epoch_id: "moving-source-2".to_string(),
                    epoch_index: 2,
                    receive_time_s: 101.0,
                    truth_ecef_m: SyntheticPvtTruthTableEcef { x_m: 10.0, y_m: 0.0, z_m: 0.0 },
                    measured_ecef_m: SyntheticPvtTruthTableEcef { x_m: 10.5, y_m: 0.0, z_m: 0.0 },
                    ecef_error_m: SyntheticPvtTruthTableEcef { x_m: 0.5, y_m: 0.0, z_m: 0.0 },
                    truth_geodetic: SyntheticPvtTruthTableGeodetic {
                        latitude_deg: 0.0,
                        longitude_deg: 0.0,
                        altitude_m: 0.0,
                    },
                    measured_geodetic: SyntheticPvtTruthTableGeodetic {
                        latitude_deg: 0.0,
                        longitude_deg: 0.0,
                        altitude_m: 0.5,
                    },
                    enu_error_m: SyntheticPvtTruthTableEnuError {
                        east_m: 0.5,
                        north_m: 0.0,
                        up_m: 0.0,
                        horiz_m: 0.5,
                        vert_m: 0.0,
                        error_3d_m: 0.5,
                    },
                    clock_bias: SyntheticPvtTruthTableClockBias {
                        truth_s: 0.0,
                        measured_s: 0.0,
                        error_s: 0.0,
                        truth_m: 0.0,
                        measured_m: 0.0,
                        error_m: 0.0,
                    },
                    residual_rms_m: 0.5,
                    pre_fit_residual_rms_m: Some(0.5),
                    post_fit_residual_rms_m: Some(0.5),
                    dop: SyntheticPvtTruthTableDop {
                        pdop: 1.5,
                        hdop: Some(1.1),
                        vdop: Some(0.9),
                        gdop: Some(1.7),
                        tdop: Some(0.5),
                    },
                    solution_status: SolutionStatus::Converged,
                    solution_quality: NavQualityFlag::Float,
                    solution_validity: SolutionValidity::Stable,
                    valid: true,
                    sat_count: 5,
                    used_sat_count: 5,
                    rejected_sat_count: 0,
                },
            ],
        };
        let static_accuracy = SyntheticPvtAccuracyReport {
            scenario_id: "pvt_motion_profile_static".to_string(),
            max_position_error_3d_m: 5.0,
            max_clock_bias_error_m: 10.0,
            max_residual_rms_m: 4.0,
            max_pdop: 6.0,
            epoch_count: 2,
            passing_epoch_count: 2,
            truth_coverage_ready: true,
            truth_coverage_issues: Vec::new(),
            pass: true,
            epochs: vec![
                SyntheticPvtAccuracyEpoch {
                    epoch_index: 1,
                    position_error_3d_m: 0.2,
                    clock_bias_error_m: 0.0,
                    residual_rms_m: 0.4,
                    pdop: 1.4,
                    pass: true,
                },
                SyntheticPvtAccuracyEpoch {
                    epoch_index: 2,
                    position_error_3d_m: 0.3,
                    clock_bias_error_m: 0.0,
                    residual_rms_m: 0.45,
                    pdop: 1.4,
                    pass: true,
                },
            ],
        };
        let moving_accuracy = SyntheticPvtAccuracyReport {
            scenario_id: "pvt_motion_profile_linear_motion".to_string(),
            max_position_error_3d_m: 5.0,
            max_clock_bias_error_m: 10.0,
            max_residual_rms_m: 4.0,
            max_pdop: 6.0,
            epoch_count: 2,
            passing_epoch_count: 2,
            truth_coverage_ready: true,
            truth_coverage_issues: Vec::new(),
            pass: true,
            epochs: vec![
                SyntheticPvtAccuracyEpoch {
                    epoch_index: 1,
                    position_error_3d_m: 0.2,
                    clock_bias_error_m: 0.0,
                    residual_rms_m: 0.4,
                    pdop: 1.5,
                    pass: true,
                },
                SyntheticPvtAccuracyEpoch {
                    epoch_index: 2,
                    position_error_3d_m: 0.5,
                    clock_bias_error_m: 0.0,
                    residual_rms_m: 0.5,
                    pdop: 1.5,
                    pass: true,
                },
            ],
        };

        let report: SyntheticPvtMotionProfileReport = summarize_truth_guided_pvt_motion_profile(
            &[
                SyntheticPvtMotionProfileCase {
                    scenario_id: "pvt_motion_profile_linear_motion",
                    truth_table: &moving_truth,
                    accuracy: &moving_accuracy,
                },
                SyntheticPvtMotionProfileCase {
                    scenario_id: "pvt_motion_profile_static",
                    truth_table: &static_truth,
                    accuracy: &static_accuracy,
                },
            ],
            "pvt_motion_profile",
        );

        assert_eq!(report.points.len(), 2);
        assert_eq!(report.points[0].scenario_id, "pvt_motion_profile_static");
        assert!(!report.points[0].moving);
        assert_eq!(report.points[0].path_length_m, 0.0);
        assert_eq!(report.points[1].scenario_id, "pvt_motion_profile_linear_motion");
        assert!(report.points[1].moving);
        assert_eq!(report.points[1].path_length_m, 10.0);
        assert_eq!(report.points[1].mean_speed_mps, 10.0);
        assert_eq!(report.points[1].stable_epoch_count, 2);
        assert!(report.points[1].ready);
    }

    fn time_profile_truth_and_accuracy(
        scenario_id: &str,
        position_error_3d_m: &[f64],
        residual_rms_m: &[f64],
        validities: &[SolutionValidity],
    ) -> (SyntheticPvtTruthTableReport, SyntheticPvtAccuracyReport) {
        assert_eq!(position_error_3d_m.len(), residual_rms_m.len());
        assert_eq!(position_error_3d_m.len(), validities.len());

        let truth = SyntheticPvtTruthTableReport {
            scenario_id: scenario_id.to_string(),
            solution_count: position_error_3d_m.len(),
            matched_epoch_count: position_error_3d_m.len(),
            unmatched_solution_epochs: Vec::new(),
            unused_reference_epochs: Vec::new(),
            epochs: position_error_3d_m
                .iter()
                .zip(residual_rms_m.iter())
                .zip(validities.iter())
                .enumerate()
                .map(|(index, ((position_error_3d_m, residual_rms_m), validity))| {
                    SyntheticPvtTruthTableEpoch {
                        artifact_id: format!("{scenario_id}-artifact-{index}"),
                        source_observation_epoch_id: format!("{scenario_id}-source-{index}"),
                        epoch_index: index as u64,
                        receive_time_s: 100.0 + index as f64,
                        truth_ecef_m: SyntheticPvtTruthTableEcef { x_m: 0.0, y_m: 0.0, z_m: 0.0 },
                        measured_ecef_m: SyntheticPvtTruthTableEcef {
                            x_m: *position_error_3d_m,
                            y_m: 0.0,
                            z_m: 0.0,
                        },
                        ecef_error_m: SyntheticPvtTruthTableEcef {
                            x_m: *position_error_3d_m,
                            y_m: 0.0,
                            z_m: 0.0,
                        },
                        truth_geodetic: SyntheticPvtTruthTableGeodetic {
                            latitude_deg: 0.0,
                            longitude_deg: 0.0,
                            altitude_m: 0.0,
                        },
                        measured_geodetic: SyntheticPvtTruthTableGeodetic {
                            latitude_deg: 0.0,
                            longitude_deg: 0.0,
                            altitude_m: *position_error_3d_m,
                        },
                        enu_error_m: SyntheticPvtTruthTableEnuError {
                            east_m: *position_error_3d_m,
                            north_m: 0.0,
                            up_m: 0.0,
                            horiz_m: *position_error_3d_m,
                            vert_m: 0.0,
                            error_3d_m: *position_error_3d_m,
                        },
                        clock_bias: SyntheticPvtTruthTableClockBias {
                            truth_s: 0.0,
                            measured_s: 0.0,
                            error_s: 0.0,
                            truth_m: 0.0,
                            measured_m: 0.0,
                            error_m: 0.0,
                        },
                        residual_rms_m: *residual_rms_m,
                        pre_fit_residual_rms_m: Some(*residual_rms_m),
                        post_fit_residual_rms_m: Some(*residual_rms_m),
                        dop: SyntheticPvtTruthTableDop {
                            pdop: 1.5,
                            hdop: Some(1.1),
                            vdop: Some(0.9),
                            gdop: Some(1.7),
                            tdop: Some(0.5),
                        },
                        solution_status: SolutionStatus::Converged,
                        solution_quality: NavQualityFlag::Float,
                        solution_validity: *validity,
                        valid: *validity != SolutionValidity::Diverging,
                        sat_count: 5,
                        used_sat_count: 5,
                        rejected_sat_count: 0,
                    }
                })
                .collect(),
        };
        let accuracy = SyntheticPvtAccuracyReport {
            scenario_id: scenario_id.to_string(),
            max_position_error_3d_m: 50.0,
            max_clock_bias_error_m: 10.0,
            max_residual_rms_m: 50.0,
            max_pdop: 6.0,
            epoch_count: position_error_3d_m.len(),
            passing_epoch_count: position_error_3d_m.len(),
            truth_coverage_ready: true,
            truth_coverage_issues: Vec::new(),
            pass: true,
            epochs: position_error_3d_m
                .iter()
                .zip(residual_rms_m.iter())
                .enumerate()
                .map(|(index, (position_error_3d_m, residual_rms_m))| SyntheticPvtAccuracyEpoch {
                    epoch_index: index as u64,
                    position_error_3d_m: *position_error_3d_m,
                    clock_bias_error_m: 0.0,
                    residual_rms_m: *residual_rms_m,
                    pdop: 1.5,
                    pass: true,
                })
                .collect(),
        };

        (truth, accuracy)
    }

    #[test]
    fn pvt_time_profile_classifies_stabilizing_drifting_and_diverging_runs() {
        let (stabilizing_truth, stabilizing_accuracy) = time_profile_truth_and_accuracy(
            "pvt_time_profile_stabilizing",
            &[5.0, 3.0, 1.5, 0.9],
            &[4.0, 2.5, 1.5, 0.8],
            &[
                SolutionValidity::Coarse,
                SolutionValidity::Converging,
                SolutionValidity::Stable,
                SolutionValidity::Stable,
            ],
        );
        let (drifting_truth, drifting_accuracy) = time_profile_truth_and_accuracy(
            "pvt_time_profile_drifting",
            &[0.4, 0.8, 1.2, 1.8],
            &[0.3, 0.6, 0.9, 1.3],
            &[SolutionValidity::Stable; 4],
        );
        let (diverging_truth, diverging_accuracy) = time_profile_truth_and_accuracy(
            "pvt_time_profile_diverging",
            &[1.0, 4.0, 12.0, 40.0],
            &[1.0, 4.5, 15.0, 45.0],
            &[
                SolutionValidity::Stable,
                SolutionValidity::Converging,
                SolutionValidity::Coarse,
                SolutionValidity::Diverging,
            ],
        );

        let report: SyntheticPvtTimeProfileReport = summarize_truth_guided_pvt_time_profile(
            &[
                SyntheticPvtTimeProfileCase {
                    scenario_id: "pvt_time_profile_diverging",
                    truth_table: &diverging_truth,
                    accuracy: &diverging_accuracy,
                },
                SyntheticPvtTimeProfileCase {
                    scenario_id: "pvt_time_profile_stabilizing",
                    truth_table: &stabilizing_truth,
                    accuracy: &stabilizing_accuracy,
                },
                SyntheticPvtTimeProfileCase {
                    scenario_id: "pvt_time_profile_drifting",
                    truth_table: &drifting_truth,
                    accuracy: &drifting_accuracy,
                },
            ],
            "pvt_time_profile",
        );

        assert_eq!(report.points.len(), 3);
        assert_eq!(report.points[0].duration_s, 3.0);
        let stabilizing = report
            .points
            .iter()
            .find(|point| point.scenario_id == "pvt_time_profile_stabilizing")
            .expect("stabilizing time profile point");
        let drifting = report
            .points
            .iter()
            .find(|point| point.scenario_id == "pvt_time_profile_drifting")
            .expect("drifting time profile point");
        let diverging = report
            .points
            .iter()
            .find(|point| point.scenario_id == "pvt_time_profile_diverging")
            .expect("diverging time profile point");

        assert_eq!(stabilizing.trend, SyntheticPvtTimeTrend::Stabilizing);
        assert_eq!(drifting.trend, SyntheticPvtTimeTrend::Drifting);
        assert_eq!(diverging.trend, SyntheticPvtTimeTrend::Diverging);
        assert!(stabilizing.ready && drifting.ready && diverging.ready, "{report:?}");
        assert!(stabilizing.position_error_drift_m_per_s.expect("stabilizing slope") < 0.0);
        assert!(drifting.position_error_drift_m_per_s.expect("drifting slope") > 0.0);
        assert!(diverging.diverging_epoch_count > 0);
    }

    fn sample_obs_epoch(epoch_idx: u64, cn0_values_dbhz: &[f64]) -> ObsEpoch {
        ObsEpoch {
            t_rx_s: Seconds(epoch_idx as f64),
            source_time: ReceiverSampleTrace::from_sample_index(epoch_idx, 1.0),
            gps_week: Some(0),
            tow_s: Some(Seconds(epoch_idx as f64)),
            epoch_idx,
            discontinuity: false,
            valid: true,
            processing_ms: None,
            role: ReceiverRole::Rover,
            sats: cn0_values_dbhz
                .iter()
                .enumerate()
                .map(|(offset, cn0_dbhz)| ObsSatellite {
                    signal_id: SigId {
                        sat: SatId { constellation: Constellation::Gps, prn: 7 + offset as u8 },
                        band: SignalBand::L1,
                        code: SignalCode::Ca,
                    },
                    pseudorange_m: Meters(20_000_000.0 + offset as f64),
                    pseudorange_var_m2: 1.0,
                    carrier_phase_cycles: Cycles(1000.0 + offset as f64),
                    carrier_phase_var_cycles2: 1.0,
                    doppler_hz: Hertz(-500.0 + offset as f64),
                    doppler_var_hz2: 1.0,
                    cn0_dbhz: *cn0_dbhz,
                    lock_flags: LockFlags {
                        code_lock: true,
                        carrier_lock: true,
                        bit_lock: true,
                        cycle_slip: false,
                    },
                    multipath_suspect: false,
                    observation_status: ObservationStatus::Accepted,
                    observation_reject_reasons: Vec::new(),
                    elevation_deg: Some(45.0),
                    azimuth_deg: Some(90.0),
                    weight: Some(1.0),
                    timing: None,
                    error_model: None,
                    metadata: ObsMetadata {
                        signal: SignalSpec {
                            constellation: Constellation::Gps,
                            band: SignalBand::L1,
                            code: SignalCode::Ca,
                            code_rate_hz: 1_023_000.0,
                            carrier_hz: FreqHz(1_575_420_000.0),
                        },
                        ..ObsMetadata::default()
                    },
                })
                .collect(),
            decision: ObservationEpochDecision::Accepted,
            decision_reason: Some("synthetic_cn0_profile".to_string()),
            manifest: None,
        }
    }

    fn collect_frames(source: &mut SyntheticSignalSource, frame_len: usize) -> SamplesFrame {
        let mut frames = Vec::new();
        while let Some(frame) = source.next_frame(frame_len).expect("synthetic frame") {
            frames.push(frame);
        }
        let first = frames.first().expect("at least one frame");
        let t0 = first.t0;
        let dt_s = first.dt_s;
        let iq = frames.into_iter().flat_map(|frame| frame.iq).collect::<Vec<_>>();
        SamplesFrame::new(t0, dt_s, iq)
    }

    #[test]
    fn synthetic_epoch_start_phase_matches_theoretical_phase_after_sixty_seconds() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_000_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            ..ReceiverPipelineConfig::default()
        };
        let params = SyntheticSignalParams {
            sat: SatId { constellation: Constellation::Gps, prn: 11 },
            doppler_hz: 0.0,
            code_phase_chips: 200.375,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 58.0,
            data_bit_flip: false,
        };
        let sat_state = SatState::new_with_receiver_clock_frequency_bias_hz(&config, params, 0.0);
        let code_period_samples = samples_per_code(
            config.sampling_freq_hz,
            config.code_freq_basis_hz,
            config.code_length,
        );
        let sixty_second_sample_index = (60.0 * config.sampling_freq_hz) as u64;
        let frame = synthetic_epoch_frame(
            &sat_state,
            &config,
            sixty_second_sample_index,
            code_period_samples,
        );

        let actual_phase_samples =
            super::code_phase_samples_at_epoch_start(&config, &frame, params.code_phase_chips);
        let expected_chip_phase = advance_code_phase_seconds(
            params.code_phase_chips,
            config.code_freq_basis_hz,
            60.0,
            config.code_length,
        )
        .expect("valid theoretical phase");
        let expected_phase_samples =
            expected_chip_phase * config.sampling_freq_hz / config.code_freq_basis_hz;

        assert_phase_samples_close(actual_phase_samples, expected_phase_samples);

        let expected_code = sample_ca_code(
            Prn(params.sat.prn),
            config.sampling_freq_hz,
            config.code_freq_basis_hz,
            expected_chip_phase,
            code_period_samples,
        )
        .expect("valid expected sampled code");
        let amplitude = signal_amplitude_from_cn0(params.cn0_db_hz, config.sampling_freq_hz);

        for (index, (sample, expected_chip)) in
            frame.iq.iter().zip(expected_code.iter()).enumerate()
        {
            let expected_value = *expected_chip * amplitude;
            assert!(
                (sample.re - expected_value).abs() <= 1e-6,
                "I component drifted at sample {index}: actual={}, expected={expected_value}",
                sample.re
            );
            assert!(
                sample.im.abs() <= 1e-6,
                "Q component drifted at sample {index}: actual={}",
                sample.im
            );
        }
    }

    fn synthetic_epoch_frame(
        sat_state: &SatState,
        config: &ReceiverPipelineConfig,
        start_sample_index: u64,
        sample_count: usize,
    ) -> SamplesFrame {
        let dt_s = 1.0 / config.sampling_freq_hz;
        let iq = (0..sample_count)
            .map(|offset| sat_state.sample_at((start_sample_index + offset as u64) as f64 * dt_s))
            .collect::<Vec<_>>();
        SamplesFrame::new(
            SampleTime {
                sample_index: start_sample_index,
                sample_rate_hz: config.sampling_freq_hz,
            },
            Seconds(dt_s),
            iq,
        )
    }

    fn assert_phase_samples_close(actual: f64, expected: f64) {
        let delta = (actual - expected).abs();
        assert!(
            delta <= RECEIVER_PHASE_TOLERANCE_SAMPLES,
            "code phase samples drifted at sixty seconds: actual={actual:.12}, expected={expected:.12}, delta={delta:.12}"
        );
    }

    fn phase_step_rad(left: Complex<f32>, right: Complex<f32>) -> f32 {
        (right * left.conj()).arg()
    }

    fn wrap_phase_rad(phase_rad: f64) -> f64 {
        let tau = std::f64::consts::TAU;
        (phase_rad + std::f64::consts::PI).rem_euclid(tau) - std::f64::consts::PI
    }

    #[test]
    fn truth_bundle_records_constant_and_alternating_nav_bit_truth() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_000_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            ..ReceiverPipelineConfig::default()
        };
        let scenario = SyntheticScenario {
            sample_rate_hz: config.sampling_freq_hz,
            intermediate_freq_hz: config.intermediate_freq_hz,
            receiver_clock_frequency_bias_hz: 250.0,
            duration_s: 0.05,
            seed: 44,
            satellites: vec![
                SyntheticSignalParams {
                    sat: SatId { constellation: Constellation::Gps, prn: 3 },
                    doppler_hz: 500.0,
                    code_phase_chips: 200.0,
                    carrier_phase_rad: 0.0,
                    cn0_db_hz: 50.0,
                    data_bit_flip: false,
                },
                SyntheticSignalParams {
                    sat: SatId { constellation: Constellation::Gps, prn: 7 },
                    doppler_hz: -750.0,
                    code_phase_chips: 321.0,
                    carrier_phase_rad: 0.2,
                    cn0_db_hz: 45.0,
                    data_bit_flip: true,
                },
            ],
            ephemerides: Vec::new(),
            id: "truth-bundle".to_string(),
        };
        let frame = generate_l1_ca_multi(&config, &scenario);
        let metadata = RawIqMetadata {
            format: IqSampleFormat::Iq16Le,
            sample_rate_hz: config.sampling_freq_hz,
            intermediate_freq_hz: config.intermediate_freq_hz,
            capture_start_utc: "2026-07-09T00:00:00Z".to_string(),
            offset_bytes: 0,
            quantization_bits: Some(16),
            notes: Some("synthetic truth bundle".to_string()),
        };

        let truth = build_truth_bundle(&scenario.id, &scenario, &frame, &metadata, 1.25, 0.8);

        assert_eq!(truth.schema_version, 3);
        assert_eq!(truth.scenario_id, "truth-bundle");
        assert_eq!(truth.seed, 44);
        assert_eq!(truth.sample_format, IqSampleFormat::Iq16Le);
        assert_eq!(truth.sample_rate_hz, 4_000_000.0);
        assert_eq!(truth.receiver_clock_frequency_bias_hz, 250.0);
        assert_eq!(truth.quantization_bits, 16);
        assert_eq!(truth.noise_std_per_component, SYNTHETIC_NOISE_STD_PER_COMPONENT);
        assert_eq!(truth.noise_power_per_complex_sample, SYNTHETIC_COMPLEX_NOISE_POWER as f32);
        assert_eq!(truth.peak_component_before_scaling, 1.25);
        assert_eq!(truth.output_scale_applied, 0.8);
        assert_eq!(truth.satellites.len(), 2);

        let constant = &truth.satellites[0];
        assert_eq!(
            constant.signal_amplitude,
            signal_amplitude_from_cn0(constant.cn0_db_hz, truth.sample_rate_hz)
        );
        assert_eq!(constant.nav_bit_mode, SyntheticNavBitMode::ConstantPositive);
        assert_eq!(constant.nav_bit_segments.len(), 1);
        assert_eq!(constant.nav_bit_segments[0].start_sample, 0);
        assert_eq!(constant.nav_bit_segments[0].end_sample, frame.len() as u64);
        assert_eq!(constant.nav_bit_segments[0].bit, 1);

        let alternating = &truth.satellites[1];
        assert_eq!(
            alternating.signal_amplitude,
            signal_amplitude_from_cn0(alternating.cn0_db_hz, truth.sample_rate_hz)
        );
        assert!(constant.signal_amplitude > alternating.signal_amplitude);
        assert_eq!(alternating.nav_bit_mode, SyntheticNavBitMode::AlternatingGpsLnav20ms);
        assert_eq!(alternating.nav_bit_segments.len(), 3);
        assert_eq!(alternating.nav_bit_segments[0].start_sample, 0);
        assert_eq!(alternating.nav_bit_segments[0].end_sample, 80_000);
        assert_eq!(alternating.nav_bit_segments[0].bit, 1);
        assert_eq!(alternating.nav_bit_segments[1].start_sample, 80_000);
        assert_eq!(alternating.nav_bit_segments[1].end_sample, 160_000);
        assert_eq!(alternating.nav_bit_segments[1].bit, -1);
        assert_eq!(alternating.nav_bit_segments[2].start_sample, 160_000);
        assert_eq!(alternating.nav_bit_segments[2].end_sample, frame.len() as u64);
        assert_eq!(alternating.nav_bit_segments[2].bit, 1);
    }

    #[test]
    fn receiver_clock_frequency_bias_shifts_synthetic_carrier_phase_increment() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            ..ReceiverPipelineConfig::default()
        };
        let params = SyntheticSignalParams {
            sat: SatId { constellation: Constellation::Gps, prn: 3 },
            doppler_hz: 1_000.0,
            code_phase_chips: 0.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 60.0,
            data_bit_flip: false,
        };
        let unbiased = SatState::new_with_receiver_clock_frequency_bias_hz(&config, params, 0.0);
        let biased = SatState::new_with_receiver_clock_frequency_bias_hz(&config, params, 500.0);
        let sample_dt_s = 1.0 / config.sampling_freq_hz;
        let unbiased_phase_step =
            phase_step_rad(unbiased.sample_at(0.0), unbiased.sample_at(sample_dt_s));
        let biased_phase_step =
            phase_step_rad(biased.sample_at(0.0), biased.sample_at(sample_dt_s));
        let expected_extra_phase_step = std::f64::consts::TAU * 500.0 / config.sampling_freq_hz;
        let actual_extra_phase_step =
            wrap_phase_rad((biased_phase_step - unbiased_phase_step) as f64);

        assert!(
            (actual_extra_phase_step - expected_extra_phase_step).abs() <= 1e-6,
            "receiver clock bias phase step mismatch: actual={actual_extra_phase_step}, expected={expected_extra_phase_step}"
        );
    }

    #[test]
    fn synthetic_fade_windows_zero_signal_inside_the_requested_interval() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 1_023_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            ..ReceiverPipelineConfig::default()
        };
        let params = SyntheticSignalParams {
            sat: SatId { constellation: Constellation::Gps, prn: 5 },
            doppler_hz: 0.0,
            code_phase_chips: 0.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 60.0,
            data_bit_flip: false,
        };
        let mut signal_only = super::generate_l1_ca_signal_only(&config, params, 0.010);
        super::apply_synthetic_fade_windows(
            &mut signal_only,
            &[SyntheticFadeWindow { start_s: 0.002, end_s: 0.004, signal_scale: 0.0 }],
        );

        let fade_start = (0.002 * config.sampling_freq_hz).round() as usize;
        let fade_end = (0.004 * config.sampling_freq_hz).round() as usize;
        assert!(
            signal_only.iq[..fade_start].iter().any(|sample| sample.norm_sqr() > 0.0),
            "samples before the fade must preserve signal energy"
        );
        assert!(
            signal_only.iq[fade_start..fade_end]
                .iter()
                .all(|sample| sample.norm_sqr() <= f32::EPSILON),
            "samples inside the fade window must be fully attenuated"
        );
        assert!(
            signal_only.iq[fade_end..].iter().any(|sample| sample.norm_sqr() > 0.0),
            "samples after the fade must preserve signal energy"
        );
    }

    #[test]
    fn synthetic_fade_generator_preserves_noise_outside_the_attenuated_signal_window() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 1_023_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            ..ReceiverPipelineConfig::default()
        };
        let params = SyntheticSignalParams {
            sat: SatId { constellation: Constellation::Gps, prn: 5 },
            doppler_hz: 0.0,
            code_phase_chips: 0.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 60.0,
            data_bit_flip: false,
        };
        let baseline = generate_l1_ca(&config, params, 0xFADE_0001, 0.010);
        let faded = generate_l1_ca_with_fades(
            &config,
            params,
            &[SyntheticFadeWindow { start_s: 0.002, end_s: 0.004, signal_scale: 0.0 }],
            0xFADE_0001,
            0.010,
        );

        let fade_start = (0.002 * config.sampling_freq_hz).round() as usize;
        let fade_end = (0.004 * config.sampling_freq_hz).round() as usize;
        assert_eq!(baseline.iq[..fade_start], faded.iq[..fade_start]);
        assert_eq!(baseline.iq[fade_end..], faded.iq[fade_end..]);
        assert!(
            baseline.iq[fade_start..fade_end] != faded.iq[fade_start..fade_end],
            "the attenuated window must differ from the unfaded baseline"
        );
    }

    #[test]
    fn synthetic_phase_window_rotates_signal_only_inside_configured_window() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 1_023_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            ..ReceiverPipelineConfig::default()
        };
        let params = SyntheticSignalParams {
            sat: SatId { constellation: Constellation::Gps, prn: 5 },
            doppler_hz: 0.0,
            code_phase_chips: 0.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 60.0,
            data_bit_flip: false,
        };
        let mut signal_only = super::generate_l1_ca_signal_only(&config, params, 0.010);
        super::apply_synthetic_phase_windows(
            &mut signal_only,
            &[SyntheticPhaseWindow {
                start_s: 0.002,
                end_s: 0.004,
                phase_offset_rad: std::f64::consts::FRAC_PI_2,
            }],
        );

        let phase_start = (0.002 * config.sampling_freq_hz).round() as usize;
        let phase_end = (0.004 * config.sampling_freq_hz).round() as usize;
        assert!(signal_only.iq[..phase_start].iter().all(|sample| sample.im.abs() <= 1.0e-6));
        assert!(
            signal_only.iq[phase_start..phase_end].iter().any(|sample| sample.im.abs() > 1.0e-3),
            "windowed samples must carry a rotated quadrature component"
        );
        assert!(signal_only.iq[phase_end..].iter().all(|sample| sample.im.abs() <= 1.0e-6));
    }

    #[test]
    fn synthetic_phase_generator_preserves_noise_outside_the_rotated_window() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 1_023_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            ..ReceiverPipelineConfig::default()
        };
        let params = SyntheticSignalParams {
            sat: SatId { constellation: Constellation::Gps, prn: 5 },
            doppler_hz: 0.0,
            code_phase_chips: 0.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 60.0,
            data_bit_flip: false,
        };
        let baseline = generate_l1_ca(&config, params, 0xFACE_0001, 0.010);
        let rotated = generate_l1_ca_with_phase_windows(
            &config,
            params,
            &[SyntheticPhaseWindow { start_s: 0.002, end_s: 0.004, phase_offset_rad: 0.8 }],
            0xFACE_0001,
            0.010,
        );

        let phase_start = (0.002 * config.sampling_freq_hz).round() as usize;
        let phase_end = (0.004 * config.sampling_freq_hz).round() as usize;
        assert_eq!(baseline.iq[..phase_start], rotated.iq[..phase_start]);
        assert_eq!(baseline.iq[phase_end..], rotated.iq[phase_end..]);
        assert_ne!(baseline.iq[phase_start..phase_end], rotated.iq[phase_start..phase_end]);
    }

    #[test]
    fn zero_doppler_rate_matches_constant_synthetic_generator() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            ..ReceiverPipelineConfig::default()
        };
        let params = SyntheticSignalParams {
            sat: SatId { constellation: Constellation::Gps, prn: 9 },
            doppler_hz: 750.0,
            code_phase_chips: 144.375,
            carrier_phase_rad: 0.15,
            cn0_db_hz: 58.0,
            data_bit_flip: false,
        };
        let baseline = generate_l1_ca(&config, params, 0xD0A0_0001, 0.020);
        let ramped = generate_l1_ca_with_doppler_ramp(
            &config,
            SyntheticDopplerRampParams { signal: params, doppler_rate_hz_per_s: 0.0 },
            0xD0A0_0001,
            0.020,
        );

        assert_eq!(ramped.t0.sample_index, baseline.t0.sample_index);
        assert_eq!(ramped.t0.sample_rate_hz, baseline.t0.sample_rate_hz);
        assert_eq!(ramped.dt_s.0, baseline.dt_s.0);
        assert_eq!(ramped.iq, baseline.iq);
    }

    #[test]
    fn doppler_ramp_updates_instantaneous_carrier_frequency_linearly() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            ..ReceiverPipelineConfig::default()
        };
        let params = SyntheticSignalParams {
            sat: SatId { constellation: Constellation::Gps, prn: 14 },
            doppler_hz: 1_200.0,
            code_phase_chips: 0.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 60.0,
            data_bit_flip: false,
        };
        let sat_state = SatState::new_with_doppler_rate_and_receiver_clock_frequency_bias_hz(
            &config, params, 150.0, 40.0,
        );

        assert!((sat_state.carrier_hz_at(0.0) - 1_350.0).abs() <= 1e-12);
        assert!((sat_state.carrier_hz_at(0.25) - 1_360.0).abs() <= 1e-12);
        assert!((sat_state.carrier_hz_at(0.50) - 1_370.0).abs() <= 1e-12);
    }

    #[test]
    fn doppler_ramp_integrates_into_carrier_phase_quadratically() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            ..ReceiverPipelineConfig::default()
        };
        let params = SyntheticSignalParams {
            sat: SatId { constellation: Constellation::Gps, prn: 21 },
            doppler_hz: 900.0,
            code_phase_chips: 0.0,
            carrier_phase_rad: 0.35,
            cn0_db_hz: 60.0,
            data_bit_flip: false,
        };
        let sat_state = SatState::new_with_doppler_rate_and_receiver_clock_frequency_bias_hz(
            &config, params, 100.0, -25.0,
        );
        let t_s = 0.40;
        let expected_phase_rad =
            0.35 + std::f64::consts::TAU * ((1_000.0 * t_s) + 0.5 * (-25.0) * t_s * t_s);

        assert!(
            (sat_state.carrier_phase_rad_at(t_s) - expected_phase_rad).abs() <= 1e-9,
            "carrier phase ramp mismatch: actual={}, expected={expected_phase_rad}",
            sat_state.carrier_phase_rad_at(t_s)
        );
    }

    #[test]
    fn alternating_nav_bit_sign_flips_on_twenty_millisecond_boundaries() {
        assert_eq!(nav_bit_index_at_time_s(-1.0), 0);
        assert_eq!(nav_bit_index_at_time_s(0.0), 0);
        assert_eq!(nav_bit_index_at_time_s(0.019_999_999), 0);
        assert_eq!(nav_bit_index_at_time_s(0.020_000_000), 1);
        assert_eq!(nav_bit_index_at_time_s(0.039_999_999), 1);
        assert_eq!(nav_bit_index_at_time_s(0.040_000_000), 2);

        assert_eq!(nav_bit_sign_at_time_s(false, 0.0), 1);
        assert_eq!(nav_bit_sign_at_time_s(true, 0.0), 1);
        assert_eq!(nav_bit_sign_at_time_s(true, 0.019_999_999), 1);
        assert_eq!(nav_bit_sign_at_time_s(true, 0.020_000_000), -1);
        assert_eq!(nav_bit_sign_at_time_s(true, 0.039_999_999), -1);
        assert_eq!(nav_bit_sign_at_time_s(true, 0.040_000_000), 1);
    }

    #[test]
    fn iq16_capture_bundle_scales_without_clipping_and_preserves_truth() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            ..ReceiverPipelineConfig::default()
        };
        let scenario = SyntheticScenario {
            sample_rate_hz: config.sampling_freq_hz,
            intermediate_freq_hz: config.intermediate_freq_hz,
            receiver_clock_frequency_bias_hz: 0.0,
            duration_s: 0.01,
            seed: 91,
            satellites: vec![
                SyntheticSignalParams {
                    sat: SatId { constellation: Constellation::Gps, prn: 3 },
                    doppler_hz: 500.0,
                    code_phase_chips: 200.0,
                    carrier_phase_rad: 0.0,
                    cn0_db_hz: 58.0,
                    data_bit_flip: true,
                },
                SyntheticSignalParams {
                    sat: SatId { constellation: Constellation::Gps, prn: 7 },
                    doppler_hz: -1000.0,
                    code_phase_chips: 321.0,
                    carrier_phase_rad: 0.2,
                    cn0_db_hz: 56.0,
                    data_bit_flip: false,
                },
            ],
            ephemerides: Vec::new(),
            id: "iq16-bundle".to_string(),
        };
        let frame = generate_l1_ca_multi(&config, &scenario);

        let bundle = build_iq16_capture_bundle(
            &scenario.id,
            &scenario,
            &frame,
            "2026-07-09T00:00:00Z",
            Some("synthetic iq bundle".to_string()),
        );

        assert_eq!(bundle.metadata.format, IqSampleFormat::Iq16Le);
        assert_eq!(bundle.metadata.quantization_bits, Some(16));
        assert_eq!(bundle.truth.scenario_id, "iq16-bundle");
        assert_eq!(bundle.truth.sample_count, frame.len());
        assert_eq!(bundle.truth.sample_rate_hz, 4_092_000.0);
        assert_eq!(bundle.truth.noise_std_per_component, SYNTHETIC_NOISE_STD_PER_COMPONENT);
        assert_eq!(
            bundle.truth.noise_power_per_complex_sample,
            SYNTHETIC_COMPLEX_NOISE_POWER as f32
        );
        assert_eq!(bundle.raw_iq_bytes.len(), frame.len() * 4);
        assert!(bundle.truth.peak_component_before_scaling > 0.0);
        assert!(bundle.truth.output_scale_applied > 0.0);
        assert!(bundle.truth.output_scale_applied <= 1.0);
        assert!(
            bundle.truth.satellites[0].signal_amplitude
                > bundle.truth.satellites[1].signal_amplitude
        );

        assert!(
            bundle
                .raw_iq_bytes
                .chunks_exact(2)
                .map(|chunk| i16::from_le_bytes([chunk[0], chunk[1]]))
                .any(|sample| sample != 0),
            "encoded synthetic capture should contain non-zero samples"
        );
    }

    #[test]
    fn truth_guided_cn0_validation_matches_injected_truth_within_tolerance() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            ..ReceiverPipelineConfig::default()
        };
        let scenario = SyntheticScenario {
            sample_rate_hz: config.sampling_freq_hz,
            intermediate_freq_hz: config.intermediate_freq_hz,
            receiver_clock_frequency_bias_hz: 0.0,
            duration_s: 0.01,
            seed: 17,
            satellites: vec![SyntheticSignalParams {
                sat: SatId { constellation: Constellation::Gps, prn: 7 },
                doppler_hz: -1000.0,
                code_phase_chips: 321.0,
                carrier_phase_rad: 0.2,
                cn0_db_hz: 52.0,
                data_bit_flip: false,
            }],
            ephemerides: Vec::new(),
            id: "cn0-validation".to_string(),
        };
        let frame = generate_l1_ca_multi(&config, &scenario);
        let bundle = build_iq16_capture_bundle(
            &scenario.id,
            &scenario,
            &frame,
            "2026-07-09T00:00:00Z",
            Some("synthetic cn0 validation".to_string()),
        );
        let scaled_frame = SamplesFrame::new(
            frame.t0,
            frame.dt_s,
            frame.iq.iter().map(|sample| *sample * bundle.truth.output_scale_applied).collect(),
        );
        let report = validate_truth_guided_cn0(&config, &scaled_frame, &bundle.truth, 3.0);

        assert!(report.pass, "{report:?}");
        assert_eq!(report.coherent_samples_per_epoch, 4092);
        assert_eq!(report.satellites.len(), 1);
        for row in &report.satellites {
            assert!(row.pass, "{row:?}");
            assert!(row.epochs_measured >= super::TRACKING_CN0_MIN_STABLE_EPOCHS, "{row:?}");
            assert!(row.epochs_measured <= 10, "{row:?}");
            assert!(row.cn0_delta_db.abs() <= 3.0, "{row:?}");
            assert!(row.measured_max_cn0_dbhz >= row.measured_min_cn0_dbhz);
        }
    }

    #[test]
    fn expected_acquisition_code_phase_uses_receiver_search_convention() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            ..ReceiverPipelineConfig::default()
        };
        let period_samples = samples_per_code(
            config.sampling_freq_hz,
            config.code_freq_basis_hz,
            config.code_length,
        );
        let frame = SamplesFrame::new(
            SampleTime { sample_index: 0, sample_rate_hz: config.sampling_freq_hz },
            Seconds(1.0 / config.sampling_freq_hz),
            vec![num_complex::Complex::new(0.0f32, 0.0f32); period_samples],
        );

        assert_eq!(expected_acquisition_code_phase_samples(&config, &frame, 0.0), 0);
        assert_eq!(
            expected_acquisition_code_phase_samples(
                &config,
                &frame,
                (period_samples - 1) as f64 * config.code_freq_basis_hz / config.sampling_freq_hz,
            ),
            1
        );
        assert!(
            (expected_acquisition_code_phase_samples_f64(&config, &frame, 0.125) - 4091.5).abs()
                < 1.0e-9
        );
    }

    #[test]
    fn wrapped_code_phase_error_measures_shortest_period_distance() {
        assert_eq!(wrapped_code_phase_error_samples(0, 0, 4092), 0);
        assert_eq!(wrapped_code_phase_error_samples(1, 4091, 4092), 2);
        assert_eq!(wrapped_code_phase_error_samples(4091, 1, 4092), 2);
        assert!((wrapped_code_phase_error_samples_f64(4091.5, 0.0, 4092) - 0.5).abs() < 1.0e-9);
    }

    #[test]
    fn truth_guided_acquisition_code_phase_validation_matches_clean_truth_within_two_samples() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            ..ReceiverPipelineConfig::default()
        };
        let scenario = SyntheticScenario {
            sample_rate_hz: config.sampling_freq_hz,
            intermediate_freq_hz: config.intermediate_freq_hz,
            receiver_clock_frequency_bias_hz: 0.0,
            duration_s: 0.04,
            seed: 24071985,
            satellites: vec![
                SyntheticSignalParams {
                    sat: SatId { constellation: Constellation::Gps, prn: 3 },
                    doppler_hz: 750.0,
                    code_phase_chips: 200.25,
                    carrier_phase_rad: 0.0,
                    cn0_db_hz: 58.0,
                    data_bit_flip: true,
                },
                SyntheticSignalParams {
                    sat: SatId { constellation: Constellation::Gps, prn: 7 },
                    doppler_hz: -1000.0,
                    code_phase_chips: 321.5,
                    carrier_phase_rad: 0.2,
                    cn0_db_hz: 52.0,
                    data_bit_flip: false,
                },
            ],
            ephemerides: Vec::new(),
            id: "acquisition-code-phase-validation".to_string(),
        };
        let frame = generate_l1_ca_multi(&config, &scenario);
        let bundle = build_iq16_capture_bundle(
            &scenario.id,
            &scenario,
            &frame,
            "2026-07-09T00:00:00Z",
            Some("synthetic acquisition code-phase validation".to_string()),
        );
        let scaled_frame = SamplesFrame::new(
            frame.t0,
            frame.dt_s,
            frame.iq.iter().map(|sample| *sample * bundle.truth.output_scale_applied).collect(),
        );
        let report =
            validate_truth_guided_acquisition_code_phase(&config, &scaled_frame, &bundle.truth, 2);

        assert!(report.pass, "{report:?}");
        assert_eq!(report.period_samples, 4092);
        assert_eq!(report.satellites.len(), 2);
        for row in &report.satellites {
            assert!(row.pass, "{row:?}");
            assert!(row.code_phase_error_samples <= 2, "{row:?}");
            assert!(matches!(row.hypothesis.as_str(), "accepted" | "ambiguous"));
        }
    }

    #[test]
    fn truth_guided_code_phase_refinement_improves_fractional_pseudorange_initialization() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_000_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            ..ReceiverPipelineConfig::default()
        };
        let mut best_improvement_samples = 0.0_f64;

        for code_phase_chips in [200.125, 200.25, 200.375, 200.5, 200.625, 200.75, 200.875] {
            let scenario = SyntheticScenario {
                sample_rate_hz: config.sampling_freq_hz,
                intermediate_freq_hz: config.intermediate_freq_hz,
                receiver_clock_frequency_bias_hz: 0.0,
                duration_s: 0.04,
                seed: 24071985,
                satellites: vec![SyntheticSignalParams {
                    sat: SatId { constellation: Constellation::Gps, prn: 3 },
                    doppler_hz: 0.0,
                    code_phase_chips,
                    carrier_phase_rad: 0.0,
                    cn0_db_hz: 65.0,
                    data_bit_flip: false,
                }],
                ephemerides: Vec::new(),
                id: "acquisition_code_phase_refinement_truth".to_string(),
            };
            let frame = generate_l1_ca_multi(&config, &scenario);
            let bundle = build_iq16_capture_bundle(
                &scenario.id,
                &scenario,
                &frame,
                "2026-07-09T00:00:00Z",
                Some("unit code-phase refinement validation".to_string()),
            );
            let scaled_frame = SamplesFrame::new(
                frame.t0,
                frame.dt_s,
                frame.iq.iter().map(|sample| *sample * bundle.truth.output_scale_applied).collect(),
            );

            let report = validate_truth_guided_acquisition_code_phase_refinement(
                &config,
                &scaled_frame,
                &bundle.truth,
            );

            assert!(report.pass, "{report:?}");
            assert_eq!(report.satellites.len(), 1);
            let row = &report.satellites[0];
            assert!(row.pass, "{row:?}");
            best_improvement_samples = best_improvement_samples.max(row.improvement_samples);
        }

        assert!(
            best_improvement_samples > 0.0,
            "expected at least one fractional synthetic fixture to improve"
        );
    }

    #[test]
    fn truth_guided_acquisition_coherent_integration_report_combines_code_phase_and_doppler() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            acquisition_doppler_search_hz: 10_000,
            acquisition_doppler_step_hz: 500,
            ..ReceiverPipelineConfig::default()
        };
        let scenario = SyntheticScenario {
            sample_rate_hz: config.sampling_freq_hz,
            intermediate_freq_hz: config.intermediate_freq_hz,
            receiver_clock_frequency_bias_hz: 0.0,
            duration_s: 0.04,
            seed: 2_407_1985,
            satellites: vec![SyntheticSignalParams {
                sat: SatId { constellation: Constellation::Gps, prn: 3 },
                doppler_hz: 750.0,
                code_phase_chips: 200.25,
                carrier_phase_rad: 0.0,
                cn0_db_hz: 58.0,
                data_bit_flip: false,
            }],
            ephemerides: Vec::new(),
            id: "acquisition-coherent-integration-profile".to_string(),
        };
        let frame = generate_l1_ca_multi(&config, &scenario);
        let bundle = build_iq16_capture_bundle(
            &scenario.id,
            &scenario,
            &frame,
            "2026-07-09T00:00:00Z",
            Some("unit acquisition coherent integration validation".to_string()),
        );
        let scaled_frame = SamplesFrame::new(
            frame.t0,
            frame.dt_s,
            frame.iq.iter().map(|sample| *sample * bundle.truth.output_scale_applied).collect(),
        );

        let report = validate_truth_guided_acquisition_coherent_integration(
            &config,
            &scaled_frame,
            &bundle.truth,
            1,
            1,
            2,
            1,
        );

        assert!(report.pass, "{report:?}");
        assert_eq!(report.coherent_ms, 1);
        assert_eq!(report.noncoherent, 1);
        assert_eq!(report.satellites.len(), 1);
        assert!(report.satellites[0].pass, "{:?}", report.satellites[0]);
    }

    #[test]
    fn acquisition_detection_probability_report_tracks_accepted_trials() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            acquisition_doppler_search_hz: 10_000,
            acquisition_doppler_step_hz: 500,
            ..ReceiverPipelineConfig::default()
        };
        let report = measure_truth_guided_acquisition_detection_probability(
            &config,
            SyntheticSignalParams {
                sat: SatId { constellation: Constellation::Gps, prn: 3 },
                doppler_hz: 750.0,
                code_phase_chips: 200.25,
                carrier_phase_rad: 0.0,
                cn0_db_hz: 58.0,
                data_bit_flip: false,
            },
            1,
            1,
            &[2_407_1985, 2_407_1986],
            "acquisition-detection-probability",
            2,
            1,
        );

        assert_eq!(report.trial_count, 2);
        assert_eq!(
            report.accepted_count,
            report.trials.iter().filter(|trial| trial.accepted).count()
        );
        assert_eq!(
            report.detected_count,
            report.trials.iter().filter(|trial| trial.detected).count()
        );
        assert!(report.accepted_count <= report.trial_count);
        assert!(report.detected_count <= report.accepted_count);
        assert!(
            (report.acceptance_probability
                - report.accepted_count as f64 / report.trial_count as f64)
                .abs()
                <= f64::EPSILON
        );
        assert!(
            (report.detection_probability
                - report.detected_count as f64 / report.trial_count as f64)
                .abs()
                <= f64::EPSILON
        );
        assert!(
            report.trials.iter().all(|trial| trial.code_phase_error_samples.is_some()
                && trial.doppler_error_bins.is_some()),
            "{report:?}"
        );
    }

    #[test]
    fn tracking_sensitivity_report_counts_stable_lock_and_refusal_trials() {
        let sat = SatId { constellation: Constellation::Gps, prn: 7 };
        let report = synthetic_tracking_sensitivity_report(
            "tracking-sensitivity",
            sat,
            24.0,
            0.06,
            60.0,
            1,
            5,
            vec![
                SyntheticTrackingSensitivityTrial {
                    scenario_id: "tracking-sensitivity-trial-0".to_string(),
                    seed: 1,
                    sat,
                    stable_lock: true,
                    refused_lock: false,
                    first_lock_epoch_index: Some(4),
                    locked_epoch_count: 8,
                    final_lock_state: "tracking".to_string(),
                    final_lock_state_reason: Some("carrier_converged".to_string()),
                },
                SyntheticTrackingSensitivityTrial {
                    scenario_id: "tracking-sensitivity-trial-1".to_string(),
                    seed: 2,
                    sat,
                    stable_lock: false,
                    refused_lock: true,
                    first_lock_epoch_index: None,
                    locked_epoch_count: 0,
                    final_lock_state: "pull_in".to_string(),
                    final_lock_state_reason: Some("cn0_below_tracking_lock_floor".to_string()),
                },
            ],
        );

        assert_eq!(report.trial_count, 2);
        assert_eq!(report.stable_lock_count, 1);
        assert_eq!(report.refused_lock_count, 1);
        assert!((report.lock_probability - 0.5).abs() <= f64::EPSILON);
        assert!((report.mean_locked_epochs - 4.0).abs() <= f64::EPSILON);
    }

    #[test]
    fn tracking_lock_rate_report_keeps_cn0_and_seeded_error_axes() {
        let sat = SatId { constellation: Constellation::Gps, prn: 7 };
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            channels: 4,
            early_late_spacing_chips: 0.5,
            dll_bw_hz: 2.0,
            pll_bw_hz: 18.0,
            fll_bw_hz: 12.0,
            ..ReceiverPipelineConfig::default()
        };
        let report = measure_truth_guided_tracking_lock_rate(
            &config,
            &[
                SyntheticTrackingLockRateCase {
                    signal: SyntheticSignalParams {
                        sat,
                        doppler_hz: 180.0,
                        code_phase_chips: 211.25,
                        carrier_phase_rad: 0.0,
                        cn0_db_hz: 24.0,
                        data_bit_flip: false,
                    },
                    duration_s: 0.04,
                    seeded_doppler_error_hz: 60.0,
                    seeded_code_phase_error_samples: 1,
                    min_locked_epochs: 4,
                },
                SyntheticTrackingLockRateCase {
                    signal: SyntheticSignalParams {
                        sat,
                        doppler_hz: 180.0,
                        code_phase_chips: 211.25,
                        carrier_phase_rad: 0.0,
                        cn0_db_hz: 30.0,
                        data_bit_flip: false,
                    },
                    duration_s: 0.04,
                    seeded_doppler_error_hz: 60.0,
                    seeded_code_phase_error_samples: 1,
                    min_locked_epochs: 4,
                },
            ],
            &[11, 29],
            "tracking-lock-rate-axes",
        );

        assert_eq!(report.points.len(), 2);
        assert_eq!(report.points[0].cn0_db_hz, 24.0);
        assert_eq!(report.points[1].cn0_db_hz, 30.0);
        assert_eq!(report.points[0].seeded_doppler_error_hz, 60.0);
        assert_eq!(report.points[0].seeded_code_phase_error_samples, 1);
        assert_eq!(report.points[0].trial_count, 2);
    }

    #[test]
    fn acquisition_false_alarm_report_tracks_noise_only_trials() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            acquisition_doppler_search_hz: 10_000,
            acquisition_doppler_step_hz: 500,
            ..ReceiverPipelineConfig::default()
        };
        let report = measure_noise_only_acquisition_false_alarm_rate(
            &config,
            SatId { constellation: Constellation::Gps, prn: 3 },
            1,
            1,
            &[17, 29],
            "acquisition-false-alarm",
        );

        assert_eq!(report.trial_count, 2);
        assert_eq!(report.false_alarm_count, 0);
        assert_eq!(report.false_alarm_rate, 0.0);
        assert!(report.trials.iter().all(|trial| !trial.accepted), "{report:?}");
    }

    #[test]
    fn acquisition_false_alarm_rate_report_keeps_integration_axes() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            acquisition_doppler_search_hz: 1_500,
            acquisition_doppler_step_hz: 250,
            ..ReceiverPipelineConfig::default()
        };
        let report = measure_noise_only_acquisition_false_alarm_rates(
            &config,
            &[
                SyntheticAcquisitionFalseAlarmRateCase {
                    sat: SatId { constellation: Constellation::Gps, prn: 7 },
                    coherent_ms: 1,
                    noncoherent: 1,
                },
                SyntheticAcquisitionFalseAlarmRateCase {
                    sat: SatId { constellation: Constellation::Gps, prn: 7 },
                    coherent_ms: 5,
                    noncoherent: 4,
                },
            ],
            &[31, 37],
            "acquisition-false-alarm-rate",
        );

        assert_eq!(report.acquisition_doppler_search_hz, 1_500);
        assert_eq!(report.acquisition_doppler_step_hz, 250);
        assert_eq!(report.points.len(), 2);
        assert_eq!(report.points[0].coherent_ms, 1);
        assert_eq!(report.points[0].noncoherent, 1);
        assert_eq!(report.points[1].coherent_ms, 5);
        assert_eq!(report.points[1].noncoherent, 4);
        assert!(report.points.iter().all(|point| point.trial_count == 2), "{report:?}");
    }

    #[test]
    fn acquisition_detection_rate_report_keeps_cn0_doppler_and_integration_axes() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            acquisition_doppler_search_hz: 1_500,
            acquisition_doppler_step_hz: 250,
            ..ReceiverPipelineConfig::default()
        };
        let report = measure_truth_guided_acquisition_detection_rate(
            &config,
            &[
                SyntheticAcquisitionDetectionRateCase {
                    signal: SyntheticSignalParams {
                        sat: SatId { constellation: Constellation::Gps, prn: 7 },
                        doppler_hz: 250.0,
                        code_phase_chips: 300.0,
                        carrier_phase_rad: 0.0,
                        cn0_db_hz: 30.0,
                        data_bit_flip: false,
                    },
                    coherent_ms: 1,
                    noncoherent: 1,
                },
                SyntheticAcquisitionDetectionRateCase {
                    signal: SyntheticSignalParams {
                        sat: SatId { constellation: Constellation::Gps, prn: 7 },
                        doppler_hz: 750.0,
                        code_phase_chips: 300.0,
                        carrier_phase_rad: 0.0,
                        cn0_db_hz: 34.0,
                        data_bit_flip: false,
                    },
                    coherent_ms: 5,
                    noncoherent: 1,
                },
            ],
            &[17, 29],
            "acquisition-detection-rate",
            2,
            1,
        );

        assert_eq!(report.doppler_step_hz, 250);
        assert_eq!(report.points.len(), 2);
        assert_eq!(report.points[0].cn0_db_hz, 30.0);
        assert_eq!(report.points[0].doppler_hz, 250.0);
        assert_eq!(report.points[0].coherent_ms, 1);
        assert_eq!(report.points[0].noncoherent, 1);
        assert_eq!(report.points[1].cn0_db_hz, 34.0);
        assert_eq!(report.points[1].doppler_hz, 750.0);
        assert_eq!(report.points[1].coherent_ms, 5);
        assert_eq!(report.points[1].noncoherent, 1);
    }

    #[test]
    fn observation_error_summary_tracks_bias_and_absolute_magnitude() {
        let summary =
            summarize_observation_errors(&[-2.0, 1.0, 3.0]).expect("error summary must exist");

        assert_eq!(summary.count, 3);
        assert!((summary.mean_error - (2.0 / 3.0)).abs() <= 1.0e-12, "{summary:?}");
        assert!((summary.median_abs_error - 2.0).abs() <= 1.0e-12, "{summary:?}");
        assert!((summary.rms_error - (14.0_f64 / 3.0).sqrt()).abs() <= 1.0e-12, "{summary:?}");
        assert!((summary.p95_abs_error - 3.0).abs() <= 1.0e-12, "{summary:?}");
        assert!((summary.max_abs_error - 3.0).abs() <= 1.0e-12, "{summary:?}");
    }

    #[test]
    fn truth_guided_acquisition_doppler_validation_matches_clean_truth_within_one_bin() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            acquisition_doppler_search_hz: 10_000,
            acquisition_doppler_step_hz: 500,
            ..ReceiverPipelineConfig::default()
        };
        let scenario = SyntheticScenario {
            sample_rate_hz: config.sampling_freq_hz,
            intermediate_freq_hz: config.intermediate_freq_hz,
            receiver_clock_frequency_bias_hz: 0.0,
            duration_s: 0.04,
            seed: 24071985,
            satellites: vec![
                SyntheticSignalParams {
                    sat: SatId { constellation: Constellation::Gps, prn: 3 },
                    doppler_hz: 750.0,
                    code_phase_chips: 200.25,
                    carrier_phase_rad: 0.0,
                    cn0_db_hz: 58.0,
                    data_bit_flip: true,
                },
                SyntheticSignalParams {
                    sat: SatId { constellation: Constellation::Gps, prn: 7 },
                    doppler_hz: -1_000.0,
                    code_phase_chips: 321.5,
                    carrier_phase_rad: 0.2,
                    cn0_db_hz: 52.0,
                    data_bit_flip: false,
                },
            ],
            ephemerides: Vec::new(),
            id: "acquisition-doppler-validation".to_string(),
        };
        let frame = generate_l1_ca_multi(&config, &scenario);
        let bundle = build_iq16_capture_bundle(
            &scenario.id,
            &scenario,
            &frame,
            "2026-07-09T00:00:00Z",
            Some("synthetic acquisition doppler validation".to_string()),
        );
        let scaled_frame = SamplesFrame::new(
            frame.t0,
            frame.dt_s,
            frame.iq.iter().map(|sample| *sample * bundle.truth.output_scale_applied).collect(),
        );
        let report =
            validate_truth_guided_acquisition_doppler(&config, &scaled_frame, &bundle.truth, 1);

        assert!(report.pass, "{report:?}");
        assert_eq!(report.tolerance_hz, 500.0);
        assert_eq!(report.doppler_step_hz, 500);
        assert_eq!(report.satellites.len(), 2);
        for row in &report.satellites {
            assert!(row.pass, "{row:?}");
            assert!(row.doppler_error_hz <= 500.0 + f64::EPSILON, "{row:?}");
            assert!(row.doppler_error_bins <= 1.0 + f64::EPSILON, "{row:?}");
            assert_ne!(row.hypothesis, "deferred");
        }
    }

    #[test]
    fn truth_guided_acquisition_receiver_clock_offset_validation_reports_positive_bias() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            acquisition_doppler_search_hz: 10_000,
            acquisition_doppler_step_hz: 250,
            ..ReceiverPipelineConfig::default()
        };
        let scenario = SyntheticScenario {
            sample_rate_hz: config.sampling_freq_hz,
            intermediate_freq_hz: config.intermediate_freq_hz,
            receiver_clock_frequency_bias_hz: 500.0,
            duration_s: 0.04,
            seed: 24071985,
            satellites: vec![
                SyntheticSignalParams {
                    sat: SatId { constellation: Constellation::Gps, prn: 3 },
                    doppler_hz: 750.0,
                    code_phase_chips: 200.25,
                    carrier_phase_rad: 0.0,
                    cn0_db_hz: 58.0,
                    data_bit_flip: true,
                },
                SyntheticSignalParams {
                    sat: SatId { constellation: Constellation::Gps, prn: 7 },
                    doppler_hz: -1_000.0,
                    code_phase_chips: 321.5,
                    carrier_phase_rad: 0.2,
                    cn0_db_hz: 52.0,
                    data_bit_flip: false,
                },
            ],
            ephemerides: Vec::new(),
            id: "acquisition-receiver-clock-offset-positive".to_string(),
        };
        let frame = generate_l1_ca_multi(&config, &scenario);
        let bundle = build_iq16_capture_bundle(
            &scenario.id,
            &scenario,
            &frame,
            "2026-07-09T00:00:00Z",
            Some("synthetic acquisition receiver clock offset validation".to_string()),
        );
        let scaled_frame = SamplesFrame::new(
            frame.t0,
            frame.dt_s,
            frame.iq.iter().map(|sample| *sample * bundle.truth.output_scale_applied).collect(),
        );
        let report = validate_truth_guided_acquisition_receiver_clock_offset(
            &config,
            &scaled_frame,
            &bundle.truth,
            1,
        );

        assert!(report.pass, "{report:?}");
        assert_eq!(report.injected_receiver_clock_frequency_bias_hz, 500.0);
        assert_eq!(report.tolerance_hz, 250.0);
        assert_eq!(report.doppler_step_hz, 250);
        assert_eq!(report.satellites.len(), 2);
        assert!(
            (report.mean_measured_receiver_clock_frequency_bias_hz - 500.0).abs() <= 250.0,
            "{report:?}"
        );
        for row in &report.satellites {
            assert!(row.pass, "{row:?}");
            assert!(
                (row.measured_receiver_clock_frequency_bias_hz - 500.0).abs() <= 250.0,
                "{row:?}"
            );
            assert!(
                (row.expected_measured_doppler_hz - row.measured_doppler_hz).abs() <= 250.0,
                "{row:?}"
            );
        }
    }

    #[test]
    fn truth_guided_acquisition_receiver_clock_offset_validation_reports_negative_bias() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            acquisition_doppler_search_hz: 10_000,
            acquisition_doppler_step_hz: 250,
            ..ReceiverPipelineConfig::default()
        };
        let scenario = SyntheticScenario {
            sample_rate_hz: config.sampling_freq_hz,
            intermediate_freq_hz: config.intermediate_freq_hz,
            receiver_clock_frequency_bias_hz: -500.0,
            duration_s: 0.04,
            seed: 24071986,
            satellites: vec![
                SyntheticSignalParams {
                    sat: SatId { constellation: Constellation::Gps, prn: 11 },
                    doppler_hz: 1_250.0,
                    code_phase_chips: 150.25,
                    carrier_phase_rad: 0.0,
                    cn0_db_hz: 58.0,
                    data_bit_flip: false,
                },
                SyntheticSignalParams {
                    sat: SatId { constellation: Constellation::Gps, prn: 19 },
                    doppler_hz: -750.0,
                    code_phase_chips: 420.5,
                    carrier_phase_rad: 0.3,
                    cn0_db_hz: 54.0,
                    data_bit_flip: true,
                },
            ],
            ephemerides: Vec::new(),
            id: "acquisition-receiver-clock-offset-negative".to_string(),
        };
        let frame = generate_l1_ca_multi(&config, &scenario);
        let bundle = build_iq16_capture_bundle(
            &scenario.id,
            &scenario,
            &frame,
            "2026-07-09T00:00:00Z",
            Some("synthetic acquisition receiver clock offset validation".to_string()),
        );
        let scaled_frame = SamplesFrame::new(
            frame.t0,
            frame.dt_s,
            frame.iq.iter().map(|sample| *sample * bundle.truth.output_scale_applied).collect(),
        );
        let report = validate_truth_guided_acquisition_receiver_clock_offset(
            &config,
            &scaled_frame,
            &bundle.truth,
            1,
        );

        assert!(report.pass, "{report:?}");
        assert_eq!(report.injected_receiver_clock_frequency_bias_hz, -500.0);
        assert_eq!(report.satellites.len(), 2);
        assert!(report.max_measured_receiver_clock_frequency_bias_hz <= 0.0, "{report:?}");
        assert!(report.min_measured_receiver_clock_frequency_bias_hz <= -250.0, "{report:?}");
        for row in &report.satellites {
            assert!(row.pass, "{row:?}");
            assert!(row.measured_receiver_clock_frequency_bias_hz < 0.0, "{row:?}");
        }
    }

    #[test]
    fn acquisition_sample_rate_validation_passes_with_distinct_low_and_high_rate_profiles() {
        let low_rate = synthetic_acquisition_sample_rate_case_fixture(
            2_046_000.0,
            "acquisition_sample_rate_validation_low_rate",
            0x2_046_000,
        );
        let high_rate = synthetic_acquisition_sample_rate_case_fixture(
            4_092_000.0,
            "acquisition_sample_rate_validation_high_rate",
            0x4_092_000,
        );

        let report =
            validate_truth_guided_acquisition_sample_rates(&[low_rate.case, high_rate.case], 2, 1);

        assert!(report.pass, "{report:?}");
        assert_eq!(report.code_phase_tolerance_samples, 2);
        assert_eq!(report.doppler_tolerance_bins, 1);
        assert_eq!(report.doppler_tolerance_hz, 500.0);
        assert_eq!(report.distinct_sample_rate_count, 2);
        assert_eq!(report.min_sample_rate_hz, 2_046_000.0);
        assert_eq!(report.max_sample_rate_hz, 4_092_000.0);
        assert_eq!(report.points.len(), 2);
        assert_eq!(report.points[0].scenario_id, low_rate.truth.scenario_id);
        assert_eq!(report.points[1].scenario_id, high_rate.truth.scenario_id);
        for point in &report.points {
            assert!(point.pass, "{point:?}");
            assert!(point.code_phase_validation.pass, "{point:?}");
            assert!(point.doppler_validation.pass, "{point:?}");
        }
    }

    #[test]
    fn acquisition_sample_rate_validation_requires_distinct_sample_rates() {
        let alpha = synthetic_acquisition_sample_rate_case_fixture(
            4_092_000.0,
            "acquisition_sample_rate_validation_same_rate_alpha",
            0x4_092_001,
        );
        let beta = synthetic_acquisition_sample_rate_case_fixture(
            4_092_000.0,
            "acquisition_sample_rate_validation_same_rate_beta",
            0x4_092_002,
        );

        let report = validate_truth_guided_acquisition_sample_rates(&[alpha.case, beta.case], 2, 1);

        assert!(!report.pass, "{report:?}");
        assert_eq!(report.distinct_sample_rate_count, 1);
        assert_eq!(report.min_sample_rate_hz, 4_092_000.0);
        assert_eq!(report.max_sample_rate_hz, 4_092_000.0);
        assert!(report.points.iter().all(|point| point.pass), "{report:?}");
    }

    #[test]
    fn acquisition_sample_rate_validation_orders_profiles_by_rate_then_scenario_id() {
        let zeta = synthetic_acquisition_sample_rate_case_fixture(
            4_092_000.0,
            "acquisition_sample_rate_validation_zeta",
            0x4_092_003,
        );
        let beta = synthetic_acquisition_sample_rate_case_fixture(
            2_046_000.0,
            "acquisition_sample_rate_validation_beta",
            0x2_046_001,
        );
        let alpha = synthetic_acquisition_sample_rate_case_fixture(
            2_046_000.0,
            "acquisition_sample_rate_validation_alpha",
            0x2_046_002,
        );

        let report = validate_truth_guided_acquisition_sample_rates(
            &[zeta.case, beta.case, alpha.case],
            2,
            1,
        );
        let ordered_points = report
            .points
            .iter()
            .map(|point| (point.sample_rate_hz, point.scenario_id.as_str()))
            .collect::<Vec<_>>();

        assert_eq!(
            ordered_points,
            vec![
                (2_046_000.0, "acquisition_sample_rate_validation_alpha"),
                (2_046_000.0, "acquisition_sample_rate_validation_beta"),
                (4_092_000.0, "acquisition_sample_rate_validation_zeta"),
            ]
        );
    }

    struct SyntheticAcquisitionSampleRateCaseFixture {
        case: SyntheticAcquisitionSampleRateValidationCase<'static>,
        truth: &'static super::SyntheticIqTruthBundle,
    }

    fn synthetic_acquisition_sample_rate_case_fixture(
        sampling_freq_hz: f64,
        scenario_id: &str,
        seed: u64,
    ) -> SyntheticAcquisitionSampleRateCaseFixture {
        let config = Box::leak(Box::new(ReceiverPipelineConfig {
            sampling_freq_hz,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            acquisition_doppler_search_hz: 10_000,
            acquisition_doppler_step_hz: 500,
            ..ReceiverPipelineConfig::default()
        }));
        let scenario = SyntheticScenario {
            sample_rate_hz: config.sampling_freq_hz,
            intermediate_freq_hz: config.intermediate_freq_hz,
            receiver_clock_frequency_bias_hz: 0.0,
            duration_s: 0.04,
            seed,
            satellites: vec![
                SyntheticSignalParams {
                    sat: SatId { constellation: Constellation::Gps, prn: 3 },
                    doppler_hz: 750.0,
                    code_phase_chips: 200.25,
                    carrier_phase_rad: 0.0,
                    cn0_db_hz: 58.0,
                    data_bit_flip: true,
                },
                SyntheticSignalParams {
                    sat: SatId { constellation: Constellation::Gps, prn: 7 },
                    doppler_hz: -1_000.0,
                    code_phase_chips: 321.5,
                    carrier_phase_rad: 0.2,
                    cn0_db_hz: 52.0,
                    data_bit_flip: false,
                },
            ],
            ephemerides: Vec::new(),
            id: scenario_id.to_string(),
        };
        let frame = generate_l1_ca_multi(config, &scenario);
        let bundle = build_iq16_capture_bundle(
            &scenario.id,
            &scenario,
            &frame,
            "2026-07-09T00:00:00Z",
            Some("synthetic acquisition sample-rate validation".to_string()),
        );
        let scaled_frame = Box::leak(Box::new(SamplesFrame::new(
            frame.t0,
            frame.dt_s,
            frame.iq.iter().map(|sample| *sample * bundle.truth.output_scale_applied).collect(),
        )));
        let truth = Box::leak(Box::new(bundle.truth));

        SyntheticAcquisitionSampleRateCaseFixture {
            case: SyntheticAcquisitionSampleRateValidationCase {
                config,
                frame: scaled_frame,
                truth,
            },
            truth,
        }
    }

    #[test]
    fn signal_only_streaming_source_matches_signal_only_frame_generation() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 1_023_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            ..ReceiverPipelineConfig::default()
        };
        let scenario = SyntheticScenario {
            sample_rate_hz: config.sampling_freq_hz,
            intermediate_freq_hz: config.intermediate_freq_hz,
            receiver_clock_frequency_bias_hz: 0.0,
            duration_s: 0.004,
            seed: 17,
            satellites: vec![
                SyntheticSignalParams {
                    sat: SatId { constellation: Constellation::Gps, prn: 3 },
                    doppler_hz: -120.0,
                    code_phase_chips: 64.0,
                    carrier_phase_rad: 0.25,
                    cn0_db_hz: 55.0,
                    data_bit_flip: false,
                },
                SyntheticSignalParams {
                    sat: SatId { constellation: Constellation::Gps, prn: 11 },
                    doppler_hz: 180.0,
                    code_phase_chips: 288.5,
                    carrier_phase_rad: 0.75,
                    cn0_db_hz: 55.0,
                    data_bit_flip: false,
                },
            ],
            ephemerides: Vec::new(),
            id: "signal-only-streaming".to_string(),
        };
        let expected = super::generate_l1_ca_multi_signal_only(&config, &scenario);
        let mut source = super::SyntheticSignalSource::new_signal_only(&config, &scenario);
        let streamed = source
            .next_frame(expected.len())
            .expect("streaming signal-only frame")
            .expect("non-empty signal-only source");

        assert_eq!(streamed.t0, expected.t0);
        assert_eq!(streamed.dt_s, expected.dt_s);
        assert_eq!(streamed.iq, expected.iq);
        assert!(source.is_done());
    }
}
