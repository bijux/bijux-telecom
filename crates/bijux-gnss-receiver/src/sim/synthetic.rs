#![allow(missing_docs)]

use std::f32::consts::TAU;

use num_complex::Complex;

use bijux_gnss_core::api::{
    AcqHypothesis, AcqResult, Constellation, Hertz, ReceiverSampleTrace, SampleClock, SampleTime,
    SamplesFrame, SatId, Seconds, SignalBand,
};
use bijux_gnss_signal::api::SignalSource;

use crate::engine::receiver_config::ReceiverPipelineConfig;
use crate::io::data::SampleSourceError;
use crate::pipeline::doppler::carrier_hz_from_doppler_hz;
use bijux_gnss_nav::api::GpsEphemeris;
use bijux_gnss_signal::api::{
    advance_code_phase_seconds, code_value_at_phase, generate_ca_code, samples_per_code,
    IqSampleFormat, Prn, RawIqMetadata,
};
use serde::{Deserialize, Serialize};

const SYNTHETIC_IQ_TRUTH_SCHEMA_VERSION: u32 = 3;
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
        let stable = epoch.lock
            && epoch.lock_state == "tracking"
            && epoch.pll_lock
            && epoch.fll_lock
            && !epoch.cycle_slip
            && epoch.lock_state_reason.as_deref() != Some("lock_lost");
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
        let sample_count = (scenario.duration_s * config.sampling_freq_hz).round() as usize;
        let noise_std = SYNTHETIC_NOISE_STD_PER_COMPONENT;

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
            let noise_i = self.rng.next_gaussian() * self.noise_std;
            let noise_q = self.rng.next_gaussian() * self.noise_std;
            iq.push(sample + Complex::new(noise_i, noise_q));
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
    let start_s = frame.t0.sample_index as f64 / frame.t0.sample_rate_hz;
    let chip_phase = advance_code_phase_seconds(
        code_phase_chips,
        config.code_freq_basis_hz,
        start_s,
        config.code_length,
    )
    .expect("synthetic epoch alignment requires a valid code phase model");
    let samples_per_chip = frame.t0.sample_rate_hz / config.code_freq_basis_hz;
    chip_phase * samples_per_chip
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
        synthetic_tracking_sensitivity_report, validate_truth_guided_acquisition_code_phase,
        validate_truth_guided_acquisition_code_phase_refinement,
        validate_truth_guided_acquisition_coherent_integration,
        validate_truth_guided_acquisition_doppler,
        validate_truth_guided_acquisition_receiver_clock_offset,
        validate_truth_guided_acquisition_sample_rates, validate_truth_guided_cn0,
        wrapped_code_phase_error_samples, wrapped_code_phase_error_samples_f64, SatState,
        SyntheticAcquisitionDetectionRateCase, SyntheticAcquisitionFalseAlarmRateCase,
        SyntheticAcquisitionSampleRateValidationCase, SyntheticDopplerRampParams,
        SyntheticFadeWindow, SyntheticNavBitMode, SyntheticPhaseWindow, SyntheticScenario,
        SyntheticSignalParams, SyntheticSignalSource, SyntheticTrackingLockRateCase,
        SyntheticTrackingSensitivityTrial, SYNTHETIC_COMPLEX_NOISE_POWER,
        SYNTHETIC_NOISE_STD_PER_COMPONENT,
    };
    use crate::engine::receiver_config::ReceiverPipelineConfig;
    use bijux_gnss_core::api::{Constellation, SampleTime, SamplesFrame, SatId, Seconds};
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
}
