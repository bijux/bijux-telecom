#![allow(missing_docs)]
//! Observation error model assumptions:
//! - code, carrier, Doppler, and C/N0 uncertainty must come from tracking evidence.
//! - observations without defensible tracking uncertainty are marked missing rather than weighted.
//! - code-carrier divergence is decomposed before multipath is inferred.
//! - clock_error_m comes from configured receiver-clock bias uncertainty.

use bijux_gnss_core::api::{
    CarrierPhaseArc, CodeCarrierDivergence, ConventionsConfig, CycleSlipDecisionEvidence,
    CycleSlipDetector, CycleSlipDetectorEvidence, Cycles, DiagnosticEvent, DiagnosticSeverity,
    GpsTime, LockFlags, Meters, ObsDecisionArtifact, ObsEpoch, ObsEpochManifest, ObsMetadata,
    ObsSatellite, ObservationEpochDecision, ObservationMeasurementCovariance, ObservationStatus,
    ReceiverRole, ReceiverSampleTrace, SatId, SatObservationDecision, Seconds, SigId, SignalBand,
    TrackEpoch,
};

use crate::engine::receiver_config::ReceiverPipelineConfig;
use crate::pipeline::doppler::doppler_hz_from_carrier_hz;
use crate::pipeline::hatch::HatchFilterState;
use crate::pipeline::observations::code_period_ambiguity::{
    apply_grouped_integer_code_period_ambiguities, code_period_ambiguity_input_from_tracking_epoch,
    CodePeriodAmbiguityInput,
};
use crate::pipeline::observations::labels::{
    carrier_phase_continuity_label, doppler_model_label, observation_status_label,
    observation_support_label, observation_uncertainty_label,
    pseudorange_model_has_resolved_alignment,
};
use crate::pipeline::observations::pseudorange_timing::pseudorange_from_tracking_epoch;
use crate::pipeline::observations::receiver_clock::{
    observation_receiver_clock, receiver_clock_carrier_phase_cycles,
};
use crate::pipeline::observations::signal_model::{
    observation_signal_model, ObservationSignalModel,
};
use crate::pipeline::observations::variance::{
    apply_variance_evidence_status, evidence_sigma_from_variance, finite_sigma, finite_value,
    has_variance_evidence, observation_error_model, observation_variance_evidence,
};
use crate::pipeline::tracking::TrackingResult;
use crate::pipeline::{StepReport, StepStats};
use bijux_gnss_signal::api::{
    samples_per_code, signal_cycles_to_meters, signal_wavelength_m, validate_obs_epochs,
};
use serde::{Deserialize, Serialize};

#[cfg(test)]
use bijux_gnss_core::api::{
    Constellation, GlonassFrequencyChannel, Hertz, ObsSignalTiming, SignalCode, SignalSpec,
    GPS_L1_CA_CARRIER_HZ,
};
#[cfg(test)]
use bijux_gnss_signal::api::glonass_l1_carrier_hz;
#[cfg(test)]
use bijux_gnss_signal::api::{
    signal_spec_beidou_b2i, signal_spec_galileo_e5a, signal_spec_gps_l1_ca, signal_spec_gps_l2c,
    signal_spec_gps_l5, signal_spec_gps_l5_q,
};

mod code_period_ambiguity;
mod labels;
mod pseudorange_timing;
mod receiver_clock;
mod signal_model;
mod variance;

#[cfg(test)]
use code_period_ambiguity::{
    resolve_integer_code_period_ambiguities, CODE_PERIOD_AMBIGUITY_EPS_S,
    CODE_PERIOD_AMBIGUITY_NON_UNIQUE,
};
#[cfg(test)]
use pseudorange_timing::resolve_pseudorange_from_transmit_time;
pub(crate) use signal_model::supports_observation_signal;
#[cfg(test)]
use signal_model::{tracked_signal_center_hz, tracked_signal_code_for_band};

const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;
const OBS_WEAK_CN0_DBHZ: f64 = 25.0;
#[derive(Debug, Clone)]
struct CarrierPhaseArcState {
    phase_cycles: f64,
    doppler_hz: f64,
    last_sample_index: u64,
    arc_start_epoch_idx: u64,
    arc_start_sample_index: u64,
    arc_start_reason: String,
    pending_reset: Option<CarrierPhaseContinuity>,
    pending_reset_reason: Option<String>,
    initialized: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum CarrierPhaseContinuity {
    Unusable,
    ArcStart,
    Continuous,
    Coasted,
    ResetAfterCycleSlip,
    ResetAfterUnlock,
    ResetAfterReacquisition,
    ResetAfterDiscontinuity,
}

#[derive(Debug, Clone)]
struct CarrierPhaseObservation {
    phase_cycles: Cycles,
    cycle_slip: bool,
    cycle_slip_reason: Option<String>,
    cycle_slip_evidence: CycleSlipDecisionEvidence,
    continuity: CarrierPhaseContinuity,
    arc_start_epoch_idx: u64,
    arc_start_sample_index: u64,
    arc: CarrierPhaseArc,
}

const BASE_CARRIER_PHASE_DISCONTINUITY_THRESHOLD_CYCLES: f64 = 0.25;
const BASE_CARRIER_PHASE_RESIDUAL_THRESHOLD_CYCLES: f64 = 0.15;
const BASE_DOPPLER_JUMP_THRESHOLD_HZ: f64 = 150.0;
const CYCLE_SLIP_DETECTION_PROBABILITY_BUDGET: f64 = 0.99;
const CYCLE_SLIP_FALSE_ALARM_PROBABILITY_BUDGET: f64 = 1.0e-3;
const GEOMETRY_FREE_IONOSPHERE_DELTA_M: f64 = 0.01;
const GEOMETRY_FREE_CYCLE_SLIP_JUMP_M: f64 = 0.10;
const MELBOURNE_WUBBENA_WIDE_LANE_SLIP_JUMP_CYCLES: f64 = 0.5;
const PSEUDORANGE_RESIDUAL_REFERENCE_MODEL: &str = "signal_travel_time_from_gps_anchor";
const CARRIER_PHASE_RESIDUAL_REFERENCE_MODEL: &str = "previous_continuous_phase_prediction";
const CN0_RESIDUAL_REFERENCE_MODEL: &str = "tracking_epoch_cn0";

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct ObservationResidualValue {
    pub raw: f64,
    pub corrected: f64,
    pub expected: Option<f64>,
    pub residual: Option<f64>,
    pub sigma: Option<f64>,
    pub reference_model: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct ObservationResidualSatellite {
    pub signal_id: SigId,
    pub accepted: bool,
    pub observation_status: ObservationStatus,
    pub observation_reject_reasons: Vec<String>,
    pub pseudorange_m: ObservationResidualValue,
    pub carrier_phase_cycles: ObservationResidualValue,
    pub doppler_hz: ObservationResidualValue,
    pub cn0_dbhz: ObservationResidualValue,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct ObservationResidualEpochReport {
    pub artifact_id: String,
    pub epoch_id: String,
    pub epoch_idx: u64,
    pub source_time: ReceiverSampleTrace,
    pub accepted: bool,
    pub decision: ObservationEpochDecision,
    pub decision_reason: Option<String>,
    pub sats: Vec<ObservationResidualSatellite>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ObservationMeasurementQualitySatellite {
    pub signal_id: SigId,
    pub accepted: bool,
    pub observation_status: ObservationStatus,
    pub observation_reject_reasons: Vec<String>,
    pub cn0_dbhz: f64,
    pub pseudorange_sigma_m: Option<f64>,
    pub carrier_phase_sigma_cycles: Option<f64>,
    pub doppler_sigma_hz: Option<f64>,
    pub cn0_sigma_dbhz: Option<f64>,
    pub measurement_covariance: Option<ObservationMeasurementCovariance>,
    pub code_carrier_divergence: Option<CodeCarrierDivergence>,
    pub cycle_slip_evidence: Option<CycleSlipDecisionEvidence>,
    pub carrier_phase_arc: Option<CarrierPhaseArc>,
    pub lock_flags: LockFlags,
    pub observation_lock_state: String,
    pub observation_lock_reason: Option<String>,
    pub tracking_lock_quality: f64,
    pub cycle_slip: bool,
    pub cycle_slip_reason: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ObservationMeasurementQualityEpochReport {
    pub artifact_id: String,
    pub epoch_id: String,
    pub epoch_idx: u64,
    pub source_time: ReceiverSampleTrace,
    pub accepted: bool,
    pub decision: ObservationEpochDecision,
    pub decision_reason: Option<String>,
    pub sats: Vec<ObservationMeasurementQualitySatellite>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ObservationPipelineArtifacts {
    pub epochs: Vec<ObsEpoch>,
    pub residuals: Vec<ObservationResidualEpochReport>,
    pub measurement_quality: Vec<ObservationMeasurementQualityEpochReport>,
}

#[derive(Debug, Clone)]
struct RawObservationSnapshot {
    raw_pseudorange_m: f64,
    raw_carrier_phase_cycles: f64,
    raw_doppler_hz: f64,
    raw_cn0_dbhz: f64,
}

#[derive(Debug, Clone, Copy)]
struct IonosphereDelayEvidence {
    signal_id: SigId,
    delay_m: f64,
}

#[derive(Debug, Default)]
struct CodeCarrierDivergenceState {
    ionosphere_delay_m_by_signal: std::collections::HashMap<SigId, f64>,
}

#[derive(Debug, Clone, Copy)]
struct CarrierPhaseReferenceState {
    corrected_cycles: f64,
    doppler_hz: f64,
    sample_index: u64,
    sample_rate_hz: f64,
}

impl ObservationResidualValue {
    fn from_measurement(
        raw: f64,
        corrected: f64,
        expected: Option<f64>,
        sigma: Option<f64>,
        reference_model: Option<&str>,
    ) -> Self {
        let expected = finite_value(expected);
        let sigma = finite_sigma(sigma);
        Self {
            raw,
            corrected,
            expected,
            residual: expected.map(|reference| corrected - reference),
            sigma,
            reference_model: reference_model.map(str::to_string),
        }
    }
}

impl ObservationMeasurementQualitySatellite {
    fn from_satellite(sat: &ObsSatellite) -> Self {
        Self {
            signal_id: sat.signal_id,
            accepted: sat.observation_status == ObservationStatus::Accepted,
            observation_status: sat.observation_status,
            observation_reject_reasons: sat.observation_reject_reasons.clone(),
            cn0_dbhz: sat.cn0_dbhz,
            pseudorange_sigma_m: evidence_sigma_from_variance(sat, sat.pseudorange_var_m2),
            carrier_phase_sigma_cycles: evidence_sigma_from_variance(
                sat,
                sat.carrier_phase_var_cycles2,
            ),
            doppler_sigma_hz: evidence_sigma_from_variance(sat, sat.doppler_var_hz2),
            cn0_sigma_dbhz: sat
                .metadata
                .tracking_uncertainty
                .as_ref()
                .and_then(|uncertainty| finite_sigma(Some(uncertainty.cn0_dbhz))),
            measurement_covariance: sat.measurement_covariance(),
            code_carrier_divergence: sat.metadata.code_carrier_divergence,
            cycle_slip_evidence: sat.metadata.cycle_slip_evidence.clone(),
            carrier_phase_arc: sat.metadata.carrier_phase_arc.clone(),
            lock_flags: sat.lock_flags,
            observation_lock_state: sat.metadata.observation_lock_state.clone(),
            observation_lock_reason: sat.metadata.observation_lock_reason.clone(),
            tracking_lock_quality: sat.metadata.tracking_lock_quality,
            cycle_slip: sat.lock_flags.cycle_slip,
            cycle_slip_reason: cycle_slip_reason(sat),
        }
    }
}

impl ObservationMeasurementQualityEpochReport {
    fn from_epoch(epoch: &ObsEpoch) -> Self {
        let observation_artifact_id = epoch
            .manifest
            .as_ref()
            .map(|manifest| manifest.artifact_id.clone())
            .unwrap_or_else(|| format!("obs-epoch-{:010}", epoch.epoch_idx));
        let epoch_id =
            epoch.manifest.as_ref().map(|manifest| manifest.epoch_id.clone()).unwrap_or_else(
                || observation_epoch_id(epoch.epoch_idx, epoch.source_time.sample_index),
            );
        Self {
            artifact_id: format!("observation-measurement-quality-{observation_artifact_id}"),
            epoch_id,
            epoch_idx: epoch.epoch_idx,
            source_time: epoch.source_time,
            accepted: epoch.decision == ObservationEpochDecision::Accepted,
            decision: epoch.decision,
            decision_reason: epoch.decision_reason.clone(),
            sats: epoch
                .sats
                .iter()
                .map(ObservationMeasurementQualitySatellite::from_satellite)
                .collect(),
        }
    }
}

pub fn observation_measurement_quality_from_epochs(
    epochs: &[ObsEpoch],
) -> Vec<ObservationMeasurementQualityEpochReport> {
    epochs.iter().map(ObservationMeasurementQualityEpochReport::from_epoch).collect()
}

pub fn observations_from_tracking(
    config: &ReceiverPipelineConfig,
    epochs: &[TrackEpoch],
) -> (Vec<ObsEpoch>, Vec<DiagnosticEvent>) {
    observations_from_tracking_with_provenance(config, None, None, epochs)
}

fn observations_from_tracking_with_provenance(
    config: &ReceiverPipelineConfig,
    capture_start_gps_time: Option<GpsTime>,
    track: Option<&TrackingResult>,
    epochs: &[TrackEpoch],
) -> (Vec<ObsEpoch>, Vec<DiagnosticEvent>) {
    if epochs.is_empty() {
        return (Vec::new(), Vec::new());
    }

    let meters_per_sample = SPEED_OF_LIGHT_MPS / config.sampling_freq_hz;

    let mut carrier_phase_state = CarrierPhaseArcState {
        phase_cycles: 0.0,
        doppler_hz: 0.0,
        last_sample_index: 0,
        arc_start_epoch_idx: 0,
        arc_start_sample_index: 0,
        arc_start_reason: String::new(),
        pending_reset: None,
        pending_reset_reason: None,
        initialized: false,
    };

    let mut out = Vec::with_capacity(epochs.len());
    let mut diagnostics = Vec::new();
    let mut last_sample_index = None;
    let mut last_locked_sample = None;
    let receiver_clock = observation_receiver_clock(config);

    for epoch in epochs {
        let signal_model = observation_signal_model(config, epoch);
        let (time_tag_source, time_tag_sample_index) =
            tracking_time_tag(epoch, &mut last_locked_sample);
        let sample_time_s = epoch.sample_index as f64 / config.sampling_freq_hz;
        let t_rx_s = Seconds(sample_time_s + receiver_clock.bias_s);

        let expected_step =
            observation_epoch_interval_samples(config, &signal_model, epoch.signal_band);
        let discontinuity = match last_sample_index {
            Some(prev) => epoch.sample_index.saturating_sub(prev) != expected_step,
            None => false,
        };
        if let Some(prev) = last_sample_index {
            if epoch.sample_index < prev {
                diagnostics.push(
                    DiagnosticEvent::new(
                        DiagnosticSeverity::Warning,
                        "OBS_TIME_BACKWARDS",
                        format!("sample index went backwards ({} -> {})", prev, epoch.sample_index),
                    )
                    .with_context("epoch", epoch.epoch.index.to_string())
                    .with_context("stage", "observations"),
                );
            }
        }
        if discontinuity {
            diagnostics.push(
                DiagnosticEvent::new(
                    DiagnosticSeverity::Warning,
                    "OBS_TIME_DISCONTINUITY",
                    format!("discontinuity at sample index {}", epoch.sample_index),
                )
                .with_context("epoch", epoch.epoch.index.to_string())
                .with_context("stage", "observations"),
            );
        }
        last_sample_index = Some(epoch.sample_index);

        let receive_gps_time = capture_start_gps_time
            .map(|gps_time| gps_time.offset_seconds(sample_time_s + receiver_clock.bias_s));
        let pseudorange = pseudorange_from_tracking_epoch(
            epoch,
            signal_model.samples_per_chip,
            signal_model.signal.code_rate_hz,
            signal_model.code_length as f64,
            receive_gps_time,
            receiver_clock.bias_s,
        );

        let tracked_carrier_phase_cycles = epoch.carrier_phase_cycles.0
            + receiver_clock_carrier_phase_cycles(&receiver_clock, signal_model.signal);
        let doppler_hz = bijux_gnss_core::api::Hertz(
            doppler_hz_from_carrier_hz(signal_model.carrier_reference_hz, epoch.carrier_hz.0)
                + receiver_clock.frequency_bias_hz,
        );
        let carrier_phase = carrier_phase_observation(
            epoch,
            tracked_carrier_phase_cycles,
            doppler_hz.0,
            config.sampling_freq_hz,
            discontinuity,
            signal_model.signal_id,
            &mut carrier_phase_state,
        );

        let cn0_dbhz = normalize_observation_cn0_dbhz(epoch.cn0_dbhz);
        let lock_quality = tracking_lock_quality(epoch, carrier_phase.cycle_slip);
        let tracking_uncertainty = epoch.tracking_uncertainty.clone();
        let variance_evidence =
            observation_variance_evidence(epoch, meters_per_sample, &receiver_clock);
        let (pseudorange_var_m2, carrier_phase_var_cycles2, doppler_var_hz2, error_model) =
            match &variance_evidence {
                Ok(evidence) => (
                    evidence.pseudorange_var_m2,
                    evidence.carrier_phase_var_cycles2,
                    evidence.doppler_var_hz2,
                    Some(evidence.pseudorange_error_m.clone()),
                ),
                Err(_) => (0.0, 0.0, 0.0, None),
            };
        let (mut observation_status, mut observation_reject_reasons) =
            classify_observation_status(epoch, cn0_dbhz);
        apply_variance_evidence_status(
            &mut observation_status,
            &mut observation_reject_reasons,
            variance_evidence.as_ref().err(),
        );
        let observation_lock_state =
            observation_lock_state(epoch, carrier_phase.cycle_slip).to_string();
        let observation_lock_reason = observation_lock_reason(
            epoch,
            carrier_phase.cycle_slip,
            carrier_phase.cycle_slip_reason.as_deref(),
        );
        let signal = signal_model.signal;
        let observation_epoch_id = observation_epoch_id(epoch.epoch.index, epoch.sample_index);
        let mut sat = ObsSatellite {
            signal_id: signal_model.signal_id,
            pseudorange_m: pseudorange.pseudorange_m,
            pseudorange_var_m2,
            carrier_phase_cycles: carrier_phase.phase_cycles,
            carrier_phase_var_cycles2,
            doppler_hz,
            doppler_var_hz2,
            cn0_dbhz,
            lock_flags: LockFlags {
                code_lock: epoch.lock,
                carrier_lock: epoch.pll_lock,
                bit_lock: false,
                cycle_slip: carrier_phase.cycle_slip,
            },
            multipath_suspect: false,
            observation_status,
            observation_reject_reasons: observation_reject_reasons.clone(),
            elevation_deg: None,
            azimuth_deg: None,
            weight: None,
            timing: pseudorange.timing,
            error_model,
            metadata: ObsMetadata {
                tracking_mode: "scalar".to_string(),
                integration_ms: config.tracking_params(epoch.signal_band).integration_ms,
                lock_quality,
                smoothing_window: 0,
                smoothing_age: 0,
                smoothing_resets: 0,
                signal,
                acquisition_hypothesis: track
                    .map(|t| t.acquisition_hypothesis.clone())
                    .unwrap_or_default(),
                acquisition_score: track.map(|t| t.acquisition_score).unwrap_or_default(),
                acquisition_code_phase_samples: track
                    .map(|t| t.acquisition_code_phase_samples)
                    .unwrap_or_default(),
                acquisition_carrier_hz: track.map(|t| t.acquisition_carrier_hz).unwrap_or_default(),
                acq_to_track_state: track.map(|t| t.acq_to_track_state.clone()).unwrap_or_default(),
                tracking_state: epoch.lock_state.clone(),
                tracking_lock_state: observation_lock_state.clone(),
                observation_lock_state,
                observation_lock_reason,
                tracking_lock_quality: lock_quality,
                observation_status: observation_status_label(observation_status).to_string(),
                observation_reject_reasons,
                observation_epoch_id: observation_epoch_id.clone(),
                observation_support_class: observation_support_label(
                    observation_status,
                    pseudorange.alignment_resolved,
                )
                .to_string(),
                observation_uncertainty_class: observation_uncertainty_label(
                    cn0_dbhz,
                    variance_evidence.as_ref().is_ok(),
                )
                .to_string(),
                pseudorange_model: pseudorange.model.to_string(),
                pseudorange_time_source: pseudorange.time_source.unwrap_or_default(),
                pseudorange_integer_code_periods: pseudorange.integer_code_periods,
                pseudorange_code_delay_s: pseudorange.code_delay_s,
                carrier_phase_model: "tracked_carrier_cycles".to_string(),
                doppler_model: doppler_model_label().to_string(),
                carrier_phase_continuity: carrier_phase_continuity_label(carrier_phase.continuity)
                    .to_string(),
                carrier_phase_arc_start_epoch_idx: carrier_phase.arc_start_epoch_idx,
                carrier_phase_arc_start_sample_index: carrier_phase.arc_start_sample_index,
                carrier_phase_arc: Some(carrier_phase.arc.clone()),
                signal_delay_alignment_source: pseudorange.alignment_source.unwrap_or_default(),
                time_tag_source,
                time_tag_sample_index,
                time_tag_sample_rate_hz: config.sampling_freq_hz,
                receiver_clock_bias_s: Seconds(receiver_clock.bias_s),
                receiver_clock_frequency_bias_hz: receiver_clock.frequency_bias_hz,
                receiver_clock_bias_sigma_s: Seconds(receiver_clock.bias_sigma_s),
                receiver_clock_source: receiver_clock.source.clone(),
                tracking_uncertainty,
                code_carrier_divergence: None,
                cycle_slip_evidence: Some(carrier_phase.cycle_slip_evidence.clone()),
            },
        };
        apply_cycle_slip_surface(&mut sat, carrier_phase.cycle_slip_reason.as_deref());
        apply_pseudorange_physics_rejection(&mut sat);
        sat.metadata.observation_status =
            observation_status_label(sat.observation_status).to_string();
        sat.metadata.observation_reject_reasons = sat.observation_reject_reasons.clone();
        sat.metadata.observation_support_class =
            observation_support_label(sat.observation_status, pseudorange.alignment_resolved)
                .to_string();

        let mut epoch = ObsEpoch {
            t_rx_s,
            source_time: epoch.source_time,
            gps_week: receive_gps_time.map(|gps_time| gps_time.week),
            tow_s: receive_gps_time.map(|gps_time| Seconds(gps_time.tow_s)),
            epoch_idx: epoch.epoch.index,
            discontinuity,
            valid: true,
            processing_ms: None,
            role: ReceiverRole::Rover,
            sats: vec![sat],
            decision: ObservationEpochDecision::Accepted,
            decision_reason: None,
            manifest: None,
        };
        let events = bijux_gnss_core::api::check_obs_epoch_sanity(&epoch);
        if events
            .iter()
            .any(|e| matches!(e.severity, bijux_gnss_core::api::DiagnosticSeverity::Error))
        {
            epoch.valid = false;
            diagnostics.extend(events);
        }
        apply_epoch_decision(&mut epoch);
        out.push(epoch);
    }

    (out, diagnostics)
}

pub fn observations_from_tracking_results(
    config: &ReceiverPipelineConfig,
    tracks: &[TrackingResult],
    hatch_window: u32,
) -> StepReport<Vec<ObsEpoch>> {
    let report = observation_artifacts_from_tracking_results(config, tracks, hatch_window);
    let StepReport { output, events, stats } = report;
    StepReport { output: output.epochs, events, stats }
}

pub fn observations_from_tracking_results_with_gps_anchor(
    config: &ReceiverPipelineConfig,
    capture_start_gps_time: Option<GpsTime>,
    tracks: &[TrackingResult],
    hatch_window: u32,
) -> StepReport<Vec<ObsEpoch>> {
    let report = observation_artifacts_from_tracking_results_with_gps_anchor(
        config,
        capture_start_gps_time,
        tracks,
        hatch_window,
    );
    let StepReport { output, events, stats } = report;
    StepReport { output: output.epochs, events, stats }
}

pub fn observation_residuals_from_tracking_results(
    config: &ReceiverPipelineConfig,
    tracks: &[TrackingResult],
    hatch_window: u32,
) -> StepReport<Vec<ObservationResidualEpochReport>> {
    let report = observation_artifacts_from_tracking_results(config, tracks, hatch_window);
    let StepReport { output, events, stats } = report;
    StepReport { output: output.residuals, events, stats }
}

pub fn observation_residuals_from_tracking_results_with_gps_anchor(
    config: &ReceiverPipelineConfig,
    capture_start_gps_time: Option<GpsTime>,
    tracks: &[TrackingResult],
    hatch_window: u32,
) -> StepReport<Vec<ObservationResidualEpochReport>> {
    let report = observation_artifacts_from_tracking_results_with_gps_anchor(
        config,
        capture_start_gps_time,
        tracks,
        hatch_window,
    );
    let StepReport { output, events, stats } = report;
    StepReport { output: output.residuals, events, stats }
}

pub fn observation_measurement_quality_from_tracking_results(
    config: &ReceiverPipelineConfig,
    tracks: &[TrackingResult],
    hatch_window: u32,
) -> StepReport<Vec<ObservationMeasurementQualityEpochReport>> {
    observation_measurement_quality_from_tracking_results_with_gps_anchor(
        config,
        None,
        tracks,
        hatch_window,
    )
}

pub fn observation_measurement_quality_from_tracking_results_with_gps_anchor(
    config: &ReceiverPipelineConfig,
    capture_start_gps_time: Option<GpsTime>,
    tracks: &[TrackingResult],
    hatch_window: u32,
) -> StepReport<Vec<ObservationMeasurementQualityEpochReport>> {
    let report = observation_artifacts_from_tracking_results_with_gps_anchor(
        config,
        capture_start_gps_time,
        tracks,
        hatch_window,
    );
    let StepReport { output, events, stats } = report;
    StepReport { output: output.measurement_quality, events, stats }
}

pub fn observation_artifacts_from_tracking_results(
    config: &ReceiverPipelineConfig,
    tracks: &[TrackingResult],
    hatch_window: u32,
) -> StepReport<ObservationPipelineArtifacts> {
    observation_artifacts_from_tracking_results_with_gps_anchor(config, None, tracks, hatch_window)
}

pub fn observation_artifacts_from_tracking_results_with_gps_anchor(
    config: &ReceiverPipelineConfig,
    capture_start_gps_time: Option<GpsTime>,
    tracks: &[TrackingResult],
    hatch_window: u32,
) -> StepReport<ObservationPipelineArtifacts> {
    use std::collections::{BTreeMap, HashMap};

    let mut by_epoch: BTreeMap<u64, ObsEpoch> = BTreeMap::new();
    let mut hatch: HashMap<SigId, HatchFilterState> = HashMap::new();
    let mut raw_snapshots: HashMap<String, RawObservationSnapshot> = HashMap::new();
    let mut ambiguity_inputs: HashMap<String, CodePeriodAmbiguityInput> = HashMap::new();
    let receiver_clock = observation_receiver_clock(config);
    let mut diagnostics = Vec::new();
    for track in tracks {
        for epoch in &track.epochs {
            if let Some(input) = code_period_ambiguity_input_from_tracking_epoch(
                config,
                capture_start_gps_time,
                epoch,
            ) {
                ambiguity_inputs
                    .insert(observation_snapshot_key(epoch.epoch.index, input.signal_id), input);
            }
        }
        let (obs, mut events) = observations_from_tracking_with_provenance(
            config,
            capture_start_gps_time,
            Some(track),
            &track.epochs,
        );
        diagnostics.append(&mut events);
        for epoch in obs {
            let entry = by_epoch.entry(epoch.epoch_idx).or_insert_with(|| ObsEpoch {
                t_rx_s: epoch.t_rx_s,
                source_time: epoch.source_time,
                gps_week: epoch.gps_week,
                tow_s: epoch.tow_s,
                epoch_idx: epoch.epoch_idx,
                discontinuity: epoch.discontinuity,
                valid: true,
                processing_ms: None,
                role: epoch.role,
                sats: Vec::new(),
                decision: ObservationEpochDecision::Accepted,
                decision_reason: None,
                manifest: None,
            });
            entry.discontinuity |= epoch.discontinuity;
            let time_mismatch_reasons =
                grouped_epoch_time_mismatch_reasons(entry, &epoch).unwrap_or_default();
            let incoming_epoch_idx = epoch.epoch_idx;
            let incoming_source_time = epoch.source_time;
            let incoming_t_rx_s = epoch.t_rx_s;
            for mut sat in epoch.sats {
                let snapshot_key = observation_snapshot_key(incoming_epoch_idx, sat.signal_id);
                if !time_mismatch_reasons.is_empty() {
                    raw_snapshots
                        .insert(snapshot_key, raw_observation_snapshot(&sat, sat.pseudorange_m.0));
                    mark_grouped_epoch_time_mismatch(&mut sat, &time_mismatch_reasons);
                    diagnostics.push(grouped_epoch_time_mismatch_diagnostic(
                        entry,
                        incoming_epoch_idx,
                        incoming_source_time,
                        incoming_t_rx_s,
                        &sat,
                    ));
                    entry.sats.push(sat);
                    continue;
                }
                let lambda_m = signal_wavelength_m(sat.metadata.signal).0;
                let state = hatch.entry(sat.signal_id).or_default();
                if !sat.lock_flags.code_lock {
                    raw_snapshots
                        .insert(snapshot_key, raw_observation_snapshot(&sat, sat.pseudorange_m.0));
                    state.clear_arc();
                    sat.metadata.smoothing_window = hatch_window;
                    sat.metadata.smoothing_age = 0;
                    sat.metadata.smoothing_resets = state.reset_count();
                    sat.error_model = observation_error_model(&sat, 0.0);
                    apply_pseudorange_physics_rejection(&mut sat);
                    entry.sats.push(sat);
                    continue;
                }
                let supports_code_carrier_slip_detection =
                    pseudorange_model_has_resolved_alignment(&sat.metadata.pseudorange_model);
                let raw_pseudorange_m = sat.pseudorange_m.0;
                raw_snapshots
                    .insert(snapshot_key, raw_observation_snapshot(&sat, raw_pseudorange_m));
                let Some(carrier_phase_arc_id) = sat
                    .metadata
                    .carrier_phase_arc
                    .as_ref()
                    .filter(|arc| arc.valid_for_smoothing)
                    .map(|arc| arc.id.clone())
                else {
                    state.clear_arc();
                    sat.metadata.smoothing_window = hatch_window;
                    sat.metadata.smoothing_age = 0;
                    sat.metadata.smoothing_resets = state.reset_count();
                    sat.error_model = observation_error_model(&sat, 0.0);
                    apply_pseudorange_physics_rejection(&mut sat);
                    entry.sats.push(sat);
                    continue;
                };
                state.align_carrier_phase_arc(&carrier_phase_arc_id);
                let raw_divergence_m = raw_pseudorange_m
                    - signal_cycles_to_meters(sat.carrier_phase_cycles, sat.metadata.signal).0;
                let threshold_m = slip_threshold_m(sat.cn0_dbhz, sat.elevation_deg);
                let divergence_delta_m = state.divergence_delta_m(raw_divergence_m).unwrap_or(0.0);
                let divergence_jump = divergence_delta_m.abs();
                let mut smoothing_cycle_slip_reason = None;
                if supports_code_carrier_slip_detection && divergence_jump > threshold_m {
                    smoothing_cycle_slip_reason = Some("code_carrier_divergence");
                }
                if supports_code_carrier_slip_detection {
                    upsert_cycle_slip_contributor(
                        &mut sat,
                        code_carrier_divergence_evidence(
                            smoothing_cycle_slip_reason.is_some(),
                            divergence_jump,
                            threshold_m,
                        ),
                    );
                }
                apply_cycle_slip_surface(&mut sat, smoothing_cycle_slip_reason);
                if sat.lock_flags.cycle_slip {
                    state.clear_arc();
                    state.align_carrier_phase_arc(&carrier_phase_arc_id);
                }
                let smoothing = state.observe(
                    raw_pseudorange_m,
                    sat.carrier_phase_cycles.0,
                    raw_divergence_m,
                    lambda_m,
                    hatch_window,
                );
                sat.pseudorange_m = Meters(smoothing.smoothed_pseudorange_m);
                sat.metadata.smoothing_window = hatch_window;
                sat.metadata.smoothing_age = smoothing.smoothing_age_epochs;
                sat.metadata.smoothing_resets = smoothing.reset_count;
                let smoothing_transient_m = smoothing.smoothed_pseudorange_m - raw_pseudorange_m;
                sat.metadata.code_carrier_divergence = Some(CodeCarrierDivergence::from_terms(
                    raw_divergence_m,
                    divergence_delta_m,
                    0.0,
                    0.0,
                    receiver_clock_divergence_drift_m(&sat),
                    smoothing_transient_m,
                    0.0,
                ));
                classify_code_carrier_divergence(&mut sat, 0.0);
                apply_pseudorange_physics_rejection(&mut sat);
                entry.sats.push(sat);
            }
        }
    }
    let mut out = Vec::new();
    let mut previous_epoch: Option<ObsEpoch> = None;
    let mut divergence_state = CodeCarrierDivergenceState::default();
    for mut epoch in by_epoch.into_values() {
        bijux_gnss_core::api::sort_obs_sats(&mut epoch);
        apply_code_carrier_divergence_decomposition(&mut epoch, &mut divergence_state);
        apply_dual_frequency_cycle_slip_fusion(
            &mut epoch,
            previous_epoch.as_ref(),
            &raw_snapshots,
            &mut hatch,
        );
        apply_grouped_integer_code_period_ambiguities(
            &mut epoch,
            &ambiguity_inputs,
            &mut diagnostics,
        );
        apply_epoch_decision(&mut epoch);
        if let Some(prev) = previous_epoch.as_ref() {
            if let Some(event) = observation_timing_interval_diagnostic(config, prev, &epoch) {
                diagnostics.push(event);
                reject_epoch_for_invalid_timing(&mut epoch);
            }
        }
        let source_time = epoch.source_time;
        let artifact_id = format!("obs-epoch-{:010}", epoch.epoch_idx);
        epoch.t_rx_s = Seconds(source_time.receiver_time_s.0 + receiver_clock.bias_s);
        epoch.source_time = source_time;
        let epoch_key = bijux_gnss_core::api::obs_epoch_stability_key(&epoch);
        epoch.manifest = Some(ObsEpochManifest {
            version: bijux_gnss_core::api::OBSERVATION_MODEL_VERSION,
            artifact_id: artifact_id.clone(),
            epoch_id: epoch_key,
            source_epoch_idx: epoch.epoch_idx,
            source_sample_index: source_time.sample_index,
            source_time,
            decision: epoch.decision,
            downstream_profile_version:
                bijux_gnss_core::api::OBSERVATION_DOWNSTREAM_PROFILE_VERSION,
        });
        for sat in &mut epoch.sats {
            sat.metadata.observation_epoch_id = artifact_id.clone();
            sat.metadata.observation_status =
                observation_status_label(sat.observation_status).to_string();
            sat.metadata.observation_reject_reasons = sat.observation_reject_reasons.clone();
            let alignment_resolved =
                pseudorange_model_has_resolved_alignment(&sat.metadata.pseudorange_model);
            sat.metadata.observation_support_class =
                observation_support_label(sat.observation_status, alignment_resolved).to_string();
            sat.metadata.observation_uncertainty_class =
                observation_uncertainty_label(sat.cn0_dbhz, has_variance_evidence(sat)).to_string();
        }
        let events = bijux_gnss_core::api::check_obs_epoch_sanity(&epoch);
        if events
            .iter()
            .any(|e| matches!(e.severity, bijux_gnss_core::api::DiagnosticSeverity::Error))
        {
            epoch.valid = false;
            diagnostics.extend(events);
        }
        previous_epoch = Some(epoch.clone());
        out.push(epoch);
    }
    #[cfg(feature = "reference-checks")]
    {
        if let Err(err) = validate_obs_epochs(&out) {
            diagnostics.push(
                DiagnosticEvent::new(DiagnosticSeverity::Error, "OBS_EPOCH_SEQUENCE_INVALID", err)
                    .with_context("stage", "observations"),
            );
        }
    }
    let residuals = observation_residual_reports_from_epochs(&out, &raw_snapshots);
    let measurement_quality = observation_measurement_quality_from_epochs(&out);
    StepReport {
        output: ObservationPipelineArtifacts { epochs: out, residuals, measurement_quality },
        events: diagnostics,
        stats: StepStats::default(),
    }
}

fn observation_timing_interval_diagnostic(
    config: &ReceiverPipelineConfig,
    previous: &ObsEpoch,
    current: &ObsEpoch,
) -> Option<DiagnosticEvent> {
    let expected_sample_interval = observation_interval_samples(config);
    let actual_sample_interval =
        current.source_time.sample_index.saturating_sub(previous.source_time.sample_index);
    let actual_interval_s = current.t_rx_s.0 - previous.t_rx_s.0;
    let expected_interval_s = expected_sample_interval as f64 / config.sampling_freq_hz;
    let tolerance_s = receiver_time_tolerance_s(config.sampling_freq_hz);

    if actual_sample_interval == expected_sample_interval
        && (actual_interval_s - expected_interval_s).abs() <= tolerance_s
    {
        return None;
    }

    Some(
        DiagnosticEvent::new(
            DiagnosticSeverity::Error,
            "GNSS_OBS_TIME_INTERVAL_INVALID",
            format!(
                "observation epoch spacing {} samples ({actual_interval_s:.9}s) does not match configured interval {} samples ({expected_interval_s:.9}s)",
                actual_sample_interval, expected_sample_interval
            ),
        )
        .with_context("epoch", current.epoch_idx.to_string())
        .with_context("previous_epoch", previous.epoch_idx.to_string())
        .with_context("stage", "observations"),
    )
}

fn normalize_observation_cn0_dbhz(cn0_dbhz: f64) -> f64 {
    if !cn0_dbhz.is_finite() {
        return cn0_dbhz;
    }

    let conventions = bijux_gnss_core::api::ConventionsConfig::default();
    cn0_dbhz.clamp(conventions.min_cn0_dbhz, conventions.max_cn0_dbhz)
}

fn observation_interval_samples(config: &ReceiverPipelineConfig) -> u64 {
    let samples_per_code =
        samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length);
    let integration_ms = config.tracking_params(SignalBand::L1).integration_ms.max(1) as u64;
    samples_per_code as u64 * integration_ms
}

fn observation_epoch_interval_samples(
    config: &ReceiverPipelineConfig,
    signal_model: &ObservationSignalModel,
    signal_band: SignalBand,
) -> u64 {
    let samples_per_code =
        (signal_model.samples_per_chip * signal_model.code_length as f64).round() as u64;
    let integration_ms = config.tracking_params(signal_band).integration_ms.max(1) as u64;
    samples_per_code.saturating_mul(integration_ms)
}

fn reject_epoch_for_invalid_timing(epoch: &mut ObsEpoch) {
    epoch.valid = false;
    if epoch.decision == ObservationEpochDecision::Accepted {
        epoch.decision = ObservationEpochDecision::Rejected;
        epoch.decision_reason = Some("invalid_observation_timing".to_string());
    }
}

pub fn observation_decisions_from_epochs(epochs: &[ObsEpoch]) -> Vec<ObsDecisionArtifact> {
    epochs
        .iter()
        .map(|epoch| {
            let artifact_id = epoch
                .manifest
                .as_ref()
                .map(|manifest| manifest.artifact_id.clone())
                .unwrap_or_else(|| format!("obs-epoch-{:010}", epoch.epoch_idx));
            let mut reasons = Vec::new();
            if let Some(reason) = &epoch.decision_reason {
                reasons.push(reason.clone());
            }
            for sat in &epoch.sats {
                reasons.extend(sat.observation_reject_reasons.clone());
            }
            reasons.sort();
            reasons.dedup();
            let accepted_sats = epoch
                .sats
                .iter()
                .map(|sat| SatObservationDecision {
                    sat: sat.signal_id.sat,
                    status: sat.observation_status,
                    reasons: sat.observation_reject_reasons.clone(),
                })
                .collect();
            ObsDecisionArtifact {
                artifact_id,
                epoch_idx: epoch.epoch_idx,
                decision: epoch.decision,
                reasons,
                accepted_sats,
            }
        })
        .collect()
}

fn raw_observation_snapshot(sat: &ObsSatellite, raw_pseudorange_m: f64) -> RawObservationSnapshot {
    RawObservationSnapshot {
        raw_pseudorange_m,
        raw_carrier_phase_cycles: sat.carrier_phase_cycles.0,
        raw_doppler_hz: sat.doppler_hz.0,
        raw_cn0_dbhz: sat.cn0_dbhz,
    }
}

fn observation_residual_reports_from_epochs(
    epochs: &[ObsEpoch],
    raw_snapshots: &std::collections::HashMap<String, RawObservationSnapshot>,
) -> Vec<ObservationResidualEpochReport> {
    use std::collections::HashMap;

    let mut phase_reference: HashMap<String, CarrierPhaseReferenceState> = HashMap::new();
    epochs
        .iter()
        .map(|epoch| {
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
            let sats = epoch
                .sats
                .iter()
                .map(|sat| {
                    let key = observation_snapshot_key(epoch.epoch_idx, sat.signal_id);
                    let snapshot = raw_snapshots
                        .get(&key)
                        .cloned()
                        .unwrap_or_else(|| raw_observation_snapshot(sat, sat.pseudorange_m.0));
                    let pseudorange_expected =
                        sat.timing.map(|timing| timing.signal_travel_time_s.0 * SPEED_OF_LIGHT_MPS);
                    let pseudorange_sigma =
                        evidence_sigma_from_variance(sat, sat.pseudorange_var_m2);
                    let phase_key = observation_signal_key(sat.signal_id);
                    let carrier_phase_expected = if sat.metadata.carrier_phase_continuity
                        == carrier_phase_continuity_label(CarrierPhaseContinuity::Continuous)
                    {
                        phase_reference.get(&phase_key).and_then(|previous| {
                            predicted_carrier_phase_cycles(
                                previous,
                                sat,
                                epoch.source_time.sample_index,
                            )
                        })
                    } else {
                        None
                    };
                    let doppler_expected = Some(snapshot.raw_doppler_hz);
                    let doppler_sigma = evidence_sigma_from_variance(sat, sat.doppler_var_hz2);
                    let cn0_expected = Some(snapshot.raw_cn0_dbhz);
                    let cn0_sigma = sat
                        .metadata
                        .tracking_uncertainty
                        .as_ref()
                        .map(|uncertainty| uncertainty.cn0_dbhz);
                    let carrier_phase_sigma =
                        evidence_sigma_from_variance(sat, sat.carrier_phase_var_cycles2);
                    phase_reference.insert(
                        phase_key,
                        CarrierPhaseReferenceState {
                            corrected_cycles: sat.carrier_phase_cycles.0,
                            doppler_hz: sat.doppler_hz.0,
                            sample_index: epoch.source_time.sample_index,
                            sample_rate_hz: epoch.source_time.sample_rate_hz,
                        },
                    );

                    ObservationResidualSatellite {
                        signal_id: sat.signal_id,
                        accepted: sat.observation_status == ObservationStatus::Accepted,
                        observation_status: sat.observation_status,
                        observation_reject_reasons: sat.observation_reject_reasons.clone(),
                        pseudorange_m: ObservationResidualValue::from_measurement(
                            snapshot.raw_pseudorange_m,
                            sat.pseudorange_m.0,
                            pseudorange_expected,
                            pseudorange_sigma,
                            sat.timing.map(|_| PSEUDORANGE_RESIDUAL_REFERENCE_MODEL),
                        ),
                        carrier_phase_cycles: ObservationResidualValue::from_measurement(
                            snapshot.raw_carrier_phase_cycles,
                            sat.carrier_phase_cycles.0,
                            carrier_phase_expected,
                            carrier_phase_sigma,
                            carrier_phase_expected.map(|_| CARRIER_PHASE_RESIDUAL_REFERENCE_MODEL),
                        ),
                        doppler_hz: ObservationResidualValue::from_measurement(
                            snapshot.raw_doppler_hz,
                            sat.doppler_hz.0,
                            doppler_expected,
                            doppler_sigma,
                            Some(sat.metadata.doppler_model.as_str()),
                        ),
                        cn0_dbhz: ObservationResidualValue::from_measurement(
                            snapshot.raw_cn0_dbhz,
                            sat.cn0_dbhz,
                            cn0_expected,
                            cn0_sigma,
                            Some(CN0_RESIDUAL_REFERENCE_MODEL),
                        ),
                    }
                })
                .collect();

            ObservationResidualEpochReport {
                artifact_id,
                epoch_id,
                epoch_idx: epoch.epoch_idx,
                source_time: epoch.source_time,
                accepted: epoch.decision == ObservationEpochDecision::Accepted,
                decision: epoch.decision,
                decision_reason: epoch.decision_reason.clone(),
                sats,
            }
        })
        .collect()
}

fn predicted_carrier_phase_cycles(
    previous: &CarrierPhaseReferenceState,
    sat: &ObsSatellite,
    current_sample_index: u64,
) -> Option<f64> {
    let sample_rate_hz = if previous.sample_rate_hz.is_finite() && previous.sample_rate_hz > 0.0 {
        previous.sample_rate_hz
    } else {
        return None;
    };
    let delta_seconds =
        current_sample_index.saturating_sub(previous.sample_index) as f64 / sample_rate_hz;
    finite_value(Some(
        previous.corrected_cycles + 0.5 * (previous.doppler_hz + sat.doppler_hz.0) * delta_seconds,
    ))
}

fn observation_snapshot_key(epoch_idx: u64, signal_id: SigId) -> String {
    format!("{epoch_idx:010}:{}", observation_signal_key(signal_id))
}

fn observation_signal_key(signal_id: SigId) -> String {
    format!(
        "{:?}:{:02}:{:?}:{:?}",
        signal_id.sat.constellation, signal_id.sat.prn, signal_id.band, signal_id.code
    )
}

fn cycle_slip_reason(sat: &ObsSatellite) -> Option<String> {
    if !sat.lock_flags.cycle_slip {
        return None;
    }
    sat.metadata
        .observation_lock_reason
        .clone()
        .or_else(|| {
            sat.observation_reject_reasons
                .iter()
                .find(|reason| *reason != "cycle_slip" && reason.contains("slip"))
                .cloned()
        })
        .or_else(|| {
            sat.observation_reject_reasons
                .iter()
                .find(|reason| *reason == "code_carrier_divergence")
                .cloned()
        })
        .or_else(|| Some("cycle_slip".to_string()))
}

fn observation_epoch_id(epoch_idx: u64, sample_index: u64) -> String {
    format!("epoch-{epoch_idx:010}-sample-{sample_index:012}")
}

fn carrier_phase_observation(
    epoch: &TrackEpoch,
    tracked_carrier_phase_cycles: f64,
    doppler_hz: f64,
    sample_rate_hz: f64,
    discontinuity: bool,
    signal_id: SigId,
    state: &mut CarrierPhaseArcState,
) -> CarrierPhaseObservation {
    if !carrier_phase_tracking_usable(epoch) {
        let mut cycle_slip_reason = None;
        if epoch.cycle_slip {
            cycle_slip_reason = Some(explicit_cycle_slip_reason(epoch, "cycle_slip"));
            record_pending_reset(
                &mut state.pending_reset,
                &mut state.pending_reset_reason,
                CarrierPhaseContinuity::ResetAfterCycleSlip,
                cycle_slip_reason.as_deref(),
            );
        } else if discontinuity {
            record_pending_reset(
                &mut state.pending_reset,
                &mut state.pending_reset_reason,
                CarrierPhaseContinuity::ResetAfterDiscontinuity,
                Some("sample_discontinuity"),
            );
        } else if carrier_phase_arc_can_coast(epoch, state) {
            return CarrierPhaseObservation {
                phase_cycles: Cycles(tracked_carrier_phase_cycles),
                cycle_slip: false,
                cycle_slip_reason: None,
                cycle_slip_evidence: cycle_slip_decision_evidence(vec![
                    tracking_lock_evidence(epoch, false, "tracking_lock_coasted"),
                    data_gap_evidence(false, None, None),
                ]),
                continuity: CarrierPhaseContinuity::Coasted,
                arc_start_epoch_idx: state.arc_start_epoch_idx,
                arc_start_sample_index: state.arc_start_sample_index,
                arc: CarrierPhaseArc::new(
                    signal_id,
                    state.arc_start_epoch_idx,
                    state.arc_start_sample_index,
                    state.arc_start_reason.clone(),
                ),
            };
        } else if state.initialized {
            cycle_slip_reason = Some("loss_of_lock".to_string());
            record_pending_reset(
                &mut state.pending_reset,
                &mut state.pending_reset_reason,
                CarrierPhaseContinuity::ResetAfterUnlock,
                cycle_slip_reason.as_deref(),
            );
        }
        state.initialized = false;
        let tracking_reason =
            cycle_slip_reason.as_deref().unwrap_or("tracking_lock_unusable").to_string();
        let discontinuity_delta_s = discontinuity.then(|| {
            carrier_phase_delta_seconds(state.last_sample_index, epoch.sample_index, sample_rate_hz)
        });
        return CarrierPhaseObservation {
            phase_cycles: Cycles(tracked_carrier_phase_cycles),
            cycle_slip: cycle_slip_reason.is_some(),
            cycle_slip_reason,
            cycle_slip_evidence: cycle_slip_decision_evidence(vec![
                tracking_lock_evidence(
                    epoch,
                    tracking_reason != "tracking_lock_unusable",
                    tracking_reason.clone(),
                ),
                data_gap_evidence(discontinuity, discontinuity_delta_s, None),
            ]),
            continuity: CarrierPhaseContinuity::Unusable,
            arc_start_epoch_idx: 0,
            arc_start_sample_index: 0,
            arc: CarrierPhaseArc::invalid_boundary(signal_id, tracking_reason),
        };
    }

    let had_prior_phase_state = state.initialized;
    let previous_doppler_hz = state.doppler_hz;
    let delta_seconds =
        carrier_phase_delta_seconds(state.last_sample_index, epoch.sample_index, sample_rate_hz);
    let observed_phase_delta_cycles = tracked_carrier_phase_cycles - state.phase_cycles;
    let predicted_from_previous_doppler_cycles = previous_doppler_hz * delta_seconds;
    let predicted_from_trapezoid_cycles = 0.5 * (previous_doppler_hz + doppler_hz) * delta_seconds;
    let doppler_predicted_phase_residual_cycles =
        observed_phase_delta_cycles - predicted_from_previous_doppler_cycles;
    let phase_innovation_residual_cycles =
        observed_phase_delta_cycles - predicted_from_trapezoid_cycles;
    let phase_discontinuity_threshold_cycles = carrier_phase_discontinuity_threshold_cycles(epoch);
    let phase_residual_threshold_cycles = carrier_phase_residual_threshold_cycles(epoch);
    let doppler_jump_threshold_hz = doppler_jump_threshold_hz(epoch);
    let doppler_delta_hz = (doppler_hz - previous_doppler_hz).abs();
    let phase_discontinuity = had_prior_phase_state
        && doppler_predicted_phase_residual_cycles.abs() > phase_discontinuity_threshold_cycles;
    let doppler_jump = had_prior_phase_state && doppler_delta_hz > doppler_jump_threshold_hz;
    let phase_residual = had_prior_phase_state
        && phase_innovation_residual_cycles.abs() > phase_residual_threshold_cycles;
    let explicit_reset = if epoch.cycle_slip {
        Some((
            CarrierPhaseContinuity::ResetAfterCycleSlip,
            explicit_cycle_slip_reason(epoch, "cycle_slip"),
        ))
    } else if epoch.lock_state_reason.as_deref() == Some("reacquired")
        && state.pending_reset.is_some()
    {
        Some((CarrierPhaseContinuity::ResetAfterReacquisition, "reacquired".to_string()))
    } else if discontinuity && state.initialized {
        Some((CarrierPhaseContinuity::ResetAfterDiscontinuity, "sample_discontinuity".to_string()))
    } else if doppler_jump {
        Some((CarrierPhaseContinuity::ResetAfterCycleSlip, "doppler_jump".to_string()))
    } else if phase_discontinuity {
        Some((
            CarrierPhaseContinuity::ResetAfterCycleSlip,
            "carrier_phase_discontinuity".to_string(),
        ))
    } else if phase_residual {
        Some((CarrierPhaseContinuity::ResetAfterCycleSlip, "phase_residual".to_string()))
    } else {
        None
    };
    let (reset_reason, cycle_slip_reason) = if let Some((continuity, reason)) = explicit_reset {
        state.pending_reset = None;
        state.pending_reset_reason = None;
        (Some(continuity), Some(reason))
    } else {
        (state.pending_reset.take(), state.pending_reset_reason.take())
    };

    let continuity = if !state.initialized {
        reset_reason.unwrap_or(CarrierPhaseContinuity::ArcStart)
    } else if let Some(reason) = reset_reason {
        reason
    } else {
        CarrierPhaseContinuity::Continuous
    };
    let cycle_slip = cycle_slip_reason.is_some()
        || matches!(
            continuity,
            CarrierPhaseContinuity::ResetAfterCycleSlip
                | CarrierPhaseContinuity::ResetAfterUnlock
                | CarrierPhaseContinuity::ResetAfterReacquisition
                | CarrierPhaseContinuity::ResetAfterDiscontinuity
        );

    if continuity != CarrierPhaseContinuity::Continuous {
        state.arc_start_epoch_idx = epoch.epoch.index;
        state.arc_start_sample_index = epoch.sample_index;
        state.arc_start_reason =
            carrier_phase_arc_start_reason(continuity, cycle_slip_reason.as_deref()).to_string();
    }
    state.phase_cycles = tracked_carrier_phase_cycles;
    state.doppler_hz = doppler_hz;
    state.last_sample_index = epoch.sample_index;
    state.initialized = true;

    CarrierPhaseObservation {
        phase_cycles: Cycles(tracked_carrier_phase_cycles),
        cycle_slip,
        cycle_slip_reason: cycle_slip_reason.clone(),
        cycle_slip_evidence: cycle_slip_decision_evidence(vec![
            tracking_lock_evidence(
                epoch,
                epoch.cycle_slip
                    || matches!(
                        continuity,
                        CarrierPhaseContinuity::ResetAfterUnlock
                            | CarrierPhaseContinuity::ResetAfterReacquisition
                    ),
                if epoch.cycle_slip
                    || matches!(continuity, CarrierPhaseContinuity::ResetAfterUnlock)
                {
                    cycle_slip_reason
                        .clone()
                        .unwrap_or_else(|| explicit_cycle_slip_reason(epoch, "cycle_slip"))
                } else if matches!(continuity, CarrierPhaseContinuity::ResetAfterReacquisition) {
                    cycle_slip_reason.clone().unwrap_or_else(|| "reacquired".to_string())
                } else {
                    "tracking_lock_continuous".to_string()
                },
            ),
            data_gap_evidence(discontinuity && had_prior_phase_state, Some(delta_seconds), None),
            doppler_predicted_phase_evidence(
                phase_discontinuity || doppler_jump,
                if doppler_jump {
                    doppler_delta_hz
                } else {
                    doppler_predicted_phase_residual_cycles.abs()
                },
                if doppler_jump {
                    doppler_jump_threshold_hz
                } else {
                    phase_discontinuity_threshold_cycles
                },
                if doppler_jump { "hz" } else { "cycles" },
                if doppler_jump { "doppler_jump" } else { "carrier_phase_discontinuity" },
            ),
            phase_innovation_evidence(
                phase_residual,
                phase_innovation_residual_cycles.abs(),
                phase_residual_threshold_cycles,
            ),
        ]),
        continuity,
        arc_start_epoch_idx: state.arc_start_epoch_idx,
        arc_start_sample_index: state.arc_start_sample_index,
        arc: CarrierPhaseArc::new(
            signal_id,
            state.arc_start_epoch_idx,
            state.arc_start_sample_index,
            state.arc_start_reason.clone(),
        ),
    }
}

fn carrier_phase_tracking_usable(epoch: &TrackEpoch) -> bool {
    epoch.lock
        && epoch.pll_lock
        && epoch.lock_state != "lost"
        && epoch.carrier_phase_cycles.0.is_finite()
}

fn carrier_phase_arc_can_coast(epoch: &TrackEpoch, state: &CarrierPhaseArcState) -> bool {
    state.initialized
        && !epoch.cycle_slip
        && epoch.carrier_phase_cycles.0.is_finite()
        && epoch.lock_state == "degraded"
        && matches!(
            epoch.lock_state_reason.as_deref(),
            Some("signal_fade") | Some("fade_recovered")
        )
}

fn carrier_phase_arc_start_reason(
    continuity: CarrierPhaseContinuity,
    cycle_slip_reason: Option<&str>,
) -> String {
    match continuity {
        CarrierPhaseContinuity::ArcStart => "arc_start".to_string(),
        CarrierPhaseContinuity::ResetAfterCycleSlip => {
            cycle_slip_reason.unwrap_or("cycle_slip").to_string()
        }
        CarrierPhaseContinuity::ResetAfterUnlock => "loss_of_lock".to_string(),
        CarrierPhaseContinuity::ResetAfterReacquisition => "reacquired".to_string(),
        CarrierPhaseContinuity::ResetAfterDiscontinuity => "sample_discontinuity".to_string(),
        CarrierPhaseContinuity::Unusable
        | CarrierPhaseContinuity::Continuous
        | CarrierPhaseContinuity::Coasted => "continuity_preserved".to_string(),
    }
}

fn explicit_cycle_slip_reason(epoch: &TrackEpoch, fallback: &str) -> String {
    epoch
        .cycle_slip_reason
        .clone()
        .or_else(|| epoch.lock_state_reason.clone())
        .unwrap_or_else(|| fallback.to_string())
}

fn cycle_slip_decision_evidence(
    contributors: Vec<CycleSlipDetectorEvidence>,
) -> CycleSlipDecisionEvidence {
    CycleSlipDecisionEvidence::from_contributors(
        contributors,
        CYCLE_SLIP_DETECTION_PROBABILITY_BUDGET,
        CYCLE_SLIP_FALSE_ALARM_PROBABILITY_BUDGET,
    )
}

fn tracking_lock_evidence(
    epoch: &TrackEpoch,
    triggered: bool,
    reason: impl Into<String>,
) -> CycleSlipDetectorEvidence {
    let value = Some(if epoch.lock && epoch.pll_lock { 1.0 } else { 0.0 });
    CycleSlipDetectorEvidence::new(
        CycleSlipDetector::TrackingLock,
        triggered,
        value,
        Some(1.0),
        "lock_state",
        reason,
    )
}

fn data_gap_evidence(
    triggered: bool,
    delta_s: Option<f64>,
    threshold_s: Option<f64>,
) -> CycleSlipDetectorEvidence {
    CycleSlipDetectorEvidence::new(
        CycleSlipDetector::DataGap,
        triggered,
        delta_s,
        threshold_s,
        "s",
        if triggered { "sample_discontinuity" } else { "continuous_samples" },
    )
}

fn doppler_predicted_phase_evidence(
    triggered: bool,
    value: f64,
    threshold: f64,
    units: &'static str,
    reason: &'static str,
) -> CycleSlipDetectorEvidence {
    CycleSlipDetectorEvidence::new(
        CycleSlipDetector::DopplerPredictedPhase,
        triggered,
        Some(value),
        Some(threshold),
        units,
        reason,
    )
}

fn phase_innovation_evidence(
    triggered: bool,
    value_cycles: f64,
    threshold_cycles: f64,
) -> CycleSlipDetectorEvidence {
    CycleSlipDetectorEvidence::new(
        CycleSlipDetector::PhaseInnovation,
        triggered,
        Some(value_cycles),
        Some(threshold_cycles),
        "cycles",
        if triggered { "phase_residual" } else { "phase_innovation_nominal" },
    )
}

fn code_carrier_divergence_evidence(
    triggered: bool,
    value_m: f64,
    threshold_m: f64,
) -> CycleSlipDetectorEvidence {
    CycleSlipDetectorEvidence::new(
        CycleSlipDetector::CodeCarrierDivergence,
        triggered,
        Some(value_m),
        Some(threshold_m),
        "m",
        if triggered { "code_carrier_divergence" } else { "code_carrier_divergence_nominal" },
    )
}

fn carrier_phase_delta_seconds(
    previous_sample_index: u64,
    current_sample_index: u64,
    sample_rate_hz: f64,
) -> f64 {
    if !sample_rate_hz.is_finite() || sample_rate_hz <= 0.0 {
        return 0.0;
    }
    current_sample_index.saturating_sub(previous_sample_index) as f64 / sample_rate_hz
}

fn carrier_phase_discontinuity_threshold_cycles(epoch: &TrackEpoch) -> f64 {
    carrier_phase_threshold_cycles(
        BASE_CARRIER_PHASE_DISCONTINUITY_THRESHOLD_CYCLES,
        epoch.cn0_dbhz,
        epoch
            .tracking_uncertainty
            .as_ref()
            .map(|uncertainty| uncertainty.carrier_phase_cycles * 6.0)
            .unwrap_or(0.0),
    )
}

fn carrier_phase_residual_threshold_cycles(epoch: &TrackEpoch) -> f64 {
    carrier_phase_threshold_cycles(
        BASE_CARRIER_PHASE_RESIDUAL_THRESHOLD_CYCLES,
        epoch.cn0_dbhz,
        epoch
            .tracking_uncertainty
            .as_ref()
            .map(|uncertainty| uncertainty.carrier_phase_cycles * 4.0)
            .unwrap_or(0.0),
    )
}

fn carrier_phase_threshold_cycles(base_threshold_cycles: f64, cn0_dbhz: f64, floor: f64) -> f64 {
    let cn0_scale =
        if cn0_dbhz.is_finite() { (45.0 / cn0_dbhz.clamp(25.0, 60.0)).sqrt() } else { 1.5 };
    (base_threshold_cycles * cn0_scale).max(base_threshold_cycles).max(floor)
}

fn doppler_jump_threshold_hz(epoch: &TrackEpoch) -> f64 {
    let cn0_scale =
        if epoch.cn0_dbhz.is_finite() { 45.0 / epoch.cn0_dbhz.clamp(25.0, 60.0) } else { 2.0 };
    let uncertainty_floor = epoch
        .tracking_uncertainty
        .as_ref()
        .map(|uncertainty| (uncertainty.doppler_hz * 6.0).max(25.0))
        .unwrap_or(0.0);
    (BASE_DOPPLER_JUMP_THRESHOLD_HZ * cn0_scale)
        .max(BASE_DOPPLER_JUMP_THRESHOLD_HZ)
        .max(uncertainty_floor)
}

fn record_pending_reset(
    pending: &mut Option<CarrierPhaseContinuity>,
    pending_reason: &mut Option<String>,
    candidate: CarrierPhaseContinuity,
    reason: Option<&str>,
) {
    if pending
        .map(|current| {
            carrier_phase_reset_priority(candidate) >= carrier_phase_reset_priority(current)
        })
        .unwrap_or(true)
    {
        *pending = Some(candidate);
        *pending_reason = reason.map(str::to_string);
    }
}

fn carrier_phase_reset_priority(value: CarrierPhaseContinuity) -> u8 {
    match value {
        CarrierPhaseContinuity::ResetAfterCycleSlip => 3,
        CarrierPhaseContinuity::ResetAfterDiscontinuity
        | CarrierPhaseContinuity::ResetAfterReacquisition => 2,
        CarrierPhaseContinuity::ResetAfterUnlock => 1,
        CarrierPhaseContinuity::Unusable
        | CarrierPhaseContinuity::ArcStart
        | CarrierPhaseContinuity::Continuous
        | CarrierPhaseContinuity::Coasted => 0,
    }
}

fn classify_observation_status(
    epoch: &TrackEpoch,
    cn0_dbhz: f64,
) -> (ObservationStatus, Vec<String>) {
    let mut reasons = Vec::new();
    if epoch.lock_state_reason.as_deref() == Some("sample_rate_mismatch") {
        reasons.push("sample_rate_mismatch".to_string());
    }
    if !epoch.lock {
        reasons.push("tracking_unlock".to_string());
    }
    if cn0_dbhz < OBS_WEAK_CN0_DBHZ {
        reasons.push(format!("cn0_below_{OBS_WEAK_CN0_DBHZ:.1}dbhz"));
    }
    if epoch.cycle_slip {
        reasons.push("cycle_slip".to_string());
    }
    if !epoch.prompt_i.is_finite() || !epoch.prompt_q.is_finite() || !cn0_dbhz.is_finite() {
        reasons.push("non_finite_observable".to_string());
    }

    let status = if reasons
        .iter()
        .any(|reason| reason == "non_finite_observable" || reason == "sample_rate_mismatch")
    {
        ObservationStatus::Inconsistent
    } else if reasons.iter().any(|reason| reason == "tracking_unlock") {
        ObservationStatus::Missing
    } else if reasons.iter().any(|reason| reason.starts_with("cn0_below_")) {
        ObservationStatus::Weak
    } else {
        ObservationStatus::Accepted
    };
    (status, reasons)
}

fn apply_epoch_decision(epoch: &mut ObsEpoch) {
    let accepted_count = epoch
        .sats
        .iter()
        .filter(|sat| sat.observation_status == ObservationStatus::Accepted)
        .count();
    let has_inconsistent =
        epoch.sats.iter().any(|sat| sat.observation_status == ObservationStatus::Inconsistent);

    if has_inconsistent {
        epoch.decision = ObservationEpochDecision::Rejected;
        epoch.decision_reason = Some("inconsistent_observable".to_string());
        epoch.valid = false;
        return;
    }
    if accepted_count == 0 {
        epoch.decision = ObservationEpochDecision::Rejected;
        epoch.decision_reason = Some("no_accepted_observables".to_string());
        epoch.valid = false;
        return;
    }
    if let Err(reason) = validate_obs_epochs(std::slice::from_ref(epoch)) {
        epoch.decision = ObservationEpochDecision::Rejected;
        epoch.decision_reason = Some(format!("malformed_observation_set:{reason}"));
        epoch.valid = false;
        return;
    }
    epoch.decision = ObservationEpochDecision::Accepted;
    epoch.decision_reason = Some("accepted_observables_present".to_string());
}

fn apply_pseudorange_physics_rejection(sat: &mut ObsSatellite) {
    let mut reasons = pseudorange_physics_reject_reasons(sat.pseudorange_m);
    if reasons.is_empty() {
        return;
    }
    sat.observation_status = ObservationStatus::Inconsistent;
    sat.observation_reject_reasons.append(&mut reasons);
    sat.observation_reject_reasons.sort();
    sat.observation_reject_reasons.dedup();
}

fn pseudorange_physics_reject_reasons(pseudorange_m: Meters) -> Vec<String> {
    let pseudorange_m = pseudorange_m.0;
    if !pseudorange_m.is_finite() {
        return vec!["non_finite_pseudorange".to_string()];
    }
    if pseudorange_m <= 0.0 {
        return vec!["non_positive_pseudorange".to_string()];
    }
    let cfg = ConventionsConfig::default();
    if pseudorange_m < cfg.min_pseudorange_m || pseudorange_m > cfg.max_pseudorange_m {
        return vec!["pseudorange_out_of_bounds".to_string()];
    }
    Vec::new()
}

fn slip_threshold_m(cn0_dbhz: f64, elevation_deg: Option<f64>) -> f64 {
    let cn0 = cn0_dbhz.clamp(20.0, 60.0);
    let cn0_factor = 45.0 / cn0;
    let elev = elevation_deg.unwrap_or(30.0).clamp(0.0, 90.0);
    let elev_factor = 1.0 + (30.0 - elev).max(0.0) / 30.0;
    5.0 * cn0_factor * elev_factor
}

fn receiver_clock_divergence_drift_m(sat: &ObsSatellite) -> f64 {
    let carrier_hz = sat.metadata.signal.carrier_hz.value();
    let integration_s = sat.metadata.integration_ms as f64 / 1_000.0;
    let frequency_bias_hz = sat.metadata.receiver_clock_frequency_bias_hz;
    if !carrier_hz.is_finite()
        || carrier_hz <= 0.0
        || !integration_s.is_finite()
        || integration_s <= 0.0
        || !frequency_bias_hz.is_finite()
    {
        return 0.0;
    }
    let wavelength_m = SPEED_OF_LIGHT_MPS / carrier_hz;
    -frequency_bias_hz * wavelength_m * integration_s
}

fn apply_code_carrier_divergence_decomposition(
    epoch: &mut ObsEpoch,
    state: &mut CodeCarrierDivergenceState,
) {
    use std::collections::HashMap;

    let mut expected_ionosphere_by_signal = HashMap::<SigId, f64>::new();
    for evidence in ionosphere_delay_evidence_from_epoch(epoch) {
        if let Some(previous_delay_m) =
            state.ionosphere_delay_m_by_signal.insert(evidence.signal_id, evidence.delay_m)
        {
            expected_ionosphere_by_signal
                .insert(evidence.signal_id, 2.0 * (evidence.delay_m - previous_delay_m));
        }
    }

    for sat in &mut epoch.sats {
        let expected_ionosphere_m =
            expected_ionosphere_by_signal.get(&sat.signal_id).copied().unwrap_or(0.0);
        classify_code_carrier_divergence(sat, expected_ionosphere_m);
    }
}

fn classify_code_carrier_divergence(sat: &mut ObsSatellite, expected_ionosphere_m: f64) {
    let Some(current) = sat.metadata.code_carrier_divergence else {
        return;
    };
    let without_multipath = CodeCarrierDivergence::from_terms(
        current.raw_m,
        current.jump_m,
        expected_ionosphere_m,
        current.receiver_clock_m,
        current.oscillator_m,
        current.smoothing_transient_m,
        0.0,
    );
    let threshold_m = slip_threshold_m(sat.cn0_dbhz, sat.elevation_deg) * 0.8;
    let multipath_m = if without_multipath.unexplained_abs_m() > threshold_m {
        without_multipath.unexplained_m
    } else {
        0.0
    };
    let decomposed = CodeCarrierDivergence::from_terms(
        current.raw_m,
        current.jump_m,
        expected_ionosphere_m,
        current.receiver_clock_m,
        current.oscillator_m,
        current.smoothing_transient_m,
        multipath_m,
    );
    sat.multipath_suspect = multipath_m.abs() > threshold_m;
    sat.error_model = observation_error_model(sat, multipath_m.abs());
    sat.metadata.code_carrier_divergence = Some(decomposed);
}

fn ionosphere_delay_evidence_from_epoch(epoch: &ObsEpoch) -> Vec<IonosphereDelayEvidence> {
    use std::collections::BTreeMap;

    let mut by_sat = BTreeMap::<SatId, Vec<&ObsSatellite>>::new();
    for sat in &epoch.sats {
        by_sat.entry(sat.signal_id.sat).or_default().push(sat);
    }

    let mut out = Vec::new();
    for sats in by_sat.values() {
        for left_index in 0..sats.len() {
            for right_index in (left_index + 1)..sats.len() {
                let left = sats[left_index];
                let right = sats[right_index];
                if let Some((left_delay_m, right_delay_m)) =
                    dual_frequency_code_delay_evidence(left, right)
                {
                    out.push(IonosphereDelayEvidence {
                        signal_id: left.signal_id,
                        delay_m: left_delay_m,
                    });
                    out.push(IonosphereDelayEvidence {
                        signal_id: right.signal_id,
                        delay_m: right_delay_m,
                    });
                }
            }
        }
    }
    out
}

fn dual_frequency_code_delay_evidence(
    first: &ObsSatellite,
    second: &ObsSatellite,
) -> Option<(f64, f64)> {
    if first.signal_id.sat != second.signal_id.sat
        || first.signal_id.band == second.signal_id.band
        || !first.lock_flags.code_lock
        || !second.lock_flags.code_lock
    {
        return None;
    }
    let first_frequency_hz = first.metadata.signal.carrier_hz.value();
    let second_frequency_hz = second.metadata.signal.carrier_hz.value();
    if !first_frequency_hz.is_finite()
        || !second_frequency_hz.is_finite()
        || first_frequency_hz <= 0.0
        || second_frequency_hz <= 0.0
        || (first_frequency_hz - second_frequency_hz).abs() <= f64::EPSILON
        || !first.pseudorange_m.0.is_finite()
        || !second.pseudorange_m.0.is_finite()
    {
        return None;
    }

    let first_inverse_frequency2 = 1.0 / first_frequency_hz.powi(2);
    let second_inverse_frequency2 = 1.0 / second_frequency_hz.powi(2);
    let denominator = second_inverse_frequency2 - first_inverse_frequency2;
    if !denominator.is_finite() || denominator == 0.0 {
        return None;
    }
    let code_geometry_free_m = second.pseudorange_m.0 - first.pseudorange_m.0;
    let ionosphere_constant = code_geometry_free_m / denominator;
    Some((
        ionosphere_constant * first_inverse_frequency2,
        ionosphere_constant * second_inverse_frequency2,
    ))
}

fn apply_dual_frequency_cycle_slip_fusion(
    epoch: &mut ObsEpoch,
    previous_epoch: Option<&ObsEpoch>,
    raw_snapshots: &std::collections::HashMap<String, RawObservationSnapshot>,
    hatch: &mut std::collections::HashMap<SigId, HatchFilterState>,
) {
    let pair_indices = same_satellite_dual_frequency_pair_indices(epoch);
    for (left_index, right_index) in pair_indices {
        let left = &epoch.sats[left_index];
        let right = &epoch.sats[right_index];
        let previous_pair = previous_epoch.and_then(|previous| {
            let previous_left = previous.sats.iter().find(|sat| sat.signal_id == left.signal_id)?;
            let previous_right =
                previous.sats.iter().find(|sat| sat.signal_id == right.signal_id)?;
            Some((previous_left, previous_right))
        });

        let geometry_free_contributor =
            geometry_free_cycle_slip_contributor(left, right, previous_pair);
        apply_dual_frequency_cycle_slip_contributor(
            epoch,
            left_index,
            right_index,
            geometry_free_contributor,
            raw_snapshots,
            hatch,
        );

        let left = &epoch.sats[left_index];
        let right = &epoch.sats[right_index];
        let previous_pair = previous_epoch.and_then(|previous| {
            let previous_left = previous.sats.iter().find(|sat| sat.signal_id == left.signal_id)?;
            let previous_right =
                previous.sats.iter().find(|sat| sat.signal_id == right.signal_id)?;
            Some((previous_left, previous_right))
        });
        let melbourne_wubbena_contributor =
            melbourne_wubbena_cycle_slip_contributor(left, right, previous_pair);
        apply_dual_frequency_cycle_slip_contributor(
            epoch,
            left_index,
            right_index,
            melbourne_wubbena_contributor,
            raw_snapshots,
            hatch,
        );
    }
}

fn same_satellite_dual_frequency_pair_indices(epoch: &ObsEpoch) -> Vec<(usize, usize)> {
    let mut pairs = Vec::new();
    for left_index in 0..epoch.sats.len() {
        for right_index in (left_index + 1)..epoch.sats.len() {
            let left = &epoch.sats[left_index];
            let right = &epoch.sats[right_index];
            if left.signal_id.sat == right.signal_id.sat
                && left.signal_id.band != right.signal_id.band
                && left.lock_flags.carrier_lock
                && right.lock_flags.carrier_lock
            {
                pairs.push((left_index, right_index));
            }
        }
    }
    pairs
}

fn geometry_free_cycle_slip_contributor(
    left: &ObsSatellite,
    right: &ObsSatellite,
    previous_pair: Option<(&ObsSatellite, &ObsSatellite)>,
) -> CycleSlipDetectorEvidence {
    let Some(current_m) = geometry_free_phase_pair_m(left, right) else {
        return CycleSlipDetectorEvidence::new(
            CycleSlipDetector::GeometryFreePhase,
            false,
            None,
            Some(GEOMETRY_FREE_CYCLE_SLIP_JUMP_M),
            "m",
            "geometry_free_unavailable",
        );
    };
    let Some((previous_left, previous_right)) = previous_pair else {
        return CycleSlipDetectorEvidence::new(
            CycleSlipDetector::GeometryFreePhase,
            false,
            Some(0.0),
            Some(GEOMETRY_FREE_CYCLE_SLIP_JUMP_M),
            "m",
            "geometry_free_insufficient_history",
        );
    };
    let Some(previous_m) = geometry_free_phase_pair_m(previous_left, previous_right) else {
        return CycleSlipDetectorEvidence::new(
            CycleSlipDetector::GeometryFreePhase,
            false,
            None,
            Some(GEOMETRY_FREE_CYCLE_SLIP_JUMP_M),
            "m",
            "geometry_free_unavailable",
        );
    };
    let delta_m = current_m - previous_m;
    let abs_delta_m = delta_m.abs();
    let reason = if abs_delta_m >= GEOMETRY_FREE_CYCLE_SLIP_JUMP_M {
        "geometry_free_phase"
    } else if abs_delta_m >= GEOMETRY_FREE_IONOSPHERE_DELTA_M {
        "geometry_free_ionosphere_drift"
    } else {
        "geometry_free_nominal"
    };
    CycleSlipDetectorEvidence::new(
        CycleSlipDetector::GeometryFreePhase,
        abs_delta_m >= GEOMETRY_FREE_CYCLE_SLIP_JUMP_M,
        Some(abs_delta_m),
        Some(GEOMETRY_FREE_CYCLE_SLIP_JUMP_M),
        "m",
        reason,
    )
}

fn melbourne_wubbena_cycle_slip_contributor(
    left: &ObsSatellite,
    right: &ObsSatellite,
    previous_pair: Option<(&ObsSatellite, &ObsSatellite)>,
) -> CycleSlipDetectorEvidence {
    let Some(current_m) = melbourne_wubbena_pair_m(left, right) else {
        return CycleSlipDetectorEvidence::new(
            CycleSlipDetector::MelbourneWubbena,
            false,
            None,
            Some(MELBOURNE_WUBBENA_WIDE_LANE_SLIP_JUMP_CYCLES),
            "wide_lane_cycles",
            "melbourne_wubbena_unavailable",
        );
    };
    let Some((previous_left, previous_right)) = previous_pair else {
        return CycleSlipDetectorEvidence::new(
            CycleSlipDetector::MelbourneWubbena,
            false,
            Some(0.0),
            Some(MELBOURNE_WUBBENA_WIDE_LANE_SLIP_JUMP_CYCLES),
            "wide_lane_cycles",
            "melbourne_wubbena_insufficient_history",
        );
    };
    let Some(previous_m) = melbourne_wubbena_pair_m(previous_left, previous_right) else {
        return CycleSlipDetectorEvidence::new(
            CycleSlipDetector::MelbourneWubbena,
            false,
            None,
            Some(MELBOURNE_WUBBENA_WIDE_LANE_SLIP_JUMP_CYCLES),
            "wide_lane_cycles",
            "melbourne_wubbena_unavailable",
        );
    };
    let Some(wide_lane_wavelength_m) = wide_lane_wavelength_m(left, right) else {
        return CycleSlipDetectorEvidence::new(
            CycleSlipDetector::MelbourneWubbena,
            false,
            None,
            Some(MELBOURNE_WUBBENA_WIDE_LANE_SLIP_JUMP_CYCLES),
            "wide_lane_cycles",
            "melbourne_wubbena_wavelength_unavailable",
        );
    };
    let delta_cycles = (current_m - previous_m) / wide_lane_wavelength_m;
    let abs_delta_cycles = delta_cycles.abs();
    CycleSlipDetectorEvidence::new(
        CycleSlipDetector::MelbourneWubbena,
        abs_delta_cycles >= MELBOURNE_WUBBENA_WIDE_LANE_SLIP_JUMP_CYCLES,
        Some(abs_delta_cycles),
        Some(MELBOURNE_WUBBENA_WIDE_LANE_SLIP_JUMP_CYCLES),
        "wide_lane_cycles",
        if abs_delta_cycles >= MELBOURNE_WUBBENA_WIDE_LANE_SLIP_JUMP_CYCLES {
            "melbourne_wubbena"
        } else {
            "melbourne_wubbena_nominal"
        },
    )
}

fn apply_dual_frequency_cycle_slip_contributor(
    epoch: &mut ObsEpoch,
    left_index: usize,
    right_index: usize,
    contributor: CycleSlipDetectorEvidence,
    raw_snapshots: &std::collections::HashMap<String, RawObservationSnapshot>,
    hatch: &mut std::collections::HashMap<SigId, HatchFilterState>,
) {
    let triggered = contributor.triggered;
    let reason = contributor.reason.clone();
    for index in [left_index, right_index] {
        let sat = &mut epoch.sats[index];
        upsert_cycle_slip_contributor(sat, contributor.clone());
        if triggered {
            apply_cycle_slip_surface(sat, Some(&reason));
            reset_smoothed_observation_after_fused_slip(epoch.epoch_idx, sat, raw_snapshots, hatch);
        }
    }
}

fn upsert_cycle_slip_contributor(sat: &mut ObsSatellite, contributor: CycleSlipDetectorEvidence) {
    let evidence = sat
        .metadata
        .cycle_slip_evidence
        .get_or_insert_with(|| cycle_slip_decision_evidence(Vec::new()));
    evidence.upsert_contributor(contributor);
}

fn reset_smoothed_observation_after_fused_slip(
    epoch_idx: u64,
    sat: &mut ObsSatellite,
    raw_snapshots: &std::collections::HashMap<String, RawObservationSnapshot>,
    hatch: &mut std::collections::HashMap<SigId, HatchFilterState>,
) {
    if let Some(snapshot) = raw_snapshots.get(&observation_snapshot_key(epoch_idx, sat.signal_id)) {
        sat.pseudorange_m = Meters(snapshot.raw_pseudorange_m);
    }
    let state = hatch.entry(sat.signal_id).or_default();
    state.clear_arc();
    sat.metadata.smoothing_age = 1;
    sat.metadata.smoothing_resets = state.reset_count();
    if let Some(divergence) = sat.metadata.code_carrier_divergence {
        sat.metadata.code_carrier_divergence = Some(CodeCarrierDivergence::from_terms(
            divergence.raw_m,
            divergence.jump_m,
            divergence.expected_ionosphere_m,
            divergence.receiver_clock_m,
            divergence.oscillator_m,
            0.0,
            divergence.multipath_m,
        ));
    }
}

fn geometry_free_phase_pair_m(left: &ObsSatellite, right: &ObsSatellite) -> Option<f64> {
    Some(
        signal_cycles_to_meters(left.carrier_phase_cycles, left.metadata.signal).0
            - signal_cycles_to_meters(right.carrier_phase_cycles, right.metadata.signal).0,
    )
    .filter(|value| value.is_finite())
}

fn melbourne_wubbena_pair_m(left: &ObsSatellite, right: &ObsSatellite) -> Option<f64> {
    Some(geometry_free_phase_pair_m(left, right)? - (left.pseudorange_m.0 - right.pseudorange_m.0))
        .filter(|value| value.is_finite())
}

fn wide_lane_wavelength_m(left: &ObsSatellite, right: &ObsSatellite) -> Option<f64> {
    let left_hz = left.metadata.signal.carrier_hz.value();
    let right_hz = right.metadata.signal.carrier_hz.value();
    let delta_hz = (left_hz - right_hz).abs();
    (delta_hz.is_finite() && delta_hz > f64::EPSILON).then_some(SPEED_OF_LIGHT_MPS / delta_hz)
}

fn tracking_lock_quality(epoch: &TrackEpoch, observation_cycle_slip: bool) -> f64 {
    let mut quality =
        if epoch.cn0_dbhz.is_finite() { (epoch.cn0_dbhz / 60.0).clamp(0.0, 1.0) } else { 0.0 };
    if epoch.anti_false_lock {
        quality *= 0.5;
    }
    if epoch.cycle_slip {
        quality *= 0.2;
    }
    if !epoch.lock {
        quality *= 0.2;
    }
    if !epoch.pll_lock {
        quality *= 0.7;
    }
    if !epoch.dll_lock {
        quality *= 0.8;
    }
    if !epoch.fll_lock {
        quality *= 0.9;
    }
    if observation_cycle_slip {
        quality *= 0.2;
    }
    quality
}

fn observation_lock_state(epoch: &TrackEpoch, observation_cycle_slip: bool) -> &'static str {
    if observation_cycle_slip {
        return "cycle_slip";
    }
    if epoch.lock_state_reason.as_deref() == Some("reacquired") {
        return "reacquired";
    }
    match epoch.lock_state.as_str() {
        "tracking" => "locked",
        "acquired" => "acquired",
        "pull_in" => "pull_in",
        "degraded" => "degraded",
        "lost" => "lost",
        "inactive" => "inactive",
        _ if epoch.lock => "locked",
        _ => "inactive",
    }
}

fn observation_lock_reason(
    epoch: &TrackEpoch,
    observation_cycle_slip: bool,
    cycle_slip_reason: Option<&str>,
) -> Option<String> {
    if observation_cycle_slip {
        return cycle_slip_reason
            .map(str::to_string)
            .or_else(|| epoch.cycle_slip_reason.clone())
            .or_else(|| epoch.lock_state_reason.clone())
            .or_else(|| Some("cycle_slip".to_string()));
    }
    epoch
        .lock_state_reason
        .clone()
        .or_else(|| epoch.anti_false_lock.then(|| "anti_false_lock".to_string()))
}

fn apply_cycle_slip_surface(sat: &mut ObsSatellite, reason: Option<&str>) {
    if !sat.lock_flags.cycle_slip && reason.is_none() {
        return;
    }
    let already_cycle_slip = sat.metadata.observation_lock_state == "cycle_slip";
    sat.lock_flags.cycle_slip = true;
    push_observation_reject_reason(&mut sat.observation_reject_reasons, "cycle_slip");
    if let Some(reason) = reason {
        push_observation_reject_reason(&mut sat.observation_reject_reasons, reason);
    }
    sat.metadata.observation_lock_state = "cycle_slip".to_string();
    sat.metadata.tracking_lock_state = "cycle_slip".to_string();
    if !already_cycle_slip
        || sat
            .metadata
            .observation_lock_reason
            .as_deref()
            .is_none_or(|current| current == "cycle_slip")
    {
        sat.metadata.observation_lock_reason = Some(reason.unwrap_or("cycle_slip").to_string());
    }
    sat.metadata.lock_quality *= 0.2;
}

fn push_observation_reject_reason(reasons: &mut Vec<String>, reason: &str) {
    if reasons.iter().any(|existing| existing == reason) {
        return;
    }
    reasons.push(reason.to_string());
    reasons.sort();
}

fn grouped_epoch_time_mismatch_reasons(
    grouped: &ObsEpoch,
    incoming: &ObsEpoch,
) -> Option<Vec<&'static str>> {
    if grouped.sats.is_empty() {
        return None;
    }

    let mut reasons = Vec::new();
    let tolerance_s = receiver_time_tolerance_s(
        grouped.source_time.sample_rate_hz.max(incoming.source_time.sample_rate_hz),
    );

    if (grouped.t_rx_s.0 - incoming.t_rx_s.0).abs() > tolerance_s {
        reasons.push("receiver_time_mismatch");
    }
    if grouped.source_time.sample_index != incoming.source_time.sample_index
        || (grouped.source_time.sample_rate_hz - incoming.source_time.sample_rate_hz).abs()
            > f64::EPSILON
    {
        reasons.push("receiver_sample_trace_mismatch");
    }
    if grouped.gps_week != incoming.gps_week
        || !optional_seconds_match(grouped.tow_s, incoming.tow_s, tolerance_s)
    {
        reasons.push("gps_receive_time_mismatch");
    }

    (!reasons.is_empty()).then_some(reasons)
}

fn receiver_time_tolerance_s(sample_rate_hz: f64) -> f64 {
    if sample_rate_hz.is_finite() && sample_rate_hz > 0.0 {
        (0.5 / sample_rate_hz).max(1.0e-12)
    } else {
        1.0e-12
    }
}

fn optional_seconds_match(lhs: Option<Seconds>, rhs: Option<Seconds>, tolerance_s: f64) -> bool {
    match (lhs, rhs) {
        (Some(lhs), Some(rhs)) => (lhs.0 - rhs.0).abs() <= tolerance_s,
        (None, None) => true,
        _ => false,
    }
}

fn mark_grouped_epoch_time_mismatch(sat: &mut ObsSatellite, reasons: &[&str]) {
    sat.observation_status = ObservationStatus::Inconsistent;
    for reason in reasons {
        push_observation_reject_reason(&mut sat.observation_reject_reasons, reason);
    }
}

fn grouped_epoch_time_mismatch_diagnostic(
    grouped: &ObsEpoch,
    incoming_epoch_idx: u64,
    incoming_source_time: ReceiverSampleTrace,
    incoming_t_rx_s: Seconds,
    sat: &ObsSatellite,
) -> DiagnosticEvent {
    DiagnosticEvent::new(
        DiagnosticSeverity::Error,
        "OBS_GROUPED_RECEIVER_TIME_MISMATCH",
        format!(
            "satellite {} sample trace {}@{}s does not match grouped epoch {}@{}s",
            sat.signal_id.sat.prn,
            incoming_source_time.sample_index,
            incoming_t_rx_s.0,
            grouped.source_time.sample_index,
            grouped.t_rx_s.0
        ),
    )
    .with_context("epoch", incoming_epoch_idx.to_string())
    .with_context("stage", "observations")
}

fn tracking_time_tag(epoch: &TrackEpoch, last_locked_sample: &mut Option<u64>) -> (String, u64) {
    if epoch.lock || epoch.lock_state == "tracking" || epoch.lock_state == "acquired" {
        *last_locked_sample = Some(epoch.sample_index);
        return ("tracking".to_string(), epoch.sample_index);
    }
    match (epoch.lock, *last_locked_sample) {
        (false, Some(last)) => ("tracking_last_locked".to_string(), last),
        (false, None) => ("tracking_unlocked".to_string(), epoch.sample_index),
        _ => ("tracking".to_string(), epoch.sample_index),
    }
}

#[cfg(test)]
pub(crate) fn fake_obs_epoch_for_nav_tests(epoch_idx: u64) -> ObsEpoch {
    let receive_tow_s = epoch_idx as f64 * 0.001;
    let sats = (1..=4)
        .map(|prn| ObsSatellite {
            signal_id: SigId {
                sat: SatId { constellation: Constellation::Gps, prn },
                band: SignalBand::L1,
                code: SignalCode::Ca,
            },
            pseudorange_m: Meters(20_200_000.0 + prn as f64),
            pseudorange_var_m2: 100.0,
            carrier_phase_cycles: Cycles(0.0),
            carrier_phase_var_cycles2: 1.0,
            doppler_hz: Hertz(0.0),
            doppler_var_hz2: 1.0,
            cn0_dbhz: 45.0,
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
            azimuth_deg: Some(0.0),
            weight: Some(1.0),
            timing: Some(ObsSignalTiming {
                signal_travel_time_s: Seconds((20_200_000.0 + prn as f64) / SPEED_OF_LIGHT_MPS),
                transmit_gps_time: GpsTime {
                    week: 0,
                    tow_s: receive_tow_s - ((20_200_000.0 + prn as f64) / SPEED_OF_LIGHT_MPS),
                },
            }),
            error_model: None,
            metadata: ObsMetadata {
                tracking_mode: "scalar".to_string(),
                integration_ms: 1,
                lock_quality: 1.0,
                smoothing_window: 0,
                smoothing_age: 0,
                smoothing_resets: 0,
                signal: signal_spec_gps_l1_ca(),
                tracking_lock_state: "locked".to_string(),
                observation_lock_state: "locked".to_string(),
                observation_lock_reason: Some("stable_tracking".to_string()),
                tracking_lock_quality: 1.0,
                ..ObsMetadata::default()
            },
        })
        .collect();
    ObsEpoch {
        t_rx_s: Seconds(receive_tow_s),
        source_time: ReceiverSampleTrace::from_sample_index(epoch_idx, 1_000.0),
        gps_week: None,
        tow_s: None,
        epoch_idx,
        discontinuity: false,
        valid: true,
        processing_ms: None,
        role: ReceiverRole::Rover,
        sats,
        decision: ObservationEpochDecision::Accepted,
        decision_reason: Some("accepted_observables_present".to_string()),
        manifest: Some(ObsEpochManifest {
            version: bijux_gnss_core::api::OBSERVATION_MODEL_VERSION,
            artifact_id: format!("obs-epoch-{epoch_idx:010}"),
            epoch_id: observation_epoch_id(epoch_idx, epoch_idx),
            source_epoch_idx: epoch_idx,
            source_sample_index: epoch_idx,
            source_time: ReceiverSampleTrace::from_sample_index(epoch_idx, 1_000.0),
            decision: ObservationEpochDecision::Accepted,
            downstream_profile_version:
                bijux_gnss_core::api::OBSERVATION_DOWNSTREAM_PROFILE_VERSION,
        }),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeSet;

    use bijux_gnss_core::api::{
        Chips, Epoch, Hertz, Meters, SatId, SignalDelayAlignment, TrackingTransmitTime,
        TrackingUncertainty,
    };
    use bijux_gnss_signal::api::{
        registered_signal_registry_entries, signal_cycles_to_meters, signal_meters_to_cycles,
    };

    fn make_tracking_epoch(
        prn: u8,
        config: &ReceiverPipelineConfig,
        epoch_idx: u64,
        carrier_hz: f64,
    ) -> TrackEpoch {
        make_tracking_epoch_with_phase(prn, config, epoch_idx, carrier_hz, 0.0)
    }

    #[test]
    fn observation_signal_support_follows_tracked_signal_mapping() {
        assert!(supports_observation_signal(Constellation::Gps, SignalBand::L1, SignalCode::Ca));
        assert!(supports_observation_signal(Constellation::Gps, SignalBand::L2, SignalCode::L2C));
        assert!(supports_observation_signal(Constellation::Gps, SignalBand::L5, SignalCode::L5I));
        assert!(supports_observation_signal(
            Constellation::Galileo,
            SignalBand::E5,
            SignalCode::E5a
        ));
        assert!(supports_observation_signal(
            Constellation::Beidou,
            SignalBand::B2,
            SignalCode::B2I
        ));
        assert!(supports_observation_signal(
            Constellation::Glonass,
            SignalBand::L1,
            SignalCode::Unknown
        ));
        assert!(!supports_observation_signal(Constellation::Gps, SignalBand::L2, SignalCode::Ca));
        assert_eq!(tracked_signal_code_for_band(Constellation::Unknown, SignalBand::Unknown), None);
    }

    #[test]
    fn observation_signal_support_matches_registered_signal_inventory() {
        let registered = registered_signal_registry_entries();
        let supported = registered
            .iter()
            .filter(|entry| {
                supports_observation_signal(
                    entry.spec.constellation,
                    entry.spec.band,
                    entry.spec.code,
                )
            })
            .map(|entry| (entry.spec.constellation, entry.spec.band, entry.spec.code))
            .collect::<BTreeSet<_>>();
        let unsupported = registered
            .iter()
            .filter(|entry| {
                !supports_observation_signal(
                    entry.spec.constellation,
                    entry.spec.band,
                    entry.spec.code,
                )
            })
            .map(|entry| (entry.spec.constellation, entry.spec.band, entry.spec.code))
            .collect::<BTreeSet<_>>();

        assert_eq!(
            supported,
            BTreeSet::from([
                (Constellation::Gps, SignalBand::L1, SignalCode::Ca),
                (Constellation::Gps, SignalBand::L2, SignalCode::L2C),
                (Constellation::Gps, SignalBand::L5, SignalCode::L5I),
                (Constellation::Gps, SignalBand::L5, SignalCode::L5Q),
                (Constellation::Galileo, SignalBand::E1, SignalCode::E1B),
                (Constellation::Galileo, SignalBand::E5, SignalCode::E5a),
                (Constellation::Galileo, SignalBand::E5, SignalCode::E5b),
                (Constellation::Glonass, SignalBand::L1, SignalCode::Unknown),
                (Constellation::Beidou, SignalBand::B1, SignalCode::B1I),
                (Constellation::Beidou, SignalBand::B2, SignalCode::B2I),
            ])
        );
        assert_eq!(
            unsupported,
            BTreeSet::from([
                (Constellation::Gps, SignalBand::L2, SignalCode::Py),
                (Constellation::Galileo, SignalBand::E1, SignalCode::E1C),
            ])
        );
    }

    fn make_tracking_epoch_with_phase(
        prn: u8,
        config: &ReceiverPipelineConfig,
        epoch_idx: u64,
        carrier_hz: f64,
        carrier_phase_cycles: f64,
    ) -> TrackEpoch {
        let sample_index = epoch_idx
            * samples_per_code(
                config.sampling_freq_hz,
                config.code_freq_basis_hz,
                config.code_length,
            ) as u64;
        TrackEpoch {
            epoch: Epoch { index: epoch_idx },
            sample_index,
            source_time: ReceiverSampleTrace::from_sample_index(
                sample_index,
                config.sampling_freq_hz,
            ),
            sat: SatId { constellation: Constellation::Gps, prn },
            prompt_i: 1.0,
            prompt_q: 0.0,
            carrier_hz: Hertz(carrier_hz),
            carrier_phase_cycles: Cycles(carrier_phase_cycles),
            code_rate_hz: Hertz(config.code_freq_basis_hz),
            code_phase_samples: Chips(0.0),
            lock: true,
            cn0_dbhz: 45.0,
            pll_lock: true,
            dll_lock: true,
            fll_lock: true,
            cycle_slip: false,
            nav_bit_lock: false,
            dll_err: 0.0,
            pll_err: 0.0,
            fll_err: 0.0,
            anti_false_lock: false,
            cycle_slip_reason: None,
            lock_state: "tracking".to_string(),
            lock_state_reason: Some("stable_tracking".to_string()),
            tracking_uncertainty: Some(test_tracking_uncertainty()),
            processing_ms: None,
            ..TrackEpoch::default()
        }
    }

    fn test_tracking_uncertainty() -> TrackingUncertainty {
        TrackingUncertainty {
            code_phase_samples: 0.05,
            carrier_phase_cycles: 0.02,
            doppler_hz: 1.0,
            cn0_dbhz: 0.5,
        }
    }

    fn set_code_phase_uncertainty(epoch: &mut TrackEpoch, code_phase_samples: f64) {
        let mut uncertainty =
            epoch.tracking_uncertainty.clone().unwrap_or_else(test_tracking_uncertainty);
        uncertainty.code_phase_samples = code_phase_samples;
        epoch.tracking_uncertainty = Some(uncertainty);
    }

    fn make_tracking_epoch_with_alignment(
        prn: u8,
        config: &ReceiverPipelineConfig,
        epoch_idx: u64,
        carrier_hz: f64,
        carrier_phase_cycles: f64,
        whole_code_periods: u64,
        code_phase_samples: f64,
    ) -> TrackEpoch {
        TrackEpoch {
            code_phase_samples: Chips(test_tracking_code_phase_samples(config, code_phase_samples)),
            signal_delay_alignment: Some(SignalDelayAlignment {
                whole_code_periods,
                sample_delay_samples: 0,
                source: "synthetic_truth".to_string(),
            }),
            ..make_tracking_epoch_with_phase(
                prn,
                config,
                epoch_idx,
                carrier_hz,
                carrier_phase_cycles,
            )
        }
    }

    fn test_tracking_code_phase_samples(
        config: &ReceiverPipelineConfig,
        aligned_code_phase_samples: f64,
    ) -> f64 {
        if !aligned_code_phase_samples.is_finite() || aligned_code_phase_samples < 0.0 {
            return aligned_code_phase_samples;
        }
        let period_samples = samples_per_code(
            config.sampling_freq_hz,
            config.code_freq_basis_hz,
            config.code_length,
        ) as f64;
        (period_samples - aligned_code_phase_samples).rem_euclid(period_samples)
    }

    fn make_observation_ready_epoch(
        prn: u8,
        config: &ReceiverPipelineConfig,
        epoch_idx: u64,
    ) -> TrackEpoch {
        make_tracking_epoch_with_alignment(prn, config, epoch_idx, 0.0, 0.0, 68, 128.0)
    }

    fn aligned_pseudorange_m(
        config: &ReceiverPipelineConfig,
        whole_code_periods: u64,
        code_phase_samples: f64,
    ) -> f64 {
        let code_phase_chips = code_phase_samples
            / (samples_per_code(
                config.sampling_freq_hz,
                config.code_freq_basis_hz,
                config.code_length,
            ) as f64
                / config.code_length as f64);
        ((whole_code_periods as f64 * config.code_length as f64) + code_phase_chips)
            / config.code_freq_basis_hz
            * SPEED_OF_LIGHT_MPS
    }

    fn test_tracking_code_phase_samples_for_signal(
        config: &ReceiverPipelineConfig,
        signal: SignalSpec,
        code_length: usize,
        aligned_code_phase_chips: f64,
    ) -> f64 {
        if !aligned_code_phase_chips.is_finite() || aligned_code_phase_chips < 0.0 {
            return aligned_code_phase_chips;
        }
        let samples_per_chip = config.sampling_freq_hz / signal.code_rate_hz;
        let period_samples = samples_per_chip * code_length as f64;
        let aligned_code_phase_samples = aligned_code_phase_chips * samples_per_chip;
        (period_samples - aligned_code_phase_samples).rem_euclid(period_samples)
    }

    fn aligned_pseudorange_m_for_signal(
        signal: SignalSpec,
        code_length: usize,
        whole_code_periods: u64,
        code_phase_chips: f64,
    ) -> f64 {
        ((whole_code_periods as f64 * code_length as f64) + code_phase_chips) / signal.code_rate_hz
            * SPEED_OF_LIGHT_MPS
    }

    fn gps_l1ca_decoded_time_epoch(
        prn: u8,
        config: &ReceiverPipelineConfig,
        epoch_idx: u64,
        capture_start_gps_time: GpsTime,
        receive_gps_time: GpsTime,
        decoded_code_periods: f64,
        aligned_code_phase_chips: f64,
    ) -> TrackEpoch {
        let signal = signal_spec_gps_l1_ca();
        let code_period_s = 1023.0 / signal.code_rate_hz;
        let code_delay_s = aligned_code_phase_chips / signal.code_rate_hz;
        let expected_signal_travel_time_s = decoded_code_periods * code_period_s + code_delay_s;
        let decoded_transmit_time = receive_gps_time.offset_seconds(-expected_signal_travel_time_s);
        let epoch_sample_index = ((receive_gps_time.tow_s - capture_start_gps_time.tow_s)
            * config.sampling_freq_hz)
            .round() as u64;

        TrackEpoch {
            sat: SatId { constellation: Constellation::Gps, prn },
            signal_band: SignalBand::L1,
            carrier_hz: Hertz(tracked_signal_center_hz(config.intermediate_freq_hz, signal)),
            code_rate_hz: Hertz(signal.code_rate_hz),
            code_phase_samples: Chips(test_tracking_code_phase_samples_for_signal(
                config,
                signal,
                1023,
                aligned_code_phase_chips,
            )),
            transmit_time: Some(TrackingTransmitTime {
                transmit_gps_time: decoded_transmit_time,
                source: "decoded_lnav_how".to_string(),
            }),
            sample_index: epoch_sample_index,
            source_time: ReceiverSampleTrace::from_sample_index(
                epoch_sample_index,
                config.sampling_freq_hz,
            ),
            signal_delay_alignment: None,
            ..make_tracking_epoch_with_phase(
                prn,
                config,
                epoch_idx,
                tracked_signal_center_hz(config.intermediate_freq_hz, signal),
                0.0,
            )
        }
    }

    fn epoch_sample_index(config: &ReceiverPipelineConfig, epoch_idx: u64) -> u64 {
        epoch_idx
            * samples_per_code(
                config.sampling_freq_hz,
                config.code_freq_basis_hz,
                config.code_length,
            ) as u64
    }

    fn observation_sample_index(
        config: &ReceiverPipelineConfig,
        initial_sample_index: u64,
        observation_offset: u64,
    ) -> u64 {
        initial_sample_index + observation_offset * observation_interval_samples(config)
    }

    fn make_track(prn: u8, config: &ReceiverPipelineConfig) -> TrackingResult {
        let sat = SatId { constellation: Constellation::Gps, prn };
        let epoch = make_tracking_epoch(prn, config, 70, 0.0);
        TrackingResult {
            sat,
            carrier_hz: 0.0,
            code_phase_samples: 0.0,
            acquisition_hypothesis: "deferred".to_string(),
            acquisition_score: 0.0,
            acquisition_code_phase_samples: 0,
            acquisition_carrier_hz: 0.0,
            acq_to_track_state: "deferred".to_string(),
            epochs: vec![epoch],
            transitions: Vec::new(),
        }
    }

    fn track_from_epoch(epoch: TrackEpoch) -> TrackingResult {
        let sat = epoch.sat;
        TrackingResult {
            sat,
            carrier_hz: epoch.carrier_hz.0,
            code_phase_samples: epoch.code_phase_samples.0,
            acquisition_hypothesis: "accepted".to_string(),
            acquisition_score: 1.0,
            acquisition_code_phase_samples: 0,
            acquisition_carrier_hz: epoch.carrier_hz.0,
            acq_to_track_state: "accepted".to_string(),
            epochs: vec![epoch],
            transitions: Vec::new(),
        }
    }

    fn residual_epoch(
        report: &StepReport<Vec<ObservationResidualEpochReport>>,
        epoch_idx: u64,
    ) -> &ObservationResidualEpochReport {
        report.output.iter().find(|epoch| epoch.epoch_idx == epoch_idx).expect("residual epoch")
    }

    #[test]
    fn observation_satellite_order_is_canonical() {
        let config = ReceiverPipelineConfig::default();
        let tracks_a = vec![make_track(2, &config), make_track(1, &config)];
        let tracks_b = vec![make_track(1, &config), make_track(2, &config)];

        let report_a = observations_from_tracking_results(&config, &tracks_a, 10);
        let report_b = observations_from_tracking_results(&config, &tracks_b, 10);

        let json_a = serde_json::to_string(&report_a.output).unwrap();
        let json_b = serde_json::to_string(&report_b.output).unwrap();
        assert_eq!(json_a, json_b);
    }

    #[test]
    fn observations_from_tracking_derives_if_relative_doppler_from_carrier() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 2_000.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            ..ReceiverPipelineConfig::default()
        };
        let doppler_hz = -250.0;
        let carrier_hz = crate::pipeline::doppler::carrier_hz_from_doppler_hz(
            config.intermediate_freq_hz,
            doppler_hz,
        );
        let epochs = vec![
            make_tracking_epoch(7, &config, 70, carrier_hz),
            make_tracking_epoch(7, &config, 71, carrier_hz),
        ];

        let (observations, diagnostics) = observations_from_tracking(&config, &epochs);

        assert!(diagnostics.is_empty(), "unexpected diagnostics: {diagnostics:?}");
        assert_eq!(observations.len(), 2);
        assert!(
            observations
                .iter()
                .all(|epoch| (epoch.sats[0].doppler_hz.0 - doppler_hz).abs() <= f64::EPSILON),
            "{observations:?}"
        );
    }

    #[test]
    fn observations_from_tracking_results_preserve_distinct_satellite_dopplers() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 2_000.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            ..ReceiverPipelineConfig::default()
        };
        let track_a = TrackingResult {
            sat: SatId { constellation: Constellation::Gps, prn: 3 },
            carrier_hz: 0.0,
            code_phase_samples: 0.0,
            acquisition_hypothesis: "deferred".to_string(),
            acquisition_score: 0.0,
            acquisition_code_phase_samples: 0,
            acquisition_carrier_hz: 0.0,
            acq_to_track_state: "deferred".to_string(),
            epochs: vec![make_tracking_epoch(
                3,
                &config,
                70,
                crate::pipeline::doppler::carrier_hz_from_doppler_hz(
                    config.intermediate_freq_hz,
                    125.0,
                ),
            )],
            transitions: Vec::new(),
        };
        let track_b = TrackingResult {
            sat: SatId { constellation: Constellation::Gps, prn: 8 },
            carrier_hz: 0.0,
            code_phase_samples: 0.0,
            acquisition_hypothesis: "deferred".to_string(),
            acquisition_score: 0.0,
            acquisition_code_phase_samples: 0,
            acquisition_carrier_hz: 0.0,
            acq_to_track_state: "deferred".to_string(),
            epochs: vec![make_tracking_epoch(
                8,
                &config,
                70,
                crate::pipeline::doppler::carrier_hz_from_doppler_hz(
                    config.intermediate_freq_hz,
                    -175.0,
                ),
            )],
            transitions: Vec::new(),
        };

        let report = observations_from_tracking_results(&config, &[track_a, track_b], 10);
        let epoch = report.output.first().expect("observation epoch");
        let dopplers = epoch
            .sats
            .iter()
            .map(|sat| (sat.signal_id.sat.prn, sat.doppler_hz.0))
            .collect::<Vec<_>>();

        assert!(report.events.is_empty(), "unexpected diagnostics: {:?}", report.events);
        assert_eq!(dopplers, vec![(3, 125.0), (8, -175.0)]);
    }

    #[test]
    fn observations_mark_carrier_phase_arc_start_and_continuity() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            ..ReceiverPipelineConfig::default()
        };
        let carrier_hz = crate::pipeline::doppler::carrier_hz_from_doppler_hz(0.0, 125.0);
        let epochs = vec![
            make_tracking_epoch_with_phase(6, &config, 70, carrier_hz, 10.00),
            make_tracking_epoch_with_phase(6, &config, 71, carrier_hz, 10.125),
            make_tracking_epoch_with_phase(6, &config, 72, carrier_hz, 10.250),
        ];

        let (observations, diagnostics) = observations_from_tracking(&config, &epochs);

        assert!(diagnostics.is_empty(), "unexpected diagnostics: {diagnostics:?}");
        assert_eq!(observations.len(), 3);
        assert_eq!(observations[0].sats[0].metadata.carrier_phase_continuity, "arc_start");
        assert_eq!(observations[0].sats[0].metadata.carrier_phase_arc_start_epoch_idx, 70);
        assert_eq!(
            observations[0].sats[0].metadata.carrier_phase_arc_start_sample_index,
            observations[0].sats[0].metadata.time_tag_sample_index
        );
        assert_eq!(
            observations[1].sats[0]
                .metadata
                .cycle_slip_evidence
                .as_ref()
                .expect("cycle-slip evidence")
                .triggered_detectors(),
            Vec::<CycleSlipDetector>::new()
        );
        assert_eq!(observations[1].sats[0].metadata.carrier_phase_continuity, "continuous");
        assert_eq!(observations[2].sats[0].metadata.carrier_phase_continuity, "continuous");
        assert!((observations[1].sats[0].carrier_phase_cycles.0 - 10.125).abs() <= f64::EPSILON);
        assert_eq!(
            observations[2].sats[0].metadata.carrier_phase_arc_start_sample_index,
            observations[0].sats[0].metadata.carrier_phase_arc_start_sample_index
        );
        let first_arc = observations[0].sats[0]
            .metadata
            .carrier_phase_arc
            .as_ref()
            .expect("first carrier-phase arc");
        assert_eq!(first_arc.signal_id, observations[0].sats[0].signal_id);
        assert_eq!(first_arc.start_reason, "arc_start");
        assert!(first_arc.valid_for_smoothing);
        assert!(first_arc.valid_for_ambiguity);
        assert_eq!(
            observations[1].sats[0]
                .metadata
                .carrier_phase_arc
                .as_ref()
                .expect("continuous carrier-phase arc")
                .id,
            first_arc.id
        );
        assert_eq!(
            observations[2].sats[0]
                .metadata
                .carrier_phase_arc
                .as_ref()
                .expect("second continuous carrier-phase arc")
                .id,
            first_arc.id
        );
    }

    #[test]
    fn observations_reset_carrier_phase_arc_after_unlock() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            ..ReceiverPipelineConfig::default()
        };
        let carrier_hz = crate::pipeline::doppler::carrier_hz_from_doppler_hz(0.0, 100.0);
        let unlocked_epoch = TrackEpoch {
            epoch: Epoch { index: 71 },
            sample_index: epoch_sample_index(&config, 71),
            source_time: ReceiverSampleTrace::from_sample_index(
                epoch_sample_index(&config, 71),
                config.sampling_freq_hz,
            ),
            sat: SatId { constellation: Constellation::Gps, prn: 10 },
            carrier_hz: Hertz(carrier_hz),
            lock: false,
            pll_lock: false,
            dll_lock: false,
            fll_lock: false,
            lock_state: "lost".to_string(),
            ..TrackEpoch::default()
        };
        let track = TrackingResult {
            sat: SatId { constellation: Constellation::Gps, prn: 10 },
            carrier_hz: carrier_hz,
            code_phase_samples: 0.0,
            acquisition_hypothesis: "accepted".to_string(),
            acquisition_score: 1.0,
            acquisition_code_phase_samples: 0,
            acquisition_carrier_hz: carrier_hz,
            acq_to_track_state: "accepted".to_string(),
            epochs: vec![
                make_tracking_epoch_with_phase(10, &config, 70, carrier_hz, 8.0),
                unlocked_epoch,
                make_tracking_epoch_with_phase(10, &config, 72, carrier_hz, 0.5),
            ],
            transitions: Vec::new(),
        };

        let report = observations_from_tracking_results(&config, &[track], 10);
        let unlocked = &report.output[1].sats[0];
        let relocked = &report.output[2].sats[0];

        assert_eq!(unlocked.metadata.carrier_phase_continuity, "unusable");
        assert_eq!(unlocked.metadata.carrier_phase_arc_start_epoch_idx, 0);
        let unusable_arc =
            unlocked.metadata.carrier_phase_arc.as_ref().expect("unusable carrier-phase boundary");
        assert_eq!(unusable_arc.start_reason, "loss_of_lock");
        assert!(!unusable_arc.valid_for_smoothing);
        assert!(!unusable_arc.valid_for_ambiguity);
        assert_eq!(relocked.metadata.carrier_phase_continuity, "reset_after_unlock");
        assert_eq!(relocked.metadata.carrier_phase_arc_start_epoch_idx, 72);
        assert_eq!(
            relocked.metadata.carrier_phase_arc_start_sample_index,
            epoch_sample_index(&config, 72)
        );
        let relocked_arc =
            relocked.metadata.carrier_phase_arc.as_ref().expect("relocked carrier-phase arc");
        assert_eq!(relocked_arc.start_reason, "loss_of_lock");
        assert!(relocked_arc.valid_for_smoothing);
        assert_ne!(unusable_arc.id, relocked_arc.id);
        assert!(relocked.lock_flags.cycle_slip);
        let evidence = relocked.metadata.cycle_slip_evidence.as_ref().expect("cycle-slip evidence");
        assert!(evidence.detected);
        assert_eq!(evidence.primary_reason.as_deref(), Some("loss_of_lock"));
        assert!(evidence.triggered_detectors().contains(&CycleSlipDetector::TrackingLock));
        assert!((relocked.carrier_phase_cycles.0 - 0.5).abs() <= f64::EPSILON);
    }

    #[test]
    fn observations_record_phase_innovation_cycle_slip_evidence() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            ..ReceiverPipelineConfig::default()
        };
        let carrier_hz = crate::pipeline::doppler::carrier_hz_from_doppler_hz(0.0, 125.0);
        let epochs = vec![
            make_tracking_epoch_with_phase(11, &config, 70, carrier_hz, 10.0),
            make_tracking_epoch_with_phase(11, &config, 71, carrier_hz, 10.125),
            make_tracking_epoch_with_phase(11, &config, 72, carrier_hz, 10.750),
        ];

        let (observations, diagnostics) = observations_from_tracking(&config, &epochs);

        assert!(diagnostics.is_empty(), "unexpected diagnostics: {diagnostics:?}");
        let slipped = &observations[2].sats[0];
        let evidence = slipped.metadata.cycle_slip_evidence.as_ref().expect("cycle-slip evidence");
        assert!(slipped.lock_flags.cycle_slip);
        assert!(evidence.detected);
        assert_eq!(evidence.primary_reason.as_deref(), Some("carrier_phase_discontinuity"));
        assert!(
            evidence.triggered_detectors().contains(&CycleSlipDetector::DopplerPredictedPhase),
            "{evidence:?}"
        );
        assert!(
            evidence.triggered_detectors().contains(&CycleSlipDetector::PhaseInnovation),
            "{evidence:?}"
        );
        assert_eq!(evidence.detection_probability_budget, CYCLE_SLIP_DETECTION_PROBABILITY_BUDGET);
        assert_eq!(
            evidence.false_alarm_probability_budget,
            CYCLE_SLIP_FALSE_ALARM_PROBABILITY_BUDGET
        );
    }

    #[test]
    fn observations_coast_carrier_phase_arc_through_recoverable_fade() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            ..ReceiverPipelineConfig::default()
        };
        let carrier_hz = crate::pipeline::doppler::carrier_hz_from_doppler_hz(0.0, 125.0);
        let faded_epoch = TrackEpoch {
            lock: false,
            pll_lock: false,
            dll_lock: false,
            fll_lock: false,
            lock_state: "degraded".to_string(),
            lock_state_reason: Some("signal_fade".to_string()),
            ..make_tracking_epoch_with_phase(13, &config, 71, carrier_hz, 10.125)
        };
        let recovered_epoch = TrackEpoch {
            lock_state_reason: Some("fade_recovered".to_string()),
            ..make_tracking_epoch_with_phase(13, &config, 72, carrier_hz, 10.250)
        };
        let track = TrackingResult {
            sat: SatId { constellation: Constellation::Gps, prn: 13 },
            carrier_hz,
            code_phase_samples: 0.0,
            acquisition_hypothesis: "accepted".to_string(),
            acquisition_score: 1.0,
            acquisition_code_phase_samples: 0,
            acquisition_carrier_hz: carrier_hz,
            acq_to_track_state: "accepted".to_string(),
            epochs: vec![
                make_tracking_epoch_with_phase(13, &config, 70, carrier_hz, 10.0),
                faded_epoch,
                recovered_epoch,
            ],
            transitions: Vec::new(),
        };

        let report = observations_from_tracking_results(&config, &[track], 10);
        let first = &report.output[0].sats[0];
        let faded = &report.output[1].sats[0];
        let recovered = &report.output[2].sats[0];

        assert_eq!(first.metadata.carrier_phase_continuity, "arc_start");
        assert_eq!(faded.metadata.carrier_phase_continuity, "coasted");
        assert!(!faded.lock_flags.cycle_slip);
        let first_arc = first.metadata.carrier_phase_arc.as_ref().expect("first arc");
        assert_eq!(
            faded.metadata.carrier_phase_arc_start_sample_index,
            first.metadata.carrier_phase_arc_start_sample_index
        );
        assert_eq!(
            faded.metadata.carrier_phase_arc.as_ref().expect("coasted carrier-phase arc").id,
            first_arc.id
        );
        assert_eq!(recovered.metadata.carrier_phase_continuity, "continuous");
        assert!(!recovered.lock_flags.cycle_slip);
        assert_eq!(
            recovered.metadata.carrier_phase_arc_start_sample_index,
            first.metadata.carrier_phase_arc_start_sample_index
        );
        assert_eq!(
            recovered.metadata.carrier_phase_arc.as_ref().expect("recovered carrier-phase arc").id,
            first_arc.id
        );
    }

    #[test]
    fn observations_start_new_carrier_phase_arc_after_reacquisition() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            ..ReceiverPipelineConfig::default()
        };
        let carrier_hz = crate::pipeline::doppler::carrier_hz_from_doppler_hz(0.0, 100.0);
        let lost_epoch = TrackEpoch {
            epoch: Epoch { index: 71 },
            sample_index: epoch_sample_index(&config, 71),
            source_time: ReceiverSampleTrace::from_sample_index(
                epoch_sample_index(&config, 71),
                config.sampling_freq_hz,
            ),
            sat: SatId { constellation: Constellation::Gps, prn: 14 },
            carrier_hz: Hertz(carrier_hz),
            lock: false,
            pll_lock: false,
            dll_lock: false,
            fll_lock: false,
            lock_state: "lost".to_string(),
            lock_state_reason: Some("prompt_power_drop".to_string()),
            ..TrackEpoch::default()
        };
        let reacquired_epoch = TrackEpoch {
            lock_state_reason: Some("reacquired".to_string()),
            ..make_tracking_epoch_with_phase(14, &config, 72, carrier_hz, 0.5)
        };
        let settled_epoch = make_tracking_epoch_with_phase(14, &config, 73, carrier_hz, 0.6);
        let track = TrackingResult {
            sat: SatId { constellation: Constellation::Gps, prn: 14 },
            carrier_hz,
            code_phase_samples: 0.0,
            acquisition_hypothesis: "accepted".to_string(),
            acquisition_score: 1.0,
            acquisition_code_phase_samples: 0,
            acquisition_carrier_hz: carrier_hz,
            acq_to_track_state: "accepted".to_string(),
            epochs: vec![
                make_tracking_epoch_with_phase(14, &config, 70, carrier_hz, 8.0),
                lost_epoch,
                reacquired_epoch,
                settled_epoch,
            ],
            transitions: Vec::new(),
        };

        let report = observations_from_tracking_results(&config, &[track], 10);
        let lost = &report.output[1].sats[0];
        let reacquired = &report.output[2].sats[0];
        let settled = &report.output[3].sats[0];

        assert_eq!(lost.metadata.carrier_phase_continuity, "unusable");
        assert_eq!(reacquired.metadata.carrier_phase_continuity, "reset_after_reacquisition");
        assert_eq!(reacquired.metadata.carrier_phase_arc_start_epoch_idx, 72);
        let lost_arc =
            lost.metadata.carrier_phase_arc.as_ref().expect("lost carrier-phase boundary");
        let reacquired_arc =
            reacquired.metadata.carrier_phase_arc.as_ref().expect("reacquired carrier-phase arc");
        assert_eq!(lost_arc.start_reason, "loss_of_lock");
        assert!(!lost_arc.valid_for_smoothing);
        assert_eq!(reacquired_arc.start_reason, "reacquired");
        assert!(reacquired_arc.valid_for_smoothing);
        assert_ne!(lost_arc.id, reacquired_arc.id);
        assert!(reacquired.lock_flags.cycle_slip);
        assert_eq!(settled.metadata.carrier_phase_continuity, "continuous");
        assert_eq!(settled.metadata.carrier_phase_arc_start_epoch_idx, 72);
        assert_eq!(
            settled.metadata.carrier_phase_arc.as_ref().expect("settled carrier-phase arc").id,
            reacquired_arc.id
        );
    }

    #[test]
    fn observations_reset_carrier_phase_arc_after_cycle_slip() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            ..ReceiverPipelineConfig::default()
        };
        let carrier_hz = crate::pipeline::doppler::carrier_hz_from_doppler_hz(0.0, 125.0);
        let slip_epoch = TrackEpoch {
            cycle_slip: true,
            cycle_slip_reason: Some("phase_jump".to_string()),
            ..make_tracking_epoch_with_phase(12, &config, 71, carrier_hz, 21.0)
        };
        let track = TrackingResult {
            sat: SatId { constellation: Constellation::Gps, prn: 12 },
            carrier_hz,
            code_phase_samples: 0.0,
            acquisition_hypothesis: "accepted".to_string(),
            acquisition_score: 1.0,
            acquisition_code_phase_samples: 0,
            acquisition_carrier_hz: carrier_hz,
            acq_to_track_state: "accepted".to_string(),
            epochs: vec![
                make_tracking_epoch_with_phase(12, &config, 70, carrier_hz, 10.0),
                slip_epoch,
                make_tracking_epoch_with_phase(12, &config, 72, carrier_hz, 21.125),
            ],
            transitions: Vec::new(),
        };

        let report = observations_from_tracking_results(&config, &[track], 10);
        let slipped = &report.output[1].sats[0];
        let post_slip = &report.output[2].sats[0];

        assert_eq!(slipped.metadata.carrier_phase_continuity, "reset_after_cycle_slip");
        assert_eq!(slipped.metadata.carrier_phase_arc_start_epoch_idx, 71);
        let slipped_arc =
            slipped.metadata.carrier_phase_arc.as_ref().expect("slipped carrier-phase arc");
        assert_eq!(slipped_arc.start_reason, "phase_jump");
        assert!(slipped_arc.valid_for_smoothing);
        assert!(slipped.lock_flags.cycle_slip);
        assert_eq!(post_slip.metadata.carrier_phase_continuity, "continuous");
        assert_eq!(post_slip.metadata.carrier_phase_arc_start_epoch_idx, 71);
        assert_eq!(
            post_slip.metadata.carrier_phase_arc_start_sample_index,
            epoch_sample_index(&config, 71)
        );
        assert_eq!(
            post_slip.metadata.carrier_phase_arc.as_ref().expect("post-slip carrier-phase arc").id,
            slipped_arc.id
        );
    }

    #[test]
    fn observations_advance_hatch_metadata_during_continuous_lock() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 1_023_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            ..ReceiverPipelineConfig::default()
        };
        let carrier_hz = crate::pipeline::doppler::carrier_hz_from_doppler_hz(0.0, 125.0);
        let track = TrackingResult {
            sat: SatId { constellation: Constellation::Gps, prn: 18 },
            carrier_hz,
            code_phase_samples: 0.0,
            acquisition_hypothesis: "accepted".to_string(),
            acquisition_score: 1.0,
            acquisition_code_phase_samples: 0,
            acquisition_carrier_hz: carrier_hz,
            acq_to_track_state: "accepted".to_string(),
            epochs: vec![
                make_tracking_epoch_with_alignment(18, &config, 70, carrier_hz, 10.0, 68, 0.0),
                make_tracking_epoch_with_alignment(18, &config, 71, carrier_hz, 10.125, 68, 0.0),
                make_tracking_epoch_with_alignment(18, &config, 72, carrier_hz, 10.250, 68, 0.0),
            ],
            transitions: Vec::new(),
        };

        let report = observations_from_tracking_results(&config, &[track], 3);
        let sats = report
            .output
            .iter()
            .map(|epoch| epoch.sats.first().expect("observation satellite"))
            .collect::<Vec<_>>();

        assert_eq!(sats.len(), 3);
        assert!(sats.iter().all(|sat| sat.metadata.smoothing_window == 3));
        assert_eq!(
            sats.iter().map(|sat| sat.metadata.smoothing_age).collect::<Vec<_>>(),
            vec![1, 2, 3]
        );
        assert!(sats.iter().all(|sat| sat.metadata.smoothing_resets == 0));
    }

    #[test]
    fn observations_leave_hatch_smoothing_uninitialized_during_unlock() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 1_023_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            ..ReceiverPipelineConfig::default()
        };
        let carrier_hz = crate::pipeline::doppler::carrier_hz_from_doppler_hz(0.0, 125.0);
        let unlocked_epoch = TrackEpoch {
            lock: false,
            pll_lock: false,
            dll_lock: false,
            fll_lock: false,
            lock_state: "lost".to_string(),
            lock_state_reason: Some("prompt_power_drop".to_string()),
            ..make_tracking_epoch_with_alignment(19, &config, 71, carrier_hz, 10.125, 68, 0.0)
        };
        let track = TrackingResult {
            sat: SatId { constellation: Constellation::Gps, prn: 19 },
            carrier_hz,
            code_phase_samples: 0.0,
            acquisition_hypothesis: "accepted".to_string(),
            acquisition_score: 1.0,
            acquisition_code_phase_samples: 0,
            acquisition_carrier_hz: carrier_hz,
            acq_to_track_state: "accepted".to_string(),
            epochs: vec![
                make_tracking_epoch_with_alignment(19, &config, 70, carrier_hz, 10.0, 68, 0.0),
                unlocked_epoch,
                make_tracking_epoch_with_alignment(19, &config, 72, carrier_hz, 10.250, 68, 0.0),
            ],
            transitions: Vec::new(),
        };

        let report = observations_from_tracking_results(&config, &[track], 10);
        let locked = &report.output[0].sats[0];
        let unlocked = &report.output[1].sats[0];
        let relocked = &report.output[2].sats[0];
        let expected_raw = aligned_pseudorange_m(&config, 68, 0.0);

        assert_eq!(locked.metadata.smoothing_age, 1);
        assert_eq!(locked.metadata.smoothing_resets, 0);
        assert_eq!(unlocked.metadata.smoothing_age, 0);
        assert_eq!(unlocked.metadata.smoothing_resets, 1);
        assert!((unlocked.pseudorange_m.0 - expected_raw).abs() <= f64::EPSILON);
        assert_eq!(relocked.metadata.smoothing_age, 1);
        assert_eq!(relocked.metadata.smoothing_resets, 1);
        assert!((relocked.pseudorange_m.0 - expected_raw).abs() <= f64::EPSILON);
    }

    #[test]
    fn observations_restart_hatch_smoothing_on_detected_cycle_slip() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 1_023_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            ..ReceiverPipelineConfig::default()
        };
        let carrier_hz = crate::pipeline::doppler::carrier_hz_from_doppler_hz(0.0, 125.0);
        let track = TrackingResult {
            sat: SatId { constellation: Constellation::Gps, prn: 20 },
            carrier_hz,
            code_phase_samples: 0.0,
            acquisition_hypothesis: "accepted".to_string(),
            acquisition_score: 1.0,
            acquisition_code_phase_samples: 0,
            acquisition_carrier_hz: carrier_hz,
            acq_to_track_state: "accepted".to_string(),
            epochs: vec![
                make_tracking_epoch_with_alignment(20, &config, 70, carrier_hz, 10.0, 68, 0.0),
                make_tracking_epoch_with_alignment(20, &config, 71, carrier_hz, 10.125, 68, 0.0),
                make_tracking_epoch_with_alignment(20, &config, 72, carrier_hz, 10.250, 68, 0.02),
                make_tracking_epoch_with_alignment(20, &config, 73, carrier_hz, 10.375, 68, 0.02),
            ],
            transitions: Vec::new(),
        };

        let report = observations_from_tracking_results(&config, &[track], 10);
        let sats = report.output.iter().map(|epoch| &epoch.sats[0]).collect::<Vec<_>>();
        let slipped = sats[2];
        let post_slip = sats[3];
        let expected_raw = aligned_pseudorange_m(&config, 68, 0.02);

        assert_eq!(
            sats.iter().map(|sat| sat.metadata.smoothing_age).collect::<Vec<_>>(),
            vec![1, 2, 1, 2]
        );
        assert_eq!(
            sats.iter().map(|sat| sat.metadata.smoothing_resets).collect::<Vec<_>>(),
            vec![0, 0, 1, 1]
        );
        assert!(slipped.lock_flags.cycle_slip);
        let evidence = slipped.metadata.cycle_slip_evidence.as_ref().expect("cycle-slip evidence");
        let detectors = evidence.triggered_detectors();
        let divergence = evidence
            .contributors
            .iter()
            .find(|contributor| contributor.detector == CycleSlipDetector::CodeCarrierDivergence)
            .expect("code-carrier divergence contributor");
        assert!(detectors.contains(&CycleSlipDetector::CodeCarrierDivergence));
        assert!(divergence.triggered);
        assert!(
            divergence.value.expect("divergence value")
                > divergence.threshold.expect("divergence threshold")
        );
        assert_eq!(divergence.units, "m");
        assert_eq!(evidence.primary_reason.as_deref(), Some("code_carrier_divergence"));
        assert_eq!(evidence.detection_probability_budget, CYCLE_SLIP_DETECTION_PROBABILITY_BUDGET);
        assert_eq!(
            evidence.false_alarm_probability_budget,
            CYCLE_SLIP_FALSE_ALARM_PROBABILITY_BUDGET
        );
        assert!((slipped.pseudorange_m.0 - expected_raw).abs() <= f64::EPSILON);
        assert_eq!(post_slip.metadata.smoothing_age, 2);
    }

    #[test]
    fn observations_refuse_hatch_smoothing_across_invalid_carrier_phase_arc() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 1_023_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            ..ReceiverPipelineConfig::default()
        };
        let carrier_hz = crate::pipeline::doppler::carrier_hz_from_doppler_hz(0.0, 125.0);
        let carrier_invalid = TrackEpoch {
            pll_lock: false,
            lock_state: "degraded".to_string(),
            lock_state_reason: Some("carrier_loop_unstable".to_string()),
            ..make_tracking_epoch_with_alignment(21, &config, 71, carrier_hz, 10.125, 68, 0.01)
        };
        let track = TrackingResult {
            sat: SatId { constellation: Constellation::Gps, prn: 21 },
            carrier_hz,
            code_phase_samples: 0.0,
            acquisition_hypothesis: "accepted".to_string(),
            acquisition_score: 1.0,
            acquisition_code_phase_samples: 0,
            acquisition_carrier_hz: carrier_hz,
            acq_to_track_state: "accepted".to_string(),
            epochs: vec![
                make_tracking_epoch_with_alignment(21, &config, 70, carrier_hz, 10.0, 68, 0.0),
                carrier_invalid,
                make_tracking_epoch_with_alignment(21, &config, 72, carrier_hz, 0.5, 68, 0.02),
            ],
            transitions: Vec::new(),
        };

        let report = observations_from_tracking_results(&config, &[track], 10);
        let sats = report.output.iter().map(|epoch| &epoch.sats[0]).collect::<Vec<_>>();
        let first_arc =
            sats[0].metadata.carrier_phase_arc.as_ref().expect("first carrier-phase arc");
        let invalid_arc =
            sats[1].metadata.carrier_phase_arc.as_ref().expect("invalid carrier-phase boundary");
        let reset_arc =
            sats[2].metadata.carrier_phase_arc.as_ref().expect("reset carrier-phase arc");

        assert_eq!(
            sats.iter().map(|sat| sat.metadata.smoothing_age).collect::<Vec<_>>(),
            vec![1, 0, 1]
        );
        assert_eq!(
            sats.iter().map(|sat| sat.metadata.smoothing_resets).collect::<Vec<_>>(),
            vec![0, 1, 1]
        );
        assert_eq!(sats[1].metadata.carrier_phase_continuity, "unusable");
        assert_eq!(invalid_arc.start_reason, "loss_of_lock");
        assert!(!invalid_arc.valid_for_smoothing);
        assert_ne!(first_arc.id, invalid_arc.id);
        assert_eq!(sats[2].metadata.carrier_phase_continuity, "reset_after_unlock");
        assert!(reset_arc.valid_for_smoothing);
        assert_ne!(invalid_arc.id, reset_arc.id);
    }

    #[test]
    fn tracking_time_tag_prefers_last_locked_sample() {
        let locked = TrackEpoch {
            epoch: Epoch { index: 0 },
            sample_index: 100,
            sat: SatId { constellation: Constellation::Gps, prn: 1 },
            lock_state: "tracking".to_string(),
            lock: true,
            ..TrackEpoch::default()
        };
        let unlocked_after = TrackEpoch {
            epoch: Epoch { index: 1 },
            sample_index: 101,
            sat: SatId { constellation: Constellation::Gps, prn: 1 },
            lock_state: "lost".to_string(),
            lock: false,
            ..TrackEpoch::default()
        };
        let mut last_locked = None;
        let (source_locked, sample_locked) = tracking_time_tag(&locked, &mut last_locked);
        let (source_unlocked, sample_unlocked) =
            tracking_time_tag(&unlocked_after, &mut last_locked);

        assert_eq!(source_locked, "tracking");
        assert_eq!(sample_locked, 100);
        assert_eq!(source_unlocked, "tracking_last_locked");
        assert_eq!(sample_unlocked, 100);
    }

    #[test]
    fn observation_manifest_uses_stable_model_version_and_epoch_key() {
        let config = ReceiverPipelineConfig::default();
        let tracks = vec![make_track(1, &config), make_track(2, &config)];
        let report = observations_from_tracking_results(&config, &tracks, 10);
        assert!(!report.output.is_empty());
        for epoch in &report.output {
            let manifest = epoch.manifest.as_ref().expect("manifest");
            assert_eq!(manifest.version, bijux_gnss_core::api::OBSERVATION_MODEL_VERSION);
            assert_eq!(manifest.epoch_id, bijux_gnss_core::api::obs_epoch_stability_key(epoch));
        }
    }

    #[test]
    fn grouped_observation_epochs_preserve_common_receiver_sample_trace() {
        let config = ReceiverPipelineConfig::default();
        let epoch_idx = 70;
        let expected_sample_index = epoch_sample_index(&config, epoch_idx);
        let tracks = vec![make_track(3, &config), make_track(8, &config)];

        let report = observations_from_tracking_results(&config, &tracks, 10);
        let epoch = report.output.iter().find(|epoch| epoch.epoch_idx == epoch_idx).expect("epoch");
        let manifest = epoch.manifest.as_ref().expect("manifest");

        assert_eq!(epoch.sats.len(), 2);
        assert_eq!(epoch.source_time.sample_index, expected_sample_index);
        assert_eq!(epoch.source_time.sample_rate_hz, config.sampling_freq_hz);
        assert_eq!(epoch.t_rx_s, epoch.source_time.receiver_time_s);
        assert_eq!(manifest.source_sample_index, expected_sample_index);
        assert_eq!(manifest.source_time, epoch.source_time);
    }

    #[test]
    fn grouped_observation_epochs_reject_mixed_receiver_times() {
        let config = ReceiverPipelineConfig::default();
        let expected_sample_index = epoch_sample_index(&config, 70);
        let shifted_sample_index = epoch_sample_index(&config, 71);
        let mut shifted_epoch = make_tracking_epoch(8, &config, 70, 0.0);
        shifted_epoch.sample_index = shifted_sample_index;
        shifted_epoch.source_time =
            ReceiverSampleTrace::from_sample_index(shifted_sample_index, config.sampling_freq_hz);

        let report = observations_from_tracking_results(
            &config,
            &[make_track(3, &config), track_from_epoch(shifted_epoch)],
            10,
        );
        let epoch = report.output.iter().find(|epoch| epoch.epoch_idx == 70).expect("epoch");
        let shifted_sat = epoch.sats.iter().find(|sat| sat.signal_id.sat.prn == 8).expect("sat");

        assert_eq!(epoch.source_time.sample_index, expected_sample_index);
        assert_eq!(epoch.decision, ObservationEpochDecision::Rejected);
        assert_eq!(epoch.decision_reason.as_deref(), Some("inconsistent_observable"));
        assert_eq!(shifted_sat.observation_status, ObservationStatus::Inconsistent);
        assert!(shifted_sat
            .observation_reject_reasons
            .iter()
            .any(|reason| reason == "receiver_time_mismatch"));
        assert!(shifted_sat
            .observation_reject_reasons
            .iter()
            .any(|reason| reason == "receiver_sample_trace_mismatch"));
        assert!(report
            .events
            .iter()
            .any(|event| event.code == "OBS_GROUPED_RECEIVER_TIME_MISMATCH"));
    }

    #[test]
    fn observations_accept_expected_epoch_timing_interval() {
        let config = ReceiverPipelineConfig {
            tracking_integration_ms: 10,
            ..ReceiverPipelineConfig::default()
        };
        let base_epoch_idx = 70;
        let base_sample_index = epoch_sample_index(&config, base_epoch_idx);
        let next_epoch_idx = base_epoch_idx + 1;
        let mut next_epoch = make_tracking_epoch(3, &config, next_epoch_idx, 0.0);
        next_epoch.sample_index = observation_sample_index(&config, base_sample_index, 1);
        next_epoch.source_time = ReceiverSampleTrace::from_sample_index(
            next_epoch.sample_index,
            config.sampling_freq_hz,
        );

        let report = observations_from_tracking_results(
            &config,
            &[
                track_from_epoch(make_tracking_epoch(3, &config, base_epoch_idx, 0.0)),
                track_from_epoch(next_epoch),
            ],
            10,
        );

        assert_eq!(report.output.len(), 2);
        assert!(report.events.iter().all(|event| event.code != "GNSS_OBS_TIME_INTERVAL_INVALID"));
        assert!(report.output.iter().all(|epoch| epoch.valid));
    }

    #[test]
    fn observations_reject_invalid_epoch_timing_interval() {
        let config = ReceiverPipelineConfig {
            tracking_integration_ms: 10,
            ..ReceiverPipelineConfig::default()
        };
        let base_epoch_idx = 70;
        let base_sample_index = epoch_sample_index(&config, base_epoch_idx);
        let next_epoch_idx = base_epoch_idx + 1;
        let mut next_epoch = make_tracking_epoch(3, &config, next_epoch_idx, 0.0);
        next_epoch.sample_index = observation_sample_index(&config, base_sample_index, 1)
            + observation_interval_samples(&config) / 10;
        next_epoch.source_time = ReceiverSampleTrace::from_sample_index(
            next_epoch.sample_index,
            config.sampling_freq_hz,
        );

        let report = observations_from_tracking_results(
            &config,
            &[
                track_from_epoch(make_tracking_epoch(3, &config, base_epoch_idx, 0.0)),
                track_from_epoch(next_epoch),
            ],
            10,
        );
        let epoch =
            report.output.iter().find(|epoch| epoch.epoch_idx == next_epoch_idx).expect("epoch");

        assert!(!epoch.valid);
        assert_eq!(epoch.decision, ObservationEpochDecision::Rejected);
        assert_eq!(epoch.decision_reason.as_deref(), Some("invalid_observation_timing"));
        assert!(report.events.iter().any(|event| event.code == "GNSS_OBS_TIME_INTERVAL_INVALID"));
    }

    #[test]
    fn observations_stamp_receive_and_transmit_gps_times_from_anchor() {
        let config = ReceiverPipelineConfig::default();
        let mut track = make_track(4, &config);
        track.epochs[0].signal_delay_alignment = Some(SignalDelayAlignment {
            whole_code_periods: 70,
            sample_delay_samples: 0,
            source: "synthetic_truth".to_string(),
        });
        let capture_start_gps_time = GpsTime { week: 2200, tow_s: 345_600.0 };

        let report = observations_from_tracking_results_with_gps_anchor(
            &config,
            Some(capture_start_gps_time),
            &[track],
            10,
        );
        let epoch = report.output.first().expect("observation epoch");
        let sat = epoch.sats.first().expect("observation satellite");
        let timing = sat.timing.expect("signal timing");

        assert_eq!(epoch.gps_week, Some(2200));
        let tow_s = epoch.tow_s.expect("epoch tow");
        assert!((tow_s.0 - 345_600.07).abs() <= 1.0e-6);
        assert!(
            (timing.signal_travel_time_s.0 - sat.pseudorange_m.0 / SPEED_OF_LIGHT_MPS).abs()
                <= 1.0e-12
        );
        assert_eq!(timing.transmit_gps_time.week, 2200);
        assert!((timing.transmit_gps_time.tow_s - 345_600.0).abs() <= 1.0e-9);
    }

    #[test]
    fn observations_apply_receiver_clock_model_to_observables() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            receiver_clock_bias_s: 2.0e-6,
            receiver_clock_frequency_bias_hz: 17.5,
            receiver_clock_bias_sigma_s: 3.0e-8,
            receiver_clock_source: "synthetic_receiver_clock".to_string(),
            ..ReceiverPipelineConfig::default()
        };
        let raw_carrier_phase_cycles = 125.25;
        let raw_doppler_hz = 100.0;
        let carrier_hz = crate::pipeline::doppler::carrier_hz_from_doppler_hz(0.0, raw_doppler_hz);
        let track = track_from_epoch(make_tracking_epoch_with_alignment(
            6,
            &config,
            70,
            carrier_hz,
            raw_carrier_phase_cycles,
            68,
            128.0,
        ));

        let report = observations_from_tracking_results(&config, &[track], 10);
        let epoch = report.output.first().expect("observation epoch");
        let sat = epoch.sats.first().expect("observation satellite");
        let error_model = sat.error_model.as_ref().expect("error model");
        let expected_pseudorange_m =
            aligned_pseudorange_m(&config, 68, 128.0) + 2.0e-6 * SPEED_OF_LIGHT_MPS;
        let expected_carrier_phase_cycles =
            raw_carrier_phase_cycles + 2.0e-6 * GPS_L1_CA_CARRIER_HZ.value();

        assert!((epoch.t_rx_s.0 - (70.0e-3 + 2.0e-6)).abs() <= 1.0e-12);
        assert!((sat.pseudorange_m.0 - expected_pseudorange_m).abs() <= 1.0e-6);
        assert!((sat.carrier_phase_cycles.0 - expected_carrier_phase_cycles).abs() <= 1.0e-6);
        assert!((sat.doppler_hz.0 - (raw_doppler_hz + 17.5)).abs() <= f64::EPSILON);
        assert!((error_model.clock_error_m.0 - 3.0e-8 * SPEED_OF_LIGHT_MPS).abs() <= 1.0e-12);
        assert_eq!(sat.metadata.receiver_clock_bias_s, Seconds(2.0e-6));
        assert_eq!(sat.metadata.receiver_clock_frequency_bias_hz, 17.5);
        assert_eq!(sat.metadata.receiver_clock_bias_sigma_s, Seconds(3.0e-8));
        assert_eq!(sat.metadata.receiver_clock_source, "synthetic_receiver_clock");
    }

    #[test]
    fn hatch_smoothed_observations_preserve_receiver_clock_uncertainty() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            receiver_clock_bias_sigma_s: 4.0e-8,
            ..ReceiverPipelineConfig::default()
        };
        let carrier_hz = crate::pipeline::doppler::carrier_hz_from_doppler_hz(0.0, 100.0);
        let first =
            make_tracking_epoch_with_alignment(6, &config, 70, carrier_hz, 125.25, 68, 128.0);
        let second =
            make_tracking_epoch_with_alignment(6, &config, 71, carrier_hz, 126.25, 68, 128.0);

        let report = observations_from_tracking_results(
            &config,
            &[TrackingResult {
                sat: first.sat,
                carrier_hz,
                code_phase_samples: first.code_phase_samples.0,
                acquisition_hypothesis: "accepted".to_string(),
                acquisition_score: 1.0,
                acquisition_code_phase_samples: 0,
                acquisition_carrier_hz: carrier_hz,
                acq_to_track_state: "accepted".to_string(),
                epochs: vec![first, second],
                transitions: Vec::new(),
            }],
            10,
        );
        let smoothed_sat = report.output[1].sats.first().expect("smoothed observation satellite");
        let error_model = smoothed_sat.error_model.as_ref().expect("error model");

        assert!(smoothed_sat.metadata.smoothing_age > 0);
        assert!((error_model.clock_error_m.0 - 4.0e-8 * SPEED_OF_LIGHT_MPS).abs() <= 1.0e-12);
    }

    #[test]
    fn observations_without_alignment_keep_fallback_model_and_omit_signal_timing() {
        let config = ReceiverPipelineConfig::default();
        let track = make_track(5, &config);
        let capture_start_gps_time = GpsTime { week: 2200, tow_s: 345_600.0 };

        let report = observations_from_tracking_results_with_gps_anchor(
            &config,
            Some(capture_start_gps_time),
            &[track],
            10,
        );
        let sat = report.output.first().expect("observation epoch").sats.first().expect("sat");

        assert_eq!(sat.metadata.pseudorange_model, "receiver_epoch_fallback");
        assert_eq!(sat.metadata.observation_support_class, "degraded");
        assert!(sat.metadata.signal_delay_alignment_source.is_empty());
        assert!(sat.timing.is_none());
    }

    #[test]
    fn pseudorange_resolver_recovers_integer_code_period_ambiguity_from_transmit_time() {
        let receive_gps_time = GpsTime { week: 2200, tow_s: 345_600.250 };
        let code_period_s = 0.001;
        let code_delay_s = 0.000_375;
        let expected_periods = 72;
        let decoded_transmit_time = receive_gps_time
            .offset_seconds(-(expected_periods as f64 * code_period_s + code_delay_s));

        let resolved = resolve_pseudorange_from_transmit_time(
            receive_gps_time,
            decoded_transmit_time,
            code_delay_s,
            code_period_s,
        )
        .expect("resolved pseudorange timing");

        assert_eq!(resolved.integer_code_periods, expected_periods);
        assert!((resolved.code_delay_s.0 - code_delay_s).abs() <= 1.0e-12);
        assert!(
            (resolved.signal_travel_time_s.0
                - (expected_periods as f64 * code_period_s + code_delay_s))
                .abs()
                <= 1.0e-12
        );
        assert!((resolved.transmit_gps_time.tow_s - decoded_transmit_time.tow_s).abs() <= 1.0e-12);
    }

    #[test]
    fn integer_code_period_solver_recovers_joint_vector_from_truth() {
        let receive_gps_time = GpsTime { week: 2200, tow_s: 345_600.250 };
        let code_period_s = 0.001;
        let first_delay_s = 0.000_375;
        let second_delay_s = 0.000_640;
        let first_periods = 72;
        let second_periods = 80;
        let first_signal_id = SigId {
            sat: SatId { constellation: Constellation::Gps, prn: 3 },
            band: SignalBand::L1,
            code: SignalCode::Ca,
        };
        let second_signal_id = SigId {
            sat: SatId { constellation: Constellation::Gps, prn: 9 },
            band: SignalBand::L1,
            code: SignalCode::Ca,
        };
        let inputs = vec![
            CodePeriodAmbiguityInput {
                signal_id: second_signal_id,
                receive_gps_time,
                decoded_transmit_gps_time: receive_gps_time
                    .offset_seconds(-(second_periods as f64 * code_period_s + second_delay_s)),
                code_period_s,
                code_delay_s: second_delay_s,
            },
            CodePeriodAmbiguityInput {
                signal_id: first_signal_id,
                receive_gps_time,
                decoded_transmit_gps_time: receive_gps_time
                    .offset_seconds(-(first_periods as f64 * code_period_s + first_delay_s)),
                code_period_s,
                code_delay_s: first_delay_s,
            },
        ];

        let solution =
            resolve_integer_code_period_ambiguities(&inputs).expect("unique ambiguity solution");

        assert!(solution.common_receiver_clock_bias_s.0.abs() <= CODE_PERIOD_AMBIGUITY_EPS_S);
        assert_eq!(solution.satellites[0].signal_id, first_signal_id);
        assert_eq!(solution.satellites[0].integer_code_periods, first_periods);
        assert_eq!(solution.satellites[1].signal_id, second_signal_id);
        assert_eq!(solution.satellites[1].integer_code_periods, second_periods);
    }

    #[test]
    fn integer_code_period_solver_estimates_common_receiver_clock_bias() {
        let receive_gps_time = GpsTime { week: 2200, tow_s: 345_600.250 };
        let code_period_s = 0.001;
        let common_clock_bias_s = 0.000_120;
        let first_signal_id = SigId {
            sat: SatId { constellation: Constellation::Gps, prn: 5 },
            band: SignalBand::L1,
            code: SignalCode::Ca,
        };
        let second_signal_id = SigId {
            sat: SatId { constellation: Constellation::Gps, prn: 11 },
            band: SignalBand::L1,
            code: SignalCode::Ca,
        };
        let inputs = vec![
            CodePeriodAmbiguityInput {
                signal_id: first_signal_id,
                receive_gps_time,
                decoded_transmit_gps_time: receive_gps_time
                    .offset_seconds(-(68.0 * code_period_s + 0.000_200 + common_clock_bias_s)),
                code_period_s,
                code_delay_s: 0.000_200,
            },
            CodePeriodAmbiguityInput {
                signal_id: second_signal_id,
                receive_gps_time,
                decoded_transmit_gps_time: receive_gps_time
                    .offset_seconds(-(84.0 * code_period_s + 0.000_730 + common_clock_bias_s)),
                code_period_s,
                code_delay_s: 0.000_730,
            },
        ];

        let solution =
            resolve_integer_code_period_ambiguities(&inputs).expect("clock-biased solution");

        assert!(
            (solution.common_receiver_clock_bias_s.0 - common_clock_bias_s).abs()
                <= CODE_PERIOD_AMBIGUITY_EPS_S
        );
        assert_eq!(solution.satellites[0].integer_code_periods, 68);
        assert_eq!(solution.satellites[1].integer_code_periods, 84);
    }

    #[test]
    fn integer_code_period_solver_refuses_non_unique_boundary() {
        let receive_gps_time = GpsTime { week: 2200, tow_s: 345_600.250 };
        let code_period_s = 0.001;
        let delay_s = 0.000_250;
        let signal_id = SigId {
            sat: SatId { constellation: Constellation::Gps, prn: 4 },
            band: SignalBand::L1,
            code: SignalCode::Ca,
        };
        let decoded_transmit_gps_time =
            receive_gps_time.offset_seconds(-(72.5 * code_period_s + delay_s));
        let inputs = vec![CodePeriodAmbiguityInput {
            signal_id,
            receive_gps_time,
            decoded_transmit_gps_time,
            code_period_s,
            code_delay_s: delay_s,
        }];

        let refusal = resolve_integer_code_period_ambiguities(&inputs)
            .expect_err("half-period ambiguity must not be resolved");

        assert_eq!(refusal, CODE_PERIOD_AMBIGUITY_NON_UNIQUE);
    }

    #[test]
    fn observations_resolve_absolute_pseudorange_from_decoded_transmit_time() {
        let signal = signal_spec_gps_l1_ca();
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: signal.code_rate_hz,
            code_length: 1023,
            ..ReceiverPipelineConfig::default()
        };
        let receive_gps_time = GpsTime { week: 2200, tow_s: 345_600.250 };
        let whole_code_periods = 72;
        let aligned_code_phase_chips = 384.0;
        let code_delay_s = aligned_code_phase_chips / signal.code_rate_hz;
        let expected_signal_travel_time_s =
            whole_code_periods as f64 * (1023.0 / signal.code_rate_hz) + code_delay_s;
        let decoded_transmit_time = receive_gps_time.offset_seconds(-expected_signal_travel_time_s);
        let epoch_sample_index =
            ((receive_gps_time.tow_s - 345_600.0) * config.sampling_freq_hz).round() as u64;
        let epoch = TrackEpoch {
            sat: SatId { constellation: Constellation::Gps, prn: 7 },
            signal_band: SignalBand::L1,
            carrier_hz: Hertz(tracked_signal_center_hz(config.intermediate_freq_hz, signal)),
            code_rate_hz: Hertz(signal.code_rate_hz),
            code_phase_samples: Chips(test_tracking_code_phase_samples_for_signal(
                &config,
                signal,
                1023,
                aligned_code_phase_chips,
            )),
            transmit_time: Some(TrackingTransmitTime {
                transmit_gps_time: decoded_transmit_time,
                source: "decoded_lnav_how".to_string(),
            }),
            sample_index: epoch_sample_index,
            source_time: ReceiverSampleTrace::from_sample_index(
                epoch_sample_index,
                config.sampling_freq_hz,
            ),
            ..make_tracking_epoch_with_phase(
                7,
                &config,
                250,
                tracked_signal_center_hz(config.intermediate_freq_hz, signal),
                0.0,
            )
        };

        let report = observations_from_tracking_results_with_gps_anchor(
            &config,
            Some(GpsTime { week: 2200, tow_s: 345_600.0 }),
            &[track_from_epoch(epoch)],
            10,
        );
        let obs_sat = report.output[0].sats.first().expect("observation satellite");
        let timing = obs_sat.timing.expect("resolved signal timing");

        assert_eq!(obs_sat.metadata.pseudorange_model, "decoded_transmit_time_code_phase");
        assert_eq!(obs_sat.metadata.pseudorange_time_source, "decoded_lnav_how");
        assert_eq!(obs_sat.metadata.pseudorange_integer_code_periods, Some(whole_code_periods));
        assert_eq!(obs_sat.metadata.signal_delay_alignment_source, "");
        assert_eq!(obs_sat.metadata.observation_support_class, "supported");
        assert!(
            (obs_sat.pseudorange_m.0 - timing.signal_travel_time_s.0 * SPEED_OF_LIGHT_MPS).abs()
                <= 1.0e-6
        );
        assert!(
            (timing.signal_travel_time_s.0 - expected_signal_travel_time_s).abs()
                <= CODE_PERIOD_AMBIGUITY_EPS_S
        );
        assert!(
            (timing.transmit_gps_time.tow_s - decoded_transmit_time.tow_s).abs()
                <= CODE_PERIOD_AMBIGUITY_EPS_S
        );
    }

    #[test]
    fn decoded_transmit_time_observation_applies_receiver_clock_bias_once() {
        let signal = signal_spec_gps_l1_ca();
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: signal.code_rate_hz,
            code_length: 1023,
            receiver_clock_bias_s: 0.000_200,
            receiver_clock_source: "known_receiver_clock".to_string(),
            ..ReceiverPipelineConfig::default()
        };
        let capture_start_gps_time = GpsTime { week: 2200, tow_s: 345_600.0 };
        let true_receive_gps_time = GpsTime { week: 2200, tow_s: 345_600.250 };
        let whole_code_periods = 72;
        let aligned_code_phase_chips = 384.0;
        let physical_signal_time_s = whole_code_periods as f64 * (1023.0 / signal.code_rate_hz)
            + aligned_code_phase_chips / signal.code_rate_hz;
        let epoch = gps_l1ca_decoded_time_epoch(
            7,
            &config,
            250,
            capture_start_gps_time,
            true_receive_gps_time,
            whole_code_periods as f64,
            aligned_code_phase_chips,
        );

        let report = observations_from_tracking_results_with_gps_anchor(
            &config,
            Some(capture_start_gps_time),
            &[track_from_epoch(epoch)],
            10,
        );
        let obs_sat = report.output[0].sats.first().expect("observation satellite");
        let timing = obs_sat.timing.expect("resolved signal timing");
        let expected_observed_time_s = physical_signal_time_s + config.receiver_clock_bias_s;

        assert_eq!(obs_sat.metadata.pseudorange_integer_code_periods, Some(whole_code_periods));
        assert_eq!(obs_sat.metadata.receiver_clock_source, "known_receiver_clock");
        assert!((timing.signal_travel_time_s.0 - expected_observed_time_s).abs() <= 2.5e-7);
        assert!(
            (obs_sat.pseudorange_m.0 - expected_observed_time_s * SPEED_OF_LIGHT_MPS).abs()
                <= CODE_PERIOD_AMBIGUITY_EPS_S * SPEED_OF_LIGHT_MPS
        );
    }

    #[test]
    fn observations_apply_joint_integer_code_period_vector() {
        let signal = signal_spec_gps_l1_ca();
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: signal.code_rate_hz,
            code_length: 1023,
            ..ReceiverPipelineConfig::default()
        };
        let capture_start_gps_time = GpsTime { week: 2200, tow_s: 345_600.0 };
        let receive_gps_time = GpsTime { week: 2200, tow_s: 345_600.250 };
        let first_epoch = gps_l1ca_decoded_time_epoch(
            3,
            &config,
            250,
            capture_start_gps_time,
            receive_gps_time,
            72.0,
            384.0,
        );
        let second_epoch = gps_l1ca_decoded_time_epoch(
            9,
            &config,
            250,
            capture_start_gps_time,
            receive_gps_time,
            80.0,
            640.0,
        );

        let report = observations_from_tracking_results_with_gps_anchor(
            &config,
            Some(capture_start_gps_time),
            &[track_from_epoch(second_epoch), track_from_epoch(first_epoch)],
            10,
        );
        let epoch = report.output.first().expect("grouped observation epoch");
        let first_sat = epoch.sats.iter().find(|sat| sat.signal_id.sat.prn == 3).expect("sat");
        let second_sat = epoch.sats.iter().find(|sat| sat.signal_id.sat.prn == 9).expect("sat");
        let first_expected_s = 72.0 * (1023.0 / signal.code_rate_hz) + 384.0 / signal.code_rate_hz;
        let second_expected_s = 80.0 * (1023.0 / signal.code_rate_hz) + 640.0 / signal.code_rate_hz;

        assert_eq!(epoch.decision, ObservationEpochDecision::Accepted);
        assert_eq!(first_sat.metadata.pseudorange_integer_code_periods, Some(72));
        assert_eq!(second_sat.metadata.pseudorange_integer_code_periods, Some(80));
        assert_eq!(first_sat.metadata.observation_support_class, "supported");
        assert_eq!(second_sat.metadata.observation_support_class, "supported");
        assert!(
            (first_sat.pseudorange_m.0 - first_expected_s * SPEED_OF_LIGHT_MPS).abs()
                <= CODE_PERIOD_AMBIGUITY_EPS_S * SPEED_OF_LIGHT_MPS
        );
        assert!(
            (second_sat.pseudorange_m.0 - second_expected_s * SPEED_OF_LIGHT_MPS).abs()
                <= CODE_PERIOD_AMBIGUITY_EPS_S * SPEED_OF_LIGHT_MPS
        );
        assert!(!report
            .events
            .iter()
            .any(|event| event.code == "OBS_INTEGER_CODE_PERIOD_AMBIGUITY_REFUSED"));
    }

    #[test]
    fn observations_refuse_non_unique_integer_code_period_boundary() {
        let signal = signal_spec_gps_l1_ca();
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: signal.code_rate_hz,
            code_length: 1023,
            ..ReceiverPipelineConfig::default()
        };
        let capture_start_gps_time = GpsTime { week: 2200, tow_s: 345_600.0 };
        let receive_gps_time = GpsTime { week: 2200, tow_s: 345_600.250 };
        let epoch = gps_l1ca_decoded_time_epoch(
            4,
            &config,
            250,
            capture_start_gps_time,
            receive_gps_time,
            72.5,
            256.0,
        );

        let report = observations_from_tracking_results_with_gps_anchor(
            &config,
            Some(capture_start_gps_time),
            &[track_from_epoch(epoch)],
            10,
        );
        let epoch = report.output.first().expect("observation epoch");
        let sat = epoch.sats.first().expect("observation satellite");

        assert_eq!(epoch.decision, ObservationEpochDecision::Rejected);
        assert_eq!(epoch.decision_reason.as_deref(), Some("inconsistent_observable"));
        assert_eq!(sat.observation_status, ObservationStatus::Inconsistent);
        assert!(sat
            .observation_reject_reasons
            .iter()
            .any(|reason| reason == CODE_PERIOD_AMBIGUITY_NON_UNIQUE));
        assert_eq!(sat.metadata.observation_support_class, "unsupported");
        assert!(report.events.iter().any(|event| {
            event.code == "OBS_INTEGER_CODE_PERIOD_AMBIGUITY_REFUSED"
                && event.context.iter().any(|(key, value)| {
                    key == "reason" && value == CODE_PERIOD_AMBIGUITY_NON_UNIQUE
                })
                && event
                    .context
                    .iter()
                    .any(|(key, value)| key == "signals" && value == "Gps:04:L1:Ca")
        }));
    }

    #[test]
    fn observations_preserve_tracking_uncertainty_and_measurement_variances() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            ..ReceiverPipelineConfig::default()
        };
        let sat = SatId { constellation: Constellation::Gps, prn: 9 };
        let uncertainty = TrackingUncertainty {
            code_phase_samples: 0.25,
            carrier_phase_cycles: 0.05,
            doppler_hz: 1.5,
            cn0_dbhz: 0.75,
        };
        let track = TrackingResult {
            sat,
            carrier_hz: 0.0,
            code_phase_samples: 0.0,
            acquisition_hypothesis: "accepted".to_string(),
            acquisition_score: 1.0,
            acquisition_code_phase_samples: 0,
            acquisition_carrier_hz: 0.0,
            acq_to_track_state: "accepted".to_string(),
            epochs: vec![TrackEpoch {
                epoch: Epoch { index: 70 },
                sample_index: 70 * 4092,
                source_time: ReceiverSampleTrace::from_sample_index(
                    70 * 4092,
                    config.sampling_freq_hz,
                ),
                sat,
                lock: true,
                cn0_dbhz: 48.0,
                pll_lock: true,
                dll_lock: true,
                fll_lock: true,
                lock_state: "tracking".to_string(),
                lock_state_reason: Some("stable_tracking".to_string()),
                transmit_time: None,
                tracking_uncertainty: Some(uncertainty.clone()),
                ..TrackEpoch::default()
            }],
            transitions: Vec::new(),
        };

        let report = observations_from_tracking_results(&config, &[track], 10);
        let epoch = report.output.first().expect("observation epoch");
        let sat = epoch.sats.first().expect("observation satellite");
        let quality = observation_measurement_quality_from_epochs(&report.output);
        let quality_sat = quality[0].sats.first().expect("measurement quality satellite");
        let meters_per_sample = SPEED_OF_LIGHT_MPS / config.sampling_freq_hz;
        let expected_pseudorange_sigma_m = uncertainty.code_phase_samples * meters_per_sample;

        assert_eq!(sat.metadata.tracking_uncertainty.as_ref(), Some(&uncertainty));
        assert!(
            (sat.pseudorange_var_m2 - expected_pseudorange_sigma_m.powi(2)).abs() <= 1.0e-9,
            "{sat:?}"
        );
        assert!(
            (sat.carrier_phase_var_cycles2 - uncertainty.carrier_phase_cycles.powi(2)).abs()
                <= 1.0e-12,
            "{sat:?}"
        );
        assert!((sat.doppler_var_hz2 - uncertainty.doppler_hz.powi(2)).abs() <= 1.0e-12, "{sat:?}");
        let error_model = sat.error_model.as_ref().expect("error model");
        assert!((error_model.tracking_jitter_m.0 - expected_pseudorange_sigma_m).abs() <= 1.0e-9);
        assert_eq!(error_model.thermal_noise_m, Meters(0.0));
        let covariance = sat.measurement_covariance().expect("measurement covariance");
        let covariance_matrix = covariance.matrix();
        assert!((covariance.code_phase_m2 - sat.pseudorange_var_m2).abs() <= 1.0e-9);
        assert!((covariance.doppler_hz2 - sat.doppler_var_hz2).abs() <= 1.0e-12);
        assert_eq!(covariance_matrix[0][1], covariance_matrix[1][0]);
        assert_eq!(covariance_matrix[1][2], covariance_matrix[2][1]);
        assert!(covariance.carrier_doppler_m_hz > 0.0);
        assert_eq!(quality_sat.measurement_covariance, Some(covariance));
        assert_eq!(quality_sat.code_carrier_divergence, sat.metadata.code_carrier_divergence);
        assert_eq!(quality_sat.cycle_slip_evidence, sat.metadata.cycle_slip_evidence);
        let slip_evidence = quality_sat.cycle_slip_evidence.as_ref().expect("cycle-slip evidence");
        assert_eq!(
            slip_evidence.detection_probability_budget,
            CYCLE_SLIP_DETECTION_PROBABILITY_BUDGET
        );
        assert_eq!(
            slip_evidence.false_alarm_probability_budget,
            CYCLE_SLIP_FALSE_ALARM_PROBABILITY_BUDGET
        );
        let divergence =
            quality_sat.code_carrier_divergence.expect("code-carrier divergence quality evidence");
        assert!(divergence.raw_m.is_finite());
        assert!(divergence.jump_m.is_finite());
    }

    #[test]
    fn observations_mark_missing_variance_evidence_unusable() {
        let config = ReceiverPipelineConfig::default();
        let mut epoch = make_observation_ready_epoch(12, &config, 70);
        epoch.tracking_uncertainty = None;

        let report =
            observation_artifacts_from_tracking_results(&config, &[track_from_epoch(epoch)], 10);
        let observation_epoch = report.output.epochs.first().expect("observation epoch");
        let sat = observation_epoch.sats.first().expect("observation satellite");
        let quality = report.output.measurement_quality[0]
            .sats
            .first()
            .expect("measurement quality satellite");
        let residual = report.output.residuals[0].sats.first().expect("residual satellite");

        assert_eq!(observation_epoch.decision, ObservationEpochDecision::Rejected);
        assert_eq!(observation_epoch.decision_reason.as_deref(), Some("no_accepted_observables"));
        assert_eq!(sat.observation_status, ObservationStatus::Missing);
        assert!(sat
            .observation_reject_reasons
            .iter()
            .any(|reason| reason == "missing_tracking_uncertainty"));
        assert_eq!(sat.pseudorange_var_m2, 0.0);
        assert_eq!(sat.carrier_phase_var_cycles2, 0.0);
        assert_eq!(sat.doppler_var_hz2, 0.0);
        assert!(sat.error_model.is_none());
        assert!(sat.measurement_covariance().is_none());
        assert!(sat.covariance_pseudorange_sigma_m().is_none());
        assert_eq!(sat.metadata.observation_uncertainty_class, "unknown");
        assert!(quality.pseudorange_sigma_m.is_none());
        assert!(quality.carrier_phase_sigma_cycles.is_none());
        assert!(quality.doppler_sigma_hz.is_none());
        assert!(quality.measurement_covariance.is_none());
        assert_eq!(quality.code_carrier_divergence, sat.metadata.code_carrier_divergence);
        assert_eq!(quality.cycle_slip_evidence, sat.metadata.cycle_slip_evidence);
        assert!(residual.pseudorange_m.sigma.is_none());
        assert!(residual.carrier_phase_cycles.sigma.is_none());
        assert!(residual.doppler_hz.sigma.is_none());
    }

    #[test]
    fn observations_preserve_degraded_doppler_uncertainty() {
        let config = ReceiverPipelineConfig::default();
        let sat = SatId { constellation: Constellation::Gps, prn: 14 };
        let expected_doppler_hz = 125.0;
        let doppler_uncertainty_hz: f64 = 180.0;
        let carrier_hz = crate::pipeline::doppler::carrier_hz_from_doppler_hz(
            config.intermediate_freq_hz,
            expected_doppler_hz,
        );
        let uncertainty = TrackingUncertainty {
            code_phase_samples: 0.4,
            carrier_phase_cycles: 0.2,
            doppler_hz: doppler_uncertainty_hz,
            cn0_dbhz: 1.0,
        };
        let epoch = TrackEpoch {
            epoch: Epoch { index: 70 },
            sample_index: epoch_sample_index(&config, 70),
            source_time: ReceiverSampleTrace::from_sample_index(
                epoch_sample_index(&config, 70),
                config.sampling_freq_hz,
            ),
            sat,
            carrier_hz: Hertz(carrier_hz),
            code_rate_hz: Hertz(config.code_freq_basis_hz),
            lock: true,
            pll_lock: false,
            dll_lock: true,
            fll_lock: false,
            cn0_dbhz: 45.0,
            lock_state: "degraded".to_string(),
            lock_state_reason: Some("doppler_estimator_divergence".to_string()),
            transmit_time: None,
            tracking_uncertainty: Some(uncertainty.clone()),
            ..TrackEpoch::default()
        };
        let report = observations_from_tracking_results(&config, &[track_from_epoch(epoch)], 10);
        let observation_epoch = report.output.first().expect("observation epoch");
        let observed_sat = observation_epoch.sats.first().expect("observation satellite");

        assert_eq!(observed_sat.metadata.tracking_state, "degraded");
        assert_eq!(
            observed_sat.metadata.observation_lock_reason.as_deref(),
            Some("doppler_estimator_divergence")
        );
        assert!(!observed_sat.lock_flags.carrier_lock, "{observed_sat:?}");
        assert!(
            (observed_sat.doppler_hz.0 - expected_doppler_hz).abs() <= f64::EPSILON,
            "{observed_sat:?}"
        );
        assert_eq!(observed_sat.metadata.tracking_uncertainty.as_ref(), Some(&uncertainty));
        assert!(
            (observed_sat.doppler_var_hz2 - doppler_uncertainty_hz.powi(2)).abs() <= 1.0e-12,
            "{observed_sat:?}"
        );
    }

    #[test]
    fn observations_use_tracking_uncertainty_for_weaker_cn0() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            ..ReceiverPipelineConfig::default()
        };
        let mut strong_epoch = make_observation_ready_epoch(30, &config, 70);
        strong_epoch.cn0_dbhz = 48.0;
        set_code_phase_uncertainty(&mut strong_epoch, 0.04);
        let mut weak_epoch = make_observation_ready_epoch(31, &config, 70);
        weak_epoch.cn0_dbhz = 28.0;
        set_code_phase_uncertainty(&mut weak_epoch, 0.16);

        let strong =
            observations_from_tracking_results(&config, &[track_from_epoch(strong_epoch)], 10);
        let weak = observations_from_tracking_results(&config, &[track_from_epoch(weak_epoch)], 10);
        let strong_sat = strong.output[0].sats.first().expect("strong satellite");
        let weak_sat = weak.output[0].sats.first().expect("weak satellite");

        assert!(
            weak_sat.pseudorange_var_m2 > strong_sat.pseudorange_var_m2,
            "weaker C/N0 should inflate pseudorange variance: strong={strong_sat:?} weak={weak_sat:?}"
        );
    }

    #[test]
    fn observations_use_tracking_uncertainty_for_integration_duration() {
        let short_config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            tracking_integration_ms: 1,
            ..ReceiverPipelineConfig::default()
        };
        let long_config =
            ReceiverPipelineConfig { tracking_integration_ms: 10, ..short_config.clone() };
        let mut short_epoch = make_observation_ready_epoch(32, &short_config, 70);
        short_epoch.cn0_dbhz = 40.0;
        set_code_phase_uncertainty(&mut short_epoch, 0.14);
        let mut long_epoch = make_observation_ready_epoch(32, &long_config, 70);
        long_epoch.cn0_dbhz = 40.0;
        set_code_phase_uncertainty(&mut long_epoch, 0.05);

        let short =
            observations_from_tracking_results(&short_config, &[track_from_epoch(short_epoch)], 10);
        let long =
            observations_from_tracking_results(&long_config, &[track_from_epoch(long_epoch)], 10);
        let short_sat = short.output[0].sats.first().expect("short integration satellite");
        let long_sat = long.output[0].sats.first().expect("long integration satellite");

        assert!(
            long_sat.pseudorange_var_m2 < short_sat.pseudorange_var_m2,
            "longer coherent integration should tighten pseudorange variance: short={short_sat:?} long={long_sat:?}"
        );
    }

    #[test]
    fn observations_use_tracking_uncertainty_when_dll_lock_is_lost() {
        let config = ReceiverPipelineConfig::default();
        let mut locked_epoch = make_observation_ready_epoch(33, &config, 70);
        set_code_phase_uncertainty(&mut locked_epoch, 0.05);
        let mut unlocked_epoch = make_observation_ready_epoch(34, &config, 70);
        unlocked_epoch.dll_lock = false;
        set_code_phase_uncertainty(&mut unlocked_epoch, 0.18);

        let locked =
            observations_from_tracking_results(&config, &[track_from_epoch(locked_epoch)], 10);
        let unlocked =
            observations_from_tracking_results(&config, &[track_from_epoch(unlocked_epoch)], 10);
        let locked_sat = locked.output[0].sats.first().expect("locked satellite");
        let unlocked_sat = unlocked.output[0].sats.first().expect("unlocked satellite");

        assert!(
            unlocked_sat.pseudorange_var_m2 > locked_sat.pseudorange_var_m2,
            "loss of DLL lock should inflate pseudorange variance: locked={locked_sat:?} unlocked={unlocked_sat:?}"
        );
    }

    #[test]
    fn observations_use_tracking_uncertainty_for_low_lock_quality() {
        let config = ReceiverPipelineConfig::default();
        let mut locked_epoch = make_observation_ready_epoch(35, &config, 70);
        set_code_phase_uncertainty(&mut locked_epoch, 0.05);
        let mut guarded_epoch = make_observation_ready_epoch(36, &config, 70);
        guarded_epoch.anti_false_lock = true;
        set_code_phase_uncertainty(&mut guarded_epoch, 0.20);

        let locked =
            observations_from_tracking_results(&config, &[track_from_epoch(locked_epoch)], 10);
        let guarded =
            observations_from_tracking_results(&config, &[track_from_epoch(guarded_epoch)], 10);
        let locked_sat = locked.output[0].sats.first().expect("locked satellite");
        let guarded_sat = guarded.output[0].sats.first().expect("guarded satellite");

        assert!(
            guarded_sat.pseudorange_var_m2 > locked_sat.pseudorange_var_m2,
            "lower lock quality should inflate pseudorange variance: locked={locked_sat:?} guarded={guarded_sat:?}"
        );
    }

    #[test]
    fn observations_preserve_tracking_cn0_on_accepted_rows() {
        let config = ReceiverPipelineConfig::default();
        let sat = SatId { constellation: Constellation::Gps, prn: 10 };
        let expected_cn0_dbhz = 47.5;
        let track = TrackingResult {
            sat,
            carrier_hz: 0.0,
            code_phase_samples: 0.0,
            acquisition_hypothesis: "accepted".to_string(),
            acquisition_score: 1.0,
            acquisition_code_phase_samples: 0,
            acquisition_carrier_hz: 0.0,
            acq_to_track_state: "accepted".to_string(),
            epochs: vec![TrackEpoch {
                epoch: Epoch { index: 70 },
                sample_index: epoch_sample_index(&config, 70),
                source_time: ReceiverSampleTrace::from_sample_index(
                    epoch_sample_index(&config, 70),
                    config.sampling_freq_hz,
                ),
                sat,
                lock: true,
                cn0_dbhz: expected_cn0_dbhz,
                pll_lock: true,
                dll_lock: true,
                fll_lock: true,
                lock_state: "tracking".to_string(),
                lock_state_reason: Some("stable_tracking".to_string()),
                tracking_uncertainty: Some(test_tracking_uncertainty()),
                ..TrackEpoch::default()
            }],
            transitions: Vec::new(),
        };

        let report = observations_from_tracking_results(&config, &[track], 10);
        let epoch = report.output.first().expect("observation epoch");
        let sat = epoch.sats.first().expect("observation satellite");

        assert_eq!(sat.observation_status, ObservationStatus::Accepted);
        assert!((sat.cn0_dbhz - expected_cn0_dbhz).abs() <= f64::EPSILON, "{sat:?}");
    }

    #[test]
    fn observations_record_locked_observation_lock_state() {
        let config = ReceiverPipelineConfig::default();
        let epoch = make_observation_ready_epoch(14, &config, 70);
        let report = observations_from_tracking_results(&config, &[track_from_epoch(epoch)], 10);
        let sat = report.output[0].sats.first().expect("observation satellite");

        assert_eq!(sat.metadata.observation_lock_state, "locked");
        assert_eq!(sat.metadata.observation_lock_reason.as_deref(), Some("stable_tracking"));
        assert_eq!(sat.metadata.tracking_lock_state, sat.metadata.observation_lock_state);
    }

    #[test]
    fn observations_record_degraded_observation_lock_state() {
        let config = ReceiverPipelineConfig::default();
        let mut epoch = make_observation_ready_epoch(15, &config, 70);
        epoch.lock_state = "degraded".to_string();
        epoch.lock_state_reason = Some("signal_fade".to_string());
        let report = observations_from_tracking_results(&config, &[track_from_epoch(epoch)], 10);
        let sat = report.output[0].sats.first().expect("observation satellite");

        assert_eq!(sat.metadata.observation_lock_state, "degraded");
        assert_eq!(sat.metadata.observation_lock_reason.as_deref(), Some("signal_fade"));
        assert_eq!(sat.metadata.tracking_lock_state, sat.metadata.observation_lock_state);
    }

    #[test]
    fn observations_record_lost_observation_lock_state() {
        let config = ReceiverPipelineConfig::default();
        let mut epoch = make_observation_ready_epoch(16, &config, 70);
        epoch.lock = false;
        epoch.pll_lock = false;
        epoch.dll_lock = false;
        epoch.fll_lock = false;
        epoch.lock_state = "lost".to_string();
        epoch.lock_state_reason = Some("prompt_power_drop".to_string());
        let report = observations_from_tracking_results(&config, &[track_from_epoch(epoch)], 10);
        let sat = report.output[0].sats.first().expect("observation satellite");

        assert_eq!(sat.metadata.observation_lock_state, "lost");
        assert_eq!(sat.metadata.observation_lock_reason.as_deref(), Some("prompt_power_drop"));
        assert_eq!(sat.metadata.tracking_lock_state, sat.metadata.observation_lock_state);
    }

    #[test]
    fn observations_record_reacquired_observation_lock_state() {
        let config = ReceiverPipelineConfig::default();
        let mut epoch = make_observation_ready_epoch(17, &config, 70);
        epoch.lock_state_reason = Some("reacquired".to_string());
        let report = observations_from_tracking_results(&config, &[track_from_epoch(epoch)], 10);
        let sat = report.output[0].sats.first().expect("observation satellite");

        assert_eq!(sat.metadata.observation_lock_state, "reacquired");
        assert_eq!(sat.metadata.observation_lock_reason.as_deref(), Some("reacquired"));
        assert_eq!(sat.metadata.tracking_lock_state, sat.metadata.observation_lock_state);
    }

    #[test]
    fn observations_record_cycle_slip_observation_lock_state() {
        let config = ReceiverPipelineConfig::default();
        let mut epoch = make_observation_ready_epoch(18, &config, 70);
        epoch.lock = false;
        epoch.pll_lock = false;
        epoch.dll_lock = false;
        epoch.fll_lock = false;
        epoch.cycle_slip = true;
        epoch.cycle_slip_reason = Some("phase_jump".to_string());
        epoch.lock_state = "lost".to_string();
        epoch.lock_state_reason = Some("phase_jump".to_string());
        let report = observations_from_tracking_results(&config, &[track_from_epoch(epoch)], 10);
        let sat = report.output[0].sats.first().expect("observation satellite");

        assert_eq!(sat.metadata.observation_lock_state, "cycle_slip");
        assert_eq!(sat.metadata.observation_lock_reason.as_deref(), Some("phase_jump"));
        assert_eq!(sat.metadata.tracking_lock_state, sat.metadata.observation_lock_state);
    }

    #[test]
    fn observations_declare_if_relative_doppler_contract() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 2_000.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            ..ReceiverPipelineConfig::default()
        };
        let sat = SatId { constellation: Constellation::Gps, prn: 10 };
        let expected_doppler_hz = -250.0;
        let carrier_hz = crate::pipeline::doppler::carrier_hz_from_doppler_hz(
            config.intermediate_freq_hz,
            expected_doppler_hz,
        );
        let track = TrackingResult {
            sat,
            carrier_hz,
            code_phase_samples: 0.0,
            acquisition_hypothesis: "accepted".to_string(),
            acquisition_score: 1.0,
            acquisition_code_phase_samples: 0,
            acquisition_carrier_hz: carrier_hz,
            acq_to_track_state: "accepted".to_string(),
            epochs: vec![make_tracking_epoch(10, &config, 70, carrier_hz)],
            transitions: Vec::new(),
        };

        let report = observations_from_tracking_results(&config, &[track], 10);
        let epoch = report.output.first().expect("observation epoch");
        let sat = epoch.sats.first().expect("observation satellite");

        assert_eq!(
            sat.metadata.doppler_model,
            bijux_gnss_core::api::OBSERVATION_DOPPLER_MODEL_TRACKED_CARRIER_IF_OFFSET
        );
        assert!((sat.doppler_hz.0 - expected_doppler_hz).abs() <= f64::EPSILON, "{sat:?}");
    }

    #[test]
    fn observations_emit_glonass_l1_signal_identity_and_fdma_relative_doppler() {
        let channel = GlonassFrequencyChannel::new(-4).expect("channel -4 must be valid");
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 2_044_000.0,
            intermediate_freq_hz: GPS_L1_CA_CARRIER_HZ.value()
                - glonass_l1_carrier_hz(channel).value(),
            code_freq_basis_hz: 511_000.0,
            code_length: 511,
            ..ReceiverPipelineConfig::default()
        };
        let sat = SatId { constellation: Constellation::Glonass, prn: 8 };
        let expected_doppler_hz = 125.0;
        let carrier_hz =
            crate::pipeline::doppler::carrier_hz_from_doppler_hz(0.0, expected_doppler_hz);
        let mut epoch = make_tracking_epoch(8, &config, 70, carrier_hz);
        epoch.sat = sat;
        epoch.signal_band = SignalBand::L1;
        epoch.glonass_frequency_channel = Some(channel);
        epoch.code_rate_hz = Hertz(511_000.0);
        let track = TrackingResult {
            sat,
            carrier_hz,
            code_phase_samples: 0.0,
            acquisition_hypothesis: "accepted".to_string(),
            acquisition_score: 1.0,
            acquisition_code_phase_samples: 0,
            acquisition_carrier_hz: carrier_hz,
            acq_to_track_state: "accepted".to_string(),
            epochs: vec![epoch],
            transitions: Vec::new(),
        };

        let report = observations_from_tracking_results(&config, &[track], 10);
        let epoch = report.output.first().expect("observation epoch");
        let obs_sat = epoch.sats.first().expect("observation satellite");

        assert_eq!(obs_sat.signal_id.sat, sat);
        assert_eq!(obs_sat.signal_id.band, SignalBand::L1);
        assert_eq!(obs_sat.signal_id.code, SignalCode::Unknown);
        assert_eq!(obs_sat.metadata.signal.code, SignalCode::Unknown);
        assert_eq!(
            obs_sat.metadata.signal.carrier_hz.value(),
            glonass_l1_carrier_hz(channel).value()
        );
        assert!((obs_sat.doppler_hz.0 - expected_doppler_hz).abs() <= f64::EPSILON, "{obs_sat:?}");
    }

    #[test]
    fn observations_emit_gps_l2c_signal_identity_and_civil_code_pseudorange() {
        let signal = signal_spec_gps_l2c();
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 5_115_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: signal.code_rate_hz,
            code_length: 10230,
            ..ReceiverPipelineConfig::default()
        };
        let whole_code_periods = 4;
        let aligned_code_phase_chips = 768.0;
        let expected_pseudorange_m = aligned_pseudorange_m_for_signal(
            signal,
            10230,
            whole_code_periods,
            aligned_code_phase_chips,
        );
        let epoch = TrackEpoch {
            sat: SatId { constellation: Constellation::Gps, prn: 11 },
            signal_band: SignalBand::L2,
            carrier_hz: Hertz(tracked_signal_center_hz(config.intermediate_freq_hz, signal)),
            code_rate_hz: Hertz(signal.code_rate_hz),
            code_phase_samples: Chips(test_tracking_code_phase_samples_for_signal(
                &config,
                signal,
                10230,
                aligned_code_phase_chips,
            )),
            signal_delay_alignment: Some(SignalDelayAlignment {
                whole_code_periods,
                sample_delay_samples: 0,
                source: "synthetic_truth".to_string(),
            }),
            ..make_tracking_epoch_with_phase(
                11,
                &config,
                70,
                tracked_signal_center_hz(config.intermediate_freq_hz, signal),
                0.0,
            )
        };
        let report = observations_from_tracking_results(&config, &[track_from_epoch(epoch)], 10);
        let obs_sat = report.output[0].sats.first().expect("observation satellite");

        assert_eq!(obs_sat.signal_id.band, SignalBand::L2);
        assert_eq!(obs_sat.signal_id.code, SignalCode::L2C);
        assert_eq!(obs_sat.metadata.signal, signal);
        assert!((obs_sat.metadata.signal.code_rate_hz - 511_500.0).abs() <= f64::EPSILON);
        assert_eq!(obs_sat.metadata.pseudorange_model, "tracked_code_phase_alignment");
        assert!((obs_sat.pseudorange_m.0 - expected_pseudorange_m).abs() <= 1.0e-6, "{obs_sat:?}");
    }

    #[test]
    fn observations_subtract_known_sample_delay_from_aligned_pseudorange() {
        let signal = signal_spec_gps_l1_ca();
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: signal.code_rate_hz,
            code_length: 1023,
            ..ReceiverPipelineConfig::default()
        };
        let whole_code_periods = 12;
        let aligned_code_phase_chips = 512.0;
        let sample_delay_samples = 20;
        let samples_per_chip = config.sampling_freq_hz / signal.code_rate_hz;
        let delayed_code_phase_chips =
            aligned_code_phase_chips + sample_delay_samples as f64 / samples_per_chip;
        let expected_pseudorange_m = aligned_pseudorange_m_for_signal(
            signal,
            1023,
            whole_code_periods,
            aligned_code_phase_chips,
        );
        let epoch = TrackEpoch {
            sat: SatId { constellation: Constellation::Gps, prn: 7 },
            signal_band: SignalBand::L1,
            carrier_hz: Hertz(tracked_signal_center_hz(config.intermediate_freq_hz, signal)),
            code_rate_hz: Hertz(signal.code_rate_hz),
            code_phase_samples: Chips(test_tracking_code_phase_samples_for_signal(
                &config,
                signal,
                1023,
                delayed_code_phase_chips,
            )),
            signal_delay_alignment: Some(SignalDelayAlignment {
                whole_code_periods,
                sample_delay_samples,
                source: "synthetic_truth".to_string(),
            }),
            ..make_tracking_epoch_with_phase(
                7,
                &config,
                70,
                tracked_signal_center_hz(config.intermediate_freq_hz, signal),
                0.0,
            )
        };

        let report = observations_from_tracking_results(&config, &[track_from_epoch(epoch)], 10);
        let obs_sat = report.output[0].sats.first().expect("observation satellite");

        assert_eq!(obs_sat.metadata.pseudorange_model, "tracked_code_phase_alignment");
        assert!((obs_sat.pseudorange_m.0 - expected_pseudorange_m).abs() <= 1.0e-6, "{obs_sat:?}");
    }

    #[test]
    fn observations_emit_gps_l5_signal_identity_and_aligned_pseudorange() {
        let signal = signal_spec_gps_l5();
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 10_230_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: signal.code_rate_hz,
            code_length: 10230,
            ..ReceiverPipelineConfig::default()
        };
        let whole_code_periods = 8;
        let aligned_code_phase_chips = 2_048.0;
        let expected_pseudorange_m = aligned_pseudorange_m_for_signal(
            signal,
            10230,
            whole_code_periods,
            aligned_code_phase_chips,
        );
        let epoch = TrackEpoch {
            sat: SatId { constellation: Constellation::Gps, prn: 12 },
            signal_band: SignalBand::L5,
            carrier_hz: Hertz(tracked_signal_center_hz(config.intermediate_freq_hz, signal)),
            code_rate_hz: Hertz(signal.code_rate_hz),
            code_phase_samples: Chips(test_tracking_code_phase_samples_for_signal(
                &config,
                signal,
                10230,
                aligned_code_phase_chips,
            )),
            signal_delay_alignment: Some(SignalDelayAlignment {
                whole_code_periods,
                sample_delay_samples: 0,
                source: "synthetic_truth".to_string(),
            }),
            ..make_tracking_epoch_with_phase(
                12,
                &config,
                70,
                tracked_signal_center_hz(config.intermediate_freq_hz, signal),
                0.0,
            )
        };
        let report = observations_from_tracking_results(&config, &[track_from_epoch(epoch)], 10);
        let obs_sat = report.output[0].sats.first().expect("observation satellite");

        assert_eq!(obs_sat.signal_id.band, SignalBand::L5);
        assert_eq!(obs_sat.signal_id.code, SignalCode::L5I);
        assert_eq!(obs_sat.metadata.signal, signal);
        assert!((obs_sat.metadata.signal.code_rate_hz - 10_230_000.0).abs() <= f64::EPSILON);
        assert_eq!(obs_sat.metadata.pseudorange_model, "tracked_code_phase_alignment");
        assert!((obs_sat.pseudorange_m.0 - expected_pseudorange_m).abs() <= 1.0e-6, "{obs_sat:?}");
    }

    #[test]
    fn observations_preserve_gps_l5_q_signal_identity_and_aligned_pseudorange() {
        let signal = signal_spec_gps_l5_q();
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 10_230_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: signal.code_rate_hz,
            code_length: 10230,
            ..ReceiverPipelineConfig::default()
        };
        let whole_code_periods = 8;
        let aligned_code_phase_chips = 2_048.0;
        let expected_pseudorange_m = aligned_pseudorange_m_for_signal(
            signal,
            10230,
            whole_code_periods,
            aligned_code_phase_chips,
        );
        let epoch = TrackEpoch {
            sat: SatId { constellation: Constellation::Gps, prn: 12 },
            signal_band: SignalBand::L5,
            signal_code: SignalCode::L5Q,
            carrier_hz: Hertz(tracked_signal_center_hz(config.intermediate_freq_hz, signal)),
            code_rate_hz: Hertz(signal.code_rate_hz),
            code_phase_samples: Chips(test_tracking_code_phase_samples_for_signal(
                &config,
                signal,
                10230,
                aligned_code_phase_chips,
            )),
            signal_delay_alignment: Some(SignalDelayAlignment {
                whole_code_periods,
                sample_delay_samples: 0,
                source: "synthetic_truth".to_string(),
            }),
            ..make_tracking_epoch_with_phase(
                12,
                &config,
                70,
                tracked_signal_center_hz(config.intermediate_freq_hz, signal),
                0.0,
            )
        };
        let report = observations_from_tracking_results(&config, &[track_from_epoch(epoch)], 10);
        let obs_sat = report.output[0].sats.first().expect("observation satellite");

        assert_eq!(obs_sat.signal_id.band, SignalBand::L5);
        assert_eq!(obs_sat.signal_id.code, SignalCode::L5Q);
        assert_eq!(obs_sat.metadata.signal, signal);
        assert!((obs_sat.metadata.signal.code_rate_hz - 10_230_000.0).abs() <= f64::EPSILON);
        assert_eq!(obs_sat.metadata.pseudorange_model, "tracked_code_phase_alignment");
        assert!((obs_sat.pseudorange_m.0 - expected_pseudorange_m).abs() <= 1.0e-6, "{obs_sat:?}");
    }

    #[test]
    fn observations_emit_galileo_e5_signal_identity_and_aligned_pseudorange() {
        let signal = signal_spec_galileo_e5a();
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 10_230_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: signal.code_rate_hz,
            code_length: 10230,
            ..ReceiverPipelineConfig::default()
        };
        let whole_code_periods = 6;
        let aligned_code_phase_chips = 1_536.0;
        let expected_pseudorange_m = aligned_pseudorange_m_for_signal(
            signal,
            10230,
            whole_code_periods,
            aligned_code_phase_chips,
        );
        let expected_phase_cycles =
            signal_meters_to_cycles(Meters(expected_pseudorange_m), signal).0;
        let epoch = TrackEpoch {
            sat: SatId { constellation: Constellation::Galileo, prn: 11 },
            signal_band: SignalBand::E5,
            carrier_hz: Hertz(tracked_signal_center_hz(config.intermediate_freq_hz, signal)),
            code_rate_hz: Hertz(signal.code_rate_hz),
            code_phase_samples: Chips(test_tracking_code_phase_samples_for_signal(
                &config,
                signal,
                10230,
                aligned_code_phase_chips,
            )),
            signal_delay_alignment: Some(SignalDelayAlignment {
                whole_code_periods,
                sample_delay_samples: 0,
                source: "synthetic_truth".to_string(),
            }),
            ..make_tracking_epoch_with_phase(
                11,
                &config,
                70,
                tracked_signal_center_hz(config.intermediate_freq_hz, signal),
                expected_phase_cycles,
            )
        };
        let report = observations_from_tracking_results(&config, &[track_from_epoch(epoch)], 10);
        let obs_sat = report.output[0].sats.first().expect("observation satellite");

        assert_eq!(obs_sat.signal_id.band, SignalBand::E5);
        assert_eq!(obs_sat.signal_id.code, SignalCode::E5a);
        assert_eq!(obs_sat.metadata.signal, signal);
        assert!((obs_sat.metadata.signal.code_rate_hz - 10_230_000.0).abs() <= f64::EPSILON);
        assert_eq!(obs_sat.metadata.pseudorange_model, "tracked_code_phase_alignment");
        assert!((obs_sat.pseudorange_m.0 - expected_pseudorange_m).abs() <= 1.0e-6, "{obs_sat:?}");
        assert!(
            (signal_cycles_to_meters(obs_sat.carrier_phase_cycles, obs_sat.metadata.signal).0
                - expected_pseudorange_m)
                .abs()
                <= 1.0e-6,
            "{obs_sat:?}"
        );
    }

    #[test]
    fn observations_emit_beidou_b2_signal_identity_and_aligned_pseudorange() {
        let signal = signal_spec_beidou_b2i();
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 2_046_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: signal.code_rate_hz,
            code_length: 2046,
            ..ReceiverPipelineConfig::default()
        };
        let whole_code_periods = 8;
        let aligned_code_phase_chips = 768.0;
        let expected_pseudorange_m = aligned_pseudorange_m_for_signal(
            signal,
            2046,
            whole_code_periods,
            aligned_code_phase_chips,
        );
        let expected_phase_cycles =
            signal_meters_to_cycles(Meters(expected_pseudorange_m), signal).0;
        let epoch = TrackEpoch {
            sat: SatId { constellation: Constellation::Beidou, prn: 11 },
            signal_band: SignalBand::B2,
            carrier_hz: Hertz(tracked_signal_center_hz(config.intermediate_freq_hz, signal)),
            code_rate_hz: Hertz(signal.code_rate_hz),
            code_phase_samples: Chips(test_tracking_code_phase_samples_for_signal(
                &config,
                signal,
                2046,
                aligned_code_phase_chips,
            )),
            signal_delay_alignment: Some(SignalDelayAlignment {
                whole_code_periods,
                sample_delay_samples: 0,
                source: "synthetic_truth".to_string(),
            }),
            ..make_tracking_epoch_with_phase(
                11,
                &config,
                70,
                tracked_signal_center_hz(config.intermediate_freq_hz, signal),
                expected_phase_cycles,
            )
        };
        let report = observations_from_tracking_results(&config, &[track_from_epoch(epoch)], 10);
        let obs_sat = report.output[0].sats.first().expect("observation satellite");

        assert_eq!(obs_sat.signal_id.band, SignalBand::B2);
        assert_eq!(obs_sat.signal_id.code, SignalCode::B2I);
        assert_eq!(obs_sat.metadata.signal, signal);
        assert!((obs_sat.metadata.signal.code_rate_hz - 2_046_000.0).abs() <= f64::EPSILON);
        assert_eq!(obs_sat.metadata.pseudorange_model, "tracked_code_phase_alignment");
        assert!((obs_sat.pseudorange_m.0 - expected_pseudorange_m).abs() <= 1.0e-6, "{obs_sat:?}");
        assert!(
            (signal_cycles_to_meters(obs_sat.carrier_phase_cycles, obs_sat.metadata.signal).0
                - expected_pseudorange_m)
                .abs()
                <= 1.0e-6,
            "{obs_sat:?}"
        );
    }

    #[test]
    fn observation_metadata_sets_support_and_uncertainty_classes() {
        let config = ReceiverPipelineConfig::default();
        let sat = SatId { constellation: Constellation::Gps, prn: 7 };
        let mut weak = make_observation_ready_epoch(7, &config, 70);
        weak.cn0_dbhz = 20.0;
        let mut missing = make_observation_ready_epoch(8, &config, 70);
        missing.lock = false;
        missing.cn0_dbhz = 45.0;
        missing.lock_state = "lost".to_string();
        let tracks = vec![
            TrackingResult {
                sat,
                carrier_hz: 0.0,
                code_phase_samples: 0.0,
                acquisition_hypothesis: "deferred".to_string(),
                acquisition_score: 0.0,
                acquisition_code_phase_samples: 0,
                acquisition_carrier_hz: 0.0,
                acq_to_track_state: "deferred".to_string(),
                epochs: vec![weak],
                transitions: Vec::new(),
            },
            TrackingResult {
                sat: SatId { constellation: Constellation::Gps, prn: 8 },
                carrier_hz: 0.0,
                code_phase_samples: 0.0,
                acquisition_hypothesis: "deferred".to_string(),
                acquisition_score: 0.0,
                acquisition_code_phase_samples: 0,
                acquisition_carrier_hz: 0.0,
                acq_to_track_state: "deferred".to_string(),
                epochs: vec![missing],
                transitions: Vec::new(),
            },
        ];
        let report = observations_from_tracking_results(&config, &tracks, 10);
        let epoch = report.output.iter().find(|row| row.epoch_idx == 70).expect("epoch");
        let labels = epoch
            .sats
            .iter()
            .map(|sat| {
                (
                    sat.metadata.observation_support_class.clone(),
                    sat.metadata.observation_uncertainty_class.clone(),
                )
            })
            .collect::<Vec<_>>();
        assert!(labels
            .iter()
            .any(|(support, uncertainty)| support == "degraded" && uncertainty == "high"));
        assert!(labels
            .iter()
            .any(|(support, uncertainty)| support == "degraded" && uncertainty == "low"));
    }

    #[test]
    fn apply_epoch_decision_refuses_malformed_duplicate_signal_set() {
        let mut epoch = fake_obs_epoch_for_nav_tests(0);
        let duplicate = epoch.sats[0].clone();
        epoch.sats.push(duplicate);
        apply_epoch_decision(&mut epoch);
        assert_eq!(epoch.decision, ObservationEpochDecision::Rejected);
        let reason = epoch.decision_reason.expect("decision reason");
        assert!(reason.contains("malformed_observation_set"));
        assert!(reason.contains("duplicate signal_id"));
    }

    #[test]
    fn observations_reject_sample_rate_mismatch_tracking_reason() {
        let config = ReceiverPipelineConfig::default();
        let sat = SatId { constellation: Constellation::Gps, prn: 11 };
        let mismatch_epoch = TrackEpoch {
            epoch: Epoch { index: 0 },
            sample_index: 0,
            sat,
            lock: true,
            cn0_dbhz: 45.0,
            dll_lock: false,
            pll_lock: false,
            lock_state: "tracking".to_string(),
            lock_state_reason: Some("sample_rate_mismatch".to_string()),
            ..TrackEpoch::default()
        };
        let track = TrackingResult {
            sat,
            carrier_hz: 0.0,
            code_phase_samples: 0.0,
            acquisition_hypothesis: "accepted".to_string(),
            acquisition_score: 1.0,
            acquisition_code_phase_samples: 0,
            acquisition_carrier_hz: 0.0,
            acq_to_track_state: "accepted".to_string(),
            epochs: vec![mismatch_epoch],
            transitions: Vec::new(),
        };

        let report = observations_from_tracking_results(&config, &[track], 10);
        let epoch = report.output.first().expect("observation epoch");
        let sat = epoch.sats.first().expect("observation satellite");

        assert_eq!(sat.observation_status, ObservationStatus::Inconsistent);
        assert!(sat
            .observation_reject_reasons
            .iter()
            .any(|reason| reason == "sample_rate_mismatch"));
        assert_eq!(epoch.decision, ObservationEpochDecision::Rejected);
        assert_eq!(epoch.decision_reason.as_deref(), Some("inconsistent_observable"));
    }

    #[test]
    fn observations_preserve_tracking_cn0_on_unlock_rows() {
        let config = ReceiverPipelineConfig::default();
        let sat = SatId { constellation: Constellation::Gps, prn: 11 };
        let expected_cn0_dbhz = 31.25;
        let unlocked_epoch = TrackEpoch {
            epoch: Epoch { index: 70 },
            sample_index: epoch_sample_index(&config, 70),
            source_time: ReceiverSampleTrace::from_sample_index(
                epoch_sample_index(&config, 70),
                config.sampling_freq_hz,
            ),
            sat,
            lock: false,
            pll_lock: false,
            dll_lock: false,
            fll_lock: false,
            cn0_dbhz: expected_cn0_dbhz,
            lock_state: "lost".to_string(),
            lock_state_reason: Some("prompt_power_drop".to_string()),
            ..TrackEpoch::default()
        };
        let report =
            observations_from_tracking_results(&config, &[track_from_epoch(unlocked_epoch)], 10);
        let epoch = report.output.first().expect("observation epoch");
        let sat = epoch.sats.first().expect("observation satellite");

        assert_eq!(sat.observation_status, ObservationStatus::Missing);
        assert!((sat.cn0_dbhz - expected_cn0_dbhz).abs() <= f64::EPSILON, "{sat:?}");
    }

    #[test]
    fn observations_preserve_tracking_cn0_on_inconsistent_rows() {
        let config = ReceiverPipelineConfig::default();
        let sat = SatId { constellation: Constellation::Gps, prn: 12 };
        let expected_cn0_dbhz = 36.5;
        let mismatch_epoch = TrackEpoch {
            epoch: Epoch { index: 70 },
            sample_index: epoch_sample_index(&config, 70),
            source_time: ReceiverSampleTrace::from_sample_index(
                epoch_sample_index(&config, 70),
                config.sampling_freq_hz,
            ),
            sat,
            lock: true,
            pll_lock: false,
            dll_lock: false,
            cn0_dbhz: expected_cn0_dbhz,
            lock_state: "tracking".to_string(),
            lock_state_reason: Some("sample_rate_mismatch".to_string()),
            ..TrackEpoch::default()
        };
        let report =
            observations_from_tracking_results(&config, &[track_from_epoch(mismatch_epoch)], 10);
        let epoch = report.output.first().expect("observation epoch");
        let sat = epoch.sats.first().expect("observation satellite");

        assert_eq!(sat.observation_status, ObservationStatus::Inconsistent);
        assert!((sat.cn0_dbhz - expected_cn0_dbhz).abs() <= f64::EPSILON, "{sat:?}");
    }

    #[test]
    fn observations_keep_doppler_on_tracking_unlock_epoch() {
        let config = ReceiverPipelineConfig::default();
        let sat = SatId { constellation: Constellation::Gps, prn: 12 };
        let expected_doppler_hz = 125.0;
        let carrier_hz = crate::pipeline::doppler::carrier_hz_from_doppler_hz(
            config.intermediate_freq_hz,
            expected_doppler_hz,
        );
        let unlocked_epoch = TrackEpoch {
            epoch: Epoch { index: 70 },
            sample_index: epoch_sample_index(&config, 70),
            source_time: ReceiverSampleTrace::from_sample_index(
                epoch_sample_index(&config, 70),
                config.sampling_freq_hz,
            ),
            sat,
            carrier_hz: Hertz(carrier_hz),
            code_rate_hz: Hertz(config.code_freq_basis_hz),
            lock: false,
            pll_lock: false,
            dll_lock: false,
            fll_lock: false,
            cn0_dbhz: 45.0,
            lock_state: "lost".to_string(),
            lock_state_reason: Some("prompt_power_drop".to_string()),
            ..TrackEpoch::default()
        };
        let report =
            observations_from_tracking_results(&config, &[track_from_epoch(unlocked_epoch)], 10);
        let epoch = report.output.first().expect("observation epoch");
        let sat = epoch.sats.first().expect("observation satellite");

        assert_eq!(sat.observation_status, ObservationStatus::Missing);
        assert_eq!(
            sat.metadata.doppler_model,
            bijux_gnss_core::api::OBSERVATION_DOPPLER_MODEL_TRACKED_CARRIER_IF_OFFSET
        );
        assert!((sat.doppler_hz.0 - expected_doppler_hz).abs() <= f64::EPSILON, "{sat:?}");
        assert!(sat.doppler_var_hz2.is_finite());
    }

    #[test]
    fn observations_keep_doppler_on_sample_rate_mismatch_epoch() {
        let config = ReceiverPipelineConfig::default();
        let sat = SatId { constellation: Constellation::Gps, prn: 13 };
        let expected_doppler_hz = -80.0;
        let carrier_hz = crate::pipeline::doppler::carrier_hz_from_doppler_hz(
            config.intermediate_freq_hz,
            expected_doppler_hz,
        );
        let mismatch_epoch = TrackEpoch {
            epoch: Epoch { index: 70 },
            sample_index: epoch_sample_index(&config, 70),
            source_time: ReceiverSampleTrace::from_sample_index(
                epoch_sample_index(&config, 70),
                config.sampling_freq_hz,
            ),
            sat,
            carrier_hz: Hertz(carrier_hz),
            code_rate_hz: Hertz(config.code_freq_basis_hz),
            lock: true,
            pll_lock: false,
            dll_lock: false,
            cn0_dbhz: 45.0,
            lock_state: "tracking".to_string(),
            lock_state_reason: Some("sample_rate_mismatch".to_string()),
            ..TrackEpoch::default()
        };
        let report =
            observations_from_tracking_results(&config, &[track_from_epoch(mismatch_epoch)], 10);
        let epoch = report.output.first().expect("observation epoch");
        let sat = epoch.sats.first().expect("observation satellite");

        assert_eq!(sat.observation_status, ObservationStatus::Inconsistent);
        assert_eq!(
            sat.metadata.doppler_model,
            bijux_gnss_core::api::OBSERVATION_DOPPLER_MODEL_TRACKED_CARRIER_IF_OFFSET
        );
        assert!((sat.doppler_hz.0 - expected_doppler_hz).abs() <= f64::EPSILON, "{sat:?}");
        assert!(sat.doppler_var_hz2.is_finite());
    }

    #[test]
    fn observations_reject_non_positive_pseudorange() {
        let config = ReceiverPipelineConfig::default();
        let carrier_hz = crate::pipeline::doppler::carrier_hz_from_doppler_hz(0.0, 0.0);
        let epoch = make_tracking_epoch_with_alignment(12, &config, 70, carrier_hz, 0.0, 0, -8.0);
        let report = observations_from_tracking_results(&config, &[track_from_epoch(epoch)], 10);

        let epoch = report.output.first().expect("observation epoch");
        let sat = epoch.sats.first().expect("observation satellite");

        assert_eq!(sat.observation_status, ObservationStatus::Inconsistent);
        assert!(sat
            .observation_reject_reasons
            .iter()
            .any(|reason| reason == "non_positive_pseudorange"));
        assert_eq!(epoch.decision, ObservationEpochDecision::Rejected);
        assert_eq!(epoch.decision_reason.as_deref(), Some("inconsistent_observable"));
        assert!(!epoch.valid);
    }

    #[test]
    fn observations_reject_out_of_bounds_pseudorange() {
        let config = ReceiverPipelineConfig::default();
        let carrier_hz = crate::pipeline::doppler::carrier_hz_from_doppler_hz(0.0, 0.0);
        let epoch =
            make_tracking_epoch_with_alignment(13, &config, 70, carrier_hz, 0.0, 200_000, 0.0);
        let report = observations_from_tracking_results(&config, &[track_from_epoch(epoch)], 10);

        let epoch = report.output.first().expect("observation epoch");
        let sat = epoch.sats.first().expect("observation satellite");

        assert_eq!(sat.observation_status, ObservationStatus::Inconsistent);
        assert!(sat
            .observation_reject_reasons
            .iter()
            .any(|reason| reason == "pseudorange_out_of_bounds"));
        assert_eq!(epoch.decision, ObservationEpochDecision::Rejected);
        assert_eq!(epoch.decision_reason.as_deref(), Some("inconsistent_observable"));
        assert!(!epoch.valid);
    }

    #[test]
    fn observations_reject_non_finite_pseudorange() {
        let config = ReceiverPipelineConfig::default();
        let carrier_hz = crate::pipeline::doppler::carrier_hz_from_doppler_hz(0.0, 0.0);
        let epoch =
            make_tracking_epoch_with_alignment(14, &config, 70, carrier_hz, 0.0, 68, f64::NAN);
        let (epochs, _) = observations_from_tracking(&config, &[epoch]);
        let epoch = epochs.first().expect("observation epoch");
        let sat = epoch.sats.first().expect("observation satellite");

        assert_eq!(sat.observation_status, ObservationStatus::Inconsistent);
        assert!(sat
            .observation_reject_reasons
            .iter()
            .any(|reason| reason == "non_finite_pseudorange"));
        assert_eq!(epoch.decision, ObservationEpochDecision::Rejected);
        assert_eq!(epoch.decision_reason.as_deref(), Some("inconsistent_observable"));
        assert!(!epoch.valid);
    }

    #[test]
    fn observation_residuals_report_raw_corrected_and_expected_pseudorange() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 1_023_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            ..ReceiverPipelineConfig::default()
        };
        let carrier_hz = crate::pipeline::doppler::carrier_hz_from_doppler_hz(0.0, 125.0);
        let track = TrackingResult {
            sat: SatId { constellation: Constellation::Gps, prn: 21 },
            carrier_hz,
            code_phase_samples: 0.0,
            acquisition_hypothesis: "accepted".to_string(),
            acquisition_score: 1.0,
            acquisition_code_phase_samples: 0,
            acquisition_carrier_hz: carrier_hz,
            acq_to_track_state: "accepted".to_string(),
            epochs: vec![
                make_tracking_epoch_with_alignment(21, &config, 70, carrier_hz, 10.0, 68, 0.0),
                make_tracking_epoch_with_alignment(21, &config, 71, carrier_hz, 10.125, 68, 0.0),
            ],
            transitions: Vec::new(),
        };

        let report = observation_residuals_from_tracking_results_with_gps_anchor(
            &config,
            Some(GpsTime { week: 2200, tow_s: 345_600.0 }),
            &[track],
            10,
        );
        let epoch = residual_epoch(&report, 71);
        let sat = epoch.sats.first().expect("residual satellite");
        let lambda_m = SPEED_OF_LIGHT_MPS / GPS_L1_CA_CARRIER_HZ.value();
        let expected_raw = aligned_pseudorange_m(&config, 68, 0.0);
        let expected_corrected = expected_raw + 0.5 * 0.125 * lambda_m;

        assert!((sat.pseudorange_m.raw - expected_raw).abs() <= 1.0e-9, "{sat:?}");
        assert!((sat.pseudorange_m.corrected - expected_corrected).abs() <= 1.0e-9, "{sat:?}");
        assert_eq!(sat.pseudorange_m.expected, Some(expected_raw));
        assert_eq!(
            sat.pseudorange_m.reference_model.as_deref(),
            Some("signal_travel_time_from_gps_anchor")
        );
        assert!(
            (sat.pseudorange_m.residual.expect("pseudorange residual")
                - (expected_corrected - expected_raw))
                .abs()
                <= 1.0e-9,
            "{sat:?}"
        );
        assert!(sat.pseudorange_m.sigma.expect("pseudorange sigma") > 0.0);
    }

    #[test]
    fn observation_residuals_predict_carrier_phase_for_continuous_arcs() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 1_023_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            ..ReceiverPipelineConfig::default()
        };
        let carrier_hz = crate::pipeline::doppler::carrier_hz_from_doppler_hz(0.0, 125.0);
        let track = TrackingResult {
            sat: SatId { constellation: Constellation::Gps, prn: 22 },
            carrier_hz,
            code_phase_samples: 0.0,
            acquisition_hypothesis: "accepted".to_string(),
            acquisition_score: 1.0,
            acquisition_code_phase_samples: 0,
            acquisition_carrier_hz: carrier_hz,
            acq_to_track_state: "accepted".to_string(),
            epochs: vec![
                make_tracking_epoch_with_alignment(22, &config, 70, carrier_hz, 10.0, 68, 0.0),
                make_tracking_epoch_with_alignment(22, &config, 71, carrier_hz, 10.125, 68, 0.0),
                make_tracking_epoch_with_alignment(22, &config, 72, carrier_hz, 10.250, 68, 0.0),
            ],
            transitions: Vec::new(),
        };

        let report = observation_residuals_from_tracking_results(&config, &[track], 10);
        let epoch = residual_epoch(&report, 72);
        let sat = epoch.sats.first().expect("residual satellite");

        assert_eq!(
            sat.carrier_phase_cycles.reference_model.as_deref(),
            Some("previous_continuous_phase_prediction")
        );
        assert_eq!(sat.carrier_phase_cycles.expected, Some(10.250));
        assert!(sat.carrier_phase_cycles.residual.expect("carrier residual").abs() <= 1.0e-12);
        assert_eq!(sat.carrier_phase_cycles.raw, 10.250);
        assert_eq!(sat.carrier_phase_cycles.corrected, 10.250);
    }

    #[test]
    fn observation_residuals_surface_epoch_and_satellite_rejections() {
        let config = ReceiverPipelineConfig::default();
        let carrier_hz = crate::pipeline::doppler::carrier_hz_from_doppler_hz(0.0, 0.0);
        let epoch = make_tracking_epoch_with_alignment(23, &config, 70, carrier_hz, 0.0, 0, -8.0);
        let report =
            observation_residuals_from_tracking_results(&config, &[track_from_epoch(epoch)], 10);
        let epoch = residual_epoch(&report, 70);
        let sat = epoch.sats.first().expect("residual satellite");

        assert!(!epoch.accepted);
        assert_eq!(epoch.decision, ObservationEpochDecision::Rejected);
        assert_eq!(epoch.decision_reason.as_deref(), Some("inconsistent_observable"));
        assert!(!sat.accepted);
        assert_eq!(sat.observation_status, ObservationStatus::Inconsistent);
        assert!(sat
            .observation_reject_reasons
            .iter()
            .any(|reason| reason == "non_positive_pseudorange"));
    }

    #[test]
    fn observation_decisions_surface_impossible_pseudorange_reason() {
        let config = ReceiverPipelineConfig::default();
        let carrier_hz = crate::pipeline::doppler::carrier_hz_from_doppler_hz(0.0, 0.0);
        let epoch = make_tracking_epoch_with_alignment(15, &config, 70, carrier_hz, 0.0, 0, -8.0);
        let report = observations_from_tracking_results(&config, &[track_from_epoch(epoch)], 10);
        let decisions = observation_decisions_from_epochs(&report.output);
        let decision = decisions.first().expect("observation decision");

        assert_eq!(decision.decision, ObservationEpochDecision::Rejected);
        assert!(decision.reasons.iter().any(|reason| reason == "non_positive_pseudorange"));
    }

    #[test]
    fn code_carrier_divergence_decomposition_exempts_expected_ionosphere_from_multipath() {
        let mut state = CodeCarrierDivergenceState::default();
        let l1_delay_before_m = 3.0;
        let l1_delay_after_m = 9.0;
        let l1_jump_m = 2.0 * (l1_delay_after_m - l1_delay_before_m);
        let l2_delay_before_m = scaled_ionosphere_delay_m(
            l1_delay_before_m,
            signal_spec_gps_l1_ca(),
            signal_spec_gps_l2c(),
        );
        let l2_delay_after_m = scaled_ionosphere_delay_m(
            l1_delay_after_m,
            signal_spec_gps_l1_ca(),
            signal_spec_gps_l2c(),
        );
        let l2_jump_m = 2.0 * (l2_delay_after_m - l2_delay_before_m);

        let mut before = divergence_epoch(l1_delay_before_m, 106.0, 0.0, l2_delay_before_m, 0.0);
        assert_ne!(before.sats[0].signal_id.band, before.sats[1].signal_id.band);
        assert_eq!(before.sats[0].signal_id.sat, before.sats[1].signal_id.sat);
        assert!(before.sats[0].lock_flags.code_lock);
        assert!(before.sats[1].lock_flags.code_lock);
        assert!(before.sats[0].metadata.signal.carrier_hz.value() > 0.0);
        assert!(before.sats[1].metadata.signal.carrier_hz.value() > 0.0);
        assert_ne!(
            before.sats[0].metadata.signal.carrier_hz.value(),
            before.sats[1].metadata.signal.carrier_hz.value()
        );
        assert!(before.sats[0].pseudorange_m.0.is_finite());
        assert!(before.sats[1].pseudorange_m.0.is_finite());
        assert!(dual_frequency_code_delay_evidence(&before.sats[0], &before.sats[1]).is_some());
        assert_eq!(ionosphere_delay_evidence_from_epoch(&before).len(), 2);
        apply_code_carrier_divergence_decomposition(&mut before, &mut state);
        let mut after = divergence_epoch(
            l1_delay_after_m,
            106.0 + l1_jump_m,
            l1_jump_m,
            l2_delay_after_m,
            l2_jump_m,
        );
        apply_code_carrier_divergence_decomposition(&mut after, &mut state);

        let sat = after
            .sats
            .iter()
            .find(|sat| sat.signal_id.band == SignalBand::L1)
            .expect("L1 observation");
        let divergence =
            sat.metadata.code_carrier_divergence.expect("code-carrier divergence decomposition");

        assert!(!sat.multipath_suspect, "{sat:?}");
        assert!((divergence.expected_ionosphere_m - l1_jump_m).abs() < 1.0e-6);
        assert!(divergence.multipath_m.abs() < 1.0e-9);
        assert!(divergence.unexplained_m.abs() < 1.0e-6);
        assert_eq!(sat.error_model.as_ref().expect("error model").multipath_proxy_m, Meters(0.0));
    }

    #[test]
    fn code_carrier_divergence_decomposition_preserves_ionosphere_sign() {
        let mut state = CodeCarrierDivergenceState::default();
        let l1_delay_before_m = 9.0;
        let l1_delay_after_m = 3.0;
        let l1_jump_m = 2.0 * (l1_delay_after_m - l1_delay_before_m);
        let l2_delay_before_m = scaled_ionosphere_delay_m(
            l1_delay_before_m,
            signal_spec_gps_l1_ca(),
            signal_spec_gps_l2c(),
        );
        let l2_delay_after_m = scaled_ionosphere_delay_m(
            l1_delay_after_m,
            signal_spec_gps_l1_ca(),
            signal_spec_gps_l2c(),
        );
        let l2_jump_m = 2.0 * (l2_delay_after_m - l2_delay_before_m);

        let mut before = divergence_epoch(l1_delay_before_m, 118.0, 0.0, l2_delay_before_m, 0.0);
        apply_code_carrier_divergence_decomposition(&mut before, &mut state);
        let mut after = divergence_epoch(
            l1_delay_after_m,
            118.0 + l1_jump_m,
            l1_jump_m,
            l2_delay_after_m,
            l2_jump_m,
        );
        apply_code_carrier_divergence_decomposition(&mut after, &mut state);

        let sat = after
            .sats
            .iter()
            .find(|sat| sat.signal_id.band == SignalBand::L1)
            .expect("L1 observation");
        let divergence =
            sat.metadata.code_carrier_divergence.expect("code-carrier divergence decomposition");

        assert!(!sat.multipath_suspect, "{sat:?}");
        assert!((divergence.jump_m - l1_jump_m).abs() < 1.0e-6);
        assert!((divergence.expected_ionosphere_m - l1_jump_m).abs() < 1.0e-6);
        assert!(divergence.multipath_m.abs() < 1.0e-9);
        assert!(divergence.unexplained_m.abs() < 1.0e-6);
    }

    #[test]
    fn code_carrier_divergence_decomposition_keeps_residual_multipath_separate() {
        let mut state = CodeCarrierDivergenceState::default();
        let l1_delay_before_m = 3.0;
        let l1_delay_after_m = 9.0;
        let residual_multipath_m = 8.0;
        let l1_ionosphere_jump_m = 2.0 * (l1_delay_after_m - l1_delay_before_m);
        let l1_jump_m = l1_ionosphere_jump_m + residual_multipath_m;
        let l2_delay_before_m = scaled_ionosphere_delay_m(
            l1_delay_before_m,
            signal_spec_gps_l1_ca(),
            signal_spec_gps_l2c(),
        );
        let l2_delay_after_m = scaled_ionosphere_delay_m(
            l1_delay_after_m,
            signal_spec_gps_l1_ca(),
            signal_spec_gps_l2c(),
        );
        let l2_jump_m = 2.0 * (l2_delay_after_m - l2_delay_before_m);

        let mut before = divergence_epoch(l1_delay_before_m, 106.0, 0.0, l2_delay_before_m, 0.0);
        assert_eq!(ionosphere_delay_evidence_from_epoch(&before).len(), 2);
        apply_code_carrier_divergence_decomposition(&mut before, &mut state);
        let mut after = divergence_epoch(
            l1_delay_after_m,
            106.0 + l1_jump_m,
            l1_jump_m,
            l2_delay_after_m,
            l2_jump_m,
        );
        apply_code_carrier_divergence_decomposition(&mut after, &mut state);

        let sat = after
            .sats
            .iter()
            .find(|sat| sat.signal_id.band == SignalBand::L1)
            .expect("L1 observation");
        let divergence =
            sat.metadata.code_carrier_divergence.expect("code-carrier divergence decomposition");

        assert!(sat.multipath_suspect, "{sat:?}");
        assert!((divergence.expected_ionosphere_m - l1_ionosphere_jump_m).abs() < 1.0e-6);
        assert!((divergence.multipath_m - residual_multipath_m).abs() < 1.0e-6);
        assert!(divergence.unexplained_m.abs() < 1.0e-6);
        assert!(
            (sat.error_model.as_ref().expect("error model").multipath_proxy_m.0
                - residual_multipath_m)
                .abs()
                < 1.0e-6
        );
    }

    #[test]
    fn dual_frequency_cycle_slip_fusion_records_geometry_free_detector() {
        let l1_delay_m = 4.0;
        let l2_delay_m =
            scaled_ionosphere_delay_m(l1_delay_m, signal_spec_gps_l1_ca(), signal_spec_gps_l2c());
        let previous = divergence_epoch(l1_delay_m, 100.0, 0.0, l2_delay_m, 0.0);
        let mut current = divergence_epoch(l1_delay_m, 100.0, 0.0, l2_delay_m, 0.0);
        current.epoch_idx = 71;
        let signal = current.sats[0].metadata.signal;
        current.sats[0].carrier_phase_cycles = Cycles(
            current.sats[0].carrier_phase_cycles.0
                + signal_meters_to_cycles(Meters(0.25), signal).0,
        );
        current.sats[0].pseudorange_m = Meters(current.sats[0].pseudorange_m.0 + 10.0);
        let raw_pseudorange_m = current.sats[0].pseudorange_m.0 - 10.0;
        let mut raw_snapshots = std::collections::HashMap::new();
        raw_snapshots.insert(
            observation_snapshot_key(current.epoch_idx, current.sats[0].signal_id),
            raw_observation_snapshot(&current.sats[0], raw_pseudorange_m),
        );
        let mut hatch = std::collections::HashMap::new();

        apply_dual_frequency_cycle_slip_fusion(
            &mut current,
            Some(&previous),
            &raw_snapshots,
            &mut hatch,
        );

        let sat = current
            .sats
            .iter()
            .find(|sat| sat.signal_id.band == SignalBand::L1)
            .expect("L1 observation");
        let evidence = sat.metadata.cycle_slip_evidence.as_ref().expect("cycle-slip evidence");
        assert!(sat.lock_flags.cycle_slip, "{sat:?}");
        assert!(evidence.triggered_detectors().contains(&CycleSlipDetector::GeometryFreePhase));
        assert_eq!(sat.metadata.observation_lock_reason.as_deref(), Some("geometry_free_phase"));
        assert!((sat.pseudorange_m.0 - raw_pseudorange_m).abs() < 1.0e-9);
        assert_eq!(sat.metadata.smoothing_age, 1);
    }

    #[test]
    fn dual_frequency_cycle_slip_fusion_records_melbourne_wubbena_detector() {
        let l1_delay_m = 4.0;
        let l2_delay_m =
            scaled_ionosphere_delay_m(l1_delay_m, signal_spec_gps_l1_ca(), signal_spec_gps_l2c());
        let previous = divergence_epoch(l1_delay_m, 100.0, 0.0, l2_delay_m, 0.0);
        let mut current = divergence_epoch(l1_delay_m, 100.0, 0.0, l2_delay_m, 0.0);
        current.epoch_idx = 72;
        current.sats[0].pseudorange_m = Meters(current.sats[0].pseudorange_m.0 - 2.0);
        let mut raw_snapshots = std::collections::HashMap::new();
        raw_snapshots.insert(
            observation_snapshot_key(current.epoch_idx, current.sats[0].signal_id),
            raw_observation_snapshot(&current.sats[0], current.sats[0].pseudorange_m.0),
        );
        let mut hatch = std::collections::HashMap::new();

        apply_dual_frequency_cycle_slip_fusion(
            &mut current,
            Some(&previous),
            &raw_snapshots,
            &mut hatch,
        );

        let sat = current
            .sats
            .iter()
            .find(|sat| sat.signal_id.band == SignalBand::L1)
            .expect("L1 observation");
        let evidence = sat.metadata.cycle_slip_evidence.as_ref().expect("cycle-slip evidence");
        assert!(sat.lock_flags.cycle_slip, "{sat:?}");
        assert!(evidence.triggered_detectors().contains(&CycleSlipDetector::MelbourneWubbena));
        assert_eq!(sat.metadata.observation_lock_reason.as_deref(), Some("melbourne_wubbena"));
    }

    fn scaled_ionosphere_delay_m(
        reference_delay_m: f64,
        reference_signal: SignalSpec,
        target_signal: SignalSpec,
    ) -> f64 {
        reference_delay_m * reference_signal.carrier_hz.value().powi(2)
            / target_signal.carrier_hz.value().powi(2)
    }

    fn divergence_epoch(
        l1_ionosphere_delay_m: f64,
        l1_raw_divergence_m: f64,
        l1_divergence_jump_m: f64,
        l2_ionosphere_delay_m: f64,
        l2_divergence_jump_m: f64,
    ) -> ObsEpoch {
        let base_range_m = 20_200_000.0;
        ObsEpoch {
            t_rx_s: Seconds(70.0),
            source_time: ReceiverSampleTrace::from_sample_index(286_440, 4_092_000.0),
            gps_week: None,
            tow_s: None,
            epoch_idx: 70,
            discontinuity: false,
            valid: true,
            processing_ms: None,
            role: ReceiverRole::Rover,
            sats: vec![
                divergence_satellite(
                    signal_spec_gps_l1_ca(),
                    l1_ionosphere_delay_m,
                    l1_raw_divergence_m,
                    l1_divergence_jump_m,
                    base_range_m,
                ),
                divergence_satellite(
                    signal_spec_gps_l2c(),
                    l2_ionosphere_delay_m,
                    100.0 + l2_divergence_jump_m,
                    l2_divergence_jump_m,
                    base_range_m,
                ),
            ],
            decision: ObservationEpochDecision::Accepted,
            decision_reason: None,
            manifest: None,
        }
    }

    fn divergence_satellite(
        signal: SignalSpec,
        ionosphere_delay_m: f64,
        raw_divergence_m: f64,
        divergence_jump_m: f64,
        base_range_m: f64,
    ) -> ObsSatellite {
        let pseudorange_m = base_range_m + ionosphere_delay_m;
        let carrier_phase_m = pseudorange_m - raw_divergence_m;
        ObsSatellite {
            signal_id: SigId {
                sat: SatId { constellation: Constellation::Gps, prn: 9 },
                band: signal.band,
                code: signal.code,
            },
            pseudorange_m: Meters(pseudorange_m),
            pseudorange_var_m2: 1.0,
            carrier_phase_cycles: signal_meters_to_cycles(Meters(carrier_phase_m), signal),
            carrier_phase_var_cycles2: 1.0,
            doppler_hz: Hertz(0.0),
            doppler_var_hz2: 1.0,
            cn0_dbhz: 45.0,
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
            azimuth_deg: Some(0.0),
            weight: None,
            timing: None,
            error_model: Some(bijux_gnss_core::api::MeasurementErrorModel {
                thermal_noise_m: Meters(0.0),
                tracking_jitter_m: Meters(1.0),
                multipath_proxy_m: Meters(0.0),
                clock_error_m: Meters(0.0),
            }),
            metadata: ObsMetadata {
                signal,
                integration_ms: 20,
                code_carrier_divergence: Some(CodeCarrierDivergence::from_terms(
                    raw_divergence_m,
                    divergence_jump_m,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                )),
                ..ObsMetadata::default()
            },
        }
    }
}
