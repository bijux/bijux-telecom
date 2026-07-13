#![allow(missing_docs)]
//! Observation error model assumptions:
//! - pseudorange and Doppler variance prefer tracking-produced uncertainty when available.
//! - thermal_noise_m falls back to a fixed 1.0 m placeholder only when tracking did not emit uncertainty.
//! - tracking_jitter_m falls back to a CN0-derived heuristic only for legacy epochs without tracking uncertainty.
//! - multipath_proxy_m = |Δ divergence| meters (proxy, not a modeled multipath estimator).
//! - clock_error_m = 0.0 m (placeholder).
//!
//! TODO(ref-grade): add literature citations for CN0 weighting and multipath heuristics.

use bijux_gnss_core::api::{
    signal_cycles_to_meters, signal_registry, signal_spec_beidou_b1i, signal_spec_beidou_b2i,
    signal_spec_galileo_e1b, signal_spec_galileo_e5a, signal_spec_glonass_l1,
    signal_spec_gps_l1_ca, signal_spec_gps_l2c, signal_spec_gps_l5, signal_wavelength_m,
    Constellation, ConventionsConfig, Cycles, DiagnosticEvent, DiagnosticSeverity, GpsTime,
    LockFlags, Meters, ObsDecisionArtifact, ObsEpoch, ObsEpochManifest, ObsMetadata, ObsSatellite,
    ObsSignalTiming, ObservationEpochDecision, ObservationStatus, ObservationSupportClass,
    ObservationUncertaintyClass, ReceiverRole, ReceiverSampleTrace, SatObservationDecision,
    Seconds, SigId, SignalBand, SignalCode, SignalSpec, TrackEpoch, GPS_L1_CA_CARRIER_HZ,
};

use crate::engine::receiver_config::ReceiverPipelineConfig;
use crate::pipeline::doppler::doppler_hz_from_carrier_hz;
use crate::pipeline::hatch::HatchFilterState;
use crate::pipeline::tracking::TrackingResult;
use crate::pipeline::{StepReport, StepStats};
use bijux_gnss_signal::api::samples_per_code;
use serde::{Deserialize, Serialize};

#[cfg(test)]
use bijux_gnss_core::api::{glonass_l1_carrier_hz, GlonassFrequencyChannel, Hertz, SatId};

const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;
const OBS_WEAK_CN0_DBHZ: f64 = 25.0;
const OBS_BASE_THERMAL_CODE_SIGMA_SAMPLES: f64 = 0.02;
const OBS_MIN_TRACKING_JITTER_SAMPLES: f64 = 0.01;

#[derive(Debug, Clone)]
struct PseudorangeComputation {
    pseudorange_m: Meters,
    timing: Option<ObsSignalTiming>,
    model: &'static str,
    alignment_source: Option<String>,
    alignment_resolved: bool,
}

#[derive(Debug, Clone)]
struct CarrierPhaseArcState {
    phase_cycles: f64,
    doppler_hz: f64,
    last_sample_index: u64,
    arc_start_epoch_idx: u64,
    arc_start_sample_index: u64,
    pending_reset: Option<CarrierPhaseContinuity>,
    pending_reset_reason: Option<String>,
    initialized: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum CarrierPhaseContinuity {
    Unusable,
    ArcStart,
    Continuous,
    ResetAfterCycleSlip,
    ResetAfterUnlock,
    ResetAfterDiscontinuity,
}

#[derive(Debug, Clone)]
struct CarrierPhaseObservation {
    phase_cycles: Cycles,
    cycle_slip: bool,
    cycle_slip_reason: Option<String>,
    continuity: CarrierPhaseContinuity,
    arc_start_epoch_idx: u64,
    arc_start_sample_index: u64,
}

#[derive(Debug, Clone, Copy)]
struct PseudorangeUncertaintyModel {
    sigma_m: f64,
    thermal_noise_m: f64,
    tracking_jitter_m: f64,
}

const BASE_CARRIER_PHASE_DISCONTINUITY_THRESHOLD_CYCLES: f64 = 0.25;
const BASE_CARRIER_PHASE_RESIDUAL_THRESHOLD_CYCLES: f64 = 0.15;
const BASE_DOPPLER_JUMP_THRESHOLD_HZ: f64 = 150.0;
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

#[derive(Debug, Clone)]
struct ObservationSignalModel {
    signal_id: SigId,
    signal: SignalSpec,
    code_length: usize,
    samples_per_chip: f64,
    carrier_reference_hz: f64,
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
            pseudorange_sigma_m: sigma_from_variance(sat.pseudorange_var_m2),
            carrier_phase_sigma_cycles: sigma_from_variance(sat.carrier_phase_var_cycles2),
            doppler_sigma_hz: sigma_from_variance(sat.doppler_var_hz2),
            cn0_sigma_dbhz: sat
                .metadata
                .tracking_uncertainty
                .as_ref()
                .and_then(|uncertainty| finite_sigma(Some(uncertainty.cn0_dbhz))),
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
        pending_reset: None,
        pending_reset_reason: None,
        initialized: false,
    };

    let mut out = Vec::with_capacity(epochs.len());
    let mut diagnostics = Vec::new();
    let mut last_sample_index = None;
    let mut last_locked_sample = None;
    let clock_bias_s = 0.0_f64;

    for epoch in epochs {
        let signal_model = observation_signal_model(config, epoch);
        let (time_tag_source, time_tag_sample_index) =
            tracking_time_tag(epoch, &mut last_locked_sample);
        let t_rx_s = Seconds(epoch.sample_index as f64 / config.sampling_freq_hz + clock_bias_s);

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

        let receive_gps_time =
            capture_start_gps_time.map(|gps_time| gps_time.offset_seconds(t_rx_s.0));
        let pseudorange = pseudorange_from_tracking_epoch(
            epoch,
            signal_model.samples_per_chip,
            signal_model.signal.code_rate_hz,
            signal_model.code_length as f64,
            receive_gps_time,
            clock_bias_s,
        );

        let tracked_carrier_phase_cycles = epoch.carrier_phase_cycles.0;
        let doppler_hz = bijux_gnss_core::api::Hertz(doppler_hz_from_carrier_hz(
            signal_model.carrier_reference_hz,
            epoch.carrier_hz.0,
        ));
        let carrier_phase = carrier_phase_observation(
            epoch,
            tracked_carrier_phase_cycles,
            doppler_hz.0,
            config.sampling_freq_hz,
            discontinuity,
            &mut carrier_phase_state,
        );

        let cn0_dbhz = normalize_observation_cn0_dbhz(epoch.cn0_dbhz);
        let lock_quality = tracking_lock_quality(epoch, carrier_phase.cycle_slip);
        let tracking_uncertainty = epoch.tracking_uncertainty.clone();
        let pseudorange_uncertainty = pseudorange_uncertainty_model(
            config,
            epoch,
            signal_model.samples_per_chip,
            meters_per_sample,
            lock_quality,
        );

        let variance_m2 = pseudorange_uncertainty.sigma_m.powi(2);
        let carrier_phase_var_cycles2 = tracking_uncertainty
            .as_ref()
            .map(|uncertainty| uncertainty.carrier_phase_cycles.powi(2))
            .unwrap_or(variance_m2 * 0.01);
        let doppler_var_hz2 = tracking_uncertainty
            .as_ref()
            .map(|uncertainty| uncertainty.doppler_hz.powi(2))
            .unwrap_or(4.0);
        let (observation_status, observation_reject_reasons) =
            classify_observation_status(epoch, cn0_dbhz);
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
            pseudorange_var_m2: variance_m2,
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
            error_model: Some(bijux_gnss_core::api::MeasurementErrorModel {
                thermal_noise_m: Meters(pseudorange_uncertainty.thermal_noise_m),
                tracking_jitter_m: Meters(pseudorange_uncertainty.tracking_jitter_m),
                multipath_proxy_m: Meters(0.0),
                clock_error_m: Meters(0.0),
            }),
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
                observation_uncertainty_class: observation_uncertainty_label(cn0_dbhz).to_string(),
                pseudorange_model: pseudorange.model.to_string(),
                carrier_phase_model: "tracked_carrier_cycles".to_string(),
                doppler_model: doppler_model_label().to_string(),
                carrier_phase_continuity: carrier_phase_continuity_label(carrier_phase.continuity)
                    .to_string(),
                carrier_phase_arc_start_epoch_idx: carrier_phase.arc_start_epoch_idx,
                carrier_phase_arc_start_sample_index: carrier_phase.arc_start_sample_index,
                signal_delay_alignment_source: pseudorange.alignment_source.unwrap_or_default(),
                time_tag_source,
                time_tag_sample_index,
                time_tag_sample_rate_hz: config.sampling_freq_hz,
                tracking_uncertainty,
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
    let mut diagnostics = Vec::new();
    for track in tracks {
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
                    sat.error_model = Some(observation_error_model(&sat, 0.0));
                    apply_pseudorange_physics_rejection(&mut sat);
                    entry.sats.push(sat);
                    continue;
                }
                let supports_code_carrier_slip_detection =
                    sat.metadata.pseudorange_model == "tracked_code_phase_alignment";
                let raw_pseudorange_m = sat.pseudorange_m.0;
                raw_snapshots
                    .insert(snapshot_key, raw_observation_snapshot(&sat, raw_pseudorange_m));
                let raw_divergence_m = raw_pseudorange_m
                    - signal_cycles_to_meters(sat.carrier_phase_cycles, sat.metadata.signal).0;
                let threshold_m = slip_threshold_m(sat.cn0_dbhz, sat.elevation_deg);
                let divergence_jump = state.divergence_jump_m(raw_divergence_m).unwrap_or(0.0);
                let mut smoothing_cycle_slip_reason = None;
                if supports_code_carrier_slip_detection && divergence_jump > threshold_m {
                    smoothing_cycle_slip_reason = Some("code_carrier_divergence");
                }
                apply_cycle_slip_surface(&mut sat, smoothing_cycle_slip_reason);
                if sat.lock_flags.cycle_slip {
                    state.clear_arc();
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
                sat.multipath_suspect = divergence_jump > threshold_m * 0.8;
                sat.error_model = Some(observation_error_model(&sat, divergence_jump));
                apply_pseudorange_physics_rejection(&mut sat);
                entry.sats.push(sat);
            }
        }
    }
    let mut out = Vec::new();
    let mut previous_epoch: Option<ObsEpoch> = None;
    for mut epoch in by_epoch.into_values() {
        bijux_gnss_core::api::sort_obs_sats(&mut epoch);
        apply_epoch_decision(&mut epoch);
        if let Some(prev) = previous_epoch.as_ref() {
            if let Some(event) = observation_timing_interval_diagnostic(config, prev, &epoch) {
                diagnostics.push(event);
                reject_epoch_for_invalid_timing(&mut epoch);
            }
        }
        let source_time = epoch.source_time;
        let artifact_id = format!("obs-epoch-{:010}", epoch.epoch_idx);
        epoch.t_rx_s = source_time.receiver_time_s;
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
                sat.metadata.pseudorange_model == "tracked_code_phase_alignment";
            sat.metadata.observation_support_class =
                observation_support_label(sat.observation_status, alignment_resolved).to_string();
            sat.metadata.observation_uncertainty_class =
                observation_uncertainty_label(sat.cn0_dbhz).to_string();
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
        if let Err(err) = bijux_gnss_core::api::validate_obs_epochs(&out) {
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
                    let pseudorange_sigma = Some(sat.pseudorange_var_m2.sqrt());
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
                    let doppler_sigma = Some(sat.doppler_var_hz2.sqrt());
                    let cn0_expected = Some(snapshot.raw_cn0_dbhz);
                    let cn0_sigma = sat
                        .metadata
                        .tracking_uncertainty
                        .as_ref()
                        .map(|uncertainty| uncertainty.cn0_dbhz);
                    let carrier_phase_sigma = Some(sat.carrier_phase_var_cycles2.sqrt());
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

fn finite_value(value: Option<f64>) -> Option<f64> {
    value.filter(|candidate| candidate.is_finite())
}

fn finite_sigma(value: Option<f64>) -> Option<f64> {
    value.filter(|candidate| candidate.is_finite() && *candidate >= 0.0)
}

fn sigma_from_variance(variance: f64) -> Option<f64> {
    finite_sigma((variance.is_finite() && variance >= 0.0).then(|| variance.sqrt()))
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

fn observation_status_label(status: ObservationStatus) -> &'static str {
    match status {
        ObservationStatus::Accepted => "accepted",
        ObservationStatus::Missing => "missing",
        ObservationStatus::Weak => "weak",
        ObservationStatus::Inconsistent => "inconsistent",
        ObservationStatus::Rejected => "rejected",
    }
}

fn observation_support_label(status: ObservationStatus, alignment_resolved: bool) -> &'static str {
    match status {
        ObservationStatus::Accepted if alignment_resolved => {
            support_class_label(ObservationSupportClass::Supported)
        }
        ObservationStatus::Accepted => support_class_label(ObservationSupportClass::Degraded),
        ObservationStatus::Weak | ObservationStatus::Missing => {
            support_class_label(ObservationSupportClass::Degraded)
        }
        ObservationStatus::Inconsistent | ObservationStatus::Rejected => {
            support_class_label(ObservationSupportClass::Unsupported)
        }
    }
}

fn doppler_model_label() -> &'static str {
    bijux_gnss_core::api::OBSERVATION_DOPPLER_MODEL_TRACKED_CARRIER_IF_OFFSET
}

fn observation_signal_model(
    config: &ReceiverPipelineConfig,
    epoch: &TrackEpoch,
) -> ObservationSignalModel {
    let signal_code = tracked_signal_code(epoch);
    let registry_entry = signal_registry(epoch.sat.constellation, epoch.signal_band, signal_code);
    let signal = match (epoch.sat.constellation, epoch.signal_band, epoch.glonass_frequency_channel)
    {
        (Constellation::Gps, SignalBand::L1, _) => signal_spec_gps_l1_ca(),
        (Constellation::Gps, SignalBand::L2, _) => signal_spec_gps_l2c(),
        (Constellation::Gps, SignalBand::L5, _) => signal_spec_gps_l5(),
        (Constellation::Galileo, SignalBand::E1, _) => signal_spec_galileo_e1b(),
        (Constellation::Galileo, SignalBand::E5, _) => signal_spec_galileo_e5a(),
        (Constellation::Beidou, SignalBand::B1, _) => signal_spec_beidou_b1i(),
        (Constellation::Beidou, SignalBand::B2, _) => signal_spec_beidou_b2i(),
        (Constellation::Glonass, SignalBand::L1, Some(channel)) => signal_spec_glonass_l1(channel),
        _ => registry_entry.as_ref().map(|entry| entry.spec).unwrap_or_else(signal_spec_gps_l1_ca),
    };
    let code_length = registry_entry
        .as_ref()
        .and_then(|entry| entry.code_length)
        .map(|length| length as usize)
        .unwrap_or_else(|| fallback_code_length(epoch));
    let samples_per_code =
        samples_per_code(config.sampling_freq_hz, signal.code_rate_hz, code_length);
    ObservationSignalModel {
        signal_id: SigId { sat: epoch.sat, band: signal.band, code: signal.code },
        signal,
        code_length,
        samples_per_chip: samples_per_code as f64 / code_length as f64,
        carrier_reference_hz: tracked_signal_center_hz(config.intermediate_freq_hz, signal),
    }
}

fn tracked_signal_code(epoch: &TrackEpoch) -> SignalCode {
    match (epoch.sat.constellation, epoch.signal_band) {
        (Constellation::Gps, SignalBand::L1) => SignalCode::Ca,
        (Constellation::Gps, SignalBand::L2) => SignalCode::L2C,
        (Constellation::Gps, SignalBand::L5) => SignalCode::Unknown,
        (Constellation::Galileo, SignalBand::E1) => SignalCode::E1B,
        (Constellation::Galileo, SignalBand::E5) => SignalCode::E5a,
        (Constellation::Beidou, SignalBand::B1) => SignalCode::B1I,
        (Constellation::Beidou, SignalBand::B2) => SignalCode::B2I,
        (Constellation::Glonass, SignalBand::L1) => SignalCode::Unknown,
        _ => SignalCode::Unknown,
    }
}

fn fallback_code_length(epoch: &TrackEpoch) -> usize {
    match (epoch.sat.constellation, epoch.signal_band) {
        (Constellation::Gps, SignalBand::L1) => 1023,
        (Constellation::Gps, SignalBand::L2) => 10230,
        (Constellation::Gps, SignalBand::L5) => 10230,
        (Constellation::Galileo, SignalBand::E1) => 4092,
        (Constellation::Galileo, SignalBand::E5) => 10230,
        (Constellation::Beidou, SignalBand::B1) => 2046,
        (Constellation::Beidou, SignalBand::B2) => 2046,
        (Constellation::Glonass, SignalBand::L1) => 511,
        _ => 1023,
    }
}

fn tracked_signal_center_hz(intermediate_freq_hz: f64, signal: SignalSpec) -> f64 {
    intermediate_freq_hz + (signal.carrier_hz.value() - GPS_L1_CA_CARRIER_HZ.value())
}

fn pseudorange_from_tracking_epoch(
    epoch: &TrackEpoch,
    samples_per_chip: f64,
    code_rate_hz: f64,
    code_length_chips: f64,
    receive_gps_time: Option<GpsTime>,
    clock_bias_s: f64,
) -> PseudorangeComputation {
    let code_phase_chips =
        aligned_code_phase_chips(code_length_chips, epoch.code_phase_samples.0 / samples_per_chip);
    if let Some(alignment) = &epoch.signal_delay_alignment {
        let code_time_s = (alignment.whole_code_periods as f64 * code_length_chips
            + code_phase_chips)
            / code_rate_hz;
        let pseudorange_m =
            Meters(code_time_s * SPEED_OF_LIGHT_MPS + clock_bias_s * SPEED_OF_LIGHT_MPS);
        let signal_travel_time_s = Seconds(pseudorange_m.0 / SPEED_OF_LIGHT_MPS);
        let timing = receive_gps_time.map(|gps_time| ObsSignalTiming {
            signal_travel_time_s,
            transmit_gps_time: gps_time.offset_seconds(-signal_travel_time_s.0),
        });
        return PseudorangeComputation {
            pseudorange_m,
            timing,
            model: "tracked_code_phase_alignment",
            alignment_source: Some(alignment.source.clone()),
            alignment_resolved: true,
        };
    }

    let code_epoch = epoch.epoch.index as f64;
    let code_time_s = (code_epoch * code_length_chips + code_phase_chips) / code_rate_hz;
    PseudorangeComputation {
        pseudorange_m: Meters(code_time_s * SPEED_OF_LIGHT_MPS + clock_bias_s * SPEED_OF_LIGHT_MPS),
        timing: None,
        model: "receiver_epoch_fallback",
        alignment_source: None,
        alignment_resolved: false,
    }
}

fn carrier_phase_observation(
    epoch: &TrackEpoch,
    tracked_carrier_phase_cycles: f64,
    doppler_hz: f64,
    sample_rate_hz: f64,
    discontinuity: bool,
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
        return CarrierPhaseObservation {
            phase_cycles: Cycles(tracked_carrier_phase_cycles),
            cycle_slip: cycle_slip_reason.is_some(),
            cycle_slip_reason,
            continuity: CarrierPhaseContinuity::Unusable,
            arc_start_epoch_idx: 0,
            arc_start_sample_index: 0,
        };
    }

    let delta_seconds =
        carrier_phase_delta_seconds(state.last_sample_index, epoch.sample_index, sample_rate_hz);
    let observed_phase_delta_cycles = tracked_carrier_phase_cycles - state.phase_cycles;
    let predicted_from_previous_doppler_cycles = state.doppler_hz * delta_seconds;
    let predicted_from_trapezoid_cycles = 0.5 * (state.doppler_hz + doppler_hz) * delta_seconds;
    let phase_discontinuity = state.initialized
        && (observed_phase_delta_cycles - predicted_from_previous_doppler_cycles).abs()
            > carrier_phase_discontinuity_threshold_cycles(epoch);
    let doppler_jump = state.initialized
        && (doppler_hz - state.doppler_hz).abs() > doppler_jump_threshold_hz(epoch);
    let phase_residual = state.initialized
        && (observed_phase_delta_cycles - predicted_from_trapezoid_cycles).abs()
            > carrier_phase_residual_threshold_cycles(epoch);
    let explicit_reset = if epoch.cycle_slip {
        Some((
            CarrierPhaseContinuity::ResetAfterCycleSlip,
            explicit_cycle_slip_reason(epoch, "cycle_slip"),
        ))
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
                | CarrierPhaseContinuity::ResetAfterDiscontinuity
        );

    if continuity != CarrierPhaseContinuity::Continuous {
        state.arc_start_epoch_idx = epoch.epoch.index;
        state.arc_start_sample_index = epoch.sample_index;
    }
    state.phase_cycles = tracked_carrier_phase_cycles;
    state.doppler_hz = doppler_hz;
    state.last_sample_index = epoch.sample_index;
    state.initialized = true;

    CarrierPhaseObservation {
        phase_cycles: Cycles(tracked_carrier_phase_cycles),
        cycle_slip,
        cycle_slip_reason,
        continuity,
        arc_start_epoch_idx: state.arc_start_epoch_idx,
        arc_start_sample_index: state.arc_start_sample_index,
    }
}

fn aligned_code_phase_chips(code_length_chips: f64, tracking_code_phase_chips: f64) -> f64 {
    if !tracking_code_phase_chips.is_finite() || tracking_code_phase_chips < 0.0 {
        return tracking_code_phase_chips;
    }
    (code_length_chips - tracking_code_phase_chips.rem_euclid(code_length_chips))
        .rem_euclid(code_length_chips)
}

fn carrier_phase_tracking_usable(epoch: &TrackEpoch) -> bool {
    epoch.lock
        && epoch.pll_lock
        && epoch.lock_state != "lost"
        && epoch.carrier_phase_cycles.0.is_finite()
}

fn explicit_cycle_slip_reason(epoch: &TrackEpoch, fallback: &str) -> String {
    epoch
        .cycle_slip_reason
        .clone()
        .or_else(|| epoch.lock_state_reason.clone())
        .unwrap_or_else(|| fallback.to_string())
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
        CarrierPhaseContinuity::ResetAfterDiscontinuity => 2,
        CarrierPhaseContinuity::ResetAfterUnlock => 1,
        CarrierPhaseContinuity::Unusable
        | CarrierPhaseContinuity::ArcStart
        | CarrierPhaseContinuity::Continuous => 0,
    }
}

fn carrier_phase_continuity_label(value: CarrierPhaseContinuity) -> &'static str {
    match value {
        CarrierPhaseContinuity::Unusable => "unusable",
        CarrierPhaseContinuity::ArcStart => "arc_start",
        CarrierPhaseContinuity::Continuous => "continuous",
        CarrierPhaseContinuity::ResetAfterCycleSlip => "reset_after_cycle_slip",
        CarrierPhaseContinuity::ResetAfterUnlock => "reset_after_unlock",
        CarrierPhaseContinuity::ResetAfterDiscontinuity => "reset_after_discontinuity",
    }
}

fn support_class_label(value: ObservationSupportClass) -> &'static str {
    match value {
        ObservationSupportClass::Supported => "supported",
        ObservationSupportClass::Degraded => "degraded",
        ObservationSupportClass::Unsupported => "unsupported",
    }
}

fn observation_uncertainty_label(cn0_dbhz: f64) -> &'static str {
    let class = if !cn0_dbhz.is_finite() {
        ObservationUncertaintyClass::Unknown
    } else if cn0_dbhz >= 45.0 {
        ObservationUncertaintyClass::Low
    } else if cn0_dbhz >= 35.0 {
        ObservationUncertaintyClass::Medium
    } else {
        ObservationUncertaintyClass::High
    };
    match class {
        ObservationUncertaintyClass::Low => "low",
        ObservationUncertaintyClass::Medium => "medium",
        ObservationUncertaintyClass::High => "high",
        ObservationUncertaintyClass::Unknown => "unknown",
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
    if let Err(reason) = bijux_gnss_core::api::validate_obs_epochs(std::slice::from_ref(epoch)) {
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

fn pseudorange_uncertainty_model(
    config: &ReceiverPipelineConfig,
    epoch: &TrackEpoch,
    samples_per_chip: f64,
    meters_per_sample: f64,
    lock_quality: f64,
) -> PseudorangeUncertaintyModel {
    let tracking_jitter_samples = epoch
        .tracking_uncertainty
        .as_ref()
        .map(|uncertainty| uncertainty.code_phase_samples)
        .unwrap_or_else(|| {
            ((epoch.dll_err.abs() as f64) * samples_per_chip).max(OBS_MIN_TRACKING_JITTER_SAMPLES)
        });
    let cn0_scale = if epoch.cn0_dbhz.is_finite() {
        10.0_f64.powf(((45.0 - epoch.cn0_dbhz).clamp(-15.0, 20.0)) / 20.0)
    } else {
        4.0
    };
    let integration_scale = 1.0 / (config.tracking_integration_ms.max(1) as f64).sqrt();
    let dll_scale = if epoch.dll_lock { 1.0 } else { 1.5 };
    let lock_quality_scale = 1.0 / lock_quality.clamp(0.25, 1.0);
    let thermal_noise_samples = OBS_BASE_THERMAL_CODE_SIGMA_SAMPLES
        * cn0_scale
        * integration_scale
        * dll_scale
        * lock_quality_scale;
    let discriminator_samples =
        (epoch.dll_err.abs() as f64) * samples_per_chip * dll_scale * lock_quality_scale;
    let tracking_jitter_m = tracking_jitter_samples * meters_per_sample;
    let thermal_noise_m = thermal_noise_samples.max(discriminator_samples) * meters_per_sample;
    let sigma_m = tracking_jitter_m.max(thermal_noise_m);

    PseudorangeUncertaintyModel { sigma_m, thermal_noise_m, tracking_jitter_m }
}

fn observation_error_model(
    sat: &ObsSatellite,
    multipath_proxy_m: f64,
) -> bijux_gnss_core::api::MeasurementErrorModel {
    let fallback_sigma_m = sat.pseudorange_var_m2.sqrt();
    let (thermal_noise_m, tracking_jitter_m) = sat
        .error_model
        .as_ref()
        .map(|model| (model.thermal_noise_m, model.tracking_jitter_m))
        .unwrap_or((Meters(fallback_sigma_m), Meters(fallback_sigma_m)));

    bijux_gnss_core::api::MeasurementErrorModel {
        thermal_noise_m,
        tracking_jitter_m,
        multipath_proxy_m: Meters(multipath_proxy_m),
        clock_error_m: Meters(0.0),
    }
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
                signal: bijux_gnss_core::api::signal_spec_gps_l1_ca(),
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
    use bijux_gnss_core::api::{
        signal_cycles_to_meters, signal_meters_to_cycles, Chips, Epoch, Hertz, Meters, SatId,
        SignalDelayAlignment, TrackingUncertainty,
    };

    fn make_tracking_epoch(
        prn: u8,
        config: &ReceiverPipelineConfig,
        epoch_idx: u64,
        carrier_hz: f64,
    ) -> TrackEpoch {
        make_tracking_epoch_with_phase(prn, config, epoch_idx, carrier_hz, 0.0)
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
            processing_ms: None,
            ..TrackEpoch::default()
        }
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
        assert_eq!(observations[1].sats[0].metadata.carrier_phase_continuity, "continuous");
        assert_eq!(observations[2].sats[0].metadata.carrier_phase_continuity, "continuous");
        assert!((observations[1].sats[0].carrier_phase_cycles.0 - 10.125).abs() <= f64::EPSILON);
        assert_eq!(
            observations[2].sats[0].metadata.carrier_phase_arc_start_sample_index,
            observations[0].sats[0].metadata.carrier_phase_arc_start_sample_index
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
        assert_eq!(relocked.metadata.carrier_phase_continuity, "reset_after_unlock");
        assert_eq!(relocked.metadata.carrier_phase_arc_start_epoch_idx, 72);
        assert_eq!(
            relocked.metadata.carrier_phase_arc_start_sample_index,
            epoch_sample_index(&config, 72)
        );
        assert!(relocked.lock_flags.cycle_slip);
        assert!((relocked.carrier_phase_cycles.0 - 0.5).abs() <= f64::EPSILON);
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
        assert!(slipped.lock_flags.cycle_slip);
        assert_eq!(post_slip.metadata.carrier_phase_continuity, "continuous");
        assert_eq!(post_slip.metadata.carrier_phase_arc_start_epoch_idx, 71);
        assert_eq!(
            post_slip.metadata.carrier_phase_arc_start_sample_index,
            epoch_sample_index(&config, 71)
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
        assert!((slipped.pseudorange_m.0 - expected_raw).abs() <= f64::EPSILON);
        assert_eq!(post_slip.metadata.smoothing_age, 2);
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
                tracking_uncertainty: Some(uncertainty.clone()),
                ..TrackEpoch::default()
            }],
            transitions: Vec::new(),
        };

        let report = observations_from_tracking_results(&config, &[track], 10);
        let epoch = report.output.first().expect("observation epoch");
        let sat = epoch.sats.first().expect("observation satellite");
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
        assert!(error_model.thermal_noise_m.0 > 0.0, "{error_model:?}");
        assert!(
            error_model.thermal_noise_m.0 <= error_model.tracking_jitter_m.0,
            "{error_model:?}"
        );
    }

    #[test]
    fn observations_inflate_pseudorange_variance_for_weaker_cn0() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            ..ReceiverPipelineConfig::default()
        };
        let mut strong_epoch = make_observation_ready_epoch(30, &config, 70);
        strong_epoch.cn0_dbhz = 48.0;
        let mut weak_epoch = make_observation_ready_epoch(31, &config, 70);
        weak_epoch.cn0_dbhz = 28.0;

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
    fn observations_reward_longer_tracking_integration_in_pseudorange_variance() {
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
        let mut long_epoch = make_observation_ready_epoch(32, &long_config, 70);
        long_epoch.cn0_dbhz = 40.0;

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
    fn observations_inflate_pseudorange_variance_when_dll_lock_is_lost() {
        let config = ReceiverPipelineConfig::default();
        let locked_epoch = make_observation_ready_epoch(33, &config, 70);
        let mut unlocked_epoch = make_observation_ready_epoch(34, &config, 70);
        unlocked_epoch.dll_lock = false;

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
    fn observations_inflate_pseudorange_variance_for_low_lock_quality() {
        let config = ReceiverPipelineConfig::default();
        let locked_epoch = make_observation_ready_epoch(35, &config, 70);
        let mut guarded_epoch = make_observation_ready_epoch(36, &config, 70);
        guarded_epoch.anti_false_lock = true;

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
        assert_eq!(obs_sat.signal_id.code, SignalCode::Unknown);
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
}
