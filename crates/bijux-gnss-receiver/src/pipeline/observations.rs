#![allow(missing_docs)]
//! Observation error model assumptions:
//! - code, carrier, Doppler, and C/N0 uncertainty must come from tracking evidence.
//! - observations without defensible tracking uncertainty are marked missing rather than weighted.
//! - code-carrier divergence is decomposed before multipath is inferred.
//! - clock_error_m comes from configured receiver-clock bias uncertainty.

use bijux_gnss_core::api::{
    DiagnosticEvent, GpsTime, LockFlags, ObsEpoch, ObsMetadata, ObsSatellite,
    ObservationEpochDecision, ReceiverRole, Seconds, SigId, TrackEpoch,
};

use crate::engine::receiver_config::ReceiverPipelineConfig;
use crate::pipeline::doppler::doppler_hz_from_carrier_hz;
use crate::pipeline::hatch::HatchFilterState;
use crate::pipeline::observations::carrier_phase::{
    carrier_phase_observation, CarrierPhaseArcState,
};
#[cfg(test)]
use crate::pipeline::observations::carrier_phase::{
    CYCLE_SLIP_DETECTION_PROBABILITY_BUDGET, CYCLE_SLIP_FALSE_ALARM_PROBABILITY_BUDGET,
};
use crate::pipeline::observations::code_carrier_divergence::{
    apply_code_carrier_divergence_decomposition, CodeCarrierDivergenceState,
};
#[cfg(test)]
use crate::pipeline::observations::code_carrier_divergence::{
    dual_frequency_code_delay_evidence, ionosphere_delay_evidence_from_epoch,
};
use crate::pipeline::observations::code_period_ambiguity::{
    apply_grouped_integer_code_period_ambiguities, code_period_ambiguity_input_from_tracking_epoch,
    CodePeriodAmbiguityInput,
};
use crate::pipeline::observations::cycle_slip_fusion::apply_dual_frequency_cycle_slip_fusion;
use crate::pipeline::observations::hatch_smoothing::apply_hatch_smoothing;
use crate::pipeline::observations::labels::{
    carrier_phase_continuity_label, doppler_model_label, observation_status_label,
    observation_support_label, observation_uncertainty_label,
};
use crate::pipeline::observations::lock_state::{
    apply_cycle_slip_surface, cycle_slip_reason, observation_lock_reason, observation_lock_state,
    tracking_lock_quality,
};
use crate::pipeline::observations::measurement_quality::observation_measurement_quality_from_epochs;
use crate::pipeline::observations::pseudorange_timing::pseudorange_from_tracking_epoch;
use crate::pipeline::observations::receiver_clock::{
    observation_receiver_clock, receiver_clock_carrier_phase_cycles,
};
use crate::pipeline::observations::residual_reports::{
    observation_residual_reports_from_epochs, raw_observation_snapshot, RawObservationSnapshot,
};
use crate::pipeline::observations::signal_model::observation_signal_model;
use crate::pipeline::observations::status::{
    apply_epoch_decision, apply_pseudorange_physics_rejection, classify_observation_status,
};
use crate::pipeline::observations::timing::{
    grouped_epoch_time_mismatch_diagnostic, grouped_epoch_time_mismatch_reasons,
    mark_grouped_epoch_time_mismatch, normalize_observation_cn0_dbhz, observation_epoch_id,
    observation_epoch_interval_samples, observation_timing_interval_diagnostic,
    reject_epoch_for_invalid_timing, tracking_epoch_has_sample_discontinuity, tracking_time_tag,
};
use crate::pipeline::observations::variance::{
    apply_variance_evidence_status, observation_variance_evidence,
};
use crate::pipeline::tracking::TrackingResult;
use crate::pipeline::{StepReport, StepStats};
use serde::{Deserialize, Serialize};

#[cfg(test)]
use bijux_gnss_core::api::{
    CodeCarrierDivergence, Constellation, CycleSlipDetector, Cycles, GlonassFrequencyChannel,
    ObsEpochManifest, ObservationStatus, ReceiverSampleTrace, SignalBand, SignalCode, SignalSpec,
    GPS_L1_CA_CARRIER_HZ,
};
#[cfg(test)]
use bijux_gnss_signal::api::{glonass_l1_carrier_hz, samples_per_code};
#[cfg(test)]
use bijux_gnss_signal::api::{
    signal_spec_beidou_b2i, signal_spec_galileo_e5a, signal_spec_gps_l1_ca, signal_spec_gps_l2c,
    signal_spec_gps_l5, signal_spec_gps_l5_q,
};
#[cfg(test)]
use timing::observation_interval_samples;

mod carrier_phase;
mod code_carrier_divergence;
mod code_period_ambiguity;
mod cycle_slip_fusion;
mod decision_artifacts;
mod epoch_manifest;
mod epoch_validation;
mod hatch_smoothing;
mod labels;
mod lock_state;
mod measurement_quality;
#[cfg(test)]
mod nav_epoch_fixture;
mod pseudorange_timing;
mod receiver_clock;
mod residual_reports;
mod signal_model;
mod status;
mod timing;
mod tracking_reports;
mod variance;

#[cfg(test)]
use code_period_ambiguity::{
    resolve_integer_code_period_ambiguities, CODE_PERIOD_AMBIGUITY_EPS_S,
    CODE_PERIOD_AMBIGUITY_NON_UNIQUE,
};
pub use decision_artifacts::observation_decisions_from_epochs;
use epoch_manifest::stamp_observation_epoch_manifest;
use epoch_validation::{apply_observation_epoch_sanity, validate_observation_epoch_sequence};
pub use measurement_quality::{
    ObservationMeasurementQualityEpochReport, ObservationMeasurementQualitySatellite,
};
#[cfg(test)]
use nav_epoch_fixture::nav_observation_epoch_fixture;
#[cfg(test)]
use pseudorange_timing::resolve_pseudorange_from_transmit_time;
use residual_reports::{observation_signal_key, observation_snapshot_key};
pub use residual_reports::{
    ObservationResidualEpochReport, ObservationResidualSatellite, ObservationResidualValue,
};
pub(crate) use signal_model::supports_observation_signal;
#[cfg(test)]
use signal_model::{tracked_signal_center_hz, tracked_signal_code_for_band};
pub use tracking_reports::{
    observation_artifacts_from_tracking_results,
    observation_measurement_quality_from_tracking_results,
    observation_measurement_quality_from_tracking_results_with_gps_anchor,
    observation_residuals_from_tracking_results,
    observation_residuals_from_tracking_results_with_gps_anchor,
    observations_from_tracking_results, observations_from_tracking_results_with_gps_anchor,
};

const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ObservationPipelineArtifacts {
    pub epochs: Vec<ObsEpoch>,
    pub residuals: Vec<ObservationResidualEpochReport>,
    pub measurement_quality: Vec<ObservationMeasurementQualityEpochReport>,
}

#[cfg(test)]
pub(crate) fn accepted_rover_observation_epoch(
    t_rx_s: Seconds,
    source_time: ReceiverSampleTrace,
    gps_week: Option<u32>,
    tow_s: Option<Seconds>,
    epoch_idx: u64,
    discontinuity: bool,
    sats: Vec<ObsSatellite>,
    decision_reason: Option<String>,
    manifest: Option<ObsEpochManifest>,
) -> ObsEpoch {
    ObsEpoch {
        t_rx_s,
        source_time,
        gps_week,
        tow_s,
        epoch_idx,
        discontinuity,
        valid: true,
        processing_ms: None,
        role: ReceiverRole::Rover,
        sats,
        decision: ObservationEpochDecision::Accepted,
        decision_reason,
        manifest,
    }
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

    let mut carrier_phase_state = CarrierPhaseArcState::default();

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
        let discontinuity = tracking_epoch_has_sample_discontinuity(
            epoch,
            last_sample_index,
            expected_step,
            &mut diagnostics,
        );
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
        apply_observation_epoch_sanity(&mut epoch, &mut diagnostics);
        apply_epoch_decision(&mut epoch);
        out.push(epoch);
    }

    (out, diagnostics)
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
                apply_hatch_smoothing(
                    &mut sat,
                    snapshot_key,
                    hatch_window,
                    &mut raw_snapshots,
                    &mut hatch,
                );
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
        stamp_observation_epoch_manifest(&mut epoch, &receiver_clock);
        apply_observation_epoch_sanity(&mut epoch, &mut diagnostics);
        previous_epoch = Some(epoch.clone());
        out.push(epoch);
    }
    validate_observation_epoch_sequence(&out, &mut diagnostics);
    let residuals = observation_residual_reports_from_epochs(&out, &raw_snapshots);
    let measurement_quality = observation_measurement_quality_from_epochs(&out);
    StepReport {
        output: ObservationPipelineArtifacts { epochs: out, residuals, measurement_quality },
        events: diagnostics,
        stats: StepStats::default(),
    }
}

#[cfg(test)]
#[path = "observations/tests.rs"]
mod tests;
