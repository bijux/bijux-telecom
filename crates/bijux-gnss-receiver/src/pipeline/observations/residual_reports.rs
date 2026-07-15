use std::collections::HashMap;

use bijux_gnss_core::api::{
    ObsEpoch, ObsSatellite, ObservationEpochDecision, ObservationStatus, ReceiverSampleTrace, SigId,
};
use serde::{Deserialize, Serialize};

use super::labels::carrier_phase_continuity_label;
use super::variance::{evidence_sigma_from_variance, finite_sigma, finite_value};
use super::{CarrierPhaseContinuity, SPEED_OF_LIGHT_MPS};

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

#[derive(Debug, Clone)]
pub(super) struct RawObservationSnapshot {
    pub(super) raw_pseudorange_m: f64,
    raw_carrier_phase_cycles: f64,
    raw_doppler_hz: f64,
    raw_cn0_dbhz: f64,
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

pub(super) fn raw_observation_snapshot(
    sat: &ObsSatellite,
    raw_pseudorange_m: f64,
) -> RawObservationSnapshot {
    RawObservationSnapshot {
        raw_pseudorange_m,
        raw_carrier_phase_cycles: sat.carrier_phase_cycles.0,
        raw_doppler_hz: sat.doppler_hz.0,
        raw_cn0_dbhz: sat.cn0_dbhz,
    }
}

pub(super) fn observation_residual_reports_from_epochs(
    epochs: &[ObsEpoch],
    raw_snapshots: &HashMap<String, RawObservationSnapshot>,
) -> Vec<ObservationResidualEpochReport> {
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

pub(super) fn observation_snapshot_key(epoch_idx: u64, signal_id: SigId) -> String {
    format!("{epoch_idx:010}:{}", observation_signal_key(signal_id))
}

pub(super) fn observation_signal_key(signal_id: SigId) -> String {
    format!(
        "{:?}:{:02}:{:?}:{:?}",
        signal_id.sat.constellation, signal_id.sat.prn, signal_id.band, signal_id.code
    )
}
