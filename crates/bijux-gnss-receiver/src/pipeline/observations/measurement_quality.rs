use bijux_gnss_core::api::{
    CarrierPhaseArc, CodeCarrierDivergence, CycleSlipDecisionEvidence, LockFlags, ObsEpoch,
    ObsSatellite, ObservationEpochDecision, ObservationMeasurementCovariance, ObservationStatus,
    ReceiverSampleTrace, SigId,
};
use serde::{Deserialize, Serialize};

use super::cycle_slip_reason;
use super::timing::observation_epoch_id;
use super::variance::{evidence_sigma_from_variance, finite_sigma};

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
