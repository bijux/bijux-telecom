use bijux_gnss_core::api::{ObsEpoch, ObsEpochManifest, Seconds};

use super::labels::{
    observation_status_label, observation_support_label, observation_uncertainty_label,
    pseudorange_model_has_resolved_alignment,
};
use super::receiver_clock::ObservationReceiverClock;
use super::variance::has_variance_evidence;

pub(super) fn stamp_observation_epoch_manifest(
    epoch: &mut ObsEpoch,
    receiver_clock: &ObservationReceiverClock,
) {
    let source_time = epoch.source_time;
    let artifact_id = format!("obs-epoch-{:010}", epoch.epoch_idx);
    epoch.t_rx_s = Seconds(source_time.receiver_time_s.0 + receiver_clock.bias_s);
    epoch.source_time = source_time;
    let epoch_key = bijux_gnss_core::api::obs_epoch_stability_key(epoch);
    epoch.manifest = Some(ObsEpochManifest {
        version: bijux_gnss_core::api::OBSERVATION_MODEL_VERSION,
        artifact_id: artifact_id.clone(),
        epoch_id: epoch_key,
        source_epoch_idx: epoch.epoch_idx,
        source_sample_index: source_time.sample_index,
        source_time,
        decision: epoch.decision,
        downstream_profile_version: bijux_gnss_core::api::OBSERVATION_DOWNSTREAM_PROFILE_VERSION,
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
}
