use bijux_gnss_core::api::{ObsDecisionArtifact, ObsEpoch, SatObservationDecision};

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
