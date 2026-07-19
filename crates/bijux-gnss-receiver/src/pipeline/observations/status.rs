use bijux_gnss_core::api::{
    ConventionsConfig, Meters, ObsEpoch, ObsSatellite, ObservationEpochDecision, ObservationStatus,
    TrackEpoch,
};
use bijux_gnss_signal::api::validate_obs_epochs;

const OBS_WEAK_CN0_DBHZ: f64 = 25.0;

pub(super) fn classify_observation_status(
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

pub(super) fn apply_epoch_decision(epoch: &mut ObsEpoch) {
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

pub(super) fn apply_pseudorange_physics_rejection(sat: &mut ObsSatellite) {
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

pub(super) fn push_observation_reject_reason(reasons: &mut Vec<String>, reason: &str) {
    if reasons.iter().any(|existing| existing == reason) {
        return;
    }
    reasons.push(reason.to_string());
    reasons.sort();
}
