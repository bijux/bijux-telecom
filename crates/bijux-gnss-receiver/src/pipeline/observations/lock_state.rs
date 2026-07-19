use bijux_gnss_core::api::{ObsSatellite, TrackEpoch};

use super::status::push_observation_reject_reason;

pub(super) fn cycle_slip_reason(sat: &ObsSatellite) -> Option<String> {
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

pub(super) fn tracking_lock_quality(epoch: &TrackEpoch, observation_cycle_slip: bool) -> f64 {
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

pub(super) fn observation_lock_state(
    epoch: &TrackEpoch,
    observation_cycle_slip: bool,
) -> &'static str {
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

pub(super) fn observation_lock_reason(
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

pub(super) fn apply_cycle_slip_surface(sat: &mut ObsSatellite, reason: Option<&str>) {
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
