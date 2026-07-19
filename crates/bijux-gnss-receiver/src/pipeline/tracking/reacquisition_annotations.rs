fn stable_reacquisition_tracking_epoch(epoch: &TrackEpoch) -> bool {
    let doppler_uncertainty_bounded = epoch
        .tracking_uncertainty
        .as_ref()
        .is_some_and(|uncertainty| {
            uncertainty.doppler_hz.is_finite()
                && uncertainty.doppler_hz
                    <= REACQUISITION_STABLE_TRACKING_MAX_DOPPLER_UNCERTAINTY_HZ
        });
    epoch.lock
        && epoch.lock_state == ChannelState::Tracking.to_string()
        && epoch.lock_state_reason.as_deref() == Some("carrier_converged")
        && epoch.dll_lock
        && epoch.pll_lock
        && epoch.fll_lock
        && !epoch.cycle_slip
        && !epoch.anti_false_lock
        && doppler_uncertainty_bounded
}

fn apply_reacquisition_annotations(
    state: &mut LoopState,
    epochs: &mut [TrackEpoch],
    transitions: &mut [TrackTransition],
    reacquisition_outcome: ReacquisitionOutcome,
) {
    let Some(last_epoch) = epochs.last_mut() else {
        return;
    };

    if state.reacquisition_pending {
        let stable_tracking_epoch = stable_reacquisition_tracking_epoch(last_epoch);
        state.reacquisition_stable_tracking_epochs = if stable_tracking_epoch {
            state.reacquisition_stable_tracking_epochs.saturating_add(1)
        } else {
            0
        };
    }

    if state.reacquisition_pending
        && state.reacquisition_stable_tracking_epochs >= REACQUISITION_STABLE_TRACKING_EPOCHS
        && stable_reacquisition_tracking_epoch(last_epoch)
    {
        last_epoch.lock_state_reason = Some("reacquired".to_string());
        if let Some(last_transition) = transitions.last_mut() {
            if last_transition.to_state == ChannelState::Tracking.to_string()
                && last_transition.reason == "carrier_converged"
            {
                last_transition.reason = "reacquired".to_string();
            }
        }
        state.reacquisition_pending = false;
        state.reacquisition_attempt_epochs = 0;
        state.reacquisition_stable_tracking_epochs = 0;
        return;
    }

    if state.reacquisition_pending {
        state.reacquisition_attempt_epochs = state.reacquisition_attempt_epochs.saturating_add(1);
        if state.reacquisition_attempt_epochs >= REACQUISITION_PULL_IN_EPOCH_BUDGET {
            last_epoch.lock_state = ChannelState::Lost.to_string();
            last_epoch.lock_state_reason = Some("reacquisition_failed".to_string());
            if let Some(last_transition) = transitions.last_mut() {
                last_transition.to_state = ChannelState::Lost.to_string();
                last_transition.reason = "reacquisition_failed".to_string();
            }
            state.state = ChannelState::Lost;
            state.lost_reason = Some("reacquisition_failed".to_string());
            state.reacquisition_pending = false;
            state.reacquisition_attempt_epochs = 0;
            state.reacquisition_stable_tracking_epochs = 0;
            state.unlocked_count = 0;
            state.degraded_epochs = 0;
            return;
        }
    }

    if last_epoch.lock_state == ChannelState::Lost.to_string() {
        match reacquisition_outcome {
            ReacquisitionOutcome::Failed => {
                last_epoch.lock_state_reason = Some("reacquisition_failed".to_string());
            }
            ReacquisitionOutcome::SeedUnavailable | ReacquisitionOutcome::NotNeeded => {
                if let Some(lost_reason) = state.lost_reason.as_ref() {
                    last_epoch.lock_state_reason = Some(lost_reason.clone());
                }
            }
            ReacquisitionOutcome::Started => {}
        }
    }
}
