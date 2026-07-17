fn acq_to_track_state(hypothesis: &AcqHypothesis) -> &'static str {
    match hypothesis {
        AcqHypothesis::Accepted => "accepted",
        AcqHypothesis::Ambiguous => "degraded",
        AcqHypothesis::Rejected => "rejected",
        AcqHypothesis::Deferred => "deferred",
    }
}

fn tracking_channel_uid(sat: SatId, channel_id: u8) -> String {
    format!("{:?}-{:02}-ch{:02}", sat.constellation, sat.prn, channel_id)
}

fn tracking_channel_state_report(result: &TrackingResult) -> TrackingChannelStateReport {
    let sat = result.sat;
    let channel_id = result.epochs.first().and_then(|epoch| epoch.channel_id).unwrap_or_default();
    let channel_uid = result
        .epochs
        .first()
        .map(|epoch| epoch.channel_uid.clone())
        .unwrap_or_else(|| tracking_channel_uid(sat, channel_id));
    let initial_epoch_idx = result.epochs.first().map(|epoch| epoch.epoch.index).unwrap_or(0);
    let initial_sample_index = result.epochs.first().map(|epoch| epoch.sample_index).unwrap_or(0);
    let mut emitted_states = vec![TrackingChannelStateEvent {
        state: TrackingChannelState::Acquired,
        epoch_idx: initial_epoch_idx,
        sample_index: initial_sample_index,
        reason: None,
    }];
    let mut last_steady_state = Some(TrackingChannelState::Acquired);

    for epoch in &result.epochs {
        if let Some(steady_state) = tracking_channel_steady_state(epoch) {
            if last_steady_state != Some(steady_state) {
                emitted_states.push(TrackingChannelStateEvent {
                    state: steady_state,
                    epoch_idx: epoch.epoch.index,
                    sample_index: epoch.sample_index,
                    reason: epoch.lock_state_reason.clone(),
                });
                last_steady_state = Some(steady_state);
            }
        }

        if let Some(marker_state) =
            tracking_channel_marker_state(epoch.lock_state_reason.as_deref())
        {
            let duplicate_marker =
                emitted_states.last().is_some_and(|event| event.state == marker_state);
            if !duplicate_marker {
                emitted_states.push(TrackingChannelStateEvent {
                    state: marker_state,
                    epoch_idx: epoch.epoch.index,
                    sample_index: epoch.sample_index,
                    reason: epoch.lock_state_reason.clone(),
                });
            }
        }
    }

    let (final_state, final_reason) = result
        .epochs
        .last()
        .map(|epoch| {
            (
                tracking_channel_final_state(epoch).unwrap_or(TrackingChannelState::Acquired),
                epoch.lock_state_reason.clone(),
            )
        })
        .unwrap_or((TrackingChannelState::Acquired, None));

    TrackingChannelStateReport {
        sat,
        channel_id,
        channel_uid,
        final_state,
        final_reason,
        emitted_states,
    }
}

fn tracking_channel_steady_state(epoch: &TrackEpoch) -> Option<TrackingChannelState> {
    match epoch.lock_state.as_str() {
        "acquired" => Some(TrackingChannelState::Acquired),
        "pull_in" => Some(TrackingChannelState::PullIn),
        "tracking" => Some(TrackingChannelState::Locked),
        "degraded" => Some(TrackingChannelState::Degraded),
        "lost" => Some(TrackingChannelState::Lost),
        _ => None,
    }
}

fn tracking_channel_marker_state(reason: Option<&str>) -> Option<TrackingChannelState> {
    match reason {
        Some("reacquired") => Some(TrackingChannelState::Reacquired),
        Some("cn0_below_tracking_lock_floor") => Some(TrackingChannelState::Refused),
        _ => None,
    }
}

fn tracking_channel_final_state(epoch: &TrackEpoch) -> Option<TrackingChannelState> {
    if epoch.lock_state_reason.as_deref() == Some("cn0_below_tracking_lock_floor") {
        return Some(TrackingChannelState::Refused);
    }
    tracking_channel_steady_state(epoch)
}

fn acquisition_hypothesis_rank(hypothesis: AcqHypothesis) -> u8 {
    match hypothesis {
        AcqHypothesis::Accepted => 0,
        AcqHypothesis::Ambiguous => 1,
        AcqHypothesis::Rejected => 2,
        AcqHypothesis::Deferred => 3,
    }
}

fn compare_tracking_start_contexts(
    left: &TrackingStartContext,
    right: &TrackingStartContext,
) -> std::cmp::Ordering {
    left.acquisition_hypothesis_rank
        .cmp(&right.acquisition_hypothesis_rank)
        .then_with(|| right.acquisition_score.total_cmp(&left.acquisition_score))
        .then_with(|| right.acquisition_cn0_proxy_dbhz.total_cmp(&left.acquisition_cn0_proxy_dbhz))
        .then_with(|| left.seed.sat.constellation.cmp(&right.seed.sat.constellation))
        .then_with(|| left.seed.sat.prn.cmp(&right.seed.sat.prn))
}

fn epoch_lock_quality(
    lock: bool,
    pll_lock: bool,
    dll_lock: bool,
    fll_lock: bool,
    cn0_dbhz: f64,
) -> f64 {
    let mut quality = (cn0_dbhz / 60.0).clamp(0.0, 1.0);
    if !lock {
        quality *= 0.2;
    }
    if !pll_lock {
        quality *= 0.7;
    }
    if !dll_lock {
        quality *= 0.8;
    }
    if !fll_lock {
        quality *= 0.9;
    }
    quality
}

fn tracking_quality_class(channel_state: ChannelState) -> TrackingQualityClass {
    match channel_state {
        ChannelState::Tracking => TrackingQualityClass::Tracking,
        ChannelState::Degraded => TrackingQualityClass::Degraded,
        ChannelState::PullIn | ChannelState::Acquired => TrackingQualityClass::PullIn,
        ChannelState::Lost | ChannelState::Idle => TrackingQualityClass::Lost,
    }
}

fn refresh_prompt_power_reference(
    current_reference: f32,
    prompt_power: f32,
    state: ChannelState,
    anti_false_lock: bool,
) -> f32 {
    signal_refresh_prompt_power_reference(
        current_reference,
        prompt_power,
        tracking_quality_class(state),
        anti_false_lock,
    )
}

fn refresh_lock_reference_cn0_dbhz(
    current_reference: f64,
    cn0_dbhz: f64,
    reliable_tracking_lock: bool,
) -> f64 {
    signal_refresh_lock_reference_cn0_dbhz(current_reference, cn0_dbhz, reliable_tracking_lock)
}

fn prompt_power_ratio(prompt_power: f32, prompt_power_reference: f32) -> Option<f32> {
    signal_prompt_power_ratio(prompt_power, prompt_power_reference)
}

fn anti_false_lock_detected(early: Complex<f32>, prompt: Complex<f32>, late: Complex<f32>) -> bool {
    signal_anti_false_lock_detected(early, prompt, late)
}

fn should_apply_fll(state: ChannelState, raw_fll_lock: bool) -> bool {
    matches!(state, ChannelState::PullIn | ChannelState::Degraded) || !raw_fll_lock
}

fn apply_fll_during_epoch(
    fll_bw_hz: f64,
    state: ChannelState,
    raw_fll_lock: bool,
    doppler_consistency: DopplerEstimatorConsistency,
) -> bool {
    fll_bw_hz > 0.0
        && should_apply_fll(state, raw_fll_lock)
        && (doppler_consistency.consistent || state == ChannelState::PullIn)
}

fn doppler_estimator_consistency(
    loop_residual_hz: f64,
    phase_rate_residual_hz: f64,
    prompt_correlation_residual_hz: f64,
    fll_lock_limit_hz: f64,
) -> DopplerEstimatorConsistency {
    let estimates = [loop_residual_hz, phase_rate_residual_hz, prompt_correlation_residual_hz];
    let (min_estimate, max_estimate) = estimates.iter().fold(
        (f64::INFINITY, f64::NEG_INFINITY),
        |(min_estimate, max_estimate), estimate| {
            (min_estimate.min(*estimate), max_estimate.max(*estimate))
        },
    );
    let spread_hz = if estimates.iter().all(|estimate| estimate.is_finite()) {
        max_estimate - min_estimate
    } else {
        f64::INFINITY
    };
    let limit_hz = (fll_lock_limit_hz.abs() * DOPPLER_ESTIMATOR_SPREAD_LOCK_MULTIPLIER)
        .max(DOPPLER_ESTIMATOR_MIN_SPREAD_LIMIT_HZ);

    DopplerEstimatorConsistency {
        loop_residual_hz,
        phase_rate_residual_hz,
        prompt_correlation_residual_hz,
        spread_hz,
        limit_hz,
        consistent: spread_hz <= limit_hz,
    }
}

fn doppler_estimator_uncertainty_sample_hz(
    consistency: DopplerEstimatorConsistency,
    selected_residual_hz: f64,
) -> f64 {
    selected_residual_hz.abs().max(consistency.spread_hz)
}

fn doppler_estimator_provenance(consistency: DopplerEstimatorConsistency) -> String {
    let status = if consistency.consistent { "consistent" } else { "divergent" };
    format!(
        " doppler_estimator_consistency={} doppler_estimator_spread_hz={:.3} doppler_estimator_limit_hz={:.3} doppler_loop_residual_hz={:.3} doppler_phase_rate_residual_hz={:.3} doppler_prompt_residual_hz={:.3}",
        status,
        consistency.spread_hz,
        consistency.limit_hz,
        consistency.loop_residual_hz,
        consistency.phase_rate_residual_hz,
        consistency.prompt_correlation_residual_hz,
    )
}

fn tracking_provenance_segment(
    provenance: &str,
    first_token_prefix: &str,
    token_count: usize,
) -> Option<String> {
    let mut tokens = provenance
        .split_whitespace()
        .skip_while(|token| !token.starts_with(first_token_prefix))
        .take(token_count)
        .peekable();
    tokens.peek()?;
    Some(tokens.collect::<Vec<_>>().join(" "))
}

fn update_windowed_tracking_cn0_estimate(
    prompt_cn0_window: &mut VecDeque<f64>,
    epoch_cn0_dbhz: f64,
) -> Option<f64> {
    signal_update_windowed_tracking_cn0_estimate(
        prompt_cn0_window,
        epoch_cn0_dbhz,
        TRACKING_CN0_WINDOW_EPOCHS,
        TRACKING_CN0_MIN_WINDOW_EPOCHS,
    )
}

fn push_tracking_uncertainty_sample(window: &mut VecDeque<f64>, value: f64) {
    signal_push_tracking_uncertainty_sample(window, value, TRACKING_UNCERTAINTY_WINDOW_EPOCHS)
}

fn estimate_tracking_uncertainty(
    state: &LoopState,
    input: TrackingUncertaintyInputs,
) -> TrackingUncertainty {
    signal_estimate_tracking_uncertainty(
        &state.code_error_window_samples,
        &state.carrier_phase_error_window_cycles,
        &state.doppler_error_window_hz,
        &state.cn0_estimate_window_dbhz,
        SignalTrackingUncertaintyInputs {
            samples_per_chip: input.samples_per_chip,
            dll_err: input.dll_err,
            pll_err_rad: input.pll_err_rad,
            fll_err_hz: input.fll_err_hz,
            cn0_dbhz: input.cn0_dbhz,
            cn0_reference_dbhz: input.cn0_reference_dbhz,
            integration_ms: input.integration_ms,
            channel_locked: input.channel_locked,
            dll_locked: input.dll_locked,
            anti_false_lock: input.anti_false_lock,
            cycle_slip: input.cycle_slip,
            quality_class: tracking_quality_class(input.channel_state),
        },
    )
}

fn clear_tracking_uncertainty_windows(state: &mut LoopState) {
    state.prompt_cn0_window.clear();
    state.code_error_window_samples.clear();
    state.carrier_phase_error_window_cycles.clear();
    state.doppler_error_window_hz.clear();
    state.cn0_estimate_window_dbhz.clear();
}

fn update_discriminator_instability_epochs(
    current_epochs: u8,
    from_state: ChannelState,
    prompt_power_ratio: Option<f32>,
    raw_pll_lock: bool,
    raw_fll_lock: bool,
    cycle_slip: bool,
    anti_false_lock: bool,
) -> u8 {
    let strong_prompt = prompt_power_ratio
        .is_some_and(|ratio| ratio >= DISCRIMINATOR_INSTABILITY_MIN_PROMPT_POWER_RATIO);
    let unstable = matches!(from_state, ChannelState::Tracking | ChannelState::Degraded)
        && strong_prompt
        && !raw_pll_lock
        && !raw_fll_lock
        && !cycle_slip
        && !anti_false_lock;
    if unstable {
        current_epochs.saturating_add(1)
    } else {
        0
    }
}

fn classify_loss_of_lock_cause(
    from_state: ChannelState,
    cycle_slip: bool,
    prompt_power_ratio: Option<f32>,
    unstable_discriminator_epochs: u8,
) -> Option<LossOfLockCause> {
    let strong_prompt = prompt_power_ratio
        .is_some_and(|ratio| ratio >= DISCRIMINATOR_INSTABILITY_MIN_PROMPT_POWER_RATIO);
    if matches!(from_state, ChannelState::Tracking | ChannelState::Degraded)
        && cycle_slip
        && strong_prompt
    {
        return Some(LossOfLockCause::PhaseJump);
    }
    if matches!(from_state, ChannelState::Tracking | ChannelState::Degraded)
        && prompt_power_ratio.is_some_and(|ratio| ratio <= PROMPT_POWER_DROP_RATIO_THRESHOLD)
    {
        return Some(LossOfLockCause::PromptPowerDrop);
    }
    if unstable_discriminator_epochs >= DISCRIMINATOR_INSTABILITY_REQUIRED_EPOCHS {
        return Some(LossOfLockCause::DiscriminatorInstability);
    }
    None
}

fn is_sustained_lock_loss_reason(reason: &str) -> bool {
    matches!(
        reason,
        "lock_lost"
            | "prompt_power_drop"
            | "discriminator_instability"
            | "doppler_estimator_divergence"
            | "phase_jump"
            | "reacquisition_failed"
    )
}

#[derive(Debug, Clone)]
struct TransitionDecision {
    to_state: ChannelState,
    reason: String,
    next_unlocked_count: u8,
    next_degraded_epochs: u16,
}

#[derive(Debug, Clone, Copy)]
struct ChannelTransitionRequest<'a> {
    from_state: ChannelState,
    lock: bool,
    ready_for_tracking: bool,
    anti_false_lock: bool,
    loss_of_lock_cause: Option<LossOfLockCause>,
    unlocked_count: u8,
    degraded_epochs: u16,
    short_fade_epoch_budget: u16,
    short_fade_relock_evidence: bool,
    degraded_tracking_reason: Option<&'a str>,
}

fn deterministic_transition_rule(request: ChannelTransitionRequest<'_>) -> TransitionDecision {
    let ChannelTransitionRequest {
        from_state,
        lock,
        ready_for_tracking,
        anti_false_lock,
        loss_of_lock_cause,
        unlocked_count,
        degraded_epochs,
        short_fade_epoch_budget,
        short_fade_relock_evidence,
        degraded_tracking_reason,
    } = request;
    if loss_of_lock_cause == Some(LossOfLockCause::PhaseJump) {
        return TransitionDecision {
            to_state: ChannelState::Lost,
            reason: LossOfLockCause::PhaseJump.reason().to_string(),
            next_unlocked_count: unlocked_count.saturating_add(1),
            next_degraded_epochs: 0,
        };
    }
    if loss_of_lock_cause == Some(LossOfLockCause::DiscriminatorInstability)
        && (from_state != ChannelState::Degraded
            || degraded_epochs < DEGRADED_FADE_INSTABILITY_GRACE_EPOCHS)
    {
        return TransitionDecision {
            to_state: ChannelState::Lost,
            reason: LossOfLockCause::DiscriminatorInstability.reason().to_string(),
            next_unlocked_count: unlocked_count.saturating_add(1),
            next_degraded_epochs: 0,
        };
    }
    if from_state == ChannelState::Lost {
        return TransitionDecision {
            to_state: ChannelState::Lost,
            reason: "lost".to_string(),
            next_unlocked_count: unlocked_count,
            next_degraded_epochs: 0,
        };
    }
    if from_state == ChannelState::Degraded {
        if ready_for_tracking {
            return TransitionDecision {
                to_state: ChannelState::Tracking,
                reason: "fade_recovered".to_string(),
                next_unlocked_count: 0,
                next_degraded_epochs: 0,
            };
        }
        let next_degraded_epochs = degraded_epochs.saturating_add(1);
        let effective_short_fade_budget =
            short_fade_epoch_budget.saturating_add(if short_fade_relock_evidence {
                SHORT_FADE_RELOCK_EVIDENCE_GRACE_EPOCHS
            } else {
                0
            });
        if next_degraded_epochs > effective_short_fade_budget {
            let loss_reason = degraded_tracking_reason.map(str::to_string).unwrap_or_else(|| {
                loss_of_lock_cause.unwrap_or(LossOfLockCause::PromptPowerDrop).reason().to_string()
            });
            return TransitionDecision {
                to_state: ChannelState::Lost,
                reason: loss_reason,
                next_unlocked_count: 1,
                next_degraded_epochs: 0,
            };
        }
        return TransitionDecision {
            to_state: ChannelState::Degraded,
            reason: degraded_tracking_reason.unwrap_or("signal_fade").to_string(),
            next_unlocked_count: 0,
            next_degraded_epochs,
        };
    }
    if from_state == ChannelState::Tracking && !ready_for_tracking {
        return TransitionDecision {
            to_state: ChannelState::Degraded,
            reason: degraded_tracking_reason.unwrap_or("signal_fade").to_string(),
            next_unlocked_count: 0,
            next_degraded_epochs: 1,
        };
    }
    if ready_for_tracking {
        return TransitionDecision {
            to_state: ChannelState::Tracking,
            reason: "carrier_converged".to_string(),
            next_unlocked_count: 0,
            next_degraded_epochs: 0,
        };
    }
    if lock {
        return TransitionDecision {
            to_state: ChannelState::PullIn,
            reason: "carrier_pull_in".to_string(),
            next_unlocked_count: 0,
            next_degraded_epochs: 0,
        };
    }
    if anti_false_lock {
        return TransitionDecision {
            to_state: ChannelState::PullIn,
            reason: "anti_false_lock".to_string(),
            next_unlocked_count: unlocked_count.saturating_add(1),
            next_degraded_epochs: 0,
        };
    }
    let next_unlocked = unlocked_count.saturating_add(1);
    if from_state == ChannelState::Tracking && next_unlocked >= 2 {
        return TransitionDecision {
            to_state: ChannelState::Lost,
            reason: "lock_lost".to_string(),
            next_unlocked_count: next_unlocked,
            next_degraded_epochs: 0,
        };
    }
    TransitionDecision {
        to_state: ChannelState::PullIn,
        reason: "carrier_pull_in".to_string(),
        next_unlocked_count: next_unlocked,
        next_degraded_epochs: 0,
    }
}

fn sustained_lock_loss_reacquire_seed(epochs: &[TrackEpoch]) -> Option<SustainedLockLossSeed> {
    let mut consecutive_lost_epochs = 0usize;
    let mut stable_tracking_seed = None;
    for epoch in epochs {
        if epoch.lock && epoch.lock_state == ChannelState::Tracking.to_string() {
            stable_tracking_seed = Some(SustainedLockLossSeed {
                carrier_hz: epoch.carrier_hz.0,
                code_phase_samples: epoch.code_phase_samples.0,
                code_rate_hz: epoch.code_rate_hz.0,
                sample_index: epoch.sample_index,
            });
        }
        if !epoch.lock
            && epoch.lock_state == "lost"
            && epoch.lock_state_reason.as_deref().is_some_and(is_sustained_lock_loss_reason)
        {
            consecutive_lost_epochs += 1;
            if consecutive_lost_epochs >= REACQUISITION_REQUIRED_LOST_EPOCHS {
                return stable_tracking_seed.or(Some(SustainedLockLossSeed {
                    carrier_hz: epoch.carrier_hz.0,
                    code_phase_samples: epoch.code_phase_samples.0,
                    code_rate_hz: epoch.code_rate_hz.0,
                    sample_index: epoch.sample_index,
                }));
            }
            continue;
        }
        consecutive_lost_epochs = 0;
    }
    None
}

fn project_reacquisition_code_phase_samples(
    seed: SustainedLockLossSeed,
    target_sample_index: u64,
    nominal_code_rate_hz: f64,
    samples_per_code: usize,
) -> f64 {
    let elapsed_samples = target_sample_index.saturating_sub(seed.sample_index) as f64;
    let projected_code_phase_samples =
        seed.code_phase_samples + elapsed_samples * (seed.code_rate_hz / nominal_code_rate_hz);
    wrap_code_phase_samples(projected_code_phase_samples, samples_per_code)
}
