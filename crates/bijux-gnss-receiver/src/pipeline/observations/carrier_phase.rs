use bijux_gnss_core::api::{
    CarrierPhaseArc, CycleSlipDecisionEvidence, CycleSlipDetector, CycleSlipDetectorEvidence,
    Cycles, SigId, TrackEpoch,
};

const BASE_CARRIER_PHASE_DISCONTINUITY_THRESHOLD_CYCLES: f64 = 0.25;
const BASE_CARRIER_PHASE_RESIDUAL_THRESHOLD_CYCLES: f64 = 0.15;
const BASE_DOPPLER_JUMP_THRESHOLD_HZ: f64 = 150.0;
pub(super) const CYCLE_SLIP_DETECTION_PROBABILITY_BUDGET: f64 = 0.99;
pub(super) const CYCLE_SLIP_FALSE_ALARM_PROBABILITY_BUDGET: f64 = 1.0e-3;

#[derive(Debug, Clone, Default)]
pub(super) struct CarrierPhaseArcState {
    phase_cycles: f64,
    doppler_hz: f64,
    last_sample_index: u64,
    arc_start_epoch_idx: u64,
    arc_start_sample_index: u64,
    arc_start_reason: String,
    pending_reset: Option<CarrierPhaseContinuity>,
    pending_reset_reason: Option<String>,
    initialized: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum CarrierPhaseContinuity {
    Unusable,
    ArcStart,
    Continuous,
    Coasted,
    ResetAfterCycleSlip,
    ResetAfterUnlock,
    ResetAfterReacquisition,
    ResetAfterDiscontinuity,
}

#[derive(Debug, Clone)]
pub(super) struct CarrierPhaseObservation {
    pub(super) phase_cycles: Cycles,
    pub(super) cycle_slip: bool,
    pub(super) cycle_slip_reason: Option<String>,
    pub(super) cycle_slip_evidence: CycleSlipDecisionEvidence,
    pub(super) continuity: CarrierPhaseContinuity,
    pub(super) arc_start_epoch_idx: u64,
    pub(super) arc_start_sample_index: u64,
    pub(super) arc: CarrierPhaseArc,
}

pub(super) fn carrier_phase_observation(
    epoch: &TrackEpoch,
    tracked_carrier_phase_cycles: f64,
    doppler_hz: f64,
    sample_rate_hz: f64,
    discontinuity: bool,
    signal_id: SigId,
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
        } else if carrier_phase_arc_can_coast(epoch, state) {
            return CarrierPhaseObservation {
                phase_cycles: Cycles(tracked_carrier_phase_cycles),
                cycle_slip: false,
                cycle_slip_reason: None,
                cycle_slip_evidence: cycle_slip_decision_evidence(vec![
                    tracking_lock_evidence(epoch, false, "tracking_lock_coasted"),
                    data_gap_evidence(false, None, None),
                ]),
                continuity: CarrierPhaseContinuity::Coasted,
                arc_start_epoch_idx: state.arc_start_epoch_idx,
                arc_start_sample_index: state.arc_start_sample_index,
                arc: CarrierPhaseArc::new(
                    signal_id,
                    state.arc_start_epoch_idx,
                    state.arc_start_sample_index,
                    state.arc_start_reason.clone(),
                ),
            };
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
        let tracking_reason =
            cycle_slip_reason.as_deref().unwrap_or("tracking_lock_unusable").to_string();
        let discontinuity_delta_s = discontinuity.then(|| {
            carrier_phase_delta_seconds(state.last_sample_index, epoch.sample_index, sample_rate_hz)
        });
        return CarrierPhaseObservation {
            phase_cycles: Cycles(tracked_carrier_phase_cycles),
            cycle_slip: cycle_slip_reason.is_some(),
            cycle_slip_reason,
            cycle_slip_evidence: cycle_slip_decision_evidence(vec![
                tracking_lock_evidence(
                    epoch,
                    tracking_reason != "tracking_lock_unusable",
                    tracking_reason.clone(),
                ),
                data_gap_evidence(discontinuity, discontinuity_delta_s, None),
            ]),
            continuity: CarrierPhaseContinuity::Unusable,
            arc_start_epoch_idx: 0,
            arc_start_sample_index: 0,
            arc: CarrierPhaseArc::invalid_boundary(signal_id, tracking_reason),
        };
    }

    let had_prior_phase_state = state.initialized;
    let previous_doppler_hz = state.doppler_hz;
    let delta_seconds =
        carrier_phase_delta_seconds(state.last_sample_index, epoch.sample_index, sample_rate_hz);
    let observed_phase_delta_cycles = tracked_carrier_phase_cycles - state.phase_cycles;
    let predicted_from_previous_doppler_cycles = previous_doppler_hz * delta_seconds;
    let predicted_from_trapezoid_cycles = 0.5 * (previous_doppler_hz + doppler_hz) * delta_seconds;
    let doppler_predicted_phase_residual_cycles =
        observed_phase_delta_cycles - predicted_from_previous_doppler_cycles;
    let phase_innovation_residual_cycles =
        observed_phase_delta_cycles - predicted_from_trapezoid_cycles;
    let phase_discontinuity_threshold_cycles = carrier_phase_discontinuity_threshold_cycles(epoch);
    let phase_residual_threshold_cycles = carrier_phase_residual_threshold_cycles(epoch);
    let doppler_jump_threshold_hz = doppler_jump_threshold_hz(epoch);
    let doppler_delta_hz = (doppler_hz - previous_doppler_hz).abs();
    let phase_discontinuity = had_prior_phase_state
        && doppler_predicted_phase_residual_cycles.abs() > phase_discontinuity_threshold_cycles;
    let doppler_jump = had_prior_phase_state && doppler_delta_hz > doppler_jump_threshold_hz;
    let phase_residual = had_prior_phase_state
        && phase_innovation_residual_cycles.abs() > phase_residual_threshold_cycles;
    let explicit_reset = if epoch.cycle_slip {
        Some((
            CarrierPhaseContinuity::ResetAfterCycleSlip,
            explicit_cycle_slip_reason(epoch, "cycle_slip"),
        ))
    } else if epoch.lock_state_reason.as_deref() == Some("reacquired")
        && state.pending_reset.is_some()
    {
        Some((CarrierPhaseContinuity::ResetAfterReacquisition, "reacquired".to_string()))
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
        state.pending_reset = None;
        state.pending_reset_reason = None;
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
                | CarrierPhaseContinuity::ResetAfterReacquisition
                | CarrierPhaseContinuity::ResetAfterDiscontinuity
        );

    if continuity != CarrierPhaseContinuity::Continuous {
        state.arc_start_epoch_idx = epoch.epoch.index;
        state.arc_start_sample_index = epoch.sample_index;
        state.arc_start_reason =
            carrier_phase_arc_start_reason(continuity, cycle_slip_reason.as_deref()).to_string();
    }
    state.phase_cycles = tracked_carrier_phase_cycles;
    state.doppler_hz = doppler_hz;
    state.last_sample_index = epoch.sample_index;
    state.initialized = true;

    CarrierPhaseObservation {
        phase_cycles: Cycles(tracked_carrier_phase_cycles),
        cycle_slip,
        cycle_slip_reason: cycle_slip_reason.clone(),
        cycle_slip_evidence: cycle_slip_decision_evidence(vec![
            tracking_lock_evidence(
                epoch,
                epoch.cycle_slip
                    || matches!(
                        continuity,
                        CarrierPhaseContinuity::ResetAfterUnlock
                            | CarrierPhaseContinuity::ResetAfterReacquisition
                    ),
                if epoch.cycle_slip
                    || matches!(continuity, CarrierPhaseContinuity::ResetAfterUnlock)
                {
                    cycle_slip_reason
                        .clone()
                        .unwrap_or_else(|| explicit_cycle_slip_reason(epoch, "cycle_slip"))
                } else if matches!(continuity, CarrierPhaseContinuity::ResetAfterReacquisition) {
                    cycle_slip_reason.clone().unwrap_or_else(|| "reacquired".to_string())
                } else {
                    "tracking_lock_continuous".to_string()
                },
            ),
            data_gap_evidence(discontinuity && had_prior_phase_state, Some(delta_seconds), None),
            doppler_predicted_phase_evidence(
                phase_discontinuity || doppler_jump,
                if doppler_jump {
                    doppler_delta_hz
                } else {
                    doppler_predicted_phase_residual_cycles.abs()
                },
                if doppler_jump {
                    doppler_jump_threshold_hz
                } else {
                    phase_discontinuity_threshold_cycles
                },
                if doppler_jump { "hz" } else { "cycles" },
                if doppler_jump { "doppler_jump" } else { "carrier_phase_discontinuity" },
            ),
            phase_innovation_evidence(
                phase_residual,
                phase_innovation_residual_cycles.abs(),
                phase_residual_threshold_cycles,
            ),
        ]),
        continuity,
        arc_start_epoch_idx: state.arc_start_epoch_idx,
        arc_start_sample_index: state.arc_start_sample_index,
        arc: CarrierPhaseArc::new(
            signal_id,
            state.arc_start_epoch_idx,
            state.arc_start_sample_index,
            state.arc_start_reason.clone(),
        ),
    }
}

fn carrier_phase_tracking_usable(epoch: &TrackEpoch) -> bool {
    epoch.lock
        && epoch.pll_lock
        && epoch.lock_state != "lost"
        && epoch.carrier_phase_cycles.0.is_finite()
}

fn carrier_phase_arc_can_coast(epoch: &TrackEpoch, state: &CarrierPhaseArcState) -> bool {
    state.initialized
        && !epoch.cycle_slip
        && epoch.carrier_phase_cycles.0.is_finite()
        && epoch.lock_state == "degraded"
        && matches!(
            epoch.lock_state_reason.as_deref(),
            Some("signal_fade") | Some("fade_recovered")
        )
}

fn carrier_phase_arc_start_reason(
    continuity: CarrierPhaseContinuity,
    cycle_slip_reason: Option<&str>,
) -> String {
    match continuity {
        CarrierPhaseContinuity::ArcStart => "arc_start".to_string(),
        CarrierPhaseContinuity::ResetAfterCycleSlip => {
            cycle_slip_reason.unwrap_or("cycle_slip").to_string()
        }
        CarrierPhaseContinuity::ResetAfterUnlock => "loss_of_lock".to_string(),
        CarrierPhaseContinuity::ResetAfterReacquisition => "reacquired".to_string(),
        CarrierPhaseContinuity::ResetAfterDiscontinuity => "sample_discontinuity".to_string(),
        CarrierPhaseContinuity::Unusable
        | CarrierPhaseContinuity::Continuous
        | CarrierPhaseContinuity::Coasted => "continuity_preserved".to_string(),
    }
}

fn explicit_cycle_slip_reason(epoch: &TrackEpoch, fallback: &str) -> String {
    epoch
        .cycle_slip_reason
        .clone()
        .or_else(|| epoch.lock_state_reason.clone())
        .unwrap_or_else(|| fallback.to_string())
}

pub(super) fn cycle_slip_decision_evidence(
    contributors: Vec<CycleSlipDetectorEvidence>,
) -> CycleSlipDecisionEvidence {
    CycleSlipDecisionEvidence::from_contributors(
        contributors,
        CYCLE_SLIP_DETECTION_PROBABILITY_BUDGET,
        CYCLE_SLIP_FALSE_ALARM_PROBABILITY_BUDGET,
    )
}

fn tracking_lock_evidence(
    epoch: &TrackEpoch,
    triggered: bool,
    reason: impl Into<String>,
) -> CycleSlipDetectorEvidence {
    let value = Some(if epoch.lock && epoch.pll_lock { 1.0 } else { 0.0 });
    CycleSlipDetectorEvidence::new(
        CycleSlipDetector::TrackingLock,
        triggered,
        value,
        Some(1.0),
        "lock_state",
        reason,
    )
}

fn data_gap_evidence(
    triggered: bool,
    delta_s: Option<f64>,
    threshold_s: Option<f64>,
) -> CycleSlipDetectorEvidence {
    CycleSlipDetectorEvidence::new(
        CycleSlipDetector::DataGap,
        triggered,
        delta_s,
        threshold_s,
        "s",
        if triggered { "sample_discontinuity" } else { "continuous_samples" },
    )
}

fn doppler_predicted_phase_evidence(
    triggered: bool,
    value: f64,
    threshold: f64,
    units: &'static str,
    reason: &'static str,
) -> CycleSlipDetectorEvidence {
    CycleSlipDetectorEvidence::new(
        CycleSlipDetector::DopplerPredictedPhase,
        triggered,
        Some(value),
        Some(threshold),
        units,
        reason,
    )
}

fn phase_innovation_evidence(
    triggered: bool,
    value_cycles: f64,
    threshold_cycles: f64,
) -> CycleSlipDetectorEvidence {
    CycleSlipDetectorEvidence::new(
        CycleSlipDetector::PhaseInnovation,
        triggered,
        Some(value_cycles),
        Some(threshold_cycles),
        "cycles",
        if triggered { "phase_residual" } else { "phase_innovation_nominal" },
    )
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
        CarrierPhaseContinuity::ResetAfterDiscontinuity
        | CarrierPhaseContinuity::ResetAfterReacquisition => 2,
        CarrierPhaseContinuity::ResetAfterUnlock => 1,
        CarrierPhaseContinuity::Unusable
        | CarrierPhaseContinuity::ArcStart
        | CarrierPhaseContinuity::Continuous
        | CarrierPhaseContinuity::Coasted => 0,
    }
}
