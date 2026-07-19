#[derive(Debug, Clone)]
struct LoopState {
    carrier_hz: f64,
    carrier_phase_cycles: f64,
    carrier_rate_hz_per_s: f64,
    code_rate_hz: f64,
    code_rate_reference_hz: f64,
    code_phase_samples: f64,
    tracking_adaptation_state: SignalTrackingAdaptationState,
    tracking_loop_profile: SignalTrackingLoopProfile,
    signal_delay_alignment: Option<SignalDelayAlignment>,
    subcarrier_code_phase_refined: bool,
    acquisition_cn0_proxy_dbhz: f64,
    lock_reference_cn0_dbhz: f64,
    prev_prompt: Option<Complex<f32>>,
    prev_prompt_phase_cycles: Option<f64>,
    secondary_code_prompt_history: VecDeque<SecondaryCodePromptSample>,
    secondary_code_sync: Option<SecondaryCodeSyncResult>,
    nav_bit_phase_offset_cycles: f64,
    nav_bit_transition_count: u32,
    pull_in_stable_epochs: u8,
    weak_cn0_epochs: u8,
    degraded_epochs: u16,
    prompt_power_reference: f32,
    prompt_cn0_window: VecDeque<f64>,
    code_error_window_samples: VecDeque<f64>,
    carrier_phase_error_window_cycles: VecDeque<f64>,
    doppler_error_window_hz: VecDeque<f64>,
    cn0_estimate_window_dbhz: VecDeque<f64>,
    unstable_discriminator_epochs: u8,
    state: ChannelState,
    unlocked_count: u8,
    lost_reason: Option<String>,
    reacquisition_candidate: Option<ReacquisitionSeed>,
    reacquisition_candidate_streak: u8,
    reacquisition_pending: bool,
    reacquisition_attempt_epochs: u8,
    reacquisition_stable_tracking_epochs: u8,
}

#[derive(Debug, Clone, Copy)]
struct CodePhaseStabilityDiagnostic {
    first_unstable_epoch_index: usize,
    max_abs_phase_step_samples: f64,
    phase_step_limit_samples: f64,
    catastrophic: bool,
}

#[derive(Debug, Clone, Copy)]
struct TrackingUncertaintyInputs {
    samples_per_chip: f64,
    dll_err: f32,
    pll_err_rad: f64,
    fll_err_hz: f64,
    cn0_dbhz: f64,
    cn0_reference_dbhz: f64,
    integration_ms: u32,
    channel_locked: bool,
    dll_locked: bool,
    anti_false_lock: bool,
    cycle_slip: bool,
    channel_state: ChannelState,
}

#[derive(Debug, Clone, Copy)]
struct PromptPhaseDecision {
    aligned_phase_cycles: f64,
    aligned_phase_delta_cycles: f64,
    nav_bit_phase_offset_cycles: f64,
    nav_bit_transition: bool,
    cycle_slip: bool,
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct DopplerEstimatorConsistency {
    loop_residual_hz: f64,
    phase_rate_residual_hz: f64,
    prompt_correlation_residual_hz: f64,
    spread_hz: f64,
    limit_hz: f64,
    consistent: bool,
}
