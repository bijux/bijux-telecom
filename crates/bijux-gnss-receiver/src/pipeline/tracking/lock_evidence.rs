fn coherent_integration_seconds(epoch_len_samples: usize, sample_rate_hz: f64) -> f64 {
    signal_coherent_integration_seconds(epoch_len_samples, sample_rate_hz)
}

fn tracking_lock_detector_thresholds(
    cn0_dbhz: f64,
    coherent_integration_s: f64,
    samples_per_chip: f64,
    tracking_params: TrackingParams,
    carrier_rate_hz_per_s: f64,
) -> LockDetectorThresholds {
    calibrated_lock_detector_thresholds(LockDetectorCalibrationInput {
        cn0_dbhz,
        coherent_integration_s,
        samples_per_chip,
        early_late_spacing_chips: tracking_params.early_late_spacing_chips,
        dll_false_unlock_probability: DLL_FALSE_UNLOCK_PROBABILITY,
        pll_false_unlock_probability: PLL_FALSE_UNLOCK_PROBABILITY,
        fll_false_unlock_probability: FLL_FALSE_UNLOCK_PROBABILITY,
        fll_bw_hz: tracking_params.fll_bw_hz,
        dynamic_stress_hz: carrier_rate_hz_per_s.abs() * coherent_integration_s.max(0.0),
    })
}

fn lock_detector_provenance(thresholds: LockDetectorThresholds) -> String {
    format!(
        " lock_detector=statistical lock_detector_coherent_snr={:.6} dll_lock_threshold={:.6} dll_hold_threshold={:.6} pll_lock_threshold_rad={:.6} pll_hold_threshold_rad={:.6} fll_lock_threshold_hz={:.6} fll_sigma_hz={:.6}",
        thresholds.distributions.coherent_snr_linear,
        thresholds.dll_lock,
        thresholds.dll_hold,
        thresholds.pll_lock_rad,
        thresholds.pll_hold_rad,
        thresholds.fll_lock_hz,
        thresholds.distributions.fll_sigma_hz,
    )
}

fn low_resolution_tracking_geometry(samples_per_chip: f64, early_late_spacing_chips: f64) -> bool {
    samples_per_chip * early_late_spacing_chips.abs()
        <= LOW_RESOLUTION_DLL_MIN_SAMPLE_SEPARATION + f64::EPSILON
}

fn low_resolution_code_lock(
    samples_per_chip: f64,
    early_late_spacing_chips: f64,
    prompt_lock: bool,
    pll_lock: bool,
    fll_lock: bool,
    cycle_slip: bool,
    anti_false_lock: bool,
) -> bool {
    low_resolution_tracking_geometry(samples_per_chip, early_late_spacing_chips)
        && prompt_lock
        && (pll_lock || fll_lock)
        && !cycle_slip
        && !anti_false_lock
}

fn low_resolution_false_lock_override(
    samples_per_chip: f64,
    early_late_spacing_chips: f64,
    prompt_lock: bool,
    prompt_power_supports_lock: bool,
    fll_lock: bool,
    doppler_consistent: bool,
    pll_err_rad: f32,
    cycle_slip: bool,
) -> bool {
    low_resolution_tracking_geometry(samples_per_chip, early_late_spacing_chips)
        && prompt_lock
        && prompt_power_supports_lock
        && fll_lock
        && doppler_consistent
        && pll_err_rad.abs() <= CARRIER_CONVERGENCE_MAX_PHASE_ERROR_RAD
        && !cycle_slip
}

fn short_fade_epoch_budget(tracking_params: TrackingParams) -> u16 {
    let integration_ms = tracking_params.integration_ms.max(1) as f64;
    ((SHORT_FADE_MAX_DURATION_S * 1000.0) / integration_ms).ceil() as u16
        + SHORT_FADE_RECOVERY_GRACE_EPOCHS
}

fn update_pull_in_stable_epochs(
    current_stable_epochs: u8,
    prompt_lock: bool,
    dll_lock: bool,
    pll_lock: bool,
    fll_lock: bool,
    cycle_slip: bool,
) -> u8 {
    if cycle_slip || !prompt_lock || !dll_lock || (!pll_lock && !fll_lock) {
        return 0;
    }
    current_stable_epochs.saturating_add(1)
}

fn update_prelock_cn0_refusal(
    current_state: ChannelState,
    weak_cn0_epochs: u8,
    cn0_dbhz: f64,
) -> (u8, bool, bool) {
    if matches!(current_state, ChannelState::Tracking | ChannelState::Degraded) {
        let cn0_supports_lock = cn0_dbhz.is_finite() && cn0_dbhz >= TRACKING_LOCK_MIN_CN0_DBHZ;
        return (0, cn0_supports_lock, false);
    }
    if !cn0_dbhz.is_finite() || cn0_dbhz >= TRACKING_LOCK_MIN_CN0_DBHZ {
        return (0, cn0_dbhz.is_finite(), false);
    }

    let next_weak_cn0_epochs = weak_cn0_epochs.saturating_add(1);
    (next_weak_cn0_epochs, false, next_weak_cn0_epochs >= TRACKING_LOCK_REFUSAL_EPOCHS)
}
