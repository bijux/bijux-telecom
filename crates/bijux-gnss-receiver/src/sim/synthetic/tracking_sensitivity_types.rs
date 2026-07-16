/// Per-trial tracking outcome for a synthetic lock-sensitivity profile.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticTrackingSensitivityTrial {
    /// Stable scenario identifier for this trial.
    pub scenario_id: String,
    /// Deterministic seed used for the synthetic noise realization.
    pub seed: u64,
    /// Satellite identifier under test.
    pub sat: SatId,
    /// Whether the receiver reached a sustained stable tracking window.
    pub stable_lock: bool,
    /// Whether the receiver explicitly refused to declare lock under weak signal conditions.
    pub refused_lock: bool,
    /// First epoch index of the sustained stable tracking window, when one exists.
    pub first_lock_epoch_index: Option<usize>,
    /// Count of epochs in the sustained stable tracking window.
    pub locked_epoch_count: usize,
    /// Final per-epoch tracking state emitted by the receiver.
    pub final_lock_state: String,
    /// Final per-epoch tracking state reason emitted by the receiver.
    pub final_lock_state_reason: Option<String>,
}

/// Lock-probability summary for one synthetic tracking profile.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticTrackingSensitivityReport {
    /// Scenario identifier prefix shared across the trials.
    pub scenario_id_prefix: String,
    /// Satellite identifier under test.
    pub sat: SatId,
    /// Injected carrier-to-noise density ratio in dB-Hz.
    pub cn0_db_hz: f32,
    /// Duration of each synthetic tracking trial in seconds.
    pub duration_s: f64,
    /// Seeded Doppler error relative to truth, in Hz.
    pub seeded_doppler_error_hz: f64,
    /// Seeded code-phase error relative to truth, in samples.
    pub seeded_code_phase_error_samples: isize,
    /// Minimum sustained locked-epoch count required to declare stable lock.
    pub min_locked_epochs: usize,
    /// Number of synthetic trials measured for this profile.
    pub trial_count: usize,
    /// Number of trials that reached a sustained stable lock window.
    pub stable_lock_count: usize,
    /// Number of trials that explicitly refused lock under weak-signal conditions.
    pub refused_lock_count: usize,
    /// Stable-lock probability across the measured trials.
    pub lock_probability: f64,
    /// Mean count of epochs inside the sustained lock window.
    pub mean_locked_epochs: f64,
    /// Per-trial tracking rows.
    pub trials: Vec<SyntheticTrackingSensitivityTrial>,
}

/// Tracking lock-rate input for one synthetic measurement point.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticTrackingLockRateCase {
    /// Signal configuration under test.
    pub signal: SyntheticSignalParams,
    /// Duration of each synthetic tracking trial in seconds.
    pub duration_s: f64,
    /// Seeded Doppler error relative to truth, in Hz.
    pub seeded_doppler_error_hz: f64,
    /// Seeded code-phase error relative to truth, in samples.
    pub seeded_code_phase_error_samples: isize,
    /// Minimum sustained locked-epoch count required to declare stable lock.
    pub min_locked_epochs: usize,
}

/// Lock-rate summary for one synthetic tracking measurement point.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticTrackingLockRatePoint {
    /// Satellite identifier under test.
    pub sat: SatId,
    /// Injected carrier-to-noise density ratio in dB-Hz.
    pub cn0_db_hz: f32,
    /// Duration of each synthetic tracking trial in seconds.
    pub duration_s: f64,
    /// Seeded Doppler error relative to truth, in Hz.
    pub seeded_doppler_error_hz: f64,
    /// Seeded code-phase error relative to truth, in samples.
    pub seeded_code_phase_error_samples: isize,
    /// Minimum sustained locked-epoch count required to declare stable lock.
    pub min_locked_epochs: usize,
    /// Number of synthetic trials measured for this point.
    pub trial_count: usize,
    /// Number of trials that reached a sustained stable lock window.
    pub stable_lock_count: usize,
    /// Number of trials that explicitly refused lock under weak-signal conditions.
    pub refused_lock_count: usize,
    /// Stable-lock probability across the measured trials.
    pub lock_probability: f64,
    /// Mean count of epochs inside the sustained lock window.
    pub mean_locked_epochs: f64,
}

/// Lock-rate report across multiple C/N0 tracking measurement points.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticTrackingLockRateReport {
    /// Scenario identifier prefix shared across the measurement points.
    pub scenario_id_prefix: String,
    /// Measurement points captured in the report.
    pub points: Vec<SyntheticTrackingLockRatePoint>,
}

/// Lock-detector calibration input for one synthetic operating point.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct SyntheticTrackingLockDetectorCalibrationCase {
    /// Injected carrier-to-noise density ratio in dB-Hz.
    pub cn0_db_hz: f64,
    /// Coherent integration interval in milliseconds.
    pub coherent_ms: u32,
    /// Early-late correlator spacing in chips.
    pub early_late_spacing_chips: f64,
    /// Residual dynamic stress applied to the FLL detector in Hz.
    pub dynamic_stress_hz: f64,
    /// Half-width of the unlocked FLL search region in Hz.
    pub unlocked_fll_half_width_hz: f64,
    /// Unlocked discriminator bias expressed in calibrated sigma units.
    pub missed_unlock_bias_sigma: f64,
}

/// Calibrated lock-detector probabilities for one synthetic operating point.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticTrackingLockDetectorCalibrationPoint {
    /// Injected carrier-to-noise density ratio in dB-Hz.
    pub cn0_db_hz: f64,
    /// Coherent integration interval in milliseconds.
    pub coherent_ms: u32,
    /// Early-late correlator spacing in chips.
    pub early_late_spacing_chips: f64,
    /// Residual dynamic stress applied to the FLL detector in Hz.
    pub dynamic_stress_hz: f64,
    /// Coherent signal-to-noise ratio used by the detector model.
    pub coherent_snr_linear: f64,
    /// DLL normalized-envelope lock threshold.
    pub dll_lock_threshold: f64,
    /// PLL phase-error lock threshold in radians.
    pub pll_lock_threshold_rad: f64,
    /// FLL frequency-error lock threshold in Hz.
    pub fll_lock_threshold_hz: f64,
    /// Probability that any locked detector unlocks falsely at this point.
    pub false_unlock_probability: f64,
    /// Probability that all unlocked detector gates accept noise at this point.
    pub false_lock_probability: f64,
    /// Probability that all biased unlocked detector gates still accept at this point.
    pub missed_unlock_probability: f64,
    /// Locked DLL false-unlock probability.
    pub dll_false_unlock_probability: f64,
    /// Locked PLL false-unlock probability.
    pub pll_false_unlock_probability: f64,
    /// Locked FLL false-unlock probability.
    pub fll_false_unlock_probability: f64,
    /// Unlocked DLL false-lock probability.
    pub dll_false_lock_probability: f64,
    /// Unlocked PLL false-lock probability.
    pub pll_false_lock_probability: f64,
    /// Unlocked FLL false-lock probability.
    pub fll_false_lock_probability: f64,
}

/// Lock-detector calibration report across C/N0 and dynamics operating points.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticTrackingLockDetectorCalibrationReport {
    /// Scenario identifier prefix shared across the measurement points.
    pub scenario_id_prefix: String,
    /// Target locked DLL false-unlock probability used to calibrate thresholds.
    pub dll_false_unlock_target: f64,
    /// Target locked PLL false-unlock probability used to calibrate thresholds.
    pub pll_false_unlock_target: f64,
    /// Target locked FLL false-unlock probability used to calibrate thresholds.
    pub fll_false_unlock_target: f64,
    /// Number of operating points captured in the report.
    pub point_count: usize,
    /// Whether every reported point has finite thresholds and probabilities.
    pub pass: bool,
    /// Measurement points captured in the report.
    pub points: Vec<SyntheticTrackingLockDetectorCalibrationPoint>,
}
