/// Per-epoch tracking truth-table row for synthetic validation.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticTrackingTruthTableEpoch {
    /// Zero-based epoch index inside the satellite track.
    pub epoch_index: usize,
    /// Absolute sample index at the start of the epoch.
    pub sample_index: u64,
    /// Expected carrier frequency in Hz at this epoch.
    pub expected_carrier_hz: f64,
    /// Measured carrier frequency in Hz at this epoch.
    pub measured_carrier_hz: f64,
    /// Absolute carrier-frequency error in Hz.
    pub carrier_error_hz: f64,
    /// Expected Doppler in Hz at this epoch.
    pub expected_doppler_hz: f64,
    /// Measured Doppler in Hz at this epoch.
    pub measured_doppler_hz: f64,
    /// Absolute Doppler error in Hz.
    pub doppler_error_hz: f64,
    /// PLL discriminator residual in radians for this tracking epoch.
    #[serde(default)]
    pub pll_phase_error_rad: f64,
    /// Absolute PLL discriminator residual in cycles for this tracking epoch.
    #[serde(default)]
    pub pll_phase_error_cycles: f64,
    /// Expected code phase in samples at this epoch.
    pub expected_code_phase_samples: f64,
    /// Measured code phase in samples at this epoch.
    pub measured_code_phase_samples: f64,
    /// Wrapped absolute code-phase error in samples.
    pub code_phase_error_samples: f64,
    /// Expected C/N0 in dB-Hz at this epoch.
    pub expected_cn0_db_hz: f64,
    /// Measured C/N0 in dB-Hz at this epoch.
    pub measured_cn0_dbhz: f64,
    /// Absolute C/N0 error in dB-Hz.
    pub cn0_error_db: f64,
    /// Whether the prompt detector reported signal lock.
    pub lock: bool,
    /// Whether the PLL reported lock.
    pub pll_lock: bool,
    /// Whether the DLL reported lock.
    pub dll_lock: bool,
    /// Whether the FLL reported lock.
    pub fll_lock: bool,
    /// Whether the receiver reported a cycle slip at this epoch.
    pub cycle_slip: bool,
    /// Receiver lock-state label for this epoch.
    pub lock_state: String,
    /// Receiver lock-state reason for this epoch, when one was emitted.
    pub lock_state_reason: Option<String>,
    /// Whether this epoch belongs to a stable tracking window.
    pub stable_tracking_epoch: bool,
    /// Whether every tracked error stayed within tolerance for this stable epoch.
    pub pass: bool,
}

/// Per-satellite tracking truth table for synthetic validation.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticTrackingTruthTableSatellite {
    /// Satellite identifier.
    pub sat: SatId,
    /// Explicit signal band validated for this satellite.
    #[serde(default = "default_signal_band")]
    pub signal_band: SignalBand,
    /// Explicit signal code validated for this satellite.
    #[serde(default = "default_signal_code")]
    pub signal_code: SignalCode,
    /// GLONASS FDMA channel when the validated signal uses GLONASS L1.
    #[serde(default)]
    pub glonass_frequency_channel: Option<GlonassFrequencyChannel>,
    /// Injected Doppler shift in Hz.
    pub injected_doppler_hz: f64,
    /// Expected measured Doppler after the receiver clock bias is applied, in Hz.
    pub expected_measured_doppler_hz: f64,
    /// Injected code phase at sample zero, in chips.
    pub injected_code_phase_chips: f64,
    /// Injected carrier-to-noise density ratio in dB-Hz.
    pub injected_cn0_db_hz: f32,
    /// Count of recorded epochs for this satellite.
    pub epoch_count: usize,
    /// Count of epochs inside stable tracking windows.
    pub stable_epoch_count: usize,
    /// First stable tracking epoch index, when one exists.
    pub first_stable_epoch_index: Option<usize>,
    /// Whether every stable tracking epoch stayed within tolerance.
    pub pass: bool,
    /// Per-epoch truth-table rows.
    pub epochs: Vec<SyntheticTrackingTruthTableEpoch>,
}

/// Truth-guided tracking truth table for a synthetic capture.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticTrackingTruthTableReport {
    /// Stable scenario identifier for this capture.
    pub scenario_id: String,
    /// Allowed carrier-frequency error in Hz.
    pub carrier_tolerance_hz: f64,
    /// Allowed Doppler error in Hz.
    pub doppler_tolerance_hz: f64,
    /// Allowed wrapped absolute code-phase error in samples.
    pub code_phase_tolerance_samples: f64,
    /// Allowed absolute C/N0 error in dB-Hz.
    pub cn0_tolerance_db_hz: f64,
    /// Capture sample rate in Hz.
    pub sample_rate_hz: f64,
    /// Number of samples in one code period at the configured rate.
    pub period_samples: usize,
    /// Output scale applied before quantization.
    pub output_scale_applied: f32,
    /// Whether every measured satellite passed the requested tolerances.
    pub pass: bool,
    /// Per-satellite truth-table rows.
    pub satellites: Vec<SyntheticTrackingTruthTableSatellite>,
}

/// Stable identity for a tracking-supported signal profile.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq, PartialOrd, Ord)]
pub struct SyntheticTrackingSignalIdentity {
    /// Constellation that owns the signal.
    pub constellation: Constellation,
    /// Signal band being characterized.
    pub signal_band: SignalBand,
    /// Signal code being characterized.
    pub signal_code: SignalCode,
    /// GLONASS FDMA channel when the signal identity requires one.
    pub glonass_frequency_channel: Option<GlonassFrequencyChannel>,
}

/// Empirical distribution summary for one tracking-noise measurement.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticTrackingNoiseDistribution {
    /// Number of finite samples used in this distribution.
    pub sample_count: usize,
    /// Signed arithmetic mean of the samples.
    pub mean: f64,
    /// Root-mean-square sample value.
    pub rms: f64,
    /// Population standard deviation of the samples.
    pub standard_deviation: f64,
    /// Median absolute sample value.
    pub p50_abs: f64,
    /// Ninety-fifth percentile absolute sample value.
    pub p95_abs: f64,
    /// Maximum absolute sample value.
    pub max_abs: f64,
}

/// Empirical tracking-noise profile for one supported signal identity.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticTrackingNoiseProfile {
    /// Signal identity covered by this empirical profile.
    pub signal: SyntheticTrackingSignalIdentity,
    /// Number of satellites contributing stable epochs.
    pub satellite_count: usize,
    /// Number of stable tracking epochs used in every distribution.
    pub stable_epoch_count: usize,
    /// Minimum truth C/N0 covered by this profile, in dB-Hz.
    pub min_cn0_db_hz: f64,
    /// Maximum truth C/N0 covered by this profile, in dB-Hz.
    pub max_cn0_db_hz: f64,
    /// Minimum absolute truth Doppler covered by this profile, in Hz.
    pub min_abs_doppler_hz: f64,
    /// Maximum absolute truth Doppler covered by this profile, in Hz.
    pub max_abs_doppler_hz: f64,
    /// Empirical DLL code-jitter distribution, in samples.
    pub dll_jitter_samples: SyntheticTrackingNoiseDistribution,
    /// Empirical PLL discriminator phase-error distribution, in cycles.
    pub pll_phase_error_cycles: SyntheticTrackingNoiseDistribution,
    /// Empirical tracking Doppler-error distribution, in Hz.
    pub doppler_error_hz: SyntheticTrackingNoiseDistribution,
    /// Empirical C/N0 bias distribution, in dB-Hz.
    pub cn0_bias_db_hz: SyntheticTrackingNoiseDistribution,
    /// Number of stable epochs that reported cycle slips.
    pub cycle_slip_count: usize,
    /// Empirical cycle-slip probability across stable epochs.
    pub cycle_slip_probability: f64,
    /// Whether this profile has enough stable samples for a defensible distribution.
    pub pass: bool,
}

/// Empirical tracking-noise characterization across supported receiver signals.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticTrackingNoiseReport {
    /// Scenario identifiers that contributed input tracking truth tables.
    pub scenario_ids: Vec<String>,
    /// Minimum stable samples required for each supported signal profile.
    pub required_stable_epoch_count: usize,
    /// Number of signal identities included in the empirical noise matrix.
    pub supported_signal_count: usize,
    /// Tracking-supported signals that lack executable synthetic acquisition support.
    #[serde(default)]
    pub tracking_only_signal_count: usize,
    /// Tracking-supported signals excluded because acquisition is not executable.
    #[serde(default)]
    pub tracking_only_signals: Vec<SyntheticTrackingSignalIdentity>,
    /// Tracking-supported capture signals that do not produce stable truth rows for noise profiles.
    #[serde(default)]
    pub unstable_tracking_truth_signal_count: usize,
    /// Capture-supported signals excluded because stable tracking truth is unavailable.
    #[serde(default)]
    pub unstable_tracking_truth_signals: Vec<SyntheticTrackingSignalIdentity>,
    /// Number of supported signal identities with empirical profiles.
    pub characterized_signal_count: usize,
    /// Tracking-supported signal identities not covered by the input reports.
    pub missing_signals: Vec<SyntheticTrackingSignalIdentity>,
    /// Signal identities whose empirical profile had too few stable samples.
    pub under_sampled_signals: Vec<SyntheticTrackingSignalIdentity>,
    /// Per-signal empirical profiles.
    pub profiles: Vec<SyntheticTrackingNoiseProfile>,
    /// Whether every supported signal had enough empirical tracking-noise samples.
    pub pass: bool,
}

/// Boundedness summary for one numerical tracking state family.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticTrackingNumericalStateSummary {
    /// Number of epochs inspected for this state family.
    pub epoch_count: usize,
    /// Number of inspected epochs that contained finite values.
    pub finite_epoch_count: usize,
    /// Minimum observed state value.
    pub min_value: f64,
    /// Maximum observed state value.
    pub max_value: f64,
    /// Maximum absolute adjacent state step after wrapping rules are applied.
    pub max_abs_step: f64,
    /// Whether every inspected value stayed finite and inside the configured bound.
    pub pass: bool,
}

/// Long-duration boundedness verdict for one tracked satellite signal.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticTrackingNumericalStabilitySatellite {
    /// Satellite identifier.
    pub sat: SatId,
    /// Explicit signal band validated for this satellite.
    pub signal_band: SignalBand,
    /// Explicit signal code validated for this satellite.
    pub signal_code: SignalCode,
    /// GLONASS FDMA channel when the validated signal uses GLONASS L1.
    #[serde(default)]
    pub glonass_frequency_channel: Option<GlonassFrequencyChannel>,
    /// Number of tracking epochs inspected for this satellite signal.
    pub epoch_count: usize,
    /// Whether timestamp sample indices and receiver sample traces stayed monotonic and finite.
    pub timestamps: SyntheticTrackingNumericalStateSummary,
    /// Whether receiver code phase stayed finite and inside one code period.
    pub code_phase: SyntheticTrackingNumericalStateSummary,
    /// Whether accumulated carrier phase stayed finite and adjacent steps stayed bounded.
    pub carrier_phase: SyntheticTrackingNumericalStateSummary,
    /// Whether NCO-driving loop state stayed finite and bounded.
    pub nco_state: SyntheticTrackingNumericalStateSummary,
    /// Whether secondary-code phase stayed finite and bounded when the signal reports one.
    pub secondary_code_phase: Option<SyntheticTrackingNumericalStateSummary>,
    /// Whether every state family passed for this satellite signal.
    pub pass: bool,
}

/// Long-duration numerical boundedness verdict for synthetic receiver tracking outputs.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticTrackingNumericalStabilityReport {
    /// Stable scenario identifier for this capture.
    pub scenario_id: String,
    /// Capture sample rate in Hz.
    pub sample_rate_hz: f64,
    /// Number of samples in one receiver code period at the configured rate.
    pub period_samples: usize,
    /// Expected input samples consumed by the long-duration stream.
    pub expected_input_samples: u64,
    /// Actual input samples consumed by receiver tracking.
    pub processed_input_samples: u64,
    /// Minimum tracking epochs required per satellite signal.
    pub required_epoch_count: usize,
    /// Number of tracked satellite signals included in the report.
    pub tracked_signal_count: usize,
    /// Per-satellite numerical boundedness verdicts.
    pub satellites: Vec<SyntheticTrackingNumericalStabilitySatellite>,
    /// Whether every inspected tracking output stayed finite and bounded.
    pub pass: bool,
}
