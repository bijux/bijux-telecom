/// Synthetic acquisition-uncertainty coverage input for one signal profile.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionUncertaintyCoverageCase {
    /// Signal configuration under test.
    pub signal: SyntheticSignalParams,
    /// Coherent integration length under test, in milliseconds.
    pub coherent_ms: u32,
    /// Noncoherent integration count under test.
    pub noncoherent: u32,
    /// Duration of each synthetic acquisition frame, in seconds.
    pub duration_s: f64,
    /// Optional linear Doppler-rate term applied during generation, in Hz/s.
    #[serde(default)]
    pub doppler_rate_hz_per_s: Option<f64>,
}

/// Per-trial acquisition uncertainty coverage row for a synthetic profile.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionUncertaintyCoverageTrial {
    /// Stable scenario identifier for this trial.
    pub scenario_id: String,
    /// Deterministic seed used for the synthetic noise realization.
    pub seed: u64,
    /// Hypothesis returned by the receiver.
    pub hypothesis: String,
    /// Measured Doppler in Hz relative to the configured IF.
    pub measured_doppler_hz: f64,
    /// Expected Doppler in Hz relative to the configured IF.
    pub expected_doppler_hz: f64,
    /// Absolute Doppler error in Hz.
    pub doppler_error_hz: f64,
    /// Receiver-reported Doppler one-sigma uncertainty in Hz, when available.
    pub reported_doppler_sigma_hz: Option<f64>,
    /// Whether the Doppler error stayed within one reported sigma.
    pub doppler_within_one_sigma: Option<bool>,
    /// Measured code phase in samples after refinement.
    pub measured_code_phase_samples: f64,
    /// Expected code phase in samples under the receiver search convention.
    pub expected_code_phase_samples: f64,
    /// Wrapped absolute code-phase error in samples.
    pub code_phase_error_samples: f64,
    /// Receiver-reported code-phase one-sigma uncertainty in samples, when available.
    pub reported_code_phase_sigma_samples: Option<f64>,
    /// Whether the code-phase error stayed within one reported sigma.
    pub code_phase_within_one_sigma: Option<bool>,
    /// Measured Doppler rate in Hz/s, when the acquisition search exposed one.
    pub measured_doppler_rate_hz_per_s: Option<f64>,
    /// Expected Doppler rate in Hz/s, when the synthetic profile injected one.
    pub expected_doppler_rate_hz_per_s: Option<f64>,
    /// Absolute Doppler-rate error in Hz/s, when both expected and measured values exist.
    pub doppler_rate_error_hz_per_s: Option<f64>,
    /// Receiver-reported Doppler-rate one-sigma uncertainty in Hz/s, when available.
    pub reported_doppler_rate_sigma_hz_per_s: Option<f64>,
    /// Whether the Doppler-rate error stayed within one reported sigma.
    pub doppler_rate_within_one_sigma: Option<bool>,
}

/// Aggregate acquisition uncertainty coverage summary for one synthetic profile.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionUncertaintyCoveragePoint {
    /// Signal profile used for this point.
    pub case: SyntheticAcquisitionUncertaintyCoverageCase,
    /// Number of deterministic trials measured for this point.
    pub trial_count: usize,
    /// Number of trials that produced accepted acquisitions with reported uncertainty.
    pub successful_trial_count: usize,
    /// Expected one-sigma containment rate under a Gaussian local model.
    pub expected_one_sigma_rate: f64,
    /// Count of successful trials whose Doppler error stayed within one reported sigma.
    pub doppler_within_one_sigma_count: usize,
    /// Fraction of successful trials whose Doppler error stayed within one reported sigma.
    pub doppler_within_one_sigma_rate: f64,
    /// Count of successful trials whose code-phase error stayed within one reported sigma.
    pub code_phase_within_one_sigma_count: usize,
    /// Fraction of successful trials whose code-phase error stayed within one reported sigma.
    pub code_phase_within_one_sigma_rate: f64,
    /// Count of successful trials whose Doppler-rate error stayed within one reported sigma.
    pub doppler_rate_within_one_sigma_count: Option<usize>,
    /// Fraction of successful trials whose Doppler-rate error stayed within one reported sigma.
    pub doppler_rate_within_one_sigma_rate: Option<f64>,
    /// Per-trial coverage rows.
    pub trials: Vec<SyntheticAcquisitionUncertaintyCoverageTrial>,
}

/// Acquisition uncertainty coverage report across multiple synthetic profiles.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionUncertaintyCoverageReport {
    /// Scenario identifier prefix shared across the measurement points.
    pub scenario_id_prefix: String,
    /// Effective acquisition Doppler bin width in Hz.
    pub doppler_step_hz: i32,
    /// Effective acquisition Doppler-rate bin width in Hz/s.
    pub doppler_rate_step_hz_per_s: i32,
    /// Measurement points captured in the report.
    pub points: Vec<SyntheticAcquisitionUncertaintyCoveragePoint>,
}
