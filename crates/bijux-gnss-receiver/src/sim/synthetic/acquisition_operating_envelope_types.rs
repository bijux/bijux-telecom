/// Acquisition integration settings for one operating-envelope point.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
pub struct SyntheticAcquisitionIntegrationProfile {
    /// Coherent integration length under test, in milliseconds.
    pub coherent_ms: u32,
    /// Noncoherent integration count under test.
    pub noncoherent: u32,
}

/// Independent acquisition sensitivity axis represented by one operating-envelope point.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
pub enum SyntheticAcquisitionOperatingEnvelopeAxis {
    /// Sweep C/N0 while holding the remaining parameters fixed.
    Cn0DbHz,
    /// Sweep coherent and noncoherent integration settings.
    IntegrationProfile,
    /// Sweep line-of-sight Doppler relative to the acquisition grid.
    DopplerHz,
    /// Sweep receiver clock frequency bias in the synthetic truth model.
    ReceiverClockFrequencyBiasHz,
    /// Sweep primary-code phase within the code period.
    CodePhaseChips,
}

/// One signal-specific operating-envelope measurement plan.
#[derive(Debug, Clone)]
pub struct SyntheticAcquisitionOperatingEnvelopeSignalCase {
    /// Receiver configuration used for this signal family.
    pub config: ReceiverPipelineConfig,
    /// Baseline signal parameters held constant except on the swept axis.
    pub signal: SyntheticSignalParams,
    /// Integration settings to sweep.
    pub integration_profiles: Vec<SyntheticAcquisitionIntegrationProfile>,
    /// C/N0 values to sweep in dB-Hz.
    pub cn0_db_hz_points: Vec<f32>,
    /// Doppler values to sweep in Hz.
    pub doppler_hz_points: Vec<f64>,
    /// Receiver clock frequency biases to sweep in Hz.
    pub receiver_clock_frequency_bias_hz_points: Vec<f64>,
    /// Code phases to sweep in chips.
    pub code_phase_chips_points: Vec<f64>,
}

/// One operating point inside a signal-specific acquisition envelope.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionOperatingEnvelopePoint {
    /// Swept axis represented by this point.
    pub axis: SyntheticAcquisitionOperatingEnvelopeAxis,
    /// Satellite identifier under test.
    pub sat: SatId,
    /// Signal band under test.
    pub signal_band: SignalBand,
    /// Signal code under test.
    pub signal_code: SignalCode,
    /// GLONASS frequency channel when required by the signal identity.
    pub glonass_frequency_channel: Option<GlonassFrequencyChannel>,
    /// Injected carrier-to-noise density ratio in dB-Hz.
    pub cn0_db_hz: f32,
    /// Coherent integration length under test, in milliseconds.
    pub coherent_ms: u32,
    /// Noncoherent integration count under test.
    pub noncoherent: u32,
    /// Injected line-of-sight Doppler in Hz.
    pub doppler_hz: f64,
    /// Injected receiver clock frequency bias in Hz.
    pub receiver_clock_frequency_bias_hz: f64,
    /// Injected primary-code phase in chips.
    pub code_phase_chips: f64,
    /// Number of target-present trials measured for this point.
    pub trial_count: usize,
    /// Number of target-present trials that produced accepted results.
    pub accepted_count: usize,
    /// Number of target-present trials that produced non-rejected results within truth tolerances.
    pub detected_count: usize,
    /// Number of noise-only trials used for the matched false-alarm estimate.
    pub false_alarm_trial_count: usize,
    /// Number of noise-only trials that produced accepted results.
    pub false_alarm_count: usize,
    /// Accepted-trial probability across the target-present trials.
    pub acceptance_probability: f64,
    /// Detection probability across the target-present trials.
    pub detection_probability: f64,
    /// False-alarm probability across the matched noise-only trials.
    pub false_alarm_rate: f64,
    /// Mean peak-to-mean ratio across the target-present trials.
    pub mean_peak_mean_ratio: f64,
}

/// Deterministic operating-envelope report for one acquired signal.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionOperatingEnvelopeSignalReport {
    /// Satellite identifier under test.
    pub sat: SatId,
    /// Signal band under test.
    pub signal_band: SignalBand,
    /// Signal code under test.
    pub signal_code: SignalCode,
    /// GLONASS frequency channel when required by the signal identity.
    pub glonass_frequency_channel: Option<GlonassFrequencyChannel>,
    /// Sample rate used for this signal operating envelope.
    pub sampling_freq_hz: f64,
    /// Intermediate frequency used for this signal operating envelope.
    pub intermediate_freq_hz: f64,
    /// Code-rate basis used for this signal operating envelope.
    pub code_freq_basis_hz: f64,
    /// Primary-code length used for this signal operating envelope.
    pub code_length: usize,
    /// Acquisition Doppler search half-width in Hz.
    pub acquisition_doppler_search_hz: i32,
    /// Acquisition Doppler step in Hz.
    pub acquisition_doppler_step_hz: i32,
    /// Operating points measured for this signal.
    pub points: Vec<SyntheticAcquisitionOperatingEnvelopePoint>,
}

/// Deterministic operating-envelope report across every requested acquired signal.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionOperatingEnvelopeReport {
    /// Scenario identifier prefix shared across the measured operating points.
    pub scenario_id_prefix: String,
    /// Allowed code-phase error in samples.
    pub code_phase_tolerance_samples: usize,
    /// Allowed Doppler error in acquisition bins.
    pub doppler_tolerance_bins: usize,
    /// Signal-specific operating envelopes.
    pub signals: Vec<SyntheticAcquisitionOperatingEnvelopeSignalReport>,
}
