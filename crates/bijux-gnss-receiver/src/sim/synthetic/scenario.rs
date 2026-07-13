fn default_signal_band() -> SignalBand {
    SignalBand::L1
}

fn default_signal_code() -> SignalCode {
    SignalCode::Unknown
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct SyntheticSignalParams {
    pub sat: SatId,
    #[serde(default)]
    pub glonass_frequency_channel: Option<GlonassFrequencyChannel>,
    #[serde(default = "default_signal_band")]
    pub signal_band: SignalBand,
    #[serde(default = "default_signal_code")]
    pub signal_code: SignalCode,
    pub doppler_hz: f64,
    pub code_phase_chips: f64,
    pub carrier_phase_rad: f64,
    pub cn0_db_hz: f32,
    pub data_bit_flip: bool,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct SyntheticDopplerRampParams {
    pub signal: SyntheticSignalParams,
    pub doppler_rate_hz_per_s: f64,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct SyntheticFadeWindow {
    /// Fade start time in seconds, inclusive.
    pub start_s: f64,
    /// Fade end time in seconds, exclusive.
    pub end_s: f64,
    /// Multiplicative scale applied to the synthetic signal inside the window.
    pub signal_scale: f32,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct SyntheticPhaseWindow {
    /// Phase-offset start time in seconds, inclusive.
    pub start_s: f64,
    /// Phase-offset end time in seconds, exclusive.
    pub end_s: f64,
    /// Additive carrier phase offset applied inside the window.
    pub phase_offset_rad: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SyntheticScenario {
    pub sample_rate_hz: f64,
    pub intermediate_freq_hz: f64,
    #[serde(default)]
    pub receiver_clock_frequency_bias_hz: f64,
    pub duration_s: f64,
    pub seed: u64,
    pub satellites: Vec<SyntheticSignalParams>,
    #[serde(default)]
    pub ephemerides: Vec<GpsEphemeris>,
    #[serde(default)]
    pub id: String,
}

/// Per-satellite signal specification for a synthetic navigation validation case.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct SyntheticNavigationSignalSpec {
    /// Satellite identifier.
    pub sat: SatId,
    /// GLONASS FDMA channel when the satellite uses the GLONASS L1 signal plan.
    #[serde(default)]
    pub glonass_frequency_channel: Option<GlonassFrequencyChannel>,
    /// Explicit signal band carried by the synthetic signal.
    #[serde(default = "default_signal_band")]
    pub signal_band: SignalBand,
    /// Explicit signal code carried by the synthetic signal.
    #[serde(default = "default_signal_code")]
    pub signal_code: SignalCode,
    /// Injected Doppler shift in Hz.
    pub doppler_hz: f64,
    /// Injected carrier phase at sample zero, in radians.
    pub carrier_phase_rad: f64,
    /// Injected carrier-to-noise density ratio in dB-Hz.
    pub cn0_db_hz: f32,
    /// Whether the synthetic signal should alternate navigation symbols at the signal-native rate.
    pub data_bit_flip: bool,
}

/// Truth-complete synthetic navigation validation case.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SyntheticNavigationValidationScenario {
    /// Stable scenario identifier for the validation run.
    #[serde(default)]
    pub id: String,
    /// Capture sample rate in Hz.
    pub sample_rate_hz: f64,
    /// Capture intermediate frequency in Hz.
    pub intermediate_freq_hz: f64,
    /// Common receiver clock frequency bias applied to all satellites, in Hz.
    #[serde(default)]
    pub receiver_clock_frequency_bias_hz: f64,
    /// Synthetic capture duration in seconds.
    pub duration_s: f64,
    /// Deterministic seed used for noise generation.
    pub seed: u64,
    /// Truth receiver coordinates in ECEF meters.
    pub receiver_ecef_m: [f64; 3],
    /// Absolute receiver receive-time anchor used for observation and PVT truth, in seconds.
    pub reference_receive_time_s: f64,
    /// Per-satellite validation signal specifications.
    pub satellites: Vec<SyntheticNavigationSignalSpec>,
    /// Broadcast ephemerides used for geometric truth and navigation.
    pub ephemerides: Vec<GpsEphemeris>,
}

/// Deterministic navigation-bit schedule used by a synthetic signal.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum SyntheticNavBitMode {
    /// Keep the navigation-bit sign positive for the full capture.
    ConstantPositive,
    /// Apply the GLONASS L1 string structure with constant positive raw data bits.
    GlonassL1FixedDataString,
    /// Apply the GLONASS L1 string structure with alternating raw data bits after the idle bit.
    GlonassL1AlternatingDataString,
    /// Alternate the bit sign every 20 ms, starting positive at sample zero.
    AlternatingGpsLnav20ms,
    /// Alternate the bit sign every 4 ms, starting positive at sample zero.
    AlternatingGalileoInav4ms,
    /// Alternate the bit sign every 10 ms, starting positive at sample zero.
    AlternatingGpsL5I10ms,
    /// Apply the published 20-chip L5-Q NH pilot sequence at 1 ms epochs.
    GpsL5QNh20,
}

/// Sample-aligned navigation-bit truth interval.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticNavBitSegment {
    /// Inclusive segment start sample.
    pub start_sample: u64,
    /// Exclusive segment end sample.
    pub end_sample: u64,
    /// Segment start time in seconds.
    pub start_s: f64,
    /// Segment end time in seconds.
    pub end_s: f64,
    /// Navigation-bit sign applied over this interval.
    pub bit: i8,
}

/// Per-satellite truth carried alongside a synthetic IQ export.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticSatelliteTruth {
    /// Satellite identifier.
    pub sat: SatId,
    /// GLONASS FDMA carrier channel when this truth row models a GLONASS L1 signal.
    #[serde(default)]
    pub glonass_frequency_channel: Option<GlonassFrequencyChannel>,
    /// Explicit signal band carried by this synthetic signal.
    #[serde(default = "default_signal_band")]
    pub signal_band: SignalBand,
    /// Explicit signal code carried by this synthetic signal.
    #[serde(default = "default_signal_code")]
    pub signal_code: SignalCode,
    /// Injected Doppler shift in Hz.
    pub doppler_hz: f64,
    /// Injected code phase at sample zero, in chips.
    pub code_phase_chips: f64,
    /// Injected carrier phase at sample zero, in radians.
    pub carrier_phase_rad: f64,
    /// Injected carrier-to-noise density ratio in dB-Hz.
    pub cn0_db_hz: f32,
    /// Complex sample amplitude before additive noise and output scaling.
    pub signal_amplitude: f32,
    /// Deterministic navigation-bit model used for this signal.
    pub nav_bit_mode: SyntheticNavBitMode,
    /// Sample-aligned navigation-bit truth intervals.
    pub nav_bit_segments: Vec<SyntheticNavBitSegment>,
}

/// Machine-readable truth for an exported synthetic IQ capture.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticIqTruthBundle {
    /// Truth schema version.
    pub schema_version: u32,
    /// Stable scenario identifier for this capture.
    pub scenario_id: String,
    /// Deterministic seed used for the synthetic noise realization.
    pub seed: u64,
    /// Output sample format.
    pub sample_format: IqSampleFormat,
    /// Output sample rate in Hz.
    pub sample_rate_hz: f64,
    /// Output intermediate frequency in Hz.
    pub intermediate_freq_hz: f64,
    /// Common receiver clock frequency bias added to every synthetic carrier, in Hz.
    pub receiver_clock_frequency_bias_hz: f64,
    /// Synthetic capture start timestamp in UTC.
    pub capture_start_utc: String,
    /// Output quantization depth in bits.
    pub quantization_bits: u8,
    /// Total capture duration in seconds.
    pub duration_s: f64,
    /// Total complex samples emitted into the file.
    pub sample_count: usize,
    /// Gaussian noise standard deviation applied to each I/Q component.
    pub noise_std_per_component: f32,
    /// Total noise power per complex sample before quantization.
    pub noise_power_per_complex_sample: f32,
    /// Peak absolute I/Q component before output scaling.
    pub peak_component_before_scaling: f32,
    /// Scale factor applied before quantization.
    pub output_scale_applied: f32,
    /// Per-satellite truth rows.
    pub satellites: Vec<SyntheticSatelliteTruth>,
}

/// Encoded synthetic capture bundle ready to write to disk.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticIqCaptureBundle {
    /// Raw interleaved IQ bytes in the declared output format.
    pub raw_iq_bytes: Vec<u8>,
    /// Sidecar metadata for the encoded raw IQ file.
    pub metadata: RawIqMetadata,
    /// Machine-readable truth for the emitted capture.
    pub truth: SyntheticIqTruthBundle,
}

/// End-to-end synthetic navigation validation output for one run.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SyntheticNavigationValidationRun {
    /// Truth-complete signal scenario derived from the validation case.
    pub signal_scenario: SyntheticScenario,
    /// Machine-readable truth for the scaled synthetic capture.
    pub truth_bundle: SyntheticIqTruthBundle,
    /// Canonical receiver pipeline artifacts emitted by the validation run.
    #[serde(skip)]
    pub pipeline_artifacts: crate::api::RunArtifacts,
    /// Truth-guided acquisition accuracy report.
    pub acquisition_accuracy: SyntheticAcquisitionAccuracyReport,
    /// Truth-guided tracking accuracy report.
    pub tracking_accuracy: SyntheticTrackingAccuracyReport,
    /// Truth-guided observation accuracy report.
    pub observation_accuracy: SyntheticObservationAccuracyReport,
    /// Truth-guided PVT accuracy report.
    pub pvt_accuracy: SyntheticPvtAccuracyReport,
    /// Final run-level GNSS accuracy artifact.
    pub artifact: SyntheticGnssAccuracyArtifact,
}

/// Error raised while building or validating a synthetic navigation scenario.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SyntheticNavigationValidationError {
    /// The scenario did not declare any satellites.
    EmptySatelliteSet,
    /// One or more satellites were missing matching broadcast ephemerides.
    MissingEphemeris { sat: SatId },
    /// The declared satellites do not share one receiver code epoch base.
    InconsistentReceiverEpochBase,
    /// The scenario did not declare any ephemerides.
    EmptyEphemerides,
    /// The canonical receiver pipeline could not complete for this synthetic scenario.
    ReceiverPipeline { message: String },
}

impl std::fmt::Display for SyntheticNavigationValidationError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::EmptySatelliteSet => {
                write!(f, "synthetic navigation validation scenario must declare satellites")
            }
            Self::MissingEphemeris { sat } => {
                write!(f, "missing ephemeris for {:?}", sat)
            }
            Self::InconsistentReceiverEpochBase => write!(
                f,
                "synthetic navigation validation satellites must share one receiver code epoch base"
            ),
            Self::EmptyEphemerides => {
                write!(f, "synthetic navigation validation scenario must declare ephemerides")
            }
            Self::ReceiverPipeline { message } => {
                write!(f, "synthetic navigation receiver pipeline failed: {message}")
            }
        }
    }
}

impl std::error::Error for SyntheticNavigationValidationError {}

/// Per-satellite C/N0 comparison between synthetic truth and receiver measurement.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticCn0ValidationSatellite {
    /// Satellite identifier.
    pub sat: SatId,
    /// Injected carrier-to-noise density ratio in dB-Hz.
    pub injected_cn0_db_hz: f32,
    /// Mean tracking-correlator estimate across the measured stable tracking window.
    pub measured_mean_cn0_dbhz: f64,
    /// Minimum tracking-correlator estimate across the measured stable tracking window.
    pub measured_min_cn0_dbhz: f64,
    /// Maximum tracking-correlator estimate across the measured stable tracking window.
    pub measured_max_cn0_dbhz: f64,
    /// Mean estimate minus injected truth, in dB-Hz.
    pub cn0_delta_db: f64,
    /// Injected complex amplitude before additive noise and output scaling.
    pub signal_amplitude: f32,
    /// Injected complex amplitude after the export scaling factor is applied.
    pub output_signal_amplitude: f32,
    /// Count of stable tracking epochs measured for this satellite.
    pub epochs_measured: usize,
    /// Whether the measured mean stayed within the requested tolerance.
    pub pass: bool,
}

/// Truth-guided C/N0 validation report for an exported synthetic IQ capture.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticCn0ValidationReport {
    /// Stable scenario identifier for this capture.
    pub scenario_id: String,
    /// Allowed absolute C/N0 error in dB-Hz.
    pub tolerance_db_hz: f64,
    /// Capture sample rate in Hz.
    pub sample_rate_hz: f64,
    /// Number of samples used per coherent receiver estimate.
    pub coherent_samples_per_epoch: usize,
    /// Coherent integration interval in seconds.
    pub coherent_integration_s: f64,
    /// Quantization depth declared in the truth bundle.
    pub quantization_bits: u8,
    /// Gaussian noise standard deviation applied to each I/Q component.
    pub noise_std_per_component: f32,
    /// Total noise power per complex sample before quantization.
    pub noise_power_per_complex_sample: f32,
    /// Scale factor applied before quantization.
    pub output_scale_applied: f32,
    /// Whether every measured satellite passed the requested tolerance.
    pub pass: bool,
    /// Per-satellite comparison rows.
    pub satellites: Vec<SyntheticCn0ValidationSatellite>,
}

fn shared_receiver_epoch_base(
    pseudorange_chips: &[f64],
) -> Result<u64, SyntheticNavigationValidationError> {
    let mut epoch_indices = pseudorange_chips.iter().map(|chips| (chips / 1023.0).floor() as u64);
    let Some(receiver_epoch_base) = epoch_indices.next() else {
        return Err(SyntheticNavigationValidationError::EmptySatelliteSet);
    };
    if !epoch_indices.all(|epoch_idx| epoch_idx == receiver_epoch_base) {
        return Err(SyntheticNavigationValidationError::InconsistentReceiverEpochBase);
    }
    Ok(receiver_epoch_base)
}

fn encode_receiver_code_phase_chips(pseudorange_chips: f64, receiver_epoch_base: u64) -> f64 {
    let code_phase_chips = pseudorange_chips - receiver_epoch_base as f64 * 1023.0;
    code_phase_chips.rem_euclid(1023.0)
}

/// Build a signal-complete synthetic scenario from a truth-complete navigation validation case.
pub fn build_signal_scenario_from_navigation_validation_scenario(
    scenario: &SyntheticNavigationValidationScenario,
) -> Result<SyntheticScenario, SyntheticNavigationValidationError> {
    if scenario.satellites.is_empty() {
        return Err(SyntheticNavigationValidationError::EmptySatelliteSet);
    }
    if scenario.ephemerides.is_empty() {
        return Err(SyntheticNavigationValidationError::EmptyEphemerides);
    }

    let pseudorange_chips = scenario
        .satellites
        .iter()
        .map(|signal| {
            let ephemeris =
                scenario.ephemerides.iter().find(|candidate| candidate.sat == signal.sat).ok_or(
                    SyntheticNavigationValidationError::MissingEphemeris { sat: signal.sat },
                )?;
            Ok(synthetic_pseudorange_m(
                ephemeris,
                scenario.reference_receive_time_s,
                scenario.receiver_ecef_m,
            ) * (1_023_000.0 / SPEED_OF_LIGHT_MPS))
        })
        .collect::<Result<Vec<_>, SyntheticNavigationValidationError>>()?;
    let receiver_epoch_base = shared_receiver_epoch_base(&pseudorange_chips)?;

    Ok(SyntheticScenario {
        sample_rate_hz: scenario.sample_rate_hz,
        intermediate_freq_hz: scenario.intermediate_freq_hz,
        receiver_clock_frequency_bias_hz: scenario.receiver_clock_frequency_bias_hz,
        duration_s: scenario.duration_s,
        seed: scenario.seed,
        satellites: scenario
            .satellites
            .iter()
            .zip(pseudorange_chips.iter().copied())
            .map(|(signal, pseudorange_phase_chips)| SyntheticSignalParams {
                sat: signal.sat,
                glonass_frequency_channel: signal.glonass_frequency_channel,
                signal_band: signal.signal_band,
                signal_code: signal.signal_code,
                doppler_hz: signal.doppler_hz,
                code_phase_chips: encode_receiver_code_phase_chips(
                    pseudorange_phase_chips,
                    receiver_epoch_base,
                ),
                carrier_phase_rad: signal.carrier_phase_rad,
                cn0_db_hz: signal.cn0_db_hz,
                data_bit_flip: signal.data_bit_flip,
            })
            .collect(),
        ephemerides: scenario.ephemerides.clone(),
        id: if scenario.id.trim().is_empty() {
            "synthetic_navigation_validation".to_string()
        } else {
            scenario.id.clone()
        },
    })
}
