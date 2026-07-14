fn default_signal_band() -> SignalBand {
    SignalBand::L1
}

fn default_signal_code() -> SignalCode {
    SignalCode::Unknown
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq, Default)]
#[serde(rename_all = "snake_case")]
pub enum SyntheticNavigationData {
    #[default]
    ConstantPositive,
    ConstantNegative,
    AlternatingStartPositive,
    AlternatingStartNegative,
    SymbolSequence(Vec<i8>),
    GlonassL1String {
        raw_data_bits: Vec<i8>,
    },
}

impl SyntheticNavigationData {
    pub const fn constant_positive() -> Self {
        Self::ConstantPositive
    }

    pub const fn alternating_positive() -> Self {
        Self::AlternatingStartPositive
    }
}

impl From<bool> for SyntheticNavigationData {
    fn from(alternating: bool) -> Self {
        if alternating {
            Self::AlternatingStartPositive
        } else {
            Self::ConstantPositive
        }
    }
}

#[derive(Debug, Clone, Serialize, PartialEq)]
pub struct SyntheticSignalParams {
    pub sat: SatId,
    pub glonass_frequency_channel: Option<GlonassFrequencyChannel>,
    pub signal_band: SignalBand,
    pub signal_code: SignalCode,
    pub doppler_hz: f64,
    pub code_phase_chips: f64,
    pub carrier_phase_rad: f64,
    pub cn0_db_hz: f32,
    pub navigation_data: SyntheticNavigationData,
}

#[derive(Debug, Clone, Deserialize)]
struct SyntheticSignalParamsWire {
    sat: SatId,
    #[serde(default)]
    glonass_frequency_channel: Option<GlonassFrequencyChannel>,
    #[serde(default = "default_signal_band")]
    signal_band: SignalBand,
    #[serde(default = "default_signal_code")]
    signal_code: SignalCode,
    doppler_hz: f64,
    code_phase_chips: f64,
    carrier_phase_rad: f64,
    cn0_db_hz: f32,
    #[serde(default)]
    navigation_data: Option<SyntheticNavigationData>,
    #[serde(default)]
    data_bit_flip: Option<bool>,
}

impl<'de> Deserialize<'de> for SyntheticSignalParams {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        let wire = SyntheticSignalParamsWire::deserialize(deserializer)?;
        Ok(Self {
            sat: wire.sat,
            glonass_frequency_channel: wire.glonass_frequency_channel,
            signal_band: wire.signal_band,
            signal_code: wire.signal_code,
            doppler_hz: wire.doppler_hz,
            code_phase_chips: wire.code_phase_chips,
            carrier_phase_rad: wire.carrier_phase_rad,
            cn0_db_hz: wire.cn0_db_hz,
            navigation_data: resolved_navigation_data(wire.navigation_data, wire.data_bit_flip),
        })
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticDopplerRampParams {
    pub signal: SyntheticSignalParams,
    pub doppler_rate_hz_per_s: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticCarrierDynamicsParams {
    pub signal: SyntheticSignalParams,
    pub doppler_rate_hz_per_s: f64,
    pub doppler_jerk_hz_per_s2: f64,
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

/// Receiver-local oscillator model applied during synthetic signal generation.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticReceiverOscillatorModel {
    /// Constant carrier-frequency offset added to every generated signal, in hertz.
    #[serde(default)]
    pub carrier_frequency_bias_hz: f64,
    /// Linear carrier-frequency drift added to every generated signal, in hertz per second.
    #[serde(default)]
    pub carrier_frequency_drift_hz_per_s: f64,
    /// Fractional sampling-clock error applied to elapsed sample time.
    #[serde(default)]
    pub sampling_clock_fractional_error: f64,
    /// Linear drift of the fractional sampling-clock error, in fractional error per second.
    #[serde(default)]
    pub sampling_clock_fractional_drift_per_s: f64,
    /// Deterministic phase-noise profile applied as an additive carrier phase offset.
    #[serde(default)]
    pub phase_noise: SyntheticReceiverPhaseNoiseModel,
}

impl Default for SyntheticReceiverOscillatorModel {
    fn default() -> Self {
        Self {
            carrier_frequency_bias_hz: 0.0,
            carrier_frequency_drift_hz_per_s: 0.0,
            sampling_clock_fractional_error: 0.0,
            sampling_clock_fractional_drift_per_s: 0.0,
            phase_noise: SyntheticReceiverPhaseNoiseModel::default(),
        }
    }
}

impl SyntheticReceiverOscillatorModel {
    pub const fn with_carrier_frequency_bias_hz(carrier_frequency_bias_hz: f64) -> Self {
        Self {
            carrier_frequency_bias_hz,
            carrier_frequency_drift_hz_per_s: 0.0,
            sampling_clock_fractional_error: 0.0,
            sampling_clock_fractional_drift_per_s: 0.0,
            phase_noise: SyntheticReceiverPhaseNoiseModel {
                seed: 0,
                knot_interval_samples: 0,
                step_std_rad: 0.0,
            },
        }
    }

    pub fn is_nominal(&self) -> bool {
        self.carrier_frequency_bias_hz.abs() <= f64::EPSILON
            && self.carrier_frequency_drift_hz_per_s.abs() <= f64::EPSILON
            && self.sampling_clock_fractional_error.abs() <= f64::EPSILON
            && self.sampling_clock_fractional_drift_per_s.abs() <= f64::EPSILON
            && !self.phase_noise.is_enabled()
    }
}

/// Deterministic random-walk carrier phase-noise profile.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticReceiverPhaseNoiseModel {
    /// Seed for the random-walk phase-noise sequence.
    #[serde(default)]
    pub seed: u64,
    /// Sample spacing between consecutive phase-noise truth knots.
    #[serde(default)]
    pub knot_interval_samples: u64,
    /// One-sigma random-walk phase increment at each knot, in radians.
    #[serde(default)]
    pub step_std_rad: f64,
}

impl Default for SyntheticReceiverPhaseNoiseModel {
    fn default() -> Self {
        Self { seed: 0, knot_interval_samples: 0, step_std_rad: 0.0 }
    }
}

impl SyntheticReceiverPhaseNoiseModel {
    pub fn is_enabled(&self) -> bool {
        self.knot_interval_samples > 0 && self.step_std_rad.abs() > f64::EPSILON
    }
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
#[derive(Debug, Clone, Serialize, PartialEq)]
pub struct SyntheticNavigationSignalSpec {
    /// Satellite identifier.
    pub sat: SatId,
    /// GLONASS FDMA channel when the satellite uses the GLONASS L1 signal plan.
    pub glonass_frequency_channel: Option<GlonassFrequencyChannel>,
    /// Explicit signal band carried by the synthetic signal.
    pub signal_band: SignalBand,
    /// Explicit signal code carried by the synthetic signal.
    pub signal_code: SignalCode,
    /// Injected Doppler shift in Hz.
    pub doppler_hz: f64,
    /// Injected carrier phase at sample zero, in radians.
    pub carrier_phase_rad: f64,
    /// Injected carrier-to-noise density ratio in dB-Hz.
    pub cn0_db_hz: f32,
    /// Navigation symbols applied at the signal-native symbol cadence.
    pub navigation_data: SyntheticNavigationData,
}

#[derive(Debug, Clone, Deserialize)]
struct SyntheticNavigationSignalSpecWire {
    sat: SatId,
    #[serde(default)]
    glonass_frequency_channel: Option<GlonassFrequencyChannel>,
    #[serde(default = "default_signal_band")]
    signal_band: SignalBand,
    #[serde(default = "default_signal_code")]
    signal_code: SignalCode,
    doppler_hz: f64,
    carrier_phase_rad: f64,
    cn0_db_hz: f32,
    #[serde(default)]
    navigation_data: Option<SyntheticNavigationData>,
    #[serde(default)]
    data_bit_flip: Option<bool>,
}

impl<'de> Deserialize<'de> for SyntheticNavigationSignalSpec {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        let wire = SyntheticNavigationSignalSpecWire::deserialize(deserializer)?;
        Ok(Self {
            sat: wire.sat,
            glonass_frequency_channel: wire.glonass_frequency_channel,
            signal_band: wire.signal_band,
            signal_code: wire.signal_code,
            doppler_hz: wire.doppler_hz,
            carrier_phase_rad: wire.carrier_phase_rad,
            cn0_db_hz: wire.cn0_db_hz,
            navigation_data: resolved_navigation_data(wire.navigation_data, wire.data_bit_flip),
        })
    }
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
    /// Optional source-side front-end filter applied before the receiver ingests the capture.
    #[serde(default)]
    pub source_front_end_filter: Option<bijux_gnss_signal::api::FrontEndFilterSpec>,
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
    /// Keep the navigation-bit sign negative for the full capture.
    ConstantNegative,
    /// Apply the GLONASS L1 string structure with constant positive raw data bits.
    GlonassL1FixedDataString,
    /// Apply the GLONASS L1 string structure with alternating raw data bits after the idle bit.
    GlonassL1AlternatingDataString,
    /// Apply the GLONASS L1 string structure using an explicit raw data-bit sequence.
    GlonassL1CustomDataString,
    /// Alternate the bit sign every 20 ms, starting positive at sample zero.
    AlternatingGpsLnav20ms,
    /// Alternate the bit sign every 4 ms, starting positive at sample zero.
    AlternatingGalileoInav4ms,
    /// Alternate the bit sign every 10 ms, starting positive at sample zero.
    AlternatingGpsL5I10ms,
    /// Apply an explicit symbol stream at the signal-native data boundary.
    NativeSymbolSequence,
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

/// One sampled receiver-oscillator truth point.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticReceiverOscillatorTruthPoint {
    /// Absolute sample index covered by this truth point.
    pub sample_index: u64,
    /// Elapsed nominal capture time in seconds at this truth point.
    pub time_s: f64,
    /// Effect value at this truth point.
    pub value: f64,
}

/// Per-effect receiver-oscillator truth series for one synthetic capture.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Default)]
pub struct SyntheticReceiverOscillatorTruth {
    /// Carrier-frequency bias series, in hertz.
    #[serde(default)]
    pub carrier_frequency_bias_hz: Vec<SyntheticReceiverOscillatorTruthPoint>,
    /// Carrier-frequency drift series, in hertz per second.
    #[serde(default)]
    pub carrier_frequency_drift_hz_per_s: Vec<SyntheticReceiverOscillatorTruthPoint>,
    /// Additive carrier phase-noise series, in radians.
    #[serde(default)]
    pub phase_noise_rad: Vec<SyntheticReceiverOscillatorTruthPoint>,
    /// Sampling-clock time-error series, in seconds.
    #[serde(default)]
    pub sampling_clock_time_error_s: Vec<SyntheticReceiverOscillatorTruthPoint>,
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
    /// Synthetic navigation-data stream applied to this signal.
    pub navigation_data: SyntheticNavigationData,
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
    /// Explicit quantization model used before storage.
    pub quantization: IqQuantization,
    /// Output sample rate in Hz.
    pub sample_rate_hz: f64,
    /// Output intermediate frequency in Hz.
    pub intermediate_freq_hz: f64,
    /// Common receiver clock frequency bias added to every synthetic carrier, in Hz.
    pub receiver_clock_frequency_bias_hz: f64,
    /// Full receiver oscillator model applied during generation.
    #[serde(default)]
    pub receiver_oscillator_model: SyntheticReceiverOscillatorModel,
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
    /// Optional source-side front-end filter applied before capture export.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub source_front_end_filter: Option<bijux_gnss_signal::api::FrontEndFilterSpec>,
    /// Sample delay contributed by the source-side front-end filter.
    #[serde(default)]
    pub source_front_end_sample_delay_samples: u64,
    /// Peak absolute I/Q component before output scaling.
    pub peak_component_before_scaling: f32,
    /// Scale factor applied before quantization.
    pub output_scale_applied: f32,
    /// Per-effect receiver-oscillator truth series for the emitted capture.
    #[serde(default)]
    pub receiver_oscillator_truth: SyntheticReceiverOscillatorTruth,
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

/// Per-satellite quantization loss measurement against one float32 synthetic reference.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticQuantizationLossSatellite {
    /// Satellite identifier.
    pub sat: SatId,
    /// Reference acquisition correlation peak from the unquantized float32 capture.
    pub reference_acquisition_peak: f32,
    /// Quantized acquisition correlation peak measured for this profile.
    pub quantized_acquisition_peak: f32,
    /// Positive acquisition correlation loss relative to the float32 reference, in dB.
    ///
    /// `None` means the quantized profile did not produce a measurable acquisition peak.
    pub acquisition_correlation_loss_db: Option<f64>,
    /// Reference acquisition peak-to-mean ratio.
    pub reference_peak_mean_ratio: f32,
    /// Quantized acquisition peak-to-mean ratio.
    pub quantized_peak_mean_ratio: f32,
    /// Reference stable-window tracking C/N0 estimate, in dB-Hz.
    pub reference_mean_cn0_db_hz: f64,
    /// Quantized stable-window tracking C/N0 estimate, in dB-Hz.
    pub quantized_mean_cn0_db_hz: f64,
    /// Positive tracking C/N0 loss relative to the float32 reference, in dB-Hz.
    pub cn0_loss_db_hz: f64,
}

/// Quantization-loss measurements for one synthetic capture profile.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticQuantizationLossPoint {
    /// Quantization profile under test.
    pub quantization: IqQuantization,
    /// Raw IQ sample format used to store this profile.
    pub sample_format: IqSampleFormat,
    /// Effective quantization depth used for this profile.
    pub quantization_bits: u8,
    /// Common output scale applied before quantization.
    pub output_scale_applied: f32,
    /// Per-satellite acquisition and tracking loss measurements.
    pub satellites: Vec<SyntheticQuantizationLossSatellite>,
}

/// Quantization-loss report for one synthetic scenario relative to one float32 reference capture.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticQuantizationLossReport {
    /// Stable scenario identifier for this measurement set.
    pub scenario_id: String,
    /// Reference quantization profile used as the unquantized baseline.
    pub reference_quantization: IqQuantization,
    /// Capture sample rate in Hz.
    pub sample_rate_hz: f64,
    /// Capture intermediate frequency in Hz.
    pub intermediate_freq_hz: f64,
    /// Number of samples used per coherent tracking estimate.
    pub coherent_samples_per_epoch: usize,
    /// Coherent integration interval in seconds.
    pub coherent_integration_s: f64,
    /// Quantization points measured against the float32 reference.
    pub points: Vec<SyntheticQuantizationLossPoint>,
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

fn resolved_navigation_data(
    navigation_data: Option<SyntheticNavigationData>,
    data_bit_flip: Option<bool>,
) -> SyntheticNavigationData {
    navigation_data.or_else(|| data_bit_flip.map(SyntheticNavigationData::from)).unwrap_or_default()
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
                navigation_data: signal.navigation_data.clone(),
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
