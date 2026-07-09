#![allow(missing_docs)]

use std::f32::consts::TAU;

use num_complex::Complex;

use bijux_gnss_core::api::{
    Constellation, SampleClock, SampleTime, SamplesFrame, SatId, Seconds, SignalBand,
};
use bijux_gnss_signal::api::SignalSource;

use crate::engine::receiver_config::ReceiverPipelineConfig;
use crate::io::data::SampleSourceError;
use crate::pipeline::doppler::carrier_hz_from_doppler_hz;
use bijux_gnss_nav::api::GpsEphemeris;
use bijux_gnss_signal::api::{
    advance_code_phase_seconds, code_value_at_phase, generate_ca_code, samples_per_code,
    IqSampleFormat, Prn, RawIqMetadata,
};
use serde::{Deserialize, Serialize};

const SYNTHETIC_IQ_TRUTH_SCHEMA_VERSION: u32 = 2;
const GPS_L1_CA_NAV_BIT_PERIOD_S: f64 = 0.02;
const SYNTHETIC_COMPLEX_NOISE_POWER: f64 = 1.0;
const SYNTHETIC_NOISE_STD_PER_COMPONENT: f32 = std::f32::consts::FRAC_1_SQRT_2;

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct SyntheticSignalParams {
    pub sat: SatId,
    pub doppler_hz: f64,
    pub code_phase_chips: f64,
    pub carrier_phase_rad: f64,
    pub cn0_db_hz: f32,
    pub data_bit_flip: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SyntheticScenario {
    pub sample_rate_hz: f64,
    pub intermediate_freq_hz: f64,
    pub duration_s: f64,
    pub seed: u64,
    pub satellites: Vec<SyntheticSignalParams>,
    #[serde(default)]
    pub ephemerides: Vec<GpsEphemeris>,
    #[serde(default)]
    pub id: String,
}

/// Deterministic navigation-bit schedule used by a synthetic signal.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum SyntheticNavBitMode {
    /// Keep the navigation-bit sign positive for the full capture.
    ConstantPositive,
    /// Alternate the bit sign every 20 ms, starting positive at sample zero.
    AlternatingGpsLnav20ms,
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

/// Per-satellite C/N0 comparison between synthetic truth and receiver measurement.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticCn0ValidationSatellite {
    /// Satellite identifier.
    pub sat: SatId,
    /// Injected carrier-to-noise density ratio in dB-Hz.
    pub injected_cn0_db_hz: f32,
    /// Mean truth-guided receiver estimate across measured epochs.
    pub measured_mean_cn0_dbhz: f64,
    /// Minimum truth-guided receiver estimate across measured epochs.
    pub measured_min_cn0_dbhz: f64,
    /// Maximum truth-guided receiver estimate across measured epochs.
    pub measured_max_cn0_dbhz: f64,
    /// Mean estimate minus injected truth, in dB-Hz.
    pub cn0_delta_db: f64,
    /// Injected complex amplitude before additive noise and output scaling.
    pub signal_amplitude: f32,
    /// Injected complex amplitude after the export scaling factor is applied.
    pub output_signal_amplitude: f32,
    /// Count of coherent epochs measured for this satellite.
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

/// Per-satellite acquisition code-phase comparison between clean synthetic truth and receiver output.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionCodePhaseValidationSatellite {
    /// Satellite identifier.
    pub sat: SatId,
    /// Injected code phase at the start of the validation frame, in chips.
    pub injected_code_phase_chips: f64,
    /// Expected acquisition-reported code phase sample under the receiver search convention.
    pub expected_code_phase_samples: usize,
    /// Measured acquisition-reported code phase sample.
    pub measured_code_phase_samples: usize,
    /// Wrapped absolute error between expected and measured code phase samples.
    pub code_phase_error_samples: usize,
    /// Peak-to-mean ratio for the selected acquisition result.
    pub peak_mean_ratio: f32,
    /// Acquisition hypothesis returned by the receiver.
    pub hypothesis: String,
    /// Whether the measured code phase stayed within tolerance.
    pub pass: bool,
}

/// Truth-guided acquisition code-phase validation report for a synthetic capture.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionCodePhaseValidationReport {
    /// Stable scenario identifier for this capture.
    pub scenario_id: String,
    /// Allowed wrapped absolute error in samples.
    pub tolerance_samples: usize,
    /// Capture sample rate in Hz.
    pub sample_rate_hz: f64,
    /// Number of samples in one code period at the configured rate.
    pub period_samples: usize,
    /// Whether every measured satellite passed the requested tolerance.
    pub pass: bool,
    /// Per-satellite comparison rows.
    pub satellites: Vec<SyntheticAcquisitionCodePhaseValidationSatellite>,
}

/// Build a machine-readable truth bundle for an emitted synthetic capture.
pub fn build_truth_bundle(
    scenario_id: &str,
    scenario: &SyntheticScenario,
    frame: &SamplesFrame,
    metadata: &RawIqMetadata,
    peak_component_before_scaling: f32,
    output_scale_applied: f32,
) -> SyntheticIqTruthBundle {
    SyntheticIqTruthBundle {
        schema_version: SYNTHETIC_IQ_TRUTH_SCHEMA_VERSION,
        scenario_id: scenario_id.to_string(),
        seed: scenario.seed,
        sample_format: metadata.format,
        sample_rate_hz: frame.t0.sample_rate_hz,
        intermediate_freq_hz: metadata.intermediate_freq_hz,
        capture_start_utc: metadata.capture_start_utc.clone(),
        quantization_bits: metadata.quantization_bits.unwrap_or_default(),
        duration_s: frame.len() as f64 * frame.dt_s.0,
        sample_count: frame.len(),
        noise_std_per_component: SYNTHETIC_NOISE_STD_PER_COMPONENT,
        noise_power_per_complex_sample: SYNTHETIC_COMPLEX_NOISE_POWER as f32,
        peak_component_before_scaling,
        output_scale_applied,
        satellites: scenario
            .satellites
            .iter()
            .map(|params| SyntheticSatelliteTruth {
                sat: params.sat,
                doppler_hz: params.doppler_hz,
                code_phase_chips: params.code_phase_chips,
                carrier_phase_rad: params.carrier_phase_rad,
                cn0_db_hz: params.cn0_db_hz,
                signal_amplitude: signal_amplitude_from_cn0(
                    params.cn0_db_hz,
                    frame.t0.sample_rate_hz,
                ),
                nav_bit_mode: nav_bit_mode(params),
                nav_bit_segments: nav_bit_segments(
                    frame.t0.sample_rate_hz,
                    frame.len() as u64,
                    params.data_bit_flip,
                ),
            })
            .collect(),
    }
}

/// Encode a generated synthetic frame as an IQ16 little-endian capture with truth metadata.
pub fn build_iq16_capture_bundle(
    scenario_id: &str,
    scenario: &SyntheticScenario,
    frame: &SamplesFrame,
    capture_start_utc: &str,
    notes: Option<String>,
) -> SyntheticIqCaptureBundle {
    let peak_component_before_scaling = peak_component(&frame.iq);
    let output_scale_applied = if peak_component_before_scaling <= 0.999 {
        1.0
    } else {
        0.999 / peak_component_before_scaling
    };
    let raw_iq_bytes = encode_iq16_le_bytes(&frame.iq, output_scale_applied);
    let metadata = RawIqMetadata {
        format: IqSampleFormat::Iq16Le,
        sample_rate_hz: frame.t0.sample_rate_hz,
        intermediate_freq_hz: scenario.intermediate_freq_hz,
        capture_start_utc: capture_start_utc.to_string(),
        offset_bytes: 0,
        quantization_bits: Some(16),
        notes,
    };
    let truth = build_truth_bundle(
        scenario_id,
        scenario,
        frame,
        &metadata,
        peak_component_before_scaling,
        output_scale_applied,
    );
    SyntheticIqCaptureBundle { raw_iq_bytes, metadata, truth }
}

/// Measure truth-guided C/N0 directly from a synthetic capture using receiver correlators.
pub fn validate_truth_guided_cn0(
    config: &ReceiverPipelineConfig,
    frame: &SamplesFrame,
    truth: &SyntheticIqTruthBundle,
    tolerance_db_hz: f64,
) -> SyntheticCn0ValidationReport {
    let coherent_samples_per_epoch =
        samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length)
            .min(frame.len());
    let coherent_integration_s = coherent_samples_per_epoch as f64 * frame.dt_s.0;
    let tracking = crate::pipeline::tracking::Tracking::new(
        config.clone(),
        crate::engine::runtime::ReceiverRuntime::default(),
    );
    let early_late_spacing_chips = config.tracking_params(SignalBand::L1).early_late_spacing_chips;
    let measured_epochs =
        if coherent_samples_per_epoch == 0 { 0 } else { frame.len() / coherent_samples_per_epoch };
    let satellites = truth
        .satellites
        .iter()
        .map(|sat_truth| {
            let isolated_frame =
                regenerate_isolated_scaled_satellite_frame(config, frame, truth, sat_truth);
            let mut cn0_values = Vec::with_capacity(measured_epochs);
            for epoch_index in 0..measured_epochs {
                let start = epoch_index * coherent_samples_per_epoch;
                let end = start + coherent_samples_per_epoch;
                let epoch_frame = SamplesFrame::new(
                    SampleTime {
                        sample_index: isolated_frame.t0.sample_index + start as u64,
                        sample_rate_hz: isolated_frame.t0.sample_rate_hz,
                    },
                    isolated_frame.dt_s,
                    isolated_frame.iq[start..end].to_vec(),
                );
                let code_phase_samples = code_phase_samples_at_epoch_start(
                    config,
                    &epoch_frame,
                    sat_truth.code_phase_chips,
                );
                let carrier_hz = synthetic_carrier_hz(
                    truth.intermediate_freq_hz,
                    sat_truth.sat,
                    sat_truth.doppler_hz,
                );
                let (track_epoch, _) = tracking.track_epoch(
                    &epoch_frame,
                    0,
                    sat_truth.sat,
                    carrier_hz,
                    code_phase_samples,
                    early_late_spacing_chips,
                );
                let prompt = Complex::new(track_epoch.prompt_i, track_epoch.prompt_q);
                cn0_values.push(measure_cn0_from_prompt_with_known_noise(
                    prompt,
                    truth.output_scale_applied,
                    truth.noise_power_per_complex_sample as f64,
                    frame.t0.sample_rate_hz,
                    coherent_samples_per_epoch as f64,
                ));
            }

            let measured_mean_cn0_dbhz = if cn0_values.is_empty() {
                0.0
            } else {
                cn0_values.iter().sum::<f64>() / cn0_values.len() as f64
            };
            let measured_min_cn0_dbhz = cn0_values.iter().copied().reduce(f64::min).unwrap_or(0.0);
            let measured_max_cn0_dbhz = cn0_values.iter().copied().reduce(f64::max).unwrap_or(0.0);
            let cn0_delta_db = measured_mean_cn0_dbhz - sat_truth.cn0_db_hz as f64;
            let pass = !cn0_values.is_empty() && cn0_delta_db.abs() <= tolerance_db_hz;

            SyntheticCn0ValidationSatellite {
                sat: sat_truth.sat,
                injected_cn0_db_hz: sat_truth.cn0_db_hz,
                measured_mean_cn0_dbhz,
                measured_min_cn0_dbhz,
                measured_max_cn0_dbhz,
                cn0_delta_db,
                signal_amplitude: sat_truth.signal_amplitude,
                output_signal_amplitude: sat_truth.signal_amplitude * truth.output_scale_applied,
                epochs_measured: cn0_values.len(),
                pass,
            }
        })
        .collect::<Vec<_>>();
    let pass = !satellites.is_empty() && satellites.iter().all(|row| row.pass);

    SyntheticCn0ValidationReport {
        scenario_id: truth.scenario_id.clone(),
        tolerance_db_hz,
        sample_rate_hz: truth.sample_rate_hz,
        coherent_samples_per_epoch,
        coherent_integration_s,
        quantization_bits: truth.quantization_bits,
        noise_std_per_component: truth.noise_std_per_component,
        noise_power_per_complex_sample: truth.noise_power_per_complex_sample,
        output_scale_applied: truth.output_scale_applied,
        pass,
        satellites,
    }
}

/// Convert a truth-model code phase into the receiver's acquisition-reported sample convention.
pub fn expected_acquisition_code_phase_samples(
    config: &ReceiverPipelineConfig,
    frame: &SamplesFrame,
    code_phase_chips: f64,
) -> usize {
    let period_samples =
        samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length)
            .max(1);
    let phase_samples = code_phase_samples_at_epoch_start(config, frame, code_phase_chips);
    let injected_sample = (phase_samples.round() as usize) % period_samples;
    (period_samples - injected_sample) % period_samples
}

/// Measure wrapped code-phase error in samples over one code period.
pub fn wrapped_code_phase_error_samples(
    actual: usize,
    expected: usize,
    period_samples: usize,
) -> usize {
    let period_samples = period_samples.max(1);
    let forward = actual.abs_diff(expected);
    let wrapped = period_samples.saturating_sub(forward);
    forward.min(wrapped)
}

/// Measure truth-guided acquisition code-phase accuracy from a synthetic capture.
pub fn validate_truth_guided_acquisition_code_phase(
    config: &ReceiverPipelineConfig,
    frame: &SamplesFrame,
    truth: &SyntheticIqTruthBundle,
    tolerance_samples: usize,
) -> SyntheticAcquisitionCodePhaseValidationReport {
    let period_samples =
        samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length)
            .max(1);
    let satellites = truth
        .satellites
        .iter()
        .map(|sat_truth| {
            let isolated_frame =
                regenerate_isolated_scaled_satellite_signal_only_frame(config, frame, truth, sat_truth);
            let mut acquisition_config = config.clone();
            acquisition_config.intermediate_freq_hz =
                synthetic_carrier_hz(truth.intermediate_freq_hz, sat_truth.sat, sat_truth.doppler_hz);
            acquisition_config.acquisition_doppler_search_hz = 0;
            acquisition_config.acquisition_doppler_step_hz = 1;
            let acquisition = crate::pipeline::acquisition::Acquisition::new(
                acquisition_config,
                crate::engine::runtime::ReceiverRuntime::default(),
            )
            .with_doppler(0, 1);
            let result = acquisition.run_fft(&isolated_frame, &[sat_truth.sat]).remove(0);
            let expected_code_phase_samples =
                expected_acquisition_code_phase_samples(config, &isolated_frame, sat_truth.code_phase_chips);
            let code_phase_error_samples = wrapped_code_phase_error_samples(
                result.code_phase_samples,
                expected_code_phase_samples,
                period_samples,
            );
            let pass = matches!(
                result.hypothesis,
                crate::api::core::AcqHypothesis::Accepted | crate::api::core::AcqHypothesis::Ambiguous
            ) && code_phase_error_samples <= tolerance_samples;

            SyntheticAcquisitionCodePhaseValidationSatellite {
                sat: sat_truth.sat,
                injected_code_phase_chips: sat_truth.code_phase_chips,
                expected_code_phase_samples,
                measured_code_phase_samples: result.code_phase_samples,
                code_phase_error_samples,
                peak_mean_ratio: result.peak_mean_ratio,
                hypothesis: result.hypothesis.to_string(),
                pass,
            }
        })
        .collect::<Vec<_>>();
    let pass = !satellites.is_empty() && satellites.iter().all(|row| row.pass);

    SyntheticAcquisitionCodePhaseValidationReport {
        scenario_id: truth.scenario_id.clone(),
        tolerance_samples,
        sample_rate_hz: truth.sample_rate_hz,
        period_samples,
        pass,
        satellites,
    }
}

/// Generate a synthetic GPS L1 C/A signal at the receiver sample rate.
///
/// The C/N0 control is approximate and intended for test harnesses.
pub fn generate_l1_ca(
    config: &ReceiverPipelineConfig,
    params: SyntheticSignalParams,
    seed: u64,
    duration_s: f64,
) -> SamplesFrame {
    generate_l1_ca_multi(
        config,
        &SyntheticScenario {
            sample_rate_hz: config.sampling_freq_hz,
            intermediate_freq_hz: config.intermediate_freq_hz,
            duration_s,
            seed,
            satellites: vec![params],
            ephemerides: Vec::new(),
            id: "synthetic".to_string(),
        },
    )
}

pub fn generate_l1_ca_multi(
    config: &ReceiverPipelineConfig,
    scenario: &SyntheticScenario,
) -> SamplesFrame {
    let signal_only = generate_l1_ca_multi_signal_only(config, scenario);
    let clock = SampleClock::new(config.sampling_freq_hz);
    let dt_s = clock.dt_s();
    let noise_std = SYNTHETIC_NOISE_STD_PER_COMPONENT;
    let mut rng = XorShift64::new(scenario.seed);
    let mut iq = Vec::with_capacity(signal_only.len());
    for sample in signal_only.iq {
        let noise_i = rng.next_gaussian() * noise_std;
        let noise_q = rng.next_gaussian() * noise_std;
        let noise = Complex::new(noise_i, noise_q);
        iq.push(sample + noise);
    }
    SamplesFrame::new(signal_only.t0, Seconds(dt_s), iq)
}

/// Streaming synthetic GPS L1 C/A signal source for long-duration receiver tests.
pub struct SyntheticSignalSource {
    sample_rate_hz: f64,
    dt_s: f64,
    remaining_samples: usize,
    next_sample_index: u64,
    noise_std: f32,
    sat_states: Vec<SatState>,
    rng: XorShift64,
}

fn generate_l1_ca_multi_signal_only(
    config: &ReceiverPipelineConfig,
    scenario: &SyntheticScenario,
) -> SamplesFrame {
    let clock = SampleClock::new(config.sampling_freq_hz);
    let dt_s = clock.dt_s();
    let sample_count = (scenario.duration_s * config.sampling_freq_hz).round() as usize;
    let sat_states: Vec<SatState> =
        scenario.satellites.iter().map(|sat| SatState::new(config, *sat)).collect();
    let mut iq = Vec::with_capacity(sample_count);
    for n in 0..sample_count {
        let t = n as f64 * dt_s;
        let mut sample = Complex::new(0.0f32, 0.0f32);
        for sat in &sat_states {
            sample += sat.sample_at(t);
        }
        iq.push(sample);
    }
    let t0 = SampleTime { sample_index: 0, sample_rate_hz: config.sampling_freq_hz };
    SamplesFrame::new(t0, Seconds(dt_s), iq)
}

impl SyntheticSignalSource {
    /// Build a streaming synthetic source from a scenario without materializing the full capture.
    pub fn new(config: &ReceiverPipelineConfig, scenario: &SyntheticScenario) -> Self {
        let sample_count = (scenario.duration_s * config.sampling_freq_hz).round() as usize;
        let noise_std = SYNTHETIC_NOISE_STD_PER_COMPONENT;

        Self {
            sample_rate_hz: config.sampling_freq_hz,
            dt_s: 1.0 / config.sampling_freq_hz,
            remaining_samples: sample_count,
            next_sample_index: 0,
            noise_std,
            sat_states: scenario.satellites.iter().map(|sat| SatState::new(config, *sat)).collect(),
            rng: XorShift64::new(scenario.seed),
        }
    }
}

impl SignalSource for SyntheticSignalSource {
    type Error = SampleSourceError;

    fn sample_rate_hz(&self) -> f64 {
        self.sample_rate_hz
    }

    fn next_frame(&mut self, frame_len: usize) -> Result<Option<SamplesFrame>, Self::Error> {
        if self.remaining_samples == 0 {
            return Ok(None);
        }
        let count = self.remaining_samples.min(frame_len.max(1));
        let t0 = SampleTime {
            sample_index: self.next_sample_index,
            sample_rate_hz: self.sample_rate_hz,
        };
        let mut iq = Vec::with_capacity(count);
        for offset in 0..count {
            let t = (self.next_sample_index + offset as u64) as f64 * self.dt_s;
            let mut sample = Complex::new(0.0f32, 0.0f32);
            for sat in &self.sat_states {
                sample += sat.sample_at(t);
            }
            let noise_i = self.rng.next_gaussian() * self.noise_std;
            let noise_q = self.rng.next_gaussian() * self.noise_std;
            iq.push(sample + Complex::new(noise_i, noise_q));
        }
        self.next_sample_index += count as u64;
        self.remaining_samples -= count;
        Ok(Some(SamplesFrame::new(t0, Seconds(self.dt_s), iq)))
    }

    fn is_done(&self) -> bool {
        self.remaining_samples == 0
    }
}

#[derive(Debug, Clone)]
struct SatState {
    doppler_hz: f64,
    code_phase_chips: f64,
    carrier_phase_rad: f64,
    cn0_db_hz: f32,
    data_bit_flip: bool,
    code: Vec<i8>,
    code_rate_hz: f64,
    if_hz: f64,
    sample_rate_hz: f64,
}

impl SatState {
    fn new(config: &ReceiverPipelineConfig, params: SyntheticSignalParams) -> Self {
        Self {
            doppler_hz: params.doppler_hz,
            code_phase_chips: params.code_phase_chips,
            carrier_phase_rad: params.carrier_phase_rad,
            cn0_db_hz: params.cn0_db_hz,
            data_bit_flip: params.data_bit_flip,
            code: generate_ca_code(Prn(params.sat.prn)).unwrap_or_else(|_| vec![1; 1023]),
            code_rate_hz: config.code_freq_basis_hz,
            if_hz: synthetic_intermediate_frequency_hz(config.intermediate_freq_hz, params.sat),
            sample_rate_hz: config.sampling_freq_hz,
        }
    }

    fn sample_at(&self, t: f64) -> Complex<f32> {
        let code_phase = advance_code_phase_seconds(
            self.code_phase_chips,
            self.code_rate_hz,
            t,
            self.code.len(),
        )
        .expect("synthetic generator requires a valid code phase model");
        let chip = code_value_at_phase(&self.code, code_phase).unwrap_or(1.0);
        let data_bit = nav_bit_value_at_time_s(self.data_bit_flip, t);

        let carrier_hz = self.if_hz + self.doppler_hz;
        let phase = self.carrier_phase_rad as f32 + TAU * (carrier_hz as f32) * (t as f32);
        let carrier = Complex::new(phase.cos(), phase.sin());

        let amplitude = signal_amplitude_from_cn0(self.cn0_db_hz, self.sample_rate_hz);

        carrier * (chip * data_bit * amplitude)
    }
}

fn signal_amplitude_from_cn0(cn0_db_hz: f32, sample_rate_hz: f64) -> f32 {
    let cn0_linear = 10.0_f64.powf(cn0_db_hz as f64 / 10.0).max(1e-12);
    ((cn0_linear * SYNTHETIC_COMPLEX_NOISE_POWER) / sample_rate_hz).sqrt() as f32
}

fn measure_cn0_from_prompt_with_known_noise(
    prompt: Complex<f32>,
    output_scale_applied: f32,
    noise_power_per_complex_sample: f64,
    sample_rate_hz: f64,
    prompt_coherent_gain: f64,
) -> f64 {
    let scaled_noise_power_per_complex_sample =
        noise_power_per_complex_sample * (output_scale_applied as f64).powi(2);
    let prompt_noise_power = scaled_noise_power_per_complex_sample * prompt_coherent_gain;
    let prompt_power = prompt.norm_sqr() as f64;
    let signal_power_per_sample =
        ((prompt_power - prompt_noise_power).max(1e-12)) / prompt_coherent_gain.powi(2);
    let cn0_linear =
        signal_power_per_sample * sample_rate_hz / scaled_noise_power_per_complex_sample;
    10.0 * cn0_linear.max(1e-12).log10()
}

fn regenerate_isolated_scaled_satellite_frame(
    config: &ReceiverPipelineConfig,
    measured_frame: &SamplesFrame,
    truth: &SyntheticIqTruthBundle,
    sat_truth: &SyntheticSatelliteTruth,
) -> SamplesFrame {
    let isolated_frame = generate_l1_ca_multi(config, &isolated_satellite_scenario(measured_frame, truth, sat_truth));
    let iq = isolated_frame
        .iq
        .iter()
        .map(|sample| *sample * truth.output_scale_applied)
        .collect::<Vec<_>>();
    SamplesFrame::new(measured_frame.t0, measured_frame.dt_s, iq)
}

fn regenerate_isolated_scaled_satellite_signal_only_frame(
    config: &ReceiverPipelineConfig,
    measured_frame: &SamplesFrame,
    truth: &SyntheticIqTruthBundle,
    sat_truth: &SyntheticSatelliteTruth,
) -> SamplesFrame {
    let isolated_frame = generate_l1_ca_multi_signal_only(
        config,
        &isolated_satellite_scenario(measured_frame, truth, sat_truth),
    );
    let iq = isolated_frame
        .iq
        .iter()
        .map(|sample| *sample * truth.output_scale_applied)
        .collect::<Vec<_>>();
    SamplesFrame::new(measured_frame.t0, measured_frame.dt_s, iq)
}

fn isolated_satellite_scenario(
    measured_frame: &SamplesFrame,
    truth: &SyntheticIqTruthBundle,
    sat_truth: &SyntheticSatelliteTruth,
) -> SyntheticScenario {
    SyntheticScenario {
        sample_rate_hz: truth.sample_rate_hz,
        intermediate_freq_hz: truth.intermediate_freq_hz,
        duration_s: measured_frame.len() as f64 * measured_frame.dt_s.0,
        seed: truth.seed,
        satellites: vec![SyntheticSignalParams {
            sat: sat_truth.sat,
            doppler_hz: sat_truth.doppler_hz,
            code_phase_chips: sat_truth.code_phase_chips,
            carrier_phase_rad: sat_truth.carrier_phase_rad,
            cn0_db_hz: sat_truth.cn0_db_hz,
            data_bit_flip: sat_truth.nav_bit_mode == SyntheticNavBitMode::AlternatingGpsLnav20ms,
        }],
        ephemerides: Vec::new(),
        id: sat_truth.sat.prn.to_string(),
    }
}

fn code_phase_samples_at_epoch_start(
    config: &ReceiverPipelineConfig,
    frame: &SamplesFrame,
    code_phase_chips: f64,
) -> f64 {
    let start_s = frame.t0.sample_index as f64 / frame.t0.sample_rate_hz;
    let chip_phase = advance_code_phase_seconds(
        code_phase_chips,
        config.code_freq_basis_hz,
        start_s,
        config.code_length,
    )
    .expect("synthetic epoch alignment requires a valid code phase model");
    let samples_per_chip = frame.t0.sample_rate_hz / config.code_freq_basis_hz;
    chip_phase * samples_per_chip
}

fn synthetic_intermediate_frequency_hz(intermediate_freq_hz: f64, sat: SatId) -> f64 {
    intermediate_freq_hz
        + (synthetic_constellation_carrier_hz(sat)
            - bijux_gnss_core::api::GPS_L1_CA_CARRIER_HZ.value())
}

fn synthetic_carrier_hz(intermediate_freq_hz: f64, sat: SatId, doppler_hz: f64) -> f64 {
    carrier_hz_from_doppler_hz(
        synthetic_intermediate_frequency_hz(intermediate_freq_hz, sat),
        doppler_hz,
    )
}

fn synthetic_constellation_carrier_hz(sat: SatId) -> f64 {
    match sat.constellation {
        Constellation::Galileo => bijux_gnss_core::api::GALILEO_E1_CARRIER_HZ.value(),
        Constellation::Gps => bijux_gnss_core::api::GPS_L1_CA_CARRIER_HZ.value(),
        Constellation::Glonass => bijux_gnss_core::api::GLONASS_L1_CARRIER_HZ.value(),
        _ => bijux_gnss_core::api::GPS_L1_CA_CARRIER_HZ.value(),
    }
}

fn nav_bit_mode(params: &SyntheticSignalParams) -> SyntheticNavBitMode {
    if params.data_bit_flip {
        SyntheticNavBitMode::AlternatingGpsLnav20ms
    } else {
        SyntheticNavBitMode::ConstantPositive
    }
}

fn nav_bit_value_at_time_s(data_bit_flip: bool, time_s: f64) -> f32 {
    nav_bit_sign_at_time_s(data_bit_flip, time_s) as f32
}

fn nav_bit_sign_at_time_s(data_bit_flip: bool, time_s: f64) -> i8 {
    if !data_bit_flip {
        return 1;
    }
    nav_bit_sign_for_index(nav_bit_index_at_time_s(time_s))
}

fn nav_bit_index_at_time_s(time_s: f64) -> u64 {
    if !time_s.is_finite() || time_s <= 0.0 {
        return 0;
    }
    (time_s / GPS_L1_CA_NAV_BIT_PERIOD_S).floor() as u64
}

fn nav_bit_sign_for_index(bit_index: u64) -> i8 {
    if bit_index % 2 == 0 {
        1
    } else {
        -1
    }
}

fn peak_component(samples: &[Complex<f32>]) -> f32 {
    samples.iter().flat_map(|sample| [sample.re.abs(), sample.im.abs()]).fold(0.0f32, f32::max)
}

fn encode_iq16_le_bytes(samples: &[Complex<f32>], scale: f32) -> Vec<u8> {
    let mut encoded = Vec::with_capacity(samples.len() * 4);
    for sample in samples {
        encoded.extend_from_slice(&quantize_i16_component(sample.re * scale).to_le_bytes());
        encoded.extend_from_slice(&quantize_i16_component(sample.im * scale).to_le_bytes());
    }
    encoded
}

fn quantize_i16_component(value: f32) -> i16 {
    let scaled = (value * 32768.0).round();
    scaled.clamp(-32768.0, 32767.0) as i16
}

fn nav_bit_segments(
    sample_rate_hz: f64,
    sample_count: u64,
    data_bit_flip: bool,
) -> Vec<SyntheticNavBitSegment> {
    if sample_count == 0 {
        return Vec::new();
    }
    if !data_bit_flip {
        return vec![SyntheticNavBitSegment {
            start_sample: 0,
            end_sample: sample_count,
            start_s: 0.0,
            end_s: sample_count as f64 / sample_rate_hz,
            bit: 1,
        }];
    }

    let mut segments = Vec::new();
    let mut bit_index = 0u64;
    loop {
        let start_sample =
            ((bit_index as f64 * GPS_L1_CA_NAV_BIT_PERIOD_S * sample_rate_hz).ceil()) as u64;
        if start_sample >= sample_count {
            break;
        }
        let end_sample = ((((bit_index + 1) as f64) * GPS_L1_CA_NAV_BIT_PERIOD_S * sample_rate_hz)
            .ceil()) as u64;
        let clamped_end = end_sample.min(sample_count);
        segments.push(SyntheticNavBitSegment {
            start_sample,
            end_sample: clamped_end,
            start_s: start_sample as f64 / sample_rate_hz,
            end_s: clamped_end as f64 / sample_rate_hz,
            bit: nav_bit_sign_for_index(bit_index),
        });
        bit_index += 1;
    }
    segments
}

#[derive(Debug, Clone)]
struct XorShift64 {
    state: u64,
}

impl XorShift64 {
    fn new(seed: u64) -> Self {
        let seed = if seed == 0 { 0xDEADBEEFCAFEBABE } else { seed };
        Self { state: seed }
    }

    fn next_u64(&mut self) -> u64 {
        let mut x = self.state;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.state = x;
        x
    }

    fn next_f32(&mut self) -> f32 {
        let val = (self.next_u64() >> 40) as u32;
        val as f32 / (u32::MAX as f32)
    }

    fn next_gaussian(&mut self) -> f32 {
        let u1 = self.next_f32().max(1e-12);
        let u2 = self.next_f32();
        let r = (-2.0 * u1.ln()).sqrt();
        let theta = TAU * u2;
        r * theta.cos()
    }
}

#[cfg(test)]
mod tests {
    use super::{
        build_iq16_capture_bundle, build_truth_bundle, generate_l1_ca_multi,
        expected_acquisition_code_phase_samples, nav_bit_index_at_time_s, nav_bit_sign_at_time_s,
        signal_amplitude_from_cn0, validate_truth_guided_acquisition_code_phase,
        validate_truth_guided_cn0, wrapped_code_phase_error_samples, SatState,
        SyntheticNavBitMode, SyntheticScenario, SyntheticSignalParams, SyntheticSignalSource,
        SYNTHETIC_COMPLEX_NOISE_POWER, SYNTHETIC_NOISE_STD_PER_COMPONENT,
    };
    use crate::engine::receiver_config::ReceiverPipelineConfig;
    use bijux_gnss_core::api::{Constellation, SampleTime, SamplesFrame, SatId, Seconds};
    use bijux_gnss_signal::api::{
        advance_code_phase_seconds, sample_ca_code, samples_per_code, IqSampleFormat, Prn,
        RawIqMetadata, SignalSource,
    };

    const RECEIVER_PHASE_TOLERANCE_SAMPLES: f64 = 1e-6;

    #[test]
    fn synthetic_signal_source_matches_materialized_generator() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 1_023_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            ..ReceiverPipelineConfig::default()
        };
        let scenario = SyntheticScenario {
            sample_rate_hz: config.sampling_freq_hz,
            intermediate_freq_hz: config.intermediate_freq_hz,
            duration_s: 0.004,
            seed: 29,
            satellites: vec![SyntheticSignalParams {
                sat: SatId { constellation: Constellation::Gps, prn: 7 },
                doppler_hz: 750.0,
                code_phase_chips: 15.0,
                carrier_phase_rad: 0.0,
                cn0_db_hz: 47.0,
                data_bit_flip: false,
            }],
            ephemerides: Vec::new(),
            id: "synthetic-stream".to_string(),
        };

        let expected = generate_l1_ca_multi(&config, &scenario);
        let mut source = SyntheticSignalSource::new(&config, &scenario);
        let streamed = collect_frames(&mut source, 2 * 1_023);

        assert_eq!(expected.len(), streamed.len());
        assert_eq!(expected.t0, streamed.t0);
        assert_eq!(expected.dt_s, streamed.dt_s);
        assert_eq!(expected.iq, streamed.iq);
        assert!(source.is_done());
    }

    fn collect_frames(source: &mut SyntheticSignalSource, frame_len: usize) -> SamplesFrame {
        let mut frames = Vec::new();
        while let Some(frame) = source.next_frame(frame_len).expect("synthetic frame") {
            frames.push(frame);
        }
        let first = frames.first().expect("at least one frame");
        let t0 = first.t0;
        let dt_s = first.dt_s;
        let iq = frames.into_iter().flat_map(|frame| frame.iq).collect::<Vec<_>>();
        SamplesFrame::new(t0, dt_s, iq)
    }

    #[test]
    fn synthetic_epoch_start_phase_matches_theoretical_phase_after_sixty_seconds() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_000_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            ..ReceiverPipelineConfig::default()
        };
        let params = SyntheticSignalParams {
            sat: SatId { constellation: Constellation::Gps, prn: 11 },
            doppler_hz: 0.0,
            code_phase_chips: 200.375,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 58.0,
            data_bit_flip: false,
        };
        let sat_state = SatState::new(&config, params);
        let code_period_samples = samples_per_code(
            config.sampling_freq_hz,
            config.code_freq_basis_hz,
            config.code_length,
        );
        let sixty_second_sample_index = (60.0 * config.sampling_freq_hz) as u64;
        let frame = synthetic_epoch_frame(
            &sat_state,
            &config,
            sixty_second_sample_index,
            code_period_samples,
        );

        let actual_phase_samples =
            super::code_phase_samples_at_epoch_start(&config, &frame, params.code_phase_chips);
        let expected_chip_phase = advance_code_phase_seconds(
            params.code_phase_chips,
            config.code_freq_basis_hz,
            60.0,
            config.code_length,
        )
        .expect("valid theoretical phase");
        let expected_phase_samples =
            expected_chip_phase * config.sampling_freq_hz / config.code_freq_basis_hz;

        assert_phase_samples_close(actual_phase_samples, expected_phase_samples);

        let expected_code = sample_ca_code(
            Prn(params.sat.prn),
            config.sampling_freq_hz,
            config.code_freq_basis_hz,
            expected_chip_phase,
            code_period_samples,
        )
        .expect("valid expected sampled code");
        let amplitude = signal_amplitude_from_cn0(params.cn0_db_hz, config.sampling_freq_hz);

        for (index, (sample, expected_chip)) in
            frame.iq.iter().zip(expected_code.iter()).enumerate()
        {
            let expected_value = *expected_chip * amplitude;
            assert!(
                (sample.re - expected_value).abs() <= 1e-6,
                "I component drifted at sample {index}: actual={}, expected={expected_value}",
                sample.re
            );
            assert!(
                sample.im.abs() <= 1e-6,
                "Q component drifted at sample {index}: actual={}",
                sample.im
            );
        }
    }

    fn synthetic_epoch_frame(
        sat_state: &SatState,
        config: &ReceiverPipelineConfig,
        start_sample_index: u64,
        sample_count: usize,
    ) -> SamplesFrame {
        let dt_s = 1.0 / config.sampling_freq_hz;
        let iq = (0..sample_count)
            .map(|offset| sat_state.sample_at((start_sample_index + offset as u64) as f64 * dt_s))
            .collect::<Vec<_>>();
        SamplesFrame::new(
            SampleTime {
                sample_index: start_sample_index,
                sample_rate_hz: config.sampling_freq_hz,
            },
            Seconds(dt_s),
            iq,
        )
    }

    fn assert_phase_samples_close(actual: f64, expected: f64) {
        let delta = (actual - expected).abs();
        assert!(
            delta <= RECEIVER_PHASE_TOLERANCE_SAMPLES,
            "code phase samples drifted at sixty seconds: actual={actual:.12}, expected={expected:.12}, delta={delta:.12}"
        );
    }

    #[test]
    fn truth_bundle_records_constant_and_alternating_nav_bit_truth() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_000_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            ..ReceiverPipelineConfig::default()
        };
        let scenario = SyntheticScenario {
            sample_rate_hz: config.sampling_freq_hz,
            intermediate_freq_hz: config.intermediate_freq_hz,
            duration_s: 0.05,
            seed: 44,
            satellites: vec![
                SyntheticSignalParams {
                    sat: SatId { constellation: Constellation::Gps, prn: 3 },
                    doppler_hz: 500.0,
                    code_phase_chips: 200.0,
                    carrier_phase_rad: 0.0,
                    cn0_db_hz: 50.0,
                    data_bit_flip: false,
                },
                SyntheticSignalParams {
                    sat: SatId { constellation: Constellation::Gps, prn: 7 },
                    doppler_hz: -750.0,
                    code_phase_chips: 321.0,
                    carrier_phase_rad: 0.2,
                    cn0_db_hz: 45.0,
                    data_bit_flip: true,
                },
            ],
            ephemerides: Vec::new(),
            id: "truth-bundle".to_string(),
        };
        let frame = generate_l1_ca_multi(&config, &scenario);
        let metadata = RawIqMetadata {
            format: IqSampleFormat::Iq16Le,
            sample_rate_hz: config.sampling_freq_hz,
            intermediate_freq_hz: config.intermediate_freq_hz,
            capture_start_utc: "2026-07-09T00:00:00Z".to_string(),
            offset_bytes: 0,
            quantization_bits: Some(16),
            notes: Some("synthetic truth bundle".to_string()),
        };

        let truth = build_truth_bundle(&scenario.id, &scenario, &frame, &metadata, 1.25, 0.8);

        assert_eq!(truth.schema_version, 2);
        assert_eq!(truth.scenario_id, "truth-bundle");
        assert_eq!(truth.seed, 44);
        assert_eq!(truth.sample_format, IqSampleFormat::Iq16Le);
        assert_eq!(truth.sample_rate_hz, 4_000_000.0);
        assert_eq!(truth.quantization_bits, 16);
        assert_eq!(truth.noise_std_per_component, SYNTHETIC_NOISE_STD_PER_COMPONENT);
        assert_eq!(truth.noise_power_per_complex_sample, SYNTHETIC_COMPLEX_NOISE_POWER as f32);
        assert_eq!(truth.peak_component_before_scaling, 1.25);
        assert_eq!(truth.output_scale_applied, 0.8);
        assert_eq!(truth.satellites.len(), 2);

        let constant = &truth.satellites[0];
        assert_eq!(
            constant.signal_amplitude,
            signal_amplitude_from_cn0(constant.cn0_db_hz, truth.sample_rate_hz)
        );
        assert_eq!(constant.nav_bit_mode, SyntheticNavBitMode::ConstantPositive);
        assert_eq!(constant.nav_bit_segments.len(), 1);
        assert_eq!(constant.nav_bit_segments[0].start_sample, 0);
        assert_eq!(constant.nav_bit_segments[0].end_sample, frame.len() as u64);
        assert_eq!(constant.nav_bit_segments[0].bit, 1);

        let alternating = &truth.satellites[1];
        assert_eq!(
            alternating.signal_amplitude,
            signal_amplitude_from_cn0(alternating.cn0_db_hz, truth.sample_rate_hz)
        );
        assert!(constant.signal_amplitude > alternating.signal_amplitude);
        assert_eq!(alternating.nav_bit_mode, SyntheticNavBitMode::AlternatingGpsLnav20ms);
        assert_eq!(alternating.nav_bit_segments.len(), 3);
        assert_eq!(alternating.nav_bit_segments[0].start_sample, 0);
        assert_eq!(alternating.nav_bit_segments[0].end_sample, 80_000);
        assert_eq!(alternating.nav_bit_segments[0].bit, 1);
        assert_eq!(alternating.nav_bit_segments[1].start_sample, 80_000);
        assert_eq!(alternating.nav_bit_segments[1].end_sample, 160_000);
        assert_eq!(alternating.nav_bit_segments[1].bit, -1);
        assert_eq!(alternating.nav_bit_segments[2].start_sample, 160_000);
        assert_eq!(alternating.nav_bit_segments[2].end_sample, frame.len() as u64);
        assert_eq!(alternating.nav_bit_segments[2].bit, 1);
    }

    #[test]
    fn alternating_nav_bit_sign_flips_on_twenty_millisecond_boundaries() {
        assert_eq!(nav_bit_index_at_time_s(-1.0), 0);
        assert_eq!(nav_bit_index_at_time_s(0.0), 0);
        assert_eq!(nav_bit_index_at_time_s(0.019_999_999), 0);
        assert_eq!(nav_bit_index_at_time_s(0.020_000_000), 1);
        assert_eq!(nav_bit_index_at_time_s(0.039_999_999), 1);
        assert_eq!(nav_bit_index_at_time_s(0.040_000_000), 2);

        assert_eq!(nav_bit_sign_at_time_s(false, 0.0), 1);
        assert_eq!(nav_bit_sign_at_time_s(true, 0.0), 1);
        assert_eq!(nav_bit_sign_at_time_s(true, 0.019_999_999), 1);
        assert_eq!(nav_bit_sign_at_time_s(true, 0.020_000_000), -1);
        assert_eq!(nav_bit_sign_at_time_s(true, 0.039_999_999), -1);
        assert_eq!(nav_bit_sign_at_time_s(true, 0.040_000_000), 1);
    }

    #[test]
    fn iq16_capture_bundle_scales_without_clipping_and_preserves_truth() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            ..ReceiverPipelineConfig::default()
        };
        let scenario = SyntheticScenario {
            sample_rate_hz: config.sampling_freq_hz,
            intermediate_freq_hz: config.intermediate_freq_hz,
            duration_s: 0.01,
            seed: 91,
            satellites: vec![
                SyntheticSignalParams {
                    sat: SatId { constellation: Constellation::Gps, prn: 3 },
                    doppler_hz: 500.0,
                    code_phase_chips: 200.0,
                    carrier_phase_rad: 0.0,
                    cn0_db_hz: 58.0,
                    data_bit_flip: true,
                },
                SyntheticSignalParams {
                    sat: SatId { constellation: Constellation::Gps, prn: 7 },
                    doppler_hz: -1000.0,
                    code_phase_chips: 321.0,
                    carrier_phase_rad: 0.2,
                    cn0_db_hz: 56.0,
                    data_bit_flip: false,
                },
            ],
            ephemerides: Vec::new(),
            id: "iq16-bundle".to_string(),
        };
        let frame = generate_l1_ca_multi(&config, &scenario);

        let bundle = build_iq16_capture_bundle(
            &scenario.id,
            &scenario,
            &frame,
            "2026-07-09T00:00:00Z",
            Some("synthetic iq bundle".to_string()),
        );

        assert_eq!(bundle.metadata.format, IqSampleFormat::Iq16Le);
        assert_eq!(bundle.metadata.quantization_bits, Some(16));
        assert_eq!(bundle.truth.scenario_id, "iq16-bundle");
        assert_eq!(bundle.truth.sample_count, frame.len());
        assert_eq!(bundle.truth.sample_rate_hz, 4_092_000.0);
        assert_eq!(bundle.truth.noise_std_per_component, SYNTHETIC_NOISE_STD_PER_COMPONENT);
        assert_eq!(
            bundle.truth.noise_power_per_complex_sample,
            SYNTHETIC_COMPLEX_NOISE_POWER as f32
        );
        assert_eq!(bundle.raw_iq_bytes.len(), frame.len() * 4);
        assert!(bundle.truth.peak_component_before_scaling > 0.0);
        assert!(bundle.truth.output_scale_applied > 0.0);
        assert!(bundle.truth.output_scale_applied <= 1.0);
        assert!(
            bundle.truth.satellites[0].signal_amplitude
                > bundle.truth.satellites[1].signal_amplitude
        );

        assert!(
            bundle
                .raw_iq_bytes
                .chunks_exact(2)
                .map(|chunk| i16::from_le_bytes([chunk[0], chunk[1]]))
                .any(|sample| sample != 0),
            "encoded synthetic capture should contain non-zero samples"
        );
    }

    #[test]
    fn truth_guided_cn0_validation_matches_injected_truth_within_tolerance() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            ..ReceiverPipelineConfig::default()
        };
        let scenario = SyntheticScenario {
            sample_rate_hz: config.sampling_freq_hz,
            intermediate_freq_hz: config.intermediate_freq_hz,
            duration_s: 0.01,
            seed: 17,
            satellites: vec![SyntheticSignalParams {
                sat: SatId { constellation: Constellation::Gps, prn: 7 },
                doppler_hz: -1000.0,
                code_phase_chips: 321.0,
                carrier_phase_rad: 0.2,
                cn0_db_hz: 52.0,
                data_bit_flip: false,
            }],
            ephemerides: Vec::new(),
            id: "cn0-validation".to_string(),
        };
        let frame = generate_l1_ca_multi(&config, &scenario);
        let bundle = build_iq16_capture_bundle(
            &scenario.id,
            &scenario,
            &frame,
            "2026-07-09T00:00:00Z",
            Some("synthetic cn0 validation".to_string()),
        );
        let scaled_frame = SamplesFrame::new(
            frame.t0,
            frame.dt_s,
            frame.iq.iter().map(|sample| *sample * bundle.truth.output_scale_applied).collect(),
        );
        let report = validate_truth_guided_cn0(&config, &scaled_frame, &bundle.truth, 3.0);

        assert!(report.pass, "{report:?}");
        assert_eq!(report.coherent_samples_per_epoch, 4092);
        assert_eq!(report.satellites.len(), 1);
        for row in &report.satellites {
            assert!(row.pass, "{row:?}");
            assert_eq!(row.epochs_measured, 10);
            assert!(row.cn0_delta_db.abs() <= 3.0, "{row:?}");
            assert!(row.measured_max_cn0_dbhz >= row.measured_min_cn0_dbhz);
        }
    }

    #[test]
    fn expected_acquisition_code_phase_uses_receiver_search_convention() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            ..ReceiverPipelineConfig::default()
        };
        let period_samples =
            samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length);
        let frame = SamplesFrame::new(
            SampleTime { sample_index: 0, sample_rate_hz: config.sampling_freq_hz },
            Seconds(1.0 / config.sampling_freq_hz),
            vec![num_complex::Complex::new(0.0f32, 0.0f32); period_samples],
        );

        assert_eq!(expected_acquisition_code_phase_samples(&config, &frame, 0.0), 0);
        assert_eq!(
            expected_acquisition_code_phase_samples(
                &config,
                &frame,
                (period_samples - 1) as f64 * config.code_freq_basis_hz / config.sampling_freq_hz,
            ),
            1
        );
    }

    #[test]
    fn wrapped_code_phase_error_measures_shortest_period_distance() {
        assert_eq!(wrapped_code_phase_error_samples(0, 0, 4092), 0);
        assert_eq!(wrapped_code_phase_error_samples(1, 4091, 4092), 2);
        assert_eq!(wrapped_code_phase_error_samples(4091, 1, 4092), 2);
    }

    #[test]
    fn truth_guided_acquisition_code_phase_validation_matches_clean_truth_within_two_samples() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            ..ReceiverPipelineConfig::default()
        };
        let scenario = SyntheticScenario {
            sample_rate_hz: config.sampling_freq_hz,
            intermediate_freq_hz: config.intermediate_freq_hz,
            duration_s: 0.04,
            seed: 24071985,
            satellites: vec![
                SyntheticSignalParams {
                    sat: SatId { constellation: Constellation::Gps, prn: 3 },
                    doppler_hz: 750.0,
                    code_phase_chips: 200.25,
                    carrier_phase_rad: 0.0,
                    cn0_db_hz: 58.0,
                    data_bit_flip: true,
                },
                SyntheticSignalParams {
                    sat: SatId { constellation: Constellation::Gps, prn: 7 },
                    doppler_hz: -1000.0,
                    code_phase_chips: 321.5,
                    carrier_phase_rad: 0.2,
                    cn0_db_hz: 52.0,
                    data_bit_flip: false,
                },
            ],
            ephemerides: Vec::new(),
            id: "acquisition-code-phase-validation".to_string(),
        };
        let frame = generate_l1_ca_multi(&config, &scenario);
        let bundle = build_iq16_capture_bundle(
            &scenario.id,
            &scenario,
            &frame,
            "2026-07-09T00:00:00Z",
            Some("synthetic acquisition code-phase validation".to_string()),
        );
        let scaled_frame = SamplesFrame::new(
            frame.t0,
            frame.dt_s,
            frame.iq.iter().map(|sample| *sample * bundle.truth.output_scale_applied).collect(),
        );
        let report =
            validate_truth_guided_acquisition_code_phase(&config, &scaled_frame, &bundle.truth, 2);

        assert!(report.pass, "{report:?}");
        assert_eq!(report.period_samples, 4092);
        assert_eq!(report.satellites.len(), 2);
        for row in &report.satellites {
            assert!(row.pass, "{row:?}");
            assert!(row.code_phase_error_samples <= 2, "{row:?}");
            assert!(matches!(row.hypothesis.as_str(), "accepted" | "ambiguous"));
        }
    }
}
