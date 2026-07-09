#![allow(missing_docs)]

use std::f32::consts::TAU;

use num_complex::Complex;

use bijux_gnss_core::api::{Constellation, SampleClock, SampleTime, SamplesFrame, SatId, Seconds};
use bijux_gnss_signal::api::SignalSource;

use crate::engine::receiver_config::ReceiverPipelineConfig;
use crate::io::data::SampleSourceError;
use bijux_gnss_nav::api::GpsEphemeris;
use bijux_gnss_signal::api::{generate_ca_code, IqSampleFormat, Prn, RawIqMetadata};
use serde::{Deserialize, Serialize};

const SYNTHETIC_IQ_TRUTH_SCHEMA_VERSION: u32 = 1;
const GPS_L1_CA_NAV_BIT_PERIOD_S: f64 = 0.02;

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
    /// Peak absolute I/Q component before output scaling.
    pub peak_component_before_scaling: f32,
    /// Scale factor applied before quantization.
    pub output_scale_applied: f32,
    /// Per-satellite truth rows.
    pub satellites: Vec<SyntheticSatelliteTruth>,
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
    let clock = SampleClock::new(config.sampling_freq_hz);
    let dt_s = clock.dt_s();
    let sample_count = (scenario.duration_s * config.sampling_freq_hz).round() as usize;

    let max_cn0 = scenario.satellites.iter().map(|s| s.cn0_db_hz).fold(0.0_f32, f32::max);
    let snr_db = max_cn0 - 30.0;
    let snr_linear = 10.0f32.powf(snr_db / 10.0).max(1e-6);
    let noise_std = 1.0f32 / snr_linear.sqrt();

    let mut rng = XorShift64::new(scenario.seed);
    let mut iq = Vec::with_capacity(sample_count);

    let sat_states: Vec<SatState> =
        scenario.satellites.iter().map(|sat| SatState::new(config, *sat)).collect();

    for n in 0..sample_count {
        let t = n as f64 * dt_s;
        let mut sample = Complex::new(0.0f32, 0.0f32);
        for sat in &sat_states {
            sample += sat.sample_at(t);
        }

        let noise_i = rng.next_gaussian() * noise_std;
        let noise_q = rng.next_gaussian() * noise_std;
        let noise = Complex::new(noise_i, noise_q);

        iq.push(sample + noise);
    }

    let t0 = SampleTime { sample_index: 0, sample_rate_hz: config.sampling_freq_hz };

    SamplesFrame::new(t0, Seconds(dt_s), iq)
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

impl SyntheticSignalSource {
    /// Build a streaming synthetic source from a scenario without materializing the full capture.
    pub fn new(config: &ReceiverPipelineConfig, scenario: &SyntheticScenario) -> Self {
        let sample_count = (scenario.duration_s * config.sampling_freq_hz).round() as usize;
        let max_cn0 = scenario.satellites.iter().map(|s| s.cn0_db_hz).fold(0.0_f32, f32::max);
        let snr_db = max_cn0 - 30.0;
        let snr_linear = 10.0f32.powf(snr_db / 10.0).max(1e-6);
        let noise_std = 1.0f32 / snr_linear.sqrt();

        Self {
            sample_rate_hz: config.sampling_freq_hz,
            dt_s: 1.0 / config.sampling_freq_hz,
            remaining_samples: sample_count,
            next_sample_index: 0,
            noise_std,
            sat_states: scenario
                .satellites
                .iter()
                .map(|sat| SatState::new(config, *sat))
                .collect(),
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
}

impl SatState {
    fn new(config: &ReceiverPipelineConfig, params: SyntheticSignalParams) -> Self {
        let carrier = match params.sat.constellation {
            Constellation::Galileo => bijux_gnss_core::api::GALILEO_E1_CARRIER_HZ.value(),
            Constellation::Gps => bijux_gnss_core::api::GPS_L1_CA_CARRIER_HZ.value(),
            Constellation::Glonass => bijux_gnss_core::api::GLONASS_L1_CARRIER_HZ.value(),
            _ => bijux_gnss_core::api::GPS_L1_CA_CARRIER_HZ.value(),
        };
        Self {
            doppler_hz: params.doppler_hz,
            code_phase_chips: params.code_phase_chips,
            carrier_phase_rad: params.carrier_phase_rad,
            cn0_db_hz: params.cn0_db_hz,
            data_bit_flip: params.data_bit_flip,
            code: generate_ca_code(Prn(params.sat.prn)).unwrap_or_else(|_| vec![1; 1023]),
            code_rate_hz: config.code_freq_basis_hz,
            if_hz: config.intermediate_freq_hz
                + (carrier - bijux_gnss_core::api::GPS_L1_CA_CARRIER_HZ.value()),
        }
    }

    fn sample_at(&self, t: f64) -> Complex<f32> {
        let code_phase = self.code_phase_chips + self.code_rate_hz * t;
        let chip_index = (code_phase.floor() as usize) % self.code.len();
        let chip = self.code[chip_index] as f32;

        let data_bit = if self.data_bit_flip {
            let bit_index = (t / 0.02).floor() as i64;
            if bit_index % 2 == 0 {
                1.0
            } else {
                -1.0
            }
        } else {
            1.0
        };

        let carrier_hz = self.if_hz + self.doppler_hz;
        let phase = self.carrier_phase_rad as f32 + TAU * (carrier_hz as f32) * (t as f32);
        let carrier = Complex::new(phase.cos(), phase.sin());

        let snr_db = self.cn0_db_hz - 30.0;
        let snr_linear = 10.0f32.powf(snr_db / 10.0).max(1e-6);
        let amplitude = snr_linear.sqrt();

        carrier * (chip * data_bit * amplitude)
    }
}

fn nav_bit_mode(params: &SyntheticSignalParams) -> SyntheticNavBitMode {
    if params.data_bit_flip {
        SyntheticNavBitMode::AlternatingGpsLnav20ms
    } else {
        SyntheticNavBitMode::ConstantPositive
    }
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
            bit: if bit_index % 2 == 0 { 1 } else { -1 },
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
        build_truth_bundle, generate_l1_ca_multi, SyntheticNavBitMode, SyntheticScenario,
        SyntheticSignalParams, SyntheticSignalSource,
    };
    use crate::engine::receiver_config::ReceiverPipelineConfig;
    use bijux_gnss_core::api::{Constellation, SatId, SamplesFrame};
    use bijux_gnss_signal::api::{IqSampleFormat, RawIqMetadata, SignalSource};

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

        assert_eq!(truth.schema_version, 1);
        assert_eq!(truth.scenario_id, "truth-bundle");
        assert_eq!(truth.seed, 44);
        assert_eq!(truth.sample_format, IqSampleFormat::Iq16Le);
        assert_eq!(truth.sample_rate_hz, 4_000_000.0);
        assert_eq!(truth.quantization_bits, 16);
        assert_eq!(truth.peak_component_before_scaling, 1.25);
        assert_eq!(truth.output_scale_applied, 0.8);
        assert_eq!(truth.satellites.len(), 2);

        let constant = &truth.satellites[0];
        assert_eq!(constant.nav_bit_mode, SyntheticNavBitMode::ConstantPositive);
        assert_eq!(constant.nav_bit_segments.len(), 1);
        assert_eq!(constant.nav_bit_segments[0].start_sample, 0);
        assert_eq!(constant.nav_bit_segments[0].end_sample, frame.len() as u64);
        assert_eq!(constant.nav_bit_segments[0].bit, 1);

        let alternating = &truth.satellites[1];
        assert_eq!(
            alternating.nav_bit_mode,
            SyntheticNavBitMode::AlternatingGpsLnav20ms
        );
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
}
