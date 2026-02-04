#![allow(missing_docs)]

use std::f32::consts::TAU;

use num_complex::Complex;

use bijux_gnss_core::api::{Constellation, SampleClock, SampleTime, SamplesFrame, SatId, Seconds};

use crate::engine::receiver_config::ReceiverRuntimeConfig;
use bijux_gnss_nav::api::GpsEphemeris;
use bijux_gnss_signal::api::{generate_ca_code, Prn};
use serde::{Deserialize, Serialize};

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

/// Generate a synthetic GPS L1 C/A signal at the receiver sample rate.
///
/// The C/N0 control is approximate and intended for test harnesses.
pub fn generate_l1_ca(
    config: &ReceiverRuntimeConfig,
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

pub fn generate_l1_ca_multi(config: &ReceiverRuntimeConfig, scenario: &SyntheticScenario) -> SamplesFrame {
    let clock = SampleClock::new(config.sampling_freq_hz);
    let dt_s = clock.dt_s();
    let sample_count = (scenario.duration_s * config.sampling_freq_hz).round() as usize;

    let max_cn0 = scenario
        .satellites
        .iter()
        .map(|s| s.cn0_db_hz)
        .fold(0.0_f32, f32::max);
    let snr_db = max_cn0 - 30.0;
    let snr_linear = 10.0f32.powf(snr_db / 10.0).max(1e-6);
    let noise_std = 1.0f32 / snr_linear.sqrt();

    let mut rng = XorShift64::new(scenario.seed);
    let mut iq = Vec::with_capacity(sample_count);

    let sat_states: Vec<SatState> = scenario
        .satellites
        .iter()
        .map(|sat| SatState::new(config, *sat))
        .collect();

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

    let t0 = SampleTime {
        sample_index: 0,
        sample_rate_hz: config.sampling_freq_hz,
    };

    SamplesFrame::new(t0, Seconds(dt_s), iq)
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
    fn new(config: &ReceiverRuntimeConfig, params: SyntheticSignalParams) -> Self {
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
