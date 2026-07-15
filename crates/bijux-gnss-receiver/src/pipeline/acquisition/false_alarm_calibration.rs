use std::f64::consts::TAU;

use bijux_gnss_core::api::{AcqRequest, SampleTime, SamplesFrame, Seconds};
use bijux_gnss_signal::api::AcquisitionSignalModel;
use num_complex::Complex;

use super::signal_model::resolved_request_signal_code;

#[derive(Debug, Clone, Copy)]
pub(super) struct FalseAlarmRateMeasurement {
    pub(super) false_alarm_rate: f64,
    pub(super) confidence_interval_low: f64,
    pub(super) confidence_interval_high: f64,
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
        let u1 = self.next_f32().max(1.0e-12);
        let u2 = self.next_f32();
        let r = (-2.0 * u1.ln()).sqrt();
        let theta = (TAU as f32) * u2;
        r * theta.cos()
    }
}

pub(super) fn calibration_seed(
    request: AcqRequest,
    signal_model: &AcquisitionSignalModel,
    peak_mean_threshold: f32,
) -> u64 {
    let mut seed = 0xA11C_9A7E_4D35_2B61_u64;
    seed = mix_seed(seed, request.sat.prn as u64);
    seed = mix_seed(seed, request.sat.constellation as u64);
    seed = mix_seed(seed, request.signal_band as u64);
    seed = mix_seed(seed, resolved_request_signal_code(request) as u64);
    seed = mix_seed(seed, request.coherent_ms as u64);
    seed = mix_seed(seed, request.noncoherent as u64);
    seed = mix_seed(seed, request.doppler_search_hz as i64 as u64);
    seed = mix_seed(seed, request.doppler_step_hz.max(1) as i64 as u64);
    seed = mix_seed(seed, signal_model.code_length as u64);
    seed = mix_seed(seed, signal_model.code_rate_hz.to_bits());
    seed = mix_seed(seed, peak_mean_threshold.to_bits() as u64);
    if let Some(channel) = request.glonass_frequency_channel {
        seed = mix_seed(seed, channel.value() as i64 as u64);
    }
    seed
}

pub(super) fn mix_seed(seed: u64, value: u64) -> u64 {
    let mixed = seed ^ value.wrapping_add(0x9E37_79B9_7F4A_7C15);
    mixed.rotate_left(27).wrapping_mul(0x94D0_49BB_1331_11EB)
}

pub(super) fn noise_only_frame(
    sampling_freq_hz: f64,
    sample_count: usize,
    seed: u64,
) -> SamplesFrame {
    let mut rng = XorShift64::new(seed);
    let iq = (0..sample_count)
        .map(|_| Complex::new(rng.next_gaussian(), rng.next_gaussian()))
        .collect::<Vec<_>>();
    SamplesFrame::new(
        SampleTime { sample_index: 0, sample_rate_hz: sampling_freq_hz },
        Seconds(1.0 / sampling_freq_hz),
        iq,
    )
}

pub(super) fn false_alarm_rate(false_alarm_count: usize, trial_count: usize) -> f64 {
    if trial_count == 0 {
        return 0.0;
    }
    false_alarm_count as f64 / trial_count as f64
}

pub(super) fn wilson_confidence_interval(
    false_alarm_count: usize,
    trial_count: usize,
    confidence_level: f64,
) -> (f64, f64) {
    if trial_count == 0 {
        return (0.0, 0.0);
    }
    let p_hat = false_alarm_rate(false_alarm_count, trial_count);
    let z = inverse_standard_normal_cdf(0.5 + (confidence_level * 0.5));
    let n = trial_count as f64;
    let z2 = z * z;
    let denominator = 1.0 + (z2 / n);
    let center = (p_hat + (z2 / (2.0 * n))) / denominator;
    let margin = (z * ((p_hat * (1.0 - p_hat) / n) + (z2 / (4.0 * n * n))).sqrt()) / denominator;
    ((center - margin).max(0.0), (center + margin).min(1.0))
}

fn inverse_standard_normal_cdf(p: f64) -> f64 {
    debug_assert!(p > 0.0 && p < 1.0);
    const A: [f64; 6] = [
        -3.969_683_028_665_376e1,
        2.209_460_984_245_205e2,
        -2.759_285_104_469_687e2,
        1.383_577_518_672_69e2,
        -3.066_479_806_614_716e1,
        2.506_628_277_459_239,
    ];
    const B: [f64; 5] = [
        -5.447_609_879_822_406e1,
        1.615_858_368_580_409e2,
        -1.556_989_798_598_866e2,
        6.680_131_188_771_972e1,
        -1.328_068_155_288_572e1,
    ];
    const C: [f64; 6] = [
        -7.784_894_002_430_293e-3,
        -3.223_964_580_411_365e-1,
        -2.400_758_277_161_838,
        -2.549_732_539_343_734,
        4.374_664_141_464_968,
        2.938_163_982_698_783,
    ];
    const D: [f64; 4] = [
        7.784_695_709_041_462e-3,
        3.224_671_290_700_398e-1,
        2.445_134_137_142_996,
        3.754_408_661_907_416,
    ];
    const LOW: f64 = 0.02425;
    const HIGH: f64 = 1.0 - LOW;

    if p < LOW {
        let q = (-2.0 * p.ln()).sqrt();
        return (((((C[0] * q + C[1]) * q + C[2]) * q + C[3]) * q + C[4]) * q + C[5])
            / ((((D[0] * q + D[1]) * q + D[2]) * q + D[3]) * q + 1.0);
    }
    if p > HIGH {
        let q = (-2.0 * (1.0 - p).ln()).sqrt();
        return -(((((C[0] * q + C[1]) * q + C[2]) * q + C[3]) * q + C[4]) * q + C[5])
            / ((((D[0] * q + D[1]) * q + D[2]) * q + D[3]) * q + 1.0);
    }
    let q = p - 0.5;
    let r = q * q;
    (((((A[0] * r + A[1]) * r + A[2]) * r + A[3]) * r + A[4]) * r + A[5]) * q
        / (((((B[0] * r + B[1]) * r + B[2]) * r + B[3]) * r + B[4]) * r + 1.0)
}
