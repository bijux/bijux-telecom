use bijux_gnss_core::api::{SamplesFrame, SatId};
use bijux_gnss_signal::api::{
    wipeoff_carrier, wipeoff_carrier_with_linear_rate, AcquisitionSignalModel, SignalError,
};
use num_complex::Complex;
use rustfft::{num_traits::Zero, FftPlanner};

use crate::engine::receiver_config::ReceiverPipelineConfig;

const CODE_PHASE_REFINEMENT_MAX_OFFSET_SAMPLES: f64 = 0.5;
const CODE_PHASE_REFINEMENT_EPSILON: f64 = 1.0e-12;
const CODE_PHASE_REFINEMENT_MIN_ABS_OFFSET_SAMPLES: f64 = 0.05;

pub(super) fn wipeoff_search_carrier(
    samples: &[Complex<f32>],
    carrier_hz: f64,
    doppler_rate_hz_per_s: f64,
    sample_rate_hz: f64,
    start_sample_index: u64,
    initial_phase_radians: f64,
) -> Result<Vec<Complex<f32>>, SignalError> {
    if doppler_rate_hz_per_s.abs() <= f64::EPSILON {
        return wipeoff_carrier(
            samples,
            carrier_hz,
            sample_rate_hz,
            start_sample_index,
            initial_phase_radians,
        );
    }

    wipeoff_carrier_with_linear_rate(
        samples,
        carrier_hz,
        doppler_rate_hz_per_s,
        sample_rate_hz,
        start_sample_index,
        initial_phase_radians,
    )
}

pub(super) fn measure_code_phase_profile(
    config: &ReceiverPipelineConfig,
    signal_model: &AcquisitionSignalModel,
    frame: &SamplesFrame,
    _sat: SatId,
    carrier_hz: f64,
    doppler_rate_hz_per_s: f64,
    coherent_ms: u32,
    noncoherent: u32,
) -> Option<Vec<f32>> {
    let samples_per_code = signal_model.samples_per_code(config.sampling_freq_hz);
    let coherent_periods = signal_model.coherent_periods(coherent_ms)?;
    if samples_per_code == 0 {
        return None;
    }
    let mut planner = FftPlanner::<f32>::new();
    let fft = planner.plan_fft_forward(samples_per_code);
    let ifft = planner.plan_fft_inverse(samples_per_code);
    let local_code = signal_model
        .sampled_local_code_period(config.sampling_freq_hz, samples_per_code)
        .unwrap_or_else(|_| vec![1.0; samples_per_code]);
    let mut code_fft: Vec<Complex<f32>> =
        local_code.iter().map(|&x| Complex::new(x, 0.0)).collect();
    fft.process(&mut code_fft);
    let mut noncoherent_acc = vec![0.0f32; samples_per_code];

    for nc in 0..noncoherent {
        let mut coherent_corr: Vec<Complex<f32>> = vec![Complex::zero(); samples_per_code];
        for c in 0..coherent_periods {
            let offset_period = (nc * coherent_periods + c) as usize;
            let start = offset_period * samples_per_code;
            let end = start + samples_per_code;
            let block = &frame.iq[start..end];
            let mixed = wipeoff_search_carrier(
                block,
                carrier_hz,
                doppler_rate_hz_per_s,
                config.sampling_freq_hz,
                start as u64,
                0.0,
            )
            .ok()?;
            let mut input_fft = mixed;
            fft.process(&mut input_fft);

            let mut prod = vec![Complex::zero(); samples_per_code];
            for i in 0..samples_per_code {
                prod[i] = input_fft[i] * code_fft[i].conj();
            }

            ifft.process(&mut prod);
            for i in 0..samples_per_code {
                coherent_corr[i] += prod[i];
            }
        }

        for i in 0..samples_per_code {
            noncoherent_acc[i] += coherent_corr[i].norm();
        }
    }

    Some(noncoherent_acc)
}

pub(super) fn estimate_parabolic_code_phase_offset_samples(
    correlation_profile: &[f32],
    coarse_code_phase_samples: usize,
) -> Option<(f64, f32, f32, f32)> {
    if correlation_profile.len() < 3 {
        return None;
    }
    let period_samples = correlation_profile.len();
    let center_index = coarse_code_phase_samples % period_samples;
    let left_index = (center_index + period_samples - 1) % period_samples;
    let right_index = (center_index + 1) % period_samples;
    let left = correlation_profile[left_index];
    let center = correlation_profile[center_index];
    let right = correlation_profile[right_index];

    if center < left || center < right {
        return None;
    }

    let denominator = left as f64 - (2.0 * center as f64) + right as f64;
    if !denominator.is_finite() || denominator.abs() <= CODE_PHASE_REFINEMENT_EPSILON {
        return None;
    }

    let raw_offset_samples = 0.5 * (left as f64 - right as f64) / denominator;
    if !raw_offset_samples.is_finite() {
        return None;
    }

    let offset_samples = raw_offset_samples
        .clamp(-CODE_PHASE_REFINEMENT_MAX_OFFSET_SAMPLES, CODE_PHASE_REFINEMENT_MAX_OFFSET_SAMPLES);
    if offset_samples.abs() < CODE_PHASE_REFINEMENT_MIN_ABS_OFFSET_SAMPLES {
        return None;
    }

    Some((offset_samples, left, center, right))
}

pub(super) fn wrap_acquisition_code_phase_samples(
    code_phase_samples: f64,
    period_samples: usize,
) -> f64 {
    let period_samples = period_samples.max(1) as f64;
    code_phase_samples.rem_euclid(period_samples)
}
