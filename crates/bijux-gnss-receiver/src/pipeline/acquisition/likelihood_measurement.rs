use bijux_gnss_core::api::SamplesFrame;
use bijux_gnss_signal::api::AcquisitionSignalModel;

use crate::engine::receiver_config::ReceiverPipelineConfig;

use super::code_phase_profile::{measure_code_phase_profile, CodePhaseProfileRequest};
use super::likelihood_covariance::{
    LocalAcquisitionLikelihoodSurface, LocalAcquisitionLikelihoodVolume,
};

const DOPPLER_OFFSET_LIMIT_BINS: f64 = 0.5;
const CODE_PHASE_OFFSET_LIMIT_SAMPLES: f64 = 0.5;
const LIKELIHOOD_CURVATURE_EPSILON: f64 = 1.0e-12;

#[derive(Clone, Copy)]
pub(super) struct AcquisitionLikelihoodSurfaceRequest<'a> {
    pub(super) config: &'a ReceiverPipelineConfig,
    pub(super) signal_model: &'a AcquisitionSignalModel,
    pub(super) frame: &'a SamplesFrame,
    pub(super) coarse_carrier_hz: f64,
    pub(super) coarse_doppler_rate_hz_per_s: f64,
    pub(super) coarse_code_phase_samples: usize,
    pub(super) doppler_step_hz: f64,
    pub(super) coherent_ms: u32,
    pub(super) noncoherent: u32,
}

#[derive(Clone, Copy)]
pub(super) struct AcquisitionLikelihoodVolumeRequest<'a> {
    pub(super) surface: AcquisitionLikelihoodSurfaceRequest<'a>,
    pub(super) doppler_rate_step_hz_per_s: f64,
}

pub(super) fn measure_local_acquisition_likelihood_surface(
    request: AcquisitionLikelihoodSurfaceRequest<'_>,
) -> Option<LocalAcquisitionLikelihoodSurface> {
    if !request.doppler_step_hz.is_finite() || request.doppler_step_hz <= f64::EPSILON {
        return None;
    }
    let samples_per_code = request.signal_model.samples_per_code(request.config.sampling_freq_hz);
    if samples_per_code < 3 {
        return None;
    }

    let center_index = request.coarse_code_phase_samples % samples_per_code;
    let left_index = (center_index + samples_per_code - 1) % samples_per_code;
    let right_index = (center_index + 1) % samples_per_code;
    let mut values = [[0.0f32; 3]; 3];

    for (doppler_index, carrier_hz) in [
        request.coarse_carrier_hz - request.doppler_step_hz,
        request.coarse_carrier_hz,
        request.coarse_carrier_hz + request.doppler_step_hz,
    ]
    .into_iter()
    .enumerate()
    {
        let profile = measure_code_phase_profile(CodePhaseProfileRequest {
            config: request.config,
            signal_model: request.signal_model,
            frame: request.frame,
            carrier_hz,
            doppler_rate_hz_per_s: request.coarse_doppler_rate_hz_per_s,
            coherent_ms: request.coherent_ms,
            noncoherent: request.noncoherent,
        })?;
        values[doppler_index] = [profile[left_index], profile[center_index], profile[right_index]];
    }

    let center = values[1][1];
    if center < values[0][1]
        || center < values[2][1]
        || center < values[1][0]
        || center < values[1][2]
    {
        return None;
    }

    Some(LocalAcquisitionLikelihoodSurface {
        doppler_cross_section: [values[0][1], values[1][1], values[2][1]],
        code_phase_cross_section: values[1],
        values,
    })
}

pub(super) fn measure_local_acquisition_likelihood_volume(
    request: AcquisitionLikelihoodVolumeRequest<'_>,
) -> Option<LocalAcquisitionLikelihoodVolume> {
    if !request.surface.doppler_step_hz.is_finite()
        || request.surface.doppler_step_hz <= f64::EPSILON
        || !request.doppler_rate_step_hz_per_s.is_finite()
        || request.doppler_rate_step_hz_per_s <= f64::EPSILON
    {
        return None;
    }
    let samples_per_code =
        request.surface.signal_model.samples_per_code(request.surface.config.sampling_freq_hz);
    if samples_per_code < 3 {
        return None;
    }

    let center_index = request.surface.coarse_code_phase_samples % samples_per_code;
    let left_index = (center_index + samples_per_code - 1) % samples_per_code;
    let right_index = (center_index + 1) % samples_per_code;
    let mut values = [[[0.0f32; 3]; 3]; 3];
    let doppler_rates = [
        request.surface.coarse_doppler_rate_hz_per_s - request.doppler_rate_step_hz_per_s,
        request.surface.coarse_doppler_rate_hz_per_s,
        request.surface.coarse_doppler_rate_hz_per_s + request.doppler_rate_step_hz_per_s,
    ];
    let carriers = [
        request.surface.coarse_carrier_hz - request.surface.doppler_step_hz,
        request.surface.coarse_carrier_hz,
        request.surface.coarse_carrier_hz + request.surface.doppler_step_hz,
    ];

    for (rate_index, doppler_rate_hz_per_s) in doppler_rates.into_iter().enumerate() {
        for (doppler_index, carrier_hz) in carriers.into_iter().enumerate() {
            let profile = measure_code_phase_profile(CodePhaseProfileRequest {
                config: request.surface.config,
                signal_model: request.surface.signal_model,
                frame: request.surface.frame,
                carrier_hz,
                doppler_rate_hz_per_s,
                coherent_ms: request.surface.coherent_ms,
                noncoherent: request.surface.noncoherent,
            })?;
            values[rate_index][doppler_index] =
                [profile[left_index], profile[center_index], profile[right_index]];
        }
    }

    let center = values[1][1][1];
    let center_neighbors = [
        values[1][0][1],
        values[1][2][1],
        values[1][1][0],
        values[1][1][2],
        values[0][1][1],
        values[2][1][1],
    ];
    if center_neighbors.into_iter().any(|neighbor| center < neighbor) {
        return None;
    }

    Some(LocalAcquisitionLikelihoodVolume { values })
}

pub(super) fn estimate_quadratic_surface_peak_offsets(
    surface: &LocalAcquisitionLikelihoodSurface,
) -> Option<(f64, f64)> {
    let values = &surface.values;
    if values[1][1] < values[0][1]
        || values[1][1] < values[2][1]
        || values[1][1] < values[1][0]
        || values[1][1] < values[1][2]
    {
        return None;
    }
    let center = values[1][1] as f64;
    let doppler_curvature = 0.5 * (values[2][1] as f64 + values[0][1] as f64 - (2.0 * center));
    let code_curvature = 0.5 * (values[1][2] as f64 + values[1][0] as f64 - (2.0 * center));
    let cross_curvature = 0.25
        * (values[2][2] as f64 - values[2][0] as f64 - values[0][2] as f64 + values[0][0] as f64);
    let doppler_slope = 0.5 * (values[2][1] as f64 - values[0][1] as f64);
    let code_slope = 0.5 * (values[1][2] as f64 - values[1][0] as f64);

    if !doppler_curvature.is_finite()
        || !code_curvature.is_finite()
        || !cross_curvature.is_finite()
        || !doppler_slope.is_finite()
        || !code_slope.is_finite()
        || doppler_curvature >= -LIKELIHOOD_CURVATURE_EPSILON
        || code_curvature >= -LIKELIHOOD_CURVATURE_EPSILON
    {
        return None;
    }

    let determinant = (4.0 * doppler_curvature * code_curvature) - cross_curvature.powi(2);
    if !determinant.is_finite() || determinant <= LIKELIHOOD_CURVATURE_EPSILON {
        return None;
    }

    let doppler_offset_bins =
        ((cross_curvature * code_slope) - (2.0 * code_curvature * doppler_slope)) / determinant;
    let code_phase_offset_samples =
        ((cross_curvature * doppler_slope) - (2.0 * doppler_curvature * code_slope)) / determinant;
    if !doppler_offset_bins.is_finite() || !code_phase_offset_samples.is_finite() {
        return None;
    }

    Some((
        doppler_offset_bins.clamp(-DOPPLER_OFFSET_LIMIT_BINS, DOPPLER_OFFSET_LIMIT_BINS),
        code_phase_offset_samples
            .clamp(-CODE_PHASE_OFFSET_LIMIT_SAMPLES, CODE_PHASE_OFFSET_LIMIT_SAMPLES),
    ))
}
