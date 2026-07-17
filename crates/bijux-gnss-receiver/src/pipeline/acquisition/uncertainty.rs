use bijux_gnss_core::api::{AcqHypothesis, AcqResult, AcqUncertainty, SamplesFrame};
use bijux_gnss_signal::api::AcquisitionSignalModel;

use crate::engine::receiver_config::ReceiverPipelineConfig;

use super::likelihood_covariance::{
    estimate_log_likelihood_covariance_2x2, estimate_log_likelihood_covariance_3x3,
    estimate_log_likelihood_covariance_from_refinement_axes,
};
use super::likelihood_measurement::{
    measure_local_acquisition_likelihood_surface, measure_local_acquisition_likelihood_volume,
    AcquisitionLikelihoodSurfaceRequest, AcquisitionLikelihoodVolumeRequest,
};

pub(super) fn estimate_acquisition_uncertainty(
    config: &ReceiverPipelineConfig,
    frame: &SamplesFrame,
    signal_model: &AcquisitionSignalModel,
    candidate: &AcqResult,
    coherent_ms: u32,
    noncoherent: u32,
    doppler_step_hz: i32,
    doppler_rate_search_hz_per_s: i32,
    doppler_rate_step_hz_per_s: i32,
) -> Option<AcqUncertainty> {
    if !matches!(candidate.hypothesis, AcqHypothesis::Accepted) {
        return None;
    }
    let code_phase_center_samples = candidate.resolved_code_phase_samples().round() as usize;
    let covariance = if doppler_rate_search_hz_per_s > 0 {
        measure_local_acquisition_likelihood_volume(AcquisitionLikelihoodVolumeRequest {
            surface: AcquisitionLikelihoodSurfaceRequest {
                config,
                signal_model,
                frame,
                coarse_carrier_hz: candidate.carrier_hz.0,
                coarse_doppler_rate_hz_per_s: candidate.doppler_rate_hz_per_s,
                coarse_code_phase_samples: code_phase_center_samples,
                doppler_step_hz: doppler_step_hz as f64,
                coherent_ms,
                noncoherent,
            },
            doppler_rate_step_hz_per_s: doppler_rate_step_hz_per_s as f64,
        })
        .and_then(|volume| {
            estimate_log_likelihood_covariance_3x3(
                &volume,
                candidate.mean as f64,
                doppler_step_hz as f64,
                1.0,
                doppler_rate_step_hz_per_s as f64,
            )
        })
    } else {
        None
    }
    .or_else(|| {
        measure_local_acquisition_likelihood_surface(AcquisitionLikelihoodSurfaceRequest {
            config,
            signal_model,
            frame,
            coarse_carrier_hz: candidate.carrier_hz.0,
            coarse_doppler_rate_hz_per_s: candidate.doppler_rate_hz_per_s,
            coarse_code_phase_samples: code_phase_center_samples,
            doppler_step_hz: doppler_step_hz as f64,
            coherent_ms,
            noncoherent,
        })
        .and_then(|surface| {
            estimate_log_likelihood_covariance_2x2(
                &surface,
                candidate.mean as f64,
                doppler_step_hz as f64,
                1.0,
            )
        })
    })
    .or_else(|| {
        estimate_log_likelihood_covariance_from_refinement_axes(
            candidate,
            candidate.mean as f64,
            doppler_step_hz as f64,
        )
    })?;

    let doppler_hz = covariance.doppler_variance_hz2.sqrt();
    let code_phase_samples = covariance.code_phase_variance_samples2.sqrt();
    let doppler_rate_hz_per_s = covariance.doppler_rate_variance_hz2_per_s2.map(f64::sqrt);
    if !doppler_hz.is_finite()
        || !code_phase_samples.is_finite()
        || doppler_hz <= f64::EPSILON
        || code_phase_samples <= f64::EPSILON
        || doppler_rate_hz_per_s
            .is_some_and(|rate_sigma| !rate_sigma.is_finite() || rate_sigma <= f64::EPSILON)
    {
        return None;
    }

    Some(AcqUncertainty {
        doppler_hz,
        code_phase_samples,
        doppler_rate_hz_per_s,
        covariance: Some(covariance),
    })
}
