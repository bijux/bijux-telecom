use bijux_gnss_core::api::{AcqCodePhaseRefinement, SamplesFrame};
use bijux_gnss_signal::api::AcquisitionSignalModel;

use super::code_phase_profile::{
    estimate_parabolic_code_phase_offset_samples, measure_code_phase_profile,
    wrap_acquisition_code_phase_samples, CodePhaseProfileRequest,
};
use super::likelihood_measurement::{
    estimate_quadratic_surface_peak_offsets, measure_local_acquisition_likelihood_surface,
    AcquisitionLikelihoodSurfaceRequest,
};
use super::Acquisition;

const SUB_SAMPLE_CODE_PHASE_REFINEMENT_METHOD: &str = "parabolic_code_peak";
const SUB_SAMPLE_CODE_PHASE_REFINEMENT_MIN_ABS_OFFSET_SAMPLES: f64 = 0.05;

#[derive(Debug, Clone, Copy)]
pub(super) struct JointAcquisitionRefinement {
    pub(super) doppler_offset_bins: f64,
    pub(super) code_phase_offset_samples: f64,
    pub(super) refined_code_phase_samples: f64,
    pub(super) doppler_cross_section: [f32; 3],
    pub(super) code_phase_cross_section: [f32; 3],
}

#[derive(Clone, Copy)]
pub(super) struct JointAcquisitionRefinementRequest<'a> {
    pub(super) frame: &'a SamplesFrame,
    pub(super) signal_model: &'a AcquisitionSignalModel,
    pub(super) coarse_carrier_hz: f64,
    pub(super) coarse_doppler_rate_hz_per_s: f64,
    pub(super) coarse_code_phase_samples: usize,
    pub(super) doppler_step_hz: i32,
    pub(super) coherent_ms: u32,
    pub(super) noncoherent: u32,
}

#[derive(Clone, Copy)]
pub(super) struct CodePhaseRefinementRequest<'a> {
    pub(super) frame: &'a SamplesFrame,
    pub(super) signal_model: &'a AcquisitionSignalModel,
    pub(super) carrier_hz: f64,
    pub(super) doppler_rate_hz_per_s: f64,
    pub(super) coarse_code_phase_samples: usize,
    pub(super) coherent_ms: u32,
    pub(super) noncoherent: u32,
}

impl Acquisition {
    pub(super) fn estimate_joint_acquisition_refinement(
        &self,
        request: JointAcquisitionRefinementRequest<'_>,
    ) -> Option<JointAcquisitionRefinement> {
        if request.doppler_step_hz <= 0 {
            return None;
        }
        let samples_per_code = request.signal_model.samples_per_code(self.config.sampling_freq_hz);
        let coherent_periods = request.signal_model.coherent_periods(request.coherent_ms)?;
        if samples_per_code == 0
            || request.frame.len()
                < samples_per_code
                    * coherent_periods.max(1) as usize
                    * request.noncoherent.max(1) as usize
        {
            return None;
        }

        let surface =
            measure_local_acquisition_likelihood_surface(AcquisitionLikelihoodSurfaceRequest {
                config: &self.config,
                signal_model: request.signal_model,
                frame: request.frame,
                coarse_carrier_hz: request.coarse_carrier_hz,
                coarse_doppler_rate_hz_per_s: request.coarse_doppler_rate_hz_per_s,
                coarse_code_phase_samples: request.coarse_code_phase_samples,
                doppler_step_hz: request.doppler_step_hz as f64,
                coherent_ms: request.coherent_ms,
                noncoherent: request.noncoherent,
            })?;
        let (doppler_offset_bins, code_phase_offset_samples) =
            estimate_quadratic_surface_peak_offsets(&surface)?;
        if doppler_offset_bins.abs() < f64::EPSILON
            && code_phase_offset_samples.abs()
                < SUB_SAMPLE_CODE_PHASE_REFINEMENT_MIN_ABS_OFFSET_SAMPLES
        {
            return None;
        }

        Some(JointAcquisitionRefinement {
            doppler_offset_bins,
            code_phase_offset_samples,
            refined_code_phase_samples: wrap_acquisition_code_phase_samples(
                request.coarse_code_phase_samples as f64 + code_phase_offset_samples,
                samples_per_code,
            ),
            doppler_cross_section: surface.doppler_cross_section,
            code_phase_cross_section: surface.code_phase_cross_section,
        })
    }

    pub(super) fn estimate_acquisition_code_phase_refinement(
        &self,
        request: CodePhaseRefinementRequest<'_>,
    ) -> Option<AcqCodePhaseRefinement> {
        let samples_per_code = request.signal_model.samples_per_code(self.config.sampling_freq_hz);
        let coherent_periods = request.signal_model.coherent_periods(request.coherent_ms)?;
        if samples_per_code == 0
            || request.frame.len()
                < samples_per_code
                    * coherent_periods.max(1) as usize
                    * request.noncoherent.max(1) as usize
        {
            return None;
        }
        let correlation_profile = measure_code_phase_profile(CodePhaseProfileRequest {
            config: &self.config,
            signal_model: request.signal_model,
            frame: request.frame,
            carrier_hz: request.carrier_hz,
            doppler_rate_hz_per_s: request.doppler_rate_hz_per_s,
            coherent_ms: request.coherent_ms,
            noncoherent: request.noncoherent,
        })?;
        let (
            offset_samples,
            left_correlation_norm,
            center_correlation_norm,
            right_correlation_norm,
        ) = estimate_parabolic_code_phase_offset_samples(
            &correlation_profile,
            request.coarse_code_phase_samples,
        )?;
        let refined_code_phase_samples = wrap_acquisition_code_phase_samples(
            request.coarse_code_phase_samples as f64 + offset_samples,
            samples_per_code,
        );

        Some(AcqCodePhaseRefinement {
            method: SUB_SAMPLE_CODE_PHASE_REFINEMENT_METHOD.to_string(),
            offset_samples,
            refined_code_phase_samples,
            left_correlation_norm,
            center_correlation_norm,
            right_correlation_norm,
        })
    }
}
