use bijux_gnss_core::api::{
    AcqCodePhaseRefinement, AcqDopplerRefinement, AcqResult, Hertz, SamplesFrame,
};
use bijux_gnss_signal::api::AcquisitionSignalModel;

use crate::pipeline::doppler::doppler_hz_from_carrier_hz;

use super::code_phase_refinement::{CodePhaseRefinementRequest, JointAcquisitionRefinementRequest};
use super::doppler_refinement::estimate_acquisition_doppler_refinement;
use super::Acquisition;

const JOINT_ACQUISITION_REFINEMENT_METHOD: &str = "quadratic_likelihood_surface";

#[derive(Clone, Copy)]
pub(super) struct CandidateRefinementRequest<'a> {
    pub(super) acquisition: &'a Acquisition,
    pub(super) frame: &'a SamplesFrame,
    pub(super) signal_model: &'a AcquisitionSignalModel,
    pub(super) grid_candidates: &'a [AcqResult],
    pub(super) doppler_step_hz: i32,
    pub(super) coherent_ms: u32,
    pub(super) noncoherent: u32,
}

pub(super) fn refine_acquisition_candidates(
    request: CandidateRefinementRequest<'_>,
    candidates: &mut [AcqResult],
) {
    let search_center_hz =
        request.signal_model.search_center_hz(request.acquisition.config.intermediate_freq_hz);
    for candidate in candidates {
        let coarse_carrier_hz = candidate.carrier_hz.0;
        if let Some(refinement) = request.acquisition.estimate_joint_acquisition_refinement(
            JointAcquisitionRefinementRequest {
                frame: request.frame,
                signal_model: request.signal_model,
                coarse_carrier_hz,
                coarse_doppler_rate_hz_per_s: candidate.doppler_rate_hz_per_s,
                coarse_code_phase_samples: candidate.code_phase_samples,
                doppler_step_hz: request.doppler_step_hz,
                coherent_ms: request.coherent_ms,
                noncoherent: request.noncoherent,
            },
        ) {
            candidate.carrier_hz = Hertz(
                coarse_carrier_hz
                    + (refinement.doppler_offset_bins * request.doppler_step_hz as f64),
            );
            candidate.doppler_hz =
                Hertz(doppler_hz_from_carrier_hz(search_center_hz, candidate.carrier_hz.0));
            candidate.doppler_refinement = Some(AcqDopplerRefinement {
                method: JOINT_ACQUISITION_REFINEMENT_METHOD.to_string(),
                coarse_carrier_hz: Hertz(coarse_carrier_hz),
                offset_hz: refinement.doppler_offset_bins * request.doppler_step_hz as f64,
                offset_bins: refinement.doppler_offset_bins,
                left_peak_mean_ratio: refinement.doppler_cross_section[0],
                center_peak_mean_ratio: refinement.doppler_cross_section[1],
                right_peak_mean_ratio: refinement.doppler_cross_section[2],
            });
            candidate.code_phase_refinement = Some(AcqCodePhaseRefinement {
                method: JOINT_ACQUISITION_REFINEMENT_METHOD.to_string(),
                offset_samples: refinement.code_phase_offset_samples,
                refined_code_phase_samples: refinement.refined_code_phase_samples,
                left_correlation_norm: refinement.code_phase_cross_section[0],
                center_correlation_norm: refinement.code_phase_cross_section[1],
                right_correlation_norm: refinement.code_phase_cross_section[2],
            });
            continue;
        }
        if let Some(refinement) = estimate_acquisition_doppler_refinement(
            coarse_carrier_hz,
            candidate.doppler_rate_hz_per_s,
            request.grid_candidates,
            request.doppler_step_hz,
        ) {
            candidate.carrier_hz = Hertz(coarse_carrier_hz + refinement.offset_hz);
            candidate.doppler_hz =
                Hertz(doppler_hz_from_carrier_hz(search_center_hz, candidate.carrier_hz.0));
            candidate.doppler_refinement = Some(refinement);
        }
        candidate.code_phase_refinement = request
            .acquisition
            .estimate_acquisition_code_phase_refinement(CodePhaseRefinementRequest {
                frame: request.frame,
                signal_model: request.signal_model,
                carrier_hz: candidate.carrier_hz.0,
                doppler_rate_hz_per_s: candidate.doppler_rate_hz_per_s,
                coarse_code_phase_samples: candidate.code_phase_samples,
                coherent_ms: request.coherent_ms,
                noncoherent: request.noncoherent,
            });
    }
}
