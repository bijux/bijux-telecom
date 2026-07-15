use bijux_gnss_core::api::{
    AcqCodePhaseRefinement, AcqDopplerRefinement, AcqResult, Hertz, SamplesFrame, SatId,
};
use bijux_gnss_signal::api::AcquisitionSignalModel;

use crate::pipeline::doppler::doppler_hz_from_carrier_hz;

use super::doppler_refinement::estimate_acquisition_doppler_refinement;
use super::Acquisition;

const JOINT_ACQUISITION_REFINEMENT_METHOD: &str = "quadratic_likelihood_surface";

pub(super) fn refine_acquisition_candidates(
    acquisition: &Acquisition,
    frame: &SamplesFrame,
    signal_model: &AcquisitionSignalModel,
    sat: SatId,
    candidates: &mut [AcqResult],
    grid_candidates: &[AcqResult],
    doppler_step_hz: i32,
    coherent_ms: u32,
    noncoherent: u32,
) {
    let search_center_hz = signal_model.search_center_hz(acquisition.config.intermediate_freq_hz);
    for candidate in candidates {
        let coarse_carrier_hz = candidate.carrier_hz.0;
        if let Some(refinement) = acquisition.estimate_joint_acquisition_refinement(
            frame,
            signal_model,
            sat,
            coarse_carrier_hz,
            candidate.doppler_rate_hz_per_s,
            candidate.code_phase_samples,
            doppler_step_hz,
            coherent_ms,
            noncoherent,
        ) {
            candidate.carrier_hz = Hertz(
                coarse_carrier_hz + (refinement.doppler_offset_bins * doppler_step_hz as f64),
            );
            candidate.doppler_hz =
                Hertz(doppler_hz_from_carrier_hz(search_center_hz, candidate.carrier_hz.0));
            candidate.doppler_refinement = Some(AcqDopplerRefinement {
                method: JOINT_ACQUISITION_REFINEMENT_METHOD.to_string(),
                coarse_carrier_hz: Hertz(coarse_carrier_hz),
                offset_hz: refinement.doppler_offset_bins * doppler_step_hz as f64,
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
            grid_candidates,
            doppler_step_hz,
        ) {
            candidate.carrier_hz = Hertz(coarse_carrier_hz + refinement.offset_hz);
            candidate.doppler_hz =
                Hertz(doppler_hz_from_carrier_hz(search_center_hz, candidate.carrier_hz.0));
            candidate.doppler_refinement = Some(refinement);
        }
        candidate.code_phase_refinement = acquisition.estimate_acquisition_code_phase_refinement(
            frame,
            signal_model,
            sat,
            candidate.carrier_hz.0,
            candidate.doppler_rate_hz_per_s,
            candidate.code_phase_samples,
            coherent_ms,
            noncoherent,
        );
    }
}
