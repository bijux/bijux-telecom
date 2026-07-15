use bijux_gnss_core::api::{
    AcqAssumptions, AcqExplain, AcqExplainCandidate, AcqHypothesis, AcqRequest, ReceiverSampleTrace,
};

use crate::engine::receiver_config::ReceiverPipelineConfig;

use super::candidate_failures::{acquisition_request_error_candidates, zero_signal_candidate};
use super::signal_model::{
    acquisition_signal_model_for_request, request_search_center_hz, resolved_request_signal_code,
};
use super::threshold_resolution::threshold_provenance_for_request;
use super::AcquisitionRun;

pub(super) fn zero_signal_run(
    config: &ReceiverPipelineConfig,
    requests: &[AcqRequest],
    source_time: ReceiverSampleTrace,
    frame_samples: usize,
    zero_signal_reason: Option<&str>,
    emit_explanations: bool,
) -> AcquisitionRun {
    let mut results = Vec::with_capacity(requests.len());
    let mut explains = Vec::new();

    for &request in requests {
        let sat = request.sat;
        let threshold_provenance = threshold_provenance_for_request(config, request);
        let signal_model = match acquisition_signal_model_for_request(config, request) {
            Ok(signal_model) => signal_model,
            Err(error) => {
                let candidates = acquisition_request_error_candidates(
                    config,
                    request,
                    &threshold_provenance,
                    source_time,
                    frame_samples,
                    error,
                );
                if emit_explanations {
                    explains.push(AcqExplain {
                        sat,
                        selected_rank: Some(1),
                        selected_reason: "invalid_acquisition_signal_model".to_string(),
                        candidate_count: candidates.len(),
                        candidates: candidates
                            .iter()
                            .enumerate()
                            .map(|(index, candidate)| AcqExplainCandidate {
                                rank: (index + 1) as u8,
                                code_phase_samples: candidate.code_phase_samples,
                                carrier_hz: candidate.carrier_hz.0,
                                peak: candidate.peak,
                                peak_mean_ratio: candidate.peak_mean_ratio,
                                peak_second_ratio: candidate.peak_second_ratio,
                                second_peak_ratio: if candidate.peak == 0.0 {
                                    0.0
                                } else {
                                    candidate.second_peak / candidate.peak
                                },
                                mean: candidate.mean,
                                hypothesis: candidate.hypothesis,
                                score: candidate.score,
                                threshold_hit: false,
                                reason: candidate
                                    .explain_selection_reason
                                    .clone()
                                    .unwrap_or_default(),
                            })
                            .collect(),
                    });
                }
                results.push(candidates);
                continue;
            }
        };
        let search_center_hz =
            request_search_center_hz(&signal_model, config.intermediate_freq_hz, request);
        let signal_code = resolved_request_signal_code(request);
        let assumptions = AcqAssumptions {
            doppler_center_hz: request.doppler_center_hz,
            doppler_rate_center_hz_per_s: request.doppler_rate_center_hz_per_s,
            expected_line_of_sight_doppler_hz: request.expected_line_of_sight_doppler_hz,
            assistance_bounds: request.assistance_bounds,
            doppler_search_hz: threshold_provenance.doppler_search_hz,
            doppler_step_hz: threshold_provenance.doppler_step_hz,
            doppler_rate_search_hz_per_s: threshold_provenance.doppler_rate_search_hz_per_s,
            doppler_rate_step_hz_per_s: threshold_provenance.doppler_rate_step_hz_per_s,
            coherent_ms: threshold_provenance.coherent_ms,
            noncoherent: threshold_provenance.noncoherent,
            samples_per_code: signal_model.samples_per_code(config.sampling_freq_hz),
            frame_samples,
            code_phase_search_start_sample: 0,
            code_phase_search_step_samples: 1,
            code_phase_search_bins: signal_model.samples_per_code(config.sampling_freq_hz),
            code_phase_search_mode: "full_code".to_string(),
        };
        let result = zero_signal_candidate(
            sat,
            &signal_model,
            signal_code,
            request.glonass_frequency_channel,
            &assumptions,
            &threshold_provenance,
            search_center_hz,
            source_time,
            zero_signal_reason,
        );
        if emit_explanations {
            explains.push(AcqExplain {
                sat,
                selected_rank: Some(1),
                selected_reason: "zero_signal_input".to_string(),
                candidate_count: 1,
                candidates: vec![AcqExplainCandidate {
                    rank: 1,
                    code_phase_samples: 0,
                    carrier_hz: search_center_hz,
                    peak: 0.0,
                    peak_mean_ratio: 0.0,
                    peak_second_ratio: 0.0,
                    second_peak_ratio: 0.0,
                    mean: 0.0,
                    hypothesis: AcqHypothesis::Rejected,
                    score: 0.0,
                    threshold_hit: false,
                    reason: result.explain_selection_reason.clone().unwrap_or_default(),
                }],
            });
        }
        results.push(vec![result]);
    }

    AcquisitionRun { results, explains }
}
