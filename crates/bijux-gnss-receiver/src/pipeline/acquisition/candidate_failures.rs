use bijux_gnss_core::api::{
    AcqAssumptions, AcqHypothesis, AcqRequest, AcqResult, AcqThresholdProvenance,
    GlonassFrequencyChannel, Hertz, ReceiverSampleTrace, SatId, SignalCode,
};
use bijux_gnss_signal::api::{samples_per_code, AcquisitionSignalModel, SignalError};

use crate::engine::receiver_config::{
    supported_acquisition_integration_ms_csv, ReceiverPipelineConfig,
};
use crate::pipeline::doppler::carrier_hz_from_doppler_hz;

use super::signal_model::resolved_signal_code;

#[derive(Clone, Copy)]
pub(super) struct AcquisitionCandidateContext<'a> {
    pub(super) sat: SatId,
    pub(super) signal_model: &'a AcquisitionSignalModel,
    pub(super) signal_code: SignalCode,
    pub(super) glonass_frequency_channel: Option<GlonassFrequencyChannel>,
    pub(super) assumptions: &'a AcqAssumptions,
    pub(super) threshold_provenance: &'a AcqThresholdProvenance,
    pub(super) intermediate_freq_hz: f64,
    pub(super) source_time: ReceiverSampleTrace,
}

pub(super) fn insufficient_frame_candidates(
    context: AcquisitionCandidateContext<'_>,
    available_samples: usize,
    required_samples: usize,
) -> Vec<AcqResult> {
    let candidate_reason = insufficient_frame_candidate_reason(available_samples, required_samples);
    vec![AcqResult {
        sat: context.sat,
        signal_band: context.signal_model.signal_band,
        signal_code: context.signal_code,
        glonass_frequency_channel: context.glonass_frequency_channel,
        source_time: context.source_time,
        candidate_rank: 1,
        is_primary_candidate: true,
        doppler_hz: Hertz(0.0),
        doppler_rate_hz_per_s: context.assumptions.doppler_rate_center_hz_per_s,
        carrier_hz: Hertz(context.intermediate_freq_hz),
        code_phase_samples: 0,
        peak: 0.0,
        second_peak: 0.0,
        mean: 0.0,
        peak_mean_ratio: 0.0,
        peak_second_ratio: 0.0,
        cn0_proxy: 0.0,
        score: 0.0,
        hypothesis: AcqHypothesis::Deferred,
        assumptions: Some(context.assumptions.clone()),
        evidence: Vec::new(),
        threshold_provenance: Some(context.threshold_provenance.clone()),
        explain_selection_reason: Some(candidate_reason),
        doppler_refinement: None,
        code_phase_refinement: None,
        signal_delay_alignment: None,
        uncertainty: None,
    }]
}

pub(super) fn acquisition_request_error_candidates(
    config: &ReceiverPipelineConfig,
    request: AcqRequest,
    threshold_provenance: &AcqThresholdProvenance,
    source_time: ReceiverSampleTrace,
    frame_samples: usize,
    error: SignalError,
) -> Vec<AcqResult> {
    let samples_per_code =
        samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length);
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
        samples_per_code,
        frame_samples,
        code_phase_search_start_sample: 0,
        code_phase_search_step_samples: 1,
        code_phase_search_bins: samples_per_code,
        code_phase_search_mode: "full_code".to_string(),
    };
    let reason = match error {
        SignalError::MissingGlonassFrequencyChannel(sat) => format!(
            "missing_glonass_frequency_channel: acquisition request for {} must declare glonass_frequency_channel",
            bijux_gnss_core::api::format_sat(sat)
        ),
        other => format!("invalid_acquisition_signal_model: {other}"),
    };

    vec![AcqResult {
        sat: request.sat,
        signal_band: request.signal_band,
        signal_code: resolved_signal_code(request.sat, request.signal_band, request.signal_code),
        glonass_frequency_channel: request.glonass_frequency_channel,
        source_time,
        candidate_rank: 1,
        is_primary_candidate: true,
        doppler_hz: Hertz(request.doppler_center_hz),
        doppler_rate_hz_per_s: request.doppler_rate_center_hz_per_s,
        carrier_hz: Hertz(carrier_hz_from_doppler_hz(
            config.intermediate_freq_hz,
            request.doppler_center_hz,
        )),
        code_phase_samples: 0,
        peak: 0.0,
        second_peak: 0.0,
        mean: 0.0,
        peak_mean_ratio: 0.0,
        peak_second_ratio: 0.0,
        cn0_proxy: 0.0,
        score: 0.0,
        hypothesis: AcqHypothesis::Deferred,
        assumptions: Some(assumptions),
        evidence: Vec::new(),
        threshold_provenance: Some(threshold_provenance.clone()),
        explain_selection_reason: Some(reason),
        doppler_refinement: None,
        code_phase_refinement: None,
        signal_delay_alignment: None,
        uncertainty: None,
    }]
}

pub(super) fn zero_signal_candidate(
    context: AcquisitionCandidateContext<'_>,
    zero_signal_reason: Option<&str>,
) -> AcqResult {
    AcqResult {
        sat: context.sat,
        signal_band: context.signal_model.signal_band,
        signal_code: context.signal_code,
        glonass_frequency_channel: context.glonass_frequency_channel,
        source_time: context.source_time,
        candidate_rank: 1,
        is_primary_candidate: true,
        doppler_hz: Hertz(context.assumptions.doppler_center_hz),
        doppler_rate_hz_per_s: context.assumptions.doppler_rate_center_hz_per_s,
        carrier_hz: Hertz(context.intermediate_freq_hz),
        code_phase_samples: 0,
        peak: 0.0,
        second_peak: 0.0,
        mean: 0.0,
        peak_mean_ratio: 0.0,
        peak_second_ratio: 0.0,
        cn0_proxy: 0.0,
        score: 0.0,
        hypothesis: AcqHypothesis::Rejected,
        assumptions: Some(context.assumptions.clone()),
        evidence: Vec::new(),
        threshold_provenance: Some(context.threshold_provenance.clone()),
        explain_selection_reason: Some(zero_signal_candidate_reason(zero_signal_reason)),
        doppler_refinement: None,
        code_phase_refinement: None,
        signal_delay_alignment: None,
        uncertainty: None,
    }
}

pub(super) fn unsupported_coherent_integration_candidates(
    context: AcquisitionCandidateContext<'_>,
    coherent_ms: u32,
) -> Vec<AcqResult> {
    vec![unsupported_coherent_integration_candidate(context, coherent_ms)]
}

fn zero_signal_candidate_reason(zero_signal_reason: Option<&str>) -> String {
    match zero_signal_reason {
        Some(reason) => format!("zero_signal_input: {reason}"),
        None => "zero_signal_input".to_string(),
    }
}

fn insufficient_frame_candidate_reason(
    available_samples: usize,
    required_samples: usize,
) -> String {
    format!(
        "insufficient_frame: acquisition requires {required_samples} samples but received {available_samples}"
    )
}

fn unsupported_coherent_integration_candidate(
    context: AcquisitionCandidateContext<'_>,
    coherent_ms: u32,
) -> AcqResult {
    AcqResult {
        sat: context.sat,
        signal_band: context.signal_model.signal_band,
        signal_code: context.signal_code,
        glonass_frequency_channel: context.glonass_frequency_channel,
        source_time: context.source_time,
        candidate_rank: 1,
        is_primary_candidate: true,
        doppler_hz: Hertz(context.assumptions.doppler_center_hz),
        doppler_rate_hz_per_s: context.assumptions.doppler_rate_center_hz_per_s,
        carrier_hz: Hertz(context.intermediate_freq_hz),
        code_phase_samples: 0,
        peak: 0.0,
        second_peak: 0.0,
        mean: 0.0,
        peak_mean_ratio: 0.0,
        peak_second_ratio: 0.0,
        cn0_proxy: 0.0,
        score: 0.0,
        hypothesis: AcqHypothesis::Deferred,
        assumptions: Some(context.assumptions.clone()),
        evidence: Vec::new(),
        threshold_provenance: Some(context.threshold_provenance.clone()),
        explain_selection_reason: Some(unsupported_coherent_integration_candidate_reason(
            coherent_ms,
        )),
        doppler_refinement: None,
        code_phase_refinement: None,
        signal_delay_alignment: None,
        uncertainty: None,
    }
}

fn unsupported_coherent_integration_candidate_reason(coherent_ms: u32) -> String {
    format!(
        "unsupported_coherent_integration_ms: acquisition coherent integration must be one of [{}] ms but received {} ms",
        supported_acquisition_integration_ms_csv(),
        coherent_ms,
    )
}
