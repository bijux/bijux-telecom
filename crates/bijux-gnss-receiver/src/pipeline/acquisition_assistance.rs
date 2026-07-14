#![allow(missing_docs)]

use bijux_gnss_core::api::{
    AcqHypothesis, AcqRequest, AcqResult, GlonassFrequencyChannel, SatId, SignalBand, SignalCode,
};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CommonOscillatorBiasSignalEstimate {
    pub sat: SatId,
    pub signal_band: SignalBand,
    pub signal_code: SignalCode,
    pub glonass_frequency_channel: Option<GlonassFrequencyChannel>,
    pub request_index: usize,
    pub candidate_rank: u8,
    pub expected_line_of_sight_doppler_hz: f64,
    pub measured_doppler_hz: f64,
    pub implied_bias_hz: f64,
    pub residual_hz: f64,
    pub hypothesis: AcqHypothesis,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CommonOscillatorBiasEstimate {
    pub estimated_bias_hz: f64,
    pub support_count: usize,
    pub residual_spread_hz: f64,
    pub max_supporting_residual_hz: f64,
    pub supporting_signals: Vec<CommonOscillatorBiasSignalEstimate>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CommonOscillatorBiasFollowUpRequest {
    pub request_index: usize,
    pub estimated_signal_doppler_hz: f64,
    pub request: AcqRequest,
}

pub fn estimate_common_oscillator_bias(
    requests: &[AcqRequest],
    acquisition_rows: &[Vec<AcqResult>],
) -> Option<CommonOscillatorBiasEstimate> {
    let mut support = requests
        .iter()
        .copied()
        .zip(acquisition_rows.iter())
        .enumerate()
        .filter_map(|(request_index, (request, row))| {
            let expected_line_of_sight_doppler_hz = request.expected_line_of_sight_doppler_hz?;
            let primary = row.first()?;
            if !matches!(primary.hypothesis, AcqHypothesis::Accepted | AcqHypothesis::Ambiguous) {
                return None;
            }
            let measured_doppler_hz = primary.doppler_hz.0;
            if !measured_doppler_hz.is_finite() || !expected_line_of_sight_doppler_hz.is_finite() {
                return None;
            }
            Some(CommonOscillatorBiasSignalEstimate {
                sat: request.sat,
                signal_band: request.signal_band,
                signal_code: request.signal_code,
                glonass_frequency_channel: request.glonass_frequency_channel,
                request_index,
                candidate_rank: primary.candidate_rank,
                expected_line_of_sight_doppler_hz,
                measured_doppler_hz,
                implied_bias_hz: measured_doppler_hz - expected_line_of_sight_doppler_hz,
                residual_hz: 0.0,
                hypothesis: primary.hypothesis,
            })
        })
        .collect::<Vec<_>>();
    if support.len() < 2 {
        return None;
    }

    let estimated_bias_hz =
        support.iter().map(|signal| signal.implied_bias_hz).sum::<f64>() / support.len() as f64;
    for signal in &mut support {
        signal.residual_hz = signal.implied_bias_hz - estimated_bias_hz;
    }
    let (min_bias_hz, max_bias_hz) = support.iter().fold(
        (f64::INFINITY, f64::NEG_INFINITY),
        |(min_bias_hz, max_bias_hz), signal| {
            (min_bias_hz.min(signal.implied_bias_hz), max_bias_hz.max(signal.implied_bias_hz))
        },
    );
    let max_supporting_residual_hz =
        support.iter().map(|signal| signal.residual_hz.abs()).fold(0.0_f64, f64::max);

    Some(CommonOscillatorBiasEstimate {
        estimated_bias_hz,
        support_count: support.len(),
        residual_spread_hz: max_bias_hz - min_bias_hz,
        max_supporting_residual_hz,
        supporting_signals: support,
    })
}

pub fn build_common_oscillator_bias_follow_up_requests(
    requests: &[AcqRequest],
    acquisition_rows: &[Vec<AcqResult>],
    estimate: &CommonOscillatorBiasEstimate,
) -> Vec<CommonOscillatorBiasFollowUpRequest> {
    let mut follow_up_requests = Vec::new();
    for (request_index, (request, row)) in
        requests.iter().copied().zip(acquisition_rows.iter()).enumerate()
    {
        let Some(expected_line_of_sight_doppler_hz) = request.expected_line_of_sight_doppler_hz
        else {
            continue;
        };
        let primary = row.first();
        if primary.is_some_and(|candidate| {
            matches!(candidate.hypothesis, AcqHypothesis::Accepted | AcqHypothesis::Ambiguous)
        }) {
            continue;
        }

        let step_hz = request.doppler_step_hz.max(1);
        let narrowed_search_hz = ((estimate.max_supporting_residual_hz.ceil() as i32)
            .saturating_add(step_hz))
        .max(step_hz)
        .min(request.doppler_search_hz.max(step_hz));
        let estimated_signal_doppler_hz =
            expected_line_of_sight_doppler_hz + estimate.estimated_bias_hz;
        let follow_up_request = AcqRequest {
            doppler_center_hz: estimated_signal_doppler_hz,
            doppler_search_hz: narrowed_search_hz,
            ..request
        };
        if (follow_up_request.doppler_center_hz - request.doppler_center_hz).abs() <= f64::EPSILON
            && follow_up_request.doppler_search_hz >= request.doppler_search_hz
        {
            continue;
        }
        follow_up_requests.push(CommonOscillatorBiasFollowUpRequest {
            request_index,
            estimated_signal_doppler_hz,
            request: follow_up_request,
        });
    }
    follow_up_requests
}

#[cfg(test)]
mod tests {
    use super::{build_common_oscillator_bias_follow_up_requests, estimate_common_oscillator_bias};
    use bijux_gnss_core::api::{
        AcqHypothesis, AcqRequest, AcqResult, Constellation, Hertz, ReceiverSampleTrace, SatId,
        SignalBand, SignalCode,
    };

    #[test]
    fn estimates_common_oscillator_bias_from_trackable_primary_candidates() {
        let requests = vec![
            request_for_bias_test(3, Some(750.0), 0.0, 2_000),
            request_for_bias_test(7, Some(-1_000.0), 0.0, 2_000),
            request_for_bias_test(11, None, 0.0, 2_000),
        ];
        let acquisition_rows = vec![
            vec![result_for_bias_test(3, AcqHypothesis::Accepted, 1_250.0)],
            vec![result_for_bias_test(7, AcqHypothesis::Ambiguous, -500.0)],
            vec![result_for_bias_test(11, AcqHypothesis::Accepted, 200.0)],
        ];

        let estimate = estimate_common_oscillator_bias(&requests, &acquisition_rows)
            .expect("two supported rows should yield a bias estimate");

        assert_eq!(estimate.support_count, 2);
        assert!((estimate.estimated_bias_hz - 500.0).abs() <= f64::EPSILON, "{estimate:?}");
        assert!(estimate.max_supporting_residual_hz <= f64::EPSILON, "{estimate:?}");
        assert_eq!(
            estimate
                .supporting_signals
                .iter()
                .map(|signal| signal.request_index)
                .collect::<Vec<_>>(),
            vec![0, 1]
        );
    }

    #[test]
    fn builds_follow_up_requests_for_unacquired_assisted_rows() {
        let requests = vec![
            request_for_bias_test(3, Some(750.0), 0.0, 2_000),
            request_for_bias_test(7, Some(-1_000.0), 0.0, 2_000),
            request_for_bias_test(11, Some(250.0), 0.0, 2_000),
        ];
        let acquisition_rows = vec![
            vec![result_for_bias_test(3, AcqHypothesis::Accepted, 1_250.0)],
            vec![result_for_bias_test(7, AcqHypothesis::Accepted, -500.0)],
            vec![result_for_bias_test(11, AcqHypothesis::Rejected, -250.0)],
        ];

        let estimate = estimate_common_oscillator_bias(&requests, &acquisition_rows)
            .expect("two supported rows should yield a bias estimate");
        let follow_up_requests = build_common_oscillator_bias_follow_up_requests(
            &requests,
            &acquisition_rows,
            &estimate,
        );

        assert_eq!(follow_up_requests.len(), 1);
        let follow_up = &follow_up_requests[0];
        assert_eq!(follow_up.request_index, 2);
        assert!((follow_up.estimated_signal_doppler_hz - 750.0).abs() <= f64::EPSILON);
        assert!((follow_up.request.doppler_center_hz - 750.0).abs() <= f64::EPSILON);
        assert!(follow_up.request.doppler_search_hz <= requests[2].doppler_search_hz);
    }

    fn request_for_bias_test(
        prn: u8,
        expected_line_of_sight_doppler_hz: Option<f64>,
        doppler_center_hz: f64,
        doppler_search_hz: i32,
    ) -> AcqRequest {
        AcqRequest {
            sat: SatId { constellation: Constellation::Gps, prn },
            glonass_frequency_channel: None,
            signal_band: SignalBand::L1,
            signal_code: SignalCode::Ca,
            doppler_center_hz,
            expected_line_of_sight_doppler_hz,
            assistance_bounds: None,
            doppler_search_hz,
            doppler_step_hz: 250,
            coherent_ms: 1,
            noncoherent: 1,
        }
    }

    fn result_for_bias_test(prn: u8, hypothesis: AcqHypothesis, doppler_hz: f64) -> AcqResult {
        AcqResult {
            sat: SatId { constellation: Constellation::Gps, prn },
            signal_band: SignalBand::L1,
            signal_code: SignalCode::Ca,
            glonass_frequency_channel: None,
            source_time: ReceiverSampleTrace::default(),
            candidate_rank: 1,
            is_primary_candidate: true,
            doppler_hz: Hertz(doppler_hz),
            carrier_hz: Hertz(doppler_hz),
            code_phase_samples: 0,
            peak: 0.0,
            second_peak: 0.0,
            mean: 0.0,
            peak_mean_ratio: 0.0,
            peak_second_ratio: 0.0,
            cn0_proxy: 0.0,
            score: 0.0,
            hypothesis,
            assumptions: None,
            evidence: Vec::new(),
            threshold_provenance: None,
            explain_selection_reason: None,
            doppler_refinement: None,
            code_phase_refinement: None,
            signal_delay_alignment: None,
            uncertainty: None,
        }
    }
}
