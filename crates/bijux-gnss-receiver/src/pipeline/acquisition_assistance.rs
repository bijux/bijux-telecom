#![allow(missing_docs)]

use bijux_gnss_core::api::{
    AcqAssistanceBounds, AcqHypothesis, AcqRequest, AcqResult, GlonassFrequencyChannel, SatId,
    SignalBand, SignalCode,
};
use bijux_gnss_signal::api::{
    resolved_signal_registry_entry, shared_path_doppler_hz, AcquisitionSignalModel,
};
use serde::{Deserialize, Serialize};

use crate::engine::receiver_config::ReceiverPipelineConfig;

const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;
const FULL_CODE_SEARCH_MODE: &str = "full_code";
const ASSISTED_BOUNDS_SEARCH_MODE: &str = "assisted_bounds";

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct ResolvedAcquisitionSearchBounds {
    pub doppler_search_hz: i32,
    pub derived_doppler_uncertainty_hz: Option<f64>,
    pub code_phase_search_start_sample: usize,
    pub code_phase_search_step_samples: usize,
    pub code_phase_search_bins: usize,
    pub code_phase_search_mode: String,
    pub derived_code_phase_uncertainty_samples: Option<f64>,
    pub search_domain_reduced: bool,
}

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

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RelatedSignalFollowUpRequest {
    pub request_index: usize,
    pub source_request_index: usize,
    pub source_signal_band: SignalBand,
    pub source_signal_code: SignalCode,
    pub estimated_signal_doppler_hz: f64,
    pub transferred_code_phase_samples: f64,
    pub request: AcqRequest,
}

pub fn resolve_acquisition_search_bounds(
    config: &ReceiverPipelineConfig,
    signal_model: &AcquisitionSignalModel,
    request: AcqRequest,
) -> ResolvedAcquisitionSearchBounds {
    let samples_per_code = signal_model.samples_per_code(config.sampling_freq_hz).max(1);
    let full_search = ResolvedAcquisitionSearchBounds {
        doppler_search_hz: request.doppler_search_hz.max(0),
        derived_doppler_uncertainty_hz: None,
        code_phase_search_start_sample: 0,
        code_phase_search_step_samples: 1,
        code_phase_search_bins: samples_per_code,
        code_phase_search_mode: FULL_CODE_SEARCH_MODE.to_string(),
        derived_code_phase_uncertainty_samples: None,
        search_domain_reduced: false,
    };
    let Some(assistance_bounds) = request.assistance_bounds else {
        return full_search;
    };
    let Some(code_phase_uncertainty_samples) =
        derive_code_phase_uncertainty_samples(config.sampling_freq_hz, assistance_bounds)
    else {
        return full_search;
    };
    let Some(doppler_uncertainty_hz) =
        derive_doppler_uncertainty_hz(signal_model, assistance_bounds)
    else {
        return full_search;
    };
    let doppler_search_hz = quantized_doppler_search_hz(
        request.doppler_search_hz.max(0),
        request.doppler_step_hz.max(1),
        doppler_uncertainty_hz,
    );
    let (code_phase_search_start_sample, code_phase_search_bins) =
        assisted_code_phase_search_window(
            samples_per_code,
            assistance_bounds.expected_code_phase_samples,
            code_phase_uncertainty_samples,
        )
        .unwrap_or((0, samples_per_code));
    let search_domain_reduced =
        doppler_search_hz < request.doppler_search_hz.max(0) || code_phase_search_bins < samples_per_code;

    ResolvedAcquisitionSearchBounds {
        doppler_search_hz,
        derived_doppler_uncertainty_hz: Some(doppler_uncertainty_hz),
        code_phase_search_start_sample,
        code_phase_search_step_samples: 1,
        code_phase_search_bins,
        code_phase_search_mode: if search_domain_reduced {
            ASSISTED_BOUNDS_SEARCH_MODE.to_string()
        } else {
            FULL_CODE_SEARCH_MODE.to_string()
        },
        derived_code_phase_uncertainty_samples: Some(code_phase_uncertainty_samples),
        search_domain_reduced,
    }
}

fn derive_code_phase_uncertainty_samples(
    sampling_freq_hz: f64,
    assistance_bounds: AcqAssistanceBounds,
) -> Option<f64> {
    if !sampling_freq_hz.is_finite()
        || sampling_freq_hz <= 0.0
        || !assistance_bounds.expected_code_phase_samples.is_finite()
        || !assistance_bounds.time_uncertainty_s.is_finite()
        || assistance_bounds.time_uncertainty_s < 0.0
        || !assistance_bounds.position_uncertainty_m.is_finite()
        || assistance_bounds.position_uncertainty_m < 0.0
    {
        return None;
    }
    let time_uncertainty_from_position_s =
        assistance_bounds.position_uncertainty_m / SPEED_OF_LIGHT_MPS;
    Some(
        (assistance_bounds.time_uncertainty_s + time_uncertainty_from_position_s) * sampling_freq_hz,
    )
}

fn derive_doppler_uncertainty_hz(
    signal_model: &AcquisitionSignalModel,
    assistance_bounds: AcqAssistanceBounds,
) -> Option<f64> {
    if !assistance_bounds.oscillator_uncertainty_hz.is_finite()
        || assistance_bounds.oscillator_uncertainty_hz < 0.0
        || !assistance_bounds.approximate_velocity_uncertainty_mps.is_finite()
        || assistance_bounds.approximate_velocity_uncertainty_mps < 0.0
    {
        return None;
    }
    let carrier_hz = signal_model.carrier_hz.value();
    if !carrier_hz.is_finite() || carrier_hz <= 0.0 {
        return None;
    }
    let velocity_doppler_hz =
        carrier_hz * assistance_bounds.approximate_velocity_uncertainty_mps / SPEED_OF_LIGHT_MPS;
    Some(assistance_bounds.oscillator_uncertainty_hz + velocity_doppler_hz)
}

fn quantized_doppler_search_hz(
    full_search_hz: i32,
    doppler_step_hz: i32,
    doppler_uncertainty_hz: f64,
) -> i32 {
    if full_search_hz <= 0 {
        return 0;
    }
    if !doppler_uncertainty_hz.is_finite() || doppler_uncertainty_hz <= 0.0 {
        return 0;
    }
    let step_hz = doppler_step_hz.max(1) as f64;
    let quantized_search_hz = (doppler_uncertainty_hz / step_hz).ceil() as i32 * doppler_step_hz.max(1);
    quantized_search_hz.min(full_search_hz)
}

fn assisted_code_phase_search_window(
    samples_per_code: usize,
    expected_code_phase_samples: f64,
    code_phase_uncertainty_samples: f64,
) -> Option<(usize, usize)> {
    if samples_per_code == 0
        || !expected_code_phase_samples.is_finite()
        || !code_phase_uncertainty_samples.is_finite()
        || code_phase_uncertainty_samples < 0.0
    {
        return None;
    }
    let radius_samples = code_phase_uncertainty_samples.ceil() as usize;
    let bounded_bins = radius_samples
        .saturating_mul(2)
        .saturating_add(1)
        .min(samples_per_code);
    if bounded_bins >= samples_per_code {
        return Some((0, samples_per_code));
    }
    let center_sample = expected_code_phase_samples.round().rem_euclid(samples_per_code as f64) as usize;
    let start_sample = (center_sample + samples_per_code - radius_samples % samples_per_code) % samples_per_code;
    Some((start_sample, bounded_bins))
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
            doppler_rate_center_hz_per_s: 0.0,
            doppler_rate_search_hz_per_s: 0,
            doppler_rate_step_hz_per_s: 250,
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

pub fn build_related_signal_follow_up_requests(
    config: &ReceiverPipelineConfig,
    requests: &[AcqRequest],
    acquisition_rows: &[Vec<AcqResult>],
) -> Vec<RelatedSignalFollowUpRequest> {
    let mut follow_up_requests = Vec::new();
    for (request_index, (request, row)) in
        requests.iter().copied().zip(acquisition_rows.iter()).enumerate()
    {
        let primary = row.first();
        if primary.is_some_and(|candidate| matches!(candidate.hypothesis, AcqHypothesis::Accepted))
        {
            continue;
        }
        let Some(follow_up_request) = requests
            .iter()
            .copied()
            .zip(acquisition_rows.iter())
            .enumerate()
            .filter_map(|(source_request_index, (source_request, source_row))| {
                build_related_signal_follow_up_request(
                    config,
                    request_index,
                    request,
                    source_request_index,
                    source_request,
                    source_row.first(),
                )
            })
            .max_by(|left, right| {
                let left_score = acquisition_rows[left.source_request_index]
                    .first()
                    .map(|candidate| candidate.score)
                    .unwrap_or(f32::NEG_INFINITY);
                let right_score = acquisition_rows[right.source_request_index]
                    .first()
                    .map(|candidate| candidate.score)
                    .unwrap_or(f32::NEG_INFINITY);
                left_score
                    .total_cmp(&right_score)
                    .then_with(|| left.source_request_index.cmp(&right.source_request_index))
            })
        else {
            continue;
        };
        follow_up_requests.push(follow_up_request);
    }
    follow_up_requests
}

fn build_related_signal_follow_up_request(
    config: &ReceiverPipelineConfig,
    request_index: usize,
    request: AcqRequest,
    source_request_index: usize,
    source_request: AcqRequest,
    source_primary: Option<&AcqResult>,
) -> Option<RelatedSignalFollowUpRequest> {
    if source_request_index == request_index
        || source_request.sat != request.sat
        || source_request.signal_band == request.signal_band
        || request.assistance_bounds.is_some()
        || request.expected_line_of_sight_doppler_hz.is_some()
        || request.doppler_center_hz.abs() > f64::EPSILON
    {
        return None;
    }
    let source_primary = source_primary?;
    if !matches!(source_primary.hypothesis, AcqHypothesis::Accepted | AcqHypothesis::Ambiguous) {
        return None;
    }

    let source_signal = resolved_signal_registry_entry(
        source_request.sat,
        source_request.signal_band,
        source_request.signal_code,
        source_request.glonass_frequency_channel,
    )
    .ok()
    .flatten()?;
    let target_signal = resolved_signal_registry_entry(
        request.sat,
        request.signal_band,
        request.signal_code,
        request.glonass_frequency_channel,
    )
    .ok()
    .flatten()?;
    let estimated_signal_doppler_hz = shared_path_doppler_hz(
        source_primary.doppler_hz.0,
        source_signal.spec,
        target_signal.spec,
    )?;
    let scaled_doppler_uncertainty_hz = source_primary
        .uncertainty
        .as_ref()
        .and_then(|uncertainty| {
            shared_path_doppler_hz(
                uncertainty.doppler_hz.abs(),
                source_signal.spec,
                target_signal.spec,
            )
        })
        .unwrap_or(request.doppler_step_hz.max(1) as f64);
    if !config.sampling_freq_hz.is_finite() || config.sampling_freq_hz <= 0.0 {
        return None;
    }
    let transferred_code_phase_samples = source_primary.resolved_code_phase_samples();
    let code_phase_uncertainty_samples = source_primary
        .uncertainty
        .as_ref()
        .map(|uncertainty| uncertainty.code_phase_samples.abs())
        .unwrap_or(1.0);
    if !transferred_code_phase_samples.is_finite()
        || !code_phase_uncertainty_samples.is_finite()
        || code_phase_uncertainty_samples <= 0.0
        || !scaled_doppler_uncertainty_hz.is_finite()
        || scaled_doppler_uncertainty_hz <= 0.0
    {
        return None;
    }

    let follow_up_request = AcqRequest {
        doppler_center_hz: estimated_signal_doppler_hz,
        assistance_bounds: Some(AcqAssistanceBounds {
            expected_code_phase_samples: transferred_code_phase_samples,
            time_uncertainty_s: code_phase_uncertainty_samples / config.sampling_freq_hz,
            position_uncertainty_m: 0.0,
            oscillator_uncertainty_hz: scaled_doppler_uncertainty_hz,
            approximate_velocity_uncertainty_mps: 0.0,
        }),
        ..request
    };
    Some(RelatedSignalFollowUpRequest {
        request_index,
        source_request_index,
        source_signal_band: source_request.signal_band,
        source_signal_code: source_request.signal_code,
        estimated_signal_doppler_hz,
        transferred_code_phase_samples,
        request: follow_up_request,
    })
}

#[cfg(test)]
mod tests {
    use super::{
        build_common_oscillator_bias_follow_up_requests, build_related_signal_follow_up_requests,
        estimate_common_oscillator_bias, resolve_acquisition_search_bounds,
    };
    use crate::engine::receiver_config::ReceiverPipelineConfig;
    use bijux_gnss_core::api::{
        AcqAssistanceBounds, AcqHypothesis, AcqRequest, AcqResult, AcqUncertainty,
        Constellation,
        GPS_L1_CA_CARRIER_HZ, Hertz, ReceiverSampleTrace, SatId, SignalBand, SignalCode,
    };
    use bijux_gnss_signal::api::{shared_path_doppler_hz, signal_spec_gps_l1_ca, signal_spec_gps_l5_i, AcquisitionSignalModel};

    #[test]
    fn resolves_assisted_search_bounds_from_uncertainty_contract() {
        let config = ReceiverPipelineConfig::default();
        let signal_model = AcquisitionSignalModel::gps_l1_ca_or_ones(
            3,
            config.code_freq_basis_hz,
            config.code_length,
            GPS_L1_CA_CARRIER_HZ,
        )
        .expect("GPS L1 C/A search model");
        let request = request_for_bounds_test(
            3,
            4_000,
            Some(AcqAssistanceBounds {
                expected_code_phase_samples: 512.0,
                time_uncertainty_s: 1.0e-6,
                position_uncertainty_m: 150.0,
                oscillator_uncertainty_hz: 180.0,
                approximate_velocity_uncertainty_mps: 8.0,
            }),
        );

        let resolved = resolve_acquisition_search_bounds(&config, &signal_model, request);

        assert!(resolved.search_domain_reduced, "{resolved:?}");
        assert_eq!(resolved.doppler_search_hz, 250, "{resolved:?}");
        assert_eq!(resolved.code_phase_search_bins, 17, "{resolved:?}");
        assert_eq!(resolved.code_phase_search_mode, "assisted_bounds");
    }

    #[test]
    fn wraps_assisted_code_phase_centers_into_one_code_period() {
        let config = ReceiverPipelineConfig::default();
        let signal_model = AcquisitionSignalModel::gps_l1_ca_or_ones(
            7,
            config.code_freq_basis_hz,
            config.code_length,
            GPS_L1_CA_CARRIER_HZ,
        )
        .expect("GPS L1 C/A search model");
        let samples_per_code = signal_model.samples_per_code(config.sampling_freq_hz).max(1);
        let request = request_for_bounds_test(
            7,
            2_000,
            Some(AcqAssistanceBounds {
                expected_code_phase_samples: samples_per_code as f64 + 18.4,
                time_uncertainty_s: 0.0,
                position_uncertainty_m: 0.0,
                oscillator_uncertainty_hz: 0.0,
                approximate_velocity_uncertainty_mps: 0.0,
            }),
        );

        let resolved = resolve_acquisition_search_bounds(&config, &signal_model, request);

        assert_eq!(resolved.code_phase_search_bins, 1, "{resolved:?}");
        assert_eq!(resolved.code_phase_search_start_sample, 18, "{resolved:?}");
        assert_eq!(resolved.doppler_search_hz, 0, "{resolved:?}");
    }

    #[test]
    fn falls_back_to_full_search_when_assistance_is_invalid() {
        let config = ReceiverPipelineConfig::default();
        let signal_model = AcquisitionSignalModel::gps_l1_ca_or_ones(
            11,
            config.code_freq_basis_hz,
            config.code_length,
            GPS_L1_CA_CARRIER_HZ,
        )
        .expect("GPS L1 C/A search model");
        let request = request_for_bounds_test(
            11,
            3_000,
            Some(AcqAssistanceBounds {
                expected_code_phase_samples: 64.0,
                time_uncertainty_s: 0.0,
                position_uncertainty_m: -1.0,
                oscillator_uncertainty_hz: 100.0,
                approximate_velocity_uncertainty_mps: 5.0,
            }),
        );

        let resolved = resolve_acquisition_search_bounds(&config, &signal_model, request);

        assert!(!resolved.search_domain_reduced, "{resolved:?}");
        assert_eq!(resolved.doppler_search_hz, request.doppler_search_hz);
        assert_eq!(
            resolved.code_phase_search_bins,
            signal_model.samples_per_code(config.sampling_freq_hz)
        );
        assert_eq!(resolved.code_phase_search_mode, "full_code");
        assert_eq!(resolved.derived_doppler_uncertainty_hz, None);
        assert_eq!(resolved.derived_code_phase_uncertainty_samples, None);
    }

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

    #[test]
    fn builds_related_signal_follow_up_for_same_satellite_cross_band_pair() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 10_230_000.0,
            ..ReceiverPipelineConfig::default()
        };
        let sat = SatId { constellation: Constellation::Gps, prn: 9 };
        let requests = vec![
            AcqRequest {
                sat,
                glonass_frequency_channel: None,
                signal_band: SignalBand::L1,
                signal_code: SignalCode::Ca,
                doppler_center_hz: 0.0,
                doppler_rate_center_hz_per_s: 0.0,
                expected_line_of_sight_doppler_hz: None,
                assistance_bounds: None,
                doppler_search_hz: 4_000,
                doppler_step_hz: 250,
                doppler_rate_search_hz_per_s: 0,
                doppler_rate_step_hz_per_s: 250,
                coherent_ms: 1,
                noncoherent: 1,
            },
            AcqRequest {
                sat,
                glonass_frequency_channel: None,
                signal_band: SignalBand::L5,
                signal_code: SignalCode::L5I,
                doppler_center_hz: 0.0,
                doppler_rate_center_hz_per_s: 0.0,
                expected_line_of_sight_doppler_hz: None,
                assistance_bounds: None,
                doppler_search_hz: 4_000,
                doppler_step_hz: 250,
                doppler_rate_search_hz_per_s: 0,
                doppler_rate_step_hz_per_s: 250,
                coherent_ms: 1,
                noncoherent: 1,
            },
        ];
        let acquisition_rows = vec![
            vec![AcqResult {
                sat,
                signal_band: SignalBand::L1,
                signal_code: SignalCode::Ca,
                glonass_frequency_channel: None,
                source_time: ReceiverSampleTrace::default(),
                candidate_rank: 1,
                is_primary_candidate: true,
                doppler_hz: Hertz(-1_250.0),
                doppler_rate_hz_per_s: 0.0,
                carrier_hz: Hertz(-1_250.0),
                code_phase_samples: 321,
                peak: 8.0,
                second_peak: 4.0,
                mean: 1.0,
                peak_mean_ratio: 8.0,
                peak_second_ratio: 2.0,
                cn0_proxy: 0.0,
                score: 0.75,
                hypothesis: AcqHypothesis::Accepted,
                assumptions: None,
                evidence: Vec::new(),
                threshold_provenance: None,
                explain_selection_reason: None,
                doppler_refinement: None,
                code_phase_refinement: None,
                signal_delay_alignment: None,
                uncertainty: Some(AcqUncertainty {
                    doppler_hz: 125.0,
                    code_phase_samples: 0.5,
                    doppler_rate_hz_per_s: None,
                    covariance: None,
                }),
            }],
            vec![result_for_bias_test(9, AcqHypothesis::Rejected, 0.0)],
        ];

        let follow_up_requests =
            build_related_signal_follow_up_requests(&config, &requests, &acquisition_rows);

        assert_eq!(follow_up_requests.len(), 1);
        let follow_up = &follow_up_requests[0];
        let expected_doppler_hz = shared_path_doppler_hz(
            -1_250.0,
            signal_spec_gps_l1_ca(),
            signal_spec_gps_l5_i(),
        )
        .expect("scaled same-satellite Doppler");

        assert_eq!(follow_up.request_index, 1);
        assert_eq!(follow_up.source_request_index, 0);
        assert_eq!(follow_up.source_signal_band, SignalBand::L1);
        assert_eq!(follow_up.source_signal_code, SignalCode::Ca);
        assert!((follow_up.estimated_signal_doppler_hz - expected_doppler_hz).abs() <= 1.0e-12);
        assert!((follow_up.transferred_code_phase_samples - 321.0).abs() <= 1.0e-12);
        assert!(
            (follow_up.request.doppler_center_hz - expected_doppler_hz).abs() <= 1.0e-12
        );
        let bounds = follow_up.request.assistance_bounds.expect("cross-band assistance bounds");
        assert!((bounds.expected_code_phase_samples - 321.0).abs() <= 1.0e-12);
        assert!((bounds.time_uncertainty_s - (0.5 / config.sampling_freq_hz)).abs() <= 1.0e-12);
        assert!(bounds.oscillator_uncertainty_hz > 0.0);
    }

    #[test]
    fn rejects_related_signal_follow_up_for_different_satellite_or_same_band_rows() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 10_230_000.0,
            ..ReceiverPipelineConfig::default()
        };
        let requests = vec![
            AcqRequest {
                sat: SatId { constellation: Constellation::Gps, prn: 9 },
                glonass_frequency_channel: None,
                signal_band: SignalBand::L1,
                signal_code: SignalCode::Ca,
                doppler_center_hz: 0.0,
                doppler_rate_center_hz_per_s: 0.0,
                expected_line_of_sight_doppler_hz: None,
                assistance_bounds: None,
                doppler_search_hz: 4_000,
                doppler_step_hz: 250,
                doppler_rate_search_hz_per_s: 0,
                doppler_rate_step_hz_per_s: 250,
                coherent_ms: 1,
                noncoherent: 1,
            },
            AcqRequest {
                sat: SatId { constellation: Constellation::Gps, prn: 10 },
                glonass_frequency_channel: None,
                signal_band: SignalBand::L5,
                signal_code: SignalCode::L5I,
                doppler_center_hz: 0.0,
                doppler_rate_center_hz_per_s: 0.0,
                expected_line_of_sight_doppler_hz: None,
                assistance_bounds: None,
                doppler_search_hz: 4_000,
                doppler_step_hz: 250,
                doppler_rate_search_hz_per_s: 0,
                doppler_rate_step_hz_per_s: 250,
                coherent_ms: 1,
                noncoherent: 1,
            },
            AcqRequest {
                sat: SatId { constellation: Constellation::Galileo, prn: 12 },
                glonass_frequency_channel: None,
                signal_band: SignalBand::E1,
                signal_code: SignalCode::E1C,
                doppler_center_hz: 0.0,
                doppler_rate_center_hz_per_s: 0.0,
                expected_line_of_sight_doppler_hz: None,
                assistance_bounds: None,
                doppler_search_hz: 4_000,
                doppler_step_hz: 250,
                doppler_rate_search_hz_per_s: 0,
                doppler_rate_step_hz_per_s: 250,
                coherent_ms: 1,
                noncoherent: 1,
            },
            AcqRequest {
                sat: SatId { constellation: Constellation::Galileo, prn: 12 },
                glonass_frequency_channel: None,
                signal_band: SignalBand::E1,
                signal_code: SignalCode::E1B,
                doppler_center_hz: 0.0,
                doppler_rate_center_hz_per_s: 0.0,
                expected_line_of_sight_doppler_hz: None,
                assistance_bounds: None,
                doppler_search_hz: 4_000,
                doppler_step_hz: 250,
                doppler_rate_search_hz_per_s: 0,
                doppler_rate_step_hz_per_s: 250,
                coherent_ms: 1,
                noncoherent: 1,
            },
        ];
        let acquisition_rows = vec![
            vec![result_for_bias_test(9, AcqHypothesis::Accepted, -750.0)],
            vec![result_for_bias_test(10, AcqHypothesis::Rejected, 0.0)],
            vec![result_for_bias_test(12, AcqHypothesis::Rejected, 0.0)],
            vec![AcqResult {
                sat: SatId { constellation: Constellation::Galileo, prn: 12 },
                signal_band: SignalBand::E1,
                signal_code: SignalCode::E1B,
                glonass_frequency_channel: None,
                source_time: ReceiverSampleTrace::default(),
                candidate_rank: 1,
                is_primary_candidate: true,
                doppler_hz: Hertz(300.0),
                doppler_rate_hz_per_s: 0.0,
                carrier_hz: Hertz(300.0),
                code_phase_samples: 200,
                peak: 7.0,
                second_peak: 3.0,
                mean: 1.0,
                peak_mean_ratio: 7.0,
                peak_second_ratio: 2.0,
                cn0_proxy: 0.0,
                score: 0.65,
                hypothesis: AcqHypothesis::Accepted,
                assumptions: None,
                evidence: Vec::new(),
                threshold_provenance: None,
                explain_selection_reason: None,
                doppler_refinement: None,
                code_phase_refinement: None,
                signal_delay_alignment: None,
                uncertainty: Some(AcqUncertainty {
                    doppler_hz: 50.0,
                    code_phase_samples: 0.5,
                    doppler_rate_hz_per_s: None,
                    covariance: None,
                }),
            }],
        ];

        let follow_up_requests =
            build_related_signal_follow_up_requests(&config, &requests, &acquisition_rows);

        assert!(follow_up_requests.is_empty(), "{follow_up_requests:#?}");
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
            doppler_rate_center_hz_per_s: 0.0,
            doppler_rate_search_hz_per_s: 0,
            doppler_rate_step_hz_per_s: 250,
            expected_line_of_sight_doppler_hz,
            assistance_bounds: None,
            doppler_search_hz,
            doppler_step_hz: 250,
            coherent_ms: 1,
            noncoherent: 1,
        }
    }

    fn request_for_bounds_test(
        prn: u8,
        doppler_search_hz: i32,
        assistance_bounds: Option<AcqAssistanceBounds>,
    ) -> AcqRequest {
        AcqRequest {
            sat: SatId { constellation: Constellation::Gps, prn },
            glonass_frequency_channel: None,
            signal_band: SignalBand::L1,
            signal_code: SignalCode::Ca,
            doppler_center_hz: 0.0,
            doppler_rate_center_hz_per_s: 0.0,
            doppler_rate_search_hz_per_s: 0,
            doppler_rate_step_hz_per_s: 250,
            expected_line_of_sight_doppler_hz: Some(0.0),
            assistance_bounds,
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
            doppler_rate_hz_per_s: 0.0,
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
            uncertainty: Some(AcqUncertainty {
                doppler_hz: 50.0,
                code_phase_samples: 0.5,
                doppler_rate_hz_per_s: None,
                covariance: None,
            }),
        }
    }
}
