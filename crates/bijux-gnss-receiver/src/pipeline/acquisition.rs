#![allow(missing_docs)]

use std::collections::HashMap;
use std::sync::Mutex;

#[cfg(test)]
use bijux_gnss_core::api::stable_acq_result_keys;
#[cfg(test)]
use bijux_gnss_core::api::AcqThresholdProvenance;
#[cfg(test)]
use bijux_gnss_core::api::SignalCode;
use bijux_gnss_core::api::{
    acq_result_stability_key, AcqEvidence, AcqExplain, AcqHypothesis, AcqRequest, AcqResult, Hertz,
    ReceiverSampleTrace, SamplesFrame, SatId,
};
#[cfg(test)]
use num_complex::Complex;
use rustfft::FftPlanner;

use crate::engine::receiver_config::{
    acquisition_integration_ms_is_supported, ReceiverPipelineConfig,
};
use crate::engine::runtime::{ReceiverRuntime, TraceRecord};
use crate::pipeline::acquisition_assistance::{
    build_related_signal_follow_up_requests, resolve_acquisition_search_bounds,
};
use crate::pipeline::acquisition_components::acquisition_strategies_for_signal;
#[cfg(test)]
use crate::pipeline::acquisition_components::AcquisitionComponentPlan;
#[cfg(test)]
use crate::pipeline::acquisition_symbol_hypotheses::coherent_data_sign_hypotheses;
use crate::pipeline::doppler::carrier_hz_from_doppler_hz;
#[cfg(test)]
use bijux_gnss_signal::api::SignalError;
use bijux_gnss_signal::api::{measure_iq_front_end_metrics, AcquisitionSignalModel};

mod peak_metrics;

use cache::{CodeFftCache, CodeFftRequest};
use candidate_decision::{
    acquisition_decision, multipath_candidate_reason, multipath_suspect_decision,
    ranked_alternative_candidate_reason, selected_candidate_reason,
};
#[cfg(test)]
use candidate_decision::{
    selected_reason_for_candidate, AcquisitionDecision, AcquisitionDecisionReason,
};
use candidate_failures::{
    acquisition_request_error_candidates, insufficient_frame_candidates,
    unsupported_coherent_integration_candidates, AcquisitionCandidateContext,
};
use candidate_ranking::competing_candidate_ratio;
use candidate_refinement::refine_acquisition_candidates;
#[cfg(test)]
use code_phase_profile::{
    estimate_parabolic_code_phase_offset_samples, wrap_acquisition_code_phase_samples,
};
use code_phase_profile::{measure_code_phase_profile, CodePhaseProfileRequest};
use correlation_accumulation::{
    accumulate_component_correlations, combine_component_accumulations, ComponentCorrelationRequest,
};
#[cfg(test)]
use correlation_accumulation::{
    best_coherent_data_correlation, best_coherent_secondary_code_phase_correlation,
    coherent_correlation_with_signs,
};
#[cfg(test)]
use doppler_refinement::estimate_acquisition_doppler_refinement;
#[cfg(test)]
use false_alarm_calibration::noise_only_frame;
use front_end_rejection::zero_signal_run;
#[cfg(test)]
use likelihood_covariance::{
    estimate_log_likelihood_covariance_2x2, estimate_log_likelihood_covariance_3x3,
};
#[cfg(test)]
use likelihood_covariance::{LocalAcquisitionLikelihoodSurface, LocalAcquisitionLikelihoodVolume};
#[cfg(test)]
use likelihood_measurement::estimate_quadratic_surface_peak_offsets;
use peak_metrics::{
    correlation_metrics, correlation_metrics_in_window, delayed_secondary_peak_diagnostic,
    DelayedSecondaryPeakDiagnostic,
};
use related_signal_follow_up::{
    annotate_related_signal_follow_up_candidates, annotate_related_signal_follow_up_explain,
    should_replace_related_signal_row,
};
use search_window::{
    append_assisted_search_fallback_reason, assisted_code_phase_search_window_diagnostic,
    assisted_search_fallback_reason, search_window_candidate_reason, should_retry_assisted_search,
    signal_outside_doppler_rate_search_range, signal_outside_search_range,
    AssistedCodePhaseWindowDiagnosticRequest,
};
#[cfg(test)]
use search_window::{SearchWindowDimension, SearchWindowEdge};
#[cfg(test)]
use signal_model::acquisition_signal_model_for_request;
#[cfg(test)]
use signal_model::acquisition_signal_model_for_sat;
use signal_model::{
    request_search_center_hz, resolved_request_signal_code, unsupported_acquisition_signal_error,
};
use strategy_components::{
    candidate_uses_data_sign_hypotheses, strategy_component_indexes, strategy_component_provenance,
    strategy_supports_search_model_refinement, strategy_uses_data_sign_hypotheses,
    unique_strategy_components,
};
use threshold_resolution::{
    threshold_provenance_for_request, ResolvedAcquisitionThresholds, ThresholdResolutionCache,
};
use uncertainty::{estimate_acquisition_uncertainty, AcquisitionUncertaintyRequest};
use wrong_prn_suppression::{suppress_wrong_prn_correlations, AcquisitionSatEvaluation};

mod cache;
mod candidate_decision;
mod candidate_failures;
mod candidate_ranking;
mod candidate_refinement;
mod code_phase_profile;
mod code_phase_refinement;
mod correlation_accumulation;
mod doppler_refinement;
mod false_alarm_calibration;
mod front_end_rejection;
mod likelihood_covariance;
mod likelihood_measurement;
mod related_signal_follow_up;
mod request_planning;
mod result_reporting;
mod search_window;
mod signal_model;
mod strategy_components;
mod threshold_calibration;
mod threshold_resolution;
mod uncertainty;
mod wrong_prn_suppression;

/// Acquisition engine (coarse search).
pub struct Acquisition {
    config: ReceiverPipelineConfig,
    runtime: ReceiverRuntime,
    doppler_search_hz: i32,
    doppler_step_hz: i32,
    cache: Mutex<CodeFftCache>,
    threshold_cache: Mutex<ThresholdResolutionCache>,
    stats: Mutex<AcquisitionStats>,
}

#[derive(Debug, Default, Clone, Copy)]
pub struct AcquisitionStats {
    pub sat_count: u64,
    pub doppler_bins: u64,
    pub code_search_bins: u64,
    pub cache_hits: u64,
    pub cache_misses: u64,
    pub cache_miss_cold_start: u64,
    pub cache_miss_incompatible: u64,
    pub accepted_count: u64,
    pub ambiguous_count: u64,
    pub rejected_count: u64,
    pub deferred_count: u64,
}

#[derive(Debug, Clone)]
pub struct AcquisitionRun {
    pub results: Vec<Vec<AcqResult>>,
    pub explains: Vec<AcqExplain>,
}

impl Acquisition {
    fn with_stats<F, R>(&self, f: F) -> R
    where
        F: FnOnce(&mut AcquisitionStats) -> R,
    {
        match self.stats.lock() {
            Ok(mut guard) => f(&mut guard),
            Err(_) => f(&mut AcquisitionStats::default()),
        }
    }

    pub fn stats_snapshot(&self) -> AcquisitionStats {
        match self.stats.lock() {
            Ok(stats) => *stats,
            Err(_) => AcquisitionStats::default(),
        }
    }

    pub fn new(config: ReceiverPipelineConfig, runtime: ReceiverRuntime) -> Self {
        let doppler_step_hz = config.acquisition_doppler_step_hz.max(1);
        Self {
            config: config.clone(),
            runtime,
            doppler_search_hz: config.acquisition_doppler_search_hz,
            doppler_step_hz,
            cache: Mutex::new(HashMap::new()),
            threshold_cache: Mutex::new(HashMap::new()),
            stats: Mutex::new(AcquisitionStats::default()),
        }
    }

    pub fn with_doppler(mut self, search_hz: i32, step_hz: i32) -> Self {
        self.doppler_search_hz = search_hz.abs();
        self.doppler_step_hz = step_hz.max(1);
        self
    }

    /// Perform satellite acquisition on a buffer that spans the configured integration window.
    pub fn run_fft(&self, frame: &SamplesFrame, sats: &[SatId]) -> Vec<AcqResult> {
        self.run_fft_topn_for_requests(
            frame,
            &self.default_requests_for_sats(
                sats,
                self.config.acquisition_integration_ms,
                self.config.acquisition_noncoherent,
            ),
            1,
        )
        .into_iter()
        .filter_map(|mut v| if v.is_empty() { None } else { Some(v.remove(0)) })
        .collect()
    }

    /// Perform acquisition for explicit request rows.
    pub fn run_fft_for_requests(
        &self,
        frame: &SamplesFrame,
        requests: &[AcqRequest],
    ) -> Vec<AcqResult> {
        self.run_fft_topn_for_requests(frame, requests, 1)
            .into_iter()
            .filter_map(|mut rows| if rows.is_empty() { None } else { Some(rows.remove(0)) })
            .collect()
    }

    pub fn run_fft_topn_with_explain(
        &self,
        frame: &SamplesFrame,
        sats: &[SatId],
        top_n: usize,
        coherent_ms: u32,
        noncoherent: u32,
    ) -> AcquisitionRun {
        self.run_fft_topn_for_requests_with_explain(
            frame,
            &self.default_requests_for_sats(sats, coherent_ms, noncoherent),
            top_n,
        )
    }

    pub fn run_fft_topn(
        &self,
        frame: &SamplesFrame,
        sats: &[SatId],
        top_n: usize,
        coherent_ms: u32,
        noncoherent: u32,
    ) -> Vec<Vec<AcqResult>> {
        self.run_fft_topn_for_requests(
            frame,
            &self.default_requests_for_sats(sats, coherent_ms, noncoherent),
            top_n,
        )
    }

    /// Perform acquisition for explicit request rows and return explain artifacts.
    pub fn run_fft_topn_for_requests_with_explain(
        &self,
        frame: &SamplesFrame,
        requests: &[AcqRequest],
        top_n: usize,
    ) -> AcquisitionRun {
        self.run_fft_topn_internal(frame, requests, top_n, true, true)
    }

    /// Perform acquisition for explicit request rows.
    pub fn run_fft_topn_for_requests(
        &self,
        frame: &SamplesFrame,
        requests: &[AcqRequest],
        top_n: usize,
    ) -> Vec<Vec<AcqResult>> {
        self.run_fft_topn_internal(frame, requests, top_n, false, true).results
    }

    fn run_fft_topn_internal(
        &self,
        frame: &SamplesFrame,
        requests: &[AcqRequest],
        top_n: usize,
        emit_explanations: bool,
        allow_related_signal_follow_up: bool,
    ) -> AcquisitionRun {
        let mut run = self.run_fft_topn_single_pass(frame, requests, top_n, emit_explanations);
        if !allow_related_signal_follow_up || requests.len() < 2 {
            return run;
        }
        let follow_up_requests =
            build_related_signal_follow_up_requests(&self.config, requests, &run.results);
        for follow_up_request in follow_up_requests {
            let follow_up_run = self.run_fft_topn_single_pass(
                frame,
                &[follow_up_request.request],
                top_n,
                emit_explanations,
            );
            let Some(replacement_candidates) = follow_up_run.results.into_iter().next() else {
                continue;
            };
            if !should_replace_related_signal_row(
                run.results.get(follow_up_request.request_index).map(Vec::as_slice).unwrap_or(&[]),
                &replacement_candidates,
            ) {
                continue;
            }
            self.runtime.trace.record(TraceRecord {
                name: "acquisition_related_signal_follow_up",
                fields: vec![
                    ("constellation", format!("{:?}", follow_up_request.request.sat.constellation)),
                    ("prn", follow_up_request.request.sat.prn.to_string()),
                    ("source_band", format!("{:?}", follow_up_request.source_signal_band)),
                    ("source_code", format!("{:?}", follow_up_request.source_signal_code)),
                    ("target_band", format!("{:?}", follow_up_request.request.signal_band)),
                    ("target_code", format!("{:?}", follow_up_request.request.signal_code)),
                    (
                        "doppler_center_hz",
                        format!("{:.6}", follow_up_request.estimated_signal_doppler_hz),
                    ),
                    (
                        "code_phase_samples",
                        format!("{:.6}", follow_up_request.transferred_code_phase_samples),
                    ),
                ],
            });
            let mut replacement_candidates = replacement_candidates;
            annotate_related_signal_follow_up_candidates(
                &mut replacement_candidates,
                &follow_up_request,
            );
            run.results[follow_up_request.request_index] = replacement_candidates;
            if emit_explanations {
                if let Some(replacement_explain) = follow_up_run.explains.into_iter().next() {
                    run.explains[follow_up_request.request_index] =
                        annotate_related_signal_follow_up_explain(
                            replacement_explain,
                            &follow_up_request,
                        );
                }
            }
        }
        run
    }

    fn run_fft_topn_single_pass(
        &self,
        frame: &SamplesFrame,
        requests: &[AcqRequest],
        top_n: usize,
        emit_explanations: bool,
    ) -> AcquisitionRun {
        let mut planner = FftPlanner::<f32>::new();

        self.with_stats(|stats| {
            stats.sat_count += requests.len() as u64;
        });
        let front_end_metrics = measure_iq_front_end_metrics(&frame.iq);
        if front_end_metrics.zero_signal_detected {
            self.runtime.trace.record(TraceRecord {
                name: "acquisition_front_end_rejection",
                fields: vec![
                    ("reason", "zero_signal_input".to_string()),
                    ("sample_count", frame.len().to_string()),
                    ("centered_rms", format!("{:.9}", front_end_metrics.centered_rms)),
                ],
            });
            self.with_stats(|stats| {
                stats.rejected_count = stats.rejected_count.saturating_add(requests.len() as u64);
            });
            return zero_signal_run(
                &self.config,
                requests,
                ReceiverSampleTrace::from_sample_time(frame.t0),
                frame.len(),
                front_end_metrics.zero_signal_reason.as_deref(),
                emit_explanations,
            );
        }

        let mut sat_evaluations = Vec::new();
        for &request in requests {
            let sat = request.sat;
            let requested_threshold_provenance =
                threshold_provenance_for_request(&self.config, request);
            let signal_code = resolved_request_signal_code(request);
            let strategies = match acquisition_strategies_for_signal(
                sat,
                request.signal_band,
                signal_code,
                request.glonass_frequency_channel,
                request.coherent_ms,
            ) {
                Ok(strategies) => strategies,
                Err(error) => {
                    sat_evaluations.push(AcquisitionSatEvaluation {
                        sat,
                        candidates: acquisition_request_error_candidates(
                            &self.config,
                            request,
                            &requested_threshold_provenance,
                            ReceiverSampleTrace::from_sample_time(frame.t0),
                            frame.len(),
                            error,
                        ),
                        search_window_diagnostic: None,
                    });
                    continue;
                }
            };
            let signal_model = match strategies.first() {
                Some(strategy) => strategy.search_model.clone(),
                None => {
                    sat_evaluations.push(AcquisitionSatEvaluation {
                        sat,
                        candidates: acquisition_request_error_candidates(
                            &self.config,
                            request,
                            &requested_threshold_provenance,
                            ReceiverSampleTrace::from_sample_time(frame.t0),
                            frame.len(),
                            unsupported_acquisition_signal_error(
                                request.sat,
                                request.signal_band,
                                request.signal_code,
                            ),
                        ),
                        search_window_diagnostic: None,
                    });
                    continue;
                }
            };
            let strategy_components = unique_strategy_components(&strategies);
            let strategy_component_indexes = strategies
                .iter()
                .map(|strategy| strategy_component_indexes(strategy, &strategy_components))
                .collect::<Vec<_>>();
            let search_center_hz =
                request_search_center_hz(&signal_model, self.config.intermediate_freq_hz, request);
            let samples_per_code = signal_model.samples_per_code(self.config.sampling_freq_hz);
            let resolved_bounds =
                resolve_acquisition_search_bounds(&self.config, &signal_model, request);
            let resolved_request =
                AcqRequest { doppler_search_hz: resolved_bounds.doppler_search_hz, ..request };
            self.with_stats(|stats| {
                stats.doppler_bins += doppler_bin_count(
                    resolved_request.doppler_search_hz,
                    resolved_request.doppler_step_hz.max(1),
                );
            });
            let assumptions =
                self.search_assumptions(frame.len(), request, &resolved_bounds, samples_per_code);
            let candidate_context = AcquisitionCandidateContext {
                sat,
                signal_model: &signal_model,
                signal_code,
                glonass_frequency_channel: request.glonass_frequency_channel,
                assumptions: &assumptions,
                threshold_provenance: &requested_threshold_provenance,
                intermediate_freq_hz: search_center_hz,
                source_time: ReceiverSampleTrace::from_sample_time(frame.t0),
            };
            if !acquisition_integration_ms_is_supported(request.coherent_ms) {
                sat_evaluations.push(AcquisitionSatEvaluation {
                    sat,
                    candidates: unsupported_coherent_integration_candidates(
                        candidate_context,
                        request.coherent_ms,
                    ),
                    search_window_diagnostic: None,
                });
                continue;
            }
            let coherent_periods = match signal_model.coherent_periods(request.coherent_ms) {
                Some(periods) => periods,
                None => {
                    sat_evaluations.push(AcquisitionSatEvaluation {
                        sat,
                        candidates: unsupported_coherent_integration_candidates(
                            candidate_context,
                            request.coherent_ms,
                        ),
                        search_window_diagnostic: None,
                    });
                    continue;
                }
            };
            let required = samples_per_code
                * coherent_periods.max(1) as usize
                * request.noncoherent.max(1) as usize;
            if frame.len() < required {
                sat_evaluations.push(AcquisitionSatEvaluation {
                    sat,
                    candidates: insufficient_frame_candidates(
                        candidate_context,
                        frame.len(),
                        required,
                    ),
                    search_window_diagnostic: None,
                });
                continue;
            }
            let resolved_thresholds =
                self.resolve_thresholds_for_request(resolved_request, &signal_model);
            self.runtime.trace.record(TraceRecord {
                name: "acquisition_sat_start",
                fields: vec![
                    ("constellation", format!("{:?}", sat.constellation)),
                    ("prn", sat.prn.to_string()),
                ],
            });
            self.with_stats(|stats| {
                stats.code_search_bins = stats
                    .code_search_bins
                    .saturating_add(resolved_bounds.code_phase_search_bins as u64);
            });
            let fft = planner.plan_fft_forward(samples_per_code);
            let ifft = planner.plan_fft_inverse(samples_per_code);
            let component_ffts = strategy_components
                .iter()
                .map(|component| {
                    self.code_fft(CodeFftRequest {
                        signal_model: &signal_model,
                        component,
                        sat,
                        signal_code,
                        samples_per_code,
                        fft: fft.as_ref(),
                    })
                })
                .collect::<Vec<_>>();
            let mut grid_candidates = Vec::new();
            let mut doppler_rate = -resolved_request.doppler_rate_search_hz_per_s;
            while doppler_rate <= resolved_request.doppler_rate_search_hz_per_s {
                let absolute_doppler_rate_hz_per_s =
                    request.doppler_rate_center_hz_per_s + doppler_rate as f64;
                let mut doppler = -resolved_request.doppler_search_hz;
                while doppler <= resolved_request.doppler_search_hz {
                    let absolute_doppler_hz = request.doppler_center_hz + doppler as f64;
                    let carrier = carrier_hz_from_doppler_hz(search_center_hz, doppler as f64);
                    let mut component_accumulations = Vec::with_capacity(strategy_components.len());

                    for (component, code_fft) in
                        strategy_components.iter().zip(component_ffts.iter())
                    {
                        let accumulation =
                            accumulate_component_correlations(ComponentCorrelationRequest {
                                frame,
                                component,
                                code_fft,
                                carrier_hz: carrier,
                                doppler_rate_hz_per_s: absolute_doppler_rate_hz_per_s,
                                sample_rate_hz: self.config.sampling_freq_hz,
                                samples_per_code,
                                coherent_periods,
                                noncoherent: request.noncoherent,
                                fft: fft.as_ref(),
                                ifft: ifft.as_ref(),
                            });
                        component_accumulations.push(accumulation);
                    }

                    for (strategy, component_indexes) in
                        strategies.iter().zip(strategy_component_indexes.iter())
                    {
                        let combined_accumulator = combine_component_accumulations(
                            strategy.combination_mode,
                            component_indexes,
                            &component_accumulations,
                            samples_per_code,
                            request.noncoherent,
                        );

                        let correlation_metrics = correlation_metrics_in_window(
                            &combined_accumulator,
                            resolved_bounds.code_phase_search_start_sample,
                            resolved_bounds.code_phase_search_step_samples,
                            resolved_bounds.code_phase_search_bins,
                        );
                        let peak_mean_ratio =
                            correlation_metrics.peak / (correlation_metrics.mean + 1e-6);
                        let peak_second_ratio =
                            correlation_metrics.peak / (correlation_metrics.second + 1e-6);
                        let cn0_proxy = peak_mean_ratio * 10.0;
                        let component_provenance = strategy_component_provenance(
                            strategy,
                            component_indexes,
                            &component_accumulations,
                        );
                        grid_candidates.push(AcqResult {
                            sat,
                            signal_band: signal_model.signal_band,
                            signal_code,
                            glonass_frequency_channel: request.glonass_frequency_channel,
                            source_time: ReceiverSampleTrace::from_sample_time(frame.t0),
                            candidate_rank: 1,
                            is_primary_candidate: true,
                            doppler_hz: Hertz(absolute_doppler_hz),
                            doppler_rate_hz_per_s: absolute_doppler_rate_hz_per_s,
                            carrier_hz: Hertz(carrier),
                            code_phase_samples: correlation_metrics.peak_idx,
                            peak: correlation_metrics.peak,
                            second_peak: correlation_metrics.second,
                            mean: correlation_metrics.mean,
                            peak_mean_ratio,
                            peak_second_ratio,
                            cn0_proxy,
                            score: 0.0,
                            hypothesis: AcqHypothesis::Deferred,
                            assumptions: Some(assumptions.clone()),
                            evidence: vec![AcqEvidence {
                                rank: 1,
                                code_phase_samples: correlation_metrics.peak_idx,
                                doppler_hz: absolute_doppler_hz,
                                doppler_rate_hz_per_s: absolute_doppler_rate_hz_per_s,
                                peak: correlation_metrics.peak,
                                second_peak: correlation_metrics.second,
                                peak_mean_ratio,
                                peak_second_ratio,
                                mean: correlation_metrics.mean,
                                component_provenance: Some(component_provenance),
                            }],
                            threshold_provenance: Some(resolved_thresholds.provenance.clone()),
                            explain_selection_reason: None,
                            doppler_refinement: None,
                            code_phase_refinement: None,
                            signal_delay_alignment: None,
                            uncertainty: None,
                        });
                    }

                    doppler += resolved_request.doppler_step_hz.max(1);
                }

                doppler_rate += resolved_request.doppler_rate_step_hz_per_s.max(1);
            }

            let doppler_search_window_diagnostic = signal_outside_search_range(
                &grid_candidates,
                search_center_hz,
                resolved_request.doppler_search_hz,
                resolved_request.doppler_step_hz.max(1),
                resolved_thresholds.peak_mean_threshold,
            );

            let mut ranked_candidates = grid_candidates.clone();
            ranked_candidates.sort_by(|a, b| {
                let primary = b
                    .peak_mean_ratio
                    .partial_cmp(&a.peak_mean_ratio)
                    .unwrap_or(std::cmp::Ordering::Equal);
                if primary == std::cmp::Ordering::Equal {
                    return acq_result_stability_key(a).cmp(&acq_result_stability_key(b));
                }
                primary
            });
            let competing_peak_ratio = competing_candidate_ratio(&ranked_candidates);
            let mut candidates = ranked_candidates;
            candidates.truncate(top_n.max(1));
            if strategies.len() == 1
                && strategy_supports_search_model_refinement(&strategies[0])
                && !strategy_uses_data_sign_hypotheses(&strategies[0], coherent_periods)
            {
                refine_acquisition_candidates(
                    candidate_refinement::CandidateRefinementRequest {
                        acquisition: self,
                        frame,
                        signal_model: &signal_model,
                        grid_candidates: &grid_candidates,
                        doppler_step_hz: request.doppler_step_hz.max(1),
                        coherent_ms: request.coherent_ms,
                        noncoherent: request.noncoherent,
                    },
                    &mut candidates,
                );
            }
            if candidates.is_empty() {
                if should_retry_assisted_search(
                    request,
                    &resolved_bounds,
                    None,
                    doppler_search_window_diagnostic.as_ref(),
                ) {
                    let fallback_reason = assisted_search_fallback_reason(
                        None,
                        doppler_search_window_diagnostic.as_ref(),
                    );
                    let fallback_request = AcqRequest { assistance_bounds: None, ..request };
                    let mut fallback_candidates = self
                        .run_fft_topn_for_requests_with_explain(frame, &[fallback_request], top_n)
                        .results
                        .into_iter()
                        .next()
                        .unwrap_or_default();
                    append_assisted_search_fallback_reason(
                        &mut fallback_candidates,
                        &fallback_reason,
                    );
                    sat_evaluations.push(AcquisitionSatEvaluation {
                        sat,
                        candidates: fallback_candidates,
                        search_window_diagnostic: None,
                    });
                    continue;
                }
                sat_evaluations.push(AcquisitionSatEvaluation {
                    sat,
                    candidates: Vec::new(),
                    search_window_diagnostic: None,
                });
                continue;
            }
            let mut selected_search_window_diagnostic = doppler_search_window_diagnostic.clone();
            for (rank, candidate) in candidates.iter_mut().enumerate() {
                candidate.candidate_rank = rank as u8 + 1;
                candidate.is_primary_candidate = rank == 0;
                if let Some(evidence) = candidate.evidence.first_mut() {
                    evidence.rank = candidate.candidate_rank;
                    evidence.code_phase_samples = candidate.code_phase_samples;
                    evidence.doppler_hz = candidate.doppler_hz.0;
                    evidence.doppler_rate_hz_per_s = candidate.doppler_rate_hz_per_s;
                    evidence.peak = candidate.peak;
                    evidence.second_peak = candidate.second_peak;
                    evidence.peak_mean_ratio = candidate.peak_mean_ratio;
                    evidence.peak_second_ratio = candidate.peak_second_ratio;
                    evidence.mean = candidate.mean;
                }
                let local_peak_separation_ratio = candidate.peak_second_ratio;
                if rank == 0 {
                    let search_window_diagnostic = selected_search_window_diagnostic
                        .clone()
                        .or_else(|| {
                            assisted_code_phase_search_window_diagnostic(
                                AssistedCodePhaseWindowDiagnosticRequest {
                                    config: &self.config,
                                    signal_model: &signal_model,
                                    frame,
                                    carrier_hz: candidate.carrier_hz.0,
                                    doppler_rate_hz_per_s: candidate.doppler_rate_hz_per_s,
                                    coherent_ms: request.coherent_ms,
                                    noncoherent: request.noncoherent,
                                    resolved_bounds: &resolved_bounds,
                                    peak_mean_threshold: resolved_thresholds.peak_mean_threshold,
                                },
                            )
                        })
                        .or_else(|| {
                            signal_outside_doppler_rate_search_range(
                                &grid_candidates,
                                candidate
                                    .doppler_refinement
                                    .as_ref()
                                    .map(|refinement| refinement.coarse_carrier_hz.0)
                                    .unwrap_or(candidate.carrier_hz.0),
                                request.doppler_rate_center_hz_per_s,
                                resolved_request.doppler_rate_search_hz_per_s,
                                resolved_request.doppler_rate_step_hz_per_s.max(1),
                                resolved_thresholds.peak_mean_threshold,
                            )
                        });
                    if let Some(diagnostic) = search_window_diagnostic.as_ref() {
                        selected_search_window_diagnostic = Some(diagnostic.clone());
                        candidate.hypothesis = AcqHypothesis::Rejected;
                        candidate.score = 0.0;
                        candidate.explain_selection_reason =
                            Some(search_window_candidate_reason(diagnostic));
                    } else {
                        let decision = acquisition_decision(
                            candidate.peak_mean_ratio,
                            candidate.peak_second_ratio,
                            local_peak_separation_ratio,
                            competing_peak_ratio,
                            &resolved_thresholds,
                        );
                        let multipath_diagnostic = if candidate_uses_data_sign_hypotheses(
                            candidate,
                            &strategies,
                            coherent_periods,
                        ) {
                            None
                        } else {
                            classify_delayed_secondary_peak(DelayedSecondaryPeakRequest {
                                config: &self.config,
                                frame,
                                signal_model: &signal_model,
                                carrier_hz: candidate.carrier_hz.0,
                                doppler_rate_hz_per_s: candidate.doppler_rate_hz_per_s,
                                code_phase_samples: candidate.code_phase_samples,
                                samples_per_code,
                                coherent_ms: request.coherent_ms,
                                noncoherent: request.noncoherent,
                                peak_mean_ratio: candidate.peak_mean_ratio,
                                peak_second_ratio: candidate.peak_second_ratio,
                                competing_peak_ratio,
                                thresholds: &resolved_thresholds,
                            })
                        };
                        let decision =
                            multipath_diagnostic.as_ref().map_or(decision, |diagnostic| {
                                multipath_suspect_decision(
                                    candidate.peak_mean_ratio,
                                    candidate.peak_second_ratio,
                                    competing_peak_ratio,
                                    diagnostic,
                                )
                            });
                        candidate.hypothesis = decision.hypothesis;
                        candidate.score = decision.score;
                        candidate.explain_selection_reason =
                            Some(match multipath_diagnostic.as_ref() {
                                Some(diagnostic) => multipath_candidate_reason(
                                    candidate.peak_mean_ratio,
                                    candidate.peak_second_ratio,
                                    competing_peak_ratio,
                                    diagnostic,
                                    samples_per_code,
                                    signal_model.code_length,
                                    decision.score,
                                ),
                                None => selected_candidate_reason(
                                    decision,
                                    candidate.peak_mean_ratio,
                                    local_peak_separation_ratio,
                                    competing_peak_ratio,
                                    &resolved_thresholds,
                                ),
                            });
                        candidate.uncertainty =
                            estimate_acquisition_uncertainty(AcquisitionUncertaintyRequest {
                                config: &self.config,
                                frame,
                                signal_model: &signal_model,
                                candidate,
                                coherent_ms: request.coherent_ms,
                                noncoherent: request.noncoherent,
                                doppler_step_hz: request.doppler_step_hz.max(1),
                                doppler_rate_search_hz_per_s: request
                                    .doppler_rate_search_hz_per_s
                                    .max(0),
                                doppler_rate_step_hz_per_s: request
                                    .doppler_rate_step_hz_per_s
                                    .max(1),
                            });
                    }
                }
            }
            let primary_candidate = candidates
                .first()
                .cloned()
                .expect("retained acquisition candidates must include a primary row");
            if should_retry_assisted_search(
                request,
                &resolved_bounds,
                Some(&primary_candidate),
                selected_search_window_diagnostic.as_ref(),
            ) {
                let fallback_reason = assisted_search_fallback_reason(
                    Some(&primary_candidate),
                    selected_search_window_diagnostic.as_ref(),
                );
                let fallback_request = AcqRequest { assistance_bounds: None, ..request };
                let mut fallback_candidates = self
                    .run_fft_topn_for_requests_with_explain(frame, &[fallback_request], top_n)
                    .results
                    .into_iter()
                    .next()
                    .unwrap_or_default();
                append_assisted_search_fallback_reason(&mut fallback_candidates, &fallback_reason);
                sat_evaluations.push(AcquisitionSatEvaluation {
                    sat,
                    candidates: fallback_candidates,
                    search_window_diagnostic: None,
                });
                continue;
            }
            for candidate in candidates.iter_mut().skip(1) {
                candidate.hypothesis = AcqHypothesis::Rejected;
                candidate.score = 0.0;
                candidate.explain_selection_reason =
                    Some(ranked_alternative_candidate_reason(&primary_candidate, candidate));
            }
            sat_evaluations.push(AcquisitionSatEvaluation {
                sat,
                candidates,
                search_window_diagnostic: selected_search_window_diagnostic,
            });
        }
        suppress_wrong_prn_correlations(&mut sat_evaluations);

        self.report_satellite_evaluations(sat_evaluations, emit_explanations)
    }
}

fn doppler_bin_count(search_hz: i32, step_hz: i32) -> u64 {
    if step_hz <= 0 {
        return 0;
    }
    let search = search_hz.unsigned_abs() as u64;
    let step = step_hz.unsigned_abs() as u64;
    (search / step).saturating_mul(2).saturating_add(1)
}

struct DelayedSecondaryPeakRequest<'a> {
    config: &'a ReceiverPipelineConfig,
    frame: &'a SamplesFrame,
    signal_model: &'a AcquisitionSignalModel,
    carrier_hz: f64,
    doppler_rate_hz_per_s: f64,
    code_phase_samples: usize,
    samples_per_code: usize,
    coherent_ms: u32,
    noncoherent: u32,
    peak_mean_ratio: f32,
    peak_second_ratio: f32,
    competing_peak_ratio: f32,
    thresholds: &'a ResolvedAcquisitionThresholds,
}

fn classify_delayed_secondary_peak(
    request: DelayedSecondaryPeakRequest<'_>,
) -> Option<DelayedSecondaryPeakDiagnostic> {
    let DelayedSecondaryPeakRequest {
        config,
        frame,
        signal_model,
        carrier_hz,
        doppler_rate_hz_per_s,
        code_phase_samples,
        samples_per_code,
        coherent_ms,
        noncoherent,
        peak_mean_ratio,
        peak_second_ratio,
        competing_peak_ratio,
        thresholds,
    } = request;
    if peak_mean_ratio < thresholds.peak_mean_threshold
        || peak_second_ratio >= thresholds.peak_second_threshold
        || competing_peak_ratio < thresholds.peak_second_threshold
    {
        return None;
    }
    if !signal_model.supports_secondary_peak_multipath_screening() {
        return None;
    }
    let correlation_profile = measure_code_phase_profile(CodePhaseProfileRequest {
        config,
        signal_model,
        frame,
        carrier_hz,
        doppler_rate_hz_per_s,
        coherent_ms,
        noncoherent,
    })?;
    delayed_secondary_peak_diagnostic(
        &correlation_profile,
        code_phase_samples,
        samples_per_code,
        signal_model.code_length,
    )
}

#[cfg(test)]
#[path = "acquisition/tests.rs"]
mod tests;
