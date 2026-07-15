#![allow(missing_docs)]

use std::collections::HashMap;
use std::sync::Mutex;

use crate::engine::signal_selection::{
    default_signal_code_for_band, resolved_acquisition_signal_band,
};
use bijux_gnss_core::api::{
    acq_result_stability_key, stable_acq_result_keys, AcqAssumptions, AcqCodePhaseRefinement,
    AcqComponentCombinationMode, AcqDopplerRefinement, AcqEvidence, AcqExplain,
    AcqExplainCandidate, AcqHypothesis, AcqRequest, AcqResult, AcqThresholdProvenance,
    AcqUncertainty, AcqUncertaintyCovariance, Hertz, ReceiverSampleTrace, SamplesFrame, SatId,
    SignalCode,
};
use num_complex::Complex;
use rustfft::{num_traits::Zero, FftPlanner};

use crate::engine::receiver_config::{
    acquisition_integration_ms_is_supported, supported_acquisition_integration_ms_csv,
    AcquisitionThresholdMode, ReceiverPipelineConfig,
};
use crate::engine::runtime::{ReceiverRuntime, TraceRecord};
use crate::pipeline::acquisition_assistance::{
    build_related_signal_follow_up_requests, resolve_acquisition_search_bounds,
    ResolvedAcquisitionSearchBounds,
};
use crate::pipeline::acquisition_components::{
    acquisition_strategies_for_signal, AcquisitionComponentPlan,
};
use crate::pipeline::acquisition_symbol_hypotheses::coherent_secondary_code_phase_hypotheses;
use crate::pipeline::doppler::{carrier_hz_from_doppler_hz, doppler_hz_from_carrier_hz};
use bijux_gnss_signal::api::{
    measure_iq_front_end_metrics, samples_per_code, wipeoff_carrier,
    wipeoff_carrier_with_linear_rate, AcquisitionSignalModel, SignalError,
};

mod peak_metrics;

use cache::{
    CacheMissReason, CodeFftCache, CodeFftCacheKey, ACQUISITION_CACHE_MODEL_VERSION,
    ACQUISITION_CACHE_POLICY_VERSION,
};
use false_alarm_calibration::{
    calibration_seed, false_alarm_rate, mix_seed, noise_only_frame, wilson_confidence_interval,
    FalseAlarmRateMeasurement,
};
use peak_metrics::{
    correlation_metrics, correlation_metrics_in_window, delayed_secondary_peak_diagnostic,
    CorrelationMetrics, DelayedSecondaryPeakDiagnostic,
};
use related_signal_follow_up::{
    annotate_related_signal_follow_up_candidates, annotate_related_signal_follow_up_explain,
    should_replace_related_signal_row,
};
use search_window::{
    append_assisted_search_fallback_reason, assisted_search_fallback_reason,
    code_phase_outside_search_range, search_window_candidate_reason, should_retry_assisted_search,
    SearchWindowDiagnostic, SearchWindowDimension, SearchWindowEdge,
    SEARCH_EDGE_HINT_PEAK_MEAN_RATIO_FRACTION, SEARCH_EDGE_RISE_RATIO_EPSILON,
};
#[cfg(test)]
use signal_model::acquisition_signal_model_for_sat;
use signal_model::{
    acquisition_signal_model_for_request, request_search_center_hz, resolved_request_signal_code,
    resolved_signal_code, unsupported_acquisition_signal_error,
};
use strategy_components::{
    candidate_uses_data_sign_hypotheses, component_data_sign_hypotheses,
    strategy_component_indexes, strategy_component_provenance,
    strategy_supports_search_model_refinement, strategy_uses_data_sign_hypotheses,
    unique_strategy_components,
};
use threshold_resolution::{
    threshold_provenance_for_request, AcquisitionThresholdCacheKey, ResolvedAcquisitionThresholds,
    ThresholdResolutionCache,
};

mod cache;
mod false_alarm_calibration;
mod related_signal_follow_up;
mod search_window;
mod signal_model;
mod strategy_components;
mod threshold_resolution;

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

const JOINT_ACQUISITION_REFINEMENT_METHOD: &str = "quadratic_likelihood_surface";
const SUB_BIN_DOPPLER_REFINEMENT_METHOD: &str = "parabolic_peak";
const SUB_BIN_DOPPLER_REFINEMENT_MAX_OFFSET_BINS: f64 = 0.5;
const SUB_BIN_DOPPLER_REFINEMENT_EPSILON: f64 = 1e-12;
const SUB_SAMPLE_CODE_PHASE_REFINEMENT_METHOD: &str = "parabolic_code_peak";
const SUB_SAMPLE_CODE_PHASE_REFINEMENT_MAX_OFFSET_SAMPLES: f64 = 0.5;
const SUB_SAMPLE_CODE_PHASE_REFINEMENT_EPSILON: f64 = 1e-12;
const SUB_SAMPLE_CODE_PHASE_REFINEMENT_MIN_ABS_OFFSET_SAMPLES: f64 = 0.05;
const ACQUISITION_UNCERTAINTY_LOG_RESPONSE_FLOOR_RATIO: f64 = 1.0e-6;
const FALSE_ALARM_CALIBRATION_SEARCH_ITERATIONS: usize = 7;
const WRONG_PRN_DOMINANCE_RATIO_MIN: f32 = 4.0;
const WRONG_PRN_PEAK_SECOND_RATIO_MAX: f32 = 1.1;

#[derive(Debug, Clone)]
struct AcquisitionSatEvaluation {
    sat: SatId,
    candidates: Vec<AcqResult>,
    search_window_diagnostic: Option<SearchWindowDiagnostic>,
}

#[derive(Debug, Clone, Copy)]
struct JointAcquisitionRefinement {
    doppler_offset_bins: f64,
    code_phase_offset_samples: f64,
    refined_code_phase_samples: f64,
    doppler_cross_section: [f32; 3],
    code_phase_cross_section: [f32; 3],
}

#[derive(Debug, Clone, Copy)]
struct LocalAcquisitionLikelihoodSurface {
    doppler_cross_section: [f32; 3],
    code_phase_cross_section: [f32; 3],
    values: [[f32; 3]; 3],
}

#[derive(Debug, Clone, Copy)]
struct LocalAcquisitionLikelihoodVolume {
    values: [[[f32; 3]; 3]; 3],
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum AcquisitionDecisionReason {
    AcceptedByRatioThresholds,
    AmbiguousRatioThresholds,
    MultipathSuspect,
    LowPeakMetric,
}

impl AcquisitionDecisionReason {
    fn as_str(self) -> &'static str {
        match self {
            AcquisitionDecisionReason::AcceptedByRatioThresholds => "accepted_by_ratio_thresholds",
            AcquisitionDecisionReason::AmbiguousRatioThresholds => "ambiguous_ratio_thresholds",
            AcquisitionDecisionReason::MultipathSuspect => "multipath_suspect",
            AcquisitionDecisionReason::LowPeakMetric => "low_peak_metric",
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct AcquisitionDecision {
    hypothesis: AcqHypothesis,
    reason: AcquisitionDecisionReason,
    score: f32,
}

#[derive(Debug, Clone)]
struct ComponentCorrelationAccumulation {
    per_noncoherent: Vec<Vec<Complex<f32>>>,
    noncoherent_accumulator: Vec<f32>,
    secondary_code_phase_periods: Option<u32>,
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

    fn search_assumptions(
        &self,
        frame_samples: usize,
        request: AcqRequest,
        resolved_bounds: &ResolvedAcquisitionSearchBounds,
        samples_per_code: usize,
    ) -> AcqAssumptions {
        AcqAssumptions {
            doppler_center_hz: request.doppler_center_hz,
            doppler_rate_center_hz_per_s: request.doppler_rate_center_hz_per_s,
            expected_line_of_sight_doppler_hz: request.expected_line_of_sight_doppler_hz,
            assistance_bounds: request.assistance_bounds,
            doppler_search_hz: resolved_bounds.doppler_search_hz,
            doppler_step_hz: request.doppler_step_hz.max(1),
            doppler_rate_search_hz_per_s: request.doppler_rate_search_hz_per_s.max(0),
            doppler_rate_step_hz_per_s: request.doppler_rate_step_hz_per_s.max(1),
            coherent_ms: request.coherent_ms,
            noncoherent: request.noncoherent,
            samples_per_code,
            frame_samples,
            code_phase_search_start_sample: resolved_bounds.code_phase_search_start_sample,
            code_phase_search_step_samples: resolved_bounds.code_phase_search_step_samples,
            code_phase_search_bins: resolved_bounds.code_phase_search_bins,
            code_phase_search_mode: resolved_bounds.code_phase_search_mode.clone(),
        }
    }

    fn default_request(&self, sat: SatId, coherent_ms: u32, noncoherent: u32) -> AcqRequest {
        AcqRequest {
            sat,
            glonass_frequency_channel: None,
            signal_band: resolved_acquisition_signal_band(&self.config, sat),
            signal_code: default_signal_code_for_band(
                sat.constellation,
                resolved_acquisition_signal_band(&self.config, sat),
            ),
            doppler_center_hz: 0.0,
            doppler_rate_center_hz_per_s: 0.0,
            expected_line_of_sight_doppler_hz: None,
            assistance_bounds: None,
            doppler_search_hz: self.doppler_search_hz,
            doppler_step_hz: self.doppler_step_hz,
            doppler_rate_search_hz_per_s: self.config.acquisition_doppler_rate_search_hz_per_s,
            doppler_rate_step_hz_per_s: self.config.acquisition_doppler_rate_step_hz_per_s.max(1),
            coherent_ms,
            noncoherent,
        }
    }

    fn default_requests_for_sats(
        &self,
        sats: &[SatId],
        coherent_ms: u32,
        noncoherent: u32,
    ) -> Vec<AcqRequest> {
        sats.iter()
            .copied()
            .map(|sat| self.default_request(sat, coherent_ms, noncoherent))
            .collect()
    }

    fn resolve_thresholds_for_request(
        &self,
        request: AcqRequest,
        signal_model: &AcquisitionSignalModel,
    ) -> ResolvedAcquisitionThresholds {
        match self.config.acquisition_threshold_policy.mode {
            AcquisitionThresholdMode::FixedRatio => self.fixed_thresholds_for_request(request),
            AcquisitionThresholdMode::CalibratedFalseAlarm => {
                self.calibrated_thresholds_for_request(request, signal_model)
            }
        }
    }

    fn fixed_thresholds_for_request(&self, request: AcqRequest) -> ResolvedAcquisitionThresholds {
        let mut provenance = threshold_provenance_for_request(&self.config, request);
        provenance.mode = "fixed_ratio".to_string();
        provenance.peak_mean_threshold = self.config.acquisition_peak_mean_threshold;
        provenance.peak_second_threshold = self.config.acquisition_peak_second_threshold;
        provenance.false_alarm_probability = None;
        provenance.calibration_trial_count = None;
        provenance.calibration_confidence_level = None;
        provenance.calibration_false_alarm_rate = None;
        provenance.calibration_false_alarm_interval_low = None;
        provenance.calibration_false_alarm_interval_high = None;
        ResolvedAcquisitionThresholds {
            peak_mean_threshold: self.config.acquisition_peak_mean_threshold,
            peak_second_threshold: self.config.acquisition_peak_second_threshold,
            provenance,
        }
    }

    fn calibrated_thresholds_for_request(
        &self,
        request: AcqRequest,
        signal_model: &AcquisitionSignalModel,
    ) -> ResolvedAcquisitionThresholds {
        let signal_code = resolved_request_signal_code(request);
        let cache_key = AcquisitionThresholdCacheKey::from_request(
            request,
            signal_model.signal_band,
            signal_code,
            signal_model.samples_per_code(self.config.sampling_freq_hz),
        );
        if let Some(cached) =
            self.threshold_cache.lock().ok().and_then(|cache| cache.get(&cache_key).cloned())
        {
            return cached;
        }
        let resolved = self.calibrate_thresholds_for_request(request, signal_model);
        if let Ok(mut cache) = self.threshold_cache.lock() {
            cache.insert(cache_key, resolved.clone());
        }
        resolved
    }

    fn calibrate_thresholds_for_request(
        &self,
        request: AcqRequest,
        signal_model: &AcquisitionSignalModel,
    ) -> ResolvedAcquisitionThresholds {
        let policy = &self.config.acquisition_threshold_policy;
        let target_false_alarm_probability = policy.false_alarm_probability;
        let trial_count = policy.calibration_trial_count;
        let confidence_level = policy.confidence_level;
        let mut lower = 1.0_f32;
        let mut upper = self.config.acquisition_peak_mean_threshold.max(lower + 1.0);
        let mut upper_measurement = self.measure_noise_only_false_alarm_rate(
            request,
            signal_model,
            upper,
            trial_count,
            confidence_level,
        );
        while upper_measurement.false_alarm_rate > target_false_alarm_probability && upper < 64.0 {
            lower = upper;
            upper = (upper * 2.0).min(64.0);
            upper_measurement = self.measure_noise_only_false_alarm_rate(
                request,
                signal_model,
                upper,
                trial_count,
                confidence_level,
            );
        }

        for _ in 0..FALSE_ALARM_CALIBRATION_SEARCH_ITERATIONS {
            let midpoint = (lower + upper) * 0.5;
            let midpoint_measurement = self.measure_noise_only_false_alarm_rate(
                request,
                signal_model,
                midpoint,
                trial_count,
                confidence_level,
            );
            if midpoint_measurement.false_alarm_rate <= target_false_alarm_probability {
                upper = midpoint;
                upper_measurement = midpoint_measurement;
            } else {
                lower = midpoint;
            }
        }

        self.runtime.trace.record(TraceRecord {
            name: "acquisition_threshold_calibration",
            fields: vec![
                ("constellation", format!("{:?}", request.sat.constellation)),
                ("prn", request.sat.prn.to_string()),
                ("signal_band", format!("{:?}", signal_model.signal_band)),
                ("signal_code", format!("{:?}", resolved_request_signal_code(request))),
                ("trial_count", trial_count.to_string()),
                ("target_false_alarm_probability", format!("{target_false_alarm_probability:.9}")),
                ("peak_mean_threshold", format!("{upper:.6}")),
                ("measured_false_alarm_rate", format!("{:.9}", upper_measurement.false_alarm_rate)),
                (
                    "confidence_interval_low",
                    format!("{:.9}", upper_measurement.confidence_interval_low),
                ),
                (
                    "confidence_interval_high",
                    format!("{:.9}", upper_measurement.confidence_interval_high),
                ),
            ],
        });

        let mut provenance = threshold_provenance_for_request(&self.config, request);
        provenance.mode = "calibrated_false_alarm".to_string();
        provenance.peak_mean_threshold = upper;
        provenance.peak_second_threshold = self.config.acquisition_peak_second_threshold;
        provenance.false_alarm_probability = Some(target_false_alarm_probability);
        provenance.calibration_trial_count = Some(trial_count);
        provenance.calibration_confidence_level = Some(confidence_level);
        provenance.calibration_false_alarm_rate = Some(upper_measurement.false_alarm_rate);
        provenance.calibration_false_alarm_interval_low =
            Some(upper_measurement.confidence_interval_low);
        provenance.calibration_false_alarm_interval_high =
            Some(upper_measurement.confidence_interval_high);

        ResolvedAcquisitionThresholds {
            peak_mean_threshold: upper,
            peak_second_threshold: self.config.acquisition_peak_second_threshold,
            provenance,
        }
    }

    fn measure_noise_only_false_alarm_rate(
        &self,
        request: AcqRequest,
        signal_model: &AcquisitionSignalModel,
        peak_mean_threshold: f32,
        trial_count: usize,
        confidence_level: f64,
    ) -> FalseAlarmRateMeasurement {
        let mut calibration_config = self.config.clone();
        calibration_config.acquisition_peak_mean_threshold = peak_mean_threshold;
        calibration_config.acquisition_threshold_policy.mode = AcquisitionThresholdMode::FixedRatio;
        let calibration_engine = Self::new(calibration_config, ReceiverRuntime::default());
        let required_samples = required_samples_for_request(
            &self.config,
            signal_model,
            request.coherent_ms,
            request.noncoherent,
        );
        let calibration_seed = calibration_seed(request, signal_model, peak_mean_threshold);
        let false_alarm_count = (0..trial_count)
            .filter(|trial_index| {
                let frame = noise_only_frame(
                    self.config.sampling_freq_hz,
                    required_samples,
                    mix_seed(calibration_seed, *trial_index as u64),
                );
                calibration_engine
                    .run_fft_for_requests(&frame, &[request])
                    .into_iter()
                    .any(|result| matches!(result.hypothesis, AcqHypothesis::Accepted))
            })
            .count();
        let (confidence_interval_low, confidence_interval_high) =
            wilson_confidence_interval(false_alarm_count, trial_count, confidence_level);
        FalseAlarmRateMeasurement {
            false_alarm_rate: false_alarm_rate(false_alarm_count, trial_count),
            confidence_interval_low,
            confidence_interval_high,
        }
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
            if !acquisition_integration_ms_is_supported(request.coherent_ms) {
                sat_evaluations.push(AcquisitionSatEvaluation {
                    sat,
                    candidates: unsupported_coherent_integration_candidates(
                        sat,
                        &signal_model,
                        signal_code,
                        request.glonass_frequency_channel,
                        &assumptions,
                        &requested_threshold_provenance,
                        search_center_hz,
                        ReceiverSampleTrace::from_sample_time(frame.t0),
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
                            sat,
                            &signal_model,
                            signal_code,
                            request.glonass_frequency_channel,
                            &assumptions,
                            &requested_threshold_provenance,
                            search_center_hz,
                            ReceiverSampleTrace::from_sample_time(frame.t0),
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
                        sat,
                        &signal_model,
                        signal_code,
                        request.glonass_frequency_channel,
                        &assumptions,
                        &requested_threshold_provenance,
                        search_center_hz,
                        ReceiverSampleTrace::from_sample_time(frame.t0),
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
                    self.code_fft(
                        &signal_model,
                        component,
                        sat,
                        signal_code,
                        samples_per_code,
                        request.coherent_ms,
                        request.noncoherent,
                        fft.as_ref(),
                    )
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
                        let accumulation = accumulate_component_correlations(
                            frame,
                            component,
                            code_fft,
                            carrier,
                            absolute_doppler_rate_hz_per_s,
                            self.config.sampling_freq_hz,
                            samples_per_code,
                            coherent_periods,
                            request.noncoherent,
                            fft.as_ref(),
                            ifft.as_ref(),
                        );
                        component_accumulations.push(accumulation);
                    }

                    for (strategy, component_indexes) in
                        strategies.iter().zip(strategy_component_indexes.iter())
                    {
                        let combined_accumulator = match strategy.combination_mode {
                            AcqComponentCombinationMode::SingleComponent => component_accumulations
                                [component_indexes[0]]
                                .noncoherent_accumulator
                                .clone(),
                            AcqComponentCombinationMode::NoncoherentComponentSum => {
                                let mut combined = vec![0.0f32; samples_per_code];
                                for &component_index in component_indexes {
                                    for (combined_value, component_value) in
                                        combined.iter_mut().zip(
                                            component_accumulations[component_index]
                                                .noncoherent_accumulator
                                                .iter(),
                                        )
                                    {
                                        *combined_value += *component_value;
                                    }
                                }
                                combined
                            }
                            AcqComponentCombinationMode::CoherentComponentSum => {
                                let mut combined = vec![0.0f32; samples_per_code];
                                for nc in 0..request.noncoherent as usize {
                                    for sample_index in 0..samples_per_code {
                                        let mut coherent_sum: Complex<f32> = Complex::zero();
                                        for &component_index in component_indexes {
                                            coherent_sum += component_accumulations
                                                [component_index]
                                                .per_noncoherent[nc][sample_index];
                                        }
                                        combined[sample_index] += coherent_sum.norm();
                                    }
                                }
                                combined
                            }
                        };

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
                    self,
                    frame,
                    &signal_model,
                    sat,
                    &mut candidates,
                    &grid_candidates,
                    request.doppler_step_hz.max(1),
                    request.coherent_ms,
                    request.noncoherent,
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
                                &self.config,
                                &signal_model,
                                frame,
                                sat,
                                candidate.carrier_hz.0,
                                candidate.doppler_rate_hz_per_s,
                                request.coherent_ms,
                                request.noncoherent,
                                &resolved_bounds,
                                resolved_thresholds.peak_mean_threshold,
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
                            classify_delayed_secondary_peak(
                                &self.config,
                                frame,
                                &signal_model,
                                sat,
                                candidate.carrier_hz.0,
                                candidate.doppler_rate_hz_per_s,
                                candidate.code_phase_samples,
                                samples_per_code,
                                request.coherent_ms,
                                request.noncoherent,
                                candidate.peak_mean_ratio,
                                candidate.peak_second_ratio,
                                competing_peak_ratio,
                                &resolved_thresholds,
                            )
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
                        candidate.uncertainty = estimate_acquisition_uncertainty(
                            &self.config,
                            frame,
                            &signal_model,
                            candidate,
                            request.coherent_ms,
                            request.noncoherent,
                            request.doppler_step_hz.max(1),
                            request.doppler_rate_search_hz_per_s.max(0),
                            request.doppler_rate_step_hz_per_s.max(1),
                        );
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

        let mut results = Vec::with_capacity(sat_evaluations.len());
        let mut explains = Vec::with_capacity(sat_evaluations.len());
        for evaluation in sat_evaluations {
            let sat = evaluation.sat;
            let search_window_diagnostic = evaluation.search_window_diagnostic;
            let candidates = evaluation.candidates;
            if candidates.is_empty() {
                self.runtime.trace.record(TraceRecord {
                    name: "acquisition_sat_done",
                    fields: vec![
                        ("constellation", format!("{:?}", sat.constellation)),
                        ("prn", sat.prn.to_string()),
                        ("outcome", "no_candidates".to_string()),
                    ],
                });
                self.with_stats(|stats| {
                    stats.deferred_count = stats.deferred_count.saturating_add(1);
                });
                if emit_explanations {
                    explains.push(AcqExplain {
                        sat,
                        selected_rank: None,
                        selected_reason: "no_candidates".to_string(),
                        candidate_count: 0,
                        candidates: Vec::new(),
                    });
                }
                results.push(candidates);
                continue;
            }

            let best = &candidates[0];
            if emit_explanations {
                let selected_reason = match search_window_diagnostic.as_ref() {
                    Some(_) => "signal_outside_search_range",
                    None => selected_reason_for_candidate(best),
                };
                explains.push(AcqExplain {
                    sat,
                    selected_rank: Some(best.candidate_rank),
                    selected_reason: selected_reason.to_string(),
                    candidate_count: candidates.len(),
                    candidates: candidates
                        .iter()
                        .map(|candidate| AcqExplainCandidate {
                            rank: candidate.candidate_rank,
                            code_phase_samples: candidate.code_phase_samples,
                            carrier_hz: candidate.carrier_hz.0,
                            peak: candidate.peak,
                            peak_mean_ratio: candidate.peak_mean_ratio,
                            peak_second_ratio: candidate.peak_second_ratio,
                            second_peak_ratio: (if candidate.peak == 0.0 {
                                f32::INFINITY
                            } else {
                                candidate.peak / (candidate.second_peak + 1e-6)
                            }),
                            mean: candidate.mean,
                            hypothesis: candidate.hypothesis,
                            score: candidate.score,
                            threshold_hit: matches!(candidate.hypothesis, AcqHypothesis::Accepted),
                            reason: candidate
                                .explain_selection_reason
                                .clone()
                                .unwrap_or_else(|| "discarded".to_string()),
                        })
                        .collect(),
                });
            }
            let outcome = match best.hypothesis {
                AcqHypothesis::Accepted => "accepted",
                AcqHypothesis::Ambiguous => "ambiguous",
                AcqHypothesis::Rejected => "rejected",
                AcqHypothesis::Deferred => "deferred",
            };
            self.runtime.trace.record(TraceRecord {
                name: "acquisition_sat_done",
                fields: vec![
                    ("constellation", format!("{:?}", best.sat.constellation)),
                    ("prn", best.sat.prn.to_string()),
                    ("outcome", outcome.to_string()),
                    (
                        "reason",
                        search_window_diagnostic.as_ref().map_or_else(
                            || selected_reason_for_candidate(best).to_string(),
                            |_| "signal_outside_search_range".to_string(),
                        ),
                    ),
                    ("carrier_hz", format!("{:.3}", best.carrier_hz.0)),
                    ("score", format!("{:.6}", best.score)),
                    ("peak_mean_ratio", format!("{:.6}", best.peak_mean_ratio)),
                ],
            });
            if let Some(diagnostic) = search_window_diagnostic.as_ref() {
                self.runtime.trace.record(TraceRecord {
                    name: "acquisition_search_window_rejection",
                    fields: vec![
                        ("constellation", format!("{:?}", best.sat.constellation)),
                        ("prn", best.sat.prn.to_string()),
                        ("reason", "signal_outside_search_range".to_string()),
                        (
                            "edge",
                            match diagnostic.edge {
                                SearchWindowEdge::Lower => "lower".to_string(),
                                SearchWindowEdge::Upper => "upper".to_string(),
                            },
                        ),
                        (
                            "dimension",
                            match diagnostic.dimension {
                                SearchWindowDimension::Doppler => "doppler".to_string(),
                                SearchWindowDimension::DopplerRate => "doppler_rate".to_string(),
                                SearchWindowDimension::CodePhase => "code_phase".to_string(),
                            },
                        ),
                        ("best_axis_value", format!("{:.3}", diagnostic.best_axis_value)),
                        ("interior_axis_value", format!("{:.3}", diagnostic.interior_axis_value)),
                        ("best_peak_mean_ratio", format!("{:.6}", diagnostic.best_peak_mean_ratio)),
                        (
                            "interior_peak_mean_ratio",
                            format!("{:.6}", diagnostic.interior_peak_mean_ratio),
                        ),
                    ],
                });
            }
            if !best.evidence.is_empty() {
                self.runtime.trace.record(TraceRecord {
                    name: "acquisition_evidence",
                    fields: vec![
                        ("constellation", format!("{:?}", best.sat.constellation)),
                        ("prn", best.sat.prn.to_string()),
                        ("rank", format!("{}", best.evidence.first().map(|e| e.rank).unwrap_or(0))),
                        ("peak", format!("{:.6}", best.peak)),
                        ("second_peak", format!("{:.6}", best.second_peak)),
                        ("mean", format!("{:.6}", best.mean)),
                    ],
                });
            }
            self.with_stats(|stats| match best.hypothesis {
                AcqHypothesis::Accepted => {
                    stats.accepted_count = stats.accepted_count.saturating_add(1)
                }
                AcqHypothesis::Ambiguous => {
                    stats.ambiguous_count = stats.ambiguous_count.saturating_add(1)
                }
                AcqHypothesis::Rejected => {
                    stats.rejected_count = stats.rejected_count.saturating_add(1)
                }
                AcqHypothesis::Deferred => {
                    stats.deferred_count = stats.deferred_count.saturating_add(1)
                }
            });
            let stable_keys = stable_acq_result_keys(&candidates);
            self.runtime.trace.record(TraceRecord {
                name: "acquisition_stability_signature",
                fields: vec![
                    ("constellation", format!("{:?}", sat.constellation)),
                    ("prn", sat.prn.to_string()),
                    ("candidate_count", stable_keys.len().to_string()),
                    (
                        "top_signature",
                        stable_keys.first().cloned().unwrap_or_else(|| "none".to_string()),
                    ),
                ],
            });
            results.push(candidates);
        }
        AcquisitionRun { results, explains }
    }

    fn code_fft(
        &self,
        signal_model: &AcquisitionSignalModel,
        component: &AcquisitionComponentPlan,
        sat: SatId,
        signal_code: SignalCode,
        samples_per_code: usize,
        _coherent_ms: u32,
        _noncoherent: u32,
        fft: &dyn rustfft::Fft<f32>,
    ) -> Vec<Complex<f32>> {
        let key = CodeFftCacheKey::from_runtime(
            &self.config,
            &signal_model,
            component.role,
            sat,
            signal_code,
            samples_per_code,
            self.doppler_search_hz,
            self.doppler_step_hz,
        );
        if let Some(cached) = self.cache.lock().ok().and_then(|m| m.get(&key).cloned()) {
            self.with_stats(|stats| {
                stats.cache_hits = stats.cache_hits.saturating_add(1);
            });
            self.runtime.trace.record(TraceRecord {
                name: "acquisition_code_fft_cache_hit",
                fields: vec![
                    ("constellation", format!("{:?}", sat.constellation)),
                    ("prn", sat.prn.to_string()),
                    ("signal_band", format!("{:?}", signal_model.signal_band)),
                    ("signal_code", format!("{:?}", signal_code)),
                    ("component_role", format!("{:?}", component.role)),
                    ("samples_per_code", samples_per_code.to_string()),
                ],
            });
            return cached;
        }

        let miss_reason = self.cache.lock().ok().map_or(CacheMissReason::ColdStart, |cache| {
            let has_same_satellite =
                cache.keys().any(|cached| cached.matches_signal_period(sat, samples_per_code));
            if has_same_satellite {
                CacheMissReason::IncompatibleAssumptions
            } else {
                CacheMissReason::ColdStart
            }
        });

        self.with_stats(|stats| {
            stats.cache_misses = stats.cache_misses.saturating_add(1);
            match miss_reason {
                CacheMissReason::ColdStart => {
                    stats.cache_miss_cold_start = stats.cache_miss_cold_start.saturating_add(1)
                }
                CacheMissReason::IncompatibleAssumptions => {
                    stats.cache_miss_incompatible = stats.cache_miss_incompatible.saturating_add(1)
                }
            }
        });
        self.runtime.trace.record(TraceRecord {
            name: "acquisition_code_fft_cache_miss",
            fields: vec![
                ("constellation", format!("{:?}", sat.constellation)),
                ("prn", sat.prn.to_string()),
                ("signal_band", format!("{:?}", signal_model.signal_band)),
                ("signal_code", format!("{:?}", signal_code)),
                ("component_role", format!("{:?}", component.role)),
                ("samples_per_code", samples_per_code.to_string()),
                ("reason", miss_reason.as_str().to_string()),
                ("model_version", ACQUISITION_CACHE_MODEL_VERSION.to_string()),
                ("policy_version", ACQUISITION_CACHE_POLICY_VERSION.to_string()),
            ],
        });
        let local_code = component
            .sample_local_code_period(self.config.sampling_freq_hz, samples_per_code)
            .unwrap_or_else(|_| vec![1.0; samples_per_code]);
        let mut code_fft: Vec<Complex<f32>> =
            local_code.iter().map(|&x| Complex::new(x, 0.0)).collect();
        fft.process(&mut code_fft);
        if let Ok(mut cache) = self.cache.lock() {
            cache.insert(key, code_fft.clone());
        }
        code_fft
    }

    fn estimate_joint_acquisition_refinement(
        &self,
        frame: &SamplesFrame,
        signal_model: &AcquisitionSignalModel,
        sat: SatId,
        coarse_carrier_hz: f64,
        coarse_doppler_rate_hz_per_s: f64,
        coarse_code_phase_samples: usize,
        doppler_step_hz: i32,
        coherent_ms: u32,
        noncoherent: u32,
    ) -> Option<JointAcquisitionRefinement> {
        if doppler_step_hz <= 0 {
            return None;
        }
        let samples_per_code = signal_model.samples_per_code(self.config.sampling_freq_hz);
        let coherent_periods = signal_model.coherent_periods(coherent_ms)?;
        if samples_per_code == 0
            || frame.len()
                < samples_per_code * coherent_periods.max(1) as usize * noncoherent.max(1) as usize
        {
            return None;
        }

        let surface = measure_local_acquisition_likelihood_surface(
            &self.config,
            signal_model,
            frame,
            sat,
            coarse_carrier_hz,
            coarse_doppler_rate_hz_per_s,
            coarse_code_phase_samples,
            doppler_step_hz as f64,
            coherent_ms,
            noncoherent,
        )?;
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
                coarse_code_phase_samples as f64 + code_phase_offset_samples,
                samples_per_code,
            ),
            doppler_cross_section: surface.doppler_cross_section,
            code_phase_cross_section: surface.code_phase_cross_section,
        })
    }

    fn estimate_acquisition_code_phase_refinement(
        &self,
        frame: &SamplesFrame,
        signal_model: &AcquisitionSignalModel,
        sat: SatId,
        carrier_hz: f64,
        doppler_rate_hz_per_s: f64,
        coarse_code_phase_samples: usize,
        coherent_ms: u32,
        noncoherent: u32,
    ) -> Option<AcqCodePhaseRefinement> {
        let samples_per_code = signal_model.samples_per_code(self.config.sampling_freq_hz);
        let coherent_periods = signal_model.coherent_periods(coherent_ms)?;
        if samples_per_code == 0
            || frame.len()
                < samples_per_code * coherent_periods.max(1) as usize * noncoherent.max(1) as usize
        {
            return None;
        }
        let correlation_profile = measure_code_phase_profile(
            &self.config,
            signal_model,
            frame,
            sat,
            carrier_hz,
            doppler_rate_hz_per_s,
            coherent_ms,
            noncoherent,
        )?;
        let (
            offset_samples,
            left_correlation_norm,
            center_correlation_norm,
            right_correlation_norm,
        ) = estimate_parabolic_code_phase_offset_samples(
            &correlation_profile,
            coarse_code_phase_samples,
        )?;
        let refined_code_phase_samples = wrap_acquisition_code_phase_samples(
            coarse_code_phase_samples as f64 + offset_samples,
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

fn required_samples_for_request(
    config: &ReceiverPipelineConfig,
    signal_model: &AcquisitionSignalModel,
    coherent_ms: u32,
    noncoherent: u32,
) -> usize {
    let samples_per_code = signal_model.samples_per_code(config.sampling_freq_hz);
    let coherent_periods = signal_model.coherent_periods(coherent_ms).unwrap_or(1).max(1) as usize;
    samples_per_code.saturating_mul(coherent_periods).saturating_mul(noncoherent.max(1) as usize)
}

fn doppler_bin_count(search_hz: i32, step_hz: i32) -> u64 {
    if step_hz <= 0 {
        return 0;
    }
    let search = search_hz.unsigned_abs() as u64;
    let step = step_hz.unsigned_abs() as u64;
    (search / step).saturating_mul(2).saturating_add(1)
}

fn zero_signal_run(
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

fn insufficient_frame_candidates(
    sat: SatId,
    signal_model: &AcquisitionSignalModel,
    signal_code: SignalCode,
    glonass_frequency_channel: Option<bijux_gnss_core::api::GlonassFrequencyChannel>,
    assumptions: &AcqAssumptions,
    threshold_provenance: &AcqThresholdProvenance,
    intermediate_freq_hz: f64,
    source_time: ReceiverSampleTrace,
    available_samples: usize,
    required_samples: usize,
) -> Vec<AcqResult> {
    let candidate_reason = insufficient_frame_candidate_reason(available_samples, required_samples);
    vec![AcqResult {
        sat,
        signal_band: signal_model.signal_band,
        signal_code,
        glonass_frequency_channel,
        source_time,
        candidate_rank: 1,
        is_primary_candidate: true,
        doppler_hz: Hertz(0.0),
        doppler_rate_hz_per_s: assumptions.doppler_rate_center_hz_per_s,
        carrier_hz: Hertz(intermediate_freq_hz),
        code_phase_samples: 0,
        peak: 0.0,
        second_peak: 0.0,
        mean: 0.0,
        peak_mean_ratio: 0.0,
        peak_second_ratio: 0.0,
        cn0_proxy: 0.0,
        score: 0.0,
        hypothesis: AcqHypothesis::Deferred,
        assumptions: Some(assumptions.clone()),
        evidence: Vec::new(),
        threshold_provenance: Some(threshold_provenance.clone()),
        explain_selection_reason: Some(candidate_reason),
        doppler_refinement: None,
        code_phase_refinement: None,
        signal_delay_alignment: None,
        uncertainty: None,
    }]
}

fn acquisition_request_error_candidates(
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

fn zero_signal_candidate(
    sat: SatId,
    signal_model: &AcquisitionSignalModel,
    signal_code: SignalCode,
    glonass_frequency_channel: Option<bijux_gnss_core::api::GlonassFrequencyChannel>,
    assumptions: &AcqAssumptions,
    threshold_provenance: &AcqThresholdProvenance,
    intermediate_freq_hz: f64,
    source_time: ReceiverSampleTrace,
    zero_signal_reason: Option<&str>,
) -> AcqResult {
    AcqResult {
        sat,
        signal_band: signal_model.signal_band,
        signal_code,
        glonass_frequency_channel,
        source_time,
        candidate_rank: 1,
        is_primary_candidate: true,
        doppler_hz: Hertz(assumptions.doppler_center_hz),
        doppler_rate_hz_per_s: assumptions.doppler_rate_center_hz_per_s,
        carrier_hz: Hertz(intermediate_freq_hz),
        code_phase_samples: 0,
        peak: 0.0,
        second_peak: 0.0,
        mean: 0.0,
        peak_mean_ratio: 0.0,
        peak_second_ratio: 0.0,
        cn0_proxy: 0.0,
        score: 0.0,
        hypothesis: AcqHypothesis::Rejected,
        assumptions: Some(assumptions.clone()),
        evidence: Vec::new(),
        threshold_provenance: Some(threshold_provenance.clone()),
        explain_selection_reason: Some(zero_signal_candidate_reason(zero_signal_reason)),
        doppler_refinement: None,
        code_phase_refinement: None,
        signal_delay_alignment: None,
        uncertainty: None,
    }
}

fn unsupported_coherent_integration_candidates(
    sat: SatId,
    signal_model: &AcquisitionSignalModel,
    signal_code: SignalCode,
    glonass_frequency_channel: Option<bijux_gnss_core::api::GlonassFrequencyChannel>,
    assumptions: &AcqAssumptions,
    threshold_provenance: &AcqThresholdProvenance,
    intermediate_freq_hz: f64,
    source_time: ReceiverSampleTrace,
    coherent_ms: u32,
) -> Vec<AcqResult> {
    vec![unsupported_coherent_integration_candidate(
        sat,
        signal_model,
        signal_code,
        glonass_frequency_channel,
        assumptions,
        threshold_provenance,
        intermediate_freq_hz,
        source_time,
        coherent_ms,
    )]
}

fn unsupported_coherent_integration_candidate(
    sat: SatId,
    signal_model: &AcquisitionSignalModel,
    signal_code: SignalCode,
    glonass_frequency_channel: Option<bijux_gnss_core::api::GlonassFrequencyChannel>,
    assumptions: &AcqAssumptions,
    threshold_provenance: &AcqThresholdProvenance,
    intermediate_freq_hz: f64,
    source_time: ReceiverSampleTrace,
    coherent_ms: u32,
) -> AcqResult {
    AcqResult {
        sat,
        signal_band: signal_model.signal_band,
        signal_code,
        glonass_frequency_channel,
        source_time,
        candidate_rank: 1,
        is_primary_candidate: true,
        doppler_hz: Hertz(assumptions.doppler_center_hz),
        doppler_rate_hz_per_s: assumptions.doppler_rate_center_hz_per_s,
        carrier_hz: Hertz(intermediate_freq_hz),
        code_phase_samples: 0,
        peak: 0.0,
        second_peak: 0.0,
        mean: 0.0,
        peak_mean_ratio: 0.0,
        peak_second_ratio: 0.0,
        cn0_proxy: 0.0,
        score: 0.0,
        hypothesis: AcqHypothesis::Deferred,
        assumptions: Some(assumptions.clone()),
        evidence: Vec::new(),
        threshold_provenance: Some(threshold_provenance.clone()),
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

fn signal_outside_search_range(
    candidates: &[AcqResult],
    intermediate_freq_hz: f64,
    doppler_search_hz: i32,
    doppler_step_hz: i32,
    peak_mean_threshold: f32,
) -> Option<SearchWindowDiagnostic> {
    if candidates.len() < 2 || doppler_search_hz <= 0 || doppler_step_hz <= 0 {
        return None;
    }

    let best_peak_mean_ratio =
        candidates.iter().map(|candidate| candidate.peak_mean_ratio).fold(0.0_f32, f32::max);
    let hint_threshold =
        peak_mean_threshold.max(best_peak_mean_ratio * SEARCH_EDGE_HINT_PEAK_MEAN_RATIO_FRACTION);
    [
        (
            SearchWindowEdge::Lower,
            carrier_hz_from_doppler_hz(intermediate_freq_hz, -(doppler_search_hz as f64)),
            carrier_hz_from_doppler_hz(
                intermediate_freq_hz,
                -((doppler_search_hz - doppler_step_hz) as f64),
            ),
        ),
        (
            SearchWindowEdge::Upper,
            carrier_hz_from_doppler_hz(intermediate_freq_hz, doppler_search_hz as f64),
            carrier_hz_from_doppler_hz(
                intermediate_freq_hz,
                (doppler_search_hz - doppler_step_hz) as f64,
            ),
        ),
    ]
    .into_iter()
    .filter_map(|(edge, edge_carrier_hz, interior_carrier_hz)| {
        let edge_candidate = find_candidate_by_carrier_hz(candidates, edge_carrier_hz)?;
        let interior_candidate = find_candidate_by_carrier_hz(candidates, interior_carrier_hz)?;
        if edge_candidate.peak_mean_ratio < hint_threshold {
            return None;
        }
        if edge_candidate.peak_mean_ratio
            <= interior_candidate.peak_mean_ratio * (1.0 + SEARCH_EDGE_RISE_RATIO_EPSILON)
        {
            return None;
        }
        Some(SearchWindowDiagnostic {
            dimension: SearchWindowDimension::Doppler,
            edge,
            best_axis_value: edge_candidate.carrier_hz.0,
            interior_axis_value: interior_candidate.carrier_hz.0,
            best_peak_mean_ratio: edge_candidate.peak_mean_ratio,
            interior_peak_mean_ratio: interior_candidate.peak_mean_ratio,
        })
    })
    .max_by(|left, right| {
        left.best_peak_mean_ratio
            .partial_cmp(&right.best_peak_mean_ratio)
            .unwrap_or(std::cmp::Ordering::Equal)
    })
}

fn assisted_code_phase_search_window_diagnostic(
    config: &ReceiverPipelineConfig,
    signal_model: &AcquisitionSignalModel,
    frame: &SamplesFrame,
    sat: SatId,
    carrier_hz: f64,
    doppler_rate_hz_per_s: f64,
    coherent_ms: u32,
    noncoherent: u32,
    resolved_bounds: &ResolvedAcquisitionSearchBounds,
    peak_mean_threshold: f32,
) -> Option<SearchWindowDiagnostic> {
    if resolved_bounds.code_phase_search_bins == 0
        || resolved_bounds.code_phase_search_mode == "full_code"
    {
        return None;
    }
    let correlation_profile = measure_code_phase_profile(
        config,
        signal_model,
        frame,
        sat,
        carrier_hz,
        doppler_rate_hz_per_s,
        coherent_ms,
        noncoherent,
    )?;
    code_phase_outside_search_range(
        &correlation_profile,
        resolved_bounds.code_phase_search_start_sample,
        resolved_bounds.code_phase_search_step_samples,
        resolved_bounds.code_phase_search_bins,
        peak_mean_threshold,
    )
}

fn signal_outside_doppler_rate_search_range(
    candidates: &[AcqResult],
    carrier_hz: f64,
    doppler_rate_center_hz_per_s: f64,
    doppler_rate_search_hz_per_s: i32,
    doppler_rate_step_hz_per_s: i32,
    peak_mean_threshold: f32,
) -> Option<SearchWindowDiagnostic> {
    if candidates.len() < 2 || doppler_rate_search_hz_per_s <= 0 || doppler_rate_step_hz_per_s <= 0
    {
        return None;
    }

    let best_peak_mean_ratio = candidates
        .iter()
        .filter(|candidate| (candidate.carrier_hz.0 - carrier_hz).abs() <= f64::EPSILON)
        .map(|candidate| candidate.peak_mean_ratio)
        .fold(0.0_f32, f32::max);
    if best_peak_mean_ratio <= f32::EPSILON {
        return None;
    }
    let hint_threshold =
        peak_mean_threshold.max(best_peak_mean_ratio * SEARCH_EDGE_HINT_PEAK_MEAN_RATIO_FRACTION);
    [
        (
            SearchWindowEdge::Lower,
            doppler_rate_center_hz_per_s - doppler_rate_search_hz_per_s as f64,
            doppler_rate_center_hz_per_s
                - (doppler_rate_search_hz_per_s - doppler_rate_step_hz_per_s) as f64,
        ),
        (
            SearchWindowEdge::Upper,
            doppler_rate_center_hz_per_s + doppler_rate_search_hz_per_s as f64,
            doppler_rate_center_hz_per_s
                + (doppler_rate_search_hz_per_s - doppler_rate_step_hz_per_s) as f64,
        ),
    ]
    .into_iter()
    .filter_map(|(edge, edge_rate_hz_per_s, interior_rate_hz_per_s)| {
        let edge_candidate =
            find_best_candidate_by_carrier_hz_and_rate(candidates, carrier_hz, edge_rate_hz_per_s)?;
        let interior_candidate = find_best_candidate_by_carrier_hz_and_rate(
            candidates,
            carrier_hz,
            interior_rate_hz_per_s,
        )?;
        if edge_candidate.peak_mean_ratio < hint_threshold {
            return None;
        }
        if edge_candidate.peak_mean_ratio
            <= interior_candidate.peak_mean_ratio * (1.0 + SEARCH_EDGE_RISE_RATIO_EPSILON)
        {
            return None;
        }
        Some(SearchWindowDiagnostic {
            dimension: SearchWindowDimension::DopplerRate,
            edge,
            best_axis_value: edge_candidate.doppler_rate_hz_per_s,
            interior_axis_value: interior_candidate.doppler_rate_hz_per_s,
            best_peak_mean_ratio: edge_candidate.peak_mean_ratio,
            interior_peak_mean_ratio: interior_candidate.peak_mean_ratio,
        })
    })
    .max_by(|left, right| {
        left.best_peak_mean_ratio
            .partial_cmp(&right.best_peak_mean_ratio)
            .unwrap_or(std::cmp::Ordering::Equal)
    })
}

fn acquisition_decision(
    peak_mean_ratio: f32,
    peak_second_ratio: f32,
    local_peak_separation_ratio: f32,
    competing_peak_ratio: f32,
    thresholds: &ResolvedAcquisitionThresholds,
) -> AcquisitionDecision {
    if peak_mean_ratio < thresholds.peak_mean_threshold {
        return AcquisitionDecision {
            hypothesis: AcqHypothesis::Rejected,
            reason: AcquisitionDecisionReason::LowPeakMetric,
            score: 0.0,
        };
    }
    if local_peak_separation_ratio.is_nan() || competing_peak_ratio.is_nan() {
        return AcquisitionDecision {
            hypothesis: AcqHypothesis::Ambiguous,
            reason: AcquisitionDecisionReason::AmbiguousRatioThresholds,
            score: 0.25 * peak_mean_ratio,
        };
    }
    let limiting_ratio =
        peak_second_ratio.min(local_peak_separation_ratio).min(competing_peak_ratio);
    if peak_second_ratio < thresholds.peak_second_threshold
        || local_peak_separation_ratio < thresholds.peak_second_threshold
        || competing_peak_ratio < thresholds.peak_second_threshold
    {
        return AcquisitionDecision {
            hypothesis: AcqHypothesis::Ambiguous,
            reason: AcquisitionDecisionReason::AmbiguousRatioThresholds,
            score: (peak_mean_ratio * 0.35) + (limiting_ratio * 0.15),
        };
    }
    AcquisitionDecision {
        hypothesis: AcqHypothesis::Accepted,
        reason: AcquisitionDecisionReason::AcceptedByRatioThresholds,
        score: (peak_mean_ratio * 0.5) + (peak_second_ratio * 0.5),
    }
}

fn selected_reason_for_candidate(candidate: &AcqResult) -> &'static str {
    if let Some(reason) = candidate.explain_selection_reason.as_deref() {
        if let Some(reason_prefix) = known_selection_reason_prefix(reason) {
            return reason_prefix;
        }
    }
    match candidate.hypothesis {
        AcqHypothesis::Accepted => AcquisitionDecisionReason::AcceptedByRatioThresholds.as_str(),
        AcqHypothesis::Ambiguous => AcquisitionDecisionReason::AmbiguousRatioThresholds.as_str(),
        AcqHypothesis::Rejected => AcquisitionDecisionReason::LowPeakMetric.as_str(),
        AcqHypothesis::Deferred => "deferred",
    }
}

fn known_selection_reason_prefix(reason: &str) -> Option<&'static str> {
    let reason_prefix = reason.split_once(':').map_or(reason, |(prefix, _)| prefix);
    match reason_prefix {
        "accepted_by_ratio_thresholds" => Some("accepted_by_ratio_thresholds"),
        "ambiguous_ratio_thresholds" => Some("ambiguous_ratio_thresholds"),
        "multipath_suspect" => Some("multipath_suspect"),
        "low_peak_metric" => Some("low_peak_metric"),
        "ranked_alternative" => Some("ranked_alternative"),
        "wrong_prn_correlation" => Some("wrong_prn_correlation"),
        "missing_glonass_frequency_channel" => Some("missing_glonass_frequency_channel"),
        "insufficient_frame" => Some("insufficient_frame"),
        "unsupported_coherent_integration_ms" => Some("unsupported_coherent_integration_ms"),
        _ => None,
    }
}

fn ranked_alternative_candidate_reason(primary: &AcqResult, alternative: &AcqResult) -> String {
    format!(
        "ranked_alternative: preserved rank {} candidate behind rank {} primary (peak_mean_ratio {:.6} < {:.6}, peak_second_ratio {:.6} vs {:.6})",
        alternative.candidate_rank,
        primary.candidate_rank,
        alternative.peak_mean_ratio,
        primary.peak_mean_ratio,
        alternative.peak_second_ratio,
        primary.peak_second_ratio,
    )
}

fn suppress_wrong_prn_correlations(sat_evaluations: &mut [AcquisitionSatEvaluation]) {
    let dominant = sat_evaluations
        .iter()
        .filter_map(|evaluation| evaluation.candidates.first())
        .filter(|candidate| {
            matches!(candidate.hypothesis, AcqHypothesis::Accepted | AcqHypothesis::Ambiguous)
        })
        .max_by(|left, right| {
            left.peak_mean_ratio
                .partial_cmp(&right.peak_mean_ratio)
                .unwrap_or(std::cmp::Ordering::Equal)
        })
        .map(|candidate| (candidate.sat, candidate.peak_mean_ratio));

    let Some((dominant_sat, dominant_peak_mean_ratio)) = dominant else {
        return;
    };

    for evaluation in sat_evaluations {
        if evaluation.search_window_diagnostic.is_some() {
            continue;
        }
        let Some(candidate) = evaluation.candidates.first_mut() else {
            continue;
        };
        if candidate.sat == dominant_sat
            || !matches!(candidate.hypothesis, AcqHypothesis::Ambiguous)
        {
            continue;
        }
        if candidate.peak_second_ratio >= WRONG_PRN_PEAK_SECOND_RATIO_MAX
            || candidate.peak_mean_ratio <= f32::EPSILON
        {
            continue;
        }
        let dominance_ratio = dominant_peak_mean_ratio / candidate.peak_mean_ratio;
        if dominance_ratio < WRONG_PRN_DOMINANCE_RATIO_MIN {
            continue;
        }
        candidate.hypothesis = AcqHypothesis::Rejected;
        candidate.score = 0.0;
        candidate.explain_selection_reason = Some(wrong_prn_candidate_reason(
            candidate.sat,
            dominant_sat,
            candidate.peak_mean_ratio,
            dominant_peak_mean_ratio,
            dominance_ratio,
            candidate.peak_second_ratio,
        ));
    }
}

fn wrong_prn_candidate_reason(
    candidate_sat: SatId,
    dominant_sat: SatId,
    candidate_peak_mean_ratio: f32,
    dominant_peak_mean_ratio: f32,
    dominance_ratio: f32,
    candidate_peak_second_ratio: f32,
) -> String {
    format!(
        "wrong_prn_correlation: prn {} suppressed by stronger prn {} (peak_mean_ratio {:.6} vs {:.6}, dominance_ratio {:.6}, peak_second_ratio {:.6})",
        candidate_sat.prn,
        dominant_sat.prn,
        candidate_peak_mean_ratio,
        dominant_peak_mean_ratio,
        dominance_ratio,
        candidate_peak_second_ratio,
    )
}

fn selected_candidate_reason(
    decision: AcquisitionDecision,
    peak_mean_ratio: f32,
    local_peak_separation_ratio: f32,
    competing_peak_ratio: f32,
    thresholds: &ResolvedAcquisitionThresholds,
) -> String {
    match decision.reason {
        AcquisitionDecisionReason::AcceptedByRatioThresholds
        | AcquisitionDecisionReason::AmbiguousRatioThresholds => format!(
            "{}: peak_mean_ratio={:.6}, local_peak_separation_ratio={:.6}, competing_peak_ratio={:.6}, score={:.6}",
            decision.reason.as_str(),
            peak_mean_ratio,
            local_peak_separation_ratio,
            competing_peak_ratio,
            decision.score,
        ),
        AcquisitionDecisionReason::LowPeakMetric => format!(
            "{}: peak_mean_ratio={:.6} below threshold {:.6}",
            decision.reason.as_str(),
            peak_mean_ratio,
            thresholds.peak_mean_threshold,
        ),
        AcquisitionDecisionReason::MultipathSuspect => unreachable!(
            "multipath_suspect reasons must be rendered through multipath_candidate_reason"
        ),
    }
}

fn classify_delayed_secondary_peak(
    config: &ReceiverPipelineConfig,
    frame: &SamplesFrame,
    signal_model: &AcquisitionSignalModel,
    sat: SatId,
    carrier_hz: f64,
    doppler_rate_hz_per_s: f64,
    code_phase_samples: usize,
    samples_per_code: usize,
    coherent_ms: u32,
    noncoherent: u32,
    peak_mean_ratio: f32,
    peak_second_ratio: f32,
    competing_peak_ratio: f32,
    thresholds: &ResolvedAcquisitionThresholds,
) -> Option<DelayedSecondaryPeakDiagnostic> {
    if peak_mean_ratio < thresholds.peak_mean_threshold
        || peak_second_ratio >= thresholds.peak_second_threshold
        || competing_peak_ratio < thresholds.peak_second_threshold
    {
        return None;
    }
    if !signal_model.supports_secondary_peak_multipath_screening() {
        return None;
    }
    let correlation_profile = measure_code_phase_profile(
        config,
        signal_model,
        frame,
        sat,
        carrier_hz,
        doppler_rate_hz_per_s,
        coherent_ms,
        noncoherent,
    )?;
    delayed_secondary_peak_diagnostic(
        &correlation_profile,
        code_phase_samples,
        samples_per_code,
        signal_model.code_length,
    )
}

fn accumulate_component_correlations(
    frame: &SamplesFrame,
    component: &AcquisitionComponentPlan,
    code_fft: &[Complex<f32>],
    carrier_hz: f64,
    doppler_rate_hz_per_s: f64,
    sample_rate_hz: f64,
    samples_per_code: usize,
    coherent_periods: u32,
    noncoherent: u32,
    fft: &dyn rustfft::Fft<f32>,
    ifft: &dyn rustfft::Fft<f32>,
) -> ComponentCorrelationAccumulation {
    let total_periods = coherent_periods as usize * noncoherent as usize;
    let mut per_period = Vec::with_capacity(total_periods);
    for offset_period in 0..total_periods {
        let start = offset_period * samples_per_code;
        let end = start + samples_per_code;
        let block = &frame.iq[start..end];

        let mixed = wipeoff_search_carrier(
            block,
            carrier_hz,
            doppler_rate_hz_per_s,
            sample_rate_hz,
            start as u64,
            0.0,
        )
        .expect("acquisition carrier wipeoff requires finite carrier inputs");

        let mut input_fft = mixed;
        fft.process(&mut input_fft);

        let mut prod = vec![Complex::zero(); samples_per_code];
        for i in 0..samples_per_code {
            prod[i] = input_fft[i] * code_fft[i].conj();
        }

        ifft.process(&mut prod);
        per_period.push(prod);
    }

    if let Some(secondary_code_period_signs) = component.secondary_code_period_signs() {
        return best_coherent_secondary_code_phase_correlation(
            &per_period,
            coherent_periods as usize,
            secondary_code_period_signs,
        );
    }

    let mut noncoherent_accumulator = vec![0.0f32; samples_per_code];
    let mut per_noncoherent = Vec::with_capacity(noncoherent as usize);
    let coherent_sign_hypotheses =
        component_data_sign_hypotheses(component, coherent_periods).unwrap_or_default();

    for nc in 0..noncoherent as usize {
        let start = nc * coherent_periods as usize;
        let end = start + coherent_periods as usize;
        let coherent_correlation = if coherent_sign_hypotheses.is_empty() {
            coherent_correlation_with_signs(&per_period[start..end], &vec![1; end - start])
        } else {
            best_coherent_data_correlation(&per_period[start..end], &coherent_sign_hypotheses)
        };
        for (accumulator, sample) in
            noncoherent_accumulator.iter_mut().zip(coherent_correlation.iter())
        {
            *accumulator += sample.norm();
        }
        per_noncoherent.push(coherent_correlation);
    }

    ComponentCorrelationAccumulation {
        per_noncoherent,
        noncoherent_accumulator,
        secondary_code_phase_periods: None,
    }
}

fn wipeoff_search_carrier(
    samples: &[Complex<f32>],
    carrier_hz: f64,
    doppler_rate_hz_per_s: f64,
    sample_rate_hz: f64,
    start_sample_index: u64,
    initial_phase_radians: f64,
) -> Result<Vec<Complex<f32>>, SignalError> {
    if doppler_rate_hz_per_s.abs() <= f64::EPSILON {
        return wipeoff_carrier(
            samples,
            carrier_hz,
            sample_rate_hz,
            start_sample_index,
            initial_phase_radians,
        );
    }

    wipeoff_carrier_with_linear_rate(
        samples,
        carrier_hz,
        doppler_rate_hz_per_s,
        sample_rate_hz,
        start_sample_index,
        initial_phase_radians,
    )
}

fn best_coherent_data_correlation(
    per_period: &[Vec<Complex<f32>>],
    sign_hypotheses: &[Vec<i8>],
) -> Vec<Complex<f32>> {
    let samples_per_code = per_period.first().map_or(0, Vec::len);
    let mut best = vec![Complex::zero(); samples_per_code];
    let mut best_norms = vec![0.0f32; samples_per_code];

    for signs in sign_hypotheses {
        let candidate = coherent_correlation_with_signs(per_period, signs);
        for sample_index in 0..samples_per_code {
            let candidate_norm = candidate[sample_index].norm_sqr();
            if candidate_norm > best_norms[sample_index] {
                best_norms[sample_index] = candidate_norm;
                best[sample_index] = candidate[sample_index];
            }
        }
    }

    best
}

fn best_coherent_secondary_code_phase_correlation(
    per_period: &[Vec<Complex<f32>>],
    coherent_periods: usize,
    secondary_code_period_signs: &[i8],
) -> ComponentCorrelationAccumulation {
    let samples_per_code = per_period.first().map_or(0, Vec::len);
    let noncoherent_periods = per_period.len() / coherent_periods.max(1);
    let hypotheses =
        coherent_secondary_code_phase_hypotheses(per_period.len(), secondary_code_period_signs);
    let mut best_accumulation = None;
    let mut best_metrics: Option<(CorrelationMetrics, u32)> = None;

    for hypothesis in hypotheses {
        let mut per_noncoherent = Vec::with_capacity(noncoherent_periods);
        let mut noncoherent_accumulator = vec![0.0f32; samples_per_code];

        for nc in 0..noncoherent_periods {
            let start = nc * coherent_periods;
            let end = start + coherent_periods;
            let coherent_correlation = coherent_correlation_with_signs(
                &per_period[start..end],
                &hypothesis.period_signs[start..end],
            );
            for (accumulator, sample) in
                noncoherent_accumulator.iter_mut().zip(coherent_correlation.iter())
            {
                *accumulator += sample.norm();
            }
            per_noncoherent.push(coherent_correlation);
        }

        let metrics = correlation_metrics(&noncoherent_accumulator);
        let candidate_peak_mean_ratio = metrics.peak / (metrics.mean + 1e-6);
        let candidate_peak_second_ratio = metrics.peak / (metrics.second + 1e-6);
        let replace = best_metrics.is_none_or(|(best, best_phase)| {
            let best_peak_mean_ratio = best.peak / (best.mean + 1e-6);
            let best_peak_second_ratio = best.peak / (best.second + 1e-6);
            candidate_peak_mean_ratio > best_peak_mean_ratio + f32::EPSILON
                || ((candidate_peak_mean_ratio - best_peak_mean_ratio).abs() <= f32::EPSILON
                    && (candidate_peak_second_ratio > best_peak_second_ratio + f32::EPSILON
                        || ((candidate_peak_second_ratio - best_peak_second_ratio).abs()
                            <= f32::EPSILON
                            && hypothesis.secondary_code_phase_periods < best_phase)))
        });
        if replace {
            best_metrics = Some((metrics, hypothesis.secondary_code_phase_periods));
            best_accumulation = Some(ComponentCorrelationAccumulation {
                per_noncoherent,
                noncoherent_accumulator,
                secondary_code_phase_periods: Some(hypothesis.secondary_code_phase_periods),
            });
        }
    }

    best_accumulation.expect("secondary-code phase search must yield at least one hypothesis")
}

fn coherent_correlation_with_signs(
    per_period: &[Vec<Complex<f32>>],
    signs: &[i8],
) -> Vec<Complex<f32>> {
    let samples_per_code = per_period.first().map_or(0, Vec::len);
    let mut coherent = vec![Complex::zero(); samples_per_code];

    for (period_index, period) in per_period.iter().enumerate() {
        let sign = signs.get(period_index).copied().unwrap_or(1) as f32;
        for sample_index in 0..samples_per_code {
            coherent[sample_index] += period[sample_index] * sign;
        }
    }

    coherent
}

fn multipath_suspect_decision(
    peak_mean_ratio: f32,
    peak_second_ratio: f32,
    competing_peak_ratio: f32,
    diagnostic: &DelayedSecondaryPeakDiagnostic,
) -> AcquisitionDecision {
    let delay_penalty = 1.0 / (1.0 + diagnostic.delay_samples as f32);
    let limiting_ratio = peak_second_ratio.min(competing_peak_ratio);
    AcquisitionDecision {
        hypothesis: AcqHypothesis::Ambiguous,
        reason: AcquisitionDecisionReason::MultipathSuspect,
        score: (peak_mean_ratio * 0.35) + (limiting_ratio * 0.15) - delay_penalty.min(0.1),
    }
}

fn multipath_candidate_reason(
    peak_mean_ratio: f32,
    peak_second_ratio: f32,
    competing_peak_ratio: f32,
    diagnostic: &DelayedSecondaryPeakDiagnostic,
    samples_per_code: usize,
    code_length: usize,
    score: f32,
) -> String {
    let delay_chips = diagnostic.delay_samples as f64 * code_length.max(1) as f64
        / samples_per_code.max(1) as f64;
    format!(
        "multipath_suspect: delayed secondary peak at sample {} (+{} samples, {:.3} chips) with peak_mean_ratio={:.6}, peak_second_ratio={:.6}, competing_peak_ratio={:.6}, score={:.6}",
        diagnostic.secondary_code_phase_samples,
        diagnostic.delay_samples,
        delay_chips,
        peak_mean_ratio,
        peak_second_ratio,
        competing_peak_ratio,
        score,
    )
}

fn competing_candidate_ratio(candidates: &[AcqResult]) -> f32 {
    let Some(best) = candidates.first() else {
        return f32::INFINITY;
    };

    candidates
        .iter()
        .skip(1)
        .find(|candidate| {
            !same_acquisition_hypothesis(best, candidate)
                && candidate.peak_mean_ratio > f32::EPSILON
        })
        .map_or(f32::INFINITY, |competing| best.peak_mean_ratio / competing.peak_mean_ratio)
}

fn same_acquisition_hypothesis(left: &AcqResult, right: &AcqResult) -> bool {
    left.sat == right.sat
        && left.signal_band == right.signal_band
        && left.signal_code == right.signal_code
        && left.glonass_frequency_channel == right.glonass_frequency_channel
        && left.code_phase_samples == right.code_phase_samples
        && (left.doppler_rate_hz_per_s - right.doppler_rate_hz_per_s).abs() <= f64::EPSILON
        && (left.carrier_hz.0 - right.carrier_hz.0).abs() <= f64::EPSILON
}

fn refine_acquisition_candidates(
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

fn estimate_acquisition_uncertainty(
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
        measure_local_acquisition_likelihood_volume(
            config,
            signal_model,
            frame,
            candidate.sat,
            candidate.carrier_hz.0,
            candidate.doppler_rate_hz_per_s,
            code_phase_center_samples,
            doppler_step_hz as f64,
            doppler_rate_step_hz_per_s as f64,
            coherent_ms,
            noncoherent,
        )
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
        measure_local_acquisition_likelihood_surface(
            config,
            signal_model,
            frame,
            candidate.sat,
            candidate.carrier_hz.0,
            candidate.doppler_rate_hz_per_s,
            code_phase_center_samples,
            doppler_step_hz as f64,
            coherent_ms,
            noncoherent,
        )
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

fn estimate_acquisition_doppler_refinement(
    coarse_carrier_hz: f64,
    coarse_doppler_rate_hz_per_s: f64,
    grid_candidates: &[AcqResult],
    doppler_step_hz: i32,
) -> Option<AcqDopplerRefinement> {
    if doppler_step_hz <= 0 {
        return None;
    }
    let step_hz = doppler_step_hz as f64;
    let left = find_best_candidate_by_carrier_hz_and_rate(
        grid_candidates,
        coarse_carrier_hz - step_hz,
        coarse_doppler_rate_hz_per_s,
    )?;
    let center = find_best_candidate_by_carrier_hz_and_rate(
        grid_candidates,
        coarse_carrier_hz,
        coarse_doppler_rate_hz_per_s,
    )?;
    let right = find_best_candidate_by_carrier_hz_and_rate(
        grid_candidates,
        coarse_carrier_hz + step_hz,
        coarse_doppler_rate_hz_per_s,
    )?;
    if center.peak_mean_ratio < left.peak_mean_ratio
        || center.peak_mean_ratio < right.peak_mean_ratio
    {
        return None;
    }

    let left_peak_mean_ratio = left.peak_mean_ratio as f64;
    let center_peak_mean_ratio = center.peak_mean_ratio as f64;
    let right_peak_mean_ratio = right.peak_mean_ratio as f64;
    let denominator = left_peak_mean_ratio - (2.0 * center_peak_mean_ratio) + right_peak_mean_ratio;
    if !denominator.is_finite() || denominator.abs() <= SUB_BIN_DOPPLER_REFINEMENT_EPSILON {
        return None;
    }

    let raw_offset_bins = 0.5 * (left_peak_mean_ratio - right_peak_mean_ratio) / denominator;
    if !raw_offset_bins.is_finite() {
        return None;
    }
    let offset_bins = raw_offset_bins.clamp(
        -SUB_BIN_DOPPLER_REFINEMENT_MAX_OFFSET_BINS,
        SUB_BIN_DOPPLER_REFINEMENT_MAX_OFFSET_BINS,
    );

    Some(AcqDopplerRefinement {
        method: SUB_BIN_DOPPLER_REFINEMENT_METHOD.to_string(),
        coarse_carrier_hz: Hertz(coarse_carrier_hz),
        offset_hz: offset_bins * step_hz,
        offset_bins,
        left_peak_mean_ratio: left.peak_mean_ratio,
        center_peak_mean_ratio: center.peak_mean_ratio,
        right_peak_mean_ratio: right.peak_mean_ratio,
    })
}

fn measure_local_acquisition_likelihood_surface(
    config: &ReceiverPipelineConfig,
    signal_model: &AcquisitionSignalModel,
    frame: &SamplesFrame,
    sat: SatId,
    coarse_carrier_hz: f64,
    coarse_doppler_rate_hz_per_s: f64,
    coarse_code_phase_samples: usize,
    doppler_step_hz: f64,
    coherent_ms: u32,
    noncoherent: u32,
) -> Option<LocalAcquisitionLikelihoodSurface> {
    if !doppler_step_hz.is_finite() || doppler_step_hz <= f64::EPSILON {
        return None;
    }
    let samples_per_code = signal_model.samples_per_code(config.sampling_freq_hz);
    if samples_per_code < 3 {
        return None;
    }

    let center_index = coarse_code_phase_samples % samples_per_code;
    let left_index = (center_index + samples_per_code - 1) % samples_per_code;
    let right_index = (center_index + 1) % samples_per_code;
    let mut values = [[0.0f32; 3]; 3];

    for (doppler_index, carrier_hz) in [
        coarse_carrier_hz - doppler_step_hz,
        coarse_carrier_hz,
        coarse_carrier_hz + doppler_step_hz,
    ]
    .into_iter()
    .enumerate()
    {
        let profile = measure_code_phase_profile(
            config,
            signal_model,
            frame,
            sat,
            carrier_hz,
            coarse_doppler_rate_hz_per_s,
            coherent_ms,
            noncoherent,
        )?;
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

fn measure_local_acquisition_likelihood_volume(
    config: &ReceiverPipelineConfig,
    signal_model: &AcquisitionSignalModel,
    frame: &SamplesFrame,
    sat: SatId,
    coarse_carrier_hz: f64,
    coarse_doppler_rate_hz_per_s: f64,
    coarse_code_phase_samples: usize,
    doppler_step_hz: f64,
    doppler_rate_step_hz_per_s: f64,
    coherent_ms: u32,
    noncoherent: u32,
) -> Option<LocalAcquisitionLikelihoodVolume> {
    if !doppler_step_hz.is_finite()
        || doppler_step_hz <= f64::EPSILON
        || !doppler_rate_step_hz_per_s.is_finite()
        || doppler_rate_step_hz_per_s <= f64::EPSILON
    {
        return None;
    }
    let samples_per_code = signal_model.samples_per_code(config.sampling_freq_hz);
    if samples_per_code < 3 {
        return None;
    }

    let center_index = coarse_code_phase_samples % samples_per_code;
    let left_index = (center_index + samples_per_code - 1) % samples_per_code;
    let right_index = (center_index + 1) % samples_per_code;
    let mut values = [[[0.0f32; 3]; 3]; 3];
    let doppler_rates = [
        coarse_doppler_rate_hz_per_s - doppler_rate_step_hz_per_s,
        coarse_doppler_rate_hz_per_s,
        coarse_doppler_rate_hz_per_s + doppler_rate_step_hz_per_s,
    ];
    let carriers = [
        coarse_carrier_hz - doppler_step_hz,
        coarse_carrier_hz,
        coarse_carrier_hz + doppler_step_hz,
    ];

    for (rate_index, doppler_rate_hz_per_s) in doppler_rates.into_iter().enumerate() {
        for (doppler_index, carrier_hz) in carriers.into_iter().enumerate() {
            let profile = measure_code_phase_profile(
                config,
                signal_model,
                frame,
                sat,
                carrier_hz,
                doppler_rate_hz_per_s,
                coherent_ms,
                noncoherent,
            )?;
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

fn estimate_log_likelihood_covariance_2x2(
    surface: &LocalAcquisitionLikelihoodSurface,
    noise_floor: f64,
    doppler_step_hz: f64,
    code_phase_step_samples: f64,
) -> Option<AcqUncertaintyCovariance> {
    let fallback = || {
        Some(AcqUncertaintyCovariance {
            doppler_variance_hz2: estimate_log_likelihood_axis_variance(
                surface.values[0][1] as f64,
                surface.values[1][1] as f64,
                surface.values[2][1] as f64,
                noise_floor,
                doppler_step_hz,
            )?,
            doppler_code_phase_covariance_hz_samples: 0.0,
            code_phase_variance_samples2: estimate_log_likelihood_axis_variance(
                surface.values[1][0] as f64,
                surface.values[1][1] as f64,
                surface.values[1][2] as f64,
                noise_floor,
                code_phase_step_samples,
            )?,
            doppler_rate_variance_hz2_per_s2: None,
            doppler_doppler_rate_covariance_hz2_per_s: None,
            code_phase_doppler_rate_covariance_samples_hz_per_s: None,
        })
    };
    let phi = log_likelihood_surface_values_2x2(&surface.values, noise_floor)?;
    let hessian = [
        [
            centered_second_derivative(phi[0][1], phi[1][1], phi[2][1], doppler_step_hz)?,
            centered_mixed_derivative(
                phi[0][0],
                phi[0][2],
                phi[2][0],
                phi[2][2],
                doppler_step_hz,
                code_phase_step_samples,
            )?,
        ],
        [
            centered_mixed_derivative(
                phi[0][0],
                phi[0][2],
                phi[2][0],
                phi[2][2],
                doppler_step_hz,
                code_phase_step_samples,
            )?,
            centered_second_derivative(phi[1][0], phi[1][1], phi[1][2], code_phase_step_samples)?,
        ],
    ];
    let Some(covariance) = invert_2x2(hessian) else {
        return fallback();
    };
    if covariance[0][0] <= f64::EPSILON || covariance[1][1] <= f64::EPSILON {
        return fallback();
    }
    Some(AcqUncertaintyCovariance {
        doppler_variance_hz2: covariance[0][0],
        doppler_code_phase_covariance_hz_samples: covariance[0][1],
        code_phase_variance_samples2: covariance[1][1],
        doppler_rate_variance_hz2_per_s2: None,
        doppler_doppler_rate_covariance_hz2_per_s: None,
        code_phase_doppler_rate_covariance_samples_hz_per_s: None,
    })
}

fn estimate_log_likelihood_covariance_3x3(
    volume: &LocalAcquisitionLikelihoodVolume,
    noise_floor: f64,
    doppler_step_hz: f64,
    code_phase_step_samples: f64,
    doppler_rate_step_hz_per_s: f64,
) -> Option<AcqUncertaintyCovariance> {
    let fallback = || {
        let center_surface = LocalAcquisitionLikelihoodSurface {
            doppler_cross_section: [
                volume.values[1][0][1],
                volume.values[1][1][1],
                volume.values[1][2][1],
            ],
            code_phase_cross_section: volume.values[1][1],
            values: volume.values[1],
        };
        let mut covariance = estimate_log_likelihood_covariance_2x2(
            &center_surface,
            noise_floor,
            doppler_step_hz,
            code_phase_step_samples,
        )?;
        covariance.doppler_rate_variance_hz2_per_s2 = Some(estimate_log_likelihood_axis_variance(
            volume.values[0][1][1] as f64,
            volume.values[1][1][1] as f64,
            volume.values[2][1][1] as f64,
            noise_floor,
            doppler_rate_step_hz_per_s,
        )?);
        covariance.doppler_doppler_rate_covariance_hz2_per_s = Some(0.0);
        covariance.code_phase_doppler_rate_covariance_samples_hz_per_s = Some(0.0);
        Some(covariance)
    };
    let phi = log_likelihood_surface_values_3x3(&volume.values, noise_floor)?;
    let hessian = [
        [
            centered_second_derivative(phi[1][0][1], phi[1][1][1], phi[1][2][1], doppler_step_hz)?,
            centered_mixed_derivative(
                phi[1][0][0],
                phi[1][0][2],
                phi[1][2][0],
                phi[1][2][2],
                doppler_step_hz,
                code_phase_step_samples,
            )?,
            centered_mixed_derivative(
                phi[0][0][1],
                phi[0][2][1],
                phi[2][0][1],
                phi[2][2][1],
                doppler_step_hz,
                doppler_rate_step_hz_per_s,
            )?,
        ],
        [
            centered_mixed_derivative(
                phi[1][0][0],
                phi[1][0][2],
                phi[1][2][0],
                phi[1][2][2],
                doppler_step_hz,
                code_phase_step_samples,
            )?,
            centered_second_derivative(
                phi[1][1][0],
                phi[1][1][1],
                phi[1][1][2],
                code_phase_step_samples,
            )?,
            centered_mixed_derivative(
                phi[0][1][0],
                phi[0][1][2],
                phi[2][1][0],
                phi[2][1][2],
                code_phase_step_samples,
                doppler_rate_step_hz_per_s,
            )?,
        ],
        [
            centered_mixed_derivative(
                phi[0][0][1],
                phi[0][2][1],
                phi[2][0][1],
                phi[2][2][1],
                doppler_step_hz,
                doppler_rate_step_hz_per_s,
            )?,
            centered_mixed_derivative(
                phi[0][1][0],
                phi[0][1][2],
                phi[2][1][0],
                phi[2][1][2],
                code_phase_step_samples,
                doppler_rate_step_hz_per_s,
            )?,
            centered_second_derivative(
                phi[0][1][1],
                phi[1][1][1],
                phi[2][1][1],
                doppler_rate_step_hz_per_s,
            )?,
        ],
    ];
    let Some(covariance) = invert_3x3(hessian) else {
        return fallback();
    };
    if covariance[0][0] <= f64::EPSILON
        || covariance[1][1] <= f64::EPSILON
        || covariance[2][2] <= f64::EPSILON
    {
        return fallback();
    }
    Some(AcqUncertaintyCovariance {
        doppler_variance_hz2: covariance[0][0],
        doppler_code_phase_covariance_hz_samples: covariance[0][1],
        code_phase_variance_samples2: covariance[1][1],
        doppler_rate_variance_hz2_per_s2: Some(covariance[2][2]),
        doppler_doppler_rate_covariance_hz2_per_s: Some(covariance[0][2]),
        code_phase_doppler_rate_covariance_samples_hz_per_s: Some(covariance[1][2]),
    })
}

fn estimate_log_likelihood_covariance_from_refinement_axes(
    candidate: &AcqResult,
    noise_floor: f64,
    doppler_step_hz: f64,
) -> Option<AcqUncertaintyCovariance> {
    let doppler_refinement = candidate.doppler_refinement.as_ref()?;
    let code_phase_refinement = candidate.code_phase_refinement.as_ref()?;
    Some(AcqUncertaintyCovariance {
        doppler_variance_hz2: estimate_log_likelihood_axis_variance(
            doppler_refinement.left_peak_mean_ratio as f64,
            doppler_refinement.center_peak_mean_ratio as f64,
            doppler_refinement.right_peak_mean_ratio as f64,
            1.0,
            doppler_step_hz,
        )?,
        doppler_code_phase_covariance_hz_samples: 0.0,
        code_phase_variance_samples2: estimate_log_likelihood_axis_variance(
            code_phase_refinement.left_correlation_norm as f64,
            code_phase_refinement.center_correlation_norm as f64,
            code_phase_refinement.right_correlation_norm as f64,
            noise_floor,
            1.0,
        )?,
        doppler_rate_variance_hz2_per_s2: None,
        doppler_doppler_rate_covariance_hz2_per_s: None,
        code_phase_doppler_rate_covariance_samples_hz_per_s: None,
    })
}

fn log_likelihood_surface_values_2x2(
    values: &[[f32; 3]; 3],
    noise_floor: f64,
) -> Option<[[f64; 3]; 3]> {
    let center_excess = excess_likelihood_response(values[1][1] as f64, noise_floor, None)?;
    let floor = Some(center_excess);
    let mut phi = [[0.0_f64; 3]; 3];
    for doppler_index in 0..3 {
        for code_index in 0..3 {
            phi[doppler_index][code_index] = negative_log_likelihood_response(
                values[doppler_index][code_index] as f64,
                noise_floor,
                center_excess,
                floor,
            )?;
        }
    }
    Some(phi)
}

fn log_likelihood_surface_values_3x3(
    values: &[[[f32; 3]; 3]; 3],
    noise_floor: f64,
) -> Option<[[[f64; 3]; 3]; 3]> {
    let center_excess = excess_likelihood_response(values[1][1][1] as f64, noise_floor, None)?;
    let floor = Some(center_excess);
    let mut phi = [[[0.0_f64; 3]; 3]; 3];
    for rate_index in 0..3 {
        for doppler_index in 0..3 {
            for code_index in 0..3 {
                phi[rate_index][doppler_index][code_index] = negative_log_likelihood_response(
                    values[rate_index][doppler_index][code_index] as f64,
                    noise_floor,
                    center_excess,
                    floor,
                )?;
            }
        }
    }
    Some(phi)
}

fn negative_log_likelihood_response(
    value: f64,
    noise_floor: f64,
    center_excess: f64,
    floor_reference: Option<f64>,
) -> Option<f64> {
    let excess = excess_likelihood_response(value, noise_floor, floor_reference)?;
    let normalized = excess / center_excess;
    if !normalized.is_finite() || normalized <= 0.0 {
        return None;
    }
    Some(-normalized.ln())
}

fn excess_likelihood_response(
    value: f64,
    noise_floor: f64,
    floor_reference: Option<f64>,
) -> Option<f64> {
    if !value.is_finite() || !noise_floor.is_finite() {
        return None;
    }
    let epsilon = floor_reference
        .map(|reference| reference * ACQUISITION_UNCERTAINTY_LOG_RESPONSE_FLOOR_RATIO)
        .unwrap_or(1.0e-12);
    let excess = (value - noise_floor).max(epsilon);
    if !excess.is_finite() || excess <= 0.0 {
        return None;
    }
    Some(excess)
}

fn estimate_log_likelihood_axis_variance(
    left: f64,
    center: f64,
    right: f64,
    noise_floor: f64,
    step: f64,
) -> Option<f64> {
    let center_excess = excess_likelihood_response(center, noise_floor, None)?;
    let floor = Some(center_excess);
    let phi_left = negative_log_likelihood_response(left, noise_floor, center_excess, floor)?;
    let phi_center = negative_log_likelihood_response(center, noise_floor, center_excess, floor)?;
    let phi_right = negative_log_likelihood_response(right, noise_floor, center_excess, floor)?;
    let curvature = centered_second_derivative(phi_left, phi_center, phi_right, step)?;
    let variance = 1.0 / curvature;
    (variance.is_finite() && variance > f64::EPSILON).then_some(variance)
}

fn centered_second_derivative(left: f64, center: f64, right: f64, step: f64) -> Option<f64> {
    if !left.is_finite()
        || !center.is_finite()
        || !right.is_finite()
        || !step.is_finite()
        || step <= f64::EPSILON
    {
        return None;
    }
    let derivative = (left - (2.0 * center) + right) / step.powi(2);
    (derivative.is_finite() && derivative > SUB_BIN_DOPPLER_REFINEMENT_EPSILON)
        .then_some(derivative)
}

fn centered_mixed_derivative(
    lower_lower: f64,
    lower_upper: f64,
    upper_lower: f64,
    upper_upper: f64,
    step_a: f64,
    step_b: f64,
) -> Option<f64> {
    if !lower_lower.is_finite()
        || !lower_upper.is_finite()
        || !upper_lower.is_finite()
        || !upper_upper.is_finite()
        || !step_a.is_finite()
        || !step_b.is_finite()
        || step_a <= f64::EPSILON
        || step_b <= f64::EPSILON
    {
        return None;
    }
    let derivative =
        (upper_upper - upper_lower - lower_upper + lower_lower) / (4.0 * step_a * step_b);
    derivative.is_finite().then_some(derivative)
}

fn invert_2x2(matrix: [[f64; 2]; 2]) -> Option<[[f64; 2]; 2]> {
    let determinant = matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0];
    if !determinant.is_finite() || determinant.abs() <= SUB_BIN_DOPPLER_REFINEMENT_EPSILON {
        return None;
    }
    Some([
        [matrix[1][1] / determinant, -matrix[0][1] / determinant],
        [-matrix[1][0] / determinant, matrix[0][0] / determinant],
    ])
}

fn invert_3x3(matrix: [[f64; 3]; 3]) -> Option<[[f64; 3]; 3]> {
    let determinant = matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1])
        - matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
        + matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]);
    if !determinant.is_finite() || determinant.abs() <= SUB_BIN_DOPPLER_REFINEMENT_EPSILON {
        return None;
    }
    Some([
        [
            (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1]) / determinant,
            (matrix[0][2] * matrix[2][1] - matrix[0][1] * matrix[2][2]) / determinant,
            (matrix[0][1] * matrix[1][2] - matrix[0][2] * matrix[1][1]) / determinant,
        ],
        [
            (matrix[1][2] * matrix[2][0] - matrix[1][0] * matrix[2][2]) / determinant,
            (matrix[0][0] * matrix[2][2] - matrix[0][2] * matrix[2][0]) / determinant,
            (matrix[0][2] * matrix[1][0] - matrix[0][0] * matrix[1][2]) / determinant,
        ],
        [
            (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]) / determinant,
            (matrix[0][1] * matrix[2][0] - matrix[0][0] * matrix[2][1]) / determinant,
            (matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0]) / determinant,
        ],
    ])
}

fn estimate_quadratic_surface_peak_offsets(
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
        || doppler_curvature >= -SUB_BIN_DOPPLER_REFINEMENT_EPSILON
        || code_curvature >= -SUB_SAMPLE_CODE_PHASE_REFINEMENT_EPSILON
    {
        return None;
    }

    let determinant = (4.0 * doppler_curvature * code_curvature) - cross_curvature.powi(2);
    if !determinant.is_finite() || determinant <= SUB_BIN_DOPPLER_REFINEMENT_EPSILON {
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
        doppler_offset_bins.clamp(
            -SUB_BIN_DOPPLER_REFINEMENT_MAX_OFFSET_BINS,
            SUB_BIN_DOPPLER_REFINEMENT_MAX_OFFSET_BINS,
        ),
        code_phase_offset_samples.clamp(
            -SUB_SAMPLE_CODE_PHASE_REFINEMENT_MAX_OFFSET_SAMPLES,
            SUB_SAMPLE_CODE_PHASE_REFINEMENT_MAX_OFFSET_SAMPLES,
        ),
    ))
}

fn measure_code_phase_profile(
    config: &ReceiverPipelineConfig,
    signal_model: &AcquisitionSignalModel,
    frame: &SamplesFrame,
    _sat: SatId,
    carrier_hz: f64,
    doppler_rate_hz_per_s: f64,
    coherent_ms: u32,
    noncoherent: u32,
) -> Option<Vec<f32>> {
    let samples_per_code = signal_model.samples_per_code(config.sampling_freq_hz);
    let coherent_periods = signal_model.coherent_periods(coherent_ms)?;
    if samples_per_code == 0 {
        return None;
    }
    let mut planner = FftPlanner::<f32>::new();
    let fft = planner.plan_fft_forward(samples_per_code);
    let ifft = planner.plan_fft_inverse(samples_per_code);
    let local_code = signal_model
        .sampled_local_code_period(config.sampling_freq_hz, samples_per_code)
        .unwrap_or_else(|_| vec![1.0; samples_per_code]);
    let mut code_fft: Vec<Complex<f32>> =
        local_code.iter().map(|&x| Complex::new(x, 0.0)).collect();
    fft.process(&mut code_fft);
    let mut noncoherent_acc = vec![0.0f32; samples_per_code];

    for nc in 0..noncoherent {
        let mut coherent_corr: Vec<Complex<f32>> = vec![Complex::zero(); samples_per_code];
        for c in 0..coherent_periods {
            let offset_period = (nc * coherent_periods + c) as usize;
            let start = offset_period * samples_per_code;
            let end = start + samples_per_code;
            let block = &frame.iq[start..end];
            let mixed = wipeoff_search_carrier(
                block,
                carrier_hz,
                doppler_rate_hz_per_s,
                config.sampling_freq_hz,
                start as u64,
                0.0,
            )
            .ok()?;
            let mut input_fft = mixed;
            fft.process(&mut input_fft);

            let mut prod = vec![Complex::zero(); samples_per_code];
            for i in 0..samples_per_code {
                prod[i] = input_fft[i] * code_fft[i].conj();
            }

            ifft.process(&mut prod);
            for i in 0..samples_per_code {
                coherent_corr[i] += prod[i];
            }
        }

        for i in 0..samples_per_code {
            noncoherent_acc[i] += coherent_corr[i].norm();
        }
    }

    Some(noncoherent_acc)
}

fn estimate_parabolic_code_phase_offset_samples(
    correlation_profile: &[f32],
    coarse_code_phase_samples: usize,
) -> Option<(f64, f32, f32, f32)> {
    if correlation_profile.len() < 3 {
        return None;
    }
    let period_samples = correlation_profile.len();
    let center_index = coarse_code_phase_samples % period_samples;
    let left_index = (center_index + period_samples - 1) % period_samples;
    let right_index = (center_index + 1) % period_samples;
    let left = correlation_profile[left_index];
    let center = correlation_profile[center_index];
    let right = correlation_profile[right_index];

    if center < left || center < right {
        return None;
    }

    let denominator = left as f64 - (2.0 * center as f64) + right as f64;
    if !denominator.is_finite() || denominator.abs() <= SUB_SAMPLE_CODE_PHASE_REFINEMENT_EPSILON {
        return None;
    }

    let raw_offset_samples = 0.5 * (left as f64 - right as f64) / denominator;
    if !raw_offset_samples.is_finite() {
        return None;
    }

    let offset_samples = raw_offset_samples.clamp(
        -SUB_SAMPLE_CODE_PHASE_REFINEMENT_MAX_OFFSET_SAMPLES,
        SUB_SAMPLE_CODE_PHASE_REFINEMENT_MAX_OFFSET_SAMPLES,
    );
    if offset_samples.abs() < SUB_SAMPLE_CODE_PHASE_REFINEMENT_MIN_ABS_OFFSET_SAMPLES {
        return None;
    }

    Some((offset_samples, left, center, right))
}

fn wrap_acquisition_code_phase_samples(code_phase_samples: f64, period_samples: usize) -> f64 {
    let period_samples = period_samples.max(1) as f64;
    code_phase_samples.rem_euclid(period_samples)
}

fn find_candidate_by_carrier_hz(candidates: &[AcqResult], carrier_hz: f64) -> Option<&AcqResult> {
    candidates
        .iter()
        .filter(|candidate| (candidate.carrier_hz.0 - carrier_hz).abs() <= f64::EPSILON)
        .max_by(|left, right| {
            left.peak_mean_ratio
                .partial_cmp(&right.peak_mean_ratio)
                .unwrap_or(std::cmp::Ordering::Equal)
        })
}

fn find_best_candidate_by_carrier_hz_and_rate(
    candidates: &[AcqResult],
    carrier_hz: f64,
    doppler_rate_hz_per_s: f64,
) -> Option<&AcqResult> {
    candidates
        .iter()
        .filter(|candidate| {
            (candidate.carrier_hz.0 - carrier_hz).abs() <= f64::EPSILON
                && (candidate.doppler_rate_hz_per_s - doppler_rate_hz_per_s).abs() <= f64::EPSILON
        })
        .max_by(|left, right| {
            left.peak_mean_ratio
                .partial_cmp(&right.peak_mean_ratio)
                .unwrap_or(std::cmp::Ordering::Equal)
        })
}

#[allow(clippy::items_after_test_module)]
#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::runtime::ReceiverRuntime;
    use crate::sim::synthetic::{
        generate_l1_ca, SyntheticScenario, SyntheticSignalParams, SyntheticSignalSource,
    };
    use bijux_gnss_core::api::{
        AcqAssistanceBounds, AcqComponentCombinationMode, AcqComponentProvenance,
        AcqComponentStatistic, Constellation, GlonassFrequencyChannel, ReceiverSampleTrace,
        SampleTime, SamplesFrame, SatId, Seconds, SignalBand, SignalComponentRole,
        GPS_L1_CA_CARRIER_HZ,
    };
    use bijux_gnss_signal::api::{
        glonass_l1_carrier_hz, sample_glonass_l1_st_code, samples_per_code, shared_path_doppler_hz,
        signal_spec_gps_l1_ca, signal_spec_gps_l5_i, SignalSource,
    };

    fn acquisition_component_plan_for_signal(
        sat: SatId,
        signal_band: SignalBand,
        signal_code: SignalCode,
        coherent_ms: u32,
    ) -> AcquisitionComponentPlan {
        acquisition_strategies_for_signal(sat, signal_band, signal_code, None, coherent_ms)
            .expect("acquisition strategies")
            .into_iter()
            .next()
            .and_then(|strategy| strategy.components.into_iter().next())
            .expect("primary acquisition component")
    }

    fn alternating_frame(sample_rate_hz: f64, sample_count: usize) -> SamplesFrame {
        SamplesFrame::new(
            SampleTime { sample_index: 0, sample_rate_hz },
            Seconds(1.0 / sample_rate_hz),
            (0..sample_count)
                .map(
                    |idx| {
                        if idx % 2 == 0 {
                            Complex::new(1.0, 0.0)
                        } else {
                            Complex::new(-1.0, 0.0)
                        }
                    },
                )
                .collect(),
        )
    }

    fn signal_only_frame(
        config: &ReceiverPipelineConfig,
        scenario: &SyntheticScenario,
        frame_len: usize,
    ) -> SamplesFrame {
        let mut source = SyntheticSignalSource::new_signal_only(config, scenario);
        source.next_frame(frame_len).expect("signal-only frame").expect("acquisition frame")
    }

    fn resolved_thresholds(config: &ReceiverPipelineConfig) -> ResolvedAcquisitionThresholds {
        ResolvedAcquisitionThresholds {
            peak_mean_threshold: config.acquisition_peak_mean_threshold,
            peak_second_threshold: config.acquisition_peak_second_threshold,
            provenance: AcqThresholdProvenance {
                mode: "fixed_ratio".to_string(),
                coherent_ms: config.acquisition_integration_ms,
                noncoherent: config.acquisition_noncoherent,
                doppler_search_hz: config.acquisition_doppler_search_hz,
                doppler_step_hz: config.acquisition_doppler_step_hz,
                doppler_rate_search_hz_per_s: config.acquisition_doppler_rate_search_hz_per_s,
                doppler_rate_step_hz_per_s: config.acquisition_doppler_rate_step_hz_per_s,
                peak_mean_threshold: config.acquisition_peak_mean_threshold,
                peak_second_threshold: config.acquisition_peak_second_threshold,
                false_alarm_probability: None,
                calibration_trial_count: None,
                calibration_confidence_level: None,
                calibration_false_alarm_rate: None,
                calibration_false_alarm_interval_low: None,
                calibration_false_alarm_interval_high: None,
            },
        }
    }

    #[test]
    fn best_coherent_data_correlation_restores_unknown_symbol_flip_gain() {
        let per_period = vec![
            vec![Complex::new(1.0, 0.0), Complex::new(0.5, 0.0)],
            vec![Complex::new(-1.0, 0.0), Complex::new(0.5, 0.0)],
        ];
        let signs = coherent_data_sign_hypotheses(2, 20);

        let coherent = best_coherent_data_correlation(&per_period, &signs);

        assert!((coherent[0].norm() - 2.0).abs() <= f32::EPSILON);
        assert!((coherent[1].norm() - 1.0).abs() <= f32::EPSILON);
    }

    #[test]
    fn best_coherent_secondary_code_phase_correlation_selects_global_phase() {
        let per_period = vec![
            vec![Complex::new(-3.0, 0.0)],
            vec![Complex::new(-2.0, 0.0)],
            vec![Complex::new(5.0, 0.0)],
            vec![Complex::new(-4.0, 0.0)],
        ];

        let accumulation =
            best_coherent_secondary_code_phase_correlation(&per_period, 2, &[1, -1, -1]);

        assert_eq!(accumulation.secondary_code_phase_periods, Some(1));
        assert_eq!(accumulation.per_noncoherent.len(), 2);
        assert!((accumulation.noncoherent_accumulator[0] - 14.0).abs() <= f32::EPSILON);
    }

    #[test]
    fn coherent_correlation_with_signs_preserves_fixed_sign_sum_without_hypotheses() {
        let per_period = vec![
            vec![Complex::new(1.0, 0.0), Complex::new(0.25, -0.25)],
            vec![Complex::new(2.0, 0.0), Complex::new(0.75, 0.25)],
        ];

        let coherent = coherent_correlation_with_signs(&per_period, &[1, 1]);

        assert_eq!(coherent, vec![Complex::new(3.0, 0.0), Complex::new(1.0, 0.0)]);
    }

    #[test]
    fn strategy_uses_data_sign_hypotheses_for_long_gps_l1_coherent_windows() {
        let sat = SatId { constellation: Constellation::Gps, prn: 3 };
        let strategy =
            acquisition_strategies_for_signal(sat, SignalBand::L1, SignalCode::Ca, None, 20)
                .expect("GPS L1 acquisition strategies")
                .into_iter()
                .next()
                .expect("GPS L1 acquisition strategy");

        assert!(strategy_uses_data_sign_hypotheses(&strategy, 20));
        assert!(!strategy_uses_data_sign_hypotheses(&strategy, 1));
    }

    #[test]
    fn candidate_uses_data_sign_hypotheses_matches_strategy_provenance() {
        let sat = SatId { constellation: Constellation::Galileo, prn: 11 };
        let strategies =
            acquisition_strategies_for_signal(sat, SignalBand::E5, SignalCode::E5b, None, 20)
                .expect("Galileo E5b acquisition strategies");
        let candidate = AcqResult {
            sat,
            signal_band: SignalBand::E5,
            signal_code: SignalCode::E5b,
            glonass_frequency_channel: None,
            source_time: ReceiverSampleTrace::default(),
            candidate_rank: 1,
            is_primary_candidate: true,
            doppler_hz: Hertz(0.0),
            doppler_rate_hz_per_s: 0.0,
            carrier_hz: Hertz(0.0),
            code_phase_samples: 0,
            peak: 10.0,
            second_peak: 5.0,
            mean: 2.0,
            peak_mean_ratio: 5.0,
            peak_second_ratio: 2.0,
            cn0_proxy: 50.0,
            score: 0.0,
            hypothesis: AcqHypothesis::Deferred,
            assumptions: None,
            evidence: vec![AcqEvidence {
                rank: 1,
                code_phase_samples: 0,
                doppler_hz: 0.0,
                doppler_rate_hz_per_s: 0.0,
                peak: 10.0,
                second_peak: 5.0,
                peak_mean_ratio: 5.0,
                peak_second_ratio: 2.0,
                mean: 2.0,
                component_provenance: Some(AcqComponentProvenance {
                    combination_mode: AcqComponentCombinationMode::SingleComponent,
                    components: vec![AcqComponentStatistic {
                        role: SignalComponentRole::Data,
                        peak: 10.0,
                        second_peak: 5.0,
                        mean: 2.0,
                        peak_mean_ratio: 5.0,
                        peak_second_ratio: 2.0,
                        secondary_code_phase_periods: None,
                    }],
                }),
            }],
            threshold_provenance: None,
            explain_selection_reason: None,
            doppler_refinement: None,
            code_phase_refinement: None,
            signal_delay_alignment: None,
            uncertainty: None,
        };

        assert!(candidate_uses_data_sign_hypotheses(&candidate, &strategies, 20));
        assert!(!candidate_uses_data_sign_hypotheses(&candidate, &strategies, 1));
    }

    #[test]
    fn acquisition_decision_rejects_weak_primary_peak() {
        let config = ReceiverPipelineConfig::default();
        let thresholds = resolved_thresholds(&config);
        let decision = acquisition_decision(2.0, 2.0, 2.0, 2.0, &thresholds);
        assert_eq!(decision.hypothesis.to_string(), "rejected");
        assert_eq!(decision.reason, AcquisitionDecisionReason::LowPeakMetric);
        assert_eq!(decision.score, 0.0);
    }

    #[test]
    fn acquisition_decision_marks_ambiguous_on_low_peak_separation() {
        let config = ReceiverPipelineConfig::default();
        let thresholds = resolved_thresholds(&config);
        let decision = acquisition_decision(3.0, 1.0, 1.4, 2.0, &thresholds);
        assert_eq!(decision.hypothesis.to_string(), "ambiguous");
        assert_eq!(decision.reason, AcquisitionDecisionReason::AmbiguousRatioThresholds);
        assert!((decision.score - 1.2).abs() < 1e-6);
    }

    #[test]
    fn acquisition_decision_marks_ambiguous_on_comparable_competing_candidate() {
        let config = ReceiverPipelineConfig::default();
        let thresholds = resolved_thresholds(&config);
        let decision = acquisition_decision(3.5, 2.0, 2.0, 1.4, &thresholds);
        assert_eq!(decision.hypothesis.to_string(), "ambiguous");
        assert_eq!(decision.reason, AcquisitionDecisionReason::AmbiguousRatioThresholds);
        assert!((decision.score - 1.435).abs() < 1e-6);
    }

    #[test]
    fn acquisition_decision_accepts_clean_peak() {
        let config = ReceiverPipelineConfig::default();
        let thresholds = resolved_thresholds(&config);
        let decision = acquisition_decision(3.5, 2.0, 2.5, 2.1, &thresholds);
        assert_eq!(decision.hypothesis.to_string(), "accepted");
        assert_eq!(decision.reason, AcquisitionDecisionReason::AcceptedByRatioThresholds);
        assert!((decision.score - 2.75).abs() < f32::EPSILON);
    }

    #[test]
    fn glonass_request_requires_frequency_channel() {
        let sat = SatId { constellation: Constellation::Glonass, prn: 8 };
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 511_000.0,
            intermediate_freq_hz: 125_000.0,
            code_freq_basis_hz: 511_000.0,
            code_length: 511,
            ..ReceiverPipelineConfig::default()
        };
        let error = acquisition_signal_model_for_request(
            &config,
            AcqRequest {
                sat,
                glonass_frequency_channel: None,
                signal_band: SignalBand::L1,
                signal_code: SignalCode::Unknown,
                doppler_center_hz: 0.0,
                doppler_rate_center_hz_per_s: 0.0,
                doppler_rate_search_hz_per_s: 0,
                doppler_rate_step_hz_per_s: 250,
                expected_line_of_sight_doppler_hz: None,
                assistance_bounds: None,
                doppler_search_hz: 2_000,
                doppler_step_hz: 250,
                coherent_ms: 1,
                noncoherent: 1,
            },
        )
        .expect_err("GLONASS requests must declare one FDMA channel");

        assert!(
            matches!(error, SignalError::MissingGlonassFrequencyChannel(error_sat) if error_sat == sat)
        );
    }

    #[test]
    fn unsupported_registry_signal_request_returns_explicit_error() {
        let sat = SatId { constellation: Constellation::Gps, prn: 11 };
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 511_500.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 511_500.0,
            code_length: 10_230,
            ..ReceiverPipelineConfig::default()
        };
        let error = acquisition_signal_model_for_request(
            &config,
            AcqRequest {
                sat,
                glonass_frequency_channel: None,
                signal_band: SignalBand::L2,
                signal_code: SignalCode::L2C,
                doppler_center_hz: 0.0,
                doppler_rate_center_hz_per_s: 0.0,
                doppler_rate_search_hz_per_s: 0,
                doppler_rate_step_hz_per_s: 250,
                expected_line_of_sight_doppler_hz: None,
                assistance_bounds: None,
                doppler_search_hz: 2_000,
                doppler_step_hz: 250,
                coherent_ms: 1,
                noncoherent: 1,
            },
        )
        .expect_err("GPS L2C acquisition must report unsupported search implementation");

        assert_eq!(
            error,
            SignalError::UnsupportedSignalDefinition {
                constellation: Constellation::Gps,
                signal_band: SignalBand::L2,
                signal_code: SignalCode::L2C,
            }
        );
    }

    #[test]
    fn glonass_request_uses_glonass_l1_search_model() {
        let sat = SatId { constellation: Constellation::Glonass, prn: 8 };
        let channel = GlonassFrequencyChannel::new(-4).expect("channel -4 must be valid");
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 511_000.0,
            intermediate_freq_hz: 125_000.0,
            code_freq_basis_hz: 511_000.0,
            code_length: 511,
            ..ReceiverPipelineConfig::default()
        };
        let model = acquisition_signal_model_for_request(
            &config,
            AcqRequest {
                sat,
                glonass_frequency_channel: Some(channel),
                signal_band: SignalBand::L1,
                signal_code: SignalCode::Unknown,
                doppler_center_hz: 0.0,
                doppler_rate_center_hz_per_s: 0.0,
                doppler_rate_search_hz_per_s: 0,
                doppler_rate_step_hz_per_s: 250,
                expected_line_of_sight_doppler_hz: None,
                assistance_bounds: None,
                doppler_search_hz: 2_000,
                doppler_step_hz: 250,
                coherent_ms: 1,
                noncoherent: 1,
            },
        )
        .expect("GLONASS request model");
        let expected_center_hz = config.intermediate_freq_hz
            + (glonass_l1_carrier_hz(channel).value() - GPS_L1_CA_CARRIER_HZ.value());
        let sampled_code = model
            .sampled_local_code_period(config.sampling_freq_hz, 511)
            .expect("GLONASS local code");
        let expected_code =
            sample_glonass_l1_st_code(config.sampling_freq_hz, 0.0, 511).expect("GLONASS code");

        assert_eq!(model.signal_band, SignalBand::L1);
        assert_eq!(model.code_rate_hz, 511_000.0);
        assert_eq!(model.code_length, 511);
        assert_eq!(model.code_period_ms, 1);
        assert!(
            (model.search_center_hz(config.intermediate_freq_hz) - expected_center_hz).abs()
                <= f64::EPSILON
        );
        assert_eq!(sampled_code, expected_code);
    }

    #[test]
    fn glonass_request_skips_secondary_peak_multipath_screening() {
        let sat = SatId { constellation: Constellation::Glonass, prn: 8 };
        let channel = GlonassFrequencyChannel::new(-4).expect("channel -4 must be valid");
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 511_000.0,
            intermediate_freq_hz: 125_000.0,
            code_freq_basis_hz: 511_000.0,
            code_length: 511,
            ..ReceiverPipelineConfig::default()
        };
        let model = acquisition_signal_model_for_request(
            &config,
            AcqRequest {
                sat,
                glonass_frequency_channel: Some(channel),
                signal_band: SignalBand::L1,
                signal_code: SignalCode::Unknown,
                doppler_center_hz: 0.0,
                doppler_rate_center_hz_per_s: 0.0,
                doppler_rate_search_hz_per_s: 0,
                doppler_rate_step_hz_per_s: 250,
                expected_line_of_sight_doppler_hz: None,
                assistance_bounds: None,
                doppler_search_hz: 2_000,
                doppler_step_hz: 250,
                coherent_ms: 1,
                noncoherent: 1,
            },
        )
        .expect("GLONASS request model");

        assert!(!model.supports_secondary_peak_multipath_screening());
    }

    #[test]
    fn zero_signal_run_preserves_unsupported_request_error() {
        let sat = SatId { constellation: Constellation::Gps, prn: 11 };
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 511_500.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 511_500.0,
            code_length: 10_230,
            ..ReceiverPipelineConfig::default()
        };
        let request = AcqRequest {
            sat,
            glonass_frequency_channel: None,
            signal_band: SignalBand::L2,
            signal_code: SignalCode::L2C,
            doppler_center_hz: 0.0,
            doppler_rate_center_hz_per_s: 0.0,
            doppler_rate_search_hz_per_s: 0,
            doppler_rate_step_hz_per_s: 250,
            expected_line_of_sight_doppler_hz: None,
            assistance_bounds: None,
            doppler_search_hz: 2_000,
            doppler_step_hz: 250,
            coherent_ms: 1,
            noncoherent: 1,
        };

        let run = zero_signal_run(
            &config,
            &[request],
            ReceiverSampleTrace::from_sample_time(SampleTime {
                sample_index: 0,
                sample_rate_hz: config.sampling_freq_hz,
            }),
            4092,
            Some("zeroed_fixture"),
            true,
        );

        assert_eq!(run.results.len(), 1);
        assert_eq!(run.results[0].len(), 1);
        let candidate = &run.results[0][0];
        assert_eq!(candidate.hypothesis.to_string(), AcqHypothesis::Deferred.to_string());
        assert_eq!(
            candidate.explain_selection_reason.as_deref(),
            Some("invalid_acquisition_signal_model: unsupported signal definition for Gps L2 L2C")
        );
        assert_eq!(candidate.carrier_hz.0, config.intermediate_freq_hz);
        assert_eq!(run.explains.len(), 1);
        assert_eq!(run.explains[0].selected_reason, "invalid_acquisition_signal_model");
    }

    #[test]
    fn zero_signal_run_preserves_explicit_gps_l5q_signal_code() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 10_230_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 10_230_000.0,
            code_length: 10_230,
            ..ReceiverPipelineConfig::default()
        };
        let request = AcqRequest {
            sat: SatId { constellation: Constellation::Gps, prn: 7 },
            glonass_frequency_channel: None,
            signal_band: SignalBand::L5,
            signal_code: SignalCode::L5Q,
            doppler_center_hz: 0.0,
            doppler_rate_center_hz_per_s: 0.0,
            doppler_rate_search_hz_per_s: 0,
            doppler_rate_step_hz_per_s: 250,
            expected_line_of_sight_doppler_hz: None,
            assistance_bounds: None,
            doppler_search_hz: 2_000,
            doppler_step_hz: 250,
            coherent_ms: 1,
            noncoherent: 1,
        };

        let run = zero_signal_run(
            &config,
            &[request],
            ReceiverSampleTrace::from_sample_time(SampleTime {
                sample_index: 0,
                sample_rate_hz: config.sampling_freq_hz,
            }),
            10_230,
            Some("zeroed_fixture"),
            true,
        );

        let candidate = &run.results[0][0];
        assert_eq!(candidate.signal_band, SignalBand::L5);
        assert_eq!(candidate.signal_code, SignalCode::L5Q);
        assert_eq!(candidate.hypothesis.to_string(), AcqHypothesis::Rejected.to_string());
        assert_eq!(run.explains[0].selected_reason, "zero_signal_input");
    }

    #[test]
    fn zero_signal_run_preserves_explicit_galileo_e5b_signal_code() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 10_230_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 10_230_000.0,
            code_length: 10_230,
            ..ReceiverPipelineConfig::default()
        };
        let request = AcqRequest {
            sat: SatId { constellation: Constellation::Galileo, prn: 11 },
            glonass_frequency_channel: None,
            signal_band: SignalBand::E5,
            signal_code: SignalCode::E5b,
            doppler_center_hz: 0.0,
            doppler_rate_center_hz_per_s: 0.0,
            doppler_rate_search_hz_per_s: 0,
            doppler_rate_step_hz_per_s: 250,
            expected_line_of_sight_doppler_hz: None,
            assistance_bounds: None,
            doppler_search_hz: 2_000,
            doppler_step_hz: 250,
            coherent_ms: 1,
            noncoherent: 1,
        };

        let run = zero_signal_run(
            &config,
            &[request],
            ReceiverSampleTrace::from_sample_time(SampleTime {
                sample_index: 0,
                sample_rate_hz: config.sampling_freq_hz,
            }),
            10_230,
            Some("zeroed_fixture"),
            true,
        );

        let candidate = &run.results[0][0];
        assert_eq!(candidate.signal_band, SignalBand::E5);
        assert_eq!(candidate.signal_code, SignalCode::E5b);
        assert_eq!(candidate.hypothesis.to_string(), AcqHypothesis::Rejected.to_string());
        assert_eq!(run.explains[0].selected_reason, "zero_signal_input");
    }

    #[test]
    fn competing_candidate_ratio_uses_top_two_peak_mean_ratios() {
        let sat = SatId { constellation: Constellation::Gps, prn: 1 };
        let ratio = competing_candidate_ratio(&[
            candidate_for_search_window_test(sat, 0.0, 4.0),
            candidate_for_search_window_test(sat, 250.0, 3.0),
            candidate_for_search_window_test(sat, 500.0, 1.5),
        ]);

        assert!((ratio - (4.0 / 3.0)).abs() < 1.0e-6);
    }

    #[test]
    fn acquisition_decision_accepts_absent_competing_candidate_with_strong_local_peak() {
        let config = ReceiverPipelineConfig::default();
        let thresholds = resolved_thresholds(&config);
        let decision = acquisition_decision(12.0, 1.8, 1.8, f32::INFINITY, &thresholds);

        assert_eq!(decision.hypothesis.to_string(), "accepted");
        assert_eq!(decision.reason, AcquisitionDecisionReason::AcceptedByRatioThresholds);
    }

    #[test]
    fn selected_candidate_reason_reports_low_peak_metric_threshold() {
        let config = ReceiverPipelineConfig::default();
        let thresholds = resolved_thresholds(&config);
        let reason = selected_candidate_reason(
            AcquisitionDecision {
                hypothesis: AcqHypothesis::Rejected,
                reason: AcquisitionDecisionReason::LowPeakMetric,
                score: 0.0,
            },
            2.0,
            2.0,
            2.0,
            &thresholds,
        );

        assert_eq!(reason, "low_peak_metric: peak_mean_ratio=2.000000 below threshold 2.500000");
    }

    #[test]
    fn calibrated_thresholds_record_false_alarm_provenance() {
        let sat = SatId { constellation: Constellation::Gps, prn: 7 };
        let mut config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            acquisition_doppler_search_hz: 1_500,
            acquisition_doppler_step_hz: 500,
            acquisition_integration_ms: 1,
            acquisition_noncoherent: 1,
            ..ReceiverPipelineConfig::default()
        };
        config.acquisition_threshold_policy.mode = AcquisitionThresholdMode::CalibratedFalseAlarm;
        config.acquisition_threshold_policy.false_alarm_probability = 0.05;
        config.acquisition_threshold_policy.calibration_trial_count = 12;
        config.acquisition_threshold_policy.confidence_level = 0.95;

        let frame = generate_l1_ca(
            &config,
            SyntheticSignalParams {
                sat,
                glonass_frequency_channel: None,
                signal_band: SignalBand::L1,
                signal_code: SignalCode::Ca,
                doppler_hz: 0.0,
                code_phase_chips: 217.0,
                carrier_phase_rad: 0.0,
                cn0_db_hz: 52.0,
                navigation_data: false.into(),
            },
            0x6A11_E177,
            0.005,
        );
        let acquisition = Acquisition::new(config.clone(), ReceiverRuntime::default());
        let result = acquisition.run_fft(&frame, &[sat]).remove(0);
        let provenance = result
            .threshold_provenance
            .as_ref()
            .expect("acquisition result should carry threshold provenance");

        assert_eq!(provenance.mode, "calibrated_false_alarm");
        assert_eq!(
            provenance.false_alarm_probability,
            Some(config.acquisition_threshold_policy.false_alarm_probability)
        );
        assert_eq!(
            provenance.calibration_trial_count,
            Some(config.acquisition_threshold_policy.calibration_trial_count)
        );
        assert_eq!(
            provenance.calibration_confidence_level,
            Some(config.acquisition_threshold_policy.confidence_level)
        );
        assert!(provenance.peak_mean_threshold > 1.0);
        assert_eq!(provenance.peak_second_threshold, config.acquisition_peak_second_threshold);
        let measured_rate = provenance
            .calibration_false_alarm_rate
            .expect("calibrated threshold provenance should record the measured rate");
        let interval_low = provenance
            .calibration_false_alarm_interval_low
            .expect("calibrated threshold provenance should record the confidence interval low");
        let interval_high = provenance
            .calibration_false_alarm_interval_high
            .expect("calibrated threshold provenance should record the confidence interval high");
        assert!((0.0..=1.0).contains(&measured_rate));
        assert!(interval_low <= measured_rate);
        assert!(measured_rate <= interval_high);
    }

    #[test]
    fn selected_reason_for_candidate_reports_low_peak_metric() {
        let sat = SatId { constellation: Constellation::Gps, prn: 1 };
        let mut candidate = candidate_for_search_window_test(sat, 0.0, 2.0);
        candidate.hypothesis = AcqHypothesis::Rejected;

        assert_eq!(selected_reason_for_candidate(&candidate), "low_peak_metric");
    }

    #[test]
    fn selected_reason_for_candidate_reports_wrong_prn_correlation() {
        let sat = SatId { constellation: Constellation::Gps, prn: 8 };
        let mut candidate = candidate_for_search_window_test(sat, 0.0, 3.0);
        candidate.hypothesis = AcqHypothesis::Rejected;
        candidate.explain_selection_reason =
            Some("wrong_prn_correlation: prn 8 suppressed by stronger prn 7".to_string());

        assert_eq!(selected_reason_for_candidate(&candidate), "wrong_prn_correlation");
    }

    #[test]
    fn selected_reason_for_candidate_reports_multipath_suspect() {
        let sat = SatId { constellation: Constellation::Gps, prn: 7 };
        let mut candidate = candidate_for_search_window_test(sat, 0.0, 3.0);
        candidate.hypothesis = AcqHypothesis::Ambiguous;
        candidate.explain_selection_reason = Some(
            "multipath_suspect: delayed secondary peak at sample 1440 (+240 samples, 60.000 chips)"
                .to_string(),
        );

        assert_eq!(selected_reason_for_candidate(&candidate), "multipath_suspect");
    }

    #[test]
    fn suppress_wrong_prn_correlations_rejects_ambiguous_cross_prn_candidates() {
        let dominant_sat = SatId { constellation: Constellation::Gps, prn: 7 };
        let wrong_sat = SatId { constellation: Constellation::Gps, prn: 8 };
        let mut sat_evaluations = vec![
            AcquisitionSatEvaluation {
                sat: dominant_sat,
                candidates: vec![AcqResult {
                    hypothesis: AcqHypothesis::Ambiguous,
                    peak_mean_ratio: 18.6,
                    peak_second_ratio: 1.25,
                    explain_selection_reason: Some("ambiguous_ratio_thresholds".to_string()),
                    ..candidate_for_search_window_test(dominant_sat, 0.0, 18.6)
                }],
                search_window_diagnostic: None,
            },
            AcquisitionSatEvaluation {
                sat: wrong_sat,
                candidates: vec![AcqResult {
                    hypothesis: AcqHypothesis::Ambiguous,
                    peak_mean_ratio: 3.1,
                    peak_second_ratio: 1.02,
                    explain_selection_reason: Some("ambiguous_ratio_thresholds".to_string()),
                    ..candidate_for_search_window_test(wrong_sat, 0.0, 3.1)
                }],
                search_window_diagnostic: None,
            },
        ];

        suppress_wrong_prn_correlations(&mut sat_evaluations);

        let suppressed = sat_evaluations[1].candidates.first().expect("suppressed candidate");
        assert_eq!(suppressed.hypothesis.to_string(), "rejected");
        assert_eq!(selected_reason_for_candidate(suppressed), "wrong_prn_correlation");
        assert_eq!(suppressed.score, 0.0);
    }

    #[test]
    fn parabolic_refinement_estimates_sub_bin_offset_from_neighbor_peaks() {
        let sat = SatId { constellation: Constellation::Gps, prn: 1 };
        let candidates = vec![
            candidate_for_search_window_test(sat, -250.0, 9.0),
            candidate_for_search_window_test(sat, 0.0, 16.0),
            candidate_for_search_window_test(sat, 250.0, 12.0),
        ];

        let refinement = estimate_acquisition_doppler_refinement(0.0, 0.0, &candidates, 250)
            .expect("refinement");

        assert_eq!(refinement.method, "parabolic_peak");
        assert_eq!(refinement.coarse_carrier_hz.0, 0.0);
        assert!((refinement.offset_bins - 0.136_363_636_4).abs() < 1.0e-6);
        assert!((refinement.offset_hz - 34.090_909_1).abs() < 1.0e-6);
    }

    #[test]
    fn parabolic_refinement_skips_search_edge_candidates() {
        let sat = SatId { constellation: Constellation::Gps, prn: 1 };
        let candidates = vec![
            candidate_for_search_window_test(sat, 0.0, 16.0),
            candidate_for_search_window_test(sat, 250.0, 12.0),
        ];

        let refinement = estimate_acquisition_doppler_refinement(0.0, 0.0, &candidates, 250);

        assert!(refinement.is_none());
    }

    #[test]
    fn quadratic_surface_refinement_estimates_joint_peak_offsets() {
        let surface = LocalAcquisitionLikelihoodSurface {
            doppler_cross_section: [7.84, 10.0, 9.16],
            code_phase_cross_section: [9.38, 10.0, 8.62],
            values: [[7.62, 7.84, 6.06], [9.38, 10.0, 8.62], [8.14, 9.16, 8.18]],
        };

        let (doppler_offset_bins, code_phase_offset_samples) =
            estimate_quadratic_surface_peak_offsets(&surface).expect("joint refinement");

        assert!((doppler_offset_bins - 0.2).abs() < 1.0e-6);
        assert!((code_phase_offset_samples + 0.15).abs() < 1.0e-6);
    }

    #[test]
    fn quadratic_surface_refinement_rejects_non_maximum_center() {
        let surface = LocalAcquisitionLikelihoodSurface {
            doppler_cross_section: [11.0, 10.0, 8.0],
            code_phase_cross_section: [8.0, 10.0, 7.0],
            values: [[7.0, 11.0, 6.5], [8.0, 10.0, 7.0], [6.0, 8.0, 5.5]],
        };

        assert!(estimate_quadratic_surface_peak_offsets(&surface).is_none());
    }

    #[test]
    fn log_likelihood_covariance_tightens_for_sharper_surfaces() {
        let broad = estimate_log_likelihood_covariance_2x2(
            &LocalAcquisitionLikelihoodSurface {
                doppler_cross_section: [14.0, 16.0, 15.0],
                code_phase_cross_section: [14.0, 16.0, 15.0],
                values: [[13.0, 14.0, 13.5], [14.0, 16.0, 15.0], [13.5, 15.0, 14.0]],
            },
            1.0,
            250.0,
            1.0,
        )
        .expect("broad covariance");
        let sharp = estimate_log_likelihood_covariance_2x2(
            &LocalAcquisitionLikelihoodSurface {
                doppler_cross_section: [9.0, 16.0, 12.0],
                code_phase_cross_section: [9.0, 16.0, 12.0],
                values: [[8.0, 9.0, 8.5], [9.0, 16.0, 12.0], [8.5, 12.0, 10.0]],
            },
            1.0,
            250.0,
            1.0,
        )
        .expect("sharp covariance");

        assert!(sharp.doppler_variance_hz2 < broad.doppler_variance_hz2, "{sharp:?} {broad:?}");
        assert!(
            sharp.code_phase_variance_samples2 < broad.code_phase_variance_samples2,
            "{sharp:?} {broad:?}"
        );
    }

    #[test]
    fn log_likelihood_covariance_reports_rate_variance_when_rate_axis_is_available() {
        let covariance = estimate_log_likelihood_covariance_3x3(
            &LocalAcquisitionLikelihoodVolume {
                values: [
                    [[3.0, 4.0, 3.0], [5.0, 7.0, 5.0], [3.0, 4.0, 3.0]],
                    [[4.0, 6.0, 4.0], [8.0, 16.0, 8.0], [4.0, 6.0, 4.0]],
                    [[3.0, 4.0, 3.0], [5.0, 7.0, 5.0], [3.0, 4.0, 3.0]],
                ],
            },
            1.0,
            250.0,
            1.0,
            5_000.0,
        )
        .expect("rate-aware covariance");

        assert!(covariance.doppler_variance_hz2 > 0.0);
        assert!(covariance.code_phase_variance_samples2 > 0.0);
        assert!(
            covariance.doppler_rate_variance_hz2_per_s2.is_some_and(|variance| variance > 0.0),
            "{covariance:?}"
        );
    }

    #[test]
    fn estimate_acquisition_uncertainty_skips_ambiguous_candidate() {
        let sat = SatId { constellation: Constellation::Gps, prn: 1 };
        let config = ReceiverPipelineConfig::default();
        let signal_model =
            AcquisitionSignalModel::for_sat_signal(sat, Some(SignalBand::L1), SignalCode::Ca, None)
                .expect("signal model lookup")
                .expect("gps l1ca signal model");
        let frame = noise_only_frame(
            config.sampling_freq_hz,
            signal_model.samples_per_code(config.sampling_freq_hz),
            0xA11CE,
        );
        let uncertainty = estimate_acquisition_uncertainty(
            &config,
            &frame,
            &signal_model,
            &AcqResult {
                hypothesis: AcqHypothesis::Ambiguous,
                ..candidate_for_search_window_test(sat, 0.0, 4.0)
            },
            1,
            1,
            250,
            0,
            250,
        );

        assert!(uncertainty.is_none());
    }

    #[test]
    fn code_phase_refinement_estimates_sub_sample_offset_from_neighbor_bins() {
        let profile = [12.0, 16.0, 15.0];
        let (offset_samples, left, center, right) =
            estimate_parabolic_code_phase_offset_samples(&profile, 1).expect("refinement");

        assert!((offset_samples - 0.3).abs() < 1.0e-6);
        assert_eq!(left, 12.0);
        assert_eq!(center, 16.0);
        assert_eq!(right, 15.0);
    }

    #[test]
    fn code_phase_refinement_wraps_negative_offsets_at_period_edges() {
        let offset_samples = -0.2;
        let refined_code_phase_samples =
            wrap_acquisition_code_phase_samples(0.0 + offset_samples, 4);

        assert!(offset_samples < 0.0);
        assert!((refined_code_phase_samples - 3.8).abs() < 1.0e-6);
    }

    #[test]
    fn correlation_metrics_report_primary_and_secondary_peak_indices() {
        let metrics = correlation_metrics(&[1.0, 4.0, 2.0, 3.5, 1.5]);

        assert_eq!(metrics.peak_idx, 1);
        assert_eq!(metrics.second_idx, 3);
        assert_eq!(metrics.peak, 4.0);
        assert_eq!(metrics.second, 3.5);
        assert!((metrics.mean - 2.4).abs() < 1.0e-6);
    }

    #[test]
    fn delayed_secondary_peak_diagnostic_ignores_main_lobe_neighbors() {
        let diagnostic =
            delayed_secondary_peak_diagnostic(&[1.0, 8.0, 7.6, 7.2, 1.4, 5.5, 1.2, 0.8], 1, 8, 4)
                .expect("delayed secondary peak");

        assert_eq!(diagnostic.secondary_code_phase_samples, 5);
        assert_eq!(diagnostic.delay_samples, 4);
    }

    #[test]
    fn delayed_secondary_peak_diagnostic_skips_short_wraparound_alias() {
        let diagnostic =
            delayed_secondary_peak_diagnostic(&[6.0, 1.0, 0.8, 0.9, 1.1, 1.3, 1.2, 5.4], 7, 8, 4);

        assert!(diagnostic.is_none());
    }

    #[test]
    fn multipath_candidate_reason_reports_delay_context() {
        let reason = multipath_candidate_reason(
            5.2,
            1.3,
            3.4,
            &DelayedSecondaryPeakDiagnostic {
                secondary_code_phase_samples: 1440,
                delay_samples: 240,
            },
            4092,
            1023,
            1.92,
        );

        assert!(reason.starts_with("multipath_suspect: delayed secondary peak"));
        assert!(reason.contains("+240 samples, 60.000 chips"));
        assert!(reason.contains("peak_second_ratio=1.300000"));
    }

    #[test]
    fn acquisition_rejects_unsupported_coherent_integration_lengths() {
        let config = ReceiverPipelineConfig::default();
        let sat = SatId { constellation: Constellation::Gps, prn: 1 };
        let frame =
            SamplesFrame::new(
                SampleTime { sample_index: 0, sample_rate_hz: config.sampling_freq_hz },
                Seconds(1.0 / config.sampling_freq_hz),
                (0..16)
                    .map(|idx| {
                        if idx % 2 == 0 {
                            Complex::new(1.0, 0.0)
                        } else {
                            Complex::new(-1.0, 0.0)
                        }
                    })
                    .collect(),
            );
        let acquisition = Acquisition::new(config, ReceiverRuntime::default());

        let run = acquisition.run_fft_topn_with_explain(&frame, &[sat], 1, 3, 1);

        let result = run.results.first().and_then(|rows| rows.first()).expect("result");
        let explain = run.explains.first().expect("explain");
        assert_eq!(result.hypothesis.to_string(), AcqHypothesis::Deferred.to_string());
        assert_eq!(explain.selected_reason, "unsupported_coherent_integration_ms");
        assert_eq!(
            result.explain_selection_reason.as_deref(),
            Some(
                "unsupported_coherent_integration_ms: acquisition coherent integration must be one of [1, 2, 5, 10, 20] ms but received 3 ms"
            )
        );
    }

    #[test]
    fn unsupported_coherent_integration_preserves_explicit_signal_code() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 10_230_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 10_230_000.0,
            code_length: 10_230,
            ..ReceiverPipelineConfig::default()
        };
        let sat = SatId { constellation: Constellation::Gps, prn: 7 };
        let frame =
            SamplesFrame::new(
                SampleTime { sample_index: 0, sample_rate_hz: config.sampling_freq_hz },
                Seconds(1.0 / config.sampling_freq_hz),
                (0..32)
                    .map(|idx| {
                        if idx % 2 == 0 {
                            Complex::new(1.0, 0.0)
                        } else {
                            Complex::new(-1.0, 0.0)
                        }
                    })
                    .collect(),
            );
        let acquisition = Acquisition::new(config, ReceiverRuntime::default());
        let request = AcqRequest {
            sat,
            glonass_frequency_channel: None,
            signal_band: SignalBand::L5,
            signal_code: SignalCode::L5Q,
            doppler_center_hz: 0.0,
            doppler_rate_center_hz_per_s: 0.0,
            doppler_rate_search_hz_per_s: 0,
            doppler_rate_step_hz_per_s: 250,
            expected_line_of_sight_doppler_hz: None,
            assistance_bounds: None,
            doppler_search_hz: 2_000,
            doppler_step_hz: 250,
            coherent_ms: 3,
            noncoherent: 1,
        };

        let run = acquisition.run_fft_topn_for_requests_with_explain(&frame, &[request], 1);

        let result = run.results[0].first().expect("deferred result");
        assert_eq!(result.signal_band, SignalBand::L5);
        assert_eq!(result.signal_code, SignalCode::L5Q);
        assert_eq!(result.hypothesis.to_string(), AcqHypothesis::Deferred.to_string());
        assert_eq!(run.explains[0].selected_reason, "unsupported_coherent_integration_ms");
    }

    #[test]
    fn code_fft_cache_reuses_local_code_across_integration_profiles() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            ..ReceiverPipelineConfig::default()
        };
        let sat = SatId { constellation: Constellation::Gps, prn: 11 };
        let samples_per_code = samples_per_code(
            config.sampling_freq_hz,
            config.code_freq_basis_hz,
            config.code_length,
        );
        let acquisition = Acquisition::new(config, ReceiverRuntime::default());
        let mut planner = FftPlanner::<f32>::new();
        let fft = planner.plan_fft_forward(samples_per_code);
        let signal_model = acquisition_signal_model_for_sat(
            &acquisition.config,
            sat,
            SignalBand::L1,
            SignalCode::Unknown,
            None,
        );
        let component =
            acquisition_component_plan_for_signal(sat, SignalBand::L1, SignalCode::Ca, 1);

        acquisition.code_fft(
            &signal_model,
            &component,
            sat,
            SignalCode::Ca,
            samples_per_code,
            1,
            1,
            fft.as_ref(),
        );
        let after_first_profile = acquisition.stats_snapshot();
        assert_eq!(after_first_profile.cache_misses, 1);
        assert_eq!(after_first_profile.cache_hits, 0);

        acquisition.code_fft(
            &signal_model,
            &component,
            sat,
            SignalCode::Ca,
            samples_per_code,
            1,
            4,
            fft.as_ref(),
        );
        let after_second_profile = acquisition.stats_snapshot();
        assert_eq!(after_second_profile.cache_misses, 1);
        assert_eq!(after_second_profile.cache_hits, 1);
        assert_eq!(after_second_profile.cache_miss_incompatible, 0);
    }

    #[test]
    fn code_fft_cache_separates_gps_l5_signal_codes() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 10_230_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 10_230_000.0,
            code_length: 10_230,
            ..ReceiverPipelineConfig::default()
        };
        let sat = SatId { constellation: Constellation::Gps, prn: 7 };
        let acquisition = Acquisition::new(config, ReceiverRuntime::default());
        let samples_per_code = 10_230;
        let mut planner = FftPlanner::<f32>::new();
        let fft = planner.plan_fft_forward(samples_per_code);
        let gps_l5_i = acquisition_signal_model_for_sat(
            &acquisition.config,
            sat,
            SignalBand::L5,
            SignalCode::L5I,
            None,
        );
        let gps_l5_q = acquisition_signal_model_for_sat(
            &acquisition.config,
            sat,
            SignalBand::L5,
            SignalCode::L5Q,
            None,
        );
        let gps_l5_i_component =
            acquisition_component_plan_for_signal(sat, SignalBand::L5, SignalCode::L5I, 1);
        let gps_l5_q_component =
            acquisition_component_plan_for_signal(sat, SignalBand::L5, SignalCode::L5Q, 1);

        acquisition.code_fft(
            &gps_l5_i,
            &gps_l5_i_component,
            sat,
            SignalCode::L5I,
            samples_per_code,
            1,
            1,
            fft.as_ref(),
        );
        let after_l5_i = acquisition.stats_snapshot();
        assert_eq!(after_l5_i.cache_misses, 1);
        assert_eq!(after_l5_i.cache_hits, 0);

        acquisition.code_fft(
            &gps_l5_q,
            &gps_l5_q_component,
            sat,
            SignalCode::L5Q,
            samples_per_code,
            1,
            1,
            fft.as_ref(),
        );
        let after_l5_q = acquisition.stats_snapshot();
        assert_eq!(after_l5_q.cache_misses, 2);
        assert_eq!(after_l5_q.cache_hits, 0);
        assert_eq!(after_l5_q.cache_miss_incompatible, 1);

        acquisition.code_fft(
            &gps_l5_q,
            &gps_l5_q_component,
            sat,
            SignalCode::L5Q,
            samples_per_code,
            1,
            1,
            fft.as_ref(),
        );
        let after_l5_q_reuse = acquisition.stats_snapshot();
        assert_eq!(after_l5_q_reuse.cache_misses, 2);
        assert_eq!(after_l5_q_reuse.cache_hits, 1);
    }

    #[test]
    fn run_fft_for_requests_preserves_explicit_gps_l5q_signal_code() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 10_230_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 10_230_000.0,
            code_length: 10_230,
            ..ReceiverPipelineConfig::default()
        };
        let frame =
            SamplesFrame::new(
                SampleTime { sample_index: 0, sample_rate_hz: config.sampling_freq_hz },
                Seconds(1.0 / config.sampling_freq_hz),
                (0..10_230)
                    .map(|idx| {
                        if idx % 2 == 0 {
                            Complex::new(1.0, 0.0)
                        } else {
                            Complex::new(-1.0, 0.0)
                        }
                    })
                    .collect(),
            );
        let acquisition = Acquisition::new(config, ReceiverRuntime::default());
        let request = AcqRequest {
            sat: SatId { constellation: Constellation::Gps, prn: 7 },
            glonass_frequency_channel: None,
            signal_band: SignalBand::L5,
            signal_code: SignalCode::L5Q,
            doppler_center_hz: 0.0,
            doppler_rate_center_hz_per_s: 0.0,
            doppler_rate_search_hz_per_s: 0,
            doppler_rate_step_hz_per_s: 250,
            expected_line_of_sight_doppler_hz: None,
            assistance_bounds: None,
            doppler_search_hz: 0,
            doppler_step_hz: 1,
            coherent_ms: 1,
            noncoherent: 1,
        };

        let run = acquisition.run_fft_topn_for_requests_with_explain(&frame, &[request], 1);

        let result = run.results[0].first().expect("acquisition result");
        assert_eq!(result.signal_band, SignalBand::L5);
        assert_eq!(result.signal_code, SignalCode::L5Q);
    }

    #[test]
    fn centered_doppler_requests_report_absolute_doppler_and_assumptions() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            acquisition_doppler_search_hz: 0,
            acquisition_doppler_step_hz: 1,
            ..ReceiverPipelineConfig::default()
        };
        let sat = SatId { constellation: Constellation::Gps, prn: 7 };
        let true_doppler_hz = 750.0;
        let frame = generate_l1_ca(
            &config,
            SyntheticSignalParams {
                sat,
                glonass_frequency_channel: None,
                signal_band: SignalBand::L1,
                signal_code: SignalCode::Unknown,
                doppler_hz: true_doppler_hz,
                code_phase_chips: 147.25,
                carrier_phase_rad: 0.0,
                cn0_db_hz: 58.0,
                navigation_data: false.into(),
            },
            0xC3E1_7EAD,
            0.020,
        );
        let request = AcqRequest {
            sat,
            glonass_frequency_channel: None,
            signal_band: SignalBand::L1,
            signal_code: SignalCode::Unknown,
            doppler_center_hz: true_doppler_hz,
            doppler_rate_center_hz_per_s: 0.0,
            doppler_rate_search_hz_per_s: 0,
            doppler_rate_step_hz_per_s: 250,
            expected_line_of_sight_doppler_hz: Some(true_doppler_hz),
            assistance_bounds: None,
            doppler_search_hz: 0,
            doppler_step_hz: 1,
            coherent_ms: 1,
            noncoherent: 1,
        };

        let result = Acquisition::new(config, ReceiverRuntime::default())
            .run_fft_for_requests(&frame, &[request])
            .remove(0);

        assert!(
            matches!(result.hypothesis, AcqHypothesis::Accepted | AcqHypothesis::Ambiguous),
            "{result:?}"
        );
        assert!((result.doppler_hz.0 - true_doppler_hz).abs() <= 1.0e-6, "{result:?}");
        let assumptions = result
            .assumptions
            .as_ref()
            .expect("centered acquisition result should preserve assumptions");
        assert_eq!(assumptions.doppler_center_hz, true_doppler_hz);
        assert_eq!(assumptions.expected_line_of_sight_doppler_hz, Some(true_doppler_hz));
    }

    #[test]
    fn assisted_bounds_reduce_reported_search_domain() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            acquisition_doppler_search_hz: 2_000,
            acquisition_doppler_step_hz: 250,
            ..ReceiverPipelineConfig::default()
        };
        let sat = SatId { constellation: Constellation::Gps, prn: 9 };
        let true_doppler_hz = 0.0;
        let code_phase_chips = 147.25;
        let frame = generate_l1_ca(
            &config,
            SyntheticSignalParams {
                sat,
                glonass_frequency_channel: None,
                signal_band: SignalBand::L1,
                signal_code: SignalCode::Unknown,
                doppler_hz: true_doppler_hz,
                code_phase_chips,
                carrier_phase_rad: 0.0,
                cn0_db_hz: 58.0,
                navigation_data: false.into(),
            },
            0xB01D_5E47,
            0.020,
        );
        let request = AcqRequest {
            sat,
            glonass_frequency_channel: None,
            signal_band: SignalBand::L1,
            signal_code: SignalCode::Unknown,
            doppler_center_hz: true_doppler_hz,
            doppler_rate_center_hz_per_s: 0.0,
            doppler_rate_search_hz_per_s: 0,
            doppler_rate_step_hz_per_s: 250,
            expected_line_of_sight_doppler_hz: Some(true_doppler_hz),
            assistance_bounds: Some(AcqAssistanceBounds {
                expected_code_phase_samples: code_phase_chips * config.sampling_freq_hz
                    / config.code_freq_basis_hz,
                time_uncertainty_s: 0.5e-6,
                position_uncertainty_m: 75.0,
                oscillator_uncertainty_hz: 120.0,
                approximate_velocity_uncertainty_mps: 6.0,
            }),
            doppler_search_hz: 2_000,
            doppler_step_hz: 250,
            coherent_ms: 1,
            noncoherent: 1,
        };

        let result = Acquisition::new(config, ReceiverRuntime::default())
            .run_fft_for_requests(&frame, &[request])
            .remove(0);

        assert!(
            matches!(result.hypothesis, AcqHypothesis::Accepted | AcqHypothesis::Ambiguous),
            "{result:?}"
        );
        let assumptions = result
            .assumptions
            .as_ref()
            .expect("assisted acquisition result should preserve assumptions");
        assert_eq!(assumptions.assistance_bounds, request.assistance_bounds);
        assert!(assumptions.doppler_search_hz < request.doppler_search_hz, "{assumptions:?}");
        assert!(
            assumptions.code_phase_search_bins < assumptions.samples_per_code,
            "{assumptions:?}"
        );
        assert_eq!(assumptions.code_phase_search_mode, "assisted_bounds");
    }

    #[test]
    fn wrong_assisted_code_phase_bounds_retry_full_search() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            acquisition_doppler_search_hz: 2_000,
            acquisition_doppler_step_hz: 250,
            ..ReceiverPipelineConfig::default()
        };
        let sat = SatId { constellation: Constellation::Gps, prn: 10 };
        let code_phase_chips = 147.25;
        let frame = generate_l1_ca(
            &config,
            SyntheticSignalParams {
                sat,
                glonass_frequency_channel: None,
                signal_band: SignalBand::L1,
                signal_code: SignalCode::Unknown,
                doppler_hz: 0.0,
                code_phase_chips,
                carrier_phase_rad: 0.0,
                cn0_db_hz: 58.0,
                navigation_data: false.into(),
            },
            0xFA11_BAC4,
            0.020,
        );
        let request = AcqRequest {
            sat,
            glonass_frequency_channel: None,
            signal_band: SignalBand::L1,
            signal_code: SignalCode::Unknown,
            doppler_center_hz: 0.0,
            doppler_rate_center_hz_per_s: 0.0,
            doppler_rate_search_hz_per_s: 0,
            doppler_rate_step_hz_per_s: 250,
            expected_line_of_sight_doppler_hz: Some(0.0),
            assistance_bounds: Some(AcqAssistanceBounds {
                expected_code_phase_samples: 32.0,
                time_uncertainty_s: 0.0,
                position_uncertainty_m: 0.0,
                oscillator_uncertainty_hz: 120.0,
                approximate_velocity_uncertainty_mps: 6.0,
            }),
            doppler_search_hz: 2_000,
            doppler_step_hz: 250,
            coherent_ms: 1,
            noncoherent: 1,
        };

        let result = Acquisition::new(config, ReceiverRuntime::default())
            .run_fft_for_requests(&frame, &[request])
            .remove(0);

        assert!(
            matches!(result.hypothesis, AcqHypothesis::Accepted | AcqHypothesis::Ambiguous),
            "{result:?}"
        );
        let assumptions = result
            .assumptions
            .as_ref()
            .expect("fallback acquisition result should preserve assumptions");
        assert_eq!(assumptions.code_phase_search_mode, "full_code");
        assert_eq!(assumptions.assistance_bounds, None);
        assert!(
            result
                .explain_selection_reason
                .as_deref()
                .is_some_and(|reason| reason.contains("assistance_bounds_fallback")),
            "{result:?}"
        );
    }

    #[test]
    fn run_fft_for_requests_preserves_explicit_galileo_e5b_signal_code() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 10_230_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 10_230_000.0,
            code_length: 10_230,
            ..ReceiverPipelineConfig::default()
        };
        let frame =
            SamplesFrame::new(
                SampleTime { sample_index: 0, sample_rate_hz: config.sampling_freq_hz },
                Seconds(1.0 / config.sampling_freq_hz),
                (0..10_230)
                    .map(|idx| {
                        if idx % 2 == 0 {
                            Complex::new(1.0, 0.0)
                        } else {
                            Complex::new(-1.0, 0.0)
                        }
                    })
                    .collect(),
            );
        let acquisition = Acquisition::new(config, ReceiverRuntime::default());
        let request = AcqRequest {
            sat: SatId { constellation: Constellation::Galileo, prn: 11 },
            glonass_frequency_channel: None,
            signal_band: SignalBand::E5,
            signal_code: SignalCode::E5b,
            doppler_center_hz: 0.0,
            doppler_rate_center_hz_per_s: 0.0,
            doppler_rate_search_hz_per_s: 0,
            doppler_rate_step_hz_per_s: 250,
            expected_line_of_sight_doppler_hz: None,
            assistance_bounds: None,
            doppler_search_hz: 0,
            doppler_step_hz: 1,
            coherent_ms: 1,
            noncoherent: 1,
        };

        let run = acquisition.run_fft_topn_for_requests_with_explain(&frame, &[request], 1);

        let result = run.results[0].first().expect("acquisition result");
        assert_eq!(result.signal_band, SignalBand::E5);
        assert_eq!(result.signal_code, SignalCode::E5b);
    }

    #[test]
    fn run_fft_for_requests_keeps_galileo_e5_cache_entries_separate() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 10_230_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 10_230_000.0,
            code_length: 10_230,
            ..ReceiverPipelineConfig::default()
        };
        let frame =
            SamplesFrame::new(
                SampleTime { sample_index: 0, sample_rate_hz: config.sampling_freq_hz },
                Seconds(1.0 / config.sampling_freq_hz),
                (0..10_230)
                    .map(|idx| {
                        if idx % 2 == 0 {
                            Complex::new(1.0, 0.0)
                        } else {
                            Complex::new(-1.0, 0.0)
                        }
                    })
                    .collect(),
            );
        let acquisition = Acquisition::new(config, ReceiverRuntime::default());
        let e5a_request = AcqRequest {
            sat: SatId { constellation: Constellation::Galileo, prn: 11 },
            glonass_frequency_channel: None,
            signal_band: SignalBand::E5,
            signal_code: SignalCode::E5a,
            doppler_center_hz: 0.0,
            doppler_rate_center_hz_per_s: 0.0,
            doppler_rate_search_hz_per_s: 0,
            doppler_rate_step_hz_per_s: 250,
            expected_line_of_sight_doppler_hz: None,
            assistance_bounds: None,
            doppler_search_hz: 0,
            doppler_step_hz: 1,
            coherent_ms: 1,
            noncoherent: 1,
        };
        let e5b_request = AcqRequest { signal_code: SignalCode::E5b, ..e5a_request };

        let e5a_run = acquisition.run_fft_topn_for_requests_with_explain(&frame, &[e5a_request], 1);
        let after_e5a = acquisition.stats_snapshot();
        assert_eq!(e5a_run.results[0][0].signal_code, SignalCode::E5a);
        assert_eq!(after_e5a.cache_misses, 2);
        assert_eq!(after_e5a.cache_hits, 0);
        assert_eq!(after_e5a.cache_miss_incompatible, 1);

        let e5b_run = acquisition.run_fft_topn_for_requests_with_explain(&frame, &[e5b_request], 1);
        let after_e5b = acquisition.stats_snapshot();
        assert_eq!(e5b_run.results[0][0].signal_code, SignalCode::E5b);
        assert_eq!(after_e5b.cache_misses, 4);
        assert_eq!(after_e5b.cache_hits, 0);
        assert_eq!(after_e5b.cache_miss_incompatible, 3);
    }

    #[test]
    fn run_fft_for_galileo_e5a_records_component_combination_provenance() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 10_230_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 10_230_000.0,
            code_length: 10_230,
            ..ReceiverPipelineConfig::default()
        };
        let frame = alternating_frame(config.sampling_freq_hz, 10_230);
        let acquisition = Acquisition::new(config, ReceiverRuntime::default());
        let request = AcqRequest {
            sat: SatId { constellation: Constellation::Galileo, prn: 11 },
            glonass_frequency_channel: None,
            signal_band: SignalBand::E5,
            signal_code: SignalCode::E5a,
            doppler_center_hz: 0.0,
            doppler_rate_center_hz_per_s: 0.0,
            doppler_rate_search_hz_per_s: 0,
            doppler_rate_step_hz_per_s: 250,
            expected_line_of_sight_doppler_hz: None,
            assistance_bounds: None,
            doppler_search_hz: 0,
            doppler_step_hz: 1,
            coherent_ms: 1,
            noncoherent: 1,
        };

        let run = acquisition.run_fft_topn_for_requests_with_explain(&frame, &[request], 4);
        let candidates = &run.results[0];

        assert_eq!(candidates.len(), 4);
        assert!(candidates.iter().all(|candidate| candidate.signal_code == SignalCode::E5a));
        assert!(candidates.iter().any(|candidate| {
            candidate.component_provenance().is_some_and(|provenance| {
                provenance.combination_mode == AcqComponentCombinationMode::SingleComponent
                    && provenance
                        .components
                        .iter()
                        .map(|component| component.role)
                        .collect::<Vec<_>>()
                        == vec![SignalComponentRole::Data]
            })
        }));
        assert!(candidates.iter().any(|candidate| {
            candidate.component_provenance().is_some_and(|provenance| {
                provenance.combination_mode == AcqComponentCombinationMode::SingleComponent
                    && provenance
                        .components
                        .iter()
                        .map(|component| component.role)
                        .collect::<Vec<_>>()
                        == vec![SignalComponentRole::Pilot]
            })
        }));
        assert!(candidates.iter().any(|candidate| {
            candidate.component_provenance().is_some_and(|provenance| {
                provenance.combination_mode == AcqComponentCombinationMode::NoncoherentComponentSum
                    && provenance
                        .components
                        .iter()
                        .map(|component| component.role)
                        .collect::<Vec<_>>()
                        == vec![SignalComponentRole::Data, SignalComponentRole::Pilot]
            })
        }));
        assert!(candidates.iter().any(|candidate| {
            candidate.component_provenance().is_some_and(|provenance| {
                provenance.combination_mode == AcqComponentCombinationMode::CoherentComponentSum
                    && provenance
                        .components
                        .iter()
                        .map(|component| component.role)
                        .collect::<Vec<_>>()
                        == vec![SignalComponentRole::Data, SignalComponentRole::Pilot]
            })
        }));
    }

    #[test]
    fn run_fft_for_galileo_e5a_skips_coherent_component_sum_beyond_one_millisecond() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 10_230_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 10_230_000.0,
            code_length: 10_230,
            ..ReceiverPipelineConfig::default()
        };
        let frame = alternating_frame(config.sampling_freq_hz, 20_460);
        let acquisition = Acquisition::new(config, ReceiverRuntime::default());
        let request = AcqRequest {
            sat: SatId { constellation: Constellation::Galileo, prn: 11 },
            glonass_frequency_channel: None,
            signal_band: SignalBand::E5,
            signal_code: SignalCode::E5a,
            doppler_center_hz: 0.0,
            doppler_rate_center_hz_per_s: 0.0,
            doppler_rate_search_hz_per_s: 0,
            doppler_rate_step_hz_per_s: 250,
            expected_line_of_sight_doppler_hz: None,
            assistance_bounds: None,
            doppler_search_hz: 0,
            doppler_step_hz: 1,
            coherent_ms: 2,
            noncoherent: 1,
        };

        let run = acquisition.run_fft_topn_for_requests_with_explain(&frame, &[request], 4);
        let candidates = &run.results[0];

        assert_eq!(candidates.len(), 3);
        assert!(candidates.iter().all(|candidate| {
            candidate.component_provenance().is_some_and(|provenance| {
                provenance.combination_mode != AcqComponentCombinationMode::CoherentComponentSum
            })
        }));
    }

    #[test]
    fn run_fft_for_gps_l5q_records_single_pilot_component_provenance() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 10_230_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 10_230_000.0,
            code_length: 10_230,
            ..ReceiverPipelineConfig::default()
        };
        let frame = alternating_frame(config.sampling_freq_hz, 10_230);
        let acquisition = Acquisition::new(config, ReceiverRuntime::default());
        let request = AcqRequest {
            sat: SatId { constellation: Constellation::Gps, prn: 7 },
            glonass_frequency_channel: None,
            signal_band: SignalBand::L5,
            signal_code: SignalCode::L5Q,
            doppler_center_hz: 0.0,
            doppler_rate_center_hz_per_s: 0.0,
            doppler_rate_search_hz_per_s: 0,
            doppler_rate_step_hz_per_s: 250,
            expected_line_of_sight_doppler_hz: None,
            assistance_bounds: None,
            doppler_search_hz: 0,
            doppler_step_hz: 1,
            coherent_ms: 1,
            noncoherent: 1,
        };

        let run = acquisition.run_fft_topn_for_requests_with_explain(&frame, &[request], 1);
        let provenance = run.results[0][0].component_provenance().expect("component provenance");

        assert_eq!(provenance.combination_mode, AcqComponentCombinationMode::SingleComponent);
        assert_eq!(
            provenance.components.iter().map(|component| component.role).collect::<Vec<_>>(),
            vec![SignalComponentRole::Pilot]
        );
    }

    #[test]
    fn acquisition_stability_keys_are_sorted() {
        let sat = SatId { constellation: bijux_gnss_core::api::Constellation::Gps, prn: 1 };
        let mut rows = vec![
            AcqResult {
                sat,
                signal_band: SignalBand::L1,
                signal_code: SignalCode::Unknown,
                glonass_frequency_channel: None,
                source_time: ReceiverSampleTrace::default(),
                candidate_rank: 1,
                is_primary_candidate: true,
                doppler_hz: Hertz(100.0),
                doppler_rate_hz_per_s: 0.0,
                carrier_hz: Hertz(100.0),
                code_phase_samples: 10,
                peak: 10.0,
                second_peak: 2.0,
                mean: 1.0,
                peak_mean_ratio: 10.0,
                peak_second_ratio: 5.0,
                cn0_proxy: 10.0,
                score: 2.0,
                hypothesis: AcqHypothesis::Accepted,
                assumptions: None,
                evidence: Vec::new(),
                threshold_provenance: None,
                explain_selection_reason: None,
                doppler_refinement: None,
                code_phase_refinement: None,
                signal_delay_alignment: None,
                uncertainty: None,
            },
            AcqResult {
                sat,
                signal_band: SignalBand::L1,
                signal_code: SignalCode::Unknown,
                glonass_frequency_channel: None,
                source_time: ReceiverSampleTrace::default(),
                candidate_rank: 1,
                is_primary_candidate: true,
                doppler_hz: Hertz(50.0),
                doppler_rate_hz_per_s: 0.0,
                carrier_hz: Hertz(50.0),
                code_phase_samples: 20,
                peak: 10.0,
                second_peak: 2.0,
                mean: 1.0,
                peak_mean_ratio: 10.0,
                peak_second_ratio: 5.0,
                cn0_proxy: 10.0,
                score: 2.0,
                hypothesis: AcqHypothesis::Accepted,
                assumptions: None,
                evidence: Vec::new(),
                threshold_provenance: None,
                explain_selection_reason: None,
                doppler_refinement: None,
                code_phase_refinement: None,
                signal_delay_alignment: None,
                uncertainty: None,
            },
        ];
        rows.sort_by(|a, b| {
            let primary = b
                .peak_mean_ratio
                .partial_cmp(&a.peak_mean_ratio)
                .unwrap_or(std::cmp::Ordering::Equal);
            if primary == std::cmp::Ordering::Equal {
                return acq_result_stability_key(a).cmp(&acq_result_stability_key(b));
            }
            primary
        });
        let keys = stable_acq_result_keys(&rows);
        assert!(keys.windows(2).all(|window| window[0] <= window[1]));
    }

    #[test]
    fn search_window_diagnostic_detects_rising_upper_edge_peak() {
        let sat = SatId { constellation: bijux_gnss_core::api::Constellation::Gps, prn: 1 };
        let diagnostic = signal_outside_search_range(
            &[
                candidate_for_search_window_test(sat, -1_500.0, 1.5),
                candidate_for_search_window_test(sat, -1_250.0, 1.7),
                candidate_for_search_window_test(sat, 1_250.0, 2.4),
                candidate_for_search_window_test(sat, 1_500.0, 2.9),
            ],
            0.0,
            1_500,
            250,
            2.5,
        )
        .expect("search-window diagnostic");

        assert_eq!(diagnostic.dimension, SearchWindowDimension::Doppler);
        assert_eq!(diagnostic.edge, SearchWindowEdge::Upper);
        assert_eq!(diagnostic.best_axis_value, 1_500.0);
        assert_eq!(diagnostic.interior_axis_value, 1_250.0);
    }

    #[test]
    fn search_window_diagnostic_ignores_interior_peak() {
        let sat = SatId { constellation: bijux_gnss_core::api::Constellation::Gps, prn: 1 };
        let diagnostic = signal_outside_search_range(
            &[
                candidate_for_search_window_test(sat, -1_500.0, 1.4),
                candidate_for_search_window_test(sat, -1_250.0, 1.8),
                candidate_for_search_window_test(sat, 0.0, 3.2),
                candidate_for_search_window_test(sat, 1_500.0, 2.7),
            ],
            0.0,
            1_500,
            250,
            2.5,
        );

        assert!(diagnostic.is_none());
    }

    #[test]
    fn search_window_diagnostic_prefers_edge_signal_over_interior_ambiguity() {
        let sat = SatId { constellation: bijux_gnss_core::api::Constellation::Gps, prn: 1 };
        let diagnostic = signal_outside_search_range(
            &[
                candidate_for_search_window_test(sat, -1_500.0, 3.5),
                candidate_for_search_window_test(sat, -1_250.0, 1.6),
                candidate_for_search_window_test(sat, 0.0, 3.6),
                candidate_for_search_window_test(sat, 1_500.0, 1.9),
            ],
            0.0,
            1_500,
            250,
            2.5,
        )
        .expect("search-window diagnostic");

        assert_eq!(diagnostic.dimension, SearchWindowDimension::Doppler);
        assert_eq!(diagnostic.edge, SearchWindowEdge::Lower);
        assert_eq!(diagnostic.best_axis_value, -1_500.0);
        assert_eq!(diagnostic.interior_axis_value, -1_250.0);
    }

    #[test]
    fn search_window_diagnostic_ignores_weak_edge_peak() {
        let sat = SatId { constellation: bijux_gnss_core::api::Constellation::Gps, prn: 1 };
        let diagnostic = signal_outside_search_range(
            &[
                candidate_for_search_window_test(sat, -1_500.0, 1.1),
                candidate_for_search_window_test(sat, -1_250.0, 1.0),
                candidate_for_search_window_test(sat, 1_250.0, 1.8),
                candidate_for_search_window_test(sat, 1_500.0, 2.0),
            ],
            0.0,
            1_500,
            250,
            2.5,
        );

        assert!(diagnostic.is_none());
    }

    #[test]
    fn search_window_diagnostic_respects_nonzero_intermediate_frequency() {
        let sat = SatId { constellation: bijux_gnss_core::api::Constellation::Gps, prn: 1 };
        let intermediate_freq_hz = 1_250_000.0;
        let diagnostic = signal_outside_search_range(
            &[
                candidate_for_search_window_test(
                    sat,
                    carrier_hz_from_doppler_hz(intermediate_freq_hz, -1_500.0),
                    1.5,
                ),
                candidate_for_search_window_test(
                    sat,
                    carrier_hz_from_doppler_hz(intermediate_freq_hz, -1_250.0),
                    1.7,
                ),
                candidate_for_search_window_test(
                    sat,
                    carrier_hz_from_doppler_hz(intermediate_freq_hz, 1_250.0),
                    2.4,
                ),
                candidate_for_search_window_test(
                    sat,
                    carrier_hz_from_doppler_hz(intermediate_freq_hz, 1_500.0),
                    2.9,
                ),
            ],
            intermediate_freq_hz,
            1_500,
            250,
            2.5,
        )
        .expect("search-window diagnostic");

        assert_eq!(diagnostic.dimension, SearchWindowDimension::Doppler);
        assert_eq!(diagnostic.edge, SearchWindowEdge::Upper);
        assert_eq!(
            diagnostic.best_axis_value,
            carrier_hz_from_doppler_hz(intermediate_freq_hz, 1_500.0)
        );
        assert_eq!(
            diagnostic.interior_axis_value,
            carrier_hz_from_doppler_hz(intermediate_freq_hz, 1_250.0)
        );
    }

    #[test]
    fn run_fft_returns_deferred_result_for_each_satellite_when_frame_is_too_short() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            acquisition_integration_ms: 2,
            acquisition_noncoherent: 1,
            ..ReceiverPipelineConfig::default()
        };
        let samples_per_code = samples_per_code(
            config.sampling_freq_hz,
            config.code_freq_basis_hz,
            config.code_length,
        );
        let frame =
            SamplesFrame::new(
                SampleTime { sample_index: 0, sample_rate_hz: config.sampling_freq_hz },
                Seconds(1.0 / config.sampling_freq_hz),
                (0..samples_per_code)
                    .map(|idx| {
                        if idx % 2 == 0 {
                            Complex::new(1.0, 0.0)
                        } else {
                            Complex::new(-1.0, 0.0)
                        }
                    })
                    .collect(),
            );
        let sats = vec![
            SatId { constellation: Constellation::Gps, prn: 3 },
            SatId { constellation: Constellation::Gps, prn: 7 },
        ];
        let acquisition = Acquisition::new(config, ReceiverRuntime::default());

        let run = acquisition.run_fft_topn_with_explain(&frame, &sats, 1, 2, 1);

        assert_eq!(run.results.len(), sats.len());
        assert_eq!(run.explains.len(), sats.len());
        for (idx, sat) in sats.iter().enumerate() {
            let result = run.results[idx].first().expect("placeholder result");
            assert_eq!(result.sat, *sat);
            assert_eq!(result.hypothesis.to_string(), AcqHypothesis::Deferred.to_string());
            assert_eq!(result.code_phase_samples, 0);
            assert_eq!(result.peak_mean_ratio, 0.0);
            assert_eq!(
                result.explain_selection_reason.as_deref(),
                Some("insufficient_frame: acquisition requires 8184 samples but received 4092"),
            );

            let explain = &run.explains[idx];
            assert_eq!(explain.sat, *sat);
            assert_eq!(explain.selected_reason, "insufficient_frame");
            assert_eq!(explain.candidate_count, 1);
        }
    }

    #[test]
    fn insufficient_frame_preserves_explicit_signal_code() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 10_230_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 10_230_000.0,
            code_length: 10_230,
            ..ReceiverPipelineConfig::default()
        };
        let frame =
            SamplesFrame::new(
                SampleTime { sample_index: 0, sample_rate_hz: config.sampling_freq_hz },
                Seconds(1.0 / config.sampling_freq_hz),
                (0..1023)
                    .map(|idx| {
                        if idx % 2 == 0 {
                            Complex::new(1.0, 0.0)
                        } else {
                            Complex::new(-1.0, 0.0)
                        }
                    })
                    .collect(),
            );
        let acquisition = Acquisition::new(config, ReceiverRuntime::default());
        let request = AcqRequest {
            sat: SatId { constellation: Constellation::Galileo, prn: 11 },
            glonass_frequency_channel: None,
            signal_band: SignalBand::E5,
            signal_code: SignalCode::E5b,
            doppler_center_hz: 0.0,
            doppler_rate_center_hz_per_s: 0.0,
            doppler_rate_search_hz_per_s: 0,
            doppler_rate_step_hz_per_s: 250,
            expected_line_of_sight_doppler_hz: None,
            assistance_bounds: None,
            doppler_search_hz: 2_000,
            doppler_step_hz: 250,
            coherent_ms: 1,
            noncoherent: 1,
        };

        let run = acquisition.run_fft_topn_for_requests_with_explain(&frame, &[request], 1);

        let result = run.results[0].first().expect("deferred result");
        assert_eq!(result.signal_band, SignalBand::E5);
        assert_eq!(result.signal_code, SignalCode::E5b);
        assert_eq!(result.hypothesis.to_string(), AcqHypothesis::Deferred.to_string());
        assert_eq!(run.explains[0].selected_reason, "insufficient_frame");
    }

    #[test]
    fn galileo_e1_cboc_profile_reports_side_peak_geometry() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 4092,
            acquisition_integration_ms: 20,
            acquisition_noncoherent: 1,
            ..ReceiverPipelineConfig::default()
        };
        let sat = SatId { constellation: Constellation::Galileo, prn: 11 };
        let frame = generate_l1_ca(
            &config,
            SyntheticSignalParams {
                sat,
                glonass_frequency_channel: None,
                signal_band: SignalBand::E1,
                signal_code: SignalCode::E1B,
                doppler_hz: 0.0,
                code_phase_chips: 321.0,
                carrier_phase_rad: 0.25,
                cn0_db_hz: 60.0,
                navigation_data: false.into(),
            },
            0x6A11_E175,
            0.020,
        );
        let signal_model =
            acquisition_signal_model_for_sat(&config, sat, SignalBand::E1, SignalCode::E1B, None);
        let profile = measure_code_phase_profile(
            &config,
            &signal_model,
            &frame,
            sat,
            signal_model.search_center_hz(config.intermediate_freq_hz),
            0.0,
            config.acquisition_integration_ms,
            config.acquisition_noncoherent,
        )
        .expect("Galileo E1 correlation profile");
        let metrics = correlation_metrics(&profile);
        let diagnostic = delayed_secondary_peak_diagnostic(
            &profile,
            metrics.peak_idx,
            signal_model.samples_per_code(config.sampling_freq_hz),
            signal_model.code_length,
        );
        let delayed_peak = diagnostic
            .as_ref()
            .map(|value| profile[value.secondary_code_phase_samples])
            .unwrap_or_default();
        let delayed_peak_mean_ratio = delayed_peak / (metrics.mean + 1.0e-6);

        assert!(metrics.peak > delayed_peak);
        assert!(diagnostic.is_some(), "{metrics:?}");
        assert!(
            delayed_peak_mean_ratio > config.acquisition_peak_mean_threshold,
            "delayed local peak should still look acquisition-worthy: {delayed_peak_mean_ratio}"
        );
    }

    #[test]
    fn galileo_e1_strategy_candidates_prefer_pilot_cboc_acquisition() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 4092,
            acquisition_doppler_search_hz: 500,
            acquisition_doppler_step_hz: 500,
            acquisition_integration_ms: 20,
            acquisition_noncoherent: 1,
            ..ReceiverPipelineConfig::default()
        };
        let sat = SatId { constellation: Constellation::Galileo, prn: 11 };
        let frame = generate_l1_ca(
            &config,
            SyntheticSignalParams {
                sat,
                glonass_frequency_channel: None,
                signal_band: SignalBand::E1,
                signal_code: SignalCode::E1B,
                doppler_hz: 0.0,
                code_phase_chips: 321.0,
                carrier_phase_rad: 0.25,
                cn0_db_hz: 60.0,
                navigation_data: false.into(),
            },
            0x6A11_E176,
            0.020,
        );
        let request = AcqRequest {
            sat,
            glonass_frequency_channel: None,
            signal_band: SignalBand::E1,
            signal_code: SignalCode::E1B,
            doppler_center_hz: 0.0,
            doppler_rate_center_hz_per_s: 0.0,
            doppler_rate_search_hz_per_s: 0,
            doppler_rate_step_hz_per_s: 250,
            expected_line_of_sight_doppler_hz: None,
            assistance_bounds: None,
            doppler_search_hz: config.acquisition_doppler_search_hz,
            doppler_step_hz: config.acquisition_doppler_step_hz,
            coherent_ms: config.acquisition_integration_ms,
            noncoherent: config.acquisition_noncoherent,
        };
        let run = Acquisition::new(config, ReceiverRuntime::default())
            .run_fft_topn_for_requests_with_explain(&frame, &[request], 4);
        let selected = run.results[0].first().expect("selected Galileo E1 candidate");
        let provenance =
            selected.component_provenance().expect("selected Galileo E1 component provenance");

        assert_eq!(selected.hypothesis.to_string(), "accepted", "{run:?}");
        assert_eq!(run.explains[0].selected_reason, "accepted_by_ratio_thresholds", "{run:?}");
        assert_eq!(provenance.combination_mode, AcqComponentCombinationMode::SingleComponent);
        assert_eq!(
            provenance.components.iter().map(|component| component.role).collect::<Vec<_>>(),
            vec![SignalComponentRole::Pilot]
        );
        assert_eq!(provenance.components[0].secondary_code_phase_periods, Some(0), "{run:?}");
        let code_phase_uncertainty_samples = selected
            .uncertainty
            .as_ref()
            .expect("Galileo E1 accepted candidate uncertainty")
            .code_phase_samples;
        assert!(
            code_phase_uncertainty_samples > 0.0 && code_phase_uncertainty_samples < 1.0,
            "expected sub-chip Galileo E1 code-phase uncertainty, got {code_phase_uncertainty_samples}: {run:?}",
        );
    }

    #[test]
    fn competing_candidate_ratio_ignores_strategy_variants_of_same_hypothesis() {
        let sat = SatId { constellation: Constellation::Galileo, prn: 11 };
        let best = candidate_for_search_window_test(sat, 0.0, 12.0);
        let same_hypothesis_variant = candidate_for_search_window_test(sat, 0.0, 10.0);
        let competing = candidate_for_search_window_test(sat, 500.0, 4.0);

        assert!(
            (competing_candidate_ratio(&[best, same_hypothesis_variant, competing]) - 3.0).abs()
                <= f32::EPSILON
        );
    }

    #[test]
    fn galileo_e1_signal_only_streaming_frame_returns_explicit_ambiguity() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 4092,
            acquisition_doppler_search_hz: 500,
            acquisition_doppler_step_hz: 500,
            acquisition_integration_ms: 20,
            acquisition_noncoherent: 1,
            ..ReceiverPipelineConfig::default()
        };
        let sat = SatId { constellation: Constellation::Galileo, prn: 11 };
        let scenario = SyntheticScenario {
            sample_rate_hz: config.sampling_freq_hz,
            intermediate_freq_hz: config.intermediate_freq_hz,
            receiver_clock_frequency_bias_hz: 0.0,
            duration_s: 0.080,
            seed: 0x6A11_E100,
            satellites: vec![SyntheticSignalParams {
                sat,
                glonass_frequency_channel: None,
                signal_band: SignalBand::L1,
                signal_code: SignalCode::Unknown,
                doppler_hz: 0.0,
                code_phase_chips: 321.0,
                carrier_phase_rad: 0.25,
                cn0_db_hz: 60.0,
                navigation_data: false.into(),
            }],
            ephemerides: Vec::new(),
            id: "galileo-e1-streaming-candidate-shape".to_string(),
        };
        let mut source = SyntheticSignalSource::new_signal_only(&config, &scenario);
        let frame = source
            .next_frame(81_840)
            .expect("signal-only frame")
            .expect("signal-only acquisition frame");
        let request = AcqRequest {
            sat,
            glonass_frequency_channel: None,
            signal_band: SignalBand::E1,
            signal_code: SignalCode::E1B,
            doppler_center_hz: 0.0,
            doppler_rate_center_hz_per_s: 0.0,
            doppler_rate_search_hz_per_s: 0,
            doppler_rate_step_hz_per_s: 250,
            expected_line_of_sight_doppler_hz: None,
            assistance_bounds: None,
            doppler_search_hz: config.acquisition_doppler_search_hz,
            doppler_step_hz: config.acquisition_doppler_step_hz,
            coherent_ms: config.acquisition_integration_ms,
            noncoherent: config.acquisition_noncoherent,
        };
        let run = Acquisition::new(config, ReceiverRuntime::default())
            .run_fft_topn_for_requests_with_explain(&frame, &[request], 4);
        let selected = run.results[0].first().expect("selected Galileo E1 signal-only candidate");
        let provenance = selected
            .component_provenance()
            .expect("selected Galileo E1 signal-only component provenance");

        assert_eq!(selected.hypothesis.to_string(), "ambiguous", "{run:?}");
        assert_eq!(run.explains[0].selected_reason, "ambiguous_ratio_thresholds", "{run:?}");
        assert_eq!(provenance.combination_mode, AcqComponentCombinationMode::SingleComponent);
        assert_eq!(
            provenance.components.iter().map(|component| component.role).collect::<Vec<_>>(),
            vec![SignalComponentRole::Pilot]
        );
        assert!(selected.uncertainty.is_none(), "{run:?}");
    }

    #[test]
    fn related_signal_follow_up_reruns_same_satellite_cross_band_request() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 10_230_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 10_230_000.0,
            code_length: 10_230,
            acquisition_doppler_search_hz: 2_000,
            acquisition_doppler_step_hz: 250,
            acquisition_integration_ms: 1,
            acquisition_noncoherent: 1,
            ..ReceiverPipelineConfig::default()
        };
        let sat = SatId { constellation: Constellation::Gps, prn: 17 };
        let l1_doppler_hz = -750.0;
        let l5_doppler_hz =
            shared_path_doppler_hz(l1_doppler_hz, signal_spec_gps_l1_ca(), signal_spec_gps_l5_i())
                .expect("same-satellite carrier scaling");
        let scenario = SyntheticScenario {
            sample_rate_hz: config.sampling_freq_hz,
            intermediate_freq_hz: config.intermediate_freq_hz,
            receiver_clock_frequency_bias_hz: 0.0,
            duration_s: 0.030,
            seed: 0x2810_0001,
            satellites: vec![
                SyntheticSignalParams {
                    sat,
                    glonass_frequency_channel: None,
                    signal_band: SignalBand::L1,
                    signal_code: SignalCode::Ca,
                    doppler_hz: l1_doppler_hz,
                    code_phase_chips: 32.1,
                    carrier_phase_rad: 0.25,
                    cn0_db_hz: 56.0,
                    navigation_data: false.into(),
                },
                SyntheticSignalParams {
                    sat,
                    glonass_frequency_channel: None,
                    signal_band: SignalBand::L5,
                    signal_code: SignalCode::L5I,
                    doppler_hz: l5_doppler_hz,
                    code_phase_chips: 321.0,
                    carrier_phase_rad: 0.25,
                    cn0_db_hz: 31.5,
                    navigation_data: false.into(),
                },
            ],
            ephemerides: Vec::new(),
            id: "related-signal-follow-up-same-satellite".to_string(),
        };
        let frame = signal_only_frame(&config, &scenario, 30_690);
        let requests = [
            AcqRequest {
                sat,
                glonass_frequency_channel: None,
                signal_band: SignalBand::L1,
                signal_code: SignalCode::Ca,
                doppler_center_hz: 0.0,
                doppler_rate_center_hz_per_s: 0.0,
                doppler_rate_search_hz_per_s: 0,
                doppler_rate_step_hz_per_s: 250,
                expected_line_of_sight_doppler_hz: None,
                assistance_bounds: None,
                doppler_search_hz: config.acquisition_doppler_search_hz,
                doppler_step_hz: config.acquisition_doppler_step_hz,
                coherent_ms: config.acquisition_integration_ms,
                noncoherent: config.acquisition_noncoherent,
            },
            AcqRequest {
                sat,
                glonass_frequency_channel: None,
                signal_band: SignalBand::L5,
                signal_code: SignalCode::L5I,
                doppler_center_hz: 0.0,
                doppler_rate_center_hz_per_s: 0.0,
                doppler_rate_search_hz_per_s: 0,
                doppler_rate_step_hz_per_s: 250,
                expected_line_of_sight_doppler_hz: None,
                assistance_bounds: None,
                doppler_search_hz: config.acquisition_doppler_search_hz,
                doppler_step_hz: config.acquisition_doppler_step_hz,
                coherent_ms: config.acquisition_integration_ms,
                noncoherent: config.acquisition_noncoherent,
            },
        ];

        let run = Acquisition::new(config, ReceiverRuntime::default())
            .run_fft_topn_for_requests_with_explain(&frame, &requests, 1);

        let l1_result = run.results[0].first().expect("L1 acquisition result");
        let l5_result = run.results[1].first().expect("L5 acquisition result");
        let assumptions = l5_result
            .assumptions
            .as_ref()
            .expect("cross-band follow-up should preserve assumptions");
        let expected_l5_center_hz = shared_path_doppler_hz(
            l1_result.doppler_hz.0,
            signal_spec_gps_l1_ca(),
            signal_spec_gps_l5_i(),
        )
        .expect("measured L1 Doppler should scale onto L5");

        assert!(
            matches!(l5_result.hypothesis, AcqHypothesis::Accepted | AcqHypothesis::Ambiguous),
            "{run:#?}"
        );
        assert!(assumptions.assistance_bounds.is_some(), "{l5_result:#?}");
        assert!(
            (assumptions.doppler_center_hz - expected_l5_center_hz).abs() <= 1.0e-6,
            "{l5_result:#?}"
        );
        assert!(
            l5_result
                .explain_selection_reason
                .as_deref()
                .is_some_and(|reason| reason.contains("same_satellite_cross_band_assistance")),
            "{l5_result:#?}"
        );
        assert!(
            run.explains[1].selected_reason.contains("same_satellite_cross_band_assistance"),
            "{run:#?}"
        );
    }

    #[test]
    fn related_signal_follow_up_does_not_cross_satellite_boundaries() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 10_230_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 10_230_000.0,
            code_length: 10_230,
            acquisition_doppler_search_hz: 2_000,
            acquisition_doppler_step_hz: 250,
            acquisition_integration_ms: 1,
            acquisition_noncoherent: 1,
            ..ReceiverPipelineConfig::default()
        };
        let l1_sat = SatId { constellation: Constellation::Gps, prn: 17 };
        let l5_sat = SatId { constellation: Constellation::Gps, prn: 18 };
        let l5_doppler_hz =
            shared_path_doppler_hz(-750.0, signal_spec_gps_l1_ca(), signal_spec_gps_l5_i())
                .expect("same-carrier-ratio scaling");
        let scenario = SyntheticScenario {
            sample_rate_hz: config.sampling_freq_hz,
            intermediate_freq_hz: config.intermediate_freq_hz,
            receiver_clock_frequency_bias_hz: 0.0,
            duration_s: 0.030,
            seed: 0x2810_0002,
            satellites: vec![
                SyntheticSignalParams {
                    sat: l1_sat,
                    glonass_frequency_channel: None,
                    signal_band: SignalBand::L1,
                    signal_code: SignalCode::Ca,
                    doppler_hz: -750.0,
                    code_phase_chips: 32.1,
                    carrier_phase_rad: 0.25,
                    cn0_db_hz: 56.0,
                    navigation_data: false.into(),
                },
                SyntheticSignalParams {
                    sat: l5_sat,
                    glonass_frequency_channel: None,
                    signal_band: SignalBand::L5,
                    signal_code: SignalCode::L5I,
                    doppler_hz: l5_doppler_hz,
                    code_phase_chips: 321.0,
                    carrier_phase_rad: 0.25,
                    cn0_db_hz: 31.5,
                    navigation_data: false.into(),
                },
            ],
            ephemerides: Vec::new(),
            id: "related-signal-follow-up-cross-satellite".to_string(),
        };
        let frame = signal_only_frame(&config, &scenario, 30_690);
        let requests = [
            AcqRequest {
                sat: l1_sat,
                glonass_frequency_channel: None,
                signal_band: SignalBand::L1,
                signal_code: SignalCode::Ca,
                doppler_center_hz: 0.0,
                doppler_rate_center_hz_per_s: 0.0,
                doppler_rate_search_hz_per_s: 0,
                doppler_rate_step_hz_per_s: 250,
                expected_line_of_sight_doppler_hz: None,
                assistance_bounds: None,
                doppler_search_hz: config.acquisition_doppler_search_hz,
                doppler_step_hz: config.acquisition_doppler_step_hz,
                coherent_ms: config.acquisition_integration_ms,
                noncoherent: config.acquisition_noncoherent,
            },
            AcqRequest {
                sat: l5_sat,
                glonass_frequency_channel: None,
                signal_band: SignalBand::L5,
                signal_code: SignalCode::L5I,
                doppler_center_hz: 0.0,
                doppler_rate_center_hz_per_s: 0.0,
                doppler_rate_search_hz_per_s: 0,
                doppler_rate_step_hz_per_s: 250,
                expected_line_of_sight_doppler_hz: None,
                assistance_bounds: None,
                doppler_search_hz: config.acquisition_doppler_search_hz,
                doppler_step_hz: config.acquisition_doppler_step_hz,
                coherent_ms: config.acquisition_integration_ms,
                noncoherent: config.acquisition_noncoherent,
            },
        ];

        let run = Acquisition::new(config, ReceiverRuntime::default())
            .run_fft_topn_for_requests_with_explain(&frame, &requests, 1);

        let l5_result = run.results[1].first().expect("L5 acquisition result");
        let assumptions = l5_result.assumptions.as_ref().expect("L5 assumptions");

        assert!(assumptions.assistance_bounds.is_none(), "{l5_result:#?}");
        assert!(
            !run.explains[1].selected_reason.contains("same_satellite_cross_band_assistance"),
            "{run:#?}"
        );
    }

    #[test]
    fn search_window_diagnostic_detects_doppler_rate_edge_rise() {
        let sat = SatId { constellation: Constellation::Gps, prn: 1 };
        let candidates = vec![
            search_window_rate_candidate(sat, 250.0, -20_000.0, 6.0),
            search_window_rate_candidate(sat, 250.0, -15_000.0, 5.0),
            search_window_rate_candidate(sat, 250.0, 15_000.0, 7.0),
            search_window_rate_candidate(sat, 250.0, 20_000.0, 9.0),
        ];

        let diagnostic =
            signal_outside_doppler_rate_search_range(&candidates, 250.0, 0.0, 20_000, 5_000, 4.0)
                .expect("doppler-rate edge diagnostic");

        assert_eq!(diagnostic.dimension, SearchWindowDimension::DopplerRate);
        assert_eq!(diagnostic.edge, SearchWindowEdge::Upper);
        assert_eq!(diagnostic.best_axis_value, 20_000.0);
        assert_eq!(diagnostic.interior_axis_value, 15_000.0);
    }

    fn candidate_for_search_window_test(
        sat: SatId,
        carrier_hz: f64,
        peak_mean_ratio: f32,
    ) -> AcqResult {
        AcqResult {
            sat,
            signal_band: SignalBand::L1,
            signal_code: SignalCode::Unknown,
            glonass_frequency_channel: None,
            source_time: ReceiverSampleTrace::default(),
            candidate_rank: 1,
            is_primary_candidate: true,
            doppler_hz: Hertz(carrier_hz),
            doppler_rate_hz_per_s: 0.0,
            carrier_hz: Hertz(carrier_hz),
            code_phase_samples: 0,
            peak: peak_mean_ratio,
            second_peak: 1.0,
            mean: 1.0,
            peak_mean_ratio,
            peak_second_ratio: peak_mean_ratio,
            cn0_proxy: peak_mean_ratio,
            score: 0.0,
            hypothesis: AcqHypothesis::Deferred,
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

    fn search_window_rate_candidate(
        sat: SatId,
        carrier_hz: f64,
        doppler_rate_hz_per_s: f64,
        peak_mean_ratio: f32,
    ) -> AcqResult {
        let mut candidate = candidate_for_search_window_test(sat, carrier_hz, peak_mean_ratio);
        candidate.doppler_rate_hz_per_s = doppler_rate_hz_per_s;
        candidate
    }
}
