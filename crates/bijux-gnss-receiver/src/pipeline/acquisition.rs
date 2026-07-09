#![allow(missing_docs)]

use std::collections::HashMap;
use std::sync::Mutex;

use bijux_gnss_core::api::{
    acq_result_stability_key, stable_acq_result_keys, AcqAssumptions, AcqCodePhaseRefinement,
    AcqDopplerRefinement, AcqEvidence, AcqExplain, AcqExplainCandidate, AcqHypothesis, AcqResult,
    AcqThresholdProvenance, AcqUncertainty, Hertz, ReceiverSampleTrace, SamplesFrame, SatId,
};
use num_complex::Complex;
use rustfft::{num_traits::Zero, FftPlanner};

use crate::engine::receiver_config::ReceiverPipelineConfig;
use crate::engine::receiver_config::{
    acquisition_integration_ms_is_supported, supported_acquisition_integration_ms_csv,
};
use crate::engine::runtime::{ReceiverRuntime, TraceRecord};
use crate::pipeline::doppler::carrier_hz_from_doppler_hz;
use bijux_gnss_signal::api::samples_per_code;
use bijux_gnss_signal::api::{
    generate_ca_code, measure_iq_front_end_metrics, sample_code, wipeoff_carrier, Prn,
};

/// Acquisition engine (coarse search).
pub struct Acquisition {
    config: ReceiverPipelineConfig,
    runtime: ReceiverRuntime,
    doppler_search_hz: i32,
    doppler_step_hz: i32,
    cache: Mutex<CodeFftCache>,
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

const ACQUISITION_CACHE_MODEL_VERSION: u32 = 1;
const ACQUISITION_CACHE_POLICY_VERSION: u32 = 1;
const SEARCH_EDGE_HINT_PEAK_MEAN_RATIO_FRACTION: f32 = 0.9;
const SEARCH_EDGE_RISE_RATIO_EPSILON: f32 = 0.05;
const SUB_BIN_DOPPLER_REFINEMENT_METHOD: &str = "parabolic_peak";
const SUB_BIN_DOPPLER_REFINEMENT_MAX_OFFSET_BINS: f64 = 0.5;
const SUB_BIN_DOPPLER_REFINEMENT_EPSILON: f64 = 1e-12;
const SUB_SAMPLE_CODE_PHASE_REFINEMENT_METHOD: &str = "parabolic_code_peak";
const SUB_SAMPLE_CODE_PHASE_REFINEMENT_MAX_OFFSET_SAMPLES: f64 = 0.5;
const SUB_SAMPLE_CODE_PHASE_REFINEMENT_EPSILON: f64 = 1e-12;
const SUB_SAMPLE_CODE_PHASE_REFINEMENT_MIN_ABS_OFFSET_SAMPLES: f64 = 0.05;
const ACQUISITION_UNCERTAINTY_MIN_RESOLUTION_FRACTION: f64 = 0.05;
const ACQUISITION_UNCERTAINTY_REFERENCE_RESOLUTION_FRACTION: f64 = 0.25;
const ACQUISITION_UNCERTAINTY_MAX_RESOLUTION_FRACTION: f64 = 0.5;
const MULTIPATH_SECONDARY_GUARD_CHIPS: usize = 2;
const WRONG_PRN_DOMINANCE_RATIO_MIN: f32 = 4.0;
const WRONG_PRN_PEAK_SECOND_RATIO_MAX: f32 = 1.1;

#[derive(Debug, Clone, Copy, Hash, PartialEq, Eq)]
struct CodeFftCacheKey {
    sat: SatId,
    samples_per_code: usize,
    sampling_hz_bits: u64,
    if_hz_bits: u64,
    code_hz_bits: u64,
    code_length: usize,
    doppler_search_hz: i32,
    doppler_step_hz: i32,
    model_version: u32,
    policy_version: u32,
}

impl CodeFftCacheKey {
    fn from_runtime(
        config: &ReceiverPipelineConfig,
        sat: SatId,
        samples_per_code: usize,
        doppler_search_hz: i32,
        doppler_step_hz: i32,
    ) -> Self {
        Self {
            sat,
            samples_per_code,
            sampling_hz_bits: config.sampling_freq_hz.to_bits(),
            if_hz_bits: config.intermediate_freq_hz.to_bits(),
            code_hz_bits: config.code_freq_basis_hz.to_bits(),
            code_length: config.code_length,
            doppler_search_hz,
            doppler_step_hz,
            model_version: ACQUISITION_CACHE_MODEL_VERSION,
            policy_version: ACQUISITION_CACHE_POLICY_VERSION,
        }
    }
}

#[derive(Debug, Clone, Copy)]
enum CacheMissReason {
    ColdStart,
    IncompatibleAssumptions,
}

impl CacheMissReason {
    fn as_str(self) -> &'static str {
        match self {
            CacheMissReason::ColdStart => "cold_start",
            CacheMissReason::IncompatibleAssumptions => "incompatible_assumptions",
        }
    }
}

type CodeFftCache = HashMap<CodeFftCacheKey, Vec<Complex<f32>>>;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SearchWindowEdge {
    Lower,
    Upper,
}

#[derive(Debug, Clone)]
struct SearchWindowDiagnostic {
    edge: SearchWindowEdge,
    best_carrier_hz: f64,
    interior_carrier_hz: f64,
    best_peak_mean_ratio: f32,
    interior_peak_mean_ratio: f32,
}

#[derive(Debug, Clone)]
struct AcquisitionSatEvaluation {
    sat: SatId,
    candidates: Vec<AcqResult>,
    search_window_diagnostic: Option<SearchWindowDiagnostic>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct CorrelationMetrics {
    peak_idx: usize,
    peak: f32,
    second_idx: usize,
    second: f32,
    mean: f32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct DelayedSecondaryPeakDiagnostic {
    secondary_code_phase_samples: usize,
    delay_samples: usize,
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
            stats: Mutex::new(AcquisitionStats::default()),
        }
    }

    pub fn with_doppler(mut self, search_hz: i32, step_hz: i32) -> Self {
        self.doppler_search_hz = search_hz.abs();
        self.doppler_step_hz = step_hz.max(1);
        self
    }

    fn full_code_search_assumptions(
        &self,
        frame_samples: usize,
        coherent_ms: u32,
        noncoherent: u32,
        samples_per_code: usize,
    ) -> AcqAssumptions {
        AcqAssumptions {
            doppler_search_hz: self.doppler_search_hz,
            doppler_step_hz: self.doppler_step_hz,
            coherent_ms,
            noncoherent,
            samples_per_code,
            frame_samples,
            code_phase_search_start_sample: 0,
            code_phase_search_step_samples: 1,
            code_phase_search_bins: samples_per_code,
            code_phase_search_mode: "full_code".to_string(),
        }
    }

    /// Perform satellite acquisition on a buffer that spans the configured integration window.
    pub fn run_fft(&self, frame: &SamplesFrame, sats: &[SatId]) -> Vec<AcqResult> {
        self.run_fft_topn(
            frame,
            sats,
            1,
            self.config.acquisition_integration_ms,
            self.config.acquisition_noncoherent,
        )
        .into_iter()
        .filter_map(|mut v| if v.is_empty() { None } else { Some(v.remove(0)) })
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
        self.run_fft_topn_internal(frame, sats, top_n, coherent_ms, noncoherent, true)
    }

    pub fn run_fft_topn(
        &self,
        frame: &SamplesFrame,
        sats: &[SatId],
        top_n: usize,
        coherent_ms: u32,
        noncoherent: u32,
    ) -> Vec<Vec<AcqResult>> {
        self.run_fft_topn_internal(frame, sats, top_n, coherent_ms, noncoherent, false).results
    }

    fn run_fft_topn_internal(
        &self,
        frame: &SamplesFrame,
        sats: &[SatId],
        top_n: usize,
        coherent_ms: u32,
        noncoherent: u32,
        emit_explanations: bool,
    ) -> AcquisitionRun {
        let samples_per_code = samples_per_code(
            self.config.sampling_freq_hz,
            self.config.code_freq_basis_hz,
            self.config.code_length,
        );
        let total_ms = (coherent_ms * noncoherent).max(1) as usize;
        let required = samples_per_code * total_ms;
        let assumptions = self.full_code_search_assumptions(
            frame.len(),
            coherent_ms,
            noncoherent,
            samples_per_code,
        );
        let threshold_provenance = AcqThresholdProvenance {
            coherent_ms,
            noncoherent,
            doppler_search_hz: self.doppler_search_hz,
            doppler_step_hz: self.doppler_step_hz,
            peak_mean_threshold: self.config.acquisition_peak_mean_threshold,
            peak_second_threshold: self.config.acquisition_peak_second_threshold,
        };
        if !acquisition_integration_ms_is_supported(coherent_ms) {
            self.runtime.trace.record(TraceRecord {
                name: "acquisition_input_rejection",
                fields: vec![
                    ("reason", "unsupported_coherent_integration_ms".to_string()),
                    ("coherent_ms", coherent_ms.to_string()),
                    ("supported_coherent_ms", supported_acquisition_integration_ms_csv()),
                    ("sat_count", sats.len().to_string()),
                ],
            });
            self.with_stats(|stats| {
                stats.deferred_count = stats.deferred_count.saturating_add(sats.len() as u64);
            });
            return unsupported_coherent_integration_run(
                sats,
                &assumptions,
                &threshold_provenance,
                self.config.intermediate_freq_hz,
                ReceiverSampleTrace::from_sample_time(frame.t0),
                coherent_ms,
                emit_explanations,
            );
        }
        if frame.len() < required {
            self.runtime.trace.record(TraceRecord {
                name: "acquisition_input_rejection",
                fields: vec![
                    ("reason", "insufficient_frame".to_string()),
                    ("available_samples", frame.len().to_string()),
                    ("required_samples", required.to_string()),
                    ("sat_count", sats.len().to_string()),
                ],
            });
            self.with_stats(|stats| {
                stats.deferred_count = stats.deferred_count.saturating_add(sats.len() as u64);
            });
            return insufficient_frame_run(
                sats,
                &assumptions,
                &threshold_provenance,
                self.config.intermediate_freq_hz,
                ReceiverSampleTrace::from_sample_time(frame.t0),
                frame.len(),
                required,
                emit_explanations,
            );
        }

        let mut planner = FftPlanner::<f32>::new();
        let fft = planner.plan_fft_forward(samples_per_code);
        let ifft = planner.plan_fft_inverse(samples_per_code);

        self.with_stats(|stats| {
            stats.sat_count += sats.len() as u64;
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
                stats.rejected_count = stats.rejected_count.saturating_add(sats.len() as u64);
            });
            return zero_signal_run(
                sats,
                &assumptions,
                &threshold_provenance,
                self.config.intermediate_freq_hz,
                ReceiverSampleTrace::from_sample_time(frame.t0),
                front_end_metrics.zero_signal_reason.as_deref(),
                emit_explanations,
            );
        }

        let mut sat_evaluations = Vec::new();
        self.with_stats(|stats| {
            stats.doppler_bins +=
                doppler_bin_count(self.doppler_search_hz, self.doppler_step_hz) * sats.len() as u64;
            stats.code_search_bins += (samples_per_code * sats.len()) as u64;
        });
        for &sat in sats {
            self.runtime.trace.record(TraceRecord {
                name: "acquisition_sat_start",
                fields: vec![
                    ("constellation", format!("{:?}", sat.constellation)),
                    ("prn", sat.prn.to_string()),
                ],
            });
            let code_fft =
                self.code_fft(sat, samples_per_code, coherent_ms, noncoherent, fft.as_ref());
            let mut grid_candidates = Vec::new();

            let mut doppler = -self.doppler_search_hz;
            while doppler <= self.doppler_search_hz {
                let carrier =
                    carrier_hz_from_doppler_hz(self.config.intermediate_freq_hz, doppler as f64);
                let mut noncoherent_acc = vec![0.0f32; samples_per_code];

                for nc in 0..noncoherent {
                    let mut coherent_corr: Vec<Complex<f32>> =
                        vec![Complex::zero(); samples_per_code];
                    for c in 0..coherent_ms {
                        let offset_ms = (nc * coherent_ms + c) as usize;
                        let start = offset_ms * samples_per_code;
                        let end = start + samples_per_code;
                        let block = &frame.iq[start..end];

                        let mixed = wipeoff_carrier(
                            block,
                            carrier,
                            self.config.sampling_freq_hz,
                            frame.t0.sample_index + start as u64,
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
                        for i in 0..samples_per_code {
                            coherent_corr[i] += prod[i];
                        }
                    }

                    for i in 0..samples_per_code {
                        noncoherent_acc[i] += coherent_corr[i].norm();
                    }
                }

                let correlation_metrics = correlation_metrics(&noncoherent_acc);
                let peak_mean_ratio =
                    correlation_metrics.peak / (correlation_metrics.mean + 1e-6);
                let peak_second_ratio =
                    correlation_metrics.peak / (correlation_metrics.second + 1e-6);
                let cn0_proxy = peak_mean_ratio * 10.0;
                grid_candidates.push(AcqResult {
                    sat,
                    source_time: ReceiverSampleTrace::from_sample_time(frame.t0),
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
                    evidence: Vec::new(),
                    threshold_provenance: Some(threshold_provenance.clone()),
                    explain_selection_reason: None,
                    doppler_refinement: None,
                    code_phase_refinement: None,
                    uncertainty: None,
                });

                doppler += self.doppler_step_hz;
            }

            let search_window_diagnostic = signal_outside_search_range(
                &grid_candidates,
                self.config.intermediate_freq_hz,
                self.doppler_search_hz,
                self.doppler_step_hz,
                self.config.acquisition_peak_mean_threshold,
            );

            let mut candidates = grid_candidates.clone();
            candidates.sort_by(|a, b| {
                let primary = b
                    .peak_mean_ratio
                    .partial_cmp(&a.peak_mean_ratio)
                    .unwrap_or(std::cmp::Ordering::Equal);
                if primary == std::cmp::Ordering::Equal {
                    return acq_result_stability_key(a).cmp(&acq_result_stability_key(b));
                }
                primary
            });
            candidates.truncate(top_n.max(1));
            refine_acquisition_candidates(
                self,
                frame,
                sat,
                &mut candidates,
                &grid_candidates,
                self.doppler_step_hz,
                coherent_ms,
                noncoherent,
            );
            if candidates.is_empty() {
                sat_evaluations.push(AcquisitionSatEvaluation {
                    sat,
                    candidates: Vec::new(),
                    search_window_diagnostic: None,
                });
                continue;
            }

            let competing_peak_ratio = competing_candidate_ratio(&candidates);
            for (rank, candidate) in candidates.iter_mut().enumerate() {
                candidate.evidence.push(AcqEvidence {
                    rank: rank as u8 + 1,
                    code_phase_samples: candidate.code_phase_samples,
                    doppler_hz: candidate.carrier_hz.0,
                    peak: candidate.peak,
                    second_peak: candidate.second_peak,
                    peak_mean_ratio: candidate.peak_mean_ratio,
                    peak_second_ratio: candidate.peak_second_ratio,
                    mean: candidate.mean,
                });
                let local_peak_separation_ratio = candidate.peak_second_ratio;
                if rank == 0 {
                    if let Some(diagnostic) = search_window_diagnostic.as_ref() {
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
                            &self.config,
                        );
                        let multipath_diagnostic = classify_delayed_secondary_peak(
                            &self.config,
                            frame,
                            sat,
                            candidate.carrier_hz.0,
                            candidate.code_phase_samples,
                            samples_per_code,
                            coherent_ms,
                            noncoherent,
                            candidate.peak_mean_ratio,
                            candidate.peak_second_ratio,
                            competing_peak_ratio,
                        );
                        let decision = multipath_diagnostic
                            .as_ref()
                            .map_or(decision, |diagnostic| multipath_suspect_decision(
                                candidate.peak_mean_ratio,
                                candidate.peak_second_ratio,
                                competing_peak_ratio,
                                diagnostic,
                            ));
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
                                    self.config.code_length,
                                    decision.score,
                                ),
                                None => selected_candidate_reason(
                                    decision,
                                    candidate.peak_mean_ratio,
                                    local_peak_separation_ratio,
                                    competing_peak_ratio,
                                    &self.config,
                                ),
                            });
                        candidate.uncertainty =
                            estimate_acquisition_uncertainty(candidate, self.doppler_step_hz);
                    }
                } else {
                    candidate.hypothesis = AcqHypothesis::Deferred;
                    candidate.explain_selection_reason = Some("not_selected".to_string());
                }
            }
            sat_evaluations.push(AcquisitionSatEvaluation {
                sat,
                candidates,
                search_window_diagnostic,
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
                    selected_rank: Some(1),
                    selected_reason: selected_reason.to_string(),
                    candidate_count: candidates.len(),
                    candidates: candidates
                        .iter()
                        .enumerate()
                        .map(|(rank, candidate)| AcqExplainCandidate {
                            rank: rank as u8 + 1,
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
                        ("best_carrier_hz", format!("{:.3}", diagnostic.best_carrier_hz)),
                        ("interior_carrier_hz", format!("{:.3}", diagnostic.interior_carrier_hz)),
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
        sat: SatId,
        samples_per_code: usize,
        _coherent_ms: u32,
        _noncoherent: u32,
        fft: &dyn rustfft::Fft<f32>,
    ) -> Vec<Complex<f32>> {
        let key = CodeFftCacheKey::from_runtime(
            &self.config,
            sat,
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
                    ("samples_per_code", samples_per_code.to_string()),
                ],
            });
            return cached;
        }

        let miss_reason = self.cache.lock().ok().map_or(CacheMissReason::ColdStart, |cache| {
            let has_same_satellite = cache
                .keys()
                .any(|cached| cached.sat == sat && cached.samples_per_code == samples_per_code);
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
                ("samples_per_code", samples_per_code.to_string()),
                ("reason", miss_reason.as_str().to_string()),
                ("model_version", ACQUISITION_CACHE_MODEL_VERSION.to_string()),
                ("policy_version", ACQUISITION_CACHE_POLICY_VERSION.to_string()),
            ],
        });
        let code = ca_code_or_default(sat.prn);
        let local_code = sample_local_code_period(&self.config, &code, samples_per_code);
        let mut code_fft: Vec<Complex<f32>> =
            local_code.iter().map(|&x| Complex::new(x, 0.0)).collect();
        fft.process(&mut code_fft);
        if let Ok(mut cache) = self.cache.lock() {
            cache.insert(key, code_fft.clone());
        }
        code_fft
    }

    fn estimate_acquisition_code_phase_refinement(
        &self,
        frame: &SamplesFrame,
        sat: SatId,
        carrier_hz: f64,
        coarse_code_phase_samples: usize,
        coherent_ms: u32,
        noncoherent: u32,
    ) -> Option<AcqCodePhaseRefinement> {
        let samples_per_code = samples_per_code(
            self.config.sampling_freq_hz,
            self.config.code_freq_basis_hz,
            self.config.code_length,
        );
        if samples_per_code == 0
            || frame.len() < samples_per_code * (coherent_ms * noncoherent) as usize
        {
            return None;
        }
        let correlation_profile = measure_code_phase_profile(
            &self.config,
            frame,
            sat,
            carrier_hz,
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

fn doppler_bin_count(search_hz: i32, step_hz: i32) -> u64 {
    if step_hz <= 0 {
        return 0;
    }
    let search = search_hz.unsigned_abs() as u64;
    let step = step_hz.unsigned_abs() as u64;
    (search / step).saturating_mul(2).saturating_add(1)
}

fn zero_signal_run(
    sats: &[SatId],
    assumptions: &AcqAssumptions,
    threshold_provenance: &AcqThresholdProvenance,
    intermediate_freq_hz: f64,
    source_time: ReceiverSampleTrace,
    zero_signal_reason: Option<&str>,
    emit_explanations: bool,
) -> AcquisitionRun {
    let candidate_reason = zero_signal_candidate_reason(zero_signal_reason);
    let mut results = Vec::with_capacity(sats.len());
    let mut explains = Vec::new();

    for &sat in sats {
        let result = AcqResult {
            sat,
            source_time,
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
            explain_selection_reason: Some(candidate_reason.clone()),
            doppler_refinement: None,
            code_phase_refinement: None,
            uncertainty: None,
        };
        if emit_explanations {
            explains.push(AcqExplain {
                sat,
                selected_rank: Some(1),
                selected_reason: "zero_signal_input".to_string(),
                candidate_count: 1,
                candidates: vec![AcqExplainCandidate {
                    rank: 1,
                    code_phase_samples: 0,
                    carrier_hz: intermediate_freq_hz,
                    peak: 0.0,
                    peak_mean_ratio: 0.0,
                    peak_second_ratio: 0.0,
                    second_peak_ratio: 0.0,
                    mean: 0.0,
                    hypothesis: AcqHypothesis::Rejected,
                    score: 0.0,
                    threshold_hit: false,
                    reason: candidate_reason.clone(),
                }],
            });
        }
        results.push(vec![result]);
    }

    AcquisitionRun { results, explains }
}

fn insufficient_frame_run(
    sats: &[SatId],
    assumptions: &AcqAssumptions,
    threshold_provenance: &AcqThresholdProvenance,
    intermediate_freq_hz: f64,
    source_time: ReceiverSampleTrace,
    available_samples: usize,
    required_samples: usize,
    emit_explanations: bool,
) -> AcquisitionRun {
    let candidate_reason = insufficient_frame_candidate_reason(available_samples, required_samples);
    let mut results = Vec::with_capacity(sats.len());
    let mut explains = Vec::new();

    for &sat in sats {
        let result = AcqResult {
            sat,
            source_time,
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
            explain_selection_reason: Some(candidate_reason.clone()),
            doppler_refinement: None,
            code_phase_refinement: None,
            uncertainty: None,
        };
        if emit_explanations {
            explains.push(AcqExplain {
                sat,
                selected_rank: Some(1),
                selected_reason: "insufficient_frame".to_string(),
                candidate_count: 1,
                candidates: vec![AcqExplainCandidate {
                    rank: 1,
                    code_phase_samples: 0,
                    carrier_hz: intermediate_freq_hz,
                    peak: 0.0,
                    peak_mean_ratio: 0.0,
                    peak_second_ratio: 0.0,
                    second_peak_ratio: 0.0,
                    mean: 0.0,
                    hypothesis: AcqHypothesis::Deferred,
                    score: 0.0,
                    threshold_hit: false,
                    reason: candidate_reason.clone(),
                }],
            });
        }
        results.push(vec![result]);
    }

    AcquisitionRun { results, explains }
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

fn unsupported_coherent_integration_run(
    sats: &[SatId],
    assumptions: &AcqAssumptions,
    threshold_provenance: &AcqThresholdProvenance,
    intermediate_freq_hz: f64,
    source_time: ReceiverSampleTrace,
    coherent_ms: u32,
    emit_explanations: bool,
) -> AcquisitionRun {
    let candidate_reason = unsupported_coherent_integration_candidate_reason(coherent_ms);
    let mut results = Vec::with_capacity(sats.len());
    let mut explains = Vec::new();

    for &sat in sats {
        let result = AcqResult {
            sat,
            source_time,
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
            explain_selection_reason: Some(candidate_reason.clone()),
            doppler_refinement: None,
            code_phase_refinement: None,
            uncertainty: None,
        };
        if emit_explanations {
            explains.push(AcqExplain {
                sat,
                selected_rank: Some(1),
                selected_reason: "unsupported_coherent_integration_ms".to_string(),
                candidate_count: 1,
                candidates: vec![AcqExplainCandidate {
                    rank: 1,
                    code_phase_samples: 0,
                    carrier_hz: intermediate_freq_hz,
                    peak: 0.0,
                    peak_mean_ratio: 0.0,
                    peak_second_ratio: 0.0,
                    second_peak_ratio: 0.0,
                    mean: 0.0,
                    hypothesis: AcqHypothesis::Deferred,
                    score: 0.0,
                    threshold_hit: false,
                    reason: candidate_reason.clone(),
                }],
            });
        }
        results.push(vec![result]);
    }

    AcquisitionRun { results, explains }
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

    let hint_threshold = peak_mean_threshold * SEARCH_EDGE_HINT_PEAK_MEAN_RATIO_FRACTION;
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
        let edge_candidate = candidates
            .iter()
            .find(|candidate| (candidate.carrier_hz.0 - edge_carrier_hz).abs() <= f64::EPSILON)?;
        let interior_candidate = candidates.iter().find(|candidate| {
            (candidate.carrier_hz.0 - interior_carrier_hz).abs() <= f64::EPSILON
        })?;
        if edge_candidate.peak_mean_ratio < hint_threshold {
            return None;
        }
        if edge_candidate.peak_mean_ratio
            <= interior_candidate.peak_mean_ratio * (1.0 + SEARCH_EDGE_RISE_RATIO_EPSILON)
        {
            return None;
        }
        Some(SearchWindowDiagnostic {
            edge,
            best_carrier_hz: edge_candidate.carrier_hz.0,
            interior_carrier_hz: interior_candidate.carrier_hz.0,
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

fn search_window_candidate_reason(diagnostic: &SearchWindowDiagnostic) -> String {
    let edge = match diagnostic.edge {
        SearchWindowEdge::Lower => "lower",
        SearchWindowEdge::Upper => "upper",
    };
    format!(
        "signal_outside_search_range: best carrier {:.3} Hz sits on the {edge} search edge and exceeds the interior neighbor at {:.3} Hz (peak_mean_ratio {:.6} > {:.6})",
        diagnostic.best_carrier_hz,
        diagnostic.interior_carrier_hz,
        diagnostic.best_peak_mean_ratio,
        diagnostic.interior_peak_mean_ratio,
    )
}

fn acquisition_decision(
    peak_mean_ratio: f32,
    peak_second_ratio: f32,
    local_peak_separation_ratio: f32,
    competing_peak_ratio: f32,
    config: &ReceiverPipelineConfig,
) -> AcquisitionDecision {
    if peak_mean_ratio < config.acquisition_peak_mean_threshold {
        return AcquisitionDecision {
            hypothesis: AcqHypothesis::Rejected,
            reason: AcquisitionDecisionReason::LowPeakMetric,
            score: 0.0,
        };
    }
    if !local_peak_separation_ratio.is_finite() || !competing_peak_ratio.is_finite() {
        return AcquisitionDecision {
            hypothesis: AcqHypothesis::Ambiguous,
            reason: AcquisitionDecisionReason::AmbiguousRatioThresholds,
            score: 0.25 * peak_mean_ratio,
        };
    }
    let limiting_ratio =
        peak_second_ratio.min(local_peak_separation_ratio).min(competing_peak_ratio);
    if peak_second_ratio < config.acquisition_peak_second_threshold
        || local_peak_separation_ratio < config.acquisition_peak_second_threshold
        || competing_peak_ratio < config.acquisition_peak_second_threshold
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
        "wrong_prn_correlation" => Some("wrong_prn_correlation"),
        _ => None,
    }
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
    config: &ReceiverPipelineConfig,
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
            config.acquisition_peak_mean_threshold,
        ),
        AcquisitionDecisionReason::MultipathSuspect => unreachable!(
            "multipath_suspect reasons must be rendered through multipath_candidate_reason"
        ),
    }
}

fn classify_delayed_secondary_peak(
    config: &ReceiverPipelineConfig,
    frame: &SamplesFrame,
    sat: SatId,
    carrier_hz: f64,
    code_phase_samples: usize,
    samples_per_code: usize,
    coherent_ms: u32,
    noncoherent: u32,
    peak_mean_ratio: f32,
    peak_second_ratio: f32,
    competing_peak_ratio: f32,
) -> Option<DelayedSecondaryPeakDiagnostic> {
    if peak_mean_ratio < config.acquisition_peak_mean_threshold
        || peak_second_ratio >= config.acquisition_peak_second_threshold
        || competing_peak_ratio < config.acquisition_peak_second_threshold
    {
        return None;
    }
    let correlation_profile =
        measure_code_phase_profile(config, frame, sat, carrier_hz, coherent_ms, noncoherent)?;
    delayed_secondary_peak_diagnostic(
        &correlation_profile,
        code_phase_samples,
        samples_per_code,
        config.code_length,
    )
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
    let delay_chips =
        diagnostic.delay_samples as f64 * code_length.max(1) as f64 / samples_per_code.max(1) as f64;
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
    match (candidates.first(), candidates.get(1)) {
        (Some(best), Some(competing)) if competing.peak_mean_ratio > f32::EPSILON => {
            best.peak_mean_ratio / competing.peak_mean_ratio
        }
        _ => f32::INFINITY,
    }
}

fn refine_acquisition_candidates(
    acquisition: &Acquisition,
    frame: &SamplesFrame,
    sat: SatId,
    candidates: &mut [AcqResult],
    grid_candidates: &[AcqResult],
    doppler_step_hz: i32,
    coherent_ms: u32,
    noncoherent: u32,
) {
    for candidate in candidates {
        let coarse_carrier_hz = candidate.carrier_hz.0;
        if let Some(refinement) = estimate_acquisition_doppler_refinement(
            coarse_carrier_hz,
            grid_candidates,
            doppler_step_hz,
        ) {
            candidate.carrier_hz = Hertz(coarse_carrier_hz + refinement.offset_hz);
            candidate.doppler_refinement = Some(refinement);
        }
        candidate.code_phase_refinement = acquisition.estimate_acquisition_code_phase_refinement(
            frame,
            sat,
            candidate.carrier_hz.0,
            candidate.code_phase_samples,
            coherent_ms,
            noncoherent,
        );
    }
}

fn estimate_acquisition_uncertainty(
    candidate: &AcqResult,
    doppler_step_hz: i32,
) -> Option<AcqUncertainty> {
    if !matches!(candidate.hypothesis, AcqHypothesis::Accepted) {
        return None;
    }
    Some(AcqUncertainty {
        doppler_hz: estimate_doppler_uncertainty_hz(candidate, doppler_step_hz)?,
        code_phase_samples: estimate_code_phase_uncertainty_samples(candidate)?,
    })
}

fn estimate_doppler_uncertainty_hz(candidate: &AcqResult, doppler_step_hz: i32) -> Option<f64> {
    let step_hz = doppler_step_hz.unsigned_abs() as f64;
    if step_hz <= f64::EPSILON {
        return None;
    }
    let uncertainty_bins = candidate
        .doppler_refinement
        .as_ref()
        .and_then(|refinement| {
            curvature_based_resolution_fraction(
                refinement.left_peak_mean_ratio as f64,
                refinement.center_peak_mean_ratio as f64,
                refinement.right_peak_mean_ratio as f64,
                1.0 + refinement.offset_bins.abs(),
            )
        })
        .unwrap_or(ACQUISITION_UNCERTAINTY_MAX_RESOLUTION_FRACTION);
    Some(uncertainty_bins * step_hz)
}

fn estimate_code_phase_uncertainty_samples(candidate: &AcqResult) -> Option<f64> {
    Some(
        candidate
            .code_phase_refinement
            .as_ref()
            .and_then(|refinement| {
                curvature_based_resolution_fraction(
                    refinement.left_correlation_norm as f64,
                    refinement.center_correlation_norm as f64,
                    refinement.right_correlation_norm as f64,
                    1.0 + refinement.offset_samples.abs(),
                )
            })
            .unwrap_or(ACQUISITION_UNCERTAINTY_MAX_RESOLUTION_FRACTION),
    )
}

fn curvature_based_resolution_fraction(
    left_metric: f64,
    center_metric: f64,
    right_metric: f64,
    offset_penalty: f64,
) -> Option<f64> {
    if !left_metric.is_finite()
        || !center_metric.is_finite()
        || !right_metric.is_finite()
        || !offset_penalty.is_finite()
        || center_metric <= f64::EPSILON
    {
        return None;
    }
    let normalized_curvature =
        ((2.0 * center_metric) - left_metric - right_metric).max(0.0) / center_metric.max(f64::EPSILON);
    if normalized_curvature <= f64::EPSILON {
        return Some(ACQUISITION_UNCERTAINTY_MAX_RESOLUTION_FRACTION);
    }
    Some(
        ((ACQUISITION_UNCERTAINTY_REFERENCE_RESOLUTION_FRACTION / normalized_curvature.sqrt())
            * offset_penalty)
            .clamp(
                ACQUISITION_UNCERTAINTY_MIN_RESOLUTION_FRACTION,
                ACQUISITION_UNCERTAINTY_MAX_RESOLUTION_FRACTION,
            ),
    )
}

fn estimate_acquisition_doppler_refinement(
    coarse_carrier_hz: f64,
    grid_candidates: &[AcqResult],
    doppler_step_hz: i32,
) -> Option<AcqDopplerRefinement> {
    if doppler_step_hz <= 0 {
        return None;
    }
    let step_hz = doppler_step_hz as f64;
    let left = find_candidate_by_carrier_hz(grid_candidates, coarse_carrier_hz - step_hz)?;
    let center = find_candidate_by_carrier_hz(grid_candidates, coarse_carrier_hz)?;
    let right = find_candidate_by_carrier_hz(grid_candidates, coarse_carrier_hz + step_hz)?;
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

fn measure_code_phase_profile(
    config: &ReceiverPipelineConfig,
    frame: &SamplesFrame,
    sat: SatId,
    carrier_hz: f64,
    coherent_ms: u32,
    noncoherent: u32,
) -> Option<Vec<f32>> {
    let samples_per_code =
        samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length);
    if samples_per_code == 0 {
        return None;
    }
    let mut planner = FftPlanner::<f32>::new();
    let fft = planner.plan_fft_forward(samples_per_code);
    let ifft = planner.plan_fft_inverse(samples_per_code);
    let code = ca_code_or_default(sat.prn);
    let local_code = sample_local_code_period(config, &code, samples_per_code);
    let mut code_fft: Vec<Complex<f32>> =
        local_code.iter().map(|&x| Complex::new(x, 0.0)).collect();
    fft.process(&mut code_fft);
    let mut noncoherent_acc = vec![0.0f32; samples_per_code];

    for nc in 0..noncoherent {
        let mut coherent_corr: Vec<Complex<f32>> = vec![Complex::zero(); samples_per_code];
        for c in 0..coherent_ms {
            let offset_ms = (nc * coherent_ms + c) as usize;
            let start = offset_ms * samples_per_code;
            let end = start + samples_per_code;
            let block = &frame.iq[start..end];
            let mixed = wipeoff_carrier(
                block,
                carrier_hz,
                config.sampling_freq_hz,
                frame.t0.sample_index + start as u64,
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
    candidates.iter().find(|candidate| (candidate.carrier_hz.0 - carrier_hz).abs() <= f64::EPSILON)
}

#[allow(clippy::items_after_test_module)]
#[cfg(test)]
mod tests {
    use super::*;
    use bijux_gnss_core::api::{Constellation, SampleTime, SamplesFrame, SatId, Seconds};

    #[test]
    fn acquisition_decision_rejects_weak_primary_peak() {
        let config = ReceiverPipelineConfig::default();
        let decision = acquisition_decision(2.0, 2.0, 2.0, 2.0, &config);
        assert_eq!(decision.hypothesis.to_string(), "rejected");
        assert_eq!(decision.reason, AcquisitionDecisionReason::LowPeakMetric);
        assert_eq!(decision.score, 0.0);
    }

    #[test]
    fn acquisition_decision_marks_ambiguous_on_low_peak_separation() {
        let config = ReceiverPipelineConfig::default();
        let decision = acquisition_decision(3.0, 1.0, 1.4, 2.0, &config);
        assert_eq!(decision.hypothesis.to_string(), "ambiguous");
        assert_eq!(decision.reason, AcquisitionDecisionReason::AmbiguousRatioThresholds);
        assert!((decision.score - 1.2).abs() < 1e-6);
    }

    #[test]
    fn acquisition_decision_marks_ambiguous_on_comparable_competing_candidate() {
        let config = ReceiverPipelineConfig::default();
        let decision = acquisition_decision(3.5, 2.0, 2.0, 1.4, &config);
        assert_eq!(decision.hypothesis.to_string(), "ambiguous");
        assert_eq!(decision.reason, AcquisitionDecisionReason::AmbiguousRatioThresholds);
        assert!((decision.score - 1.435).abs() < 1e-6);
    }

    #[test]
    fn acquisition_decision_accepts_clean_peak() {
        let config = ReceiverPipelineConfig::default();
        let decision = acquisition_decision(3.5, 2.0, 2.5, 2.1, &config);
        assert_eq!(decision.hypothesis.to_string(), "accepted");
        assert_eq!(decision.reason, AcquisitionDecisionReason::AcceptedByRatioThresholds);
        assert!((decision.score - 2.75).abs() < f32::EPSILON);
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
    fn selected_candidate_reason_reports_low_peak_metric_threshold() {
        let config = ReceiverPipelineConfig::default();
        let reason = selected_candidate_reason(
            AcquisitionDecision {
                hypothesis: AcqHypothesis::Rejected,
                reason: AcquisitionDecisionReason::LowPeakMetric,
                score: 0.0,
            },
            2.0,
            2.0,
            2.0,
            &config,
        );

        assert_eq!(reason, "low_peak_metric: peak_mean_ratio=2.000000 below threshold 2.500000");
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

        let refinement =
            estimate_acquisition_doppler_refinement(0.0, &candidates, 250).expect("refinement");

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

        let refinement = estimate_acquisition_doppler_refinement(0.0, &candidates, 250);

        assert!(refinement.is_none());
    }

    #[test]
    fn curvature_based_resolution_fraction_shrinks_for_sharper_peaks() {
        let broad = curvature_based_resolution_fraction(14.0, 16.0, 15.0, 1.0).expect("broad");
        let sharp = curvature_based_resolution_fraction(9.0, 16.0, 12.0, 1.0).expect("sharp");

        assert!(sharp < broad, "sharp={sharp:?} broad={broad:?}");
        assert!(sharp >= ACQUISITION_UNCERTAINTY_MIN_RESOLUTION_FRACTION);
        assert!(broad <= ACQUISITION_UNCERTAINTY_MAX_RESOLUTION_FRACTION);
    }

    #[test]
    fn estimate_acquisition_uncertainty_reports_positive_values_for_accepted_candidate() {
        let sat = SatId { constellation: Constellation::Gps, prn: 1 };
        let uncertainty = estimate_acquisition_uncertainty(
            &AcqResult {
                hypothesis: AcqHypothesis::Accepted,
                doppler_refinement: Some(AcqDopplerRefinement {
                    method: "parabolic_peak".to_string(),
                    coarse_carrier_hz: Hertz(250.0),
                    offset_hz: 20.0,
                    offset_bins: 0.08,
                    left_peak_mean_ratio: 9.0,
                    center_peak_mean_ratio: 16.0,
                    right_peak_mean_ratio: 12.0,
                }),
                code_phase_refinement: Some(AcqCodePhaseRefinement {
                    method: "parabolic_code_peak".to_string(),
                    offset_samples: 0.2,
                    refined_code_phase_samples: 1500.2,
                    left_correlation_norm: 10.0,
                    center_correlation_norm: 16.0,
                    right_correlation_norm: 13.0,
                }),
                ..candidate_for_search_window_test(sat, 250.0, 16.0)
            },
            250,
        )
        .expect("acquisition uncertainty");

        assert!(uncertainty.doppler_hz > 0.0);
        assert!(uncertainty.doppler_hz < 125.0);
        assert!(uncertainty.code_phase_samples > 0.0);
        assert!(uncertainty.code_phase_samples < 0.5);
    }

    #[test]
    fn estimate_acquisition_uncertainty_skips_ambiguous_candidate() {
        let sat = SatId { constellation: Constellation::Gps, prn: 1 };
        let uncertainty = estimate_acquisition_uncertainty(
            &AcqResult {
                hypothesis: AcqHypothesis::Ambiguous,
                ..candidate_for_search_window_test(sat, 0.0, 4.0)
            },
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
        let diagnostic = delayed_secondary_peak_diagnostic(
            &[1.0, 8.0, 7.6, 7.2, 1.4, 5.5, 1.2, 0.8],
            1,
            8,
            4,
        )
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
        let frame = SamplesFrame::new(
            SampleTime { sample_index: 0, sample_rate_hz: config.sampling_freq_hz },
            Seconds(1.0 / config.sampling_freq_hz),
            Vec::new(),
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

        acquisition.code_fft(sat, samples_per_code, 1, 1, fft.as_ref());
        let after_first_profile = acquisition.stats_snapshot();
        assert_eq!(after_first_profile.cache_misses, 1);
        assert_eq!(after_first_profile.cache_hits, 0);

        acquisition.code_fft(sat, samples_per_code, 1, 4, fft.as_ref());
        let after_second_profile = acquisition.stats_snapshot();
        assert_eq!(after_second_profile.cache_misses, 1);
        assert_eq!(after_second_profile.cache_hits, 1);
        assert_eq!(after_second_profile.cache_miss_incompatible, 0);
    }

    #[test]
    fn acquisition_stability_keys_are_sorted() {
        let sat = SatId { constellation: bijux_gnss_core::api::Constellation::Gps, prn: 1 };
        let mut rows = vec![
            AcqResult {
                sat,
                source_time: ReceiverSampleTrace::default(),
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
                uncertainty: None,
            },
            AcqResult {
                sat,
                source_time: ReceiverSampleTrace::default(),
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

        assert_eq!(diagnostic.edge, SearchWindowEdge::Upper);
        assert_eq!(diagnostic.best_carrier_hz, 1_500.0);
        assert_eq!(diagnostic.interior_carrier_hz, 1_250.0);
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
                candidate_for_search_window_test(sat, -1_500.0, 2.3),
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

        assert_eq!(diagnostic.edge, SearchWindowEdge::Lower);
        assert_eq!(diagnostic.best_carrier_hz, -1_500.0);
        assert_eq!(diagnostic.interior_carrier_hz, -1_250.0);
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

        assert_eq!(diagnostic.edge, SearchWindowEdge::Upper);
        assert_eq!(
            diagnostic.best_carrier_hz,
            carrier_hz_from_doppler_hz(intermediate_freq_hz, 1_500.0)
        );
        assert_eq!(
            diagnostic.interior_carrier_hz,
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
        let frame = SamplesFrame::new(
            SampleTime { sample_index: 0, sample_rate_hz: config.sampling_freq_hz },
            Seconds(1.0 / config.sampling_freq_hz),
            vec![Complex::zero(); samples_per_code],
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

    fn candidate_for_search_window_test(
        sat: SatId,
        carrier_hz: f64,
        peak_mean_ratio: f32,
    ) -> AcqResult {
        AcqResult {
            sat,
            source_time: ReceiverSampleTrace::default(),
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
            uncertainty: None,
        }
    }
}

fn correlation_metrics(corr: &[f32]) -> CorrelationMetrics {
    let mut peak_idx = 0;
    let mut peak = 0.0f32;
    let mut second_idx = 0usize;
    let mut second = 0.0f32;
    let mut sum = 0.0f32;

    for (idx, &mag) in corr.iter().enumerate() {
        sum += mag;
        if mag > peak {
            second_idx = peak_idx;
            second = peak;
            peak = mag;
            peak_idx = idx;
        } else if mag > second {
            second_idx = idx;
            second = mag;
        }
    }

    let mean = sum / corr.len().max(1) as f32;
    CorrelationMetrics { peak_idx, peak, second_idx, second, mean }
}

fn delayed_secondary_peak_diagnostic(
    correlation_profile: &[f32],
    peak_idx: usize,
    samples_per_code: usize,
    code_length: usize,
) -> Option<DelayedSecondaryPeakDiagnostic> {
    if correlation_profile.len() < 3 || samples_per_code == 0 || code_length == 0 {
        return None;
    }
    let min_delay_samples =
        samples_per_chip(samples_per_code, code_length).saturating_mul(MULTIPATH_SECONDARY_GUARD_CHIPS);
    let period_samples = correlation_profile.len();
    let mut best_secondary_idx = None;
    let mut best_secondary_peak = 0.0f32;

    for (idx, &magnitude) in correlation_profile.iter().enumerate() {
        let delay_samples = wrapped_code_phase_offset_samples(peak_idx, idx, period_samples);
        if delay_samples < min_delay_samples || delay_samples > period_samples / 2 {
            continue;
        }
        if !is_local_code_phase_peak(correlation_profile, idx) {
            continue;
        }
        if magnitude > best_secondary_peak {
            best_secondary_peak = magnitude;
            best_secondary_idx = Some((idx, delay_samples));
        }
    }

    let (secondary_code_phase_samples, delay_samples) = best_secondary_idx?;
    Some(DelayedSecondaryPeakDiagnostic { secondary_code_phase_samples, delay_samples })
}

fn is_local_code_phase_peak(correlation_profile: &[f32], idx: usize) -> bool {
    if correlation_profile.len() < 3 {
        return false;
    }
    let period_samples = correlation_profile.len();
    let left = correlation_profile[(idx + period_samples - 1) % period_samples];
    let center = correlation_profile[idx];
    let right = correlation_profile[(idx + 1) % period_samples];

    (center >= left && center >= right) && (center > left || center > right)
}

fn samples_per_chip(samples_per_code: usize, code_length: usize) -> usize {
    ((samples_per_code + code_length.saturating_sub(1)) / code_length.max(1)).max(1)
}

fn wrapped_code_phase_offset_samples(
    primary_code_phase_samples: usize,
    secondary_code_phase_samples: usize,
    period_samples: usize,
) -> usize {
    if period_samples == 0 {
        return 0;
    }
    (secondary_code_phase_samples + period_samples - primary_code_phase_samples) % period_samples
}

fn sample_local_code_period(
    config: &ReceiverPipelineConfig,
    code: &[i8],
    samples_per_code: usize,
) -> Vec<f32> {
    sample_code(code, config.sampling_freq_hz, config.code_freq_basis_hz, 0.0, samples_per_code)
        .unwrap_or_else(|_| vec![1.0; samples_per_code])
}

fn ca_code_or_default(prn: u8) -> Vec<i8> {
    match generate_ca_code(Prn(prn)) {
        Ok(code) => code,
        Err(_) => vec![1; 1023],
    }
}
