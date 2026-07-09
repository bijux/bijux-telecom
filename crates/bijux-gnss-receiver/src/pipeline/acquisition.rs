#![allow(missing_docs)]

use std::collections::HashMap;
use std::sync::Mutex;

use bijux_gnss_core::api::{
    acq_result_stability_key, stable_acq_result_keys, AcqAssumptions, AcqEvidence, AcqExplain,
    AcqExplainCandidate, AcqHypothesis, AcqResult, AcqThresholdProvenance, Hertz,
    ReceiverSampleTrace, SamplesFrame, SatId,
};
use num_complex::Complex;
use rustfft::{num_traits::Zero, FftPlanner};

use crate::engine::receiver_config::ReceiverPipelineConfig;
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
    coherent_ms: u32,
    noncoherent: u32,
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
        coherent_ms: u32,
        noncoherent: u32,
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
            coherent_ms,
            noncoherent,
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
        let assumptions = AcqAssumptions {
            doppler_search_hz: self.doppler_search_hz,
            doppler_step_hz: self.doppler_step_hz,
            coherent_ms,
            noncoherent,
            samples_per_code,
            frame_samples: frame.len(),
            code_phase_search_start_sample: 0,
            code_phase_search_step_samples: 1,
            code_phase_search_bins: samples_per_code,
            code_phase_search_mode: "full_code".to_string(),
        };
        let threshold_provenance = AcqThresholdProvenance {
            coherent_ms,
            noncoherent,
            doppler_search_hz: self.doppler_search_hz,
            doppler_step_hz: self.doppler_step_hz,
            peak_mean_threshold: self.config.acquisition_peak_mean_threshold,
            peak_second_threshold: self.config.acquisition_peak_second_threshold,
        };
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

        let mut results = Vec::new();
        let mut explains = Vec::new();
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
            let mut candidates = Vec::new();

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

                let (peak_idx, peak, second, mean) = correlation_metrics(&noncoherent_acc);
                let peak_mean_ratio = peak / (mean + 1e-6);
                let peak_second_ratio = peak / (second + 1e-6);
                let cn0_proxy = peak_mean_ratio * 10.0;

                candidates.push(AcqResult {
                    sat,
                    source_time: ReceiverSampleTrace::from_sample_time(frame.t0),
                    carrier_hz: Hertz(carrier),
                    code_phase_samples: peak_idx,
                    peak,
                    second_peak: second,
                    mean,
                    peak_mean_ratio,
                    peak_second_ratio,
                    cn0_proxy,
                    score: 0.0,
                    hypothesis: AcqHypothesis::Deferred,
                    assumptions: Some(assumptions.clone()),
                    evidence: Vec::new(),
                    threshold_provenance: Some(threshold_provenance.clone()),
                    explain_selection_reason: None,
                });

                doppler += self.doppler_step_hz;
            }

            let search_window_diagnostic = signal_outside_search_range(
                &candidates,
                self.config.intermediate_freq_hz,
                self.doppler_search_hz,
                self.doppler_step_hz,
                self.config.acquisition_peak_mean_threshold,
            );

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
                continue;
            }

            let second_peak_ratio = candidates
                .get(1)
                .map(|candidate| candidate.peak_second_ratio)
                .unwrap_or(f32::INFINITY);
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
                let separation_ratio =
                    if rank == 0 { second_peak_ratio } else { candidate.peak_second_ratio };
                if rank == 0 {
                    if let Some(diagnostic) = search_window_diagnostic.as_ref() {
                        candidate.hypothesis = AcqHypothesis::Rejected;
                        candidate.score = 0.0;
                        candidate.explain_selection_reason =
                            Some(search_window_candidate_reason(diagnostic));
                    } else {
                        let (hypothesis, score) = acquisition_hypothesis(
                            candidate.peak_mean_ratio,
                            candidate.peak_second_ratio,
                            separation_ratio,
                            &self.config,
                        );
                        candidate.hypothesis = hypothesis;
                        candidate.score = score;
                        candidate.explain_selection_reason = Some(format!(
                            "rank 1 selected; peak_mean_ratio={:.6}, separation_ratio={:.6}, score={:.6}",
                            candidate.peak_mean_ratio,
                            separation_ratio,
                            score
                        ));
                    }
                } else {
                    candidate.hypothesis = AcqHypothesis::Deferred;
                    candidate.explain_selection_reason = Some("not_selected".to_string());
                }
            }

            let best = &candidates[0];
            if emit_explanations {
                let selected_reason = match search_window_diagnostic.as_ref() {
                    Some(_) => "signal_outside_search_range",
                    None => match best.hypothesis {
                        AcqHypothesis::Accepted => "accepted_by_ratio_thresholds",
                        AcqHypothesis::Ambiguous => "ambiguous_ratio_thresholds",
                        AcqHypothesis::Rejected => "rejected_peak_mean_threshold",
                        AcqHypothesis::Deferred => "deferred",
                    },
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
        coherent_ms: u32,
        noncoherent: u32,
        fft: &dyn rustfft::Fft<f32>,
    ) -> Vec<Complex<f32>> {
        let key = CodeFftCacheKey::from_runtime(
            &self.config,
            sat,
            samples_per_code,
            self.doppler_search_hz,
            self.doppler_step_hz,
            coherent_ms,
            noncoherent,
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
    let candidate_reason =
        insufficient_frame_candidate_reason(available_samples, required_samples);
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

fn insufficient_frame_candidate_reason(available_samples: usize, required_samples: usize) -> String {
    format!(
        "insufficient_frame: acquisition requires {required_samples} samples but received {available_samples}"
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

fn acquisition_hypothesis(
    peak_mean_ratio: f32,
    peak_second_ratio: f32,
    separation_ratio: f32,
    config: &ReceiverPipelineConfig,
) -> (AcqHypothesis, f32) {
    if peak_mean_ratio < config.acquisition_peak_mean_threshold {
        return (AcqHypothesis::Rejected, 0.0);
    }
    if !separation_ratio.is_finite() {
        return (AcqHypothesis::Ambiguous, 0.25 * peak_mean_ratio);
    }
    if peak_second_ratio < config.acquisition_peak_second_threshold
        || separation_ratio < config.acquisition_peak_second_threshold
    {
        return (AcqHypothesis::Ambiguous, (peak_mean_ratio * 0.35) + (peak_second_ratio * 0.15));
    }
    (AcqHypothesis::Accepted, (peak_mean_ratio * 0.5) + (peak_second_ratio * 0.5))
}

#[allow(clippy::items_after_test_module)]
#[cfg(test)]
mod tests {
    use super::*;
    use bijux_gnss_core::api::{Constellation, SampleTime, SamplesFrame, SatId, Seconds};

    #[test]
    fn acquisition_hypothesis_rejects_weak_primary_peak() {
        let config = ReceiverPipelineConfig::default();
        let (hypothesis, score) = acquisition_hypothesis(2.0, 2.0, 2.0, &config);
        assert_eq!(hypothesis.to_string(), "rejected");
        assert_eq!(score, 0.0);
    }

    #[test]
    fn acquisition_hypothesis_marks_ambiguous_on_low_peak_separation() {
        let config = ReceiverPipelineConfig::default();
        let (hypothesis, score) = acquisition_hypothesis(3.0, 1.0, 1.4, &config);
        assert_eq!(hypothesis.to_string(), "ambiguous");
        assert!((score - 1.2).abs() < 1e-6);
    }

    #[test]
    fn acquisition_hypothesis_accepts_clean_peak() {
        let config = ReceiverPipelineConfig::default();
        let (hypothesis, score) = acquisition_hypothesis(3.5, 2.0, 2.5, &config);
        assert_eq!(hypothesis.to_string(), "accepted");
        assert!((score - 2.75).abs() < f32::EPSILON);
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
        let samples_per_code =
            samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length);
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
        }
    }
}

fn correlation_metrics(corr: &[f32]) -> (usize, f32, f32, f32) {
    let mut peak_idx = 0;
    let mut peak = 0.0f32;
    let mut second = 0.0f32;
    let mut sum = 0.0f32;

    for (idx, &mag) in corr.iter().enumerate() {
        sum += mag;
        if mag > peak {
            second = peak;
            peak = mag;
            peak_idx = idx;
        } else if mag > second {
            second = mag;
        }
    }

    let mean = sum / corr.len().max(1) as f32;
    (peak_idx, peak, second, mean)
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
