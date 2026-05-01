#![allow(missing_docs)]

use std::collections::HashMap;
use std::sync::Mutex;

use bijux_gnss_core::api::{
    acq_result_stability_key, stable_acq_result_keys, AcqAssumptions, AcqEvidence, AcqExplain,
    AcqExplainCandidate, AcqHypothesis, AcqResult, AcqThresholdProvenance, Hertz, SamplesFrame,
    SatId,
};
use num_complex::Complex;
use rustfft::{num_traits::Zero, FftPlanner};

use crate::engine::receiver_config::ReceiverPipelineConfig;
use crate::engine::runtime::{ReceiverRuntime, TraceRecord};
use bijux_gnss_signal::api::samples_per_code;
use bijux_gnss_signal::api::Nco;
use bijux_gnss_signal::api::{generate_ca_code, Prn};

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

    /// Perform satellite acquisition on a 1 ms buffer using FFT-based circular correlation.
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
        let results =
            self.run_fft_topn_internal(frame, sats, top_n, coherent_ms, noncoherent, true);
        results
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
        if frame.len() < required {
            return AcquisitionRun { results: Vec::new(), explains: Vec::new() };
        }

        let mut planner = FftPlanner::<f32>::new();
        let fft = planner.plan_fft_forward(samples_per_code);
        let ifft = planner.plan_fft_inverse(samples_per_code);

        self.with_stats(|stats| {
            stats.sat_count += sats.len() as u64;
            stats.doppler_bins += (doppler_bin_count(self.doppler_search_hz, self.doppler_step_hz)
                * sats.len() as u64) as u64;
            stats.code_search_bins += (samples_per_code * sats.len()) as u64;
        });

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

        let mut results = Vec::new();
        let mut explains = Vec::new();
        for &sat in sats {
            self.runtime.trace.record(TraceRecord {
                name: "acquisition_sat_start",
                fields: vec![
                    ("constellation", format!("{:?}", sat.constellation)),
                    ("prn", sat.prn.to_string()),
                ],
            });

            let threshold_provenance = AcqThresholdProvenance {
                coherent_ms,
                noncoherent,
                doppler_search_hz: self.doppler_search_hz,
                doppler_step_hz: self.doppler_step_hz,
                peak_mean_threshold: self.config.acquisition_peak_mean_threshold,
                peak_second_threshold: self.config.acquisition_peak_second_threshold,
            };
            let code_fft =
                self.code_fft(sat, samples_per_code, coherent_ms, noncoherent, fft.as_ref());
            let mut candidates = Vec::new();

            let mut doppler = -self.doppler_search_hz;
            while doppler <= self.doppler_search_hz {
                let carrier = self.config.intermediate_freq_hz + doppler as f64;
                let mut noncoherent_acc = vec![0.0f32; samples_per_code];

                for nc in 0..noncoherent {
                    let mut coherent_corr: Vec<Complex<f32>> =
                        vec![Complex::zero(); samples_per_code];
                    for c in 0..coherent_ms {
                        let offset_ms = (nc * coherent_ms + c) as usize;
                        let start = offset_ms * samples_per_code;
                        let end = start + samples_per_code;
                        let block = &frame.iq[start..end];

                        let mut mixed = vec![Complex::zero(); samples_per_code];
                        let mut nco = Nco::new(-carrier, self.config.sampling_freq_hz);
                        for (i, sample) in block.iter().enumerate() {
                            let (sin, cos) = nco.next_sin_cos();
                            let rot = Complex::new(cos as f32, -sin as f32);
                            mixed[i] = *sample * rot;
                        }

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
                } else {
                    candidate.hypothesis = AcqHypothesis::Deferred;
                    candidate.explain_selection_reason = Some("not_selected".to_string());
                }
            }

            let best = &candidates[0];
            if emit_explanations {
                let selected_reason = match best.hypothesis {
                    AcqHypothesis::Accepted => "accepted_by_ratio_thresholds",
                    AcqHypothesis::Ambiguous => "ambiguous_ratio_thresholds",
                    AcqHypothesis::Rejected => "rejected_peak_mean_threshold",
                    AcqHypothesis::Deferred => "deferred",
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
        let local_code = upsample_code(&code, samples_per_code);
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
    let search = search_hz.abs() as u64;
    let step = step_hz.abs() as u64;
    (search / step).saturating_mul(2).saturating_add(1)
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

#[cfg(test)]
mod tests {
    use super::*;

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

fn upsample_code(code: &[i8], samples_per_code: usize) -> Vec<f32> {
    let chips = code.len();
    let samples_per_chip = samples_per_code as f64 / chips as f64;
    let mut out = vec![0.0f32; samples_per_code];
    for (i, value) in out.iter_mut().enumerate() {
        let chip_index = (i as f64 / samples_per_chip).floor() as usize;
        *value = code[chip_index] as f32;
    }
    out
}

fn ca_code_or_default(prn: u8) -> Vec<i8> {
    match generate_ca_code(Prn(prn)) {
        Ok(code) => code,
        Err(_) => vec![1; 1023],
    }
}
