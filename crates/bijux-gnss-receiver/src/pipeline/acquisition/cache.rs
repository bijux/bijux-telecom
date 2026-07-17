use std::collections::HashMap;

use bijux_gnss_core::api::{SatId, SignalBand, SignalCode, SignalComponentRole};
use bijux_gnss_signal::api::AcquisitionSignalModel;
use num_complex::Complex;

use crate::engine::receiver_config::ReceiverPipelineConfig;
use crate::engine::runtime::TraceRecord;
use crate::pipeline::acquisition_components::AcquisitionComponentPlan;

use super::Acquisition;

pub(super) const ACQUISITION_CACHE_MODEL_VERSION: u32 = 1;
pub(super) const ACQUISITION_CACHE_POLICY_VERSION: u32 = 1;

#[derive(Clone, Copy)]
pub(super) struct CodeFftCacheKeyRequest<'a> {
    pub(super) config: &'a ReceiverPipelineConfig,
    pub(super) model: &'a AcquisitionSignalModel,
    pub(super) component_role: SignalComponentRole,
    pub(super) sat: SatId,
    pub(super) signal_code: SignalCode,
    pub(super) samples_per_code: usize,
    pub(super) doppler_search_hz: i32,
    pub(super) doppler_step_hz: i32,
}

#[derive(Clone, Copy)]
pub(super) struct CodeFftRequest<'a> {
    pub(super) signal_model: &'a AcquisitionSignalModel,
    pub(super) component: &'a AcquisitionComponentPlan,
    pub(super) sat: SatId,
    pub(super) signal_code: SignalCode,
    pub(super) samples_per_code: usize,
    pub(super) fft: &'a dyn rustfft::Fft<f32>,
}

#[derive(Debug, Clone, Copy, Hash, PartialEq, Eq)]
pub(super) struct CodeFftCacheKey {
    sat: SatId,
    signal_band: SignalBand,
    signal_code: SignalCode,
    component_role_key: u8,
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
    pub(super) fn from_runtime(request: CodeFftCacheKeyRequest<'_>) -> Self {
        Self {
            sat: request.sat,
            signal_band: request.model.signal_band,
            signal_code: request.signal_code,
            component_role_key: signal_component_role_key(request.component_role),
            samples_per_code: request.samples_per_code,
            sampling_hz_bits: request.config.sampling_freq_hz.to_bits(),
            if_hz_bits: request.config.intermediate_freq_hz.to_bits(),
            code_hz_bits: request.model.code_rate_hz.to_bits(),
            code_length: request.model.code_length,
            doppler_search_hz: request.doppler_search_hz,
            doppler_step_hz: request.doppler_step_hz,
            model_version: ACQUISITION_CACHE_MODEL_VERSION,
            policy_version: ACQUISITION_CACHE_POLICY_VERSION,
        }
    }

    pub(super) fn matches_signal_period(self, sat: SatId, samples_per_code: usize) -> bool {
        self.sat == sat && self.samples_per_code == samples_per_code
    }
}

fn signal_component_role_key(role: SignalComponentRole) -> u8 {
    match role {
        SignalComponentRole::Data => 0,
        SignalComponentRole::Pilot => 1,
    }
}

#[derive(Debug, Clone, Copy)]
pub(super) enum CacheMissReason {
    ColdStart,
    IncompatibleAssumptions,
}

impl CacheMissReason {
    pub(super) fn as_str(self) -> &'static str {
        match self {
            CacheMissReason::ColdStart => "cold_start",
            CacheMissReason::IncompatibleAssumptions => "incompatible_assumptions",
        }
    }
}

pub(super) type CodeFftCache = HashMap<CodeFftCacheKey, Vec<Complex<f32>>>;

impl Acquisition {
    pub(super) fn code_fft(&self, request: CodeFftRequest<'_>) -> Vec<Complex<f32>> {
        let key = CodeFftCacheKey::from_runtime(CodeFftCacheKeyRequest {
            config: &self.config,
            model: request.signal_model,
            component_role: request.component.role,
            sat: request.sat,
            signal_code: request.signal_code,
            samples_per_code: request.samples_per_code,
            doppler_search_hz: self.doppler_search_hz,
            doppler_step_hz: self.doppler_step_hz,
        });
        if let Some(cached) = self.cache.lock().ok().and_then(|m| m.get(&key).cloned()) {
            self.with_stats(|stats| {
                stats.cache_hits = stats.cache_hits.saturating_add(1);
            });
            self.runtime.trace.record(TraceRecord {
                name: "acquisition_code_fft_cache_hit",
                fields: vec![
                    ("constellation", format!("{:?}", request.sat.constellation)),
                    ("prn", request.sat.prn.to_string()),
                    ("signal_band", format!("{:?}", request.signal_model.signal_band)),
                    ("signal_code", format!("{:?}", request.signal_code)),
                    ("component_role", format!("{:?}", request.component.role)),
                    ("samples_per_code", request.samples_per_code.to_string()),
                ],
            });
            return cached;
        }

        let miss_reason = self.cache.lock().ok().map_or(CacheMissReason::ColdStart, |cache| {
            let has_same_satellite = cache
                .keys()
                .any(|cached| cached.matches_signal_period(request.sat, request.samples_per_code));
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
                ("constellation", format!("{:?}", request.sat.constellation)),
                ("prn", request.sat.prn.to_string()),
                ("signal_band", format!("{:?}", request.signal_model.signal_band)),
                ("signal_code", format!("{:?}", request.signal_code)),
                ("component_role", format!("{:?}", request.component.role)),
                ("samples_per_code", request.samples_per_code.to_string()),
                ("reason", miss_reason.as_str().to_string()),
                ("model_version", ACQUISITION_CACHE_MODEL_VERSION.to_string()),
                ("policy_version", ACQUISITION_CACHE_POLICY_VERSION.to_string()),
            ],
        });
        let local_code = request
            .component
            .sample_local_code_period(self.config.sampling_freq_hz, request.samples_per_code)
            .unwrap_or_else(|_| vec![1.0; request.samples_per_code]);
        let mut code_fft: Vec<Complex<f32>> =
            local_code.iter().map(|&x| Complex::new(x, 0.0)).collect();
        request.fft.process(&mut code_fft);
        if let Ok(mut cache) = self.cache.lock() {
            cache.insert(key, code_fft.clone());
        }
        code_fft
    }
}
