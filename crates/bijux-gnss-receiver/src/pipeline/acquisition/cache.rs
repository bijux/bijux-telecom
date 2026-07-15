use std::collections::HashMap;

use bijux_gnss_core::api::{SatId, SignalBand, SignalCode, SignalComponentRole};
use bijux_gnss_signal::api::AcquisitionSignalModel;
use num_complex::Complex;

use crate::engine::receiver_config::ReceiverPipelineConfig;

pub(super) const ACQUISITION_CACHE_MODEL_VERSION: u32 = 1;
pub(super) const ACQUISITION_CACHE_POLICY_VERSION: u32 = 1;

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
    pub(super) fn from_runtime(
        config: &ReceiverPipelineConfig,
        model: &AcquisitionSignalModel,
        component_role: SignalComponentRole,
        sat: SatId,
        signal_code: SignalCode,
        samples_per_code: usize,
        doppler_search_hz: i32,
        doppler_step_hz: i32,
    ) -> Self {
        Self {
            sat,
            signal_band: model.signal_band,
            signal_code,
            component_role_key: signal_component_role_key(component_role),
            samples_per_code,
            sampling_hz_bits: config.sampling_freq_hz.to_bits(),
            if_hz_bits: config.intermediate_freq_hz.to_bits(),
            code_hz_bits: model.code_rate_hz.to_bits(),
            code_length: model.code_length,
            doppler_search_hz,
            doppler_step_hz,
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
