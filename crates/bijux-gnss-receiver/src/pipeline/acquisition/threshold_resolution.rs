use std::collections::HashMap;

use bijux_gnss_core::api::{
    AcqRequest, AcqThresholdProvenance, GlonassFrequencyChannel, SatId, SignalBand, SignalCode,
};

use crate::engine::receiver_config::{AcquisitionThresholdMode, ReceiverPipelineConfig};

#[derive(Debug, Clone)]
pub(super) struct ResolvedAcquisitionThresholds {
    pub(super) peak_mean_threshold: f32,
    pub(super) peak_second_threshold: f32,
    pub(super) provenance: AcqThresholdProvenance,
}

#[derive(Debug, Clone, Copy, Hash, PartialEq, Eq)]
pub(super) struct AcquisitionThresholdCacheKey {
    sat: SatId,
    signal_band: SignalBand,
    signal_code: SignalCode,
    glonass_frequency_channel: Option<GlonassFrequencyChannel>,
    coherent_ms: u32,
    noncoherent: u32,
    doppler_search_hz: i32,
    doppler_step_hz: i32,
    samples_per_code: usize,
}

impl AcquisitionThresholdCacheKey {
    pub(super) fn from_request(
        request: AcqRequest,
        signal_band: SignalBand,
        signal_code: SignalCode,
        samples_per_code: usize,
    ) -> Self {
        Self {
            sat: request.sat,
            signal_band,
            signal_code,
            glonass_frequency_channel: request.glonass_frequency_channel,
            coherent_ms: request.coherent_ms,
            noncoherent: request.noncoherent,
            doppler_search_hz: request.doppler_search_hz,
            doppler_step_hz: request.doppler_step_hz.max(1),
            samples_per_code,
        }
    }
}

pub(super) type ThresholdResolutionCache =
    HashMap<AcquisitionThresholdCacheKey, ResolvedAcquisitionThresholds>;

pub(super) fn threshold_provenance_for_request(
    config: &ReceiverPipelineConfig,
    request: AcqRequest,
) -> AcqThresholdProvenance {
    let threshold_policy = &config.acquisition_threshold_policy;
    let (mode, false_alarm_probability, calibration_trial_count, calibration_confidence_level) =
        match threshold_policy.mode {
            AcquisitionThresholdMode::FixedRatio => ("fixed_ratio", None, None, None),
            AcquisitionThresholdMode::CalibratedFalseAlarm => (
                "calibrated_false_alarm",
                Some(threshold_policy.false_alarm_probability),
                Some(threshold_policy.calibration_trial_count),
                Some(threshold_policy.confidence_level),
            ),
        };
    AcqThresholdProvenance {
        mode: mode.to_string(),
        coherent_ms: request.coherent_ms,
        noncoherent: request.noncoherent,
        doppler_search_hz: request.doppler_search_hz,
        doppler_step_hz: request.doppler_step_hz.max(1),
        doppler_rate_search_hz_per_s: request.doppler_rate_search_hz_per_s.max(0),
        doppler_rate_step_hz_per_s: request.doppler_rate_step_hz_per_s.max(1),
        peak_mean_threshold: config.acquisition_peak_mean_threshold,
        peak_second_threshold: config.acquisition_peak_second_threshold,
        false_alarm_probability,
        calibration_trial_count,
        calibration_confidence_level,
        calibration_false_alarm_rate: None,
        calibration_false_alarm_interval_low: None,
        calibration_false_alarm_interval_high: None,
    }
}
