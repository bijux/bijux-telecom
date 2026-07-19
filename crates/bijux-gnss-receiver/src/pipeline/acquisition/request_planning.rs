use bijux_gnss_core::api::{AcqAssumptions, AcqRequest, SatId};
use bijux_gnss_signal::api::AcquisitionSignalModel;

use crate::engine::receiver_config::ReceiverPipelineConfig;
use crate::engine::signal_selection::{
    default_signal_code_for_band, resolved_acquisition_signal_band,
};
use crate::pipeline::acquisition_assistance::ResolvedAcquisitionSearchBounds;

use super::Acquisition;

impl Acquisition {
    pub(super) fn search_assumptions(
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

    pub(super) fn default_request(
        &self,
        sat: SatId,
        coherent_ms: u32,
        noncoherent: u32,
    ) -> AcqRequest {
        let signal_band = resolved_acquisition_signal_band(&self.config, sat);
        AcqRequest {
            sat,
            glonass_frequency_channel: None,
            signal_band,
            signal_code: default_signal_code_for_band(sat.constellation, signal_band),
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

    pub(super) fn default_requests_for_sats(
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
}

pub(super) fn required_samples_for_request(
    config: &ReceiverPipelineConfig,
    signal_model: &AcquisitionSignalModel,
    coherent_ms: u32,
    noncoherent: u32,
) -> usize {
    let samples_per_code = signal_model.samples_per_code(config.sampling_freq_hz);
    let coherent_periods = signal_model.coherent_periods(coherent_ms).unwrap_or(1).max(1) as usize;
    samples_per_code.saturating_mul(coherent_periods).saturating_mul(noncoherent.max(1) as usize)
}
