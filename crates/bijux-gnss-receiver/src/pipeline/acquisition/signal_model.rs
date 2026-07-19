use bijux_gnss_core::api::{AcqRequest, SatId, SignalBand, SignalCode};
use bijux_gnss_signal::api::{AcquisitionSignalModel, SignalError};

use crate::engine::receiver_config::ReceiverPipelineConfig;
use crate::engine::signal_selection::default_signal_code_for_band;
use crate::pipeline::doppler::carrier_hz_from_doppler_hz;

#[cfg(test)]
pub(super) fn fallback_acquisition_signal_model(
    config: &ReceiverPipelineConfig,
    sat: SatId,
) -> Result<AcquisitionSignalModel, SignalError> {
    AcquisitionSignalModel::gps_l1_ca_or_ones(
        sat.prn,
        config.code_freq_basis_hz,
        config.code_length,
        bijux_gnss_core::api::GPS_L1_CA_CARRIER_HZ,
    )
}

pub(super) fn resolved_signal_code(
    sat: SatId,
    signal_band: SignalBand,
    signal_code: SignalCode,
) -> SignalCode {
    if signal_code != SignalCode::Unknown {
        return signal_code;
    }
    default_signal_code_for_band(sat.constellation, signal_band)
}

pub(super) fn resolved_request_signal_code(request: AcqRequest) -> SignalCode {
    resolved_signal_code(request.sat, request.signal_band, request.signal_code)
}

pub(super) fn unsupported_acquisition_signal_error(
    sat: SatId,
    signal_band: SignalBand,
    signal_code: SignalCode,
) -> SignalError {
    SignalError::UnsupportedSignalDefinition {
        constellation: sat.constellation,
        signal_band,
        signal_code: resolved_signal_code(sat, signal_band, signal_code),
    }
}

#[cfg(test)]
pub(super) fn acquisition_signal_model_for_sat(
    config: &ReceiverPipelineConfig,
    sat: SatId,
    signal_band: SignalBand,
    signal_code: SignalCode,
    glonass_frequency_channel: Option<bijux_gnss_core::api::GlonassFrequencyChannel>,
) -> AcquisitionSignalModel {
    acquisition_signal_model_for_request(
        config,
        AcqRequest {
            sat,
            glonass_frequency_channel,
            signal_band,
            signal_code,
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
        },
    )
    .unwrap_or_else(|_| {
        fallback_acquisition_signal_model(config, sat)
            .expect("fallback acquisition signal model must be constructible")
    })
}

pub(super) fn acquisition_signal_model_for_request(
    _config: &ReceiverPipelineConfig,
    request: AcqRequest,
) -> Result<AcquisitionSignalModel, SignalError> {
    match AcquisitionSignalModel::for_sat_signal(
        request.sat,
        Some(request.signal_band),
        resolved_request_signal_code(request),
        request.glonass_frequency_channel,
    )? {
        Some(model) => Ok(model),
        None => Err(unsupported_acquisition_signal_error(
            request.sat,
            request.signal_band,
            request.signal_code,
        )),
    }
}

pub(super) fn request_search_center_hz(
    signal_model: &AcquisitionSignalModel,
    intermediate_freq_hz: f64,
    request: AcqRequest,
) -> f64 {
    carrier_hz_from_doppler_hz(
        signal_model.search_center_hz(intermediate_freq_hz),
        request.doppler_center_hz,
    )
}
