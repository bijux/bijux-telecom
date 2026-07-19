use bijux_gnss_core::api::{
    Constellation, SigId, SignalBand, SignalCode, SignalSpec, TrackEpoch, GPS_L1_CA_CARRIER_HZ,
};
use bijux_gnss_signal::api::{
    resolved_signal_registry_entry, samples_per_code, signal_registry, signal_spec_beidou_b1i,
    signal_spec_beidou_b2i, signal_spec_galileo_e1b, signal_spec_galileo_e5a,
    signal_spec_glonass_l1, signal_spec_gps_l1_ca, signal_spec_gps_l2c, signal_spec_gps_l5,
};

use crate::engine::receiver_config::ReceiverPipelineConfig;

#[derive(Debug, Clone)]
pub(super) struct ObservationSignalModel {
    pub(super) signal_id: SigId,
    pub(super) signal: SignalSpec,
    pub(super) code_length: usize,
    pub(super) samples_per_chip: f64,
    pub(super) carrier_reference_hz: f64,
}

pub(super) fn observation_signal_model(
    config: &ReceiverPipelineConfig,
    epoch: &TrackEpoch,
) -> ObservationSignalModel {
    let signal_code = tracked_signal_code(epoch);
    let registry_entry = resolved_signal_registry_entry(
        epoch.sat,
        epoch.signal_band,
        signal_code,
        epoch.glonass_frequency_channel,
    )
    .ok()
    .flatten()
    .or_else(|| signal_registry(epoch.sat.constellation, epoch.signal_band, signal_code));
    let signal = registry_entry.as_ref().map(|entry| entry.spec).unwrap_or_else(|| {
        match (epoch.sat.constellation, epoch.signal_band, epoch.glonass_frequency_channel) {
            (Constellation::Glonass, SignalBand::L1, Some(channel)) => {
                signal_spec_glonass_l1(channel)
            }
            (Constellation::Gps, SignalBand::L1, _) => signal_spec_gps_l1_ca(),
            (Constellation::Gps, SignalBand::L2, _) => signal_spec_gps_l2c(),
            (Constellation::Gps, SignalBand::L5, _) => signal_spec_gps_l5(),
            (Constellation::Galileo, SignalBand::E1, _) => signal_spec_galileo_e1b(),
            (Constellation::Galileo, SignalBand::E5, _) => signal_spec_galileo_e5a(),
            (Constellation::Beidou, SignalBand::B1, _) => signal_spec_beidou_b1i(),
            (Constellation::Beidou, SignalBand::B2, _) => signal_spec_beidou_b2i(),
            _ => signal_spec_gps_l1_ca(),
        }
    });
    let component = registry_entry.as_ref().and_then(|entry| entry.default_component().copied());
    let code_length = component
        .map(|component| component.primary_code_chips as usize)
        .or_else(|| {
            registry_entry
                .as_ref()
                .and_then(|entry| entry.code_length)
                .map(|length| length as usize)
        })
        .unwrap_or_else(|| fallback_code_length(epoch));
    let code_rate_hz =
        component.map(|component| component.primary_code_rate_hz).unwrap_or(signal.code_rate_hz);
    let samples_per_code = samples_per_code(config.sampling_freq_hz, code_rate_hz, code_length);
    ObservationSignalModel {
        signal_id: SigId { sat: epoch.sat, band: signal.band, code: signal.code },
        signal,
        code_length,
        samples_per_chip: samples_per_code as f64 / code_length as f64,
        carrier_reference_hz: tracked_signal_center_hz(config.intermediate_freq_hz, signal),
    }
}

pub(crate) fn supports_observation_signal(
    constellation: Constellation,
    signal_band: SignalBand,
    signal_code: SignalCode,
) -> bool {
    matches!(
        (constellation, signal_band, signal_code),
        (Constellation::Gps, SignalBand::L1, SignalCode::Ca)
            | (Constellation::Gps, SignalBand::L2, SignalCode::L2C)
            | (Constellation::Gps, SignalBand::L5, SignalCode::L5I)
            | (Constellation::Gps, SignalBand::L5, SignalCode::L5Q)
            | (Constellation::Galileo, SignalBand::E1, SignalCode::E1B)
            | (Constellation::Galileo, SignalBand::E5, SignalCode::E5a)
            | (Constellation::Galileo, SignalBand::E5, SignalCode::E5b)
            | (Constellation::Beidou, SignalBand::B1, SignalCode::B1I)
            | (Constellation::Beidou, SignalBand::B2, SignalCode::B2I)
            | (Constellation::Glonass, SignalBand::L1, SignalCode::Unknown)
    )
}

pub(crate) fn tracked_signal_code_for_band(
    constellation: Constellation,
    signal_band: SignalBand,
) -> Option<SignalCode> {
    match (constellation, signal_band) {
        (Constellation::Gps, SignalBand::L1) => Some(SignalCode::Ca),
        (Constellation::Gps, SignalBand::L2) => Some(SignalCode::L2C),
        (Constellation::Gps, SignalBand::L5) => Some(SignalCode::L5I),
        (Constellation::Galileo, SignalBand::E1) => Some(SignalCode::E1B),
        (Constellation::Galileo, SignalBand::E5) => Some(SignalCode::E5a),
        (Constellation::Beidou, SignalBand::B1) => Some(SignalCode::B1I),
        (Constellation::Beidou, SignalBand::B2) => Some(SignalCode::B2I),
        (Constellation::Glonass, SignalBand::L1) => Some(SignalCode::Unknown),
        _ => None,
    }
}

fn tracked_signal_code(epoch: &TrackEpoch) -> SignalCode {
    if epoch.signal_code != SignalCode::Unknown {
        return epoch.signal_code;
    }
    tracked_signal_code_for_band(epoch.sat.constellation, epoch.signal_band)
        .unwrap_or(SignalCode::Unknown)
}

fn fallback_code_length(epoch: &TrackEpoch) -> usize {
    match (epoch.sat.constellation, epoch.signal_band) {
        (Constellation::Gps, SignalBand::L1) => 1023,
        (Constellation::Gps, SignalBand::L2) => 10230,
        (Constellation::Gps, SignalBand::L5) => 10230,
        (Constellation::Galileo, SignalBand::E1) => 4092,
        (Constellation::Galileo, SignalBand::E5) => 10230,
        (Constellation::Beidou, SignalBand::B1) => 2046,
        (Constellation::Beidou, SignalBand::B2) => 2046,
        (Constellation::Glonass, SignalBand::L1) => 511,
        _ => 1023,
    }
}

pub(super) fn tracked_signal_center_hz(intermediate_freq_hz: f64, signal: SignalSpec) -> f64 {
    intermediate_freq_hz + (signal.carrier_hz.value() - GPS_L1_CA_CARRIER_HZ.value())
}
