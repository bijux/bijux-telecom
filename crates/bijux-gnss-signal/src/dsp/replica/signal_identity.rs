use crate::catalog::{default_acquisition_signal, resolved_signal_registry_entry};
use crate::error::SignalError;
use bijux_gnss_core::api::{
    Constellation, FreqHz, GlonassFrequencyChannel, SatId, SignalBand, SignalCode,
};

/// Carrier frequency for the default synthesized signal of a satellite.
pub fn default_signal_carrier_hz(
    sat: SatId,
    glonass_frequency_channel: Option<GlonassFrequencyChannel>,
) -> Result<Option<FreqHz>, SignalError> {
    default_signal_carrier_hz_for_band(
        sat,
        default_acquisition_signal(sat.constellation).map(|signal| signal.spec.band),
        glonass_frequency_channel,
    )
}

/// Carrier frequency for one explicit synthesized satellite signal band.
pub fn default_signal_carrier_hz_for_band(
    sat: SatId,
    signal_band: Option<SignalBand>,
    glonass_frequency_channel: Option<GlonassFrequencyChannel>,
) -> Result<Option<FreqHz>, SignalError> {
    let Some(signal_band) = signal_band else {
        return Ok(None);
    };
    default_signal_carrier_hz_for_signal(
        sat,
        Some(signal_band),
        default_signal_code_for_band(sat.constellation, signal_band),
        glonass_frequency_channel,
    )
}

/// Carrier frequency for one explicit synthesized satellite signal identity.
pub fn default_signal_carrier_hz_for_signal(
    sat: SatId,
    signal_band: Option<SignalBand>,
    signal_code: SignalCode,
    glonass_frequency_channel: Option<GlonassFrequencyChannel>,
) -> Result<Option<FreqHz>, SignalError> {
    let Some(signal_band) = signal_band else {
        return Ok(None);
    };

    Ok(resolved_signal_registry_entry(sat, signal_band, signal_code, glonass_frequency_channel)?
        .map(|entry| entry.spec.carrier_hz))
}

pub(crate) fn default_signal_code_for_band(
    constellation: Constellation,
    signal_band: SignalBand,
) -> SignalCode {
    match (constellation, signal_band) {
        (Constellation::Gps, SignalBand::L1) => SignalCode::Ca,
        (Constellation::Gps, SignalBand::L2) => SignalCode::L2C,
        (Constellation::Gps, SignalBand::L5) => SignalCode::L5I,
        (Constellation::Galileo, SignalBand::E1) => SignalCode::E1B,
        (Constellation::Galileo, SignalBand::E5) => SignalCode::E5a,
        (Constellation::Beidou, SignalBand::B1) => SignalCode::B1I,
        (Constellation::Beidou, SignalBand::B2) => SignalCode::B2I,
        (Constellation::Glonass, SignalBand::L1) => SignalCode::Unknown,
        _ => SignalCode::Unknown,
    }
}
