#![allow(missing_docs)]

use crate::engine::acquisition_catalog::default_acquisition_satellites;
use crate::engine::receiver_config::ReceiverPipelineConfig;
use bijux_gnss_core::api::{
    default_signal_band_for_constellation, Constellation, SatId, SignalBand, SignalCode,
};
use bijux_gnss_signal::api::default_local_code_model;

pub(crate) fn resolved_acquisition_signal_band(
    config: &ReceiverPipelineConfig,
    sat: SatId,
) -> SignalBand {
    supported_signal_bands(sat.constellation)
        .iter()
        .copied()
        .find(|signal_band| signal_band_matches_config(config, sat, *signal_band))
        .unwrap_or_else(|| default_signal_band_for_constellation(sat.constellation))
}

pub(crate) fn acquisition_constellation_matches_config(
    config: &ReceiverPipelineConfig,
    constellation: Constellation,
) -> bool {
    default_acquisition_satellites(constellation).into_iter().next().is_some_and(|sat| {
        supported_signal_bands(constellation)
            .iter()
            .copied()
            .any(|signal_band| signal_band_matches_config(config, sat, signal_band))
    })
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

fn signal_band_matches_config(
    config: &ReceiverPipelineConfig,
    sat: SatId,
    signal_band: SignalBand,
) -> bool {
    default_local_code_model(sat, signal_band).ok().flatten().is_some_and(|model| {
        (model.code_rate_hz() - config.code_freq_basis_hz).abs() <= f64::EPSILON
            && model.code_length() == config.code_length
    })
}

fn supported_signal_bands(constellation: Constellation) -> &'static [SignalBand] {
    match constellation {
        Constellation::Gps => &[SignalBand::L1, SignalBand::L2, SignalBand::L5],
        Constellation::Galileo => &[SignalBand::E1, SignalBand::E5],
        Constellation::Glonass => &[SignalBand::L1],
        Constellation::Beidou => &[SignalBand::B1, SignalBand::B2],
        Constellation::Unknown => &[],
    }
}

#[cfg(test)]
mod tests {
    use super::{acquisition_constellation_matches_config, resolved_acquisition_signal_band};
    use crate::engine::receiver_config::ReceiverPipelineConfig;
    use bijux_gnss_core::api::{Constellation, SatId, SignalBand};

    #[test]
    fn resolved_acquisition_signal_band_matches_gps_l5_config() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 10_230_000.0,
            code_freq_basis_hz: 10_230_000.0,
            code_length: 10_230,
            ..ReceiverPipelineConfig::default()
        };
        let sat = SatId { constellation: Constellation::Gps, prn: 7 };

        assert_eq!(resolved_acquisition_signal_band(&config, sat), SignalBand::L5);
        assert!(acquisition_constellation_matches_config(&config, Constellation::Gps));
    }

    #[test]
    fn resolved_acquisition_signal_band_matches_beidou_b1_config() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 2_046_000.0,
            code_freq_basis_hz: 2_046_000.0,
            code_length: 2046,
            ..ReceiverPipelineConfig::default()
        };
        let sat = SatId { constellation: Constellation::Beidou, prn: 11 };

        assert_eq!(resolved_acquisition_signal_band(&config, sat), SignalBand::B1);
        assert!(acquisition_constellation_matches_config(&config, Constellation::Beidou));
    }
}
