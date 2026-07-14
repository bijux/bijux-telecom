#![allow(missing_docs)]

use crate::engine::acquisition_catalog::default_acquisition_satellites;
use crate::engine::receiver_config::ReceiverPipelineConfig;
use bijux_gnss_core::api::{
    default_signal_band_for_constellation, Constellation, SatId, SignalBand, SignalCode,
};
use bijux_gnss_signal::api::{registered_signal_registry_entries, AcquisitionSignalModel};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct AcquisitionSignalIdentity {
    pub signal_band: SignalBand,
    pub signal_code: SignalCode,
}

pub(crate) fn resolved_acquisition_signal_band(
    config: &ReceiverPipelineConfig,
    sat: SatId,
) -> SignalBand {
    resolved_primary_acquisition_signal(config, sat).signal_band
}

pub(crate) fn resolved_primary_acquisition_signal(
    config: &ReceiverPipelineConfig,
    sat: SatId,
) -> AcquisitionSignalIdentity {
    resolved_acquisition_signals(config, sat).into_iter().next().unwrap_or(
        AcquisitionSignalIdentity {
            signal_band: default_signal_band_for_constellation(sat.constellation),
            signal_code: default_signal_code_for_band(
                sat.constellation,
                default_signal_band_for_constellation(sat.constellation),
            ),
        },
    )
}

pub(crate) fn resolved_acquisition_signals(
    config: &ReceiverPipelineConfig,
    sat: SatId,
) -> Vec<AcquisitionSignalIdentity> {
    let mut matches = registered_signal_registry_entries()
        .iter()
        .filter(|entry| entry.spec.constellation == sat.constellation)
        .filter(|entry| {
            acquisition_signal_matches_config(config, sat, entry.spec.band, entry.spec.code)
        })
        .map(|entry| AcquisitionSignalIdentity {
            signal_band: entry.spec.band,
            signal_code: entry.spec.code,
        })
        .collect::<Vec<_>>();

    if matches.is_empty() {
        matches.push(resolved_primary_fallback_signal(sat));
    }

    matches
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
    registered_signal_registry_entries()
        .iter()
        .filter(|entry| entry.spec.constellation == sat.constellation)
        .filter(|entry| entry.spec.band == signal_band)
        .any(|entry| acquisition_signal_matches_config(config, sat, signal_band, entry.spec.code))
}

fn acquisition_signal_matches_config(
    config: &ReceiverPipelineConfig,
    sat: SatId,
    signal_band: SignalBand,
    signal_code: SignalCode,
) -> bool {
    AcquisitionSignalModel::for_sat_signal(sat, Some(signal_band), signal_code, None)
        .ok()
        .flatten()
        .is_some_and(|model| {
            (model.code_rate_hz - config.code_freq_basis_hz).abs() <= f64::EPSILON
                && model.code_length == config.code_length
        })
}

fn resolved_primary_fallback_signal(sat: SatId) -> AcquisitionSignalIdentity {
    let signal_band = default_signal_band_for_constellation(sat.constellation);
    AcquisitionSignalIdentity {
        signal_band,
        signal_code: default_signal_code_for_band(sat.constellation, signal_band),
    }
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
    use super::{
        acquisition_constellation_matches_config, resolved_acquisition_signal_band,
        resolved_acquisition_signals, resolved_primary_acquisition_signal,
        signal_band_matches_config,
    };
    use crate::engine::receiver_config::ReceiverPipelineConfig;
    use bijux_gnss_core::api::{Constellation, SatId, SignalBand, SignalCode};

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
    fn resolved_acquisition_signals_include_both_gps_l5_components() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 10_230_000.0,
            code_freq_basis_hz: 10_230_000.0,
            code_length: 10_230,
            ..ReceiverPipelineConfig::default()
        };
        let sat = SatId { constellation: Constellation::Gps, prn: 7 };

        assert_eq!(
            resolved_acquisition_signals(&config, sat),
            vec![
                super::AcquisitionSignalIdentity {
                    signal_band: SignalBand::L5,
                    signal_code: SignalCode::L5I,
                },
                super::AcquisitionSignalIdentity {
                    signal_band: SignalBand::L5,
                    signal_code: SignalCode::L5Q,
                },
            ]
        );
        assert_eq!(
            resolved_primary_acquisition_signal(&config, sat),
            super::AcquisitionSignalIdentity {
                signal_band: SignalBand::L5,
                signal_code: SignalCode::L5I,
            }
        );
    }

    #[test]
    fn resolved_acquisition_signals_include_both_galileo_e5_components() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 10_230_000.0,
            code_freq_basis_hz: 10_230_000.0,
            code_length: 10_230,
            ..ReceiverPipelineConfig::default()
        };
        let sat = SatId { constellation: Constellation::Galileo, prn: 11 };

        assert_eq!(
            resolved_acquisition_signals(&config, sat),
            vec![
                super::AcquisitionSignalIdentity {
                    signal_band: SignalBand::E5,
                    signal_code: SignalCode::E5a,
                },
                super::AcquisitionSignalIdentity {
                    signal_band: SignalBand::E5,
                    signal_code: SignalCode::E5b,
                },
            ]
        );
        assert_eq!(
            resolved_primary_acquisition_signal(&config, sat),
            super::AcquisitionSignalIdentity {
                signal_band: SignalBand::E5,
                signal_code: SignalCode::E5a,
            }
        );
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

    #[test]
    fn unsupported_registry_band_does_not_claim_acquisition_match() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 511_500.0,
            code_freq_basis_hz: 511_500.0,
            code_length: 10_230,
            ..ReceiverPipelineConfig::default()
        };
        let sat = SatId { constellation: Constellation::Gps, prn: 7 };

        assert_eq!(resolved_acquisition_signal_band(&config, sat), SignalBand::L1);
        assert!(!acquisition_constellation_matches_config(&config, Constellation::Gps));
    }

    #[test]
    fn acquisition_band_matching_follows_executable_search_inventory() {
        let cases = [
            (
                ReceiverPipelineConfig {
                    sampling_freq_hz: 1_023_000.0,
                    code_freq_basis_hz: 1_023_000.0,
                    code_length: 1023,
                    ..ReceiverPipelineConfig::default()
                },
                SatId { constellation: Constellation::Gps, prn: 7 },
                SignalBand::L1,
                true,
            ),
            (
                ReceiverPipelineConfig {
                    sampling_freq_hz: 511_500.0,
                    code_freq_basis_hz: 511_500.0,
                    code_length: 10_230,
                    ..ReceiverPipelineConfig::default()
                },
                SatId { constellation: Constellation::Gps, prn: 7 },
                SignalBand::L2,
                false,
            ),
            (
                ReceiverPipelineConfig {
                    sampling_freq_hz: 10_230_000.0,
                    code_freq_basis_hz: 10_230_000.0,
                    code_length: 10_230,
                    ..ReceiverPipelineConfig::default()
                },
                SatId { constellation: Constellation::Gps, prn: 7 },
                SignalBand::L5,
                true,
            ),
            (
                ReceiverPipelineConfig {
                    sampling_freq_hz: 1_023_000.0,
                    code_freq_basis_hz: 1_023_000.0,
                    code_length: 4092,
                    ..ReceiverPipelineConfig::default()
                },
                SatId { constellation: Constellation::Galileo, prn: 11 },
                SignalBand::E1,
                true,
            ),
            (
                ReceiverPipelineConfig {
                    sampling_freq_hz: 10_230_000.0,
                    code_freq_basis_hz: 10_230_000.0,
                    code_length: 10_230,
                    ..ReceiverPipelineConfig::default()
                },
                SatId { constellation: Constellation::Galileo, prn: 11 },
                SignalBand::E5,
                true,
            ),
            (
                ReceiverPipelineConfig {
                    sampling_freq_hz: 511_000.0,
                    code_freq_basis_hz: 511_000.0,
                    code_length: 511,
                    ..ReceiverPipelineConfig::default()
                },
                SatId { constellation: Constellation::Glonass, prn: 8 },
                SignalBand::L1,
                false,
            ),
            (
                ReceiverPipelineConfig {
                    sampling_freq_hz: 2_046_000.0,
                    code_freq_basis_hz: 2_046_000.0,
                    code_length: 2046,
                    ..ReceiverPipelineConfig::default()
                },
                SatId { constellation: Constellation::Beidou, prn: 11 },
                SignalBand::B1,
                true,
            ),
            (
                ReceiverPipelineConfig {
                    sampling_freq_hz: 2_046_000.0,
                    code_freq_basis_hz: 2_046_000.0,
                    code_length: 2046,
                    ..ReceiverPipelineConfig::default()
                },
                SatId { constellation: Constellation::Beidou, prn: 11 },
                SignalBand::B2,
                true,
            ),
        ];

        for (config, sat, band, expected) in cases {
            assert_eq!(
                signal_band_matches_config(&config, sat, band),
                expected,
                "{sat:?} {band:?}"
            );
        }
    }
}
