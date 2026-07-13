#![allow(missing_docs)]

use bijux_gnss_core::api::{
    default_acquisition_signal, Constellation, SatId, SignalBand, SignalCode,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct SignalExecutionSupport {
    pub acquisition: bool,
    pub tracking: bool,
    pub data_decoding: bool,
    pub observations: bool,
    pub positioning: bool,
}

pub(crate) fn signal_execution_support(
    constellation: Constellation,
    band: SignalBand,
    code: SignalCode,
) -> SignalExecutionSupport {
    SignalExecutionSupport {
        acquisition: supports_acquisition_signal(constellation, band, code),
        tracking: supports_tracking_signal(constellation, band, code),
        data_decoding: supports_data_decoding_signal(constellation, band, code),
        observations: supports_observation_signal(constellation, band, code),
        positioning: supports_positioning_signal(constellation, band, code),
    }
}

pub(crate) fn supports_acquisition_signal(
    constellation: Constellation,
    band: SignalBand,
    code: SignalCode,
) -> bool {
    default_acquisition_signal(constellation)
        .map(|signal| signal.spec.band == band && signal.spec.code == code)
        .unwrap_or(false)
}

pub(crate) fn supports_tracking_signal(
    constellation: Constellation,
    band: SignalBand,
    code: SignalCode,
) -> bool {
    crate::pipeline::observations::tracked_signal_code_for_band(constellation, band) == Some(code)
        && crate::pipeline::tracking::supports_tracking_signal(sample_sat(constellation), band)
}

pub(crate) fn supports_data_decoding_signal(
    constellation: Constellation,
    band: SignalBand,
    code: SignalCode,
) -> bool {
    matches!((constellation, band, code), (Constellation::Gps, SignalBand::L1, SignalCode::Ca))
}

pub(crate) fn supports_observation_signal(
    constellation: Constellation,
    band: SignalBand,
    code: SignalCode,
) -> bool {
    crate::pipeline::observations::supports_observation_signal(constellation, band, code)
}

#[cfg(feature = "nav")]
pub(crate) fn supports_positioning_signal(
    constellation: Constellation,
    band: SignalBand,
    code: SignalCode,
) -> bool {
    crate::pipeline::navigation::supports_positioning_signal(constellation, band, code)
}

#[cfg(not(feature = "nav"))]
pub(crate) fn supports_positioning_signal(
    _constellation: Constellation,
    _band: SignalBand,
    _code: SignalCode,
) -> bool {
    false
}

fn sample_sat(constellation: Constellation) -> SatId {
    SatId { constellation, prn: representative_prn(constellation) }
}

fn representative_prn(constellation: Constellation) -> u8 {
    match constellation {
        Constellation::Gps | Constellation::Galileo | Constellation::Glonass | Constellation::Beidou => 1,
        Constellation::Unknown => 0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gps_l1_reports_full_execution_support() {
        let support = signal_execution_support(Constellation::Gps, SignalBand::L1, SignalCode::Ca);

        assert_eq!(
            support,
            SignalExecutionSupport {
                acquisition: true,
                tracking: true,
                data_decoding: true,
                observations: true,
                positioning: cfg!(feature = "nav"),
            }
        );
    }

    #[test]
    fn observation_only_registered_signals_remain_non_tracking_capabilities() {
        let gps_l2c =
            signal_execution_support(Constellation::Gps, SignalBand::L2, SignalCode::L2C);
        assert_eq!(
            gps_l2c,
            SignalExecutionSupport {
                acquisition: false,
                tracking: false,
                data_decoding: false,
                observations: true,
                positioning: false,
            }
        );

        let galileo_e5 =
            signal_execution_support(Constellation::Galileo, SignalBand::E5, SignalCode::E5a);
        assert_eq!(
            galileo_e5,
            SignalExecutionSupport {
                acquisition: false,
                tracking: false,
                data_decoding: false,
                observations: true,
                positioning: false,
            }
        );
    }

    #[test]
    fn tracking_requires_supported_band_and_supported_signal_identity() {
        assert!(supports_tracking_signal(
            Constellation::Galileo,
            SignalBand::E1,
            SignalCode::E1B
        ));
        assert!(!supports_tracking_signal(
            Constellation::Galileo,
            SignalBand::E1,
            SignalCode::E1C
        ));
        assert!(!supports_tracking_signal(
            Constellation::Gps,
            SignalBand::L5,
            SignalCode::Unknown
        ));
    }
}
