use bijux_gnss_core::api::{ObsSatellite, SatId, SignalBand};
use bijux_gnss_signal::api::supported_dual_frequency_band_pairs_for_constellation;

const TRANSMIT_TIME_COMPATIBILITY_TOLERANCE_S: f64 = 1.0e-3;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum DualFrequencyPairIssue {
    MissingFrequency,
    UnsupportedBandPair,
    ConstellationMismatch,
    TimeSystemMismatch,
}

pub(crate) fn dual_frequency_pair_issue(
    sat: SatId,
    band_1: SignalBand,
    band_2: SignalBand,
    first: Option<&ObsSatellite>,
    second: Option<&ObsSatellite>,
) -> Option<DualFrequencyPairIssue> {
    if has_constellation_mismatch(sat, first, second) {
        return Some(DualFrequencyPairIssue::ConstellationMismatch);
    }

    if !constellation_supports_pair(sat, band_1, band_2) {
        return Some(DualFrequencyPairIssue::UnsupportedBandPair);
    }

    if let (Some(first), Some(second)) = (first, second) {
        if has_time_system_mismatch(first, second) {
            return Some(DualFrequencyPairIssue::TimeSystemMismatch);
        }
        return None;
    }

    Some(DualFrequencyPairIssue::MissingFrequency)
}

fn has_constellation_mismatch(
    sat: SatId,
    first: Option<&ObsSatellite>,
    second: Option<&ObsSatellite>,
) -> bool {
    [first, second].into_iter().flatten().any(|observation| {
        observation.signal_id.sat != sat
            || observation.metadata.signal.constellation != sat.constellation
    })
}

fn constellation_supports_pair(sat: SatId, band_1: SignalBand, band_2: SignalBand) -> bool {
    supported_dual_frequency_band_pairs_for_constellation(sat.constellation).iter().any(
        |&(supported_band_1, supported_band_2)| {
            supported_band_1 == band_1 && supported_band_2 == band_2
        },
    )
}

fn has_time_system_mismatch(first: &ObsSatellite, second: &ObsSatellite) -> bool {
    let (Some(first_timing), Some(second_timing)) = (first.timing, second.timing) else {
        return false;
    };

    first_timing.transmit_gps_time.week != second_timing.transmit_gps_time.week
        || !first_timing.transmit_gps_time.tow_s.is_finite()
        || !second_timing.transmit_gps_time.tow_s.is_finite()
        || (first_timing.transmit_gps_time.tow_s - second_timing.transmit_gps_time.tow_s).abs()
            > TRANSMIT_TIME_COMPATIBILITY_TOLERANCE_S
}

#[cfg(test)]
mod tests {
    use super::{dual_frequency_pair_issue, DualFrequencyPairIssue};
    use bijux_gnss_core::api::{
        Constellation, Cycles, GpsTime, Hertz, LockFlags, Meters, ObsMetadata, ObsSatellite,
        ObsSignalTiming, ObservationStatus, SatId, Seconds, SigId, SignalBand, SignalCode,
        SignalSpec,
    };
    use bijux_gnss_signal::api::{
        signal_spec_galileo_e1b, signal_spec_gps_l1_ca, signal_spec_gps_l2_py,
    };

    fn satellite(
        sat: SatId,
        band: SignalBand,
        code: SignalCode,
        signal: SignalSpec,
    ) -> ObsSatellite {
        ObsSatellite {
            signal_id: SigId { sat, band, code },
            pseudorange_m: Meters(20_200_000.0),
            pseudorange_var_m2: 1.0,
            carrier_phase_cycles: Cycles(1000.0),
            carrier_phase_var_cycles2: 0.01,
            doppler_hz: Hertz(0.0),
            doppler_var_hz2: 1.0,
            cn0_dbhz: 45.0,
            lock_flags: LockFlags {
                code_lock: true,
                carrier_lock: true,
                bit_lock: false,
                cycle_slip: false,
            },
            multipath_suspect: false,
            observation_status: ObservationStatus::Accepted,
            observation_reject_reasons: Vec::new(),
            elevation_deg: None,
            azimuth_deg: None,
            weight: None,
            timing: None,
            error_model: None,
            metadata: ObsMetadata {
                tracking_mode: "test".to_string(),
                integration_ms: 1,
                lock_quality: 45.0,
                smoothing_window: 0,
                smoothing_age: 0,
                smoothing_resets: 0,
                signal,
                ..ObsMetadata::default()
            },
        }
    }

    #[test]
    fn rejects_band_pairs_not_supported_by_constellation() {
        let sat = SatId { constellation: Constellation::Gps, prn: 11 };
        let first = satellite(sat, SignalBand::L1, SignalCode::Ca, signal_spec_gps_l1_ca());
        let second = satellite(sat, SignalBand::L2, SignalCode::Py, signal_spec_gps_l2_py());

        let issue = dual_frequency_pair_issue(
            sat,
            SignalBand::L2,
            SignalBand::L5,
            Some(&first),
            Some(&second),
        );

        assert_eq!(issue, Some(DualFrequencyPairIssue::UnsupportedBandPair));
    }

    #[test]
    fn rejects_constellation_mismatch_between_satellite_and_signal_metadata() {
        let sat = SatId { constellation: Constellation::Gps, prn: 11 };
        let first = satellite(sat, SignalBand::L1, SignalCode::Ca, signal_spec_gps_l1_ca());
        let second = satellite(sat, SignalBand::L2, SignalCode::Py, signal_spec_galileo_e1b());

        let issue = dual_frequency_pair_issue(
            sat,
            SignalBand::L1,
            SignalBand::L2,
            Some(&first),
            Some(&second),
        );

        assert_eq!(issue, Some(DualFrequencyPairIssue::ConstellationMismatch));
    }

    #[test]
    fn rejects_transmit_time_mismatch_when_bands_do_not_share_a_common_reference() {
        let sat = SatId { constellation: Constellation::Gps, prn: 11 };
        let mut first = satellite(sat, SignalBand::L1, SignalCode::Ca, signal_spec_gps_l1_ca());
        let mut second = satellite(sat, SignalBand::L2, SignalCode::Py, signal_spec_gps_l2_py());
        first.timing = Some(ObsSignalTiming {
            signal_travel_time_s: Seconds(0.075),
            transmit_gps_time: GpsTime { week: 2200, tow_s: 345_600.0 },
        });
        second.timing = Some(ObsSignalTiming {
            signal_travel_time_s: Seconds(0.075),
            transmit_gps_time: GpsTime { week: 2201, tow_s: 0.0 },
        });

        let issue = dual_frequency_pair_issue(
            sat,
            SignalBand::L1,
            SignalBand::L2,
            Some(&first),
            Some(&second),
        );

        assert_eq!(issue, Some(DualFrequencyPairIssue::TimeSystemMismatch));
    }
}
