#![allow(missing_docs)]

use std::collections::BTreeMap;

use bijux_gnss_core::api::{ObsEpoch, ObsSatellite, SatId, SignalBand};
use bijux_gnss_signal::api::{signal_cycles_to_meters, signal_wavelength_m};

use crate::corrections::combinations::narrow_lane_wavelength_m_from_frequencies;
use crate::corrections::dual_frequency::{dual_frequency_pair_issue, DualFrequencyPairIssue};

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct NarrowLaneObservation {
    pub epoch_idx: u64,
    pub t_rx_s: f64,
    pub sat: SatId,
    pub band_1: SignalBand,
    pub band_2: SignalBand,
    pub f1_hz: f64,
    pub f2_hz: f64,
    pub phase_m: Option<f64>,
    pub variance_m2: Option<f64>,
    pub phase_cycles: Option<f64>,
    pub variance_cycles2: Option<f64>,
    pub narrow_lane_wavelength_m: Option<f64>,
    pub status: String,
    pub reason: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum NarrowLaneStatus {
    Ok,
    MissingFrequency,
    UnsupportedBandPair,
    ConstellationMismatch,
    TimeSystemMismatch,
    CarrierLockInvalid,
    VarianceInvalid,
    FrequencyInvalid,
}

pub fn narrow_lane_from_obs_epochs(
    epochs: &[ObsEpoch],
    band_1: SignalBand,
    band_2: SignalBand,
) -> Vec<NarrowLaneObservation> {
    let mut out = Vec::new();

    for epoch in epochs {
        let mut by_sat: BTreeMap<SatId, Vec<&ObsSatellite>> = BTreeMap::new();
        for sat in &epoch.sats {
            by_sat.entry(sat.signal_id.sat).or_default().push(sat);
        }

        for (sat, sats) in by_sat {
            let first = sats.iter().find(|candidate| candidate.signal_id.band == band_1).copied();
            let second = sats.iter().find(|candidate| candidate.signal_id.band == band_2).copied();
            out.push(narrow_lane_from_pair(
                epoch.epoch_idx,
                epoch.t_rx_s.0,
                sat,
                band_1,
                band_2,
                first,
                second,
            ));
        }
    }

    out
}

pub(crate) fn narrow_lane_from_pair(
    epoch_idx: u64,
    t_rx_s: f64,
    sat: SatId,
    band_1: SignalBand,
    band_2: SignalBand,
    first: Option<&ObsSatellite>,
    second: Option<&ObsSatellite>,
) -> NarrowLaneObservation {
    let f1_hz =
        first.map(|observation| observation.metadata.signal.carrier_hz.value()).unwrap_or(0.0);
    let f2_hz =
        second.map(|observation| observation.metadata.signal.carrier_hz.value()).unwrap_or(0.0);

    let (phase_m, variance_m2, narrow_lane_wavelength_m, status) =
        match dual_frequency_pair_issue(sat, band_1, band_2, first, second) {
            Some(issue) => (None, None, None, narrow_lane_status_from_pair_issue(issue)),
            None => {
                let (Some(first), Some(second)) = (first, second) else {
                    unreachable!("compatible dual-frequency pairs must include both observations");
                };
                evaluate_narrow_lane(first, second)
            }
        };
    let (phase_cycles, variance_cycles2) = match (phase_m, variance_m2, narrow_lane_wavelength_m) {
        (Some(phase_m), Some(variance_m2), Some(narrow_lane_wavelength_m)) => {
            let phase_cycles = phase_m / narrow_lane_wavelength_m;
            let variance_cycles2 = variance_m2 / narrow_lane_wavelength_m.powi(2);
            (Some(phase_cycles), Some(variance_cycles2))
        }
        _ => (None, None),
    };
    let (status_text, reason) = status_reason(status);

    NarrowLaneObservation {
        epoch_idx,
        t_rx_s,
        sat,
        band_1,
        band_2,
        f1_hz,
        f2_hz,
        phase_m,
        variance_m2,
        phase_cycles,
        variance_cycles2,
        narrow_lane_wavelength_m,
        status: status_text,
        reason,
    }
}

fn evaluate_narrow_lane(
    first: &ObsSatellite,
    second: &ObsSatellite,
) -> (Option<f64>, Option<f64>, Option<f64>, NarrowLaneStatus) {
    if !first.lock_flags.carrier_lock || !second.lock_flags.carrier_lock {
        return (None, None, None, NarrowLaneStatus::CarrierLockInvalid);
    }
    if !first.carrier_phase_var_cycles2.is_finite()
        || !second.carrier_phase_var_cycles2.is_finite()
        || first.carrier_phase_var_cycles2 < 0.0
        || second.carrier_phase_var_cycles2 < 0.0
    {
        return (None, None, None, NarrowLaneStatus::VarianceInvalid);
    }

    let f1_hz = first.metadata.signal.carrier_hz.value();
    let f2_hz = second.metadata.signal.carrier_hz.value();
    let Some(narrow_lane_wavelength_m) = narrow_lane_wavelength_m_from_frequencies(f1_hz, f2_hz)
    else {
        return (None, None, None, NarrowLaneStatus::FrequencyInvalid);
    };

    let lambda1 = signal_wavelength_m(first.metadata.signal).0;
    let lambda2 = signal_wavelength_m(second.metadata.signal).0;
    let phase_m = signal_cycles_to_meters(first.carrier_phase_cycles, first.metadata.signal).0
        + signal_cycles_to_meters(second.carrier_phase_cycles, second.metadata.signal).0;
    let variance_m2 = lambda1.powi(2) * first.carrier_phase_var_cycles2
        + lambda2.powi(2) * second.carrier_phase_var_cycles2;

    (Some(phase_m), Some(variance_m2), Some(narrow_lane_wavelength_m), NarrowLaneStatus::Ok)
}

fn status_reason(status: NarrowLaneStatus) -> (String, String) {
    match status {
        NarrowLaneStatus::Ok => ("ok".to_string(), "ok".to_string()),
        NarrowLaneStatus::MissingFrequency => {
            ("invalid".to_string(), "missing_frequency".to_string())
        }
        NarrowLaneStatus::UnsupportedBandPair => {
            ("invalid".to_string(), "unsupported_band_pair".to_string())
        }
        NarrowLaneStatus::ConstellationMismatch => {
            ("invalid".to_string(), "constellation_mismatch".to_string())
        }
        NarrowLaneStatus::TimeSystemMismatch => {
            ("invalid".to_string(), "time_system_mismatch".to_string())
        }
        NarrowLaneStatus::CarrierLockInvalid => {
            ("invalid".to_string(), "carrier_lock_invalid".to_string())
        }
        NarrowLaneStatus::VarianceInvalid => {
            ("invalid".to_string(), "variance_invalid".to_string())
        }
        NarrowLaneStatus::FrequencyInvalid => {
            ("invalid".to_string(), "frequency_invalid".to_string())
        }
    }
}

fn narrow_lane_status_from_pair_issue(issue: DualFrequencyPairIssue) -> NarrowLaneStatus {
    match issue {
        DualFrequencyPairIssue::MissingFrequency => NarrowLaneStatus::MissingFrequency,
        DualFrequencyPairIssue::UnsupportedBandPair => NarrowLaneStatus::UnsupportedBandPair,
        DualFrequencyPairIssue::ConstellationMismatch => NarrowLaneStatus::ConstellationMismatch,
        DualFrequencyPairIssue::TimeSystemMismatch => NarrowLaneStatus::TimeSystemMismatch,
    }
}

#[cfg(test)]
mod tests {
    use super::narrow_lane_from_obs_epochs;
    use bijux_gnss_core::api::{
        Constellation, Cycles, GpsTime, Hertz, LockFlags, Meters, ObsEpoch, ObsMetadata,
        ObsSatellite, ObsSignalTiming, ObservationEpochDecision, ObservationStatus, ReceiverRole,
        ReceiverSampleTrace, SatId, Seconds, SigId, SignalBand, SignalCode, SignalSpec,
    };
    use bijux_gnss_signal::api::{
        signal_spec_galileo_e1b, signal_spec_gps_l1_ca, signal_spec_gps_l2_py, signal_spec_gps_l5,
    };

    const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;

    struct DualFrequencySignalObservation {
        band: SignalBand,
        code: SignalCode,
        signal: SignalSpec,
        phase_cycles: f64,
        carrier_lock: bool,
        variance_cycles2: f64,
    }

    struct DualFrequencyEpochRequest {
        second_observation: DualFrequencySignalObservation,
        first_phase_cycles: f64,
        first_carrier_lock: bool,
        first_variance_cycles2: f64,
    }

    fn dual_frequency_epoch(request: DualFrequencyEpochRequest) -> ObsEpoch {
        let sat = SatId { constellation: Constellation::Gps, prn: 11 };
        let signal_1 = signal_spec_gps_l1_ca();
        ObsEpoch {
            t_rx_s: Seconds(0.0),
            source_time: ReceiverSampleTrace::from_sample_index(0, 1_000.0),
            gps_week: None,
            tow_s: None,
            epoch_idx: 0,
            discontinuity: false,
            valid: true,
            processing_ms: None,
            role: ReceiverRole::Rover,
            sats: vec![
                make_satellite(
                    sat,
                    SignalBand::L1,
                    SignalCode::Ca,
                    signal_1,
                    request.first_phase_cycles,
                    request.first_carrier_lock,
                    request.first_variance_cycles2,
                ),
                make_satellite(
                    sat,
                    request.second_observation.band,
                    request.second_observation.code,
                    request.second_observation.signal,
                    request.second_observation.phase_cycles,
                    request.second_observation.carrier_lock,
                    request.second_observation.variance_cycles2,
                ),
            ],
            decision: ObservationEpochDecision::Accepted,
            decision_reason: Some("accepted_observables_present".to_string()),
            manifest: None,
        }
    }

    fn make_satellite(
        sat: SatId,
        band: SignalBand,
        code: SignalCode,
        signal: SignalSpec,
        phase_cycles: f64,
        carrier_lock: bool,
        variance_cycles2: f64,
    ) -> ObsSatellite {
        ObsSatellite {
            signal_id: SigId { sat, band, code },
            pseudorange_m: Meters(0.0),
            pseudorange_var_m2: f64::NAN,
            carrier_phase_cycles: Cycles(phase_cycles),
            carrier_phase_var_cycles2: variance_cycles2,
            doppler_hz: Hertz(0.0),
            doppler_var_hz2: 1.0,
            cn0_dbhz: 45.0,
            lock_flags: LockFlags {
                code_lock: false,
                carrier_lock,
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

    fn carrier_cycles(range_m: f64, wavelength_m: f64, ambiguity_cycles: f64) -> f64 {
        range_m / wavelength_m + ambiguity_cycles
    }

    #[test]
    fn narrow_lane_phase_cycles_follow_sum_of_carrier_paths() {
        let l1 = signal_spec_gps_l1_ca();
        let l2 = signal_spec_gps_l2_py();
        let base_range_m = 24_000_000.0;
        let ambiguity_l1_cycles = 17.0;
        let ambiguity_l2_cycles = 11.0;
        let lambda1 = SPEED_OF_LIGHT_MPS / l1.carrier_hz.value();
        let lambda2 = SPEED_OF_LIGHT_MPS / l2.carrier_hz.value();
        let observations = narrow_lane_from_obs_epochs(
            &[dual_frequency_epoch(DualFrequencyEpochRequest {
                second_observation: DualFrequencySignalObservation {
                    band: SignalBand::L2,
                    code: SignalCode::Py,
                    signal: l2,
                    phase_cycles: carrier_cycles(base_range_m, lambda2, ambiguity_l2_cycles),
                    carrier_lock: true,
                    variance_cycles2: 0.01,
                },
                first_phase_cycles: carrier_cycles(base_range_m, lambda1, ambiguity_l1_cycles),
                first_carrier_lock: true,
                first_variance_cycles2: 0.01,
            })],
            SignalBand::L1,
            SignalBand::L2,
        );

        let observation = &observations[0];
        let lambda_nl = SPEED_OF_LIGHT_MPS / (l1.carrier_hz.value() + l2.carrier_hz.value());
        let expected_phase_cycles =
            (2.0 * base_range_m + lambda1 * ambiguity_l1_cycles + lambda2 * ambiguity_l2_cycles)
                / lambda_nl;

        assert_eq!(observation.status, "ok");
        assert!(
            (observation.phase_cycles.expect("phase cycles") - expected_phase_cycles).abs()
                < 1.0e-6
        );
    }

    #[test]
    fn narrow_lane_supports_l1_l5_pairs() {
        let l1 = signal_spec_gps_l1_ca();
        let l5 = signal_spec_gps_l5();
        let observations = narrow_lane_from_obs_epochs(
            &[dual_frequency_epoch(DualFrequencyEpochRequest {
                second_observation: DualFrequencySignalObservation {
                    band: SignalBand::L5,
                    code: SignalCode::Unknown,
                    signal: l5,
                    phase_cycles: carrier_cycles(
                        24_000_000.0,
                        SPEED_OF_LIGHT_MPS / l5.carrier_hz.value(),
                        4.0,
                    ),
                    carrier_lock: true,
                    variance_cycles2: 0.04,
                },
                first_phase_cycles: carrier_cycles(
                    24_000_000.0,
                    SPEED_OF_LIGHT_MPS / l1.carrier_hz.value(),
                    9.0,
                ),
                first_carrier_lock: true,
                first_variance_cycles2: 0.01,
            })],
            SignalBand::L1,
            SignalBand::L5,
        );

        assert_eq!(observations.len(), 1);
        assert_eq!(observations[0].status, "ok");
        assert!(observations[0].phase_m.is_some());
        assert!(observations[0].variance_m2.is_some());
        assert!(observations[0].narrow_lane_wavelength_m.is_some());
    }

    #[test]
    fn narrow_lane_rejects_unsupported_band_pairs() {
        let observations = narrow_lane_from_obs_epochs(
            &[dual_frequency_epoch(DualFrequencyEpochRequest {
                second_observation: DualFrequencySignalObservation {
                    band: SignalBand::L2,
                    code: SignalCode::Py,
                    signal: signal_spec_gps_l2_py(),
                    phase_cycles: 1001.0,
                    carrier_lock: true,
                    variance_cycles2: 0.01,
                },
                first_phase_cycles: 1000.0,
                first_carrier_lock: true,
                first_variance_cycles2: 0.01,
            })],
            SignalBand::L2,
            SignalBand::L5,
        );

        assert_eq!(observations.len(), 1);
        assert_eq!(observations[0].reason, "unsupported_band_pair");
    }

    #[test]
    fn narrow_lane_rejects_constellation_mismatches() {
        let sat = SatId { constellation: Constellation::Gps, prn: 11 };
        let epoch = ObsEpoch {
            t_rx_s: Seconds(0.0),
            source_time: ReceiverSampleTrace::from_sample_index(0, 1_000.0),
            gps_week: None,
            tow_s: None,
            epoch_idx: 0,
            discontinuity: false,
            valid: true,
            processing_ms: None,
            role: ReceiverRole::Rover,
            sats: vec![
                make_satellite(
                    sat,
                    SignalBand::L1,
                    SignalCode::Ca,
                    signal_spec_gps_l1_ca(),
                    1000.0,
                    true,
                    0.01,
                ),
                make_satellite(
                    sat,
                    SignalBand::L2,
                    SignalCode::Py,
                    signal_spec_galileo_e1b(),
                    1001.0,
                    true,
                    0.01,
                ),
            ],
            decision: ObservationEpochDecision::Accepted,
            decision_reason: Some("accepted_observables_present".to_string()),
            manifest: None,
        };

        let observations = narrow_lane_from_obs_epochs(&[epoch], SignalBand::L1, SignalBand::L2);

        assert_eq!(observations[0].reason, "constellation_mismatch");
    }

    #[test]
    fn narrow_lane_rejects_time_system_mismatches() {
        let sat = SatId { constellation: Constellation::Gps, prn: 11 };
        let mut first = make_satellite(
            sat,
            SignalBand::L1,
            SignalCode::Ca,
            signal_spec_gps_l1_ca(),
            1000.0,
            true,
            0.01,
        );
        let mut second = make_satellite(
            sat,
            SignalBand::L2,
            SignalCode::Py,
            signal_spec_gps_l2_py(),
            1001.0,
            true,
            0.01,
        );
        first.timing = Some(ObsSignalTiming {
            signal_travel_time_s: Seconds(0.075),
            transmit_gps_time: GpsTime { week: 2200, tow_s: 345_600.0 },
        });
        second.timing = Some(ObsSignalTiming {
            signal_travel_time_s: Seconds(0.075),
            transmit_gps_time: GpsTime { week: 2201, tow_s: 0.0 },
        });
        let epoch = ObsEpoch {
            t_rx_s: Seconds(0.0),
            source_time: ReceiverSampleTrace::from_sample_index(0, 1_000.0),
            gps_week: None,
            tow_s: None,
            epoch_idx: 0,
            discontinuity: false,
            valid: true,
            processing_ms: None,
            role: ReceiverRole::Rover,
            sats: vec![first, second],
            decision: ObservationEpochDecision::Accepted,
            decision_reason: Some("accepted_observables_present".to_string()),
            manifest: None,
        };

        let observations = narrow_lane_from_obs_epochs(&[epoch], SignalBand::L1, SignalBand::L2);

        assert_eq!(observations[0].reason, "time_system_mismatch");
    }
}
