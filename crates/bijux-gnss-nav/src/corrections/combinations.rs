#![allow(missing_docs)]

use std::collections::BTreeMap;

use bijux_gnss_core::api::{
    signal_cycles_to_meters, ObsEpoch, ObsSatellite, SatId, SigId, SignalBand,
};

use crate::corrections::dual_frequency::{dual_frequency_pair_issue, DualFrequencyPairIssue};
use crate::corrections::iono_free_code::iono_free_code_from_pair;
use crate::corrections::iono_free_phase::iono_free_phase_from_pair;
use crate::corrections::narrow_lane::narrow_lane_from_pair;

const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct CombinationObservation {
    pub epoch_idx: u64,
    pub t_rx_s: f64,
    pub sat: SatId,
    pub signal_1: Option<SigId>,
    pub signal_2: Option<SigId>,
    pub band_1: SignalBand,
    pub band_2: SignalBand,
    pub f1_hz: f64,
    pub f2_hz: f64,
    pub if_code_m: Option<f64>,
    pub if_code_bias_m: Option<f64>,
    pub if_code_corrected_m: Option<f64>,
    pub if_code_var_m2: Option<f64>,
    pub if_code_status: String,
    pub if_code_reason: String,
    pub if_phase_m: Option<f64>,
    pub if_phase_var_m2: Option<f64>,
    pub if_phase_cycles: Option<f64>,
    pub if_phase_var_cycles2: Option<f64>,
    pub if_phase_status: String,
    pub if_phase_reason: String,
    pub narrow_lane_wavelength_m: Option<f64>,
    pub geometry_free_phase_m: Option<f64>,
    pub wide_lane_wavelength_m: Option<f64>,
    pub wide_lane_cycles: Option<f64>,
    pub narrow_lane_cycles: Option<f64>,
    pub melbourne_wubbena_m: Option<f64>,
    pub status: String,
    pub reason: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CombinationStatus {
    Ok,
    MissingFrequency,
    UnsupportedBandPair,
    ConstellationMismatch,
    TimeSystemMismatch,
    LockInvalid,
    VarianceInvalid,
    FrequencyInvalid,
}

fn status_reason(status: CombinationStatus) -> (String, String) {
    match status {
        CombinationStatus::Ok => ("ok".to_string(), "ok".to_string()),
        CombinationStatus::MissingFrequency => {
            ("invalid".to_string(), "missing_frequency".to_string())
        }
        CombinationStatus::UnsupportedBandPair => {
            ("invalid".to_string(), "unsupported_band_pair".to_string())
        }
        CombinationStatus::ConstellationMismatch => {
            ("invalid".to_string(), "constellation_mismatch".to_string())
        }
        CombinationStatus::TimeSystemMismatch => {
            ("invalid".to_string(), "time_system_mismatch".to_string())
        }
        CombinationStatus::LockInvalid => ("invalid".to_string(), "lock_invalid".to_string()),
        CombinationStatus::VarianceInvalid => {
            ("invalid".to_string(), "variance_invalid".to_string())
        }
        CombinationStatus::FrequencyInvalid => {
            ("invalid".to_string(), "frequency_invalid".to_string())
        }
    }
}

pub fn wide_lane_wavelength_m_from_frequencies(f1_hz: f64, f2_hz: f64) -> Option<f64> {
    let frequency_separation_hz = (f1_hz - f2_hz).abs();
    if !f1_hz.is_finite()
        || !f2_hz.is_finite()
        || f1_hz <= 0.0
        || f2_hz <= 0.0
        || !frequency_separation_hz.is_finite()
        || frequency_separation_hz <= f64::EPSILON
    {
        return None;
    }
    Some(SPEED_OF_LIGHT_MPS / frequency_separation_hz)
}

pub fn narrow_lane_wavelength_m_from_frequencies(f1_hz: f64, f2_hz: f64) -> Option<f64> {
    let frequency_sum_hz = f1_hz + f2_hz;
    if !f1_hz.is_finite()
        || !f2_hz.is_finite()
        || f1_hz <= 0.0
        || f2_hz <= 0.0
        || !frequency_sum_hz.is_finite()
        || frequency_sum_hz <= f64::EPSILON
    {
        return None;
    }
    Some(SPEED_OF_LIGHT_MPS / frequency_sum_hz)
}

pub fn combinations_from_obs_epochs(
    epochs: &[ObsEpoch],
    band_1: SignalBand,
    band_2: SignalBand,
) -> Vec<CombinationObservation> {
    let mut out = Vec::new();
    for epoch in epochs {
        let mut by_sat: BTreeMap<SatId, Vec<&ObsSatellite>> = BTreeMap::new();
        for sat in &epoch.sats {
            by_sat.entry(sat.signal_id.sat).or_default().push(sat);
        }
        for (sat_id, sats) in by_sat {
            let s1 = sats.iter().find(|s| s.signal_id.band == band_1);
            let s2 = sats.iter().find(|s| s.signal_id.band == band_2);
            let mut status = CombinationStatus::Ok;
            if let Some(issue) =
                dual_frequency_pair_issue(sat_id, band_1, band_2, s1.copied(), s2.copied())
            {
                status = combination_status_from_pair_issue(issue);
            }
            let f1_hz = s1.map(|s| s.metadata.signal.carrier_hz.value()).unwrap_or(0.0);
            let f2_hz = s2.map(|s| s.metadata.signal.carrier_hz.value()).unwrap_or(0.0);
            let if_code = iono_free_code_from_pair(
                epoch.epoch_idx,
                epoch.t_rx_s.0,
                sat_id,
                band_1,
                band_2,
                s1.copied(),
                s2.copied(),
            );
            let if_phase = iono_free_phase_from_pair(
                epoch.epoch_idx,
                epoch.t_rx_s.0,
                sat_id,
                band_1,
                band_2,
                s1.copied(),
                s2.copied(),
            );
            let narrow_lane = narrow_lane_from_pair(
                epoch.epoch_idx,
                epoch.t_rx_s.0,
                sat_id,
                band_1,
                band_2,
                s1.copied(),
                s2.copied(),
            );
            let (mut if_phase_m, mut geometry_free_phase_m) = (None, None);
            let (mut wide_lane_wavelength_m, mut wide_lane_cycles, mut mw_m) = (None, None, None);
            if status == CombinationStatus::Ok {
                let (Some(s1), Some(s2)) = (s1, s2) else {
                    unreachable!("compatible dual-frequency pairs must include both observations");
                };
                if !s1.lock_flags.code_lock
                    || !s1.lock_flags.carrier_lock
                    || !s2.lock_flags.code_lock
                    || !s2.lock_flags.carrier_lock
                {
                    status = CombinationStatus::LockInvalid;
                } else if !s1.pseudorange_var_m2.is_finite()
                    || !s2.pseudorange_var_m2.is_finite()
                    || !s1.carrier_phase_var_cycles2.is_finite()
                    || !s2.carrier_phase_var_cycles2.is_finite()
                {
                    status = CombinationStatus::VarianceInvalid;
                } else {
                    let f1_2 = f1_hz * f1_hz;
                    let f2_2 = f2_hz * f2_hz;
                    let denom = f1_2 - f2_2;
                    if !f1_hz.is_finite()
                        || !f2_hz.is_finite()
                        || f1_hz <= 0.0
                        || f2_hz <= 0.0
                        || !denom.is_finite()
                        || denom.abs() <= f64::EPSILON
                    {
                        status = CombinationStatus::FrequencyInvalid;
                    } else {
                        let phi1_m =
                            signal_cycles_to_meters(s1.carrier_phase_cycles, s1.metadata.signal).0;
                        let phi2_m =
                            signal_cycles_to_meters(s2.carrier_phase_cycles, s2.metadata.signal).0;

                        if_phase_m = if_phase.phase_m;
                        geometry_free_phase_m = Some(phi1_m - phi2_m);

                        let lambda_wl = wide_lane_wavelength_m_from_frequencies(f1_hz, f2_hz)
                            .expect("wide-lane wavelength must exist for valid frequencies");
                        wide_lane_wavelength_m = Some(lambda_wl);
                        wide_lane_cycles = Some((phi1_m - phi2_m) / lambda_wl);
                        mw_m = Some((phi1_m - phi2_m) - (s1.pseudorange_m.0 - s2.pseudorange_m.0));
                    }
                }
            }
            let (status_str, reason) = status_reason(status);
            out.push(CombinationObservation {
                epoch_idx: epoch.epoch_idx,
                t_rx_s: epoch.t_rx_s.0,
                sat: sat_id,
                signal_1: if_code.signal_1,
                signal_2: if_code.signal_2,
                band_1,
                band_2,
                f1_hz,
                f2_hz,
                if_code_m: if_code.code_m,
                if_code_bias_m: if_code.code_bias_m,
                if_code_corrected_m: if_code.corrected_code_m,
                if_code_var_m2: if_code.variance_m2,
                if_code_status: if_code.status,
                if_code_reason: if_code.reason,
                if_phase_m: if_phase.phase_m.or(if_phase_m),
                if_phase_var_m2: if_phase.variance_m2,
                if_phase_cycles: if_phase.phase_cycles,
                if_phase_var_cycles2: if_phase.variance_cycles2,
                if_phase_status: if_phase.status,
                if_phase_reason: if_phase.reason,
                narrow_lane_wavelength_m: narrow_lane.narrow_lane_wavelength_m,
                geometry_free_phase_m,
                wide_lane_wavelength_m,
                wide_lane_cycles,
                narrow_lane_cycles: narrow_lane.phase_cycles,
                melbourne_wubbena_m: mw_m,
                status: status_str,
                reason,
            });
        }
    }
    out
}

fn combination_status_from_pair_issue(issue: DualFrequencyPairIssue) -> CombinationStatus {
    match issue {
        DualFrequencyPairIssue::MissingFrequency => CombinationStatus::MissingFrequency,
        DualFrequencyPairIssue::UnsupportedBandPair => CombinationStatus::UnsupportedBandPair,
        DualFrequencyPairIssue::ConstellationMismatch => CombinationStatus::ConstellationMismatch,
        DualFrequencyPairIssue::TimeSystemMismatch => CombinationStatus::TimeSystemMismatch,
    }
}

#[cfg(test)]
mod tests {
    use super::{
        combinations_from_obs_epochs, narrow_lane_wavelength_m_from_frequencies,
        wide_lane_wavelength_m_from_frequencies,
    };
    use bijux_gnss_core::api::{
        signal_spec_galileo_e1b, signal_spec_gps_l1_ca, signal_spec_gps_l2_py, Constellation,
        Cycles, GpsTime, Hertz, LockFlags, Meters, ObsEpoch, ObsMetadata, ObsSatellite,
        ObsSignalTiming, ObservationEpochDecision, ObservationStatus, ReceiverRole,
        ReceiverSampleTrace, SatId, Seconds, SigId, SignalBand, SignalCode,
    };

    fn dual_frequency_epoch(code_lock: bool, carrier_lock: bool) -> ObsEpoch {
        let sat = SatId { constellation: Constellation::Gps, prn: 3 };
        let l1 = signal_spec_gps_l1_ca();
        let l2 = signal_spec_gps_l2_py();

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
                satellite(sat, SignalBand::L1, SignalCode::Ca, l1, code_lock, carrier_lock),
                satellite(sat, SignalBand::L2, SignalCode::Py, l2, code_lock, carrier_lock),
            ],
            decision: ObservationEpochDecision::Accepted,
            decision_reason: Some("accepted_observables_present".to_string()),
            manifest: None,
        }
    }

    fn satellite(
        sat: SatId,
        band: SignalBand,
        code: SignalCode,
        signal: bijux_gnss_core::api::SignalSpec,
        code_lock: bool,
        carrier_lock: bool,
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
            lock_flags: LockFlags { code_lock, carrier_lock, bit_lock: false, cycle_slip: false },
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
    fn combinations_keep_iono_free_code_when_carrier_phase_is_unavailable() {
        let combinations = combinations_from_obs_epochs(
            &[dual_frequency_epoch(true, false)],
            SignalBand::L1,
            SignalBand::L2,
        );

        assert_eq!(combinations.len(), 1);
        assert_eq!(combinations[0].status, "invalid");
        assert_eq!(combinations[0].reason, "lock_invalid");
        assert_eq!(combinations[0].if_code_status, "ok");
        assert_eq!(combinations[0].if_code_reason, "ok");
        assert!(combinations[0].if_code_m.is_some());
        assert_eq!(
            combinations[0].signal_1,
            Some(SigId { sat: combinations[0].sat, band: SignalBand::L1, code: SignalCode::Ca })
        );
        assert_eq!(
            combinations[0].signal_2,
            Some(SigId { sat: combinations[0].sat, band: SignalBand::L2, code: SignalCode::Py })
        );
        assert!(combinations[0].if_code_var_m2.is_some());
        assert_eq!(combinations[0].if_phase_status, "invalid");
        assert_eq!(combinations[0].if_phase_reason, "carrier_lock_invalid");
        assert!(combinations[0].if_phase_m.is_none());
    }

    #[test]
    fn combinations_keep_full_combination_invalid_when_code_lock_is_missing() {
        let combinations = combinations_from_obs_epochs(
            &[dual_frequency_epoch(false, true)],
            SignalBand::L1,
            SignalBand::L2,
        );

        assert_eq!(combinations.len(), 1);
        assert_eq!(combinations[0].status, "invalid");
        assert_eq!(combinations[0].reason, "lock_invalid");
        assert_eq!(combinations[0].if_code_status, "invalid");
        assert_eq!(combinations[0].if_code_reason, "code_lock_invalid");
        assert_eq!(combinations[0].if_phase_status, "ok");
        assert_eq!(combinations[0].if_phase_reason, "ok");
        assert!(combinations[0].if_phase_m.is_some());
        assert!(combinations[0].if_code_m.is_none());
    }

    #[test]
    fn lane_wavelength_helpers_follow_frequency_sum_and_difference() {
        let l1 = signal_spec_gps_l1_ca();
        let l2 = signal_spec_gps_l2_py();

        let wide_lane =
            wide_lane_wavelength_m_from_frequencies(l1.carrier_hz.value(), l2.carrier_hz.value())
                .expect("wide-lane wavelength");
        let narrow_lane =
            narrow_lane_wavelength_m_from_frequencies(l1.carrier_hz.value(), l2.carrier_hz.value())
                .expect("narrow-lane wavelength");

        assert!(
            (wide_lane - 299_792_458.0 / (l1.carrier_hz.value() - l2.carrier_hz.value()).abs())
                .abs()
                < 1.0e-12
        );
        assert!(
            (narrow_lane - 299_792_458.0 / (l1.carrier_hz.value() + l2.carrier_hz.value())).abs()
                < 1.0e-12
        );
    }

    #[test]
    fn combinations_reject_unsupported_band_pairs_before_reporting_missing_frequency() {
        let combinations = combinations_from_obs_epochs(
            &[dual_frequency_epoch(true, true)],
            SignalBand::L2,
            SignalBand::L5,
        );

        assert_eq!(combinations.len(), 1);
        assert_eq!(combinations[0].status, "invalid");
        assert_eq!(combinations[0].reason, "unsupported_band_pair");
    }

    #[test]
    fn combinations_reject_constellation_mismatches() {
        let sat = SatId { constellation: Constellation::Gps, prn: 3 };
        let l1 =
            satellite(sat, SignalBand::L1, SignalCode::Ca, signal_spec_gps_l1_ca(), true, true);
        let l2 =
            satellite(sat, SignalBand::L2, SignalCode::Py, signal_spec_galileo_e1b(), true, true);
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
            sats: vec![l1, l2],
            decision: ObservationEpochDecision::Accepted,
            decision_reason: Some("accepted_observables_present".to_string()),
            manifest: None,
        };

        let combinations = combinations_from_obs_epochs(&[epoch], SignalBand::L1, SignalBand::L2);

        assert_eq!(combinations[0].reason, "constellation_mismatch");
    }

    #[test]
    fn combinations_reject_transmit_time_mismatches() {
        let sat = SatId { constellation: Constellation::Gps, prn: 3 };
        let mut l1 =
            satellite(sat, SignalBand::L1, SignalCode::Ca, signal_spec_gps_l1_ca(), true, true);
        let mut l2 =
            satellite(sat, SignalBand::L2, SignalCode::Py, signal_spec_gps_l2_py(), true, true);
        l1.timing = Some(ObsSignalTiming {
            signal_travel_time_s: Seconds(0.075),
            transmit_gps_time: GpsTime { week: 2200, tow_s: 345_600.0 },
        });
        l2.timing = Some(ObsSignalTiming {
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
            sats: vec![l1, l2],
            decision: ObservationEpochDecision::Accepted,
            decision_reason: Some("accepted_observables_present".to_string()),
            manifest: None,
        };

        let combinations = combinations_from_obs_epochs(&[epoch], SignalBand::L1, SignalBand::L2);

        assert_eq!(combinations[0].reason, "time_system_mismatch");
    }
}
