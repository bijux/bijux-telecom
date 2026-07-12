#![allow(missing_docs)]

use std::collections::BTreeMap;

use bijux_gnss_core::api::{
    signal_cycles_to_meters, signal_wavelength_m, ObsEpoch, ObsSatellite, SatId, SignalBand,
};

use crate::corrections::combinations::narrow_lane_wavelength_m_from_frequencies;

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct IonoFreePhaseObservation {
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
enum IonoFreePhaseStatus {
    Ok,
    MissingFrequency,
    CarrierLockInvalid,
    VarianceInvalid,
    FrequencyInvalid,
}

pub fn iono_free_phase_from_obs_epochs(
    epochs: &[ObsEpoch],
    band_1: SignalBand,
    band_2: SignalBand,
) -> Vec<IonoFreePhaseObservation> {
    let mut out = Vec::new();

    for epoch in epochs {
        let mut by_sat: BTreeMap<SatId, Vec<&ObsSatellite>> = BTreeMap::new();
        for sat in &epoch.sats {
            by_sat.entry(sat.signal_id.sat).or_default().push(sat);
        }

        for (sat, sats) in by_sat {
            let first = sats.iter().find(|candidate| candidate.signal_id.band == band_1).copied();
            let second = sats.iter().find(|candidate| candidate.signal_id.band == band_2).copied();
            if first.is_none() && second.is_none() {
                continue;
            }
            out.push(iono_free_phase_from_pair(
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

pub(crate) fn iono_free_phase_from_pair(
    epoch_idx: u64,
    t_rx_s: f64,
    sat: SatId,
    band_1: SignalBand,
    band_2: SignalBand,
    first: Option<&ObsSatellite>,
    second: Option<&ObsSatellite>,
) -> IonoFreePhaseObservation {
    let f1_hz =
        first.map(|observation| observation.metadata.signal.carrier_hz.value()).unwrap_or(0.0);
    let f2_hz =
        second.map(|observation| observation.metadata.signal.carrier_hz.value()).unwrap_or(0.0);

    let (phase_m, variance_m2, narrow_lane_wavelength_m, status) = match (first, second) {
        (Some(first), Some(second)) => evaluate_iono_free_phase(first, second),
        _ => (None, None, None, IonoFreePhaseStatus::MissingFrequency),
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

    IonoFreePhaseObservation {
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

fn evaluate_iono_free_phase(
    first: &ObsSatellite,
    second: &ObsSatellite,
) -> (Option<f64>, Option<f64>, Option<f64>, IonoFreePhaseStatus) {
    if !first.lock_flags.carrier_lock || !second.lock_flags.carrier_lock {
        return (None, None, None, IonoFreePhaseStatus::CarrierLockInvalid);
    }
    if !first.carrier_phase_var_cycles2.is_finite()
        || !second.carrier_phase_var_cycles2.is_finite()
        || first.carrier_phase_var_cycles2 < 0.0
        || second.carrier_phase_var_cycles2 < 0.0
    {
        return (None, None, None, IonoFreePhaseStatus::VarianceInvalid);
    }

    let f1_hz = first.metadata.signal.carrier_hz.value();
    let f2_hz = second.metadata.signal.carrier_hz.value();
    if !f1_hz.is_finite() || !f2_hz.is_finite() || f1_hz <= 0.0 || f2_hz <= 0.0 {
        return (None, None, None, IonoFreePhaseStatus::FrequencyInvalid);
    }

    let f1_2 = f1_hz * f1_hz;
    let f2_2 = f2_hz * f2_hz;
    let denom = f1_2 - f2_2;
    if !denom.is_finite() || denom.abs() <= f64::EPSILON {
        return (None, None, None, IonoFreePhaseStatus::FrequencyInvalid);
    }

    let lambda1 = signal_wavelength_m(first.metadata.signal).0;
    let lambda2 = signal_wavelength_m(second.metadata.signal).0;
    let weight_1 = f1_2 / denom;
    let weight_2 = -f2_2 / denom;
    let phase_m = weight_1
        * signal_cycles_to_meters(first.carrier_phase_cycles, first.metadata.signal).0
        + weight_2 * signal_cycles_to_meters(second.carrier_phase_cycles, second.metadata.signal).0;
    let variance_m2 = weight_1.powi(2) * lambda1.powi(2) * first.carrier_phase_var_cycles2
        + weight_2.powi(2) * lambda2.powi(2) * second.carrier_phase_var_cycles2;
    let narrow_lane_wavelength_m = narrow_lane_wavelength_m_from_frequencies(f1_hz, f2_hz)
        .expect("narrow-lane wavelength must exist for valid frequencies");

    (Some(phase_m), Some(variance_m2), Some(narrow_lane_wavelength_m), IonoFreePhaseStatus::Ok)
}

fn status_reason(status: IonoFreePhaseStatus) -> (String, String) {
    match status {
        IonoFreePhaseStatus::Ok => ("ok".to_string(), "ok".to_string()),
        IonoFreePhaseStatus::MissingFrequency => {
            ("invalid".to_string(), "missing_frequency".to_string())
        }
        IonoFreePhaseStatus::CarrierLockInvalid => {
            ("invalid".to_string(), "carrier_lock_invalid".to_string())
        }
        IonoFreePhaseStatus::VarianceInvalid => {
            ("invalid".to_string(), "variance_invalid".to_string())
        }
        IonoFreePhaseStatus::FrequencyInvalid => {
            ("invalid".to_string(), "frequency_invalid".to_string())
        }
    }
}

#[cfg(test)]
mod tests {
    use super::iono_free_phase_from_obs_epochs;
    use bijux_gnss_core::api::{
        signal_spec_gps_l1_ca, signal_spec_gps_l2_py, signal_spec_gps_l5, Constellation, Cycles,
        Hertz, LockFlags, Meters, ObsEpoch, ObsMetadata, ObsSatellite, ObservationEpochDecision,
        ObservationStatus, ReceiverRole, ReceiverSampleTrace, SatId, Seconds, SigId, SignalBand,
        SignalCode,
    };

    const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;

    fn dual_frequency_epoch(
        second_band: SignalBand,
        second_code: SignalCode,
        second_signal: bijux_gnss_core::api::SignalSpec,
        phase_1_cycles: f64,
        phase_2_cycles: f64,
        carrier_lock_1: bool,
        carrier_lock_2: bool,
        variance_1_cycles2: f64,
        variance_2_cycles2: f64,
    ) -> ObsEpoch {
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
                    phase_1_cycles,
                    carrier_lock_1,
                    variance_1_cycles2,
                ),
                make_satellite(
                    sat,
                    second_band,
                    second_code,
                    second_signal,
                    phase_2_cycles,
                    carrier_lock_2,
                    variance_2_cycles2,
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
        signal: bijux_gnss_core::api::SignalSpec,
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

    fn carrier_cycles(range_m: f64, iono_m: f64, wavelength_m: f64, ambiguity_cycles: f64) -> f64 {
        (range_m - iono_m) / wavelength_m + ambiguity_cycles
    }

    #[test]
    fn iono_free_phase_removes_first_order_ionosphere_for_l1_l2() {
        let base_range_m = 20_200_000.0;
        let iono_l1_m = 5.0;
        let l1 = signal_spec_gps_l1_ca();
        let l2 = signal_spec_gps_l2_py();
        let iono_l2_m = iono_l1_m * (l1.carrier_hz.value() * l1.carrier_hz.value())
            / (l2.carrier_hz.value() * l2.carrier_hz.value());
        let lambda1 = SPEED_OF_LIGHT_MPS / l1.carrier_hz.value();
        let lambda2 = SPEED_OF_LIGHT_MPS / l2.carrier_hz.value();
        let epoch = dual_frequency_epoch(
            SignalBand::L2,
            SignalCode::Py,
            l2,
            carrier_cycles(base_range_m, iono_l1_m, lambda1, 0.0),
            carrier_cycles(base_range_m, iono_l2_m, lambda2, 0.0),
            true,
            true,
            0.01,
            0.04,
        );

        let observations =
            iono_free_phase_from_obs_epochs(&[epoch], SignalBand::L1, SignalBand::L2);

        assert_eq!(observations.len(), 1);
        assert_eq!(observations[0].status, "ok");
        assert!((observations[0].phase_m.expect("iono-free phase") - base_range_m).abs() < 1.0e-6);
        let f1_2 = l1.carrier_hz.value() * l1.carrier_hz.value();
        let f2_2 = l2.carrier_hz.value() * l2.carrier_hz.value();
        let denom = f1_2 - f2_2;
        let weight_1 = f1_2 / denom;
        let weight_2 = -f2_2 / denom;
        let expected_variance =
            weight_1.powi(2) * lambda1.powi(2) * 0.01 + weight_2.powi(2) * lambda2.powi(2) * 0.04;
        assert!(
            (observations[0].variance_m2.expect("iono-free variance") - expected_variance).abs()
                < 1.0e-12
        );
    }

    #[test]
    fn iono_free_phase_removes_first_order_ionosphere_for_l1_l5() {
        let base_range_m = 20_200_000.0;
        let iono_l1_m = 5.0;
        let l1 = signal_spec_gps_l1_ca();
        let l5 = signal_spec_gps_l5();
        let iono_l5_m = iono_l1_m * (l1.carrier_hz.value() * l1.carrier_hz.value())
            / (l5.carrier_hz.value() * l5.carrier_hz.value());
        let lambda1 = SPEED_OF_LIGHT_MPS / l1.carrier_hz.value();
        let lambda5 = SPEED_OF_LIGHT_MPS / l5.carrier_hz.value();
        let epoch = dual_frequency_epoch(
            SignalBand::L5,
            SignalCode::Unknown,
            l5,
            carrier_cycles(base_range_m, iono_l1_m, lambda1, 0.0),
            carrier_cycles(base_range_m, iono_l5_m, lambda5, 0.0),
            true,
            true,
            0.01,
            0.01,
        );

        let observations =
            iono_free_phase_from_obs_epochs(&[epoch], SignalBand::L1, SignalBand::L5);

        assert_eq!(observations.len(), 1);
        assert_eq!(observations[0].status, "ok");
        assert!((observations[0].phase_m.expect("iono-free phase") - base_range_m).abs() < 1.0e-6);
    }

    #[test]
    fn iono_free_phase_preserves_l1_l2_narrow_lane_ambiguity() {
        let base_range_m = 20_200_000.0;
        let iono_l1_m = 5.0;
        let ambiguity_l1_cycles = 17.0;
        let ambiguity_l2_cycles = 11.0;
        let l1 = signal_spec_gps_l1_ca();
        let l2 = signal_spec_gps_l2_py();
        let iono_l2_m = iono_l1_m * (l1.carrier_hz.value() * l1.carrier_hz.value())
            / (l2.carrier_hz.value() * l2.carrier_hz.value());
        let lambda1 = SPEED_OF_LIGHT_MPS / l1.carrier_hz.value();
        let lambda2 = SPEED_OF_LIGHT_MPS / l2.carrier_hz.value();
        let epoch = dual_frequency_epoch(
            SignalBand::L2,
            SignalCode::Py,
            l2,
            carrier_cycles(base_range_m, iono_l1_m, lambda1, ambiguity_l1_cycles),
            carrier_cycles(base_range_m, iono_l2_m, lambda2, ambiguity_l2_cycles),
            true,
            true,
            0.01,
            0.01,
        );

        let observations =
            iono_free_phase_from_obs_epochs(&[epoch], SignalBand::L1, SignalBand::L2);
        let observation = &observations[0];
        let narrow_lane_wavelength_m =
            observation.narrow_lane_wavelength_m.expect("narrow-lane wavelength");
        let expected_narrow_lane_ambiguity_cycles = (l1.carrier_hz.value() * ambiguity_l1_cycles
            - l2.carrier_hz.value() * ambiguity_l2_cycles)
            / (l1.carrier_hz.value() - l2.carrier_hz.value());

        assert_eq!(observation.status, "ok");
        assert!(
            (observation.phase_cycles.expect("iono-free phase cycles")
                - (base_range_m / narrow_lane_wavelength_m
                    + expected_narrow_lane_ambiguity_cycles))
                .abs()
                < 1.0e-6
        );
    }

    #[test]
    fn iono_free_phase_does_not_require_code_lock_or_code_variance() {
        let l2 = signal_spec_gps_l2_py();
        let epoch = dual_frequency_epoch(
            SignalBand::L2,
            SignalCode::Py,
            l2,
            1000.0,
            1001.0,
            true,
            true,
            0.01,
            0.01,
        );

        let observations =
            iono_free_phase_from_obs_epochs(&[epoch], SignalBand::L1, SignalBand::L2);

        assert_eq!(observations[0].status, "ok");
        assert!(observations[0].phase_m.is_some());
        assert!(observations[0].variance_m2.is_some());
    }

    #[test]
    fn iono_free_phase_rejects_missing_carrier_lock() {
        let l2 = signal_spec_gps_l2_py();
        let epoch = dual_frequency_epoch(
            SignalBand::L2,
            SignalCode::Py,
            l2,
            1000.0,
            1001.0,
            true,
            false,
            0.01,
            0.01,
        );

        let observations =
            iono_free_phase_from_obs_epochs(&[epoch], SignalBand::L1, SignalBand::L2);

        assert_eq!(observations[0].status, "invalid");
        assert_eq!(observations[0].reason, "carrier_lock_invalid");
        assert!(observations[0].phase_m.is_none());
        assert!(observations[0].phase_cycles.is_none());
    }
}
