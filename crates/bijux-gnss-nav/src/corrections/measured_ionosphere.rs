#![allow(missing_docs)]

use std::collections::BTreeMap;

use bijux_gnss_core::api::{ObsEpoch, ObsSatellite, SatId, SigId, SignalBand};

use crate::corrections::dual_frequency::{dual_frequency_pair_issue, DualFrequencyPairIssue};

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct MeasuredIonosphereObservation {
    pub epoch_idx: u64,
    pub t_rx_s: f64,
    pub sat: SatId,
    pub signal_1: Option<SigId>,
    pub signal_2: Option<SigId>,
    pub band_1: SignalBand,
    pub band_2: SignalBand,
    pub f1_hz: f64,
    pub f2_hz: f64,
    pub code_geometry_free_m: Option<f64>,
    pub code_geometry_free_var_m2: Option<f64>,
    pub code_delay_band_1_m: Option<f64>,
    pub code_delay_band_2_m: Option<f64>,
    pub code_delay_var_band_1_m2: Option<f64>,
    pub code_delay_var_band_2_m2: Option<f64>,
    pub code_status: String,
    pub code_reason: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum MeasuredIonosphereCodeStatus {
    Ok,
    MissingFrequency,
    UnsupportedBandPair,
    ConstellationMismatch,
    TimeSystemMismatch,
    CodeLockInvalid,
    VarianceInvalid,
    FrequencyInvalid,
}

pub fn measured_ionosphere_from_obs_epochs(
    epochs: &[ObsEpoch],
    band_1: SignalBand,
    band_2: SignalBand,
) -> Vec<MeasuredIonosphereObservation> {
    let mut out = Vec::new();

    for epoch in epochs {
        let mut by_sat: BTreeMap<SatId, Vec<&ObsSatellite>> = BTreeMap::new();
        for sat in &epoch.sats {
            by_sat.entry(sat.signal_id.sat).or_default().push(sat);
        }

        for (sat, sats) in by_sat {
            let first = sats.iter().find(|candidate| candidate.signal_id.band == band_1).copied();
            let second = sats.iter().find(|candidate| candidate.signal_id.band == band_2).copied();
            out.push(measured_ionosphere_from_pair(
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

pub(crate) fn measured_ionosphere_from_pair(
    epoch_idx: u64,
    t_rx_s: f64,
    sat: SatId,
    band_1: SignalBand,
    band_2: SignalBand,
    first: Option<&ObsSatellite>,
    second: Option<&ObsSatellite>,
) -> MeasuredIonosphereObservation {
    let f1_hz =
        first.map(|observation| observation.metadata.signal.carrier_hz.value()).unwrap_or(0.0);
    let f2_hz =
        second.map(|observation| observation.metadata.signal.carrier_hz.value()).unwrap_or(0.0);
    let signal_1 = first.map(|observation| observation.signal_id);
    let signal_2 = second.map(|observation| observation.signal_id);

    let (
        code_geometry_free_m,
        code_geometry_free_var_m2,
        code_delay_band_1_m,
        code_delay_band_2_m,
        code_delay_var_band_1_m2,
        code_delay_var_band_2_m2,
        code_status,
    ) = match dual_frequency_pair_issue(sat, band_1, band_2, first, second) {
        Some(issue) => (
            None,
            None,
            None,
            None,
            None,
            None,
            measured_ionosphere_code_status_from_pair_issue(issue),
        ),
        None => {
            let (Some(first), Some(second)) = (first, second) else {
                unreachable!("compatible dual-frequency pairs must include both observations");
            };
            evaluate_measured_ionosphere_code(first, second)
        }
    };
    let (code_status, code_reason) = status_reason(code_status);

    MeasuredIonosphereObservation {
        epoch_idx,
        t_rx_s,
        sat,
        signal_1,
        signal_2,
        band_1,
        band_2,
        f1_hz,
        f2_hz,
        code_geometry_free_m,
        code_geometry_free_var_m2,
        code_delay_band_1_m,
        code_delay_band_2_m,
        code_delay_var_band_1_m2,
        code_delay_var_band_2_m2,
        code_status,
        code_reason,
    }
}

fn evaluate_measured_ionosphere_code(
    first: &ObsSatellite,
    second: &ObsSatellite,
) -> (
    Option<f64>,
    Option<f64>,
    Option<f64>,
    Option<f64>,
    Option<f64>,
    Option<f64>,
    MeasuredIonosphereCodeStatus,
) {
    if !first.lock_flags.code_lock || !second.lock_flags.code_lock {
        return (None, None, None, None, None, None, MeasuredIonosphereCodeStatus::CodeLockInvalid);
    }
    if !first.pseudorange_var_m2.is_finite()
        || !second.pseudorange_var_m2.is_finite()
        || first.pseudorange_var_m2 < 0.0
        || second.pseudorange_var_m2 < 0.0
    {
        return (None, None, None, None, None, None, MeasuredIonosphereCodeStatus::VarianceInvalid);
    }

    let f1_hz = first.metadata.signal.carrier_hz.value();
    let f2_hz = second.metadata.signal.carrier_hz.value();
    if !f1_hz.is_finite() || !f2_hz.is_finite() || f1_hz <= 0.0 || f2_hz <= 0.0 {
        return (None, None, None, None, None, None, MeasuredIonosphereCodeStatus::FrequencyInvalid);
    }

    let f1_2 = f1_hz * f1_hz;
    let f2_2 = f2_hz * f2_hz;
    let denom = f1_2 - f2_2;
    if !denom.is_finite() || denom.abs() <= f64::EPSILON {
        return (None, None, None, None, None, None, MeasuredIonosphereCodeStatus::FrequencyInvalid);
    }

    let geometry_free_m = second.pseudorange_m.0 - first.pseudorange_m.0;
    let geometry_free_var_m2 = first.pseudorange_var_m2 + second.pseudorange_var_m2;
    let scale_band_1 = f2_2 / denom;
    let scale_band_2 = f1_2 / denom;
    let delay_band_1_m = geometry_free_m * scale_band_1;
    let delay_band_2_m = geometry_free_m * scale_band_2;
    let delay_var_band_1_m2 = geometry_free_var_m2 * scale_band_1.powi(2);
    let delay_var_band_2_m2 = geometry_free_var_m2 * scale_band_2.powi(2);

    (
        Some(geometry_free_m),
        Some(geometry_free_var_m2),
        Some(delay_band_1_m),
        Some(delay_band_2_m),
        Some(delay_var_band_1_m2),
        Some(delay_var_band_2_m2),
        MeasuredIonosphereCodeStatus::Ok,
    )
}

fn status_reason(status: MeasuredIonosphereCodeStatus) -> (String, String) {
    match status {
        MeasuredIonosphereCodeStatus::Ok => ("ok".to_string(), "ok".to_string()),
        MeasuredIonosphereCodeStatus::MissingFrequency => {
            ("invalid".to_string(), "missing_frequency".to_string())
        }
        MeasuredIonosphereCodeStatus::UnsupportedBandPair => {
            ("invalid".to_string(), "unsupported_band_pair".to_string())
        }
        MeasuredIonosphereCodeStatus::ConstellationMismatch => {
            ("invalid".to_string(), "constellation_mismatch".to_string())
        }
        MeasuredIonosphereCodeStatus::TimeSystemMismatch => {
            ("invalid".to_string(), "time_system_mismatch".to_string())
        }
        MeasuredIonosphereCodeStatus::CodeLockInvalid => {
            ("invalid".to_string(), "code_lock_invalid".to_string())
        }
        MeasuredIonosphereCodeStatus::VarianceInvalid => {
            ("invalid".to_string(), "variance_invalid".to_string())
        }
        MeasuredIonosphereCodeStatus::FrequencyInvalid => {
            ("invalid".to_string(), "frequency_invalid".to_string())
        }
    }
}

fn measured_ionosphere_code_status_from_pair_issue(
    issue: DualFrequencyPairIssue,
) -> MeasuredIonosphereCodeStatus {
    match issue {
        DualFrequencyPairIssue::MissingFrequency => MeasuredIonosphereCodeStatus::MissingFrequency,
        DualFrequencyPairIssue::UnsupportedBandPair => {
            MeasuredIonosphereCodeStatus::UnsupportedBandPair
        }
        DualFrequencyPairIssue::ConstellationMismatch => {
            MeasuredIonosphereCodeStatus::ConstellationMismatch
        }
        DualFrequencyPairIssue::TimeSystemMismatch => {
            MeasuredIonosphereCodeStatus::TimeSystemMismatch
        }
    }
}

#[cfg(test)]
mod tests {
    use super::measured_ionosphere_from_obs_epochs;
    use bijux_gnss_core::api::{
        first_order_ionosphere_code_delay_m, signal_spec_gps_l1_ca, signal_spec_gps_l2_py,
        Constellation, Cycles, Hertz, LockFlags, Meters, ObsEpoch, ObsMetadata, ObsSatellite,
        ObservationEpochDecision, ObservationStatus, ReceiverRole, ReceiverSampleTrace, SatId,
        Seconds, SigId, SignalBand, SignalCode,
    };

    fn dual_frequency_epoch(
        pseudorange_1_m: f64,
        pseudorange_2_m: f64,
        code_lock_1: bool,
        code_lock_2: bool,
        variance_1_m2: f64,
        variance_2_m2: f64,
    ) -> ObsEpoch {
        let sat = SatId { constellation: Constellation::Gps, prn: 11 };
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
                    signal_spec_gps_l1_ca(),
                    pseudorange_1_m,
                    code_lock_1,
                    variance_1_m2,
                ),
                make_satellite(
                    sat,
                    SignalBand::L2,
                    SignalCode::Py,
                    signal_spec_gps_l2_py(),
                    pseudorange_2_m,
                    code_lock_2,
                    variance_2_m2,
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
        pseudorange_m: f64,
        code_lock: bool,
        variance_m2: f64,
    ) -> ObsSatellite {
        ObsSatellite {
            signal_id: SigId { sat, band, code },
            pseudorange_m: Meters(pseudorange_m),
            pseudorange_var_m2: variance_m2,
            carrier_phase_cycles: Cycles(0.0),
            carrier_phase_var_cycles2: 0.0,
            doppler_hz: Hertz(0.0),
            doppler_var_hz2: 0.0,
            cn0_dbhz: 45.0,
            lock_flags: LockFlags {
                code_lock,
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
    fn measured_ionosphere_recovers_band_specific_l1_l2_delay() {
        let geometry_m = 24_000_000.0;
        let l1_delay_m = 8.0;
        let l2_delay_m = first_order_ionosphere_code_delay_m(
            Meters(l1_delay_m),
            signal_spec_gps_l1_ca(),
            signal_spec_gps_l2_py(),
        )
        .expect("L2 delay")
        .0;
        let observations = measured_ionosphere_from_obs_epochs(
            &[dual_frequency_epoch(
                geometry_m + l1_delay_m,
                geometry_m + l2_delay_m,
                true,
                true,
                1.5,
                2.5,
            )],
            SignalBand::L1,
            SignalBand::L2,
        );

        assert_eq!(observations.len(), 1);
        let observation = &observations[0];
        assert_eq!(observation.code_status, "ok");
        assert_eq!(observation.code_reason, "ok");
        let measured_geometry_free_m = observation.code_geometry_free_m.expect("geometry-free code");
        assert!(
            (measured_geometry_free_m - (l2_delay_m - l1_delay_m)).abs() < 1.0e-6,
            "measured={measured_geometry_free_m} expected={}",
            l2_delay_m - l1_delay_m
        );
        assert!((observation.code_delay_band_1_m.expect("L1 delay") - l1_delay_m).abs() < 1.0e-6);
        assert!((observation.code_delay_band_2_m.expect("L2 delay") - l2_delay_m).abs() < 1.0e-6);
        assert!((observation.code_geometry_free_var_m2.expect("geometry-free variance") - 4.0).abs() < 1.0e-12);
    }

    #[test]
    fn measured_ionosphere_rejects_pairs_without_code_lock() {
        let observations = measured_ionosphere_from_obs_epochs(
            &[dual_frequency_epoch(24_000_000.0, 24_000_006.0, true, false, 1.0, 1.0)],
            SignalBand::L1,
            SignalBand::L2,
        );

        assert_eq!(observations.len(), 1);
        assert_eq!(observations[0].code_status, "invalid");
        assert_eq!(observations[0].code_reason, "code_lock_invalid");
        assert!(observations[0].code_delay_band_1_m.is_none());
        assert!(observations[0].code_delay_band_2_m.is_none());
    }
}
