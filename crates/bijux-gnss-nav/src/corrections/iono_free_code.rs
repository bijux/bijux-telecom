#![allow(missing_docs)]

use std::collections::BTreeMap;

use bijux_gnss_core::api::{GpsTime, ObsEpoch, ObsSatellite, SatId, SigId, SignalBand};

use crate::corrections::biases::{iono_free_code_bias_m_at, CodeBiasProvider};
use crate::corrections::dual_frequency::{dual_frequency_pair_issue, DualFrequencyPairIssue};

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct IonoFreeCodeObservation {
    pub epoch_idx: u64,
    pub t_rx_s: f64,
    pub sat: SatId,
    pub signal_1: Option<SigId>,
    pub signal_2: Option<SigId>,
    pub band_1: SignalBand,
    pub band_2: SignalBand,
    pub f1_hz: f64,
    pub f2_hz: f64,
    pub code_m: Option<f64>,
    pub code_bias_m: Option<f64>,
    pub corrected_code_m: Option<f64>,
    pub variance_m2: Option<f64>,
    pub status: String,
    pub reason: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum IonoFreeCodeStatus {
    Ok,
    MissingFrequency,
    UnsupportedBandPair,
    ConstellationMismatch,
    TimeSystemMismatch,
    CodeLockInvalid,
    VarianceInvalid,
    FrequencyInvalid,
}

#[derive(Clone, Copy)]
struct IonoFreeCodePairInput<'a> {
    epoch_idx: u64,
    t_rx_s: f64,
    gps_time: Option<GpsTime>,
    sat: SatId,
    band_1: SignalBand,
    band_2: SignalBand,
    first: Option<&'a ObsSatellite>,
    second: Option<&'a ObsSatellite>,
    biases: Option<&'a dyn CodeBiasProvider>,
}

pub fn iono_free_code_from_obs_epochs(
    epochs: &[ObsEpoch],
    band_1: SignalBand,
    band_2: SignalBand,
) -> Vec<IonoFreeCodeObservation> {
    iono_free_code_from_obs_epochs_with_biases(epochs, band_1, band_2, None)
}

pub fn iono_free_code_from_obs_epochs_with_biases(
    epochs: &[ObsEpoch],
    band_1: SignalBand,
    band_2: SignalBand,
    biases: Option<&dyn CodeBiasProvider>,
) -> Vec<IonoFreeCodeObservation> {
    let mut out = Vec::new();

    for epoch in epochs {
        let mut by_sat: BTreeMap<SatId, Vec<&ObsSatellite>> = BTreeMap::new();
        for sat in &epoch.sats {
            by_sat.entry(sat.signal_id.sat).or_default().push(sat);
        }

        for (sat, sats) in by_sat {
            let first = sats.iter().find(|candidate| candidate.signal_id.band == band_1).copied();
            let second = sats.iter().find(|candidate| candidate.signal_id.band == band_2).copied();
            let gps_time = epoch_gps_time(epoch);
            out.push(iono_free_code_from_pair_with_biases(IonoFreeCodePairInput {
                epoch_idx: epoch.epoch_idx,
                t_rx_s: epoch.t_rx_s.0,
                gps_time,
                sat,
                band_1,
                band_2,
                first,
                second,
                biases,
            }));
        }
    }

    out
}

pub(crate) fn iono_free_code_from_pair(
    epoch_idx: u64,
    t_rx_s: f64,
    sat: SatId,
    band_1: SignalBand,
    band_2: SignalBand,
    first: Option<&ObsSatellite>,
    second: Option<&ObsSatellite>,
) -> IonoFreeCodeObservation {
    iono_free_code_from_pair_with_biases(IonoFreeCodePairInput {
        epoch_idx,
        t_rx_s,
        gps_time: None,
        sat,
        band_1,
        band_2,
        first,
        second,
        biases: None,
    })
}

fn iono_free_code_from_pair_with_biases(
    input: IonoFreeCodePairInput<'_>,
) -> IonoFreeCodeObservation {
    let f1_hz = input
        .first
        .map(|observation| observation.metadata.signal.carrier_hz.value())
        .unwrap_or(0.0);
    let f2_hz = input
        .second
        .map(|observation| observation.metadata.signal.carrier_hz.value())
        .unwrap_or(0.0);
    let signal_1 = input.first.map(|observation| observation.signal_id);
    let signal_2 = input.second.map(|observation| observation.signal_id);

    let (code_m, variance_m2, status) = match dual_frequency_pair_issue(
        input.sat,
        input.band_1,
        input.band_2,
        input.first,
        input.second,
    ) {
        Some(issue) => (None, None, iono_free_code_status_from_pair_issue(issue)),
        None => {
            let (Some(first), Some(second)) = (input.first, input.second) else {
                unreachable!("compatible dual-frequency pairs must include both observations");
            };
            evaluate_iono_free_code(first, second)
        }
    };
    let code_bias_m = match (input.biases, signal_1, signal_2, code_m) {
        (Some(provider), Some(signal_1), Some(signal_2), Some(_)) => {
            iono_free_code_bias_m_at(provider, signal_1, signal_2, f1_hz, f2_hz, input.gps_time)
        }
        _ => None,
    };
    let corrected_code_m = code_m.zip(code_bias_m).map(|(code_m, bias_m)| code_m - bias_m);
    let (status_text, reason) = status_reason(status);

    IonoFreeCodeObservation {
        epoch_idx: input.epoch_idx,
        t_rx_s: input.t_rx_s,
        sat: input.sat,
        signal_1,
        signal_2,
        band_1: input.band_1,
        band_2: input.band_2,
        f1_hz,
        f2_hz,
        code_m,
        code_bias_m,
        corrected_code_m,
        variance_m2,
        status: status_text,
        reason,
    }
}

fn epoch_gps_time(epoch: &ObsEpoch) -> Option<GpsTime> {
    Some(GpsTime { week: epoch.gps_week?, tow_s: epoch.tow_s?.0 })
}

fn evaluate_iono_free_code(
    first: &ObsSatellite,
    second: &ObsSatellite,
) -> (Option<f64>, Option<f64>, IonoFreeCodeStatus) {
    if !first.lock_flags.code_lock || !second.lock_flags.code_lock {
        return (None, None, IonoFreeCodeStatus::CodeLockInvalid);
    }
    if !first.pseudorange_var_m2.is_finite()
        || !second.pseudorange_var_m2.is_finite()
        || first.pseudorange_var_m2 < 0.0
        || second.pseudorange_var_m2 < 0.0
    {
        return (None, None, IonoFreeCodeStatus::VarianceInvalid);
    }

    let f1_hz = first.metadata.signal.carrier_hz.value();
    let f2_hz = second.metadata.signal.carrier_hz.value();
    if !f1_hz.is_finite() || !f2_hz.is_finite() || f1_hz <= 0.0 || f2_hz <= 0.0 {
        return (None, None, IonoFreeCodeStatus::FrequencyInvalid);
    }

    let f1_2 = f1_hz * f1_hz;
    let f2_2 = f2_hz * f2_hz;
    let denom = f1_2 - f2_2;
    if !denom.is_finite() || denom.abs() <= f64::EPSILON {
        return (None, None, IonoFreeCodeStatus::FrequencyInvalid);
    }

    let weight_1 = f1_2 / denom;
    let weight_2 = -f2_2 / denom;
    let code_m = weight_1 * first.pseudorange_m.0 + weight_2 * second.pseudorange_m.0;
    let variance_m2 =
        weight_1.powi(2) * first.pseudorange_var_m2 + weight_2.powi(2) * second.pseudorange_var_m2;
    (Some(code_m), Some(variance_m2), IonoFreeCodeStatus::Ok)
}

fn status_reason(status: IonoFreeCodeStatus) -> (String, String) {
    match status {
        IonoFreeCodeStatus::Ok => ("ok".to_string(), "ok".to_string()),
        IonoFreeCodeStatus::MissingFrequency => {
            ("invalid".to_string(), "missing_frequency".to_string())
        }
        IonoFreeCodeStatus::UnsupportedBandPair => {
            ("invalid".to_string(), "unsupported_band_pair".to_string())
        }
        IonoFreeCodeStatus::ConstellationMismatch => {
            ("invalid".to_string(), "constellation_mismatch".to_string())
        }
        IonoFreeCodeStatus::TimeSystemMismatch => {
            ("invalid".to_string(), "time_system_mismatch".to_string())
        }
        IonoFreeCodeStatus::CodeLockInvalid => {
            ("invalid".to_string(), "code_lock_invalid".to_string())
        }
        IonoFreeCodeStatus::VarianceInvalid => {
            ("invalid".to_string(), "variance_invalid".to_string())
        }
        IonoFreeCodeStatus::FrequencyInvalid => {
            ("invalid".to_string(), "frequency_invalid".to_string())
        }
    }
}

fn iono_free_code_status_from_pair_issue(issue: DualFrequencyPairIssue) -> IonoFreeCodeStatus {
    match issue {
        DualFrequencyPairIssue::MissingFrequency => IonoFreeCodeStatus::MissingFrequency,
        DualFrequencyPairIssue::UnsupportedBandPair => IonoFreeCodeStatus::UnsupportedBandPair,
        DualFrequencyPairIssue::ConstellationMismatch => IonoFreeCodeStatus::ConstellationMismatch,
        DualFrequencyPairIssue::TimeSystemMismatch => IonoFreeCodeStatus::TimeSystemMismatch,
    }
}

#[cfg(test)]
mod tests {
    use super::{iono_free_code_from_obs_epochs, iono_free_code_from_obs_epochs_with_biases};
    use crate::corrections::biases::{CodeBias, SignalCodeBiases};
    use bijux_gnss_core::api::{
        Constellation, Cycles, GpsTime, Hertz, LockFlags, Meters, ObsEpoch, ObsMetadata,
        ObsSatellite, ObsSignalTiming, ObservationEpochDecision, ObservationStatus, ReceiverRole,
        ReceiverSampleTrace, SatId, Seconds, SigId, SignalBand, SignalCode,
    };
    use bijux_gnss_signal::api::{
        signal_spec_galileo_e1b, signal_spec_gps_l1_ca, signal_spec_gps_l2_py, signal_spec_gps_l5,
    };

    fn dual_frequency_epoch(
        second_band: SignalBand,
        second_code: SignalCode,
        second_signal: bijux_gnss_core::api::SignalSpec,
        pseudorange_1_m: f64,
        pseudorange_2_m: f64,
        code_lock_1: bool,
        code_lock_2: bool,
        variance_1_m2: f64,
        variance_2_m2: f64,
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
                    pseudorange_1_m,
                    code_lock_1,
                    variance_1_m2,
                ),
                make_satellite(
                    sat,
                    second_band,
                    second_code,
                    second_signal,
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
            carrier_phase_var_cycles2: f64::NAN,
            doppler_hz: Hertz(0.0),
            doppler_var_hz2: 1.0,
            cn0_dbhz: 45.0,
            lock_flags: LockFlags {
                code_lock,
                carrier_lock: false,
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
    fn iono_free_code_removes_first_order_ionosphere_for_l1_l2() {
        let base_range_m = 20_200_000.0;
        let iono_l1_m = 5.0;
        let l1 = signal_spec_gps_l1_ca();
        let l2 = signal_spec_gps_l2_py();
        let iono_l2_m = iono_l1_m * (l1.carrier_hz.value() * l1.carrier_hz.value())
            / (l2.carrier_hz.value() * l2.carrier_hz.value());
        let epoch = dual_frequency_epoch(
            SignalBand::L2,
            SignalCode::Py,
            l2,
            base_range_m + iono_l1_m,
            base_range_m + iono_l2_m,
            true,
            true,
            1.0,
            4.0,
        );

        let observations = iono_free_code_from_obs_epochs(&[epoch], SignalBand::L1, SignalBand::L2);

        assert_eq!(observations.len(), 1);
        assert_eq!(observations[0].status, "ok");
        assert!((observations[0].code_m.expect("iono-free code") - base_range_m).abs() < 1.0e-6);
        let f1_2 = l1.carrier_hz.value() * l1.carrier_hz.value();
        let f2_2 = l2.carrier_hz.value() * l2.carrier_hz.value();
        let denom = f1_2 - f2_2;
        let expected_variance = (f1_2 / denom).powi(2) * 1.0 + (f2_2 / denom).powi(2) * 4.0;
        assert!(
            (observations[0].variance_m2.expect("iono-free variance") - expected_variance).abs()
                < 1.0e-9
        );
    }

    #[test]
    fn iono_free_code_removes_first_order_ionosphere_for_l1_l5() {
        let base_range_m = 20_200_000.0;
        let iono_l1_m = 5.0;
        let l1 = signal_spec_gps_l1_ca();
        let l5 = signal_spec_gps_l5();
        let iono_l5_m = iono_l1_m * (l1.carrier_hz.value() * l1.carrier_hz.value())
            / (l5.carrier_hz.value() * l5.carrier_hz.value());
        let epoch = dual_frequency_epoch(
            SignalBand::L5,
            SignalCode::Unknown,
            l5,
            base_range_m + iono_l1_m,
            base_range_m + iono_l5_m,
            true,
            true,
            1.0,
            1.0,
        );

        let observations = iono_free_code_from_obs_epochs(&[epoch], SignalBand::L1, SignalBand::L5);

        assert_eq!(observations.len(), 1);
        assert_eq!(observations[0].status, "ok");
        assert!((observations[0].code_m.expect("iono-free code") - base_range_m).abs() < 1.0e-6);
    }

    #[test]
    fn iono_free_code_does_not_require_carrier_lock_or_carrier_variance() {
        let l2 = signal_spec_gps_l2_py();
        let epoch = dual_frequency_epoch(
            SignalBand::L2,
            SignalCode::Py,
            l2,
            20_200_005.0,
            20_200_008.235308182,
            true,
            true,
            1.0,
            1.0,
        );

        let observations = iono_free_code_from_obs_epochs(&[epoch], SignalBand::L1, SignalBand::L2);

        assert_eq!(observations[0].status, "ok");
        assert!(observations[0].code_m.is_some());
    }

    #[test]
    fn iono_free_code_rejects_missing_code_lock() {
        let l2 = signal_spec_gps_l2_py();
        let epoch = dual_frequency_epoch(
            SignalBand::L2,
            SignalCode::Py,
            l2,
            20_200_005.0,
            20_200_008.235308182,
            true,
            false,
            1.0,
            1.0,
        );

        let observations = iono_free_code_from_obs_epochs(&[epoch], SignalBand::L1, SignalBand::L2);

        assert_eq!(observations[0].status, "invalid");
        assert_eq!(observations[0].reason, "code_lock_invalid");
        assert!(observations[0].code_m.is_none());
    }

    #[test]
    fn iono_free_code_reports_signal_specific_bias_and_corrected_code() {
        let sat = SatId { constellation: Constellation::Gps, prn: 11 };
        let l1_signal = SigId { sat, band: SignalBand::L1, code: SignalCode::Ca };
        let l2_signal = SigId { sat, band: SignalBand::L2, code: SignalCode::Py };
        let bias_table = SignalCodeBiases::from_biases([
            CodeBias { sig: l1_signal, bias_m: 2.0 },
            CodeBias { sig: l2_signal, bias_m: -0.5 },
        ]);
        let observations = iono_free_code_from_obs_epochs_with_biases(
            &[dual_frequency_epoch(
                SignalBand::L2,
                SignalCode::Py,
                signal_spec_gps_l2_py(),
                20_200_010.0,
                20_200_005.0,
                true,
                true,
                1.0,
                1.0,
            )],
            SignalBand::L1,
            SignalBand::L2,
            Some(&bias_table),
        );

        assert_eq!(observations[0].signal_1, Some(l1_signal));
        assert_eq!(observations[0].signal_2, Some(l2_signal));
        assert!(observations[0].code_bias_m.is_some());
        assert!(observations[0].corrected_code_m.is_some());
        assert_ne!(
            observations[0].code_m.expect("raw iono-free code"),
            observations[0].corrected_code_m.expect("bias-corrected iono-free code")
        );
    }

    #[test]
    fn iono_free_code_rejects_unsupported_band_pairs() {
        let observations = iono_free_code_from_obs_epochs(
            &[dual_frequency_epoch(
                SignalBand::L2,
                SignalCode::Py,
                signal_spec_gps_l2_py(),
                20_200_010.0,
                20_200_005.0,
                true,
                true,
                1.0,
                1.0,
            )],
            SignalBand::L2,
            SignalBand::L5,
        );

        assert_eq!(observations.len(), 1);
        assert_eq!(observations[0].status, "invalid");
        assert_eq!(observations[0].reason, "unsupported_band_pair");
    }

    #[test]
    fn iono_free_code_rejects_constellation_mismatches() {
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
                    20_200_010.0,
                    true,
                    1.0,
                ),
                make_satellite(
                    sat,
                    SignalBand::L2,
                    SignalCode::Py,
                    signal_spec_galileo_e1b(),
                    20_200_005.0,
                    true,
                    1.0,
                ),
            ],
            decision: ObservationEpochDecision::Accepted,
            decision_reason: Some("accepted_observables_present".to_string()),
            manifest: None,
        };

        let observations = iono_free_code_from_obs_epochs(&[epoch], SignalBand::L1, SignalBand::L2);

        assert_eq!(observations[0].reason, "constellation_mismatch");
    }

    #[test]
    fn iono_free_code_rejects_time_system_mismatches() {
        let sat = SatId { constellation: Constellation::Gps, prn: 11 };
        let mut first = make_satellite(
            sat,
            SignalBand::L1,
            SignalCode::Ca,
            signal_spec_gps_l1_ca(),
            20_200_010.0,
            true,
            1.0,
        );
        let mut second = make_satellite(
            sat,
            SignalBand::L2,
            SignalCode::Py,
            signal_spec_gps_l2_py(),
            20_200_005.0,
            true,
            1.0,
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

        let observations = iono_free_code_from_obs_epochs(&[epoch], SignalBand::L1, SignalBand::L2);

        assert_eq!(observations[0].reason, "time_system_mismatch");
    }
}
