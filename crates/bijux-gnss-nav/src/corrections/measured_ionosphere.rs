#![allow(missing_docs)]

use std::collections::BTreeMap;

use bijux_gnss_core::api::{
    signal_cycles_to_meters, ObsEpoch, ObsSatellite, SatId, SigId, SignalBand,
};

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
    pub phase_geometry_free_m: Option<f64>,
    pub leveled_phase_geometry_free_m: Option<f64>,
    pub phase_delay_band_1_m: Option<f64>,
    pub phase_delay_band_2_m: Option<f64>,
    pub phase_level_bias_m: Option<f64>,
    pub phase_arc_reset: bool,
    pub code_status: String,
    pub code_reason: String,
    pub phase_status: String,
    pub phase_reason: String,
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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum MeasuredIonospherePhaseStatus {
    Ok,
    MissingFrequency,
    UnsupportedBandPair,
    ConstellationMismatch,
    TimeSystemMismatch,
    CarrierLockInvalid,
    VarianceInvalid,
    FrequencyInvalid,
    BiasUnavailable,
}

#[derive(Debug, Clone, Copy, Default)]
struct PhaseArcState {
    level_bias_m: Option<f64>,
}

pub fn measured_ionosphere_from_obs_epochs(
    epochs: &[ObsEpoch],
    band_1: SignalBand,
    band_2: SignalBand,
) -> Vec<MeasuredIonosphereObservation> {
    let mut out = Vec::new();
    let mut phase_arc_by_sat = BTreeMap::<SatId, PhaseArcState>::new();

    for epoch in epochs {
        let mut by_sat: BTreeMap<SatId, Vec<&ObsSatellite>> = BTreeMap::new();
        for sat in &epoch.sats {
            by_sat.entry(sat.signal_id.sat).or_default().push(sat);
        }

        for (sat, sats) in by_sat {
            let first = sats.iter().find(|candidate| candidate.signal_id.band == band_1).copied();
            let second = sats.iter().find(|candidate| candidate.signal_id.band == band_2).copied();
            let arc_state = phase_arc_by_sat.entry(sat).or_default();
            let phase_arc_broken =
                should_reset_phase_arc(epoch.discontinuity, sat, band_1, band_2, first, second);
            if phase_arc_broken {
                arc_state.level_bias_m = None;
            }

            let mut observation = measured_ionosphere_from_pair(
                epoch.epoch_idx,
                epoch.t_rx_s.0,
                sat,
                band_1,
                band_2,
                first,
                second,
                arc_state.level_bias_m,
                phase_arc_broken,
            );
            observation.phase_arc_reset = phase_arc_broken
                || (arc_state.level_bias_m.is_none() && observation.phase_level_bias_m.is_some());
            arc_state.level_bias_m = observation.phase_level_bias_m;
            out.push(observation);
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
    prior_phase_level_bias_m: Option<f64>,
    phase_arc_reset: bool,
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

    let (
        phase_geometry_free_m,
        leveled_phase_geometry_free_m,
        phase_delay_band_1_m,
        phase_delay_band_2_m,
        phase_level_bias_m,
        phase_status,
    ) = match dual_frequency_pair_issue(sat, band_1, band_2, first, second) {
        Some(issue) => (
            None,
            None,
            None,
            None,
            None,
            measured_ionosphere_phase_status_from_pair_issue(issue),
        ),
        None => {
            let (Some(first), Some(second)) = (first, second) else {
                unreachable!("compatible dual-frequency pairs must include both observations");
            };
            evaluate_measured_ionosphere_phase(
                first,
                second,
                code_geometry_free_m,
                prior_phase_level_bias_m,
            )
        }
    };

    let (code_status, code_reason) = code_status_reason(code_status);
    let (phase_status, phase_reason) = phase_status_reason(phase_status);

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
        phase_geometry_free_m,
        leveled_phase_geometry_free_m,
        phase_delay_band_1_m,
        phase_delay_band_2_m,
        phase_level_bias_m,
        phase_arc_reset,
        code_status,
        code_reason,
        phase_status,
        phase_reason,
    }
}

fn should_reset_phase_arc(
    epoch_discontinuity: bool,
    sat: SatId,
    band_1: SignalBand,
    band_2: SignalBand,
    first: Option<&ObsSatellite>,
    second: Option<&ObsSatellite>,
) -> bool {
    epoch_discontinuity
        || dual_frequency_pair_issue(sat, band_1, band_2, first, second).is_some()
        || [first, second]
            .into_iter()
            .flatten()
            .any(|observation| {
                observation.lock_flags.cycle_slip || !observation.lock_flags.carrier_lock
            })
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

fn evaluate_measured_ionosphere_phase(
    first: &ObsSatellite,
    second: &ObsSatellite,
    code_geometry_free_m: Option<f64>,
    prior_phase_level_bias_m: Option<f64>,
) -> (
    Option<f64>,
    Option<f64>,
    Option<f64>,
    Option<f64>,
    Option<f64>,
    MeasuredIonospherePhaseStatus,
) {
    if !first.lock_flags.carrier_lock || !second.lock_flags.carrier_lock {
        return (
            None,
            None,
            None,
            None,
            None,
            MeasuredIonospherePhaseStatus::CarrierLockInvalid,
        );
    }
    if !first.carrier_phase_var_cycles2.is_finite()
        || !second.carrier_phase_var_cycles2.is_finite()
        || first.carrier_phase_var_cycles2 < 0.0
        || second.carrier_phase_var_cycles2 < 0.0
    {
        return (
            None,
            None,
            None,
            None,
            None,
            MeasuredIonospherePhaseStatus::VarianceInvalid,
        );
    }

    let f1_hz = first.metadata.signal.carrier_hz.value();
    let f2_hz = second.metadata.signal.carrier_hz.value();
    if !f1_hz.is_finite() || !f2_hz.is_finite() || f1_hz <= 0.0 || f2_hz <= 0.0 {
        return (
            None,
            None,
            None,
            None,
            None,
            MeasuredIonospherePhaseStatus::FrequencyInvalid,
        );
    }

    let f1_2 = f1_hz * f1_hz;
    let f2_2 = f2_hz * f2_hz;
    let denom = f1_2 - f2_2;
    if !denom.is_finite() || denom.abs() <= f64::EPSILON {
        return (
            None,
            None,
            None,
            None,
            None,
            MeasuredIonospherePhaseStatus::FrequencyInvalid,
        );
    }

    let geometry_free_m = signal_cycles_to_meters(first.carrier_phase_cycles, first.metadata.signal).0
        - signal_cycles_to_meters(second.carrier_phase_cycles, second.metadata.signal).0;
    let phase_level_bias_m = prior_phase_level_bias_m.or_else(|| {
        code_geometry_free_m.map(|code_geometry_free_m| geometry_free_m - code_geometry_free_m)
    });
    let Some(phase_level_bias_m) = phase_level_bias_m else {
        return (
            Some(geometry_free_m),
            None,
            None,
            None,
            None,
            MeasuredIonospherePhaseStatus::BiasUnavailable,
        );
    };

    let leveled_geometry_free_m = geometry_free_m - phase_level_bias_m;
    let scale_band_1 = f2_2 / denom;
    let scale_band_2 = f1_2 / denom;
    let delay_band_1_m = leveled_geometry_free_m * scale_band_1;
    let delay_band_2_m = leveled_geometry_free_m * scale_band_2;

    (
        Some(geometry_free_m),
        Some(leveled_geometry_free_m),
        Some(delay_band_1_m),
        Some(delay_band_2_m),
        Some(phase_level_bias_m),
        MeasuredIonospherePhaseStatus::Ok,
    )
}

fn code_status_reason(status: MeasuredIonosphereCodeStatus) -> (String, String) {
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

fn phase_status_reason(status: MeasuredIonospherePhaseStatus) -> (String, String) {
    match status {
        MeasuredIonospherePhaseStatus::Ok => ("ok".to_string(), "ok".to_string()),
        MeasuredIonospherePhaseStatus::MissingFrequency => {
            ("invalid".to_string(), "missing_frequency".to_string())
        }
        MeasuredIonospherePhaseStatus::UnsupportedBandPair => {
            ("invalid".to_string(), "unsupported_band_pair".to_string())
        }
        MeasuredIonospherePhaseStatus::ConstellationMismatch => {
            ("invalid".to_string(), "constellation_mismatch".to_string())
        }
        MeasuredIonospherePhaseStatus::TimeSystemMismatch => {
            ("invalid".to_string(), "time_system_mismatch".to_string())
        }
        MeasuredIonospherePhaseStatus::CarrierLockInvalid => {
            ("invalid".to_string(), "carrier_lock_invalid".to_string())
        }
        MeasuredIonospherePhaseStatus::VarianceInvalid => {
            ("invalid".to_string(), "variance_invalid".to_string())
        }
        MeasuredIonospherePhaseStatus::FrequencyInvalid => {
            ("invalid".to_string(), "frequency_invalid".to_string())
        }
        MeasuredIonospherePhaseStatus::BiasUnavailable => {
            ("invalid".to_string(), "phase_level_bias_unavailable".to_string())
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

fn measured_ionosphere_phase_status_from_pair_issue(
    issue: DualFrequencyPairIssue,
) -> MeasuredIonospherePhaseStatus {
    match issue {
        DualFrequencyPairIssue::MissingFrequency => MeasuredIonospherePhaseStatus::MissingFrequency,
        DualFrequencyPairIssue::UnsupportedBandPair => {
            MeasuredIonospherePhaseStatus::UnsupportedBandPair
        }
        DualFrequencyPairIssue::ConstellationMismatch => {
            MeasuredIonospherePhaseStatus::ConstellationMismatch
        }
        DualFrequencyPairIssue::TimeSystemMismatch => {
            MeasuredIonospherePhaseStatus::TimeSystemMismatch
        }
    }
}

#[cfg(test)]
mod tests {
    use super::measured_ionosphere_from_obs_epochs;
    use bijux_gnss_core::api::{
        first_order_ionosphere_code_delay_m, signal_meters_to_cycles, signal_spec_gps_l1_ca,
        signal_spec_gps_l2_py, Constellation, Hertz, LockFlags, Meters, ObsEpoch, ObsMetadata,
        ObsSatellite, ObservationEpochDecision, ObservationStatus, ReceiverRole,
        ReceiverSampleTrace, SatId, Seconds, SigId, SignalBand, SignalCode,
    };

    struct EpochTerms {
        epoch_idx: u64,
        l1_code_m: f64,
        l2_code_m: f64,
        phase_bias_m: f64,
        code_lock: bool,
        carrier_lock: bool,
        cycle_slip: bool,
    }

    fn dual_frequency_epoch(terms: EpochTerms) -> ObsEpoch {
        let sat = SatId { constellation: Constellation::Gps, prn: 11 };
        let l1_signal = signal_spec_gps_l1_ca();
        let l2_signal = signal_spec_gps_l2_py();
        let geometry_free_code_m = terms.l2_code_m - terms.l1_code_m;
        let geometry_free_phase_m = geometry_free_code_m + terms.phase_bias_m;
        let reference_phase_m = 22_000_000.0;

        ObsEpoch {
            t_rx_s: Seconds(terms.epoch_idx as f64),
            source_time: ReceiverSampleTrace::from_sample_index(terms.epoch_idx, 1_000.0),
            gps_week: None,
            tow_s: None,
            epoch_idx: terms.epoch_idx,
            discontinuity: false,
            valid: true,
            processing_ms: None,
            role: ReceiverRole::Rover,
            sats: vec![
                make_satellite(
                    sat,
                    SignalBand::L1,
                    SignalCode::Ca,
                    l1_signal,
                    terms.l1_code_m,
                    signal_meters_to_cycles(Meters(reference_phase_m + geometry_free_phase_m), l1_signal).0,
                    terms.code_lock,
                    terms.carrier_lock,
                    terms.cycle_slip,
                ),
                make_satellite(
                    sat,
                    SignalBand::L2,
                    SignalCode::Py,
                    l2_signal,
                    terms.l2_code_m,
                    signal_meters_to_cycles(Meters(reference_phase_m), l2_signal).0,
                    terms.code_lock,
                    terms.carrier_lock,
                    terms.cycle_slip,
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
        carrier_phase_cycles: f64,
        code_lock: bool,
        carrier_lock: bool,
        cycle_slip: bool,
    ) -> ObsSatellite {
        ObsSatellite {
            signal_id: SigId { sat, band, code },
            pseudorange_m: Meters(pseudorange_m),
            pseudorange_var_m2: 1.0,
            carrier_phase_cycles: bijux_gnss_core::api::Cycles(carrier_phase_cycles),
            carrier_phase_var_cycles2: 0.01,
            doppler_hz: Hertz(0.0),
            doppler_var_hz2: 0.0,
            cn0_dbhz: 45.0,
            lock_flags: LockFlags {
                code_lock,
                carrier_lock,
                bit_lock: false,
                cycle_slip,
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
            &[dual_frequency_epoch(EpochTerms {
                epoch_idx: 0,
                l1_code_m: geometry_m + l1_delay_m,
                l2_code_m: geometry_m + l2_delay_m,
                phase_bias_m: 4.25,
                code_lock: true,
                carrier_lock: true,
                cycle_slip: false,
            })],
            SignalBand::L1,
            SignalBand::L2,
        );

        assert_eq!(observations.len(), 1);
        let observation = &observations[0];
        let measured_geometry_free_m = observation.code_geometry_free_m.expect("geometry-free code");
        assert!(
            (measured_geometry_free_m - (l2_delay_m - l1_delay_m)).abs() < 1.0e-6,
            "measured={measured_geometry_free_m} expected={}",
            l2_delay_m - l1_delay_m
        );
        assert!((observation.code_delay_band_1_m.expect("L1 delay") - l1_delay_m).abs() < 1.0e-6);
        assert!((observation.code_delay_band_2_m.expect("L2 delay") - l2_delay_m).abs() < 1.0e-6);
        assert_eq!(observation.phase_status, "ok");
        assert!((observation.phase_delay_band_1_m.expect("phase L1 delay") - l1_delay_m).abs() < 1.0e-6);
    }

    #[test]
    fn measured_ionosphere_rejects_pairs_without_code_lock() {
        let observations = measured_ionosphere_from_obs_epochs(
            &[dual_frequency_epoch(EpochTerms {
                epoch_idx: 0,
                l1_code_m: 24_000_000.0,
                l2_code_m: 24_000_006.0,
                phase_bias_m: 4.25,
                code_lock: false,
                carrier_lock: true,
                cycle_slip: false,
            })],
            SignalBand::L1,
            SignalBand::L2,
        );

        assert_eq!(observations.len(), 1);
        assert_eq!(observations[0].code_status, "invalid");
        assert_eq!(observations[0].code_reason, "code_lock_invalid");
        assert!(observations[0].code_delay_band_1_m.is_none());
        assert!(observations[0].code_delay_band_2_m.is_none());
        assert_eq!(observations[0].phase_status, "invalid");
        assert_eq!(observations[0].phase_reason, "phase_level_bias_unavailable");
    }

    #[test]
    fn measured_ionosphere_phase_tracks_code_leveled_arc_changes() {
        let geometry_m = 24_000_000.0;
        let l1_delays_m = [5.0, 7.0, 6.25];
        let observations = measured_ionosphere_from_obs_epochs(
            &l1_delays_m
                .into_iter()
                .enumerate()
                .map(|(epoch_idx, l1_delay_m)| {
                    let l2_delay_m = first_order_ionosphere_code_delay_m(
                        Meters(l1_delay_m),
                        signal_spec_gps_l1_ca(),
                        signal_spec_gps_l2_py(),
                    )
                    .expect("L2 delay")
                    .0;
                    dual_frequency_epoch(EpochTerms {
                        epoch_idx: epoch_idx as u64,
                        l1_code_m: geometry_m + l1_delay_m,
                        l2_code_m: geometry_m + l2_delay_m,
                        phase_bias_m: 3.5,
                        code_lock: true,
                        carrier_lock: true,
                        cycle_slip: false,
                    })
                })
                .collect::<Vec<_>>(),
            SignalBand::L1,
            SignalBand::L2,
        );

        assert_eq!(observations.len(), 3);
        for (observation, expected_l1_delay_m) in observations.iter().zip(l1_delays_m) {
            assert_eq!(observation.phase_status, "ok");
            assert!(!observation.phase_arc_reset || observation.epoch_idx == 0);
            assert!(
                (observation.phase_delay_band_1_m.expect("phase L1 delay") - expected_l1_delay_m).abs()
                    < 1.0e-6
            );
        }
        assert_eq!(observations[0].phase_arc_reset, true);
        assert_eq!(observations[1].phase_arc_reset, false);
        assert_eq!(observations[2].phase_arc_reset, false);
    }

    #[test]
    fn measured_ionosphere_phase_relevels_after_cycle_slip() {
        let geometry_m = 24_000_000.0;
        let before_slip_l1_delay_m = 5.0;
        let after_slip_l1_delay_m = 9.0;
        let before_slip_l2_delay_m = first_order_ionosphere_code_delay_m(
            Meters(before_slip_l1_delay_m),
            signal_spec_gps_l1_ca(),
            signal_spec_gps_l2_py(),
        )
        .expect("L2 delay")
        .0;
        let after_slip_l2_delay_m = first_order_ionosphere_code_delay_m(
            Meters(after_slip_l1_delay_m),
            signal_spec_gps_l1_ca(),
            signal_spec_gps_l2_py(),
        )
        .expect("L2 delay")
        .0;
        let observations = measured_ionosphere_from_obs_epochs(
            &[
                dual_frequency_epoch(EpochTerms {
                    epoch_idx: 0,
                    l1_code_m: geometry_m + before_slip_l1_delay_m,
                    l2_code_m: geometry_m + before_slip_l2_delay_m,
                    phase_bias_m: 3.5,
                    code_lock: true,
                    carrier_lock: true,
                    cycle_slip: false,
                }),
                dual_frequency_epoch(EpochTerms {
                    epoch_idx: 1,
                    l1_code_m: geometry_m + after_slip_l1_delay_m,
                    l2_code_m: geometry_m + after_slip_l2_delay_m,
                    phase_bias_m: 7.0,
                    code_lock: true,
                    carrier_lock: true,
                    cycle_slip: true,
                }),
            ],
            SignalBand::L1,
            SignalBand::L2,
        );

        assert_eq!(observations.len(), 2);
        assert_eq!(observations[0].phase_arc_reset, true);
        assert_eq!(observations[1].phase_arc_reset, true);
        assert_eq!(observations[1].phase_status, "ok");
        assert!(
            (observations[1].phase_delay_band_1_m.expect("releveled phase delay")
                - after_slip_l1_delay_m)
                .abs()
                < 1.0e-6
        );
        assert!(
            (observations[1].phase_level_bias_m.expect("new bias")
                - observations[0].phase_level_bias_m.expect("old bias"))
                .abs()
                > 1.0
        );
    }
}
