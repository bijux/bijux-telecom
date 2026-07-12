#![allow(missing_docs)]
#![allow(dead_code)]

use std::collections::{BTreeMap, BTreeSet};

use serde::{Deserialize, Serialize};

use crate::api::{
    signal_registry, Constellation, ConventionsConfig, ObsEpoch, ObsSatellite, SatId, Seconds,
    SignalBand, OBSERVATION_DOPPLER_MODEL_TRACKED_CARRIER_IF_OFFSET,
};

/// Event describing missing band observations over time.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BandLagEvent {
    pub sat: SatId,
    pub band: SignalBand,
    pub lag_epochs: u64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InterFrequencyAlignmentReport {
    pub total_events: usize,
    pub max_lag_epochs: u64,
    pub events: Vec<BandLagEvent>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DualFrequencyPairStatus {
    Complete,
    Incomplete,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DualFrequencyPairIssue {
    MissingFrequency,
    LockInvalid,
    SignalDefinitionInvalid,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DualFrequencyObservationPair {
    pub epoch_idx: u64,
    pub sat: SatId,
    pub band_1: SignalBand,
    pub band_2: SignalBand,
    pub status: DualFrequencyPairStatus,
    pub issue: Option<DualFrequencyPairIssue>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DualFrequencyObservationReport {
    pub observed_pairs: usize,
    pub complete_pairs: usize,
    pub incomplete_pairs: usize,
    pub l1_l2_pairs: usize,
    pub l1_l5_pairs: usize,
    pub e1_e5_pairs: usize,
    pub b1_b2_pairs: usize,
    pub pairs: Vec<DualFrequencyObservationPair>,
}

pub fn check_inter_frequency_alignment(epochs: &[ObsEpoch]) -> InterFrequencyAlignmentReport {
    let mut last_seen: BTreeMap<(SatId, SignalBand), u64> = BTreeMap::new();
    let mut events = Vec::new();
    for epoch in epochs {
        let mut present: BTreeMap<SatId, Vec<SignalBand>> = BTreeMap::new();
        for sat in &epoch.sats {
            present.entry(sat.signal_id.sat).or_default().push(sat.signal_id.band);
            last_seen.entry((sat.signal_id.sat, sat.signal_id.band)).or_insert(epoch.epoch_idx);
        }
        for (sat, bands) in present {
            for ((seen_sat, seen_band), last_epoch) in &last_seen {
                if *seen_sat != sat {
                    continue;
                }
                if bands.contains(seen_band) {
                    continue;
                }
                let lag = epoch.epoch_idx.saturating_sub(*last_epoch);
                if lag > 0 {
                    events.push(BandLagEvent { sat, band: *seen_band, lag_epochs: lag });
                }
            }
        }
        for sat in &epoch.sats {
            last_seen.insert((sat.signal_id.sat, sat.signal_id.band), epoch.epoch_idx);
        }
    }
    let max_lag = events.iter().map(|event| event.lag_epochs).max().unwrap_or(0);
    InterFrequencyAlignmentReport { total_events: events.len(), max_lag_epochs: max_lag, events }
}

pub fn check_dual_frequency_observations(epochs: &[ObsEpoch]) -> DualFrequencyObservationReport {
    let mut pairs = Vec::new();
    let mut observed_pairs = 0usize;
    let mut complete_pairs = 0usize;
    let mut l1_l2_pairs = 0usize;
    let mut l1_l5_pairs = 0usize;
    let mut e1_e5_pairs = 0usize;
    let mut b1_b2_pairs = 0usize;

    for epoch in epochs {
        let mut by_sat: BTreeMap<SatId, Vec<&ObsSatellite>> = BTreeMap::new();
        for sat in &epoch.sats {
            by_sat.entry(sat.signal_id.sat).or_default().push(sat);
        }

        for (sat, observations) in by_sat {
            for (band_1, band_2) in supported_dual_frequency_band_pairs() {
                let obs_1 = observations.iter().find(|obs| obs.signal_id.band == *band_1).copied();
                let obs_2 = observations.iter().find(|obs| obs.signal_id.band == *band_2).copied();
                if obs_1.is_none() && obs_2.is_none() {
                    continue;
                }

                observed_pairs += 1;
                let issue = dual_frequency_pair_issue(obs_1, obs_2);
                let status = if issue.is_none() {
                    complete_pairs += 1;
                    match (band_1, band_2) {
                        (SignalBand::L1, SignalBand::L2) => l1_l2_pairs += 1,
                        (SignalBand::L1, SignalBand::L5) => l1_l5_pairs += 1,
                        (SignalBand::E1, SignalBand::E5) => e1_e5_pairs += 1,
                        (SignalBand::B1, SignalBand::B2) => b1_b2_pairs += 1,
                        _ => {}
                    }
                    DualFrequencyPairStatus::Complete
                } else {
                    DualFrequencyPairStatus::Incomplete
                };

                pairs.push(DualFrequencyObservationPair {
                    epoch_idx: epoch.epoch_idx,
                    sat,
                    band_1: *band_1,
                    band_2: *band_2,
                    status,
                    issue,
                });
            }
        }
    }

    DualFrequencyObservationReport {
        observed_pairs,
        complete_pairs,
        incomplete_pairs: observed_pairs.saturating_sub(complete_pairs),
        l1_l2_pairs,
        l1_l5_pairs,
        e1_e5_pairs,
        b1_b2_pairs,
        pairs,
    }
}

pub fn validate_obs_epochs(epochs: &[ObsEpoch]) -> Result<(), String> {
    let conventions = ConventionsConfig::default();
    let mut last_t: Option<Seconds> = None;
    for epoch in epochs {
        if let Some(prev) = last_t {
            if epoch.t_rx_s.0 < prev.0 {
                return Err("non-monotonic t_rx_s".to_string());
            }
        }
        last_t = Some(epoch.t_rx_s);
        if !epoch.t_rx_s.0.is_finite() {
            return Err("t_rx_s is not finite".to_string());
        }
        let mut seen = BTreeSet::new();
        for sat in &epoch.sats {
            if !seen.insert(sat.signal_id) {
                return Err("duplicate signal_id within epoch".to_string());
            }
            if !sat.pseudorange_m.0.is_finite()
                || !sat.carrier_phase_cycles.0.is_finite()
                || !sat.doppler_hz.0.is_finite()
            {
                return Err("non-finite observable value".to_string());
            }
            if !sat.pseudorange_var_m2.is_finite()
                || !sat.carrier_phase_var_cycles2.is_finite()
                || !sat.doppler_var_hz2.is_finite()
            {
                return Err("non-finite variance".to_string());
            }
            if sat.pseudorange_var_m2 < 0.0
                || sat.carrier_phase_var_cycles2 < 0.0
                || sat.doppler_var_hz2 < 0.0
            {
                return Err("negative variance".to_string());
            }
            if !sat.cn0_dbhz.is_finite() {
                return Err("non-finite cn0".to_string());
            }
            if sat.cn0_dbhz < conventions.min_cn0_dbhz || sat.cn0_dbhz > conventions.max_cn0_dbhz {
                return Err("cn0 out of bounds".to_string());
            }
            if sat.metadata.doppler_model.is_empty() {
                return Err("missing doppler model".to_string());
            }
            if !known_doppler_model(&sat.metadata.doppler_model) {
                return Err("unknown doppler model".to_string());
            }
            if sat.metadata.observation_lock_state.is_empty() {
                return Err("missing observation lock state".to_string());
            }
            if !known_observation_lock_state(&sat.metadata.observation_lock_state) {
                return Err("unknown observation lock state".to_string());
            }
        }
    }
    Ok(())
}

fn known_doppler_model(model: &str) -> bool {
    matches!(model, OBSERVATION_DOPPLER_MODEL_TRACKED_CARRIER_IF_OFFSET | "unavailable")
}

fn known_observation_lock_state(state: &str) -> bool {
    matches!(
        state,
        "acquired"
            | "pull_in"
            | "locked"
            | "degraded"
            | "lost"
            | "reacquired"
            | "cycle_slip"
            | "inactive"
            | "imported"
    )
}

pub fn supported_dual_frequency_band_pairs() -> &'static [(SignalBand, SignalBand)] {
    &[
        (SignalBand::L1, SignalBand::L2),
        (SignalBand::L1, SignalBand::L5),
        (SignalBand::E1, SignalBand::E5),
        (SignalBand::B1, SignalBand::B2),
    ]
}

pub fn supported_dual_frequency_band_pairs_for_constellation(
    constellation: Constellation,
) -> &'static [(SignalBand, SignalBand)] {
    match constellation {
        Constellation::Gps => &[(SignalBand::L1, SignalBand::L2), (SignalBand::L1, SignalBand::L5)],
        Constellation::Galileo => &[(SignalBand::E1, SignalBand::E5)],
        Constellation::Beidou => &[(SignalBand::B1, SignalBand::B2)],
        _ => &[],
    }
}

fn dual_frequency_pair_issue(
    band_1: Option<&ObsSatellite>,
    band_2: Option<&ObsSatellite>,
) -> Option<DualFrequencyPairIssue> {
    let (Some(band_1), Some(band_2)) = (band_1, band_2) else {
        return Some(DualFrequencyPairIssue::MissingFrequency);
    };

    if !has_registered_signal_definition(band_1) || !has_registered_signal_definition(band_2) {
        return Some(DualFrequencyPairIssue::SignalDefinitionInvalid);
    }

    if !band_1.lock_flags.code_lock
        || !band_1.lock_flags.carrier_lock
        || !band_2.lock_flags.code_lock
        || !band_2.lock_flags.carrier_lock
    {
        return Some(DualFrequencyPairIssue::LockInvalid);
    }

    None
}

fn has_registered_signal_definition(observation: &ObsSatellite) -> bool {
    let registered = signal_registry(
        observation.signal_id.sat.constellation,
        observation.signal_id.band,
        observation.signal_id.code,
    );
    let Some(entry) = registered else {
        return false;
    };

    let signal = observation.metadata.signal;
    signal.constellation == entry.spec.constellation
        && signal.band == entry.spec.band
        && signal.code == entry.spec.code
}

#[cfg(test)]
mod tests {
    use super::{
        check_dual_frequency_observations, supported_dual_frequency_band_pairs_for_constellation,
        validate_obs_epochs, DualFrequencyPairIssue, DualFrequencyPairStatus,
    };
    use crate::api::{
        signal_registry, Constellation, Cycles, Hertz, LockFlags, Meters, ObsEpoch, ObsMetadata,
        ObsSatellite, ObservationEpochDecision, ObservationStatus, ReceiverRole,
        ReceiverSampleTrace, SatId, Seconds, SigId, SignalBand, SignalCode,
    };

    fn observation_epoch_with_models_and_cn0(doppler_model: &str, cn0_dbhz: f64) -> ObsEpoch {
        ObsEpoch {
            t_rx_s: Seconds(0.07),
            source_time: ReceiverSampleTrace::from_sample_index(286_440, 4_092_000.0),
            gps_week: None,
            tow_s: None,
            epoch_idx: 70,
            discontinuity: false,
            valid: true,
            processing_ms: None,
            role: ReceiverRole::Rover,
            sats: vec![ObsSatellite {
                signal_id: SigId {
                    sat: SatId { constellation: Constellation::Gps, prn: 4 },
                    band: SignalBand::L1,
                    code: SignalCode::Ca,
                },
                pseudorange_m: Meters(21_000_000.0),
                pseudorange_var_m2: 1.0,
                carrier_phase_cycles: Cycles(12.25),
                carrier_phase_var_cycles2: 1.0,
                doppler_hz: Hertz(125.0),
                doppler_var_hz2: 1.0,
                cn0_dbhz,
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
                    doppler_model: doppler_model.to_string(),
                    signal: signal_registry(Constellation::Gps, SignalBand::L1, SignalCode::Ca)
                        .expect("GPS L1 C/A signal must exist")
                        .spec,
                    ..ObsMetadata::default()
                },
            }],
            decision: ObservationEpochDecision::Accepted,
            decision_reason: None,
            manifest: None,
        }
    }

    #[test]
    fn validate_obs_epochs_rejects_missing_doppler_model() {
        let epoch = observation_epoch_with_models_and_cn0("", 45.0);

        let error = validate_obs_epochs(&[epoch]).expect_err("missing doppler model must fail");
        assert_eq!(error, "missing doppler model");
    }

    #[test]
    fn validate_obs_epochs_rejects_unknown_doppler_model() {
        let epoch = observation_epoch_with_models_and_cn0("legacy_absolute_doppler_hz", 45.0);

        let error = validate_obs_epochs(&[epoch]).expect_err("unknown doppler model must fail");
        assert_eq!(error, "unknown doppler model");
    }

    #[test]
    fn validate_obs_epochs_accepts_imported_observation_metadata() {
        let mut epoch = observation_epoch_with_models_and_cn0("unavailable", 45.0);
        epoch.sats[0].metadata.observation_lock_state = "imported".to_string();

        validate_obs_epochs(&[epoch]).expect("imported observation metadata must validate");
    }

    #[test]
    fn validate_obs_epochs_rejects_non_finite_cn0() {
        let epoch = observation_epoch_with_models_and_cn0(
            "tracked_carrier_hz_minus_intermediate_freq",
            f64::NAN,
        );

        let error = validate_obs_epochs(&[epoch]).expect_err("non-finite cn0 must fail");
        assert_eq!(error, "non-finite cn0");
    }

    #[test]
    fn validate_obs_epochs_rejects_negative_cn0() {
        let epoch = observation_epoch_with_models_and_cn0(
            "tracked_carrier_hz_minus_intermediate_freq",
            -1.0,
        );

        let error = validate_obs_epochs(&[epoch]).expect_err("negative cn0 must fail");
        assert_eq!(error, "cn0 out of bounds");
    }

    #[test]
    fn validate_obs_epochs_rejects_excessive_cn0() {
        let epoch = observation_epoch_with_models_and_cn0(
            "tracked_carrier_hz_minus_intermediate_freq",
            81.0,
        );

        let error = validate_obs_epochs(&[epoch]).expect_err("excessive cn0 must fail");
        assert_eq!(error, "cn0 out of bounds");
    }

    #[test]
    fn validate_obs_epochs_rejects_missing_observation_lock_state() {
        let mut epoch = observation_epoch_with_models_and_cn0(
            "tracked_carrier_hz_minus_intermediate_freq",
            45.0,
        );
        epoch.sats[0].metadata.observation_lock_state.clear();

        let error =
            validate_obs_epochs(&[epoch]).expect_err("missing observation lock state must fail");
        assert_eq!(error, "missing observation lock state");
    }

    #[test]
    fn validate_obs_epochs_rejects_unknown_observation_lock_state() {
        let mut epoch = observation_epoch_with_models_and_cn0(
            "tracked_carrier_hz_minus_intermediate_freq",
            45.0,
        );
        epoch.sats[0].metadata.observation_lock_state = "transitional".to_string();

        let error =
            validate_obs_epochs(&[epoch]).expect_err("unknown observation lock state must fail");
        assert_eq!(error, "unknown observation lock state");
    }

    fn dual_frequency_satellite(
        band: SignalBand,
        code: SignalCode,
        code_lock: bool,
        carrier_lock: bool,
    ) -> ObsSatellite {
        ObsSatellite {
            signal_id: SigId {
                sat: SatId { constellation: Constellation::Gps, prn: 7 },
                band,
                code,
            },
            pseudorange_m: Meters(20_000_000.0),
            pseudorange_var_m2: 1.0,
            carrier_phase_cycles: Cycles(42.0),
            carrier_phase_var_cycles2: 1.0,
            doppler_hz: Hertz(10.0),
            doppler_var_hz2: 1.0,
            cn0_dbhz: 45.0,
            lock_flags: LockFlags { code_lock, carrier_lock, bit_lock: true, cycle_slip: false },
            multipath_suspect: false,
            observation_status: ObservationStatus::Accepted,
            observation_reject_reasons: Vec::new(),
            elevation_deg: None,
            azimuth_deg: None,
            weight: None,
            timing: None,
            error_model: None,
            metadata: ObsMetadata {
                doppler_model: "tracked_carrier_hz_minus_intermediate_freq".to_string(),
                observation_lock_state: "locked".to_string(),
                signal: signal_registry(Constellation::Gps, band, code)
                    .expect("dual-frequency signal must exist")
                    .spec,
                ..ObsMetadata::default()
            },
        }
    }

    fn dual_frequency_epoch(sats: Vec<ObsSatellite>) -> ObsEpoch {
        ObsEpoch {
            t_rx_s: Seconds(1.0),
            source_time: ReceiverSampleTrace::from_sample_index(1024, 1_024.0),
            gps_week: None,
            tow_s: None,
            epoch_idx: 1,
            discontinuity: false,
            valid: true,
            processing_ms: None,
            role: ReceiverRole::Rover,
            sats,
            decision: ObservationEpochDecision::Accepted,
            decision_reason: Some("accepted_observables_present".to_string()),
            manifest: None,
        }
    }

    #[test]
    fn dual_frequency_report_marks_complete_l1_l2_pairs() {
        let report = check_dual_frequency_observations(&[dual_frequency_epoch(vec![
            dual_frequency_satellite(SignalBand::L1, SignalCode::Ca, true, true),
            dual_frequency_satellite(SignalBand::L2, SignalCode::Py, true, true),
        ])]);

        assert_eq!(report.observed_pairs, 2);
        assert_eq!(report.complete_pairs, 1);
        assert_eq!(report.incomplete_pairs, 1);
        assert_eq!(report.l1_l2_pairs, 1);
        assert_eq!(report.l1_l5_pairs, 0);
        assert_eq!(report.e1_e5_pairs, 0);
        assert_eq!(report.b1_b2_pairs, 0);
        assert_eq!(report.pairs[0].status, DualFrequencyPairStatus::Complete);
        assert_eq!(report.pairs[0].issue, None);
        assert_eq!(report.pairs[1].issue, Some(DualFrequencyPairIssue::MissingFrequency));
    }

    #[test]
    fn dual_frequency_report_marks_complete_l1_l5_pairs() {
        let report = check_dual_frequency_observations(&[dual_frequency_epoch(vec![
            dual_frequency_satellite(SignalBand::L1, SignalCode::Ca, true, true),
            dual_frequency_satellite(SignalBand::L5, SignalCode::Unknown, true, true),
        ])]);

        assert_eq!(report.observed_pairs, 2);
        assert_eq!(report.complete_pairs, 1);
        assert_eq!(report.incomplete_pairs, 1);
        assert_eq!(report.l1_l2_pairs, 0);
        assert_eq!(report.l1_l5_pairs, 1);
        assert_eq!(report.e1_e5_pairs, 0);
        assert_eq!(report.b1_b2_pairs, 0);
        assert_eq!(report.pairs[0].issue, Some(DualFrequencyPairIssue::MissingFrequency));
        assert_eq!(report.pairs[1].status, DualFrequencyPairStatus::Complete);
        assert_eq!(report.pairs[1].issue, None);
    }

    #[test]
    fn dual_frequency_report_marks_complete_e1_e5_pairs() {
        let report = check_dual_frequency_observations(&[dual_frequency_epoch(vec![
            ObsSatellite {
                signal_id: SigId {
                    sat: SatId { constellation: Constellation::Galileo, prn: 11 },
                    band: SignalBand::E1,
                    code: SignalCode::E1B,
                },
                metadata: ObsMetadata {
                    doppler_model: "tracked_carrier_hz_minus_intermediate_freq".to_string(),
                    observation_lock_state: "locked".to_string(),
                    signal: signal_registry(Constellation::Galileo, SignalBand::E1, SignalCode::E1B)
                        .expect("Galileo E1B signal must exist")
                        .spec,
                    ..ObsMetadata::default()
                },
                ..dual_frequency_satellite(SignalBand::L1, SignalCode::Ca, true, true)
            },
            ObsSatellite {
                signal_id: SigId {
                    sat: SatId { constellation: Constellation::Galileo, prn: 11 },
                    band: SignalBand::E5,
                    code: SignalCode::E5a,
                },
                metadata: ObsMetadata {
                    doppler_model: "tracked_carrier_hz_minus_intermediate_freq".to_string(),
                    observation_lock_state: "locked".to_string(),
                    signal: signal_registry(Constellation::Galileo, SignalBand::E5, SignalCode::E5a)
                        .expect("Galileo E5a signal must exist")
                        .spec,
                    ..ObsMetadata::default()
                },
                ..dual_frequency_satellite(SignalBand::L5, SignalCode::Unknown, true, true)
            },
        ])]);

        assert_eq!(report.complete_pairs, 1);
        assert_eq!(report.e1_e5_pairs, 1);
        assert_eq!(report.b1_b2_pairs, 0);
    }

    #[test]
    fn dual_frequency_report_marks_complete_b1_b2_pairs() {
        let report = check_dual_frequency_observations(&[dual_frequency_epoch(vec![
            ObsSatellite {
                signal_id: SigId {
                    sat: SatId { constellation: Constellation::Beidou, prn: 11 },
                    band: SignalBand::B1,
                    code: SignalCode::B1I,
                },
                metadata: ObsMetadata {
                    doppler_model: "tracked_carrier_hz_minus_intermediate_freq".to_string(),
                    observation_lock_state: "locked".to_string(),
                    signal: signal_registry(Constellation::Beidou, SignalBand::B1, SignalCode::B1I)
                        .expect("BeiDou B1I signal must exist")
                        .spec,
                    ..ObsMetadata::default()
                },
                ..dual_frequency_satellite(SignalBand::L1, SignalCode::Ca, true, true)
            },
            ObsSatellite {
                signal_id: SigId {
                    sat: SatId { constellation: Constellation::Beidou, prn: 11 },
                    band: SignalBand::B2,
                    code: SignalCode::B2I,
                },
                metadata: ObsMetadata {
                    doppler_model: "tracked_carrier_hz_minus_intermediate_freq".to_string(),
                    observation_lock_state: "locked".to_string(),
                    signal: signal_registry(Constellation::Beidou, SignalBand::B2, SignalCode::B2I)
                        .expect("BeiDou B2I signal must exist")
                        .spec,
                    ..ObsMetadata::default()
                },
                ..dual_frequency_satellite(SignalBand::L2, SignalCode::Py, true, true)
            },
        ])]);

        assert_eq!(report.complete_pairs, 1);
        assert_eq!(report.e1_e5_pairs, 0);
        assert_eq!(report.b1_b2_pairs, 1);
    }

    #[test]
    fn supported_dual_frequency_pairs_follow_constellation_preferences() {
        assert_eq!(
            supported_dual_frequency_band_pairs_for_constellation(Constellation::Gps),
            &[(SignalBand::L1, SignalBand::L2), (SignalBand::L1, SignalBand::L5)]
        );
        assert_eq!(
            supported_dual_frequency_band_pairs_for_constellation(Constellation::Galileo),
            &[(SignalBand::E1, SignalBand::E5)]
        );
        assert_eq!(
            supported_dual_frequency_band_pairs_for_constellation(Constellation::Beidou),
            &[(SignalBand::B1, SignalBand::B2)]
        );
        assert!(supported_dual_frequency_band_pairs_for_constellation(Constellation::Glonass).is_empty());
    }

    #[test]
    fn dual_frequency_report_marks_lock_invalid_pairs() {
        let report = check_dual_frequency_observations(&[dual_frequency_epoch(vec![
            dual_frequency_satellite(SignalBand::L1, SignalCode::Ca, true, true),
            dual_frequency_satellite(SignalBand::L2, SignalCode::Py, false, true),
        ])]);

        assert_eq!(report.complete_pairs, 0);
        assert_eq!(report.incomplete_pairs, 2);
        assert_eq!(report.pairs[0].issue, Some(DualFrequencyPairIssue::LockInvalid));
        assert_eq!(report.pairs[1].issue, Some(DualFrequencyPairIssue::MissingFrequency));
    }

    #[test]
    fn dual_frequency_report_marks_signal_definition_invalid_pairs() {
        let mut l1 = dual_frequency_satellite(SignalBand::L1, SignalCode::Ca, true, true);
        l1.metadata.signal = signal_registry(Constellation::Gps, SignalBand::L2, SignalCode::Py)
            .expect("GPS L2 P(Y) signal must exist")
            .spec;
        let report = check_dual_frequency_observations(&[dual_frequency_epoch(vec![
            l1,
            dual_frequency_satellite(SignalBand::L2, SignalCode::Py, true, true),
        ])]);

        assert_eq!(report.complete_pairs, 0);
        assert_eq!(report.pairs[0].issue, Some(DualFrequencyPairIssue::SignalDefinitionInvalid));
    }
}
