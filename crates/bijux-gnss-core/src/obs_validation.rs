#![allow(missing_docs)]
#![allow(dead_code)]

use std::collections::{BTreeMap, BTreeSet};

use serde::{Deserialize, Serialize};

use crate::api::{
    ConventionsConfig, ObsEpoch, SatId, Seconds, SignalBand,
    OBSERVATION_DOPPLER_MODEL_TRACKED_CARRIER_IF_OFFSET,
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
            if sat.cn0_dbhz < conventions.min_cn0_dbhz || sat.cn0_dbhz > conventions.max_cn0_dbhz
            {
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
    model == OBSERVATION_DOPPLER_MODEL_TRACKED_CARRIER_IF_OFFSET
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
    )
}

#[cfg(test)]
mod tests {
    use super::validate_obs_epochs;
    use crate::api::{
        Constellation, Cycles, Hertz, LockFlags, Meters, ObsEpoch, ObsMetadata, ObsSatellite,
        ObservationEpochDecision, ObservationStatus, ReceiverRole, ReceiverSampleTrace, SatId,
        Seconds, SigId, SignalBand, SignalCode,
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
                metadata: ObsMetadata { doppler_model: doppler_model.to_string(), ..ObsMetadata::default() },
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
    fn validate_obs_epochs_rejects_non_finite_cn0() {
        let epoch = observation_epoch_with_models_and_cn0("tracked_carrier_hz_minus_intermediate_freq", f64::NAN);

        let error = validate_obs_epochs(&[epoch]).expect_err("non-finite cn0 must fail");
        assert_eq!(error, "non-finite cn0");
    }

    #[test]
    fn validate_obs_epochs_rejects_negative_cn0() {
        let epoch = observation_epoch_with_models_and_cn0("tracked_carrier_hz_minus_intermediate_freq", -1.0);

        let error = validate_obs_epochs(&[epoch]).expect_err("negative cn0 must fail");
        assert_eq!(error, "cn0 out of bounds");
    }

    #[test]
    fn validate_obs_epochs_rejects_excessive_cn0() {
        let epoch = observation_epoch_with_models_and_cn0("tracked_carrier_hz_minus_intermediate_freq", 81.0);

        let error = validate_obs_epochs(&[epoch]).expect_err("excessive cn0 must fail");
        assert_eq!(error, "cn0 out of bounds");
    }

    #[test]
    fn validate_obs_epochs_rejects_missing_observation_lock_state() {
        let mut epoch =
            observation_epoch_with_models_and_cn0("tracked_carrier_hz_minus_intermediate_freq", 45.0);
        epoch.sats[0].metadata.observation_lock_state.clear();

        let error =
            validate_obs_epochs(&[epoch]).expect_err("missing observation lock state must fail");
        assert_eq!(error, "missing observation lock state");
    }

    #[test]
    fn validate_obs_epochs_rejects_unknown_observation_lock_state() {
        let mut epoch =
            observation_epoch_with_models_and_cn0("tracked_carrier_hz_minus_intermediate_freq", 45.0);
        epoch.sats[0].metadata.observation_lock_state = "transitional".to_string();

        let error =
            validate_obs_epochs(&[epoch]).expect_err("unknown observation lock state must fail");
        assert_eq!(error, "unknown observation lock state");
    }
}
