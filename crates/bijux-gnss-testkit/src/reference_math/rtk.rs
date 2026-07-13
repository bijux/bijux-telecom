//! Independent RTK observation differencing for synthetic truth scenarios.

use std::collections::BTreeMap;

use bijux_gnss_core::api::{AmbiguityId, ObsEpoch, ObsSatellite, ObservationStatus, SigId};
use bijux_gnss_nav::api::{RtkDoubleDifferenceObservation, RtkSingleDifferenceObservation};

pub(crate) fn single_differences_from_epochs(
    base_epoch: &ObsEpoch,
    rover_epoch: &ObsEpoch,
) -> Vec<RtkSingleDifferenceObservation> {
    let mut base_by_signal = BTreeMap::new();
    for observation in &base_epoch.sats {
        base_by_signal.insert(observation.signal_id, observation);
    }

    let mut differences = Vec::new();
    for rover_observation in &rover_epoch.sats {
        let Some(base_observation) = base_by_signal.get(&rover_observation.signal_id) else {
            continue;
        };
        if !usable_for_single_difference(rover_observation)
            || !usable_for_single_difference(base_observation)
        {
            continue;
        }

        differences.push(RtkSingleDifferenceObservation {
            sig: rover_observation.signal_id,
            min_cn0_dbhz: rover_observation.cn0_dbhz.min(base_observation.cn0_dbhz),
            multipath_suspect: rover_observation.multipath_suspect || base_observation.multipath_suspect,
            rover_pseudorange_m: rover_observation.pseudorange_m.0,
            rover_signal_timing: rover_observation.timing,
            base_pseudorange_m: base_observation.pseudorange_m.0,
            base_signal_timing: base_observation.timing,
            code_m: rover_observation.pseudorange_m.0 - base_observation.pseudorange_m.0,
            phase_cycles: rover_observation.carrier_phase_cycles.0
                - base_observation.carrier_phase_cycles.0,
            doppler_hz: rover_observation.doppler_hz.0 - base_observation.doppler_hz.0,
            code_variance_m2: rover_observation.pseudorange_var_m2 + base_observation.pseudorange_var_m2,
            phase_variance_cycles2: rover_observation.carrier_phase_var_cycles2
                + base_observation.carrier_phase_var_cycles2,
            ambiguity_rover: ambiguity_id(rover_observation),
            ambiguity_base: ambiguity_id(base_observation),
        });
    }
    differences.sort_by_key(|difference| difference.sig);
    differences
}

pub(crate) fn choose_reference_signal(
    observations: &[RtkSingleDifferenceObservation],
) -> Option<SigId> {
    observations
        .iter()
        .min_by(|left, right| {
            left.multipath_suspect
                .cmp(&right.multipath_suspect)
                .then_with(|| right.min_cn0_dbhz.total_cmp(&left.min_cn0_dbhz))
                .then_with(|| left.code_variance_m2.total_cmp(&right.code_variance_m2))
        })
        .map(|observation| observation.sig)
}

pub(crate) fn double_differences_from_single_differences(
    observations: &[RtkSingleDifferenceObservation],
    reference_signal: SigId,
) -> Vec<RtkDoubleDifferenceObservation> {
    let Some(reference) = observations.iter().find(|observation| observation.sig == reference_signal)
    else {
        return Vec::new();
    };

    let mut differences = Vec::new();
    for observation in observations {
        if observation.sig == reference_signal {
            continue;
        }
        if observation.sig.sat.constellation != reference.sig.sat.constellation {
            continue;
        }

        differences.push(RtkDoubleDifferenceObservation {
            sig: observation.sig,
            ref_sig: reference.sig,
            min_cn0_dbhz: observation.min_cn0_dbhz.min(reference.min_cn0_dbhz),
            multipath_suspect: observation.multipath_suspect || reference.multipath_suspect,
            rover_signal_pseudorange_m: observation.rover_pseudorange_m,
            rover_signal_timing: observation.rover_signal_timing,
            base_signal_pseudorange_m: observation.base_pseudorange_m,
            base_signal_timing: observation.base_signal_timing,
            rover_ref_pseudorange_m: reference.rover_pseudorange_m,
            rover_ref_signal_timing: reference.rover_signal_timing,
            base_ref_pseudorange_m: reference.base_pseudorange_m,
            base_ref_signal_timing: reference.base_signal_timing,
            code_m: observation.code_m - reference.code_m,
            phase_cycles: observation.phase_cycles - reference.phase_cycles,
            doppler_hz: observation.doppler_hz - reference.doppler_hz,
            code_variance_m2: observation.code_variance_m2 + reference.code_variance_m2,
            phase_variance_cycles2: observation.phase_variance_cycles2
                + reference.phase_variance_cycles2,
            canceled: vec![
                observation.ambiguity_rover.clone(),
                observation.ambiguity_base.clone(),
                reference.ambiguity_rover.clone(),
                reference.ambiguity_base.clone(),
            ],
        });
    }
    differences.sort_by_key(|difference| difference.sig);
    differences
}

fn usable_for_single_difference(observation: &ObsSatellite) -> bool {
    observation.observation_status == ObservationStatus::Accepted
        && observation.lock_flags.code_lock
        && observation.lock_flags.carrier_lock
        && observation.pseudorange_m.0.is_finite()
        && observation.carrier_phase_cycles.0.is_finite()
        && observation.doppler_hz.0.is_finite()
}

fn ambiguity_id(observation: &ObsSatellite) -> AmbiguityId {
    AmbiguityId {
        sig: observation.signal_id,
        signal: format!("{:?}", observation.metadata.signal.band),
    }
}

#[cfg(test)]
mod tests {
    use bijux_gnss_core::api::{
        Constellation, Cycles, Hertz, LockFlags, Meters, ObsEpoch, ObsMetadata, ObsSatellite,
        ObservationEpochDecision, ObservationStatus, ReceiverRole, SatId, Seconds, SigId,
        SignalBand, SignalCode, SignalSpec, GPS_L1_CA_CARRIER_HZ,
    };

    use super::{
        choose_reference_signal, double_differences_from_single_differences,
        single_differences_from_epochs,
    };

    fn obs_satellite(prn: u8, pseudorange_m: f64, carrier_cycles: f64, cn0_dbhz: f64) -> ObsSatellite {
        ObsSatellite {
            signal_id: SigId {
                sat: SatId { constellation: Constellation::Gps, prn },
                band: SignalBand::L1,
                code: SignalCode::Ca,
            },
            pseudorange_m: Meters(pseudorange_m),
            pseudorange_var_m2: 0.25,
            carrier_phase_cycles: Cycles(carrier_cycles),
            carrier_phase_var_cycles2: 0.04,
            doppler_hz: Hertz(0.0),
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
                tracking_mode: "synthetic".to_string(),
                integration_ms: 1,
                lock_quality: cn0_dbhz,
                smoothing_window: 0,
                smoothing_age: 0,
                smoothing_resets: 0,
                signal: SignalSpec {
                    constellation: Constellation::Gps,
                    band: SignalBand::L1,
                    code: SignalCode::Ca,
                    code_rate_hz: 1_023_000.0,
                    carrier_hz: GPS_L1_CA_CARRIER_HZ,
                },
                ..ObsMetadata::default()
            },
        }
    }

    fn epoch(sats: Vec<ObsSatellite>) -> ObsEpoch {
        ObsEpoch {
            t_rx_s: Seconds(0.0),
            source_time: Default::default(),
            gps_week: Some(2200),
            tow_s: Some(Seconds(0.0)),
            epoch_idx: 0,
            discontinuity: false,
            valid: true,
            processing_ms: None,
            role: ReceiverRole::Rover,
            sats,
            decision: ObservationEpochDecision::Accepted,
            decision_reason: Some("synthetic".to_string()),
            manifest: None,
        }
    }

    #[test]
    fn reference_differences_preserve_rover_minus_base_sign() {
        let base = epoch(vec![obs_satellite(3, 20.0, 100.0, 42.0), obs_satellite(7, 30.0, 200.0, 48.0)]);
        let rover = epoch(vec![obs_satellite(3, 24.0, 103.0, 40.0), obs_satellite(7, 34.0, 205.0, 45.0)]);

        let singles = single_differences_from_epochs(&base, &rover);
        let reference = choose_reference_signal(&singles).expect("reference signal");
        let doubles = double_differences_from_single_differences(&singles, reference);

        assert_eq!(singles.len(), 2);
        assert_eq!(singles[0].code_m, 4.0);
        assert_eq!(singles[1].phase_cycles, 5.0);
        assert_eq!(reference.sat.prn, 7);
        assert_eq!(doubles.len(), 1);
        assert_eq!(doubles[0].code_m, 0.0);
    }
}
