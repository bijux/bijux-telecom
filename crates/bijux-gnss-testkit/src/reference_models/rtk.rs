//! Independent RTK differencing for synthetic truth scenarios.

use std::collections::BTreeMap;

use bijux_gnss_core::api::{
    AmbiguityId, Constellation, GlonassFrequencyChannel, ObsEpoch, ObsSatellite, ObservationStatus,
    SigId, GLONASS_L1_CARRIER_HZ, GLONASS_L1_CHANNEL_SPACING_HZ,
};
use bijux_gnss_nav::api::{
    RtkDoubleDifferenceObservation, RtkEpochAlignmentEvidence,
    RtkGlonassInterFrequencyBiasEvidence, RtkGlonassInterFrequencyBiasStatus,
    RtkSingleDifferenceCovarianceEvidence, RtkSingleDifferenceObservation,
    RtkSourceObservationVariance, RTK_EPOCH_ALIGNMENT_TOLERANCE_S,
};

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
        let base_receive_time_s = base_epoch.t_rx_s.0;
        let rover_receive_time_s = rover_epoch.t_rx_s.0;
        let delta_s = (base_receive_time_s - rover_receive_time_s).abs();
        if !base_receive_time_s.is_finite()
            || !rover_receive_time_s.is_finite()
            || delta_s > RTK_EPOCH_ALIGNMENT_TOLERANCE_S
        {
            continue;
        }

        differences.push(RtkSingleDifferenceObservation {
            sig: rover_observation.signal_id,
            min_cn0_dbhz: rover_observation.cn0_dbhz.min(base_observation.cn0_dbhz),
            multipath_suspect: rover_observation.multipath_suspect
                || base_observation.multipath_suspect,
            glonass_frequency_channel: matching_glonass_frequency_channel(
                rover_observation,
                base_observation,
            ),
            rover_pseudorange_m: rover_observation.pseudorange_m.0,
            rover_signal_timing: rover_observation.timing,
            base_pseudorange_m: base_observation.pseudorange_m.0,
            base_signal_timing: base_observation.timing,
            epoch_alignment: RtkEpochAlignmentEvidence {
                base_receive_time_s,
                rover_receive_time_s,
                delta_s,
                tolerance_s: RTK_EPOCH_ALIGNMENT_TOLERANCE_S,
            },
            covariance_evidence: covariance_evidence(rover_observation, base_observation),
            code_m: rover_observation.pseudorange_m.0 - base_observation.pseudorange_m.0,
            phase_cycles: rover_observation.carrier_phase_cycles.0
                - base_observation.carrier_phase_cycles.0,
            doppler_hz: rover_observation.doppler_hz.0 - base_observation.doppler_hz.0,
            code_variance_m2: rover_observation.pseudorange_var_m2
                + base_observation.pseudorange_var_m2,
            phase_variance_cycles2: rover_observation.carrier_phase_var_cycles2
                + base_observation.carrier_phase_var_cycles2,
            ambiguity_rover: ambiguity_id(rover_observation),
            ambiguity_base: ambiguity_id(base_observation),
        });
    }
    differences.sort_by_key(|difference| difference.sig);
    differences
}

fn matching_glonass_frequency_channel(
    rover_observation: &ObsSatellite,
    base_observation: &ObsSatellite,
) -> Option<GlonassFrequencyChannel> {
    if rover_observation.signal_id.sat.constellation != Constellation::Glonass {
        return None;
    }
    let rover_channel = glonass_frequency_channel_from_observation(rover_observation)?;
    let base_channel = glonass_frequency_channel_from_observation(base_observation)?;
    (rover_channel == base_channel).then_some(rover_channel)
}

fn glonass_frequency_channel_from_observation(
    observation: &ObsSatellite,
) -> Option<GlonassFrequencyChannel> {
    if observation.signal_id.sat.constellation != Constellation::Glonass {
        return None;
    }
    let offset = (observation.metadata.signal.carrier_hz.value() - GLONASS_L1_CARRIER_HZ.value())
        / GLONASS_L1_CHANNEL_SPACING_HZ.value();
    if !offset.is_finite() {
        return None;
    }
    let channel = offset.round();
    if (offset - channel).abs() > 1.0e-6 {
        return None;
    }
    GlonassFrequencyChannel::new(channel as i8)
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
    let Some(reference) =
        observations.iter().find(|observation| observation.sig == reference_signal)
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
        if observation.epoch_alignment != reference.epoch_alignment {
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
            epoch_alignment: observation.epoch_alignment,
            covariance_evidence: bijux_gnss_nav::api::RtkDoubleDifferenceCovarianceEvidence {
                signal: observation.covariance_evidence,
                reference: reference.covariance_evidence,
            },
            glonass_inter_frequency_bias: glonass_inter_frequency_bias_evidence(
                observation,
                reference,
            ),
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

fn glonass_inter_frequency_bias_evidence(
    signal: &RtkSingleDifferenceObservation,
    reference: &RtkSingleDifferenceObservation,
) -> RtkGlonassInterFrequencyBiasEvidence {
    if signal.sig.sat.constellation != Constellation::Glonass {
        return RtkGlonassInterFrequencyBiasEvidence::default();
    }
    let status = match (signal.glonass_frequency_channel, reference.glonass_frequency_channel) {
        (Some(signal_channel), Some(reference_channel)) if signal_channel == reference_channel => {
            RtkGlonassInterFrequencyBiasStatus::BiasHandled
        }
        (Some(_), Some(_)) => RtkGlonassInterFrequencyBiasStatus::CalibrationRequired,
        _ => RtkGlonassInterFrequencyBiasStatus::ChannelEvidenceMissing,
    };
    RtkGlonassInterFrequencyBiasEvidence {
        status,
        signal_channel: signal.glonass_frequency_channel,
        reference_channel: reference.glonass_frequency_channel,
        code_bias_m: 0.0,
        phase_bias_cycles: 0.0,
    }
}

fn usable_for_single_difference(observation: &ObsSatellite) -> bool {
    observation.observation_status == ObservationStatus::Accepted
        && observation.lock_flags.code_lock
        && observation.lock_flags.carrier_lock
        && observation.pseudorange_m.0.is_finite()
        && observation.carrier_phase_cycles.0.is_finite()
        && observation.doppler_hz.0.is_finite()
}

fn covariance_evidence(
    rover_observation: &ObsSatellite,
    base_observation: &ObsSatellite,
) -> RtkSingleDifferenceCovarianceEvidence {
    RtkSingleDifferenceCovarianceEvidence {
        rover: source_variance(rover_observation),
        base: source_variance(base_observation),
        rover_base_code_covariance_m2: 0.0,
        rover_base_phase_covariance_cycles2: 0.0,
        rover_base_doppler_covariance_hz2: 0.0,
        shared_code_covariance_m2: 0.0,
        shared_phase_covariance_cycles2: 0.0,
        shared_doppler_covariance_hz2: 0.0,
    }
}

fn source_variance(observation: &ObsSatellite) -> RtkSourceObservationVariance {
    RtkSourceObservationVariance {
        code_m2: observation.pseudorange_var_m2,
        phase_cycles2: observation.carrier_phase_var_cycles2,
        doppler_hz2: observation.doppler_var_hz2,
        shared_clock_code_m2: observation
            .error_model
            .as_ref()
            .map(|model| model.clock_error_m.0.powi(2))
            .filter(|variance| variance.is_finite() && *variance >= 0.0)
            .unwrap_or(0.0),
    }
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

    fn obs_satellite(
        prn: u8,
        pseudorange_m: f64,
        carrier_cycles: f64,
        cn0_dbhz: f64,
    ) -> ObsSatellite {
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
        let base =
            epoch(vec![obs_satellite(3, 20.0, 100.0, 42.0), obs_satellite(7, 30.0, 200.0, 48.0)]);
        let rover =
            epoch(vec![obs_satellite(3, 24.0, 103.0, 40.0), obs_satellite(7, 34.0, 205.0, 45.0)]);

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
