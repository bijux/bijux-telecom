use bijux_gnss_core::api::{
    Constellation, Cycles, GpsTime, Hertz, LockFlags, Meters, ObsEpoch as ObservationEpoch,
    ObsEpochManifest, ObsMetadata, ObsSatellite, ObsSignalTiming, ObservationEpochDecision,
    ObservationStatus, ReceiverSampleTrace, SatId, Seconds, SigId, SignalBand, SignalCode,
};
use bijux_gnss_signal::api::signal_spec_gps_l1_ca;

use super::timing::observation_epoch_id;
use super::SPEED_OF_LIGHT_MPS;
use super::{accepted_rover_observation_epoch, AcceptedRoverObservationEpochRequest};

pub(crate) fn nav_observation_epoch_fixture(epoch_idx: u64) -> ObservationEpoch {
    let receive_tow_s = epoch_idx as f64 * 0.001;
    let sats = (1..=4)
        .map(|prn| ObsSatellite {
            signal_id: SigId {
                sat: SatId { constellation: Constellation::Gps, prn },
                band: SignalBand::L1,
                code: SignalCode::Ca,
            },
            pseudorange_m: Meters(20_200_000.0 + prn as f64),
            pseudorange_var_m2: 100.0,
            carrier_phase_cycles: Cycles(0.0),
            carrier_phase_var_cycles2: 1.0,
            doppler_hz: Hertz(0.0),
            doppler_var_hz2: 1.0,
            cn0_dbhz: 45.0,
            lock_flags: LockFlags {
                code_lock: true,
                carrier_lock: true,
                bit_lock: true,
                cycle_slip: false,
            },
            multipath_suspect: false,
            observation_status: ObservationStatus::Accepted,
            observation_reject_reasons: Vec::new(),
            elevation_deg: Some(45.0),
            azimuth_deg: Some(0.0),
            weight: Some(1.0),
            timing: Some(ObsSignalTiming {
                signal_travel_time_s: Seconds((20_200_000.0 + prn as f64) / SPEED_OF_LIGHT_MPS),
                transmit_gps_time: GpsTime {
                    week: 0,
                    tow_s: receive_tow_s - ((20_200_000.0 + prn as f64) / SPEED_OF_LIGHT_MPS),
                },
            }),
            error_model: None,
            metadata: ObsMetadata {
                tracking_mode: "scalar".to_string(),
                integration_ms: 1,
                lock_quality: 1.0,
                smoothing_window: 0,
                smoothing_age: 0,
                smoothing_resets: 0,
                signal: signal_spec_gps_l1_ca(),
                tracking_lock_state: "locked".to_string(),
                observation_lock_state: "locked".to_string(),
                observation_lock_reason: Some("stable_tracking".to_string()),
                tracking_lock_quality: 1.0,
                ..ObsMetadata::default()
            },
        })
        .collect();
    accepted_rover_observation_epoch(AcceptedRoverObservationEpochRequest {
        t_rx_s: Seconds(receive_tow_s),
        source_time: ReceiverSampleTrace::from_sample_index(epoch_idx, 1_000.0),
        gps_week: None,
        tow_s: None,
        epoch_idx,
        discontinuity: false,
        sats,
        decision_reason: Some("accepted_observables_present".to_string()),
        manifest: Some(ObsEpochManifest {
            version: bijux_gnss_core::api::OBSERVATION_MODEL_VERSION,
            artifact_id: format!("obs-epoch-{epoch_idx:010}"),
            epoch_id: observation_epoch_id(epoch_idx, epoch_idx),
            source_epoch_idx: epoch_idx,
            source_sample_index: epoch_idx,
            source_time: ReceiverSampleTrace::from_sample_index(epoch_idx, 1_000.0),
            decision: ObservationEpochDecision::Accepted,
            downstream_profile_version:
                bijux_gnss_core::api::OBSERVATION_DOWNSTREAM_PROFILE_VERSION,
        }),
    })
}
