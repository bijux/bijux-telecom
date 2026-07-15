use super::{
    constellation_residual_rms, corrected_observation_records,
    position_broadcast_navigation_from_beidou_navigations,
    position_broadcast_navigation_from_galileo_navigations,
    position_broadcast_navigation_from_glonass_frames,
    position_broadcast_navigation_from_gps_ephemerides, position_observations_from_epoch,
    resolve_position_inputs, robust_weight, robust_weights, solve_weighted_least_squares,
    unknown_inter_system_time_offset_sats, ClockStateModel, PositionBroadcastNavigation,
    PositionEstimate, PositionObservation, PositionRobustWeighting, PositionSolver,
    SatelliteGeometry, SatelliteState, WorkingSetResidual, SPEED_OF_LIGHT_MPS,
};
use crate::estimation::position::navigation::{
    navigation_time_relationship_is_known, PositionObservationCorrectionChain,
    PositionObservationCorrectionKind,
};
use crate::orbits::beidou::{
    BeidouBroadcastNavigationData, BeidouClockCorrection, BeidouEphemeris,
    BeidouIonosphericCorrection, BeidouSignalHealth, BeidouSystemTime,
};
use crate::orbits::galileo::{
    GalileoBroadcastNavigationData, GalileoClockCorrection, GalileoEphemeris,
    GalileoIonosphericCorrection, GalileoIonosphericDisturbanceFlags, GalileoSignalHealth,
    GalileoSystemTime,
};
use crate::orbits::glonass::{
    GlonassAlmanacTimeData, GlonassBroadcastNavigationFrame, GlonassFrameTime,
    GlonassImmediateHealth, GlonassImmediateNavigationData, GlonassSatelliteType,
    GlonassStateVector, GlonassSystemTime,
};
use crate::orbits::gps::GpsEphemeris;
use crate::orbits::satellite_uncertainty::SatelliteStateUncertainty;
use bijux_gnss_core::api::{
    Constellation, Cycles, Hertz, LockFlags, MeasurementErrorModel, Meters, ObsEpoch, ObsMetadata,
    ObsSatellite, ObservationEpochDecision, ObservationStatus, ReceiverRole, ReceiverSampleTrace,
    SatId, Seconds, SigId, SignalBand, SignalCode, TrackingUncertainty,
};
use bijux_gnss_signal::api::signal_spec_gps_l1_ca;

#[path = "tests/correction_records.rs"]
mod correction_records;
#[path = "tests/least_squares.rs"]
mod least_squares;
#[path = "tests/navigation_inputs.rs"]
mod navigation_inputs;
#[path = "tests/residual_summary.rs"]
mod residual_summary;
#[path = "tests/robust_weighting.rs"]
mod robust_weighting;

#[test]
fn position_observation_without_signal_timing_is_valid_when_pseudorange_is_finite() {
    let observation = PositionObservation {
        sat: SatId { constellation: Constellation::Gps, prn: 13 },
        pseudorange_m: 24_000_000.0,
        doppler_hz: None,
        doppler_var_hz2: None,
        cn0_dbhz: 45.0,
        elevation_deg: None,
        weight: 1.0,
        gps_receive_time: None,
        signal_timing: None,
        signal_id: None,
    };

    assert!(super::position_observation_has_valid_satellite_time(&observation, 0.0));
}

#[test]
fn position_observations_from_epoch_prefers_l1_per_satellite() {
    let sat = SatId { constellation: Constellation::Gps, prn: 7 };
    let epoch = ObsEpoch {
        t_rx_s: Seconds(1000.0),
        source_time: ReceiverSampleTrace::from_sample_index(0, 1000.0),
        gps_week: Some(2000),
        tow_s: Some(Seconds(1000.0)),
        epoch_idx: 0,
        discontinuity: false,
        valid: true,
        processing_ms: None,
        role: ReceiverRole::Rover,
        sats: vec![
            ObsSatellite {
                signal_id: SigId { sat, band: SignalBand::L2, code: SignalCode::Py },
                pseudorange_m: Meters(22_000_010.0),
                pseudorange_var_m2: 1.0,
                carrier_phase_cycles: Cycles(10.0),
                carrier_phase_var_cycles2: 1.0,
                doppler_hz: Hertz(0.0),
                doppler_var_hz2: 1.0,
                cn0_dbhz: 35.0,
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
                metadata: ObsMetadata::default(),
            },
            ObsSatellite {
                signal_id: SigId { sat, band: SignalBand::L1, code: SignalCode::Ca },
                pseudorange_m: Meters(22_000_000.0),
                pseudorange_var_m2: 1.0,
                carrier_phase_cycles: Cycles(20.0),
                carrier_phase_var_cycles2: 1.0,
                doppler_hz: Hertz(0.0),
                doppler_var_hz2: 1.0,
                cn0_dbhz: 45.0,
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
                metadata: ObsMetadata::default(),
            },
        ],
        decision: ObservationEpochDecision::Accepted,
        decision_reason: None,
        manifest: None,
    };

    let observations = position_observations_from_epoch(&epoch);

    assert_eq!(observations.len(), 1);
    assert_eq!(observations[0].sat, sat);
    assert_eq!(observations[0].pseudorange_m, 22_000_000.0);
    assert_eq!(observations[0].doppler_hz, Some(0.0));
    assert_eq!(observations[0].doppler_var_hz2, Some(1.0));
    assert_eq!(observations[0].cn0_dbhz, 45.0);
    assert_eq!(
        observations[0].signal_id,
        Some(SigId { sat, band: SignalBand::L1, code: SignalCode::Ca })
    );
}

#[test]
fn position_observations_from_epoch_weight_with_observation_covariance() {
    let sat = SatId { constellation: Constellation::Gps, prn: 7 };
    let epoch = ObsEpoch {
        t_rx_s: Seconds(1000.0),
        source_time: ReceiverSampleTrace::from_sample_index(0, 1000.0),
        gps_week: Some(2000),
        tow_s: Some(Seconds(1000.0)),
        epoch_idx: 0,
        discontinuity: false,
        valid: true,
        processing_ms: None,
        role: ReceiverRole::Rover,
        sats: vec![ObsSatellite {
            signal_id: SigId { sat, band: SignalBand::L1, code: SignalCode::Ca },
            pseudorange_m: Meters(22_000_000.0),
            pseudorange_var_m2: 16.0,
            carrier_phase_cycles: Cycles(20.0),
            carrier_phase_var_cycles2: 0.01,
            doppler_hz: Hertz(0.0),
            doppler_var_hz2: 4.0,
            cn0_dbhz: 45.0,
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
            weight: Some(2.0),
            timing: None,
            error_model: Some(MeasurementErrorModel {
                thermal_noise_m: Meters(0.0),
                tracking_jitter_m: Meters(4.0),
                multipath_proxy_m: Meters(0.0),
                clock_error_m: Meters(0.0),
            }),
            metadata: ObsMetadata {
                signal: signal_spec_gps_l1_ca(),
                tracking_uncertainty: Some(TrackingUncertainty {
                    code_phase_samples: 0.01,
                    carrier_phase_cycles: 0.1,
                    doppler_hz: 2.0,
                    cn0_dbhz: 0.5,
                }),
                ..ObsMetadata::default()
            },
        }],
        decision: ObservationEpochDecision::Accepted,
        decision_reason: None,
        manifest: None,
    };

    let observations = position_observations_from_epoch(&epoch);

    assert_eq!(observations.len(), 1);
    assert!((observations[0].weight - 0.125).abs() < 1.0e-12);
}
