use super::{
    constellation_residual_rms, corrected_observation_records,
    position_broadcast_navigation_from_beidou_navigations,
    position_broadcast_navigation_from_galileo_navigations,
    position_broadcast_navigation_from_glonass_frames,
    position_broadcast_navigation_from_gps_ephemerides, position_observations_from_epoch,
    resolve_position_inputs, robust_weight, robust_weights, solve_weighted_least_squares,
    unknown_inter_system_time_offset_sats, ClockStateModel, PositionBroadcastNavigation,
    PositionEstimate, PositionObservation, PositionRobustWeighting, SatelliteGeometry,
    SatelliteState, WorkingSetResidual, SPEED_OF_LIGHT_MPS,
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

#[path = "tests/least_squares.rs"]
mod least_squares;
#[path = "tests/navigation_inputs.rs"]
mod navigation_inputs;

#[test]
fn constellation_residual_rms_tracks_pre_fit_and_post_fit_groups() {
    let gps_sat = SatId { constellation: Constellation::Gps, prn: 3 };
    let galileo_sat = SatId { constellation: Constellation::Galileo, prn: 19 };
    let pre_fit = vec![
        WorkingSetResidual {
            sat: gps_sat,
            residual_m: 3.0,
            base_weight: 1.0,
            effective_weight: 1.0,
        },
        WorkingSetResidual {
            sat: gps_sat,
            residual_m: 4.0,
            base_weight: 1.0,
            effective_weight: 1.0,
        },
        WorkingSetResidual {
            sat: galileo_sat,
            residual_m: 12.0,
            base_weight: 1.0,
            effective_weight: 1.0,
        },
    ];
    let post_fit = vec![
        (
            PositionObservation {
                sat: gps_sat,
                pseudorange_m: 24_000_000.0,
                doppler_hz: None,
                doppler_var_hz2: None,
                cn0_dbhz: 45.0,
                elevation_deg: Some(45.0),
                weight: 1.0,
                gps_receive_time: None,
                signal_timing: None,
                signal_id: None,
            },
            SatelliteState {
                x_m: 0.0,
                y_m: 0.0,
                z_m: 0.0,
                vx_mps: 0.0,
                vy_mps: 0.0,
                vz_mps: 0.0,
                clock_bias_s: 0.0,
                clock_drift_s_per_s: 0.0,
                uncertainty: SatelliteStateUncertainty::unavailable(),
            },
            1.5,
            1.0,
        ),
        (
            PositionObservation {
                sat: galileo_sat,
                pseudorange_m: 24_100_000.0,
                doppler_hz: None,
                doppler_var_hz2: None,
                cn0_dbhz: 45.0,
                elevation_deg: Some(50.0),
                weight: 1.0,
                gps_receive_time: None,
                signal_timing: None,
                signal_id: None,
            },
            SatelliteState {
                x_m: 0.0,
                y_m: 0.0,
                z_m: 0.0,
                vx_mps: 0.0,
                vy_mps: 0.0,
                vz_mps: 0.0,
                clock_bias_s: 0.0,
                clock_drift_s_per_s: 0.0,
                uncertainty: SatelliteStateUncertainty::unavailable(),
            },
            2.0,
            1.0,
        ),
    ];

    let summaries = constellation_residual_rms(&pre_fit, &post_fit);

    assert_eq!(summaries.len(), 2);
    let gps = summaries
        .iter()
        .find(|summary| summary.constellation == Constellation::Gps)
        .expect("gps summary");
    let galileo = summaries
        .iter()
        .find(|summary| summary.constellation == Constellation::Galileo)
        .expect("galileo summary");
    assert_eq!(gps.pre_fit_sat_count, 2);
    assert_eq!(gps.post_fit_sat_count, 1);
    assert!((gps.pre_fit_rms_m.expect("gps pre-fit").0 - 3.535_533_905_9).abs() < 1.0e-9);
    assert!((gps.post_fit_rms_m.expect("gps post-fit").0 - 1.5).abs() < 1.0e-12);
    assert_eq!(galileo.pre_fit_sat_count, 1);
    assert_eq!(galileo.post_fit_sat_count, 1);
    assert!((galileo.pre_fit_rms_m.expect("galileo pre-fit").0 - 12.0).abs() < 1.0e-12);
    assert!((galileo.post_fit_rms_m.expect("galileo post-fit").0 - 2.0).abs() < 1.0e-12);
}

#[test]
fn disabled_robust_weighting_preserves_unit_weights() {
    let weights = robust_weights(&[0.0, 12.0, 80.0], PositionRobustWeighting::disabled());

    assert_eq!(weights, vec![1.0, 1.0, 1.0]);
}

#[test]
fn huber_robust_weighting_caps_large_residuals_linearly() {
    let inlier_weight = robust_weight(15.0, PositionRobustWeighting::huber(30.0));
    let outlier_weight = robust_weight(120.0, PositionRobustWeighting::huber(30.0));

    assert_eq!(inlier_weight, 1.0);
    assert!((outlier_weight - 0.25).abs() < 1.0e-12);
}

#[test]
fn tukey_robust_weighting_zeroes_residuals_beyond_cutoff() {
    let inlier_weight = robust_weight(15.0, PositionRobustWeighting::tukey_biweight(30.0));
    let boundary_weight = robust_weight(30.0, PositionRobustWeighting::tukey_biweight(30.0));
    let far_outlier_weight = robust_weight(120.0, PositionRobustWeighting::tukey_biweight(30.0));

    assert!(inlier_weight > 0.0 && inlier_weight < 1.0);
    assert_eq!(boundary_weight, 0.0);
    assert_eq!(far_outlier_weight, 0.0);
}

#[test]
fn first_iteration_measurement_weights_start_from_base_weights() {
    let solver = super::PositionSolver::new()
        .with_robust_weighting(PositionRobustWeighting::tukey_biweight(30.0));
    let geometry = vec![
        SatelliteGeometry {
            observation: PositionObservation {
                sat: SatId { constellation: Constellation::Gps, prn: 1 },
                pseudorange_m: 24_000_000.0,
                doppler_hz: None,
                doppler_var_hz2: None,
                cn0_dbhz: 45.0,
                elevation_deg: Some(45.0),
                weight: 0.5,
                gps_receive_time: None,
                signal_timing: None,
                signal_id: None,
            },
            corrected_pseudorange_m: 24_000_000.0,
            broadcast_group_delay_correction_chain: PositionObservationCorrectionChain::new(
                24_000_000.0,
            ),
            state: SatelliteState {
                x_m: 20_200_000.0,
                y_m: -1_500_000.0,
                z_m: 21_300_000.0,
                vx_mps: 0.0,
                vy_mps: 0.0,
                vz_mps: 0.0,
                clock_bias_s: 0.0,
                clock_drift_s_per_s: 0.0,
                uncertainty: SatelliteStateUncertainty::unavailable(),
            },
            iono_delay_m: 0.0,
            tropo_delay_m: 0.0,
        },
        SatelliteGeometry {
            observation: PositionObservation {
                sat: SatId { constellation: Constellation::Gps, prn: 2 },
                pseudorange_m: 24_100_000.0,
                doppler_hz: None,
                doppler_var_hz2: None,
                cn0_dbhz: 45.0,
                elevation_deg: Some(50.0),
                weight: 2.0,
                gps_receive_time: None,
                signal_timing: None,
                signal_id: None,
            },
            corrected_pseudorange_m: 24_100_000.0,
            broadcast_group_delay_correction_chain: PositionObservationCorrectionChain::new(
                24_100_000.0,
            ),
            state: SatelliteState {
                x_m: 20_300_000.0,
                y_m: -1_600_000.0,
                z_m: 21_200_000.0,
                vx_mps: 0.0,
                vy_mps: 0.0,
                vz_mps: 0.0,
                clock_bias_s: 0.0,
                clock_drift_s_per_s: 0.0,
                uncertainty: SatelliteStateUncertainty::unavailable(),
            },
            iono_delay_m: 0.0,
            tropo_delay_m: 0.0,
        },
    ];

    let weights = solver.measurement_weights(0, &geometry, &[1.0e6, 1.0e6]);

    assert_eq!(weights, vec![0.5, 2.0]);
}

#[test]
fn later_iteration_measurement_weights_fall_back_when_tukey_zeroes_everything() {
    let solver = super::PositionSolver::new()
        .with_robust_weighting(PositionRobustWeighting::tukey_biweight(30.0));
    let geometry = vec![
        SatelliteGeometry {
            observation: PositionObservation {
                sat: SatId { constellation: Constellation::Gps, prn: 1 },
                pseudorange_m: 24_000_000.0,
                doppler_hz: None,
                doppler_var_hz2: None,
                cn0_dbhz: 45.0,
                elevation_deg: Some(45.0),
                weight: 0.5,
                gps_receive_time: None,
                signal_timing: None,
                signal_id: None,
            },
            corrected_pseudorange_m: 24_000_000.0,
            broadcast_group_delay_correction_chain: PositionObservationCorrectionChain::new(
                24_000_000.0,
            ),
            state: SatelliteState {
                x_m: 20_200_000.0,
                y_m: -1_500_000.0,
                z_m: 21_300_000.0,
                vx_mps: 0.0,
                vy_mps: 0.0,
                vz_mps: 0.0,
                clock_bias_s: 0.0,
                clock_drift_s_per_s: 0.0,
                uncertainty: SatelliteStateUncertainty::unavailable(),
            },
            iono_delay_m: 0.0,
            tropo_delay_m: 0.0,
        },
        SatelliteGeometry {
            observation: PositionObservation {
                sat: SatId { constellation: Constellation::Gps, prn: 2 },
                pseudorange_m: 24_100_000.0,
                doppler_hz: None,
                doppler_var_hz2: None,
                cn0_dbhz: 45.0,
                elevation_deg: Some(50.0),
                weight: 2.0,
                gps_receive_time: None,
                signal_timing: None,
                signal_id: None,
            },
            corrected_pseudorange_m: 24_100_000.0,
            broadcast_group_delay_correction_chain: PositionObservationCorrectionChain::new(
                24_100_000.0,
            ),
            state: SatelliteState {
                x_m: 20_300_000.0,
                y_m: -1_600_000.0,
                z_m: 21_200_000.0,
                vx_mps: 0.0,
                vy_mps: 0.0,
                vz_mps: 0.0,
                clock_bias_s: 0.0,
                clock_drift_s_per_s: 0.0,
                uncertainty: SatelliteStateUncertainty::unavailable(),
            },
            iono_delay_m: 0.0,
            tropo_delay_m: 0.0,
        },
    ];

    let weights = solver.measurement_weights(1, &geometry, &[1.0e6, 1.0e6]);

    assert_eq!(weights, vec![0.5, 2.0]);
}

#[test]
fn corrected_observation_records_reconstruct_solver_residual() {
    let sat = SatId { constellation: Constellation::Gps, prn: 3 };
    let mut group_delay_chain = PositionObservationCorrectionChain::new(20_000_010.0);
    group_delay_chain.push_component(PositionObservationCorrectionKind::BroadcastGroupDelay, -3.0);
    let geometry = vec![SatelliteGeometry {
        observation: PositionObservation {
            sat,
            pseudorange_m: 20_000_010.0,
            doppler_hz: None,
            doppler_var_hz2: None,
            cn0_dbhz: 45.0,
            elevation_deg: Some(45.0),
            weight: 1.0,
            gps_receive_time: None,
            signal_timing: None,
            signal_id: None,
        },
        corrected_pseudorange_m: 20_000_007.0,
        broadcast_group_delay_correction_chain: group_delay_chain,
        state: SatelliteState {
            x_m: 20_000_000.0,
            y_m: 0.0,
            z_m: 0.0,
            vx_mps: 0.0,
            vy_mps: 0.0,
            vz_mps: 0.0,
            clock_bias_s: 2.0e-6,
            clock_drift_s_per_s: 0.0,
            uncertainty: SatelliteStateUncertainty::unavailable(),
        },
        iono_delay_m: 5.0,
        tropo_delay_m: 2.0,
    }];
    let estimate = PositionEstimate {
        ecef_x_m: 0.0,
        ecef_y_m: 0.0,
        ecef_z_m: 0.0,
        clock_model: ClockStateModel::from_constellations([Constellation::Gps])
            .expect("clock model"),
        clock_state_s: vec![1.0e-6],
    };
    let expected_residual_m = 20_000_010.0 + 2.0e-6 * SPEED_OF_LIGHT_MPS
        - 3.0
        - 5.0
        - 2.0
        - 1.0e-6 * SPEED_OF_LIGHT_MPS
        - 20_000_000.0;
    let residuals = vec![WorkingSetResidual {
        sat,
        residual_m: expected_residual_m,
        base_weight: 1.0,
        effective_weight: 1.0,
    }];

    let records = corrected_observation_records(&estimate, &geometry, &residuals)
        .expect("corrected observation records");
    let record = records.first().expect("recorded observation");

    assert_eq!(record.correction_chain.components.len(), 11);
    assert_eq!(record.reconstruction_error_m(), 0.0);
    assert!((record.reconstructed_residual_m() - expected_residual_m).abs() < 1.0e-9);
    assert_eq!(record.residual_m, expected_residual_m);
    assert!(
        !record
            .correction_chain
            .components
            .iter()
            .find(|component| component.kind == PositionObservationCorrectionKind::Relativity)
            .expect("relativity component")
            .applied
    );
    assert!(
        record
            .correction_chain
            .components
            .iter()
            .find(|component| {
                component.kind == PositionObservationCorrectionKind::BroadcastGroupDelay
            })
            .expect("group delay component")
            .applied
    );
}

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
