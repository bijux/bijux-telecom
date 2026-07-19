use super::*;

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
    let solver =
        PositionSolver::new().with_robust_weighting(PositionRobustWeighting::tukey_biweight(30.0));
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
    let solver =
        PositionSolver::new().with_robust_weighting(PositionRobustWeighting::tukey_biweight(30.0));
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
