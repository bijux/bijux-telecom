use super::*;

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
