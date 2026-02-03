use bijux_gnss_core::{
    Constellation, LockFlags, ObsMetadata, ObsSatellite, SatId, SigId, SignalBand, SignalSpec,
};
use bijux_gnss_nav::Matrix;
use bijux_gnss_receiver::rtk::float_from_state;
use bijux_gnss_receiver::rtk::AmbiguityManager;
use bijux_gnss_receiver::rtk::{
    decorrelate_lambda, ratio_from_candidates, ratio_test, search_integer_candidates,
    select_partial_fix, FixPolicy, FixState, FloatAmbiguitySolution,
};

#[test]
fn ratio_test_requires_consecutive_accepts() {
    let policy = FixPolicy {
        ratio_threshold: 3.0,
        consecutive_required: 2,
    };
    let mut state = FixState::default();
    assert!(!ratio_test(2.0, &policy, &mut state));
    assert!(!ratio_test(3.1, &policy, &mut state));
    assert!(ratio_test(3.2, &policy, &mut state));
}

#[test]
fn ambiguity_resets_on_cycle_slip() {
    let mut manager = AmbiguityManager::new();
    let sat = ObsSatellite {
        signal_id: SigId {
            sat: SatId {
                constellation: Constellation::Gps,
                prn: 1,
            },
            band: SignalBand::L1,
            code: bijux_gnss_core::SignalCode::Ca,
        },
        pseudorange_m: 20_000_000.0,
        pseudorange_var_m2: 1.0,
        carrier_phase_cycles: 100.0,
        carrier_phase_var_cycles2: 0.01,
        doppler_hz: 0.0,
        doppler_var_hz2: 4.0,
        cn0_dbhz: 45.0,
        lock_flags: LockFlags {
            code_lock: true,
            carrier_lock: true,
            bit_lock: false,
            cycle_slip: false,
        },
        multipath_suspect: false,
        elevation_deg: None,
        azimuth_deg: None,
        weight: None,
        error_model: None,
        metadata: ObsMetadata {
            tracking_mode: "scalar".to_string(),
            integration_ms: 1,
            lock_quality: 45.0,
            smoothing_window: 0,
            smoothing_age: 0,
            smoothing_resets: 0,
            signal: SignalSpec {
                constellation: Constellation::Gps,
                band: SignalBand::L1,
                code: bijux_gnss_core::SignalCode::Ca,
                code_rate_hz: 1_023_000.0,
                carrier_hz: bijux_gnss_core::GPS_L1_CA_CARRIER_HZ,
            },
        },
    };
    manager.update_from_obs(1, std::slice::from_ref(&sat));
    let mut slip_sat = sat.clone();
    slip_sat.lock_flags.cycle_slip = true;
    manager.update_from_obs(2, std::slice::from_ref(&slip_sat));
    let state = manager.states.values().next().expect("state");
    assert_eq!(state.status, bijux_gnss_core::AmbiguityStatus::Unknown);
}

#[test]
fn float_covariance_is_symmetric_psd() {
    let ids = vec![
        bijux_gnss_core::AmbiguityId {
            sig: SigId {
                sat: SatId {
                    constellation: Constellation::Gps,
                    prn: 1,
                },
                band: SignalBand::L1,
                code: bijux_gnss_core::SignalCode::Ca,
            },
            signal: "L1".to_string(),
        },
        bijux_gnss_core::AmbiguityId {
            sig: SigId {
                sat: SatId {
                    constellation: Constellation::Gps,
                    prn: 2,
                },
                band: SignalBand::L1,
                code: bijux_gnss_core::SignalCode::Ca,
            },
            signal: "L1".to_string(),
        },
    ];
    let state = vec![0.0, 1.0, 2.0];
    let mut cov = Matrix::identity(3);
    cov[(0, 1)] = 0.1;
    cov[(1, 0)] = 0.1;
    let float = float_from_state(ids, vec![0, 1], &state, &cov);

    assert_eq!(float.covariance.len(), 2);
    let eps = 1e-9;
    assert!((float.covariance[0][1] - float.covariance[1][0]).abs() < eps);

    let tests = vec![[1.0, 0.0], [0.0, 1.0], [1.0, -1.0], [2.0, 3.0]];
    for v in tests {
        let quad = v[0] * (float.covariance[0][0] * v[0] + float.covariance[0][1] * v[1])
            + v[1] * (float.covariance[1][0] * v[0] + float.covariance[1][1] * v[1]);
        assert!(quad >= -1e-6, "non-psd quadratic {quad}");
    }
}

#[test]
fn lambda_candidate_search_is_deterministic() {
    let float = FloatAmbiguitySolution {
        ids: Vec::new(),
        float_cycles: vec![10.2, -3.7],
        covariance: vec![vec![1.0, 0.0], vec![0.0, 1.0]],
    };
    let decor = decorrelate_lambda(&float);
    let cands = search_integer_candidates(&decor.n_prime, &decor.q_prime, 2);
    assert_eq!(cands.len(), 2);
    assert_eq!(cands[0].integers, vec![10, -4]);
    let ratio = ratio_from_candidates(&cands).unwrap();
    assert!(ratio > 1.0);
}

#[test]
fn partial_fix_reduces_ambiguity_count() {
    let float = FloatAmbiguitySolution {
        ids: vec![
            bijux_gnss_core::AmbiguityId {
                sig: SigId {
                    sat: SatId {
                        constellation: Constellation::Gps,
                        prn: 1,
                    },
                    band: SignalBand::L1,
                    code: bijux_gnss_core::SignalCode::Ca,
                },
                signal: "L1".to_string(),
            },
            bijux_gnss_core::AmbiguityId {
                sig: SigId {
                    sat: SatId {
                        constellation: Constellation::Gps,
                        prn: 2,
                    },
                    band: SignalBand::L1,
                    code: bijux_gnss_core::SignalCode::Ca,
                },
                signal: "L1".to_string(),
            },
            bijux_gnss_core::AmbiguityId {
                sig: SigId {
                    sat: SatId {
                        constellation: Constellation::Gps,
                        prn: 3,
                    },
                    band: SignalBand::L1,
                    code: bijux_gnss_core::SignalCode::Ca,
                },
                signal: "L1".to_string(),
            },
        ],
        float_cycles: vec![1.1, 2.2, 3.3],
        covariance: vec![
            vec![1.0, 0.0, 0.0],
            vec![0.0, 1.0, 0.0],
            vec![0.0, 0.0, 1.0],
        ],
    };
    let subset = select_partial_fix(&float, 2);
    assert_eq!(subset.float_cycles.len(), 2);
}
