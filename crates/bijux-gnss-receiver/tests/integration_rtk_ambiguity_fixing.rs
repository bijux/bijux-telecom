#![allow(missing_docs)]

use bijux_gnss_core::api::{
    Constellation, LockFlags, ObsMetadata, ObsSatellite, SatId, SigId, SignalBand, SignalSpec,
};
use bijux_gnss_receiver::api::{
    build_dd, build_sd, choose_ref_sat, rtk_ambiguity_state_from_fixed_solution,
    rtk_conditioned_baseline_from_fixed_ambiguities,
    rtk_float_ambiguity_state_from_baseline_solution, solve_float_baseline_dd,
    RtkAmbiguityFixPolicy, RtkAmbiguityFixState, RtkAmbiguityTracker,
    RtkDoubleDifferenceAmbiguityId, RtkFloatAmbiguityEstimate, RtkFloatBaselineSolution,
    RtkRatioTestFixer,
};
use bijux_gnss_testkit::rtk_baseline::clean_gps_l1_short_baseline_case;

#[test]
fn receiver_reports_failed_status_for_real_rtk_float_solution() {
    let scenario = clean_gps_l1_short_baseline_case();
    let single_differences = build_sd(&scenario.base_epoch, &scenario.rover_epoch);
    let reference = choose_ref_sat(&single_differences).expect("reference");
    let double_differences = build_dd(&single_differences, reference);
    let float_solution = solve_float_baseline_dd(
        &double_differences,
        scenario.base_ecef_m,
        &scenario.ephemerides,
        scenario.receive_gps_time.tow_s,
    )
    .expect("float solution");
    let float_state =
        rtk_float_ambiguity_state_from_baseline_solution(&float_solution).expect("float state");
    let fixer = RtkRatioTestFixer::new(RtkAmbiguityFixPolicy {
        ratio_threshold: 3.0,
        consecutive_required: 1,
    });

    let (result, audit) =
        fixer.fix_with_state(0, &float_state, &mut RtkAmbiguityFixState::default());

    assert_eq!(float_state.covariance_cycles2.len(), double_differences.len());
    assert_eq!(audit.reason, "ratio_fail");
    assert!(result.ratio.expect("ratio") < 3.0);
    assert!(rtk_ambiguity_state_from_fixed_solution(&result).is_none());
}

#[test]
fn receiver_rtk_ambiguity_tracker_resets_on_cycle_slip() {
    let mut tracker = RtkAmbiguityTracker::new();
    let sat = ObsSatellite {
        signal_id: SigId {
            sat: SatId { constellation: Constellation::Gps, prn: 1 },
            band: SignalBand::L1,
            code: bijux_gnss_core::api::SignalCode::Ca,
        },
        pseudorange_m: bijux_gnss_core::api::Meters(20_000_000.0),
        pseudorange_var_m2: 1.0,
        carrier_phase_cycles: bijux_gnss_core::api::Cycles(100.0),
        carrier_phase_var_cycles2: 0.01,
        doppler_hz: bijux_gnss_core::api::Hertz(0.0),
        doppler_var_hz2: 4.0,
        cn0_dbhz: 45.0,
        lock_flags: LockFlags {
            code_lock: true,
            carrier_lock: true,
            bit_lock: false,
            cycle_slip: false,
        },
        multipath_suspect: false,
        observation_status: bijux_gnss_core::api::ObservationStatus::Accepted,
        observation_reject_reasons: Vec::new(),
        elevation_deg: None,
        azimuth_deg: None,
        weight: None,
        timing: None,
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
                code: bijux_gnss_core::api::SignalCode::Ca,
                code_rate_hz: 1_023_000.0,
                carrier_hz: bijux_gnss_core::api::GPS_L1_CA_CARRIER_HZ,
            },
            ..ObsMetadata::default()
        },
    };
    tracker.update_from_obs(1, std::slice::from_ref(&sat));
    let mut slip_sat = sat.clone();
    slip_sat.lock_flags.cycle_slip = true;
    tracker.update_from_obs(2, std::slice::from_ref(&slip_sat));

    let state = tracker.states.values().next().expect("state");
    assert_eq!(state.status, bijux_gnss_core::api::AmbiguityStatus::Unknown);
}

fn gps_l1_id(prn: u8) -> RtkDoubleDifferenceAmbiguityId {
    let sig = SigId {
        sat: SatId { constellation: Constellation::Gps, prn },
        band: SignalBand::L1,
        code: bijux_gnss_core::api::SignalCode::Ca,
    };
    RtkDoubleDifferenceAmbiguityId { sig, ref_sig: sig }
}

#[test]
fn receiver_conditions_baseline_for_high_confidence_integer_fix() {
    let float_solution = RtkFloatBaselineSolution {
        enu_m: [1.0, 2.0, 3.0],
        covariance_enu_m2: [[1.0, 0.0, 0.0], [0.0, 1.2, 0.0], [0.0, 0.0, 1.5]],
        enu_ambiguity_covariance_m_cycles: vec![
            vec![0.004, 0.0],
            vec![0.0, 0.008],
            vec![0.001, -0.002],
        ],
        float_ambiguities: vec![
            RtkFloatAmbiguityEstimate {
                sig: gps_l1_id(7).sig,
                ref_sig: gps_l1_id(7).ref_sig,
                float_cycles: 10.05,
                variance_cycles2: 0.01,
            },
            RtkFloatAmbiguityEstimate {
                sig: gps_l1_id(11).sig,
                ref_sig: gps_l1_id(11).ref_sig,
                float_cycles: -3.02,
                variance_cycles2: 0.04,
            },
        ],
        ambiguity_covariance_cycles2: vec![vec![0.01, 0.0], vec![0.0, 0.04]],
    };
    let float_state =
        rtk_float_ambiguity_state_from_baseline_solution(&float_solution).expect("float state");
    let fixer = RtkRatioTestFixer::new(RtkAmbiguityFixPolicy {
        ratio_threshold: 3.0,
        consecutive_required: 1,
    });

    let (result, _audit) =
        fixer.fix_with_state(8, &float_state, &mut RtkAmbiguityFixState::default());
    let (fixed_ids, fixed_integers) =
        rtk_ambiguity_state_from_fixed_solution(&result).expect("accepted integer fix");
    let conditioned = rtk_conditioned_baseline_from_fixed_ambiguities(
        &float_solution,
        &fixed_ids,
        &fixed_integers,
    )
    .expect("conditioned baseline");

    assert_eq!(fixed_integers, vec![10, -3]);
    assert!((conditioned.enu_m[0] - 0.98).abs() < 1.0e-9);
    assert!((conditioned.enu_m[1] - 2.004).abs() < 1.0e-9);
    assert!((conditioned.enu_m[2] - 2.994).abs() < 1.0e-9);
}
