#![allow(missing_docs)]

use bijux_gnss_core::api::{Constellation, SatId, SigId, SignalBand};
use bijux_gnss_nav::api::{
    rtk_ambiguity_state_from_fixed_solution, rtk_conditioned_baseline_from_fixed_ambiguities,
    rtk_float_ambiguity_state_from_baseline_solution, rtk_float_baseline_from_double_differences,
    rtk_transform_float_ambiguity_reference, RtkAmbiguityFixPolicy, RtkAmbiguityFixState,
    RtkAmbiguityFixStatus, RtkDoubleDifferenceAmbiguityId, RtkFloatAmbiguityEstimate,
    RtkFloatAmbiguityState, RtkFloatBaselineSolution, RtkRatioTestFixer,
};
use bijux_gnss_testkit::rtk_baseline::clean_gps_l1_short_baseline_case;

fn gps_l1_id(prn: u8) -> RtkDoubleDifferenceAmbiguityId {
    let sig = gps_l1_sig(prn);
    RtkDoubleDifferenceAmbiguityId { sig, ref_sig: sig }
}

fn gps_l1_sig(prn: u8) -> SigId {
    SigId {
        sat: SatId { constellation: Constellation::Gps, prn },
        band: SignalBand::L1,
        code: bijux_gnss_core::api::SignalCode::Ca,
    }
}

fn gps_l1_dd_id(sig_prn: u8, ref_prn: u8) -> RtkDoubleDifferenceAmbiguityId {
    let sig = SigId {
        sat: SatId { constellation: Constellation::Gps, prn: sig_prn },
        band: SignalBand::L1,
        code: bijux_gnss_core::api::SignalCode::Ca,
    };
    let ref_sig = SigId {
        sat: SatId { constellation: Constellation::Gps, prn: ref_prn },
        band: SignalBand::L1,
        code: bijux_gnss_core::api::SignalCode::Ca,
    };
    RtkDoubleDifferenceAmbiguityId { sig, ref_sig }
}

#[test]
fn rtk_ratio_test_fixer_reports_failed_status_for_clean_short_baseline_solution() {
    let scenario = clean_gps_l1_short_baseline_case();
    let float_solution = rtk_float_baseline_from_double_differences(
        &scenario.double_differences,
        scenario.base_ecef_m,
        &scenario.ephemerides,
        scenario.receive_gps_time.tow_s,
    )
    .expect("float baseline");
    let float_state =
        rtk_float_ambiguity_state_from_baseline_solution(&float_solution).expect("float state");
    let fixer = RtkRatioTestFixer::new(RtkAmbiguityFixPolicy {
        ratio_threshold: 3.0,
        consecutive_required: 1,
    });

    let (result, audit) =
        fixer.fix_with_state(0, &float_state, &mut RtkAmbiguityFixState::default());

    assert_eq!(result.status, RtkAmbiguityFixStatus::Failed, "result={result:?}");
    assert_eq!(audit.status, RtkAmbiguityFixStatus::Failed, "audit={audit:?}");
    assert_eq!(audit.reason, "ratio_fail");
    assert!(result.ratio.expect("ratio") < 3.0);
    assert!(result.selected_ids.is_none());
    assert!(result.selected_integers.is_none());
}

#[test]
fn rtk_ratio_test_fixer_reports_failed_status_for_indistinguishable_candidates() {
    let float_state = RtkFloatAmbiguityState {
        ids: vec![gps_l1_id(7), gps_l1_id(11)],
        float_cycles: vec![10.5, -3.5],
        covariance_cycles2: vec![vec![1.0, 0.0], vec![0.0, 1.0]],
    };
    let fixer = RtkRatioTestFixer::new(RtkAmbiguityFixPolicy {
        ratio_threshold: 3.0,
        consecutive_required: 1,
    });

    let (result, audit) =
        fixer.fix_with_state(42, &float_state, &mut RtkAmbiguityFixState::default());

    assert_eq!(result.status, RtkAmbiguityFixStatus::Failed);
    assert_eq!(audit.status, RtkAmbiguityFixStatus::Failed);
    assert_eq!(audit.reason, "ratio_fail");
    assert!(result.selected_ids.is_none());
    assert!(result.selected_integers.is_none());
}

#[test]
fn rtk_float_ambiguity_reference_transform_preserves_satellite_arcs() {
    let float_state = RtkFloatAmbiguityState {
        ids: vec![gps_l1_dd_id(7, 3), gps_l1_dd_id(11, 3), gps_l1_dd_id(14, 3)],
        float_cycles: vec![20.0, 35.0, 50.0],
        covariance_cycles2: vec![vec![4.0, 1.0, 0.5], vec![1.0, 9.0, 2.0], vec![0.5, 2.0, 16.0]],
    };

    let transformed =
        rtk_transform_float_ambiguity_reference(&float_state, gps_l1_sig(11)).expect("transform");

    assert_eq!(
        transformed.ids,
        vec![gps_l1_dd_id(3, 11), gps_l1_dd_id(7, 11), gps_l1_dd_id(14, 11)]
    );
    assert_eq!(transformed.float_cycles, vec![-35.0, -15.0, 15.0]);
    assert_eq!(
        transformed.covariance_cycles2,
        vec![vec![9.0, 8.0, 7.0], vec![8.0, 11.0, 6.5], vec![7.0, 6.5, 21.0]]
    );
}

#[test]
fn rtk_float_ambiguity_reference_transform_refuses_missing_reference() {
    let float_state = RtkFloatAmbiguityState {
        ids: vec![gps_l1_dd_id(7, 3), gps_l1_dd_id(11, 3)],
        float_cycles: vec![20.0, 35.0],
        covariance_cycles2: vec![vec![4.0, 1.0], vec![1.0, 9.0]],
    };

    assert!(rtk_transform_float_ambiguity_reference(&float_state, gps_l1_sig(14)).is_none());
}

#[test]
fn rtk_ratio_test_fixer_conditions_baseline_for_high_confidence_integer_fix() {
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

    let (result, audit) =
        fixer.fix_with_state(8, &float_state, &mut RtkAmbiguityFixState::default());
    let (fixed_ids, fixed_integers) =
        rtk_ambiguity_state_from_fixed_solution(&result).expect("accepted integer fix");
    let conditioned = rtk_conditioned_baseline_from_fixed_ambiguities(
        &float_solution,
        &fixed_ids,
        &fixed_integers,
    )
    .expect("conditioned baseline");

    assert_eq!(result.status, RtkAmbiguityFixStatus::Fixed, "result={result:?}");
    assert_eq!(audit.status, RtkAmbiguityFixStatus::Fixed, "audit={audit:?}");
    assert_eq!(fixed_ids.len(), 2);
    assert_eq!(fixed_integers, vec![10, -3]);
    assert!((conditioned.enu_m[0] - 0.98).abs() < 1.0e-9);
    assert!((conditioned.enu_m[1] - 2.004).abs() < 1.0e-9);
    assert!((conditioned.enu_m[2] - 2.994).abs() < 1.0e-9);
    assert!(conditioned.covariance_enu_m2[0][0] < float_solution.covariance_enu_m2[0][0]);
    assert!(conditioned.covariance_enu_m2[1][1] < float_solution.covariance_enu_m2[1][1]);
    assert!(conditioned.covariance_enu_m2[2][2] < float_solution.covariance_enu_m2[2][2]);
}
