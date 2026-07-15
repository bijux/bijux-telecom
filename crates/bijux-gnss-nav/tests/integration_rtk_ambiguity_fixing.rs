#![allow(missing_docs)]

use bijux_gnss_core::api::ArtifactPayloadValidate;
use bijux_gnss_core::api::{Constellation, SatId, SigId, SignalBand};
use bijux_gnss_nav::api::{
    rtk_ambiguity_state_from_fixed_solution, rtk_apply_ambiguity_fix_lifecycle,
    rtk_conditioned_baseline_from_fix_result, rtk_conditioned_baseline_from_fixed_ambiguities,
    rtk_fixed_ambiguity_hold_from_fix_result, rtk_float_ambiguity_state_from_baseline_solution,
    rtk_float_baseline_from_double_differences, rtk_integer_ambiguity_candidates,
    rtk_lambda_decorrelate, rtk_lambda_integer_ambiguity_candidates,
    rtk_monitor_fixed_ambiguity_hold, rtk_select_partial_ambiguity_fix_with_evidence,
    rtk_transform_fixed_ambiguity_reference, rtk_transform_float_ambiguity_reference,
    rtk_transform_float_baseline_reference, RtkAmbiguityFixPolicy, RtkAmbiguityFixResult,
    RtkAmbiguityFixState, RtkAmbiguityFixStatus, RtkAmbiguityHoldAction, RtkAmbiguityHoldPolicy,
    RtkAmbiguityHoldState, RtkDoubleDifferenceAmbiguityId, RtkFixedAmbiguityHold,
    RtkFloatAmbiguityEstimate, RtkFloatAmbiguityState, RtkFloatBaselineSolution,
    RtkIntegerAmbiguityCandidate, RtkPartialAmbiguitySelectionCriterion, RtkRatioTestFixer,
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

fn assert_matrix_near(actual: &[Vec<f64>], expected: &[Vec<f64>], tolerance: f64) {
    assert_eq!(actual.len(), expected.len());
    for (actual_row, expected_row) in actual.iter().zip(expected.iter()) {
        assert_eq!(actual_row.len(), expected_row.len());
        for (actual_value, expected_value) in actual_row.iter().zip(expected_row.iter()) {
            assert!(
                (actual_value - expected_value).abs() <= tolerance,
                "actual={actual_value} expected={expected_value}"
            );
        }
    }
}

fn integer_identity(size: usize) -> Vec<Vec<i64>> {
    (0..size).map(|row| (0..size).map(|col| if row == col { 1 } else { 0 }).collect()).collect()
}

fn integer_matrix_mul(left: &[Vec<i64>], right: &[Vec<i64>]) -> Vec<Vec<i64>> {
    let mut out = vec![vec![0_i64; right[0].len()]; left.len()];
    for row in 0..left.len() {
        for col in 0..right[0].len() {
            out[row][col] =
                (0..right.len()).map(|index| left[row][index] * right[index][col]).sum();
        }
    }
    out
}

fn transformed_vector(transform: &[Vec<i64>], values: &[f64]) -> Vec<f64> {
    transform
        .iter()
        .map(|row| {
            row.iter()
                .zip(values.iter())
                .map(|(coefficient, value)| *coefficient as f64 * value)
                .sum()
        })
        .collect()
}

fn transformed_covariance(transform: &[Vec<i64>], covariance: &[Vec<f64>]) -> Vec<Vec<f64>> {
    let size = transform.len();
    let mut out = vec![vec![0.0; size]; size];
    for row in 0..size {
        for col in 0..size {
            let mut sum = 0.0;
            for left in 0..size {
                for right in 0..size {
                    sum += transform[row][left] as f64
                        * covariance[left][right]
                        * transform[col][right] as f64;
                }
            }
            out[row][col] = sum;
        }
    }
    out
}

fn correlation_abs(covariance: &[Vec<f64>], left: usize, right: usize) -> f64 {
    covariance[left][right].abs() / (covariance[left][left] * covariance[right][right]).sqrt()
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
fn rtk_integer_ambiguity_candidates_use_full_covariance_squared_norm() {
    let candidates =
        rtk_integer_ambiguity_candidates(&[-0.5, -0.45], &[vec![1.0, 0.8], vec![0.8, 1.0]], 4);

    assert_eq!(
        candidates.iter().map(|candidate| candidate.integers.clone()).collect::<Vec<_>>(),
        vec![vec![0, 0], vec![-1, -1], vec![-1, 0], vec![1, 1]]
    );
    assert!(candidates.windows(2).all(|pair| pair[0].cost <= pair[1].cost));
    assert!((candidates[0].cost - 0.256_944_444_444_444_4).abs() < 1.0e-12);
    assert!((candidates[1].cost - 0.312_5).abs() < 1.0e-12);
}

#[test]
fn rtk_lambda_decorrelation_carries_unimodular_transform_evidence() {
    let float_state = RtkFloatAmbiguityState {
        ids: vec![gps_l1_dd_id(7, 3), gps_l1_dd_id(11, 3)],
        float_cycles: vec![-0.5, -0.45],
        covariance_cycles2: vec![vec![1.0, 0.8], vec![0.8, 1.0]],
    };

    let decorrelated = rtk_lambda_decorrelate(&float_state);

    assert_ne!(decorrelated.z_inverse, integer_identity(2));
    assert_eq!(integer_matrix_mul(&decorrelated.z_inverse, &decorrelated.z), integer_identity(2));
    assert_eq!(
        decorrelated.n_prime,
        transformed_vector(&decorrelated.z_inverse, &float_state.float_cycles)
    );
    assert_matrix_near(
        &decorrelated.q_prime,
        &transformed_covariance(&decorrelated.z_inverse, &float_state.covariance_cycles2),
        1.0e-12,
    );
    assert!(
        correlation_abs(&decorrelated.q_prime, 0, 1)
            < correlation_abs(&float_state.covariance_cycles2, 0, 1)
    );
}

#[test]
fn rtk_lambda_integer_candidates_back_transform_to_original_ambiguities() {
    let float_state = RtkFloatAmbiguityState {
        ids: vec![gps_l1_dd_id(7, 3), gps_l1_dd_id(11, 3)],
        float_cycles: vec![-0.5, -0.45],
        covariance_cycles2: vec![vec![1.0, 0.8], vec![0.8, 1.0]],
    };

    let candidates = rtk_lambda_integer_ambiguity_candidates(&float_state, 3);

    assert_eq!(
        candidates.iter().map(|candidate| candidate.integers.clone()).collect::<Vec<_>>(),
        vec![vec![0, 0], vec![-1, -1], vec![-1, 0]]
    );
    assert!(candidates.windows(2).all(|pair| pair[0].cost <= pair[1].cost));
}

#[test]
fn rtk_lambda_integer_candidates_match_three_dimensional_benchmark() {
    let float_state = RtkFloatAmbiguityState {
        ids: vec![gps_l1_dd_id(7, 3), gps_l1_dd_id(11, 3), gps_l1_dd_id(14, 3)],
        float_cycles: vec![5.45, -2.35, 3.62],
        covariance_cycles2: vec![vec![4.0, 1.2, -0.6], vec![1.2, 2.25, 0.8], vec![-0.6, 0.8, 1.44]],
    };

    let candidates = rtk_lambda_integer_ambiguity_candidates(&float_state, 5);

    assert_eq!(
        candidates.iter().map(|candidate| candidate.integers.clone()).collect::<Vec<_>>(),
        vec![vec![5, -2, 4], vec![6, -2, 4], vec![5, -3, 3], vec![6, -3, 3], vec![6, -2, 3],]
    );
    assert!(candidates.windows(2).all(|pair| pair[0].cost <= pair[1].cost));
    assert!((candidates[0].cost - 0.179_505_373_640_877_46).abs() < 1.0e-12);
    assert!((candidates[1].cost - 0.253_353_654_704_292_44).abs() < 1.0e-12);
}

#[test]
fn rtk_partial_ambiguity_selection_reports_lowest_variance_criterion() {
    let float_state = RtkFloatAmbiguityState {
        ids: vec![gps_l1_dd_id(7, 3), gps_l1_dd_id(11, 3), gps_l1_dd_id(14, 3)],
        float_cycles: vec![5.45, -2.35, 3.62],
        covariance_cycles2: vec![vec![4.0, 1.2, -0.6], vec![1.2, 2.25, 0.8], vec![-0.6, 0.8, 1.44]],
    };

    let (partial, selection) =
        rtk_select_partial_ambiguity_fix_with_evidence(&float_state, 2).expect("partial selection");

    assert_eq!(selection.criterion, RtkPartialAmbiguitySelectionCriterion::LowestVariance);
    assert_eq!(selection.requested_count, 2);
    assert_eq!(selection.selected_indices, vec![2, 1]);
    assert_eq!(selection.excluded_indices, vec![0]);
    assert_eq!(selection.selected_ids, vec![gps_l1_dd_id(14, 3), gps_l1_dd_id(11, 3)]);
    assert_eq!(selection.excluded_ids, vec![gps_l1_dd_id(7, 3)]);
    assert_eq!(selection.selected_variance_cycles2, vec![1.44, 2.25]);
    assert_eq!(selection.excluded_variance_cycles2, vec![4.0]);
    assert_eq!(selection.max_selected_variance_cycles2, Some(2.25));
    assert_eq!(selection.min_excluded_variance_cycles2, Some(4.0));
    assert_eq!(partial.ids, selection.selected_ids);
    assert_eq!(partial.float_cycles, vec![3.62, -2.35]);
    assert_eq!(partial.covariance_cycles2, vec![vec![1.44, 0.8], vec![0.8, 2.25]]);
}

#[test]
fn rtk_ratio_test_fixer_reports_partial_fix_without_constraining_excluded_ambiguities() {
    let float_state = RtkFloatAmbiguityState {
        ids: vec![gps_l1_dd_id(7, 3), gps_l1_dd_id(11, 3)],
        float_cycles: vec![0.01, 0.01],
        covariance_cycles2: vec![vec![0.01, 0.0], vec![0.0, 100.0]],
    };
    let fixer = RtkRatioTestFixer::new(RtkAmbiguityFixPolicy {
        ratio_threshold: 3.0,
        consecutive_required: 1,
    });

    let (result, audit) =
        fixer.fix_with_state(77, &float_state, &mut RtkAmbiguityFixState::default());

    let selection = result.partial_selection.as_ref().expect("partial selection");
    assert_eq!(result.status, RtkAmbiguityFixStatus::Fixed);
    assert_eq!(result.fixed_count, 1);
    assert_eq!(result.selected_ids, Some(vec![gps_l1_dd_id(7, 3)]));
    assert_eq!(result.selected_integers, Some(vec![0]));
    assert_eq!(selection.selected_ids, vec![gps_l1_dd_id(7, 3)]);
    assert_eq!(selection.excluded_ids, vec![gps_l1_dd_id(11, 3)]);
    assert_eq!(selection.criterion, RtkPartialAmbiguitySelectionCriterion::LowestVariance);
    assert_eq!(audit.reason, "partial_fix");
    assert_eq!(audit.partial_selection, result.partial_selection);
}

#[test]
fn rtk_fixed_solution_extraction_rejects_inconsistent_partial_selection() {
    let float_state = RtkFloatAmbiguityState {
        ids: vec![gps_l1_dd_id(7, 3), gps_l1_dd_id(11, 3)],
        float_cycles: vec![0.01, 0.01],
        covariance_cycles2: vec![vec![0.01, 0.0], vec![0.0, 100.0]],
    };
    let (_partial, selection) =
        rtk_select_partial_ambiguity_fix_with_evidence(&float_state, 1).expect("partial selection");

    let mismatched = RtkAmbiguityFixResult {
        candidates: vec![RtkIntegerAmbiguityCandidate { integers: vec![0], cost: 0.01 }],
        ratio: Some(100.0),
        status: RtkAmbiguityFixStatus::Fixed,
        fixed_count: 1,
        selected_ids: Some(vec![gps_l1_dd_id(11, 3)]),
        selected_integers: Some(vec![0]),
        partial_selection: Some(selection.clone()),
    };
    let wrong_length = RtkAmbiguityFixResult {
        selected_ids: Some(selection.selected_ids.clone()),
        selected_integers: Some(vec![0, 1]),
        partial_selection: Some(selection),
        ..mismatched.clone()
    };

    assert!(rtk_ambiguity_state_from_fixed_solution(&mismatched).is_none());
    assert!(rtk_ambiguity_state_from_fixed_solution(&wrong_length).is_none());
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
fn rtk_fixed_ambiguity_reference_transform_preserves_integer_arcs() {
    let fixed_ids = vec![gps_l1_dd_id(7, 3), gps_l1_dd_id(11, 3), gps_l1_dd_id(14, 3)];
    let fixed_integers = vec![20, 35, 50];

    let (ids, integers) =
        rtk_transform_fixed_ambiguity_reference(&fixed_ids, &fixed_integers, gps_l1_sig(11))
            .expect("fixed transform");

    assert_eq!(ids, vec![gps_l1_dd_id(3, 11), gps_l1_dd_id(7, 11), gps_l1_dd_id(14, 11)]);
    assert_eq!(integers, vec![-35, -15, 15]);
}

#[test]
fn rtk_fixed_ambiguity_reference_transform_rejects_incomplete_state() {
    let fixed_ids = vec![gps_l1_dd_id(7, 3), gps_l1_dd_id(11, 3)];

    assert!(rtk_transform_fixed_ambiguity_reference(&fixed_ids, &[20], gps_l1_sig(11)).is_none());
    assert!(
        rtk_transform_fixed_ambiguity_reference(&fixed_ids, &[20, 35], gps_l1_sig(14)).is_none()
    );
}

#[test]
fn rtk_float_baseline_reference_transform_preserves_cross_covariance() {
    let float_solution = RtkFloatBaselineSolution {
        enu_m: [4.0, -2.0, 1.0],
        covariance_enu_m2: [[1.0, 0.1, 0.2], [0.1, 1.5, 0.3], [0.2, 0.3, 2.0]],
        enu_ambiguity_covariance_m_cycles: vec![
            vec![0.1, 0.2, 0.3],
            vec![0.4, 0.5, 0.6],
            vec![0.7, 0.8, 0.9],
        ],
        float_ambiguities: vec![
            RtkFloatAmbiguityEstimate {
                sig: gps_l1_sig(7),
                ref_sig: gps_l1_sig(3),
                float_cycles: 20.0,
                variance_cycles2: 4.0,
            },
            RtkFloatAmbiguityEstimate {
                sig: gps_l1_sig(11),
                ref_sig: gps_l1_sig(3),
                float_cycles: 35.0,
                variance_cycles2: 9.0,
            },
            RtkFloatAmbiguityEstimate {
                sig: gps_l1_sig(14),
                ref_sig: gps_l1_sig(3),
                float_cycles: 50.0,
                variance_cycles2: 16.0,
            },
        ],
        ambiguity_covariance_cycles2: vec![
            vec![4.0, 1.0, 0.5],
            vec![1.0, 9.0, 2.0],
            vec![0.5, 2.0, 16.0],
        ],
    };

    let transformed =
        rtk_transform_float_baseline_reference(&float_solution, gps_l1_sig(11)).expect("transform");

    assert_eq!(transformed.enu_m, float_solution.enu_m);
    assert_eq!(transformed.covariance_enu_m2, float_solution.covariance_enu_m2);
    assert_eq!(
        transformed
            .float_ambiguities
            .iter()
            .map(|ambiguity| RtkDoubleDifferenceAmbiguityId {
                sig: ambiguity.sig,
                ref_sig: ambiguity.ref_sig,
            })
            .collect::<Vec<_>>(),
        vec![gps_l1_dd_id(3, 11), gps_l1_dd_id(7, 11), gps_l1_dd_id(14, 11)]
    );
    assert_eq!(
        transformed
            .float_ambiguities
            .iter()
            .map(|ambiguity| ambiguity.float_cycles)
            .collect::<Vec<_>>(),
        vec![-35.0, -15.0, 15.0]
    );
    assert_matrix_near(
        &transformed.enu_ambiguity_covariance_m_cycles,
        &[vec![-0.2, -0.1, 0.1], vec![-0.5, -0.1, 0.1], vec![-0.8, -0.1, 0.1]],
        1.0e-12,
    );
    assert_eq!(
        transformed.ambiguity_covariance_cycles2,
        vec![vec![9.0, 8.0, 7.0], vec![8.0, 11.0, 6.5], vec![7.0, 6.5, 21.0]]
    );
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

#[test]
fn rtk_fix_result_conditioning_uses_only_selected_partial_ambiguities() {
    let selected_id = gps_l1_dd_id(7, 3);
    let excluded_id = gps_l1_dd_id(11, 3);
    let float_solution = RtkFloatBaselineSolution {
        enu_m: [1.0, 2.0, 3.0],
        covariance_enu_m2: [[1.0, 0.0, 0.0], [0.0, 1.2, 0.0], [0.0, 0.0, 1.5]],
        enu_ambiguity_covariance_m_cycles: vec![vec![0.004, 5.0], vec![0.0, 7.0], vec![0.0, 11.0]],
        float_ambiguities: vec![
            RtkFloatAmbiguityEstimate {
                sig: selected_id.sig,
                ref_sig: selected_id.ref_sig,
                float_cycles: 0.01,
                variance_cycles2: 0.01,
            },
            RtkFloatAmbiguityEstimate {
                sig: excluded_id.sig,
                ref_sig: excluded_id.ref_sig,
                float_cycles: 0.01,
                variance_cycles2: 100.0,
            },
        ],
        ambiguity_covariance_cycles2: vec![vec![0.01, 0.0], vec![0.0, 100.0]],
    };
    let float_state =
        rtk_float_ambiguity_state_from_baseline_solution(&float_solution).expect("float state");
    let fixer = RtkRatioTestFixer::new(RtkAmbiguityFixPolicy {
        ratio_threshold: 3.0,
        consecutive_required: 1,
    });

    let (result, audit) =
        fixer.fix_with_state(9, &float_state, &mut RtkAmbiguityFixState::default());
    let conditioned =
        rtk_conditioned_baseline_from_fix_result(&float_solution, &result).expect("partial fix");
    let selection = result.partial_selection.as_ref().expect("partial selection");

    assert_eq!(result.status, RtkAmbiguityFixStatus::Fixed, "result={result:?}");
    assert_eq!(audit.status, RtkAmbiguityFixStatus::Fixed, "audit={audit:?}");
    assert_eq!(selection.selected_ids, vec![selected_id]);
    assert_eq!(selection.excluded_ids, vec![excluded_id]);
    assert_eq!(result.selected_integers, Some(vec![0]));
    assert!((conditioned.enu_m[0] - 0.996).abs() < 1.0e-12);
    assert!((conditioned.enu_m[1] - 2.0).abs() < 1.0e-12);
    assert!((conditioned.enu_m[2] - 3.0).abs() < 1.0e-12);
}

#[test]
fn rtk_fixed_ambiguity_hold_validation_requires_consistent_fixed_state() {
    let hold = RtkFixedAmbiguityHold {
        accepted_epoch_idx: 4,
        updated_epoch_idx: 4,
        fixed_ids: vec![gps_l1_dd_id(7, 3)],
        fixed_integers: vec![0],
        ratio: Some(f64::NAN),
        conditioned_baseline: bijux_gnss_nav::api::RtkConditionedBaselineSolution {
            enu_m: [f64::INFINITY, 2.0, 3.0],
            covariance_enu_m2: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        },
        partial_selection: Some(bijux_gnss_nav::api::RtkPartialAmbiguitySelection {
            criterion: RtkPartialAmbiguitySelectionCriterion::LowestVariance,
            requested_count: 1,
            selected_count: 1,
            excluded_count: 0,
            selected_indices: vec![0],
            excluded_indices: Vec::new(),
            selected_ids: vec![gps_l1_dd_id(11, 3)],
            excluded_ids: Vec::new(),
            selected_variance_cycles2: vec![0.01],
            excluded_variance_cycles2: Vec::new(),
            max_selected_variance_cycles2: Some(0.01),
            min_excluded_variance_cycles2: None,
        }),
    };

    let events = hold.validate_payload();

    assert!(events.iter().any(|event| event.code == "RTK_FIXED_AMBIGUITY_HOLD_RATIO_INVALID"));
    assert!(events.iter().any(|event| event.code == "RTK_FIXED_AMBIGUITY_HOLD_BASELINE_INVALID"));
    assert!(events.iter().any(|event| event.code == "RTK_FIXED_AMBIGUITY_HOLD_SELECTION_MISMATCH"));
}

#[test]
fn rtk_fixed_ambiguity_hold_captures_accepted_fix_evidence() {
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
        fixer.fix_with_state(10, &float_state, &mut RtkAmbiguityFixState::default());
    let hold =
        rtk_fixed_ambiguity_hold_from_fix_result(10, &float_solution, &result).expect("held fix");

    assert_eq!(hold.accepted_epoch_idx, 10);
    assert_eq!(hold.updated_epoch_idx, 10);
    assert_eq!(hold.fixed_integers, vec![10, -3]);
    assert_eq!(hold.ratio, result.ratio);
    assert!((hold.conditioned_baseline.enu_m[0] - 0.98).abs() < 1.0e-9);
    assert!(hold.validate_payload().is_empty());
}

#[test]
fn rtk_fixed_ambiguity_hold_monitor_rejects_false_fix_residuals() {
    let fixed_id = gps_l1_dd_id(7, 3);
    let hold = RtkFixedAmbiguityHold {
        accepted_epoch_idx: 3,
        updated_epoch_idx: 3,
        fixed_ids: vec![fixed_id],
        fixed_integers: vec![10],
        ratio: Some(9.0),
        conditioned_baseline: bijux_gnss_nav::api::RtkConditionedBaselineSolution {
            enu_m: [1.0, 2.0, 3.0],
            covariance_enu_m2: [[0.5, 0.0, 0.0], [0.0, 0.6, 0.0], [0.0, 0.0, 0.7]],
        },
        partial_selection: None,
    };
    let float_solution = RtkFloatBaselineSolution {
        enu_m: [1.0, 2.0, 3.0],
        covariance_enu_m2: [[1.0, 0.0, 0.0], [0.0, 1.2, 0.0], [0.0, 0.0, 1.5]],
        enu_ambiguity_covariance_m_cycles: vec![vec![0.004], vec![0.0], vec![0.0]],
        float_ambiguities: vec![RtkFloatAmbiguityEstimate {
            sig: fixed_id.sig,
            ref_sig: fixed_id.ref_sig,
            float_cycles: 10.8,
            variance_cycles2: 0.01,
        }],
        ambiguity_covariance_cycles2: vec![vec![0.01]],
    };

    let monitor = rtk_monitor_fixed_ambiguity_hold(
        4,
        &float_solution,
        &hold,
        RtkAmbiguityHoldPolicy::default(),
    );

    assert!(!monitor.accepted);
    assert_eq!(monitor.checked_count, 1);
    assert_eq!(monitor.max_abs_float_integer_residual_cycles, Some(0.8000000000000007));
    assert!(monitor.reasons.contains(&"ambiguity_residual".to_string()));
    assert!(monitor.reasons.contains(&"normalized_ambiguity_residual".to_string()));
    assert!(monitor.validate_payload().is_empty());
}

#[test]
fn rtk_ambiguity_lifecycle_continues_valid_hold_after_ratio_failure() {
    let fixed_id = gps_l1_dd_id(7, 3);
    let initial_solution = RtkFloatBaselineSolution {
        enu_m: [1.0, 2.0, 3.0],
        covariance_enu_m2: [[1.0, 0.0, 0.0], [0.0, 1.2, 0.0], [0.0, 0.0, 1.5]],
        enu_ambiguity_covariance_m_cycles: vec![vec![0.004], vec![0.0], vec![0.0]],
        float_ambiguities: vec![RtkFloatAmbiguityEstimate {
            sig: fixed_id.sig,
            ref_sig: fixed_id.ref_sig,
            float_cycles: 10.01,
            variance_cycles2: 0.01,
        }],
        ambiguity_covariance_cycles2: vec![vec![0.01]],
    };
    let accepted = RtkAmbiguityFixResult {
        candidates: vec![RtkIntegerAmbiguityCandidate { integers: vec![10], cost: 0.01 }],
        ratio: Some(12.0),
        status: RtkAmbiguityFixStatus::Fixed,
        fixed_count: 1,
        selected_ids: Some(vec![fixed_id]),
        selected_integers: Some(vec![10]),
        partial_selection: None,
    };
    let mut lifecycle = RtkAmbiguityHoldState::default();
    let start = rtk_apply_ambiguity_fix_lifecycle(
        5,
        &initial_solution,
        &accepted,
        &mut lifecycle,
        RtkAmbiguityHoldPolicy::default(),
    );

    let next_solution = RtkFloatBaselineSolution {
        enu_m: [1.02, 2.0, 3.0],
        covariance_enu_m2: [[1.1, 0.0, 0.0], [0.0, 1.3, 0.0], [0.0, 0.0, 1.6]],
        enu_ambiguity_covariance_m_cycles: vec![vec![0.004], vec![0.0], vec![0.0]],
        float_ambiguities: vec![RtkFloatAmbiguityEstimate {
            sig: fixed_id.sig,
            ref_sig: fixed_id.ref_sig,
            float_cycles: 10.02,
            variance_cycles2: 0.01,
        }],
        ambiguity_covariance_cycles2: vec![vec![0.01]],
    };
    let failed = RtkAmbiguityFixResult {
        candidates: Vec::new(),
        ratio: Some(1.2),
        status: RtkAmbiguityFixStatus::Failed,
        fixed_count: 0,
        selected_ids: None,
        selected_integers: None,
        partial_selection: None,
    };
    let continued = rtk_apply_ambiguity_fix_lifecycle(
        6,
        &next_solution,
        &failed,
        &mut lifecycle,
        RtkAmbiguityHoldPolicy::default(),
    );

    assert_eq!(start.action, RtkAmbiguityHoldAction::HoldStarted);
    assert_eq!(continued.action, RtkAmbiguityHoldAction::HoldContinued);
    assert_eq!(continued.baseline.status, RtkAmbiguityFixStatus::Fixed);
    assert_eq!(continued.held_fix.as_ref().expect("hold").updated_epoch_idx, 6);
    assert!(continued.monitor.as_ref().expect("monitor").accepted);
    assert!(lifecycle.held_fix.is_some());
}

#[test]
fn rtk_ambiguity_lifecycle_rolls_back_false_hold_to_float_covariance() {
    let fixed_id = gps_l1_dd_id(7, 3);
    let mut lifecycle = RtkAmbiguityHoldState {
        held_fix: Some(RtkFixedAmbiguityHold {
            accepted_epoch_idx: 3,
            updated_epoch_idx: 3,
            fixed_ids: vec![fixed_id],
            fixed_integers: vec![10],
            ratio: Some(9.0),
            conditioned_baseline: bijux_gnss_nav::api::RtkConditionedBaselineSolution {
                enu_m: [1.0, 2.0, 3.0],
                covariance_enu_m2: [[0.5, 0.0, 0.0], [0.0, 0.6, 0.0], [0.0, 0.0, 0.7]],
            },
            partial_selection: None,
        }),
    };
    let float_solution = RtkFloatBaselineSolution {
        enu_m: [9.0, 8.0, 7.0],
        covariance_enu_m2: [[7.0, 0.1, 0.2], [0.1, 8.0, 0.3], [0.2, 0.3, 9.0]],
        enu_ambiguity_covariance_m_cycles: vec![vec![0.004], vec![0.0], vec![0.0]],
        float_ambiguities: vec![RtkFloatAmbiguityEstimate {
            sig: fixed_id.sig,
            ref_sig: fixed_id.ref_sig,
            float_cycles: 10.8,
            variance_cycles2: 0.01,
        }],
        ambiguity_covariance_cycles2: vec![vec![0.01]],
    };
    let failed = RtkAmbiguityFixResult {
        candidates: Vec::new(),
        ratio: Some(1.1),
        status: RtkAmbiguityFixStatus::Failed,
        fixed_count: 0,
        selected_ids: None,
        selected_integers: None,
        partial_selection: None,
    };

    let update = rtk_apply_ambiguity_fix_lifecycle(
        4,
        &float_solution,
        &failed,
        &mut lifecycle,
        RtkAmbiguityHoldPolicy::default(),
    );

    assert_eq!(update.action, RtkAmbiguityHoldAction::RolledBack);
    assert_eq!(update.baseline.status, RtkAmbiguityFixStatus::Float);
    assert_eq!(update.baseline.enu_m, float_solution.enu_m);
    assert_eq!(update.baseline.covariance_enu_m2, float_solution.covariance_enu_m2);
    assert!(update.reasons.contains(&"ambiguity_residual".to_string()));
    assert!(lifecycle.held_fix.is_none());
    assert!(update.validate_payload().is_empty());
}
