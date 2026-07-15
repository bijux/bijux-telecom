#![allow(missing_docs)]

use bijux_gnss_core::api::{Constellation, SatId, SigId, SignalBand};
use bijux_gnss_nav::api::{
    rtk_ambiguity_state_from_fixed_solution, rtk_conditioned_baseline_from_fixed_ambiguities,
    rtk_float_ambiguity_state_from_baseline_solution, rtk_float_baseline_from_double_differences,
    rtk_integer_ambiguity_candidates, rtk_lambda_decorrelate,
    rtk_lambda_integer_ambiguity_candidates, rtk_select_partial_ambiguity_fix_with_evidence,
    rtk_transform_fixed_ambiguity_reference, rtk_transform_float_ambiguity_reference,
    rtk_transform_float_baseline_reference, RtkAmbiguityFixPolicy, RtkAmbiguityFixState,
    RtkAmbiguityFixStatus, RtkDoubleDifferenceAmbiguityId, RtkFloatAmbiguityEstimate,
    RtkFloatAmbiguityState, RtkFloatBaselineSolution, RtkPartialAmbiguitySelectionCriterion,
    RtkRatioTestFixer,
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
