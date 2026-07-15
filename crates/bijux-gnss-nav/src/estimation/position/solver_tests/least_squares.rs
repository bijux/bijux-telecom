use super::*;

#[test]
fn weighted_least_squares_recovers_full_rank_solution() {
    let h = vec![
        vec![1.0, 0.0, 0.0, 0.0],
        vec![0.0, 1.0, 0.0, 0.0],
        vec![0.0, 0.0, 1.0, 0.0],
        vec![0.0, 0.0, 0.0, 1.0],
        vec![1.0, -1.0, 0.5, 2.0],
    ];
    let expected_delta = [2.0, -1.0, 0.5, 3.0];
    let v = h
        .iter()
        .map(|row| row.iter().zip(expected_delta).map(|(left, right)| left * right).sum())
        .collect::<Vec<_>>();

    let solution =
        solve_weighted_least_squares(&h, &v, &[1.0; 5]).expect("full-rank geometry should solve");

    assert_eq!(solution.rank, 4);
    assert!(solution.condition_number.is_some_and(|condition| condition.is_finite()));
    for (actual, expected) in solution.delta.iter().zip(expected_delta) {
        assert!((actual - expected).abs() < 1.0e-10);
    }
}

#[test]
fn weighted_least_squares_rejects_rank_deficient_geometry() {
    let h = vec![
        vec![1.0, 0.0, 0.0, 1.0],
        vec![1.0, 0.0, 0.0, 1.0],
        vec![1.0, 0.0, 0.0, 1.0],
        vec![1.0, 0.0, 0.0, 1.0],
    ];

    assert!(solve_weighted_least_squares(&h, &[1.0; 4], &[1.0; 4]).is_none());
}

#[test]
fn weighted_least_squares_reports_larger_condition_for_weak_geometry() {
    let healthy = vec![
        vec![1.0, 0.0, 0.0, 0.0],
        vec![0.0, 1.0, 0.0, 0.0],
        vec![0.0, 0.0, 1.0, 0.0],
        vec![0.0, 0.0, 0.0, 1.0],
    ];
    let weak = vec![
        vec![1.0, 0.0, 0.0, 1.0],
        vec![0.0, 1.0, 0.0, 1.0],
        vec![0.0, 0.0, 1.0, 1.0],
        vec![1.0, 1.0, 1.0, 3.000_001],
        vec![1.0, -1.0, 0.0, 0.0],
    ];

    let healthy_condition = solve_weighted_least_squares(&healthy, &[0.0; 4], &[1.0; 4])
        .and_then(|solution| solution.condition_number)
        .expect("healthy geometry condition");
    let weak_condition = solve_weighted_least_squares(&weak, &[0.0; 5], &[1.0; 5])
        .and_then(|solution| solution.condition_number)
        .expect("weak geometry condition");

    assert!(weak_condition > healthy_condition);
}
