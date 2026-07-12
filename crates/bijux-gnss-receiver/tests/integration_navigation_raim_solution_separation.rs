#![allow(missing_docs)]

mod support;

use support::navigation_outlier::single_bad_satellite_navigation_run;

fn explain_value(solution: &bijux_gnss_core::api::NavSolutionEpoch, key: &str) -> Option<usize> {
    solution.explain_reasons.iter().find_map(|reason| {
        reason
            .strip_prefix(key)
            .and_then(|value| value.parse::<usize>().ok())
    })
}

#[test]
fn navigation_pipeline_reports_subset_solution_comparisons_for_raim() {
    let run = single_bad_satellite_navigation_run();
    let compared_solutions = run
        .run
        .solutions
        .iter()
        .filter_map(|solution| {
            let reference_satellites =
                explain_value(solution, "raim_solution_separation_reference_satellites=")?;
            let compared_subsets =
                explain_value(solution, "raim_solution_separation_compared_subsets=")?;
            Some((solution, reference_satellites, compared_subsets))
        })
        .collect::<Vec<_>>();

    assert!(
        !compared_solutions.is_empty(),
        "expected navigation solutions to surface RAIM subset comparisons: {:?}",
        run.run
            .solutions
            .iter()
            .map(|solution| (solution.epoch.index, solution.status, solution.explain_reasons.clone()))
            .collect::<Vec<_>>(),
    );
    for (solution, reference_satellites, compared_subsets) in compared_solutions {
        assert_eq!(
            reference_satellites, solution.used_sat_count,
            "solution should report RAIM reference satellites from the final working set: {solution:?}",
        );
        assert_eq!(
            compared_subsets, reference_satellites,
            "solution should compare one solved subset per retained satellite: {solution:?}",
        );
    }
}
