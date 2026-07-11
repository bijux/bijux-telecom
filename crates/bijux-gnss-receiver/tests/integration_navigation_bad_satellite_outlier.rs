#![allow(missing_docs)]

mod support;

use support::navigation_outlier::{
    bad_satellite_prn, rejected_outlier_prns, single_bad_satellite_navigation_run,
    solution_position_errors_m,
};

#[test]
fn navigation_pipeline_rejects_bad_satellite_pseudorange_and_preserves_truth() {
    let run = single_bad_satellite_navigation_run();
    let rejected_prns = rejected_outlier_prns(&run);
    let position_errors_m = solution_position_errors_m(&run);
    let suspect_reason = format!("raim_suspect_prn={}", bad_satellite_prn());

    assert!(
        !run.run.solutions.is_empty(),
        "expected navigation solutions from the bad-satellite synthetic run",
    );
    assert!(
        run.run.solutions.iter().all(|solution| solution.valid),
        "expected the remaining clean satellites to keep the navigation solution valid: {:?}",
        run.run
            .solutions
            .iter()
            .map(|solution| (solution.epoch.index, solution.used_sat_count, solution.rejected_sat_count))
            .collect::<Vec<_>>(),
    );
    assert!(
        rejected_prns.iter().any(|prn| *prn == bad_satellite_prn()),
        "expected PRN {} to be rejected as an outlier: {:?}",
        bad_satellite_prn(),
        rejected_prns,
    );
    assert!(
        run.run
            .solutions
            .iter()
            .any(|solution| solution.explain_reasons.iter().any(|reason| reason == "raim_fault_detected")),
        "expected at least one navigation epoch to report RAIM fault detection: {:?}",
        run.run
            .solutions
            .iter()
            .map(|solution| (solution.epoch.index, solution.status, solution.explain_reasons.clone()))
            .collect::<Vec<_>>(),
    );
    assert!(
        run.run
            .solutions
            .iter()
            .any(|solution| solution.explain_reasons.iter().any(|reason| reason == &suspect_reason)),
        "expected at least one navigation epoch to name PRN {} as the RAIM suspect: {:?}",
        bad_satellite_prn(),
        run.run
            .solutions
            .iter()
            .map(|solution| (solution.epoch.index, solution.status, solution.explain_reasons.clone()))
            .collect::<Vec<_>>(),
    );
    assert!(
        run.run.solutions.iter().all(|solution| solution.used_sat_count >= 5),
        "expected at least five satellites to remain after excluding the bad measurement: {:?}",
        run.run
            .solutions
            .iter()
            .map(|solution| (solution.epoch.index, solution.used_sat_count, solution.rejected_sat_count))
            .collect::<Vec<_>>(),
    );
    assert!(
        position_errors_m.iter().all(|error_m| *error_m < 10.0),
        "bad-satellite handling should keep the solution near truth: {:?}",
        position_errors_m,
    );
}
