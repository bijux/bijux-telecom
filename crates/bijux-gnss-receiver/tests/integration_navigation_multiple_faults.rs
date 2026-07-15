#![allow(missing_docs)]

mod support;

use support::navigation_outlier::{
    rejected_outlier_prns, simultaneous_bad_satellite_navigation_run,
    simultaneous_bad_satellite_prns,
};

#[test]
fn navigation_pipeline_handles_simultaneous_raim_faults_fail_closed() {
    let run = simultaneous_bad_satellite_navigation_run();
    let rejected_prns = rejected_outlier_prns(&run);
    let injected_prns = simultaneous_bad_satellite_prns();
    let isolated_faults =
        injected_prns.iter().all(|prn| rejected_prns.iter().any(|row| row == prn));
    let integrity_unavailable = run.run.solutions.iter().any(|solution| {
        solution.explain_reasons.iter().any(|reason| reason == "integrity_unavailable")
            || solution.explain_reasons.iter().any(|reason| reason == "raim_multi_fault_unresolved")
    });

    assert!(
        !run.run.solutions.is_empty(),
        "expected navigation epochs for simultaneous-fault synthetic run",
    );
    assert!(
        isolated_faults || integrity_unavailable,
        "multiple RAIM faults must be isolated or fail closed: rejected={rejected_prns:?} solutions={:?}",
        run.run
            .solutions
            .iter()
            .map(|solution| (
                solution.epoch.index,
                solution.status,
                solution.valid,
                solution.explain_reasons.clone()
            ))
            .collect::<Vec<_>>(),
    );
    assert!(
        run.run.solutions.iter().all(|solution| {
            solution
                .explain_reasons
                .iter()
                .any(|reason| reason.starts_with("raim_multi_fault_hypotheses="))
                || solution
                    .explain_reasons
                    .iter()
                    .any(|reason| reason == "raim_exclusion_underdetermined")
                || solution
                    .explain_reasons
                    .iter()
                    .any(|reason| reason == "navigation_solution_unavailable")
        }),
        "simultaneous-fault epochs should carry RAIM multi-hypothesis evidence: {:?}",
        run.run
            .solutions
            .iter()
            .map(|solution| (
                solution.epoch.index,
                solution.status,
                solution.explain_reasons.clone()
            ))
            .collect::<Vec<_>>(),
    );
}
