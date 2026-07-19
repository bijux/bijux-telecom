#![allow(missing_docs)]

mod support;

use bijux_gnss_core::api::{NavRefusalClass, SolutionStatus};

use support::navigation_outlier::underdetermined_bad_satellite_navigation_run;
use support::navigation_protection_faults::noisy_run_validation;

#[test]
fn underdetermined_fault_triggers_integrity_failure_without_protection_claims() {
    let run = underdetermined_bad_satellite_navigation_run();
    let report = noisy_run_validation(&run);

    assert!(
        !run.run.solutions.is_empty(),
        "expected underdetermined synthetic fault run to produce solved epochs",
    );
    assert!(
        run.run.solutions.iter().all(|solution| solution.status == SolutionStatus::IntegrityFailed),
        "expected every underdetermined fault epoch to refuse integrity",
    );
    assert!(
        run.run
            .solutions
            .iter()
            .all(|solution| solution.refusal_class == Some(NavRefusalClass::InsufficientGeometry)),
        "expected underdetermined fault refusal to map to insufficient geometry",
    );
    assert!(
        run.run.solutions.iter().all(|solution| solution.integrity_hpl_m.is_none()),
        "underdetermined fault epochs must not claim HPL after refusal",
    );
    assert!(
        run.run.solutions.iter().all(|solution| solution.integrity_vpl_m.is_none()),
        "underdetermined fault epochs must not claim VPL after refusal",
    );
    assert_eq!(report.protection_levels.horizontal_reported_epoch_count, 0);
    assert_eq!(report.protection_levels.vertical_reported_epoch_count, 0);
}
