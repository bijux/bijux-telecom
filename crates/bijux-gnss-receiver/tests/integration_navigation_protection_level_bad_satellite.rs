#![allow(missing_docs)]

mod support;

use support::navigation_outlier::{
    bad_satellite_prn, single_bad_satellite_navigation_run,
};
use support::navigation_protection_faults::{noisy_run_validation, max_reported_hpl_m, max_reported_vpl_m};

#[test]
fn bad_satellite_fault_preserves_contained_protection_levels_after_exclusion() {
    let run = single_bad_satellite_navigation_run();
    let report = noisy_run_validation(&run);
    let suspect_reason = format!("raim_suspect_prn={}", bad_satellite_prn());
    let excluded_reason = format!("raim_excluded_prn={}", bad_satellite_prn());

    assert!(
        run.run.solutions.iter().any(|solution| {
            solution.explain_reasons.iter().any(|reason| reason == "raim_fault_detected")
        }),
        "expected at least one solved epoch to report RAIM fault detection",
    );
    assert!(
        run.run.solutions.iter().any(|solution| {
            solution.explain_reasons.iter().any(|reason| reason == "raim_fault_excluded")
        }),
        "expected at least one solved epoch to report RAIM fault exclusion",
    );
    assert!(
        run.run.solutions.iter().any(|solution| {
            solution.explain_reasons.iter().any(|reason| reason == &suspect_reason)
        }),
        "expected at least one solved epoch to name the suspected satellite",
    );
    assert!(
        run.run.solutions.iter().any(|solution| {
            solution.explain_reasons.iter().any(|reason| reason == &excluded_reason)
        }),
        "expected at least one solved epoch to name the excluded satellite",
    );
    assert!(
        max_reported_hpl_m(&run.run).is_some(),
        "expected solved epochs to keep reporting HPL after fault exclusion",
    );
    assert!(
        max_reported_vpl_m(&run.run).is_some(),
        "expected solved epochs to keep reporting VPL after fault exclusion",
    );
    assert!(
        report.protection_levels.horizontal_breach_epochs.is_empty(),
        "fault exclusion should keep horizontal truth error within reported HPL: {:?}",
        report.protection_levels.horizontal_breach_epochs,
    );
    assert!(
        report.protection_levels.vertical_breach_epochs.is_empty(),
        "fault exclusion should keep vertical truth error within reported VPL: {:?}",
        report.protection_levels.vertical_breach_epochs,
    );
}
