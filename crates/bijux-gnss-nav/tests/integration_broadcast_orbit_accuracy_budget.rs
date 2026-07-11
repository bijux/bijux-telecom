#![allow(missing_docs)]

mod support;

use support::orbit_reference::{
    broadcast_orbit_accuracy_budget, broadcast_reference_fixtures, direct_broadcast_orbit_errors,
    validate_broadcast_orbit_accuracy_budget,
};

#[test]
fn broadcast_orbit_accuracy_budget_enforces_hard_reference_thresholds() {
    let mut errors = Vec::new();
    for fixture in broadcast_reference_fixtures() {
        errors.extend(direct_broadcast_orbit_errors(&fixture));
    }

    let budget = broadcast_orbit_accuracy_budget();
    let report = validate_broadcast_orbit_accuracy_budget(&errors, budget);

    assert!(report.pass, "{report:?}");
    assert_eq!(report.sample_count, errors.len());
    assert_eq!(report.max_position_error_m, budget.max_position_error_m);
    assert_eq!(report.max_rms_position_error_m, budget.max_rms_position_error_m);
    assert!(report.observed_max_position_error_m <= report.max_position_error_m + f64::EPSILON);
    assert!(
        report.observed_rms_position_error_m <= report.max_rms_position_error_m + f64::EPSILON
    );
}
