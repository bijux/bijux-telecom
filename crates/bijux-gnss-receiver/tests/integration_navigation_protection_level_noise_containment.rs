#![allow(missing_docs)]

mod support;

use support::navigation_noise::synthetic_pseudorange_noise_sweep_profiles;
use support::navigation_pipeline::noisy_synthetic_navigation_run;
use support::navigation_protection_faults::noisy_run_validation;

#[test]
fn moderate_synthetic_faults_remain_contained_by_reported_protection_levels() {
    let profile = synthetic_pseudorange_noise_sweep_profiles()
        .into_iter()
        .find(|profile| profile.profile_name == "moderate_pseudorange_noise")
        .expect("moderate synthetic fault profile");
    let run = noisy_synthetic_navigation_run(profile);
    let report = noisy_run_validation(&run);

    assert!(
        !report.reference_position_errors.is_empty(),
        "expected reference-matched epochs from moderate synthetic fault run",
    );
    assert_eq!(
        report.protection_levels.horizontal_contained_epoch_count,
        report.protection_levels.horizontal_reported_epoch_count,
    );
    assert_eq!(
        report.protection_levels.vertical_contained_epoch_count,
        report.protection_levels.vertical_reported_epoch_count,
    );
    assert!(report.protection_levels.horizontal_breach_epochs.is_empty());
    assert!(report.protection_levels.vertical_breach_epochs.is_empty());
}
