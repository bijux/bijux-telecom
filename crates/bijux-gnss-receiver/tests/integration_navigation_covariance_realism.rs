#![allow(missing_docs)]

#[path = "support/navigation_time_profile.rs"]
mod navigation_time_profile;

use bijux_gnss_receiver::api::{
    sim::validate_pvt_covariance_realism, CovarianceCoverageClass,
};

use navigation_time_profile::{build_navigation_time_cases, navigation_time_case};

#[test]
fn navigation_covariance_realism_distinguishes_conservative_and_optimistic_profiles() {
    let cases = build_navigation_time_cases();
    let stabilizing =
        navigation_time_case(&cases, "stabilizing_navigation_accuracy");
    let drifting = navigation_time_case(&cases, "drifting_navigation_accuracy");
    let diverging = navigation_time_case(&cases, "diverging_navigation_accuracy");

    let stabilizing_realism = validate_pvt_covariance_realism(&stabilizing.truth_table);
    let drifting_realism = validate_pvt_covariance_realism(&drifting.truth_table);
    let diverging_realism = validate_pvt_covariance_realism(&diverging.truth_table);

    assert_eq!(
        stabilizing_realism.horizontal_95.classification,
        CovarianceCoverageClass::Conservative
    );
    assert_eq!(
        stabilizing_realism.vertical_95.classification,
        CovarianceCoverageClass::Conservative
    );
    assert_eq!(
        stabilizing_realism.position_3d_95.classification,
        CovarianceCoverageClass::Conservative
    );
    assert_eq!(stabilizing_realism.warnings.len(), 3);

    assert_eq!(
        drifting_realism.horizontal_95.classification,
        CovarianceCoverageClass::Conservative
    );
    assert_eq!(
        drifting_realism.vertical_95.classification,
        CovarianceCoverageClass::Conservative
    );
    assert_eq!(
        drifting_realism.position_3d_95.classification,
        CovarianceCoverageClass::Conservative
    );
    assert_eq!(drifting_realism.warnings.len(), 3);

    assert_eq!(
        diverging_realism.horizontal_95.classification,
        CovarianceCoverageClass::Optimistic
    );
    assert_eq!(
        diverging_realism.vertical_95.classification,
        CovarianceCoverageClass::Optimistic
    );
    assert_eq!(
        diverging_realism.position_3d_95.classification,
        CovarianceCoverageClass::Optimistic
    );
    assert_eq!(diverging_realism.warnings.len(), 3);
    assert!(
        diverging_realism.position_3d_95.observed_rate.expect("diverging observed rate") < 0.5
    );
}
