#![allow(missing_docs)]

#[path = "support/navigation_time_profile.rs"]
mod navigation_time_profile;

use bijux_gnss_receiver::api::{sim::validate_pvt_covariance_realism, CovarianceCoverageClass};

use navigation_time_profile::{build_navigation_time_cases, navigation_time_case};

fn assert_coverage_class(
    label: &str,
    actual: CovarianceCoverageClass,
    expected: CovarianceCoverageClass,
    report: &bijux_gnss_receiver::api::CovarianceRealismReport,
) {
    assert_eq!(actual, expected, "{label} covariance realism report: {report:#?}");
}

#[test]
fn navigation_covariance_realism_respects_profile_sample_support() {
    let cases = build_navigation_time_cases();
    let stabilizing = navigation_time_case(&cases, "stabilizing_navigation_accuracy");
    let drifting = navigation_time_case(&cases, "drifting_navigation_accuracy");
    let diverging = navigation_time_case(&cases, "diverging_navigation_accuracy");

    let stabilizing_realism = validate_pvt_covariance_realism(&stabilizing.truth_table);
    let drifting_realism = validate_pvt_covariance_realism(&drifting.truth_table);
    let diverging_realism = validate_pvt_covariance_realism(&diverging.truth_table);

    assert_coverage_class(
        "stabilizing horizontal",
        stabilizing_realism.horizontal_95.classification,
        CovarianceCoverageClass::InsufficientData,
        &stabilizing_realism,
    );
    assert_coverage_class(
        "stabilizing vertical",
        stabilizing_realism.vertical_95.classification,
        CovarianceCoverageClass::InsufficientData,
        &stabilizing_realism,
    );
    assert_coverage_class(
        "stabilizing position",
        stabilizing_realism.position_3d_95.classification,
        CovarianceCoverageClass::InsufficientData,
        &stabilizing_realism,
    );
    assert_eq!(stabilizing_realism.total_epoch_count, 250);
    assert!(stabilizing_realism.covariance_epoch_count > 0);
    assert!(stabilizing_realism.horizontal_95.sample_count < 20);
    assert!(stabilizing_realism.warnings.is_empty());

    assert_coverage_class(
        "drifting horizontal",
        drifting_realism.horizontal_95.classification,
        CovarianceCoverageClass::Conservative,
        &drifting_realism,
    );
    assert_coverage_class(
        "drifting vertical",
        drifting_realism.vertical_95.classification,
        CovarianceCoverageClass::Conservative,
        &drifting_realism,
    );
    assert_coverage_class(
        "drifting position",
        drifting_realism.position_3d_95.classification,
        CovarianceCoverageClass::Conservative,
        &drifting_realism,
    );
    assert_eq!(drifting_realism.total_epoch_count, 250);
    assert_eq!(drifting_realism.covariance_epoch_count, 250);
    assert_eq!(drifting_realism.warnings.len(), 3);

    assert_coverage_class(
        "diverging horizontal",
        diverging_realism.horizontal_95.classification,
        CovarianceCoverageClass::InsufficientData,
        &diverging_realism,
    );
    assert_coverage_class(
        "diverging vertical",
        diverging_realism.vertical_95.classification,
        CovarianceCoverageClass::InsufficientData,
        &diverging_realism,
    );
    assert_coverage_class(
        "diverging position",
        diverging_realism.position_3d_95.classification,
        CovarianceCoverageClass::InsufficientData,
        &diverging_realism,
    );
    assert_eq!(diverging_realism.total_epoch_count, 250);
    assert!(diverging_realism.covariance_epoch_count > 0);
    assert!(diverging_realism.horizontal_95.sample_count < 20);
    assert!(diverging_realism.warnings.is_empty());
}
