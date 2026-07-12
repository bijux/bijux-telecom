#![allow(missing_docs)]

mod support;

use bijux_gnss_nav::api::formal_protection_levels;

use support::navigation_pipeline::clean_synthetic_navigation_run;

#[test]
fn navigation_pipeline_reports_formal_protection_levels() {
    let run = clean_synthetic_navigation_run();

    assert!(
        !run.solutions.is_empty(),
        "expected clean synthetic navigation run to produce solutions",
    );

    for solution in &run.solutions {
        let covariance = solution.position_covariance_ecef_m2.expect("position covariance");
        let expected = formal_protection_levels(
            [solution.ecef_x_m.0, solution.ecef_y_m.0, solution.ecef_z_m.0],
            covariance,
        )
        .expect("formal protection levels");

        assert_eq!(solution.integrity_hpl_m, Some(expected.horizontal_m));
        assert_eq!(solution.integrity_vpl_m, Some(expected.vertical_m));
    }
}
