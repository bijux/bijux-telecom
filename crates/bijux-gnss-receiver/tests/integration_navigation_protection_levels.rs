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

        assert_option_close(solution.integrity_hpl_m, Some(expected.horizontal_m), 1.0e-12);
        assert_option_close(solution.integrity_vpl_m, Some(expected.vertical_m), 1.0e-12);
    }
}

fn assert_option_close(actual: Option<f64>, expected: Option<f64>, tolerance: f64) {
    match (actual, expected) {
        (Some(actual), Some(expected)) => {
            let error = (actual - expected).abs();
            assert!(
                error <= tolerance,
                "value mismatch: actual={actual:.15} expected={expected:.15} error={error:.15} tolerance={tolerance:.15}"
            );
        }
        (None, None) => {}
        _ => panic!("option mismatch: actual={actual:?} expected={expected:?}"),
    }
}
