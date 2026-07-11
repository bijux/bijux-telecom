#![allow(missing_docs)]

#[path = "support/navigation_pvt_truth_table.rs"]
mod navigation_pvt_truth_table;
mod support;

use navigation_pvt_truth_table::build_pvt_truth_table_fixture;

use support::navigation_pipeline::position_error_3d_m;

#[test]
fn pvt_truth_table_records_position_clock_residual_dop_and_status_per_epoch() {
    let fixture = build_pvt_truth_table_fixture("clean_synthetic_navigation_pvt_truth", 0.0);
    let report = &fixture.report;

    assert_eq!(report.solution_count, fixture.run.solutions.len());
    assert_eq!(report.matched_epoch_count, fixture.run.solutions.len());
    assert!(report.unmatched_solution_epochs.is_empty(), "{report:?}");
    assert!(report.unused_reference_epochs.is_empty(), "{report:?}");
    assert_eq!(report.epochs.len(), fixture.run.solutions.len());

    for (epoch, solution) in report.epochs.iter().zip(&fixture.run.solutions) {
        assert_eq!(epoch.epoch_index, solution.epoch.index);
        assert_eq!(epoch.receive_time_s, solution.t_rx_s.0);

        assert_eq!(epoch.measured_ecef_m.x_m, solution.ecef_x_m.0);
        assert_eq!(epoch.measured_ecef_m.y_m, solution.ecef_y_m.0);
        assert_eq!(epoch.measured_ecef_m.z_m, solution.ecef_z_m.0);
        assert_eq!(epoch.measured_geodetic.latitude_deg, solution.latitude_deg);
        assert_eq!(epoch.measured_geodetic.longitude_deg, solution.longitude_deg);
        assert_eq!(epoch.measured_geodetic.altitude_m, solution.altitude_m.0);

        assert_eq!(epoch.truth_ecef_m.x_m, fixture.run.profile.truth_ecef_m.0);
        assert_eq!(epoch.truth_ecef_m.y_m, fixture.run.profile.truth_ecef_m.1);
        assert_eq!(epoch.truth_ecef_m.z_m, fixture.run.profile.truth_ecef_m.2);

        assert_eq!(epoch.clock_bias.truth_s, fixture.run.profile.truth_clock_bias_s);
        assert_eq!(epoch.clock_bias.measured_s, solution.clock_bias_s.0);
        assert_eq!(epoch.clock_bias.measured_m, solution.clock_bias_m.0);
        assert_eq!(
            epoch.enu_error_m.error_3d_m,
            position_error_3d_m(solution, fixture.run.profile.truth_ecef_m),
        );

        assert_eq!(epoch.residual_rms_m, solution.rms_m.0);
        assert_eq!(
            epoch.pre_fit_residual_rms_m,
            solution.pre_fit_residual_rms_m.map(|value| value.0)
        );
        assert_eq!(
            epoch.post_fit_residual_rms_m,
            solution.post_fit_residual_rms_m.map(|value| value.0)
        );
        assert_eq!(epoch.dop.pdop, solution.pdop);
        assert_eq!(epoch.dop.hdop, solution.hdop);
        assert_eq!(epoch.dop.vdop, solution.vdop);
        assert_eq!(epoch.dop.gdop, solution.gdop);
        assert_eq!(epoch.dop.tdop, solution.tdop);
        assert_eq!(epoch.solution_status, solution.status);
        assert_eq!(epoch.solution_quality, solution.quality);
        assert_eq!(epoch.solution_validity, solution.validity);
        assert_eq!(epoch.valid, solution.valid);
    }
}
