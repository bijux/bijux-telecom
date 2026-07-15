#![allow(missing_docs)]

mod support;

use support::public_spp_case::{ab43_public_spp_case, solve_public_ab43_epoch};
use support::rtklib_reference::ab43_rtklib_single_reference;

const RTKLIB_POSITION_DELTA_TOLERANCE_M: f64 = 13.0;

#[test]
fn public_single_point_solution_stays_near_rtklib_reference_positions() {
    let case = ab43_public_spp_case();
    let reference = ab43_rtklib_single_reference();

    assert_eq!(
        case.observations.epochs.len(),
        reference.len(),
        "AB43 public observations and RTKLIB reference should cover the same epoch count"
    );

    let mut compared_epochs = 0usize;
    for (epoch, reference_epoch) in case.observations.epochs.iter().zip(reference.iter()) {
        let Ok(solved) = solve_public_ab43_epoch(epoch) else {
            continue;
        };
        assert_eq!(
            solved.gps_time, reference_epoch.gps_time,
            "AB43 public epoch timing should align with the RTKLIB reference"
        );

        let dx_m = solved.ecef_m.0 - reference_epoch.ecef_m.0;
        let dy_m = solved.ecef_m.1 - reference_epoch.ecef_m.1;
        let dz_m = solved.ecef_m.2 - reference_epoch.ecef_m.2;
        let position_delta_m = (dx_m * dx_m + dy_m * dy_m + dz_m * dz_m).sqrt();

        assert!(
            position_delta_m <= RTKLIB_POSITION_DELTA_TOLERANCE_M,
            "AB43 public epoch at GPS week {} TOW {:.1}s drifted {:.3} m from RTKLIB (dx={:.3} m, dy={:.3} m, dz={:.3} m)",
            solved.gps_time.week,
            solved.gps_time.tow_s,
            position_delta_m,
            dx_m,
            dy_m,
            dz_m
        );
        compared_epochs += 1;
    }

    assert!(compared_epochs > 0, "expected at least one AB43 public epoch to compare with RTKLIB");
}
