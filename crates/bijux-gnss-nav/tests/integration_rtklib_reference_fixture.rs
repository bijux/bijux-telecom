#![allow(missing_docs)]

mod support;

use support::public_station_truth::{public_station_truth_by_fixture, station_enu_error_m};
use support::rtklib_reference::ab43_rtklib_single_reference;

#[test]
fn rtklib_reference_fixture_covers_ab43_public_spp_epochs() {
    let reference = ab43_rtklib_single_reference();
    let station_truth = public_station_truth_by_fixture("unavco_ab43_20180114.obs");

    assert_eq!(reference.len(), 9, "RTKLIB AB43 reference should cover nine public epochs");

    let mut previous_epoch_s: Option<f64> = None;
    for epoch in reference {
        assert_eq!(epoch.quality_flag, 5, "RTKLIB AB43 epoch should be a single-point solution");
        assert_eq!(
            epoch.residuals.len(),
            7,
            "RTKLIB AB43 epoch at TOW {:.1}s should retain seven GPS residuals",
            epoch.gps_time.tow_s
        );
        let epoch_s = epoch.gps_time.week as f64 * 604_800.0 + epoch.gps_time.tow_s;
        if let Some(previous_epoch_s) = previous_epoch_s {
            assert!(
                (epoch_s - previous_epoch_s - 15.0).abs() < 1.0e-9,
                "RTKLIB AB43 epoch spacing drifted: previous={previous_epoch_s:.1}s current={epoch_s:.1}s",
            );
        }
        previous_epoch_s = Some(epoch_s);

        let enu_error = station_enu_error_m(epoch.ecef_m, &station_truth);
        assert!(
            enu_error.horizontal_m < 1.5,
            "RTKLIB AB43 horizontal error {:.3} m exceeds fixture bound at TOW {:.1}s",
            enu_error.horizontal_m,
            epoch.gps_time.tow_s
        );
        assert!(
            enu_error.three_dimensional_m < 8.5,
            "RTKLIB AB43 three-dimensional error {:.3} m exceeds fixture bound at TOW {:.1}s",
            enu_error.three_dimensional_m,
            epoch.gps_time.tow_s
        );
    }
}
