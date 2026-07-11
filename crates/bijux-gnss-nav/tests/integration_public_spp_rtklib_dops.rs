#![allow(missing_docs)]

mod support;

use bijux_gnss_nav::position_dops_from_satellite_positions;

use support::public_spp_case::{public_ab43_epoch, public_ab43_satellite_geometry};
use support::rtklib_reference::{ab43_rtklib_dop_reference, ab43_rtklib_single_reference};

fn assert_close(label: &str, actual: f64, expected: f64, tolerance: f64) {
    let error = (actual - expected).abs();
    assert!(
        error <= tolerance,
        "{label} mismatch: actual={actual:.12} expected={expected:.12} error={error:.12} tolerance={tolerance:.12}"
    );
}

#[test]
fn public_ab43_geometry_dops_track_rtklib_reference() {
    let rtklib_positions = ab43_rtklib_single_reference();
    let rtklib_dops = ab43_rtklib_dop_reference();

    assert_eq!(
        rtklib_positions.len(),
        rtklib_dops.len(),
        "RTKLIB AB43 position and DOP references should cover the same epochs"
    );

    for (position_epoch, dop_epoch) in rtklib_positions.iter().zip(rtklib_dops.iter()) {
        assert_eq!(
            position_epoch.gps_time, dop_epoch.gps_time,
            "RTKLIB AB43 position and DOP references should align by epoch"
        );

        let epoch = public_ab43_epoch(dop_epoch.gps_time).expect("find AB43 public epoch");
        let geometry =
            public_ab43_satellite_geometry(epoch, &dop_epoch.satellites).expect("build AB43 geometry");
        let satellite_positions = geometry.iter().map(|satellite| satellite.ecef_m).collect::<Vec<_>>();
        let dops = position_dops_from_satellite_positions(
            [
                position_epoch.ecef_m.0,
                position_epoch.ecef_m.1,
                position_epoch.ecef_m.2,
            ],
            &satellite_positions,
        )
        .expect("compute public AB43 geometry DOPs");

        assert_close("gdop", dops.gdop, dop_epoch.gdop, 0.05);
        assert_close("pdop", dops.pdop, dop_epoch.pdop, 0.05);
        assert_close("hdop", dops.hdop, dop_epoch.hdop, 0.05);
        assert_close("vdop", dops.vdop, dop_epoch.vdop, 0.05);
    }
}
