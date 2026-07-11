use bijux_gnss_nav::api::geodetic_to_ecef;
use bijux_gnss_nav::position_dops_from_satellite_positions;

fn satellite_positions_ecef(geodetic_points: &[(f64, f64, f64)]) -> Vec<[f64; 3]> {
    geodetic_points
        .iter()
        .map(|&(lat_deg, lon_deg, alt_m)| {
            let (x_m, y_m, z_m) = geodetic_to_ecef(lat_deg, lon_deg, alt_m);
            [x_m, y_m, z_m]
        })
        .collect()
}

#[test]
fn geometry_dops_require_four_satellites() {
    let receiver_ecef_m = {
        let (x_m, y_m, z_m) = geodetic_to_ecef(37.0, -122.0, 10.0);
        [x_m, y_m, z_m]
    };
    let satellites = satellite_positions_ecef(&[
        (55.0, -150.0, 20_200_000.0),
        (10.0, -40.0, 20_200_000.0),
        (-20.0, 80.0, 20_200_000.0),
    ]);

    assert!(position_dops_from_satellite_positions(receiver_ecef_m, &satellites).is_none());
}

#[test]
fn geometry_dops_worsen_when_satellites_cluster() {
    let receiver_ecef_m = {
        let (x_m, y_m, z_m) = geodetic_to_ecef(37.0, -122.0, 10.0);
        [x_m, y_m, z_m]
    };
    let wide_geometry = satellite_positions_ecef(&[
        (60.0, -160.0, 20_200_000.0),
        (10.0, -40.0, 20_200_000.0),
        (-25.0, 45.0, 20_200_000.0),
        (50.0, 110.0, 20_200_000.0),
    ]);
    let clustered_geometry = satellite_positions_ecef(&[
        (41.0, -121.0, 20_200_000.0),
        (42.0, -119.5, 20_200_000.0),
        (43.0, -118.0, 20_200_000.0),
        (44.0, -116.5, 20_200_000.0),
    ]);

    let wide_dops =
        position_dops_from_satellite_positions(receiver_ecef_m, &wide_geometry).expect("wide dops");
    let clustered_dops =
        position_dops_from_satellite_positions(receiver_ecef_m, &clustered_geometry)
            .expect("clustered dops");

    assert!(wide_dops.pdop.is_finite());
    assert!(wide_dops.gdop.is_finite());
    assert!(clustered_dops.pdop > wide_dops.pdop);
    assert!(clustered_dops.gdop > wide_dops.gdop);
}
