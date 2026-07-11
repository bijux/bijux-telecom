use bijux_gnss_nav::api::{geodetic_to_ecef, position_dops_from_satellite_positions};

fn satellite_positions_ecef(geodetic_points: &[(f64, f64, f64)]) -> Vec<[f64; 3]> {
    geodetic_points
        .iter()
        .map(|&(lat_deg, lon_deg, alt_m)| {
            let (x_m, y_m, z_m) = geodetic_to_ecef(lat_deg, lon_deg, alt_m);
            [x_m, y_m, z_m]
        })
        .collect()
}

fn enu_offset_satellite_positions(
    receiver_lat_deg: f64,
    receiver_lon_deg: f64,
    receiver_alt_m: f64,
    enu_offsets_m: &[[f64; 3]],
) -> Vec<[f64; 3]> {
    let (rx_x_m, rx_y_m, rx_z_m) =
        geodetic_to_ecef(receiver_lat_deg, receiver_lon_deg, receiver_alt_m);
    let lat = receiver_lat_deg.to_radians();
    let lon = receiver_lon_deg.to_radians();
    let sin_lat = lat.sin();
    let cos_lat = lat.cos();
    let sin_lon = lon.sin();
    let cos_lon = lon.cos();

    enu_offsets_m
        .iter()
        .map(|offset| {
            let east_m = offset[0];
            let north_m = offset[1];
            let up_m = offset[2];
            let dx_m = -sin_lon * east_m - sin_lat * cos_lon * north_m + cos_lat * cos_lon * up_m;
            let dy_m = cos_lon * east_m - sin_lat * sin_lon * north_m + cos_lat * sin_lon * up_m;
            let dz_m = cos_lat * north_m + sin_lat * up_m;
            [rx_x_m + dx_m, rx_y_m + dy_m, rx_z_m + dz_m]
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

#[test]
fn geometry_dops_match_regular_tetrahedron_reference() {
    let receiver_lat_deg = 37.0;
    let receiver_lon_deg = -122.0;
    let receiver_alt_m = 10.0;
    let receiver_ecef_m = {
        let (x_m, y_m, z_m) = geodetic_to_ecef(receiver_lat_deg, receiver_lon_deg, receiver_alt_m);
        [x_m, y_m, z_m]
    };
    let radius_m = 20_200_000.0;
    let scale = radius_m / 3.0_f64.sqrt();
    let satellites = enu_offset_satellite_positions(
        receiver_lat_deg,
        receiver_lon_deg,
        receiver_alt_m,
        &[
            [scale, scale, scale],
            [scale, -scale, -scale],
            [-scale, scale, -scale],
            [-scale, -scale, scale],
        ],
    );

    let dops = position_dops_from_satellite_positions(receiver_ecef_m, &satellites)
        .expect("tetrahedron dops");

    assert!((dops.hdop - 1.224_744_871_391_589).abs() < 1.0e-12);
    assert!((dops.vdop - 0.866_025_403_784_438_6).abs() < 1.0e-12);
    assert!((dops.pdop - 1.5).abs() < 1.0e-12);
    assert!((dops.tdop - 0.5).abs() < 1.0e-12);
    assert!((dops.gdop - 1.581_138_830_084_189_8).abs() < 1.0e-12);
}
