//! Independent coordinate transforms exposed for test truth and support fixtures.

use crate::reference_math::coordinates::{
    ecef_to_enu_m as reference_ecef_to_enu_m,
    ecef_to_geodetic_point as reference_ecef_to_geodetic_point,
    elevation_azimuth_deg as reference_elevation_azimuth_deg,
    enu_to_ecef_m as reference_enu_to_ecef_m, geodetic_to_ecef_m as reference_geodetic_to_ecef_m,
    EnuVector, GeodeticPoint,
};

pub fn geodetic_to_ecef(lat_deg: f64, lon_deg: f64, alt_m: f64) -> (f64, f64, f64) {
    let ecef_m = reference_geodetic_to_ecef_m(GeodeticPoint { lat_deg, lon_deg, alt_m });
    (ecef_m[0], ecef_m[1], ecef_m[2])
}

pub fn ecef_to_geodetic(x_m: f64, y_m: f64, z_m: f64) -> (f64, f64, f64) {
    let geodetic = reference_ecef_to_geodetic_point([x_m, y_m, z_m]);
    (geodetic.lat_deg, geodetic.lon_deg, geodetic.alt_m)
}

pub fn ecef_to_enu(
    x_m: f64,
    y_m: f64,
    z_m: f64,
    reference_lat_deg: f64,
    reference_lon_deg: f64,
    reference_alt_m: f64,
) -> (f64, f64, f64) {
    let enu_m = reference_ecef_to_enu_m(
        [x_m, y_m, z_m],
        GeodeticPoint {
            lat_deg: reference_lat_deg,
            lon_deg: reference_lon_deg,
            alt_m: reference_alt_m,
        },
    );
    (enu_m.east_m, enu_m.north_m, enu_m.up_m)
}

pub fn enu_to_ecef(
    base_ecef_m: (f64, f64, f64),
    reference_lat_deg: f64,
    reference_lon_deg: f64,
    reference_alt_m: f64,
    enu_m: (f64, f64, f64),
) -> (f64, f64, f64) {
    let ecef_m = reference_enu_to_ecef_m(
        [base_ecef_m.0, base_ecef_m.1, base_ecef_m.2],
        GeodeticPoint {
            lat_deg: reference_lat_deg,
            lon_deg: reference_lon_deg,
            alt_m: reference_alt_m,
        },
        EnuVector { east_m: enu_m.0, north_m: enu_m.1, up_m: enu_m.2 },
    );
    (ecef_m[0], ecef_m[1], ecef_m[2])
}

pub fn elevation_azimuth_deg(
    receiver_ecef_m: (f64, f64, f64),
    satellite_ecef_m: (f64, f64, f64),
) -> (f64, f64) {
    reference_elevation_azimuth_deg(
        [receiver_ecef_m.0, receiver_ecef_m.1, receiver_ecef_m.2],
        [satellite_ecef_m.0, satellite_ecef_m.1, satellite_ecef_m.2],
    )
}
