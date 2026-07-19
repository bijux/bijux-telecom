//! Independent WGS-84 coordinate utilities for scientific test expectations.

const WGS84_SEMI_MAJOR_AXIS_M: f64 = 6_378_137.0;
const WGS84_FLATTENING: f64 = 1.0 / 298.257_223_563;
const WGS84_SEMI_MINOR_AXIS_M: f64 = WGS84_SEMI_MAJOR_AXIS_M * (1.0 - WGS84_FLATTENING);
const WGS84_FIRST_ECCENTRICITY_SQUARED: f64 = WGS84_FLATTENING * (2.0 - WGS84_FLATTENING);
const WGS84_SECOND_ECCENTRICITY_SQUARED: f64 = (WGS84_SEMI_MAJOR_AXIS_M * WGS84_SEMI_MAJOR_AXIS_M
    - WGS84_SEMI_MINOR_AXIS_M * WGS84_SEMI_MINOR_AXIS_M)
    / (WGS84_SEMI_MINOR_AXIS_M * WGS84_SEMI_MINOR_AXIS_M);

#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct GeodeticPoint {
    pub lat_deg: f64,
    pub lon_deg: f64,
    pub alt_m: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct EnuVector {
    pub east_m: f64,
    pub north_m: f64,
    pub up_m: f64,
}

pub(crate) fn geodetic_to_ecef_m(geodetic: GeodeticPoint) -> [f64; 3] {
    let latitude_rad = geodetic.lat_deg.to_radians();
    let longitude_rad = geodetic.lon_deg.to_radians();
    let sin_latitude = latitude_rad.sin();
    let cos_latitude = latitude_rad.cos();
    let sin_longitude = longitude_rad.sin();
    let cos_longitude = longitude_rad.cos();
    let prime_vertical_radius_m = WGS84_SEMI_MAJOR_AXIS_M
        / (1.0 - WGS84_FIRST_ECCENTRICITY_SQUARED * sin_latitude * sin_latitude).sqrt();

    [
        (prime_vertical_radius_m + geodetic.alt_m) * cos_latitude * cos_longitude,
        (prime_vertical_radius_m + geodetic.alt_m) * cos_latitude * sin_longitude,
        (prime_vertical_radius_m * (1.0 - WGS84_FIRST_ECCENTRICITY_SQUARED) + geodetic.alt_m)
            * sin_latitude,
    ]
}

pub(crate) fn ecef_to_geodetic_point(ecef_m: [f64; 3]) -> GeodeticPoint {
    let x_m = ecef_m[0];
    let y_m = ecef_m[1];
    let z_m = ecef_m[2];
    let longitude_rad = y_m.atan2(x_m);
    let horizontal_radius_m = (x_m * x_m + y_m * y_m).sqrt().max(f64::EPSILON);
    let parametric_latitude_rad =
        (z_m * WGS84_SEMI_MAJOR_AXIS_M).atan2(horizontal_radius_m * WGS84_SEMI_MINOR_AXIS_M);
    let sin_parametric = parametric_latitude_rad.sin();
    let cos_parametric = parametric_latitude_rad.cos();
    let latitude_rad = (z_m
        + WGS84_SECOND_ECCENTRICITY_SQUARED * WGS84_SEMI_MINOR_AXIS_M * sin_parametric.powi(3))
    .atan2(
        horizontal_radius_m
            - WGS84_FIRST_ECCENTRICITY_SQUARED * WGS84_SEMI_MAJOR_AXIS_M * cos_parametric.powi(3),
    );
    let sin_latitude = latitude_rad.sin();
    let prime_vertical_radius_m = WGS84_SEMI_MAJOR_AXIS_M
        / (1.0 - WGS84_FIRST_ECCENTRICITY_SQUARED * sin_latitude * sin_latitude).sqrt();
    let altitude_m = horizontal_radius_m / latitude_rad.cos() - prime_vertical_radius_m;

    GeodeticPoint {
        lat_deg: latitude_rad.to_degrees(),
        lon_deg: longitude_rad.to_degrees(),
        alt_m: altitude_m,
    }
}

pub(crate) fn ecef_to_enu_m(
    target_ecef_m: [f64; 3],
    reference_geodetic: GeodeticPoint,
) -> EnuVector {
    let reference_ecef_m = geodetic_to_ecef_m(reference_geodetic);
    let delta_m = subtract3(target_ecef_m, reference_ecef_m);
    let latitude_rad = reference_geodetic.lat_deg.to_radians();
    let longitude_rad = reference_geodetic.lon_deg.to_radians();
    let sin_latitude = latitude_rad.sin();
    let cos_latitude = latitude_rad.cos();
    let sin_longitude = longitude_rad.sin();
    let cos_longitude = longitude_rad.cos();

    EnuVector {
        east_m: -sin_longitude * delta_m[0] + cos_longitude * delta_m[1],
        north_m: -sin_latitude * cos_longitude * delta_m[0]
            - sin_latitude * sin_longitude * delta_m[1]
            + cos_latitude * delta_m[2],
        up_m: cos_latitude * cos_longitude * delta_m[0]
            + cos_latitude * sin_longitude * delta_m[1]
            + sin_latitude * delta_m[2],
    }
}

pub(crate) fn enu_to_ecef_m(
    reference_ecef_m: [f64; 3],
    reference_geodetic: GeodeticPoint,
    enu_m: EnuVector,
) -> [f64; 3] {
    let latitude_rad = reference_geodetic.lat_deg.to_radians();
    let longitude_rad = reference_geodetic.lon_deg.to_radians();
    let sin_latitude = latitude_rad.sin();
    let cos_latitude = latitude_rad.cos();
    let sin_longitude = longitude_rad.sin();
    let cos_longitude = longitude_rad.cos();

    [
        reference_ecef_m[0]
            - sin_longitude * enu_m.east_m
            - sin_latitude * cos_longitude * enu_m.north_m
            + cos_latitude * cos_longitude * enu_m.up_m,
        reference_ecef_m[1] + cos_longitude * enu_m.east_m
            - sin_latitude * sin_longitude * enu_m.north_m
            + cos_latitude * sin_longitude * enu_m.up_m,
        reference_ecef_m[2] + cos_latitude * enu_m.north_m + sin_latitude * enu_m.up_m,
    ]
}

pub(crate) fn elevation_azimuth_deg(
    receiver_ecef_m: [f64; 3],
    satellite_ecef_m: [f64; 3],
) -> (f64, f64) {
    let receiver_geodetic = ecef_to_geodetic_point(receiver_ecef_m);
    let local = ecef_to_enu_m(satellite_ecef_m, receiver_geodetic);
    let azimuth_deg = local.east_m.atan2(local.north_m).to_degrees().rem_euclid(360.0);
    let elevation_deg =
        (local.up_m / norm3([local.east_m, local.north_m, local.up_m])).asin().to_degrees();
    (azimuth_deg, elevation_deg)
}

pub(crate) fn dot3(left: [f64; 3], right: [f64; 3]) -> f64 {
    left[0] * right[0] + left[1] * right[1] + left[2] * right[2]
}

pub(crate) fn cross3(left: [f64; 3], right: [f64; 3]) -> [f64; 3] {
    [
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    ]
}

pub(crate) fn norm3(vector: [f64; 3]) -> f64 {
    dot3(vector, vector).sqrt().max(f64::EPSILON)
}

pub(crate) fn normalize3(vector: [f64; 3]) -> [f64; 3] {
    let magnitude = norm3(vector);
    [vector[0] / magnitude, vector[1] / magnitude, vector[2] / magnitude]
}

pub(crate) fn subtract3(left: [f64; 3], right: [f64; 3]) -> [f64; 3] {
    [left[0] - right[0], left[1] - right[1], left[2] - right[2]]
}

#[cfg(test)]
mod tests {
    use super::{
        ecef_to_enu_m, ecef_to_geodetic_point, elevation_azimuth_deg, enu_to_ecef_m,
        geodetic_to_ecef_m, EnuVector, GeodeticPoint,
    };

    #[test]
    fn geodetic_to_ecef_matches_equatorial_reference_point() {
        let ecef_m = geodetic_to_ecef_m(GeodeticPoint { lat_deg: 0.0, lon_deg: 0.0, alt_m: 0.0 });

        assert!((ecef_m[0] - 6_378_137.0).abs() < 1.0e-6);
        assert!(ecef_m[1].abs() < 1.0e-9);
        assert!(ecef_m[2].abs() < 1.0e-9);
    }

    #[test]
    fn ecef_to_geodetic_round_trips_known_station() {
        let original =
            GeodeticPoint { lat_deg: 58.19884205, lon_deg: -136.64080781, alt_m: 26.9246 };
        let ecef_m = geodetic_to_ecef_m(original);
        let recovered = ecef_to_geodetic_point(ecef_m);

        assert!((recovered.lat_deg - original.lat_deg).abs() < 1.0e-8);
        assert!((recovered.lon_deg - original.lon_deg).abs() < 1.0e-8);
        assert!((recovered.alt_m - original.alt_m).abs() < 1.0e-4);
    }

    #[test]
    fn enu_round_trip_preserves_reference_offset() {
        let reference = GeodeticPoint { lat_deg: 37.0, lon_deg: -122.0, alt_m: 10.0 };
        let reference_ecef_m = geodetic_to_ecef_m(reference);
        let offset = EnuVector { east_m: 8.5, north_m: -4.25, up_m: 1.75 };
        let shifted_ecef_m = enu_to_ecef_m(reference_ecef_m, reference, offset);
        let recovered = ecef_to_enu_m(shifted_ecef_m, reference);

        assert!((recovered.east_m - offset.east_m).abs() < 1.0e-9);
        assert!((recovered.north_m - offset.north_m).abs() < 1.0e-9);
        assert!((recovered.up_m - offset.up_m).abs() < 1.0e-9);
    }

    #[test]
    fn elevation_and_azimuth_match_due_east_horizon_case() {
        let receiver = GeodeticPoint { lat_deg: 0.0, lon_deg: 0.0, alt_m: 0.0 };
        let receiver_ecef_m = geodetic_to_ecef_m(receiver);
        let satellite_ecef_m = enu_to_ecef_m(
            receiver_ecef_m,
            receiver,
            EnuVector { east_m: 10_000.0, north_m: 0.0, up_m: 0.0 },
        );

        let (azimuth_deg, elevation_deg) = elevation_azimuth_deg(receiver_ecef_m, satellite_ecef_m);

        assert!((azimuth_deg - 90.0).abs() < 1.0e-9);
        assert!(elevation_deg.abs() < 1.0e-9);
    }
}
