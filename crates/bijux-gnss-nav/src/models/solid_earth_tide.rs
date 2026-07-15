//! Solid Earth tide displacement model.

use bijux_gnss_core::api::GpsTime;
use serde::{Deserialize, Serialize};

use crate::models::celestial::{approximate_moon_position_ecef_m, approximate_sun_position_ecef_m};

const EARTH_GRAVITATIONAL_PARAMETER_M3PS2: f64 = 3.986_004_418e14;
const IERS_EARTH_EQUATORIAL_RADIUS_M: f64 = 6_378_136.6;
const MOON_GRAVITATIONAL_PARAMETER_M3PS2: f64 = 4.902_801e12;
const SUN_GRAVITATIONAL_PARAMETER_M3PS2: f64 = 1.327_124_420_99e20;

const DEGREE_TWO_LOVE_NUMBER: f64 = 0.6078;
const DEGREE_TWO_SHIDA_NUMBER: f64 = 0.0847;
const DEGREE_TWO_LATITUDE_LOVE_OFFSET: f64 = -0.0006;
const DEGREE_TWO_LATITUDE_SHIDA_OFFSET: f64 = 0.0002;
const DEGREE_THREE_LOVE_NUMBER: f64 = 0.292;
const DEGREE_THREE_SHIDA_NUMBER: f64 = 0.015;

const DIURNAL_L1_SHIDA_NUMBER: f64 = 0.0012;
const SEMIDIURNAL_L1_SHIDA_NUMBER: f64 = 0.0024;
const DIURNAL_IMAGINARY_LOVE_NUMBER: f64 = -0.0025;
const DIURNAL_IMAGINARY_SHIDA_NUMBER: f64 = -0.0007;
const SEMIDIURNAL_IMAGINARY_LOVE_NUMBER: f64 = -0.0022;
const SEMIDIURNAL_IMAGINARY_SHIDA_NUMBER: f64 = -0.0007;

/// Conventional solid Earth tide model for static monument displacement.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct SolidEarthTideModel;

impl SolidEarthTideModel {
    /// Computes the station displacement in ECEF meters at a GPS epoch.
    pub fn displacement_ecef_m(
        &self,
        receiver_ecef_m: [f64; 3],
        gps_time: GpsTime,
    ) -> Option<[f64; 3]> {
        let station = StationFrame::from_receiver_ecef_m(receiver_ecef_m)?;
        let moon = BodyState::from_ecef_m(
            approximate_moon_position_ecef_m(gps_time),
            MOON_GRAVITATIONAL_PARAMETER_M3PS2,
        )?;
        let sun = BodyState::from_ecef_m(
            approximate_sun_position_ecef_m(gps_time),
            SUN_GRAVITATIONAL_PARAMETER_M3PS2,
        )?;

        let mut displacement_ecef_m = [0.0_f64; 3];
        add_assign3(
            &mut displacement_ecef_m,
            add3(degree_two_in_phase_m(station, moon), degree_two_in_phase_m(station, sun)),
        );
        add_assign3(&mut displacement_ecef_m, degree_three_moon_m(station, moon));
        add_assign3(
            &mut displacement_ecef_m,
            add3(diurnal_l1_transverse_m(station, moon), diurnal_l1_transverse_m(station, sun)),
        );
        add_assign3(
            &mut displacement_ecef_m,
            add3(
                semidiurnal_l1_transverse_m(station, moon),
                semidiurnal_l1_transverse_m(station, sun),
            ),
        );
        add_assign3(
            &mut displacement_ecef_m,
            add3(diurnal_out_of_phase_m(station, moon), diurnal_out_of_phase_m(station, sun)),
        );
        add_assign3(
            &mut displacement_ecef_m,
            add3(
                semidiurnal_out_of_phase_m(station, moon),
                semidiurnal_out_of_phase_m(station, sun),
            ),
        );

        Some(displacement_ecef_m)
    }
}

#[derive(Debug, Clone, Copy)]
struct StationFrame {
    radial_hat: [f64; 3],
    east_hat: [f64; 3],
    north_hat: [f64; 3],
    sin_lat: f64,
    cos_lat: f64,
    lon_rad: f64,
}

impl StationFrame {
    fn from_receiver_ecef_m(receiver_ecef_m: [f64; 3]) -> Option<Self> {
        if !receiver_ecef_m.into_iter().all(f64::is_finite) {
            return None;
        }
        let radius_m = norm3(receiver_ecef_m);
        if !radius_m.is_finite() || radius_m <= 0.0 {
            return None;
        }
        let radial_hat = scale3(receiver_ecef_m, 1.0 / radius_m);
        let sin_lat = radial_hat[2];
        let cos_lat = (1.0 - sin_lat * sin_lat).max(0.0).sqrt();
        let lon_rad = receiver_ecef_m[1].atan2(receiver_ecef_m[0]);
        let (sin_lon, cos_lon) = lon_rad.sin_cos();
        let east_hat = [-sin_lon, cos_lon, 0.0];
        let north_hat = [-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat];

        Some(Self { radial_hat, east_hat, north_hat, sin_lat, cos_lat, lon_rad })
    }

    fn latitude_adjustment(self) -> f64 {
        1.5 * self.sin_lat * self.sin_lat - 0.5
    }

    fn radial_transverse_to_ecef_m(self, radial_m: f64, east_m: f64, north_m: f64) -> [f64; 3] {
        add3(
            scale3(self.radial_hat, radial_m),
            add3(scale3(self.east_hat, east_m), scale3(self.north_hat, north_m)),
        )
    }
}

#[derive(Debug, Clone, Copy)]
struct BodyState {
    scale_degree_two_m: f64,
    scale_degree_three_m: f64,
    unit_ecef: [f64; 3],
    sin_lat: f64,
    cos_lat: f64,
    lon_rad: f64,
}

impl BodyState {
    fn from_ecef_m(vector_ecef_m: [f64; 3], gravitational_parameter_m3ps2: f64) -> Option<Self> {
        if !vector_ecef_m.into_iter().all(f64::is_finite) {
            return None;
        }
        let distance_m = norm3(vector_ecef_m);
        if !distance_m.is_finite() || distance_m <= 0.0 {
            return None;
        }
        let unit_ecef = scale3(vector_ecef_m, 1.0 / distance_m);
        let sin_lat = unit_ecef[2];
        let cos_lat = (1.0 - sin_lat * sin_lat).max(0.0).sqrt();
        Some(Self {
            scale_degree_two_m: gravitational_parameter_m3ps2
                * IERS_EARTH_EQUATORIAL_RADIUS_M.powi(4)
                / (EARTH_GRAVITATIONAL_PARAMETER_M3PS2 * distance_m.powi(3)),
            scale_degree_three_m: gravitational_parameter_m3ps2
                * IERS_EARTH_EQUATORIAL_RADIUS_M.powi(5)
                / (EARTH_GRAVITATIONAL_PARAMETER_M3PS2 * distance_m.powi(4)),
            unit_ecef,
            sin_lat,
            cos_lat,
            lon_rad: vector_ecef_m[1].atan2(vector_ecef_m[0]),
        })
    }
}

fn degree_two_in_phase_m(station: StationFrame, body: BodyState) -> [f64; 3] {
    let love_number =
        DEGREE_TWO_LOVE_NUMBER + DEGREE_TWO_LATITUDE_LOVE_OFFSET * station.latitude_adjustment();
    let shida_number =
        DEGREE_TWO_SHIDA_NUMBER + DEGREE_TWO_LATITUDE_SHIDA_OFFSET * station.latitude_adjustment();
    let dot = dot3(body.unit_ecef, station.radial_hat);
    let tangential_hat = subtract3(body.unit_ecef, scale3(station.radial_hat, dot));
    let radial_m = body.scale_degree_two_m * love_number * (1.5 * dot * dot - 0.5);
    let tangential_m = body.scale_degree_two_m * 3.0 * shida_number * dot;
    add3(scale3(station.radial_hat, radial_m), scale3(tangential_hat, tangential_m))
}

fn degree_three_moon_m(station: StationFrame, body: BodyState) -> [f64; 3] {
    let dot = dot3(body.unit_ecef, station.radial_hat);
    let tangential_hat = subtract3(body.unit_ecef, scale3(station.radial_hat, dot));
    let radial_m =
        body.scale_degree_three_m * DEGREE_THREE_LOVE_NUMBER * (2.5 * dot * dot * dot - 1.5 * dot);
    let tangential_m =
        body.scale_degree_three_m * DEGREE_THREE_SHIDA_NUMBER * (7.5 * dot * dot - 1.5);
    add3(scale3(station.radial_hat, radial_m), scale3(tangential_hat, tangential_m))
}

fn diurnal_l1_transverse_m(station: StationFrame, body: BodyState) -> [f64; 3] {
    let delta_lon_rad = station.lon_rad - body.lon_rad;
    let p21 = 3.0 * body.sin_lat * body.cos_lat;
    let cos_2phi = station.cos_lat * station.cos_lat - station.sin_lat * station.sin_lat;
    let north_m = -DIURNAL_L1_SHIDA_NUMBER
        * station.sin_lat
        * body.scale_degree_two_m
        * p21
        * station.sin_lat
        * delta_lon_rad.cos();
    let east_m = DIURNAL_L1_SHIDA_NUMBER
        * station.sin_lat
        * body.scale_degree_two_m
        * p21
        * cos_2phi
        * delta_lon_rad.sin();
    station.radial_transverse_to_ecef_m(0.0, east_m, north_m)
}

fn semidiurnal_l1_transverse_m(station: StationFrame, body: BodyState) -> [f64; 3] {
    let delta_lon_rad = station.lon_rad - body.lon_rad;
    let p22 = 3.0 * body.cos_lat * body.cos_lat;
    let north_m = -0.5
        * SEMIDIURNAL_L1_SHIDA_NUMBER
        * station.sin_lat
        * station.cos_lat
        * body.scale_degree_two_m
        * p22
        * (2.0 * delta_lon_rad).cos();
    let east_m = -0.5
        * SEMIDIURNAL_L1_SHIDA_NUMBER
        * station.sin_lat
        * station.cos_lat
        * body.scale_degree_two_m
        * p22
        * station.sin_lat
        * (2.0 * delta_lon_rad).sin();
    station.radial_transverse_to_ecef_m(0.0, east_m, north_m)
}

fn diurnal_out_of_phase_m(station: StationFrame, body: BodyState) -> [f64; 3] {
    let delta_lon_rad = station.lon_rad - body.lon_rad;
    let sin_2phi = 2.0 * station.sin_lat * station.cos_lat;
    let cos_2phi = station.cos_lat * station.cos_lat - station.sin_lat * station.sin_lat;
    let sin_2_body_lat = 2.0 * body.sin_lat * body.cos_lat;
    let radial_m = -0.75
        * DIURNAL_IMAGINARY_LOVE_NUMBER
        * body.scale_degree_two_m
        * sin_2_body_lat
        * sin_2phi
        * delta_lon_rad.sin();
    let north_m = -1.5
        * DIURNAL_IMAGINARY_SHIDA_NUMBER
        * body.scale_degree_two_m
        * sin_2_body_lat
        * cos_2phi
        * delta_lon_rad.sin();
    let east_m = -1.5
        * DIURNAL_IMAGINARY_SHIDA_NUMBER
        * body.scale_degree_two_m
        * sin_2_body_lat
        * station.sin_lat
        * delta_lon_rad.cos();
    station.radial_transverse_to_ecef_m(radial_m, east_m, north_m)
}

fn semidiurnal_out_of_phase_m(station: StationFrame, body: BodyState) -> [f64; 3] {
    let delta_lon_rad = station.lon_rad - body.lon_rad;
    let sin_2phi = 2.0 * station.sin_lat * station.cos_lat;
    let cos_phi_squared = station.cos_lat * station.cos_lat;
    let cos_body_lat_squared = body.cos_lat * body.cos_lat;
    let radial_m = -0.75
        * SEMIDIURNAL_IMAGINARY_LOVE_NUMBER
        * body.scale_degree_two_m
        * cos_body_lat_squared
        * cos_phi_squared
        * (2.0 * delta_lon_rad).sin();
    let north_m = 0.75
        * SEMIDIURNAL_IMAGINARY_SHIDA_NUMBER
        * body.scale_degree_two_m
        * cos_body_lat_squared
        * sin_2phi
        * (2.0 * delta_lon_rad).sin();
    let east_m = -1.5
        * SEMIDIURNAL_IMAGINARY_SHIDA_NUMBER
        * body.scale_degree_two_m
        * cos_body_lat_squared
        * station.cos_lat
        * (2.0 * delta_lon_rad).cos();
    station.radial_transverse_to_ecef_m(radial_m, east_m, north_m)
}

fn dot3(left: [f64; 3], right: [f64; 3]) -> f64 {
    left[0] * right[0] + left[1] * right[1] + left[2] * right[2]
}

fn norm3(vector: [f64; 3]) -> f64 {
    (vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]).sqrt()
}

fn scale3(vector: [f64; 3], scalar: f64) -> [f64; 3] {
    [vector[0] * scalar, vector[1] * scalar, vector[2] * scalar]
}

fn subtract3(left: [f64; 3], right: [f64; 3]) -> [f64; 3] {
    [left[0] - right[0], left[1] - right[1], left[2] - right[2]]
}

fn add3(left: [f64; 3], right: [f64; 3]) -> [f64; 3] {
    [left[0] + right[0], left[1] + right[1], left[2] + right[2]]
}

fn add_assign3(target: &mut [f64; 3], delta: [f64; 3]) {
    target[0] += delta[0];
    target[1] += delta[1];
    target[2] += delta[2];
}

#[cfg(test)]
mod tests {
    use bijux_gnss_core::api::GpsTime;

    use super::{
        degree_two_in_phase_m, norm3, BodyState, SolidEarthTideModel, StationFrame,
        DEGREE_TWO_LATITUDE_LOVE_OFFSET, DEGREE_TWO_LOVE_NUMBER, IERS_EARTH_EQUATORIAL_RADIUS_M,
        MOON_GRAVITATIONAL_PARAMETER_M3PS2,
    };
    use crate::estimation::position::solver::geodesy::geodetic_to_ecef;

    #[test]
    fn degree_two_in_phase_matches_aligned_radial_closed_form() {
        let station =
            StationFrame::from_receiver_ecef_m([IERS_EARTH_EQUATORIAL_RADIUS_M, 0.0, 0.0])
                .expect("station frame");
        let body_distance_m = 384_400_000.0;
        let body =
            BodyState::from_ecef_m([body_distance_m, 0.0, 0.0], MOON_GRAVITATIONAL_PARAMETER_M3PS2)
                .expect("body");

        let displacement = degree_two_in_phase_m(station, body);
        let expected_love_number = DEGREE_TWO_LOVE_NUMBER - 0.5 * DEGREE_TWO_LATITUDE_LOVE_OFFSET;
        let expected_x_m = body.scale_degree_two_m * expected_love_number;

        assert!((displacement[0] - expected_x_m).abs() < 1.0e-12);
        assert!(displacement[1].abs() < 1.0e-12);
        assert!(displacement[2].abs() < 1.0e-12);
    }

    #[test]
    fn model_returns_finite_submeter_displacement_for_real_epoch() {
        let receiver_ecef_m = {
            let (x, y, z) = geodetic_to_ecef(47.0, 8.0, 450.0);
            [x, y, z]
        };
        let displacement = SolidEarthTideModel
            .displacement_ecef_m(receiver_ecef_m, GpsTime { week: 2200, tow_s: 43_200.0 })
            .expect("solid Earth tide displacement");

        assert!(displacement.into_iter().all(f64::is_finite));
        let displacement_norm_m = norm3(displacement);
        assert!(
            displacement_norm_m > 0.05,
            "unexpectedly small displacement {displacement_norm_m}"
        );
        assert!(displacement_norm_m < 0.6, "unexpectedly large displacement {displacement_norm_m}");
    }

    #[test]
    fn model_varies_over_hours() {
        let receiver_ecef_m = {
            let (x, y, z) = geodetic_to_ecef(34.0, -118.0, 100.0);
            [x, y, z]
        };
        let model = SolidEarthTideModel;
        let start = model
            .displacement_ecef_m(receiver_ecef_m, GpsTime { week: 2200, tow_s: 0.0 })
            .expect("start displacement");
        let later = model
            .displacement_ecef_m(receiver_ecef_m, GpsTime { week: 2200, tow_s: 21_600.0 })
            .expect("later displacement");
        let delta_m = norm3([later[0] - start[0], later[1] - start[1], later[2] - start[2]]);

        assert!(delta_m > 0.01);
    }

    #[test]
    fn model_rejects_non_physical_receiver_position() {
        assert!(SolidEarthTideModel
            .displacement_ecef_m([0.0, 0.0, 0.0], GpsTime { week: 2200, tow_s: 0.0 })
            .is_none());
    }
}
