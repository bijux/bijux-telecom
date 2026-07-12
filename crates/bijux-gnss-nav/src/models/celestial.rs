//! Approximate celestial body positions for station and satellite modeling.

use bijux_gnss_core::api::{gps_to_utc, GpsTime, LeapSeconds};

const ASTRONOMICAL_UNIT_M: f64 = 149_597_870_700.0;
const IERS_EARTH_EQUATORIAL_RADIUS_M: f64 = 6_378_136.6;
const MEAN_LUNAR_SEMI_MAJOR_AXIS_EARTH_RADII: f64 = 60.2666;
const MOON_ORBIT_ECCENTRICITY: f64 = 0.0549;

/// Approximates the Sun position in ECEF meters.
pub(crate) fn approximate_sun_position_ecef_m(gps_time: GpsTime) -> [f64; 3] {
    let julian_day = julian_day_utc(gps_time);
    let centuries = (julian_day - 2_451_545.0) / 36_525.0;
    let mean_longitude_deg = wrap_degrees(280.460 + 36_000.770 * centuries);
    let mean_anomaly_deg = wrap_degrees(357.528 + 35_999.050 * centuries);
    let mean_anomaly_rad = mean_anomaly_deg.to_radians();
    let ecliptic_longitude_deg = mean_longitude_deg
        + 1.915 * mean_anomaly_rad.sin()
        + 0.020 * (2.0 * mean_anomaly_rad).sin();
    let ecliptic_longitude_rad = ecliptic_longitude_deg.to_radians();
    let obliquity_rad = (23.4393 - 0.0130 * centuries).to_radians();

    let sun_eci_m = [
        ASTRONOMICAL_UNIT_M * ecliptic_longitude_rad.cos(),
        ASTRONOMICAL_UNIT_M * obliquity_rad.cos() * ecliptic_longitude_rad.sin(),
        ASTRONOMICAL_UNIT_M * obliquity_rad.sin() * ecliptic_longitude_rad.sin(),
    ];

    rotate_eci_to_ecef_m(sun_eci_m, julian_day)
}

/// Approximates the Moon position in ECEF meters.
pub(crate) fn approximate_moon_position_ecef_m(gps_time: GpsTime) -> [f64; 3] {
    let julian_day = julian_day_utc(gps_time);
    let days_since_j2000 = julian_day - 2_451_543.5;
    let ascending_node_deg = wrap_degrees(125.1228 - 0.052_953_808_3 * days_since_j2000);
    let inclination_rad = 5.1454_f64.to_radians();
    let argument_of_perigee_deg = wrap_degrees(318.0634 + 0.164_357_322_3 * days_since_j2000);
    let mean_anomaly_deg = wrap_degrees(115.3654 + 13.064_992_950_9 * days_since_j2000);
    let mean_longitude_deg =
        wrap_degrees(ascending_node_deg + argument_of_perigee_deg + mean_anomaly_deg);
    let (solar_mean_anomaly_deg, solar_mean_longitude_deg) =
        approximate_solar_orbital_elements_deg(days_since_j2000);
    let mean_elongation_deg = wrap_degrees(mean_longitude_deg - solar_mean_longitude_deg);
    let argument_of_latitude_deg = wrap_degrees(mean_longitude_deg - ascending_node_deg);

    let mean_anomaly_rad = mean_anomaly_deg.to_radians();
    let eccentric_anomaly_rad = mean_anomaly_rad
        + MOON_ORBIT_ECCENTRICITY
            * mean_anomaly_rad.sin()
            * (1.0 + MOON_ORBIT_ECCENTRICITY * mean_anomaly_rad.cos());
    let x_orbit = MEAN_LUNAR_SEMI_MAJOR_AXIS_EARTH_RADII
        * (eccentric_anomaly_rad.cos() - MOON_ORBIT_ECCENTRICITY);
    let y_orbit = MEAN_LUNAR_SEMI_MAJOR_AXIS_EARTH_RADII
        * (1.0 - MOON_ORBIT_ECCENTRICITY * MOON_ORBIT_ECCENTRICITY).sqrt()
        * eccentric_anomaly_rad.sin();
    let true_anomaly_rad = y_orbit.atan2(x_orbit);
    let mut radius_earth_radii = (x_orbit * x_orbit + y_orbit * y_orbit).sqrt();
    let mut ecliptic_longitude_rad =
        (true_anomaly_rad + argument_of_perigee_deg.to_radians()).rem_euclid(std::f64::consts::TAU);
    let mut ecliptic_latitude_rad = 0.0_f64;

    apply_lunar_perturbations(
        &mut ecliptic_longitude_rad,
        &mut ecliptic_latitude_rad,
        &mut radius_earth_radii,
        mean_anomaly_deg,
        solar_mean_anomaly_deg,
        mean_elongation_deg,
        argument_of_latitude_deg,
    );

    let ascending_node_rad = ascending_node_deg.to_radians();
    let argument_from_node_rad = ecliptic_longitude_rad - ascending_node_rad;
    let sin_argument = argument_from_node_rad.sin();
    let cos_argument = argument_from_node_rad.cos();
    let sin_node = ascending_node_rad.sin();
    let cos_node = ascending_node_rad.cos();
    let sin_inclination = inclination_rad.sin();
    let cos_inclination = inclination_rad.cos();
    let cos_latitude = ecliptic_latitude_rad.cos();
    let sin_latitude = ecliptic_latitude_rad.sin();
    let radius_m = radius_earth_radii * IERS_EARTH_EQUATORIAL_RADIUS_M;

    let moon_ecliptic_m = [
        radius_m
            * (cos_node * cos_argument - sin_node * sin_argument * cos_inclination)
            * cos_latitude,
        radius_m
            * (sin_node * cos_argument + cos_node * sin_argument * cos_inclination)
            * cos_latitude,
        radius_m * sin_argument * sin_inclination * cos_latitude + radius_m * sin_latitude,
    ];
    let obliquity_rad = (23.4393 - 3.563e-7 * days_since_j2000).to_radians();
    let moon_eci_m = [
        moon_ecliptic_m[0],
        moon_ecliptic_m[1] * obliquity_rad.cos() - moon_ecliptic_m[2] * obliquity_rad.sin(),
        moon_ecliptic_m[1] * obliquity_rad.sin() + moon_ecliptic_m[2] * obliquity_rad.cos(),
    ];

    rotate_eci_to_ecef_m(moon_eci_m, julian_day)
}

fn julian_day_utc(gps_time: GpsTime) -> f64 {
    let utc = gps_to_utc(gps_time, &LeapSeconds::default_table());
    utc.unix_s / 86_400.0 + 2_440_587.5
}

fn approximate_solar_orbital_elements_deg(days_since_j2000: f64) -> (f64, f64) {
    let mean_anomaly_deg = wrap_degrees(356.0470 + 0.985_600_258_5 * days_since_j2000);
    let perihelion_longitude_deg = wrap_degrees(282.9404 + 4.709_35e-5 * days_since_j2000);
    let mean_longitude_deg = wrap_degrees(mean_anomaly_deg + perihelion_longitude_deg);
    (mean_anomaly_deg, mean_longitude_deg)
}

fn apply_lunar_perturbations(
    ecliptic_longitude_rad: &mut f64,
    ecliptic_latitude_rad: &mut f64,
    radius_earth_radii: &mut f64,
    lunar_mean_anomaly_deg: f64,
    solar_mean_anomaly_deg: f64,
    mean_elongation_deg: f64,
    argument_of_latitude_deg: f64,
) {
    let lunar_mean_anomaly_rad = lunar_mean_anomaly_deg.to_radians();
    let solar_mean_anomaly_rad = solar_mean_anomaly_deg.to_radians();
    let mean_elongation_rad = mean_elongation_deg.to_radians();
    let argument_of_latitude_rad = argument_of_latitude_deg.to_radians();

    let longitude_correction_deg = -1.274 * (lunar_mean_anomaly_rad - 2.0 * mean_elongation_rad).sin()
        + 0.658 * (2.0 * mean_elongation_rad).sin()
        - 0.186 * solar_mean_anomaly_rad.sin()
        - 0.059 * (2.0 * lunar_mean_anomaly_rad - 2.0 * mean_elongation_rad).sin()
        - 0.057
            * (lunar_mean_anomaly_rad - 2.0 * mean_elongation_rad + solar_mean_anomaly_rad).sin()
        + 0.053 * (lunar_mean_anomaly_rad + 2.0 * mean_elongation_rad).sin()
        + 0.046 * (2.0 * mean_elongation_rad - solar_mean_anomaly_rad).sin()
        + 0.041 * (lunar_mean_anomaly_rad - solar_mean_anomaly_rad).sin()
        - 0.035 * mean_elongation_rad.sin()
        - 0.031 * (lunar_mean_anomaly_rad + solar_mean_anomaly_rad).sin()
        - 0.015 * (2.0 * argument_of_latitude_rad - 2.0 * mean_elongation_rad).sin()
        + 0.011 * (lunar_mean_anomaly_rad - 4.0 * mean_elongation_rad).sin();
    let latitude_correction_deg = -0.173 * (argument_of_latitude_rad - 2.0 * mean_elongation_rad).sin()
        - 0.055
            * (lunar_mean_anomaly_rad - argument_of_latitude_rad - 2.0 * mean_elongation_rad).sin()
        - 0.046
            * (lunar_mean_anomaly_rad + argument_of_latitude_rad - 2.0 * mean_elongation_rad).sin()
        + 0.033 * (argument_of_latitude_rad + 2.0 * mean_elongation_rad).sin()
        + 0.017 * (2.0 * lunar_mean_anomaly_rad + argument_of_latitude_rad).sin();
    let radius_correction_earth_radii = -0.58 * (lunar_mean_anomaly_rad - 2.0 * mean_elongation_rad).cos()
        - 0.46 * (2.0 * mean_elongation_rad).cos();

    *ecliptic_longitude_rad =
        (*ecliptic_longitude_rad + longitude_correction_deg.to_radians()).rem_euclid(std::f64::consts::TAU);
    *ecliptic_latitude_rad += latitude_correction_deg.to_radians();
    *radius_earth_radii += radius_correction_earth_radii;
}

fn rotate_eci_to_ecef_m(eci_m: [f64; 3], julian_day: f64) -> [f64; 3] {
    let gmst_deg =
        wrap_degrees(280.460_618_37 + 360.985_647_366_29 * (julian_day - 2_451_545.0));
    let gmst_rad = gmst_deg.to_radians();
    let cos_theta = gmst_rad.cos();
    let sin_theta = gmst_rad.sin();

    [
        cos_theta * eci_m[0] + sin_theta * eci_m[1],
        -sin_theta * eci_m[0] + cos_theta * eci_m[1],
        eci_m[2],
    ]
}

fn wrap_degrees(angle_deg: f64) -> f64 {
    angle_deg.rem_euclid(360.0)
}

#[cfg(test)]
mod tests {
    use bijux_gnss_core::api::GpsTime;

    use super::{approximate_moon_position_ecef_m, approximate_sun_position_ecef_m};

    fn norm_m(vector: [f64; 3]) -> f64 {
        (vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]).sqrt()
    }

    #[test]
    fn approximate_sun_position_stays_near_one_astronomical_unit() {
        let sun_pos_m = approximate_sun_position_ecef_m(GpsTime { week: 2200, tow_s: 43_200.0 });

        assert!(sun_pos_m.into_iter().all(f64::is_finite));
        let sun_distance_m = norm_m(sun_pos_m);
        assert!(sun_distance_m > 1.45e11);
        assert!(sun_distance_m < 1.52e11);
    }

    #[test]
    fn approximate_moon_position_stays_near_expected_orbit_radius() {
        let moon_pos_m = approximate_moon_position_ecef_m(GpsTime { week: 2200, tow_s: 43_200.0 });

        assert!(moon_pos_m.into_iter().all(f64::is_finite));
        let moon_distance_m = norm_m(moon_pos_m);
        assert!(moon_distance_m > 3.4e8);
        assert!(moon_distance_m < 4.1e8);
    }

    #[test]
    fn approximate_moon_position_moves_substantially_over_hours() {
        let start = approximate_moon_position_ecef_m(GpsTime { week: 2200, tow_s: 0.0 });
        let later = approximate_moon_position_ecef_m(GpsTime { week: 2200, tow_s: 21_600.0 });
        let delta_m = norm_m([later[0] - start[0], later[1] - start[1], later[2] - start[2]]);

        assert!(delta_m > 1.0e7);
    }
}
