#![allow(missing_docs)]

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BroadcastOrbitConstants {
    pub gravitational_parameter_m3_s2: f64,
    pub earth_rotation_rate_rad_s: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BroadcastKeplerianOrbit {
    pub toe_s: f64,
    pub sqrt_a_m: f64,
    pub eccentricity: f64,
    pub inclination_rad: f64,
    pub inclination_rate_rad_s: f64,
    pub right_ascension_rad: f64,
    pub right_ascension_rate_rad_s: f64,
    pub argument_of_perigee_rad: f64,
    pub mean_anomaly_rad: f64,
    pub mean_motion_delta_rad_s: f64,
    pub latitude_cosine_correction_rad: f64,
    pub latitude_sine_correction_rad: f64,
    pub radius_cosine_correction_m: f64,
    pub radius_sine_correction_m: f64,
    pub inclination_cosine_correction_rad: f64,
    pub inclination_sine_correction_rad: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct KeplerSolution {
    pub eccentric_anomaly_rad: f64,
    pub sin_eccentric_anomaly: f64,
    pub cos_eccentric_anomaly: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BroadcastEarthRotationCorrection {
    pub signal_travel_time_s: f64,
    pub rotation_rad: f64,
    pub delta_x_m: f64,
    pub delta_y_m: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BroadcastOrbitState {
    pub x_m: f64,
    pub y_m: f64,
    pub z_m: f64,
    pub time_from_ephemeris_s: f64,
    pub eccentric_anomaly_rad: f64,
    pub sin_eccentric_anomaly: f64,
    pub cos_eccentric_anomaly: f64,
}

pub fn propagate_broadcast_orbit(
    orbit: BroadcastKeplerianOrbit,
    transmit_time_s: f64,
    signal_travel_time_s: f64,
    constants: BroadcastOrbitConstants,
) -> BroadcastOrbitState {
    let semi_major_axis_m = orbit.sqrt_a_m * orbit.sqrt_a_m;
    let nominal_mean_motion_rad_s =
        (constants.gravitational_parameter_m3_s2 / semi_major_axis_m.powi(3)).sqrt();
    let corrected_mean_motion_rad_s = nominal_mean_motion_rad_s + orbit.mean_motion_delta_rad_s;
    let time_from_ephemeris_s = wrap_gnss_week_seconds(transmit_time_s - orbit.toe_s);
    let mean_anomaly_rad =
        orbit.mean_anomaly_rad + corrected_mean_motion_rad_s * time_from_ephemeris_s;
    let kepler = solve_kepler(mean_anomaly_rad, orbit.eccentricity);
    let true_anomaly_rad = ((1.0 - orbit.eccentricity * orbit.eccentricity).sqrt()
        * kepler.sin_eccentric_anomaly)
        .atan2(kepler.cos_eccentric_anomaly - orbit.eccentricity);
    let argument_of_latitude_rad = true_anomaly_rad + orbit.argument_of_perigee_rad;
    let sin_double_latitude = (2.0 * argument_of_latitude_rad).sin();
    let cos_double_latitude = (2.0 * argument_of_latitude_rad).cos();
    let corrected_argument_of_latitude_rad = argument_of_latitude_rad
        + orbit.latitude_cosine_correction_rad * cos_double_latitude
        + orbit.latitude_sine_correction_rad * sin_double_latitude;
    let corrected_radius_m = semi_major_axis_m
        * (1.0 - orbit.eccentricity * kepler.cos_eccentric_anomaly)
        + orbit.radius_cosine_correction_m * cos_double_latitude
        + orbit.radius_sine_correction_m * sin_double_latitude;
    let corrected_inclination_rad = orbit.inclination_rad
        + orbit.inclination_rate_rad_s * time_from_ephemeris_s
        + orbit.inclination_cosine_correction_rad * cos_double_latitude
        + orbit.inclination_sine_correction_rad * sin_double_latitude;
    let orbital_x_m = corrected_radius_m * corrected_argument_of_latitude_rad.cos();
    let orbital_y_m = corrected_radius_m * corrected_argument_of_latitude_rad.sin();
    let corrected_right_ascension_rad = orbit.right_ascension_rad
        + (orbit.right_ascension_rate_rad_s - constants.earth_rotation_rate_rad_s)
            * time_from_ephemeris_s
        - constants.earth_rotation_rate_rad_s * orbit.toe_s;
    let x_unrotated_m = orbital_x_m * corrected_right_ascension_rad.cos()
        - orbital_y_m * corrected_inclination_rad.cos() * corrected_right_ascension_rad.sin();
    let y_unrotated_m = orbital_x_m * corrected_right_ascension_rad.sin()
        + orbital_y_m * corrected_inclination_rad.cos() * corrected_right_ascension_rad.cos();
    let z_m = orbital_y_m * corrected_inclination_rad.sin();
    let earth_rotation = earth_rotation_correction(
        x_unrotated_m,
        y_unrotated_m,
        signal_travel_time_s,
        constants.earth_rotation_rate_rad_s,
    );

    BroadcastOrbitState {
        x_m: x_unrotated_m + earth_rotation.delta_x_m,
        y_m: y_unrotated_m + earth_rotation.delta_y_m,
        z_m,
        time_from_ephemeris_s,
        eccentric_anomaly_rad: kepler.eccentric_anomaly_rad,
        sin_eccentric_anomaly: kepler.sin_eccentric_anomaly,
        cos_eccentric_anomaly: kepler.cos_eccentric_anomaly,
    }
}

pub fn solve_kepler(mean_anomaly_rad: f64, eccentricity: f64) -> KeplerSolution {
    let mut eccentric_anomaly_rad = mean_anomaly_rad;
    for _ in 0..12 {
        let residual =
            eccentric_anomaly_rad - eccentricity * eccentric_anomaly_rad.sin() - mean_anomaly_rad;
        let derivative = 1.0 - eccentricity * eccentric_anomaly_rad.cos();
        let step = residual / derivative;
        eccentric_anomaly_rad -= step;
        if step.abs() < 1e-12 {
            break;
        }
    }
    if !eccentric_anomaly_rad.is_finite() {
        eccentric_anomaly_rad = mean_anomaly_rad;
    }
    KeplerSolution {
        eccentric_anomaly_rad,
        sin_eccentric_anomaly: eccentric_anomaly_rad.sin(),
        cos_eccentric_anomaly: eccentric_anomaly_rad.cos(),
    }
}

pub fn earth_rotation_correction(
    x_m: f64,
    y_m: f64,
    signal_travel_time_s: f64,
    earth_rotation_rate_rad_s: f64,
) -> BroadcastEarthRotationCorrection {
    let rotation_rad = earth_rotation_rate_rad_s * signal_travel_time_s;
    let (x_rotated_m, y_rotated_m) = if rotation_rad.abs() > 0.0 {
        let cos_rot = rotation_rad.cos();
        let sin_rot = rotation_rad.sin();
        (cos_rot * x_m + sin_rot * y_m, -sin_rot * x_m + cos_rot * y_m)
    } else {
        (x_m, y_m)
    };
    BroadcastEarthRotationCorrection {
        signal_travel_time_s,
        rotation_rad,
        delta_x_m: x_rotated_m - x_m,
        delta_y_m: y_rotated_m - y_m,
    }
}

pub fn wrap_gnss_week_seconds(mut seconds: f64) -> f64 {
    while seconds > 302_400.0 {
        seconds -= 604_800.0;
    }
    while seconds < -302_400.0 {
        seconds += 604_800.0;
    }
    seconds
}

#[cfg(test)]
mod tests {
    use super::{
        earth_rotation_correction, propagate_broadcast_orbit, solve_kepler, wrap_gnss_week_seconds,
        BroadcastKeplerianOrbit, BroadcastOrbitConstants,
    };

    #[test]
    fn kepler_solver_returns_circular_mean_anomaly() {
        let solution = solve_kepler(1.0, 0.0);

        assert!((solution.eccentric_anomaly_rad - 1.0).abs() < 1.0e-12);
        assert!((solution.sin_eccentric_anomaly - 1.0_f64.sin()).abs() < 1.0e-12);
        assert!((solution.cos_eccentric_anomaly - 1.0_f64.cos()).abs() < 1.0e-12);
    }

    #[test]
    fn week_seconds_wraps_to_nearest_epoch_interval() {
        assert!((wrap_gnss_week_seconds(302_401.0) + 302_399.0).abs() < 1.0e-12);
        assert!((wrap_gnss_week_seconds(-302_401.0) - 302_399.0).abs() < 1.0e-12);
    }

    #[test]
    fn earth_rotation_correction_matches_rotation_matrix() {
        let correction = earth_rotation_correction(20_200_000.0, 1_500_000.0, 0.07, 7.292_115e-5);
        let angle: f64 = 7.292_115e-5 * 0.07;
        let expected_x = angle.cos() * 20_200_000.0 + angle.sin() * 1_500_000.0;
        let expected_y = -angle.sin() * 20_200_000.0 + angle.cos() * 1_500_000.0;

        assert!((correction.delta_x_m - (expected_x - 20_200_000.0)).abs() < 1.0e-9);
        assert!((correction.delta_y_m - (expected_y - 1_500_000.0)).abs() < 1.0e-9);
    }

    #[test]
    fn broadcast_orbit_propagation_returns_finite_rotated_state() {
        let orbit = BroadcastKeplerianOrbit {
            toe_s: 100_000.0,
            sqrt_a_m: 5_153.795_477_5,
            eccentricity: 0.01,
            inclination_rad: 0.94,
            inclination_rate_rad_s: 1.0e-10,
            right_ascension_rad: 1.0,
            right_ascension_rate_rad_s: -8.0e-9,
            argument_of_perigee_rad: 0.5,
            mean_anomaly_rad: 0.2,
            mean_motion_delta_rad_s: 4.0e-9,
            latitude_cosine_correction_rad: 1.0e-6,
            latitude_sine_correction_rad: -2.0e-6,
            radius_cosine_correction_m: 100.0,
            radius_sine_correction_m: -50.0,
            inclination_cosine_correction_rad: 3.0e-7,
            inclination_sine_correction_rad: -4.0e-7,
        };
        let constants = BroadcastOrbitConstants {
            gravitational_parameter_m3_s2: 3.986_005e14,
            earth_rotation_rate_rad_s: 7.292_115_146_7e-5,
        };

        let state = propagate_broadcast_orbit(orbit, 101_000.0, 0.07, constants);

        assert!(state.x_m.is_finite());
        assert!(state.y_m.is_finite());
        assert!(state.z_m.is_finite());
        assert!(state.sin_eccentric_anomaly.abs() <= 1.0);
        assert!(state.cos_eccentric_anomaly.abs() <= 1.0);
    }
}
