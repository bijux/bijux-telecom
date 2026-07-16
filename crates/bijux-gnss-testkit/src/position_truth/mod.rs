//! Independent GPS position-truth scenarios for navigation and receiver validation.
#![allow(missing_docs)]

use bijux_gnss_core::api::Llh;

use crate::independent_models::coordinates::{
    ecef_to_geodetic_point, geodetic_to_ecef_m, GeodeticPoint,
};

mod observation_synthesis;
mod residual_model;
mod scenario_catalog;

const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;
const GPS_L1_CA_CARRIER_HZ_MPS: f64 = 1_575_420_000.0;
const GPS_L1_CA_WAVELENGTH_M: f64 = SPEED_OF_LIGHT_MPS / GPS_L1_CA_CARRIER_HZ_MPS;

pub use observation_synthesis::{
    add_klobuchar_delay_to_observations, add_saastamoinen_delay_to_observations,
    add_satellite_delay_biases_to_observations, add_uniform_delay_bias_to_observations,
    gps_l1ca_doppler_from_truth, gps_l1ca_signal_id, pseudorange_from_truth,
    receiver_clock_bias_with_drift_s, timed_position_observation,
    timed_position_observation_from_truth, timed_position_observation_with_doppler,
};
pub use residual_model::{
    iterative_pseudorange_residual_m, iterative_pseudorange_residual_without_earth_rotation_m,
};
pub use scenario_catalog::{
    clear_broadcast_clock_parameters, four_satellite_position_scenario,
    four_satellite_position_scenario_from_truth, four_satellite_position_scenario_with_ephemerides,
    impossible_geometry_position_scenario, sample_ephemerides,
    sample_ephemerides_with_clock_parameters, sample_ephemeris,
    sample_ephemeris_with_clock_parameters, sample_klobuchar_coefficients,
    BroadcastClockParameters, SyntheticPositionScenario,
};

fn geodetic_point_to_tuple(point: GeodeticPoint) -> (f64, f64, f64) {
    let ecef_m = geodetic_to_ecef_m(point);
    (ecef_m[0], ecef_m[1], ecef_m[2])
}

fn llh_from_ecef_tuple(ecef_m: (f64, f64, f64)) -> Llh {
    let geodetic = ecef_to_geodetic_point(ecef_tuple_to_array(ecef_m));
    Llh { lat_deg: geodetic.lat_deg, lon_deg: geodetic.lon_deg, alt_m: geodetic.alt_m }
}

fn ecef_tuple_to_array(ecef_m: (f64, f64, f64)) -> [f64; 3] {
    [ecef_m.0, ecef_m.1, ecef_m.2]
}

#[cfg(test)]
mod tests {
    use super::{
        add_klobuchar_delay_to_observations, four_satellite_position_scenario,
        sample_klobuchar_coefficients,
    };

    #[test]
    fn scenario_contains_four_observations() {
        let scenario = four_satellite_position_scenario(0.0);

        assert_eq!(scenario.ephemerides.len(), 4);
        assert_eq!(scenario.observations.len(), 4);
    }

    #[test]
    fn klobuchar_biases_increase_pseudorange() {
        let scenario = four_satellite_position_scenario(0.0);
        let biased = add_klobuchar_delay_to_observations(
            &scenario.observations,
            &scenario.ephemerides,
            scenario.truth_ecef_m,
            scenario.t_rx_s,
            sample_klobuchar_coefficients(),
        );

        let total_added_delay_m = biased
            .iter()
            .zip(&scenario.observations)
            .map(|(left, right)| left.pseudorange_m - right.pseudorange_m)
            .sum::<f64>();

        assert!(biased
            .iter()
            .zip(&scenario.observations)
            .all(|(left, right)| left.pseudorange_m + 1.0e-9 >= right.pseudorange_m));
        assert!(total_added_delay_m > 0.0);
    }
}
