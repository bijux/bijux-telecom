#![allow(missing_docs)]
mod support;

use bijux_gnss_nav::api::position_observation_has_valid_satellite_time;
use support::position_truth::{
    add_satellite_delay_biases_to_observations, add_uniform_delay_bias_to_observations,
    four_satellite_position_scenario,
};

#[test]
fn satellite_delay_bias_helper_preserves_internal_timing_consistency() {
    let scenario = four_satellite_position_scenario(0.0);
    let delayed = add_satellite_delay_biases_to_observations(
        &scenario.observations,
        &[(scenario.ephemerides[0].sat, 120.0), (scenario.ephemerides[2].sat, 35.0)],
    );

    assert!(
        delayed.iter().all(|observation| {
            position_observation_has_valid_satellite_time(observation, scenario.t_rx_s)
        }),
        "{delayed:?}"
    );
    assert!(
        (delayed[0].pseudorange_m - scenario.observations[0].pseudorange_m - 120.0).abs() < 1.0e-9
    );
    assert!((delayed[1].pseudorange_m - scenario.observations[1].pseudorange_m).abs() < 1.0e-9);
    assert!(
        (delayed[2].pseudorange_m - scenario.observations[2].pseudorange_m - 35.0).abs() < 1.0e-9
    );
}

#[test]
fn uniform_delay_bias_helper_preserves_internal_timing_consistency() {
    let scenario = four_satellite_position_scenario(0.0);
    let delayed = add_uniform_delay_bias_to_observations(&scenario.observations, 85.0);

    assert!(
        delayed.iter().all(|observation| {
            position_observation_has_valid_satellite_time(observation, scenario.t_rx_s)
        }),
        "{delayed:?}"
    );
    assert!(delayed.iter().zip(&scenario.observations).all(|(biased, original)| {
        (biased.pseudorange_m - original.pseudorange_m - 85.0).abs() < 1.0e-9
    }));
}
