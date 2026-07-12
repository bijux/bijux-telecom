#![allow(missing_docs)]

mod support;

use bijux_gnss_core::api::{Llh, Seconds};
use bijux_gnss_nav::api::{
    ecef_to_geodetic, elevation_azimuth_deg, sat_state_gps_l1ca_from_observation,
    SaastamoinenModel, TroposphereModel,
};
use bijux_gnss_testkit::public_troposphere::LOW_ELEVATION_CEILING_DEG;
use support::position_truth::{
    add_saastamoinen_delay_to_observations, four_satellite_position_scenario,
};

#[derive(Debug, Clone, Copy)]
struct ResidualSample {
    elevation_deg: f64,
    corrected_abs_residual_m: f64,
    uncorrected_abs_residual_m: f64,
}

#[test]
fn synthetic_low_elevation_residuals_improve_with_troposphere_correction() {
    let scenario = four_satellite_position_scenario(0.0);
    let biased_observations = add_saastamoinen_delay_to_observations(
        &scenario.observations,
        &scenario.ephemerides,
        scenario.truth_ecef_m,
        scenario.t_rx_s,
    );
    let samples = synthetic_residual_samples(&scenario, &biased_observations);
    let low_samples = samples
        .iter()
        .copied()
        .filter(|sample| sample.elevation_deg > 0.0 && sample.elevation_deg < LOW_ELEVATION_CEILING_DEG)
        .collect::<Vec<_>>();
    let high_samples = samples
        .iter()
        .copied()
        .filter(|sample| sample.elevation_deg >= 45.0)
        .collect::<Vec<_>>();

    assert!(!low_samples.is_empty(), "synthetic scenario should include low-elevation satellites");
    assert!(!high_samples.is_empty(), "synthetic scenario should include high-elevation satellites");
    assert!(
        mean_abs_improvement_m(&samples) > 0.5,
        "synthetic troposphere correction should reduce residuals overall: {samples:?}"
    );
    assert!(
        mean_abs_improvement_m(&low_samples) > 1.0,
        "low-elevation synthetic satellites should see clear residual improvement: {low_samples:?}"
    );
    assert!(
        mean_abs_improvement_m(&low_samples) > mean_abs_improvement_m(&high_samples),
        "low-elevation residual improvement should exceed high-elevation improvement; low={low_samples:?} high={high_samples:?}"
    );
}

fn synthetic_residual_samples(
    scenario: &support::position_truth::SyntheticPositionScenario,
    observations: &[bijux_gnss_nav::api::PositionObservation],
) -> Vec<ResidualSample> {
    let (lat_deg, lon_deg, alt_m) =
        ecef_to_geodetic(scenario.truth_ecef_m.0, scenario.truth_ecef_m.1, scenario.truth_ecef_m.2);
    let receiver = Llh { lat_deg, lon_deg, alt_m };
    let model = SaastamoinenModel;

    scenario
        .observations
        .iter()
        .filter_map(|reference_observation| {
            let biased_observation =
                observations.iter().find(|observation| observation.sat == reference_observation.sat)?;
            let ephemeris = scenario
                .ephemerides
                .iter()
                .find(|ephemeris| ephemeris.sat == reference_observation.sat)?;
            let state = sat_state_gps_l1ca_from_observation(
                ephemeris,
                scenario.t_rx_s,
                reference_observation.pseudorange_m,
                reference_observation.signal_timing,
            );
            let (_azimuth_deg, elevation_deg) = elevation_azimuth_deg(
                scenario.truth_ecef_m.0,
                scenario.truth_ecef_m.1,
                scenario.truth_ecef_m.2,
                state.x_m,
                state.y_m,
                state.z_m,
            );
            let modeled_delay_m = model.delay_m(receiver, elevation_deg, Seconds(scenario.t_rx_s));
            let injected_delay_m = biased_observation.pseudorange_m - reference_observation.pseudorange_m;

            Some(ResidualSample {
                elevation_deg,
                corrected_abs_residual_m: (injected_delay_m - modeled_delay_m).abs(),
                uncorrected_abs_residual_m: injected_delay_m.abs(),
            })
        })
        .collect()
}

fn mean_abs_improvement_m(samples: &[ResidualSample]) -> f64 {
    samples
        .iter()
        .map(|sample| sample.uncorrected_abs_residual_m - sample.corrected_abs_residual_m)
        .sum::<f64>()
        / samples.len() as f64
}
