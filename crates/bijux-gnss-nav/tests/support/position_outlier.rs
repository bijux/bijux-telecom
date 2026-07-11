#![allow(dead_code, missing_docs)]

use bijux_gnss_core::api::{Seconds, SatId};
use bijux_gnss_nav::api::PositionObservation;

use super::position_truth::{
    four_satellite_position_scenario, sample_ephemeris, timed_position_observation_from_truth,
    SyntheticPositionScenario,
};

const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;

#[derive(Debug, Clone)]
pub struct BadPseudorangeScenario {
    pub baseline: SyntheticPositionScenario,
    pub observations: Vec<PositionObservation>,
    pub bad_sat: SatId,
    pub pseudorange_bias_m: f64,
}

pub fn single_bad_pseudorange_scenario(pseudorange_bias_m: f64) -> BadPseudorangeScenario {
    let mut baseline = four_satellite_position_scenario(0.0);
    let extra_ephemeris = sample_ephemeris(5, 3.2, 3.6);
    baseline.observations.push(timed_position_observation_from_truth(
        &extra_ephemeris,
        baseline.truth_ecef_m,
        baseline.t_rx_s,
        baseline.receiver_clock_bias_s,
    ));
    baseline.ephemerides.push(extra_ephemeris);

    let bad_sat = baseline.ephemerides.last().expect("bad-satellite ephemeris").sat;
    let observations = baseline
        .observations
        .iter()
        .cloned()
        .map(|observation| {
            if observation.sat == bad_sat {
                observation_with_pseudorange_bias(&observation, pseudorange_bias_m)
            } else {
                observation
            }
        })
        .collect();

    BadPseudorangeScenario { baseline, observations, bad_sat, pseudorange_bias_m }
}

pub fn observation_with_pseudorange_bias(
    observation: &PositionObservation,
    pseudorange_bias_m: f64,
) -> PositionObservation {
    let mut biased = observation.clone();
    biased.pseudorange_m += pseudorange_bias_m;
    if let Some(signal_timing) = &mut biased.signal_timing {
        let signal_travel_time_bias_s = pseudorange_bias_m / SPEED_OF_LIGHT_MPS;
        signal_timing.signal_travel_time_s =
            Seconds(signal_timing.signal_travel_time_s.0 + signal_travel_time_bias_s);
        signal_timing.transmit_gps_time =
            signal_timing.transmit_gps_time.offset_seconds(-signal_travel_time_bias_s);
    }
    biased
}

pub fn position_error_3d_m(
    ecef_x_m: f64,
    ecef_y_m: f64,
    ecef_z_m: f64,
    truth_ecef_m: (f64, f64, f64),
) -> f64 {
    let dx = ecef_x_m - truth_ecef_m.0;
    let dy = ecef_y_m - truth_ecef_m.1;
    let dz = ecef_z_m - truth_ecef_m.2;
    (dx * dx + dy * dy + dz * dz).sqrt()
}
