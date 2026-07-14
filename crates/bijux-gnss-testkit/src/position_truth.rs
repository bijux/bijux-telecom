//! Independent GPS position-truth scenarios for navigation and receiver validation.
#![allow(missing_docs)]

use std::collections::BTreeMap;

use bijux_gnss_core::api::{
    Constellation, GpsTime, Llh, ObsSignalTiming, SatId, Seconds, SigId, SignalBand, SignalCode,
};
use bijux_gnss_nav::api::{GpsEphemeris, KlobucharCoefficients, PositionObservation};

use crate::reference_math::atmosphere::{klobuchar_delay_m, saastamoinen_delay_m};
use crate::reference_math::coordinates::{
    ecef_to_geodetic_point, elevation_azimuth_deg, geodetic_to_ecef_m, GeodeticPoint,
};
use crate::reference_math::gps_broadcast::{
    pseudorange_from_truth as broadcast_pseudorange_from_truth, sat_state_gps_l1ca,
    satellite_state_from_observation,
};

const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;
const GPS_L1_CA_CARRIER_HZ_MPS: f64 = 1_575_420_000.0;
const GPS_L1_CA_WAVELENGTH_M: f64 = SPEED_OF_LIGHT_MPS / GPS_L1_CA_CARRIER_HZ_MPS;

#[derive(Debug, Clone, Copy, Default)]
pub struct BroadcastClockParameters {
    pub af0_s: f64,
    pub af1_s_per_s: f64,
    pub af2_s_per_s2: f64,
    pub tgd_s: f64,
}

#[derive(Debug, Clone)]
pub struct SyntheticPositionScenario {
    pub ephemerides: Vec<GpsEphemeris>,
    pub observations: Vec<PositionObservation>,
    pub truth_ecef_m: (f64, f64, f64),
    pub receiver_clock_bias_s: f64,
    pub t_rx_s: f64,
}

pub fn four_satellite_position_scenario(receiver_clock_bias_s: f64) -> SyntheticPositionScenario {
    four_satellite_position_scenario_from_truth(
        receiver_clock_bias_s,
        geodetic_point_to_tuple(GeodeticPoint { lat_deg: 37.0, lon_deg: -122.0, alt_m: 10.0 }),
        sample_ephemerides(),
    )
}

pub fn impossible_geometry_position_scenario(
    receiver_clock_bias_s: f64,
) -> SyntheticPositionScenario {
    four_satellite_position_scenario_from_truth(
        receiver_clock_bias_s,
        geodetic_point_to_tuple(GeodeticPoint {
            lat_deg: 37.0,
            lon_deg: -122.0,
            alt_m: -2_000_000.0,
        }),
        sample_ephemerides(),
    )
}

pub fn four_satellite_position_scenario_with_ephemerides(
    receiver_clock_bias_s: f64,
    ephemerides: Vec<GpsEphemeris>,
) -> SyntheticPositionScenario {
    four_satellite_position_scenario_from_truth(
        receiver_clock_bias_s,
        geodetic_point_to_tuple(GeodeticPoint { lat_deg: 37.0, lon_deg: -122.0, alt_m: 10.0 }),
        ephemerides,
    )
}

pub fn four_satellite_position_scenario_from_truth(
    receiver_clock_bias_s: f64,
    truth_ecef_m: (f64, f64, f64),
    ephemerides: Vec<GpsEphemeris>,
) -> SyntheticPositionScenario {
    let t_rx_s = 504_018.07 + receiver_clock_bias_s;
    let observations = ephemerides
        .iter()
        .map(|ephemeris| {
            timed_position_observation_from_truth(
                ephemeris,
                truth_ecef_m,
                t_rx_s,
                receiver_clock_bias_s,
            )
        })
        .collect();

    SyntheticPositionScenario {
        ephemerides,
        observations,
        truth_ecef_m,
        receiver_clock_bias_s,
        t_rx_s,
    }
}

pub fn sample_ephemerides() -> Vec<GpsEphemeris> {
    sample_ephemerides_with_clock_parameters(&[])
}

pub fn sample_ephemerides_with_clock_parameters(
    clock_parameters_by_prn: &[(u8, BroadcastClockParameters)],
) -> Vec<GpsEphemeris> {
    let clock_parameters_by_prn =
        clock_parameters_by_prn.iter().copied().collect::<BTreeMap<_, _>>();
    vec![
        sample_ephemeris_with_clock_parameters(
            1,
            0.0,
            0.0,
            clock_parameters_by_prn.get(&1).copied().unwrap_or_default(),
        ),
        sample_ephemeris_with_clock_parameters(
            2,
            0.8,
            0.9,
            clock_parameters_by_prn.get(&2).copied().unwrap_or_default(),
        ),
        sample_ephemeris_with_clock_parameters(
            3,
            1.6,
            1.8,
            clock_parameters_by_prn.get(&3).copied().unwrap_or_default(),
        ),
        sample_ephemeris_with_clock_parameters(
            4,
            2.4,
            2.7,
            clock_parameters_by_prn.get(&4).copied().unwrap_or_default(),
        ),
    ]
}

pub fn sample_ephemeris(prn: u8, omega0: f64, m0: f64) -> GpsEphemeris {
    sample_ephemeris_with_clock_parameters(prn, omega0, m0, BroadcastClockParameters::default())
}

pub fn sample_ephemeris_with_clock_parameters(
    prn: u8,
    omega0: f64,
    m0: f64,
    clock_parameters: BroadcastClockParameters,
) -> GpsEphemeris {
    GpsEphemeris {
        sat: SatId { constellation: Constellation::Gps, prn },
        iodc: 1,
        iode: 1,
        week: 2209,
        sv_health: 0,
        toe_s: 504_000.0,
        toc_s: 504_018.0,
        sqrt_a: 5153.7954775,
        e: 0.01,
        i0: 0.94,
        idot: 0.0,
        omega0,
        omegadot: 0.0,
        w: 0.0,
        m0,
        delta_n: 0.0,
        cuc: 0.0,
        cus: 0.0,
        crc: 0.0,
        crs: 0.0,
        cic: 0.0,
        cis: 0.0,
        af0: clock_parameters.af0_s,
        af1: clock_parameters.af1_s_per_s,
        af2: clock_parameters.af2_s_per_s2,
        tgd: clock_parameters.tgd_s,
    }
}

pub fn clear_broadcast_clock_parameters(ephemerides: &[GpsEphemeris]) -> Vec<GpsEphemeris> {
    ephemerides
        .iter()
        .cloned()
        .map(|mut ephemeris| {
            ephemeris.af0 = 0.0;
            ephemeris.af1 = 0.0;
            ephemeris.af2 = 0.0;
            ephemeris.tgd = 0.0;
            ephemeris
        })
        .collect()
}

pub fn sample_klobuchar_coefficients() -> KlobucharCoefficients {
    KlobucharCoefficients::new(
        [0.1212e-7, 0.1490e-7, -0.5960e-7, 0.1192e-6],
        [0.1167e6, -0.2294e6, -0.1311e6, 0.1049e7],
    )
}

pub fn add_klobuchar_delay_to_observations(
    observations: &[PositionObservation],
    ephemerides: &[GpsEphemeris],
    receiver_ecef_m: (f64, f64, f64),
    t_rx_s: f64,
    klobuchar: KlobucharCoefficients,
) -> Vec<PositionObservation> {
    let receiver = llh_from_ecef_tuple(receiver_ecef_m);

    observations
        .iter()
        .map(|observation| {
            let ephemeris = ephemerides
                .iter()
                .find(|candidate| candidate.sat == observation.sat)
                .expect("matching ephemeris");
            let state = satellite_state_from_observation(
                ephemeris,
                t_rx_s,
                observation.pseudorange_m,
                observation.signal_timing,
            );
            let (azimuth_deg, elevation_deg) = elevation_azimuth_deg(
                ecef_tuple_to_array(receiver_ecef_m),
                [state.x_m, state.y_m, state.z_m],
            );
            let delay_m =
                klobuchar_delay_m(receiver, azimuth_deg, elevation_deg, t_rx_s, klobuchar);
            delay_bias_to_observation(observation, delay_m)
        })
        .collect()
}

pub fn add_saastamoinen_delay_to_observations(
    observations: &[PositionObservation],
    ephemerides: &[GpsEphemeris],
    receiver_ecef_m: (f64, f64, f64),
    t_rx_s: f64,
) -> Vec<PositionObservation> {
    let receiver = llh_from_ecef_tuple(receiver_ecef_m);

    observations
        .iter()
        .map(|observation| {
            let ephemeris = ephemerides
                .iter()
                .find(|candidate| candidate.sat == observation.sat)
                .expect("matching ephemeris");
            let state = satellite_state_from_observation(
                ephemeris,
                t_rx_s,
                observation.pseudorange_m,
                observation.signal_timing,
            );
            let (_azimuth_deg, elevation_deg) = elevation_azimuth_deg(
                ecef_tuple_to_array(receiver_ecef_m),
                [state.x_m, state.y_m, state.z_m],
            );
            let delay_m = saastamoinen_delay_m(receiver, elevation_deg, Seconds(t_rx_s));
            delay_bias_to_observation(observation, delay_m)
        })
        .collect()
}

pub fn add_satellite_delay_biases_to_observations(
    observations: &[PositionObservation],
    delay_biases_by_sat_m: &[(SatId, f64)],
) -> Vec<PositionObservation> {
    let delay_biases_by_sat_m = delay_biases_by_sat_m.iter().copied().collect::<BTreeMap<_, _>>();
    observations
        .iter()
        .map(|observation| {
            let delay_m = delay_biases_by_sat_m.get(&observation.sat).copied().unwrap_or(0.0);
            delay_bias_to_observation(observation, delay_m)
        })
        .collect()
}

pub fn add_uniform_delay_bias_to_observations(
    observations: &[PositionObservation],
    delay_m: f64,
) -> Vec<PositionObservation> {
    observations.iter().map(|observation| delay_bias_to_observation(observation, delay_m)).collect()
}

pub fn timed_position_observation(
    sat: SatId,
    pseudorange_m: f64,
    t_rx_s: f64,
) -> PositionObservation {
    let signal_travel_time_s = pseudorange_m / SPEED_OF_LIGHT_MPS;
    PositionObservation {
        sat,
        pseudorange_m,
        doppler_hz: None,
        doppler_var_hz2: None,
        cn0_dbhz: 45.0,
        elevation_deg: None,
        weight: 1.0,
        gps_receive_time: Some(GpsTime { week: 0, tow_s: t_rx_s }),
        signal_timing: Some(ObsSignalTiming {
            signal_travel_time_s: Seconds(signal_travel_time_s),
            transmit_gps_time: GpsTime { week: 0, tow_s: t_rx_s - signal_travel_time_s },
        }),
        signal_id: None,
    }
}

pub fn gps_l1ca_signal_id(sat: SatId) -> SigId {
    SigId { sat, band: SignalBand::L1, code: SignalCode::Ca }
}

pub fn receiver_clock_bias_with_drift_s(
    initial_clock_bias_s: f64,
    receiver_clock_drift_s_per_s: f64,
    elapsed_s: f64,
) -> f64 {
    initial_clock_bias_s + receiver_clock_drift_s_per_s * elapsed_s
}

pub fn timed_position_observation_with_doppler(
    sat: SatId,
    pseudorange_m: f64,
    doppler_hz: f64,
    doppler_var_hz2: f64,
    t_rx_s: f64,
) -> PositionObservation {
    let mut observation = timed_position_observation(sat, pseudorange_m, t_rx_s);
    observation.doppler_hz = Some(doppler_hz);
    observation.doppler_var_hz2 = Some(doppler_var_hz2);
    observation.signal_id = Some(gps_l1ca_signal_id(sat));
    observation
}

pub fn timed_position_observation_from_truth(
    ephemeris: &GpsEphemeris,
    truth_ecef_m: (f64, f64, f64),
    t_rx_s: f64,
    receiver_clock_bias_s: f64,
) -> PositionObservation {
    let pseudorange_m =
        pseudorange_from_truth(ephemeris, truth_ecef_m, t_rx_s, receiver_clock_bias_s);
    timed_position_observation(ephemeris.sat, pseudorange_m, t_rx_s)
}

pub fn pseudorange_from_truth(
    ephemeris: &GpsEphemeris,
    truth_ecef_m: (f64, f64, f64),
    t_rx_s: f64,
    receiver_clock_bias_s: f64,
) -> f64 {
    broadcast_pseudorange_from_truth(
        ephemeris,
        ecef_tuple_to_array(truth_ecef_m),
        t_rx_s,
        receiver_clock_bias_s,
    )
}

pub fn gps_l1ca_doppler_from_truth(
    ephemeris: &GpsEphemeris,
    truth_ecef_m: (f64, f64, f64),
    truth_velocity_ecef_mps: (f64, f64, f64),
    t_rx_s: f64,
    receiver_clock_bias_s: f64,
    receiver_clock_drift_s_per_s: f64,
) -> f64 {
    let pseudorange_m =
        pseudorange_from_truth(ephemeris, truth_ecef_m, t_rx_s, receiver_clock_bias_s);
    let signal_travel_time_s = pseudorange_m / SPEED_OF_LIGHT_MPS;
    let transmit_tow_s = t_rx_s - signal_travel_time_s;
    let state = sat_state_gps_l1ca(ephemeris, transmit_tow_s, signal_travel_time_s);
    let satellite_velocity_mps =
        gps_satellite_velocity_mps(ephemeris, transmit_tow_s, signal_travel_time_s);

    let dx_m = truth_ecef_m.0 - state.x_m;
    let dy_m = truth_ecef_m.1 - state.y_m;
    let dz_m = truth_ecef_m.2 - state.z_m;
    let range_m = (dx_m * dx_m + dy_m * dy_m + dz_m * dz_m).sqrt().max(1.0);
    let line_of_sight = [dx_m / range_m, dy_m / range_m, dz_m / range_m];
    let relative_velocity_mps = [
        truth_velocity_ecef_mps.0 - satellite_velocity_mps.0,
        truth_velocity_ecef_mps.1 - satellite_velocity_mps.1,
        truth_velocity_ecef_mps.2 - satellite_velocity_mps.2,
    ];
    let range_rate_mps = line_of_sight[0] * relative_velocity_mps[0]
        + line_of_sight[1] * relative_velocity_mps[1]
        + line_of_sight[2] * relative_velocity_mps[2];

    -range_rate_mps / GPS_L1_CA_WAVELENGTH_M
        + SPEED_OF_LIGHT_MPS * (receiver_clock_drift_s_per_s - state.clock_correction.drift_s_per_s)
            / GPS_L1_CA_WAVELENGTH_M
}

pub fn iterative_pseudorange_residual_m(
    ephemeris: &GpsEphemeris,
    observation: &PositionObservation,
    receiver_ecef_m: (f64, f64, f64),
    receiver_clock_bias_s: f64,
    t_rx_s: f64,
) -> f64 {
    iterative_pseudorange_residual_with_earth_rotation_mode_m(
        ephemeris,
        observation,
        receiver_ecef_m,
        receiver_clock_bias_s,
        t_rx_s,
        true,
    )
}

pub fn iterative_pseudorange_residual_without_earth_rotation_m(
    ephemeris: &GpsEphemeris,
    observation: &PositionObservation,
    receiver_ecef_m: (f64, f64, f64),
    receiver_clock_bias_s: f64,
    t_rx_s: f64,
) -> f64 {
    iterative_pseudorange_residual_with_earth_rotation_mode_m(
        ephemeris,
        observation,
        receiver_ecef_m,
        receiver_clock_bias_s,
        t_rx_s,
        false,
    )
}

fn gps_satellite_velocity_mps(
    ephemeris: &GpsEphemeris,
    transmit_tow_s: f64,
    signal_travel_time_s: f64,
) -> (f64, f64, f64) {
    let dt_s = 1.0e-3;
    let previous = sat_state_gps_l1ca(ephemeris, transmit_tow_s - dt_s, signal_travel_time_s);
    let next = sat_state_gps_l1ca(ephemeris, transmit_tow_s + dt_s, signal_travel_time_s);
    let inverse_dt_s = 1.0 / (2.0 * dt_s);
    (
        (next.x_m - previous.x_m) * inverse_dt_s,
        (next.y_m - previous.y_m) * inverse_dt_s,
        (next.z_m - previous.z_m) * inverse_dt_s,
    )
}

fn delay_bias_to_observation(
    observation: &PositionObservation,
    delay_m: f64,
) -> PositionObservation {
    let delay_s = delay_m / SPEED_OF_LIGHT_MPS;
    let mut biased = observation.clone();
    biased.pseudorange_m += delay_m;
    if let Some(signal_timing) = &mut biased.signal_timing {
        signal_timing.signal_travel_time_s =
            Seconds(signal_timing.signal_travel_time_s.0 + delay_s);
        signal_timing.transmit_gps_time = signal_timing.transmit_gps_time.offset_seconds(-delay_s);
    }
    biased
}

fn iterative_pseudorange_residual_with_earth_rotation_mode_m(
    ephemeris: &GpsEphemeris,
    observation: &PositionObservation,
    receiver_ecef_m: (f64, f64, f64),
    receiver_clock_bias_s: f64,
    t_rx_s: f64,
    apply_earth_rotation: bool,
) -> f64 {
    let receive_tow_s =
        observation.gps_receive_time.map(|gps_time| gps_time.tow_s).unwrap_or(t_rx_s);
    let mut signal_travel_time_s = observation
        .signal_timing
        .map(|timing| timing.signal_travel_time_s.0)
        .unwrap_or(observation.pseudorange_m / SPEED_OF_LIGHT_MPS);
    let predicted_pseudorange_m = iterative_predicted_pseudorange_m(
        ephemeris,
        receiver_ecef_m,
        receiver_clock_bias_s,
        receive_tow_s,
        &mut signal_travel_time_s,
        apply_earth_rotation,
    );
    observation.pseudorange_m - predicted_pseudorange_m
}

fn iterative_predicted_pseudorange_m(
    ephemeris: &GpsEphemeris,
    receiver_ecef_m: (f64, f64, f64),
    receiver_clock_bias_s: f64,
    receive_tow_s: f64,
    signal_travel_time_s: &mut f64,
    apply_earth_rotation: bool,
) -> f64 {
    let mut predicted_pseudorange_m = 0.0;
    for _ in 0..10 {
        let state = sat_state_gps_l1ca(
            ephemeris,
            receive_tow_s - *signal_travel_time_s,
            if apply_earth_rotation { *signal_travel_time_s } else { 0.0 },
        );
        let dx_m = receiver_ecef_m.0 - state.x_m;
        let dy_m = receiver_ecef_m.1 - state.y_m;
        let dz_m = receiver_ecef_m.2 - state.z_m;
        let range_m = (dx_m * dx_m + dy_m * dy_m + dz_m * dz_m).sqrt();
        predicted_pseudorange_m = range_m + receiver_clock_bias_s * SPEED_OF_LIGHT_MPS
            - state.clock_correction.bias_s * SPEED_OF_LIGHT_MPS;
        let next_signal_travel_time_s = predicted_pseudorange_m / SPEED_OF_LIGHT_MPS;
        if (next_signal_travel_time_s - *signal_travel_time_s).abs() < 1.0e-12 {
            break;
        }
        *signal_travel_time_s = next_signal_travel_time_s;
    }
    predicted_pseudorange_m
}

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
