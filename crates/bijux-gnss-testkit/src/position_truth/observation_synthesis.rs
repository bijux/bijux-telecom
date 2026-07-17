use std::collections::BTreeMap;

use bijux_gnss_core::api::{
    GpsTime, ObsSignalTiming, SatId, Seconds, SigId, SignalBand, SignalCode,
};
use bijux_gnss_nav::api::{GpsEphemeris, KlobucharCoefficients, PositionObservation};

use crate::reference_models::atmosphere::{klobuchar_delay_m, saastamoinen_delay_m};
use crate::reference_models::coordinates::elevation_azimuth_deg;
use crate::reference_models::gps_broadcast::{
    pseudorange_from_truth as broadcast_pseudorange_from_truth, sat_state_gps_l1ca,
    satellite_state_from_observation,
};

use super::{ecef_tuple_to_array, llh_from_ecef_tuple, GPS_L1_CA_WAVELENGTH_M, SPEED_OF_LIGHT_MPS};

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
