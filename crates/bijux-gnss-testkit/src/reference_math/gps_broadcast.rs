//! Independent GPS broadcast orbit and clock propagation for validation truth.

use bijux_gnss_core::api::ObsSignalTiming;
use bijux_gnss_nav::api::GpsEphemeris;

const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;
const EARTH_ROTATION_RAD_PER_S: f64 = 7.292_115_146_7e-5;
const EARTH_GRAVITATIONAL_CONSTANT_M3_PER_S2: f64 = 3.986_005e14;
const RELATIVISTIC_CLOCK_CONSTANT: f64 = -4.442_807_633e-10;
const HALF_WEEK_S: f64 = 302_400.0;
const FULL_WEEK_S: f64 = 604_800.0;

#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct BroadcastClockCorrection {
    pub bias_s: f64,
    pub drift_s_per_s: f64,
    pub drift_rate_s_per_s2: f64,
    pub relativistic_s: f64,
    pub group_delay_s: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct BroadcastGpsState {
    pub x_m: f64,
    pub y_m: f64,
    pub z_m: f64,
    pub clock_correction: BroadcastClockCorrection,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct SolvedTransmitState {
    pub transmit_state: BroadcastGpsState,
    pub signal_travel_time_s: f64,
}

pub(crate) fn sat_state_gps_l1ca(
    ephemeris: &GpsEphemeris,
    transmit_tow_s: f64,
    signal_travel_time_s: f64,
) -> BroadcastGpsState {
    let orbital_terms = orbital_terms(ephemeris, transmit_tow_s);
    let raw_position_m = orbital_position_ecef_m(ephemeris, orbital_terms, transmit_tow_s);
    let rotated_position_m = rotate_for_earth_rotation(raw_position_m, signal_travel_time_s);
    BroadcastGpsState {
        x_m: rotated_position_m[0],
        y_m: rotated_position_m[1],
        z_m: rotated_position_m[2],
        clock_correction: satellite_clock_correction(ephemeris, transmit_tow_s, orbital_terms),
    }
}

pub(crate) fn sat_state_gps_l1ca_at_receive_time(
    ephemeris: &GpsEphemeris,
    receive_tow_s: f64,
    signal_travel_time_s: f64,
) -> BroadcastGpsState {
    sat_state_gps_l1ca(ephemeris, receive_tow_s - signal_travel_time_s, signal_travel_time_s)
}

pub(crate) fn sat_state_gps_l1ca_from_observation(
    ephemeris: &GpsEphemeris,
    receive_tow_s: f64,
    pseudorange_m: f64,
    signal_timing: Option<ObsSignalTiming>,
) -> BroadcastGpsState {
    let signal_travel_time_s = signal_timing
        .map(|timing| timing.signal_travel_time_s.0)
        .unwrap_or(pseudorange_m / SPEED_OF_LIGHT_MPS);
    sat_state_gps_l1ca_at_receive_time(ephemeris, receive_tow_s, signal_travel_time_s)
}

pub(crate) fn satellite_state_from_observation(
    ephemeris: &GpsEphemeris,
    receive_tow_s: f64,
    pseudorange_m: f64,
    signal_timing: Option<ObsSignalTiming>,
) -> BroadcastGpsState {
    sat_state_gps_l1ca_from_observation(ephemeris, receive_tow_s, pseudorange_m, signal_timing)
}

pub(crate) fn solve_transmit_state_for_receiver(
    ephemeris: &GpsEphemeris,
    receive_tow_s: f64,
    receiver_ecef_m: [f64; 3],
) -> SolvedTransmitState {
    let mut signal_travel_time_s = 0.07;
    let mut transmit_state =
        sat_state_gps_l1ca_at_receive_time(ephemeris, receive_tow_s, signal_travel_time_s);

    for _ in 0..10 {
        let next_signal_travel_time_s = geometric_range_m(
            receiver_ecef_m,
            [transmit_state.x_m, transmit_state.y_m, transmit_state.z_m],
        ) / SPEED_OF_LIGHT_MPS;
        if (next_signal_travel_time_s - signal_travel_time_s).abs() < 1.0e-12 {
            signal_travel_time_s = next_signal_travel_time_s;
            break;
        }
        signal_travel_time_s = next_signal_travel_time_s;
        transmit_state =
            sat_state_gps_l1ca_at_receive_time(ephemeris, receive_tow_s, signal_travel_time_s);
    }

    SolvedTransmitState { transmit_state, signal_travel_time_s }
}

pub(crate) fn pseudorange_from_truth(
    ephemeris: &GpsEphemeris,
    truth_ecef_m: [f64; 3],
    receive_tow_s: f64,
    receiver_clock_bias_s: f64,
) -> f64 {
    let mut signal_travel_time_s = 0.07;
    let mut pseudorange_m = 0.0;
    for _ in 0..10 {
        let state =
            sat_state_gps_l1ca_at_receive_time(ephemeris, receive_tow_s, signal_travel_time_s);
        let geometric_range_m = geometric_range_m(truth_ecef_m, [state.x_m, state.y_m, state.z_m]);
        pseudorange_m = geometric_range_m + receiver_clock_bias_s * SPEED_OF_LIGHT_MPS
            - state.clock_correction.bias_s * SPEED_OF_LIGHT_MPS;
        let next_signal_travel_time_s = pseudorange_m / SPEED_OF_LIGHT_MPS;
        if (next_signal_travel_time_s - signal_travel_time_s).abs() < 1.0e-12 {
            break;
        }
        signal_travel_time_s = next_signal_travel_time_s;
    }
    pseudorange_m
}

pub(crate) fn geometric_range_m(receiver_ecef_m: [f64; 3], sat_ecef_m: [f64; 3]) -> f64 {
    let dx = receiver_ecef_m[0] - sat_ecef_m[0];
    let dy = receiver_ecef_m[1] - sat_ecef_m[1];
    let dz = receiver_ecef_m[2] - sat_ecef_m[2];
    (dx * dx + dy * dy + dz * dz).sqrt()
}

#[derive(Debug, Clone, Copy)]
struct OrbitalTerms {
    eccentric_anomaly_rad: f64,
    corrected_argument_of_latitude_rad: f64,
    corrected_radius_m: f64,
    corrected_inclination_rad: f64,
}

fn orbital_terms(ephemeris: &GpsEphemeris, transmit_tow_s: f64) -> OrbitalTerms {
    let a_m = ephemeris.sqrt_a * ephemeris.sqrt_a;
    let tk_s = wrapped_time_difference_s(transmit_tow_s, ephemeris.toe_s);
    let mean_motion_rad_per_s =
        (EARTH_GRAVITATIONAL_CONSTANT_M3_PER_S2 / a_m.powi(3)).sqrt() + ephemeris.delta_n;
    let mean_anomaly_rad = ephemeris.m0 + mean_motion_rad_per_s * tk_s;
    let eccentric_anomaly_rad = solve_eccentric_anomaly_rad(mean_anomaly_rad, ephemeris.e);
    let sin_e = eccentric_anomaly_rad.sin();
    let cos_e = eccentric_anomaly_rad.cos();
    let true_anomaly_rad =
        ((1.0 - ephemeris.e * ephemeris.e).sqrt() * sin_e).atan2(cos_e - ephemeris.e);
    let argument_of_latitude_rad = true_anomaly_rad + ephemeris.w;
    let twice_argument_rad = 2.0 * argument_of_latitude_rad;
    let du_rad =
        ephemeris.cus * twice_argument_rad.sin() + ephemeris.cuc * twice_argument_rad.cos();
    let dr_m = ephemeris.crs * twice_argument_rad.sin() + ephemeris.crc * twice_argument_rad.cos();
    let di_rad =
        ephemeris.cis * twice_argument_rad.sin() + ephemeris.cic * twice_argument_rad.cos();

    OrbitalTerms {
        eccentric_anomaly_rad,
        corrected_argument_of_latitude_rad: argument_of_latitude_rad + du_rad,
        corrected_radius_m: a_m * (1.0 - ephemeris.e * cos_e) + dr_m,
        corrected_inclination_rad: ephemeris.i0 + di_rad + ephemeris.idot * tk_s,
    }
}

fn orbital_position_ecef_m(
    ephemeris: &GpsEphemeris,
    terms: OrbitalTerms,
    transmit_tow_s: f64,
) -> [f64; 3] {
    let tk_s = wrapped_time_difference_s(transmit_tow_s, ephemeris.toe_s);
    let x_orbital_m = terms.corrected_radius_m * terms.corrected_argument_of_latitude_rad.cos();
    let y_orbital_m = terms.corrected_radius_m * terms.corrected_argument_of_latitude_rad.sin();
    let longitude_of_ascending_node_rad = ephemeris.omega0
        + (ephemeris.omegadot - EARTH_ROTATION_RAD_PER_S) * tk_s
        - EARTH_ROTATION_RAD_PER_S * ephemeris.toe_s;
    let cos_longitude = longitude_of_ascending_node_rad.cos();
    let sin_longitude = longitude_of_ascending_node_rad.sin();
    let cos_inclination = terms.corrected_inclination_rad.cos();
    let sin_inclination = terms.corrected_inclination_rad.sin();

    [
        x_orbital_m * cos_longitude - y_orbital_m * cos_inclination * sin_longitude,
        x_orbital_m * sin_longitude + y_orbital_m * cos_inclination * cos_longitude,
        y_orbital_m * sin_inclination,
    ]
}

fn satellite_clock_correction(
    ephemeris: &GpsEphemeris,
    transmit_tow_s: f64,
    orbital_terms: OrbitalTerms,
) -> BroadcastClockCorrection {
    let clock_time_s = wrapped_time_difference_s(transmit_tow_s, ephemeris.toc_s);
    let relativistic_s = RELATIVISTIC_CLOCK_CONSTANT
        * ephemeris.e
        * ephemeris.sqrt_a
        * orbital_terms.eccentric_anomaly_rad.sin();
    let drift_s_per_s = ephemeris.af1 + 2.0 * ephemeris.af2 * clock_time_s;
    let bias_s = ephemeris.af0
        + ephemeris.af1 * clock_time_s
        + ephemeris.af2 * clock_time_s * clock_time_s
        + relativistic_s
        - ephemeris.tgd;

    BroadcastClockCorrection {
        bias_s,
        drift_s_per_s,
        drift_rate_s_per_s2: 2.0 * ephemeris.af2,
        relativistic_s,
        group_delay_s: ephemeris.tgd,
    }
}

fn solve_eccentric_anomaly_rad(mean_anomaly_rad: f64, eccentricity: f64) -> f64 {
    let mut eccentric_anomaly_rad = mean_anomaly_rad;
    for _ in 0..12 {
        let next = mean_anomaly_rad + eccentricity * eccentric_anomaly_rad.sin();
        if (next - eccentric_anomaly_rad).abs() < 1.0e-14 {
            eccentric_anomaly_rad = next;
            break;
        }
        eccentric_anomaly_rad = next;
    }
    eccentric_anomaly_rad
}

fn wrapped_time_difference_s(time_s: f64, reference_s: f64) -> f64 {
    let mut delta_s = time_s - reference_s;
    if delta_s > HALF_WEEK_S {
        delta_s -= FULL_WEEK_S;
    } else if delta_s < -HALF_WEEK_S {
        delta_s += FULL_WEEK_S;
    }
    delta_s
}

fn rotate_for_earth_rotation(position_ecef_m: [f64; 3], signal_travel_time_s: f64) -> [f64; 3] {
    let rotation_rad = EARTH_ROTATION_RAD_PER_S * signal_travel_time_s;
    let cos_rotation = rotation_rad.cos();
    let sin_rotation = rotation_rad.sin();
    [
        cos_rotation * position_ecef_m[0] + sin_rotation * position_ecef_m[1],
        -sin_rotation * position_ecef_m[0] + cos_rotation * position_ecef_m[1],
        position_ecef_m[2],
    ]
}

#[cfg(test)]
mod tests {
    use super::{geometric_range_m, pseudorange_from_truth, sat_state_gps_l1ca};
    use bijux_gnss_core::api::{Constellation, SatId};
    use bijux_gnss_nav::api::GpsEphemeris;

    fn sample_ephemeris() -> GpsEphemeris {
        GpsEphemeris {
            sat: SatId { constellation: Constellation::Gps, prn: 3 },
            iodc: 1,
            iode: 1,
            week: 2209,
            sv_health: 0,
            sv_accuracy: Some(2),
            toe_s: 504_000.0,
            toc_s: 504_018.0,
            sqrt_a: 5153.7954775,
            e: 0.01,
            i0: 0.94,
            idot: 0.0,
            omega0: 0.8,
            omegadot: 0.0,
            w: 0.0,
            m0: 0.9,
            delta_n: 0.0,
            cuc: 0.0,
            cus: 0.0,
            crc: 0.0,
            crs: 0.0,
            cic: 0.0,
            cis: 0.0,
            af0: 0.0,
            af1: 0.0,
            af2: 0.0,
            tgd: 0.0,
        }
    }

    #[test]
    fn propagated_state_stays_in_medio_orbit_range() {
        let state = sat_state_gps_l1ca(&sample_ephemeris(), 504_018.0, 0.07);
        let radius_m = geometric_range_m([0.0, 0.0, 0.0], [state.x_m, state.y_m, state.z_m]);

        assert!((25_000_000.0..29_000_000.0).contains(&radius_m));
        assert!(state.clock_correction.bias_s.abs() < 1.0e-3);
    }

    #[test]
    fn pseudorange_from_truth_returns_physical_range_scale() {
        let pseudorange_m = pseudorange_from_truth(
            &sample_ephemeris(),
            [5_000.0, -4_000.0, 3_000.0],
            504_018.07,
            0.0,
        );

        assert!((20_000_000.0..30_000_000.0).contains(&pseudorange_m));
    }
}
