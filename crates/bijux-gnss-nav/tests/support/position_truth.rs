#![allow(dead_code)]
#![allow(missing_docs)]

use bijux_gnss_core::api::{Constellation, GpsTime, ObsSignalTiming, SatId, Seconds};
use bijux_gnss_nav::api::{
    geodetic_to_ecef, sat_state_gps_l1ca, GpsEphemeris, PositionObservation,
};

const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;

#[derive(Debug, Clone)]
pub struct SyntheticPositionScenario {
    pub ephemerides: Vec<GpsEphemeris>,
    pub observations: Vec<PositionObservation>,
    pub truth_ecef_m: (f64, f64, f64),
    pub receiver_clock_bias_s: f64,
    pub t_rx_s: f64,
}

pub fn four_satellite_position_scenario(
    receiver_clock_bias_s: f64,
) -> SyntheticPositionScenario {
    let ephemerides = sample_ephemerides();
    let truth_ecef_m = geodetic_to_ecef(37.0, -122.0, 10.0);
    let t_rx_s = 504_018.07 + receiver_clock_bias_s;
    let observations = ephemerides
        .iter()
        .map(|eph| {
            timed_position_observation_from_truth(
                eph,
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
    vec![
        sample_ephemeris(1, 0.0, 0.0),
        sample_ephemeris(2, 0.8, 0.9),
        sample_ephemeris(3, 1.6, 1.8),
        sample_ephemeris(4, 2.4, 2.7),
    ]
}

pub fn sample_ephemeris(prn: u8, omega0: f64, m0: f64) -> GpsEphemeris {
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
        af0: 0.0,
        af1: 0.0,
        af2: 0.0,
        tgd: 0.0,
    }
}

pub fn timed_position_observation(sat: SatId, pseudorange_m: f64, t_rx_s: f64) -> PositionObservation {
    let signal_travel_time_s = pseudorange_m / SPEED_OF_LIGHT_MPS;
    PositionObservation {
        sat,
        pseudorange_m,
        cn0_dbhz: 45.0,
        elevation_deg: None,
        weight: 1.0,
        gps_receive_time: Some(GpsTime { week: 0, tow_s: t_rx_s }),
        signal_timing: Some(ObsSignalTiming {
            signal_travel_time_s: Seconds(signal_travel_time_s),
            transmit_gps_time: GpsTime { week: 0, tow_s: t_rx_s - signal_travel_time_s },
        }),
    }
}

pub fn timed_position_observation_from_truth(
    eph: &GpsEphemeris,
    truth_ecef_m: (f64, f64, f64),
    t_rx_s: f64,
    receiver_clock_bias_s: f64,
) -> PositionObservation {
    let pseudorange_m = pseudorange_from_truth(eph, truth_ecef_m, t_rx_s, receiver_clock_bias_s);
    timed_position_observation(eph.sat, pseudorange_m, t_rx_s)
}

pub fn pseudorange_from_truth(
    eph: &GpsEphemeris,
    truth_ecef_m: (f64, f64, f64),
    t_rx_s: f64,
    receiver_clock_bias_s: f64,
) -> f64 {
    let mut tau = 0.07;
    let mut pseudorange_m = 0.0;
    for _ in 0..10 {
        let state = sat_state_gps_l1ca(eph, t_rx_s - tau, tau);
        let dx = truth_ecef_m.0 - state.x_m;
        let dy = truth_ecef_m.1 - state.y_m;
        let dz = truth_ecef_m.2 - state.z_m;
        let range_m = (dx * dx + dy * dy + dz * dz).sqrt();
        pseudorange_m = range_m
            + receiver_clock_bias_s * SPEED_OF_LIGHT_MPS
            - state.clock_correction.bias_s * SPEED_OF_LIGHT_MPS;
        let next_tau = pseudorange_m / SPEED_OF_LIGHT_MPS;
        if (next_tau - tau).abs() < 1.0e-12 {
            break;
        }
        tau = next_tau;
    }
    pseudorange_m
}

pub fn iterative_pseudorange_residual_m(
    eph: &GpsEphemeris,
    observation: &PositionObservation,
    receiver_ecef_m: (f64, f64, f64),
    receiver_clock_bias_s: f64,
    t_rx_s: f64,
) -> f64 {
    let receive_tow_s = observation
        .gps_receive_time
        .map(|gps_time| gps_time.tow_s)
        .unwrap_or(t_rx_s);
    let mut tau = observation
        .signal_timing
        .map(|timing| timing.signal_travel_time_s.0)
        .unwrap_or(observation.pseudorange_m / SPEED_OF_LIGHT_MPS);
    let mut predicted_pseudorange_m = 0.0;
    for _ in 0..10 {
        let state = sat_state_gps_l1ca(eph, receive_tow_s - tau, tau);
        let dx = receiver_ecef_m.0 - state.x_m;
        let dy = receiver_ecef_m.1 - state.y_m;
        let dz = receiver_ecef_m.2 - state.z_m;
        let range_m = (dx * dx + dy * dy + dz * dz).sqrt();
        predicted_pseudorange_m = range_m
            + receiver_clock_bias_s * SPEED_OF_LIGHT_MPS
            - state.clock_correction.bias_s * SPEED_OF_LIGHT_MPS;
        let next_tau = predicted_pseudorange_m / SPEED_OF_LIGHT_MPS;
        if (next_tau - tau).abs() < 1.0e-12 {
            break;
        }
        tau = next_tau;
    }
    observation.pseudorange_m - predicted_pseudorange_m
}
