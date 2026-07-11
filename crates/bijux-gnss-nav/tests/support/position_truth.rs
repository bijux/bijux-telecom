#![allow(dead_code)]
#![allow(missing_docs)]

use bijux_gnss_core::api::{Constellation, GpsTime, Llh, ObsSignalTiming, SatId, Seconds};
use bijux_gnss_nav::api::{
    ecef_to_geodetic, elevation_azimuth_deg, geodetic_to_ecef, sat_state_gps_l1ca,
    sat_state_gps_l1ca_from_observation, GpsEphemeris, KlobucharCoefficients, KlobucharModel,
    PositionObservation, SaastamoinenModel, TroposphereModel,
};
use bijux_gnss_nav::api::IonosphereModel;

const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;

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

pub fn four_satellite_position_scenario(
    receiver_clock_bias_s: f64,
) -> SyntheticPositionScenario {
    four_satellite_position_scenario_with_ephemerides(
        receiver_clock_bias_s,
        sample_ephemerides(),
    )
}

pub fn four_satellite_position_scenario_with_ephemerides(
    receiver_clock_bias_s: f64,
    ephemerides: Vec<GpsEphemeris>,
) -> SyntheticPositionScenario {
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
    sample_ephemerides_with_clock_parameters(&[])
}

pub fn sample_ephemerides_with_clock_parameters(
    clock_parameters_by_prn: &[(u8, BroadcastClockParameters)],
) -> Vec<GpsEphemeris> {
    let clock_parameters_by_prn = clock_parameters_by_prn
        .iter()
        .copied()
        .collect::<std::collections::BTreeMap<_, _>>();
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
    let (lat_deg, lon_deg, alt_m) =
        ecef_to_geodetic(receiver_ecef_m.0, receiver_ecef_m.1, receiver_ecef_m.2);
    let receiver = Llh { lat_deg, lon_deg, alt_m };
    let model = KlobucharModel::new(klobuchar);

    observations
        .iter()
        .map(|observation| {
            let ephemeris = ephemerides
                .iter()
                .find(|ephemeris| ephemeris.sat == observation.sat)
                .expect("matching ephemeris");
            let state = sat_state_gps_l1ca_from_observation(
                ephemeris,
                t_rx_s,
                observation.pseudorange_m,
                observation.signal_timing,
            );
            let (azimuth_deg, elevation_deg) = elevation_azimuth_deg(
                receiver_ecef_m.0,
                receiver_ecef_m.1,
                receiver_ecef_m.2,
                state.x_m,
                state.y_m,
                state.z_m,
            );
            let delay_m = model.delay_m(receiver, azimuth_deg, elevation_deg, Seconds(t_rx_s));
            let delay_s = delay_m / SPEED_OF_LIGHT_MPS;
            let mut biased = observation.clone();
            biased.pseudorange_m += delay_m;
            if let Some(signal_timing) = &mut biased.signal_timing {
                signal_timing.signal_travel_time_s = Seconds(signal_timing.signal_travel_time_s.0 + delay_s);
                signal_timing.transmit_gps_time =
                    signal_timing.transmit_gps_time.offset_seconds(-delay_s);
            }
            biased
        })
        .collect()
}

pub fn add_saastamoinen_delay_to_observations(
    observations: &[PositionObservation],
    ephemerides: &[GpsEphemeris],
    receiver_ecef_m: (f64, f64, f64),
    t_rx_s: f64,
) -> Vec<PositionObservation> {
    let (lat_deg, lon_deg, alt_m) =
        ecef_to_geodetic(receiver_ecef_m.0, receiver_ecef_m.1, receiver_ecef_m.2);
    let receiver = Llh { lat_deg, lon_deg, alt_m };
    let model = SaastamoinenModel;

    observations
        .iter()
        .map(|observation| {
            let ephemeris = ephemerides
                .iter()
                .find(|ephemeris| ephemeris.sat == observation.sat)
                .expect("matching ephemeris");
            let state = sat_state_gps_l1ca_from_observation(
                ephemeris,
                t_rx_s,
                observation.pseudorange_m,
                observation.signal_timing,
            );
            let (_azimuth_deg, elevation_deg) = elevation_azimuth_deg(
                receiver_ecef_m.0,
                receiver_ecef_m.1,
                receiver_ecef_m.2,
                state.x_m,
                state.y_m,
                state.z_m,
            );
            let delay_m = model.delay_m(receiver, elevation_deg, Seconds(t_rx_s));
            let delay_s = delay_m / SPEED_OF_LIGHT_MPS;
            let mut biased = observation.clone();
            biased.pseudorange_m += delay_m;
            if let Some(signal_timing) = &mut biased.signal_timing {
                signal_timing.signal_travel_time_s = Seconds(signal_timing.signal_travel_time_s.0 + delay_s);
                signal_timing.transmit_gps_time =
                    signal_timing.transmit_gps_time.offset_seconds(-delay_s);
            }
            biased
        })
        .collect()
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
    iterative_pseudorange_residual_with_earth_rotation_mode_m(
        eph,
        observation,
        receiver_ecef_m,
        receiver_clock_bias_s,
        t_rx_s,
        true,
    )
}

pub fn iterative_pseudorange_residual_without_earth_rotation_m(
    eph: &GpsEphemeris,
    observation: &PositionObservation,
    receiver_ecef_m: (f64, f64, f64),
    receiver_clock_bias_s: f64,
    t_rx_s: f64,
) -> f64 {
    iterative_pseudorange_residual_with_earth_rotation_mode_m(
        eph,
        observation,
        receiver_ecef_m,
        receiver_clock_bias_s,
        t_rx_s,
        false,
    )
}

fn iterative_pseudorange_residual_with_earth_rotation_mode_m(
    eph: &GpsEphemeris,
    observation: &PositionObservation,
    receiver_ecef_m: (f64, f64, f64),
    receiver_clock_bias_s: f64,
    t_rx_s: f64,
    apply_earth_rotation: bool,
) -> f64 {
    let receive_tow_s = observation
        .gps_receive_time
        .map(|gps_time| gps_time.tow_s)
        .unwrap_or(t_rx_s);
    let mut tau = observation
        .signal_timing
        .map(|timing| timing.signal_travel_time_s.0)
        .unwrap_or(observation.pseudorange_m / SPEED_OF_LIGHT_MPS);
    let predicted_pseudorange_m = iterative_predicted_pseudorange_m(
        eph,
        receiver_ecef_m,
        receiver_clock_bias_s,
        receive_tow_s,
        &mut tau,
        apply_earth_rotation,
    );
    observation.pseudorange_m - predicted_pseudorange_m
}

fn iterative_predicted_pseudorange_m(
    eph: &GpsEphemeris,
    receiver_ecef_m: (f64, f64, f64),
    receiver_clock_bias_s: f64,
    receive_tow_s: f64,
    tau_s: &mut f64,
    apply_earth_rotation: bool,
) -> f64 {
    let mut predicted_pseudorange_m = 0.0;
    for _ in 0..10 {
        let state = sat_state_gps_l1ca(
            eph,
            receive_tow_s - *tau_s,
            if apply_earth_rotation { *tau_s } else { 0.0 },
        );
        let dx = receiver_ecef_m.0 - state.x_m;
        let dy = receiver_ecef_m.1 - state.y_m;
        let dz = receiver_ecef_m.2 - state.z_m;
        let range_m = (dx * dx + dy * dy + dz * dz).sqrt();
        predicted_pseudorange_m = range_m
            + receiver_clock_bias_s * SPEED_OF_LIGHT_MPS
            - state.clock_correction.bias_s * SPEED_OF_LIGHT_MPS;
        let next_tau = predicted_pseudorange_m / SPEED_OF_LIGHT_MPS;
        if (next_tau - *tau_s).abs() < 1.0e-12 {
            break;
        }
        *tau_s = next_tau;
    }
    predicted_pseudorange_m
}
