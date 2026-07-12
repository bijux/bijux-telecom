#![allow(missing_docs)]

use bijux_gnss_core::api::{Constellation, GpsTime, Llh, SatId, Seconds, SigId, SignalBand, SignalCode};
use bijux_gnss_nav::api::{
    ecef_to_geodetic, elevation_azimuth_deg, geodetic_to_ecef,
    position_broadcast_navigation_from_galileo_navigations, sat_state_galileo_e1,
    GalileoBroadcastNavigationData, GalileoClockCorrection, GalileoEphemeris,
    GalileoIonosphericCorrection, GalileoIonosphericDisturbanceFlags, GalileoSignalHealth,
    GalileoSystemTime, PositionBroadcastNavigation, PositionObservation, PositionSolver,
};

fn position_error_3d_m(
    actual_ecef_m: (f64, f64, f64),
    truth_ecef_m: (f64, f64, f64),
) -> f64 {
    let dx = actual_ecef_m.0 - truth_ecef_m.0;
    let dy = actual_ecef_m.1 - truth_ecef_m.1;
    let dz = actual_ecef_m.2 - truth_ecef_m.2;
    (dx * dx + dy * dy + dz * dz).sqrt()
}

fn observation_with_signal_id(
    mut observation: PositionObservation,
    signal_id: SigId,
) -> PositionObservation {
    observation.signal_id = Some(signal_id);
    observation
}

fn observation_with_pseudorange_bias(
    mut observation: PositionObservation,
    pseudorange_bias_m: f64,
) -> PositionObservation {
    observation.pseudorange_m += pseudorange_bias_m;
    if let Some(signal_timing) = &mut observation.signal_timing {
        let delay_s = pseudorange_bias_m / 299_792_458.0;
        signal_timing.signal_travel_time_s =
            Seconds(signal_timing.signal_travel_time_s.0 + delay_s);
        signal_timing.transmit_gps_time = signal_timing.transmit_gps_time.offset_seconds(-delay_s);
    }
    observation
}

fn timed_position_observation(sat: SatId, pseudorange_m: f64, t_rx_s: f64) -> PositionObservation {
    let signal_travel_time_s = pseudorange_m / 299_792_458.0;
    PositionObservation {
        sat,
        pseudorange_m,
        doppler_hz: None,
        doppler_var_hz2: None,
        cn0_dbhz: 45.0,
        elevation_deg: None,
        weight: 1.0,
        gps_receive_time: Some(GpsTime { week: 2222, tow_s: t_rx_s }),
        signal_timing: Some(bijux_gnss_core::api::ObsSignalTiming {
            signal_travel_time_s: Seconds(signal_travel_time_s),
            transmit_gps_time: GpsTime { week: 2222, tow_s: t_rx_s - signal_travel_time_s },
        }),
        signal_id: None,
    }
}

fn sample_galileo_navigation(prn: u8, omega0: f64, m0: f64) -> GalileoBroadcastNavigationData {
    GalileoBroadcastNavigationData {
        sat: SatId { constellation: Constellation::Galileo, prn },
        iodnav: prn as u16,
        gst: GalileoSystemTime { week: 2222, tow_s: 504_018 },
        sisa_e1_e5b: 77,
        signal_health: GalileoSignalHealth {
            e5b_signal_health: 0,
            e1b_signal_health: 0,
            e5b_data_valid: true,
            e1b_data_valid: true,
        },
        clock: GalileoClockCorrection {
            t0c_s: 504_018.0,
            af0: 0.0,
            af1: 0.0,
            af2: 0.0,
            bgd_e1_e5a_s: 0.0,
            bgd_e1_e5b_s: 0.0,
        },
        ephemeris: GalileoEphemeris {
            sat: SatId { constellation: Constellation::Galileo, prn },
            iodnav: prn as u16,
            toe_s: 504_000.0,
            sqrt_a: 5_440.612_319,
            e: 0.001_23,
            i0: 0.953,
            idot: -2.1e-10,
            omega0,
            omegadot: -5.8e-9,
            w: -0.37,
            m0,
            delta_n: 4.7e-9,
            cuc: -3.2e-6,
            cus: 4.1e-6,
            crc: 178.0,
            crs: -91.0,
            cic: 1.9e-7,
            cis: -2.4e-7,
        },
        ionosphere: GalileoIonosphericCorrection {
            ai0: 82.0,
            ai1: 0.18,
            ai2: -0.01,
            disturbance_flags: GalileoIonosphericDisturbanceFlags {
                region_1: false,
                region_2: false,
                region_3: false,
                region_4: false,
                region_5: false,
            },
        },
    }
}

fn galileo_pseudorange_from_truth(
    navigation: &GalileoBroadcastNavigationData,
    truth_ecef_m: (f64, f64, f64),
    t_rx_s: f64,
    receiver_clock_bias_s: f64,
    galileo_bias_s: f64,
) -> f64 {
    let c = 299_792_458.0;
    let mut tau = 0.07;
    let mut pseudorange_m = 0.0;
    for _ in 0..10 {
        let state = sat_state_galileo_e1(navigation, t_rx_s - tau, tau);
        let dx = truth_ecef_m.0 - state.x_m;
        let dy = truth_ecef_m.1 - state.y_m;
        let dz = truth_ecef_m.2 - state.z_m;
        let range_m = (dx * dx + dy * dy + dz * dz).sqrt();
        pseudorange_m = range_m + (receiver_clock_bias_s + galileo_bias_s) * c
            - state.clock_correction.bias_s * c;
        let next_tau = pseudorange_m / c;
        if (next_tau - tau).abs() < 1.0e-12 {
            break;
        }
        tau = next_tau;
    }
    pseudorange_m
}

struct GalileoPositionScenario {
    observations: Vec<PositionObservation>,
    navigations: Vec<GalileoBroadcastNavigationData>,
    truth_ecef_m: (f64, f64, f64),
    t_rx_s: f64,
}

fn galileo_four_satellite_position_scenario(
    receiver_clock_bias_s: f64,
) -> GalileoPositionScenario {
    let truth_ecef_m = geodetic_to_ecef(37.0, -122.0, 10.0);
    let t_rx_s = 504_018.07 + receiver_clock_bias_s;
    let navigations = [
        (11, 0.17, 0.24),
        (12, 1.17, 0.84),
        (19, -0.83, 1.52),
        (24, 2.11, -0.41),
        (27, -2.34, 0.61),
        (30, 0.48, -1.14),
        (31, 1.84, 2.27),
        (33, -1.47, -2.02),
        (36, 2.73, 1.06),
        (37, -0.26, 2.88),
        (39, 0.95, -2.41),
        (40, -2.88, -0.73),
    ]
    .into_iter()
    .map(|(prn, omega0, m0)| sample_galileo_navigation(prn, omega0, m0))
    .filter(|navigation| {
        let pseudorange_m = galileo_pseudorange_from_truth(
            navigation,
            truth_ecef_m,
            t_rx_s,
            receiver_clock_bias_s,
            0.0,
        );
        let signal_travel_time_s = pseudorange_m / 299_792_458.0;
        let state =
            sat_state_galileo_e1(navigation, t_rx_s - signal_travel_time_s, signal_travel_time_s);
        let (_azimuth_deg, elevation_deg) = elevation_azimuth_deg(
            truth_ecef_m.0,
            truth_ecef_m.1,
            truth_ecef_m.2,
            state.x_m,
            state.y_m,
            state.z_m,
        );
        elevation_deg > 10.0
    })
    .take(4)
    .collect::<Vec<_>>();
    assert_eq!(navigations.len(), 4, "expected four visible Galileo satellites");
    let observations = navigations
        .iter()
        .map(|navigation| {
            let signal_id = SigId {
                sat: navigation.sat,
                band: SignalBand::E1,
                code: SignalCode::E1B,
            };
            let pseudorange_m = galileo_pseudorange_from_truth(
                navigation,
                truth_ecef_m,
                t_rx_s,
                receiver_clock_bias_s,
                0.0,
            );
            observation_with_signal_id(
                timed_position_observation(navigation.sat, pseudorange_m, t_rx_s),
                signal_id,
            )
        })
        .collect::<Vec<_>>();

    GalileoPositionScenario { observations, navigations, truth_ecef_m, t_rx_s }
}

fn add_galileo_nequick_delay_to_observations(
    observations: &[PositionObservation],
    navigation: &[PositionBroadcastNavigation],
    truth_ecef_m: (f64, f64, f64),
) -> Vec<PositionObservation> {
    let (receiver_lat_deg, receiver_lon_deg, receiver_alt_m) =
        ecef_to_geodetic(truth_ecef_m.0, truth_ecef_m.1, truth_ecef_m.2);
    let receiver = Llh { lat_deg: receiver_lat_deg, lon_deg: receiver_lon_deg, alt_m: receiver_alt_m };

    observations
        .iter()
        .map(|observation| {
            let Some(signal_id) = observation.signal_id else {
                return observation.clone();
            };
            let Some(gps_time) = observation.gps_receive_time else {
                return observation.clone();
            };
            let Some(PositionBroadcastNavigation::Galileo(galileo_navigation)) =
                navigation.iter().find(|entry| entry.sat() == observation.sat)
            else {
                return observation.clone();
            };
            let signal_travel_time_s = observation
                .signal_timing
                .map(|timing| timing.signal_travel_time_s.0)
                .unwrap_or_else(|| observation.pseudorange_m / 299_792_458.0);
            let state = sat_state_galileo_e1(
                galileo_navigation,
                gps_time.tow_s - signal_travel_time_s,
                signal_travel_time_s,
            );
            let (sat_lat_deg, sat_lon_deg, sat_alt_m) =
                ecef_to_geodetic(state.x_m, state.y_m, state.z_m);
            let satellite = Llh { lat_deg: sat_lat_deg, lon_deg: sat_lon_deg, alt_m: sat_alt_m };
            let delay_m = galileo_navigation
                .nequick_delay_m(signal_id, receiver, satellite, gps_time)
                .expect("nequick delay");
            observation_with_pseudorange_bias(observation.clone(), delay_m)
        })
        .collect()
}

#[test]
fn galileo_broadcast_navigation_payload_recovers_single_point_position() {
    let scenario = galileo_four_satellite_position_scenario(0.0);
    let navigation =
        position_broadcast_navigation_from_galileo_navigations(&scenario.navigations);
    let ionosphere_biased_observations = add_galileo_nequick_delay_to_observations(
        &scenario.observations,
        &navigation,
        scenario.truth_ecef_m,
    );

    let corrected_solution = PositionSolver::new()
        .solve_wls_with_navigation_data(
            &ionosphere_biased_observations,
            &navigation,
            scenario.t_rx_s,
        )
        .expect("corrected galileo solution");
    let uncorrected_solution = PositionSolver::new()
        .with_broadcast_ionosphere(false)
        .solve_wls_with_navigation_data(
            &ionosphere_biased_observations,
            &navigation,
            scenario.t_rx_s,
        )
        .expect("uncorrected galileo solution");

    let corrected_error_m = position_error_3d_m(
        (
            corrected_solution.ecef_x_m,
            corrected_solution.ecef_y_m,
            corrected_solution.ecef_z_m,
        ),
        scenario.truth_ecef_m,
    );
    let uncorrected_error_m = position_error_3d_m(
        (
            uncorrected_solution.ecef_x_m,
            uncorrected_solution.ecef_y_m,
            uncorrected_solution.ecef_z_m,
        ),
        scenario.truth_ecef_m,
    );

    assert!(corrected_solution.broadcast_ionosphere_applied);
    assert!(!uncorrected_solution.broadcast_ionosphere_applied);
    assert!(corrected_error_m < 5.0);
    assert!(
        uncorrected_error_m > corrected_error_m + 2.0,
        "corrected_error_m={corrected_error_m} uncorrected_error_m={uncorrected_error_m}"
    );
}

#[test]
fn galileo_broadcast_navigation_payload_matches_manual_navigation_entries() {
    let scenario = galileo_four_satellite_position_scenario(0.0);
    let payload_navigation =
        position_broadcast_navigation_from_galileo_navigations(&scenario.navigations);
    let manual_navigation = scenario
        .navigations
        .iter()
        .cloned()
        .map(PositionBroadcastNavigation::Galileo)
        .collect::<Vec<_>>();
    let ionosphere_biased_observations = add_galileo_nequick_delay_to_observations(
        &scenario.observations,
        &payload_navigation,
        scenario.truth_ecef_m,
    );

    let payload_solution = PositionSolver::new()
        .solve_wls_with_navigation_data(
            &ionosphere_biased_observations,
            &payload_navigation,
            scenario.t_rx_s,
        )
        .expect("payload galileo solution");
    let manual_solution = PositionSolver::new()
        .solve_wls_with_navigation_data(
            &ionosphere_biased_observations,
            &manual_navigation,
            scenario.t_rx_s,
        )
        .expect("manual galileo solution");

    assert!((payload_solution.ecef_x_m - manual_solution.ecef_x_m).abs() < 1.0e-9);
    assert!((payload_solution.ecef_y_m - manual_solution.ecef_y_m).abs() < 1.0e-9);
    assert!((payload_solution.ecef_z_m - manual_solution.ecef_z_m).abs() < 1.0e-9);
    assert_eq!(
        payload_solution.broadcast_ionosphere_applied,
        manual_solution.broadcast_ionosphere_applied
    );
}
