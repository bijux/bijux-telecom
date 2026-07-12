#![allow(missing_docs)]

use bijux_gnss_core::api::{
    ecef_to_geodetic, Constellation, Cycles, GpsTime, Hertz, Llh, LockFlags, Meters, ObsEpoch,
    ObsEpochManifest, ObsMetadata, ObsSatellite, ObsSignalTiming, ObservationEpochDecision,
    ObservationStatus, ReceiverRole, ReceiverSampleTrace, SatId, Seconds, SigId, SignalBand,
    SignalCode, NAV_SOLUTION_MODEL_VERSION, OBSERVATION_DOWNSTREAM_PROFILE_VERSION,
};
use bijux_gnss_nav::api::{
    elevation_azimuth_deg, geodetic_to_ecef, position_broadcast_navigation_from_galileo_navigations,
    sat_state_galileo_e1, GalileoBroadcastNavigationData, GalileoClockCorrection,
    GalileoEphemeris, GalileoIonosphericCorrection, GalileoIonosphericDisturbanceFlags,
    GalileoSignalHealth, GalileoSystemTime,
};
use bijux_gnss_receiver::api::{Navigation, ReceiverPipelineConfig, ReceiverRuntime};

const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;

fn position_error_3d_m(
    actual_ecef_m: (f64, f64, f64),
    truth_ecef_m: (f64, f64, f64),
) -> f64 {
    let dx = actual_ecef_m.0 - truth_ecef_m.0;
    let dy = actual_ecef_m.1 - truth_ecef_m.1;
    let dz = actual_ecef_m.2 - truth_ecef_m.2;
    (dx * dx + dy * dy + dz * dz).sqrt()
}

fn make_galileo_navigation(prn: u8, omega0: f64, m0: f64) -> GalileoBroadcastNavigationData {
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

fn galileo_pseudorange_m(
    navigation: &GalileoBroadcastNavigationData,
    t_rx_s: f64,
    truth_ecef_m: (f64, f64, f64),
    receiver_clock_bias_s: f64,
) -> f64 {
    let mut tau = 0.07;
    let mut pseudorange_m = 0.0;
    for _ in 0..10 {
        let state = sat_state_galileo_e1(navigation, t_rx_s - tau, tau);
        let dx = truth_ecef_m.0 - state.x_m;
        let dy = truth_ecef_m.1 - state.y_m;
        let dz = truth_ecef_m.2 - state.z_m;
        let range_m = (dx * dx + dy * dy + dz * dz).sqrt();
        pseudorange_m = range_m + receiver_clock_bias_s * SPEED_OF_LIGHT_MPS
            - state.clock_correction.bias_s * SPEED_OF_LIGHT_MPS;
        let next_tau = pseudorange_m / SPEED_OF_LIGHT_MPS;
        if (next_tau - tau).abs() < 1.0e-12 {
            break;
        }
        tau = next_tau;
    }
    pseudorange_m
}

fn visible_galileo_navigations(
    t_rx_s: f64,
    truth_ecef_m: (f64, f64, f64),
    receiver_clock_bias_s: f64,
) -> Vec<GalileoBroadcastNavigationData> {
    [
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
    .map(|(prn, omega0, m0)| make_galileo_navigation(prn, omega0, m0))
    .filter(|navigation| {
        let pseudorange_m =
            galileo_pseudorange_m(navigation, t_rx_s, truth_ecef_m, receiver_clock_bias_s);
        let signal_travel_time_s = pseudorange_m / SPEED_OF_LIGHT_MPS;
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
    .collect()
}

fn make_galileo_observation_epoch(
    epoch_idx: u64,
    t_rx_s: f64,
    truth_ecef_m: (f64, f64, f64),
    navigations: &[GalileoBroadcastNavigationData],
    receiver_clock_bias_s: f64,
) -> ObsEpoch {
    let sats = navigations
        .iter()
        .map(|navigation| {
            let pseudorange_m =
                galileo_pseudorange_m(navigation, t_rx_s, truth_ecef_m, receiver_clock_bias_s);
            let signal_travel_time_s = pseudorange_m / SPEED_OF_LIGHT_MPS;
            ObsSatellite {
                signal_id: SigId {
                    sat: navigation.sat,
                    band: SignalBand::E1,
                    code: SignalCode::E1B,
                },
                pseudorange_m: Meters(pseudorange_m),
                pseudorange_var_m2: 9.0,
                carrier_phase_cycles: Cycles(0.0),
                carrier_phase_var_cycles2: 1.0,
                doppler_hz: Hertz(0.0),
                doppler_var_hz2: 4.0,
                cn0_dbhz: 45.0,
                lock_flags: LockFlags {
                    code_lock: true,
                    carrier_lock: true,
                    bit_lock: true,
                    cycle_slip: false,
                },
                multipath_suspect: false,
                observation_status: ObservationStatus::Accepted,
                observation_reject_reasons: Vec::new(),
                elevation_deg: None,
                azimuth_deg: None,
                weight: Some(1.0),
                timing: Some(ObsSignalTiming {
                    signal_travel_time_s: Seconds(signal_travel_time_s),
                    transmit_gps_time: GpsTime { week: 2222, tow_s: t_rx_s - signal_travel_time_s },
                }),
                error_model: None,
                metadata: ObsMetadata::default(),
            }
        })
        .collect();

    ObsEpoch {
        t_rx_s: Seconds(t_rx_s),
        source_time: ReceiverSampleTrace::from_sample_index(epoch_idx, 1_000.0),
        role: ReceiverRole::Rover,
        gps_week: Some(2222),
        tow_s: Some(Seconds(t_rx_s)),
        epoch_idx,
        discontinuity: false,
        valid: true,
        processing_ms: None,
        sats,
        decision: ObservationEpochDecision::Accepted,
        decision_reason: Some("accepted_observables_present".to_string()),
        manifest: Some(ObsEpochManifest {
            version: bijux_gnss_core::api::OBSERVATION_MODEL_VERSION,
            artifact_id: format!("obs-epoch-{epoch_idx:010}"),
            epoch_id: format!("obs-epoch-{epoch_idx:010}-synthetic-galileo"),
            source_epoch_idx: epoch_idx,
            source_sample_index: epoch_idx,
            source_time: ReceiverSampleTrace::from_sample_index(epoch_idx, 1_000.0),
            decision: ObservationEpochDecision::Accepted,
            downstream_profile_version: OBSERVATION_DOWNSTREAM_PROFILE_VERSION,
        }),
    }
}

fn apply_galileo_nequick_bias(
    observation: &ObsEpoch,
    truth_ecef_m: (f64, f64, f64),
    navigations: &[GalileoBroadcastNavigationData],
) -> ObsEpoch {
    let (lat_deg, lon_deg, alt_m) =
        ecef_to_geodetic(truth_ecef_m.0, truth_ecef_m.1, truth_ecef_m.2);
    let receiver = Llh { lat_deg, lon_deg, alt_m };
    let mut biased = observation.clone();

    for satellite in &mut biased.sats {
        let navigation = navigations
            .iter()
            .find(|navigation| navigation.sat == satellite.signal_id.sat)
            .expect("matching galileo navigation");
        let signal_travel_time_s =
            satellite.timing.expect("satellite timing").signal_travel_time_s.0;
        let state =
            sat_state_galileo_e1(navigation, observation.t_rx_s.0 - signal_travel_time_s, signal_travel_time_s);
        let (sat_lat_deg, sat_lon_deg, sat_alt_m) = ecef_to_geodetic(state.x_m, state.y_m, state.z_m);
        let satellite_llh = Llh { lat_deg: sat_lat_deg, lon_deg: sat_lon_deg, alt_m: sat_alt_m };
        let gps_time = GpsTime { week: 2222, tow_s: observation.t_rx_s.0 };
        let delay_m = navigation
            .nequick_delay_m(satellite.signal_id, receiver, satellite_llh, gps_time)
            .expect("galileo nequick delay");
        let delay_s = delay_m / SPEED_OF_LIGHT_MPS;

        satellite.pseudorange_m.0 += delay_m;
        if let Some(timing) = &mut satellite.timing {
            timing.signal_travel_time_s.0 += delay_s;
            timing.transmit_gps_time.tow_s -= delay_s;
        }
    }

    biased
}

#[test]
fn receiver_navigation_uses_galileo_broadcast_payload_to_recover_position() {
    let config = ReceiverPipelineConfig::default();
    let truth_ecef_m = geodetic_to_ecef(37.0, -122.0, 10.0);
    let t_rx_s = 504_018.07;
    let receiver_clock_bias_s = 0.0;
    let navigations = visible_galileo_navigations(t_rx_s, truth_ecef_m, receiver_clock_bias_s);
    assert_eq!(navigations.len(), 4, "expected four visible Galileo satellites");
    let observation = make_galileo_observation_epoch(
        24,
        t_rx_s,
        truth_ecef_m,
        &navigations,
        receiver_clock_bias_s,
    );
    let ionosphere_biased_observation =
        apply_galileo_nequick_bias(&observation, truth_ecef_m, &navigations);
    let mut navigation = Navigation::new(config, ReceiverRuntime::default());

    let solution = navigation
        .solve_epoch_with_galileo_broadcast_navigations(
            &ionosphere_biased_observation,
            &navigations,
        )
        .expect("galileo broadcast navigation solution");

    let error_m = position_error_3d_m(
        (
            solution.ecef_x_m.0,
            solution.ecef_y_m.0,
            solution.ecef_z_m.0,
        ),
        truth_ecef_m,
    );

    assert_eq!(solution.model_version, NAV_SOLUTION_MODEL_VERSION);
    assert!(solution.valid);
    assert!(solution
        .explain_reasons
        .iter()
        .any(|reason| reason == "ionosphere_correction=galileo_nequick"));
    assert!(!solution
        .explain_reasons
        .iter()
        .any(|reason| reason == "ionosphere_uncorrected"));
    assert!(error_m < 5.0, "galileo payload error_m={error_m}");
}

#[test]
fn receiver_navigation_galileo_payload_matches_manual_navigation_entries() {
    let config = ReceiverPipelineConfig::default();
    let truth_ecef_m = geodetic_to_ecef(37.0, -122.0, 10.0);
    let t_rx_s = 504_018.07;
    let receiver_clock_bias_s = 0.0;
    let navigations = visible_galileo_navigations(t_rx_s, truth_ecef_m, receiver_clock_bias_s);
    let manual_navigation =
        position_broadcast_navigation_from_galileo_navigations(&navigations);
    let observation = make_galileo_observation_epoch(
        25,
        t_rx_s,
        truth_ecef_m,
        &navigations,
        receiver_clock_bias_s,
    );
    let ionosphere_biased_observation =
        apply_galileo_nequick_bias(&observation, truth_ecef_m, &navigations);
    let mut payload_navigation = Navigation::new(config.clone(), ReceiverRuntime::default());
    let mut explicit_navigation = Navigation::new(config, ReceiverRuntime::default());

    let payload_solution = payload_navigation
        .solve_epoch_with_galileo_broadcast_navigations(
            &ionosphere_biased_observation,
            &navigations,
        )
        .expect("payload galileo solution");
    let manual_solution = explicit_navigation
        .solve_epoch_with_navigation_data(&ionosphere_biased_observation, &manual_navigation)
        .expect("manual galileo solution");

    assert!((payload_solution.ecef_x_m.0 - manual_solution.ecef_x_m.0).abs() < 1.0e-9);
    assert!((payload_solution.ecef_y_m.0 - manual_solution.ecef_y_m.0).abs() < 1.0e-9);
    assert!((payload_solution.ecef_z_m.0 - manual_solution.ecef_z_m.0).abs() < 1.0e-9);
    assert_eq!(payload_solution.valid, manual_solution.valid);
    assert_eq!(payload_solution.explain_reasons, manual_solution.explain_reasons);
}
