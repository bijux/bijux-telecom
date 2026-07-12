#![allow(missing_docs)]

use bijux_gnss_core::api::{
    Constellation, Cycles, GpsTime, Hertz, LockFlags, Meters, ObsEpoch, ObsMetadata, ObsSatellite,
    ObsSignalTiming, ObservationEpochDecision, ObservationStatus, ReceiverRole, NavRefusalClass,
    ReceiverSampleTrace, SatId, Seconds, SigId, SignalBand, SignalCode,
};
use bijux_gnss_receiver::api::{
    nav::{
        geodetic_to_ecef, position_broadcast_navigation_from_beidou_navigations,
        position_broadcast_navigation_from_gps_ephemerides, sat_state_beidou_b1i,
        sat_state_galileo_e1, sat_state_gps_l1ca, BeidouBroadcastNavigationData,
        BeidouClockCorrection, BeidouEphemeris, BeidouIonosphericCorrection,
        BeidouSignalHealth, BeidouSystemTime, GalileoBroadcastNavigationData,
        GalileoClockCorrection, GalileoEphemeris, GalileoIonosphericCorrection,
        GalileoIonosphericDisturbanceFlags, GalileoSignalHealth, GalileoSystemTime, GpsEphemeris,
        PositionBroadcastNavigation,
    },
    Navigation, ReceiverPipelineConfig, ReceiverRuntime,
};

fn make_gps_ephemeris(prn: u8, omega0: f64, m0: f64, t_ref_s: f64) -> GpsEphemeris {
    GpsEphemeris {
        sat: SatId { constellation: Constellation::Gps, prn },
        iodc: 1,
        iode: 1,
        week: 2209,
        sv_health: 0,
        toe_s: t_ref_s,
        toc_s: t_ref_s,
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

fn make_galileo_navigation(
    prn: u8,
    omega0: f64,
    m0: f64,
    t_ref_s: f64,
) -> GalileoBroadcastNavigationData {
    GalileoBroadcastNavigationData {
        sat: SatId { constellation: Constellation::Galileo, prn },
        iodnav: prn as u16,
        gst: GalileoSystemTime { week: 2222, tow_s: t_ref_s as u32 },
        sisa_e1_e5b: 77,
        signal_health: GalileoSignalHealth {
            e5b_signal_health: 0,
            e1b_signal_health: 0,
            e5b_data_valid: true,
            e1b_data_valid: true,
        },
        clock: GalileoClockCorrection {
            t0c_s: t_ref_s,
            af0: 0.0,
            af1: 0.0,
            af2: 0.0,
            bgd_e1_e5a_s: 0.0,
            bgd_e1_e5b_s: 0.0,
        },
        ephemeris: GalileoEphemeris {
            sat: SatId { constellation: Constellation::Galileo, prn },
            iodnav: prn as u16,
            toe_s: t_ref_s,
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
            ai0: 0.0,
            ai1: 0.0,
            ai2: 0.0,
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

fn make_beidou_navigation(
    prn: u8,
    omega0: f64,
    m0: f64,
    t_ref_s: f64,
) -> BeidouBroadcastNavigationData {
    BeidouBroadcastNavigationData {
        sat: SatId { constellation: Constellation::Beidou, prn },
        bdt: BeidouSystemTime { week: 888, sow_s: t_ref_s as u32 },
        urai: 0,
        signal_health: BeidouSignalHealth { autonomous_satellite_good: true },
        clock: BeidouClockCorrection {
            toc_s: t_ref_s,
            aodc: prn,
            af0: 0.0,
            af1: 0.0,
            af2: 0.0,
            tgd1_s: 0.0,
            tgd2_s: 0.0,
        },
        ephemeris: BeidouEphemeris {
            sat: SatId { constellation: Constellation::Beidou, prn },
            aode: prn,
            toe_s: t_ref_s,
            sqrt_a: 5_282.61,
            e: 0.0021,
            i0: 0.962,
            idot: -1.4e-10,
            omega0,
            omegadot: -7.9e-9,
            w: -0.41,
            m0,
            delta_n: 3.4e-9,
            cuc: -1.2e-6,
            cus: 3.1e-6,
            crc: 146.0,
            crs: -84.0,
            cic: 1.1e-7,
            cis: -1.7e-7,
        },
        ionosphere: BeidouIonosphericCorrection {
            alpha0: 0.0,
            alpha1: 0.0,
            alpha2: 0.0,
            alpha3: 0.0,
            beta0: 0.0,
            beta1: 0.0,
            beta2: 0.0,
            beta3: 0.0,
        },
    }
}

fn gps_pseudorange_m(
    ephemeris: &GpsEphemeris,
    t_rx_s: f64,
    truth_ecef_m: (f64, f64, f64),
    receiver_clock_bias_s: f64,
) -> f64 {
    let mut tau = 0.07;
    let mut pseudorange_m = 0.0;
    for _ in 0..10 {
        let state = sat_state_gps_l1ca(ephemeris, t_rx_s - tau, tau);
        let dx = truth_ecef_m.0 - state.x_m;
        let dy = truth_ecef_m.1 - state.y_m;
        let dz = truth_ecef_m.2 - state.z_m;
        let range_m = (dx * dx + dy * dy + dz * dz).sqrt();
        pseudorange_m = range_m + receiver_clock_bias_s * 299_792_458.0
            - state.clock_correction.bias_s * 299_792_458.0;
        let next_tau = pseudorange_m / 299_792_458.0;
        if (next_tau - tau).abs() < 1.0e-12 {
            break;
        }
        tau = next_tau;
    }
    pseudorange_m
}

fn galileo_pseudorange_m(
    navigation: &GalileoBroadcastNavigationData,
    t_rx_s: f64,
    truth_ecef_m: (f64, f64, f64),
    receiver_clock_bias_s: f64,
    galileo_bias_s: f64,
) -> f64 {
    let mut tau = 0.07;
    let mut pseudorange_m = 0.0;
    for _ in 0..10 {
        let state = sat_state_galileo_e1(navigation, t_rx_s - tau, tau);
        let dx = truth_ecef_m.0 - state.x_m;
        let dy = truth_ecef_m.1 - state.y_m;
        let dz = truth_ecef_m.2 - state.z_m;
        let range_m = (dx * dx + dy * dy + dz * dz).sqrt();
        pseudorange_m = range_m + (receiver_clock_bias_s + galileo_bias_s) * 299_792_458.0
            - state.clock_correction.bias_s * 299_792_458.0;
        let next_tau = pseudorange_m / 299_792_458.0;
        if (next_tau - tau).abs() < 1.0e-12 {
            break;
        }
        tau = next_tau;
    }
    pseudorange_m
}

fn beidou_pseudorange_m(
    navigation: &BeidouBroadcastNavigationData,
    t_rx_s: f64,
    truth_ecef_m: (f64, f64, f64),
    receiver_clock_bias_s: f64,
    beidou_bias_s: f64,
) -> f64 {
    let mut tau = 0.07;
    let mut pseudorange_m = 0.0;
    for _ in 0..10 {
        let state = sat_state_beidou_b1i(navigation, t_rx_s - tau, tau);
        let dx = truth_ecef_m.0 - state.x_m;
        let dy = truth_ecef_m.1 - state.y_m;
        let dz = truth_ecef_m.2 - state.z_m;
        let range_m = (dx * dx + dy * dy + dz * dz).sqrt();
        pseudorange_m = range_m + (receiver_clock_bias_s + beidou_bias_s) * 299_792_458.0
            - state.clock_correction.bias_s * 299_792_458.0;
        let next_tau = pseudorange_m / 299_792_458.0;
        if (next_tau - tau).abs() < 1.0e-12 {
            break;
        }
        tau = next_tau;
    }
    pseudorange_m
}

fn synthetic_satellite(
    sat: SatId,
    band: SignalBand,
    code: SignalCode,
    pseudorange_m: f64,
    t_rx_s: f64,
) -> ObsSatellite {
    let signal_travel_time_s = pseudorange_m / 299_792_458.0;
    ObsSatellite {
        signal_id: SigId { sat, band, code },
        pseudorange_m: Meters(pseudorange_m),
        pseudorange_var_m2: 4.0,
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
        elevation_deg: Some(45.0),
        azimuth_deg: Some(0.0),
        weight: Some(1.0),
        timing: Some(ObsSignalTiming {
            signal_travel_time_s: Seconds(signal_travel_time_s),
            transmit_gps_time: GpsTime { week: 0, tow_s: t_rx_s - signal_travel_time_s },
        }),
        error_model: None,
        metadata: ObsMetadata::default(),
    }
}

#[test]
fn public_navigation_api_solves_mixed_gps_galileo_epoch() {
    let config = ReceiverPipelineConfig::default();
    let runtime = ReceiverRuntime::default();
    let mut navigation_engine = Navigation::new(config, runtime);
    let truth_ecef_m = geodetic_to_ecef(37.0, -122.0, 25.0);
    let t_rx_s = 100_000.0;
    let receiver_clock_bias_s = 2.75e-4;
    let galileo_bias_s = -1.15e-6;
    let gps_ephemerides = vec![
        make_gps_ephemeris(1, 0.0, 0.0, t_rx_s),
        make_gps_ephemeris(2, 0.8, 0.9, t_rx_s),
        make_gps_ephemeris(3, 1.6, 1.8, t_rx_s),
        make_gps_ephemeris(4, 2.4, 2.7, t_rx_s),
    ];
    let galileo_navigation = vec![
        make_galileo_navigation(19, 1.17, 0.84, t_rx_s),
        make_galileo_navigation(24, -0.83, 1.52, t_rx_s),
    ];

    let mut sats = gps_ephemerides
        .iter()
        .map(|ephemeris| {
            synthetic_satellite(
                ephemeris.sat,
                SignalBand::L1,
                SignalCode::Ca,
                gps_pseudorange_m(ephemeris, t_rx_s, truth_ecef_m, receiver_clock_bias_s),
                t_rx_s,
            )
        })
        .collect::<Vec<_>>();
    sats.extend(galileo_navigation.iter().map(|navigation| {
        synthetic_satellite(
            navigation.sat,
            SignalBand::E1,
            SignalCode::E1B,
            galileo_pseudorange_m(
                navigation,
                t_rx_s,
                truth_ecef_m,
                receiver_clock_bias_s,
                galileo_bias_s,
            ),
            t_rx_s,
        )
    }));

    let obs = ObsEpoch {
        t_rx_s: Seconds(t_rx_s),
        source_time: ReceiverSampleTrace::from_sample_index(0, 1_000.0),
        gps_week: Some(0),
        tow_s: Some(Seconds(t_rx_s)),
        epoch_idx: 1,
        discontinuity: false,
        valid: true,
        processing_ms: None,
        role: ReceiverRole::Rover,
        sats,
        decision: ObservationEpochDecision::Accepted,
        decision_reason: Some("accepted_observables_present".to_string()),
        manifest: None,
    };

    let mut navigation_data = position_broadcast_navigation_from_gps_ephemerides(&gps_ephemerides);
    navigation_data
        .extend(galileo_navigation.iter().cloned().map(PositionBroadcastNavigation::Galileo));

    let solution = navigation_engine
        .solve_epoch_with_navigation_data(&obs, &navigation_data)
        .expect("mixed solution");

    assert!(solution.valid);
    assert_eq!(solution.refusal_class, None);
    assert_eq!(solution.used_sat_count, 6);
    assert_eq!(solution.isb.len(), 1);
    assert_eq!(solution.isb[0].constellation, Constellation::Galileo);
    assert_eq!(solution.isb[0].bias_s.0.signum(), galileo_bias_s.signum());
    assert!((solution.isb[0].bias_s.0 - galileo_bias_s).abs() < 5.0e-8);
    assert!((solution.clock_bias_s.0 - receiver_clock_bias_s).abs() < 5.0e-8);
}

#[test]
fn public_navigation_api_solves_mixed_gps_beidou_epoch() {
    let config = ReceiverPipelineConfig::default();
    let runtime = ReceiverRuntime::default();
    let mut navigation_engine = Navigation::new(config, runtime);
    let truth_ecef_m = geodetic_to_ecef(37.0, -122.0, 25.0);
    let t_rx_s = 100_000.0;
    let receiver_clock_bias_s = 2.75e-4;
    let beidou_bias_s = 9.25e-7;
    let gps_ephemerides = vec![
        make_gps_ephemeris(1, 0.0, 0.0, t_rx_s),
        make_gps_ephemeris(2, 0.8, 0.9, t_rx_s),
        make_gps_ephemeris(3, 1.6, 1.8, t_rx_s),
        make_gps_ephemeris(4, 2.4, 2.7, t_rx_s),
    ];
    let beidou_navigation = vec![
        make_beidou_navigation(11, 0.77, 0.53, t_rx_s),
        make_beidou_navigation(12, -1.09, 1.31, t_rx_s),
    ];

    let mut sats = gps_ephemerides
        .iter()
        .map(|ephemeris| {
            synthetic_satellite(
                ephemeris.sat,
                SignalBand::L1,
                SignalCode::Ca,
                gps_pseudorange_m(ephemeris, t_rx_s, truth_ecef_m, receiver_clock_bias_s),
                t_rx_s,
            )
        })
        .collect::<Vec<_>>();
    sats.extend(beidou_navigation.iter().map(|navigation| {
        synthetic_satellite(
            navigation.sat,
            SignalBand::B1,
            SignalCode::B1I,
            beidou_pseudorange_m(
                navigation,
                t_rx_s,
                truth_ecef_m,
                receiver_clock_bias_s,
                beidou_bias_s,
            ),
            t_rx_s,
        )
    }));

    let obs = ObsEpoch {
        t_rx_s: Seconds(t_rx_s),
        source_time: ReceiverSampleTrace::from_sample_index(0, 1_000.0),
        gps_week: Some(0),
        tow_s: Some(Seconds(t_rx_s)),
        epoch_idx: 1,
        discontinuity: false,
        valid: true,
        processing_ms: None,
        role: ReceiverRole::Rover,
        sats,
        decision: ObservationEpochDecision::Accepted,
        decision_reason: Some("accepted_observables_present".to_string()),
        manifest: None,
    };

    let mut navigation_data = position_broadcast_navigation_from_gps_ephemerides(&gps_ephemerides);
    navigation_data.extend(position_broadcast_navigation_from_beidou_navigations(
        &beidou_navigation,
    ));

    let solution = navigation_engine
        .solve_epoch_with_navigation_data(&obs, &navigation_data)
        .expect("mixed solution");

    assert!(solution.valid);
    assert_eq!(solution.refusal_class, None);
    assert_eq!(solution.used_sat_count, 6);
    assert_eq!(solution.isb.len(), 1);
    assert_eq!(solution.isb[0].constellation, Constellation::Beidou);
    assert_eq!(solution.isb[0].bias_s.0.signum(), beidou_bias_s.signum());
    assert!((solution.isb[0].bias_s.0 - beidou_bias_s).abs() < 5.0e-8);
    assert!((solution.clock_bias_s.0 - receiver_clock_bias_s).abs() < 5.0e-8);
}

#[test]
fn public_navigation_api_solves_mixed_gps_galileo_beidou_epoch() {
    let config = ReceiverPipelineConfig::default();
    let runtime = ReceiverRuntime::default();
    let mut navigation_engine = Navigation::new(config, runtime);
    let truth_ecef_m = geodetic_to_ecef(37.0, -122.0, 25.0);
    let t_rx_s = 100_000.0;
    let receiver_clock_bias_s = 2.75e-4;
    let galileo_bias_s = -1.15e-6;
    let beidou_bias_s = 9.25e-7;
    let gps_ephemerides = vec![
        make_gps_ephemeris(1, 0.0, 0.0, t_rx_s),
        make_gps_ephemeris(2, 0.8, 0.9, t_rx_s),
        make_gps_ephemeris(3, 1.6, 1.8, t_rx_s),
        make_gps_ephemeris(4, 2.4, 2.7, t_rx_s),
    ];
    let galileo_navigation = vec![
        make_galileo_navigation(19, 1.17, 0.84, t_rx_s),
        make_galileo_navigation(24, -0.83, 1.52, t_rx_s),
    ];
    let beidou_navigation = vec![
        make_beidou_navigation(11, 0.77, 0.53, t_rx_s),
        make_beidou_navigation(12, -1.09, 1.31, t_rx_s),
    ];

    let mut sats = gps_ephemerides
        .iter()
        .map(|ephemeris| {
            synthetic_satellite(
                ephemeris.sat,
                SignalBand::L1,
                SignalCode::Ca,
                gps_pseudorange_m(ephemeris, t_rx_s, truth_ecef_m, receiver_clock_bias_s),
                t_rx_s,
            )
        })
        .collect::<Vec<_>>();
    sats.extend(galileo_navigation.iter().map(|navigation| {
        synthetic_satellite(
            navigation.sat,
            SignalBand::E1,
            SignalCode::E1B,
            galileo_pseudorange_m(
                navigation,
                t_rx_s,
                truth_ecef_m,
                receiver_clock_bias_s,
                galileo_bias_s,
            ),
            t_rx_s,
        )
    }));
    sats.extend(beidou_navigation.iter().map(|navigation| {
        synthetic_satellite(
            navigation.sat,
            SignalBand::B1,
            SignalCode::B1I,
            beidou_pseudorange_m(
                navigation,
                t_rx_s,
                truth_ecef_m,
                receiver_clock_bias_s,
                beidou_bias_s,
            ),
            t_rx_s,
        )
    }));

    let obs = ObsEpoch {
        t_rx_s: Seconds(t_rx_s),
        source_time: ReceiverSampleTrace::from_sample_index(0, 1_000.0),
        gps_week: Some(0),
        tow_s: Some(Seconds(t_rx_s)),
        epoch_idx: 1,
        discontinuity: false,
        valid: true,
        processing_ms: None,
        role: ReceiverRole::Rover,
        sats,
        decision: ObservationEpochDecision::Accepted,
        decision_reason: Some("accepted_observables_present".to_string()),
        manifest: None,
    };

    let mut navigation_data = position_broadcast_navigation_from_gps_ephemerides(&gps_ephemerides);
    navigation_data
        .extend(galileo_navigation.iter().cloned().map(PositionBroadcastNavigation::Galileo));
    navigation_data.extend(position_broadcast_navigation_from_beidou_navigations(
        &beidou_navigation,
    ));

    let solution = navigation_engine
        .solve_epoch_with_navigation_data(&obs, &navigation_data)
        .expect("mixed solution");

    assert!(solution.valid);
    assert_eq!(solution.refusal_class, None);
    assert_eq!(solution.used_sat_count, 8);
    assert_eq!(solution.isb.len(), 2);
    let galileo_isb = solution
        .isb
        .iter()
        .find(|bias| bias.constellation == Constellation::Galileo)
        .expect("galileo inter-system bias");
    let beidou_isb = solution
        .isb
        .iter()
        .find(|bias| bias.constellation == Constellation::Beidou)
        .expect("beidou inter-system bias");
    assert_eq!(galileo_isb.bias_s.0.signum(), galileo_bias_s.signum());
    assert_eq!(beidou_isb.bias_s.0.signum(), beidou_bias_s.signum());
    assert!((galileo_isb.bias_s.0 - galileo_bias_s).abs() < 5.0e-8);
    assert!((beidou_isb.bias_s.0 - beidou_bias_s).abs() < 5.0e-8);
    assert!((solution.clock_bias_s.0 - receiver_clock_bias_s).abs() < 5.0e-8);
}

#[test]
fn public_navigation_api_refuses_mixed_gps_glonass_epoch_without_time_handling() {
    let config = ReceiverPipelineConfig::default();
    let runtime = ReceiverRuntime::default();
    let mut navigation_engine = Navigation::new(config, runtime);
    let truth_ecef_m = geodetic_to_ecef(37.0, -122.0, 25.0);
    let t_rx_s = 100_000.0;
    let receiver_clock_bias_s = 2.75e-4;
    let gps_ephemerides = vec![
        make_gps_ephemeris(1, 0.0, 0.0, t_rx_s),
        make_gps_ephemeris(2, 0.8, 0.9, t_rx_s),
        make_gps_ephemeris(3, 1.6, 1.8, t_rx_s),
        make_gps_ephemeris(4, 2.4, 2.7, t_rx_s),
    ];

    let mut sats = gps_ephemerides
        .iter()
        .map(|ephemeris| {
            synthetic_satellite(
                ephemeris.sat,
                SignalBand::L1,
                SignalCode::Ca,
                gps_pseudorange_m(ephemeris, t_rx_s, truth_ecef_m, receiver_clock_bias_s),
                t_rx_s,
            )
        })
        .collect::<Vec<_>>();
    sats.push(synthetic_satellite(
        SatId { constellation: Constellation::Glonass, prn: 8 },
        SignalBand::L1,
        SignalCode::Unknown,
        24_000_000.0,
        t_rx_s,
    ));

    let obs = ObsEpoch {
        t_rx_s: Seconds(t_rx_s),
        source_time: ReceiverSampleTrace::from_sample_index(0, 1_000.0),
        gps_week: Some(0),
        tow_s: Some(Seconds(t_rx_s)),
        epoch_idx: 1,
        discontinuity: false,
        valid: true,
        processing_ms: None,
        role: ReceiverRole::Rover,
        sats,
        decision: ObservationEpochDecision::Accepted,
        decision_reason: Some("accepted_observables_present".to_string()),
        manifest: None,
    };

    let navigation_data = position_broadcast_navigation_from_gps_ephemerides(&gps_ephemerides);

    let solution = navigation_engine
        .solve_epoch_with_navigation_data(&obs, &navigation_data)
        .expect("mixed constellation refusal");

    assert!(!solution.valid);
    assert_eq!(solution.refusal_class, Some(NavRefusalClass::MixedConstellationInput));
    assert_eq!(solution.explain_decision, "mixed_constellation_time_handling_refused");
    assert!(solution
        .explain_reasons
        .iter()
        .any(|reason| reason == "unknown_inter_system_time_offset"));
    assert_eq!(solution.used_sat_count, 0);
}
