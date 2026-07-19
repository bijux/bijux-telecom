#![allow(missing_docs)]

use bijux_gnss_core::api::{
    Constellation, Cycles, GpsTime, Hertz, LockFlags, Meters, ObsEpoch, ObsMetadata, ObsSatellite,
    ObsSignalTiming, ObservationEpochDecision, ObservationStatus, ReceiverRole,
    ReceiverSampleTrace, SatId, Seconds, SigId, SignalBand, SignalCode,
};
use bijux_gnss_receiver::api::ValidationReferenceEpoch;
use bijux_gnss_receiver::api::{
    nav::{
        position_broadcast_navigation_from_beidou_navigations,
        position_broadcast_navigation_from_gps_ephemerides, sat_state_beidou_b1i,
        sat_state_galileo_e1, BeidouBroadcastNavigationData, BeidouClockCorrection,
        BeidouEphemeris, BeidouIonosphericCorrection, BeidouSignalHealth, BeidouSystemTime,
        GalileoBroadcastNavigationData, GalileoClockCorrection, GalileoEphemeris,
        GalileoIonosphericCorrection, GalileoIonosphericDisturbanceFlags, GalileoSignalHealth,
        GalileoSystemTime, GpsEphemeris, PositionBroadcastNavigation,
    },
    sim::{
        truth_guided_receiver_accuracy_budgets, validate_pvt_accuracy_budget,
        validate_truth_guided_pvt_table, SyntheticPvtAccuracyReport,
        SyntheticPvtConstellationGeometryProfileCase, SyntheticPvtTruthReferenceEpoch,
        SyntheticPvtTruthTableReport,
    },
    Navigation, ReceiverPipelineConfig, ReceiverRuntime,
};
use bijux_gnss_testkit::geometry::{ecef_to_geodetic, geodetic_to_ecef};
use bijux_gnss_testkit::position_truth::pseudorange_from_truth;

pub struct ReceiverConstellationGeometryCase {
    pub scenario_id: String,
    pub constellations: Vec<Constellation>,
    pub visible_satellite_count: usize,
    pub truth_table: SyntheticPvtTruthTableReport,
    pub accuracy: SyntheticPvtAccuracyReport,
}

impl ReceiverConstellationGeometryCase {
    pub fn profile_case(&self) -> SyntheticPvtConstellationGeometryProfileCase<'_> {
        SyntheticPvtConstellationGeometryProfileCase {
            scenario_id: &self.scenario_id,
            constellations: &self.constellations,
            visible_satellite_count: self.visible_satellite_count,
            truth_table: &self.truth_table,
            accuracy: &self.accuracy,
        }
    }
}

pub fn build_receiver_constellation_geometry_case(
    scenario_id: &str,
    gps_count: usize,
    galileo_count: usize,
    beidou_count: usize,
) -> ReceiverConstellationGeometryCase {
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

    let mut satellites = gps_ephemerides
        .iter()
        .take(gps_count)
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
    satellites.extend(galileo_navigation.iter().take(galileo_count).map(|navigation| {
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
    satellites.extend(beidou_navigation.iter().take(beidou_count).map(|navigation| {
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
        sats: satellites,
        decision: ObservationEpochDecision::Accepted,
        decision_reason: Some("accepted_observables_present".to_string()),
        manifest: None,
    };

    let mut navigation_data = position_broadcast_navigation_from_gps_ephemerides(
        &gps_ephemerides.iter().take(gps_count).cloned().collect::<Vec<_>>(),
    );
    navigation_data.extend(
        galileo_navigation
            .iter()
            .take(galileo_count)
            .cloned()
            .map(PositionBroadcastNavigation::Galileo),
    );
    navigation_data.extend(position_broadcast_navigation_from_beidou_navigations(
        &beidou_navigation.iter().take(beidou_count).cloned().collect::<Vec<_>>(),
    ));

    let solutions = navigation_engine
        .solve_epoch_with_navigation_data(&obs, &navigation_data)
        .into_iter()
        .filter(|solution| solution.valid)
        .collect::<Vec<_>>();
    let truth_reference = truth_reference_epochs(1, t_rx_s, truth_ecef_m, receiver_clock_bias_s);
    let truth_table = validate_truth_guided_pvt_table(scenario_id, &solutions, &truth_reference);
    let accuracy =
        validate_pvt_accuracy_budget(&truth_table, truth_guided_receiver_accuracy_budgets().pvt);

    ReceiverConstellationGeometryCase {
        scenario_id: scenario_id.to_string(),
        constellations: enabled_constellations(gps_count, galileo_count, beidou_count),
        visible_satellite_count: gps_count + galileo_count + beidou_count,
        truth_table,
        accuracy,
    }
}

fn enabled_constellations(
    gps_count: usize,
    galileo_count: usize,
    beidou_count: usize,
) -> Vec<Constellation> {
    let mut constellations = Vec::new();
    if gps_count > 0 {
        constellations.push(Constellation::Gps);
    }
    if galileo_count > 0 {
        constellations.push(Constellation::Galileo);
    }
    if beidou_count > 0 {
        constellations.push(Constellation::Beidou);
    }
    constellations
}

fn truth_reference_epochs(
    epoch_idx: u64,
    t_rx_s: f64,
    truth_ecef_m: (f64, f64, f64),
    receiver_clock_bias_s: f64,
) -> Vec<SyntheticPvtTruthReferenceEpoch> {
    let (latitude_deg, longitude_deg, altitude_m) =
        ecef_to_geodetic(truth_ecef_m.0, truth_ecef_m.1, truth_ecef_m.2);
    vec![SyntheticPvtTruthReferenceEpoch {
        position: ValidationReferenceEpoch {
            epoch_idx,
            t_rx_s: Some(t_rx_s),
            latitude_deg,
            longitude_deg,
            altitude_m,
            ecef_x_m: Some(truth_ecef_m.0),
            ecef_y_m: Some(truth_ecef_m.1),
            ecef_z_m: Some(truth_ecef_m.2),
            vel_x_mps: None,
            vel_y_mps: None,
            vel_z_mps: None,
        },
        clock_bias_s: receiver_clock_bias_s,
    }]
}

fn make_gps_ephemeris(prn: u8, omega0: f64, m0: f64, t_ref_s: f64) -> GpsEphemeris {
    GpsEphemeris {
        sat: SatId { constellation: Constellation::Gps, prn },
        iodc: 1,
        iode: 1,
        week: 2209,
        sv_health: 0,
        sv_accuracy: Some(2),
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
    pseudorange_from_truth(ephemeris, truth_ecef_m, t_rx_s, receiver_clock_bias_s)
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
