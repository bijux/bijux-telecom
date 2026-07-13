#![allow(dead_code, missing_docs)]

use bijux_gnss_core::api::{
    Constellation, GpsTime, LockFlags, Meters, NavHealthEvent, NavSolutionEpoch, ObsEpoch,
    ObsMetadata, ObsSatellite, ObsSignalTiming, ObservationEpochDecision, ObservationStatus,
    ReceiverRole, ReceiverSampleTrace, SatId, Seconds, SigId, SignalBand, SignalCode,
};
use bijux_gnss_receiver::api::{
    nav::{
        position_broadcast_navigation_from_gps_ephemerides, sat_state_galileo_e1,
        GalileoBroadcastNavigationData, GalileoClockCorrection, GalileoEphemeris,
        GalileoIonosphericCorrection, GalileoIonosphericDisturbanceFlags, GalileoSignalHealth,
        GalileoSystemTime, GpsEphemeris, PositionBroadcastNavigation,
    },
    Navigation, ReceiverPipelineConfig, ReceiverRuntime,
};
use bijux_gnss_testkit::coordinates::geodetic_to_ecef;
use bijux_gnss_testkit::position_truth::pseudorange_from_truth;

const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;
const RECEIVE_TIME_S: f64 = 100_000.0;
const TRUTH_LAT_DEG: f64 = 37.0;
const TRUTH_LON_DEG: f64 = -122.0;
const TRUTH_ALT_M: f64 = 25.0;
const RECEIVER_CLOCK_BIAS_S: f64 = 2.75e-4;
const GALILEO_BIAS_S: f64 = -1.15e-6;
const GALILEO_CLOCK_STEP_M: f64 = 80.0;
const EPOCH_COUNT: usize = 8;
const EPOCH_SPACING_S: f64 = 0.001;
const ANOMALY_ONSET_EPOCH_INDEX: u64 = 4;

pub struct ConstellationClockInconsistencyRun {
    pub solutions: Vec<NavSolutionEpoch>,
    pub anomaly_constellation: Constellation,
    pub anomaly_onset_epoch_index: u64,
}

pub fn static_constellation_clock_inconsistency_run() -> ConstellationClockInconsistencyRun {
    mixed_constellation_run(true)
}

pub fn static_stable_mixed_constellation_run() -> ConstellationClockInconsistencyRun {
    mixed_constellation_run(false)
}

pub fn constellation_clock_inconsistency_health_events(
    solution: &NavSolutionEpoch,
) -> Vec<&NavHealthEvent> {
    solution
        .health
        .iter()
        .filter(|event| matches!(event, NavHealthEvent::ConstellationClockInconsistency { .. }))
        .collect()
}

fn mixed_constellation_run(inject_clock_step: bool) -> ConstellationClockInconsistencyRun {
    let config = navigation_test_config();
    let mut navigation = Navigation::new(config, ReceiverRuntime::default());
    let truth_ecef_m = geodetic_to_ecef(TRUTH_LAT_DEG, TRUTH_LON_DEG, TRUTH_ALT_M);
    let gps_ephemerides = gps_ephemerides();
    let galileo_navigation = galileo_navigation();
    let mut navigation_data = position_broadcast_navigation_from_gps_ephemerides(&gps_ephemerides);
    navigation_data
        .extend(galileo_navigation.iter().cloned().map(PositionBroadcastNavigation::Galileo));

    let solutions = (0..EPOCH_COUNT)
        .filter_map(|epoch_index| {
            let elapsed_s = epoch_index as f64 * EPOCH_SPACING_S;
            let epoch_idx = (elapsed_s * 1000.0).round() as u64;
            let t_rx_s = RECEIVE_TIME_S + elapsed_s + RECEIVER_CLOCK_BIAS_S;
            let galileo_bias_s = GALILEO_BIAS_S
                + if inject_clock_step && epoch_idx >= ANOMALY_ONSET_EPOCH_INDEX {
                    GALILEO_CLOCK_STEP_M / SPEED_OF_LIGHT_MPS
                } else {
                    0.0
                };
            let obs = make_obs_epoch(
                t_rx_s,
                epoch_idx,
                truth_ecef_m,
                &gps_ephemerides,
                &galileo_navigation,
                galileo_bias_s,
            );
            navigation.solve_epoch_with_navigation_data(&obs, &navigation_data)
        })
        .collect::<Vec<_>>();

    ConstellationClockInconsistencyRun {
        solutions,
        anomaly_constellation: Constellation::Galileo,
        anomaly_onset_epoch_index: ANOMALY_ONSET_EPOCH_INDEX,
    }
}

fn navigation_test_config() -> ReceiverPipelineConfig {
    let mut config = ReceiverPipelineConfig::default();
    config.weighting.enabled = false;
    config.weighting.elev_mask_deg = -90.0;
    config.science_thresholds.min_mean_cn0_dbhz = 1.0;
    config.science_thresholds.max_pdop = 100.0;
    config.science_thresholds.max_gdop = 100.0;
    config.science_thresholds.max_residual_rms_m = 1_000.0;
    config
}

fn make_obs_epoch(
    t_rx_s: f64,
    epoch_idx: u64,
    truth_ecef_m: (f64, f64, f64),
    gps_ephemerides: &[GpsEphemeris],
    galileo_navigation: &[GalileoBroadcastNavigationData],
    galileo_bias_s: f64,
) -> ObsEpoch {
    let mut sats = gps_ephemerides
        .iter()
        .map(|ephemeris| {
            synthetic_satellite(
                ephemeris.sat,
                SignalBand::L1,
                SignalCode::Ca,
                gps_pseudorange_m(ephemeris, t_rx_s, truth_ecef_m, RECEIVER_CLOCK_BIAS_S),
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
                RECEIVER_CLOCK_BIAS_S,
                galileo_bias_s,
            ),
            t_rx_s,
        )
    }));

    ObsEpoch {
        t_rx_s: Seconds(t_rx_s),
        source_time: ReceiverSampleTrace::from_sample_index(epoch_idx, 1_000.0),
        gps_week: Some(2209),
        tow_s: Some(Seconds(t_rx_s)),
        epoch_idx,
        discontinuity: false,
        valid: true,
        processing_ms: None,
        role: ReceiverRole::Rover,
        sats,
        decision: ObservationEpochDecision::Accepted,
        decision_reason: Some("accepted_observables_present".to_string()),
        manifest: None,
    }
}

fn synthetic_satellite(
    sat: SatId,
    band: SignalBand,
    code: SignalCode,
    pseudorange_m: f64,
    t_rx_s: f64,
) -> ObsSatellite {
    ObsSatellite {
        signal_id: SigId { sat, band, code },
        pseudorange_m: Meters(pseudorange_m),
        pseudorange_var_m2: 4.0,
        carrier_phase_cycles: bijux_gnss_core::api::Cycles(0.0),
        carrier_phase_var_cycles2: 1.0,
        doppler_hz: bijux_gnss_core::api::Hertz(0.0),
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
            signal_travel_time_s: Seconds(pseudorange_m / SPEED_OF_LIGHT_MPS),
            transmit_gps_time: GpsTime {
                week: 2209,
                tow_s: t_rx_s - (pseudorange_m / SPEED_OF_LIGHT_MPS),
            },
        }),
        error_model: None,
        metadata: ObsMetadata {
            tracking_mode: "synthetic".to_string(),
            integration_ms: 1,
            lock_quality: 1.0,
            smoothing_window: 0,
            smoothing_age: 0,
            smoothing_resets: 0,
            observation_status: "accepted".to_string(),
            ..ObsMetadata::default()
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
        pseudorange_m = range_m + (receiver_clock_bias_s + galileo_bias_s) * SPEED_OF_LIGHT_MPS
            - state.clock_correction.bias_s * SPEED_OF_LIGHT_MPS;
        let next_tau = pseudorange_m / SPEED_OF_LIGHT_MPS;
        if (next_tau - tau).abs() < 1.0e-12 {
            break;
        }
        tau = next_tau;
    }
    pseudorange_m
}

fn gps_ephemerides() -> Vec<GpsEphemeris> {
    vec![
        make_gps_ephemeris(1, 0.0, 0.0),
        make_gps_ephemeris(2, 0.8, 0.9),
        make_gps_ephemeris(3, 1.6, 1.8),
        make_gps_ephemeris(4, 2.4, 2.7),
    ]
}

fn galileo_navigation() -> Vec<GalileoBroadcastNavigationData> {
    vec![make_galileo_navigation(19, 1.17, 0.84), make_galileo_navigation(24, -0.83, 1.52)]
}

fn make_gps_ephemeris(prn: u8, omega0: f64, m0: f64) -> GpsEphemeris {
    GpsEphemeris {
        sat: SatId { constellation: Constellation::Gps, prn },
        iodc: 1,
        iode: 1,
        week: 2209,
        sv_health: 0,
        toe_s: RECEIVE_TIME_S,
        toc_s: RECEIVE_TIME_S,
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

fn make_galileo_navigation(prn: u8, omega0: f64, m0: f64) -> GalileoBroadcastNavigationData {
    GalileoBroadcastNavigationData {
        sat: SatId { constellation: Constellation::Galileo, prn },
        iodnav: prn as u16,
        gst: GalileoSystemTime { week: 2222, tow_s: RECEIVE_TIME_S as u32 },
        sisa_e1_e5b: 77,
        signal_health: GalileoSignalHealth {
            e5b_signal_health: 0,
            e1b_signal_health: 0,
            e5b_data_valid: true,
            e1b_data_valid: true,
        },
        clock: GalileoClockCorrection {
            t0c_s: RECEIVE_TIME_S,
            af0: 0.0,
            af1: 0.0,
            af2: 0.0,
            bgd_e1_e5a_s: 0.0,
            bgd_e1_e5b_s: 0.0,
        },
        ephemeris: GalileoEphemeris {
            sat: SatId { constellation: Constellation::Galileo, prn },
            iodnav: prn as u16,
            toe_s: RECEIVE_TIME_S,
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
