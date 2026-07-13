#![allow(dead_code, missing_docs)]

use bijux_gnss_core::api::{
    Constellation, GpsTime, LockFlags, Meters, NavHealthEvent, NavSolutionEpoch, ObsEpoch,
    ObsMetadata, ObsSatellite, ObsSignalTiming, ObservationEpochDecision, ObservationStatus,
    ObservationSupportClass, ObservationUncertaintyClass, ReceiverRole, ReceiverSampleTrace, SatId,
    Seconds, SigId, SignalBand, SignalCode, SignalSpec,
};
use bijux_gnss_nav::api::GpsEphemeris;
use bijux_gnss_receiver::api::{Navigation, ReceiverPipelineConfig, ReceiverRuntime};
use bijux_gnss_testkit::coordinates::geodetic_to_ecef;
use bijux_gnss_testkit::position_truth::pseudorange_from_truth;

const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;
const RECEIVE_TIME_S: f64 = 100_000.0;
const TERRESTRIAL_ALTITUDE_M: f64 = 10.0;
const IMPOSSIBLE_ALTITUDE_M: f64 = -2_000_000.0;

pub struct ImpossibleGeometryRun {
    pub solution: NavSolutionEpoch,
}

pub fn impossible_geometry_run() -> ImpossibleGeometryRun {
    solve_navigation_case(IMPOSSIBLE_ALTITUDE_M)
}

pub fn terrestrial_geometry_run() -> ImpossibleGeometryRun {
    solve_navigation_case(TERRESTRIAL_ALTITUDE_M)
}

pub fn impossible_geometry_health_events(solution: &NavSolutionEpoch) -> Vec<&NavHealthEvent> {
    solution
        .health
        .iter()
        .filter(|event| matches!(event, NavHealthEvent::ImpossibleGeometry { .. }))
        .collect()
}

fn solve_navigation_case(truth_altitude_m: f64) -> ImpossibleGeometryRun {
    let truth_ecef_m = geodetic_to_ecef(37.0, -122.0, truth_altitude_m);
    let ephemerides = navigation_ephemerides();
    let observation = make_obs_epoch(RECEIVE_TIME_S, truth_ecef_m, &ephemerides);
    let mut navigation = Navigation::new(navigation_test_config(), ReceiverRuntime::default());
    let solution = navigation.solve_epoch(&observation, &ephemerides).expect("navigation solution");

    ImpossibleGeometryRun { solution }
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

fn navigation_ephemerides() -> Vec<GpsEphemeris> {
    vec![
        make_ephemeris(1, 0.0, 0.0),
        make_ephemeris(2, 0.8, 0.9),
        make_ephemeris(3, 1.6, 1.8),
        make_ephemeris(4, 2.4, 2.7),
        make_ephemeris(5, 3.2, 3.6),
        make_ephemeris(6, 4.0, 4.5),
    ]
}

fn make_ephemeris(prn: u8, omega0: f64, m0: f64) -> GpsEphemeris {
    GpsEphemeris {
        sat: SatId { constellation: Constellation::Gps, prn },
        iodc: 0,
        iode: 0,
        week: 0,
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

fn make_obs_epoch(
    t_rx_s: f64,
    truth_ecef_m: (f64, f64, f64),
    ephemerides: &[GpsEphemeris],
) -> ObsEpoch {
    let sats = ephemerides
        .iter()
        .map(|ephemeris| make_obs_satellite(ephemeris, t_rx_s, truth_ecef_m))
        .collect::<Vec<_>>();

    ObsEpoch {
        t_rx_s: Seconds(t_rx_s),
        source_time: ReceiverSampleTrace::from_sample_index(0, 1_000.0),
        gps_week: Some(0),
        tow_s: Some(Seconds(t_rx_s)),
        epoch_idx: 0,
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

fn make_obs_satellite(
    ephemeris: &GpsEphemeris,
    t_rx_s: f64,
    truth_ecef_m: (f64, f64, f64),
) -> ObsSatellite {
    let pseudorange_m = synthetic_pseudorange_m(ephemeris, t_rx_s, truth_ecef_m);
    ObsSatellite {
        signal_id: SigId { sat: ephemeris.sat, band: SignalBand::L1, code: SignalCode::Ca },
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
                week: 0,
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
            signal: SignalSpec {
                constellation: Constellation::Gps,
                band: SignalBand::L1,
                code: SignalCode::Ca,
                code_rate_hz: 1_023_000.0,
                carrier_hz: bijux_gnss_core::api::GPS_L1_CA_CARRIER_HZ,
            },
            observation_status: "accepted".to_string(),
            observation_support_class: match ObservationSupportClass::Supported {
                ObservationSupportClass::Supported => "supported",
                ObservationSupportClass::Degraded => "degraded",
                ObservationSupportClass::Unsupported => "unsupported",
            }
            .to_string(),
            observation_uncertainty_class: match ObservationUncertaintyClass::Unknown {
                ObservationUncertaintyClass::Low => "low",
                ObservationUncertaintyClass::Medium => "medium",
                ObservationUncertaintyClass::High => "high",
                ObservationUncertaintyClass::Unknown => "unknown",
            }
            .to_string(),
            ..ObsMetadata::default()
        },
    }
}

fn synthetic_pseudorange_m(
    ephemeris: &GpsEphemeris,
    t_rx_s: f64,
    truth_ecef_m: (f64, f64, f64),
) -> f64 {
    pseudorange_from_truth(ephemeris, truth_ecef_m, t_rx_s, 0.0)
}
