#![allow(dead_code, missing_docs)]

use bijux_gnss_core::api::{
    Constellation, GpsTime, LockFlags, Meters, NavHealthEvent, NavSolutionEpoch, ObsEpoch,
    ObsMetadata, ObsSatellite, ObsSignalTiming, ObservationEpochDecision, ObservationStatus,
    ReceiverRole, ReceiverSampleTrace, SatId, Seconds, SigId, SignalBand, SignalCode,
};
use bijux_gnss_nav::api::GpsEphemeris;
use bijux_gnss_receiver::api::{Navigation, ReceiverPipelineConfig, ReceiverRuntime};
use bijux_gnss_testkit::geometry::geodetic_to_ecef;
use bijux_gnss_testkit::position_truth::pseudorange_from_truth;

const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;
const RECEIVE_TIME_S: f64 = 100_000.0;
const TRUTH_LAT_DEG: f64 = 37.0;
const TRUTH_LON_DEG: f64 = -122.0;
const TRUTH_ALT_M: f64 = 25.0;
const EPOCH_COUNT: usize = 8;
const EPOCH_SPACING_S: f64 = 0.001;
const ANOMALY_ONSET_EPOCH_INDEX: u64 = 4;
const EXPECTED_REFUSAL_EPOCH_INDEX: u64 = ANOMALY_ONSET_EPOCH_INDEX + 2;
const RESIDUAL_BIAS_PATTERN_M: [f64; 8] = [32.0, -28.0, 24.0, -20.0, 16.0, -12.0, 8.0, -4.0];

pub struct ResidualWhitenessRun {
    pub solutions: Vec<NavSolutionEpoch>,
    pub anomaly_onset_epoch_index: u64,
    pub expected_refusal_epoch_index: u64,
}

#[derive(Clone, Copy)]
enum ResidualPatternMode {
    Fixed,
    Rotating,
}

pub fn static_residual_temporal_correlation_run() -> ResidualWhitenessRun {
    residual_whiteness_run(ResidualPatternMode::Fixed)
}

pub fn static_rotating_residual_pattern_run() -> ResidualWhitenessRun {
    residual_whiteness_run(ResidualPatternMode::Rotating)
}

pub fn residual_temporal_correlation_health_events(
    solution: &NavSolutionEpoch,
) -> Vec<&NavHealthEvent> {
    solution
        .health
        .iter()
        .filter(|event| matches!(event, NavHealthEvent::ResidualTemporalCorrelation { .. }))
        .collect()
}

fn residual_whiteness_run(pattern_mode: ResidualPatternMode) -> ResidualWhitenessRun {
    let truth_ecef_m = geodetic_to_ecef(TRUTH_LAT_DEG, TRUTH_LON_DEG, TRUTH_ALT_M);
    let ephemerides = navigation_ephemerides();
    let mut navigation = Navigation::new(navigation_test_config(), ReceiverRuntime::default());
    let solutions = (0..EPOCH_COUNT)
        .filter_map(|epoch_index| {
            let elapsed_s = epoch_index as f64 * EPOCH_SPACING_S;
            let epoch_idx = (elapsed_s * 1000.0).round() as u64;
            let observation = make_obs_epoch(
                RECEIVE_TIME_S + elapsed_s,
                epoch_idx,
                truth_ecef_m,
                &ephemerides,
                pattern_mode,
            );
            navigation.solve_epoch(&observation, &ephemerides)
        })
        .collect::<Vec<_>>();

    ResidualWhitenessRun {
        solutions,
        anomaly_onset_epoch_index: ANOMALY_ONSET_EPOCH_INDEX,
        expected_refusal_epoch_index: EXPECTED_REFUSAL_EPOCH_INDEX,
    }
}

fn navigation_test_config() -> ReceiverPipelineConfig {
    let mut config = ReceiverPipelineConfig::default();
    config.raim = false;
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
        make_ephemeris(7, 4.8, 5.4),
        make_ephemeris(8, 5.6, 6.3),
    ]
}

fn make_ephemeris(prn: u8, omega0: f64, m0: f64) -> GpsEphemeris {
    GpsEphemeris {
        sat: SatId { constellation: Constellation::Gps, prn },
        iodc: 0,
        iode: 0,
        week: 0,
        sv_health: 0,
        sv_accuracy: Some(2),
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
    epoch_idx: u64,
    truth_ecef_m: (f64, f64, f64),
    ephemerides: &[GpsEphemeris],
    pattern_mode: ResidualPatternMode,
) -> ObsEpoch {
    let sats = ephemerides
        .iter()
        .enumerate()
        .map(|(satellite_index, ephemeris)| {
            let bias_m = residual_bias_for_epoch(epoch_idx, satellite_index, pattern_mode);
            let pseudorange_m = synthetic_pseudorange_m(ephemeris, t_rx_s, truth_ecef_m) + bias_m;
            make_obs_satellite(ephemeris.sat, pseudorange_m, t_rx_s)
        })
        .collect::<Vec<_>>();

    ObsEpoch {
        t_rx_s: Seconds(t_rx_s),
        source_time: ReceiverSampleTrace::from_sample_index(epoch_idx, 1_000.0),
        gps_week: Some(0),
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

fn residual_bias_for_epoch(
    epoch_idx: u64,
    satellite_index: usize,
    pattern_mode: ResidualPatternMode,
) -> f64 {
    if epoch_idx < ANOMALY_ONSET_EPOCH_INDEX {
        return 0.0;
    }

    match pattern_mode {
        ResidualPatternMode::Fixed => RESIDUAL_BIAS_PATTERN_M[satellite_index],
        ResidualPatternMode::Rotating => {
            let rotation = (epoch_idx - ANOMALY_ONSET_EPOCH_INDEX) as usize;
            let rotated_index = (satellite_index + rotation) % RESIDUAL_BIAS_PATTERN_M.len();
            RESIDUAL_BIAS_PATTERN_M[rotated_index]
        }
    }
}

fn make_obs_satellite(sat: SatId, pseudorange_m: f64, t_rx_s: f64) -> ObsSatellite {
    ObsSatellite {
        signal_id: SigId { sat, band: SignalBand::L1, code: SignalCode::Ca },
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
            observation_status: "accepted".to_string(),
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
