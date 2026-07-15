#![allow(dead_code, missing_docs)]

use std::collections::BTreeMap;

use bijux_gnss_core::api::{
    Chips, Constellation, Cycles, Epoch, GpsTime, Hertz, NavHealthEvent, NavSolutionEpoch,
    ObsEpoch, ReceiverSampleTrace, SatId, SignalDelayAlignment, TrackEpoch,
};
use bijux_gnss_nav::api::GpsEphemeris;
use bijux_gnss_receiver::api::sim::SyntheticSignalParams;
use bijux_gnss_receiver::api::{
    observations_from_tracking_results_with_gps_anchor, Navigation, ReceiverPipelineConfig,
    ReceiverRuntime, TrackingResult,
};
use bijux_gnss_testkit::coordinates::geodetic_to_ecef;
use bijux_gnss_testkit::position_truth::pseudorange_from_truth;

#[path = "navigation_motion_profile.rs"]
mod navigation_motion_profile;

use navigation_motion_profile::{receiver_motion_profile, NavigationMotionTruthEpoch};

const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;
const GPS_L1_CA_CODE_RATE_HZ: f64 = 1_023_000.0;
const GPS_L1_CA_CODE_PERIOD_CHIPS: f64 = 1023.0;
const RECEIVE_TIME_S: f64 = 100_000.0;
const TRUTH_LAT_DEG: f64 = 37.0;
const TRUTH_LON_DEG: f64 = -122.0;
const TRUTH_ALT_M: f64 = 10.0;
const EPOCH_COUNT: usize = 250;
const EPOCH_SPACING_S: f64 = 0.001;
const ANOMALY_ONSET_EPOCH_INDEX: u64 = 120;

pub struct ReplayTimingAnomalyRun {
    pub observations: Vec<ObsEpoch>,
    pub solutions: Vec<NavSolutionEpoch>,
    pub anomaly_onset_epoch_index: u64,
}

pub fn static_replay_timing_anomaly_run() -> ReplayTimingAnomalyRun {
    static_delay_step_run(
        "replay_timing_anomaly_static",
        &[(3, 140.0), (7, 110.0), (11, 80.0), (19, 30.0), (23, 130.0), (29, 20.0)],
    )
}

pub fn static_uniform_delay_step_run() -> ReplayTimingAnomalyRun {
    static_delay_step_run(
        "uniform_delay_step_static",
        &[(3, 85.0), (7, 85.0), (11, 85.0), (19, 85.0), (23, 85.0), (29, 85.0)],
    )
}

pub fn static_single_satellite_delay_step_run() -> ReplayTimingAnomalyRun {
    static_delay_step_run("single_satellite_delay_step_static", &[(11, 140.0)])
}

pub fn replay_timing_health_events(solution: &NavSolutionEpoch) -> Vec<&NavHealthEvent> {
    solution
        .health
        .iter()
        .filter(|event| matches!(event, NavHealthEvent::ReplayTimingAnomaly { .. }))
        .collect()
}

fn static_delay_step_run(
    profile_name: &'static str,
    delay_steps_by_prn_m: &[(u8, f64)],
) -> ReplayTimingAnomalyRun {
    let truth_origin_ecef_m = geodetic_to_ecef(TRUTH_LAT_DEG, TRUTH_LON_DEG, TRUTH_ALT_M);
    let profile = receiver_motion_profile(
        profile_name,
        truth_origin_ecef_m,
        (0.0, 0.0, 0.0),
        RECEIVE_TIME_S,
        EPOCH_SPACING_S,
        EPOCH_COUNT,
    );
    let config = navigation_motion_config();
    let ephemerides = motion_ephemerides();
    let signals = motion_satellite_signals(&ephemerides);
    let tracking = truth_seeded_tracking_results_with_delay_steps(
        &config,
        &ephemerides,
        &signals,
        &profile.truth_epochs,
        delay_steps_by_prn_m,
    );
    let gps_time = Some(GpsTime {
        week: ephemerides.first().expect("replay timing ephemeris").week,
        tow_s: profile.truth_epochs.first().expect("replay timing truth epoch").receive_time_s,
    });
    let observations =
        observations_from_tracking_results_with_gps_anchor(&config, gps_time, &tracking, 1)
            .output
            .into_iter()
            .filter(|epoch| epoch.valid && epoch.sats.len() >= 4)
            .collect::<Vec<_>>();
    let mut navigation = Navigation::new(config, ReceiverRuntime::default());
    let solutions = observations
        .iter()
        .filter_map(|epoch| navigation.solve_epoch(epoch, &ephemerides))
        .collect::<Vec<_>>();

    ReplayTimingAnomalyRun {
        observations,
        solutions,
        anomaly_onset_epoch_index: ANOMALY_ONSET_EPOCH_INDEX,
    }
}

fn navigation_motion_config() -> ReceiverPipelineConfig {
    let mut config = ReceiverPipelineConfig {
        sampling_freq_hz: 1_023_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 6,
        tracking_budget_ms: 100.0,
        tracking_over_budget_action: "continue".to_string(),
        tropo_enable: false,
        ..ReceiverPipelineConfig::default()
    };
    config.weighting.enabled = false;
    config.weighting.elev_mask_deg = -90.0;
    config.raim = false;
    config.robust_solver = false;
    config
}

fn motion_ephemerides() -> Vec<GpsEphemeris> {
    vec![
        make_ephemeris(3, 0.4, 0.4),
        make_ephemeris(7, 0.8, 0.8),
        make_ephemeris(11, 1.2, 1.2),
        make_ephemeris(19, 2.0, 2.0),
        make_ephemeris(23, 2.8, 2.8),
        make_ephemeris(29, 4.4, 4.4),
    ]
}

fn motion_satellite_signals(ephemerides: &[GpsEphemeris]) -> Vec<SyntheticSignalParams> {
    ephemerides
        .iter()
        .zip([-1_800.0, -1_000.0, -250.0, 500.0, 1_250.0, 2_000.0])
        .map(|(ephemeris, doppler_hz)| SyntheticSignalParams {
            sat: ephemeris.sat,
            glonass_frequency_channel: None,
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Ca,
            doppler_hz,
            code_phase_chips: 0.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 52.0,
            navigation_data: false.into(),
        })
        .collect()
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
        e: 0.0,
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

fn truth_seeded_tracking_results_with_delay_steps(
    config: &ReceiverPipelineConfig,
    ephemerides: &[GpsEphemeris],
    signals: &[SyntheticSignalParams],
    truth_epochs: &[NavigationMotionTruthEpoch],
    delay_steps_by_prn_m: &[(u8, f64)],
) -> Vec<TrackingResult> {
    let delay_steps_by_prn_m = delay_steps_by_prn_m.iter().copied().collect::<BTreeMap<_, _>>();
    let initial_receive_time_s =
        truth_epochs.first().expect("replay timing tracking requires a truth epoch").receive_time_s;

    signals
        .iter()
        .map(|signal| {
            let ephemeris = ephemerides
                .iter()
                .find(|ephemeris| ephemeris.sat == signal.sat)
                .expect("replay timing ephemeris for signal");
            let epochs = truth_epochs
                .iter()
                .map(|truth_epoch| {
                    let anomaly_active = truth_epoch.epoch_idx >= ANOMALY_ONSET_EPOCH_INDEX;
                    let pseudorange_step_m = anomaly_active
                        .then_some(
                            delay_steps_by_prn_m.get(&signal.sat.prn).copied().unwrap_or(0.0),
                        )
                        .unwrap_or(0.0);
                    let pseudorange_m = synthetic_pseudorange_m(
                        ephemeris,
                        truth_epoch.receive_time_s,
                        truth_epoch.truth_ecef_m,
                    ) + pseudorange_step_m;
                    let pseudorange_chips =
                        pseudorange_m * GPS_L1_CA_CODE_RATE_HZ / SPEED_OF_LIGHT_MPS;
                    let whole_code_periods =
                        (pseudorange_chips / GPS_L1_CA_CODE_PERIOD_CHIPS).floor() as u64;
                    let code_phase_chips =
                        pseudorange_chips - whole_code_periods as f64 * GPS_L1_CA_CODE_PERIOD_CHIPS;
                    let sample_index = ((truth_epoch.receive_time_s - initial_receive_time_s)
                        * config.sampling_freq_hz)
                        .round() as u64;

                    TrackEpoch {
                        epoch: Epoch { index: truth_epoch.epoch_idx },
                        sample_index,
                        source_time: ReceiverSampleTrace::from_sample_index(
                            sample_index,
                            config.sampling_freq_hz,
                        ),
                        sat: signal.sat,
                        signal_band: bijux_gnss_core::api::SignalBand::L1,
                        signal_code: bijux_gnss_core::api::SignalCode::Ca,
                        glonass_frequency_channel: None,
                        prompt_i: 1.0,
                        prompt_q: 0.0,
                        early_i: 0.0,
                        early_q: 0.0,
                        late_i: 0.0,
                        late_q: 0.0,
                        carrier_hz: Hertz(signal.doppler_hz),
                        carrier_phase_cycles: Cycles(0.0),
                        code_rate_hz: Hertz(config.code_freq_basis_hz),
                        code_phase_samples: Chips(tracking_code_phase_samples(
                            config,
                            code_phase_chips,
                        )),
                        lock: true,
                        cn0_dbhz: signal.cn0_db_hz as f64,
                        pll_lock: true,
                        dll_lock: true,
                        fll_lock: true,
                        cycle_slip: false,
                        nav_bit_lock: false,
                        navigation_bit_sign: None,
                        dll_err: 0.0,
                        pll_err: 0.0,
                        fll_err: 0.0,
                        anti_false_lock: false,
                        cycle_slip_reason: None,
                        lock_state: "tracking".to_string(),
                        lock_state_reason: Some("synthetic_truth".to_string()),
                        channel_id: Some(signal.sat.prn),
                        channel_uid: format!("Gps-{:02}-replay-timing", signal.sat.prn),
                        tracking_provenance: "replay_timing_anomaly_truth".to_string(),
                        tracking_assumptions: None,
                        signal_delay_alignment: Some(SignalDelayAlignment {
                            whole_code_periods,
                            sample_delay_samples: 0,
                            source: "synthetic_truth".to_string(),
                        }),
                        transmit_time: None,
                        tracking_uncertainty: Some(bijux_gnss_core::api::TrackingUncertainty {
                            code_phase_samples: 0.05,
                            carrier_phase_cycles: 0.02,
                            doppler_hz: 1.0,
                            cn0_dbhz: 0.5,
                        }),
                        processing_ms: None,
                    }
                })
                .collect::<Vec<_>>();
            let first_epoch = epochs.first().expect("replay timing track epoch");

            TrackingResult {
                sat: signal.sat,
                carrier_hz: first_epoch.carrier_hz.0,
                code_phase_samples: first_epoch.code_phase_samples.0,
                acquisition_hypothesis: "accepted".to_string(),
                acquisition_score: 1.0,
                acquisition_code_phase_samples: first_epoch.code_phase_samples.0.round() as usize,
                acquisition_carrier_hz: first_epoch.carrier_hz.0,
                acq_to_track_state: "accepted".to_string(),
                epochs,
                transitions: Vec::new(),
            }
        })
        .collect()
}

fn synthetic_pseudorange_m(
    ephemeris: &GpsEphemeris,
    receive_time_s: f64,
    receiver_ecef_m: (f64, f64, f64),
) -> f64 {
    pseudorange_from_truth(ephemeris, receiver_ecef_m, receive_time_s, 0.0)
}

fn tracking_code_phase_samples(config: &ReceiverPipelineConfig, code_phase_chips: f64) -> f64 {
    let samples_per_chip = config.sampling_freq_hz / config.code_freq_basis_hz;
    let period_samples = samples_per_chip * config.code_length as f64;
    let aligned_code_phase_samples = code_phase_chips * samples_per_chip;
    if !aligned_code_phase_samples.is_finite() || aligned_code_phase_samples < 0.0 {
        return aligned_code_phase_samples;
    }
    (period_samples - aligned_code_phase_samples).rem_euclid(period_samples)
}
