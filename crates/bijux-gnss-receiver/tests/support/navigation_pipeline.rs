#![allow(dead_code, missing_docs)]

use bijux_gnss_core::api::{
    ecef_to_geodetic, Chips, Constellation, Cycles, Epoch, GpsTime, Hertz, NavSolutionEpoch,
    ObsEpoch, ReceiverSampleTrace, SatId, SignalDelayAlignment, TrackEpoch,
    ValidationReferenceEpoch,
};
use bijux_gnss_nav::api::{geodetic_to_ecef, sat_state_gps_l1ca, GpsEphemeris};
use bijux_gnss_receiver::api::sim::SyntheticSignalParams;
use bijux_gnss_receiver::api::{
    observations_from_tracking_results_with_gps_anchor, Navigation, ReceiverPipelineConfig,
    ReceiverRuntime, TrackingResult, ValidationBudgets,
};

pub const CLEAN_SYNTHETIC_PVT_POSITION_ERROR_MAX_M: f64 = 5.0;
const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;
const GPS_L1_CA_CODE_RATE_HZ: f64 = 1_023_000.0;
const GPS_L1_CA_CODE_PERIOD_CHIPS: f64 = 1023.0;
const CLEAN_SYNTHETIC_RECEIVE_TIME_S: f64 = 100_000.0;
const CLEAN_SYNTHETIC_TRUTH_LAT_DEG: f64 = 37.0;
const CLEAN_SYNTHETIC_TRUTH_LON_DEG: f64 = -122.0;
const CLEAN_SYNTHETIC_TRUTH_ALT_M: f64 = 10.0;

#[derive(Debug, Clone)]
pub struct CleanSyntheticPvtScenario {
    pub ephemerides: Vec<GpsEphemeris>,
    pub satellites: Vec<CleanSyntheticSignal>,
    pub truth_ecef_m: (f64, f64, f64),
    pub receive_time_s: f64,
    pub target_epoch_idx: u64,
}

#[derive(Debug, Clone)]
pub struct CleanSyntheticSignal {
    pub signal: SyntheticSignalParams,
    pub whole_code_periods: u64,
}

#[derive(Debug, Clone)]
pub struct SatellitePseudorangeNoise {
    pub sat: SatId,
    pub pseudorange_noise_m: f64,
}

#[derive(Debug, Clone)]
pub struct SyntheticPseudorangeNoiseProfile {
    pub profile_name: &'static str,
    pub satellites: Vec<SatellitePseudorangeNoise>,
}

pub struct CleanSyntheticNavigationRun {
    pub config: ReceiverPipelineConfig,
    pub profile: CleanSyntheticPvtScenario,
    pub tracking: Vec<TrackingResult>,
    pub observations: Vec<ObsEpoch>,
    pub solutions: Vec<NavSolutionEpoch>,
    pub reference_epochs: Vec<ValidationReferenceEpoch>,
}

pub struct NoisySyntheticNavigationRun {
    pub noise_profile: SyntheticPseudorangeNoiseProfile,
    pub run: CleanSyntheticNavigationRun,
}

pub fn clean_synthetic_navigation_run() -> CleanSyntheticNavigationRun {
    navigation_run(clean_synthetic_pvt_scenario())
}

pub fn noisy_synthetic_navigation_run(
    noise_profile: SyntheticPseudorangeNoiseProfile,
) -> NoisySyntheticNavigationRun {
    let run = navigation_run(clean_synthetic_pvt_scenario_with_noise(&noise_profile));
    NoisySyntheticNavigationRun { noise_profile, run }
}

fn navigation_run(profile: CleanSyntheticPvtScenario) -> CleanSyntheticNavigationRun {
    let config = clean_synthetic_navigation_config();
    let tracking_results = tracking_results_for_profile(&config, &profile);
    let capture_start_gps_time = profile
        .ephemerides
        .first()
        .map(|ephemeris| GpsTime { week: ephemeris.week, tow_s: profile.receive_time_s });
    let observation_report = observations_from_tracking_results_with_gps_anchor(
        &config,
        capture_start_gps_time,
        &tracking_results,
        10,
    );
    let observations = observation_report
        .output
        .into_iter()
        .filter(|epoch| {
            epoch.epoch_idx >= profile.target_epoch_idx && epoch.valid && epoch.sats.len() >= 4
        })
        .collect::<Vec<_>>();
    let mut navigation = Navigation::new(config.clone(), ReceiverRuntime::default());
    let solutions = observations
        .iter()
        .filter_map(|epoch| navigation.solve_epoch(epoch, &profile.ephemerides))
        .collect::<Vec<_>>();
    let reference_epochs = reference_epochs_for_truth(&solutions, profile.truth_ecef_m);

    CleanSyntheticNavigationRun {
        config,
        profile,
        tracking: tracking_results,
        observations,
        solutions,
        reference_epochs,
    }
}

fn tracking_results_for_profile(
    config: &ReceiverPipelineConfig,
    profile: &CleanSyntheticPvtScenario,
) -> Vec<TrackingResult> {
    profile
        .satellites
        .iter()
        .map(|satellite| synthetic_truth_track(config, &satellite.signal, satellite.whole_code_periods))
        .collect()
}

pub fn clean_synthetic_pvt_budgets() -> ValidationBudgets {
    ValidationBudgets {
        tracking_carrier_jitter_hz: f64::INFINITY,
        nav_min_lock_epochs: 0,
        nav_rejected_ratio_max: 1.0,
        reference_position_error_3d_m_max: Some(CLEAN_SYNTHETIC_PVT_POSITION_ERROR_MAX_M),
        ..ValidationBudgets::default()
    }
}

pub fn position_error_3d_m(solution: &NavSolutionEpoch, truth_ecef_m: (f64, f64, f64)) -> f64 {
    let dx = solution.ecef_x_m.0 - truth_ecef_m.0;
    let dy = solution.ecef_y_m.0 - truth_ecef_m.1;
    let dz = solution.ecef_z_m.0 - truth_ecef_m.2;
    (dx * dx + dy * dy + dz * dz).sqrt()
}

pub fn pseudorange_noise_to_code_phase_chips(pseudorange_noise_m: f64) -> f64 {
    pseudorange_noise_m * GPS_L1_CA_CODE_RATE_HZ / SPEED_OF_LIGHT_MPS
}

fn clean_synthetic_navigation_config() -> ReceiverPipelineConfig {
    ReceiverPipelineConfig {
        sampling_freq_hz: 1_023_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 5,
        tracking_budget_ms: 100.0,
        tracking_over_budget_action: "continue".to_string(),
        tropo_enable: false,
        ..ReceiverPipelineConfig::default()
    }
}

fn reference_epochs_for_truth(
    solutions: &[NavSolutionEpoch],
    truth_ecef_m: (f64, f64, f64),
) -> Vec<ValidationReferenceEpoch> {
    let (latitude_deg, longitude_deg, altitude_m) =
        ecef_to_geodetic(truth_ecef_m.0, truth_ecef_m.1, truth_ecef_m.2);

    solutions
        .iter()
        .map(|solution| ValidationReferenceEpoch {
            epoch_idx: solution.epoch.index,
            t_rx_s: Some(solution.t_rx_s.0),
            latitude_deg,
            longitude_deg,
            altitude_m,
            ecef_x_m: Some(truth_ecef_m.0),
            ecef_y_m: Some(truth_ecef_m.1),
            ecef_z_m: Some(truth_ecef_m.2),
            vel_x_mps: None,
            vel_y_mps: None,
            vel_z_mps: None,
        })
        .collect()
}

fn synthetic_truth_track(
    config: &ReceiverPipelineConfig,
    signal: &bijux_gnss_receiver::api::sim::SyntheticSignalParams,
    whole_code_periods: u64,
) -> TrackingResult {
    let samples_per_chip = config.sampling_freq_hz / config.code_freq_basis_hz;
    let code_phase_samples = signal.code_phase_chips * samples_per_chip;
    let carrier_phase_cycles = signal.carrier_phase_rad / std::f64::consts::TAU;
    let epoch = TrackEpoch {
        epoch: Epoch { index: 0 },
        sample_index: 0,
        source_time: ReceiverSampleTrace::from_sample_index(0, config.sampling_freq_hz),
        sat: signal.sat,
        prompt_i: 1.0,
        prompt_q: 0.0,
        early_i: 0.0,
        early_q: 0.0,
        late_i: 0.0,
        late_q: 0.0,
        carrier_hz: Hertz(signal.doppler_hz),
        carrier_phase_cycles: Cycles(carrier_phase_cycles),
        code_rate_hz: Hertz(config.code_freq_basis_hz),
        code_phase_samples: Chips(code_phase_samples),
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
        channel_uid: format!("Gps-{:02}-clean-synthetic", signal.sat.prn),
        tracking_provenance: "clean_synthetic_navigation_truth".to_string(),
        tracking_assumptions: None,
        signal_delay_alignment: signal_delay_alignment(whole_code_periods),
        tracking_uncertainty: None,
        processing_ms: None,
    };

    TrackingResult {
        sat: signal.sat,
        carrier_hz: signal.doppler_hz,
        code_phase_samples,
        acquisition_hypothesis: "accepted".to_string(),
        acquisition_score: 1.0,
        acquisition_code_phase_samples: code_phase_samples.round() as usize,
        acquisition_carrier_hz: signal.doppler_hz,
        acq_to_track_state: "accepted".to_string(),
        epochs: vec![epoch],
        transitions: Vec::new(),
    }
}

fn clean_synthetic_pvt_scenario() -> CleanSyntheticPvtScenario {
    let truth_ecef_m = geodetic_to_ecef(
        CLEAN_SYNTHETIC_TRUTH_LAT_DEG,
        CLEAN_SYNTHETIC_TRUTH_LON_DEG,
        CLEAN_SYNTHETIC_TRUTH_ALT_M,
    );
    let ephemerides = vec![
        make_ephemeris(3, 0.0, 0.0),
        make_ephemeris(7, 0.8, 0.8),
        make_ephemeris(11, 1.6, 1.6),
        make_ephemeris(19, 2.4, 2.4),
        make_ephemeris(23, 3.2, 3.2),
    ];
    let satellites = ephemerides
        .iter()
        .zip([-1_500.0, -700.0, 0.0, 700.0, 1_500.0])
        .map(|(ephemeris, doppler_hz)| {
            let pseudorange_m =
                synthetic_pseudorange_m(ephemeris, CLEAN_SYNTHETIC_RECEIVE_TIME_S, truth_ecef_m);
            let pseudorange_chips = pseudorange_m * GPS_L1_CA_CODE_RATE_HZ / SPEED_OF_LIGHT_MPS;
            let whole_code_periods =
                (pseudorange_chips / GPS_L1_CA_CODE_PERIOD_CHIPS).floor() as u64;
            let code_phase_chips =
                pseudorange_chips - whole_code_periods as f64 * GPS_L1_CA_CODE_PERIOD_CHIPS;
            CleanSyntheticSignal {
                signal: SyntheticSignalParams {
                    sat: ephemeris.sat,
                    doppler_hz,
                    code_phase_chips,
                    carrier_phase_rad: 0.0,
                    cn0_db_hz: 52.0,
                    data_bit_flip: false,
                },
                whole_code_periods,
            }
        })
        .collect::<Vec<_>>();

    CleanSyntheticPvtScenario {
        ephemerides,
        satellites,
        truth_ecef_m,
        receive_time_s: CLEAN_SYNTHETIC_RECEIVE_TIME_S,
        target_epoch_idx: 0,
    }
}

fn clean_synthetic_pvt_scenario_with_noise(
    noise_profile: &SyntheticPseudorangeNoiseProfile,
) -> CleanSyntheticPvtScenario {
    let mut scenario = clean_synthetic_pvt_scenario();

    for satellite in &mut scenario.satellites {
        let injected_noise_m =
            pseudorange_noise_for_satellite(&noise_profile.satellites, satellite.signal.sat);
        if injected_noise_m == 0.0 {
            continue;
        }

        let pseudorange_chips = satellite.whole_code_periods as f64 * GPS_L1_CA_CODE_PERIOD_CHIPS
            + satellite.signal.code_phase_chips
            + pseudorange_noise_to_code_phase_chips(injected_noise_m);
        let whole_code_periods = (pseudorange_chips / GPS_L1_CA_CODE_PERIOD_CHIPS).floor() as u64;
        let code_phase_chips =
            pseudorange_chips - whole_code_periods as f64 * GPS_L1_CA_CODE_PERIOD_CHIPS;

        satellite.whole_code_periods = whole_code_periods;
        satellite.signal.code_phase_chips = code_phase_chips;
    }

    scenario
}

fn pseudorange_noise_for_satellite(
    satellites: &[SatellitePseudorangeNoise],
    sat: SatId,
) -> f64 {
    satellites
        .iter()
        .find(|satellite| satellite.sat == sat)
        .map(|satellite| satellite.pseudorange_noise_m)
        .unwrap_or_default()
}

fn signal_delay_alignment(whole_code_periods: u64) -> Option<SignalDelayAlignment> {
    Some(SignalDelayAlignment { whole_code_periods, source: "synthetic_truth".to_string() })
}

fn make_ephemeris(prn: u8, omega0: f64, m0: f64) -> GpsEphemeris {
    GpsEphemeris {
        sat: SatId { constellation: Constellation::Gps, prn },
        iodc: 0,
        iode: 0,
        week: 0,
        sv_health: 0,
        toe_s: CLEAN_SYNTHETIC_RECEIVE_TIME_S,
        toc_s: CLEAN_SYNTHETIC_RECEIVE_TIME_S,
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

fn synthetic_pseudorange_m(
    ephemeris: &GpsEphemeris,
    receive_time_s: f64,
    truth_ecef_m: (f64, f64, f64),
) -> f64 {
    let mut tau = 0.07;
    let mut pseudorange_m = 0.0;
    for _ in 0..10 {
        let sat = sat_state_gps_l1ca(ephemeris, receive_time_s - tau, tau);
        let dx = truth_ecef_m.0 - sat.x_m;
        let dy = truth_ecef_m.1 - sat.y_m;
        let dz = truth_ecef_m.2 - sat.z_m;
        let range_m = (dx * dx + dy * dy + dz * dz).sqrt();
        pseudorange_m = range_m - sat.clock_correction.bias_s * SPEED_OF_LIGHT_MPS;
        let next_tau = pseudorange_m / SPEED_OF_LIGHT_MPS;
        if (next_tau - tau).abs() < 1.0e-12 {
            break;
        }
        tau = next_tau;
    }
    pseudorange_m
}
