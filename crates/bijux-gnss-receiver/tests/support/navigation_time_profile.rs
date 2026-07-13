#![allow(dead_code, missing_docs)]

use bijux_gnss_core::api::{
    Chips, Constellation, Cycles, Epoch, GpsTime, Hertz, NavSolutionEpoch, ReceiverSampleTrace,
    SatId, SignalDelayAlignment, TrackEpoch,
};
use bijux_gnss_nav::api::GpsEphemeris;
use bijux_gnss_receiver::api::ValidationReferenceEpoch;
use bijux_gnss_receiver::api::{
    observations_from_tracking_results_with_gps_anchor,
    sim::{
        truth_guided_receiver_accuracy_budgets, validate_pvt_accuracy_budget,
        validate_truth_guided_pvt_table, SyntheticPvtAccuracyReport, SyntheticPvtTimeProfileCase,
        SyntheticPvtTruthReferenceEpoch, SyntheticPvtTruthTableReport,
    },
    Navigation, ReceiverPipelineConfig, ReceiverRuntime, TrackingResult,
};
use bijux_gnss_testkit::coordinates::{ecef_to_geodetic, geodetic_to_ecef};
use bijux_gnss_testkit::position_truth::pseudorange_from_truth;

const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;
const GPS_L1_CA_CODE_RATE_HZ: f64 = 1_023_000.0;
const GPS_L1_CA_CODE_PERIOD_CHIPS: f64 = 1023.0;
const TIME_PROFILE_EPOCH_COUNT: usize = 250;
const TIME_PROFILE_EPOCH_SPACING_S: f64 = 0.02;
const TIME_PROFILE_RECEIVE_TIME_S: f64 = 100_000.0;
const TIME_PROFILE_TRUTH_LAT_DEG: f64 = 37.0;
const TIME_PROFILE_TRUTH_LON_DEG: f64 = -122.0;
const TIME_PROFILE_TRUTH_ALT_M: f64 = 10.0;
const TIME_PROFILE_SHAPE_BY_SATELLITE: [(u8, f64); 5] =
    [(3, -0.85), (7, 0.40), (11, 1.25), (19, -1.10), (23, 0.55)];
const TIME_PROFILE_LEVELS_M: [(&str, f64, f64); 3] = [
    ("stabilizing_navigation_accuracy", 8.0, 0.0),
    ("drifting_navigation_accuracy", 0.0, 4.0),
    ("diverging_navigation_accuracy", 0.0, 72.0),
];

#[derive(Debug, Clone, PartialEq)]
pub struct NavigationTimeTruthEpoch {
    pub epoch_idx: u64,
    pub receive_time_s: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct NavigationTimeProfile {
    pub profile_name: &'static str,
    pub truth_ecef_m: (f64, f64, f64),
    pub truth_epochs: Vec<NavigationTimeTruthEpoch>,
    pub start_level_m: f64,
    pub end_level_m: f64,
}

pub struct NavigationTimeCase {
    pub scenario_id: String,
    pub time_profile: NavigationTimeProfile,
    pub truth_table: SyntheticPvtTruthTableReport,
    pub pvt_accuracy: SyntheticPvtAccuracyReport,
    pub solutions: Vec<NavSolutionEpoch>,
}

pub fn navigation_time_profile(
    profile_name: &'static str,
    start_level_m: f64,
    end_level_m: f64,
    initial_receive_time_s: f64,
    epoch_spacing_s: f64,
    epoch_count: usize,
) -> NavigationTimeProfile {
    let truth_ecef_m = geodetic_to_ecef(
        TIME_PROFILE_TRUTH_LAT_DEG,
        TIME_PROFILE_TRUTH_LON_DEG,
        TIME_PROFILE_TRUTH_ALT_M,
    );

    let truth_epochs = (0..epoch_count)
        .map(|epoch_index| {
            let elapsed_s = epoch_index as f64 * epoch_spacing_s;
            NavigationTimeTruthEpoch {
                epoch_idx: (elapsed_s * 1000.0).round() as u64,
                receive_time_s: initial_receive_time_s + elapsed_s,
            }
        })
        .collect::<Vec<_>>();

    NavigationTimeProfile { profile_name, truth_ecef_m, truth_epochs, start_level_m, end_level_m }
}

pub fn synthetic_navigation_time_profiles() -> Vec<NavigationTimeProfile> {
    TIME_PROFILE_LEVELS_M
        .into_iter()
        .map(|(profile_name, start_level_m, end_level_m)| {
            navigation_time_profile(
                profile_name,
                start_level_m,
                end_level_m,
                TIME_PROFILE_RECEIVE_TIME_S,
                TIME_PROFILE_EPOCH_SPACING_S,
                TIME_PROFILE_EPOCH_COUNT,
            )
        })
        .collect()
}

pub fn synthetic_navigation_time_profile(profile_name: &str) -> NavigationTimeProfile {
    synthetic_navigation_time_profiles()
        .into_iter()
        .find(|profile| profile.profile_name == profile_name)
        .unwrap_or_else(|| panic!("unknown navigation time profile: {profile_name}"))
}

pub fn build_navigation_time_cases() -> Vec<NavigationTimeCase> {
    synthetic_navigation_time_profiles().into_iter().map(build_navigation_time_case).collect()
}

pub fn navigation_time_case<'a>(
    cases: &'a [NavigationTimeCase],
    profile_name: &str,
) -> &'a NavigationTimeCase {
    cases
        .iter()
        .find(|case| case.time_profile.profile_name == profile_name)
        .unwrap_or_else(|| panic!("unknown navigation time case: {profile_name}"))
}

pub fn truth_guided_time_profile_cases<'a>(
    cases: &'a [NavigationTimeCase],
) -> Vec<SyntheticPvtTimeProfileCase<'a>> {
    cases
        .iter()
        .map(|case| SyntheticPvtTimeProfileCase {
            scenario_id: &case.scenario_id,
            truth_table: &case.truth_table,
            accuracy: &case.pvt_accuracy,
        })
        .collect()
}

pub fn build_navigation_time_case(profile: NavigationTimeProfile) -> NavigationTimeCase {
    validate_truth_epochs(&profile.truth_epochs);

    let config = navigation_time_config();
    let ephemerides = time_profile_ephemerides();
    let scenario_id = format!("navigation_time_profile_{}", profile.profile_name);
    let tracking = truth_seeded_time_tracking_results(&config, &ephemerides, &profile);
    let gps_time = Some(GpsTime {
        week: ephemerides.first().expect("navigation time ephemeris").week,
        tow_s: profile.truth_epochs.first().expect("navigation time truth epoch").receive_time_s,
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
    let truth_table = validate_truth_guided_pvt_table(
        &scenario_id,
        &solutions,
        &truth_reference_epochs_for_time(&profile, &solutions),
    );
    let pvt_accuracy =
        validate_pvt_accuracy_budget(&truth_table, truth_guided_receiver_accuracy_budgets().pvt);

    NavigationTimeCase { scenario_id, time_profile: profile, truth_table, pvt_accuracy, solutions }
}

pub fn max_start_abs_pseudorange_bias_m(profile: &NavigationTimeProfile) -> f64 {
    TIME_PROFILE_SHAPE_BY_SATELLITE
        .iter()
        .map(|(_, scale)| (profile.start_level_m * scale).abs())
        .fold(0.0, f64::max)
}

pub fn max_end_abs_pseudorange_bias_m(profile: &NavigationTimeProfile) -> f64 {
    TIME_PROFILE_SHAPE_BY_SATELLITE
        .iter()
        .map(|(_, scale)| (profile.end_level_m * scale).abs())
        .fold(0.0, f64::max)
}

fn navigation_time_config() -> ReceiverPipelineConfig {
    let mut config = ReceiverPipelineConfig {
        sampling_freq_hz: 1_023_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 6,
        tracking_integration_ms: 20,
        tracking_budget_ms: 100.0,
        tracking_over_budget_action: "continue".to_string(),
        tropo_enable: false,
        ..ReceiverPipelineConfig::default()
    };
    config.weighting.enabled = false;
    config.weighting.elev_mask_deg = -90.0;
    config
}

fn time_profile_ephemerides() -> Vec<GpsEphemeris> {
    vec![
        make_ephemeris(3, 0.4, 0.4),
        make_ephemeris(7, 0.8, 0.8),
        make_ephemeris(11, 1.2, 1.2),
        make_ephemeris(19, 2.0, 2.0),
        make_ephemeris(23, 2.8, 2.8),
        make_ephemeris(29, 4.4, 4.4),
    ]
}

fn make_ephemeris(prn: u8, omega0: f64, m0: f64) -> GpsEphemeris {
    GpsEphemeris {
        sat: SatId { constellation: Constellation::Gps, prn },
        iodc: 0,
        iode: 0,
        week: 0,
        sv_health: 0,
        toe_s: TIME_PROFILE_RECEIVE_TIME_S,
        toc_s: TIME_PROFILE_RECEIVE_TIME_S,
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

fn truth_seeded_time_tracking_results(
    config: &ReceiverPipelineConfig,
    ephemerides: &[GpsEphemeris],
    profile: &NavigationTimeProfile,
) -> Vec<TrackingResult> {
    let initial_receive_time_s = profile
        .truth_epochs
        .first()
        .expect("receiver time profile requires a truth epoch")
        .receive_time_s;

    ephemerides
        .iter()
        .zip([-1_800.0, -1_000.0, -250.0, 500.0, 1_250.0, 2_000.0])
        .map(|(ephemeris, doppler_hz)| {
            let epochs = profile
                .truth_epochs
                .iter()
                .map(|truth_epoch| {
                    let pseudorange_m =
                        synthetic_pseudorange_m(
                            ephemeris,
                            truth_epoch.receive_time_s,
                            profile.truth_ecef_m,
                        ) + pseudorange_bias_m(profile, truth_epoch, ephemeris.sat.prn);
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
                        sat: ephemeris.sat,
                        signal_band: bijux_gnss_core::api::SignalBand::L1,
                        glonass_frequency_channel: None,
                        prompt_i: 1.0,
                        prompt_q: 0.0,
                        early_i: 0.0,
                        early_q: 0.0,
                        late_i: 0.0,
                        late_q: 0.0,
                        carrier_hz: Hertz(doppler_hz),
                        carrier_phase_cycles: Cycles(0.0),
                        code_rate_hz: Hertz(config.code_freq_basis_hz),
                        code_phase_samples: Chips(tracking_code_phase_samples(
                            config,
                            code_phase_chips,
                        )),
                        lock: true,
                        cn0_dbhz: 52.0,
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
                        channel_id: Some(ephemeris.sat.prn),
                        channel_uid: format!("Gps-{:02}-time-profile", ephemeris.sat.prn),
                        tracking_provenance: "navigation_time_profile_truth".to_string(),
                        tracking_assumptions: None,
                        signal_delay_alignment: Some(SignalDelayAlignment {
                            whole_code_periods,
                            source: "synthetic_truth".to_string(),
                        }),
                        tracking_uncertainty: None,
                        processing_ms: None,
                    }
                })
                .collect::<Vec<_>>();
            let first_epoch = epochs.first().expect("receiver time-profile track epoch");

            TrackingResult {
                sat: ephemeris.sat,
                carrier_hz: doppler_hz,
                code_phase_samples: first_epoch.code_phase_samples.0,
                acquisition_hypothesis: "accepted".to_string(),
                acquisition_score: 1.0,
                acquisition_code_phase_samples: first_epoch.code_phase_samples.0.round() as usize,
                acquisition_carrier_hz: doppler_hz,
                acq_to_track_state: "accepted".to_string(),
                epochs,
                transitions: Vec::new(),
            }
        })
        .collect()
}

fn truth_reference_epochs_for_time(
    profile: &NavigationTimeProfile,
    solutions: &[NavSolutionEpoch],
) -> Vec<SyntheticPvtTruthReferenceEpoch> {
    let truth_by_epoch = profile
        .truth_epochs
        .iter()
        .map(|epoch| (epoch.epoch_idx, epoch))
        .collect::<std::collections::BTreeMap<_, _>>();

    solutions
        .iter()
        .map(|solution| {
            let truth_epoch = truth_by_epoch
                .get(&solution.epoch.index)
                .expect("receiver time-profile truth reference epoch");
            let (latitude_deg, longitude_deg, altitude_m) = ecef_to_geodetic(
                profile.truth_ecef_m.0,
                profile.truth_ecef_m.1,
                profile.truth_ecef_m.2,
            );

            SyntheticPvtTruthReferenceEpoch {
                position: ValidationReferenceEpoch {
                    epoch_idx: solution.epoch.index,
                    t_rx_s: Some(truth_epoch.receive_time_s),
                    latitude_deg,
                    longitude_deg,
                    altitude_m,
                    ecef_x_m: Some(profile.truth_ecef_m.0),
                    ecef_y_m: Some(profile.truth_ecef_m.1),
                    ecef_z_m: Some(profile.truth_ecef_m.2),
                    vel_x_mps: Some(0.0),
                    vel_y_mps: Some(0.0),
                    vel_z_mps: Some(0.0),
                },
                clock_bias_s: 0.0,
            }
        })
        .collect()
}

fn pseudorange_bias_m(
    profile: &NavigationTimeProfile,
    truth_epoch: &NavigationTimeTruthEpoch,
    prn: u8,
) -> f64 {
    let shape = TIME_PROFILE_SHAPE_BY_SATELLITE
        .iter()
        .find(|(shape_prn, _)| *shape_prn == prn)
        .map(|(_, scale)| *scale)
        .unwrap_or(0.0);
    let last_epoch_idx =
        profile.truth_epochs.last().expect("receiver time profile truth epoch").epoch_idx as f64;
    let progress =
        if last_epoch_idx <= 0.0 { 0.0 } else { truth_epoch.epoch_idx as f64 / last_epoch_idx };
    let level_m = profile.start_level_m + (profile.end_level_m - profile.start_level_m) * progress;
    level_m * shape
}

fn synthetic_pseudorange_m(
    ephemeris: &GpsEphemeris,
    receive_time_s: f64,
    truth_ecef_m: (f64, f64, f64),
) -> f64 {
    pseudorange_from_truth(ephemeris, truth_ecef_m, receive_time_s, 0.0)
}

fn validate_truth_epochs(truth_epochs: &[NavigationTimeTruthEpoch]) {
    assert!(!truth_epochs.is_empty(), "receiver time profile requires at least one truth epoch");

    for window in truth_epochs.windows(2) {
        assert!(
            window[1].epoch_idx > window[0].epoch_idx,
            "receiver time profile requires strictly increasing epoch indices",
        );
        assert!(
            window[1].receive_time_s > window[0].receive_time_s,
            "receiver time profile requires strictly increasing receive times",
        );
    }
}

fn tracking_code_phase_samples(
    config: &ReceiverPipelineConfig,
    aligned_code_phase_chips: f64,
) -> f64 {
    let samples_per_chip = config.sampling_freq_hz / config.code_freq_basis_hz;
    let period_samples = samples_per_chip * config.code_length as f64;
    let aligned_code_phase_samples = aligned_code_phase_chips * samples_per_chip;
    if !aligned_code_phase_samples.is_finite() || aligned_code_phase_samples < 0.0 {
        return aligned_code_phase_samples;
    }
    (period_samples - aligned_code_phase_samples).rem_euclid(period_samples)
}
