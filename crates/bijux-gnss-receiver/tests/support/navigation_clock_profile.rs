#![allow(dead_code, missing_docs)]

use bijux_gnss_core::api::{
    Chips, Constellation, Cycles, Epoch, GpsTime, Hertz, NavSolutionEpoch, ObsEpoch,
    ReceiverSampleTrace, SatId, SignalDelayAlignment, TrackEpoch, GPS_L1_CA_CARRIER_HZ,
};
use bijux_gnss_nav::api::GpsEphemeris;
use bijux_gnss_receiver::api::ValidationReferenceEpoch;
use bijux_gnss_receiver::api::{
    observations_from_tracking_results_with_gps_anchor,
    sim::{
        truth_guided_receiver_accuracy_budgets, validate_pvt_accuracy_budget,
        validate_truth_guided_pvt_table, SyntheticPvtAccuracyReport, SyntheticPvtClockProfileCase,
        SyntheticPvtTruthReferenceEpoch, SyntheticPvtTruthTableReport, SyntheticSignalParams,
    },
    Navigation, ReceiverPipelineConfig, ReceiverRuntime, TrackingResult,
};
use bijux_gnss_testkit::coordinates::{ecef_to_geodetic, geodetic_to_ecef};
use bijux_gnss_testkit::position_truth::pseudorange_from_truth;

const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;
const GPS_L1_CA_CODE_RATE_HZ: f64 = 1_023_000.0;
const GPS_L1_CA_CODE_PERIOD_CHIPS: f64 = 1023.0;
const CLOCK_PROFILE_EPOCH_COUNT: usize = 250;
const CLOCK_PROFILE_EPOCH_SPACING_S: f64 = 0.001;
const CLOCK_PROFILE_RECEIVE_TIME_S: f64 = 100_000.0;
const CLOCK_PROFILE_TRUTH_LAT_DEG: f64 = 37.0;
const CLOCK_PROFILE_TRUTH_LON_DEG: f64 = -122.0;
const CLOCK_PROFILE_TRUTH_ALT_M: f64 = 10.0;
const OSCILLATOR_DRIFT_S_PER_S: f64 = 5.0e-6;

#[derive(Debug, Clone, PartialEq)]
pub struct NavigationClockTruthEpoch {
    pub epoch_idx: u64,
    pub receive_time_s: f64,
    pub truth_clock_bias_s: f64,
    pub truth_clock_drift_s_per_s: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct NavigationClockProfile {
    pub profile_name: &'static str,
    pub truth_ecef_m: (f64, f64, f64),
    pub truth_epochs: Vec<NavigationClockTruthEpoch>,
}

#[allow(dead_code)]
pub struct NavigationClockCase {
    pub scenario_id: String,
    pub clock_profile: NavigationClockProfile,
    pub observations: Vec<ObsEpoch>,
    pub solutions: Vec<NavSolutionEpoch>,
    pub truth_table: SyntheticPvtTruthTableReport,
    pub pvt_accuracy: SyntheticPvtAccuracyReport,
}

pub fn synthetic_navigation_clock_profiles() -> Vec<NavigationClockProfile> {
    let truth_ecef_m = geodetic_to_ecef(
        CLOCK_PROFILE_TRUTH_LAT_DEG,
        CLOCK_PROFILE_TRUTH_LON_DEG,
        CLOCK_PROFILE_TRUTH_ALT_M,
    );

    vec![
        receiver_clock_profile(
            "stable_receiver_clock",
            truth_ecef_m,
            0.0,
            0.0,
            CLOCK_PROFILE_RECEIVE_TIME_S,
            CLOCK_PROFILE_EPOCH_SPACING_S,
            CLOCK_PROFILE_EPOCH_COUNT,
        ),
        receiver_clock_profile(
            "oscillator_drift_receiver_clock",
            truth_ecef_m,
            0.0,
            OSCILLATOR_DRIFT_S_PER_S,
            CLOCK_PROFILE_RECEIVE_TIME_S,
            CLOCK_PROFILE_EPOCH_SPACING_S,
            CLOCK_PROFILE_EPOCH_COUNT,
        ),
    ]
}

pub fn synthetic_navigation_clock_profile(profile_name: &str) -> NavigationClockProfile {
    synthetic_navigation_clock_profiles()
        .into_iter()
        .find(|profile| profile.profile_name == profile_name)
        .unwrap_or_else(|| panic!("unknown navigation clock profile: {profile_name}"))
}

pub fn build_navigation_clock_cases() -> Vec<NavigationClockCase> {
    synthetic_navigation_clock_profiles().into_iter().map(build_navigation_clock_case).collect()
}

pub fn navigation_clock_case<'a>(
    cases: &'a [NavigationClockCase],
    profile_name: &str,
) -> &'a NavigationClockCase {
    cases
        .iter()
        .find(|case| case.clock_profile.profile_name == profile_name)
        .unwrap_or_else(|| panic!("unknown navigation clock case: {profile_name}"))
}

pub fn receiver_clock_profile(
    profile_name: &'static str,
    truth_ecef_m: (f64, f64, f64),
    initial_clock_bias_s: f64,
    truth_clock_drift_s_per_s: f64,
    initial_receive_time_s: f64,
    epoch_spacing_s: f64,
    epoch_count: usize,
) -> NavigationClockProfile {
    assert!(epoch_count > 0, "receiver clock profile requires at least one truth epoch");
    assert!(
        epoch_spacing_s.is_finite() && epoch_spacing_s > 0.0,
        "receiver clock profile requires positive finite epoch spacing",
    );
    assert!(
        [
            truth_ecef_m.0,
            truth_ecef_m.1,
            truth_ecef_m.2,
            initial_clock_bias_s,
            truth_clock_drift_s_per_s,
            initial_receive_time_s,
        ]
        .into_iter()
        .all(f64::is_finite),
        "receiver clock profile requires finite truth coordinates, bias, drift, and time",
    );

    let truth_epochs = (0..epoch_count)
        .map(|epoch_index| {
            let elapsed_s = epoch_index as f64 * epoch_spacing_s;
            NavigationClockTruthEpoch {
                epoch_idx: (elapsed_s * 1000.0).round() as u64,
                receive_time_s: initial_receive_time_s + elapsed_s,
                truth_clock_bias_s: initial_clock_bias_s + truth_clock_drift_s_per_s * elapsed_s,
                truth_clock_drift_s_per_s,
            }
        })
        .collect::<Vec<_>>();

    NavigationClockProfile { profile_name, truth_ecef_m, truth_epochs }
}

pub fn build_navigation_clock_case(profile: NavigationClockProfile) -> NavigationClockCase {
    validate_truth_epochs(&profile.truth_epochs);

    let config = navigation_clock_config();
    let ephemerides = clock_profile_ephemerides();
    let scenario_id = format!("navigation_clock_profile_{}", profile.profile_name);
    let tracking = truth_seeded_clock_tracking_results(&config, &ephemerides, &profile);
    let gps_time = Some(GpsTime {
        week: ephemerides.first().expect("navigation clock ephemeris").week,
        tow_s: profile.truth_epochs.first().expect("navigation clock truth epoch").receive_time_s,
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
    let reference = truth_reference_epochs_for_clock(&profile, &solutions);
    let truth_table = validate_truth_guided_pvt_table(&scenario_id, &solutions, &reference);
    let pvt_accuracy =
        validate_pvt_accuracy_budget(&truth_table, truth_guided_receiver_accuracy_budgets().pvt);

    NavigationClockCase {
        scenario_id,
        clock_profile: profile,
        observations,
        solutions,
        truth_table,
        pvt_accuracy,
    }
}

pub fn receiver_clock_drift_doppler_offset_hz(clock_drift_s_per_s: f64) -> f64 {
    -GPS_L1_CA_CARRIER_HZ.value() * clock_drift_s_per_s
}

pub fn truth_guided_clock_profile_cases<'a>(
    cases: &'a [NavigationClockCase],
) -> Vec<SyntheticPvtClockProfileCase<'a>> {
    let stable_case = navigation_clock_case(cases, "stable_receiver_clock");

    cases
        .iter()
        .map(|case| {
            let expected_observation_doppler_offset_hz = receiver_clock_drift_doppler_offset_hz(
                truth_clock_drift_s_per_s(&case.clock_profile),
            );
            let reference_observations = (case.clock_profile.profile_name
                != "stable_receiver_clock")
                .then_some(stable_case.observations.as_slice());

            SyntheticPvtClockProfileCase {
                scenario_id: &case.scenario_id,
                injected_clock_drift_s_per_s: truth_clock_drift_s_per_s(&case.clock_profile),
                expected_observation_doppler_offset_hz,
                observations: &case.observations,
                reference_observations,
                solutions: &case.solutions,
                accuracy: &case.pvt_accuracy,
            }
        })
        .collect()
}

pub fn clock_profile_base_doppler_hz(sat: SatId) -> f64 {
    satellite_signal_seed(sat).doppler_hz
}

pub fn truth_clock_bias_by_epoch(
    profile: &NavigationClockProfile,
) -> std::collections::BTreeMap<u64, f64> {
    profile.truth_epochs.iter().map(|epoch| (epoch.epoch_idx, epoch.truth_clock_bias_s)).collect()
}

pub fn truth_clock_drift_s_per_s(profile: &NavigationClockProfile) -> f64 {
    profile
        .truth_epochs
        .first()
        .map(|epoch| epoch.truth_clock_drift_s_per_s)
        .expect("receiver clock profile truth epoch")
}

fn navigation_clock_config() -> ReceiverPipelineConfig {
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
    config
}

fn clock_profile_ephemerides() -> Vec<GpsEphemeris> {
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
        toe_s: CLOCK_PROFILE_RECEIVE_TIME_S,
        toc_s: CLOCK_PROFILE_RECEIVE_TIME_S,
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

fn truth_seeded_clock_tracking_results(
    config: &ReceiverPipelineConfig,
    ephemerides: &[GpsEphemeris],
    profile: &NavigationClockProfile,
) -> Vec<TrackingResult> {
    let initial_receive_time_s = profile
        .truth_epochs
        .first()
        .expect("receiver clock tracking requires a truth epoch")
        .receive_time_s;
    ephemerides
        .iter()
        .map(|ephemeris| {
            let signal_seed = satellite_signal_seed(ephemeris.sat);
            let epochs = profile
                .truth_epochs
                .iter()
                .map(|truth_epoch| {
                    let pseudorange_m = synthetic_pseudorange_m(
                        ephemeris,
                        truth_epoch.receive_time_s,
                        profile.truth_ecef_m,
                    ) + truth_epoch.truth_clock_bias_s * SPEED_OF_LIGHT_MPS;
                    let pseudorange_chips =
                        pseudorange_m * GPS_L1_CA_CODE_RATE_HZ / SPEED_OF_LIGHT_MPS;
                    let whole_code_periods =
                        (pseudorange_chips / GPS_L1_CA_CODE_PERIOD_CHIPS).floor() as u64;
                    let code_phase_chips =
                        pseudorange_chips - whole_code_periods as f64 * GPS_L1_CA_CODE_PERIOD_CHIPS;
                    let sample_index = ((truth_epoch.receive_time_s - initial_receive_time_s)
                        * config.sampling_freq_hz)
                        .round() as u64;
                    let doppler_hz = signal_seed.doppler_hz
                        + receiver_clock_drift_doppler_offset_hz(
                            truth_epoch.truth_clock_drift_s_per_s,
                        );

                    TrackEpoch {
                        epoch: Epoch { index: truth_epoch.epoch_idx },
                        sample_index,
                        source_time: ReceiverSampleTrace::from_sample_index(
                            sample_index,
                            config.sampling_freq_hz,
                        ),
                        sat: ephemeris.sat,
                        signal_band: bijux_gnss_core::api::SignalBand::L1,
                        signal_code: bijux_gnss_core::api::SignalCode::Ca,
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
                        cn0_dbhz: signal_seed.cn0_db_hz as f64,
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
                        channel_uid: format!("Gps-{:02}-clock-profile", ephemeris.sat.prn),
                        tracking_provenance: "navigation_clock_profile_truth".to_string(),
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
            let first_epoch = epochs.first().expect("receiver clock track epoch");

            TrackingResult {
                sat: ephemeris.sat,
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

fn truth_reference_epochs_for_clock(
    profile: &NavigationClockProfile,
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
                .expect("receiver clock truth reference epoch");
            let (latitude_deg, longitude_deg, altitude_m) = ecef_to_geodetic(
                profile.truth_ecef_m.0,
                profile.truth_ecef_m.1,
                profile.truth_ecef_m.2,
            );

            SyntheticPvtTruthReferenceEpoch {
                position: ValidationReferenceEpoch {
                    epoch_idx: solution.epoch.index,
                    t_rx_s: Some(solution.t_rx_s.0),
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
                clock_bias_s: truth_epoch.truth_clock_bias_s,
            }
        })
        .collect()
}

fn satellite_signal_seed(sat: SatId) -> SyntheticSignalParams {
    let doppler_hz = match sat.prn {
        3 => -1_800.0,
        7 => -1_000.0,
        11 => -250.0,
        19 => 500.0,
        23 => 1_250.0,
        29 => 2_000.0,
        prn => panic!("missing synthetic clock profile signal seed for PRN {prn}"),
    };

    SyntheticSignalParams {
        sat,
        glonass_frequency_channel: None,
        signal_band: bijux_gnss_core::api::SignalBand::L1,
        signal_code: bijux_gnss_core::api::SignalCode::Ca,
        doppler_hz,
        code_phase_chips: 0.0,
        carrier_phase_rad: 0.0,
        cn0_db_hz: 52.0,
        navigation_data: false.into(),
    }
}

fn synthetic_pseudorange_m(
    ephemeris: &GpsEphemeris,
    receive_time_s: f64,
    truth_ecef_m: (f64, f64, f64),
) -> f64 {
    pseudorange_from_truth(ephemeris, truth_ecef_m, receive_time_s, 0.0)
}

fn validate_truth_epochs(truth_epochs: &[NavigationClockTruthEpoch]) {
    assert!(!truth_epochs.is_empty(), "receiver clock profile requires at least one truth epoch");

    for window in truth_epochs.windows(2) {
        assert!(
            window[1].epoch_idx > window[0].epoch_idx,
            "receiver clock profile requires strictly increasing epoch indices",
        );
        assert!(
            window[1].receive_time_s > window[0].receive_time_s,
            "receiver clock profile requires strictly increasing receive times",
        );
    }

    assert!(
        truth_epochs.iter().all(|epoch| {
            [epoch.receive_time_s, epoch.truth_clock_bias_s, epoch.truth_clock_drift_s_per_s]
                .into_iter()
                .all(f64::is_finite)
        }),
        "receiver clock profile requires finite truth epochs",
    );
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
