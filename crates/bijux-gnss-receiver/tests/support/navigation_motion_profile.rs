#![allow(missing_docs)]

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
        validate_truth_guided_pvt_table, SyntheticPvtAccuracyReport,
        SyntheticPvtTruthReferenceEpoch, SyntheticPvtTruthTableReport, SyntheticSignalParams,
    },
    Navigation, ReceiverPipelineConfig, ReceiverRuntime, TrackingResult,
};
use bijux_gnss_testkit::coordinates::{ecef_to_geodetic, geodetic_to_ecef};
use bijux_gnss_testkit::position_truth::pseudorange_from_truth;

const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;
const GPS_L1_CA_CODE_RATE_HZ: f64 = 1_023_000.0;
const GPS_L1_CA_CODE_PERIOD_CHIPS: f64 = 1023.0;
const MOTION_EPOCH_COUNT: usize = 250;
const MOTION_EPOCH_SPACING_S: f64 = 0.001;
const LINEAR_MOTION_VELOCITY_ECEF_MPS: (f64, f64, f64) = (8.0, -3.0, 1.5);
const MOTION_RECEIVE_TIME_S: f64 = 100_000.0;
const MOTION_TRUTH_LAT_DEG: f64 = 37.0;
const MOTION_TRUTH_LON_DEG: f64 = -122.0;
const MOTION_TRUTH_ALT_M: f64 = 10.0;

#[derive(Debug, Clone, PartialEq)]
pub struct NavigationMotionTruthEpoch {
    pub epoch_idx: u64,
    pub receive_time_s: f64,
    pub truth_ecef_m: (f64, f64, f64),
    pub truth_velocity_ecef_mps: (f64, f64, f64),
}

#[derive(Debug, Clone, PartialEq)]
pub struct NavigationMotionProfile {
    pub profile_name: &'static str,
    pub truth_epochs: Vec<NavigationMotionTruthEpoch>,
}

#[allow(dead_code)]
pub struct NavigationMotionCase {
    pub scenario_id: String,
    pub motion_profile: NavigationMotionProfile,
    pub solutions: Vec<NavSolutionEpoch>,
    pub truth_table: SyntheticPvtTruthTableReport,
    pub pvt_accuracy: SyntheticPvtAccuracyReport,
}

pub fn synthetic_navigation_motion_profiles() -> Vec<NavigationMotionProfile> {
    let truth_ecef_m =
        geodetic_to_ecef(MOTION_TRUTH_LAT_DEG, MOTION_TRUTH_LON_DEG, MOTION_TRUTH_ALT_M);

    vec![
        receiver_motion_profile(
            "static_reference",
            truth_ecef_m,
            (0.0, 0.0, 0.0),
            MOTION_RECEIVE_TIME_S,
            MOTION_EPOCH_SPACING_S,
            MOTION_EPOCH_COUNT,
        ),
        receiver_motion_profile(
            "linear_receiver_motion",
            truth_ecef_m,
            LINEAR_MOTION_VELOCITY_ECEF_MPS,
            MOTION_RECEIVE_TIME_S,
            MOTION_EPOCH_SPACING_S,
            MOTION_EPOCH_COUNT,
        ),
    ]
}

pub fn receiver_motion_profile(
    profile_name: &'static str,
    initial_truth_ecef_m: (f64, f64, f64),
    truth_velocity_ecef_mps: (f64, f64, f64),
    initial_receive_time_s: f64,
    epoch_spacing_s: f64,
    epoch_count: usize,
) -> NavigationMotionProfile {
    assert!(epoch_count > 0, "receiver motion profile requires at least one truth epoch");
    assert!(
        epoch_spacing_s.is_finite() && epoch_spacing_s > 0.0,
        "receiver motion profile requires positive finite epoch spacing",
    );
    assert!(
        [initial_truth_ecef_m.0, initial_truth_ecef_m.1, initial_truth_ecef_m.2]
            .into_iter()
            .all(f64::is_finite),
        "receiver motion profile requires finite truth coordinates",
    );
    assert!(
        [
            truth_velocity_ecef_mps.0,
            truth_velocity_ecef_mps.1,
            truth_velocity_ecef_mps.2,
            initial_receive_time_s,
        ]
        .into_iter()
        .all(f64::is_finite),
        "receiver motion profile requires finite velocity and receive time",
    );

    let truth_epochs = (0..epoch_count)
        .map(|epoch_index| {
            let elapsed_s = epoch_index as f64 * epoch_spacing_s;
            NavigationMotionTruthEpoch {
                epoch_idx: (elapsed_s * 1000.0).round() as u64,
                receive_time_s: initial_receive_time_s + elapsed_s,
                truth_ecef_m: (
                    initial_truth_ecef_m.0 + truth_velocity_ecef_mps.0 * elapsed_s,
                    initial_truth_ecef_m.1 + truth_velocity_ecef_mps.1 * elapsed_s,
                    initial_truth_ecef_m.2 + truth_velocity_ecef_mps.2 * elapsed_s,
                ),
                truth_velocity_ecef_mps,
            }
        })
        .collect::<Vec<_>>();

    NavigationMotionProfile { profile_name, truth_epochs }
}

pub fn build_navigation_motion_case(profile: NavigationMotionProfile) -> NavigationMotionCase {
    validate_truth_epochs(&profile.truth_epochs);

    let config = navigation_motion_config();
    let ephemerides = motion_ephemerides();
    let signals = motion_satellite_signals(&ephemerides);
    let scenario_id = format!("navigation_motion_profile_{}", profile.profile_name);
    let tracking = truth_seeded_motion_tracking_results(
        &config,
        &ephemerides,
        &signals,
        &profile.truth_epochs,
    );
    let gps_time = Some(GpsTime {
        week: ephemerides.first().expect("navigation motion ephemeris").week,
        tow_s: profile.truth_epochs.first().expect("navigation motion truth epoch").receive_time_s,
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
    let reference = truth_reference_epochs_for_motion(&profile.truth_epochs, &solutions);
    let truth_table = validate_truth_guided_pvt_table(&scenario_id, &solutions, &reference);
    let pvt_accuracy =
        validate_pvt_accuracy_budget(&truth_table, truth_guided_receiver_accuracy_budgets().pvt);

    NavigationMotionCase {
        scenario_id,
        motion_profile: profile,
        solutions,
        truth_table,
        pvt_accuracy,
    }
}

pub fn motion_profile_path_length_m(profile: &NavigationMotionProfile) -> f64 {
    profile
        .truth_epochs
        .windows(2)
        .map(|window| {
            let previous = window[0].truth_ecef_m;
            let current = window[1].truth_ecef_m;
            let dx = current.0 - previous.0;
            let dy = current.1 - previous.1;
            let dz = current.2 - previous.2;
            (dx * dx + dy * dy + dz * dz).sqrt()
        })
        .sum()
}

pub fn motion_profile_mean_speed_mps(profile: &NavigationMotionProfile) -> f64 {
    let duration_s = match (profile.truth_epochs.first(), profile.truth_epochs.last()) {
        (Some(first), Some(last)) if last.receive_time_s > first.receive_time_s => {
            last.receive_time_s - first.receive_time_s
        }
        _ => 0.0,
    };
    if duration_s == 0.0 {
        0.0
    } else {
        motion_profile_path_length_m(profile) / duration_s
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
    // This fixture validates receiver-motion truth tracking against an idealized full-sky
    // synthetic constellation, so it must not cull satellites by a local horizon mask.
    config.weighting.enabled = false;
    config.weighting.elev_mask_deg = -90.0;
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
        toe_s: MOTION_RECEIVE_TIME_S,
        toc_s: MOTION_RECEIVE_TIME_S,
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
    pseudorange_from_truth(ephemeris, truth_ecef_m, receive_time_s, 0.0)
}

fn truth_seeded_motion_tracking_results(
    config: &ReceiverPipelineConfig,
    ephemerides: &[GpsEphemeris],
    signals: &[SyntheticSignalParams],
    truth_epochs: &[NavigationMotionTruthEpoch],
) -> Vec<TrackingResult> {
    let initial_receive_time_s = truth_epochs
        .first()
        .expect("receiver motion tracking requires a truth epoch")
        .receive_time_s;
    signals
        .iter()
        .map(|signal| {
            let ephemeris = ephemerides
                .iter()
                .find(|ephemeris| ephemeris.sat == signal.sat)
                .expect("receiver motion ephemeris for signal");
            let epochs = truth_epochs
                .iter()
                .map(|truth_epoch| {
                    let pseudorange_m = synthetic_pseudorange_m(
                        ephemeris,
                        truth_epoch.receive_time_s,
                        truth_epoch.truth_ecef_m,
                    );
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
                        channel_uid: format!("Gps-{:02}-motion-truth", signal.sat.prn),
                        tracking_provenance: "navigation_motion_truth".to_string(),
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
            let first_epoch = epochs.first().expect("receiver motion track epoch");

            TrackingResult {
                sat: signal.sat,
                carrier_hz: signal.doppler_hz,
                code_phase_samples: first_epoch.code_phase_samples.0,
                acquisition_hypothesis: "accepted".to_string(),
                acquisition_score: 1.0,
                acquisition_code_phase_samples: first_epoch.code_phase_samples.0.round() as usize,
                acquisition_carrier_hz: signal.doppler_hz,
                acq_to_track_state: "accepted".to_string(),
                epochs,
                transitions: Vec::new(),
            }
        })
        .collect()
}

fn truth_reference_epochs_for_motion(
    truth_epochs: &[NavigationMotionTruthEpoch],
    solutions: &[bijux_gnss_core::api::NavSolutionEpoch],
) -> Vec<SyntheticPvtTruthReferenceEpoch> {
    let truth_by_epoch = truth_epochs
        .iter()
        .map(|epoch| (epoch.epoch_idx, epoch))
        .collect::<std::collections::BTreeMap<_, _>>();

    solutions
        .iter()
        .map(|solution| {
            let truth_epoch = truth_by_epoch
                .get(&solution.epoch.index)
                .expect("receiver motion truth reference epoch");
            let (latitude_deg, longitude_deg, altitude_m) = ecef_to_geodetic(
                truth_epoch.truth_ecef_m.0,
                truth_epoch.truth_ecef_m.1,
                truth_epoch.truth_ecef_m.2,
            );

            SyntheticPvtTruthReferenceEpoch {
                position: ValidationReferenceEpoch {
                    epoch_idx: solution.epoch.index,
                    t_rx_s: Some(solution.t_rx_s.0),
                    latitude_deg,
                    longitude_deg,
                    altitude_m,
                    ecef_x_m: Some(truth_epoch.truth_ecef_m.0),
                    ecef_y_m: Some(truth_epoch.truth_ecef_m.1),
                    ecef_z_m: Some(truth_epoch.truth_ecef_m.2),
                    vel_x_mps: Some(truth_epoch.truth_velocity_ecef_mps.0),
                    vel_y_mps: Some(truth_epoch.truth_velocity_ecef_mps.1),
                    vel_z_mps: Some(truth_epoch.truth_velocity_ecef_mps.2),
                },
                clock_bias_s: 0.0,
            }
        })
        .collect()
}

fn validate_truth_epochs(truth_epochs: &[NavigationMotionTruthEpoch]) {
    assert!(!truth_epochs.is_empty(), "receiver motion profile requires at least one truth epoch",);

    for window in truth_epochs.windows(2) {
        assert!(
            window[1].epoch_idx > window[0].epoch_idx,
            "receiver motion profile requires strictly increasing epoch indices",
        );
        assert!(
            window[1].receive_time_s > window[0].receive_time_s,
            "receiver motion profile requires strictly increasing receive times",
        );
    }

    assert!(
        truth_epochs.iter().all(|epoch| {
            [
                epoch.receive_time_s,
                epoch.truth_ecef_m.0,
                epoch.truth_ecef_m.1,
                epoch.truth_ecef_m.2,
                epoch.truth_velocity_ecef_mps.0,
                epoch.truth_velocity_ecef_mps.1,
                epoch.truth_velocity_ecef_mps.2,
            ]
            .into_iter()
            .all(f64::is_finite)
        }),
        "receiver motion profile requires finite truth epochs",
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
