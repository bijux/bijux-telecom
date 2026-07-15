#![allow(dead_code, missing_docs)]

use bijux_gnss_core::api::{
    Chips, Cycles, Epoch, GpsTime, Hertz, NavSolutionEpoch, ObsEpoch, ReceiverSampleTrace,
    SamplesFrame, SignalDelayAlignment, TrackEpoch, TrackingUncertainty,
};
use bijux_gnss_receiver::api::ValidationReferenceEpoch;
use bijux_gnss_receiver::api::{
    carrier_hz_from_doppler_hz, observations_from_tracking_results_with_gps_anchor,
    sim::{
        build_iq16_capture_bundle, build_truth_guided_gnss_accuracy_artifact, generate_l1_ca_multi,
        truth_guided_receiver_accuracy_budgets, validate_acquisition_accuracy_budget,
        validate_observation_accuracy_budget, validate_pvt_accuracy_budget,
        validate_tracking_accuracy_budget, validate_truth_guided_acquisition_table,
        validate_truth_guided_observations, validate_truth_guided_pvt_table,
        validate_truth_guided_tracking_table, SyntheticAcquisitionAccuracyReport,
        SyntheticGnssAccuracyArtifact, SyntheticGnssAccuracyArtifactCase,
        SyntheticGnssAccuracyDataSource, SyntheticGnssAccuracyReferenceTruth,
        SyntheticIqTruthBundle, SyntheticObservationAccuracyReport,
        SyntheticObservationTruthReference, SyntheticPvtAccuracyReport,
        SyntheticPvtTruthReferenceEpoch, SyntheticTrackingAccuracyReport,
    },
    Navigation, ReceiverPipelineConfig, ReceiverRuntime, TrackingResult,
};

#[path = "navigation_truth.rs"]
mod navigation_truth;

use bijux_gnss_testkit::coordinates::ecef_to_geodetic;
use navigation_truth::SyntheticPvtScenario;

const SYNTHETIC_REFERENCE_RECEIVE_TIME_S: f64 = 100_000.0;
const SYNTHETIC_HATCH_WINDOW: u32 = 10;
const TRACKING_CARRIER_PHASE_SIGMA_CYCLES: f64 = 0.05;
const TRACKING_DOPPLER_SIGMA_HZ: f64 = 0.25;
const TRACKING_CN0_SIGMA_DBHZ: f64 = 0.75;

pub struct NavigationAccuracyArtifactFixture {
    pub config: ReceiverPipelineConfig,
    pub profile: SyntheticPvtScenario,
    pub frame: SamplesFrame,
    pub truth: SyntheticIqTruthBundle,
    pub tracks: Vec<TrackingResult>,
    pub observations: Vec<ObsEpoch>,
    pub solutions: Vec<NavSolutionEpoch>,
    pub acquisition_accuracy: SyntheticAcquisitionAccuracyReport,
    pub tracking_accuracy: SyntheticTrackingAccuracyReport,
    pub observation_accuracy: SyntheticObservationAccuracyReport,
    pub pvt_accuracy: SyntheticPvtAccuracyReport,
    pub data_source: SyntheticGnssAccuracyDataSource,
    pub reference_truth: SyntheticGnssAccuracyReferenceTruth,
}

pub fn build_navigation_accuracy_artifact_fixture() -> NavigationAccuracyArtifactFixture {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 1_023_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 5,
        acquisition_doppler_search_hz: 2_000,
        acquisition_doppler_step_hz: 250,
        tracking_budget_ms: 100.0,
        tracking_over_budget_action: "continue".to_string(),
        tropo_enable: false,
        ..ReceiverPipelineConfig::default()
    };
    let mut profile = navigation_truth::four_satellite_pvt_scenario(&config);
    for (signal, doppler_hz) in
        profile.scenario.satellites.iter_mut().zip([-250.0, -125.0, 0.0, 125.0, 250.0])
    {
        signal.doppler_hz = doppler_hz;
    }
    let observation_reference = SyntheticObservationTruthReference {
        receive_time_s: SYNTHETIC_REFERENCE_RECEIVE_TIME_S,
        receiver_ecef_m: [profile.truth_ecef_m.0, profile.truth_ecef_m.1, profile.truth_ecef_m.2],
        ionosphere_delay_model: None,
    };
    let tracks = truth_aligned_tracks(&config, &profile, 2);
    let frame = generate_l1_ca_multi(&config, &profile.scenario);
    let truth = build_iq_truth_bundle(&profile, &frame);
    let scaled_frame = scaled_truth_frame(&frame, &truth);
    let budgets = truth_guided_receiver_accuracy_budgets();

    let acquisition_truth = validate_truth_guided_acquisition_table(
        &config,
        &scaled_frame,
        &truth,
        1,
        budgets.acquisition.max_code_phase_error_samples,
    );
    let acquisition_accuracy =
        validate_acquisition_accuracy_budget(&acquisition_truth, budgets.acquisition);

    let tracking_truth = validate_truth_guided_tracking_table(
        &config,
        &scaled_frame,
        &truth,
        budgets.tracking.max_carrier_error_hz,
        budgets.tracking.max_doppler_error_hz,
        budgets.tracking.max_code_phase_error_samples,
        budgets.tracking.max_cn0_error_db_hz,
    );
    let tracking_accuracy = validate_tracking_accuracy_budget(&tracking_truth, budgets.tracking);

    let observation_validation = validate_truth_guided_observations(
        &config,
        &tracks,
        &profile.scenario,
        &observation_reference,
        SYNTHETIC_HATCH_WINDOW,
    );
    let observation_accuracy =
        validate_observation_accuracy_budget(&observation_validation, budgets.observation);

    let gps_time = Some(GpsTime {
        week: profile.ephemerides.first().expect("synthetic navigation ephemeris").week,
        tow_s: SYNTHETIC_REFERENCE_RECEIVE_TIME_S,
    });
    let observations = observations_from_tracking_results_with_gps_anchor(
        &config,
        gps_time,
        &tracks,
        SYNTHETIC_HATCH_WINDOW,
    )
    .output
    .into_iter()
    .filter(|epoch| epoch.valid && epoch.sats.len() >= 4)
    .collect::<Vec<_>>();
    let mut navigation = Navigation::new(config.clone(), ReceiverRuntime::default());
    let solutions = observations
        .iter()
        .filter_map(|epoch| navigation.solve_epoch(epoch, &profile.ephemerides))
        .collect::<Vec<_>>();
    let pvt_reference = synthetic_pvt_reference_epochs(&profile, &solutions);
    let pvt_truth =
        validate_truth_guided_pvt_table(&profile.scenario.id, &solutions, &pvt_reference);
    let pvt_accuracy = validate_pvt_accuracy_budget(&pvt_truth, budgets.pvt);

    let data_source = SyntheticGnssAccuracyDataSource {
        source_kind: "synthetic_gps_l1_ca_capture".to_string(),
        sample_rate_hz: profile.scenario.sample_rate_hz,
        intermediate_freq_hz: profile.scenario.intermediate_freq_hz,
        duration_s: profile.scenario.duration_s,
        satellite_count: profile.scenario.satellites.len(),
    };
    let reference_truth = SyntheticGnssAccuracyReferenceTruth {
        truth_kind: "synthetic_signal_and_position_truth".to_string(),
        receiver_ecef_m: Some([
            profile.truth_ecef_m.0,
            profile.truth_ecef_m.1,
            profile.truth_ecef_m.2,
        ]),
        reference_receive_time_s: Some(SYNTHETIC_REFERENCE_RECEIVE_TIME_S),
        satellite_count: profile.ephemerides.len(),
        reference_epoch_count: pvt_reference.len(),
    };

    NavigationAccuracyArtifactFixture {
        config,
        profile,
        frame: scaled_frame,
        truth,
        tracks,
        observations,
        solutions,
        acquisition_accuracy,
        tracking_accuracy,
        observation_accuracy,
        pvt_accuracy,
        data_source,
        reference_truth,
    }
}

pub fn build_navigation_accuracy_artifact() -> SyntheticGnssAccuracyArtifact {
    let fixture = build_navigation_accuracy_artifact_fixture();

    build_truth_guided_gnss_accuracy_artifact(SyntheticGnssAccuracyArtifactCase {
        scenario_id: &fixture.profile.scenario.id,
        data_source: fixture.data_source,
        reference_truth: fixture.reference_truth,
        acquisition: &fixture.acquisition_accuracy,
        tracking: &fixture.tracking_accuracy,
        observation: &fixture.observation_accuracy,
        pvt: &fixture.pvt_accuracy,
    })
}

fn build_iq_truth_bundle(
    profile: &SyntheticPvtScenario,
    frame: &SamplesFrame,
) -> SyntheticIqTruthBundle {
    build_iq16_capture_bundle(
        &profile.scenario.id,
        &profile.scenario,
        frame,
        "2026-07-11T00:00:00Z",
        Some("navigation accuracy artifact fixture".to_string()),
    )
    .truth
}

fn scaled_truth_frame(frame: &SamplesFrame, truth: &SyntheticIqTruthBundle) -> SamplesFrame {
    SamplesFrame::new(
        frame.t0,
        frame.dt_s,
        frame.iq.iter().map(|sample| *sample * truth.output_scale_applied).collect(),
    )
}

fn synthetic_pvt_reference_epochs(
    profile: &SyntheticPvtScenario,
    solutions: &[NavSolutionEpoch],
) -> Vec<SyntheticPvtTruthReferenceEpoch> {
    let (latitude_deg, longitude_deg, altitude_m) =
        ecef_to_geodetic(profile.truth_ecef_m.0, profile.truth_ecef_m.1, profile.truth_ecef_m.2);

    solutions
        .iter()
        .map(|solution| SyntheticPvtTruthReferenceEpoch {
            position: ValidationReferenceEpoch {
                epoch_idx: solution.epoch.index,
                t_rx_s: Some(solution.t_rx_s.0),
                latitude_deg,
                longitude_deg,
                altitude_m,
                ecef_x_m: Some(profile.truth_ecef_m.0),
                ecef_y_m: Some(profile.truth_ecef_m.1),
                ecef_z_m: Some(profile.truth_ecef_m.2),
                vel_x_mps: None,
                vel_y_mps: None,
                vel_z_mps: None,
            },
            clock_bias_s: 0.0,
        })
        .collect()
}

fn truth_aligned_tracks(
    config: &ReceiverPipelineConfig,
    profile: &SyntheticPvtScenario,
    epoch_count: usize,
) -> Vec<TrackingResult> {
    profile
        .scenario
        .satellites
        .iter()
        .map(|signal| {
            synthetic_truth_track(config, signal, profile.pseudorange_epoch_base, epoch_count)
        })
        .collect()
}

fn synthetic_truth_track(
    config: &ReceiverPipelineConfig,
    signal: &bijux_gnss_receiver::api::sim::SyntheticSignalParams,
    whole_code_periods: u64,
    epoch_count: usize,
) -> TrackingResult {
    let code_phase_samples = tracking_code_phase_samples(config, signal.code_phase_chips);
    let sample_step = (config.sampling_freq_hz * 0.001).round() as u64;
    let base_carrier_phase_cycles = signal.carrier_phase_rad / std::f64::consts::TAU;

    let epochs = (0..epoch_count)
        .map(|epoch_index| {
            let sample_index = epoch_index as u64 * sample_step;
            let elapsed_s = sample_index as f64 / config.sampling_freq_hz;
            let carrier_phase_cycles = base_carrier_phase_cycles + signal.doppler_hz * elapsed_s;
            TrackEpoch {
                epoch: Epoch { index: epoch_index as u64 },
                sample_index,
                source_time: ReceiverSampleTrace::from_sample_index(
                    sample_index,
                    config.sampling_freq_hz,
                ),
                sat: signal.sat,
                signal_band: bijux_gnss_core::api::SignalBand::L1,
                signal_code: bijux_gnss_core::api::SignalCode::Unknown,
                glonass_frequency_channel: None,
                prompt_i: 1.0,
                prompt_q: 0.0,
                early_i: 0.0,
                early_q: 0.0,
                late_i: 0.0,
                late_q: 0.0,
                carrier_hz: Hertz(carrier_hz_from_doppler_hz(
                    config.intermediate_freq_hz,
                    signal.doppler_hz,
                )),
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
                lock_state_reason: Some("stable_tracking".to_string()),
                channel_id: Some(signal.sat.prn),
                channel_uid: format!("Gps-{:02}-accuracy-artifact", signal.sat.prn),
                tracking_provenance: "navigation_accuracy_artifact_truth".to_string(),
                tracking_assumptions: None,
                signal_delay_alignment: Some(SignalDelayAlignment {
                    whole_code_periods,
                    sample_delay_samples: 0,
                    source: "synthetic_truth".to_string(),
                }),
                transmit_time: None,
                tracking_uncertainty: Some(TrackingUncertainty {
                    code_phase_samples: 0.25,
                    carrier_phase_cycles: TRACKING_CARRIER_PHASE_SIGMA_CYCLES,
                    doppler_hz: TRACKING_DOPPLER_SIGMA_HZ,
                    cn0_dbhz: TRACKING_CN0_SIGMA_DBHZ,
                }),
                processing_ms: None,
            }
        })
        .collect::<Vec<_>>();

    TrackingResult {
        sat: signal.sat,
        carrier_hz: signal.doppler_hz,
        code_phase_samples,
        acquisition_hypothesis: "accepted".to_string(),
        acquisition_score: 1.0,
        acquisition_code_phase_samples: code_phase_samples.round() as usize,
        acquisition_carrier_hz: signal.doppler_hz,
        acq_to_track_state: "accepted".to_string(),
        epochs,
        transitions: Vec::new(),
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
