#![allow(missing_docs)]

use bijux_gnss_core::api::{
    AcqHypothesis, AcqResult, Constellation, Hertz, ObservationStatus, ReceiverSampleTrace,
    SamplesFrame, SatId, SignalBand, SignalCode, TrackEpoch,
};
use bijux_gnss_receiver::api::{
    observations_from_tracking_results,
    sim::{
        build_iq16_capture_bundle, expected_acquisition_code_phase_samples, generate_l1_ca_multi,
        truth_guided_receiver_accuracy_budgets, validate_truth_guided_acquisition_table,
        validate_truth_guided_tracking_table, wrapped_code_phase_error_samples_f64,
        SyntheticIqTruthBundle, SyntheticScenario, SyntheticSignalParams, SyntheticSignalSource,
    },
    Receiver, ReceiverPipelineConfig, ReceiverRuntime, TrackingChannelState, TrackingEngine,
};

const GPS_L5_TRUTH_CODE_PHASE_TOLERANCE_SAMPLES: usize = 3;
const GPS_L5_TRUTH_DOPPLER_TOLERANCE_BINS: usize = 1;

struct GpsL5TruthCaptureFixture {
    config: ReceiverPipelineConfig,
    scenario: SyntheticScenario,
    frame: SamplesFrame,
    truth: SyntheticIqTruthBundle,
}

fn gps_l5_truth_capture_fixture(
    signal_code: SignalCode,
    scenario_id: &str,
) -> GpsL5TruthCaptureFixture {
    let scenario = SyntheticScenario {
        sample_rate_hz: 10_230_000.0,
        intermediate_freq_hz: 0.0,
        receiver_clock_frequency_bias_hz: 0.0,
        duration_s: 0.060,
        seed: 0x15AB_C5E0,
        satellites: vec![SyntheticSignalParams {
            sat: SatId { constellation: Constellation::Gps, prn: 18 },
            glonass_frequency_channel: None,
            signal_band: SignalBand::L5,
            signal_code,
            doppler_hz: 750.0,
            code_phase_chips: 2_048.25,
            carrier_phase_rad: 0.4,
            cn0_db_hz: 60.0,
            data_bit_flip: false,
        }],
        ephemerides: Vec::new(),
        id: scenario_id.to_string(),
    };
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: scenario.sample_rate_hz,
        intermediate_freq_hz: scenario.intermediate_freq_hz,
        code_freq_basis_hz: 10_230_000.0,
        code_length: 10_230,
        acquisition_doppler_search_hz: 2_000,
        acquisition_doppler_step_hz: 250,
        acquisition_integration_ms: 1,
        acquisition_noncoherent: 1,
        channels: 2,
        tracking_budget_ms: 100.0,
        tracking_over_budget_action: "continue".to_string(),
        ..ReceiverPipelineConfig::default()
    };
    let frame = generate_l1_ca_multi(&config, &scenario);
    let bundle = build_iq16_capture_bundle(
        &scenario.id,
        &scenario,
        &frame,
        "2026-07-13T00:00:00Z",
        Some(format!("integration {scenario_id}")),
    );
    let scaled_frame = SamplesFrame::new(
        frame.t0,
        frame.dt_s,
        frame.iq.iter().map(|sample| *sample * bundle.truth.output_scale_applied).collect(),
    );

    GpsL5TruthCaptureFixture { config, scenario, frame: scaled_frame, truth: bundle.truth }
}

fn gps_l5i_truth_capture_fixture() -> GpsL5TruthCaptureFixture {
    gps_l5_truth_capture_fixture(SignalCode::L5I, "receiver-gps-l5i-truth-capture")
}

fn gps_l5q_truth_capture_fixture() -> GpsL5TruthCaptureFixture {
    gps_l5_truth_capture_fixture(SignalCode::L5Q, "receiver-gps-l5q-truth-capture")
}

fn gps_l5q_continuity_scenario() -> (ReceiverPipelineConfig, SyntheticScenario) {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 10_230_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 10_230_000.0,
        code_length: 10_230,
        acquisition_doppler_search_hz: 1_000,
        acquisition_doppler_step_hz: 250,
        acquisition_integration_ms: 1,
        acquisition_noncoherent: 1,
        channels: 2,
        tracking_budget_ms: 100.0,
        tracking_over_budget_action: "continue".to_string(),
        ..ReceiverPipelineConfig::default()
    };
    let scenario = SyntheticScenario {
        sample_rate_hz: config.sampling_freq_hz,
        intermediate_freq_hz: config.intermediate_freq_hz,
        receiver_clock_frequency_bias_hz: 0.0,
        duration_s: 0.060,
        seed: 0x15AB_C5E1,
        satellites: vec![SyntheticSignalParams {
            sat: SatId { constellation: Constellation::Gps, prn: 24 },
            glonass_frequency_channel: None,
            signal_band: SignalBand::L5,
            signal_code: SignalCode::L5Q,
            doppler_hz: 0.0,
            code_phase_chips: 0.0,
            carrier_phase_rad: 0.25,
            cn0_db_hz: 60.0,
            data_bit_flip: false,
        }],
        ephemerides: Vec::new(),
        id: "receiver-gps-l5q-secondary-code-continuity".to_string(),
    };
    (config, scenario)
}

fn longest_tracking_lock_window(
    epochs: &[TrackEpoch],
    signal_code: SignalCode,
) -> Option<(usize, usize)> {
    let mut best_start = 0usize;
    let mut best_len = 0usize;
    let mut run_start = 0usize;
    let mut run_len = 0usize;

    for (idx, epoch) in epochs.iter().enumerate() {
        let locked = epoch.signal_band == SignalBand::L5
            && epoch.signal_code == signal_code
            && epoch.lock_state == "tracking"
            && epoch.lock
            && epoch.dll_lock
            && epoch.pll_lock
            && !epoch.cycle_slip;
        if locked {
            if run_len == 0 {
                run_start = idx;
            }
            run_len += 1;
            if run_len > best_len {
                best_start = run_start;
                best_len = run_len;
            }
        } else {
            run_len = 0;
        }
    }

    (best_len > 0).then_some((best_start, best_len))
}

#[test]
fn gps_l5i_acquisition_truth_table_matches_capture_truth() {
    let fixture = gps_l5i_truth_capture_fixture();
    let report = validate_truth_guided_acquisition_table(
        &fixture.config,
        &fixture.frame,
        &fixture.truth,
        GPS_L5_TRUTH_DOPPLER_TOLERANCE_BINS,
        GPS_L5_TRUTH_CODE_PHASE_TOLERANCE_SAMPLES,
    );

    assert!(report.pass, "{report:#?}");
    assert_eq!(report.scenario_id, fixture.scenario.id);
    assert_eq!(report.sample_rate_hz, fixture.config.sampling_freq_hz);
    assert_eq!(report.satellites.len(), 1);

    let satellite = &report.satellites[0];
    assert_eq!(satellite.sat, fixture.scenario.satellites[0].sat);
    assert!(satellite.pass, "{satellite:#?}");
    assert!(satellite.doppler_pass, "{satellite:#?}");
    assert!(satellite.code_phase_pass, "{satellite:#?}");
    assert_eq!(satellite.injected_doppler_hz, fixture.scenario.satellites[0].doppler_hz);
    assert_eq!(
        satellite.injected_code_phase_chips,
        fixture.scenario.satellites[0].code_phase_chips
    );
    assert!(matches!(satellite.hypothesis.as_str(), "accepted" | "ambiguous"));
}

#[test]
fn gps_l5i_tracking_truth_table_matches_capture_truth() {
    let fixture = gps_l5i_truth_capture_fixture();
    let budget = truth_guided_receiver_accuracy_budgets().tracking;
    let report = validate_truth_guided_tracking_table(
        &fixture.config,
        &fixture.frame,
        &fixture.truth,
        budget.max_carrier_error_hz,
        budget.max_doppler_error_hz,
        budget.max_code_phase_error_samples,
        budget.max_cn0_error_db_hz,
    );

    assert!(report.pass, "{report:#?}");
    assert_eq!(report.scenario_id, fixture.scenario.id);
    assert_eq!(report.sample_rate_hz, fixture.config.sampling_freq_hz);
    assert_eq!(report.satellites.len(), 1);

    let satellite = &report.satellites[0];
    assert_eq!(satellite.sat, fixture.scenario.satellites[0].sat);
    assert!(satellite.pass, "{satellite:#?}");
    assert!(satellite.stable_epoch_count > 0, "{satellite:#?}");
    assert!(satellite.first_stable_epoch_index.is_some(), "{satellite:#?}");
    assert!(satellite.epochs.iter().any(|epoch| epoch.lock_state == "tracking"), "{satellite:#?}");
    assert!(
        satellite.epochs.iter().filter(|epoch| epoch.stable_tracking_epoch).all(|epoch| epoch.pass),
        "{satellite:#?}"
    );
}

#[test]
fn gps_l5q_acquisition_truth_table_matches_capture_truth() {
    let fixture = gps_l5q_truth_capture_fixture();
    let report = validate_truth_guided_acquisition_table(
        &fixture.config,
        &fixture.frame,
        &fixture.truth,
        GPS_L5_TRUTH_DOPPLER_TOLERANCE_BINS,
        GPS_L5_TRUTH_CODE_PHASE_TOLERANCE_SAMPLES,
    );

    assert!(report.pass, "{report:#?}");
    assert_eq!(report.scenario_id, fixture.scenario.id);
    assert_eq!(report.sample_rate_hz, fixture.config.sampling_freq_hz);
    assert_eq!(report.satellites.len(), 1);

    let satellite = &report.satellites[0];
    assert_eq!(satellite.sat, fixture.scenario.satellites[0].sat);
    assert!(satellite.pass, "{satellite:#?}");
    assert!(satellite.doppler_pass, "{satellite:#?}");
    assert!(satellite.code_phase_pass, "{satellite:#?}");
    assert_eq!(satellite.injected_doppler_hz, fixture.scenario.satellites[0].doppler_hz);
    assert_eq!(
        satellite.injected_code_phase_chips,
        fixture.scenario.satellites[0].code_phase_chips
    );
    assert!(matches!(satellite.hypothesis.as_str(), "accepted" | "ambiguous"));
}

#[test]
fn receiver_run_emits_gps_l5i_artifacts_from_truth_capture() {
    let fixture = gps_l5i_truth_capture_fixture();
    let sat = fixture.scenario.satellites[0].sat;
    let expected_code_phase_samples = expected_acquisition_code_phase_samples(
        &fixture.config,
        &fixture.frame,
        fixture.scenario.satellites[0].code_phase_chips,
    ) as f64;
    let mut source = SyntheticSignalSource::new_signal_only(&fixture.config, &fixture.scenario);
    let receiver = Receiver::new(fixture.config.clone(), ReceiverRuntime::default());

    let artifacts = receiver.run(&mut source).expect("receiver run");

    let acquisition =
        artifacts.acquisitions.iter().find(|result| result.sat == sat).expect("L5 acquisition");
    assert_eq!(acquisition.signal_band, SignalBand::L5, "{acquisition:#?}");
    assert!(
        matches!(acquisition.hypothesis, AcqHypothesis::Accepted | AcqHypothesis::Ambiguous),
        "{acquisition:#?}"
    );
    assert!(
        wrapped_code_phase_error_samples_f64(
            acquisition.resolved_code_phase_samples(),
            expected_code_phase_samples,
            fixture.config.code_length,
        ) <= GPS_L5_TRUTH_CODE_PHASE_TOLERANCE_SAMPLES as f64,
        "{acquisition:#?}"
    );

    let tracking = artifacts.tracking.iter().find(|track| track.sat == sat).expect("L5 tracking");
    assert!(
        tracking
            .epochs
            .iter()
            .any(|epoch| epoch.signal_band == SignalBand::L5 && epoch.lock_state == "tracking"),
        "{tracking:#?}"
    );
    assert!(
        tracking.epochs.iter().any(|epoch| epoch.signal_band == SignalBand::L5 && epoch.dll_lock),
        "{tracking:#?}"
    );

    let observations = artifacts
        .observations
        .iter()
        .flat_map(|epoch| epoch.sats.iter())
        .filter(|observation| observation.signal_id.sat == sat)
        .collect::<Vec<_>>();
    assert!(!observations.is_empty(), "{:#?}", artifacts.observations);
    assert!(
        observations.iter().all(|observation| {
            observation.signal_id.band == SignalBand::L5
                && observation.signal_id.code == SignalCode::L5I
                && observation.metadata.signal.band == SignalBand::L5
                && observation.metadata.signal.code == SignalCode::L5I
        }),
        "{observations:#?}"
    );
    assert!(
        observations
            .iter()
            .any(|observation| observation.observation_status == ObservationStatus::Accepted),
        "{observations:#?}"
    );
}

#[test]
fn receiver_run_tracks_gps_l5q_across_secondary_code_boundaries() {
    let (config, scenario) = gps_l5q_continuity_scenario();
    let sat = scenario.satellites[0].sat;
    let frame = generate_l1_ca_multi(&config, &scenario);
    let code_phase_samples = expected_acquisition_code_phase_samples(
        &config,
        &frame,
        scenario.satellites[0].code_phase_chips,
    );
    let acquisition = AcqResult {
        sat,
        signal_band: SignalBand::L5,
        signal_code: SignalCode::L5Q,
        glonass_frequency_channel: None,
        source_time: ReceiverSampleTrace::default(),
        candidate_rank: 1,
        is_primary_candidate: true,
        doppler_hz: Hertz(scenario.satellites[0].doppler_hz),
        carrier_hz: Hertz(scenario.satellites[0].doppler_hz),
        code_phase_samples,
        peak: 1.0,
        second_peak: 0.1,
        mean: 0.01,
        peak_mean_ratio: 20.0,
        peak_second_ratio: 10.0,
        cn0_proxy: 60.0,
        score: 1.0,
        hypothesis: AcqHypothesis::Accepted,
        assumptions: None,
        evidence: Vec::new(),
        threshold_provenance: None,
        explain_selection_reason: Some("seeded_l5q_pilot_tracking".to_string()),
        doppler_refinement: None,
        code_phase_refinement: None,
        signal_delay_alignment: None,
        uncertainty: None,
    };
    let tracking = TrackingEngine::new(config.clone(), ReceiverRuntime::default());
    let mut session = tracking.begin_tracking_session(&[acquisition]);
    tracking.track_session_frame(&mut session, &frame);
    let tracking_artifacts = tracking.finish_tracking_session(session);

    let tracking = tracking_artifacts
        .tracking
        .iter()
        .find(|result| result.sat == sat)
        .expect("L5-Q tracking result");
    let (window_start, window_len) =
        longest_tracking_lock_window(&tracking.epochs, SignalCode::L5Q)
            .expect("locked GPS L5-Q tracking window");
    assert!(
        window_len >= 25,
        "locked L5-Q window must span more than one NH20 cycle: start={window_start} len={window_len}"
    );

    let lock_window = &tracking.epochs[window_start..window_start + window_len];
    assert!(
        lock_window.iter().all(|epoch| epoch.signal_code == SignalCode::L5Q),
        "{lock_window:#?}"
    );
    assert!(
        lock_window.iter().all(|epoch| epoch.lock_state_reason.as_deref() != Some("lock_lost")),
        "{lock_window:#?}"
    );

    let channel_report = tracking_artifacts
        .channel_state_reports
        .iter()
        .find(|report| report.sat == sat)
        .expect("L5-Q channel state report");
    assert_eq!(channel_report.final_state, TrackingChannelState::Locked, "{channel_report:#?}");
    assert!(
        channel_report.emitted_states.iter().all(|event| event.state != TrackingChannelState::Lost),
        "{channel_report:#?}"
    );

    let observation_report =
        observations_from_tracking_results(&config, &tracking_artifacts.tracking, 10);
    let observations = observation_report
        .output
        .iter()
        .flat_map(|epoch| epoch.sats.iter())
        .filter(|observation| observation.signal_id.sat == sat)
        .collect::<Vec<_>>();
    assert!(
        observations.len() >= 25,
        "accepted L5-Q observation span must cross an NH20 boundary: count={}",
        observations.len()
    );
    assert!(
        observations.iter().all(|observation| {
            observation.signal_id.band == SignalBand::L5
                && observation.signal_id.code == SignalCode::L5Q
                && observation.metadata.signal.band == SignalBand::L5
                && observation.metadata.signal.code == SignalCode::L5Q
        }),
        "{observations:#?}"
    );
    assert!(
        observations
            .iter()
            .any(|observation| observation.observation_status == ObservationStatus::Accepted),
        "{observations:#?}"
    );
}
