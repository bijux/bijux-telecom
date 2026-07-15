#![allow(missing_docs)]

mod support;

use bijux_gnss_core::api::{
    AcqHypothesis, AcqResult, Hertz, ReceiverSampleTrace, SamplesFrame, SatId, SignalBand,
    SignalCode, TrackEpoch,
};
use bijux_gnss_receiver::api::{
    sim::{
        expected_acquisition_code_phase_samples, generate_l1_ca_multi,
        summarize_tracking_numerical_stability, SyntheticNavigationData, SyntheticScenario,
        SyntheticSignalParams,
    },
    ReceiverPipelineConfig, ReceiverRuntime, TrackingArtifacts, TrackingEngine, TrackingResult,
};

use support::tracking_truth::carrier_phase_step_cycles;

const JOINT_TRACKING_MIN_PHASE_STEP_CYCLES: f64 = 0.01;
const JOINT_TRACKING_MAX_PHASE_STEP_CYCLES: f64 = 0.35;
const SECONDARY_CODE_STABILITY_REQUIRED_EPOCHS: usize = 40;

fn tracking_config(code_freq_basis_hz: f64, code_length: usize) -> ReceiverPipelineConfig {
    ReceiverPipelineConfig {
        sampling_freq_hz: 10_230_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz,
        code_length,
        acquisition_doppler_search_hz: 1_000,
        acquisition_doppler_step_hz: 250,
        acquisition_integration_ms: 1,
        acquisition_noncoherent: 1,
        channels: 4,
        tracking_budget_ms: 100.0,
        tracking_over_budget_action: "continue".to_string(),
        ..ReceiverPipelineConfig::default()
    }
}

fn seeded_acquisition(
    config: &ReceiverPipelineConfig,
    frame: &SamplesFrame,
    signal: &SyntheticSignalParams,
) -> AcqResult {
    AcqResult {
        sat: signal.sat,
        signal_band: signal.signal_band,
        signal_code: signal.signal_code,
        glonass_frequency_channel: signal.glonass_frequency_channel,
        source_time: ReceiverSampleTrace::default(),
        candidate_rank: 1,
        is_primary_candidate: true,
        doppler_hz: Hertz(signal.doppler_hz),
        doppler_rate_hz_per_s: 0.0,
        carrier_hz: Hertz(signal.doppler_hz),
        code_phase_samples: expected_acquisition_code_phase_samples(
            config,
            frame,
            signal.code_phase_chips,
        ),
        peak: 1.0,
        second_peak: 0.1,
        mean: 0.01,
        peak_mean_ratio: 20.0,
        peak_second_ratio: 10.0,
        cn0_proxy: signal.cn0_db_hz,
        score: 1.0,
        hypothesis: AcqHypothesis::Accepted,
        assumptions: None,
        evidence: Vec::new(),
        threshold_provenance: None,
        explain_selection_reason: Some("seeded_joint_component_tracking".to_string()),
        doppler_refinement: None,
        code_phase_refinement: None,
        signal_delay_alignment: None,
        uncertainty: None,
    }
}

fn run_seeded_tracking(
    config: &ReceiverPipelineConfig,
    scenario: &SyntheticScenario,
    tracked_signal: &SyntheticSignalParams,
) -> Vec<TrackEpoch> {
    run_seeded_tracking_artifacts(config, scenario, tracked_signal)
        .tracking
        .first()
        .expect("tracking result")
        .epochs
        .clone()
}

fn run_seeded_tracking_artifacts(
    config: &ReceiverPipelineConfig,
    scenario: &SyntheticScenario,
    tracked_signal: &SyntheticSignalParams,
) -> TrackingArtifacts {
    let frame = generate_l1_ca_multi(config, scenario);
    let tracking = TrackingEngine::new(config.clone(), ReceiverRuntime::default());
    let tracking_results: Vec<TrackingResult> = tracking
        .track_from_acquisition(&frame, &[seeded_acquisition(config, &frame, tracked_signal)]);
    TrackingArtifacts {
        processed_input_samples: frame.len() as u64,
        processed_input_epochs: (scenario.duration_s * 1000.0).round() as u64,
        tracking: tracking_results,
        ..TrackingArtifacts::default()
    }
}

fn recovered_sign_runs(epochs: &[TrackEpoch]) -> Vec<(i8, usize)> {
    let mut runs = Vec::new();
    for sign in epochs.iter().filter_map(|epoch| epoch.navigation_bit_sign) {
        if let Some((current_sign, len)) = runs.last_mut() {
            if *current_sign == sign {
                *len += 1;
                continue;
            }
        }
        runs.push((sign, 1));
    }
    runs
}

fn stable_joint_tracking_window(epochs: &[TrackEpoch]) -> &[TrackEpoch] {
    let mut best_start = 0usize;
    let mut best_len = 0usize;
    let mut run_start = None;

    for (index, epoch) in epochs.iter().enumerate() {
        let stable = epoch.lock
            && epoch.lock_state == "tracking"
            && epoch.pll_lock
            && epoch.nav_bit_lock
            && !epoch.cycle_slip
            && epoch.navigation_bit_sign.is_some();
        match (run_start, stable) {
            (None, true) => run_start = Some(index),
            (Some(start), false) => {
                let run_len = index - start;
                if run_len > best_len {
                    best_start = start;
                    best_len = run_len;
                }
                run_start = None;
            }
            _ => {}
        }
    }

    if let Some(start) = run_start {
        let run_len = epochs.len() - start;
        if run_len > best_len {
            best_start = start;
            best_len = run_len;
        }
    }

    if best_len == 0 {
        &[]
    } else {
        &epochs[best_start..best_start + best_len]
    }
}

fn assert_joint_tracking_metadata(epochs: &[TrackEpoch]) {
    assert!(
        epochs.iter().all(|epoch| {
            epoch
                .tracking_assumptions
                .as_ref()
                .is_some_and(|assumptions| assumptions.aiding_mode == "pilot_carrier")
        }),
        "tracking assumptions did not advertise pilot-carrier aiding: epochs={epochs:?}"
    );
    assert!(
        epochs.iter().all(|epoch| {
            epoch.tracking_provenance.contains("aiding_mode=pilot_carrier")
                && epoch.tracking_provenance.contains("pilot_component=true")
                && epoch.tracking_provenance.contains("data_symbol_component=true")
        }),
        "tracking provenance did not advertise the joint component model: epochs={epochs:?}"
    );
}

fn assert_secondary_code_synchronization_accepted(epochs: &[TrackEpoch]) {
    assert!(
        epochs.iter().all(|epoch| {
            epoch.tracking_provenance.contains("secondary_code_sync=accepted")
                && epoch.tracking_provenance.contains("secondary_code_phase_periods=")
                && epoch.tracking_provenance.contains("secondary_code_sync_confidence=")
                && epoch.tracking_provenance.contains("secondary_code_observed_periods=")
        }),
        "tracking provenance did not report accepted secondary-code synchronization: epochs={epochs:?}"
    );
}

fn assert_recovered_signs_follow_alternating_data(epochs: &[TrackEpoch]) {
    let stable_epochs = stable_joint_tracking_window(epochs);
    let runs = recovered_sign_runs(stable_epochs);

    assert!(
        !stable_epochs.is_empty(),
        "tracking never reached a stable joint-tracking window with recovered signs: epochs={epochs:?}"
    );
    assert!(
        runs.len() >= 2,
        "tracking did not recover enough sign runs to cover alternating data: runs={runs:?}, stable_epochs={stable_epochs:?}, epochs={epochs:?}"
    );
    assert!(
        runs.iter()
            .enumerate()
            .all(|(index, (_, len))| {
                index == 0 || index + 1 == runs.len() || *len >= 2
            }),
        "interior recovered sign runs should remain stable for more than one epoch: runs={runs:?}, stable_epochs={stable_epochs:?}, epochs={epochs:?}"
    );
    assert!(
        runs.windows(2).all(|pair| pair[0].0 == -pair[1].0),
        "recovered sign runs must alternate with the injected data pattern: runs={runs:?}, stable_epochs={stable_epochs:?}, epochs={epochs:?}"
    );
}

fn assert_carrier_continuity_across_sign_changes(epochs: &[TrackEpoch]) {
    let stable_epochs = stable_joint_tracking_window(epochs);
    let sign_change_indices = stable_epochs
        .windows(2)
        .enumerate()
        .filter_map(|(index, pair)| {
            let previous_sign = pair[0].navigation_bit_sign?;
            let current_sign = pair[1].navigation_bit_sign?;
            (previous_sign != current_sign).then_some(index + 1)
        })
        .collect::<Vec<_>>();

    assert!(
        !sign_change_indices.is_empty(),
        "tracking never recovered a data-sign transition inside stable lock: stable_epochs={stable_epochs:?}, epochs={epochs:?}"
    );

    for index in sign_change_indices {
        let previous = &stable_epochs[index - 1];
        let current = &stable_epochs[index];
        let phase_step_cycles = carrier_phase_step_cycles(previous, current);

        assert!(
            previous.lock && current.lock,
            "tracking dropped lock at a recovered transition: stable_epochs={stable_epochs:?}, epochs={epochs:?}"
        );
        assert!(
            !previous.cycle_slip && !current.cycle_slip,
            "tracking reported a cycle slip at a recovered data-sign transition: stable_epochs={stable_epochs:?}, epochs={epochs:?}"
        );
        assert!(
            phase_step_cycles.is_finite(),
            "carrier phase step must remain finite at a recovered data-sign transition: stable_epochs={stable_epochs:?}, epochs={epochs:?}"
        );
        assert!(
            phase_step_cycles >= JOINT_TRACKING_MIN_PHASE_STEP_CYCLES,
            "carrier phase must continue advancing at recovered data-sign transitions: phase_step_cycles={phase_step_cycles}, stable_epochs={stable_epochs:?}, epochs={epochs:?}"
        );
        assert!(
            phase_step_cycles <= JOINT_TRACKING_MAX_PHASE_STEP_CYCLES,
            "carrier phase step grew too large at a recovered data-sign transition: phase_step_cycles={phase_step_cycles}, stable_epochs={stable_epochs:?}, epochs={epochs:?}"
        );
    }
}

#[test]
fn gps_l2c_seeded_tracking_reports_joint_component_provenance() {
    let config = tracking_config(511_500.0, 10_230);
    let signal = SyntheticSignalParams {
        sat: SatId { constellation: bijux_gnss_core::api::Constellation::Gps, prn: 12 },
        glonass_frequency_channel: None,
        signal_band: SignalBand::L2,
        signal_code: SignalCode::L2C,
        doppler_hz: 180.0,
        code_phase_chips: 1_260.25,
        carrier_phase_rad: 0.3,
        cn0_db_hz: 58.0,
        navigation_data: SyntheticNavigationData::AlternatingStartPositive,
    };
    let scenario = SyntheticScenario {
        sample_rate_hz: config.sampling_freq_hz,
        intermediate_freq_hz: config.intermediate_freq_hz,
        receiver_clock_frequency_bias_hz: 0.0,
        duration_s: 0.080,
        seed: 0x12C0_2870,
        satellites: vec![signal.clone()],
        ephemerides: Vec::new(),
        id: "tracking-joint-components-gps-l2c".to_string(),
    };

    let epochs = run_seeded_tracking(&config, &scenario, &signal);

    assert!(epochs.iter().all(|epoch| epoch.signal_code == SignalCode::L2C), "{epochs:?}");
    assert_joint_tracking_metadata(&epochs);
}

#[test]
fn gps_l5i_tracking_uses_l5q_pilot_without_corrupting_data_signs() {
    let config = tracking_config(10_230_000.0, 10_230);
    let sat = SatId { constellation: bijux_gnss_core::api::Constellation::Gps, prn: 18 };
    let l5i_signal = SyntheticSignalParams {
        sat,
        glonass_frequency_channel: None,
        signal_band: SignalBand::L5,
        signal_code: SignalCode::L5I,
        doppler_hz: 180.0,
        code_phase_chips: 2_048.25,
        carrier_phase_rad: 0.3,
        cn0_db_hz: 60.0,
        navigation_data: SyntheticNavigationData::AlternatingStartPositive,
    };
    let l5q_signal = SyntheticSignalParams {
        sat,
        glonass_frequency_channel: None,
        signal_band: SignalBand::L5,
        signal_code: SignalCode::L5Q,
        doppler_hz: 180.0,
        code_phase_chips: 2_048.25,
        carrier_phase_rad: 0.3,
        cn0_db_hz: 60.0,
        navigation_data: SyntheticNavigationData::ConstantPositive,
    };
    let scenario = SyntheticScenario {
        sample_rate_hz: config.sampling_freq_hz,
        intermediate_freq_hz: config.intermediate_freq_hz,
        receiver_clock_frequency_bias_hz: 0.0,
        duration_s: 0.080,
        seed: 0x15A0_2871,
        satellites: vec![l5i_signal.clone(), l5q_signal],
        ephemerides: Vec::new(),
        id: "tracking-joint-components-gps-l5".to_string(),
    };

    let epochs = run_seeded_tracking(&config, &scenario, &l5i_signal);

    assert!(epochs.iter().all(|epoch| epoch.signal_code == SignalCode::L5I), "{epochs:?}");
    assert!(epochs.iter().any(|epoch| epoch.nav_bit_lock), "{epochs:?}");
    assert_joint_tracking_metadata(&epochs);
    assert_secondary_code_synchronization_accepted(&epochs);
    assert_recovered_signs_follow_alternating_data(&epochs);
    assert_carrier_continuity_across_sign_changes(&epochs);
}

#[test]
fn gps_l5i_tracking_reports_bounded_secondary_code_phase_state() {
    let config = tracking_config(10_230_000.0, 10_230);
    let sat = SatId { constellation: bijux_gnss_core::api::Constellation::Gps, prn: 18 };
    let l5i_signal = SyntheticSignalParams {
        sat,
        glonass_frequency_channel: None,
        signal_band: SignalBand::L5,
        signal_code: SignalCode::L5I,
        doppler_hz: 180.0,
        code_phase_chips: 2_048.25,
        carrier_phase_rad: 0.3,
        cn0_db_hz: 60.0,
        navigation_data: SyntheticNavigationData::AlternatingStartPositive,
    };
    let l5q_signal = SyntheticSignalParams {
        sat,
        glonass_frequency_channel: None,
        signal_band: SignalBand::L5,
        signal_code: SignalCode::L5Q,
        doppler_hz: 180.0,
        code_phase_chips: 2_048.25,
        carrier_phase_rad: 0.3,
        cn0_db_hz: 60.0,
        navigation_data: SyntheticNavigationData::ConstantPositive,
    };
    let scenario = SyntheticScenario {
        sample_rate_hz: config.sampling_freq_hz,
        intermediate_freq_hz: config.intermediate_freq_hz,
        receiver_clock_frequency_bias_hz: 0.0,
        duration_s: 0.080,
        seed: 0x15A0_2871,
        satellites: vec![l5i_signal.clone(), l5q_signal],
        ephemerides: Vec::new(),
        id: "tracking-secondary-code-numerical-stability".to_string(),
    };
    let artifacts = run_seeded_tracking_artifacts(&config, &scenario, &l5i_signal);
    let report = summarize_tracking_numerical_stability(
        &scenario.id,
        &config,
        artifacts.processed_input_samples,
        SECONDARY_CODE_STABILITY_REQUIRED_EPOCHS,
        &artifacts,
    );

    assert!(report.pass, "secondary-code numerical stability failed: {report:#?}");
    let satellite = report.satellites.first().expect("tracked secondary-code satellite");
    let secondary_code_phase =
        satellite.secondary_code_phase.as_ref().expect("secondary-code phase stability summary");
    assert!(secondary_code_phase.pass, "{satellite:#?}");
    assert!(
        secondary_code_phase.max_value <= 20.0,
        "GPS L5 secondary-code phase left expected period range: {satellite:#?}"
    );
}

#[test]
fn galileo_e5b_joint_tracking_preserves_carrier_continuity_and_data_signs() {
    let config = tracking_config(10_230_000.0, 10_230);
    let signal = SyntheticSignalParams {
        sat: SatId { constellation: bijux_gnss_core::api::Constellation::Galileo, prn: 11 },
        glonass_frequency_channel: None,
        signal_band: SignalBand::E5,
        signal_code: SignalCode::E5b,
        doppler_hz: 180.0,
        code_phase_chips: 2_048.25,
        carrier_phase_rad: 0.3,
        cn0_db_hz: 60.0,
        navigation_data: SyntheticNavigationData::AlternatingStartPositive,
    };
    let scenario = SyntheticScenario {
        sample_rate_hz: config.sampling_freq_hz,
        intermediate_freq_hz: config.intermediate_freq_hz,
        receiver_clock_frequency_bias_hz: 0.0,
        duration_s: 0.080,
        seed: 0x6A11_2872,
        satellites: vec![signal.clone()],
        ephemerides: Vec::new(),
        id: "tracking-joint-components-galileo-e5b".to_string(),
    };

    let epochs = run_seeded_tracking(&config, &scenario, &signal);

    assert!(epochs.iter().all(|epoch| epoch.signal_code == SignalCode::E5b), "{epochs:?}");
    assert!(epochs.iter().any(|epoch| epoch.nav_bit_lock), "{epochs:?}");
    assert_joint_tracking_metadata(&epochs);
    assert_recovered_signs_follow_alternating_data(&epochs);
    assert_carrier_continuity_across_sign_changes(&epochs);
}

#[test]
fn galileo_e5a_seeded_tracking_reports_joint_component_provenance() {
    let config = tracking_config(10_230_000.0, 10_230);
    let signal = SyntheticSignalParams {
        sat: SatId { constellation: bijux_gnss_core::api::Constellation::Galileo, prn: 19 },
        glonass_frequency_channel: None,
        signal_band: SignalBand::E5,
        signal_code: SignalCode::E5a,
        doppler_hz: 180.0,
        code_phase_chips: 2_048.25,
        carrier_phase_rad: 0.3,
        cn0_db_hz: 60.0,
        navigation_data: SyntheticNavigationData::AlternatingStartPositive,
    };
    let scenario = SyntheticScenario {
        sample_rate_hz: config.sampling_freq_hz,
        intermediate_freq_hz: config.intermediate_freq_hz,
        receiver_clock_frequency_bias_hz: 0.0,
        duration_s: 0.080,
        seed: 0x6A11_2873,
        satellites: vec![signal.clone()],
        ephemerides: Vec::new(),
        id: "tracking-joint-components-galileo-e5a".to_string(),
    };

    let epochs = run_seeded_tracking(&config, &scenario, &signal);

    assert!(epochs.iter().all(|epoch| epoch.signal_code == SignalCode::E5a), "{epochs:?}");
    assert!(epochs.iter().any(|epoch| epoch.nav_bit_lock), "{epochs:?}");
    assert_joint_tracking_metadata(&epochs);
    assert!(epochs.iter().any(|epoch| epoch.navigation_bit_sign.is_some()), "{epochs:?}");
}
