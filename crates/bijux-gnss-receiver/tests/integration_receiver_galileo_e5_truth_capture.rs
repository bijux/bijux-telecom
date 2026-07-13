#![allow(missing_docs)]

use bijux_gnss_core::api::{
    Constellation, SamplesFrame, SatId, SignalBand, SignalCode, TrackEpoch,
};
use bijux_gnss_receiver::api::{
    sim::{
        build_iq16_capture_bundle, generate_l1_ca_multi, truth_guided_receiver_accuracy_budgets,
        validate_truth_guided_acquisition_table, validate_truth_guided_tracking_table,
        SyntheticIqTruthBundle, SyntheticScenario, SyntheticSignalParams, SyntheticSignalSource,
    },
    Receiver, ReceiverPipelineConfig, ReceiverRuntime,
};

const GALILEO_E5A_TRUTH_CODE_PHASE_TOLERANCE_SAMPLES: usize = 3;
const GALILEO_E5A_TRUTH_DOPPLER_TOLERANCE_BINS: usize = 1;

struct GalileoE5aTruthCaptureFixture {
    config: ReceiverPipelineConfig,
    scenario: SyntheticScenario,
    frame: SamplesFrame,
    truth: SyntheticIqTruthBundle,
}

fn galileo_e5a_truth_capture_fixture(
    scenario_id: &str,
    duration_s: f64,
) -> GalileoE5aTruthCaptureFixture {
    let scenario = SyntheticScenario {
        sample_rate_hz: 10_230_000.0,
        intermediate_freq_hz: 0.0,
        receiver_clock_frequency_bias_hz: 0.0,
        duration_s,
        seed: 0x6AE5_A000,
        satellites: vec![SyntheticSignalParams {
            sat: SatId { constellation: Constellation::Galileo, prn: 18 },
            glonass_frequency_channel: None,
            signal_band: SignalBand::E5,
            signal_code: SignalCode::E5a,
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

    GalileoE5aTruthCaptureFixture { config, scenario, frame: scaled_frame, truth: bundle.truth }
}

fn longest_tracking_lock_window(epochs: &[TrackEpoch]) -> Option<(usize, usize)> {
    let mut best_start = 0usize;
    let mut best_len = 0usize;
    let mut run_start = 0usize;
    let mut run_len = 0usize;

    for (idx, epoch) in epochs.iter().enumerate() {
        let locked = epoch.signal_band == SignalBand::E5
            && epoch.signal_code == SignalCode::E5a
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
fn galileo_e5a_acquisition_truth_table_matches_capture_truth() {
    let fixture = galileo_e5a_truth_capture_fixture("receiver-galileo-e5a-truth-capture", 0.060);
    let report = validate_truth_guided_acquisition_table(
        &fixture.config,
        &fixture.frame,
        &fixture.truth,
        GALILEO_E5A_TRUTH_DOPPLER_TOLERANCE_BINS,
        GALILEO_E5A_TRUTH_CODE_PHASE_TOLERANCE_SAMPLES,
    );

    assert!(report.pass, "{report:#?}");
    assert_eq!(report.scenario_id, fixture.scenario.id);
    assert_eq!(report.satellites.len(), 1);
    assert!(report.satellites[0].pass, "{:#?}", report.satellites[0]);
}

#[test]
fn galileo_e5a_tracking_truth_table_matches_capture_truth() {
    let fixture = galileo_e5a_truth_capture_fixture("receiver-galileo-e5a-truth-capture", 0.060);
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
    assert_eq!(report.satellites.len(), 1);
    assert!(report.satellites[0].pass, "{:#?}", report.satellites[0]);
    assert!(report.satellites[0].stable_epoch_count > 0, "{:#?}", report.satellites[0]);
}

#[test]
fn receiver_finds_sustained_galileo_e5a_lock_window() {
    let fixture = galileo_e5a_truth_capture_fixture("receiver-galileo-e5a-truth-capture", 0.060);
    let mut source = SyntheticSignalSource::new_signal_only(&fixture.config, &fixture.scenario);
    let receiver = Receiver::new(fixture.config.clone(), ReceiverRuntime::default());

    let artifacts = receiver.run(&mut source).expect("receiver run");
    let tracking = artifacts
        .tracking
        .iter()
        .find(|result| result.sat == fixture.scenario.satellites[0].sat)
        .expect("Galileo E5a tracking result");
    let (start, len) =
        longest_tracking_lock_window(&tracking.epochs).expect("tracking lock window");
    let lock_window = &tracking.epochs[start..start + len];

    assert!(len >= 10, "{tracking:#?}");
    assert!(lock_window.iter().all(|epoch| epoch.signal_code == SignalCode::E5a), "{tracking:#?}");
}
