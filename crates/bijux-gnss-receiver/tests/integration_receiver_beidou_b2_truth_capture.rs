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

const BEIDOU_B2_TRUTH_CODE_PHASE_TOLERANCE_SAMPLES: usize = 3;
const BEIDOU_B2_TRUTH_DOPPLER_TOLERANCE_BINS: usize = 1;

struct BeidouB2TruthCaptureFixture {
    config: ReceiverPipelineConfig,
    scenario: SyntheticScenario,
    frame: SamplesFrame,
    truth: SyntheticIqTruthBundle,
}

fn beidou_b2_truth_capture_fixture() -> BeidouB2TruthCaptureFixture {
    let scenario = SyntheticScenario {
        sample_rate_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        receiver_clock_frequency_bias_hz: 0.0,
        duration_s: 0.060,
        seed: 0xB2D0_2580,
        satellites: vec![SyntheticSignalParams {
            sat: SatId { constellation: Constellation::Beidou, prn: 11 },
            glonass_frequency_channel: None,
            signal_band: SignalBand::B2,
            signal_code: SignalCode::B2I,
            doppler_hz: 750.0,
            code_phase_chips: 321.375,
            carrier_phase_rad: 0.25,
            cn0_db_hz: 60.0,
            data_bit_flip: false,
        }],
        ephemerides: Vec::new(),
        id: "receiver-beidou-b2i-truth-capture".to_string(),
    };
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: scenario.sample_rate_hz,
        intermediate_freq_hz: scenario.intermediate_freq_hz,
        code_freq_basis_hz: 2_046_000.0,
        code_length: 2046,
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
        Some("integration receiver-beidou-b2i-truth-capture".to_string()),
    );
    let scaled_frame = SamplesFrame::new(
        frame.t0,
        frame.dt_s,
        frame.iq.iter().map(|sample| *sample * bundle.truth.output_scale_applied).collect(),
    );

    BeidouB2TruthCaptureFixture { config, scenario, frame: scaled_frame, truth: bundle.truth }
}

fn longest_tracking_lock_window(epochs: &[TrackEpoch]) -> Option<(usize, usize)> {
    let mut best_start = 0usize;
    let mut best_len = 0usize;
    let mut run_start = 0usize;
    let mut run_len = 0usize;

    for (idx, epoch) in epochs.iter().enumerate() {
        let locked = epoch.signal_band == SignalBand::B2
            && epoch.signal_code == SignalCode::B2I
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
fn beidou_b2i_acquisition_truth_table_matches_capture_truth() {
    let fixture = beidou_b2_truth_capture_fixture();
    let report = validate_truth_guided_acquisition_table(
        &fixture.config,
        &fixture.frame,
        &fixture.truth,
        BEIDOU_B2_TRUTH_DOPPLER_TOLERANCE_BINS,
        BEIDOU_B2_TRUTH_CODE_PHASE_TOLERANCE_SAMPLES,
    );

    assert!(report.pass, "{report:#?}");
    assert_eq!(report.scenario_id, fixture.scenario.id);
    assert_eq!(report.satellites.len(), 1);
    assert!(report.satellites[0].pass, "{:#?}", report.satellites[0]);
}

#[test]
fn beidou_b2i_tracking_truth_table_matches_capture_truth() {
    let fixture = beidou_b2_truth_capture_fixture();
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
fn receiver_run_emits_beidou_b2i_artifacts_from_truth_capture() {
    let fixture = beidou_b2_truth_capture_fixture();
    let sat = fixture.scenario.satellites[0].sat;
    let mut source = SyntheticSignalSource::new_signal_only(&fixture.config, &fixture.scenario);
    let receiver = Receiver::new(fixture.config.clone(), ReceiverRuntime::default());

    let artifacts = receiver.run(&mut source).expect("receiver run");
    let acquisition = artifacts
        .acquisitions
        .iter()
        .find(|result| result.sat == sat)
        .expect("BeiDou B2I acquisition result");
    let tracking = artifacts
        .tracking
        .iter()
        .find(|result| result.sat == sat)
        .expect("BeiDou B2I tracking result");
    let (start, len) =
        longest_tracking_lock_window(&tracking.epochs).expect("tracking lock window");
    let observation_epoch = artifacts.observations.first().expect("BeiDou B2I observation epoch");
    let observation = observation_epoch
        .sats
        .iter()
        .find(|row| row.signal_id.sat == sat)
        .expect("BeiDou B2I observation row");

    assert_eq!(acquisition.signal_band, SignalBand::B2, "{acquisition:?}");
    assert_eq!(acquisition.signal_code, SignalCode::B2I, "{acquisition:?}");
    assert!(len >= 10, "{tracking:#?}");
    assert!(
        tracking.epochs[start..start + len].iter().all(
            |epoch| epoch.signal_band == SignalBand::B2 && epoch.signal_code == SignalCode::B2I
        ),
        "{tracking:#?}"
    );
    assert_eq!(observation.signal_id.band, SignalBand::B2, "{observation:?}");
    assert_eq!(observation.signal_id.code, SignalCode::B2I, "{observation:?}");
    assert_eq!(observation.metadata.signal.code, SignalCode::B2I, "{observation:?}");
}
