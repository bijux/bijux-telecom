#![allow(missing_docs)]

use bijux_gnss_core::api::{Constellation, SamplesFrame, SatId, SignalBand};
use bijux_gnss_receiver::api::{
    sim::{
        build_iq16_capture_bundle, generate_l1_ca_multi, truth_guided_receiver_accuracy_budgets,
        validate_truth_guided_acquisition_table, validate_truth_guided_tracking_table,
        SyntheticIqTruthBundle, SyntheticScenario, SyntheticSignalParams,
    },
    ReceiverPipelineConfig,
};

const GPS_L5_TRUTH_CODE_PHASE_TOLERANCE_SAMPLES: usize = 3;
const GPS_L5_TRUTH_DOPPLER_TOLERANCE_BINS: usize = 1;

struct GpsL5TruthCaptureFixture {
    config: ReceiverPipelineConfig,
    scenario: SyntheticScenario,
    frame: SamplesFrame,
    truth: SyntheticIqTruthBundle,
}

fn gps_l5_truth_capture_fixture() -> GpsL5TruthCaptureFixture {
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
            doppler_hz: 750.0,
            code_phase_chips: 2_048.25,
            carrier_phase_rad: 0.4,
            cn0_db_hz: 60.0,
            data_bit_flip: false,
        }],
        ephemerides: Vec::new(),
        id: "receiver-gps-l5i-truth-capture".to_string(),
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
        Some("integration gps l5i truth capture".to_string()),
    );
    let scaled_frame = SamplesFrame::new(
        frame.t0,
        frame.dt_s,
        frame.iq.iter().map(|sample| *sample * bundle.truth.output_scale_applied).collect(),
    );

    GpsL5TruthCaptureFixture {
        config,
        scenario,
        frame: scaled_frame,
        truth: bundle.truth,
    }
}

#[test]
fn gps_l5i_acquisition_truth_table_matches_capture_truth() {
    let fixture = gps_l5_truth_capture_fixture();
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
    let fixture = gps_l5_truth_capture_fixture();
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
        satellite
            .epochs
            .iter()
            .filter(|epoch| epoch.stable_tracking_epoch)
            .all(|epoch| epoch.pass),
        "{satellite:#?}"
    );
}
