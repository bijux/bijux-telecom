#![allow(missing_docs)]

use std::fs;
use std::path::Path;

use bijux_gnss_core::api::SamplesFrame;
use bijux_gnss_receiver::api::{
    sim::{
        build_iq16_capture_bundle, generate_l1_ca_multi, truth_guided_receiver_accuracy_budgets,
        validate_truth_guided_tracking_table, SyntheticIqTruthBundle, SyntheticScenario,
    },
    ReceiverPipelineConfig,
};

#[test]
fn tracking_truth_table_covers_reference_low_and_high_rate_profiles() {
    let low_rate = build_truth_table_fixture("synthetic_iq_acquisition_reference_low_rate.toml");
    let high_rate = build_truth_table_fixture("synthetic_iq_acquisition_reference_high_rate.toml");
    let budget = truth_guided_receiver_accuracy_budgets().tracking;

    let low_rate_report = validate_truth_guided_tracking_table(
        &low_rate.config,
        &low_rate.frame,
        &low_rate.truth,
        budget.max_carrier_error_hz,
        budget.max_doppler_error_hz,
        budget.max_code_phase_error_samples,
        budget.max_cn0_error_db_hz,
    );
    let high_rate_report = validate_truth_guided_tracking_table(
        &high_rate.config,
        &high_rate.frame,
        &high_rate.truth,
        budget.max_carrier_error_hz,
        budget.max_doppler_error_hz,
        budget.max_code_phase_error_samples,
        budget.max_cn0_error_db_hz,
    );

    assert!(low_rate_report.pass, "{low_rate_report:?}");
    assert_eq!(low_rate_report.satellites.len(), 2);
    assert_eq!(high_rate_report.satellites.len(), 2);
    assert_eq!(low_rate_report.sample_rate_hz, 2_046_000.0);
    assert_eq!(high_rate_report.sample_rate_hz, 4_092_000.0);
    assert_eq!(low_rate_report.period_samples, 2046);
    assert_eq!(high_rate_report.period_samples, 4092);

    let low_rate_prn3 = low_rate_report
        .satellites
        .iter()
        .find(|satellite| satellite.sat.prn == 3)
        .expect("low-rate PRN 3 row");
    let high_rate_prn3 = high_rate_report
        .satellites
        .iter()
        .find(|satellite| satellite.sat.prn == 3)
        .expect("high-rate PRN 3 row");

    assert_eq!(low_rate_prn3.injected_code_phase_chips, 200.25);
    assert_eq!(high_rate_prn3.injected_code_phase_chips, 200.125);
    assert_ne!(
        low_rate_prn3.epochs[0].expected_code_phase_samples,
        high_rate_prn3.epochs[0].expected_code_phase_samples
    );

    for report in [&low_rate_report, &high_rate_report] {
        for satellite in &report.satellites {
            assert!(satellite.stable_epoch_count > 0, "{satellite:?}");
            assert!(
                satellite.epochs.iter().any(|epoch| epoch.stable_tracking_epoch),
                "{satellite:?}"
            );
            let passing_stable_epochs = satellite
                .epochs
                .iter()
                .filter(|epoch| epoch.stable_tracking_epoch && epoch.pass)
                .collect::<Vec<_>>();
            assert!(
                !passing_stable_epochs.is_empty(),
                "sample-rate profile must retain at least one stable truth row inside budget: satellite={satellite:?}"
            );
            for epoch in passing_stable_epochs {
                assert!(
                    epoch.carrier_error_hz <= budget.max_carrier_error_hz + f64::EPSILON,
                    "{epoch:?}"
                );
                assert!(
                    epoch.doppler_error_hz <= budget.max_doppler_error_hz + f64::EPSILON,
                    "{epoch:?}"
                );
                assert!(
                    epoch.code_phase_error_samples
                        <= budget.max_code_phase_error_samples + f64::EPSILON,
                    "{epoch:?}"
                );
                assert!(
                    epoch.cn0_error_db <= budget.max_cn0_error_db_hz + f64::EPSILON,
                    "{epoch:?}"
                );
            }
        }
    }
}

struct TruthTableFixture {
    config: ReceiverPipelineConfig,
    frame: SamplesFrame,
    truth: SyntheticIqTruthBundle,
}

fn build_truth_table_fixture(scenario_file: &str) -> TruthTableFixture {
    let scenario = load_scenario(scenario_file);
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: scenario.sample_rate_hz,
        intermediate_freq_hz: scenario.intermediate_freq_hz,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 4,
        ..ReceiverPipelineConfig::default()
    };
    let frame = generate_l1_ca_multi(&config, &scenario);
    let bundle = build_iq16_capture_bundle(
        &scenario.id,
        &scenario,
        &frame,
        "2026-07-11T00:00:00Z",
        Some("integration tracking truth table sample rates".to_string()),
    );
    let scaled_frame = SamplesFrame::new(
        frame.t0,
        frame.dt_s,
        frame.iq.iter().map(|sample| *sample * bundle.truth.output_scale_applied).collect(),
    );

    TruthTableFixture { config, frame: scaled_frame, truth: bundle.truth }
}

fn load_scenario(scenario_file: &str) -> SyntheticScenario {
    let path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join(format!("../../configs/scenarios/{scenario_file}"));
    let contents = fs::read_to_string(path).expect("scenario file");
    toml::from_str(&contents).expect("valid scenario")
}
