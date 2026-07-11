#![allow(missing_docs)]

use std::fs;
use std::path::Path;

use bijux_gnss_core::api::SamplesFrame;
use bijux_gnss_receiver::api::{
    sim::{
        build_iq16_capture_bundle, generate_l1_ca_multi, truth_guided_receiver_accuracy_budgets,
        validate_acquisition_accuracy_budget, validate_truth_guided_acquisition_table,
        SyntheticIqTruthBundle, SyntheticScenario,
    },
    ReceiverPipelineConfig,
};

#[test]
fn acquisition_accuracy_budget_enforces_hard_truth_thresholds() {
    let fixture = build_truth_table_fixture("synthetic_iq_acquisition_reference_low_rate.toml");
    let budgets = truth_guided_receiver_accuracy_budgets();
    let truth_table = validate_truth_guided_acquisition_table(
        &fixture.config,
        &fixture.frame,
        &fixture.truth,
        1,
        2,
    );
    let report = validate_acquisition_accuracy_budget(&truth_table, budgets.acquisition);

    assert!(report.pass, "{report:?}");
    assert_eq!(report.scenario_id, truth_table.scenario_id);
    assert_eq!(report.max_doppler_error_hz, budgets.acquisition.max_doppler_error_hz);
    assert_eq!(
        report.max_code_phase_error_samples,
        budgets.acquisition.max_code_phase_error_samples
    );
    assert_eq!(report.satellite_count, truth_table.satellites.len());
    assert_eq!(report.passing_satellite_count, report.satellite_count);

    for satellite in &report.satellites {
        assert!(satellite.pass, "{satellite:?}");
        assert!(
            satellite.doppler_error_hz <= report.max_doppler_error_hz + f64::EPSILON,
            "{satellite:?}"
        );
        assert!(
            satellite.code_phase_error_samples <= report.max_code_phase_error_samples,
            "{satellite:?}"
        );
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
        acquisition_doppler_search_hz: 10_000,
        acquisition_doppler_step_hz: 500,
        ..ReceiverPipelineConfig::default()
    };
    let frame = generate_l1_ca_multi(&config, &scenario);
    let bundle = build_iq16_capture_bundle(
        &scenario.id,
        &scenario,
        &frame,
        "2026-07-11T00:00:00Z",
        Some("integration acquisition accuracy budget".to_string()),
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
