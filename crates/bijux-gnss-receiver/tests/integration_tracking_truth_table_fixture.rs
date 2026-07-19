#![allow(missing_docs)]

use std::env;
use std::fs;
use std::path::Path;

use bijux_gnss_core::api::SamplesFrame;
use bijux_gnss_receiver::api::{
    sim::{
        build_iq16_capture_bundle, generate_l1_ca_multi, truth_guided_receiver_accuracy_budgets,
        validate_truth_guided_tracking_table, SyntheticIqTruthBundle, SyntheticScenario,
        SyntheticTrackingTruthTableReport,
    },
    ReceiverPipelineConfig,
};

const REGENERATE_TRACKING_TRUTH_FIXTURE_ENV: &str = "BIJUX_REGENERATE_TRACKING_TRUTH_FIXTURE";

#[test]
fn tracking_truth_table_matches_low_rate_reference_fixture() {
    let fixture = build_truth_table_fixture("synthetic_iq_acquisition_reference_low_rate.toml");
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
    if env::var_os(REGENERATE_TRACKING_TRUTH_FIXTURE_ENV).is_some() {
        write_truth_table_fixture("truth_table_reference_low_rate.json", &report);
    }
    let expected = load_truth_table_fixture("truth_table_reference_low_rate.json");

    assert_eq!(report.scenario_id, expected.scenario_id);
    assert_eq!(report.period_samples, expected.period_samples);
    assert_eq!(report.pass, expected.pass);
    assert_eq!(report.satellites.len(), expected.satellites.len());
    assert_close(
        "carrier_tolerance_hz",
        report.carrier_tolerance_hz,
        expected.carrier_tolerance_hz,
        1.0e-12,
    );
    assert_close(
        "doppler_tolerance_hz",
        report.doppler_tolerance_hz,
        expected.doppler_tolerance_hz,
        1.0e-12,
    );
    assert_close(
        "code_phase_tolerance_samples",
        report.code_phase_tolerance_samples,
        expected.code_phase_tolerance_samples,
        1.0e-12,
    );
    assert_close(
        "cn0_tolerance_db_hz",
        report.cn0_tolerance_db_hz,
        expected.cn0_tolerance_db_hz,
        1.0e-12,
    );
    assert_close("sample_rate_hz", report.sample_rate_hz, expected.sample_rate_hz, 1.0e-12);
    assert_close(
        "output_scale_applied",
        report.output_scale_applied as f64,
        expected.output_scale_applied as f64,
        1.0e-8,
    );

    for (actual_satellite, expected_satellite) in
        report.satellites.iter().zip(expected.satellites.iter())
    {
        assert_eq!(actual_satellite.sat, expected_satellite.sat);
        assert_eq!(actual_satellite.epoch_count, expected_satellite.epoch_count);
        assert_eq!(actual_satellite.stable_epoch_count, expected_satellite.stable_epoch_count);
        assert_eq!(
            actual_satellite.first_stable_epoch_index,
            expected_satellite.first_stable_epoch_index
        );
        assert_eq!(actual_satellite.pass, expected_satellite.pass);
        assert_eq!(actual_satellite.epochs.len(), expected_satellite.epochs.len());
        assert_close(
            "injected_doppler_hz",
            actual_satellite.injected_doppler_hz,
            expected_satellite.injected_doppler_hz,
            1.0e-12,
        );
        assert_close(
            "expected_measured_doppler_hz",
            actual_satellite.expected_measured_doppler_hz,
            expected_satellite.expected_measured_doppler_hz,
            1.0e-12,
        );
        assert_close(
            "injected_code_phase_chips",
            actual_satellite.injected_code_phase_chips,
            expected_satellite.injected_code_phase_chips,
            1.0e-12,
        );
        assert_close(
            "injected_cn0_db_hz",
            actual_satellite.injected_cn0_db_hz as f64,
            expected_satellite.injected_cn0_db_hz as f64,
            1.0e-12,
        );

        for (actual_epoch, expected_epoch) in
            actual_satellite.epochs.iter().zip(expected_satellite.epochs.iter())
        {
            assert_eq!(actual_epoch.epoch_index, expected_epoch.epoch_index);
            assert_eq!(actual_epoch.sample_index, expected_epoch.sample_index);
            assert_eq!(actual_epoch.lock, expected_epoch.lock);
            assert_eq!(actual_epoch.pll_lock, expected_epoch.pll_lock);
            assert_eq!(actual_epoch.dll_lock, expected_epoch.dll_lock);
            assert_eq!(actual_epoch.fll_lock, expected_epoch.fll_lock);
            assert_eq!(actual_epoch.cycle_slip, expected_epoch.cycle_slip);
            assert_eq!(actual_epoch.lock_state, expected_epoch.lock_state);
            assert_eq!(actual_epoch.lock_state_reason, expected_epoch.lock_state_reason);
            assert_eq!(actual_epoch.stable_tracking_epoch, expected_epoch.stable_tracking_epoch);
            assert_eq!(actual_epoch.pass, expected_epoch.pass);
            assert_close(
                "expected_carrier_hz",
                actual_epoch.expected_carrier_hz,
                expected_epoch.expected_carrier_hz,
                1.0e-12,
            );
            assert_close(
                "measured_carrier_hz",
                actual_epoch.measured_carrier_hz,
                expected_epoch.measured_carrier_hz,
                1.0e-9,
            );
            assert_close(
                "carrier_error_hz",
                actual_epoch.carrier_error_hz,
                expected_epoch.carrier_error_hz,
                1.0e-12,
            );
            assert_close(
                "expected_doppler_hz",
                actual_epoch.expected_doppler_hz,
                expected_epoch.expected_doppler_hz,
                1.0e-12,
            );
            assert_close(
                "measured_doppler_hz",
                actual_epoch.measured_doppler_hz,
                expected_epoch.measured_doppler_hz,
                1.0e-9,
            );
            assert_close(
                "doppler_error_hz",
                actual_epoch.doppler_error_hz,
                expected_epoch.doppler_error_hz,
                1.0e-12,
            );
            assert_close(
                "expected_code_phase_samples",
                actual_epoch.expected_code_phase_samples,
                expected_epoch.expected_code_phase_samples,
                1.0e-12,
            );
            assert_close(
                "measured_code_phase_samples",
                actual_epoch.measured_code_phase_samples,
                expected_epoch.measured_code_phase_samples,
                1.0e-9,
            );
            assert_close(
                "code_phase_error_samples",
                actual_epoch.code_phase_error_samples,
                expected_epoch.code_phase_error_samples,
                1.0e-12,
            );
            assert_close(
                "expected_cn0_db_hz",
                actual_epoch.expected_cn0_db_hz,
                expected_epoch.expected_cn0_db_hz,
                1.0e-12,
            );
            assert_close(
                "measured_cn0_dbhz",
                actual_epoch.measured_cn0_dbhz,
                expected_epoch.measured_cn0_dbhz,
                1.0e-9,
            );
            assert_close(
                "cn0_error_db",
                actual_epoch.cn0_error_db,
                expected_epoch.cn0_error_db,
                1.0e-12,
            );
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
        Some("integration tracking truth table fixture".to_string()),
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

fn load_truth_table_fixture(fixture_file: &str) -> SyntheticTrackingTruthTableReport {
    let path =
        Path::new(env!("CARGO_MANIFEST_DIR")).join(format!("tests/data/tracking/{fixture_file}"));
    let contents = fs::read_to_string(path).expect("truth table fixture");
    serde_json::from_str(&contents).expect("valid truth table fixture")
}

fn write_truth_table_fixture(fixture_file: &str, report: &SyntheticTrackingTruthTableReport) {
    let path =
        Path::new(env!("CARGO_MANIFEST_DIR")).join(format!("tests/data/tracking/{fixture_file}"));
    let contents = serde_json::to_string_pretty(report).expect("serialize truth table fixture");
    fs::write(path, format!("{contents}\n")).expect("write truth table fixture");
}

fn assert_close(label: &str, actual: f64, expected: f64, tolerance: f64) {
    let error = (actual - expected).abs();
    assert!(
        error <= tolerance,
        "{label} mismatch: actual={actual:.15} expected={expected:.15} error={error:.15} tolerance={tolerance:.15}"
    );
}
