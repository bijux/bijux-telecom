#![allow(missing_docs)]

use std::env;
use std::fs;
use std::path::Path;

use bijux_gnss_core::api::SamplesFrame;
use bijux_gnss_receiver::api::{
    sim::{
        build_iq16_capture_bundle, generate_l1_ca_multi, validate_truth_guided_acquisition_table,
        SyntheticAcquisitionTruthTableReport, SyntheticIqTruthBundle, SyntheticScenario,
    },
    ReceiverPipelineConfig,
};

const REGENERATE_ACQUISITION_TRUTH_FIXTURE_ENV: &str = "BIJUX_REGENERATE_ACQUISITION_TRUTH_FIXTURE";
const DOPPLER_REFINEMENT_TOLERANCE_HZ: f64 = 5.0e-5;
const PEAK_MEAN_RATIO_ULP_TOLERANCE: u32 = 4;

#[test]
fn acquisition_truth_table_matches_low_rate_reference_fixture() {
    let fixture = build_truth_table_fixture("synthetic_iq_acquisition_reference_low_rate.toml");
    let report = validate_truth_guided_acquisition_table(
        &fixture.config,
        &fixture.frame,
        &fixture.truth,
        1,
        2,
    );
    if env::var_os(REGENERATE_ACQUISITION_TRUTH_FIXTURE_ENV).is_some() {
        write_truth_table_fixture("truth_table_reference_low_rate.json", &report);
    }
    let expected = load_truth_table_fixture("truth_table_reference_low_rate.json");

    assert_eq!(report.scenario_id, expected.scenario_id);
    assert_eq!(report.doppler_tolerance_bins, expected.doppler_tolerance_bins);
    assert_eq!(report.code_phase_tolerance_samples, expected.code_phase_tolerance_samples);
    assert_eq!(report.period_samples, expected.period_samples);
    assert_eq!(report.doppler_step_hz, expected.doppler_step_hz);
    assert_eq!(report.pass, expected.pass);
    let doppler_refinement_tolerance_bins =
        DOPPLER_REFINEMENT_TOLERANCE_HZ / f64::from(report.doppler_step_hz);
    assert_close("sample_rate_hz", report.sample_rate_hz, expected.sample_rate_hz, 1.0e-12);
    assert_close(
        "doppler_tolerance_hz",
        report.doppler_tolerance_hz,
        expected.doppler_tolerance_hz,
        1.0e-12,
    );
    assert_eq!(report.satellites.len(), expected.satellites.len());

    for (actual_satellite, expected_satellite) in
        report.satellites.iter().zip(expected.satellites.iter())
    {
        assert_eq!(actual_satellite.sat, expected_satellite.sat);
        assert_eq!(
            actual_satellite.glonass_frequency_channel,
            expected_satellite.glonass_frequency_channel
        );
        assert_eq!(actual_satellite.signal_band, expected_satellite.signal_band);
        assert_eq!(actual_satellite.signal_code, expected_satellite.signal_code);
        assert_eq!(
            actual_satellite.expected_code_phase_samples,
            expected_satellite.expected_code_phase_samples
        );
        assert_eq!(
            actual_satellite.measured_code_phase_samples,
            expected_satellite.measured_code_phase_samples
        );
        assert_eq!(
            actual_satellite.code_phase_error_samples,
            expected_satellite.code_phase_error_samples
        );
        assert_eq!(actual_satellite.hypothesis, expected_satellite.hypothesis);
        assert_eq!(actual_satellite.doppler_pass, expected_satellite.doppler_pass);
        assert_eq!(actual_satellite.code_phase_pass, expected_satellite.code_phase_pass);
        assert_eq!(actual_satellite.pass, expected_satellite.pass);
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
            "measured_doppler_hz",
            actual_satellite.measured_doppler_hz,
            expected_satellite.measured_doppler_hz,
            DOPPLER_REFINEMENT_TOLERANCE_HZ,
        );
        assert_close(
            "doppler_error_hz",
            actual_satellite.doppler_error_hz,
            expected_satellite.doppler_error_hz,
            DOPPLER_REFINEMENT_TOLERANCE_HZ,
        );
        assert_close(
            "doppler_error_bins",
            actual_satellite.doppler_error_bins,
            expected_satellite.doppler_error_bins,
            doppler_refinement_tolerance_bins,
        );
        assert_close(
            "injected_code_phase_chips",
            actual_satellite.injected_code_phase_chips,
            expected_satellite.injected_code_phase_chips,
            1.0e-12,
        );
        assert_f32_close(
            "peak_mean_ratio",
            actual_satellite.peak_mean_ratio,
            expected_satellite.peak_mean_ratio,
            PEAK_MEAN_RATIO_ULP_TOLERANCE,
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
        Some("integration acquisition truth table".to_string()),
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

fn load_truth_table_fixture(fixture_file: &str) -> SyntheticAcquisitionTruthTableReport {
    let path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join(format!("tests/data/acquisition/{fixture_file}"));
    let contents = fs::read_to_string(path).expect("truth table fixture");
    serde_json::from_str(&contents).expect("valid truth table fixture")
}

fn write_truth_table_fixture(fixture_file: &str, report: &SyntheticAcquisitionTruthTableReport) {
    let path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join(format!("tests/data/acquisition/{fixture_file}"));
    let serialized = serde_json::to_string_pretty(report).expect("serialize truth table fixture");
    fs::write(path, serialized).expect("write truth table fixture");
}

fn assert_close(label: &str, actual: f64, expected: f64, tolerance: f64) {
    let error = (actual - expected).abs();
    assert!(
        error <= tolerance,
        "{label} mismatch: actual={actual:.15} expected={expected:.15} error={error:.15} tolerance={tolerance:.15}"
    );
}

fn assert_f32_close(label: &str, actual: f32, expected: f32, ulp_tolerance: u32) {
    assert!(
        actual.is_finite() && expected.is_finite() && actual >= 0.0 && expected >= 0.0,
        "{label} requires finite non-negative values: actual={actual} expected={expected}"
    );
    let ulp_error = actual.to_bits().abs_diff(expected.to_bits());
    assert!(
        ulp_error <= ulp_tolerance,
        "{label} mismatch: actual={actual:.9} expected={expected:.9} ulp_error={ulp_error} ulp_tolerance={ulp_tolerance}"
    );
}

#[test]
fn peak_mean_ratio_comparison_accepts_bounded_f32_rounding_only() {
    let reference = 64.0_f32;
    assert_f32_close(
        "within_budget",
        f32::from_bits(reference.to_bits() + PEAK_MEAN_RATIO_ULP_TOLERANCE),
        reference,
        PEAK_MEAN_RATIO_ULP_TOLERANCE,
    );

    let outside_budget = f32::from_bits(reference.to_bits() + PEAK_MEAN_RATIO_ULP_TOLERANCE + 1);
    let rejection = std::panic::catch_unwind(|| {
        assert_f32_close(
            "outside_budget",
            outside_budget,
            reference,
            PEAK_MEAN_RATIO_ULP_TOLERANCE,
        );
    });
    assert!(rejection.is_err(), "comparison must reject drift beyond its ULP budget");
}
