#![allow(missing_docs)]

use std::fs;
use std::path::Path;

use bijux_gnss_core::api::SamplesFrame;
use bijux_gnss_receiver::api::{
    sim::{
        build_iq16_capture_bundle, generate_l1_ca_multi, validate_truth_guided_acquisition_table,
        SyntheticIqTruthBundle, SyntheticScenario,
    },
    ReceiverPipelineConfig,
};

#[test]
fn acquisition_truth_table_records_estimates_and_errors_per_satellite() {
    let fixture = build_truth_table_fixture("synthetic_iq_acquisition_reference_low_rate.toml");
    let report = validate_truth_guided_acquisition_table(&fixture.config, &fixture.frame, &fixture.truth, 1, 2);

    assert!(report.pass, "{report:?}");
    assert_eq!(report.scenario_id, "synthetic_iq_acquisition_reference_low_rate");
    assert_eq!(report.doppler_tolerance_bins, 1);
    assert_eq!(report.code_phase_tolerance_samples, 2);
    assert_eq!(report.sample_rate_hz, fixture.config.sampling_freq_hz);
    assert_eq!(report.doppler_step_hz, fixture.config.acquisition_doppler_step_hz);
    assert_eq!(report.satellites.len(), 2);

    for satellite in &report.satellites {
        assert!(satellite.pass, "{satellite:?}");
        assert!(satellite.doppler_pass, "{satellite:?}");
        assert!(satellite.code_phase_pass, "{satellite:?}");
        assert!(satellite.measured_doppler_hz.is_finite(), "{satellite:?}");
        assert!(satellite.peak_mean_ratio.is_finite(), "{satellite:?}");
        assert!(satellite.doppler_error_hz <= report.doppler_tolerance_hz + f64::EPSILON, "{satellite:?}");
        assert!(satellite.code_phase_error_samples <= report.code_phase_tolerance_samples, "{satellite:?}");
        assert!(
            matches!(
                satellite.hypothesis.as_str(),
                "accepted" | "ambiguous" | "rejected" | "deferred"
            ),
            "{satellite:?}"
        );
    }

    let prn3 = report
        .satellites
        .iter()
        .find(|satellite| satellite.sat.prn == 3)
        .expect("PRN 3 row");
    assert_eq!(prn3.injected_doppler_hz, 750.0);
    assert_eq!(prn3.injected_code_phase_chips, 200.25);

    let prn7 = report
        .satellites
        .iter()
        .find(|satellite| satellite.sat.prn == 7)
        .expect("PRN 7 row");
    assert_eq!(prn7.injected_doppler_hz, -1_000.0);
    assert_eq!(prn7.injected_code_phase_chips, 321.5);
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
