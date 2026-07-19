#![allow(missing_docs)]

use std::fs;
use std::path::Path;

use bijux_gnss_core::api::SamplesFrame;
use bijux_gnss_receiver::api::{
    sim::{
        build_iq16_capture_bundle, generate_l1_ca_multi,
        validate_truth_guided_acquisition_sample_rates,
        SyntheticAcquisitionSampleRateValidationCase, SyntheticIqTruthBundle, SyntheticScenario,
    },
    ReceiverPipelineConfig,
};

#[test]
fn acquisition_reference_profiles_pass_validation_at_low_and_high_sample_rates() {
    let low_rate = build_validation_case("synthetic_iq_acquisition_reference_low_rate.toml");
    let high_rate = build_validation_case("synthetic_iq_acquisition_reference_high_rate.toml");

    let report =
        validate_truth_guided_acquisition_sample_rates(&[low_rate.case(), high_rate.case()], 2, 1);

    assert!(report.pass, "{report:?}");
    assert_eq!(report.distinct_sample_rate_count, 2);
    assert_eq!(report.min_sample_rate_hz, 2_046_000.0);
    assert_eq!(report.max_sample_rate_hz, 4_092_000.0);
    assert_eq!(report.points.len(), 2);
    for point in &report.points {
        assert!(point.pass, "{point:?}");
        assert!(point.code_phase_validation.pass, "{point:?}");
        assert!(point.doppler_validation.pass, "{point:?}");
        assert_eq!(point.code_phase_validation.satellites.len(), 2);
        assert_eq!(point.doppler_validation.satellites.len(), 2);
    }
}

struct ValidationCaseFixture {
    config: ReceiverPipelineConfig,
    frame: SamplesFrame,
    truth: SyntheticIqTruthBundle,
}

impl ValidationCaseFixture {
    fn case(&self) -> SyntheticAcquisitionSampleRateValidationCase<'_> {
        SyntheticAcquisitionSampleRateValidationCase {
            config: &self.config,
            frame: &self.frame,
            truth: &self.truth,
        }
    }
}

fn build_validation_case(scenario_file: &str) -> ValidationCaseFixture {
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
        "2026-07-09T00:00:00Z",
        Some("integration acquisition sample-rate validation".to_string()),
    );
    let scaled_frame = SamplesFrame::new(
        frame.t0,
        frame.dt_s,
        frame.iq.iter().map(|sample| *sample * bundle.truth.output_scale_applied).collect(),
    );

    ValidationCaseFixture { config, frame: scaled_frame, truth: bundle.truth }
}

fn load_scenario(scenario_file: &str) -> SyntheticScenario {
    let path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join(format!("../../configs/scenarios/{scenario_file}"));
    let contents = fs::read_to_string(path).expect("scenario file");
    toml::from_str(&contents).expect("valid scenario")
}
