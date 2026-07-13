#![allow(missing_docs)]

use std::fs;
use std::path::Path;

use bijux_gnss_core::api::SamplesFrame;
use bijux_gnss_receiver::api::{
    sim::{build_iq16_capture_bundle, generate_l1_ca_multi, SyntheticIqTruthBundle, SyntheticScenario},
    ReceiverPipelineConfig,
};

#[derive(Clone)]
pub struct CompositeComponentRecoveryFixture {
    pub config: ReceiverPipelineConfig,
    pub scenario: SyntheticScenario,
}

pub fn load_composite_component_recovery_fixture(
    scenario_file: &str,
) -> CompositeComponentRecoveryFixture {
    let scenario = load_scenario(scenario_file);
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: scenario.sample_rate_hz,
        intermediate_freq_hz: scenario.intermediate_freq_hz,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: scenario.satellites.len().max(4),
        ..ReceiverPipelineConfig::default()
    };
    CompositeComponentRecoveryFixture { config, scenario }
}

pub fn build_noisy_capture(
    fixture: &CompositeComponentRecoveryFixture,
) -> (SamplesFrame, SyntheticIqTruthBundle) {
    build_capture(
        fixture,
        generate_l1_ca_multi(&fixture.config, &fixture.scenario),
        "integration composite component recovery noisy",
    )
}

pub fn scaled_capture_frame(frame: &SamplesFrame, output_scale_applied: f32) -> SamplesFrame {
    SamplesFrame::new(
        frame.t0,
        frame.dt_s,
        frame.iq.iter().map(|sample| *sample * output_scale_applied).collect(),
    )
}

fn build_capture(
    fixture: &CompositeComponentRecoveryFixture,
    frame: SamplesFrame,
    notes: &str,
) -> (SamplesFrame, SyntheticIqTruthBundle) {
    let bundle = build_iq16_capture_bundle(
        &fixture.scenario.id,
        &fixture.scenario,
        &frame,
        "2026-07-13T00:00:00Z",
        Some(notes.to_string()),
    );
    let scaled_frame = scaled_capture_frame(&frame, bundle.truth.output_scale_applied);
    (scaled_frame, bundle.truth)
}

fn load_scenario(scenario_file: &str) -> SyntheticScenario {
    let path =
        Path::new(env!("CARGO_MANIFEST_DIR")).join(format!("../../configs/scenarios/{scenario_file}"));
    let contents = fs::read_to_string(path).expect("scenario file");
    toml::from_str(&contents).expect("valid scenario")
}
