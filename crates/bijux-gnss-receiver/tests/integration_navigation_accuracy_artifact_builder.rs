#![allow(missing_docs)]

#[path = "support/navigation_accuracy_artifact.rs"]
mod navigation_accuracy_artifact;

use navigation_accuracy_artifact::build_navigation_accuracy_artifact_fixture;

#[test]
fn navigation_accuracy_artifact_fixture_builds_truth_ready_stage_reports() {
    let fixture = build_navigation_accuracy_artifact_fixture();

    assert!(!fixture.truth.satellites.is_empty());
    assert!(!fixture.tracks.is_empty());
    assert!(!fixture.observations.is_empty());
    assert!(!fixture.solutions.is_empty());

    assert_eq!(fixture.acquisition_accuracy.scenario_id, fixture.profile.scenario.id);
    assert_eq!(fixture.tracking_accuracy.scenario_id, fixture.profile.scenario.id);
    assert_eq!(fixture.observation_accuracy.scenario_id, fixture.profile.scenario.id);
    assert_eq!(fixture.pvt_accuracy.scenario_id, fixture.profile.scenario.id);

    assert!(fixture.acquisition_accuracy.satellite_count > 0, "{:?}", fixture.acquisition_accuracy);
    assert!(fixture.tracking_accuracy.satellite_count > 0, "{:?}", fixture.tracking_accuracy);
    assert!(fixture.observation_accuracy.satellite_count > 0, "{:?}", fixture.observation_accuracy);
    assert!(fixture.pvt_accuracy.epoch_count > 0, "{:?}", fixture.pvt_accuracy);
    assert!(
        fixture
            .observation_accuracy
            .satellites
            .iter()
            .any(|satellite| satellite.max_pseudorange_error_m.is_some()),
        "{:?}",
        fixture.observation_accuracy
    );

    assert_eq!(fixture.data_source.sample_rate_hz, fixture.config.sampling_freq_hz);
    assert_eq!(fixture.data_source.intermediate_freq_hz, fixture.config.intermediate_freq_hz);
    assert_eq!(fixture.data_source.satellite_count, fixture.profile.scenario.satellites.len());
    assert_eq!(fixture.reference_truth.satellite_count, fixture.profile.ephemerides.len());
    assert_eq!(fixture.reference_truth.reference_epoch_count, fixture.solutions.len());
}
