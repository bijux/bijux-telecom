#![allow(missing_docs)]

#[path = "support/navigation_accuracy_artifact.rs"]
mod navigation_accuracy_artifact;

use bijux_gnss_receiver::api::sim::{
    build_truth_guided_gnss_accuracy_artifact, SyntheticGnssAccuracyArtifactCase,
};

use navigation_accuracy_artifact::build_navigation_accuracy_artifact_fixture;

#[test]
fn navigation_accuracy_artifact_summarizes_acquisition_and_tracking_stages() {
    let fixture = build_navigation_accuracy_artifact_fixture();
    let artifact = build_truth_guided_gnss_accuracy_artifact(SyntheticGnssAccuracyArtifactCase {
        scenario_id: &fixture.profile.scenario.id,
        data_source: fixture.data_source.clone(),
        reference_truth: fixture.reference_truth.clone(),
        acquisition: &fixture.acquisition_accuracy,
        tracking: &fixture.tracking_accuracy,
        observation: &fixture.observation_accuracy,
        pvt: &fixture.pvt_accuracy,
    });

    assert_eq!(artifact.scenario_id, fixture.profile.scenario.id);
    assert_eq!(artifact.data_source, fixture.data_source);
    assert_eq!(artifact.reference_truth, fixture.reference_truth);

    assert_eq!(artifact.acquisition.summary.pass, fixture.acquisition_accuracy.pass);
    assert_eq!(
        artifact.acquisition.summary.truth_coverage_ready,
        fixture.acquisition_accuracy.truth_coverage_ready
    );
    assert_eq!(
        artifact.acquisition.summary.observed_max_doppler_error_hz,
        fixture
            .acquisition_accuracy
            .satellites
            .iter()
            .map(|satellite| satellite.doppler_error_hz)
            .max_by(f64::total_cmp)
    );
    assert_eq!(
        artifact.acquisition.summary.observed_max_code_phase_error_samples,
        fixture
            .acquisition_accuracy
            .satellites
            .iter()
            .map(|satellite| satellite.code_phase_error_samples)
            .max()
    );
    assert_eq!(
        artifact.acquisition.summary.threshold_max_doppler_error_hz,
        fixture.acquisition_accuracy.max_doppler_error_hz
    );
    assert_eq!(
        artifact.acquisition.summary.threshold_max_code_phase_error_samples,
        fixture.acquisition_accuracy.max_code_phase_error_samples
    );

    assert_eq!(artifact.tracking.summary.pass, fixture.tracking_accuracy.pass);
    assert_eq!(
        artifact.tracking.summary.truth_coverage_ready,
        fixture.tracking_accuracy.truth_coverage_ready
    );
    assert_eq!(
        artifact.tracking.summary.observed_max_carrier_error_hz,
        fixture
            .tracking_accuracy
            .satellites
            .iter()
            .map(|satellite| satellite.max_carrier_error_hz)
            .max_by(f64::total_cmp)
    );
    assert_eq!(
        artifact.tracking.summary.observed_max_doppler_error_hz,
        fixture
            .tracking_accuracy
            .satellites
            .iter()
            .map(|satellite| satellite.max_doppler_error_hz)
            .max_by(f64::total_cmp)
    );
    assert_eq!(
        artifact.tracking.summary.observed_max_code_phase_error_samples,
        fixture
            .tracking_accuracy
            .satellites
            .iter()
            .map(|satellite| satellite.max_code_phase_error_samples)
            .max_by(f64::total_cmp)
    );
    assert_eq!(
        artifact.tracking.summary.observed_max_cn0_error_db_hz,
        fixture
            .tracking_accuracy
            .satellites
            .iter()
            .map(|satellite| satellite.max_cn0_error_db_hz)
            .max_by(f64::total_cmp)
    );
    assert_eq!(
        artifact.tracking.summary.threshold_max_carrier_error_hz,
        fixture.tracking_accuracy.max_carrier_error_hz
    );
    assert_eq!(
        artifact.tracking.summary.threshold_max_doppler_error_hz,
        fixture.tracking_accuracy.max_doppler_error_hz
    );
    assert_eq!(
        artifact.tracking.summary.threshold_max_code_phase_error_samples,
        fixture.tracking_accuracy.max_code_phase_error_samples
    );
    assert_eq!(
        artifact.tracking.summary.threshold_max_cn0_error_db_hz,
        fixture.tracking_accuracy.max_cn0_error_db_hz
    );
}
