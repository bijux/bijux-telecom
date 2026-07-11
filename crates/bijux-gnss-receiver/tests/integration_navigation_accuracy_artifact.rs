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

#[test]
fn navigation_accuracy_artifact_summarizes_observation_pvt_and_overall_status() {
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

    assert_eq!(artifact.observation.summary.pass, fixture.observation_accuracy.pass);
    assert_eq!(
        artifact.observation.summary.truth_coverage_ready,
        fixture.observation_accuracy.truth_coverage_ready
    );
    assert_eq!(
        artifact.observation.summary.observed_max_pseudorange_error_m,
        fixture
            .observation_accuracy
            .satellites
            .iter()
            .filter_map(|satellite| satellite.max_pseudorange_error_m)
            .max_by(f64::total_cmp)
    );
    assert_eq!(
        artifact.observation.summary.observed_max_carrier_phase_error_cycles,
        fixture
            .observation_accuracy
            .satellites
            .iter()
            .filter_map(|satellite| satellite.max_carrier_phase_error_cycles)
            .max_by(f64::total_cmp)
    );
    assert_eq!(
        artifact.observation.summary.observed_max_doppler_error_hz,
        fixture
            .observation_accuracy
            .satellites
            .iter()
            .filter_map(|satellite| satellite.max_doppler_error_hz)
            .max_by(f64::total_cmp)
    );
    assert_eq!(
        artifact.observation.summary.observed_max_cn0_error_db_hz,
        fixture
            .observation_accuracy
            .satellites
            .iter()
            .filter_map(|satellite| satellite.max_cn0_error_db_hz)
            .max_by(f64::total_cmp)
    );
    assert_eq!(
        artifact.observation.summary.threshold_max_pseudorange_error_m,
        fixture.observation_accuracy.max_pseudorange_error_m
    );
    assert_eq!(
        artifact.observation.summary.threshold_max_carrier_phase_error_cycles,
        fixture.observation_accuracy.max_carrier_phase_error_cycles
    );
    assert_eq!(
        artifact.observation.summary.threshold_max_doppler_error_hz,
        fixture.observation_accuracy.max_doppler_error_hz
    );
    assert_eq!(
        artifact.observation.summary.threshold_max_cn0_error_db_hz,
        fixture.observation_accuracy.max_cn0_error_db_hz
    );

    assert_eq!(artifact.pvt.summary.pass, fixture.pvt_accuracy.pass);
    assert_eq!(
        artifact.pvt.summary.truth_coverage_ready,
        fixture.pvt_accuracy.truth_coverage_ready
    );
    assert_eq!(
        artifact.pvt.summary.observed_max_position_error_3d_m,
        fixture
            .pvt_accuracy
            .epochs
            .iter()
            .map(|epoch| epoch.position_error_3d_m)
            .max_by(f64::total_cmp)
    );
    assert_eq!(
        artifact.pvt.summary.observed_max_clock_bias_error_m,
        fixture
            .pvt_accuracy
            .epochs
            .iter()
            .map(|epoch| epoch.clock_bias_error_m)
            .max_by(f64::total_cmp)
    );
    assert_eq!(
        artifact.pvt.summary.observed_max_residual_rms_m,
        fixture
            .pvt_accuracy
            .epochs
            .iter()
            .map(|epoch| epoch.residual_rms_m)
            .max_by(f64::total_cmp)
    );
    assert_eq!(
        artifact.pvt.summary.observed_max_pdop,
        fixture.pvt_accuracy.epochs.iter().map(|epoch| epoch.pdop).max_by(f64::total_cmp)
    );
    assert_eq!(
        artifact.pvt.summary.threshold_max_position_error_3d_m,
        fixture.pvt_accuracy.max_position_error_3d_m
    );
    assert_eq!(
        artifact.pvt.summary.threshold_max_clock_bias_error_m,
        fixture.pvt_accuracy.max_clock_bias_error_m
    );
    assert_eq!(
        artifact.pvt.summary.threshold_max_residual_rms_m,
        fixture.pvt_accuracy.max_residual_rms_m
    );
    assert_eq!(artifact.pvt.summary.threshold_max_pdop, fixture.pvt_accuracy.max_pdop);

    assert_eq!(
        artifact.pass,
        fixture.acquisition_accuracy.pass
            && fixture.tracking_accuracy.pass
            && fixture.observation_accuracy.pass
            && fixture.pvt_accuracy.pass
    );
    assert_eq!(
        artifact.truth_coverage_ready,
        fixture.acquisition_accuracy.truth_coverage_ready
            && fixture.tracking_accuracy.truth_coverage_ready
            && fixture.observation_accuracy.truth_coverage_ready
            && fixture.pvt_accuracy.truth_coverage_ready
    );
}
