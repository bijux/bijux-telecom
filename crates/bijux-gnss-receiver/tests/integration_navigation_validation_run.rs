#![allow(missing_docs)]

use bijux_gnss_receiver::api::{
    signal::FrontEndFilterSpec,
    sim::{validate_synthetic_navigation_run, SyntheticClosureStageKind, SyntheticClosureStageStatus},
};

#[path = "support/synthetic_navigation_validation.rs"]
mod synthetic_navigation_validation;

use synthetic_navigation_validation::bounded_navigation_validation_fixture;

#[test]
fn synthetic_navigation_validation_run_builds_run_level_accuracy_artifact() {
    let (config, scenario) = bounded_navigation_validation_fixture();

    let run = validate_synthetic_navigation_run(
        &config.to_pipeline_config(),
        &scenario,
        config.navigation.hatch_window,
    )
    .expect("build synthetic navigation validation run");

    assert_eq!(run.artifact.scenario_id, "synthetic_navigation_accuracy");
    assert_eq!(run.artifact.data_source.satellite_count, scenario.satellites.len());
    assert_eq!(run.artifact.reference_truth.satellite_count, scenario.ephemerides.len());
    assert_eq!(run.acquisition_accuracy.satellite_count, scenario.satellites.len());
    assert_eq!(run.tracking_accuracy.satellite_count, scenario.satellites.len());
    assert_eq!(run.observation_accuracy.satellite_count, scenario.satellites.len());
    assert!(run.pvt_accuracy.epoch_count > 0);
    assert!(run.artifact.closure_ready, "closure stage evidence: {:#?}", run.artifact.closure);
    assert!(run.artifact.closure.pass);
    assert_eq!(run.artifact.closure.applicable_stage_count, 9);
    assert_eq!(run.artifact.closure.passed_stage_count, 9);
    assert_eq!(run.artifact.closure.not_applicable_stage_count, 1);
    assert_eq!(
        run.artifact.closure.stages.iter().map(|stage| stage.stage).collect::<Vec<_>>(),
        vec![
            SyntheticClosureStageKind::IqInput,
            SyntheticClosureStageKind::Acquisition,
            SyntheticClosureStageKind::Tracking,
            SyntheticClosureStageKind::NavigationDataTiming,
            SyntheticClosureStageKind::Observations,
            SyntheticClosureStageKind::SatelliteStates,
            SyntheticClosureStageKind::Corrections,
            SyntheticClosureStageKind::Estimator,
            SyntheticClosureStageKind::AmbiguityProcessing,
            SyntheticClosureStageKind::IntegrityDecision,
        ]
    );
    let ambiguity = run
        .artifact
        .closure
        .stages
        .iter()
        .find(|stage| stage.stage == SyntheticClosureStageKind::AmbiguityProcessing)
        .expect("ambiguity closure stage");
    assert_eq!(ambiguity.status, SyntheticClosureStageStatus::NotApplicable);
    assert_eq!(ambiguity.reason, "code_only_spp_path_has_no_carrier_ambiguity_processing");
}

#[test]
fn synthetic_navigation_validation_run_carries_source_and_receiver_front_end_delay() {
    let (mut config, mut scenario) = bounded_navigation_validation_fixture();
    let source_front_end_filter = FrontEndFilterSpec::LowPass { cutoff_hz: 400_000.0, taps: 17 };
    let receiver_front_end_filter =
        FrontEndFilterSpec::LowPass { cutoff_hz: 350_000.0, taps: 21 };
    let combined_sample_delay_samples = source_front_end_filter.group_delay_samples() as u64
        + receiver_front_end_filter.group_delay_samples() as u64;

    config.front_end.filter = Some(receiver_front_end_filter.clone());
    scenario.source_front_end_filter = Some(source_front_end_filter.clone());

    let run = validate_synthetic_navigation_run(
        &config.to_pipeline_config(),
        &scenario,
        config.navigation.hatch_window,
    )
    .expect("build synthetic navigation validation run with front-end filters");

    assert_eq!(run.truth_bundle.source_front_end_filter, Some(source_front_end_filter.clone()));
    assert_eq!(
        run.truth_bundle.source_front_end_sample_delay_samples,
        source_front_end_filter.group_delay_samples() as u64
    );
    assert!(!run.pipeline_artifacts.acquisitions.is_empty());
    for acquisition in &run.pipeline_artifacts.acquisitions {
        let alignment = acquisition
            .signal_delay_alignment
            .as_ref()
            .expect("synthetic alignment on acquisition");
        assert_eq!(alignment.sample_delay_samples, combined_sample_delay_samples);
    }
    assert_eq!(run.acquisition_accuracy.satellite_count, scenario.satellites.len());
    assert_eq!(run.tracking_accuracy.satellite_count, scenario.satellites.len());
    assert_eq!(run.observation_accuracy.satellite_count, scenario.satellites.len());
    assert!(run.artifact.closure_ready, "closure stage evidence: {:#?}", run.artifact.closure);
    for stage in &run.artifact.closure.stages {
        assert!(
            stage.status != SyntheticClosureStageStatus::Failed,
            "closure stage failed: {stage:?}"
        );
    }
}
