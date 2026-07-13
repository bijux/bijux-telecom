#![allow(missing_docs)]

mod support;

use bijux_gnss_core::api::{NavHealthEvent, NavRefusalClass, SolutionStatus};

use support::navigation_residual_whiteness::{
    residual_temporal_correlation_health_events, static_residual_temporal_correlation_run,
    static_rotating_residual_pattern_run,
};

#[test]
fn navigation_pipeline_refuses_persistent_residual_temporal_correlation() {
    let run = static_residual_temporal_correlation_run();
    let flagged_solutions = run
        .solutions
        .iter()
        .filter(|solution| !residual_temporal_correlation_health_events(solution).is_empty())
        .collect::<Vec<_>>();

    assert!(
        !flagged_solutions.is_empty(),
        "expected residual temporal correlation evidence: {:?}",
        run.solutions
            .iter()
            .map(|solution| (
                solution.epoch.index,
                solution.status,
                solution.refusal_class,
                solution.explain_reasons.clone(),
                solution.health.clone(),
                solution
                    .residuals
                    .iter()
                    .filter(|residual| !residual.rejected)
                    .map(|residual| (residual.sat.prn, residual.residual_m.0))
                    .collect::<Vec<_>>(),
            ))
            .collect::<Vec<_>>(),
    );
    assert!(
        flagged_solutions
            .iter()
            .all(|solution| solution.epoch.index > run.anomaly_onset_epoch_index),
        "residual correlation should not be flagged at onset: {:?}",
        flagged_solutions.iter().map(|solution| solution.epoch.index).collect::<Vec<_>>(),
    );

    let refused_solutions = flagged_solutions
        .iter()
        .copied()
        .filter(|solution| solution.status == SolutionStatus::IntegrityFailed)
        .collect::<Vec<_>>();
    assert!(
        !refused_solutions.is_empty(),
        "persistent residual correlation should eventually refuse epochs: {:?}",
        flagged_solutions
            .iter()
            .map(|solution| (solution.epoch.index, solution.status, solution.refusal_class))
            .collect::<Vec<_>>(),
    );
    assert!(
        refused_solutions
            .iter()
            .all(|solution| solution.epoch.index >= run.expected_refusal_epoch_index),
        "refusal should start only after the persistent streak forms: {:?}",
        refused_solutions.iter().map(|solution| solution.epoch.index).collect::<Vec<_>>(),
    );
    assert!(
        refused_solutions.iter().all(|solution| {
            solution.refusal_class == Some(NavRefusalClass::InconsistentObservations)
        }),
        "persistent residual correlation should refuse as inconsistent observations: {:?}",
        refused_solutions
            .iter()
            .map(|solution| (solution.epoch.index, solution.refusal_class))
            .collect::<Vec<_>>(),
    );
    assert!(
        refused_solutions.iter().any(|solution| solution
            .explain_reasons
            .iter()
            .any(|reason| reason == "residual_whiteness")),
        "expected residual whiteness explainability: {:?}",
        refused_solutions
            .iter()
            .map(|solution| (solution.epoch.index, solution.explain_reasons.clone()))
            .collect::<Vec<_>>(),
    );
    assert!(
        refused_solutions.iter().any(|solution| solution.health.iter().any(|event| matches!(
            event,
            NavHealthEvent::ResidualTemporalCorrelation {
                lag1_correlation,
                correlation_threshold,
                matched_satellite_count,
                persistent_suspect_epochs,
                ..
            } if lag1_correlation.abs() >= *correlation_threshold
                && *matched_satellite_count >= 4
                && *persistent_suspect_epochs >= 2
        ))),
        "expected persistent residual temporal correlation evidence: {:?}",
        refused_solutions.iter().map(|solution| &solution.health).collect::<Vec<_>>(),
    );
}

#[test]
fn navigation_pipeline_requires_persistent_streak_before_refusal() {
    let run = static_residual_temporal_correlation_run();
    let first_flagged = run
        .solutions
        .iter()
        .find(|solution| !residual_temporal_correlation_health_events(solution).is_empty())
        .expect("expected at least one flagged epoch");

    assert_eq!(first_flagged.epoch.index, run.anomaly_onset_epoch_index + 1);
    assert_eq!(first_flagged.status, SolutionStatus::Degraded);
    assert_eq!(first_flagged.refusal_class, None);
    assert!(
        first_flagged.explain_reasons.iter().any(|reason| reason == "residual_whiteness"),
        "expected residual whiteness explainability on first flagged epoch: {:?}",
        first_flagged.explain_reasons,
    );
    assert!(
        first_flagged.health.iter().any(|event| matches!(
            event,
            NavHealthEvent::ResidualTemporalCorrelation {
                persistent_suspect_epochs,
                ..
            } if *persistent_suspect_epochs == 1
        )),
        "expected first flagged epoch to carry a one-epoch suspect streak: {:?}",
        first_flagged.health,
    );
}

#[test]
fn navigation_pipeline_does_not_flag_rotating_high_rms_residual_patterns() {
    let run = static_rotating_residual_pattern_run();

    assert!(
        run.solutions
            .iter()
            .all(|solution| residual_temporal_correlation_health_events(solution).is_empty()),
        "rotating residual pattern should avoid temporal-correlation classification: {:?}",
        run.solutions
            .iter()
            .map(|solution| (
                solution.epoch.index,
                solution.status,
                solution.refusal_class,
                solution
                    .residuals
                    .iter()
                    .filter(|residual| !residual.rejected)
                    .map(|residual| (residual.sat.prn, residual.residual_m.0))
                    .collect::<Vec<_>>(),
            ))
            .collect::<Vec<_>>(),
    );
    assert!(
        run.solutions
            .iter()
            .filter(|solution| solution.epoch.index >= run.anomaly_onset_epoch_index)
            .flat_map(|solution| solution.residuals.iter())
            .filter(|residual| !residual.rejected)
            .any(|residual| residual.residual_m.0.abs() >= 3.0),
        "rotating control should still carry materially large residuals after onset: {:?}",
        run.solutions
            .iter()
            .map(|solution| (
                solution.epoch.index,
                solution
                    .residuals
                    .iter()
                    .filter(|residual| !residual.rejected)
                    .map(|residual| residual.residual_m.0)
                    .collect::<Vec<_>>(),
            ))
            .collect::<Vec<_>>(),
    );
}
