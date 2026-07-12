#![allow(missing_docs)]

mod support;

use bijux_gnss_core::api::{NavHealthEvent, NavRefusalClass, SolutionStatus};

use support::navigation_residual_whiteness::{
    residual_temporal_correlation_health_events, static_residual_temporal_correlation_run,
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
        .filter(|solution| solution.status == SolutionStatus::Invalid)
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
