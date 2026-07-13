#![allow(missing_docs)]

mod support;

use bijux_gnss_core::api::{Constellation, NavHealthEvent, NavRefusalClass, SolutionStatus};

use support::navigation_constellation_clock_inconsistency::{
    constellation_clock_inconsistency_health_events, static_constellation_clock_inconsistency_run,
    static_stable_mixed_constellation_run,
};

#[test]
fn navigation_pipeline_refuses_constellation_clock_inconsistency_after_bias_jump() {
    let run = static_constellation_clock_inconsistency_run();
    let flagged_solutions = run
        .solutions
        .iter()
        .filter(|solution| !constellation_clock_inconsistency_health_events(solution).is_empty())
        .collect::<Vec<_>>();

    assert!(
        !flagged_solutions.is_empty(),
        "expected constellation clock inconsistency classification: {:?}",
        run.solutions
            .iter()
            .map(|solution| (
                solution.epoch.index,
                solution.status,
                solution.refusal_class,
                solution.explain_reasons.clone(),
                solution.health.clone()
            ))
            .collect::<Vec<_>>(),
    );
    assert!(
        flagged_solutions
            .iter()
            .all(|solution| solution.epoch.index >= run.anomaly_onset_epoch_index),
        "constellation clock inconsistency must not be flagged before onset: {:?}",
        flagged_solutions.iter().map(|solution| solution.epoch.index).collect::<Vec<_>>(),
    );
    assert!(
        flagged_solutions
            .iter()
            .all(|solution| solution.status == SolutionStatus::IntegrityFailed),
        "clock inconsistency should refuse the mixed solution: {:?}",
        flagged_solutions
            .iter()
            .map(|solution| (solution.epoch.index, solution.status))
            .collect::<Vec<_>>(),
    );
    assert!(
        flagged_solutions.iter().all(|solution| {
            solution.refusal_class == Some(NavRefusalClass::InconsistentObservations)
        }),
        "clock inconsistency should use inconsistent-observations refusal: {:?}",
        flagged_solutions
            .iter()
            .map(|solution| (solution.epoch.index, solution.refusal_class))
            .collect::<Vec<_>>(),
    );
    assert!(
        flagged_solutions.iter().any(|solution| solution
            .explain_reasons
            .iter()
            .any(|reason| reason == "constellation_clock_inconsistency")),
        "expected explicit explainability: {:?}",
        flagged_solutions
            .iter()
            .map(|solution| (solution.epoch.index, solution.explain_reasons.clone()))
            .collect::<Vec<_>>(),
    );
    assert!(
        flagged_solutions.iter().any(|solution| solution.health.iter().any(|event| matches!(
            event,
            NavHealthEvent::ConstellationClockInconsistency {
                constellation,
                bias_step_m,
                bias_step_threshold_m,
                supporting_satellite_count,
                ..
            } if *constellation == Constellation::Galileo
                && *bias_step_m > *bias_step_threshold_m
                && *supporting_satellite_count >= 2
        ))),
        "expected galileo-wide clock inconsistency evidence: {:?}",
        flagged_solutions.iter().map(|solution| &solution.health).collect::<Vec<_>>(),
    );
}

#[test]
fn navigation_pipeline_accepts_stable_mixed_constellation_offsets() {
    let run = static_stable_mixed_constellation_run();

    assert!(
        run.solutions
            .iter()
            .all(|solution| constellation_clock_inconsistency_health_events(solution).is_empty()),
        "stable mixed-constellation offsets must not be flagged: {:?}",
        run.solutions
            .iter()
            .map(|solution| (
                solution.epoch.index,
                solution.status,
                solution.refusal_class,
                solution.health.clone(),
                solution.explain_reasons.clone()
            ))
            .collect::<Vec<_>>(),
    );
}
