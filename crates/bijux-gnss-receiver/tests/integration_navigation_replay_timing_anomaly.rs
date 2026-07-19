#![allow(missing_docs)]

mod support;

use bijux_gnss_core::api::NavHealthEvent;

use support::navigation_replay_timing_anomaly::{
    replay_timing_health_events, static_replay_timing_anomaly_run,
    static_single_satellite_delay_step_run, static_uniform_delay_step_run,
};

#[test]
fn navigation_pipeline_classifies_replay_timing_anomaly_after_delay_onset() {
    let run = static_replay_timing_anomaly_run();
    let flagged_solutions = run
        .solutions
        .iter()
        .filter(|solution| !replay_timing_health_events(solution).is_empty())
        .collect::<Vec<_>>();

    assert!(
        !flagged_solutions.is_empty(),
        "expected replay timing anomaly classification: {:?}",
        run.solutions
            .iter()
            .map(|solution| (
                solution.epoch.index,
                solution.status,
                solution.explain_reasons.clone(),
                solution.health.clone()
            ))
            .collect::<Vec<_>>(),
    );
    assert!(
        flagged_solutions
            .iter()
            .all(|solution| solution.epoch.index >= run.anomaly_onset_epoch_index),
        "replay timing anomaly must not be flagged before onset: {:?}",
        flagged_solutions.iter().map(|solution| solution.epoch.index).collect::<Vec<_>>(),
    );
    assert!(
        flagged_solutions.iter().all(|solution| solution
            .explain_reasons
            .iter()
            .any(|reason| reason == "replay_timing_anomaly")),
        "expected replay timing explainability: {:?}",
        flagged_solutions
            .iter()
            .map(|solution| (solution.epoch.index, solution.explain_reasons.clone()))
            .collect::<Vec<_>>(),
    );
    assert!(
        flagged_solutions.iter().any(|solution| solution.health.iter().any(|event| matches!(
            event,
            NavHealthEvent::ReplayTimingAnomaly {
                common_delay_step_m,
                centered_delay_rms_m,
                max_centered_delay_m,
                matched_satellite_count,
                positive_step_satellite_count,
                common_delay_step_threshold_m,
                centered_delay_rms_threshold_m,
            } if *common_delay_step_m > *common_delay_step_threshold_m
                && *centered_delay_rms_m > *centered_delay_rms_threshold_m
                && *max_centered_delay_m >= 40.0
                && *matched_satellite_count >= 4
                && *positive_step_satellite_count >= 4
        ))),
        "expected replay timing health evidence: {:?}",
        flagged_solutions.iter().map(|solution| &solution.health).collect::<Vec<_>>(),
    );
}

#[test]
fn navigation_pipeline_ignores_uniform_delay_step() {
    let run = static_uniform_delay_step_run();

    assert!(
        run.solutions.iter().all(|solution| replay_timing_health_events(solution).is_empty()),
        "uniform delay step must not be promoted to replay timing anomaly: {:?}",
        run.solutions
            .iter()
            .map(|solution| (
                solution.epoch.index,
                solution.health.clone(),
                solution.explain_reasons.clone()
            ))
            .collect::<Vec<_>>(),
    );
}

#[test]
fn navigation_pipeline_ignores_single_satellite_delay_step() {
    let run = static_single_satellite_delay_step_run();

    assert!(
        run.solutions.iter().all(|solution| replay_timing_health_events(solution).is_empty()),
        "single-satellite delay step must not be promoted to replay timing anomaly: {:?}",
        run.solutions
            .iter()
            .map(|solution| (
                solution.epoch.index,
                solution.health.clone(),
                solution.explain_reasons.clone()
            ))
            .collect::<Vec<_>>(),
    );
}
