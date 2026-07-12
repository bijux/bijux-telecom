#![allow(missing_docs)]

mod support;

use bijux_gnss_core::api::NavHealthEvent;

use support::navigation_common_code_doppler_anomaly::{
    common_anomaly_health_events, static_common_code_doppler_anomaly_run,
};
use support::navigation_clock_profile::build_navigation_clock_case;
use support::navigation_satellite_clock_anomaly::static_satellite_clock_anomaly_run;

#[test]
fn navigation_pipeline_classifies_common_code_doppler_anomaly() {
    let run = static_common_code_doppler_anomaly_run();
    let flagged_solutions = run
        .solutions
        .iter()
        .filter(|solution| !common_anomaly_health_events(solution).is_empty())
        .collect::<Vec<_>>();

    assert!(
        !flagged_solutions.is_empty(),
        "expected a common code/doppler anomaly to be classified: {:?}",
        run.solutions
            .iter()
            .map(|solution| (solution.epoch.index, solution.status, solution.explain_reasons.clone()))
            .collect::<Vec<_>>(),
    );
    assert!(
        flagged_solutions
            .iter()
            .all(|solution| solution.epoch.index >= run.anomaly_onset_epoch_index),
        "common anomaly must not be flagged before the injected onset: {:?}",
        flagged_solutions.iter().map(|solution| solution.epoch.index).collect::<Vec<_>>(),
    );
    assert!(
        flagged_solutions.iter().any(|solution| solution
            .explain_reasons
            .iter()
            .any(|reason| reason == "common_code_doppler_anomaly")),
        "expected classified solutions to explain the anomaly: {:?}",
        flagged_solutions
            .iter()
            .map(|solution| (solution.epoch.index, solution.explain_reasons.clone()))
            .collect::<Vec<_>>(),
    );
    assert!(
        flagged_solutions.iter().any(|solution| solution.health.iter().any(|event| matches!(
            event,
            NavHealthEvent::CommonCodeDopplerAnomaly {
                common_code_step_m,
                common_doppler_step_hz,
                matched_satellite_count,
                aligned_satellite_count,
                code_step_threshold_m,
                doppler_step_threshold_hz,
            } if *common_code_step_m > *code_step_threshold_m
                && *common_doppler_step_hz > *doppler_step_threshold_hz
                && *aligned_satellite_count >= 4
                && *aligned_satellite_count == *matched_satellite_count
        ))),
        "expected a health event with coherent common code/doppler evidence: {:?}",
        flagged_solutions.iter().map(|solution| &solution.health).collect::<Vec<_>>(),
    );
}

#[test]
fn navigation_pipeline_does_not_misclassify_single_satellite_clock_anomaly() {
    let run = static_satellite_clock_anomaly_run();

    assert!(
        run.solutions
            .iter()
            .all(|solution| common_anomaly_health_events(solution).is_empty()),
        "single-satellite anomaly must not be promoted to a common anomaly: {:?}",
        run.solutions
            .iter()
            .map(|solution| {
                (
                    solution.epoch.index,
                    solution.health.clone(),
                    solution.explain_reasons.clone(),
                )
            })
            .collect::<Vec<_>>(),
    );
}

#[test]
fn navigation_pipeline_does_not_flag_smooth_receiver_clock_drift() {
    let drifting_case = build_navigation_clock_case(
        support::navigation_clock_profile::synthetic_navigation_clock_profile(
            "oscillator_drift_receiver_clock",
        ),
    );

    assert!(
        drifting_case
            .solutions
            .iter()
            .all(|solution| common_anomaly_health_events(solution).is_empty()),
        "smooth receiver clock drift must not be flagged as a spoofing-like common anomaly: {:?}",
        drifting_case
            .solutions
            .iter()
            .map(|solution| {
                (
                    solution.epoch.index,
                    solution.clock_bias_s.0,
                    solution.clock_drift_s_per_s,
                    solution.health.clone(),
                )
            })
            .collect::<Vec<_>>(),
    );
}
