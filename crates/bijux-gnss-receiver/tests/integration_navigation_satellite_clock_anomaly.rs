#![allow(missing_docs)]

mod support;

use bijux_gnss_core::api::NavHealthEvent;

use support::navigation_satellite_clock_anomaly::{
    anomaly_prn, clock_anomaly_health_events, static_satellite_clock_anomaly_run,
};

#[test]
fn navigation_pipeline_classifies_single_satellite_clock_anomaly() {
    let run = static_satellite_clock_anomaly_run();
    let flagged_solutions = run
        .solutions
        .iter()
        .filter(|solution| !clock_anomaly_health_events(solution).is_empty())
        .collect::<Vec<_>>();

    assert!(
        !flagged_solutions.is_empty(),
        "expected a clock anomaly to be classified: {:?}",
        run.solutions
            .iter()
            .map(|solution| (solution.epoch.index, solution.status, solution.explain_reasons.clone()))
            .collect::<Vec<_>>(),
    );
    assert!(
        flagged_solutions
            .iter()
            .all(|solution| solution.epoch.index >= run.anomaly_onset_epoch_index),
        "clock anomaly must not be flagged before the injected onset: {:?}",
        flagged_solutions.iter().map(|solution| solution.epoch.index).collect::<Vec<_>>(),
    );
    assert!(
        flagged_solutions.iter().any(|solution| solution
            .explain_reasons
            .iter()
            .any(|reason| reason == "satellite_clock_anomaly")),
        "expected classified solutions to explain the anomaly: {:?}",
        flagged_solutions
            .iter()
            .map(|solution| (solution.epoch.index, solution.explain_reasons.clone()))
            .collect::<Vec<_>>(),
    );
    assert!(
        flagged_solutions.iter().any(|solution| solution
            .explain_reasons
            .iter()
            .any(|reason| reason == &format!("satellite_clock_anomaly_prn={}", anomaly_prn()))),
        "expected the anomaly to name PRN {}: {:?}",
        anomaly_prn(),
        flagged_solutions
            .iter()
            .map(|solution| (solution.epoch.index, solution.explain_reasons.clone()))
            .collect::<Vec<_>>(),
    );
    assert!(
        flagged_solutions.iter().any(|solution| solution.health.iter().any(|event| matches!(
            event,
            NavHealthEvent::SatelliteClockAnomaly {
                sat,
                persistent_suspect_epochs,
                max_solution_separation_m,
                separation_threshold_m,
            }
            if sat.prn == anomaly_prn()
                && *persistent_suspect_epochs >= 2
                && *max_solution_separation_m > *separation_threshold_m
        ))),
        "expected a health event for PRN {} with persistent unresolved separation evidence: {:?}",
        anomaly_prn(),
        flagged_solutions.iter().map(|solution| &solution.health).collect::<Vec<_>>(),
    );
}
