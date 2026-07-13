#![allow(missing_docs)]

mod support;

use bijux_gnss_core::api::{NavHealthEvent, NavRefusalClass, SolutionStatus};

use support::navigation_impossible_geometry::{
    impossible_geometry_health_events, impossible_geometry_run, terrestrial_geometry_run,
};

#[test]
fn navigation_pipeline_refuses_impossible_geometry() {
    let run = impossible_geometry_run();
    let health_events = impossible_geometry_health_events(&run.solution);

    assert!(
        !health_events.is_empty(),
        "expected impossible geometry evidence in health events: {:?}",
        run.solution.health,
    );
    assert_eq!(run.solution.status, SolutionStatus::IntegrityFailed);
    assert_eq!(
        run.solution.refusal_class,
        Some(NavRefusalClass::InconsistentObservations)
    );
    assert!(!run.solution.valid, "impossible geometry must be refused");
    assert_eq!(
        (run.solution.ecef_x_m.0, run.solution.ecef_y_m.0, run.solution.ecef_z_m.0),
        (0.0, 0.0, 0.0),
        "refused solution should not retain impossible coordinates",
    );
    assert!(
        run.solution.explain_reasons.iter().any(|reason| reason == "impossible_geometry"),
        "expected impossible geometry explainability: {:?}",
        run.solution.explain_reasons,
    );
    assert!(
        run.solution
            .explain_reasons
            .iter()
            .any(|reason| reason.starts_with("receiver_radius_m=")),
        "expected receiver radius evidence: {:?}",
        run.solution.explain_reasons,
    );
    assert!(
        run.solution.explain_reasons.iter().any(|reason| reason.starts_with("altitude_m=")),
        "expected altitude evidence: {:?}",
        run.solution.explain_reasons,
    );
    assert!(
        run.solution.health.iter().any(|event| matches!(
            event,
            NavHealthEvent::ImpossibleGeometry {
                receiver_radius_m,
                altitude_m,
                used_satellite_count,
                min_receiver_radius_m,
                min_altitude_m,
                ..
            } if *receiver_radius_m < *min_receiver_radius_m
                && *altitude_m < *min_altitude_m
                && *used_satellite_count >= 4
        )),
        "expected impossible geometry thresholds to be exceeded: {:?}",
        run.solution.health,
    );
}

#[test]
fn navigation_pipeline_accepts_terrestrial_geometry() {
    let run = terrestrial_geometry_run();

    assert!(
        impossible_geometry_health_events(&run.solution).is_empty(),
        "terrestrial truth must not be flagged as impossible geometry: {:?}",
        run.solution.health,
    );
    assert_eq!(run.solution.refusal_class, None);
    assert_ne!(run.solution.status, SolutionStatus::IntegrityFailed);
    assert!(
        !run.solution
            .explain_reasons
            .iter()
            .any(|reason| reason == "impossible_geometry"),
        "terrestrial truth must not carry impossible geometry explainability: {:?}",
        run.solution.explain_reasons,
    );
}
