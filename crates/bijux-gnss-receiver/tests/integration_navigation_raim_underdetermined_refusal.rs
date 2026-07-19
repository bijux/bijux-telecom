#![allow(missing_docs)]

mod support;

use bijux_gnss_core::api::{NavRefusalClass, SolutionStatus};

use support::navigation_outlier::{
    rejected_outlier_prns, underdetermined_bad_satellite_navigation_run,
};

#[test]
fn navigation_pipeline_refuses_underdetermined_raim_exclusion() {
    let run = underdetermined_bad_satellite_navigation_run();
    let rejected_prns = rejected_outlier_prns(&run);

    assert!(
        !run.run.solutions.is_empty(),
        "expected navigation solutions from the underdetermined RAIM synthetic run",
    );
    assert!(
        run.run.solutions.iter().all(|solution| solution.status == SolutionStatus::IntegrityFailed),
        "expected every navigation epoch to refuse the underdetermined RAIM case: {:?}",
        run.run
            .solutions
            .iter()
            .map(|solution| {
                (
                    solution.epoch.index,
                    solution.status,
                    solution.refusal_class,
                    solution.used_sat_count,
                    solution.explain_reasons.clone(),
                )
            })
            .collect::<Vec<_>>(),
    );
    assert!(
        run.run.solutions.iter().all(|solution| !solution.valid),
        "expected the five-satellite RAIM case to remain invalid after refusal",
    );
    assert!(
        run.run
            .solutions
            .iter()
            .all(|solution| solution.refusal_class == Some(NavRefusalClass::InsufficientGeometry)),
        "expected underdetermined RAIM refusals to map to insufficient geometry: {:?}",
        run.run
            .solutions
            .iter()
            .map(|solution| (solution.epoch.index, solution.refusal_class))
            .collect::<Vec<_>>(),
    );
    assert!(
        run.run.solutions.iter().all(|solution| solution.used_sat_count == 4),
        "expected every refused solution to report four usable satellites after suspect removal: {:?}",
        run.run
            .solutions
            .iter()
            .map(|solution| (solution.epoch.index, solution.used_sat_count))
            .collect::<Vec<_>>(),
    );
    assert!(
        run.run.solutions.iter().all(|solution| solution
            .explain_reasons
            .iter()
            .any(|reason| reason == "raim_exclusion_underdetermined")),
        "expected every refused epoch to explain the underdetermined RAIM condition: {:?}",
        run.run
            .solutions
            .iter()
            .map(|solution| (solution.epoch.index, solution.explain_reasons.clone()))
            .collect::<Vec<_>>(),
    );
    assert!(
        run.run.solutions.iter().all(|solution| solution
            .explain_reasons
            .iter()
            .any(|reason| reason.starts_with("raim_suspect_prn="))),
        "expected every refused epoch to name a RAIM suspect: {:?}",
        run.run
            .solutions
            .iter()
            .map(|solution| (solution.epoch.index, solution.explain_reasons.clone()))
            .collect::<Vec<_>>(),
    );
    assert!(
        run.run.solutions.iter().all(|solution| solution
            .explain_reasons
            .iter()
            .any(|reason| reason == "raim_usable_satellites=4")),
        "expected every refused epoch to report four usable satellites: {:?}",
        run.run
            .solutions
            .iter()
            .map(|solution| (solution.epoch.index, solution.explain_reasons.clone()))
            .collect::<Vec<_>>(),
    );
    assert!(
        run.run.solutions.iter().all(|solution| solution
            .explain_reasons
            .iter()
            .all(|reason| reason != "raim_fault_excluded")),
        "underdetermined RAIM cases must not claim successful exclusion: {:?}",
        run.run
            .solutions
            .iter()
            .map(|solution| (solution.epoch.index, solution.explain_reasons.clone()))
            .collect::<Vec<_>>(),
    );
    assert!(!rejected_prns.is_empty(), "expected the refused run to surface a suspected outlier",);
}
