#![allow(missing_docs)]

#[path = "support/navigation_clock_profile.rs"]
mod navigation_clock_profile;

use bijux_gnss_receiver::api::sim::{
    summarize_truth_guided_pvt_clock_profile, SyntheticPvtClockProfileCase,
};
use navigation_clock_profile::{
    build_navigation_clock_case, receiver_clock_drift_doppler_offset_hz,
    synthetic_navigation_clock_profiles, truth_clock_drift_s_per_s,
};

#[test]
fn synthetic_navigation_clock_profiles_produce_truth_ready_navigation_cases() {
    let profiles = synthetic_navigation_clock_profiles();
    assert_eq!(profiles.len(), 2);

    for profile in profiles {
        let expected_drift_s_per_s = truth_clock_drift_s_per_s(&profile);
        let expected_epoch_count = profile.truth_epochs.len();
        let case = build_navigation_clock_case(profile.clone());

        assert_eq!(case.clock_profile.profile_name, profile.profile_name);
        assert_eq!(case.truth_table.scenario_id, case.scenario_id);
        assert_eq!(case.truth_table.solution_count, case.solutions.len());
        assert_eq!(case.truth_table.matched_epoch_count, case.solutions.len());
        assert_eq!(case.truth_table.epochs.len(), case.solutions.len());
        assert!(case.truth_table.unmatched_solution_epochs.is_empty(), "{:?}", case.truth_table);
        assert!(case.truth_table.unused_reference_epochs.is_empty(), "{:?}", case.truth_table);
        assert_eq!(case.clock_profile.truth_epochs.len(), expected_epoch_count);
        assert_eq!(truth_clock_drift_s_per_s(&case.clock_profile), expected_drift_s_per_s);
        assert!(!case.observations.is_empty());
        assert!(!case.solutions.is_empty());
    }
}

#[test]
fn navigation_clock_profile_report_tracks_injected_clock_drift() {
    let cases = synthetic_navigation_clock_profiles()
        .into_iter()
        .map(build_navigation_clock_case)
        .collect::<Vec<_>>();
    let stable_case = cases
        .iter()
        .find(|case| case.clock_profile.profile_name == "stable_receiver_clock")
        .expect("stable receiver clock case");
    let report = summarize_truth_guided_pvt_clock_profile(
        &cases
            .iter()
            .map(|case| {
                let expected_observation_doppler_offset_hz = receiver_clock_drift_doppler_offset_hz(
                    truth_clock_drift_s_per_s(&case.clock_profile),
                );
                let reference_observations = (case.clock_profile.profile_name
                    != "stable_receiver_clock")
                    .then_some(stable_case.observations.as_slice());

                SyntheticPvtClockProfileCase {
                    scenario_id: &case.scenario_id,
                    injected_clock_drift_s_per_s: truth_clock_drift_s_per_s(&case.clock_profile),
                    expected_observation_doppler_offset_hz,
                    observations: &case.observations,
                    reference_observations,
                    solutions: &case.solutions,
                    accuracy: &case.pvt_accuracy,
                }
            })
            .collect::<Vec<_>>(),
        "navigation_clock_profile",
    );

    assert_eq!(report.points.len(), cases.len());

    let stable = &report.points[0];
    let drifting = &report.points[1];

    assert!(stable.ready && drifting.ready, "{report:?}");
    assert_eq!(stable.injected_clock_drift_s_per_s, 0.0);
    assert!(
        stable.final_solved_clock_drift_s_per_s.expect("stable drift").abs() <= 1.0e-12,
        "{report:?}"
    );

    assert!(drifting.injected_clock_drift_s_per_s > stable.injected_clock_drift_s_per_s);
    assert_eq!(drifting.epoch_count, drifting.passing_epoch_count, "{report:?}");
    assert_eq!(drifting.pass_rate, 1.0, "{report:?}");
    assert!(
        (drifting.final_solved_clock_drift_s_per_s.expect("final solved drift")
            - drifting.injected_clock_drift_s_per_s)
            .abs()
            <= 1.0e-8,
        "{report:?}"
    );
    assert!(drifting.max_residual_rms_m.expect("residual rms") >= 0.0, "{report:?}");
}

#[test]
fn navigation_clock_profile_report_tracks_clock_induced_doppler_offset() {
    let cases = synthetic_navigation_clock_profiles()
        .into_iter()
        .map(build_navigation_clock_case)
        .collect::<Vec<_>>();
    let stable_case = cases
        .iter()
        .find(|case| case.clock_profile.profile_name == "stable_receiver_clock")
        .expect("stable receiver clock case");
    let drifting_case = cases
        .iter()
        .find(|case| case.clock_profile.profile_name == "oscillator_drift_receiver_clock")
        .expect("oscillator drift receiver clock case");
    let expected_doppler_offset_hz = receiver_clock_drift_doppler_offset_hz(
        truth_clock_drift_s_per_s(&drifting_case.clock_profile),
    );
    let report = summarize_truth_guided_pvt_clock_profile(
        &[
            SyntheticPvtClockProfileCase {
                scenario_id: &stable_case.scenario_id,
                injected_clock_drift_s_per_s: truth_clock_drift_s_per_s(&stable_case.clock_profile),
                expected_observation_doppler_offset_hz: 0.0,
                observations: &stable_case.observations,
                reference_observations: None,
                solutions: &stable_case.solutions,
                accuracy: &stable_case.pvt_accuracy,
            },
            SyntheticPvtClockProfileCase {
                scenario_id: &drifting_case.scenario_id,
                injected_clock_drift_s_per_s: truth_clock_drift_s_per_s(
                    &drifting_case.clock_profile,
                ),
                expected_observation_doppler_offset_hz: expected_doppler_offset_hz,
                observations: &drifting_case.observations,
                reference_observations: Some(&stable_case.observations),
                solutions: &drifting_case.solutions,
                accuracy: &drifting_case.pvt_accuracy,
            },
        ],
        "navigation_clock_profile",
    );

    let drifting = report
        .points
        .iter()
        .find(|point| point.scenario_id == drifting_case.scenario_id)
        .expect("drifting clock profile point");

    assert!(drifting.ready, "{report:?}");
    assert!(drifting.observation_doppler_pair_count > 0, "{report:?}");
    assert_eq!(drifting.expected_observation_doppler_offset_hz, expected_doppler_offset_hz);
    assert!(
        (drifting
            .observed_mean_observation_doppler_offset_hz
            .expect("observed mean doppler offset")
            - expected_doppler_offset_hz)
            .abs()
            <= 1.0e-9,
        "{report:?}"
    );
    assert_eq!(drifting.max_observation_doppler_offset_error_hz, Some(0.0), "{report:?}");
}

#[test]
fn navigation_clock_profile_report_requires_reference_observations_for_drift_validation() {
    let drifting_case = build_navigation_clock_case(
        synthetic_navigation_clock_profiles()
            .into_iter()
            .find(|profile| profile.profile_name == "oscillator_drift_receiver_clock")
            .expect("oscillator drift receiver clock profile"),
    );
    let report = summarize_truth_guided_pvt_clock_profile(
        &[SyntheticPvtClockProfileCase {
            scenario_id: &drifting_case.scenario_id,
            injected_clock_drift_s_per_s: truth_clock_drift_s_per_s(&drifting_case.clock_profile),
            expected_observation_doppler_offset_hz: receiver_clock_drift_doppler_offset_hz(
                truth_clock_drift_s_per_s(&drifting_case.clock_profile),
            ),
            observations: &drifting_case.observations,
            reference_observations: None,
            solutions: &drifting_case.solutions,
            accuracy: &drifting_case.pvt_accuracy,
        }],
        "navigation_clock_profile",
    );

    let drifting = report.points.first().expect("drifting clock profile point");

    assert!(!drifting.truth_coverage_ready, "{report:?}");
    assert!(
        drifting
            .truth_coverage_issues
            .iter()
            .any(|issue| issue.code == "missing_clock_profile_reference_observations"),
        "{report:?}"
    );
    assert!(!drifting.ready, "{report:?}");
}
