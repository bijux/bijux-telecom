#![allow(missing_docs)]

#[path = "support/navigation_clock_profile.rs"]
mod navigation_clock_profile;

use navigation_clock_profile::{
    build_navigation_clock_case, synthetic_navigation_clock_profile, truth_clock_drift_s_per_s,
};

const MAX_CLOCK_DRIFT_ERROR_S_PER_S: f64 = 1.0e-8;
const MAX_FINAL_CLOCK_BIAS_ERROR_S: f64 = 1.0e-9;

#[test]
fn navigation_estimates_injected_receiver_clock_drift() {
    let profile = synthetic_navigation_clock_profile("oscillator_drift_receiver_clock");
    let truth_drift_s_per_s = truth_clock_drift_s_per_s(&profile);
    let truth_final_clock_bias_s = profile
        .truth_epochs
        .last()
        .map(|epoch| epoch.truth_clock_bias_s)
        .expect("clock drift truth epoch");
    let case = build_navigation_clock_case(profile);
    let final_solution = case.solutions.last().expect("navigation solution");

    assert!(
        (final_solution.clock_drift_s_per_s - truth_drift_s_per_s).abs()
            <= MAX_CLOCK_DRIFT_ERROR_S_PER_S,
        "estimated clock drift diverged from injected oscillator drift: solution={final_solution:?} truth_drift_s_per_s={truth_drift_s_per_s}"
    );
    assert!(
        (final_solution.clock_bias_s.0 - truth_final_clock_bias_s).abs()
            <= MAX_FINAL_CLOCK_BIAS_ERROR_S,
        "final clock bias diverged from the injected drift profile: solution={final_solution:?} truth_final_clock_bias_s={truth_final_clock_bias_s}"
    );

    let measured_biases_s =
        case.solutions.iter().map(|solution| solution.clock_bias_s.0).collect::<Vec<_>>();
    for window in measured_biases_s.windows(2) {
        assert!(
            window[1] >= window[0],
            "drifting receiver clock should produce nondecreasing solved clock bias: {:?}",
            measured_biases_s
        );
    }
}
