#![allow(missing_docs)]

use bijux_gnss_receiver::api::{
    evaluate_rtk_fixed_baseline_guard, BaselineSolution, RtkFixedBaselineGuardPolicy,
};
use bijux_gnss_testkit::rtk_baseline::{
    clean_gps_l1_short_baseline_case, multipath_gps_l1_short_baseline_case,
    noisy_gps_l1_short_baseline_case,
};

#[test]
fn clean_rtk_fixed_claim_guard_accepts_truth_baseline() {
    let scenario = clean_gps_l1_short_baseline_case();
    let baseline =
        BaselineSolution { enu_m: scenario.truth_enu_m, covariance_m2: None, fixed: true };

    let decision = evaluate_rtk_fixed_baseline_guard(
        &scenario.double_differences,
        scenario.base_ecef_m,
        &baseline,
        &scenario.ephemerides,
        scenario.receive_gps_time.tow_s,
        RtkFixedBaselineGuardPolicy::default(),
    );

    assert!(decision.accepted, "{decision:?}");
    assert!(decision.reasons.is_empty(), "{decision:?}");
}

#[test]
fn noisy_rtk_fixed_claim_guard_rejects_weak_noisy_observations() {
    let scenario = noisy_gps_l1_short_baseline_case();
    let baseline =
        BaselineSolution { enu_m: scenario.truth_enu_m, covariance_m2: None, fixed: true };

    let decision = evaluate_rtk_fixed_baseline_guard(
        &scenario.double_differences,
        scenario.base_ecef_m,
        &baseline,
        &scenario.ephemerides,
        scenario.receive_gps_time.tow_s,
        RtkFixedBaselineGuardPolicy::default(),
    );

    assert!(!decision.accepted, "{decision:?}");
    assert!(decision.reasons.iter().any(|reason| reason == "low_cn0"));
    assert!(decision.reasons.iter().any(|reason| reason == "high_observation_variance"));
    assert!(decision.reasons.iter().any(|reason| reason == "solution_separation"));
    assert!(
        decision.max_solution_separation_m.is_some_and(|separation_m| separation_m > 0.75),
        "{decision:?}"
    );
}

#[test]
fn multipath_rtk_fixed_claim_guard_rejects_suspect_observations() {
    let scenario = multipath_gps_l1_short_baseline_case();
    let baseline =
        BaselineSolution { enu_m: scenario.truth_enu_m, covariance_m2: None, fixed: true };

    let decision = evaluate_rtk_fixed_baseline_guard(
        &scenario.double_differences,
        scenario.base_ecef_m,
        &baseline,
        &scenario.ephemerides,
        scenario.receive_gps_time.tow_s,
        RtkFixedBaselineGuardPolicy::default(),
    );

    assert!(!decision.accepted, "{decision:?}");
    assert!(decision.multipath_suspect, "{decision:?}");
    assert!(decision.reasons.iter().any(|reason| reason == "multipath_suspect"));
}
