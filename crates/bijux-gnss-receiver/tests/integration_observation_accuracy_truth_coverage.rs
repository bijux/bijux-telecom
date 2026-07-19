#![allow(missing_docs)]

#[path = "support/observation_truth_table.rs"]
mod observation_truth_table;
mod support;

use bijux_gnss_receiver::api::{
    sim::{
        truth_guided_receiver_accuracy_budgets, validate_observation_accuracy_budget,
        validate_truth_guided_observations,
    },
    ReceiverPipelineConfig,
};

use observation_truth_table::build_observation_truth_fixture;

#[test]
fn observation_accuracy_budget_requires_complete_synthetic_reference() {
    let mut fixture = build_observation_truth_fixture(
        ReceiverPipelineConfig {
            sampling_freq_hz: 1_023_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            channels: 5,
            ..ReceiverPipelineConfig::default()
        },
        3,
    );
    fixture.profile.scenario.ephemerides.clear();

    let validation = validate_truth_guided_observations(
        &fixture.config,
        &fixture.tracks,
        &fixture.profile.scenario,
        &fixture.reference,
        10,
    );
    let budgets = truth_guided_receiver_accuracy_budgets();
    let accuracy = validate_observation_accuracy_budget(&validation, budgets.observation);

    assert!(!accuracy.truth_coverage_ready, "{accuracy:?}");
    assert!(!accuracy.pass, "{accuracy:?}");
    assert!(
        accuracy
            .truth_coverage_issues
            .iter()
            .any(|issue| issue.code == "missing_ephemeris_for_pseudorange_truth"),
        "{accuracy:?}"
    );
    assert!(
        accuracy
            .truth_coverage_issues
            .iter()
            .any(|issue| issue.code == "missing_pseudorange_truth"),
        "{accuracy:?}"
    );

    for satellite in &accuracy.satellites {
        assert_eq!(satellite.max_pseudorange_error_m, None, "{satellite:?}");
        assert!(satellite.max_carrier_phase_error_cycles.is_some(), "{satellite:?}");
        assert!(satellite.max_doppler_error_hz.is_some(), "{satellite:?}");
        assert!(satellite.max_cn0_error_db_hz.is_some(), "{satellite:?}");
        assert!(!satellite.pass, "{satellite:?}");
    }
}
