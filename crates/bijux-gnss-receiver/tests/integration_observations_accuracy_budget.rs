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
fn observation_accuracy_budget_enforces_hard_truth_thresholds() {
    let fixture = build_observation_truth_fixture(
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
    let budgets = truth_guided_receiver_accuracy_budgets();
    let validation = validate_truth_guided_observations(
        &fixture.config,
        &fixture.tracks,
        &fixture.profile.scenario,
        &fixture.reference,
        10,
    );
    let report = validate_observation_accuracy_budget(&validation, budgets.observation);

    assert!(report.pass, "{report:?}");
    assert_eq!(report.scenario_id, validation.scenario_id);
    assert_eq!(report.max_pseudorange_error_m, budgets.observation.max_pseudorange_error_m);
    assert_eq!(
        report.max_carrier_phase_error_cycles,
        budgets.observation.max_carrier_phase_error_cycles
    );
    assert_eq!(report.max_doppler_error_hz, budgets.observation.max_doppler_error_hz);
    assert_eq!(report.max_cn0_error_db_hz, budgets.observation.max_cn0_error_db_hz);
    assert_eq!(report.satellite_count, validation.satellites.len());
    assert_eq!(report.passing_satellite_count, report.satellite_count);

    for satellite in &report.satellites {
        assert!(satellite.pass, "{satellite:?}");
        assert!(
            satellite.max_pseudorange_error_m.expect("pseudorange error")
                <= report.max_pseudorange_error_m + f64::EPSILON,
            "{satellite:?}"
        );
        assert!(
            satellite.max_carrier_phase_error_cycles.expect("carrier-phase error")
                <= report.max_carrier_phase_error_cycles + f64::EPSILON,
            "{satellite:?}"
        );
        assert!(
            satellite.max_doppler_error_hz.expect("doppler error")
                <= report.max_doppler_error_hz + f64::EPSILON,
            "{satellite:?}"
        );
        assert!(
            satellite.max_cn0_error_db_hz.expect("cn0 error")
                <= report.max_cn0_error_db_hz + f64::EPSILON,
            "{satellite:?}"
        );
    }
}
