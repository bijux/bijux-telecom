#![allow(missing_docs)]

#[path = "support/observation_truth_table.rs"]
mod observation_truth_table;
mod support;

use bijux_gnss_receiver::api::{
    sim::{validate_truth_guided_observation_table, validate_truth_guided_observations},
    ReceiverPipelineConfig,
};

use observation_truth_table::build_observation_truth_fixture;

const OBSERVATION_EPOCH_COUNT: usize = 3;
const MAX_ABS_PSEUDORANGE_ERROR_M: f64 = 5.0e-2;
const MAX_ABS_PHASE_LIKE_ERROR: f64 = 1.0e-6;

#[test]
fn observation_truth_validation_matches_truth_table_residuals_per_satellite() {
    let fixture = build_observation_truth_fixture(
        ReceiverPipelineConfig {
            sampling_freq_hz: 1_023_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            channels: 5,
            ..ReceiverPipelineConfig::default()
        },
        OBSERVATION_EPOCH_COUNT,
    );
    let truth_table = validate_truth_guided_observation_table(
        &fixture.config,
        &fixture.tracks,
        &fixture.profile.scenario,
        &fixture.reference,
        10,
    );
    let report = validate_truth_guided_observations(
        &fixture.config,
        &fixture.tracks,
        &fixture.profile.scenario,
        &fixture.reference,
        10,
    );

    assert_eq!(report.scenario_id, fixture.profile.scenario.id);
    assert_eq!(report.satellites.len(), fixture.profile.scenario.satellites.len());
    assert_eq!(truth_table.satellites.len(), report.satellites.len());

    for (satellite, truth_rows) in report.satellites.iter().zip(truth_table.satellites.iter()) {
        assert!(satellite.notes.is_empty(), "{satellite:?}");
        assert_eq!(satellite.sat, truth_rows.sat);
        assert_eq!(truth_rows.epoch_count, OBSERVATION_EPOCH_COUNT, "{truth_rows:?}");

        let pseudorange_errors = truth_rows
            .epochs
            .iter()
            .map(|epoch| epoch.pseudorange_m.residual.expect("pseudorange residual"))
            .collect::<Vec<_>>();
        let pseudorange = satellite
            .pseudorange_error_m
            .as_ref()
            .unwrap_or_else(|| panic!("missing pseudorange stats for {:?}", satellite.sat));
        assert_eq!(pseudorange.count, OBSERVATION_EPOCH_COUNT, "{satellite:?}");
        assert!(
            (pseudorange.max_abs_error
                - pseudorange_errors.iter().map(|error| error.abs()).fold(0.0_f64, f64::max))
            .abs()
                <= 1.0e-12
        );
        assert!(pseudorange.max_abs_error <= MAX_ABS_PSEUDORANGE_ERROR_M, "{satellite:?}");

        let carrier_phase_errors = truth_rows
            .epochs
            .iter()
            .map(|epoch| epoch.carrier_phase_cycles.residual.expect("carrier-phase residual"))
            .collect::<Vec<_>>();
        let carrier_phase = satellite
            .carrier_phase_error_cycles
            .as_ref()
            .unwrap_or_else(|| panic!("missing carrier-phase stats for {:?}", satellite.sat));
        assert_eq!(carrier_phase.count, OBSERVATION_EPOCH_COUNT, "{satellite:?}");
        assert_eq!(satellite.carrier_phase_arcs_evaluated, truth_rows.carrier_phase_arcs_evaluated);
        assert!(
            (carrier_phase.max_abs_error
                - carrier_phase_errors.iter().map(|error| error.abs()).fold(0.0_f64, f64::max))
            .abs()
                <= 1.0e-12
        );
        assert!(carrier_phase.max_abs_error <= MAX_ABS_PHASE_LIKE_ERROR, "{satellite:?}");

        let doppler_errors = truth_rows
            .epochs
            .iter()
            .map(|epoch| epoch.doppler_hz.residual.expect("doppler residual"))
            .collect::<Vec<_>>();
        let doppler = satellite
            .doppler_error_hz
            .as_ref()
            .unwrap_or_else(|| panic!("missing Doppler stats for {:?}", satellite.sat));
        assert_eq!(doppler.count, OBSERVATION_EPOCH_COUNT, "{satellite:?}");
        assert!(
            (doppler.max_abs_error
                - doppler_errors.iter().map(|error| error.abs()).fold(0.0_f64, f64::max))
            .abs()
                <= 1.0e-12
        );
        assert!(doppler.max_abs_error <= MAX_ABS_PHASE_LIKE_ERROR, "{satellite:?}");

        let cn0_errors = truth_rows
            .epochs
            .iter()
            .map(|epoch| epoch.cn0_db_hz.residual.expect("cn0 residual"))
            .collect::<Vec<_>>();
        let cn0 = satellite
            .cn0_error_db_hz
            .as_ref()
            .unwrap_or_else(|| panic!("missing C/N0 stats for {:?}", satellite.sat));
        assert_eq!(cn0.count, OBSERVATION_EPOCH_COUNT, "{satellite:?}");
        assert!(
            (cn0.max_abs_error
                - cn0_errors.iter().map(|error| error.abs()).fold(0.0_f64, f64::max))
            .abs()
                <= 1.0e-12
        );
        assert!(cn0.max_abs_error <= MAX_ABS_PHASE_LIKE_ERROR, "{satellite:?}");
    }
}
