#![allow(missing_docs)]

#[path = "support/observation_truth_table.rs"]
mod observation_truth_table;
mod support;

use bijux_gnss_core::api::ObservationStatus;
use bijux_gnss_receiver::api::{
    sim::validate_truth_guided_observation_table, ReceiverPipelineConfig,
};

use observation_truth_table::{
    build_observation_truth_fixture, TRACKING_CARRIER_PHASE_SIGMA_CYCLES, TRACKING_CN0_SIGMA_DBHZ,
    TRACKING_DOPPLER_SIGMA_HZ,
};

const OBSERVATION_EPOCH_COUNT: usize = 3;
const MAX_ABS_PSEUDORANGE_RESIDUAL_M: f64 = 5.0e-2;
const MAX_ABS_PHASE_LIKE_RESIDUAL: f64 = 1.0e-6;

#[test]
fn observation_truth_table_records_observable_values_sigma_and_residuals_per_epoch() {
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
    let report = validate_truth_guided_observation_table(
        &fixture.config,
        &fixture.tracks,
        &fixture.profile.scenario,
        &fixture.reference,
        10,
    );

    assert_eq!(report.scenario_id, fixture.profile.scenario.id);
    assert_eq!(report.sample_rate_hz, fixture.config.sampling_freq_hz);
    assert_eq!(report.reference_receive_time_s, fixture.reference.receive_time_s);
    assert_eq!(report.satellites.len(), fixture.profile.scenario.satellites.len());

    for satellite in &report.satellites {
        assert_eq!(satellite.epoch_count, OBSERVATION_EPOCH_COUNT, "{satellite:?}");
        assert_eq!(satellite.epochs.len(), OBSERVATION_EPOCH_COUNT, "{satellite:?}");
        assert_eq!(satellite.carrier_phase_arcs_evaluated, 1, "{satellite:?}");
        assert!(satellite.notes.is_empty(), "{satellite:?}");

        for epoch in &satellite.epochs {
            assert!(!epoch.artifact_id.is_empty(), "{epoch:?}");
            assert!(!epoch.epoch_id.is_empty(), "{epoch:?}");
            assert_eq!(epoch.observation_status, ObservationStatus::Accepted, "{epoch:?}");
            assert!(epoch.observation_reject_reasons.is_empty(), "{epoch:?}");

            assert!(epoch.pseudorange_m.truth.is_some(), "{epoch:?}");
            assert!(epoch.pseudorange_m.sigma.expect("pseudorange sigma") > 0.0, "{epoch:?}");
            assert!(
                epoch.pseudorange_m.residual.expect("pseudorange residual").abs()
                    <= MAX_ABS_PSEUDORANGE_RESIDUAL_M,
                "{epoch:?}"
            );

            assert!(epoch.carrier_phase_cycles.truth.is_some(), "{epoch:?}");
            assert_eq!(
                epoch.carrier_phase_cycles.sigma,
                Some(TRACKING_CARRIER_PHASE_SIGMA_CYCLES),
                "{epoch:?}"
            );
            assert_eq!(epoch.carrier_phase_arc_start_sample_index, Some(0), "{epoch:?}");
            assert!(epoch.carrier_phase_arc_bias_cycles.is_some(), "{epoch:?}");
            assert!(
                epoch.carrier_phase_cycles.residual.expect("carrier-phase residual").abs()
                    <= MAX_ABS_PHASE_LIKE_RESIDUAL,
                "{epoch:?}"
            );

            assert_eq!(epoch.doppler_hz.sigma, Some(TRACKING_DOPPLER_SIGMA_HZ), "{epoch:?}");
            assert!(
                epoch.doppler_hz.residual.expect("doppler residual").abs()
                    <= MAX_ABS_PHASE_LIKE_RESIDUAL,
                "{epoch:?}"
            );

            assert_eq!(epoch.cn0_db_hz.sigma, Some(TRACKING_CN0_SIGMA_DBHZ), "{epoch:?}");
            assert!(
                epoch.cn0_db_hz.residual.expect("cn0 residual").abs()
                    <= MAX_ABS_PHASE_LIKE_RESIDUAL,
                "{epoch:?}"
            );
        }
    }
}
