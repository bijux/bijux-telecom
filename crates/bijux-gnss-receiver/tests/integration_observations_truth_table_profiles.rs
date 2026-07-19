#![allow(missing_docs)]

#[path = "support/observation_truth_table.rs"]
mod observation_truth_table;
mod support;

use bijux_gnss_receiver::api::{
    sim::validate_truth_guided_observation_table, ReceiverPipelineConfig,
};

use observation_truth_table::{
    build_observation_truth_fixture, build_observation_truth_fixture_with_cycle_slips,
};

const HATCH_WINDOW: u32 = 10;

#[test]
fn observation_truth_table_covers_low_and_high_rate_profiles() {
    let low_rate = build_observation_truth_fixture(
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
    let high_rate = build_observation_truth_fixture(
        ReceiverPipelineConfig {
            sampling_freq_hz: 2_046_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            channels: 5,
            ..ReceiverPipelineConfig::default()
        },
        3,
    );

    let low_rate_report = validate_truth_guided_observation_table(
        &low_rate.config,
        &low_rate.tracks,
        &low_rate.profile.scenario,
        &low_rate.reference,
        HATCH_WINDOW,
    );
    let high_rate_report = validate_truth_guided_observation_table(
        &high_rate.config,
        &high_rate.tracks,
        &high_rate.profile.scenario,
        &high_rate.reference,
        HATCH_WINDOW,
    );

    assert_eq!(low_rate_report.sample_rate_hz, 1_023_000.0);
    assert_eq!(high_rate_report.sample_rate_hz, 2_046_000.0);
    assert_eq!(low_rate_report.satellites.len(), 5);
    assert_eq!(high_rate_report.satellites.len(), 5);

    let low_rate_prn3 = low_rate_report
        .satellites
        .iter()
        .find(|satellite| satellite.sat.prn == 3)
        .expect("low-rate PRN 3 row");
    let high_rate_prn3 = high_rate_report
        .satellites
        .iter()
        .find(|satellite| satellite.sat.prn == 3)
        .expect("high-rate PRN 3 row");

    assert_eq!(low_rate_prn3.epochs[1].sample_index, 1_023);
    assert_eq!(high_rate_prn3.epochs[1].sample_index, 2_046);
    assert_ne!(low_rate_prn3.epochs[1].sample_index, high_rate_prn3.epochs[1].sample_index);

    for report in [&low_rate_report, &high_rate_report] {
        for satellite in &report.satellites {
            assert!(satellite.notes.is_empty(), "{satellite:?}");
            assert_eq!(satellite.epoch_count, 3, "{satellite:?}");
            for epoch in &satellite.epochs {
                assert!(epoch.pseudorange_m.truth.is_some(), "{epoch:?}");
                assert!(epoch.pseudorange_m.sigma.expect("pseudorange sigma") > 0.0, "{epoch:?}");
                assert!(
                    epoch.pseudorange_m.residual.expect("pseudorange residual").abs() <= 0.1,
                    "{epoch:?}"
                );
            }
        }
    }
}

#[test]
fn observation_truth_table_starts_new_carrier_phase_arc_after_cycle_slip() {
    let fixture = build_observation_truth_fixture_with_cycle_slips(
        ReceiverPipelineConfig {
            sampling_freq_hz: 1_023_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            channels: 5,
            ..ReceiverPipelineConfig::default()
        },
        4,
        &[2],
    );
    let report = validate_truth_guided_observation_table(
        &fixture.config,
        &fixture.tracks,
        &fixture.profile.scenario,
        &fixture.reference,
        HATCH_WINDOW,
    );

    for satellite in &report.satellites {
        assert_eq!(satellite.carrier_phase_arcs_evaluated, 2, "{satellite:?}");
        assert!(
            satellite.epochs.iter().any(|epoch| {
                epoch.observation_reject_reasons.iter().any(|reason| reason == "cycle_slip")
            }),
            "{satellite:?}"
        );

        let arc_starts = satellite
            .epochs
            .iter()
            .map(|epoch| epoch.carrier_phase_arc_start_sample_index.expect("carrier-phase arc"))
            .collect::<Vec<_>>();
        assert_eq!(arc_starts[0], 0, "{satellite:?}");
        assert_eq!(arc_starts[1], 0, "{satellite:?}");
        assert_eq!(arc_starts[2], 2_046, "{satellite:?}");
        assert_eq!(arc_starts[3], 2_046, "{satellite:?}");

        for epoch in &satellite.epochs {
            assert!(epoch.carrier_phase_arc_bias_cycles.is_some(), "{epoch:?}");
            assert!(
                epoch.carrier_phase_cycles.residual.expect("carrier-phase residual").abs()
                    <= 1.0e-6,
                "{epoch:?}"
            );
        }
    }
}
