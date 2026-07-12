#![allow(missing_docs)]

#[path = "support/observation_truth_table.rs"]
mod observation_truth_table;
mod support;

use bijux_gnss_core::api::{SignalBand, SignalCode};
use bijux_gnss_receiver::api::{
    observation_artifacts_from_tracking_results,
    sim::{validate_truth_guided_observation_table, validate_truth_guided_observations},
    ReceiverPipelineConfig,
};

use observation_truth_table::build_l5_observation_truth_fixture;

const OBSERVATION_EPOCH_COUNT: usize = 3;

#[test]
fn observation_truth_table_separates_same_satellite_l1_and_l5_rows() {
    let fixture = build_l5_observation_truth_fixture(
        ReceiverPipelineConfig {
            sampling_freq_hz: 10_230_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            channels: 6,
            ..ReceiverPipelineConfig::default()
        },
        OBSERVATION_EPOCH_COUNT,
    );
    let duplicated_sat = fixture.profile.scenario.satellites[0].sat;
    let truth_table = validate_truth_guided_observation_table(
        &fixture.config,
        &fixture.tracks,
        &fixture.profile.scenario,
        &fixture.reference,
        10,
    );
    let artifacts = observation_artifacts_from_tracking_results(&fixture.config, &fixture.tracks, 10);
    let report = validate_truth_guided_observations(
        &fixture.config,
        &fixture.tracks,
        &fixture.profile.scenario,
        &fixture.reference,
        10,
    );

    let duplicate_truth_rows = truth_table
        .satellites
        .iter()
        .filter(|satellite| satellite.sat == duplicated_sat)
        .collect::<Vec<_>>();
    let duplicate_report_rows = report
        .satellites
        .iter()
        .filter(|satellite| satellite.sat == duplicated_sat)
        .collect::<Vec<_>>();

    assert_eq!(duplicate_truth_rows.len(), 2, "{duplicate_truth_rows:?}");
    assert_eq!(duplicate_report_rows.len(), 2, "{duplicate_report_rows:?}");
    assert!(
        artifacts.output.epochs.iter().all(|epoch| epoch.valid),
        "{:#?}",
        artifacts.output.epochs
    );
    assert!(
        duplicate_truth_rows.iter().all(|satellite| satellite.notes.is_empty()),
        "{duplicate_truth_rows:#?}"
    );
    assert!(
        duplicate_report_rows.iter().all(|satellite| satellite.notes.is_empty()),
        "{duplicate_report_rows:#?}"
    );
    assert!(duplicate_truth_rows.iter().any(|satellite| {
        satellite.signal_id.band == SignalBand::L1 && satellite.signal_id.code == SignalCode::Ca
    }));
    assert!(duplicate_truth_rows.iter().any(|satellite| {
        satellite.signal_id.band == SignalBand::L5
            && satellite.signal_id.code == SignalCode::Unknown
    }));
    assert!(duplicate_truth_rows
        .iter()
        .all(|satellite| satellite.epoch_count == OBSERVATION_EPOCH_COUNT));
    assert!(duplicate_report_rows.iter().any(|satellite| {
        satellite.signal_id.band == SignalBand::L1 && satellite.signal_id.code == SignalCode::Ca
    }));
    assert!(duplicate_report_rows.iter().any(|satellite| {
        satellite.signal_id.band == SignalBand::L5
            && satellite.signal_id.code == SignalCode::Unknown
    }));
}
