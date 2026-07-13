#![allow(missing_docs)]

#[path = "support/observation_truth_table.rs"]
mod observation_truth_table;
mod support;

use bijux_gnss_core::api::{SignalBand, SignalCode};
use bijux_gnss_nav::api::iono_free_code_from_obs_epochs;
use bijux_gnss_receiver::api::{
    observation_artifacts_from_tracking_results,
    sim::{validate_truth_guided_observation_table, validate_truth_guided_observations},
    ReceiverPipelineConfig,
};
use bijux_gnss_signal::api::{signal_spec_gps_l1_ca, signal_spec_gps_l5};

use observation_truth_table::build_l5_observation_truth_fixture;
use support::navigation_truth::synthetic_pseudorange_m;

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
    let artifacts =
        observation_artifacts_from_tracking_results(&fixture.config, &fixture.tracks, 10);
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

#[test]
fn l5_observation_truth_applies_dispersive_ionosphere_delay() {
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
    let artifacts =
        observation_artifacts_from_tracking_results(&fixture.config, &fixture.tracks, 10);

    let ionosphere_delay_model =
        fixture.reference.ionosphere_delay_model.expect("L5 ionosphere model");
    let duplicated_ephemeris = fixture
        .profile
        .ephemerides
        .iter()
        .find(|candidate| candidate.sat == duplicated_sat)
        .expect("duplicated L5 ephemeris");
    let geometric_pseudorange_m = synthetic_pseudorange_m(
        duplicated_ephemeris,
        fixture.reference.receive_time_s,
        fixture.profile.truth_ecef_m,
    );
    let expected_l1_pseudorange_m = ionosphere_delay_model
        .pseudorange_m(geometric_pseudorange_m, signal_spec_gps_l1_ca())
        .expect("L1 pseudorange");
    let expected_l5_pseudorange_m = ionosphere_delay_model
        .pseudorange_m(geometric_pseudorange_m, signal_spec_gps_l5())
        .expect("L5 pseudorange");

    let l1_row = truth_table
        .satellites
        .iter()
        .find(|satellite| {
            satellite.sat == duplicated_sat
                && satellite.signal_id.band == SignalBand::L1
                && satellite.signal_id.code == SignalCode::Ca
        })
        .expect("L1 L5 truth row");
    let l5_row = truth_table
        .satellites
        .iter()
        .find(|satellite| {
            satellite.sat == duplicated_sat
                && satellite.signal_id.band == SignalBand::L5
                && satellite.signal_id.code == SignalCode::Unknown
        })
        .expect("L5 truth row");

    assert!(
        (l1_row.epochs[0].pseudorange_m.truth.expect("L1 truth") - expected_l1_pseudorange_m).abs()
            < 1.0e-6
    );
    assert!(
        (l5_row.epochs[0].pseudorange_m.truth.expect("L5 truth") - expected_l5_pseudorange_m).abs()
            < 1.0e-6
    );
    assert!((expected_l5_pseudorange_m - expected_l1_pseudorange_m).abs() > 1.0e-3);

    let iono_free =
        iono_free_code_from_obs_epochs(&artifacts.output.epochs, SignalBand::L1, SignalBand::L5);
    let duplicated_observation = iono_free
        .into_iter()
        .find(|observation| observation.sat == duplicated_sat)
        .expect("L1 L5 iono-free observation");

    assert_eq!(duplicated_observation.status, "ok");
    assert!(
        (duplicated_observation.code_m.expect("iono-free code") - geometric_pseudorange_m).abs()
            < 1.0e-6
    );
}
