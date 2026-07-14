#![allow(missing_docs)]

#[path = "support/navigation_accuracy_artifact.rs"]
mod navigation_accuracy_artifact;

use bijux_gnss_core::api::{Constellation, SatId, SignalBand, SignalCode};
use bijux_gnss_receiver::api::sim::{
    build_truth_guided_gnss_accuracy_artifact, write_truth_guided_gnss_accuracy_artifact,
    SyntheticAcquisitionAccuracyReport, SyntheticAcquisitionAccuracySatellite,
    SyntheticGnssAccuracyArtifact, SyntheticGnssAccuracyArtifactCase,
};

use navigation_accuracy_artifact::{
    build_navigation_accuracy_artifact, build_navigation_accuracy_artifact_fixture,
};

#[test]
fn navigation_accuracy_artifact_summarizes_acquisition_and_tracking_stages() {
    let fixture = build_navigation_accuracy_artifact_fixture();
    let artifact = build_truth_guided_gnss_accuracy_artifact(SyntheticGnssAccuracyArtifactCase {
        scenario_id: &fixture.profile.scenario.id,
        data_source: fixture.data_source.clone(),
        reference_truth: fixture.reference_truth.clone(),
        acquisition: &fixture.acquisition_accuracy,
        tracking: &fixture.tracking_accuracy,
        observation: &fixture.observation_accuracy,
        pvt: &fixture.pvt_accuracy,
    });

    assert_eq!(artifact.scenario_id, fixture.profile.scenario.id);
    assert_eq!(artifact.data_source, fixture.data_source);
    assert_eq!(artifact.reference_truth, fixture.reference_truth);

    assert_eq!(artifact.acquisition.summary.pass, fixture.acquisition_accuracy.pass);
    assert_eq!(
        artifact.acquisition.summary.truth_coverage_ready,
        fixture.acquisition_accuracy.truth_coverage_ready
    );
    assert_eq!(
        artifact.acquisition.summary.observed_max_doppler_error_hz,
        fixture
            .acquisition_accuracy
            .satellites
            .iter()
            .map(|satellite| satellite.doppler_error_hz)
            .max_by(f64::total_cmp)
    );
    assert_eq!(
        artifact.acquisition.summary.observed_max_code_phase_error_samples,
        fixture
            .acquisition_accuracy
            .satellites
            .iter()
            .map(|satellite| satellite.code_phase_error_samples)
            .max()
    );
    assert_eq!(
        artifact.acquisition.summary.threshold_max_doppler_error_hz,
        fixture.acquisition_accuracy.max_doppler_error_hz
    );
    assert_eq!(
        artifact.acquisition.summary.threshold_max_code_phase_error_samples,
        fixture.acquisition_accuracy.max_code_phase_error_samples
    );

    assert_eq!(artifact.tracking.summary.pass, fixture.tracking_accuracy.pass);
    assert_eq!(
        artifact.tracking.summary.truth_coverage_ready,
        fixture.tracking_accuracy.truth_coverage_ready
    );
    assert_eq!(
        artifact.tracking.summary.observed_max_carrier_error_hz,
        fixture
            .tracking_accuracy
            .satellites
            .iter()
            .map(|satellite| satellite.max_carrier_error_hz)
            .max_by(f64::total_cmp)
    );
    assert_eq!(
        artifact.tracking.summary.observed_max_doppler_error_hz,
        fixture
            .tracking_accuracy
            .satellites
            .iter()
            .map(|satellite| satellite.max_doppler_error_hz)
            .max_by(f64::total_cmp)
    );
    assert_eq!(
        artifact.tracking.summary.observed_max_code_phase_error_samples,
        fixture
            .tracking_accuracy
            .satellites
            .iter()
            .map(|satellite| satellite.max_code_phase_error_samples)
            .max_by(f64::total_cmp)
    );
    assert_eq!(
        artifact.tracking.summary.observed_max_cn0_error_db_hz,
        fixture
            .tracking_accuracy
            .satellites
            .iter()
            .map(|satellite| satellite.max_cn0_error_db_hz)
            .max_by(f64::total_cmp)
    );
    assert_eq!(
        artifact.tracking.summary.threshold_max_carrier_error_hz,
        fixture.tracking_accuracy.max_carrier_error_hz
    );
    assert_eq!(
        artifact.tracking.summary.threshold_max_doppler_error_hz,
        fixture.tracking_accuracy.max_doppler_error_hz
    );
    assert_eq!(
        artifact.tracking.summary.threshold_max_code_phase_error_samples,
        fixture.tracking_accuracy.max_code_phase_error_samples
    );
    assert_eq!(
        artifact.tracking.summary.threshold_max_cn0_error_db_hz,
        fixture.tracking_accuracy.max_cn0_error_db_hz
    );
}

#[test]
fn navigation_accuracy_artifact_preserves_signal_specific_acquisition_thresholds() {
    let fixture = build_navigation_accuracy_artifact_fixture();
    let acquisition = SyntheticAcquisitionAccuracyReport {
        scenario_id: fixture.acquisition_accuracy.scenario_id.clone(),
        max_doppler_error_hz: 500.0,
        max_code_phase_error_samples: 2,
        satellite_count: 2,
        passing_satellite_count: 2,
        truth_coverage_ready: true,
        truth_coverage_issues: Vec::new(),
        pass: true,
        satellites: vec![
            SyntheticAcquisitionAccuracySatellite {
                sat: SatId { constellation: Constellation::Gps, prn: 3 },
                glonass_frequency_channel: None,
                signal_band: SignalBand::L1,
                signal_code: SignalCode::Ca,
                doppler_error_hz: 0.25,
                code_phase_error_samples: 1,
                max_doppler_error_hz: 500.0,
                max_doppler_error_bins: 1.0,
                max_code_phase_error_samples: 2,
                max_code_phase_error_chips: 1.0,
                pass: true,
            },
            SyntheticAcquisitionAccuracySatellite {
                sat: SatId { constellation: Constellation::Galileo, prn: 11 },
                glonass_frequency_channel: None,
                signal_band: SignalBand::E1,
                signal_code: SignalCode::E1B,
                doppler_error_hz: 0.5,
                code_phase_error_samples: 1,
                max_doppler_error_hz: 500.0,
                max_doppler_error_bins: 1.0,
                max_code_phase_error_samples: 1,
                max_code_phase_error_chips: 0.25,
                pass: true,
            },
        ],
    };
    let artifact = build_truth_guided_gnss_accuracy_artifact(SyntheticGnssAccuracyArtifactCase {
        scenario_id: &fixture.profile.scenario.id,
        data_source: fixture.data_source.clone(),
        reference_truth: fixture.reference_truth.clone(),
        acquisition: &acquisition,
        tracking: &fixture.tracking_accuracy,
        observation: &fixture.observation_accuracy,
        pvt: &fixture.pvt_accuracy,
    });

    assert_eq!(artifact.acquisition.summary.threshold_max_doppler_error_hz, 500.0);
    assert_eq!(artifact.acquisition.summary.threshold_max_code_phase_error_samples, 2);
    assert_eq!(artifact.acquisition.report.satellites.len(), 2);
    assert_eq!(artifact.acquisition.report.satellites[0].max_code_phase_error_samples, 2);
    assert_eq!(artifact.acquisition.report.satellites[1].max_code_phase_error_samples, 1);
    assert!(
        (artifact.acquisition.report.satellites[1].max_code_phase_error_chips - 0.25).abs()
            <= f64::EPSILON
    );
}

#[test]
fn navigation_accuracy_artifact_summarizes_observation_pvt_and_overall_status() {
    let fixture = build_navigation_accuracy_artifact_fixture();
    let artifact = build_truth_guided_gnss_accuracy_artifact(SyntheticGnssAccuracyArtifactCase {
        scenario_id: &fixture.profile.scenario.id,
        data_source: fixture.data_source.clone(),
        reference_truth: fixture.reference_truth.clone(),
        acquisition: &fixture.acquisition_accuracy,
        tracking: &fixture.tracking_accuracy,
        observation: &fixture.observation_accuracy,
        pvt: &fixture.pvt_accuracy,
    });

    assert_eq!(artifact.observation.summary.pass, fixture.observation_accuracy.pass);
    assert_eq!(
        artifact.observation.summary.truth_coverage_ready,
        fixture.observation_accuracy.truth_coverage_ready
    );
    assert_eq!(
        artifact.observation.summary.observed_max_pseudorange_error_m,
        fixture
            .observation_accuracy
            .satellites
            .iter()
            .filter_map(|satellite| satellite.max_pseudorange_error_m)
            .max_by(f64::total_cmp)
    );
    assert_eq!(
        artifact.observation.summary.observed_max_carrier_phase_error_cycles,
        fixture
            .observation_accuracy
            .satellites
            .iter()
            .filter_map(|satellite| satellite.max_carrier_phase_error_cycles)
            .max_by(f64::total_cmp)
    );
    assert_eq!(
        artifact.observation.summary.observed_max_doppler_error_hz,
        fixture
            .observation_accuracy
            .satellites
            .iter()
            .filter_map(|satellite| satellite.max_doppler_error_hz)
            .max_by(f64::total_cmp)
    );
    assert_eq!(
        artifact.observation.summary.observed_max_cn0_error_db_hz,
        fixture
            .observation_accuracy
            .satellites
            .iter()
            .filter_map(|satellite| satellite.max_cn0_error_db_hz)
            .max_by(f64::total_cmp)
    );
    assert_eq!(
        artifact.observation.summary.threshold_max_pseudorange_error_m,
        fixture.observation_accuracy.max_pseudorange_error_m
    );
    assert_eq!(
        artifact.observation.summary.threshold_max_carrier_phase_error_cycles,
        fixture.observation_accuracy.max_carrier_phase_error_cycles
    );
    assert_eq!(
        artifact.observation.summary.threshold_max_doppler_error_hz,
        fixture.observation_accuracy.max_doppler_error_hz
    );
    assert_eq!(
        artifact.observation.summary.threshold_max_cn0_error_db_hz,
        fixture.observation_accuracy.max_cn0_error_db_hz
    );

    assert_eq!(artifact.pvt.summary.pass, fixture.pvt_accuracy.pass);
    assert_eq!(
        artifact.pvt.summary.truth_coverage_ready,
        fixture.pvt_accuracy.truth_coverage_ready
    );
    assert_eq!(
        artifact.pvt.summary.observed_max_position_error_3d_m,
        fixture
            .pvt_accuracy
            .epochs
            .iter()
            .map(|epoch| epoch.position_error_3d_m)
            .max_by(f64::total_cmp)
    );
    assert_eq!(
        artifact.pvt.summary.observed_max_clock_bias_error_m,
        fixture
            .pvt_accuracy
            .epochs
            .iter()
            .map(|epoch| epoch.clock_bias_error_m)
            .max_by(f64::total_cmp)
    );
    assert_eq!(
        artifact.pvt.summary.observed_max_residual_rms_m,
        fixture.pvt_accuracy.epochs.iter().map(|epoch| epoch.residual_rms_m).max_by(f64::total_cmp)
    );
    assert_eq!(
        artifact.pvt.summary.observed_max_pdop,
        fixture.pvt_accuracy.epochs.iter().map(|epoch| epoch.pdop).max_by(f64::total_cmp)
    );
    assert_eq!(
        artifact.pvt.summary.threshold_max_position_error_3d_m,
        fixture.pvt_accuracy.max_position_error_3d_m
    );
    assert_eq!(
        artifact.pvt.summary.threshold_max_clock_bias_error_m,
        fixture.pvt_accuracy.max_clock_bias_error_m
    );
    assert_eq!(
        artifact.pvt.summary.threshold_max_residual_rms_m,
        fixture.pvt_accuracy.max_residual_rms_m
    );
    assert_eq!(artifact.pvt.summary.threshold_max_pdop, fixture.pvt_accuracy.max_pdop);

    assert_eq!(
        artifact.pass,
        fixture.acquisition_accuracy.pass
            && fixture.tracking_accuracy.pass
            && fixture.observation_accuracy.pass
            && fixture.pvt_accuracy.pass
    );
    assert_eq!(
        artifact.truth_coverage_ready,
        fixture.acquisition_accuracy.truth_coverage_ready
            && fixture.tracking_accuracy.truth_coverage_ready
            && fixture.observation_accuracy.truth_coverage_ready
            && fixture.pvt_accuracy.truth_coverage_ready
    );
}

#[test]
fn navigation_accuracy_artifact_writes_single_json_file() {
    let artifact = build_navigation_accuracy_artifact();
    let unique_suffix = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .expect("system time after unix epoch")
        .as_nanos();
    let path = std::env::temp_dir().join(format!(
        "bijux-navigation-accuracy-artifact-{}-{unique_suffix}.json",
        std::process::id()
    ));

    write_truth_guided_gnss_accuracy_artifact(&path, &artifact).expect("write accuracy artifact");

    let bytes = std::fs::read(&path).expect("read emitted accuracy artifact");
    let parsed: SyntheticGnssAccuracyArtifact =
        serde_json::from_slice(&bytes).expect("parse emitted accuracy artifact");
    let value: serde_json::Value =
        serde_json::from_slice(&bytes).expect("inspect emitted accuracy artifact");

    assert_eq!(parsed.schema_version, artifact.schema_version);
    assert_eq!(parsed.scenario_id, artifact.scenario_id);
    assert_eq!(parsed.pass, artifact.pass);
    assert_eq!(parsed.truth_coverage_ready, artifact.truth_coverage_ready);
    assert_eq!(parsed.data_source, artifact.data_source);
    assert_eq!(parsed.reference_truth, artifact.reference_truth);
    assert_eq!(parsed.acquisition.summary, artifact.acquisition.summary);
    assert_eq!(parsed.observation.summary, artifact.observation.summary);
    assert_eq!(parsed.pvt.summary.pass, artifact.pvt.summary.pass);
    assert_eq!(parsed.pvt.summary.truth_coverage_ready, artifact.pvt.summary.truth_coverage_ready);
    assert_eq!(parsed.pvt.summary.epoch_count, artifact.pvt.summary.epoch_count);
    assert_eq!(parsed.pvt.summary.passing_epoch_count, artifact.pvt.summary.passing_epoch_count);
    assert_optional_close(
        parsed.pvt.summary.observed_max_position_error_3d_m,
        artifact.pvt.summary.observed_max_position_error_3d_m,
        1.0e-12,
    );
    assert_optional_close(
        parsed.pvt.summary.observed_max_clock_bias_error_m,
        artifact.pvt.summary.observed_max_clock_bias_error_m,
        1.0e-12,
    );
    assert_optional_close(
        parsed.pvt.summary.observed_max_residual_rms_m,
        artifact.pvt.summary.observed_max_residual_rms_m,
        1.0e-12,
    );
    assert_optional_close(
        parsed.pvt.summary.observed_max_pdop,
        artifact.pvt.summary.observed_max_pdop,
        1.0e-12,
    );
    assert_eq!(
        parsed.pvt.summary.threshold_max_position_error_3d_m,
        artifact.pvt.summary.threshold_max_position_error_3d_m
    );
    assert_eq!(
        parsed.pvt.summary.threshold_max_clock_bias_error_m,
        artifact.pvt.summary.threshold_max_clock_bias_error_m
    );
    assert_eq!(
        parsed.pvt.summary.threshold_max_residual_rms_m,
        artifact.pvt.summary.threshold_max_residual_rms_m
    );
    assert_eq!(parsed.pvt.summary.threshold_max_pdop, artifact.pvt.summary.threshold_max_pdop);
    assert_eq!(parsed.tracking.summary.pass, artifact.tracking.summary.pass);
    assert_eq!(
        parsed.tracking.summary.passing_satellite_count,
        artifact.tracking.summary.passing_satellite_count
    );
    assert_eq!(
        parsed.tracking.summary.threshold_max_carrier_error_hz,
        artifact.tracking.summary.threshold_max_carrier_error_hz
    );
    assert_eq!(value["scenario_id"], artifact.scenario_id);
    assert_eq!(value["data_source"]["source_kind"], artifact.data_source.source_kind);
    assert_eq!(value["reference_truth"]["truth_kind"], artifact.reference_truth.truth_kind);
    assert!(value["acquisition"]["summary"]["threshold_max_doppler_error_hz"].is_number());
    assert!(value["tracking"]["summary"]["observed_max_carrier_error_hz"].is_number());
    assert!(value["observation"]["summary"]["threshold_max_pseudorange_error_m"].is_number());
    assert!(value["pvt"]["summary"]["threshold_max_position_error_3d_m"].is_number());

    std::fs::remove_file(path).expect("remove emitted accuracy artifact");
}

fn assert_optional_close(actual: Option<f64>, expected: Option<f64>, tolerance: f64) {
    match (actual, expected) {
        (Some(actual), Some(expected)) => {
            let error = (actual - expected).abs();
            assert!(
                error <= tolerance,
                "actual={actual:.15} expected={expected:.15} error={error:.15} tolerance={tolerance:.15}"
            );
        }
        (None, None) => {}
        _ => panic!("optional float mismatch: actual={actual:?} expected={expected:?}"),
    }
}
