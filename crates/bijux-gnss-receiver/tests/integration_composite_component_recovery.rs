#![allow(missing_docs)]

mod support;

use bijux_gnss_core::api::SignalBand;
use bijux_gnss_receiver::api::sim::validate_truth_guided_composite_component_recovery;

use support::composite_component_recovery::{
    build_noisy_capture, load_composite_component_recovery_fixture,
};

const NOISY_POWER_TOLERANCE_DB: f64 = 0.75;
const NOISY_PHASE_TOLERANCE_RAD: f64 = 0.20;
const MAX_RECOVERED_POWER_ERROR_DB: f64 = 0.50;
const MAX_RECOVERED_PHASE_ERROR_RAD: f64 = 0.08;

#[test]
fn composite_component_recovery_matches_multiconstellation_fixture() {
    let fixture =
        load_composite_component_recovery_fixture("composite_component_recovery.toml");
    let (scaled_frame, truth) = build_noisy_capture(&fixture);

    let report = validate_truth_guided_composite_component_recovery(
        &fixture.config,
        &scaled_frame,
        &truth,
        NOISY_POWER_TOLERANCE_DB,
        NOISY_PHASE_TOLERANCE_RAD,
    );

    assert!(report.pass, "{report:?}");
    assert_eq!(report.scenario_id, "composite_component_recovery");
    assert_eq!(report.solver_status, "ok");
    assert_eq!(report.sample_count, scaled_frame.len());
    assert_eq!(report.satellites.len(), 3, "{report:?}");
    assert!(report.residual_rms.is_finite(), "{report:?}");
    assert!(report.residual_rms > 0.0, "{report:?}");
    assert!(report.satellites.iter().all(|row| row.pass), "{report:?}");
    assert!(report.satellites.iter().any(|row| row.signal_band == SignalBand::L1));
    assert!(report.satellites.iter().any(|row| row.signal_band == SignalBand::E1));
    assert!(report.satellites.iter().any(|row| row.signal_band == SignalBand::B2));
}

#[test]
fn composite_component_recovery_keeps_noisy_power_and_phase_errors_small() {
    let fixture =
        load_composite_component_recovery_fixture("composite_component_recovery.toml");
    let (scaled_frame, truth) = build_noisy_capture(&fixture);

    let report = validate_truth_guided_composite_component_recovery(
        &fixture.config,
        &scaled_frame,
        &truth,
        NOISY_POWER_TOLERANCE_DB,
        NOISY_PHASE_TOLERANCE_RAD,
    );

    assert!(report.pass, "{report:?}");
    for row in &report.satellites {
        assert!(row.power_error_db.abs() <= MAX_RECOVERED_POWER_ERROR_DB, "{row:?}");
        assert!(row.phase_error_rad.abs() <= MAX_RECOVERED_PHASE_ERROR_RAD, "{row:?}");
    }
}
