#![allow(missing_docs)]

mod support;

use bijux_gnss_core::api::SignalBand;
use bijux_gnss_receiver::api::sim::validate_truth_guided_composite_component_recovery;

use support::composite_component_recovery::{
    build_noisy_capture, load_composite_component_recovery_fixture,
};

#[test]
fn composite_component_recovery_matches_multiconstellation_fixture() {
    let fixture =
        load_composite_component_recovery_fixture("composite_component_recovery.toml");
    let (scaled_frame, truth) = build_noisy_capture(&fixture);

    let report = validate_truth_guided_composite_component_recovery(
        &fixture.config,
        &scaled_frame,
        &truth,
        0.75,
        0.20,
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
