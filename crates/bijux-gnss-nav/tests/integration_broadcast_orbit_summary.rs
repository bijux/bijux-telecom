#![allow(missing_docs)]

mod support;

use support::orbit_reference::{
    broadcast_reference_fixtures, direct_broadcast_orbit_errors, summarize_orbit_errors,
};

const GPS_BROADCAST_POSITION_TOLERANCE_M: f64 = 5.0;
const GPS_BROADCAST_POSITION_RMS_TOLERANCE_M: f64 = 3.0;

#[test]
fn broadcast_orbit_reference_error_budget_stays_stable() {
    let mut errors = Vec::new();
    for fixture in broadcast_reference_fixtures() {
        errors.extend(direct_broadcast_orbit_errors(&fixture));
    }

    let summary = summarize_orbit_errors(&errors);
    assert_eq!(
        summary.sample_count, 10,
        "broadcast orbit reference comparison should cover every documented sample epoch"
    );
    assert!(
        summary.max_position_error_m <= GPS_BROADCAST_POSITION_TOLERANCE_M,
        "broadcast orbit max error drifted to {:.3} m across {} samples",
        summary.max_position_error_m,
        summary.sample_count
    );
    assert!(
        summary.rms_position_error_m <= GPS_BROADCAST_POSITION_RMS_TOLERANCE_M,
        "broadcast orbit RMS error drifted to {:.3} m across {} samples",
        summary.rms_position_error_m,
        summary.sample_count
    );
}
