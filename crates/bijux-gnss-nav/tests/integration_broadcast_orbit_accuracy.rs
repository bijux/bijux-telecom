#![allow(missing_docs)]

mod support;

use support::orbit_reference::{broadcast_reference_fixtures, direct_broadcast_orbit_errors};

const GPS_BROADCAST_POSITION_TOLERANCE_M: f64 = 5.0;

#[test]
fn broadcast_orbit_matches_external_reference_within_position_tolerance() {
    for fixture in broadcast_reference_fixtures() {
        for error in direct_broadcast_orbit_errors(&fixture) {
            assert!(
                error.position_error_m <= GPS_BROADCAST_POSITION_TOLERANCE_M,
                "{} {:?} at transmit TOW {:.1}s drifted {:.3} m against external orbit reference at {:.1}s (dx={:.3} m, dy={:.3} m, dz={:.3} m)",
                error.label,
                error.sat,
                error.transmit_tow_s,
                error.position_error_m,
                error.reference_t_s,
                error.dx_m,
                error.dy_m,
                error.dz_m
            );
        }
    }
}
