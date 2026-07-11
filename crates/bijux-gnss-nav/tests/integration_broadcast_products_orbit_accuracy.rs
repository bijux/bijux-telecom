#![allow(missing_docs)]

mod support;

use support::orbit_reference::{broadcast_reference_fixtures, provider_broadcast_orbit_comparison};

const GPS_BROADCAST_POSITION_TOLERANCE_M: f64 = 5.0;

#[test]
fn broadcast_products_provider_matches_external_reference_without_fallbacks() {
    for fixture in broadcast_reference_fixtures() {
        let comparison = provider_broadcast_orbit_comparison(&fixture);
        assert!(
            comparison.fallbacks.is_empty(),
            "{} broadcast provider reported unexpected fallbacks: {}",
            fixture.label,
            comparison.fallbacks.join("; ")
        );

        for error in comparison.errors {
            assert!(
                error.position_error_m <= GPS_BROADCAST_POSITION_TOLERANCE_M,
                "{} {:?} provider state at transmit TOW {:.1}s drifted {:.3} m against external orbit reference at {:.1}s",
                error.label,
                error.sat,
                error.transmit_tow_s,
                error.position_error_m,
                error.reference_t_s
            );
        }
    }
}
