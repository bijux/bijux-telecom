#![allow(missing_docs)]

mod support;

use bijux_gnss_nav::api::sat_state_gps_l1ca;
use support::broadcast_reference::{gps_prn1_20220513_fixture, gps_prn2_20220514_fixture};

const GPS_BROADCAST_POSITION_TOLERANCE_M: f64 = 5.0;

#[test]
fn broadcast_position_matches_igs_reference_within_documented_tolerance() {
    for fixture in [gps_prn1_20220513_fixture(), gps_prn2_20220514_fixture()] {
        for (&transmit_tow_s, &reference_t_s) in
            fixture.transmit_times_s.iter().zip(fixture.reference_times_s.iter())
        {
            let broadcast = sat_state_gps_l1ca(&fixture.ephemeris, transmit_tow_s, 0.0);
            let precise = fixture
                .sp3
                .sat_state(fixture.sat, reference_t_s)
                .expect("SP3 state for broadcast comparison");
            let dx = broadcast.x_m - precise.x_m;
            let dy = broadcast.y_m - precise.y_m;
            let dz = broadcast.z_m - precise.z_m;
            let position_error_m = (dx * dx + dy * dy + dz * dz).sqrt();

            assert!(
                position_error_m <= GPS_BROADCAST_POSITION_TOLERANCE_M,
                "{} at TOW {:.1}s drifted {:.3} m against IGS precise orbit",
                fixture.label,
                transmit_tow_s,
                position_error_m
            );
        }
    }
}
