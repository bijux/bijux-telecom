#![allow(missing_docs)]

mod support;

use bijux_gnss_nav::api::sat_state_gps_l1ca;
use support::broadcast_reference::{gps_prn1_20220513_fixture, gps_prn2_20220514_fixture};

const GPS_BROADCAST_CLOCK_TOLERANCE_S: f64 = 70e-9;

#[test]
fn broadcast_clock_matches_igs_reference_within_documented_tolerance() {
    for fixture in [gps_prn1_20220513_fixture(), gps_prn2_20220514_fixture()] {
        for (&transmit_tow_s, &reference_t_s) in fixture
            .transmit_times_s
            .iter()
            .zip(fixture.reference_times_s.iter())
        {
            let broadcast = sat_state_gps_l1ca(&fixture.ephemeris, transmit_tow_s, 0.0);
            let precise_clock_s = fixture
                .clk
                .bias_s(fixture.sat, reference_t_s)
                .expect("CLK bias for broadcast comparison");
            let clock_error_s = broadcast.clock_correction.bias_s - precise_clock_s;

            assert!(
                clock_error_s.abs() <= GPS_BROADCAST_CLOCK_TOLERANCE_S,
                "{} at TOW {:.1}s drifted {:.3} ns against IGS precise clock",
                fixture.label,
                transmit_tow_s,
                clock_error_s * 1e9
            );
        }
    }
}
