#![allow(missing_docs)]

use bijux_gnss_signal::api::{
    advance_code_phase_chips, advance_code_phase_seconds, CA_CODE_PERIOD_CHIPS,
};

const GPS_L1_CA_CODE_RATE_HZ: f64 = 1_023_000.0;
const SIXTY_SECONDS: f64 = 60.0;
const PHASE_TOLERANCE_CHIPS: f64 = 1e-9;

#[test]
fn sampled_ca_code_phase_returns_to_the_same_chip_after_sixty_seconds() {
    let cases = [(4_000_000.0, 0.0), (4_092_000.0, 137.625), (5_000_000.0, 1022.75)];

    for (sample_rate_hz, start_chip_phase) in cases {
        let total_samples = (SIXTY_SECONDS * sample_rate_hz) as usize;
        let phase_from_samples = advance_code_phase_chips(
            start_chip_phase,
            sample_rate_hz,
            GPS_L1_CA_CODE_RATE_HZ,
            total_samples,
            CA_CODE_PERIOD_CHIPS,
        )
        .expect("valid sample-count phase advance");
        let phase_from_seconds = advance_code_phase_seconds(
            start_chip_phase,
            GPS_L1_CA_CODE_RATE_HZ,
            SIXTY_SECONDS,
            CA_CODE_PERIOD_CHIPS,
        )
        .expect("valid elapsed-time phase advance");

        assert_phase_close(
            phase_from_samples,
            phase_from_seconds,
            "sample-count and elapsed-time phase models diverged",
        );
        assert_phase_close(
            phase_from_seconds,
            start_chip_phase.rem_euclid(CA_CODE_PERIOD_CHIPS as f64),
            "sixty-second phase no longer wraps back to the injected origin",
        );
    }
}

fn assert_phase_close(actual: f64, expected: f64, message: &str) {
    let delta = (actual - expected).abs();
    assert!(
        delta <= PHASE_TOLERANCE_CHIPS,
        "{message}: actual={actual:.12}, expected={expected:.12}, delta={delta:.12}"
    );
}
