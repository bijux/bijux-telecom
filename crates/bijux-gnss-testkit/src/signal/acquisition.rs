//! Independent acquisition code-phase conventions for synthetic validation.
#![allow(missing_docs)]

const GPS_L1_CA_CODE_RATE_HZ: f64 = 1_023_000.0;
const GPS_L1_CA_CODE_LENGTH_CHIPS: usize = 1023;

pub fn gps_l1ca_expected_acquisition_code_phase_samples(
    sample_rate_hz: f64,
    sample_index: u64,
    code_phase_chips: f64,
) -> usize {
    gps_l1ca_expected_acquisition_code_phase_samples_f64(
        sample_rate_hz,
        sample_index,
        code_phase_chips,
    )
    .round() as usize
}

pub fn gps_l1ca_expected_acquisition_code_phase_samples_f64(
    sample_rate_hz: f64,
    sample_index: u64,
    code_phase_chips: f64,
) -> f64 {
    expected_acquisition_code_phase_samples_f64(
        sample_rate_hz,
        GPS_L1_CA_CODE_RATE_HZ,
        GPS_L1_CA_CODE_LENGTH_CHIPS,
        sample_index,
        code_phase_chips,
    )
}

pub fn expected_acquisition_code_phase_samples_f64(
    sample_rate_hz: f64,
    code_rate_hz: f64,
    code_length_chips: usize,
    sample_index: u64,
    code_phase_chips: f64,
) -> f64 {
    let period_samples =
        samples_per_code(sample_rate_hz, code_rate_hz, code_length_chips).max(1) as f64;
    let phase_samples = code_phase_samples_at_sample_index(
        sample_rate_hz,
        code_rate_hz,
        code_length_chips,
        sample_index,
        code_phase_chips,
    );
    (period_samples - phase_samples.rem_euclid(period_samples)).rem_euclid(period_samples)
}

pub fn wrapped_code_phase_error_samples_f64(
    actual_samples: f64,
    expected_samples: f64,
    period_samples: usize,
) -> f64 {
    let period_samples = period_samples.max(1) as f64;
    let forward_error = (actual_samples - expected_samples).abs().rem_euclid(period_samples);
    let wrapped_error = (period_samples - forward_error).rem_euclid(period_samples);
    forward_error.min(wrapped_error)
}

pub fn samples_per_code(sample_rate_hz: f64, code_rate_hz: f64, code_length_chips: usize) -> usize {
    (sample_rate_hz / (code_rate_hz / code_length_chips as f64)).round() as usize
}

fn code_phase_samples_at_sample_index(
    sample_rate_hz: f64,
    code_rate_hz: f64,
    code_length_chips: usize,
    sample_index: u64,
    initial_code_phase_chips: f64,
) -> f64 {
    let elapsed_seconds = sample_index as f64 / sample_rate_hz;
    let advanced_chip_phase = advance_code_phase_seconds(
        initial_code_phase_chips,
        code_rate_hz,
        elapsed_seconds,
        code_length_chips,
    );
    let samples_per_chip = sample_rate_hz / code_rate_hz;
    advanced_chip_phase * samples_per_chip
}

fn advance_code_phase_seconds(
    start_chip_phase: f64,
    code_rate_hz: f64,
    elapsed_seconds: f64,
    code_length_chips: usize,
) -> f64 {
    (start_chip_phase + elapsed_seconds * code_rate_hz).rem_euclid(code_length_chips as f64)
}

#[cfg(test)]
mod tests {
    use super::{
        gps_l1ca_expected_acquisition_code_phase_samples,
        gps_l1ca_expected_acquisition_code_phase_samples_f64, samples_per_code,
        wrapped_code_phase_error_samples_f64,
    };

    #[test]
    fn l1ca_fractional_phase_matches_receiver_search_convention() {
        let phase_samples =
            gps_l1ca_expected_acquisition_code_phase_samples_f64(4_092_000.0, 0, 0.125);

        assert!((phase_samples - 4091.5).abs() < 1.0e-9);
    }

    #[test]
    fn l1ca_integer_phase_rounds_to_nearest_sample() {
        let phase_samples = gps_l1ca_expected_acquisition_code_phase_samples(4_092_000.0, 0, 0.0);

        assert_eq!(phase_samples, 0);
    }

    #[test]
    fn wrapped_error_uses_shorter_period_distance() {
        let period_samples = samples_per_code(4_092_000.0, 1_023_000.0, 1023);
        let error_samples = wrapped_code_phase_error_samples_f64(4091.5, 0.0, period_samples);

        assert!((error_samples - 0.5).abs() < 1.0e-9);
    }
}
