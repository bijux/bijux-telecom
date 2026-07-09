#![allow(missing_docs)]

use bijux_gnss_signal::api::{generate_ca_code, sample_ca_code, Prn};

const INTEGER_RATE_SAMPLE_RATE_HZ: f64 = 4_092_000.0;
const GPS_L1_CA_CODE_RATE_HZ: f64 = 1_023_000.0;
const SAMPLES_PER_CHIP: usize = 4;

#[test]
fn sampled_ca_code_matches_exact_chip_repetition_at_integer_rate() {
    for prn in 1..=32 {
        let chips = generate_ca_code(Prn(prn)).expect("valid PRN");
        let samples = sample_ca_code(
            Prn(prn),
            INTEGER_RATE_SAMPLE_RATE_HZ,
            GPS_L1_CA_CODE_RATE_HZ,
            0.0,
            chips.len() * SAMPLES_PER_CHIP,
        )
        .expect("valid integer-rate sampling");

        for (chip_index, chip) in chips.iter().enumerate() {
            let start = chip_index * SAMPLES_PER_CHIP;
            let end = start + SAMPLES_PER_CHIP;
            assert!(
                samples[start..end]
                    .iter()
                    .all(|sample| (*sample - *chip as f32).abs() < f32::EPSILON),
                "sampled code did not repeat chip {chip_index} correctly for PRN {prn}"
            );
        }
    }
}
