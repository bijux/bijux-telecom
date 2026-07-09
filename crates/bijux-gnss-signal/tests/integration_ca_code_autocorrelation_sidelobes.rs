#![allow(missing_docs)]

use bijux_gnss_signal::api::{ca_code_autocorrelation_summary, Prn};

const EXPECTED_NONZERO_AUTOCORRELATION_VALUES: [i16; 3] = [-65, -1, 63];
const EXPECTED_MAX_NONZERO_ABS_SIDELOBE: i16 = 65;

#[test]
fn ca_code_autocorrelation_sidelobes_match_expected_value_set() {
    for prn in 1u8..=32 {
        let summary = ca_code_autocorrelation_summary(Prn(prn)).expect("valid PRN");
        assert_eq!(
            summary.unique_nonzero_values, EXPECTED_NONZERO_AUTOCORRELATION_VALUES,
            "autocorrelation sidelobe set mismatch for PRN {prn}"
        );
    }
}

#[test]
fn ca_code_autocorrelation_sidelobes_respect_documented_bound() {
    for prn in 1u8..=32 {
        let summary = ca_code_autocorrelation_summary(Prn(prn)).expect("valid PRN");
        assert_eq!(
            summary.max_nonzero_abs, EXPECTED_MAX_NONZERO_ABS_SIDELOBE,
            "autocorrelation sidelobe bound mismatch for PRN {prn}"
        );
    }
}
