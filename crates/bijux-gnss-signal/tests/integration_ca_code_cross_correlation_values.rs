#![allow(missing_docs)]

use bijux_gnss_signal::api::{ca_code_cross_correlation_summary, Prn};

const EXPECTED_CROSS_CORRELATION_VALUES: [i16; 3] = [-65, -1, 63];

#[test]
fn ca_code_cross_correlation_values_match_expected_set_for_all_distinct_prns() {
    for left_prn in 1..=32 {
        for right_prn in (left_prn + 1)..=32 {
            let summary = ca_code_cross_correlation_summary(Prn(left_prn), Prn(right_prn))
                .expect("valid PRN pair");
            assert_eq!(
                summary.unique_values, EXPECTED_CROSS_CORRELATION_VALUES,
                "cross-correlation value set mismatch for PRNs {left_prn} and {right_prn}"
            );
        }
    }
}
