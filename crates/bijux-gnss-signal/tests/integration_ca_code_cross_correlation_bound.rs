#![allow(missing_docs)]

use bijux_gnss_signal::api::{
    ca_code_cross_correlation_summary, ca_code_periodic_cross_correlation, Prn,
    CA_CODE_PERIOD_CHIPS,
};

const EXPECTED_MAX_CROSS_CORRELATION_ABS: i16 = 65;

#[test]
fn ca_code_cross_correlation_respects_documented_bound_for_all_distinct_prns() {
    for left_prn in 1..=32 {
        for right_prn in (left_prn + 1)..=32 {
            let summary = ca_code_cross_correlation_summary(Prn(left_prn), Prn(right_prn))
                .expect("valid PRN pair");
            assert_eq!(
                summary.max_abs,
                EXPECTED_MAX_CROSS_CORRELATION_ABS,
                "cross-correlation bound mismatch for PRNs {left_prn} and {right_prn}"
            );
        }
    }
}

#[test]
fn ca_code_cross_correlation_never_reaches_a_full_period_peak_for_distinct_prns() {
    for left_prn in 1..=32 {
        for right_prn in (left_prn + 1)..=32 {
            let correlation =
                ca_code_periodic_cross_correlation(Prn(left_prn), Prn(right_prn))
                    .expect("valid PRN pair");
            assert!(
                correlation
                    .iter()
                    .all(|value| value.abs() < CA_CODE_PERIOD_CHIPS as i16),
                "cross-correlation produced a false full-period peak for PRNs {left_prn} and {right_prn}"
            );
        }
    }
}
