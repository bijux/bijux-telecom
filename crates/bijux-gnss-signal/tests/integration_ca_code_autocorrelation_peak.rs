#![allow(missing_docs)]

use bijux_gnss_signal::api::{
    ca_code_autocorrelation_summary, ca_code_periodic_autocorrelation, Prn, CA_CODE_PERIOD_CHIPS,
};

#[test]
fn ca_code_autocorrelation_peak_is_full_period_for_all_prns() {
    for prn in 1u8..=32 {
        let summary = ca_code_autocorrelation_summary(Prn(prn)).expect("valid PRN");
        assert_eq!(
            summary.peak, CA_CODE_PERIOD_CHIPS as i16,
            "autocorrelation peak mismatch for PRN {prn}"
        );
    }
}

#[test]
fn ca_code_autocorrelation_peak_is_unique_for_all_prns() {
    for prn in 1u8..=32 {
        let correlation = ca_code_periodic_autocorrelation(Prn(prn)).expect("valid PRN");
        let peak = correlation[0];
        assert_eq!(
            correlation.iter().filter(|&&value| value == peak).count(),
            1,
            "autocorrelation peak was not unique for PRN {prn}"
        );
    }
}
