#![allow(missing_docs)]

use bijux_gnss_signal::api::{generate_ca_code_chips, Prn, CA_CODE_PERIOD_CHIPS};

#[test]
fn ca_code_repeats_exactly_after_one_period_for_all_prns() {
    for prn in 1u8..=32 {
        let chips = generate_ca_code_chips(Prn(prn), CA_CODE_PERIOD_CHIPS * 2).expect("valid PRN");
        let (first_period, second_period) = chips.split_at(CA_CODE_PERIOD_CHIPS);
        assert_eq!(first_period, second_period, "period repetition mismatch for PRN {prn}");
    }
}

#[test]
fn ca_code_third_period_matches_first_period_for_all_prns() {
    for prn in 1u8..=32 {
        let chips = generate_ca_code_chips(Prn(prn), CA_CODE_PERIOD_CHIPS * 3).expect("valid PRN");
        let first_period = &chips[..CA_CODE_PERIOD_CHIPS];
        let third_period = &chips[(CA_CODE_PERIOD_CHIPS * 2)..];
        assert_eq!(first_period, third_period, "third-period repetition mismatch for PRN {prn}");
    }
}
