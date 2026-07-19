#![allow(missing_docs)]

use bijux_gnss_signal::api::{generate_ca_code, generate_ca_code_chips, Prn, CA_CODE_PERIOD_CHIPS};

#[test]
fn ca_code_period_length_is_1023_for_all_prns() {
    for prn in 1u8..=32 {
        let code = generate_ca_code(Prn(prn)).expect("valid PRN");
        assert_eq!(code.len(), CA_CODE_PERIOD_CHIPS, "period length mismatch for PRN {prn}");
    }
}

#[test]
fn exact_period_generator_matches_single_period_helper() {
    for prn in 1u8..=32 {
        let expected = generate_ca_code(Prn(prn)).expect("valid PRN");
        let generated = generate_ca_code_chips(Prn(prn), CA_CODE_PERIOD_CHIPS).expect("valid PRN");
        assert_eq!(generated, expected, "single-period generation mismatch for PRN {prn}");
    }
}
