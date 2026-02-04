#![allow(missing_docs)]
use bijux_gnss_signal::api::{generate_ca_code, Prn};
use proptest::prelude::*;

proptest! {
    #[test]
    fn ca_code_length_is_1023(prn in 1u8..=32u8) {
        let code = generate_ca_code(Prn(prn)).expect("valid PRN");
        prop_assert_eq!(code.len(), 1023);
    }

    #[test]
    fn ca_code_outputs_are_pm_one(prn in 1u8..=32u8) {
        let code = generate_ca_code(Prn(prn)).expect("valid PRN");
        prop_assert!(code.iter().all(|&c| c == 1 || c == -1));
    }

    #[test]
    fn ca_code_not_all_same(prn in 1u8..=32u8) {
        let code = generate_ca_code(Prn(prn)).expect("valid PRN");
        let first = code[0];
        prop_assert!(code.iter().any(|&c| c != first));
    }
}
