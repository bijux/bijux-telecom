#![allow(missing_docs)]

mod support;

use bijux_gnss_signal::api::{ca_code_assignment, generate_ca_code, Prn, SignalError};

use support::ca_reference::{assert_code_matches_reference, load_reference_catalog};
use support::period_reference::{assert_period_repetition, logical_bits_from_bipolar};

#[derive(Clone, Copy)]
struct ExpectedCaCodeReference {
    prn: u8,
    g2_taps: (u8, u8),
    g2_delay_chips: u16,
    first_ten_chips_octal: u16,
}

const EXPECTED_CA_CODE_REFERENCES: [ExpectedCaCodeReference; 32] = [
    ExpectedCaCodeReference {
        prn: 1,
        g2_taps: (2, 6),
        g2_delay_chips: 5,
        first_ten_chips_octal: 1440,
    },
    ExpectedCaCodeReference {
        prn: 2,
        g2_taps: (3, 7),
        g2_delay_chips: 6,
        first_ten_chips_octal: 1620,
    },
    ExpectedCaCodeReference {
        prn: 3,
        g2_taps: (4, 8),
        g2_delay_chips: 7,
        first_ten_chips_octal: 1710,
    },
    ExpectedCaCodeReference {
        prn: 4,
        g2_taps: (5, 9),
        g2_delay_chips: 8,
        first_ten_chips_octal: 1744,
    },
    ExpectedCaCodeReference {
        prn: 5,
        g2_taps: (1, 9),
        g2_delay_chips: 17,
        first_ten_chips_octal: 1133,
    },
    ExpectedCaCodeReference {
        prn: 6,
        g2_taps: (2, 10),
        g2_delay_chips: 18,
        first_ten_chips_octal: 1455,
    },
    ExpectedCaCodeReference {
        prn: 7,
        g2_taps: (1, 8),
        g2_delay_chips: 139,
        first_ten_chips_octal: 1131,
    },
    ExpectedCaCodeReference {
        prn: 8,
        g2_taps: (2, 9),
        g2_delay_chips: 140,
        first_ten_chips_octal: 1454,
    },
    ExpectedCaCodeReference {
        prn: 9,
        g2_taps: (3, 10),
        g2_delay_chips: 141,
        first_ten_chips_octal: 1626,
    },
    ExpectedCaCodeReference {
        prn: 10,
        g2_taps: (2, 3),
        g2_delay_chips: 251,
        first_ten_chips_octal: 1504,
    },
    ExpectedCaCodeReference {
        prn: 11,
        g2_taps: (3, 4),
        g2_delay_chips: 252,
        first_ten_chips_octal: 1642,
    },
    ExpectedCaCodeReference {
        prn: 12,
        g2_taps: (5, 6),
        g2_delay_chips: 254,
        first_ten_chips_octal: 1750,
    },
    ExpectedCaCodeReference {
        prn: 13,
        g2_taps: (6, 7),
        g2_delay_chips: 255,
        first_ten_chips_octal: 1764,
    },
    ExpectedCaCodeReference {
        prn: 14,
        g2_taps: (7, 8),
        g2_delay_chips: 256,
        first_ten_chips_octal: 1772,
    },
    ExpectedCaCodeReference {
        prn: 15,
        g2_taps: (8, 9),
        g2_delay_chips: 257,
        first_ten_chips_octal: 1775,
    },
    ExpectedCaCodeReference {
        prn: 16,
        g2_taps: (9, 10),
        g2_delay_chips: 258,
        first_ten_chips_octal: 1776,
    },
    ExpectedCaCodeReference {
        prn: 17,
        g2_taps: (1, 4),
        g2_delay_chips: 469,
        first_ten_chips_octal: 1156,
    },
    ExpectedCaCodeReference {
        prn: 18,
        g2_taps: (2, 5),
        g2_delay_chips: 470,
        first_ten_chips_octal: 1467,
    },
    ExpectedCaCodeReference {
        prn: 19,
        g2_taps: (3, 6),
        g2_delay_chips: 471,
        first_ten_chips_octal: 1633,
    },
    ExpectedCaCodeReference {
        prn: 20,
        g2_taps: (4, 7),
        g2_delay_chips: 472,
        first_ten_chips_octal: 1715,
    },
    ExpectedCaCodeReference {
        prn: 21,
        g2_taps: (5, 8),
        g2_delay_chips: 473,
        first_ten_chips_octal: 1746,
    },
    ExpectedCaCodeReference {
        prn: 22,
        g2_taps: (6, 9),
        g2_delay_chips: 474,
        first_ten_chips_octal: 1763,
    },
    ExpectedCaCodeReference {
        prn: 23,
        g2_taps: (1, 3),
        g2_delay_chips: 509,
        first_ten_chips_octal: 1063,
    },
    ExpectedCaCodeReference {
        prn: 24,
        g2_taps: (4, 6),
        g2_delay_chips: 512,
        first_ten_chips_octal: 1706,
    },
    ExpectedCaCodeReference {
        prn: 25,
        g2_taps: (5, 7),
        g2_delay_chips: 513,
        first_ten_chips_octal: 1743,
    },
    ExpectedCaCodeReference {
        prn: 26,
        g2_taps: (6, 8),
        g2_delay_chips: 514,
        first_ten_chips_octal: 1761,
    },
    ExpectedCaCodeReference {
        prn: 27,
        g2_taps: (7, 9),
        g2_delay_chips: 515,
        first_ten_chips_octal: 1770,
    },
    ExpectedCaCodeReference {
        prn: 28,
        g2_taps: (8, 10),
        g2_delay_chips: 516,
        first_ten_chips_octal: 1774,
    },
    ExpectedCaCodeReference {
        prn: 29,
        g2_taps: (1, 6),
        g2_delay_chips: 859,
        first_ten_chips_octal: 1127,
    },
    ExpectedCaCodeReference {
        prn: 30,
        g2_taps: (2, 7),
        g2_delay_chips: 860,
        first_ten_chips_octal: 1453,
    },
    ExpectedCaCodeReference {
        prn: 31,
        g2_taps: (3, 8),
        g2_delay_chips: 861,
        first_ten_chips_octal: 1625,
    },
    ExpectedCaCodeReference {
        prn: 32,
        g2_taps: (4, 9),
        g2_delay_chips: 862,
        first_ten_chips_octal: 1712,
    },
];

#[test]
fn published_assignments_match_official_reference_table() {
    for expected in EXPECTED_CA_CODE_REFERENCES {
        let assignment = ca_code_assignment(Prn(expected.prn)).expect("valid published PRN");
        assert_eq!(assignment.prn, Prn(expected.prn), "PRN mismatch for published assignment");
        assert_eq!(
            assignment.g2_taps, expected.g2_taps,
            "G2 tap mismatch for PRN {}",
            expected.prn
        );
        assert_eq!(
            assignment.g2_delay_chips, expected.g2_delay_chips,
            "G2 delay mismatch for PRN {}",
            expected.prn
        );
        assert_eq!(
            assignment.first_ten_chips_octal, expected.first_ten_chips_octal,
            "first-ten-chip octal mismatch for PRN {}",
            expected.prn
        );
    }
}

#[test]
fn generated_codes_match_official_first_ten_chip_references() {
    for expected in EXPECTED_CA_CODE_REFERENCES {
        let code = generate_ca_code(Prn(expected.prn)).expect("valid PRN");
        assert_eq!(
            &code[..10],
            &decode_first_ten_chips_octal(expected.first_ten_chips_octal),
            "first ten chips mismatch for PRN {}",
            expected.prn
        );
    }
}

#[test]
fn generated_codes_match_immutable_reference_catalog() {
    let catalog = load_reference_catalog();

    for reference in &catalog.code {
        let code = generate_ca_code(Prn(reference.prn)).expect("valid PRN");
        let logical_bits = logical_bits_from_bipolar(&code, "GPS C/A code");

        assert_code_matches_reference(&catalog, reference.prn, &logical_bits);
        assert_period_repetition(&code, catalog.code_length, &format!("GPS C/A PRN {}", reference.prn));
    }
}

#[test]
fn published_assignment_rejects_out_of_range_prns() {
    assert_eq!(ca_code_assignment(Prn(0)), Err(SignalError::UnsupportedPrn(0)));
    assert_eq!(ca_code_assignment(Prn(33)), Err(SignalError::UnsupportedPrn(33)));
}

fn decode_first_ten_chips_octal(octal_digits: u16) -> [i8; 10] {
    let first_chip = u8::try_from((octal_digits / 1000) % 10).expect("single decimal digit");
    let hundreds = u8::try_from((octal_digits / 100) % 10).expect("single decimal digit");
    let tens = u8::try_from((octal_digits / 10) % 10).expect("single decimal digit");
    let ones = u8::try_from(octal_digits % 10).expect("single decimal digit");

    assert!(first_chip <= 1, "first chip must be encoded as 0 or 1");
    assert!(hundreds < 8 && tens < 8 && ones < 8, "octal digits must be in range 0..7");

    let trailing_bits = (u16::from(hundreds) << 6) | (u16::from(tens) << 3) | u16::from(ones);
    let mut chips = [1i8; 10];
    chips[0] = chip_from_reference_bit(first_chip);

    for bit_index in 0..9 {
        let shift = 8 - bit_index;
        let bit = u8::from(((trailing_bits >> shift) & 1) != 0);
        chips[bit_index + 1] = chip_from_reference_bit(bit);
    }

    chips
}

fn chip_from_reference_bit(bit: u8) -> i8 {
    if bit == 1 {
        -1
    } else {
        1
    }
}
