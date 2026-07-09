#![allow(missing_docs)]

//! GPS L1 C/A code generation.
//!
//! Clean-room implementation based on public interface specifications.

/// PRN identifier for GPS C/A codes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Prn(pub u8);

/// Number of chips in one GPS L1 C/A code period.
pub const CA_CODE_PERIOD_CHIPS: usize = 1023;

/// Published GPS L1 C/A code assignment metadata for one PRN.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CaCodeAssignment {
    /// PRN identifier covered by this assignment.
    pub prn: Prn,
    /// One-based G2 tap indices selected for this PRN.
    pub g2_taps: (u8, u8),
    /// G2 code delay in chips from the published assignment table.
    pub g2_delay_chips: u16,
    /// Published first ten C/A chips in the octal notation used by the SPS signal specification.
    pub first_ten_chips_octal: u16,
}

impl CaCodeAssignment {
    fn zero_based_g2_taps(self) -> (usize, usize) {
        (usize::from(self.g2_taps.0 - 1), usize::from(self.g2_taps.1 - 1))
    }
}

const CA_CODE_ASSIGNMENTS: [CaCodeAssignment; 32] = [
    CaCodeAssignment {
        prn: Prn(1),
        g2_taps: (2, 6),
        g2_delay_chips: 5,
        first_ten_chips_octal: 1440,
    },
    CaCodeAssignment {
        prn: Prn(2),
        g2_taps: (3, 7),
        g2_delay_chips: 6,
        first_ten_chips_octal: 1620,
    },
    CaCodeAssignment {
        prn: Prn(3),
        g2_taps: (4, 8),
        g2_delay_chips: 7,
        first_ten_chips_octal: 1710,
    },
    CaCodeAssignment {
        prn: Prn(4),
        g2_taps: (5, 9),
        g2_delay_chips: 8,
        first_ten_chips_octal: 1744,
    },
    CaCodeAssignment {
        prn: Prn(5),
        g2_taps: (1, 9),
        g2_delay_chips: 17,
        first_ten_chips_octal: 1133,
    },
    CaCodeAssignment {
        prn: Prn(6),
        g2_taps: (2, 10),
        g2_delay_chips: 18,
        first_ten_chips_octal: 1455,
    },
    CaCodeAssignment {
        prn: Prn(7),
        g2_taps: (1, 8),
        g2_delay_chips: 139,
        first_ten_chips_octal: 1131,
    },
    CaCodeAssignment {
        prn: Prn(8),
        g2_taps: (2, 9),
        g2_delay_chips: 140,
        first_ten_chips_octal: 1454,
    },
    CaCodeAssignment {
        prn: Prn(9),
        g2_taps: (3, 10),
        g2_delay_chips: 141,
        first_ten_chips_octal: 1626,
    },
    CaCodeAssignment {
        prn: Prn(10),
        g2_taps: (2, 3),
        g2_delay_chips: 251,
        first_ten_chips_octal: 1504,
    },
    CaCodeAssignment {
        prn: Prn(11),
        g2_taps: (3, 4),
        g2_delay_chips: 252,
        first_ten_chips_octal: 1642,
    },
    CaCodeAssignment {
        prn: Prn(12),
        g2_taps: (5, 6),
        g2_delay_chips: 254,
        first_ten_chips_octal: 1750,
    },
    CaCodeAssignment {
        prn: Prn(13),
        g2_taps: (6, 7),
        g2_delay_chips: 255,
        first_ten_chips_octal: 1764,
    },
    CaCodeAssignment {
        prn: Prn(14),
        g2_taps: (7, 8),
        g2_delay_chips: 256,
        first_ten_chips_octal: 1772,
    },
    CaCodeAssignment {
        prn: Prn(15),
        g2_taps: (8, 9),
        g2_delay_chips: 257,
        first_ten_chips_octal: 1775,
    },
    CaCodeAssignment {
        prn: Prn(16),
        g2_taps: (9, 10),
        g2_delay_chips: 258,
        first_ten_chips_octal: 1776,
    },
    CaCodeAssignment {
        prn: Prn(17),
        g2_taps: (1, 4),
        g2_delay_chips: 469,
        first_ten_chips_octal: 1156,
    },
    CaCodeAssignment {
        prn: Prn(18),
        g2_taps: (2, 5),
        g2_delay_chips: 470,
        first_ten_chips_octal: 1467,
    },
    CaCodeAssignment {
        prn: Prn(19),
        g2_taps: (3, 6),
        g2_delay_chips: 471,
        first_ten_chips_octal: 1633,
    },
    CaCodeAssignment {
        prn: Prn(20),
        g2_taps: (4, 7),
        g2_delay_chips: 472,
        first_ten_chips_octal: 1715,
    },
    CaCodeAssignment {
        prn: Prn(21),
        g2_taps: (5, 8),
        g2_delay_chips: 473,
        first_ten_chips_octal: 1746,
    },
    CaCodeAssignment {
        prn: Prn(22),
        g2_taps: (6, 9),
        g2_delay_chips: 474,
        first_ten_chips_octal: 1763,
    },
    CaCodeAssignment {
        prn: Prn(23),
        g2_taps: (1, 3),
        g2_delay_chips: 509,
        first_ten_chips_octal: 1063,
    },
    CaCodeAssignment {
        prn: Prn(24),
        g2_taps: (4, 6),
        g2_delay_chips: 512,
        first_ten_chips_octal: 1706,
    },
    CaCodeAssignment {
        prn: Prn(25),
        g2_taps: (5, 7),
        g2_delay_chips: 513,
        first_ten_chips_octal: 1743,
    },
    CaCodeAssignment {
        prn: Prn(26),
        g2_taps: (6, 8),
        g2_delay_chips: 514,
        first_ten_chips_octal: 1761,
    },
    CaCodeAssignment {
        prn: Prn(27),
        g2_taps: (7, 9),
        g2_delay_chips: 515,
        first_ten_chips_octal: 1770,
    },
    CaCodeAssignment {
        prn: Prn(28),
        g2_taps: (8, 10),
        g2_delay_chips: 516,
        first_ten_chips_octal: 1774,
    },
    CaCodeAssignment {
        prn: Prn(29),
        g2_taps: (1, 6),
        g2_delay_chips: 859,
        first_ten_chips_octal: 1127,
    },
    CaCodeAssignment {
        prn: Prn(30),
        g2_taps: (2, 7),
        g2_delay_chips: 860,
        first_ten_chips_octal: 1453,
    },
    CaCodeAssignment {
        prn: Prn(31),
        g2_taps: (3, 8),
        g2_delay_chips: 861,
        first_ten_chips_octal: 1625,
    },
    CaCodeAssignment {
        prn: Prn(32),
        g2_taps: (4, 9),
        g2_delay_chips: 862,
        first_ten_chips_octal: 1712,
    },
];

/// Return the published GPS L1 C/A assignment metadata for one PRN.
pub fn ca_code_assignment(
    prn: Prn,
) -> Result<&'static CaCodeAssignment, crate::error::SignalError> {
    prn_index(prn).map(|index| &CA_CODE_ASSIGNMENTS[index])
}

/// Return the published GPS L1 C/A assignment metadata for PRNs 1 through 32.
pub fn ca_code_assignments() -> &'static [CaCodeAssignment; 32] {
    &CA_CODE_ASSIGNMENTS
}

/// Generate one 1023-chip C/A code sequence for a given PRN (1..=32).
///
/// Returns chips in {-1, +1}.
pub fn generate_ca_code(prn: Prn) -> Result<Vec<i8>, crate::error::SignalError> {
    generate_ca_code_chips(prn, CA_CODE_PERIOD_CHIPS)
}

/// Generate a GPS L1 C/A code sequence of arbitrary length for a given PRN (1..=32).
///
/// Returns chips in {-1, +1}.
pub fn generate_ca_code_chips(
    prn: Prn,
    chip_count: usize,
) -> Result<Vec<i8>, crate::error::SignalError> {
    let (tap1, tap2) = ca_code_assignment(prn)?.zero_based_g2_taps();
    let mut g1 = [1i8; 10];
    let mut g2 = [1i8; 10];

    let mut code = Vec::with_capacity(chip_count);

    for _ in 0..chip_count {
        let g1_out = g1[9];
        let g2_out = g2[tap1] ^ g2[tap2];
        let chip = g1_out ^ g2_out;
        code.push(if chip == 0 { 1 } else { -1 });

        let g1_feedback = g1[2] ^ g1[9];
        let g2_feedback = g2[1] ^ g2[2] ^ g2[5] ^ g2[7] ^ g2[8] ^ g2[9];

        shift_register(&mut g1, g1_feedback);
        shift_register(&mut g2, g2_feedback);
    }

    Ok(code)
}

fn shift_register(reg: &mut [i8; 10], feedback: i8) {
    for i in (1..10).rev() {
        reg[i] = reg[i - 1];
    }
    reg[0] = feedback & 1;
}

fn prn_index(prn: Prn) -> Result<usize, crate::error::SignalError> {
    match prn.0 {
        1..=32 => Ok(usize::from(prn.0 - 1)),
        _ => Err(crate::error::SignalError::UnsupportedPrn(prn.0)),
    }
}
