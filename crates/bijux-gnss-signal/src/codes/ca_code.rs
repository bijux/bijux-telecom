#![allow(missing_docs)]

//! GPS L1 C/A code generation.
//!
//! Clean-room implementation based on public interface specifications.

/// PRN identifier for GPS C/A codes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Prn(pub u8);

/// Generate one 1023-chip C/A code sequence for a given PRN (1..=32).
///
/// Returns chips in {-1, +1}.
pub fn generate_ca_code(prn: Prn) -> Vec<i8> {
    let prn = prn.0;
    assert!(
        (1..=32).contains(&prn),
        "PRN must be in 1..=32 for GPS L1 C/A"
    );

    let (tap1, tap2) = g2_taps(prn);
    let mut g1 = [1i8; 10];
    let mut g2 = [1i8; 10];

    let mut code = Vec::with_capacity(1023);

    for _ in 0..1023 {
        let g1_out = g1[9];
        let g2_out = g2[tap1] ^ g2[tap2];
        let chip = g1_out ^ g2_out;
        code.push(if chip == 0 { 1 } else { -1 });

        let g1_feedback = g1[2] ^ g1[9];
        let g2_feedback = g2[1] ^ g2[2] ^ g2[5] ^ g2[7] ^ g2[8] ^ g2[9];

        shift_register(&mut g1, g1_feedback);
        shift_register(&mut g2, g2_feedback);
    }

    code
}

fn shift_register(reg: &mut [i8; 10], feedback: i8) {
    for i in (1..10).rev() {
        reg[i] = reg[i - 1];
    }
    reg[0] = feedback & 1;
}

fn g2_taps(prn: u8) -> (usize, usize) {
    match prn {
        1 => (1, 5),
        2 => (2, 6),
        3 => (3, 7),
        4 => (4, 8),
        5 => (0, 8),
        6 => (1, 9),
        7 => (0, 7),
        8 => (1, 8),
        9 => (2, 9),
        10 => (1, 2),
        11 => (2, 3),
        12 => (4, 5),
        13 => (5, 6),
        14 => (6, 7),
        15 => (7, 8),
        16 => (8, 9),
        17 => (0, 3),
        18 => (1, 4),
        19 => (2, 5),
        20 => (3, 6),
        21 => (4, 7),
        22 => (5, 8),
        23 => (0, 2),
        24 => (3, 5),
        25 => (4, 6),
        26 => (5, 7),
        27 => (6, 8),
        28 => (7, 9),
        29 => (0, 5),
        30 => (1, 6),
        31 => (2, 7),
        32 => (3, 8),
        _ => panic!("Unsupported PRN {prn}"),
    }
}
