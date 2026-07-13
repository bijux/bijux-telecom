#![allow(missing_docs)]

//! GPS L2C CM ranging-code assignments.
//!
//! Clean-room implementation derived from the public IS-GPS-200L signal definition.

use super::gps_l2c_register::{generate_bipolar_code_period, generate_bipolar_code_range};
use crate::dsp::signal::sample_code;
use crate::error::SignalError;

/// Number of chips in one published GPS L2C CM code period.
pub const GPS_L2C_CM_CODE_CHIPS: usize = 10_230;

/// Code rate of the GPS L2C CM ranging code.
pub const GPS_L2C_CM_CODE_RATE_HZ: f64 = 511_500.0;

/// Published shift-register assignment for one GPS L2C CM PRN.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GpsL2cCmCodeAssignment {
    pub prn: u8,
    pub initial_state_octal: u32,
    pub end_state_octal: u32,
}

const GPS_L2C_CM_CODE_ASSIGNMENTS: [GpsL2cCmCodeAssignment; 115] = [
    GpsL2cCmCodeAssignment { prn: 1, initial_state_octal: 0o742417664, end_state_octal: 0o552566002 },
    GpsL2cCmCodeAssignment { prn: 2, initial_state_octal: 0o756014035, end_state_octal: 0o034445034 },
    GpsL2cCmCodeAssignment { prn: 3, initial_state_octal: 0o002747144, end_state_octal: 0o723443711 },
    GpsL2cCmCodeAssignment { prn: 4, initial_state_octal: 0o066265724, end_state_octal: 0o511222013 },
    GpsL2cCmCodeAssignment { prn: 5, initial_state_octal: 0o601403471, end_state_octal: 0o463055213 },
    GpsL2cCmCodeAssignment { prn: 6, initial_state_octal: 0o703232733, end_state_octal: 0o667044524 },
    GpsL2cCmCodeAssignment { prn: 7, initial_state_octal: 0o124510070, end_state_octal: 0o652322653 },
    GpsL2cCmCodeAssignment { prn: 8, initial_state_octal: 0o617316361, end_state_octal: 0o505703344 },
    GpsL2cCmCodeAssignment { prn: 9, initial_state_octal: 0o047541621, end_state_octal: 0o520302775 },
    GpsL2cCmCodeAssignment { prn: 10, initial_state_octal: 0o733031046, end_state_octal: 0o244205506 },
    GpsL2cCmCodeAssignment { prn: 11, initial_state_octal: 0o713512145, end_state_octal: 0o236174002 },
    GpsL2cCmCodeAssignment { prn: 12, initial_state_octal: 0o024437606, end_state_octal: 0o654305531 },
    GpsL2cCmCodeAssignment { prn: 13, initial_state_octal: 0o021264003, end_state_octal: 0o435070571 },
    GpsL2cCmCodeAssignment { prn: 14, initial_state_octal: 0o230655351, end_state_octal: 0o630431251 },
    GpsL2cCmCodeAssignment { prn: 15, initial_state_octal: 0o001314400, end_state_octal: 0o234043417 },
    GpsL2cCmCodeAssignment { prn: 16, initial_state_octal: 0o222021506, end_state_octal: 0o535540745 },
    GpsL2cCmCodeAssignment { prn: 17, initial_state_octal: 0o540264026, end_state_octal: 0o043056734 },
    GpsL2cCmCodeAssignment { prn: 18, initial_state_octal: 0o205521705, end_state_octal: 0o731304103 },
    GpsL2cCmCodeAssignment { prn: 19, initial_state_octal: 0o064022144, end_state_octal: 0o412120105 },
    GpsL2cCmCodeAssignment { prn: 20, initial_state_octal: 0o120161274, end_state_octal: 0o365636111 },
    GpsL2cCmCodeAssignment { prn: 21, initial_state_octal: 0o044023533, end_state_octal: 0o143324657 },
    GpsL2cCmCodeAssignment { prn: 22, initial_state_octal: 0o724744327, end_state_octal: 0o110766462 },
    GpsL2cCmCodeAssignment { prn: 23, initial_state_octal: 0o045743577, end_state_octal: 0o602405203 },
    GpsL2cCmCodeAssignment { prn: 24, initial_state_octal: 0o741201660, end_state_octal: 0o177735650 },
    GpsL2cCmCodeAssignment { prn: 25, initial_state_octal: 0o700274134, end_state_octal: 0o630177560 },
    GpsL2cCmCodeAssignment { prn: 26, initial_state_octal: 0o010247261, end_state_octal: 0o653467107 },
    GpsL2cCmCodeAssignment { prn: 27, initial_state_octal: 0o713433445, end_state_octal: 0o406576630 },
    GpsL2cCmCodeAssignment { prn: 28, initial_state_octal: 0o737324162, end_state_octal: 0o221777100 },
    GpsL2cCmCodeAssignment { prn: 29, initial_state_octal: 0o311627434, end_state_octal: 0o773266673 },
    GpsL2cCmCodeAssignment { prn: 30, initial_state_octal: 0o710452007, end_state_octal: 0o100010710 },
    GpsL2cCmCodeAssignment { prn: 31, initial_state_octal: 0o722462133, end_state_octal: 0o431037132 },
    GpsL2cCmCodeAssignment { prn: 32, initial_state_octal: 0o050172213, end_state_octal: 0o624127475 },
    GpsL2cCmCodeAssignment { prn: 33, initial_state_octal: 0o500653703, end_state_octal: 0o154624012 },
    GpsL2cCmCodeAssignment { prn: 34, initial_state_octal: 0o755077436, end_state_octal: 0o275636742 },
    GpsL2cCmCodeAssignment { prn: 35, initial_state_octal: 0o136717361, end_state_octal: 0o644341556 },
    GpsL2cCmCodeAssignment { prn: 36, initial_state_octal: 0o756675453, end_state_octal: 0o514260662 },
    GpsL2cCmCodeAssignment { prn: 37, initial_state_octal: 0o435506112, end_state_octal: 0o133501670 },
    GpsL2cCmCodeAssignment { prn: 38, initial_state_octal: 0o771353753, end_state_octal: 0o453413162 },
    GpsL2cCmCodeAssignment { prn: 39, initial_state_octal: 0o226107701, end_state_octal: 0o637760505 },
    GpsL2cCmCodeAssignment { prn: 40, initial_state_octal: 0o022025110, end_state_octal: 0o612775765 },
    GpsL2cCmCodeAssignment { prn: 41, initial_state_octal: 0o402466344, end_state_octal: 0o136315217 },
    GpsL2cCmCodeAssignment { prn: 42, initial_state_octal: 0o752566114, end_state_octal: 0o264252240 },
    GpsL2cCmCodeAssignment { prn: 43, initial_state_octal: 0o702011164, end_state_octal: 0o113027466 },
    GpsL2cCmCodeAssignment { prn: 44, initial_state_octal: 0o041216771, end_state_octal: 0o774524245 },
    GpsL2cCmCodeAssignment { prn: 45, initial_state_octal: 0o047457275, end_state_octal: 0o161633757 },
    GpsL2cCmCodeAssignment { prn: 46, initial_state_octal: 0o266333164, end_state_octal: 0o603442167 },
    GpsL2cCmCodeAssignment { prn: 47, initial_state_octal: 0o713167356, end_state_octal: 0o213146546 },
    GpsL2cCmCodeAssignment { prn: 48, initial_state_octal: 0o060546335, end_state_octal: 0o721323277 },
    GpsL2cCmCodeAssignment { prn: 49, initial_state_octal: 0o355173035, end_state_octal: 0o207073253 },
    GpsL2cCmCodeAssignment { prn: 50, initial_state_octal: 0o617201036, end_state_octal: 0o130632332 },
    GpsL2cCmCodeAssignment { prn: 51, initial_state_octal: 0o157465571, end_state_octal: 0o606370621 },
    GpsL2cCmCodeAssignment { prn: 52, initial_state_octal: 0o767360553, end_state_octal: 0o330610170 },
    GpsL2cCmCodeAssignment { prn: 53, initial_state_octal: 0o023127030, end_state_octal: 0o744312067 },
    GpsL2cCmCodeAssignment { prn: 54, initial_state_octal: 0o431343777, end_state_octal: 0o154235152 },
    GpsL2cCmCodeAssignment { prn: 55, initial_state_octal: 0o747317317, end_state_octal: 0o525024652 },
    GpsL2cCmCodeAssignment { prn: 56, initial_state_octal: 0o045706125, end_state_octal: 0o535207413 },
    GpsL2cCmCodeAssignment { prn: 57, initial_state_octal: 0o002744276, end_state_octal: 0o655375733 },
    GpsL2cCmCodeAssignment { prn: 58, initial_state_octal: 0o060036467, end_state_octal: 0o316666241 },
    GpsL2cCmCodeAssignment { prn: 59, initial_state_octal: 0o217744147, end_state_octal: 0o525453337 },
    GpsL2cCmCodeAssignment { prn: 60, initial_state_octal: 0o603340174, end_state_octal: 0o114323414 },
    GpsL2cCmCodeAssignment { prn: 61, initial_state_octal: 0o326616775, end_state_octal: 0o755234667 },
    GpsL2cCmCodeAssignment { prn: 62, initial_state_octal: 0o063240065, end_state_octal: 0o526032633 },
    GpsL2cCmCodeAssignment { prn: 63, initial_state_octal: 0o111460621, end_state_octal: 0o602375063 },
    GpsL2cCmCodeAssignment { prn: 159, initial_state_octal: 0o604055104, end_state_octal: 0o425373114 },
    GpsL2cCmCodeAssignment { prn: 160, initial_state_octal: 0o157065232, end_state_octal: 0o427153064 },
    GpsL2cCmCodeAssignment { prn: 161, initial_state_octal: 0o013305707, end_state_octal: 0o310366577 },
    GpsL2cCmCodeAssignment { prn: 162, initial_state_octal: 0o603552017, end_state_octal: 0o623710414 },
    GpsL2cCmCodeAssignment { prn: 163, initial_state_octal: 0o230461355, end_state_octal: 0o252761705 },
    GpsL2cCmCodeAssignment { prn: 164, initial_state_octal: 0o603653437, end_state_octal: 0o050174703 },
    GpsL2cCmCodeAssignment { prn: 165, initial_state_octal: 0o652346475, end_state_octal: 0o050301454 },
    GpsL2cCmCodeAssignment { prn: 166, initial_state_octal: 0o743107103, end_state_octal: 0o416652040 },
    GpsL2cCmCodeAssignment { prn: 167, initial_state_octal: 0o401521277, end_state_octal: 0o050301251 },
    GpsL2cCmCodeAssignment { prn: 168, initial_state_octal: 0o167335110, end_state_octal: 0o744136527 },
    GpsL2cCmCodeAssignment { prn: 169, initial_state_octal: 0o014013575, end_state_octal: 0o633772375 },
    GpsL2cCmCodeAssignment { prn: 170, initial_state_octal: 0o362051132, end_state_octal: 0o007131446 },
    GpsL2cCmCodeAssignment { prn: 171, initial_state_octal: 0o617753265, end_state_octal: 0o142007172 },
    GpsL2cCmCodeAssignment { prn: 172, initial_state_octal: 0o216363634, end_state_octal: 0o655543571 },
    GpsL2cCmCodeAssignment { prn: 173, initial_state_octal: 0o755561123, end_state_octal: 0o031272346 },
    GpsL2cCmCodeAssignment { prn: 174, initial_state_octal: 0o365304033, end_state_octal: 0o203260313 },
    GpsL2cCmCodeAssignment { prn: 175, initial_state_octal: 0o625025543, end_state_octal: 0o226613112 },
    GpsL2cCmCodeAssignment { prn: 176, initial_state_octal: 0o054420334, end_state_octal: 0o736560607 },
    GpsL2cCmCodeAssignment { prn: 177, initial_state_octal: 0o415473671, end_state_octal: 0o011741374 },
    GpsL2cCmCodeAssignment { prn: 178, initial_state_octal: 0o662364360, end_state_octal: 0o765056120 },
    GpsL2cCmCodeAssignment { prn: 179, initial_state_octal: 0o373446602, end_state_octal: 0o262725266 },
    GpsL2cCmCodeAssignment { prn: 180, initial_state_octal: 0o417564100, end_state_octal: 0o013051476 },
    GpsL2cCmCodeAssignment { prn: 181, initial_state_octal: 0o000526452, end_state_octal: 0o144541215 },
    GpsL2cCmCodeAssignment { prn: 182, initial_state_octal: 0o226631300, end_state_octal: 0o534125243 },
    GpsL2cCmCodeAssignment { prn: 183, initial_state_octal: 0o113752074, end_state_octal: 0o250001521 },
    GpsL2cCmCodeAssignment { prn: 184, initial_state_octal: 0o706134401, end_state_octal: 0o276000566 },
    GpsL2cCmCodeAssignment { prn: 185, initial_state_octal: 0o041352546, end_state_octal: 0o447447071 },
    GpsL2cCmCodeAssignment { prn: 186, initial_state_octal: 0o664630154, end_state_octal: 0o000202044 },
    GpsL2cCmCodeAssignment { prn: 187, initial_state_octal: 0o276524255, end_state_octal: 0o751430577 },
    GpsL2cCmCodeAssignment { prn: 188, initial_state_octal: 0o714720530, end_state_octal: 0o136741270 },
    GpsL2cCmCodeAssignment { prn: 189, initial_state_octal: 0o714051771, end_state_octal: 0o257252440 },
    GpsL2cCmCodeAssignment { prn: 190, initial_state_octal: 0o044526647, end_state_octal: 0o757666513 },
    GpsL2cCmCodeAssignment { prn: 191, initial_state_octal: 0o207164322, end_state_octal: 0o606512137 },
    GpsL2cCmCodeAssignment { prn: 192, initial_state_octal: 0o262120161, end_state_octal: 0o734247645 },
    GpsL2cCmCodeAssignment { prn: 193, initial_state_octal: 0o204244652, end_state_octal: 0o415505547 },
    GpsL2cCmCodeAssignment { prn: 194, initial_state_octal: 0o202133131, end_state_octal: 0o705146647 },
    GpsL2cCmCodeAssignment { prn: 195, initial_state_octal: 0o714351204, end_state_octal: 0o006215430 },
    GpsL2cCmCodeAssignment { prn: 196, initial_state_octal: 0o657127260, end_state_octal: 0o371216176 },
    GpsL2cCmCodeAssignment { prn: 197, initial_state_octal: 0o130567507, end_state_octal: 0o645502771 },
    GpsL2cCmCodeAssignment { prn: 198, initial_state_octal: 0o670517677, end_state_octal: 0o455175106 },
    GpsL2cCmCodeAssignment { prn: 199, initial_state_octal: 0o607275514, end_state_octal: 0o127161032 },
    GpsL2cCmCodeAssignment { prn: 200, initial_state_octal: 0o045413633, end_state_octal: 0o470332401 },
    GpsL2cCmCodeAssignment { prn: 201, initial_state_octal: 0o212645405, end_state_octal: 0o252026355 },
    GpsL2cCmCodeAssignment { prn: 202, initial_state_octal: 0o613700455, end_state_octal: 0o113771472 },
    GpsL2cCmCodeAssignment { prn: 203, initial_state_octal: 0o706202440, end_state_octal: 0o754447142 },
    GpsL2cCmCodeAssignment { prn: 204, initial_state_octal: 0o705056276, end_state_octal: 0o627405712 },
    GpsL2cCmCodeAssignment { prn: 205, initial_state_octal: 0o020373522, end_state_octal: 0o325721745 },
    GpsL2cCmCodeAssignment { prn: 206, initial_state_octal: 0o746013617, end_state_octal: 0o056714616 },
    GpsL2cCmCodeAssignment { prn: 207, initial_state_octal: 0o132720621, end_state_octal: 0o706035241 },
    GpsL2cCmCodeAssignment { prn: 208, initial_state_octal: 0o434015513, end_state_octal: 0o173076740 },
    GpsL2cCmCodeAssignment { prn: 209, initial_state_octal: 0o566721727, end_state_octal: 0o145721746 },
    GpsL2cCmCodeAssignment { prn: 210, initial_state_octal: 0o140633660, end_state_octal: 0o465052527 },
];

/// Return the published GPS L2C CM shift-register assignment for one PRN.
pub fn gps_l2c_cm_code_assignment(
    prn: u8,
) -> Result<&'static GpsL2cCmCodeAssignment, SignalError> {
    GPS_L2C_CM_CODE_ASSIGNMENTS
        .iter()
        .find(|assignment| assignment.prn == prn)
        .ok_or(SignalError::UnsupportedPrn(prn))
}

/// Return the published GPS L2C CM shift-register assignments for all supported PRNs.
pub fn gps_l2c_cm_code_assignments() -> &'static [GpsL2cCmCodeAssignment; 115] {
    &GPS_L2C_CM_CODE_ASSIGNMENTS
}

/// Generate one full-period GPS L2C CM ranging code for a given PRN.
///
/// Returns chips in `{-1, +1}`.
pub fn generate_gps_l2c_cm_code(prn: u8) -> Result<Vec<i8>, SignalError> {
    Ok(generate_bipolar_code_period(
        gps_l2c_cm_code_assignment(prn)?.initial_state_octal,
        GPS_L2C_CM_CODE_CHIPS,
    ))
}

/// Generate a GPS L2C CM ranging-code sequence of arbitrary length by repeating the
/// published 10,230-chip period.
pub fn generate_gps_l2c_cm_code_chips(prn: u8, chip_count: usize) -> Result<Vec<i8>, SignalError> {
    Ok(generate_bipolar_code_range(
        gps_l2c_cm_code_assignment(prn)?.initial_state_octal,
        GPS_L2C_CM_CODE_CHIPS,
        0,
        chip_count,
    ))
}

/// Sample the GPS L2C CM ranging code at an arbitrary sample rate from a chip-phase origin.
pub fn sample_gps_l2c_cm_code(
    prn: u8,
    sample_rate_hz: f64,
    start_chip_phase: f64,
    sample_count: usize,
) -> Result<Vec<f32>, SignalError> {
    let code = generate_gps_l2c_cm_code(prn)?;
    sample_code(&code, sample_rate_hz, GPS_L2C_CM_CODE_RATE_HZ, start_chip_phase, sample_count)
}

#[cfg(test)]
mod tests {
    use sha2::{Digest, Sha256};

    use super::{
        generate_gps_l2c_cm_code, generate_gps_l2c_cm_code_chips, gps_l2c_cm_code_assignment,
        gps_l2c_cm_code_assignments, sample_gps_l2c_cm_code, GpsL2cCmCodeAssignment,
        GPS_L2C_CM_CODE_CHIPS, GPS_L2C_CM_CODE_RATE_HZ,
    };
    use crate::codes::gps_l2c_register::{
        advance_register_state, register_state_from_octal, register_state_to_octal,
    };
    use crate::error::SignalError;

    #[test]
    fn gps_l2c_cm_assignments_cover_published_prn_ranges() {
        let assignments = gps_l2c_cm_code_assignments();
        let first = assignments.first().expect("first assignment");
        let last = assignments.last().expect("last assignment");

        assert_eq!(assignments.len(), 115);
        assert_eq!(first.prn, 1);
        assert_eq!(last.prn, 210);
        assert_eq!(
            assignments
                .iter()
                .map(|assignment| assignment.prn)
                .filter(|prn| (64..=158).contains(prn))
                .count(),
            0
        );
    }

    #[test]
    fn gps_l2c_cm_assignments_match_selected_official_table_rows() {
        assert_eq!(
            gps_l2c_cm_code_assignment(1),
            Ok(&GpsL2cCmCodeAssignment {
                prn: 1,
                initial_state_octal: 0o742417664,
                end_state_octal: 0o552566002,
            })
        );
        assert_eq!(
            gps_l2c_cm_code_assignment(20),
            Ok(&GpsL2cCmCodeAssignment {
                prn: 20,
                initial_state_octal: 0o120161274,
                end_state_octal: 0o365636111,
            })
        );
        assert_eq!(
            gps_l2c_cm_code_assignment(63),
            Ok(&GpsL2cCmCodeAssignment {
                prn: 63,
                initial_state_octal: 0o111460621,
                end_state_octal: 0o602375063,
            })
        );
        assert_eq!(
            gps_l2c_cm_code_assignment(37),
            Ok(&GpsL2cCmCodeAssignment {
                prn: 37,
                initial_state_octal: 0o435506112,
                end_state_octal: 0o133501670,
            })
        );
        assert_eq!(
            gps_l2c_cm_code_assignment(159),
            Ok(&GpsL2cCmCodeAssignment {
                prn: 159,
                initial_state_octal: 0o604055104,
                end_state_octal: 0o425373114,
            })
        );
        assert_eq!(
            gps_l2c_cm_code_assignment(210),
            Ok(&GpsL2cCmCodeAssignment {
                prn: 210,
                initial_state_octal: 0o140633660,
                end_state_octal: 0o465052527,
            })
        );
    }

    #[test]
    fn gps_l2c_cm_assignments_reject_unsupported_prns_and_the_unassigned_gap() {
        assert_eq!(gps_l2c_cm_code_assignment(0), Err(SignalError::UnsupportedPrn(0)));
        assert_eq!(gps_l2c_cm_code_assignment(64), Err(SignalError::UnsupportedPrn(64)));
        assert_eq!(gps_l2c_cm_code_assignment(158), Err(SignalError::UnsupportedPrn(158)));
        assert_eq!(gps_l2c_cm_code_assignment(211), Err(SignalError::UnsupportedPrn(211)));
    }

    #[test]
    fn gps_l2c_cm_generator_reaches_published_final_register_states() {
        for prn in [1_u8, 20, 37, 38, 63, 159, 210] {
            let assignment = gps_l2c_cm_code_assignment(prn).expect("published PRN");
            let mut state = register_state_from_octal(assignment.initial_state_octal);
            for _ in 0..(GPS_L2C_CM_CODE_CHIPS - 1) {
                advance_register_state(&mut state);
            }
            assert_eq!(register_state_to_octal(&state), assignment.end_state_octal, "prn={prn}");
        }
    }

    #[test]
    fn gps_l2c_cm_primary_code_is_bipolar_and_balanced() {
        let code = generate_gps_l2c_cm_code(38).expect("valid L2C CM PRN");
        let positive = code.iter().filter(|chip| **chip == 1).count();
        let negative = code.iter().filter(|chip| **chip == -1).count();

        assert_eq!(code.len(), GPS_L2C_CM_CODE_CHIPS);
        assert_eq!(positive, negative);
        assert!(code.iter().all(|chip| *chip == -1 || *chip == 1));
    }

    #[test]
    fn gps_l2c_cm_arbitrary_length_repeats_the_full_period() {
        let repeated =
            generate_gps_l2c_cm_code_chips(159, GPS_L2C_CM_CODE_CHIPS * 2).expect("valid L2C CM PRN");
        let (first_period, second_period) = repeated.split_at(GPS_L2C_CM_CODE_CHIPS);

        assert_eq!(first_period, second_period);
    }

    #[test]
    fn gps_l2c_cm_matches_reference_period_digests() {
        let references = [
            (1_u8, "7f7dcda35092a9ea6b3bccbe760c6df5dc3bd6e4298076958bf9ba4684d9fada"),
            (38_u8, "c9a746b27fd6ea8f5400bc5c697197e33e349d08292bfb18a6ffa633a7c46459"),
            (159_u8, "de2cbc17009d235043b1bac3183d25076a6f835cb0d6c7edb0f67f4dc15f3650"),
            (210_u8, "58ed7844ec12b7ef88b9e1801cd10fe9a29c3bf9b6e71334b4e174eb98f4442d"),
        ];

        for (prn, expected_digest) in references {
            let code = generate_gps_l2c_cm_code(prn).expect("valid L2C CM PRN");
            let digest = sha256_hex(&code.iter().map(|chip| *chip as u8).collect::<Vec<_>>());
            assert_eq!(digest, expected_digest, "prn={prn}");
        }
    }

    #[test]
    fn gps_l2c_cm_matches_reference_prefix_vectors() {
        let references = [
            (
                1_u8,
                "0010101111011110000111101011101000101001100001111100101100010010",
            ),
            (
                38_u8,
                "1100011000011100010111110000001100110000011000001000110011101101",
            ),
            (
                159_u8,
                "0010010100000001000111101101011010000101001010111100000100001001",
            ),
            (
                210_u8,
                "0000110011101100110000010011110010110100110000111101010101010001",
            ),
        ];

        for (prn, expected_prefix) in references {
            let code = generate_gps_l2c_cm_code(prn).expect("valid L2C CM PRN");
            let prefix: String = code
                .iter()
                .take(expected_prefix.len())
                .map(|chip| if *chip == -1 { '1' } else { '0' })
                .collect();
            assert_eq!(prefix, expected_prefix, "prn={prn}");
        }
    }

    #[test]
    fn gps_l2c_cm_sample_helper_tracks_chip_boundaries() {
        let samples = sample_gps_l2c_cm_code(1, GPS_L2C_CM_CODE_RATE_HZ, 0.0, 8)
            .expect("valid L2C CM PRN");
        let code = generate_gps_l2c_cm_code(1).expect("valid L2C CM PRN");

        assert_eq!(
            samples,
            code.iter().take(8).map(|chip| f32::from(*chip)).collect::<Vec<_>>()
        );
    }

    fn sha256_hex(bytes: &[u8]) -> String {
        let digest = Sha256::digest(bytes);
        let mut output = String::with_capacity(digest.len() * 2);
        for byte in digest {
            use std::fmt::Write as _;

            write!(&mut output, "{byte:02x}").expect("sha256 digest hex");
        }
        output
    }
}
