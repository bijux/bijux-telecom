#![allow(missing_docs)]

//! GPS L2C CL ranging-code assignments.
//!
//! Clean-room implementation derived from the public IS-GPS-200L signal definition.

use super::gps_l2c_register::{generate_bipolar_code_period, generate_bipolar_code_range};
use crate::dsp::signal::sample_code;
use crate::error::SignalError;

/// Number of chips in one published GPS L2C CL code period.
pub const GPS_L2C_CL_CODE_CHIPS: usize = 767_250;

/// Code rate of the GPS L2C CL ranging code.
pub const GPS_L2C_CL_CODE_RATE_HZ: f64 = 511_500.0;

/// Published shift-register assignment for one GPS L2C CL PRN.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GpsL2cClCodeAssignment {
    pub prn: u8,
    pub initial_state_octal: u32,
    pub end_state_octal: u32,
}

const GPS_L2C_CL_CODE_ASSIGNMENTS: [GpsL2cClCodeAssignment; 115] = [
    GpsL2cClCodeAssignment {
        prn: 1,
        initial_state_octal: 0o624145772,
        end_state_octal: 0o267724236,
    },
    GpsL2cClCodeAssignment {
        prn: 2,
        initial_state_octal: 0o506610362,
        end_state_octal: 0o167516066,
    },
    GpsL2cClCodeAssignment {
        prn: 3,
        initial_state_octal: 0o220360016,
        end_state_octal: 0o771756405,
    },
    GpsL2cClCodeAssignment {
        prn: 4,
        initial_state_octal: 0o710406104,
        end_state_octal: 0o047202624,
    },
    GpsL2cClCodeAssignment {
        prn: 5,
        initial_state_octal: 0o001143345,
        end_state_octal: 0o052770433,
    },
    GpsL2cClCodeAssignment {
        prn: 6,
        initial_state_octal: 0o053023326,
        end_state_octal: 0o761743665,
    },
    GpsL2cClCodeAssignment {
        prn: 7,
        initial_state_octal: 0o652521276,
        end_state_octal: 0o133015726,
    },
    GpsL2cClCodeAssignment {
        prn: 8,
        initial_state_octal: 0o206124777,
        end_state_octal: 0o610611511,
    },
    GpsL2cClCodeAssignment {
        prn: 9,
        initial_state_octal: 0o015563374,
        end_state_octal: 0o352150323,
    },
    GpsL2cClCodeAssignment {
        prn: 10,
        initial_state_octal: 0o561522076,
        end_state_octal: 0o051266046,
    },
    GpsL2cClCodeAssignment {
        prn: 11,
        initial_state_octal: 0o023163525,
        end_state_octal: 0o305611373,
    },
    GpsL2cClCodeAssignment {
        prn: 12,
        initial_state_octal: 0o117776450,
        end_state_octal: 0o504676773,
    },
    GpsL2cClCodeAssignment {
        prn: 13,
        initial_state_octal: 0o606516355,
        end_state_octal: 0o272572634,
    },
    GpsL2cClCodeAssignment {
        prn: 14,
        initial_state_octal: 0o003037343,
        end_state_octal: 0o731320771,
    },
    GpsL2cClCodeAssignment {
        prn: 15,
        initial_state_octal: 0o046515565,
        end_state_octal: 0o631326563,
    },
    GpsL2cClCodeAssignment {
        prn: 16,
        initial_state_octal: 0o671511621,
        end_state_octal: 0o231516360,
    },
    GpsL2cClCodeAssignment {
        prn: 17,
        initial_state_octal: 0o605402220,
        end_state_octal: 0o030367366,
    },
    GpsL2cClCodeAssignment {
        prn: 18,
        initial_state_octal: 0o002576207,
        end_state_octal: 0o713543613,
    },
    GpsL2cClCodeAssignment {
        prn: 19,
        initial_state_octal: 0o525163451,
        end_state_octal: 0o232674654,
    },
    GpsL2cClCodeAssignment {
        prn: 20,
        initial_state_octal: 0o266527765,
        end_state_octal: 0o641733155,
    },
    GpsL2cClCodeAssignment {
        prn: 21,
        initial_state_octal: 0o006760703,
        end_state_octal: 0o730125345,
    },
    GpsL2cClCodeAssignment {
        prn: 22,
        initial_state_octal: 0o501474556,
        end_state_octal: 0o000316074,
    },
    GpsL2cClCodeAssignment {
        prn: 23,
        initial_state_octal: 0o743747443,
        end_state_octal: 0o171313614,
    },
    GpsL2cClCodeAssignment {
        prn: 24,
        initial_state_octal: 0o615534726,
        end_state_octal: 0o001523662,
    },
    GpsL2cClCodeAssignment {
        prn: 25,
        initial_state_octal: 0o763621420,
        end_state_octal: 0o023457250,
    },
    GpsL2cClCodeAssignment {
        prn: 26,
        initial_state_octal: 0o720727474,
        end_state_octal: 0o330733254,
    },
    GpsL2cClCodeAssignment {
        prn: 27,
        initial_state_octal: 0o700521043,
        end_state_octal: 0o625055726,
    },
    GpsL2cClCodeAssignment {
        prn: 28,
        initial_state_octal: 0o222567263,
        end_state_octal: 0o476524061,
    },
    GpsL2cClCodeAssignment {
        prn: 29,
        initial_state_octal: 0o132765304,
        end_state_octal: 0o602066031,
    },
    GpsL2cClCodeAssignment {
        prn: 30,
        initial_state_octal: 0o746332245,
        end_state_octal: 0o012412526,
    },
    GpsL2cClCodeAssignment {
        prn: 31,
        initial_state_octal: 0o102300466,
        end_state_octal: 0o705144501,
    },
    GpsL2cClCodeAssignment {
        prn: 32,
        initial_state_octal: 0o255231716,
        end_state_octal: 0o615373171,
    },
    GpsL2cClCodeAssignment {
        prn: 33,
        initial_state_octal: 0o437661701,
        end_state_octal: 0o041637664,
    },
    GpsL2cClCodeAssignment {
        prn: 34,
        initial_state_octal: 0o717047302,
        end_state_octal: 0o100107264,
    },
    GpsL2cClCodeAssignment {
        prn: 35,
        initial_state_octal: 0o222614207,
        end_state_octal: 0o634251723,
    },
    GpsL2cClCodeAssignment {
        prn: 36,
        initial_state_octal: 0o561123307,
        end_state_octal: 0o257012032,
    },
    GpsL2cClCodeAssignment {
        prn: 37,
        initial_state_octal: 0o240713073,
        end_state_octal: 0o703702423,
    },
    GpsL2cClCodeAssignment {
        prn: 38,
        initial_state_octal: 0o101232630,
        end_state_octal: 0o463624741,
    },
    GpsL2cClCodeAssignment {
        prn: 39,
        initial_state_octal: 0o132525726,
        end_state_octal: 0o673421367,
    },
    GpsL2cClCodeAssignment {
        prn: 40,
        initial_state_octal: 0o315216367,
        end_state_octal: 0o703006075,
    },
    GpsL2cClCodeAssignment {
        prn: 41,
        initial_state_octal: 0o377046065,
        end_state_octal: 0o746566507,
    },
    GpsL2cClCodeAssignment {
        prn: 42,
        initial_state_octal: 0o655351360,
        end_state_octal: 0o444022714,
    },
    GpsL2cClCodeAssignment {
        prn: 43,
        initial_state_octal: 0o435776513,
        end_state_octal: 0o136645570,
    },
    GpsL2cClCodeAssignment {
        prn: 44,
        initial_state_octal: 0o744242321,
        end_state_octal: 0o645752300,
    },
    GpsL2cClCodeAssignment {
        prn: 45,
        initial_state_octal: 0o024346717,
        end_state_octal: 0o656113341,
    },
    GpsL2cClCodeAssignment {
        prn: 46,
        initial_state_octal: 0o562646415,
        end_state_octal: 0o015705106,
    },
    GpsL2cClCodeAssignment {
        prn: 47,
        initial_state_octal: 0o731455342,
        end_state_octal: 0o002757466,
    },
    GpsL2cClCodeAssignment {
        prn: 48,
        initial_state_octal: 0o723352536,
        end_state_octal: 0o100273370,
    },
    GpsL2cClCodeAssignment {
        prn: 49,
        initial_state_octal: 0o000013134,
        end_state_octal: 0o304463615,
    },
    GpsL2cClCodeAssignment {
        prn: 50,
        initial_state_octal: 0o011566642,
        end_state_octal: 0o054341657,
    },
    GpsL2cClCodeAssignment {
        prn: 51,
        initial_state_octal: 0o475432222,
        end_state_octal: 0o333276704,
    },
    GpsL2cClCodeAssignment {
        prn: 52,
        initial_state_octal: 0o463506741,
        end_state_octal: 0o750231416,
    },
    GpsL2cClCodeAssignment {
        prn: 53,
        initial_state_octal: 0o617127534,
        end_state_octal: 0o541445326,
    },
    GpsL2cClCodeAssignment {
        prn: 54,
        initial_state_octal: 0o026050332,
        end_state_octal: 0o316216573,
    },
    GpsL2cClCodeAssignment {
        prn: 55,
        initial_state_octal: 0o733774235,
        end_state_octal: 0o007360406,
    },
    GpsL2cClCodeAssignment {
        prn: 56,
        initial_state_octal: 0o751477772,
        end_state_octal: 0o112114774,
    },
    GpsL2cClCodeAssignment {
        prn: 57,
        initial_state_octal: 0o417631550,
        end_state_octal: 0o042303316,
    },
    GpsL2cClCodeAssignment {
        prn: 58,
        initial_state_octal: 0o052247456,
        end_state_octal: 0o353150521,
    },
    GpsL2cClCodeAssignment {
        prn: 59,
        initial_state_octal: 0o560404163,
        end_state_octal: 0o044511154,
    },
    GpsL2cClCodeAssignment {
        prn: 60,
        initial_state_octal: 0o417751005,
        end_state_octal: 0o244410144,
    },
    GpsL2cClCodeAssignment {
        prn: 61,
        initial_state_octal: 0o004302173,
        end_state_octal: 0o562324657,
    },
    GpsL2cClCodeAssignment {
        prn: 62,
        initial_state_octal: 0o715005045,
        end_state_octal: 0o027501534,
    },
    GpsL2cClCodeAssignment {
        prn: 63,
        initial_state_octal: 0o001154457,
        end_state_octal: 0o521240373,
    },
    GpsL2cClCodeAssignment {
        prn: 159,
        initial_state_octal: 0o605253024,
        end_state_octal: 0o044547544,
    },
    GpsL2cClCodeAssignment {
        prn: 160,
        initial_state_octal: 0o063314262,
        end_state_octal: 0o707116115,
    },
    GpsL2cClCodeAssignment {
        prn: 161,
        initial_state_octal: 0o066073422,
        end_state_octal: 0o412264037,
    },
    GpsL2cClCodeAssignment {
        prn: 162,
        initial_state_octal: 0o737276117,
        end_state_octal: 0o223755032,
    },
    GpsL2cClCodeAssignment {
        prn: 163,
        initial_state_octal: 0o737243704,
        end_state_octal: 0o403114174,
    },
    GpsL2cClCodeAssignment {
        prn: 164,
        initial_state_octal: 0o067557532,
        end_state_octal: 0o671505575,
    },
    GpsL2cClCodeAssignment {
        prn: 165,
        initial_state_octal: 0o227354537,
        end_state_octal: 0o606261015,
    },
    GpsL2cClCodeAssignment {
        prn: 166,
        initial_state_octal: 0o704765502,
        end_state_octal: 0o223023120,
    },
    GpsL2cClCodeAssignment {
        prn: 167,
        initial_state_octal: 0o044746712,
        end_state_octal: 0o370035547,
    },
    GpsL2cClCodeAssignment {
        prn: 168,
        initial_state_octal: 0o720535263,
        end_state_octal: 0o516101304,
    },
    GpsL2cClCodeAssignment {
        prn: 169,
        initial_state_octal: 0o733541364,
        end_state_octal: 0o044115766,
    },
    GpsL2cClCodeAssignment {
        prn: 170,
        initial_state_octal: 0o270060042,
        end_state_octal: 0o704125517,
    },
    GpsL2cClCodeAssignment {
        prn: 171,
        initial_state_octal: 0o737176640,
        end_state_octal: 0o406332330,
    },
    GpsL2cClCodeAssignment {
        prn: 172,
        initial_state_octal: 0o133776704,
        end_state_octal: 0o506446631,
    },
    GpsL2cClCodeAssignment {
        prn: 173,
        initial_state_octal: 0o005645427,
        end_state_octal: 0o743702511,
    },
    GpsL2cClCodeAssignment {
        prn: 174,
        initial_state_octal: 0o704321074,
        end_state_octal: 0o022623276,
    },
    GpsL2cClCodeAssignment {
        prn: 175,
        initial_state_octal: 0o137740372,
        end_state_octal: 0o704221045,
    },
    GpsL2cClCodeAssignment {
        prn: 176,
        initial_state_octal: 0o056375464,
        end_state_octal: 0o372577721,
    },
    GpsL2cClCodeAssignment {
        prn: 177,
        initial_state_octal: 0o704374004,
        end_state_octal: 0o105175230,
    },
    GpsL2cClCodeAssignment {
        prn: 178,
        initial_state_octal: 0o216320123,
        end_state_octal: 0o760701311,
    },
    GpsL2cClCodeAssignment {
        prn: 179,
        initial_state_octal: 0o011322115,
        end_state_octal: 0o737141001,
    },
    GpsL2cClCodeAssignment {
        prn: 180,
        initial_state_octal: 0o761050112,
        end_state_octal: 0o227627616,
    },
    GpsL2cClCodeAssignment {
        prn: 181,
        initial_state_octal: 0o725304036,
        end_state_octal: 0o245154134,
    },
    GpsL2cClCodeAssignment {
        prn: 182,
        initial_state_octal: 0o721320336,
        end_state_octal: 0o040015760,
    },
    GpsL2cClCodeAssignment {
        prn: 183,
        initial_state_octal: 0o443462103,
        end_state_octal: 0o002154472,
    },
    GpsL2cClCodeAssignment {
        prn: 184,
        initial_state_octal: 0o510466244,
        end_state_octal: 0o301767766,
    },
    GpsL2cClCodeAssignment {
        prn: 185,
        initial_state_octal: 0o745522652,
        end_state_octal: 0o226475246,
    },
    GpsL2cClCodeAssignment {
        prn: 186,
        initial_state_octal: 0o373417061,
        end_state_octal: 0o733673015,
    },
    GpsL2cClCodeAssignment {
        prn: 187,
        initial_state_octal: 0o225526762,
        end_state_octal: 0o602507667,
    },
    GpsL2cClCodeAssignment {
        prn: 188,
        initial_state_octal: 0o047614504,
        end_state_octal: 0o753362551,
    },
    GpsL2cClCodeAssignment {
        prn: 189,
        initial_state_octal: 0o034730440,
        end_state_octal: 0o746265601,
    },
    GpsL2cClCodeAssignment {
        prn: 190,
        initial_state_octal: 0o453073141,
        end_state_octal: 0o036253206,
    },
    GpsL2cClCodeAssignment {
        prn: 191,
        initial_state_octal: 0o533654510,
        end_state_octal: 0o202512772,
    },
    GpsL2cClCodeAssignment {
        prn: 192,
        initial_state_octal: 0o377016461,
        end_state_octal: 0o701234023,
    },
    GpsL2cClCodeAssignment {
        prn: 193,
        initial_state_octal: 0o235525312,
        end_state_octal: 0o722043377,
    },
    GpsL2cClCodeAssignment {
        prn: 194,
        initial_state_octal: 0o507056307,
        end_state_octal: 0o240751052,
    },
    GpsL2cClCodeAssignment {
        prn: 195,
        initial_state_octal: 0o221720061,
        end_state_octal: 0o375674043,
    },
    GpsL2cClCodeAssignment {
        prn: 196,
        initial_state_octal: 0o520470122,
        end_state_octal: 0o166677056,
    },
    GpsL2cClCodeAssignment {
        prn: 197,
        initial_state_octal: 0o603764120,
        end_state_octal: 0o123055362,
    },
    GpsL2cClCodeAssignment {
        prn: 198,
        initial_state_octal: 0o145604016,
        end_state_octal: 0o707017665,
    },
    GpsL2cClCodeAssignment {
        prn: 199,
        initial_state_octal: 0o051237167,
        end_state_octal: 0o437503241,
    },
    GpsL2cClCodeAssignment {
        prn: 200,
        initial_state_octal: 0o033326347,
        end_state_octal: 0o275605155,
    },
    GpsL2cClCodeAssignment {
        prn: 201,
        initial_state_octal: 0o534627074,
        end_state_octal: 0o376333266,
    },
    GpsL2cClCodeAssignment {
        prn: 202,
        initial_state_octal: 0o645230164,
        end_state_octal: 0o467523556,
    },
    GpsL2cClCodeAssignment {
        prn: 203,
        initial_state_octal: 0o000171400,
        end_state_octal: 0o144132537,
    },
    GpsL2cClCodeAssignment {
        prn: 204,
        initial_state_octal: 0o022715417,
        end_state_octal: 0o451024205,
    },
    GpsL2cClCodeAssignment {
        prn: 205,
        initial_state_octal: 0o135471311,
        end_state_octal: 0o722446427,
    },
    GpsL2cClCodeAssignment {
        prn: 206,
        initial_state_octal: 0o137422057,
        end_state_octal: 0o412376261,
    },
    GpsL2cClCodeAssignment {
        prn: 207,
        initial_state_octal: 0o714426456,
        end_state_octal: 0o441570172,
    },
    GpsL2cClCodeAssignment {
        prn: 208,
        initial_state_octal: 0o640724672,
        end_state_octal: 0o063217710,
    },
    GpsL2cClCodeAssignment {
        prn: 209,
        initial_state_octal: 0o501254540,
        end_state_octal: 0o110320656,
    },
    GpsL2cClCodeAssignment {
        prn: 210,
        initial_state_octal: 0o513322453,
        end_state_octal: 0o113765506,
    },
];

/// Return the published GPS L2C CL shift-register assignment for one PRN.
pub fn gps_l2c_cl_code_assignment(prn: u8) -> Result<&'static GpsL2cClCodeAssignment, SignalError> {
    GPS_L2C_CL_CODE_ASSIGNMENTS
        .iter()
        .find(|assignment| assignment.prn == prn)
        .ok_or(SignalError::UnsupportedPrn(prn))
}

/// Return the published GPS L2C CL shift-register assignments for all supported PRNs.
pub fn gps_l2c_cl_code_assignments() -> &'static [GpsL2cClCodeAssignment; 115] {
    &GPS_L2C_CL_CODE_ASSIGNMENTS
}

/// Generate one full-period GPS L2C CL ranging code for a given PRN.
///
/// Returns chips in `{-1, +1}`.
pub fn generate_gps_l2c_cl_code(prn: u8) -> Result<Vec<i8>, SignalError> {
    Ok(generate_bipolar_code_period(
        gps_l2c_cl_code_assignment(prn)?.initial_state_octal,
        GPS_L2C_CL_CODE_CHIPS,
    ))
}

/// Generate a contiguous range of GPS L2C CL ranging-code chips.
///
/// The start offset wraps within the published 767,250-chip code period.
pub fn generate_gps_l2c_cl_code_range(
    prn: u8,
    start_chip: usize,
    chip_count: usize,
) -> Result<Vec<i8>, SignalError> {
    Ok(generate_bipolar_code_range(
        gps_l2c_cl_code_assignment(prn)?.initial_state_octal,
        GPS_L2C_CL_CODE_CHIPS,
        start_chip,
        chip_count,
    ))
}

/// Sample the GPS L2C CL ranging code at an arbitrary sample rate from a chip-phase origin.
pub fn sample_gps_l2c_cl_code(
    prn: u8,
    sample_rate_hz: f64,
    start_chip_phase: f64,
    sample_count: usize,
) -> Result<Vec<f32>, SignalError> {
    let code = generate_gps_l2c_cl_code(prn)?;
    sample_code(&code, sample_rate_hz, GPS_L2C_CL_CODE_RATE_HZ, start_chip_phase, sample_count)
}

#[cfg(test)]
mod tests {
    use super::{
        generate_gps_l2c_cl_code, generate_gps_l2c_cl_code_range, gps_l2c_cl_code_assignment,
        gps_l2c_cl_code_assignments, sample_gps_l2c_cl_code, GpsL2cClCodeAssignment,
        GPS_L2C_CL_CODE_CHIPS, GPS_L2C_CL_CODE_RATE_HZ,
    };
    use crate::codes::gps_l2c_register::{
        advance_register_state, register_state_from_octal, register_state_to_octal,
    };
    use crate::error::SignalError;

    #[test]
    fn gps_l2c_cl_assignments_cover_published_prn_ranges() {
        let assignments = gps_l2c_cl_code_assignments();
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
        assert_eq!(GPS_L2C_CL_CODE_CHIPS, 767_250);
        assert_eq!(GPS_L2C_CL_CODE_RATE_HZ, 511_500.0);
    }

    #[test]
    fn gps_l2c_cl_assignments_match_selected_official_table_rows() {
        assert_eq!(
            gps_l2c_cl_code_assignment(1),
            Ok(&GpsL2cClCodeAssignment {
                prn: 1,
                initial_state_octal: 0o624145772,
                end_state_octal: 0o267724236,
            })
        );
        assert_eq!(
            gps_l2c_cl_code_assignment(37),
            Ok(&GpsL2cClCodeAssignment {
                prn: 37,
                initial_state_octal: 0o240713073,
                end_state_octal: 0o703702423,
            })
        );
        assert_eq!(
            gps_l2c_cl_code_assignment(38),
            Ok(&GpsL2cClCodeAssignment {
                prn: 38,
                initial_state_octal: 0o101232630,
                end_state_octal: 0o463624741,
            })
        );
        assert_eq!(
            gps_l2c_cl_code_assignment(159),
            Ok(&GpsL2cClCodeAssignment {
                prn: 159,
                initial_state_octal: 0o605253024,
                end_state_octal: 0o044547544,
            })
        );
        assert_eq!(
            gps_l2c_cl_code_assignment(210),
            Ok(&GpsL2cClCodeAssignment {
                prn: 210,
                initial_state_octal: 0o513322453,
                end_state_octal: 0o113765506,
            })
        );
    }

    #[test]
    fn gps_l2c_cl_assignments_reject_unsupported_prns_and_the_unassigned_gap() {
        assert_eq!(gps_l2c_cl_code_assignment(0), Err(SignalError::UnsupportedPrn(0)));
        assert_eq!(gps_l2c_cl_code_assignment(64), Err(SignalError::UnsupportedPrn(64)));
        assert_eq!(gps_l2c_cl_code_assignment(158), Err(SignalError::UnsupportedPrn(158)));
        assert_eq!(gps_l2c_cl_code_assignment(211), Err(SignalError::UnsupportedPrn(211)));
    }

    #[test]
    fn gps_l2c_cl_generator_reaches_published_final_register_states() {
        for prn in [1_u8, 37, 38, 63, 159, 210] {
            let assignment = gps_l2c_cl_code_assignment(prn).expect("published PRN");
            let mut state = register_state_from_octal(assignment.initial_state_octal);
            for _ in 0..(GPS_L2C_CL_CODE_CHIPS - 1) {
                advance_register_state(&mut state);
            }
            assert_eq!(register_state_to_octal(&state), assignment.end_state_octal, "prn={prn}");
        }
    }

    #[test]
    fn gps_l2c_cl_generator_emits_full_published_period() {
        let code = generate_gps_l2c_cl_code(38).expect("valid L2C CL PRN");

        assert_eq!(code.len(), GPS_L2C_CL_CODE_CHIPS);
        assert!(code.iter().all(|chip| *chip == -1 || *chip == 1));
    }

    #[test]
    fn gps_l2c_cl_arbitrary_ranges_wrap_the_published_period() {
        let period = generate_gps_l2c_cl_code(159).expect("valid L2C CL PRN");
        let wrapped = generate_gps_l2c_cl_code_range(159, GPS_L2C_CL_CODE_CHIPS - 7, 16)
            .expect("valid L2C CL PRN");

        let mut expected = period[GPS_L2C_CL_CODE_CHIPS - 7..].to_vec();
        expected.extend_from_slice(&period[..9]);

        assert_eq!(wrapped, expected);
    }

    #[test]
    fn gps_l2c_cl_sample_helper_tracks_chip_boundaries() {
        let samples =
            sample_gps_l2c_cl_code(1, GPS_L2C_CL_CODE_RATE_HZ, 0.0, 8).expect("valid L2C CL PRN");
        let code = generate_gps_l2c_cl_code_range(1, 0, 8).expect("valid L2C CL PRN");

        assert_eq!(samples, code.iter().map(|chip| f32::from(*chip)).collect::<Vec<_>>());
    }
}
