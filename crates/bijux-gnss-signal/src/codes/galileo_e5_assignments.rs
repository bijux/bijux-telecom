#![allow(missing_docs)]

//! Galileo E5 public-code assignments from the Galileo OS SIS ICD.

use crate::codes::galileo_e5_tables::{
    GALILEO_E5A_I_INITIAL_SEQUENCE_HEX, GALILEO_E5A_I_START_VALUES_OCTAL,
    GALILEO_E5A_Q_INITIAL_SEQUENCE_HEX, GALILEO_E5A_Q_SECONDARY_HEX,
    GALILEO_E5A_Q_START_VALUES_OCTAL, GALILEO_E5B_I_INITIAL_SEQUENCE_HEX,
    GALILEO_E5B_I_START_VALUES_OCTAL, GALILEO_E5B_Q_INITIAL_SEQUENCE_HEX,
    GALILEO_E5B_Q_SECONDARY_HEX, GALILEO_E5B_Q_START_VALUES_OCTAL,
};
use crate::error::SignalError;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GalileoE5aICodeAssignment {
    pub prn: u8,
    pub register_2_start_octal: &'static str,
    pub initial_sequence_hex: &'static str,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GalileoE5aQCodeAssignment {
    pub prn: u8,
    pub register_2_start_octal: &'static str,
    pub initial_sequence_hex: &'static str,
    pub secondary_code_hex: &'static str,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GalileoE5bICodeAssignment {
    pub prn: u8,
    pub register_2_start_octal: &'static str,
    pub initial_sequence_hex: &'static str,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GalileoE5bQCodeAssignment {
    pub prn: u8,
    pub register_2_start_octal: &'static str,
    pub initial_sequence_hex: &'static str,
    pub secondary_code_hex: &'static str,
}

pub const GALILEO_E5A_I_CODE_ASSIGNMENTS: [GalileoE5aICodeAssignment; 50] =
    build_e5a_i_code_assignments();
pub const GALILEO_E5A_Q_CODE_ASSIGNMENTS: [GalileoE5aQCodeAssignment; 50] =
    build_e5a_q_code_assignments();
pub const GALILEO_E5B_I_CODE_ASSIGNMENTS: [GalileoE5bICodeAssignment; 50] =
    build_e5b_i_code_assignments();
pub const GALILEO_E5B_Q_CODE_ASSIGNMENTS: [GalileoE5bQCodeAssignment; 50] =
    build_e5b_q_code_assignments();

const fn build_e5a_i_code_assignments() -> [GalileoE5aICodeAssignment; 50] {
    let mut assignments = [GalileoE5aICodeAssignment {
        prn: 1,
        register_2_start_octal: "",
        initial_sequence_hex: "",
    }; 50];
    let mut index = 0usize;
    while index < assignments.len() {
        assignments[index] = GalileoE5aICodeAssignment {
            prn: (index + 1) as u8,
            register_2_start_octal: GALILEO_E5A_I_START_VALUES_OCTAL[index],
            initial_sequence_hex: GALILEO_E5A_I_INITIAL_SEQUENCE_HEX[index],
        };
        index += 1;
    }
    assignments
}

const fn build_e5a_q_code_assignments() -> [GalileoE5aQCodeAssignment; 50] {
    let mut assignments = [GalileoE5aQCodeAssignment {
        prn: 1,
        register_2_start_octal: "",
        initial_sequence_hex: "",
        secondary_code_hex: "",
    }; 50];
    let mut index = 0usize;
    while index < assignments.len() {
        assignments[index] = GalileoE5aQCodeAssignment {
            prn: (index + 1) as u8,
            register_2_start_octal: GALILEO_E5A_Q_START_VALUES_OCTAL[index],
            initial_sequence_hex: GALILEO_E5A_Q_INITIAL_SEQUENCE_HEX[index],
            secondary_code_hex: GALILEO_E5A_Q_SECONDARY_HEX[index],
        };
        index += 1;
    }
    assignments
}

const fn build_e5b_i_code_assignments() -> [GalileoE5bICodeAssignment; 50] {
    let mut assignments = [GalileoE5bICodeAssignment {
        prn: 1,
        register_2_start_octal: "",
        initial_sequence_hex: "",
    }; 50];
    let mut index = 0usize;
    while index < assignments.len() {
        assignments[index] = GalileoE5bICodeAssignment {
            prn: (index + 1) as u8,
            register_2_start_octal: GALILEO_E5B_I_START_VALUES_OCTAL[index],
            initial_sequence_hex: GALILEO_E5B_I_INITIAL_SEQUENCE_HEX[index],
        };
        index += 1;
    }
    assignments
}

const fn build_e5b_q_code_assignments() -> [GalileoE5bQCodeAssignment; 50] {
    let mut assignments = [GalileoE5bQCodeAssignment {
        prn: 1,
        register_2_start_octal: "",
        initial_sequence_hex: "",
        secondary_code_hex: "",
    }; 50];
    let mut index = 0usize;
    while index < assignments.len() {
        assignments[index] = GalileoE5bQCodeAssignment {
            prn: (index + 1) as u8,
            register_2_start_octal: GALILEO_E5B_Q_START_VALUES_OCTAL[index],
            initial_sequence_hex: GALILEO_E5B_Q_INITIAL_SEQUENCE_HEX[index],
            secondary_code_hex: GALILEO_E5B_Q_SECONDARY_HEX[index],
        };
        index += 1;
    }
    assignments
}

pub fn galileo_e5a_i_code_assignment(
    prn: u8,
) -> Result<&'static GalileoE5aICodeAssignment, SignalError> {
    let index = e5_prn_index(prn)?;
    Ok(&GALILEO_E5A_I_CODE_ASSIGNMENTS[index])
}

pub fn galileo_e5a_q_code_assignment(
    prn: u8,
) -> Result<&'static GalileoE5aQCodeAssignment, SignalError> {
    let index = e5_prn_index(prn)?;
    Ok(&GALILEO_E5A_Q_CODE_ASSIGNMENTS[index])
}

pub fn galileo_e5a_i_code_assignments() -> &'static [GalileoE5aICodeAssignment; 50] {
    &GALILEO_E5A_I_CODE_ASSIGNMENTS
}

pub fn galileo_e5a_q_code_assignments() -> &'static [GalileoE5aQCodeAssignment; 50] {
    &GALILEO_E5A_Q_CODE_ASSIGNMENTS
}

pub fn galileo_e5b_i_code_assignment(
    prn: u8,
) -> Result<&'static GalileoE5bICodeAssignment, SignalError> {
    let index = e5_prn_index(prn)?;
    Ok(&GALILEO_E5B_I_CODE_ASSIGNMENTS[index])
}

pub fn galileo_e5b_q_code_assignment(
    prn: u8,
) -> Result<&'static GalileoE5bQCodeAssignment, SignalError> {
    let index = e5_prn_index(prn)?;
    Ok(&GALILEO_E5B_Q_CODE_ASSIGNMENTS[index])
}

pub fn galileo_e5b_i_code_assignments() -> &'static [GalileoE5bICodeAssignment; 50] {
    &GALILEO_E5B_I_CODE_ASSIGNMENTS
}

pub fn galileo_e5b_q_code_assignments() -> &'static [GalileoE5bQCodeAssignment; 50] {
    &GALILEO_E5B_Q_CODE_ASSIGNMENTS
}

fn e5_prn_index(prn: u8) -> Result<usize, SignalError> {
    if prn == 0 || prn > 50 {
        return Err(SignalError::UnsupportedPrn(prn));
    }
    Ok(usize::from(prn - 1))
}
