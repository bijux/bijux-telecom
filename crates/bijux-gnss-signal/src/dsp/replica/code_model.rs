use crate::catalog::default_acquisition_signal;
use crate::codes::beidou_b1i::{generate_beidou_b1i_code, BEIDOU_B1I_CODE_RATE_HZ};
use crate::codes::beidou_b2i::{generate_beidou_b2i_code, BEIDOU_B2I_CODE_RATE_HZ};
use crate::codes::beidou_d1::beidou_d1_epoch_symbol;
use crate::codes::ca_code::{generate_ca_code, Prn};
use crate::codes::galileo_e1::{
    galileo_e1_cboc_value, generate_galileo_e1b_code, generate_galileo_e1c_code,
    GALILEO_E1_CODE_RATE_HZ,
};
use crate::codes::galileo_e5::{
    galileo_e5a_i_value, galileo_e5a_q_secondary_code, galileo_e5a_qpsk_value, galileo_e5b_i_value,
    galileo_e5b_q_secondary_code, galileo_e5b_qpsk_value, generate_galileo_e5a_i_code,
    generate_galileo_e5a_q_code, generate_galileo_e5b_i_code, generate_galileo_e5b_q_code,
    GALILEO_E5A_CODE_RATE_HZ, GALILEO_E5A_PRIMARY_CODE_CHIPS, GALILEO_E5A_Q_SECONDARY_CODE_CHIPS,
    GALILEO_E5B_CODE_RATE_HZ, GALILEO_E5B_PRIMARY_CODE_CHIPS, GALILEO_E5B_Q_SECONDARY_CODE_CHIPS,
};
use crate::codes::glonass_l1::{generate_glonass_l1_st_code, GLONASS_L1_ST_CODE_RATE_HZ};
use crate::codes::gps_l2c::{
    gps_l2c_time_multiplexed_value, GPS_L2C_TIME_MULTIPLEXED_CODE_CHIPS,
    GPS_L2C_TIME_MULTIPLEXED_CODE_RATE_HZ,
};
use crate::codes::gps_l2c_cl::generate_gps_l2c_cl_code;
use crate::codes::gps_l2c_cm::generate_gps_l2c_cm_code;
use crate::codes::gps_l5::{
    generate_gps_l5_i_code, generate_gps_l5_q_code, gps_l5_i_epoch_symbol, gps_l5_q_epoch_symbol,
    GPS_L5_PRIMARY_CODE_CHIPS, GPS_L5_PRIMARY_CODE_RATE_HZ,
};
use crate::dsp::signal::code_value_at_phase;
use crate::error::SignalError;
use bijux_gnss_core::api::{Constellation, SatId, SignalBand, SignalCode};
use num_complex::Complex;

use super::{
    default_signal_code_for_band, sample_modulated_replica_at_sample_index, ReplicaBlockRequest,
    ReplicaSampleIndexRequest,
};

/// Reusable spreading-code models for synthesized signal replicas.
#[derive(Debug, Clone, PartialEq)]
pub enum ReplicaCodeModel {
    /// GPS L1 C/A primary code.
    GpsL1Ca { code: Vec<i8> },
    /// GPS L2C time-multiplexed CM/CL composite code.
    GpsL2cTimeMultiplexed { cm_code: Vec<i8>, cl_code: Vec<i8> },
    /// GPS L5-I primary code with Neumann-Hoffman data modulation.
    GpsL5I { code: Vec<i8> },
    /// GPS L5-Q primary code with pilot Neumann-Hoffman modulation.
    GpsL5Q { code: Vec<i8> },
    /// Galileo E1 CBOC composite code.
    GalileoE1Cboc { e1b_code: Vec<i8>, e1c_code: Vec<i8> },
    /// Galileo E5a-I primary code with tiered secondary modulation.
    GalileoE5aI { code: Vec<i8> },
    /// Galileo E5a narrowband QPSK composite code.
    GalileoE5aQpsk {
        e5ai_code: Vec<i8>,
        e5aq_code: Vec<i8>,
        e5aq_secondary_code: [i8; GALILEO_E5A_Q_SECONDARY_CODE_CHIPS],
    },
    /// Galileo E5b-I primary code with I/NAV tiered secondary modulation.
    GalileoE5bI { code: Vec<i8> },
    /// Galileo E5b narrowband QPSK composite code.
    GalileoE5bQpsk {
        e5bi_code: Vec<i8>,
        e5bq_code: Vec<i8>,
        e5bq_secondary_code: [i8; GALILEO_E5B_Q_SECONDARY_CODE_CHIPS],
    },
    /// BeiDou B1I primary code.
    BeidouB1I { code: Vec<i8> },
    /// BeiDou B2I primary code.
    BeidouB2I { code: Vec<i8> },
    /// GLONASS L1 ST primary code.
    GlonassL1St { code: Vec<i8> },
}

impl ReplicaCodeModel {
    /// Build the default synthesized replica for a supported satellite signal.
    pub fn from_default_signal(sat: SatId) -> Result<Option<Self>, SignalError> {
        match default_acquisition_signal(sat.constellation) {
            Some(signal) => Self::for_sat_signal(sat, Some(signal.spec.band), signal.spec.code),
            None => Ok(None),
        }
    }

    /// Build the synthesized replica for one explicit satellite signal band.
    pub fn for_sat_signal_band(
        sat: SatId,
        signal_band: Option<SignalBand>,
    ) -> Result<Option<Self>, SignalError> {
        let Some(signal_band) = signal_band else {
            return Ok(None);
        };
        Self::for_sat_signal(
            sat,
            Some(signal_band),
            default_signal_code_for_band(sat.constellation, signal_band),
        )
    }

    /// Build the synthesized replica for one explicit signal identity.
    pub fn for_sat_signal(
        sat: SatId,
        signal_band: Option<SignalBand>,
        signal_code: SignalCode,
    ) -> Result<Option<Self>, SignalError> {
        let Some(signal_band) = signal_band else {
            return Ok(None);
        };

        match (sat.constellation, signal_band, signal_code) {
            (Constellation::Gps, SignalBand::L1, SignalCode::Ca) => {
                Self::gps_l1_ca(sat.prn).map(Some)
            }
            (Constellation::Gps, SignalBand::L5, _) => match signal_code {
                SignalCode::L5I => Self::gps_l5_i(sat.prn).map(Some),
                SignalCode::L5Q => Self::gps_l5_q(sat.prn).map(Some),
                _ => Ok(None),
            },
            (Constellation::Galileo, SignalBand::E1, SignalCode::E1B) => {
                Self::galileo_e1_cboc(sat.prn).map(Some)
            }
            (Constellation::Galileo, SignalBand::E5, SignalCode::E5a) => {
                Self::galileo_e5a(sat.prn).map(Some)
            }
            (Constellation::Galileo, SignalBand::E5, SignalCode::E5b) => {
                Self::galileo_e5b(sat.prn).map(Some)
            }
            (Constellation::Beidou, SignalBand::B1, SignalCode::B1I) => {
                Self::beidou_b1i(sat.prn).map(Some)
            }
            (Constellation::Beidou, SignalBand::B2, SignalCode::B2I) => {
                Self::beidou_b2i(sat.prn).map(Some)
            }
            (Constellation::Glonass, SignalBand::L1, SignalCode::Unknown) => {
                Ok(Some(Self::glonass_l1_st()))
            }
            _ => Ok(None),
        }
    }

    /// Build the default synthesized replica, falling back to an all-ones GPS C/A code.
    pub fn from_default_signal_or_ones(sat: SatId) -> Self {
        Self::from_default_signal(sat)
            .ok()
            .flatten()
            .unwrap_or_else(|| Self::gps_l1_ca_or_ones(sat.prn))
    }

    /// Build a GPS L1 C/A replica from a PRN.
    pub fn gps_l1_ca(prn: u8) -> Result<Self, SignalError> {
        Ok(Self::GpsL1Ca { code: generate_ca_code(Prn(prn))? })
    }

    /// Build a GPS L1 C/A replica, falling back to an all-ones code when invalid.
    pub fn gps_l1_ca_or_ones(prn: u8) -> Self {
        Self::gps_l1_ca(prn).unwrap_or_else(|_| Self::GpsL1Ca { code: vec![1; 1023] })
    }

    /// Build a GPS L2C time-multiplexed replica from one PRN.
    pub fn gps_l2c_time_multiplexed(prn: u8) -> Result<Self, SignalError> {
        Ok(Self::GpsL2cTimeMultiplexed {
            cm_code: generate_gps_l2c_cm_code(prn)?,
            cl_code: generate_gps_l2c_cl_code(prn)?,
        })
    }

    /// Build a GPS L2C time-multiplexed replica, falling back to all-ones codes when invalid.
    pub fn gps_l2c_time_multiplexed_or_ones(prn: u8) -> Self {
        Self::gps_l2c_time_multiplexed(prn).unwrap_or_else(|_| Self::GpsL2cTimeMultiplexed {
            cm_code: vec![1; 10_230],
            cl_code: vec![1; 767_250],
        })
    }

    /// Build a GPS L5-I replica from a PRN.
    pub fn gps_l5_i(prn: u8) -> Result<Self, SignalError> {
        Ok(Self::GpsL5I { code: generate_gps_l5_i_code(prn)? })
    }

    /// Build a GPS L5-I replica, falling back to an all-ones code when invalid.
    pub fn gps_l5_i_or_ones(prn: u8) -> Self {
        Self::gps_l5_i(prn)
            .unwrap_or_else(|_| Self::GpsL5I { code: vec![1; GPS_L5_PRIMARY_CODE_CHIPS] })
    }

    /// Build a GPS L5-Q replica from a PRN.
    pub fn gps_l5_q(prn: u8) -> Result<Self, SignalError> {
        Ok(Self::GpsL5Q { code: generate_gps_l5_q_code(prn)? })
    }

    /// Build a GPS L5-Q replica, falling back to an all-ones code when invalid.
    pub fn gps_l5_q_or_ones(prn: u8) -> Self {
        Self::gps_l5_q(prn)
            .unwrap_or_else(|_| Self::GpsL5Q { code: vec![1; GPS_L5_PRIMARY_CODE_CHIPS] })
    }

    /// Build a Galileo E1 CBOC replica from a PRN.
    pub fn galileo_e1_cboc(prn: u8) -> Result<Self, SignalError> {
        Ok(Self::GalileoE1Cboc {
            e1b_code: generate_galileo_e1b_code(prn)?,
            e1c_code: generate_galileo_e1c_code(prn)?,
        })
    }

    /// Build a Galileo E1 CBOC replica, falling back to all-ones codes when invalid.
    pub fn galileo_e1_cboc_or_ones(prn: u8) -> Self {
        Self::galileo_e1_cboc(prn).unwrap_or_else(|_| Self::GalileoE1Cboc {
            e1b_code: vec![1; 4092],
            e1c_code: vec![1; 4092],
        })
    }

    /// Build a Galileo E5a-I replica from a PRN.
    pub fn galileo_e5a(prn: u8) -> Result<Self, SignalError> {
        Ok(Self::GalileoE5aI { code: generate_galileo_e5a_i_code(prn)? })
    }

    /// Build a Galileo E5a-I replica, falling back to an all-ones code when invalid.
    pub fn galileo_e5a_or_ones(prn: u8) -> Self {
        Self::galileo_e5a(prn)
            .unwrap_or_else(|_| Self::GalileoE5aI { code: vec![1; GALILEO_E5A_PRIMARY_CODE_CHIPS] })
    }

    /// Build a Galileo E5a QPSK composite replica from a PRN.
    pub fn galileo_e5a_qpsk(prn: u8) -> Result<Self, SignalError> {
        Ok(Self::GalileoE5aQpsk {
            e5ai_code: generate_galileo_e5a_i_code(prn)?,
            e5aq_code: generate_galileo_e5a_q_code(prn)?,
            e5aq_secondary_code: galileo_e5a_q_secondary_code(prn)?,
        })
    }

    /// Build a Galileo E5a QPSK replica, falling back to all-ones component codes when invalid.
    pub fn galileo_e5a_qpsk_or_ones(prn: u8) -> Self {
        Self::galileo_e5a_qpsk(prn).unwrap_or_else(|_| Self::GalileoE5aQpsk {
            e5ai_code: vec![1; GALILEO_E5A_PRIMARY_CODE_CHIPS],
            e5aq_code: vec![1; GALILEO_E5A_PRIMARY_CODE_CHIPS],
            e5aq_secondary_code: [1; GALILEO_E5A_Q_SECONDARY_CODE_CHIPS],
        })
    }

    /// Build a Galileo E5b-I replica from a PRN.
    pub fn galileo_e5b(prn: u8) -> Result<Self, SignalError> {
        Ok(Self::GalileoE5bI { code: generate_galileo_e5b_i_code(prn)? })
    }

    /// Build a Galileo E5b-I replica, falling back to an all-ones code when invalid.
    pub fn galileo_e5b_or_ones(prn: u8) -> Self {
        Self::galileo_e5b(prn)
            .unwrap_or_else(|_| Self::GalileoE5bI { code: vec![1; GALILEO_E5B_PRIMARY_CODE_CHIPS] })
    }

    /// Build a Galileo E5b QPSK composite replica from a PRN.
    pub fn galileo_e5b_qpsk(prn: u8) -> Result<Self, SignalError> {
        Ok(Self::GalileoE5bQpsk {
            e5bi_code: generate_galileo_e5b_i_code(prn)?,
            e5bq_code: generate_galileo_e5b_q_code(prn)?,
            e5bq_secondary_code: galileo_e5b_q_secondary_code(prn)?,
        })
    }

    /// Build a Galileo E5b QPSK replica, falling back to all-ones component codes when invalid.
    pub fn galileo_e5b_qpsk_or_ones(prn: u8) -> Self {
        Self::galileo_e5b_qpsk(prn).unwrap_or_else(|_| Self::GalileoE5bQpsk {
            e5bi_code: vec![1; GALILEO_E5B_PRIMARY_CODE_CHIPS],
            e5bq_code: vec![1; GALILEO_E5B_PRIMARY_CODE_CHIPS],
            e5bq_secondary_code: [1; GALILEO_E5B_Q_SECONDARY_CODE_CHIPS],
        })
    }

    /// Build a BeiDou B1I replica from a PRN.
    pub fn beidou_b1i(prn: u8) -> Result<Self, SignalError> {
        Ok(Self::BeidouB1I { code: generate_beidou_b1i_code(prn)? })
    }

    /// Build a BeiDou B1I replica, falling back to an all-ones code when invalid.
    pub fn beidou_b1i_or_ones(prn: u8) -> Self {
        Self::beidou_b1i(prn).unwrap_or_else(|_| Self::BeidouB1I { code: vec![1; 2046] })
    }

    /// Build a BeiDou B2I replica from a PRN.
    pub fn beidou_b2i(prn: u8) -> Result<Self, SignalError> {
        Ok(Self::BeidouB2I { code: generate_beidou_b2i_code(prn)? })
    }

    /// Build a BeiDou B2I replica, falling back to an all-ones code when invalid.
    pub fn beidou_b2i_or_ones(prn: u8) -> Self {
        Self::beidou_b2i(prn).unwrap_or_else(|_| Self::BeidouB2I { code: vec![1; 2046] })
    }

    /// Build a GLONASS L1 ST replica.
    pub fn glonass_l1_st() -> Self {
        Self::GlonassL1St { code: generate_glonass_l1_st_code() }
    }

    /// Return the code rate in chips per second.
    pub fn code_rate_hz(&self) -> f64 {
        match self {
            Self::GpsL1Ca { .. } => 1_023_000.0,
            Self::GpsL2cTimeMultiplexed { .. } => GPS_L2C_TIME_MULTIPLEXED_CODE_RATE_HZ,
            Self::GpsL5I { .. } => GPS_L5_PRIMARY_CODE_RATE_HZ,
            Self::GpsL5Q { .. } => GPS_L5_PRIMARY_CODE_RATE_HZ,
            Self::GalileoE1Cboc { .. } => GALILEO_E1_CODE_RATE_HZ,
            Self::GalileoE5aI { .. } => GALILEO_E5A_CODE_RATE_HZ,
            Self::GalileoE5aQpsk { .. } => GALILEO_E5A_CODE_RATE_HZ,
            Self::GalileoE5bI { .. } => GALILEO_E5B_CODE_RATE_HZ,
            Self::GalileoE5bQpsk { .. } => GALILEO_E5B_CODE_RATE_HZ,
            Self::BeidouB1I { .. } => BEIDOU_B1I_CODE_RATE_HZ,
            Self::BeidouB2I { .. } => BEIDOU_B2I_CODE_RATE_HZ,
            Self::GlonassL1St { .. } => GLONASS_L1_ST_CODE_RATE_HZ,
        }
    }

    /// Return the primary code period length in chips.
    pub fn code_length(&self) -> usize {
        match self {
            Self::GpsL1Ca { code } => code.len(),
            Self::GpsL2cTimeMultiplexed { .. } => GPS_L2C_TIME_MULTIPLEXED_CODE_CHIPS,
            Self::GpsL5I { code } => code.len(),
            Self::GpsL5Q { code } => code.len(),
            Self::GalileoE1Cboc { e1b_code, .. } => e1b_code.len(),
            Self::GalileoE5aI { code } => code.len(),
            Self::GalileoE5aQpsk { e5ai_code, .. } => e5ai_code.len(),
            Self::GalileoE5bI { code } => code.len(),
            Self::GalileoE5bQpsk { e5bi_code, .. } => e5bi_code.len(),
            Self::BeidouB1I { code } => code.len(),
            Self::BeidouB2I { code } => code.len(),
            Self::GlonassL1St { code } => code.len(),
        }
    }

    /// Sample the replica code/modulation value at a chip phase and primary-period index.
    pub fn sample_value(
        &self,
        chip_phase: f64,
        primary_code_period_index: usize,
        data_bit: i8,
    ) -> Result<Complex<f32>, SignalError> {
        match self {
            Self::GpsL1Ca { code } => {
                Ok(Complex::new(code_value_at_phase(code, chip_phase)? * data_bit as f32, 0.0))
            }
            Self::GpsL2cTimeMultiplexed { cm_code, cl_code } => Ok(Complex::new(
                gps_l2c_time_multiplexed_value(cm_code, cl_code, chip_phase, &[data_bit])?,
                0.0,
            )),
            Self::GpsL5I { code } => Ok(Complex::new(
                code_value_at_phase(code, chip_phase)?
                    * gps_l5_i_epoch_symbol(&[data_bit], primary_code_period_index)? as f32
                    * std::f32::consts::FRAC_1_SQRT_2,
                0.0,
            )),
            Self::GpsL5Q { code } => Ok(Complex::new(
                code_value_at_phase(code, chip_phase)?
                    * gps_l5_q_epoch_symbol(primary_code_period_index) as f32
                    * std::f32::consts::FRAC_1_SQRT_2,
                0.0,
            )),
            Self::GalileoE1Cboc { e1b_code, e1c_code } => Ok(Complex::new(
                galileo_e1_cboc_value(
                    e1b_code,
                    e1c_code,
                    chip_phase,
                    primary_code_period_index,
                    data_bit,
                )?,
                0.0,
            )),
            Self::GalileoE5aI { code } => Ok(Complex::new(
                galileo_e5a_i_value(code, chip_phase, primary_code_period_index, &[data_bit])?,
                0.0,
            )),
            Self::GalileoE5aQpsk { e5ai_code, e5aq_code, e5aq_secondary_code } => {
                galileo_e5a_qpsk_value(
                    e5ai_code,
                    e5aq_code,
                    e5aq_secondary_code,
                    chip_phase,
                    primary_code_period_index,
                    &[data_bit],
                )
            }
            Self::GalileoE5bI { code } => Ok(Complex::new(
                galileo_e5b_i_value(code, chip_phase, primary_code_period_index, &[data_bit])?,
                0.0,
            )),
            Self::GalileoE5bQpsk { e5bi_code, e5bq_code, e5bq_secondary_code } => {
                galileo_e5b_qpsk_value(
                    e5bi_code,
                    e5bq_code,
                    e5bq_secondary_code,
                    chip_phase,
                    primary_code_period_index,
                    &[data_bit],
                )
            }
            Self::BeidouB1I { code } | Self::BeidouB2I { code } => Ok(Complex::new(
                code_value_at_phase(code, chip_phase)?
                    * beidou_d1_epoch_symbol(&[data_bit], primary_code_period_index)? as f32,
                0.0,
            )),
            Self::GlonassL1St { code } => {
                Ok(Complex::new(code_value_at_phase(code, chip_phase)? * data_bit as f32, 0.0))
            }
        }
    }

    /// Sample a replica block from an absolute sample origin without chunk-boundary drift.
    pub fn sample_block(
        &self,
        request: ReplicaBlockRequest,
    ) -> Result<Vec<Complex<f32>>, SignalError> {
        let mut samples = Vec::with_capacity(request.sample_count);
        for sample_offset in 0..request.sample_count {
            samples.push(sample_modulated_replica_at_sample_index(
                self,
                ReplicaSampleIndexRequest {
                    sample_rate_hz: request.sample_rate_hz,
                    initial_code_phase_chips: request.initial_code_phase_chips,
                    initial_carrier_phase_radians: request.initial_carrier_phase_radians,
                    initial_carrier_hz: request.initial_carrier_hz,
                    carrier_rate_hz_per_s: request.carrier_rate_hz_per_s,
                    sample_index: request.start_sample_index + sample_offset as u64,
                    data_bit: request.data_bit,
                    amplitude: request.amplitude,
                },
            )?);
        }
        Ok(samples)
    }
}
