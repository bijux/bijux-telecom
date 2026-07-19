//! Local-code models used by acquisition and tracking workflows.

use crate::codes::beidou_b1i::{generate_beidou_b1i_code, BEIDOU_B1I_CODE_RATE_HZ};
use crate::codes::beidou_b2i::{generate_beidou_b2i_code, BEIDOU_B2I_CODE_RATE_HZ};
use crate::codes::beidou_d1::beidou_d1_epoch_symbol;
use crate::codes::ca_code::{generate_ca_code, Prn};
use crate::codes::galileo_e1::{
    boc_subcarrier_value, generate_galileo_e1b_code, sample_boc_code, GALILEO_E1_CODE_RATE_HZ,
};
use crate::codes::galileo_e5::{
    galileo_e5a_i_epoch_symbol, galileo_e5b_i_epoch_symbol, generate_galileo_e5a_i_code,
    generate_galileo_e5b_i_code, GALILEO_E5A_CODE_RATE_HZ, GALILEO_E5A_PRIMARY_CODE_CHIPS,
    GALILEO_E5B_CODE_RATE_HZ, GALILEO_E5B_PRIMARY_CODE_CHIPS,
};
use crate::codes::glonass_l1::{generate_glonass_l1_st_code, GLONASS_L1_ST_CODE_RATE_HZ};
use crate::codes::gps_l2c_cl::{generate_gps_l2c_cl_code, GPS_L2C_CL_CODE_RATE_HZ};
use crate::codes::gps_l2c_cm::{generate_gps_l2c_cm_code, GPS_L2C_CM_CODE_RATE_HZ};
use crate::codes::gps_l5::{
    generate_gps_l5_i_code, generate_gps_l5_q_code, gps_l5_i_epoch_symbol, gps_l5_q_epoch_symbol,
    GPS_L5_PRIMARY_CODE_CHIPS, GPS_L5_PRIMARY_CODE_RATE_HZ,
};
use crate::dsp::sample_timing::code_sample_position_at_index;
use crate::dsp::signal::{code_value_at_phase, sample_code};
use crate::error::SignalError;
use bijux_gnss_core::api::{Constellation, SatId, SignalBand, SignalCode};

/// Reusable local code models for acquisition and tracking replicas.
#[derive(Debug, Clone, PartialEq)]
pub enum LocalCodeModel {
    /// Generic BPSK local code with an explicit chip rate.
    Bpsk { code: Vec<i8>, code_rate_hz: f64 },
    /// GPS L1 C/A primary code.
    GpsL1Ca { code: Vec<i8> },
    /// GPS L2C CM primary code.
    GpsL2cCm { code: Vec<i8> },
    /// GPS L2C CL pilot code.
    GpsL2cCl { code: Vec<i8> },
    /// GPS L5-I primary code.
    GpsL5I { code: Vec<i8> },
    /// GPS L5-Q primary code.
    GpsL5Q { code: Vec<i8> },
    /// Galileo E1 BOC(1,1) primary code.
    GalileoE1Boc11 { primary_code: Vec<i8> },
    /// Galileo E5a-I primary code.
    GalileoE5aI { primary_code: Vec<i8> },
    /// Galileo E5b-I primary code.
    GalileoE5bI { primary_code: Vec<i8> },
    /// BeiDou B1I primary code.
    BeidouB1I { code: Vec<i8> },
    /// BeiDou B2I primary code.
    BeidouB2I { code: Vec<i8> },
    /// GLONASS L1 ST primary code.
    GlonassL1St { code: Vec<i8> },
}

impl LocalCodeModel {
    /// Build an all-ones BPSK local code model.
    pub fn ones(code_length: usize, code_rate_hz: f64) -> Self {
        Self::Bpsk { code: vec![1; code_length.max(1)], code_rate_hz }
    }

    /// Build a GPS L1 C/A local code model from a PRN.
    pub fn gps_l1_ca(prn: u8) -> Result<Self, SignalError> {
        Ok(Self::GpsL1Ca { code: generate_ca_code(Prn(prn))? })
    }

    /// Build a GPS L1 C/A local code model, falling back to an all-ones code when invalid.
    pub fn gps_l1_ca_or_ones(prn: u8) -> Self {
        Self::gps_l1_ca(prn).unwrap_or_else(|_| Self::GpsL1Ca { code: vec![1; 1023] })
    }

    /// Build a GPS L2C CM local code model from a PRN.
    pub fn gps_l2c_cm(prn: u8) -> Result<Self, SignalError> {
        Ok(Self::GpsL2cCm { code: generate_gps_l2c_cm_code(prn)? })
    }

    /// Build a GPS L2C CM local code model, falling back to an all-ones code when invalid.
    pub fn gps_l2c_cm_or_ones(prn: u8) -> Self {
        Self::gps_l2c_cm(prn).unwrap_or_else(|_| Self::GpsL2cCm { code: vec![1; 10_230] })
    }

    /// Build a GPS L2C CL local code model from a PRN.
    pub fn gps_l2c_cl(prn: u8) -> Result<Self, SignalError> {
        Ok(Self::GpsL2cCl { code: generate_gps_l2c_cl_code(prn)? })
    }

    /// Build a GPS L2C CL local code model, falling back to an all-ones code when invalid.
    pub fn gps_l2c_cl_or_ones(prn: u8) -> Self {
        Self::gps_l2c_cl(prn).unwrap_or_else(|_| Self::GpsL2cCl { code: vec![1; 767_250] })
    }

    /// Build a GPS L5-I local code model from a PRN.
    pub fn gps_l5_i(prn: u8) -> Result<Self, SignalError> {
        Ok(Self::GpsL5I { code: generate_gps_l5_i_code(prn)? })
    }

    /// Build a GPS L5-I local code model, falling back to an all-ones code when invalid.
    pub fn gps_l5_i_or_ones(prn: u8) -> Self {
        Self::gps_l5_i(prn)
            .unwrap_or_else(|_| Self::GpsL5I { code: vec![1; GPS_L5_PRIMARY_CODE_CHIPS] })
    }

    /// Build a GPS L5-Q local code model from a PRN.
    pub fn gps_l5_q(prn: u8) -> Result<Self, SignalError> {
        Ok(Self::GpsL5Q { code: generate_gps_l5_q_code(prn)? })
    }

    /// Build a GPS L5-Q local code model, falling back to an all-ones code when invalid.
    pub fn gps_l5_q_or_ones(prn: u8) -> Self {
        Self::gps_l5_q(prn)
            .unwrap_or_else(|_| Self::GpsL5Q { code: vec![1; GPS_L5_PRIMARY_CODE_CHIPS] })
    }

    /// Build a Galileo E1 BOC(1,1) local code model from a PRN.
    pub fn galileo_e1_boc11(prn: u8) -> Result<Self, SignalError> {
        Ok(Self::GalileoE1Boc11 { primary_code: generate_galileo_e1b_code(prn)? })
    }

    /// Build a Galileo E1 BOC(1,1) local code model, falling back to an all-ones code when invalid.
    pub fn galileo_e1_boc11_or_ones(prn: u8) -> Self {
        Self::galileo_e1_boc11(prn)
            .unwrap_or_else(|_| Self::GalileoE1Boc11 { primary_code: vec![1; 4092] })
    }

    /// Build a Galileo E5a-I local code model from a PRN.
    pub fn galileo_e5a_i(prn: u8) -> Result<Self, SignalError> {
        Ok(Self::GalileoE5aI { primary_code: generate_galileo_e5a_i_code(prn)? })
    }

    /// Build a Galileo E5a-I local code model, falling back to an all-ones code when invalid.
    pub fn galileo_e5a_i_or_ones(prn: u8) -> Self {
        Self::galileo_e5a_i(prn).unwrap_or_else(|_| Self::GalileoE5aI {
            primary_code: vec![1; GALILEO_E5A_PRIMARY_CODE_CHIPS],
        })
    }

    /// Build a Galileo E5b-I local code model from a PRN.
    pub fn galileo_e5b_i(prn: u8) -> Result<Self, SignalError> {
        Ok(Self::GalileoE5bI { primary_code: generate_galileo_e5b_i_code(prn)? })
    }

    /// Build a Galileo E5b-I local code model, falling back to an all-ones code when invalid.
    pub fn galileo_e5b_i_or_ones(prn: u8) -> Self {
        Self::galileo_e5b_i(prn).unwrap_or_else(|_| Self::GalileoE5bI {
            primary_code: vec![1; GALILEO_E5B_PRIMARY_CODE_CHIPS],
        })
    }

    /// Build a BeiDou B1I local code model from a PRN.
    pub fn beidou_b1i(prn: u8) -> Result<Self, SignalError> {
        Ok(Self::BeidouB1I { code: generate_beidou_b1i_code(prn)? })
    }

    /// Build a BeiDou B1I local code model, falling back to an all-ones code when invalid.
    pub fn beidou_b1i_or_ones(prn: u8) -> Self {
        Self::beidou_b1i(prn).unwrap_or_else(|_| Self::BeidouB1I { code: vec![1; 2046] })
    }

    /// Build a BeiDou B2I local code model from a PRN.
    pub fn beidou_b2i(prn: u8) -> Result<Self, SignalError> {
        Ok(Self::BeidouB2I { code: generate_beidou_b2i_code(prn)? })
    }

    /// Build a BeiDou B2I local code model, falling back to an all-ones code when invalid.
    pub fn beidou_b2i_or_ones(prn: u8) -> Self {
        Self::beidou_b2i(prn).unwrap_or_else(|_| Self::BeidouB2I { code: vec![1; 2046] })
    }

    /// Build a GLONASS L1 ST local code model.
    pub fn glonass_l1_st() -> Self {
        Self::GlonassL1St { code: generate_glonass_l1_st_code() }
    }

    /// Return the code rate in chips per second.
    pub fn code_rate_hz(&self) -> f64 {
        match self {
            Self::Bpsk { code_rate_hz, .. } => *code_rate_hz,
            Self::GpsL1Ca { .. } => 1_023_000.0,
            Self::GpsL2cCm { .. } => GPS_L2C_CM_CODE_RATE_HZ,
            Self::GpsL2cCl { .. } => GPS_L2C_CL_CODE_RATE_HZ,
            Self::GpsL5I { .. } => GPS_L5_PRIMARY_CODE_RATE_HZ,
            Self::GpsL5Q { .. } => GPS_L5_PRIMARY_CODE_RATE_HZ,
            Self::GalileoE1Boc11 { .. } => GALILEO_E1_CODE_RATE_HZ,
            Self::GalileoE5aI { .. } => GALILEO_E5A_CODE_RATE_HZ,
            Self::GalileoE5bI { .. } => GALILEO_E5B_CODE_RATE_HZ,
            Self::BeidouB1I { .. } => BEIDOU_B1I_CODE_RATE_HZ,
            Self::BeidouB2I { .. } => BEIDOU_B2I_CODE_RATE_HZ,
            Self::GlonassL1St { .. } => GLONASS_L1_ST_CODE_RATE_HZ,
        }
    }

    /// Return the primary code period length in chips.
    pub fn code_length(&self) -> usize {
        match self {
            Self::Bpsk { code, .. } => code.len(),
            Self::GpsL1Ca { code } => code.len(),
            Self::GpsL2cCm { code } => code.len(),
            Self::GpsL2cCl { code } => code.len(),
            Self::GpsL5I { code } => code.len(),
            Self::GpsL5Q { code } => code.len(),
            Self::GalileoE1Boc11 { primary_code } => primary_code.len(),
            Self::GalileoE5aI { primary_code } => primary_code.len(),
            Self::GalileoE5bI { primary_code } => primary_code.len(),
            Self::BeidouB1I { code } => code.len(),
            Self::BeidouB2I { code } => code.len(),
            Self::GlonassL1St { code } => code.len(),
        }
    }

    /// Whether secondary-peak multipath screening is meaningful for this code family.
    pub fn supports_secondary_peak_multipath_screening(&self) -> bool {
        !matches!(self, Self::GlonassL1St { .. })
    }

    /// Sample the local code value at a chip phase.
    pub fn sample_value(&self, chip_phase: f64) -> Result<f32, SignalError> {
        match self {
            Self::Bpsk { code, .. }
            | Self::GpsL1Ca { code }
            | Self::GpsL2cCm { code }
            | Self::GpsL2cCl { code }
            | Self::GpsL5I { code }
            | Self::GpsL5Q { code }
            | Self::GalileoE5aI { primary_code: code }
            | Self::GalileoE5bI { primary_code: code }
            | Self::BeidouB1I { code }
            | Self::BeidouB2I { code }
            | Self::GlonassL1St { code } => code_value_at_phase(code, chip_phase),
            Self::GalileoE1Boc11 { primary_code } => {
                Ok(code_value_at_phase(primary_code, chip_phase)?
                    * boc_subcarrier_value(chip_phase, 1)?)
            }
        }
    }

    /// Sample the local code value for tracking with an explicit primary-code-period index.
    pub fn sample_tracking_value(
        &self,
        chip_phase: f64,
        primary_code_period_index: usize,
    ) -> Result<f32, SignalError> {
        match self {
            Self::GpsL5I { code } => Ok(interpolated_bpsk_code_value_at_phase(code, chip_phase)?
                * gps_l5_i_epoch_symbol(&[1], primary_code_period_index)? as f32
                * std::f32::consts::FRAC_1_SQRT_2),
            Self::GpsL5Q { code } => Ok(interpolated_bpsk_code_value_at_phase(code, chip_phase)?
                * gps_l5_q_epoch_symbol(primary_code_period_index) as f32
                * std::f32::consts::FRAC_1_SQRT_2),
            Self::GalileoE5aI { primary_code } => {
                Ok(interpolated_bpsk_code_value_at_phase(primary_code, chip_phase)?
                    * galileo_e5a_i_epoch_symbol(&[1], primary_code_period_index)? as f32
                    * std::f32::consts::FRAC_1_SQRT_2)
            }
            Self::GalileoE5bI { primary_code } => {
                Ok(interpolated_bpsk_code_value_at_phase(primary_code, chip_phase)?
                    * galileo_e5b_i_epoch_symbol(&[1], primary_code_period_index)? as f32
                    * std::f32::consts::FRAC_1_SQRT_2)
            }
            Self::BeidouB1I { code } | Self::BeidouB2I { code } => {
                Ok(interpolated_bpsk_code_value_at_phase(code, chip_phase)?
                    * beidou_d1_epoch_symbol(&[1], primary_code_period_index)? as f32
                    * std::f32::consts::FRAC_1_SQRT_2)
            }
            _ => self.sample_value(chip_phase),
        }
    }

    /// Sample one local code period at an arbitrary sample rate and chip phase.
    pub fn sample_period(
        &self,
        sample_rate_hz: f64,
        start_chip_phase: f64,
        sample_count: usize,
    ) -> Result<Vec<f32>, SignalError> {
        match self {
            Self::Bpsk { code, .. }
            | Self::GpsL1Ca { code }
            | Self::GpsL2cCm { code }
            | Self::GpsL2cCl { code }
            | Self::GpsL5I { code }
            | Self::GpsL5Q { code }
            | Self::GalileoE5aI { primary_code: code }
            | Self::GalileoE5bI { primary_code: code }
            | Self::BeidouB1I { code }
            | Self::BeidouB2I { code }
            | Self::GlonassL1St { code } => sample_code(
                code,
                sample_rate_hz,
                self.code_rate_hz(),
                start_chip_phase,
                sample_count,
            ),
            Self::GalileoE1Boc11 { primary_code } => sample_boc_code(
                primary_code,
                sample_rate_hz,
                self.code_rate_hz(),
                start_chip_phase,
                sample_count,
                1,
            ),
        }
    }

    /// Sample a local-code block from an absolute sample origin without chunk-boundary drift.
    pub fn sample_block(
        &self,
        sample_rate_hz: f64,
        initial_code_phase_chips: f64,
        start_sample_index: u64,
        sample_count: usize,
    ) -> Result<Vec<f32>, SignalError> {
        let mut samples = Vec::with_capacity(sample_count);
        for sample_offset in 0..sample_count {
            let position = code_sample_position_at_index(
                initial_code_phase_chips,
                sample_rate_hz,
                self.code_rate_hz(),
                self.code_length(),
                start_sample_index + sample_offset as u64,
            )?;
            samples.push(self.sample_value(position.chip_phase)?);
        }
        Ok(samples)
    }

    /// Sample a tracking-local code block from an absolute sample origin while preserving
    /// secondary-code and epoch-symbol continuity.
    pub fn sample_tracking_block(
        &self,
        sample_rate_hz: f64,
        initial_code_phase_chips: f64,
        start_sample_index: u64,
        sample_count: usize,
    ) -> Result<Vec<f32>, SignalError> {
        let mut samples = Vec::with_capacity(sample_count);
        for sample_offset in 0..sample_count {
            let position = code_sample_position_at_index(
                initial_code_phase_chips,
                sample_rate_hz,
                self.code_rate_hz(),
                self.code_length(),
                start_sample_index + sample_offset as u64,
            )?;
            samples.push(
                self.sample_tracking_value(
                    position.chip_phase,
                    position.primary_code_period_index,
                )?,
            );
        }
        Ok(samples)
    }
}

fn interpolated_bpsk_code_value_at_phase(code: &[i8], chip_phase: f64) -> Result<f32, SignalError> {
    let current = code_value_at_phase(code, chip_phase)?;
    if code.len() <= 1 {
        return Ok(current);
    }

    let wrapped = chip_phase.rem_euclid(code.len() as f64);
    let chip_index = wrapped.floor() as usize;
    let fractional_offset = (wrapped - chip_index as f64) as f32;
    if fractional_offset <= f32::EPSILON {
        return Ok(current);
    }

    let next = code[(chip_index + 1) % code.len()] as f32;
    Ok(current + (next - current) * fractional_offset)
}

/// Build the default local code model for a supported satellite and signal band.
pub fn default_local_code_model(
    sat: SatId,
    signal_band: SignalBand,
) -> Result<Option<LocalCodeModel>, SignalError> {
    default_local_code_model_for_signal(
        sat,
        signal_band,
        default_signal_code_for_band(sat.constellation, signal_band),
    )
}

/// Build the local code model for one explicit signal identity.
pub fn default_local_code_model_for_signal(
    sat: SatId,
    signal_band: SignalBand,
    signal_code: SignalCode,
) -> Result<Option<LocalCodeModel>, SignalError> {
    match (sat.constellation, signal_band, signal_code) {
        (Constellation::Gps, SignalBand::L1, SignalCode::Ca) => {
            LocalCodeModel::gps_l1_ca(sat.prn).map(Some)
        }
        (Constellation::Gps, SignalBand::L2, SignalCode::L2C) => {
            LocalCodeModel::gps_l2c_cm(sat.prn).map(Some)
        }
        (Constellation::Gps, SignalBand::L5, _) => match signal_code {
            SignalCode::L5I => LocalCodeModel::gps_l5_i(sat.prn).map(Some),
            SignalCode::L5Q => LocalCodeModel::gps_l5_q(sat.prn).map(Some),
            _ => Ok(None),
        },
        (Constellation::Galileo, SignalBand::E1, SignalCode::E1B) => {
            LocalCodeModel::galileo_e1_boc11(sat.prn).map(Some)
        }
        (Constellation::Galileo, SignalBand::E5, SignalCode::E5a) => {
            LocalCodeModel::galileo_e5a_i(sat.prn).map(Some)
        }
        (Constellation::Galileo, SignalBand::E5, SignalCode::E5b) => {
            LocalCodeModel::galileo_e5b_i(sat.prn).map(Some)
        }
        (Constellation::Beidou, SignalBand::B1, SignalCode::B1I) => {
            LocalCodeModel::beidou_b1i(sat.prn).map(Some)
        }
        (Constellation::Beidou, SignalBand::B2, SignalCode::B2I) => {
            LocalCodeModel::beidou_b2i(sat.prn).map(Some)
        }
        (Constellation::Glonass, SignalBand::L1, SignalCode::Unknown) => {
            Ok(Some(LocalCodeModel::glonass_l1_st()))
        }
        _ => Ok(None),
    }
}

fn default_signal_code_for_band(
    constellation: Constellation,
    signal_band: SignalBand,
) -> SignalCode {
    match (constellation, signal_band) {
        (Constellation::Gps, SignalBand::L1) => SignalCode::Ca,
        (Constellation::Gps, SignalBand::L2) => SignalCode::L2C,
        (Constellation::Gps, SignalBand::L5) => SignalCode::L5I,
        (Constellation::Galileo, SignalBand::E1) => SignalCode::E1B,
        (Constellation::Galileo, SignalBand::E5) => SignalCode::E5a,
        (Constellation::Beidou, SignalBand::B1) => SignalCode::B1I,
        (Constellation::Beidou, SignalBand::B2) => SignalCode::B2I,
        (Constellation::Glonass, SignalBand::L1) => SignalCode::Unknown,
        _ => SignalCode::Unknown,
    }
}

#[cfg(test)]
mod tests {
    use super::LocalCodeModel;
    use crate::codes::beidou_d1::BEIDOU_D1_PRIMARY_EPOCHS_PER_SYMBOL;
    use crate::dsp::sample_timing::code_sample_position_at_index;

    #[test]
    fn sample_block_matches_phase_anchored_sampling_at_origin() {
        let model = LocalCodeModel::galileo_e1_boc11(11).expect("valid Galileo E1 PRN");
        let sample_rate_hz = 4_000_000.0;
        let initial_code_phase_chips = 137.625;

        let period_samples = model
            .sample_period(sample_rate_hz, initial_code_phase_chips, 64)
            .expect("phase-anchored local code period");
        let block_samples = model
            .sample_block(sample_rate_hz, initial_code_phase_chips, 0, 64)
            .expect("absolute-index local code block");

        assert_eq!(block_samples, period_samples);
    }

    #[test]
    fn sample_tracking_block_matches_sample_tracking_value_iteration() {
        let model = LocalCodeModel::gps_l5_q(24).expect("valid GPS L5-Q PRN");
        let sample_rate_hz = 1_500_001.0;
        let initial_code_phase_chips = 137.625;
        let start_sample_index = 90_000_123_u64;
        let block_samples = model
            .sample_tracking_block(sample_rate_hz, initial_code_phase_chips, start_sample_index, 64)
            .expect("tracking-local block");
        let iterated = (0..64_u64)
            .map(|sample_offset| {
                let position = code_sample_position_at_index(
                    initial_code_phase_chips,
                    sample_rate_hz,
                    model.code_rate_hz(),
                    model.code_length(),
                    start_sample_index + sample_offset,
                )
                .expect("valid absolute sample position");
                model
                    .sample_tracking_value(position.chip_phase, position.primary_code_period_index)
                    .expect("tracking-local sample")
            })
            .collect::<Vec<_>>();

        assert_eq!(block_samples, iterated);
    }

    #[test]
    fn beidou_tracking_samples_follow_d1_neumann_hoffman_overlay() {
        let model = LocalCodeModel::beidou_b1i(11).expect("valid BeiDou B1I PRN");
        let epoch_zero =
            model.sample_tracking_value(0.25, 0).expect("valid BeiDou B1I epoch-zero sample");
        let epoch_five =
            model.sample_tracking_value(0.25, 5).expect("valid BeiDou B1I NH-flipped sample");
        let epoch_twenty = model
            .sample_tracking_value(0.25, BEIDOU_D1_PRIMARY_EPOCHS_PER_SYMBOL)
            .expect("valid BeiDou B1I repeated NH sample");

        assert!((epoch_zero + epoch_five).abs() <= 1.0e-6);
        assert!((epoch_zero - epoch_twenty).abs() <= 1.0e-6);
    }
}
