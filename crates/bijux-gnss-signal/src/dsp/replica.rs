//! Replica-generation and modulation helpers for synthetic and tracking workflows.

use crate::codes::beidou_b1i::{generate_beidou_b1i_code, BEIDOU_B1I_CODE_RATE_HZ};
use crate::codes::ca_code::{generate_ca_code, Prn};
use crate::codes::galileo_e1::{
    galileo_e1_cboc_value, generate_galileo_e1b_code, generate_galileo_e1c_code,
    GALILEO_E1_CODE_RATE_HZ,
};
use crate::codes::glonass_l1::{generate_glonass_l1_st_code, GLONASS_L1_ST_CODE_RATE_HZ};
use crate::dsp::signal::code_value_at_phase;
use crate::error::SignalError;
use num_complex::Complex;

/// Complex noise power implied by unit-variance I and Q components.
pub const UNIT_VARIANCE_COMPLEX_NOISE_POWER: f64 = 2.0;

/// Reusable spreading-code models for synthesized signal replicas.
#[derive(Debug, Clone, PartialEq)]
pub enum ReplicaCodeModel {
    /// GPS L1 C/A primary code.
    GpsL1Ca { code: Vec<i8> },
    /// Galileo E1 CBOC composite code.
    GalileoE1Cboc { e1b_code: Vec<i8>, e1c_code: Vec<i8> },
    /// BeiDou B1I primary code.
    BeidouB1I { code: Vec<i8> },
    /// GLONASS L1 ST primary code.
    GlonassL1St { code: Vec<i8> },
}

impl ReplicaCodeModel {
    /// Build a GPS L1 C/A replica from a PRN.
    pub fn gps_l1_ca(prn: u8) -> Result<Self, SignalError> {
        Ok(Self::GpsL1Ca { code: generate_ca_code(Prn(prn))? })
    }

    /// Build a Galileo E1 CBOC replica from a PRN.
    pub fn galileo_e1_cboc(prn: u8) -> Result<Self, SignalError> {
        Ok(Self::GalileoE1Cboc {
            e1b_code: generate_galileo_e1b_code(prn)?,
            e1c_code: generate_galileo_e1c_code(prn)?,
        })
    }

    /// Build a BeiDou B1I replica from a PRN.
    pub fn beidou_b1i(prn: u8) -> Result<Self, SignalError> {
        Ok(Self::BeidouB1I { code: generate_beidou_b1i_code(prn)? })
    }

    /// Build a GLONASS L1 ST replica.
    pub fn glonass_l1_st() -> Self {
        Self::GlonassL1St { code: generate_glonass_l1_st_code() }
    }

    /// Return the code rate in chips per second.
    pub fn code_rate_hz(&self) -> f64 {
        match self {
            Self::GpsL1Ca { .. } => 1_023_000.0,
            Self::GalileoE1Cboc { .. } => GALILEO_E1_CODE_RATE_HZ,
            Self::BeidouB1I { .. } => BEIDOU_B1I_CODE_RATE_HZ,
            Self::GlonassL1St { .. } => GLONASS_L1_ST_CODE_RATE_HZ,
        }
    }

    /// Return the primary code period length in chips.
    pub fn code_length(&self) -> usize {
        match self {
            Self::GpsL1Ca { code } => code.len(),
            Self::GalileoE1Cboc { e1b_code, .. } => e1b_code.len(),
            Self::BeidouB1I { code } => code.len(),
            Self::GlonassL1St { code } => code.len(),
        }
    }

    /// Sample the replica code/modulation value at a chip phase and primary-period index.
    pub fn sample_value(
        &self,
        chip_phase: f64,
        primary_code_period_index: usize,
        data_bit: i8,
    ) -> Result<f32, SignalError> {
        match self {
            Self::GpsL1Ca { code } => Ok(code_value_at_phase(code, chip_phase)? * data_bit as f32),
            Self::GalileoE1Cboc { e1b_code, e1c_code } => galileo_e1_cboc_value(
                e1b_code,
                e1c_code,
                chip_phase,
                primary_code_period_index,
                data_bit,
            ),
            Self::BeidouB1I { code } | Self::GlonassL1St { code } => {
                Ok(code_value_at_phase(code, chip_phase)? * data_bit as f32)
            }
        }
    }
}

/// Carrier frequency at elapsed time for a linear Doppler-rate model.
pub fn carrier_hz_at_time(
    initial_carrier_hz: f64,
    carrier_rate_hz_per_s: f64,
    elapsed_s: f64,
) -> f64 {
    initial_carrier_hz + carrier_rate_hz_per_s * elapsed_s
}

/// Carrier phase in radians at elapsed time for a linear Doppler-rate model.
pub fn carrier_phase_radians_at_time(
    initial_phase_radians: f64,
    initial_carrier_hz: f64,
    carrier_rate_hz_per_s: f64,
    elapsed_s: f64,
) -> f64 {
    initial_phase_radians
        + std::f64::consts::TAU
            * (initial_carrier_hz * elapsed_s + 0.5 * carrier_rate_hz_per_s * elapsed_s * elapsed_s)
}

/// Convert C/N0 into the synthesized signal amplitude for a known complex noise power.
pub fn signal_amplitude_from_cn0_db_hz(
    cn0_db_hz: f32,
    sample_rate_hz: f64,
    complex_noise_power: f64,
) -> f32 {
    let cn0_linear = 10.0_f64.powf(cn0_db_hz as f64 / 10.0).max(1e-12);
    ((cn0_linear * complex_noise_power.max(1e-12)) / sample_rate_hz.max(1e-12)).sqrt() as f32
}

/// Sample a modulated replica at one elapsed time.
pub fn sample_modulated_replica_at_time(
    model: &ReplicaCodeModel,
    initial_code_phase_chips: f64,
    initial_carrier_phase_radians: f64,
    initial_carrier_hz: f64,
    carrier_rate_hz_per_s: f64,
    elapsed_s: f64,
    data_bit: i8,
    amplitude: f32,
) -> Result<Complex<f32>, SignalError> {
    let total_chip_phase = initial_code_phase_chips + model.code_rate_hz() * elapsed_s;
    let code_length = model.code_length().max(1) as f64;
    let primary_code_period_index =
        if total_chip_phase <= 0.0 { 0 } else { (total_chip_phase / code_length).floor() as usize };
    let code_phase = total_chip_phase.rem_euclid(code_length);
    let signal_value = model.sample_value(code_phase, primary_code_period_index, data_bit)?;
    let phase = carrier_phase_radians_at_time(
        initial_carrier_phase_radians,
        initial_carrier_hz,
        carrier_rate_hz_per_s,
        elapsed_s,
    ) as f32;
    let carrier = Complex::new(phase.cos(), phase.sin());
    Ok(carrier * (signal_value * amplitude))
}

#[cfg(test)]
mod tests {
    use super::{
        carrier_hz_at_time, carrier_phase_radians_at_time, sample_modulated_replica_at_time,
        signal_amplitude_from_cn0_db_hz, ReplicaCodeModel, UNIT_VARIANCE_COMPLEX_NOISE_POWER,
    };

    #[test]
    fn carrier_hz_at_time_applies_linear_rate() {
        assert!((carrier_hz_at_time(1_350.0, 40.0, 0.5) - 1_370.0).abs() < 1.0e-12);
    }

    #[test]
    fn carrier_phase_radians_at_time_integrates_linear_rate() {
        let phase = carrier_phase_radians_at_time(0.25, 1_000.0, 20.0, 0.5);
        let expected = 0.25 + std::f64::consts::TAU * (1_000.0 * 0.5 + 0.5 * 20.0 * 0.25);
        assert!((phase - expected).abs() < 1.0e-12, "phase={phase}");
    }

    #[test]
    fn signal_amplitude_from_cn0_db_hz_scales_with_cn0() {
        let weak =
            signal_amplitude_from_cn0_db_hz(35.0, 4_000_000.0, UNIT_VARIANCE_COMPLEX_NOISE_POWER);
        let strong =
            signal_amplitude_from_cn0_db_hz(45.0, 4_000_000.0, UNIT_VARIANCE_COMPLEX_NOISE_POWER);
        assert!(strong > weak, "weak={weak} strong={strong}");
    }

    #[test]
    fn sample_modulated_replica_at_time_matches_unity_carrier_and_code_origin() {
        let model = ReplicaCodeModel::gps_l1_ca(1).expect("valid GPS PRN");
        let sample = sample_modulated_replica_at_time(&model, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 2.0)
            .expect("valid replica");
        assert!((sample.norm() - 2.0).abs() < 1.0e-6, "sample={sample:?}");
    }
}
