//! Replica-generation and modulation helpers for synthetic and tracking workflows.

use crate::catalog::{default_acquisition_signal, signal_spec_glonass_l1};
use crate::codes::beidou_b1i::{generate_beidou_b1i_code, BEIDOU_B1I_CODE_RATE_HZ};
use crate::codes::ca_code::{generate_ca_code, Prn};
use crate::codes::galileo_e1::{
    galileo_e1_cboc_value, generate_galileo_e1b_code, generate_galileo_e1c_code,
    GALILEO_E1_CODE_RATE_HZ,
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
use crate::dsp::local_code::LocalCodeModel;
use crate::dsp::signal::{code_value_at_phase, samples_per_code};
use crate::error::SignalError;
use bijux_gnss_core::api::{
    Constellation, FreqHz, GlonassFrequencyChannel, SatId, SignalBand, SignalCode,
    GPS_L1_CA_CARRIER_HZ,
};
use num_complex::Complex;

/// Complex noise power implied by unit-variance I and Q components.
pub const UNIT_VARIANCE_COMPLEX_NOISE_POWER: f64 = 2.0;

/// Signal-owned metadata and local-code sampling for supported acquisition search signals.
#[derive(Debug, Clone, PartialEq)]
pub struct AcquisitionSignalModel {
    /// Signal band searched during acquisition.
    pub signal_band: SignalBand,
    /// Code rate in chips per second.
    pub code_rate_hz: f64,
    /// Effective code period length in chips.
    pub code_length: usize,
    /// Whole-millisecond coherent period of the primary code.
    pub code_period_ms: u32,
    /// Nominal RF carrier for the signal.
    pub carrier_hz: FreqHz,
    local_code_model: LocalCodeModel,
}

impl AcquisitionSignalModel {
    /// Build the default acquisition model for a supported satellite signal.
    pub fn from_default_signal(
        sat: SatId,
        glonass_frequency_channel: Option<GlonassFrequencyChannel>,
    ) -> Result<Option<Self>, SignalError> {
        match default_acquisition_signal(sat.constellation) {
            Some(signal) => Self::for_sat_signal(
                sat,
                Some(signal.spec.band),
                signal.spec.code,
                glonass_frequency_channel,
            ),
            None => Ok(None),
        }
    }

    /// Build the acquisition model for one explicit satellite signal band.
    pub fn for_sat_signal_band(
        sat: SatId,
        signal_band: Option<SignalBand>,
        glonass_frequency_channel: Option<GlonassFrequencyChannel>,
    ) -> Result<Option<Self>, SignalError> {
        let Some(signal_band) = signal_band else {
            return Ok(None);
        };
        Self::for_sat_signal(
            sat,
            Some(signal_band),
            default_signal_code_for_band(sat.constellation, signal_band),
            glonass_frequency_channel,
        )
    }

    /// Build the acquisition model for one explicit satellite signal identity.
    pub fn for_sat_signal(
        sat: SatId,
        signal_band: Option<SignalBand>,
        signal_code: SignalCode,
        glonass_frequency_channel: Option<GlonassFrequencyChannel>,
    ) -> Result<Option<Self>, SignalError> {
        let Some(signal_band) = signal_band else {
            return Ok(None);
        };

        match (sat.constellation, signal_band, signal_code) {
            (Constellation::Gps, SignalBand::L1, SignalCode::Ca) => {
                Self::gps_l1_ca(sat.prn, 1_023_000.0, 1023, GPS_L1_CA_CARRIER_HZ).map(Some)
            }
            (Constellation::Gps, SignalBand::L5, _) => match signal_code {
                SignalCode::L5I => Self::new(
                    SignalBand::L5,
                    GPS_L5_PRIMARY_CODE_RATE_HZ,
                    GPS_L5_PRIMARY_CODE_CHIPS,
                    bijux_gnss_core::api::GPS_L5_CARRIER_HZ,
                    LocalCodeModel::gps_l5_i(sat.prn)?,
                )
                .map(Some),
                SignalCode::L5Q => Self::new(
                    SignalBand::L5,
                    GPS_L5_PRIMARY_CODE_RATE_HZ,
                    GPS_L5_PRIMARY_CODE_CHIPS,
                    bijux_gnss_core::api::GPS_L5_CARRIER_HZ,
                    LocalCodeModel::gps_l5_q(sat.prn)?,
                )
                .map(Some),
                _ => Ok(None),
            },
            (Constellation::Galileo, SignalBand::E1, SignalCode::E1B) => Self::new(
                SignalBand::E1,
                GALILEO_E1_CODE_RATE_HZ,
                4092,
                bijux_gnss_core::api::GALILEO_E1_CARRIER_HZ,
                LocalCodeModel::galileo_e1_boc11(sat.prn)?,
            )
            .map(Some),
            (Constellation::Beidou, SignalBand::B1, SignalCode::B1I) => Self::new(
                SignalBand::B1,
                BEIDOU_B1I_CODE_RATE_HZ,
                2046,
                bijux_gnss_core::api::BEIDOU_B1_CARRIER_HZ,
                LocalCodeModel::beidou_b1i(sat.prn)?,
            )
            .map(Some),
            (Constellation::Glonass, SignalBand::L1, SignalCode::Unknown) => {
                let channel = glonass_frequency_channel
                    .ok_or(SignalError::MissingGlonassFrequencyChannel(sat))?;
                let signal = signal_spec_glonass_l1(channel);
                Self::new(
                    signal.band,
                    signal.code_rate_hz,
                    511,
                    signal.carrier_hz,
                    LocalCodeModel::glonass_l1_st(),
                )
                .map(Some)
            }
            _ => Ok(None),
        }
    }

    /// Build a GPS L1 C/A acquisition model with explicit search metadata.
    pub fn gps_l1_ca(
        prn: u8,
        code_rate_hz: f64,
        code_length: usize,
        carrier_hz: FreqHz,
    ) -> Result<Self, SignalError> {
        Self::new(
            SignalBand::L1,
            code_rate_hz,
            code_length,
            carrier_hz,
            LocalCodeModel::gps_l1_ca(prn)?,
        )
    }

    /// Build a GPS L1 C/A acquisition model, falling back to an all-ones code when invalid.
    pub fn gps_l1_ca_or_ones(
        prn: u8,
        code_rate_hz: f64,
        code_length: usize,
        carrier_hz: FreqHz,
    ) -> Result<Self, SignalError> {
        Self::new(
            SignalBand::L1,
            code_rate_hz,
            code_length,
            carrier_hz,
            LocalCodeModel::gps_l1_ca_or_ones(prn),
        )
    }

    fn new(
        signal_band: SignalBand,
        code_rate_hz: f64,
        code_length: usize,
        carrier_hz: FreqHz,
        local_code_model: LocalCodeModel,
    ) -> Result<Self, SignalError> {
        Ok(Self {
            signal_band,
            code_rate_hz,
            code_length,
            code_period_ms: primary_code_period_ms(code_length, code_rate_hz)?,
            carrier_hz,
            local_code_model,
        })
    }

    /// Return the sampled-code period length for a sample rate.
    pub fn samples_per_code(&self, sample_rate_hz: f64) -> usize {
        samples_per_code(sample_rate_hz, self.code_rate_hz, self.code_length)
    }

    /// Return the number of whole primary-code periods in a coherent integration interval.
    pub fn coherent_periods(&self, coherent_ms: u32) -> Option<u32> {
        if coherent_ms == 0 || coherent_ms % self.code_period_ms != 0 {
            return None;
        }
        Some(coherent_ms / self.code_period_ms)
    }

    /// Return the centered IF used by a receiver whose reference carrier is GPS L1 C/A.
    pub fn search_center_hz(&self, intermediate_freq_hz: f64) -> f64 {
        intermediate_freq_hz + (self.carrier_hz.value() - GPS_L1_CA_CARRIER_HZ.value())
    }

    /// Sample one acquisition local-code period at the supplied sample rate.
    pub fn local_code_period(&self, sample_rate_hz: f64) -> Result<Vec<f32>, SignalError> {
        self.local_code_model.sample_period(
            sample_rate_hz,
            0.0,
            self.samples_per_code(sample_rate_hz),
        )
    }

    /// Sample one acquisition local-code period into a caller-specified number of samples.
    pub fn sampled_local_code_period(
        &self,
        sample_rate_hz: f64,
        sample_count: usize,
    ) -> Result<Vec<f32>, SignalError> {
        self.local_code_model.sample_period(sample_rate_hz, 0.0, sample_count)
    }

    /// Whether delayed-secondary-peak multipath screening is meaningful for this code family.
    pub fn supports_secondary_peak_multipath_screening(&self) -> bool {
        self.local_code_model.supports_secondary_peak_multipath_screening()
    }
}

fn primary_code_period_ms(code_length: usize, code_rate_hz: f64) -> Result<u32, SignalError> {
    if code_length == 0 {
        return Err(SignalError::EmptyCodeSequence);
    }
    if !code_rate_hz.is_finite() || code_rate_hz <= 0.0 {
        return Err(SignalError::InvalidCodeRate);
    }
    let period_ms = ((code_length as f64 / code_rate_hz) * 1_000.0).round();
    if !period_ms.is_finite() || period_ms < 1.0 {
        return Err(SignalError::InvalidCodeRate);
    }
    Ok(period_ms as u32)
}

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
    /// BeiDou B1I primary code.
    BeidouB1I { code: Vec<i8> },
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
            (Constellation::Beidou, SignalBand::B1, SignalCode::B1I) => {
                Self::beidou_b1i(sat.prn).map(Some)
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

    /// Build a BeiDou B1I replica from a PRN.
    pub fn beidou_b1i(prn: u8) -> Result<Self, SignalError> {
        Ok(Self::BeidouB1I { code: generate_beidou_b1i_code(prn)? })
    }

    /// Build a BeiDou B1I replica, falling back to an all-ones code when invalid.
    pub fn beidou_b1i_or_ones(prn: u8) -> Self {
        Self::beidou_b1i(prn).unwrap_or_else(|_| Self::BeidouB1I { code: vec![1; 2046] })
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
            Self::BeidouB1I { .. } => BEIDOU_B1I_CODE_RATE_HZ,
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
            Self::GpsL2cTimeMultiplexed { cm_code, cl_code } => {
                gps_l2c_time_multiplexed_value(cm_code, cl_code, chip_phase, &[data_bit])
            }
            Self::GpsL5I { code } => Ok(code_value_at_phase(code, chip_phase)?
                * gps_l5_i_epoch_symbol(&[data_bit], primary_code_period_index)? as f32
                * std::f32::consts::FRAC_1_SQRT_2),
            Self::GpsL5Q { code } => Ok(code_value_at_phase(code, chip_phase)?
                * gps_l5_q_epoch_symbol(primary_code_period_index) as f32
                * std::f32::consts::FRAC_1_SQRT_2),
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

/// Carrier frequency for the default synthesized signal of a satellite.
pub fn default_signal_carrier_hz(
    sat: SatId,
    glonass_frequency_channel: Option<GlonassFrequencyChannel>,
) -> Result<Option<FreqHz>, SignalError> {
    default_signal_carrier_hz_for_band(
        sat,
        default_acquisition_signal(sat.constellation).map(|signal| signal.spec.band),
        glonass_frequency_channel,
    )
}

/// Carrier frequency for one explicit synthesized satellite signal band.
pub fn default_signal_carrier_hz_for_band(
    sat: SatId,
    signal_band: Option<SignalBand>,
    glonass_frequency_channel: Option<GlonassFrequencyChannel>,
) -> Result<Option<FreqHz>, SignalError> {
    let Some(signal_band) = signal_band else {
        return Ok(None);
    };
    default_signal_carrier_hz_for_signal(
        sat,
        Some(signal_band),
        default_signal_code_for_band(sat.constellation, signal_band),
        glonass_frequency_channel,
    )
}

/// Carrier frequency for one explicit synthesized satellite signal identity.
pub fn default_signal_carrier_hz_for_signal(
    sat: SatId,
    signal_band: Option<SignalBand>,
    signal_code: SignalCode,
    glonass_frequency_channel: Option<GlonassFrequencyChannel>,
) -> Result<Option<FreqHz>, SignalError> {
    let Some(signal_band) = signal_band else {
        return Ok(None);
    };

    match (sat.constellation, signal_band, signal_code) {
        (Constellation::Gps, SignalBand::L5, SignalCode::L5I | SignalCode::L5Q) => {
            Ok(Some(bijux_gnss_core::api::GPS_L5_CARRIER_HZ))
        }
        (Constellation::Glonass, SignalBand::L1, SignalCode::Unknown) => {
            let channel = glonass_frequency_channel
                .ok_or(SignalError::MissingGlonassFrequencyChannel(sat))?;
            Ok(Some(signal_spec_glonass_l1(channel).carrier_hz))
        }
        _ => match default_acquisition_signal(sat.constellation) {
            Some(signal) if signal.spec.band == signal_band && signal.spec.code == signal_code => {
                Ok(Some(signal.spec.carrier_hz))
            }
            _ => Ok(None),
        },
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
        (Constellation::Beidou, SignalBand::B1) => SignalCode::B1I,
        (Constellation::Glonass, SignalBand::L1) => SignalCode::Unknown,
        _ => SignalCode::Unknown,
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
        carrier_hz_at_time, carrier_phase_radians_at_time, default_signal_carrier_hz,
        default_signal_carrier_hz_for_band, sample_modulated_replica_at_time,
        signal_amplitude_from_cn0_db_hz, AcquisitionSignalModel, ReplicaCodeModel,
        GPS_L2C_TIME_MULTIPLEXED_CODE_CHIPS,
        GPS_L2C_TIME_MULTIPLEXED_CODE_RATE_HZ, UNIT_VARIANCE_COMPLEX_NOISE_POWER,
    };
    use crate::codes::galileo_e1::{
        sample_galileo_e1_boc11_code, GalileoE1Channel, GALILEO_E1_CODE_RATE_HZ,
    };
    use crate::codes::glonass_l1::sample_glonass_l1_st_code;
    use crate::codes::gps_l2c_cl::{sample_gps_l2c_cl_code, GPS_L2C_CL_CODE_RATE_HZ};
    use crate::codes::gps_l2c_cm::{sample_gps_l2c_cm_code, GPS_L2C_CM_CODE_RATE_HZ};
    use crate::codes::gps_l5::{
        sample_gps_l5_i_primary_code, sample_gps_l5_q_primary_code, GPS_L5_PRIMARY_CODE_RATE_HZ,
    };
    use crate::dsp::local_code::{
        default_local_code_model, default_local_code_model_for_signal, LocalCodeModel,
    };
    use crate::error::SignalError;
    use bijux_gnss_core::api::{
        Constellation, GlonassFrequencyChannel, SatId, SignalBand, SignalCode,
        GPS_L1_CA_CARRIER_HZ, GPS_L5_CARRIER_HZ,
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

    #[test]
    fn replica_code_model_fallback_constructors_preserve_code_lengths() {
        let gps = ReplicaCodeModel::gps_l1_ca_or_ones(0);
        let gps_l2c = ReplicaCodeModel::gps_l2c_time_multiplexed_or_ones(0);
        let gps_l5 = ReplicaCodeModel::gps_l5_i_or_ones(0);
        let gps_l5q = ReplicaCodeModel::gps_l5_q_or_ones(0);
        let galileo = ReplicaCodeModel::galileo_e1_cboc_or_ones(0);
        let beidou = ReplicaCodeModel::beidou_b1i_or_ones(0);

        assert_eq!(gps.code_length(), 1023);
        assert_eq!(gps_l2c.code_length(), GPS_L2C_TIME_MULTIPLEXED_CODE_CHIPS);
        assert_eq!(gps_l5.code_length(), 10_230);
        assert_eq!(gps_l5q.code_length(), 10_230);
        assert_eq!(galileo.code_length(), 4092);
        assert_eq!(beidou.code_length(), 2046);
    }

    #[test]
    fn local_code_model_samples_glonass_period_consistently() {
        let model = LocalCodeModel::glonass_l1_st();
        let samples = model.sample_period(511_000.0, 0.0, 511).expect("GLONASS local code period");
        let expected = sample_glonass_l1_st_code(511_000.0, 0.0, 511).expect("GLONASS reference");
        assert_eq!(samples, expected);
        assert!(!model.supports_secondary_peak_multipath_screening());
    }

    #[test]
    fn local_code_model_samples_gps_l2c_cm_period_consistently() {
        let model = LocalCodeModel::gps_l2c_cm(38).expect("valid GPS L2C CM PRN");
        let samples = model
            .sample_period(GPS_L2C_CM_CODE_RATE_HZ, 0.0, 32)
            .expect("GPS L2C CM local code period");
        let expected = sample_gps_l2c_cm_code(38, GPS_L2C_CM_CODE_RATE_HZ, 0.0, 32)
            .expect("GPS L2C CM reference");

        assert_eq!(samples, expected);
        assert!(model.supports_secondary_peak_multipath_screening());
    }

    #[test]
    fn local_code_model_samples_gps_l2c_cl_period_consistently() {
        let model = LocalCodeModel::gps_l2c_cl(38).expect("valid GPS L2C CL PRN");
        let samples = model
            .sample_period(GPS_L2C_CL_CODE_RATE_HZ, 0.0, 32)
            .expect("GPS L2C CL local code period");
        let expected = sample_gps_l2c_cl_code(38, GPS_L2C_CL_CODE_RATE_HZ, 0.0, 32)
            .expect("GPS L2C CL reference");

        assert_eq!(samples, expected);
        assert!(model.supports_secondary_peak_multipath_screening());
    }

    #[test]
    fn local_code_model_samples_gps_l5_i_period_consistently() {
        let model = LocalCodeModel::gps_l5_i(7).expect("valid GPS L5-I PRN");
        let samples = model
            .sample_period(GPS_L5_PRIMARY_CODE_RATE_HZ, 0.0, 32)
            .expect("GPS L5-I local code period");
        let expected = sample_gps_l5_i_primary_code(7, GPS_L5_PRIMARY_CODE_RATE_HZ, 0.0, 32)
            .expect("GPS L5-I reference");

        assert_eq!(samples, expected);
        assert!(model.supports_secondary_peak_multipath_screening());
    }

    #[test]
    fn local_code_model_samples_gps_l5_q_period_consistently() {
        let model = LocalCodeModel::gps_l5_q(7).expect("valid GPS L5-Q PRN");
        let samples = model
            .sample_period(GPS_L5_PRIMARY_CODE_RATE_HZ, 0.0, 32)
            .expect("GPS L5-Q local code period");
        let expected = sample_gps_l5_q_primary_code(7, GPS_L5_PRIMARY_CODE_RATE_HZ, 0.0, 32)
            .expect("GPS L5-Q reference");

        assert_eq!(samples, expected);
        assert!(model.supports_secondary_peak_multipath_screening());
    }

    #[test]
    fn local_code_model_ones_uses_requested_rate_and_length() {
        let model = LocalCodeModel::ones(7, 2_500.0);
        assert_eq!(model.code_length(), 7);
        assert_eq!(model.code_rate_hz(), 2_500.0);
        assert_eq!(model.sample_value(3.25).expect("ones code"), 1.0);
    }

    #[test]
    fn local_code_model_samples_galileo_boc11_value() {
        let model = LocalCodeModel::galileo_e1_boc11(11).expect("valid Galileo PRN");
        let chip_phase = 12.25;
        let sample = model.sample_value(chip_phase).expect("sample value");
        let expected =
            model.sample_period(GALILEO_E1_CODE_RATE_HZ, chip_phase, 1).expect("single sample")[0];
        assert!(
            (sample.abs() - expected.abs()).abs() < 1.0e-6,
            "sample={sample} expected={expected}"
        );
    }

    #[test]
    fn local_code_model_samples_galileo_period_consistently() {
        let model = LocalCodeModel::galileo_e1_boc11(11).expect("valid Galileo PRN");
        let samples = model.sample_period(4_092_000.0, 0.0, 16).expect("Galileo local code period");
        let expected =
            sample_galileo_e1_boc11_code(11, GalileoE1Channel::E1B, 4_092_000.0, 0.0, 16)
                .expect("Galileo reference");
        assert_eq!(samples, expected);
        assert!(model.supports_secondary_peak_multipath_screening());
    }

    #[test]
    fn acquisition_signal_model_samples_glonass_period_consistently() {
        let sat = SatId { constellation: Constellation::Glonass, prn: 8 };
        let channel = GlonassFrequencyChannel::new(-4).expect("valid GLONASS channel");
        let model = AcquisitionSignalModel::from_default_signal(sat, Some(channel))
            .expect("signal model result")
            .expect("GLONASS acquisition model");
        let samples = model.local_code_period(511_000.0).expect("GLONASS local code period");
        let expected = sample_glonass_l1_st_code(511_000.0, 0.0, 511).expect("GLONASS reference");

        assert_eq!(model.signal_band, SignalBand::L1);
        assert_eq!(model.code_rate_hz, 511_000.0);
        assert_eq!(model.code_length, 511);
        assert_eq!(model.code_period_ms, 1);
        assert_eq!(samples, expected);
        assert!(!model.supports_secondary_peak_multipath_screening());
    }

    #[test]
    fn acquisition_signal_model_requires_glonass_channel() {
        let sat = SatId { constellation: Constellation::Glonass, prn: 8 };
        let error = AcquisitionSignalModel::from_default_signal(sat, None)
            .expect_err("GLONASS acquisition model must reject missing channel");

        assert_eq!(error, SignalError::MissingGlonassFrequencyChannel(sat));
    }

    #[test]
    fn acquisition_signal_model_reports_search_center_relative_to_l1_ca() {
        let model = AcquisitionSignalModel::gps_l1_ca(11, 1_023_000.0, 1023, GPS_L1_CA_CARRIER_HZ)
            .expect("GPS acquisition model");

        assert!((model.search_center_hz(125_000.0) - 125_000.0).abs() < f64::EPSILON);
    }

    #[test]
    fn acquisition_signal_model_builds_gps_l5_i_search_metadata() {
        let sat = SatId { constellation: Constellation::Gps, prn: 7 };
        let model = AcquisitionSignalModel::for_sat_signal_band(sat, Some(SignalBand::L5), None)
            .expect("signal model result")
            .expect("GPS L5-I acquisition model");

        assert_eq!(model.signal_band, SignalBand::L5);
        assert_eq!(model.code_rate_hz, 10_230_000.0);
        assert_eq!(model.code_length, 10_230);
        assert_eq!(model.code_period_ms, 1);
        assert_eq!(model.carrier_hz, GPS_L5_CARRIER_HZ);
    }

    #[test]
    fn acquisition_signal_model_builds_gps_l5_q_search_metadata() {
        let sat = SatId { constellation: Constellation::Gps, prn: 7 };
        let model = AcquisitionSignalModel::for_sat_signal(
            sat,
            Some(SignalBand::L5),
            SignalCode::L5Q,
            None,
        )
        .expect("signal model result")
        .expect("GPS L5-Q acquisition model");

        assert_eq!(model.signal_band, SignalBand::L5);
        assert_eq!(model.code_rate_hz, 10_230_000.0);
        assert_eq!(model.code_length, 10_230);
        assert_eq!(model.code_period_ms, 1);
        assert_eq!(model.carrier_hz, GPS_L5_CARRIER_HZ);
    }

    #[test]
    fn replica_code_model_builds_default_galileo_signal() {
        let sat = SatId { constellation: Constellation::Galileo, prn: 11 };
        let model = ReplicaCodeModel::from_default_signal(sat)
            .expect("signal model result")
            .expect("Galileo default signal");

        assert_eq!(model.code_length(), 4092);
        assert!((model.code_rate_hz() - GALILEO_E1_CODE_RATE_HZ).abs() < f64::EPSILON);
    }

    #[test]
    fn replica_code_model_builds_gps_l2c_time_multiplexed_signal() {
        let model = ReplicaCodeModel::gps_l2c_time_multiplexed(38).expect("valid GPS L2C PRN");

        assert_eq!(model.code_length(), GPS_L2C_TIME_MULTIPLEXED_CODE_CHIPS);
        assert!(
            (model.code_rate_hz() - GPS_L2C_TIME_MULTIPLEXED_CODE_RATE_HZ).abs() < f64::EPSILON
        );
    }

    #[test]
    fn replica_code_model_builds_gps_l5_i_signal() {
        let sat = SatId { constellation: Constellation::Gps, prn: 7 };
        let model = ReplicaCodeModel::for_sat_signal_band(sat, Some(SignalBand::L5))
            .expect("signal model result")
            .expect("GPS L5-I signal");

        assert_eq!(model.code_length(), 10_230);
        assert_eq!(model.code_rate_hz(), 10_230_000.0);
    }

    #[test]
    fn replica_code_model_builds_gps_l5_q_signal() {
        let sat = SatId { constellation: Constellation::Gps, prn: 7 };
        let model = ReplicaCodeModel::for_sat_signal(sat, Some(SignalBand::L5), SignalCode::L5Q)
            .expect("signal model result")
            .expect("GPS L5-Q signal");

        assert_eq!(model.code_length(), 10_230);
        assert_eq!(model.code_rate_hz(), 10_230_000.0);
    }

    #[test]
    fn gps_l5_i_replica_uses_half_power_component_scaling() {
        let model = ReplicaCodeModel::gps_l5_i(7).expect("valid GPS L5-I PRN");
        let sample = sample_modulated_replica_at_time(&model, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 1.0)
            .expect("valid replica");

        assert!((sample.norm() - std::f32::consts::FRAC_1_SQRT_2).abs() < 1.0e-6, "{sample:?}");
    }

    #[test]
    fn gps_l5_q_replica_uses_half_power_component_scaling() {
        let model = ReplicaCodeModel::gps_l5_q(7).expect("valid GPS L5-Q PRN");
        let sample = sample_modulated_replica_at_time(&model, 0.0, 0.0, 0.0, 0.0, 0.0, -1, 1.0)
            .expect("valid replica");

        assert!((sample.norm() - std::f32::consts::FRAC_1_SQRT_2).abs() < 1.0e-6, "{sample:?}");
    }

    #[test]
    fn gps_l5_q_tracking_replica_interpolates_subchip_boundaries() {
        let model = LocalCodeModel::GpsL5Q { code: vec![1, -1] };

        let sample = model.sample_tracking_value(0.5, 0).expect("tracking sample");

        assert!(sample.abs() < 1.0e-6, "{sample}");
    }

    #[test]
    fn default_signal_carrier_hz_requires_glonass_channel() {
        let sat = SatId { constellation: Constellation::Glonass, prn: 8 };
        let error = default_signal_carrier_hz(sat, None)
            .expect_err("GLONASS carrier lookup must reject missing channel");

        assert_eq!(error, SignalError::MissingGlonassFrequencyChannel(sat));
    }

    #[test]
    fn default_local_code_model_builds_galileo_tracking_code() {
        let sat = SatId { constellation: Constellation::Galileo, prn: 11 };
        let model = default_local_code_model(sat, SignalBand::E1)
            .expect("local code result")
            .expect("Galileo local code");

        assert_eq!(model.code_length(), 4092);
        assert!(model.supports_secondary_peak_multipath_screening());
    }

    #[test]
    fn default_local_code_model_builds_gps_l2c_tracking_code() {
        let sat = SatId { constellation: Constellation::Gps, prn: 38 };
        let model = default_local_code_model(sat, SignalBand::L2)
            .expect("local code result")
            .expect("GPS L2C local code");

        assert_eq!(model.code_length(), 10_230);
        assert!((model.code_rate_hz() - GPS_L2C_CM_CODE_RATE_HZ).abs() < f64::EPSILON);
        assert!(model.supports_secondary_peak_multipath_screening());
    }

    #[test]
    fn default_local_code_model_builds_gps_l5_i_tracking_code() {
        let sat = SatId { constellation: Constellation::Gps, prn: 7 };
        let model = default_local_code_model(sat, SignalBand::L5)
            .expect("local code result")
            .expect("GPS L5-I local code");

        assert_eq!(model.code_length(), 10_230);
        assert!((model.code_rate_hz() - 10_230_000.0).abs() < f64::EPSILON);
        assert!(model.supports_secondary_peak_multipath_screening());
    }

    #[test]
    fn default_signal_carrier_hz_for_band_returns_gps_l5_carrier() {
        let sat = SatId { constellation: Constellation::Gps, prn: 7 };
        let carrier = default_signal_carrier_hz_for_band(sat, Some(SignalBand::L5), None)
            .expect("carrier lookup")
            .expect("GPS L5 carrier");

        assert_eq!(carrier, GPS_L5_CARRIER_HZ);
    }

    #[test]
    fn default_local_code_model_for_signal_builds_gps_l5_q_tracking_code() {
        let sat = SatId { constellation: Constellation::Gps, prn: 7 };
        let model =
            default_local_code_model_for_signal(sat, SignalBand::L5, SignalCode::L5Q)
                .expect("local code result")
                .expect("GPS L5-Q local code");

        assert_eq!(model.code_length(), 10_230);
        assert!((model.code_rate_hz() - 10_230_000.0).abs() < f64::EPSILON);
        assert!(model.supports_secondary_peak_multipath_screening());
    }
}
