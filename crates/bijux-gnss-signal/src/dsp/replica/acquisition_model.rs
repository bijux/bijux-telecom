use crate::catalog::{default_acquisition_signal, resolved_signal_registry_entry};
use crate::dsp::local_code::LocalCodeModel;
use crate::dsp::signal::samples_per_code;
use crate::error::SignalError;
use bijux_gnss_core::api::{
    Constellation, FreqHz, GlonassFrequencyChannel, SatId, SignalBand, SignalCode,
    GPS_L1_CA_CARRIER_HZ,
};

use super::signal_identity::default_signal_code_for_band;

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

        let registry_entry = resolved_signal_registry_entry(
            sat,
            signal_band,
            signal_code,
            glonass_frequency_channel,
        )?;
        let local_code_model = match (sat.constellation, signal_band, signal_code) {
            (Constellation::Gps, SignalBand::L1, SignalCode::Ca) => {
                Some(LocalCodeModel::gps_l1_ca(sat.prn)?)
            }
            (Constellation::Gps, SignalBand::L5, SignalCode::L5I) => {
                Some(LocalCodeModel::gps_l5_i(sat.prn)?)
            }
            (Constellation::Gps, SignalBand::L5, SignalCode::L5Q) => {
                Some(LocalCodeModel::gps_l5_q(sat.prn)?)
            }
            (Constellation::Galileo, SignalBand::E1, SignalCode::E1B) => {
                Some(LocalCodeModel::galileo_e1_boc11(sat.prn)?)
            }
            (Constellation::Galileo, SignalBand::E5, SignalCode::E5a) => {
                Some(LocalCodeModel::galileo_e5a_i(sat.prn)?)
            }
            (Constellation::Galileo, SignalBand::E5, SignalCode::E5b) => {
                Some(LocalCodeModel::galileo_e5b_i(sat.prn)?)
            }
            (Constellation::Beidou, SignalBand::B1, SignalCode::B1I) => {
                Some(LocalCodeModel::beidou_b1i(sat.prn)?)
            }
            (Constellation::Beidou, SignalBand::B2, SignalCode::B2I) => {
                Some(LocalCodeModel::beidou_b2i(sat.prn)?)
            }
            (Constellation::Glonass, SignalBand::L1, SignalCode::Unknown) => {
                Some(LocalCodeModel::glonass_l1_st())
            }
            _ => None,
        };

        match (registry_entry, local_code_model) {
            (Some(registry_entry), Some(local_code_model)) => {
                Self::from_registry_entry(&registry_entry, local_code_model).map(Some)
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

    fn from_registry_entry(
        registry_entry: &bijux_gnss_core::api::SignalRegistryEntry,
        local_code_model: LocalCodeModel,
    ) -> Result<Self, SignalError> {
        let component =
            registry_entry.default_component().ok_or(SignalError::UnsupportedSignalDefinition {
                constellation: registry_entry.spec.constellation,
                signal_band: registry_entry.spec.band,
                signal_code: registry_entry.spec.code,
            })?;
        Self::new(
            registry_entry.spec.band,
            component.primary_code_rate_hz,
            component.primary_code_chips as usize,
            registry_entry.spec.carrier_hz,
            local_code_model,
        )
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
