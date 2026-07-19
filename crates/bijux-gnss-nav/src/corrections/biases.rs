#![allow(missing_docs)]
#![allow(dead_code)]

use std::collections::BTreeMap;

use bijux_gnss_core::api::{GpsTime, SigId};
use serde::{Deserialize, Serialize};

use crate::formats::bias_sinex::BiasSinexProvider;

#[derive(Debug, Clone)]
pub struct CodeBias {
    pub sig: SigId,
    pub bias_m: f64,
}

#[derive(Debug, Clone)]
pub struct PhaseBias {
    pub sig: SigId,
    pub bias_cycles: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PhaseBiasProvenance {
    ExternalProduct,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ResolvedPhaseBias {
    pub bias_cycles: f64,
    pub provenance: PhaseBiasProvenance,
}

pub trait CodeBiasProvider {
    fn code_bias_m(&self, sig: SigId) -> Option<f64>;

    fn code_bias_m_at(&self, sig: SigId, _time: Option<GpsTime>) -> Option<f64> {
        self.code_bias_m(sig)
    }

    fn differential_code_bias_m(&self, first: SigId, second: SigId) -> Option<f64> {
        Some(self.code_bias_m(first)? - self.code_bias_m(second)?)
    }

    fn differential_code_bias_m_at(
        &self,
        first: SigId,
        second: SigId,
        time: Option<GpsTime>,
    ) -> Option<f64> {
        Some(self.code_bias_m_at(first, time)? - self.code_bias_m_at(second, time)?)
    }

    fn iono_free_code_bias_m(
        &self,
        first: SigId,
        second: SigId,
        f1_hz: f64,
        f2_hz: f64,
    ) -> Option<f64> {
        iono_free_code_bias_from_values(
            self.code_bias_m(first)?,
            self.code_bias_m(second)?,
            f1_hz,
            f2_hz,
        )
    }

    fn iono_free_code_bias_m_at(
        &self,
        first: SigId,
        second: SigId,
        f1_hz: f64,
        f2_hz: f64,
        time: Option<GpsTime>,
    ) -> Option<f64> {
        iono_free_code_bias_from_values(
            self.code_bias_m_at(first, time)?,
            self.code_bias_m_at(second, time)?,
            f1_hz,
            f2_hz,
        )
    }
}

pub trait PhaseBiasProvider {
    fn phase_bias_cycles(&self, sig: SigId) -> Option<f64>;

    fn phase_bias_cycles_at(&self, sig: SigId, _time: Option<GpsTime>) -> Option<f64> {
        self.phase_bias_cycles(sig)
    }

    fn phase_bias_for_ambiguity_resolution(
        &self,
        _sig: SigId,
        _time: Option<GpsTime>,
    ) -> Option<ResolvedPhaseBias> {
        None
    }
}

#[derive(Debug, Clone, Default)]
pub struct SignalCodeBiases {
    biases_m: BTreeMap<SigId, f64>,
}

#[derive(Debug, Clone, Default)]
pub struct SignalPhaseBiases {
    biases_cycles: BTreeMap<SigId, f64>,
}

impl SignalCodeBiases {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn from_biases<I>(biases: I) -> Self
    where
        I: IntoIterator<Item = CodeBias>,
    {
        let mut by_signal = BTreeMap::new();
        for bias in biases {
            by_signal.insert(bias.sig, bias.bias_m);
        }
        Self { biases_m: by_signal }
    }

    pub fn insert(&mut self, sig: SigId, bias_m: f64) -> Option<f64> {
        self.biases_m.insert(sig, bias_m)
    }
}

impl SignalPhaseBiases {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn from_biases<I>(biases: I) -> Self
    where
        I: IntoIterator<Item = PhaseBias>,
    {
        let mut by_signal = BTreeMap::new();
        for bias in biases {
            by_signal.insert(bias.sig, bias.bias_cycles);
        }
        Self { biases_cycles: by_signal }
    }

    pub fn insert(&mut self, sig: SigId, bias_cycles: f64) -> Option<f64> {
        self.biases_cycles.insert(sig, bias_cycles)
    }
}

#[derive(Debug, Clone)]
pub struct ZeroBiases;

impl CodeBiasProvider for ZeroBiases {
    fn code_bias_m(&self, _sig: SigId) -> Option<f64> {
        Some(0.0)
    }
}

impl PhaseBiasProvider for ZeroBiases {
    fn phase_bias_cycles(&self, _sig: SigId) -> Option<f64> {
        Some(0.0)
    }
}

impl CodeBiasProvider for SignalCodeBiases {
    fn code_bias_m(&self, sig: SigId) -> Option<f64> {
        self.biases_m.get(&sig).copied()
    }
}

impl PhaseBiasProvider for SignalPhaseBiases {
    fn phase_bias_cycles(&self, sig: SigId) -> Option<f64> {
        self.biases_cycles.get(&sig).copied()
    }

    fn phase_bias_for_ambiguity_resolution(
        &self,
        sig: SigId,
        _time: Option<GpsTime>,
    ) -> Option<ResolvedPhaseBias> {
        Some(ResolvedPhaseBias {
            bias_cycles: self.biases_cycles.get(&sig).copied()?,
            provenance: PhaseBiasProvenance::ExternalProduct,
        })
    }
}

pub fn iono_free_code_bias_m(
    provider: &dyn CodeBiasProvider,
    first: SigId,
    second: SigId,
    f1_hz: f64,
    f2_hz: f64,
) -> Option<f64> {
    provider.iono_free_code_bias_m(first, second, f1_hz, f2_hz)
}

pub fn iono_free_code_bias_m_at(
    provider: &dyn CodeBiasProvider,
    first: SigId,
    second: SigId,
    f1_hz: f64,
    f2_hz: f64,
    time: Option<GpsTime>,
) -> Option<f64> {
    provider.iono_free_code_bias_m_at(first, second, f1_hz, f2_hz, time)
}

fn iono_free_code_bias_from_values(
    bias_1_m: f64,
    bias_2_m: f64,
    f1_hz: f64,
    f2_hz: f64,
) -> Option<f64> {
    if !f1_hz.is_finite() || !f2_hz.is_finite() || f1_hz <= 0.0 || f2_hz <= 0.0 {
        return None;
    }
    let f1_2 = f1_hz * f1_hz;
    let f2_2 = f2_hz * f2_hz;
    let denom = f1_2 - f2_2;
    if !denom.is_finite() || denom.abs() <= f64::EPSILON {
        return None;
    }

    let weight_1 = f1_2 / denom;
    let weight_2 = -f2_2 / denom;
    Some(weight_1 * bias_1_m + weight_2 * bias_2_m)
}

impl CodeBiasProvider for BiasSinexProvider {
    fn code_bias_m(&self, sig: SigId) -> Option<f64> {
        BiasSinexProvider::code_bias_m_at(self, sig, None)
    }

    fn code_bias_m_at(&self, sig: SigId, time: Option<GpsTime>) -> Option<f64> {
        BiasSinexProvider::code_bias_m_at(self, sig, time)
    }

    fn differential_code_bias_m(&self, first: SigId, second: SigId) -> Option<f64> {
        BiasSinexProvider::differential_code_bias_m_at(self, first, second, None)
    }

    fn differential_code_bias_m_at(
        &self,
        first: SigId,
        second: SigId,
        time: Option<GpsTime>,
    ) -> Option<f64> {
        BiasSinexProvider::differential_code_bias_m_at(self, first, second, time)
    }

    fn iono_free_code_bias_m(
        &self,
        first: SigId,
        second: SigId,
        f1_hz: f64,
        f2_hz: f64,
    ) -> Option<f64> {
        BiasSinexProvider::iono_free_code_bias_m_at(self, first, second, f1_hz, f2_hz, None)
    }

    fn iono_free_code_bias_m_at(
        &self,
        first: SigId,
        second: SigId,
        f1_hz: f64,
        f2_hz: f64,
        time: Option<GpsTime>,
    ) -> Option<f64> {
        BiasSinexProvider::iono_free_code_bias_m_at(self, first, second, f1_hz, f2_hz, time)
    }
}

#[cfg(test)]
mod tests {
    use bijux_gnss_core::api::{
        Constellation, GpsTime, SatId, SigId, SignalBand, SignalCode, GPS_L1_CA_CARRIER_HZ,
        GPS_L2_PY_CARRIER_HZ,
    };

    use super::{
        iono_free_code_bias_m, iono_free_code_bias_m_at, CodeBias, CodeBiasProvider, PhaseBias,
        PhaseBiasProvenance, PhaseBiasProvider, SignalCodeBiases, SignalPhaseBiases, ZeroBiases,
    };

    fn gps_signal(band: SignalBand, code: SignalCode) -> SigId {
        SigId { sat: SatId { constellation: Constellation::Gps, prn: 7 }, band, code }
    }

    #[test]
    fn signal_code_biases_lookup_by_signal_id() {
        let l1 = gps_signal(SignalBand::L1, SignalCode::Ca);
        let l2 = gps_signal(SignalBand::L2, SignalCode::Py);
        let biases = SignalCodeBiases::from_biases([
            CodeBias { sig: l1, bias_m: 1.25 },
            CodeBias { sig: l2, bias_m: -0.75 },
        ]);

        assert_eq!(biases.code_bias_m(l1), Some(1.25));
        assert_eq!(biases.code_bias_m(l2), Some(-0.75));
    }

    #[test]
    fn iono_free_code_bias_uses_signal_specific_weights() {
        let l1 = gps_signal(SignalBand::L1, SignalCode::Ca);
        let l2 = gps_signal(SignalBand::L2, SignalCode::Py);
        let biases = SignalCodeBiases::from_biases([
            CodeBias { sig: l1, bias_m: 3.0 },
            CodeBias { sig: l2, bias_m: -1.0 },
        ]);

        let combined_bias_m = iono_free_code_bias_m(
            &biases,
            l1,
            l2,
            GPS_L1_CA_CARRIER_HZ.value(),
            GPS_L2_PY_CARRIER_HZ.value(),
        )
        .expect("iono-free code bias");

        let f1_2 = GPS_L1_CA_CARRIER_HZ.value().powi(2);
        let f2_2 = GPS_L2_PY_CARRIER_HZ.value().powi(2);
        let expected_m = (f1_2 * 3.0 - -f2_2) / (f1_2 - f2_2);
        assert!((combined_bias_m - expected_m).abs() < 1.0e-12);
        assert!((combined_bias_m - 3.0).abs() > 1.0e-3);
    }

    #[test]
    fn time_scoped_queries_fall_back_to_signal_table_lookup() {
        let l1 = gps_signal(SignalBand::L1, SignalCode::Ca);
        let l2 = gps_signal(SignalBand::L2, SignalCode::Py);
        let biases = SignalCodeBiases::from_biases([
            CodeBias { sig: l1, bias_m: 2.0 },
            CodeBias { sig: l2, bias_m: -1.5 },
        ]);
        let time = Some(GpsTime { week: 2_123, tow_s: 12_345.0 });

        assert_eq!(biases.code_bias_m_at(l1, time), Some(2.0));
        assert_eq!(biases.differential_code_bias_m_at(l1, l2, time), Some(3.5));
        assert!(iono_free_code_bias_m_at(
            &biases,
            l1,
            l2,
            GPS_L1_CA_CARRIER_HZ.value(),
            GPS_L2_PY_CARRIER_HZ.value(),
            time,
        )
        .is_some());
    }

    #[test]
    fn signal_phase_biases_carry_ambiguity_resolution_provenance() {
        let l1 = gps_signal(SignalBand::L1, SignalCode::Ca);
        let biases = SignalPhaseBiases::from_biases([PhaseBias { sig: l1, bias_cycles: 0.25 }]);

        let resolved = biases
            .phase_bias_for_ambiguity_resolution(l1, Some(GpsTime { week: 2_123, tow_s: 10.0 }))
            .expect("phase bias provenance");

        assert_eq!(resolved.bias_cycles, 0.25);
        assert_eq!(resolved.provenance, PhaseBiasProvenance::ExternalProduct);
    }

    #[test]
    fn zero_phase_biases_do_not_claim_ambiguity_resolution_provenance() {
        let l1 = gps_signal(SignalBand::L1, SignalCode::Ca);
        let biases = ZeroBiases;

        assert_eq!(biases.phase_bias_cycles(l1), Some(0.0));
        assert!(biases.phase_bias_for_ambiguity_resolution(l1, None).is_none());
    }
}
