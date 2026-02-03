#![allow(missing_docs)]

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct SatId {
    pub constellation: Constellation,
    pub prn: u8,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct SigId {
    pub sat: SatId,
    pub band: SignalBand,
    pub code: SignalCode,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum Constellation {
    Gps,
    Glonass,
    Galileo,
    Beidou,
    Unknown,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum SignalBand {
    L1,
    L2,
    L5,
    E1,
    E5,
    B1,
    B2,
    Unknown,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum SignalCode {
    Ca,
    Py,
    E1B,
    E1C,
    E5a,
    E5b,
    Unknown,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(transparent)]
pub struct FreqHz(pub f64);

impl FreqHz {
    pub const fn new(value: f64) -> Self {
        Self(value)
    }

    pub fn value(self) -> f64 {
        self.0
    }
}

pub const GPS_L1_CA_CARRIER_HZ: FreqHz = FreqHz::new(1_575_420_000.0);
pub const GPS_L2_PY_CARRIER_HZ: FreqHz = FreqHz::new(1_227_600_000.0);
pub const GPS_L5_CARRIER_HZ: FreqHz = FreqHz::new(1_176_450_000.0);
pub const GALILEO_E1_CARRIER_HZ: FreqHz = FreqHz::new(1_575_420_000.0);
pub const GALILEO_E5_CARRIER_HZ: FreqHz = FreqHz::new(1_176_450_000.0);
pub const GLONASS_L1_CARRIER_HZ: FreqHz = FreqHz::new(1_602_000_000.0);

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct SignalSpec {
    pub constellation: Constellation,
    pub band: SignalBand,
    pub code: SignalCode,
    pub code_rate_hz: f64,
    pub carrier_hz: FreqHz,
}

pub fn signal_spec_gps_l1_ca() -> SignalSpec {
    SignalSpec {
        constellation: Constellation::Gps,
        band: SignalBand::L1,
        code: SignalCode::Ca,
        code_rate_hz: 1_023_000.0,
        carrier_hz: GPS_L1_CA_CARRIER_HZ,
    }
}

pub fn signal_spec_gps_l2_py() -> SignalSpec {
    SignalSpec {
        constellation: Constellation::Gps,
        band: SignalBand::L2,
        code: SignalCode::Py,
        code_rate_hz: 10_230_000.0,
        carrier_hz: GPS_L2_PY_CARRIER_HZ,
    }
}
