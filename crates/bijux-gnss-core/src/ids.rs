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

/// Sort satellite IDs in-place by (constellation, prn).
pub fn sort_sat_ids(ids: &mut [SatId]) {
    ids.sort_by_key(|sat| (constellation_rank(sat.constellation), sat.prn));
}

/// Sort signal IDs in-place by (constellation, prn, band, code).
pub fn sort_sig_ids(ids: &mut [SigId]) {
    ids.sort_by_key(|sig| {
        (
            constellation_rank(sig.sat.constellation),
            sig.sat.prn,
            band_rank(sig.band),
            code_rank(sig.code),
        )
    });
}

/// Sort satellites within an observation epoch in-place.
pub fn sort_obs_sats(epoch: &mut crate::api::ObsEpoch) {
    epoch.sats.sort_by_key(|sat| {
        (
            constellation_rank(sat.signal_id.sat.constellation),
            sat.signal_id.sat.prn,
            band_rank(sat.signal_id.band),
            code_rank(sat.signal_id.code),
        )
    });
}

/// Format a satellite ID as `<Constellation>-<PRN>`.
pub fn format_sat(sat: SatId) -> String {
    format!("{:?}-{}", sat.constellation, sat.prn)
}

/// Convert GPS PRNs into satellite IDs.
pub fn prns_to_sats(prns: &[u8]) -> Vec<SatId> {
    prns.iter()
        .map(|&prn| SatId {
            constellation: Constellation::Gps,
            prn,
        })
        .collect()
}

fn constellation_rank(constellation: Constellation) -> u8 {
    match constellation {
        Constellation::Gps => 0,
        Constellation::Galileo => 1,
        Constellation::Glonass => 2,
        Constellation::Beidou => 3,
        Constellation::Unknown => 9,
    }
}

fn band_rank(band: SignalBand) -> u8 {
    match band {
        SignalBand::L1 => 0,
        SignalBand::L2 => 1,
        SignalBand::L5 => 2,
        SignalBand::E1 => 3,
        SignalBand::E5 => 4,
        SignalBand::B1 => 5,
        SignalBand::B2 => 6,
        SignalBand::Unknown => 9,
    }
}

fn code_rank(code: SignalCode) -> u8 {
    match code {
        SignalCode::Ca => 0,
        SignalCode::Py => 1,
        SignalCode::E1B => 2,
        SignalCode::E1C => 3,
        SignalCode::E5a => 4,
        SignalCode::E5b => 5,
        SignalCode::Unknown => 9,
    }
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
