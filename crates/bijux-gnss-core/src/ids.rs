#![allow(missing_docs)]

use serde::{Deserialize, Serialize};

/// Strict satellite identity (constellation + PRN).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct SatId {
    pub constellation: Constellation,
    pub prn: u8,
}

/// One-based GLONASS orbital slot identifier.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct GlonassSlot(u8);

impl GlonassSlot {
    pub const MIN: u8 = 1;
    pub const MAX: u8 = 24;

    pub const fn new(value: u8) -> Option<Self> {
        if value >= Self::MIN && value <= Self::MAX {
            Some(Self(value))
        } else {
            None
        }
    }

    pub const fn value(self) -> u8 {
        self.0
    }
}

/// GLONASS FDMA carrier channel number.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct GlonassFrequencyChannel(i8);

impl GlonassFrequencyChannel {
    pub const MIN: i8 = -7;
    pub const MAX: i8 = 13;

    pub const fn new(value: i8) -> Option<Self> {
        if value >= Self::MIN && value <= Self::MAX {
            Some(Self(value))
        } else {
            None
        }
    }

    pub const fn value(self) -> i8 {
        self.0
    }
}

/// Concrete GLONASS L1 FDMA signal identity.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct GlonassL1FdmaSignal {
    pub slot: GlonassSlot,
    pub frequency_channel: GlonassFrequencyChannel,
}

impl GlonassL1FdmaSignal {
    pub fn sat_id(self) -> SatId {
        glonass_slot_sat(self.slot)
    }

    pub fn signal_id(self) -> SigId {
        SigId { sat: self.sat_id(), band: SignalBand::L1, code: SignalCode::Unknown }
    }

    pub fn carrier_hz(self) -> FreqHz {
        FreqHz::new(
            GLONASS_L1_CARRIER_HZ.value()
                + f64::from(self.frequency_channel.value()) * GLONASS_L1_CHANNEL_SPACING_HZ.value(),
        )
    }
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
    L2C,
    L5I,
    L5Q,
    Py,
    E1B,
    E1C,
    E5a,
    E5b,
    B1I,
    B2I,
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

/// Build a GLONASS satellite identifier from a validated orbital slot.
pub fn glonass_slot_sat(slot: GlonassSlot) -> SatId {
    SatId { constellation: Constellation::Glonass, prn: slot.value() }
}

/// Extract a validated GLONASS slot from one satellite ID.
pub fn glonass_slot_from_sat(sat: SatId) -> Option<GlonassSlot> {
    (sat.constellation == Constellation::Glonass).then(|| GlonassSlot::new(sat.prn)).flatten()
}

/// Convert GPS PRNs into satellite IDs.
pub fn prns_to_sats(prns: &[u8]) -> Vec<SatId> {
    prns.iter().map(|&prn| SatId { constellation: Constellation::Gps, prn }).collect()
}

/// Return the default civil tracking band used for a constellation when no explicit signal is given.
pub fn default_signal_band_for_constellation(constellation: Constellation) -> SignalBand {
    match constellation {
        Constellation::Gps => SignalBand::L1,
        Constellation::Galileo => SignalBand::E1,
        Constellation::Glonass => SignalBand::L1,
        Constellation::Beidou => SignalBand::B1,
        Constellation::Unknown => SignalBand::Unknown,
    }
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
        SignalCode::L2C => 1,
        SignalCode::L5I => 2,
        SignalCode::L5Q => 3,
        SignalCode::Py => 4,
        SignalCode::E1B => 5,
        SignalCode::E1C => 6,
        SignalCode::E5a => 7,
        SignalCode::E5b => 8,
        SignalCode::B1I => 9,
        SignalCode::B2I => 10,
        SignalCode::Unknown => 11,
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
pub const GPS_L2C_CARRIER_HZ: FreqHz = FreqHz::new(1_227_600_000.0);
pub const GPS_L2_PY_CARRIER_HZ: FreqHz = FreqHz::new(1_227_600_000.0);
pub const GPS_L5_CARRIER_HZ: FreqHz = FreqHz::new(1_176_450_000.0);
pub const GALILEO_E1_CARRIER_HZ: FreqHz = FreqHz::new(1_575_420_000.0);
pub const GALILEO_E5A_CARRIER_HZ: FreqHz = FreqHz::new(1_176_450_000.0);
pub const GALILEO_E5B_CARRIER_HZ: FreqHz = FreqHz::new(1_207_140_000.0);
pub const GALILEO_E5_CARRIER_HZ: FreqHz = GALILEO_E5A_CARRIER_HZ;
pub const GLONASS_L1_CHANNEL_SPACING_HZ: FreqHz = FreqHz::new(562_500.0);
pub const GLONASS_L1_CARRIER_HZ: FreqHz = FreqHz::new(1_602_000_000.0);
pub const BEIDOU_B1_CARRIER_HZ: FreqHz = FreqHz::new(1_561_098_000.0);
pub const BEIDOU_B2_CARRIER_HZ: FreqHz = FreqHz::new(1_207_140_000.0);

#[derive(Debug, Clone)]
pub struct SignalRegistryEntry {
    pub spec: SignalSpec,
    pub code_length: Option<u32>,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct SignalSpec {
    pub constellation: Constellation,
    pub band: SignalBand,
    pub code: SignalCode,
    pub code_rate_hz: f64,
    pub carrier_hz: FreqHz,
}
