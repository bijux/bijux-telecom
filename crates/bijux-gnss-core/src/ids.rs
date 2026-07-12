#![allow(missing_docs)]

use crate::units::{Cycles, Meters};
use serde::{Deserialize, Serialize};

const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;

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
        glonass_l1_carrier_hz(self.frequency_channel)
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

/// Return the default acquisition satellite catalog for one constellation.
pub fn default_acquisition_sats(constellation: Constellation) -> Vec<SatId> {
    match constellation {
        Constellation::Gps => sat_ids_for_prn_range(constellation, 1..=32),
        // The Galileo E1 code catalog currently spans PRNs 1 through 50 in the signal crate.
        Constellation::Galileo => sat_ids_for_prn_range(constellation, 1..=50),
        Constellation::Glonass => {
            sat_ids_for_prn_range(constellation, GlonassSlot::MIN..=GlonassSlot::MAX)
        }
        // The BeiDou B1I code assignments currently span PRNs 1 through 37 in the ICD-backed
        // signal catalog.
        Constellation::Beidou => sat_ids_for_prn_range(constellation, 1..=37),
        _ => Vec::new(),
    }
}

/// Convert GPS PRNs into satellite IDs.
pub fn prns_to_sats(prns: &[u8]) -> Vec<SatId> {
    prns.iter().map(|&prn| SatId { constellation: Constellation::Gps, prn }).collect()
}

fn sat_ids_for_prn_range(
    constellation: Constellation,
    prns: std::ops::RangeInclusive<u8>,
) -> Vec<SatId> {
    prns.map(|prn| SatId { constellation, prn }).collect()
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
        SignalCode::Py => 2,
        SignalCode::E1B => 3,
        SignalCode::E1C => 4,
        SignalCode::E5a => 5,
        SignalCode::E5b => 6,
        SignalCode::B1I => 7,
        SignalCode::B2I => 8,
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
pub const GPS_L2C_CARRIER_HZ: FreqHz = FreqHz::new(1_227_600_000.0);
pub const GPS_L2_PY_CARRIER_HZ: FreqHz = FreqHz::new(1_227_600_000.0);
pub const GPS_L5_CARRIER_HZ: FreqHz = FreqHz::new(1_176_450_000.0);
pub const GALILEO_E1_CARRIER_HZ: FreqHz = FreqHz::new(1_575_420_000.0);
pub const GALILEO_E5_CARRIER_HZ: FreqHz = FreqHz::new(1_176_450_000.0);
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

pub fn signal_spec_gps_l1_ca() -> SignalSpec {
    SignalSpec {
        constellation: Constellation::Gps,
        band: SignalBand::L1,
        code: SignalCode::Ca,
        code_rate_hz: 1_023_000.0,
        carrier_hz: GPS_L1_CA_CARRIER_HZ,
    }
}

pub fn signal_spec_gps_l2c() -> SignalSpec {
    SignalSpec {
        constellation: Constellation::Gps,
        band: SignalBand::L2,
        code: SignalCode::L2C,
        code_rate_hz: 511_500.0,
        carrier_hz: GPS_L2C_CARRIER_HZ,
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

pub fn signal_spec_gps_l5() -> SignalSpec {
    SignalSpec {
        constellation: Constellation::Gps,
        band: SignalBand::L5,
        code: SignalCode::Unknown,
        code_rate_hz: 10_230_000.0,
        carrier_hz: GPS_L5_CARRIER_HZ,
    }
}

pub fn signal_spec_galileo_e1b() -> SignalSpec {
    SignalSpec {
        constellation: Constellation::Galileo,
        band: SignalBand::E1,
        code: SignalCode::E1B,
        code_rate_hz: 1_023_000.0,
        carrier_hz: GALILEO_E1_CARRIER_HZ,
    }
}

pub fn signal_spec_galileo_e1c() -> SignalSpec {
    SignalSpec {
        constellation: Constellation::Galileo,
        band: SignalBand::E1,
        code: SignalCode::E1C,
        code_rate_hz: 1_023_000.0,
        carrier_hz: GALILEO_E1_CARRIER_HZ,
    }
}

pub fn signal_spec_galileo_e5a() -> SignalSpec {
    SignalSpec {
        constellation: Constellation::Galileo,
        band: SignalBand::E5,
        code: SignalCode::E5a,
        code_rate_hz: 10_230_000.0,
        carrier_hz: GALILEO_E5_CARRIER_HZ,
    }
}

pub fn signal_spec_glonass_l1(frequency_channel: GlonassFrequencyChannel) -> SignalSpec {
    SignalSpec {
        constellation: Constellation::Glonass,
        band: SignalBand::L1,
        code: SignalCode::Unknown,
        code_rate_hz: 511_000.0,
        carrier_hz: glonass_l1_carrier_hz(frequency_channel),
    }
}

pub fn signal_spec_beidou_b1i() -> SignalSpec {
    SignalSpec {
        constellation: Constellation::Beidou,
        band: SignalBand::B1,
        code: SignalCode::B1I,
        code_rate_hz: 2_046_000.0,
        carrier_hz: BEIDOU_B1_CARRIER_HZ,
    }
}

pub fn signal_spec_beidou_b2i() -> SignalSpec {
    SignalSpec {
        constellation: Constellation::Beidou,
        band: SignalBand::B2,
        code: SignalCode::B2I,
        code_rate_hz: 2_046_000.0,
        carrier_hz: BEIDOU_B2_CARRIER_HZ,
    }
}

pub fn carrier_wavelength_m(carrier_hz: FreqHz) -> Meters {
    Meters(SPEED_OF_LIGHT_MPS / carrier_hz.value())
}

pub fn signal_wavelength_m(signal: SignalSpec) -> Meters {
    carrier_wavelength_m(signal.carrier_hz)
}

pub fn signal_id_wavelength_m(signal_id: SigId) -> Option<Meters> {
    let signal = signal_registry(signal_id.sat.constellation, signal_id.band, signal_id.code)?.spec;
    Some(signal_wavelength_m(signal))
}

pub fn signal_cycles_to_meters(cycles: Cycles, signal: SignalSpec) -> Meters {
    Meters(cycles.0 * signal_wavelength_m(signal).0)
}

pub fn signal_id_cycles_to_meters(cycles: Cycles, signal_id: SigId) -> Option<Meters> {
    Some(Meters(cycles.0 * signal_id_wavelength_m(signal_id)?.0))
}

pub fn signal_meters_to_cycles(distance_m: Meters, signal: SignalSpec) -> Cycles {
    Cycles(distance_m.0 / signal_wavelength_m(signal).0)
}

pub fn signal_id_meters_to_cycles(distance_m: Meters, signal_id: SigId) -> Option<Cycles> {
    Some(Cycles(distance_m.0 / signal_id_wavelength_m(signal_id)?.0))
}

pub fn glonass_l1_carrier_hz(frequency_channel: GlonassFrequencyChannel) -> FreqHz {
    FreqHz::new(
        GLONASS_L1_CARRIER_HZ.value()
            + f64::from(frequency_channel.value()) * GLONASS_L1_CHANNEL_SPACING_HZ.value(),
    )
}

pub fn signal_registry(
    constellation: Constellation,
    band: SignalBand,
    code: SignalCode,
) -> Option<SignalRegistryEntry> {
    let spec =
        SignalSpec { constellation, band, code, code_rate_hz: 0.0, carrier_hz: FreqHz::new(0.0) };
    let (carrier_hz, code_rate_hz, code_length) = match (constellation, band, code) {
        (Constellation::Gps, SignalBand::L1, SignalCode::Ca) => {
            (GPS_L1_CA_CARRIER_HZ, 1_023_000.0, Some(1023))
        }
        (Constellation::Gps, SignalBand::L2, SignalCode::L2C) => {
            (GPS_L2C_CARRIER_HZ, 511_500.0, Some(10230))
        }
        (Constellation::Gps, SignalBand::L2, SignalCode::Py) => {
            (GPS_L2_PY_CARRIER_HZ, 10_230_000.0, None)
        }
        (Constellation::Gps, SignalBand::L5, SignalCode::Unknown) => {
            (GPS_L5_CARRIER_HZ, 10_230_000.0, None)
        }
        (Constellation::Galileo, SignalBand::E1, SignalCode::E1B) => {
            (GALILEO_E1_CARRIER_HZ, 1_023_000.0, Some(4092))
        }
        (Constellation::Galileo, SignalBand::E1, SignalCode::E1C) => {
            (GALILEO_E1_CARRIER_HZ, 1_023_000.0, Some(4092))
        }
        (Constellation::Galileo, SignalBand::E5, SignalCode::E5a) => {
            (GALILEO_E5_CARRIER_HZ, 10_230_000.0, Some(10230))
        }
        (Constellation::Glonass, SignalBand::L1, SignalCode::Unknown) => {
            (GLONASS_L1_CARRIER_HZ, 511_000.0, Some(511))
        }
        (Constellation::Beidou, SignalBand::B1, SignalCode::B1I) => {
            (BEIDOU_B1_CARRIER_HZ, 2_046_000.0, Some(2046))
        }
        (Constellation::Beidou, SignalBand::B2, SignalCode::B2I) => {
            (BEIDOU_B2_CARRIER_HZ, 2_046_000.0, Some(2046))
        }
        _ => return None,
    };
    Some(SignalRegistryEntry { spec: SignalSpec { carrier_hz, code_rate_hz, ..spec }, code_length })
}

pub fn default_acquisition_signal(constellation: Constellation) -> Option<SignalRegistryEntry> {
    match constellation {
        Constellation::Gps => signal_registry(Constellation::Gps, SignalBand::L1, SignalCode::Ca),
        Constellation::Galileo => {
            signal_registry(Constellation::Galileo, SignalBand::E1, SignalCode::E1B)
        }
        Constellation::Glonass => {
            signal_registry(Constellation::Glonass, SignalBand::L1, SignalCode::Unknown)
        }
        Constellation::Beidou => {
            signal_registry(Constellation::Beidou, SignalBand::B1, SignalCode::B1I)
        }
        Constellation::Unknown => None,
    }
}

#[cfg(test)]
mod tests {
    use super::{
        carrier_wavelength_m, signal_cycles_to_meters, signal_id_cycles_to_meters,
        signal_id_meters_to_cycles, signal_id_wavelength_m, signal_meters_to_cycles,
        signal_registry, signal_spec_beidou_b2i, signal_spec_gps_l2_py, signal_spec_gps_l2c,
        signal_wavelength_m, Constellation, FreqHz, SatId, SigId, SignalBand, SignalCode,
    };
    use crate::units::{Cycles, Meters};

    #[test]
    fn gps_l2c_signal_spec_matches_civil_code_timing() {
        let signal = signal_spec_gps_l2c();

        assert_eq!(signal.code, SignalCode::L2C);
        assert!((signal.code_rate_hz - 511_500.0).abs() <= f64::EPSILON);
        let registry = signal_registry(Constellation::Gps, SignalBand::L2, SignalCode::L2C)
            .expect("GPS L2C registry entry");
        assert_eq!(registry.code_length, Some(10230));
        assert_eq!(registry.spec, signal);
    }

    #[test]
    fn gps_l2_py_signal_spec_preserves_legacy_rate_without_short_code_length() {
        let signal = signal_spec_gps_l2_py();

        assert_eq!(signal.code, SignalCode::Py);
        assert!((signal.code_rate_hz - 10_230_000.0).abs() <= f64::EPSILON);
        let registry = signal_registry(Constellation::Gps, SignalBand::L2, SignalCode::Py)
            .expect("GPS L2 P(Y) registry entry");
        assert_eq!(registry.code_length, None);
        assert_eq!(registry.spec, signal);
    }

    #[test]
    fn beidou_b2i_signal_spec_matches_open_service_timing() {
        let signal = signal_spec_beidou_b2i();

        assert_eq!(signal.code, SignalCode::B2I);
        assert!((signal.code_rate_hz - 2_046_000.0).abs() <= f64::EPSILON);
        let registry = signal_registry(Constellation::Beidou, SignalBand::B2, SignalCode::B2I)
            .expect("BeiDou B2I registry entry");
        assert_eq!(registry.code_length, Some(2046));
        assert_eq!(registry.spec, signal);
    }

    #[test]
    fn carrier_wavelength_matches_speed_of_light_ratio() {
        let wavelength = carrier_wavelength_m(FreqHz::new(1_575_420_000.0));

        assert!((wavelength.0 - 0.190_293_672_798_364_87).abs() < 1.0e-15);
    }

    #[test]
    fn signal_wavelength_uses_signal_carrier_frequency() {
        let signal = signal_spec_beidou_b2i();
        let wavelength = signal_wavelength_m(signal);

        assert!((wavelength.0 - 0.248_349_369_584_306_9).abs() < 1.0e-15);
    }

    #[test]
    fn signal_id_wavelength_resolves_registered_signal_identity() {
        let wavelength = signal_id_wavelength_m(SigId {
            sat: SatId { constellation: Constellation::Beidou, prn: 11 },
            band: SignalBand::B2,
            code: SignalCode::B2I,
        })
        .expect("BeiDou B2I wavelength");

        assert!((wavelength.0 - 0.248_349_369_584_306_9).abs() < 1.0e-15);
    }

    #[test]
    fn signal_phase_conversion_helpers_round_trip_registered_signal() {
        let signal_id = SigId {
            sat: SatId { constellation: Constellation::Gps, prn: 9 },
            band: SignalBand::L2,
            code: SignalCode::Py,
        };
        let phase_m = signal_id_cycles_to_meters(Cycles(12_345.25), signal_id)
            .expect("GPS L2 P(Y) phase meters");
        let phase_cycles =
            signal_id_meters_to_cycles(phase_m, signal_id).expect("GPS L2 P(Y) phase cycles");

        assert!((phase_cycles.0 - 12_345.25).abs() < 1.0e-12);
    }

    #[test]
    fn signal_phase_conversion_helpers_round_trip_signal_spec() {
        let signal = signal_spec_beidou_b2i();
        let phase_m = signal_cycles_to_meters(Cycles(512.5), signal);
        let phase_cycles = signal_meters_to_cycles(Meters(phase_m.0), signal);

        assert!((phase_cycles.0 - 512.5).abs() < 1.0e-12);
    }
}
