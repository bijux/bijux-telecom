use crate::models::atmosphere::KlobucharCoefficients;
use crate::orbits::beidou::BeidouBroadcastNavigationData;
use crate::orbits::galileo::GalileoBroadcastNavigationData;
use crate::orbits::glonass::GlonassBroadcastNavigationFrame;
use crate::orbits::gps::GpsEphemeris;

#[derive(Debug, Clone, PartialEq)]
pub(super) struct RinexNavHeader {
    pub(super) version: f64,
    pub(super) is_mixed: bool,
    pub(super) klobuchar: Option<KlobucharCoefficients>,
    pub(super) time_system_corrections: Vec<RinexNavigationTimeSystemCorrection>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct RinexNavigationTimeSystemCorrection {
    pub code: String,
    pub a0_s: f64,
    pub a1_s_per_s: f64,
    pub reference_time_s: u32,
    pub reference_week: u32,
    pub provider: Option<String>,
    pub utc_id: Option<String>,
}

#[derive(Debug, Clone)]
pub struct RinexBroadcastNavigationDataset {
    pub version: f64,
    pub klobuchar: Option<KlobucharCoefficients>,
    pub time_system_corrections: Vec<RinexNavigationTimeSystemCorrection>,
    pub gps: Vec<GpsEphemeris>,
    pub galileo: Vec<GalileoBroadcastNavigationData>,
    pub beidou: Vec<BeidouBroadcastNavigationData>,
    pub glonass: Vec<GlonassBroadcastNavigationFrame>,
}
