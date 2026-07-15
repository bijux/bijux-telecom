#![allow(missing_docs)]

use bijux_gnss_core::api::{gps_to_utc, utc_to_gps, GpsTime, LeapSeconds, TaiTime, UtcTime};
use serde::{Deserialize, Serialize};

const UTC_TAI_EPOCH_OFFSET_S: f64 = 19.0;
const WEEK_SECONDS: f64 = 604_800.0;
const BEIDOU_EPOCH_GPS_SECONDS: f64 = 1_356.0 * WEEK_SECONDS + 14.0;
const BEIDOU_GPS_OFFSET_S: f64 = 14.0;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum GnssTimeSystem {
    Gpst,
    Gst,
    Bdt,
    Glonass,
    Utc,
    Tai,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum TimeOffsetSource {
    LeapSecondTable,
    FixedSystemDefinition,
    BroadcastNavigationMessage,
    CompositeOffsetModel,
    ReferenceUtcDay,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TimeOffsetEvidence {
    pub from: GnssTimeSystem,
    pub to: GnssTimeSystem,
    pub offset_s: f64,
    pub source: TimeOffsetSource,
    pub source_detail: String,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TimeConversion<T> {
    pub time: T,
    pub offset: TimeOffsetEvidence,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct GalileoTime {
    pub week: u32,
    pub tow_s: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GalileoGpsTimeOffset {
    pub gst_minus_gps_s: f64,
    pub source: TimeOffsetSource,
    pub source_detail: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct BeidouTime {
    pub week: u32,
    pub tow_s: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct GlonassTime {
    pub day_index: i64,
    pub seconds_of_day: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GlonassUtcOffset {
    pub utc_su_minus_utc_s: f64,
    pub source: TimeOffsetSource,
    pub source_detail: String,
}

impl GalileoTime {
    pub fn from_seconds(total_seconds: f64) -> Self {
        let mut seconds = total_seconds.max(0.0);
        let week = (seconds / WEEK_SECONDS).floor() as u64;
        seconds -= week as f64 * WEEK_SECONDS;
        Self { week: week as u32, tow_s: seconds }
    }

    pub fn to_seconds(&self) -> f64 {
        self.week as f64 * WEEK_SECONDS + self.tow_s
    }
}

impl GalileoGpsTimeOffset {
    pub fn system_definition() -> Self {
        Self {
            gst_minus_gps_s: 0.0,
            source: TimeOffsetSource::FixedSystemDefinition,
            source_detail: "GST steered to GPST within broadcast offset tolerance".to_string(),
        }
    }

    pub fn broadcast(gst_minus_gps_s: f64, source_detail: impl Into<String>) -> Self {
        Self {
            gst_minus_gps_s,
            source: TimeOffsetSource::BroadcastNavigationMessage,
            source_detail: source_detail.into(),
        }
    }
}

impl BeidouTime {
    pub fn from_seconds(total_seconds: f64) -> Self {
        let mut seconds = total_seconds.max(0.0);
        let week = (seconds / WEEK_SECONDS).floor() as u64;
        seconds -= week as f64 * WEEK_SECONDS;
        Self { week: week as u32, tow_s: seconds }
    }

    pub fn to_seconds(&self) -> f64 {
        self.week as f64 * WEEK_SECONDS + self.tow_s
    }
}

impl GlonassUtcOffset {
    pub fn broadcast(utc_su_minus_utc_s: f64, source_detail: impl Into<String>) -> Self {
        Self {
            utc_su_minus_utc_s,
            source: TimeOffsetSource::BroadcastNavigationMessage,
            source_detail: source_detail.into(),
        }
    }
}

pub fn normalize_tow(tow_s: f64) -> f64 {
    if tow_s.is_finite() {
        tow_s.rem_euclid(604_800.0)
    } else {
        0.0
    }
}

pub fn gps_week_rollover(week: u16, reference_week: u32) -> u32 {
    const GPS_WEEK_ROLLOVER_CYCLE: i64 = 1024;
    const GPS_WEEK_ROLLOVER_HALF_CYCLE: i64 = GPS_WEEK_ROLLOVER_CYCLE / 2;

    let cycle = GPS_WEEK_ROLLOVER_CYCLE;
    let mut resolved_week = (reference_week / cycle as u32) as i64 * cycle + week as i64;
    let reference_week = reference_week as i64;

    if resolved_week - reference_week > GPS_WEEK_ROLLOVER_HALF_CYCLE {
        resolved_week -= cycle;
    } else if reference_week - resolved_week > GPS_WEEK_ROLLOVER_HALF_CYCLE {
        resolved_week += cycle;
    }

    resolved_week as u32
}

pub fn gps_time_from_utc(utc: UtcTime) -> GpsTime {
    utc_to_gps(utc, &LeapSeconds::default_table())
}

pub fn utc_to_gps_with_offset(utc: UtcTime, leap: &LeapSeconds) -> TimeConversion<GpsTime> {
    let offset_s = leap.offset_at_utc(utc.unix_s) as f64;
    TimeConversion {
        time: utc_to_gps(utc, leap),
        offset: TimeOffsetEvidence {
            from: GnssTimeSystem::Utc,
            to: GnssTimeSystem::Gpst,
            offset_s,
            source: TimeOffsetSource::LeapSecondTable,
            source_detail: "GPS-UTC from leap second table at UTC instant".to_string(),
        },
    }
}

pub fn gps_to_utc_with_offset(gps: GpsTime, leap: &LeapSeconds) -> TimeConversion<UtcTime> {
    let utc = gps_to_utc(gps, leap);
    let offset_s = -(leap.offset_at_utc(utc.unix_s) as f64);
    TimeConversion {
        time: utc,
        offset: TimeOffsetEvidence {
            from: GnssTimeSystem::Gpst,
            to: GnssTimeSystem::Utc,
            offset_s,
            source: TimeOffsetSource::LeapSecondTable,
            source_detail: "UTC-GPS from leap second table after GPS-to-UTC resolution".to_string(),
        },
    }
}

pub fn utc_to_tai_with_offset(utc: UtcTime, leap: &LeapSeconds) -> TimeConversion<TaiTime> {
    let offset_s = UTC_TAI_EPOCH_OFFSET_S + leap.offset_at_utc(utc.unix_s) as f64;
    TimeConversion {
        time: TaiTime { tai_s: utc.unix_s + offset_s },
        offset: TimeOffsetEvidence {
            from: GnssTimeSystem::Utc,
            to: GnssTimeSystem::Tai,
            offset_s,
            source: TimeOffsetSource::LeapSecondTable,
            source_detail: "TAI-UTC from 1972 TAI epoch offset plus leap second table".to_string(),
        },
    }
}

pub fn tai_to_utc_with_offset(tai: TaiTime, leap: &LeapSeconds) -> TimeConversion<UtcTime> {
    let mut offset_s = UTC_TAI_EPOCH_OFFSET_S + leap.latest_offset() as f64;
    let mut utc_s = tai.tai_s - offset_s;
    for _ in 0..3 {
        let adjusted_offset_s = UTC_TAI_EPOCH_OFFSET_S + leap.offset_at_utc(utc_s) as f64;
        if (adjusted_offset_s - offset_s).abs() <= f64::EPSILON {
            break;
        }
        offset_s = adjusted_offset_s;
        utc_s = tai.tai_s - offset_s;
    }
    TimeConversion {
        time: UtcTime { unix_s: utc_s },
        offset: TimeOffsetEvidence {
            from: GnssTimeSystem::Tai,
            to: GnssTimeSystem::Utc,
            offset_s: -offset_s,
            source: TimeOffsetSource::LeapSecondTable,
            source_detail: "UTC-TAI from inverse leap second table resolution".to_string(),
        },
    }
}

pub fn gps_to_galileo_with_offset(
    gps: GpsTime,
    offset: &GalileoGpsTimeOffset,
) -> TimeConversion<GalileoTime> {
    TimeConversion {
        time: GalileoTime::from_seconds(gps.to_seconds() + offset.gst_minus_gps_s),
        offset: TimeOffsetEvidence {
            from: GnssTimeSystem::Gpst,
            to: GnssTimeSystem::Gst,
            offset_s: offset.gst_minus_gps_s,
            source: offset.source,
            source_detail: offset.source_detail.clone(),
        },
    }
}

pub fn galileo_to_gps_with_offset(
    gst: GalileoTime,
    offset: &GalileoGpsTimeOffset,
) -> TimeConversion<GpsTime> {
    TimeConversion {
        time: GpsTime::from_seconds(gst.to_seconds() - offset.gst_minus_gps_s),
        offset: TimeOffsetEvidence {
            from: GnssTimeSystem::Gst,
            to: GnssTimeSystem::Gpst,
            offset_s: -offset.gst_minus_gps_s,
            source: offset.source,
            source_detail: offset.source_detail.clone(),
        },
    }
}

pub fn gps_to_beidou_with_offset(gps: GpsTime) -> TimeConversion<BeidouTime> {
    TimeConversion {
        time: BeidouTime::from_seconds(gps.to_seconds() - BEIDOU_EPOCH_GPS_SECONDS),
        offset: TimeOffsetEvidence {
            from: GnssTimeSystem::Gpst,
            to: GnssTimeSystem::Bdt,
            offset_s: -BEIDOU_GPS_OFFSET_S,
            source: TimeOffsetSource::FixedSystemDefinition,
            source_detail: "BDT epoch is 2006-01-01 UTC and BDT is GPST minus 14 seconds"
                .to_string(),
        },
    }
}

pub fn beidou_to_gps_with_offset(bdt: BeidouTime) -> TimeConversion<GpsTime> {
    TimeConversion {
        time: GpsTime::from_seconds(BEIDOU_EPOCH_GPS_SECONDS + bdt.to_seconds()),
        offset: TimeOffsetEvidence {
            from: GnssTimeSystem::Bdt,
            to: GnssTimeSystem::Gpst,
            offset_s: BEIDOU_GPS_OFFSET_S,
            source: TimeOffsetSource::FixedSystemDefinition,
            source_detail: "GPST is BDT plus 14 seconds at the BeiDou time epoch".to_string(),
        },
    }
}

pub fn utc_to_glonass_with_offset(
    utc: UtcTime,
    offset: &GlonassUtcOffset,
) -> TimeConversion<GlonassTime> {
    let glonass_seconds = utc.unix_s + 10_800.0 + offset.utc_su_minus_utc_s;
    let day_index = (glonass_seconds / 86_400.0).floor() as i64;
    let seconds_of_day = glonass_seconds - day_index as f64 * 86_400.0;
    TimeConversion {
        time: GlonassTime { day_index, seconds_of_day },
        offset: TimeOffsetEvidence {
            from: GnssTimeSystem::Utc,
            to: GnssTimeSystem::Glonass,
            offset_s: 10_800.0 + offset.utc_su_minus_utc_s,
            source: offset.source,
            source_detail: offset.source_detail.clone(),
        },
    }
}

pub fn glonass_to_utc_with_offset(
    glonass: GlonassTime,
    offset: &GlonassUtcOffset,
) -> TimeConversion<UtcTime> {
    TimeConversion {
        time: UtcTime {
            unix_s: glonass.day_index as f64 * 86_400.0 + glonass.seconds_of_day
                - 10_800.0
                - offset.utc_su_minus_utc_s,
        },
        offset: TimeOffsetEvidence {
            from: GnssTimeSystem::Glonass,
            to: GnssTimeSystem::Utc,
            offset_s: -10_800.0 - offset.utc_su_minus_utc_s,
            source: offset.source,
            source_detail: offset.source_detail.clone(),
        },
    }
}

pub fn gps_to_glonass_with_offset(
    gps: GpsTime,
    leap: &LeapSeconds,
    offset: &GlonassUtcOffset,
) -> TimeConversion<GlonassTime> {
    let utc = gps_to_utc(gps, leap);
    let glonass = utc_to_glonass_with_offset(utc, offset);
    TimeConversion {
        time: glonass.time,
        offset: TimeOffsetEvidence {
            from: GnssTimeSystem::Gpst,
            to: GnssTimeSystem::Glonass,
            offset_s: glonass.offset.offset_s - leap.offset_at_utc(utc.unix_s) as f64,
            source: TimeOffsetSource::CompositeOffsetModel,
            source_detail: format!(
                "GPS-UTC from leap second table and GLONASS UTC relation: {}",
                offset.source_detail
            ),
        },
    }
}

pub fn glonass_to_gps_with_offset(
    glonass: GlonassTime,
    leap: &LeapSeconds,
    offset: &GlonassUtcOffset,
) -> TimeConversion<GpsTime> {
    let utc = glonass_to_utc_with_offset(glonass, offset);
    let gps = utc_to_gps(utc.time, leap);
    TimeConversion {
        time: gps,
        offset: TimeOffsetEvidence {
            from: GnssTimeSystem::Glonass,
            to: GnssTimeSystem::Gpst,
            offset_s: utc.offset.offset_s + leap.offset_at_utc(utc.time.unix_s) as f64,
            source: TimeOffsetSource::CompositeOffsetModel,
            source_detail: format!(
                "GLONASS UTC relation and GPS-UTC from leap second table: {}",
                offset.source_detail
            ),
        },
    }
}

#[cfg(test)]
mod tests {
    use super::{
        beidou_to_gps_with_offset, galileo_to_gps_with_offset, glonass_to_gps_with_offset,
        glonass_to_utc_with_offset, gps_to_beidou_with_offset, gps_to_galileo_with_offset,
        gps_to_glonass_with_offset, gps_to_utc_with_offset, gps_week_rollover,
        tai_to_utc_with_offset, utc_to_glonass_with_offset, utc_to_gps_with_offset,
        utc_to_tai_with_offset, BeidouTime, GalileoGpsTimeOffset, GalileoTime, GlonassTime,
        GlonassUtcOffset, GnssTimeSystem, TimeOffsetSource,
    };
    use bijux_gnss_core::api::{GpsTime, LeapSeconds, TaiTime, UtcTime};

    #[test]
    fn gps_week_rollover_keeps_same_cycle_when_reference_matches() {
        assert_eq!(gps_week_rollover(161, 2209), 2209);
    }

    #[test]
    fn gps_week_rollover_uses_previous_cycle_near_lower_boundary() {
        assert_eq!(gps_week_rollover(1023, 1024), 1023);
    }

    #[test]
    fn gps_week_rollover_uses_next_cycle_near_upper_boundary() {
        assert_eq!(gps_week_rollover(0, 1023), 1024);
    }

    #[test]
    fn gps_week_rollover_prefers_nearest_week_across_multiple_cycles() {
        assert_eq!(gps_week_rollover(7, 3070), 3079);
    }

    #[test]
    fn utc_to_gps_reports_leap_second_offset_before_and_after_boundary() {
        let leap = LeapSeconds::default_table();
        let before = utc_to_gps_with_offset(UtcTime { unix_s: 1_483_228_799.0 }, &leap);
        let after = utc_to_gps_with_offset(UtcTime { unix_s: 1_483_228_800.0 }, &leap);

        assert_eq!(before.offset.from, GnssTimeSystem::Utc);
        assert_eq!(before.offset.to, GnssTimeSystem::Gpst);
        assert_eq!(before.offset.source, TimeOffsetSource::LeapSecondTable);
        assert_eq!(before.offset.offset_s, 17.0);
        assert_eq!(after.offset.offset_s, 18.0);
        assert!((after.time.to_seconds() - before.time.to_seconds() - 2.0).abs() < 1.0e-9);
    }

    #[test]
    fn gps_to_utc_reports_resolved_leap_second_offset() {
        let leap = LeapSeconds::default_table();
        let conversion = gps_to_utc_with_offset(GpsTime { week: 1930, tow_s: 18.0 }, &leap);

        assert_eq!(conversion.offset.from, GnssTimeSystem::Gpst);
        assert_eq!(conversion.offset.to, GnssTimeSystem::Utc);
        assert_eq!(conversion.offset.source, TimeOffsetSource::LeapSecondTable);
        assert_eq!(conversion.offset.offset_s, -18.0);
        assert_eq!(conversion.time.unix_s, 1_483_228_800.0);
    }

    #[test]
    fn utc_tai_conversion_reports_historical_leap_table_offsets() {
        let leap = LeapSeconds::default_table();
        let utc = UtcTime { unix_s: 1_483_228_800.0 };
        let tai = utc_to_tai_with_offset(utc, &leap);
        let round_trip = tai_to_utc_with_offset(tai.time, &leap);

        assert_eq!(tai.offset.from, GnssTimeSystem::Utc);
        assert_eq!(tai.offset.to, GnssTimeSystem::Tai);
        assert_eq!(tai.offset.source, TimeOffsetSource::LeapSecondTable);
        assert_eq!(tai.offset.offset_s, 37.0);
        assert_eq!(tai.time.tai_s, 1_483_228_837.0);
        assert_eq!(round_trip.offset.offset_s, -37.0);
        assert_eq!(round_trip.time, utc);
    }

    #[test]
    fn tai_to_utc_uses_epoch_offset_at_historical_instant() {
        let leap = LeapSeconds::default_table();
        let conversion = tai_to_utc_with_offset(TaiTime { tai_s: 315_964_819.0 }, &leap);

        assert_eq!(conversion.offset.offset_s, -19.0);
        assert_eq!(conversion.time.unix_s, 315_964_800.0);
    }

    #[test]
    fn galileo_system_definition_offset_preserves_gps_time() {
        let offset = GalileoGpsTimeOffset::system_definition();
        let gps = GpsTime { week: 2_300, tow_s: 604_799.5 };
        let gst = gps_to_galileo_with_offset(gps, &offset);
        let round_trip = galileo_to_gps_with_offset(gst.time, &offset);

        assert_eq!(gst.offset.source, TimeOffsetSource::FixedSystemDefinition);
        assert_eq!(gst.offset.from, GnssTimeSystem::Gpst);
        assert_eq!(gst.offset.to, GnssTimeSystem::Gst);
        assert_eq!(gst.offset.offset_s, 0.0);
        assert_eq!(gst.time, GalileoTime { week: 2_300, tow_s: 604_799.5 });
        assert_eq!(round_trip.time, gps);
    }

    #[test]
    fn galileo_broadcast_offset_crosses_week_boundary_with_evidence() {
        let offset = GalileoGpsTimeOffset::broadcast(0.75, "galileo inav word gst-gps offset");
        let gps = GpsTime { week: 2_300, tow_s: 604_799.5 };
        let gst = gps_to_galileo_with_offset(gps, &offset);
        let round_trip = galileo_to_gps_with_offset(gst.time, &offset);

        assert_eq!(gst.offset.source, TimeOffsetSource::BroadcastNavigationMessage);
        assert_eq!(gst.offset.offset_s, 0.75);
        assert_eq!(gst.time, GalileoTime { week: 2_301, tow_s: 0.25 });
        assert_eq!(round_trip.offset.from, GnssTimeSystem::Gst);
        assert_eq!(round_trip.offset.to, GnssTimeSystem::Gpst);
        assert_eq!(round_trip.offset.offset_s, -0.75);
        assert!((round_trip.time.to_seconds() - gps.to_seconds()).abs() < 1.0e-9);
    }

    #[test]
    fn beidou_epoch_maps_to_gps_week_with_fixed_offset_evidence() {
        let bdt = BeidouTime { week: 0, tow_s: 0.0 };
        let gps = beidou_to_gps_with_offset(bdt);
        let round_trip = gps_to_beidou_with_offset(gps.time);

        assert_eq!(gps.offset.from, GnssTimeSystem::Bdt);
        assert_eq!(gps.offset.to, GnssTimeSystem::Gpst);
        assert_eq!(gps.offset.source, TimeOffsetSource::FixedSystemDefinition);
        assert_eq!(gps.offset.offset_s, 14.0);
        assert_eq!(gps.time, GpsTime { week: 1_356, tow_s: 14.0 });
        assert_eq!(round_trip.offset.offset_s, -14.0);
        assert_eq!(round_trip.time, bdt);
    }

    #[test]
    fn beidou_conversion_crosses_week_boundary_without_losing_epoch_offset() {
        let gps = GpsTime { week: 1_357, tow_s: 10.0 };
        let bdt = gps_to_beidou_with_offset(gps);
        let round_trip = beidou_to_gps_with_offset(bdt.time);

        assert_eq!(bdt.time, BeidouTime { week: 0, tow_s: 604_796.0 });
        assert_eq!(bdt.offset.source, TimeOffsetSource::FixedSystemDefinition);
        assert_eq!(bdt.offset.offset_s, -14.0);
        assert!((round_trip.time.to_seconds() - gps.to_seconds()).abs() < 1.0e-9);
    }

    #[test]
    fn glonass_utc_conversion_reports_broadcast_utc_relation() {
        let offset = GlonassUtcOffset::broadcast(0.0, "glonass tau_c broadcast value");
        let utc = UtcTime { unix_s: 1_483_228_800.0 };
        let glonass = utc_to_glonass_with_offset(utc, &offset);
        let round_trip = glonass_to_utc_with_offset(glonass.time, &offset);

        assert_eq!(glonass.offset.from, GnssTimeSystem::Utc);
        assert_eq!(glonass.offset.to, GnssTimeSystem::Glonass);
        assert_eq!(glonass.offset.source, TimeOffsetSource::BroadcastNavigationMessage);
        assert_eq!(glonass.offset.offset_s, 10_800.0);
        assert_eq!(glonass.time.seconds_of_day, 10_800.0);
        assert_eq!(round_trip.offset.offset_s, -10_800.0);
        assert_eq!(round_trip.time, utc);
    }

    #[test]
    fn glonass_conversion_crosses_moscow_day_boundary() {
        let offset = GlonassUtcOffset::broadcast(0.0, "glonass tau_c broadcast value");
        let utc = UtcTime { unix_s: 1_483_308_000.0 };
        let glonass = utc_to_glonass_with_offset(utc, &offset);
        let round_trip = glonass_to_utc_with_offset(glonass.time, &offset);

        assert_eq!(glonass.time.seconds_of_day, 3_600.0);
        assert_eq!(round_trip.time, utc);
    }

    #[test]
    fn gps_glonass_conversion_combines_leap_and_utc_relation_evidence() {
        let leap = LeapSeconds::default_table();
        let offset = GlonassUtcOffset::broadcast(0.25, "glonass utc su relation");
        let gps = GpsTime { week: 1_930, tow_s: 18.0 };
        let glonass = gps_to_glonass_with_offset(gps, &leap, &offset);
        let round_trip = glonass_to_gps_with_offset(glonass.time, &leap, &offset);

        assert_eq!(glonass.offset.source, TimeOffsetSource::CompositeOffsetModel);
        assert_eq!(glonass.offset.offset_s, 10_782.25);
        assert_eq!(glonass.time, GlonassTime { day_index: 17_167, seconds_of_day: 10_800.25 });
        assert!((round_trip.time.to_seconds() - gps.to_seconds()).abs() < 1.0e-9);
    }
}
