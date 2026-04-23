#![allow(missing_docs)]
#![allow(dead_code)]

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct SampleTime {
    pub sample_index: u64,
    pub sample_rate_hz: f64,
}

impl SampleTime {
    pub fn seconds(&self) -> f64 {
        self.sample_index as f64 / self.sample_rate_hz
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct Epoch {
    pub index: u64,
}

impl Epoch {
    pub fn t0_seconds(&self) -> f64 {
        self.index as f64 * 0.001
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct SampleClock {
    pub sample_rate_hz: f64,
}

impl SampleClock {
    pub fn new(sample_rate_hz: f64) -> Self {
        Self { sample_rate_hz }
    }

    pub fn dt_s(&self) -> f64 {
        1.0 / self.sample_rate_hz
    }

    pub fn samples_per_epoch(&self) -> u64 {
        (self.sample_rate_hz * 0.001).round() as u64
    }

    pub fn time_from_samples(&self, sample_index: u64) -> SampleTime {
        SampleTime { sample_index, sample_rate_hz: self.sample_rate_hz }
    }

    pub fn epoch_from_samples(&self, sample_index: u64) -> Epoch {
        let samples_per_epoch = self.samples_per_epoch().max(1);
        Epoch { index: sample_index / samples_per_epoch }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct GpsTime {
    pub week: u16,
    pub tow_s: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct UtcTime {
    pub unix_s: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct TaiTime {
    pub tai_s: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub(crate) struct ReceiverTime {
    pub rx_s: f64,
}

/// Single leap second table entry.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct LeapSecondEntry {
    pub utc_unix_s: i64,
    pub offset_s: i32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LeapSeconds {
    pub entries: Vec<LeapSecondEntry>,
}

impl LeapSeconds {
    pub fn default_table() -> Self {
        Self {
            entries: vec![
                LeapSecondEntry { utc_unix_s: 362_793_600, offset_s: 1 },
                LeapSecondEntry { utc_unix_s: 394_329_600, offset_s: 2 },
                LeapSecondEntry { utc_unix_s: 425_865_600, offset_s: 3 },
                LeapSecondEntry { utc_unix_s: 489_024_000, offset_s: 4 },
                LeapSecondEntry { utc_unix_s: 567_993_600, offset_s: 5 },
                LeapSecondEntry { utc_unix_s: 631_152_000, offset_s: 6 },
                LeapSecondEntry { utc_unix_s: 662_688_000, offset_s: 7 },
                LeapSecondEntry { utc_unix_s: 709_948_800, offset_s: 8 },
                LeapSecondEntry { utc_unix_s: 741_484_800, offset_s: 9 },
                LeapSecondEntry { utc_unix_s: 773_020_800, offset_s: 10 },
                LeapSecondEntry { utc_unix_s: 820_454_400, offset_s: 11 },
                LeapSecondEntry { utc_unix_s: 867_715_200, offset_s: 12 },
                LeapSecondEntry { utc_unix_s: 915_148_800, offset_s: 13 },
                LeapSecondEntry { utc_unix_s: 1_136_073_600, offset_s: 14 },
                LeapSecondEntry { utc_unix_s: 1_230_768_000, offset_s: 15 },
                LeapSecondEntry { utc_unix_s: 1_341_100_800, offset_s: 16 },
                LeapSecondEntry { utc_unix_s: 1_435_708_800, offset_s: 17 },
                LeapSecondEntry { utc_unix_s: 1_483_228_800, offset_s: 18 },
            ],
        }
    }

    pub fn offset_at_utc(&self, utc_unix_s: f64) -> i32 {
        let mut offset = 0;
        for entry in &self.entries {
            if utc_unix_s >= entry.utc_unix_s as f64 {
                offset = entry.offset_s;
            } else {
                break;
            }
        }
        offset
    }

    pub fn latest_offset(&self) -> i32 {
        self.entries.last().map(|entry| entry.offset_s).unwrap_or(0)
    }

    pub fn validate(&self) -> Result<(), String> {
        if self.entries.is_empty() {
            return Err("leap second table is empty".to_string());
        }
        let mut prev_time = i64::MIN;
        let mut prev_offset = 0;
        for entry in &self.entries {
            if entry.utc_unix_s <= prev_time {
                return Err("leap second table not strictly increasing".to_string());
            }
            if entry.offset_s <= prev_offset {
                return Err("leap second offsets must be strictly increasing".to_string());
            }
            if entry.offset_s != prev_offset + 1 {
                return Err("leap second offsets must increment by 1".to_string());
            }
            prev_time = entry.utc_unix_s;
            prev_offset = entry.offset_s;
        }
        Ok(())
    }
}

impl GpsTime {
    pub fn from_seconds(total_seconds: f64) -> Self {
        let week_seconds = 604_800.0;
        let mut seconds = total_seconds.max(0.0);
        let week = (seconds / week_seconds).floor() as u64;
        seconds -= week as f64 * week_seconds;
        let week_mod = (week % 1024) as u16;
        Self { week: week_mod, tow_s: seconds }
    }

    pub fn to_seconds(&self) -> f64 {
        self.week as f64 * 604_800.0 + self.tow_s
    }
}

pub(crate) fn gps_to_utc(gps: GpsTime, leap: &LeapSeconds) -> UtcTime {
    let gps_s = gps.to_seconds();
    let mut offset = leap.latest_offset();
    let mut utc_s = gps_s - offset as f64;
    let adjusted = leap.offset_at_utc(utc_s);
    if adjusted != offset {
        offset = adjusted;
        utc_s = gps_s - offset as f64;
    }
    UtcTime { unix_s: utc_s }
}

pub(crate) fn utc_to_gps(utc: UtcTime, leap: &LeapSeconds) -> GpsTime {
    let offset = leap.offset_at_utc(utc.unix_s);
    GpsTime::from_seconds(utc.unix_s + offset as f64)
}

pub(crate) fn tai_to_utc(tai: TaiTime, leap: &LeapSeconds) -> UtcTime {
    let offset = leap.latest_offset();
    UtcTime { unix_s: tai.tai_s - (offset as f64 + 19.0) }
}

pub(crate) fn utc_to_tai(utc: UtcTime, leap: &LeapSeconds) -> TaiTime {
    let offset = leap.offset_at_utc(utc.unix_s);
    TaiTime { tai_s: utc.unix_s + offset as f64 + 19.0 }
}
