

use num_complex::Complex;
use serde::{Deserialize, Serialize};
use thiserror::Error;

pub type Sample = Complex<f32>;

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
        SampleTime {
            sample_index,
            sample_rate_hz: self.sample_rate_hz,
        }
    }

    pub fn epoch_from_samples(&self, sample_index: u64) -> Epoch {
        let samples_per_epoch = self.samples_per_epoch().max(1);
        Epoch {
            index: sample_index / samples_per_epoch,
        }
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
pub struct ReceiverTime {
    pub rx_s: f64,
}

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
                LeapSecondEntry {
                    utc_unix_s: 362_793_600,
                    offset_s: 1,
                },
                LeapSecondEntry {
                    utc_unix_s: 394_329_600,
                    offset_s: 2,
                },
                LeapSecondEntry {
                    utc_unix_s: 425_865_600,
                    offset_s: 3,
                },
                LeapSecondEntry {
                    utc_unix_s: 489_024_000,
                    offset_s: 4,
                },
                LeapSecondEntry {
                    utc_unix_s: 567_993_600,
                    offset_s: 5,
                },
                LeapSecondEntry {
                    utc_unix_s: 631_152_000,
                    offset_s: 6,
                },
                LeapSecondEntry {
                    utc_unix_s: 662_688_000,
                    offset_s: 7,
                },
                LeapSecondEntry {
                    utc_unix_s: 709_948_800,
                    offset_s: 8,
                },
                LeapSecondEntry {
                    utc_unix_s: 741_484_800,
                    offset_s: 9,
                },
                LeapSecondEntry {
                    utc_unix_s: 773_020_800,
                    offset_s: 10,
                },
                LeapSecondEntry {
                    utc_unix_s: 820_454_400,
                    offset_s: 11,
                },
                LeapSecondEntry {
                    utc_unix_s: 867_715_200,
                    offset_s: 12,
                },
                LeapSecondEntry {
                    utc_unix_s: 915_148_800,
                    offset_s: 13,
                },
                LeapSecondEntry {
                    utc_unix_s: 1_136_073_600,
                    offset_s: 14,
                },
                LeapSecondEntry {
                    utc_unix_s: 1_230_768_000,
                    offset_s: 15,
                },
                LeapSecondEntry {
                    utc_unix_s: 1_341_100_800,
                    offset_s: 16,
                },
                LeapSecondEntry {
                    utc_unix_s: 1_435_708_800,
                    offset_s: 17,
                },
                LeapSecondEntry {
                    utc_unix_s: 1_483_228_800,
                    offset_s: 18,
                },
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
        Self {
            week: week_mod,
            tow_s: seconds,
        }
    }

    pub fn to_seconds(&self) -> f64 {
        self.week as f64 * 604_800.0 + self.tow_s
    }
}

pub fn gps_to_utc(gps: GpsTime, leap: &LeapSeconds) -> UtcTime {
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

pub fn utc_to_gps(utc: UtcTime, leap: &LeapSeconds) -> GpsTime {
    let offset = leap.offset_at_utc(utc.unix_s);
    GpsTime::from_seconds(utc.unix_s + offset as f64)
}

pub fn tai_to_utc(tai: TaiTime, leap: &LeapSeconds) -> UtcTime {
    let offset = leap.latest_offset();
    UtcTime {
        unix_s: tai.tai_s - (offset as f64 + 19.0),
    }
}

pub fn utc_to_tai(utc: UtcTime, leap: &LeapSeconds) -> TaiTime {
    let offset = leap.offset_at_utc(utc.unix_s);
    TaiTime {
        tai_s: utc.unix_s + offset as f64 + 19.0,
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SamplesFrame {
    pub t0: SampleTime,
    pub dt_s: f64,
    pub iq: Vec<Sample>,
}

impl SamplesFrame {
    pub fn new(t0: SampleTime, dt_s: f64, iq: Vec<Sample>) -> Self {
        Self { t0, dt_s, iq }
    }

    pub fn len(&self) -> usize {
        self.iq.len()
    }

    pub fn is_empty(&self) -> bool {
        self.iq.is_empty()
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct AcqRequest {
    pub sat: SatId,
    pub doppler_search_hz: i32,
    pub doppler_step_hz: i32,
    pub coherent_ms: u32,
    pub noncoherent: u32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AcqResult {
    pub sat: SatId,
    pub carrier_hz: f64,
    pub code_phase_samples: usize,
    pub peak: f32,
    pub second_peak: f32,
    pub mean: f32,
    pub peak_mean_ratio: f32,
    pub peak_second_ratio: f32,
    pub cn0_proxy: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TrackEpoch {
    pub epoch: Epoch,
    pub sample_index: u64,
    pub sat: SatId,
    pub prompt_i: f32,
    pub prompt_q: f32,
    pub carrier_hz: f64,
    pub code_rate_hz: f64,
    pub code_phase_samples: f64,
    pub lock: bool,
    pub cn0_dbhz: f64,
    pub pll_lock: bool,
    pub dll_lock: bool,
    pub fll_lock: bool,
    pub cycle_slip: bool,
    pub nav_bit_lock: bool,
    pub dll_err: f32,
    pub pll_err: f32,
    pub fll_err: f32,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct LockFlags {
    pub code_lock: bool,
    pub carrier_lock: bool,
    pub bit_lock: bool,
    pub cycle_slip: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum Constellation {
    Gps,
    Glonass,
    Galileo,
    Beidou,
    Unknown,
}

impl Default for Constellation {
    fn default() -> Self {
        Self::Gps
    }
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

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct SignalSpec {
    pub constellation: Constellation,
    pub band: SignalBand,
    pub code: SignalCode,
    pub code_rate_hz: f64,
    pub carrier_hz: FreqHz,
}

pub const GPS_L1_CA_CARRIER_HZ: FreqHz = FreqHz::new(1_575_420_000.0);
pub const GPS_L2_PY_CARRIER_HZ: FreqHz = FreqHz::new(1_227_600_000.0);
pub const GPS_L5_CARRIER_HZ: FreqHz = FreqHz::new(1_176_450_000.0);
pub const GALILEO_E1_CARRIER_HZ: FreqHz = FreqHz::new(1_575_420_000.0);
pub const GALILEO_E5_CARRIER_HZ: FreqHz = FreqHz::new(1_176_450_000.0);
pub const GLONASS_L1_CARRIER_HZ: FreqHz = FreqHz::new(1_602_000_000.0);

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
        code_rate_hz: 1_023_000.0,
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

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ObsMetadata {
    pub tracking_mode: String,
    pub integration_ms: u32,
    pub lock_quality: f64,
    pub smoothing_window: u32,
    pub smoothing_age: u32,
    pub smoothing_resets: u32,
    pub signal: SignalSpec,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MeasurementErrorModel {
    pub thermal_noise_m: f64,
    pub tracking_jitter_m: f64,
    pub multipath_proxy_m: f64,
    pub clock_error_m: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ObsSatellite {
    pub signal_id: SigId,
    pub pseudorange_m: f64,
    pub pseudorange_var_m2: f64,
    pub carrier_phase_cycles: f64,
    pub carrier_phase_var_cycles2: f64,
    pub doppler_hz: f64,
    pub doppler_var_hz2: f64,
    pub cn0_dbhz: f64,
    pub lock_flags: LockFlags,
    pub multipath_suspect: bool,
    pub elevation_deg: Option<f64>,
    pub azimuth_deg: Option<f64>,
    pub weight: Option<f64>,
    pub error_model: Option<MeasurementErrorModel>,
    pub metadata: ObsMetadata,
}

