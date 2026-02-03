//! Core GNSS pipeline contracts and error taxonomy.

#![deny(clippy::unwrap_used)]

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

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ObsEpoch {
    pub t_rx_s: f64,
    pub gps_week: Option<u16>,
    pub tow_s: Option<f64>,
    pub epoch_idx: u64,
    pub discontinuity: bool,
    pub role: ReceiverRole,
    pub sats: Vec<ObsSatellite>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BandLagEvent {
    pub sat: SatId,
    pub band: SignalBand,
    pub lag_epochs: u64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InterFrequencyAlignmentReport {
    pub total_events: usize,
    pub max_lag_epochs: u64,
    pub events: Vec<BandLagEvent>,
}

pub fn check_inter_frequency_alignment(epochs: &[ObsEpoch]) -> InterFrequencyAlignmentReport {
    use std::collections::BTreeMap;
    let mut last_seen: BTreeMap<(SatId, SignalBand), u64> = BTreeMap::new();
    let mut events = Vec::new();
    for epoch in epochs {
        let mut present: BTreeMap<SatId, Vec<SignalBand>> = BTreeMap::new();
        for sat in &epoch.sats {
            present
                .entry(sat.signal_id.sat)
                .or_default()
                .push(sat.signal_id.band);
            last_seen
                .entry((sat.signal_id.sat, sat.signal_id.band))
                .or_insert(epoch.epoch_idx);
        }
        for (sat, bands) in present {
            for ((seen_sat, seen_band), last_epoch) in last_seen.iter() {
                if *seen_sat != sat {
                    continue;
                }
                if bands.contains(seen_band) {
                    continue;
                }
                let lag = epoch.epoch_idx.saturating_sub(*last_epoch);
                if lag > 0 {
                    events.push(BandLagEvent {
                        sat,
                        band: *seen_band,
                        lag_epochs: lag,
                    });
                }
            }
        }
        for sat in &epoch.sats {
            last_seen.insert((sat.signal_id.sat, sat.signal_id.band), epoch.epoch_idx);
        }
    }
    let max_lag = events.iter().map(|e| e.lag_epochs).max().unwrap_or(0);
    InterFrequencyAlignmentReport {
        total_events: events.len(),
        max_lag_epochs: max_lag,
        events,
    }
}

pub fn validate_obs_epochs(epochs: &[ObsEpoch]) -> Result<(), String> {
    let mut last_t = None;
    for epoch in epochs {
        if let Some(prev) = last_t {
            if epoch.t_rx_s < prev {
                return Err("non-monotonic t_rx_s".to_string());
            }
        }
        last_t = Some(epoch.t_rx_s);
        if !epoch.t_rx_s.is_finite() {
            return Err("t_rx_s is not finite".to_string());
        }
        let mut seen = std::collections::BTreeSet::new();
        for sat in &epoch.sats {
            if !seen.insert(sat.signal_id) {
                return Err("duplicate signal_id within epoch".to_string());
            }
            if !sat.pseudorange_m.is_finite()
                || !sat.carrier_phase_cycles.is_finite()
                || !sat.doppler_hz.is_finite()
            {
                return Err("non-finite observable value".to_string());
            }
            if !sat.pseudorange_var_m2.is_finite()
                || !sat.carrier_phase_var_cycles2.is_finite()
                || !sat.doppler_var_hz2.is_finite()
            {
                return Err("non-finite variance".to_string());
            }
            if sat.pseudorange_var_m2 < 0.0
                || sat.carrier_phase_var_cycles2 < 0.0
                || sat.doppler_var_hz2 < 0.0
            {
                return Err("negative variance".to_string());
            }
        }
    }
    Ok(())
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub enum ReceiverRole {
    Base,
    Rover,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ObsStreamTag {
    pub role: ReceiverRole,
    pub source_id: String,
    pub sample_rate_hz: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NavResidual {
    pub sat: SatId,
    pub residual_m: f64,
    pub rejected: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InterSystemBias {
    pub constellation: Constellation,
    pub band: Option<SignalBand>,
    pub bias_s: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NavSolutionEpoch {
    pub epoch: Epoch,
    pub ecef_x_m: f64,
    pub ecef_y_m: f64,
    pub ecef_z_m: f64,
    pub latitude_deg: f64,
    pub longitude_deg: f64,
    pub altitude_m: f64,
    pub clock_bias_s: f64,
    pub pdop: f64,
    pub rms_m: f64,
    pub residuals: Vec<NavResidual>,
    pub isb: Vec<InterSystemBias>,
    pub sigma_h_m: Option<f64>,
    pub sigma_v_m: Option<f64>,
    pub ekf_innovation_rms: Option<f64>,
    pub ekf_condition_number: Option<f64>,
    pub ekf_whiteness_ratio: Option<f64>,
    pub ekf_predicted_variance: Option<f64>,
    pub ekf_observed_variance: Option<f64>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum NavHealthEvent {
    CovarianceSymmetrized,
    CovarianceClamped { min_eigenvalue: f64 },
    CovarianceDiverged { max_variance: f64 },
    InnovationRejected { reason: String },
    ZtdClamped { before_m: f64, after_m: f64 },
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct AmbiguityId {
    pub sig: SigId,
    pub signal: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum AmbiguityStatus {
    Unknown,
    Float,
    Fixed,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AmbiguityState {
    pub id: AmbiguityId,
    pub float_cycles: f64,
    pub variance: f64,
    pub status: AmbiguityStatus,
    pub last_update_epoch: u64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CarrierPhaseTerms {
    pub range_m: f64,
    pub receiver_clock_s: f64,
    pub sat_clock_s: f64,
    pub tropo_m: f64,
    pub iono_m: f64,
    pub wavelength_m: f64,
    pub ambiguity_cycles: f64,
}

pub fn carrier_phase_cycles(terms: &CarrierPhaseTerms) -> f64 {
    let corrected = terms.range_m
        + 299_792_458.0 * (terms.receiver_clock_s - terms.sat_clock_s)
        + terms.tropo_m
        - terms.iono_m;
    corrected / terms.wavelength_m + terms.ambiguity_cycles
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SingleDifference {
    pub sig: SigId,
    pub code_m: f64,
    pub phase_cycles: f64,
    pub doppler_hz: f64,
    pub ambiguity_rover: AmbiguityId,
    pub ambiguity_base: AmbiguityId,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DoubleDifference {
    pub ref_sig: SigId,
    pub sig: SigId,
    pub code_m: f64,
    pub phase_cycles: f64,
    pub doppler_hz: f64,
    pub canceled: Vec<AmbiguityId>,
}

pub fn geometry_free_phase_m(
    phase1_cycles: f64,
    phase2_cycles: f64,
    lambda1_m: f64,
    lambda2_m: f64,
) -> f64 {
    phase1_cycles * lambda1_m - phase2_cycles * lambda2_m
}

pub fn melbourne_wubbena_m(
    code1_m: f64,
    code2_m: f64,
    phase1_cycles: f64,
    phase2_cycles: f64,
    lambda1_m: f64,
    lambda2_m: f64,
) -> f64 {
    let phi1 = phase1_cycles * lambda1_m;
    let phi2 = phase2_cycles * lambda2_m;
    (phi1 - phi2) - (code1_m - code2_m)
}

#[derive(Debug, Error)]
#[error("input error: {message}")]
pub struct InputError {
    pub message: String,
}

#[derive(Debug, Error)]
#[error("config error: {message}")]
pub struct ConfigError {
    pub message: String,
}

#[derive(Debug, Error)]
#[error("signal error: {message}")]
pub struct SignalError {
    pub message: String,
}

#[derive(Debug, Error)]
#[error("acquisition error: {message}")]
pub struct AcqError {
    pub message: String,
}

#[derive(Debug, Error)]
#[error("tracking error: {message}")]
pub struct TrackError {
    pub message: String,
}

#[derive(Debug, Error)]
#[error("navigation error: {message}")]
pub struct NavError {
    pub message: String,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn leap_second_table_validates() {
        let table = LeapSeconds::default_table();
        table.validate().expect("leap second table valid");
        assert_eq!(table.latest_offset(), 18);
    }

    #[test]
    fn leap_second_offset_lookup() {
        let table = LeapSeconds::default_table();
        assert_eq!(table.offset_at_utc(0.0), 0);
        assert_eq!(table.offset_at_utc(362_793_600.0), 1);
        assert_eq!(table.offset_at_utc(1_483_228_800.0), 18);
    }

    #[test]
    fn utc_to_gps_uses_leap_offset() {
        let table = LeapSeconds::default_table();
        let utc = UtcTime {
            unix_s: 1_700_000_000.0,
        };
        let gps = utc_to_gps(utc, &table);
        let offset = table.offset_at_utc(utc.unix_s);
        let expected = utc.unix_s + offset as f64;
        let cycle = 604_800.0 * 1024.0;
        let diff = (gps.to_seconds() - expected).abs();
        assert!((diff % cycle) < 1e-6);
    }
}
