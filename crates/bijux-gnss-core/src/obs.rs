#![allow(missing_docs)]
#![allow(dead_code)]

use std::collections::BTreeMap;

use crate::api::{
    Chips, Constellation, Cycles, Epoch, Hertz, Meters, SampleTime, SatId, Seconds, SigId,
    SignalBand, SignalSpec,
};
use num_complex::Complex;
use serde::{Deserialize, Serialize};

pub type Sample = Complex<f32>;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SamplesFrame {
    pub t0: SampleTime,
    pub dt_s: Seconds,
    pub iq: Vec<Sample>,
}

impl SamplesFrame {
    pub fn new(t0: SampleTime, dt_s: Seconds, iq: Vec<Sample>) -> Self {
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
    pub carrier_hz: Hertz,
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
    pub carrier_hz: Hertz,
    pub code_rate_hz: Hertz,
    pub code_phase_samples: Chips,
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
    #[serde(default)]
    pub processing_ms: Option<f64>,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct LockFlags {
    pub code_lock: bool,
    pub carrier_lock: bool,
    pub bit_lock: bool,
    pub cycle_slip: bool,
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
    pub thermal_noise_m: Meters,
    pub tracking_jitter_m: Meters,
    pub multipath_proxy_m: Meters,
    pub clock_error_m: Meters,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ObsSatellite {
    pub signal_id: SigId,
    pub pseudorange_m: Meters,
    pub pseudorange_var_m2: f64,
    pub carrier_phase_cycles: Cycles,
    pub carrier_phase_var_cycles2: f64,
    pub doppler_hz: Hertz,
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
    pub t_rx_s: Seconds,
    pub gps_week: Option<u16>,
    pub tow_s: Option<Seconds>,
    pub epoch_idx: u64,
    pub discontinuity: bool,
    #[serde(default)]
    pub valid: bool,
    #[serde(default)]
    pub processing_ms: Option<f64>,
    pub role: ReceiverRole,
    pub sats: Vec<ObsSatellite>,
}

impl ObsEpoch {
    /// Validate basic physics sanity checks for this epoch.
    pub fn validate_physics(&self) -> Vec<crate::api::DiagnosticEvent> {
        let mut events = crate::api::check_obs_epoch_sanity(self);
        for sat in &self.sats {
            if !sat.pseudorange_m.0.is_finite()
                || !sat.carrier_phase_cycles.0.is_finite()
                || !sat.doppler_hz.0.is_finite()
            {
                events.push(crate::api::DiagnosticEvent::new(
                    crate::api::DiagnosticSeverity::Error,
                    "GNSS_OBS_NUMERIC_INVALID",
                    "obs contains NaN/Inf",
                ));
            }
        }
        events
    }
}

/// Event describing missing band observations over time.
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
    let mut last_t: Option<Seconds> = None;
    for epoch in epochs {
        if let Some(prev) = last_t {
            if epoch.t_rx_s.0 < prev.0 {
                return Err("non-monotonic t_rx_s".to_string());
            }
        }
        last_t = Some(epoch.t_rx_s);
        if !epoch.t_rx_s.0.is_finite() {
            return Err("t_rx_s is not finite".to_string());
        }
        let mut seen = std::collections::BTreeSet::new();
        for sat in &epoch.sats {
            if !seen.insert(sat.signal_id) {
                return Err("duplicate signal_id within epoch".to_string());
            }
            if !sat.pseudorange_m.0.is_finite()
                || !sat.carrier_phase_cycles.0.is_finite()
                || !sat.doppler_hz.0.is_finite()
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
pub(crate) struct ObsStreamTag {
    pub role: ReceiverRole,
    pub source_id: String,
    pub sample_rate_hz: Hertz,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NavResidual {
    pub sat: SatId,
    pub residual_m: Meters,
    pub rejected: bool,
    #[serde(default)]
    pub weight: Option<f64>,
    #[serde(default)]
    pub reject_reason: Option<MeasurementRejectReason>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InterSystemBias {
    pub constellation: Constellation,
    pub band: Option<SignalBand>,
    pub bias_s: Seconds,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NavSolutionEpoch {
    pub epoch: Epoch,
    pub t_rx_s: Seconds,
    pub ecef_x_m: Meters,
    pub ecef_y_m: Meters,
    pub ecef_z_m: Meters,
    pub latitude_deg: f64,
    pub longitude_deg: f64,
    pub altitude_m: Meters,
    pub clock_bias_s: Seconds,
    #[serde(default)]
    pub clock_drift_s_per_s: f64,
    pub pdop: f64,
    pub rms_m: Meters,
    pub status: SolutionStatus,
    #[serde(default)]
    pub quality: NavQualityFlag,
    #[serde(default)]
    pub validity: SolutionValidity,
    pub valid: bool,
    #[serde(default)]
    pub processing_ms: Option<f64>,
    pub residuals: Vec<NavResidual>,
    #[serde(default)]
    pub health: Vec<NavHealthEvent>,
    pub isb: Vec<InterSystemBias>,
    pub sigma_h_m: Option<Meters>,
    pub sigma_v_m: Option<Meters>,
    #[serde(default)]
    pub innovation_rms_m: Option<f64>,
    #[serde(default)]
    pub normalized_innovation_rms: Option<f64>,
    #[serde(default)]
    pub normalized_innovation_max: Option<f64>,
    pub ekf_innovation_rms: Option<f64>,
    pub ekf_condition_number: Option<f64>,
    pub ekf_whiteness_ratio: Option<f64>,
    pub ekf_predicted_variance: Option<f64>,
    pub ekf_observed_variance: Option<f64>,
    #[serde(default)]
    pub integrity_hpl_m: Option<f64>,
    #[serde(default)]
    pub integrity_vpl_m: Option<f64>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SolutionStatus {
    Invalid,
    Degraded,
    Coarse,
    Converged,
    Float,
    Fixed,
}

impl SolutionStatus {
    pub fn is_valid(self) -> bool {
        !matches!(self, SolutionStatus::Invalid)
    }

    pub fn quality_flag(self) -> NavQualityFlag {
        match self {
            SolutionStatus::Invalid => NavQualityFlag::NoFix,
            SolutionStatus::Degraded => NavQualityFlag::Degraded,
            SolutionStatus::Coarse | SolutionStatus::Converged | SolutionStatus::Float => {
                NavQualityFlag::Float
            }
            SolutionStatus::Fixed => NavQualityFlag::Fix,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum NavQualityFlag {
    NoFix,
    Float,
    Fix,
    Degraded,
}

impl Default for NavQualityFlag {
    fn default() -> Self {
        NavQualityFlag::NoFix
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SolutionValidity {
    Invalid,
    Coarse,
    Converging,
    Stable,
    Diverging,
}

impl Default for SolutionValidity {
    fn default() -> Self {
        SolutionValidity::Invalid
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum MeasurementRejectReason {
    Outlier,
    Geometry,
    CycleSlip,
    InvalidEphemeris,
    TimeInconsistency,
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
    pub float_cycles: Cycles,
    pub variance: f64,
    pub status: AmbiguityStatus,
    pub last_update_epoch: u64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub(crate) struct CarrierPhaseTerms {
    pub range_m: Meters,
    pub receiver_clock_s: Seconds,
    pub sat_clock_s: Seconds,
    pub tropo_m: Meters,
    pub iono_m: Meters,
    pub wavelength_m: Meters,
    pub ambiguity_cycles: Cycles,
}

pub(crate) fn carrier_phase_cycles(terms: &CarrierPhaseTerms) -> f64 {
    let corrected_m = terms.range_m.0
        + 299_792_458.0 * (terms.receiver_clock_s.0 - terms.sat_clock_s.0)
        + terms.tropo_m.0
        - terms.iono_m.0;
    corrected_m / terms.wavelength_m.0 + terms.ambiguity_cycles.0
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SingleDifference {
    pub sig: SigId,
    pub code_m: Meters,
    pub phase_cycles: Cycles,
    pub doppler_hz: Hertz,
    pub ambiguity_rover: AmbiguityId,
    pub ambiguity_base: AmbiguityId,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DoubleDifference {
    pub ref_sig: SigId,
    pub sig: SigId,
    pub code_m: Meters,
    pub phase_cycles: Cycles,
    pub doppler_hz: Hertz,
    pub canceled: Vec<AmbiguityId>,
}

pub(crate) fn geometry_free_phase_m(
    phase1_cycles: f64,
    phase2_cycles: f64,
    lambda1_m: f64,
    lambda2_m: f64,
) -> f64 {
    phase1_cycles * lambda1_m - phase2_cycles * lambda2_m
}

pub(crate) fn melbourne_wubbena_m(
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

// Errors moved to crate::error.

#[cfg(test)]
mod tests {
    use crate::api::{LeapSeconds, UtcTime};
    use crate::time::utc_to_gps;

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
