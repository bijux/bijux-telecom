use crate::api::{ReceiverSampleTrace, SampleTime, SatId, Seconds};
use crate::observation_quality::ObsSatellite;
use num_complex::Complex;
use serde::{Deserialize, Serialize};

use super::{Hertz, OBSERVATION_DOWNSTREAM_PROFILE_VERSION};

fn default_observation_downstream_profile_version() -> u32 {
    OBSERVATION_DOWNSTREAM_PROFILE_VERSION
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ObsEpochManifest {
    pub version: u32,
    pub artifact_id: String,
    pub epoch_id: String,
    pub source_epoch_idx: u64,
    pub source_sample_index: u64,
    #[serde(default)]
    pub source_time: ReceiverSampleTrace,
    pub decision: ObservationEpochDecision,
    #[serde(default = "default_observation_downstream_profile_version")]
    pub downstream_profile_version: u32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SatObservationDecision {
    pub sat: SatId,
    pub status: ObservationStatus,
    pub reasons: Vec<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ObsDecisionArtifact {
    pub artifact_id: String,
    pub epoch_idx: u64,
    pub decision: ObservationEpochDecision,
    pub reasons: Vec<String>,
    pub accepted_sats: Vec<SatObservationDecision>,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
pub enum ObservationStatus {
    Accepted,
    Missing,
    Weak,
    Inconsistent,
    Rejected,
}

impl Default for ObservationStatus {
    fn default() -> Self {
        Self::Accepted
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
pub enum ObservationSupportClass {
    Supported,
    Degraded,
    Unsupported,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
pub enum ObservationUncertaintyClass {
    Low,
    Medium,
    High,
    Unknown,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
pub enum ObservationEpochDecision {
    Accepted,
    Rejected,
}

impl Default for ObservationEpochDecision {
    fn default() -> Self {
        Self::Accepted
    }
}

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

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ObsEpoch {
    pub t_rx_s: Seconds,
    #[serde(default)]
    pub source_time: ReceiverSampleTrace,
    pub gps_week: Option<u32>,
    pub tow_s: Option<Seconds>,
    pub epoch_idx: u64,
    pub discontinuity: bool,
    #[serde(default)]
    pub valid: bool,
    #[serde(default)]
    pub processing_ms: Option<f64>,
    pub role: ReceiverRole,
    pub sats: Vec<ObsSatellite>,
    #[serde(default)]
    pub decision: ObservationEpochDecision,
    #[serde(default)]
    pub decision_reason: Option<String>,
    #[serde(default)]
    pub manifest: Option<ObsEpochManifest>,
}

pub fn obs_epoch_stability_key(epoch: &ObsEpoch) -> String {
    let mut sat_keys = epoch
        .sats
        .iter()
        .map(|sat| {
            let timing_key = sat
                .timing
                .map(|timing| {
                    format!(
                        ":{:.12}:{}:{:.12}",
                        timing.signal_travel_time_s.0,
                        timing.transmit_gps_time.week,
                        timing.transmit_gps_time.tow_s
                    )
                })
                .unwrap_or_default();
            format!(
                "{:?}-{:02}:{:?}:{:?}:{:.3}:{:.6}:{:.3}:{}:{:.6}:{}:{}:{}:{}{}",
                sat.signal_id.sat.constellation,
                sat.signal_id.sat.prn,
                sat.signal_id.band,
                sat.signal_id.code,
                sat.pseudorange_m.0,
                sat.carrier_phase_cycles.0,
                sat.doppler_hz.0,
                sat.metadata.doppler_model,
                sat.cn0_dbhz,
                sat.metadata.carrier_phase_continuity,
                sat.metadata.carrier_phase_arc_start_epoch_idx,
                sat.metadata.carrier_phase_arc_start_sample_index,
                sat.metadata
                    .carrier_phase_arc
                    .as_ref()
                    .map(|arc| arc.id.as_str())
                    .unwrap_or("no-carrier-phase-arc"),
                timing_key
            )
        })
        .collect::<Vec<_>>();
    sat_keys.sort();
    format!(
        "epoch:{}|sample:{}|t:{:.9}|decision:{:?}|sats:{}",
        epoch.epoch_idx,
        epoch.source_time.sample_index,
        epoch.t_rx_s.0,
        epoch.decision,
        sat_keys.join(";")
    )
}

impl ObsEpoch {
    pub fn gps_time(&self) -> Option<crate::api::GpsTime> {
        Some(crate::api::GpsTime { week: self.gps_week?, tow_s: self.tow_s?.0 })
    }

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
