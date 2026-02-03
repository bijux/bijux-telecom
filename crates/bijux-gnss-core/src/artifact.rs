//! Artifact contracts and versioned payloads.

use serde::{Deserialize, Serialize};

use crate::{
    AcqResult, DiagnosticEvent, DiagnosticSeverity, NavSolutionEpoch, ObsEpoch, TrackEpoch,
};

/// Artifact header metadata included with every serialized artifact.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ArtifactHeaderV1 {
    /// Schema version for the artifact payload.
    pub schema_version: u32,
    /// Unix timestamp (ms) when the artifact was created.
    pub created_at_unix_ms: u128,
    /// Git SHA of the build (or "unknown").
    pub git_sha: String,
    /// Hash of the receiver config used to generate the artifact.
    pub config_hash: String,
    /// Optional dataset identifier.
    pub dataset_id: Option<String>,
    /// Toolchain identifier (e.g. rustc version).
    pub toolchain: String,
    /// Enabled feature flags.
    pub features: Vec<String>,
    /// Whether deterministic mode was enabled.
    pub deterministic: bool,
    /// Whether the git worktree was dirty at creation time.
    pub git_dirty: bool,
}

/// Artifact kind enumeration used for validation.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub enum ArtifactKind {
    /// Observation epochs.
    Obs,
    /// Tracking epochs.
    Track,
    /// Acquisition results.
    Acq,
    /// Navigation solution epochs.
    Nav,
}

/// Compatibility policy for artifacts.
pub struct ArtifactReadPolicy;

impl ArtifactReadPolicy {
    /// Latest supported schema version.
    pub const LATEST: u32 = 1;
    /// Oldest supported schema version.
    pub const MIN_SUPPORTED: u32 = 1;

    /// Check whether a schema version is supported.
    pub fn is_supported(version: u32) -> bool {
        (Self::MIN_SUPPORTED..=Self::LATEST).contains(&version)
    }
}

/// Versioned observation epoch artifact (v1).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ObsEpochV1 {
    /// Artifact header.
    pub header: ArtifactHeaderV1,
    /// Observation epoch payload.
    pub epoch: ObsEpoch,
}

/// Versioned tracking epoch artifact (v1).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TrackEpochV1 {
    /// Artifact header.
    pub header: ArtifactHeaderV1,
    /// Tracking epoch payload.
    pub epoch: TrackEpoch,
}

/// Versioned acquisition result artifact (v1).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AcqResultV1 {
    /// Artifact header.
    pub header: ArtifactHeaderV1,
    /// Acquisition payload.
    pub result: AcqResult,
}

/// Versioned navigation solution artifact (v1).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NavSolutionEpochV1 {
    /// Artifact header.
    pub header: ArtifactHeaderV1,
    /// Navigation solution payload.
    pub epoch: NavSolutionEpoch,
}

/// Trait for validating artifact invariants.
pub trait ArtifactValidate {
    /// Validate the artifact and return diagnostics.
    fn validate(&self) -> Vec<DiagnosticEvent>;
}

impl ArtifactValidate for ObsEpochV1 {
    fn validate(&self) -> Vec<DiagnosticEvent> {
        check_obs_epoch_finite(&self.epoch)
    }
}

impl ArtifactValidate for TrackEpochV1 {
    fn validate(&self) -> Vec<DiagnosticEvent> {
        let mut events = Vec::new();
        if !self.epoch.carrier_hz.0.is_finite() || !self.epoch.code_rate_hz.0.is_finite() {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Error,
                "GNSS_NUMERIC_TRACK_INVALID",
                "tracking epoch contains NaN/Inf",
            ));
        }
        events
    }
}

impl ArtifactValidate for AcqResultV1 {
    fn validate(&self) -> Vec<DiagnosticEvent> {
        let mut events = Vec::new();
        if !self.result.carrier_hz.0.is_finite() {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Error,
                "GNSS_NUMERIC_ACQ_INVALID",
                "acquisition result contains NaN/Inf",
            ));
        }
        events
    }
}

impl ArtifactValidate for NavSolutionEpochV1 {
    fn validate(&self) -> Vec<DiagnosticEvent> {
        check_nav_solution_finite(&self.epoch)
    }
}

/// Migration placeholder for V1 -> V2.
pub fn convert_v1_to_v2<T>(_artifact: &T) {
    // TODO: implement when V2 is defined.
}

/// Check for NaN/Inf values inside an observation epoch.
pub fn check_obs_epoch_finite(epoch: &ObsEpoch) -> Vec<DiagnosticEvent> {
    let mut events = Vec::new();
    if !epoch.t_rx_s.0.is_finite() {
        events.push(DiagnosticEvent::new(
            DiagnosticSeverity::Error,
            "GNSS_NUMERIC_T_RX_INVALID",
            "t_rx_s is not finite",
        ));
    }
    for sat in &epoch.sats {
        if !sat.pseudorange_m.0.is_finite()
            || !sat.carrier_phase_cycles.0.is_finite()
            || !sat.doppler_hz.0.is_finite()
        {
            events.push(
                DiagnosticEvent::new(
                    DiagnosticSeverity::Error,
                    "GNSS_NUMERIC_OBS_INVALID",
                    "observation contains NaN/Inf",
                )
                .with_context("sat".to_string(), format!("{}", sat.signal_id.sat.prn)),
            );
        }
    }
    events
}

/// Check for NaN/Inf values inside a navigation solution epoch.
pub fn check_nav_solution_finite(epoch: &NavSolutionEpoch) -> Vec<DiagnosticEvent> {
    let mut events = Vec::new();
    if !epoch.ecef_x_m.0.is_finite()
        || !epoch.ecef_y_m.0.is_finite()
        || !epoch.ecef_z_m.0.is_finite()
    {
        events.push(DiagnosticEvent::new(
            DiagnosticSeverity::Error,
            "GNSS_NUMERIC_PVT_INVALID",
            "PVT position contains NaN/Inf",
        ));
    }
    events
}
