//! Artifact contracts and versioned payloads.

use serde::{Deserialize, Serialize};

use crate::{
    AcqResult, DiagnosticEvent, DiagnosticSeverity, NavSolutionEpoch, ObsEpoch, TrackEpoch,
};

/// Artifact header metadata included with every serialized artifact.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ArtifactHeader {
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
pub struct ArtifactCompatibility;

impl ArtifactCompatibility {
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
    pub header: ArtifactHeader,
    /// Observation epoch payload.
    pub epoch: ObsEpoch,
}

/// Versioned tracking epoch artifact (v1).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TrackEpochV1 {
    /// Artifact header.
    pub header: ArtifactHeader,
    /// Tracking epoch payload.
    pub epoch: TrackEpoch,
}

/// Versioned acquisition result artifact (v1).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AcqResultV1 {
    /// Artifact header.
    pub header: ArtifactHeader,
    /// Acquisition payload.
    pub result: AcqResult,
}

/// Versioned navigation solution artifact (v1).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NavSolutionEpochV1 {
    /// Artifact header.
    pub header: ArtifactHeader,
    /// Navigation solution payload.
    pub epoch: NavSolutionEpoch,
}

/// Check for NaN/Inf values inside an observation epoch.
pub fn check_obs_epoch_finite(epoch: &ObsEpoch) -> Vec<DiagnosticEvent> {
    let mut events = Vec::new();
    if !epoch.t_rx_s.is_finite() {
        events.push(DiagnosticEvent::new(
            DiagnosticSeverity::Error,
            "GNSS_NUMERIC_T_RX_INVALID",
            "t_rx_s is not finite",
        ));
    }
    for sat in &epoch.sats {
        if !sat.pseudorange_m.is_finite()
            || !sat.carrier_phase_cycles.is_finite()
            || !sat.doppler_hz.is_finite()
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
    if !epoch.ecef_x_m.is_finite() || !epoch.ecef_y_m.is_finite() || !epoch.ecef_z_m.is_finite() {
        events.push(DiagnosticEvent::new(
            DiagnosticSeverity::Error,
            "GNSS_NUMERIC_PVT_INVALID",
            "PVT position contains NaN/Inf",
        ));
    }
    events
}
