//! Artifact contracts and versioned payloads.

use serde::{Deserialize, Serialize};

use crate::api::DiagnosticEvent;

/// Artifact header metadata included with every serialized artifact.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ArtifactHeaderV1 {
    /// Schema version for the artifact payload.
    pub schema_version: u32,
    /// Producer crate name.
    #[serde(default = "default_producer")]
    pub producer: String,
    /// Producer version string.
    #[serde(default = "default_producer_version")]
    pub producer_version: String,
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
    /// Acquisition selection explain artifacts.
    AcqExplain,
    /// Channel transition events.
    TrackTransition,
    /// Observation epoch decisions.
    ObsDecision,
    /// Support matrix report.
    SupportMatrix,
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

/// Versioned artifact wrapper (v1).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ArtifactV1<T> {
    /// Artifact header.
    pub header: ArtifactHeaderV1,
    /// Artifact payload.
    pub payload: T,
}

/// Payload validation trait for artifacts.
pub trait ArtifactPayloadValidate {
    /// Validate payload invariants.
    fn validate_payload(&self) -> Vec<DiagnosticEvent>;
}

/// Trait for validating artifact invariants.
pub trait ArtifactValidate {
    /// Validate the artifact and return diagnostics.
    fn validate(&self) -> Vec<DiagnosticEvent>;
}

/// Migration placeholder for V1 -> V2.
pub fn convert_v1_to_v2<T>(_artifact: &T) {
    // TODO: implement when V2 is defined.
}

/// Versioned artifact modules.
pub mod v1;

fn default_producer() -> String {
    "unknown".to_string()
}

fn default_producer_version() -> String {
    "unknown".to_string()
}
