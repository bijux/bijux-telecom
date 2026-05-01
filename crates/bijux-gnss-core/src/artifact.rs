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
pub mod v1 {
    /// Versioned v1 artifact types.
    use super::{ArtifactPayloadValidate, ArtifactV1};
    use crate::api::{DiagnosticEvent, DiagnosticSeverity};

    pub mod acq {
        use super::*;
        use crate::api::AcqResult;

        /// Acquisition result artifact v1.
        pub type AcqResultV1 = ArtifactV1<AcqResult>;

        impl ArtifactPayloadValidate for AcqResult {
            fn validate_payload(&self) -> Vec<DiagnosticEvent> {
                let mut events = Vec::new();
                if !self.carrier_hz.0.is_finite() {
                    events.push(DiagnosticEvent::new(
                        DiagnosticSeverity::Error,
                        "GNSS_NUMERIC_ACQ_INVALID",
                        "acquisition result contains NaN/Inf",
                    ));
                }
                events
            }
        }
    }

    pub mod track {
        use super::*;
        use crate::api::TrackEpoch;

        /// Tracking epoch artifact v1.
        pub type TrackEpochV1 = ArtifactV1<TrackEpoch>;

        impl ArtifactPayloadValidate for TrackEpoch {
            fn validate_payload(&self) -> Vec<DiagnosticEvent> {
                let mut events = Vec::new();
                if !self.carrier_hz.0.is_finite() || !self.code_rate_hz.0.is_finite() {
                    events.push(DiagnosticEvent::new(
                        DiagnosticSeverity::Error,
                        "GNSS_NUMERIC_TRACK_INVALID",
                        "tracking epoch contains NaN/Inf",
                    ));
                }
                events
            }
        }
    }

    pub mod acq_explain {
        use super::*;
        use crate::api::AcqExplain;

        /// Acquisition explain artifact v1.
        pub type AcqExplainV1 = ArtifactV1<AcqExplain>;

        impl ArtifactPayloadValidate for AcqExplain {
            fn validate_payload(&self) -> Vec<DiagnosticEvent> {
                let mut events = Vec::new();
                if self.sat.prn == 0 {
                    events.push(DiagnosticEvent::new(
                        DiagnosticSeverity::Error,
                        "GNSS_ACQ_EXPLAIN_INVALID",
                        "explain record has invalid sat",
                    ));
                }
                events
            }
        }
    }

    pub mod track_transition {
        use super::*;
        use crate::api::TrackTransition;

        /// Track transition artifact v1.
        pub type TrackTransitionV1 = ArtifactV1<TrackTransition>;

        impl ArtifactPayloadValidate for TrackTransition {
            fn validate_payload(&self) -> Vec<DiagnosticEvent> {
                let mut events = Vec::new();
                if self.sample_index == 0 && self.epoch_idx == 0 {
                    events.push(DiagnosticEvent::new(
                        DiagnosticSeverity::Warning,
                        "GNSS_TRACK_TRANSITION_UNINITIALIZED",
                        "transition uses zero epoch/sample index",
                    ));
                }
                events
            }
        }
    }

    pub mod obs_decision {
        use super::*;
        use crate::api::ObsDecisionArtifact;

        /// Observation decision artifact v1.
        pub type ObsDecisionV1 = ArtifactV1<ObsDecisionArtifact>;

        impl ArtifactPayloadValidate for ObsDecisionArtifact {
            fn validate_payload(&self) -> Vec<DiagnosticEvent> {
                let mut events = Vec::new();
                if self.artifact_id.is_empty() {
                    events.push(DiagnosticEvent::new(
                        DiagnosticSeverity::Error,
                        "GNSS_OBS_DECISION_INVALID",
                        "observation decision missing artifact id",
                    ));
                }
                events
            }
        }
    }

    pub mod support_matrix {
        use super::*;
        use crate::api::SupportMatrix;

        /// Support matrix artifact v1.
        pub type SupportMatrixV1 = ArtifactV1<SupportMatrix>;

        impl ArtifactPayloadValidate for SupportMatrix {
            fn validate_payload(&self) -> Vec<DiagnosticEvent> {
                if self.rows.is_empty() {
                    return vec![DiagnosticEvent::new(
                        DiagnosticSeverity::Warning,
                        "GNSS_SUPPORT_MATRIX_EMPTY",
                        "support matrix is empty",
                    )];
                }
                Vec::new()
            }
        }
    }

    pub mod obs {
        use super::*;
        use crate::api::{
            ArtifactValidate, Constellation, DiagnosticSeverity, ObsEpoch, SigId, SignalBand,
            SignalCode,
        };

        /// Observation epoch artifact v1.
        pub type ObsEpochV1 = ArtifactV1<ObsEpoch>;

        impl ArtifactValidate for ObsEpochV1 {
            fn validate(&self) -> Vec<DiagnosticEvent> {
                let mut events = self.payload.validate_payload();
                for sat in &self.payload.sats {
                    if !is_valid_signal_id(&sat.signal_id) {
                        events.push(DiagnosticEvent::new(
                            DiagnosticSeverity::Error,
                            "GNSS_OBS_ID_INVALID",
                            "obs signal_id is invalid",
                        ));
                    }
                }
                events
            }
        }

        impl ArtifactPayloadValidate for ObsEpoch {
            fn validate_payload(&self) -> Vec<DiagnosticEvent> {
                let mut events = self.validate_physics();
                if !self.t_rx_s.0.is_finite() {
                    events.push(DiagnosticEvent::new(
                        DiagnosticSeverity::Error,
                        "GNSS_NUMERIC_T_RX_INVALID",
                        "t_rx_s is not finite",
                    ));
                }
                events
            }
        }

        fn is_valid_signal_id(id: &SigId) -> bool {
            if id.sat.prn == 0 {
                return false;
            }
            !matches!(id.sat.constellation, Constellation::Unknown)
                && !matches!(id.band, SignalBand::Unknown)
                && !matches!(id.code, SignalCode::Unknown)
        }
    }

    pub mod nav {
        use super::*;
        use crate::api::NavSolutionEpoch;

        /// Navigation solution artifact v1.
        pub type NavSolutionEpochV1 = ArtifactV1<NavSolutionEpoch>;

        impl ArtifactPayloadValidate for NavSolutionEpoch {
            fn validate_payload(&self) -> Vec<DiagnosticEvent> {
                let mut events = Vec::new();
                if !self.ecef_x_m.0.is_finite()
                    || !self.ecef_y_m.0.is_finite()
                    || !self.ecef_z_m.0.is_finite()
                {
                    events.push(DiagnosticEvent::new(
                        DiagnosticSeverity::Error,
                        "GNSS_NUMERIC_PVT_INVALID",
                        "PVT position contains NaN/Inf",
                    ));
                }
                events
            }
        }
    }

    pub mod rtk {
        //! RTK artifact aliases.
        use super::*;

        /// Single-difference artifact v1.
        pub type RtkSdEpochV1<T> = ArtifactV1<T>;
        /// Double-difference artifact v1.
        pub type RtkDdEpochV1<T> = ArtifactV1<T>;
        /// Baseline solution artifact v1.
        pub type RtkBaselineEpochV1<T> = ArtifactV1<T>;
        /// Baseline quality artifact v1.
        pub type RtkBaselineQualityV1<T> = ArtifactV1<T>;
        /// Fix audit artifact v1.
        pub type RtkFixAuditV1<T> = ArtifactV1<T>;
        /// Precision artifact v1.
        pub type RtkPrecisionV1<T> = ArtifactV1<T>;
    }

    pub mod ppp {
        //! PPP artifact aliases.
        use super::*;

        /// PPP solution artifact v1.
        pub type PppEpochV1<T> = ArtifactV1<T>;
    }
}

fn default_producer() -> String {
    "unknown".to_string()
}

fn default_producer_version() -> String {
    "unknown".to_string()
}
