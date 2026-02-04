//! Artifact contracts and versioned payloads.

use serde::{Deserialize, Serialize};

use crate::DiagnosticEvent;

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

impl<T: ArtifactPayloadValidate> ArtifactValidate for ArtifactV1<T> {
    fn validate(&self) -> Vec<DiagnosticEvent> {
        self.payload.validate_payload()
    }
}

/// Migration placeholder for V1 -> V2.
pub fn convert_v1_to_v2<T>(_artifact: &T) {
    // TODO: implement when V2 is defined.
}

/// Versioned artifact modules.
pub mod v1 {
    //! Versioned v1 artifact types.

    use super::{ArtifactV1, ArtifactPayloadValidate};
    use crate::{DiagnosticEvent, DiagnosticSeverity};

    pub mod acq {
        use super::*;
        use crate::AcqResult;

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
        use crate::TrackEpoch;

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

    pub mod obs {
        use super::*;
        use crate::ObsEpoch;

        /// Observation epoch artifact v1.
        pub type ObsEpochV1 = ArtifactV1<ObsEpoch>;

        impl ArtifactPayloadValidate for ObsEpoch {
            fn validate_payload(&self) -> Vec<DiagnosticEvent> {
                let mut events = Vec::new();
                if !self.t_rx_s.0.is_finite() {
                    events.push(DiagnosticEvent::new(
                        DiagnosticSeverity::Error,
                        "GNSS_NUMERIC_T_RX_INVALID",
                        "t_rx_s is not finite",
                    ));
                }
                for sat in &self.sats {
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
        }
    }

    pub mod nav {
        use super::*;
        use crate::NavSolutionEpoch;

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
