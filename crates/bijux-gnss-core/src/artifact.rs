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
    use crate::api::{DiagnosticEvent, DiagnosticSeverity, ReceiverSampleTrace, Seconds};

    fn validate_receiver_sample_trace(
        trace: ReceiverSampleTrace,
        context: &str,
        expected_receiver_time: Option<Seconds>,
        expected_sample_index: Option<u64>,
    ) -> Vec<DiagnosticEvent> {
        let mut events = Vec::new();
        if let Err(reason) = trace.validate() {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Error,
                "GNSS_RECEIVER_SAMPLE_TRACE_INVALID",
                format!("{context} receiver sample trace invalid: {reason}"),
            ));
        }
        if let Some(sample_index) = expected_sample_index {
            if trace.sample_index != sample_index {
                events.push(DiagnosticEvent::new(
                    DiagnosticSeverity::Error,
                    "GNSS_RECEIVER_SAMPLE_TRACE_SAMPLE_MISMATCH",
                    format!("{context} receiver sample trace sample index does not match"),
                ));
            }
        }
        if let Some(receiver_time) = expected_receiver_time {
            let tolerance_s = if trace.sample_rate_hz > 0.0 {
                (0.5 / trace.sample_rate_hz).max(1.0e-12)
            } else {
                1.0e-12
            };
            if (trace.receiver_time_s.0 - receiver_time.0).abs() > tolerance_s {
                events.push(DiagnosticEvent::new(
                    DiagnosticSeverity::Error,
                    "GNSS_RECEIVER_SAMPLE_TRACE_TIME_MISMATCH",
                    format!("{context} receiver sample trace time does not match"),
                ));
            }
        }
        events
    }

    pub mod acq {
        use super::*;
        use crate::api::AcqResult;

        /// Acquisition result artifact v1.
        pub type AcqResultV1 = ArtifactV1<AcqResult>;

        impl ArtifactPayloadValidate for AcqResult {
            fn validate_payload(&self) -> Vec<DiagnosticEvent> {
                let mut events = Vec::new();
                if !self.doppler_hz.0.is_finite() || !self.carrier_hz.0.is_finite() {
                    events.push(DiagnosticEvent::new(
                        DiagnosticSeverity::Error,
                        "GNSS_NUMERIC_ACQ_INVALID",
                        "acquisition result contains NaN/Inf",
                    ));
                }
                if self.signal_band == crate::api::SignalBand::Unknown {
                    events.push(DiagnosticEvent::new(
                        DiagnosticSeverity::Error,
                        "GNSS_ACQ_SIGNAL_BAND_INVALID",
                        "acquisition result must declare an explicit signal band",
                    ));
                }
                if let Some(refinement) = &self.doppler_refinement {
                    if !refinement.coarse_carrier_hz.0.is_finite()
                        || !refinement.offset_hz.is_finite()
                        || !refinement.offset_bins.is_finite()
                        || !refinement.left_peak_mean_ratio.is_finite()
                        || !refinement.center_peak_mean_ratio.is_finite()
                        || !refinement.right_peak_mean_ratio.is_finite()
                    {
                        events.push(DiagnosticEvent::new(
                            DiagnosticSeverity::Error,
                            "GNSS_NUMERIC_ACQ_REFINEMENT_INVALID",
                            "acquisition refinement contains NaN/Inf",
                        ));
                    }
                }
                if let Some(refinement) = &self.code_phase_refinement {
                    if !refinement.offset_samples.is_finite()
                        || !refinement.refined_code_phase_samples.is_finite()
                        || !refinement.left_correlation_norm.is_finite()
                        || !refinement.center_correlation_norm.is_finite()
                        || !refinement.right_correlation_norm.is_finite()
                    {
                        events.push(DiagnosticEvent::new(
                            DiagnosticSeverity::Error,
                            "GNSS_NUMERIC_ACQ_CODE_PHASE_REFINEMENT_INVALID",
                            "acquisition code-phase refinement contains NaN/Inf",
                        ));
                    }
                }
                if let Some(uncertainty) = &self.uncertainty {
                    if !uncertainty.doppler_hz.is_finite()
                        || !uncertainty.code_phase_samples.is_finite()
                        || uncertainty.doppler_hz < 0.0
                        || uncertainty.code_phase_samples < 0.0
                    {
                        events.push(DiagnosticEvent::new(
                            DiagnosticSeverity::Error,
                            "GNSS_NUMERIC_ACQ_UNCERTAINTY_INVALID",
                            "acquisition uncertainty contains invalid values",
                        ));
                    }
                }
                if self.candidate_rank == 0 {
                    events.push(DiagnosticEvent::new(
                        DiagnosticSeverity::Error,
                        "GNSS_ACQ_CANDIDATE_RANK_INVALID",
                        "acquisition candidate rank must be at least 1",
                    ));
                }
                if self.is_primary_candidate && self.candidate_rank != 1 {
                    events.push(DiagnosticEvent::new(
                        DiagnosticSeverity::Error,
                        "GNSS_ACQ_PRIMARY_CANDIDATE_RANK_INVALID",
                        "primary acquisition candidate must use rank 1",
                    ));
                }
                if !self.is_primary_candidate && self.candidate_rank == 1 {
                    events.push(DiagnosticEvent::new(
                        DiagnosticSeverity::Error,
                        "GNSS_ACQ_ALTERNATIVE_CANDIDATE_RANK_INVALID",
                        "non-primary acquisition candidates must use rank 2 or greater",
                    ));
                }
                events.extend(validate_receiver_sample_trace(
                    self.source_time,
                    "acquisition",
                    None,
                    None,
                ));
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
                if self.navigation_bit_sign.is_some_and(|sign| sign != -1 && sign != 1) {
                    events.push(DiagnosticEvent::new(
                        DiagnosticSeverity::Error,
                        "GNSS_TRACK_NAVIGATION_BIT_SIGN_INVALID",
                        "tracking epoch navigation bit sign must be -1 or 1",
                    ));
                }
                if let Some(uncertainty) = &self.tracking_uncertainty {
                    if !uncertainty.code_phase_samples.is_finite()
                        || !uncertainty.carrier_phase_cycles.is_finite()
                        || !uncertainty.doppler_hz.is_finite()
                        || !uncertainty.cn0_dbhz.is_finite()
                        || uncertainty.code_phase_samples < 0.0
                        || uncertainty.carrier_phase_cycles < 0.0
                        || uncertainty.doppler_hz < 0.0
                        || uncertainty.cn0_dbhz < 0.0
                    {
                        events.push(DiagnosticEvent::new(
                            DiagnosticSeverity::Error,
                            "GNSS_NUMERIC_TRACK_UNCERTAINTY_INVALID",
                            "tracking uncertainty contains invalid values",
                        ));
                    }
                }
                events.extend(validate_receiver_sample_trace(
                    self.source_time,
                    "tracking",
                    None,
                    Some(self.sample_index),
                ));
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
                events.extend(validate_receiver_sample_trace(
                    self.source_time,
                    "observation",
                    Some(self.t_rx_s),
                    None,
                ));
                if let Some(manifest) = &self.manifest {
                    events.extend(validate_receiver_sample_trace(
                        manifest.source_time,
                        "observation manifest",
                        Some(self.t_rx_s),
                        Some(manifest.source_sample_index),
                    ));
                    if manifest.source_time.sample_index != self.source_time.sample_index {
                        events.push(DiagnosticEvent::new(
                            DiagnosticSeverity::Error,
                            "GNSS_OBS_SOURCE_TIME_MISMATCH",
                            "observation manifest source trace does not match epoch source trace",
                        ));
                    }
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
        use crate::api::{NavSolutionEpoch, SolutionStatus};

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
                if let Some(covariance) = self.position_covariance_ecef_m2 {
                    if !covariance.iter().flat_map(|row| row.iter()).all(|value| value.is_finite())
                    {
                        events.push(DiagnosticEvent::new(
                            DiagnosticSeverity::Error,
                            "GNSS_NAV_POSITION_COVARIANCE_INVALID",
                            "nav solution position covariance contains NaN/Inf",
                        ));
                    }
                    if covariance[0][0] < 0.0 || covariance[1][1] < 0.0 || covariance[2][2] < 0.0 {
                        events.push(DiagnosticEvent::new(
                            DiagnosticSeverity::Error,
                            "GNSS_NAV_POSITION_COVARIANCE_NEGATIVE_VARIANCE",
                            "nav solution position covariance diagonal must be non-negative",
                        ));
                    }
                } else if self.valid {
                    events.push(DiagnosticEvent::new(
                        DiagnosticSeverity::Warning,
                        "GNSS_NAV_POSITION_COVARIANCE_MISSING",
                        "valid nav solution should carry an ECEF position covariance matrix",
                    ));
                }
                let enu_sigmas = [self.sigma_e_m, self.sigma_n_m, self.sigma_u_m];
                if enu_sigmas.iter().flatten().any(|value| !value.0.is_finite()) {
                    events.push(DiagnosticEvent::new(
                        DiagnosticSeverity::Error,
                        "GNSS_NAV_POSITION_SIGMA_ENU_INVALID",
                        "nav solution ENU position standard deviations contain NaN/Inf",
                    ));
                }
                if enu_sigmas.iter().flatten().any(|value| value.0 < 0.0) {
                    events.push(DiagnosticEvent::new(
                        DiagnosticSeverity::Error,
                        "GNSS_NAV_POSITION_SIGMA_ENU_NEGATIVE",
                        "nav solution ENU position standard deviations must be non-negative",
                    ));
                }
                if self.valid && enu_sigmas.iter().any(|value| value.is_none()) {
                    events.push(DiagnosticEvent::new(
                        DiagnosticSeverity::Warning,
                        "GNSS_NAV_POSITION_SIGMA_ENU_MISSING",
                        "valid nav solution should carry east, north, and up position standard deviations",
                    ));
                }
                if !self.clock_bias_s.0.is_finite() || !self.clock_bias_m.0.is_finite() {
                    events.push(DiagnosticEvent::new(
                        DiagnosticSeverity::Error,
                        "GNSS_NAV_CLOCK_BIAS_INVALID",
                        "nav solution clock bias contains NaN/Inf",
                    ));
                } else {
                    let expected_clock_bias_m = self.clock_bias_s.0 * 299_792_458.0;
                    if (self.clock_bias_m.0 - expected_clock_bias_m).abs() > 1.0e-6 {
                        events.push(DiagnosticEvent::new(
                            DiagnosticSeverity::Error,
                            "GNSS_NAV_CLOCK_BIAS_UNITS_INCONSISTENT",
                            "nav solution clock bias meters do not match clock bias seconds",
                        ));
                    }
                }
                if !self.pdop.is_finite()
                    || self.hdop.is_some_and(|value| !value.is_finite())
                    || self.vdop.is_some_and(|value| !value.is_finite())
                    || self.gdop.is_some_and(|value| !value.is_finite())
                    || self.tdop.is_some_and(|value| !value.is_finite())
                {
                    events.push(DiagnosticEvent::new(
                        DiagnosticSeverity::Error,
                        "GNSS_NAV_DOPS_INVALID",
                        "nav solution DOP values contain NaN/Inf",
                    ));
                }
                if self.pre_fit_residual_rms_m.is_some_and(|value| !value.0.is_finite())
                    || self.post_fit_residual_rms_m.is_some_and(|value| !value.0.is_finite())
                {
                    events.push(DiagnosticEvent::new(
                        DiagnosticSeverity::Error,
                        "GNSS_NAV_RESIDUAL_RMS_INVALID",
                        "nav solution residual RMS values contain NaN/Inf",
                    ));
                }
                if self.pre_fit_residual_rms_m.is_some() ^ self.post_fit_residual_rms_m.is_some() {
                    events.push(DiagnosticEvent::new(
                        DiagnosticSeverity::Warning,
                        "GNSS_NAV_RESIDUAL_RMS_INCOMPLETE",
                        "nav solution should either provide both residual RMS values or neither",
                    ));
                }
                if let Some(post_fit_residual_rms_m) = self.post_fit_residual_rms_m {
                    if (post_fit_residual_rms_m.0 - self.rms_m.0).abs() > 1.0e-9 {
                        events.push(DiagnosticEvent::new(
                            DiagnosticSeverity::Warning,
                            "GNSS_NAV_POST_FIT_RMS_MISMATCH",
                            "nav solution post-fit residual RMS does not match rms_m",
                        ));
                    }
                }
                if self.model_version == 0 {
                    events.push(DiagnosticEvent::new(
                        DiagnosticSeverity::Error,
                        "GNSS_NAV_MODEL_VERSION_INVALID",
                        "nav solution model_version must be non-zero",
                    ));
                }
                if self.sat_count != self.used_sat_count + self.rejected_sat_count {
                    events.push(DiagnosticEvent::new(
                        DiagnosticSeverity::Error,
                        "GNSS_NAV_SAT_COUNTS_INVALID",
                        "sat_count does not match used_sat_count + rejected_sat_count",
                    ));
                }
                if self.valid != self.status.is_valid() {
                    events.push(DiagnosticEvent::new(
                        DiagnosticSeverity::Warning,
                        "GNSS_NAV_STATUS_VALID_MISMATCH",
                        "nav solution valid flag does not match status validity",
                    ));
                }
                if self.artifact_id.is_empty() {
                    events.push(DiagnosticEvent::new(
                        DiagnosticSeverity::Warning,
                        "GNSS_NAV_ARTIFACT_ID_MISSING",
                        "nav solution artifact_id is empty",
                    ));
                }
                if self.source_observation_epoch_id.is_empty() {
                    events.push(DiagnosticEvent::new(
                        DiagnosticSeverity::Warning,
                        "GNSS_NAV_SOURCE_EPOCH_ID_MISSING",
                        "nav solution source_observation_epoch_id is empty",
                    ));
                }
                events.extend(validate_receiver_sample_trace(
                    self.source_time,
                    "navigation",
                    Some(self.t_rx_s),
                    None,
                ));
                if self.refusal_class.is_some() && self.explain_decision.is_empty() {
                    events.push(DiagnosticEvent::new(
                        DiagnosticSeverity::Warning,
                        "GNSS_NAV_EXPLAIN_DECISION_MISSING",
                        "nav solution refusal_class requires explain_decision",
                    ));
                }
                if self.stability_signature.is_empty() {
                    events.push(DiagnosticEvent::new(
                        DiagnosticSeverity::Warning,
                        "GNSS_NAV_STABILITY_SIGNATURE_MISSING",
                        "nav solution stability signature is empty",
                    ));
                }
                if self.stability_signature_version == 0 {
                    events.push(DiagnosticEvent::new(
                        DiagnosticSeverity::Warning,
                        "GNSS_NAV_STABILITY_SIGNATURE_VERSION_INVALID",
                        "nav solution stability signature version is zero",
                    ));
                }
                if matches!(self.status, SolutionStatus::Invalid) && self.refusal_class.is_none() {
                    events.push(DiagnosticEvent::new(
                        DiagnosticSeverity::Warning,
                        "GNSS_NAV_REFUSAL_CLASS_MISSING",
                        "invalid nav solution should carry a refusal_class",
                    ));
                }
                if !self.residuals.is_empty() && self.post_fit_residual_rms_m.is_none() {
                    events.push(DiagnosticEvent::new(
                        DiagnosticSeverity::Warning,
                        "GNSS_NAV_POST_FIT_RMS_MISSING",
                        "nav solution with residuals should carry post-fit residual RMS",
                    ));
                }
                if !self.residuals.is_empty() && self.constellation_residual_rms.is_empty() {
                    events.push(DiagnosticEvent::new(
                        DiagnosticSeverity::Warning,
                        "GNSS_NAV_CONSTELLATION_RMS_MISSING",
                        "nav solution with residuals should carry per-constellation residual RMS",
                    ));
                }
                let accepted_residual_counts = self
                    .residuals
                    .iter()
                    .filter(|residual| !residual.rejected)
                    .fold(std::collections::BTreeMap::new(), |mut counts, residual| {
                        *counts.entry(residual.sat.constellation).or_insert(0usize) += 1;
                        counts
                    });
                let mut summarized_post_fit_count = 0usize;
                let mut seen_constellations = std::collections::BTreeSet::new();
                for summary in &self.constellation_residual_rms {
                    if !seen_constellations.insert(summary.constellation) {
                        events.push(DiagnosticEvent::new(
                            DiagnosticSeverity::Error,
                            "GNSS_NAV_CONSTELLATION_RMS_DUPLICATE",
                            "nav solution duplicates a constellation residual RMS entry",
                        ));
                    }
                    if summary.pre_fit_rms_m.is_some_and(|value| !value.0.is_finite())
                        || summary.post_fit_rms_m.is_some_and(|value| !value.0.is_finite())
                    {
                        events.push(DiagnosticEvent::new(
                            DiagnosticSeverity::Error,
                            "GNSS_NAV_CONSTELLATION_RMS_INVALID",
                            "nav solution constellation residual RMS contains NaN/Inf",
                        ));
                    }
                    if summary.pre_fit_rms_m.is_some() && summary.pre_fit_sat_count == 0 {
                        events.push(DiagnosticEvent::new(
                            DiagnosticSeverity::Error,
                            "GNSS_NAV_CONSTELLATION_PRE_FIT_COUNT_INVALID",
                            "nav solution constellation pre-fit RMS requires a non-zero satellite count",
                        ));
                    }
                    if summary.post_fit_rms_m.is_some() && summary.post_fit_sat_count == 0 {
                        events.push(DiagnosticEvent::new(
                            DiagnosticSeverity::Error,
                            "GNSS_NAV_CONSTELLATION_POST_FIT_COUNT_INVALID",
                            "nav solution constellation post-fit RMS requires a non-zero satellite count",
                        ));
                    }
                    if summary.post_fit_sat_count > 0
                        && accepted_residual_counts
                            .get(&summary.constellation)
                            .copied()
                            .unwrap_or(0)
                            != summary.post_fit_sat_count
                    {
                        events.push(DiagnosticEvent::new(
                            DiagnosticSeverity::Error,
                            "GNSS_NAV_CONSTELLATION_POST_FIT_COUNT_MISMATCH",
                            "nav solution constellation post-fit satellite counts do not match accepted residuals",
                        ));
                    }
                    summarized_post_fit_count += summary.post_fit_sat_count;
                }
                if !self.constellation_residual_rms.is_empty()
                    && summarized_post_fit_count != accepted_residual_counts.values().sum::<usize>()
                {
                    events.push(DiagnosticEvent::new(
                        DiagnosticSeverity::Error,
                        "GNSS_NAV_CONSTELLATION_POST_FIT_TOTAL_MISMATCH",
                        "nav solution constellation residual RMS totals do not match accepted residuals",
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
