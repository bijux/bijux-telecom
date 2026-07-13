#![allow(missing_docs)]

use serde::{Deserialize, Serialize};

use bijux_gnss_core::api::{ArtifactPayloadValidate, DiagnosticEvent, DiagnosticSeverity};

pub const ADVANCED_SUPPORT_MATRIX_VERSION: u32 = 2;

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
pub enum AdvancedMode {
    Rtk,
    Ppp,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
pub enum AdvancedMaturity {
    NotReady,
    Experimental,
    Enforced,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
pub enum AdvancedRefusalClass {
    IncompleteCorrections,
    MissingBaseline,
    UnsupportedModel,
    InsufficientGeometry,
    IncompleteAmbiguityState,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
pub enum AdvancedSolutionClaim {
    NotReady,
    Float,
    Fixed,
    FallbackNav,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum ExecutionStatus {
    Executed,
    NotReady,
    Unsupported,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExecutionArtifact<T> {
    pub epoch_idx: u64,
    pub status: ExecutionStatus,
    pub reason: Option<String>,
    pub value: Option<T>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AdvancedSupportRow {
    pub mode: AdvancedMode,
    pub maturity: AdvancedMaturity,
    pub real_solver: bool,
    pub required_inputs: Vec<String>,
    pub notes: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AdvancedSupportMatrix {
    pub schema_version: u32,
    pub rows: Vec<AdvancedSupportRow>,
}

#[derive(Debug, Clone, Serialize, Deserialize, Default, PartialEq)]
pub struct AdvancedSolutionMeasurements {
    #[serde(default)]
    pub sigma_h_m: Option<f64>,
    #[serde(default)]
    pub sigma_v_m: Option<f64>,
    #[serde(default)]
    pub residual_rms_m: Option<f64>,
    #[serde(default)]
    pub predicted_rms_m: Option<f64>,
    #[serde(default)]
    pub hpl_m: Option<f64>,
    #[serde(default)]
    pub vpl_m: Option<f64>,
}

#[derive(Debug, Clone, Serialize, Deserialize, Default, PartialEq, Eq)]
pub struct AdvancedSolutionEvidence {
    pub covariance_supported: bool,
    pub residual_supported: bool,
    pub ambiguity_supported: bool,
    pub correction_supported: bool,
    pub integrity_supported: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AdvancedPrerequisites {
    pub has_base_observations: bool,
    pub has_rover_observations: bool,
    pub has_ephemeris: bool,
    pub has_reference_frame: bool,
    pub has_corrections: bool,
    pub has_min_satellites: bool,
    pub has_ambiguity_state: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AdvancedPrereqDecision {
    pub ready: bool,
    pub refusal_class: Option<AdvancedRefusalClass>,
    pub reasons: Vec<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub struct AdvancedClaimDecision {
    pub status: String,
    pub downgraded: bool,
    pub downgrade_reason: Option<String>,
    pub claim: AdvancedSolutionClaim,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AdvancedSolutionProvenance {
    pub claim: AdvancedSolutionClaim,
    pub ambiguity_state_count: usize,
    pub correction_source: String,
    pub fallback_from: Option<String>,
    pub fixed_ratio: Option<f64>,
    #[serde(default)]
    pub measurements: AdvancedSolutionMeasurements,
    #[serde(default)]
    pub evidence: AdvancedSolutionEvidence,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CorrectionInputArtifact {
    pub epoch_idx: u64,
    pub mode: AdvancedMode,
    pub ephemeris_count: usize,
    pub products_ok: bool,
    pub correction_tags: Vec<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AmbiguityStateArtifact {
    pub epoch_idx: u64,
    pub mode: AdvancedMode,
    pub float_count: usize,
    pub fixed_count: usize,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AdvancedSolutionArtifact {
    pub epoch_idx: u64,
    pub mode: AdvancedMode,
    pub status: String,
    pub downgraded: bool,
    pub downgrade_reason: Option<String>,
    pub prerequisites: AdvancedPrerequisites,
    pub refusal_class: Option<AdvancedRefusalClass>,
    pub provenance: AdvancedSolutionProvenance,
    pub artifact_id: String,
    pub source_observation_epoch_id: String,
}

const RTK_FIXED_RATIO_THRESHOLD: f64 = 3.0;
const RTK_MAX_RESIDUAL_SCALE: f64 = 3.0;

pub fn support_status_matrix() -> AdvancedSupportMatrix {
    AdvancedSupportMatrix {
        schema_version: ADVANCED_SUPPORT_MATRIX_VERSION,
        rows: vec![
            AdvancedSupportRow {
                mode: AdvancedMode::Rtk,
                maturity: AdvancedMaturity::Experimental,
                real_solver: true,
                required_inputs: vec![
                    "base_obs".to_string(),
                    "rover_obs".to_string(),
                    "ephemeris".to_string(),
                    "base_ecef".to_string(),
                ],
                notes:
                    "double-difference baseline is implemented with explicit refusal and downgrade"
                        .to_string(),
            },
            AdvancedSupportRow {
                mode: AdvancedMode::Ppp,
                maturity: AdvancedMaturity::NotReady,
                real_solver: false,
                required_inputs: vec![
                    "multi_frequency_obs".to_string(),
                    "products_or_broadcast".to_string(),
                    "time_consistent_epochs".to_string(),
                ],
                notes: "ppp readiness is reported separately; runtime execution remains not_ready"
                    .to_string(),
            },
        ],
    }
}

pub fn evaluate_solution_evidence(
    mode: AdvancedMode,
    claim: AdvancedSolutionClaim,
    ambiguity_state_count: usize,
    fixed_ratio: Option<f64>,
    correction_source: &str,
    measurements: &AdvancedSolutionMeasurements,
) -> AdvancedSolutionEvidence {
    match mode {
        AdvancedMode::Ppp => AdvancedSolutionEvidence {
            covariance_supported: has_positive_finite(measurements.sigma_h_m)
                && has_positive_finite(measurements.sigma_v_m),
            residual_supported: has_bounded_residual_scale(
                measurements.residual_rms_m,
                measurements.predicted_rms_m,
            ),
            ambiguity_supported: ambiguity_state_count > 0,
            correction_supported: has_correction_source(correction_source),
            integrity_supported: has_positive_finite(measurements.hpl_m)
                && has_positive_finite(measurements.vpl_m),
        },
        AdvancedMode::Rtk => {
            let ambiguity_supported = ambiguity_state_count > 0
                && match claim {
                    AdvancedSolutionClaim::Fixed => fixed_ratio.is_some_and(|ratio| {
                        ratio.is_finite() && ratio >= RTK_FIXED_RATIO_THRESHOLD
                    }),
                    AdvancedSolutionClaim::Float => true,
                    AdvancedSolutionClaim::NotReady | AdvancedSolutionClaim::FallbackNav => true,
                };
            AdvancedSolutionEvidence {
                covariance_supported: has_positive_finite(measurements.sigma_h_m)
                    && has_positive_finite(measurements.sigma_v_m),
                residual_supported: has_bounded_residual_scale(
                    measurements.residual_rms_m,
                    measurements.predicted_rms_m,
                ),
                ambiguity_supported,
                correction_supported: has_correction_source(correction_source),
                integrity_supported: has_positive_finite(measurements.hpl_m)
                    && has_positive_finite(measurements.vpl_m),
            }
        }
    }
}

impl AdvancedSolutionEvidence {
    pub fn supports_strong_claim(&self) -> bool {
        self.covariance_supported
            && self.residual_supported
            && self.ambiguity_supported
            && self.correction_supported
            && self.integrity_supported
    }

    pub fn missing_reasons(&self) -> Vec<String> {
        let mut reasons = Vec::new();
        if !self.covariance_supported {
            reasons.push("missing_covariance_evidence".to_string());
        }
        if !self.residual_supported {
            reasons.push("missing_residual_evidence".to_string());
        }
        if !self.ambiguity_supported {
            reasons.push("missing_ambiguity_evidence".to_string());
        }
        if !self.correction_supported {
            reasons.push("missing_correction_evidence".to_string());
        }
        if !self.integrity_supported {
            reasons.push("missing_integrity_evidence".to_string());
        }
        reasons
    }
}

fn has_positive_finite(value: Option<f64>) -> bool {
    value.is_some_and(|value| value.is_finite() && value > 0.0)
}

fn has_correction_source(correction_source: &str) -> bool {
    !correction_source.is_empty() && correction_source != "uncorrected"
}

fn has_bounded_residual_scale(residual_rms_m: Option<f64>, predicted_rms_m: Option<f64>) -> bool {
    let Some(residual_rms_m) = residual_rms_m else {
        return false;
    };
    let Some(predicted_rms_m) = predicted_rms_m else {
        return false;
    };
    if !residual_rms_m.is_finite()
        || !predicted_rms_m.is_finite()
        || residual_rms_m < 0.0
        || predicted_rms_m < 0.0
    {
        return false;
    }
    if predicted_rms_m <= f64::EPSILON {
        return residual_rms_m <= f64::EPSILON;
    }
    residual_rms_m / predicted_rms_m <= RTK_MAX_RESIDUAL_SCALE
}

pub fn evaluate_prerequisites(
    mode: AdvancedMode,
    prereq: &AdvancedPrerequisites,
) -> AdvancedPrereqDecision {
    let mut reasons = Vec::new();
    if !prereq.has_ephemeris {
        reasons.push("missing_ephemeris".to_string());
    }
    if !prereq.has_reference_frame {
        reasons.push("missing_reference_frame".to_string());
    }
    if !prereq.has_min_satellites {
        reasons.push("insufficient_geometry".to_string());
    }
    match mode {
        AdvancedMode::Rtk => {
            if !prereq.has_base_observations || !prereq.has_rover_observations {
                reasons.push("missing_baseline_observations".to_string());
            }
            if !prereq.has_ambiguity_state {
                reasons.push("missing_ambiguity_state".to_string());
            }
            if reasons.is_empty() {
                return AdvancedPrereqDecision {
                    ready: true,
                    refusal_class: None,
                    reasons: vec!["rtk_prerequisites_met".to_string()],
                };
            }
            let refusal = if reasons.iter().any(|r| r == "missing_baseline_observations") {
                Some(AdvancedRefusalClass::MissingBaseline)
            } else if reasons.iter().any(|r| r == "missing_ambiguity_state") {
                Some(AdvancedRefusalClass::IncompleteAmbiguityState)
            } else if reasons.iter().any(|r| r == "insufficient_geometry") {
                Some(AdvancedRefusalClass::InsufficientGeometry)
            } else {
                Some(AdvancedRefusalClass::UnsupportedModel)
            };
            AdvancedPrereqDecision { ready: false, refusal_class: refusal, reasons }
        }
        AdvancedMode::Ppp => {
            if !prereq.has_corrections {
                reasons.push("incomplete_corrections".to_string());
            }
            if reasons.is_empty() {
                return AdvancedPrereqDecision {
                    ready: true,
                    refusal_class: None,
                    reasons: vec!["ppp_prerequisites_met".to_string()],
                };
            }
            let refusal = if reasons.iter().any(|r| r == "incomplete_corrections") {
                Some(AdvancedRefusalClass::IncompleteCorrections)
            } else if reasons.iter().any(|r| r == "insufficient_geometry") {
                Some(AdvancedRefusalClass::InsufficientGeometry)
            } else {
                Some(AdvancedRefusalClass::UnsupportedModel)
            };
            AdvancedPrereqDecision { ready: false, refusal_class: refusal, reasons }
        }
    }
}

pub fn apply_downgrade_policy(
    mode: AdvancedMode,
    decision: &AdvancedPrereqDecision,
    claim: AdvancedSolutionClaim,
    evidence: Option<&AdvancedSolutionEvidence>,
) -> AdvancedClaimDecision {
    if mode == AdvancedMode::Ppp {
        return if decision.ready {
            AdvancedClaimDecision {
                status: "not_ready".to_string(),
                downgraded: false,
                downgrade_reason: None,
                claim: AdvancedSolutionClaim::NotReady,
            }
        } else {
            AdvancedClaimDecision {
                status: "not_ready".to_string(),
                downgraded: false,
                downgrade_reason: Some(decision.reasons.join(",")),
                claim: AdvancedSolutionClaim::NotReady,
            }
        };
    }

    if decision.ready {
        if claim_requires_strong_evidence(claim) {
            let evidence = evidence.cloned().unwrap_or_default();
            let missing_reasons = evidence.missing_reasons();
            if !missing_reasons.is_empty() {
                return AdvancedClaimDecision {
                    status: "degraded".to_string(),
                    downgraded: true,
                    downgrade_reason: Some(missing_reasons.join(",")),
                    claim: AdvancedSolutionClaim::FallbackNav,
                };
            }
        }
        let status = match claim {
            AdvancedSolutionClaim::Fixed => "accepted_fixed",
            AdvancedSolutionClaim::Float => "accepted_float",
            AdvancedSolutionClaim::NotReady => "not_ready",
            AdvancedSolutionClaim::FallbackNav => "accepted_fallback",
        };
        return AdvancedClaimDecision {
            status: status.to_string(),
            downgraded: false,
            downgrade_reason: None,
            claim,
        };
    }
    let reason = decision.reasons.join(",");
    let downgraded_claim = match mode {
        AdvancedMode::Rtk => AdvancedSolutionClaim::FallbackNav,
        AdvancedMode::Ppp => AdvancedSolutionClaim::NotReady,
    };
    AdvancedClaimDecision {
        status: "degraded".to_string(),
        downgraded: true,
        downgrade_reason: Some(reason),
        claim: downgraded_claim,
    }
}

fn claim_requires_strong_evidence(claim: AdvancedSolutionClaim) -> bool {
    matches!(claim, AdvancedSolutionClaim::Fixed | AdvancedSolutionClaim::Float)
}

impl ArtifactPayloadValidate for AdvancedSolutionArtifact {
    fn validate_payload(&self) -> Vec<DiagnosticEvent> {
        let mut events = Vec::new();
        if self.artifact_id.is_empty() || self.source_observation_epoch_id.is_empty() {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Warning,
                "ADVANCED_SOLUTION_ID_MISSING",
                "advanced solution artifact id fields are empty",
            ));
        }
        if self.downgraded && self.downgrade_reason.as_deref().unwrap_or("").is_empty() {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Warning,
                "ADVANCED_SOLUTION_DOWNGRADE_REASON_MISSING",
                "downgraded advanced solution missing downgrade reason",
            ));
        }
        if claim_requires_strong_evidence(self.provenance.claim)
            && !self.provenance.evidence.supports_strong_claim()
        {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Error,
                "ADVANCED_SOLUTION_EVIDENCE_MISSING",
                "strong advanced solution claim is missing required numerical evidence",
            ));
        }
        events
    }
}

impl<T> ArtifactPayloadValidate for ExecutionArtifact<T>
where
    T: ArtifactPayloadValidate,
{
    fn validate_payload(&self) -> Vec<DiagnosticEvent> {
        let mut events = Vec::new();
        match (self.status, self.value.as_ref()) {
            (ExecutionStatus::Executed, None) => events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Error,
                "EXECUTION_ARTIFACT_VALUE_MISSING",
                "executed artifact is missing its payload value",
            )),
            (ExecutionStatus::NotReady | ExecutionStatus::Unsupported, Some(_)) => {
                events.push(DiagnosticEvent::new(
                    DiagnosticSeverity::Error,
                    "EXECUTION_ARTIFACT_VALUE_UNEXPECTED",
                    "non-executed artifact must not include a payload value",
                ));
            }
            _ => {}
        }
        if matches!(self.status, ExecutionStatus::NotReady | ExecutionStatus::Unsupported)
            && self.reason.as_deref().unwrap_or("").is_empty()
        {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Warning,
                "EXECUTION_ARTIFACT_REASON_MISSING",
                "non-executed artifact is missing its refusal reason",
            ));
        }
        if let Some(value) = self.value.as_ref() {
            events.extend(value.validate_payload());
        }
        events
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Debug, Clone)]
    struct DummyPayload;

    impl ArtifactPayloadValidate for DummyPayload {
        fn validate_payload(&self) -> Vec<DiagnosticEvent> {
            Vec::new()
        }
    }

    fn ready_prereq() -> AdvancedPrerequisites {
        AdvancedPrerequisites {
            has_base_observations: true,
            has_rover_observations: true,
            has_ephemeris: true,
            has_reference_frame: true,
            has_corrections: true,
            has_min_satellites: true,
            has_ambiguity_state: true,
        }
    }

    fn strong_measurements() -> AdvancedSolutionMeasurements {
        AdvancedSolutionMeasurements {
            sigma_h_m: Some(0.04),
            sigma_v_m: Some(0.06),
            residual_rms_m: Some(0.02),
            predicted_rms_m: Some(0.01),
            hpl_m: Some(0.24),
            vpl_m: Some(0.36),
        }
    }

    #[test]
    fn rtk_prerequisite_decision_refuses_missing_baseline() {
        let mut prereq = ready_prereq();
        prereq.has_base_observations = false;
        let decision = evaluate_prerequisites(AdvancedMode::Rtk, &prereq);
        assert!(!decision.ready);
        assert_eq!(decision.refusal_class, Some(AdvancedRefusalClass::MissingBaseline));
    }

    #[test]
    fn ppp_prerequisite_decision_refuses_incomplete_corrections() {
        let mut prereq = ready_prereq();
        prereq.has_corrections = false;
        let decision = evaluate_prerequisites(AdvancedMode::Ppp, &prereq);
        assert!(!decision.ready);
        assert_eq!(decision.refusal_class, Some(AdvancedRefusalClass::IncompleteCorrections));
    }

    #[test]
    fn rtk_fixed_evidence_requires_all_strong_claim_inputs() {
        let evidence = evaluate_solution_evidence(
            AdvancedMode::Rtk,
            AdvancedSolutionClaim::Fixed,
            5,
            Some(3.4),
            "broadcast_ephemeris",
            &strong_measurements(),
        );

        assert!(evidence.supports_strong_claim(), "expected all strong-claim evidence to pass");
        assert!(evidence.missing_reasons().is_empty(), "unexpected missing evidence reasons");
    }

    #[test]
    fn rtk_float_evidence_rejects_missing_integrity_support() {
        let mut measurements = strong_measurements();
        measurements.hpl_m = None;

        let evidence = evaluate_solution_evidence(
            AdvancedMode::Rtk,
            AdvancedSolutionClaim::Float,
            5,
            None,
            "broadcast_ephemeris",
            &measurements,
        );

        assert!(!evidence.supports_strong_claim());
        assert!(evidence
            .missing_reasons()
            .iter()
            .any(|reason| reason == "missing_integrity_evidence"));
    }

    #[test]
    fn rtk_fixed_evidence_rejects_low_ratio_and_residual_overrun() {
        let mut measurements = strong_measurements();
        measurements.predicted_rms_m = Some(0.005);
        measurements.residual_rms_m = Some(0.05);

        let evidence = evaluate_solution_evidence(
            AdvancedMode::Rtk,
            AdvancedSolutionClaim::Fixed,
            5,
            Some(2.4),
            "broadcast_ephemeris",
            &measurements,
        );

        assert!(!evidence.ambiguity_supported);
        assert!(!evidence.residual_supported);
        assert!(!evidence.supports_strong_claim());
    }

    #[test]
    fn downgrade_policy_falls_back_when_not_ready() {
        let decision = AdvancedPrereqDecision {
            ready: false,
            refusal_class: Some(AdvancedRefusalClass::InsufficientGeometry),
            reasons: vec!["insufficient_geometry".to_string()],
        };
        let decision = apply_downgrade_policy(
            AdvancedMode::Rtk,
            &decision,
            AdvancedSolutionClaim::Fixed,
            None,
        );
        assert_eq!(decision.status, "degraded");
        assert!(decision.downgraded);
        assert!(decision.downgrade_reason.is_some());
        assert_eq!(decision.claim, AdvancedSolutionClaim::FallbackNav);
    }

    #[test]
    fn ppp_policy_reports_not_ready_even_when_prerequisites_are_met() {
        let decision = AdvancedPrereqDecision {
            ready: true,
            refusal_class: None,
            reasons: vec!["ppp_prerequisites_met".to_string()],
        };
        let decision = apply_downgrade_policy(
            AdvancedMode::Ppp,
            &decision,
            AdvancedSolutionClaim::Float,
            None,
        );
        assert_eq!(decision.status, "not_ready");
        assert!(!decision.downgraded);
        assert!(decision.downgrade_reason.is_none());
        assert_eq!(decision.claim, AdvancedSolutionClaim::NotReady);
    }

    #[test]
    fn ppp_policy_reports_not_ready_with_prerequisite_reasons() {
        let decision = AdvancedPrereqDecision {
            ready: false,
            refusal_class: Some(AdvancedRefusalClass::IncompleteCorrections),
            reasons: vec!["incomplete_corrections".to_string()],
        };
        let decision = apply_downgrade_policy(
            AdvancedMode::Ppp,
            &decision,
            AdvancedSolutionClaim::Float,
            None,
        );
        assert_eq!(decision.status, "not_ready");
        assert!(!decision.downgraded);
        assert_eq!(decision.downgrade_reason.as_deref(), Some("incomplete_corrections"));
        assert_eq!(decision.claim, AdvancedSolutionClaim::NotReady);
    }

    #[test]
    fn downgrade_policy_rejects_ready_fixed_claim_without_supporting_evidence() {
        let decision = AdvancedPrereqDecision {
            ready: true,
            refusal_class: None,
            reasons: vec!["rtk_prerequisites_met".to_string()],
        };
        let evidence = AdvancedSolutionEvidence {
            covariance_supported: true,
            residual_supported: true,
            ambiguity_supported: true,
            correction_supported: true,
            integrity_supported: false,
        };
        let decision = apply_downgrade_policy(
            AdvancedMode::Rtk,
            &decision,
            AdvancedSolutionClaim::Fixed,
            Some(&evidence),
        );

        assert_eq!(decision.status, "degraded");
        assert!(decision.downgraded);
        assert_eq!(
            decision.downgrade_reason.as_deref(),
            Some("missing_integrity_evidence")
        );
        assert_eq!(decision.claim, AdvancedSolutionClaim::FallbackNav);
    }

    #[test]
    fn execution_artifact_requires_value_for_executed_status() {
        let artifact = ExecutionArtifact::<DummyPayload> {
            epoch_idx: 3,
            status: ExecutionStatus::Executed,
            reason: None,
            value: None,
        };

        let events = artifact.validate_payload();
        assert!(events.iter().any(|event| event.code == "EXECUTION_ARTIFACT_VALUE_MISSING"));
    }

    #[test]
    fn execution_artifact_rejects_payload_for_not_ready_status() {
        let artifact = ExecutionArtifact {
            epoch_idx: 5,
            status: ExecutionStatus::NotReady,
            reason: Some("missing_baseline".to_string()),
            value: Some(DummyPayload),
        };

        let events = artifact.validate_payload();
        assert!(events.iter().any(|event| event.code == "EXECUTION_ARTIFACT_VALUE_UNEXPECTED"));
    }

    #[test]
    fn advanced_solution_artifact_rejects_strong_claim_without_evidence() {
        let artifact = AdvancedSolutionArtifact {
            epoch_idx: 7,
            mode: AdvancedMode::Rtk,
            status: "accepted_fixed".to_string(),
            downgraded: false,
            downgrade_reason: None,
            prerequisites: ready_prereq(),
            refusal_class: None,
            provenance: AdvancedSolutionProvenance {
                claim: AdvancedSolutionClaim::Fixed,
                ambiguity_state_count: 4,
                correction_source: "broadcast_ephemeris".to_string(),
                fallback_from: None,
                fixed_ratio: Some(3.4),
                measurements: strong_measurements(),
                evidence: AdvancedSolutionEvidence {
                    covariance_supported: true,
                    residual_supported: true,
                    ambiguity_supported: true,
                    correction_supported: true,
                    integrity_supported: false,
                },
            },
            artifact_id: "rtk-advanced-epoch-0000000007".to_string(),
            source_observation_epoch_id: "obs-epoch-0000000007".to_string(),
        };

        let events = artifact.validate_payload();
        assert!(events.iter().any(|event| event.code == "ADVANCED_SOLUTION_EVIDENCE_MISSING"));
    }
}
