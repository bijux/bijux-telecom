#![allow(missing_docs)]

use serde::{Deserialize, Serialize};

use bijux_gnss_core::api::{ArtifactPayloadValidate, DiagnosticEvent, DiagnosticSeverity};

pub const ADVANCED_SUPPORT_MATRIX_VERSION: u32 = 1;

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
pub enum AdvancedMode {
    Rtk,
    Ppp,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
pub enum AdvancedMaturity {
    Scaffolding,
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
    Scaffolding,
    Float,
    Fixed,
    FallbackNav,
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

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AdvancedSolutionProvenance {
    pub claim: AdvancedSolutionClaim,
    pub ambiguity_state_count: usize,
    pub correction_source: String,
    pub fallback_from: Option<String>,
    pub fixed_ratio: Option<f64>,
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
                maturity: AdvancedMaturity::Scaffolding,
                real_solver: false,
                required_inputs: vec![
                    "multi_frequency_obs".to_string(),
                    "products_or_broadcast".to_string(),
                    "time_consistent_epochs".to_string(),
                ],
                notes: "ppp readiness is reported; runtime claims remain conservative".to_string(),
            },
        ],
    }
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
) -> (String, bool, Option<String>, AdvancedSolutionClaim) {
    if decision.ready {
        let status = match claim {
            AdvancedSolutionClaim::Fixed => "accepted_fixed",
            AdvancedSolutionClaim::Float => "accepted_float",
            AdvancedSolutionClaim::Scaffolding => "accepted_scaffolding",
            AdvancedSolutionClaim::FallbackNav => "accepted_fallback",
        };
        return (status.to_string(), false, None, claim);
    }
    let reason = decision.reasons.join(",");
    let downgraded_claim = match mode {
        AdvancedMode::Rtk => AdvancedSolutionClaim::FallbackNav,
        AdvancedMode::Ppp => AdvancedSolutionClaim::Scaffolding,
    };
    ("degraded".to_string(), true, Some(reason), downgraded_claim)
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
        events
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
    fn downgrade_policy_falls_back_when_not_ready() {
        let decision = AdvancedPrereqDecision {
            ready: false,
            refusal_class: Some(AdvancedRefusalClass::InsufficientGeometry),
            reasons: vec!["insufficient_geometry".to_string()],
        };
        let (status, downgraded, reason, claim) =
            apply_downgrade_policy(AdvancedMode::Rtk, &decision, AdvancedSolutionClaim::Fixed);
        assert_eq!(status, "degraded");
        assert!(downgraded);
        assert!(reason.is_some());
        assert_eq!(claim, AdvancedSolutionClaim::FallbackNav);
    }
}
