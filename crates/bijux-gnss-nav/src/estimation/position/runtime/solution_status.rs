use crate::api::{PositionFilterDivergenceReason, PositionSolveRefusalKind};
use bijux_gnss_core::api::{
    is_solution_valid, NavRefusalClass, NavSolutionEpoch, SolutionStatus, SolutionValidity,
};

pub(super) fn deterministic_solution_transition(
    previous: Option<SolutionStatus>,
    proposed: SolutionStatus,
    refusal_class: Option<NavRefusalClass>,
    reused_previous: bool,
) -> SolutionStatus {
    if let Some(refusal_class) = refusal_class {
        if !reused_previous {
            return refusal_status(refusal_class);
        }
        return SolutionStatus::Degraded;
    }
    match (previous, proposed) {
        (_, SolutionStatus::Unavailable)
        | (_, SolutionStatus::Refused)
        | (_, SolutionStatus::IntegrityFailed)
        | (_, SolutionStatus::Diverged) => proposed,
        (_, other) => other,
    }
}

pub(super) fn refusal_status(refusal_class: NavRefusalClass) -> SolutionStatus {
    match refusal_class {
        NavRefusalClass::UnsupportedConstellation
        | NavRefusalClass::MixedConstellationInput
        | NavRefusalClass::InvalidEphemeris
        | NavRefusalClass::PartialDecodedNavigationState => SolutionStatus::Unavailable,
        NavRefusalClass::InsufficientGeometry
        | NavRefusalClass::InvalidSatelliteTime
        | NavRefusalClass::InconsistentObservations
        | NavRefusalClass::ScientificPrerequisitesTooWeak
        | NavRefusalClass::SolverFailure => SolutionStatus::Refused,
    }
}

pub(super) fn solver_refusal_status(
    refusal_kind: PositionSolveRefusalKind,
    refusal_class: NavRefusalClass,
) -> SolutionStatus {
    match refusal_kind {
        PositionSolveRefusalKind::UnderdeterminedRaimExclusion => SolutionStatus::IntegrityFailed,
        PositionSolveRefusalKind::FilterDivergence(_) => SolutionStatus::Diverged,
        PositionSolveRefusalKind::SolverFailure
            if refusal_class == NavRefusalClass::SolverFailure =>
        {
            SolutionStatus::Diverged
        }
        _ => refusal_status(refusal_class),
    }
}

pub(super) fn filter_divergence_explain_reason(reason: PositionFilterDivergenceReason) -> String {
    let label = match reason {
        PositionFilterDivergenceReason::InnovationInconsistency => "innovation_inconsistency",
        PositionFilterDivergenceReason::InnovationGrowth => "innovation_growth",
        PositionFilterDivergenceReason::CovarianceCollapse => "covariance_collapse",
        PositionFilterDivergenceReason::CovarianceDivergence => "covariance_divergence",
        PositionFilterDivergenceReason::ResidualExplosion => "residual_explosion",
    };
    format!("filter_divergence={label}")
}

pub(super) fn override_solution_status(
    mut solution: NavSolutionEpoch,
    status: SolutionStatus,
) -> NavSolutionEpoch {
    solution.status = status;
    solution.lifecycle_state = status.lifecycle_state();
    solution.quality = status.quality_flag();
    solution.valid = is_solution_valid(status);
    solution.validity = if solution.valid { solution.validity } else { SolutionValidity::Invalid };
    solution
}

pub(super) fn mark_integrity_failure(mut solution: NavSolutionEpoch) -> NavSolutionEpoch {
    solution = override_solution_status(solution, SolutionStatus::IntegrityFailed);
    solution.explain_decision = "integrity_failed".to_string();
    solution
}

pub(super) fn default_decision_reason(status: SolutionStatus) -> &'static str {
    match status {
        SolutionStatus::Unavailable => "navigation_solution_unavailable",
        SolutionStatus::Refused => "navigation_solution_refused",
        SolutionStatus::Degraded => "quality_or_geometry_degraded",
        SolutionStatus::IntegrityFailed => "navigation_solution_integrity_failed",
        SolutionStatus::Diverged => "navigation_solution_diverged",
        SolutionStatus::CodeOnly | SolutionStatus::Float | SolutionStatus::Fixed => {
            "navigation_solution_usable"
        }
    }
}

pub(super) fn status_needs_default_decision_reason(
    status: SolutionStatus,
    explain_reasons: &[String],
) -> bool {
    explain_reasons.is_empty()
        || (status.decision_label() != "accepted"
            && explain_reasons.len() == 1
            && explain_reasons[0] == "navigation_solution_usable")
}

pub(super) fn normalized_status_decision_label(
    status: SolutionStatus,
    explain_decision: String,
) -> String {
    if matches!(explain_decision.as_str(), "accepted" | "refused") {
        status.decision_label().to_string()
    } else {
        explain_decision
    }
}
