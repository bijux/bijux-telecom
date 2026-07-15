use bijux_gnss_core::api::{Meters, NavHealthEvent, NavSolutionEpoch, SolutionStatus};

const FIXED_VALIDATED_SIGMA_H_FLOOR_M: f64 = 0.02;
const FIXED_VALIDATED_SIGMA_V_FLOOR_M: f64 = 0.03;
const FIXED_SIGMA_H_FLOOR_M: f64 = 0.10;
const FIXED_SIGMA_V_FLOOR_M: f64 = 0.15;
const FLOAT_SIGMA_H_FLOOR_M: f64 = 0.25;
const FLOAT_SIGMA_V_FLOOR_M: f64 = 0.40;
const CODE_ONLY_SIGMA_H_FLOOR_M: f64 = 3.0;
const CODE_ONLY_SIGMA_V_FLOOR_M: f64 = 5.0;
const DEGRADED_SIGMA_H_FLOOR_M: f64 = 5.0;
const DEGRADED_SIGMA_V_FLOOR_M: f64 = 8.0;

#[derive(Debug, Clone, Copy)]
struct PrecisionReportingPolicy {
    horizontal_sigma_floor_m: f64,
    vertical_sigma_floor_m: f64,
    low_uncertainty_allowed: bool,
    explain_reason: &'static str,
}

pub(super) fn apply_precision_reporting_policy(solution: &mut NavSolutionEpoch) {
    let policy = precision_reporting_policy(solution);
    let scale = precision_reporting_scale(solution, policy);
    if scale <= 1.0 + f64::EPSILON {
        return;
    }

    scale_nav_precision_outputs(solution, scale);
    push_unique_reason(solution, policy.explain_reason);
}

pub(super) fn low_uncertainty_allowed(solution: &NavSolutionEpoch) -> bool {
    precision_reporting_policy(solution).low_uncertainty_allowed
}

fn precision_reporting_policy(solution: &NavSolutionEpoch) -> PrecisionReportingPolicy {
    match solution.status {
        SolutionStatus::Fixed if has_validated_centimeter_precision_support(solution) => {
            PrecisionReportingPolicy {
                horizontal_sigma_floor_m: FIXED_VALIDATED_SIGMA_H_FLOOR_M,
                vertical_sigma_floor_m: FIXED_VALIDATED_SIGMA_V_FLOOR_M,
                low_uncertainty_allowed: true,
                explain_reason: "precision_floor=validated_fixed_solution_minimum",
            }
        }
        SolutionStatus::Fixed => PrecisionReportingPolicy {
            horizontal_sigma_floor_m: FIXED_SIGMA_H_FLOOR_M,
            vertical_sigma_floor_m: FIXED_SIGMA_V_FLOOR_M,
            low_uncertainty_allowed: true,
            explain_reason: "precision_floor=fixed_solution_without_validated_support",
        },
        SolutionStatus::Float => PrecisionReportingPolicy {
            horizontal_sigma_floor_m: FLOAT_SIGMA_H_FLOOR_M,
            vertical_sigma_floor_m: FLOAT_SIGMA_V_FLOOR_M,
            low_uncertainty_allowed: true,
            explain_reason: "precision_floor=float_solution_without_integer_fix",
        },
        SolutionStatus::CodeOnly => PrecisionReportingPolicy {
            horizontal_sigma_floor_m: CODE_ONLY_SIGMA_H_FLOOR_M,
            vertical_sigma_floor_m: CODE_ONLY_SIGMA_V_FLOOR_M,
            low_uncertainty_allowed: false,
            explain_reason: "precision_floor=code_navigation_without_carrier_ambiguity",
        },
        SolutionStatus::Degraded => PrecisionReportingPolicy {
            horizontal_sigma_floor_m: DEGRADED_SIGMA_H_FLOOR_M,
            vertical_sigma_floor_m: DEGRADED_SIGMA_V_FLOOR_M,
            low_uncertainty_allowed: false,
            explain_reason: "precision_floor=degraded_navigation_precision",
        },
        SolutionStatus::Unavailable
        | SolutionStatus::Refused
        | SolutionStatus::IntegrityFailed
        | SolutionStatus::Diverged => PrecisionReportingPolicy {
            horizontal_sigma_floor_m: DEGRADED_SIGMA_H_FLOOR_M,
            vertical_sigma_floor_m: DEGRADED_SIGMA_V_FLOOR_M,
            low_uncertainty_allowed: false,
            explain_reason: "precision_floor=invalid_navigation_solution",
        },
    }
}

fn has_validated_centimeter_precision_support(solution: &NavSolutionEpoch) -> bool {
    let corrections_supported = solution.explain_reasons.iter().any(|reason| {
        reason.starts_with("ionosphere_correction=") && reason != "ionosphere_uncorrected"
    }) && solution
        .explain_reasons
        .iter()
        .any(|reason| reason == "troposphere_correction=saastamoinen");
    let validation_supported = solution.valid
        && solution.refusal_class.is_none()
        && solution.integrity_hpl_m.is_some()
        && solution.integrity_vpl_m.is_some()
        && !solution.health.iter().any(|event| {
            matches!(
                event,
                NavHealthEvent::CovarianceDiverged { .. }
                    | NavHealthEvent::CommonCodeDopplerAnomaly { .. }
                    | NavHealthEvent::ReplayTimingAnomaly { .. }
                    | NavHealthEvent::ConstellationClockInconsistency { .. }
                    | NavHealthEvent::ResidualTemporalCorrelation { .. }
                    | NavHealthEvent::ImpossibleGeometry { .. }
                    | NavHealthEvent::SatelliteClockAnomaly { .. }
            )
        });
    corrections_supported && validation_supported
}

fn precision_reporting_scale(solution: &NavSolutionEpoch, policy: PrecisionReportingPolicy) -> f64 {
    let mut scale = 1.0_f64;
    if let Some(sigma_h_m) = solution.sigma_h_m.map(|value| value.0) {
        if sigma_h_m.is_finite() && sigma_h_m > 0.0 && sigma_h_m < policy.horizontal_sigma_floor_m {
            scale = scale.max((policy.horizontal_sigma_floor_m / sigma_h_m).powi(2));
        }
    }
    if let Some(sigma_v_m) = solution.sigma_v_m.map(|value| value.0) {
        if sigma_v_m.is_finite() && sigma_v_m > 0.0 && sigma_v_m < policy.vertical_sigma_floor_m {
            scale = scale.max((policy.vertical_sigma_floor_m / sigma_v_m).powi(2));
        }
    }
    scale
}

fn scale_nav_precision_outputs(solution: &mut NavSolutionEpoch, covariance_scale: f64) {
    let sigma_scale = covariance_scale.sqrt();
    scale_covariance_in_place(&mut solution.position_covariance_ecef_m2, covariance_scale);
    scale_optional_meters(&mut solution.sigma_e_m, sigma_scale);
    scale_optional_meters(&mut solution.sigma_n_m, sigma_scale);
    scale_optional_meters(&mut solution.sigma_u_m, sigma_scale);
    scale_optional_meters(&mut solution.sigma_h_m, sigma_scale);
    scale_optional_meters(&mut solution.sigma_v_m, sigma_scale);
    scale_optional_meters(&mut solution.horizontal_error_ellipse_major_axis_m, sigma_scale);
    scale_optional_meters(&mut solution.horizontal_error_ellipse_minor_axis_m, sigma_scale);
    scale_optional_f64(&mut solution.integrity_hpl_m, sigma_scale);
    scale_optional_f64(&mut solution.integrity_vpl_m, sigma_scale);
}

fn scale_covariance_in_place(covariance: &mut Option<[[f64; 3]; 3]>, covariance_scale: f64) {
    let Some(covariance) = covariance.as_mut() else {
        return;
    };
    for row in covariance.iter_mut() {
        for element in row.iter_mut() {
            *element *= covariance_scale;
        }
    }
}

fn scale_optional_meters(value: &mut Option<Meters>, scale: f64) {
    if let Some(value) = value.as_mut() {
        value.0 *= scale;
    }
}

fn scale_optional_f64(value: &mut Option<f64>, scale: f64) {
    if let Some(value) = value.as_mut() {
        *value *= scale;
    }
}

pub(super) fn push_unique_reason(solution: &mut NavSolutionEpoch, reason: &'static str) {
    if !solution.explain_reasons.iter().any(|existing| existing == reason) {
        solution.explain_reasons.push(reason.to_string());
    }
}
