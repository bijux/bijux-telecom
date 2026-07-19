use bijux_gnss_core::api::{AcqResult, AcqUncertaintyCovariance};

const LOG_RESPONSE_FLOOR_RATIO: f64 = 1.0e-6;
const CURVATURE_EPSILON: f64 = 1.0e-12;

#[derive(Debug, Clone, Copy)]
pub(super) struct LocalAcquisitionLikelihoodSurface {
    pub(super) doppler_cross_section: [f32; 3],
    pub(super) code_phase_cross_section: [f32; 3],
    pub(super) values: [[f32; 3]; 3],
}

#[derive(Debug, Clone, Copy)]
pub(super) struct LocalAcquisitionLikelihoodVolume {
    pub(super) values: [[[f32; 3]; 3]; 3],
}

pub(super) fn estimate_log_likelihood_covariance_2x2(
    surface: &LocalAcquisitionLikelihoodSurface,
    noise_floor: f64,
    doppler_step_hz: f64,
    code_phase_step_samples: f64,
) -> Option<AcqUncertaintyCovariance> {
    let fallback = || {
        Some(AcqUncertaintyCovariance {
            doppler_variance_hz2: estimate_log_likelihood_axis_variance(
                surface.values[0][1] as f64,
                surface.values[1][1] as f64,
                surface.values[2][1] as f64,
                noise_floor,
                doppler_step_hz,
            )?,
            doppler_code_phase_covariance_hz_samples: 0.0,
            code_phase_variance_samples2: estimate_log_likelihood_axis_variance(
                surface.values[1][0] as f64,
                surface.values[1][1] as f64,
                surface.values[1][2] as f64,
                noise_floor,
                code_phase_step_samples,
            )?,
            doppler_rate_variance_hz2_per_s2: None,
            doppler_doppler_rate_covariance_hz2_per_s: None,
            code_phase_doppler_rate_covariance_samples_hz_per_s: None,
        })
    };
    let phi = log_likelihood_surface_values_2x2(&surface.values, noise_floor)?;
    let hessian = [
        [
            centered_second_derivative(phi[0][1], phi[1][1], phi[2][1], doppler_step_hz)?,
            centered_mixed_derivative(
                phi[0][0],
                phi[0][2],
                phi[2][0],
                phi[2][2],
                doppler_step_hz,
                code_phase_step_samples,
            )?,
        ],
        [
            centered_mixed_derivative(
                phi[0][0],
                phi[0][2],
                phi[2][0],
                phi[2][2],
                doppler_step_hz,
                code_phase_step_samples,
            )?,
            centered_second_derivative(phi[1][0], phi[1][1], phi[1][2], code_phase_step_samples)?,
        ],
    ];
    let Some(covariance) = invert_2x2(hessian) else {
        return fallback();
    };
    if covariance[0][0] <= f64::EPSILON || covariance[1][1] <= f64::EPSILON {
        return fallback();
    }
    Some(AcqUncertaintyCovariance {
        doppler_variance_hz2: covariance[0][0],
        doppler_code_phase_covariance_hz_samples: covariance[0][1],
        code_phase_variance_samples2: covariance[1][1],
        doppler_rate_variance_hz2_per_s2: None,
        doppler_doppler_rate_covariance_hz2_per_s: None,
        code_phase_doppler_rate_covariance_samples_hz_per_s: None,
    })
}

pub(super) fn estimate_log_likelihood_covariance_3x3(
    volume: &LocalAcquisitionLikelihoodVolume,
    noise_floor: f64,
    doppler_step_hz: f64,
    code_phase_step_samples: f64,
    doppler_rate_step_hz_per_s: f64,
) -> Option<AcqUncertaintyCovariance> {
    let fallback = || {
        let center_surface = LocalAcquisitionLikelihoodSurface {
            doppler_cross_section: [
                volume.values[1][0][1],
                volume.values[1][1][1],
                volume.values[1][2][1],
            ],
            code_phase_cross_section: volume.values[1][1],
            values: volume.values[1],
        };
        let mut covariance = estimate_log_likelihood_covariance_2x2(
            &center_surface,
            noise_floor,
            doppler_step_hz,
            code_phase_step_samples,
        )?;
        covariance.doppler_rate_variance_hz2_per_s2 = Some(estimate_log_likelihood_axis_variance(
            volume.values[0][1][1] as f64,
            volume.values[1][1][1] as f64,
            volume.values[2][1][1] as f64,
            noise_floor,
            doppler_rate_step_hz_per_s,
        )?);
        covariance.doppler_doppler_rate_covariance_hz2_per_s = Some(0.0);
        covariance.code_phase_doppler_rate_covariance_samples_hz_per_s = Some(0.0);
        Some(covariance)
    };
    let phi = log_likelihood_surface_values_3x3(&volume.values, noise_floor)?;
    let hessian = [
        [
            centered_second_derivative(phi[1][0][1], phi[1][1][1], phi[1][2][1], doppler_step_hz)?,
            centered_mixed_derivative(
                phi[1][0][0],
                phi[1][0][2],
                phi[1][2][0],
                phi[1][2][2],
                doppler_step_hz,
                code_phase_step_samples,
            )?,
            centered_mixed_derivative(
                phi[0][0][1],
                phi[0][2][1],
                phi[2][0][1],
                phi[2][2][1],
                doppler_step_hz,
                doppler_rate_step_hz_per_s,
            )?,
        ],
        [
            centered_mixed_derivative(
                phi[1][0][0],
                phi[1][0][2],
                phi[1][2][0],
                phi[1][2][2],
                doppler_step_hz,
                code_phase_step_samples,
            )?,
            centered_second_derivative(
                phi[1][1][0],
                phi[1][1][1],
                phi[1][1][2],
                code_phase_step_samples,
            )?,
            centered_mixed_derivative(
                phi[0][1][0],
                phi[0][1][2],
                phi[2][1][0],
                phi[2][1][2],
                code_phase_step_samples,
                doppler_rate_step_hz_per_s,
            )?,
        ],
        [
            centered_mixed_derivative(
                phi[0][0][1],
                phi[0][2][1],
                phi[2][0][1],
                phi[2][2][1],
                doppler_step_hz,
                doppler_rate_step_hz_per_s,
            )?,
            centered_mixed_derivative(
                phi[0][1][0],
                phi[0][1][2],
                phi[2][1][0],
                phi[2][1][2],
                code_phase_step_samples,
                doppler_rate_step_hz_per_s,
            )?,
            centered_second_derivative(
                phi[0][1][1],
                phi[1][1][1],
                phi[2][1][1],
                doppler_rate_step_hz_per_s,
            )?,
        ],
    ];
    let Some(covariance) = invert_3x3(hessian) else {
        return fallback();
    };
    if covariance[0][0] <= f64::EPSILON
        || covariance[1][1] <= f64::EPSILON
        || covariance[2][2] <= f64::EPSILON
    {
        return fallback();
    }
    Some(AcqUncertaintyCovariance {
        doppler_variance_hz2: covariance[0][0],
        doppler_code_phase_covariance_hz_samples: covariance[0][1],
        code_phase_variance_samples2: covariance[1][1],
        doppler_rate_variance_hz2_per_s2: Some(covariance[2][2]),
        doppler_doppler_rate_covariance_hz2_per_s: Some(covariance[0][2]),
        code_phase_doppler_rate_covariance_samples_hz_per_s: Some(covariance[1][2]),
    })
}

pub(super) fn estimate_log_likelihood_covariance_from_refinement_axes(
    candidate: &AcqResult,
    noise_floor: f64,
    doppler_step_hz: f64,
) -> Option<AcqUncertaintyCovariance> {
    let doppler_refinement = candidate.doppler_refinement.as_ref()?;
    let code_phase_refinement = candidate.code_phase_refinement.as_ref()?;
    Some(AcqUncertaintyCovariance {
        doppler_variance_hz2: estimate_log_likelihood_axis_variance(
            doppler_refinement.left_peak_mean_ratio as f64,
            doppler_refinement.center_peak_mean_ratio as f64,
            doppler_refinement.right_peak_mean_ratio as f64,
            1.0,
            doppler_step_hz,
        )?,
        doppler_code_phase_covariance_hz_samples: 0.0,
        code_phase_variance_samples2: estimate_log_likelihood_axis_variance(
            code_phase_refinement.left_correlation_norm as f64,
            code_phase_refinement.center_correlation_norm as f64,
            code_phase_refinement.right_correlation_norm as f64,
            noise_floor,
            1.0,
        )?,
        doppler_rate_variance_hz2_per_s2: None,
        doppler_doppler_rate_covariance_hz2_per_s: None,
        code_phase_doppler_rate_covariance_samples_hz_per_s: None,
    })
}

fn log_likelihood_surface_values_2x2(
    values: &[[f32; 3]; 3],
    noise_floor: f64,
) -> Option<[[f64; 3]; 3]> {
    let center_excess = excess_likelihood_response(values[1][1] as f64, noise_floor, None)?;
    let floor = Some(center_excess);
    let mut phi = [[0.0_f64; 3]; 3];
    for doppler_index in 0..3 {
        for code_index in 0..3 {
            phi[doppler_index][code_index] = negative_log_likelihood_response(
                values[doppler_index][code_index] as f64,
                noise_floor,
                center_excess,
                floor,
            )?;
        }
    }
    Some(phi)
}

fn log_likelihood_surface_values_3x3(
    values: &[[[f32; 3]; 3]; 3],
    noise_floor: f64,
) -> Option<[[[f64; 3]; 3]; 3]> {
    let center_excess = excess_likelihood_response(values[1][1][1] as f64, noise_floor, None)?;
    let floor = Some(center_excess);
    let mut phi = [[[0.0_f64; 3]; 3]; 3];
    for rate_index in 0..3 {
        for doppler_index in 0..3 {
            for code_index in 0..3 {
                phi[rate_index][doppler_index][code_index] = negative_log_likelihood_response(
                    values[rate_index][doppler_index][code_index] as f64,
                    noise_floor,
                    center_excess,
                    floor,
                )?;
            }
        }
    }
    Some(phi)
}

fn negative_log_likelihood_response(
    value: f64,
    noise_floor: f64,
    center_excess: f64,
    floor_reference: Option<f64>,
) -> Option<f64> {
    let excess = excess_likelihood_response(value, noise_floor, floor_reference)?;
    let normalized = excess / center_excess;
    if !normalized.is_finite() || normalized <= 0.0 {
        return None;
    }
    Some(-normalized.ln())
}

fn excess_likelihood_response(
    value: f64,
    noise_floor: f64,
    floor_reference: Option<f64>,
) -> Option<f64> {
    if !value.is_finite() || !noise_floor.is_finite() {
        return None;
    }
    let epsilon =
        floor_reference.map(|reference| reference * LOG_RESPONSE_FLOOR_RATIO).unwrap_or(1.0e-12);
    let excess = (value - noise_floor).max(epsilon);
    if !excess.is_finite() || excess <= 0.0 {
        return None;
    }
    Some(excess)
}

fn estimate_log_likelihood_axis_variance(
    left: f64,
    center: f64,
    right: f64,
    noise_floor: f64,
    step: f64,
) -> Option<f64> {
    let center_excess = excess_likelihood_response(center, noise_floor, None)?;
    let floor = Some(center_excess);
    let phi_left = negative_log_likelihood_response(left, noise_floor, center_excess, floor)?;
    let phi_center = negative_log_likelihood_response(center, noise_floor, center_excess, floor)?;
    let phi_right = negative_log_likelihood_response(right, noise_floor, center_excess, floor)?;
    let curvature = centered_second_derivative(phi_left, phi_center, phi_right, step)?;
    let variance = 1.0 / curvature;
    (variance.is_finite() && variance > f64::EPSILON).then_some(variance)
}

fn centered_second_derivative(left: f64, center: f64, right: f64, step: f64) -> Option<f64> {
    if !left.is_finite()
        || !center.is_finite()
        || !right.is_finite()
        || !step.is_finite()
        || step <= f64::EPSILON
    {
        return None;
    }
    let derivative = (left - (2.0 * center) + right) / step.powi(2);
    (derivative.is_finite() && derivative > CURVATURE_EPSILON).then_some(derivative)
}

fn centered_mixed_derivative(
    lower_lower: f64,
    lower_upper: f64,
    upper_lower: f64,
    upper_upper: f64,
    step_a: f64,
    step_b: f64,
) -> Option<f64> {
    if !lower_lower.is_finite()
        || !lower_upper.is_finite()
        || !upper_lower.is_finite()
        || !upper_upper.is_finite()
        || !step_a.is_finite()
        || !step_b.is_finite()
        || step_a <= f64::EPSILON
        || step_b <= f64::EPSILON
    {
        return None;
    }
    let derivative =
        (upper_upper - upper_lower - lower_upper + lower_lower) / (4.0 * step_a * step_b);
    derivative.is_finite().then_some(derivative)
}

fn invert_2x2(matrix: [[f64; 2]; 2]) -> Option<[[f64; 2]; 2]> {
    let determinant = matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0];
    if !determinant.is_finite() || determinant.abs() <= CURVATURE_EPSILON {
        return None;
    }
    Some([
        [matrix[1][1] / determinant, -matrix[0][1] / determinant],
        [-matrix[1][0] / determinant, matrix[0][0] / determinant],
    ])
}

fn invert_3x3(matrix: [[f64; 3]; 3]) -> Option<[[f64; 3]; 3]> {
    let determinant = matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1])
        - matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
        + matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]);
    if !determinant.is_finite() || determinant.abs() <= CURVATURE_EPSILON {
        return None;
    }
    Some([
        [
            (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1]) / determinant,
            (matrix[0][2] * matrix[2][1] - matrix[0][1] * matrix[2][2]) / determinant,
            (matrix[0][1] * matrix[1][2] - matrix[0][2] * matrix[1][1]) / determinant,
        ],
        [
            (matrix[1][2] * matrix[2][0] - matrix[1][0] * matrix[2][2]) / determinant,
            (matrix[0][0] * matrix[2][2] - matrix[0][2] * matrix[2][0]) / determinant,
            (matrix[0][2] * matrix[1][0] - matrix[0][0] * matrix[1][2]) / determinant,
        ],
        [
            (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]) / determinant,
            (matrix[0][1] * matrix[2][0] - matrix[0][0] * matrix[2][1]) / determinant,
            (matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0]) / determinant,
        ],
    ])
}
