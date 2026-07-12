use bijux_gnss_core::api::ecef_to_geodetic;
use serde::{Deserialize, Serialize};

const EXPECTED_95_COVERAGE_RATE: f64 = 0.95;
const HORIZONTAL_95_MAHALANOBIS2_THRESHOLD: f64 = 5.991_464_547_107_979;
const POSITION_95_MAHALANOBIS2_THRESHOLD: f64 = 7.814_727_903_251_179;
const VERTICAL_95_Z_THRESHOLD: f64 = 1.959_963_984_540_054;
const COVERAGE_CLASSIFICATION_Z_SCORE: f64 = 1.959_963_984_540_054;
const MIN_CLASSIFIED_SAMPLE_COUNT: usize = 20;

/// Empirical covariance-coverage classification.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum CovarianceCoverageClass {
    /// Too few comparable epochs were available to classify coverage behavior.
    InsufficientData,
    /// Observed coverage matches the expected confidence rate within sampling tolerance.
    Nominal,
    /// Observed coverage falls below the expected rate, so the predicted bound is too tight.
    Optimistic,
    /// Observed coverage exceeds the expected rate, so the predicted bound is looser than needed.
    Conservative,
}

/// Empirical coverage summary for one predicted confidence bound.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct CovarianceCoverageRate {
    /// Number of epochs that contributed to this coverage check.
    pub sample_count: usize,
    /// Number of epochs whose empirical error fell inside the predicted bound.
    pub inside_count: usize,
    /// Expected fraction of epochs inside the bound.
    pub expected_rate: f64,
    /// Observed fraction of epochs inside the bound.
    pub observed_rate: Option<f64>,
    /// Classification tolerance derived from the finite sample count.
    pub tolerance_rate: Option<f64>,
    /// Empirical coverage classification for this bound.
    pub classification: CovarianceCoverageClass,
}

/// Truth-matched covariance realism summary for navigation position outputs.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct CovarianceRealismReport {
    /// Number of truth-matched epochs inspected.
    pub total_epoch_count: usize,
    /// Matched epochs that carried a position covariance matrix.
    pub covariance_epoch_count: usize,
    /// Matched epochs whose covariance could not support full horizontal and 3D realism checks.
    pub unusable_covariance_epoch_count: usize,
    /// Empirical coverage of the predicted horizontal 95% confidence ellipse.
    pub horizontal_95: CovarianceCoverageRate,
    /// Empirical coverage of the predicted vertical 95% confidence interval.
    pub vertical_95: CovarianceCoverageRate,
    /// Empirical coverage of the predicted 3D 95% confidence ellipsoid.
    pub position_3d_95: CovarianceCoverageRate,
    /// Mean horizontal 2D normalized position error squared.
    pub horizontal_nees_mean: Option<f64>,
    /// Mean vertical normalized position error squared.
    pub vertical_normalized_error_squared_mean: Option<f64>,
    /// Mean 3D normalized position error squared.
    pub position_nees_mean: Option<f64>,
    /// Human-readable realism warnings derived from the empirical coverage checks.
    pub warnings: Vec<String>,
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct CovarianceRealismEpochSample {
    pub(crate) receiver_ecef_m: [f64; 3],
    pub(crate) east_m: f64,
    pub(crate) north_m: f64,
    pub(crate) up_m: f64,
    pub(crate) position_covariance_ecef_m2: Option<[[f64; 3]; 3]>,
}

pub(crate) fn evaluate_covariance_realism(
    samples: &[CovarianceRealismEpochSample],
) -> CovarianceRealismReport {
    let mut covariance_epoch_count = 0;
    let mut unusable_covariance_epoch_count = 0;
    let mut horizontal_inside_count = 0;
    let mut horizontal_sample_count = 0;
    let mut vertical_inside_count = 0;
    let mut vertical_sample_count = 0;
    let mut position_inside_count = 0;
    let mut position_sample_count = 0;
    let mut horizontal_nees_values = Vec::new();
    let mut vertical_normalized_error_squared_values = Vec::new();
    let mut position_nees_values = Vec::new();

    for sample in samples {
        let Some(covariance_ecef_m2) = sample.position_covariance_ecef_m2 else {
            continue;
        };
        covariance_epoch_count += 1;

        let Some(covariance_enu_m2) =
            covariance_ecef_to_enu(sample.receiver_ecef_m, covariance_ecef_m2)
        else {
            unusable_covariance_epoch_count += 1;
            continue;
        };

        let mut full_covariance_usable = false;
        if let Some(horizontal_mahalanobis2) = horizontal_mahalanobis2(
            sample.east_m,
            sample.north_m,
            covariance_enu_m2,
        ) {
            horizontal_sample_count += 1;
            if horizontal_mahalanobis2 <= HORIZONTAL_95_MAHALANOBIS2_THRESHOLD + f64::EPSILON {
                horizontal_inside_count += 1;
            }
            horizontal_nees_values.push(horizontal_mahalanobis2);
            full_covariance_usable = true;
        }

        if let Some(vertical_normalized_error_squared) =
            vertical_normalized_error_squared(sample.up_m, covariance_enu_m2)
        {
            vertical_sample_count += 1;
            if vertical_normalized_error_squared
                <= VERTICAL_95_Z_THRESHOLD * VERTICAL_95_Z_THRESHOLD + f64::EPSILON
            {
                vertical_inside_count += 1;
            }
            vertical_normalized_error_squared_values.push(vertical_normalized_error_squared);
        }

        if let Some(position_mahalanobis2) =
            position_mahalanobis2(sample.east_m, sample.north_m, sample.up_m, covariance_enu_m2)
        {
            position_sample_count += 1;
            if position_mahalanobis2 <= POSITION_95_MAHALANOBIS2_THRESHOLD + f64::EPSILON {
                position_inside_count += 1;
            }
            position_nees_values.push(position_mahalanobis2);
            full_covariance_usable = true;
        }

        if !full_covariance_usable {
            unusable_covariance_epoch_count += 1;
        }
    }

    let horizontal_95 =
        coverage_rate(horizontal_sample_count, horizontal_inside_count, "horizontal_95");
    let vertical_95 = coverage_rate(vertical_sample_count, vertical_inside_count, "vertical_95");
    let position_3d_95 =
        coverage_rate(position_sample_count, position_inside_count, "position_3d_95");

    let mut warnings = Vec::new();
    warning_for_coverage("horizontal 95%", &horizontal_95, &mut warnings);
    warning_for_coverage("vertical 95%", &vertical_95, &mut warnings);
    warning_for_coverage("3d 95%", &position_3d_95, &mut warnings);

    CovarianceRealismReport {
        total_epoch_count: samples.len(),
        covariance_epoch_count,
        unusable_covariance_epoch_count,
        horizontal_95,
        vertical_95,
        position_3d_95,
        horizontal_nees_mean: mean(&horizontal_nees_values),
        vertical_normalized_error_squared_mean: mean(&vertical_normalized_error_squared_values),
        position_nees_mean: mean(&position_nees_values),
        warnings,
    }
}

fn warning_for_coverage(
    label: &str,
    coverage: &CovarianceCoverageRate,
    warnings: &mut Vec<String>,
) {
    let Some(observed_rate) = coverage.observed_rate else {
        return;
    };
    let Some(tolerance_rate) = coverage.tolerance_rate else {
        return;
    };
    match coverage.classification {
        CovarianceCoverageClass::Optimistic => warnings.push(format!(
            "{label} covariance is optimistic: observed coverage {:.1}% across {} epochs, expected {:.1}% +/- {:.1}%",
            observed_rate * 100.0,
            coverage.sample_count,
            coverage.expected_rate * 100.0,
            tolerance_rate * 100.0
        )),
        CovarianceCoverageClass::Conservative => warnings.push(format!(
            "{label} covariance is conservative: observed coverage {:.1}% across {} epochs, expected {:.1}% +/- {:.1}%",
            observed_rate * 100.0,
            coverage.sample_count,
            coverage.expected_rate * 100.0,
            tolerance_rate * 100.0
        )),
        CovarianceCoverageClass::InsufficientData | CovarianceCoverageClass::Nominal => {}
    }
}

fn coverage_rate(
    sample_count: usize,
    inside_count: usize,
    _label: &str,
) -> CovarianceCoverageRate {
    let observed_rate =
        (sample_count > 0).then_some(inside_count as f64 / sample_count as f64);
    let tolerance_rate = (sample_count >= MIN_CLASSIFIED_SAMPLE_COUNT).then_some(
        COVERAGE_CLASSIFICATION_Z_SCORE
            * (EXPECTED_95_COVERAGE_RATE * (1.0 - EXPECTED_95_COVERAGE_RATE) / sample_count as f64)
                .sqrt(),
    );
    let classification = match (observed_rate, tolerance_rate) {
        (Some(observed_rate), Some(tolerance_rate))
            if observed_rate + f64::EPSILON < EXPECTED_95_COVERAGE_RATE - tolerance_rate =>
        {
            CovarianceCoverageClass::Optimistic
        }
        (Some(observed_rate), Some(tolerance_rate))
            if observed_rate > EXPECTED_95_COVERAGE_RATE + tolerance_rate + f64::EPSILON =>
        {
            CovarianceCoverageClass::Conservative
        }
        (Some(_), Some(_)) => CovarianceCoverageClass::Nominal,
        _ => CovarianceCoverageClass::InsufficientData,
    };

    CovarianceCoverageRate {
        sample_count,
        inside_count,
        expected_rate: EXPECTED_95_COVERAGE_RATE,
        observed_rate,
        tolerance_rate,
        classification,
    }
}

fn mean(values: &[f64]) -> Option<f64> {
    (!values.is_empty()).then_some(values.iter().sum::<f64>() / values.len() as f64)
}

fn vertical_normalized_error_squared(up_m: f64, covariance_enu_m2: [[f64; 3]; 3]) -> Option<f64> {
    let variance_u_m2 = covariance_enu_m2[2][2];
    if !variance_u_m2.is_finite() || variance_u_m2 <= 0.0 {
        return None;
    }
    Some((up_m * up_m) / variance_u_m2)
}

fn horizontal_mahalanobis2(
    east_m: f64,
    north_m: f64,
    covariance_enu_m2: [[f64; 3]; 3],
) -> Option<f64> {
    let covariance_2d = [
        [covariance_enu_m2[0][0], covariance_enu_m2[0][1]],
        [covariance_enu_m2[1][0], covariance_enu_m2[1][1]],
    ];
    let inverse = invert_2x2(covariance_2d)?;
    Some(
        east_m * (inverse[0][0] * east_m + inverse[0][1] * north_m)
            + north_m * (inverse[1][0] * east_m + inverse[1][1] * north_m),
    )
}

fn position_mahalanobis2(
    east_m: f64,
    north_m: f64,
    up_m: f64,
    covariance_enu_m2: [[f64; 3]; 3],
) -> Option<f64> {
    let inverse = invert_3x3(covariance_enu_m2)?;
    Some(
        east_m * (inverse[0][0] * east_m + inverse[0][1] * north_m + inverse[0][2] * up_m)
            + north_m
                * (inverse[1][0] * east_m + inverse[1][1] * north_m + inverse[1][2] * up_m)
            + up_m * (inverse[2][0] * east_m + inverse[2][1] * north_m + inverse[2][2] * up_m),
    )
}

fn invert_2x2(matrix: [[f64; 2]; 2]) -> Option<[[f64; 2]; 2]> {
    if !matrix.iter().flatten().all(|value| value.is_finite()) {
        return None;
    }
    let determinant = matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0];
    if determinant.abs() <= 1.0e-12 {
        return None;
    }
    Some([
        [matrix[1][1] / determinant, -matrix[0][1] / determinant],
        [-matrix[1][0] / determinant, matrix[0][0] / determinant],
    ])
}

fn invert_3x3(matrix: [[f64; 3]; 3]) -> Option<[[f64; 3]; 3]> {
    if !matrix.iter().flatten().all(|value| value.is_finite()) {
        return None;
    }

    let determinant = matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1])
        - matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
        + matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]);
    if determinant.abs() <= 1.0e-12 {
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

fn covariance_ecef_to_enu(
    receiver_ecef_m: [f64; 3],
    covariance_ecef_m2: [[f64; 3]; 3],
) -> Option<[[f64; 3]; 3]> {
    if !covariance_ecef_m2.iter().flatten().all(|value| value.is_finite()) {
        return None;
    }

    let receiver_radius_m =
        receiver_ecef_m.iter().map(|component| component * component).sum::<f64>().sqrt();
    if !receiver_radius_m.is_finite() || receiver_radius_m <= 0.0 {
        return None;
    }

    let (lat_deg, lon_deg, _alt_m) =
        ecef_to_geodetic(receiver_ecef_m[0], receiver_ecef_m[1], receiver_ecef_m[2]);
    if !lat_deg.is_finite() || !lon_deg.is_finite() {
        return None;
    }

    let rotation = ecef_to_enu_rotation(lat_deg.to_radians(), lon_deg.to_radians());
    let mut covariance_enu = [[0.0_f64; 3]; 3];
    for row in 0..3 {
        for col in 0..3 {
            for inner_row in 0..3 {
                for inner_col in 0..3 {
                    covariance_enu[row][col] += rotation[row][inner_row]
                        * covariance_ecef_m2[inner_row][inner_col]
                        * rotation[col][inner_col];
                }
            }
        }
    }
    Some(covariance_enu)
}

fn ecef_to_enu_rotation(lat_rad: f64, lon_rad: f64) -> [[f64; 3]; 3] {
    let sin_lat = lat_rad.sin();
    let cos_lat = lat_rad.cos();
    let sin_lon = lon_rad.sin();
    let cos_lon = lon_rad.cos();

    [
        [-sin_lon, cos_lon, 0.0],
        [-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat],
        [cos_lat * cos_lon, cos_lat * sin_lon, sin_lat],
    ]
}

#[cfg(test)]
mod tests {
    use super::{
        evaluate_covariance_realism, CovarianceCoverageClass, CovarianceRealismEpochSample,
    };

    fn sample(
        east_m: f64,
        north_m: f64,
        up_m: f64,
        position_covariance_ecef_m2: [[f64; 3]; 3],
    ) -> CovarianceRealismEpochSample {
        CovarianceRealismEpochSample {
            receiver_ecef_m: [6_378_137.0, 0.0, 0.0],
            east_m,
            north_m,
            up_m,
            position_covariance_ecef_m2: Some(position_covariance_ecef_m2),
        }
    }

    #[test]
    fn covariance_realism_classifies_nominal_coverage() {
        let mut samples = Vec::new();
        for _ in 0..38 {
            samples.push(sample(0.5, 0.25, 0.5, [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]));
        }
        for _ in 0..2 {
            samples.push(sample(3.5, 0.0, 3.0, [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]));
        }

        let report = evaluate_covariance_realism(&samples);

        assert_eq!(report.horizontal_95.classification, CovarianceCoverageClass::Nominal);
        assert_eq!(report.vertical_95.classification, CovarianceCoverageClass::Nominal);
        assert_eq!(report.position_3d_95.classification, CovarianceCoverageClass::Nominal);
        assert!(report.warnings.is_empty(), "{:?}", report.warnings);
    }

    #[test]
    fn covariance_realism_classifies_optimistic_coverage() {
        let mut samples = Vec::new();
        for _ in 0..30 {
            samples.push(sample(0.5, 0.25, 0.5, [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]));
        }
        for _ in 0..10 {
            samples.push(sample(3.5, 0.0, 3.0, [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]));
        }

        let report = evaluate_covariance_realism(&samples);

        assert_eq!(report.horizontal_95.classification, CovarianceCoverageClass::Optimistic);
        assert_eq!(report.vertical_95.classification, CovarianceCoverageClass::Optimistic);
        assert_eq!(report.position_3d_95.classification, CovarianceCoverageClass::Optimistic);
        assert_eq!(report.warnings.len(), 3);
    }

    #[test]
    fn covariance_realism_classifies_conservative_coverage() {
        let samples = (0..100)
            .map(|_| sample(0.1, 0.05, 0.1, [[4.0, 0.0, 0.0], [0.0, 4.0, 0.0], [0.0, 0.0, 4.0]]))
            .collect::<Vec<_>>();

        let report = evaluate_covariance_realism(&samples);

        assert_eq!(report.horizontal_95.classification, CovarianceCoverageClass::Conservative);
        assert_eq!(report.vertical_95.classification, CovarianceCoverageClass::Conservative);
        assert_eq!(report.position_3d_95.classification, CovarianceCoverageClass::Conservative);
        assert_eq!(report.warnings.len(), 3);
    }
}
