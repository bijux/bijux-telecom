#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PositionRobustWeighting {
    Disabled,
    Huber { threshold_m: f64 },
    TukeyBiweight { threshold_m: f64 },
}

impl PositionRobustWeighting {
    pub fn disabled() -> Self {
        Self::Disabled
    }

    pub fn huber(threshold_m: f64) -> Self {
        Self::Huber { threshold_m }
    }

    pub fn tukey_biweight(threshold_m: f64) -> Self {
        Self::TukeyBiweight { threshold_m }
    }
}

pub(super) fn robust_weights(
    residuals: &[f64],
    robust_weighting: PositionRobustWeighting,
) -> Vec<f64> {
    residuals.iter().map(|residual_m| robust_weight(residual_m.abs(), robust_weighting)).collect()
}

pub(super) fn robust_weight(residual_abs_m: f64, robust_weighting: PositionRobustWeighting) -> f64 {
    match robust_weighting {
        PositionRobustWeighting::Disabled => 1.0,
        PositionRobustWeighting::Huber { threshold_m } => {
            if residual_abs_m <= threshold_m {
                1.0
            } else {
                threshold_m / residual_abs_m
            }
        }
        PositionRobustWeighting::TukeyBiweight { threshold_m } => {
            if residual_abs_m >= threshold_m {
                0.0
            } else {
                let scaled_residual = residual_abs_m / threshold_m;
                let attenuation = 1.0 - scaled_residual * scaled_residual;
                attenuation * attenuation
            }
        }
    }
}
