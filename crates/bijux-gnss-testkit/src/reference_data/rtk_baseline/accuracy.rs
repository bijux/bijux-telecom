/// Axis and norm errors between an estimated and truth RTK baseline.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RtkBaselineAccuracy {
    /// Signed east error in meters.
    pub east_error_m: f64,
    /// Signed north error in meters.
    pub north_error_m: f64,
    /// Signed up error in meters.
    pub up_error_m: f64,
    /// Horizontal norm of the east/north error in meters.
    pub horizontal_error_m: f64,
    /// Three-dimensional norm of the ENU error in meters.
    pub three_dimensional_error_m: f64,
    /// Largest absolute axis error in meters.
    pub max_axis_error_m: f64,
}

/// Accuracy budget for clean short-baseline RTK truth validation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RtkBaselineAccuracyBudget {
    /// Maximum absolute east error in meters.
    pub max_east_error_m: f64,
    /// Maximum absolute north error in meters.
    pub max_north_error_m: f64,
    /// Maximum absolute up error in meters.
    pub max_up_error_m: f64,
    /// Maximum horizontal error norm in meters.
    pub max_horizontal_error_m: f64,
    /// Maximum three-dimensional error norm in meters.
    pub max_three_dimensional_error_m: f64,
}

impl RtkBaselineAccuracy {
    /// Return whether the measured baseline error fits within the provided budget.
    pub fn satisfies(self, budget: RtkBaselineAccuracyBudget) -> bool {
        self.east_error_m.abs() <= budget.max_east_error_m
            && self.north_error_m.abs() <= budget.max_north_error_m
            && self.up_error_m.abs() <= budget.max_up_error_m
            && self.horizontal_error_m <= budget.max_horizontal_error_m
            && self.three_dimensional_error_m <= budget.max_three_dimensional_error_m
    }
}

/// Compare an estimated ENU baseline against RTK truth.
pub fn rtk_baseline_accuracy(
    estimated_enu_m: [f64; 3],
    truth_enu_m: [f64; 3],
) -> RtkBaselineAccuracy {
    let east_error_m = estimated_enu_m[0] - truth_enu_m[0];
    let north_error_m = estimated_enu_m[1] - truth_enu_m[1];
    let up_error_m = estimated_enu_m[2] - truth_enu_m[2];
    let horizontal_error_m = (east_error_m * east_error_m + north_error_m * north_error_m).sqrt();
    let three_dimensional_error_m =
        (horizontal_error_m * horizontal_error_m + up_error_m * up_error_m).sqrt();
    let max_axis_error_m = east_error_m.abs().max(north_error_m.abs()).max(up_error_m.abs());
    RtkBaselineAccuracy {
        east_error_m,
        north_error_m,
        up_error_m,
        horizontal_error_m,
        three_dimensional_error_m,
        max_axis_error_m,
    }
}

/// Return the clean short-baseline validation budget for centimeter-level RTK accuracy.
pub fn centimeter_level_rtk_baseline_budget() -> RtkBaselineAccuracyBudget {
    RtkBaselineAccuracyBudget {
        max_east_error_m: 0.01,
        max_north_error_m: 0.01,
        max_up_error_m: 0.02,
        max_horizontal_error_m: 0.015,
        max_three_dimensional_error_m: 0.02,
    }
}

#[cfg(test)]
mod tests {
    use super::{centimeter_level_rtk_baseline_budget, rtk_baseline_accuracy, RtkBaselineAccuracy};

    #[test]
    fn rtk_baseline_accuracy_reports_axis_and_norm_errors() {
        let accuracy = rtk_baseline_accuracy([1.01, 1.98, 3.005], [1.0, 2.0, 3.0]);

        assert!((accuracy.east_error_m - 0.01).abs() < 1.0e-12);
        assert!((accuracy.north_error_m + 0.02).abs() < 1.0e-12);
        assert!((accuracy.up_error_m - 0.005).abs() < 1.0e-12);
        assert!(
            (accuracy.horizontal_error_m - (0.01_f64 * 0.01 + 0.02 * 0.02).sqrt()).abs() < 1.0e-12
        );
        assert!((accuracy.max_axis_error_m - 0.02).abs() < 1.0e-12);
    }

    #[test]
    fn centimeter_level_budget_accepts_small_clean_errors() {
        let budget = centimeter_level_rtk_baseline_budget();
        let accuracy = rtk_baseline_accuracy([8.505, -4.243, 1.764], [8.5, -4.25, 1.75]);

        assert!(accuracy.satisfies(budget), "{accuracy:?}");
    }

    #[test]
    fn centimeter_level_budget_rejects_large_axis_error() {
        let budget = centimeter_level_rtk_baseline_budget();
        let accuracy = RtkBaselineAccuracy {
            east_error_m: 0.03,
            north_error_m: 0.0,
            up_error_m: 0.0,
            horizontal_error_m: 0.03,
            three_dimensional_error_m: 0.03,
            max_axis_error_m: 0.03,
        };

        assert!(!accuracy.satisfies(budget));
    }
}
