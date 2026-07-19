//! Synthetic RTK short-baseline fixtures and accuracy budgets.

mod accuracy;
mod scenario;

pub use accuracy::{
    centimeter_level_rtk_baseline_budget, rtk_baseline_accuracy, RtkBaselineAccuracy,
    RtkBaselineAccuracyBudget,
};
pub use scenario::{
    clean_gps_l1_short_baseline_case, multipath_gps_l1_short_baseline_case,
    noisy_gps_l1_short_baseline_case, GpsL1RtkBaselineCase,
};
