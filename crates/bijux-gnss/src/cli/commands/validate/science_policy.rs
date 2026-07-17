use super::*;

pub(crate) fn validation_science_policy(
    profile: &ReceiverConfig,
) -> bijux_gnss_infra::api::receiver::ValidationSciencePolicy {
    bijux_gnss_infra::api::receiver::ValidationSciencePolicy {
        min_mean_cn0_dbhz: profile.navigation.science_thresholds.min_mean_cn0_dbhz,
        max_pdop: profile.navigation.science_thresholds.max_pdop,
        max_gdop: profile.navigation.science_thresholds.max_gdop,
        max_residual_rms_m: profile.navigation.science_thresholds.max_residual_rms_m,
        min_used_satellites: profile.navigation.science_thresholds.min_used_satellites,
        min_lock_ratio: profile.navigation.science_thresholds.min_lock_ratio,
    }
}
