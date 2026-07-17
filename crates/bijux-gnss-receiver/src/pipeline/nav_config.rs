#![allow(missing_docs)]

use bijux_gnss_nav::api::{
    NavigationFilterConfig, NavigationFilterThresholds, PositionConstellationPolicy,
    PositionFilterMotionClass, PositionRuntimeConfig, PositionRuntimeThresholds,
    PositionRuntimeWeightingConfig, PositionWeightingModel,
};

use crate::engine::receiver_config::{
    ConstellationSelectionPolicy, NavigationMotionClass, NavigationWeightingMode,
    ReceiverPipelineConfig,
};

pub(crate) fn navigation_filter_config(config: &ReceiverPipelineConfig) -> NavigationFilterConfig {
    NavigationFilterConfig {
        tropo_enabled: config.tropo_enable,
        tropo_ztd_m: config.tropo_ztd_m,
        thresholds: NavigationFilterThresholds {
            max_pdop: config.science_thresholds.max_pdop,
            max_gdop: config.science_thresholds.max_gdop,
            min_used_satellites: config.science_thresholds.min_used_satellites,
        },
    }
}

pub(crate) fn position_runtime_config(config: &ReceiverPipelineConfig) -> PositionRuntimeConfig {
    PositionRuntimeConfig {
        robust_solver: config.robust_solver,
        huber_k: config.huber_k,
        raim: config.raim,
        position_solution_smoothing: config.position_solution_smoothing,
        position_solution_motion_class: position_filter_motion_class(
            config.position_solution_motion_class,
        ),
        weighting: PositionRuntimeWeightingConfig {
            enabled: config.weighting.enabled,
            mode: position_weighting_model(config.weighting.mode),
            min_elev_deg: config.weighting.min_elev_deg,
            elev_exponent: config.weighting.elev_exponent,
            cn0_ref_dbhz: config.weighting.cn0_ref_dbhz,
            min_weight: config.weighting.min_weight,
            elev_mask_deg: config.weighting.elev_mask_deg,
            tracking_mode_scalar_weight: config.weighting.tracking_mode_scalar_weight,
            tracking_mode_vector_weight: config.weighting.tracking_mode_vector_weight,
        },
        tropo_enable: config.tropo_enable,
        science_thresholds: PositionRuntimeThresholds {
            min_mean_cn0_dbhz: config.science_thresholds.min_mean_cn0_dbhz,
            max_pdop: config.science_thresholds.max_pdop,
            max_gdop: config.science_thresholds.max_gdop,
            max_residual_rms_m: config.science_thresholds.max_residual_rms_m,
            min_used_satellites: config.science_thresholds.min_used_satellites,
            min_lock_ratio: config.science_thresholds.min_lock_ratio,
        },
        constellation_policy: position_constellation_policy(config.constellation_policy),
    }
}

fn position_filter_motion_class(motion_class: NavigationMotionClass) -> PositionFilterMotionClass {
    match motion_class {
        NavigationMotionClass::Static => PositionFilterMotionClass::Static,
        NavigationMotionClass::Pedestrian => PositionFilterMotionClass::Pedestrian,
        NavigationMotionClass::Vehicle => PositionFilterMotionClass::Vehicle,
        NavigationMotionClass::Airborne => PositionFilterMotionClass::Airborne,
    }
}

fn position_weighting_model(mode: NavigationWeightingMode) -> PositionWeightingModel {
    match mode {
        NavigationWeightingMode::Elevation => PositionWeightingModel::Elevation,
        NavigationWeightingMode::Cn0 => PositionWeightingModel::Cn0,
        NavigationWeightingMode::ElevationCn0 => PositionWeightingModel::ElevationCn0,
    }
}

fn position_constellation_policy(
    policy: ConstellationSelectionPolicy,
) -> PositionConstellationPolicy {
    match policy {
        ConstellationSelectionPolicy::GpsOnly => PositionConstellationPolicy::GpsOnly,
        ConstellationSelectionPolicy::GalileoOnly => PositionConstellationPolicy::GalileoOnly,
        ConstellationSelectionPolicy::GlonassOnly => PositionConstellationPolicy::GlonassOnly,
        ConstellationSelectionPolicy::BeidouOnly => PositionConstellationPolicy::BeidouOnly,
        ConstellationSelectionPolicy::Mixed => PositionConstellationPolicy::Mixed,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn navigation_filter_config_tracks_receiver_science_thresholds() {
        let mut config = ReceiverPipelineConfig {
            tropo_enable: false,
            tropo_ztd_m: 1.75,
            ..ReceiverPipelineConfig::default()
        };
        config.science_thresholds.max_pdop = 5.5;
        config.science_thresholds.max_gdop = 7.5;
        config.science_thresholds.min_used_satellites = 6;

        let filter = navigation_filter_config(&config);

        assert!(!filter.tropo_enabled);
        assert_eq!(filter.tropo_ztd_m, 1.75);
        assert_eq!(filter.thresholds.max_pdop, 5.5);
        assert_eq!(filter.thresholds.max_gdop, 7.5);
        assert_eq!(filter.thresholds.min_used_satellites, 6);
    }

    #[test]
    fn position_runtime_config_preserves_receiver_navigation_policy() {
        let mut config = ReceiverPipelineConfig {
            position_solution_motion_class: NavigationMotionClass::Pedestrian,
            ..ReceiverPipelineConfig::default()
        };
        config.weighting.mode = NavigationWeightingMode::ElevationCn0;
        config.constellation_policy = ConstellationSelectionPolicy::GalileoOnly;
        config.science_thresholds.min_lock_ratio = 0.85;

        let runtime = position_runtime_config(&config);

        assert_eq!(runtime.position_solution_motion_class, PositionFilterMotionClass::Pedestrian);
        assert_eq!(runtime.weighting.mode, PositionWeightingModel::ElevationCn0);
        assert_eq!(runtime.constellation_policy, PositionConstellationPolicy::GalileoOnly);
        assert_eq!(runtime.science_thresholds.min_lock_ratio, 0.85);
    }
}
