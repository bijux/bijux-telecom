#![allow(missing_docs)]

use bijux_gnss_core::api::{Constellation, NavSolutionEpoch, ObsEpoch};
use bijux_gnss_nav::api::{
    GalileoBroadcastNavigationData, GpsBroadcastNavigationData, GpsEphemeris,
    KlobucharCoefficients, NavEngine, PositionBroadcastNavigation, PositionConstellationPolicy,
    PositionFilterMotionClass, PositionRuntime, PositionRuntimeConfig,
    PositionRuntimeThresholds, PositionRuntimeWeightingConfig, PositionWeightingModel,
};

use crate::engine::receiver_config::{
    ConstellationSelectionPolicy, NavigationMotionClass, NavigationWeightingMode,
    ReceiverPipelineConfig,
};
use crate::engine::runtime::ReceiverRuntime;

pub use bijux_gnss_nav::api::{EkfState, NavigationEngine};

pub struct Navigation {
    inner: PositionRuntime,
    runtime: ReceiverRuntime,
}

impl Navigation {
    pub fn new(config: ReceiverPipelineConfig, runtime: ReceiverRuntime) -> Self {
        Self { inner: PositionRuntime::new(position_runtime_config(&config)), runtime }
    }

    pub fn solve_epoch(
        &mut self,
        obs: &ObsEpoch,
        eph: &[GpsEphemeris],
    ) -> Option<NavSolutionEpoch> {
        let solution = self.inner.solve_epoch(obs, eph);
        self.flush_diagnostic_events();
        solution
    }

    pub fn solve_epoch_with_broadcast_ionosphere(
        &mut self,
        obs: &ObsEpoch,
        eph: &[GpsEphemeris],
        klobuchar: Option<&KlobucharCoefficients>,
    ) -> Option<NavSolutionEpoch> {
        let solution = self.inner.solve_epoch_with_broadcast_ionosphere(obs, eph, klobuchar);
        self.flush_diagnostic_events();
        solution
    }

    pub fn solve_epoch_with_gps_broadcast_navigation(
        &mut self,
        obs: &ObsEpoch,
        navigation: &GpsBroadcastNavigationData,
    ) -> Option<NavSolutionEpoch> {
        let solution = self.inner.solve_epoch_with_gps_broadcast_navigation(obs, navigation);
        self.flush_diagnostic_events();
        solution
    }

    pub fn solve_epoch_with_galileo_broadcast_navigations(
        &mut self,
        obs: &ObsEpoch,
        navigations: &[GalileoBroadcastNavigationData],
    ) -> Option<NavSolutionEpoch> {
        let solution = self.inner.solve_epoch_with_galileo_broadcast_navigations(obs, navigations);
        self.flush_diagnostic_events();
        solution
    }

    pub fn solve_epoch_with_navigation_data(
        &mut self,
        obs: &ObsEpoch,
        navigation: &[PositionBroadcastNavigation],
    ) -> Option<NavSolutionEpoch> {
        let solution = self.inner.solve_epoch_with_navigation_data(obs, navigation);
        self.flush_diagnostic_events();
        solution
    }

    pub fn solve_epoch_with_navigation_data_and_broadcast_ionosphere(
        &mut self,
        obs: &ObsEpoch,
        navigation: &[PositionBroadcastNavigation],
        klobuchar: Option<&KlobucharCoefficients>,
    ) -> Option<NavSolutionEpoch> {
        let solution = self
            .inner
            .solve_epoch_with_navigation_data_and_broadcast_ionosphere(obs, navigation, klobuchar);
        self.flush_diagnostic_events();
        solution
    }

    fn flush_diagnostic_events(&mut self) {
        for event in self.inner.drain_diagnostic_events() {
            self.runtime.logger.event(&event);
        }
    }
}

impl NavEngine for Navigation {
    fn update(
        &mut self,
        obs: &bijux_gnss_core::api::ObsEpochV1,
    ) -> bijux_gnss_core::api::NavSolutionEpochV1 {
        let solution = self.inner.update(obs);
        self.flush_diagnostic_events();
        solution
    }
}

pub(crate) fn supports_positioning_signal(
    constellation: Constellation,
    band: bijux_gnss_core::api::SignalBand,
    code: bijux_gnss_core::api::SignalCode,
) -> bool {
    bijux_gnss_nav::api::supports_positioning_signal(constellation, band, code)
}

fn position_runtime_config(config: &ReceiverPipelineConfig) -> PositionRuntimeConfig {
    PositionRuntimeConfig {
        robust_solver: config.robust_solver,
        huber_k: config.huber_k,
        raim: config.raim,
        position_solution_smoothing: config.position_solution_smoothing,
        position_solution_motion_class: match config.position_solution_motion_class {
            NavigationMotionClass::Static => PositionFilterMotionClass::Static,
            NavigationMotionClass::Pedestrian => PositionFilterMotionClass::Pedestrian,
            NavigationMotionClass::Vehicle => PositionFilterMotionClass::Vehicle,
            NavigationMotionClass::Airborne => PositionFilterMotionClass::Airborne,
        },
        weighting: PositionRuntimeWeightingConfig {
            enabled: config.weighting.enabled,
            mode: match config.weighting.mode {
                NavigationWeightingMode::Elevation => PositionWeightingModel::Elevation,
                NavigationWeightingMode::Cn0 => PositionWeightingModel::Cn0,
                NavigationWeightingMode::ElevationCn0 => PositionWeightingModel::ElevationCn0,
            },
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
        constellation_policy: match config.constellation_policy {
            ConstellationSelectionPolicy::GpsOnly => PositionConstellationPolicy::GpsOnly,
            ConstellationSelectionPolicy::GalileoOnly => PositionConstellationPolicy::GalileoOnly,
            ConstellationSelectionPolicy::GlonassOnly => PositionConstellationPolicy::GlonassOnly,
            ConstellationSelectionPolicy::BeidouOnly => PositionConstellationPolicy::BeidouOnly,
            ConstellationSelectionPolicy::Mixed => PositionConstellationPolicy::Mixed,
        },
    }
}
