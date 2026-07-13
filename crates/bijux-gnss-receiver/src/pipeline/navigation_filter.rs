#![allow(missing_docs)]

use bijux_gnss_core::api::{NavSolutionEpoch, ObsEpoch};
use bijux_gnss_nav::api::{
    GpsEphemeris, KlobucharCoefficients, NavigationFilter as NavNavigationFilter,
    NavigationFilterConfig,
};

use crate::engine::receiver_config::ReceiverPipelineConfig;

pub use bijux_gnss_nav::api::NavigationFilterThresholds;

pub struct NavigationFilter {
    inner: NavNavigationFilter,
}

impl NavigationFilter {
    #[cfg(test)]
    pub fn new() -> Self {
        Self { inner: NavNavigationFilter::new() }
    }

    pub fn from_pipeline_config(config: &ReceiverPipelineConfig) -> Self {
        Self {
            inner: NavNavigationFilter::new_with_config(NavigationFilterConfig {
                tropo_enabled: config.tropo_enable,
                tropo_ztd_m: config.tropo_ztd_m,
                thresholds: NavigationFilterThresholds {
                    max_pdop: config.science_thresholds.max_pdop,
                    max_gdop: config.science_thresholds.max_gdop,
                    min_used_satellites: config.science_thresholds.min_used_satellites,
                },
            }),
        }
    }

    pub fn solve_epoch(
        &mut self,
        obs: &ObsEpoch,
        ephemerides: &[GpsEphemeris],
        klobuchar: Option<&KlobucharCoefficients>,
    ) -> Option<NavSolutionEpoch> {
        self.inner.solve_epoch(obs, ephemerides, klobuchar)
    }
}
