#![allow(missing_docs)]

use bijux_gnss_core::api::{NavSolutionEpoch, ObsEpoch};
use bijux_gnss_nav::api::{
    GpsEphemeris, KlobucharCoefficients, NavigationFilter as NavNavigationFilter,
};

use crate::engine::receiver_config::ReceiverPipelineConfig;
use crate::pipeline::nav_config::navigation_filter_config;

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
        Self { inner: NavNavigationFilter::new_with_config(navigation_filter_config(config)) }
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
