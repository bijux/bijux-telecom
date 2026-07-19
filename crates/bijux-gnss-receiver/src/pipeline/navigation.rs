#![allow(missing_docs)]

use bijux_gnss_core::api::{NavSolutionEpoch, ObsEpoch};
use bijux_gnss_nav::api::{
    GalileoBroadcastNavigationData, GpsBroadcastNavigationData, GpsEphemeris,
    KlobucharCoefficients, NavEngine, PositionBroadcastNavigation, PositionRuntime,
};

use crate::engine::receiver_config::ReceiverPipelineConfig;
use crate::engine::runtime::ReceiverRuntime;
use crate::pipeline::nav_config::position_runtime_config;

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
