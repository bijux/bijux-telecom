//! Receiver entrypoint helpers.
#![allow(missing_docs)]

use crate::api::{Receiver, ReceiverPipelineConfig, ReceiverRuntime};
#[cfg(feature = "nav")]
use crate::pipeline::navigation::Navigation;
#[cfg(feature = "nav")]
use crate::pipeline::navigation_filter::NavigationFilter;
#[cfg(feature = "nav")]
use bijux_gnss_core::api::{NavEpoch, ObsEpoch};
#[cfg(feature = "nav")]
use bijux_gnss_nav::api::{GpsBroadcastNavigationData, GpsEphemeris, KlobucharCoefficients};

impl Receiver {
    /// Create a new receiver with the provided configuration.
    pub fn new(config: ReceiverPipelineConfig, runtime: ReceiverRuntime) -> Self {
        Self { config, runtime }
    }

    /// Borrow the receiver configuration.
    pub fn config(&self) -> &ReceiverPipelineConfig {
        &self.config
    }

    /// Borrow the receiver runtime options.
    pub fn runtime(&self) -> &ReceiverRuntime {
        &self.runtime
    }

    #[cfg(feature = "nav")]
    pub fn solve_observation_epochs(
        &self,
        observations: &[ObsEpoch],
        ephemerides: &[GpsEphemeris],
    ) -> Vec<NavEpoch> {
        self.solve_observation_epochs_with_runner(observations, |navigation, observation| {
            navigation.solve_epoch(observation, ephemerides)
        })
    }

    #[cfg(feature = "nav")]
    pub fn solve_observation_epochs_with_broadcast_ionosphere(
        &self,
        observations: &[ObsEpoch],
        ephemerides: &[GpsEphemeris],
        klobuchar: Option<&KlobucharCoefficients>,
    ) -> Vec<NavEpoch> {
        self.solve_observation_epochs_with_runner(observations, |navigation, observation| {
            navigation.solve_epoch_with_broadcast_ionosphere(observation, ephemerides, klobuchar)
        })
    }

    #[cfg(feature = "nav")]
    pub fn solve_observation_epochs_with_gps_broadcast_navigation(
        &self,
        observations: &[ObsEpoch],
        navigation_data: &GpsBroadcastNavigationData,
    ) -> Vec<NavEpoch> {
        self.solve_observation_epochs_with_runner(observations, |navigation, observation| {
            navigation.solve_epoch_with_gps_broadcast_navigation(observation, navigation_data)
        })
    }

    #[cfg(feature = "nav")]
    pub fn solve_observation_epochs_with_gps_broadcast_navigation_filter(
        &self,
        observations: &[ObsEpoch],
        navigation_data: &GpsBroadcastNavigationData,
    ) -> Vec<NavEpoch> {
        let mut navigation_filter = NavigationFilter::from_pipeline_config(&self.config);
        observations
            .iter()
            .filter_map(|observation| {
                navigation_filter.solve_epoch(
                    observation,
                    &navigation_data.ephemerides,
                    navigation_data.klobuchar.as_ref(),
                )
            })
            .collect()
    }

    #[cfg(feature = "nav")]
    fn solve_observation_epochs_with_runner<F>(
        &self,
        observations: &[ObsEpoch],
        mut solve_epoch: F,
    ) -> Vec<NavEpoch>
    where
        F: FnMut(&mut Navigation, &ObsEpoch) -> Option<NavEpoch>,
    {
        let mut navigation = Navigation::new(self.config.clone(), self.runtime.clone());
        observations
            .iter()
            .filter_map(|observation| solve_epoch(&mut navigation, observation))
            .collect()
    }
}
