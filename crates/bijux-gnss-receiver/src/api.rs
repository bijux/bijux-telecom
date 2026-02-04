//! Public API for bijux-gnss-receiver.

/// Receiver configuration and schema.
pub use crate::runtime::receiver_config::{ReceiverConfig, ReceiverError, ReceiverProfile};

/// I/O helpers.
pub mod data {
    pub use crate::io::data::{FileSamples, MemorySamples, SampleSourceError};
    pub use bijux_gnss_signal::api::SignalSource;
}

/// Re-export core API for downstream crates.
pub use bijux_gnss_core::api as core;

/// Re-export signal API for downstream crates.
pub use bijux_gnss_signal::api as signal;

/// Re-export nav API for downstream crates.
#[cfg(feature = "nav")]
pub use bijux_gnss_nav::api as nav;

/// Runtime logging helpers.
pub use crate::runtime::logging;

/// Stage modules for acquisition, tracking, observations, and navigation.
pub use crate::stages::acquisition;
/// Navigation bridge helpers.
#[cfg(feature = "nav")]
pub use crate::stages::navigation;
/// Observation-building helpers.
pub use crate::stages::observations;
/// Tracking stage helpers.
pub use crate::stages::tracking;

/// RTK differencing and baseline helpers.
#[cfg(feature = "nav")]
pub use crate::rtk_internal as rtk;

/// Synthetic signal generation for tests and demos.
#[cfg(feature = "nav")]
pub use crate::sim_internal::synthetic as sim;

/// Validation report helpers.
#[cfg(feature = "nav")]
pub use crate::validation_report;

/// Artifacts produced by a receiver pipeline run.
#[derive(Debug, Default, Clone)]
pub struct RunArtifacts {
    /// Acquisition candidates captured during the run.
    pub acquisitions: Vec<bijux_gnss_core::api::AcqResult>,
    /// Tracking reports captured during the run.
    pub tracking: Vec<crate::stages::tracking::TrackingResult>,
    /// Observation epochs captured during the run.
    pub observations: Vec<bijux_gnss_core::api::ObsEpoch>,
    /// Navigation solution epochs captured during the run.
    pub navigation: Vec<bijux_gnss_core::api::NavEpoch>,
}

/// Receiver engine boundary trait.
pub trait ReceiverEngine {
    /// Run the receiver pipeline against a signal source.
    fn run(
        &mut self,
        input: &mut dyn bijux_gnss_signal::api::SignalSource<
            Error = crate::io::data::SampleSourceError,
        >,
    ) -> Result<RunArtifacts, ReceiverError>;
}

/// High-level receiver pipeline entrypoint.
pub struct Receiver {
    config: ReceiverConfig,
}

impl Receiver {
    /// Create a new receiver with the provided configuration.
    pub fn new(config: ReceiverConfig) -> Self {
        Self { config }
    }

    /// Run a full pipeline: acquisition -> tracking.
    ///
    /// This is a scaffold and will be fully implemented incrementally.
    pub fn run(
        &self,
        input: &mut dyn bijux_gnss_signal::api::SignalSource<
            Error = crate::io::data::SampleSourceError,
        >,
    ) -> Result<RunArtifacts, ReceiverError> {
        let samples_per_code = bijux_gnss_signal::api::samples_per_code(
            self.config.sampling_freq_hz,
            self.config.code_freq_basis_hz,
            self.config.code_length,
        );
        let frame = match input.next_frame(samples_per_code) {
            Ok(Some(frame)) => frame,
            Ok(None) => return Ok(RunArtifacts::default()),
            Err(err) => {
                crate::runtime::diagnostics::dump_on_error(
                    &format!("input error: {err}"),
                    None,
                    None,
                );
                return Err(ReceiverError::Input(bijux_gnss_core::api::InputError {
                    message: err.to_string(),
                }))
            }
        };

        let sats: Vec<bijux_gnss_core::api::SatId> = (1..=32)
            .map(|prn| bijux_gnss_core::api::SatId {
                constellation: bijux_gnss_core::api::Constellation::Gps,
                prn,
            })
            .collect();
        let acquisition = crate::stages::acquisition::Acquisition::new(self.config.clone());
        let acquisitions = acquisition.run_fft(&frame, &sats);

        let tracking = crate::stages::tracking::Tracking::new(self.config.clone());
        let tracking_results = tracking.track_from_acquisition(
            &frame,
            &acquisitions,
            bijux_gnss_core::api::SignalBand::L1,
        );
        let artifacts = RunArtifacts {
            acquisitions,
            tracking: tracking_results,
            observations: Vec::new(),
            navigation: Vec::new(),
        };
        crate::runtime::metrics::write_metrics_summary(&artifacts);
        Ok(artifacts)
    }

    /// Borrow the receiver configuration.
    pub fn config(&self) -> &ReceiverConfig {
        &self.config
    }
}

impl ReceiverEngine for Receiver {
    fn run(
        &mut self,
        input: &mut dyn bijux_gnss_signal::api::SignalSource<
            Error = crate::io::data::SampleSourceError,
        >,
    ) -> Result<RunArtifacts, ReceiverError> {
        Receiver::run(self, input)
    }
}
