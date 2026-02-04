//! GNSS receiver pipeline orchestration.
//! See docs/GLOSSARY.md for acronym definitions.
//!
//! Module map:
//! - `stages`: acquisition, tracking, observations, navigation
//! - `io`: sample sources and dataset readers
//! - `runtime`: logging and runtime configuration
//! - `rtk`: differencing and baseline solution helpers
//! - `sim`: synthetic signal generation for tests
//! - `Receiver`: high-level pipeline entrypoint
//! - `ReceiverConfig`: runtime configuration
//! - `ReceiverProfile`: config file schema
//! - `ReceiverError`: error taxonomy for receiver stages

#![deny(clippy::unwrap_used)]
#![deny(missing_docs)]

mod io;
#[path = "rtk/mod.rs"]
mod rtk_internal;
mod runtime;
#[path = "sim/mod.rs"]
mod sim_internal;
mod stages;

/// Public API surface for this crate.
pub mod api;

/// Re-export public API at the crate root.
pub use api::{
    acquisition, data, logging, navigation, observations, rtk, sim, tracking, ReceiverConfig,
    ReceiverEngine, ReceiverError, ReceiverProfile, RunArtifacts,
};

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
        input: &mut dyn bijux_gnss_signal::SignalSource<Error = data::SampleSourceError>,
    ) -> Result<RunArtifacts, ReceiverError> {
        let samples_per_code = bijux_gnss_signal::samples_per_code(
            self.config.sampling_freq_hz,
            self.config.code_freq_basis_hz,
            self.config.code_length,
        );
        let frame = match input.next_frame(samples_per_code) {
            Ok(Some(frame)) => frame,
            Ok(None) => return Ok(RunArtifacts::default()),
            Err(err) => {
                return Err(ReceiverError::Input(bijux_gnss_core::InputError {
                    message: err.to_string(),
                }))
            }
        };

        let sats: Vec<bijux_gnss_core::SatId> = (1..=32)
            .map(|prn| bijux_gnss_core::SatId {
                constellation: bijux_gnss_core::Constellation::Gps,
                prn,
            })
            .collect();
        let acquisition = acquisition::Acquisition::new(self.config.clone());
        let acquisitions = acquisition.run_fft(&frame, &sats);

        let tracking = tracking::Tracking::new(self.config.clone());
        let _tracking_results =
            tracking.track_from_acquisition(&frame, &acquisitions, bijux_gnss_core::SignalBand::L1);

        Ok(RunArtifacts {
            acquisitions,
            tracking: Vec::new(),
            observations: Vec::new(),
            navigation: Vec::new(),
        })
    }

    /// Borrow the receiver configuration.
    pub fn config(&self) -> &ReceiverConfig {
        &self.config
    }
}

impl ReceiverEngine for Receiver {
    fn run(
        &mut self,
        input: &mut dyn bijux_gnss_signal::SignalSource<Error = data::SampleSourceError>,
    ) -> Result<RunArtifacts, ReceiverError> {
        Receiver::run(self, input)
    }
}
