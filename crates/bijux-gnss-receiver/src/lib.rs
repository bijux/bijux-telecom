//! GNSS receiver pipeline orchestration.
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

pub mod rtk;
pub mod sim;

mod io;
mod runtime;
mod stages;

pub use crate::io::data;
pub use crate::runtime::logging;
pub use crate::runtime::receiver_config::{ReceiverConfig, ReceiverError, ReceiverProfile};
pub use crate::stages::{acquisition, navigation, observations, tracking};

/// High-level receiver pipeline entrypoint.
pub struct Receiver {
    config: ReceiverConfig,
}

impl Receiver {
    pub fn new(config: ReceiverConfig) -> Self {
        Self { config }
    }

    /// Run a full pipeline: acquisition -> tracking.
    ///
    /// This is a scaffold and will be fully implemented incrementally.
    pub fn run(&self, _input: &mut dyn data::SampleSource) -> Result<(), ReceiverError> {
        let samples_per_code = bijux_gnss_signal::samples_per_code(
            self.config.sampling_freq_hz,
            self.config.code_freq_basis_hz,
            self.config.code_length,
        );
        let frame = match _input.next_frame(samples_per_code) {
            Ok(Some(frame)) => frame,
            Ok(None) => return Ok(()),
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

        Ok(())
    }

    pub fn config(&self) -> &ReceiverConfig {
        &self.config
    }
}
