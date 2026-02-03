//! bijux-gnss receiver core library.
//!
//! This library defines a modular GNSS receiver pipeline. Algorithms are
//! implemented independently (not line-by-line translations) to preserve
//! Apache-2.0 licensing.

#![deny(clippy::unwrap_used)]

pub mod acquisition;
pub mod ambiguity;
pub mod ca_code;
pub mod config;
pub mod data;
pub mod differencing;
pub mod logging;
pub mod navigation;
pub mod observations;
pub mod rtk;
pub mod samples;
pub mod signal;
pub mod synthetic;
pub mod trace_dump;
pub mod tracking;
pub mod types;

pub use crate::config::ReceiverProfile;
pub use crate::types::{ReceiverConfig, ReceiverError};

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
        let samples_per_code = signal::samples_per_code(
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
        let _tracking_results = tracking.track_from_acquisition(&frame, &acquisitions, 0.5);

        Ok(())
    }

    pub fn config(&self) -> &ReceiverConfig {
        &self.config
    }
}
