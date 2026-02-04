//! Receiver pipeline engine implementation.
#![allow(missing_docs)]

use crate::api::{Receiver, ReceiverEngine, RunArtifacts};
use crate::runtime::metrics::write_metrics_summary;
use bijux_gnss_core::api::{Constellation, InputError, SatId, SignalBand};

impl Receiver {
    /// Run a full pipeline: acquisition -> tracking.
    ///
    /// This is a scaffold and will be fully implemented incrementally.
    pub fn run(
        &self,
        input: &mut dyn bijux_gnss_signal::api::SignalSource<
            Error = crate::io::data::SampleSourceError,
        >,
    ) -> Result<RunArtifacts, crate::runtime::receiver_config::ReceiverError> {
        let samples_per_code = bijux_gnss_signal::api::samples_per_code(
            self.config().sampling_freq_hz,
            self.config().code_freq_basis_hz,
            self.config().code_length,
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
                return Err(crate::runtime::receiver_config::ReceiverError::Input(InputError {
                    message: err.to_string(),
                }));
            }
        };

        let sats: Vec<SatId> = (1..=32)
            .map(|prn| SatId {
                constellation: Constellation::Gps,
                prn,
            })
            .collect();
        let acquisition =
            crate::stages::acquisition::Acquisition::new(self.config().clone());
        let acquisitions = acquisition.run_fft(&frame, &sats);

        let tracking = crate::stages::tracking::Tracking::new(self.config().clone());
        let tracking_results =
            tracking.track_from_acquisition(&frame, &acquisitions, SignalBand::L1);
        let artifacts = RunArtifacts {
            acquisitions,
            tracking: tracking_results,
            observations: Vec::new(),
            navigation: Vec::new(),
        };
        write_metrics_summary(&artifacts);
        Ok(artifacts)
    }
}

impl ReceiverEngine for Receiver {
    fn run(
        &mut self,
        input: &mut dyn bijux_gnss_signal::api::SignalSource<
            Error = crate::io::data::SampleSourceError,
        >,
    ) -> Result<RunArtifacts, crate::runtime::receiver_config::ReceiverError> {
        Receiver::run(self, input)
    }
}
