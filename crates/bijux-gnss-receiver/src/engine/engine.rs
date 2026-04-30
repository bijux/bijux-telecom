//! Receiver pipeline engine implementation.
#![allow(missing_docs)]

use crate::api::{Receiver, ReceiverEngine, RunArtifacts};
use crate::engine::metrics::write_metrics_summary;
use crate::engine::runtime::{Metric, TraceRecord};
use crate::pipeline::observations::observations_from_tracking_results;
use bijux_gnss_core::api::{Constellation, InputError, SatId, SignalBand};
use std::time::Instant;

impl Receiver {
    /// Run a full pipeline: acquisition -> tracking.
    ///
    /// This is a scaffold and will be fully implemented incrementally.
    pub fn run(
        &self,
        input: &mut dyn bijux_gnss_signal::api::SignalSource<
            Error = crate::io::data::SampleSourceError,
        >,
    ) -> Result<RunArtifacts, crate::engine::receiver_config::ReceiverError> {
        let runtime = self.runtime().clone();
        runtime.trace.record(TraceRecord {
            name: "pipeline_start",
            fields: vec![("entry", "run".to_string())],
        });
        let pipeline_start = Instant::now();

        let samples_per_code = bijux_gnss_signal::api::samples_per_code(
            self.config().sampling_freq_hz,
            self.config().code_freq_basis_hz,
            self.config().code_length,
        );
        let frame = match input.next_frame(samples_per_code) {
            Ok(Some(frame)) => frame,
            Ok(None) => return Ok(RunArtifacts::default()),
            Err(err) => {
                crate::engine::diagnostics::dump_on_error(
                    self.runtime(),
                    &format!("input error: {err}"),
                    None,
                    None,
                );
                return Err(crate::engine::receiver_config::ReceiverError::Input(InputError {
                    message: err.to_string(),
                }));
            }
        };

        let sats: Vec<SatId> =
            (1..=32).map(|prn| SatId { constellation: Constellation::Gps, prn }).collect();

        runtime.trace.record(TraceRecord {
            name: "pipeline_stage",
            fields: vec![
                ("stage", "acquisition".to_string()),
                ("sat_count", sats.len().to_string()),
            ],
        });
        let acquisition_start = Instant::now();
        let acquisition = crate::pipeline::acquisition::Acquisition::new(
            self.config().clone(),
            self.runtime().clone(),
        );
        let acquisitions = acquisition.run_fft(&frame, &sats);
        let acquisition_stats = acquisition.stats_snapshot();
        let acquisition_ms = acquisition_start.elapsed().as_secs_f64() * 1000.0;
        runtime.metrics.metric(Metric { name: "stage_acquisition_ms", value: acquisition_ms });
        runtime.metrics.metric(Metric {
            name: "acquisition_candidate_count",
            value: acquisitions.len() as f64,
        });
        runtime.metrics.metric(Metric {
            name: "acquisition_sat_count",
            value: acquisition_stats.sat_count as f64,
        });
        runtime.metrics.metric(Metric {
            name: "acquisition_doppler_bins",
            value: acquisition_stats.doppler_bins as f64,
        });
        runtime.metrics.metric(Metric {
            name: "acquisition_code_search_bins",
            value: acquisition_stats.code_search_bins as f64,
        });
        runtime.metrics.metric(Metric {
            name: "acquisition_cache_hits",
            value: acquisition_stats.cache_hits as f64,
        });
        runtime.metrics.metric(Metric {
            name: "acquisition_cache_misses",
            value: acquisition_stats.cache_misses as f64,
        });
        runtime.metrics.metric(Metric {
            name: "acquisition_accepted_count",
            value: acquisition_stats.accepted_count as f64,
        });
        runtime.metrics.metric(Metric {
            name: "acquisition_ambiguous_count",
            value: acquisition_stats.ambiguous_count as f64,
        });
        runtime.metrics.metric(Metric {
            name: "acquisition_rejected_count",
            value: acquisition_stats.rejected_count as f64,
        });
        runtime.metrics.metric(Metric {
            name: "acquisition_deferred_count",
            value: acquisition_stats.deferred_count as f64,
        });
        runtime.trace.record(TraceRecord {
            name: "pipeline_stage_complete",
            fields: vec![
                ("stage", "acquisition".to_string()),
                ("accepted_candidates", acquisitions.len().to_string()),
            ],
        });

        runtime.trace.record(TraceRecord {
            name: "pipeline_stage",
            fields: vec![("stage", "tracking".to_string())],
        });
        let tracking_start = Instant::now();

        let tracking =
            crate::pipeline::tracking::Tracking::new(self.config().clone(), self.runtime().clone());
        let tracking_results =
            tracking.track_from_acquisition(&frame, &acquisitions, SignalBand::L1);
        let tracking_ms = tracking_start.elapsed().as_secs_f64() * 1000.0;
        let track_channel_count: f64 =
            tracking_results.iter().filter(|result| result.epochs.len() > 0).count() as f64;
        runtime.metrics.metric(Metric { name: "stage_tracking_ms", value: tracking_ms });
        runtime
            .metrics
            .metric(Metric { name: "tracking_channel_count", value: track_channel_count });
        runtime.trace.record(TraceRecord {
            name: "pipeline_stage_complete",
            fields: vec![
                ("stage", "tracking".to_string()),
                ("channels", track_channel_count.to_string()),
            ],
        });

        runtime.trace.record(TraceRecord {
            name: "pipeline_stage",
            fields: vec![("stage", "observations".to_string())],
        });
        let observation_start = Instant::now();
        let observation_report = observations_from_tracking_results(
            self.config(),
            &tracking_results,
            self.config().hatch_window,
        );
        let observation_ms = observation_start.elapsed().as_secs_f64() * 1000.0;
        let observation_epoch_count = observation_report.output.len() as f64;
        runtime.metrics.metric(Metric { name: "stage_observation_ms", value: observation_ms });
        runtime
            .metrics
            .metric(Metric { name: "observation_epoch_count", value: observation_epoch_count });
        runtime.trace.record(TraceRecord {
            name: "pipeline_stage_complete",
            fields: vec![
                ("stage", "observations".to_string()),
                ("epochs", observation_report.output.len().to_string()),
            ],
        });
        runtime.trace.record(TraceRecord {
            name: "pipeline_stage",
            fields: vec![
                ("stage", "navigation".to_string()),
                ("status", "not_executed".to_string()),
            ],
        });
        runtime.trace.record(TraceRecord {
            name: "pipeline_stage_complete",
            fields: vec![
                ("stage", "navigation".to_string()),
                ("status", "not_executed".to_string()),
            ],
        });
        runtime.trace.record(TraceRecord {
            name: "pipeline_stage",
            fields: vec![
                ("stage", "artifacts".to_string()),
                ("status", "write_metrics_summary".to_string()),
            ],
        });

        let artifacts = RunArtifacts {
            acquisitions,
            tracking: tracking_results,
            observations: observation_report.output,
            navigation: Vec::new(),
        };
        runtime.trace.record(TraceRecord {
            name: "pipeline_stage_complete",
            fields: vec![
                ("stage", "artifacts".to_string()),
                ("status", "write_metrics_summary".to_string()),
            ],
        });
        runtime.metrics.metric(Metric {
            name: "pipeline_run_ms",
            value: pipeline_start.elapsed().as_secs_f64() * 1000.0,
        });
        runtime.trace.record(TraceRecord {
            name: "pipeline_complete",
            fields: vec![("stages", "acquisition,tracking,observations".to_string())],
        });
        write_metrics_summary(self.runtime(), &artifacts, acquisition_stats);
        Ok(artifacts)
    }
}

impl ReceiverEngine for Receiver {
    fn run(
        &mut self,
        input: &mut dyn bijux_gnss_signal::api::SignalSource<
            Error = crate::io::data::SampleSourceError,
        >,
    ) -> Result<RunArtifacts, crate::engine::receiver_config::ReceiverError> {
        Receiver::run(self, input)
    }
}
