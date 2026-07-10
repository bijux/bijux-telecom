//! Receiver pipeline engine implementation.
#![allow(missing_docs)]

use crate::api::{Receiver, ReceiverEngine, RunArtifacts};
use crate::engine::metrics::write_metrics_summary;
use crate::engine::runtime::{Metric, TraceRecord};
use crate::pipeline::observations::{
    observation_decisions_from_epochs, observations_from_tracking_results_with_gps_anchor,
};
use bijux_gnss_core::api::{
    signal_registry, Constellation, InputError, SatId, SignalBand, SignalCode, SignalSupportRow,
    SupportMatrix, SupportStatus,
};
use bijux_gnss_signal::api::remove_dc_offset_in_place;
use std::time::Instant;

const STREAMING_TRACKING_CODE_PERIODS: usize = 100;

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
        let mut frame = match input.next_frame(samples_per_code) {
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
        if self.config().remove_dc_offset {
            let removed = remove_dc_offset_in_place(&mut frame.iq);
            runtime.metrics.metric(Metric {
                name: "front_end_dc_imbalance_before_removal",
                value: removed.dc_imbalance,
            });
            runtime.metrics.metric(Metric {
                name: "front_end_i_power_before_removal",
                value: removed.i_power,
            });
            runtime.metrics.metric(Metric {
                name: "front_end_q_power_before_removal",
                value: removed.q_power,
            });
            runtime.metrics.metric(Metric {
                name: "front_end_iq_power_ratio_before_removal",
                value: removed.iq_power_ratio,
            });
            runtime.metrics.metric(Metric {
                name: "front_end_power_imbalance_warning_before_removal",
                value: if removed.power_imbalance_warning { 1.0 } else { 0.0 },
            });
            if let Some(quadrature_error_deg) = removed.quadrature_error_deg {
                runtime.metrics.metric(Metric {
                    name: "front_end_quadrature_error_deg_before_removal",
                    value: quadrature_error_deg,
                });
            }
            runtime.metrics.metric(Metric {
                name: "front_end_quadrature_error_warning_before_removal",
                value: if removed.quadrature_error_warning { 1.0 } else { 0.0 },
            });
            runtime.trace.record(TraceRecord {
                name: "front_end_dc_removal",
                fields: vec![
                    ("sample_count", removed.sample_count.to_string()),
                    ("i_mean", format!("{:.9}", removed.i_mean)),
                    ("q_mean", format!("{:.9}", removed.q_mean)),
                    ("i_power", format!("{:.9}", removed.i_power)),
                    ("q_power", format!("{:.9}", removed.q_power)),
                    ("iq_power_ratio", format!("{:.9}", removed.iq_power_ratio)),
                    ("power_imbalance_warning", removed.power_imbalance_warning.to_string()),
                    (
                        "quadrature_error_deg",
                        removed
                            .quadrature_error_deg
                            .map(|value| format!("{value:.9}"))
                            .unwrap_or_else(|| "n/a".to_string()),
                    ),
                    ("quadrature_error_warning", removed.quadrature_error_warning.to_string()),
                    ("rms", format!("{:.9}", removed.rms)),
                    ("dc_imbalance", format!("{:.9}", removed.dc_imbalance)),
                ],
            });
        }

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
        let acquisition_run = acquisition.run_fft_topn_with_explain(
            &frame,
            &sats,
            4,
            self.config().acquisition_integration_ms,
            self.config().acquisition_noncoherent,
        );
        let acquisition_candidates = acquisition_run.results;
        let acquisitions: Vec<_> = acquisition_candidates
            .iter()
            .filter_map(|candidates| candidates.first().cloned())
            .collect();
        let acquisition_stats = acquisition.stats_snapshot();
        let acquisition_explain = acquisition_run.explains;
        let acquisition_ms = acquisition_start.elapsed().as_secs_f64() * 1000.0;
        runtime.metrics.metric(Metric { name: "stage_acquisition_ms", value: acquisition_ms });
        runtime.metrics.metric(Metric {
            name: "acquisition_candidate_count",
            value: acquisition_candidates.len() as f64,
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
            name: "acquisition_cache_miss_cold_start",
            value: acquisition_stats.cache_miss_cold_start as f64,
        });
        runtime.metrics.metric(Metric {
            name: "acquisition_cache_miss_incompatible",
            value: acquisition_stats.cache_miss_incompatible as f64,
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
                ("explained_sats", acquisition_explain.len().to_string()),
            ],
        });

        runtime.trace.record(TraceRecord {
            name: "pipeline_stage",
            fields: vec![("stage", "tracking".to_string())],
        });
        let tracking_start = Instant::now();

        let tracking =
            crate::pipeline::tracking::Tracking::new(self.config().clone(), self.runtime().clone());
        let has_trackable_channels = acquisitions
            .iter()
            .filter(|acq| {
                matches!(
                    acq.hypothesis,
                    bijux_gnss_core::api::AcqHypothesis::Accepted
                        | bijux_gnss_core::api::AcqHypothesis::Ambiguous
                )
            })
            .take(self.config().channels.max(1))
            .next()
            .is_some();
        let mut tracking_session = tracking.begin_tracking_session(&acquisitions);
        tracking.track_session_frame(&mut tracking_session, &frame);
        if has_trackable_channels {
            let tracking_frame_len = streaming_tracking_frame_len(samples_per_code);
            loop {
                let next_frame = match input.next_frame(tracking_frame_len) {
                    Ok(frame) => frame,
                    Err(err) => {
                        crate::engine::diagnostics::dump_on_error(
                            self.runtime(),
                            &format!("input error: {err}"),
                            None,
                            None,
                        );
                        return Err(crate::engine::receiver_config::ReceiverError::Input(
                            InputError { message: err.to_string() },
                        ));
                    }
                };
                let Some(mut tracking_frame) = next_frame else {
                    break;
                };
                if self.config().remove_dc_offset {
                    remove_dc_offset_in_place(&mut tracking_frame.iq);
                }
                tracking.track_session_frame(&mut tracking_session, &tracking_frame);
            }
        }
        let tracking_artifacts = tracking.finish_tracking_session(tracking_session);
        let track_transitions = tracking_artifacts.track_transitions.clone();
        let channel_state_reports = tracking_artifacts.channel_state_reports.clone();
        let tracking_results = tracking_artifacts.tracking;
        let tracking_ms = tracking_start.elapsed().as_secs_f64() * 1000.0;
        let track_channel_count: f64 =
            tracking_results.iter().filter(|result| !result.epochs.is_empty()).count() as f64;
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
        let observation_report = observations_from_tracking_results_with_gps_anchor(
            self.config(),
            self.runtime().config.capture_start_gps_time,
            &tracking_results,
            self.config().hatch_window,
        );
        let observation_decisions = observation_decisions_from_epochs(&observation_report.output);
        let observation_ms = observation_start.elapsed().as_secs_f64() * 1000.0;
        let observation_decisions_count = observation_decisions.len();
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
                ("decision_artifacts", observation_decisions_count.to_string()),
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
            processed_input_samples: tracking_artifacts.processed_input_samples,
            processed_input_epochs: tracking_artifacts.processed_input_epochs,
            acquisitions,
            acquisition_explain,
            track_transitions,
            channel_state_reports,
            tracking: tracking_results,
            observations: observation_report.output,
            observation_decisions,
            support_matrix: Some(build_support_matrix()),
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

fn streaming_tracking_frame_len(samples_per_code: usize) -> usize {
    samples_per_code.saturating_mul(STREAMING_TRACKING_CODE_PERIODS.max(1))
}

fn build_support_matrix() -> SupportMatrix {
    let constellations =
        [Constellation::Gps, Constellation::Galileo, Constellation::Glonass, Constellation::Beidou];
    let bands = [
        SignalBand::L1,
        SignalBand::L2,
        SignalBand::L5,
        SignalBand::E1,
        SignalBand::E5,
        SignalBand::B1,
        SignalBand::B2,
    ];
    let codes = [
        SignalCode::Ca,
        SignalCode::Py,
        SignalCode::E1B,
        SignalCode::E1C,
        SignalCode::E5a,
        SignalCode::E5b,
        SignalCode::Unknown,
    ];

    let mut rows = Vec::new();
    for constellation in constellations {
        for band in bands {
            for code in codes {
                let status = signal_support_status(constellation, band, code);
                if matches!(status, SupportStatus::Unsupported)
                    && signal_registry(constellation, band, code).is_none()
                {
                    continue;
                }
                rows.push(SignalSupportRow {
                    constellation,
                    band,
                    code,
                    status,
                    reason: support_reason(constellation, band, code),
                });
            }
        }
    }
    rows.sort_by_key(|row| {
        (format!("{:?}", row.constellation), format!("{:?}", row.band), format!("{:?}", row.code))
    });
    SupportMatrix { schema_version: 1, rows }
}

fn signal_support_status(
    constellation: Constellation,
    band: SignalBand,
    code: SignalCode,
) -> SupportStatus {
    if constellation == Constellation::Gps && band == SignalBand::L1 && code == SignalCode::Ca {
        return SupportStatus::Supported;
    }
    if signal_registry(constellation, band, code).is_some() {
        return SupportStatus::Planned;
    }
    SupportStatus::Unsupported
}

fn support_reason(constellation: Constellation, band: SignalBand, code: SignalCode) -> String {
    if constellation == Constellation::Gps && band == SignalBand::L1 && code == SignalCode::Ca {
        return "receiver pipeline supports this signal path".to_string();
    }
    if signal_registry(constellation, band, code).is_some() {
        return "signal model registered; receiver pipeline support not yet complete".to_string();
    }
    "unsupported constellation-band-code combination".to_string()
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
