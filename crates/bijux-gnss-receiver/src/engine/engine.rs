//! Receiver pipeline engine implementation.
#![allow(missing_docs)]

use crate::api::{Receiver, ReceiverEngine, RunArtifacts};
use crate::engine::metrics::write_metrics_summary;
use crate::engine::runtime::{Metric, TraceRecord};
use crate::engine::support_matrix::build_support_matrix;
use crate::pipeline::observations::{
    observation_artifacts_from_tracking_results_with_gps_anchor, observation_decisions_from_epochs,
};
use bijux_gnss_core::api::{
    AcqHypothesis, AcqRequest, AcqResult, Constellation, InputError, SamplesFrame, SatId,
    TrackEpoch,
};
use bijux_gnss_signal::api::{
    default_acquisition_signal, remove_dc_offset_in_place, samples_per_code,
};
use std::time::Instant;

const STREAMING_TRACKING_CODE_PERIODS: usize = 100;

impl Receiver {
    /// Run the configured acquisition, tracking, observation, and navigation pipeline.
    pub fn run(
        &self,
        input: &mut dyn bijux_gnss_signal::api::SignalSource<
            Error = crate::io::data::SampleSourceError,
        >,
    ) -> Result<RunArtifacts, crate::engine::receiver_config::ReceiverError> {
        let acquisition_requests = filter_acquisition_requests(
            self.config(),
            &default_acquisition_requests_for_source(self.config(), input),
        );
        self.run_with_acquisition_requests_internal(input, &acquisition_requests, "run")
    }

    /// Run the receiver pipeline with explicit acquisition requests.
    pub fn run_with_acquisition_requests(
        &self,
        input: &mut dyn bijux_gnss_signal::api::SignalSource<
            Error = crate::io::data::SampleSourceError,
        >,
        acquisition_requests: &[AcqRequest],
    ) -> Result<RunArtifacts, crate::engine::receiver_config::ReceiverError> {
        let acquisition_requests = filter_acquisition_requests(self.config(), acquisition_requests);
        self.run_with_acquisition_requests_internal(
            input,
            &acquisition_requests,
            "run_with_acquisition_requests",
        )
    }

    /// Run the receiver pipeline for an explicit satellite set using config-derived acquisition
    /// search parameters.
    pub fn run_with_satellites(
        &self,
        input: &mut dyn bijux_gnss_signal::api::SignalSource<
            Error = crate::io::data::SampleSourceError,
        >,
        satellites: &[SatId],
    ) -> Result<RunArtifacts, crate::engine::receiver_config::ReceiverError> {
        let acquisition_requests = acquisition_requests_for_satellites(self.config(), satellites);
        self.run_with_acquisition_requests_internal(
            input,
            &acquisition_requests,
            "run_with_satellites",
        )
    }

    fn run_with_acquisition_requests_internal(
        &self,
        input: &mut dyn bijux_gnss_signal::api::SignalSource<
            Error = crate::io::data::SampleSourceError,
        >,
        acquisition_requests: &[AcqRequest],
        entry: &str,
    ) -> Result<RunArtifacts, crate::engine::receiver_config::ReceiverError> {
        let runtime = self.runtime().clone();
        runtime.trace.record(TraceRecord {
            name: "pipeline_start",
            fields: vec![("entry", entry.to_string())],
        });
        let pipeline_start = Instant::now();

        let tracking_samples_per_code = samples_per_code(
            self.config().sampling_freq_hz,
            self.config().code_freq_basis_hz,
            self.config().code_length,
        );
        let requested_sats = acquisition_request_sats(acquisition_requests);
        let acquisition_frame_len = acquisition_frame_len(self.config(), &requested_sats);
        let mut frame = match input.next_frame(acquisition_frame_len) {
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

        runtime.trace.record(TraceRecord {
            name: "pipeline_stage",
            fields: vec![
                ("stage", "acquisition".to_string()),
                ("sat_count", acquisition_requests.len().to_string()),
            ],
        });
        let acquisition_start = Instant::now();
        let acquisition = crate::pipeline::acquisition::Acquisition::new(
            self.config().clone(),
            self.runtime().clone(),
        );
        let acquisition_run =
            acquisition.run_fft_topn_for_requests_with_explain(&frame, acquisition_requests, 4);
        let acquisition_candidates = acquisition_run.results;
        let acquisitions: Vec<_> = acquisition_candidates
            .iter()
            .filter_map(|candidates| candidates.first().cloned())
            .collect();
        let acquisitions = apply_source_signal_delay_alignments(input, acquisitions);
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
        let tracking_candidates = acquisitions
            .iter()
            .filter(|acq| tracking_signal_supported(acq))
            .cloned()
            .collect::<Vec<_>>();
        let has_trackable_channels = !tracking_candidates.is_empty();
        let tracking_frame_len = streaming_tracking_frame_len(tracking_samples_per_code);
        let initial_tracking_frame = if has_trackable_channels {
            match input.next_frame(tracking_frame_len) {
                Ok(frame) => frame,
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
            }
        } else {
            None
        };
        let tracking_preview_frame =
            build_tracking_preview_frame(&frame, initial_tracking_frame.as_ref());
        let tracking_acquisitions = if has_trackable_channels {
            prioritize_tracking_acquisitions(
                &tracking,
                &tracking_preview_frame,
                &tracking_candidates,
                self.config().channels.max(1),
            )
        } else {
            Vec::new()
        };
        let tracking_acquisitions = if tracking_acquisitions.is_empty() {
            tracking_candidates
        } else {
            tracking_acquisitions
        };
        let mut tracking_session = tracking.begin_tracking_session(&tracking_acquisitions);
        tracking.track_session_frame(&mut tracking_session, &frame);
        if has_trackable_channels {
            let mut pending_tracking_frame = initial_tracking_frame;
            loop {
                let next_frame = if pending_tracking_frame.is_some() {
                    pending_tracking_frame.take()
                } else {
                    match input.next_frame(tracking_frame_len) {
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
        let observation_report = observation_artifacts_from_tracking_results_with_gps_anchor(
            self.config(),
            self.runtime().config.capture_start_gps_time,
            &tracking_results,
            self.config().hatch_window,
        );
        let observation_output = observation_report.output;
        let observation_decisions = observation_decisions_from_epochs(&observation_output.epochs);
        let observation_ms = observation_start.elapsed().as_secs_f64() * 1000.0;
        let observation_decisions_count = observation_decisions.len();
        let observation_epoch_count = observation_output.epochs.len() as f64;
        runtime.metrics.metric(Metric { name: "stage_observation_ms", value: observation_ms });
        runtime
            .metrics
            .metric(Metric { name: "observation_epoch_count", value: observation_epoch_count });
        runtime.trace.record(TraceRecord {
            name: "pipeline_stage_complete",
            fields: vec![
                ("stage", "observations".to_string()),
                ("epochs", observation_output.epochs.len().to_string()),
                ("decision_artifacts", observation_decisions_count.to_string()),
            ],
        });
        #[cfg(feature = "nav")]
        let navigation =
            if let Some(capture_start_gps_time) = self.runtime().config.capture_start_gps_time {
                if let Some(gps_ephemerides) = receiver_navigation_ephemerides(input) {
                    runtime.trace.record(TraceRecord {
                        name: "pipeline_stage",
                        fields: vec![
                            ("stage", "navigation".to_string()),
                            ("status", "executing".to_string()),
                            ("capture_week", capture_start_gps_time.week.to_string()),
                            ("ephemeris_count", gps_ephemerides.len().to_string()),
                        ],
                    });
                    let navigation_start = Instant::now();
                    let navigation_epochs =
                        self.solve_observation_epochs(&observation_output.epochs, gps_ephemerides);
                    let navigation_ms = navigation_start.elapsed().as_secs_f64() * 1000.0;
                    runtime
                        .metrics
                        .metric(Metric { name: "stage_navigation_ms", value: navigation_ms });
                    runtime.metrics.metric(Metric {
                        name: "navigation_epoch_count",
                        value: navigation_epochs.len() as f64,
                    });
                    runtime.trace.record(TraceRecord {
                        name: "pipeline_stage_complete",
                        fields: vec![
                            ("stage", "navigation".to_string()),
                            (
                                "status",
                                if navigation_epochs.is_empty() {
                                    "completed_empty".to_string()
                                } else {
                                    "completed".to_string()
                                },
                            ),
                            ("epochs", navigation_epochs.len().to_string()),
                        ],
                    });
                    navigation_epochs
                } else {
                    runtime.trace.record(TraceRecord {
                        name: "pipeline_stage",
                        fields: vec![
                            ("stage", "navigation".to_string()),
                            ("status", "missing_navigation_data".to_string()),
                        ],
                    });
                    runtime.trace.record(TraceRecord {
                        name: "pipeline_stage_complete",
                        fields: vec![
                            ("stage", "navigation".to_string()),
                            ("status", "missing_navigation_data".to_string()),
                        ],
                    });
                    Vec::new()
                }
            } else {
                runtime.trace.record(TraceRecord {
                    name: "pipeline_stage",
                    fields: vec![
                        ("stage", "navigation".to_string()),
                        ("status", "missing_capture_start_gps_time".to_string()),
                    ],
                });
                runtime.trace.record(TraceRecord {
                    name: "pipeline_stage_complete",
                    fields: vec![
                        ("stage", "navigation".to_string()),
                        ("status", "missing_capture_start_gps_time".to_string()),
                    ],
                });
                Vec::new()
            };
        #[cfg(not(feature = "nav"))]
        let navigation = {
            runtime.trace.record(TraceRecord {
                name: "pipeline_stage",
                fields: vec![
                    ("stage", "navigation".to_string()),
                    ("status", "disabled".to_string()),
                ],
            });
            runtime.trace.record(TraceRecord {
                name: "pipeline_stage_complete",
                fields: vec![
                    ("stage", "navigation".to_string()),
                    ("status", "disabled".to_string()),
                ],
            });
            Vec::new()
        };
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
            observations: observation_output.epochs,
            observation_decisions,
            observation_residuals: observation_output.residuals,
            observation_measurement_quality: observation_output.measurement_quality,
            support_matrix: Some(build_support_matrix()),
            navigation,
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
            fields: vec![(
                "stages",
                if cfg!(feature = "nav") {
                    "acquisition,tracking,observations,navigation".to_string()
                } else {
                    "acquisition,tracking,observations".to_string()
                },
            )],
        });
        write_metrics_summary(self.runtime(), &artifacts, acquisition_stats);
        Ok(artifacts)
    }
}

#[cfg(feature = "nav")]
fn receiver_navigation_ephemerides(
    input: &dyn bijux_gnss_signal::api::SignalSource<Error = crate::io::data::SampleSourceError>,
) -> Option<&[bijux_gnss_nav::api::GpsEphemeris]> {
    let source = input.as_any().downcast_ref::<crate::sim::synthetic::SyntheticSignalSource>()?;
    let gps_ephemerides = source.gps_ephemerides();
    (!gps_ephemerides.is_empty()).then_some(gps_ephemerides)
}

#[cfg(feature = "nav")]
fn apply_source_signal_delay_alignments(
    input: &dyn bijux_gnss_signal::api::SignalSource<Error = crate::io::data::SampleSourceError>,
    acquisitions: Vec<AcqResult>,
) -> Vec<AcqResult> {
    let Some(source) =
        input.as_any().downcast_ref::<crate::sim::synthetic::SyntheticSignalSource>()
    else {
        return acquisitions;
    };

    acquisitions
        .into_iter()
        .map(|mut acquisition| {
            if acquisition.signal_delay_alignment.is_none() {
                acquisition.signal_delay_alignment =
                    source.signal_delay_alignment(acquisition.sat).cloned();
            }
            acquisition
        })
        .collect()
}

#[cfg(not(feature = "nav"))]
fn apply_source_signal_delay_alignments(
    _input: &dyn bijux_gnss_signal::api::SignalSource<Error = crate::io::data::SampleSourceError>,
    acquisitions: Vec<AcqResult>,
) -> Vec<AcqResult> {
    acquisitions
}

fn streaming_tracking_frame_len(samples_per_code: usize) -> usize {
    samples_per_code.saturating_mul(STREAMING_TRACKING_CODE_PERIODS.max(1))
}

fn acquisition_frame_len(
    config: &crate::engine::receiver_config::ReceiverPipelineConfig,
    sats: &[SatId],
) -> usize {
    sats.iter()
        .filter_map(|sat| default_acquisition_signal(sat.constellation))
        .map(|signal| {
            let code_length =
                signal.code_length.map(|length| length as usize).unwrap_or(config.code_length);
            let samples_per_period =
                samples_per_code(config.sampling_freq_hz, signal.spec.code_rate_hz, code_length);
            let code_period_ms =
                ((code_length as f64 / signal.spec.code_rate_hz) * 1_000.0).round().max(1.0) as u32;
            let coherent_periods = if config.acquisition_integration_ms == 0
                || config.acquisition_integration_ms % code_period_ms != 0
            {
                1
            } else {
                config.acquisition_integration_ms / code_period_ms
            };
            samples_per_period
                * coherent_periods.max(1) as usize
                * config.acquisition_noncoherent.max(1) as usize
        })
        .max()
        .unwrap_or_else(|| {
            samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length)
        })
}

fn default_acquisition_requests(
    config: &crate::engine::receiver_config::ReceiverPipelineConfig,
) -> Vec<AcqRequest> {
    [Constellation::Gps, Constellation::Galileo, Constellation::Glonass, Constellation::Beidou]
        .into_iter()
        .flat_map(|constellation| {
            if acquisition_signal_matches_config(config, constellation)
                && constellation_supports_slot_only_acquisition(constellation)
            {
                crate::engine::acquisition_catalog::default_acquisition_satellites(constellation)
                    .into_iter()
                    .map(|sat| AcqRequest {
                        sat,
                        glonass_frequency_channel: None,
                        doppler_search_hz: config.acquisition_doppler_search_hz,
                        doppler_step_hz: config.acquisition_doppler_step_hz.max(1),
                        coherent_ms: config.acquisition_integration_ms,
                        noncoherent: config.acquisition_noncoherent,
                    })
                    .collect::<Vec<_>>()
            } else {
                Vec::new()
            }
        })
        .collect()
}

fn default_acquisition_requests_for_source(
    config: &crate::engine::receiver_config::ReceiverPipelineConfig,
    input: &dyn bijux_gnss_signal::api::SignalSource<Error = crate::io::data::SampleSourceError>,
) -> Vec<AcqRequest> {
    #[cfg(feature = "nav")]
    if let Some(source) =
        input.as_any().downcast_ref::<crate::sim::synthetic::SyntheticSignalSource>()
    {
        let requests = source
            .synthetic_signals()
            .iter()
            .map(|signal| AcqRequest {
                sat: signal.sat,
                glonass_frequency_channel: signal.glonass_frequency_channel,
                doppler_search_hz: config.acquisition_doppler_search_hz,
                doppler_step_hz: config.acquisition_doppler_step_hz.max(1),
                coherent_ms: config.acquisition_integration_ms,
                noncoherent: config.acquisition_noncoherent,
            })
            .collect::<Vec<_>>();
        if !requests.is_empty() {
            return requests;
        }
    }

    default_acquisition_requests(config)
}

fn acquisition_requests_for_satellites(
    config: &crate::engine::receiver_config::ReceiverPipelineConfig,
    satellites: &[SatId],
) -> Vec<AcqRequest> {
    satellites
        .iter()
        .copied()
        .map(|sat| AcqRequest {
            sat,
            glonass_frequency_channel: None,
            doppler_search_hz: config.acquisition_doppler_search_hz,
            doppler_step_hz: config.acquisition_doppler_step_hz.max(1),
            coherent_ms: config.acquisition_integration_ms,
            noncoherent: config.acquisition_noncoherent,
        })
        .collect()
}

fn filter_acquisition_requests(
    config: &crate::engine::receiver_config::ReceiverPipelineConfig,
    acquisition_requests: &[AcqRequest],
) -> Vec<AcqRequest> {
    acquisition_requests
        .iter()
        .filter(|request| config.allows_constellation(request.sat.constellation))
        .cloned()
        .collect()
}

fn acquisition_request_sats(requests: &[AcqRequest]) -> Vec<SatId> {
    requests.iter().map(|request| request.sat).collect()
}

fn constellation_supports_slot_only_acquisition(constellation: Constellation) -> bool {
    match constellation {
        Constellation::Glonass => false,
        _ => true,
    }
}

fn acquisition_signal_matches_config(
    config: &crate::engine::receiver_config::ReceiverPipelineConfig,
    constellation: Constellation,
) -> bool {
    let Some(signal) = default_acquisition_signal(constellation) else {
        return false;
    };
    let code_length =
        signal.code_length.map(|length| length as usize).unwrap_or(config.code_length);
    (signal.spec.code_rate_hz - config.code_freq_basis_hz).abs() <= f64::EPSILON
        && code_length == config.code_length
}

fn tracking_signal_supported(acq: &AcqResult) -> bool {
    matches!(acq.hypothesis, AcqHypothesis::Accepted | AcqHypothesis::Ambiguous)
        && crate::pipeline::tracking::supports_tracking_signal(acq.sat, acq.signal_band)
}

fn build_tracking_preview_frame(
    acquisition_frame: &SamplesFrame,
    initial_tracking_frame: Option<&SamplesFrame>,
) -> SamplesFrame {
    let mut iq = acquisition_frame.iq.clone();
    if let Some(initial_tracking_frame) = initial_tracking_frame {
        iq.extend_from_slice(&initial_tracking_frame.iq);
    }
    SamplesFrame::new(acquisition_frame.t0, acquisition_frame.dt_s, iq)
}

fn prioritize_tracking_acquisitions(
    tracking: &crate::pipeline::tracking::Tracking,
    preview_frame: &SamplesFrame,
    acquisitions: &[AcqResult],
    channel_limit: usize,
) -> Vec<AcqResult> {
    let mut ranked = acquisitions
        .iter()
        .filter(|acq| matches!(acq.hypothesis, AcqHypothesis::Accepted | AcqHypothesis::Ambiguous))
        .map(|acq| {
            let preview = tracking.track_from_acquisition(preview_frame, std::slice::from_ref(acq));
            let summary = preview
                .first()
                .map(|result| preview_tracking_summary(&result.epochs))
                .unwrap_or_default();
            (summary, acq.clone())
        })
        .collect::<Vec<_>>();
    ranked.sort_by(|(left_summary, left_acq), (right_summary, right_acq)| {
        compare_tracking_preview_summaries(left_summary, right_summary)
            .then_with(|| {
                acquisition_hypothesis_rank(left_acq.hypothesis)
                    .cmp(&acquisition_hypothesis_rank(right_acq.hypothesis))
            })
            .then_with(|| right_acq.score.total_cmp(&left_acq.score))
            .then_with(|| right_acq.cn0_proxy.total_cmp(&left_acq.cn0_proxy))
            .then_with(|| left_acq.sat.constellation.cmp(&right_acq.sat.constellation))
            .then_with(|| left_acq.sat.prn.cmp(&right_acq.sat.prn))
    });
    ranked.into_iter().take(channel_limit.max(1)).map(|(_, acq)| acq).collect()
}

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
struct TrackingPreviewSummary {
    locked_epoch_count: usize,
    first_locked_epoch_index: Option<usize>,
}

fn preview_tracking_summary(epochs: &[TrackEpoch]) -> TrackingPreviewSummary {
    let first_locked_epoch_index = epochs.iter().position(|epoch| epoch.lock && epoch.dll_lock);
    let locked_epoch_count = epochs.iter().filter(|epoch| epoch.lock && epoch.dll_lock).count();
    TrackingPreviewSummary { locked_epoch_count, first_locked_epoch_index }
}

fn compare_tracking_preview_summaries(
    left: &TrackingPreviewSummary,
    right: &TrackingPreviewSummary,
) -> std::cmp::Ordering {
    (right.locked_epoch_count > 0)
        .cmp(&(left.locked_epoch_count > 0))
        .then_with(|| right.locked_epoch_count.cmp(&left.locked_epoch_count))
        .then_with(|| match (left.first_locked_epoch_index, right.first_locked_epoch_index) {
            (Some(left_index), Some(right_index)) => left_index.cmp(&right_index),
            (Some(_), None) => std::cmp::Ordering::Less,
            (None, Some(_)) => std::cmp::Ordering::Greater,
            (None, None) => std::cmp::Ordering::Equal,
        })
}

fn acquisition_hypothesis_rank(hypothesis: AcqHypothesis) -> u8 {
    match hypothesis {
        AcqHypothesis::Accepted => 0,
        AcqHypothesis::Ambiguous => 1,
        AcqHypothesis::Rejected => 2,
        AcqHypothesis::Deferred => 3,
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

#[cfg(test)]
mod tests {
    use super::{
        acquisition_requests_for_satellites, default_acquisition_requests,
        filter_acquisition_requests,
    };
    use crate::engine::receiver_config::{ConstellationSelectionPolicy, ReceiverPipelineConfig};
    use bijux_gnss_core::api::{Constellation, SatId};

    #[test]
    fn default_acquisition_requests_honor_galileo_only_policy() {
        let config = ReceiverPipelineConfig {
            constellation_policy: ConstellationSelectionPolicy::GalileoOnly,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 4092,
            ..ReceiverPipelineConfig::default()
        };

        let requests = filter_acquisition_requests(&config, &default_acquisition_requests(&config));

        assert!(!requests.is_empty());
        assert!(requests.iter().all(|request| request.sat.constellation == Constellation::Galileo));
    }

    #[test]
    fn default_acquisition_requests_honor_beidou_only_policy() {
        let config = ReceiverPipelineConfig {
            constellation_policy: ConstellationSelectionPolicy::BeidouOnly,
            code_freq_basis_hz: 2_046_000.0,
            code_length: 2046,
            ..ReceiverPipelineConfig::default()
        };

        let requests = filter_acquisition_requests(&config, &default_acquisition_requests(&config));

        assert!(!requests.is_empty());
        assert!(requests.iter().all(|request| request.sat.constellation == Constellation::Beidou));
    }

    #[test]
    fn mixed_policy_keeps_explicit_glonass_requests() {
        let config = ReceiverPipelineConfig::default();
        let requests = vec![
            bijux_gnss_core::api::AcqRequest {
                sat: bijux_gnss_core::api::SatId { constellation: Constellation::Glonass, prn: 8 },
                glonass_frequency_channel: bijux_gnss_core::api::GlonassFrequencyChannel::new(-4),
                doppler_search_hz: 0,
                doppler_step_hz: 250,
                coherent_ms: 1,
                noncoherent: 1,
            },
            bijux_gnss_core::api::AcqRequest {
                sat: bijux_gnss_core::api::SatId { constellation: Constellation::Gps, prn: 3 },
                glonass_frequency_channel: None,
                doppler_search_hz: 0,
                doppler_step_hz: 250,
                coherent_ms: 1,
                noncoherent: 1,
            },
        ];

        let filtered = filter_acquisition_requests(&config, &requests);

        assert_eq!(filtered.len(), 2);
        assert_eq!(filtered[0].sat.constellation, Constellation::Glonass);
        assert_eq!(filtered[0].glonass_frequency_channel, requests[0].glonass_frequency_channel);
        assert_eq!(filtered[1].sat.constellation, Constellation::Gps);
    }

    #[test]
    fn gps_only_policy_filters_out_explicit_glonass_requests() {
        let config = ReceiverPipelineConfig {
            constellation_policy: ConstellationSelectionPolicy::GpsOnly,
            ..ReceiverPipelineConfig::default()
        };
        let requests = vec![
            bijux_gnss_core::api::AcqRequest {
                sat: bijux_gnss_core::api::SatId { constellation: Constellation::Glonass, prn: 8 },
                glonass_frequency_channel: bijux_gnss_core::api::GlonassFrequencyChannel::new(-4),
                doppler_search_hz: 0,
                doppler_step_hz: 250,
                coherent_ms: 1,
                noncoherent: 1,
            },
            bijux_gnss_core::api::AcqRequest {
                sat: bijux_gnss_core::api::SatId { constellation: Constellation::Gps, prn: 3 },
                glonass_frequency_channel: None,
                doppler_search_hz: 0,
                doppler_step_hz: 250,
                coherent_ms: 1,
                noncoherent: 1,
            },
        ];

        let filtered = filter_acquisition_requests(&config, &requests);

        assert_eq!(filtered.len(), 1);
        assert_eq!(filtered[0].sat.constellation, Constellation::Gps);
    }

    #[test]
    fn explicit_satellite_requests_inherit_acquisition_search_settings() {
        let config = ReceiverPipelineConfig {
            acquisition_doppler_search_hz: 4_500,
            acquisition_doppler_step_hz: 375,
            acquisition_integration_ms: 4,
            acquisition_noncoherent: 3,
            ..ReceiverPipelineConfig::default()
        };
        let satellites = vec![
            SatId { constellation: Constellation::Gps, prn: 3 },
            SatId { constellation: Constellation::Galileo, prn: 11 },
        ];

        let requests = acquisition_requests_for_satellites(&config, &satellites);

        assert_eq!(requests.len(), satellites.len());
        for (request, sat) in requests.iter().zip(satellites) {
            assert_eq!(request.sat, sat);
            assert_eq!(request.glonass_frequency_channel, None);
            assert_eq!(request.doppler_search_hz, 4_500);
            assert_eq!(request.doppler_step_hz, 375);
            assert_eq!(request.coherent_ms, 4);
            assert_eq!(request.noncoherent, 3);
        }
    }
}
