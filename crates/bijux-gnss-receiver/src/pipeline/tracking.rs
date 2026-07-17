#![allow(missing_docs)]

use std::collections::VecDeque;

use num_complex::Complex;
use serde::{Deserialize, Serialize};

use bijux_gnss_core::api::{
    AcqHypothesis, AcqTrackingSeed, AcqUncertainty, Chips, Constellation, Cycles, FreqHz, GpsTime,
    Hertz, ReceiverSampleTrace, SampleClock, SampleTime, SamplesFrame, SatId, Seconds, SignalBand,
    SignalCode, SignalComponentRole, SignalDelayAlignment, SignalSecondaryCodeSpec, SignalSpec,
    SignalSubcarrierSpec, TrackEpoch, TrackTransition, TrackingAssumptions, TrackingTransmitTime,
    TrackingUncertainty, GPS_L1_CA_CARRIER_HZ,
};

use crate::engine::receiver_config::{ReceiverPipelineConfig, TrackingParams};
use crate::engine::runtime::{ReceiverRuntime, TraceRecord};
use crate::engine::signal_selection::default_signal_code_for_band;
use bijux_gnss_core::api::Sample;
use bijux_gnss_signal::api::samples_per_code;
use bijux_gnss_signal::api::{
    advance_tracking_adaptation, anti_false_lock_detected as signal_anti_false_lock_detected,
    apply_carrier_tracking_loop as signal_apply_carrier_tracking_loop,
    apply_code_loop as signal_apply_code_loop, calibrated_lock_detector_thresholds,
    carrier_frequency_error_hz_from_phase_delta, carrier_phase_offset_radians, code_value_at_phase,
    coherent_integration_seconds as signal_coherent_integration_seconds,
    correlate_early_prompt_late, default_local_code_model_for_signal, discriminators,
    double_delta_dll_discriminator, epoch_start_code_phase_samples_from_receiver_phase,
    estimate_cn0_dbhz, estimate_tracking_uncertainty as signal_estimate_tracking_uncertainty,
    galileo_e5a_q_epoch_symbol, galileo_e5a_q_secondary_code, galileo_e5b_q_epoch_symbol,
    galileo_e5b_q_secondary_code, generate_galileo_e5a_q_code, generate_galileo_e5b_q_code,
    generate_gps_l2c_cl_code, gps_l5_q_epoch_symbol, normalize_dll_discriminator,
    prompt_power_ratio as signal_prompt_power_ratio,
    push_tracking_uncertainty_sample as signal_push_tracking_uncertainty_sample,
    refresh_lock_reference_cn0_dbhz as signal_refresh_lock_reference_cn0_dbhz,
    refresh_prompt_power_reference as signal_refresh_prompt_power_reference,
    resolved_signal_registry_entry, shared_path_code_rate_hz,
    update_windowed_tracking_cn0_estimate as signal_update_windowed_tracking_cn0_estimate,
    wrap_code_phase_samples, wrap_phase_cycles_signed, wrapped_code_phase_delta_samples,
    wrapped_phase_delta_cycles, EarlyPromptLateCorrelatorInput, LocalCodeModel,
    LockDetectorCalibrationInput, LockDetectorThresholds,
    TrackingAdaptationInput as SignalTrackingAdaptationInput,
    TrackingAdaptationState as SignalTrackingAdaptationState,
    TrackingLoopProfile as SignalTrackingLoopProfile, TrackingQualityClass,
    TrackingUncertaintyInputs as SignalTrackingUncertaintyInputs,
    GALILEO_E5A_Q_SECONDARY_CODE_CHIPS, GALILEO_E5B_Q_SECONDARY_CODE_CHIPS,
    GPS_L5_Q_PRIMARY_EPOCHS_PER_SYMBOL,
};

include!("tracking/tracking_thresholds.rs");
include!("tracking/channel_lifecycle.rs");
include!("tracking/session_artifacts.rs");
include!("tracking/reacquisition_policy.rs");
include!("tracking/loop_state.rs");

include!("tracking/vector_tracking.rs");

include!("tracking/signal_model.rs");
include!("tracking/channel_runtime.rs");

include!("tracking/loop_corrections.rs");
include!("tracking/loop_initialization.rs");

/// Tracking engine with basic E/P/L correlation per epoch.
pub struct Tracking {
    config: ReceiverPipelineConfig,
    runtime: ReceiverRuntime,
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct TrackingSignalEstimate {
    carrier_hz: f64,
    carrier_phase_cycles: f64,
    code_rate_hz: f64,
    code_phase_samples: f64,
    early_late_spacing_chips: f64,
}

#[derive(Debug, Clone, Copy)]
struct TrackingCorrelationRange<'a> {
    frame: &'a SamplesFrame,
    start: usize,
    end: usize,
    sample_index: u64,
}

#[derive(Debug, Clone, Copy)]
struct TrackingEpochCorrelationRequest<'a> {
    range: TrackingCorrelationRange<'a>,
    signal_model: &'a TrackingSignalModel,
    estimate: TrackingSignalEstimate,
}

#[derive(Debug, Clone, Copy)]
struct TrackEpochRangeRequest<'a> {
    correlation: TrackingEpochCorrelationRequest<'a>,
    channel_id: u8,
    sat: SatId,
}

#[derive(Debug, Clone, Copy)]
struct TrackEpochSequenceRequest<'a> {
    frame: &'a SamplesFrame,
    channel_id: u8,
    sat: SatId,
    signal_model: &'a TrackingSignalModel,
    initial_estimate: TrackingSignalEstimate,
    acquisition_cn0_proxy_dbhz: f64,
    tracking_params: TrackingParams,
    epochs: usize,
}

#[derive(Debug, Clone, Copy)]
pub struct TrackingCorrelationRequest<'a> {
    pub frame: &'a SamplesFrame,
    pub sat: SatId,
    pub carrier_hz: f64,
    pub carrier_phase_cycles: f64,
    pub code_rate_hz: f64,
    pub code_phase_samples: f64,
    pub early_late_spacing_chips: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct TrackEpochRequest<'a> {
    pub frame: &'a SamplesFrame,
    pub channel_id: u8,
    pub sat: SatId,
    pub carrier_hz: f64,
    pub carrier_phase_cycles: f64,
    pub code_rate_hz: f64,
    pub code_phase_samples: f64,
    pub early_late_spacing_chips: f64,
}

impl Tracking {
    pub fn new(config: ReceiverPipelineConfig, runtime: ReceiverRuntime) -> Self {
        Self { config, runtime }
    }

    pub fn transition_state(channel: u8, from: ChannelState, to: ChannelState) -> ChannelState {
        let _ = (channel, from, to);
        to
    }

    fn tracking_start_context(
        &self,
        acquisition: &bijux_gnss_core::api::AcqResult,
    ) -> Option<TrackingStartContext> {
        if !matches!(acquisition.hypothesis, AcqHypothesis::Accepted | AcqHypothesis::Ambiguous) {
            return None;
        }
        let signal_model = TrackingSignalModel::for_sat_signal_band(
            &self.config,
            acquisition.sat,
            acquisition.signal_band,
            acquisition.signal_code,
            acquisition.glonass_frequency_channel,
        );
        let acquisition_carrier_hz =
            normalize_acquisition_carrier_hz(&self.config, &signal_model, acquisition);
        if carrier_aiding_validation_required(&signal_model) {
            if let Err(reason) =
                carrier_aiding_reference(&self.config, &signal_model, acquisition_carrier_hz)
            {
                self.runtime.trace.record(TraceRecord {
                    name: "tracking_acquisition_refused",
                    fields: vec![
                        ("constellation", format!("{:?}", acquisition.sat.constellation)),
                        ("prn", acquisition.sat.prn.to_string()),
                        ("signal_band", format!("{:?}", acquisition.signal_band)),
                        ("signal_code", format!("{:?}", acquisition.signal_code)),
                        ("reason", reason.to_string()),
                        ("acquisition_carrier_hz", format!("{acquisition_carrier_hz:.3}")),
                        ("acquisition_doppler_hz", format!("{:.3}", acquisition.doppler_hz.0)),
                    ],
                });
                return None;
            }
        }
        Some(TrackingStartContext {
            seed: acquisition.tracking_seed(),
            acquisition_hypothesis: acquisition.hypothesis.to_string(),
            acquisition_hypothesis_rank: acquisition_hypothesis_rank(acquisition.hypothesis),
            acquisition_score: acquisition.score,
            acquisition_code_phase_samples: acquisition.code_phase_samples,
            acquisition_carrier_hz,
            acquisition_cn0_proxy_dbhz: acquisition.cn0_proxy as f64,
            subcarrier_code_phase_refined: acquisition.code_phase_refinement.is_some(),
            acq_to_track_state: acq_to_track_state(&acquisition.hypothesis).to_string(),
        })
    }

    pub fn correlate_epoch(&self, request: TrackingCorrelationRequest<'_>) -> CorrelatorOutput {
        let TrackingCorrelationRequest {
            frame,
            sat,
            carrier_hz,
            carrier_phase_cycles,
            code_rate_hz,
            code_phase_samples,
            early_late_spacing_chips,
        } = request;
        let signal_model = TrackingSignalModel::for_sat(&self.config, sat);
        self.correlate_epoch_range_with_signal_model(TrackingEpochCorrelationRequest {
            range: TrackingCorrelationRange {
                frame,
                start: 0,
                end: frame.len(),
                sample_index: frame.t0.sample_index,
            },
            signal_model: &signal_model,
            estimate: TrackingSignalEstimate {
                carrier_hz,
                carrier_phase_cycles,
                code_rate_hz,
                code_phase_samples,
                early_late_spacing_chips,
            },
        })
    }

    fn correlate_epoch_range_with_signal_model(
        &self,
        request: TrackingEpochCorrelationRequest<'_>,
    ) -> CorrelatorOutput {
        self.tracking_epoch_correlation(request).primary
    }

    fn tracking_epoch_correlation(
        &self,
        request: TrackingEpochCorrelationRequest<'_>,
    ) -> TrackingEpochCorrelation {
        let TrackingEpochCorrelationRequest {
            range: TrackingCorrelationRange { frame, start, end, sample_index },
            signal_model,
            estimate:
                TrackingSignalEstimate {
                    carrier_hz: carrier_freq_hz,
                    carrier_phase_cycles,
                    code_rate_hz,
                    code_phase_samples,
                    early_late_spacing_chips,
                },
        } = request;
        let sample_rate_hz = self.config.sampling_freq_hz;
        let samples = &frame.iq[start..end];
        let primary_code_period_samples = signal_model.samples_per_code(sample_rate_hz);
        let nominal_chips_per_sample = signal_model.nominal_chips_per_sample(sample_rate_hz);
        let tracked_chips_per_sample = code_rate_hz / sample_rate_hz;
        let epoch_primary_code_period_index =
            sample_index as usize / primary_code_period_samples.max(1);
        let epoch_start_code_phase_samples = epoch_start_code_phase_samples_from_receiver_phase(
            code_phase_samples,
            primary_code_period_samples,
        );
        let base_chip_phase = epoch_start_code_phase_samples * nominal_chips_per_sample;
        let correlator_input = EarlyPromptLateCorrelatorInput {
            samples,
            sample_rate_hz,
            carrier_hz: carrier_freq_hz,
            carrier_phase_offset_radians: carrier_phase_offset_radians(carrier_phase_cycles),
            base_chip_phase,
            chips_per_sample: tracked_chips_per_sample,
            early_late_spacing_chips,
        };
        let primary = correlate_early_prompt_late(correlator_input, |chip_phase| {
            signal_model.value_at_phase(chip_phase, epoch_primary_code_period_index)
        });
        let double_delta_outer = (code_discriminator_mode(signal_model)
            == CodeDiscriminatorMode::DoubleDeltaEarlyPromptLate)
            .then(|| {
                correlate_early_prompt_late(
                    EarlyPromptLateCorrelatorInput {
                        early_late_spacing_chips: early_late_spacing_chips * 2.0,
                        ..correlator_input
                    },
                    |chip_phase| {
                        signal_model.value_at_phase(chip_phase, epoch_primary_code_period_index)
                    },
                )
            });
        let subcarrier_ambiguity_guard =
            subcarrier_ambiguity_guard(SubcarrierAmbiguityGuardRequest {
                signal_model,
                samples,
                sample_rate_hz,
                carrier_hz: carrier_freq_hz,
                carrier_phase_cycles,
                base_chip_phase,
                tracked_chips_per_sample,
                epoch_primary_code_period_index,
                prompt: primary.prompt,
            });
        let pilot_prompt = signal_model.pilot_component.as_ref().map(|component| {
            correlate_early_prompt_late(
                EarlyPromptLateCorrelatorInput {
                    early_late_spacing_chips: 0.0,
                    ..correlator_input
                },
                |chip_phase| {
                    component.sample_value_from_primary_phase(
                        chip_phase,
                        epoch_primary_code_period_index,
                        signal_model.code_length,
                    )
                },
            )
            .prompt
        });
        let data_prompt = signal_model.data_symbol_component().map(|component| {
            if component.role == signal_model.component_role {
                primary.prompt
            } else {
                correlate_early_prompt_late(
                    EarlyPromptLateCorrelatorInput {
                        early_late_spacing_chips: 0.0,
                        ..correlator_input
                    },
                    |chip_phase| {
                        component.sample_value_from_primary_phase(
                            chip_phase,
                            epoch_primary_code_period_index,
                            signal_model.code_length,
                        )
                    },
                )
                .prompt
            }
        });
        let (carrier_prompt, carrier_prompt_source) = select_carrier_prompt(
            primary.prompt,
            pilot_prompt,
            signal_model.aiding_mode,
            requires_dedicated_pilot_carrier(signal_model),
        );
        let secondary_code_prompt_period_index = prompt_center_primary_code_period_index(
            epoch_primary_code_period_index,
            base_chip_phase,
            tracked_chips_per_sample,
            samples.len(),
            signal_model.code_length,
        );
        TrackingEpochCorrelation {
            primary,
            double_delta_outer,
            carrier_prompt,
            carrier_prompt_source,
            data_prompt,
            secondary_code_prompt_period_index,
            subcarrier_ambiguity_guard,
        }
    }

    pub fn track_epoch(&self, request: TrackEpochRequest<'_>) -> (TrackEpoch, CorrelatorOutput) {
        let TrackEpochRequest {
            frame,
            channel_id,
            sat,
            carrier_hz,
            carrier_phase_cycles,
            code_rate_hz,
            code_phase_samples,
            early_late_spacing_chips,
        } = request;
        let signal_model = TrackingSignalModel::for_sat(&self.config, sat);
        let (epoch, correlation) =
            self.track_epoch_range_with_signal_model(TrackEpochRangeRequest {
                correlation: TrackingEpochCorrelationRequest {
                    range: TrackingCorrelationRange {
                        frame,
                        start: 0,
                        end: frame.len(),
                        sample_index: frame.t0.sample_index,
                    },
                    signal_model: &signal_model,
                    estimate: TrackingSignalEstimate {
                        carrier_hz,
                        carrier_phase_cycles,
                        code_rate_hz,
                        code_phase_samples,
                        early_late_spacing_chips,
                    },
                },
                channel_id,
                sat,
            });
        (epoch, correlation.primary)
    }

    fn track_epoch_range_with_signal_model(
        &self,
        request: TrackEpochRangeRequest<'_>,
    ) -> (TrackEpoch, TrackingEpochCorrelation) {
        let TrackEpochRangeRequest { correlation, channel_id, sat } = request;
        let TrackingEpochCorrelationRequest {
            range: TrackingCorrelationRange { frame, start, end, sample_index: _ },
            signal_model,
            estimate:
                TrackingSignalEstimate {
                    carrier_hz: carrier_freq_hz,
                    carrier_phase_cycles,
                    code_rate_hz,
                    code_phase_samples,
                    ..
                },
        } = correlation;
        let sample_index = frame.t0.sample_index + start as u64;
        let source_time = SampleTime { sample_index, sample_rate_hz: frame.t0.sample_rate_hz };
        let clock = SampleClock::new(self.config.sampling_freq_hz);
        let epoch = clock.epoch_from_samples(sample_index);
        let correlation = self.tracking_epoch_correlation(TrackingEpochCorrelationRequest {
            range: TrackingCorrelationRange { frame, start, end, sample_index },
            signal_model,
            estimate: correlation.estimate,
        });
        let coherent_samples = end.saturating_sub(start);
        let cn0_dbhz = estimate_cn0_dbhz(
            correlation.primary.prompt,
            correlation.primary.early - correlation.primary.late,
            self.config.sampling_freq_hz,
            coherent_samples as f64,
            correlation.primary.early_late_noise_weight_energy,
        );
        if !correlation.primary.prompt.re.is_finite() || !correlation.primary.prompt.im.is_finite()
        {
            self.runtime.logger.event(&bijux_gnss_core::api::DiagnosticEvent::new(
                bijux_gnss_core::api::DiagnosticSeverity::Error,
                "TRACK_NUMERIC_INVALID",
                "tracking correlator produced NaN/Inf",
            ));
        }
        let track_epoch = TrackEpoch {
            epoch,
            sample_index,
            source_time: ReceiverSampleTrace::from_sample_time(source_time),
            sat,
            signal_band: signal_model.signal_band,
            signal_code: signal_model.signal_code,
            glonass_frequency_channel: signal_model.glonass_frequency_channel,
            prompt_i: correlation.primary.prompt.re,
            prompt_q: correlation.primary.prompt.im,
            early_i: correlation.primary.early.re,
            early_q: correlation.primary.early.im,
            late_i: correlation.primary.late.re,
            late_q: correlation.primary.late.im,
            carrier_hz: Hertz(carrier_freq_hz),
            carrier_phase_cycles: Cycles(carrier_phase_cycles),
            code_rate_hz: Hertz(code_rate_hz),
            code_phase_samples: Chips(code_phase_samples),
            lock: correlation.primary.prompt.norm() > 0.0,
            cn0_dbhz,
            pll_lock: false,
            dll_lock: false,
            fll_lock: false,
            cycle_slip: false,
            nav_bit_lock: false,
            navigation_bit_sign: None,
            dll_err: 0.0,
            pll_err: 0.0,
            fll_err: 0.0,
            anti_false_lock: false,
            cycle_slip_reason: None,
            lock_state: ChannelState::Idle.to_string(),
            lock_state_reason: Some("initializing".to_string()),
            channel_id: Some(channel_id),
            channel_uid: tracking_channel_uid(sat, channel_id),
            tracking_provenance: format!(
                "channel={} sat={:?}-{}",
                channel_id, sat.constellation, sat.prn
            ),
            tracking_assumptions: Some(default_tracking_assumptions(&self.config, signal_model)),
            tracking_uncertainty: None,
            signal_delay_alignment: None,
            transmit_time: None,
            processing_ms: None,
        };
        (track_epoch, correlation)
    }

    pub fn run(&self, samples: &[Sample]) -> Vec<TrackingResult> {
        if samples.is_empty() {
            return Vec::new();
        }
        let frame = SamplesFrame::new(
            SampleTime { sample_index: 0, sample_rate_hz: self.config.sampling_freq_hz },
            Seconds(1.0 / self.config.sampling_freq_hz),
            samples.to_vec(),
        );
        let sat = SatId { constellation: Constellation::Gps, prn: 1 };
        let signal_model = TrackingSignalModel::for_sat(&self.config, sat);
        let tracking_params = resolve_signal_tracking_params(&self.config, &signal_model);
        let epochs = self.track_epochs(TrackEpochSequenceRequest {
            frame: &frame,
            channel_id: 0,
            sat,
            signal_model: &signal_model,
            initial_estimate: TrackingSignalEstimate {
                carrier_hz: 0.0,
                carrier_phase_cycles: 0.0,
                code_rate_hz: self.config.code_freq_basis_hz,
                code_phase_samples: 0.0,
                early_late_spacing_chips: tracking_params.early_late_spacing_chips,
            },
            acquisition_cn0_proxy_dbhz: f64::INFINITY,
            tracking_params,
            epochs: 5,
        });
        vec![TrackingResult {
            sat,
            carrier_hz: 0.0,
            code_phase_samples: 0.0,
            acquisition_hypothesis: AcqHypothesis::Deferred.to_string(),
            acquisition_score: 0.0,
            acquisition_code_phase_samples: 0,
            acquisition_carrier_hz: 0.0,
            acq_to_track_state: "deferred".to_string(),
            epochs: epochs.0,
            transitions: epochs.1,
        }]
    }

    pub fn track_from_acquisition(
        &self,
        frame: &SamplesFrame,
        acquisitions: &[bijux_gnss_core::api::AcqResult],
    ) -> Vec<TrackingResult> {
        let mut session = self.begin_tracking_session(acquisitions);
        self.track_session_frame(&mut session, frame);
        self.finish_tracking_session(session).tracking
    }

    pub fn begin_tracking_session(
        &self,
        acquisitions: &[bijux_gnss_core::api::AcqResult],
    ) -> TrackingSession {
        TrackingSession {
            tracking: self.begin_incremental_tracking(acquisitions),
            processed_input_samples: 0,
            processed_input_epochs: 0,
        }
    }

    pub fn track_session_frame(&self, session: &mut TrackingSession, frame: &SamplesFrame) {
        let samples_per_code = samples_per_code(
            self.config.sampling_freq_hz,
            self.config.code_freq_basis_hz,
            self.config.code_length,
        );
        session.processed_input_samples += frame.len() as u64;
        session.processed_input_epochs += code_periods_in_frame(frame.len(), samples_per_code);
        self.track_incremental_frame(&mut session.tracking, frame);
    }

    pub fn finish_tracking_session(&self, session: TrackingSession) -> TrackingArtifacts {
        let common_frequency =
            session.tracking.vector_state.common_frequency_estimate(self.config.sampling_freq_hz);
        let tracking = self.finish_incremental_tracking(session.tracking);
        let track_transitions =
            tracking.iter().flat_map(|result| result.transitions.iter().cloned()).collect();
        let channel_state_reports =
            tracking.iter().map(tracking_channel_state_report).collect::<Vec<_>>();
        TrackingArtifacts {
            processed_input_samples: session.processed_input_samples,
            processed_input_epochs: session.processed_input_epochs,
            track_transitions,
            channel_state_reports,
            tracking,
            common_frequency,
        }
    }

    fn apply_sample_rate_mismatch_diagnostic(
        &self,
        sat: SatId,
        acquisition_uncertainty: Option<&AcqUncertainty>,
        epochs: &mut [TrackEpoch],
    ) {
        if let Some(acquisition_uncertainty) = acquisition_uncertainty {
            if acquisition_uncertainty.code_phase_samples > 0.5 + f64::EPSILON {
                return;
            }
            if acquisition_uncertainty.doppler_hz
                > self.config.acquisition_doppler_step_hz.max(1) as f64 + f64::EPSILON
            {
                return;
            }
        }
        let samples_per_code = samples_per_code(
            self.config.sampling_freq_hz,
            self.config.code_freq_basis_hz,
            self.config.code_length,
        );
        let Some(diagnostic) = detect_sample_rate_mismatch(epochs, samples_per_code) else {
            return;
        };

        let first_epoch = &epochs[diagnostic.first_unstable_epoch_index];
        self.runtime.trace.record(TraceRecord {
            name: "tracking_sample_rate_mismatch",
            fields: vec![
                ("constellation", format!("{:?}", sat.constellation)),
                ("prn", sat.prn.to_string()),
                ("first_epoch_idx", first_epoch.epoch.index.to_string()),
                (
                    "max_abs_phase_step_samples",
                    format!("{:.6}", diagnostic.max_abs_phase_step_samples),
                ),
                ("phase_step_limit_samples", format!("{:.6}", diagnostic.phase_step_limit_samples)),
            ],
        });

        for epoch in epochs.iter_mut().skip(diagnostic.first_unstable_epoch_index) {
            let replaceable_reason = epoch.lock_state_reason.as_deref().is_none_or(|reason| {
                matches!(
                    reason,
                    "carrier_pull_in" | "carrier_converged" | "fade_recovered" | "signal_fade"
                )
            });
            if replaceable_reason {
                epoch.lock_state_reason = Some("sample_rate_mismatch".to_string());
            }
        }
    }

    fn track_epochs(
        &self,
        request: TrackEpochSequenceRequest<'_>,
    ) -> (Vec<TrackEpoch>, Vec<TrackTransition>) {
        let TrackEpochSequenceRequest {
            frame,
            channel_id,
            sat,
            signal_model,
            initial_estimate,
            acquisition_cn0_proxy_dbhz,
            tracking_params,
            epochs,
        } = request;
        let mut state = self.initial_loop_state(TrackingLoopInitialization {
            signal_model,
            carrier_hz: initial_estimate.carrier_hz,
            code_phase_samples: initial_estimate.code_phase_samples,
            acquisition_cn0_proxy_dbhz,
            signal_delay_alignment: None,
            subcarrier_code_phase_refined: false,
            tracking_params,
            reacquisition_pending: false,
        });

        let mut out = Vec::new();
        let mut transitions = Vec::new();
        self.append_tracked_epochs(
            frame,
            channel_id,
            sat,
            signal_model,
            tracking_params,
            epochs,
            &mut state,
            &mut out,
            &mut transitions,
        );
        (out, transitions)
    }

    pub(crate) fn begin_incremental_tracking(
        &self,
        acquisitions: &[bijux_gnss_core::api::AcqResult],
    ) -> IncrementalTrackingState {
        let mut start_contexts = acquisitions
            .iter()
            .filter_map(|acq| self.tracking_start_context(acq))
            .collect::<Vec<_>>();
        start_contexts.sort_by(compare_tracking_start_contexts);

        let channels = start_contexts
            .into_iter()
            .take(self.config.channels.max(1))
            .enumerate()
            .map(|(channel_idx, context)| {
                let channel_id = channel_idx as u8;
                let signal_model = TrackingSignalModel::for_sat_signal_band(
                    &self.config,
                    context.seed.sat,
                    context.seed.signal_band,
                    context.seed.signal_code,
                    context.seed.glonass_frequency_channel,
                );
                let tracking_params = resolve_signal_tracking_params(&self.config, &signal_model);
                self.runtime.trace.record(TraceRecord {
                    name: "tracking_sat_start",
                    fields: vec![
                        ("constellation", format!("{:?}", context.seed.sat.constellation)),
                        ("prn", context.seed.sat.prn.to_string()),
                        ("hypothesis", context.acquisition_hypothesis.clone()),
                        ("to_track", context.acq_to_track_state.clone()),
                        ("cn0_proxy_dbhz", format!("{:.3}", context.acquisition_cn0_proxy_dbhz)),
                    ],
                });
                IncrementalTrackingChannel {
                    state: self.initial_loop_state(TrackingLoopInitialization {
                        signal_model: &signal_model,
                        carrier_hz: context.acquisition_carrier_hz,
                        code_phase_samples: context.seed.code_phase_samples.0,
                        acquisition_cn0_proxy_dbhz: context.acquisition_cn0_proxy_dbhz,
                        signal_delay_alignment: context.seed.signal_delay_alignment.clone(),
                        subcarrier_code_phase_refined: context.subcarrier_code_phase_refined,
                        tracking_params,
                        reacquisition_pending: false,
                    }),
                    sat: context.seed.sat,
                    channel_id,
                    start_source_time: context.seed.source_time,
                    signal_band: context.seed.signal_band,
                    signal_model: signal_model.clone(),
                    acquisition_uncertainty: context.seed.uncertainty.clone(),
                    acquisition_hypothesis: context.acquisition_hypothesis,
                    acquisition_score: context.acquisition_score,
                    acquisition_code_phase_samples: context.acquisition_code_phase_samples,
                    acquisition_doppler_hz: context.seed.doppler_hz.0,
                    acquisition_resolved_code_phase_samples: context.seed.code_phase_samples.0,
                    acquisition_carrier_hz: context.acquisition_carrier_hz,
                    acq_to_track_state: context.acq_to_track_state,
                    tracking_params,
                    epochs: Vec::new(),
                    transitions: Vec::new(),
                }
            })
            .collect();
        IncrementalTrackingState { channels, vector_state: VectorTrackingState::default() }
    }

    pub(crate) fn track_incremental_frame(
        &self,
        tracking: &mut IncrementalTrackingState,
        frame: &SamplesFrame,
    ) {
        let vector_state = &mut tracking.vector_state;
        for channel in &mut tracking.channels {
            let Some(channel_frame_start) = tracking_frame_start_offset(
                frame,
                if channel.epochs.is_empty() {
                    Some(channel.start_source_time.sample_index)
                } else {
                    None
                },
            ) else {
                continue;
            };
            let mut epoch_start = channel_frame_start;
            while epoch_start < frame.len() {
                let tracking_params =
                    self.tracking_params_for_state(channel.tracking_params, &channel.state);
                let samples_per_epoch = channel
                    .signal_model
                    .tracking_epoch_samples(self.config.sampling_freq_hz, tracking_params);
                let epoch_end = (epoch_start + samples_per_epoch).min(frame.len());
                let reacquisition_outcome = if channel.state.state == ChannelState::Lost {
                    let epoch_frame = frame_slice(frame, epoch_start, epoch_end);
                    self.try_reacquire_channel(channel, &epoch_frame)
                } else {
                    ReacquisitionOutcome::NotNeeded
                };
                let epoch_count_before = channel.epochs.len();
                let transition_count_before = channel.transitions.len();
                let vector_prediction = if self.config.vector_tracking_enabled {
                    vector_state.prediction_for(
                        frame.t0.sample_index + epoch_start as u64,
                        self.config.sampling_freq_hz,
                    )
                } else {
                    None
                };
                let measurement = self.append_tracked_epoch_range(
                    frame,
                    epoch_start,
                    epoch_end,
                    channel.channel_id,
                    channel.sat,
                    &channel.signal_model,
                    channel.tracking_params,
                    tracking_params,
                    vector_prediction,
                    &mut channel.state,
                    &mut channel.epochs,
                    &mut channel.transitions,
                );
                if self.config.vector_tracking_enabled {
                    if let Some(measurement) = measurement {
                        vector_state.record(measurement, self.config.sampling_freq_hz);
                    }
                }
                apply_reacquisition_annotations(
                    &mut channel.state,
                    &mut channel.epochs[epoch_count_before..],
                    &mut channel.transitions[transition_count_before..],
                    reacquisition_outcome,
                );
                epoch_start = epoch_end;
            }
        }
    }

    pub(crate) fn finish_incremental_tracking(
        &self,
        tracking: IncrementalTrackingState,
    ) -> Vec<TrackingResult> {
        tracking
            .channels
            .into_iter()
            .map(|mut channel| {
                for epoch in &mut channel.epochs {
                    let runtime_tracking_provenance = epoch.tracking_provenance.clone();
                    epoch.tracking_provenance = format!(
                        "acq_hypothesis={} acq_score={:.6} acq_signal_band={:?} acq_doppler_hz={:.3} acq_carrier_hz={:.3} acq_code_phase_samples={} acq_resolved_code_phase_samples={:.6} acq_start_sample_index={} track_component_role={:?} nominal_carrier_hz={:.3} secondary_code={} discriminator_family={} code_discriminator={} phase_transition_source={} aiding_mode={} pilot_component={} data_symbol_component={}",
                        channel.acquisition_hypothesis,
                        channel.acquisition_score,
                        channel.signal_band,
                        channel.acquisition_doppler_hz,
                        channel.acquisition_carrier_hz,
                        channel.acquisition_code_phase_samples,
                        channel.acquisition_resolved_code_phase_samples,
                        channel.start_source_time.sample_index,
                        channel.signal_model.component_role,
                        channel.signal_model.nominal_carrier_hz(),
                        channel.signal_model.secondary_code.is_some(),
                        channel.signal_model.discriminator_family.label(),
                        code_discriminator_mode(&channel.signal_model).label(),
                        channel.signal_model.phase_transition_source.label(),
                        channel.signal_model.aiding_mode.label(),
                        channel.signal_model.pilot_component.is_some(),
                        channel.signal_model.data_symbol_component.is_some(),
                    );
                    epoch.tracking_provenance.push_str(&format!(
                        " code_rate_reference={} carrier_aiding_doppler_window_hz={:.3}",
                        code_rate_reference_label(&self.config, &channel.signal_model, epoch.carrier_hz.0),
                        carrier_aiding_doppler_window_hz(&self.config),
                    ));
                    if let Some(secondary_code_sync_provenance) = secondary_code_sync_provenance(
                        &channel.signal_model,
                        channel.state.secondary_code_sync,
                    ) {
                        epoch.tracking_provenance.push_str(&secondary_code_sync_provenance);
                    }
                    if runtime_tracking_provenance.starts_with("vector_tracking=applied") {
                        epoch.tracking_provenance.push(' ');
                        epoch.tracking_provenance.push_str(&runtime_tracking_provenance);
                    } else if let Some(subcarrier_ambiguity_provenance) =
                        runtime_tracking_provenance.find("subcarrier_ambiguity_guard=").map(
                            |start| runtime_tracking_provenance[start..].to_string(),
                        )
                    {
                        epoch.tracking_provenance.push(' ');
                        epoch.tracking_provenance.push_str(&subcarrier_ambiguity_provenance);
                    }
                    if let Some(doppler_estimator_provenance) = tracking_provenance_segment(
                        &runtime_tracking_provenance,
                        "doppler_estimator_consistency=",
                        DOPPLER_ESTIMATOR_PROVENANCE_TOKEN_COUNT,
                    ) {
                        epoch.tracking_provenance.push(' ');
                        epoch.tracking_provenance.push_str(&doppler_estimator_provenance);
                    }
                }
                self.apply_sample_rate_mismatch_diagnostic(
                    channel.sat,
                    channel.acquisition_uncertainty.as_ref(),
                    &mut channel.epochs,
                );
                annotate_navigation_bit_signs(&channel.signal_model, &mut channel.epochs);
                annotate_lnav_transmit_times(
                    &channel.signal_model,
                    self.runtime.config.capture_start_gps_time,
                    &mut channel.epochs,
                );
                stabilize_joint_navigation_bit_signs(&channel.signal_model, &mut channel.epochs);
                let stability_signature = tracking_stability_signature(&channel.epochs);
                let outcome = if channel.epochs.is_empty() { "not_tracked" } else { "tracked" };
                self.runtime.trace.record(TraceRecord {
                    name: "tracking_sat_done",
                    fields: vec![
                        ("constellation", format!("{:?}", channel.sat.constellation)),
                        ("prn", channel.sat.prn.to_string()),
                        ("outcome", outcome.to_string()),
                        ("hypothesis", channel.acquisition_hypothesis.clone()),
                        ("epochs", channel.epochs.len().to_string()),
                        ("stability_signature", stability_signature),
                    ],
                });
                TrackingResult {
                    sat: channel.sat,
                    carrier_hz: channel.acquisition_carrier_hz,
                    code_phase_samples: channel.acquisition_resolved_code_phase_samples,
                    acquisition_hypothesis: channel.acquisition_hypothesis,
                    acquisition_score: channel.acquisition_score,
                    acquisition_code_phase_samples: channel.acquisition_code_phase_samples,
                    acquisition_carrier_hz: channel.acquisition_carrier_hz,
                    acq_to_track_state: channel.acq_to_track_state,
                    epochs: channel.epochs,
                    transitions: channel.transitions,
                }
            })
            .collect()
    }

    #[allow(clippy::too_many_arguments)]
    fn append_tracked_epochs(
        &self,
        frame: &SamplesFrame,
        channel_id: u8,
        sat: SatId,
        signal_model: &TrackingSignalModel,
        base_tracking_params: TrackingParams,
        epochs: usize,
        state: &mut LoopState,
        out: &mut Vec<TrackEpoch>,
        transitions: &mut Vec<TrackTransition>,
    ) {
        let mut start = 0usize;
        for _ in 0..epochs {
            let tracking_params = self.tracking_params_for_state(base_tracking_params, state);
            let samples_per_epoch =
                signal_model.tracking_epoch_samples(self.config.sampling_freq_hz, tracking_params);
            let end = (start + samples_per_epoch).min(frame.len());
            if start >= end {
                break;
            }
            self.append_tracked_epoch_range(
                frame,
                start,
                end,
                channel_id,
                sat,
                signal_model,
                base_tracking_params,
                tracking_params,
                None,
                state,
                out,
                transitions,
            );
            start = end;
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn append_tracked_epoch_range(
        &self,
        frame: &SamplesFrame,
        start: usize,
        end: usize,
        channel_id: u8,
        sat: SatId,
        signal_model: &TrackingSignalModel,
        base_tracking_params: TrackingParams,
        tracking_params: TrackingParams,
        vector_prediction: Option<VectorTrackingPrediction>,
        state: &mut LoopState,
        out: &mut Vec<TrackEpoch>,
        transitions: &mut Vec<TrackTransition>,
    ) -> Option<VectorTrackingMeasurement> {
        let samples_per_code = signal_model.samples_per_code(self.config.sampling_freq_hz);
        let samples_per_chip = samples_per_code as f64 / signal_model.code_length as f64;
        let vector_application =
            vector_prediction.and_then(|prediction| vector_tracking_application(prediction, state));
        let aided_carrier_hz = state.carrier_hz
            + vector_application
                .map(|application| application.carrier_frequency_correction_hz)
                .unwrap_or_default();
        let aided_code_rate_hz = state.code_rate_hz
            + vector_application
                .map(|application| application.code_rate_correction_hz)
                .unwrap_or_default();
        let aided_code_phase_samples = state.code_phase_samples
            + vector_application
                .map(|application| application.code_phase_correction_samples)
                .unwrap_or_default();
        let aided_carrier_rate_hz_per_s = state.carrier_rate_hz_per_s
            + vector_application
                .map(|application| application.carrier_rate_correction_hz_per_s)
                .unwrap_or_default();
        let alloc_before = crate::engine::alloc::allocation_count();
        let (mut track_epoch, correlation) =
            self.track_epoch_range_with_signal_model(TrackEpochRangeRequest {
                correlation: TrackingEpochCorrelationRequest {
                    range: TrackingCorrelationRange {
                        frame,
                        start,
                        end,
                        sample_index: frame.t0.sample_index + start as u64,
                    },
                    signal_model,
                    estimate: TrackingSignalEstimate {
                        carrier_hz: aided_carrier_hz,
                        carrier_phase_cycles: state.carrier_phase_cycles,
                        code_rate_hz: aided_code_rate_hz,
                        code_phase_samples: aided_code_phase_samples,
                        early_late_spacing_chips: tracking_params.early_late_spacing_chips,
                    },
                },
                channel_id,
                sat,
            });
        track_epoch.processing_ms = None;
        let alloc_after = crate::engine::alloc::allocation_count();
        if alloc_after > alloc_before {
            self.runtime.logger.event(&bijux_gnss_core::api::DiagnosticEvent::new(
                bijux_gnss_core::api::DiagnosticSeverity::Warning,
                "TRACK_ALLOCATIONS",
                format!("tracking epoch allocated {} times", alloc_after - alloc_before),
            ));
        }

        let primary_correlator = correlation.primary;
        let carrier_prompt = correlation.carrier_prompt;
        update_secondary_code_synchronization(
            signal_model,
            state,
            correlation.secondary_code_prompt_period_index,
            carrier_prompt,
        );
        let dll_err = tracking_dll_discriminator(&correlation);
        let dll_err =
            normalize_dll_discriminator(dll_err, tracking_params.early_late_spacing_chips);
        let (_raw_pll_err, raw_fll_err, lock) =
            carrier_prompt_discriminators(carrier_prompt, state.prev_prompt);
        state.prev_prompt = Some(carrier_prompt);
        let phase_cycles = carrier_prompt.arg() as f64 / (2.0 * std::f64::consts::PI);
        let phase_transition_source =
            carrier_phase_transition_source_for_prompt(signal_model, state.secondary_code_sync);
        let phase_decision = classify_prompt_phase(
            phase_cycles,
            state.prev_prompt_phase_cycles,
            state.nav_bit_phase_offset_cycles,
            phase_transition_source,
        );
        state.prev_prompt_phase_cycles = Some(phase_decision.aligned_phase_cycles);
        state.nav_bit_phase_offset_cycles = phase_decision.nav_bit_phase_offset_cycles;
        let pll_err = (phase_decision.aligned_phase_cycles * std::f64::consts::TAU) as f32;
        if phase_decision.nav_bit_transition {
            state.nav_bit_transition_count = state.nav_bit_transition_count.saturating_add(1);
        }

        let cycle_slip = phase_decision.cycle_slip;
        let cycle_slip_reason = cycle_slip.then(|| LossOfLockCause::PhaseJump.reason().to_string());
        let prompt_power = carrier_prompt.norm();

        let mut cn0_dbhz = state.acquisition_cn0_proxy_dbhz;
        if !cn0_dbhz.is_finite() || cn0_dbhz <= 0.0 {
            cn0_dbhz = track_epoch.cn0_dbhz;
        }
        let epoch_len_samples = end - start;
        if let Some(windowed_cn0_dbhz) =
            update_windowed_tracking_cn0_estimate(&mut state.prompt_cn0_window, cn0_dbhz)
        {
            cn0_dbhz = windowed_cn0_dbhz;
        }
        track_epoch.cn0_dbhz = cn0_dbhz;
        let dll_bw = tracking_params.dll_bw_hz;
        let pll_bw = tracking_params.pll_bw_hz;
        let fll_bw = tracking_params.fll_bw_hz;
        let coherent_integration_s =
            coherent_integration_seconds(epoch_len_samples, self.config.sampling_freq_hz);
        let raw_fll_err_hz =
            carrier_frequency_error_hz_from_phase_delta(raw_fll_err as f64, coherent_integration_s);
        let nav_bit_aware_fll_err_hz = carrier_frequency_error_hz_from_phase_delta(
            phase_decision.aligned_phase_delta_cycles * std::f64::consts::TAU,
            coherent_integration_s,
        );
        let use_nav_bit_aware_fll = phase_transition_source.allows_half_cycle_transition()
            || phase_decision.nav_bit_transition
            || state.nav_bit_phase_offset_cycles.abs() > f64::EPSILON
            || phase_decision.nav_bit_phase_offset_cycles.abs() > f64::EPSILON;
        let fll_err_hz =
            if use_nav_bit_aware_fll { nav_bit_aware_fll_err_hz } else { raw_fll_err_hz } as f32;
        let from_state = state.state;
        let lock_detector_thresholds = tracking_lock_detector_thresholds(
            cn0_dbhz,
            coherent_integration_s,
            samples_per_chip,
            tracking_params,
            state.carrier_rate_hz_per_s,
        );
        let raw_dll_lock = dll_err.abs() < lock_detector_thresholds.dll_lock;
        let subcarrier_ambiguity =
            subcarrier_ambiguity_detected(correlation.subcarrier_ambiguity_guard);
        let subcarrier_code_phase_unambiguous =
            !signal_model.discriminator_family.requires_unambiguous_code_lock()
                || state.subcarrier_code_phase_refined;
        let fll_enabled = fll_bw > f64::EPSILON;
        let raw_pll_lock = pll_err.abs() < lock_detector_thresholds.pll_lock_rad;
        let raw_fll_lock =
            !fll_enabled || (fll_err_hz as f64).abs() <= lock_detector_thresholds.fll_lock_hz;
        let prompt_correlation_fll_err_hz =
            if use_nav_bit_aware_fll { nav_bit_aware_fll_err_hz } else { raw_fll_err_hz };
        let doppler_consistency = doppler_estimator_consistency(
            0.0,
            nav_bit_aware_fll_err_hz,
            prompt_correlation_fll_err_hz,
            lock_detector_thresholds.fll_lock_hz,
        );
        let sustained_dll_lock = raw_dll_lock
            || (matches!(
                from_state,
                ChannelState::PullIn | ChannelState::Tracking | ChannelState::Degraded
            ) && dll_err.abs() < lock_detector_thresholds.dll_hold);
        let sustained_pll_lock = raw_pll_lock
            || (matches!(
                from_state,
                ChannelState::PullIn | ChannelState::Tracking | ChannelState::Degraded
            ) && pll_err.abs() < lock_detector_thresholds.pll_hold_rad);
        let detected_anti_false_lock = anti_false_lock_detected(
            primary_correlator.early,
            primary_correlator.prompt,
            primary_correlator.late,
        ) && !sustained_pll_lock;
        let prompt_power_ratio = prompt_power_ratio(prompt_power, state.prompt_power_reference);
        let prompt_power_supports_lock = prompt_power_ratio
            .is_some_and(|ratio| ratio >= DISCRIMINATOR_INSTABILITY_MIN_PROMPT_POWER_RATIO);
        let anti_false_lock = detected_anti_false_lock
            && !low_resolution_false_lock_override(
                samples_per_chip,
                tracking_params.early_late_spacing_chips,
                lock || prompt_power_supports_lock,
                prompt_power_supports_lock,
                raw_fll_lock,
                doppler_consistency.consistent,
                pll_err,
                cycle_slip,
            );
        state.prompt_power_reference = refresh_prompt_power_reference(
            state.prompt_power_reference,
            prompt_power,
            state.state,
            anti_false_lock,
        );
        let sustained_prompt_lock = lock
            || (matches!(
                state.state,
                ChannelState::PullIn | ChannelState::Tracking | ChannelState::Degraded
            ) && prompt_power_supports_lock
                && !cycle_slip
                && !anti_false_lock);
        let sustained_code_lock = sustained_dll_lock
            || low_resolution_code_lock(
                samples_per_chip,
                tracking_params.early_late_spacing_chips,
                sustained_prompt_lock,
                sustained_pll_lock,
                raw_fll_lock,
                cycle_slip,
                anti_false_lock,
            );
        state.unstable_discriminator_epochs = update_discriminator_instability_epochs(
            state.unstable_discriminator_epochs,
            state.state,
            prompt_power_ratio,
            raw_pll_lock,
            raw_fll_lock,
            cycle_slip,
            anti_false_lock,
        );
        let loss_of_lock_cause = classify_loss_of_lock_cause(
            state.state,
            cycle_slip,
            prompt_power_ratio,
            state.unstable_discriminator_epochs,
        );
        let (weak_cn0_epochs, cn0_supports_lock, refuse_lock) = update_prelock_cn0_refusal(
            state.state,
            state.weak_cn0_epochs,
            state.acquisition_cn0_proxy_dbhz,
        );
        state.weak_cn0_epochs = weak_cn0_epochs;
        let carrier_convergence_lock = cn0_supports_lock
            && sustained_prompt_lock
            && sustained_code_lock
            && raw_fll_lock
            && doppler_consistency.consistent
            && pll_err.abs() <= CARRIER_CONVERGENCE_MAX_PHASE_ERROR_RAD
            && !cycle_slip
            && !anti_false_lock;
        let sustained_or_converged_pll_lock = sustained_pll_lock || carrier_convergence_lock;
        state.pull_in_stable_epochs = update_pull_in_stable_epochs(
            state.pull_in_stable_epochs,
            sustained_prompt_lock,
            sustained_code_lock,
            sustained_or_converged_pll_lock,
            raw_fll_lock,
            cycle_slip,
        );
        let discriminator_feedback_ready = sustained_prompt_lock
            && prompt_power_ratio.is_none_or(|ratio| {
                ratio >= DISCRIMINATOR_INSTABILITY_MIN_PROMPT_POWER_RATIO
            })
            && !cycle_slip
            && !anti_false_lock;
        let steady_state_tracking_ready = if from_state == ChannelState::Degraded {
            cn0_supports_lock
                && discriminator_feedback_ready
                && prompt_power_supports_lock
                && sustained_code_lock
                && (raw_pll_lock || carrier_convergence_lock)
                && raw_fll_lock
                && doppler_consistency.consistent
                && !cycle_slip
                && !anti_false_lock
        } else {
            cn0_supports_lock
                && sustained_prompt_lock
                && sustained_code_lock
                && sustained_or_converged_pll_lock
                && doppler_consistency.consistent
                && !cycle_slip
                && !anti_false_lock
        };
        let ready_for_tracking =
            if matches!(from_state, ChannelState::Tracking | ChannelState::Degraded) {
                steady_state_tracking_ready
            } else {
                cn0_supports_lock
                    && sustained_or_converged_pll_lock
                    && state.pull_in_stable_epochs >= PULL_IN_REQUIRED_STABLE_EPOCHS
            };
        let short_fade_relock_evidence = doppler_consistency.consistent
            && !anti_false_lock
            && !cycle_slip
            && (raw_pll_lock || raw_fll_lock);
        let reliable_reacquisition_reference = cn0_supports_lock
            && sustained_prompt_lock
            && raw_pll_lock
            && raw_fll_lock
            && doppler_consistency.consistent
            && !cycle_slip
            && !anti_false_lock;
        state.lock_reference_cn0_dbhz = refresh_lock_reference_cn0_dbhz(
            state.lock_reference_cn0_dbhz,
            cn0_dbhz,
            reliable_reacquisition_reference,
        );
        let doppler_estimator_degraded = !doppler_consistency.consistent
            && matches!(from_state, ChannelState::Tracking | ChannelState::Degraded)
            && sustained_prompt_lock
            && sustained_code_lock
            && !cycle_slip
            && !anti_false_lock;
        let degraded_tracking_reason =
            doppler_estimator_degraded.then_some("doppler_estimator_divergence");

        let transition = if refuse_lock {
            TransitionDecision {
                to_state: ChannelState::PullIn,
                reason: "cn0_below_tracking_lock_floor".to_string(),
                next_unlocked_count: state.unlocked_count.saturating_add(1),
                next_degraded_epochs: 0,
            }
        } else {
            deterministic_transition_rule(ChannelTransitionRequest {
                from_state,
                lock: sustained_prompt_lock,
                ready_for_tracking,
                anti_false_lock,
                loss_of_lock_cause,
                unlocked_count: state.unlocked_count,
                degraded_epochs: state.degraded_epochs,
                short_fade_epoch_budget: short_fade_epoch_budget(tracking_params),
                short_fade_relock_evidence,
                degraded_tracking_reason,
            })
        };
        state.unlocked_count = transition.next_unlocked_count;
        state.degraded_epochs = transition.next_degraded_epochs;
        state.state = transition.to_state;
        push_tracking_uncertainty_sample(
            &mut state.code_error_window_samples,
            (dll_err.abs() as f64) * samples_per_chip,
        );
        push_tracking_uncertainty_sample(
            &mut state.carrier_phase_error_window_cycles,
            (pll_err.abs() as f64) / std::f64::consts::TAU,
        );
        push_tracking_uncertainty_sample(
            &mut state.doppler_error_window_hz,
            doppler_estimator_uncertainty_sample_hz(doppler_consistency, fll_err_hz as f64),
        );
        push_tracking_uncertainty_sample(&mut state.cn0_estimate_window_dbhz, cn0_dbhz);
        let tracking_uncertainty = Some(estimate_tracking_uncertainty(
            state,
            TrackingUncertaintyInputs {
                samples_per_chip,
                dll_err,
                pll_err_rad: pll_err as f64,
                fll_err_hz: fll_err_hz as f64,
                cn0_dbhz,
                cn0_reference_dbhz: state.lock_reference_cn0_dbhz,
                integration_ms: tracking_params.integration_ms,
                channel_locked: state.state != ChannelState::Lost && sustained_prompt_lock,
                dll_locked: state.state != ChannelState::Lost
                    && sustained_code_lock
                    && subcarrier_code_phase_unambiguous
                    && !subcarrier_ambiguity,
                anti_false_lock,
                cycle_slip,
                channel_state: state.state,
            },
        ));
        if state.state == ChannelState::Lost {
            state.reacquisition_pending = false;
            state.reacquisition_attempt_epochs = 0;
            clear_tracking_uncertainty_windows(state);
            if from_state != ChannelState::Lost {
                state.lost_reason = Some(transition.reason.clone());
                state.reacquisition_candidate = None;
                state.reacquisition_candidate_streak = 0;
                state.reacquisition_stable_tracking_epochs = 0;
            }
        } else {
            state.lost_reason = None;
            state.reacquisition_candidate = None;
            state.reacquisition_candidate_streak = 0;
            if !state.reacquisition_pending {
                state.reacquisition_stable_tracking_epochs = 0;
            }
            if !state.reacquisition_pending {
                state.reacquisition_attempt_epochs = 0;
            }
            if !matches!(state.state, ChannelState::Tracking | ChannelState::Degraded) {
                clear_tracking_uncertainty_windows(state);
            }
        }

        let lock_state = match state.state {
            ChannelState::Idle => "idle".to_string(),
            ChannelState::Acquired => "acquired".to_string(),
            ChannelState::PullIn => "pull_in".to_string(),
            ChannelState::Tracking => "tracking".to_string(),
            ChannelState::Degraded => "degraded".to_string(),
            ChannelState::Lost => "lost".to_string(),
        };
        let lock_state_reason = Some(transition.reason.clone());
        let channel_locked = state.state != ChannelState::Lost && sustained_prompt_lock;
        let tracking_state_locked =
            matches!(state.state, ChannelState::Tracking | ChannelState::Degraded);
        let dll_lock = tracking_state_locked
            && sustained_code_lock
            && subcarrier_code_phase_unambiguous
            && !subcarrier_ambiguity;
        let pll_lock = tracking_state_locked
            && cn0_supports_lock
            && sustained_prompt_lock
            && sustained_or_converged_pll_lock
            && !cycle_slip;
        let fll_lock = state.state != ChannelState::Lost
            && doppler_consistency.consistent
            && (raw_fll_lock || (tracking_state_locked && pll_lock));
        let navigation_bit_sign = recover_epoch_navigation_bit_sign(
            signal_model,
            correlation.data_prompt,
            carrier_prompt,
            correlation.carrier_prompt_source,
            phase_decision.nav_bit_phase_offset_cycles,
            tracking_state_locked && dll_lock && (pll_lock || fll_lock),
        );
        let nav_bit_lock = navigation_bit_sign.is_some()
            || (signal_model.phase_transition_source.reports_navigation_bit_lock()
                && state.nav_bit_transition_count > 0);
        if track_epoch.early_i.is_infinite() || track_epoch.late_i.is_infinite() {
            self.runtime.logger.event(&bijux_gnss_core::api::DiagnosticEvent::new(
                bijux_gnss_core::api::DiagnosticSeverity::Warning,
                "TRACK_NUMERIC_INVALID",
                "infinite tracking sample in early/late output",
            ));
        }

        let apply_fll = discriminator_feedback_ready
            && apply_fll_during_epoch(fll_bw, state.state, raw_fll_lock, doppler_consistency);
        let tracked_center_hz =
            tracked_signal_center_hz(self.config.intermediate_freq_hz, signal_model.signal_spec);
        let current_carrier_doppler_hz = tracked_signal_doppler_hz(
            self.config.intermediate_freq_hz,
            aided_carrier_hz,
            signal_model.signal_spec,
        );
        let carrier_loop = apply_carrier_loop(CarrierLoopInput {
            current_carrier_hz: current_carrier_doppler_hz,
            current_carrier_phase_cycles: state.carrier_phase_cycles,
            current_carrier_rate_hz_per_s: aided_carrier_rate_hz_per_s,
            epoch_len_samples,
            sample_rate_hz: self.config.sampling_freq_hz,
            coherent_integration_s,
            pll_bw_hz: pll_bw,
            pll_err_rad: pll_err as f64,
            fll_bw_hz: fll_bw,
            fll_err_hz: fll_err_hz as f64,
            apply_fll,
            apply_pll_frequency: discriminator_feedback_ready
                && (!apply_fll || matches!(state.state, ChannelState::PullIn)),
            apply_pll_phase: discriminator_feedback_ready,
        });
        let tracked_carrier_hz = tracked_center_hz + carrier_loop.carrier_hz;
        let code_rate_reference_hz = next_code_rate_reference_hz(
            &self.config,
            signal_model,
            tracked_carrier_hz,
            state.code_rate_reference_hz,
            discriminator_feedback_ready && (raw_fll_lock || sustained_or_converged_pll_lock),
        );
        let code_loop = apply_dll_code_loop(CodeLoopInput {
            current_code_rate_hz: aided_code_rate_hz,
            previous_reference_code_rate_hz: state.code_rate_reference_hz,
            reference_code_rate_hz: code_rate_reference_hz,
            current_code_phase_samples: aided_code_phase_samples,
            epoch_len_samples,
            coherent_integration_s,
            nominal_code_rate_hz: signal_model.code_rate_hz,
            dll_bw_hz: if discriminator_feedback_ready { dll_bw } else { 0.0 },
            dll_err,
            samples_per_chip,
            samples_per_code,
        });
        state.carrier_hz = tracked_carrier_hz;
        state.carrier_phase_cycles = carrier_loop.carrier_phase_cycles;
        state.carrier_rate_hz_per_s = carrier_loop.carrier_rate_hz_per_s;
        state.code_rate_hz = code_loop.code_rate_hz;
        state.code_rate_reference_hz = code_rate_reference_hz;
        state.code_phase_samples = code_loop.code_phase_samples;
        if self.config.adaptive_tracking_enabled {
            let adaptation = advance_tracking_adaptation(
                Self::tracking_loop_profile(base_tracking_params),
                state.tracking_adaptation_state,
                SignalTrackingAdaptationInput {
                    cn0_dbhz,
                    fll_error_hz: fll_err_hz as f64,
                    carrier_rate_hz_per_s: carrier_loop.carrier_rate_hz_per_s,
                    carrier_lock_ready: raw_fll_lock || sustained_pll_lock,
                    steady_state_lock: matches!(
                        state.state,
                        ChannelState::Tracking | ChannelState::Degraded
                    ) && sustained_prompt_lock
                        && sustained_code_lock,
                    discriminator_stable: state.unstable_discriminator_epochs == 0
                        && !cycle_slip
                        && !anti_false_lock,
                },
            );
            state.tracking_adaptation_state = adaptation.state;
            state.tracking_loop_profile = adaptation.profile;
        }

        if state.state != from_state {
            transitions.push(TrackTransition {
                sat,
                channel_id,
                epoch_idx: track_epoch.epoch.index,
                sample_index: track_epoch.sample_index,
                from_state: from_state.to_string(),
                to_state: state.state.to_string(),
                reason: lock_state_reason.clone().unwrap_or_else(|| "state_transition".to_string()),
                lock_quality: epoch_lock_quality(
                    sustained_prompt_lock,
                    pll_lock,
                    dll_lock,
                    fll_lock,
                    cn0_dbhz,
                ),
            });
        }

        let measurement = VectorTrackingMeasurement {
            sat,
            channel_id,
            epoch_idx: track_epoch.epoch.index,
            sample_index: track_epoch.sample_index,
            cn0_dbhz,
            dll_error_samples: (dll_err as f64) * samples_per_chip,
            pll_error_rad: pll_err as f64,
            fll_error_hz: fll_err_hz as f64,
            code_rate_error_hz: state.code_rate_hz - signal_model.code_rate_hz,
            carrier_rate_hz_per_s: state.carrier_rate_hz_per_s,
            prompt_locked: channel_locked,
            dll_locked: dll_lock,
            pll_locked: pll_lock,
            fll_locked: fll_lock,
            channel_state: state.state,
        };
        let mut epoch_assumptions = tracking_assumptions(signal_model, tracking_params);
        if vector_application.is_some() {
            epoch_assumptions.aiding_mode = vector_tracking_aiding_mode_label(signal_model);
        }
        let vector_provenance = vector_application.map(vector_tracking_provenance);
        let mut tracking_provenance =
            vector_provenance.unwrap_or_else(|| track_epoch.tracking_provenance.clone());
        if let Some(subcarrier_ambiguity_provenance) =
            subcarrier_ambiguity_provenance(correlation.subcarrier_ambiguity_guard)
        {
            tracking_provenance.push_str(&subcarrier_ambiguity_provenance);
        }
        if signal_model.discriminator_family.requires_unambiguous_code_lock() {
            tracking_provenance.push_str(if state.subcarrier_code_phase_refined {
                " subcarrier_code_phase_handoff=refined"
            } else {
                " subcarrier_code_phase_handoff=coarse"
            });
        }
        tracking_provenance.push_str(&lock_detector_provenance(lock_detector_thresholds));
        tracking_provenance.push_str(&doppler_estimator_provenance(doppler_consistency));

        out.push(TrackEpoch {
            lock: channel_locked,
            carrier_hz: Hertz(state.carrier_hz),
            carrier_phase_cycles: Cycles(state.carrier_phase_cycles),
            code_rate_hz: Hertz(state.code_rate_hz),
            code_phase_samples: Chips(state.code_phase_samples),
            pll_lock,
            dll_lock,
            fll_lock,
            cycle_slip,
            nav_bit_lock,
            navigation_bit_sign,
            dll_err,
            pll_err,
            fll_err: fll_err_hz,
            anti_false_lock,
            cycle_slip_reason,
            lock_state,
            lock_state_reason,
            tracking_provenance,
            tracking_assumptions: Some(epoch_assumptions),
            signal_delay_alignment: state.signal_delay_alignment.clone(),
            tracking_uncertainty,
            ..track_epoch
        });
        Some(measurement)
    }
}

include!("tracking/navigation_annotation.rs");

include!("tracking/channel_state.rs");

include!("tracking/epoch_processing.rs");

include!("tracking/lock_evidence.rs");
include!("tracking/sample_rate_diagnostics.rs");
include!("tracking/reacquisition_annotations.rs");
include!("tracking/reacquisition_execution.rs");

#[cfg(test)]
#[path = "tracking/tests/mod.rs"]
mod tests;
