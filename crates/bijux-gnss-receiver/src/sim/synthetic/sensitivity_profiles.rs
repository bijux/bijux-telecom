/// Measure stable tracking lock probability for one synthetic signal profile.
pub fn measure_truth_guided_tracking_lock_probability(
    config: &ReceiverPipelineConfig,
    signal: SyntheticSignalParams,
    duration_s: f64,
    trial_seeds: &[u64],
    scenario_id_prefix: &str,
    seeded_doppler_error_hz: f64,
    seeded_code_phase_error_samples: isize,
    min_locked_epochs: usize,
) -> SyntheticTrackingSensitivityReport {
    let period_samples =
        samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length);

    let trials = trial_seeds
        .iter()
        .enumerate()
        .map(|(trial_index, seed)| {
            let scenario_id = format!("{scenario_id_prefix}_trial_{trial_index}");
            let scenario = SyntheticScenario {
                sample_rate_hz: config.sampling_freq_hz,
                intermediate_freq_hz: config.intermediate_freq_hz,
                receiver_clock_frequency_bias_hz: 0.0,
                duration_s,
                seed: *seed,
                satellites: vec![signal],
                ephemerides: Vec::new(),
                id: scenario_id.clone(),
            };
            let frame = generate_l1_ca_multi(config, &scenario);
            let expected_code_phase_samples =
                signal.code_phase_chips * config.sampling_freq_hz / config.code_freq_basis_hz;
            let seeded_code_phase_samples = wrap_seeded_code_phase_samples(
                expected_code_phase_samples.round() as isize + seeded_code_phase_error_samples,
                period_samples,
            );
            let tracking = crate::pipeline::tracking::Tracking::new(
                config.clone(),
                crate::engine::runtime::ReceiverRuntime::default(),
            );
            let tracks = tracking.track_from_acquisition(
                &frame,
                &[seeded_tracking_acquisition(
                    signal.sat,
                    signal.signal_band,
                    signal.doppler_hz + seeded_doppler_error_hz,
                    config.intermediate_freq_hz,
                    seeded_code_phase_samples,
                    signal.cn0_db_hz,
                    format!(
                        "synthetic_tracking_seed_doppler_error_{:+.3}_code_phase_error_{}",
                        seeded_doppler_error_hz, seeded_code_phase_error_samples
                    ),
                )],
            );
            let epochs = &tracks.first().expect("track").epochs;
            let stable_window =
                stable_tracking_window_bounds(epochs, min_locked_epochs).unwrap_or((0, 0));
            let last_epoch = epochs.last();

            SyntheticTrackingSensitivityTrial {
                scenario_id,
                seed: *seed,
                sat: signal.sat,
                stable_lock: stable_window.1 > 0,
                refused_lock: epochs.iter().any(|epoch| {
                    epoch.lock_state_reason.as_deref() == Some("cn0_below_tracking_lock_floor")
                }),
                first_lock_epoch_index: (stable_window.1 > 0).then_some(stable_window.0),
                locked_epoch_count: stable_window.1,
                final_lock_state: last_epoch
                    .map(|epoch| epoch.lock_state.clone())
                    .unwrap_or_else(|| "not_tracked".to_string()),
                final_lock_state_reason: last_epoch
                    .and_then(|epoch| epoch.lock_state_reason.clone()),
            }
        })
        .collect::<Vec<_>>();

    synthetic_tracking_sensitivity_report(
        scenario_id_prefix,
        signal.sat,
        signal.cn0_db_hz,
        duration_s,
        seeded_doppler_error_hz,
        seeded_code_phase_error_samples,
        min_locked_epochs,
        trials,
    )
}

/// Measure tracking lock rate across multiple C/N0 points.
pub fn measure_truth_guided_tracking_lock_rate(
    config: &ReceiverPipelineConfig,
    cases: &[SyntheticTrackingLockRateCase],
    trial_seeds: &[u64],
    scenario_id_prefix: &str,
) -> SyntheticTrackingLockRateReport {
    let points = cases
        .iter()
        .map(|case| {
            let sensitivity = measure_truth_guided_tracking_lock_probability(
                config,
                case.signal,
                case.duration_s,
                trial_seeds,
                &tracking_lock_rate_case_id(scenario_id_prefix, case),
                case.seeded_doppler_error_hz,
                case.seeded_code_phase_error_samples,
                case.min_locked_epochs,
            );

            SyntheticTrackingLockRatePoint {
                sat: case.signal.sat,
                cn0_db_hz: case.signal.cn0_db_hz,
                duration_s: case.duration_s,
                seeded_doppler_error_hz: case.seeded_doppler_error_hz,
                seeded_code_phase_error_samples: case.seeded_code_phase_error_samples,
                min_locked_epochs: case.min_locked_epochs,
                trial_count: sensitivity.trial_count,
                stable_lock_count: sensitivity.stable_lock_count,
                refused_lock_count: sensitivity.refused_lock_count,
                lock_probability: sensitivity.lock_probability,
                mean_locked_epochs: sensitivity.mean_locked_epochs,
            }
        })
        .collect::<Vec<_>>();

    SyntheticTrackingLockRateReport { scenario_id_prefix: scenario_id_prefix.to_string(), points }
}

/// Measure noise-only false-alarm rate for an acquisition integration profile.
pub fn measure_noise_only_acquisition_false_alarm_rate(
    config: &ReceiverPipelineConfig,
    sat: SatId,
    coherent_ms: u32,
    noncoherent: u32,
    trial_seeds: &[u64],
    scenario_id_prefix: &str,
) -> SyntheticAcquisitionFalseAlarmReport {
    let mut profile_config = config.clone();
    profile_config.acquisition_integration_ms = coherent_ms;
    profile_config.acquisition_noncoherent = noncoherent;
    let duration_s = (coherent_ms.saturating_mul(noncoherent).max(1) as f64) / 1000.0;

    let trials = trial_seeds
        .iter()
        .enumerate()
        .map(|(trial_index, seed)| {
            let scenario_id = format!("{scenario_id_prefix}_trial_{trial_index}");
            let scenario = SyntheticScenario {
                sample_rate_hz: profile_config.sampling_freq_hz,
                intermediate_freq_hz: profile_config.intermediate_freq_hz,
                receiver_clock_frequency_bias_hz: 0.0,
                duration_s,
                seed: *seed,
                satellites: Vec::new(),
                ephemerides: Vec::new(),
                id: scenario_id.clone(),
            };
            let frame = generate_l1_ca_multi(&profile_config, &scenario);
            let acquisition = crate::pipeline::acquisition::Acquisition::new(
                profile_config.clone(),
                crate::engine::runtime::ReceiverRuntime::default(),
            );
            let result = acquisition.run_fft(&frame, &[sat]).remove(0);
            let accepted = matches!(result.hypothesis, crate::api::core::AcqHypothesis::Accepted);

            SyntheticAcquisitionSensitivityTrial {
                scenario_id,
                seed: *seed,
                sat,
                hypothesis: result.hypothesis.to_string(),
                accepted,
                detected: false,
                code_phase_error_samples: None,
                doppler_error_bins: None,
                peak_mean_ratio: result.peak_mean_ratio,
            }
        })
        .collect::<Vec<_>>();
    let trial_count = trials.len();
    let false_alarm_count = trials.iter().filter(|trial| trial.accepted).count();
    let mean_peak_mean_ratio = if trial_count == 0 {
        0.0
    } else {
        trials.iter().map(|trial| trial.peak_mean_ratio as f64).sum::<f64>() / trial_count as f64
    };

    SyntheticAcquisitionFalseAlarmReport {
        scenario_id_prefix: scenario_id_prefix.to_string(),
        sat,
        coherent_ms,
        noncoherent,
        trial_count,
        false_alarm_count,
        false_alarm_rate: if trial_count == 0 {
            0.0
        } else {
            false_alarm_count as f64 / trial_count as f64
        },
        mean_peak_mean_ratio,
        trials,
    }
}

/// Measure noise-only false-alarm rate across multiple acquisition integration profiles.
pub fn measure_noise_only_acquisition_false_alarm_rates(
    config: &ReceiverPipelineConfig,
    cases: &[SyntheticAcquisitionFalseAlarmRateCase],
    trial_seeds: &[u64],
    scenario_id_prefix: &str,
) -> SyntheticAcquisitionFalseAlarmRateReport {
    let points = cases
        .iter()
        .map(|case| {
            let report = measure_noise_only_acquisition_false_alarm_rate(
                config,
                case.sat,
                case.coherent_ms,
                case.noncoherent,
                trial_seeds,
                &false_alarm_rate_case_id(scenario_id_prefix, case),
            );

            SyntheticAcquisitionFalseAlarmRatePoint {
                sat: case.sat,
                coherent_ms: case.coherent_ms,
                noncoherent: case.noncoherent,
                trial_count: report.trial_count,
                false_alarm_count: report.false_alarm_count,
                false_alarm_rate: report.false_alarm_rate,
                mean_peak_mean_ratio: report.mean_peak_mean_ratio,
            }
        })
        .collect::<Vec<_>>();

    SyntheticAcquisitionFalseAlarmRateReport {
        scenario_id_prefix: scenario_id_prefix.to_string(),
        acquisition_doppler_search_hz: config.acquisition_doppler_search_hz,
        acquisition_doppler_step_hz: config.acquisition_doppler_step_hz.max(1),
        points,
    }
}

fn synthetic_acquisition_sensitivity_report(
    scenario_id_prefix: &str,
    sat: SatId,
    cn0_db_hz: Option<f32>,
    coherent_ms: u32,
    noncoherent: u32,
    code_phase_tolerance_samples: usize,
    doppler_tolerance_bins: usize,
    doppler_step_hz: i32,
    trials: Vec<SyntheticAcquisitionSensitivityTrial>,
) -> SyntheticAcquisitionSensitivityReport {
    let trial_count = trials.len();
    let accepted_count = trials.iter().filter(|trial| trial.accepted).count();
    let detected_count = trials.iter().filter(|trial| trial.detected).count();
    let mean_peak_mean_ratio = if trial_count == 0 {
        0.0
    } else {
        trials.iter().map(|trial| trial.peak_mean_ratio as f64).sum::<f64>() / trial_count as f64
    };

    SyntheticAcquisitionSensitivityReport {
        scenario_id_prefix: scenario_id_prefix.to_string(),
        sat,
        cn0_db_hz: cn0_db_hz.unwrap_or_default(),
        coherent_ms,
        noncoherent,
        code_phase_tolerance_samples,
        doppler_tolerance_bins,
        doppler_step_hz,
        trial_count,
        accepted_count,
        detected_count,
        acceptance_probability: if trial_count == 0 {
            0.0
        } else {
            accepted_count as f64 / trial_count as f64
        },
        detection_probability: if trial_count == 0 {
            0.0
        } else {
            detected_count as f64 / trial_count as f64
        },
        mean_peak_mean_ratio,
        trials,
    }
}

fn synthetic_tracking_sensitivity_report(
    scenario_id_prefix: &str,
    sat: SatId,
    cn0_db_hz: f32,
    duration_s: f64,
    seeded_doppler_error_hz: f64,
    seeded_code_phase_error_samples: isize,
    min_locked_epochs: usize,
    trials: Vec<SyntheticTrackingSensitivityTrial>,
) -> SyntheticTrackingSensitivityReport {
    let trial_count = trials.len();
    let stable_lock_count = trials.iter().filter(|trial| trial.stable_lock).count();
    let refused_lock_count = trials.iter().filter(|trial| trial.refused_lock).count();
    let mean_locked_epochs = if trial_count == 0 {
        0.0
    } else {
        trials.iter().map(|trial| trial.locked_epoch_count as f64).sum::<f64>() / trial_count as f64
    };

    SyntheticTrackingSensitivityReport {
        scenario_id_prefix: scenario_id_prefix.to_string(),
        sat,
        cn0_db_hz,
        duration_s,
        seeded_doppler_error_hz,
        seeded_code_phase_error_samples,
        min_locked_epochs,
        trial_count,
        stable_lock_count,
        refused_lock_count,
        lock_probability: if trial_count == 0 {
            0.0
        } else {
            stable_lock_count as f64 / trial_count as f64
        },
        mean_locked_epochs,
        trials,
    }
}

fn detection_rate_case_id(
    scenario_id_prefix: &str,
    case: &SyntheticAcquisitionDetectionRateCase,
) -> String {
    format!(
        "{scenario_id_prefix}_prn_{}_cn0_{:03}_doppler_{:+05}_coherent_{}ms_noncoherent_{}",
        case.signal.sat.prn,
        (case.signal.cn0_db_hz * 10.0).round() as i32,
        case.signal.doppler_hz.round() as i32,
        case.coherent_ms,
        case.noncoherent,
    )
}

fn tracking_lock_rate_case_id(
    scenario_id_prefix: &str,
    case: &SyntheticTrackingLockRateCase,
) -> String {
    format!(
        "{scenario_id_prefix}_prn_{}_cn0_{:03}_doppler_error_{:+05}_code_error_{:+03}_duration_ms_{}",
        case.signal.sat.prn,
        (case.signal.cn0_db_hz * 10.0).round() as i32,
        case.seeded_doppler_error_hz.round() as i32,
        case.seeded_code_phase_error_samples,
        (case.duration_s * 1_000.0).round() as usize,
    )
}

fn false_alarm_rate_case_id(
    scenario_id_prefix: &str,
    case: &SyntheticAcquisitionFalseAlarmRateCase,
) -> String {
    format!(
        "{scenario_id_prefix}_prn_{}_coherent_{}ms_noncoherent_{}",
        case.sat.prn, case.coherent_ms, case.noncoherent,
    )
}

fn seeded_tracking_acquisition(
    sat: SatId,
    signal_band: SignalBand,
    doppler_hz: f64,
    intermediate_freq_hz: f64,
    code_phase_samples: usize,
    cn0_db_hz: f32,
    explain_selection_reason: String,
) -> AcqResult {
    AcqResult {
        sat,
        signal_band,
        glonass_frequency_channel: None,
        source_time: ReceiverSampleTrace::default(),
        candidate_rank: 1,
        is_primary_candidate: true,
        doppler_hz: Hertz(doppler_hz),
        carrier_hz: Hertz(carrier_hz_from_doppler_hz(intermediate_freq_hz, doppler_hz)),
        code_phase_samples,
        peak: 1.0,
        second_peak: 0.1,
        mean: 0.01,
        peak_mean_ratio: 20.0,
        peak_second_ratio: 10.0,
        cn0_proxy: cn0_db_hz,
        score: 1.0,
        hypothesis: AcqHypothesis::Accepted,
        assumptions: None,
        evidence: Vec::new(),
        threshold_provenance: None,
        explain_selection_reason: Some(explain_selection_reason),
        doppler_refinement: None,
        code_phase_refinement: None,
        signal_delay_alignment: None,
        uncertainty: None,
    }
}

fn seeded_tracking_acquisition_with_refined_code_phase(
    sat: SatId,
    signal_band: SignalBand,
    doppler_hz: f64,
    intermediate_freq_hz: f64,
    code_phase_samples: usize,
    refined_code_phase_samples: f64,
    cn0_db_hz: f32,
    explain_selection_reason: String,
) -> AcqResult {
    let mut result = seeded_tracking_acquisition(
        sat,
        signal_band,
        doppler_hz,
        intermediate_freq_hz,
        code_phase_samples,
        cn0_db_hz,
        explain_selection_reason,
    );
    result.code_phase_refinement = Some(bijux_gnss_core::api::AcqCodePhaseRefinement {
        method: "truth_guided_tracking_seed".to_string(),
        offset_samples: refined_code_phase_samples - code_phase_samples as f64,
        refined_code_phase_samples,
        left_correlation_norm: 0.0,
        center_correlation_norm: 1.0,
        right_correlation_norm: 0.0,
    });
    result
}

fn wrap_seeded_code_phase_samples(code_phase_samples: isize, period_samples: usize) -> usize {
    let period_samples = period_samples.max(1) as isize;
    code_phase_samples.rem_euclid(period_samples) as usize
}

fn stable_tracking_window_bounds(
    epochs: &[crate::api::core::TrackEpoch],
    min_locked_epochs: usize,
) -> Option<(usize, usize)> {
    if min_locked_epochs == 0 {
        return Some((0, epochs.len()));
    }

    let mut stable_start = None;
    for (index, epoch) in epochs.iter().enumerate() {
        let stable = tracking_epoch_is_stable(epoch);
        match (stable_start, stable) {
            (None, true) => stable_start = Some(index),
            (Some(start), false) => {
                if index - start >= min_locked_epochs {
                    return Some((start, index - start));
                }
                stable_start = None;
            }
            _ => {}
        }
    }

    if let Some(start) = stable_start {
        if epochs.len() - start >= min_locked_epochs {
            return Some((start, epochs.len() - start));
        }
    }

    None
}

/// Measure whether acquisition Doppler refinement improves on the raw search bin.
pub fn validate_truth_guided_acquisition_doppler_refinement(
    config: &ReceiverPipelineConfig,
    frame: &SamplesFrame,
    truth: &SyntheticIqTruthBundle,
) -> SyntheticAcquisitionDopplerRefinementReport {
    let doppler_step_hz = config.acquisition_doppler_step_hz.max(1);
    let satellites = truth
        .satellites
        .iter()
        .map(|sat_truth| {
            let isolated_frame = regenerate_isolated_scaled_satellite_signal_only_frame(
                config, frame, truth, sat_truth,
            );
            let acquisition = crate::pipeline::acquisition::Acquisition::new(
                config.clone(),
                crate::engine::runtime::ReceiverRuntime::default(),
            );
            let result = acquisition.run_fft(&isolated_frame, &[sat_truth.sat]).remove(0);
            let refined_doppler_hz = crate::pipeline::doppler::doppler_hz_from_carrier_hz(
                config.intermediate_freq_hz,
                result.carrier_hz.0,
            );
            let (coarse_doppler_hz, refinement_bins) = result
                .doppler_refinement
                .as_ref()
                .map(|refinement| {
                    (
                        crate::pipeline::doppler::doppler_hz_from_carrier_hz(
                            config.intermediate_freq_hz,
                            refinement.coarse_carrier_hz.0,
                        ),
                        refinement.offset_bins,
                    )
                })
                .unwrap_or((refined_doppler_hz, 0.0));
            let expected_measured_doppler_hz =
                synthetic_truth_measured_doppler_hz(truth, sat_truth);
            let coarse_error_hz = (coarse_doppler_hz - expected_measured_doppler_hz).abs();
            let refined_error_hz = (refined_doppler_hz - expected_measured_doppler_hz).abs();
            let improvement_hz = coarse_error_hz - refined_error_hz;
            let pass = result.doppler_refinement.is_some() && improvement_hz > f64::EPSILON;

            SyntheticAcquisitionDopplerRefinementSatellite {
                sat: sat_truth.sat,
                injected_doppler_hz: sat_truth.doppler_hz,
                coarse_doppler_hz,
                refined_doppler_hz,
                coarse_error_hz,
                refined_error_hz,
                improvement_hz,
                doppler_step_hz,
                refinement_bins,
                peak_mean_ratio: result.peak_mean_ratio,
                hypothesis: result.hypothesis.to_string(),
                pass,
            }
        })
        .collect::<Vec<_>>();
    let pass = !satellites.is_empty() && satellites.iter().all(|row| row.pass);

    SyntheticAcquisitionDopplerRefinementReport {
        scenario_id: truth.scenario_id.clone(),
        sample_rate_hz: truth.sample_rate_hz,
        doppler_step_hz,
        pass,
        satellites,
    }
}

/// Validate acquisition code phase and Doppler across multiple sample-rate profiles.
pub fn validate_truth_guided_acquisition_sample_rates(
    cases: &[SyntheticAcquisitionSampleRateValidationCase<'_>],
    code_phase_tolerance_samples: usize,
    doppler_tolerance_bins: usize,
) -> SyntheticAcquisitionSampleRateValidationReport {
    let mut points = cases
        .iter()
        .map(|case| {
            let code_phase_validation = validate_truth_guided_acquisition_code_phase(
                case.config,
                case.frame,
                case.truth,
                code_phase_tolerance_samples,
            );
            let doppler_validation = validate_truth_guided_acquisition_doppler(
                case.config,
                case.frame,
                case.truth,
                doppler_tolerance_bins,
            );
            let pass = code_phase_validation.pass && doppler_validation.pass;
            SyntheticAcquisitionSampleRateValidationPoint {
                scenario_id: case.truth.scenario_id.clone(),
                sample_rate_hz: case.truth.sample_rate_hz,
                code_phase_validation,
                doppler_validation,
                pass,
            }
        })
        .collect::<Vec<_>>();
    points.sort_by(|left, right| {
        left.sample_rate_hz
            .partial_cmp(&right.sample_rate_hz)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then_with(|| left.scenario_id.cmp(&right.scenario_id))
    });

    let distinct_sample_rate_count = points
        .iter()
        .map(|point| point.sample_rate_hz.to_bits())
        .collect::<std::collections::BTreeSet<_>>()
        .len();
    let min_sample_rate_hz = points.first().map(|point| point.sample_rate_hz).unwrap_or(0.0);
    let max_sample_rate_hz = points.last().map(|point| point.sample_rate_hz).unwrap_or(0.0);
    let doppler_tolerance_hz = points
        .first()
        .map(|point| point.doppler_validation.tolerance_hz)
        .unwrap_or(doppler_tolerance_bins as f64);
    let pass = distinct_sample_rate_count >= 2
        && !points.is_empty()
        && points.iter().all(|point| point.pass);

    SyntheticAcquisitionSampleRateValidationReport {
        code_phase_tolerance_samples,
        doppler_tolerance_bins,
        doppler_tolerance_hz,
        distinct_sample_rate_count,
        min_sample_rate_hz,
        max_sample_rate_hz,
        pass,
        points,
    }
}
