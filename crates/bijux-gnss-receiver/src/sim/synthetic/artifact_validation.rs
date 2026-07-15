fn summarize_observation_errors(errors: &[f64]) -> Option<SyntheticObservationErrorStats> {
    if errors.is_empty() {
        return None;
    }

    let signed = stats(errors);
    let absolute_errors = errors.iter().map(|error| error.abs()).collect::<Vec<_>>();
    let absolute = stats(&absolute_errors);

    Some(SyntheticObservationErrorStats {
        count: signed.count,
        mean_error: signed.mean,
        median_abs_error: absolute.median,
        rms_error: signed.rms,
        p95_abs_error: absolute.p95,
        max_abs_error: absolute.max,
    })
}

fn mean_f64(values: &[f64]) -> Option<f64> {
    if values.is_empty() {
        None
    } else {
        Some(values.iter().sum::<f64>() / values.len() as f64)
    }
}

fn time_profile_window_len(epoch_count: usize) -> usize {
    (epoch_count / 2).clamp(1, 25)
}

const MIN_TIME_PROFILE_EPOCH_COUNT: usize = 5;
const MIN_TIME_PROFILE_DURATION_S: f64 = 1.0;

fn linear_trend_slope(receive_times_s: &[f64], values: &[f64]) -> Option<f64> {
    if receive_times_s.len() != values.len() || values.len() < 2 {
        return None;
    }
    let mean_time_s = mean_f64(receive_times_s)?;
    let mean_value = mean_f64(values)?;
    let (numerator, denominator) =
        receive_times_s.iter().zip(values.iter()).fold((0.0, 0.0), |acc, (time_s, value)| {
            let centered_time_s = time_s - mean_time_s;
            (
                acc.0 + centered_time_s * (value - mean_value),
                acc.1 + centered_time_s * centered_time_s,
            )
        });
    (denominator > f64::EPSILON).then_some(numerator / denominator)
}

fn classify_pvt_time_trend(
    diverging_epoch_count: usize,
    first_window_mean_position_error_3d_m: Option<f64>,
    last_window_mean_position_error_3d_m: Option<f64>,
    position_error_drift_m_per_s: Option<f64>,
    first_window_mean_residual_rms_m: Option<f64>,
    last_window_mean_residual_rms_m: Option<f64>,
    residual_rms_drift_m_per_s: Option<f64>,
) -> SyntheticPvtTimeTrend {
    if diverging_epoch_count > 0 {
        return SyntheticPvtTimeTrend::Diverging;
    }

    let position_growth_m = first_window_mean_position_error_3d_m
        .zip(last_window_mean_position_error_3d_m)
        .map(|(first, last)| last - first)
        .unwrap_or(0.0);
    let residual_growth_m = first_window_mean_residual_rms_m
        .zip(last_window_mean_residual_rms_m)
        .map(|(first, last)| last - first)
        .unwrap_or(0.0);
    let position_slope_m_per_s = position_error_drift_m_per_s.unwrap_or(0.0);
    let residual_slope_m_per_s = residual_rms_drift_m_per_s.unwrap_or(0.0);

    if position_growth_m > 5.0
        || residual_growth_m > 5.0
        || position_slope_m_per_s > 1.0
        || residual_slope_m_per_s > 1.0
    {
        SyntheticPvtTimeTrend::Diverging
    } else if position_growth_m > 0.5
        || position_slope_m_per_s > 0.1
        || ((position_growth_m > 0.0 || position_slope_m_per_s >= 0.0)
            && (residual_growth_m > 0.5 || residual_slope_m_per_s > 0.1))
    {
        SyntheticPvtTimeTrend::Drifting
    } else {
        SyntheticPvtTimeTrend::Stabilizing
    }
}

fn truth_coverage_issue(
    sat: Option<SatId>,
    epoch_index: Option<u64>,
    code: impl Into<String>,
) -> SyntheticTruthCoverageIssue {
    SyntheticTruthCoverageIssue { sat, epoch_index, code: code.into() }
}

fn tracked_signal_band(track: &crate::pipeline::tracking::TrackingResult) -> Option<SignalBand> {
    track.epochs.first().map(|epoch| epoch.signal_band)
}

fn expected_signal_id(sat: SatId, signal_band: Option<SignalBand>) -> SigId {
    let code = match (sat.constellation, signal_band.unwrap_or(SignalBand::Unknown)) {
        (Constellation::Gps, SignalBand::L1) => SignalCode::Ca,
        (Constellation::Gps, SignalBand::L2) => SignalCode::L2C,
        (Constellation::Gps, SignalBand::L5) => SignalCode::L5I,
        (Constellation::Galileo, SignalBand::E1) => SignalCode::E1B,
        (Constellation::Beidou, SignalBand::B1) => SignalCode::B1I,
        (Constellation::Glonass, SignalBand::L1) => SignalCode::Unknown,
        _ => SignalCode::Unknown,
    };
    SigId { sat, band: signal_band.unwrap_or(SignalBand::Unknown), code }
}

fn acquisition_truth_coverage_issues(
    report: &SyntheticAcquisitionTruthTableReport,
) -> Vec<SyntheticTruthCoverageIssue> {
    let mut issues = Vec::new();
    if report.satellites.is_empty() {
        issues.push(truth_coverage_issue(None, None, "no_truth_satellites"));
    }
    for satellite in &report.satellites {
        if !satellite.expected_measured_doppler_hz.is_finite() {
            issues.push(truth_coverage_issue(
                Some(satellite.sat),
                None,
                "non_finite_expected_doppler_truth",
            ));
        }
        if !satellite.measured_doppler_hz.is_finite() {
            issues.push(truth_coverage_issue(
                Some(satellite.sat),
                None,
                "non_finite_measured_doppler",
            ));
        }
    }
    issues
}

fn tracking_truth_coverage_issues(
    report: &SyntheticTrackingTruthTableReport,
) -> Vec<SyntheticTruthCoverageIssue> {
    let mut issues = Vec::new();
    if report.satellites.is_empty() {
        issues.push(truth_coverage_issue(None, None, "no_truth_satellites"));
    }
    for satellite in &report.satellites {
        if satellite.epoch_count == 0 {
            issues.push(truth_coverage_issue(Some(satellite.sat), None, "no_tracking_epochs"));
        }
        if satellite.stable_epoch_count == 0 {
            issues.push(truth_coverage_issue(
                Some(satellite.sat),
                None,
                "no_stable_tracking_truth_epochs",
            ));
        }
    }
    issues
}

fn observation_truth_coverage_issues(
    report: &SyntheticObservationValidationReport,
) -> Vec<SyntheticTruthCoverageIssue> {
    let mut issues = Vec::new();
    if report.satellites.is_empty() {
        issues.push(truth_coverage_issue(None, None, "no_truth_satellites"));
    }
    for satellite in &report.satellites {
        if satellite.pseudorange_error_m.is_none() {
            issues.push(truth_coverage_issue(
                Some(satellite.sat),
                None,
                "missing_pseudorange_truth",
            ));
        }
        if satellite.carrier_phase_error_cycles.is_none() {
            issues.push(truth_coverage_issue(
                Some(satellite.sat),
                None,
                "missing_carrier_phase_truth",
            ));
        }
        if satellite.doppler_error_hz.is_none() {
            issues.push(truth_coverage_issue(Some(satellite.sat), None, "missing_doppler_truth"));
        }
        if satellite.cn0_error_db_hz.is_none() {
            issues.push(truth_coverage_issue(Some(satellite.sat), None, "missing_cn0_truth"));
        }
        for note in &satellite.notes {
            issues.push(truth_coverage_issue(Some(satellite.sat), None, note.clone()));
        }
    }
    issues
}

fn pvt_truth_coverage_issues(
    report: &SyntheticPvtTruthTableReport,
) -> Vec<SyntheticTruthCoverageIssue> {
    let mut issues = Vec::new();
    if report.solution_count == 0 {
        issues.push(truth_coverage_issue(None, None, "no_navigation_solutions"));
    }
    if report.matched_epoch_count == 0 {
        issues.push(truth_coverage_issue(None, None, "no_matched_truth_epochs"));
    }
    for epoch_index in &report.unmatched_solution_epochs {
        issues.push(truth_coverage_issue(None, Some(*epoch_index), "unmatched_solution_epoch"));
    }
    for epoch_index in &report.unused_reference_epochs {
        issues.push(truth_coverage_issue(None, Some(*epoch_index), "unused_truth_reference_epoch"));
    }
    issues
}

struct ObservedSatelliteRow<'a> {
    artifact_id: String,
    epoch_id: String,
    epoch_index: u64,
    sample_index: u64,
    observation: &'a bijux_gnss_core::api::ObsSatellite,
}

/// Build a truth-guided observation table from synthetic tracking inputs.
pub fn validate_truth_guided_observation_table(
    config: &ReceiverPipelineConfig,
    tracks: &[crate::pipeline::tracking::TrackingResult],
    truth: &SyntheticScenario,
    reference: &SyntheticObservationTruthReference,
    hatch_window: u32,
) -> SyntheticObservationTruthTableReport {
    let observations = crate::pipeline::observations::observation_artifacts_from_tracking_results(
        config,
        tracks,
        hatch_window,
    )
    .output
    .epochs;
    let satellites = truth
        .satellites
        .iter()
        .map(|sat_truth| {
            let matching_track = tracks.iter().find(|track| track.sat == sat_truth.sat);
            let expected_band = matching_track.and_then(tracked_signal_band);
            let signal_id = expected_band
                .map(|band| expected_signal_id(sat_truth.sat, Some(band)))
                .unwrap_or_else(|| expected_signal_id(sat_truth.sat, None));
            let observed_rows =
                comparable_signal_band_observations(&observations, sat_truth.sat, expected_band);
            let expected_doppler_hz = sat_truth.doppler_hz + truth.receiver_clock_frequency_bias_hz;
            let mut notes = Vec::new();
            if matching_track.is_none() {
                notes.push("missing_tracking_result_for_truth_signal".to_string());
            }
            if observed_rows.is_empty() {
                notes.push("no_comparable_observation_rows".to_string());
            }
            let ephemeris =
                truth.ephemerides.iter().find(|candidate| candidate.sat == sat_truth.sat);
            if ephemeris.is_none() {
                notes.push("missing_ephemeris_for_pseudorange_truth".to_string());
            } else if !observed_rows.iter().any(|row| {
                row.observation.metadata.pseudorange_model == "tracked_code_phase_alignment"
            }) {
                notes.push("no_absolute_pseudorange_rows".to_string());
            }
            let sat_state = SatState::new_with_receiver_clock_frequency_bias_hz(
                config,
                sat_truth.clone(),
                truth.receiver_clock_frequency_bias_hz,
            );
            let carrier_phase_arc_bias_cycles =
                carrier_phase_arc_bias_cycles_by_start_sample(config, &sat_state, &observed_rows);
            if carrier_phase_arc_bias_cycles.is_empty() {
                notes.push("no_usable_carrier_phase_rows".to_string());
            }
            let epochs = observed_rows
                .iter()
                .map(|row| {
                    let pseudorange_truth_m = ephemeris.and_then(|ephemeris| {
                        (row.observation.metadata.pseudorange_model
                            == "tracked_code_phase_alignment")
                            .then(|| {
                                let receive_time_s = reference.receive_time_s
                                    + row.sample_index as f64 / config.sampling_freq_hz;
                                let geometric_pseudorange_m = synthetic_pseudorange_m(
                                    ephemeris,
                                    receive_time_s,
                                    reference.receiver_ecef_m,
                                );
                                let mut pseudorange_m = reference
                                    .ionosphere_delay_model
                                    .map(|model| {
                                        model.pseudorange_m(
                                            geometric_pseudorange_m,
                                            row.observation.metadata.signal,
                                        )
                                    })
                                    .unwrap_or(Some(geometric_pseudorange_m))?;
                                if reference.troposphere_delay_model
                                    == Some(SyntheticTroposphereDelayModel::Saastamoinen)
                                {
                                    pseudorange_m += synthetic_saastamoinen_delay_m(
                                        ephemeris,
                                        receive_time_s,
                                        reference.receiver_ecef_m,
                                    );
                                }
                                Some(pseudorange_m)
                            })
                            .flatten()
                    });
                    let carrier_phase_truth_cycles = Some(
                        sat_state.carrier_phase_rad_at(
                            row.sample_index as f64 / config.sampling_freq_hz,
                        ) / std::f64::consts::TAU,
                    );
                    let carrier_phase_arc_start_sample_index =
                        carrier_phase_arc_start_sample_index(row.observation);
                    let carrier_phase_arc_bias =
                        carrier_phase_arc_start_sample_index.and_then(|arc_start| {
                            carrier_phase_arc_bias_cycles.get(&arc_start).copied()
                        });

                    SyntheticObservationTruthTableEpoch {
                        artifact_id: row.artifact_id.clone(),
                        epoch_id: row.epoch_id.clone(),
                        epoch_index: row.epoch_index,
                        sample_index: row.sample_index,
                        observation_status: row.observation.observation_status,
                        observation_reject_reasons: row
                            .observation
                            .observation_reject_reasons
                            .clone(),
                        pseudorange_m: SyntheticObservationTruthTableValue {
                            truth: pseudorange_truth_m,
                            measured: row.observation.pseudorange_m.0,
                            sigma: Some(row.observation.pseudorange_var_m2.sqrt()),
                            residual: pseudorange_truth_m
                                .map(|truth_m| row.observation.pseudorange_m.0 - truth_m),
                        },
                        carrier_phase_cycles: SyntheticObservationTruthTableValue {
                            truth: carrier_phase_truth_cycles,
                            measured: row.observation.carrier_phase_cycles.0,
                            sigma: Some(row.observation.carrier_phase_var_cycles2.sqrt()),
                            residual: carrier_phase_truth_cycles.zip(carrier_phase_arc_bias).map(
                                |(truth_cycles, arc_bias_cycles)| {
                                    row.observation.carrier_phase_cycles.0
                                        - truth_cycles
                                        - arc_bias_cycles
                                },
                            ),
                        },
                        carrier_phase_arc_start_sample_index,
                        carrier_phase_arc_bias_cycles: carrier_phase_arc_bias,
                        doppler_hz: SyntheticObservationTruthTableValue {
                            truth: Some(expected_doppler_hz),
                            measured: row.observation.doppler_hz.0,
                            sigma: Some(row.observation.doppler_var_hz2.sqrt()),
                            residual: Some(row.observation.doppler_hz.0 - expected_doppler_hz),
                        },
                        cn0_db_hz: SyntheticObservationTruthTableValue {
                            truth: Some(sat_truth.cn0_db_hz as f64),
                            measured: row.observation.cn0_dbhz,
                            sigma: row
                                .observation
                                .metadata
                                .tracking_uncertainty
                                .as_ref()
                                .map(|uncertainty| uncertainty.cn0_dbhz),
                            residual: Some(row.observation.cn0_dbhz - sat_truth.cn0_db_hz as f64),
                        },
                    }
                })
                .collect::<Vec<_>>();

            SyntheticObservationTruthTableSatellite {
                sat: sat_truth.sat,
                signal_id: observed_rows
                    .first()
                    .map(|row| row.observation.signal_id)
                    .unwrap_or(signal_id),
                injected_doppler_hz: sat_truth.doppler_hz,
                expected_measured_doppler_hz: expected_doppler_hz,
                injected_code_phase_chips: sat_truth.code_phase_chips,
                injected_cn0_db_hz: sat_truth.cn0_db_hz,
                epoch_count: epochs.len(),
                carrier_phase_arcs_evaluated: carrier_phase_arc_bias_cycles.len(),
                notes,
                epochs,
            }
        })
        .collect();

    SyntheticObservationTruthTableReport {
        scenario_id: truth.id.clone(),
        sample_rate_hz: truth.sample_rate_hz,
        hatch_window,
        reference_receive_time_s: reference.receive_time_s,
        satellites,
    }
}

/// Measure emitted observation values against synthetic truth on a per-satellite basis.
pub fn validate_truth_guided_observations(
    config: &ReceiverPipelineConfig,
    tracks: &[crate::pipeline::tracking::TrackingResult],
    truth: &SyntheticScenario,
    reference: &SyntheticObservationTruthReference,
    hatch_window: u32,
) -> SyntheticObservationValidationReport {
    let table =
        validate_truth_guided_observation_table(config, tracks, truth, reference, hatch_window);
    let satellites = table
        .satellites
        .iter()
        .map(|satellite| SyntheticObservationValidationSatellite {
            sat: satellite.sat,
            signal_id: satellite.signal_id,
            pseudorange_error_m: summarize_observation_errors(
                &satellite
                    .epochs
                    .iter()
                    .filter_map(|epoch| epoch.pseudorange_m.residual)
                    .collect::<Vec<_>>(),
            ),
            carrier_phase_error_cycles: summarize_observation_errors(
                &satellite
                    .epochs
                    .iter()
                    .filter_map(|epoch| epoch.carrier_phase_cycles.residual)
                    .collect::<Vec<_>>(),
            ),
            carrier_phase_arcs_evaluated: satellite.carrier_phase_arcs_evaluated,
            doppler_error_hz: summarize_observation_errors(
                &satellite
                    .epochs
                    .iter()
                    .filter_map(|epoch| epoch.doppler_hz.residual)
                    .collect::<Vec<_>>(),
            ),
            cn0_error_db_hz: summarize_observation_errors(
                &satellite
                    .epochs
                    .iter()
                    .filter_map(|epoch| epoch.cn0_db_hz.residual)
                    .collect::<Vec<_>>(),
            ),
            notes: satellite.notes.clone(),
        })
        .collect();

    SyntheticObservationValidationReport {
        scenario_id: table.scenario_id,
        sample_rate_hz: table.sample_rate_hz,
        hatch_window: table.hatch_window,
        reference_receive_time_s: table.reference_receive_time_s,
        satellites,
    }
}

/// Return the hard truth-guided receiver accuracy budgets used in synthetic validation.
pub fn truth_guided_receiver_accuracy_budgets() -> SyntheticReceiverAccuracyBudgets {
    SyntheticReceiverAccuracyBudgets {
        acquisition: SyntheticAcquisitionAccuracyBudget {
            max_doppler_error_hz: 500.0,
            max_code_phase_error_samples: 2,
            max_doppler_error_bins: 1.0,
            max_code_phase_error_chips: 1.0,
        },
        tracking: SyntheticTrackingAccuracyBudget {
            max_carrier_error_hz: 30.0,
            max_doppler_error_hz: 30.0,
            max_code_phase_error_samples: 1.0,
            max_cn0_error_db_hz: 10.0,
        },
        observation: SyntheticObservationAccuracyBudget {
            max_pseudorange_error_m: 5.0e-2,
            max_carrier_phase_error_cycles: 1.0e-6,
            max_doppler_error_hz: 1.0e-6,
            max_cn0_error_db_hz: 1.0e-6,
        },
        pvt: SyntheticPvtAccuracyBudget {
            max_position_error_3d_m: 5.0,
            max_clock_bias_error_m: 0.5,
            max_residual_rms_m: 1.0,
            max_pdop: 3.0,
        },
    }
}

#[derive(Debug, Clone, Copy)]
struct ResolvedSyntheticAcquisitionAccuracyBudget {
    max_doppler_error_hz: f64,
    max_doppler_error_bins: f64,
    max_code_phase_error_samples: usize,
    max_code_phase_error_chips: f64,
}

fn resolve_acquisition_accuracy_budget(
    report: &SyntheticAcquisitionTruthTableReport,
    satellite: &SyntheticAcquisitionTruthTableSatellite,
    budget: SyntheticAcquisitionAccuracyBudget,
) -> ResolvedSyntheticAcquisitionAccuracyBudget {
    let max_doppler_error_hz = if report.doppler_step_hz > 0 {
        budget.max_doppler_error_bins * f64::from(report.doppler_step_hz)
    } else {
        budget.max_doppler_error_hz
    };
    let max_code_phase_error_chips = bijux_gnss_signal::api::signal_registry(
        satellite.sat.constellation,
        satellite.signal_band,
        satellite.signal_code,
    )
    .map(|entry| acquisition_code_phase_budget_chips_for_signal(&entry))
    .unwrap_or(budget.max_code_phase_error_chips);
    let max_code_phase_error_samples =
        resolve_code_phase_budget_samples(report.sample_rate_hz, satellite, max_code_phase_error_chips)
            .unwrap_or(budget.max_code_phase_error_samples);

    ResolvedSyntheticAcquisitionAccuracyBudget {
        max_doppler_error_hz,
        max_doppler_error_bins: budget.max_doppler_error_bins,
        max_code_phase_error_samples,
        max_code_phase_error_chips,
    }
}

fn acquisition_code_phase_budget_chips_for_signal(
    entry: &bijux_gnss_core::api::SignalRegistryEntry,
) -> f64 {
    if entry
        .components
        .iter()
        .any(|component| {
            matches!(
                component.subcarrier,
                bijux_gnss_core::api::SignalSubcarrierSpec::Cboc { .. }
            )
        })
    {
        0.25
    } else if entry
        .components
        .iter()
        .any(|component| {
            matches!(
                component.subcarrier,
                bijux_gnss_core::api::SignalSubcarrierSpec::Boc { .. }
            )
        })
    {
        0.5
    } else {
        1.0
    }
}

fn resolve_code_phase_budget_samples(
    sample_rate_hz: f64,
    satellite: &SyntheticAcquisitionTruthTableSatellite,
    max_code_phase_error_chips: f64,
) -> Option<usize> {
    let signal = bijux_gnss_signal::api::signal_registry(
        satellite.sat.constellation,
        satellite.signal_band,
        satellite.signal_code,
    )?
    .spec;
    if !sample_rate_hz.is_finite()
        || sample_rate_hz <= 0.0
        || !signal.code_rate_hz.is_finite()
        || signal.code_rate_hz <= 0.0
        || !max_code_phase_error_chips.is_finite()
        || max_code_phase_error_chips <= 0.0
    {
        return None;
    }

    Some(((sample_rate_hz / signal.code_rate_hz) * max_code_phase_error_chips).ceil() as usize)
}

/// Evaluate whether a truth-guided acquisition report satisfies a hard accuracy budget.
pub fn validate_acquisition_accuracy_budget(
    report: &SyntheticAcquisitionTruthTableReport,
    budget: SyntheticAcquisitionAccuracyBudget,
) -> SyntheticAcquisitionAccuracyReport {
    let truth_coverage_issues = acquisition_truth_coverage_issues(report);
    let truth_coverage_ready = truth_coverage_issues.is_empty();
    let satellites = report
        .satellites
        .iter()
        .map(|satellite| {
            let resolved_budget = resolve_acquisition_accuracy_budget(report, satellite, budget);
            let pass = satellite.doppler_error_hz.is_finite()
                && satellite.code_phase_error_samples <= resolved_budget.max_code_phase_error_samples
                && satellite.doppler_error_hz <= resolved_budget.max_doppler_error_hz + f64::EPSILON;
            SyntheticAcquisitionAccuracySatellite {
                sat: satellite.sat,
                glonass_frequency_channel: satellite.glonass_frequency_channel,
                signal_band: satellite.signal_band,
                signal_code: satellite.signal_code,
                doppler_error_hz: satellite.doppler_error_hz,
                code_phase_error_samples: satellite.code_phase_error_samples,
                max_doppler_error_hz: resolved_budget.max_doppler_error_hz,
                max_doppler_error_bins: resolved_budget.max_doppler_error_bins,
                max_code_phase_error_samples: resolved_budget.max_code_phase_error_samples,
                max_code_phase_error_chips: resolved_budget.max_code_phase_error_chips,
                pass,
            }
        })
        .collect::<Vec<_>>();
    let passing_satellite_count = satellites.iter().filter(|satellite| satellite.pass).count();
    let max_doppler_error_hz = satellites
        .iter()
        .map(|satellite| satellite.max_doppler_error_hz)
        .max_by(f64::total_cmp)
        .unwrap_or(budget.max_doppler_error_hz);
    let max_code_phase_error_samples = satellites
        .iter()
        .map(|satellite| satellite.max_code_phase_error_samples)
        .max()
        .unwrap_or(budget.max_code_phase_error_samples);

    SyntheticAcquisitionAccuracyReport {
        scenario_id: report.scenario_id.clone(),
        max_doppler_error_hz,
        max_code_phase_error_samples,
        satellite_count: satellites.len(),
        passing_satellite_count,
        truth_coverage_ready,
        truth_coverage_issues,
        pass: truth_coverage_ready
            && !satellites.is_empty()
            && passing_satellite_count == satellites.len(),
        satellites,
    }
}

/// Evaluate whether a truth-guided tracking report satisfies a hard accuracy budget.
pub fn validate_tracking_accuracy_budget(
    report: &SyntheticTrackingTruthTableReport,
    budget: SyntheticTrackingAccuracyBudget,
) -> SyntheticTrackingAccuracyReport {
    let truth_coverage_issues = tracking_truth_coverage_issues(report);
    let truth_coverage_ready = truth_coverage_issues.is_empty();
    let satellites = report
        .satellites
        .iter()
        .map(|satellite| {
            let stable_epochs = satellite
                .epochs
                .iter()
                .filter(|epoch| epoch.stable_tracking_epoch)
                .collect::<Vec<_>>();
            let max_carrier_error_hz =
                stable_epochs.iter().map(|epoch| epoch.carrier_error_hz).fold(0.0_f64, f64::max);
            let max_doppler_error_hz =
                stable_epochs.iter().map(|epoch| epoch.doppler_error_hz).fold(0.0_f64, f64::max);
            let max_code_phase_error_samples = stable_epochs
                .iter()
                .map(|epoch| epoch.code_phase_error_samples)
                .fold(0.0_f64, f64::max);
            let max_cn0_error_db_hz =
                stable_epochs.iter().map(|epoch| epoch.cn0_error_db).fold(0.0_f64, f64::max);
            let pass = !stable_epochs.is_empty()
                && max_carrier_error_hz <= budget.max_carrier_error_hz + f64::EPSILON
                && max_doppler_error_hz <= budget.max_doppler_error_hz + f64::EPSILON
                && max_code_phase_error_samples
                    <= budget.max_code_phase_error_samples + f64::EPSILON
                && max_cn0_error_db_hz <= budget.max_cn0_error_db_hz + f64::EPSILON;

            SyntheticTrackingAccuracySatellite {
                sat: satellite.sat,
                stable_epoch_count: stable_epochs.len(),
                max_carrier_error_hz,
                max_doppler_error_hz,
                max_code_phase_error_samples,
                max_cn0_error_db_hz,
                pass,
            }
        })
        .collect::<Vec<_>>();
    let passing_satellite_count = satellites.iter().filter(|satellite| satellite.pass).count();

    SyntheticTrackingAccuracyReport {
        scenario_id: report.scenario_id.clone(),
        max_carrier_error_hz: budget.max_carrier_error_hz,
        max_doppler_error_hz: budget.max_doppler_error_hz,
        max_code_phase_error_samples: budget.max_code_phase_error_samples,
        max_cn0_error_db_hz: budget.max_cn0_error_db_hz,
        satellite_count: satellites.len(),
        passing_satellite_count,
        truth_coverage_ready,
        truth_coverage_issues,
        pass: truth_coverage_ready
            && !satellites.is_empty()
            && passing_satellite_count == satellites.len(),
        satellites,
    }
}

/// Evaluate whether a truth-guided observation report satisfies a hard accuracy budget.
pub fn validate_observation_accuracy_budget(
    report: &SyntheticObservationValidationReport,
    budget: SyntheticObservationAccuracyBudget,
) -> SyntheticObservationAccuracyReport {
    let truth_coverage_issues = observation_truth_coverage_issues(report);
    let truth_coverage_ready = truth_coverage_issues.is_empty();
    let satellites = report
        .satellites
        .iter()
        .map(|satellite| {
            let max_pseudorange_error_m =
                satellite.pseudorange_error_m.as_ref().map(|stats| stats.max_abs_error);
            let max_carrier_phase_error_cycles =
                satellite.carrier_phase_error_cycles.as_ref().map(|stats| stats.max_abs_error);
            let max_doppler_error_hz =
                satellite.doppler_error_hz.as_ref().map(|stats| stats.max_abs_error);
            let max_cn0_error_db_hz =
                satellite.cn0_error_db_hz.as_ref().map(|stats| stats.max_abs_error);
            let pass = max_pseudorange_error_m
                .zip(max_carrier_phase_error_cycles)
                .zip(max_doppler_error_hz.zip(max_cn0_error_db_hz))
                .map(
                    |(
                        (pseudorange_error_m, carrier_phase_error_cycles),
                        (doppler_error_hz, cn0_error_db_hz),
                    )| {
                        pseudorange_error_m <= budget.max_pseudorange_error_m + f64::EPSILON
                            && carrier_phase_error_cycles
                                <= budget.max_carrier_phase_error_cycles + f64::EPSILON
                            && doppler_error_hz <= budget.max_doppler_error_hz + f64::EPSILON
                            && cn0_error_db_hz <= budget.max_cn0_error_db_hz + f64::EPSILON
                    },
                )
                .unwrap_or(false);

            SyntheticObservationAccuracySatellite {
                sat: satellite.sat,
                max_pseudorange_error_m,
                max_carrier_phase_error_cycles,
                max_doppler_error_hz,
                max_cn0_error_db_hz,
                pass,
            }
        })
        .collect::<Vec<_>>();
    let passing_satellite_count = satellites.iter().filter(|satellite| satellite.pass).count();

    SyntheticObservationAccuracyReport {
        scenario_id: report.scenario_id.clone(),
        max_pseudorange_error_m: budget.max_pseudorange_error_m,
        max_carrier_phase_error_cycles: budget.max_carrier_phase_error_cycles,
        max_doppler_error_hz: budget.max_doppler_error_hz,
        max_cn0_error_db_hz: budget.max_cn0_error_db_hz,
        satellite_count: satellites.len(),
        passing_satellite_count,
        truth_coverage_ready,
        truth_coverage_issues,
        pass: truth_coverage_ready
            && !satellites.is_empty()
            && passing_satellite_count == satellites.len(),
        satellites,
    }
}

/// Evaluate whether a truth-guided PVT report satisfies a hard accuracy budget.
pub fn validate_pvt_covariance_realism(
    report: &SyntheticPvtTruthTableReport,
) -> crate::covariance_realism::CovarianceRealismReport {
    let samples = report
        .epochs
        .iter()
        .map(|epoch| crate::covariance_realism::CovarianceRealismEpochSample {
            receiver_ecef_m: [
                epoch.measured_ecef_m.x_m,
                epoch.measured_ecef_m.y_m,
                epoch.measured_ecef_m.z_m,
            ],
            east_m: epoch.enu_error_m.east_m,
            north_m: epoch.enu_error_m.north_m,
            up_m: epoch.enu_error_m.up_m,
            position_covariance_ecef_m2: epoch.position_covariance_ecef_m2,
        })
        .collect::<Vec<_>>();

    crate::covariance_realism::evaluate_covariance_realism(&samples)
}

/// Evaluate whether a truth-guided PVT report satisfies a hard accuracy budget.
pub fn validate_pvt_accuracy_budget(
    report: &SyntheticPvtTruthTableReport,
    budget: SyntheticPvtAccuracyBudget,
) -> SyntheticPvtAccuracyReport {
    let truth_coverage_issues = pvt_truth_coverage_issues(report);
    let truth_coverage_ready = truth_coverage_issues.is_empty();
    let epochs = report
        .epochs
        .iter()
        .map(|epoch| {
            let clock_bias_error_m = epoch.clock_bias.error_m.abs();
            let pass = epoch.enu_error_m.error_3d_m
                <= budget.max_position_error_3d_m + f64::EPSILON
                && clock_bias_error_m <= budget.max_clock_bias_error_m + f64::EPSILON
                && epoch.residual_rms_m <= budget.max_residual_rms_m + f64::EPSILON
                && epoch.dop.pdop <= budget.max_pdop + f64::EPSILON
                && epoch.valid;
            SyntheticPvtAccuracyEpoch {
                epoch_index: epoch.epoch_index,
                position_error_3d_m: epoch.enu_error_m.error_3d_m,
                clock_bias_error_m,
                residual_rms_m: epoch.residual_rms_m,
                pdop: epoch.dop.pdop,
                pass,
            }
        })
        .collect::<Vec<_>>();
    let passing_epoch_count = epochs.iter().filter(|epoch| epoch.pass).count();

    SyntheticPvtAccuracyReport {
        scenario_id: report.scenario_id.clone(),
        max_position_error_3d_m: budget.max_position_error_3d_m,
        max_clock_bias_error_m: budget.max_clock_bias_error_m,
        max_residual_rms_m: budget.max_residual_rms_m,
        max_pdop: budget.max_pdop,
        epoch_count: epochs.len(),
        passing_epoch_count,
        truth_coverage_ready,
        truth_coverage_issues,
        pass: truth_coverage_ready && !epochs.is_empty() && passing_epoch_count == epochs.len(),
        epochs,
    }
}
