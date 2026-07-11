fn observation_doppler_offsets_hz(
    observations: &[ObsEpoch],
    reference_observations: &[ObsEpoch],
) -> Vec<f64> {
    let reference_by_epoch = reference_observations
        .iter()
        .map(|epoch| {
            let sats = epoch
                .sats
                .iter()
                .map(|sat| (sat.signal_id, sat.doppler_hz.0))
                .collect::<BTreeMap<_, _>>();
            (epoch.epoch_idx, sats)
        })
        .collect::<BTreeMap<_, _>>();
    let mut offsets_hz = Vec::new();
    for epoch in observations {
        let Some(reference_sats) = reference_by_epoch.get(&epoch.epoch_idx) else {
            continue;
        };
        for sat in &epoch.sats {
            if let Some(reference_doppler_hz) = reference_sats.get(&sat.signal_id) {
                offsets_hz.push(sat.doppler_hz.0 - reference_doppler_hz);
            }
        }
    }
    offsets_hz
}

fn clock_profile_truth_coverage_issues(
    case: &SyntheticPvtClockProfileCase<'_>,
    observation_doppler_pair_count: usize,
) -> Vec<SyntheticTruthCoverageIssue> {
    if case.expected_observation_doppler_offset_hz.abs() <= f64::EPSILON {
        return Vec::new();
    }
    if case.reference_observations.is_none() {
        return vec![truth_coverage_issue(
            None,
            None,
            "missing_clock_profile_reference_observations",
        )];
    }
    if observation_doppler_pair_count == 0 {
        return vec![truth_coverage_issue(
            None,
            None,
            "no_clock_profile_observation_doppler_pairs",
        )];
    }
    Vec::new()
}

fn time_profile_truth_coverage_issues(
    truth_epoch_count: usize,
    epoch_count: usize,
    duration_s: f64,
    analysis_window_epoch_count: usize,
    position_error_drift_m_per_s: Option<f64>,
    residual_rms_drift_m_per_s: Option<f64>,
) -> Vec<SyntheticTruthCoverageIssue> {
    let mut issues = Vec::new();
    if truth_epoch_count < MIN_TIME_PROFILE_EPOCH_COUNT {
        issues.push(truth_coverage_issue(None, None, "insufficient_time_profile_truth_epochs"));
    }
    if epoch_count < MIN_TIME_PROFILE_EPOCH_COUNT {
        issues.push(truth_coverage_issue(None, None, "insufficient_time_profile_accuracy_epochs"));
    }
    if duration_s < MIN_TIME_PROFILE_DURATION_S {
        issues.push(truth_coverage_issue(None, None, "insufficient_time_profile_duration"));
    }
    if analysis_window_epoch_count < 2 {
        issues.push(truth_coverage_issue(None, None, "insufficient_time_profile_analysis_window"));
    }
    if position_error_drift_m_per_s.is_none() {
        issues.push(truth_coverage_issue(None, None, "missing_time_profile_position_drift_slope"));
    }
    if residual_rms_drift_m_per_s.is_none() {
        issues.push(truth_coverage_issue(None, None, "missing_time_profile_residual_drift_slope"));
    }
    issues
}

/// Summarize truth-guided PVT accuracy across long-run time-evolution points.
pub fn summarize_truth_guided_pvt_time_profile(
    cases: &[SyntheticPvtTimeProfileCase<'_>],
    scenario_id_prefix: &str,
) -> SyntheticPvtTimeProfileReport {
    let mut points = cases
        .iter()
        .map(|case| {
            let epoch_count = case.accuracy.epoch_count;
            let passing_epoch_count =
                case.accuracy.epochs.iter().filter(|epoch| epoch.pass).count();
            let pass_rate = if epoch_count == 0 {
                0.0
            } else {
                passing_epoch_count as f64 / epoch_count as f64
            };
            let stable_epoch_count = case
                .truth_table
                .epochs
                .iter()
                .filter(|epoch| epoch.solution_validity == SolutionValidity::Stable)
                .count();
            let diverging_epoch_count = case
                .truth_table
                .epochs
                .iter()
                .filter(|epoch| epoch.solution_validity == SolutionValidity::Diverging)
                .count();
            let truth_epoch_count = case.truth_table.epochs.len();
            let stable_epoch_rate = if truth_epoch_count == 0 {
                0.0
            } else {
                stable_epoch_count as f64 / truth_epoch_count as f64
            };
            let diverging_epoch_rate = if truth_epoch_count == 0 {
                0.0
            } else {
                diverging_epoch_count as f64 / truth_epoch_count as f64
            };
            let receive_times_s = case
                .truth_table
                .epochs
                .iter()
                .map(|epoch| epoch.receive_time_s)
                .collect::<Vec<_>>();
            let duration_s = match (receive_times_s.first(), receive_times_s.last()) {
                (Some(first), Some(last)) if last > first => last - first,
                _ => 0.0,
            };
            let position_error_3d_m = case
                .accuracy
                .epochs
                .iter()
                .map(|epoch| epoch.position_error_3d_m)
                .collect::<Vec<_>>();
            let residual_rms_m =
                case.accuracy.epochs.iter().map(|epoch| epoch.residual_rms_m).collect::<Vec<_>>();
            let window_len = time_profile_window_len(epoch_count);
            let first_window_mean_position_error_3d_m =
                mean_f64(&position_error_3d_m[..window_len.min(position_error_3d_m.len())]);
            let last_window_mean_position_error_3d_m = if position_error_3d_m.is_empty() {
                None
            } else {
                mean_f64(
                    &position_error_3d_m
                        [position_error_3d_m.len() - window_len.min(position_error_3d_m.len())..],
                )
            };
            let first_window_mean_residual_rms_m =
                mean_f64(&residual_rms_m[..window_len.min(residual_rms_m.len())]);
            let last_window_mean_residual_rms_m = if residual_rms_m.is_empty() {
                None
            } else {
                mean_f64(
                    &residual_rms_m[residual_rms_m.len() - window_len.min(residual_rms_m.len())..],
                )
            };
            let position_error_growth_m = first_window_mean_position_error_3d_m
                .zip(last_window_mean_position_error_3d_m)
                .map(|(first, last)| last - first);
            let residual_rms_growth_m = first_window_mean_residual_rms_m
                .zip(last_window_mean_residual_rms_m)
                .map(|(first, last)| last - first);
            let position_error_drift_m_per_s =
                linear_trend_slope(&receive_times_s, &position_error_3d_m);
            let residual_rms_drift_m_per_s = linear_trend_slope(&receive_times_s, &residual_rms_m);
            let analysis_window_epoch_count = window_len.min(epoch_count);
            let time_profile_truth_coverage_issues = time_profile_truth_coverage_issues(
                truth_epoch_count,
                epoch_count,
                duration_s,
                analysis_window_epoch_count,
                position_error_drift_m_per_s,
                residual_rms_drift_m_per_s,
            );
            let truth_coverage_ready =
                case.accuracy.truth_coverage_ready && time_profile_truth_coverage_issues.is_empty();
            let mut truth_coverage_issues = case.accuracy.truth_coverage_issues.clone();
            truth_coverage_issues.extend(time_profile_truth_coverage_issues);
            let trend = classify_pvt_time_trend(
                diverging_epoch_count,
                first_window_mean_position_error_3d_m,
                last_window_mean_position_error_3d_m,
                position_error_drift_m_per_s,
                first_window_mean_residual_rms_m,
                last_window_mean_residual_rms_m,
                residual_rms_drift_m_per_s,
            );

            SyntheticPvtTimeProfilePoint {
                scenario_id: case.scenario_id.to_string(),
                truth_epoch_count,
                duration_s,
                epoch_count,
                passing_epoch_count,
                pass_rate,
                stable_epoch_count,
                stable_epoch_rate,
                diverging_epoch_count,
                diverging_epoch_rate,
                analysis_window_epoch_count,
                first_window_mean_position_error_3d_m,
                last_window_mean_position_error_3d_m,
                position_error_growth_m,
                position_error_drift_m_per_s,
                first_window_mean_residual_rms_m,
                last_window_mean_residual_rms_m,
                residual_rms_growth_m,
                residual_rms_drift_m_per_s,
                trend,
                truth_coverage_ready,
                truth_coverage_issues,
                ready: truth_coverage_ready,
            }
        })
        .collect::<Vec<_>>();
    points.sort_by(|left, right| {
        left.duration_s
            .partial_cmp(&right.duration_s)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then_with(|| left.scenario_id.cmp(&right.scenario_id))
    });

    SyntheticPvtTimeProfileReport { scenario_id_prefix: scenario_id_prefix.to_string(), points }
}

/// Merge acquisition, tracking, and PVT C/N0 profiles into one stage-level output.
pub fn summarize_truth_guided_accuracy_cn0_profile(
    scenario_id_prefix: &str,
    acquisition: &SyntheticAcquisitionDetectionRateReport,
    tracking: &SyntheticTrackingLockRateReport,
    pvt: &SyntheticPvtCn0ProfileReport,
) -> SyntheticAccuracyCn0ProfileReport {
    #[derive(Default)]
    struct Accumulator {
        acquisition_trial_count: usize,
        acquisition_acceptance_probability: Vec<f64>,
        acquisition_detection_probability: Vec<f64>,
        tracking_trial_count: usize,
        tracking_lock_probability: Vec<f64>,
        tracking_refused_lock_rate: Vec<f64>,
        pvt_epoch_count: usize,
        pvt_pass_rate: Vec<f64>,
        pvt_rms_position_error_3d_m: Vec<f64>,
        pvt_max_position_error_3d_m: Vec<f64>,
    }

    let mut by_cn0 = BTreeMap::<i32, Accumulator>::new();
    for point in &acquisition.points {
        let entry = by_cn0.entry((point.cn0_db_hz * 10.0).round() as i32).or_default();
        entry.acquisition_trial_count += point.trial_count;
        entry.acquisition_acceptance_probability.push(point.acceptance_probability);
        entry.acquisition_detection_probability.push(point.detection_probability);
    }
    for point in &tracking.points {
        let entry = by_cn0.entry((point.cn0_db_hz * 10.0).round() as i32).or_default();
        entry.tracking_trial_count += point.trial_count;
        entry.tracking_lock_probability.push(point.lock_probability);
        if point.trial_count > 0 {
            entry
                .tracking_refused_lock_rate
                .push(point.refused_lock_count as f64 / point.trial_count as f64);
        }
    }
    for point in &pvt.points {
        let entry =
            by_cn0.entry((point.mean_observation_cn0_dbhz * 10.0).round() as i32).or_default();
        entry.pvt_epoch_count += point.epoch_count;
        entry.pvt_pass_rate.push(point.pass_rate);
        if let Some(rms_position_error_3d_m) = point.rms_position_error_3d_m {
            entry.pvt_rms_position_error_3d_m.push(rms_position_error_3d_m);
        }
        if let Some(max_position_error_3d_m) = point.max_position_error_3d_m {
            entry.pvt_max_position_error_3d_m.push(max_position_error_3d_m);
        }
    }

    let points = by_cn0
        .into_iter()
        .map(|(cn0_tenths_db_hz, values)| SyntheticAccuracyCn0ProfilePoint {
            cn0_db_hz: cn0_tenths_db_hz as f64 / 10.0,
            acquisition_case_count: values.acquisition_detection_probability.len(),
            acquisition_trial_count: values.acquisition_trial_count,
            acquisition_acceptance_probability_mean: mean_f64(
                &values.acquisition_acceptance_probability,
            ),
            acquisition_detection_probability_mean: mean_f64(
                &values.acquisition_detection_probability,
            ),
            tracking_case_count: values.tracking_lock_probability.len(),
            tracking_trial_count: values.tracking_trial_count,
            tracking_lock_probability_mean: mean_f64(&values.tracking_lock_probability),
            tracking_refused_lock_rate_mean: mean_f64(&values.tracking_refused_lock_rate),
            pvt_case_count: values.pvt_pass_rate.len(),
            pvt_epoch_count: values.pvt_epoch_count,
            pvt_pass_rate_mean: mean_f64(&values.pvt_pass_rate),
            pvt_rms_position_error_3d_m_mean: mean_f64(&values.pvt_rms_position_error_3d_m),
            pvt_max_position_error_3d_m_max: values
                .pvt_max_position_error_3d_m
                .iter()
                .copied()
                .reduce(f64::max),
        })
        .collect::<Vec<_>>();

    SyntheticAccuracyCn0ProfileReport { scenario_id_prefix: scenario_id_prefix.to_string(), points }
}
