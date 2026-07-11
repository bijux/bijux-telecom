/// Summarize truth-guided PVT accuracy across multiple receiver-motion truth paths.
pub fn summarize_truth_guided_pvt_motion_profile(
    cases: &[SyntheticPvtMotionProfileCase<'_>],
    scenario_id_prefix: &str,
) -> SyntheticPvtMotionProfileReport {
    let mut points = cases
        .iter()
        .map(|case| {
            let position_error_3d_m = case
                .accuracy
                .epochs
                .iter()
                .map(|epoch| epoch.position_error_3d_m)
                .collect::<Vec<_>>();
            let residual_rms_m =
                case.accuracy.epochs.iter().map(|epoch| epoch.residual_rms_m).collect::<Vec<_>>();
            let passing_epoch_count =
                case.accuracy.epochs.iter().filter(|epoch| epoch.pass).count();
            let epoch_count = case.accuracy.epoch_count;
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
            let truth_epoch_count = case.truth_table.epochs.len();
            let stable_epoch_rate = if truth_epoch_count == 0 {
                0.0
            } else {
                stable_epoch_count as f64 / truth_epoch_count as f64
            };
            let truth_path_segments = case
                .truth_table
                .epochs
                .windows(2)
                .map(|window| {
                    let previous = &window[0].truth_ecef_m;
                    let current = &window[1].truth_ecef_m;
                    let dx = current.x_m - previous.x_m;
                    let dy = current.y_m - previous.y_m;
                    let dz = current.z_m - previous.z_m;
                    let dt_s = window[1].receive_time_s - window[0].receive_time_s;
                    ((dx * dx + dy * dy + dz * dz).sqrt(), dt_s)
                })
                .collect::<Vec<_>>();
            let path_length_m =
                truth_path_segments.iter().map(|(segment_length_m, _)| *segment_length_m).sum();
            let duration_s = truth_path_segments.iter().map(|(_, dt_s)| *dt_s).sum::<f64>();
            let mean_speed_mps = if duration_s <= 0.0 { 0.0 } else { path_length_m / duration_s };

            SyntheticPvtMotionProfilePoint {
                scenario_id: case.scenario_id.to_string(),
                truth_epoch_count,
                moving: path_length_m > 0.0,
                path_length_m,
                mean_speed_mps,
                epoch_count,
                passing_epoch_count,
                pass_rate,
                stable_epoch_count,
                stable_epoch_rate,
                rms_position_error_3d_m: (!position_error_3d_m.is_empty())
                    .then(|| stats(&position_error_3d_m).rms),
                max_position_error_3d_m: position_error_3d_m.iter().copied().reduce(f64::max),
                rms_residual_rms_m: (!residual_rms_m.is_empty())
                    .then(|| stats(&residual_rms_m).rms),
                max_residual_rms_m: residual_rms_m.iter().copied().reduce(f64::max),
                truth_coverage_ready: case.accuracy.truth_coverage_ready,
                truth_coverage_issues: case.accuracy.truth_coverage_issues.clone(),
                ready: case.accuracy.truth_coverage_ready
                    && truth_epoch_count > 0
                    && epoch_count > 0,
            }
        })
        .collect::<Vec<_>>();
    points.sort_by(|left, right| {
        left.path_length_m
            .partial_cmp(&right.path_length_m)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then_with(|| {
                left.mean_speed_mps
                    .partial_cmp(&right.mean_speed_mps)
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .then_with(|| left.scenario_id.cmp(&right.scenario_id))
    });

    SyntheticPvtMotionProfileReport { scenario_id_prefix: scenario_id_prefix.to_string(), points }
}

/// Summarize truth-guided PVT accuracy across multiple injected receiver clock-drift points.
pub fn summarize_truth_guided_pvt_clock_profile(
    cases: &[SyntheticPvtClockProfileCase<'_>],
    scenario_id_prefix: &str,
) -> SyntheticPvtClockProfileReport {
    let mut points = cases
        .iter()
        .map(|case| {
            let observation_doppler_offsets_hz = case
                .reference_observations
                .map(|reference_observations| {
                    observation_doppler_offsets_hz(case.observations, reference_observations)
                })
                .unwrap_or_default();
            let clock_profile_truth_coverage_issues =
                clock_profile_truth_coverage_issues(case, observation_doppler_offsets_hz.len());
            let solved_clock_drift_s_per_s = case
                .solutions
                .iter()
                .map(|solution| solution.clock_drift_s_per_s)
                .filter(|value| value.is_finite())
                .collect::<Vec<_>>();
            let clock_drift_error_s_per_s = solved_clock_drift_s_per_s
                .iter()
                .map(|value| (value - case.injected_clock_drift_s_per_s).abs())
                .collect::<Vec<_>>();
            let clock_bias_error_m = case
                .accuracy
                .epochs
                .iter()
                .map(|epoch| epoch.clock_bias_error_m)
                .collect::<Vec<_>>();
            let residual_rms_m =
                case.accuracy.epochs.iter().map(|epoch| epoch.residual_rms_m).collect::<Vec<_>>();
            let passing_epoch_count =
                case.accuracy.epochs.iter().filter(|epoch| epoch.pass).count();
            let epoch_count = case.accuracy.epoch_count;
            let pass_rate = if epoch_count == 0 {
                0.0
            } else {
                passing_epoch_count as f64 / epoch_count as f64
            };
            let truth_coverage_ready = case.accuracy.truth_coverage_ready
                && clock_profile_truth_coverage_issues.is_empty();
            let mut truth_coverage_issues = case.accuracy.truth_coverage_issues.clone();
            truth_coverage_issues.extend(clock_profile_truth_coverage_issues);

            SyntheticPvtClockProfilePoint {
                scenario_id: case.scenario_id.to_string(),
                injected_clock_drift_s_per_s: case.injected_clock_drift_s_per_s,
                expected_observation_doppler_offset_hz: case.expected_observation_doppler_offset_hz,
                epoch_count,
                passing_epoch_count,
                pass_rate,
                observation_doppler_pair_count: observation_doppler_offsets_hz.len(),
                observed_mean_observation_doppler_offset_hz: mean_f64(
                    &observation_doppler_offsets_hz,
                ),
                observed_min_observation_doppler_offset_hz: observation_doppler_offsets_hz
                    .iter()
                    .copied()
                    .reduce(f64::min),
                observed_max_observation_doppler_offset_hz: observation_doppler_offsets_hz
                    .iter()
                    .copied()
                    .reduce(f64::max),
                max_observation_doppler_offset_error_hz: observation_doppler_offsets_hz
                    .iter()
                    .map(|value| (value - case.expected_observation_doppler_offset_hz).abs())
                    .reduce(f64::max),
                final_solved_clock_drift_s_per_s: solved_clock_drift_s_per_s.last().copied(),
                mean_solved_clock_drift_s_per_s: mean_f64(&solved_clock_drift_s_per_s),
                max_clock_drift_error_s_per_s: clock_drift_error_s_per_s
                    .iter()
                    .copied()
                    .reduce(f64::max),
                rms_clock_bias_error_m: (!clock_bias_error_m.is_empty())
                    .then(|| stats(&clock_bias_error_m).rms),
                max_clock_bias_error_m: clock_bias_error_m.iter().copied().reduce(f64::max),
                rms_residual_rms_m: (!residual_rms_m.is_empty())
                    .then(|| stats(&residual_rms_m).rms),
                max_residual_rms_m: residual_rms_m.iter().copied().reduce(f64::max),
                truth_coverage_ready,
                truth_coverage_issues,
                ready: truth_coverage_ready
                    && epoch_count > 0
                    && !solved_clock_drift_s_per_s.is_empty()
                    && (case.reference_observations.is_some()
                        || case.expected_observation_doppler_offset_hz.abs() <= f64::EPSILON),
            }
        })
        .collect::<Vec<_>>();
    points.sort_by(|left, right| {
        left.injected_clock_drift_s_per_s
            .partial_cmp(&right.injected_clock_drift_s_per_s)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then_with(|| left.scenario_id.cmp(&right.scenario_id))
    });

    SyntheticPvtClockProfileReport { scenario_id_prefix: scenario_id_prefix.to_string(), points }
}

