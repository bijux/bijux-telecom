/// Summarize truth-guided PVT accuracy across multiple signal-strength points.
pub fn summarize_truth_guided_pvt_cn0_profile(
    cases: &[SyntheticPvtCn0ProfileCase<'_>],
    scenario_id_prefix: &str,
) -> SyntheticPvtCn0ProfileReport {
    let mut points = cases
        .iter()
        .map(|case| {
            let epoch_mean_cn0 = case
                .observations
                .iter()
                .filter_map(|epoch| {
                    let values = epoch
                        .sats
                        .iter()
                        .filter_map(|sat| sat.cn0_dbhz.is_finite().then_some(sat.cn0_dbhz))
                        .collect::<Vec<_>>();
                    if values.is_empty() {
                        None
                    } else {
                        Some(values.iter().sum::<f64>() / values.len() as f64)
                    }
                })
                .collect::<Vec<_>>();
            let observation_epoch_count = epoch_mean_cn0.len();
            let mean_observation_cn0_dbhz = if epoch_mean_cn0.is_empty() {
                0.0
            } else {
                epoch_mean_cn0.iter().sum::<f64>() / epoch_mean_cn0.len() as f64
            };
            let min_observation_cn0_dbhz =
                epoch_mean_cn0.iter().copied().reduce(f64::min).unwrap_or(0.0);
            let max_observation_cn0_dbhz =
                epoch_mean_cn0.iter().copied().reduce(f64::max).unwrap_or(0.0);
            let position_error_3d_m = case
                .accuracy
                .epochs
                .iter()
                .map(|epoch| epoch.position_error_3d_m)
                .collect::<Vec<_>>();
            let clock_bias_error_m = case
                .accuracy
                .epochs
                .iter()
                .map(|epoch| epoch.clock_bias_error_m)
                .collect::<Vec<_>>();
            let residual_rms_m =
                case.accuracy.epochs.iter().map(|epoch| epoch.residual_rms_m).collect::<Vec<_>>();
            let pdop = case.accuracy.epochs.iter().map(|epoch| epoch.pdop).collect::<Vec<_>>();
            let passing_epoch_count =
                case.accuracy.epochs.iter().filter(|epoch| epoch.pass).count();
            let epoch_count = case.accuracy.epoch_count;
            let pass_rate = if epoch_count == 0 {
                0.0
            } else {
                passing_epoch_count as f64 / epoch_count as f64
            };

            SyntheticPvtCn0ProfilePoint {
                scenario_id: case.scenario_id.to_string(),
                observation_epoch_count,
                mean_observation_cn0_dbhz,
                min_observation_cn0_dbhz,
                max_observation_cn0_dbhz,
                epoch_count,
                passing_epoch_count,
                pass_rate,
                rms_position_error_3d_m: (!position_error_3d_m.is_empty())
                    .then(|| stats(&position_error_3d_m).rms),
                max_position_error_3d_m: position_error_3d_m.iter().copied().reduce(f64::max),
                rms_clock_bias_error_m: (!clock_bias_error_m.is_empty())
                    .then(|| stats(&clock_bias_error_m).rms),
                max_clock_bias_error_m: clock_bias_error_m.iter().copied().reduce(f64::max),
                rms_residual_rms_m: (!residual_rms_m.is_empty())
                    .then(|| stats(&residual_rms_m).rms),
                max_residual_rms_m: residual_rms_m.iter().copied().reduce(f64::max),
                max_pdop: pdop.iter().copied().reduce(f64::max),
                truth_coverage_ready: case.accuracy.truth_coverage_ready,
                truth_coverage_issues: case.accuracy.truth_coverage_issues.clone(),
                ready: case.accuracy.truth_coverage_ready
                    && observation_epoch_count > 0
                    && epoch_count > 0,
            }
        })
        .collect::<Vec<_>>();
    points.sort_by(|left, right| {
        left.mean_observation_cn0_dbhz
            .partial_cmp(&right.mean_observation_cn0_dbhz)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then_with(|| left.scenario_id.cmp(&right.scenario_id))
    });

    SyntheticPvtCn0ProfileReport { scenario_id_prefix: scenario_id_prefix.to_string(), points }
}

/// Summarize truth-guided PVT accuracy across multiple satellite-geometry points.
pub fn summarize_truth_guided_pvt_geometry_profile(
    cases: &[SyntheticPvtGeometryProfileCase<'_>],
    scenario_id_prefix: &str,
) -> SyntheticPvtGeometryProfileReport {
    let mut points = cases
        .iter()
        .map(|case| {
            let position_error_3d_m = case
                .accuracy
                .epochs
                .iter()
                .map(|epoch| epoch.position_error_3d_m)
                .collect::<Vec<_>>();
            let clock_bias_error_m = case
                .accuracy
                .epochs
                .iter()
                .map(|epoch| epoch.clock_bias_error_m)
                .collect::<Vec<_>>();
            let residual_rms_m =
                case.accuracy.epochs.iter().map(|epoch| epoch.residual_rms_m).collect::<Vec<_>>();
            let pdop = case.accuracy.epochs.iter().map(|epoch| epoch.pdop).collect::<Vec<_>>();
            let passing_epoch_count =
                case.accuracy.epochs.iter().filter(|epoch| epoch.pass).count();
            let epoch_count = case.accuracy.epoch_count;
            let pass_rate = if epoch_count == 0 {
                0.0
            } else {
                passing_epoch_count as f64 / epoch_count as f64
            };

            SyntheticPvtGeometryProfilePoint {
                scenario_id: case.scenario_id.to_string(),
                epoch_count,
                passing_epoch_count,
                pass_rate,
                mean_pdop: mean_f64(&pdop),
                min_pdop: pdop.iter().copied().reduce(f64::min),
                max_pdop: pdop.iter().copied().reduce(f64::max),
                rms_position_error_3d_m: (!position_error_3d_m.is_empty())
                    .then(|| stats(&position_error_3d_m).rms),
                max_position_error_3d_m: position_error_3d_m.iter().copied().reduce(f64::max),
                rms_clock_bias_error_m: (!clock_bias_error_m.is_empty())
                    .then(|| stats(&clock_bias_error_m).rms),
                max_clock_bias_error_m: clock_bias_error_m.iter().copied().reduce(f64::max),
                rms_residual_rms_m: (!residual_rms_m.is_empty())
                    .then(|| stats(&residual_rms_m).rms),
                max_residual_rms_m: residual_rms_m.iter().copied().reduce(f64::max),
                truth_coverage_ready: case.accuracy.truth_coverage_ready,
                truth_coverage_issues: case.accuracy.truth_coverage_issues.clone(),
                ready: case.accuracy.truth_coverage_ready && !pdop.is_empty() && epoch_count > 0,
            }
        })
        .collect::<Vec<_>>();
    points.sort_by(|left, right| {
        left.mean_pdop
            .partial_cmp(&right.mean_pdop)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then_with(|| left.scenario_id.cmp(&right.scenario_id))
    });

    SyntheticPvtGeometryProfileReport { scenario_id_prefix: scenario_id_prefix.to_string(), points }
}

/// Summarize truth-guided PVT accuracy across GPS-only and mixed-constellation geometry points.
pub fn summarize_truth_guided_pvt_constellation_geometry_profile(
    cases: &[SyntheticPvtConstellationGeometryProfileCase<'_>],
    scenario_id_prefix: &str,
) -> SyntheticPvtConstellationGeometryProfileReport {
    let mut points = cases
        .iter()
        .map(|case| {
            let position_error_3d_m = case
                .accuracy
                .epochs
                .iter()
                .map(|epoch| epoch.position_error_3d_m)
                .collect::<Vec<_>>();
            let clock_bias_error_m = case
                .accuracy
                .epochs
                .iter()
                .map(|epoch| epoch.clock_bias_error_m)
                .collect::<Vec<_>>();
            let residual_rms_m =
                case.accuracy.epochs.iter().map(|epoch| epoch.residual_rms_m).collect::<Vec<_>>();
            let pdop = case.accuracy.epochs.iter().map(|epoch| epoch.pdop).collect::<Vec<_>>();
            let gdop = case
                .truth_table
                .epochs
                .iter()
                .filter_map(|epoch| epoch.dop.gdop)
                .collect::<Vec<_>>();
            let expected_epoch_count =
                case.truth_table.matched_epoch_count + case.truth_table.unused_reference_epochs.len();
            let solved_epoch_count = case.truth_table.matched_epoch_count;
            let availability_rate = if expected_epoch_count == 0 {
                0.0
            } else {
                solved_epoch_count as f64 / expected_epoch_count as f64
            };
            let passing_epoch_count =
                case.accuracy.epochs.iter().filter(|epoch| epoch.pass).count();
            let epoch_count = case.accuracy.epoch_count;
            let pass_rate = if epoch_count == 0 {
                0.0
            } else {
                passing_epoch_count as f64 / epoch_count as f64
            };

            SyntheticPvtConstellationGeometryProfilePoint {
                scenario_id: case.scenario_id.to_string(),
                constellations: case.constellations.to_vec(),
                visible_satellite_count: case.visible_satellite_count,
                expected_epoch_count,
                solved_epoch_count,
                availability_rate,
                passing_epoch_count,
                pass_rate,
                mean_pdop: mean_f64(&pdop),
                min_pdop: pdop.iter().copied().reduce(f64::min),
                max_pdop: pdop.iter().copied().reduce(f64::max),
                mean_gdop: mean_f64(&gdop),
                min_gdop: gdop.iter().copied().reduce(f64::min),
                max_gdop: gdop.iter().copied().reduce(f64::max),
                rms_position_error_3d_m: (!position_error_3d_m.is_empty())
                    .then(|| stats(&position_error_3d_m).rms),
                max_position_error_3d_m: position_error_3d_m.iter().copied().reduce(f64::max),
                rms_clock_bias_error_m: (!clock_bias_error_m.is_empty())
                    .then(|| stats(&clock_bias_error_m).rms),
                max_clock_bias_error_m: clock_bias_error_m.iter().copied().reduce(f64::max),
                rms_residual_rms_m: (!residual_rms_m.is_empty())
                    .then(|| stats(&residual_rms_m).rms),
                max_residual_rms_m: residual_rms_m.iter().copied().reduce(f64::max),
                truth_coverage_ready: case.accuracy.truth_coverage_ready,
                truth_coverage_issues: case.accuracy.truth_coverage_issues.clone(),
                ready: case.accuracy.truth_coverage_ready && expected_epoch_count > 0,
            }
        })
        .collect::<Vec<_>>();
    points.sort_by(|left, right| {
        right
            .availability_rate
            .partial_cmp(&left.availability_rate)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then_with(|| {
                left.mean_pdop
                    .partial_cmp(&right.mean_pdop)
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .then_with(|| left.scenario_id.cmp(&right.scenario_id))
    });

    SyntheticPvtConstellationGeometryProfileReport {
        scenario_id_prefix: scenario_id_prefix.to_string(),
        points,
    }
}

/// Summarize truth-guided PVT accuracy across multiple synthetic multipath points.
pub fn summarize_truth_guided_pvt_multipath_profile(
    cases: &[SyntheticPvtMultipathProfileCase<'_>],
    scenario_id_prefix: &str,
) -> SyntheticPvtMultipathProfileReport {
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

            SyntheticPvtMultipathProfilePoint {
                scenario_id: case.scenario_id.to_string(),
                affected_satellite_count: case.affected_satellite_count,
                mean_abs_pseudorange_bias_m: case.mean_abs_pseudorange_bias_m,
                max_abs_pseudorange_bias_m: case.max_abs_pseudorange_bias_m,
                epoch_count,
                passing_epoch_count,
                pass_rate,
                stable_epoch_count,
                stable_epoch_rate,
                diverging_epoch_count,
                diverging_epoch_rate,
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
        left.max_abs_pseudorange_bias_m
            .partial_cmp(&right.max_abs_pseudorange_bias_m)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then_with(|| left.scenario_id.cmp(&right.scenario_id))
    });

    SyntheticPvtMultipathProfileReport {
        scenario_id_prefix: scenario_id_prefix.to_string(),
        points,
    }
}
