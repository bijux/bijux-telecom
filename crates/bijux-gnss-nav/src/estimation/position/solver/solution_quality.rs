use std::collections::BTreeMap;

use bijux_gnss_core::api::{Constellation, Meters, NavConstellationResidualRms};

use super::geodesy::ecef_to_geodetic;
use super::state::{ConstellationResidualAccumulator, PositionEstimate, WorkingSetResidual};
use super::{
    ImpossibleGeometryEvidence, PositionObservation, RaimFaultDetection,
    RaimSolutionSeparationCheck, ReplayTimingAnomalyEvidence, SatelliteState,
    REPLAY_TIMING_ANOMALY_CENTERED_DELAY_RMS_THRESHOLD_M,
    REPLAY_TIMING_ANOMALY_MAX_CENTERED_DELAY_THRESHOLD_M,
    REPLAY_TIMING_ANOMALY_MIN_MATCHED_SATELLITES, TERRESTRIAL_GEOMETRY_MAX_ALTITUDE_M,
    TERRESTRIAL_GEOMETRY_MAX_RECEIVER_RADIUS_M, TERRESTRIAL_GEOMETRY_MIN_ALTITUDE_M,
    TERRESTRIAL_GEOMETRY_MIN_RECEIVER_RADIUS_M,
};

pub(super) fn sanitize_covariance(
    mut covariance: Vec<Vec<f64>>,
) -> (Vec<Vec<f64>>, bool, bool, Option<f64>) {
    let mut covariance_symmetrized = false;
    let mut covariance_clamped = false;
    let mut covariance_max_variance = None;
    let mut row_index = 0;
    while row_index < covariance.len() {
        let mut column_index = 0;
        while column_index < covariance.len() {
            if (covariance[row_index][column_index] - covariance[column_index][row_index]).abs()
                > 1e-9
            {
                covariance_symmetrized = true;
                let average = 0.5
                    * (covariance[row_index][column_index] + covariance[column_index][row_index]);
                covariance[row_index][column_index] = average;
                covariance[column_index][row_index] = average;
            }
            column_index += 1;
        }
        if covariance[row_index][row_index] < 0.0 {
            covariance_clamped = true;
            covariance[row_index][row_index] = 0.0;
        }
        covariance_max_variance = Some(
            covariance_max_variance
                .map(|running_max: f64| running_max.max(covariance[row_index][row_index]))
                .unwrap_or(covariance[row_index][row_index]),
        );
        row_index += 1;
    }
    (covariance, covariance_symmetrized, covariance_clamped, covariance_max_variance)
}

pub(super) fn detect_impossible_geometry(
    estimate: &PositionEstimate,
    used_satellite_count: usize,
) -> Option<ImpossibleGeometryEvidence> {
    let (receiver_radius_m, altitude_m) = terrestrial_geometry_metrics(estimate);
    if terrestrial_geometry_is_plausible(receiver_radius_m, altitude_m) {
        return None;
    }

    Some(ImpossibleGeometryEvidence {
        receiver_radius_m,
        altitude_m,
        used_satellite_count,
        min_receiver_radius_m: TERRESTRIAL_GEOMETRY_MIN_RECEIVER_RADIUS_M,
        max_receiver_radius_m: TERRESTRIAL_GEOMETRY_MAX_RECEIVER_RADIUS_M,
        min_altitude_m: TERRESTRIAL_GEOMETRY_MIN_ALTITUDE_M,
        max_altitude_m: TERRESTRIAL_GEOMETRY_MAX_ALTITUDE_M,
    })
}

pub(super) fn estimate_has_plausible_terrestrial_geometry(estimate: &PositionEstimate) -> bool {
    let (receiver_radius_m, altitude_m) = terrestrial_geometry_metrics(estimate);
    terrestrial_geometry_is_plausible(receiver_radius_m, altitude_m)
}

fn terrestrial_geometry_metrics(estimate: &PositionEstimate) -> (f64, f64) {
    let receiver_radius_m =
        (estimate.ecef_x_m.powi(2) + estimate.ecef_y_m.powi(2) + estimate.ecef_z_m.powi(2)).sqrt();
    let (_latitude_deg, _longitude_deg, altitude_m) =
        ecef_to_geodetic(estimate.ecef_x_m, estimate.ecef_y_m, estimate.ecef_z_m);
    (receiver_radius_m, altitude_m)
}

fn terrestrial_geometry_is_plausible(receiver_radius_m: f64, altitude_m: f64) -> bool {
    receiver_radius_m.is_finite()
        && altitude_m.is_finite()
        && (TERRESTRIAL_GEOMETRY_MIN_RECEIVER_RADIUS_M..=TERRESTRIAL_GEOMETRY_MAX_RECEIVER_RADIUS_M)
            .contains(&receiver_radius_m)
        && (TERRESTRIAL_GEOMETRY_MIN_ALTITUDE_M..=TERRESTRIAL_GEOMETRY_MAX_ALTITUDE_M)
            .contains(&altitude_m)
}

pub(super) fn working_set_rms_m(residuals: &[WorkingSetResidual]) -> f64 {
    if residuals.is_empty() {
        return 0.0;
    }
    let squared_sum = residuals.iter().map(|residual| residual.residual_m.powi(2)).sum::<f64>();
    (squared_sum / residuals.len() as f64).sqrt()
}

pub(super) fn detect_replay_timing_anomaly(
    filtered: &[(PositionObservation, SatelliteState, f64, f64)],
) -> Option<ReplayTimingAnomalyEvidence> {
    let mut excess_delays_m = filtered
        .iter()
        .filter_map(|(observation, _state, residual_m, _effective_weight)| {
            observation.signal_timing.is_some().then_some(*residual_m)
        })
        .filter(|delay_m| delay_m.is_finite())
        .collect::<Vec<_>>();
    if excess_delays_m.len() < REPLAY_TIMING_ANOMALY_MIN_MATCHED_SATELLITES {
        return None;
    }

    let median_excess_delay_m = median(&mut excess_delays_m);
    let mut centered_delays_m = excess_delays_m
        .into_iter()
        .map(|delay_m| delay_m - median_excess_delay_m)
        .collect::<Vec<_>>();
    let centered_delay_rms_m =
        (centered_delays_m.iter().map(|delay_m| delay_m * delay_m).sum::<f64>()
            / centered_delays_m.len() as f64)
            .sqrt();
    let max_centered_delay_m =
        centered_delays_m.iter().map(|delay_m| delay_m.abs()).fold(0.0, f64::max);
    centered_delays_m.sort_by(|left, right| left.total_cmp(right));
    let strongest_negative_m = centered_delays_m.first().copied().unwrap_or(0.0).abs();
    let strongest_positive_m = centered_delays_m.last().copied().unwrap_or(0.0);

    if centered_delay_rms_m < REPLAY_TIMING_ANOMALY_CENTERED_DELAY_RMS_THRESHOLD_M
        || max_centered_delay_m < REPLAY_TIMING_ANOMALY_MAX_CENTERED_DELAY_THRESHOLD_M
        || strongest_negative_m < REPLAY_TIMING_ANOMALY_MAX_CENTERED_DELAY_THRESHOLD_M * 0.5
        || strongest_positive_m < REPLAY_TIMING_ANOMALY_MAX_CENTERED_DELAY_THRESHOLD_M * 0.5
    {
        return None;
    }

    Some(ReplayTimingAnomalyEvidence {
        matched_satellite_count: centered_delays_m.len(),
        median_excess_delay_m,
        centered_delay_rms_m,
        max_centered_delay_m,
        centered_delay_rms_threshold_m: REPLAY_TIMING_ANOMALY_CENTERED_DELAY_RMS_THRESHOLD_M,
        max_centered_delay_threshold_m: REPLAY_TIMING_ANOMALY_MAX_CENTERED_DELAY_THRESHOLD_M,
    })
}

fn median(values: &mut [f64]) -> f64 {
    values.sort_by(|left, right| left.total_cmp(right));
    let middle = values.len() / 2;
    if values.len() % 2 == 0 {
        (values[middle - 1] + values[middle]) * 0.5
    } else {
        values[middle]
    }
}

pub(super) fn constellation_residual_rms(
    pre_fit: &[WorkingSetResidual],
    post_fit: &[(PositionObservation, SatelliteState, f64, f64)],
) -> Vec<NavConstellationResidualRms> {
    let mut by_constellation = BTreeMap::<Constellation, ConstellationResidualAccumulator>::new();

    for residual in pre_fit {
        let entry = by_constellation.entry(residual.sat.constellation).or_default();
        entry.pre_fit_sum_sq_m2 += residual.residual_m.powi(2);
        entry.pre_fit_sat_count += 1;
    }

    for (observation, _state, residual_m, _effective_weight) in post_fit {
        let entry = by_constellation.entry(observation.sat.constellation).or_default();
        entry.post_fit_sum_sq_m2 += residual_m.powi(2);
        entry.post_fit_sat_count += 1;
    }

    by_constellation
        .into_iter()
        .map(|(constellation, summary)| NavConstellationResidualRms {
            constellation,
            pre_fit_rms_m: (summary.pre_fit_sat_count > 0).then(|| {
                Meters((summary.pre_fit_sum_sq_m2 / summary.pre_fit_sat_count as f64).sqrt())
            }),
            post_fit_rms_m: (summary.post_fit_sat_count > 0).then(|| {
                Meters((summary.post_fit_sum_sq_m2 / summary.post_fit_sat_count as f64).sqrt())
            }),
            pre_fit_sat_count: summary.pre_fit_sat_count,
            post_fit_sat_count: summary.post_fit_sat_count,
        })
        .collect()
}

pub(super) fn solution_separation_m(left: &PositionEstimate, right: &PositionEstimate) -> f64 {
    let dx = left.ecef_x_m - right.ecef_x_m;
    let dy = left.ecef_y_m - right.ecef_y_m;
    let dz = left.ecef_z_m - right.ecef_z_m;
    (dx * dx + dy * dy + dz * dz).sqrt()
}

pub(super) fn supports_reliable_raim_exclusion(usable_sat_count: usize) -> bool {
    usable_sat_count >= 6
}

pub(super) fn supports_reliable_raim_exclusion_after_prior_exclusions(
    usable_sat_count: usize,
    has_prior_exclusion: bool,
) -> bool {
    supports_reliable_raim_exclusion(usable_sat_count)
        || (has_prior_exclusion && usable_sat_count >= 5)
}

pub(super) fn raim_fault_detection_from_separation(
    separation: &RaimSolutionSeparationCheck,
    threshold_m: f64,
) -> RaimFaultDetection {
    if let Some(max_subset) = separation.max_separation() {
        if max_subset.separation_m > threshold_m {
            RaimFaultDetection::fault_detected(
                max_subset.excluded_sat,
                max_subset.separation_m,
                threshold_m,
            )
        } else {
            RaimFaultDetection::consistent(max_subset.separation_m, threshold_m)
        }
    } else {
        RaimFaultDetection::consistent(0.0, threshold_m)
    }
}
