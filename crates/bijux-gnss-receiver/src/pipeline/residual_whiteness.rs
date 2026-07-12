#![allow(missing_docs)]

use std::collections::BTreeMap;

use bijux_gnss_core::api::{NavResidual, NavSolutionEpoch, SatId};

const MAX_OBSERVATION_INTERVAL_S: f64 = 0.1;
const MIN_MATCHED_SATELLITES: usize = 4;
const MIN_CENTERED_RESIDUAL_RMS_M: f64 = 2.3;
const RESIDUAL_CORRELATION_THRESHOLD: f64 = 0.85;
const MIN_PERSISTENT_SUSPECT_EPOCHS: usize = 2;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ResidualTemporalCorrelationEvidence {
    pub lag1_correlation: f64,
    pub correlation_threshold: f64,
    pub matched_satellite_count: usize,
    pub previous_centered_rms_m: f64,
    pub current_centered_rms_m: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ResidualTemporalCorrelation {
    pub lag1_correlation: f64,
    pub correlation_threshold: f64,
    pub matched_satellite_count: usize,
    pub previous_centered_rms_m: f64,
    pub current_centered_rms_m: f64,
    pub persistent_suspect_epochs: usize,
}

pub fn detect_residual_temporal_correlation(
    previous_solution: Option<&NavSolutionEpoch>,
    current_solution: &NavSolutionEpoch,
) -> Option<ResidualTemporalCorrelationEvidence> {
    let previous_solution = match previous_solution {
        Some(solution) if solution.valid && current_solution.valid => solution,
        _ => return None,
    };

    let dt_s = current_solution.t_rx_s.0 - previous_solution.t_rx_s.0;
    if !dt_s.is_finite() || dt_s <= 0.0 || dt_s > MAX_OBSERVATION_INTERVAL_S {
        return None;
    }

    let previous_by_sat = residuals_by_sat(&previous_solution.residuals);
    let matched_pairs = current_solution
        .residuals
        .iter()
        .filter_map(|residual| {
            (!residual.rejected && residual.residual_m.0.is_finite())
                .then_some((residual.sat, residual.residual_m.0))
        })
        .filter_map(|(sat, current_residual_m)| {
            previous_by_sat.get(&sat).map(|previous_residual_m| (*previous_residual_m, current_residual_m))
        })
        .collect::<Vec<_>>();
    if matched_pairs.len() < MIN_MATCHED_SATELLITES {
        return None;
    }

    let previous = matched_pairs.iter().map(|(previous, _)| *previous).collect::<Vec<_>>();
    let current = matched_pairs.iter().map(|(_, current)| *current).collect::<Vec<_>>();
    let previous_mean_m = mean(&previous);
    let current_mean_m = mean(&current);
    let previous_centered = previous
        .iter()
        .map(|residual_m| residual_m - previous_mean_m)
        .collect::<Vec<_>>();
    let current_centered = current
        .iter()
        .map(|residual_m| residual_m - current_mean_m)
        .collect::<Vec<_>>();
    let previous_centered_rms_m = rms(&previous_centered);
    let current_centered_rms_m = rms(&current_centered);
    if previous_centered_rms_m < MIN_CENTERED_RESIDUAL_RMS_M
        || current_centered_rms_m < MIN_CENTERED_RESIDUAL_RMS_M
    {
        return None;
    }

    let lag1_correlation = pearson_correlation(&previous_centered, &current_centered)?;
    if lag1_correlation.abs() < RESIDUAL_CORRELATION_THRESHOLD {
        return None;
    }

    Some(ResidualTemporalCorrelationEvidence {
        lag1_correlation,
        correlation_threshold: RESIDUAL_CORRELATION_THRESHOLD,
        matched_satellite_count: matched_pairs.len(),
        previous_centered_rms_m,
        current_centered_rms_m,
    })
}

pub fn advance_residual_whiteness_suspect_streak(
    previous_streak: usize,
    evidence: Option<ResidualTemporalCorrelationEvidence>,
) -> usize {
    if evidence.is_some() {
        previous_streak + 1
    } else {
        0
    }
}

pub fn classify_residual_temporal_correlation(
    evidence: ResidualTemporalCorrelationEvidence,
    persistent_suspect_epochs: usize,
) -> ResidualTemporalCorrelation {
    ResidualTemporalCorrelation {
        lag1_correlation: evidence.lag1_correlation,
        correlation_threshold: evidence.correlation_threshold,
        matched_satellite_count: evidence.matched_satellite_count,
        previous_centered_rms_m: evidence.previous_centered_rms_m,
        current_centered_rms_m: evidence.current_centered_rms_m,
        persistent_suspect_epochs,
    }
}

pub fn residual_temporal_correlation_is_persistent(persistent_suspect_epochs: usize) -> bool {
    persistent_suspect_epochs >= MIN_PERSISTENT_SUSPECT_EPOCHS
}

fn residuals_by_sat(residuals: &[NavResidual]) -> BTreeMap<SatId, f64> {
    residuals
        .iter()
        .filter_map(|residual| {
            (!residual.rejected && residual.residual_m.0.is_finite())
                .then_some((residual.sat, residual.residual_m.0))
        })
        .collect()
}

fn mean(values: &[f64]) -> f64 {
    values.iter().sum::<f64>() / values.len() as f64
}

fn rms(values: &[f64]) -> f64 {
    (values.iter().map(|value| value * value).sum::<f64>() / values.len() as f64).sqrt()
}

fn pearson_correlation(left: &[f64], right: &[f64]) -> Option<f64> {
    let numerator = left.iter().zip(right.iter()).map(|(x, y)| x * y).sum::<f64>();
    let left_energy = left.iter().map(|value| value * value).sum::<f64>();
    let right_energy = right.iter().map(|value| value * value).sum::<f64>();
    let denominator = (left_energy * right_energy).sqrt();
    (denominator.is_finite() && denominator > 0.0).then_some(numerator / denominator)
}

#[cfg(test)]
mod tests {
    use bijux_gnss_core::api::{
        Constellation, Epoch, Meters, NavLifecycleState, NavQualityFlag, NavResidual,
        NavSolutionEpoch, NavUncertaintyClass, SatId, Seconds, SolutionStatus, SolutionValidity,
    };

    use super::*;

    #[test]
    fn detects_strong_positive_residual_correlation() {
        let sat_ids = gps_satellites(&[3, 7, 11, 19, 23, 29]);
        let previous_solution =
            solution_epoch(100_000.0, &[6.0, -5.0, 4.0, -3.0, 2.0, -1.0], &sat_ids);
        let current_solution =
            solution_epoch(100_000.001, &[5.8, -4.7, 3.9, -2.8, 2.1, -0.9], &sat_ids);

        let evidence =
            detect_residual_temporal_correlation(Some(&previous_solution), &current_solution)
                .expect("correlated residual pattern must be detected");

        assert!(evidence.lag1_correlation >= evidence.correlation_threshold);
        assert_eq!(evidence.matched_satellite_count, sat_ids.len());
        assert!(evidence.previous_centered_rms_m >= MIN_CENTERED_RESIDUAL_RMS_M);
        assert!(evidence.current_centered_rms_m >= MIN_CENTERED_RESIDUAL_RMS_M);
    }

    #[test]
    fn ignores_low_energy_residual_patterns() {
        let sat_ids = gps_satellites(&[3, 7, 11, 19, 23, 29]);
        let previous_solution =
            solution_epoch(100_000.0, &[0.10, -0.08, 0.06, -0.04, 0.02, -0.01], &sat_ids);
        let current_solution =
            solution_epoch(100_000.001, &[0.09, -0.07, 0.05, -0.05, 0.03, -0.02], &sat_ids);

        assert!(
            detect_residual_temporal_correlation(Some(&previous_solution), &current_solution)
                .is_none()
        );
    }

    #[test]
    fn ignores_uncorrelated_residual_patterns() {
        let sat_ids = gps_satellites(&[3, 7, 11, 19, 23, 29]);
        let previous_solution =
            solution_epoch(100_000.0, &[6.0, -5.0, 4.0, -3.0, 2.0, -1.0], &sat_ids);
        let current_solution =
            solution_epoch(100_000.001, &[-2.0, 3.0, -6.0, 1.0, 4.0, 0.0], &sat_ids);

        assert!(
            detect_residual_temporal_correlation(Some(&previous_solution), &current_solution)
                .is_none()
        );
    }

    #[test]
    fn streak_requires_persistence_before_classification() {
        let sat_ids = gps_satellites(&[3, 7, 11, 19, 23, 29]);
        let previous_solution =
            solution_epoch(100_000.0, &[6.0, -5.0, 4.0, -3.0, 2.0, -1.0], &sat_ids);
        let current_solution =
            solution_epoch(100_000.001, &[5.8, -4.7, 3.9, -2.8, 2.1, -0.9], &sat_ids);
        let evidence =
            detect_residual_temporal_correlation(Some(&previous_solution), &current_solution)
                .expect("correlated residual pattern must be detected");

        assert_eq!(advance_residual_whiteness_suspect_streak(0, Some(evidence)), 1);
        assert!(!residual_temporal_correlation_is_persistent(1));

        let correlation = classify_residual_temporal_correlation(evidence, 2);
        assert_eq!(correlation.persistent_suspect_epochs, 2);
        assert!(residual_temporal_correlation_is_persistent(
            correlation.persistent_suspect_epochs
        ));
    }

    fn gps_satellites(prns: &[u8]) -> Vec<SatId> {
        prns.iter().copied().map(|prn| SatId { constellation: Constellation::Gps, prn }).collect()
    }

    fn solution_epoch(
        t_rx_s: f64,
        residuals_m: &[f64],
        sat_ids: &[SatId],
    ) -> NavSolutionEpoch {
        NavSolutionEpoch {
            epoch: Epoch { index: (t_rx_s * 1000.0).round() as u64 },
            t_rx_s: Seconds(t_rx_s),
            source_time: Default::default(),
            ecef_x_m: Meters(0.0),
            ecef_y_m: Meters(0.0),
            ecef_z_m: Meters(0.0),
            position_covariance_ecef_m2: None,
            latitude_deg: 0.0,
            longitude_deg: 0.0,
            altitude_m: Meters(0.0),
            clock_bias_s: Seconds(0.0),
            clock_bias_m: Meters(0.0),
            clock_drift_s_per_s: 0.0,
            pdop: 1.0,
            pre_fit_residual_rms_m: None,
            post_fit_residual_rms_m: None,
            rms_m: Meters(1.0),
            status: SolutionStatus::Converged,
            quality: NavQualityFlag::Fix,
            validity: SolutionValidity::Stable,
            valid: true,
            processing_ms: None,
            residuals: sat_ids
                .iter()
                .zip(residuals_m.iter().copied())
                .map(|(sat, residual_m)| NavResidual {
                    sat: *sat,
                    residual_m: Meters(residual_m),
                    rejected: false,
                    weight: Some(1.0),
                    reject_reason: None,
                })
                .collect(),
            constellation_residual_rms: Vec::new(),
            health: Vec::new(),
            isb: Vec::new(),
            sigma_e_m: None,
            sigma_n_m: None,
            sigma_u_m: None,
            horizontal_error_ellipse_major_axis_m: None,
            horizontal_error_ellipse_minor_axis_m: None,
            horizontal_error_ellipse_azimuth_deg: None,
            sigma_h_m: None,
            sigma_v_m: None,
            innovation_rms_m: None,
            normalized_innovation_rms: None,
            normalized_innovation_max: None,
            ekf_innovation_rms: None,
            ekf_condition_number: None,
            ekf_whiteness_ratio: None,
            ekf_predicted_variance: None,
            ekf_observed_variance: None,
            integrity_hpl_m: None,
            integrity_vpl_m: None,
            model_version: 4,
            lifecycle_state: NavLifecycleState::Converged,
            uncertainty_class: NavUncertaintyClass::Low,
            assumptions: None,
            refusal_class: None,
            artifact_id: String::new(),
            source_observation_epoch_id: String::new(),
            explain_decision: String::new(),
            explain_reasons: Vec::new(),
            provenance: None,
            sat_count: sat_ids.len(),
            used_sat_count: sat_ids.len(),
            rejected_sat_count: 0,
            hdop: Some(1.0),
            vdop: Some(1.0),
            gdop: Some(1.0),
            tdop: Some(1.0),
            stability_signature: String::new(),
            stability_signature_version: 2,
        }
    }
}
