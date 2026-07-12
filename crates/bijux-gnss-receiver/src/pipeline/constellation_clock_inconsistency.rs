#![allow(missing_docs)]

use std::collections::BTreeMap;

use bijux_gnss_core::api::{Constellation, NavSolutionEpoch};

const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;
const MAX_OBSERVATION_INTERVAL_S: f64 = 0.1;
const CONSTELLATION_CLOCK_STEP_THRESHOLD_M: f64 = 40.0;
const MIN_SUPPORTING_SATELLITES: usize = 2;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ConstellationClockInconsistency {
    pub constellation: Constellation,
    pub previous_bias_s: f64,
    pub current_bias_s: f64,
    pub bias_step_m: f64,
    pub bias_step_threshold_m: f64,
    pub supporting_satellite_count: usize,
}

pub fn detect_constellation_clock_inconsistencies(
    previous_solution: Option<&NavSolutionEpoch>,
    current_solution: &NavSolutionEpoch,
) -> Vec<ConstellationClockInconsistency> {
    let previous_solution = match previous_solution {
        Some(solution) if solution.valid && current_solution.valid => solution,
        _ => return Vec::new(),
    };

    let dt_s = current_solution.t_rx_s.0 - previous_solution.t_rx_s.0;
    if !dt_s.is_finite() || dt_s <= 0.0 || dt_s > MAX_OBSERVATION_INTERVAL_S {
        return Vec::new();
    }

    let previous_bias_by_constellation = previous_solution
        .isb
        .iter()
        .map(|bias| (bias.constellation, bias.bias_s.0))
        .collect::<BTreeMap<_, _>>();
    let supporting_satellite_count_by_constellation = current_solution
        .constellation_residual_rms
        .iter()
        .map(|summary| (summary.constellation, summary.post_fit_sat_count))
        .collect::<BTreeMap<_, _>>();

    current_solution
        .isb
        .iter()
        .filter_map(|bias| {
            let previous_bias_s = previous_bias_by_constellation.get(&bias.constellation).copied()?;
            let supporting_satellite_count =
                supporting_satellite_count_by_constellation.get(&bias.constellation).copied()?;
            if supporting_satellite_count < MIN_SUPPORTING_SATELLITES {
                return None;
            }

            let bias_step_m = (bias.bias_s.0 - previous_bias_s).abs() * SPEED_OF_LIGHT_MPS;
            (bias_step_m.is_finite() && bias_step_m >= CONSTELLATION_CLOCK_STEP_THRESHOLD_M)
                .then_some(ConstellationClockInconsistency {
                    constellation: bias.constellation,
                    previous_bias_s,
                    current_bias_s: bias.bias_s.0,
                    bias_step_m,
                    bias_step_threshold_m: CONSTELLATION_CLOCK_STEP_THRESHOLD_M,
                    supporting_satellite_count,
                })
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use bijux_gnss_core::api::{
        Constellation, Epoch, InterSystemBias, Meters, NavConstellationResidualRms,
        NavLifecycleState, NavQualityFlag, NavSolutionEpoch, NavUncertaintyClass, Seconds,
        SignalBand, SolutionStatus, SolutionValidity,
    };

    use super::*;

    #[test]
    fn detects_abrupt_galileo_inter_system_bias_jump() {
        let previous_solution =
            mixed_solution_epoch(100_000.0, -1.15e-6, 2, true, SolutionStatus::Converged);
        let current_solution =
            mixed_solution_epoch(100_000.001, -8.5e-7, 2, true, SolutionStatus::Converged);

        let inconsistencies = detect_constellation_clock_inconsistencies(
            Some(&previous_solution),
            &current_solution,
        );

        assert_eq!(inconsistencies.len(), 1);
        let inconsistency = inconsistencies[0];
        assert_eq!(inconsistency.constellation, Constellation::Galileo);
        assert_eq!(inconsistency.supporting_satellite_count, 2);
        assert!(inconsistency.bias_step_m >= inconsistency.bias_step_threshold_m);
    }

    #[test]
    fn ignores_stable_inter_system_bias_evolution() {
        let previous_solution =
            mixed_solution_epoch(100_000.0, -1.15e-6, 2, true, SolutionStatus::Converged);
        let current_solution =
            mixed_solution_epoch(100_000.001, -1.1499e-6, 2, true, SolutionStatus::Converged);

        assert!(
            detect_constellation_clock_inconsistencies(Some(&previous_solution), &current_solution)
                .is_empty()
        );
    }

    #[test]
    fn ignores_gps_only_solution() {
        let previous_solution =
            mixed_solution_epoch(100_000.0, 0.0, 0, true, SolutionStatus::Converged);
        let current_solution =
            mixed_solution_epoch(100_000.001, 0.0, 0, true, SolutionStatus::Converged);

        assert!(
            detect_constellation_clock_inconsistencies(Some(&previous_solution), &current_solution)
                .is_empty()
        );
    }

    #[test]
    fn ignores_invalid_previous_solution() {
        let previous_solution =
            mixed_solution_epoch(100_000.0, -1.15e-6, 2, false, SolutionStatus::Invalid);
        let current_solution =
            mixed_solution_epoch(100_000.001, -8.5e-7, 2, true, SolutionStatus::Converged);

        assert!(
            detect_constellation_clock_inconsistencies(Some(&previous_solution), &current_solution)
                .is_empty()
        );
    }

    fn mixed_solution_epoch(
        t_rx_s: f64,
        galileo_bias_s: f64,
        galileo_satellites: usize,
        valid: bool,
        status: SolutionStatus,
    ) -> NavSolutionEpoch {
        let isb = (galileo_satellites > 0)
            .then_some(InterSystemBias {
                constellation: Constellation::Galileo,
                band: Some(SignalBand::E1),
                bias_s: Seconds(galileo_bias_s),
            })
            .into_iter()
            .collect::<Vec<_>>();
        let mut constellation_residual_rms = vec![NavConstellationResidualRms {
            constellation: Constellation::Gps,
            pre_fit_rms_m: Some(Meters(2.0)),
            post_fit_rms_m: Some(Meters(1.0)),
            pre_fit_sat_count: 4,
            post_fit_sat_count: 4,
        }];
        if galileo_satellites > 0 {
            constellation_residual_rms.push(NavConstellationResidualRms {
                constellation: Constellation::Galileo,
                pre_fit_rms_m: Some(Meters(2.0)),
                post_fit_rms_m: Some(Meters(1.0)),
                pre_fit_sat_count: galileo_satellites,
                post_fit_sat_count: galileo_satellites,
            });
        }

        NavSolutionEpoch {
            epoch: Epoch { index: 0 },
            t_rx_s: Seconds(t_rx_s),
            source_time: Default::default(),
            ecef_x_m: Meters(0.0),
            ecef_y_m: Meters(0.0),
            ecef_z_m: Meters(0.0),
            position_covariance_ecef_m2: None,
            latitude_deg: 0.0,
            longitude_deg: 0.0,
            altitude_m: Meters(0.0),
            clock_bias_s: Seconds(2.75e-4),
            clock_bias_m: Meters(0.0),
            clock_drift_s_per_s: 0.0,
            pdop: 2.0,
            pre_fit_residual_rms_m: Some(Meters(2.0)),
            post_fit_residual_rms_m: Some(Meters(1.0)),
            rms_m: Meters(1.0),
            status,
            quality: if valid {
                NavQualityFlag::Fix
            } else {
                NavQualityFlag::NoFix
            },
            validity: if valid {
                SolutionValidity::Stable
            } else {
                SolutionValidity::Invalid
            },
            valid,
            processing_ms: None,
            residuals: Vec::new(),
            constellation_residual_rms,
            health: Vec::new(),
            isb,
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
            lifecycle_state: if valid {
                NavLifecycleState::Converged
            } else {
                NavLifecycleState::Invalid
            },
            uncertainty_class: NavUncertaintyClass::Low,
            assumptions: None,
            refusal_class: None,
            artifact_id: String::new(),
            source_observation_epoch_id: String::new(),
            explain_decision: String::new(),
            explain_reasons: Vec::new(),
            provenance: None,
            sat_count: 4 + galileo_satellites,
            used_sat_count: 4 + galileo_satellites,
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
