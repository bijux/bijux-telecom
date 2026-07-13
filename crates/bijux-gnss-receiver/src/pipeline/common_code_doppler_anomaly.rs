#![allow(missing_docs)]

use std::collections::BTreeMap;

use bijux_gnss_core::api::NavSolutionEpoch;
use bijux_gnss_nav::api::PositionObservation;

const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;
const COMMON_CODE_STEP_THRESHOLD_M: f64 = 40.0;
const COMMON_DOPPLER_STEP_THRESHOLD_HZ: f64 = 75.0;
const DOPPLER_ALIGNMENT_TOLERANCE_HZ: f64 = 20.0;
const MIN_MATCHED_SATELLITES: usize = 4;
const MIN_ALIGNED_FRACTION: f64 = 0.8;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CommonCodeDopplerAnomaly {
    pub common_code_step_m: f64,
    pub common_doppler_step_hz: f64,
    pub matched_satellite_count: usize,
    pub aligned_satellite_count: usize,
    pub code_step_threshold_m: f64,
    pub doppler_step_threshold_hz: f64,
}

pub fn detect_common_code_doppler_anomaly(
    previous_solution: Option<&NavSolutionEpoch>,
    previous_observations: Option<&[PositionObservation]>,
    current_solution: &NavSolutionEpoch,
    current_observations: &[PositionObservation],
) -> Option<CommonCodeDopplerAnomaly> {
    let previous_solution = previous_solution?;
    let previous_observations = previous_observations?;

    let dt_s = current_solution.t_rx_s.0 - previous_solution.t_rx_s.0;
    if !dt_s.is_finite() || dt_s <= 0.0 {
        return None;
    }

    let expected_clock_bias_s =
        previous_solution.clock_bias_s.0 + previous_solution.clock_drift_s_per_s * dt_s;
    let common_code_step_m =
        (current_solution.clock_bias_s.0 - expected_clock_bias_s) * SPEED_OF_LIGHT_MPS;
    if !common_code_step_m.is_finite() || common_code_step_m.abs() < COMMON_CODE_STEP_THRESHOLD_M {
        return None;
    }

    let previous_doppler_by_sat = previous_observations
        .iter()
        .filter_map(|observation| {
            observation.doppler_hz.map(|doppler_hz| (observation.sat, doppler_hz))
        })
        .collect::<BTreeMap<_, _>>();
    let mut common_doppler_steps_hz = current_observations
        .iter()
        .filter_map(|observation| {
            observation.doppler_hz.and_then(|current_doppler_hz| {
                previous_doppler_by_sat
                    .get(&observation.sat)
                    .map(|previous_doppler_hz| current_doppler_hz - previous_doppler_hz)
            })
        })
        .filter(|delta_hz| delta_hz.is_finite())
        .collect::<Vec<_>>();
    if common_doppler_steps_hz.len() < MIN_MATCHED_SATELLITES {
        return None;
    }

    let common_doppler_step_hz = median(&mut common_doppler_steps_hz);
    if common_doppler_step_hz.abs() < COMMON_DOPPLER_STEP_THRESHOLD_HZ {
        return None;
    }

    let aligned_satellite_count = common_doppler_steps_hz
        .iter()
        .filter(|delta_hz| {
            same_direction(**delta_hz, common_doppler_step_hz)
                && (**delta_hz - common_doppler_step_hz).abs() <= DOPPLER_ALIGNMENT_TOLERANCE_HZ
        })
        .count();
    let aligned_fraction = aligned_satellite_count as f64 / common_doppler_steps_hz.len() as f64;
    if aligned_fraction < MIN_ALIGNED_FRACTION {
        return None;
    }

    Some(CommonCodeDopplerAnomaly {
        common_code_step_m,
        common_doppler_step_hz,
        matched_satellite_count: common_doppler_steps_hz.len(),
        aligned_satellite_count,
        code_step_threshold_m: COMMON_CODE_STEP_THRESHOLD_M,
        doppler_step_threshold_hz: COMMON_DOPPLER_STEP_THRESHOLD_HZ,
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

fn same_direction(left: f64, right: f64) -> bool {
    left == 0.0 || right == 0.0 || left.signum() == right.signum()
}

#[cfg(test)]
mod tests {
    use bijux_gnss_core::api::{
        Constellation, Epoch, Meters, NavLifecycleState, NavRefusalClass,
        NavResidual, NavSolutionEpoch, NavUncertaintyClass, SatId, Seconds, SolutionStatus,
        SolutionValidity,
    };
    use bijux_gnss_nav::api::PositionObservation;

    use super::*;

    #[test]
    fn detects_common_code_and_doppler_step() {
        let sat_ids = gps_satellites(&[3, 7, 11, 19, 23, 29]);
        let previous_solution = solution_epoch(100_000.0, 0.0, 0.0);
        let current_solution = solution_epoch(100_000.001, 80.0 / SPEED_OF_LIGHT_MPS, 0.0);
        let previous_observations = observations_with_doppler(
            &sat_ids,
            &[-1_800.0, -1_000.0, -250.0, 500.0, 1_250.0, 2_000.0],
        );
        let current_observations = observations_with_doppler(
            &sat_ids,
            &[-1_320.0, -520.0, 230.0, 980.0, 1_730.0, 2_480.0],
        );

        let anomaly = detect_common_code_doppler_anomaly(
            Some(&previous_solution),
            Some(&previous_observations),
            &current_solution,
            &current_observations,
        )
        .expect("coherent common step must be detected");

        assert!(anomaly.common_code_step_m >= 79.0);
        assert!(anomaly.common_doppler_step_hz >= 470.0);
        assert_eq!(anomaly.matched_satellite_count, sat_ids.len());
        assert_eq!(anomaly.aligned_satellite_count, sat_ids.len());
    }

    #[test]
    fn ignores_single_satellite_doppler_change() {
        let sat_ids = gps_satellites(&[3, 7, 11, 19, 23, 29]);
        let previous_solution = solution_epoch(100_000.0, 0.0, 0.0);
        let current_solution = solution_epoch(100_000.001, 80.0 / SPEED_OF_LIGHT_MPS, 0.0);
        let previous_observations = observations_with_doppler(
            &sat_ids,
            &[-1_800.0, -1_000.0, -250.0, 500.0, 1_250.0, 2_000.0],
        );
        let current_observations = observations_with_doppler(
            &sat_ids,
            &[-1_800.0, -1_000.0, 250.0, 500.0, 1_250.0, 2_000.0],
        );

        assert!(detect_common_code_doppler_anomaly(
            Some(&previous_solution),
            Some(&previous_observations),
            &current_solution,
            &current_observations,
        )
        .is_none());
    }

    #[test]
    fn ignores_smooth_clock_evolution_without_common_step() {
        let sat_ids = gps_satellites(&[3, 7, 11, 19, 23, 29]);
        let previous_solution = solution_epoch(100_000.0, 0.0, 0.0);
        let dt_s = 0.001;
        let drift_s_per_s = 5.0e-6;
        let current_solution =
            solution_epoch(100_000.0 + dt_s, drift_s_per_s * dt_s, drift_s_per_s);
        let previous_observations = observations_with_doppler(
            &sat_ids,
            &[-1_800.0, -1_000.0, -250.0, 500.0, 1_250.0, 2_000.0],
        );

        assert!(detect_common_code_doppler_anomaly(
            Some(&previous_solution),
            Some(&previous_observations),
            &current_solution,
            &previous_observations,
        )
        .is_none());
    }

    fn gps_satellites(prns: &[u8]) -> Vec<SatId> {
        prns.iter().copied().map(|prn| SatId { constellation: Constellation::Gps, prn }).collect()
    }

    fn observations_with_doppler(
        sat_ids: &[SatId],
        doppler_hz: &[f64],
    ) -> Vec<PositionObservation> {
        sat_ids
            .iter()
            .zip(doppler_hz.iter().copied())
            .map(|(sat, doppler_hz)| PositionObservation {
                sat: *sat,
                pseudorange_m: 24_000_000.0,
                doppler_hz: Some(doppler_hz),
                doppler_var_hz2: Some(1.0),
                cn0_dbhz: 45.0,
                elevation_deg: Some(45.0),
                weight: 1.0,
                gps_receive_time: None,
                signal_timing: None,
                signal_id: None,
            })
            .collect()
    }

    fn solution_epoch(
        t_rx_s: f64,
        clock_bias_s: f64,
        clock_drift_s_per_s: f64,
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
            clock_bias_s: Seconds(clock_bias_s),
            clock_bias_m: Meters(clock_bias_s * SPEED_OF_LIGHT_MPS),
            clock_drift_s_per_s,
            pdop: 0.0,
            pre_fit_residual_rms_m: None,
            post_fit_residual_rms_m: None,
            rms_m: Meters(0.0),
            status: SolutionStatus::CodeOnly,
            quality: SolutionStatus::CodeOnly.quality_flag(),
            validity: SolutionValidity::Stable,
            valid: true,
            processing_ms: None,
            residuals: Vec::<NavResidual>::new(),
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
            model_version: 0,
            lifecycle_state: NavLifecycleState::CodeOnly,
            uncertainty_class: NavUncertaintyClass::Low,
            assumptions: None,
            refusal_class: None::<NavRefusalClass>,
            artifact_id: String::new(),
            source_observation_epoch_id: String::new(),
            explain_decision: String::new(),
            explain_reasons: Vec::new(),
            provenance: None,
            sat_count: 0,
            used_sat_count: 0,
            rejected_sat_count: 0,
            hdop: None,
            vdop: None,
            gdop: None,
            tdop: None,
            stability_signature: String::new(),
            stability_signature_version: 0,
        }
    }
}
