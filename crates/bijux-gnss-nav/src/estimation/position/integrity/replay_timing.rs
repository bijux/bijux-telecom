#![allow(missing_docs)]

use std::collections::BTreeMap;

use bijux_gnss_core::api::NavSolutionEpoch;

use crate::estimation::position::solver::PositionObservation;

const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;
const MAX_OBSERVATION_INTERVAL_S: f64 = 0.1;
const COMMON_DELAY_STEP_THRESHOLD_M: f64 = 40.0;
const CENTERED_DELAY_RMS_THRESHOLD_M: f64 = 20.0;
const MAX_CENTERED_DELAY_THRESHOLD_M: f64 = 40.0;
const POSITIVE_DELAY_STEP_FLOOR_M: f64 = 10.0;
const MIN_MATCHED_SATELLITES: usize = 4;
const MIN_POSITIVE_STEP_FRACTION: f64 = 0.8;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ReplayTimingAnomaly {
    pub common_delay_step_m: f64,
    pub centered_delay_rms_m: f64,
    pub max_centered_delay_m: f64,
    pub matched_satellite_count: usize,
    pub positive_step_satellite_count: usize,
    pub clock_step_m: f64,
    pub common_delay_step_threshold_m: f64,
    pub centered_delay_rms_threshold_m: f64,
}

pub fn detect_replay_timing_anomaly(
    previous_solution: Option<&NavSolutionEpoch>,
    previous_observations: Option<&[PositionObservation]>,
    current_solution: &NavSolutionEpoch,
    current_observations: &[PositionObservation],
) -> Option<ReplayTimingAnomaly> {
    let previous_solution = previous_solution?;
    let previous_observations = previous_observations?;

    let dt_s = current_solution.t_rx_s.0 - previous_solution.t_rx_s.0;
    if !dt_s.is_finite() || dt_s <= 0.0 || dt_s > MAX_OBSERVATION_INTERVAL_S {
        return None;
    }

    let expected_clock_bias_s =
        previous_solution.clock_bias_s.0 + previous_solution.clock_drift_s_per_s * dt_s;
    let clock_step_m =
        (current_solution.clock_bias_s.0 - expected_clock_bias_s) * SPEED_OF_LIGHT_MPS;
    if !clock_step_m.is_finite() {
        return None;
    }

    let previous_pseudorange_by_sat = previous_observations
        .iter()
        .filter(|observation| observation.signal_timing.is_some())
        .map(|observation| (observation.sat, observation.pseudorange_m))
        .collect::<BTreeMap<_, _>>();
    let mut delay_steps_m = current_observations
        .iter()
        .filter(|observation| observation.signal_timing.is_some())
        .filter_map(|observation| {
            previous_pseudorange_by_sat
                .get(&observation.sat)
                .map(|previous_pseudorange_m| observation.pseudorange_m - previous_pseudorange_m)
        })
        .filter(|step_m| step_m.is_finite())
        .collect::<Vec<_>>();
    if delay_steps_m.len() < MIN_MATCHED_SATELLITES {
        return None;
    }

    let positive_step_satellite_count =
        delay_steps_m.iter().filter(|step_m| **step_m >= POSITIVE_DELAY_STEP_FLOOR_M).count();
    let positive_step_fraction = positive_step_satellite_count as f64 / delay_steps_m.len() as f64;
    if positive_step_fraction < MIN_POSITIVE_STEP_FRACTION {
        return None;
    }

    let common_delay_step_m = median(&mut delay_steps_m);
    if common_delay_step_m < COMMON_DELAY_STEP_THRESHOLD_M {
        return None;
    }

    let centered_delay_rms_m =
        (delay_steps_m.iter().map(|step_m| (step_m - common_delay_step_m).powi(2)).sum::<f64>()
            / delay_steps_m.len() as f64)
            .sqrt();
    let max_centered_delay_m =
        delay_steps_m.iter().map(|step_m| (step_m - common_delay_step_m).abs()).fold(0.0, f64::max);
    if centered_delay_rms_m < CENTERED_DELAY_RMS_THRESHOLD_M
        || max_centered_delay_m < MAX_CENTERED_DELAY_THRESHOLD_M
    {
        return None;
    }

    Some(ReplayTimingAnomaly {
        common_delay_step_m,
        centered_delay_rms_m,
        max_centered_delay_m,
        matched_satellite_count: delay_steps_m.len(),
        positive_step_satellite_count,
        clock_step_m,
        common_delay_step_threshold_m: COMMON_DELAY_STEP_THRESHOLD_M,
        centered_delay_rms_threshold_m: CENTERED_DELAY_RMS_THRESHOLD_M,
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

#[cfg(test)]
mod tests {
    use bijux_gnss_core::api::{
        Constellation, Epoch, GpsTime, Meters, NavLifecycleState, NavSolutionEpoch,
        NavUncertaintyClass, ObsSignalTiming, SatId, Seconds, SolutionStatus, SolutionValidity,
    };

    use super::*;

    #[test]
    fn detects_positive_delay_steps_with_non_common_spread() {
        let sat_ids = gps_satellites(&[3, 7, 11, 19, 23, 29]);
        let previous_solution = solution_epoch(100_000.0, 0.0, 0.0, sat_ids.len());
        let current_solution =
            solution_epoch(100_000.001, 60.0 / SPEED_OF_LIGHT_MPS, 0.0, sat_ids.len());
        let previous_observations = observations_with_pseudorange(
            &sat_ids,
            &[24_000_000.0, 24_001_000.0, 24_002_000.0, 24_003_000.0, 24_004_000.0, 24_005_000.0],
        );
        let current_observations = observations_with_pseudorange(
            &sat_ids,
            &[24_000_140.0, 24_001_110.0, 24_002_080.0, 24_003_030.0, 24_004_130.0, 24_005_020.0],
        );

        let anomaly = detect_replay_timing_anomaly(
            Some(&previous_solution),
            Some(&previous_observations),
            &current_solution,
            &current_observations,
        )
        .expect("replay-like delay spread must be detected");

        assert_eq!(anomaly.matched_satellite_count, sat_ids.len());
        assert_eq!(anomaly.positive_step_satellite_count, sat_ids.len());
        assert!(anomaly.centered_delay_rms_m >= anomaly.centered_delay_rms_threshold_m);
        assert!(anomaly.max_centered_delay_m >= MAX_CENTERED_DELAY_THRESHOLD_M);
    }

    #[test]
    fn ignores_uniform_delay_step() {
        let sat_ids = gps_satellites(&[3, 7, 11, 19, 23, 29]);
        let previous_solution = solution_epoch(100_000.0, 0.0, 0.0, sat_ids.len());
        let current_solution =
            solution_epoch(100_000.001, 85.0 / SPEED_OF_LIGHT_MPS, 0.0, sat_ids.len());
        let previous_observations = observations_with_pseudorange(
            &sat_ids,
            &[24_000_000.0, 24_001_000.0, 24_002_000.0, 24_003_000.0, 24_004_000.0, 24_005_000.0],
        );
        let current_observations = observations_with_pseudorange(
            &sat_ids,
            &[24_000_085.0, 24_001_085.0, 24_002_085.0, 24_003_085.0, 24_004_085.0, 24_005_085.0],
        );

        assert!(detect_replay_timing_anomaly(
            Some(&previous_solution),
            Some(&previous_observations),
            &current_solution,
            &current_observations,
        )
        .is_none());
    }

    #[test]
    fn ignores_single_satellite_delay_step() {
        let sat_ids = gps_satellites(&[3, 7, 11, 19, 23, 29]);
        let previous_solution = solution_epoch(100_000.0, 0.0, 0.0, sat_ids.len());
        let current_solution =
            solution_epoch(100_000.001, 50.0 / SPEED_OF_LIGHT_MPS, 0.0, sat_ids.len());
        let previous_observations = observations_with_pseudorange(
            &sat_ids,
            &[24_000_000.0, 24_001_000.0, 24_002_000.0, 24_003_000.0, 24_004_000.0, 24_005_000.0],
        );
        let current_observations = observations_with_pseudorange(
            &sat_ids,
            &[24_000_000.0, 24_001_000.0, 24_002_140.0, 24_003_000.0, 24_004_000.0, 24_005_000.0],
        );

        assert!(detect_replay_timing_anomaly(
            Some(&previous_solution),
            Some(&previous_observations),
            &current_solution,
            &current_observations,
        )
        .is_none());
    }

    fn gps_satellites(prns: &[u8]) -> Vec<SatId> {
        prns.iter().copied().map(|prn| SatId { constellation: Constellation::Gps, prn }).collect()
    }

    fn observations_with_pseudorange(
        sat_ids: &[SatId],
        pseudoranges_m: &[f64],
    ) -> Vec<PositionObservation> {
        sat_ids
            .iter()
            .zip(pseudoranges_m.iter().copied())
            .map(|(sat, pseudorange_m)| PositionObservation {
                sat: *sat,
                pseudorange_m,
                doppler_hz: None,
                doppler_var_hz2: None,
                cn0_dbhz: 45.0,
                elevation_deg: Some(45.0),
                weight: 1.0,
                gps_receive_time: None,
                signal_timing: Some(ObsSignalTiming {
                    signal_travel_time_s: Seconds(pseudorange_m / SPEED_OF_LIGHT_MPS),
                    transmit_gps_time: GpsTime {
                        week: 0,
                        tow_s: 100_000.0 - pseudorange_m / SPEED_OF_LIGHT_MPS,
                    },
                }),
                signal_id: None,
            })
            .collect()
    }

    fn solution_epoch(
        t_rx_s: f64,
        clock_bias_s: f64,
        clock_drift_s_per_s: f64,
        sat_count: usize,
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
            pdop: 1.0,
            pre_fit_residual_rms_m: None,
            post_fit_residual_rms_m: None,
            rms_m: Meters(1.0),
            status: SolutionStatus::CodeOnly,
            quality: SolutionStatus::CodeOnly.quality_flag(),
            validity: SolutionValidity::Stable,
            valid: true,
            processing_ms: None,
            residuals: Vec::new(),
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
            wls_solver_rank: None,
            wls_condition_number: None,
            ekf_whiteness_ratio: None,
            ekf_predicted_variance: None,
            ekf_observed_variance: None,
            integrity_hpl_m: None,
            integrity_vpl_m: None,
            model_version: 4,
            lifecycle_state: NavLifecycleState::CodeOnly,
            uncertainty_class: NavUncertaintyClass::Low,
            assumptions: None,
            refusal_class: None,
            artifact_id: String::new(),
            source_observation_epoch_id: String::new(),
            explain_decision: String::new(),
            explain_reasons: Vec::new(),
            provenance: None,
            sat_count,
            used_sat_count: sat_count,
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
