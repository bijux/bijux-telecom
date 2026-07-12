#![allow(missing_docs)]

use std::collections::BTreeMap;

use bijux_gnss_core::api::{NavResidual, NavSolutionEpoch, SatId};

const MIN_SHARED_SATELLITES: usize = 5;
const MIN_ANOMALY_DEVIATION_M: f64 = 40.0;
const MAX_PEER_RMS_DELTA_M: f64 = 12.5;
const MIN_ISOLATION_RATIO: f64 = 2.5;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SatelliteClockAnomaly {
    pub sat: SatId,
    pub residual_delta_m: f64,
    pub peer_median_delta_m: f64,
    pub peer_rms_delta_m: f64,
}

#[derive(Debug, Clone, Copy)]
struct ResidualDelta {
    sat: SatId,
    delta_m: f64,
    deviation_m: f64,
}

pub fn detect_satellite_clock_anomaly(
    previous: &NavSolutionEpoch,
    current: &NavSolutionEpoch,
) -> Option<SatelliteClockAnomaly> {
    if !previous.valid || !current.valid {
        return None;
    }

    let previous_residuals = residual_map(&previous.residuals);
    let current_residuals = residual_map(&current.residuals);
    let mut shared_deltas = previous_residuals
        .into_iter()
        .filter_map(|(sat, previous_residual_m)| {
            current_residuals
                .get(&sat)
                .copied()
                .map(|current_residual_m| (sat, current_residual_m - previous_residual_m))
        })
        .collect::<Vec<_>>();
    if shared_deltas.len() < MIN_SHARED_SATELLITES {
        return None;
    }

    shared_deltas.sort_by(|left, right| left.1.total_cmp(&right.1));
    let peer_median_delta_m = median(shared_deltas.iter().map(|(_, delta_m)| *delta_m).collect());
    let mut ranked_deltas = shared_deltas
        .into_iter()
        .map(|(sat, delta_m)| ResidualDelta {
            sat,
            delta_m,
            deviation_m: (delta_m - peer_median_delta_m).abs(),
        })
        .collect::<Vec<_>>();
    ranked_deltas.sort_by(|left, right| right.deviation_m.total_cmp(&left.deviation_m));

    let best = ranked_deltas.first().copied()?;
    let second_deviation_m =
        ranked_deltas.get(1).map(|candidate| candidate.deviation_m).unwrap_or(0.0);
    let peer_rms_delta_m = rms(
        ranked_deltas
            .iter()
            .skip(1)
            .map(|candidate| candidate.deviation_m)
            .collect(),
    );

    (best.deviation_m >= MIN_ANOMALY_DEVIATION_M
        && peer_rms_delta_m <= MAX_PEER_RMS_DELTA_M
        && (second_deviation_m == 0.0
            || best.deviation_m / second_deviation_m.max(1.0) >= MIN_ISOLATION_RATIO))
    .then_some(SatelliteClockAnomaly {
        sat: best.sat,
        residual_delta_m: best.delta_m,
        peer_median_delta_m,
        peer_rms_delta_m,
    })
}

fn residual_map(residuals: &[NavResidual]) -> BTreeMap<SatId, f64> {
    residuals
        .iter()
        .filter(|residual| !residual.rejected && residual.residual_m.0.is_finite())
        .map(|residual| (residual.sat, residual.residual_m.0))
        .collect()
}

fn median(mut values: Vec<f64>) -> f64 {
    if values.is_empty() {
        return 0.0;
    }
    values.sort_by(f64::total_cmp);
    let midpoint = values.len() / 2;
    if values.len() % 2 == 0 {
        (values[midpoint - 1] + values[midpoint]) * 0.5
    } else {
        values[midpoint]
    }
}

fn rms(values: Vec<f64>) -> f64 {
    if values.is_empty() {
        return 0.0;
    }
    let mean_square = values.iter().map(|value| value * value).sum::<f64>() / values.len() as f64;
    mean_square.sqrt()
}

#[cfg(test)]
mod tests {
    use bijux_gnss_core::api::{
        Epoch, Meters, NavAssumptions, NavConstellationResidualRms, NavLifecycleState,
        NavProvenance, NavQualityFlag, NavRefusalClass, NavUncertaintyClass,
        ReceiverSampleTrace, SolutionStatus, SolutionValidity,
        NAV_OUTPUT_STABILITY_SIGNATURE_VERSION, NAV_SOLUTION_MODEL_VERSION,
    };

    use super::*;

    #[test]
    fn detects_isolated_satellite_residual_step() {
        let previous = sample_solution(&[(3, 0.5), (7, -0.5), (11, 0.2), (19, -0.2), (23, 0.7), (29, 0.1)]);
        let current = sample_solution(&[(3, 1.0), (7, 0.0), (11, 0.8), (19, 0.3), (23, 1.4), (29, 82.0)]);

        let anomaly =
            detect_satellite_clock_anomaly(&previous, &current).expect("isolated step should classify");

        assert_eq!(anomaly.sat.prn, 29);
        assert!(anomaly.residual_delta_m > 70.0, "{anomaly:?}");
        assert!(anomaly.peer_rms_delta_m < 5.0, "{anomaly:?}");
    }

    #[test]
    fn ignores_common_residual_shift_consistent_with_motion() {
        let previous = sample_solution(&[(3, -1.0), (7, -0.5), (11, 0.0), (19, 0.5), (23, 1.0), (29, 1.5)]);
        let current = sample_solution(&[(3, 69.0), (7, 69.5), (11, 70.0), (19, 70.5), (23, 71.0), (29, 71.5)]);

        assert!(detect_satellite_clock_anomaly(&previous, &current).is_none());
    }

    fn sample_solution(residuals: &[(u8, f64)]) -> NavSolutionEpoch {
        let satellites_used = residuals
            .iter()
            .map(|(prn, _)| SatId {
                constellation: bijux_gnss_core::api::Constellation::Gps,
                prn: *prn,
            })
            .collect::<Vec<_>>();
        NavSolutionEpoch {
            epoch: Epoch { index: 0 },
            t_rx_s: bijux_gnss_core::api::Seconds(100_000.0),
            source_time: ReceiverSampleTrace::from_sample_index(0, 1_023_000.0),
            ecef_x_m: Meters(0.0),
            ecef_y_m: Meters(0.0),
            ecef_z_m: Meters(0.0),
            position_covariance_ecef_m2: None,
            latitude_deg: 0.0,
            longitude_deg: 0.0,
            altitude_m: Meters(0.0),
            clock_bias_s: bijux_gnss_core::api::Seconds(0.0),
            clock_bias_m: Meters(0.0),
            clock_drift_s_per_s: 0.0,
            pdop: 1.0,
            pre_fit_residual_rms_m: None,
            post_fit_residual_rms_m: None,
            rms_m: Meters(1.0),
            status: SolutionStatus::Converged,
            quality: NavQualityFlag::Float,
            validity: SolutionValidity::Stable,
            valid: true,
            processing_ms: None,
            residuals: residuals
                .iter()
                .map(|(prn, residual_m)| NavResidual {
                    sat: SatId {
                        constellation: bijux_gnss_core::api::Constellation::Gps,
                        prn: *prn,
                    },
                    residual_m: Meters(*residual_m),
                    rejected: false,
                    weight: Some(1.0),
                    reject_reason: None,
                })
                .collect(),
            constellation_residual_rms: Vec::<NavConstellationResidualRms>::new(),
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
            model_version: NAV_SOLUTION_MODEL_VERSION,
            lifecycle_state: NavLifecycleState::Converged,
            uncertainty_class: NavUncertaintyClass::Low,
            assumptions: Some(NavAssumptions {
                time_system: "gps".to_string(),
                reference_frame: "ecef_wgs84".to_string(),
                clock_model: "receiver_clock_bias_drift_linear".to_string(),
                ephemeris_source: "synthetic".to_string(),
                frame_decode_mode: "synthetic".to_string(),
                ephemeris_completeness: "complete".to_string(),
                ephemeris_count: residuals.len(),
            }),
            refusal_class: Option::<NavRefusalClass>::None,
            artifact_id: String::new(),
            source_observation_epoch_id: String::new(),
            explain_decision: String::new(),
            explain_reasons: Vec::new(),
            provenance: Some(NavProvenance {
                solver_family: "wls".to_string(),
                weighting_mode: "uniform".to_string(),
                robust_solver: false,
                raim_enabled: true,
                satellites_used,
            }),
            sat_count: residuals.len(),
            used_sat_count: residuals.len(),
            rejected_sat_count: 0,
            hdop: None,
            vdop: None,
            gdop: None,
            tdop: None,
            stability_signature: String::new(),
            stability_signature_version: NAV_OUTPUT_STABILITY_SIGNATURE_VERSION,
        }
    }
}
