//! Reference trajectory validation helpers.

use serde::{Deserialize, Serialize};

use crate::geo::ecef_to_enu;
use crate::stats::lla_to_ecef;

/// Reference alignment policy.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub enum ReferenceAlign {
    /// Nearest reference epoch.
    Nearest,
    /// Linear interpolation between surrounding epochs.
    Linear,
}

/// Reference trajectory epoch.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ValidationReferenceEpoch {
    /// Epoch index.
    pub epoch_idx: u64,
    /// Receiver time tag in seconds (optional).
    #[serde(default)]
    pub t_rx_s: Option<f64>,
    /// Latitude in degrees.
    pub latitude_deg: f64,
    /// Longitude in degrees.
    pub longitude_deg: f64,
    /// Altitude in meters.
    pub altitude_m: f64,
    /// ECEF X (optional).
    #[serde(default)]
    pub ecef_x_m: Option<f64>,
    /// ECEF Y (optional).
    #[serde(default)]
    pub ecef_y_m: Option<f64>,
    /// ECEF Z (optional).
    #[serde(default)]
    pub ecef_z_m: Option<f64>,
    /// Optional velocity X (m/s).
    #[serde(default)]
    pub vel_x_mps: Option<f64>,
    /// Optional velocity Y (m/s).
    #[serde(default)]
    pub vel_y_mps: Option<f64>,
    /// Optional velocity Z (m/s).
    #[serde(default)]
    pub vel_z_mps: Option<f64>,
}

/// Summary stats for reference comparison.
#[derive(Debug, Serialize)]
pub struct ReferenceCompareStats {
    /// Number of matched epochs.
    pub count: usize,
    /// East RMS error (meters).
    pub east_rms_m: f64,
    /// North RMS error (meters).
    pub north_rms_m: f64,
    /// Up RMS error (meters).
    pub up_rms_m: f64,
    /// Horizontal RMS error (meters).
    pub horiz_rms_m: f64,
    /// Vertical RMS error (meters).
    pub vert_rms_m: f64,
    /// 3D RMS error (meters).
    pub error_3d_rms_m: f64,
}

/// Solution consistency report for sanity checks.
#[derive(Debug, Serialize)]
pub struct SolutionConsistencyReport {
    /// Position jump count.
    pub position_jump_count: usize,
    /// Clock jump count.
    pub clock_jump_count: usize,
    /// PDOP spike count.
    pub pdop_spike_count: usize,
    /// Warning messages.
    pub warnings: Vec<String>,
}

/// Convert reference epoch to ECEF coordinates.
pub fn reference_ecef(r: &ValidationReferenceEpoch) -> (f64, f64, f64) {
    if let (Some(x), Some(y), Some(z)) = (r.ecef_x_m, r.ecef_y_m, r.ecef_z_m) {
        (x, y, z)
    } else {
        lla_to_ecef(r.latitude_deg, r.longitude_deg, r.altitude_m)
    }
}

/// Align reference epochs by time to solution epochs.
pub fn align_reference_by_time(
    solutions: &[crate::api::NavSolutionEpoch],
    reference: &[ValidationReferenceEpoch],
    policy: ReferenceAlign,
) -> Vec<ValidationReferenceEpoch> {
    let mut out = Vec::new();
    let mut ref_sorted: Vec<_> =
        reference.iter().filter_map(|r| r.t_rx_s.map(|t| (t, r))).collect();
    ref_sorted.sort_by(|a, b| a.0.total_cmp(&b.0));
    for sol in solutions {
        let t = sol.t_rx_s.0;
        let mut best = None;
        for window in ref_sorted.windows(2) {
            if t >= window[0].0 && t <= window[1].0 {
                let (t0, r0) = window[0];
                let (t1, r1) = window[1];
                let alpha = if (t1 - t0).abs() < 1e-9 { 0.0 } else { (t - t0) / (t1 - t0) };
                let chosen = match policy {
                    ReferenceAlign::Nearest => {
                        if (t - t0).abs() <= (t1 - t).abs() {
                            r0.clone()
                        } else {
                            r1.clone()
                        }
                    }
                    ReferenceAlign::Linear => {
                        let (x0, y0, z0) = reference_ecef(r0);
                        let (x1, y1, z1) = reference_ecef(r1);
                        let x = x0 + alpha * (x1 - x0);
                        let y = y0 + alpha * (y1 - y0);
                        let z = z0 + alpha * (z1 - z0);
                        ValidationReferenceEpoch {
                            epoch_idx: sol.epoch.index,
                            t_rx_s: Some(t),
                            latitude_deg: r0.latitude_deg,
                            longitude_deg: r0.longitude_deg,
                            altitude_m: r0.altitude_m,
                            ecef_x_m: Some(x),
                            ecef_y_m: Some(y),
                            ecef_z_m: Some(z),
                            vel_x_mps: r0.vel_x_mps,
                            vel_y_mps: r0.vel_y_mps,
                            vel_z_mps: r0.vel_z_mps,
                        }
                    }
                };
                best = Some(chosen);
                break;
            }
        }
        if let Some(mut ref_epoch) = best {
            ref_epoch.epoch_idx = sol.epoch.index;
            out.push(ref_epoch);
        }
    }
    out
}

/// Compare solutions to a reference trajectory.
pub fn reference_compare(
    solutions: &[crate::api::NavSolutionEpoch],
    reference: &[ValidationReferenceEpoch],
) -> (Vec<String>, ReferenceCompareStats) {
    let mut ref_map = std::collections::BTreeMap::new();
    for r in reference {
        ref_map.insert(r.epoch_idx, r);
    }
    let mut rows = Vec::new();
    rows.push("epoch_idx,east_m,north_m,up_m,horiz_m,vert_m,error_3d_m".to_string());
    let mut east = Vec::new();
    let mut north = Vec::new();
    let mut up = Vec::new();
    let mut horiz = Vec::new();
    let mut vert = Vec::new();
    let mut error_3d = Vec::new();
    for sol in solutions {
        if let Some(r) = ref_map.get(&sol.epoch.index) {
            let (e, n, u) = ecef_to_enu(
                sol.ecef_x_m.0,
                sol.ecef_y_m.0,
                sol.ecef_z_m.0,
                r.latitude_deg,
                r.longitude_deg,
                r.altitude_m,
            );
            let h = (e * e + n * n).sqrt();
            let v = u.abs();
            let d3 = (h * h + u * u).sqrt();
            east.push(e);
            north.push(n);
            up.push(u);
            horiz.push(h);
            vert.push(v);
            error_3d.push(d3);
            rows.push(format!(
                "{},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4}",
                sol.epoch.index, e, n, u, h, v, d3
            ));
        }
    }
    let east_rms = rms(&east);
    let north_rms = rms(&north);
    let up_rms = rms(&up);
    let horiz_rms = rms(&horiz);
    let vert_rms = rms(&vert);
    let error_3d_rms = rms(&error_3d);
    (
        rows,
        ReferenceCompareStats {
            count: horiz.len(),
            east_rms_m: east_rms,
            north_rms_m: north_rms,
            up_rms_m: up_rms,
            horiz_rms_m: horiz_rms,
            vert_rms_m: vert_rms,
            error_3d_rms_m: error_3d_rms,
        },
    )
}

fn rms(values: &[f64]) -> f64 {
    if values.is_empty() {
        0.0
    } else {
        (values.iter().map(|value| value * value).sum::<f64>() / values.len() as f64).sqrt()
    }
}

/// Check solution consistency for obvious anomalies.
pub fn check_solution_consistency(
    solutions: &[crate::api::NavSolutionEpoch],
) -> SolutionConsistencyReport {
    let mut position_jump_count = 0;
    let mut clock_jump_count = 0;
    let mut pdop_spike_count = 0;
    let mut warnings = Vec::new();
    let mut prev: Option<&crate::api::NavSolutionEpoch> = None;
    let mut prev_pdop: Option<f64> = None;
    for sol in solutions {
        if let Some(prev_sol) = prev {
            let dx = sol.ecef_x_m.0 - prev_sol.ecef_x_m.0;
            let dy = sol.ecef_y_m.0 - prev_sol.ecef_y_m.0;
            let dz = sol.ecef_z_m.0 - prev_sol.ecef_z_m.0;
            let dist = (dx * dx + dy * dy + dz * dz).sqrt();
            if dist > 50.0 {
                position_jump_count += 1;
            }
            let clock_jump = (sol.clock_bias_s.0 - prev_sol.clock_bias_s.0).abs();
            if clock_jump > 1e-3 {
                clock_jump_count += 1;
            }
        }
        if let Some(prev_pdop) = prev_pdop {
            if sol.pdop > prev_pdop * 2.5 && sol.pdop > 6.0 {
                pdop_spike_count += 1;
            }
        }
        prev_pdop = Some(sol.pdop);
        prev = Some(sol);
    }
    if position_jump_count > 0 {
        warnings.push("position jumps detected".to_string());
    }
    if clock_jump_count > 0 {
        warnings.push("clock jumps detected".to_string());
    }
    if pdop_spike_count > 0 {
        warnings.push("pdop spikes detected".to_string());
    }
    SolutionConsistencyReport { position_jump_count, clock_jump_count, pdop_spike_count, warnings }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::api::{
        Epoch, Meters, NavLifecycleState, NavSolutionEpoch, NavUncertaintyClass,
        ReceiverSampleTrace, Seconds, SolutionStatus, SolutionValidity,
        NAV_OUTPUT_STABILITY_SIGNATURE_VERSION, NAV_SOLUTION_MODEL_VERSION,
    };
    use crate::stats::lla_to_ecef;

    fn sample_solution(ecef_x_m: f64, ecef_y_m: f64, ecef_z_m: f64) -> NavSolutionEpoch {
        NavSolutionEpoch {
            epoch: Epoch { index: 7 },
            t_rx_s: Seconds(7.0),
            source_time: ReceiverSampleTrace::from_sample_index(7, 1.0),
            ecef_x_m: Meters(ecef_x_m),
            ecef_y_m: Meters(ecef_y_m),
            ecef_z_m: Meters(ecef_z_m),
            position_covariance_ecef_m2: None,
            latitude_deg: 0.0,
            longitude_deg: 0.0,
            altitude_m: Meters(0.0),
            clock_bias_s: Seconds(0.0),
            clock_bias_m: Meters(0.0),
            clock_drift_s_per_s: 0.0,
            pdop: 1.0,
            pre_fit_residual_rms_m: Some(Meters(0.0)),
            post_fit_residual_rms_m: Some(Meters(0.0)),
            rms_m: Meters(0.0),
            status: SolutionStatus::Converged,
            quality: SolutionStatus::Converged.quality_flag(),
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
            assumptions: None,
            refusal_class: None,
            artifact_id: "nav-epoch-0000000007-reference".to_string(),
            source_observation_epoch_id: "obs-epoch-0000000007-reference".to_string(),
            explain_decision: "accepted".to_string(),
            explain_reasons: vec!["reference_fixture".to_string()],
            provenance: None,
            sat_count: 4,
            used_sat_count: 4,
            rejected_sat_count: 0,
            hdop: Some(1.0),
            vdop: Some(1.0),
            gdop: Some(1.0),
            tdop: Some(0.5),
            stability_signature: "navsig:v2:reference".to_string(),
            stability_signature_version: NAV_OUTPUT_STABILITY_SIGNATURE_VERSION,
        }
    }

    #[test]
    fn reference_compare_reports_enu_components() {
        let (x_ref, y_ref, z_ref) = lla_to_ecef(0.0, 0.0, 0.0);
        let solution = sample_solution(x_ref + 3.0, y_ref + 1.0, z_ref + 2.0);
        let reference = ValidationReferenceEpoch {
            epoch_idx: 7,
            t_rx_s: Some(7.0),
            latitude_deg: 0.0,
            longitude_deg: 0.0,
            altitude_m: 0.0,
            ecef_x_m: Some(x_ref),
            ecef_y_m: Some(y_ref),
            ecef_z_m: Some(z_ref),
            vel_x_mps: None,
            vel_y_mps: None,
            vel_z_mps: None,
        };

        let (rows, stats) = reference_compare(&[solution], &[reference]);

        assert_eq!(rows[0], "epoch_idx,east_m,north_m,up_m,horiz_m,vert_m,error_3d_m");
        assert_eq!(rows[1], "7,1.0000,2.0000,3.0000,2.2361,3.0000,3.7417");
        assert_eq!(stats.count, 1);
        assert!((stats.east_rms_m - 1.0).abs() < 1.0e-12);
        assert!((stats.north_rms_m - 2.0).abs() < 1.0e-12);
        assert!((stats.up_rms_m - 3.0).abs() < 1.0e-12);
        assert!((stats.horiz_rms_m - 5.0_f64.sqrt()).abs() < 1.0e-12);
        assert!((stats.vert_rms_m - 3.0).abs() < 1.0e-12);
        assert!((stats.error_3d_rms_m - 14.0_f64.sqrt()).abs() < 1.0e-12);
    }
}
