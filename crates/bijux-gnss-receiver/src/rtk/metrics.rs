#![allow(missing_docs)]

use bijux_gnss_nav::api::{
    ecef_to_enu, rtk_double_difference_residual_metrics, rtk_single_difference_residual_metrics,
};

use super::core::{solve_float_baseline_dd, DdObservation, SdObservation, SolutionSeparation};

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct JitterSummary {
    pub mean_s: f64,
    pub p95_s: f64,
    pub max_s: f64,
    pub bin_edges_s: Vec<f64>,
    pub counts: Vec<usize>,
}

pub fn jitter_summary(samples: &[f64]) -> JitterSummary {
    let mut data = samples.to_vec();
    data.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let mean = if data.is_empty() { 0.0 } else { data.iter().sum::<f64>() / data.len() as f64 };
    let p95 = if data.is_empty() {
        0.0
    } else {
        let idx = ((data.len() as f64) * 0.95).floor() as usize;
        data[idx.min(data.len() - 1)]
    };
    let max = data.last().copied().unwrap_or(0.0);
    let bin_edges = vec![0.0, 1e-5, 5e-5, 1e-4, 2e-4, 5e-4, 1e-3, 2e-3, 5e-3, 1e-2];
    let mut counts = vec![0usize; bin_edges.len() - 1];
    for &val in samples {
        for i in 0..counts.len() {
            if val >= bin_edges[i] && val < bin_edges[i + 1] {
                counts[i] += 1;
                break;
            }
        }
    }
    JitterSummary { mean_s: mean, p95_s: p95, max_s: max, bin_edges_s: bin_edges, counts }
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct BaselineSolution {
    pub enu_m: [f64; 3],
    pub covariance_m2: Option<[[f64; 3]; 3]>,
    pub fixed: bool,
}

impl bijux_gnss_core::api::ArtifactPayloadValidate for BaselineSolution {
    fn validate_payload(&self) -> Vec<bijux_gnss_core::api::DiagnosticEvent> {
        let mut events = Vec::new();
        if !self.enu_m.iter().all(|v| v.is_finite()) {
            events.push(bijux_gnss_core::api::DiagnosticEvent::new(
                bijux_gnss_core::api::DiagnosticSeverity::Error,
                "RTK_BASELINE_NUMERIC_INVALID",
                "baseline ENU contains NaN/Inf",
            ));
        }
        events
    }
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct RtkBaselineQuality {
    pub epoch_idx: u64,
    pub fixed: bool,
    pub sigma_e: f64,
    pub sigma_n: f64,
    pub sigma_u: f64,
    pub used_sats: usize,
    pub residual_rms_m: f64,
    pub predicted_rms_m: f64,
    pub hpl_m: f64,
    pub vpl_m: f64,
    pub separation_sig: Option<String>,
    pub separation_max_m: Option<f64>,
}

impl bijux_gnss_core::api::ArtifactPayloadValidate for RtkBaselineQuality {
    fn validate_payload(&self) -> Vec<bijux_gnss_core::api::DiagnosticEvent> {
        let mut events = Vec::new();
        if !self.sigma_e.is_finite()
            || !self.sigma_n.is_finite()
            || !self.sigma_u.is_finite()
            || !self.residual_rms_m.is_finite()
            || !self.predicted_rms_m.is_finite()
        {
            events.push(bijux_gnss_core::api::DiagnosticEvent::new(
                bijux_gnss_core::api::DiagnosticSeverity::Error,
                "RTK_QUALITY_NUMERIC_INVALID",
                "baseline quality contains NaN/Inf",
            ));
        }
        events
    }
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct RtkPrecision {
    pub epoch_idx: u64,
    pub fix_accepted: bool,
    pub ratio: Option<f64>,
    pub fixed_count: usize,
    pub ref_changed: bool,
    pub slip_count: usize,
}

impl bijux_gnss_core::api::ArtifactPayloadValidate for RtkPrecision {
    fn validate_payload(&self) -> Vec<bijux_gnss_core::api::DiagnosticEvent> {
        let mut events = Vec::new();
        if let Some(ratio) = self.ratio {
            if !ratio.is_finite() {
                events.push(bijux_gnss_core::api::DiagnosticEvent::new(
                    bijux_gnss_core::api::DiagnosticSeverity::Error,
                    "RTK_PRECISION_NUMERIC_INVALID",
                    "rtk precision ratio contains NaN/Inf",
                ));
            }
        }
        events
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RtkFixedBaselineGuardPolicy {
    pub min_cn0_dbhz: f64,
    pub max_code_variance_m2: f64,
    pub max_phase_variance_cycles2: f64,
    pub max_residual_scale: f64,
    pub max_solution_separation_m: f64,
}

impl Default for RtkFixedBaselineGuardPolicy {
    fn default() -> Self {
        Self {
            min_cn0_dbhz: 35.0,
            max_code_variance_m2: 1.0,
            max_phase_variance_cycles2: 0.02,
            max_residual_scale: 3.0,
            max_solution_separation_m: 0.75,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct RtkFixedBaselineGuardDecision {
    pub accepted: bool,
    pub reasons: Vec<String>,
    pub residual_scale: Option<f64>,
    pub max_solution_separation_m: Option<f64>,
    pub multipath_suspect: bool,
    pub weak_observation_count: usize,
    pub noisy_observation_count: usize,
}

pub fn baseline_from_ecef(base_ecef_m: [f64; 3], rover_ecef_m: [f64; 3]) -> BaselineSolution {
    let (lat, lon, alt) =
        bijux_gnss_nav::api::ecef_to_geodetic(base_ecef_m[0], base_ecef_m[1], base_ecef_m[2]);
    let (e, n, u) = ecef_to_enu(rover_ecef_m[0], rover_ecef_m[1], rover_ecef_m[2], lat, lon, alt);
    BaselineSolution { enu_m: [e, n, u], covariance_m2: None, fixed: false }
}

pub fn enu_to_ecef(base_ecef_m: [f64; 3], enu_m: [f64; 3]) -> [f64; 3] {
    let (lat, lon, _alt) =
        bijux_gnss_nav::api::ecef_to_geodetic(base_ecef_m[0], base_ecef_m[1], base_ecef_m[2]);
    let (sin_lat, cos_lat) = lat.to_radians().sin_cos();
    let (sin_lon, cos_lon) = lon.to_radians().sin_cos();
    let e = enu_m[0];
    let n = enu_m[1];
    let u = enu_m[2];
    let dx = -sin_lon * e - sin_lat * cos_lon * n + cos_lat * cos_lon * u;
    let dy = cos_lon * e - sin_lat * sin_lon * n + cos_lat * sin_lon * u;
    let dz = cos_lat * n + sin_lat * u;
    [base_ecef_m[0] + dx, base_ecef_m[1] + dy, base_ecef_m[2] + dz]
}

pub fn dd_residual_metrics(
    dd: &[DdObservation],
    base_ecef_m: [f64; 3],
    rover_enu_m: [f64; 3],
    ephs: &[bijux_gnss_nav::api::GpsEphemeris],
    t_rx_s: f64,
) -> Option<(f64, f64, usize)> {
    let metrics =
        rtk_double_difference_residual_metrics(dd, base_ecef_m, rover_enu_m, ephs, t_rx_s)?;
    Some((metrics.residual_rms_m, metrics.predicted_rms_m, metrics.used_observations))
}

pub fn sd_residual_metrics(
    sd: &[SdObservation],
    base_ecef_m: [f64; 3],
    rover_ecef_m: [f64; 3],
    ephs: &[bijux_gnss_nav::api::GpsEphemeris],
    base_t_rx_s: f64,
    rover_t_rx_s: f64,
) -> Option<(f64, f64, usize)> {
    let metrics = rtk_single_difference_residual_metrics(
        sd,
        base_ecef_m,
        rover_ecef_m,
        ephs,
        base_t_rx_s,
        rover_t_rx_s,
    )?;
    Some((metrics.residual_rms_m, metrics.predicted_rms_m, metrics.used_observations))
}

pub fn solution_separation(
    dd: &[DdObservation],
    base_ecef_m: [f64; 3],
    ephs: &[bijux_gnss_nav::api::GpsEphemeris],
    t_rx_s: f64,
) -> Option<Vec<SolutionSeparation>> {
    let full = solve_float_baseline_dd(dd, base_ecef_m, ephs, t_rx_s)?;
    if dd.len() < 2 {
        return None;
    }
    let mut out = Vec::new();
    for i in 0..dd.len() {
        let mut subset = dd.to_vec();
        let removed = subset.remove(i);
        if let Some(sol) = solve_float_baseline_dd(&subset, base_ecef_m, ephs, t_rx_s) {
            let de = sol.enu_m[0] - full.enu_m[0];
            let dn = sol.enu_m[1] - full.enu_m[1];
            let du = sol.enu_m[2] - full.enu_m[2];
            let delta = (de * de + dn * dn + du * du).sqrt();
            out.push(SolutionSeparation { sig: removed.sig, delta_enu_m: delta });
        }
    }
    if out.is_empty() {
        None
    } else {
        Some(out)
    }
}

pub fn evaluate_rtk_fixed_baseline_guard(
    dd: &[DdObservation],
    base_ecef_m: [f64; 3],
    baseline: &BaselineSolution,
    ephs: &[bijux_gnss_nav::api::GpsEphemeris],
    t_rx_s: f64,
    policy: RtkFixedBaselineGuardPolicy,
) -> RtkFixedBaselineGuardDecision {
    let multipath_suspect = dd.iter().any(|observation| observation.multipath_suspect);
    let weak_observation_count =
        dd.iter().filter(|observation| observation.min_cn0_dbhz < policy.min_cn0_dbhz).count();
    let noisy_observation_count = dd
        .iter()
        .filter(|observation| {
            observation.code_variance_m2 > policy.max_code_variance_m2
                || observation.phase_variance_cycles2 > policy.max_phase_variance_cycles2
        })
        .count();
    let residual_scale = dd_residual_metrics(dd, base_ecef_m, baseline.enu_m, ephs, t_rx_s)
        .and_then(|(residual_rms_m, predicted_rms_m, _used_observations)| {
            if predicted_rms_m > 0.0 {
                Some(residual_rms_m / predicted_rms_m)
            } else {
                None
            }
        });
    let max_solution_separation_m =
        solution_separation(dd, base_ecef_m, ephs, t_rx_s).and_then(|separation| {
            separation
                .iter()
                .map(|item| item.delta_enu_m)
                .max_by(|left, right| left.total_cmp(right))
        });

    let mut reasons = Vec::new();
    if multipath_suspect {
        reasons.push("multipath_suspect".to_string());
    }
    if weak_observation_count > 0 {
        reasons.push("low_cn0".to_string());
    }
    if noisy_observation_count > 0 {
        reasons.push("high_observation_variance".to_string());
    }
    if residual_scale.is_some_and(|scale| scale > policy.max_residual_scale) {
        reasons.push("residual_scale".to_string());
    }
    if max_solution_separation_m
        .is_some_and(|separation_m| separation_m > policy.max_solution_separation_m)
    {
        reasons.push("solution_separation".to_string());
    }

    RtkFixedBaselineGuardDecision {
        accepted: reasons.is_empty(),
        reasons,
        residual_scale,
        max_solution_separation_m,
        multipath_suspect,
        weak_observation_count,
        noisy_observation_count,
    }
}

pub fn apply_fix_hold(mut baseline: BaselineSolution, fixed: bool) -> BaselineSolution {
    baseline.fixed = fixed;
    if fixed {
        let cov =
            baseline.covariance_m2.unwrap_or([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]);
        let mut scaled = cov;
        for row in &mut scaled {
            for val in row.iter_mut() {
                *val *= 0.25;
            }
        }
        baseline.covariance_m2 = Some(scaled);
    }
    baseline
}
