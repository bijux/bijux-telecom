#![allow(missing_docs)]

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
    let mean = if data.is_empty() {
        0.0
    } else {
        data.iter().sum::<f64>() / data.len() as f64
    };
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
    JitterSummary {
        mean_s: mean,
        p95_s: p95,
        max_s: max,
        bin_edges_s: bin_edges,
        counts,
    }
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct BaselineSolution {
    pub enu_m: [f64; 3],
    pub covariance_m2: Option<[[f64; 3]; 3]>,
    pub fixed: bool,
}

pub fn baseline_from_ecef(base_ecef_m: [f64; 3], rover_ecef_m: [f64; 3]) -> BaselineSolution {
    let (lat, lon, alt) =
        bijux_gnss_nav::ecef_to_geodetic(base_ecef_m[0], base_ecef_m[1], base_ecef_m[2]);
    let (e, n, u) = ecef_to_enu(
        rover_ecef_m[0],
        rover_ecef_m[1],
        rover_ecef_m[2],
        lat,
        lon,
        alt,
    );
    BaselineSolution {
        enu_m: [e, n, u],
        covariance_m2: None,
        fixed: false,
    }
}

pub fn enu_to_ecef(base_ecef_m: [f64; 3], enu_m: [f64; 3]) -> [f64; 3] {
    let (lat, lon, _alt) =
        bijux_gnss_nav::ecef_to_geodetic(base_ecef_m[0], base_ecef_m[1], base_ecef_m[2]);
    let (sin_lat, cos_lat) = lat.to_radians().sin_cos();
    let (sin_lon, cos_lon) = lon.to_radians().sin_cos();
    let e = enu_m[0];
    let n = enu_m[1];
    let u = enu_m[2];
    let dx = -sin_lon * e - sin_lat * cos_lon * n + cos_lat * cos_lon * u;
    let dy = cos_lon * e - sin_lat * sin_lon * n + cos_lat * sin_lon * u;
    let dz = cos_lat * n + sin_lat * u;
    [
        base_ecef_m[0] + dx,
        base_ecef_m[1] + dy,
        base_ecef_m[2] + dz,
    ]
}

pub fn dd_residual_metrics(
    dd: &[DdObservation],
    base_ecef_m: [f64; 3],
    rover_enu_m: [f64; 3],
    ephs: &[bijux_gnss_nav::GpsEphemeris],
    t_rx_s: f64,
) -> Option<(f64, f64, usize)> {
    if dd.is_empty() {
        return None;
    }
    let rover_ecef = enu_to_ecef(base_ecef_m, rover_enu_m);
    let baseline_vec = [
        rover_ecef[0] - base_ecef_m[0],
        rover_ecef[1] - base_ecef_m[1],
        rover_ecef[2] - base_ecef_m[2],
    ];
    let mut residuals = Vec::new();
    let mut predicted_vars = Vec::new();
    for obs in dd {
        let eph = ephs.iter().find(|e| e.sat == obs.sig.sat)?;
        let eph_ref = ephs.iter().find(|e| e.sat == obs.ref_sig.sat)?;
        let sat = bijux_gnss_nav::sat_state_gps_l1ca(eph, t_rx_s, 0.0);
        let sat_ref = bijux_gnss_nav::sat_state_gps_l1ca(eph_ref, t_rx_s, 0.0);
        let u = los_unit(base_ecef_m, [sat.x_m, sat.y_m, sat.z_m]);
        let u_ref = los_unit(base_ecef_m, [sat_ref.x_m, sat_ref.y_m, sat_ref.z_m]);
        let h = [u_ref[0] - u[0], u_ref[1] - u[1], u_ref[2] - u[2]];
        let pred = h[0] * baseline_vec[0] + h[1] * baseline_vec[1] + h[2] * baseline_vec[2];
        residuals.push(obs.code_m - pred);
        predicted_vars.push(obs.variance_code.max(1e-6));
    }
    let rms_obs = (residuals.iter().map(|v| v * v).sum::<f64>() / residuals.len() as f64).sqrt();
    let rms_pred = (predicted_vars.iter().sum::<f64>() / predicted_vars.len() as f64).sqrt();
    Some((rms_obs, rms_pred, residuals.len()))
}

pub fn solution_separation(
    dd: &[DdObservation],
    base_ecef_m: [f64; 3],
    ephs: &[bijux_gnss_nav::GpsEphemeris],
    t_rx_s: f64,
) -> Option<Vec<SolutionSeparation>> {
    let full = solve_baseline_dd(dd, base_ecef_m, ephs, t_rx_s)?;
    if dd.len() < 2 {
        return None;
    }
    let mut out = Vec::new();
    for i in 0..dd.len() {
        let mut subset = dd.to_vec();
        let removed = subset.remove(i);
        if let Some(sol) = solve_baseline_dd(&subset, base_ecef_m, ephs, t_rx_s) {
            let de = sol.enu_m[0] - full.enu_m[0];
            let dn = sol.enu_m[1] - full.enu_m[1];
            let du = sol.enu_m[2] - full.enu_m[2];
            let delta = (de * de + dn * dn + du * du).sqrt();
            out.push(SolutionSeparation {
                sig: removed.sig,
                delta_enu_m: delta,
            });
        }
    }
    if out.is_empty() {
        None
    } else {
        Some(out)
    }
}

pub fn apply_fix_hold(mut baseline: BaselineSolution, fixed: bool) -> BaselineSolution {
    baseline.fixed = fixed;
    if fixed {
        let cov =
            baseline
                .covariance_m2
                .unwrap_or([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]);
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
use bijux_gnss_nav::ecef_to_enu;

use super::core::{los_unit, solve_baseline_dd, DdObservation, SolutionSeparation};
