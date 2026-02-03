use std::collections::BTreeMap;

use bijux_gnss_core::{AmbiguityId, Constellation, ObsEpoch, ObsSatellite, ReceiverRole, SigId};
use bijux_gnss_nav::ecef_to_enu;

#[derive(Debug, Clone)]
pub struct BaselineConfig {
    pub base_ecef_m: Option<[f64; 3]>,
    pub rover_prior_ecef_m: Option<[f64; 3]>,
    pub align_tolerance_s: f64,
    pub ref_policy: RefSatPolicy,
}

impl Default for BaselineConfig {
    fn default() -> Self {
        Self {
            base_ecef_m: None,
            rover_prior_ecef_m: None,
            align_tolerance_s: 0.0005,
            ref_policy: RefSatPolicy::Global,
        }
    }
}

#[derive(Debug, Clone)]
pub struct EpochAligner {
    tolerance_s: f64,
    pub aligned: usize,
    pub dropped_base: usize,
    pub dropped_rover: usize,
    jitter_s: Vec<f64>,
}

#[derive(Debug, Clone, Copy)]
pub enum RefSatPolicy {
    Global,
    PerConstellation,
}

type SatKey = bijux_gnss_core::SatId;

impl EpochAligner {
    pub fn new(tolerance_s: f64) -> Self {
        Self {
            tolerance_s,
            aligned: 0,
            dropped_base: 0,
            dropped_rover: 0,
            jitter_s: Vec::new(),
        }
    }

    pub fn align(&mut self, base: &[ObsEpoch], rover: &[ObsEpoch]) -> Vec<(ObsEpoch, ObsEpoch)> {
        let mut out = Vec::new();
        let mut i = 0;
        let mut j = 0;
        while i < base.len() && j < rover.len() {
            let b = &base[i];
            let r = &rover[j];
            let dt = (b.t_rx_s - r.t_rx_s).abs();
            if dt <= self.tolerance_s {
                out.push((b.clone(), r.clone()));
                self.aligned += 1;
                self.jitter_s.push(dt);
                i += 1;
                j += 1;
            } else if b.t_rx_s < r.t_rx_s {
                self.dropped_base += 1;
                i += 1;
            } else {
                self.dropped_rover += 1;
                j += 1;
            }
        }
        out
    }

    pub fn report(&self, base_len: usize, rover_len: usize) -> AlignmentReport {
        let total = base_len.max(rover_len).max(1) as f64;
        let matched_pct = (self.aligned as f64) / total;
        AlignmentReport {
            base: AlignmentDiagnostic {
                role: ReceiverRole::Base,
                aligned: self.aligned,
                dropped: self.dropped_base,
            },
            rover: AlignmentDiagnostic {
                role: ReceiverRole::Rover,
                aligned: self.aligned,
                dropped: self.dropped_rover,
            },
            matched_pct,
            jitter: jitter_summary(&self.jitter_s),
        }
    }
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct SdObservation {
    pub sig: SigId,
    pub code_m: f64,
    pub phase_cycles: f64,
    pub doppler_hz: f64,
    pub variance_code: f64,
    pub variance_phase: f64,
    pub ambiguity_rover: AmbiguityId,
    pub ambiguity_base: AmbiguityId,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct DdObservation {
    pub sig: SigId,
    pub ref_sig: SigId,
    pub code_m: f64,
    pub phase_cycles: f64,
    pub doppler_hz: f64,
    pub variance_code: f64,
    pub variance_phase: f64,
    pub canceled: Vec<AmbiguityId>,
}

fn ambiguity_id(sat: &ObsSatellite) -> AmbiguityId {
    AmbiguityId {
        sig: sat.signal_id,
        signal: format!("{:?}", sat.metadata.signal.band),
    }
}

pub fn build_sd(base: &ObsEpoch, rover: &ObsEpoch) -> Vec<SdObservation> {
    let mut base_map: BTreeMap<SatKey, &ObsSatellite> = BTreeMap::new();
    for sat in &base.sats {
        base_map.insert(sat.signal_id.sat, sat);
    }
    let mut out = Vec::new();
    for sat in &rover.sats {
        let key = sat.signal_id.sat;
        if let Some(base_sat) = base_map.get(&key) {
            let rover_code_var = if sat.pseudorange_var_m2 > 0.0 {
                sat.pseudorange_var_m2
            } else {
                variance_from_cn0_elev(sat.cn0_dbhz, sat.elevation_deg, MeasurementKind::Code)
            };
            let base_code_var = if base_sat.pseudorange_var_m2 > 0.0 {
                base_sat.pseudorange_var_m2
            } else {
                variance_from_cn0_elev(
                    base_sat.cn0_dbhz,
                    base_sat.elevation_deg,
                    MeasurementKind::Code,
                )
            };
            let rover_phase_var = if sat.carrier_phase_var_cycles2 > 0.0 {
                sat.carrier_phase_var_cycles2
            } else {
                variance_from_cn0_elev(sat.cn0_dbhz, sat.elevation_deg, MeasurementKind::Phase)
            };
            let base_phase_var = if base_sat.carrier_phase_var_cycles2 > 0.0 {
                base_sat.carrier_phase_var_cycles2
            } else {
                variance_from_cn0_elev(
                    base_sat.cn0_dbhz,
                    base_sat.elevation_deg,
                    MeasurementKind::Phase,
                )
            };
            let variance_code = rover_code_var + base_code_var;
            let variance_phase = rover_phase_var + base_phase_var;
            out.push(SdObservation {
                sig: sat.signal_id,
                code_m: sat.pseudorange_m - base_sat.pseudorange_m,
                phase_cycles: sat.carrier_phase_cycles - base_sat.carrier_phase_cycles,
                doppler_hz: sat.doppler_hz - base_sat.doppler_hz,
                variance_code,
                variance_phase,
                ambiguity_rover: ambiguity_id(sat),
                ambiguity_base: ambiguity_id(base_sat),
            });
        }
    }
    out
}

pub fn choose_ref_sat(sd: &[SdObservation]) -> Option<SigId> {
    sd.iter()
        .min_by(|a, b| {
            a.variance_code
                .partial_cmp(&b.variance_code)
                .unwrap_or(std::cmp::Ordering::Equal)
        })
        .map(|s| s.sig)
}

pub fn choose_ref_sat_per_constellation(sd: &[SdObservation]) -> BTreeMap<Constellation, SigId> {
    let mut by_const: BTreeMap<Constellation, Vec<SdObservation>> = BTreeMap::new();
    for s in sd {
        by_const
            .entry(s.ambiguity_rover.sig.sat.constellation)
            .or_default()
            .push(s.clone());
    }
    let mut out = BTreeMap::new();
    for (constellation, items) in by_const {
        if let Some(sig) = choose_ref_sat(&items) {
            out.insert(constellation, sig);
        }
    }
    out
}

pub fn build_dd(sd: &[SdObservation], ref_sig: SigId) -> Vec<DdObservation> {
    let mut ref_sd = None;
    for s in sd {
        if s.sig == ref_sig {
            ref_sd = Some(s);
            break;
        }
    }
    let Some(ref_sd) = ref_sd else {
        return Vec::new();
    };
    let mut out = Vec::new();
    for s in sd {
        if s.sig == ref_sig {
            continue;
        }
        if s.sig.sat.constellation != ref_sd.sig.sat.constellation {
            continue;
        }
        let corr = 0.0;
        let code_var = s.variance_code + ref_sd.variance_code
            - 2.0 * corr * (s.variance_code * ref_sd.variance_code).sqrt();
        let phase_var = s.variance_phase + ref_sd.variance_phase
            - 2.0 * corr * (s.variance_phase * ref_sd.variance_phase).sqrt();
        out.push(DdObservation {
            sig: s.sig,
            ref_sig: ref_sd.sig,
            code_m: s.code_m - ref_sd.code_m,
            phase_cycles: s.phase_cycles - ref_sd.phase_cycles,
            doppler_hz: s.doppler_hz - ref_sd.doppler_hz,
            variance_code: code_var,
            variance_phase: phase_var,
            canceled: vec![
                s.ambiguity_rover.clone(),
                s.ambiguity_base.clone(),
                ref_sd.ambiguity_rover.clone(),
                ref_sd.ambiguity_base.clone(),
            ],
        });
    }
    out
}

pub fn build_dd_per_constellation(
    sd: &[SdObservation],
    refs: &BTreeMap<Constellation, SigId>,
) -> Vec<DdObservation> {
    let mut out = Vec::new();
    for (constellation, ref_sig) in refs {
        let subset: Vec<SdObservation> = sd
            .iter()
            .filter(|s| s.ambiguity_rover.sig.sat.constellation == *constellation)
            .cloned()
            .collect();
        out.extend(build_dd(&subset, *ref_sig));
    }
    out
}

#[derive(Debug, Clone)]
pub struct DdCovarianceModel {
    pub variance_code: f64,
    pub variance_phase: f64,
    pub inter_frequency_corr: Option<f64>,
}

pub fn dd_covariance(dd: &DdObservation) -> DdCovarianceModel {
    DdCovarianceModel {
        variance_code: dd.variance_code,
        variance_phase: dd.variance_phase,
        inter_frequency_corr: None,
    }
}

fn variance_from_cn0_elev(cn0_dbhz: f64, elevation_deg: Option<f64>, kind: MeasurementKind) -> f64 {
    let cn0_linear = 10.0_f64.powf(cn0_dbhz / 10.0).max(1.0);
    let elev = elevation_deg.unwrap_or(30.0).to_radians().sin().max(0.1);
    let base = match kind {
        MeasurementKind::Code => 10.0,
        MeasurementKind::Phase => 0.02,
    };
    let sigma = base / (cn0_linear.sqrt() * elev);
    sigma * sigma
}

#[derive(Debug, Clone, Copy)]
enum MeasurementKind {
    Code,
    Phase,
}

#[derive(Debug, Clone)]
pub struct InnovationDiagnostics {
    pub predicted_variance: f64,
    pub observed_variance: f64,
    pub scale_suggestion: f64,
}

pub fn innovation_diagnostics(residuals: &[f64], predicted_variance: f64) -> InnovationDiagnostics {
    let observed = if residuals.is_empty() {
        0.0
    } else {
        residuals.iter().map(|r| r * r).sum::<f64>() / residuals.len() as f64
    };
    let scale = if predicted_variance > 0.0 {
        (observed / predicted_variance).sqrt()
    } else {
        1.0
    };
    InnovationDiagnostics {
        predicted_variance,
        observed_variance: observed,
        scale_suggestion: scale,
    }
}

pub fn solve_baseline_dd(
    dd: &[DdObservation],
    base_ecef_m: [f64; 3],
    ephs: &[bijux_gnss_nav::GpsEphemeris],
    t_rx_s: f64,
) -> Option<BaselineSolution> {
    if dd.len() < 3 {
        return None;
    }
    let mut h = Vec::new();
    let mut v = Vec::new();
    for obs in dd {
        let eph = ephs.iter().find(|e| e.sat == obs.sig.sat)?;
        let eph_ref = ephs.iter().find(|e| e.sat == obs.ref_sig.sat)?;
        let sat = bijux_gnss_nav::sat_state_gps_l1ca(eph, t_rx_s, 0.0);
        let sat_ref = bijux_gnss_nav::sat_state_gps_l1ca(eph_ref, t_rx_s, 0.0);
        let u = los_unit(base_ecef_m, [sat.x_m, sat.y_m, sat.z_m]);
        let u_ref = los_unit(base_ecef_m, [sat_ref.x_m, sat_ref.y_m, sat_ref.z_m]);
        h.push([u_ref[0] - u[0], u_ref[1] - u[1], u_ref[2] - u[2]]);
        v.push(obs.code_m);
    }
    let (bx, by, bz, cov) = solve_3x3(&h, &v)?;
    let mut baseline = baseline_from_ecef(
        base_ecef_m,
        [
            base_ecef_m[0] + bx,
            base_ecef_m[1] + by,
            base_ecef_m[2] + bz,
        ],
    );
    baseline.covariance_m2 = Some(cov);
    Some(baseline)
}

fn los_unit(base: [f64; 3], sat: [f64; 3]) -> [f64; 3] {
    let dx = base[0] - sat[0];
    let dy = base[1] - sat[1];
    let dz = base[2] - sat[2];
    let r = (dx * dx + dy * dy + dz * dz).sqrt().max(1.0);
    [dx / r, dy / r, dz / r]
}

fn solve_3x3(h: &[[f64; 3]], v: &[f64]) -> Option<(f64, f64, f64, [[f64; 3]; 3])> {
    let mut n = [[0.0_f64; 3]; 3];
    let mut u = [0.0_f64; 3];
    for (i, row) in h.iter().enumerate() {
        let wi = 1.0_f64;
        for r in 0..3 {
            u[r] += row[r] * v[i] * wi;
            for c in 0..3 {
                n[r][c] += row[r] * row[c] * wi;
            }
        }
    }
    let inv = invert_3x3(n)?;
    let dx = inv[0][0] * u[0] + inv[0][1] * u[1] + inv[0][2] * u[2];
    let dy = inv[1][0] * u[0] + inv[1][1] * u[1] + inv[1][2] * u[2];
    let dz = inv[2][0] * u[0] + inv[2][1] * u[1] + inv[2][2] * u[2];
    Some((dx, dy, dz, inv))
}

fn invert_3x3(a: [[f64; 3]; 3]) -> Option<[[f64; 3]; 3]> {
    let det = a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
        - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
        + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
    if det.abs() < 1e-12 {
        return None;
    }
    let inv_det = 1.0 / det;
    let mut inv = [[0.0_f64; 3]; 3];
    inv[0][0] = (a[1][1] * a[2][2] - a[1][2] * a[2][1]) * inv_det;
    inv[0][1] = (a[0][2] * a[2][1] - a[0][1] * a[2][2]) * inv_det;
    inv[0][2] = (a[0][1] * a[1][2] - a[0][2] * a[1][1]) * inv_det;
    inv[1][0] = (a[1][2] * a[2][0] - a[1][0] * a[2][2]) * inv_det;
    inv[1][1] = (a[0][0] * a[2][2] - a[0][2] * a[2][0]) * inv_det;
    inv[1][2] = (a[0][2] * a[1][0] - a[0][0] * a[1][2]) * inv_det;
    inv[2][0] = (a[1][0] * a[2][1] - a[1][1] * a[2][0]) * inv_det;
    inv[2][1] = (a[0][1] * a[2][0] - a[0][0] * a[2][1]) * inv_det;
    inv[2][2] = (a[0][0] * a[1][1] - a[0][1] * a[1][0]) * inv_det;
    Some(inv)
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct AlignmentDiagnostic {
    pub role: ReceiverRole,
    pub aligned: usize,
    pub dropped: usize,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct AlignmentReport {
    pub base: AlignmentDiagnostic,
    pub rover: AlignmentDiagnostic,
    pub matched_pct: f64,
    pub jitter: JitterSummary,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct JitterSummary {
    pub mean_s: f64,
    pub p95_s: f64,
    pub max_s: f64,
    pub bin_edges_s: Vec<f64>,
    pub counts: Vec<usize>,
}

fn jitter_summary(samples: &[f64]) -> JitterSummary {
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
