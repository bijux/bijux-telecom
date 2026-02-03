use std::collections::BTreeMap;

use bijux_gnss_core::{AmbiguityId, Constellation, ObsEpoch, ObsSatellite, ReceiverRole};
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
}

#[derive(Debug, Clone, Copy)]
pub enum RefSatPolicy {
    Global,
    PerConstellation,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct SatKey {
    pub constellation: Constellation,
    pub prn: u8,
}

impl EpochAligner {
    pub fn new(tolerance_s: f64) -> Self {
        Self {
            tolerance_s,
            aligned: 0,
            dropped_base: 0,
            dropped_rover: 0,
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
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct SdObservation {
    pub constellation: Constellation,
    pub prn: u8,
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
    pub constellation: Constellation,
    pub prn: u8,
    pub ref_prn: u8,
    pub code_m: f64,
    pub phase_cycles: f64,
    pub doppler_hz: f64,
    pub variance_code: f64,
    pub variance_phase: f64,
    pub canceled: Vec<AmbiguityId>,
}

fn ambiguity_id(sat: &ObsSatellite) -> AmbiguityId {
    AmbiguityId {
        constellation: sat.metadata.signal.constellation,
        prn: sat.prn,
        band: sat.metadata.signal.band,
        signal: format!("{:?}", sat.metadata.signal.band),
    }
}

pub fn build_sd(base: &ObsEpoch, rover: &ObsEpoch) -> Vec<SdObservation> {
    let mut base_map: BTreeMap<SatKey, &ObsSatellite> = BTreeMap::new();
    for sat in &base.sats {
        base_map.insert(
            SatKey {
                constellation: sat.metadata.signal.constellation,
                prn: sat.prn,
            },
            sat,
        );
    }
    let mut out = Vec::new();
    for sat in &rover.sats {
        let key = SatKey {
            constellation: sat.metadata.signal.constellation,
            prn: sat.prn,
        };
        if let Some(base_sat) = base_map.get(&key) {
            let variance_code =
                (1.0 / sat.cn0_dbhz.max(1.0)).powi(2) + (1.0 / base_sat.cn0_dbhz.max(1.0)).powi(2);
            let variance_phase = variance_code * 0.01;
            out.push(SdObservation {
                constellation: sat.metadata.signal.constellation,
                prn: sat.prn,
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

pub fn choose_ref_sat(sd: &[SdObservation]) -> Option<u8> {
    sd.iter()
        .min_by(|a, b| {
            a.variance_code
                .partial_cmp(&b.variance_code)
                .unwrap_or(std::cmp::Ordering::Equal)
        })
        .map(|s| s.prn)
}

pub fn choose_ref_sat_per_constellation(sd: &[SdObservation]) -> BTreeMap<Constellation, u8> {
    let mut by_const: BTreeMap<Constellation, Vec<SdObservation>> = BTreeMap::new();
    for s in sd {
        by_const
            .entry(s.ambiguity_rover.constellation)
            .or_default()
            .push(s.clone());
    }
    let mut out = BTreeMap::new();
    for (constellation, items) in by_const {
        if let Some(prn) = choose_ref_sat(&items) {
            out.insert(constellation, prn);
        }
    }
    out
}

pub fn build_dd(sd: &[SdObservation], ref_prn: u8) -> Vec<DdObservation> {
    let mut ref_sd = None;
    for s in sd {
        if s.prn == ref_prn {
            ref_sd = Some(s);
            break;
        }
    }
    let Some(ref_sd) = ref_sd else {
        return Vec::new();
    };
    let mut out = Vec::new();
    for s in sd {
        if s.prn == ref_prn {
            continue;
        }
        if s.constellation != ref_sd.constellation {
            continue;
        }
        out.push(DdObservation {
            constellation: s.constellation,
            prn: s.prn,
            ref_prn,
            code_m: s.code_m - ref_sd.code_m,
            phase_cycles: s.phase_cycles - ref_sd.phase_cycles,
            doppler_hz: s.doppler_hz - ref_sd.doppler_hz,
            variance_code: s.variance_code + ref_sd.variance_code,
            variance_phase: s.variance_phase + ref_sd.variance_phase,
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
    refs: &BTreeMap<Constellation, u8>,
) -> Vec<DdObservation> {
    let mut out = Vec::new();
    for (constellation, ref_prn) in refs {
        let subset: Vec<SdObservation> = sd
            .iter()
            .filter(|s| s.ambiguity_rover.constellation == *constellation)
            .cloned()
            .collect();
        out.extend(build_dd(&subset, *ref_prn));
    }
    out
}

#[derive(Debug, Clone)]
pub struct DdCovarianceModel {
    pub variance_code: f64,
    pub variance_phase: f64,
}

pub fn dd_covariance(dd: &DdObservation) -> DdCovarianceModel {
    DdCovarianceModel {
        variance_code: dd.variance_code,
        variance_phase: dd.variance_phase,
    }
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
        let eph = ephs.iter().find(|e| e.prn == obs.prn)?;
        let eph_ref = ephs.iter().find(|e| e.prn == obs.ref_prn)?;
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

#[derive(Debug, Clone)]
pub struct AlignmentDiagnostic {
    pub role: ReceiverRole,
    pub aligned: usize,
    pub dropped: usize,
}

#[derive(Debug, Clone)]
pub struct AlignmentReport {
    pub base: AlignmentDiagnostic,
    pub rover: AlignmentDiagnostic,
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
