#![allow(missing_docs)]

use crate::orbits::gps::{is_ephemeris_valid, sat_state_gps_l1ca, GpsEphemeris, GpsSatState};
use bijux_gnss_core::api::SatId;

#[derive(Debug, Clone)]
pub struct PositionSolution {
    pub ecef_x_m: f64,
    pub ecef_y_m: f64,
    pub ecef_z_m: f64,
    pub latitude_deg: f64,
    pub longitude_deg: f64,
    pub altitude_m: f64,
    pub clock_bias_s: f64,
    pub pdop: f64,
    pub rms_m: f64,
    pub sigma_h_m: Option<f64>,
    pub sigma_v_m: Option<f64>,
    pub residuals: Vec<(SatId, f64, f64)>,
    pub rejected: Vec<(SatId, bijux_gnss_core::api::MeasurementRejectReason)>,
    pub separation_max_m: Option<f64>,
    pub separation_suspect: Option<SatId>,
    pub covariance_symmetrized: bool,
    pub covariance_clamped: bool,
    pub covariance_max_variance: Option<f64>,
}

#[derive(Debug, Clone)]
pub struct PositionObservation {
    pub sat: SatId,
    pub pseudorange_m: f64,
    pub cn0_dbhz: f64,
    pub elevation_deg: Option<f64>,
    pub weight: f64,
}

#[derive(Debug, Clone)]
pub struct PositionSolver {
    pub max_iterations: usize,
    pub convergence_m: f64,
    pub residual_gate_m: f64,
    pub chi_square_gate: f64,
    pub robust: bool,
    pub huber_k: f64,
    pub raim: bool,
    pub separation_gate_m: f64,
}

impl Default for PositionSolver {
    fn default() -> Self {
        Self::new()
    }
}

impl PositionSolver {
    pub fn new() -> Self {
        Self {
            max_iterations: 10,
            convergence_m: 1e-3,
            residual_gate_m: 150.0,
            chi_square_gate: 9.0,
            robust: true,
            huber_k: 30.0,
            raim: true,
            separation_gate_m: 50.0,
        }
    }

    pub fn solve_wls(
        &self,
        observations: &[PositionObservation],
        ephemerides: &[GpsEphemeris],
        t_rx_s: f64,
    ) -> Option<PositionSolution> {
        if observations.len() < 4 {
            return None;
        }
        let mut observations = observations.to_vec();
        observations.sort_by_key(|obs| (obs.sat.constellation as u8, obs.sat.prn));
        let mut x = 0.0_f64;
        let mut y = 0.0_f64;
        let mut z = 0.0_f64;
        let mut cb = 0.0_f64;

        let mut used: Vec<(PositionObservation, GpsSatState, f64)> = Vec::new();

        let mut rejected = Vec::new();
        let mut residuals = Vec::new();
        let mut cov = None;
        let mut cov_symmetrized = false;
        let mut cov_clamped = false;
        let mut cov_max_variance = None;
        for _ in 0..self.max_iterations {
            used.clear();
            for obs in &observations {
                let eph = match ephemerides.iter().find(|e| e.sat == obs.sat) {
                    Some(eph) => eph,
                    None => {
                        rejected.push((obs.sat, bijux_gnss_core::api::MeasurementRejectReason::InvalidEphemeris));
                        continue;
                    }
                };
                if !is_ephemeris_valid(eph, t_rx_s) {
                    rejected.push((obs.sat, bijux_gnss_core::api::MeasurementRejectReason::InvalidEphemeris));
                    continue;
                }
                let mut tau = obs.pseudorange_m / 299_792_458.0;
                let mut state = sat_state_gps_l1ca(eph, t_rx_s - tau, tau);
                let mut converged = false;
                for _ in 0..5 {
                    let dx = x - state.x_m;
                    let dy = y - state.y_m;
                    let dz = z - state.z_m;
                    let range = (dx * dx + dy * dy + dz * dz).sqrt();
                    let next_tau = (range + cb * 299_792_458.0
                        - state.clock_bias_s * 299_792_458.0)
                        / 299_792_458.0;
                    if (next_tau - tau).abs() < 1e-9 {
                        converged = true;
                    }
                    tau = next_tau;
                    state = sat_state_gps_l1ca(eph, t_rx_s - tau, tau);
                    if converged {
                        break;
                    }
                }
                if !converged {
                    return None;
                }
                used.push((obs.clone(), state, tau));
            }
            if used.len() < 4 {
                return None;
            }
            let mut h = Vec::new();
            let mut v = Vec::new();
            residuals.clear();
            for (obs, state, _tau) in &used {
                let dx = x - state.x_m;
                let dy = y - state.y_m;
                let dz = z - state.z_m;
                let range = (dx * dx + dy * dy + dz * dz).sqrt();
                let pred = range + cb * 299_792_458.0 - state.clock_bias_s * 299_792_458.0;
                let res = obs.pseudorange_m - pred;
                residuals.push((obs.sat, res));
                let hx = dx / range;
                let hy = dy / range;
                let hz = dz / range;
                h.push([hx, hy, hz, 1.0]);
                v.push(res);
            }

            let mut weights = if self.robust {
                huber_weights(&v, self.huber_k)
            } else {
                vec![1.0; v.len()]
            };
            for (i, (obs, _, _)) in used.iter().enumerate() {
                if let Some(w) = weights.get_mut(i) {
                    *w *= obs.weight;
                }
            }
            let (dx, dy, dz, dcb, cov_out) = solve_weighted_normal_eq(&h, &v, &weights)?;
            let (cov_out, sym, clamp, max_var) = sanitize_covariance(cov_out);
            cov_symmetrized |= sym;
            cov_clamped |= clamp;
            if let Some(max_var) = max_var {
                cov_max_variance = Some(
                    cov_max_variance
                        .map(|v: f64| v.max(max_var))
                        .unwrap_or(max_var),
                );
            }
            cov = Some(cov_out);
            x += dx;
            y += dy;
            z += dz;
            cb += dcb / 299_792_458.0;
            if (dx * dx + dy * dy + dz * dz).sqrt() < self.convergence_m {
                break;
            }
        }

        let mut filtered = Vec::new();
        for (obs, state, _tau) in &used {
            let dx = x - state.x_m;
            let dy = y - state.y_m;
            let dz = z - state.z_m;
            let range = (dx * dx + dy * dy + dz * dz).sqrt();
            let pred = range + cb * 299_792_458.0 - state.clock_bias_s * 299_792_458.0;
            let res = obs.pseudorange_m - pred;
            let sigma_m = (1.0 / obs.weight.max(1e-6)).sqrt();
            let norm = res / sigma_m;
            if res.abs() > self.residual_gate_m || (norm * norm) > self.chi_square_gate {
                rejected.push((obs.sat, bijux_gnss_core::api::MeasurementRejectReason::Outlier));
            } else {
                filtered.push((obs.clone(), state.clone(), res));
            }
        }

        if filtered.len() < 4 {
            return None;
        }

        let mut separation_max = None;
        let mut separation_suspect = None;
        if self.raim && filtered.len() >= 5 {
            let (worst_idx, worst_res) = filtered
                .iter()
                .enumerate()
                .map(|(i, (_, _, r))| (i, r.abs()))
                .max_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal))
                .unwrap_or((0, 0.0));
            if worst_res > self.residual_gate_m {
                rejected.push((
                    filtered[worst_idx].0.sat,
                    bijux_gnss_core::api::MeasurementRejectReason::Outlier,
                ));
                filtered.remove(worst_idx);
            }

            for idx in 0..filtered.len() {
                let mut subset = filtered.clone();
                let removed = subset.remove(idx);
                if subset.len() < 4 {
                    continue;
                }
                let mut h_sep = Vec::new();
                let mut v_sep = Vec::new();
                for (_obs, state, res) in &subset {
                    let dx = x - state.x_m;
                    let dy = y - state.y_m;
                    let dz = z - state.z_m;
                    let range = (dx * dx + dy * dy + dz * dz).sqrt();
                    let hx = dx / range;
                    let hy = dy / range;
                    let hz = dz / range;
                    h_sep.push([hx, hy, hz, 1.0]);
                    v_sep.push(*res);
                }
                if let Some((dx, dy, dz, _dcb, _)) =
                    solve_weighted_normal_eq(&h_sep, &v_sep, &vec![1.0; v_sep.len()])
                {
                    let delta = (dx * dx + dy * dy + dz * dz).sqrt();
                    if separation_max.map(|m| delta > m).unwrap_or(true) {
                        separation_max = Some(delta);
                        separation_suspect = Some(removed.0.sat);
                    }
                }
            }
        }

        let mut h = Vec::new();
        let mut v = Vec::new();
        for (_obs, state, res) in &filtered {
            let dx = x - state.x_m;
            let dy = y - state.y_m;
            let dz = z - state.z_m;
            let range = (dx * dx + dy * dy + dz * dz).sqrt();
            let hx = dx / range;
            let hy = dy / range;
            let hz = dz / range;
            h.push([hx, hy, hz, 1.0]);
            v.push(*res);
        }

        let pdop = compute_pdop(&h).unwrap_or(0.0);
        let rms = if !v.is_empty() {
            let sum = v.iter().map(|r| r * r).sum::<f64>();
            (sum / v.len() as f64).sqrt()
        } else {
            0.0
        };

        let (lat, lon, alt) = ecef_to_geodetic(x, y, z);

        let (sigma_h_m, sigma_v_m) = cov
            .map(|cov| {
                let sigma2 = if !v.is_empty() {
                    let sum = v.iter().map(|r| r * r).sum::<f64>();
                    let dof = (v.len() as i32 - 4).max(1) as f64;
                    sum / dof
                } else {
                    0.0
                };
                let var_x = cov[0][0] * sigma2;
                let var_y = cov[1][1] * sigma2;
                let var_z = cov[2][2] * sigma2;
                ((var_x + var_y).max(0.0).sqrt(), var_z.max(0.0).sqrt())
            })
            .unwrap_or((0.0, 0.0));

        Some(PositionSolution {
            ecef_x_m: x,
            ecef_y_m: y,
            ecef_z_m: z,
            latitude_deg: lat,
            longitude_deg: lon,
            altitude_m: alt,
            clock_bias_s: cb,
            pdop,
            rms_m: rms,
            sigma_h_m: Some(sigma_h_m),
            sigma_v_m: Some(sigma_v_m),
            residuals: filtered
                .iter()
                .map(|(o, _, r)| (o.sat, *r, o.weight))
                .collect(),
            rejected,
            separation_max_m: separation_max,
            separation_suspect,
            covariance_symmetrized: cov_symmetrized,
            covariance_clamped: cov_clamped,
            covariance_max_variance: cov_max_variance,
        })
    }
}

fn sanitize_covariance(mut cov: [[f64; 4]; 4]) -> ([[f64; 4]; 4], bool, bool, Option<f64>) {
    let mut sym = false;
    let mut clamp = false;
    let mut max_var = None;
    for i in 0..4 {
        for j in 0..4 {
            if (cov[i][j] - cov[j][i]).abs() > 1e-9 {
                sym = true;
                let avg = 0.5 * (cov[i][j] + cov[j][i]);
                cov[i][j] = avg;
                cov[j][i] = avg;
            }
        }
        if cov[i][i] < 0.0 {
            clamp = true;
            cov[i][i] = 0.0;
        }
        max_var = Some(
            max_var
                .map(|v: f64| v.max(cov[i][i]))
                .unwrap_or(cov[i][i]),
        );
    }
    (cov, sym, clamp, max_var)
}

type NormalEqSolution = (f64, f64, f64, f64, [[f64; 4]; 4]);

fn solve_weighted_normal_eq(h: &[[f64; 4]], v: &[f64], w: &[f64]) -> Option<NormalEqSolution> {
    let mut n = [[0.0_f64; 4]; 4];
    let mut u = [0.0_f64; 4];
    for (i, row) in h.iter().enumerate() {
        let wi = w.get(i).copied().unwrap_or(1.0);
        for r in 0..4 {
            u[r] += row[r] * v[i] * wi;
            for c in 0..4 {
                n[r][c] += row[r] * row[c] * wi;
            }
        }
    }
    let inv = invert_4x4(n)?;
    let dx = inv[0][0] * u[0] + inv[0][1] * u[1] + inv[0][2] * u[2] + inv[0][3] * u[3];
    let dy = inv[1][0] * u[0] + inv[1][1] * u[1] + inv[1][2] * u[2] + inv[1][3] * u[3];
    let dz = inv[2][0] * u[0] + inv[2][1] * u[1] + inv[2][2] * u[2] + inv[2][3] * u[3];
    let dcb = inv[3][0] * u[0] + inv[3][1] * u[1] + inv[3][2] * u[2] + inv[3][3] * u[3];
    Some((dx, dy, dz, dcb, inv))
}

fn huber_weights(residuals: &[f64], k: f64) -> Vec<f64> {
    residuals
        .iter()
        .map(|r| {
            let a = r.abs();
            if a <= k {
                1.0
            } else {
                k / a
            }
        })
        .collect()
}

pub fn invert_4x4(a: [[f64; 4]; 4]) -> Option<[[f64; 4]; 4]> {
    let mut m = [[0.0_f64; 8]; 4];
    for i in 0..4 {
        for j in 0..4 {
            m[i][j] = a[i][j];
        }
        m[i][i + 4] = 1.0;
    }
    for i in 0..4 {
        let mut pivot = i;
        let mut max = m[i][i].abs();
        for (r, row) in m.iter().enumerate().skip(i + 1) {
            if row[i].abs() > max {
                max = row[i].abs();
                pivot = r;
            }
        }
        if max < 1e-12 {
            return None;
        }
        if pivot != i {
            m.swap(i, pivot);
        }
        let inv_pivot = 1.0 / m[i][i];
        for j in i..8 {
            m[i][j] *= inv_pivot;
        }
        for r in 0..4 {
            if r == i {
                continue;
            }
            let factor = m[r][i];
            for j in i..8 {
                m[r][j] -= factor * m[i][j];
            }
        }
    }
    let mut inv = [[0.0_f64; 4]; 4];
    for i in 0..4 {
        for j in 0..4 {
            inv[i][j] = m[i][j + 4];
        }
    }
    Some(inv)
}

fn compute_pdop(h: &[[f64; 4]]) -> Option<f64> {
    let mut n = [[0.0_f64; 4]; 4];
    for row in h {
        for r in 0..4 {
            for c in 0..4 {
                n[r][c] += row[r] * row[c];
            }
        }
    }
    let inv = invert_4x4(n)?;
    let pdop = (inv[0][0] + inv[1][1] + inv[2][2]).sqrt();
    Some(pdop)
}

pub fn ecef_to_geodetic(x: f64, y: f64, z: f64) -> (f64, f64, f64) {
    let a = 6378137.0;
    let f = 1.0 / 298.257_223_563;
    let e2 = f * (2.0 - f);

    let lon = y.atan2(x);
    let p = (x * x + y * y).sqrt();
    let mut lat = z.atan2(p * (1.0 - e2));
    let mut alt = 0.0;
    for _ in 0..5 {
        let sin_lat = lat.sin();
        let n = a / (1.0 - e2 * sin_lat * sin_lat).sqrt();
        alt = p / lat.cos() - n;
        lat = z.atan2(p * (1.0 - e2 * n / (n + alt)));
    }
    (lat.to_degrees(), lon.to_degrees(), alt)
}

pub fn geodetic_to_ecef(lat_deg: f64, lon_deg: f64, alt_m: f64) -> (f64, f64, f64) {
    let a = 6378137.0;
    let f = 1.0 / 298.257_223_563;
    let e2 = f * (2.0 - f);
    let lat = lat_deg.to_radians();
    let lon = lon_deg.to_radians();
    let sin_lat = lat.sin();
    let cos_lat = lat.cos();
    let n = a / (1.0 - e2 * sin_lat * sin_lat).sqrt();
    let x = (n + alt_m) * cos_lat * lon.cos();
    let y = (n + alt_m) * cos_lat * lon.sin();
    let z = (n * (1.0 - e2) + alt_m) * sin_lat;
    (x, y, z)
}

pub fn ecef_to_enu(
    x: f64,
    y: f64,
    z: f64,
    ref_lat_deg: f64,
    ref_lon_deg: f64,
    ref_alt_m: f64,
) -> (f64, f64, f64) {
    let (xr, yr, zr) = geodetic_to_ecef(ref_lat_deg, ref_lon_deg, ref_alt_m);
    let dx = x - xr;
    let dy = y - yr;
    let dz = z - zr;
    let lat = ref_lat_deg.to_radians();
    let lon = ref_lon_deg.to_radians();
    let sin_lat = lat.sin();
    let cos_lat = lat.cos();
    let sin_lon = lon.sin();
    let cos_lon = lon.cos();
    let east = -sin_lon * dx + cos_lon * dy;
    let north = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz;
    let up = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz;
    (east, north, up)
}

pub fn elevation_azimuth_deg(
    rx_x: f64,
    rx_y: f64,
    rx_z: f64,
    sat_x: f64,
    sat_y: f64,
    sat_z: f64,
) -> (f64, f64) {
    let (lat, lon, alt) = ecef_to_geodetic(rx_x, rx_y, rx_z);
    let (e, n, u) = ecef_to_enu(sat_x, sat_y, sat_z, lat, lon, alt);
    let az = e.atan2(n).to_degrees().rem_euclid(360.0);
    let el = (u / (e * e + n * n + u * u).sqrt()).asin().to_degrees();
    (az, el)
}

#[derive(Debug, Clone, Copy)]
pub struct WeightingConfig {
    pub min_elev_deg: f64,
    pub elev_exponent: f64,
    pub cn0_ref_dbhz: f64,
    pub min_weight: f64,
    pub enabled: bool,
}

impl Default for WeightingConfig {
    fn default() -> Self {
        Self {
            min_elev_deg: 5.0,
            elev_exponent: 2.0,
            cn0_ref_dbhz: 50.0,
            min_weight: 0.1,
            enabled: true,
        }
    }
}

pub fn weight_from_cn0_elev(cn0_dbhz: f64, elev_deg: f64, config: WeightingConfig) -> f64 {
    if !config.enabled {
        return 1.0;
    }
    let elev = elev_deg.clamp(0.0, 90.0).max(config.min_elev_deg);
    let w_elev = (elev / 90.0)
        .powf(config.elev_exponent)
        .max(config.min_weight);
    let w_cn0 = (cn0_dbhz / config.cn0_ref_dbhz).max(config.min_weight);
    (w_elev * w_cn0).max(config.min_weight)
}
