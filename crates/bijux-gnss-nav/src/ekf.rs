use std::collections::BTreeMap;

#[derive(Debug, Clone)]
pub struct Matrix {
    rows: usize,
    cols: usize,
    data: Vec<f64>,
}

impl Matrix {
    pub fn new(rows: usize, cols: usize, value: f64) -> Self {
        Self {
            rows,
            cols,
            data: vec![value; rows * cols],
        }
    }

    pub fn identity(n: usize) -> Self {
        let mut m = Self::new(n, n, 0.0);
        for i in 0..n {
            m[(i, i)] = 1.0;
        }
        m
    }

    pub fn rows(&self) -> usize {
        self.rows
    }

    pub fn cols(&self) -> usize {
        self.cols
    }

    pub fn transpose(&self) -> Self {
        let mut out = Self::new(self.cols, self.rows, 0.0);
        for r in 0..self.rows {
            for c in 0..self.cols {
                out[(c, r)] = self[(r, c)];
            }
        }
        out
    }

    pub fn mul(&self, other: &Self) -> Self {
        let mut out = Self::new(self.rows, other.cols, 0.0);
        for r in 0..self.rows {
            for c in 0..other.cols {
                let mut sum = 0.0;
                for k in 0..self.cols {
                    sum += self[(r, k)] * other[(k, c)];
                }
                out[(r, c)] = sum;
            }
        }
        out
    }

    pub fn add(&self, other: &Self) -> Self {
        let mut out = self.clone();
        for i in 0..self.data.len() {
            out.data[i] += other.data[i];
        }
        out
    }

    pub fn sub(&self, other: &Self) -> Self {
        let mut out = self.clone();
        for i in 0..self.data.len() {
            out.data[i] -= other.data[i];
        }
        out
    }

    pub fn invert(&self) -> Option<Self> {
        if self.rows != self.cols {
            return None;
        }
        let n = self.rows;
        let mut a = vec![vec![0.0_f64; 2 * n]; n];
        for r in 0..n {
            for c in 0..n {
                a[r][c] = self[(r, c)];
            }
            a[r][n + r] = 1.0;
        }
        for i in 0..n {
            let mut pivot = i;
            let mut max = a[i][i].abs();
            for (r, row) in a.iter().enumerate().skip(i + 1) {
                if row[i].abs() > max {
                    max = row[i].abs();
                    pivot = r;
                }
            }
            if max < 1e-12 {
                return None;
            }
            if pivot != i {
                a.swap(pivot, i);
            }
            let diag = a[i][i];
            for c in 0..2 * n {
                a[i][c] /= diag;
            }
            for r in 0..n {
                if r == i {
                    continue;
                }
                let factor = a[r][i];
                for c in 0..2 * n {
                    a[r][c] -= factor * a[i][c];
                }
            }
        }
        let mut inv = Self::new(n, n, 0.0);
        for r in 0..n {
            for c in 0..n {
                inv[(r, c)] = a[r][n + c];
            }
        }
        Some(inv)
    }

    pub fn submatrix(&self, rows: &[usize], cols: &[usize]) -> Self {
        let mut out = Self::new(rows.len(), cols.len(), 0.0);
        for (i, &r) in rows.iter().enumerate() {
            for (j, &c) in cols.iter().enumerate() {
                out[(i, j)] = self[(r, c)];
            }
        }
        out
    }
}

impl std::ops::Index<(usize, usize)> for Matrix {
    type Output = f64;
    fn index(&self, index: (usize, usize)) -> &Self::Output {
        &self.data[index.0 * self.cols + index.1]
    }
}

impl std::ops::IndexMut<(usize, usize)> for Matrix {
    fn index_mut(&mut self, index: (usize, usize)) -> &mut Self::Output {
        &mut self.data[index.0 * self.cols + index.1]
    }
}

#[derive(Debug, Clone)]
pub struct EkfConfig {
    pub gating_chi2: Option<f64>,
    pub huber_k: Option<f64>,
}

#[derive(Debug, Clone)]
pub struct EkfHealth {
    pub innovation_rms: f64,
    pub rejected: usize,
    pub last_rejection: Option<String>,
    pub condition_number: Option<f64>,
    pub whiteness_ratio: Option<f64>,
    pub predicted_variance: Option<f64>,
    pub observed_variance: Option<f64>,
}

pub trait StateModel {
    fn state_dim(&self) -> usize;
    fn propagate(&self, x: &mut [f64], p: &mut Matrix, dt_s: f64);
}

pub trait MeasurementModel {
    fn name(&self) -> &'static str;
    fn measurement_dim(&self) -> usize;
    fn observation(&self) -> &[f64];
    fn h(&self, x: &[f64], out: &mut [f64]);
    fn jacobian(&self, x: &[f64], h: &mut Matrix);
    fn covariance(&self, x: &[f64], r: &mut Matrix);
}

#[derive(Debug, Clone)]
pub struct Ekf {
    pub x: Vec<f64>,
    pub p: Matrix,
    pub config: EkfConfig,
    pub health: EkfHealth,
    pub labels: Vec<String>,
}

impl Ekf {
    pub fn new(x: Vec<f64>, p: Matrix, config: EkfConfig) -> Self {
        Self {
            x,
            p,
            config,
            health: EkfHealth {
                innovation_rms: 0.0,
                rejected: 0,
                last_rejection: None,
                condition_number: None,
                whiteness_ratio: None,
                predicted_variance: None,
                observed_variance: None,
            },
            labels: Vec::new(),
        }
    }

    pub fn add_state(&mut self, label: &str, value: f64, variance: f64) {
        let n = self.x.len();
        self.x.push(value);
        let mut p_new = Matrix::new(n + 1, n + 1, 0.0);
        for r in 0..n {
            for c in 0..n {
                p_new[(r, c)] = self.p[(r, c)];
            }
        }
        p_new[(n, n)] = variance;
        self.p = p_new;
        self.labels.push(label.to_string());
    }

    pub fn predict<M: StateModel>(&mut self, model: &M, dt_s: f64) {
        model.propagate(&mut self.x, &mut self.p, dt_s);
    }

    pub fn update<M: MeasurementModel>(&mut self, model: &M) -> bool {
        let m = model.measurement_dim();
        let n = self.x.len();
        let mut h_pred = vec![0.0; m];
        model.h(&self.x, &mut h_pred);
        let z = model.observation();
        let mut y = vec![0.0; m];
        for i in 0..m {
            y[i] = z[i] - h_pred[i];
        }

        let mut h = Matrix::new(m, n, 0.0);
        model.jacobian(&self.x, &mut h);
        let mut r = Matrix::new(m, m, 0.0);
        model.covariance(&self.x, &mut r);

        let ht = h.transpose();
        let s = h.mul(&self.p).mul(&ht).add(&r);
        let mut max_diag = 0.0;
        let mut min_diag = f64::MAX;
        for i in 0..m {
            let v = s[(i, i)].abs();
            if v > max_diag {
                max_diag = v;
            }
            if v < min_diag {
                min_diag = v;
            }
        }
        if min_diag > 0.0 {
            self.health.condition_number = Some(max_diag / min_diag);
        }
        let Some(s_inv) = s.invert() else {
            self.health.rejected += 1;
            self.health.last_rejection = Some(format!("{}: singular S", model.name()));
            return false;
        };

        if let Some(chi2) = self.config.gating_chi2 {
            let mut chi = 0.0;
            for i in 0..m {
                for j in 0..m {
                    chi += y[i] * s_inv[(i, j)] * y[j];
                }
            }
            if chi > chi2 {
                self.health.rejected += 1;
                self.health.last_rejection = Some(format!("{}: chi2 gate", model.name()));
                return false;
            }
        }

        if let Some(k) = self.config.huber_k {
            for val in y.iter_mut().take(m) {
                let a = val.abs();
                if a > k {
                    *val *= k / a;
                }
            }
        }

        let k_gain = self.p.mul(&ht).mul(&s_inv);
        for i in 0..n {
            let mut dx = 0.0;
            for j in 0..m {
                dx += k_gain[(i, j)] * y[j];
            }
            self.x[i] += dx;
        }

        let i_mat = Matrix::identity(n);
        let kh = k_gain.mul(&h);
        let p_new = i_mat.sub(&kh).mul(&self.p);
        self.p = p_new;

        let rms = if m > 0 {
            (y.iter().map(|v| v * v).sum::<f64>() / m as f64).sqrt()
        } else {
            0.0
        };
        self.health.innovation_rms = rms;
        let predicted = if m > 0 {
            let mut sum = 0.0;
            for i in 0..m {
                sum += s[(i, i)];
            }
            sum / m as f64
        } else {
            0.0
        };
        let observed = if m > 0 {
            y.iter().map(|v| v * v).sum::<f64>() / m as f64
        } else {
            0.0
        };
        self.health.predicted_variance = Some(predicted);
        self.health.observed_variance = Some(observed);
        if predicted > 0.0 {
            self.health.whiteness_ratio = Some(observed / predicted);
        }
        true
    }
}

#[derive(Debug, Clone)]
pub struct ProcessNoiseConfig {
    pub pos_m: f64,
    pub vel_mps: f64,
    pub clock_bias_s: f64,
    pub clock_drift_s: f64,
}

#[derive(Debug, Clone)]
pub struct NavClockModel {
    pub noise: ProcessNoiseConfig,
}

impl NavClockModel {
    pub fn new(noise: ProcessNoiseConfig) -> Self {
        Self { noise }
    }
}

impl StateModel for NavClockModel {
    fn state_dim(&self) -> usize {
        8
    }

    fn propagate(&self, x: &mut [f64], p: &mut Matrix, dt_s: f64) {
        if x.len() < 8 {
            return;
        }
        x[0] += x[3] * dt_s;
        x[1] += x[4] * dt_s;
        x[2] += x[5] * dt_s;
        x[6] += x[7] * dt_s;

        let mut f = Matrix::identity(8);
        f[(0, 3)] = dt_s;
        f[(1, 4)] = dt_s;
        f[(2, 5)] = dt_s;
        f[(6, 7)] = dt_s;

        let mut q = Matrix::new(8, 8, 0.0);
        q[(0, 0)] = self.noise.pos_m * self.noise.pos_m;
        q[(1, 1)] = self.noise.pos_m * self.noise.pos_m;
        q[(2, 2)] = self.noise.pos_m * self.noise.pos_m;
        q[(3, 3)] = self.noise.vel_mps * self.noise.vel_mps;
        q[(4, 4)] = self.noise.vel_mps * self.noise.vel_mps;
        q[(5, 5)] = self.noise.vel_mps * self.noise.vel_mps;
        q[(6, 6)] = self.noise.clock_bias_s * self.noise.clock_bias_s;
        q[(7, 7)] = self.noise.clock_drift_s * self.noise.clock_drift_s;

        let ft = f.transpose();
        let p_new = f.mul(p).mul(&ft).add(&q);
        *p = p_new;
    }
}

#[derive(Debug, Clone)]
pub struct PseudorangeMeasurement {
    pub prn: u8,
    pub z_m: f64,
    pub sat_pos_m: [f64; 3],
    pub sat_clock_s: f64,
    pub tropo_m: f64,
    pub iono_m: f64,
    pub sigma_m: f64,
    pub elevation_deg: Option<f64>,
    pub ztd_index: Option<usize>,
}

impl MeasurementModel for PseudorangeMeasurement {
    fn name(&self) -> &'static str {
        "pseudorange"
    }

    fn measurement_dim(&self) -> usize {
        1
    }

    fn observation(&self) -> &[f64] {
        std::slice::from_ref(&self.z_m)
    }

    fn h(&self, x: &[f64], out: &mut [f64]) {
        let dx = x[0] - self.sat_pos_m[0];
        let dy = x[1] - self.sat_pos_m[1];
        let dz = x[2] - self.sat_pos_m[2];
        let range = (dx * dx + dy * dy + dz * dz).sqrt();
        let mut tropo = self.tropo_m;
        if let Some(idx) = self.ztd_index {
            if let Some(elev) = self.elevation_deg {
                let m = 1.0 / elev.to_radians().sin().max(0.1);
                if let Some(ztd) = x.get(idx) {
                    tropo = ztd * m;
                }
            }
        }
        let pred = range + 299_792_458.0 * (x[6] - self.sat_clock_s) + tropo - self.iono_m;
        out[0] = pred;
    }

    fn jacobian(&self, x: &[f64], h: &mut Matrix) {
        let dx = x[0] - self.sat_pos_m[0];
        let dy = x[1] - self.sat_pos_m[1];
        let dz = x[2] - self.sat_pos_m[2];
        let range = (dx * dx + dy * dy + dz * dz).sqrt().max(1.0);
        h[(0, 0)] = dx / range;
        h[(0, 1)] = dy / range;
        h[(0, 2)] = dz / range;
        h[(0, 6)] = 299_792_458.0;
    }

    fn covariance(&self, _x: &[f64], r: &mut Matrix) {
        r[(0, 0)] = self.sigma_m * self.sigma_m;
    }
}

#[derive(Debug, Clone)]
pub struct DopplerMeasurement {
    pub prn: u8,
    pub z_hz: f64,
    pub sat_pos_m: [f64; 3],
    pub sat_vel_mps: [f64; 3],
    pub wavelength_m: f64,
    pub sigma_hz: f64,
}

impl MeasurementModel for DopplerMeasurement {
    fn name(&self) -> &'static str {
        "doppler"
    }

    fn measurement_dim(&self) -> usize {
        1
    }

    fn observation(&self) -> &[f64] {
        std::slice::from_ref(&self.z_hz)
    }

    fn h(&self, x: &[f64], out: &mut [f64]) {
        let dx = x[0] - self.sat_pos_m[0];
        let dy = x[1] - self.sat_pos_m[1];
        let dz = x[2] - self.sat_pos_m[2];
        let range = (dx * dx + dy * dy + dz * dz).sqrt().max(1.0);
        let los = [dx / range, dy / range, dz / range];
        let rel_vel = [
            x[3] - self.sat_vel_mps[0],
            x[4] - self.sat_vel_mps[1],
            x[5] - self.sat_vel_mps[2],
        ];
        let range_rate = los[0] * rel_vel[0] + los[1] * rel_vel[1] + los[2] * rel_vel[2];
        let pred = -range_rate / self.wavelength_m + 299_792_458.0 * x[7] / self.wavelength_m;
        out[0] = pred;
    }

    fn jacobian(&self, x: &[f64], h: &mut Matrix) {
        let dx = x[0] - self.sat_pos_m[0];
        let dy = x[1] - self.sat_pos_m[1];
        let dz = x[2] - self.sat_pos_m[2];
        let range = (dx * dx + dy * dy + dz * dz).sqrt().max(1.0);
        let los = [dx / range, dy / range, dz / range];
        h[(0, 3)] = -los[0] / self.wavelength_m;
        h[(0, 4)] = -los[1] / self.wavelength_m;
        h[(0, 5)] = -los[2] / self.wavelength_m;
        h[(0, 7)] = 299_792_458.0 / self.wavelength_m;
    }

    fn covariance(&self, _x: &[f64], r: &mut Matrix) {
        r[(0, 0)] = self.sigma_hz * self.sigma_hz;
    }
}

#[derive(Debug, Clone)]
pub struct CarrierPhaseMeasurement {
    pub prn: u8,
    pub z_cycles: f64,
    pub sat_pos_m: [f64; 3],
    pub sat_clock_s: f64,
    pub tropo_m: f64,
    pub iono_m: f64,
    pub wavelength_m: f64,
    pub ambiguity_index: Option<usize>,
    pub sigma_cycles: f64,
    pub elevation_deg: Option<f64>,
    pub ztd_index: Option<usize>,
}

impl MeasurementModel for CarrierPhaseMeasurement {
    fn name(&self) -> &'static str {
        "carrier_phase"
    }

    fn measurement_dim(&self) -> usize {
        1
    }

    fn observation(&self) -> &[f64] {
        std::slice::from_ref(&self.z_cycles)
    }

    fn h(&self, x: &[f64], out: &mut [f64]) {
        let dx = x[0] - self.sat_pos_m[0];
        let dy = x[1] - self.sat_pos_m[1];
        let dz = x[2] - self.sat_pos_m[2];
        let range = (dx * dx + dy * dy + dz * dz).sqrt();
        let n = self
            .ambiguity_index
            .and_then(|idx| x.get(idx).copied())
            .unwrap_or(0.0);
        let mut tropo = self.tropo_m;
        if let Some(idx) = self.ztd_index {
            if let Some(elev) = self.elevation_deg {
                let m = 1.0 / elev.to_radians().sin().max(0.1);
                if let Some(ztd) = x.get(idx) {
                    tropo = ztd * m;
                }
            }
        }
        let pred = (range + 299_792_458.0 * (x[6] - self.sat_clock_s) + tropo - self.iono_m)
            / self.wavelength_m
            + n;
        out[0] = pred;
    }

    fn jacobian(&self, x: &[f64], h: &mut Matrix) {
        let dx = x[0] - self.sat_pos_m[0];
        let dy = x[1] - self.sat_pos_m[1];
        let dz = x[2] - self.sat_pos_m[2];
        let range = (dx * dx + dy * dy + dz * dz).sqrt().max(1.0);
        h[(0, 0)] = (dx / range) / self.wavelength_m;
        h[(0, 1)] = (dy / range) / self.wavelength_m;
        h[(0, 2)] = (dz / range) / self.wavelength_m;
        h[(0, 6)] = 299_792_458.0 / self.wavelength_m;
        if let Some(idx) = self.ambiguity_index {
            h[(0, idx)] = 1.0;
        }
    }

    fn covariance(&self, _x: &[f64], r: &mut Matrix) {
        r[(0, 0)] = self.sigma_cycles * self.sigma_cycles;
    }
}

#[derive(Debug, Clone)]
pub struct AmbiguityManager {
    pub indices: BTreeMap<String, usize>,
}

impl AmbiguityManager {
    pub fn new() -> Self {
        Self {
            indices: BTreeMap::new(),
        }
    }

    pub fn get_or_add(&mut self, ekf: &mut Ekf, key: &str, value: f64, variance: f64) -> usize {
        if let Some(idx) = self.indices.get(key) {
            return *idx;
        }
        let idx = ekf.x.len();
        ekf.add_state(key, value, variance);
        self.indices.insert(key.to_string(), idx);
        idx
    }
}

impl Default for AmbiguityManager {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn matrix_inversion_identity() {
        let m = Matrix::identity(3);
        let inv = m.invert().expect("invert");
        for i in 0..3 {
            for j in 0..3 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!((inv[(i, j)] - expected).abs() < 1e-9);
            }
        }
    }

    #[test]
    fn ekf_update_runs() {
        let x = vec![0.0; 8];
        let p = Matrix::identity(8);
        let mut ekf = Ekf::new(
            x,
            p,
            EkfConfig {
                gating_chi2: Some(100.0),
                huber_k: Some(10.0),
            },
        );
        let meas = PseudorangeMeasurement {
            prn: 1,
            z_m: 20_200_000.0,
            sat_pos_m: [15_000_000.0, 0.0, 21_000_000.0],
            sat_clock_s: 0.0,
            tropo_m: 0.0,
            iono_m: 0.0,
            sigma_m: 10.0,
            elevation_deg: None,
            ztd_index: None,
        };
        assert!(ekf.update(&meas));
    }
}
