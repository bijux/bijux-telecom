use std::collections::{BTreeMap, BTreeSet};

use bijux_gnss_core::{Constellation, ObsEpoch, ObsSatellite, SatId, SigId, SignalBand};

use crate::ekf::{Ekf, EkfConfig, MeasurementKind, MeasurementModel, StateModel};
use crate::linalg::Matrix;
use crate::{
    elevation_azimuth_deg, sat_state_gps_l1ca, weight_from_cn0_elev, CodeBiasProvider,
    CorrectionContext, Corrections, GpsEphemeris, GpsSatState, PhaseBiasProvider, ProductsProvider,
    WeightingConfig, ZeroBiases,
};

const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;

#[derive(Debug, Clone)]
pub struct PppProcessNoise {
    pub clock_drift_s: f64,
    pub ztd_m: f64,
    pub iono_m: f64,
    pub ambiguity_cycles: f64,
}

#[derive(Debug, Clone)]
pub struct PppConvergenceConfig {
    pub min_time_s: f64,
    pub pos_rate_mps: f64,
    pub sigma_h_m: f64,
    pub sigma_v_m: f64,
}

#[derive(Debug, Clone)]
pub struct PppConfig {
    pub enable_iono_state: bool,
    pub use_iono_free: bool,
    pub use_doppler: bool,
    pub prune_after_epochs: u64,
    pub reset_gap_s: f64,
    pub residual_gate_m: f64,
    pub drift_window_epochs: usize,
    pub drift_threshold_m: f64,
    pub process_noise: PppProcessNoise,
    pub weighting: WeightingConfig,
    pub convergence: PppConvergenceConfig,
}

impl Default for PppConfig {
    fn default() -> Self {
        Self {
            enable_iono_state: false,
            use_iono_free: false,
            use_doppler: false,
            prune_after_epochs: 200,
            reset_gap_s: 2.0,
            residual_gate_m: 200.0,
            drift_window_epochs: 100,
            drift_threshold_m: 10.0,
            process_noise: PppProcessNoise {
                clock_drift_s: 1e-5,
                ztd_m: 0.01,
                iono_m: 0.1,
                ambiguity_cycles: 0.05,
            },
            weighting: WeightingConfig::default(),
            convergence: PppConvergenceConfig {
                min_time_s: 60.0,
                pos_rate_mps: 0.1,
                sigma_h_m: 1.0,
                sigma_v_m: 2.0,
            },
        }
    }
}

#[derive(Debug, Clone)]
pub struct PppSolutionEpoch {
    pub epoch_idx: u64,
    pub t_rx_s: f64,
    pub ecef_x_m: f64,
    pub ecef_y_m: f64,
    pub ecef_z_m: f64,
    pub clock_bias_s: f64,
    pub rms_m: f64,
    pub sigma_h_m: Option<f64>,
    pub sigma_v_m: Option<f64>,
    pub innovation_rms: f64,
    pub convergence: PppConvergenceState,
    pub residuals: Vec<(SigId, f64)>,
    pub nis_mean: Option<f64>,
}

#[derive(Debug, Clone)]
pub struct PppConvergenceState {
    pub converged: bool,
    pub time_to_first_meter_s: Option<f64>,
    pub time_to_decimeter_s: Option<f64>,
    pub time_to_centimeter_s: Option<f64>,
    pub last_position_change_m: Option<f64>,
}

#[derive(Debug, Clone)]
pub struct PppHealth {
    pub last_reset_reason: Option<String>,
    pub pruned_states: usize,
    pub convergence: PppConvergenceState,
    pub warnings: Vec<String>,
    pub nis_mean: Option<f64>,
}

#[derive(Debug, Clone)]
struct PppIndices {
    pos: [usize; 3],
    vel: [usize; 3],
    clock_bias: usize,
    clock_drift: usize,
    ztd: usize,
    isb: BTreeMap<Constellation, usize>,
    iono: BTreeMap<SatId, usize>,
    ambiguity: BTreeMap<SigId, usize>,
}

pub struct PppFilter {
    pub ekf: Ekf,
    pub config: PppConfig,
    indices: PppIndices,
    last_t_rx_s: Option<f64>,
    last_pos: Option<[f64; 3]>,
    epoch0_t_s: Option<f64>,
    last_seen_iono: BTreeMap<SatId, u64>,
    last_seen_amb: BTreeMap<SigId, u64>,
    residual_history: BTreeMap<SigId, Vec<f64>>,
    drift_history: Vec<[f64; 3]>,
    pub health: PppHealth,
    code_bias: Box<dyn CodeBiasProvider + Send + Sync>,
    phase_bias: Box<dyn PhaseBiasProvider + Send + Sync>,
    corrections: CorrectionContext,
}

impl PppFilter {
    pub fn new(config: PppConfig) -> Self {
        let x = vec![0.0_f64; 9];
        let p = Matrix::identity(9);
        let ekf = Ekf::new(
            x,
            p,
            EkfConfig {
                gating_chi2_code: Some(200.0),
                gating_chi2_phase: Some(200.0),
                gating_chi2_doppler: Some(200.0),
                huber_k: Some(10.0),
                square_root: true,
                covariance_epsilon: 1e-6,
                divergence_max_variance: 1e12,
            },
        );
        Self {
            ekf,
            config,
            indices: PppIndices {
                pos: [0, 1, 2],
                vel: [3, 4, 5],
                clock_bias: 6,
                clock_drift: 7,
                ztd: 8,
                isb: BTreeMap::new(),
                iono: BTreeMap::new(),
                ambiguity: BTreeMap::new(),
            },
            last_t_rx_s: None,
            last_pos: None,
            epoch0_t_s: None,
            last_seen_iono: BTreeMap::new(),
            last_seen_amb: BTreeMap::new(),
            health: PppHealth {
                last_reset_reason: None,
                pruned_states: 0,
                convergence: PppConvergenceState {
                    converged: false,
                    time_to_first_meter_s: None,
                    time_to_decimeter_s: None,
                    time_to_centimeter_s: None,
                    last_position_change_m: None,
                },
                warnings: Vec::new(),
                nis_mean: None,
            },
            code_bias: Box::new(ZeroBiases),
            phase_bias: Box::new(ZeroBiases),
            corrections: CorrectionContext,
            residual_history: BTreeMap::new(),
            drift_history: Vec::new(),
        }
    }

    pub fn set_biases(
        &mut self,
        code: Box<dyn CodeBiasProvider + Send + Sync>,
        phase: Box<dyn PhaseBiasProvider + Send + Sync>,
    ) {
        self.code_bias = code;
        self.phase_bias = phase;
    }

    pub fn set_corrections(&mut self, ctx: CorrectionContext) {
        self.corrections = ctx;
    }

    pub fn solve_epoch(
        &mut self,
        obs: &ObsEpoch,
        ephs: &[GpsEphemeris],
        products: &dyn ProductsProvider,
    ) -> Option<PppSolutionEpoch> {
        let dt_s = if let Some(prev) = self.last_t_rx_s {
            (obs.t_rx_s - prev).max(1e-3)
        } else {
            0.001
        };
        if dt_s > self.config.reset_gap_s {
            self.reset("epoch_gap");
        }
        self.last_t_rx_s = Some(obs.t_rx_s);
        if self.epoch0_t_s.is_none() {
            self.epoch0_t_s = Some(obs.t_rx_s);
        }
        self.predict(dt_s);

        let mut sats: Vec<&ObsSatellite> = obs.sats.iter().collect();
        sats.sort_by_key(|s| s.signal_id);
        self.ensure_states(&sats);
        let corr = crate::compute_corrections(&self.corrections);

        let mut residuals = Vec::new();
        let mut used = 0;
        let mut slip_count = 0usize;
        for sat in &sats {
            if sat.lock_flags.cycle_slip {
                slip_count += 1;
            }
            let eph = match ephs.iter().find(|e| e.sat == sat.signal_id.sat) {
                Some(e) => e,
                None => continue,
            };
            let (state, clock_bias_s, fallback) =
                match self.sat_state(products, eph, sat.signal_id.sat, obs.t_rx_s) {
                    Some(v) => v,
                    None => continue,
                };
            if fallback {
                self.health.warnings.push(format!(
                    "products fallback used for {:?}",
                    sat.signal_id.sat
                ));
            }
            let rx_x = self.ekf.x[self.indices.pos[0]];
            let rx_y = self.ekf.x[self.indices.pos[1]];
            let rx_z = self.ekf.x[self.indices.pos[2]];
            let (_az, el) =
                elevation_azimuth_deg(rx_x, rx_y, rx_z, state.x_m, state.y_m, state.z_m);
            if el < self.config.weighting.min_elev_deg {
                continue;
            }
            let weight = weight_from_cn0_elev(sat.cn0_dbhz, el, self.config.weighting);
            let sigma_m = (5.0 / weight.max(0.1)).max(1.0);

            let isb_index = self
                .indices
                .isb
                .get(&sat.signal_id.sat.constellation)
                .copied();
            let iono_index = self.indices.iono.get(&sat.signal_id.sat).copied();
            let amb_index = self.indices.ambiguity.get(&sat.signal_id).copied();

            let code_bias_m = self.code_bias.code_bias_m(sat.signal_id).unwrap_or(0.0);
            let phase_bias_cycles = self
                .phase_bias
                .phase_bias_cycles(sat.signal_id)
                .unwrap_or(0.0);

            if self.config.use_iono_free {
                if let Some((if_code_m, if_phase_m, f1, f2)) =
                    iono_free_from_obs(obs, sat.signal_id.sat)
                {
                    let code = PppIonoFreeCodeMeasurement {
                        z_m: if_code_m - code_bias_m,
                        sat_pos_m: [state.x_m, state.y_m, state.z_m],
                        sat_clock_s: clock_bias_s,
                        sigma_m,
                        ztd_index: Some(self.indices.ztd),
                        isb_index,
                        corr: corr.clone(),
                    };
                    if self.prefit_ok(code.z_m, &code, self.config.residual_gate_m)
                        && self.ekf.update(&code)
                    {
                        used += 1;
                    }
                    let lambda_if = SPEED_OF_LIGHT_MPS / ((f1 * f1 - f2 * f2) / (f1 - f2));
                    let phase = PppIonoFreePhaseMeasurement {
                        z_cycles: if_phase_m / lambda_if - phase_bias_cycles,
                        sat_pos_m: [state.x_m, state.y_m, state.z_m],
                        sat_clock_s: clock_bias_s,
                        sigma_cycles: 0.05,
                        ztd_index: Some(self.indices.ztd),
                        isb_index,
                        ambiguity_index: amb_index,
                        f1_hz: f1,
                        f2_hz: f2,
                        corr: corr.clone(),
                    };
                    let _ = self.ekf.update(&phase);
                }
            } else {
                let code = PppCodeMeasurement {
                    z_m: sat.pseudorange_m - code_bias_m,
                    sat_pos_m: [state.x_m, state.y_m, state.z_m],
                    sat_clock_s: clock_bias_s,
                    sigma_m,
                    iono_index,
                    ztd_index: Some(self.indices.ztd),
                    isb_index,
                    corr: corr.clone(),
                };
                if self.prefit_ok(code.z_m, &code, self.config.residual_gate_m)
                    && self.ekf.update(&code)
                {
                    used += 1;
                }
                let phase = PppPhaseMeasurement {
                    z_cycles: sat.carrier_phase_cycles - phase_bias_cycles,
                    sat_pos_m: [state.x_m, state.y_m, state.z_m],
                    sat_clock_s: clock_bias_s,
                    sigma_cycles: 0.05,
                    iono_index,
                    ztd_index: Some(self.indices.ztd),
                    isb_index,
                    ambiguity_index: amb_index,
                    corr: corr.clone(),
                    wavelength_m: SPEED_OF_LIGHT_MPS / sat.metadata.signal.carrier_hz.value(),
                };
                let _ = self.ekf.update(&phase);
            }

            residuals.push((sat.signal_id, self.ekf.health.innovation_rms));
            self.residual_history
                .entry(sat.signal_id)
                .or_default()
                .push(self.ekf.health.innovation_rms);
            if let Some(idx) = iono_index {
                self.last_seen_iono.insert(sat.signal_id.sat, obs.epoch_idx);
                if idx >= self.ekf.x.len() {
                    continue;
                }
            }
            if sat.lock_flags.cycle_slip {
                if let Some(idx) = amb_index {
                    if idx < self.ekf.p.rows() {
                        self.ekf.p[(idx, idx)] = 1e6;
                    }
                    self.indices.ambiguity.remove(&sat.signal_id);
                }
            }
            if let Some(idx) = amb_index {
                self.last_seen_amb.insert(sat.signal_id, obs.epoch_idx);
                if idx >= self.ekf.x.len() {
                    continue;
                }
            }
        }

        if slip_count >= 3 {
            self.reset("mass_slip");
        }
        if used < 4 {
            return None;
        }
        self.prune_states(obs.epoch_idx);

        let pos = [
            self.ekf.x[self.indices.pos[0]],
            self.ekf.x[self.indices.pos[1]],
            self.ekf.x[self.indices.pos[2]],
        ];
        let (sigma_h, sigma_v) = estimate_sigma(&self.ekf, &self.indices.pos);
        self.update_convergence(obs.t_rx_s, pos, sigma_h, sigma_v);
        self.check_consistency();
        Some(PppSolutionEpoch {
            epoch_idx: obs.epoch_idx,
            t_rx_s: obs.t_rx_s,
            ecef_x_m: pos[0],
            ecef_y_m: pos[1],
            ecef_z_m: pos[2],
            clock_bias_s: self.ekf.x[self.indices.clock_bias],
            rms_m: self.ekf.health.innovation_rms,
            sigma_h_m: sigma_h,
            sigma_v_m: sigma_v,
            innovation_rms: self.ekf.health.innovation_rms,
            convergence: self.health.convergence.clone(),
            residuals,
            nis_mean: self.health.nis_mean,
        })
    }

    fn sat_state(
        &self,
        products: &dyn ProductsProvider,
        eph: &GpsEphemeris,
        sat: SatId,
        t_s: f64,
    ) -> Option<(GpsSatState, f64, bool)> {
        let mut diag = crate::ProductDiagnostics::default();
        let state = products
            .sat_state(sat, t_s, &mut diag)
            .or_else(|| Some(sat_state_gps_l1ca(eph, t_s, 0.0)))?;
        let clock_bias_s = products
            .clock_bias_s(sat, t_s, &mut diag)
            .unwrap_or(state.clock_bias_s);
        let fallback = !diag.fallbacks.is_empty();
        let relativistic = state.relativistic_s;
        Some((state, clock_bias_s + relativistic, fallback))
    }

    fn ensure_states(&mut self, sats: &[&ObsSatellite]) {
        let mut new_isb: BTreeSet<Constellation> = BTreeSet::new();
        let mut new_iono: BTreeSet<SatId> = BTreeSet::new();
        let mut new_amb: BTreeSet<SigId> = BTreeSet::new();
        for sat in sats {
            if !self
                .indices
                .isb
                .contains_key(&sat.signal_id.sat.constellation)
            {
                new_isb.insert(sat.signal_id.sat.constellation);
            }
            if self.config.enable_iono_state && !self.indices.iono.contains_key(&sat.signal_id.sat)
            {
                new_iono.insert(sat.signal_id.sat);
            }
            if !self.indices.ambiguity.contains_key(&sat.signal_id) {
                new_amb.insert(sat.signal_id);
            }
        }
        for c in new_isb {
            let idx = self.ekf.x.len();
            self.ekf.add_state(&format!("isb_{c:?}"), 0.0, 1e-6);
            self.indices.isb.insert(c, idx);
        }
        for sat in new_iono {
            let idx = self.ekf.x.len();
            self.ekf.add_state(&format!("iono_{sat:?}"), 0.0, 10.0);
            self.indices.iono.insert(sat, idx);
        }
        for sig in new_amb {
            let idx = self.ekf.x.len();
            self.ekf
                .add_state(&format!("amb_{:?}_{:?}", sig.sat, sig.band), 0.0, 100.0);
            self.indices.ambiguity.insert(sig, idx);
        }
    }

    fn predict(&mut self, dt_s: f64) {
        let model = PppProcessModel {
            pos: self.indices.pos,
            vel: self.indices.vel,
            clock_bias: self.indices.clock_bias,
            clock_drift: self.indices.clock_drift,
            ztd: self.indices.ztd,
            process: self.config.process_noise.clone(),
        };
        self.ekf.predict(&model, dt_s);
    }

    fn prune_states(&mut self, epoch_idx: u64) {
        let mut pruned = 0;
        let cutoff = self.config.prune_after_epochs;
        self.last_seen_iono.retain(|sat, last| {
            let keep = epoch_idx.saturating_sub(*last) <= cutoff;
            if !keep {
                if let Some(idx) = self.indices.iono.remove(sat) {
                    if idx < self.ekf.p.rows() {
                        self.ekf.p[(idx, idx)] = 1e6;
                    }
                }
                pruned += 1;
            }
            keep
        });
        self.last_seen_amb.retain(|sig, last| {
            let keep = epoch_idx.saturating_sub(*last) <= cutoff;
            if !keep {
                if let Some(idx) = self.indices.ambiguity.remove(sig) {
                    if idx < self.ekf.p.rows() {
                        self.ekf.p[(idx, idx)] = 1e6;
                    }
                }
                pruned += 1;
            }
            keep
        });
        self.health.pruned_states += pruned;
    }

    fn update_convergence(
        &mut self,
        t_rx_s: f64,
        pos: [f64; 3],
        sigma_h: Option<f64>,
        sigma_v: Option<f64>,
    ) {
        let mut change = None;
        if let Some(prev) = self.last_pos {
            let dx = pos[0] - prev[0];
            let dy = pos[1] - prev[1];
            let dz = pos[2] - prev[2];
            change = Some((dx * dx + dy * dy + dz * dz).sqrt());
        }
        self.last_pos = Some(pos);
        let elapsed = self.epoch0_t_s.map(|t0| t_rx_s - t0).unwrap_or(0.0);
        let sigma_h_ok = sigma_h
            .map(|s| s < self.config.convergence.sigma_h_m)
            .unwrap_or(false);
        let sigma_v_ok = sigma_v
            .map(|s| s < self.config.convergence.sigma_v_m)
            .unwrap_or(false);
        let change_ok = change
            .map(|c| c / 1.0_f64.max(1e-6) < self.config.convergence.pos_rate_mps)
            .unwrap_or(false);
        let ready =
            elapsed >= self.config.convergence.min_time_s && sigma_h_ok && sigma_v_ok && change_ok;

        if ready && self.health.convergence.time_to_first_meter_s.is_none() {
            self.health.convergence.time_to_first_meter_s = Some(elapsed);
        }
        if ready
            && sigma_h.map(|s| s < 0.1).unwrap_or(false)
            && self.health.convergence.time_to_decimeter_s.is_none()
        {
            self.health.convergence.time_to_decimeter_s = Some(elapsed);
        }
        if ready
            && sigma_h.map(|s| s < 0.01).unwrap_or(false)
            && self.health.convergence.time_to_centimeter_s.is_none()
        {
            self.health.convergence.time_to_centimeter_s = Some(elapsed);
        }
        self.health.convergence.converged = ready;
        self.health.convergence.last_position_change_m = change;

        self.drift_history.push(pos);
        if self.drift_history.len() > self.config.drift_window_epochs.max(1) {
            self.drift_history.remove(0);
        }
        if self.drift_history.len() >= 2 {
            let mean = self.drift_history.iter().fold([0.0; 3], |mut acc, p| {
                acc[0] += p[0];
                acc[1] += p[1];
                acc[2] += p[2];
                acc
            });
            let n = self.drift_history.len() as f64;
            let mean = [mean[0] / n, mean[1] / n, mean[2] / n];
            let dx = pos[0] - mean[0];
            let dy = pos[1] - mean[1];
            let dz = pos[2] - mean[2];
            let drift = (dx * dx + dy * dy + dz * dz).sqrt();
            if drift > self.config.drift_threshold_m {
                self.health
                    .warnings
                    .push("long-run drift detected".to_string());
            }
        }
    }

    fn check_consistency(&mut self) {
        let nis = if let Some(pred) = self.ekf.health.predicted_variance {
            if pred > 0.0 {
                Some(self.ekf.health.innovation_rms.powi(2) / pred)
            } else {
                None
            }
        } else {
            None
        };
        self.health.nis_mean = nis;
        if let Some(nis) = nis {
            if nis > 5.0 {
                self.health
                    .warnings
                    .push("NIS high: possible under-confidence".to_string());
            } else if nis < 0.2 {
                self.health
                    .warnings
                    .push("NIS low: possible over-confidence".to_string());
            }
        }
    }

    fn prefit_ok<M: MeasurementModel>(&self, z: f64, model: &M, gate: f64) -> bool {
        let mut h = vec![0.0; model.measurement_dim()];
        model.h(&self.ekf.x, &mut h);
        let residual = (z - h[0]).abs();
        residual <= gate
    }

    pub fn reset(&mut self, reason: &str) {
        self.health.last_reset_reason = Some(reason.to_string());
        let x = vec![0.0_f64; 9];
        let p = Matrix::identity(9);
        self.ekf = Ekf::new(x, p, self.ekf.config.clone());
        self.indices.isb.clear();
        self.indices.iono.clear();
        self.indices.ambiguity.clear();
        self.last_seen_iono.clear();
        self.last_seen_amb.clear();
        self.last_pos = None;
        self.drift_history.clear();
    }
}

fn estimate_sigma(ekf: &Ekf, pos_idx: &[usize; 3]) -> (Option<f64>, Option<f64>) {
    if ekf.p.rows() <= pos_idx[2] {
        return (None, None);
    }
    let sigma_x = ekf.p[(pos_idx[0], pos_idx[0])].abs().sqrt();
    let sigma_y = ekf.p[(pos_idx[1], pos_idx[1])].abs().sqrt();
    let sigma_z = ekf.p[(pos_idx[2], pos_idx[2])].abs().sqrt();
    let sigma_h = (sigma_x * sigma_x + sigma_y * sigma_y).sqrt();
    (Some(sigma_h), Some(sigma_z))
}

#[derive(Debug, Clone)]
struct PppProcessModel {
    pos: [usize; 3],
    vel: [usize; 3],
    clock_bias: usize,
    clock_drift: usize,
    ztd: usize,
    process: PppProcessNoise,
}

impl StateModel for PppProcessModel {
    fn state_dim(&self) -> usize {
        9
    }

    fn propagate(&self, x: &mut [f64], p: &mut Matrix, dt_s: f64) {
        for i in 0..3 {
            let pos = self.pos[i];
            let vel = self.vel[i];
            if vel < x.len() && pos < x.len() {
                x[pos] += x[vel] * dt_s;
            }
        }
        if self.clock_bias < x.len() && self.clock_drift < x.len() {
            x[self.clock_bias] += x[self.clock_drift] * dt_s;
        }

        let mut q = Matrix::new(p.rows(), p.cols(), 0.0);
        if self.clock_drift < q.rows() {
            q[(self.clock_drift, self.clock_drift)] = self.process.clock_drift_s.powi(2);
        }
        if self.ztd < q.rows() {
            q[(self.ztd, self.ztd)] = self.process.ztd_m.powi(2);
        }
        for i in 0..q.rows() {
            if q[(i, i)] == 0.0 {
                q[(i, i)] = 1e-6;
            }
        }
        let f = Matrix::identity(p.rows());
        *p = f.mul(p).mul(&f.transpose()).add(&q);
    }
}

#[derive(Debug, Clone)]
struct PppCodeMeasurement {
    z_m: f64,
    sat_pos_m: [f64; 3],
    sat_clock_s: f64,
    sigma_m: f64,
    iono_index: Option<usize>,
    ztd_index: Option<usize>,
    isb_index: Option<usize>,
    corr: Corrections,
}

impl MeasurementModel for PppCodeMeasurement {
    fn name(&self) -> &'static str {
        "ppp_code"
    }

    fn kind(&self) -> MeasurementKind {
        MeasurementKind::Code
    }

    fn measurement_dim(&self) -> usize {
        1
    }

    fn observation(&self) -> &[f64] {
        std::slice::from_ref(&self.z_m)
    }

    fn h(&self, x: &[f64], out: &mut [f64]) {
        let rx = [
            x[0] + self.corr.earth_tide_m[0],
            x[1] + self.corr.earth_tide_m[1],
            x[2] + self.corr.earth_tide_m[2],
        ];
        let dx = rx[0] - self.sat_pos_m[0];
        let dy = rx[1] - self.sat_pos_m[1];
        let dz = rx[2] - self.sat_pos_m[2];
        let range = (dx * dx + dy * dy + dz * dz).sqrt();
        let mut tropo = 0.0;
        if let Some(idx) = self.ztd_index {
            if let Some(ztd) = x.get(idx) {
                tropo = *ztd;
            }
        }
        let mut iono = 0.0;
        if let Some(idx) = self.iono_index {
            if let Some(v) = x.get(idx) {
                iono = *v;
            }
        }
        let mut pred = range + SPEED_OF_LIGHT_MPS * (x[6] - self.sat_clock_s) + tropo - iono;
        if let Some(idx) = self.isb_index {
            if let Some(isb) = x.get(idx) {
                pred += SPEED_OF_LIGHT_MPS * isb;
            }
        }
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
        h[(0, 6)] = SPEED_OF_LIGHT_MPS;
        if let Some(idx) = self.ztd_index {
            if idx < h.cols() {
                h[(0, idx)] = 1.0;
            }
        }
        if let Some(idx) = self.iono_index {
            if idx < h.cols() {
                h[(0, idx)] = -1.0;
            }
        }
        if let Some(idx) = self.isb_index {
            if idx < h.cols() {
                h[(0, idx)] = SPEED_OF_LIGHT_MPS;
            }
        }
    }

    fn covariance(&self, _x: &[f64], r: &mut Matrix) {
        r[(0, 0)] = self.sigma_m * self.sigma_m;
    }
}

#[derive(Debug, Clone)]
struct PppPhaseMeasurement {
    z_cycles: f64,
    sat_pos_m: [f64; 3],
    sat_clock_s: f64,
    sigma_cycles: f64,
    iono_index: Option<usize>,
    ztd_index: Option<usize>,
    isb_index: Option<usize>,
    ambiguity_index: Option<usize>,
    corr: Corrections,
    wavelength_m: f64,
}

impl MeasurementModel for PppPhaseMeasurement {
    fn name(&self) -> &'static str {
        "ppp_phase"
    }

    fn kind(&self) -> MeasurementKind {
        MeasurementKind::Phase
    }

    fn measurement_dim(&self) -> usize {
        1
    }

    fn observation(&self) -> &[f64] {
        std::slice::from_ref(&self.z_cycles)
    }

    fn h(&self, x: &[f64], out: &mut [f64]) {
        let rx = [
            x[0] + self.corr.earth_tide_m[0],
            x[1] + self.corr.earth_tide_m[1],
            x[2] + self.corr.earth_tide_m[2],
        ];
        let dx = rx[0] - self.sat_pos_m[0];
        let dy = rx[1] - self.sat_pos_m[1];
        let dz = rx[2] - self.sat_pos_m[2];
        let range = (dx * dx + dy * dy + dz * dz).sqrt();
        let mut tropo = 0.0;
        if let Some(idx) = self.ztd_index {
            if let Some(ztd) = x.get(idx) {
                tropo = *ztd;
            }
        }
        let mut iono = 0.0;
        if let Some(idx) = self.iono_index {
            if let Some(v) = x.get(idx) {
                iono = *v;
            }
        }
        let mut pred = (range + SPEED_OF_LIGHT_MPS * (x[6] - self.sat_clock_s) + tropo - iono)
            / self.wavelength_m;
        if let Some(idx) = self.isb_index {
            if let Some(isb) = x.get(idx) {
                pred += SPEED_OF_LIGHT_MPS * isb / self.wavelength_m;
            }
        }
        if let Some(idx) = self.ambiguity_index {
            if let Some(n) = x.get(idx) {
                pred += *n;
            }
        }
        pred += self.corr.phase_windup_cycles;
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
        h[(0, 6)] = SPEED_OF_LIGHT_MPS / self.wavelength_m;
        if let Some(idx) = self.ztd_index {
            if idx < h.cols() {
                h[(0, idx)] = 1.0 / self.wavelength_m;
            }
        }
        if let Some(idx) = self.iono_index {
            if idx < h.cols() {
                h[(0, idx)] = -1.0 / self.wavelength_m;
            }
        }
        if let Some(idx) = self.isb_index {
            if idx < h.cols() {
                h[(0, idx)] = SPEED_OF_LIGHT_MPS / self.wavelength_m;
            }
        }
        if let Some(idx) = self.ambiguity_index {
            if idx < h.cols() {
                h[(0, idx)] = 1.0;
            }
        }
    }

    fn covariance(&self, _x: &[f64], r: &mut Matrix) {
        r[(0, 0)] = self.sigma_cycles * self.sigma_cycles;
    }
}

#[derive(Debug, Clone)]
struct PppIonoFreeCodeMeasurement {
    z_m: f64,
    sat_pos_m: [f64; 3],
    sat_clock_s: f64,
    sigma_m: f64,
    ztd_index: Option<usize>,
    isb_index: Option<usize>,
    corr: Corrections,
}

impl MeasurementModel for PppIonoFreeCodeMeasurement {
    fn name(&self) -> &'static str {
        "ppp_if_code"
    }

    fn kind(&self) -> MeasurementKind {
        MeasurementKind::Code
    }

    fn measurement_dim(&self) -> usize {
        1
    }

    fn observation(&self) -> &[f64] {
        std::slice::from_ref(&self.z_m)
    }

    fn h(&self, x: &[f64], out: &mut [f64]) {
        let rx = [
            x[0] + self.corr.earth_tide_m[0],
            x[1] + self.corr.earth_tide_m[1],
            x[2] + self.corr.earth_tide_m[2],
        ];
        let dx = rx[0] - self.sat_pos_m[0];
        let dy = rx[1] - self.sat_pos_m[1];
        let dz = rx[2] - self.sat_pos_m[2];
        let range = (dx * dx + dy * dy + dz * dz).sqrt();
        let mut tropo = 0.0;
        if let Some(idx) = self.ztd_index {
            if let Some(ztd) = x.get(idx) {
                tropo = *ztd;
            }
        }
        let mut pred = range + SPEED_OF_LIGHT_MPS * (x[6] - self.sat_clock_s) + tropo;
        if let Some(idx) = self.isb_index {
            if let Some(isb) = x.get(idx) {
                pred += SPEED_OF_LIGHT_MPS * isb;
            }
        }
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
        h[(0, 6)] = SPEED_OF_LIGHT_MPS;
        if let Some(idx) = self.ztd_index {
            if idx < h.cols() {
                h[(0, idx)] = 1.0;
            }
        }
        if let Some(idx) = self.isb_index {
            if idx < h.cols() {
                h[(0, idx)] = SPEED_OF_LIGHT_MPS;
            }
        }
    }

    fn covariance(&self, _x: &[f64], r: &mut Matrix) {
        r[(0, 0)] = self.sigma_m * self.sigma_m;
    }
}

#[derive(Debug, Clone)]
struct PppIonoFreePhaseMeasurement {
    z_cycles: f64,
    sat_pos_m: [f64; 3],
    sat_clock_s: f64,
    sigma_cycles: f64,
    ztd_index: Option<usize>,
    isb_index: Option<usize>,
    ambiguity_index: Option<usize>,
    f1_hz: f64,
    f2_hz: f64,
    corr: Corrections,
}

impl MeasurementModel for PppIonoFreePhaseMeasurement {
    fn name(&self) -> &'static str {
        "ppp_if_phase"
    }

    fn kind(&self) -> MeasurementKind {
        MeasurementKind::Phase
    }

    fn measurement_dim(&self) -> usize {
        1
    }

    fn observation(&self) -> &[f64] {
        std::slice::from_ref(&self.z_cycles)
    }

    fn h(&self, x: &[f64], out: &mut [f64]) {
        let rx = [
            x[0] + self.corr.earth_tide_m[0],
            x[1] + self.corr.earth_tide_m[1],
            x[2] + self.corr.earth_tide_m[2],
        ];
        let dx = rx[0] - self.sat_pos_m[0];
        let dy = rx[1] - self.sat_pos_m[1];
        let dz = rx[2] - self.sat_pos_m[2];
        let range = (dx * dx + dy * dy + dz * dz).sqrt();
        let lambda_if = SPEED_OF_LIGHT_MPS
            / ((self.f1_hz * self.f1_hz - self.f2_hz * self.f2_hz) / (self.f1_hz - self.f2_hz));
        let mut pred = (range + SPEED_OF_LIGHT_MPS * (x[6] - self.sat_clock_s)) / lambda_if;
        if let Some(idx) = self.ztd_index {
            if let Some(ztd) = x.get(idx) {
                pred += *ztd / lambda_if;
            }
        }
        if let Some(idx) = self.isb_index {
            if let Some(isb) = x.get(idx) {
                pred += SPEED_OF_LIGHT_MPS * isb / lambda_if;
            }
        }
        if let Some(idx) = self.ambiguity_index {
            if let Some(n) = x.get(idx) {
                pred += *n;
            }
        }
        pred += self.corr.phase_windup_cycles;
        out[0] = pred;
    }

    fn jacobian(&self, x: &[f64], h: &mut Matrix) {
        let dx = x[0] - self.sat_pos_m[0];
        let dy = x[1] - self.sat_pos_m[1];
        let dz = x[2] - self.sat_pos_m[2];
        let range = (dx * dx + dy * dy + dz * dz).sqrt().max(1.0);
        let lambda_if = SPEED_OF_LIGHT_MPS
            / ((self.f1_hz * self.f1_hz - self.f2_hz * self.f2_hz) / (self.f1_hz - self.f2_hz));
        h[(0, 0)] = (dx / range) / lambda_if;
        h[(0, 1)] = (dy / range) / lambda_if;
        h[(0, 2)] = (dz / range) / lambda_if;
        h[(0, 6)] = SPEED_OF_LIGHT_MPS / lambda_if;
        if let Some(idx) = self.ztd_index {
            if idx < h.cols() {
                h[(0, idx)] = 1.0 / lambda_if;
            }
        }
        if let Some(idx) = self.isb_index {
            if idx < h.cols() {
                h[(0, idx)] = SPEED_OF_LIGHT_MPS / lambda_if;
            }
        }
        if let Some(idx) = self.ambiguity_index {
            if idx < h.cols() {
                h[(0, idx)] = 1.0;
            }
        }
    }

    fn covariance(&self, _x: &[f64], r: &mut Matrix) {
        r[(0, 0)] = self.sigma_cycles * self.sigma_cycles;
    }
}

fn iono_free_from_obs(obs: &ObsEpoch, sat: SatId) -> Option<(f64, f64, f64, f64)> {
    let mut l1 = None;
    let mut l2 = None;
    for s in &obs.sats {
        if s.signal_id.sat != sat {
            continue;
        }
        match s.signal_id.band {
            SignalBand::L1 | SignalBand::E1 => l1 = Some(s),
            SignalBand::L2 | SignalBand::E5 => l2 = Some(s),
            _ => {}
        }
    }
    let l1 = l1?;
    let l2 = l2?;
    let f1 = l1.metadata.signal.carrier_hz.value();
    let f2 = l2.metadata.signal.carrier_hz.value();
    let f1_2 = f1 * f1;
    let f2_2 = f2 * f2;
    let denom = (f1_2 - f2_2).max(1.0);
    let if_code = (f1_2 * l1.pseudorange_m - f2_2 * l2.pseudorange_m) / denom;
    let lambda1 = SPEED_OF_LIGHT_MPS / f1;
    let lambda2 = SPEED_OF_LIGHT_MPS / f2;
    let phi1_m = l1.carrier_phase_cycles * lambda1;
    let phi2_m = l2.carrier_phase_cycles * lambda2;
    let if_phase = (f1_2 * phi1_m - f2_2 * phi2_m) / denom;
    Some((if_code, if_phase, f1, f2))
}
