#![allow(missing_docs)]

use bijux_gnss_core::api::{ObsEpoch, ObsSatellite, SatId};

use super::config::{PppFilter, WlAmbiguity};
use super::measurements::{ratio_fix, wide_lane_from_obs};
use crate::estimation::ekf::state::Ekf;
use crate::estimation::ekf::traits::MeasurementModel;
use crate::estimation::ppp::config::{PppArMode, PppCheckpoint};
use crate::linalg::Matrix;

impl PppFilter {
    pub fn update_convergence(
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
        let sigma_h_ok = sigma_h.map(|s| s < self.config.convergence.sigma_h_m).unwrap_or(false);
        let sigma_v_ok = sigma_v.map(|s| s < self.config.convergence.sigma_v_m).unwrap_or(false);
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
                self.health.warnings.push("long-run drift detected".to_string());
            }
        }
    }

    pub fn adapt_process_noise(&mut self) {
        let rms = self.ekf.health.innovation_rms;
        if rms > 50.0 {
            self.config.process_noise.clock_drift_s *= 1.2;
            self.config.process_noise.ztd_m *= 1.2;
            self.health.warnings.push("process noise increased".to_string());
        } else if rms < 5.0 {
            self.config.process_noise.clock_drift_s *= 0.95;
            self.config.process_noise.ztd_m *= 0.95;
            self.health.warnings.push("process noise decreased".to_string());
        }
    }

    pub fn check_consistency(&mut self) {
        let nis = self.ekf.health.normalized_innovation_squared.or_else(|| {
            self.ekf.health.predicted_variance.and_then(|predicted_variance| {
                if predicted_variance > 0.0 {
                    Some(self.ekf.health.innovation_rms.powi(2) / predicted_variance)
                } else {
                    None
                }
            })
        });
        self.health.nis_mean = nis;
        if let Some(nis) = nis {
            let lower_bound = self.ekf.health.innovation_consistency_lower_bound.unwrap_or(0.2);
            let upper_bound = self.ekf.health.innovation_consistency_upper_bound.unwrap_or(5.0);
            if nis > upper_bound {
                self.health.warnings.push(format!(
                    "NIS high: {:.3} exceeds {:.3}, possible under-confidence",
                    nis, upper_bound
                ));
            } else if nis < lower_bound {
                self.health.warnings.push(format!(
                    "NIS low: {:.3} below {:.3}, possible over-confidence",
                    nis, lower_bound
                ));
            }
        }
        if let Some(cond) = self.ekf.health.condition_number {
            if cond > 1e8 {
                self.health.warnings.push("condition number high".to_string());
            }
        }
    }

    pub fn prefit_ok<M: MeasurementModel>(&self, z: f64, model: &M, gate: f64) -> bool {
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
        self.wl_state.clear();
        self.ar_stable_epochs = 0;
    }

    pub fn checkpoint(&self) -> PppCheckpoint {
        let mut p = Vec::new();
        for r in 0..self.ekf.p.rows() {
            let mut row = Vec::new();
            for c in 0..self.ekf.p.cols() {
                row.push(self.ekf.p[(r, c)]);
            }
            p.push(row);
        }
        PppCheckpoint {
            x: self.ekf.x.clone(),
            p,
            indices_isb: self.indices.isb.iter().map(|(k, v)| (*k, *v)).collect(),
            indices_iono: self.indices.iono.iter().map(|(k, v)| (*k, *v)).collect(),
            indices_amb: self.indices.ambiguity.iter().map(|(k, v)| (*k, *v)).collect(),
            last_t_rx_s: self.last_t_rx_s,
            epoch0_t_s: self.epoch0_t_s,
            last_pos: self.last_pos,
        }
    }

    pub fn restore_from_checkpoint(&mut self, ck: PppCheckpoint) {
        let rows = ck.p.len();
        let cols = ck.p.first().map(|r| r.len()).unwrap_or(0);
        let mut mat = Matrix::new(rows, cols, 0.0);
        for r in 0..rows {
            for c in 0..cols {
                mat[(r, c)] = ck.p[r][c];
            }
        }
        self.ekf.x = ck.x;
        self.ekf.p = mat;
        self.indices.isb = ck.indices_isb.into_iter().collect();
        self.indices.iono = ck.indices_iono.into_iter().collect();
        self.indices.ambiguity = ck.indices_amb.into_iter().collect();
        self.last_t_rx_s = ck.last_t_rx_s;
        self.epoch0_t_s = ck.epoch0_t_s;
        self.last_pos = ck.last_pos;
    }

    pub fn update_wide_lane(&mut self, obs: &ObsEpoch, sats: &[&ObsSatellite]) {
        if self.config.ar_mode == PppArMode::FloatPpp {
            return;
        }
        for sat in sats {
            let Some(wide_lane) = wide_lane_from_obs(obs, sat.signal_id.sat) else {
                continue;
            };
            let entry = self.wl_state.entry(sat.signal_id.sat).or_insert(WlAmbiguity {
                float_cycles: wide_lane.cycles,
                variance: wide_lane.variance,
                fixed: false,
                last_update_epoch: obs.epoch_idx,
            });
            entry.float_cycles = wide_lane.cycles;
            entry.variance = wide_lane.variance;
            entry.last_update_epoch = obs.epoch_idx;
        }
    }

    pub fn try_fix_wide_lane(&mut self, _obs: &ObsEpoch, sats: &[&ObsSatellite]) -> usize {
        if self.config.ar_mode == PppArMode::FloatPpp {
            return 0;
        }
        let mut candidates: Vec<(SatId, f64, f64, f64)> = Vec::new();
        for sat in sats {
            if let Some(wl) = self.wl_state.get(&sat.signal_id.sat) {
                let el = sat.elevation_deg.unwrap_or(0.0);
                candidates.push((sat.signal_id.sat, wl.float_cycles, wl.variance, el));
            }
        }
        if candidates.is_empty() {
            self.health.ar_events.push("WL unavailable: no dual-frequency data".to_string());
            return 0;
        }
        if self.config.ar_use_elevation {
            candidates.sort_by(|a, b| b.3.partial_cmp(&a.3).unwrap_or(std::cmp::Ordering::Equal));
        } else {
            candidates.sort_by(|a, b| a.2.partial_cmp(&b.2).unwrap_or(std::cmp::Ordering::Equal));
        }
        candidates.truncate(self.config.ar_max_sats.max(1));
        let mut fixed_count = 0;
        for (sat, float, var, _el) in candidates {
            let (ratio, _fixed) = ratio_fix(float, var);
            self.health.ar_events.push(format!("WL float {:?} ratio {:.2}", sat, ratio));
            if ratio >= self.config.ar_ratio_threshold {
                if let Some(entry) = self.wl_state.get_mut(&sat) {
                    entry.fixed = true;
                    fixed_count += 1;
                }
                self.health.ar_events.push(format!("WL fix {:?} ratio {:.2}", sat, ratio));
            }
        }
        if fixed_count > 0 {
            self.ar_stable_epochs += 1;
            if self.ar_stable_epochs >= self.config.ar_stability_epochs {
                self.apply_wl_constraints();
            }
        } else {
            self.ar_stable_epochs = 0;
        }
        fixed_count
    }

    fn apply_wl_constraints(&mut self) {
        if self.config.ar_mode == PppArMode::FloatPpp {
            return;
        }
        for (sat, wl) in &self.wl_state {
            if !wl.fixed {
                continue;
            }
            for (sig, idx) in self.indices.ambiguity.iter() {
                if sig.sat != *sat {
                    continue;
                }
                if *idx < self.ekf.p.rows() {
                    self.ekf.p[(*idx, *idx)] *= 0.2;
                }
            }
        }
        self.ekf.sanitize_covariance();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::estimation::ppp::config::PppConfig;

    #[test]
    fn ppp_consistency_uses_ekf_nis_bounds_for_high_warning() {
        let mut filter = PppFilter::new(PppConfig::default());
        filter.ekf.health.normalized_innovation_squared = Some(8.0);
        filter.ekf.health.innovation_consistency_lower_bound = Some(0.1);
        filter.ekf.health.innovation_consistency_upper_bound = Some(6.6);

        filter.check_consistency();

        assert_eq!(filter.health.nis_mean, Some(8.0));
        assert!(filter.health.warnings.iter().any(|warning| {
            warning.contains("NIS high") && warning.contains("8.000") && warning.contains("6.600")
        }));
    }

    #[test]
    fn ppp_consistency_uses_ekf_nis_bounds_for_low_warning() {
        let mut filter = PppFilter::new(PppConfig::default());
        filter.ekf.health.normalized_innovation_squared = Some(0.05);
        filter.ekf.health.innovation_consistency_lower_bound = Some(0.1);
        filter.ekf.health.innovation_consistency_upper_bound = Some(6.6);

        filter.check_consistency();

        assert_eq!(filter.health.nis_mean, Some(0.05));
        assert!(filter.health.warnings.iter().any(|warning| {
            warning.contains("NIS low") && warning.contains("0.050") && warning.contains("0.100")
        }));
    }
}
