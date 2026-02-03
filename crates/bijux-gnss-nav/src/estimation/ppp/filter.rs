#![allow(missing_docs)]

use std::collections::{BTreeMap, BTreeSet};

use bijux_gnss_core::{Constellation, ObsEpoch, ObsSatellite, SatId, SigId};

use super::config::{PppConfig, PppFilter, PppIndices, SPEED_OF_LIGHT_MPS};
use super::measurements::{
    iono_free_from_obs, PppIonoFreeCodeMeasurement, PppIonoFreePhaseMeasurement,
    PppPhaseMeasurement,
};
use super::models::{PppCodeMeasurement, PppProcessModel};
use super::state::estimate_sigma;
use crate::corrections::biases::{CodeBiasProvider, PhaseBiasProvider, ZeroBiases};
use crate::corrections::CorrectionContext;
use crate::estimation::ekf::state::{Ekf, EkfConfig};
use crate::estimation::position::solver::{elevation_azimuth_deg, weight_from_cn0_elev};
use crate::estimation::ppp::config::{PppConvergenceState, PppHealth, PppSolutionEpoch};
use crate::formats::precise_products::{ProductDiagnostics, ProductsProvider};
use crate::linalg::Matrix;
use crate::orbits::gps::{sat_state_gps_l1ca, GpsEphemeris, GpsSatState};

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
                ar_events: Vec::new(),
            },
            code_bias: Box::new(ZeroBiases),
            phase_bias: Box::new(ZeroBiases),
            corrections: CorrectionContext,
            residual_history: BTreeMap::new(),
            drift_history: Vec::new(),
            wl_state: BTreeMap::new(),
            ar_stable_epochs: 0,
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
        self.update_wide_lane(obs, &sats);
        let fixed_wl = self.try_fix_wide_lane(obs, &sats);
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
        self.adapt_process_noise();
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
            ar_mode: self.config.ar_mode,
            fixed_wl,
        })
    }

    fn sat_state(
        &self,
        products: &dyn ProductsProvider,
        eph: &GpsEphemeris,
        sat: SatId,
        t_s: f64,
    ) -> Option<(GpsSatState, f64, bool)> {
        let mut diag = ProductDiagnostics::default();
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
}
