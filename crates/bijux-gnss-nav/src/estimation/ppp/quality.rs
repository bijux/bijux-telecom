#![allow(missing_docs)]

use bijux_gnss_core::api::{ObsEpoch, ObsSatellite, SatId};

use super::config::{PppFilter, WlAmbiguity};
use super::measurements::{ratio_fix, wide_lane_from_obs_with_phase_biases};
use crate::estimation::ekf::state::Ekf;
use crate::estimation::ekf::traits::MeasurementModel;
use crate::estimation::ppp::config::{
    PppAmbiguityResolutionEvidence, PppArMode, PppCheckpoint, PppIntegerAmbiguityCandidate,
    PppIntegerAmbiguityKind, PppLifecycleEvent, PppLifecycleEventKind,
};
use crate::linalg::Matrix;

impl PppFilter {
    pub fn update_convergence(
        &mut self,
        t_rx_s: f64,
        pos: [f64; 3],
        sigma_h: Option<f64>,
        sigma_v: Option<f64>,
        evidence: super::config::PppConvergenceEvidence,
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
        let heuristic_ready =
            elapsed >= self.config.convergence.min_time_s && sigma_h_ok && sigma_v_ok && change_ok;
        let missing_reasons = evidence.missing_reasons();
        let ready = heuristic_ready && missing_reasons.is_empty();

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
        self.health.convergence.evidence = evidence;
        self.health.convergence.missing_reasons = missing_reasons;

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
        self.validate_state_layout("consistency_check");
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
        self.health.lifecycle_events.push(PppLifecycleEvent {
            kind: PppLifecycleEventKind::ReceiverReset,
            epoch_idx: None,
            sat: None,
            signal: None,
            removed_states: self.state_identities.clone(),
            reason: reason.to_string(),
        });
        let x = vec![0.0_f64; 9];
        let p = Matrix::identity(9);
        self.ekf = Ekf::new(x, p, self.ekf.config.clone());
        self.state_identities = super::filter::base_ppp_state_identities();
        self.ekf.labels =
            self.state_identities.iter().map(super::filter::ppp_state_label).collect();
        self.indices.isb.clear();
        self.indices.iono.clear();
        self.indices.ambiguity.clear();
        self.last_t_rx_s = None;
        self.epoch0_t_s = None;
        self.last_seen_iono.clear();
        self.last_seen_amb.clear();
        self.last_pos = None;
        self.drift_history.clear();
        self.wl_state.clear();
        self.phase_windup.clear();
        self.product_support.clear();
        self.ar_stable_epochs = 0;
        self.ar_evidence = Default::default();
        self.ar_integer_ambiguities.clear();
        self.validate_state_layout("reset");
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
            state_identities: self.state_identities.clone(),
            indices_isb: self.indices.isb.iter().map(|(k, v)| (*k, *v)).collect(),
            indices_iono: self.indices.iono.iter().map(|(k, v)| (*k, *v)).collect(),
            indices_amb: self.indices.ambiguity.iter().map(|(k, v)| (*k, *v)).collect(),
            last_t_rx_s: self.last_t_rx_s,
            epoch0_t_s: self.epoch0_t_s,
            last_pos: self.last_pos,
            last_seen_iono: self.last_seen_iono.iter().map(|(k, v)| (*k, *v)).collect(),
            last_seen_amb: self.last_seen_amb.iter().map(|(k, v)| (*k, *v)).collect(),
            phase_windup: self.phase_windup.iter().map(|(k, v)| (*k, *v)).collect(),
            wl_state: self.wl_state.iter().map(|(k, v)| (*k, v.clone())).collect(),
            product_support: self.product_support.iter().map(|(k, v)| (*k, *v)).collect(),
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
        self.state_identities = if ck.state_identities.len() == self.ekf.x.len() {
            ck.state_identities
        } else {
            super::filter::state_identities_from_checkpoint_indices(
                self.ekf.x.len(),
                ck.indices_isb,
                ck.indices_iono,
                ck.indices_amb,
            )
        };
        if let Some(indices) =
            super::filter::ppp_indices_from_state_identities(&self.state_identities)
        {
            self.indices = indices;
        } else {
            self.health
                .warnings
                .push("PPP checkpoint restore produced invalid state identity layout".to_string());
        }
        self.ekf.labels =
            self.state_identities.iter().map(super::filter::ppp_state_label).collect();
        self.validate_state_layout("checkpoint_restore");
        self.last_t_rx_s = ck.last_t_rx_s;
        self.epoch0_t_s = ck.epoch0_t_s;
        self.last_pos = ck.last_pos;
        self.last_seen_iono = ck.last_seen_iono.into_iter().collect();
        self.last_seen_amb = ck.last_seen_amb.into_iter().collect();
        self.phase_windup = ck.phase_windup.into_iter().collect();
        self.wl_state = ck.wl_state.into_iter().collect();
        self.product_support = ck.product_support.into_iter().collect();
    }

    pub fn update_wide_lane(&mut self, obs: &ObsEpoch, sats: &[&ObsSatellite]) {
        if self.config.ar_mode == PppArMode::FloatPpp {
            return;
        }
        for sat in sats {
            let Some(wide_lane) = wide_lane_from_obs_with_phase_biases(
                obs,
                sat.signal_id.sat,
                Some(self.phase_bias.as_ref()),
            ) else {
                continue;
            };
            let entry = self.wl_state.entry(sat.signal_id.sat).or_insert(WlAmbiguity {
                float_cycles: wide_lane.cycles,
                variance: wide_lane.variance,
                fixed: false,
                integer_cycles: None,
                ratio: None,
                phase_bias_provenance_complete: wide_lane.phase_bias_provenance_complete,
                last_update_epoch: obs.epoch_idx,
            });
            entry.float_cycles = wide_lane.cycles;
            entry.variance = wide_lane.variance;
            entry.phase_bias_provenance_complete = wide_lane.phase_bias_provenance_complete;
            entry.last_update_epoch = obs.epoch_idx;
        }
    }

    pub fn try_fix_wide_lane(&mut self, obs: &ObsEpoch, sats: &[&ObsSatellite]) -> usize {
        self.ar_integer_ambiguities.clear();
        if self.config.ar_mode == PppArMode::FloatPpp {
            self.ar_evidence = PppAmbiguityResolutionEvidence::default();
            return 0;
        }
        let mut candidates: Vec<(SatId, f64, f64, f64, bool)> = Vec::new();
        for sat in sats {
            if let Some(wl) = self.wl_state.get(&sat.signal_id.sat) {
                let el = sat.elevation_deg.unwrap_or(0.0);
                candidates.push((
                    sat.signal_id.sat,
                    wl.float_cycles,
                    wl.variance,
                    el,
                    wl.phase_bias_provenance_complete,
                ));
            }
        }
        if candidates.is_empty() {
            self.health.ar_events.push("WL unavailable: no dual-frequency data".to_string());
            let narrow_count = if self.config.ar_mode == PppArMode::PppArNarrowLane {
                self.try_fix_narrow_lane(obs, false)
            } else {
                0
            };
            self.ar_evidence = PppAmbiguityResolutionEvidence {
                candidate_count: self.ar_integer_ambiguities.len(),
                accepted_count: narrow_count,
                ratio_threshold: self.config.ar_ratio_threshold,
                stability_epochs_required: self.config.ar_stability_epochs,
                stable_epochs: self.ar_stable_epochs,
                missing_reasons: if narrow_count == 0 {
                    vec![
                        "missing_wide_lane_candidates".to_string(),
                        "no_validated_narrow_lane_candidates".to_string(),
                    ]
                } else {
                    vec!["missing_wide_lane_candidates".to_string()]
                },
                ..PppAmbiguityResolutionEvidence::default()
            };
            return 0;
        }
        if self.config.ar_use_elevation {
            candidates.sort_by(|a, b| b.3.partial_cmp(&a.3).unwrap_or(std::cmp::Ordering::Equal));
        } else {
            candidates.sort_by(|a, b| a.2.partial_cmp(&b.2).unwrap_or(std::cmp::Ordering::Equal));
        }
        candidates.truncate(self.config.ar_max_sats.max(1));
        let mut provisional_count = 0;
        let mut evaluated = Vec::new();
        for (sat, float, var, _el, phase_bias_provenance_complete) in candidates {
            let (ratio, integer_cycles) = ratio_fix(float, var);
            self.health.ar_events.push(format!("WL float {:?} ratio {:.2}", sat, ratio));
            let mut validation_reasons = Vec::new();
            if !phase_bias_provenance_complete {
                validation_reasons.push("missing_phase_bias_provenance".to_string());
            }
            if !float.is_finite() || !var.is_finite() || var < 0.0 || !ratio.is_finite() {
                validation_reasons.push("invalid_candidate_statistics".to_string());
            }
            if ratio < self.config.ar_ratio_threshold {
                validation_reasons.push("ratio_below_threshold".to_string());
            }
            let provisional = validation_reasons.is_empty();
            if provisional {
                provisional_count += 1;
            }
            evaluated.push((
                sat,
                PppIntegerAmbiguityCandidate {
                    kind: PppIntegerAmbiguityKind::WideLane,
                    sat,
                    signal: None,
                    float_cycles: float,
                    integer_cycles,
                    variance_cycles2: var,
                    ratio,
                    accepted: false,
                    phase_bias_provenance_complete,
                    validation_reasons,
                },
            ));
        }
        if provisional_count > 0 {
            self.ar_stable_epochs += 1;
        } else {
            self.ar_stable_epochs = 0;
        }
        let stable = self.ar_stable_epochs >= self.config.ar_stability_epochs;
        let mut fixed_count = 0;
        for (sat, mut candidate) in evaluated {
            if stable && candidate.validation_reasons.is_empty() {
                candidate.accepted = true;
                fixed_count += 1;
                self.health
                    .ar_events
                    .push(format!("WL fix {:?} ratio {:.2}", sat, candidate.ratio));
            } else if candidate.validation_reasons.is_empty() {
                candidate.validation_reasons.push("stability_window_incomplete".to_string());
            }
            if let Some(entry) = self.wl_state.get_mut(&sat) {
                entry.fixed = candidate.accepted;
                entry.integer_cycles = Some(candidate.integer_cycles);
                entry.ratio = Some(candidate.ratio);
            }
            self.ar_integer_ambiguities.push(candidate);
        }
        if fixed_count > 0 {
            self.apply_wl_constraints();
        }
        let narrow_count = if self.config.ar_mode == PppArMode::PppArNarrowLane {
            self.try_fix_narrow_lane(obs, fixed_count > 0)
        } else {
            0
        };
        let all_phase_bias_provenance = self
            .ar_integer_ambiguities
            .iter()
            .all(|candidate| candidate.phase_bias_provenance_complete);
        let mut missing_reasons = Vec::new();
        if !all_phase_bias_provenance {
            missing_reasons.push("missing_phase_bias_provenance".to_string());
        }
        if fixed_count == 0 {
            missing_reasons.push("no_validated_integer_candidates".to_string());
        }
        if self.config.ar_mode == PppArMode::PppArNarrowLane && narrow_count == 0 {
            missing_reasons.push("no_validated_narrow_lane_candidates".to_string());
        }
        self.ar_evidence = PppAmbiguityResolutionEvidence {
            candidate_count: self.ar_integer_ambiguities.len(),
            accepted_count: fixed_count + narrow_count,
            phase_bias_provenance_complete: all_phase_bias_provenance,
            wide_lane_validated: fixed_count > 0,
            narrow_lane_validated: narrow_count > 0,
            ratio_threshold: self.config.ar_ratio_threshold,
            stability_epochs_required: self.config.ar_stability_epochs,
            stable_epochs: self.ar_stable_epochs,
            missing_reasons,
        };
        fixed_count
    }

    fn try_fix_narrow_lane(&mut self, obs: &ObsEpoch, wide_lane_validated: bool) -> usize {
        let mut fixed_count = 0;
        let ambiguity_states = self
            .indices
            .ambiguity
            .iter()
            .map(|(signal, index)| (*signal, *index))
            .collect::<Vec<_>>();
        for (signal, index) in ambiguity_states {
            let float_cycles = self.ekf.x.get(index).copied().unwrap_or(f64::NAN);
            let variance_cycles2 =
                if index < self.ekf.p.rows() { self.ekf.p[(index, index)] } else { f64::NAN };
            let (ratio, integer_cycles) = ratio_fix(float_cycles, variance_cycles2);
            let phase_bias_provenance_complete = self
                .phase_bias
                .phase_bias_for_ambiguity_resolution(signal, obs.gps_time())
                .is_some();
            let mut validation_reasons = Vec::new();
            if !wide_lane_validated {
                validation_reasons.push("missing_wide_lane_validation".to_string());
            }
            if !phase_bias_provenance_complete {
                validation_reasons.push("missing_phase_bias_provenance".to_string());
            }
            if !float_cycles.is_finite()
                || !variance_cycles2.is_finite()
                || variance_cycles2 < 0.0
                || !ratio.is_finite()
            {
                validation_reasons.push("invalid_candidate_statistics".to_string());
            }
            if ratio < self.config.ar_ratio_threshold {
                validation_reasons.push("ratio_below_threshold".to_string());
            }
            let accepted = validation_reasons.is_empty();
            if accepted {
                fixed_count += 1;
                if index < self.ekf.x.len() {
                    self.ekf.x[index] = integer_cycles as f64;
                }
                if index < self.ekf.p.rows() {
                    self.ekf.p[(index, index)] *= 0.05;
                }
                self.health.ar_events.push(format!("NL fix {:?} ratio {:.2}", signal, ratio));
            }
            self.ar_integer_ambiguities.push(PppIntegerAmbiguityCandidate {
                kind: PppIntegerAmbiguityKind::NarrowLane,
                sat: signal.sat,
                signal: Some(signal),
                float_cycles,
                integer_cycles,
                variance_cycles2,
                ratio,
                accepted,
                phase_bias_provenance_complete,
                validation_reasons,
            });
        }
        if fixed_count > 0 {
            self.ekf.sanitize_covariance();
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
mod convergence_tests {
    use super::*;
    use crate::estimation::ppp::config::{PppConfig, PppConvergenceEvidence};

    #[test]
    fn convergence_stays_blocked_without_strong_evidence() {
        let mut filter = PppFilter::new(PppConfig::default());
        filter.epoch0_t_s = Some(0.0);
        filter.last_pos = Some([1.0, 2.0, 3.0]);

        filter.update_convergence(
            1_200.0,
            [1.0, 2.0, 3.0],
            Some(0.05),
            Some(0.08),
            PppConvergenceEvidence {
                covariance_supported: true,
                residual_supported: true,
                ambiguity_supported: false,
                correction_supported: false,
                integrity_supported: false,
            },
        );

        assert!(!filter.health.convergence.converged);
        assert!(filter.health.convergence.time_to_first_meter_s.is_none());
        assert_eq!(
            filter.health.convergence.missing_reasons,
            vec![
                "missing_ambiguity_evidence".to_string(),
                "missing_correction_evidence".to_string(),
                "missing_integrity_evidence".to_string(),
            ]
        );
    }

    #[test]
    fn convergence_records_first_supported_epoch_once_evidence_is_complete() {
        let mut filter = PppFilter::new(PppConfig::default());
        filter.epoch0_t_s = Some(0.0);
        filter.last_pos = Some([1.0, 2.0, 3.0]);

        filter.update_convergence(
            1_200.0,
            [1.0, 2.0, 3.0],
            Some(0.05),
            Some(0.08),
            PppConvergenceEvidence {
                covariance_supported: true,
                residual_supported: true,
                ambiguity_supported: true,
                correction_supported: true,
                integrity_supported: true,
            },
        );

        assert!(filter.health.convergence.converged);
        assert_eq!(filter.health.convergence.time_to_first_meter_s, Some(1_200.0));
        assert_eq!(filter.health.convergence.time_to_decimeter_s, Some(1_200.0));
        assert!(filter.health.convergence.time_to_centimeter_s.is_none());
        assert!(filter.health.convergence.missing_reasons.is_empty());
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::corrections::phase_windup::PhaseWindupState;
    use crate::estimation::ppp::config::{
        PppConfig, PppLifecycleEventKind, PppProductSupport, PppStateIdentity, WlAmbiguity,
    };
    use crate::estimation::ppp::filter::{
        base_ppp_state_identities, ppp_indices_from_state_identities, ppp_state_label,
    };
    use bijux_gnss_core::api::{Constellation, SatId, SigId, SignalBand, SignalCode};

    fn append_ppp_state(filter: &mut PppFilter, identity: PppStateIdentity, value: f64) {
        filter.ekf.add_state(&ppp_state_label(&identity), value, 9.0);
        filter.state_identities.push(identity);
        filter.indices =
            ppp_indices_from_state_identities(&filter.state_identities).expect("PPP state layout");
    }

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

    #[test]
    fn ppp_state_layout_validation_reports_label_mismatch() {
        let mut filter = PppFilter::new(PppConfig::default());
        filter.ekf.labels.pop();

        assert!(!filter.validate_state_layout("test"));
        assert!(filter
            .health
            .warnings
            .iter()
            .any(|warning| warning.contains("PPP state label mismatch")));
    }

    #[test]
    fn ppp_state_layout_validation_reports_index_map_mismatch() {
        let sig = SigId {
            sat: SatId { constellation: Constellation::Gps, prn: 15 },
            band: SignalBand::L1,
            code: SignalCode::Ca,
        };
        let mut filter = PppFilter::new(PppConfig::default());
        append_ppp_state(&mut filter, PppStateIdentity::CarrierAmbiguity(sig), 1.0);
        filter.indices.ambiguity.insert(
            SigId {
                sat: SatId { constellation: Constellation::Gps, prn: 16 },
                band: SignalBand::L1,
                code: SignalCode::Ca,
            },
            filter.indices.ambiguity[&sig],
        );

        assert!(!filter.validate_state_layout("test"));
        assert!(filter
            .health
            .warnings
            .iter()
            .any(|warning| warning.contains("PPP state index map mismatch")));
    }

    #[test]
    fn checkpoint_preserves_phase_windup_continuity_state() {
        let sat = SatId { constellation: Constellation::Gps, prn: 7 };
        let mut filter = PppFilter::new(PppConfig::default());
        filter.phase_windup.insert(sat, PhaseWindupState { previous_cycles: Some(0.42) });

        let checkpoint = filter.checkpoint();
        let mut restored = PppFilter::new(PppConfig::default());
        restored.restore_from_checkpoint(checkpoint);

        assert_eq!(
            restored.phase_windup.get(&sat).and_then(|state| state.previous_cycles),
            Some(0.42)
        );
    }

    #[test]
    fn checkpoint_preserves_dynamic_state_lifecycle_bookkeeping() {
        let sig = SigId {
            sat: SatId { constellation: Constellation::Gps, prn: 11 },
            band: SignalBand::L1,
            code: SignalCode::Ca,
        };
        let mut filter = PppFilter::new(PppConfig::default());
        filter.last_seen_iono.insert(sig.sat, 42);
        filter.last_seen_amb.insert(sig, 43);
        filter.wl_state.insert(
            sig.sat,
            WlAmbiguity {
                float_cycles: 7.25,
                variance: 0.125,
                fixed: true,
                integer_cycles: Some(7),
                ratio: Some(9.0),
                phase_bias_provenance_complete: true,
                last_update_epoch: 44,
            },
        );
        filter
            .product_support
            .insert(sig.sat, PppProductSupport { precise_orbit: true, precise_clock: false });

        let checkpoint = filter.checkpoint();
        let mut restored = PppFilter::new(PppConfig::default());
        restored.restore_from_checkpoint(checkpoint);

        assert_eq!(restored.last_seen_iono.get(&sig.sat), Some(&42));
        assert_eq!(restored.last_seen_amb.get(&sig), Some(&43));
        let restored_wide_lane = restored.wl_state.get(&sig.sat).expect("wide-lane state");
        assert_eq!(restored_wide_lane.float_cycles, 7.25);
        assert_eq!(restored_wide_lane.variance, 0.125);
        assert!(restored_wide_lane.fixed);
        assert_eq!(restored_wide_lane.integer_cycles, Some(7));
        assert_eq!(restored_wide_lane.ratio, Some(9.0));
        assert!(restored_wide_lane.phase_bias_provenance_complete);
        assert_eq!(restored_wide_lane.last_update_epoch, 44);
        assert_eq!(
            restored.product_support.get(&sig.sat),
            Some(&PppProductSupport { precise_orbit: true, precise_clock: false })
        );
    }

    #[test]
    fn checkpoint_preserves_ppp_state_identity_layout() {
        let sig = SigId {
            sat: SatId { constellation: Constellation::Gps, prn: 7 },
            band: SignalBand::L1,
            code: SignalCode::Ca,
        };
        let mut filter = PppFilter::new(PppConfig::default());
        append_ppp_state(
            &mut filter,
            PppStateIdentity::InterSystemBias(Constellation::Galileo),
            1.0e-8,
        );
        append_ppp_state(&mut filter, PppStateIdentity::SlantIonosphere(sig.sat), 4.5);
        append_ppp_state(&mut filter, PppStateIdentity::CarrierAmbiguity(sig), 18.25);

        let checkpoint = filter.checkpoint();
        let mut restored = PppFilter::new(PppConfig::default());
        restored.restore_from_checkpoint(checkpoint);

        assert_eq!(restored.state_identities, filter.state_identities);
        assert_eq!(restored.ekf.labels.len(), restored.ekf.x.len());
        assert_eq!(
            restored.indices.isb[&Constellation::Galileo],
            filter.indices.isb[&Constellation::Galileo]
        );
        assert_eq!(restored.ekf.x[restored.indices.ambiguity[&sig]], 18.25);
        assert_eq!(
            restored.state_identities[restored.indices.iono[&sig.sat]],
            PppStateIdentity::SlantIonosphere(sig.sat)
        );
    }

    #[test]
    fn checkpoint_restore_rebuilds_state_identities_from_saved_indices() {
        let sig = SigId {
            sat: SatId { constellation: Constellation::Gps, prn: 9 },
            band: SignalBand::L1,
            code: SignalCode::Ca,
        };
        let mut filter = PppFilter::new(PppConfig::default());
        append_ppp_state(&mut filter, PppStateIdentity::SlantIonosphere(sig.sat), 2.0);
        append_ppp_state(&mut filter, PppStateIdentity::CarrierAmbiguity(sig), 12.0);
        let mut checkpoint = filter.checkpoint();
        checkpoint.state_identities.clear();

        let mut restored = PppFilter::new(PppConfig::default());
        restored.restore_from_checkpoint(checkpoint);

        assert_eq!(
            restored.state_identities[restored.indices.iono[&sig.sat]],
            PppStateIdentity::SlantIonosphere(sig.sat)
        );
        assert_eq!(
            restored.state_identities[restored.indices.ambiguity[&sig]],
            PppStateIdentity::CarrierAmbiguity(sig)
        );
        assert_eq!(
            restored.ekf.labels[restored.indices.ambiguity[&sig]],
            ppp_state_label(&PppStateIdentity::CarrierAmbiguity(sig))
        );
    }

    #[test]
    fn reset_clears_phase_windup_continuity_state() {
        let sat = SatId { constellation: Constellation::Gps, prn: 7 };
        let mut filter = PppFilter::new(PppConfig::default());
        filter.phase_windup.insert(sat, PhaseWindupState { previous_cycles: Some(0.42) });
        filter.last_t_rx_s = Some(10.0);
        filter.epoch0_t_s = Some(1.0);

        filter.reset("receiver_discontinuity");

        assert!(filter.phase_windup.is_empty());
        assert_eq!(filter.last_t_rx_s, None);
        assert_eq!(filter.epoch0_t_s, None);
        assert!(filter.health.lifecycle_events.iter().any(|event| {
            event.kind == PppLifecycleEventKind::ReceiverReset
                && event.reason == "receiver_discontinuity"
        }));
    }

    #[test]
    fn reset_restores_base_ppp_state_identity_layout() {
        let sig = SigId {
            sat: SatId { constellation: Constellation::Gps, prn: 12 },
            band: SignalBand::L1,
            code: SignalCode::Ca,
        };
        let mut filter = PppFilter::new(PppConfig::default());
        append_ppp_state(&mut filter, PppStateIdentity::CarrierAmbiguity(sig), 8.0);

        filter.reset("receiver_discontinuity");

        assert_eq!(filter.state_identities, base_ppp_state_identities());
        assert_eq!(filter.ekf.x.len(), base_ppp_state_identities().len());
        assert_eq!(filter.ekf.labels.len(), filter.ekf.x.len());
        assert!(filter.indices.ambiguity.is_empty());
        assert_eq!(
            filter.ekf.labels[filter.indices.clock_bias],
            ppp_state_label(&PppStateIdentity::ReceiverClockBias)
        );
    }
}
