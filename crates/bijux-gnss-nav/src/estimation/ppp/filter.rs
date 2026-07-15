#![allow(missing_docs)]

use std::collections::{BTreeMap, BTreeSet};

use bijux_gnss_core::api::{Constellation, ObsEpoch, ObsSatellite, SatId, SigId, SignalBand};
use bijux_gnss_signal::api::signal_wavelength_m;

use super::config::{PppConfig, PppFilter, PppIndices, PppStateIdentity};
use super::measurements::{
    iono_free_from_obs, ppp_ionosphere_delay_scale, IonoFreeCodeMeasurementObservation,
    PppIonoFreeCodeMeasurement, PppIonoFreePhaseMeasurement, PppPhaseMeasurement,
};
use super::models::{PppCodeMeasurement, PppProcessModel};
use super::state::estimate_position_uncertainty;
use crate::api::compute_corrections;
use crate::corrections::biases::{
    iono_free_code_bias_m_at, CodeBiasProvider, PhaseBiasProvider, ZeroBiases,
};
use crate::corrections::phase_windup::PhaseWindupState;
use crate::corrections::CorrectionContext;
use crate::estimation::ekf::state::{Ekf, EkfConfig};
use crate::estimation::ekf::statistics::InnovationConsistencyConfig;
use crate::estimation::position::solver::{
    ecef_to_geodetic, elevation_azimuth_deg, position_measurement_weight,
};
use crate::estimation::ppp::config::{
    PppConvergenceEvidence, PppConvergenceState, PppHealth, PppSolutionEpoch,
};
use crate::formats::precise_products::{ProductDiagnostics, ProductsProvider};
use crate::linalg::Matrix;
use crate::models::antenna::{ReceiverAntennaCalibrations, SatelliteAntennaCalibrations};
use crate::models::atmosphere::SaastamoinenModel;
use crate::models::celestial::approximate_sun_position_ecef_m;
use crate::orbits::gps::{
    gps_ephemeris_age, gps_satellite_clock_correction, sat_state_gps_l1ca, select_best_ephemeris,
    GpsEphemeris, GpsSatState,
};

fn resolved_iono_free_code_bias_m(
    provider: &dyn CodeBiasProvider,
    observation: super::measurements::IonoFreeCodeMeasurementObservation,
) -> f64 {
    iono_free_code_bias_m_at(
        provider,
        observation.signal_1,
        observation.signal_2,
        observation.f1_hz,
        observation.f2_hz,
        observation.gps_time,
    )
    .unwrap_or(0.0)
}

fn resolved_code_bias_m(
    provider: &dyn CodeBiasProvider,
    signal: SigId,
    gps_time: Option<bijux_gnss_core::api::GpsTime>,
) -> f64 {
    provider.code_bias_m_at(signal, gps_time).unwrap_or(0.0)
}

impl PppFilter {
    pub fn new(config: PppConfig) -> Self {
        let x = vec![0.0_f64; 9];
        let p = Matrix::identity(9);
        let mut ekf = Ekf::new(
            x,
            p,
            EkfConfig {
                gating_chi2_code: Some(200.0),
                gating_chi2_phase: Some(200.0),
                gating_chi2_doppler: Some(200.0),
                innovation_consistency: Some(InnovationConsistencyConfig::default()),
                huber_k: Some(10.0),
                square_root: true,
                covariance_epsilon: 1e-6,
                divergence_max_variance: 1e12,
            },
        );
        let state_identities = base_ppp_state_identities();
        ekf.labels = state_identities.iter().map(ppp_state_label).collect();
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
            state_identities,
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
                    evidence: PppConvergenceEvidence::default(),
                    missing_reasons: Vec::new(),
                },
                warnings: Vec::new(),
                nis_mean: None,
                ar_events: Vec::new(),
            },
            code_bias: Box::new(ZeroBiases),
            phase_bias: Box::new(ZeroBiases),
            corrections: CorrectionContext::default(),
            residual_history: BTreeMap::new(),
            drift_history: Vec::new(),
            wl_state: BTreeMap::new(),
            phase_windup: BTreeMap::new(),
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

    pub fn seed_receiver_state(&mut self, ecef_m: [f64; 3], clock_bias_s: f64) {
        self.ekf.x[self.indices.pos[0]] = ecef_m[0];
        self.ekf.x[self.indices.pos[1]] = ecef_m[1];
        self.ekf.x[self.indices.pos[2]] = ecef_m[2];
        self.ekf.x[self.indices.clock_bias] = clock_bias_s;
        self.try_seed_ztd_from_position_state();
        self.last_pos = Some(ecef_m);
    }

    pub fn solve_epoch(
        &mut self,
        obs: &ObsEpoch,
        ephs: &[GpsEphemeris],
        products: &dyn ProductsProvider,
    ) -> Option<PppSolutionEpoch> {
        let dt_s =
            if let Some(prev) = self.last_t_rx_s { (obs.t_rx_s.0 - prev).max(1e-3) } else { 0.001 };
        if dt_s > self.config.reset_gap_s {
            self.reset("epoch_gap");
        }
        self.last_t_rx_s = Some(obs.t_rx_s.0);
        if self.epoch0_t_s.is_none() {
            self.epoch0_t_s = Some(obs.t_rx_s.0);
        }
        self.predict(dt_s);
        self.try_seed_ztd_from_position_state();
        let receiver_state_pos_m = [
            self.ekf.x[self.indices.pos[0]],
            self.ekf.x[self.indices.pos[1]],
            self.ekf.x[self.indices.pos[2]],
        ];
        let mut correction_context = self.corrections.clone();
        if let Some(solid_earth_tide_m) =
            self.config.solid_earth_tide_displacement_m(receiver_state_pos_m, obs.gps_time())
        {
            correction_context.add_earth_tide_m(solid_earth_tide_m);
        }
        if let Some(ocean_tide_loading_m) =
            self.config.ocean_tide_loading_displacement_m(receiver_state_pos_m, obs.gps_time())
        {
            correction_context.add_earth_tide_m(ocean_tide_loading_m);
        }

        let mut sats: Vec<&ObsSatellite> = obs.sats.iter().collect();
        sats.sort_by_key(|s| s.signal_id);
        if self.config.use_iono_free {
            sats = iono_free_satellite_representatives(&sats);
        }
        self.ensure_states(&sats);
        self.update_wide_lane(obs, &sats);
        let fixed_wl = self.try_fix_wide_lane(obs, &sats);
        let corr = compute_corrections(&correction_context);
        let products_time_s = product_reference_time_s(obs);

        let mut residuals = Vec::new();
        let mut used = 0;
        let mut slip_count = 0usize;
        let mut removed_state_identities = BTreeSet::new();
        for sat in &sats {
            if sat.lock_flags.cycle_slip {
                slip_count += 1;
            }
            let eph = match select_best_ephemeris(ephs, sat.signal_id.sat, products_time_s) {
                Some(e) => e,
                None => continue,
            };
            let (state, clock_bias_s, fallback) =
                match self.sat_state(products, eph, sat.signal_id.sat, products_time_s) {
                    Some(v) => v,
                    None => continue,
                };
            if fallback {
                self.health
                    .warnings
                    .push(format!("products fallback used for {:?}", sat.signal_id.sat));
            }
            let rx_x = self.ekf.x[self.indices.pos[0]];
            let rx_y = self.ekf.x[self.indices.pos[1]];
            let rx_z = self.ekf.x[self.indices.pos[2]];
            let receiver_pos_m = [
                rx_x + corr.earth_tide_m[0],
                rx_y + corr.earth_tide_m[1],
                rx_z + corr.earth_tide_m[2],
            ];
            let (az, el) = elevation_azimuth_deg(
                receiver_pos_m[0],
                receiver_pos_m[1],
                receiver_pos_m[2],
                state.x_m,
                state.y_m,
                state.z_m,
            );
            if !el.is_finite() || el < self.config.weighting.min_elev_deg {
                continue;
            }
            let troposphere_mapping = SaastamoinenModel::mapping_factor(el);
            let weight = position_measurement_weight(
                Some(sat.cn0_dbhz),
                Some(el),
                None,
                self.config.weighting,
            );
            let sigma_m = (5.0 / weight.max(0.1)).max(1.0);

            let isb_index = self.indices.isb.get(&sat.signal_id.sat.constellation).copied();
            let iono_index = self.indices.iono.get(&sat.signal_id.sat).copied();
            let amb_index = self.indices.ambiguity.get(&sat.signal_id).copied();

            let code_bias_m =
                resolved_code_bias_m(self.code_bias.as_ref(), sat.signal_id, obs.gps_time());
            let phase_bias_cycles = self.phase_bias.phase_bias_cycles(sat.signal_id).unwrap_or(0.0);
            let ionosphere_scale = ppp_ionosphere_delay_scale(sat.metadata.signal);
            let sat_pos_m = [state.x_m, state.y_m, state.z_m];
            let mut phase_corr = corr.clone();
            phase_corr.phase_windup_cycles = phase_windup_cycles_for_satellite(
                &mut self.phase_windup,
                sat.signal_id.sat,
                receiver_pos_m,
                sat_pos_m,
                obs.gps_time(),
            );

            if self.config.use_iono_free {
                if let Some(iono_free) = iono_free_from_obs(obs, sat.signal_id.sat) {
                    let iono_free_code_bias_m = resolved_iono_free_code_bias_m(
                        self.code_bias.as_ref(),
                        IonoFreeCodeMeasurementObservation {
                            gps_time: obs.gps_time(),
                            signal_1: iono_free.signal_1,
                            signal_2: iono_free.signal_2,
                            code_m: iono_free.code_m,
                            code_sigma_m: iono_free.code_sigma_m,
                            f1_hz: iono_free.f1_hz,
                            f2_hz: iono_free.f2_hz,
                            band_1: iono_free.band_1,
                            band_2: iono_free.band_2,
                        },
                    );
                    let code = PppIonoFreeCodeMeasurement {
                        z_m: iono_free.code_m - iono_free_code_bias_m,
                        sat_pos_m,
                        sat_clock_s: clock_bias_s,
                        antenna_range_correction_m: iono_free_antenna_range_correction_m(
                            self.config.satellite_antenna_calibrations.as_ref(),
                            self.config.receiver_antenna_type.as_deref(),
                            self.config.receiver_antenna_calibrations.as_ref(),
                            sat.signal_id.sat,
                            iono_free.band_1,
                            iono_free.f1_hz,
                            iono_free.band_2,
                            iono_free.f2_hz,
                            obs.gps_time(),
                            sat_pos_m,
                            receiver_pos_m,
                            el,
                            Some(az),
                        ),
                        sigma_m: iono_free.code_sigma_m.max(sigma_m),
                        troposphere_mapping,
                        ztd_index: Some(self.indices.ztd),
                        isb_index,
                        corr: corr.clone(),
                    };
                    if self.prefit_ok(code.z_m, &code, self.config.residual_gate_m)
                        && self.ekf.update(&code)
                    {
                        used += 1;
                    }
                    let phase = PppIonoFreePhaseMeasurement {
                        z_cycles: iono_free.phase_cycles - phase_bias_cycles,
                        sat_pos_m,
                        sat_clock_s: clock_bias_s,
                        antenna_range_correction_m: iono_free_antenna_range_correction_m(
                            self.config.satellite_antenna_calibrations.as_ref(),
                            self.config.receiver_antenna_type.as_deref(),
                            self.config.receiver_antenna_calibrations.as_ref(),
                            sat.signal_id.sat,
                            iono_free.band_1,
                            iono_free.f1_hz,
                            iono_free.band_2,
                            iono_free.f2_hz,
                            obs.gps_time(),
                            sat_pos_m,
                            receiver_pos_m,
                            el,
                            Some(az),
                        ),
                        sigma_cycles: iono_free.phase_sigma_cycles.max(0.05),
                        troposphere_mapping,
                        ztd_index: Some(self.indices.ztd),
                        isb_index,
                        ambiguity_index: amb_index,
                        wavelength_m: iono_free.phase_wavelength_m,
                        corr: phase_corr.clone(),
                    };
                    let _ = self.ekf.update(&phase);
                }
            } else {
                let code = PppCodeMeasurement {
                    z_m: sat.pseudorange_m.0 - code_bias_m,
                    sat_pos_m,
                    sat_clock_s: clock_bias_s,
                    antenna_range_correction_m: single_frequency_antenna_range_correction_m(
                        self.config.satellite_antenna_calibrations.as_ref(),
                        self.config.receiver_antenna_type.as_deref(),
                        self.config.receiver_antenna_calibrations.as_ref(),
                        sat.signal_id.sat,
                        sat.signal_id.band,
                        obs.gps_time(),
                        sat_pos_m,
                        receiver_pos_m,
                        el,
                        Some(az),
                    ),
                    sigma_m,
                    troposphere_mapping,
                    ionosphere_scale,
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
                    z_cycles: sat.carrier_phase_cycles.0 - phase_bias_cycles,
                    sat_pos_m,
                    sat_clock_s: clock_bias_s,
                    antenna_range_correction_m: single_frequency_antenna_range_correction_m(
                        self.config.satellite_antenna_calibrations.as_ref(),
                        self.config.receiver_antenna_type.as_deref(),
                        self.config.receiver_antenna_calibrations.as_ref(),
                        sat.signal_id.sat,
                        sat.signal_id.band,
                        obs.gps_time(),
                        sat_pos_m,
                        receiver_pos_m,
                        el,
                        Some(az),
                    ),
                    sigma_cycles: 0.05,
                    troposphere_mapping,
                    ionosphere_scale,
                    iono_index,
                    ztd_index: Some(self.indices.ztd),
                    isb_index,
                    ambiguity_index: amb_index,
                    corr: phase_corr.clone(),
                    wavelength_m: signal_wavelength_m(sat.metadata.signal).0,
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
                    self.last_seen_amb.remove(&sat.signal_id);
                    removed_state_identities
                        .insert(PppStateIdentity::CarrierAmbiguity(sat.signal_id));
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
        self.remove_state_identities(&removed_state_identities);
        if used < 4 {
            return None;
        }
        self.prune_states(obs.epoch_idx);

        Some(self.solution_epoch(obs.epoch_idx, obs.t_rx_s.0, residuals, fixed_wl))
    }

    fn solution_epoch(
        &mut self,
        epoch_idx: u64,
        t_rx_s: f64,
        residuals: Vec<(SigId, f64)>,
        fixed_wl: usize,
    ) -> PppSolutionEpoch {
        let pos = [
            self.ekf.x[self.indices.pos[0]],
            self.ekf.x[self.indices.pos[1]],
            self.ekf.x[self.indices.pos[2]],
        ];
        let (
            position_covariance_ecef_m2,
            sigma_e_m,
            sigma_n_m,
            sigma_u_m,
            horizontal_error_ellipse_major_axis_m,
            horizontal_error_ellipse_minor_axis_m,
            horizontal_error_ellipse_azimuth_deg,
            sigma_h,
            sigma_v,
        ) = estimate_position_uncertainty(&self.ekf, &self.indices.pos, pos);
        let ztd_m = self.ekf.x.get(self.indices.ztd).copied().unwrap_or(0.0);
        let ztd_sigma_m = if self.indices.ztd < self.ekf.p.rows() {
            let variance = self.ekf.p[(self.indices.ztd, self.indices.ztd)];
            if variance.is_finite() && variance >= 0.0 {
                Some(variance.sqrt())
            } else {
                None
            }
        } else {
            None
        };
        self.check_consistency();
        self.update_convergence(
            t_rx_s,
            pos,
            sigma_h,
            sigma_v,
            ppp_convergence_evidence(
                position_covariance_ecef_m2,
                sigma_h,
                sigma_v,
                self.ekf.health.innovation_rms,
                self.health.nis_mean,
            ),
        );
        self.adapt_process_noise();
        PppSolutionEpoch {
            epoch_idx,
            t_rx_s,
            ecef_x_m: pos[0],
            ecef_y_m: pos[1],
            ecef_z_m: pos[2],
            ztd_m,
            ztd_sigma_m,
            troposphere_source: self.config.troposphere_source(),
            position_covariance_ecef_m2,
            sigma_e_m,
            sigma_n_m,
            sigma_u_m,
            horizontal_error_ellipse_major_axis_m,
            horizontal_error_ellipse_minor_axis_m,
            horizontal_error_ellipse_azimuth_deg,
            clock_bias_s: self.ekf.x[self.indices.clock_bias],
            constellation_clock_state_count: self.indices.isb.len(),
            slant_ionosphere_state_count: self.indices.iono.len(),
            carrier_ambiguity_state_count: self.indices.ambiguity.len(),
            rms_m: self.ekf.health.innovation_rms,
            sigma_h_m: sigma_h,
            sigma_v_m: sigma_v,
            innovation_rms: self.ekf.health.innovation_rms,
            convergence: self.health.convergence.clone(),
            residuals,
            nis_mean: self.health.nis_mean,
            ar_mode: self.config.ar_mode,
            fixed_wl,
        }
    }

    fn sat_state(
        &self,
        products: &dyn ProductsProvider,
        eph: &GpsEphemeris,
        sat: SatId,
        t_s: f64,
    ) -> Option<(GpsSatState, f64, bool)> {
        let mut diag = ProductDiagnostics::default();
        let state = match products.sat_state(sat, t_s, &mut diag) {
            Some(state) => state,
            None => {
                diag.fallback(format!("precise state missing for {:?}, using broadcast", sat));
                sat_state_gps_l1ca_if_current(eph, sat, t_s, &mut diag)?
            }
        };
        let clock_correction = match products.clock_correction(sat, t_s, &mut diag) {
            Some(clock_correction) => clock_correction,
            None => {
                diag.fallback(format!("precise clock missing for {:?}, using broadcast", sat));
                gps_satellite_clock_correction_if_current(eph, sat, t_s, &mut diag)?
            }
        };
        let fallback = !diag.fallbacks.is_empty();
        Some((state, clock_correction.bias_s, fallback))
    }

    fn ensure_states(&mut self, sats: &[&ObsSatellite]) {
        let mut new_isb: BTreeSet<Constellation> = BTreeSet::new();
        let mut new_iono: BTreeSet<SatId> = BTreeSet::new();
        let mut new_amb: BTreeSet<SigId> = BTreeSet::new();
        for sat in sats {
            if !self.indices.isb.contains_key(&sat.signal_id.sat.constellation) {
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
            let identity = PppStateIdentity::InterSystemBias(c);
            self.ekf.add_state(&ppp_state_label(&identity), 0.0, 1e-6);
            self.state_identities.push(identity);
            self.indices.isb.insert(c, idx);
        }
        for sat in new_iono {
            let idx = self.ekf.x.len();
            let identity = PppStateIdentity::SlantIonosphere(sat);
            self.ekf.add_state(&ppp_state_label(&identity), 0.0, 10.0);
            self.state_identities.push(identity);
            self.indices.iono.insert(sat, idx);
        }
        for sig in new_amb {
            let idx = self.ekf.x.len();
            let identity = PppStateIdentity::CarrierAmbiguity(sig);
            self.ekf.add_state(&ppp_state_label(&identity), 0.0, 100.0);
            self.state_identities.push(identity);
            self.indices.ambiguity.insert(sig, idx);
        }
        self.validate_state_layout("state_creation");
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

    fn try_seed_ztd_from_position_state(&mut self) {
        if self.indices.ztd >= self.ekf.x.len() {
            return;
        }
        if self.ekf.x[self.indices.ztd].is_finite() && self.ekf.x[self.indices.ztd] > 0.0 {
            return;
        }
        let ecef_x_m = self.ekf.x[self.indices.pos[0]];
        let ecef_y_m = self.ekf.x[self.indices.pos[1]];
        let ecef_z_m = self.ekf.x[self.indices.pos[2]];
        let radius_m = (ecef_x_m * ecef_x_m + ecef_y_m * ecef_y_m + ecef_z_m * ecef_z_m).sqrt();
        if !radius_m.is_finite() || !(6_000_000.0..=7_000_000.0).contains(&radius_m) {
            return;
        }
        let (lat_deg, lon_deg, alt_m) = ecef_to_geodetic(ecef_x_m, ecef_y_m, ecef_z_m);
        if !lat_deg.is_finite() || !lon_deg.is_finite() || !alt_m.is_finite() {
            return;
        }
        let receiver = bijux_gnss_core::api::Llh { lat_deg, lon_deg, alt_m };
        self.ekf.x[self.indices.ztd] = self
            .config
            .troposphere_meteorology()
            .map(|meteorology| {
                SaastamoinenModel::zenith_delay_with_meteorology_m(receiver, meteorology)
            })
            .unwrap_or_else(|| SaastamoinenModel::zenith_delay_m(receiver));
    }

    fn prune_states(&mut self, epoch_idx: u64) {
        let mut removed_state_identities = BTreeSet::new();
        let cutoff = self.config.prune_after_epochs;
        self.last_seen_iono.retain(|sat, last| {
            let keep = epoch_idx.saturating_sub(*last) <= cutoff;
            if !keep {
                self.indices.iono.remove(sat);
                removed_state_identities.insert(PppStateIdentity::SlantIonosphere(*sat));
            }
            keep
        });
        self.last_seen_amb.retain(|sig, last| {
            let keep = epoch_idx.saturating_sub(*last) <= cutoff;
            if !keep {
                self.indices.ambiguity.remove(sig);
                removed_state_identities.insert(PppStateIdentity::CarrierAmbiguity(*sig));
            }
            keep
        });
        let pruned = self.remove_state_identities(&removed_state_identities);
        self.health.pruned_states += pruned;
    }

    fn remove_state_identities(&mut self, removed: &BTreeSet<PppStateIdentity>) -> usize {
        if removed.is_empty() {
            return 0;
        }
        let retained_indices: Vec<usize> = self
            .state_identities
            .iter()
            .enumerate()
            .filter_map(|(index, identity)| (!removed.contains(identity)).then_some(index))
            .collect();
        let retained_identities: Vec<PppStateIdentity> = retained_indices
            .iter()
            .filter_map(|index| self.state_identities.get(*index).copied())
            .collect();
        let removed_count = self.state_identities.len().saturating_sub(retained_identities.len());
        if removed_count == 0 {
            return 0;
        }
        if !self.ekf.retain_states(&retained_indices) {
            self.health
                .warnings
                .push("PPP state compaction refused invalid retained state layout".to_string());
            return 0;
        }
        if let Some(indices) = ppp_indices_from_state_identities(&retained_identities) {
            self.indices = indices;
            self.state_identities = retained_identities;
            self.ekf.labels = self.state_identities.iter().map(ppp_state_label).collect();
            self.validate_state_layout("state_removal");
            removed_count
        } else {
            self.health
                .warnings
                .push("PPP state compaction produced invalid state identity layout".to_string());
            0
        }
    }

    pub(crate) fn validate_state_layout(&mut self, context: &str) -> bool {
        let mut valid = true;
        if self.state_identities.len() != self.ekf.x.len() {
            valid = false;
            self.health.warnings.push(format!(
                "PPP state layout mismatch during {context}: {} identities for {} states",
                self.state_identities.len(),
                self.ekf.x.len()
            ));
        }
        if self.ekf.p.rows() != self.ekf.x.len() || self.ekf.p.cols() != self.ekf.x.len() {
            valid = false;
            self.health.warnings.push(format!(
                "PPP covariance dimension mismatch during {context}: covariance is {}x{} for {} states",
                self.ekf.p.rows(),
                self.ekf.p.cols(),
                self.ekf.x.len()
            ));
        }
        if self.ekf.labels.len() != self.ekf.x.len() {
            valid = false;
            self.health.warnings.push(format!(
                "PPP state label mismatch during {context}: {} labels for {} states",
                self.ekf.labels.len(),
                self.ekf.x.len()
            ));
        }
        match ppp_indices_from_state_identities(&self.state_identities) {
            Some(indices) if indices == self.indices => {}
            Some(_) => {
                valid = false;
                self.health.warnings.push(format!(
                    "PPP state index map mismatch during {context}: identities do not match active indices"
                ));
            }
            None => {
                valid = false;
                self.health.warnings.push(format!(
                    "PPP state identity layout invalid during {context}: required receiver states missing or duplicate dynamic identities"
                ));
            }
        }
        valid
    }
}

fn ppp_convergence_evidence(
    position_covariance_ecef_m2: Option<[[f64; 3]; 3]>,
    sigma_h_m: Option<f64>,
    sigma_v_m: Option<f64>,
    innovation_rms_m: f64,
    nis_mean: Option<f64>,
) -> PppConvergenceEvidence {
    let covariance_supported = position_covariance_ecef_m2.is_some_and(|covariance| {
        covariance.iter().flat_map(|row| row.iter()).all(|value| value.is_finite())
    }) && sigma_h_m
        .is_some_and(|value| value.is_finite() && value > 0.0)
        && sigma_v_m.is_some_and(|value| value.is_finite() && value > 0.0);
    let residual_supported = innovation_rms_m.is_finite()
        && innovation_rms_m >= 0.0
        && nis_mean.is_some_and(|value| value.is_finite() && (0.2..=5.0).contains(&value));

    PppConvergenceEvidence {
        covariance_supported,
        residual_supported,
        ambiguity_supported: false,
        correction_supported: false,
        integrity_supported: false,
    }
}

fn iono_free_satellite_representatives<'a>(sats: &[&'a ObsSatellite]) -> Vec<&'a ObsSatellite> {
    let mut seen = BTreeSet::new();
    let mut representatives = Vec::new();
    for sat in sats {
        if seen.insert(sat.signal_id.sat) {
            representatives.push(*sat);
        }
    }
    representatives
}

fn product_reference_time_s(obs: &ObsEpoch) -> f64 {
    obs.gps_time().map(|time| time.tow_s).unwrap_or(obs.t_rx_s.0)
}

pub(crate) fn base_ppp_state_identities() -> Vec<PppStateIdentity> {
    vec![
        PppStateIdentity::ReceiverPositionX,
        PppStateIdentity::ReceiverPositionY,
        PppStateIdentity::ReceiverPositionZ,
        PppStateIdentity::ReceiverVelocityX,
        PppStateIdentity::ReceiverVelocityY,
        PppStateIdentity::ReceiverVelocityZ,
        PppStateIdentity::ReceiverClockBias,
        PppStateIdentity::ReceiverClockDrift,
        PppStateIdentity::ZenithTroposphereDelay,
    ]
}

pub(crate) fn ppp_state_label(identity: &PppStateIdentity) -> String {
    match identity {
        PppStateIdentity::ReceiverPositionX => "receiver_position_x_m".to_string(),
        PppStateIdentity::ReceiverPositionY => "receiver_position_y_m".to_string(),
        PppStateIdentity::ReceiverPositionZ => "receiver_position_z_m".to_string(),
        PppStateIdentity::ReceiverVelocityX => "receiver_velocity_x_mps".to_string(),
        PppStateIdentity::ReceiverVelocityY => "receiver_velocity_y_mps".to_string(),
        PppStateIdentity::ReceiverVelocityZ => "receiver_velocity_z_mps".to_string(),
        PppStateIdentity::ReceiverClockBias => "receiver_clock_bias_s".to_string(),
        PppStateIdentity::ReceiverClockDrift => "receiver_clock_drift_s_per_s".to_string(),
        PppStateIdentity::ZenithTroposphereDelay => "zenith_troposphere_delay_m".to_string(),
        PppStateIdentity::InterSystemBias(constellation) => {
            format!("inter_system_bias_{}_s", constellation_label(*constellation))
        }
        PppStateIdentity::SlantIonosphere(sat) => {
            format!("slant_ionosphere_{}_{:02}_m", constellation_label(sat.constellation), sat.prn)
        }
        PppStateIdentity::CarrierAmbiguity(sig) => {
            format!(
                "carrier_ambiguity_{}_{:02}_{}_{}_cycles",
                constellation_label(sig.sat.constellation),
                sig.sat.prn,
                signal_band_label(sig.band),
                signal_code_label(sig.code)
            )
        }
    }
}

fn constellation_label(constellation: Constellation) -> &'static str {
    match constellation {
        Constellation::Gps => "gps",
        Constellation::Glonass => "glonass",
        Constellation::Galileo => "galileo",
        Constellation::Beidou => "beidou",
        Constellation::Unknown => "unknown",
    }
}

fn signal_band_label(band: SignalBand) -> &'static str {
    match band {
        SignalBand::L1 => "l1",
        SignalBand::L2 => "l2",
        SignalBand::L5 => "l5",
        SignalBand::E1 => "e1",
        SignalBand::E5 => "e5",
        SignalBand::B1 => "b1",
        SignalBand::B2 => "b2",
        SignalBand::Unknown => "unknown_band",
    }
}

fn signal_code_label(code: bijux_gnss_core::api::SignalCode) -> &'static str {
    match code {
        bijux_gnss_core::api::SignalCode::Ca => "ca",
        bijux_gnss_core::api::SignalCode::L2C => "l2c",
        bijux_gnss_core::api::SignalCode::L5I => "l5i",
        bijux_gnss_core::api::SignalCode::L5Q => "l5q",
        bijux_gnss_core::api::SignalCode::Py => "py",
        bijux_gnss_core::api::SignalCode::E1B => "e1b",
        bijux_gnss_core::api::SignalCode::E1C => "e1c",
        bijux_gnss_core::api::SignalCode::E5a => "e5a",
        bijux_gnss_core::api::SignalCode::E5b => "e5b",
        bijux_gnss_core::api::SignalCode::B1I => "b1i",
        bijux_gnss_core::api::SignalCode::B2I => "b2i",
        bijux_gnss_core::api::SignalCode::Unknown => "unknown_code",
    }
}

pub(crate) fn ppp_indices_from_state_identities(
    identities: &[PppStateIdentity],
) -> Option<PppIndices> {
    let mut indices = PppIndices {
        pos: [usize::MAX; 3],
        vel: [usize::MAX; 3],
        clock_bias: usize::MAX,
        clock_drift: usize::MAX,
        ztd: usize::MAX,
        isb: BTreeMap::new(),
        iono: BTreeMap::new(),
        ambiguity: BTreeMap::new(),
    };

    for (index, identity) in identities.iter().enumerate() {
        match identity {
            PppStateIdentity::ReceiverPositionX => indices.pos[0] = index,
            PppStateIdentity::ReceiverPositionY => indices.pos[1] = index,
            PppStateIdentity::ReceiverPositionZ => indices.pos[2] = index,
            PppStateIdentity::ReceiverVelocityX => indices.vel[0] = index,
            PppStateIdentity::ReceiverVelocityY => indices.vel[1] = index,
            PppStateIdentity::ReceiverVelocityZ => indices.vel[2] = index,
            PppStateIdentity::ReceiverClockBias => indices.clock_bias = index,
            PppStateIdentity::ReceiverClockDrift => indices.clock_drift = index,
            PppStateIdentity::ZenithTroposphereDelay => indices.ztd = index,
            PppStateIdentity::InterSystemBias(constellation) => {
                if indices.isb.insert(*constellation, index).is_some() {
                    return None;
                }
            }
            PppStateIdentity::SlantIonosphere(sat) => {
                if indices.iono.insert(*sat, index).is_some() {
                    return None;
                }
            }
            PppStateIdentity::CarrierAmbiguity(sig) => {
                if indices.ambiguity.insert(*sig, index).is_some() {
                    return None;
                }
            }
        }
    }

    (indices.pos.iter().all(|index| *index != usize::MAX)
        && indices.vel.iter().all(|index| *index != usize::MAX)
        && indices.clock_bias != usize::MAX
        && indices.clock_drift != usize::MAX
        && indices.ztd != usize::MAX)
        .then_some(indices)
}

pub(crate) fn state_identities_from_checkpoint_indices(
    state_len: usize,
    indices_isb: Vec<(Constellation, usize)>,
    indices_iono: Vec<(SatId, usize)>,
    indices_amb: Vec<(SigId, usize)>,
) -> Vec<PppStateIdentity> {
    let mut identities = vec![None; state_len];
    for (index, identity) in base_ppp_state_identities().into_iter().enumerate().take(state_len) {
        identities[index] = Some(identity);
    }
    for (constellation, index) in indices_isb {
        if index < state_len {
            identities[index] = Some(PppStateIdentity::InterSystemBias(constellation));
        }
    }
    for (sat, index) in indices_iono {
        if index < state_len {
            identities[index] = Some(PppStateIdentity::SlantIonosphere(sat));
        }
    }
    for (sig, index) in indices_amb {
        if index < state_len {
            identities[index] = Some(PppStateIdentity::CarrierAmbiguity(sig));
        }
    }

    identities
        .into_iter()
        .enumerate()
        .map(|(index, identity)| {
            identity.unwrap_or_else(|| {
                PppStateIdentity::CarrierAmbiguity(checkpoint_signal_identity(index))
            })
        })
        .collect()
}

fn checkpoint_signal_identity(index: usize) -> SigId {
    SigId {
        sat: SatId {
            constellation: Constellation::Unknown,
            prn: index.min(u8::MAX as usize) as u8,
        },
        band: SignalBand::Unknown,
        code: bijux_gnss_core::api::SignalCode::Unknown,
    }
}

fn phase_windup_cycles_for_satellite(
    phase_windup: &mut BTreeMap<SatId, PhaseWindupState>,
    sat: SatId,
    receiver_pos_m: [f64; 3],
    sat_pos_m: [f64; 3],
    gps_time: Option<bijux_gnss_core::api::GpsTime>,
) -> f64 {
    let Some(gps_time) = gps_time else {
        return 0.0;
    };
    let sun_pos_m = approximate_sun_position_ecef_m(gps_time);
    phase_windup
        .entry(sat)
        .or_default()
        .apply_geometry(receiver_pos_m, sat_pos_m, sun_pos_m)
        .map(|correction| correction.continuous_cycles)
        .unwrap_or(0.0)
}

fn single_frequency_antenna_range_correction_m(
    satellite_calibrations: Option<&SatelliteAntennaCalibrations>,
    receiver_antenna_type: Option<&str>,
    receiver_calibrations: Option<&ReceiverAntennaCalibrations>,
    sat: SatId,
    band: SignalBand,
    gps_time: Option<bijux_gnss_core::api::GpsTime>,
    sat_pos_m: [f64; 3],
    receiver_pos_m: [f64; 3],
    elevation_deg: f64,
    azimuth_deg: Option<f64>,
) -> f64 {
    satellite_calibrations
        .and_then(|calibrations| {
            calibrations.range_correction_with_phase_variation_m(
                sat,
                band,
                gps_time,
                sat_pos_m,
                receiver_pos_m,
                elevation_deg,
                azimuth_deg,
            )
        })
        .unwrap_or(0.0)
        + single_frequency_receiver_antenna_range_correction_m(
            receiver_antenna_type,
            receiver_calibrations,
            band,
            gps_time,
            receiver_pos_m,
            sat_pos_m,
            elevation_deg,
            azimuth_deg,
        )
}

fn iono_free_antenna_range_correction_m(
    satellite_calibrations: Option<&SatelliteAntennaCalibrations>,
    receiver_antenna_type: Option<&str>,
    receiver_calibrations: Option<&ReceiverAntennaCalibrations>,
    sat: SatId,
    band_1: SignalBand,
    f1_hz: f64,
    band_2: SignalBand,
    f2_hz: f64,
    gps_time: Option<bijux_gnss_core::api::GpsTime>,
    sat_pos_m: [f64; 3],
    receiver_pos_m: [f64; 3],
    elevation_deg: f64,
    azimuth_deg: Option<f64>,
) -> f64 {
    satellite_calibrations
        .and_then(|calibrations| {
            calibrations.iono_free_range_correction_with_phase_variation_m(
                sat,
                band_1,
                f1_hz,
                band_2,
                f2_hz,
                gps_time,
                sat_pos_m,
                receiver_pos_m,
                elevation_deg,
                azimuth_deg,
            )
        })
        .unwrap_or(0.0)
        + iono_free_receiver_antenna_range_correction_m(
            receiver_antenna_type,
            receiver_calibrations,
            band_1,
            f1_hz,
            band_2,
            f2_hz,
            gps_time,
            receiver_pos_m,
            sat_pos_m,
            elevation_deg,
            azimuth_deg,
        )
}

fn single_frequency_receiver_antenna_range_correction_m(
    receiver_antenna_type: Option<&str>,
    calibrations: Option<&ReceiverAntennaCalibrations>,
    band: SignalBand,
    gps_time: Option<bijux_gnss_core::api::GpsTime>,
    receiver_pos_m: [f64; 3],
    sat_pos_m: [f64; 3],
    elevation_deg: f64,
    azimuth_deg: Option<f64>,
) -> f64 {
    let Some(receiver_antenna_type) = receiver_antenna_type else {
        return 0.0;
    };
    calibrations
        .and_then(|calibrations| {
            calibrations.range_correction_with_phase_variation_m(
                receiver_antenna_type,
                band,
                gps_time,
                receiver_pos_m,
                sat_pos_m,
                elevation_deg,
                azimuth_deg,
            )
        })
        .unwrap_or(0.0)
}

fn iono_free_receiver_antenna_range_correction_m(
    receiver_antenna_type: Option<&str>,
    calibrations: Option<&ReceiverAntennaCalibrations>,
    band_1: SignalBand,
    f1_hz: f64,
    band_2: SignalBand,
    f2_hz: f64,
    gps_time: Option<bijux_gnss_core::api::GpsTime>,
    receiver_pos_m: [f64; 3],
    sat_pos_m: [f64; 3],
    elevation_deg: f64,
    azimuth_deg: Option<f64>,
) -> f64 {
    let Some(receiver_antenna_type) = receiver_antenna_type else {
        return 0.0;
    };
    calibrations
        .and_then(|calibrations| {
            calibrations.iono_free_range_correction_with_phase_variation_m(
                receiver_antenna_type,
                band_1,
                f1_hz,
                band_2,
                f2_hz,
                gps_time,
                receiver_pos_m,
                sat_pos_m,
                elevation_deg,
                azimuth_deg,
            )
        })
        .unwrap_or(0.0)
}

fn sat_state_gps_l1ca_if_current(
    eph: &GpsEphemeris,
    sat: SatId,
    t_s: f64,
    diag: &mut ProductDiagnostics,
) -> Option<GpsSatState> {
    validate_broadcast_ephemeris(eph, sat, t_s, diag)?;
    Some(sat_state_gps_l1ca(eph, t_s, 0.0))
}

fn gps_satellite_clock_correction_if_current(
    eph: &GpsEphemeris,
    sat: SatId,
    t_s: f64,
    diag: &mut ProductDiagnostics,
) -> Option<crate::orbits::gps::GpsSatelliteClockCorrection> {
    validate_broadcast_ephemeris(eph, sat, t_s, diag)?;
    Some(gps_satellite_clock_correction(eph, t_s))
}

fn validate_broadcast_ephemeris(
    eph: &GpsEphemeris,
    sat: SatId,
    t_s: f64,
    diag: &mut ProductDiagnostics,
) -> Option<()> {
    let age = gps_ephemeris_age(eph, t_s);
    if age.is_stale() {
        diag.fallback(format!(
            "broadcast ephemeris stale for {:?} at {:.3}s (toe_age_s={:.3}, toc_age_s={:.3}, limit_s={:.3})",
            sat, t_s, age.toe_age_s, age.toc_age_s, age.max_age_s
        ));
        return None;
    }
    Some(())
}

#[cfg(test)]
mod tests {
    use std::collections::{BTreeMap, BTreeSet};

    use super::{
        iono_free_antenna_range_correction_m, iono_free_satellite_representatives,
        phase_windup_cycles_for_satellite, resolved_code_bias_m, resolved_iono_free_code_bias_m,
        single_frequency_antenna_range_correction_m, PppFilter,
    };
    use crate::api::{
        ecef_to_geodetic, elevation_azimuth_deg, geodetic_to_ecef, BroadcastProductsProvider,
        GpsEphemeris, GpsSatState, GpsSatelliteClockCorrection, OceanTideConstituent,
        OceanTideLoadingConstituent, OceanTideLoadingModel, PppConfig, ProductDiagnostics,
        ProductsProvider, SatelliteStateUncertainty, SolidEarthTideModel,
    };
    use crate::corrections::biases::{CodeBias, CodeBiasProvider, SignalCodeBiases};
    use crate::estimation::ppp::measurements::iono_free_code_observation_from_obs;
    use crate::models::antenna::{
        AntennaPhaseCenterVariation, ReceiverAntennaCalibration, ReceiverAntennaCalibrations,
        ReceiverPhaseCenterOffset, SatelliteAntennaCalibration, SatelliteAntennaCalibrations,
        SatellitePhaseCenterOffset,
    };
    use crate::models::atmosphere::SaastamoinenModel;
    use bijux_gnss_core::api::{
        Constellation, Cycles, GpsTime, Hertz, Llh, LockFlags, Meters, ObsEpoch, ObsMetadata,
        ObsSatellite, ObservationEpochDecision, ObservationStatus, ReceiverRole,
        ReceiverSampleTrace, SatId, Seconds, SigId, SignalBand, SignalCode,
    };
    use bijux_gnss_signal::api::{
        signal_spec_gps_l1_ca, signal_spec_gps_l2_py, signal_wavelength_m,
    };

    #[derive(Debug, Clone)]
    struct StubProductsProvider {
        state: GpsSatState,
        precise_clock_bias_s: Option<f64>,
    }

    impl ProductsProvider for StubProductsProvider {
        fn sat_state(
            &self,
            _sat: SatId,
            _t_s: f64,
            _diag: &mut ProductDiagnostics,
        ) -> Option<GpsSatState> {
            Some(self.state.clone())
        }

        fn clock_correction(
            &self,
            _sat: SatId,
            _t_s: f64,
            _diag: &mut ProductDiagnostics,
        ) -> Option<GpsSatelliteClockCorrection> {
            self.precise_clock_bias_s.map(GpsSatelliteClockCorrection::from_bias_s)
        }

        fn coverage_s(&self, _sat: SatId) -> Option<(f64, f64)> {
            None
        }
    }

    #[derive(Debug, Clone)]
    struct MappedProductsProvider {
        states: BTreeMap<SatId, GpsSatState>,
    }

    impl ProductsProvider for MappedProductsProvider {
        fn sat_state(
            &self,
            sat: SatId,
            _t_s: f64,
            _diag: &mut ProductDiagnostics,
        ) -> Option<GpsSatState> {
            self.states.get(&sat).cloned()
        }

        fn clock_correction(
            &self,
            sat: SatId,
            _t_s: f64,
            _diag: &mut ProductDiagnostics,
        ) -> Option<GpsSatelliteClockCorrection> {
            self.states.get(&sat).map(|state| state.clock_correction.clone())
        }

        fn coverage_s(&self, _sat: SatId) -> Option<(f64, f64)> {
            None
        }
    }

    fn make_eph(prn: u8) -> GpsEphemeris {
        GpsEphemeris {
            sat: SatId { constellation: Constellation::Gps, prn },
            iodc: 0,
            iode: 0,
            week: 0,
            sv_health: 0,
            sv_accuracy: Some(2),
            toe_s: 0.0,
            toc_s: 0.0,
            sqrt_a: 5153.7954775,
            e: 0.02,
            i0: 0.94,
            idot: 0.0,
            omega0: 0.1,
            omegadot: 0.0,
            w: 0.2,
            m0: 0.3,
            delta_n: 0.0,
            cuc: 0.0,
            cus: 0.0,
            crc: 0.0,
            crs: 0.0,
            cic: 0.0,
            cis: 0.0,
            af0: 1.0e-4,
            af1: -2.0e-12,
            af2: 3.0e-20,
            tgd: 8.0e-9,
        }
    }

    fn enu_offset_to_ecef(base_ecef_m: [f64; 3], enu_m: [f64; 3]) -> [f64; 3] {
        let (lat_deg, lon_deg, _alt_m) =
            ecef_to_geodetic(base_ecef_m[0], base_ecef_m[1], base_ecef_m[2]);
        let (sin_lat, cos_lat) = lat_deg.to_radians().sin_cos();
        let (sin_lon, cos_lon) = lon_deg.to_radians().sin_cos();
        let east_m = enu_m[0];
        let north_m = enu_m[1];
        let up_m = enu_m[2];
        let dx = -sin_lon * east_m - sin_lat * cos_lon * north_m + cos_lat * cos_lon * up_m;
        let dy = cos_lon * east_m - sin_lat * sin_lon * north_m + cos_lat * sin_lon * up_m;
        let dz = cos_lat * north_m + sin_lat * up_m;
        [base_ecef_m[0] + dx, base_ecef_m[1] + dy, base_ecef_m[2] + dz]
    }

    fn euclidean_distance_m(lhs: [f64; 3], rhs: [f64; 3]) -> f64 {
        let dx = lhs[0] - rhs[0];
        let dy = lhs[1] - rhs[1];
        let dz = lhs[2] - rhs[2];
        (dx * dx + dy * dy + dz * dz).sqrt()
    }

    fn make_static_ppp_epoch(
        epoch_idx: u64,
        gps_time: GpsTime,
        receiver_ecef_m: [f64; 3],
        satellite_positions_m: &[(SatId, [f64; 3])],
        ztd_m: f64,
    ) -> ObsEpoch {
        let signal = signal_spec_gps_l1_ca();
        let wavelength_m = signal_wavelength_m(signal).0;
        let sats = satellite_positions_m
            .iter()
            .map(|(sat, sat_pos_m)| {
                let range_m = euclidean_distance_m(receiver_ecef_m, *sat_pos_m);
                let (_az_deg, elevation_deg) = elevation_azimuth_deg(
                    receiver_ecef_m[0],
                    receiver_ecef_m[1],
                    receiver_ecef_m[2],
                    sat_pos_m[0],
                    sat_pos_m[1],
                    sat_pos_m[2],
                );
                let troposphere_mapping = SaastamoinenModel::mapping_factor(elevation_deg);
                let pseudorange_m = range_m + ztd_m * troposphere_mapping;
                ObsSatellite {
                    signal_id: SigId { sat: *sat, band: SignalBand::L1, code: SignalCode::Ca },
                    pseudorange_m: Meters(pseudorange_m),
                    pseudorange_var_m2: 0.01,
                    carrier_phase_cycles: Cycles(pseudorange_m / wavelength_m),
                    carrier_phase_var_cycles2: 1.0e-4,
                    doppler_hz: Hertz(0.0),
                    doppler_var_hz2: 1.0,
                    cn0_dbhz: 48.0,
                    lock_flags: LockFlags {
                        code_lock: true,
                        carrier_lock: true,
                        bit_lock: true,
                        cycle_slip: false,
                    },
                    multipath_suspect: false,
                    observation_status: ObservationStatus::Accepted,
                    observation_reject_reasons: Vec::new(),
                    elevation_deg: Some(elevation_deg),
                    azimuth_deg: None,
                    weight: Some(1.0),
                    timing: None,
                    error_model: None,
                    metadata: ObsMetadata {
                        tracking_mode: "test".to_string(),
                        integration_ms: 1,
                        lock_quality: 48.0,
                        smoothing_window: 0,
                        smoothing_age: 0,
                        smoothing_resets: 0,
                        signal,
                        ..ObsMetadata::default()
                    },
                }
            })
            .collect();
        ObsEpoch {
            t_rx_s: Seconds(gps_time.to_seconds()),
            source_time: ReceiverSampleTrace::from_sample_index(epoch_idx, 1_000.0),
            gps_week: Some(gps_time.week),
            tow_s: Some(Seconds(gps_time.tow_s)),
            epoch_idx,
            discontinuity: false,
            valid: true,
            processing_ms: None,
            role: ReceiverRole::Rover,
            sats,
            decision: ObservationEpochDecision::Accepted,
            decision_reason: Some("accepted_observables_present".to_string()),
            manifest: None,
        }
    }

    fn ppp_test_satellite(sat: SatId) -> ObsSatellite {
        ObsSatellite {
            signal_id: SigId { sat, band: SignalBand::L1, code: SignalCode::Ca },
            pseudorange_m: Meters(20_000_000.0),
            pseudorange_var_m2: 1.0,
            carrier_phase_cycles: Cycles(100.0),
            carrier_phase_var_cycles2: 0.01,
            doppler_hz: Hertz(0.0),
            doppler_var_hz2: 1.0,
            cn0_dbhz: 45.0,
            lock_flags: LockFlags {
                code_lock: true,
                carrier_lock: true,
                bit_lock: false,
                cycle_slip: false,
            },
            multipath_suspect: false,
            observation_status: ObservationStatus::Accepted,
            observation_reject_reasons: Vec::new(),
            elevation_deg: None,
            azimuth_deg: None,
            weight: None,
            timing: None,
            error_model: None,
            metadata: ObsMetadata {
                tracking_mode: "test".to_string(),
                integration_ms: 1,
                lock_quality: 45.0,
                smoothing_window: 0,
                smoothing_age: 0,
                smoothing_resets: 0,
                signal: signal_spec_gps_l1_ca(),
                ..ObsMetadata::default()
            },
        }
    }

    fn ppp_test_signal_satellite(sat: SatId, band: SignalBand, code: SignalCode) -> ObsSatellite {
        let mut observation = ppp_test_satellite(sat);
        observation.signal_id = SigId { sat, band, code };
        observation.metadata.signal = match (band, code) {
            (SignalBand::L1, SignalCode::Ca) => signal_spec_gps_l1_ca(),
            (SignalBand::L2, SignalCode::Py) => signal_spec_gps_l2_py(),
            _ => observation.metadata.signal,
        };
        observation
    }

    #[test]
    fn ppp_filter_aligns_dynamic_state_identities_with_indices() {
        let gps = ppp_test_satellite(SatId { constellation: Constellation::Gps, prn: 7 });
        let galileo = ppp_test_satellite(SatId { constellation: Constellation::Galileo, prn: 11 });
        let mut filter =
            PppFilter::new(PppConfig { enable_iono_state: true, ..PppConfig::default() });

        filter.ensure_states(&[&gps, &galileo]);

        assert_eq!(filter.ekf.x.len(), filter.state_identities.len());
        assert_eq!(filter.ekf.labels.len(), filter.state_identities.len());
        for (constellation, index) in &filter.indices.isb {
            assert_eq!(
                filter.state_identities[*index],
                super::PppStateIdentity::InterSystemBias(*constellation)
            );
            assert_eq!(
                filter.ekf.labels[*index],
                super::ppp_state_label(&filter.state_identities[*index])
            );
        }
        for (sat, index) in &filter.indices.iono {
            assert_eq!(
                filter.state_identities[*index],
                super::PppStateIdentity::SlantIonosphere(*sat)
            );
        }
        for (sig, index) in &filter.indices.ambiguity {
            assert_eq!(
                filter.state_identities[*index],
                super::PppStateIdentity::CarrierAmbiguity(*sig)
            );
        }
    }

    #[test]
    fn ppp_filter_creates_uncombined_dual_frequency_states() {
        let sat = SatId { constellation: Constellation::Gps, prn: 7 };
        let l1 = ppp_test_signal_satellite(sat, SignalBand::L1, SignalCode::Ca);
        let l2 = ppp_test_signal_satellite(sat, SignalBand::L2, SignalCode::Py);
        let mut filter = PppFilter::new(PppConfig {
            use_iono_free: false,
            enable_iono_state: true,
            ..PppConfig::default()
        });

        filter.ensure_states(&[&l1, &l2]);

        assert_eq!(filter.indices.iono.len(), 1);
        assert!(filter.indices.iono.contains_key(&sat));
        assert_eq!(filter.indices.ambiguity.len(), 2);
        assert!(filter.indices.ambiguity.contains_key(&l1.signal_id));
        assert!(filter.indices.ambiguity.contains_key(&l2.signal_id));
        assert_eq!(
            filter.state_identities[filter.indices.iono[&sat]],
            super::PppStateIdentity::SlantIonosphere(sat)
        );
        assert_eq!(
            filter.state_identities[filter.indices.ambiguity[&l1.signal_id]],
            super::PppStateIdentity::CarrierAmbiguity(l1.signal_id)
        );
        assert_eq!(
            filter.state_identities[filter.indices.ambiguity[&l2.signal_id]],
            super::PppStateIdentity::CarrierAmbiguity(l2.signal_id)
        );
    }

    #[test]
    fn ppp_solution_reports_uncombined_state_support() {
        let sat = SatId { constellation: Constellation::Gps, prn: 7 };
        let l1 = ppp_test_signal_satellite(sat, SignalBand::L1, SignalCode::Ca);
        let l2 = ppp_test_signal_satellite(sat, SignalBand::L2, SignalCode::Py);
        let mut filter = PppFilter::new(PppConfig {
            use_iono_free: false,
            enable_iono_state: true,
            ..PppConfig::default()
        });
        filter.ensure_states(&[&l1, &l2]);

        let solution = filter.solution_epoch(5, 12.0, Vec::new(), 0);

        assert_eq!(solution.constellation_clock_state_count, 1);
        assert_eq!(solution.slant_ionosphere_state_count, 1);
        assert_eq!(solution.carrier_ambiguity_state_count, 2);
    }

    #[test]
    fn ppp_filter_compacts_removed_satellite_states_without_covariance_index_corruption() {
        let stale = ppp_test_satellite(SatId { constellation: Constellation::Gps, prn: 7 });
        let retained = ppp_test_satellite(SatId { constellation: Constellation::Gps, prn: 11 });
        let mut filter =
            PppFilter::new(PppConfig { enable_iono_state: true, ..PppConfig::default() });
        filter.ensure_states(&[&stale, &retained]);
        let retained_ambiguity_index = filter.indices.ambiguity[&retained.signal_id];
        let retained_iono_index = filter.indices.iono[&retained.signal_id.sat];
        filter.ekf.x[retained_ambiguity_index] = 42.0;
        filter.ekf.p[(retained_ambiguity_index, retained_ambiguity_index)] = 123.0;
        filter.ekf.p[(retained_iono_index, retained_ambiguity_index)] = 7.5;
        filter.ekf.p[(retained_ambiguity_index, retained_iono_index)] = 7.5;
        let original_state_count = filter.ekf.x.len();

        let mut removed = BTreeSet::new();
        removed.insert(super::PppStateIdentity::SlantIonosphere(stale.signal_id.sat));
        removed.insert(super::PppStateIdentity::CarrierAmbiguity(stale.signal_id));
        let removed_count = filter.remove_state_identities(&removed);

        assert_eq!(removed_count, 2);
        assert_eq!(filter.ekf.x.len(), original_state_count - 2);
        assert!(!filter.indices.iono.contains_key(&stale.signal_id.sat));
        assert!(!filter.indices.ambiguity.contains_key(&stale.signal_id));
        let compacted_ambiguity_index = filter.indices.ambiguity[&retained.signal_id];
        let compacted_iono_index = filter.indices.iono[&retained.signal_id.sat];
        assert_eq!(filter.ekf.x[compacted_ambiguity_index], 42.0);
        assert_eq!(filter.ekf.p[(compacted_ambiguity_index, compacted_ambiguity_index)], 123.0);
        assert!(
            (filter.ekf.p[(compacted_iono_index, compacted_ambiguity_index)] - 7.5).abs() < 1e-9
        );
        assert_eq!(
            filter.state_identities[compacted_ambiguity_index],
            super::PppStateIdentity::CarrierAmbiguity(retained.signal_id)
        );
        assert_eq!(filter.ekf.labels.len(), filter.ekf.x.len());
    }

    #[test]
    fn ppp_filter_uses_precise_clock_correction_when_available() {
        let eph = make_eph(1);
        let t_s = 1_350.0;
        let state = BroadcastProductsProvider::new(vec![eph.clone()])
            .sat_state(eph.sat, t_s, &mut ProductDiagnostics::default())
            .expect("broadcast state");
        let provider = StubProductsProvider { state, precise_clock_bias_s: Some(2.5e-9) };
        let filter = PppFilter::new(PppConfig::default());

        let (_state, clock_bias_s, fallback) =
            filter.sat_state(&provider, &eph, eph.sat, t_s).expect("PPP precise clock state");

        assert!((clock_bias_s - 2.5e-9).abs() < 1e-18);
        assert!(!fallback);
    }

    #[test]
    fn ppp_filter_reduces_static_residuals_when_ocean_tide_loading_is_configured() {
        let base_receiver_ecef_m = {
            let (x, y, z) = geodetic_to_ecef(37.0, -122.0, 15.0);
            [x, y, z]
        };
        let gps_time = GpsTime { week: 2200, tow_s: 86_400.0 };
        let ocean_tide_loading_model = OceanTideLoadingModel {
            reference_time: gps_time,
            constituents: vec![OceanTideLoadingConstituent::new(
                OceanTideConstituent::M2,
                0.04,
                0.0,
                0.03,
                180.0,
                0.12,
                0.0,
            )],
        };
        let displacement_ecef_m = ocean_tide_loading_model
            .displacement_ecef_m(base_receiver_ecef_m, gps_time)
            .expect("ocean loading displacement");
        let displaced_receiver_ecef_m = [
            base_receiver_ecef_m[0] + displacement_ecef_m[0],
            base_receiver_ecef_m[1] + displacement_ecef_m[1],
            base_receiver_ecef_m[2] + displacement_ecef_m[2],
        ];
        let (lat_deg, lon_deg, alt_m) = ecef_to_geodetic(
            base_receiver_ecef_m[0],
            base_receiver_ecef_m[1],
            base_receiver_ecef_m[2],
        );
        let ztd_m = SaastamoinenModel::zenith_delay_m(Llh { lat_deg, lon_deg, alt_m });

        let satellite_positions_m = vec![
            (
                SatId { constellation: Constellation::Gps, prn: 1 },
                enu_offset_to_ecef(base_receiver_ecef_m, [16_000_000.0, 0.0, 20_500_000.0]),
            ),
            (
                SatId { constellation: Constellation::Gps, prn: 2 },
                enu_offset_to_ecef(
                    base_receiver_ecef_m,
                    [-14_000_000.0, 8_000_000.0, 21_000_000.0],
                ),
            ),
            (
                SatId { constellation: Constellation::Gps, prn: 3 },
                enu_offset_to_ecef(base_receiver_ecef_m, [6_000_000.0, 14_000_000.0, 20_000_000.0]),
            ),
            (
                SatId { constellation: Constellation::Gps, prn: 4 },
                enu_offset_to_ecef(
                    base_receiver_ecef_m,
                    [-8_000_000.0, -15_000_000.0, 20_800_000.0],
                ),
            ),
            (
                SatId { constellation: Constellation::Gps, prn: 5 },
                enu_offset_to_ecef(
                    base_receiver_ecef_m,
                    [12_000_000.0, -9_000_000.0, 19_800_000.0],
                ),
            ),
        ];
        let provider = MappedProductsProvider {
            states: satellite_positions_m
                .iter()
                .map(|(sat, sat_pos_m)| {
                    (
                        *sat,
                        GpsSatState {
                            x_m: sat_pos_m[0],
                            y_m: sat_pos_m[1],
                            z_m: sat_pos_m[2],
                            vx_mps: 0.0,
                            vy_mps: 0.0,
                            vz_mps: 0.0,
                            clock_correction: GpsSatelliteClockCorrection::from_bias_s(0.0),
                            uncertainty: SatelliteStateUncertainty::unavailable(),
                        },
                    )
                })
                .collect(),
        };
        let ephs: Vec<_> = satellite_positions_m.iter().map(|(sat, _)| make_eph(sat.prn)).collect();
        let mut corrected_filter = PppFilter::new(PppConfig {
            use_iono_free: false,
            use_doppler: false,
            enable_iono_state: false,
            ocean_tide_loading_model: Some(ocean_tide_loading_model.clone()),
            ..PppConfig::default()
        });
        corrected_filter.seed_receiver_state(base_receiver_ecef_m, 0.0);

        let mut uncorrected_filter = PppFilter::new(PppConfig {
            use_iono_free: false,
            use_doppler: false,
            enable_iono_state: false,
            ..PppConfig::default()
        });
        uncorrected_filter.seed_receiver_state(base_receiver_ecef_m, 0.0);
        let mut corrected = None;
        let mut uncorrected = None;
        for epoch_idx in 0..6 {
            let obs = make_static_ppp_epoch(
                epoch_idx,
                gps_time,
                displaced_receiver_ecef_m,
                &satellite_positions_m,
                ztd_m,
            );
            corrected = corrected_filter.solve_epoch(&obs, &ephs, &provider);
            uncorrected = uncorrected_filter.solve_epoch(&obs, &ephs, &provider);
        }
        let corrected = corrected.expect("PPP solution with ocean tide loading");
        let uncorrected = uncorrected.expect("PPP solution without ocean tide loading");

        let corrected_pos_m = [corrected.ecef_x_m, corrected.ecef_y_m, corrected.ecef_z_m];
        let uncorrected_pos_m = [uncorrected.ecef_x_m, uncorrected.ecef_y_m, uncorrected.ecef_z_m];
        let corrected_monument_error_m =
            euclidean_distance_m(corrected_pos_m, base_receiver_ecef_m);
        let uncorrected_monument_error_m =
            euclidean_distance_m(uncorrected_pos_m, base_receiver_ecef_m);

        assert!(
            corrected_monument_error_m < uncorrected_monument_error_m,
            "corrected monument error {:.6} vs uncorrected monument error {:.6}",
            corrected_monument_error_m,
            uncorrected_monument_error_m
        );
        assert!(
            corrected.innovation_rms < uncorrected.innovation_rms,
            "corrected innovation rms {:.6} vs uncorrected innovation rms {:.6}",
            corrected.innovation_rms,
            uncorrected.innovation_rms
        );
        assert!(
            corrected.rms_m < uncorrected.rms_m,
            "corrected rms {:.6} vs uncorrected rms {:.6}",
            corrected.rms_m,
            uncorrected.rms_m
        );
        assert!(
            uncorrected_monument_error_m - corrected_monument_error_m > 0.005,
            "corrected monument error {:.6} vs uncorrected monument error {:.6}",
            corrected_monument_error_m,
            uncorrected_monument_error_m
        );
    }

    #[test]
    fn ppp_filter_reduces_static_residuals_when_solid_earth_tide_is_configured() {
        let base_receiver_ecef_m = {
            let (x, y, z) = geodetic_to_ecef(47.0, 8.0, 450.0);
            [x, y, z]
        };
        let start_time = GpsTime { week: 2200, tow_s: 43_200.0 };
        let solid_earth_tide_model = SolidEarthTideModel;
        let displacement_ecef_m = solid_earth_tide_model
            .displacement_ecef_m(base_receiver_ecef_m, start_time)
            .expect("solid Earth tide displacement");
        let displaced_receiver_ecef_m = [
            base_receiver_ecef_m[0] + displacement_ecef_m[0],
            base_receiver_ecef_m[1] + displacement_ecef_m[1],
            base_receiver_ecef_m[2] + displacement_ecef_m[2],
        ];
        let (lat_deg, lon_deg, alt_m) = ecef_to_geodetic(
            base_receiver_ecef_m[0],
            base_receiver_ecef_m[1],
            base_receiver_ecef_m[2],
        );
        let ztd_m = SaastamoinenModel::zenith_delay_m(Llh { lat_deg, lon_deg, alt_m });

        let satellite_positions_m = vec![
            (
                SatId { constellation: Constellation::Gps, prn: 11 },
                enu_offset_to_ecef(base_receiver_ecef_m, [15_000_000.0, 1_000_000.0, 21_500_000.0]),
            ),
            (
                SatId { constellation: Constellation::Gps, prn: 14 },
                enu_offset_to_ecef(
                    base_receiver_ecef_m,
                    [-11_000_000.0, 9_000_000.0, 20_800_000.0],
                ),
            ),
            (
                SatId { constellation: Constellation::Gps, prn: 18 },
                enu_offset_to_ecef(base_receiver_ecef_m, [8_000_000.0, 15_000_000.0, 19_700_000.0]),
            ),
            (
                SatId { constellation: Constellation::Gps, prn: 22 },
                enu_offset_to_ecef(
                    base_receiver_ecef_m,
                    [-9_000_000.0, -14_000_000.0, 20_400_000.0],
                ),
            ),
            (
                SatId { constellation: Constellation::Gps, prn: 25 },
                enu_offset_to_ecef(
                    base_receiver_ecef_m,
                    [13_000_000.0, -7_000_000.0, 20_100_000.0],
                ),
            ),
            (
                SatId { constellation: Constellation::Gps, prn: 31 },
                enu_offset_to_ecef(
                    base_receiver_ecef_m,
                    [-4_000_000.0, 16_000_000.0, 21_000_000.0],
                ),
            ),
        ];
        let provider = MappedProductsProvider {
            states: satellite_positions_m
                .iter()
                .map(|(sat, sat_pos_m)| {
                    (
                        *sat,
                        GpsSatState {
                            x_m: sat_pos_m[0],
                            y_m: sat_pos_m[1],
                            z_m: sat_pos_m[2],
                            vx_mps: 0.0,
                            vy_mps: 0.0,
                            vz_mps: 0.0,
                            clock_correction: GpsSatelliteClockCorrection::from_bias_s(0.0),
                            uncertainty: SatelliteStateUncertainty::unavailable(),
                        },
                    )
                })
                .collect(),
        };
        let ephs: Vec<_> = satellite_positions_m.iter().map(|(sat, _)| make_eph(sat.prn)).collect();
        let mut corrected_filter = PppFilter::new(PppConfig {
            use_iono_free: false,
            use_doppler: false,
            enable_iono_state: false,
            solid_earth_tide_model: Some(solid_earth_tide_model),
            ..PppConfig::default()
        });
        corrected_filter.seed_receiver_state(base_receiver_ecef_m, 0.0);

        let mut uncorrected_filter = PppFilter::new(PppConfig {
            use_iono_free: false,
            use_doppler: false,
            enable_iono_state: false,
            ..PppConfig::default()
        });
        uncorrected_filter.seed_receiver_state(base_receiver_ecef_m, 0.0);

        let mut corrected = None;
        let mut uncorrected = None;
        for epoch_idx in 0..8 {
            let obs = make_static_ppp_epoch(
                epoch_idx,
                start_time,
                displaced_receiver_ecef_m,
                &satellite_positions_m,
                ztd_m,
            );
            corrected = corrected_filter.solve_epoch(&obs, &ephs, &provider);
            uncorrected = uncorrected_filter.solve_epoch(&obs, &ephs, &provider);
        }

        let corrected = corrected.expect("PPP solution with solid Earth tide");
        let uncorrected = uncorrected.expect("PPP solution without solid Earth tide");
        let corrected_pos_m = [corrected.ecef_x_m, corrected.ecef_y_m, corrected.ecef_z_m];
        let uncorrected_pos_m = [uncorrected.ecef_x_m, uncorrected.ecef_y_m, uncorrected.ecef_z_m];
        let corrected_monument_error_m =
            euclidean_distance_m(corrected_pos_m, base_receiver_ecef_m);
        let uncorrected_monument_error_m =
            euclidean_distance_m(uncorrected_pos_m, base_receiver_ecef_m);

        assert!(
            corrected_monument_error_m < uncorrected_monument_error_m,
            "corrected monument error {:.6} vs uncorrected monument error {:.6}",
            corrected_monument_error_m,
            uncorrected_monument_error_m
        );
        assert!(
            corrected.innovation_rms < uncorrected.innovation_rms,
            "corrected innovation rms {:.6} vs uncorrected innovation rms {:.6}",
            corrected.innovation_rms,
            uncorrected.innovation_rms
        );
        assert!(
            corrected.rms_m < uncorrected.rms_m,
            "corrected rms {:.6} vs uncorrected rms {:.6}",
            corrected.rms_m,
            uncorrected.rms_m
        );
        assert!(
            uncorrected_monument_error_m - corrected_monument_error_m > 0.005,
            "corrected monument error {:.6} vs uncorrected monument error {:.6}",
            corrected_monument_error_m,
            uncorrected_monument_error_m
        );
    }

    #[test]
    fn ppp_filter_falls_back_to_current_broadcast_clock_when_precise_clock_is_missing() {
        let eph = make_eph(1);
        let t_s = 1_350.0;
        let state = BroadcastProductsProvider::new(vec![eph.clone()])
            .sat_state(eph.sat, t_s, &mut ProductDiagnostics::default())
            .expect("broadcast state");
        let provider = StubProductsProvider { state, precise_clock_bias_s: None };
        let filter = PppFilter::new(PppConfig::default());

        let (_state, clock_bias_s, fallback) = filter
            .sat_state(&provider, &eph, eph.sat, t_s)
            .expect("PPP broadcast fallback clock state");
        let expected = crate::api::gps_satellite_clock_correction(&eph, t_s);

        assert!((clock_bias_s - expected.bias_s).abs() < 1e-18);
        assert!(fallback);
    }

    #[test]
    fn ppp_solution_epoch_exposes_ecef_position_covariance() {
        let mut filter = PppFilter::new(PppConfig::default());
        filter.seed_receiver_state([6_378_137.0, 10.0, 10.0], 4.0e-6);
        filter.ekf.p[(filter.indices.pos[0], filter.indices.pos[0])] = 4.0;
        filter.ekf.p[(filter.indices.pos[1], filter.indices.pos[1])] = 9.0;
        filter.ekf.p[(filter.indices.pos[2], filter.indices.pos[2])] = 16.0;
        filter.ekf.p[(filter.indices.pos[0], filter.indices.pos[1])] = 0.5;
        filter.ekf.p[(filter.indices.pos[1], filter.indices.pos[0])] = 0.5;

        let solution = filter.solution_epoch(7, 7.0, Vec::new(), 0);
        let covariance = solution
            .position_covariance_ecef_m2
            .expect("PPP solution should emit position covariance");

        assert_eq!(covariance[0][0], 4.0);
        assert_eq!(covariance[1][1], 9.0);
        assert_eq!(covariance[2][2], 16.0);
        assert_eq!(covariance[0][1], 0.5);
        assert_eq!(covariance[1][0], 0.5);
        assert!(solution.sigma_e_m.expect("east sigma").is_finite());
        assert!(solution.sigma_n_m.expect("north sigma").is_finite());
        assert!(solution.sigma_u_m.expect("up sigma").is_finite());
        assert!(solution
            .horizontal_error_ellipse_major_axis_m
            .expect("ellipse major axis")
            .is_finite());
        assert!(solution
            .horizontal_error_ellipse_minor_axis_m
            .expect("ellipse minor axis")
            .is_finite());
        assert!(solution
            .horizontal_error_ellipse_azimuth_deg
            .expect("ellipse azimuth")
            .is_finite());
    }

    #[test]
    fn iono_free_mode_keeps_one_representative_per_satellite() {
        let sat = SatId { constellation: Constellation::Gps, prn: 7 };
        let l1 = ObsSatellite {
            signal_id: SigId { sat, band: SignalBand::L1, code: SignalCode::Ca },
            pseudorange_m: Meters(20_000_000.0),
            pseudorange_var_m2: 1.0,
            carrier_phase_cycles: Cycles(100.0),
            carrier_phase_var_cycles2: 0.01,
            doppler_hz: Hertz(0.0),
            doppler_var_hz2: 1.0,
            cn0_dbhz: 45.0,
            lock_flags: LockFlags {
                code_lock: true,
                carrier_lock: true,
                bit_lock: false,
                cycle_slip: false,
            },
            multipath_suspect: false,
            observation_status: ObservationStatus::Accepted,
            observation_reject_reasons: Vec::new(),
            elevation_deg: None,
            azimuth_deg: None,
            weight: None,
            timing: None,
            error_model: None,
            metadata: ObsMetadata {
                tracking_mode: "test".to_string(),
                integration_ms: 1,
                lock_quality: 45.0,
                smoothing_window: 0,
                smoothing_age: 0,
                smoothing_resets: 0,
                signal: signal_spec_gps_l1_ca(),
                ..ObsMetadata::default()
            },
        };
        let l2 = ObsSatellite {
            signal_id: SigId { sat, band: SignalBand::L2, code: SignalCode::Py },
            metadata: ObsMetadata { signal: signal_spec_gps_l2_py(), ..l1.metadata.clone() },
            ..l1.clone()
        };
        let sats = vec![&l1, &l2];

        let representatives = iono_free_satellite_representatives(&sats);

        assert_eq!(representatives.len(), 1);
        assert_eq!(representatives[0].signal_id.band, SignalBand::L1);
    }

    #[test]
    fn ppp_filter_resolves_iono_free_code_bias_from_both_signals() {
        let sat = SatId { constellation: Constellation::Gps, prn: 7 };
        let l1 = ObsSatellite {
            signal_id: SigId { sat, band: SignalBand::L1, code: SignalCode::Ca },
            pseudorange_m: Meters(20_200_010.0),
            pseudorange_var_m2: 1.0,
            carrier_phase_cycles: Cycles(100.0),
            carrier_phase_var_cycles2: 0.01,
            doppler_hz: Hertz(0.0),
            doppler_var_hz2: 1.0,
            cn0_dbhz: 45.0,
            lock_flags: LockFlags {
                code_lock: true,
                carrier_lock: true,
                bit_lock: false,
                cycle_slip: false,
            },
            multipath_suspect: false,
            observation_status: ObservationStatus::Accepted,
            observation_reject_reasons: Vec::new(),
            elevation_deg: None,
            azimuth_deg: None,
            weight: None,
            timing: None,
            error_model: None,
            metadata: ObsMetadata {
                tracking_mode: "test".to_string(),
                integration_ms: 1,
                lock_quality: 45.0,
                smoothing_window: 0,
                smoothing_age: 0,
                smoothing_resets: 0,
                signal: signal_spec_gps_l1_ca(),
                ..ObsMetadata::default()
            },
        };
        let l2 = ObsSatellite {
            signal_id: SigId { sat, band: SignalBand::L2, code: SignalCode::Py },
            pseudorange_m: Meters(20_200_005.0),
            metadata: ObsMetadata { signal: signal_spec_gps_l2_py(), ..l1.metadata.clone() },
            ..l1.clone()
        };
        let obs = ObsEpoch {
            t_rx_s: Seconds(0.0),
            source_time: ReceiverSampleTrace::from_sample_index(0, 1_000.0),
            gps_week: None,
            tow_s: None,
            epoch_idx: 0,
            discontinuity: false,
            valid: true,
            processing_ms: None,
            role: ReceiverRole::Rover,
            sats: vec![l1, l2],
            decision: ObservationEpochDecision::Accepted,
            decision_reason: Some("accepted_observables_present".to_string()),
            manifest: None,
        };
        let measurement = iono_free_code_observation_from_obs(&obs, sat).expect("iono-free code");
        let bias_table = SignalCodeBiases::from_biases([
            CodeBias { sig: measurement.signal_1, bias_m: 2.0 },
            CodeBias { sig: measurement.signal_2, bias_m: -0.5 },
        ]);

        let resolved_bias_m = resolved_iono_free_code_bias_m(&bias_table, measurement);

        assert!((resolved_bias_m - 2.0).abs() > 1.0e-3);
        assert!(resolved_bias_m.is_finite());
    }

    #[test]
    fn ppp_filter_resolves_single_frequency_code_bias_at_epoch_time() {
        struct TimeAwareBiasProvider;

        impl CodeBiasProvider for TimeAwareBiasProvider {
            fn code_bias_m(&self, _sig: SigId) -> Option<f64> {
                None
            }

            fn code_bias_m_at(
                &self,
                _sig: SigId,
                time: Option<bijux_gnss_core::api::GpsTime>,
            ) -> Option<f64> {
                Some(time?.tow_s)
            }
        }

        let signal = SigId {
            sat: SatId { constellation: Constellation::Gps, prn: 7 },
            band: SignalBand::L1,
            code: SignalCode::Ca,
        };

        let resolved_bias_m = resolved_code_bias_m(
            &TimeAwareBiasProvider,
            signal,
            Some(bijux_gnss_core::api::GpsTime { week: 0, tow_s: 123.5 }),
        );

        assert!((resolved_bias_m - 123.5).abs() < f64::EPSILON);
    }

    #[test]
    fn ppp_filter_tracks_phase_windup_by_satellite() {
        let sat = SatId { constellation: Constellation::Gps, prn: 7 };
        let (receiver_x_m, receiver_y_m, receiver_z_m) = geodetic_to_ecef(45.0, 12.0, 80.0);
        let receiver_pos_m = [receiver_x_m, receiver_y_m, receiver_z_m];
        let sat_pos_m = [15_600_000.0, -10_200_000.0, 21_700_000.0];
        let mut states = BTreeMap::new();

        let first = phase_windup_cycles_for_satellite(
            &mut states,
            sat,
            receiver_pos_m,
            sat_pos_m,
            Some(GpsTime { week: 2200, tow_s: 86_400.0 }),
        );
        let second = phase_windup_cycles_for_satellite(
            &mut states,
            sat,
            receiver_pos_m,
            sat_pos_m,
            Some(GpsTime { week: 2200, tow_s: 86_430.0 }),
        );

        assert!(first.is_finite());
        assert!(second.is_finite());
        assert!(states.contains_key(&sat));
        assert!((second - first).abs() < 0.5);
    }

    #[test]
    fn ppp_filter_skips_phase_windup_without_epoch_time() {
        let sat = SatId { constellation: Constellation::Gps, prn: 7 };
        let mut states = BTreeMap::new();

        let windup_cycles = phase_windup_cycles_for_satellite(
            &mut states,
            sat,
            [1.0, 0.0, 0.0],
            [20_200_000.0, 14_000_000.0, 21_700_000.0],
            None,
        );

        assert_eq!(windup_cycles, 0.0);
        assert!(states.is_empty());
    }

    #[test]
    fn ppp_filter_uses_satellite_antenna_calibration_for_single_frequency_range() {
        let sat = SatId { constellation: Constellation::Gps, prn: 7 };
        let gps_time = Some(bijux_gnss_core::api::GpsTime { week: 2200, tow_s: 86_400.0 });
        let sat_pos_m = [20_200_000.0, 14_000_000.0, 21_700_000.0];
        let receiver_pos_m = [1_111_111.0, -4_222_222.0, 4_333_333.0];
        let calibrations = SatelliteAntennaCalibrations {
            entries: vec![SatelliteAntennaCalibration {
                sat,
                antenna_type: "GPS TEST".to_string(),
                valid_from_unix_s: None,
                valid_until_unix_s: None,
                offsets_by_band: BTreeMap::from([(
                    SignalBand::L1,
                    SatellitePhaseCenterOffset::new(0.12, -0.04, 0.95),
                )]),
                variations_by_band: BTreeMap::new(),
            }],
        };

        let expected = calibrations
            .range_correction_m(sat, SignalBand::L1, gps_time, sat_pos_m, receiver_pos_m)
            .expect("single-frequency antenna correction");
        let actual = single_frequency_antenna_range_correction_m(
            Some(&calibrations),
            None,
            None,
            sat,
            SignalBand::L1,
            gps_time,
            sat_pos_m,
            receiver_pos_m,
            85.0,
            None,
        );

        assert!(actual.abs() > 1.0e-6);
        assert!((actual - expected).abs() < 1.0e-12);
    }

    #[test]
    fn ppp_filter_includes_phase_variation_for_single_frequency_antenna_range() {
        let sat = SatId { constellation: Constellation::Gps, prn: 7 };
        let gps_time = Some(bijux_gnss_core::api::GpsTime { week: 2200, tow_s: 86_400.0 });
        let sat_pos_m = [20_200_000.0, 14_000_000.0, 21_700_000.0];
        let receiver_pos_m = [1_111_111.0, -4_222_222.0, 4_333_333.0];
        let receiver_antenna_type = "AOAD/M_T NONE";
        let satellite_calibrations = SatelliteAntennaCalibrations {
            entries: vec![SatelliteAntennaCalibration {
                sat,
                antenna_type: "GPS TEST".to_string(),
                valid_from_unix_s: None,
                valid_until_unix_s: None,
                offsets_by_band: BTreeMap::from([(
                    SignalBand::L1,
                    SatellitePhaseCenterOffset::new(0.0, 0.0, 0.0),
                )]),
                variations_by_band: BTreeMap::from([(
                    SignalBand::L1,
                    AntennaPhaseCenterVariation::no_azimuth(0.0, 10.0, vec![0.0, 0.10]),
                )]),
            }],
        };
        let receiver_calibrations = ReceiverAntennaCalibrations {
            entries: vec![ReceiverAntennaCalibration {
                antenna_type: receiver_antenna_type.to_string(),
                valid_from_unix_s: None,
                valid_until_unix_s: None,
                offsets_by_band: BTreeMap::from([(
                    SignalBand::L1,
                    ReceiverPhaseCenterOffset::new(0.0, 0.0, 0.0),
                )]),
                variations_by_band: BTreeMap::from([(
                    SignalBand::L1,
                    AntennaPhaseCenterVariation::no_azimuth(0.0, 10.0, vec![0.0, 0.06]),
                )]),
            }],
        };

        let actual = single_frequency_antenna_range_correction_m(
            Some(&satellite_calibrations),
            Some(receiver_antenna_type),
            Some(&receiver_calibrations),
            sat,
            SignalBand::L1,
            gps_time,
            sat_pos_m,
            receiver_pos_m,
            85.0,
            None,
        );

        assert!((actual - 0.08).abs() < 1.0e-12);
    }

    #[test]
    fn ppp_filter_uses_satellite_antenna_calibration_for_iono_free_range() {
        let sat = SatId { constellation: Constellation::Gps, prn: 7 };
        let gps_time = Some(bijux_gnss_core::api::GpsTime { week: 2200, tow_s: 86_400.0 });
        let sat_pos_m = [20_200_000.0, 14_000_000.0, 21_700_000.0];
        let receiver_pos_m = [1_111_111.0, -4_222_222.0, 4_333_333.0];
        let l1 = signal_spec_gps_l1_ca();
        let l2 = signal_spec_gps_l2_py();
        let calibrations = SatelliteAntennaCalibrations {
            entries: vec![SatelliteAntennaCalibration {
                sat,
                antenna_type: "GPS TEST".to_string(),
                valid_from_unix_s: None,
                valid_until_unix_s: None,
                offsets_by_band: BTreeMap::from([
                    (SignalBand::L1, SatellitePhaseCenterOffset::new(0.08, 0.01, 0.91)),
                    (SignalBand::L2, SatellitePhaseCenterOffset::new(0.14, -0.03, 1.12)),
                ]),
                variations_by_band: BTreeMap::new(),
            }],
        };

        let expected = calibrations
            .iono_free_range_correction_m(
                sat,
                SignalBand::L1,
                l1.carrier_hz.value(),
                SignalBand::L2,
                l2.carrier_hz.value(),
                gps_time,
                sat_pos_m,
                receiver_pos_m,
            )
            .expect("iono-free antenna correction");
        let actual = iono_free_antenna_range_correction_m(
            Some(&calibrations),
            None,
            None,
            sat,
            SignalBand::L1,
            l1.carrier_hz.value(),
            SignalBand::L2,
            l2.carrier_hz.value(),
            gps_time,
            sat_pos_m,
            receiver_pos_m,
            85.0,
            None,
        );

        assert!(actual.abs() > 1.0e-6);
        assert!((actual - expected).abs() < 1.0e-12);
    }

    #[test]
    fn ppp_filter_includes_phase_variation_for_iono_free_antenna_range() {
        let sat = SatId { constellation: Constellation::Gps, prn: 7 };
        let gps_time = Some(bijux_gnss_core::api::GpsTime { week: 2200, tow_s: 86_400.0 });
        let sat_pos_m = [20_200_000.0, 14_000_000.0, 21_700_000.0];
        let receiver_pos_m = [1_111_111.0, -4_222_222.0, 4_333_333.0];
        let receiver_antenna_type = "AOAD/M_T NONE";
        let l1 = signal_spec_gps_l1_ca();
        let l2 = signal_spec_gps_l2_py();
        let satellite_calibrations = SatelliteAntennaCalibrations {
            entries: vec![SatelliteAntennaCalibration {
                sat,
                antenna_type: "GPS TEST".to_string(),
                valid_from_unix_s: None,
                valid_until_unix_s: None,
                offsets_by_band: BTreeMap::from([
                    (SignalBand::L1, SatellitePhaseCenterOffset::new(0.0, 0.0, 0.0)),
                    (SignalBand::L2, SatellitePhaseCenterOffset::new(0.0, 0.0, 0.0)),
                ]),
                variations_by_band: BTreeMap::from([
                    (
                        SignalBand::L1,
                        AntennaPhaseCenterVariation::no_azimuth(0.0, 10.0, vec![0.0, 0.10]),
                    ),
                    (
                        SignalBand::L2,
                        AntennaPhaseCenterVariation::no_azimuth(0.0, 10.0, vec![0.0, 0.02]),
                    ),
                ]),
            }],
        };
        let receiver_calibrations = ReceiverAntennaCalibrations {
            entries: vec![ReceiverAntennaCalibration {
                antenna_type: receiver_antenna_type.to_string(),
                valid_from_unix_s: None,
                valid_until_unix_s: None,
                offsets_by_band: BTreeMap::from([
                    (SignalBand::L1, ReceiverPhaseCenterOffset::new(0.0, 0.0, 0.0)),
                    (SignalBand::L2, ReceiverPhaseCenterOffset::new(0.0, 0.0, 0.0)),
                ]),
                variations_by_band: BTreeMap::from([
                    (
                        SignalBand::L1,
                        AntennaPhaseCenterVariation::no_azimuth(0.0, 10.0, vec![0.0, 0.06]),
                    ),
                    (
                        SignalBand::L2,
                        AntennaPhaseCenterVariation::no_azimuth(0.0, 10.0, vec![0.0, 0.01]),
                    ),
                ]),
            }],
        };

        let expected = satellite_calibrations
            .iono_free_range_correction_with_phase_variation_m(
                sat,
                SignalBand::L1,
                l1.carrier_hz.value(),
                SignalBand::L2,
                l2.carrier_hz.value(),
                gps_time,
                sat_pos_m,
                receiver_pos_m,
                85.0,
                None,
            )
            .expect("satellite antenna correction")
            + receiver_calibrations
                .iono_free_range_correction_with_phase_variation_m(
                    receiver_antenna_type,
                    SignalBand::L1,
                    l1.carrier_hz.value(),
                    SignalBand::L2,
                    l2.carrier_hz.value(),
                    gps_time,
                    receiver_pos_m,
                    sat_pos_m,
                    85.0,
                    None,
                )
                .expect("receiver antenna correction");
        let actual = iono_free_antenna_range_correction_m(
            Some(&satellite_calibrations),
            Some(receiver_antenna_type),
            Some(&receiver_calibrations),
            sat,
            SignalBand::L1,
            l1.carrier_hz.value(),
            SignalBand::L2,
            l2.carrier_hz.value(),
            gps_time,
            sat_pos_m,
            receiver_pos_m,
            85.0,
            None,
        );

        assert!(expected.abs() > 1.0e-6);
        assert!((actual - expected).abs() < 1.0e-12);
    }

    #[test]
    fn ppp_filter_uses_receiver_antenna_calibration_for_single_frequency_range() {
        let gps_time = Some(bijux_gnss_core::api::GpsTime { week: 2200, tow_s: 86_400.0 });
        let sat_pos_m = [20_200_000.0, 14_000_000.0, 21_700_000.0];
        let receiver_pos_m = [1_111_111.0, -4_222_222.0, 4_333_333.0];
        let receiver_antenna_type = "AOAD/M_T NONE";
        let calibrations = ReceiverAntennaCalibrations {
            entries: vec![ReceiverAntennaCalibration {
                antenna_type: receiver_antenna_type.to_string(),
                valid_from_unix_s: None,
                valid_until_unix_s: None,
                offsets_by_band: BTreeMap::from([(
                    SignalBand::L1,
                    ReceiverPhaseCenterOffset::new(0.12, -0.04, 0.95),
                )]),
                variations_by_band: BTreeMap::new(),
            }],
        };

        let expected = calibrations
            .range_correction_m(
                receiver_antenna_type,
                SignalBand::L1,
                gps_time,
                receiver_pos_m,
                sat_pos_m,
            )
            .expect("single-frequency receiver antenna correction");
        let actual = single_frequency_antenna_range_correction_m(
            None,
            Some(receiver_antenna_type),
            Some(&calibrations),
            SatId { constellation: Constellation::Gps, prn: 7 },
            SignalBand::L1,
            gps_time,
            sat_pos_m,
            receiver_pos_m,
            85.0,
            None,
        );

        assert!(actual.abs() > 1.0e-6);
        assert!((actual - expected).abs() < 1.0e-12);
    }

    #[test]
    fn ppp_filter_uses_receiver_antenna_calibration_for_iono_free_range() {
        let gps_time = Some(bijux_gnss_core::api::GpsTime { week: 2200, tow_s: 86_400.0 });
        let sat_pos_m = [20_200_000.0, 14_000_000.0, 21_700_000.0];
        let receiver_pos_m = [1_111_111.0, -4_222_222.0, 4_333_333.0];
        let receiver_antenna_type = "AOAD/M_T NONE";
        let l1 = signal_spec_gps_l1_ca();
        let l2 = signal_spec_gps_l2_py();
        let calibrations = ReceiverAntennaCalibrations {
            entries: vec![ReceiverAntennaCalibration {
                antenna_type: receiver_antenna_type.to_string(),
                valid_from_unix_s: None,
                valid_until_unix_s: None,
                offsets_by_band: BTreeMap::from([
                    (SignalBand::L1, ReceiverPhaseCenterOffset::new(0.08, 0.01, 0.91)),
                    (SignalBand::L2, ReceiverPhaseCenterOffset::new(0.14, -0.03, 1.12)),
                ]),
                variations_by_band: BTreeMap::new(),
            }],
        };

        let expected = calibrations
            .iono_free_range_correction_m(
                receiver_antenna_type,
                SignalBand::L1,
                l1.carrier_hz.value(),
                SignalBand::L2,
                l2.carrier_hz.value(),
                gps_time,
                receiver_pos_m,
                sat_pos_m,
            )
            .expect("iono-free receiver antenna correction");
        let actual = iono_free_antenna_range_correction_m(
            None,
            Some(receiver_antenna_type),
            Some(&calibrations),
            SatId { constellation: Constellation::Gps, prn: 7 },
            SignalBand::L1,
            l1.carrier_hz.value(),
            SignalBand::L2,
            l2.carrier_hz.value(),
            gps_time,
            sat_pos_m,
            receiver_pos_m,
            85.0,
            None,
        );

        assert!(actual.abs() > 1.0e-6);
        assert!((actual - expected).abs() < 1.0e-12);
    }

    #[test]
    fn ppp_filter_receiver_antenna_type_selects_matching_calibration() {
        let gps_time = Some(bijux_gnss_core::api::GpsTime { week: 2200, tow_s: 86_400.0 });
        let sat_pos_m = [20_200_000.0, 14_000_000.0, 21_700_000.0];
        let receiver_pos_m = [1_111_111.0, -4_222_222.0, 4_333_333.0];
        let calibrations = ReceiverAntennaCalibrations {
            entries: vec![
                ReceiverAntennaCalibration {
                    antenna_type: "AOAD/M_T NONE".to_string(),
                    valid_from_unix_s: None,
                    valid_until_unix_s: None,
                    offsets_by_band: BTreeMap::from([(
                        SignalBand::L1,
                        ReceiverPhaseCenterOffset::new(0.05, 0.01, 0.70),
                    )]),
                    variations_by_band: BTreeMap::new(),
                },
                ReceiverAntennaCalibration {
                    antenna_type: "TRM57971.00 NONE".to_string(),
                    valid_from_unix_s: None,
                    valid_until_unix_s: None,
                    offsets_by_band: BTreeMap::from([(
                        SignalBand::L1,
                        ReceiverPhaseCenterOffset::new(0.25, 0.08, 1.30),
                    )]),
                    variations_by_band: BTreeMap::new(),
                },
            ],
        };

        let aoad = single_frequency_antenna_range_correction_m(
            None,
            Some("AOAD/M_T NONE"),
            Some(&calibrations),
            SatId { constellation: Constellation::Gps, prn: 7 },
            SignalBand::L1,
            gps_time,
            sat_pos_m,
            receiver_pos_m,
            85.0,
            None,
        );
        let trimble = single_frequency_antenna_range_correction_m(
            None,
            Some("TRM57971.00 NONE"),
            Some(&calibrations),
            SatId { constellation: Constellation::Gps, prn: 7 },
            SignalBand::L1,
            gps_time,
            sat_pos_m,
            receiver_pos_m,
            85.0,
            None,
        );

        assert!((trimble - aoad).abs() > 1.0e-6);
    }
}
