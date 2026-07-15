#![allow(missing_docs)]

use std::collections::{BTreeMap, BTreeSet};

use bijux_gnss_core::api::{Constellation, ObsEpoch, ObsSatellite, SatId, SigId, SignalBand};
use bijux_gnss_signal::api::signal_wavelength_m;

use super::config::{PppConfig, PppFilter, PppIndices, PppStateIdentity, SPEED_OF_LIGHT_MPS};
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
use crate::estimation::position::solver::geodesy::{ecef_to_geodetic, elevation_azimuth_deg};
use crate::estimation::ppp::config::{
    PppConvergenceEvidence, PppConvergenceState, PppHealth, PppLifecycleEvent,
    PppLifecycleEventKind, PppPreciseProductAction, PppProductSupport, PppSolutionEpoch,
    PppStochasticEvidence,
};
use crate::formats::precise_products::{
    PreciseProductDiscontinuity, PreciseProductDiscontinuityKind, PreciseProductSurface,
    ProductDiagnostics, ProductsProvider,
};
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
                lifecycle_events: Vec::new(),
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
            product_support: BTreeMap::new(),
            ar_stable_epochs: 0,
            ar_evidence: Default::default(),
            ar_integer_ambiguities: Vec::new(),
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

        let phase_discontinuities = ppp_phase_discontinuities(obs);
        self.reset_phase_continuity(
            obs.epoch_idx,
            &phase_discontinuities.signals,
            &phase_discontinuities.satellites,
            "carrier_phase_discontinuity",
        );
        let sats = ppp_measurement_observations(obs, self.config.use_iono_free);
        self.ensure_states(&sats, &phase_discontinuities);
        self.update_wide_lane(obs, &sats);
        let fixed_wl = self.try_fix_wide_lane(obs, &sats);
        let corr = compute_corrections(&correction_context);
        let products_time_s = product_reference_time_s(obs);

        let mut residuals = Vec::new();
        let mut used = 0;
        let mut slip_count = 0usize;
        let mut removed_state_identities = BTreeSet::new();
        let mut stochastic_evidence = ppp_stochastic_evidence_from_config(&self.config);
        for sat in &sats {
            if sat.lock_flags.cycle_slip {
                slip_count += 1;
            }
            let eph = match select_best_ephemeris(ephs, sat.signal_id.sat, products_time_s) {
                Some(e) => e,
                None => continue,
            };
            let (state, clock_bias_s, fallback, product_support, product_discontinuities) =
                match self.sat_state(products, eph, sat.signal_id.sat, products_time_s) {
                    Some(v) => v,
                    None => continue,
                };
            if fallback {
                self.health
                    .warnings
                    .push(format!("products fallback used for {:?}", sat.signal_id.sat));
            }
            if !self.apply_precise_product_policy(
                obs.epoch_idx,
                sat.signal_id.sat,
                sat.signal_id,
                &product_discontinuities,
            ) {
                continue;
            }
            if product_discontinuities.iter().any(|discontinuity| {
                self.config.precise_product_policy.action_for(discontinuity.kind)
                    == PppPreciseProductAction::ResetSatelliteState
            }) {
                self.ensure_states(&[*sat], &phase_discontinuities);
            }
            if self.handle_product_support(
                obs.epoch_idx,
                sat.signal_id.sat,
                product_support,
                &phase_discontinuities,
                sat,
            ) {
                self.ensure_states(&[*sat], &phase_discontinuities);
            }
            stochastic_evidence.code_observation_variance_supported |=
                positive_variance(sat.pseudorange_var_m2).is_some();
            stochastic_evidence.phase_observation_variance_supported |=
                positive_variance(sat.carrier_phase_var_cycles2).is_some();
            stochastic_evidence.satellite_orbit_uncertainty_supported |=
                state.uncertainty.orbit_sigma_m.is_some();
            stochastic_evidence.satellite_clock_uncertainty_supported |=
                state.uncertainty.clock_sigma_s.is_some();
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
            let common_range_sigma_m =
                ppp_common_range_sigma_m(&state, troposphere_mapping, &self.config);
            let sigma_m = ppp_code_sigma_m(sat, common_range_sigma_m, &self.config);

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
                        sigma_m: iono_free
                            .code_sigma_m
                            .hypot(common_range_sigma_m)
                            .max(self.config.measurement_noise.code_floor_m),
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
                    if !phase_discontinuities.satellites.contains(&sat.signal_id.sat) {
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
                            sigma_cycles: ppp_iono_free_phase_sigma_cycles(
                                iono_free.phase_sigma_cycles,
                                iono_free.phase_wavelength_m,
                                common_range_sigma_m,
                                &self.config,
                            ),
                            troposphere_mapping,
                            ztd_index: Some(self.indices.ztd),
                            isb_index,
                            ambiguity_index: amb_index,
                            wavelength_m: iono_free.phase_wavelength_m,
                            corr: phase_corr.clone(),
                        };
                        let _ = self.ekf.update(&phase);
                    }
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
                if carrier_phase_usable(sat) {
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
                        sigma_cycles: ppp_phase_sigma_cycles(
                            sat,
                            signal_wavelength_m(sat.metadata.signal).0,
                            common_range_sigma_m,
                            &self.config,
                        ),
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
        self.prune_states(obs.epoch_idx);
        if used < 4 {
            return None;
        }

        Some(self.solution_epoch(
            obs.epoch_idx,
            obs.t_rx_s.0,
            residuals,
            fixed_wl,
            stochastic_evidence,
        ))
    }

    fn solution_epoch(
        &mut self,
        epoch_idx: u64,
        t_rx_s: f64,
        residuals: Vec<(SigId, f64)>,
        fixed_wl: usize,
        stochastic_evidence: PppStochasticEvidence,
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
            lifecycle_events: self.health.lifecycle_events.clone(),
            stochastic_evidence,
            rms_m: self.ekf.health.innovation_rms,
            sigma_h_m: sigma_h,
            sigma_v_m: sigma_v,
            innovation_rms: self.ekf.health.innovation_rms,
            convergence: self.health.convergence.clone(),
            residuals,
            nis_mean: self.health.nis_mean,
            ar_mode: self.config.ar_mode,
            fixed_wl,
            ambiguity_resolution: self.ar_evidence.clone(),
            integer_ambiguities: self.ar_integer_ambiguities.clone(),
        }
    }

    fn sat_state(
        &self,
        products: &dyn ProductsProvider,
        eph: &GpsEphemeris,
        sat: SatId,
        t_s: f64,
    ) -> Option<(GpsSatState, f64, bool, PppProductSupport, Vec<PreciseProductDiscontinuity>)> {
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
        let support = PppProductSupport {
            precise_orbit: diag.sp3_interpolation_summary.is_some(),
            precise_clock: diag.clk_interpolation_summary.is_some(),
        };
        let fallback = !diag.fallbacks.is_empty();
        Some((state, clock_correction.bias_s, fallback, support, diag.discontinuities))
    }

    fn ensure_states(&mut self, sats: &[&ObsSatellite], phase_discontinuities: &PppPhaseBreaks) {
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
            if carrier_phase_usable(sat)
                && !phase_discontinuities.satellites.contains(&sat.signal_id.sat)
                && !self.indices.ambiguity.contains_key(&sat.signal_id)
            {
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

    fn reset_phase_continuity(
        &mut self,
        epoch_idx: u64,
        discontinuity_signals: &BTreeSet<SigId>,
        discontinuity_satellites: &BTreeSet<SatId>,
        reason: &str,
    ) -> usize {
        if discontinuity_signals.is_empty() && discontinuity_satellites.is_empty() {
            return 0;
        }
        let affected_signals = self
            .indices
            .ambiguity
            .keys()
            .copied()
            .filter(|signal| {
                discontinuity_signals.contains(signal)
                    || discontinuity_satellites.contains(&signal.sat)
            })
            .collect::<Vec<_>>();
        let mut removed_state_identities = BTreeSet::new();
        for signal in &affected_signals {
            self.indices.ambiguity.remove(signal);
            self.last_seen_amb.remove(signal);
            removed_state_identities.insert(PppStateIdentity::CarrierAmbiguity(*signal));
        }
        for sat in discontinuity_satellites {
            self.phase_windup.remove(sat);
            self.wl_state.remove(sat);
        }
        let removed_states = removed_state_identities.iter().copied().collect::<Vec<_>>();
        let removed_count = self.remove_state_identities(&removed_state_identities);
        for signal in discontinuity_signals {
            self.health.lifecycle_events.push(PppLifecycleEvent {
                kind: PppLifecycleEventKind::CarrierDiscontinuity,
                epoch_idx: Some(epoch_idx),
                sat: Some(signal.sat),
                signal: Some(*signal),
                removed_states: removed_states.clone(),
                reason: reason.to_string(),
            });
        }
        removed_count
    }

    fn handle_product_support(
        &mut self,
        epoch_idx: u64,
        sat: SatId,
        support: PppProductSupport,
        phase_discontinuities: &PppPhaseBreaks,
        representative: &ObsSatellite,
    ) -> bool {
        let previous = self.product_support.insert(sat, support);
        if previous.is_none() || previous == Some(support) {
            return false;
        }
        let removed_states = self.reset_satellite_dependent_states(sat);
        let removed_count = removed_states.len();
        let previous = previous.expect("previous product support");
        if removed_count > 0 || previous != support {
            self.health.lifecycle_events.push(PppLifecycleEvent {
                kind: PppLifecycleEventKind::ProductSupportChanged,
                epoch_idx: Some(epoch_idx),
                sat: Some(sat),
                signal: Some(representative.signal_id),
                removed_states,
                reason: format!(
                    "product_support_changed_from_orbit_{}_clock_{}_to_orbit_{}_clock_{}",
                    previous.precise_orbit,
                    previous.precise_clock,
                    support.precise_orbit,
                    support.precise_clock
                ),
            });
        }
        !phase_discontinuities.satellites.contains(&sat)
    }

    fn apply_precise_product_policy(
        &mut self,
        epoch_idx: u64,
        sat: SatId,
        signal: SigId,
        discontinuities: &[PreciseProductDiscontinuity],
    ) -> bool {
        let mut usable = true;
        for discontinuity in discontinuities {
            let action = self.config.precise_product_policy.action_for(discontinuity.kind);
            match action {
                PppPreciseProductAction::BridgeWithBroadcast => {
                    self.record_precise_product_policy_event(
                        epoch_idx,
                        sat,
                        signal,
                        action,
                        *discontinuity,
                        Vec::new(),
                    );
                }
                PppPreciseProductAction::InflateSatelliteState => {
                    self.inflate_satellite_dependent_states(sat);
                    self.record_precise_product_policy_event(
                        epoch_idx,
                        sat,
                        signal,
                        action,
                        *discontinuity,
                        Vec::new(),
                    );
                }
                PppPreciseProductAction::ResetSatelliteState => {
                    let removed_states = self.reset_satellite_dependent_states(sat);
                    self.record_precise_product_policy_event(
                        epoch_idx,
                        sat,
                        signal,
                        action,
                        *discontinuity,
                        removed_states,
                    );
                }
                PppPreciseProductAction::RefuseSatellite => {
                    self.record_precise_product_policy_event(
                        epoch_idx,
                        sat,
                        signal,
                        action,
                        *discontinuity,
                        Vec::new(),
                    );
                    usable = false;
                }
            }
        }
        usable
    }

    fn reset_satellite_dependent_states(&mut self, sat: SatId) -> Vec<PppStateIdentity> {
        let mut removed_state_identities = BTreeSet::new();
        if self.indices.iono.remove(&sat).is_some() {
            self.last_seen_iono.remove(&sat);
            removed_state_identities.insert(PppStateIdentity::SlantIonosphere(sat));
        }
        let affected_signals = self
            .indices
            .ambiguity
            .keys()
            .copied()
            .filter(|signal| signal.sat == sat)
            .collect::<Vec<_>>();
        for signal in &affected_signals {
            self.indices.ambiguity.remove(signal);
            self.last_seen_amb.remove(signal);
            self.residual_history.remove(signal);
            removed_state_identities.insert(PppStateIdentity::CarrierAmbiguity(*signal));
        }
        self.phase_windup.remove(&sat);
        self.wl_state.remove(&sat);
        let removed_states = removed_state_identities.iter().copied().collect::<Vec<_>>();
        self.remove_state_identities(&removed_state_identities);
        removed_states
    }

    fn inflate_satellite_dependent_states(&mut self, sat: SatId) {
        let inflation = self.config.precise_product_policy.satellite_state_inflation;
        if !inflation.is_finite() || inflation <= 1.0 {
            return;
        }
        if let Some(index) = self.indices.iono.get(&sat).copied() {
            if index < self.ekf.p.rows() {
                self.ekf.p[(index, index)] *= inflation;
            }
        }
        for index in self
            .indices
            .ambiguity
            .iter()
            .filter_map(|(signal, index)| (signal.sat == sat).then_some(*index))
        {
            if index < self.ekf.p.rows() {
                self.ekf.p[(index, index)] *= inflation;
            }
        }
        self.ekf.sanitize_covariance();
    }

    fn record_precise_product_policy_event(
        &mut self,
        epoch_idx: u64,
        sat: SatId,
        signal: SigId,
        action: PppPreciseProductAction,
        discontinuity: PreciseProductDiscontinuity,
        removed_states: Vec<PppStateIdentity>,
    ) {
        self.health.lifecycle_events.push(PppLifecycleEvent {
            kind: PppLifecycleEventKind::PreciseProductDiscontinuity,
            epoch_idx: Some(epoch_idx),
            sat: Some(sat),
            signal: Some(signal),
            removed_states,
            reason: format!(
                "precise_product_{}_{}_action_{}",
                precise_product_surface_label(discontinuity.surface),
                precise_product_discontinuity_label(discontinuity.kind),
                precise_product_action_label(action)
            ),
        });
    }

    fn predict(&mut self, dt_s: f64) {
        let model = PppProcessModel {
            pos: self.indices.pos,
            vel: self.indices.vel,
            clock_bias: self.indices.clock_bias,
            clock_drift: self.indices.clock_drift,
            ztd: self.indices.ztd,
            inter_system_biases: self.indices.isb.values().copied().collect(),
            ionospheres: self.indices.iono.values().copied().collect(),
            ambiguities: self.indices.ambiguity.values().copied().collect(),
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
        let mut removed_satellites = BTreeSet::new();
        let mut removed_signals = BTreeSet::new();
        let cutoff = self.config.prune_after_epochs;
        self.last_seen_iono.retain(|sat, last| {
            let keep = epoch_idx.saturating_sub(*last) <= cutoff;
            if !keep {
                self.indices.iono.remove(sat);
                removed_satellites.insert(*sat);
                removed_state_identities.insert(PppStateIdentity::SlantIonosphere(*sat));
            }
            keep
        });
        self.last_seen_amb.retain(|sig, last| {
            let keep = epoch_idx.saturating_sub(*last) <= cutoff;
            if !keep {
                self.indices.ambiguity.remove(sig);
                removed_signals.insert(*sig);
                removed_satellites.insert(sig.sat);
                removed_state_identities.insert(PppStateIdentity::CarrierAmbiguity(*sig));
            }
            keep
        });
        for signal in &removed_signals {
            self.residual_history.remove(signal);
        }
        for sat in &removed_satellites {
            let has_remaining_ambiguity = self.indices.ambiguity.keys().any(|sig| sig.sat == *sat);
            let has_remaining_iono = self.indices.iono.contains_key(sat);
            if !has_remaining_ambiguity && !has_remaining_iono {
                self.phase_windup.remove(sat);
                self.wl_state.remove(sat);
            }
        }
        let pruned = self.remove_state_identities(&removed_state_identities);
        self.health.pruned_states += pruned;
        if pruned > 0 {
            for sat in removed_satellites {
                let removed_states = removed_state_identities
                    .iter()
                    .copied()
                    .filter(|identity| match identity {
                        PppStateIdentity::SlantIonosphere(identity_sat) => *identity_sat == sat,
                        PppStateIdentity::CarrierAmbiguity(identity_sig) => identity_sig.sat == sat,
                        _ => false,
                    })
                    .collect::<Vec<_>>();
                self.health.lifecycle_events.push(PppLifecycleEvent {
                    kind: PppLifecycleEventKind::SatelliteStatePruned,
                    epoch_idx: Some(epoch_idx),
                    sat: Some(sat),
                    signal: None,
                    removed_states,
                    reason: "satellite_not_seen_within_prune_window".to_string(),
                });
            }
        }
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

fn precise_product_surface_label(surface: PreciseProductSurface) -> &'static str {
    match surface {
        PreciseProductSurface::Orbit => "orbit",
        PreciseProductSurface::Clock => "clock",
    }
}

fn precise_product_discontinuity_label(kind: PreciseProductDiscontinuityKind) -> &'static str {
    match kind {
        PreciseProductDiscontinuityKind::MissingSatellite => "missing_satellite",
        PreciseProductDiscontinuityKind::OutOfCoverage => "out_of_coverage",
        PreciseProductDiscontinuityKind::InsufficientSupport => "insufficient_support",
        PreciseProductDiscontinuityKind::OrbitGap => "orbit_gap",
        PreciseProductDiscontinuityKind::OrbitFlag => "orbit_flag",
        PreciseProductDiscontinuityKind::ClockGap => "clock_gap",
        PreciseProductDiscontinuityKind::ClockJump => "clock_jump",
    }
}

fn precise_product_action_label(action: PppPreciseProductAction) -> &'static str {
    match action {
        PppPreciseProductAction::BridgeWithBroadcast => "bridge_with_broadcast",
        PppPreciseProductAction::InflateSatelliteState => "inflate_satellite_state",
        PppPreciseProductAction::ResetSatelliteState => "reset_satellite_state",
        PppPreciseProductAction::RefuseSatellite => "refuse_satellite",
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

fn ppp_measurement_observations(obs: &ObsEpoch, use_iono_free: bool) -> Vec<&ObsSatellite> {
    let mut sats: Vec<&ObsSatellite> = obs.sats.iter().collect();
    sats.sort_by_key(|s| s.signal_id);
    if use_iono_free {
        iono_free_satellite_representatives(&sats)
    } else {
        sats
    }
}

#[derive(Debug, Clone, Default)]
struct PppPhaseBreaks {
    signals: BTreeSet<SigId>,
    satellites: BTreeSet<SatId>,
}

fn carrier_phase_usable(sat: &ObsSatellite) -> bool {
    sat.lock_flags.carrier_lock && !sat.lock_flags.cycle_slip
}

fn ppp_phase_discontinuities(obs: &ObsEpoch) -> PppPhaseBreaks {
    let mut breaks = PppPhaseBreaks::default();
    for sat in &obs.sats {
        if !carrier_phase_usable(sat) {
            breaks.signals.insert(sat.signal_id);
            breaks.satellites.insert(sat.signal_id.sat);
        }
    }
    breaks
}

fn product_reference_time_s(obs: &ObsEpoch) -> f64 {
    obs.gps_time().map(|time| time.tow_s).unwrap_or(obs.t_rx_s.0)
}

fn ppp_stochastic_evidence_from_config(config: &PppConfig) -> PppStochasticEvidence {
    PppStochasticEvidence {
        atmosphere_residual_supported: config.measurement_noise.troposphere_residual_m.is_finite()
            && config.measurement_noise.troposphere_residual_m >= 0.0,
        antenna_residual_supported: config.measurement_noise.antenna_residual_m.is_finite()
            && config.measurement_noise.antenna_residual_m >= 0.0,
        process_covariance_supported: ppp_process_noise_supported(config),
        ..PppStochasticEvidence::default()
    }
}

fn ppp_process_noise_supported(config: &PppConfig) -> bool {
    let process = &config.process_noise;
    [
        process.position_m,
        process.velocity_mps,
        process.clock_bias_s,
        process.clock_drift_s,
        process.inter_system_bias_s,
        process.ztd_m,
        process.iono_m,
        process.ambiguity_cycles,
    ]
    .iter()
    .all(|value| value.is_finite() && *value >= 0.0)
}

fn ppp_common_range_sigma_m(
    state: &GpsSatState,
    troposphere_mapping: f64,
    config: &PppConfig,
) -> f64 {
    let noise = &config.measurement_noise;
    let orbit_sigma_m = state
        .uncertainty
        .orbit_sigma_m
        .filter(|value| value.is_finite() && *value >= 0.0)
        .unwrap_or(0.0)
        * noise.orbit_sigma_scale.max(0.0);
    let clock_sigma_m = state
        .uncertainty
        .clock_sigma_s
        .filter(|value| value.is_finite() && *value >= 0.0)
        .unwrap_or(0.0)
        * SPEED_OF_LIGHT_MPS
        * noise.clock_sigma_scale.max(0.0);
    let atmosphere_sigma_m =
        troposphere_mapping.abs() * positive_or_zero(noise.troposphere_residual_m);
    let antenna_sigma_m = positive_or_zero(noise.antenna_residual_m);

    (orbit_sigma_m.powi(2)
        + clock_sigma_m.powi(2)
        + atmosphere_sigma_m.powi(2)
        + antenna_sigma_m.powi(2))
    .sqrt()
}

fn ppp_code_sigma_m(sat: &ObsSatellite, common_range_sigma_m: f64, config: &PppConfig) -> f64 {
    let code_sigma_m = positive_variance(sat.pseudorange_var_m2)
        .map(f64::sqrt)
        .unwrap_or_else(|| positive_or_zero(config.measurement_noise.code_floor_m));
    code_sigma_m
        .hypot(common_range_sigma_m)
        .max(positive_or_zero(config.measurement_noise.code_floor_m))
}

fn ppp_phase_sigma_cycles(
    sat: &ObsSatellite,
    wavelength_m: f64,
    common_range_sigma_m: f64,
    config: &PppConfig,
) -> f64 {
    let carrier_sigma_cycles = positive_variance(sat.carrier_phase_var_cycles2)
        .map(f64::sqrt)
        .unwrap_or_else(|| positive_or_zero(config.measurement_noise.phase_floor_cycles));
    ppp_iono_free_phase_sigma_cycles(
        carrier_sigma_cycles,
        wavelength_m,
        common_range_sigma_m,
        config,
    )
}

fn ppp_iono_free_phase_sigma_cycles(
    phase_sigma_cycles: f64,
    wavelength_m: f64,
    common_range_sigma_m: f64,
    config: &PppConfig,
) -> f64 {
    let range_sigma_cycles = if wavelength_m.is_finite() && wavelength_m > 0.0 {
        common_range_sigma_m / wavelength_m
    } else {
        0.0
    };
    positive_or_zero(phase_sigma_cycles)
        .hypot(range_sigma_cycles)
        .max(positive_or_zero(config.measurement_noise.phase_floor_cycles))
}

fn positive_variance(value: f64) -> Option<f64> {
    (value.is_finite() && value >= 0.0).then_some(value)
}

fn positive_or_zero(value: f64) -> f64 {
    if value.is_finite() && value >= 0.0 {
        value
    } else {
        0.0
    }
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
#[path = "filter_tests.rs"]
mod tests;
