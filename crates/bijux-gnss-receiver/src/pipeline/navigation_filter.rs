#![allow(missing_docs)]

use bijux_gnss_core::api::{
    elevation_azimuth_deg, obs_epoch_stability_key, signal_wavelength_m, Constellation, Epoch, Llh,
    Meters, NavHealthEvent, NavLifecycleState, NavRefusalClass, NavSolutionEpoch,
    NavUncertaintyClass, ObsEpoch, ObsSatellite, Seconds, SolutionStatus, SolutionValidity,
    NAV_OUTPUT_STABILITY_SIGNATURE_VERSION, NAV_SOLUTION_MODEL_VERSION,
};
use bijux_gnss_nav::api::{
    clamp_ztd, compute_corrections, ecef_to_geodetic, is_ephemeris_valid,
    position_dops_from_satellite_positions, position_measurement_weight,
    sat_state_gps_l1ca_at_receive_time, AmbiguityManager, AtmosphereConfig, CarrierPhaseMeasurement,
    CodeBiasProvider, CorrectionContext, DopplerMeasurement, Ekf, EkfConfig, GpsEphemeris,
    GpsSatState, InnovationConsistencyConfig, InterSystemBiasManager, KlobucharCoefficients,
    Matrix, NavClockModel, PhaseBiasProvider, PositionDops, PositionObservation, PositionSolver,
    PositionWeightingModel, ProcessNoiseConfig, PseudorangeMeasurement, WeightingConfig, ZeroBiases,
};

use crate::engine::receiver_config::ReceiverPipelineConfig;

#[derive(Debug, Clone, Copy)]
pub struct NavigationFilterThresholds {
    pub max_pdop: f64,
    pub max_gdop: f64,
    pub min_used_satellites: usize,
}

impl Default for NavigationFilterThresholds {
    fn default() -> Self {
        Self { max_pdop: 8.0, max_gdop: 12.0, min_used_satellites: 4 }
    }
}

pub struct NavigationFilter {
    ekf: Ekf,
    model: NavClockModel,
    last_t_rx_s: Option<f64>,
    ambiguity: AmbiguityManager,
    isb: InterSystemBiasManager,
    thresholds: NavigationFilterThresholds,
    tropo_enabled: bool,
    ztd_index: Option<usize>,
    atmosphere: AtmosphereConfig,
    code_bias: ZeroBiases,
    phase_bias: ZeroBiases,
    corrections: CorrectionContext,
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
struct NavigationSolutionEvidence {
    covariance_supported: bool,
    residual_supported: bool,
    ambiguity_supported: bool,
    correction_supported: bool,
    integrity_supported: bool,
}

impl NavigationSolutionEvidence {
    fn missing_reasons(self) -> Vec<&'static str> {
        let mut reasons = Vec::new();
        if !self.covariance_supported {
            reasons.push("missing_covariance_evidence");
        }
        if !self.residual_supported {
            reasons.push("missing_residual_evidence");
        }
        if !self.ambiguity_supported {
            reasons.push("missing_ambiguity_evidence");
        }
        if !self.correction_supported {
            reasons.push("missing_correction_evidence");
        }
        if !self.integrity_supported {
            reasons.push("missing_integrity_evidence");
        }
        reasons
    }

    fn supports_float_claim(self) -> bool {
        self.covariance_supported
            && self.residual_supported
            && self.ambiguity_supported
            && self.correction_supported
            && self.integrity_supported
    }
}

impl NavigationFilter {
    #[cfg(test)]
    pub fn new() -> Self {
        Self::new_with_troposphere(true, 2.3, NavigationFilterThresholds::default())
    }

    pub fn new_with_troposphere(
        tropo_enabled: bool,
        tropo_ztd_m: f64,
        thresholds: NavigationFilterThresholds,
    ) -> Self {
        let ekf = Ekf::new(
            vec![0.0_f64; 8],
            Matrix::identity(8),
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
        Self {
            ekf,
            model: NavClockModel::new(ProcessNoiseConfig {
                pos_m: 1.0,
                vel_mps: 1.0,
                clock_bias_s: 1e-4,
                clock_drift_s: 1e-5,
                ztd_m: 0.0,
            }),
            last_t_rx_s: None,
            ambiguity: AmbiguityManager::new(),
            isb: InterSystemBiasManager::new(),
            thresholds,
            tropo_enabled,
            ztd_index: None,
            atmosphere: AtmosphereConfig {
                enable_ztd: tropo_enabled,
                ztd_initial_m: tropo_ztd_m,
                ..AtmosphereConfig::default()
            },
            code_bias: ZeroBiases,
            phase_bias: ZeroBiases,
            corrections: CorrectionContext::default(),
        }
    }

    pub fn from_pipeline_config(config: &ReceiverPipelineConfig) -> Self {
        Self::new_with_troposphere(
            config.tropo_enable,
            config.tropo_ztd_m,
            NavigationFilterThresholds {
                max_pdop: config.science_thresholds.max_pdop,
                max_gdop: config.science_thresholds.max_gdop,
                min_used_satellites: config.science_thresholds.min_used_satellites,
            },
        )
    }

    pub fn solve_epoch(
        &mut self,
        obs: &ObsEpoch,
        ephemerides: &[GpsEphemeris],
        klobuchar: Option<&KlobucharCoefficients>,
    ) -> Option<NavSolutionEpoch> {
        if (klobuchar.is_some() || self.tropo_enabled) && position_is_uninitialized(self) {
            prime_state_from_wls(self, obs, ephemerides, klobuchar, self.tropo_enabled);
        }
        let dt_s =
            if let Some(prev) = self.last_t_rx_s { (obs.t_rx_s.0 - prev).max(1e-3) } else { 0.001 };
        self.last_t_rx_s = Some(obs.t_rx_s.0);
        self.ekf.predict(&self.model, dt_s);

        let mut used = 0usize;
        let mut used_satellite_positions = Vec::new();
        let mut stale_ephemeris_rejections = 0usize;
        let mut sats: Vec<&ObsSatellite> = obs.sats.iter().collect();
        sats.sort_by_key(|sat| sat.signal_id);
        let sat_count = sats.len();
        let receive_tow_s = obs.gps_time().map(|gps_time| gps_time.tow_s).unwrap_or(obs.t_rx_s.0);

        for sat in sats {
            let eph = match ephemerides.iter().find(|eph| eph.sat == sat.signal_id.sat) {
                Some(ephemeris) => ephemeris,
                None => continue,
            };
            if !is_ephemeris_valid(eph, receive_tow_s) {
                stale_ephemeris_rejections += 1;
                continue;
            }

            let _ = compute_corrections(&self.corrections);
            let state = navigation_satellite_state(sat, eph, receive_tow_s);
            let state_next = navigation_satellite_state(sat, eph, receive_tow_s + 0.1);
            let sat_vel = [
                (state_next.x_m - state.x_m) / 0.1,
                (state_next.y_m - state.y_m) / 0.1,
                (state_next.z_m - state.z_m) / 0.1,
            ];
            let rx_x = self.ekf.x[0];
            let rx_y = self.ekf.x[1];
            let rx_z = self.ekf.x[2];
            let (_azimuth_deg, elevation_deg) =
                elevation_azimuth_deg(rx_x, rx_y, rx_z, state.x_m, state.y_m, state.z_m);
            let iono_m =
                estimate_klobuchar_delay_m(klobuchar, [rx_x, rx_y, rx_z], receive_tow_s, &state);
            let tropo_m =
                estimate_saastamoinen_delay_m(self.tropo_enabled, [rx_x, rx_y, rx_z], &state);
            let weight = position_measurement_weight(
                Some(sat.cn0_dbhz),
                Some(elevation_deg),
                None,
                WeightingConfig {
                    model: PositionWeightingModel::Cn0,
                    ..WeightingConfig::default()
                },
            );
            let sigma_m = (5.0 / weight.max(0.1)).max(1.0);
            let isb_index = if sat.signal_id.sat.constellation != Constellation::Gps {
                let key = format!("isb_{:?}", sat.signal_id.sat.constellation);
                Some(self.isb.get_or_add(&mut self.ekf, &key, 0.0, 1e-6))
            } else {
                None
            };
            let code_bias_m = self.code_bias.code_bias_m(sat.signal_id).unwrap_or(0.0);
            let pseudorange_meas = PseudorangeMeasurement {
                sig: sat.signal_id,
                z_m: sat.pseudorange_m.0 - code_bias_m,
                sat_pos_m: [state.x_m, state.y_m, state.z_m],
                sat_clock_s: state.clock_correction.bias_s,
                tropo_m,
                iono_m,
                sigma_m,
                elevation_deg: Some(elevation_deg),
                ztd_index: self.ztd_index,
                isb_index,
            };
            if self.ekf.update(&pseudorange_meas) {
                used += 1;
                used_satellite_positions.push([state.x_m, state.y_m, state.z_m]);
            }

            let doppler_meas = DopplerMeasurement {
                sig: sat.signal_id,
                z_hz: sat.doppler_hz.0,
                sat_pos_m: [state.x_m, state.y_m, state.z_m],
                sat_vel_mps: sat_vel,
                wavelength_m: signal_wavelength_m(sat.metadata.signal).0,
                sigma_hz: 2.0,
            };
            self.ekf.update(&doppler_meas);

            let ambiguity_key = format!(
                "{:?}:{}:{:?}",
                sat.metadata.signal.constellation, sat.signal_id.sat.prn, sat.metadata.signal.band
            );
            let ambiguity_index =
                self.ambiguity.get_or_add(&mut self.ekf, &ambiguity_key, 0.0, 100.0);
            let phase_bias_cycles = self.phase_bias.phase_bias_cycles(sat.signal_id).unwrap_or(0.0);
            let carrier_meas = CarrierPhaseMeasurement {
                sig: sat.signal_id,
                z_cycles: sat.carrier_phase_cycles.0 - phase_bias_cycles,
                sat_pos_m: [state.x_m, state.y_m, state.z_m],
                sat_clock_s: state.clock_correction.bias_s,
                tropo_m,
                iono_m,
                wavelength_m: signal_wavelength_m(sat.metadata.signal).0,
                ambiguity_index: Some(ambiguity_index),
                sigma_cycles: 0.05,
                elevation_deg: Some(elevation_deg),
                ztd_index: self.ztd_index,
                isb_index,
            };
            self.ekf.update(&carrier_meas);
        }

        if let Some(index) = self.ztd_index {
            if index < self.ekf.x.len() {
                let before_m = self.ekf.x[index];
                let after_m = clamp_ztd(before_m, &self.atmosphere);
                if (after_m - before_m).abs() > 1e-6 {
                    self.ekf.x[index] = after_m;
                    self.ekf.health.events.push(NavHealthEvent::ZtdClamped { before_m, after_m });
                }
            }
        }

        let mut explain_reasons = vec![format!("usable_satellites={used}")];
        if stale_ephemeris_rejections > 0 {
            explain_reasons.push(format!("stale_ephemeris_rejections={stale_ephemeris_rejections}"));
        }
        explain_reasons.push(if klobuchar.is_some() {
            "ionosphere_correction=klobuchar_broadcast".to_string()
        } else {
            "ionosphere_uncorrected".to_string()
        });
        explain_reasons.push(if self.tropo_enabled {
            "troposphere_correction=saastamoinen".to_string()
        } else {
            "troposphere_uncorrected".to_string()
        });

        let geometry_dops = position_dops_from_satellite_positions(
            [self.ekf.x[0], self.ekf.x[1], self.ekf.x[2]],
            &used_satellite_positions,
        );
        let geometry_violations =
            navigation_geometry_threshold_violations(self.thresholds, used, geometry_dops.as_ref());
        if !geometry_violations.is_empty() {
            explain_reasons.extend(geometry_violations);
            let refusal_class = if stale_ephemeris_rejections > 0 {
                if used == 0 {
                    Some(NavRefusalClass::InvalidEphemeris)
                } else {
                    Some(NavRefusalClass::InsufficientGeometry)
                }
            } else {
                Some(NavRefusalClass::InsufficientGeometry)
            };
            let mut solution = navigation_refusal_epoch(
                obs,
                sat_count,
                used,
                sat_count.saturating_sub(used),
                refusal_class,
                explain_reasons,
            );
            populate_solution_dops(&mut solution, geometry_dops.as_ref());
            solution.innovation_rms_m = Some(self.ekf.health.innovation_rms);
            solution.ekf_innovation_rms = Some(self.ekf.health.innovation_rms);
            solution.ekf_condition_number = self.ekf.health.condition_number;
            solution.ekf_whiteness_ratio = self.ekf.health.whiteness_ratio;
            solution.ekf_predicted_variance = self.ekf.health.predicted_variance;
            solution.ekf_observed_variance = self.ekf.health.observed_variance;
            populate_solution_trace_identity(obs, &mut solution);
            return Some(solution);
        }

        let evidence = navigation_filter_solution_evidence(
            self,
            used,
            self.ekf.health.innovation_rms,
            klobuchar.is_some(),
        );
        let (status, status_reasons) = navigation_filter_status_floor(evidence);
        explain_reasons.extend(status_reasons);
        let (latitude_deg, longitude_deg, altitude_m) =
            ecef_to_geodetic(self.ekf.x[0], self.ekf.x[1], self.ekf.x[2]);
        let mut solution = NavSolutionEpoch {
            epoch: Epoch { index: obs.epoch_idx },
            t_rx_s: obs.t_rx_s,
            source_time: obs.source_time,
            ecef_x_m: Meters(self.ekf.x[0]),
            ecef_y_m: Meters(self.ekf.x[1]),
            ecef_z_m: Meters(self.ekf.x[2]),
            position_covariance_ecef_m2: None,
            latitude_deg,
            longitude_deg,
            altitude_m: Meters(altitude_m),
            clock_bias_s: Seconds(self.ekf.x[6]),
            clock_bias_m: Meters(self.ekf.x[6] * 299_792_458.0),
            clock_drift_s_per_s: self.ekf.x.get(7).copied().unwrap_or(0.0),
            pdop: 0.0,
            pre_fit_residual_rms_m: Some(Meters(self.ekf.health.innovation_rms)),
            post_fit_residual_rms_m: Some(Meters(self.ekf.health.innovation_rms)),
            rms_m: Meters(self.ekf.health.innovation_rms),
            status,
            quality: status.quality_flag(),
            validity: solution_validity(status),
            valid: bijux_gnss_core::api::is_solution_valid(status),
            processing_ms: None,
            residuals: Vec::new(),
            constellation_residual_rms: Vec::new(),
            health: Vec::new(),
            isb: Vec::new(),
            sigma_e_m: None,
            sigma_n_m: None,
            sigma_u_m: None,
            horizontal_error_ellipse_major_axis_m: None,
            horizontal_error_ellipse_minor_axis_m: None,
            horizontal_error_ellipse_azimuth_deg: None,
            sigma_h_m: None,
            sigma_v_m: None,
            innovation_rms_m: Some(self.ekf.health.innovation_rms),
            normalized_innovation_rms: None,
            normalized_innovation_max: None,
            ekf_innovation_rms: Some(self.ekf.health.innovation_rms),
            ekf_condition_number: self.ekf.health.condition_number,
            ekf_whiteness_ratio: self.ekf.health.whiteness_ratio,
            ekf_predicted_variance: self.ekf.health.predicted_variance,
            ekf_observed_variance: self.ekf.health.observed_variance,
            integrity_hpl_m: None,
            integrity_vpl_m: None,
            model_version: NAV_SOLUTION_MODEL_VERSION,
            lifecycle_state: solution_lifecycle_state(status),
            uncertainty_class: solution_uncertainty_class(status),
            assumptions: None,
            refusal_class: None,
            artifact_id: String::new(),
            source_observation_epoch_id: String::new(),
            explain_decision: "receiver_navigation_filter".to_string(),
            explain_reasons,
            provenance: None,
            sat_count,
            used_sat_count: used,
            rejected_sat_count: sat_count.saturating_sub(used),
            hdop: None,
            vdop: None,
            gdop: None,
            tdop: None,
            stability_signature: String::new(),
            stability_signature_version: NAV_OUTPUT_STABILITY_SIGNATURE_VERSION,
        };
        populate_solution_dops(&mut solution, geometry_dops.as_ref());
        populate_solution_trace_identity(obs, &mut solution);
        Some(solution)
    }
}

fn navigation_filter_solution_evidence(
    filter: &NavigationFilter,
    used_satellite_count: usize,
    innovation_rms_m: f64,
    ionosphere_supported: bool,
) -> NavigationSolutionEvidence {
    let covariance_supported = false;
    let residual_supported =
        used_satellite_count > 0 && innovation_rms_m.is_finite() && innovation_rms_m >= 0.0;
    let ambiguity_supported = !filter.ambiguity.indices.is_empty();
    let correction_supported = ionosphere_supported && filter.tropo_enabled;
    let integrity_supported = false;

    NavigationSolutionEvidence {
        covariance_supported,
        residual_supported,
        ambiguity_supported,
        correction_supported,
        integrity_supported,
    }
}

fn navigation_filter_status_floor(
    evidence: NavigationSolutionEvidence,
) -> (SolutionStatus, Vec<String>) {
    if evidence.supports_float_claim() {
        return (SolutionStatus::Float, Vec::new());
    }

    (
        SolutionStatus::CodeOnly,
        evidence
            .missing_reasons()
            .into_iter()
            .map(|reason| format!("status_floor={reason}"))
            .collect(),
    )
}

fn navigation_geometry_threshold_violations(
    thresholds: NavigationFilterThresholds,
    used_satellite_count: usize,
    geometry_dops: Option<&PositionDops>,
) -> Vec<String> {
    let mut violations = Vec::new();
    if used_satellite_count < thresholds.min_used_satellites {
        violations.push(format!("minimum_usable_satellites={}", thresholds.min_used_satellites));
        violations.push(format!(
            "used_satellites_below_threshold:{used_satellite_count}<{}",
            thresholds.min_used_satellites
        ));
    }
    match geometry_dops {
        Some(dops) => {
            if dops.pdop > thresholds.max_pdop {
                violations.push(format!(
                    "pdop_above_threshold:{:.3}>{:.3}",
                    dops.pdop, thresholds.max_pdop
                ));
            }
            if dops.gdop > thresholds.max_gdop {
                violations.push(format!(
                    "gdop_above_threshold:{:.3}>{:.3}",
                    dops.gdop, thresholds.max_gdop
                ));
            }
        }
        None if used_satellite_count >= thresholds.min_used_satellites => {
            violations.push("geometry_dops_unavailable".to_string());
        }
        None => {}
    }
    violations
}

fn populate_solution_dops(solution: &mut NavSolutionEpoch, geometry_dops: Option<&PositionDops>) {
    if let Some(dops) = geometry_dops {
        solution.pdop = dops.pdop;
        solution.hdop = Some(dops.hdop);
        solution.vdop = Some(dops.vdop);
        solution.gdop = Some(dops.gdop);
        solution.tdop = Some(dops.tdop);
    }
}

fn navigation_refusal_epoch(
    obs: &ObsEpoch,
    sat_count: usize,
    used_sat_count: usize,
    rejected_sat_count: usize,
    refusal_class: Option<NavRefusalClass>,
    explain_reasons: Vec<String>,
) -> NavSolutionEpoch {
    let status = refusal_status(refusal_class);
    NavSolutionEpoch {
        epoch: Epoch { index: obs.epoch_idx },
        t_rx_s: obs.t_rx_s,
        source_time: obs.source_time,
        ecef_x_m: Meters(0.0),
        ecef_y_m: Meters(0.0),
        ecef_z_m: Meters(0.0),
        position_covariance_ecef_m2: None,
        latitude_deg: 0.0,
        longitude_deg: 0.0,
        altitude_m: Meters(0.0),
        clock_bias_s: Seconds(0.0),
        clock_bias_m: Meters(0.0),
        clock_drift_s_per_s: 0.0,
        pdop: 0.0,
        pre_fit_residual_rms_m: None,
        post_fit_residual_rms_m: None,
        rms_m: Meters(0.0),
        status,
        quality: status.quality_flag(),
        validity: SolutionValidity::Invalid,
        valid: false,
        processing_ms: None,
        residuals: Vec::new(),
        constellation_residual_rms: Vec::new(),
        health: Vec::new(),
        isb: Vec::new(),
        sigma_e_m: None,
        sigma_n_m: None,
        sigma_u_m: None,
        horizontal_error_ellipse_major_axis_m: None,
        horizontal_error_ellipse_minor_axis_m: None,
        horizontal_error_ellipse_azimuth_deg: None,
        sigma_h_m: None,
        sigma_v_m: None,
        innovation_rms_m: None,
        normalized_innovation_rms: None,
        normalized_innovation_max: None,
        ekf_innovation_rms: None,
        ekf_condition_number: None,
        ekf_whiteness_ratio: None,
        ekf_predicted_variance: None,
        ekf_observed_variance: None,
        integrity_hpl_m: None,
        integrity_vpl_m: None,
        model_version: NAV_SOLUTION_MODEL_VERSION,
        lifecycle_state: solution_lifecycle_state(status),
        uncertainty_class: solution_uncertainty_class(status),
        assumptions: None,
        refusal_class,
        artifact_id: String::new(),
        source_observation_epoch_id: String::new(),
        explain_decision: "refused".to_string(),
        explain_reasons,
        provenance: None,
        sat_count,
        used_sat_count,
        rejected_sat_count,
        hdop: None,
        vdop: None,
        gdop: None,
        tdop: None,
        stability_signature: String::new(),
        stability_signature_version: NAV_OUTPUT_STABILITY_SIGNATURE_VERSION,
    }
}

fn solution_validity(status: SolutionStatus) -> SolutionValidity {
    if bijux_gnss_core::api::is_solution_valid(status) {
        SolutionValidity::Converging
    } else {
        SolutionValidity::Invalid
    }
}

fn solution_lifecycle_state(status: SolutionStatus) -> NavLifecycleState {
    match status {
        SolutionStatus::Unavailable => NavLifecycleState::Unavailable,
        SolutionStatus::Refused => NavLifecycleState::Refused,
        SolutionStatus::Degraded => NavLifecycleState::Degraded,
        SolutionStatus::IntegrityFailed => NavLifecycleState::IntegrityFailed,
        SolutionStatus::Diverged => NavLifecycleState::Diverged,
        SolutionStatus::CodeOnly => NavLifecycleState::CodeOnly,
        SolutionStatus::Float => NavLifecycleState::Float,
        SolutionStatus::Fixed => NavLifecycleState::Fixed,
    }
}

fn solution_uncertainty_class(status: SolutionStatus) -> NavUncertaintyClass {
    if !bijux_gnss_core::api::is_solution_valid(status) {
        NavUncertaintyClass::Unknown
    } else if status == SolutionStatus::Degraded {
        NavUncertaintyClass::High
    } else {
        NavUncertaintyClass::Medium
    }
}

fn refusal_status(refusal_class: Option<NavRefusalClass>) -> SolutionStatus {
    match refusal_class {
        Some(
            NavRefusalClass::UnsupportedConstellation
                | NavRefusalClass::MixedConstellationInput
                | NavRefusalClass::InvalidEphemeris
                | NavRefusalClass::PartialDecodedNavigationState,
        ) => SolutionStatus::Unavailable,
        Some(_) => SolutionStatus::Refused,
        None => SolutionStatus::Unavailable,
    }
}

fn position_is_uninitialized(filter: &NavigationFilter) -> bool {
    let radius_m =
        (filter.ekf.x[0].powi(2) + filter.ekf.x[1].powi(2) + filter.ekf.x[2].powi(2)).sqrt();
    !radius_m.is_finite() || radius_m < 1.0
}

fn prime_state_from_wls(
    filter: &mut NavigationFilter,
    obs: &ObsEpoch,
    ephemerides: &[GpsEphemeris],
    klobuchar: Option<&KlobucharCoefficients>,
    tropo_enabled: bool,
) {
    let observations = obs
        .sats
        .iter()
        .filter(|sat| sat.signal_id.sat.constellation == Constellation::Gps)
        .map(|sat| PositionObservation {
            sat: sat.signal_id.sat,
            pseudorange_m: sat.pseudorange_m.0,
            doppler_hz: Some(sat.doppler_hz.0),
            doppler_var_hz2: Some(sat.doppler_var_hz2),
            cn0_dbhz: sat.cn0_dbhz,
            elevation_deg: sat.elevation_deg,
            weight: 1.0,
            gps_receive_time: obs.gps_time(),
            signal_timing: sat.timing,
            signal_id: Some(sat.signal_id),
        })
        .filter(|observation| {
            bijux_gnss_nav::api::position_observation_has_valid_satellite_time(
                observation,
                obs.t_rx_s.0,
            )
        })
        .collect::<Vec<_>>();
    if observations.len() < 4 {
        return;
    }
    let Some(solution) = PositionSolver {
        apply_troposphere: tropo_enabled,
        ..PositionSolver::new()
    }
    .solve_wls_with_broadcast_ionosphere(
        &observations,
        ephemerides,
        obs.gps_time().map(|gps_time| gps_time.tow_s).unwrap_or(obs.t_rx_s.0),
        klobuchar,
    ) else {
        return;
    };
    filter.ekf.x[0] = solution.ecef_x_m;
    filter.ekf.x[1] = solution.ecef_y_m;
    filter.ekf.x[2] = solution.ecef_z_m;
    if filter.ekf.x.len() > 6 {
        filter.ekf.x[6] = solution.clock_bias_s;
    }
}

fn estimate_klobuchar_delay_m(
    klobuchar: Option<&KlobucharCoefficients>,
    receiver_ecef_m: [f64; 3],
    receive_tow_s: f64,
    state: &GpsSatState,
) -> f64 {
    let Some(coefficients) = klobuchar else {
        return 0.0;
    };
    let radius_m =
        (receiver_ecef_m[0].powi(2) + receiver_ecef_m[1].powi(2) + receiver_ecef_m[2].powi(2))
            .sqrt();
    if !radius_m.is_finite() || radius_m < 1.0 {
        return 0.0;
    }
    let (lat_deg, lon_deg, alt_m) =
        ecef_to_geodetic(receiver_ecef_m[0], receiver_ecef_m[1], receiver_ecef_m[2]);
    let receiver = Llh { lat_deg, lon_deg, alt_m };
    let (azimuth_deg, elevation_deg) = elevation_azimuth_deg(
        receiver_ecef_m[0],
        receiver_ecef_m[1],
        receiver_ecef_m[2],
        state.x_m,
        state.y_m,
        state.z_m,
    );
    if !elevation_deg.is_finite() || elevation_deg <= 0.0 {
        return 0.0;
    }
    let model = bijux_gnss_nav::api::KlobucharModel::new(*coefficients);
    bijux_gnss_nav::api::IonosphereModel::delay_m(
        &model,
        receiver,
        azimuth_deg,
        elevation_deg,
        Seconds(receive_tow_s),
    )
}

fn estimate_saastamoinen_delay_m(
    tropo_enabled: bool,
    receiver_ecef_m: [f64; 3],
    state: &GpsSatState,
) -> f64 {
    if !tropo_enabled {
        return 0.0;
    }
    let radius_m =
        (receiver_ecef_m[0].powi(2) + receiver_ecef_m[1].powi(2) + receiver_ecef_m[2].powi(2))
            .sqrt();
    if !radius_m.is_finite() || !(6_000_000.0..=7_000_000.0).contains(&radius_m) {
        return 0.0;
    }
    let (lat_deg, lon_deg, alt_m) =
        ecef_to_geodetic(receiver_ecef_m[0], receiver_ecef_m[1], receiver_ecef_m[2]);
    if !lat_deg.is_finite()
        || !lon_deg.is_finite()
        || !alt_m.is_finite()
        || !(-1_000.0..=20_000.0).contains(&alt_m)
    {
        return 0.0;
    }
    let receiver = Llh { lat_deg, lon_deg, alt_m };
    let (_azimuth_deg, elevation_deg) = elevation_azimuth_deg(
        receiver_ecef_m[0],
        receiver_ecef_m[1],
        receiver_ecef_m[2],
        state.x_m,
        state.y_m,
        state.z_m,
    );
    if !elevation_deg.is_finite() || elevation_deg <= 0.0 {
        return 0.0;
    }
    let model = bijux_gnss_nav::api::SaastamoinenModel;
    bijux_gnss_nav::api::TroposphereModel::delay_m(&model, receiver, elevation_deg, Seconds(0.0))
}

fn navigation_satellite_state(
    sat: &ObsSatellite,
    eph: &GpsEphemeris,
    receive_tow_s: f64,
) -> GpsSatState {
    let signal_travel_time_s = sat
        .timing
        .map(|timing| timing.signal_travel_time_s.0)
        .unwrap_or(sat.pseudorange_m.0 / 299_792_458.0);
    sat_state_gps_l1ca_at_receive_time(eph, receive_tow_s, signal_travel_time_s)
}

fn populate_solution_trace_identity(obs: &ObsEpoch, solution: &mut NavSolutionEpoch) {
    let source_observation_epoch_id = obs
        .manifest
        .as_ref()
        .map(|manifest| manifest.epoch_id.clone())
        .unwrap_or_else(|| obs_epoch_stability_key(obs));
    solution.source_observation_epoch_id = source_observation_epoch_id.clone();
    solution.artifact_id = format!(
        "nav-epoch-{:010}-{}",
        solution.epoch.index,
        trace_short_id(&source_observation_epoch_id)
    );
    solution.stability_signature = output_stability_signature(solution);
}

fn output_stability_signature(solution: &NavSolutionEpoch) -> String {
    format!(
        "navsig:v{}:epoch={}:src={}@{}:status={:?}:lifecycle={:?}:valid={}:sat={}:used={}:rej={}:pdop={:.3}:hdop={}:vdop={}:gdop={}:tdop={}:rms={:.3}:decision={}",
        NAV_OUTPUT_STABILITY_SIGNATURE_VERSION,
        solution.epoch.index,
        trace_short_id(&solution.source_observation_epoch_id),
        solution.source_time.sample_index,
        solution.status,
        solution.lifecycle_state,
        solution.valid,
        solution.sat_count,
        solution.used_sat_count,
        solution.rejected_sat_count,
        solution.pdop,
        format_optional_dop(solution.hdop),
        format_optional_dop(solution.vdop),
        format_optional_dop(solution.gdop),
        format_optional_dop(solution.tdop),
        solution.rms_m.0,
        solution.explain_decision
    )
}

fn format_optional_dop(value: Option<f64>) -> String {
    value.map(|dop| format!("{dop:.3}")).unwrap_or_else(|| "na".to_string())
}

fn trace_short_id(value: &str) -> String {
    value.chars().take(16).collect()
}

#[cfg(test)]
mod tests {
    use super::{
        navigation_filter_solution_evidence, navigation_filter_status_floor, NavigationFilter,
        NavigationSolutionEvidence,
    };
    use bijux_gnss_core::api::SolutionStatus;

    #[test]
    fn navigation_filter_status_floor_blocks_float_without_full_evidence() {
        let evidence = NavigationSolutionEvidence {
            covariance_supported: true,
            residual_supported: true,
            ambiguity_supported: true,
            correction_supported: true,
            integrity_supported: false,
        };

        let (status, reasons) = navigation_filter_status_floor(evidence);
        assert_eq!(status, SolutionStatus::CodeOnly);
        assert!(reasons.iter().any(|reason| reason == "status_floor=missing_integrity_evidence"));
    }

    #[test]
    fn navigation_filter_status_floor_allows_float_with_full_evidence() {
        let evidence = NavigationSolutionEvidence {
            covariance_supported: true,
            residual_supported: true,
            ambiguity_supported: true,
            correction_supported: true,
            integrity_supported: true,
        };

        let (status, reasons) = navigation_filter_status_floor(evidence);
        assert_eq!(status, SolutionStatus::Float);
        assert!(reasons.is_empty());
    }

    #[test]
    fn navigation_filter_solution_evidence_tracks_available_support_surfaces() {
        let mut filter = NavigationFilter::new();
        filter.ambiguity.indices.insert("gps:L1".to_string(), 8);

        let evidence = navigation_filter_solution_evidence(&filter, 4, 0.5, true);
        assert_eq!(
            evidence,
            NavigationSolutionEvidence {
                covariance_supported: false,
                residual_supported: true,
                ambiguity_supported: true,
                correction_supported: true,
                integrity_supported: false,
            }
        );
    }
}
