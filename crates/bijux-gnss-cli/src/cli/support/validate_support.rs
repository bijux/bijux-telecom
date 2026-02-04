fn build_validation_report(
    tracks: &[bijux_gnss_receiver::tracking::TrackingResult],
    obs: &[ObsEpoch],
    solutions: &[bijux_gnss_core::NavSolutionEpoch],
    reference: &[ValidationReferenceEpoch],
    sample_rate_hz: f64,
    products_ok: bool,
    product_fallbacks: Vec<String>,
) -> Result<ValidationReport> {
    let mut ref_map = std::collections::BTreeMap::new();
    for r in reference {
        ref_map.insert(r.epoch_idx, r);
    }

    let mut horiz_errors = Vec::new();
    let mut vert_errors = Vec::new();
    let mut horiz_by_time = Vec::new();
    let mut vert_by_time = Vec::new();
    let mut fix_timeline = Vec::new();
    let mut residuals = Vec::new();
    let mut nees_values = Vec::new();

    for sol in solutions {
        if let Some(r) = ref_map.get(&sol.epoch.index) {
            let (x, y, z) = reference_ecef(r);
            let dx = sol.ecef_x_m.0 - x;
            let dy = sol.ecef_y_m.0 - y;
            let dz = sol.ecef_z_m.0 - z;
            let horiz = (dx * dx + dy * dy).sqrt();
            let vert = dz.abs();
            horiz_errors.push(horiz);
            vert_errors.push(vert);
            horiz_by_time.push((sol.t_rx_s.0, horiz));
            vert_by_time.push((sol.t_rx_s.0, vert));
            if let (Some(sig_h), Some(sig_v)) = (sol.sigma_h_m, sol.sigma_v_m) {
                if sig_h.0 > 0.0 && sig_v.0 > 0.0 {
                    let (e, n, u) = bijux_gnss_nav::ecef_to_enu(
                        sol.ecef_x_m.0,
                        sol.ecef_y_m.0,
                        sol.ecef_z_m.0,
                        r.latitude_deg,
                        r.longitude_deg,
                        r.altitude_m,
                    );
                    let nees = (e * e + n * n) / (sig_h.0 * sig_h.0)
                        + (u * u) / (sig_v.0 * sig_v.0);
                    nees_values.push(nees);
                }
            }
        }
        fix_timeline.push(FixTimelineEntry {
            epoch_idx: sol.epoch.index,
            fixed: matches!(sol.status, bijux_gnss_core::SolutionStatus::Fixed),
        });
        let mut per_sat = Vec::new();
        let mut rejected = Vec::new();
        for r in &sol.residuals {
            if r.rejected {
                rejected.push(r.sat);
            } else {
                per_sat.push((r.sat, r.residual_m.0));
            }
        }
        residuals.push(NavResidualReport {
            epoch_idx: sol.epoch.index,
            rms_m: sol.rms_m.0,
            pdop: sol.pdop,
            residuals: per_sat,
            rejected,
        });
    }

    let horiz_stats = to_validation_stats(stats(&horiz_errors));
    let vert_stats = to_validation_stats(stats(&vert_errors));
    let convergence = convergence_report(&horiz_by_time, &vert_by_time);
    let budgets = ValidationBudgets::default();
    let violations = check_budgets(tracks, solutions, &budgets);
    let time_consistency = check_time_consistency(tracks, sample_rate_hz);
    let consistency = check_solution_consistency(solutions);
    let inter_frequency_alignment = bijux_gnss_core::check_inter_frequency_alignment(obs);
    let multi_freq_present = obs.iter().any(|e| {
        let mut by_sat: std::collections::BTreeMap<SatId, std::collections::BTreeSet<_>> =
            std::collections::BTreeMap::new();
        for sat in &e.sats {
            by_sat
                .entry(sat.signal_id.sat)
                .or_default()
                .insert(sat.signal_id.band);
        }
        by_sat.values().any(|bands| bands.len() > 1)
    });
    let combos = bijux_gnss_nav::combinations_from_obs_epochs(
        obs,
        bijux_gnss_core::SignalBand::L1,
        bijux_gnss_core::SignalBand::L2,
    );
    let combinations_valid = combos.iter().all(|c| c.status == "ok") && !combos.is_empty();
    let nis_values: Vec<f64> = solutions
        .iter()
        .filter_map(|s| {
            let pred = s.ekf_predicted_variance?;
            if pred > 0.0 {
                Some((s.ekf_innovation_rms.unwrap_or(0.0).powi(2)) / pred)
            } else {
                None
            }
        })
        .collect();
    let nis_mean = if nis_values.is_empty() {
        None
    } else {
        Some(nis_values.iter().sum::<f64>() / nis_values.len() as f64)
    };
    let nees_mean = if nees_values.is_empty() {
        None
    } else {
        Some(nees_values.iter().sum::<f64>() / nees_values.len() as f64)
    };
    let mut consistency_warnings = Vec::new();
    if let Some(nis) = nis_mean {
        if !(0.1..=10.0).contains(&nis) {
            consistency_warnings.push(format!("NIS out of expected range: {nis:.3}"));
        }
    }
    if let Some(nees) = nees_mean {
        if !(0.1..=10.0).contains(&nees) {
            consistency_warnings.push(format!("NEES out of expected range: {nees:.3}"));
        }
    }

    Ok(ValidationReport {
        samples: tracks.iter().map(|t| t.epochs.len()).sum(),
        epochs: solutions.len(),
        horiz_error_m: horiz_stats,
        vert_error_m: vert_stats,
        convergence,
        fix_timeline,
        residuals,
        time_consistency,
        consistency,
        budgets,
        budget_violations: violations,
        nis_mean,
        nees_mean,
        consistency_warnings,
        inter_frequency_alignment,
        ppp_readiness: PppReadinessReport {
            multi_freq_present,
            combinations_valid,
            products_ok,
            product_fallbacks,
        },
    })
}

fn convergence_report(horiz: &[(f64, f64)], vert: &[(f64, f64)]) -> ConvergenceReport {
    let mut t1 = None;
    let mut t03 = None;
    let mut t01 = None;
    let mut worst_h: f64 = 0.0;
    let mut worst_v: f64 = 0.0;
    for (t, h) in horiz {
        worst_h = worst_h.max(*h);
        if t1.is_none() && *h <= 1.0 {
            t1 = Some(*t);
        }
        if t03.is_none() && *h <= 0.3 {
            t03 = Some(*t);
        }
        if t01.is_none() && *h <= 0.1 {
            t01 = Some(*t);
        }
    }
    for (_t, v) in vert {
        worst_v = worst_v.max(*v);
    }
    ConvergenceReport {
        time_to_1m_s: t1,
        time_to_0_3m_s: t03,
        time_to_0_1m_s: t01,
        worst_horiz_m: worst_h,
        worst_vert_m: worst_v,
    }
}

#[allow(dead_code)]
fn build_ppp_config(profile: &ReceiverProfile) -> PppConfig {
    let p = &profile.navigation.ppp;
    let ar_mode = match p.ar_mode.as_str() {
        "ppp_ar_wide_lane" => bijux_gnss_nav::PppArMode::PppArWideLane,
        "ppp_ar_narrow_lane" => bijux_gnss_nav::PppArMode::PppArNarrowLane,
        _ => bijux_gnss_nav::PppArMode::FloatPpp,
    };
    PppConfig {
        enable_iono_state: p.enable_iono_state,
        use_iono_free: p.use_iono_free,
        use_doppler: p.use_doppler,
        ar_mode,
        ar_ratio_threshold: p.ar_ratio_threshold,
        ar_stability_epochs: p.ar_stability_epochs,
        ar_max_sats: p.ar_max_sats,
        ar_use_elevation: p.ar_use_elevation,
        prune_after_epochs: p.prune_after_epochs,
        reset_gap_s: p.reset_gap_s,
        residual_gate_m: p.residual_gate_m,
        drift_window_epochs: p.drift_window_epochs as usize,
        drift_threshold_m: p.drift_threshold_m,
        checkpoint_interval_epochs: p.checkpoint_interval_epochs,
        process_noise: PppProcessNoise {
            clock_drift_s: p.noise_clock_drift,
            ztd_m: p.noise_ztd,
            iono_m: p.noise_iono,
            ambiguity_cycles: p.noise_ambiguity,
        },
        weighting: WeightingConfig {
            enabled: profile.navigation.weighting.enabled,
            min_elev_deg: profile.navigation.weighting.min_elev_deg,
            elev_exponent: profile.navigation.weighting.elev_exponent,
            cn0_ref_dbhz: profile.navigation.weighting.cn0_ref_dbhz,
            min_weight: profile.navigation.weighting.min_weight,
        },
        convergence: bijux_gnss_nav::PppConvergenceConfig {
            min_time_s: p.convergence_min_time_s,
            pos_rate_mps: p.convergence_pos_rate_mps,
            sigma_h_m: p.convergence_sigma_h_m,
            sigma_v_m: p.convergence_sigma_v_m,
        },
    }
}

#[allow(dead_code)]
fn ppp_evaluation_report(
    solutions: &[bijux_gnss_nav::PppSolutionEpoch],
    reference: &[ValidationReferenceEpoch],
) -> PppEvaluationReport {
    let mut ref_map = std::collections::BTreeMap::new();
    for r in reference {
        ref_map.insert(r.epoch_idx, r);
    }
    let mut horiz = Vec::new();
    let mut vert = Vec::new();
    let mut residual_rms = Vec::new();
    let mut last_conv = None;
    for sol in solutions {
        residual_rms.push(sol.rms_m);
        if let Some(r) = ref_map.get(&sol.epoch_idx) {
            let (x, y, z) = reference_ecef(r);
            let dx = sol.ecef_x_m - x;
            let dy = sol.ecef_y_m - y;
            let dz = sol.ecef_z_m - z;
            horiz.push((dx * dx + dy * dy).sqrt());
            vert.push(dz.abs());
        }
        last_conv = Some(sol.convergence.clone());
    }
    let horiz_rms = if horiz.is_empty() {
        None
    } else {
        Some((horiz.iter().map(|v| v * v).sum::<f64>() / horiz.len() as f64).sqrt())
    };
    let vert_rms = if vert.is_empty() {
        None
    } else {
        Some((vert.iter().map(|v| v * v).sum::<f64>() / vert.len() as f64).sqrt())
    };
    let residual_rms_m = if residual_rms.is_empty() {
        0.0
    } else {
        (residual_rms.iter().map(|v| v * v).sum::<f64>() / residual_rms.len() as f64).sqrt()
    };
    let (t1, t10, t1cm) = last_conv
        .map(|c| {
            (
                c.time_to_first_meter_s,
                c.time_to_decimeter_s,
                c.time_to_centimeter_s,
            )
        })
        .unwrap_or((None, None, None));
    PppEvaluationReport {
        epochs: solutions.len(),
        horiz_rms_m: horiz_rms,
        vert_rms_m: vert_rms,
        time_to_first_meter_s: t1,
        time_to_decimeter_s: t10,
        time_to_centimeter_s: t1cm,
        residual_rms_m,
        checkpoint_path: None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn golden_reference_validation() {
        let sol = bijux_gnss_core::NavSolutionEpoch {
            epoch: bijux_gnss_core::Epoch { index: 0 },
            t_rx_s: bijux_gnss_core::Seconds(0.0),
            ecef_x_m: bijux_gnss_core::Meters(1.0),
            ecef_y_m: bijux_gnss_core::Meters(2.0),
            ecef_z_m: bijux_gnss_core::Meters(3.0),
            latitude_deg: 0.0,
            longitude_deg: 0.0,
            altitude_m: bijux_gnss_core::Meters(0.0),
            clock_bias_s: bijux_gnss_core::Seconds(0.0),
            pdop: 1.0,
            rms_m: bijux_gnss_core::Meters(0.0),
            status: bijux_gnss_core::SolutionStatus::Converged,
            valid: true,
            processing_ms: None,
            residuals: Vec::new(),
            isb: Vec::new(),
            sigma_h_m: None,
            sigma_v_m: None,
            ekf_innovation_rms: None,
            ekf_condition_number: None,
            ekf_whiteness_ratio: None,
            ekf_predicted_variance: None,
            ekf_observed_variance: None,
        };
        let reference = ValidationReferenceEpoch {
            epoch_idx: 0,
            t_rx_s: Some(0.0),
            latitude_deg: 0.0,
            longitude_deg: 0.0,
            altitude_m: 0.0,
            ecef_x_m: Some(1.0),
            ecef_y_m: Some(2.0),
            ecef_z_m: Some(3.0),
            vel_x_mps: None,
            vel_y_mps: None,
            vel_z_mps: None,
        };
        let report = build_validation_report(
            &[],
            &[],
            &[sol],
            &[reference],
            1.0,
            false,
            Vec::new(),
        )
        .expect("validation report");
        assert!(report.horiz_error_m.rms <= 1e-6);
        assert!(report.vert_error_m.rms <= 1e-6);
    }
}



impl Default for ValidationBudgets {
    fn default() -> Self {
        Self {
            acq_doppler_hz: 500.0,
            acq_code_phase_samples: 5.0,
            tracking_carrier_jitter_hz: 50.0,
            ephemeris_parity_rate_min: 0.9,
            pvt_max_iterations: 10,
        }
    }
}

fn check_budgets(
    tracks: &[bijux_gnss_receiver::tracking::TrackingResult],
    solutions: &[bijux_gnss_core::NavSolutionEpoch],
    budgets: &ValidationBudgets,
) -> Vec<String> {
    let mut violations = Vec::new();
    for track in tracks {
        let mut carriers = Vec::new();
        for e in &track.epochs {
            carriers.push(e.carrier_hz.0);
        }
        if carriers.len() > 1 {
            let mean = carriers.iter().sum::<f64>() / carriers.len() as f64;
            let var = carriers
                .iter()
                .map(|v| (v - mean) * (v - mean))
                .sum::<f64>()
                / carriers.len() as f64;
            let std = var.sqrt();
            if std > budgets.tracking_carrier_jitter_hz {
                violations.push(format!(
                    "tracking jitter too high for PRN {}: {:.1} Hz",
                    track.sat.prn, std
                ));
            }
        }
    }
    for sol in solutions {
        if sol.residuals.len() >= 4 && sol.rms_m.0.is_nan() {
            violations.push(format!("PVT RMS is NaN at epoch {}", sol.epoch.index));
        }
    }
    violations
}

fn schema_path(name: &str) -> PathBuf {
    let base = Path::new(env!("CARGO_MANIFEST_DIR"));
    base.join("../../schemas").join(name)
}

fn to_validation_stats(summary: StatsSummary) -> ValidationErrorStats {
    ValidationErrorStats {
        count: summary.count,
        mean: summary.mean,
        median: summary.median,
        rms: summary.rms,
        p95: summary.p95,
        max: summary.max,
    }
}
