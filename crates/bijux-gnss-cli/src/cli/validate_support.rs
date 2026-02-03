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
    let mut residuals = Vec::new();
    let mut nees_values = Vec::new();

    for sol in solutions {
        if let Some(r) = ref_map.get(&sol.epoch.index) {
            let (x, y, z) = lla_to_ecef(r.latitude_deg, r.longitude_deg, r.altitude_m);
            let dx = sol.ecef_x_m - x;
            let dy = sol.ecef_y_m - y;
            let dz = sol.ecef_z_m - z;
            let horiz = (dx * dx + dy * dy).sqrt();
            let vert = dz.abs();
            horiz_errors.push(horiz);
            vert_errors.push(vert);
            if let (Some(sig_h), Some(sig_v)) = (sol.sigma_h_m, sol.sigma_v_m) {
                if sig_h > 0.0 && sig_v > 0.0 {
                    let (e, n, u) = bijux_gnss_nav::ecef_to_enu(
                        sol.ecef_x_m,
                        sol.ecef_y_m,
                        sol.ecef_z_m,
                        r.latitude_deg,
                        r.longitude_deg,
                        r.altitude_m,
                    );
                    let nees = (e * e + n * n) / (sig_h * sig_h) + (u * u) / (sig_v * sig_v);
                    nees_values.push(nees);
                }
            }
        }
        let mut per_sat = Vec::new();
        let mut rejected = Vec::new();
        for r in &sol.residuals {
            if r.rejected {
                rejected.push(r.sat);
            } else {
                per_sat.push((r.sat, r.residual_m));
            }
        }
        residuals.push(NavResidualReport {
            epoch_idx: sol.epoch.index,
            rms_m: sol.rms_m,
            pdop: sol.pdop,
            residuals: per_sat,
            rejected,
        });
    }

    let horiz_stats = stats(&horiz_errors);
    let vert_stats = stats(&vert_errors);
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
    let combos = bijux_gnss_receiver::combinations::combinations_from_obs_epochs(
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

    Ok(ValidationReport {
        samples: tracks.iter().map(|t| t.epochs.len()).sum(),
        epochs: solutions.len(),
        horiz_error_m: horiz_stats,
        vert_error_m: vert_stats,
        residuals,
        time_consistency,
        consistency,
        budgets,
        budget_violations: violations,
        nis_mean,
        nees_mean,
        inter_frequency_alignment,
        ppp_readiness: PppReadinessReport {
            multi_freq_present,
            combinations_valid,
            products_ok,
            product_fallbacks,
        },
    })
}

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
            let (x, y, z) = lla_to_ecef(r.latitude_deg, r.longitude_deg, r.altitude_m);
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


#[derive(Debug, Serialize)]
struct ReferenceCompareStats {
    count: usize,
    horiz_rms_m: f64,
    vert_rms_m: f64,
}

fn reference_compare(
    solutions: &[bijux_gnss_core::NavSolutionEpoch],
    reference: &[ValidationReferenceEpoch],
) -> (Vec<String>, ReferenceCompareStats) {
    let mut ref_map = std::collections::BTreeMap::new();
    for r in reference {
        ref_map.insert(r.epoch_idx, r);
    }
    let mut rows = Vec::new();
    rows.push("epoch_idx,dx_m,dy_m,dz_m,horiz_m,vert_m".to_string());
    let mut horiz = Vec::new();
    let mut vert = Vec::new();
    for sol in solutions {
        if let Some(r) = ref_map.get(&sol.epoch.index) {
            let (x, y, z) = lla_to_ecef(r.latitude_deg, r.longitude_deg, r.altitude_m);
            let dx = sol.ecef_x_m - x;
            let dy = sol.ecef_y_m - y;
            let dz = sol.ecef_z_m - z;
            let h = (dx * dx + dy * dy).sqrt();
            let v = dz.abs();
            horiz.push(h);
            vert.push(v);
            rows.push(format!(
                "{},{:.4},{:.4},{:.4},{:.4},{:.4}",
                sol.epoch.index, dx, dy, dz, h, v
            ));
        }
    }
    let horiz_rms = if horiz.is_empty() {
        0.0
    } else {
        (horiz.iter().map(|v| v * v).sum::<f64>() / horiz.len() as f64).sqrt()
    };
    let vert_rms = if vert.is_empty() {
        0.0
    } else {
        (vert.iter().map(|v| v * v).sum::<f64>() / vert.len() as f64).sqrt()
    };
    (
        rows,
        ReferenceCompareStats {
            count: horiz.len(),
            horiz_rms_m: horiz_rms,
            vert_rms_m: vert_rms,
        },
    )
}

fn check_solution_consistency(
    solutions: &[bijux_gnss_core::NavSolutionEpoch],
) -> SolutionConsistencyReport {
    let mut position_jump_count = 0;
    let mut clock_jump_count = 0;
    let mut pdop_spike_count = 0;
    let mut warnings = Vec::new();
    let mut prev: Option<&bijux_gnss_core::NavSolutionEpoch> = None;
    let mut prev_pdop: Option<f64> = None;
    for sol in solutions {
        if let Some(prev_sol) = prev {
            let dx = sol.ecef_x_m - prev_sol.ecef_x_m;
            let dy = sol.ecef_y_m - prev_sol.ecef_y_m;
            let dz = sol.ecef_z_m - prev_sol.ecef_z_m;
            let dist = (dx * dx + dy * dy + dz * dz).sqrt();
            if dist > 50.0 {
                position_jump_count += 1;
            }
            let clock_jump = (sol.clock_bias_s - prev_sol.clock_bias_s).abs();
            if clock_jump > 1e-3 {
                clock_jump_count += 1;
            }
        }
        if let Some(prev_pdop) = prev_pdop {
            if sol.pdop > prev_pdop * 2.5 && sol.pdop > 6.0 {
                pdop_spike_count += 1;
            }
        }
        prev_pdop = Some(sol.pdop);
        prev = Some(sol);
    }
    if position_jump_count > 0 {
        warnings.push("position jumps detected".to_string());
    }
    if clock_jump_count > 0 {
        warnings.push("clock jumps detected".to_string());
    }
    if pdop_spike_count > 0 {
        warnings.push("pdop spikes detected".to_string());
    }
    SolutionConsistencyReport {
        position_jump_count,
        clock_jump_count,
        pdop_spike_count,
        warnings,
    }
}

fn check_time_consistency(
    tracks: &[bijux_gnss_receiver::tracking::TrackingResult],
    sample_rate_hz: f64,
) -> TimeConsistencyReport {
    let mut epoch_backward = 0;
    let mut epoch_gaps = 0;
    let mut sample_backward = 0;
    let mut warnings = Vec::new();
    let mut epochs_checked = 0;
    let mut sample_step_mismatch = 0;
    let configured_step = if sample_rate_hz > 0.0 {
        Some((sample_rate_hz * 0.001).round().max(1.0) as u64)
    } else {
        None
    };
    let mut expected_step: Option<u64> = configured_step;
    let mut sample_step_total = 0u64;
    let mut sample_step_count = 0u64;

    for track in tracks {
        let mut prev_epoch: Option<u64> = None;
        let mut prev_sample: Option<u64> = None;
        for epoch in &track.epochs {
            epochs_checked += 1;
            if let Some(prev) = prev_epoch {
                if epoch.epoch.index < prev {
                    epoch_backward += 1;
                } else if epoch.epoch.index > prev + 1 {
                    epoch_gaps += 1;
                }
            }
            if let Some(prev) = prev_sample {
                if epoch.sample_index < prev {
                    sample_backward += 1;
                } else {
                    let step = epoch.sample_index - prev;
                    if let Some(expected) = expected_step {
                        if step != expected {
                            sample_step_mismatch += 1;
                        }
                    } else {
                        expected_step = Some(step);
                    }
                    sample_step_total = sample_step_total.saturating_add(step);
                    sample_step_count = sample_step_count.saturating_add(1);
                }
            }
            prev_epoch = Some(epoch.epoch.index);
            prev_sample = Some(epoch.sample_index);
        }
    }

    if epoch_backward > 0 {
        warnings.push("epoch index went backwards".to_string());
    }
    if epoch_gaps > 0 {
        warnings.push("epoch index gaps detected".to_string());
    }
    if sample_backward > 0 {
        warnings.push("sample index went backwards".to_string());
    }
    if sample_step_mismatch > 0 {
        warnings.push("sample cadence mismatch detected".to_string());
    }

    TimeConsistencyReport {
        channels: tracks.len(),
        epochs_checked,
        epoch_backward,
        epoch_gaps,
        sample_backward,
        sample_step_mismatch,
        expected_step,
        observed_step_mean: if sample_step_count > 0 {
            Some(sample_step_total as f64 / sample_step_count as f64)
        } else {
            None
        },
        warnings,
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
            carriers.push(e.carrier_hz);
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
        if sol.residuals.len() >= 4 && sol.rms_m.is_nan() {
            violations.push(format!("PVT RMS is NaN at epoch {}", sol.epoch.index));
        }
    }
    violations
}

fn schema_path(name: &str) -> PathBuf {
    let base = Path::new(env!("CARGO_MANIFEST_DIR"));
    base.join("../../schemas").join(name)
}
