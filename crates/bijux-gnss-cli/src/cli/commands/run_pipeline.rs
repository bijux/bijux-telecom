fn validate_config(profile: &ReceiverConfig) -> Result<()> {
    let report = <ReceiverConfig as ValidateConfig>::validate(profile);
    if report.errors.is_empty() {
        return Ok(());
    }
    let messages: Vec<String> = report.errors.into_iter().map(|e| e.message).collect();
    bail!("invalid config: {}", messages.join(", "));
}

fn handle_acquire(command: GnssCommand) -> Result<()> {
    let GnssCommand::Acquire {
        common,
        file,
        sampling_hz,
        if_hz,
        code_hz,
        code_length,
        doppler_search_hz,
        doppler_step_hz,
        offset_bytes: _offset_bytes,
        top,
        prn,
    } = command
    else {
        bail!("invalid command for handler");
    };

    let dataset = load_dataset(&common)?;
    let mut profile = load_config(&common)?;
    apply_common_overrides(
        &mut profile,
        CommonOverrides { seed: common.seed, deterministic: common.deterministic },
    );
    apply_acquisition_doppler_overrides(&mut profile, doppler_search_hz, doppler_step_hz);
    let raw_iq_metadata = resolve_raw_iq_metadata(&common, dataset.as_ref())?;
    apply_raw_iq_metadata(&mut profile, &raw_iq_metadata, sampling_hz, if_hz)?;
    apply_overrides(&mut profile, None, None, code_hz, code_length);
    validate_config(&profile)?;
    let config = profile.to_pipeline_config();

    let input_file = resolve_input_file(file.as_ref(), dataset.as_ref())?;

    let frame = load_acquisition_frame(&input_file, &config, &raw_iq_metadata)?;
    let signal_quality = measure_signal_quality_from_samples(&raw_iq_metadata, &frame.iq);
    let runtime = runtime_config_from_env(&common, None);
    let acquisition = AcquisitionEngine::new(config, runtime);
    let sats = bijux_gnss_infra::api::core::prns_to_sats(&prn);
    let results = acquisition.run_fft_topn(
        &frame,
        &sats,
        top,
        profile.acquisition.integration_ms,
        profile.acquisition.noncoherent_integration,
    );

    let primary_results =
        results.iter().filter_map(|candidates| candidates.first().cloned()).collect::<Vec<_>>();
    let primary_rows = primary_results.iter().map(acquisition_row_from_result).collect::<Vec<_>>();
    let mut rows = Vec::new();
    for candidates in &results {
        for r in candidates {
            rows.push(acquisition_row_from_result(r));
        }
    }

    let report = AcquisitionReport {
        sats,
        search_summary: bijux_gnss_infra::api::core::AcqSearchSummary::from_results(
            &primary_results,
        ),
        doppler_search: doppler_search_settings(&profile),
        code_phase_search: code_phase_search_settings_from_results(&profile, &results),
        front_end_metrics: signal_quality.front_end_metrics.clone(),
        signal_quality,
        reported_prns: summarize_reported_prns(&rows),
        primary_results: primary_rows,
        results: rows,
    };
    match common.report {
        ReportFormat::Table => print_acquisition_table(&report),
        ReportFormat::Json => emit_report(&common, "acquire", &report)?,
    }
    write_signal_quality_report(&common, "acquire", &report.signal_quality)?;
    let (layout, header) =
        prepare_run(&infra_args(&common), "acquire", &profile, dataset.as_ref())?;
    let out_dir = layout.artifacts_dir;
    let acq_path = out_dir.join("acq.jsonl");
    let mut acq_lines = Vec::new();
    for candidates in &results {
        for result in candidates {
            let wrapped = AcqResultV1 { header: header.clone(), payload: result.clone() };
            acq_lines.push(serde_json::to_string(&wrapped)?);
        }
    }
    fs::write(&acq_path, acq_lines.join("\n"))?;
    validate_jsonl_schema(&schema_path("acq_result_v1.schema.json"), &acq_path, false)?;
    write_manifest(&common, "acquire", &profile, dataset.as_ref(), &report)?;

    Ok(())
}

fn acquisition_row_from_result(result: &bijux_gnss_infra::api::core::AcqResult) -> AcquisitionRow {
    let (coarse_carrier_hz, doppler_refinement_hz, doppler_refinement_bins) = result
        .doppler_refinement
        .as_ref()
        .map(|refinement| {
            (
                Some(refinement.coarse_carrier_hz.0),
                Some(refinement.offset_hz),
                Some(refinement.offset_bins),
            )
        })
        .unwrap_or((None, None, None));
    let (refined_code_phase_samples, code_phase_refinement_samples) = result
        .code_phase_refinement
        .as_ref()
        .map(|refinement| {
            (Some(refinement.refined_code_phase_samples), Some(refinement.offset_samples))
        })
        .unwrap_or((None, None));
    let (doppler_uncertainty_hz, code_phase_uncertainty_samples) = result
        .uncertainty
        .as_ref()
        .map(|uncertainty| (Some(uncertainty.doppler_hz), Some(uncertainty.code_phase_samples)))
        .unwrap_or((None, None));
    AcquisitionRow {
        sat: result.sat,
        signal_band: result.signal_band,
        source_sample_index: result.source_time.sample_index,
        doppler_hz: result.doppler_hz.0,
        candidate_rank: result.candidate_rank,
        is_primary_candidate: result.is_primary_candidate,
        carrier_hz: result.carrier_hz.0,
        coarse_carrier_hz,
        doppler_refinement_hz,
        doppler_refinement_bins,
        doppler_uncertainty_hz,
        code_phase_samples: result.code_phase_samples,
        refined_code_phase_samples,
        code_phase_refinement_samples,
        code_phase_uncertainty_samples,
        peak: result.peak,
        peak_mean_ratio: result.peak_mean_ratio,
        peak_second_ratio: result.peak_second_ratio,
        hypothesis: result.hypothesis.to_string(),
        selection_reason: result.explain_selection_reason.clone(),
    }
}

fn handle_pvt(command: GnssCommand) -> Result<()> {
    let GnssCommand::Pvt { common, obs, eph, ekf } = command else {
        bail!("invalid command for handler");
    };

    let runtime = runtime_config_from_env(&common, None);
    let profile = load_config(&common)?;
    let dataset = load_dataset(&common)?;
    let obs_epochs = read_obs_epochs(&obs)?;
    let ephs = read_ephemeris(&eph)?;
    let mut lines = Vec::new();
    let mut solutions = Vec::new();
    let mut timing_lines = Vec::new();
    let header = artifact_header(&common, &profile, dataset.as_ref())?;
    let mut nav = if !ekf {
        Some(bijux_gnss_infra::api::receiver::Navigation::new(
            ReceiverConfig::default().to_pipeline_config(),
            runtime.clone(),
        ))
    } else {
        None
    };
    let mut ekf_ctx = if ekf { Some(EkfContext::new()) } else { None };
    for obs_epoch in obs_epochs {
        let solution = if ekf {
            solve_epoch_ekf(&mut ekf_ctx, &obs_epoch, &ephs)?
        } else {
            nav.as_mut().and_then(|nav| nav.solve_epoch(&obs_epoch, &ephs))
        };
        if let Some(solution) = solution {
            solutions.push(solution.clone());
            if let Some(ms) = solution.processing_ms {
                timing_lines.push(serde_json::to_string(&serde_json::json!({
                    "epoch_idx": solution.epoch.index,
                    "stage": "nav",
                    "processing_ms": ms
                }))?);
            }
            let wrapped = NavSolutionEpochV1 { header: header.clone(), payload: solution };
            let line = serde_json::to_string(&wrapped)?;
            lines.push(line);
        }
    }
    let out_dir = artifacts_dir(&common, "pvt", dataset.as_ref())?;
    let path = out_dir.join("pvt.jsonl");
    fs::write(&path, lines.join("\n"))?;
    validate_jsonl_schema(&schema_path("nav_solution_epoch_v1.schema.json"), &path, false)?;
    write_nav_solution_outputs(&out_dir, &solutions)?;
    if !timing_lines.is_empty() {
        let timing_path = out_dir.join("timing_nav.jsonl");
        fs::write(&timing_path, timing_lines.join("\n"))?;
    }
    let report = serde_json::json!({
        "epochs": lines.len(),
        "ekf": ekf
    });
    write_manifest(&common, "pvt", &profile, dataset.as_ref(), &report)?;

    Ok(())
}

fn handle_experiment(command: GnssCommand) -> Result<()> {
    let GnssCommand::Experiment { common, scenario, sweep } = command else {
        bail!("invalid command for handler");
    };

    let runtime = runtime_config_from_env(&common, None);
    let mut profile = load_config(&common)?;
    apply_common_overrides(
        &mut profile,
        CommonOverrides { seed: common.seed, deterministic: common.deterministic },
    );
    profile.validate().map_err(|errs| eyre!("invalid config before sweep: {}", errs.join(", ")))?;

    let scenario_contents = fs::read_to_string(&scenario)
        .with_context(|| format!("failed to read scenario {}", scenario.display()))?;
    let scenario_def: bijux_gnss_infra::api::receiver::sim::SyntheticScenario =
        toml::from_str(&scenario_contents)?;

    let sweep_spec = parse_sweep(&sweep)?;
    let runs = expand_sweep(&sweep_spec);
    let out_dir = artifacts_dir(&common, "experiment", None)?;

    let mut results = Vec::new();
    for (idx, overrides) in runs.iter().enumerate() {
        let mut run_profile = profile.clone();
        for (key, value) in overrides {
            apply_sweep_value(&mut run_profile, key, value)?;
        }
        run_profile
            .validate()
            .map_err(|errs| eyre!("invalid config in sweep: {}", errs.join(", ")))?;

        let config = run_profile.to_pipeline_config();
        let start = std::time::Instant::now();
        let frame =
            bijux_gnss_infra::api::receiver::sim::generate_l1_ca_multi(&config, &scenario_def);
        let sats: Vec<SatId> = scenario_def.satellites.iter().map(|s| s.sat).collect();
        let acquisition = AcquisitionEngine::new(config.clone(), runtime.clone());
        let acquisitions = acquisition.run_fft(&frame, &sats);
        let tracking =
            bijux_gnss_infra::api::receiver::TrackingEngine::new(config.clone(), runtime.clone());
        let tracks = tracking.track_from_acquisition(&frame, &acquisitions);
        let obs_report = bijux_gnss_infra::api::receiver::observations_from_tracking_results(
            &config,
            &tracks,
            run_profile.navigation.hatch_window,
        );
        let obs = obs_report.output;
        for event in obs_report.events {
            runtime.logger.event(&event);
        }
        let mut nav =
            bijux_gnss_infra::api::receiver::Navigation::new(config.clone(), runtime.clone());
        let mut solutions = Vec::new();
        for epoch in &obs {
            if let Some(solution) = nav.solve_epoch(epoch, &scenario_def.ephemerides) {
                solutions.push(solution);
            }
        }
        let elapsed = start.elapsed().as_secs_f64();
        let epochs = obs.len().max(1) as f64;
        let ms_per_epoch = (elapsed * 1000.0) / epochs;

        let lock_total: usize = tracks.iter().map(|t| t.epochs.len()).sum();
        let lock_good: usize =
            tracks.iter().map(|t| t.epochs.iter().filter(|e| e.lock).count()).sum();
        let lock_pct = if lock_total > 0 { lock_good as f64 / lock_total as f64 } else { 0.0 };

        let mean_pvt_rms = if solutions.is_empty() {
            0.0
        } else {
            solutions.iter().map(|s| s.rms_m.0).sum::<f64>() / solutions.len() as f64
        };
        let mean_residual_rms = mean_pvt_rms;
        let rejected_count =
            solutions.iter().flat_map(|s| s.residuals.iter().filter(|r| r.rejected)).count();

        let config_hash = hash_config(common.config.as_ref(), &run_profile)?;
        let result = ExperimentRunResult {
            run_index: idx,
            config_hash,
            scenario_id: scenario_def.id.clone(),
            overrides: overrides.clone(),
            lock_pct,
            pvt_rms_m: mean_pvt_rms,
            residual_rms_m: mean_residual_rms,
            rejected_count,
            ms_per_epoch,
        };
        write_experiment_run(&out_dir, idx, &result, &obs, &solutions)?;
        results.push(result);
    }

    let summary = ExperimentSummary { runs: results.clone() };
    let summary_path = out_dir.join("experiment_summary.json");
    fs::write(&summary_path, serde_json::to_string_pretty(&summary)?)?;
    let schema_path = schema_path("experiment_summary.schema.json");
    if schema_path.exists() {
        validate_json_schema(&schema_path, &summary_path, false)?;
    }
    write_manifest(&common, "experiment", &profile, None, &summary)?;

    Ok(())
}

fn handle_validatesidecar(command: GnssCommand) -> Result<()> {
    let GnssCommand::ValidateSidecar { common, sidecar_file } = command else {
        bail!("invalid command for handler");
    };

    let _ = runtime_config_from_env(&common, None);
    let spec = bijux_gnss_infra::api::load_raw_iq_metadata(&sidecar_file)?;
    validate_sidecar_schema(&spec)?;
    println!("sidecar ok: {}", sidecar_file.display());
    let dataset = load_dataset(&common)?;
    let mut manifest_common = common.clone();
    manifest_common.sidecar = Some(sidecar_file.clone());
    let mut manifest_profile = ReceiverConfig::default();
    apply_raw_iq_metadata(&mut manifest_profile, &spec, None, None)?;
    let summary = serde_json::json!({ "sidecar": sidecar_file.display().to_string() });
    write_manifest(
        &manifest_common,
        "validate_sidecar",
        &manifest_profile,
        dataset.as_ref(),
        &summary,
    )?;

    Ok(())
}

fn handle_run(command: GnssCommand) -> Result<()> {
    let GnssCommand::Run { common, file, replay, rate } = command else {
        bail!("invalid command for handler");
    };

    let dataset = load_dataset(&common)?;
    let runtime = runtime_config_from_capture_start(
        &common,
        None,
        dataset.as_ref().and_then(|entry| entry.capture_start_utc.as_deref()),
    );
    let mut profile = load_config(&common)?;
    apply_common_overrides(
        &mut profile,
        CommonOverrides { seed: common.seed, deterministic: common.deterministic },
    );
    let raw_iq_metadata = resolve_raw_iq_metadata(&common, dataset.as_ref())?;
    apply_raw_iq_metadata(&mut profile, &raw_iq_metadata, None, None)?;
    validate_config(&profile)?;
    let config = profile.to_pipeline_config();
    let input_file = resolve_input_file(file.as_ref(), dataset.as_ref())?;
    let signal_quality = measure_signal_quality_from_raw_iq(&input_file, &raw_iq_metadata, 0)?;
    let receiver = Receiver::new(config.clone(), runtime);
    let mut source = FileSamples::open_raw_iq(&input_file, raw_iq_metadata.clone())?;
    let artifacts = receiver.run(&mut source)?;
    if replay && rate > 0.0 {
        let dt = (artifacts.processed_input_epochs as f64 * 0.001) / rate;
        std::thread::sleep(std::time::Duration::from_secs_f64(dt));
    }
    let report = StreamingRunReport {
        epochs: artifacts.processed_input_epochs,
        processed_input_samples: artifacts.processed_input_samples,
        acquisitions: artifacts.acquisitions.len(),
        tracked_channels: artifacts
            .tracking
            .iter()
            .filter(|track| !track.epochs.is_empty())
            .count(),
        observation_epochs: artifacts.observations.len(),
        front_end_metrics: signal_quality.front_end_metrics.clone(),
        signal_quality,
    };
    emit_report(&common, "run", &report)?;
    write_signal_quality_report(&common, "run", &report.signal_quality)?;
    write_manifest(&common, "run", &profile, dataset.as_ref(), &report)?;

    Ok(())
}
