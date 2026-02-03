fn validate_profile(profile: &ReceiverProfile) -> Result<()> {
    let report = <ReceiverProfile as ValidateConfig>::validate(profile);
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
                offset_bytes,
                top,
                prn,
            } = command else {
        bail!("invalid command for handler");
    };

    set_trace_dir(&common);
                    let dataset = load_dataset(&common)?;
                    let mut profile = load_profile(&common)?;
                    apply_common_overrides(&mut profile, &common);
                    apply_overrides(&mut profile, sampling_hz, if_hz, code_hz, code_length);
                    if let Some(entry) = &dataset {
                        profile.sample_rate_hz = entry.sample_rate_hz;
                        profile.intermediate_freq_hz = entry.intermediate_freq_hz;
                    }
                    validate_profile(&profile)?;
                    let config = profile.to_receiver_config();
    
                    let input_file = resolve_input_file(file.as_ref(), dataset.as_ref())?;
    
                    let sidecar = load_sidecar(common.sidecar.as_ref())?;
                    let frame = load_frame(&input_file, &config, offset_bytes, sidecar.as_ref())?;
    
                    let acquisition =
                        Acquisition::new(config).with_doppler(doppler_search_hz, doppler_step_hz);
                    let sats = prns_to_sats(&prn);
                    let results = acquisition.run_fft_topn(
                        &frame,
                        &sats,
                        top,
                        profile.acquisition.integration_ms,
                        1,
                    );
    
                    let mut rows = Vec::new();
                    for candidates in &results {
                        for r in candidates {
                            rows.push(AcquisitionRow {
                                sat: r.sat,
                                carrier_hz: r.carrier_hz,
                                code_phase_samples: r.code_phase_samples,
                                peak: r.peak,
                                peak_mean_ratio: r.peak_mean_ratio,
                                peak_second_ratio: r.peak_second_ratio,
                            });
                        }
                    }
    
                    let report = AcquisitionReport {
                        sats,
                        results: rows,
                    };
                    match common.report {
                        ReportFormat::Table => print_acquisition_table(&report),
                        ReportFormat::Json => emit_report(&common, "acquire", &report)?,
                    }
                    let header = artifact_header(&common, &profile, dataset.as_ref())?;
                    let out_dir = artifacts_dir(&common, "acquire", dataset.as_ref())?;
                    let acq_path = out_dir.join("acq.jsonl");
                    let mut acq_lines = Vec::new();
                    for candidates in &results {
                        for result in candidates {
                            let wrapped = AcqResultV1 {
                                header: header.clone(),
                                result: result.clone(),
                            };
                            acq_lines.push(serde_json::to_string(&wrapped)?);
                        }
                    }
                    fs::write(acq_path, acq_lines.join("\n"))?;
                    write_manifest(&common, "acquire", &profile, dataset.as_ref(), &report)?;

    Ok(())
}

fn handle_pvt(command: GnssCommand) -> Result<()> {
    let GnssCommand::Pvt {
                common,
                obs,
                eph,
                ekf,
            } = command else {
        bail!("invalid command for handler");
    };

    set_trace_dir(&common);
                    let profile = load_profile(&common)?;
                    let dataset = load_dataset(&common)?;
                    let obs_epochs = read_obs_epochs(&obs)?;
                    let ephs = read_ephemeris(&eph)?;
                    let mut lines = Vec::new();
                    let header = artifact_header(&common, &profile, dataset.as_ref())?;
                    let mut nav = if !ekf {
                        Some(bijux_gnss_receiver::navigation::Navigation::new(
                            ReceiverProfile::default().to_receiver_config(),
                        ))
                    } else {
                        None
                    };
                    let mut ekf_ctx = if ekf { Some(EkfContext::new()) } else { None };
                    for obs_epoch in obs_epochs {
                        let solution = if ekf {
                            solve_epoch_ekf(&mut ekf_ctx, &obs_epoch, &ephs)?
                        } else {
                            nav.as_mut()
                                .and_then(|nav| nav.solve_epoch(&obs_epoch, &ephs))
                        };
                        if let Some(solution) = solution {
                            let wrapped = NavSolutionEpochV1 {
                                header: header.clone(),
                                epoch: solution,
                            };
                            let line = serde_json::to_string(&wrapped)?;
                            lines.push(line);
                        }
                    }
                    let out_dir = artifacts_dir(&common, "pvt", dataset.as_ref())?;
                    let path = out_dir.join("pvt.jsonl");
                    fs::write(path, lines.join("\n"))?;
                    let report = serde_json::json!({
                        "epochs": lines.len(),
                        "ekf": ekf
                    });
                    write_manifest(&common, "pvt", &profile, dataset.as_ref(), &report)?;

    Ok(())
}

fn handle_experiment(command: GnssCommand) -> Result<()> {
    let GnssCommand::Experiment {
                common,
                scenario,
                sweep,
            } = command else {
        bail!("invalid command for handler");
    };

    set_trace_dir(&common);
                    let mut profile = load_profile(&common)?;
                    apply_common_overrides(&mut profile, &common);
                    profile
                        .validate()
                        .map_err(|errs| eyre!("invalid config before sweep: {}", errs.join(", ")))?;
    
                    let scenario_contents = fs::read_to_string(&scenario)
                        .with_context(|| format!("failed to read scenario {}", scenario.display()))?;
                    let scenario_def: bijux_gnss_receiver::sim::SyntheticScenario =
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
    
                        let config = run_profile.to_receiver_config();
                        let start = std::time::Instant::now();
                        let frame = bijux_gnss_receiver::sim::generate_l1_ca_multi(
                            &config,
                            &scenario_def,
                        );
                        let sats: Vec<SatId> = scenario_def.satellites.iter().map(|s| s.sat).collect();
                        let acquisition = Acquisition::new(config.clone())
                            .with_doppler(10_000, run_profile.acquisition.doppler_step_hz);
                        let acquisitions = acquisition.run_fft(&frame, &sats);
                        let tracking = bijux_gnss_receiver::tracking::Tracking::new(config.clone());
                        let tracks = tracking.track_from_acquisition(
                            &frame,
                            &acquisitions,
                            bijux_gnss_core::SignalBand::L1,
                        );
                        let obs = bijux_gnss_receiver::observations::observations_from_tracking_results(
                            &config,
                            &tracks,
                            run_profile.navigation.hatch_window,
                        );
                        let mut nav = bijux_gnss_receiver::navigation::Navigation::new(config.clone());
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
                        let lock_good: usize = tracks
                            .iter()
                            .map(|t| t.epochs.iter().filter(|e| e.lock).count())
                            .sum();
                        let lock_pct = if lock_total > 0 {
                            lock_good as f64 / lock_total as f64
                        } else {
                            0.0
                        };
    
                        let mean_pvt_rms = if solutions.is_empty() {
                            0.0
                        } else {
                            solutions.iter().map(|s| s.rms_m).sum::<f64>() / solutions.len() as f64
                        };
                        let mean_residual_rms = mean_pvt_rms;
                        let rejected_count = solutions
                            .iter()
                            .flat_map(|s| s.residuals.iter().filter(|r| r.rejected))
                            .count();
    
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
    
                    let summary = ExperimentSummary {
                        runs: results.clone(),
                    };
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
    let GnssCommand::ValidateSidecar { common, sidecar } = command else {
        bail!("invalid command for handler");
    };

    set_trace_dir(&common);
                    let spec = load_sidecar(Some(&sidecar))?;
                    if spec.is_some() {
                        println!("sidecar ok: {}", sidecar.display());
                    }
                    let dataset = load_dataset(&common)?;
                    let summary = serde_json::json!({ "sidecar": sidecar.display().to_string() });
                    write_manifest(
                        &common,
                        "validate_sidecar",
                        &ReceiverProfile::default(),
                        dataset.as_ref(),
                        &summary,
                    )?;

    Ok(())
}

fn handle_run(command: GnssCommand) -> Result<()> {
    let GnssCommand::Run {
                common,
                file,
                replay,
                rate,
            } = command else {
        bail!("invalid command for handler");
    };

    set_trace_dir(&common);
                    let dataset = load_dataset(&common)?;
                    let mut profile = load_profile(&common)?;
                    apply_common_overrides(&mut profile, &common);
                    if let Some(entry) = &dataset {
                        profile.sample_rate_hz = entry.sample_rate_hz;
                        profile.intermediate_freq_hz = entry.intermediate_freq_hz;
                    }
                    validate_profile(&profile)?;
                    let config = profile.to_receiver_config();
                    let input_file = resolve_input_file(file.as_ref(), dataset.as_ref())?;
                    let sidecar = load_sidecar(common.sidecar.as_ref())?;
                    let mut source = FileSamples::open(
                        input_file.to_str().unwrap_or_default(),
                        sidecar.as_ref().map(|s| s.offset_bytes).unwrap_or(0),
                        sidecar
                            .as_ref()
                            .map(|s| s.sample_rate_hz)
                            .unwrap_or(config.sampling_freq_hz),
                    )?;
                    let samples_per_code = samples_per_code(
                        config.sampling_freq_hz,
                        config.code_freq_basis_hz,
                        config.code_length,
                    );
                    let mut epoch = 0u64;
                    while let Some(frame) = source.next_frame(samples_per_code)? {
                        let _ = frame;
                        epoch += 1;
                        if epoch.is_multiple_of(100) {
                            println!("processed epochs: {}", epoch);
                        }
                        if replay && rate > 0.0 {
                            let dt = 0.001 / rate;
                            std::thread::sleep(std::time::Duration::from_secs_f64(dt));
                        }
                    }
                    let report = serde_json::json!({ "epochs": epoch });
                    write_manifest(&common, "run", &profile, dataset.as_ref(), &report)?;

    Ok(())
}
