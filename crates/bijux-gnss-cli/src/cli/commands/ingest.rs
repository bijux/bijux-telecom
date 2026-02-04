fn validate_config_ingest(profile: &ReceiverConfig) -> Result<()> {
    let report = <ReceiverConfig as ValidateConfig>::validate(profile);
    if report.errors.is_empty() {
        return Ok(());
    }
    let messages: Vec<String> = report.errors.into_iter().map(|e| e.message).collect();
    bail!("invalid config: {}", messages.join(", "));
}

fn handle_track(command: GnssCommand) -> Result<()> {
    let GnssCommand::Track {
                common,
                file,
                sampling_hz,
                if_hz,
                code_hz,
                code_length,
                doppler_search_hz,
                doppler_step_hz,
                offset_bytes,
                prn,
            } = command else {
        bail!("invalid command for handler");
    };

    set_trace_dir(&common);
                    let dataset = load_dataset(&common)?;
                    let mut profile = load_config(&common)?;
                    apply_common_overrides(
                        &mut profile,
                        CommonOverrides {
                            seed: common.seed,
                            deterministic: common.deterministic,
                        },
                    );
                    apply_overrides(&mut profile, sampling_hz, if_hz, code_hz, code_length);
                    if let Some(entry) = &dataset {
                        profile.sample_rate_hz = entry.sample_rate_hz;
                        profile.intermediate_freq_hz = entry.intermediate_freq_hz;
                    }
                    validate_config_ingest(&profile)?;
                    let config = profile.to_runtime_config();
    
                    let input_file = resolve_input_file(file.as_ref(), dataset.as_ref())?;
                    let sidecar = load_sidecar(common.sidecar.as_ref())?;
                    let frame = load_frame(&input_file, &config, offset_bytes, sidecar.as_ref())?;
    
                    let acquisition = AcquisitionEngine::new(config.clone())
                        .with_doppler(doppler_search_hz, doppler_step_hz);
                    let sats = bijux_gnss_infra::api::core::prns_to_sats(&prn);
                    let acquisitions = acquisition.run_fft(&frame, &sats);
    
                    let tracking = bijux_gnss_infra::api::receiver::TrackingEngine::new(config.clone());
                    let tracks = tracking.track_from_acquisition(
                        &frame,
                        &acquisitions,
                        bijux_gnss_infra::api::core::SignalBand::L1,
                    );
    
                    let report = TrackingReport {
                        sats: sats.clone(),
                        epochs: tracks
                            .iter()
                            .flat_map(|t| {
                                t.epochs.iter().map(move |e| TrackingRow {
                                    epoch_idx: e.epoch.index,
                                    sample_index: e.sample_index,
                                    sat: t.sat,
                                    carrier_hz: t.carrier_hz,
                                    code_rate_hz: e.code_rate_hz.0,
                                    code_phase_samples: t.code_phase_samples,
                                    prompt_i: e.prompt_i,
                                    prompt_q: e.prompt_q,
                                    lock: e.lock,
                                    cn0_dbhz: e.cn0_dbhz,
                                    pll_lock: e.pll_lock,
                                    dll_lock: e.dll_lock,
                                    fll_lock: e.fll_lock,
                                    cycle_slip: e.cycle_slip,
                                    nav_bit_lock: e.nav_bit_lock,
                                    dll_err: e.dll_err,
                                    pll_err: e.pll_err,
                                    fll_err: e.fll_err,
                                })
                            })
                            .collect(),
                    };
    
                    emit_report(&common, "track", &report)?;
                    write_track_timeseries(&common, &report, &profile, dataset.as_ref())?;
                    write_obs_timeseries(
                        &common,
                        &config,
                        &tracks,
                        profile.navigation.hatch_window,
                        &profile,
                        dataset.as_ref(),
                    )?;
                    let timing_path = artifacts_dir(&common, "track", dataset.as_ref())?
                        .join("timing.jsonl");
                    let mut timing_lines = Vec::new();
                    for track in &tracks {
                        for epoch in &track.epochs {
                            if let Some(ms) = epoch.processing_ms {
                                timing_lines.push(serde_json::to_string(&serde_json::json!({
                                    "epoch_idx": epoch.epoch.index,
                                    "stage": "tracking",
                                    "processing_ms": ms
                                }))?);
                            }
                        }
                    }
                    fs::write(&timing_path, timing_lines.join("\n"))?;
                    write_manifest(&common, "track", &profile, dataset.as_ref(), &report)?;

    Ok(())
}

fn handle_inspect(command: GnssCommand) -> Result<()> {
    let GnssCommand::Inspect {
                common,
                file,
                sampling_hz,
                max_samples,
            } = command else {
        bail!("invalid command for handler");
    };

    set_trace_dir(&common);
                    let dataset = load_dataset(&common)?;
                    let input_file = resolve_input_file(file.as_ref(), dataset.as_ref())?;
                    let sample_rate_hz = sampling_hz
                        .or_else(|| dataset.as_ref().map(|d| d.sample_rate_hz))
                        .unwrap_or(5_000_000.0);
                    let report = inspect_dataset(&input_file, sample_rate_hz, max_samples)?;
                    match common.report {
                        ReportFormat::Table => print_inspect_table(&report),
                        ReportFormat::Json => emit_report(&common, "inspect", &report)?,
                    }
                    write_manifest(
                        &common,
                        "inspect",
                        &ReceiverConfig::default(),
                        dataset.as_ref(),
                        &report,
                    )?;

    Ok(())
}

fn handle_validateconfig(command: GnssCommand) -> Result<()> {
    let GnssCommand::ValidateConfig { common, config } = command else {
        bail!("invalid command for handler");
    };

    set_trace_dir(&common);
                    let path = config
                        .or(common.config.clone())
                        .context("--config is required")?;
                    let profile = load_config_from_path(&path)?;
                    validate_config_schema(&profile)?;
                    let report = <ReceiverConfig as ValidateConfig>::validate(&profile);
                    if report.errors.is_empty() {
                        println!("config valid: {}", path.display());
                    } else {
                        bail!(
                            "config invalid: {}",
                            report
                                .errors
                                .iter()
                                .map(|e| e.message.as_str())
                                .collect::<Vec<_>>()
                                .join(", ")
                        );
                    }
                    let dataset = load_dataset(&common)?;
                    let summary = serde_json::json!({ "config": path.display().to_string() });
                    write_manifest(&common, "validate_config", &profile, dataset.as_ref(), &summary)?;

    Ok(())
}

fn handle_config(command: GnssCommand) -> Result<()> {
    let GnssCommand::Config { command } = command else {
        bail!("invalid command for handler");
    };

    match command {
        ConfigCommand::Validate {
            common,
            file,
            strict,
        } => {
            let contents = fs::read_to_string(&file)?;
            let profile: ReceiverConfig = toml::from_str(&contents)?;
            let report = <ReceiverConfig as ValidateConfig>::validate(&profile);
            if !report.errors.is_empty() {
                for err in report.errors {
                    eprintln!("error: {}", err.message);
                }
                bail!("config validation failed");
            }
            if strict && !report.warnings.is_empty() {
                for warn in report.warnings {
                    eprintln!("warning: {warn}");
                }
                bail!("config validation warnings (strict)");
            }
            for warn in report.warnings {
                eprintln!("warning: {warn}");
            }
            let summary = serde_json::json!({ "config": file.display().to_string() });
            write_manifest(
                &common,
                "config_validate",
                &ReceiverConfig::default(),
                None,
                &summary,
            )?;
        }
        ConfigCommand::PrintDefaults { common, out } => {
            let profile = ReceiverConfig::default();
            let toml = toml::to_string_pretty(&profile)?;
            if let Some(out) = out {
                fs::write(&out, toml)?;
                println!("wrote {}", out.display());
            } else {
                println!("{toml}");
            }
            let summary = serde_json::json!({ "defaults": "receiver_profile" });
            write_manifest(
                &common,
                "config_print_defaults",
                &ReceiverConfig::default(),
                None,
                &summary,
            )?;
        }
    }

    Ok(())
}

#[cfg(feature = "schema-validate")]
use schemars::schema_for;

#[cfg(feature = "schema-validate")]
fn handle_configschema(command: GnssCommand) -> Result<()> {
    let GnssCommand::ConfigSchema { common, out } = command else {
        bail!("invalid command for handler");
    };

    set_trace_dir(&common);
                        let schema = schema_for!(ReceiverConfig);
                        fs::write(&out, serde_json::to_string_pretty(&schema)?)?;
                        println!("wrote {}", out.display());
                        let summary = serde_json::json!({ "schema": out.display().to_string() });
                        write_manifest(
                            &common,
                            "config_schema",
                            &ReceiverConfig::default(),
                            None,
                            &summary,
                        )?;

    Ok(())
}

#[cfg(not(feature = "schema-validate"))]
fn handle_configschema(command: GnssCommand) -> Result<()> {
    let GnssCommand::ConfigSchema { common: _, out: _ } = command else {
        bail!("invalid command for handler");
    };
    bail!("schema generation disabled; enable --features schema-validate");
}

fn handle_configupgrade(command: GnssCommand) -> Result<()> {
    let GnssCommand::ConfigUpgrade { common, config, out } = command else {
        bail!("invalid command for handler");
    };

    set_trace_dir(&common);
    let profile = load_config_from_path(&config)?;
    validate_config_schema(&profile)?;
    let report = <ReceiverConfig as ValidateConfig>::validate(&profile);
    if !report.errors.is_empty() {
        bail!(
            "config invalid: {}",
            report
                .errors
                .iter()
                .map(|e| e.message.as_str())
                .collect::<Vec<_>>()
                .join(", ")
        );
    }
    let output_path = out.unwrap_or(config);
    let data = toml::to_string_pretty(&profile)?;
    fs::write(&output_path, data)?;
    println!("wrote {}", output_path.display());
    let summary = serde_json::json!({ "output": output_path.display().to_string() });
    write_manifest(&common, "config_upgrade", &profile, None, &summary)?;
    Ok(())
}

fn handle_rinex(command: GnssCommand) -> Result<()> {
    let GnssCommand::Rinex {
                common,
                obs,
                eph,
                strict,
            } = command else {
        bail!("invalid command for handler");
    };

    set_trace_dir(&common);
                    let obs_epochs = read_obs_epochs(&obs)?;
                    let ephs = read_ephemeris(&eph)?;
                    let out_dir = artifacts_dir(&common, "rinex", None)?;
                    let obs_path = out_dir.join("obs.rnx");
                    let nav_path = out_dir.join("nav.rnx");
                    write_rinex_obs(&obs_path, &obs_epochs, strict)?;
                    write_rinex_nav(&nav_path, &ephs, strict)?;
                    println!("wrote {}", obs_path.display());
                    println!("wrote {}", nav_path.display());
                    let summary = serde_json::json!({
                        "obs": obs_path.display().to_string(),
                        "nav": nav_path.display().to_string()
                    });
                    write_manifest(
                        &common,
                        "rinex",
                        &ReceiverConfig::default(),
                        None,
                        &summary,
                    )?;

    Ok(())
}
