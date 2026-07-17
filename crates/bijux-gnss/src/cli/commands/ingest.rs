use super::*;

pub(crate) fn validate_config_ingest(profile: &ReceiverConfig) -> Result<()> {
    let report = <ReceiverConfig as ValidateConfig>::validate(profile);
    if report.errors.is_empty() {
        return Ok(());
    }
    let messages: Vec<String> = report.errors.into_iter().map(|e| e.message).collect();
    bail!("invalid config: {}", messages.join(", "));
}

pub(crate) fn handle_track(command: GnssCommand) -> Result<()> {
    let GnssCommand::Track {
        common,
        input,
        sampling,
        intermediate_frequency,
        code_replica,
        acquisition_search,
        window: _window,
        prns,
    } = command
    else {
        bail!("invalid command for handler");
    };
    let RawCaptureInputArgs { file } = input;
    let SamplingRateOverrideArgs { sampling_hz } = sampling;
    let IntermediateFrequencyArgs { if_hz } = intermediate_frequency;
    let CodeReplicaArgs { code_hz, code_length } = code_replica;
    let AcquisitionSearchArgs { doppler_search_hz, doppler_step_hz } = acquisition_search;
    let DefaultPrnSelectionArgs { prn } = prns;

    let runtime = runtime_config_from_env(&common, None);
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
    validate_config_ingest(&profile)?;
    let config = profile.to_pipeline_config();

    let input_file = resolve_input_file(file.as_ref(), dataset.as_ref())?;
    let frame = load_tracking_frame(&input_file, &config, &raw_iq_metadata)?;
    let signal_quality = measure_signal_quality_from_samples(&raw_iq_metadata, &frame.iq);

    let sats = bijux_gnss_infra::api::core::prns_to_sats(&prn);
    let receiver = Receiver::new(config.clone(), runtime.clone());
    let mut source = open_tracking_window_source(&input_file, &config, &raw_iq_metadata)?;
    let artifacts = receiver.run_with_satellites(&mut source, &sats)?;
    let tracks = artifacts.tracking.clone();

    let report = TrackingReport {
        sats: sats.clone(),
        doppler_search: doppler_search_settings(&profile),
        front_end_metrics: signal_quality.front_end_metrics.clone(),
        signal_quality,
        epochs: tracks
            .iter()
            .flat_map(|t| {
                t.epochs.iter().map(move |e| TrackingRow {
                    epoch_idx: e.epoch.index,
                    sample_index: e.sample_index,
                    sat: t.sat,
                    carrier_hz: e.carrier_hz.0,
                    carrier_phase_cycles: e.carrier_phase_cycles.0,
                    code_rate_hz: e.code_rate_hz.0,
                    code_phase_samples: e.code_phase_samples.0,
                    prompt_i: e.prompt_i,
                    prompt_q: e.prompt_q,
                    early_i: e.early_i,
                    early_q: e.early_q,
                    late_i: e.late_i,
                    late_q: e.late_q,
                    lock: e.lock,
                    cn0_dbhz: e.cn0_dbhz,
                    pll_lock: e.pll_lock,
                    dll_lock: e.dll_lock,
                    fll_lock: e.fll_lock,
                    cycle_slip: e.cycle_slip,
                    anti_false_lock: e.anti_false_lock,
                    nav_bit_lock: e.nav_bit_lock,
                    navigation_bit_sign: e.navigation_bit_sign,
                    cycle_slip_reason: e.cycle_slip_reason.clone(),
                    lock_state: e.lock_state.clone(),
                    lock_state_reason: e.lock_state_reason.clone(),
                    dll_err: e.dll_err,
                    pll_err: e.pll_err,
                    fll_err: e.fll_err,
                })
            })
            .collect(),
    };

    emit_report(&common, "track", &report)?;
    write_signal_quality_report(&common, "track", &report.signal_quality)?;
    write_track_timeseries(&common, &report, &profile, dataset.as_ref())?;
    write_observation_artifacts_for_command(
        &common,
        "track",
        &artifacts.observation_artifacts(),
        &profile,
        dataset.as_ref(),
    )?;
    write_tracking_timing_for_command(&common, "track", &tracks, dataset.as_ref())?;
    write_manifest(&common, "track", &profile, dataset.as_ref(), &report)?;

    Ok(())
}

pub(crate) fn handle_inspect(command: GnssCommand) -> Result<()> {
    let GnssCommand::Inspect {
        common,
        input,
        sampling,
        max_samples,
    } = command
    else {
        bail!("invalid command for handler");
    };
    let RawCaptureInputArgs { file } = input;
    let SamplingRateOverrideArgs { sampling_hz } = sampling;

    let _ = runtime_config_from_env(&common, None);
                    let dataset = load_dataset(&common)?;
                    let input_file = resolve_input_file(file.as_ref(), dataset.as_ref())?;
                    let raw_iq_metadata = resolve_raw_iq_metadata(&common, dataset.as_ref())?;
                    let mut profile = ReceiverConfig::default();
                    apply_raw_iq_metadata(&mut profile, &raw_iq_metadata, sampling_hz, None)?;
                    let report = inspect_dataset(&input_file, &raw_iq_metadata, max_samples)?;
                    match common.report {
                        ReportFormat::Table => print_inspect_table(&report),
                        ReportFormat::Json => emit_report(&common, "inspect", &report)?,
                    }
                    write_signal_quality_report(&common, "inspect", &report.signal_quality)?;
                    write_manifest(
                        &common,
                        "inspect",
                        &profile,
                        dataset.as_ref(),
                        &report,
                    )?;

    Ok(())
}

pub(crate) fn handle_validate_config(command: GnssCommand) -> Result<()> {
    let GnssCommand::ValidateConfig { args } = command else {
        bail!("invalid command for handler");
    };
    let ValidateConfigArgs { common } = args;

    let _ = runtime_config_from_env(&common, None);
    let path = common.config.clone().context("--config is required")?;
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

pub(crate) fn handle_config(command: GnssCommand) -> Result<()> {
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
pub(crate) fn handle_config_schema(command: GnssCommand) -> Result<()> {
    let GnssCommand::ConfigSchema { args } = command else {
        bail!("invalid command for handler");
    };
    let ConfigSchemaArgs { common, out } = args;

    let _ = runtime_config_from_env(&common, None);
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
pub(crate) fn handle_config_schema(command: GnssCommand) -> Result<()> {
    let GnssCommand::ConfigSchema { args: _ } = command else {
        bail!("invalid command for handler");
    };
    bail!("schema generation disabled; enable --features schema-validate");
}

pub(crate) fn handle_config_upgrade(command: GnssCommand) -> Result<()> {
    let GnssCommand::ConfigUpgrade { args } = command else {
        bail!("invalid command for handler");
    };
    let ConfigUpgradeArgs { common, config, out } = args;

    let _ = runtime_config_from_env(&common, None);
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

pub(crate) fn handle_rinex(command: GnssCommand) -> Result<()> {
    let GnssCommand::Rinex {
                common,
                obs,
                eph,
                strict,
            } = command else {
        bail!("invalid command for handler");
    };

    let _ = runtime_config_from_env(&common, None);
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
