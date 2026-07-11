#[cfg(feature = "schema-validate")]
use jsonschema::validator_for;

#[derive(Copy, Clone)]
enum CsvType {
    U64,
    U8,
    F64,
    Bool,
    Str,
}

fn validate_csv_schema(path: &Path, expected: &str, types: &[CsvType]) -> Result<()> {
    let data = fs::read_to_string(path)?;
    let mut lines = data.lines();
    let header = lines.next().unwrap_or_default();
    if header.trim() != expected {
        bail!("csv schema mismatch in {}: expected {}, got {}", path.display(), expected, header);
    }
    for (idx, line) in lines.enumerate() {
        if line.trim().is_empty() {
            continue;
        }
        let cols: Vec<_> = line.split(',').collect();
        if cols.len() != types.len() {
            bail!(
                "csv schema mismatch in {} line {}: expected {} columns, got {}",
                path.display(),
                idx + 1,
                types.len(),
                cols.len()
            );
        }
        for (value, kind) in cols.iter().zip(types.iter()) {
            match kind {
                CsvType::U64 => {
                    value.parse::<u64>().map_err(|_| {
                        eyre!("csv parse error in {} line {}", path.display(), idx + 1)
                    })?;
                }
                CsvType::U8 => {
                    value.parse::<u8>().map_err(|_| {
                        eyre!("csv parse error in {} line {}", path.display(), idx + 1)
                    })?;
                }
                CsvType::F64 => {
                    value.parse::<f64>().map_err(|_| {
                        eyre!("csv parse error in {} line {}", path.display(), idx + 1)
                    })?;
                }
                CsvType::Bool => {
                    value.parse::<bool>().map_err(|_| {
                        eyre!("csv parse error in {} line {}", path.display(), idx + 1)
                    })?;
                }
                CsvType::Str => {
                    if value.trim().is_empty() {
                        bail!(
                            "csv parse error in {} line {}: empty string",
                            path.display(),
                            idx + 1
                        );
                    }
                }
            }
        }
    }
    Ok(())
}

fn validate_json_schema(schema_path: &Path, data_path: &Path, strict: bool) -> Result<()> {
    #[cfg(feature = "schema-validate")]
    {
        let schema_data = fs::read_to_string(schema_path)?;
        let schema_json: serde_json::Value = serde_json::from_str(&schema_data)?;
        let compiled = validator_for(&schema_json)
            .map_err(|e| eyre!("invalid schema {}: {}", schema_path.display(), e))?;
        let data = fs::read_to_string(data_path)?;
        let json: serde_json::Value = serde_json::from_str(&data)?;
        if strict {
            match &json {
                serde_json::Value::Array(items) if items.is_empty() => {
                    bail!("{} is empty", data_path.display());
                }
                _ => {}
            }
        }
        if !compiled.is_valid(&json) {
            let mut messages = Vec::new();
            for error in compiled.iter_errors(&json) {
                messages.push(error.to_string());
            }
            bail!("schema validation failed for {}: {}", data_path.display(), messages.join(", "));
        }
        Ok(())
    }
    #[cfg(not(feature = "schema-validate"))]
    {
        let _ = schema_path;
        if strict {
            bail!(
                "schema validation disabled; enable --features schema-validate to validate {}",
                data_path.display()
            );
        }
        Ok(())
    }
}

fn validate_jsonl_schema(schema_path: &Path, data_path: &Path, strict: bool) -> Result<()> {
    #[cfg(feature = "schema-validate")]
    {
        let schema_data = fs::read_to_string(schema_path)?;
        let schema_json: serde_json::Value = serde_json::from_str(&schema_data)?;
        let compiled = validator_for(&schema_json)
            .map_err(|e| eyre!("invalid schema {}: {}", schema_path.display(), e))?;
        let data = fs::read_to_string(data_path)?;
        let mut count = 0usize;
        for (idx, line) in data.lines().enumerate() {
            if line.trim().is_empty() {
                continue;
            }
            let json: serde_json::Value = serde_json::from_str(line)?;
            count += 1;
            let errors: Vec<String> = compiled.iter_errors(&json).map(|e| e.to_string()).collect();
            if !errors.is_empty() {
                bail!(
                    "schema validation failed for {} line {}: {}",
                    data_path.display(),
                    idx + 1,
                    errors.join(", ")
                );
            }
        }
        if strict && count == 0 {
            bail!("{} is empty", data_path.display());
        }
        Ok(())
    }
    #[cfg(not(feature = "schema-validate"))]
    {
        let _ = schema_path;
        if strict {
            bail!(
                "schema validation disabled; enable --features schema-validate to validate {}",
                data_path.display()
            );
        }
        Ok(())
    }
}

fn validate_config_schema(profile: &ReceiverConfig) -> Result<()> {
    #[cfg(feature = "schema-validate")]
    {
        let schema_path = schema_path("receiver_profile.schema.json");
        let schema_json: serde_json::Value = if schema_path.exists() {
            let file_json: serde_json::Value =
                serde_json::from_str(&fs::read_to_string(&schema_path)?)?;
            if file_json.get("properties").is_some() {
                file_json
            } else {
                serde_json::to_value(schemars::schema_for!(ReceiverConfig))?
            }
        } else {
            serde_json::to_value(schemars::schema_for!(ReceiverConfig))?
        };
        let compiled = validator_for(&schema_json).map_err(|e| eyre!("invalid schema: {}", e))?;
        let json = serde_json::to_value(profile)?;
        if !compiled.is_valid(&json) {
            let mut messages = Vec::new();
            for error in compiled.iter_errors(&json) {
                messages.push(error.to_string());
            }
            bail!("config schema validation failed: {}", messages.join(", "));
        }
        Ok(())
    }
    #[cfg(not(feature = "schema-validate"))]
    {
        let _ = profile;
        Ok(())
    }
}

fn validate_sidecar_schema(sidecar: &RawIqMetadata) -> Result<()> {
    #[cfg(feature = "schema-validate")]
    {
        let schema_path = schema_path("sidecar.schema.json");
        let schema_data = fs::read_to_string(schema_path)?;
        let schema_json: serde_json::Value = serde_json::from_str(&schema_data)?;
        let compiled =
            validator_for(&schema_json).map_err(|e| eyre!("invalid sidecar schema: {}", e))?;
        let json = serde_json::to_value(sidecar)?;
        if !compiled.is_valid(&json) {
            let mut messages = Vec::new();
            for error in compiled.iter_errors(&json) {
                messages.push(error.to_string());
            }
            bail!("sidecar schema validation failed: {}", messages.join(", "));
        }
        Ok(())
    }
    #[cfg(not(feature = "schema-validate"))]
    {
        let _ = sidecar;
        Ok(())
    }
}

fn handle_validateartifacts(command: GnssCommand) -> Result<()> {
    let GnssCommand::ValidateArtifacts { common, obs, eph, strict } = command else {
        bail!("invalid command for handler");
    };

    let _ = runtime_config_from_env(&common, None);
    if obs.is_none() && eph.is_none() {
        bail!("--obs and/or --eph is required");
    }
    let dataset = load_dataset(&common)?;
    let mut checked = Vec::new();
    if let Some(path) = obs {
        validate_jsonl_schema(&schema_path("obs_epoch_v1.schema.json"), &path, strict)?;
        println!("obs ok: {}", path.display());
        checked.push("obs".to_string());
    }
    if let Some(path) = eph {
        validate_json_schema(&schema_path("gps_ephemeris_v1.schema.json"), &path, strict)?;
        println!("ephemeris ok: {}", path.display());
        checked.push("ephemeris".to_string());
    }
    let report = serde_json::json!({ "checked": checked });
    write_manifest(
        &common,
        "validate_artifacts",
        &ReceiverConfig::default(),
        dataset.as_ref(),
        &report,
    )?;

    Ok(())
}

fn validation_science_policy(
    profile: &ReceiverConfig,
) -> bijux_gnss_infra::api::receiver::ValidationSciencePolicy {
    bijux_gnss_infra::api::receiver::ValidationSciencePolicy {
        min_mean_cn0_dbhz: profile.navigation.science_thresholds.min_mean_cn0_dbhz,
        max_pdop: profile.navigation.science_thresholds.max_pdop,
        max_gdop: profile.navigation.science_thresholds.max_gdop,
        max_residual_rms_m: profile.navigation.science_thresholds.max_residual_rms_m,
        min_used_satellites: profile.navigation.science_thresholds.min_used_satellites,
        min_lock_ratio: profile.navigation.science_thresholds.min_lock_ratio,
    }
}

fn handle_validate_capture(command: GnssCommand) -> Result<()> {
    let GnssCommand::ValidateCapture {
        common,
        file,
        sampling_hz,
        if_hz,
        code_hz,
        code_length,
        doppler_search_hz,
        doppler_step_hz,
        eph,
        prn,
    } = command
    else {
        bail!("invalid command for handler");
    };

    if prn.is_empty() {
        bail!("--prn is required for validate-capture");
    }

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
    let runtime =
        runtime_config_from_capture_start(&common, None, Some(&raw_iq_metadata.capture_start_utc));
    let sats = bijux_gnss_infra::api::core::prns_to_sats(&prn);

    let acquisition = AcquisitionEngine::new(config.clone(), runtime.clone());
    let acquisitions = acquisition.run_fft(&frame, &sats);
    let acquisition_rows = acquisitions.iter().map(acquisition_row_from_result).collect::<Vec<_>>();
    let acquisition_report = AcquisitionReport {
        sats: sats.clone(),
        search_summary: bijux_gnss_infra::api::core::AcqSearchSummary::from_results(&acquisitions),
        doppler_search: doppler_search_settings(&profile),
        code_phase_search: acquisitions
            .iter()
            .find_map(|result| result.assumptions.as_ref())
            .map(code_phase_search_settings_from_assumptions)
            .unwrap_or_else(|| code_phase_search_settings(&profile)),
        front_end_metrics: signal_quality.front_end_metrics.clone(),
        signal_quality: signal_quality.clone(),
        reported_prns: summarize_reported_prns(&acquisition_rows),
        primary_results: acquisition_rows.clone(),
        results: acquisition_rows,
    };

    let tracking =
        bijux_gnss_infra::api::receiver::TrackingEngine::new(config.clone(), runtime.clone());
    let tracks = tracking.track_from_acquisition(&frame, &acquisitions);
    let tracking_report = TrackingReport {
        sats,
        doppler_search: doppler_search_settings(&profile),
        front_end_metrics: signal_quality.front_end_metrics.clone(),
        signal_quality: signal_quality.clone(),
        epochs: tracks
            .iter()
            .flat_map(|track| {
                track.epochs.iter().map(move |epoch| TrackingRow {
                    epoch_idx: epoch.epoch.index,
                    sample_index: epoch.sample_index,
                    sat: track.sat,
                    carrier_hz: epoch.carrier_hz.0,
                    carrier_phase_cycles: epoch.carrier_phase_cycles.0,
                    code_rate_hz: epoch.code_rate_hz.0,
                    code_phase_samples: epoch.code_phase_samples.0,
                    prompt_i: epoch.prompt_i,
                    prompt_q: epoch.prompt_q,
                    early_i: epoch.early_i,
                    early_q: epoch.early_q,
                    late_i: epoch.late_i,
                    late_q: epoch.late_q,
                    lock: epoch.lock,
                    cn0_dbhz: epoch.cn0_dbhz,
                    pll_lock: epoch.pll_lock,
                    dll_lock: epoch.dll_lock,
                    fll_lock: epoch.fll_lock,
                    cycle_slip: epoch.cycle_slip,
                    nav_bit_lock: epoch.nav_bit_lock,
                    navigation_bit_sign: epoch.navigation_bit_sign,
                    cycle_slip_reason: epoch.cycle_slip_reason.clone(),
                    lock_state: epoch.lock_state.clone(),
                    lock_state_reason: epoch.lock_state_reason.clone(),
                    dll_err: epoch.dll_err,
                    pll_err: epoch.pll_err,
                    fll_err: epoch.fll_err,
                    anti_false_lock: epoch.anti_false_lock,
                })
            })
            .collect(),
    };

    write_signal_quality_report(&common, "validate_capture", &signal_quality)?;
    write_track_timeseries_for_command(
        &common,
        "validate_capture",
        &tracking_report,
        &profile,
        dataset.as_ref(),
    )?;
    let observation_artifacts = write_obs_timeseries_for_command(
        &common,
        "validate_capture",
        &config,
        &tracks,
        profile.navigation.hatch_window,
        &profile,
        dataset.as_ref(),
    )?;
    write_tracking_timing_for_command(&common, "validate_capture", &tracks, dataset.as_ref())?;

    let nav = read_ephemeris(&eph)?;
    let mut nav_solver = bijux_gnss_infra::api::receiver::Navigation::new(config, runtime.clone());
    let mut solutions = Vec::new();
    for obs_epoch in &observation_artifacts.epochs {
        if let Some(solution) = nav_solver.solve_epoch(obs_epoch, &nav) {
            solutions.push(solution);
        }
    }

    let out_dir = artifacts_dir(&common, "validate_capture", dataset.as_ref())?;
    write_nav_solution_outputs(&out_dir, &solutions)?;

    let validation = build_validation_report_from_observation_artifacts(
        &tracks,
        &observation_artifacts,
        &solutions,
        &[],
        profile.sample_rate_hz,
        false,
        vec!["broadcast_only".to_string()],
        validation_science_policy(&profile),
    )?;
    let validation_path = out_dir.join("validation_report.json");
    fs::write(&validation_path, serde_json::to_string_pretty(&validation)?)?;
    let evidence =
        validation_evidence_bundle(&observation_artifacts.epochs, &solutions, &validation);
    let evidence_path = out_dir.join("validation_evidence_bundle.json");
    fs::write(&evidence_path, serde_json::to_string_pretty(&evidence)?)?;

    let summary = CaptureValidationReport {
        acquisition: acquisition_report,
        tracked_prns: summarize_tracked_prns(&tracks),
        tracking_epochs: tracks.iter().map(|track| track.epochs.len()).sum(),
        observation_epochs: observation_artifacts.epochs.len(),
        navigation_attempts: summarize_navigation_attempts(&solutions),
        position_attempts: summarize_position_attempts(&solutions),
        validation,
    };

    emit_report(&common, "validate_capture", &summary)?;
    println!("wrote {}", validation_path.display());
    println!("wrote {}", evidence_path.display());
    write_manifest(&common, "validate_capture", &profile, dataset.as_ref(), &summary)?;

    Ok(())
}

fn handle_validate(command: GnssCommand) -> Result<()> {
    let GnssCommand::Validate { common, file, eph, reference, prn, sp3, clk } = command else {
        bail!("invalid command for handler");
    };

    let _ = runtime_config_from_env(&common, None);
    let file = file.context("--file is required for validation")?;
    let profile = load_config(&common)?;
    let dataset = load_dataset(&common)?;
    if profile.schema_version.0 != SchemaVersion::CURRENT.0 {
        bail!(
            "unsupported schema_version {}, expected {}",
            profile.schema_version.0,
            SchemaVersion::CURRENT.0
        );
    }

    validate_config_schema(&profile)?;

    let report = <ReceiverConfig as ValidateConfig>::validate(&profile);
    if !report.errors.is_empty() {
        bail!(
            "config invalid: {}",
            report.errors.iter().map(|e| e.message.as_str()).collect::<Vec<_>>().join(", ")
        );
    }

    let mut obs = read_obs_epochs(&file)?;
    let nav = read_ephemeris(&eph)?;
    let reference_epochs = read_reference_epochs(&reference)?;

    if !prn.is_empty() {
        obs.iter_mut().for_each(|e| {
            e.sats.retain(|sat| prn.contains(&sat.signal_id.sat.prn));
        });
    }

    let mut solutions = Vec::new();
    let mut nav_solver = bijux_gnss_infra::api::receiver::Navigation::new(
        profile.to_pipeline_config(),
        runtime_config_from_env(&common, None),
    );
    for obs_epoch in &obs {
        if let Some(sol) = nav_solver.solve_epoch(obs_epoch, &nav) {
            solutions.push(sol);
        }
    }

    #[cfg(feature = "precise-products")]
    let (products_ok, product_fallbacks) = {
        let mut products = bijux_gnss_infra::api::nav::Products::new(
            bijux_gnss_infra::api::nav::BroadcastProductsProvider::new(nav.clone()),
        );
        if let Some(path) = sp3 {
            let data = fs::read_to_string(path)?;
            let sp3 = data
                .parse::<bijux_gnss_infra::api::nav::Sp3Provider>()
                .map_err(|e| eyre!("sp3 parse error: {}", e))?;
            products = products.with_sp3(sp3);
        }
        if let Some(path) = clk {
            let data = fs::read_to_string(path)?;
            let clk = data
                .parse::<bijux_gnss_infra::api::nav::ClkProvider>()
                .map_err(|e| eyre!("clk parse error: {}", e))?;
            products = products.with_clk(clk);
        }
        let ok = products.sp3.is_some() || products.clk.is_some();
        let fallbacks = if ok { Vec::new() } else { vec!["broadcast_only".to_string()] };
        (ok, fallbacks)
    };
    #[cfg(not(feature = "precise-products"))]
    let (products_ok, product_fallbacks) = {
        if sp3.is_some() || clk.is_some() {
            bail!("precise-products feature disabled; recompile with feature to use SP3/CLK");
        }
        (false, vec!["precise_products_disabled".to_string()])
    };

    let report = build_validation_report(
        &[],
        &obs,
        &solutions,
        &reference_epochs,
        profile.sample_rate_hz,
        products_ok,
        product_fallbacks,
        validation_science_policy(&profile),
    )?;
    let out_dir = artifacts_dir(&common, "validate", dataset.as_ref())?;
    write_melbourne_wubbena_diagnostics(&out_dir, &obs)?;
    let out = out_dir.join("validation_report.json");
    fs::write(&out, serde_json::to_string_pretty(&report)?)?;
    let evidence = validation_evidence_bundle(&obs, &solutions, &report);
    let evidence_path = out_dir.join("validation_evidence_bundle.json");
    fs::write(&evidence_path, serde_json::to_string_pretty(&evidence)?)?;
    println!("wrote {}", out.display());
    println!("wrote {}", evidence_path.display());
    write_manifest(&common, "validate", &profile, dataset.as_ref(), &report)?;

    Ok(())
}

fn handle_validate_reference(command: GnssCommand) -> Result<()> {
    let GnssCommand::ValidateReference { common, run_dir, reference, align } = command else {
        bail!("invalid command for handler");
    };

    let _ = runtime_config_from_env(&common, None);
    let artifacts = run_dir.join("artifacts");
    let obs_path = artifacts.join("obs.jsonl");
    let nav_path = artifacts.join("pvt.jsonl");
    let obs = read_obs_epochs(&obs_path)?;
    let solutions = read_nav_solutions(&nav_path)?;
    let reference_epochs = read_reference_epochs(&reference)?;
    let align_policy = match align {
        ReferenceAlign::Nearest => bijux_gnss_infra::api::ReferenceAlign::Nearest,
        ReferenceAlign::Linear => bijux_gnss_infra::api::ReferenceAlign::Linear,
    };
    let aligned =
        bijux_gnss_infra::api::validate_reference(&solutions, &reference_epochs, align_policy)?;

    let report = build_validation_report(
        &[],
        &obs,
        &solutions,
        &aligned,
        0.0,
        false,
        vec!["run_dir_only".to_string()],
        bijux_gnss_infra::api::receiver::ValidationSciencePolicy::default(),
    )?;
    let out_dir = artifacts_dir(&common, "validate_reference", None)?;
    write_melbourne_wubbena_diagnostics(&out_dir, &obs)?;
    let out = out_dir.join("validation_report.json");
    fs::write(&out, serde_json::to_string_pretty(&report)?)?;
    let evidence = validation_evidence_bundle(&obs, &solutions, &report);
    let evidence_path = out_dir.join("validation_evidence_bundle.json");
    fs::write(&evidence_path, serde_json::to_string_pretty(&evidence)?)?;
    let summary = serde_json::json!({ "report": out.display().to_string() });
    write_manifest(&common, "validate_reference", &ReceiverConfig::default(), None, &summary)?;
    Ok(())
}

fn validation_evidence_bundle(
    obs: &[ObsEpoch],
    solutions: &[NavSolutionEpoch],
    report: &ValidationReport,
) -> serde_json::Value {
    let mut constellation_counts = std::collections::BTreeMap::new();
    let mut cn0_values = Vec::new();
    let mut lock_total = 0usize;
    let mut lock_good = 0usize;

    for epoch in obs {
        for sat in &epoch.sats {
            let key = format!("{:?}", sat.signal_id.sat.constellation);
            *constellation_counts.entry(key).or_insert(0usize) += 1;
            cn0_values.push(sat.cn0_dbhz);
            lock_total += 1;
            if sat.lock_flags.code_lock && sat.lock_flags.carrier_lock && !sat.lock_flags.cycle_slip
            {
                lock_good += 1;
            }
        }
    }

    let cn0_mean = if cn0_values.is_empty() {
        None
    } else {
        Some(cn0_values.iter().sum::<f64>() / cn0_values.len() as f64)
    };
    let cn0_min = cn0_values.iter().cloned().reduce(f64::min);
    let cn0_max = cn0_values.iter().cloned().reduce(f64::max);

    let mut refusal_counts = std::collections::BTreeMap::new();
    let mut pdop_values = Vec::new();
    let mut gdop_values = Vec::new();
    let mut rms_values = Vec::new();
    for sol in solutions {
        pdop_values.push(sol.pdop);
        if let Some(gdop) = sol.gdop {
            gdop_values.push(gdop);
        }
        rms_values.push(sol.rms_m.0);
        if let Some(refusal) = sol.refusal_class {
            let key = format!("{refusal:?}");
            *refusal_counts.entry(key).or_insert(0usize) += 1;
        }
    }
    let pdop_mean = if pdop_values.is_empty() {
        None
    } else {
        Some(pdop_values.iter().sum::<f64>() / pdop_values.len() as f64)
    };
    let pdop_max = pdop_values.iter().cloned().reduce(f64::max);
    let gdop_mean = if gdop_values.is_empty() {
        None
    } else {
        Some(gdop_values.iter().sum::<f64>() / gdop_values.len() as f64)
    };
    let gdop_max = gdop_values.iter().cloned().reduce(f64::max);
    let residual_rms_mean = if rms_values.is_empty() {
        None
    } else {
        Some(rms_values.iter().sum::<f64>() / rms_values.len() as f64)
    };
    let used_sat_mean = if solutions.is_empty() {
        None
    } else {
        Some(
            solutions.iter().map(|sol| sol.used_sat_count as f64).sum::<f64>()
                / solutions.len() as f64,
        )
    };
    let stable_solution_count = solutions
        .iter()
        .filter(|sol| matches!(sol.validity, bijux_gnss_infra::api::core::SolutionValidity::Stable))
        .count();
    let weak_integrity_stable_count = report
        .integrity
        .iter()
        .filter(|entry| {
            !matches!(entry.class, bijux_gnss_infra::api::receiver::NavIntegrityClass::Nominal)
        })
        .count();
    let reference_position_budget_pass =
        report.budgets.reference_position_error_3d_m_max.and_then(|budget_m| {
            (!report.reference_position_errors.is_empty()).then_some(
                report.reference_position_errors.iter().all(|error| error.error_3d_m <= budget_m),
            )
        });

    let mut claim_evidence_violations = Vec::new();
    if let Some(value) = cn0_mean {
        if value < report.science_policy.min_mean_cn0_dbhz {
            claim_evidence_violations.push(format!(
                "mean_cn0_below_policy:{value:.3}<{}",
                report.science_policy.min_mean_cn0_dbhz
            ));
        }
    }
    if let Some(value) = pdop_mean {
        if value > report.science_policy.max_pdop {
            claim_evidence_violations.push(format!(
                "mean_pdop_above_policy:{value:.3}>{}",
                report.science_policy.max_pdop
            ));
        }
    }
    if let Some(value) = gdop_mean {
        if value > report.science_policy.max_gdop {
            claim_evidence_violations.push(format!(
                "mean_gdop_above_policy:{value:.3}>{}",
                report.science_policy.max_gdop
            ));
        }
    }
    if let Some(value) = residual_rms_mean {
        if value > report.science_policy.max_residual_rms_m {
            claim_evidence_violations.push(format!(
                "mean_residual_rms_above_policy:{value:.3}>{}",
                report.science_policy.max_residual_rms_m
            ));
        }
    }
    if let Some(value) = used_sat_mean {
        if value < report.science_policy.min_used_satellites as f64 {
            claim_evidence_violations.push(format!(
                "mean_used_satellites_below_policy:{value:.3}<{}",
                report.science_policy.min_used_satellites
            ));
        }
    }
    let lock_quality_ratio =
        if lock_total == 0 { None } else { Some(lock_good as f64 / lock_total as f64) };
    if let Some(value) = lock_quality_ratio {
        if value < report.science_policy.min_lock_ratio {
            claim_evidence_violations.push(format!(
                "lock_quality_ratio_below_policy:{value:.3}<{}",
                report.science_policy.min_lock_ratio
            ));
        }
    }
    if stable_solution_count > 0 && weak_integrity_stable_count > 0 {
        claim_evidence_violations.push(format!(
            "stable_solutions_with_non_nominal_integrity:{weak_integrity_stable_count}/{stable_solution_count}"
        ));
    }

    serde_json::json!({
        "schema_version": 1,
        "physical": {
            "observation_epochs": obs.len(),
            "constellation_counts": constellation_counts,
            "cn0_dbhz_mean": cn0_mean,
            "cn0_dbhz_min": cn0_min,
            "cn0_dbhz_max": cn0_max,
            "lock_quality_ratio": lock_quality_ratio
        },
        "numerical": {
            "solution_epochs": solutions.len(),
            "stable_solution_epochs": stable_solution_count,
            "east_error_rms_m": report.east_error_m.rms,
            "north_error_rms_m": report.north_error_m.rms,
            "up_error_rms_m": report.up_error_m.rms,
            "horiz_error_rms_m": report.horiz_error_m.rms,
            "vert_error_rms_m": report.vert_error_m.rms,
            "error_3d_rms_m": report.error_3d_m.rms,
            "reference_match_count": report.reference_position_errors.len(),
            "reference_error_3d_max_m": report.error_3d_m.max,
            "reference_error_3d_budget_m_max": report.budgets.reference_position_error_3d_m_max,
            "reference_error_3d_budget_pass": reference_position_budget_pass,
            "mean_used_satellites": used_sat_mean,
            "pdop_mean": pdop_mean,
            "pdop_max": pdop_max,
            "gdop_mean": gdop_mean,
            "gdop_max": gdop_max,
            "residual_rms_mean_m": residual_rms_mean,
            "refusal_counts": refusal_counts
        },
        "diagnostics": {
            "advisory": report.diagnostic_partition.advisory_diagnostics,
            "enforced_refusals": report.diagnostic_partition.enforced_refusals
        },
        "claim_evidence_guard": {
            "policy": report.science_policy,
            "supported": claim_evidence_violations.is_empty(),
            "violations": claim_evidence_violations
        }
    })
}

#[cfg(test)]
mod validate_tests {
    use super::*;
    use bijux_gnss_infra::api::core::{
        Epoch, Meters, NavLifecycleState, NavSolutionEpoch, NavUncertaintyClass,
        ReceiverSampleTrace, Seconds, SolutionStatus, SolutionValidity,
        NAV_OUTPUT_STABILITY_SIGNATURE_VERSION, NAV_SOLUTION_MODEL_VERSION,
    };

    fn sample_solution(ecef_x_m: f64, ecef_y_m: f64, ecef_z_m: f64) -> NavSolutionEpoch {
        NavSolutionEpoch {
            epoch: Epoch { index: 5 },
            t_rx_s: Seconds(5.0),
            source_time: ReceiverSampleTrace::from_sample_index(5, 1.0),
            ecef_x_m: Meters(ecef_x_m),
            ecef_y_m: Meters(ecef_y_m),
            ecef_z_m: Meters(ecef_z_m),
            latitude_deg: 0.0,
            longitude_deg: 0.0,
            altitude_m: Meters(0.0),
            clock_bias_s: Seconds(0.0),
            clock_bias_m: Meters(0.0),
            clock_drift_s_per_s: 0.0,
            pdop: 1.0,
            pre_fit_residual_rms_m: None,
            post_fit_residual_rms_m: None,
            rms_m: Meters(0.5),
            status: SolutionStatus::Converged,
            quality: SolutionStatus::Converged.quality_flag(),
            validity: SolutionValidity::Stable,
            valid: true,
            processing_ms: None,
            residuals: Vec::new(),
            health: Vec::new(),
            isb: Vec::new(),
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
            lifecycle_state: NavLifecycleState::Converged,
            uncertainty_class: NavUncertaintyClass::Low,
            assumptions: None,
            refusal_class: None,
            artifact_id: "nav-epoch-0000000005-validate".to_string(),
            source_observation_epoch_id: "obs-epoch-0000000005-validate".to_string(),
            explain_decision: "accepted".to_string(),
            explain_reasons: vec!["validation_fixture".to_string()],
            provenance: None,
            sat_count: 4,
            used_sat_count: 4,
            rejected_sat_count: 0,
            hdop: Some(1.0),
            vdop: Some(1.0),
            gdop: Some(1.0),
            tdop: Some(0.5),
            stability_signature: "navsig:v2:validate".to_string(),
            stability_signature_version: NAV_OUTPUT_STABILITY_SIGNATURE_VERSION,
        }
    }

    #[test]
    fn validation_evidence_bundle_includes_enu_reference_metrics() {
        let (x_ref, y_ref, z_ref) = bijux_gnss_infra::api::core::lla_to_ecef(0.0, 0.0, 0.0);
        let solution = sample_solution(x_ref + 3.0, y_ref + 1.0, z_ref + 2.0);
        let report = bijux_gnss_infra::api::receiver::build_validation_report(
            &[],
            &[],
            std::slice::from_ref(&solution),
            &[ValidationReferenceEpoch {
                epoch_idx: 5,
                t_rx_s: Some(5.0),
                latitude_deg: 0.0,
                longitude_deg: 0.0,
                altitude_m: 0.0,
                ecef_x_m: Some(x_ref),
                ecef_y_m: Some(y_ref),
                ecef_z_m: Some(z_ref),
                vel_x_mps: None,
                vel_y_mps: None,
                vel_z_mps: None,
            }],
            1.0,
            false,
            Vec::new(),
            bijux_gnss_infra::api::receiver::ValidationSciencePolicy::default(),
        )
        .expect("validation report");

        let evidence = validation_evidence_bundle(&[], &[solution], &report);

        assert_eq!(evidence["numerical"]["east_error_rms_m"], 1.0);
        assert_eq!(evidence["numerical"]["north_error_rms_m"], 2.0);
        assert_eq!(evidence["numerical"]["up_error_rms_m"], 3.0);
        assert_eq!(evidence["numerical"]["horiz_error_rms_m"], 5.0_f64.sqrt());
        assert_eq!(evidence["numerical"]["vert_error_rms_m"], 3.0);
        assert_eq!(evidence["numerical"]["error_3d_rms_m"], 14.0_f64.sqrt());
        assert_eq!(evidence["numerical"]["reference_match_count"], 1);
        assert_eq!(evidence["numerical"]["reference_error_3d_max_m"], 14.0_f64.sqrt());
        assert!(evidence["numerical"]["reference_error_3d_budget_m_max"].is_null());
        assert!(evidence["numerical"]["reference_error_3d_budget_pass"].is_null());
    }

    #[test]
    fn validation_evidence_bundle_reports_reference_position_budget_status() {
        let (x_ref, y_ref, z_ref) = bijux_gnss_infra::api::core::lla_to_ecef(0.0, 0.0, 0.0);
        let solution = sample_solution(x_ref + 3.0, y_ref + 1.0, z_ref + 2.0);
        let report = bijux_gnss_infra::api::receiver::build_validation_report_with_budgets(
            &[],
            &[],
            std::slice::from_ref(&solution),
            &[ValidationReferenceEpoch {
                epoch_idx: 5,
                t_rx_s: Some(5.0),
                latitude_deg: 0.0,
                longitude_deg: 0.0,
                altitude_m: 0.0,
                ecef_x_m: Some(x_ref),
                ecef_y_m: Some(y_ref),
                ecef_z_m: Some(z_ref),
                vel_x_mps: None,
                vel_y_mps: None,
                vel_z_mps: None,
            }],
            1.0,
            false,
            Vec::new(),
            bijux_gnss_infra::api::receiver::ValidationSciencePolicy::default(),
            bijux_gnss_infra::api::receiver::ValidationBudgets {
                nav_min_lock_epochs: 0,
                reference_position_error_3d_m_max: Some(3.0),
                ..bijux_gnss_infra::api::receiver::ValidationBudgets::default()
            },
        )
        .expect("validation report");

        let evidence = validation_evidence_bundle(&[], &[solution], &report);

        assert_eq!(evidence["numerical"]["reference_error_3d_budget_m_max"], 3.0);
        assert_eq!(evidence["numerical"]["reference_error_3d_budget_pass"], false);
    }
}
