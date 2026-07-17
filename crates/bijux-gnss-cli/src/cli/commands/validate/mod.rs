use super::*;

mod artifact_validation;
mod evidence_bundle;
mod schema_validation;

pub(crate) use artifact_validation::handle_validate_artifacts;
pub(crate) use evidence_bundle::validation_evidence_bundle;
pub(crate) use schema_validation::{
    CsvType, validate_config_schema, validate_csv_schema, validate_json_schema,
    validate_jsonl_schema, validate_sidecar_schema,
};
#[cfg(test)]
mod tests;

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

pub(crate) fn handle_validate_capture(command: GnssCommand) -> Result<()> {
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

    let receiver = Receiver::new(config.clone(), runtime.clone());
    let mut source = open_tracking_window_source(&input_file, &config, &raw_iq_metadata)?;
    let artifacts = receiver.run_with_satellites(&mut source, &sats)?;
    let tracks = artifacts.tracking.clone();
    let observation_artifacts = artifacts.observation_artifacts();
    let acquisition_rows =
        artifacts.acquisitions.iter().map(acquisition_row_from_result).collect::<Vec<_>>();
    let acquisition_report = AcquisitionReport {
        sats: sats.clone(),
        search_summary: bijux_gnss_infra::api::core::AcqSearchSummary::from_results(
            &artifacts.acquisitions,
        ),
        doppler_search: doppler_search_settings(&profile),
        code_phase_search: artifacts
            .acquisitions
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
    let observation_artifacts = write_observation_artifacts_for_command(
        &common,
        "validate_capture",
        &observation_artifacts,
        &profile,
        dataset.as_ref(),
    )?;
    write_tracking_timing_for_command(&common, "validate_capture", &tracks, dataset.as_ref())?;

    let broadcast_navigation = read_broadcast_navigation_data(&eph)?;
    let solutions = receiver.solve_observation_epochs_with_gps_broadcast_navigation(
        &observation_artifacts.epochs,
        &broadcast_navigation,
    );

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

pub(crate) fn handle_validate(command: GnssCommand) -> Result<()> {
    let GnssCommand::Validate { common, file, eph, reference, prn, sp3, clk, bias_sinex } = command else {
        bail!("invalid command for handler");
    };

    let runtime = runtime_config_from_env(&common, None);
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
    let broadcast_navigation = read_broadcast_navigation_data(&eph)?;
    let reference_epochs = read_reference_epochs(&reference)?;

    if !prn.is_empty() {
        obs.iter_mut().for_each(|e| {
            e.sats.retain(|sat| prn.contains(&sat.signal_id.sat.prn));
        });
    }

    let receiver =
        bijux_gnss_infra::api::receiver::Receiver::new(profile.to_pipeline_config(), runtime);
    let solutions =
        receiver.solve_observation_epochs_with_gps_broadcast_navigation(&obs, &broadcast_navigation);

    #[cfg(feature = "precise-products")]
    let (products_ok, product_fallbacks, code_biases) = {
        let mut products = bijux_gnss_infra::api::nav::Products::new(
            bijux_gnss_infra::api::nav::BroadcastProductsProvider::new(
                broadcast_navigation.ephemerides.clone(),
            ),
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
        if let Some(path) = bias_sinex {
            let data = fs::read_to_string(path)?;
            let bias_sinex = data
                .parse::<bijux_gnss_infra::api::nav::BiasSinexProvider>()
                .map_err(|e| eyre!("bias sinex parse error: {}", e))?;
            products = products.with_dcb(bias_sinex);
        }
        let ok = products.sp3.is_some() || products.clk.is_some() || products.dcb.is_some();
        let fallbacks = if ok { Vec::new() } else { vec!["broadcast_only".to_string()] };
        (ok, fallbacks, products.dcb.clone())
    };
    #[cfg(not(feature = "precise-products"))]
    let (products_ok, product_fallbacks, code_biases) = {
        if sp3.is_some() || clk.is_some() || bias_sinex.is_some() {
            bail!(
                "precise-products feature disabled; recompile with feature to use SP3/CLK/Bias-SINEX"
            );
        }
        (false, vec!["precise_products_disabled".to_string()], None)
    };

    let out_dir = artifacts_dir(&common, "validate", dataset.as_ref())?;
    write_iono_free_code_artifact(
        &out_dir,
        &obs,
        code_biases
            .as_ref()
            .map(|provider| provider as &dyn bijux_gnss_infra::api::nav::CodeBiasProvider),
    )?;

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

pub(crate) fn handle_validate_reference(command: GnssCommand) -> Result<()> {
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
