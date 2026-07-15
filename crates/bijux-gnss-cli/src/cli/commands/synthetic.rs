fn export_synthetic_iq_args<'a>(
    out: Option<&'a PathBuf>,
) -> bijux_gnss_infra::api::RunContextArgs<'a> {
    bijux_gnss_infra::api::RunContextArgs {
        config: None,
        dataset_id: None,
        unregistered_dataset: true,
        out,
        resume: None,
        deterministic: true,
        sidecar: None,
    }
}

fn synthetic_navigation_args(common: &CommonArgs) -> bijux_gnss_infra::api::RunContextArgs<'_> {
    bijux_gnss_infra::api::RunContextArgs {
        config: common.config.as_ref(),
        dataset_id: None,
        unregistered_dataset: true,
        out: common.out.as_ref(),
        resume: common.resume.as_ref(),
        deterministic: common.deterministic,
        sidecar: None,
    }
}

fn resolved_scenario_id(
    scenario: &bijux_gnss_infra::api::receiver::sim::SyntheticScenario,
    path: &Path,
) -> String {
    let trimmed = scenario.id.trim();
    if !trimmed.is_empty() {
        return trimmed.to_string();
    }
    path.file_stem()
        .and_then(|stem| stem.to_str())
        .filter(|stem| !stem.trim().is_empty())
        .unwrap_or("synthetic_iq")
        .to_string()
}

fn emit_synthetic_iq_report(
    report: &SyntheticIqExportReport,
    layout: &bijux_gnss_infra::api::RunDirLayout,
    report_format: ReportFormat,
) -> Result<()> {
    let summary = serde_json::to_value(report)?;
    fs::write(&layout.summary_path, serde_json::to_string_pretty(&summary)?)?;

    match report_format {
        ReportFormat::Json => {
            let report_path = layout.run_dir.join("export_synthetic_iq_report.json");
            fs::write(&report_path, serde_json::to_string_pretty(report)?)?;
            println!("wrote {}", report_path.display());
        }
        ReportFormat::Table => print_synthetic_iq_export_table(report),
    }
    Ok(())
}

fn emit_synthetic_iq_validation_report(
    common: &CommonArgs,
    report: &SyntheticIqValidationReport,
) -> Result<()> {
    let dataset = load_dataset(common).ok().flatten();
    let run_dir = run_dir(common, "validate_synthetic_iq", dataset.as_ref())?;
    fs::write(run_dir.join("summary.json"), serde_json::to_string_pretty(report)?)?;

    match common.report {
        ReportFormat::Json => {
            let report_path = run_dir.join("validate_synthetic_iq_report.json");
            fs::write(&report_path, serde_json::to_string_pretty(report)?)?;
            println!("wrote {}", report_path.display());
        }
        ReportFormat::Table => print_synthetic_iq_validation_table(report),
    }
    Ok(())
}

fn emit_synthetic_navigation_validation_report(
    common: &CommonArgs,
    report: &SyntheticNavigationValidationReport,
) -> Result<()> {
    let run_dir =
        bijux_gnss_infra::api::run_dir(&synthetic_navigation_args(common), "validate_synthetic_navigation", None)
            .map_err(|err| eyre!(err.message))?;
    fs::write(run_dir.join("summary.json"), serde_json::to_string_pretty(report)?)?;

    match common.report {
        ReportFormat::Json => {
            let report_path = run_dir.join("validate_synthetic_navigation_report.json");
            fs::write(&report_path, serde_json::to_string_pretty(report)?)?;
            println!("wrote {}", report_path.display());
        }
        ReportFormat::Table => print_synthetic_navigation_validation_table(report),
    }
    Ok(())
}

fn emit_synthetic_quantization_measurement_report(
    common: &CommonArgs,
    report: &SyntheticQuantizationMeasurementReport,
) -> Result<()> {
    let run_dir = bijux_gnss_infra::api::run_dir(
        &synthetic_navigation_args(common),
        "measure_synthetic_quantization",
        None,
    )
    .map_err(|err| eyre!(err.message))?;
    fs::write(run_dir.join("summary.json"), serde_json::to_string_pretty(report)?)?;

    match common.report {
        ReportFormat::Json => {
            let report_path = run_dir.join("measure_synthetic_quantization_report.json");
            fs::write(&report_path, serde_json::to_string_pretty(report)?)?;
            println!("wrote {}", report_path.display());
        }
        ReportFormat::Table => print_synthetic_quantization_measurement_table(report),
    }
    Ok(())
}

fn resolved_quantization_profiles(
    quantization: &[SyntheticQuantizationArg],
) -> Vec<bijux_gnss_infra::api::signal::IqQuantization> {
    let requested = if quantization.is_empty() {
        bijux_gnss_infra::api::receiver::sim::truth_guided_quantization_reference_sweep()
            .to_vec()
    } else {
        quantization.iter().map(|entry| entry.into_quantization()).collect::<Vec<_>>()
    };
    let mut resolved = vec![bijux_gnss_infra::api::signal::IqQuantization::Float32];
    for entry in requested {
        if !resolved.contains(&entry) {
            resolved.push(entry);
        }
    }
    resolved
}

fn handle_export_synthetic_iq(command: GnssCommand) -> Result<()> {
    let GnssCommand::ExportSyntheticIq { scenario, out, report, capture_start_utc } = command
    else {
        bail!("invalid command for handler");
    };

    let scenario_contents = fs::read_to_string(&scenario)
        .with_context(|| format!("failed to read scenario {}", scenario.display()))?;
    let scenario_def: bijux_gnss_infra::api::receiver::sim::SyntheticScenario =
        toml::from_str(&scenario_contents)?;
    let scenario_id = resolved_scenario_id(&scenario_def, &scenario);

    let mut profile = ReceiverConfig::default();
    profile.sample_rate_hz = scenario_def.sample_rate_hz;
    profile.intermediate_freq_hz = scenario_def.intermediate_freq_hz;
    profile.quantization_bits = 16;
    profile.seed = scenario_def.seed;
    validate_config(&profile)?;
    let config = profile.to_pipeline_config();
    let frame = bijux_gnss_infra::api::receiver::sim::generate_l1_ca_multi(&config, &scenario_def);
    let bundle = bijux_gnss_infra::api::receiver::sim::build_iq16_capture_bundle(
        &scenario_id,
        &scenario_def,
        &frame,
        &capture_start_utc,
        Some(format!(
            "synthetic iq export from scenario {} with seed {}",
            scenario_id, scenario_def.seed
        )),
    );
    validate_sidecar_schema(&bundle.metadata)?;

    let args = export_synthetic_iq_args(out.as_ref());
    let (layout, _header) = prepare_run(&args, "export_synthetic_iq", &profile, None)
        .map_err(|err| eyre!(err.message))?;
    let base_name = scenario_id.clone();
    let iq_path = layout.artifacts_dir.join(format!("{base_name}.iq16"));
    let sidecar_path = layout.artifacts_dir.join(format!("{base_name}.sidecar.toml"));
    let truth_path = layout.artifacts_dir.join(format!("{base_name}.truth.json"));
    let scenario_copy_path = layout.artifacts_dir.join(format!("{base_name}.scenario.toml"));

    fs::write(&iq_path, &bundle.raw_iq_bytes)?;
    fs::write(&sidecar_path, toml::to_string_pretty(&bundle.metadata)?)?;
    fs::write(&scenario_copy_path, scenario_contents)?;
    fs::write(&truth_path, serde_json::to_string_pretty(&bundle.truth)?)?;
    validate_json_schema(&schema_path("synthetic_iq_truth.schema.json"), &truth_path, false)?;

    let export_report = SyntheticIqExportReport {
        scenario_id,
        seed: scenario_def.seed,
        sample_count: bundle.truth.sample_count,
        sample_rate_hz: bundle.truth.sample_rate_hz,
        output_iq: iq_path.display().to_string(),
        output_sidecar: sidecar_path.display().to_string(),
        output_truth: truth_path.display().to_string(),
        satellites: scenario_def.satellites.iter().map(|sat| sat.sat).collect(),
    };
    emit_synthetic_iq_report(&export_report, &layout, report)?;
    let summary = serde_json::to_value(&export_report)?;
    let _ = bijux_gnss_infra::api::write_manifest(
        &args,
        "export_synthetic_iq",
        &profile,
        None,
        &summary,
    )?;

    Ok(())
}

fn handle_validate_synthetic_iq(command: GnssCommand) -> Result<()> {
    let GnssCommand::ValidateSyntheticIq {
        common,
        file,
        truth,
        tolerance_db_hz,
        acquisition_code_phase_tolerance_samples,
        acquisition_doppler_tolerance_bins,
    } = command
    else {
        bail!("invalid command for handler");
    };

    let dataset = load_dataset(&common)?;
    let input_file = resolve_input_file(Some(&file), dataset.as_ref())?;
    let raw_iq_metadata = resolve_raw_iq_metadata(&common, dataset.as_ref())?;
    validate_json_schema(&schema_path("synthetic_iq_truth.schema.json"), &truth, false)?;
    let truth_bundle: bijux_gnss_infra::api::receiver::sim::SyntheticIqTruthBundle =
        serde_json::from_str(
            &fs::read_to_string(&truth)
                .with_context(|| format!("failed to read truth {}", truth.display()))?,
        )
        .with_context(|| format!("failed to parse truth {}", truth.display()))?;

    if (raw_iq_metadata.sample_rate_hz - truth_bundle.sample_rate_hz).abs() > 1e-9 {
        bail!(
            "truth/sample-rate mismatch: sidecar={} truth={}",
            raw_iq_metadata.sample_rate_hz,
            truth_bundle.sample_rate_hz
        );
    }
    if (raw_iq_metadata.intermediate_freq_hz - truth_bundle.intermediate_freq_hz).abs() > 1e-9 {
        bail!(
            "truth/intermediate-frequency mismatch: sidecar={} truth={}",
            raw_iq_metadata.intermediate_freq_hz,
            truth_bundle.intermediate_freq_hz
        );
    }
    if raw_iq_metadata.quantization_bits.unwrap_or_default() != truth_bundle.quantization_bits {
        bail!(
            "truth/quantization mismatch: sidecar={} truth={}",
            raw_iq_metadata.quantization_bits.unwrap_or_default(),
            truth_bundle.quantization_bits
        );
    }

    let mut profile = load_config(&common)?;
    apply_common_overrides(
        &mut profile,
        CommonOverrides { seed: common.seed, deterministic: common.deterministic },
    );
    apply_raw_iq_metadata(&mut profile, &raw_iq_metadata, None, None)?;
    validate_config(&profile)?;
    let config = profile.to_pipeline_config();
    let frame = load_tracking_frame(&input_file, &config, &raw_iq_metadata)?;
    let validation = bijux_gnss_infra::api::receiver::sim::validate_truth_guided_cn0(
        &config,
        &frame,
        &truth_bundle,
        tolerance_db_hz,
    );
    let acquisition_code_phase_validation =
        bijux_gnss_infra::api::receiver::sim::validate_truth_guided_acquisition_code_phase(
            &config,
            &frame,
            &truth_bundle,
            acquisition_code_phase_tolerance_samples,
        );
    let acquisition_code_phase_refinement_validation =
        bijux_gnss_infra::api::receiver::sim::validate_truth_guided_acquisition_code_phase_refinement(
            &config,
            &frame,
            &truth_bundle,
        );
    let acquisition_doppler_validation =
        bijux_gnss_infra::api::receiver::sim::validate_truth_guided_acquisition_doppler(
            &config,
            &frame,
            &truth_bundle,
            acquisition_doppler_tolerance_bins,
        );
    let acquisition_receiver_clock_offset_validation =
        bijux_gnss_infra::api::receiver::sim::validate_truth_guided_acquisition_receiver_clock_offset(
            &config,
            &frame,
            &truth_bundle,
            acquisition_doppler_tolerance_bins,
        );
    let report = SyntheticIqValidationReport {
        input_iq: input_file.display().to_string(),
        input_sidecar: common
            .sidecar
            .as_ref()
            .map(|path| path.display().to_string())
            .unwrap_or_default(),
        input_truth: truth.display().to_string(),
        validation,
        acquisition_code_phase_validation,
        acquisition_code_phase_refinement_validation,
        acquisition_doppler_validation,
        acquisition_receiver_clock_offset_validation,
    };
    emit_synthetic_iq_validation_report(&common, &report)?;
    write_manifest(&common, "validate_synthetic_iq", &profile, dataset.as_ref(), &report)?;

    if !report.validation.pass
        || !report.acquisition_code_phase_validation.pass
        || !report.acquisition_code_phase_refinement_validation.pass
        || !report.acquisition_doppler_validation.pass
        || !report.acquisition_receiver_clock_offset_validation.pass
    {
        let cn0_failures = report
            .validation
            .satellites
            .iter()
            .filter(|row| !row.pass)
            .map(|row| {
                format!(
                    "{}:{}:{:.3}",
                    format_sat(row.sat),
                    row.injected_cn0_db_hz,
                    row.cn0_delta_db
                )
            })
            .collect::<Vec<_>>()
            .join(", ");
        let acquisition_failures = report
            .acquisition_code_phase_validation
            .satellites
            .iter()
            .filter(|row| !row.pass)
            .map(|row| {
                format!(
                    "{}:{}:{}:{}",
                    format_sat(row.sat),
                    row.expected_code_phase_samples,
                    row.measured_code_phase_samples,
                    row.code_phase_error_samples
                )
            })
            .collect::<Vec<_>>()
            .join(", ");
        let acquisition_doppler_failures = report
            .acquisition_doppler_validation
            .satellites
            .iter()
            .filter(|row| !row.pass)
            .map(|row| {
                format!(
                    "{}:{:.3}:{:.3}:{:.3}",
                    format_sat(row.sat),
                    row.injected_doppler_hz,
                    row.measured_doppler_hz,
                    row.doppler_error_hz
                )
            })
            .collect::<Vec<_>>()
            .join(", ");
        let acquisition_code_phase_refinement_failures = report
            .acquisition_code_phase_refinement_validation
            .satellites
            .iter()
            .filter(|row| !row.pass)
            .map(|row| {
                format!(
                    "{}:{:.6}:{:.6}:{:.6}",
                    format_sat(row.sat),
                    row.expected_code_phase_samples,
                    row.coarse_error_samples,
                    row.refined_error_samples
                )
            })
            .collect::<Vec<_>>()
            .join(", ");
        let acquisition_receiver_clock_offset_failures = report
            .acquisition_receiver_clock_offset_validation
            .satellites
            .iter()
            .filter(|row| !row.pass)
            .map(|row| {
                format!(
                    "{}:{:.3}:{:.3}:{:.3}:{:.3}",
                    format_sat(row.sat),
                    row.injected_receiver_clock_frequency_bias_hz,
                    row.expected_measured_doppler_hz,
                    row.measured_doppler_hz,
                    row.receiver_clock_frequency_bias_error_hz
                )
            })
            .collect::<Vec<_>>()
            .join(", ");
        let mut failure_sections = Vec::new();
        if !cn0_failures.is_empty() {
            failure_sections.push(format!("cn0={cn0_failures}"));
        }
        if !acquisition_failures.is_empty() {
            failure_sections.push(format!("acq_code_phase={acquisition_failures}"));
        }
        if !acquisition_code_phase_refinement_failures.is_empty() {
            failure_sections.push(format!(
                "acq_code_phase_refinement={acquisition_code_phase_refinement_failures}"
            ));
        }
        if !acquisition_doppler_failures.is_empty() {
            failure_sections.push(format!("acq_doppler={acquisition_doppler_failures}"));
        }
        if !acquisition_receiver_clock_offset_failures.is_empty() {
            failure_sections.push(format!(
                "acq_receiver_clock_offset={acquisition_receiver_clock_offset_failures}"
            ));
        }
        bail!("synthetic IQ validation failed: {}", failure_sections.join("; "));
    }

    Ok(())
}

fn handle_validate_synthetic_navigation(command: GnssCommand) -> Result<()> {
    let GnssCommand::ValidateSyntheticNavigation { common, scenario } = command else {
        bail!("invalid command for handler");
    };

    let scenario_contents = fs::read_to_string(&scenario)
        .with_context(|| format!("failed to read scenario {}", scenario.display()))?;
    let scenario_def: bijux_gnss_infra::api::receiver::sim::SyntheticNavigationValidationScenario =
        toml::from_str(&scenario_contents)
            .with_context(|| format!("failed to parse scenario {}", scenario.display()))?;

    let mut profile = load_config(&common)?;
    apply_common_overrides(
        &mut profile,
        CommonOverrides { seed: common.seed, deterministic: common.deterministic },
    );
    profile.sample_rate_hz = scenario_def.sample_rate_hz;
    profile.intermediate_freq_hz = scenario_def.intermediate_freq_hz;
    profile.quantization_bits = 16;
    profile.seed = scenario_def.seed;
    validate_config(&profile)?;

    let validation_run = bijux_gnss_infra::api::receiver::sim::validate_synthetic_navigation_run(
        &profile.to_pipeline_config(),
        &scenario_def,
        profile.navigation.hatch_window,
    )
    .map_err(|error| eyre!(error.to_string()))?;

    let artifact_dir = bijux_gnss_infra::api::artifacts_dir(
        &synthetic_navigation_args(&common),
        "validate_synthetic_navigation",
        None,
    )
    .map_err(|err| eyre!(err.message))?;
    let artifact_path = artifact_dir.join("gnss_accuracy_artifact.json");
    bijux_gnss_infra::api::receiver::sim::write_truth_guided_gnss_accuracy_artifact(
        &artifact_path,
        &validation_run.artifact,
    )?;
    let parsed_artifact: bijux_gnss_infra::api::receiver::sim::SyntheticGnssAccuracyArtifact =
        serde_json::from_str(
            &fs::read_to_string(&artifact_path)
                .with_context(|| format!("failed to read {}", artifact_path.display()))?,
        )
        .with_context(|| format!("failed to parse {}", artifact_path.display()))?;
    if parsed_artifact.scenario_id != validation_run.artifact.scenario_id
        || parsed_artifact.pass != validation_run.artifact.pass
        || parsed_artifact.truth_coverage_ready != validation_run.artifact.truth_coverage_ready
    {
        bail!("synthetic navigation accuracy artifact summary did not round-trip cleanly");
    }
    validate_json_schema(
        &schema_path("synthetic_gnss_accuracy_artifact.schema.json"),
        &artifact_path,
        false,
    )?;

    let scenario_id = if validation_run.signal_scenario.id.trim().is_empty() {
        scenario
            .file_stem()
            .and_then(|stem| stem.to_str())
            .filter(|stem| !stem.trim().is_empty())
            .unwrap_or("synthetic_navigation_validation")
            .to_string()
    } else {
        validation_run.signal_scenario.id.clone()
    };
    let report = SyntheticNavigationValidationReport {
        scenario_id,
        scenario_path: scenario.display().to_string(),
        output_artifact: artifact_path.display().to_string(),
        pass: validation_run.artifact.pass,
        truth_coverage_ready: validation_run.artifact.truth_coverage_ready,
        data_source: validation_run.artifact.data_source.clone(),
        reference_truth: validation_run.artifact.reference_truth.clone(),
        acquisition: validation_run.artifact.acquisition.summary.clone(),
        tracking: validation_run.artifact.tracking.summary.clone(),
        observation: validation_run.artifact.observation.summary.clone(),
        pvt: validation_run.artifact.pvt.summary.clone(),
        closure_ready: validation_run.artifact.closure_ready,
        closure: validation_run.artifact.closure.clone(),
    };
    emit_synthetic_navigation_validation_report(&common, &report)?;
    let _ = bijux_gnss_infra::api::write_manifest(
        &synthetic_navigation_args(&common),
        "validate_synthetic_navigation",
        &profile,
        None,
        &serde_json::to_value(&report)?,
    )?;

    Ok(())
}

fn handle_measure_synthetic_quantization(command: GnssCommand) -> Result<()> {
    let GnssCommand::MeasureSyntheticQuantization {
        common,
        scenario,
        quantization,
        capture_start_utc,
    } = command
    else {
        bail!("invalid command for handler");
    };

    let scenario_contents = fs::read_to_string(&scenario)
        .with_context(|| format!("failed to read scenario {}", scenario.display()))?;
    let scenario_def: bijux_gnss_infra::api::receiver::sim::SyntheticScenario =
        toml::from_str(&scenario_contents)
            .with_context(|| format!("failed to parse scenario {}", scenario.display()))?;
    let scenario_id = resolved_scenario_id(&scenario_def, &scenario);
    let measured_quantizations = resolved_quantization_profiles(&quantization);

    let mut profile = load_config(&common)?;
    apply_common_overrides(
        &mut profile,
        CommonOverrides { seed: common.seed, deterministic: common.deterministic },
    );
    profile.sample_rate_hz = scenario_def.sample_rate_hz;
    profile.intermediate_freq_hz = scenario_def.intermediate_freq_hz;
    profile.quantization_bits = 32;
    profile.seed = scenario_def.seed;
    validate_config(&profile)?;

    let measurement = bijux_gnss_infra::api::receiver::sim::measure_truth_guided_quantization_loss(
        &profile.to_pipeline_config(),
        &scenario_def,
        &measured_quantizations,
        &capture_start_utc,
    );
    let artifact_dir = bijux_gnss_infra::api::artifacts_dir(
        &synthetic_navigation_args(&common),
        "measure_synthetic_quantization",
        None,
    )
    .map_err(|err| eyre!(err.message))?;
    let artifact_path = artifact_dir.join("synthetic_quantization_loss_artifact.json");
    bijux_gnss_infra::api::receiver::sim::write_truth_guided_quantization_loss_artifact(
        &artifact_path,
        &measurement,
    )?;
    let parsed_artifact: bijux_gnss_infra::api::receiver::sim::SyntheticQuantizationLossReport =
        serde_json::from_str(
            &fs::read_to_string(&artifact_path)
                .with_context(|| format!("failed to read {}", artifact_path.display()))?,
        )
        .with_context(|| format!("failed to parse {}", artifact_path.display()))?;
    if parsed_artifact != measurement {
        bail!("synthetic quantization artifact did not round-trip cleanly");
    }

    let report = SyntheticQuantizationMeasurementReport {
        scenario_id,
        scenario_path: scenario.display().to_string(),
        output_artifact: artifact_path.display().to_string(),
        measured_quantizations,
        measurement,
    };
    emit_synthetic_quantization_measurement_report(&common, &report)?;
    let _ = bijux_gnss_infra::api::write_manifest(
        &synthetic_navigation_args(&common),
        "measure_synthetic_quantization",
        &profile,
        None,
        &serde_json::to_value(&report)?,
    )?;

    Ok(())
}
