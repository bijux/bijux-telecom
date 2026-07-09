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
    };
    emit_synthetic_iq_validation_report(&common, &report)?;
    write_manifest(&common, "validate_synthetic_iq", &profile, dataset.as_ref(), &report)?;

    if !report.validation.pass || !report.acquisition_code_phase_validation.pass {
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
        let mut failure_sections = Vec::new();
        if !cn0_failures.is_empty() {
            failure_sections.push(format!("cn0={cn0_failures}"));
        }
        if !acquisition_failures.is_empty() {
            failure_sections.push(format!("acq_code_phase={acquisition_failures}"));
        }
        bail!("synthetic IQ validation failed: {}", failure_sections.join("; "));
    }

    Ok(())
}
