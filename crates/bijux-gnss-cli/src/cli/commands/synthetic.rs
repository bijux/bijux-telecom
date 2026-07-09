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
