fn load_profile(common: &CommonArgs) -> Result<ReceiverProfile> {
    match &common.config {
        Some(path) => load_profile_from_path(path),
        None => Ok(ReceiverProfile::default()),
    }
}

fn load_profile_from_path(path: &Path) -> Result<ReceiverProfile> {
    let contents = fs::read_to_string(path)
        .with_context(|| format!("failed to read config {}", path.display()))?;
    let profile: ReceiverProfile = toml::from_str(&contents)
        .with_context(|| format!("failed to parse config {}", path.display()))?;
    Ok(profile)
}

fn load_dataset(common: &CommonArgs) -> Result<Option<DatasetEntry>> {
    let Some(id) = &common.dataset else {
        return Ok(None);
    };
    let registry_path = PathBuf::from("datasets/registry.yaml");
    let contents = fs::read_to_string(&registry_path)
        .with_context(|| format!("failed to read {}", registry_path.display()))?;
    let registry: DatasetRegistry = serde_yaml::from_str(&contents)
        .with_context(|| format!("failed to parse {}", registry_path.display()))?;
    let entry = registry
        .entries
        .into_iter()
        .find(|e| e.id == *id)
        .with_context(|| format!("dataset not found: {id}"))?;
    Ok(Some(entry))
}

fn resolve_input_file(file: Option<&PathBuf>, dataset: Option<&DatasetEntry>) -> Result<PathBuf> {
    if let Some(file) = file {
        return Ok(file.clone());
    }
    if let Some(dataset) = dataset {
        return Ok(PathBuf::from(&dataset.path));
    }
    bail!("no input file provided; use --file or --dataset");
}

fn load_sidecar(path: Option<&PathBuf>) -> Result<Option<SidecarSpec>> {
    let Some(path) = path else {
        return Ok(None);
    };
    let data = fs::read_to_string(path)?;
    let spec: SidecarSpec = toml::from_str(&data)?;
    validate_sidecar_schema(&spec)?;
    Ok(Some(spec))
}


fn emit_report<T: Serialize>(common: &CommonArgs, command: &str, report: &T) -> Result<()> {
    match common.report {
        ReportFormat::Json => {
            let json = serde_json::to_string_pretty(report)?;
            if let Some(out_dir) = &common.out {
                fs::create_dir_all(out_dir)?;
                let path = out_dir.join(format!("{command}_report.json"));
                fs::write(&path, json)?;
                println!("wrote {}", path.display());
            } else {
                println!("{json}");
            }
        }
        ReportFormat::Table => {
            let json = serde_json::to_string_pretty(report)?;
            println!("{json}");
        }
    }
    Ok(())
}

fn write_manifest<T: Serialize>(
    common: &CommonArgs,
    command: &str,
    profile: &ReceiverProfile,
    dataset: Option<&DatasetEntry>,
    report: &T,
) -> Result<()> {
    let Some(out_dir) = &common.out else {
        return Ok(());
    };
    fs::create_dir_all(out_dir)?;

    let config_hash = hash_config(common.config.as_ref(), profile)?;
    let git_hash = git_hash().unwrap_or_else(|| "unknown".to_string());
    let manifest = RunManifest {
        command: command.to_string(),
        timestamp_unix_ms: SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_millis(),
        git_hash,
        config_hash,
        dataset_id: dataset.map(|d| d.id.clone()),
        build_profile: std::env::var("PROFILE").unwrap_or_else(|_| "dev".to_string()),
        cpu_features: cpu_features(),
        results: serde_json::to_value(report)?,
    };
    let path = out_dir.join("run.json");
    fs::write(&path, serde_json::to_string_pretty(&manifest)?)?;
    Ok(())
}

fn load_frame(
    path: &Path,
    config: &ReceiverConfig,
    offset_bytes: u64,
    sidecar: Option<&SidecarSpec>,
) -> Result<SamplesFrame> {
    let samples_per_code = samples_per_code(
        sidecar
            .map(|s| s.sample_rate_hz)
            .unwrap_or(config.sampling_freq_hz),
        config.code_freq_basis_hz,
        config.code_length,
    );
    let offset = sidecar.map(|s| s.offset_bytes).unwrap_or(offset_bytes);
    let mut source = FileSamples::open(
        path.to_str().unwrap_or_default(),
        offset,
        sidecar
            .map(|s| s.sample_rate_hz)
            .unwrap_or(config.sampling_freq_hz),
    )
    .with_context(|| format!("failed to open {}", path.display()))?;

    let frame = match source.next_frame(samples_per_code)? {
        Some(frame) => frame,
        None => bail!("no samples available in {}", path.display()),
    };
    if frame.len() < samples_per_code {
        bail!(
            "not enough samples: need {samples_per_code}, got {}",
            frame.len()
        );
    }
    Ok(frame)
}

fn write_experiment_run(
    out_dir: &Path,
    idx: usize,
    result: &ExperimentRunResult,
    obs: &[ObsEpoch],
    solutions: &[bijux_gnss_core::NavSolutionEpoch],
) -> Result<()> {
    let run_dir = out_dir.join(format!("run_{idx:03}"));
    fs::create_dir_all(&run_dir)?;
    fs::write(
        run_dir.join("result.json"),
        serde_json::to_string_pretty(result)?,
    )?;
    let schema_path = schema_path("experiment_run.schema.json");
    if schema_path.exists() {
        validate_json_schema(&schema_path, &run_dir.join("result.json"), false)?;
    }

    let mut cn0_lines = Vec::new();
    cn0_lines.push("epoch_idx,constellation,prn,cn0_dbhz".to_string());
    for epoch in obs {
        for sat in &epoch.sats {
            cn0_lines.push(format!(
                "{},{:?},{},{}",
                epoch.epoch_idx,
                sat.signal_id.sat.constellation,
                sat.signal_id.sat.prn,
                sat.cn0_dbhz
            ));
        }
    }
    fs::write(run_dir.join("cn0.csv"), cn0_lines.join("\n"))?;
    validate_csv_schema(
        &run_dir.join("cn0.csv"),
        "epoch_idx,constellation,prn,cn0_dbhz",
        &[CsvType::U64, CsvType::Str, CsvType::U8, CsvType::F64],
    )?;

    let mut residual_lines = Vec::new();
    residual_lines.push("epoch_idx,constellation,prn,residual_m,rejected".to_string());
    for sol in solutions {
        for res in &sol.residuals {
            residual_lines.push(format!(
                "{},{:?},{},{},{}",
                sol.epoch.index, res.sat.constellation, res.sat.prn, res.residual_m, res.rejected
            ));
        }
    }
    fs::write(run_dir.join("residuals.csv"), residual_lines.join("\n"))?;
    validate_csv_schema(
        &run_dir.join("residuals.csv"),
        "epoch_idx,constellation,prn,residual_m,rejected",
        &[
            CsvType::U64,
            CsvType::Str,
            CsvType::U8,
            CsvType::F64,
            CsvType::Bool,
        ],
    )?;

    let mut ambiguity_lines = Vec::new();
    ambiguity_lines.push("epoch_idx,constellation,prn,carrier_phase_cycles".to_string());
    for epoch in obs {
        for sat in &epoch.sats {
            ambiguity_lines.push(format!(
                "{},{:?},{},{}",
                epoch.epoch_idx,
                sat.signal_id.sat.constellation,
                sat.signal_id.sat.prn,
                sat.carrier_phase_cycles
            ));
        }
    }
    fs::write(run_dir.join("ambiguities.csv"), ambiguity_lines.join("\n"))?;
    validate_csv_schema(
        &run_dir.join("ambiguities.csv"),
        "epoch_idx,constellation,prn,carrier_phase_cycles",
        &[CsvType::U64, CsvType::Str, CsvType::U8, CsvType::F64],
    )?;
    Ok(())
}

fn write_track_timeseries(common: &CommonArgs, report: &TrackingReport) -> Result<()> {
    let Some(out_dir) = &common.out else {
        return Ok(());
    };
    fs::create_dir_all(out_dir)?;
    let path = out_dir.join("track.jsonl");
    let mut lines = Vec::new();
    for epoch in &report.epochs {
        let line = serde_json::to_string(epoch)?;
        lines.push(line);
    }
    fs::write(&path, lines.join("\n"))?;
    Ok(())
}

fn write_obs_timeseries(
    common: &CommonArgs,
    config: &ReceiverConfig,
    tracks: &[bijux_gnss_receiver::tracking::TrackingResult],
    hatch_window: u32,
) -> Result<()> {
    let Some(out_dir) = &common.out else {
        return Ok(());
    };
    fs::create_dir_all(out_dir)?;
    let obs = bijux_gnss_receiver::observations::observations_from_tracking_results(
        config,
        tracks,
        hatch_window,
    );
    let path = out_dir.join("obs.jsonl");
    let mut lines = Vec::new();
    for epoch in &obs {
        let line = serde_json::to_string(epoch)?;
        lines.push(line);
    }
    fs::write(&path, lines.join("\n"))?;
    validate_jsonl_schema(&schema_path("obs_epoch.schema.json"), &path, false)?;
    let combos = bijux_gnss_receiver::combinations::combinations_from_obs_epochs(
        &obs,
        bijux_gnss_core::SignalBand::L1,
        bijux_gnss_core::SignalBand::L2,
    );
    if !combos.is_empty() {
        let combo_path = out_dir.join("combinations.jsonl");
        let mut combo_lines = Vec::new();
        for combo in combos {
            combo_lines.push(serde_json::to_string(&combo)?);
        }
        fs::write(&combo_path, combo_lines.join("\n"))?;
        validate_jsonl_schema(&schema_path("combinations.schema.json"), &combo_path, false)?;
    }
    Ok(())
}

fn read_tracking_dump(path: &Path) -> Result<Vec<TrackingRow>> {
    let data = fs::read_to_string(path)?;
    let mut rows = Vec::new();
    for line in data.lines() {
        if line.trim().is_empty() {
            continue;
        }
        let row: TrackingRow = serde_json::from_str(line)?;
        rows.push(row);
    }
    Ok(rows)
}

fn read_obs_epochs(path: &Path) -> Result<Vec<ObsEpoch>> {
    let data = fs::read_to_string(path)?;
    let mut epochs = Vec::new();
    for line in data.lines() {
        if line.trim().is_empty() {
            continue;
        }
        let epoch: ObsEpoch = serde_json::from_str(line)?;
        epochs.push(epoch);
    }
    validate_obs_epochs(&epochs).map_err(|err| eyre!("obs epoch validation failed: {err}"))?;
    Ok(epochs)
}

fn read_ephemeris(path: &Path) -> Result<Vec<GpsEphemeris>> {
    let data = fs::read_to_string(path)?;
    let ephs: Vec<GpsEphemeris> = serde_json::from_str(&data)?;
    Ok(ephs)
}

fn read_reference_epochs(path: &Path) -> Result<Vec<ValidationReferenceEpoch>> {
    let data = fs::read_to_string(path)?;
    let mut epochs = Vec::new();
    for line in data.lines() {
        if line.trim().is_empty() {
            continue;
        }
        let epoch: ValidationReferenceEpoch = serde_json::from_str(line)?;
        epochs.push(epoch);
    }
    Ok(epochs)
}

fn write_ephemeris(common: &CommonArgs, ephs: &[GpsEphemeris]) -> Result<()> {
    let Some(out_dir) = &common.out else {
        return Ok(());
    };
    fs::create_dir_all(out_dir)?;
    let path = out_dir.join("ephemeris.json");
    let data = serde_json::to_string_pretty(ephs)?;
    fs::write(&path, data)?;
    validate_json_schema(&schema_path("gps_ephemeris.schema.json"), &path, false)?;
    Ok(())
}
