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
    if profile.schema_version.0 != SchemaVersion::CURRENT.0 {
        return Err(eyre!(
            "unsupported schema_version {}, expected {}",
            profile.schema_version.0,
            SchemaVersion::CURRENT.0
        ));
    }
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
    let summary = serde_json::to_value(report)?;
    let dataset = load_dataset(common).ok().flatten();
    let run_dir = run_dir(common, command, dataset.as_ref())?;
    let summary_path = run_dir.join("summary.json");
    fs::write(&summary_path, serde_json::to_string_pretty(&summary)?)?;

    match common.report {
        ReportFormat::Json => {
            let json = serde_json::to_string_pretty(report)?;
            let report_path = run_dir.join(format!("{command}_report.json"));
            fs::write(&report_path, json)?;
            println!("wrote {}", report_path.display());
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
    let run_dir = run_dir(common, command, dataset)?;
    let config_hash = hash_config(common.config.as_ref(), profile)?;
    let config_snapshot = common
        .config
        .as_ref()
        .and_then(|path| fs::read_to_string(path).ok());
    let git_hash = git_hash().unwrap_or_else(|| "unknown".to_string());
    let manifest = RunManifest {
        command: command.to_string(),
        timestamp_unix_ms: now_unix_ms(),
        git_hash,
        git_dirty: git_dirty(),
        config_hash,
        config_snapshot,
        dataset_id: dataset.map(|d| d.id.clone()),
        dataset_metadata: dataset.cloned(),
        build_profile: std::env::var("PROFILE").unwrap_or_else(|_| "dev".to_string()),
        cpu_features: cpu_features(),
        toolchain: std::env::var("RUSTC_VERSION").unwrap_or_else(|_| "unknown".to_string()),
        features: enabled_features(),
        summary: serde_json::to_value(report)?,
    };
    let path = run_dir.join("manifest.json");
    fs::write(&path, serde_json::to_string_pretty(&manifest)?)?;
    append_run_index(&run_dir, &manifest)?;
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
                sol.epoch.index,
                res.sat.constellation,
                res.sat.prn,
                res.residual_m.0,
                res.rejected
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
                sat.carrier_phase_cycles.0
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

fn write_track_timeseries(
    common: &CommonArgs,
    report: &TrackingReport,
    profile: &ReceiverProfile,
    dataset: Option<&DatasetEntry>,
) -> Result<()> {
    let out_dir = artifacts_dir(common, "track", dataset)?;
    let header = artifact_header(common, profile, dataset)?;
    let path = out_dir.join("track.jsonl");
    let mut lines = Vec::new();
    for epoch in &report.epochs {
        let wrapped = TrackEpochV1 {
            header: header.clone(),
            epoch: TrackEpoch {
                epoch: bijux_gnss_core::Epoch {
                    index: epoch.epoch_idx,
                },
                sample_index: epoch.sample_index,
                sat: epoch.sat,
                prompt_i: epoch.prompt_i,
                prompt_q: epoch.prompt_q,
                carrier_hz: bijux_gnss_core::Hertz(epoch.carrier_hz),
                code_rate_hz: bijux_gnss_core::Hertz(epoch.code_rate_hz),
                code_phase_samples: bijux_gnss_core::Chips(epoch.code_phase_samples),
                lock: epoch.lock,
                cn0_dbhz: epoch.cn0_dbhz,
                pll_lock: epoch.pll_lock,
                dll_lock: epoch.dll_lock,
                fll_lock: epoch.fll_lock,
                cycle_slip: epoch.cycle_slip,
                nav_bit_lock: epoch.nav_bit_lock,
                dll_err: epoch.dll_err,
                pll_err: epoch.pll_err,
                fll_err: epoch.fll_err,
            },
        };
        let line = serde_json::to_string(&wrapped)?;
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
    profile: &ReceiverProfile,
    dataset: Option<&DatasetEntry>,
) -> Result<()> {
    let out_dir = artifacts_dir(common, "track", dataset)?;
    let header = artifact_header(common, profile, dataset)?;
    let mut obs = bijux_gnss_receiver::observations::observations_from_tracking_results(
        config,
        tracks,
        hatch_window,
    );
    let path = out_dir.join("obs.jsonl");
    let mut lines = Vec::new();
    for epoch in &mut obs {
        if common.deterministic {
            sort_obs_sats(epoch);
        }
        let wrapped = ObsEpochV1 {
            header: header.clone(),
            epoch: epoch.clone(),
        };
        let line = serde_json::to_string(&wrapped)?;
        lines.push(line);
    }
    fs::write(&path, lines.join("\n"))?;
    validate_jsonl_schema(&schema_path("obs_epoch_v1.schema.json"), &path, false)?;
    let combos = bijux_gnss_nav::combinations_from_obs_epochs(
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
        if line.contains("\"header\"") {
            let wrapped: ObsEpochV1 = serde_json::from_str(line)?;
            if !ArtifactReadPolicy::is_supported(wrapped.header.schema_version) {
                bail!(
                    "unsupported obs schema_version {}",
                    wrapped.header.schema_version
                );
            }
            epochs.push(wrapped.epoch);
        } else {
            let epoch: ObsEpoch = serde_json::from_str(line)?;
            epochs.push(epoch);
        }
    }
    validate_obs_epochs(&epochs).map_err(|err| eyre!("obs epoch validation failed: {err}"))?;
    Ok(epochs)
}

fn read_ephemeris(path: &Path) -> Result<Vec<GpsEphemeris>> {
    let data = fs::read_to_string(path)?;
    if data.contains("\"header\"") {
        let wrapped: GpsEphemerisV1 = serde_json::from_str(&data)?;
        if !ArtifactReadPolicy::is_supported(wrapped.header.schema_version) {
            bail!(
                "unsupported ephemeris schema_version {}",
                wrapped.header.schema_version
            );
        }
        Ok(wrapped.ephemerides)
    } else {
        let ephs: Vec<GpsEphemeris> = serde_json::from_str(&data)?;
        Ok(ephs)
    }
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

fn write_ephemeris(
    common: &CommonArgs,
    ephs: &[GpsEphemeris],
    profile: &ReceiverProfile,
    dataset: Option<&DatasetEntry>,
) -> Result<()> {
    let out_dir = artifacts_dir(common, "nav", dataset)?;
    let header = artifact_header(common, profile, dataset)?;
    let path = out_dir.join("ephemeris.json");
    let wrapped = GpsEphemerisV1 {
        header,
        ephemerides: ephs.to_vec(),
    };
    let data = serde_json::to_string_pretty(&wrapped)?;
    fs::write(&path, data)?;
    validate_json_schema(&schema_path("gps_ephemeris_v1.schema.json"), &path, false)?;
    Ok(())
}
