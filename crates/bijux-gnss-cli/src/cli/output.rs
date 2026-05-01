fn load_config(common: &CommonArgs) -> Result<ReceiverConfig> {
    match &common.config {
        Some(path) => load_config_from_path(path),
        None => Ok(ReceiverConfig::default()),
    }
}

fn load_config_from_path(path: &Path) -> Result<ReceiverConfig> {
    let contents = fs::read_to_string(path)
        .with_context(|| format!("failed to read config {}", path.display()))?;
    let profile: ReceiverConfig = toml::from_str(&contents)
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
        if common.unregistered_dataset {
            return Ok(None);
        }
        bail!("dataset id is required (use --dataset or --unregistered-dataset)");
    };
    let registry_path = PathBuf::from("datasets/registry.toml");
    let registry = DatasetRegistry::load(&registry_path)
        .with_context(|| format!("failed to parse {}", registry_path.display()))?;
    let entry = registry
        .find(id)
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

fn load_frame(
    path: &Path,
    config: &ReceiverPipelineConfig,
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
    solutions: &[bijux_gnss_infra::api::core::NavSolutionEpoch],
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

#[derive(Debug, Serialize)]
struct NavSolutionOutput {
    schema_version: u32,
    epoch_idx: u64,
    t_rx_s: f64,
    ecef_m: [f64; 3],
    llh_deg: [f64; 3],
    velocity_mps: Option<[f64; 3]>,
    clock_bias_s: f64,
    clock_drift_s_per_s: f64,
    covariance: NavSolutionCovariance,
    fix_quality: bijux_gnss_infra::api::core::NavQualityFlag,
    validity: bijux_gnss_infra::api::core::SolutionValidity,
    rejected_measurements: usize,
}

#[derive(Debug, Serialize)]
struct NavSolutionCovariance {
    sigma_h_m: Option<f64>,
    sigma_v_m: Option<f64>,
    covariance_xyz_m2: Option<[f64; 3]>,
}

fn write_nav_solution_outputs(
    out_dir: &Path,
    solutions: &[bijux_gnss_infra::api::core::NavSolutionEpoch],
) -> Result<()> {
    let mut lines = Vec::new();
    for sol in solutions {
        let output = NavSolutionOutput {
            schema_version: 1,
            epoch_idx: sol.epoch.index,
            t_rx_s: sol.t_rx_s.0,
            ecef_m: [sol.ecef_x_m.0, sol.ecef_y_m.0, sol.ecef_z_m.0],
            llh_deg: [sol.latitude_deg, sol.longitude_deg, sol.altitude_m.0],
            velocity_mps: None,
            clock_bias_s: sol.clock_bias_s.0,
            clock_drift_s_per_s: sol.clock_drift_s_per_s,
            covariance: NavSolutionCovariance {
                sigma_h_m: sol.sigma_h_m.map(|m| m.0),
                sigma_v_m: sol.sigma_v_m.map(|m| m.0),
                covariance_xyz_m2: None,
            },
            fix_quality: sol.quality,
            validity: sol.validity,
            rejected_measurements: sol.residuals.iter().filter(|r| r.rejected).count(),
        };
        lines.push(serde_json::to_string(&output)?);
    }
    let path = out_dir.join("nav_solution.jsonl");
    fs::write(&path, lines.join("\n"))?;
    let schema = schema_path("nav_solution.schema.json");
    if schema.exists() {
        validate_json_schema(&schema, &path, false)?;
    }
    Ok(())
}

fn write_track_timeseries(
    common: &CommonArgs,
    report: &TrackingReport,
    profile: &ReceiverConfig,
    dataset: Option<&DatasetEntry>,
) -> Result<()> {
    let out_dir = artifacts_dir(common, "track", dataset)?;
    let header = artifact_header(common, profile, dataset)?;
    let path = out_dir.join("track.jsonl");
    let mut lines = Vec::new();
    for epoch in &report.epochs {
        let wrapped = TrackEpochV1 {
            header: header.clone(),
            payload: TrackEpoch {
                epoch: bijux_gnss_infra::api::core::Epoch {
                    index: epoch.epoch_idx,
                },
                sample_index: epoch.sample_index,
                sat: epoch.sat,
                prompt_i: epoch.prompt_i,
                prompt_q: epoch.prompt_q,
                early_i: epoch.early_i,
                early_q: epoch.early_q,
                late_i: epoch.late_i,
                late_q: epoch.late_q,
                carrier_hz: bijux_gnss_infra::api::core::Hertz(epoch.carrier_hz),
                code_rate_hz: bijux_gnss_infra::api::core::Hertz(epoch.code_rate_hz),
                code_phase_samples: bijux_gnss_infra::api::core::Chips(epoch.code_phase_samples),
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
                anti_false_lock: epoch.anti_false_lock,
                cycle_slip_reason: epoch.cycle_slip_reason.clone(),
                lock_state: epoch.lock_state.clone(),
                lock_state_reason: epoch.lock_state_reason.clone(),
                channel_id: None,
                channel_uid: String::new(),
                tracking_provenance: String::new(),
                tracking_assumptions: None,
                processing_ms: None,
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
    config: &ReceiverPipelineConfig,
    tracks: &[bijux_gnss_infra::api::receiver::TrackingResult],
    hatch_window: u32,
    profile: &ReceiverConfig,
    dataset: Option<&DatasetEntry>,
) -> Result<()> {
    let out_dir = artifacts_dir(common, "track", dataset)?;
    let header = artifact_header(common, profile, dataset)?;
    let runtime = runtime_config_from_env(common, None);
    let obs_report =
        bijux_gnss_infra::api::receiver::observations_from_tracking_results(
            config,
            tracks,
            hatch_window,
        );
    let mut obs = obs_report.output;
    for event in obs_report.events {
        runtime.logger.event(&event);
    }
    let path = out_dir.join("obs.jsonl");
    let mut lines = Vec::new();
    let mut timing_lines = Vec::new();
    for epoch in &mut obs {
        if common.deterministic {
            sort_obs_sats(epoch);
        }
        let wrapped = ObsEpochV1 {
            header: header.clone(),
            payload: epoch.clone(),
        };
        let line = serde_json::to_string(&wrapped)?;
        lines.push(line);
        if let Some(ms) = epoch.processing_ms {
            timing_lines.push(serde_json::to_string(&serde_json::json!({
                "epoch_idx": epoch.epoch_idx,
                "stage": "observations",
                "processing_ms": ms
            }))?);
        }
    }
    fs::write(&path, lines.join("\n"))?;
    validate_jsonl_schema(&schema_path("obs_epoch_v1.schema.json"), &path, false)?;
    if !timing_lines.is_empty() {
        let timing_path = out_dir.join("timing_obs.jsonl");
        fs::write(&timing_path, timing_lines.join("\n"))?;
    }
    let combos = bijux_gnss_infra::api::nav::combinations_from_obs_epochs(
        &obs,
        bijux_gnss_infra::api::core::SignalBand::L1,
        bijux_gnss_infra::api::core::SignalBand::L2,
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
            if wrapped.header.schema_version != ArtifactReadPolicy::LATEST {
                bail!(
                    "unsupported obs schema_version {}",
                    wrapped.header.schema_version
                );
            }
            epochs.push(wrapped.payload);
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
        if wrapped.header.schema_version != ArtifactReadPolicy::LATEST {
            bail!(
                "unsupported ephemeris schema_version {}",
                wrapped.header.schema_version
            );
        }
        Ok(wrapped.payload)
    } else {
        let ephs: Vec<GpsEphemeris> = serde_json::from_str(&data)?;
        Ok(ephs)
    }
}

fn read_reference_epochs(path: &Path) -> Result<Vec<ValidationReferenceEpoch>> {
    let ext = path.extension().and_then(|s| s.to_str()).unwrap_or("");
    let data = fs::read_to_string(path)?;
    let mut epochs = Vec::new();
    if ext.eq_ignore_ascii_case("csv") {
        for (idx, line) in data.lines().enumerate() {
            if idx == 0 && line.contains("epoch_idx") {
                continue;
            }
            if line.trim().is_empty() {
                continue;
            }
            let parts: Vec<&str> = line.split(',').collect();
            if parts.len() < 4 {
                bail!("invalid reference csv row: {line}");
            }
            let epoch_idx = parts[0].trim().parse::<u64>()?;
            let t_rx_s = parts.get(1).and_then(|v| v.trim().parse::<f64>().ok());
            let latitude_deg = parts.get(2).and_then(|v| v.trim().parse::<f64>().ok()).unwrap_or(0.0);
            let longitude_deg = parts.get(3).and_then(|v| v.trim().parse::<f64>().ok()).unwrap_or(0.0);
            let altitude_m = parts.get(4).and_then(|v| v.trim().parse::<f64>().ok()).unwrap_or(0.0);
            let ecef_x_m = parts.get(5).and_then(|v| v.trim().parse::<f64>().ok());
            let ecef_y_m = parts.get(6).and_then(|v| v.trim().parse::<f64>().ok());
            let ecef_z_m = parts.get(7).and_then(|v| v.trim().parse::<f64>().ok());
            epochs.push(ValidationReferenceEpoch {
                epoch_idx,
                t_rx_s,
                latitude_deg,
                longitude_deg,
                altitude_m,
                ecef_x_m,
                ecef_y_m,
                ecef_z_m,
                vel_x_mps: None,
                vel_y_mps: None,
                vel_z_mps: None,
            });
        }
    } else {
        validate_jsonl_schema(&schema_path("reference_epoch.schema.json"), path, false)?;
        for line in data.lines() {
            if line.trim().is_empty() {
                continue;
            }
            let epoch: ValidationReferenceEpoch = serde_json::from_str(line)?;
            epochs.push(epoch);
        }
    }
    Ok(epochs)
}

fn read_nav_solutions(path: &Path) -> Result<Vec<bijux_gnss_infra::api::core::NavSolutionEpoch>> {
    let data = fs::read_to_string(path)?;
    let mut epochs = Vec::new();
    for line in data.lines() {
        if line.trim().is_empty() {
            continue;
        }
        let wrapped: NavSolutionEpochV1 = serde_json::from_str(line)?;
        if wrapped.header.schema_version != ArtifactReadPolicy::LATEST {
            bail!(
                "unsupported nav schema_version {}",
                wrapped.header.schema_version
            );
        }
        epochs.push(wrapped.payload);
    }
    Ok(epochs)
}

fn write_ephemeris(
    common: &CommonArgs,
    ephs: &[GpsEphemeris],
    profile: &ReceiverConfig,
    dataset: Option<&DatasetEntry>,
) -> Result<()> {
    let out_dir = artifacts_dir(common, "nav", dataset)?;
    let header = artifact_header(common, profile, dataset)?;
    let path = out_dir.join("ephemeris.json");
    let wrapped = GpsEphemerisV1 {
        header,
        payload: ephs.to_vec(),
    };
    let data = serde_json::to_string_pretty(&wrapped)?;
    fs::write(&path, data)?;
    validate_json_schema(&schema_path("gps_ephemeris_v1.schema.json"), &path, false)?;
    Ok(())
}
