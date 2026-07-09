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
        return Err(classified_error(
            CliErrorClass::OperatorMisconfiguration,
            "dataset id is required (use --dataset or --unregistered-dataset)",
        ));
    };
    let registry_path = PathBuf::from("datasets/registry.toml");
    let registry = DatasetRegistry::load(&registry_path)
        .with_context(|| format!("failed to parse {}", registry_path.display()))?;
    let entry = registry.find(id).ok_or_else(|| {
        classified_error(
            CliErrorClass::OperatorMisconfiguration,
            format!("dataset not found: {id}"),
        )
    })?;
    Ok(Some(entry))
}

fn resolve_input_file(file: Option<&PathBuf>, dataset: Option<&DatasetEntry>) -> Result<PathBuf> {
    if let Some(file) = file {
        return Ok(file.clone());
    }
    if let Some(dataset) = dataset {
        return Ok(PathBuf::from(&dataset.path));
    }
    Err(classified_error(
        CliErrorClass::OperatorMisconfiguration,
        "no input file provided; use --file or --dataset",
    ))
}

fn resolve_raw_iq_metadata(
    common: &CommonArgs,
    dataset: Option<&DatasetEntry>,
) -> Result<RawIqMetadata> {
    let metadata =
        bijux_gnss_infra::api::resolve_raw_iq_metadata(dataset, common.sidecar.as_deref())?;
    validate_sidecar_schema(&metadata)?;
    Ok(metadata)
}

fn apply_raw_iq_metadata(
    profile: &mut ReceiverConfig,
    metadata: &RawIqMetadata,
    sampling_hz: Option<f64>,
    if_hz: Option<f64>,
) -> Result<()> {
    enforce_locked_capture_value("sample_rate_hz", sampling_hz, metadata.sample_rate_hz)?;
    enforce_locked_capture_value("intermediate_freq_hz", if_hz, metadata.intermediate_freq_hz)?;
    profile.sample_rate_hz = metadata.sample_rate_hz;
    profile.intermediate_freq_hz = metadata.intermediate_freq_hz;
    if let Some(bits) = metadata.quantization_bits {
        profile.quantization_bits = bits;
    }
    Ok(())
}

fn enforce_locked_capture_value(
    field: &str,
    cli_value: Option<f64>,
    metadata_value: f64,
) -> Result<()> {
    let Some(cli_value) = cli_value else {
        return Ok(());
    };
    if (cli_value - metadata_value).abs() > 1e-9 {
        return Err(classified_error(
            CliErrorClass::OperatorMisconfiguration,
            format!(
                "{field} must come from explicit raw IQ metadata; update the dataset registry or sidecar instead of overriding it on the command line"
            ),
        ));
    }
    Ok(())
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

const TRACKING_DIAGNOSTIC_CODE_PERIODS: usize = 12;

fn load_frame(
    path: &Path,
    config: &ReceiverPipelineConfig,
    metadata: &RawIqMetadata,
) -> Result<SamplesFrame> {
    load_frame_window(path, config, metadata, 1)
}

fn load_tracking_frame(
    path: &Path,
    config: &ReceiverPipelineConfig,
    metadata: &RawIqMetadata,
) -> Result<SamplesFrame> {
    let samples_per_code =
        samples_per_code(metadata.sample_rate_hz, config.code_freq_basis_hz, config.code_length);
    let desired_samples = samples_per_code.saturating_mul(TRACKING_DIAGNOSTIC_CODE_PERIODS);
    let mut source = FileSamples::open_raw_iq(path, metadata.clone())
        .with_context(|| format!("failed to open {}", path.display()))?;

    let mut frame = match source.next_frame(desired_samples)? {
        Some(frame) => frame,
        None => bail!("no samples available in {}", path.display()),
    };
    if frame.len() < samples_per_code {
        bail!("not enough samples: need {samples_per_code}, got {}", frame.len());
    }
    if config.remove_dc_offset {
        bijux_gnss_infra::api::signal::remove_dc_offset_in_place(&mut frame.iq);
    }
    Ok(frame)
}

fn load_frame_window(
    path: &Path,
    config: &ReceiverPipelineConfig,
    metadata: &RawIqMetadata,
    code_periods: usize,
) -> Result<SamplesFrame> {
    let samples_per_code =
        samples_per_code(metadata.sample_rate_hz, config.code_freq_basis_hz, config.code_length);
    let required_samples = samples_per_code.saturating_mul(code_periods.max(1));
    let mut source = FileSamples::open_raw_iq(path, metadata.clone())
        .with_context(|| format!("failed to open {}", path.display()))?;

    let mut frame = match source.next_frame(required_samples)? {
        Some(frame) => frame,
        None => bail!("no samples available in {}", path.display()),
    };
    if frame.len() < required_samples {
        bail!("not enough samples: need {required_samples}, got {}", frame.len());
    }
    if config.remove_dc_offset {
        bijux_gnss_infra::api::signal::remove_dc_offset_in_place(&mut frame.iq);
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
    fs::write(run_dir.join("result.json"), serde_json::to_string_pretty(result)?)?;
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
                sol.epoch.index, res.sat.constellation, res.sat.prn, res.residual_m.0, res.rejected
            ));
        }
    }
    fs::write(run_dir.join("residuals.csv"), residual_lines.join("\n"))?;
    validate_csv_schema(
        &run_dir.join("residuals.csv"),
        "epoch_idx,constellation,prn,residual_m,rejected",
        &[CsvType::U64, CsvType::Str, CsvType::U8, CsvType::F64, CsvType::Bool],
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
    source_time: ReceiverSampleTraceOutput,
    source_observation_epoch_id: String,
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

#[derive(Debug, Serialize)]
struct ReceiverSampleTraceOutput {
    sample_index: u64,
    sample_rate_hz: f64,
    receiver_time_s: f64,
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
            source_time: ReceiverSampleTraceOutput {
                sample_index: sol.source_time.sample_index,
                sample_rate_hz: sol.source_time.sample_rate_hz,
                receiver_time_s: sol.source_time.receiver_time_s.0,
            },
            source_observation_epoch_id: sol.source_observation_epoch_id.clone(),
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
        validate_jsonl_schema(&schema, &path, false)?;
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
                epoch: bijux_gnss_infra::api::core::Epoch { index: epoch.epoch_idx },
                sample_index: epoch.sample_index,
                source_time: bijux_gnss_infra::api::core::ReceiverSampleTrace::from_sample_index(
                    epoch.sample_index,
                    profile.sample_rate_hz,
                ),
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
    validate_jsonl_schema(&schema_path("track_epoch_v1.schema.json"), &path, false)?;
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
    let obs_report = bijux_gnss_infra::api::receiver::observations_from_tracking_results(
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
        let wrapped = ObsEpochV1 { header: header.clone(), payload: epoch.clone() };
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
                bail!("unsupported obs schema_version {}", wrapped.header.schema_version);
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
            bail!("unsupported ephemeris schema_version {}", wrapped.header.schema_version);
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
            let latitude_deg =
                parts.get(2).and_then(|v| v.trim().parse::<f64>().ok()).unwrap_or(0.0);
            let longitude_deg =
                parts.get(3).and_then(|v| v.trim().parse::<f64>().ok()).unwrap_or(0.0);
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
            bail!("unsupported nav schema_version {}", wrapped.header.schema_version);
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
    let wrapped = GpsEphemerisV1 { header, payload: ephs.to_vec() };
    let data = serde_json::to_string_pretty(&wrapped)?;
    fs::write(&path, data)?;
    validate_json_schema(&schema_path("gps_ephemeris_v1.schema.json"), &path, false)?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::{apply_raw_iq_metadata, enforce_locked_capture_value, load_frame};
    use crate::RawIqMetadata;
    use crate::{ReceiverConfig, ReceiverPipelineConfig};
    use bijux_gnss_infra::api::core::{
        Epoch, Meters, NavLifecycleState, NavSolutionEpoch, NavUncertaintyClass,
        ReceiverSampleTrace, Seconds, SolutionStatus, SolutionValidity,
        NAV_OUTPUT_STABILITY_SIGNATURE_VERSION, NAV_SOLUTION_MODEL_VERSION,
    };
    use std::fs;
    use std::path::PathBuf;
    use std::time::{SystemTime, UNIX_EPOCH};

    #[test]
    fn locked_capture_value_rejects_drift() {
        let err = enforce_locked_capture_value("sample_rate_hz", Some(4_092_000.0), 5_000_000.0)
            .expect_err("mismatch must fail");
        assert!(err.to_string().contains("sample_rate_hz"));
    }

    #[test]
    fn raw_iq_metadata_updates_receiver_profile() {
        let mut profile = ReceiverConfig::default();
        let metadata = RawIqMetadata {
            format: bijux_gnss_infra::api::signal::IqSampleFormat::Iq16Le,
            sample_rate_hz: 5_000_000.0,
            intermediate_freq_hz: 250_000.0,
            capture_start_utc: "2026-07-09T00:00:00Z".to_string(),
            offset_bytes: 64,
            quantization_bits: Some(16),
            notes: None,
        };

        apply_raw_iq_metadata(&mut profile, &metadata, Some(5_000_000.0), Some(250_000.0))
            .expect("apply raw iq metadata");

        assert_eq!(profile.sample_rate_hz, metadata.sample_rate_hz);
        assert_eq!(profile.intermediate_freq_hz, metadata.intermediate_freq_hz);
        assert_eq!(profile.quantization_bits, 16);
    }

    fn temp_file_path(name: &str) -> PathBuf {
        let nanos = SystemTime::now().duration_since(UNIX_EPOCH).expect("unix epoch").as_nanos();
        std::env::temp_dir().join(format!("bijux_{}_{}_{}.iq", name, std::process::id(), nanos))
    }

    #[test]
    fn load_frame_removes_dc_offset_when_enabled() {
        let path = temp_file_path("load_frame_dc_removal");
        let sample_count = 4_092usize;
        let mut raw = Vec::with_capacity(sample_count * 2);
        for _ in 0..sample_count {
            raw.push(64u8);
            raw.push(0u8);
        }
        fs::write(&path, raw).expect("write iq8 fixture");

        let metadata = RawIqMetadata {
            format: bijux_gnss_infra::api::signal::IqSampleFormat::Iq8,
            sample_rate_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            capture_start_utc: "2026-07-09T00:00:00Z".to_string(),
            offset_bytes: 0,
            quantization_bits: Some(8),
            notes: None,
        };
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: metadata.sample_rate_hz,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            remove_dc_offset: true,
            ..ReceiverPipelineConfig::default()
        };

        let frame = load_frame(&path, &config, &metadata).expect("load frame");
        let metrics = bijux_gnss_infra::api::signal::measure_iq_front_end_metrics(&frame.iq);

        assert!(metrics.i_mean.abs() < 1e-6, "i_mean={}", metrics.i_mean);
        assert!(metrics.q_mean.abs() < 1e-6, "q_mean={}", metrics.q_mean);
        assert_eq!(metrics.i_power, 0.0);
        assert_eq!(metrics.q_power, 0.0);
        assert_eq!(metrics.iq_power_ratio, 1.0);
        assert!(!metrics.power_imbalance_warning);
        assert_eq!(metrics.quadrature_error_deg, None);
        assert!(!metrics.quadrature_error_warning);
        assert_eq!(metrics.centered_rms, 0.0);
        assert!(metrics.zero_signal_detected);
        assert!(metrics
            .zero_signal_reason
            .as_deref()
            .expect("zero_signal_reason")
            .contains("no varying signal energy"));
        assert!(!metrics.precision_claims_allowed);
        assert!(metrics
            .precision_claims_refused_reason
            .as_deref()
            .expect("precision_claims_refused_reason")
            .contains("no varying signal energy"));

        fs::remove_file(&path).expect("remove iq8 fixture");
    }

    #[test]
    fn load_tracking_frame_reads_multiple_code_periods() {
        let path = temp_file_path("load_tracking_frame_window");
        let sample_count = 4_092usize * super::TRACKING_DIAGNOSTIC_CODE_PERIODS;
        let mut raw = Vec::with_capacity(sample_count * 2);
        for _ in 0..sample_count {
            raw.push(16u8);
            raw.push(0u8);
        }
        fs::write(&path, raw).expect("write iq8 fixture");

        let metadata = RawIqMetadata {
            format: bijux_gnss_infra::api::signal::IqSampleFormat::Iq8,
            sample_rate_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            capture_start_utc: "2026-07-09T00:00:00Z".to_string(),
            offset_bytes: 0,
            quantization_bits: Some(8),
            notes: None,
        };
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: metadata.sample_rate_hz,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            ..ReceiverPipelineConfig::default()
        };

        let frame = super::load_tracking_frame(&path, &config, &metadata)
            .expect("load tracking frame");
        assert_eq!(frame.len(), sample_count);

        fs::remove_file(&path).expect("remove iq8 fixture");
    }

    #[test]
    fn nav_solution_output_preserves_source_trace() {
        let out_dir = std::env::temp_dir().join(format!(
            "bijux_nav_solution_output_{}_{}",
            std::process::id(),
            SystemTime::now().duration_since(UNIX_EPOCH).expect("unix epoch").as_nanos()
        ));
        fs::create_dir_all(&out_dir).expect("create output directory");
        let solution = NavSolutionEpoch {
            epoch: Epoch { index: 12 },
            t_rx_s: Seconds(1.5),
            source_time: ReceiverSampleTrace::from_sample_index(6_138, 4_092_000.0),
            ecef_x_m: Meters(1.0),
            ecef_y_m: Meters(2.0),
            ecef_z_m: Meters(3.0),
            latitude_deg: 60.0,
            longitude_deg: 18.0,
            altitude_m: Meters(4.0),
            clock_bias_s: Seconds(0.001),
            clock_drift_s_per_s: 0.0,
            pdop: 1.0,
            rms_m: Meters(2.0),
            status: SolutionStatus::Converged,
            quality: SolutionStatus::Converged.quality_flag(),
            validity: SolutionValidity::Stable,
            valid: true,
            processing_ms: None,
            residuals: Vec::new(),
            health: Vec::new(),
            isb: Vec::new(),
            sigma_h_m: Some(Meters(0.5)),
            sigma_v_m: Some(Meters(0.8)),
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
            artifact_id: "nav-epoch-0000000012-source".to_string(),
            source_observation_epoch_id: "epoch-0000000012-sample-000000006138".to_string(),
            explain_decision: "accepted".to_string(),
            explain_reasons: vec!["navigation_solution_usable".to_string()],
            provenance: None,
            sat_count: 4,
            used_sat_count: 4,
            rejected_sat_count: 0,
            hdop: None,
            vdop: None,
            gdop: None,
            stability_signature: "navsig:v1:test".to_string(),
            stability_signature_version: NAV_OUTPUT_STABILITY_SIGNATURE_VERSION,
        };

        super::write_nav_solution_outputs(&out_dir, &[solution]).expect("write nav solution");

        let path = out_dir.join("nav_solution.jsonl");
        let line = fs::read_to_string(&path).expect("read nav solution");
        let payload: serde_json::Value = serde_json::from_str(line.lines().next().unwrap_or(""))
            .expect("parse nav solution line");
        assert_eq!(payload["source_time"]["sample_index"], 6_138);
        assert_eq!(payload["source_time"]["sample_rate_hz"], 4_092_000.0);
        assert_eq!(payload["source_time"]["receiver_time_s"], 0.0015);
        assert_eq!(
            payload["source_observation_epoch_id"],
            "epoch-0000000012-sample-000000006138"
        );

        fs::remove_file(&path).expect("remove nav solution output");
        fs::remove_dir(&out_dir).expect("remove output directory");
    }
}
