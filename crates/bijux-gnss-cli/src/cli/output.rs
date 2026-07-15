include!("output/config_resolution.rs");
include!("output/signal_quality.rs");
include!("output/sample_windows.rs");

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
    clock_bias_m: f64,
    clock_drift_s_per_s: f64,
    dops: NavSolutionDops,
    covariance: NavSolutionCovariance,
    solver_diagnostics: NavSolutionSolverDiagnostics,
    fix_quality: bijux_gnss_infra::api::core::NavQualityFlag,
    validity: bijux_gnss_infra::api::core::SolutionValidity,
    rejected_measurements: usize,
}

#[derive(Debug, Serialize)]
struct NavSolutionDops {
    pdop: f64,
    hdop: Option<f64>,
    vdop: Option<f64>,
    gdop: Option<f64>,
    tdop: Option<f64>,
}

#[derive(Debug, Serialize)]
struct NavSolutionCovariance {
    sigma_h_m: Option<f64>,
    sigma_v_m: Option<f64>,
    covariance_xyz_m2: Option<[f64; 3]>,
}

#[derive(Debug, Serialize)]
struct NavSolutionSolverDiagnostics {
    wls_solver_rank: Option<usize>,
    wls_condition_number: Option<f64>,
    ekf_condition_number: Option<f64>,
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
            clock_bias_m: sol.clock_bias_m.0,
            clock_drift_s_per_s: sol.clock_drift_s_per_s,
            dops: NavSolutionDops {
                pdop: sol.pdop,
                hdop: sol.hdop,
                vdop: sol.vdop,
                gdop: sol.gdop,
                tdop: sol.tdop,
            },
            covariance: NavSolutionCovariance {
                sigma_h_m: sol.sigma_h_m.map(|m| m.0),
                sigma_v_m: sol.sigma_v_m.map(|m| m.0),
                covariance_xyz_m2: None,
            },
            solver_diagnostics: NavSolutionSolverDiagnostics {
                wls_solver_rank: sol.wls_solver_rank,
                wls_condition_number: sol.wls_condition_number,
                ekf_condition_number: sol.ekf_condition_number,
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

fn write_track_timeseries_for_command(
    common: &CommonArgs,
    command: &str,
    report: &TrackingReport,
    profile: &ReceiverConfig,
    dataset: Option<&DatasetEntry>,
) -> Result<()> {
    let out_dir = artifacts_dir(common, command, dataset)?;
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
                signal_band: bijux_gnss_infra::api::core::SignalBand::L1,
                signal_code: bijux_gnss_infra::api::core::SignalCode::Unknown,
                glonass_frequency_channel: None,
                prompt_i: epoch.prompt_i,
                prompt_q: epoch.prompt_q,
                early_i: epoch.early_i,
                early_q: epoch.early_q,
                late_i: epoch.late_i,
                late_q: epoch.late_q,
                carrier_hz: bijux_gnss_infra::api::core::Hertz(epoch.carrier_hz),
                carrier_phase_cycles: bijux_gnss_infra::api::core::Cycles(
                    epoch.carrier_phase_cycles,
                ),
                code_rate_hz: bijux_gnss_infra::api::core::Hertz(epoch.code_rate_hz),
                code_phase_samples: bijux_gnss_infra::api::core::Chips(epoch.code_phase_samples),
                lock: epoch.lock,
                cn0_dbhz: epoch.cn0_dbhz,
                pll_lock: epoch.pll_lock,
                dll_lock: epoch.dll_lock,
                fll_lock: epoch.fll_lock,
                cycle_slip: epoch.cycle_slip,
                nav_bit_lock: epoch.nav_bit_lock,
                navigation_bit_sign: epoch.navigation_bit_sign,
                transmit_time: None,
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
                signal_delay_alignment: None,
                tracking_uncertainty: None,
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

fn write_track_timeseries(
    common: &CommonArgs,
    report: &TrackingReport,
    profile: &ReceiverConfig,
    dataset: Option<&DatasetEntry>,
) -> Result<()> {
    write_track_timeseries_for_command(common, "track", report, profile, dataset)
}

#[cfg(test)]
fn write_obs_timeseries_for_command(
    common: &CommonArgs,
    command: &str,
    config: &ReceiverPipelineConfig,
    tracks: &[bijux_gnss_infra::api::receiver::TrackingResult],
    hatch_window: u32,
    profile: &ReceiverConfig,
    dataset: Option<&DatasetEntry>,
) -> Result<bijux_gnss_infra::api::receiver::ObservationPipelineArtifacts> {
    let runtime = runtime_config_from_env(common, None);
    let obs_report = bijux_gnss_infra::api::receiver::observation_artifacts_from_tracking_results_with_gps_anchor(
        config,
        dataset
            .and_then(|entry| entry.capture_start_utc.as_deref())
            .and_then(capture_start_gps_time),
        tracks,
        hatch_window,
    );
    for event in obs_report.events {
        runtime.logger.event(&event);
    }
    write_observation_artifacts_for_command(common, command, &obs_report.output, profile, dataset)
}

fn write_observation_artifacts_for_command(
    common: &CommonArgs,
    command: &str,
    observation_artifacts: &bijux_gnss_infra::api::receiver::ObservationPipelineArtifacts,
    profile: &ReceiverConfig,
    dataset: Option<&DatasetEntry>,
) -> Result<bijux_gnss_infra::api::receiver::ObservationPipelineArtifacts> {
    let out_dir = artifacts_dir(common, command, dataset)?;
    let header = artifact_header(common, profile, dataset)?;
    let mut observation_artifacts = observation_artifacts.clone();
    let path = out_dir.join("obs.jsonl");
    let mut lines = Vec::new();
    let mut timing_lines = Vec::new();
    for epoch in &mut observation_artifacts.epochs {
        if common.deterministic {
            sort_obs_sats(epoch);
        }
        let wrapped = ObsEpochV1 { header: header.clone(), payload: epoch.clone() };
        lines.push(serde_json::to_string(&wrapped)?);
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
    let residual_path = out_dir.join("observation_residuals.jsonl");
    let mut residual_lines = Vec::new();
    for residual in &observation_artifacts.residuals {
        let wrapped =
            ObservationResidualEpochV1 { header: header.clone(), payload: residual.clone() };
        residual_lines.push(serde_json::to_string(&wrapped)?);
    }
    fs::write(&residual_path, residual_lines.join("\n"))?;
    validate_jsonl_schema(
        &schema_path("observation_residual_epoch_v1.schema.json"),
        &residual_path,
        false,
    )?;
    let quality_path = out_dir.join("observation_measurement_quality.jsonl");
    let mut quality_lines = Vec::new();
    for quality in &observation_artifacts.measurement_quality {
        let wrapped = ObservationMeasurementQualityEpochV1 {
            header: header.clone(),
            payload: quality.clone(),
        };
        quality_lines.push(serde_json::to_string(&wrapped)?);
    }
    fs::write(&quality_path, quality_lines.join("\n"))?;
    validate_jsonl_schema(
        &schema_path("observation_measurement_quality_epoch_v1.schema.json"),
        &quality_path,
        false,
    )?;
    let mut combos = Vec::new();
    for (band_1, band_2) in supported_observed_dual_frequency_pairs(&observation_artifacts.epochs) {
        combos.extend(bijux_gnss_infra::api::nav::combinations_from_obs_epochs(
            &observation_artifacts.epochs,
            band_1,
            band_2,
        ));
    }
    if !combos.is_empty() {
        let combo_path = out_dir.join("combinations.jsonl");
        let mut combo_lines = Vec::new();
        for combo in combos {
            combo_lines.push(serde_json::to_string(&combo)?);
        }
        fs::write(&combo_path, combo_lines.join("\n"))?;
        validate_jsonl_schema(&schema_path("combinations.schema.json"), &combo_path, false)?;
    }
    write_iono_free_code_artifact(&out_dir, &observation_artifacts.epochs, None)?;
    write_narrow_lane_artifact(&out_dir, &observation_artifacts.epochs)?;
    write_melbourne_wubbena_diagnostics(&out_dir, &observation_artifacts.epochs)?;
    write_carrier_smoothed_code_validation(&out_dir, &observation_artifacts)?;
    Ok(observation_artifacts)
}

fn dual_frequency_pair_observed(obs: &[ObsEpoch], band_1: SignalBand, band_2: SignalBand) -> bool {
    for epoch in obs {
        let mut bands_by_sat = std::collections::BTreeMap::<SatId, (bool, bool)>::new();
        for satellite in &epoch.sats {
            let entry = bands_by_sat.entry(satellite.signal_id.sat).or_insert((false, false));
            if satellite.signal_id.band == band_1 {
                entry.0 = true;
            }
            if satellite.signal_id.band == band_2 {
                entry.1 = true;
            }
        }
        if bands_by_sat.values().any(|(first_seen, second_seen)| *first_seen && *second_seen) {
            return true;
        }
    }
    false
}

fn supported_observed_dual_frequency_pairs(obs: &[ObsEpoch]) -> Vec<(SignalBand, SignalBand)> {
    bijux_gnss_infra::api::signal::supported_dual_frequency_band_pairs()
        .iter()
        .copied()
        .filter(|&(band_1, band_2)| dual_frequency_pair_observed(obs, band_1, band_2))
        .collect()
}

pub(crate) fn write_iono_free_code_artifact(
    out_dir: &Path,
    obs: &[ObsEpoch],
    biases: Option<&dyn CodeBiasProvider>,
) -> Result<()> {
    let mut lines = Vec::new();
    for (band_1, band_2) in supported_observed_dual_frequency_pairs(obs) {
        let observations = bijux_gnss_infra::api::nav::iono_free_code_from_obs_epochs_with_biases(
            obs, band_1, band_2, biases,
        );
        for observation in observations {
            lines.push(serde_json::to_string(&observation)?);
        }
    }

    if lines.is_empty() {
        return Ok(());
    }

    let path = out_dir.join("iono_free_code.jsonl");
    fs::write(&path, lines.join("\n"))?;
    validate_jsonl_schema(&schema_path("iono_free_code.schema.json"), &path, false)?;
    Ok(())
}

pub(crate) fn write_narrow_lane_artifact(out_dir: &Path, obs: &[ObsEpoch]) -> Result<()> {
    let mut lines = Vec::new();
    for (band_1, band_2) in supported_observed_dual_frequency_pairs(obs) {
        let observations =
            bijux_gnss_infra::api::nav::narrow_lane_from_obs_epochs(obs, band_1, band_2);
        for observation in observations {
            lines.push(serde_json::to_string(&observation)?);
        }
    }

    if lines.is_empty() {
        return Ok(());
    }

    let path = out_dir.join("narrow_lane.jsonl");
    fs::write(&path, lines.join("\n"))?;
    validate_jsonl_schema(&schema_path("narrow_lane.schema.json"), &path, false)?;
    Ok(())
}

pub(crate) fn write_melbourne_wubbena_diagnostics(out_dir: &Path, obs: &[ObsEpoch]) -> Result<()> {
    let mut diagnostics = Vec::new();
    for (band_1, band_2) in supported_observed_dual_frequency_pairs(obs) {
        let pair_diagnostics =
            bijux_gnss_infra::api::nav::melbourne_wubbena_diagnostics_from_obs_epochs(
                obs,
                band_1,
                band_2,
                bijux_gnss_infra::api::nav::MelbourneWubbenaThresholds::default(),
            );
        if pair_diagnostics.iter().any(|diagnostic| diagnostic.status == "ok") {
            diagnostics.extend(pair_diagnostics);
        }
    }

    if diagnostics.is_empty() {
        return Ok(());
    }

    let path = out_dir.join("melbourne_wubbena.jsonl");
    let mut lines = Vec::new();
    for diagnostic in diagnostics {
        lines.push(serde_json::to_string(&diagnostic)?);
    }
    fs::write(&path, lines.join("\n"))?;
    validate_jsonl_schema(&schema_path("melbourne_wubbena.schema.json"), &path, false)?;
    Ok(())
}

fn write_carrier_smoothed_code_validation(
    out_dir: &Path,
    observation_artifacts: &bijux_gnss_infra::api::receiver::ObservationPipelineArtifacts,
) -> Result<()> {
    let report = bijux_gnss_infra::api::receiver::validate_carrier_smoothed_code_from_artifacts(
        observation_artifacts,
    );
    let path = out_dir.join("carrier_smoothed_code_validation.json");
    fs::write(&path, serde_json::to_string_pretty(&report)?)?;
    validate_json_schema(
        &schema_path("carrier_smoothed_code_validation.schema.json"),
        &path,
        false,
    )?;
    Ok(())
}

#[cfg(test)]
fn write_obs_timeseries(
    common: &CommonArgs,
    config: &ReceiverPipelineConfig,
    tracks: &[bijux_gnss_infra::api::receiver::TrackingResult],
    hatch_window: u32,
    profile: &ReceiverConfig,
    dataset: Option<&DatasetEntry>,
) -> Result<bijux_gnss_infra::api::receiver::ObservationPipelineArtifacts> {
    write_obs_timeseries_for_command(
        common,
        "track",
        config,
        tracks,
        hatch_window,
        profile,
        dataset,
    )
}

fn write_tracking_timing_for_command(
    common: &CommonArgs,
    command: &str,
    tracks: &[bijux_gnss_infra::api::receiver::TrackingResult],
    dataset: Option<&DatasetEntry>,
) -> Result<()> {
    let timing_path = artifacts_dir(common, command, dataset)?.join("timing.jsonl");
    let mut timing_lines = Vec::new();
    for track in tracks {
        for epoch in &track.epochs {
            if let Some(ms) = epoch.processing_ms {
                timing_lines.push(serde_json::to_string(&serde_json::json!({
                    "epoch_idx": epoch.epoch.index,
                    "stage": "tracking",
                    "processing_ms": ms
                }))?);
            }
        }
    }
    fs::write(&timing_path, timing_lines.join("\n"))?;
    Ok(())
}

#[derive(Debug, Clone, Serialize)]
struct ObservationResidualEpochV1 {
    header: ArtifactHeaderV1,
    payload: bijux_gnss_infra::api::receiver::ObservationResidualEpochReport,
}

#[derive(Debug, Clone, Serialize)]
struct ObservationMeasurementQualityEpochV1 {
    header: ArtifactHeaderV1,
    payload: bijux_gnss_infra::api::receiver::ObservationMeasurementQualityEpochReport,
}

fn read_tracking_dump(path: &Path) -> Result<Vec<TrackingRow>> {
    let data = fs::read_to_string(path)?;
    if let Ok(report) = serde_json::from_str::<TrackingReport>(&data) {
        return Ok(report.epochs);
    }
    let mut rows = Vec::new();
    for line in data.lines() {
        if line.trim().is_empty() {
            continue;
        }
        if line.contains("\"header\"") && line.contains("\"payload\"") {
            let wrapped: TrackEpochV1 = serde_json::from_str(line)?;
            if wrapped.header.schema_version != ArtifactReadPolicy::LATEST {
                bail!("unsupported track schema_version {}", wrapped.header.schema_version);
            }
            rows.push(tracking_row_from_epoch(wrapped.payload));
        } else {
            let row: TrackingRow = serde_json::from_str(line)?;
            rows.push(row);
        }
    }
    Ok(rows)
}

fn tracking_row_from_epoch(epoch: TrackEpoch) -> TrackingRow {
    TrackingRow {
        epoch_idx: epoch.epoch.index,
        sample_index: epoch.sample_index,
        sat: epoch.sat,
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
        dll_err: epoch.dll_err,
        pll_err: epoch.pll_err,
        fll_err: epoch.fll_err,
        anti_false_lock: epoch.anti_false_lock,
        cycle_slip_reason: epoch.cycle_slip_reason,
        lock_state: epoch.lock_state,
        lock_state_reason: epoch.lock_state_reason,
    }
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

#[derive(serde::Deserialize)]
struct NavDecodeEphemerisReportInput {
    sat: SatId,
    #[serde(default)]
    reference_week: Option<u32>,
    #[serde(default)]
    decoded_subframes: Vec<bijux_gnss_infra::api::nav::GpsL1CaLnavDecodedSubframe>,
    #[serde(default)]
    ephemerides: Vec<GpsEphemeris>,
}

fn ephemerides_from_nav_decode_report(
    report: NavDecodeEphemerisReportInput,
) -> Result<Vec<GpsEphemeris>> {
    if !report.decoded_subframes.is_empty() {
        if let Some(reference_week) = report.reference_week {
            let (ephs, _rejections) =
                bijux_gnss_infra::api::nav::ephemerides_from_decoded_gps_l1ca_lnav(
                    report.sat.prn,
                    &report.decoded_subframes,
                    Some(reference_week),
                );
            return Ok(ephs);
        }
        if !report.ephemerides.is_empty() {
            return Ok(report.ephemerides);
        }
        bail!("decoded LNAV report is missing reference_week and embedded ephemerides");
    }

    if !report.ephemerides.is_empty() {
        return Ok(report.ephemerides);
    }

    Ok(Vec::new())
}

fn read_nav_decode_ephemerides(data: &str) -> Result<Option<Vec<GpsEphemeris>>> {
    if !data.contains("\"decoded_subframes\"") || !data.contains("\"sat\"") {
        return Ok(None);
    }

    if data.trim_start().starts_with('[') {
        let reports: Vec<NavDecodeEphemerisReportInput> = serde_json::from_str(data)?;
        let mut ephemerides = Vec::new();
        for report in reports {
            ephemerides.extend(ephemerides_from_nav_decode_report(report)?);
        }
        return Ok(Some(ephemerides));
    }

    let report: NavDecodeEphemerisReportInput = serde_json::from_str(data)?;
    Ok(Some(ephemerides_from_nav_decode_report(report)?))
}

fn read_broadcast_navigation_data(
    path: &Path,
) -> Result<bijux_gnss_infra::api::nav::GpsBroadcastNavigationData> {
    let data = fs::read_to_string(path)?;
    if data.contains("RINEX VERSION / TYPE")
        && (data.contains("NAVIGATION DATA") || data.contains("NAV DATA"))
    {
        return bijux_gnss_infra::api::nav::parse_rinex_broadcast_navigation(&data)
            .map_err(|err| eyre!("RINEX NAV parse failed: {}", err.message));
    }
    if let Some(ephemerides) = read_nav_decode_ephemerides(&data)? {
        return Ok(bijux_gnss_infra::api::nav::GpsBroadcastNavigationData {
            ephemerides,
            klobuchar: None,
        });
    }
    if data.contains("\"header\"") {
        if let Ok(wrapped) = serde_json::from_str::<
            bijux_gnss_infra::api::core::ArtifactV1<
                bijux_gnss_infra::api::nav::GpsBroadcastNavigationData,
            >,
        >(&data)
        {
            if wrapped.header.schema_version != ArtifactReadPolicy::LATEST {
                bail!(
                    "unsupported broadcast navigation schema_version {}",
                    wrapped.header.schema_version
                );
            }
            return Ok(wrapped.payload);
        }
        let wrapped: GpsEphemerisV1 = serde_json::from_str(&data)?;
        if wrapped.header.schema_version != ArtifactReadPolicy::LATEST {
            bail!("unsupported ephemeris schema_version {}", wrapped.header.schema_version);
        }
        return Ok(bijux_gnss_infra::api::nav::GpsBroadcastNavigationData {
            ephemerides: wrapped.payload,
            klobuchar: None,
        });
    }
    if data.contains("\"ephemerides\"") {
        return serde_json::from_str(&data).map_err(Into::into);
    }
    let ephemerides: Vec<GpsEphemeris> = serde_json::from_str(&data)?;
    Ok(bijux_gnss_infra::api::nav::GpsBroadcastNavigationData { ephemerides, klobuchar: None })
}

fn read_ephemeris(path: &Path) -> Result<Vec<GpsEphemeris>> {
    Ok(read_broadcast_navigation_data(path)?.ephemerides)
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
    use super::{
        apply_raw_iq_metadata, enforce_locked_capture_value, load_frame_window,
        read_broadcast_navigation_data, read_ephemeris, read_tracking_dump, DopplerSearchSettings,
        RawIqSignalQualityReport, TrackingReport, TrackingRow,
    };
    use crate::RawIqMetadata;
    use crate::{CommonArgs, ReceiverConfig, ReceiverPipelineConfig, ReportFormat};
    use bijux_gnss_infra::api::core::{
        ArtifactHeaderV1, ArtifactReadPolicy, Chips, Constellation, Cycles, Epoch, Hertz,
        LockFlags, Meters, NavLifecycleState, NavSolutionEpoch, NavUncertaintyClass, ObsEpoch,
        ObsMetadata, ObsSatellite, ObservationEpochDecision, ObservationStatus, ReceiverRole,
        ReceiverSampleTrace, SatId, Seconds, SigId, SignalBand, SignalCode, SignalDelayAlignment,
        SolutionStatus, SolutionValidity, TrackEpoch, TrackEpochV1, TrackingUncertainty,
        GPS_L1_CA_CARRIER_HZ, NAV_OUTPUT_STABILITY_SIGNATURE_VERSION, NAV_SOLUTION_MODEL_VERSION,
    };
    use bijux_gnss_infra::api::nav::{
        write_rinex_broadcast_navigation, write_rinex_nav, BiasSinexProvider,
        GpsBroadcastNavigationData, GpsEphemeris, KlobucharCoefficients,
    };
    use bijux_gnss_signal::api::{
        signal_registry, signal_spec_gps_l1_ca, signal_spec_gps_l2_py,
    };
    use std::fs;
    use std::path::{Path, PathBuf};
    use std::time::{SystemTime, UNIX_EPOCH};

    const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;

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

    #[test]
    fn read_ephemeris_accepts_rinex_navigation_files() {
        let path = std::env::temp_dir().join(format!(
            "bijux_read_ephemeris_rinex_{}_{}.rnx",
            std::process::id(),
            SystemTime::now().duration_since(UNIX_EPOCH).expect("unix epoch").as_nanos()
        ));
        let eph = sample_ephemeris();

        write_rinex_nav(&path, std::slice::from_ref(&eph), true).expect("write rinex nav");
        let parsed = read_ephemeris(&path).expect("read rinex nav");
        fs::remove_file(&path).expect("remove rinex nav");

        assert_eq!(parsed.len(), 1);
        assert_eq!(parsed[0].sat, eph.sat);
        assert_eq!(parsed[0].week, eph.week);
        assert_eq!(parsed[0].iode, eph.iode);
        assert_eq!(parsed[0].iodc, eph.iodc);
        assert!((parsed[0].toe_s - eph.toe_s).abs() < 1.0e-9);
    }

    #[test]
    fn read_ephemeris_accepts_noaa_nav_data_header() {
        let repo_root = Path::new(env!("CARGO_MANIFEST_DIR"))
            .parent()
            .and_then(Path::parent)
            .expect("workspace root");
        let path = repo_root.join("datasets/recorded/gps_l1_2022_03_27_broadcast_nav.22n");

        let parsed = read_ephemeris(&path).expect("read noaa rinex nav");

        assert!(!parsed.is_empty(), "expected public NOAA NAV file to parse");
        assert_eq!(parsed[0].sat.constellation, Constellation::Gps);
    }

    #[test]
    fn read_ephemeris_accepts_nav_decode_reports() {
        let path = std::env::temp_dir().join(format!(
            "bijux_read_ephemeris_nav_decode_{}_{}.json",
            std::process::id(),
            SystemTime::now().duration_since(UNIX_EPOCH).expect("unix epoch").as_nanos()
        ));
        let eph = sample_ephemeris();
        let report = serde_json::json!({
            "sat": eph.sat,
            "reference_week": eph.week,
            "decoded_subframes": [],
            "ephemerides": [eph.clone()]
        });

        fs::write(
            &path,
            serde_json::to_string_pretty(&report).expect("serialize nav decode report"),
        )
        .expect("write nav decode report");
        let parsed = read_ephemeris(&path).expect("read nav decode report");
        fs::remove_file(&path).expect("remove nav decode report");

        assert_eq!(parsed.len(), 1);
        assert_eq!(parsed[0].sat, eph.sat);
        assert_eq!(parsed[0].week, eph.week);
        assert_eq!(parsed[0].iode, eph.iode);
        assert_eq!(parsed[0].iodc, eph.iodc);
    }

    #[test]
    fn read_ephemeris_accepts_nav_decode_report_arrays() {
        let path = std::env::temp_dir().join(format!(
            "bijux_read_ephemeris_nav_decode_array_{}_{}.json",
            std::process::id(),
            SystemTime::now().duration_since(UNIX_EPOCH).expect("unix epoch").as_nanos()
        ));
        let first = sample_ephemeris();
        let second = GpsEphemeris {
            sat: SatId { constellation: Constellation::Gps, prn: 9 },
            ..sample_ephemeris()
        };
        let reports = serde_json::json!([
            {
                "sat": first.sat,
                "reference_week": first.week,
                "decoded_subframes": [],
                "ephemerides": [first.clone()]
            },
            {
                "sat": second.sat,
                "reference_week": second.week,
                "decoded_subframes": [],
                "ephemerides": [second.clone()]
            }
        ]);

        fs::write(
            &path,
            serde_json::to_string_pretty(&reports).expect("serialize nav decode reports"),
        )
        .expect("write nav decode reports");
        let parsed = read_ephemeris(&path).expect("read nav decode report array");
        fs::remove_file(&path).expect("remove nav decode reports");

        assert_eq!(parsed.len(), 2);
        assert_eq!(parsed[0].sat, first.sat);
        assert_eq!(parsed[1].sat, second.sat);
    }

    #[test]
    fn read_broadcast_navigation_data_accepts_json_navigation_payload() {
        let path = std::env::temp_dir().join(format!(
            "bijux_read_broadcast_navigation_payload_{}_{}.json",
            std::process::id(),
            SystemTime::now().duration_since(UNIX_EPOCH).expect("unix epoch").as_nanos()
        ));
        let ephemeris = sample_ephemeris();
        let klobuchar = sample_klobuchar_coefficients();
        let navigation = GpsBroadcastNavigationData {
            ephemerides: vec![ephemeris.clone()],
            klobuchar: Some(klobuchar),
        };

        fs::write(
            &path,
            serde_json::to_string_pretty(&navigation).expect("serialize navigation payload"),
        )
        .expect("write navigation payload");
        let parsed = read_broadcast_navigation_data(&path).expect("read navigation payload");
        fs::remove_file(&path).expect("remove navigation payload");

        assert_eq!(parsed.ephemerides.len(), 1);
        assert_eq!(parsed.ephemerides[0].sat, ephemeris.sat);
        assert_eq!(parsed.klobuchar, Some(klobuchar));
    }

    #[test]
    fn read_broadcast_navigation_data_accepts_wrapped_navigation_payload() {
        let path = std::env::temp_dir().join(format!(
            "bijux_read_broadcast_navigation_wrapped_{}_{}.json",
            std::process::id(),
            SystemTime::now().duration_since(UNIX_EPOCH).expect("unix epoch").as_nanos()
        ));
        let ephemeris = sample_ephemeris();
        let klobuchar = sample_klobuchar_coefficients();
        let wrapped = bijux_gnss_infra::api::core::ArtifactV1 {
            header: ArtifactHeaderV1 {
                schema_version: ArtifactReadPolicy::LATEST,
                producer: "bijux-gnss-cli".to_string(),
                producer_version: "test".to_string(),
                created_at_unix_ms: 0,
                git_sha: "test".to_string(),
                config_hash: "test".to_string(),
                dataset_id: None,
                toolchain: "test".to_string(),
                features: Vec::new(),
                deterministic: true,
                git_dirty: false,
            },
            payload: GpsBroadcastNavigationData {
                ephemerides: vec![ephemeris.clone()],
                klobuchar: Some(klobuchar),
            },
        };

        fs::write(
            &path,
            serde_json::to_string_pretty(&wrapped).expect("serialize wrapped navigation payload"),
        )
        .expect("write wrapped navigation payload");
        let parsed =
            read_broadcast_navigation_data(&path).expect("read wrapped navigation payload");
        fs::remove_file(&path).expect("remove wrapped navigation payload");

        assert_eq!(parsed.ephemerides.len(), 1);
        assert_eq!(parsed.ephemerides[0].sat, ephemeris.sat);
        assert_eq!(parsed.klobuchar, Some(klobuchar));
    }

    #[test]
    fn read_broadcast_navigation_data_accepts_rinex_navigation_klobuchar() {
        let path = std::env::temp_dir().join(format!(
            "bijux_read_broadcast_navigation_rinex_{}_{}.rnx",
            std::process::id(),
            SystemTime::now().duration_since(UNIX_EPOCH).expect("unix epoch").as_nanos()
        ));
        let ephemeris = sample_ephemeris();
        let klobuchar = sample_klobuchar_coefficients();
        let navigation = GpsBroadcastNavigationData {
            ephemerides: vec![ephemeris.clone()],
            klobuchar: Some(klobuchar),
        };

        write_rinex_broadcast_navigation(&path, &navigation, true)
            .expect("write rinex broadcast navigation");
        let parsed =
            read_broadcast_navigation_data(&path).expect("read rinex broadcast navigation");
        fs::remove_file(&path).expect("remove rinex broadcast navigation");

        assert_eq!(parsed.ephemerides.len(), 1);
        assert_eq!(parsed.ephemerides[0].sat, ephemeris.sat);
        assert_eq!(parsed.klobuchar, Some(klobuchar));
    }

    fn temp_file_path(name: &str) -> PathBuf {
        let nanos = SystemTime::now().duration_since(UNIX_EPOCH).expect("unix epoch").as_nanos();
        std::env::temp_dir().join(format!("bijux_{}_{}_{}.iq", name, std::process::id(), nanos))
    }

    fn temp_output_dir(name: &str) -> PathBuf {
        let nanos = SystemTime::now().duration_since(UNIX_EPOCH).expect("unix epoch").as_nanos();
        std::env::temp_dir().join(format!("bijux_{}_{}_{}", name, std::process::id(), nanos))
    }

    fn bias_sinex_fixture(name: &str) -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../bijux-gnss-nav/tests/data").join(name)
    }

    fn sample_common_args(out_dir: PathBuf) -> CommonArgs {
        CommonArgs {
            config: None,
            dataset: None,
            unregistered_dataset: true,
            out: Some(out_dir),
            report: ReportFormat::Json,
            seed: None,
            deterministic: true,
            dump: None,
            sidecar: None,
            resume: None,
        }
    }

    fn dual_frequency_epoch(
        epoch_idx: u64,
        p1_m: f64,
        p2_m: f64,
        phi1_m: f64,
        phi2_m: f64,
    ) -> ObsEpoch {
        dual_frequency_epoch_for_pair(
            epoch_idx,
            SatId { constellation: Constellation::Gps, prn: 12 },
            (SignalBand::L1, SignalCode::Ca, p1_m, phi1_m),
            (SignalBand::L2, SignalCode::Py, p2_m, phi2_m),
        )
    }

    fn dual_frequency_epoch_for_pair(
        epoch_idx: u64,
        sat: SatId,
        first: (SignalBand, SignalCode, f64, f64),
        second: (SignalBand, SignalCode, f64, f64),
    ) -> ObsEpoch {
        ObsEpoch {
            t_rx_s: Seconds(epoch_idx as f64),
            source_time: ReceiverSampleTrace::from_sample_index(epoch_idx, 1.0),
            gps_week: None,
            tow_s: None,
            epoch_idx,
            discontinuity: false,
            valid: true,
            processing_ms: None,
            role: ReceiverRole::Rover,
            sats: vec![
                dual_frequency_satellite(sat, first.0, first.1, first.2, first.3),
                dual_frequency_satellite(sat, second.0, second.1, second.2, second.3),
            ],
            decision: ObservationEpochDecision::Accepted,
            decision_reason: Some("accepted_observables_present".to_string()),
            manifest: None,
        }
    }

    fn dual_frequency_satellite(
        sat: SatId,
        band: SignalBand,
        code: SignalCode,
        pseudorange_m: f64,
        phase_m: f64,
    ) -> ObsSatellite {
        let signal = signal_registry(sat.constellation, band, code)
            .expect("dual-frequency signal must exist")
            .spec;
        let wavelength_m = SPEED_OF_LIGHT_MPS / signal.carrier_hz.value();

        ObsSatellite {
            signal_id: SigId { sat, band, code },
            pseudorange_m: Meters(pseudorange_m),
            pseudorange_var_m2: 1.0,
            carrier_phase_cycles: Cycles(phase_m / wavelength_m),
            carrier_phase_var_cycles2: 0.01,
            doppler_hz: Hertz(0.0),
            doppler_var_hz2: 1.0,
            cn0_dbhz: 45.0,
            lock_flags: LockFlags {
                code_lock: true,
                carrier_lock: true,
                bit_lock: false,
                cycle_slip: false,
            },
            multipath_suspect: false,
            observation_status: ObservationStatus::Accepted,
            observation_reject_reasons: Vec::new(),
            elevation_deg: None,
            azimuth_deg: None,
            weight: None,
            timing: None,
            error_model: None,
            metadata: ObsMetadata {
                tracking_mode: "test".to_string(),
                integration_ms: 1,
                lock_quality: 1.0,
                smoothing_window: 0,
                smoothing_age: 0,
                smoothing_resets: 0,
                signal,
                ..ObsMetadata::default()
            },
        }
    }

    fn sample_ephemeris() -> GpsEphemeris {
        GpsEphemeris {
            sat: SatId { constellation: Constellation::Gps, prn: 8 },
            iodc: 97,
            iode: 11,
            week: 2209,
            sv_health: 0,
            sv_accuracy: Some(2),
            toe_s: 345_600.0,
            toc_s: 504_018.0,
            sqrt_a: 5_153.795_477_5,
            e: 1.234_567_890_123e-2,
            i0: 9.4e-1,
            idot: 7.8e-10,
            omega0: 1.5,
            omegadot: -8.9e-9,
            w: 2.1e-1,
            m0: 6.0e-1,
            delta_n: 4.5e-9,
            cuc: 1.2e-6,
            cus: 2.3e-6,
            crc: 321.0,
            crs: 25.0,
            cic: 4.5e-8,
            cis: 5.6e-8,
            af0: -1.234_567_890_123e-4,
            af1: 2.345_678_901_234e-12,
            af2: 0.0,
            tgd: -1.9e-8,
        }
    }

    fn sample_klobuchar_coefficients() -> KlobucharCoefficients {
        KlobucharCoefficients::new(
            [0.1212e-7, 0.1490e-7, -0.5960e-7, 0.1192e-6],
            [0.1167e6, -0.2294e6, -0.1311e6, 0.1049e7],
        )
    }

    fn sample_front_end_metrics() -> bijux_gnss_infra::api::signal::IqFrontEndMetrics {
        bijux_gnss_infra::api::signal::IqFrontEndMetrics {
            sample_count: 4_092,
            i_mean: 0.0,
            q_mean: 0.0,
            i_power: 1.0,
            q_power: 1.0,
            iq_power_ratio: 1.0,
            power_imbalance_warning: false,
            quadrature_error_deg: Some(0.0),
            quadrature_error_warning: false,
            clipping_pct: Some(0.0),
            clipping_warning: false,
            centered_rms: 1.0,
            zero_signal_detected: false,
            zero_signal_reason: None,
            precision_claims_allowed: true,
            precision_claims_refused_reason: None,
            rms: 1.0,
            dc_imbalance: 0.0,
        }
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

        let frame = load_frame_window(&path, &config, &metadata, 1).expect("load frame");
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
        let sample_count = 4_092usize * super::TRACKING_HISTORY_CODE_PERIODS;
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

        let frame =
            super::load_tracking_frame(&path, &config, &metadata).expect("load tracking frame");
        assert_eq!(frame.len(), sample_count);

        fs::remove_file(&path).expect("remove iq8 fixture");
    }

    #[test]
    fn read_tracking_dump_accepts_wrapped_track_artifacts() {
        let path = temp_file_path("read_tracking_dump_artifact");
        let wrapped = TrackEpochV1 {
            header: ArtifactHeaderV1 {
                schema_version: 1,
                producer: "test".to_string(),
                producer_version: "0.1.0".to_string(),
                created_at_unix_ms: 0,
                git_sha: "test".to_string(),
                config_hash: "test".to_string(),
                dataset_id: None,
                toolchain: "rustc test".to_string(),
                features: Vec::new(),
                deterministic: true,
                git_dirty: false,
            },
            payload: TrackEpoch {
                epoch: Epoch { index: 7 },
                sample_index: 28_644,
                source_time: ReceiverSampleTrace::from_sample_index(28_644, 4_092_000.0),
                sat: SatId { constellation: Constellation::Gps, prn: 12 },
                signal_band: SignalBand::L1,
                signal_code: bijux_gnss_infra::api::core::SignalCode::Unknown,
                glonass_frequency_channel: None,
                prompt_i: -12.0,
                prompt_q: 3.0,
                early_i: -8.0,
                early_q: 1.0,
                late_i: -7.0,
                late_q: 2.0,
                carrier_hz: Hertz(120.0),
                carrier_phase_cycles: Cycles(0.25),
                code_rate_hz: Hertz(1_023_000.0),
                code_phase_samples: Chips(144.0),
                lock: true,
                cn0_dbhz: 45.0,
                pll_lock: true,
                dll_lock: true,
                fll_lock: true,
                cycle_slip: false,
                nav_bit_lock: true,
                navigation_bit_sign: Some(-1),
                transmit_time: None,
                dll_err: 0.0,
                pll_err: 0.0,
                fll_err: 0.0,
                anti_false_lock: false,
                cycle_slip_reason: None,
                lock_state: "tracking".to_string(),
                lock_state_reason: None,
                channel_id: Some(0),
                channel_uid: "Gps-12-ch00".to_string(),
                tracking_provenance: "test".to_string(),
                tracking_assumptions: None,
                signal_delay_alignment: None,
                tracking_uncertainty: None,
                processing_ms: None,
            },
        };
        fs::write(&path, serde_json::to_string(&wrapped).expect("serialize track artifact"))
            .expect("write artifact");

        let rows = read_tracking_dump(&path).expect("read tracking dump");

        assert_eq!(rows.len(), 1);
        assert_eq!(rows[0].epoch_idx, 7);
        assert_eq!(rows[0].sat.prn, 12);
        assert_eq!(rows[0].navigation_bit_sign, Some(-1));

        fs::remove_file(&path).expect("remove track artifact");
    }

    #[test]
    fn read_tracking_dump_accepts_track_report_json() {
        let path = temp_file_path("read_tracking_dump_report");
        let report = TrackingReport {
            sats: vec![SatId { constellation: Constellation::Gps, prn: 12 }],
            doppler_search: DopplerSearchSettings {
                max_search_hz: 5_000,
                bin_width_hz: 500,
                bin_count: 21,
                intermediate_freq_hz: 0.0,
            },
            front_end_metrics: sample_front_end_metrics(),
            signal_quality: RawIqSignalQualityReport {
                format: "iq8".to_string(),
                sample_rate_hz: 4_092_000.0,
                intermediate_freq_hz: 0.0,
                capture_start_utc: "2026-07-10T00:00:00Z".to_string(),
                analyzed_samples: 4_092,
                usable_duration_s: 0.001,
                estimated_noise_floor_db: -30.0,
                front_end_metrics: sample_front_end_metrics(),
            },
            epochs: vec![TrackingRow {
                epoch_idx: 3,
                sample_index: 12_276,
                sat: SatId { constellation: Constellation::Gps, prn: 12 },
                carrier_hz: 120.0,
                carrier_phase_cycles: 0.5,
                code_rate_hz: 1_023_000.0,
                code_phase_samples: 144.0,
                prompt_i: 4.0,
                prompt_q: 2.0,
                early_i: 0.0,
                early_q: 0.0,
                late_i: 0.0,
                late_q: 0.0,
                lock: true,
                cn0_dbhz: 45.0,
                pll_lock: true,
                dll_lock: true,
                fll_lock: true,
                cycle_slip: false,
                nav_bit_lock: true,
                navigation_bit_sign: Some(1),
                dll_err: 0.0,
                pll_err: 0.0,
                fll_err: 0.0,
                anti_false_lock: false,
                cycle_slip_reason: None,
                lock_state: "tracking".to_string(),
                lock_state_reason: None,
            }],
        };
        fs::write(&path, serde_json::to_string_pretty(&report).expect("serialize track report"))
            .expect("write report");

        let rows = read_tracking_dump(&path).expect("read tracking report");

        assert_eq!(rows.len(), 1);
        assert_eq!(rows[0].epoch_idx, 3);
        assert_eq!(rows[0].navigation_bit_sign, Some(1));

        fs::remove_file(&path).expect("remove track report");
    }

    #[test]
    fn load_acquisition_frame_reads_configured_integration_window() {
        let path = temp_file_path("load_acquisition_frame_window");
        let coherent_ms = 5u32;
        let noncoherent = 4u32;
        let sample_count = 4_092usize * super::acquisition_code_periods(coherent_ms, noncoherent);
        let mut raw = Vec::with_capacity(sample_count * 2);
        for _ in 0..sample_count {
            raw.push(8u8);
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
            acquisition_integration_ms: coherent_ms,
            acquisition_noncoherent: noncoherent,
            ..ReceiverPipelineConfig::default()
        };

        let frame = super::load_acquisition_frame(&path, &config, &metadata)
            .expect("load acquisition frame");
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
            position_covariance_ecef_m2: None,
            latitude_deg: 60.0,
            longitude_deg: 18.0,
            altitude_m: Meters(4.0),
            clock_bias_s: Seconds(0.001),
            clock_bias_m: Meters(299_792.458),
            clock_drift_s_per_s: 0.0,
            pdop: 1.0,
            pre_fit_residual_rms_m: None,
            post_fit_residual_rms_m: None,
            rms_m: Meters(2.0),
            status: SolutionStatus::CodeOnly,
            quality: SolutionStatus::CodeOnly.quality_flag(),
            validity: SolutionValidity::Stable,
            valid: true,
            processing_ms: None,
            residuals: Vec::new(),
            constellation_residual_rms: Vec::new(),
            health: Vec::new(),
            isb: Vec::new(),
            sigma_e_m: None,
            sigma_n_m: None,
            sigma_u_m: None,
            horizontal_error_ellipse_major_axis_m: None,
            horizontal_error_ellipse_minor_axis_m: None,
            horizontal_error_ellipse_azimuth_deg: None,
            sigma_h_m: Some(Meters(0.5)),
            sigma_v_m: Some(Meters(0.8)),
            innovation_rms_m: None,
            normalized_innovation_rms: None,
            normalized_innovation_max: None,
            ekf_innovation_rms: None,
            ekf_condition_number: None,
            wls_solver_rank: Some(4),
            wls_condition_number: Some(12.5),
            ekf_whiteness_ratio: None,
            ekf_predicted_variance: None,
            ekf_observed_variance: None,
            integrity_hpl_m: None,
            integrity_vpl_m: None,
            model_version: NAV_SOLUTION_MODEL_VERSION,
            lifecycle_state: NavLifecycleState::CodeOnly,
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
            hdop: Some(0.8),
            vdop: Some(0.6),
            gdop: Some(1.05),
            tdop: Some(0.4),
            stability_signature: "navsig:v2:test".to_string(),
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
        assert_eq!(payload["source_observation_epoch_id"], "epoch-0000000012-sample-000000006138");
        assert_eq!(payload["clock_bias_s"], 0.001);
        assert_eq!(payload["clock_bias_m"], 299_792.458);
        assert_eq!(payload["dops"]["pdop"], 1.0);
        assert_eq!(payload["dops"]["hdop"], 0.8);
        assert_eq!(payload["dops"]["vdop"], 0.6);
        assert_eq!(payload["dops"]["gdop"], 1.05);
        assert_eq!(payload["dops"]["tdop"], 0.4);
        assert_eq!(payload["solver_diagnostics"]["wls_solver_rank"], 4);
        assert_eq!(payload["solver_diagnostics"]["wls_condition_number"], 12.5);
        assert!(payload["solver_diagnostics"]["ekf_condition_number"].is_null());

        fs::remove_file(&path).expect("remove nav solution output");
        fs::remove_dir(&out_dir).expect("remove output directory");
    }

    #[test]
    fn write_obs_timeseries_emits_observation_residual_artifact() {
        let out_dir = temp_output_dir("obs_residual_output");
        let common = sample_common_args(out_dir.clone());
        let profile = ReceiverConfig::default();
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 1_023_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            ..ReceiverPipelineConfig::default()
        };
        let sat = SatId { constellation: Constellation::Gps, prn: 19 };
        let carrier_hz = bijux_gnss_infra::api::receiver::carrier_hz_from_doppler_hz(0.0, 125.0);
        let track = bijux_gnss_infra::api::receiver::TrackingResult {
            sat,
            carrier_hz,
            code_phase_samples: 0.0,
            acquisition_hypothesis: "accepted".to_string(),
            acquisition_score: 1.0,
            acquisition_code_phase_samples: 0,
            acquisition_carrier_hz: carrier_hz,
            acq_to_track_state: "accepted".to_string(),
            epochs: vec![
                TrackEpoch {
                    epoch: Epoch { index: 70 },
                    sample_index: 70 * 1023,
                    source_time: ReceiverSampleTrace::from_sample_index(70 * 1023, 1_023_000.0),
                    sat,
                    signal_band: SignalBand::L1,
                    signal_code: bijux_gnss_infra::api::core::SignalCode::Unknown,
                    glonass_frequency_channel: None,
                    prompt_i: 1.0,
                    prompt_q: 0.0,
                    early_i: 0.0,
                    early_q: 0.0,
                    late_i: 0.0,
                    late_q: 0.0,
                    carrier_hz: Hertz(carrier_hz),
                    carrier_phase_cycles: Cycles(10.0),
                    code_rate_hz: Hertz(1_023_000.0),
                    code_phase_samples: Chips(0.0),
                    lock: true,
                    cn0_dbhz: 45.0,
                    pll_lock: true,
                    dll_lock: true,
                    fll_lock: true,
                    cycle_slip: false,
                    nav_bit_lock: false,
                    navigation_bit_sign: None,
                    transmit_time: None,
                    dll_err: 0.0,
                    pll_err: 0.0,
                    fll_err: 0.0,
                    anti_false_lock: false,
                    cycle_slip_reason: None,
                    lock_state: "tracking".to_string(),
                    lock_state_reason: Some("stable_tracking".to_string()),
                    channel_id: None,
                    channel_uid: String::new(),
                    tracking_provenance: "test".to_string(),
                    tracking_assumptions: None,
                    signal_delay_alignment: Some(SignalDelayAlignment {
                        whole_code_periods: 68,
                        sample_delay_samples: 0,
                        source: "synthetic_truth".to_string(),
                    }),
                    tracking_uncertainty: None,
                    processing_ms: None,
                },
                TrackEpoch {
                    epoch: Epoch { index: 71 },
                    sample_index: 71 * 1023,
                    source_time: ReceiverSampleTrace::from_sample_index(71 * 1023, 1_023_000.0),
                    sat,
                    signal_band: SignalBand::L1,
                    signal_code: bijux_gnss_infra::api::core::SignalCode::Unknown,
                    glonass_frequency_channel: None,
                    prompt_i: 1.0,
                    prompt_q: 0.0,
                    early_i: 0.0,
                    early_q: 0.0,
                    late_i: 0.0,
                    late_q: 0.0,
                    carrier_hz: Hertz(carrier_hz),
                    carrier_phase_cycles: Cycles(10.125),
                    code_rate_hz: Hertz(1_023_000.0),
                    code_phase_samples: Chips(0.0),
                    lock: true,
                    cn0_dbhz: 45.0,
                    pll_lock: true,
                    dll_lock: true,
                    fll_lock: true,
                    cycle_slip: false,
                    nav_bit_lock: false,
                    navigation_bit_sign: None,
                    transmit_time: None,
                    dll_err: 0.0,
                    pll_err: 0.0,
                    fll_err: 0.0,
                    anti_false_lock: false,
                    cycle_slip_reason: None,
                    lock_state: "tracking".to_string(),
                    lock_state_reason: Some("stable_tracking".to_string()),
                    channel_id: None,
                    channel_uid: String::new(),
                    tracking_provenance: "test".to_string(),
                    tracking_assumptions: None,
                    signal_delay_alignment: Some(SignalDelayAlignment {
                        whole_code_periods: 68,
                        sample_delay_samples: 0,
                        source: "synthetic_truth".to_string(),
                    }),
                    tracking_uncertainty: None,
                    processing_ms: None,
                },
            ],
            transitions: Vec::new(),
        };

        super::write_obs_timeseries(&common, &config, &[track], 10, &profile, None)
            .expect("write observation artifacts");

        let artifacts_dir = super::artifacts_dir(&common, "track", None).expect("artifacts dir");
        let residual_path = artifacts_dir.join("observation_residuals.jsonl");
        let residual_text = fs::read_to_string(&residual_path).expect("read residual artifact");
        let first_line = residual_text.lines().next().expect("residual line");
        let payload: serde_json::Value = serde_json::from_str(first_line).expect("parse residual");
        let raw_pseudorange_m = payload["payload"]["sats"][0]["pseudorange_m"]["raw"]
            .as_f64()
            .expect("raw pseudorange");
        let carrier_smoothed_code_validation_path =
            artifacts_dir.join("carrier_smoothed_code_validation.json");
        let carrier_smoothed_code_validation: serde_json::Value = serde_json::from_str(
            &fs::read_to_string(&carrier_smoothed_code_validation_path)
                .expect("read carrier-smoothed code validation artifact"),
        )
        .expect("parse carrier-smoothed code validation artifact");

        assert_eq!(payload["payload"]["artifact_id"], "obs-epoch-0000000070");
        assert_eq!(payload["payload"]["accepted"], true);
        assert!(raw_pseudorange_m > 0.0);
        assert_eq!(carrier_smoothed_code_validation["observations"], 2);
        assert_eq!(carrier_smoothed_code_validation["accepted_observations"], 2);
        assert_eq!(carrier_smoothed_code_validation["cycle_slip_observations"], 0);
        assert!(carrier_smoothed_code_validation["improvement_verified"].is_null());

        fs::remove_dir_all(&out_dir).expect("remove output directory");
    }

    #[test]
    fn write_obs_timeseries_emits_observation_measurement_quality_artifact() {
        fn quality_signal(
            constellation: Constellation,
            band: SignalBand,
            code: SignalCode,
        ) -> bijux_gnss_infra::api::core::SignalSpec {
            signal_registry(constellation, band, code).expect("registered signal").spec
        }

        fn quality_alignment_periods(band: SignalBand) -> u64 {
            match band {
                SignalBand::L1 => 68,
                SignalBand::L2 => 4,
                SignalBand::L5 => 8,
                SignalBand::E1 => 17,
                SignalBand::E5 => 6,
                SignalBand::B1 => 8,
                SignalBand::B2 => 8,
                _ => 68,
            }
        }

        fn quality_track(
            config: &ReceiverPipelineConfig,
            sat: SatId,
            signal: bijux_gnss_infra::api::core::SignalSpec,
            cn0_dbhz: f64,
        ) -> bijux_gnss_infra::api::receiver::TrackingResult {
            let tracked_carrier_hz =
                signal.carrier_hz.value() - GPS_L1_CA_CARRIER_HZ.value() + 125.0;
            let epoch = TrackEpoch {
                epoch: Epoch { index: 0 },
                sample_index: 0,
                source_time: ReceiverSampleTrace::from_sample_index(0, config.sampling_freq_hz),
                sat,
                signal_band: signal.band,
                signal_code: signal.code,
                glonass_frequency_channel: None,
                prompt_i: 1.0,
                prompt_q: 0.0,
                early_i: 0.1,
                early_q: 0.0,
                late_i: -0.1,
                late_q: 0.0,
                carrier_hz: Hertz(tracked_carrier_hz + sat.prn as f64),
                carrier_phase_cycles: Cycles(2_400.0 + sat.prn as f64),
                code_rate_hz: Hertz(signal.code_rate_hz),
                code_phase_samples: Chips(0.0),
                lock: true,
                cn0_dbhz,
                pll_lock: true,
                dll_lock: true,
                fll_lock: true,
                cycle_slip: false,
                nav_bit_lock: false,
                navigation_bit_sign: None,
                transmit_time: None,
                dll_err: 0.0,
                pll_err: 0.0,
                fll_err: 0.0,
                anti_false_lock: false,
                cycle_slip_reason: None,
                lock_state: "tracking".to_string(),
                lock_state_reason: Some("stable_tracking".to_string()),
                channel_id: Some(sat.prn),
                channel_uid: format!("{:?}-{:02}-{:?}", sat.constellation, sat.prn, signal.band),
                tracking_provenance: "test".to_string(),
                tracking_assumptions: None,
                signal_delay_alignment: Some(SignalDelayAlignment {
                    whole_code_periods: quality_alignment_periods(signal.band),
                    sample_delay_samples: 0,
                    source: "quality_fixture".to_string(),
                }),
                tracking_uncertainty: Some(TrackingUncertainty {
                    code_phase_samples: 0.05,
                    carrier_phase_cycles: 0.02,
                    doppler_hz: 0.5,
                    cn0_dbhz: 0.75,
                }),
                processing_ms: None,
            };
            bijux_gnss_infra::api::receiver::TrackingResult {
                sat,
                carrier_hz: epoch.carrier_hz.0,
                code_phase_samples: epoch.code_phase_samples.0,
                acquisition_hypothesis: "accepted".to_string(),
                acquisition_score: 1.0,
                acquisition_code_phase_samples: 0,
                acquisition_carrier_hz: epoch.carrier_hz.0,
                acq_to_track_state: "accepted".to_string(),
                epochs: vec![epoch],
                transitions: Vec::new(),
            }
        }

        let out_dir = temp_output_dir("obs_quality_output");
        let common = sample_common_args(out_dir.clone());
        let profile = ReceiverConfig::default();
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 10_230_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            channels: 8,
            tracking_integration_ms: 10,
            ..ReceiverPipelineConfig::default()
        };
        let mut l5 = quality_track(
            &config,
            SatId { constellation: Constellation::Gps, prn: 5 },
            quality_signal(Constellation::Gps, SignalBand::L5, SignalCode::Unknown),
            43.0,
        );
        l5.epochs[0].cycle_slip = true;
        l5.epochs[0].cycle_slip_reason = Some("simulated_phase_slip".to_string());
        let mut b2 = quality_track(
            &config,
            SatId { constellation: Constellation::Beidou, prn: 8 },
            quality_signal(Constellation::Beidou, SignalBand::B2, SignalCode::B2I),
            39.0,
        );
        b2.epochs[0].pll_lock = false;
        let tracks = vec![
            quality_track(
                &config,
                SatId { constellation: Constellation::Gps, prn: 3 },
                signal_spec_gps_l1_ca(),
                46.0,
            ),
            quality_track(
                &config,
                SatId { constellation: Constellation::Gps, prn: 4 },
                quality_signal(Constellation::Gps, SignalBand::L2, SignalCode::L2C),
                44.0,
            ),
            l5,
            quality_track(
                &config,
                SatId { constellation: Constellation::Galileo, prn: 11 },
                quality_signal(Constellation::Galileo, SignalBand::E1, SignalCode::E1B),
                42.0,
            ),
            quality_track(
                &config,
                SatId { constellation: Constellation::Galileo, prn: 12 },
                quality_signal(Constellation::Galileo, SignalBand::E5, SignalCode::E5a),
                41.0,
            ),
            quality_track(
                &config,
                SatId { constellation: Constellation::Beidou, prn: 7 },
                quality_signal(Constellation::Beidou, SignalBand::B1, SignalCode::B1I),
                40.0,
            ),
            b2,
        ];

        super::write_obs_timeseries(&common, &config, &tracks, 10, &profile, None)
            .expect("write observation artifacts");

        let artifacts_dir = super::artifacts_dir(&common, "track", None).expect("artifacts dir");
        let quality_path = artifacts_dir.join("observation_measurement_quality.jsonl");
        let quality_text = fs::read_to_string(&quality_path).expect("read quality artifact");
        let first_line = quality_text.lines().next().expect("quality line");
        let payload: serde_json::Value = serde_json::from_str(first_line).expect("parse quality");
        let sats = payload["payload"]["sats"].as_array().expect("quality satellites");

        assert_eq!(sats.len(), 7);
        assert_eq!(
            sats.iter()
                .map(|sat| sat["signal_id"]["band"].as_str().expect("signal band"))
                .collect::<Vec<_>>(),
            vec!["L1", "L2", "L5", "E1", "E5", "B1", "B2"]
        );
        assert!(sats.iter().all(|sat| sat["cn0_dbhz"].as_f64().is_some()));
        assert!(sats.iter().all(|sat| sat["pseudorange_sigma_m"].as_f64().is_some()));
        assert!(sats.iter().all(|sat| sat["carrier_phase_sigma_cycles"].as_f64().is_some()));
        assert!(sats.iter().all(|sat| sat["doppler_sigma_hz"].as_f64().is_some()));
        assert!(sats.iter().all(|sat| sat["cn0_sigma_dbhz"].as_f64().is_some()));
        assert!(sats.iter().all(|sat| {
            let covariance = &sat["measurement_covariance"];
            covariance["status"].as_str() == Some("positive_semidefinite")
                && covariance["code_phase_m2"].as_f64().is_some_and(|value| value > 0.0)
                && covariance["carrier_phase_m2"].as_f64().is_some_and(|value| value > 0.0)
                && covariance["doppler_hz2"].as_f64().is_some_and(|value| value > 0.0)
                && covariance["code_carrier_m2"].as_f64()
                    == covariance["carrier_code_m2"].as_f64()
                && covariance["carrier_doppler_m_hz"].as_f64()
                    == covariance["doppler_carrier_hz_m"].as_f64()
        }));
        assert!(sats.iter().all(|sat| sat["observation_lock_state"]
            .as_str()
            .is_some_and(|state| !state.is_empty())));

        let l5_quality =
            sats.iter().find(|sat| sat["signal_id"]["band"] == "L5").expect("L5 quality");
        assert_eq!(l5_quality["cycle_slip"], true);
        assert_eq!(l5_quality["cycle_slip_reason"], "simulated_phase_slip");

        let b2_quality =
            sats.iter().find(|sat| sat["signal_id"]["band"] == "B2").expect("B2 quality");
        assert_eq!(b2_quality["lock_flags"]["code_lock"], true);
        assert_eq!(b2_quality["lock_flags"]["carrier_lock"], false);

        fs::remove_dir_all(&out_dir).expect("remove output directory");
    }

    #[test]
    fn write_melbourne_wubbena_diagnostics_emits_wide_lane_events() {
        let out_dir = temp_output_dir("melbourne_wubbena_output");
        fs::create_dir_all(&out_dir).expect("create output directory");

        super::write_melbourne_wubbena_diagnostics(
            &out_dir,
            &[
                dual_frequency_epoch(0, 22_000_000.0, 22_000_002.0, 21_999_999.0, 22_000_000.5),
                dual_frequency_epoch(1, 22_000_000.0, 22_000_002.0, 21_999_999.01, 22_000_000.49),
                dual_frequency_epoch(2, 22_000_000.0, 22_000_002.0, 22_000_005.5, 22_000_000.5),
            ],
        )
        .expect("write melbourne-wubbena diagnostics");

        let path = out_dir.join("melbourne_wubbena.jsonl");
        let text = fs::read_to_string(&path).expect("read melbourne-wubbena artifact");
        let rows: Vec<serde_json::Value> = text
            .lines()
            .map(|line| serde_json::from_str(line).expect("parse melbourne-wubbena row"))
            .collect();

        assert_eq!(rows.len(), 3);
        assert_eq!(rows[0]["event"], "insufficient_history");
        assert_eq!(rows[1]["event"], "nominal");
        assert_eq!(rows[2]["event"], "wide_lane_slip_suspect");
        assert!(
            rows[2]["delta_from_previous_wide_lane_cycles"].as_f64().expect("wide-lane delta")
                >= 0.5
        );

        fs::remove_dir_all(&out_dir).expect("remove output directory");
    }

    #[test]
    fn write_iono_free_code_artifact_emits_bias_corrected_rows() {
        let out_dir = temp_output_dir("iono_free_code_output");
        fs::create_dir_all(&out_dir).expect("create output directory");

        let provider = fs::read_to_string(bias_sinex_fixture("gps_l1_l2_absolute_biases.bia"))
            .expect("read bias sinex fixture")
            .parse::<BiasSinexProvider>()
            .expect("parse bias sinex fixture");
        let l1 = signal_spec_gps_l1_ca();
        let l2 = signal_spec_gps_l2_py();
        let base_range_m = 20_200_000.0;
        let iono_l1_m = 4.0;
        let l1_bias_m = 3.0e-9 * SPEED_OF_LIGHT_MPS;
        let l2_bias_m = 9.0e-9 * SPEED_OF_LIGHT_MPS;
        let l1_hz = l1.carrier_hz.value();
        let l2_hz = l2.carrier_hz.value();
        let l2_iono_m = iono_l1_m * (l1_hz * l1_hz) / (l2_hz * l2_hz);
        let mut epoch = dual_frequency_epoch(
            0,
            base_range_m + iono_l1_m + l1_bias_m,
            base_range_m + l2_iono_m + l2_bias_m,
            0.0,
            0.0,
        );
        for satellite in &mut epoch.sats {
            satellite.signal_id.sat.prn = 23;
        }

        super::write_iono_free_code_artifact(&out_dir, &[epoch], Some(&provider))
            .expect("write iono-free code artifact");

        let path = out_dir.join("iono_free_code.jsonl");
        let text = fs::read_to_string(&path).expect("read iono-free code artifact");
        let rows: Vec<serde_json::Value> = text
            .lines()
            .map(|line| serde_json::from_str(line).expect("parse iono-free code row"))
            .collect();

        assert_eq!(rows.len(), 1);
        assert_eq!(rows[0]["status"], "ok");
        assert!(rows[0]["code_bias_m"].as_f64().expect("code bias").abs() > 0.1);
        assert!(
            (rows[0]["corrected_code_m"].as_f64().expect("corrected code") - base_range_m).abs()
                < 1.0e-6
        );

        fs::remove_dir_all(&out_dir).expect("remove output directory");
    }

    #[test]
    fn write_iono_free_code_artifact_emits_supported_constellation_rows() {
        let out_dir = temp_output_dir("iono_free_code_constellations_output");
        fs::create_dir_all(&out_dir).expect("create output directory");

        let epochs = [
            dual_frequency_epoch_for_pair(
                0,
                SatId { constellation: Constellation::Galileo, prn: 19 },
                (SignalBand::E1, SignalCode::E1B, 24_000_004.0, 0.0),
                (SignalBand::E5, SignalCode::E5a, 24_000_002.5, 0.0),
            ),
            dual_frequency_epoch_for_pair(
                1,
                SatId { constellation: Constellation::Beidou, prn: 7 },
                (SignalBand::B1, SignalCode::B1I, 24_100_003.0, 0.0),
                (SignalBand::B2, SignalCode::B2I, 24_100_002.0, 0.0),
            ),
        ];

        super::write_iono_free_code_artifact(&out_dir, &epochs, None)
            .expect("write iono-free code artifact");

        let path = out_dir.join("iono_free_code.jsonl");
        let text = fs::read_to_string(&path).expect("read iono-free code artifact");
        let rows: Vec<serde_json::Value> = text
            .lines()
            .map(|line| serde_json::from_str(line).expect("parse iono-free code row"))
            .collect();

        assert_eq!(rows.len(), 2);
        assert!(rows.iter().any(|row| row["band_1"] == "E1" && row["band_2"] == "E5"));
        assert!(rows.iter().any(|row| row["band_1"] == "B1" && row["band_2"] == "B2"));
        assert!(rows.iter().all(|row| row["status"] == "ok"));

        fs::remove_dir_all(&out_dir).expect("remove output directory");
    }

    #[test]
    fn write_narrow_lane_artifact_emits_supported_constellation_rows() {
        let out_dir = temp_output_dir("narrow_lane_output");
        fs::create_dir_all(&out_dir).expect("create output directory");

        let epochs = vec![
            dual_frequency_epoch(0, 22_000_000.0, 22_000_002.0, 21_999_999.0, 22_000_000.5),
            dual_frequency_epoch_for_pair(
                1,
                SatId { constellation: Constellation::Galileo, prn: 19 },
                (SignalBand::E1, SignalCode::E1B, 24_345_678.125, 24_345_677.0),
                (SignalBand::E5, SignalCode::E5a, 24_345_679.875, 24_345_674.5),
            ),
            dual_frequency_epoch_for_pair(
                2,
                SatId { constellation: Constellation::Beidou, prn: 7 },
                (SignalBand::B1, SignalCode::B1I, 24_345_678.125, 24_345_677.5),
                (SignalBand::B2, SignalCode::B2I, 24_345_679.875, 24_345_674.25),
            ),
        ];

        super::write_narrow_lane_artifact(&out_dir, &epochs).expect("write narrow-lane artifact");

        let path = out_dir.join("narrow_lane.jsonl");
        let text = fs::read_to_string(&path).expect("read narrow-lane artifact");
        let rows: Vec<serde_json::Value> = text
            .lines()
            .map(|line| serde_json::from_str(line).expect("parse narrow-lane row"))
            .collect();

        assert_eq!(rows.len(), 3);
        assert_eq!(rows[0]["band_1"], "L1");
        assert_eq!(rows[0]["band_2"], "L2");
        assert_eq!(rows[1]["band_1"], "E1");
        assert_eq!(rows[1]["band_2"], "E5");
        assert_eq!(rows[2]["band_1"], "B1");
        assert_eq!(rows[2]["band_2"], "B2");
        assert!(rows.iter().all(|row| row["narrow_lane_wavelength_m"].as_f64().is_some()));
        assert!(rows.iter().all(|row| row["phase_cycles"].as_f64().is_some()));

        fs::remove_dir_all(&out_dir).expect("remove output directory");
    }
}
