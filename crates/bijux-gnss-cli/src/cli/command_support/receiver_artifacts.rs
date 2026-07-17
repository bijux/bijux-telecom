use super::*;

pub(crate) fn write_track_timeseries_for_command(
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

pub(crate) fn write_track_timeseries(
    common: &CommonArgs,
    report: &TrackingReport,
    profile: &ReceiverConfig,
    dataset: Option<&DatasetEntry>,
) -> Result<()> {
    write_track_timeseries_for_command(common, "track", report, profile, dataset)
}

#[cfg(test)]
pub(crate) fn write_obs_timeseries_for_command(
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

pub(crate) fn write_observation_artifacts_for_command(
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

pub(crate) fn dual_frequency_pair_observed(
    obs: &[ObsEpoch],
    band_1: SignalBand,
    band_2: SignalBand,
) -> bool {
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

pub(crate) fn supported_observed_dual_frequency_pairs(
    obs: &[ObsEpoch],
) -> Vec<(SignalBand, SignalBand)> {
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
            if observation.status == "ok" {
                lines.push(serde_json::to_string(&observation)?);
            }
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
            if observation.status == "ok" {
                lines.push(serde_json::to_string(&observation)?);
            }
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

pub(crate) fn write_carrier_smoothed_code_validation(
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
pub(crate) fn write_obs_timeseries(
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

pub(crate) fn write_tracking_timing_for_command(
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
pub(crate) struct ObservationResidualEpochV1 {
    header: ArtifactHeaderV1,
    payload: bijux_gnss_infra::api::receiver::ObservationResidualEpochReport,
}

#[derive(Debug, Clone, Serialize)]
pub(crate) struct ObservationMeasurementQualityEpochV1 {
    header: ArtifactHeaderV1,
    payload: bijux_gnss_infra::api::receiver::ObservationMeasurementQualityEpochReport,
}
