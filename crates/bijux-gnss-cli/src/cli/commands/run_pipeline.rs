fn validate_config(profile: &ReceiverConfig) -> Result<()> {
    let report = <ReceiverConfig as ValidateConfig>::validate(profile);
    if report.errors.is_empty() {
        return Ok(());
    }
    let messages: Vec<String> = report.errors.into_iter().map(|e| e.message).collect();
    bail!("invalid config: {}", messages.join(", "));
}

fn handle_acquire(command: GnssCommand) -> Result<()> {
    let GnssCommand::Acquire {
        common,
        file,
        sampling_hz,
        if_hz,
        code_hz,
        code_length,
        doppler_search_hz,
        doppler_step_hz,
        offset_bytes: _offset_bytes,
        top,
        prn,
    } = command
    else {
        bail!("invalid command for handler");
    };

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
    validate_config(&profile)?;
    let config = profile.to_pipeline_config();

    let input_file = resolve_input_file(file.as_ref(), dataset.as_ref())?;

    let frame = load_acquisition_frame(&input_file, &config, &raw_iq_metadata)?;
    let signal_quality = measure_signal_quality_from_samples(&raw_iq_metadata, &frame.iq);
    let runtime = runtime_config_from_env(&common, None);
    let acquisition = AcquisitionEngine::new(config, runtime);
    let sats = bijux_gnss_infra::api::core::prns_to_sats(&prn);
    let results = acquisition.run_fft_topn(
        &frame,
        &sats,
        top,
        profile.acquisition.integration_ms,
        profile.acquisition.noncoherent_integration,
    );

    let primary_results =
        results.iter().filter_map(|candidates| candidates.first().cloned()).collect::<Vec<_>>();
    let primary_rows = primary_results.iter().map(acquisition_row_from_result).collect::<Vec<_>>();
    let mut rows = Vec::new();
    for candidates in &results {
        for r in candidates {
            rows.push(acquisition_row_from_result(r));
        }
    }

    let report = AcquisitionReport {
        sats,
        search_summary: bijux_gnss_infra::api::core::AcqSearchSummary::from_results(
            &primary_results,
        ),
        doppler_search: doppler_search_settings(&profile),
        code_phase_search: code_phase_search_settings_from_results(&profile, &results),
        front_end_metrics: signal_quality.front_end_metrics.clone(),
        signal_quality,
        reported_prns: summarize_reported_prns(&rows),
        primary_results: primary_rows,
        results: rows,
    };
    match common.report {
        ReportFormat::Table => print_acquisition_table(&report),
        ReportFormat::Json => emit_report(&common, "acquire", &report)?,
    }
    write_signal_quality_report(&common, "acquire", &report.signal_quality)?;
    let (layout, header) =
        prepare_run(&infra_args(&common), "acquire", &profile, dataset.as_ref())?;
    let out_dir = layout.artifacts_dir;
    let acq_path = out_dir.join("acq.jsonl");
    let mut acq_lines = Vec::new();
    for candidates in &results {
        for result in candidates {
            let wrapped = AcqResultV1 { header: header.clone(), payload: result.clone() };
            acq_lines.push(serde_json::to_string(&wrapped)?);
        }
    }
    fs::write(&acq_path, acq_lines.join("\n"))?;
    validate_jsonl_schema(&schema_path("acq_result_v1.schema.json"), &acq_path, false)?;
    write_manifest(&common, "acquire", &profile, dataset.as_ref(), &report)?;

    Ok(())
}

fn acquisition_row_from_result(result: &bijux_gnss_infra::api::core::AcqResult) -> AcquisitionRow {
    let (coarse_carrier_hz, doppler_refinement_hz, doppler_refinement_bins) = result
        .doppler_refinement
        .as_ref()
        .map(|refinement| {
            (
                Some(refinement.coarse_carrier_hz.0),
                Some(refinement.offset_hz),
                Some(refinement.offset_bins),
            )
        })
        .unwrap_or((None, None, None));
    let (refined_code_phase_samples, code_phase_refinement_samples) = result
        .code_phase_refinement
        .as_ref()
        .map(|refinement| {
            (Some(refinement.refined_code_phase_samples), Some(refinement.offset_samples))
        })
        .unwrap_or((None, None));
    let (doppler_uncertainty_hz, code_phase_uncertainty_samples) = result
        .uncertainty
        .as_ref()
        .map(|uncertainty| (Some(uncertainty.doppler_hz), Some(uncertainty.code_phase_samples)))
        .unwrap_or((None, None));
    AcquisitionRow {
        sat: result.sat,
        signal_band: result.signal_band,
        source_sample_index: result.source_time.sample_index,
        doppler_hz: result.doppler_hz.0,
        candidate_rank: result.candidate_rank,
        is_primary_candidate: result.is_primary_candidate,
        carrier_hz: result.carrier_hz.0,
        coarse_carrier_hz,
        doppler_refinement_hz,
        doppler_refinement_bins,
        doppler_uncertainty_hz,
        code_phase_samples: result.code_phase_samples,
        refined_code_phase_samples,
        code_phase_refinement_samples,
        code_phase_uncertainty_samples,
        peak: result.peak,
        peak_mean_ratio: result.peak_mean_ratio,
        peak_second_ratio: result.peak_second_ratio,
        hypothesis: result.hypothesis.to_string(),
        selection_reason: result.explain_selection_reason.clone(),
    }
}

fn handle_pvt(command: GnssCommand) -> Result<()> {
    let GnssCommand::Pvt { common, obs, eph, ekf } = command else {
        bail!("invalid command for handler");
    };

    let runtime = runtime_config_from_env(&common, None);
    let profile = load_config(&common)?;
    let dataset = load_dataset(&common)?;
    let obs_epochs = read_obs_epochs(&obs)?;
    let broadcast_navigation = read_broadcast_navigation_data(&eph)?;
    let ephs = broadcast_navigation.ephemerides;
    let mut lines = Vec::new();
    let mut solutions = Vec::new();
    let mut timing_lines = Vec::new();
    let header = artifact_header(&common, &profile, dataset.as_ref())?;
    let pipeline_config = profile.to_pipeline_config();
    let mut nav = if !ekf {
        Some(bijux_gnss_infra::api::receiver::Navigation::new(
            pipeline_config.clone(),
            runtime.clone(),
        ))
    } else {
        None
    };
    let mut ekf_ctx = if ekf {
        Some(EkfContext::new_with_troposphere(
            pipeline_config.tropo_enable,
            pipeline_config.tropo_ztd_m,
            EkfScienceThresholds {
                max_pdop: pipeline_config.science_thresholds.max_pdop,
                max_gdop: pipeline_config.science_thresholds.max_gdop,
                min_used_satellites: pipeline_config.science_thresholds.min_used_satellites,
            },
        ))
    } else {
        None
    };
    for obs_epoch in obs_epochs {
        let solution = if ekf {
            solve_epoch_ekf(
                &mut ekf_ctx,
                &obs_epoch,
                &ephs,
                broadcast_navigation.klobuchar.as_ref(),
            )?
        } else {
            nav.as_mut().and_then(|nav| {
                nav.solve_epoch_with_broadcast_ionosphere(
                    &obs_epoch,
                    &ephs,
                    broadcast_navigation.klobuchar.as_ref(),
                )
            })
        };
        if let Some(solution) = solution {
            solutions.push(solution.clone());
            if let Some(ms) = solution.processing_ms {
                timing_lines.push(serde_json::to_string(&serde_json::json!({
                    "epoch_idx": solution.epoch.index,
                    "stage": "nav",
                    "processing_ms": ms
                }))?);
            }
            let wrapped = NavSolutionEpochV1 { header: header.clone(), payload: solution };
            let line = serde_json::to_string(&wrapped)?;
            lines.push(line);
        }
    }
    let out_dir = artifacts_dir(&common, "pvt", dataset.as_ref())?;
    let path = out_dir.join("pvt.jsonl");
    fs::write(&path, lines.join("\n"))?;
    validate_jsonl_schema(&schema_path("nav_solution_epoch_v1.schema.json"), &path, false)?;
    write_nav_solution_outputs(&out_dir, &solutions)?;
    if !timing_lines.is_empty() {
        let timing_path = out_dir.join("timing_nav.jsonl");
        fs::write(&timing_path, timing_lines.join("\n"))?;
    }
    let report = serde_json::json!({
        "epochs": lines.len(),
        "ekf": ekf
    });
    write_manifest(&common, "pvt", &profile, dataset.as_ref(), &report)?;

    Ok(())
}

fn handle_experiment(command: GnssCommand) -> Result<()> {
    let GnssCommand::Experiment { common, scenario, sweep } = command else {
        bail!("invalid command for handler");
    };

    let runtime = runtime_config_from_env(&common, None);
    let mut profile = load_config(&common)?;
    apply_common_overrides(
        &mut profile,
        CommonOverrides { seed: common.seed, deterministic: common.deterministic },
    );
    profile.validate().map_err(|errs| eyre!("invalid config before sweep: {}", errs.join(", ")))?;

    let scenario_contents = fs::read_to_string(&scenario)
        .with_context(|| format!("failed to read scenario {}", scenario.display()))?;
    let scenario_def: bijux_gnss_infra::api::receiver::sim::SyntheticScenario =
        toml::from_str(&scenario_contents)?;

    let sweep_spec = parse_sweep(&sweep)?;
    let runs = expand_sweep(&sweep_spec);
    let out_dir = artifacts_dir(&common, "experiment", None)?;

    let mut results = Vec::new();
    for (idx, overrides) in runs.iter().enumerate() {
        let mut run_profile = profile.clone();
        for (key, value) in overrides {
            apply_sweep_value(&mut run_profile, key, value)?;
        }
        run_profile
            .validate()
            .map_err(|errs| eyre!("invalid config in sweep: {}", errs.join(", ")))?;

        let config = run_profile.to_pipeline_config();
        let start = std::time::Instant::now();
        let frame =
            bijux_gnss_infra::api::receiver::sim::generate_l1_ca_multi(&config, &scenario_def);
        let sats: Vec<SatId> = scenario_def.satellites.iter().map(|s| s.sat).collect();
        let acquisition = AcquisitionEngine::new(config.clone(), runtime.clone());
        let acquisitions = acquisition.run_fft(&frame, &sats);
        let tracking =
            bijux_gnss_infra::api::receiver::TrackingEngine::new(config.clone(), runtime.clone());
        let tracks = tracking.track_from_acquisition(&frame, &acquisitions);
        let obs_report = bijux_gnss_infra::api::receiver::observations_from_tracking_results(
            &config,
            &tracks,
            run_profile.navigation.hatch_window,
        );
        let obs = obs_report.output;
        for event in obs_report.events {
            runtime.logger.event(&event);
        }
        let mut nav =
            bijux_gnss_infra::api::receiver::Navigation::new(config.clone(), runtime.clone());
        let mut solutions = Vec::new();
        for epoch in &obs {
            if let Some(solution) = nav.solve_epoch(epoch, &scenario_def.ephemerides) {
                solutions.push(solution);
            }
        }
        let elapsed = start.elapsed().as_secs_f64();
        let epochs = obs.len().max(1) as f64;
        let ms_per_epoch = (elapsed * 1000.0) / epochs;

        let lock_total: usize = tracks.iter().map(|t| t.epochs.len()).sum();
        let lock_good: usize =
            tracks.iter().map(|t| t.epochs.iter().filter(|e| e.lock).count()).sum();
        let lock_pct = if lock_total > 0 { lock_good as f64 / lock_total as f64 } else { 0.0 };

        let mean_pvt_rms = if solutions.is_empty() {
            0.0
        } else {
            solutions.iter().map(|s| s.rms_m.0).sum::<f64>() / solutions.len() as f64
        };
        let mean_residual_rms = mean_pvt_rms;
        let rejected_count =
            solutions.iter().flat_map(|s| s.residuals.iter().filter(|r| r.rejected)).count();

        let config_hash = hash_config(common.config.as_ref(), &run_profile)?;
        let result = ExperimentRunResult {
            run_index: idx,
            config_hash,
            scenario_id: scenario_def.id.clone(),
            overrides: overrides.clone(),
            lock_pct,
            pvt_rms_m: mean_pvt_rms,
            residual_rms_m: mean_residual_rms,
            rejected_count,
            ms_per_epoch,
        };
        write_experiment_run(&out_dir, idx, &result, &obs, &solutions)?;
        results.push(result);
    }

    let summary = ExperimentSummary { runs: results.clone() };
    let summary_path = out_dir.join("experiment_summary.json");
    fs::write(&summary_path, serde_json::to_string_pretty(&summary)?)?;
    let schema_path = schema_path("experiment_summary.schema.json");
    if schema_path.exists() {
        validate_json_schema(&schema_path, &summary_path, false)?;
    }
    write_manifest(&common, "experiment", &profile, None, &summary)?;

    Ok(())
}

fn handle_validatesidecar(command: GnssCommand) -> Result<()> {
    let GnssCommand::ValidateSidecar { common, sidecar_file } = command else {
        bail!("invalid command for handler");
    };

    let _ = runtime_config_from_env(&common, None);
    let spec = bijux_gnss_infra::api::load_raw_iq_metadata(&sidecar_file)?;
    validate_sidecar_schema(&spec)?;
    println!("sidecar ok: {}", sidecar_file.display());
    let dataset = load_dataset(&common)?;
    let mut manifest_common = common.clone();
    manifest_common.sidecar = Some(sidecar_file.clone());
    let mut manifest_profile = ReceiverConfig::default();
    apply_raw_iq_metadata(&mut manifest_profile, &spec, None, None)?;
    let summary = serde_json::json!({ "sidecar": sidecar_file.display().to_string() });
    write_manifest(
        &manifest_common,
        "validate_sidecar",
        &manifest_profile,
        dataset.as_ref(),
        &summary,
    )?;

    Ok(())
}

fn handle_run(command: GnssCommand) -> Result<()> {
    let GnssCommand::Run { common, file, replay, rate } = command else {
        bail!("invalid command for handler");
    };

    let dataset = load_dataset(&common)?;
    let runtime = runtime_config_from_capture_start(
        &common,
        None,
        dataset.as_ref().and_then(|entry| entry.capture_start_utc.as_deref()),
    );
    let mut profile = load_config(&common)?;
    apply_common_overrides(
        &mut profile,
        CommonOverrides { seed: common.seed, deterministic: common.deterministic },
    );
    let raw_iq_metadata = resolve_raw_iq_metadata(&common, dataset.as_ref())?;
    apply_raw_iq_metadata(&mut profile, &raw_iq_metadata, None, None)?;
    validate_config(&profile)?;
    let config = profile.to_pipeline_config();
    let input_file = resolve_input_file(file.as_ref(), dataset.as_ref())?;
    let signal_quality = measure_signal_quality_from_raw_iq(&input_file, &raw_iq_metadata, 0)?;
    let receiver = Receiver::new(config.clone(), runtime);
    let mut source = FileSamples::open_raw_iq(&input_file, raw_iq_metadata.clone())?;
    let artifacts = receiver.run(&mut source)?;
    if replay && rate > 0.0 {
        let dt = (artifacts.processed_input_epochs as f64 * 0.001) / rate;
        std::thread::sleep(std::time::Duration::from_secs_f64(dt));
    }
    let report = StreamingRunReport {
        epochs: artifacts.processed_input_epochs,
        processed_input_samples: artifacts.processed_input_samples,
        acquisitions: artifacts.acquisitions.len(),
        tracked_channels: artifacts
            .tracking
            .iter()
            .filter(|track| !track.epochs.is_empty())
            .count(),
        observation_epochs: artifacts.observations.len(),
        front_end_metrics: signal_quality.front_end_metrics.clone(),
        signal_quality,
    };
    emit_report(&common, "run", &report)?;
    write_signal_quality_report(&common, "run", &report.signal_quality)?;
    write_manifest(&common, "run", &profile, dataset.as_ref(), &report)?;

    Ok(())
}

#[cfg(test)]
mod pvt_tests {
    use super::*;
    use bijux_gnss_infra::api::core::{
        signal_spec_gps_l1_ca, Constellation, GpsTime, LockFlags, ObsEpoch, ObsMetadata,
        ObsSatellite, ObsSignalTiming, ObservationEpochDecision, ObservationStatus, ReceiverRole,
        ReceiverSampleTrace, SatId, Seconds, SigId, SignalBand, SignalCode,
    };
    use bijux_gnss_infra::api::nav::{
        ecef_to_geodetic, elevation_azimuth_deg, parse_rinex_nav, sat_state_gps_l1ca,
        write_rinex_broadcast_navigation, write_rinex_nav, GpsBroadcastNavigationData,
        GpsEphemeris, GpsL1CaHowWord, GpsL1CaLnavDecodedSubframe, GpsL1CaLnavSubframe1Clock,
        GpsL1CaLnavSubframe2Orbit, GpsL1CaLnavSubframe3Orbit, GpsL1CaLnavSubframeAlignment,
        GpsL1CaTlmWord, GpsL1CaWordParitySummary, IonosphereModel, KlobucharCoefficients,
        KlobucharModel, SaastamoinenModel, TroposphereModel,
    };
    use std::collections::BTreeMap;
    use std::fs;
    use std::path::{Path, PathBuf};
    use std::time::{SystemTime, UNIX_EPOCH};

    const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;

    struct SyntheticPvtCase {
        obs_epoch: ObsEpoch,
        truth_ecef_m: (f64, f64, f64),
        receiver_clock_bias_s: f64,
    }

    #[derive(Debug, Clone, Copy, Default)]
    struct BroadcastClockParameters {
        af0_s: f64,
        af1_s_per_s: f64,
        af2_s_per_s2: f64,
        tgd_s: f64,
    }

    #[derive(Debug, Clone, Copy, Default)]
    struct SyntheticSatelliteAdjustment {
        pseudorange_bias_m: f64,
        pseudorange_sigma_m: Option<f64>,
        cn0_dbhz: Option<f64>,
        elevation_deg: Option<f64>,
    }

    fn sample_common_args(out: PathBuf) -> CommonArgs {
        CommonArgs {
            config: None,
            dataset: None,
            unregistered_dataset: true,
            out: Some(out),
            report: ReportFormat::Json,
            seed: None,
            deterministic: true,
            dump: None,
            sidecar: None,
            resume: None,
        }
    }

    fn sample_common_args_with_troposphere(out: PathBuf, tropo_enabled: bool) -> CommonArgs {
        let mut common = sample_common_args(out.clone());
        if tropo_enabled == ReceiverConfig::default().navigation.tropo_enable {
            return common;
        }
        common.config = Some(write_receiver_config(&out, |profile| {
            profile.navigation.tropo_enable = tropo_enabled;
        }));
        common
    }

    fn sample_common_args_with_profile(
        out: PathBuf,
        configure: impl FnOnce(&mut ReceiverConfig),
    ) -> CommonArgs {
        let mut common = sample_common_args(out.clone());
        common.config = Some(write_receiver_config(&out, configure));
        common
    }

    fn write_receiver_config(root: &Path, configure: impl FnOnce(&mut ReceiverConfig)) -> PathBuf {
        let path = root.join("receiver.toml");
        let mut profile = ReceiverConfig::default();
        configure(&mut profile);
        fs::write(&path, toml::to_string_pretty(&profile).expect("serialize receiver config"))
            .expect("write receiver config");
        path
    }

    fn sample_ephemerides() -> Vec<GpsEphemeris> {
        sample_ephemerides_with_clock_parameters(&[])
    }

    fn sample_ephemerides_with_clock_parameters(
        clock_parameters_by_prn: &[(u8, BroadcastClockParameters)],
    ) -> Vec<GpsEphemeris> {
        let clock_parameters_by_prn =
            clock_parameters_by_prn.iter().copied().collect::<BTreeMap<_, _>>();
        [(1, 0.0, 0.0), (2, 0.8, 0.9), (3, 1.6, 1.8), (4, 2.4, 2.7), (5, 3.2, 3.6)]
            .into_iter()
            .map(|(prn, omega0, m0)| {
                let clock_parameters =
                    clock_parameters_by_prn.get(&prn).copied().unwrap_or_default();
                GpsEphemeris {
                    sat: SatId { constellation: Constellation::Gps, prn },
                    iodc: 1,
                    iode: 1,
                    week: 2209,
                    sv_health: 0,
                    toe_s: 504_000.0,
                    toc_s: 504_018.0,
                    sqrt_a: 5153.7954775,
                    e: 0.01,
                    i0: 0.94,
                    idot: 0.0,
                    omega0,
                    omegadot: 0.0,
                    w: 0.0,
                    m0,
                    delta_n: 0.0,
                    cuc: 0.0,
                    cus: 0.0,
                    crc: 0.0,
                    crs: 0.0,
                    cic: 0.0,
                    cis: 0.0,
                    af0: clock_parameters.af0_s,
                    af1: clock_parameters.af1_s_per_s,
                    af2: clock_parameters.af2_s_per_s2,
                    tgd: clock_parameters.tgd_s,
                }
            })
            .collect()
    }

    fn clear_broadcast_clock_parameters(ephemerides: &[GpsEphemeris]) -> Vec<GpsEphemeris> {
        ephemerides
            .iter()
            .cloned()
            .map(|mut ephemeris| {
                ephemeris.af0 = 0.0;
                ephemeris.af1 = 0.0;
                ephemeris.af2 = 0.0;
                ephemeris.tgd = 0.0;
                ephemeris
            })
            .collect()
    }

    fn sample_klobuchar_coefficients() -> KlobucharCoefficients {
        KlobucharCoefficients::new(
            [0.1212e-7, 0.1490e-7, -0.5960e-7, 0.1192e-6],
            [0.1167e6, -0.2294e6, -0.1311e6, 0.1049e7],
        )
    }

    fn broadcast_clock_fixture_parameters() -> [(u8, BroadcastClockParameters); 5] {
        [
            (
                1,
                BroadcastClockParameters {
                    af0_s: 240.0e-9,
                    af1_s_per_s: 0.0,
                    af2_s_per_s2: 0.0,
                    tgd_s: -8.0e-9,
                },
            ),
            (
                2,
                BroadcastClockParameters {
                    af0_s: -170.0e-9,
                    af1_s_per_s: 0.0,
                    af2_s_per_s2: 0.0,
                    tgd_s: 6.0e-9,
                },
            ),
            (
                3,
                BroadcastClockParameters {
                    af0_s: 330.0e-9,
                    af1_s_per_s: 0.0,
                    af2_s_per_s2: 0.0,
                    tgd_s: -4.0e-9,
                },
            ),
            (
                4,
                BroadcastClockParameters {
                    af0_s: -95.0e-9,
                    af1_s_per_s: 0.0,
                    af2_s_per_s2: 0.0,
                    tgd_s: 10.0e-9,
                },
            ),
            (
                5,
                BroadcastClockParameters {
                    af0_s: 210.0e-9,
                    af1_s_per_s: 0.0,
                    af2_s_per_s2: 0.0,
                    tgd_s: -5.0e-9,
                },
            ),
        ]
    }

    fn broadcast_clock_pvt_case(
        receiver_clock_bias_s: f64,
    ) -> (Vec<GpsEphemeris>, SyntheticPvtCase) {
        let ephemerides =
            sample_ephemerides_with_clock_parameters(&broadcast_clock_fixture_parameters());
        let pvt_case = sample_pvt_case(&ephemerides, receiver_clock_bias_s);
        (ephemerides, pvt_case)
    }

    fn sample_pvt_case(ephs: &[GpsEphemeris], receiver_clock_bias_s: f64) -> SyntheticPvtCase {
        sample_pvt_case_with_adjustments(ephs, receiver_clock_bias_s, &[])
    }

    fn sample_pvt_case_with_adjustments(
        ephs: &[GpsEphemeris],
        receiver_clock_bias_s: f64,
        adjustments: &[(u8, SyntheticSatelliteAdjustment)],
    ) -> SyntheticPvtCase {
        add_saastamoinen_delay_to_pvt_case(
            &sample_troposphere_free_pvt_case_with_adjustments(
                ephs,
                receiver_clock_bias_s,
                adjustments,
            ),
            ephs,
        )
    }

    fn sample_troposphere_free_pvt_case(
        ephs: &[GpsEphemeris],
        receiver_clock_bias_s: f64,
    ) -> SyntheticPvtCase {
        sample_troposphere_free_pvt_case_with_adjustments(ephs, receiver_clock_bias_s, &[])
    }

    fn sample_troposphere_free_pvt_case_with_adjustments(
        ephs: &[GpsEphemeris],
        receiver_clock_bias_s: f64,
        adjustments: &[(u8, SyntheticSatelliteAdjustment)],
    ) -> SyntheticPvtCase {
        let t_rx_s = 504_018.07 + receiver_clock_bias_s;
        let (rx_x, rx_y, rx_z) = bijux_gnss_infra::api::nav::geodetic_to_ecef(37.0, -122.0, 10.0);
        let adjustments_by_prn =
            adjustments.iter().copied().collect::<BTreeMap<u8, SyntheticSatelliteAdjustment>>();
        let sats = ephs
            .iter()
            .map(|eph| {
                let adjustment = adjustments_by_prn.get(&eph.sat.prn).copied().unwrap_or_default();
                let mut tau = 0.07;
                let mut pseudorange_m = 0.0;
                for _ in 0..10 {
                    let state = sat_state_gps_l1ca(eph, t_rx_s - tau, tau);
                    let dx = rx_x - state.x_m;
                    let dy = rx_y - state.y_m;
                    let dz = rx_z - state.z_m;
                    let range = (dx * dx + dy * dy + dz * dz).sqrt();
                    pseudorange_m = range + receiver_clock_bias_s * SPEED_OF_LIGHT_MPS
                        - state.clock_correction.bias_s * SPEED_OF_LIGHT_MPS;
                    let next_tau = pseudorange_m / SPEED_OF_LIGHT_MPS;
                    if (next_tau - tau).abs() < 1.0e-12 {
                        break;
                    }
                    tau = next_tau;
                }
                pseudorange_m += adjustment.pseudorange_bias_m;
                let signal_travel_time_s = pseudorange_m / SPEED_OF_LIGHT_MPS;
                let pseudorange_sigma_m = adjustment.pseudorange_sigma_m.unwrap_or(2.0);
                ObsSatellite {
                    signal_id: SigId { sat: eph.sat, band: SignalBand::L1, code: SignalCode::Ca },
                    pseudorange_m: bijux_gnss_infra::api::core::Meters(pseudorange_m),
                    pseudorange_var_m2: pseudorange_sigma_m.powi(2),
                    carrier_phase_cycles: bijux_gnss_infra::api::core::Cycles(
                        1_000.0 + eph.sat.prn as f64,
                    ),
                    carrier_phase_var_cycles2: 0.01,
                    doppler_hz: bijux_gnss_infra::api::core::Hertz(-500.0),
                    doppler_var_hz2: 4.0,
                    cn0_dbhz: adjustment.cn0_dbhz.unwrap_or(45.0),
                    lock_flags: LockFlags {
                        code_lock: true,
                        carrier_lock: true,
                        bit_lock: false,
                        cycle_slip: false,
                    },
                    multipath_suspect: false,
                    observation_status: ObservationStatus::Accepted,
                    observation_reject_reasons: Vec::new(),
                    elevation_deg: adjustment.elevation_deg,
                    azimuth_deg: None,
                    weight: None,
                    timing: Some(ObsSignalTiming {
                        signal_travel_time_s: Seconds(signal_travel_time_s),
                        transmit_gps_time: GpsTime {
                            week: 2209,
                            tow_s: t_rx_s - signal_travel_time_s,
                        },
                    }),
                    error_model: None,
                    metadata: ObsMetadata {
                        tracking_mode: "test".to_string(),
                        integration_ms: 1,
                        lock_quality: 45.0,
                        smoothing_window: 0,
                        smoothing_age: 0,
                        smoothing_resets: 0,
                        signal: signal_spec_gps_l1_ca(),
                        ..ObsMetadata::default()
                    },
                }
            })
            .collect();

        SyntheticPvtCase {
            obs_epoch: ObsEpoch {
                t_rx_s: Seconds(t_rx_s),
                source_time: ReceiverSampleTrace::from_sample_index(1, 1_000.0),
                gps_week: Some(2209),
                tow_s: Some(Seconds(t_rx_s)),
                epoch_idx: 1,
                discontinuity: false,
                valid: true,
                processing_ms: None,
                role: ReceiverRole::Rover,
                sats,
                decision: ObservationEpochDecision::Accepted,
                decision_reason: Some("accepted_observables_present".to_string()),
                manifest: None,
            },
            truth_ecef_m: (rx_x, rx_y, rx_z),
            receiver_clock_bias_s,
        }
    }

    fn add_saastamoinen_delay_to_pvt_case(
        pvt_case: &SyntheticPvtCase,
        ephs: &[GpsEphemeris],
    ) -> SyntheticPvtCase {
        let (lat_deg, lon_deg, alt_m) = ecef_to_geodetic(
            pvt_case.truth_ecef_m.0,
            pvt_case.truth_ecef_m.1,
            pvt_case.truth_ecef_m.2,
        );
        let receiver = bijux_gnss_infra::api::core::Llh { lat_deg, lon_deg, alt_m };
        let model = SaastamoinenModel;
        let mut biased_epoch = pvt_case.obs_epoch.clone();
        for sat in &mut biased_epoch.sats {
            let ephemeris = ephs
                .iter()
                .find(|ephemeris| ephemeris.sat == sat.signal_id.sat)
                .expect("matching ephemeris");
            let timing = sat.timing.expect("timing");
            let state = sat_state_gps_l1ca(
                ephemeris,
                timing.transmit_gps_time.tow_s,
                timing.signal_travel_time_s.0,
            );
            let (_azimuth_deg, elevation_deg) = elevation_azimuth_deg(
                pvt_case.truth_ecef_m.0,
                pvt_case.truth_ecef_m.1,
                pvt_case.truth_ecef_m.2,
                state.x_m,
                state.y_m,
                state.z_m,
            );
            let delay_m =
                model.delay_m(receiver, elevation_deg, Seconds(pvt_case.obs_epoch.t_rx_s.0));
            let delay_s = delay_m / SPEED_OF_LIGHT_MPS;
            sat.pseudorange_m.0 += delay_m;
            sat.timing = Some(ObsSignalTiming {
                signal_travel_time_s: Seconds(timing.signal_travel_time_s.0 + delay_s),
                transmit_gps_time: timing.transmit_gps_time.offset_seconds(-delay_s),
            });
        }
        SyntheticPvtCase {
            obs_epoch: biased_epoch,
            truth_ecef_m: pvt_case.truth_ecef_m,
            receiver_clock_bias_s: pvt_case.receiver_clock_bias_s,
        }
    }

    fn sparse_followup_obs_epoch(base: &ObsEpoch) -> ObsEpoch {
        let mut obs = base.clone();
        obs.epoch_idx += 1;
        obs.source_time = ReceiverSampleTrace::from_sample_index(
            base.source_time.sample_index + 1_000,
            base.source_time.sample_rate_hz,
        );
        obs.sats.truncate(3);
        obs
    }

    fn position_error_3d_m(
        solution: &bijux_gnss_infra::api::core::NavSolutionEpoch,
        truth_ecef_m: (f64, f64, f64),
    ) -> f64 {
        let dx = solution.ecef_x_m.0 - truth_ecef_m.0;
        let dy = solution.ecef_y_m.0 - truth_ecef_m.1;
        let dz = solution.ecef_z_m.0 - truth_ecef_m.2;
        (dx * dx + dy * dy + dz * dz).sqrt()
    }

    fn iterative_pseudorange_residual_with_earth_rotation_mode_m(
        eph: &GpsEphemeris,
        sat: &ObsSatellite,
        receiver_ecef_m: (f64, f64, f64),
        receiver_clock_bias_s: f64,
        receive_tow_s: f64,
        apply_earth_rotation: bool,
    ) -> f64 {
        let mut tau = sat
            .timing
            .map(|timing| timing.signal_travel_time_s.0)
            .unwrap_or(sat.pseudorange_m.0 / SPEED_OF_LIGHT_MPS);
        let mut predicted_pseudorange_m = 0.0;
        for _ in 0..10 {
            let state = sat_state_gps_l1ca(
                eph,
                receive_tow_s - tau,
                if apply_earth_rotation { tau } else { 0.0 },
            );
            let dx = receiver_ecef_m.0 - state.x_m;
            let dy = receiver_ecef_m.1 - state.y_m;
            let dz = receiver_ecef_m.2 - state.z_m;
            let range_m = (dx * dx + dy * dy + dz * dz).sqrt();
            predicted_pseudorange_m = range_m + receiver_clock_bias_s * SPEED_OF_LIGHT_MPS
                - state.clock_correction.bias_s * SPEED_OF_LIGHT_MPS;
            let next_tau = predicted_pseudorange_m / SPEED_OF_LIGHT_MPS;
            if (next_tau - tau).abs() < 1.0e-12 {
                break;
            }
            tau = next_tau;
        }
        sat.pseudorange_m.0 - predicted_pseudorange_m
    }

    fn pvt_residual_rms_with_earth_rotation_mode_m(
        solution: &bijux_gnss_infra::api::core::NavSolutionEpoch,
        ephs: &[GpsEphemeris],
        pvt_case: &SyntheticPvtCase,
        apply_earth_rotation: bool,
    ) -> f64 {
        let receive_tow_s = pvt_case.obs_epoch.tow_s.expect("receive tow").0;
        let residual_sum_squares_m2 = pvt_case
            .obs_epoch
            .sats
            .iter()
            .map(|sat| {
                let ephemeris = ephs
                    .iter()
                    .find(|ephemeris| ephemeris.sat == sat.signal_id.sat)
                    .expect("matching ephemeris");
                let residual_m = iterative_pseudorange_residual_with_earth_rotation_mode_m(
                    ephemeris,
                    sat,
                    (solution.ecef_x_m.0, solution.ecef_y_m.0, solution.ecef_z_m.0),
                    solution.clock_bias_s.0,
                    receive_tow_s,
                    apply_earth_rotation,
                );
                residual_m * residual_m
            })
            .sum::<f64>();
        (residual_sum_squares_m2 / pvt_case.obs_epoch.sats.len() as f64).sqrt()
    }

    fn solve_pvt_case_with_rinex_nav(
        case_name: &str,
        ephs: &[GpsEphemeris],
        pvt_case: &SyntheticPvtCase,
    ) -> bijux_gnss_infra::api::core::NavSolutionEpoch {
        solve_pvt_case_with_rinex_nav_mode_and_troposphere(case_name, ephs, pvt_case, false, true)
    }

    fn solve_pvt_case_with_rinex_nav_mode(
        case_name: &str,
        ephs: &[GpsEphemeris],
        pvt_case: &SyntheticPvtCase,
        ekf: bool,
    ) -> bijux_gnss_infra::api::core::NavSolutionEpoch {
        solve_pvt_case_with_rinex_nav_mode_and_troposphere(case_name, ephs, pvt_case, ekf, true)
    }

    fn solve_pvt_case_with_rinex_nav_mode_and_troposphere(
        case_name: &str,
        ephs: &[GpsEphemeris],
        pvt_case: &SyntheticPvtCase,
        ekf: bool,
        tropo_enabled: bool,
    ) -> bijux_gnss_infra::api::core::NavSolutionEpoch {
        let mut solutions = solve_obs_epochs_with_rinex_nav_mode_and_troposphere(
            case_name,
            ephs,
            &[pvt_case.obs_epoch.clone()],
            ekf,
            tropo_enabled,
        );
        assert_eq!(solutions.len(), 1);
        solutions.remove(0)
    }

    fn solve_pvt_case_with_rinex_nav_profile_and_troposphere(
        case_name: &str,
        ephs: &[GpsEphemeris],
        pvt_case: &SyntheticPvtCase,
        ekf: bool,
        tropo_enabled: bool,
        configure: impl FnOnce(&mut ReceiverConfig),
    ) -> bijux_gnss_infra::api::core::NavSolutionEpoch {
        let mut solutions = solve_obs_epochs_with_rinex_nav_profile_and_troposphere(
            case_name,
            ephs,
            &[pvt_case.obs_epoch.clone()],
            ekf,
            tropo_enabled,
            configure,
        );
        assert_eq!(solutions.len(), 1);
        solutions.remove(0)
    }

    fn solve_obs_epochs_with_rinex_nav_mode_and_troposphere(
        case_name: &str,
        ephs: &[GpsEphemeris],
        obs_epochs: &[ObsEpoch],
        ekf: bool,
        tropo_enabled: bool,
    ) -> Vec<bijux_gnss_infra::api::core::NavSolutionEpoch> {
        let root = std::env::temp_dir().join(format!(
            "bijux_pvt_{}_{}_{}",
            case_name,
            std::process::id(),
            SystemTime::now().duration_since(UNIX_EPOCH).expect("unix epoch").as_nanos()
        ));
        fs::create_dir_all(&root).expect("create test root");
        let obs_path = root.join("obs.jsonl");
        let eph_path = root.join("nav.rnx");
        fs::write(
            &obs_path,
            obs_epochs
                .iter()
                .map(|obs_epoch| {
                    serde_json::to_string(obs_epoch).expect("serialize observation epoch")
                })
                .collect::<Vec<_>>()
                .join("\n")
                + "\n",
        )
        .expect("write obs");
        write_rinex_nav(&eph_path, ephs, true).expect("write rinex nav");

        let common = sample_common_args_with_troposphere(root.clone(), tropo_enabled);
        let out_dir = artifacts_dir(&common, "pvt", None).expect("artifacts dir");
        fs::create_dir_all(&out_dir).expect("create pvt artifacts dir");
        handle_pvt(GnssCommand::Pvt { common: common.clone(), obs: obs_path, eph: eph_path, ekf })
            .expect("pvt command");

        let nav_path = out_dir.join("pvt.jsonl");
        let solutions = read_nav_solutions(&nav_path).expect("read nav solutions");
        fs::remove_dir_all(root).expect("remove test root");
        solutions
    }

    #[test]
    fn pvt_command_emits_all_dops_in_nav_solution_output() {
        let ephemerides = sample_ephemerides();
        let pvt_case = sample_pvt_case(&ephemerides, 2.75e-4);
        let root = std::env::temp_dir().join(format!(
            "bijux_pvt_output_dops_{}_{}",
            std::process::id(),
            SystemTime::now().duration_since(UNIX_EPOCH).expect("unix epoch").as_nanos()
        ));
        fs::create_dir_all(&root).expect("create test root");
        let obs_path = root.join("obs.jsonl");
        let eph_path = root.join("nav.rnx");
        fs::write(
            &obs_path,
            format!(
                "{}\n",
                serde_json::to_string(&pvt_case.obs_epoch).expect("serialize observation epoch")
            ),
        )
        .expect("write obs");
        write_rinex_nav(&eph_path, &ephemerides, true).expect("write rinex nav");

        let common = sample_common_args_with_troposphere(root.clone(), true);
        let out_dir = artifacts_dir(&common, "pvt", None).expect("artifacts dir");
        fs::create_dir_all(&out_dir).expect("create pvt artifacts dir");
        handle_pvt(GnssCommand::Pvt {
            common: common.clone(),
            obs: obs_path,
            eph: eph_path,
            ekf: false,
        })
        .expect("pvt command");

        let mut solutions =
            read_nav_solutions(&out_dir.join("pvt.jsonl")).expect("read nav solutions");
        assert_eq!(solutions.len(), 1);
        let solution = solutions.remove(0);
        let nav_output = fs::read_to_string(out_dir.join("nav_solution.jsonl"))
            .expect("read nav solution output");
        let payload: serde_json::Value =
            serde_json::from_str(nav_output.lines().next().expect("nav solution output line"))
                .expect("parse nav solution output");

        assert_eq!(payload["dops"]["pdop"].as_f64().expect("pdop"), solution.pdop);
        assert_eq!(
            payload["dops"]["hdop"].as_f64().expect("hdop"),
            solution.hdop.expect("solution hdop")
        );
        assert_eq!(
            payload["dops"]["vdop"].as_f64().expect("vdop"),
            solution.vdop.expect("solution vdop")
        );
        assert_eq!(
            payload["dops"]["gdop"].as_f64().expect("gdop"),
            solution.gdop.expect("solution gdop")
        );
        assert_eq!(
            payload["dops"]["tdop"].as_f64().expect("tdop"),
            solution.tdop.expect("solution tdop")
        );

        fs::remove_dir_all(root).expect("remove test root");
    }

    fn solve_obs_epochs_with_rinex_nav_profile_and_troposphere(
        case_name: &str,
        ephs: &[GpsEphemeris],
        obs_epochs: &[ObsEpoch],
        ekf: bool,
        tropo_enabled: bool,
        configure: impl FnOnce(&mut ReceiverConfig),
    ) -> Vec<bijux_gnss_infra::api::core::NavSolutionEpoch> {
        let root = std::env::temp_dir().join(format!(
            "bijux_pvt_profile_{}_{}_{}",
            case_name,
            std::process::id(),
            SystemTime::now().duration_since(UNIX_EPOCH).expect("unix epoch").as_nanos()
        ));
        fs::create_dir_all(&root).expect("create test root");
        let obs_path = root.join("obs.jsonl");
        let eph_path = root.join("nav.rnx");
        fs::write(
            &obs_path,
            obs_epochs
                .iter()
                .map(|obs_epoch| {
                    serde_json::to_string(obs_epoch).expect("serialize observation epoch")
                })
                .collect::<Vec<_>>()
                .join("\n")
                + "\n",
        )
        .expect("write obs");
        write_rinex_nav(&eph_path, ephs, true).expect("write rinex nav");

        let common = sample_common_args_with_profile(root.clone(), |profile| {
            profile.navigation.tropo_enable = tropo_enabled;
            configure(profile);
        });
        let out_dir = artifacts_dir(&common, "pvt", None).expect("artifacts dir");
        fs::create_dir_all(&out_dir).expect("create pvt artifacts dir");
        handle_pvt(GnssCommand::Pvt { common: common.clone(), obs: obs_path, eph: eph_path, ekf })
            .expect("pvt command");

        let nav_path = out_dir.join("pvt.jsonl");
        let solutions = read_nav_solutions(&nav_path).expect("read nav solutions");
        fs::remove_dir_all(root).expect("remove test root");
        solutions
    }

    fn solve_pvt_case_with_decoded_lnav_reports(
        case_name: &str,
        ephs: &[GpsEphemeris],
        pvt_case: &SyntheticPvtCase,
    ) -> bijux_gnss_infra::api::core::NavSolutionEpoch {
        solve_pvt_case_with_decoded_lnav_reports_and_troposphere(case_name, ephs, pvt_case, true)
    }

    fn solve_pvt_case_with_decoded_lnav_reports_and_troposphere(
        case_name: &str,
        ephs: &[GpsEphemeris],
        pvt_case: &SyntheticPvtCase,
        tropo_enabled: bool,
    ) -> bijux_gnss_infra::api::core::NavSolutionEpoch {
        let root = std::env::temp_dir().join(format!(
            "bijux_pvt_{}_{}_{}",
            case_name,
            std::process::id(),
            SystemTime::now().duration_since(UNIX_EPOCH).expect("unix epoch").as_nanos()
        ));
        fs::create_dir_all(&root).expect("create test root");
        let obs_path = root.join("obs.jsonl");
        let eph_path = root.join("nav_decode_reports.json");
        fs::write(
            &obs_path,
            format!(
                "{}\n",
                serde_json::to_string(&pvt_case.obs_epoch).expect("serialize observation epoch")
            ),
        )
        .expect("write obs");
        let reports = ephs
            .iter()
            .map(|eph| {
                serde_json::json!({
                    "sat": eph.sat,
                    "reference_week": eph.week,
                    "decoded_subframes": decoded_lnav_subframes_from_ephemeris(eph),
                    "ephemerides": []
                })
            })
            .collect::<Vec<_>>();
        fs::write(
            &eph_path,
            serde_json::to_string_pretty(&reports).expect("serialize nav decode reports"),
        )
        .expect("write nav decode reports");

        let common = sample_common_args_with_troposphere(root.clone(), tropo_enabled);
        let out_dir = artifacts_dir(&common, "pvt", None).expect("artifacts dir");
        fs::create_dir_all(&out_dir).expect("create pvt artifacts dir");
        handle_pvt(GnssCommand::Pvt {
            common: common.clone(),
            obs: obs_path,
            eph: eph_path,
            ekf: false,
        })
        .expect("pvt command");

        let nav_path = out_dir.join("pvt.jsonl");
        let mut solutions = read_nav_solutions(&nav_path).expect("read nav solutions");
        fs::remove_dir_all(root).expect("remove test root");

        assert_eq!(solutions.len(), 1);
        solutions.remove(0)
    }

    fn solve_pvt_case_with_broadcast_navigation_data_and_troposphere(
        case_name: &str,
        navigation: &GpsBroadcastNavigationData,
        pvt_case: &SyntheticPvtCase,
        ekf: bool,
        tropo_enabled: bool,
    ) -> bijux_gnss_infra::api::core::NavSolutionEpoch {
        let root = std::env::temp_dir().join(format!(
            "bijux_pvt_broadcast_navigation_{}_{}_{}",
            case_name,
            std::process::id(),
            SystemTime::now().duration_since(UNIX_EPOCH).expect("unix epoch").as_nanos()
        ));
        fs::create_dir_all(&root).expect("create test root");
        let obs_path = root.join("obs.jsonl");
        let eph_path = root.join("broadcast_navigation.json");
        fs::write(
            &obs_path,
            format!(
                "{}\n",
                serde_json::to_string(&pvt_case.obs_epoch).expect("serialize observation epoch")
            ),
        )
        .expect("write obs");
        fs::write(
            &eph_path,
            serde_json::to_string_pretty(navigation).expect("serialize broadcast navigation"),
        )
        .expect("write broadcast navigation");

        let common = sample_common_args_with_troposphere(root.clone(), tropo_enabled);
        let out_dir = artifacts_dir(&common, "pvt", None).expect("artifacts dir");
        fs::create_dir_all(&out_dir).expect("create pvt artifacts dir");
        handle_pvt(GnssCommand::Pvt { common: common.clone(), obs: obs_path, eph: eph_path, ekf })
            .expect("pvt command");

        let nav_path = out_dir.join("pvt.jsonl");
        let mut solutions = read_nav_solutions(&nav_path).expect("read nav solutions");
        fs::remove_dir_all(root).expect("remove test root");

        assert_eq!(solutions.len(), 1);
        solutions.remove(0)
    }

    fn solve_pvt_case_with_rinex_broadcast_navigation_data_and_troposphere(
        case_name: &str,
        navigation: &GpsBroadcastNavigationData,
        pvt_case: &SyntheticPvtCase,
        ekf: bool,
        tropo_enabled: bool,
    ) -> bijux_gnss_infra::api::core::NavSolutionEpoch {
        let root = std::env::temp_dir().join(format!(
            "bijux_pvt_rinex_broadcast_navigation_{}_{}_{}",
            case_name,
            std::process::id(),
            SystemTime::now().duration_since(UNIX_EPOCH).expect("unix epoch").as_nanos()
        ));
        fs::create_dir_all(&root).expect("create test root");
        let obs_path = root.join("obs.jsonl");
        let eph_path = root.join("nav.rnx");
        fs::write(
            &obs_path,
            format!(
                "{}\n",
                serde_json::to_string(&pvt_case.obs_epoch).expect("serialize observation epoch")
            ),
        )
        .expect("write obs");
        write_rinex_broadcast_navigation(&eph_path, navigation, true)
            .expect("write rinex broadcast navigation");

        let common = sample_common_args_with_troposphere(root.clone(), tropo_enabled);
        let out_dir = artifacts_dir(&common, "pvt", None).expect("artifacts dir");
        fs::create_dir_all(&out_dir).expect("create pvt artifacts dir");
        handle_pvt(GnssCommand::Pvt { common: common.clone(), obs: obs_path, eph: eph_path, ekf })
            .expect("pvt command");

        let nav_path = out_dir.join("pvt.jsonl");
        let mut solutions = read_nav_solutions(&nav_path).expect("read nav solutions");
        fs::remove_dir_all(root).expect("remove test root");

        assert_eq!(solutions.len(), 1);
        solutions.remove(0)
    }

    fn add_klobuchar_delay_to_pvt_case(
        pvt_case: &SyntheticPvtCase,
        ephs: &[GpsEphemeris],
        klobuchar: KlobucharCoefficients,
    ) -> SyntheticPvtCase {
        let (lat_deg, lon_deg, alt_m) = ecef_to_geodetic(
            pvt_case.truth_ecef_m.0,
            pvt_case.truth_ecef_m.1,
            pvt_case.truth_ecef_m.2,
        );
        let receiver = bijux_gnss_infra::api::core::Llh { lat_deg, lon_deg, alt_m };
        let model = KlobucharModel::new(klobuchar);
        let mut biased_epoch = pvt_case.obs_epoch.clone();
        for sat in &mut biased_epoch.sats {
            let ephemeris = ephs
                .iter()
                .find(|ephemeris| ephemeris.sat == sat.signal_id.sat)
                .expect("matching ephemeris");
            let timing = sat.timing.expect("timing");
            let state = sat_state_gps_l1ca(
                ephemeris,
                timing.transmit_gps_time.tow_s,
                timing.signal_travel_time_s.0,
            );
            let (azimuth_deg, elevation_deg) = elevation_azimuth_deg(
                pvt_case.truth_ecef_m.0,
                pvt_case.truth_ecef_m.1,
                pvt_case.truth_ecef_m.2,
                state.x_m,
                state.y_m,
                state.z_m,
            );
            let delay_m = model.delay_m(
                receiver,
                azimuth_deg,
                elevation_deg,
                Seconds(pvt_case.obs_epoch.t_rx_s.0),
            );
            let delay_s = delay_m / SPEED_OF_LIGHT_MPS;
            sat.pseudorange_m.0 += delay_m;
            sat.timing = Some(ObsSignalTiming {
                signal_travel_time_s: Seconds(timing.signal_travel_time_s.0 + delay_s),
                transmit_gps_time: timing.transmit_gps_time.offset_seconds(-delay_s),
            });
        }
        SyntheticPvtCase {
            obs_epoch: biased_epoch,
            truth_ecef_m: pvt_case.truth_ecef_m,
            receiver_clock_bias_s: pvt_case.receiver_clock_bias_s,
        }
    }

    fn decoded_lnav_subframes_from_ephemeris(
        eph: &GpsEphemeris,
    ) -> Vec<GpsL1CaLnavDecodedSubframe> {
        let alignment = |subframe_index: usize| GpsL1CaLnavSubframeAlignment {
            start_bit_index: subframe_index * 300,
            end_bit_index_exclusive: (subframe_index + 1) * 300,
            start_prompt_index: subframe_index * 6_000,
            end_prompt_index_exclusive: (subframe_index + 1) * 6_000,
            inverted: false,
            word_count: 10,
            parity_ok_count: 10,
        };
        let tlm = GpsL1CaTlmWord { preamble: 0x8B, parity_ok: true };
        let parity = GpsL1CaWordParitySummary {
            word_count: 10,
            passed_word_count: 10,
            failed_word_indexes: Vec::new(),
        };
        let word_parity_ok = vec![true; 10];

        vec![
            GpsL1CaLnavDecodedSubframe {
                alignment: alignment(0),
                tlm: tlm.clone(),
                how: GpsL1CaHowWord {
                    tow_count: (eph.toc_s / 6.0) as u32,
                    tow_start_s: (eph.toc_s / 6.0) as u32 * 6,
                    alert: false,
                    anti_spoof: false,
                    subframe_id: 1,
                    parity_ok: true,
                },
                clock: Some(GpsL1CaLnavSubframe1Clock {
                    week: (eph.week % 1024) as u16,
                    iodc: eph.iodc,
                    sv_health: eph.sv_health,
                    toc_s: eph.toc_s,
                    af0: eph.af0,
                    af1: eph.af1,
                    af2: eph.af2,
                    tgd: eph.tgd,
                }),
                orbit_subframe_2: None,
                orbit_subframe_3: None,
                parity: parity.clone(),
                word_parity_ok: word_parity_ok.clone(),
            },
            GpsL1CaLnavDecodedSubframe {
                alignment: alignment(1),
                tlm: tlm.clone(),
                how: GpsL1CaHowWord {
                    tow_count: (eph.toe_s / 6.0) as u32,
                    tow_start_s: (eph.toe_s / 6.0) as u32 * 6,
                    alert: false,
                    anti_spoof: false,
                    subframe_id: 2,
                    parity_ok: true,
                },
                clock: None,
                orbit_subframe_2: Some(GpsL1CaLnavSubframe2Orbit {
                    iode: eph.iode,
                    crs: eph.crs,
                    delta_n: eph.delta_n,
                    m0: eph.m0,
                    cuc: eph.cuc,
                    e: eph.e,
                    cus: eph.cus,
                    sqrt_a: eph.sqrt_a,
                    toe_s: eph.toe_s,
                }),
                orbit_subframe_3: None,
                parity: parity.clone(),
                word_parity_ok: word_parity_ok.clone(),
            },
            GpsL1CaLnavDecodedSubframe {
                alignment: alignment(2),
                tlm,
                how: GpsL1CaHowWord {
                    tow_count: (eph.toe_s / 6.0) as u32 + 1,
                    tow_start_s: ((eph.toe_s / 6.0) as u32 + 1) * 6,
                    alert: false,
                    anti_spoof: false,
                    subframe_id: 3,
                    parity_ok: true,
                },
                clock: None,
                orbit_subframe_2: None,
                orbit_subframe_3: Some(GpsL1CaLnavSubframe3Orbit {
                    iode: eph.iode,
                    cic: eph.cic,
                    omega0: eph.omega0,
                    cis: eph.cis,
                    i0: eph.i0,
                    crc: eph.crc,
                    w: eph.w,
                    omegadot: eph.omegadot,
                    idot: eph.idot,
                }),
                parity,
                word_parity_ok,
            },
        ]
    }

    #[test]
    fn pvt_command_accepts_rinex_navigation_input() {
        let root = std::env::temp_dir().join(format!(
            "bijux_pvt_rinex_nav_{}_{}",
            std::process::id(),
            SystemTime::now().duration_since(UNIX_EPOCH).expect("unix epoch").as_nanos()
        ));
        fs::create_dir_all(&root).expect("create test root");
        let obs_path = root.join("obs.jsonl");
        let eph_path = root.join("nav.rnx");

        let ephs = sample_ephemerides();
        let pvt_case = sample_pvt_case(&ephs, 2.75e-4);
        fs::write(
            &obs_path,
            format!("{}\n", serde_json::to_string(&pvt_case.obs_epoch).expect("serialize obs")),
        )
        .expect("write obs");
        write_rinex_nav(&eph_path, &ephs, true).expect("write rinex nav");
        let rinex = fs::read_to_string(&eph_path).expect("read rinex nav");
        assert_eq!(parse_rinex_nav(&rinex).expect("parse rinex nav").len(), ephs.len());

        let common = sample_common_args(root.clone());
        handle_pvt(GnssCommand::Pvt {
            common: common.clone(),
            obs: obs_path,
            eph: eph_path,
            ekf: false,
        })
        .expect("pvt command");

        let nav_path =
            artifacts_dir(&common, "pvt", None).expect("artifacts dir").join("pvt.jsonl");
        let solutions = read_nav_solutions(&nav_path).expect("read nav solutions");
        let raw_nav_line = fs::read_to_string(&nav_path).expect("read raw nav artifact");
        let raw_nav: serde_json::Value =
            serde_json::from_str(raw_nav_line.lines().next().unwrap_or("")).expect("parse raw nav");

        assert_eq!(solutions.len(), 1);
        assert!(solutions[0].valid);
        assert_eq!(solutions[0].sat_count, 5);
        assert_eq!(solutions[0].used_sat_count, 5);
        assert_eq!(solutions[0].rejected_sat_count, 0);
        assert!((solutions[0].ecef_x_m.0 - pvt_case.truth_ecef_m.0).abs() < 5.0);
        assert!((solutions[0].ecef_y_m.0 - pvt_case.truth_ecef_m.1).abs() < 5.0);
        assert!((solutions[0].ecef_z_m.0 - pvt_case.truth_ecef_m.2).abs() < 5.0);
        assert!(solutions[0].clock_bias_s.0.is_finite());
        assert!((solutions[0].clock_bias_s.0 - pvt_case.receiver_clock_bias_s).abs() < 1.0e-9);
        assert!(
            (solutions[0].clock_bias_m.0 - pvt_case.receiver_clock_bias_s * SPEED_OF_LIGHT_MPS)
                .abs()
                < 1.0e-6
        );
        assert!(
            (raw_nav["payload"]["clock_bias_s"].as_f64().expect("clock bias seconds")
                - pvt_case.receiver_clock_bias_s)
                .abs()
                < 1.0e-9
        );
        assert!(
            (raw_nav["payload"]["clock_bias_m"].as_f64().expect("clock bias meters")
                - pvt_case.receiver_clock_bias_s * SPEED_OF_LIGHT_MPS)
                .abs()
                < 1.0e-6
        );

        fs::remove_dir_all(root).expect("remove test root");
    }

    #[test]
    fn pvt_command_accepts_decoded_lnav_report_array() {
        let root = std::env::temp_dir().join(format!(
            "bijux_pvt_decoded_lnav_{}_{}",
            std::process::id(),
            SystemTime::now().duration_since(UNIX_EPOCH).expect("unix epoch").as_nanos()
        ));
        fs::create_dir_all(&root).expect("create test root");
        let obs_path = root.join("obs.jsonl");
        let eph_path = root.join("nav_decode_reports.json");

        let ephs = sample_ephemerides();
        let pvt_case = sample_pvt_case(&ephs, 2.75e-4);
        fs::write(
            &obs_path,
            format!("{}\n", serde_json::to_string(&pvt_case.obs_epoch).expect("serialize obs")),
        )
        .expect("write obs");
        let reports = ephs
            .iter()
            .map(|eph| {
                serde_json::json!({
                    "sat": eph.sat,
                    "reference_week": eph.week,
                    "decoded_subframes": decoded_lnav_subframes_from_ephemeris(eph),
                    "ephemerides": []
                })
            })
            .collect::<Vec<_>>();
        fs::write(
            &eph_path,
            serde_json::to_string_pretty(&reports).expect("serialize nav decode reports"),
        )
        .expect("write nav decode reports");

        let common = sample_common_args(root.clone());
        handle_pvt(GnssCommand::Pvt {
            common: common.clone(),
            obs: obs_path,
            eph: eph_path,
            ekf: false,
        })
        .expect("pvt command");

        let nav_path =
            artifacts_dir(&common, "pvt", None).expect("artifacts dir").join("pvt.jsonl");
        let solutions = read_nav_solutions(&nav_path).expect("read nav solutions");
        let raw_nav_line = fs::read_to_string(&nav_path).expect("read raw nav artifact");
        let raw_nav: serde_json::Value =
            serde_json::from_str(raw_nav_line.lines().next().unwrap_or("")).expect("parse raw nav");

        assert_eq!(solutions.len(), 1);
        assert!(solutions[0].valid);
        assert_eq!(solutions[0].sat_count, 5);
        assert_eq!(solutions[0].used_sat_count, 5);
        assert_eq!(solutions[0].rejected_sat_count, 0);
        assert!((solutions[0].ecef_x_m.0 - pvt_case.truth_ecef_m.0).abs() < 5.0);
        assert!((solutions[0].ecef_y_m.0 - pvt_case.truth_ecef_m.1).abs() < 5.0);
        assert!((solutions[0].ecef_z_m.0 - pvt_case.truth_ecef_m.2).abs() < 5.0);
        assert!(solutions[0].clock_bias_s.0.is_finite());
        assert!((solutions[0].clock_bias_s.0 - pvt_case.receiver_clock_bias_s).abs() < 1.0e-9);
        assert!(
            (solutions[0].clock_bias_m.0 - pvt_case.receiver_clock_bias_s * SPEED_OF_LIGHT_MPS)
                .abs()
                < 1.0e-6
        );
        assert!(
            (raw_nav["payload"]["clock_bias_s"].as_f64().expect("clock bias seconds")
                - pvt_case.receiver_clock_bias_s)
                .abs()
                < 1.0e-9
        );
        assert!(
            (raw_nav["payload"]["clock_bias_m"].as_f64().expect("clock bias meters")
                - pvt_case.receiver_clock_bias_s * SPEED_OF_LIGHT_MPS)
                .abs()
                < 1.0e-6
        );

        fs::remove_dir_all(root).expect("remove test root");
    }

    #[test]
    fn pvt_command_refuses_sparse_followup_epoch_in_wls_mode() {
        let ephemerides = sample_ephemerides();
        let first_case = sample_pvt_case(&ephemerides, 2.75e-4);
        let second_obs = sparse_followup_obs_epoch(&first_case.obs_epoch);

        let solutions = solve_obs_epochs_with_rinex_nav_mode_and_troposphere(
            "sparse_followup_wls_refusal",
            &ephemerides,
            &[first_case.obs_epoch.clone(), second_obs],
            false,
            true,
        );

        assert_eq!(solutions.len(), 2);
        assert!(solutions[0].valid);
        assert_eq!(solutions[1].status, bijux_gnss_infra::api::core::SolutionStatus::Invalid);
        assert_eq!(
            solutions[1].refusal_class,
            Some(bijux_gnss_infra::api::core::NavRefusalClass::InsufficientGeometry)
        );
        assert!(solutions[1].used_sat_count < 4);
        assert_eq!(
            solutions[1].sat_count,
            solutions[1].used_sat_count + solutions[1].rejected_sat_count
        );
        assert_eq!(solutions[1].ecef_x_m.0, 0.0);
        assert_eq!(solutions[1].ecef_y_m.0, 0.0);
        assert_eq!(solutions[1].ecef_z_m.0, 0.0);
        assert!(!solutions[1].valid);
    }

    #[test]
    fn pvt_command_refuses_sparse_followup_epoch_in_ekf_mode() {
        let ephemerides = sample_ephemerides();
        let first_case = sample_pvt_case(&ephemerides, 2.75e-4);
        let second_obs = sparse_followup_obs_epoch(&first_case.obs_epoch);

        let solutions = solve_obs_epochs_with_rinex_nav_mode_and_troposphere(
            "sparse_followup_ekf_refusal",
            &ephemerides,
            &[first_case.obs_epoch.clone(), second_obs],
            true,
            true,
        );

        assert_eq!(solutions.len(), 2);
        assert!(solutions[0].valid);
        assert_eq!(solutions[1].status, bijux_gnss_infra::api::core::SolutionStatus::Invalid);
        assert_eq!(
            solutions[1].refusal_class,
            Some(bijux_gnss_infra::api::core::NavRefusalClass::InsufficientGeometry)
        );
        assert_eq!(solutions[1].sat_count, 3);
        assert_eq!(solutions[1].used_sat_count, 3);
        assert_eq!(solutions[1].ecef_x_m.0, 0.0);
        assert_eq!(solutions[1].ecef_y_m.0, 0.0);
        assert_eq!(solutions[1].ecef_z_m.0, 0.0);
        assert!(solutions[1]
            .explain_reasons
            .iter()
            .any(|reason| reason == "minimum_usable_satellites=4"));
        assert!(!solutions[1].valid);
    }

    #[test]
    fn pvt_command_refuses_wls_solution_when_gdop_exceeds_science_threshold() {
        let ephemerides = sample_ephemerides();
        let pvt_case = sample_pvt_case(&ephemerides, 2.75e-4);
        let baseline_solution = solve_pvt_case_with_rinex_nav_mode(
            "wls_geometry_threshold_baseline",
            &ephemerides,
            &pvt_case,
            false,
        );
        let baseline_gdop = baseline_solution.gdop.expect("baseline gdop");

        let refused_solution = solve_pvt_case_with_rinex_nav_profile_and_troposphere(
            "wls_geometry_threshold_refusal",
            &ephemerides,
            &pvt_case,
            false,
            true,
            |profile| {
                profile.navigation.science_thresholds.max_pdop = 100.0;
                profile.navigation.science_thresholds.max_gdop = baseline_gdop - 0.01;
            },
        );

        assert!(baseline_solution.valid);
        assert_eq!(refused_solution.status, bijux_gnss_infra::api::core::SolutionStatus::Invalid);
        assert_eq!(
            refused_solution.refusal_class,
            Some(bijux_gnss_infra::api::core::NavRefusalClass::InsufficientGeometry)
        );
        assert!((refused_solution.gdop.expect("refused gdop") - baseline_gdop).abs() < 1.0e-9);
        assert!(refused_solution
            .explain_reasons
            .iter()
            .any(|reason| reason.starts_with("gdop_above_threshold:")));
        assert_eq!(refused_solution.ecef_x_m.0, 0.0);
        assert_eq!(refused_solution.ecef_y_m.0, 0.0);
        assert_eq!(refused_solution.ecef_z_m.0, 0.0);
        assert!(!refused_solution.valid);
    }

    #[test]
    fn pvt_command_refuses_ekf_solution_when_pdop_exceeds_science_threshold() {
        let ephemerides = sample_ephemerides();
        let pvt_case = sample_pvt_case(&ephemerides, 2.75e-4);
        let baseline_solution = solve_pvt_case_with_rinex_nav_mode(
            "ekf_geometry_threshold_baseline",
            &ephemerides,
            &pvt_case,
            true,
        );

        let refused_solution = solve_pvt_case_with_rinex_nav_profile_and_troposphere(
            "ekf_geometry_threshold_refusal",
            &ephemerides,
            &pvt_case,
            true,
            true,
            |profile| {
                profile.navigation.science_thresholds.max_pdop = baseline_solution.pdop - 0.01;
                profile.navigation.science_thresholds.max_gdop = 100.0;
            },
        );

        assert!(baseline_solution.valid);
        assert_eq!(refused_solution.status, bijux_gnss_infra::api::core::SolutionStatus::Invalid);
        assert_eq!(
            refused_solution.refusal_class,
            Some(bijux_gnss_infra::api::core::NavRefusalClass::InsufficientGeometry)
        );
        assert!((refused_solution.pdop - baseline_solution.pdop).abs() < 1.0e-9);
        assert!(refused_solution
            .explain_reasons
            .iter()
            .any(|reason| reason.starts_with("pdop_above_threshold:")));
        assert_eq!(refused_solution.ecef_x_m.0, 0.0);
        assert_eq!(refused_solution.ecef_y_m.0, 0.0);
        assert_eq!(refused_solution.ecef_z_m.0, 0.0);
        assert!(!refused_solution.valid);
    }

    #[test]
    fn pvt_command_uses_broadcast_satellite_clock_correction_from_rinex_navigation() {
        let (ephemerides, pvt_case) = broadcast_clock_pvt_case(2.75e-4);

        let corrected_solution = solve_pvt_case_with_rinex_nav(
            "broadcast_clock_rinex_corrected",
            &ephemerides,
            &pvt_case,
        );
        let zero_clock_solution = solve_pvt_case_with_rinex_nav(
            "broadcast_clock_rinex_zero_clock",
            &clear_broadcast_clock_parameters(&ephemerides),
            &pvt_case,
        );

        let corrected_error_m = position_error_3d_m(&corrected_solution, pvt_case.truth_ecef_m);
        let zero_clock_error_m = position_error_3d_m(&zero_clock_solution, pvt_case.truth_ecef_m);

        assert!(corrected_solution.valid);
        assert!(corrected_error_m < 5.0);
        assert!(zero_clock_error_m > corrected_error_m + 20.0);
        assert!(
            (corrected_solution.clock_bias_s.0 - pvt_case.receiver_clock_bias_s).abs() < 1.0e-9
        );
    }

    #[test]
    fn pvt_command_rinex_solution_prefers_earth_rotation_correction() {
        let ephemerides = sample_ephemerides();
        let pvt_case = sample_troposphere_free_pvt_case(&ephemerides, 2.75e-4);
        let solution = solve_pvt_case_with_rinex_nav_mode_and_troposphere(
            "earth_rotation_rinex",
            &ephemerides,
            &pvt_case,
            false,
            false,
        );

        let corrected_rms_m =
            pvt_residual_rms_with_earth_rotation_mode_m(&solution, &ephemerides, &pvt_case, true);
        let uncorrected_rms_m =
            pvt_residual_rms_with_earth_rotation_mode_m(&solution, &ephemerides, &pvt_case, false);

        assert!(solution.valid);
        assert!(position_error_3d_m(&solution, pvt_case.truth_ecef_m) < 5.0);
        assert!(corrected_rms_m < 1.0e-3);
        assert!(uncorrected_rms_m > corrected_rms_m + 1.0);
    }

    #[test]
    fn pvt_command_uses_broadcast_satellite_clock_correction_from_decoded_lnav() {
        let (ephemerides, pvt_case) = broadcast_clock_pvt_case(2.75e-4);

        let corrected_solution = solve_pvt_case_with_decoded_lnav_reports(
            "broadcast_clock_decoded_lnav_corrected",
            &ephemerides,
            &pvt_case,
        );
        let zero_clock_solution = solve_pvt_case_with_decoded_lnav_reports(
            "broadcast_clock_decoded_lnav_zero_clock",
            &clear_broadcast_clock_parameters(&ephemerides),
            &pvt_case,
        );

        let corrected_error_m = position_error_3d_m(&corrected_solution, pvt_case.truth_ecef_m);
        let zero_clock_error_m = position_error_3d_m(&zero_clock_solution, pvt_case.truth_ecef_m);

        assert!(corrected_solution.valid);
        assert!(corrected_error_m < 5.0);
        assert!(zero_clock_error_m > corrected_error_m + 20.0);
        assert!(
            (corrected_solution.clock_bias_s.0 - pvt_case.receiver_clock_bias_s).abs() < 1.0e-9
        );
    }

    #[test]
    fn pvt_command_decoded_lnav_solution_prefers_earth_rotation_correction() {
        let ephemerides = sample_ephemerides();
        let pvt_case = sample_troposphere_free_pvt_case(&ephemerides, 2.75e-4);
        let solution = solve_pvt_case_with_decoded_lnav_reports_and_troposphere(
            "earth_rotation_decoded_lnav",
            &ephemerides,
            &pvt_case,
            false,
        );

        let corrected_rms_m =
            pvt_residual_rms_with_earth_rotation_mode_m(&solution, &ephemerides, &pvt_case, true);
        let uncorrected_rms_m =
            pvt_residual_rms_with_earth_rotation_mode_m(&solution, &ephemerides, &pvt_case, false);

        assert!(solution.valid);
        assert!(position_error_3d_m(&solution, pvt_case.truth_ecef_m) < 5.0);
        assert!(corrected_rms_m < 1.0e-3);
        assert!(uncorrected_rms_m > corrected_rms_m + 1.0);
    }

    #[test]
    fn pvt_command_downweights_high_variance_satellite() {
        let ephs = sample_ephemerides();
        let low_variance_case = sample_pvt_case_with_adjustments(
            &ephs,
            2.75e-4,
            &[(
                5,
                SyntheticSatelliteAdjustment {
                    pseudorange_bias_m: 80.0,
                    pseudorange_sigma_m: Some(2.0),
                    ..SyntheticSatelliteAdjustment::default()
                },
            )],
        );
        let high_variance_case = sample_pvt_case_with_adjustments(
            &ephs,
            2.75e-4,
            &[(
                5,
                SyntheticSatelliteAdjustment {
                    pseudorange_bias_m: 80.0,
                    pseudorange_sigma_m: Some(200.0),
                    ..SyntheticSatelliteAdjustment::default()
                },
            )],
        );

        let low_variance_solution =
            solve_pvt_case_with_rinex_nav("low_variance_bias", &ephs, &low_variance_case);
        let high_variance_solution =
            solve_pvt_case_with_rinex_nav("high_variance_bias", &ephs, &high_variance_case);

        let low_variance_error_m =
            position_error_3d_m(&low_variance_solution, low_variance_case.truth_ecef_m);
        let high_variance_error_m =
            position_error_3d_m(&high_variance_solution, high_variance_case.truth_ecef_m);
        let provenance = high_variance_solution.provenance.as_ref().expect("nav provenance");

        assert!(high_variance_solution.valid);
        assert_eq!(provenance.solver_family, "wls_weighted");
        assert_eq!(provenance.weighting_mode, "cn0_elevation_sigma_weighted");
        assert!(high_variance_error_m < low_variance_error_m);
    }

    #[test]
    fn pvt_command_downweights_low_cn0_low_elevation_satellite() {
        let ephs = sample_ephemerides();
        let strong_signal_case = sample_pvt_case_with_adjustments(
            &ephs,
            2.75e-4,
            &[(
                5,
                SyntheticSatelliteAdjustment {
                    pseudorange_bias_m: 40.0,
                    pseudorange_sigma_m: Some(2.0),
                    cn0_dbhz: Some(55.0),
                    elevation_deg: Some(70.0),
                },
            )],
        );
        let weak_signal_case = sample_pvt_case_with_adjustments(
            &ephs,
            2.75e-4,
            &[(
                5,
                SyntheticSatelliteAdjustment {
                    pseudorange_bias_m: 40.0,
                    pseudorange_sigma_m: Some(2.0),
                    cn0_dbhz: Some(30.0),
                    elevation_deg: Some(15.0),
                },
            )],
        );

        let strong_signal_solution =
            solve_pvt_case_with_rinex_nav("strong_signal_bias", &ephs, &strong_signal_case);
        let weak_signal_solution =
            solve_pvt_case_with_rinex_nav("weak_signal_bias", &ephs, &weak_signal_case);

        let strong_signal_error_m =
            position_error_3d_m(&strong_signal_solution, strong_signal_case.truth_ecef_m);
        let weak_signal_error_m =
            position_error_3d_m(&weak_signal_solution, weak_signal_case.truth_ecef_m);
        let provenance = weak_signal_solution.provenance.as_ref().expect("nav provenance");

        assert!(weak_signal_solution.valid);
        assert_eq!(provenance.solver_family, "wls_weighted");
        assert_eq!(provenance.weighting_mode, "cn0_elevation_sigma_weighted");
        assert!(weak_signal_error_m < strong_signal_error_m);
    }

    #[test]
    fn pvt_command_marks_wls_solution_ionosphere_uncorrected_without_broadcast_model() {
        let ephemerides = sample_ephemerides();
        let pvt_case = sample_troposphere_free_pvt_case(&ephemerides, 2.75e-4);
        let solution = solve_pvt_case_with_rinex_nav_mode_and_troposphere(
            "ionosphere_uncorrected_wls",
            &ephemerides,
            &pvt_case,
            false,
            false,
        );

        assert!(solution.explain_reasons.iter().any(|reason| reason == "ionosphere_uncorrected"));
    }

    #[test]
    fn pvt_command_applies_broadcast_ionosphere_correction_in_wls() {
        let ephemerides = sample_ephemerides();
        let klobuchar = sample_klobuchar_coefficients();
        let navigation = GpsBroadcastNavigationData {
            ephemerides: ephemerides.clone(),
            klobuchar: Some(klobuchar),
        };
        let clean_case = sample_troposphere_free_pvt_case(&ephemerides, 2.75e-4);
        let ionosphere_biased_case =
            add_klobuchar_delay_to_pvt_case(&clean_case, &ephemerides, klobuchar);

        let corrected_solution = solve_pvt_case_with_broadcast_navigation_data_and_troposphere(
            "broadcast_ionosphere_wls_corrected",
            &navigation,
            &ionosphere_biased_case,
            false,
            false,
        );
        let uncorrected_solution = solve_pvt_case_with_rinex_nav_mode_and_troposphere(
            "broadcast_ionosphere_wls_uncorrected",
            &ephemerides,
            &ionosphere_biased_case,
            false,
            false,
        );

        let corrected_error_m = position_error_3d_m(&corrected_solution, clean_case.truth_ecef_m);
        let uncorrected_error_m =
            position_error_3d_m(&uncorrected_solution, clean_case.truth_ecef_m);

        assert!(corrected_solution.valid);
        assert!(corrected_solution
            .explain_reasons
            .iter()
            .any(|reason| reason == "ionosphere_correction=klobuchar_broadcast"));
        assert!(corrected_error_m < 5.0);
        assert!(uncorrected_error_m > corrected_error_m + 3.0);
    }

    #[test]
    fn pvt_command_applies_rinex_broadcast_ionosphere_correction_in_wls() {
        let ephemerides = sample_ephemerides();
        let klobuchar = sample_klobuchar_coefficients();
        let navigation = GpsBroadcastNavigationData {
            ephemerides: ephemerides.clone(),
            klobuchar: Some(klobuchar),
        };
        let clean_case = sample_troposphere_free_pvt_case(&ephemerides, 2.75e-4);
        let ionosphere_biased_case =
            add_klobuchar_delay_to_pvt_case(&clean_case, &ephemerides, klobuchar);

        let corrected_solution =
            solve_pvt_case_with_rinex_broadcast_navigation_data_and_troposphere(
                "rinex_broadcast_ionosphere_wls_corrected",
                &navigation,
                &ionosphere_biased_case,
                false,
                false,
            );
        let uncorrected_solution = solve_pvt_case_with_rinex_nav_mode_and_troposphere(
            "rinex_broadcast_ionosphere_wls_uncorrected",
            &ephemerides,
            &ionosphere_biased_case,
            false,
            false,
        );

        let corrected_error_m = position_error_3d_m(&corrected_solution, clean_case.truth_ecef_m);
        let uncorrected_error_m =
            position_error_3d_m(&uncorrected_solution, clean_case.truth_ecef_m);

        assert!(corrected_solution.valid);
        assert!(corrected_solution
            .explain_reasons
            .iter()
            .any(|reason| reason == "ionosphere_correction=klobuchar_broadcast"));
        assert!(corrected_error_m < 5.0);
        assert!(uncorrected_error_m > corrected_error_m + 3.0);
    }

    #[test]
    fn pvt_command_marks_ekf_solution_ionosphere_uncorrected_without_broadcast_model() {
        let ephemerides = sample_ephemerides();
        let pvt_case = sample_troposphere_free_pvt_case(&ephemerides, 2.75e-4);
        let solution = solve_pvt_case_with_rinex_nav_mode_and_troposphere(
            "ionosphere_uncorrected_ekf",
            &ephemerides,
            &pvt_case,
            true,
            false,
        );

        assert!(solution.explain_reasons.iter().any(|reason| reason == "ionosphere_uncorrected"));
    }

    #[test]
    fn pvt_command_applies_broadcast_ionosphere_correction_in_ekf() {
        let ephemerides = sample_ephemerides();
        let klobuchar = sample_klobuchar_coefficients();
        let navigation = GpsBroadcastNavigationData {
            ephemerides: ephemerides.clone(),
            klobuchar: Some(klobuchar),
        };
        let clean_case = sample_troposphere_free_pvt_case(&ephemerides, 2.75e-4);
        let ionosphere_biased_case =
            add_klobuchar_delay_to_pvt_case(&clean_case, &ephemerides, klobuchar);

        let corrected_solution = solve_pvt_case_with_broadcast_navigation_data_and_troposphere(
            "broadcast_ionosphere_ekf_corrected",
            &navigation,
            &ionosphere_biased_case,
            true,
            false,
        );
        let uncorrected_solution = solve_pvt_case_with_rinex_nav_mode_and_troposphere(
            "broadcast_ionosphere_ekf_uncorrected",
            &ephemerides,
            &ionosphere_biased_case,
            true,
            false,
        );

        let corrected_error_m = position_error_3d_m(&corrected_solution, clean_case.truth_ecef_m);
        let uncorrected_error_m =
            position_error_3d_m(&uncorrected_solution, clean_case.truth_ecef_m);

        assert!(corrected_solution.valid);
        assert!(corrected_solution
            .explain_reasons
            .iter()
            .any(|reason| reason == "ionosphere_correction=klobuchar_broadcast"));
        assert!(corrected_error_m < uncorrected_error_m);
    }

    #[test]
    fn pvt_command_applies_rinex_broadcast_ionosphere_correction_in_ekf() {
        let ephemerides = sample_ephemerides();
        let klobuchar = sample_klobuchar_coefficients();
        let navigation = GpsBroadcastNavigationData {
            ephemerides: ephemerides.clone(),
            klobuchar: Some(klobuchar),
        };
        let clean_case = sample_troposphere_free_pvt_case(&ephemerides, 2.75e-4);
        let ionosphere_biased_case =
            add_klobuchar_delay_to_pvt_case(&clean_case, &ephemerides, klobuchar);

        let corrected_solution =
            solve_pvt_case_with_rinex_broadcast_navigation_data_and_troposphere(
                "rinex_broadcast_ionosphere_ekf_corrected",
                &navigation,
                &ionosphere_biased_case,
                true,
                false,
            );
        let uncorrected_solution = solve_pvt_case_with_rinex_nav_mode_and_troposphere(
            "rinex_broadcast_ionosphere_ekf_uncorrected",
            &ephemerides,
            &ionosphere_biased_case,
            true,
            false,
        );

        let corrected_error_m = position_error_3d_m(&corrected_solution, clean_case.truth_ecef_m);
        let uncorrected_error_m =
            position_error_3d_m(&uncorrected_solution, clean_case.truth_ecef_m);

        assert!(corrected_solution.valid);
        assert!(corrected_solution
            .explain_reasons
            .iter()
            .any(|reason| reason == "ionosphere_correction=klobuchar_broadcast"));
        assert!(corrected_error_m < uncorrected_error_m);
    }

    #[test]
    fn pvt_command_marks_wls_solution_troposphere_uncorrected_when_disabled() {
        let ephemerides = sample_ephemerides();
        let pvt_case = sample_troposphere_free_pvt_case(&ephemerides, 2.75e-4);
        let solution = solve_pvt_case_with_rinex_nav_mode_and_troposphere(
            "troposphere_uncorrected_wls",
            &ephemerides,
            &pvt_case,
            false,
            false,
        );

        assert!(solution.explain_reasons.iter().any(|reason| reason == "troposphere_uncorrected"));
    }

    #[test]
    fn pvt_command_applies_saastamoinen_troposphere_correction_in_wls() {
        let ephemerides = sample_ephemerides();
        let clean_case = sample_troposphere_free_pvt_case(&ephemerides, 2.75e-4);
        let troposphere_biased_case = add_saastamoinen_delay_to_pvt_case(&clean_case, &ephemerides);

        let corrected_solution = solve_pvt_case_with_rinex_nav(
            "saastamoinen_troposphere_wls_corrected",
            &ephemerides,
            &troposphere_biased_case,
        );
        let uncorrected_solution = solve_pvt_case_with_rinex_nav_mode_and_troposphere(
            "saastamoinen_troposphere_wls_uncorrected",
            &ephemerides,
            &troposphere_biased_case,
            false,
            false,
        );

        let corrected_error_m = position_error_3d_m(&corrected_solution, clean_case.truth_ecef_m);
        let uncorrected_error_m =
            position_error_3d_m(&uncorrected_solution, clean_case.truth_ecef_m);

        assert!(corrected_solution.valid);
        assert!(corrected_solution
            .explain_reasons
            .iter()
            .any(|reason| reason == "troposphere_correction=saastamoinen"));
        assert!(corrected_error_m < 5.0);
        assert!(uncorrected_error_m > corrected_error_m + 3.0);
    }

    #[test]
    fn pvt_command_applies_saastamoinen_troposphere_correction_in_ekf() {
        let ephemerides = sample_ephemerides();
        let clean_case = sample_troposphere_free_pvt_case(&ephemerides, 2.75e-4);
        let troposphere_biased_case = add_saastamoinen_delay_to_pvt_case(&clean_case, &ephemerides);

        let corrected_solution = solve_pvt_case_with_rinex_nav_mode(
            "saastamoinen_troposphere_ekf_corrected",
            &ephemerides,
            &troposphere_biased_case,
            true,
        );
        let uncorrected_solution = solve_pvt_case_with_rinex_nav_mode_and_troposphere(
            "saastamoinen_troposphere_ekf_uncorrected",
            &ephemerides,
            &troposphere_biased_case,
            true,
            false,
        );

        let corrected_error_m = position_error_3d_m(&corrected_solution, clean_case.truth_ecef_m);
        let uncorrected_error_m =
            position_error_3d_m(&uncorrected_solution, clean_case.truth_ecef_m);

        assert!(corrected_solution.valid);
        assert!(corrected_solution
            .explain_reasons
            .iter()
            .any(|reason| reason == "troposphere_correction=saastamoinen"));
        assert!(corrected_error_m < uncorrected_error_m);
    }
}
