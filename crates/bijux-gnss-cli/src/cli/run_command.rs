fn run_command(command: GnssCommand) -> Result<()> {
    match command {
        cmd @ GnssCommand::CaCode { .. } => handle_cacode(cmd),
        cmd @ GnssCommand::Acquire { .. } => handle_acquire(cmd),
        cmd @ GnssCommand::Track { .. } => handle_track(cmd),
        cmd @ GnssCommand::Nav { .. } => handle_nav(cmd),
        cmd @ GnssCommand::Pvt { .. } => handle_pvt(cmd),
        cmd @ GnssCommand::Inspect { .. } => handle_inspect(cmd),
        cmd @ GnssCommand::Rtk { .. } => handle_rtk(cmd),
        cmd @ GnssCommand::Experiment { .. } => handle_experiment(cmd),
        cmd @ GnssCommand::ExportSyntheticIq { .. } => handle_export_synthetic_iq(cmd),
        cmd @ GnssCommand::ValidateConfig { .. } => handle_validateconfig(cmd),
        cmd @ GnssCommand::Config { .. } => handle_config(cmd),
        cmd @ GnssCommand::ValidateArtifacts { .. } => handle_validateartifacts(cmd),
        cmd @ GnssCommand::ValidateSidecar { .. } => handle_validatesidecar(cmd),
        cmd @ GnssCommand::Analyze { .. } => handle_analyze(cmd),
        cmd @ GnssCommand::Diff { .. } => handle_diff(cmd),
        cmd @ GnssCommand::Artifact { .. } => handle_artifact(cmd),
        cmd @ GnssCommand::Diagnostics { .. } => handle_diagnostics(cmd),
        cmd @ GnssCommand::ConfigUpgrade { .. } => handle_configupgrade(cmd),
        cmd @ GnssCommand::ConfigSchema { .. } => handle_configschema(cmd),
        cmd @ GnssCommand::Validate { .. } => handle_validate(cmd),
        cmd @ GnssCommand::ValidateReference { .. } => handle_validate_reference(cmd),
        cmd @ GnssCommand::Run { .. } => handle_run(cmd),
        cmd @ GnssCommand::Rinex { .. } => handle_rinex(cmd),
        cmd @ GnssCommand::Doctor { .. } => handle_doctor(cmd),
    }
}

fn inspect_dataset(
    path: &Path,
    metadata: &RawIqMetadata,
    max_samples: usize,
) -> Result<InspectReport> {
    let mut source = FileSamples::open_raw_iq(path, metadata.clone())
        .with_context(|| format!("failed to open {}", path.display()))?;
    let mut total_iq = 0usize;

    let mut front_end_analyzer =
        bijux_gnss_infra::api::signal::IqFrontEndAnalyzer::for_raw_iq_metadata(metadata);
    let mut power_hist = vec![0u64; 8];
    let mut power_sum = 0.0f64;

    while max_samples == 0 || total_iq < max_samples {
        let frame_len = if max_samples == 0 { 4096 } else { (max_samples - total_iq).min(4096) };
        let Some(frame) = source.next_frame(frame_len)? else {
            break;
        };
        front_end_analyzer.update(&frame.iq);
        for sample in &frame.iq {
            let i = sample.re as f64;
            let q = sample.im as f64;
            let power = i * i + q * q;
            power_sum += power;
            let bin = ((power.sqrt() / 0.25).min(7.0)) as usize;
            power_hist[bin] += 1;
            total_iq += 1;
            if max_samples != 0 && total_iq >= max_samples {
                break;
            }
        }
    }

    let mean_power = power_sum / total_iq.max(1) as f64;
    let noise_floor_db = 10.0 * mean_power.max(1e-9).log10();

    Ok(InspectReport {
        format: format!("{:?}", metadata.format),
        sample_rate_hz: metadata.sample_rate_hz,
        intermediate_freq_hz: metadata.intermediate_freq_hz,
        capture_start_utc: metadata.capture_start_utc.clone(),
        total_samples: total_iq,
        front_end_metrics: front_end_analyzer.finish(),
        noise_floor_db,
        power_histogram: power_hist,
    })
}
fn runtime_config_from_env(
    common: &CommonArgs,
    run_dir: Option<PathBuf>,
) -> bijux_gnss_infra::api::receiver::ReceiverRuntime {
    if common.deterministic {
        std::env::set_var("RAYON_NUM_THREADS", "1");
        std::env::set_var("BIJUX_DETERMINISTIC", "1");
    }
    let config = bijux_gnss_infra::api::receiver::ReceiverRuntimeConfig {
        run_id: std::env::var("BIJUX_RUN_ID").ok(),
        trace_dir: common.dump.clone(),
        run_dir: run_dir.or_else(|| std::env::var("BIJUX_RUN_DIR").ok().map(PathBuf::from)),
        diagnostics_dump: std::env::var("BIJUX_DIAGNOSTICS_DUMP").ok().as_deref() == Some("1"),
    };
    bijux_gnss_infra::api::receiver::ReceiverRuntime::new(config)
}
use bijux_gnss_infra::api::core::format_sat;

fn format_optional_degrees(value: Option<f64>) -> String {
    value.map(|degrees| format!("{degrees:.6}")).unwrap_or_else(|| "n/a".to_string())
}

fn format_optional_percent(value: Option<f64>) -> String {
    value.map(|pct| format!("{pct:.3}")).unwrap_or_else(|| "n/a".to_string())
}

fn format_optional_reason(value: Option<&str>) -> String {
    value.unwrap_or("n/a").to_string()
}

fn format_reported_prns(report: &AcquisitionReport) -> String {
    if report.reported_prns.is_empty() {
        return "none".to_string();
    }

    report
        .reported_prns
        .iter()
        .map(|entry| {
            format!(
                "{} ({}, {:.1} Hz, peak/mean {:.2}, peak/2nd {:.2})",
                format_sat(entry.sat),
                entry.classification,
                entry.carrier_hz,
                entry.peak_mean_ratio,
                entry.peak_second_ratio,
            )
        })
        .collect::<Vec<_>>()
        .join(", ")
}

fn print_acquisition_table(report: &AcquisitionReport) {
    println!(
        "I mean: {:.6}  Q mean: {:.6}  I power: {:.6}  Q power: {:.6}  I/Q ratio: {:.6}  Power warning: {}  Quadrature error(deg): {}  Quadrature warning: {}  Clipping(%): {}  Clipping warning: {}  Centered RMS: {:.6e}  Zero-signal: {}  Zero-signal reason: {}  Precision claims allowed: {}  Precision refusal: {}  RMS: {:.6}  DC imbalance: {:.6}",
        report.front_end_metrics.i_mean,
        report.front_end_metrics.q_mean,
        report.front_end_metrics.i_power,
        report.front_end_metrics.q_power,
        report.front_end_metrics.iq_power_ratio,
        report.front_end_metrics.power_imbalance_warning,
        format_optional_degrees(report.front_end_metrics.quadrature_error_deg),
        report.front_end_metrics.quadrature_error_warning,
        format_optional_percent(report.front_end_metrics.clipping_pct),
        report.front_end_metrics.clipping_warning,
        report.front_end_metrics.centered_rms,
        report.front_end_metrics.zero_signal_detected,
        format_optional_reason(report.front_end_metrics.zero_signal_reason.as_deref()),
        report.front_end_metrics.precision_claims_allowed,
        format_optional_reason(
            report
                .front_end_metrics
                .precision_claims_refused_reason
                .as_deref(),
        ),
        report.front_end_metrics.rms,
        report.front_end_metrics.dc_imbalance
    );
    println!("Reported PRNs: {}", format_reported_prns(report));
    println!("Sat\tCarrier(Hz)\tCodePhase\tPeak\tPeak/Mean\tPeak/2nd\tHypothesis\tReason");
    for row in &report.results {
        println!(
            "{}\t{:.1}\t{}\t{:.3}\t{:.2}\t{:.2}\t{}\t{}",
            format_sat(row.sat),
            row.carrier_hz,
            row.code_phase_samples,
            row.peak,
            row.peak_mean_ratio,
            row.peak_second_ratio,
            row.hypothesis,
            format_optional_reason(row.selection_reason.as_deref()),
        );
    }
}

fn print_synthetic_iq_export_table(report: &SyntheticIqExportReport) {
    let sats = report.satellites.iter().map(|sat| format_sat(*sat)).collect::<Vec<_>>().join(", ");
    println!("Scenario: {}", report.scenario_id);
    println!("Seed: {}", report.seed);
    println!("Sample rate (Hz): {:.1}", report.sample_rate_hz);
    println!("Samples: {}", report.sample_count);
    println!("Satellites: {}", sats);
    println!("IQ: {}", report.output_iq);
    println!("Sidecar: {}", report.output_sidecar);
    println!("Truth: {}", report.output_truth);
}
fn print_inspect_table(report: &InspectReport) {
    println!(
        "Format\tSampleRate(Hz)\tIF(Hz)\tCaptureStartUtc\tSamples\tIMean\tQMean\tIPower\tQPower\tIqPowerRatio\tPowerWarning\tQuadratureErrorDeg\tQuadratureWarning\tClippingPct\tClippingWarning\tCenteredRms\tZeroSignalDetected\tZeroSignalReason\tPrecisionClaimsAllowed\tPrecisionRefusal\tRms\tDcImbalance\tNoiseFloor(dB)"
    );
    println!(
        "{}\t{:.1}\t{:.1}\t{}\t{}\t{:.6}\t{:.6}\t{:.6}\t{:.6}\t{:.6}\t{}\t{}\t{}\t{}\t{}\t{:.6e}\t{}\t{}\t{}\t{}\t{:.6}\t{:.6}\t{:.2}",
        report.format,
        report.sample_rate_hz,
        report.intermediate_freq_hz,
        report.capture_start_utc,
        report.total_samples,
        report.front_end_metrics.i_mean,
        report.front_end_metrics.q_mean,
        report.front_end_metrics.i_power,
        report.front_end_metrics.q_power,
        report.front_end_metrics.iq_power_ratio,
        report.front_end_metrics.power_imbalance_warning,
        format_optional_degrees(report.front_end_metrics.quadrature_error_deg),
        report.front_end_metrics.quadrature_error_warning,
        format_optional_percent(report.front_end_metrics.clipping_pct),
        report.front_end_metrics.clipping_warning,
        report.front_end_metrics.centered_rms,
        report.front_end_metrics.zero_signal_detected,
        format_optional_reason(report.front_end_metrics.zero_signal_reason.as_deref()),
        report.front_end_metrics.precision_claims_allowed,
        format_optional_reason(
            report
                .front_end_metrics
                .precision_claims_refused_reason
                .as_deref(),
        ),
        report.front_end_metrics.rms,
        report.front_end_metrics.dc_imbalance,
        report.noise_floor_db
    );
    println!("Power histogram bins: {:?}", report.power_histogram);
}

#[cfg(test)]
mod inspect_dataset_tests {
    use super::inspect_dataset;
    use crate::RawIqMetadata;
    use bijux_gnss_infra::api::signal::IqSampleFormat;
    use std::fs;
    use std::path::PathBuf;
    use std::time::{SystemTime, UNIX_EPOCH};

    fn temp_file_path(name: &str) -> PathBuf {
        let nanos = SystemTime::now().duration_since(UNIX_EPOCH).expect("unix epoch").as_nanos();
        std::env::temp_dir().join(format!("bijux_{}_{}_{}.iq", name, std::process::id(), nanos))
    }

    #[test]
    fn inspect_dataset_reports_signed_8bit_metadata() {
        let path = temp_file_path("inspect_iq8");
        fs::write(&path, [0x80u8, 0x7fu8, 0x40u8, 0xc0u8]).expect("write iq8 fixture");

        let metadata = RawIqMetadata {
            format: IqSampleFormat::Iq8,
            sample_rate_hz: 2_000_000.0,
            intermediate_freq_hz: 125_000.0,
            capture_start_utc: "2026-07-09T00:00:00Z".to_string(),
            offset_bytes: 0,
            quantization_bits: Some(8),
            notes: None,
        };
        let report = inspect_dataset(&path, &metadata, 0).expect("inspect dataset");

        assert_eq!(report.format, "Iq8");
        assert_eq!(report.sample_rate_hz, metadata.sample_rate_hz);
        assert_eq!(report.intermediate_freq_hz, metadata.intermediate_freq_hz);
        assert_eq!(report.capture_start_utc, metadata.capture_start_utc);
        assert_eq!(report.total_samples, 2);
        assert_eq!(report.front_end_metrics.sample_count, 2);
        assert_eq!(report.front_end_metrics.clipping_pct, Some(50.0));
        assert!(report.front_end_metrics.clipping_warning);
        assert!(!report.front_end_metrics.precision_claims_allowed);
        assert!(report
            .front_end_metrics
            .precision_claims_refused_reason
            .as_deref()
            .expect("precision refusal reason")
            .contains("front-end clipping 50.000% exceeds"));

        fs::remove_file(&path).expect("remove iq8 fixture");
    }

    #[test]
    fn inspect_dataset_reports_signed_16bit_metadata() {
        let path = temp_file_path("inspect_iq16");
        fs::write(&path, [0x00u8, 0x80u8, 0xffu8, 0x7fu8, 0x00u8, 0x40u8, 0x00u8, 0xc0u8])
            .expect("write iq16 fixture");

        let metadata = RawIqMetadata {
            format: IqSampleFormat::Iq16Le,
            sample_rate_hz: 2_000_000.0,
            intermediate_freq_hz: 125_000.0,
            capture_start_utc: "2026-07-09T00:00:00Z".to_string(),
            offset_bytes: 0,
            quantization_bits: Some(16),
            notes: None,
        };
        let report = inspect_dataset(&path, &metadata, 0).expect("inspect dataset");

        assert_eq!(report.format, "Iq16Le");
        assert_eq!(report.sample_rate_hz, metadata.sample_rate_hz);
        assert_eq!(report.intermediate_freq_hz, metadata.intermediate_freq_hz);
        assert_eq!(report.capture_start_utc, metadata.capture_start_utc);
        assert_eq!(report.total_samples, 2);
        assert_eq!(report.front_end_metrics.sample_count, 2);
        assert_eq!(report.front_end_metrics.clipping_pct, Some(50.0));
        assert!(report.front_end_metrics.clipping_warning);
        assert!(!report.front_end_metrics.precision_claims_allowed);
        assert!(report
            .front_end_metrics
            .precision_claims_refused_reason
            .as_deref()
            .expect("precision refusal reason")
            .contains("front-end clipping 50.000% exceeds"));

        fs::remove_file(&path).expect("remove iq16 fixture");
    }

    #[test]
    fn inspect_dataset_reports_complex_float32_metadata() {
        let path = temp_file_path("inspect_cf32");
        fs::write(
            &path,
            [
                0x00u8, 0x00u8, 0x80u8, 0xbfu8, 0x00u8, 0x00u8, 0x40u8, 0x3fu8, 0x00u8, 0x00u8,
                0x00u8, 0x3fu8, 0x00u8, 0x00u8, 0x80u8, 0xbeu8,
            ],
        )
        .expect("write cf32 fixture");

        let metadata = RawIqMetadata {
            format: IqSampleFormat::Cf32Le,
            sample_rate_hz: 2_000_000.0,
            intermediate_freq_hz: 125_000.0,
            capture_start_utc: "2026-07-09T00:00:00Z".to_string(),
            offset_bytes: 0,
            quantization_bits: Some(32),
            notes: None,
        };
        let report = inspect_dataset(&path, &metadata, 0).expect("inspect dataset");

        assert_eq!(report.format, "Cf32Le");
        assert_eq!(report.sample_rate_hz, metadata.sample_rate_hz);
        assert_eq!(report.intermediate_freq_hz, metadata.intermediate_freq_hz);
        assert_eq!(report.capture_start_utc, metadata.capture_start_utc);
        assert_eq!(report.total_samples, 2);
        assert_eq!(report.front_end_metrics.sample_count, 2);
        assert_eq!(report.front_end_metrics.clipping_pct, None);
        assert!(!report.front_end_metrics.clipping_warning);
        assert!(report.front_end_metrics.precision_claims_allowed);
        assert_eq!(report.front_end_metrics.precision_claims_refused_reason, None);
        assert!(
            report.front_end_metrics.centered_rms > 0.0,
            "centered_rms={}",
            report.front_end_metrics.centered_rms
        );
        assert!(!report.front_end_metrics.zero_signal_detected);
        assert_eq!(report.front_end_metrics.zero_signal_reason, None);

        fs::remove_file(&path).expect("remove cf32 fixture");
    }

    #[test]
    fn inspect_dataset_reports_iq_power_imbalance_warning() {
        let path = temp_file_path("inspect_iq8_power_warning");
        fs::write(&path, [32u8, 0u8, 32u8, 0u8]).expect("write iq8 fixture");

        let metadata = RawIqMetadata {
            format: IqSampleFormat::Iq8,
            sample_rate_hz: 2_000_000.0,
            intermediate_freq_hz: 0.0,
            capture_start_utc: "2026-07-09T00:00:00Z".to_string(),
            offset_bytes: 0,
            quantization_bits: Some(8),
            notes: None,
        };
        let report = inspect_dataset(&path, &metadata, 0).expect("inspect dataset");

        assert_eq!(report.front_end_metrics.sample_count, 2);
        assert_eq!(report.front_end_metrics.i_power, 0.0625);
        assert_eq!(report.front_end_metrics.q_power, 0.0);
        assert!(report.front_end_metrics.iq_power_ratio > 1.0e10);
        assert!(report.front_end_metrics.power_imbalance_warning);
        assert_eq!(report.front_end_metrics.clipping_pct, Some(0.0));
        assert!(!report.front_end_metrics.clipping_warning);
        assert_eq!(report.front_end_metrics.centered_rms, 0.0);
        assert!(report.front_end_metrics.zero_signal_detected);
        assert!(report
            .front_end_metrics
            .zero_signal_reason
            .as_deref()
            .expect("zero_signal_reason")
            .contains("no varying signal energy"));
        assert!(!report.front_end_metrics.precision_claims_allowed);
        assert!(report
            .front_end_metrics
            .precision_claims_refused_reason
            .as_deref()
            .expect("precision_claims_refused_reason")
            .contains("no varying signal energy"));

        fs::remove_file(&path).expect("remove iq8 fixture");
    }
}

#[cfg(test)]
mod nav_trace_tests {
    use super::*;
    use bijux_gnss_infra::api::core::{
        ArtifactPayloadValidate, Constellation, Epoch, LockFlags, Meters, NavLifecycleState,
        NavUncertaintyClass, ObservationEpochDecision, ObservationStatus, ObsEpochManifest,
        ObsMetadata, ObsSatellite, ReceiverRole, ReceiverSampleTrace, Seconds, SigId, SignalBand,
        SignalCode, SolutionStatus, SolutionValidity, signal_spec_gps_l1_ca,
    };

    fn sample_obs_epoch(with_manifest: bool) -> ObsEpoch {
        let source_time = ReceiverSampleTrace::from_sample_index(4_092_000, 4_092_000.0);
        let mut epoch = ObsEpoch {
            t_rx_s: Seconds(1.0),
            source_time,
            gps_week: Some(2000),
            tow_s: Some(Seconds(1.0)),
            epoch_idx: 7,
            discontinuity: false,
            valid: true,
            processing_ms: None,
            role: ReceiverRole::Rover,
            sats: vec![ObsSatellite {
                signal_id: SigId {
                    sat: SatId { constellation: Constellation::Gps, prn: 3 },
                    band: SignalBand::L1,
                    code: SignalCode::Ca,
                },
                pseudorange_m: Meters(20_200_000.0),
                pseudorange_var_m2: 25.0,
                carrier_phase_cycles: bijux_gnss_infra::api::core::Cycles(12_345.0),
                carrier_phase_var_cycles2: 1.0,
                doppler_hz: bijux_gnss_infra::api::core::Hertz(-1_250.0),
                doppler_var_hz2: 4.0,
                cn0_dbhz: 42.5,
                lock_flags: LockFlags {
                    code_lock: true,
                    carrier_lock: true,
                    bit_lock: true,
                    cycle_slip: false,
                },
                multipath_suspect: false,
                observation_status: ObservationStatus::Accepted,
                observation_reject_reasons: Vec::new(),
                elevation_deg: Some(45.0),
                azimuth_deg: Some(120.0),
                weight: None,
                error_model: None,
                metadata: ObsMetadata {
                    signal: signal_spec_gps_l1_ca(),
                    tracking_mode: "test".to_string(),
                    integration_ms: 20,
                    lock_quality: 1.0,
                    smoothing_window: 0,
                    smoothing_age: 0,
                    smoothing_resets: 0,
                    time_tag_source: "receiver_sample_index".to_string(),
                    time_tag_sample_index: source_time.sample_index,
                    time_tag_sample_rate_hz: source_time.sample_rate_hz,
                    ..ObsMetadata::default()
                },
            }],
            decision: ObservationEpochDecision::Accepted,
            decision_reason: Some("accepted_observables_present".to_string()),
            manifest: None,
        };
        if with_manifest {
            epoch.manifest = Some(ObsEpochManifest {
                version: bijux_gnss_infra::api::core::OBSERVATION_MODEL_VERSION,
                artifact_id: "obs-epoch-0000000007".to_string(),
                epoch_id: "epoch-0000000007-sample-000004092000".to_string(),
                source_epoch_idx: epoch.epoch_idx,
                source_sample_index: source_time.sample_index,
                source_time,
                decision: ObservationEpochDecision::Accepted,
                downstream_profile_version:
                    bijux_gnss_infra::api::core::OBSERVATION_DOWNSTREAM_PROFILE_VERSION,
            });
        }
        epoch
    }

    fn sample_nav_solution(obs: &ObsEpoch) -> NavSolutionEpoch {
        NavSolutionEpoch {
            epoch: Epoch { index: obs.epoch_idx },
            t_rx_s: obs.t_rx_s,
            source_time: obs.source_time,
            ecef_x_m: Meters(1.0),
            ecef_y_m: Meters(2.0),
            ecef_z_m: Meters(3.0),
            latitude_deg: 0.0,
            longitude_deg: 0.0,
            altitude_m: Meters(4.0),
            clock_bias_s: Seconds(0.0),
            clock_drift_s_per_s: 0.0,
            pdop: 1.5,
            rms_m: Meters(2.0),
            status: SolutionStatus::Float,
            quality: SolutionStatus::Float.quality_flag(),
            validity: SolutionValidity::Converging,
            valid: true,
            processing_ms: None,
            residuals: Vec::new(),
            health: Vec::new(),
            isb: Vec::new(),
            sigma_h_m: None,
            sigma_v_m: None,
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
            model_version: bijux_gnss_infra::api::core::NAV_SOLUTION_MODEL_VERSION,
            lifecycle_state: NavLifecycleState::Float,
            uncertainty_class: NavUncertaintyClass::Medium,
            assumptions: None,
            refusal_class: None,
            artifact_id: String::new(),
            source_observation_epoch_id: String::new(),
            explain_decision: "cli_navigation_solution".to_string(),
            explain_reasons: vec!["usable_satellites=4".to_string()],
            provenance: None,
            sat_count: 4,
            used_sat_count: 4,
            rejected_sat_count: 0,
            hdop: None,
            vdop: None,
            gdop: None,
            stability_signature: String::new(),
            stability_signature_version:
                bijux_gnss_infra::api::core::NAV_OUTPUT_STABILITY_SIGNATURE_VERSION,
        }
    }

    #[test]
    fn cli_nav_trace_identity_uses_observation_manifest() {
        let obs = sample_obs_epoch(true);
        let mut solution = sample_nav_solution(&obs);

        populate_cli_nav_solution_trace_identity(&obs, &mut solution);

        assert_eq!(
            solution.source_observation_epoch_id,
            "epoch-0000000007-sample-000004092000"
        );
        assert_eq!(solution.artifact_id, "nav-epoch-0000000007-epoch-0000000007");
        assert!(solution.stability_signature.starts_with("navsig:v1:"));
        assert!(solution.validate_payload().is_empty());
    }

    #[test]
    fn cli_nav_trace_identity_falls_back_to_observation_stability_key() {
        let obs = sample_obs_epoch(false);
        let mut solution = sample_nav_solution(&obs);

        populate_cli_nav_solution_trace_identity(&obs, &mut solution);

        assert_eq!(
            solution.source_observation_epoch_id,
            bijux_gnss_infra::api::core::obs_epoch_stability_key(&obs)
        );
        assert!(solution.artifact_id.starts_with("nav-epoch-0000000007-"));
        assert!(solution.stability_signature.contains("src="));
    }
}

fn solve_epoch_ekf(
    ctx: &mut Option<EkfContext>,
    obs: &ObsEpoch,
    ephs: &[GpsEphemeris],
) -> Result<Option<bijux_gnss_infra::api::core::NavSolutionEpoch>> {
    let Some(ctx) = ctx.as_mut() else {
        return Ok(None);
    };
    let dt_s =
        if let Some(prev) = ctx.last_t_rx_s { (obs.t_rx_s.0 - prev).max(1e-3) } else { 0.001 };
    ctx.last_t_rx_s = Some(obs.t_rx_s.0);
    ctx.ekf.predict(&ctx.model, dt_s);

    let mut used = 0;
    let mut sats: Vec<&bijux_gnss_infra::api::core::ObsSatellite> = obs.sats.iter().collect();
    sats.sort_by_key(|s| s.signal_id);
    let sat_count = sats.len();
    for sat in sats {
        let eph = match ephs.iter().find(|e| e.sat == sat.signal_id.sat) {
            Some(e) => e,
            None => continue,
        };
        let _corr = bijux_gnss_infra::api::nav::compute_corrections(&ctx.corrections);
        let state = sat_state_gps_l1ca(eph, obs.t_rx_s.0, 0.0);
        let state_next = sat_state_gps_l1ca(eph, obs.t_rx_s.0 + 0.1, 0.0);
        let sat_vel = [
            (state_next.x_m - state.x_m) / 0.1,
            (state_next.y_m - state.y_m) / 0.1,
            (state_next.z_m - state.z_m) / 0.1,
        ];
        let rx_x = ctx.ekf.x[0];
        let rx_y = ctx.ekf.x[1];
        let rx_z = ctx.ekf.x[2];
        let (_az, el) = elevation_azimuth_deg(rx_x, rx_y, rx_z, state.x_m, state.y_m, state.z_m);
        let weight = bijux_gnss_infra::api::nav::weight_from_cn0_elev(
            sat.cn0_dbhz,
            el,
            WeightingConfig::default(),
        );
        let sigma_m = (5.0 / weight.max(0.1)).max(1.0);
        let isb_index = if sat.signal_id.sat.constellation != Constellation::Gps {
            let key = format!("isb_{:?}", sat.signal_id.sat.constellation);
            Some(ctx.isb.get_or_add(&mut ctx.ekf, &key, 0.0, 1e-6))
        } else {
            None
        };
        let code_bias_m = ctx.code_bias.code_bias_m(sat.signal_id).unwrap_or(0.0);
        let meas = PseudorangeMeasurement {
            sig: sat.signal_id,
            z_m: sat.pseudorange_m.0 - code_bias_m,
            sat_pos_m: [state.x_m, state.y_m, state.z_m],
            sat_clock_s: state.clock_bias_s,
            tropo_m: 0.0,
            iono_m: 0.0,
            sigma_m,
            elevation_deg: Some(el),
            ztd_index: ctx.ztd_index,
            isb_index,
        };
        if ctx.ekf.update(&meas) {
            used += 1;
        }

        let doppler_meas = bijux_gnss_infra::api::nav::DopplerMeasurement {
            sig: sat.signal_id,
            z_hz: sat.doppler_hz.0,
            sat_pos_m: [state.x_m, state.y_m, state.z_m],
            sat_vel_mps: sat_vel,
            wavelength_m: 299_792_458.0 / sat.metadata.signal.carrier_hz.value(),
            sigma_hz: 2.0,
        };
        ctx.ekf.update(&doppler_meas);

        let amb_key = format!(
            "{:?}:{}:{:?}",
            sat.metadata.signal.constellation, sat.signal_id.sat.prn, sat.metadata.signal.band
        );
        let amb_idx = ctx.ambiguity.get_or_add(&mut ctx.ekf, &amb_key, 0.0, 100.0);
        let phase_bias_cycles = ctx.phase_bias.phase_bias_cycles(sat.signal_id).unwrap_or(0.0);
        let carrier_meas = bijux_gnss_infra::api::nav::CarrierPhaseMeasurement {
            sig: sat.signal_id,
            z_cycles: sat.carrier_phase_cycles.0 - phase_bias_cycles,
            sat_pos_m: [state.x_m, state.y_m, state.z_m],
            sat_clock_s: state.clock_bias_s,
            tropo_m: 0.0,
            iono_m: 0.0,
            wavelength_m: 299_792_458.0 / sat.metadata.signal.carrier_hz.value(),
            ambiguity_index: Some(amb_idx),
            sigma_cycles: 0.05,
            elevation_deg: Some(el),
            ztd_index: ctx.ztd_index,
            isb_index,
        };
        ctx.ekf.update(&carrier_meas);
    }

    if let Some(idx) = ctx.ztd_index {
        if idx < ctx.ekf.x.len() {
            let before = ctx.ekf.x[idx];
            let after = bijux_gnss_infra::api::nav::clamp_ztd(before, &ctx.atmosphere);
            if (after - before).abs() > 1e-6 {
                ctx.ekf.x[idx] = after;
                ctx.ekf.health.events.push(
                    bijux_gnss_infra::api::core::NavHealthEvent::ZtdClamped {
                        before_m: before,
                        after_m: after,
                    },
                );
            }
        }
    }

    let (lat, lon, alt) =
        bijux_gnss_infra::api::nav::ecef_to_geodetic(ctx.ekf.x[0], ctx.ekf.x[1], ctx.ekf.x[2]);
    let status = if used < 4 {
        bijux_gnss_infra::api::core::SolutionStatus::Degraded
    } else {
        bijux_gnss_infra::api::core::SolutionStatus::Float
    };
    let mut solution = bijux_gnss_infra::api::core::NavSolutionEpoch {
        epoch: bijux_gnss_infra::api::core::Epoch { index: obs.epoch_idx },
        t_rx_s: obs.t_rx_s,
        source_time: obs.source_time,
        ecef_x_m: bijux_gnss_infra::api::core::Meters(ctx.ekf.x[0]),
        ecef_y_m: bijux_gnss_infra::api::core::Meters(ctx.ekf.x[1]),
        ecef_z_m: bijux_gnss_infra::api::core::Meters(ctx.ekf.x[2]),
        latitude_deg: lat,
        longitude_deg: lon,
        altitude_m: bijux_gnss_infra::api::core::Meters(alt),
        clock_bias_s: bijux_gnss_infra::api::core::Seconds(ctx.ekf.x[6]),
        clock_drift_s_per_s: ctx.ekf.x.get(7).copied().unwrap_or(0.0),
        pdop: 0.0,
        rms_m: bijux_gnss_infra::api::core::Meters(ctx.ekf.health.innovation_rms),
        status,
        quality: status.quality_flag(),
        validity: if status == bijux_gnss_infra::api::core::SolutionStatus::Invalid {
            bijux_gnss_infra::api::core::SolutionValidity::Invalid
        } else {
            bijux_gnss_infra::api::core::SolutionValidity::Converging
        },
        valid: bijux_gnss_infra::api::core::is_solution_valid(status),
        processing_ms: None,
        residuals: Vec::new(),
        health: Vec::new(),
        isb: Vec::new(),
        sigma_h_m: None,
        sigma_v_m: None,
        innovation_rms_m: Some(ctx.ekf.health.innovation_rms),
        normalized_innovation_rms: None,
        normalized_innovation_max: None,
        ekf_innovation_rms: Some(ctx.ekf.health.innovation_rms),
        ekf_condition_number: ctx.ekf.health.condition_number,
        ekf_whiteness_ratio: ctx.ekf.health.whiteness_ratio,
        ekf_predicted_variance: ctx.ekf.health.predicted_variance,
        ekf_observed_variance: ctx.ekf.health.observed_variance,
        integrity_hpl_m: None,
        integrity_vpl_m: None,
        model_version: bijux_gnss_infra::api::core::NAV_SOLUTION_MODEL_VERSION,
        lifecycle_state: match status {
            bijux_gnss_infra::api::core::SolutionStatus::Invalid => {
                bijux_gnss_infra::api::core::NavLifecycleState::Invalid
            }
            bijux_gnss_infra::api::core::SolutionStatus::Held => {
                bijux_gnss_infra::api::core::NavLifecycleState::Held
            }
            bijux_gnss_infra::api::core::SolutionStatus::Degraded => {
                bijux_gnss_infra::api::core::NavLifecycleState::Degraded
            }
            bijux_gnss_infra::api::core::SolutionStatus::Coarse => {
                bijux_gnss_infra::api::core::NavLifecycleState::Coarse
            }
            bijux_gnss_infra::api::core::SolutionStatus::Converged => {
                bijux_gnss_infra::api::core::NavLifecycleState::Converged
            }
            bijux_gnss_infra::api::core::SolutionStatus::Float => {
                bijux_gnss_infra::api::core::NavLifecycleState::Float
            }
            bijux_gnss_infra::api::core::SolutionStatus::Fixed => {
                bijux_gnss_infra::api::core::NavLifecycleState::Fixed
            }
        },
        uncertainty_class: if status == bijux_gnss_infra::api::core::SolutionStatus::Invalid {
            bijux_gnss_infra::api::core::NavUncertaintyClass::Unknown
        } else if status == bijux_gnss_infra::api::core::SolutionStatus::Degraded {
            bijux_gnss_infra::api::core::NavUncertaintyClass::High
        } else {
            bijux_gnss_infra::api::core::NavUncertaintyClass::Medium
        },
        assumptions: None,
        refusal_class: None,
        artifact_id: String::new(),
        source_observation_epoch_id: String::new(),
        explain_decision: "cli_navigation_solution".to_string(),
        explain_reasons: vec![format!("usable_satellites={used}")],
        provenance: None,
        sat_count,
        used_sat_count: used,
        rejected_sat_count: sat_count.saturating_sub(used),
        hdop: None,
        vdop: None,
        gdop: None,
        stability_signature: String::new(),
        stability_signature_version:
            bijux_gnss_infra::api::core::NAV_OUTPUT_STABILITY_SIGNATURE_VERSION,
    };
    populate_cli_nav_solution_trace_identity(obs, &mut solution);
    Ok(Some(solution))
}

fn populate_cli_nav_solution_trace_identity(
    obs: &ObsEpoch,
    solution: &mut bijux_gnss_infra::api::core::NavSolutionEpoch,
) {
    let source_observation_epoch_id = obs
        .manifest
        .as_ref()
        .map(|manifest| manifest.epoch_id.clone())
        .unwrap_or_else(|| bijux_gnss_infra::api::core::obs_epoch_stability_key(obs));
    solution.source_observation_epoch_id = source_observation_epoch_id.clone();
    solution.artifact_id =
        format!("nav-epoch-{:010}-{}", solution.epoch.index, cli_nav_trace_short_id(&source_observation_epoch_id));
    solution.stability_signature = cli_nav_output_stability_signature(solution);
}

fn cli_nav_output_stability_signature(
    solution: &bijux_gnss_infra::api::core::NavSolutionEpoch,
) -> String {
    format!(
        "navsig:v{}:epoch={}:src={}@{}:status={:?}:lifecycle={:?}:valid={}:sat={}:used={}:rej={}:pdop={:.3}:rms={:.3}:decision={}",
        bijux_gnss_infra::api::core::NAV_OUTPUT_STABILITY_SIGNATURE_VERSION,
        solution.epoch.index,
        cli_nav_trace_short_id(&solution.source_observation_epoch_id),
        solution.source_time.sample_index,
        solution.status,
        solution.lifecycle_state,
        solution.valid,
        solution.sat_count,
        solution.used_sat_count,
        solution.rejected_sat_count,
        solution.pdop,
        solution.rms_m.0,
        solution.explain_decision
    )
}

fn cli_nav_trace_short_id(value: &str) -> String {
    value.chars().take(16).collect()
}

#[cfg(feature = "tracing")]
fn init_tracing() {
    let _ = tracing_subscriber::fmt().with_env_filter("info").with_target(false).try_init();
}

#[cfg(not(feature = "tracing"))]
fn init_tracing() {}

fn main() -> Result<()> {
    init_tracing();
    let cli = Cli::parse();
    match cli.command {
        AppCommand::Gnss { command } => run_command(command),
    }
}
