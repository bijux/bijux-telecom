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
        cmd @ GnssCommand::ValidateSyntheticIq { .. } => handle_validate_synthetic_iq(cmd),
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

use bijux_gnss_nav::{position_dops_from_satellite_positions, PositionDops};

fn inspect_dataset(
    path: &Path,
    metadata: &RawIqMetadata,
    max_samples: usize,
) -> Result<InspectReport> {
    let signal_quality = measure_signal_quality_from_raw_iq(path, metadata, max_samples)?;
    let mut source = FileSamples::open_raw_iq(path, metadata.clone())
        .with_context(|| format!("failed to open {}", path.display()))?;
    let mut total_iq = 0usize;

    let mut power_hist = vec![0u64; 8];

    while max_samples == 0 || total_iq < max_samples {
        let frame_len = if max_samples == 0 { 4096 } else { (max_samples - total_iq).min(4096) };
        let Some(frame) = source.next_frame(frame_len)? else {
            break;
        };
        for sample in &frame.iq {
            let i = sample.re as f64;
            let q = sample.im as f64;
            let power = i * i + q * q;
            let bin = ((power.sqrt() / 0.25).min(7.0)) as usize;
            power_hist[bin] += 1;
            total_iq += 1;
            if max_samples != 0 && total_iq >= max_samples {
                break;
            }
        }
    }

    Ok(InspectReport {
        format: format!("{:?}", metadata.format),
        sample_rate_hz: metadata.sample_rate_hz,
        intermediate_freq_hz: metadata.intermediate_freq_hz,
        capture_start_utc: metadata.capture_start_utc.clone(),
        total_samples: total_iq,
        usable_duration_s: signal_quality.usable_duration_s,
        front_end_metrics: signal_quality.front_end_metrics.clone(),
        noise_floor_db: signal_quality.estimated_noise_floor_db,
        power_histogram: power_hist,
        signal_quality,
    })
}
fn runtime_config_from_env(
    common: &CommonArgs,
    run_dir: Option<PathBuf>,
) -> bijux_gnss_infra::api::receiver::ReceiverRuntime {
    runtime_config_from_capture_start(common, run_dir, None)
}

fn runtime_config_from_capture_start(
    common: &CommonArgs,
    run_dir: Option<PathBuf>,
    capture_start_utc: Option<&str>,
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
        capture_start_gps_time: capture_start_utc.and_then(capture_start_gps_time),
    };
    bijux_gnss_infra::api::receiver::ReceiverRuntime::new(config)
}

fn capture_start_gps_time(capture_start_utc: &str) -> Option<bijux_gnss_infra::api::core::GpsTime> {
    let utc = time::OffsetDateTime::parse(
        capture_start_utc,
        &time::format_description::well_known::Rfc3339,
    )
    .ok()?;
    Some(bijux_gnss_infra::api::nav::gps_time_from_utc(bijux_gnss_infra::api::core::UtcTime {
        unix_s: utc.unix_timestamp_nanos() as f64 / 1_000_000_000.0,
    }))
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

fn format_search_summary(report: &AcquisitionReport) -> String {
    format!(
        "searched {} PRNs; accepted {}; ambiguous {}; rejected {}; deferred {}",
        report.search_summary.searched_satellites,
        report.search_summary.accepted,
        report.search_summary.ambiguous,
        report.search_summary.rejected,
        report.search_summary.deferred,
    )
}

fn format_doppler_search(settings: DopplerSearchSettings) -> String {
    format!(
        "+/-{} Hz in {} Hz bins ({} bins, IF {:.1} Hz)",
        settings.max_search_hz,
        settings.bin_width_hz,
        settings.bin_count,
        settings.intermediate_freq_hz
    )
}

fn format_code_phase_search(settings: &CodePhaseSearchSettings) -> String {
    format!(
        "{} samples from {} in {}-sample steps ({} bins, mode {})",
        settings.period_samples,
        settings.start_sample,
        settings.step_samples,
        settings.bin_count,
        settings.mode
    )
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
    println!("Doppler search: {}", format_doppler_search(report.doppler_search));
    println!("Code-phase search: {}", format_code_phase_search(&report.code_phase_search));
    println!("Search summary: {}", format_search_summary(report));
    println!("Reported PRNs: {}", format_reported_prns(report));
    println!(
        "Sat\tBand\tStartSample\tDoppler(Hz)\tRank\tPrimary\tCarrier(Hz)\tCoarseCarrier(Hz)\tRefine(Hz)\tRefine(Bins)\tDopplerUnc(Hz)\tCodePhase\tRefinedCodePhase\tCodePhaseRefine\tCodePhaseUnc\tPeak\tPeak/Mean\tPeak/2nd\tHypothesis\tReason"
    );
    for row in &report.results {
        let coarse_carrier_hz = row
            .coarse_carrier_hz
            .map(|value| format!("{value:.1}"))
            .unwrap_or_else(|| "n/a".to_string());
        let doppler_refinement_hz = row
            .doppler_refinement_hz
            .map(|value| format!("{value:.3}"))
            .unwrap_or_else(|| "n/a".to_string());
        let doppler_refinement_bins = row
            .doppler_refinement_bins
            .map(|value| format!("{value:.6}"))
            .unwrap_or_else(|| "n/a".to_string());
        let doppler_uncertainty_hz = row
            .doppler_uncertainty_hz
            .map(|value| format!("{value:.3}"))
            .unwrap_or_else(|| "n/a".to_string());
        let refined_code_phase_samples = row
            .refined_code_phase_samples
            .map(|value| format!("{value:.6}"))
            .unwrap_or_else(|| "n/a".to_string());
        let code_phase_refinement_samples = row
            .code_phase_refinement_samples
            .map(|value| format!("{value:.6}"))
            .unwrap_or_else(|| "n/a".to_string());
        let code_phase_uncertainty_samples = row
            .code_phase_uncertainty_samples
            .map(|value| format!("{value:.6}"))
            .unwrap_or_else(|| "n/a".to_string());
        println!(
            "{}\t{:?}\t{}\t{:.3}\t{}\t{}\t{:.1}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{:.3}\t{:.2}\t{:.2}\t{}\t{}",
            format_sat(row.sat),
            row.signal_band,
            row.source_sample_index,
            row.doppler_hz,
            row.candidate_rank,
            row.is_primary_candidate,
            row.carrier_hz,
            coarse_carrier_hz,
            doppler_refinement_hz,
            doppler_refinement_bins,
            doppler_uncertainty_hz,
            row.code_phase_samples,
            refined_code_phase_samples,
            code_phase_refinement_samples,
            code_phase_uncertainty_samples,
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

fn print_synthetic_iq_validation_table(report: &SyntheticIqValidationReport) {
    println!("Scenario: {}", report.validation.scenario_id);
    println!("Input IQ: {}", report.input_iq);
    println!("Input sidecar: {}", report.input_sidecar);
    println!("Input truth: {}", report.input_truth);
    println!("Tolerance (dB-Hz): {:.3}", report.validation.tolerance_db_hz);
    println!(
        "Coherent integration: {} samples ({:.6} s)",
        report.validation.coherent_samples_per_epoch, report.validation.coherent_integration_s
    );
    println!("Pass: {}", report.validation.pass);
    for row in &report.validation.satellites {
        println!(
            "{}\tinjected={:.3}\tmeasured={:.3}\tdelta={:.3}\tepochs={}\tpass={}",
            format_sat(row.sat),
            row.injected_cn0_db_hz,
            row.measured_mean_cn0_dbhz,
            row.cn0_delta_db,
            row.epochs_measured,
            row.pass
        );
    }
    println!(
        "Acquisition code-phase tolerance (samples): {}",
        report.acquisition_code_phase_validation.tolerance_samples
    );
    println!("Acquisition code-phase pass: {}", report.acquisition_code_phase_validation.pass);
    for row in &report.acquisition_code_phase_validation.satellites {
        println!(
            "{}\texpected={}\tmeasured={}\terror={}\tpeak/mean={:.3}\thypothesis={}\tpass={}",
            format_sat(row.sat),
            row.expected_code_phase_samples,
            row.measured_code_phase_samples,
            row.code_phase_error_samples,
            row.peak_mean_ratio,
            row.hypothesis,
            row.pass
        );
    }
    println!(
        "Acquisition code-phase refinement pass: {}",
        report.acquisition_code_phase_refinement_validation.pass
    );
    for row in &report.acquisition_code_phase_refinement_validation.satellites {
        println!(
            "{}\texpected={:.6}\tcoarse={}\trefined={:.6}\tcoarse_err={:.6}\trefined_err={:.6}\timprovement_m={:.6}\thypothesis={}\tpass={}",
            format_sat(row.sat),
            row.expected_code_phase_samples,
            row.coarse_code_phase_samples,
            row.refined_code_phase_samples,
            row.coarse_error_samples,
            row.refined_error_samples,
            row.improvement_m,
            row.hypothesis,
            row.pass
        );
    }
    println!(
        "Acquisition Doppler tolerance: {} bins ({:.3} Hz)",
        report.acquisition_doppler_validation.tolerance_bins,
        report.acquisition_doppler_validation.tolerance_hz
    );
    println!("Acquisition Doppler pass: {}", report.acquisition_doppler_validation.pass);
    for row in &report.acquisition_doppler_validation.satellites {
        println!(
            "{}\tinjected={:.3}\texpected={:.3}\tmeasured={:.3}\terror={:.3}\terror_bins={:.3}\tpeak/mean={:.3}\thypothesis={}\tpass={}",
            format_sat(row.sat),
            row.injected_doppler_hz,
            row.expected_measured_doppler_hz,
            row.measured_doppler_hz,
            row.doppler_error_hz,
            row.doppler_error_bins,
            row.peak_mean_ratio,
            row.hypothesis,
            row.pass
        );
    }
    println!(
        "Acquisition receiver clock-offset tolerance: {} bins ({:.3} Hz)",
        report.acquisition_receiver_clock_offset_validation.tolerance_bins,
        report.acquisition_receiver_clock_offset_validation.tolerance_hz
    );
    println!(
        "Acquisition receiver clock-offset pass: {}",
        report.acquisition_receiver_clock_offset_validation.pass
    );
    for row in &report.acquisition_receiver_clock_offset_validation.satellites {
        println!(
            "{}\tinjected_bias={:.3}\texpected_doppler={:.3}\tmeasured_doppler={:.3}\tmeasured_bias={:.3}\tbias_error={:.3}\tpeak/mean={:.3}\thypothesis={}\tpass={}",
            format_sat(row.sat),
            row.injected_receiver_clock_frequency_bias_hz,
            row.expected_measured_doppler_hz,
            row.measured_doppler_hz,
            row.measured_receiver_clock_frequency_bias_hz,
            row.receiver_clock_frequency_bias_error_hz,
            row.peak_mean_ratio,
            row.hypothesis,
            row.pass
        );
    }
}
fn print_inspect_table(report: &InspectReport) {
    println!(
        "Format\tSampleRate(Hz)\tIF(Hz)\tCaptureStartUtc\tSamples\tUsableDuration(s)\tIMean\tQMean\tIPower\tQPower\tIqPowerRatio\tPowerWarning\tQuadratureErrorDeg\tQuadratureWarning\tClippingPct\tClippingWarning\tCenteredRms\tZeroSignalDetected\tZeroSignalReason\tPrecisionClaimsAllowed\tPrecisionRefusal\tRms\tDcImbalance\tNoiseFloor(dB)"
    );
    println!(
        "{}\t{:.1}\t{:.1}\t{}\t{}\t{:.6}\t{:.6}\t{:.6}\t{:.6}\t{:.6}\t{:.6}\t{}\t{}\t{}\t{}\t{}\t{:.6e}\t{}\t{}\t{}\t{}\t{:.6}\t{:.6}\t{:.2}",
        report.format,
        report.sample_rate_hz,
        report.intermediate_freq_hz,
        report.capture_start_utc,
        report.total_samples,
        report.usable_duration_s,
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
mod runtime_config_tests {
    use super::{
        capture_start_gps_time, runtime_config_from_capture_start, CommonArgs, ReportFormat,
    };

    fn common_args() -> CommonArgs {
        CommonArgs {
            config: None,
            dataset: None,
            unregistered_dataset: true,
            out: None,
            report: ReportFormat::Table,
            seed: None,
            deterministic: false,
            dump: None,
            sidecar: None,
            resume: None,
        }
    }

    #[test]
    fn capture_start_gps_time_parses_rfc3339_timestamp() {
        let gps_time = capture_start_gps_time("2022-03-27T11:32:04.2147593125Z").expect("gps time");
        assert_eq!(gps_time.week, 2203);
        assert!((gps_time.tow_s - 41_542.2147593125).abs() <= 1.0e-6);
    }

    #[test]
    fn runtime_config_from_capture_start_sets_runtime_anchor() {
        let runtime =
            runtime_config_from_capture_start(&common_args(), None, Some("2026-07-09T00:00:00Z"));

        let gps_time = runtime.config.capture_start_gps_time.expect("runtime gps anchor");
        assert_eq!(gps_time.week, 2426);
        assert!((gps_time.tow_s - 345_618.0).abs() <= 1.0e-9);
    }
}

#[cfg(test)]
mod nav_trace_tests {
    use super::*;
    use bijux_gnss_infra::api::core::{
        signal_spec_gps_l1_ca, ArtifactPayloadValidate, Constellation, Epoch, LockFlags, Meters,
        NavLifecycleState, NavUncertaintyClass, ObsEpochManifest, ObsMetadata, ObsSatellite,
        ObservationEpochDecision, ObservationStatus, ReceiverRole, ReceiverSampleTrace, Seconds,
        SigId, SignalBand, SignalCode, SolutionStatus, SolutionValidity,
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
                timing: None,
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
            clock_bias_m: Meters(0.0),
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
            tdop: None,
            stability_signature: String::new(),
            stability_signature_version:
                bijux_gnss_infra::api::core::NAV_OUTPUT_STABILITY_SIGNATURE_VERSION,
        }
    }

    fn sample_eph(prn: u8, toe_s: f64, toc_s: f64) -> GpsEphemeris {
        GpsEphemeris {
            sat: SatId { constellation: Constellation::Gps, prn },
            iodc: 0,
            iode: 0,
            week: 2000,
            sv_health: 0,
            toe_s,
            toc_s,
            sqrt_a: 5153.7954775,
            e: 0.01,
            i0: 0.94,
            idot: 0.0,
            omega0: 1.9,
            omegadot: 0.0,
            w: 0.2,
            m0: 0.6,
            delta_n: 0.0,
            cuc: 0.0,
            cus: 0.0,
            crc: 0.0,
            crs: 0.0,
            cic: 0.0,
            cis: 0.0,
            af0: 0.0,
            af1: 0.0,
            af2: 0.0,
            tgd: 0.0,
        }
    }

    fn sample_geometry_eph(prn: u8, omega0: f64, w: f64, m0: f64) -> GpsEphemeris {
        let mut eph = sample_eph(prn, 1.0, 1.0);
        eph.omega0 = omega0;
        eph.w = w;
        eph.m0 = m0;
        eph
    }

    fn sample_consistent_geometry_epoch(
        receiver_ecef_m: [f64; 3],
        ephs: &[GpsEphemeris],
    ) -> ObsEpoch {
        let mut obs = sample_obs_epoch(false);
        let receive_tow_s = obs.t_rx_s.0;
        let carrier_hz = signal_spec_gps_l1_ca().carrier_hz.value();
        let wavelength_m = 299_792_458.0 / carrier_hz;
        obs.sats = ephs
            .iter()
            .map(|eph| {
                let state = bijux_gnss_infra::api::nav::sat_state_gps_l1ca(eph, receive_tow_s, 0.0);
                let state_next =
                    bijux_gnss_infra::api::nav::sat_state_gps_l1ca(eph, receive_tow_s + 0.1, 0.0);
                let sat_pos_m = [state.x_m, state.y_m, state.z_m];
                let sat_vel_mps = [
                    (state_next.x_m - state.x_m) / 0.1,
                    (state_next.y_m - state.y_m) / 0.1,
                    (state_next.z_m - state.z_m) / 0.1,
                ];
                let delta = [
                    sat_pos_m[0] - receiver_ecef_m[0],
                    sat_pos_m[1] - receiver_ecef_m[1],
                    sat_pos_m[2] - receiver_ecef_m[2],
                ];
                let range_m = (delta[0].powi(2) + delta[1].powi(2) + delta[2].powi(2)).sqrt();
                let los = [delta[0] / range_m, delta[1] / range_m, delta[2] / range_m];
                let range_rate_mps =
                    los[0] * sat_vel_mps[0] + los[1] * sat_vel_mps[1] + los[2] * sat_vel_mps[2];
                let (azimuth_deg, elevation_deg) = elevation_azimuth_deg(
                    receiver_ecef_m[0],
                    receiver_ecef_m[1],
                    receiver_ecef_m[2],
                    sat_pos_m[0],
                    sat_pos_m[1],
                    sat_pos_m[2],
                );
                ObsSatellite {
                    signal_id: SigId { sat: eph.sat, band: SignalBand::L1, code: SignalCode::Ca },
                    pseudorange_m: Meters(range_m),
                    pseudorange_var_m2: 1.0,
                    carrier_phase_cycles: bijux_gnss_infra::api::core::Cycles(
                        range_m / wavelength_m,
                    ),
                    carrier_phase_var_cycles2: 0.05_f64.powi(2),
                    doppler_hz: bijux_gnss_infra::api::core::Hertz(-range_rate_mps / wavelength_m),
                    doppler_var_hz2: 1.0,
                    cn0_dbhz: 45.0,
                    lock_flags: LockFlags {
                        code_lock: true,
                        carrier_lock: true,
                        bit_lock: true,
                        cycle_slip: false,
                    },
                    multipath_suspect: false,
                    observation_status: ObservationStatus::Accepted,
                    observation_reject_reasons: Vec::new(),
                    elevation_deg: Some(elevation_deg),
                    azimuth_deg: Some(azimuth_deg),
                    weight: None,
                    timing: None,
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
                        time_tag_sample_index: obs.source_time.sample_index,
                        time_tag_sample_rate_hz: obs.source_time.sample_rate_hz,
                        ..ObsMetadata::default()
                    },
                }
            })
            .collect();
        obs
    }

    #[test]
    fn cli_nav_trace_identity_uses_observation_manifest() {
        let obs = sample_obs_epoch(true);
        let mut solution = sample_nav_solution(&obs);

        populate_cli_nav_solution_trace_identity(&obs, &mut solution);

        assert_eq!(solution.source_observation_epoch_id, "epoch-0000000007-sample-000004092000");
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

    #[test]
    fn cli_nav_satellite_state_uses_signal_timing_when_available() {
        let receive_gps_time =
            bijux_gnss_infra::api::core::GpsTime { week: 2200, tow_s: 345_600.08 };
        let tau_s = 0.074;
        let mut eph = sample_eph(8, 345_600.0, 345_600.0);
        eph.week = receive_gps_time.week;
        let sat = ObsSatellite {
            signal_id: SigId { sat: eph.sat, band: SignalBand::L1, code: SignalCode::Ca },
            pseudorange_m: Meters(tau_s * 299_792_458.0),
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
            timing: Some(bijux_gnss_infra::api::core::ObsSignalTiming {
                signal_travel_time_s: Seconds(tau_s),
                transmit_gps_time: receive_gps_time.offset_seconds(-tau_s),
            }),
            error_model: None,
            metadata: ObsMetadata { signal: signal_spec_gps_l1_ca(), ..ObsMetadata::default() },
        };

        let from_helper = cli_nav_satellite_state(&sat, &eph, receive_gps_time.tow_s);
        let corrected = bijux_gnss_infra::api::nav::sat_state_gps_l1ca_at_receive_time(
            &eph,
            receive_gps_time.tow_s,
            tau_s,
        );
        let uncorrected =
            bijux_gnss_infra::api::nav::sat_state_gps_l1ca(&eph, receive_gps_time.tow_s, 0.0);

        assert!((from_helper.x_m - corrected.x_m).abs() < 1.0e-9);
        assert!((from_helper.y_m - corrected.y_m).abs() < 1.0e-9);
        assert!((from_helper.z_m - corrected.z_m).abs() < 1.0e-9);
        assert!((from_helper.x_m - uncorrected.x_m).abs() > 0.01);
        assert!((from_helper.y_m - uncorrected.y_m).abs() > 0.01);
    }

    #[test]
    fn cli_ekf_rejects_stale_ephemeris() {
        let mut obs = sample_obs_epoch(false);
        obs.t_rx_s = Seconds(7_201.0);
        obs.tow_s = Some(Seconds(7_201.0));

        let eph = sample_eph(3, 0.0, 0.0);
        let mut ctx = Some(EkfContext::new());

        let solution = solve_epoch_ekf(&mut ctx, &obs, &[eph], None)
            .expect("cli ekf solve")
            .expect("solution");

        assert_eq!(solution.status, SolutionStatus::Invalid);
        assert_eq!(
            solution.refusal_class,
            Some(bijux_gnss_infra::api::core::NavRefusalClass::InvalidEphemeris)
        );
        assert_eq!(solution.used_sat_count, 0);
        assert_eq!(solution.rejected_sat_count, 1);
        assert_eq!(solution.ecef_x_m.0, 0.0);
        assert!(solution
            .explain_reasons
            .iter()
            .any(|reason| reason == "stale_ephemeris_rejections=1"));
    }

    #[test]
    fn cli_ekf_refuses_sparse_epoch_without_serializing_filter_state() {
        let obs = sample_obs_epoch(false);
        let eph = sample_eph(3, 1.0, 1.0);
        let mut ctx = Some(EkfContext::new());
        let ekf = &mut ctx.as_mut().expect("ekf context").ekf;
        ekf.x[0] = 123.0;
        ekf.x[1] = 456.0;
        ekf.x[2] = 789.0;

        let solution = solve_epoch_ekf(&mut ctx, &obs, &[eph], None)
            .expect("cli ekf solve")
            .expect("solution");

        assert_eq!(solution.status, SolutionStatus::Invalid);
        assert_eq!(
            solution.refusal_class,
            Some(bijux_gnss_infra::api::core::NavRefusalClass::InsufficientGeometry)
        );
        assert_eq!(solution.used_sat_count, 1);
        assert_eq!(solution.sat_count, 1);
        assert_eq!(solution.ecef_x_m.0, 0.0);
        assert_eq!(solution.ecef_y_m.0, 0.0);
        assert_eq!(solution.ecef_z_m.0, 0.0);
    }

    #[test]
    fn cli_ekf_rejects_poor_geometry_with_dop_evidence() {
        let receiver_ecef_m = {
            let ecef = bijux_gnss_infra::api::nav::geodetic_to_ecef(37.3349, -122.0090, 12.0);
            [ecef.0, ecef.1, ecef.2]
        };
        let ephs = vec![
            sample_geometry_eph(3, 1.90, 0.20, 0.60),
            sample_geometry_eph(7, 1.97, 0.23, 0.74),
            sample_geometry_eph(11, 2.08, 0.16, 0.91),
            sample_geometry_eph(19, 2.21, 0.27, 1.08),
        ];
        let obs = sample_consistent_geometry_epoch(receiver_ecef_m, &ephs);

        let mut refusal_ctx =
            Some(EkfContext::new_with_troposphere(false, 0.0, EkfScienceThresholds::default()));
        let refusal_ekf = &mut refusal_ctx.as_mut().expect("refusal ekf context").ekf;
        refusal_ekf.x[0] = receiver_ecef_m[0];
        refusal_ekf.x[1] = receiver_ecef_m[1];
        refusal_ekf.x[2] = receiver_ecef_m[2];
        let refusal_solution = solve_epoch_ekf(&mut refusal_ctx, &obs, &ephs, None)
            .expect("refusal cli ekf solve")
            .expect("refusal solution");

        assert_eq!(refusal_solution.status, SolutionStatus::Invalid);
        assert_eq!(
            refusal_solution.refusal_class,
            Some(bijux_gnss_infra::api::core::NavRefusalClass::InsufficientGeometry)
        );
        let refusal_gdop = refusal_solution.gdop.expect("refusal gdop");
        assert!(refusal_solution.pdop > 8.0);
        assert!(refusal_gdop > 12.0);
        assert_eq!(refusal_solution.used_sat_count, 4);
        assert_eq!(refusal_solution.ecef_x_m.0, 0.0);
        assert_eq!(refusal_solution.ecef_y_m.0, 0.0);
        assert_eq!(refusal_solution.ecef_z_m.0, 0.0);
        assert!(refusal_solution
            .explain_reasons
            .iter()
            .any(|reason| reason.starts_with("pdop_above_threshold:")));
        assert!(refusal_solution
            .explain_reasons
            .iter()
            .any(|reason| reason.starts_with("gdop_above_threshold:")));

        let mut permissive_ctx = Some(EkfContext::new_with_troposphere(
            false,
            0.0,
            EkfScienceThresholds { max_pdop: 100.0, max_gdop: 110.0, min_used_satellites: 4 },
        ));
        let permissive_ekf = &mut permissive_ctx.as_mut().expect("permissive ekf context").ekf;
        permissive_ekf.x[0] = receiver_ecef_m[0];
        permissive_ekf.x[1] = receiver_ecef_m[1];
        permissive_ekf.x[2] = receiver_ecef_m[2];
        let permissive_solution = solve_epoch_ekf(&mut permissive_ctx, &obs, &ephs, None)
            .expect("permissive cli ekf solve")
            .expect("permissive solution");

        assert_eq!(permissive_solution.status, SolutionStatus::Float);
        assert!((permissive_solution.pdop - refusal_solution.pdop).abs() < 1e-9);
        assert_eq!(permissive_solution.gdop, refusal_solution.gdop);
    }

    #[test]
    fn cli_ekf_saastamoinen_delay_is_zero_when_disabled() {
        let receiver_ecef_m = bijux_gnss_infra::api::nav::geodetic_to_ecef(37.0, -122.0, 10.0);
        let satellite_ecef_m =
            bijux_gnss_infra::api::nav::geodetic_to_ecef(37.0, -122.0, 20_200_000.0);
        let state = bijux_gnss_infra::api::nav::GpsSatState {
            x_m: satellite_ecef_m.0,
            y_m: satellite_ecef_m.1,
            z_m: satellite_ecef_m.2,
            clock_correction: bijux_gnss_infra::api::nav::GpsSatelliteClockCorrection::from_bias_s(
                0.0,
            ),
        };

        let delay_m = estimate_ekf_saastamoinen_delay_m(
            false,
            [receiver_ecef_m.0, receiver_ecef_m.1, receiver_ecef_m.2],
            &state,
        );

        assert_eq!(delay_m, 0.0);
    }

    #[test]
    fn cli_ekf_saastamoinen_delay_rejects_implausible_receiver_radius() {
        let satellite_ecef_m =
            bijux_gnss_infra::api::nav::geodetic_to_ecef(37.0, -122.0, 20_200_000.0);
        let state = bijux_gnss_infra::api::nav::GpsSatState {
            x_m: satellite_ecef_m.0,
            y_m: satellite_ecef_m.1,
            z_m: satellite_ecef_m.2,
            clock_correction: bijux_gnss_infra::api::nav::GpsSatelliteClockCorrection::from_bias_s(
                0.0,
            ),
        };

        let delay_m = estimate_ekf_saastamoinen_delay_m(true, [0.0, 0.0, 0.0], &state);

        assert_eq!(delay_m, 0.0);
    }

    #[test]
    fn cli_ekf_saastamoinen_delay_is_positive_for_visible_satellite() {
        let receiver_ecef_m = bijux_gnss_infra::api::nav::geodetic_to_ecef(37.0, -122.0, 10.0);
        let satellite_ecef_m =
            bijux_gnss_infra::api::nav::geodetic_to_ecef(37.0, -122.0, 20_200_000.0);
        let state = bijux_gnss_infra::api::nav::GpsSatState {
            x_m: satellite_ecef_m.0,
            y_m: satellite_ecef_m.1,
            z_m: satellite_ecef_m.2,
            clock_correction: bijux_gnss_infra::api::nav::GpsSatelliteClockCorrection::from_bias_s(
                0.0,
            ),
        };

        let delay_m = estimate_ekf_saastamoinen_delay_m(
            true,
            [receiver_ecef_m.0, receiver_ecef_m.1, receiver_ecef_m.2],
            &state,
        );

        assert!(delay_m.is_finite());
        assert!(delay_m > 2.0);
    }
}

fn solve_epoch_ekf(
    ctx: &mut Option<EkfContext>,
    obs: &ObsEpoch,
    ephs: &[GpsEphemeris],
    klobuchar: Option<&bijux_gnss_infra::api::nav::KlobucharCoefficients>,
) -> Result<Option<bijux_gnss_infra::api::core::NavSolutionEpoch>> {
    let Some(ctx) = ctx.as_mut() else {
        return Ok(None);
    };
    if (klobuchar.is_some() || ctx.tropo_enabled) && ekf_position_is_uninitialized(ctx) {
        prime_ekf_state_from_wls(ctx, obs, ephs, klobuchar, ctx.tropo_enabled);
    }
    let dt_s =
        if let Some(prev) = ctx.last_t_rx_s { (obs.t_rx_s.0 - prev).max(1e-3) } else { 0.001 };
    ctx.last_t_rx_s = Some(obs.t_rx_s.0);
    ctx.ekf.predict(&ctx.model, dt_s);

    let mut used = 0;
    let mut used_satellite_positions = Vec::new();
    let mut stale_ephemeris_rejections = 0usize;
    let mut sats: Vec<&bijux_gnss_infra::api::core::ObsSatellite> = obs.sats.iter().collect();
    sats.sort_by_key(|s| s.signal_id);
    let sat_count = sats.len();
    let receive_tow_s = obs.gps_time().map(|gps_time| gps_time.tow_s).unwrap_or(obs.t_rx_s.0);
    for sat in sats {
        let eph = match ephs.iter().find(|e| e.sat == sat.signal_id.sat) {
            Some(e) => e,
            None => continue,
        };
        if !bijux_gnss_infra::api::nav::is_ephemeris_valid(eph, receive_tow_s) {
            stale_ephemeris_rejections += 1;
            continue;
        }
        let _corr = bijux_gnss_infra::api::nav::compute_corrections(&ctx.corrections);
        let state = cli_nav_satellite_state(sat, eph, receive_tow_s);
        let state_next = cli_nav_satellite_state(sat, eph, receive_tow_s + 0.1);
        let sat_vel = [
            (state_next.x_m - state.x_m) / 0.1,
            (state_next.y_m - state.y_m) / 0.1,
            (state_next.z_m - state.z_m) / 0.1,
        ];
        let rx_x = ctx.ekf.x[0];
        let rx_y = ctx.ekf.x[1];
        let rx_z = ctx.ekf.x[2];
        let (_az, el) = elevation_azimuth_deg(rx_x, rx_y, rx_z, state.x_m, state.y_m, state.z_m);
        let iono_m =
            estimate_ekf_klobuchar_delay_m(klobuchar, [rx_x, rx_y, rx_z], receive_tow_s, &state);
        let tropo_m =
            estimate_ekf_saastamoinen_delay_m(ctx.tropo_enabled, [rx_x, rx_y, rx_z], &state);
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
            sat_clock_s: state.clock_correction.bias_s,
            tropo_m,
            iono_m,
            sigma_m,
            elevation_deg: Some(el),
            ztd_index: ctx.ztd_index,
            isb_index,
        };
        if ctx.ekf.update(&meas) {
            used += 1;
            used_satellite_positions.push([state.x_m, state.y_m, state.z_m]);
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
            sat_clock_s: state.clock_correction.bias_s,
            tropo_m,
            iono_m,
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

    let mut explain_reasons = vec![format!("usable_satellites={used}")];
    if stale_ephemeris_rejections > 0 {
        explain_reasons.push(format!("stale_ephemeris_rejections={stale_ephemeris_rejections}"));
    }
    explain_reasons.push(if klobuchar.is_some() {
        "ionosphere_correction=klobuchar_broadcast".to_string()
    } else {
        "ionosphere_uncorrected".to_string()
    });
    explain_reasons.push(if ctx.tropo_enabled {
        "troposphere_correction=saastamoinen".to_string()
    } else {
        "troposphere_uncorrected".to_string()
    });

    let geometry_dops = position_dops_from_satellite_positions(
        [ctx.ekf.x[0], ctx.ekf.x[1], ctx.ekf.x[2]],
        &used_satellite_positions,
    );
    let geometry_violations =
        cli_nav_geometry_threshold_violations(ctx.science_thresholds, used, geometry_dops.as_ref());
    if !geometry_violations.is_empty() {
        explain_reasons.extend(geometry_violations);
        let refusal_class = if stale_ephemeris_rejections > 0 {
            if used == 0 {
                Some(bijux_gnss_infra::api::core::NavRefusalClass::InvalidEphemeris)
            } else {
                Some(bijux_gnss_infra::api::core::NavRefusalClass::InsufficientGeometry)
            }
        } else {
            Some(bijux_gnss_infra::api::core::NavRefusalClass::InsufficientGeometry)
        };
        let mut solution = cli_nav_refusal_epoch(
            obs,
            sat_count,
            used,
            sat_count.saturating_sub(used),
            refusal_class,
            explain_reasons,
        );
        populate_cli_nav_solution_dops(&mut solution, geometry_dops.as_ref());
        solution.innovation_rms_m = Some(ctx.ekf.health.innovation_rms);
        solution.ekf_innovation_rms = Some(ctx.ekf.health.innovation_rms);
        solution.ekf_condition_number = ctx.ekf.health.condition_number;
        solution.ekf_whiteness_ratio = ctx.ekf.health.whiteness_ratio;
        solution.ekf_predicted_variance = ctx.ekf.health.predicted_variance;
        solution.ekf_observed_variance = ctx.ekf.health.observed_variance;
        populate_cli_nav_solution_trace_identity(obs, &mut solution);
        return Ok(Some(solution));
    }
    let (lat, lon, alt) =
        bijux_gnss_infra::api::nav::ecef_to_geodetic(ctx.ekf.x[0], ctx.ekf.x[1], ctx.ekf.x[2]);
    let status = bijux_gnss_infra::api::core::SolutionStatus::Float;
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
        clock_bias_m: bijux_gnss_infra::api::core::Meters(ctx.ekf.x[6] * 299_792_458.0),
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
        explain_reasons,
        provenance: None,
        sat_count,
        used_sat_count: used,
        rejected_sat_count: sat_count.saturating_sub(used),
        hdop: None,
        vdop: None,
        gdop: None,
        tdop: None,
        stability_signature: String::new(),
        stability_signature_version:
            bijux_gnss_infra::api::core::NAV_OUTPUT_STABILITY_SIGNATURE_VERSION,
    };
    populate_cli_nav_solution_dops(&mut solution, geometry_dops.as_ref());
    populate_cli_nav_solution_trace_identity(obs, &mut solution);
    Ok(Some(solution))
}

fn cli_nav_geometry_threshold_violations(
    thresholds: EkfScienceThresholds,
    used_sat_count: usize,
    geometry_dops: Option<&PositionDops>,
) -> Vec<String> {
    let mut violations = Vec::new();
    if used_sat_count < thresholds.min_used_satellites {
        violations.push(format!("minimum_usable_satellites={}", thresholds.min_used_satellites));
        violations.push(format!(
            "used_satellites_below_threshold:{used_sat_count}<{}",
            thresholds.min_used_satellites
        ));
    }
    match geometry_dops {
        Some(dops) => {
            if dops.pdop > thresholds.max_pdop {
                violations.push(format!(
                    "pdop_above_threshold:{:.3}>{:.3}",
                    dops.pdop, thresholds.max_pdop
                ));
            }
            if dops.gdop > thresholds.max_gdop {
                violations.push(format!(
                    "gdop_above_threshold:{:.3}>{:.3}",
                    dops.gdop, thresholds.max_gdop
                ));
            }
        }
        None if used_sat_count >= thresholds.min_used_satellites => {
            violations.push("geometry_dops_unavailable".to_string());
        }
        None => {}
    }
    violations
}

fn populate_cli_nav_solution_dops(
    solution: &mut bijux_gnss_infra::api::core::NavSolutionEpoch,
    geometry_dops: Option<&PositionDops>,
) {
    if let Some(dops) = geometry_dops {
        solution.pdop = dops.pdop;
        solution.hdop = Some(dops.hdop);
        solution.vdop = Some(dops.vdop);
        solution.gdop = Some(dops.gdop);
    }
}

fn cli_nav_refusal_epoch(
    obs: &ObsEpoch,
    sat_count: usize,
    used_sat_count: usize,
    rejected_sat_count: usize,
    refusal_class: Option<bijux_gnss_infra::api::core::NavRefusalClass>,
    explain_reasons: Vec<String>,
) -> bijux_gnss_infra::api::core::NavSolutionEpoch {
    bijux_gnss_infra::api::core::NavSolutionEpoch {
        epoch: bijux_gnss_infra::api::core::Epoch { index: obs.epoch_idx },
        t_rx_s: obs.t_rx_s,
        source_time: obs.source_time,
        ecef_x_m: bijux_gnss_infra::api::core::Meters(0.0),
        ecef_y_m: bijux_gnss_infra::api::core::Meters(0.0),
        ecef_z_m: bijux_gnss_infra::api::core::Meters(0.0),
        latitude_deg: 0.0,
        longitude_deg: 0.0,
        altitude_m: bijux_gnss_infra::api::core::Meters(0.0),
        clock_bias_s: bijux_gnss_infra::api::core::Seconds(0.0),
        clock_bias_m: bijux_gnss_infra::api::core::Meters(0.0),
        clock_drift_s_per_s: 0.0,
        pdop: 0.0,
        rms_m: bijux_gnss_infra::api::core::Meters(0.0),
        status: bijux_gnss_infra::api::core::SolutionStatus::Invalid,
        quality: bijux_gnss_infra::api::core::SolutionStatus::Invalid.quality_flag(),
        validity: bijux_gnss_infra::api::core::SolutionValidity::Invalid,
        valid: false,
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
        lifecycle_state: bijux_gnss_infra::api::core::NavLifecycleState::Invalid,
        uncertainty_class: bijux_gnss_infra::api::core::NavUncertaintyClass::Unknown,
        assumptions: None,
        refusal_class,
        artifact_id: String::new(),
        source_observation_epoch_id: String::new(),
        explain_decision: "refused".to_string(),
        explain_reasons,
        provenance: None,
        sat_count,
        used_sat_count,
        rejected_sat_count,
        hdop: None,
        vdop: None,
        gdop: None,
        tdop: None,
        stability_signature: String::new(),
        stability_signature_version:
            bijux_gnss_infra::api::core::NAV_OUTPUT_STABILITY_SIGNATURE_VERSION,
    }
}

fn ekf_position_is_uninitialized(ctx: &EkfContext) -> bool {
    let radius_m = (ctx.ekf.x[0].powi(2) + ctx.ekf.x[1].powi(2) + ctx.ekf.x[2].powi(2)).sqrt();
    !radius_m.is_finite() || radius_m < 1.0
}

fn prime_ekf_state_from_wls(
    ctx: &mut EkfContext,
    obs: &ObsEpoch,
    ephs: &[GpsEphemeris],
    klobuchar: Option<&bijux_gnss_infra::api::nav::KlobucharCoefficients>,
    tropo_enabled: bool,
) {
    let observations = obs
        .sats
        .iter()
        .filter(|sat| sat.signal_id.sat.constellation == Constellation::Gps)
        .map(|sat| bijux_gnss_infra::api::nav::PositionObservation {
            sat: sat.signal_id.sat,
            pseudorange_m: sat.pseudorange_m.0,
            cn0_dbhz: sat.cn0_dbhz,
            elevation_deg: sat.elevation_deg,
            weight: 1.0,
            gps_receive_time: obs.gps_time(),
            signal_timing: sat.timing,
        })
        .filter(|observation| {
            bijux_gnss_infra::api::nav::position_observation_has_valid_satellite_time(
                observation,
                obs.t_rx_s.0,
            )
        })
        .collect::<Vec<_>>();
    if observations.len() < 4 {
        return;
    }
    let Some(solution) = bijux_gnss_infra::api::nav::PositionSolver {
        apply_troposphere: tropo_enabled,
        ..bijux_gnss_infra::api::nav::PositionSolver::new()
    }
    .solve_wls_with_broadcast_ionosphere(
        &observations,
        ephs,
        obs.gps_time().map(|gps_time| gps_time.tow_s).unwrap_or(obs.t_rx_s.0),
        klobuchar,
    ) else {
        return;
    };
    ctx.ekf.x[0] = solution.ecef_x_m;
    ctx.ekf.x[1] = solution.ecef_y_m;
    ctx.ekf.x[2] = solution.ecef_z_m;
    if ctx.ekf.x.len() > 6 {
        ctx.ekf.x[6] = solution.clock_bias_s;
    }
}

fn estimate_ekf_klobuchar_delay_m(
    klobuchar: Option<&bijux_gnss_infra::api::nav::KlobucharCoefficients>,
    receiver_ecef_m: [f64; 3],
    receive_tow_s: f64,
    state: &bijux_gnss_infra::api::nav::GpsSatState,
) -> f64 {
    let Some(coefficients) = klobuchar else {
        return 0.0;
    };
    let radius_m =
        (receiver_ecef_m[0].powi(2) + receiver_ecef_m[1].powi(2) + receiver_ecef_m[2].powi(2))
            .sqrt();
    if !radius_m.is_finite() || radius_m < 1.0 {
        return 0.0;
    }
    let (lat_deg, lon_deg, alt_m) = bijux_gnss_infra::api::nav::ecef_to_geodetic(
        receiver_ecef_m[0],
        receiver_ecef_m[1],
        receiver_ecef_m[2],
    );
    let receiver = bijux_gnss_infra::api::core::Llh { lat_deg, lon_deg, alt_m };
    let (azimuth_deg, elevation_deg) = elevation_azimuth_deg(
        receiver_ecef_m[0],
        receiver_ecef_m[1],
        receiver_ecef_m[2],
        state.x_m,
        state.y_m,
        state.z_m,
    );
    if !azimuth_deg.is_finite() || !elevation_deg.is_finite() || elevation_deg <= 0.0 {
        return 0.0;
    }
    let model = bijux_gnss_infra::api::nav::KlobucharModel::new(*coefficients);
    bijux_gnss_infra::api::nav::IonosphereModel::delay_m(
        &model,
        receiver,
        azimuth_deg,
        elevation_deg,
        bijux_gnss_infra::api::core::Seconds(receive_tow_s),
    )
}

fn estimate_ekf_saastamoinen_delay_m(
    tropo_enabled: bool,
    receiver_ecef_m: [f64; 3],
    state: &bijux_gnss_infra::api::nav::GpsSatState,
) -> f64 {
    if !tropo_enabled {
        return 0.0;
    }
    let radius_m =
        (receiver_ecef_m[0].powi(2) + receiver_ecef_m[1].powi(2) + receiver_ecef_m[2].powi(2))
            .sqrt();
    if !radius_m.is_finite() || !(6_000_000.0..=7_000_000.0).contains(&radius_m) {
        return 0.0;
    }
    let (lat_deg, lon_deg, alt_m) = bijux_gnss_infra::api::nav::ecef_to_geodetic(
        receiver_ecef_m[0],
        receiver_ecef_m[1],
        receiver_ecef_m[2],
    );
    if !lat_deg.is_finite()
        || !lon_deg.is_finite()
        || !alt_m.is_finite()
        || !(-1_000.0..=20_000.0).contains(&alt_m)
    {
        return 0.0;
    }
    let receiver = bijux_gnss_infra::api::core::Llh { lat_deg, lon_deg, alt_m };
    let (_azimuth_deg, elevation_deg) = elevation_azimuth_deg(
        receiver_ecef_m[0],
        receiver_ecef_m[1],
        receiver_ecef_m[2],
        state.x_m,
        state.y_m,
        state.z_m,
    );
    if !elevation_deg.is_finite() || elevation_deg <= 0.0 {
        return 0.0;
    }
    let model = bijux_gnss_infra::api::nav::SaastamoinenModel;
    bijux_gnss_infra::api::nav::TroposphereModel::delay_m(
        &model,
        receiver,
        elevation_deg,
        bijux_gnss_infra::api::core::Seconds(0.0),
    )
}

fn cli_nav_satellite_state(
    sat: &bijux_gnss_infra::api::core::ObsSatellite,
    eph: &GpsEphemeris,
    receive_tow_s: f64,
) -> bijux_gnss_infra::api::nav::GpsSatState {
    let signal_travel_time_s = sat
        .timing
        .map(|timing| timing.signal_travel_time_s.0)
        .unwrap_or(sat.pseudorange_m.0 / 299_792_458.0);
    bijux_gnss_infra::api::nav::sat_state_gps_l1ca_at_receive_time(
        eph,
        receive_tow_s,
        signal_travel_time_s,
    )
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
    solution.artifact_id = format!(
        "nav-epoch-{:010}-{}",
        solution.epoch.index,
        cli_nav_trace_short_id(&source_observation_epoch_id)
    );
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
