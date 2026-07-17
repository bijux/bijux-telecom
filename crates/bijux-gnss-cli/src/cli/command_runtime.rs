fn execute_gnss_command(command: GnssCommand) -> Result<()> {
    match command {
        cmd @ GnssCommand::CaCode { .. } => handle_ca_code(cmd),
        cmd @ GnssCommand::Acquire { .. } => handle_acquire(cmd),
        cmd @ GnssCommand::Track { .. } => handle_track(cmd),
        cmd @ GnssCommand::Nav { .. } => handle_nav(cmd),
        cmd @ GnssCommand::Pvt { .. } => handle_pvt(cmd),
        cmd @ GnssCommand::Inspect { .. } => handle_inspect(cmd),
        cmd @ GnssCommand::Rtk { .. } => handle_rtk(cmd),
        cmd @ GnssCommand::Experiment { .. } => handle_experiment(cmd),
        cmd @ GnssCommand::ExportSyntheticIq { .. } => handle_export_synthetic_iq(cmd),
        cmd @ GnssCommand::ValidateSyntheticIq { .. } => handle_validate_synthetic_iq(cmd),
        cmd @ GnssCommand::ValidateSyntheticNavigation { .. } => {
            handle_validate_synthetic_navigation(cmd)
        }
        cmd @ GnssCommand::MeasureSyntheticQuantization { .. } => {
            handle_measure_synthetic_quantization(cmd)
        }
        cmd @ GnssCommand::ValidateConfig { .. } => handle_validate_config(cmd),
        cmd @ GnssCommand::Config { .. } => handle_config(cmd),
        cmd @ GnssCommand::ValidateArtifacts { .. } => handle_validate_artifacts(cmd),
        cmd @ GnssCommand::ValidateSidecar { .. } => handle_validate_sidecar(cmd),
        cmd @ GnssCommand::Analyze { .. } => handle_analyze(cmd),
        cmd @ GnssCommand::Diff { .. } => handle_diff(cmd),
        cmd @ GnssCommand::Artifact { .. } => handle_artifact(cmd),
        cmd @ GnssCommand::Diagnostics { .. } => handle_diagnostics(cmd),
        cmd @ GnssCommand::ConfigUpgrade { .. } => handle_config_upgrade(cmd),
        cmd @ GnssCommand::ConfigSchema { .. } => handle_config_schema(cmd),
        cmd @ GnssCommand::Validate { .. } => handle_validate(cmd),
        cmd @ GnssCommand::ValidateReference { .. } => handle_validate_reference(cmd),
        cmd @ GnssCommand::ValidateCapture { .. } => handle_validate_capture(cmd),
        cmd @ GnssCommand::Run { .. } => handle_run(cmd),
        cmd @ GnssCommand::Rinex { .. } => handle_rinex(cmd),
        cmd @ GnssCommand::Doctor { .. } => handle_doctor(cmd),
    }
}

mod runtime_environment {
    use super::*;

    include!("command_runtime/runtime_environment.rs");
}

#[cfg(test)]
pub(crate) use runtime_environment::capture_start_gps_time;
pub(crate) use runtime_environment::{runtime_config_from_capture_start, runtime_config_from_env};

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

fn print_synthetic_navigation_validation_table(report: &SyntheticNavigationValidationReport) {
    println!("Scenario: {}", report.scenario_id);
    println!("Scenario path: {}", report.scenario_path);
    println!("Artifact: {}", report.output_artifact);
    println!("Pass: {}", report.pass);
    println!("Truth coverage ready: {}", report.truth_coverage_ready);
    println!("Closure ready: {}", report.closure_ready);
    println!(
        "Data source: kind={} sample_rate_hz={:.1} intermediate_freq_hz={:.1} duration_s={:.3} satellites={}",
        report.data_source.source_kind,
        report.data_source.sample_rate_hz,
        report.data_source.intermediate_freq_hz,
        report.data_source.duration_s,
        report.data_source.satellite_count
    );
    println!(
        "Reference truth: kind={} receiver_ecef_m={:?} receive_time_s={:?} satellites={} epochs={}",
        report.reference_truth.truth_kind,
        report.reference_truth.receiver_ecef_m,
        report.reference_truth.reference_receive_time_s,
        report.reference_truth.satellite_count,
        report.reference_truth.reference_epoch_count
    );
    println!("Stage\tPass\tTruthReady\tMeasured\tPassing\tThresholds");
    println!(
        "acquisition\t{}\t{}\t{}\t{}\tdoppler_hz<={:.3}, code_phase_samples<={}",
        report.acquisition.pass,
        report.acquisition.truth_coverage_ready,
        report.acquisition.satellite_count,
        report.acquisition.passing_satellite_count,
        report.acquisition.threshold_max_doppler_error_hz,
        report.acquisition.threshold_max_code_phase_error_samples
    );
    println!(
        "tracking\t{}\t{}\t{}\t{}\tcarrier_hz<={:.3}, doppler_hz<={:.3}, code_phase_samples<={:.3}, cn0_db_hz<={:.3}",
        report.tracking.pass,
        report.tracking.truth_coverage_ready,
        report.tracking.satellite_count,
        report.tracking.passing_satellite_count,
        report.tracking.threshold_max_carrier_error_hz,
        report.tracking.threshold_max_doppler_error_hz,
        report.tracking.threshold_max_code_phase_error_samples,
        report.tracking.threshold_max_cn0_error_db_hz
    );
    println!(
        "observation\t{}\t{}\t{}\t{}\tpseudorange_m<={:.6}, carrier_phase_cycles<={:.6}, doppler_hz<={:.6}, cn0_db_hz<={:.6}",
        report.observation.pass,
        report.observation.truth_coverage_ready,
        report.observation.satellite_count,
        report.observation.passing_satellite_count,
        report.observation.threshold_max_pseudorange_error_m,
        report.observation.threshold_max_carrier_phase_error_cycles,
        report.observation.threshold_max_doppler_error_hz,
        report.observation.threshold_max_cn0_error_db_hz
    );
    println!(
        "pvt\t{}\t{}\t{}\t{}\tposition_3d_m<={:.3}, clock_bias_m<={:.3}, residual_rms_m<={:.3}, pdop<={:.3}",
        report.pvt.pass,
        report.pvt.truth_coverage_ready,
        report.pvt.epoch_count,
        report.pvt.passing_epoch_count,
        report.pvt.threshold_max_position_error_3d_m,
        report.pvt.threshold_max_clock_bias_error_m,
        report.pvt.threshold_max_residual_rms_m,
        report.pvt.threshold_max_pdop
    );
    println!(
        "closure\t{}\t{}\t{}\t{}\tapplicable={}, not_applicable={}",
        report.closure.pass,
        report.closure_ready,
        report.closure.stages.len(),
        report.closure.passed_stage_count,
        report.closure.applicable_stage_count,
        report.closure.not_applicable_stage_count
    );
}

fn print_synthetic_quantization_measurement_table(report: &SyntheticQuantizationMeasurementReport) {
    println!("Scenario: {}", report.scenario_id);
    println!("Scenario path: {}", report.scenario_path);
    println!("Artifact: {}", report.output_artifact);
    println!("Reference quantization: {}", report.measurement.reference_quantization);
    println!(
        "Measured quantizations: {}",
        report
            .measured_quantizations
            .iter()
            .map(ToString::to_string)
            .collect::<Vec<_>>()
            .join(", ")
    );
    println!(
        "Coherent integration: {} samples ({:.6} s)",
        report.measurement.coherent_samples_per_epoch, report.measurement.coherent_integration_s
    );
    println!(
        "Quantization\tBits\tFormat\tSatellite\tPeakLoss(dB)\tCn0Loss(dB-Hz)\tRefPeak\tQuantPeak\tRefCn0\tQuantCn0"
    );
    for point in &report.measurement.points {
        for satellite in &point.satellites {
            println!(
                "{}\t{}\t{:?}\t{}\t{}\t{:.6}\t{:.6}\t{:.6}\t{:.6}\t{:.6}",
                point.quantization,
                point.quantization_bits,
                point.sample_format,
                format_sat(satellite.sat),
                satellite
                    .acquisition_correlation_loss_db
                    .map(|value| format!("{value:.6}"))
                    .unwrap_or_else(|| "unbounded".to_string()),
                satellite.cn0_loss_db_hz,
                satellite.reference_acquisition_peak,
                satellite.quantized_acquisition_peak,
                satellite.reference_mean_cn0_db_hz,
                satellite.quantized_mean_cn0_db_hz,
            );
        }
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

#[cfg(feature = "tracing")]
fn init_tracing() {
    let _ = tracing_subscriber::fmt().with_env_filter("info").with_target(false).try_init();
}

#[cfg(not(feature = "tracing"))]
fn init_tracing() {}

fn main() -> Result<()> {
    init_tracing();
    let command_line = CommandLine::parse();
    match command_line.command {
        ApplicationCommand::Gnss { command } => execute_gnss_command(command),
    }
}
