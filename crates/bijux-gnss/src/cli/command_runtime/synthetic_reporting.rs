use bijux_gnss_infra::api::core::format_sat;

pub(crate) fn print_synthetic_iq_export_table(report: &SyntheticIqExportReport) {
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

pub(crate) fn print_synthetic_iq_validation_table(report: &SyntheticIqValidationReport) {
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

pub(crate) fn print_synthetic_navigation_validation_table(
    report: &SyntheticNavigationValidationReport,
) {
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

pub(crate) fn print_synthetic_quantization_measurement_table(
    report: &SyntheticQuantizationMeasurementReport,
) {
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
