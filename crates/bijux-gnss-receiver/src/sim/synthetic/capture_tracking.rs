/// Build a machine-readable truth bundle for an emitted synthetic capture.
pub fn build_truth_bundle(
    scenario_id: &str,
    scenario: &SyntheticScenario,
    frame: &SamplesFrame,
    metadata: &RawIqMetadata,
    quantization: IqQuantization,
    peak_component_before_scaling: f32,
    output_scale_applied: f32,
) -> SyntheticIqTruthBundle {
    build_truth_bundle_with_source_front_end(
        scenario_id,
        scenario,
        frame,
        metadata,
        quantization,
        peak_component_before_scaling,
        output_scale_applied,
        None,
    )
}

pub fn build_truth_bundle_with_source_front_end(
    scenario_id: &str,
    scenario: &SyntheticScenario,
    frame: &SamplesFrame,
    metadata: &RawIqMetadata,
    quantization: IqQuantization,
    peak_component_before_scaling: f32,
    output_scale_applied: f32,
    source_front_end_filter: Option<&bijux_gnss_signal::api::FrontEndFilterSpec>,
) -> SyntheticIqTruthBundle {
    let noise_std_per_component =
        source_front_end_noise_std_per_component(source_front_end_filter, frame.t0.sample_rate_hz);
    let noise_power_per_complex_sample = 2.0 * noise_std_per_component * noise_std_per_component;
    SyntheticIqTruthBundle {
        schema_version: SYNTHETIC_IQ_TRUTH_SCHEMA_VERSION,
        scenario_id: scenario_id.to_string(),
        seed: scenario.seed,
        sample_format: metadata.format,
        quantization,
        sample_rate_hz: frame.t0.sample_rate_hz,
        intermediate_freq_hz: metadata.intermediate_freq_hz,
        receiver_clock_frequency_bias_hz: scenario.receiver_clock_frequency_bias_hz,
        capture_start_utc: metadata.capture_start_utc.clone(),
        quantization_bits: metadata.quantization_bits.unwrap_or_default(),
        duration_s: frame.len() as f64 * frame.dt_s.0,
        sample_count: frame.len(),
        noise_std_per_component,
        noise_power_per_complex_sample,
        source_front_end_filter: source_front_end_filter.cloned(),
        source_front_end_sample_delay_samples: source_front_end_sample_delay_samples(
            source_front_end_filter,
        ),
        peak_component_before_scaling,
        output_scale_applied,
        satellites: scenario
            .satellites
            .iter()
            .map(|params| {
                let nav_bit_mode = nav_bit_mode(params);
                SyntheticSatelliteTruth {
                    sat: params.sat,
                    glonass_frequency_channel: params.glonass_frequency_channel,
                    signal_band: params.signal_band,
                    signal_code: params.signal_code,
                    doppler_hz: params.doppler_hz,
                    code_phase_chips: params.code_phase_chips,
                    carrier_phase_rad: params.carrier_phase_rad,
                    cn0_db_hz: params.cn0_db_hz,
                    signal_amplitude: signal_amplitude_from_cn0(
                        params.cn0_db_hz,
                        frame.t0.sample_rate_hz,
                    ),
                    navigation_data: params.navigation_data.clone(),
                    nav_bit_mode,
                    nav_bit_segments: nav_bit_segments(
                        frame.t0.sample_rate_hz,
                        frame.len() as u64,
                        params,
                        nav_bit_mode,
                    ),
                }
            })
            .collect(),
    }
}

/// Encode a generated synthetic frame with one explicit quantization profile and truth metadata.
pub fn build_quantized_capture_bundle(
    scenario_id: &str,
    scenario: &SyntheticScenario,
    frame: &SamplesFrame,
    quantization: IqQuantization,
    capture_start_utc: &str,
    notes: Option<String>,
) -> SyntheticIqCaptureBundle {
    build_quantized_capture_bundle_with_source_front_end(
        scenario_id,
        scenario,
        frame,
        quantization,
        capture_start_utc,
        notes,
        None,
    )
}

pub fn build_quantized_capture_bundle_with_source_front_end(
    scenario_id: &str,
    scenario: &SyntheticScenario,
    frame: &SamplesFrame,
    quantization: IqQuantization,
    capture_start_utc: &str,
    notes: Option<String>,
    source_front_end_filter: Option<&bijux_gnss_signal::api::FrontEndFilterSpec>,
) -> SyntheticIqCaptureBundle {
    let peak_component_before_scaling = peak_component(&frame.iq);
    let output_scale_applied = if peak_component_before_scaling <= 0.999 {
        1.0
    } else {
        0.999 / peak_component_before_scaling
    };
    let scaled_iq = frame.iq.iter().map(|sample| *sample * output_scale_applied).collect::<Vec<_>>();
    let raw_iq_bytes = encode_quantized_samples(&scaled_iq, quantization);
    let metadata = RawIqMetadata {
        format: quantization.sample_format(),
        sample_rate_hz: frame.t0.sample_rate_hz,
        intermediate_freq_hz: scenario.intermediate_freq_hz,
        capture_start_utc: capture_start_utc.to_string(),
        offset_bytes: 0,
        quantization_bits: Some(quantization.quantization_bits()),
        notes,
    };
    let truth = build_truth_bundle_with_source_front_end(
        scenario_id,
        scenario,
        frame,
        &metadata,
        quantization,
        peak_component_before_scaling,
        output_scale_applied,
        source_front_end_filter,
    );
    SyntheticIqCaptureBundle { raw_iq_bytes, metadata, truth }
}

/// Encode a generated synthetic frame as an IQ16 little-endian capture with truth metadata.
pub fn build_iq16_capture_bundle(
    scenario_id: &str,
    scenario: &SyntheticScenario,
    frame: &SamplesFrame,
    capture_start_utc: &str,
    notes: Option<String>,
) -> SyntheticIqCaptureBundle {
    build_quantized_capture_bundle(
        scenario_id,
        scenario,
        frame,
        IqQuantization::Signed16Bit,
        capture_start_utc,
        notes,
    )
}

pub fn build_iq16_capture_bundle_with_source_front_end(
    scenario_id: &str,
    scenario: &SyntheticScenario,
    frame: &SamplesFrame,
    capture_start_utc: &str,
    notes: Option<String>,
    source_front_end_filter: Option<&bijux_gnss_signal::api::FrontEndFilterSpec>,
) -> SyntheticIqCaptureBundle {
    build_quantized_capture_bundle_with_source_front_end(
        scenario_id,
        scenario,
        frame,
        IqQuantization::Signed16Bit,
        capture_start_utc,
        notes,
        source_front_end_filter,
    )
}

/// Measure truth-guided C/N0 directly from a synthetic capture using receiver correlators.
pub fn validate_truth_guided_cn0(
    config: &ReceiverPipelineConfig,
    frame: &SamplesFrame,
    truth: &SyntheticIqTruthBundle,
    tolerance_db_hz: f64,
) -> SyntheticCn0ValidationReport {
    let coherent_samples_per_epoch =
        samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length)
            .min(frame.len());
    let coherent_integration_s = coherent_samples_per_epoch as f64 * frame.dt_s.0;
    let period_samples =
        samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length);
    let satellites = truth
        .satellites
        .iter()
        .map(|sat_truth| {
            let isolated_frame = regenerate_isolated_scaled_satellite_frame_with_noise(
                config, frame, truth, sat_truth,
            );
            let tracking = crate::pipeline::tracking::Tracking::new(
                config.clone(),
                crate::engine::runtime::ReceiverRuntime::default(),
            );
            let seeded_code_phase_samples = wrap_seeded_code_phase_samples(
                expected_truth_guided_acquisition_code_phase_samples_f64(
                    config,
                    &isolated_frame,
                    truth,
                    sat_truth.code_phase_chips,
                )
                .round() as isize,
                period_samples,
            );
            let doppler_hz = synthetic_truth_measured_doppler_hz(truth, sat_truth);
            let tracks = tracking.track_from_acquisition(
                &isolated_frame,
                &[seeded_tracking_acquisition(
                    sat_truth.sat,
                    sat_truth.signal_band,
                    sat_truth.signal_code,
                    sat_truth.glonass_frequency_channel,
                    doppler_hz,
                    config.intermediate_freq_hz,
                    seeded_code_phase_samples,
                    sat_truth.cn0_db_hz,
                    format!("truth_guided_cn0_tracking_seed_{}", sat_truth.sat.prn),
                )],
            );
            let cn0_values = tracks
                .first()
                .map(|track| stable_tracking_cn0_values(&track.epochs))
                .unwrap_or_default();
            let measured_mean_cn0_dbhz = if cn0_values.is_empty() {
                0.0
            } else {
                cn0_values.iter().sum::<f64>() / cn0_values.len() as f64
            };
            let measured_min_cn0_dbhz = cn0_values.iter().copied().reduce(f64::min).unwrap_or(0.0);
            let measured_max_cn0_dbhz = cn0_values.iter().copied().reduce(f64::max).unwrap_or(0.0);
            let cn0_delta_db = measured_mean_cn0_dbhz - sat_truth.cn0_db_hz as f64;
            let pass = !cn0_values.is_empty() && cn0_delta_db.abs() <= tolerance_db_hz;

            SyntheticCn0ValidationSatellite {
                sat: sat_truth.sat,
                injected_cn0_db_hz: sat_truth.cn0_db_hz,
                measured_mean_cn0_dbhz,
                measured_min_cn0_dbhz,
                measured_max_cn0_dbhz,
                cn0_delta_db,
                signal_amplitude: sat_truth.signal_amplitude,
                output_signal_amplitude: sat_truth.signal_amplitude * truth.output_scale_applied,
                epochs_measured: cn0_values.len(),
                pass,
            }
        })
        .collect::<Vec<_>>();
    let pass = !satellites.is_empty() && satellites.iter().all(|row| row.pass);

    SyntheticCn0ValidationReport {
        scenario_id: truth.scenario_id.clone(),
        tolerance_db_hz,
        sample_rate_hz: truth.sample_rate_hz,
        coherent_samples_per_epoch,
        coherent_integration_s,
        quantization_bits: truth.quantization_bits,
        noise_std_per_component: truth.noise_std_per_component,
        noise_power_per_complex_sample: truth.noise_power_per_complex_sample,
        output_scale_applied: truth.output_scale_applied,
        pass,
        satellites,
    }
}

/// Measure acquisition correlation loss and tracking C/N0 loss against a float32 reference.
pub fn measure_truth_guided_quantization_loss(
    config: &ReceiverPipelineConfig,
    scenario: &SyntheticScenario,
    quantizations: &[IqQuantization],
    capture_start_utc: &str,
) -> SyntheticQuantizationLossReport {
    let frame = generate_l1_ca_multi(config, scenario);
    let reference_bundle = build_quantized_capture_bundle(
        &scenario.id,
        scenario,
        &frame,
        IqQuantization::Float32,
        capture_start_utc,
        Some("synthetic float32 quantization reference".to_string()),
    );
    let reference_frame = quantized_capture_frame(
        &frame,
        reference_bundle.truth.output_scale_applied,
        IqQuantization::Float32,
    );
    let sat_ids = scenario.satellites.iter().map(|signal| signal.sat).collect::<Vec<_>>();
    let runtime = crate::engine::runtime::ReceiverRuntime::default();
    let acquisition = crate::pipeline::acquisition::Acquisition::new(config.clone(), runtime);
    let reference_acquisition = acquisition.run_fft(&reference_frame, &sat_ids);
    let reference_cn0 =
        validate_truth_guided_cn0(config, &reference_frame, &reference_bundle.truth, f64::INFINITY);
    let coherent_samples_per_epoch = reference_cn0.coherent_samples_per_epoch;
    let coherent_integration_s = reference_cn0.coherent_integration_s;

    let points = quantizations
        .iter()
        .copied()
        .map(|quantization| {
            let bundle = build_quantized_capture_bundle(
                &scenario.id,
                scenario,
                &frame,
                quantization,
                capture_start_utc,
                Some(format!(
                    "synthetic {:?} quantization measurement",
                    quantization
                )),
            );
            let quantized_frame =
                quantized_capture_frame(&frame, bundle.truth.output_scale_applied, quantization);
            let quantized_acquisition = acquisition.run_fft(&quantized_frame, &sat_ids);
            let quantized_cn0 =
                validate_truth_guided_cn0(config, &quantized_frame, &bundle.truth, f64::INFINITY);
            let satellites = scenario
                .satellites
                .iter()
                .map(|signal| {
                    let reference_acq = find_acquisition_result(&reference_acquisition, signal.sat);
                    let quantized_acq = find_acquisition_result(&quantized_acquisition, signal.sat);
                    let reference_cn0_row = find_cn0_validation_row(&reference_cn0, signal.sat);
                    let quantized_cn0_row = find_cn0_validation_row(&quantized_cn0, signal.sat);
                    let reference_peak = reference_acq.map(|result| result.peak).unwrap_or_default();
                    let quantized_peak = quantized_acq.map(|result| result.peak).unwrap_or_default();
                    let reference_mean_cn0_db_hz =
                        reference_cn0_row.map(|row| row.measured_mean_cn0_dbhz).unwrap_or_default();
                    let quantized_mean_cn0_db_hz =
                        quantized_cn0_row.map(|row| row.measured_mean_cn0_dbhz).unwrap_or_default();

                    SyntheticQuantizationLossSatellite {
                        sat: signal.sat,
                        reference_acquisition_peak: reference_peak,
                        quantized_acquisition_peak: quantized_peak,
                        acquisition_correlation_loss_db: positive_amplitude_loss_db(
                            reference_peak as f64,
                            quantized_peak as f64,
                        ),
                        reference_peak_mean_ratio: reference_acq
                            .map(|result| result.peak_mean_ratio)
                            .unwrap_or_default(),
                        quantized_peak_mean_ratio: quantized_acq
                            .map(|result| result.peak_mean_ratio)
                            .unwrap_or_default(),
                        reference_mean_cn0_db_hz,
                        quantized_mean_cn0_db_hz,
                        cn0_loss_db_hz: (reference_mean_cn0_db_hz - quantized_mean_cn0_db_hz)
                            .max(0.0),
                    }
                })
                .collect();

            SyntheticQuantizationLossPoint {
                quantization,
                sample_format: bundle.metadata.format,
                quantization_bits: bundle.truth.quantization_bits,
                output_scale_applied: bundle.truth.output_scale_applied,
                satellites,
            }
        })
        .collect();

    SyntheticQuantizationLossReport {
        scenario_id: scenario.id.clone(),
        reference_quantization: IqQuantization::Float32,
        sample_rate_hz: config.sampling_freq_hz,
        intermediate_freq_hz: config.intermediate_freq_hz,
        coherent_samples_per_epoch,
        coherent_integration_s,
        points,
    }
}

const TRACKING_CN0_MIN_STABLE_EPOCHS: usize = 5;

fn stable_tracking_cn0_values(epochs: &[crate::api::core::TrackEpoch]) -> Vec<f64> {
    let Some((start, count)) =
        stable_tracking_window_bounds(epochs, TRACKING_CN0_MIN_STABLE_EPOCHS)
    else {
        return Vec::new();
    };
    epochs[start..start + count]
        .iter()
        .filter_map(|epoch| epoch.cn0_dbhz.is_finite().then_some(epoch.cn0_dbhz))
        .collect()
}

fn quantized_capture_frame(
    frame: &SamplesFrame,
    output_scale_applied: f32,
    quantization: IqQuantization,
) -> SamplesFrame {
    let scaled_iq = frame.iq.iter().map(|sample| *sample * output_scale_applied).collect::<Vec<_>>();
    SamplesFrame::new(
        frame.t0,
        frame.dt_s,
        quantize_samples_for_storage(&scaled_iq, quantization),
    )
}

fn find_acquisition_result(
    results: &[crate::api::core::AcqResult],
    sat: SatId,
) -> Option<&crate::api::core::AcqResult> {
    results.iter().find(|result| result.sat == sat)
}

fn find_cn0_validation_row(
    report: &SyntheticCn0ValidationReport,
    sat: SatId,
) -> Option<&SyntheticCn0ValidationSatellite> {
    report.satellites.iter().find(|row| row.sat == sat)
}

fn positive_amplitude_loss_db(reference_amplitude: f64, measured_amplitude: f64) -> f64 {
    if reference_amplitude <= f64::EPSILON || measured_amplitude <= f64::EPSILON {
        return f64::INFINITY;
    }
    (20.0 * (reference_amplitude / measured_amplitude).log10()).max(0.0)
}

pub fn expected_acquisition_code_phase_samples(
    config: &ReceiverPipelineConfig,
    frame: &SamplesFrame,
    code_phase_chips: f64,
) -> usize {
    expected_acquisition_code_phase_samples_f64(config, frame, code_phase_chips).round() as usize
}

/// Convert a truth-model code phase into the receiver's continuous acquisition sample convention.
pub fn expected_acquisition_code_phase_samples_f64(
    config: &ReceiverPipelineConfig,
    frame: &SamplesFrame,
    code_phase_chips: f64,
) -> f64 {
    receiver_search_code_phase_samples(
        frame.t0.sample_rate_hz,
        config.code_freq_basis_hz,
        config.code_length,
        frame.t0.sample_index,
        code_phase_chips,
    )
    .expect("synthetic acquisition code phase requires a valid code phase model")
}

fn expected_truth_guided_acquisition_code_phase_samples_f64(
    config: &ReceiverPipelineConfig,
    frame: &SamplesFrame,
    truth: &SyntheticIqTruthBundle,
    code_phase_chips: f64,
) -> f64 {
    expected_acquisition_code_phase_samples_f64(config, frame, code_phase_chips)
        + truth.source_front_end_sample_delay_samples as f64
}

/// Measure wrapped code-phase error in samples over one code period.
pub fn wrapped_code_phase_error_samples(
    actual: usize,
    expected: usize,
    period_samples: usize,
) -> usize {
    bijux_gnss_signal::api::wrapped_code_phase_error_samples(actual, expected, period_samples)
}

/// Measure wrapped code-phase error in samples over one code period for fractional estimates.
pub fn wrapped_code_phase_error_samples_f64(
    actual: f64,
    expected: f64,
    period_samples: usize,
) -> f64 {
    bijux_gnss_signal::api::wrapped_code_phase_error_samples_f64(actual, expected, period_samples)
}

fn code_phase_error_samples_to_pseudorange_m(error_samples: f64, sample_rate_hz: f64) -> f64 {
    (error_samples / sample_rate_hz) * SPEED_OF_LIGHT_MPS
}

fn tracking_epoch_is_stable(epoch: &crate::api::core::TrackEpoch) -> bool {
    epoch.lock
        && epoch.lock_state == "tracking"
        && epoch.pll_lock
        && epoch.fll_lock
        && !epoch.cycle_slip
        && epoch.lock_state_reason.as_deref() != Some("lock_lost")
}

fn expected_tracking_code_phase_samples(
    config: &ReceiverPipelineConfig,
    sample_rate_hz: f64,
    sample_index: u64,
    source_front_end_sample_delay_samples: u64,
    code_phase_chips: f64,
) -> f64 {
    receiver_search_code_phase_samples(
        sample_rate_hz,
        config.code_freq_basis_hz,
        config.code_length,
        sample_index,
        code_phase_chips,
    )
    .expect("synthetic tracking code phase requires a valid code phase model")
        + source_front_end_sample_delay_samples as f64
}

fn source_front_end_sample_delay_samples(
    source_front_end_filter: Option<&bijux_gnss_signal::api::FrontEndFilterSpec>,
) -> u64 {
    source_front_end_filter.map(|spec| spec.group_delay_samples() as u64).unwrap_or(0)
}

fn source_front_end_noise_std_per_component(
    source_front_end_filter: Option<&bijux_gnss_signal::api::FrontEndFilterSpec>,
    sample_rate_hz: f64,
) -> f32 {
    let Some(spec) = source_front_end_filter else {
        return SYNTHETIC_NOISE_STD_PER_COMPONENT;
    };
    let filter = spec
        .design(sample_rate_hz)
        .expect("validated synthetic source front-end filter must design");
    let tap_energy = filter.taps().iter().map(|tap| tap.norm_sqr()).sum::<f32>();
    SYNTHETIC_NOISE_STD_PER_COMPONENT * tap_energy.sqrt()
}

fn synthetic_measured_doppler_hz_from_carrier_hz(
    intermediate_freq_hz: f64,
    sat: SatId,
    signal_band: SignalBand,
    signal_code: SignalCode,
    glonass_frequency_channel: Option<GlonassFrequencyChannel>,
    carrier_hz: f64,
) -> f64 {
    crate::pipeline::doppler::doppler_hz_from_carrier_hz(
        synthetic_intermediate_frequency_hz(
            intermediate_freq_hz,
            sat,
            signal_band,
            signal_code,
            glonass_frequency_channel,
        ),
        carrier_hz,
    )
}

/// Build a truth-guided tracking table from a synthetic capture.
pub fn validate_truth_guided_tracking_table(
    config: &ReceiverPipelineConfig,
    frame: &SamplesFrame,
    truth: &SyntheticIqTruthBundle,
    carrier_tolerance_hz: f64,
    doppler_tolerance_hz: f64,
    code_phase_tolerance_samples: f64,
    cn0_tolerance_db_hz: f64,
) -> SyntheticTrackingTruthTableReport {
    let period_samples =
        samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length)
            .max(1);
    let satellites = truth
        .satellites
        .iter()
        .map(|sat_truth| {
            let isolated_frame = regenerate_isolated_scaled_satellite_frame_with_noise(
                config, frame, truth, sat_truth,
            );
            let expected_measured_doppler_hz =
                synthetic_truth_measured_doppler_hz(truth, sat_truth);
            let expected_carrier_hz = synthetic_carrier_hz(
                config.intermediate_freq_hz,
                sat_truth.sat,
                sat_truth.signal_band,
                sat_truth.signal_code,
                sat_truth.glonass_frequency_channel,
                expected_measured_doppler_hz,
            );
            let seeded_code_phase_samples = wrap_seeded_code_phase_samples(
                expected_truth_guided_acquisition_code_phase_samples_f64(
                    config,
                    &isolated_frame,
                    truth,
                    sat_truth.code_phase_chips,
                )
                .round() as isize,
                period_samples,
            );
            let refined_code_phase_samples =
                expected_truth_guided_acquisition_code_phase_samples_f64(
                    config,
                    &isolated_frame,
                    truth,
                    sat_truth.code_phase_chips,
                );
            let tracking = crate::pipeline::tracking::Tracking::new(
                config.clone(),
                crate::engine::runtime::ReceiverRuntime::default(),
            );
            let tracks = tracking.track_from_acquisition(
                &isolated_frame,
                &[seeded_tracking_acquisition_with_refined_code_phase(
                    sat_truth.sat,
                    sat_truth.signal_band,
                    sat_truth.signal_code,
                    sat_truth.glonass_frequency_channel,
                    expected_measured_doppler_hz,
                    config.intermediate_freq_hz,
                    seeded_code_phase_samples,
                    refined_code_phase_samples,
                    sat_truth.cn0_db_hz,
                    format!("truth_guided_tracking_seed_{}", sat_truth.sat.prn),
                )],
            );
            let epochs = tracks.first().map(|track| track.epochs.clone()).unwrap_or_default();
            let epoch_rows = epochs
                .iter()
                .enumerate()
                .map(|(epoch_index, epoch)| {
                    let expected_code_phase_samples = expected_tracking_code_phase_samples(
                        config,
                        truth.sample_rate_hz,
                        epoch.sample_index,
                        truth.source_front_end_sample_delay_samples,
                        sat_truth.code_phase_chips,
                    );
                    let measured_code_phase_samples = epoch.code_phase_samples.0;
                    let code_phase_error_samples = wrapped_code_phase_error_samples_f64(
                        measured_code_phase_samples,
                        expected_code_phase_samples,
                        period_samples,
                    );
                    let measured_carrier_hz = epoch.carrier_hz.0;
                    let carrier_error_hz = (measured_carrier_hz - expected_carrier_hz).abs();
                    let measured_doppler_hz = synthetic_measured_doppler_hz_from_carrier_hz(
                        config.intermediate_freq_hz,
                        sat_truth.sat,
                        sat_truth.signal_band,
                        sat_truth.signal_code,
                        sat_truth.glonass_frequency_channel,
                        measured_carrier_hz,
                    );
                    let doppler_error_hz =
                        (measured_doppler_hz - expected_measured_doppler_hz).abs();
                    let measured_cn0_dbhz = epoch.cn0_dbhz;
                    let cn0_error_db = (measured_cn0_dbhz - sat_truth.cn0_db_hz as f64).abs();
                    let stable_tracking_epoch = tracking_epoch_is_stable(epoch);
                    let pass = stable_tracking_epoch
                        && measured_carrier_hz.is_finite()
                        && measured_doppler_hz.is_finite()
                        && measured_code_phase_samples.is_finite()
                        && measured_cn0_dbhz.is_finite()
                        && carrier_error_hz <= carrier_tolerance_hz + f64::EPSILON
                        && doppler_error_hz <= doppler_tolerance_hz + f64::EPSILON
                        && code_phase_error_samples <= code_phase_tolerance_samples + f64::EPSILON
                        && cn0_error_db <= cn0_tolerance_db_hz + f64::EPSILON;

                    SyntheticTrackingTruthTableEpoch {
                        epoch_index,
                        sample_index: epoch.sample_index,
                        expected_carrier_hz,
                        measured_carrier_hz,
                        carrier_error_hz,
                        expected_doppler_hz: expected_measured_doppler_hz,
                        measured_doppler_hz,
                        doppler_error_hz,
                        expected_code_phase_samples,
                        measured_code_phase_samples,
                        code_phase_error_samples,
                        expected_cn0_db_hz: sat_truth.cn0_db_hz as f64,
                        measured_cn0_dbhz,
                        cn0_error_db,
                        lock: epoch.lock,
                        pll_lock: epoch.pll_lock,
                        dll_lock: epoch.dll_lock,
                        fll_lock: epoch.fll_lock,
                        cycle_slip: epoch.cycle_slip,
                        lock_state: epoch.lock_state.clone(),
                        lock_state_reason: epoch.lock_state_reason.clone(),
                        stable_tracking_epoch,
                        pass,
                    }
                })
                .collect::<Vec<_>>();
            let stable_epoch_count =
                epoch_rows.iter().filter(|row| row.stable_tracking_epoch).count();
            let first_stable_epoch_index =
                epoch_rows.iter().find(|row| row.stable_tracking_epoch).map(|row| row.epoch_index);
            let pass = stable_epoch_count > 0
                && epoch_rows.iter().filter(|row| row.stable_tracking_epoch).all(|row| row.pass);

            SyntheticTrackingTruthTableSatellite {
                sat: sat_truth.sat,
                injected_doppler_hz: sat_truth.doppler_hz,
                expected_measured_doppler_hz,
                injected_code_phase_chips: sat_truth.code_phase_chips,
                injected_cn0_db_hz: sat_truth.cn0_db_hz,
                epoch_count: epoch_rows.len(),
                stable_epoch_count,
                first_stable_epoch_index,
                pass,
                epochs: epoch_rows,
            }
        })
        .collect::<Vec<_>>();
    let pass = !satellites.is_empty() && satellites.iter().all(|row| row.pass);

    SyntheticTrackingTruthTableReport {
        scenario_id: truth.scenario_id.clone(),
        carrier_tolerance_hz,
        doppler_tolerance_hz,
        code_phase_tolerance_samples,
        cn0_tolerance_db_hz,
        sample_rate_hz: truth.sample_rate_hz,
        period_samples,
        output_scale_applied: truth.output_scale_applied,
        pass,
        satellites,
    }
}
