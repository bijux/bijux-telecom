fn truth_guided_acquisition_request(
    config: &ReceiverPipelineConfig,
    sat_truth: &SyntheticSatelliteTruth,
) -> crate::api::core::AcqRequest {
    let signal_code = resolved_truth_signal_code(sat_truth.sat, sat_truth.signal_band, sat_truth.signal_code);
    crate::api::core::AcqRequest {
        sat: sat_truth.sat,
        glonass_frequency_channel: sat_truth.glonass_frequency_channel,
        signal_band: sat_truth.signal_band,
        signal_code,
        doppler_center_hz: 0.0,
        doppler_rate_center_hz_per_s: 0.0,
        expected_line_of_sight_doppler_hz: Some(sat_truth.doppler_hz),
        assistance_bounds: None,
        doppler_search_hz: config.acquisition_doppler_search_hz,
        doppler_step_hz: config.acquisition_doppler_step_hz,
        doppler_rate_search_hz_per_s: config.acquisition_doppler_rate_search_hz_per_s,
        doppler_rate_step_hz_per_s: config.acquisition_doppler_rate_step_hz_per_s,
        coherent_ms: config.acquisition_integration_ms,
        noncoherent: config.acquisition_noncoherent,
    }
}

/// Build a truth-guided acquisition table from a synthetic capture.
pub fn validate_truth_guided_acquisition_table(
    config: &ReceiverPipelineConfig,
    frame: &SamplesFrame,
    truth: &SyntheticIqTruthBundle,
    doppler_tolerance_bins: usize,
    code_phase_tolerance_samples: usize,
) -> SyntheticAcquisitionTruthTableReport {
    let period_samples =
        samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length)
            .max(1);
    let doppler_step_hz = config.acquisition_doppler_step_hz.max(1);
    let doppler_tolerance_hz = doppler_tolerance_bins as f64 * doppler_step_hz as f64;
    let satellites = truth
        .satellites
        .iter()
        .map(|sat_truth| {
            let signal_code =
                resolved_truth_signal_code(sat_truth.sat, sat_truth.signal_band, sat_truth.signal_code);
            let isolated_frame = regenerate_isolated_scaled_satellite_signal_only_frame(
                config, frame, truth, sat_truth,
            );
            let acquisition = crate::pipeline::acquisition::Acquisition::new(
                config.clone(),
                crate::engine::runtime::ReceiverRuntime::default(),
            );
            let result = acquisition
                .run_fft_for_requests(
                    &isolated_frame,
                    &[truth_guided_acquisition_request(config, sat_truth)],
                )
                .remove(0);
            let expected_measured_doppler_hz =
                synthetic_truth_measured_doppler_hz(truth, sat_truth);
            let measured_doppler_hz = synthetic_measured_doppler_hz_from_carrier_hz(
                config.intermediate_freq_hz,
                sat_truth.sat,
                sat_truth.signal_band,
                sat_truth.signal_code,
                sat_truth.glonass_frequency_channel,
                result.carrier_hz.0,
            );
            let doppler_error_hz = (measured_doppler_hz - expected_measured_doppler_hz).abs();
            let doppler_error_bins = doppler_error_hz / doppler_step_hz as f64;
            let doppler_pass = measured_doppler_hz.is_finite()
                && doppler_error_hz <= doppler_tolerance_hz + f64::EPSILON;

            let expected_code_phase_samples =
                expected_truth_guided_acquisition_code_phase_samples_f64(
                    config,
                    &isolated_frame,
                    truth,
                    sat_truth.code_phase_chips,
                )
                .round() as usize;
            let measured_code_phase_samples = result.code_phase_samples;
            let code_phase_error_samples = wrapped_code_phase_error_samples(
                measured_code_phase_samples,
                expected_code_phase_samples,
                period_samples,
            );
            let code_phase_pass = code_phase_error_samples <= code_phase_tolerance_samples;

            SyntheticAcquisitionTruthTableSatellite {
                sat: sat_truth.sat,
                glonass_frequency_channel: sat_truth.glonass_frequency_channel,
                signal_band: sat_truth.signal_band,
                signal_code,
                injected_doppler_hz: sat_truth.doppler_hz,
                expected_measured_doppler_hz,
                measured_doppler_hz,
                doppler_error_hz,
                doppler_error_bins,
                injected_code_phase_chips: sat_truth.code_phase_chips,
                expected_code_phase_samples,
                measured_code_phase_samples,
                code_phase_error_samples,
                peak_mean_ratio: result.peak_mean_ratio,
                hypothesis: result.hypothesis.to_string(),
                doppler_pass,
                code_phase_pass,
                pass: doppler_pass && code_phase_pass,
            }
        })
        .collect::<Vec<_>>();
    let pass = !satellites.is_empty() && satellites.iter().all(|row| row.pass);

    SyntheticAcquisitionTruthTableReport {
        scenario_id: truth.scenario_id.clone(),
        doppler_tolerance_bins,
        doppler_tolerance_hz,
        code_phase_tolerance_samples,
        sample_rate_hz: truth.sample_rate_hz,
        period_samples,
        doppler_step_hz,
        pass,
        satellites,
    }
}

fn resolved_truth_signal_code(
    sat: SatId,
    signal_band: SignalBand,
    signal_code: SignalCode,
) -> SignalCode {
    if signal_code != SignalCode::Unknown {
        signal_code
    } else {
        crate::engine::signal_selection::default_signal_code_for_band(sat.constellation, signal_band)
    }
}
