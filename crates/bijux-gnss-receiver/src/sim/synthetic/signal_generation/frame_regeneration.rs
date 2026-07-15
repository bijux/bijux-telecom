fn signal_amplitude_from_cn0(cn0_db_hz: f32, sample_rate_hz: f64) -> f32 {
    bijux_gnss_signal::api::signal_amplitude_from_cn0_db_hz(
        cn0_db_hz,
        sample_rate_hz,
        SYNTHETIC_COMPLEX_NOISE_POWER,
    )
}

fn regenerate_isolated_scaled_satellite_signal_only_frame(
    config: &ReceiverPipelineConfig,
    measured_frame: &SamplesFrame,
    truth: &SyntheticIqTruthBundle,
    sat_truth: &SyntheticSatelliteTruth,
) -> SamplesFrame {
    let isolated_frame = generate_l1_ca_multi_signal_only_with_capture_effects(
        config,
        &isolated_satellite_scenario(measured_frame, truth, sat_truth),
        &truth.receiver_oscillator_model,
        truth.source_front_end_filter.as_ref(),
    );
    let iq = isolated_frame
        .iq
        .iter()
        .map(|sample| *sample * truth.output_scale_applied)
        .collect::<Vec<_>>();
    SamplesFrame::new(
        measured_frame.t0,
        measured_frame.dt_s,
        quantize_samples_for_storage(&iq, truth.quantization),
    )
}

fn regenerate_isolated_scaled_satellite_frame_with_noise(
    config: &ReceiverPipelineConfig,
    measured_frame: &SamplesFrame,
    truth: &SyntheticIqTruthBundle,
    sat_truth: &SyntheticSatelliteTruth,
) -> SamplesFrame {
    let isolated_frame = generate_l1_ca_multi_with_capture_effects(
        config,
        &isolated_satellite_scenario(measured_frame, truth, sat_truth),
        &truth.receiver_oscillator_model,
        truth.source_front_end_filter.as_ref(),
    );
    let iq = isolated_frame
        .iq
        .iter()
        .map(|sample| *sample * truth.output_scale_applied)
        .collect::<Vec<_>>();
    SamplesFrame::new(measured_frame.t0, measured_frame.dt_s, iq)
}

fn isolated_satellite_scenario(
    measured_frame: &SamplesFrame,
    truth: &SyntheticIqTruthBundle,
    sat_truth: &SyntheticSatelliteTruth,
) -> SyntheticScenario {
    SyntheticScenario {
        sample_rate_hz: truth.sample_rate_hz,
        intermediate_freq_hz: truth.intermediate_freq_hz,
        receiver_clock_frequency_bias_hz: truth.receiver_clock_frequency_bias_hz,
        duration_s: measured_frame.len() as f64 * measured_frame.dt_s.0,
        seed: truth.seed,
        satellites: vec![SyntheticSignalParams {
            sat: sat_truth.sat,
            glonass_frequency_channel: sat_truth.glonass_frequency_channel,
            signal_band: sat_truth.signal_band,
            signal_code: sat_truth.signal_code,
            doppler_hz: sat_truth.doppler_hz,
            code_phase_chips: sat_truth.code_phase_chips,
            carrier_phase_rad: sat_truth.carrier_phase_rad,
            cn0_db_hz: sat_truth.cn0_db_hz,
            navigation_data: sat_truth.navigation_data.clone(),
        }],
        ephemerides: Vec::new(),
        id: sat_truth.sat.prn.to_string(),
    }
}

#[cfg(test)]
fn code_phase_samples_at_epoch_start(
    config: &ReceiverPipelineConfig,
    frame: &SamplesFrame,
    code_phase_chips: f64,
) -> f64 {
    bijux_gnss_signal::api::code_phase_samples_at_sample_index(
        frame.t0.sample_rate_hz,
        config.code_freq_basis_hz,
        config.code_length,
        frame.t0.sample_index,
        code_phase_chips,
    )
    .expect("synthetic epoch alignment requires a valid code phase model")
}

fn synthetic_intermediate_frequency_hz(
    intermediate_freq_hz: f64,
    sat: SatId,
    signal_band: SignalBand,
    signal_code: bijux_gnss_core::api::SignalCode,
    glonass_frequency_channel: Option<GlonassFrequencyChannel>,
) -> f64 {
    intermediate_freq_hz
        + (synthetic_constellation_carrier_hz(
            sat,
            signal_band,
            signal_code,
            glonass_frequency_channel,
        ) - bijux_gnss_core::api::GPS_L1_CA_CARRIER_HZ.value())
}

fn synthetic_carrier_hz(
    intermediate_freq_hz: f64,
    sat: SatId,
    signal_band: SignalBand,
    signal_code: bijux_gnss_core::api::SignalCode,
    glonass_frequency_channel: Option<GlonassFrequencyChannel>,
    doppler_hz: f64,
) -> f64 {
    carrier_hz_from_doppler_hz(
        synthetic_intermediate_frequency_hz(
            intermediate_freq_hz,
            sat,
            signal_band,
            signal_code,
            glonass_frequency_channel,
        ),
        doppler_hz,
    )
}

fn synthetic_truth_measured_doppler_hz(
    truth: &SyntheticIqTruthBundle,
    sat_truth: &SyntheticSatelliteTruth,
) -> f64 {
    sat_truth.doppler_hz + truth.receiver_clock_frequency_bias_hz
}

fn synthetic_constellation_carrier_hz(
    sat: SatId,
    signal_band: SignalBand,
    signal_code: bijux_gnss_core::api::SignalCode,
    glonass_frequency_channel: Option<GlonassFrequencyChannel>,
) -> f64 {
    match bijux_gnss_signal::api::default_signal_carrier_hz_for_signal(
        sat,
        Some(signal_band),
        resolved_signal_code(sat, signal_band, signal_code),
        glonass_frequency_channel,
    ) {
        Ok(Some(carrier_hz)) => carrier_hz.value(),
        Ok(None) => bijux_gnss_core::api::GPS_L1_CA_CARRIER_HZ.value(),
        Err(bijux_gnss_signal::api::SignalError::MissingGlonassFrequencyChannel(_)) => {
            panic!(
                "GLONASS synthetic signal for {} requires glonass_frequency_channel",
                bijux_gnss_core::api::format_sat(sat)
            )
        }
        Err(_) => bijux_gnss_core::api::GPS_L1_CA_CARRIER_HZ.value(),
    }
}
