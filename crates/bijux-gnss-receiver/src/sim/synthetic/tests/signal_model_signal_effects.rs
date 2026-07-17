#[test]
fn glonass_frequency_channel_offsets_synthetic_intermediate_frequency() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_000_000.0,
        intermediate_freq_hz: 2_500_000.0,
        code_freq_basis_hz: 511_000.0,
        code_length: 511,
        ..ReceiverPipelineConfig::default()
    };
    let lower_channel =
        bijux_gnss_core::api::GlonassFrequencyChannel::new(-7).expect("channel -7 must be valid");
    let upper_channel =
        bijux_gnss_core::api::GlonassFrequencyChannel::new(6).expect("channel 6 must be valid");
    let lower = SatState::new_with_receiver_clock_frequency_bias_hz(
        &config,
        SyntheticSignalParams {
            sat: SatId { constellation: Constellation::Glonass, prn: 5 },
            glonass_frequency_channel: Some(lower_channel),
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Unknown,
            doppler_hz: 0.0,
            code_phase_chips: 0.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 48.0,
            navigation_data: false.into(),
        },
        0.0,
    );
    let upper = SatState::new_with_receiver_clock_frequency_bias_hz(
        &config,
        SyntheticSignalParams {
            sat: SatId { constellation: Constellation::Glonass, prn: 12 },
            glonass_frequency_channel: Some(upper_channel),
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Unknown,
            doppler_hz: 0.0,
            code_phase_chips: 0.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 48.0,
            navigation_data: false.into(),
        },
        0.0,
    );

    let expected_spacing_hz = 13.0 * bijux_gnss_core::api::GLONASS_L1_CHANNEL_SPACING_HZ.value();

    assert!((upper.if_hz - lower.if_hz - expected_spacing_hz).abs() <= 1e-6);
}

#[test]
fn receiver_clock_frequency_bias_shifts_synthetic_carrier_phase_increment() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 4,
        early_late_spacing_chips: 0.5,
        dll_bw_hz: 2.0,
        pll_bw_hz: 18.0,
        fll_bw_hz: 12.0,
        ..ReceiverPipelineConfig::default()
    };
    let params = SyntheticSignalParams {
        sat: SatId { constellation: Constellation::Gps, prn: 3 },
        glonass_frequency_channel: None,
        signal_band: bijux_gnss_core::api::SignalBand::L1,
        signal_code: bijux_gnss_core::api::SignalCode::Unknown,
        doppler_hz: 1_000.0,
        code_phase_chips: 0.0,
        carrier_phase_rad: 0.0,
        cn0_db_hz: 60.0,
        navigation_data: false.into(),
    };
    let unbiased =
        SatState::new_with_receiver_clock_frequency_bias_hz(&config, params.clone(), 0.0);
    let biased =
        SatState::new_with_receiver_clock_frequency_bias_hz(&config, params.clone(), 500.0);
    let sample_dt_s = 1.0 / config.sampling_freq_hz;
    let unbiased_phase_step =
        phase_step_rad(unbiased.sample_at(0.0), unbiased.sample_at(sample_dt_s));
    let biased_phase_step = phase_step_rad(biased.sample_at(0.0), biased.sample_at(sample_dt_s));
    let expected_extra_phase_step = std::f64::consts::TAU * 500.0 / config.sampling_freq_hz;
    let actual_extra_phase_step = wrap_phase_rad((biased_phase_step - unbiased_phase_step) as f64);

    assert!(
                (actual_extra_phase_step - expected_extra_phase_step).abs() <= 1e-6,
                "receiver clock bias phase step mismatch: actual={actual_extra_phase_step}, expected={expected_extra_phase_step}"
            );
}

#[test]
fn synthetic_fade_windows_zero_signal_inside_the_requested_interval() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 1_023_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let params = SyntheticSignalParams {
        sat: SatId { constellation: Constellation::Gps, prn: 5 },
        glonass_frequency_channel: None,
        signal_band: bijux_gnss_core::api::SignalBand::L1,
        signal_code: bijux_gnss_core::api::SignalCode::Unknown,
        doppler_hz: 0.0,
        code_phase_chips: 0.0,
        carrier_phase_rad: 0.0,
        cn0_db_hz: 60.0,
        navigation_data: false.into(),
    };
    let mut signal_only = super::generate_l1_ca_signal_only(&config, params.clone(), 0.010);
    super::apply_synthetic_fade_windows(
        &mut signal_only,
        &[SyntheticFadeWindow { start_s: 0.002, end_s: 0.004, signal_scale: 0.0 }],
    );

    let fade_start = (0.002 * config.sampling_freq_hz).round() as usize;
    let fade_end = (0.004 * config.sampling_freq_hz).round() as usize;
    assert!(
        signal_only.iq[..fade_start].iter().any(|sample| sample.norm_sqr() > 0.0),
        "samples before the fade must preserve signal energy"
    );
    assert!(
        signal_only.iq[fade_start..fade_end].iter().all(|sample| sample.norm_sqr() <= f32::EPSILON),
        "samples inside the fade window must be fully attenuated"
    );
    assert!(
        signal_only.iq[fade_end..].iter().any(|sample| sample.norm_sqr() > 0.0),
        "samples after the fade must preserve signal energy"
    );
}

#[test]
fn synthetic_fade_generator_preserves_noise_outside_the_attenuated_signal_window() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 1_023_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let params = SyntheticSignalParams {
        sat: SatId { constellation: Constellation::Gps, prn: 5 },
        glonass_frequency_channel: None,
        signal_band: bijux_gnss_core::api::SignalBand::L1,
        signal_code: bijux_gnss_core::api::SignalCode::Unknown,
        doppler_hz: 0.0,
        code_phase_chips: 0.0,
        carrier_phase_rad: 0.0,
        cn0_db_hz: 60.0,
        navigation_data: false.into(),
    };
    let baseline = generate_l1_ca(&config, params.clone(), 0xFADE_0001, 0.010);
    let faded = generate_l1_ca_with_fades(
        &config,
        params,
        &[SyntheticFadeWindow { start_s: 0.002, end_s: 0.004, signal_scale: 0.0 }],
        0xFADE_0001,
        0.010,
    );

    let fade_start = (0.002 * config.sampling_freq_hz).round() as usize;
    let fade_end = (0.004 * config.sampling_freq_hz).round() as usize;
    assert_eq!(baseline.iq[..fade_start], faded.iq[..fade_start]);
    assert_eq!(baseline.iq[fade_end..], faded.iq[fade_end..]);
    assert!(
        baseline.iq[fade_start..fade_end] != faded.iq[fade_start..fade_end],
        "the attenuated window must differ from the unfaded baseline"
    );
}

#[test]
fn synthetic_phase_window_rotates_signal_only_inside_configured_window() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 1_023_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let params = SyntheticSignalParams {
        sat: SatId { constellation: Constellation::Gps, prn: 5 },
        glonass_frequency_channel: None,
        signal_band: bijux_gnss_core::api::SignalBand::L1,
        signal_code: bijux_gnss_core::api::SignalCode::Unknown,
        doppler_hz: 0.0,
        code_phase_chips: 0.0,
        carrier_phase_rad: 0.0,
        cn0_db_hz: 60.0,
        navigation_data: false.into(),
    };
    let mut signal_only = super::generate_l1_ca_signal_only(&config, params.clone(), 0.010);
    super::apply_synthetic_phase_windows(
        &mut signal_only,
        &[SyntheticPhaseWindow {
            start_s: 0.002,
            end_s: 0.004,
            phase_offset_rad: std::f64::consts::FRAC_PI_2,
        }],
    );

    let phase_start = (0.002 * config.sampling_freq_hz).round() as usize;
    let phase_end = (0.004 * config.sampling_freq_hz).round() as usize;
    assert!(signal_only.iq[..phase_start].iter().all(|sample| sample.im.abs() <= 1.0e-6));
    assert!(
        signal_only.iq[phase_start..phase_end].iter().any(|sample| sample.im.abs() > 1.0e-3),
        "windowed samples must carry a rotated quadrature component"
    );
    assert!(signal_only.iq[phase_end..].iter().all(|sample| sample.im.abs() <= 1.0e-6));
}

#[test]
fn synthetic_phase_generator_preserves_noise_outside_the_rotated_window() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 1_023_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let params = SyntheticSignalParams {
        sat: SatId { constellation: Constellation::Gps, prn: 5 },
        glonass_frequency_channel: None,
        signal_band: bijux_gnss_core::api::SignalBand::L1,
        signal_code: bijux_gnss_core::api::SignalCode::Unknown,
        doppler_hz: 0.0,
        code_phase_chips: 0.0,
        carrier_phase_rad: 0.0,
        cn0_db_hz: 60.0,
        navigation_data: false.into(),
    };
    let baseline = generate_l1_ca(&config, params.clone(), 0xFACE_0001, 0.010);
    let rotated = generate_l1_ca_with_phase_windows(
        &config,
        params,
        &[SyntheticPhaseWindow { start_s: 0.002, end_s: 0.004, phase_offset_rad: 0.8 }],
        0xFACE_0001,
        0.010,
    );

    let phase_start = (0.002 * config.sampling_freq_hz).round() as usize;
    let phase_end = (0.004 * config.sampling_freq_hz).round() as usize;
    assert_eq!(baseline.iq[..phase_start], rotated.iq[..phase_start]);
    assert_eq!(baseline.iq[phase_end..], rotated.iq[phase_end..]);
    assert_ne!(baseline.iq[phase_start..phase_end], rotated.iq[phase_start..phase_end]);
}
