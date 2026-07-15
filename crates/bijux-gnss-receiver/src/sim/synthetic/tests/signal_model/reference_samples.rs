#[test]
fn synthetic_epoch_start_phase_matches_theoretical_phase_after_sixty_seconds() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_000_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let params = SyntheticSignalParams {
        sat: SatId { constellation: Constellation::Gps, prn: 11 },
        glonass_frequency_channel: None,
        signal_band: bijux_gnss_core::api::SignalBand::L1,
        signal_code: bijux_gnss_core::api::SignalCode::Unknown,
        doppler_hz: 0.0,
        code_phase_chips: 200.375,
        carrier_phase_rad: 0.0,
        cn0_db_hz: 58.0,
        navigation_data: false.into(),
    };
    let sat_state =
        SatState::new_with_receiver_clock_frequency_bias_hz(&config, params.clone(), 0.0);
    let code_period_samples =
        samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length);
    let sixty_second_sample_index = (60.0 * config.sampling_freq_hz) as u64;
    let frame =
        synthetic_epoch_frame(&sat_state, &config, sixty_second_sample_index, code_period_samples);

    let actual_phase_samples =
        super::code_phase_samples_at_epoch_start(&config, &frame, params.code_phase_chips);
    let expected_chip_phase = advance_code_phase_seconds(
        params.code_phase_chips,
        config.code_freq_basis_hz,
        60.0,
        config.code_length,
    )
    .expect("valid theoretical phase");
    let expected_phase_samples =
        expected_chip_phase * config.sampling_freq_hz / config.code_freq_basis_hz;

    assert_phase_samples_close(actual_phase_samples, expected_phase_samples);

    let expected_code = sample_ca_code(
        Prn(params.sat.prn),
        config.sampling_freq_hz,
        config.code_freq_basis_hz,
        expected_chip_phase,
        code_period_samples,
    )
    .expect("valid expected sampled code");
    let amplitude = signal_amplitude_from_cn0(params.cn0_db_hz, config.sampling_freq_hz);

    for (index, (sample, expected_chip)) in frame.iq.iter().zip(expected_code.iter()).enumerate() {
        let expected_value = *expected_chip * amplitude;
        assert!(
            (sample.re - expected_value).abs() <= 1e-6,
            "I component drifted at sample {index}: actual={}, expected={expected_value}",
            sample.re
        );
        assert!(
            sample.im.abs() <= 1e-6,
            "Q component drifted at sample {index}: actual={}",
            sample.im
        );
    }
}

#[test]
fn galileo_e1_signal_only_matches_cboc_reference_samples() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 5_000_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 4092,
        ..ReceiverPipelineConfig::default()
    };
    let params = SyntheticSignalParams {
        sat: SatId { constellation: Constellation::Galileo, prn: 11 },
        glonass_frequency_channel: None,
        signal_band: bijux_gnss_core::api::SignalBand::E1,
        signal_code: bijux_gnss_core::api::SignalCode::E1B,
        doppler_hz: 0.0,
        code_phase_chips: 0.0,
        carrier_phase_rad: 0.0,
        cn0_db_hz: 58.0,
        navigation_data: false.into(),
    };
    let frame =
        super::generate_l1_ca_signal_only(&config, params.clone(), 20.0 / config.sampling_freq_hz);
    let expected = bijux_gnss_signal::api::sample_galileo_e1_cboc(
        params.sat.prn,
        config.sampling_freq_hz,
        0.0,
        20,
        0,
        1,
    )
    .expect("valid Galileo E1 reference samples");
    let amplitude = signal_amplitude_from_cn0(params.cn0_db_hz, config.sampling_freq_hz);

    for (index, (sample, expected_value)) in frame.iq.iter().zip(expected.iter()).enumerate() {
        let scaled = *expected_value * amplitude;
        assert!(
            (sample.re - scaled).abs() <= 1.0e-6,
            "Galileo E1 I mismatch at sample {index}: actual={}, expected={scaled}",
            sample.re
        );
        assert!(
            sample.im.abs() <= 1.0e-6,
            "Galileo E1 Q mismatch at sample {index}: actual={}",
            sample.im
        );
    }
}

#[test]
fn galileo_e1_secondary_code_advances_each_primary_period() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 4092,
        ..ReceiverPipelineConfig::default()
    };
    let params = SyntheticSignalParams {
        sat: SatId { constellation: Constellation::Galileo, prn: 19 },
        glonass_frequency_channel: None,
        signal_band: bijux_gnss_core::api::SignalBand::E1,
        signal_code: bijux_gnss_core::api::SignalCode::E1B,
        doppler_hz: 0.0,
        code_phase_chips: 0.0,
        carrier_phase_rad: 0.0,
        cn0_db_hz: 58.0,
        navigation_data: false.into(),
    };
    let period_samples = (config.sampling_freq_hz * 0.004).round() as usize;
    let frame = super::generate_l1_ca_signal_only(
        &config,
        params.clone(),
        2.0 * period_samples as f64 / config.sampling_freq_hz,
    );
    let amplitude = signal_amplitude_from_cn0(params.cn0_db_hz, config.sampling_freq_hz);
    let first_period = bijux_gnss_signal::api::sample_galileo_e1_cboc(
        params.sat.prn,
        config.sampling_freq_hz,
        0.0,
        1,
        0,
        1,
    )
    .expect("valid first-period reference");
    let second_period = bijux_gnss_signal::api::sample_galileo_e1_cboc(
        params.sat.prn,
        config.sampling_freq_hz,
        0.0,
        1,
        1,
        1,
    )
    .expect("valid second-period reference");

    assert!((frame.iq[0].re - (first_period[0] * amplitude)).abs() <= 1.0e-6);
    assert!((frame.iq[period_samples].re - (second_period[0] * amplitude)).abs() <= 1.0e-6);
}

#[test]
fn beidou_b1i_signal_only_matches_reference_samples() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: bijux_gnss_core::api::GPS_L1_CA_CARRIER_HZ.value()
            - bijux_gnss_core::api::BEIDOU_B1_CARRIER_HZ.value(),
        code_freq_basis_hz: 2_046_000.0,
        code_length: 2046,
        ..ReceiverPipelineConfig::default()
    };
    let params = SyntheticSignalParams {
        sat: SatId { constellation: Constellation::Beidou, prn: 11 },
        glonass_frequency_channel: None,
        signal_band: bijux_gnss_core::api::SignalBand::B1,
        signal_code: bijux_gnss_core::api::SignalCode::B1I,
        doppler_hz: 0.0,
        code_phase_chips: 0.25,
        carrier_phase_rad: 0.0,
        cn0_db_hz: 58.0,
        navigation_data: false.into(),
    };
    let frame =
        super::generate_l1_ca_signal_only(&config, params.clone(), 20.0 / config.sampling_freq_hz);
    let expected = bijux_gnss_signal::api::sample_beidou_b1i_code(
        params.sat.prn,
        config.sampling_freq_hz,
        params.code_phase_chips,
        20,
    )
    .expect("valid BeiDou B1I reference samples");
    let amplitude = signal_amplitude_from_cn0(params.cn0_db_hz, config.sampling_freq_hz);

    for (index, (sample, expected_value)) in frame.iq.iter().zip(expected.iter()).enumerate() {
        let scaled = *expected_value * amplitude;
        assert!(
            (sample.re - scaled).abs() <= 1.0e-6,
            "BeiDou B1I I mismatch at sample {index}: actual={}, expected={scaled}",
            sample.re
        );
        assert!(
            sample.im.abs() <= 1.0e-6,
            "BeiDou B1I Q mismatch at sample {index}: actual={}",
            sample.im
        );
    }
}

#[test]
fn beidou_b1i_signal_only_applies_d1_epoch_modulation() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 2_046_000.0,
        intermediate_freq_hz: bijux_gnss_core::api::GPS_L1_CA_CARRIER_HZ.value()
            - bijux_gnss_core::api::BEIDOU_B1_CARRIER_HZ.value(),
        code_freq_basis_hz: 2_046_000.0,
        code_length: 2046,
        ..ReceiverPipelineConfig::default()
    };
    let params = SyntheticSignalParams {
        sat: SatId { constellation: Constellation::Beidou, prn: 11 },
        glonass_frequency_channel: None,
        signal_band: bijux_gnss_core::api::SignalBand::B1,
        signal_code: bijux_gnss_core::api::SignalCode::B1I,
        doppler_hz: 0.0,
        code_phase_chips: 0.0,
        carrier_phase_rad: 0.0,
        cn0_db_hz: 58.0,
        navigation_data: false.into(),
    };
    let frame = super::generate_l1_ca_signal_only(&config, params.clone(), 0.021);
    let first_chip = bijux_gnss_signal::api::sample_beidou_b1i_code(
        params.sat.prn,
        config.sampling_freq_hz,
        0.0,
        1,
    )
    .expect("valid BeiDou B1I first chip")[0];
    let amplitude = signal_amplitude_from_cn0(params.cn0_db_hz, config.sampling_freq_hz);

    for &epoch_index in &[0usize, 5, 20] {
        let sample_index = epoch_index * 2046;
        let expected = first_chip
            * bijux_gnss_signal::api::beidou_d1_epoch_symbol(&[1], epoch_index)
                .expect("BeiDou D1 epoch symbol") as f32
            * amplitude;
        assert!(
            (frame.iq[sample_index].re - expected).abs() <= 1.0e-6,
            "BeiDou B1I epoch modulation mismatch at epoch {epoch_index}: actual={}, expected={expected}",
            frame.iq[sample_index].re
        );
    }
}

#[test]
fn beidou_b2i_signal_only_applies_d1_epoch_modulation() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 2_046_000.0,
        intermediate_freq_hz: bijux_gnss_core::api::GPS_L1_CA_CARRIER_HZ.value()
            - bijux_gnss_core::api::BEIDOU_B2_CARRIER_HZ.value(),
        code_freq_basis_hz: 2_046_000.0,
        code_length: 2046,
        ..ReceiverPipelineConfig::default()
    };
    let params = SyntheticSignalParams {
        sat: SatId { constellation: Constellation::Beidou, prn: 11 },
        glonass_frequency_channel: None,
        signal_band: bijux_gnss_core::api::SignalBand::B2,
        signal_code: bijux_gnss_core::api::SignalCode::B2I,
        doppler_hz: 0.0,
        code_phase_chips: 0.0,
        carrier_phase_rad: 0.0,
        cn0_db_hz: 58.0,
        navigation_data: false.into(),
    };
    let frame = super::generate_l1_ca_signal_only(&config, params.clone(), 0.021);
    let first_chip = bijux_gnss_signal::api::sample_beidou_b2i_code(
        params.sat.prn,
        config.sampling_freq_hz,
        0.0,
        1,
    )
    .expect("valid BeiDou B2I first chip")[0];
    let amplitude = signal_amplitude_from_cn0(params.cn0_db_hz, config.sampling_freq_hz);

    for &epoch_index in &[0usize, 5, 20] {
        let sample_index = epoch_index * 2046;
        let expected = first_chip
            * bijux_gnss_signal::api::beidou_d1_epoch_symbol(&[1], epoch_index)
                .expect("BeiDou D1 epoch symbol") as f32
            * amplitude;
        assert!(
            (frame.iq[sample_index].re - expected).abs() <= 1.0e-6,
            "BeiDou B2I epoch modulation mismatch at epoch {epoch_index}: actual={}, expected={expected}",
            frame.iq[sample_index].re
        );
    }
}

#[test]
fn glonass_l1_signal_only_matches_reference_samples() {
    let channel =
        bijux_gnss_core::api::GlonassFrequencyChannel::new(-4).expect("channel -4 must be valid");
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 2_044_000.0,
        intermediate_freq_hz: bijux_gnss_core::api::GPS_L1_CA_CARRIER_HZ.value()
            - bijux_gnss_signal::api::glonass_l1_carrier_hz(channel).value(),
        code_freq_basis_hz: 511_000.0,
        code_length: 511,
        ..ReceiverPipelineConfig::default()
    };
    let params = SyntheticSignalParams {
        sat: SatId { constellation: Constellation::Glonass, prn: 8 },
        glonass_frequency_channel: Some(channel),
        signal_band: bijux_gnss_core::api::SignalBand::L1,
        signal_code: bijux_gnss_core::api::SignalCode::Unknown,
        doppler_hz: 0.0,
        code_phase_chips: 0.0,
        carrier_phase_rad: 0.0,
        cn0_db_hz: 58.0,
        navigation_data: false.into(),
    };
    let frame =
        super::generate_l1_ca_signal_only(&config, params.clone(), 20.0 / config.sampling_freq_hz);
    let expected =
        bijux_gnss_signal::api::sample_glonass_l1_st_code(config.sampling_freq_hz, 0.0, 20)
            .expect("valid GLONASS L1 reference samples");
    let amplitude = signal_amplitude_from_cn0(params.cn0_db_hz, config.sampling_freq_hz);

    for (index, (sample, expected_value)) in frame.iq.iter().zip(expected.iter()).enumerate() {
        let scaled = *expected_value * amplitude;
        assert!(
            (sample.re - scaled).abs() <= 1.0e-6,
            "GLONASS L1 I mismatch at sample {index}: actual={}, expected={scaled}",
            sample.re
        );
        assert!(
            sample.im.abs() <= 1.0e-6,
            "GLONASS L1 Q mismatch at sample {index}: actual={}",
            sample.im
        );
    }
}

#[test]
fn glonass_l1_navigation_mode_applies_meander_and_time_mark_schedule() {
    assert_eq!(
        super::nav_bit_sign_for_mode_at_time_s(
            SyntheticNavBitMode::GlonassL1FixedDataString,
            0.000
        ),
        1
    );
    assert_eq!(
        super::nav_bit_sign_for_mode_at_time_s(
            SyntheticNavBitMode::GlonassL1FixedDataString,
            0.010
        ),
        -1
    );
    assert_eq!(
        super::nav_bit_sign_for_mode_at_time_s(
            SyntheticNavBitMode::GlonassL1FixedDataString,
            1.700
        ),
        bijux_gnss_signal::api::GLONASS_L1_TIME_MARK[0]
    );
    assert_eq!(
        super::nav_bit_sign_for_mode_at_time_s(
            SyntheticNavBitMode::GlonassL1FixedDataString,
            1.990
        ),
        bijux_gnss_signal::api::GLONASS_L1_TIME_MARK
            [bijux_gnss_signal::api::GLONASS_L1_TIME_MARK.len() - 1]
    );
    assert_eq!(
        super::nav_bit_sign_for_mode_at_time_s(
            SyntheticNavBitMode::GlonassL1FixedDataString,
            2.000
        ),
        1
    );
}

#[test]
fn glonass_l1_alternating_data_mode_changes_the_second_20ms_data_symbol() {
    assert_eq!(
        super::nav_bit_sign_for_mode_at_time_s(
            SyntheticNavBitMode::GlonassL1AlternatingDataString,
            0.020,
        ),
        -1
    );
    assert_eq!(
        super::nav_bit_sign_for_mode_at_time_s(
            SyntheticNavBitMode::GlonassL1AlternatingDataString,
            0.030,
        ),
        1
    );
}

fn synthetic_epoch_frame(
    sat_state: &SatState,
    config: &ReceiverPipelineConfig,
    start_sample_index: u64,
    sample_count: usize,
) -> SamplesFrame {
    let dt_s = 1.0 / config.sampling_freq_hz;
    let iq = (0..sample_count)
        .map(|offset| sat_state.sample_at((start_sample_index + offset as u64) as f64 * dt_s))
        .collect::<Vec<_>>();
    SamplesFrame::new(
        SampleTime { sample_index: start_sample_index, sample_rate_hz: config.sampling_freq_hz },
        Seconds(dt_s),
        iq,
    )
}

fn assert_phase_samples_close(actual: f64, expected: f64) {
    let delta = (actual - expected).abs();
    assert!(
                delta <= RECEIVER_PHASE_TOLERANCE_SAMPLES,
                "code phase samples drifted at sixty seconds: actual={actual:.12}, expected={expected:.12}, delta={delta:.12}"
            );
}

fn phase_step_rad(left: Complex<f32>, right: Complex<f32>) -> f32 {
    (right * left.conj()).arg()
}

fn wrap_phase_rad(phase_rad: f64) -> f64 {
    let tau = std::f64::consts::TAU;
    (phase_rad + std::f64::consts::PI).rem_euclid(tau) - std::f64::consts::PI
}
