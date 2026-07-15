#[test]
fn receiver_oscillator_phase_noise_is_deterministic_and_distinct_from_nominal_carrier() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 1_023_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let scenario = SyntheticScenario {
        sample_rate_hz: config.sampling_freq_hz,
        intermediate_freq_hz: config.intermediate_freq_hz,
        receiver_clock_frequency_bias_hz: 0.0,
        duration_s: 0.004,
        seed: 11,
        satellites: vec![SyntheticSignalParams {
            sat: SatId { constellation: Constellation::Gps, prn: 6 },
            glonass_frequency_channel: None,
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Unknown,
            doppler_hz: 0.0,
            code_phase_chips: 0.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 55.0,
            navigation_data: false.into(),
        }],
        ephemerides: Vec::new(),
        id: "receiver-phase-noise".to_string(),
    };
    let receiver_oscillator = SyntheticReceiverOscillatorModel {
        carrier_frequency_bias_hz: 0.0,
        carrier_frequency_drift_hz_per_s: 0.0,
        sampling_clock_fractional_error: 0.0,
        sampling_clock_fractional_drift_per_s: 0.0,
        phase_noise: SyntheticReceiverPhaseNoiseModel {
            seed: 91,
            knot_interval_samples: 1_023,
            step_std_rad: 0.08,
        },
        noise: SyntheticReceiverOscillatorNoiseModel::default(),
    };

    let phase_noisy_a =
        generate_l1_ca_multi_with_receiver_oscillator(&config, &scenario, &receiver_oscillator);
    let phase_noisy_b =
        generate_l1_ca_multi_with_receiver_oscillator(&config, &scenario, &receiver_oscillator);
    let nominal = generate_l1_ca_multi(&config, &scenario);

    assert_eq!(phase_noisy_a.iq, phase_noisy_b.iq);
    assert_ne!(phase_noisy_a.iq[1_023], nominal.iq[1_023]);
}

#[test]
fn receiver_oscillator_white_phase_noise_is_deterministic_and_distinct_from_nominal_carrier() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 1_023_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let scenario = SyntheticScenario {
        sample_rate_hz: config.sampling_freq_hz,
        intermediate_freq_hz: config.intermediate_freq_hz,
        receiver_clock_frequency_bias_hz: 0.0,
        duration_s: 0.004,
        seed: 11,
        satellites: vec![SyntheticSignalParams {
            sat: SatId { constellation: Constellation::Gps, prn: 6 },
            glonass_frequency_channel: None,
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Unknown,
            doppler_hz: 0.0,
            code_phase_chips: 0.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 55.0,
            navigation_data: false.into(),
        }],
        ephemerides: Vec::new(),
        id: "receiver-white-phase-noise".to_string(),
    };
    let receiver_oscillator = SyntheticReceiverOscillatorModel {
        noise: SyntheticReceiverOscillatorNoiseModel {
            seed: 173,
            update_interval_samples: 1_023,
            white_phase_std_rad: 0.06,
            white_frequency_std_hz: 0.0,
            random_walk_frequency_step_std_hz: 0.0,
        },
        ..SyntheticReceiverOscillatorModel::default()
    };

    let phase_noisy_a =
        generate_l1_ca_multi_with_receiver_oscillator(&config, &scenario, &receiver_oscillator);
    let phase_noisy_b =
        generate_l1_ca_multi_with_receiver_oscillator(&config, &scenario, &receiver_oscillator);
    let nominal = generate_l1_ca_multi(&config, &scenario);

    assert_eq!(phase_noisy_a.iq, phase_noisy_b.iq);
    assert_ne!(phase_noisy_a.iq[1_023], nominal.iq[1_023]);
}

#[test]
fn receiver_oscillator_white_frequency_noise_changes_instantaneous_carrier_truth() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 1_023_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let params = SyntheticSignalParams {
        sat: SatId { constellation: Constellation::Gps, prn: 4 },
        glonass_frequency_channel: None,
        signal_band: bijux_gnss_core::api::SignalBand::L1,
        signal_code: bijux_gnss_core::api::SignalCode::Unknown,
        doppler_hz: 20.0,
        code_phase_chips: 0.0,
        carrier_phase_rad: 0.0,
        cn0_db_hz: 60.0,
        navigation_data: false.into(),
    };
    let receiver_oscillator = SyntheticReceiverOscillatorModel {
        noise: SyntheticReceiverOscillatorNoiseModel {
            seed: 211,
            update_interval_samples: 1_023,
            white_phase_std_rad: 0.0,
            white_frequency_std_hz: 8.0,
            random_walk_frequency_step_std_hz: 0.0,
        },
        ..SyntheticReceiverOscillatorModel::default()
    };
    let noisy = SatState::new_with_receiver_oscillator(
        &config,
        params.clone(),
        receiver_oscillator,
        4_092,
    );
    let nominal = SatState::new_with_receiver_oscillator(
        &config,
        params,
        SyntheticReceiverOscillatorModel::default(),
        4_092,
    );

    assert_ne!(noisy.carrier_hz_at(0.001), nominal.carrier_hz_at(0.001));
    assert!(noisy.carrier_phase_rad_at(0.003).is_finite());
}

#[test]
fn receiver_oscillator_random_walk_frequency_noise_accumulates_carrier_phase() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 1_023_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let params = SyntheticSignalParams {
        sat: SatId { constellation: Constellation::Gps, prn: 10 },
        glonass_frequency_channel: None,
        signal_band: bijux_gnss_core::api::SignalBand::L1,
        signal_code: bijux_gnss_core::api::SignalCode::Unknown,
        doppler_hz: 30.0,
        code_phase_chips: 0.0,
        carrier_phase_rad: 0.0,
        cn0_db_hz: 60.0,
        navigation_data: false.into(),
    };
    let receiver_oscillator = SyntheticReceiverOscillatorModel {
        noise: SyntheticReceiverOscillatorNoiseModel {
            seed: 313,
            update_interval_samples: 1_023,
            white_phase_std_rad: 0.0,
            white_frequency_std_hz: 0.0,
            random_walk_frequency_step_std_hz: 12.0,
        },
        ..SyntheticReceiverOscillatorModel::default()
    };
    let noisy = SatState::new_with_receiver_oscillator(
        &config,
        params.clone(),
        receiver_oscillator,
        6_138,
    );
    let nominal = SatState::new_with_receiver_oscillator(
        &config,
        params,
        SyntheticReceiverOscillatorModel::default(),
        6_138,
    );

    let first_phase_delta = noisy.carrier_phase_rad_at(0.001) - nominal.carrier_phase_rad_at(0.001);
    let later_phase_delta = noisy.carrier_phase_rad_at(0.005) - nominal.carrier_phase_rad_at(0.005);

    assert!(first_phase_delta.abs() <= f64::EPSILON);
    assert!(later_phase_delta.abs() > 1.0e-6);
}

#[test]
fn receiver_oscillator_sampling_clock_error_advances_code_phase_at_expected_rate() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let params = SyntheticSignalParams {
        sat: SatId { constellation: Constellation::Gps, prn: 4 },
        glonass_frequency_channel: None,
        signal_band: bijux_gnss_core::api::SignalBand::L1,
        signal_code: bijux_gnss_core::api::SignalCode::Unknown,
        doppler_hz: 0.0,
        code_phase_chips: 100.0,
        carrier_phase_rad: 0.0,
        cn0_db_hz: 60.0,
        navigation_data: false.into(),
    };
    let nominal = SatState::new_with_receiver_oscillator(
        &config,
        params.clone(),
        SyntheticReceiverOscillatorModel::default(),
        4_092,
    );
    let skewed = SatState::new_with_receiver_oscillator(
        &config,
        params,
        SyntheticReceiverOscillatorModel {
            carrier_frequency_bias_hz: 0.0,
            carrier_frequency_drift_hz_per_s: 0.0,
            sampling_clock_fractional_error: 100.0e-6,
            sampling_clock_fractional_drift_per_s: 0.0,
            phase_noise: SyntheticReceiverPhaseNoiseModel::default(),
            noise: SyntheticReceiverOscillatorNoiseModel::default(),
        },
        4_092,
    );
    let t_s = 0.001;
    let actual_extra_chips = skewed.total_chip_phase_at(t_s) - nominal.total_chip_phase_at(t_s);
    let expected_extra_chips = config.code_freq_basis_hz * t_s * 100.0e-6;

    assert!((actual_extra_chips - expected_extra_chips).abs() <= 1.0e-9);
}

#[test]
fn receiver_oscillator_sampling_clock_drift_accelerates_code_phase_quadratically() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let params = SyntheticSignalParams {
        sat: SatId { constellation: Constellation::Gps, prn: 10 },
        glonass_frequency_channel: None,
        signal_band: bijux_gnss_core::api::SignalBand::L1,
        signal_code: bijux_gnss_core::api::SignalCode::Unknown,
        doppler_hz: 0.0,
        code_phase_chips: 100.0,
        carrier_phase_rad: 0.0,
        cn0_db_hz: 60.0,
        navigation_data: false.into(),
    };
    let nominal = SatState::new_with_receiver_oscillator(
        &config,
        params.clone(),
        SyntheticReceiverOscillatorModel::default(),
        4_092,
    );
    let drifted = SatState::new_with_receiver_oscillator(
        &config,
        params,
        SyntheticReceiverOscillatorModel {
            carrier_frequency_bias_hz: 0.0,
            carrier_frequency_drift_hz_per_s: 0.0,
            sampling_clock_fractional_error: 25.0e-6,
            sampling_clock_fractional_drift_per_s: 60.0e-6,
            phase_noise: SyntheticReceiverPhaseNoiseModel::default(),
            noise: SyntheticReceiverOscillatorNoiseModel::default(),
        },
        4_092,
    );
    let t_s = 0.010;
    let actual_extra_chips = drifted.total_chip_phase_at(t_s) - nominal.total_chip_phase_at(t_s);
    let expected_extra_chips = config.code_freq_basis_hz
        * (t_s * 25.0e-6 + 0.5 * 60.0e-6 * t_s * t_s);

    assert!((actual_extra_chips - expected_extra_chips).abs() <= 1.0e-9);
}
