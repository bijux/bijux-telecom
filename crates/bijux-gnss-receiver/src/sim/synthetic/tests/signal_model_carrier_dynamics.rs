#[test]
fn zero_doppler_rate_matches_constant_synthetic_generator() {
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
        sat: SatId { constellation: Constellation::Gps, prn: 9 },
        glonass_frequency_channel: None,
        signal_band: bijux_gnss_core::api::SignalBand::L1,
        signal_code: bijux_gnss_core::api::SignalCode::Unknown,
        doppler_hz: 750.0,
        code_phase_chips: 144.375,
        carrier_phase_rad: 0.15,
        cn0_db_hz: 58.0,
        navigation_data: false.into(),
    };
    let baseline = generate_l1_ca(&config, params.clone(), 0xD0A0_0001, 0.020);
    let ramped = generate_l1_ca_with_doppler_ramp(
        &config,
        SyntheticDopplerRampParams { signal: params.clone(), doppler_rate_hz_per_s: 0.0 },
        0xD0A0_0001,
        0.020,
    );

    assert_eq!(ramped.t0.sample_index, baseline.t0.sample_index);
    assert_eq!(ramped.t0.sample_rate_hz, baseline.t0.sample_rate_hz);
    assert_eq!(ramped.dt_s.0, baseline.dt_s.0);
    assert_eq!(ramped.iq, baseline.iq);
}

#[test]
fn zero_jerk_carrier_dynamics_matches_doppler_ramp_generator() {
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
        sat: SatId { constellation: Constellation::Gps, prn: 12 },
        glonass_frequency_channel: None,
        signal_band: bijux_gnss_core::api::SignalBand::L1,
        signal_code: bijux_gnss_core::api::SignalCode::Unknown,
        doppler_hz: 650.0,
        code_phase_chips: 211.25,
        carrier_phase_rad: 0.30,
        cn0_db_hz: 58.0,
        navigation_data: false.into(),
    };
    let ramped = generate_l1_ca_with_doppler_ramp(
        &config,
        SyntheticDopplerRampParams { signal: params.clone(), doppler_rate_hz_per_s: 45.0 },
        0xCA44_1000,
        0.020,
    );
    let dynamics = generate_l1_ca_with_carrier_dynamics(
        &config,
        SyntheticCarrierDynamicsParams {
            signal: params,
            doppler_rate_hz_per_s: 45.0,
            doppler_jerk_hz_per_s2: 0.0,
        },
        0xCA44_1000,
        0.020,
    );

    assert_eq!(dynamics.t0.sample_index, ramped.t0.sample_index);
    assert_eq!(dynamics.t0.sample_rate_hz, ramped.t0.sample_rate_hz);
    assert_eq!(dynamics.dt_s.0, ramped.dt_s.0);
    assert_eq!(dynamics.iq, ramped.iq);
}

#[test]
fn doppler_ramp_updates_instantaneous_carrier_frequency_linearly() {
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
        sat: SatId { constellation: Constellation::Gps, prn: 14 },
        glonass_frequency_channel: None,
        signal_band: bijux_gnss_core::api::SignalBand::L1,
        signal_code: bijux_gnss_core::api::SignalCode::Unknown,
        doppler_hz: 1_200.0,
        code_phase_chips: 0.0,
        carrier_phase_rad: 0.0,
        cn0_db_hz: 60.0,
        navigation_data: false.into(),
    };
    let sat_state = SatState::new_with_doppler_rate_and_receiver_clock_frequency_bias_hz(
        &config, params, 150.0, 40.0,
    );

    assert!((sat_state.carrier_hz_at(0.0) - 1_350.0).abs() <= 1e-12);
    assert!((sat_state.carrier_hz_at(0.25) - 1_360.0).abs() <= 1e-12);
    assert!((sat_state.carrier_hz_at(0.50) - 1_370.0).abs() <= 1e-12);
}

#[test]
fn carrier_dynamics_updates_instantaneous_carrier_frequency_quadratically() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let params = SyntheticSignalParams {
        sat: SatId { constellation: Constellation::Gps, prn: 15 },
        glonass_frequency_channel: None,
        signal_band: bijux_gnss_core::api::SignalBand::L1,
        signal_code: bijux_gnss_core::api::SignalCode::Unknown,
        doppler_hz: 1_200.0,
        code_phase_chips: 0.0,
        carrier_phase_rad: 0.0,
        cn0_db_hz: 60.0,
        navigation_data: false.into(),
    };
    let sat_state = SatState::new_with_carrier_dynamics_and_receiver_oscillator(
        &config,
        params,
        SyntheticReceiverOscillatorModel {
            carrier_frequency_bias_hz: 150.0,
            carrier_frequency_drift_hz_per_s: 0.0,
            sampling_clock_fractional_error: 0.0,
            sampling_clock_fractional_drift_per_s: 0.0,
            phase_noise: SyntheticReceiverPhaseNoiseModel::default(),
            noise: SyntheticReceiverOscillatorNoiseModel::default(),
        },
        40.0,
        120.0,
        4_092,
    );

    assert!((sat_state.carrier_hz_at(0.0) - 1_350.0).abs() <= 1.0e-12);
    assert!((sat_state.carrier_hz_at(0.25) - 1_363.75).abs() <= 1.0e-12);
    assert!((sat_state.carrier_hz_at(0.50) - 1_385.0).abs() <= 1.0e-12);
}

#[test]
fn receiver_oscillator_drift_accumulates_separately_from_signal_doppler_rate() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let params = SyntheticSignalParams {
        sat: SatId { constellation: Constellation::Gps, prn: 18 },
        glonass_frequency_channel: None,
        signal_band: bijux_gnss_core::api::SignalBand::L1,
        signal_code: bijux_gnss_core::api::SignalCode::Unknown,
        doppler_hz: 1_200.0,
        code_phase_chips: 0.0,
        carrier_phase_rad: 0.0,
        cn0_db_hz: 60.0,
        navigation_data: false.into(),
    };
    let sat_state = SatState::new_with_doppler_rate_and_receiver_oscillator(
        &config,
        params,
        SyntheticReceiverOscillatorModel {
            carrier_frequency_bias_hz: 150.0,
            carrier_frequency_drift_hz_per_s: 25.0,
            sampling_clock_fractional_error: 0.0,
            sampling_clock_fractional_drift_per_s: 0.0,
            phase_noise: SyntheticReceiverPhaseNoiseModel::default(),
            noise: SyntheticReceiverOscillatorNoiseModel::default(),
        },
        40.0,
        4_092,
    );

    assert!((sat_state.carrier_hz_at(0.0) - 1_350.0).abs() <= 1.0e-12);
    assert!((sat_state.carrier_hz_at(0.50) - 1_382.5).abs() <= 1.0e-12);
}

#[test]
fn doppler_ramp_integrates_into_carrier_phase_quadratically() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let params = SyntheticSignalParams {
        sat: SatId { constellation: Constellation::Gps, prn: 21 },
        glonass_frequency_channel: None,
        signal_band: bijux_gnss_core::api::SignalBand::L1,
        signal_code: bijux_gnss_core::api::SignalCode::Unknown,
        doppler_hz: 900.0,
        code_phase_chips: 0.0,
        carrier_phase_rad: 0.35,
        cn0_db_hz: 60.0,
        navigation_data: false.into(),
    };
    let sat_state = SatState::new_with_doppler_rate_and_receiver_clock_frequency_bias_hz(
        &config, params, 100.0, -25.0,
    );
    let t_s: f64 = 0.40;
    let expected_phase_rad =
        0.35 + std::f64::consts::TAU * ((1_000.0 * t_s) + 0.5 * (-25.0) * t_s * t_s);

    assert!(
        (sat_state.carrier_phase_rad_at(t_s) - expected_phase_rad).abs() <= 1e-9,
        "carrier phase ramp mismatch: actual={}, expected={expected_phase_rad}",
        sat_state.carrier_phase_rad_at(t_s)
    );
}

#[test]
fn carrier_dynamics_integrates_into_carrier_phase_cubically() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let params = SyntheticSignalParams {
        sat: SatId { constellation: Constellation::Gps, prn: 21 },
        glonass_frequency_channel: None,
        signal_band: bijux_gnss_core::api::SignalBand::L1,
        signal_code: bijux_gnss_core::api::SignalCode::Unknown,
        doppler_hz: 900.0,
        code_phase_chips: 0.0,
        carrier_phase_rad: 0.35,
        cn0_db_hz: 60.0,
        navigation_data: false.into(),
    };
    let sat_state = SatState::new_with_carrier_dynamics_and_receiver_oscillator(
        &config,
        params,
        SyntheticReceiverOscillatorModel {
            carrier_frequency_bias_hz: 100.0,
            carrier_frequency_drift_hz_per_s: 0.0,
            sampling_clock_fractional_error: 0.0,
            sampling_clock_fractional_drift_per_s: 0.0,
            phase_noise: SyntheticReceiverPhaseNoiseModel::default(),
            noise: SyntheticReceiverOscillatorNoiseModel::default(),
        },
        -25.0,
        90.0,
        4_092,
    );
    let t_s: f64 = 0.40;
    let expected_phase_rad = 0.35
        + std::f64::consts::TAU
            * ((1_000.0 * t_s) + 0.5 * (-25.0) * t_s * t_s + 90.0 * t_s.powi(3) / 6.0);

    assert!(
        (sat_state.carrier_phase_rad_at(t_s) - expected_phase_rad).abs() <= 1e-9,
        "carrier dynamics phase mismatch: actual={}, expected={expected_phase_rad}",
        sat_state.carrier_phase_rad_at(t_s)
    );
}
