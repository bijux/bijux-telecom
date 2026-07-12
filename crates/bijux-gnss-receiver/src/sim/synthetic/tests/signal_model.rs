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
            doppler_hz: 0.0,
            code_phase_chips: 200.375,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 58.0,
            data_bit_flip: false,
        };
        let sat_state = SatState::new_with_receiver_clock_frequency_bias_hz(&config, params, 0.0);
        let code_period_samples = samples_per_code(
            config.sampling_freq_hz,
            config.code_freq_basis_hz,
            config.code_length,
        );
        let sixty_second_sample_index = (60.0 * config.sampling_freq_hz) as u64;
        let frame = synthetic_epoch_frame(
            &sat_state,
            &config,
            sixty_second_sample_index,
            code_period_samples,
        );

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

        for (index, (sample, expected_chip)) in
            frame.iq.iter().zip(expected_code.iter()).enumerate()
        {
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
            doppler_hz: 0.0,
            code_phase_chips: 0.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 58.0,
            data_bit_flip: false,
        };
        let frame = super::generate_l1_ca_signal_only(&config, params, 20.0 / config.sampling_freq_hz);
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
            doppler_hz: 0.0,
            code_phase_chips: 0.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 58.0,
            data_bit_flip: false,
        };
        let period_samples = (config.sampling_freq_hz * 0.004).round() as usize;
        let frame =
            super::generate_l1_ca_signal_only(&config, params, 2.0 * period_samples as f64 / config.sampling_freq_hz);
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
    fn glonass_l1_signal_only_matches_reference_samples() {
        let channel =
            bijux_gnss_core::api::GlonassFrequencyChannel::new(-4).expect("channel -4 must be valid");
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 2_044_000.0,
            intermediate_freq_hz: bijux_gnss_core::api::GPS_L1_CA_CARRIER_HZ.value()
                - bijux_gnss_core::api::glonass_l1_carrier_hz(channel).value(),
            code_freq_basis_hz: 511_000.0,
            code_length: 511,
            ..ReceiverPipelineConfig::default()
        };
        let params = SyntheticSignalParams {
            sat: SatId { constellation: Constellation::Glonass, prn: 8 },
            glonass_frequency_channel: Some(channel),
            doppler_hz: 0.0,
            code_phase_chips: 0.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 58.0,
            data_bit_flip: false,
        };
        let frame = super::generate_l1_ca_signal_only(&config, params, 20.0 / config.sampling_freq_hz);
        let expected = bijux_gnss_signal::api::sample_glonass_l1_st_code(
            config.sampling_freq_hz,
            0.0,
            20,
        )
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
            SampleTime {
                sample_index: start_sample_index,
                sample_rate_hz: config.sampling_freq_hz,
            },
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

    #[test]
    fn truth_bundle_records_constant_and_alternating_nav_bit_truth() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_000_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            ..ReceiverPipelineConfig::default()
        };
        let scenario = SyntheticScenario {
            sample_rate_hz: config.sampling_freq_hz,
            intermediate_freq_hz: config.intermediate_freq_hz,
            receiver_clock_frequency_bias_hz: 250.0,
            duration_s: 0.05,
            seed: 44,
            satellites: vec![
                SyntheticSignalParams {
                    sat: SatId { constellation: Constellation::Gps, prn: 3 },
                    glonass_frequency_channel: None,
                    doppler_hz: 500.0,
                    code_phase_chips: 200.0,
                    carrier_phase_rad: 0.0,
                    cn0_db_hz: 50.0,
                    data_bit_flip: false,
                },
                SyntheticSignalParams {
                    sat: SatId { constellation: Constellation::Gps, prn: 7 },
                    glonass_frequency_channel: None,
                    doppler_hz: -750.0,
                    code_phase_chips: 321.0,
                    carrier_phase_rad: 0.2,
                    cn0_db_hz: 45.0,
                    data_bit_flip: true,
                },
            ],
            ephemerides: Vec::new(),
            id: "truth-bundle".to_string(),
        };
        let frame = generate_l1_ca_multi(&config, &scenario);
        let metadata = RawIqMetadata {
            format: IqSampleFormat::Iq16Le,
            sample_rate_hz: config.sampling_freq_hz,
            intermediate_freq_hz: config.intermediate_freq_hz,
            capture_start_utc: "2026-07-09T00:00:00Z".to_string(),
            offset_bytes: 0,
            quantization_bits: Some(16),
            notes: Some("synthetic truth bundle".to_string()),
        };

        let truth = build_truth_bundle(&scenario.id, &scenario, &frame, &metadata, 1.25, 0.8);

        assert_eq!(truth.schema_version, 4);
        assert_eq!(truth.scenario_id, "truth-bundle");
        assert_eq!(truth.seed, 44);
        assert_eq!(truth.sample_format, IqSampleFormat::Iq16Le);
        assert_eq!(truth.sample_rate_hz, 4_000_000.0);
        assert_eq!(truth.receiver_clock_frequency_bias_hz, 250.0);
        assert_eq!(truth.quantization_bits, 16);
        assert_eq!(truth.noise_std_per_component, SYNTHETIC_NOISE_STD_PER_COMPONENT);
        assert_eq!(truth.noise_power_per_complex_sample, SYNTHETIC_COMPLEX_NOISE_POWER as f32);
        assert_eq!(truth.peak_component_before_scaling, 1.25);
        assert_eq!(truth.output_scale_applied, 0.8);
        assert_eq!(truth.satellites.len(), 2);

        let constant = &truth.satellites[0];
        assert_eq!(
            constant.signal_amplitude,
            signal_amplitude_from_cn0(constant.cn0_db_hz, truth.sample_rate_hz)
        );
        assert_eq!(constant.glonass_frequency_channel, None);
        assert_eq!(constant.nav_bit_mode, SyntheticNavBitMode::ConstantPositive);
        assert_eq!(constant.nav_bit_segments.len(), 1);
        assert_eq!(constant.nav_bit_segments[0].start_sample, 0);
        assert_eq!(constant.nav_bit_segments[0].end_sample, frame.len() as u64);
        assert_eq!(constant.nav_bit_segments[0].bit, 1);

        let alternating = &truth.satellites[1];
        assert_eq!(
            alternating.signal_amplitude,
            signal_amplitude_from_cn0(alternating.cn0_db_hz, truth.sample_rate_hz)
        );
        assert!(constant.signal_amplitude > alternating.signal_amplitude);
        assert_eq!(alternating.glonass_frequency_channel, None);
        assert_eq!(alternating.nav_bit_mode, SyntheticNavBitMode::AlternatingGpsLnav20ms);
        assert_eq!(alternating.nav_bit_segments.len(), 3);
        assert_eq!(alternating.nav_bit_segments[0].start_sample, 0);
        assert_eq!(alternating.nav_bit_segments[0].end_sample, 80_000);
        assert_eq!(alternating.nav_bit_segments[0].bit, 1);
        assert_eq!(alternating.nav_bit_segments[1].start_sample, 80_000);
        assert_eq!(alternating.nav_bit_segments[1].end_sample, 160_000);
        assert_eq!(alternating.nav_bit_segments[1].bit, -1);
        assert_eq!(alternating.nav_bit_segments[2].start_sample, 160_000);
        assert_eq!(alternating.nav_bit_segments[2].end_sample, frame.len() as u64);
        assert_eq!(alternating.nav_bit_segments[2].bit, 1);
    }

    #[test]
    fn truth_bundle_preserves_glonass_frequency_channel_metadata() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_000_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 511_000.0,
            code_length: 511,
            ..ReceiverPipelineConfig::default()
        };
        let channel =
            bijux_gnss_core::api::GlonassFrequencyChannel::new(-4).expect("channel -4 must be valid");
        let scenario = SyntheticScenario {
            sample_rate_hz: config.sampling_freq_hz,
            intermediate_freq_hz: config.intermediate_freq_hz,
            receiver_clock_frequency_bias_hz: 0.0,
            duration_s: 0.01,
            seed: 7,
            satellites: vec![SyntheticSignalParams {
                sat: SatId { constellation: Constellation::Glonass, prn: 8 },
                glonass_frequency_channel: Some(channel),
                doppler_hz: 250.0,
                code_phase_chips: 10.0,
                carrier_phase_rad: 0.0,
                cn0_db_hz: 48.0,
                data_bit_flip: false,
            }],
            ephemerides: Vec::new(),
            id: "glonass-truth".to_string(),
        };
        let frame = generate_l1_ca_multi(&config, &scenario);
        let metadata = RawIqMetadata {
            format: IqSampleFormat::Iq16Le,
            sample_rate_hz: config.sampling_freq_hz,
            intermediate_freq_hz: config.intermediate_freq_hz,
            capture_start_utc: "2026-07-09T00:00:00Z".to_string(),
            offset_bytes: 0,
            quantization_bits: Some(16),
            notes: Some("glonass truth bundle".to_string()),
        };

        let truth = build_truth_bundle(&scenario.id, &scenario, &frame, &metadata, 1.0, 0.75);

        assert_eq!(truth.schema_version, 4);
        assert_eq!(truth.satellites.len(), 1);
        assert_eq!(truth.satellites[0].sat, scenario.satellites[0].sat);
        assert_eq!(truth.satellites[0].glonass_frequency_channel, Some(channel));
    }

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
                doppler_hz: 0.0,
                code_phase_chips: 0.0,
                carrier_phase_rad: 0.0,
                cn0_db_hz: 48.0,
                data_bit_flip: false,
            },
            0.0,
        );
        let upper = SatState::new_with_receiver_clock_frequency_bias_hz(
            &config,
            SyntheticSignalParams {
                sat: SatId { constellation: Constellation::Glonass, prn: 12 },
                glonass_frequency_channel: Some(upper_channel),
                doppler_hz: 0.0,
                code_phase_chips: 0.0,
                carrier_phase_rad: 0.0,
                cn0_db_hz: 48.0,
                data_bit_flip: false,
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
            doppler_hz: 1_000.0,
            code_phase_chips: 0.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 60.0,
            data_bit_flip: false,
        };
        let unbiased = SatState::new_with_receiver_clock_frequency_bias_hz(&config, params, 0.0);
        let biased = SatState::new_with_receiver_clock_frequency_bias_hz(&config, params, 500.0);
        let sample_dt_s = 1.0 / config.sampling_freq_hz;
        let unbiased_phase_step =
            phase_step_rad(unbiased.sample_at(0.0), unbiased.sample_at(sample_dt_s));
        let biased_phase_step =
            phase_step_rad(biased.sample_at(0.0), biased.sample_at(sample_dt_s));
        let expected_extra_phase_step = std::f64::consts::TAU * 500.0 / config.sampling_freq_hz;
        let actual_extra_phase_step =
            wrap_phase_rad((biased_phase_step - unbiased_phase_step) as f64);

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
            doppler_hz: 0.0,
            code_phase_chips: 0.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 60.0,
            data_bit_flip: false,
        };
        let mut signal_only = super::generate_l1_ca_signal_only(&config, params, 0.010);
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
            signal_only.iq[fade_start..fade_end]
                .iter()
                .all(|sample| sample.norm_sqr() <= f32::EPSILON),
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
            doppler_hz: 0.0,
            code_phase_chips: 0.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 60.0,
            data_bit_flip: false,
        };
        let baseline = generate_l1_ca(&config, params, 0xFADE_0001, 0.010);
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
            doppler_hz: 0.0,
            code_phase_chips: 0.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 60.0,
            data_bit_flip: false,
        };
        let mut signal_only = super::generate_l1_ca_signal_only(&config, params, 0.010);
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
            doppler_hz: 0.0,
            code_phase_chips: 0.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 60.0,
            data_bit_flip: false,
        };
        let baseline = generate_l1_ca(&config, params, 0xFACE_0001, 0.010);
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
            doppler_hz: 750.0,
            code_phase_chips: 144.375,
            carrier_phase_rad: 0.15,
            cn0_db_hz: 58.0,
            data_bit_flip: false,
        };
        let baseline = generate_l1_ca(&config, params, 0xD0A0_0001, 0.020);
        let ramped = generate_l1_ca_with_doppler_ramp(
            &config,
            SyntheticDopplerRampParams { signal: params, doppler_rate_hz_per_s: 0.0 },
            0xD0A0_0001,
            0.020,
        );

        assert_eq!(ramped.t0.sample_index, baseline.t0.sample_index);
        assert_eq!(ramped.t0.sample_rate_hz, baseline.t0.sample_rate_hz);
        assert_eq!(ramped.dt_s.0, baseline.dt_s.0);
        assert_eq!(ramped.iq, baseline.iq);
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
            doppler_hz: 1_200.0,
            code_phase_chips: 0.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 60.0,
            data_bit_flip: false,
        };
        let sat_state = SatState::new_with_doppler_rate_and_receiver_clock_frequency_bias_hz(
            &config, params, 150.0, 40.0,
        );

        assert!((sat_state.carrier_hz_at(0.0) - 1_350.0).abs() <= 1e-12);
        assert!((sat_state.carrier_hz_at(0.25) - 1_360.0).abs() <= 1e-12);
        assert!((sat_state.carrier_hz_at(0.50) - 1_370.0).abs() <= 1e-12);
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
            doppler_hz: 900.0,
            code_phase_chips: 0.0,
            carrier_phase_rad: 0.35,
            cn0_db_hz: 60.0,
            data_bit_flip: false,
        };
        let sat_state = SatState::new_with_doppler_rate_and_receiver_clock_frequency_bias_hz(
            &config, params, 100.0, -25.0,
        );
        let t_s = 0.40;
        let expected_phase_rad =
            0.35 + std::f64::consts::TAU * ((1_000.0 * t_s) + 0.5 * (-25.0) * t_s * t_s);

        assert!(
            (sat_state.carrier_phase_rad_at(t_s) - expected_phase_rad).abs() <= 1e-9,
            "carrier phase ramp mismatch: actual={}, expected={expected_phase_rad}",
            sat_state.carrier_phase_rad_at(t_s)
        );
    }

    #[test]
    fn alternating_nav_bit_sign_flips_on_twenty_millisecond_boundaries() {
        assert_eq!(nav_bit_index_at_time_s(-1.0), 0);
        assert_eq!(nav_bit_index_at_time_s(0.0), 0);
        assert_eq!(nav_bit_index_at_time_s(0.019_999_999), 0);
        assert_eq!(nav_bit_index_at_time_s(0.020_000_000), 1);
        assert_eq!(nav_bit_index_at_time_s(0.039_999_999), 1);
        assert_eq!(nav_bit_index_at_time_s(0.040_000_000), 2);

        assert_eq!(nav_bit_sign_at_time_s(false, 0.0), 1);
        assert_eq!(nav_bit_sign_at_time_s(true, 0.0), 1);
        assert_eq!(nav_bit_sign_at_time_s(true, 0.019_999_999), 1);
        assert_eq!(nav_bit_sign_at_time_s(true, 0.020_000_000), -1);
        assert_eq!(nav_bit_sign_at_time_s(true, 0.039_999_999), -1);
        assert_eq!(nav_bit_sign_at_time_s(true, 0.040_000_000), 1);
    }

    #[test]
    fn iq16_capture_bundle_scales_without_clipping_and_preserves_truth() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            ..ReceiverPipelineConfig::default()
        };
        let scenario = SyntheticScenario {
            sample_rate_hz: config.sampling_freq_hz,
            intermediate_freq_hz: config.intermediate_freq_hz,
            receiver_clock_frequency_bias_hz: 0.0,
            duration_s: 0.01,
            seed: 91,
            satellites: vec![
                SyntheticSignalParams {
                    sat: SatId { constellation: Constellation::Gps, prn: 3 },
                    glonass_frequency_channel: None,
                    doppler_hz: 500.0,
                    code_phase_chips: 200.0,
                    carrier_phase_rad: 0.0,
                    cn0_db_hz: 58.0,
                    data_bit_flip: true,
                },
                SyntheticSignalParams {
                    sat: SatId { constellation: Constellation::Gps, prn: 7 },
                    glonass_frequency_channel: None,
                    doppler_hz: -1000.0,
                    code_phase_chips: 321.0,
                    carrier_phase_rad: 0.2,
                    cn0_db_hz: 56.0,
                    data_bit_flip: false,
                },
            ],
            ephemerides: Vec::new(),
            id: "iq16-bundle".to_string(),
        };
        let frame = generate_l1_ca_multi(&config, &scenario);

        let bundle = build_iq16_capture_bundle(
            &scenario.id,
            &scenario,
            &frame,
            "2026-07-09T00:00:00Z",
            Some("synthetic iq bundle".to_string()),
        );

        assert_eq!(bundle.metadata.format, IqSampleFormat::Iq16Le);
        assert_eq!(bundle.metadata.quantization_bits, Some(16));
        assert_eq!(bundle.truth.scenario_id, "iq16-bundle");
        assert_eq!(bundle.truth.sample_count, frame.len());
        assert_eq!(bundle.truth.sample_rate_hz, 4_092_000.0);
        assert_eq!(bundle.truth.noise_std_per_component, SYNTHETIC_NOISE_STD_PER_COMPONENT);
        assert_eq!(
            bundle.truth.noise_power_per_complex_sample,
            SYNTHETIC_COMPLEX_NOISE_POWER as f32
        );
        assert_eq!(bundle.raw_iq_bytes.len(), frame.len() * 4);
        assert!(bundle.truth.peak_component_before_scaling > 0.0);
        assert!(bundle.truth.output_scale_applied > 0.0);
        assert!(bundle.truth.output_scale_applied <= 1.0);
        assert!(
            bundle.truth.satellites[0].signal_amplitude
                > bundle.truth.satellites[1].signal_amplitude
        );

        assert!(
            bundle
                .raw_iq_bytes
                .chunks_exact(2)
                .map(|chunk| i16::from_le_bytes([chunk[0], chunk[1]]))
                .any(|sample| sample != 0),
            "encoded synthetic capture should contain non-zero samples"
        );
    }
