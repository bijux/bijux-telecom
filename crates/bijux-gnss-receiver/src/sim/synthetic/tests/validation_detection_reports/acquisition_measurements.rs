    #[test]
    fn truth_guided_cn0_validation_matches_injected_truth_within_tolerance() {
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
        let scenario = SyntheticScenario {
            sample_rate_hz: config.sampling_freq_hz,
            intermediate_freq_hz: config.intermediate_freq_hz,
            receiver_clock_frequency_bias_hz: 0.0,
            duration_s: 0.12,
            seed: 17,
            satellites: vec![SyntheticSignalParams {
                sat: SatId { constellation: Constellation::Gps, prn: 7 },
                glonass_frequency_channel: None,
                signal_band: bijux_gnss_core::api::SignalBand::L1,
                signal_code: bijux_gnss_core::api::SignalCode::Unknown,
                doppler_hz: 0.0,
                code_phase_chips: 0.0,
                carrier_phase_rad: 0.2,
                cn0_db_hz: 58.0,
                navigation_data: false.into(),
            }],
            ephemerides: Vec::new(),
            id: "cn0-validation".to_string(),
        };
        let frame = generate_l1_ca_multi(&config, &scenario);
        let metadata = RawIqMetadata {
            format: IqSampleFormat::Cf32Le,
            sample_rate_hz: config.sampling_freq_hz,
            intermediate_freq_hz: config.intermediate_freq_hz,
            capture_start_utc: "2026-07-09T00:00:00Z".to_string(),
            offset_bytes: 0,
            quantization_bits: Some(32),
            notes: Some("synthetic cn0 validation".to_string()),
        };
        let truth = build_truth_bundle(
            &scenario.id,
            &scenario,
            &frame,
            &metadata,
            IqQuantization::Float32,
            1.0,
            1.0,
        );
        let report = validate_truth_guided_cn0(&config, &frame, &truth, 4.5);

        assert!(report.pass, "{report:?}");
        assert_eq!(report.coherent_samples_per_epoch, 4092);
        assert_eq!(report.satellites.len(), 1);
        let max_tracking_epochs =
            (scenario.duration_s / report.coherent_integration_s).ceil() as usize;
        for row in &report.satellites {
            assert!(row.pass, "{row:?}");
            assert!(row.epochs_measured >= super::TRACKING_CN0_MIN_STABLE_EPOCHS, "{row:?}");
            assert!(row.epochs_measured <= max_tracking_epochs, "{row:?}");
            assert!(row.cn0_delta_db.abs() <= 4.5, "{row:?}");
            assert!(row.measured_max_cn0_dbhz >= row.measured_min_cn0_dbhz);
        }
    }
    #[test]
    fn expected_acquisition_code_phase_uses_receiver_search_convention() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            ..ReceiverPipelineConfig::default()
        };
        let period_samples =
            samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length);
        let frame = SamplesFrame::new(
            SampleTime { sample_index: 0, sample_rate_hz: config.sampling_freq_hz },
            Seconds(1.0 / config.sampling_freq_hz),
            vec![num_complex::Complex::new(0.0f32, 0.0f32); period_samples],
        );

        assert_eq!(expected_acquisition_code_phase_samples(&config, &frame, 0.0), 0);
        assert_eq!(
            expected_acquisition_code_phase_samples(
                &config,
                &frame,
                (period_samples - 1) as f64 * config.code_freq_basis_hz / config.sampling_freq_hz,
            ),
            1
        );
        assert!(
            (expected_acquisition_code_phase_samples_f64(&config, &frame, 0.125) - 4091.5).abs()
                < 1.0e-9
        );
    }

    #[test]
    fn wrapped_code_phase_error_measures_shortest_period_distance() {
        assert_eq!(wrapped_code_phase_error_samples(0, 0, 4092), 0);
        assert_eq!(wrapped_code_phase_error_samples(1, 4091, 4092), 2);
        assert_eq!(wrapped_code_phase_error_samples(4091, 1, 4092), 2);
        assert!((wrapped_code_phase_error_samples_f64(4091.5, 0.0, 4092) - 0.5).abs() < 1.0e-9);
    }

    #[test]
    fn truth_guided_acquisition_code_phase_validation_matches_clean_truth_within_two_samples() {
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
            duration_s: 0.04,
            seed: 24071985,
            satellites: vec![
                SyntheticSignalParams {
                    sat: SatId { constellation: Constellation::Gps, prn: 3 },
                    glonass_frequency_channel: None,
                    signal_band: bijux_gnss_core::api::SignalBand::L1,
 signal_code: bijux_gnss_core::api::SignalCode::Unknown,
                    doppler_hz: 750.0,
                    code_phase_chips: 200.25,
                    carrier_phase_rad: 0.0,
                    cn0_db_hz: 58.0,
                    navigation_data: true.into(),
                },
                SyntheticSignalParams {
                    sat: SatId { constellation: Constellation::Gps, prn: 7 },
                    glonass_frequency_channel: None,
                    signal_band: bijux_gnss_core::api::SignalBand::L1,
 signal_code: bijux_gnss_core::api::SignalCode::Unknown,
                    doppler_hz: -1000.0,
                    code_phase_chips: 321.5,
                    carrier_phase_rad: 0.2,
                    cn0_db_hz: 52.0,
                    navigation_data: false.into(),
                },
            ],
            ephemerides: Vec::new(),
            id: "acquisition-code-phase-validation".to_string(),
        };
        let frame = generate_l1_ca_multi(&config, &scenario);
        let bundle = build_iq16_capture_bundle(
            &scenario.id,
            &scenario,
            &frame,
            "2026-07-09T00:00:00Z",
            Some("synthetic acquisition code-phase validation".to_string()),
        );
        let scaled_frame = SamplesFrame::new(
            frame.t0,
            frame.dt_s,
            frame.iq.iter().map(|sample| *sample * bundle.truth.output_scale_applied).collect(),
        );
        let report =
            validate_truth_guided_acquisition_code_phase(&config, &scaled_frame, &bundle.truth, 2);

        assert!(report.pass, "{report:?}");
        assert_eq!(report.period_samples, 4092);
        assert_eq!(report.satellites.len(), 2);
        for row in &report.satellites {
            assert!(row.pass, "{row:?}");
            assert!(row.code_phase_error_samples <= 2, "{row:?}");
            assert!(matches!(row.hypothesis.as_str(), "accepted" | "ambiguous"));
        }
    }
