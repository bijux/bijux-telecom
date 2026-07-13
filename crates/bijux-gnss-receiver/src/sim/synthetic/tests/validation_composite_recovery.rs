    #[test]
    fn truth_guided_composite_component_recovery_matches_clean_multiconstellation_truth() {
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
            receiver_clock_frequency_bias_hz: 125.0,
            duration_s: 0.04,
            seed: 0x2640_0001,
            satellites: vec![
                SyntheticSignalParams {
                    sat: SatId { constellation: Constellation::Gps, prn: 3 },
                    glonass_frequency_channel: None,
                    signal_band: SignalBand::L1,
                    signal_code: SignalCode::Ca,
                    doppler_hz: 750.0,
                    code_phase_chips: 200.25,
                    carrier_phase_rad: 0.15,
                    cn0_db_hz: 56.0,
                    navigation_data: SyntheticNavigationData::AlternatingStartPositive,
                },
                SyntheticSignalParams {
                    sat: SatId { constellation: Constellation::Galileo, prn: 11 },
                    glonass_frequency_channel: None,
                    signal_band: SignalBand::E1,
                    signal_code: SignalCode::E1B,
                    doppler_hz: -425.0,
                    code_phase_chips: 418.75,
                    carrier_phase_rad: -0.55,
                    cn0_db_hz: 52.0,
                    navigation_data: SyntheticNavigationData::ConstantNegative,
                },
                SyntheticSignalParams {
                    sat: SatId { constellation: Constellation::Beidou, prn: 14 },
                    glonass_frequency_channel: None,
                    signal_band: SignalBand::B2,
                    signal_code: SignalCode::B2I,
                    doppler_hz: 980.0,
                    code_phase_chips: 87.5,
                    carrier_phase_rad: 1.2,
                    cn0_db_hz: 49.0,
                    navigation_data: SyntheticNavigationData::AlternatingStartNegative,
                },
            ],
            ephemerides: Vec::new(),
            id: "composite_component_recovery_clean".to_string(),
        };
        let frame = generate_l1_ca_multi_signal_only(&config, &scenario);
        let bundle = build_iq16_capture_bundle(
            &scenario.id,
            &scenario,
            &frame,
            "2026-07-13T00:00:00Z",
            Some("synthetic composite component recovery clean".to_string()),
        );
        let scaled_frame = SamplesFrame::new(
            frame.t0,
            frame.dt_s,
            frame.iq.iter().map(|sample| *sample * bundle.truth.output_scale_applied).collect(),
        );

        let report = validate_truth_guided_composite_component_recovery(
            &config,
            &scaled_frame,
            &bundle.truth,
            1.0e-8,
            1.0e-9,
        );

        assert!(report.pass, "{report:?}");
        assert_eq!(report.scenario_id, scenario.id);
        assert_eq!(report.solver_status, "ok");
        assert_eq!(report.satellites.len(), 3);
        assert!(report.residual_rms <= 1.0e-6, "{report:?}");
        for row in &report.satellites {
            assert!(row.pass, "{row:?}");
            assert!(row.power_error_db.abs() <= 1.0e-8, "{row:?}");
            assert!(row.phase_error_rad.abs() <= 1.0e-9, "{row:?}");
            assert!((row.recovered_coefficient_magnitude - 1.0).abs() <= 1.0e-9, "{row:?}");
            assert!(row.recovered_coefficient_phase_rad.abs() <= 1.0e-9, "{row:?}");
        }
    }

    #[test]
    fn truth_guided_composite_component_recovery_flags_degenerate_truth_basis() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            ..ReceiverPipelineConfig::default()
        };
        let duplicated_signal = SyntheticSignalParams {
            sat: SatId { constellation: Constellation::Gps, prn: 3 },
            glonass_frequency_channel: None,
            signal_band: SignalBand::L1,
            signal_code: SignalCode::Ca,
            doppler_hz: 0.0,
            code_phase_chips: 100.0,
            carrier_phase_rad: 0.25,
            cn0_db_hz: 50.0,
            navigation_data: SyntheticNavigationData::ConstantPositive,
        };
        let scenario = SyntheticScenario {
            sample_rate_hz: config.sampling_freq_hz,
            intermediate_freq_hz: config.intermediate_freq_hz,
            receiver_clock_frequency_bias_hz: 0.0,
            duration_s: 0.02,
            seed: 0x2640_0002,
            satellites: vec![duplicated_signal.clone(), duplicated_signal],
            ephemerides: Vec::new(),
            id: "composite_component_recovery_degenerate".to_string(),
        };
        let frame = generate_l1_ca_multi_signal_only(&config, &scenario);
        let bundle = build_iq16_capture_bundle(
            &scenario.id,
            &scenario,
            &frame,
            "2026-07-13T00:00:00Z",
            Some("synthetic composite component recovery degenerate".to_string()),
        );
        let scaled_frame = SamplesFrame::new(
            frame.t0,
            frame.dt_s,
            frame.iq.iter().map(|sample| *sample * bundle.truth.output_scale_applied).collect(),
        );

        let report = validate_truth_guided_composite_component_recovery(
            &config,
            &scaled_frame,
            &bundle.truth,
            0.25,
            0.05,
        );

        assert!(!report.pass, "{report:?}");
        assert_eq!(report.solver_status, "degenerate_truth_basis");
        assert!(report.satellites.iter().all(|row| !row.pass), "{report:?}");
    }
