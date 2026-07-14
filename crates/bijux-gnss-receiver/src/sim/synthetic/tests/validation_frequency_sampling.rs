    #[test]
    fn truth_guided_acquisition_doppler_validation_matches_clean_truth_within_one_bin() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            acquisition_doppler_search_hz: 10_000,
            acquisition_doppler_step_hz: 500,
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
                    doppler_hz: -1_000.0,
                    code_phase_chips: 321.5,
                    carrier_phase_rad: 0.2,
                    cn0_db_hz: 52.0,
                    navigation_data: false.into(),
                },
            ],
            ephemerides: Vec::new(),
            id: "acquisition-doppler-validation".to_string(),
        };
        let frame = generate_l1_ca_multi(&config, &scenario);
        let bundle = build_iq16_capture_bundle(
            &scenario.id,
            &scenario,
            &frame,
            "2026-07-09T00:00:00Z",
            Some("synthetic acquisition doppler validation".to_string()),
        );
        let scaled_frame = SamplesFrame::new(
            frame.t0,
            frame.dt_s,
            frame.iq.iter().map(|sample| *sample * bundle.truth.output_scale_applied).collect(),
        );
        let report =
            validate_truth_guided_acquisition_doppler(&config, &scaled_frame, &bundle.truth, 1);

        assert!(report.pass, "{report:?}");
        assert_eq!(report.tolerance_hz, 500.0);
        assert_eq!(report.doppler_step_hz, 500);
        assert_eq!(report.satellites.len(), 2);
        for row in &report.satellites {
            assert!(row.pass, "{row:?}");
            assert!(row.doppler_error_hz <= 500.0 + f64::EPSILON, "{row:?}");
            assert!(row.doppler_error_bins <= 1.0 + f64::EPSILON, "{row:?}");
            assert_ne!(row.hypothesis, "deferred");
        }
    }

    #[test]
    fn truth_guided_acquisition_receiver_clock_offset_validation_reports_positive_bias() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            acquisition_doppler_search_hz: 10_000,
            acquisition_doppler_step_hz: 250,
            ..ReceiverPipelineConfig::default()
        };
        let scenario = SyntheticScenario {
            sample_rate_hz: config.sampling_freq_hz,
            intermediate_freq_hz: config.intermediate_freq_hz,
            receiver_clock_frequency_bias_hz: 500.0,
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
                    doppler_hz: -1_000.0,
                    code_phase_chips: 321.5,
                    carrier_phase_rad: 0.2,
                    cn0_db_hz: 52.0,
                    navigation_data: false.into(),
                },
            ],
            ephemerides: Vec::new(),
            id: "acquisition-receiver-clock-offset-positive".to_string(),
        };
        let frame = generate_l1_ca_multi(&config, &scenario);
        let bundle = build_iq16_capture_bundle(
            &scenario.id,
            &scenario,
            &frame,
            "2026-07-09T00:00:00Z",
            Some("synthetic acquisition receiver clock offset validation".to_string()),
        );
        let scaled_frame = SamplesFrame::new(
            frame.t0,
            frame.dt_s,
            frame.iq.iter().map(|sample| *sample * bundle.truth.output_scale_applied).collect(),
        );
        let report = validate_truth_guided_acquisition_receiver_clock_offset(
            &config,
            &scaled_frame,
            &bundle.truth,
            1,
        );

        assert!(report.pass, "{report:?}");
        assert_eq!(report.injected_receiver_clock_frequency_bias_hz, 500.0);
        assert_eq!(report.tolerance_hz, 250.0);
        assert_eq!(report.doppler_step_hz, 250);
        assert_eq!(report.satellites.len(), 2);
        assert!(
            (report.mean_measured_receiver_clock_frequency_bias_hz - 500.0).abs() <= 250.0,
            "{report:?}"
        );
        for row in &report.satellites {
            assert!(row.pass, "{row:?}");
            assert!((row.measured_receiver_clock_frequency_bias_hz - 500.0).abs() <= 250.0, "{row:?}");
            assert!(
                (row.expected_measured_doppler_hz - row.measured_doppler_hz).abs() <= 250.0,
                "{row:?}"
            );
        }
    }

    #[test]
    fn truth_guided_acquisition_receiver_clock_offset_validation_reports_negative_bias() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            acquisition_doppler_search_hz: 10_000,
            acquisition_doppler_step_hz: 250,
            ..ReceiverPipelineConfig::default()
        };
        let scenario = SyntheticScenario {
            sample_rate_hz: config.sampling_freq_hz,
            intermediate_freq_hz: config.intermediate_freq_hz,
            receiver_clock_frequency_bias_hz: -500.0,
            duration_s: 0.04,
            seed: 24071986,
            satellites: vec![
                SyntheticSignalParams {
                    sat: SatId { constellation: Constellation::Gps, prn: 11 },
                    glonass_frequency_channel: None,
                    signal_band: bijux_gnss_core::api::SignalBand::L1,
 signal_code: bijux_gnss_core::api::SignalCode::Unknown,
                    doppler_hz: 1_250.0,
                    code_phase_chips: 150.25,
                    carrier_phase_rad: 0.0,
                    cn0_db_hz: 58.0,
                    navigation_data: false.into(),
                },
                SyntheticSignalParams {
                    sat: SatId { constellation: Constellation::Gps, prn: 19 },
                    glonass_frequency_channel: None,
                    signal_band: bijux_gnss_core::api::SignalBand::L1,
 signal_code: bijux_gnss_core::api::SignalCode::Unknown,
                    doppler_hz: -750.0,
                    code_phase_chips: 420.5,
                    carrier_phase_rad: 0.3,
                    cn0_db_hz: 54.0,
                    navigation_data: true.into(),
                },
            ],
            ephemerides: Vec::new(),
            id: "acquisition-receiver-clock-offset-negative".to_string(),
        };
        let frame = generate_l1_ca_multi(&config, &scenario);
        let bundle = build_iq16_capture_bundle(
            &scenario.id,
            &scenario,
            &frame,
            "2026-07-09T00:00:00Z",
            Some("synthetic acquisition receiver clock offset validation".to_string()),
        );
        let scaled_frame = SamplesFrame::new(
            frame.t0,
            frame.dt_s,
            frame.iq.iter().map(|sample| *sample * bundle.truth.output_scale_applied).collect(),
        );
        let report = validate_truth_guided_acquisition_receiver_clock_offset(
            &config,
            &scaled_frame,
            &bundle.truth,
            1,
        );

        assert!(report.pass, "{report:?}");
        assert_eq!(report.injected_receiver_clock_frequency_bias_hz, -500.0);
        assert_eq!(report.satellites.len(), 2);
        assert!(report.max_measured_receiver_clock_frequency_bias_hz <= 0.0, "{report:?}");
        assert!(report.min_measured_receiver_clock_frequency_bias_hz <= -250.0, "{report:?}");
        for row in &report.satellites {
            assert!(row.pass, "{row:?}");
            assert!(row.measured_receiver_clock_frequency_bias_hz < 0.0, "{row:?}");
        }
    }

    #[test]
    fn truth_guided_common_oscillator_bias_follow_up_recovers_bias_and_rescues_missed_signal() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            acquisition_doppler_search_hz: 2_000,
            acquisition_doppler_step_hz: 250,
            ..ReceiverPipelineConfig::default()
        };
        let scenario = SyntheticScenario {
            sample_rate_hz: config.sampling_freq_hz,
            intermediate_freq_hz: config.intermediate_freq_hz,
            receiver_clock_frequency_bias_hz: 500.0,
            duration_s: 0.04,
            seed: 24071987,
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
                    doppler_hz: -1_000.0,
                    code_phase_chips: 321.5,
                    carrier_phase_rad: 0.2,
                    cn0_db_hz: 60.0,
                    navigation_data: false.into(),
                },
                SyntheticSignalParams {
                    sat: SatId { constellation: Constellation::Gps, prn: 11 },
                    glonass_frequency_channel: None,
                    signal_band: bijux_gnss_core::api::SignalBand::L1,
                    signal_code: bijux_gnss_core::api::SignalCode::Unknown,
                    doppler_hz: 250.0,
                    code_phase_chips: 420.5,
                    carrier_phase_rad: 0.3,
                    cn0_db_hz: 60.0,
                    navigation_data: false.into(),
                },
            ],
            ephemerides: Vec::new(),
            id: "common-oscillator-bias-follow-up".to_string(),
        };
        let frame = generate_l1_ca_multi_signal_only(&config, &scenario);
        let bundle = build_iq16_capture_bundle(
            &scenario.id,
            &scenario,
            &frame,
            "2026-07-14T00:00:00Z",
            Some("synthetic common oscillator bias follow-up".to_string()),
        );
        let scaled_frame = SamplesFrame::new(
            frame.t0,
            frame.dt_s,
            frame.iq.iter().map(|sample| *sample * bundle.truth.output_scale_applied).collect(),
        );
        let requests = vec![
            bijux_gnss_core::api::AcqRequest {
                sat: scenario.satellites[0].sat,
                glonass_frequency_channel: None,
                signal_band: bijux_gnss_core::api::SignalBand::L1,
                signal_code: bijux_gnss_core::api::SignalCode::Unknown,
                doppler_center_hz: 750.0,
                expected_line_of_sight_doppler_hz: Some(750.0),
                doppler_search_hz: 750,
                doppler_step_hz: 250,
                coherent_ms: 1,
                noncoherent: 1,
            },
            bijux_gnss_core::api::AcqRequest {
                sat: scenario.satellites[1].sat,
                glonass_frequency_channel: None,
                signal_band: bijux_gnss_core::api::SignalBand::L1,
                signal_code: bijux_gnss_core::api::SignalCode::Unknown,
                doppler_center_hz: -1_000.0,
                expected_line_of_sight_doppler_hz: Some(-1_000.0),
                doppler_search_hz: 750,
                doppler_step_hz: 250,
                coherent_ms: 1,
                noncoherent: 1,
            },
            bijux_gnss_core::api::AcqRequest {
                sat: scenario.satellites[2].sat,
                glonass_frequency_channel: None,
                signal_band: bijux_gnss_core::api::SignalBand::L1,
                signal_code: bijux_gnss_core::api::SignalCode::Unknown,
                doppler_center_hz: 250.0,
                expected_line_of_sight_doppler_hz: Some(250.0),
                doppler_search_hz: 250,
                doppler_step_hz: 250,
                coherent_ms: 1,
                noncoherent: 1,
            },
        ];

        let report = validate_truth_guided_common_oscillator_bias_follow_up(
            &config,
            &scaled_frame,
            &bundle.truth,
            &requests,
            1,
        );

        assert!(report.pass, "{report:#?}");
        assert_eq!(report.injected_receiver_clock_frequency_bias_hz, 500.0);
        assert_eq!(report.support_count, 2);
        assert_eq!(report.follow_up_count, 1);
        assert_eq!(report.improved_follow_up_count, 1);
        assert!(
            report
                .estimated_common_oscillator_bias_hz
                .is_some_and(|estimated_bias_hz| (estimated_bias_hz - 500.0).abs() <= 250.0),
            "{report:#?}"
        );

        let rescued_satellite = report
            .satellites
            .iter()
            .find(|satellite| satellite.sat == scenario.satellites[2].sat)
            .expect("rescued satellite row");
        assert_eq!(rescued_satellite.initial_hypothesis, "rejected", "{rescued_satellite:#?}");
        assert!(rescued_satellite.follow_up_requested, "{rescued_satellite:#?}");
        assert_eq!(rescued_satellite.follow_up_doppler_search_hz, Some(250));
        assert!(
            rescued_satellite
                .follow_up_doppler_center_hz
                .is_some_and(|center_hz| (center_hz - 750.0).abs() <= 250.0),
            "{rescued_satellite:#?}"
        );
        assert!(
            rescued_satellite
                .follow_up_hypothesis
                .as_deref()
                .is_some_and(|hypothesis| matches!(hypothesis, "accepted" | "ambiguous")),
            "{rescued_satellite:#?}"
        );
        assert!(rescued_satellite.improved, "{rescued_satellite:#?}");
    }

    #[test]
    fn acquisition_sample_rate_validation_passes_with_distinct_low_and_high_rate_profiles() {
        let low_rate = synthetic_acquisition_sample_rate_case_fixture(
            2_046_000.0,
            "acquisition_sample_rate_validation_low_rate",
            0x2_046_000,
        );
        let high_rate = synthetic_acquisition_sample_rate_case_fixture(
            4_092_000.0,
            "acquisition_sample_rate_validation_high_rate",
            0x4_092_000,
        );

        let report =
            validate_truth_guided_acquisition_sample_rates(&[low_rate.case, high_rate.case], 2, 1);

        assert!(report.pass, "{report:?}");
        assert_eq!(report.code_phase_tolerance_samples, 2);
        assert_eq!(report.doppler_tolerance_bins, 1);
        assert_eq!(report.doppler_tolerance_hz, 500.0);
        assert_eq!(report.distinct_sample_rate_count, 2);
        assert_eq!(report.min_sample_rate_hz, 2_046_000.0);
        assert_eq!(report.max_sample_rate_hz, 4_092_000.0);
        assert_eq!(report.points.len(), 2);
        assert_eq!(report.points[0].scenario_id, low_rate.truth.scenario_id);
        assert_eq!(report.points[1].scenario_id, high_rate.truth.scenario_id);
        for point in &report.points {
            assert!(point.pass, "{point:?}");
            assert!(point.code_phase_validation.pass, "{point:?}");
            assert!(point.doppler_validation.pass, "{point:?}");
        }
    }

    #[test]
    fn acquisition_sample_rate_validation_requires_distinct_sample_rates() {
        let alpha = synthetic_acquisition_sample_rate_case_fixture(
            4_092_000.0,
            "acquisition_sample_rate_validation_same_rate_alpha",
            0x4_092_001,
        );
        let beta = synthetic_acquisition_sample_rate_case_fixture(
            4_092_000.0,
            "acquisition_sample_rate_validation_same_rate_beta",
            0x4_092_002,
        );

        let report = validate_truth_guided_acquisition_sample_rates(&[alpha.case, beta.case], 2, 1);

        assert!(!report.pass, "{report:?}");
        assert_eq!(report.distinct_sample_rate_count, 1);
        assert_eq!(report.min_sample_rate_hz, 4_092_000.0);
        assert_eq!(report.max_sample_rate_hz, 4_092_000.0);
        assert!(report.points.iter().all(|point| point.pass), "{report:?}");
    }

    #[test]
    fn acquisition_sample_rate_validation_orders_profiles_by_rate_then_scenario_id() {
        let zeta = synthetic_acquisition_sample_rate_case_fixture(
            4_092_000.0,
            "acquisition_sample_rate_validation_zeta",
            0x4_092_003,
        );
        let beta = synthetic_acquisition_sample_rate_case_fixture(
            2_046_000.0,
            "acquisition_sample_rate_validation_beta",
            0x2_046_001,
        );
        let alpha = synthetic_acquisition_sample_rate_case_fixture(
            2_046_000.0,
            "acquisition_sample_rate_validation_alpha",
            0x2_046_002,
        );

        let report =
            validate_truth_guided_acquisition_sample_rates(&[zeta.case, beta.case, alpha.case], 2, 1);
        let ordered_points = report
            .points
            .iter()
            .map(|point| (point.sample_rate_hz, point.scenario_id.as_str()))
            .collect::<Vec<_>>();

        assert_eq!(
            ordered_points,
            vec![
                (2_046_000.0, "acquisition_sample_rate_validation_alpha"),
                (2_046_000.0, "acquisition_sample_rate_validation_beta"),
                (4_092_000.0, "acquisition_sample_rate_validation_zeta"),
            ]
        );
    }

    struct SyntheticAcquisitionSampleRateCaseFixture {
        case: SyntheticAcquisitionSampleRateValidationCase<'static>,
        truth: &'static super::SyntheticIqTruthBundle,
    }

    fn synthetic_acquisition_sample_rate_case_fixture(
        sampling_freq_hz: f64,
        scenario_id: &str,
        seed: u64,
    ) -> SyntheticAcquisitionSampleRateCaseFixture {
        let config = Box::leak(Box::new(ReceiverPipelineConfig {
            sampling_freq_hz,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            acquisition_doppler_search_hz: 10_000,
            acquisition_doppler_step_hz: 500,
            ..ReceiverPipelineConfig::default()
        }));
        let scenario = SyntheticScenario {
            sample_rate_hz: config.sampling_freq_hz,
            intermediate_freq_hz: config.intermediate_freq_hz,
            receiver_clock_frequency_bias_hz: 0.0,
            duration_s: 0.04,
            seed,
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
                    doppler_hz: -1_000.0,
                    code_phase_chips: 321.5,
                    carrier_phase_rad: 0.2,
                    cn0_db_hz: 52.0,
                    navigation_data: false.into(),
                },
            ],
            ephemerides: Vec::new(),
            id: scenario_id.to_string(),
        };
        let frame = generate_l1_ca_multi(config, &scenario);
        let bundle = build_iq16_capture_bundle(
            &scenario.id,
            &scenario,
            &frame,
            "2026-07-09T00:00:00Z",
            Some("synthetic acquisition sample-rate validation".to_string()),
        );
        let scaled_frame = Box::leak(Box::new(SamplesFrame::new(
            frame.t0,
            frame.dt_s,
            frame.iq.iter().map(|sample| *sample * bundle.truth.output_scale_applied).collect(),
        )));
        let truth = Box::leak(Box::new(bundle.truth));

        SyntheticAcquisitionSampleRateCaseFixture {
            case: SyntheticAcquisitionSampleRateValidationCase { config, frame: scaled_frame, truth },
            truth,
        }
    }

    #[test]
    fn signal_only_streaming_source_matches_signal_only_frame_generation() {
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
            seed: 17,
            satellites: vec![
                SyntheticSignalParams {
                    sat: SatId { constellation: Constellation::Gps, prn: 3 },
                    glonass_frequency_channel: None,
                    signal_band: bijux_gnss_core::api::SignalBand::L1,
 signal_code: bijux_gnss_core::api::SignalCode::Unknown,
                    doppler_hz: -120.0,
                    code_phase_chips: 64.0,
                    carrier_phase_rad: 0.25,
                    cn0_db_hz: 55.0,
                    navigation_data: false.into(),
                },
                SyntheticSignalParams {
                    sat: SatId { constellation: Constellation::Gps, prn: 11 },
                    glonass_frequency_channel: None,
                    signal_band: bijux_gnss_core::api::SignalBand::L1,
 signal_code: bijux_gnss_core::api::SignalCode::Unknown,
                    doppler_hz: 180.0,
                    code_phase_chips: 288.5,
                    carrier_phase_rad: 0.75,
                    cn0_db_hz: 55.0,
                    navigation_data: false.into(),
                },
            ],
            ephemerides: Vec::new(),
            id: "signal-only-streaming".to_string(),
        };
        let expected = super::generate_l1_ca_multi_signal_only(&config, &scenario);
        let mut source = super::SyntheticSignalSource::new_signal_only(&config, &scenario);
        let streamed = source
            .next_frame(expected.len())
            .expect("streaming signal-only frame")
            .expect("non-empty signal-only source");

        assert_eq!(streamed.t0, expected.t0);
        assert_eq!(streamed.dt_s, expected.dt_s);
        assert_eq!(streamed.iq, expected.iq);
        assert!(source.is_done());
    }

    #[test]
    fn signal_only_streaming_source_preserves_gps_l5q_secondary_phase_across_irregular_frames() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 1_500_001.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 10_230_000.0,
            code_length: 10_230,
            ..ReceiverPipelineConfig::default()
        };
        let scenario = SyntheticScenario {
            sample_rate_hz: config.sampling_freq_hz,
            intermediate_freq_hz: config.intermediate_freq_hz,
            receiver_clock_frequency_bias_hz: 0.0,
            duration_s: 0.045,
            seed: 71,
            satellites: vec![SyntheticSignalParams {
                sat: SatId { constellation: Constellation::Gps, prn: 24 },
                glonass_frequency_channel: None,
                signal_band: bijux_gnss_core::api::SignalBand::L5,
                signal_code: bijux_gnss_core::api::SignalCode::L5Q,
                doppler_hz: 180.0,
                code_phase_chips: 1_024.75,
                carrier_phase_rad: 0.25,
                cn0_db_hz: 55.0,
                navigation_data: false.into(),
            }],
            ephemerides: Vec::new(),
            id: "signal-only-gps-l5q-streaming".to_string(),
        };
        let expected = super::generate_l1_ca_multi_signal_only(&config, &scenario);
        let mut source = super::SyntheticSignalSource::new_signal_only(&config, &scenario);
        let streamed = collect_streamed_signal_only_frames(&mut source, &[257, 4093, 8191, 173]);

        assert_eq!(streamed.t0, expected.t0);
        assert_eq!(streamed.dt_s, expected.dt_s);
        assert_eq!(streamed.iq, expected.iq);
        assert!(source.is_done());
    }

    #[test]
    fn signal_only_streaming_source_preserves_galileo_e1_secondary_phase_across_irregular_frames() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 2_700_001.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 4_092,
            ..ReceiverPipelineConfig::default()
        };
        let scenario = SyntheticScenario {
            sample_rate_hz: config.sampling_freq_hz,
            intermediate_freq_hz: config.intermediate_freq_hz,
            receiver_clock_frequency_bias_hz: 0.0,
            duration_s: 0.108,
            seed: 73,
            satellites: vec![SyntheticSignalParams {
                sat: SatId { constellation: Constellation::Galileo, prn: 11 },
                glonass_frequency_channel: None,
                signal_band: bijux_gnss_core::api::SignalBand::E1,
                signal_code: bijux_gnss_core::api::SignalCode::E1B,
                doppler_hz: -120.0,
                code_phase_chips: 137.625,
                carrier_phase_rad: 0.75,
                cn0_db_hz: 55.0,
                navigation_data: false.into(),
            }],
            ephemerides: Vec::new(),
            id: "signal-only-galileo-e1-streaming".to_string(),
        };
        let expected = super::generate_l1_ca_multi_signal_only(&config, &scenario);
        let mut source = super::SyntheticSignalSource::new_signal_only(&config, &scenario);
        let streamed =
            collect_streamed_signal_only_frames(&mut source, &[509, 4097, 12_289, 641, 97]);

        assert_eq!(streamed.t0, expected.t0);
        assert_eq!(streamed.dt_s, expected.dt_s);
        assert_eq!(streamed.iq, expected.iq);
        assert!(source.is_done());
    }

    fn collect_streamed_signal_only_frames(
        source: &mut super::SyntheticSignalSource,
        frame_lengths: &[usize],
    ) -> SamplesFrame {
        let mut combined = Vec::new();
        let mut t0 = None;
        let mut dt_s = None;
        let mut frame_index = 0usize;

        while let Some(frame) = source
            .next_frame(frame_lengths[frame_index % frame_lengths.len()])
            .expect("streaming signal-only frame")
        {
            t0.get_or_insert(frame.t0);
            dt_s.get_or_insert(frame.dt_s);
            combined.extend(frame.iq);
            frame_index += 1;
        }

        SamplesFrame::new(
            t0.expect("streaming source emitted at least one frame"),
            dt_s.expect("streaming source emitted frame spacing"),
            combined,
        )
    }
