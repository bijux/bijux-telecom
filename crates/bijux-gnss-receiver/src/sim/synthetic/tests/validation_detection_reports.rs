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

    #[test]
    fn truth_guided_code_phase_refinement_improves_fractional_pseudorange_initialization() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_000_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            ..ReceiverPipelineConfig::default()
        };
        let mut best_improvement_samples = 0.0_f64;

        for code_phase_chips in [200.125, 200.25, 200.375, 200.5, 200.625, 200.75, 200.875] {
            let scenario = SyntheticScenario {
                sample_rate_hz: config.sampling_freq_hz,
                intermediate_freq_hz: config.intermediate_freq_hz,
                receiver_clock_frequency_bias_hz: 0.0,
                duration_s: 0.04,
                seed: 24071985,
                satellites: vec![SyntheticSignalParams {
                    sat: SatId { constellation: Constellation::Gps, prn: 3 },
                    glonass_frequency_channel: None,
                    signal_band: bijux_gnss_core::api::SignalBand::L1,
 signal_code: bijux_gnss_core::api::SignalCode::Unknown,
                    doppler_hz: 0.0,
                    code_phase_chips,
                    carrier_phase_rad: 0.0,
                    cn0_db_hz: 65.0,
                    navigation_data: false.into(),
                }],
                ephemerides: Vec::new(),
                id: "acquisition_code_phase_refinement_truth".to_string(),
            };
            let frame = generate_l1_ca_multi(&config, &scenario);
            let bundle = build_iq16_capture_bundle(
                &scenario.id,
                &scenario,
                &frame,
                "2026-07-09T00:00:00Z",
                Some("unit code-phase refinement validation".to_string()),
            );
            let scaled_frame = SamplesFrame::new(
                frame.t0,
                frame.dt_s,
                frame.iq.iter().map(|sample| *sample * bundle.truth.output_scale_applied).collect(),
            );

            let report = validate_truth_guided_acquisition_code_phase_refinement(
                &config,
                &scaled_frame,
                &bundle.truth,
            );

            assert!(report.pass, "{report:?}");
            assert_eq!(report.satellites.len(), 1);
            let row = &report.satellites[0];
            assert!(row.pass, "{row:?}");
            best_improvement_samples = best_improvement_samples.max(row.improvement_samples);
        }

        assert!(
            best_improvement_samples > 0.0,
            "expected at least one fractional synthetic fixture to improve"
        );
    }

    #[test]
    fn truth_guided_acquisition_coherent_integration_report_combines_code_phase_and_doppler() {
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
            seed: 2_407_1985,
            satellites: vec![SyntheticSignalParams {
                sat: SatId { constellation: Constellation::Gps, prn: 3 },
                glonass_frequency_channel: None,
                signal_band: bijux_gnss_core::api::SignalBand::L1,
 signal_code: bijux_gnss_core::api::SignalCode::Unknown,
                doppler_hz: 750.0,
                code_phase_chips: 200.25,
                carrier_phase_rad: 0.0,
                cn0_db_hz: 58.0,
                navigation_data: false.into(),
            }],
            ephemerides: Vec::new(),
            id: "acquisition-coherent-integration-profile".to_string(),
        };
        let frame = generate_l1_ca_multi(&config, &scenario);
        let bundle = build_iq16_capture_bundle(
            &scenario.id,
            &scenario,
            &frame,
            "2026-07-09T00:00:00Z",
            Some("unit acquisition coherent integration validation".to_string()),
        );
        let scaled_frame = SamplesFrame::new(
            frame.t0,
            frame.dt_s,
            frame.iq.iter().map(|sample| *sample * bundle.truth.output_scale_applied).collect(),
        );

        let report = validate_truth_guided_acquisition_coherent_integration(
            &config,
            &scaled_frame,
            &bundle.truth,
            1,
            1,
            2,
            1,
        );

        assert!(report.pass, "{report:?}");
        assert_eq!(report.coherent_ms, 1);
        assert_eq!(report.noncoherent, 1);
        assert_eq!(report.satellites.len(), 1);
        assert!(report.satellites[0].pass, "{:?}", report.satellites[0]);
    }

    #[test]
    fn acquisition_detection_probability_report_tracks_accepted_trials() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            acquisition_doppler_search_hz: 10_000,
            acquisition_doppler_step_hz: 500,
            ..ReceiverPipelineConfig::default()
        };
        let report = measure_truth_guided_acquisition_detection_probability(
            &config,
            SyntheticSignalParams {
                sat: SatId { constellation: Constellation::Gps, prn: 3 },
                glonass_frequency_channel: None,
                signal_band: bijux_gnss_core::api::SignalBand::L1,
 signal_code: bijux_gnss_core::api::SignalCode::Unknown,
                doppler_hz: 750.0,
                code_phase_chips: 200.25,
                carrier_phase_rad: 0.0,
                cn0_db_hz: 58.0,
                navigation_data: false.into(),
            },
            1,
            1,
            &[2_407_1985, 2_407_1986],
            "acquisition-detection-probability",
            2,
            1,
        );

        assert_eq!(report.trial_count, 2);
        assert_eq!(report.accepted_count, report.trials.iter().filter(|trial| trial.accepted).count());
        assert_eq!(report.detected_count, report.trials.iter().filter(|trial| trial.detected).count());
        assert!(report.accepted_count <= report.trial_count);
        assert!(report.detected_count <= report.accepted_count);
        assert!(
            (report.acceptance_probability - report.accepted_count as f64 / report.trial_count as f64)
                .abs()
                <= f64::EPSILON
        );
        assert!(
            (report.detection_probability - report.detected_count as f64 / report.trial_count as f64)
                .abs()
                <= f64::EPSILON
        );
        assert!(
            report
                .trials
                .iter()
                .all(|trial| trial.code_phase_error_samples.is_some()
                    && trial.doppler_error_bins.is_some()),
            "{report:?}"
        );
    }

    #[test]
    fn tracking_sensitivity_report_counts_stable_lock_and_refusal_trials() {
        let sat = SatId { constellation: Constellation::Gps, prn: 7 };
        let report = synthetic_tracking_sensitivity_report(
            "tracking-sensitivity",
            sat,
            24.0,
            0.06,
            60.0,
            1,
            5,
            vec![
                SyntheticTrackingSensitivityTrial {
                    scenario_id: "tracking-sensitivity-trial-0".to_string(),
                    seed: 1,
                    sat,
                    stable_lock: true,
                    refused_lock: false,
                    first_lock_epoch_index: Some(4),
                    locked_epoch_count: 8,
                    final_lock_state: "tracking".to_string(),
                    final_lock_state_reason: Some("carrier_converged".to_string()),
                },
                SyntheticTrackingSensitivityTrial {
                    scenario_id: "tracking-sensitivity-trial-1".to_string(),
                    seed: 2,
                    sat,
                    stable_lock: false,
                    refused_lock: true,
                    first_lock_epoch_index: None,
                    locked_epoch_count: 0,
                    final_lock_state: "pull_in".to_string(),
                    final_lock_state_reason: Some("cn0_below_tracking_lock_floor".to_string()),
                },
            ],
        );

        assert_eq!(report.trial_count, 2);
        assert_eq!(report.stable_lock_count, 1);
        assert_eq!(report.refused_lock_count, 1);
        assert!((report.lock_probability - 0.5).abs() <= f64::EPSILON);
        assert!((report.mean_locked_epochs - 4.0).abs() <= f64::EPSILON);
    }

    #[test]
    fn tracking_lock_rate_report_keeps_cn0_and_seeded_error_axes() {
        let sat = SatId { constellation: Constellation::Gps, prn: 7 };
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
        let report = measure_truth_guided_tracking_lock_rate(
            &config,
            &[
                SyntheticTrackingLockRateCase {
                    signal: SyntheticSignalParams {
                        sat,
                        glonass_frequency_channel: None,
                        signal_band: bijux_gnss_core::api::SignalBand::L1,
 signal_code: bijux_gnss_core::api::SignalCode::Unknown,
                        doppler_hz: 180.0,
                        code_phase_chips: 211.25,
                        carrier_phase_rad: 0.0,
                        cn0_db_hz: 24.0,
                        navigation_data: false.into(),
                    },
                    duration_s: 0.04,
                    seeded_doppler_error_hz: 60.0,
                    seeded_code_phase_error_samples: 1,
                    min_locked_epochs: 4,
                },
                SyntheticTrackingLockRateCase {
                    signal: SyntheticSignalParams {
                        sat,
                        glonass_frequency_channel: None,
                        signal_band: bijux_gnss_core::api::SignalBand::L1,
 signal_code: bijux_gnss_core::api::SignalCode::Unknown,
                        doppler_hz: 180.0,
                        code_phase_chips: 211.25,
                        carrier_phase_rad: 0.0,
                        cn0_db_hz: 30.0,
                        navigation_data: false.into(),
                    },
                    duration_s: 0.04,
                    seeded_doppler_error_hz: 60.0,
                    seeded_code_phase_error_samples: 1,
                    min_locked_epochs: 4,
                },
            ],
            &[11, 29],
            "tracking-lock-rate-axes",
        );

        assert_eq!(report.points.len(), 2);
        assert_eq!(report.points[0].cn0_db_hz, 24.0);
        assert_eq!(report.points[1].cn0_db_hz, 30.0);
        assert_eq!(report.points[0].seeded_doppler_error_hz, 60.0);
        assert_eq!(report.points[0].seeded_code_phase_error_samples, 1);
        assert_eq!(report.points[0].trial_count, 2);
    }

    #[test]
    fn acquisition_false_alarm_report_tracks_noise_only_trials() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            acquisition_doppler_search_hz: 10_000,
            acquisition_doppler_step_hz: 500,
            ..ReceiverPipelineConfig::default()
        };
        let report = measure_noise_only_acquisition_false_alarm_rate(
            &config,
            SatId { constellation: Constellation::Gps, prn: 3 },
            1,
            1,
            &[17, 29],
            "acquisition-false-alarm",
        );

        assert_eq!(report.trial_count, 2);
        assert_eq!(report.false_alarm_count, 0);
        assert_eq!(report.false_alarm_rate, 0.0);
        assert!(report.trials.iter().all(|trial| !trial.accepted), "{report:?}");
    }

    #[test]
    fn acquisition_false_alarm_rate_report_keeps_integration_axes() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            acquisition_doppler_search_hz: 1_500,
            acquisition_doppler_step_hz: 250,
            ..ReceiverPipelineConfig::default()
        };
        let report = measure_noise_only_acquisition_false_alarm_rates(
            &config,
            &[
                SyntheticAcquisitionFalseAlarmRateCase {
                    sat: SatId { constellation: Constellation::Gps, prn: 7 },
                    coherent_ms: 1,
                    noncoherent: 1,
                },
                SyntheticAcquisitionFalseAlarmRateCase {
                    sat: SatId { constellation: Constellation::Gps, prn: 7 },
                    coherent_ms: 5,
                    noncoherent: 4,
                },
            ],
            &[31, 37],
            "acquisition-false-alarm-rate",
        );

        assert_eq!(report.acquisition_doppler_search_hz, 1_500);
        assert_eq!(report.acquisition_doppler_step_hz, 250);
        assert_eq!(report.points.len(), 2);
        assert_eq!(report.points[0].coherent_ms, 1);
        assert_eq!(report.points[0].noncoherent, 1);
        assert_eq!(report.points[1].coherent_ms, 5);
        assert_eq!(report.points[1].noncoherent, 4);
        assert!(report.points.iter().all(|point| point.trial_count == 2), "{report:?}");
    }

    #[test]
    fn acquisition_detection_rate_report_keeps_cn0_doppler_and_integration_axes() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            acquisition_doppler_search_hz: 1_500,
            acquisition_doppler_step_hz: 250,
            ..ReceiverPipelineConfig::default()
        };
        let report = measure_truth_guided_acquisition_detection_rate(
            &config,
            &[
                SyntheticAcquisitionDetectionRateCase {
                    signal: SyntheticSignalParams {
                        sat: SatId { constellation: Constellation::Gps, prn: 7 },
                        glonass_frequency_channel: None,
                        signal_band: bijux_gnss_core::api::SignalBand::L1,
 signal_code: bijux_gnss_core::api::SignalCode::Unknown,
                        doppler_hz: 250.0,
                        code_phase_chips: 300.0,
                        carrier_phase_rad: 0.0,
                        cn0_db_hz: 30.0,
                        navigation_data: false.into(),
                    },
                    coherent_ms: 1,
                    noncoherent: 1,
                },
                SyntheticAcquisitionDetectionRateCase {
                    signal: SyntheticSignalParams {
                        sat: SatId { constellation: Constellation::Gps, prn: 7 },
                        glonass_frequency_channel: None,
                        signal_band: bijux_gnss_core::api::SignalBand::L1,
 signal_code: bijux_gnss_core::api::SignalCode::Unknown,
                        doppler_hz: 750.0,
                        code_phase_chips: 300.0,
                        carrier_phase_rad: 0.0,
                        cn0_db_hz: 34.0,
                        navigation_data: false.into(),
                    },
                    coherent_ms: 5,
                    noncoherent: 1,
                },
            ],
            &[17, 29],
            "acquisition-detection-rate",
            2,
            1,
        );

        assert_eq!(report.doppler_step_hz, 250);
        assert_eq!(report.points.len(), 2);
        assert_eq!(report.points[0].cn0_db_hz, 30.0);
        assert_eq!(report.points[0].doppler_hz, 250.0);
        assert_eq!(report.points[0].coherent_ms, 1);
        assert_eq!(report.points[0].noncoherent, 1);
        assert_eq!(report.points[1].cn0_db_hz, 34.0);
        assert_eq!(report.points[1].doppler_hz, 750.0);
        assert_eq!(report.points[1].coherent_ms, 5);
        assert_eq!(report.points[1].noncoherent, 1);
    }

    #[test]
    fn acquisition_interference_report_keeps_profile_axes_and_classifications() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            acquisition_doppler_search_hz: 1_500,
            acquisition_doppler_step_hz: 250,
            ..ReceiverPipelineConfig::default()
        };
        let report = measure_truth_guided_acquisition_interference(
            &config,
            &[SyntheticAcquisitionInterferenceCase {
                target_signal: SyntheticSignalParams {
                    sat: SatId { constellation: Constellation::Gps, prn: 7 },
                    glonass_frequency_channel: None,
                    signal_band: bijux_gnss_core::api::SignalBand::L1,
                    signal_code: bijux_gnss_core::api::SignalCode::Ca,
                    doppler_hz: 250.0,
                    code_phase_chips: 300.0,
                    carrier_phase_rad: 0.0,
                    cn0_db_hz: 30.0,
                    navigation_data: false.into(),
                },
                interfering_signals: vec![SyntheticSignalParams {
                    sat: SatId { constellation: Constellation::Gps, prn: 3 },
                    glonass_frequency_channel: None,
                    signal_band: bijux_gnss_core::api::SignalBand::L1,
                    signal_code: bijux_gnss_core::api::SignalCode::Ca,
                    doppler_hz: 250.0,
                    code_phase_chips: 300.0,
                    carrier_phase_rad: 0.25,
                    cn0_db_hz: 54.0,
                    navigation_data: false.into(),
                }],
                coherent_ms: 1,
                noncoherent: 1,
            }],
            &[17, 29],
            "acquisition-interference",
            2,
            1,
        );

        assert_eq!(report.doppler_step_hz, 250);
        assert_eq!(report.points.len(), 1);
        let point = &report.points[0];
        assert_eq!(point.case.target_signal.sat.prn, 7);
        assert_eq!(point.case.interfering_signals.len(), 1);
        assert_eq!(point.case.coherent_ms, 1);
        assert_eq!(point.trial_count, 2);
        assert_eq!(point.trials.len(), 2);
        assert_eq!(
            point.thermal_noise_failure_count + point.cross_signal_interference_failure_count
                + point.interfered_detection_count,
            point.trial_count
        );
        assert!(
            point.thermal_noise_false_alarm_count + point.cross_signal_false_alarm_count
                <= point.trial_count
        );
        assert!(
            point.trials.iter().all(|trial| matches!(
                trial.failure_class,
                SyntheticAcquisitionInterferenceFailureClass::Detected
                    | SyntheticAcquisitionInterferenceFailureClass::ThermalNoiseLimited
                    | SyntheticAcquisitionInterferenceFailureClass::CrossSignalInterference
            )),
            "{report:?}"
        );
        assert!(
            point.trials.iter().all(|trial| matches!(
                trial.false_alarm_class,
                SyntheticAcquisitionFalseAlarmClass::None
                    | SyntheticAcquisitionFalseAlarmClass::ThermalNoise
                    | SyntheticAcquisitionFalseAlarmClass::CrossSignalInterference
            )),
            "{report:?}"
        );
    }

    #[test]
    fn acquisition_uncertainty_coverage_report_keeps_stationary_axes() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_000_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            acquisition_doppler_search_hz: 1_500,
            acquisition_doppler_step_hz: 250,
            acquisition_peak_second_threshold: 1.01,
            ..ReceiverPipelineConfig::default()
        };
        let report = measure_synthetic_acquisition_uncertainty_coverage(
            &config,
            &[SyntheticAcquisitionUncertaintyCoverageCase {
                signal: SyntheticSignalParams {
                    sat: SatId { constellation: Constellation::Gps, prn: 3 },
                    glonass_frequency_channel: None,
                    signal_band: bijux_gnss_core::api::SignalBand::L1,
                    signal_code: bijux_gnss_core::api::SignalCode::Ca,
                    doppler_hz: 375.0,
                    code_phase_chips: 200.375,
                    carrier_phase_rad: 0.0,
                    cn0_db_hz: 65.0,
                    navigation_data: false.into(),
                },
                coherent_ms: 1,
                noncoherent: 1,
                duration_s: 0.001,
                doppler_rate_hz_per_s: None,
            }],
            2,
        );

        assert_eq!(report.doppler_step_hz, 250);
        assert_eq!(report.points.len(), 1);
        let point = &report.points[0];
        assert_eq!(point.case.signal.sat.prn, 3);
        assert_eq!(point.trial_count, 2);
        assert!(point.successful_trial_count > 0, "{report:?}");
        assert_eq!(point.trials.len(), 2);
        assert_eq!(point.doppler_rate_within_one_sigma_count, None);
        assert_eq!(point.doppler_rate_within_one_sigma_rate, None);
        assert!(point.doppler_within_one_sigma_rate.is_finite());
        assert!(point.code_phase_within_one_sigma_rate.is_finite());
        assert!(point.trials.iter().any(|trial| {
            trial.reported_doppler_sigma_hz.is_some_and(|sigma| sigma > 0.0)
                && trial
                    .reported_code_phase_sigma_samples
                    .is_some_and(|sigma| sigma > 0.0)
                && trial.reported_doppler_rate_sigma_hz_per_s.is_none()
        }));
    }

    #[test]
    fn acquisition_uncertainty_coverage_report_keeps_rate_axis_when_present() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            acquisition_doppler_search_hz: 500,
            acquisition_doppler_step_hz: 250,
            acquisition_doppler_rate_search_hz_per_s: 25_000,
            acquisition_doppler_rate_step_hz_per_s: 5_000,
            acquisition_integration_ms: 20,
            acquisition_noncoherent: 1,
            acquisition_peak_mean_threshold: 8.0,
            acquisition_peak_second_threshold: 1.01,
            ..ReceiverPipelineConfig::default()
        };
        let report = measure_synthetic_acquisition_uncertainty_coverage(
            &config,
            &[SyntheticAcquisitionUncertaintyCoverageCase {
                signal: SyntheticSignalParams {
                    sat: SatId { constellation: Constellation::Gps, prn: 16 },
                    glonass_frequency_channel: None,
                    signal_band: bijux_gnss_core::api::SignalBand::L1,
                    signal_code: bijux_gnss_core::api::SignalCode::Ca,
                    doppler_hz: 250.0,
                    code_phase_chips: 211.25,
                    carrier_phase_rad: 0.40,
                    cn0_db_hz: 75.0,
                    navigation_data: false.into(),
                },
                coherent_ms: 20,
                noncoherent: 1,
                duration_s: 0.020,
                doppler_rate_hz_per_s: Some(20_000.0),
            }],
            1,
        );

        assert_eq!(report.doppler_rate_step_hz_per_s, 5_000);
        assert_eq!(report.points.len(), 1);
        let point = &report.points[0];
        assert_eq!(point.successful_trial_count, 1, "{report:?}");
        assert_eq!(point.doppler_rate_within_one_sigma_count, Some(1));
        assert!(point.doppler_rate_within_one_sigma_rate.is_some());
        assert!(point.trials.iter().all(|trial| {
            trial
                .reported_doppler_rate_sigma_hz_per_s
                .is_some_and(|sigma| sigma > 0.0)
                && trial.doppler_rate_within_one_sigma.is_some()
        }));
    }

    #[test]
    fn observation_error_summary_tracks_bias_and_absolute_magnitude() {
        let summary =
            summarize_observation_errors(&[-2.0, 1.0, 3.0]).expect("error summary must exist");

        assert_eq!(summary.count, 3);
        assert!((summary.mean_error - (2.0 / 3.0)).abs() <= 1.0e-12, "{summary:?}");
        assert!((summary.median_abs_error - 2.0).abs() <= 1.0e-12, "{summary:?}");
        assert!((summary.rms_error - (14.0_f64 / 3.0).sqrt()).abs() <= 1.0e-12, "{summary:?}");
        assert!((summary.p95_abs_error - 3.0).abs() <= 1.0e-12, "{summary:?}");
        assert!((summary.max_abs_error - 3.0).abs() <= 1.0e-12, "{summary:?}");
    }
