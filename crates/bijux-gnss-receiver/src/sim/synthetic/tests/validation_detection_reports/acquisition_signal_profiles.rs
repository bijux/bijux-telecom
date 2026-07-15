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
