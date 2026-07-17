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
