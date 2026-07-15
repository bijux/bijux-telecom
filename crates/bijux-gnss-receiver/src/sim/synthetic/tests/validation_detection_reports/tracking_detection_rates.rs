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
