    #[test]
    fn synthetic_signal_source_matches_materialized_generator() {
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
            seed: 29,
            satellites: vec![SyntheticSignalParams {
                sat: SatId { constellation: Constellation::Gps, prn: 7 },
                glonass_frequency_channel: None,
                doppler_hz: 750.0,
                code_phase_chips: 15.0,
                carrier_phase_rad: 0.0,
                cn0_db_hz: 47.0,
                data_bit_flip: false,
            }],
            ephemerides: Vec::new(),
            id: "synthetic-stream".to_string(),
        };

        let expected = generate_l1_ca_multi(&config, &scenario);
        let mut source = SyntheticSignalSource::new(&config, &scenario);
        let streamed = collect_frames(&mut source, 2 * 1_023);

        assert_eq!(expected.len(), streamed.len());
        assert_eq!(expected.t0, streamed.t0);
        assert_eq!(expected.dt_s, streamed.dt_s);
        assert_eq!(expected.iq, streamed.iq);
        assert!(source.is_done());
    }

    #[test]
    fn pvt_truth_table_records_truth_measured_values_and_errors() {
        let truth_ecef = lla_to_ecef(37.0, -122.0, 10.0);
        let measured_ecef = (truth_ecef.0 + 1.5, truth_ecef.1 - 2.0, truth_ecef.2 + 0.75);
        let measured_geodetic = ecef_to_geodetic(measured_ecef.0, measured_ecef.1, measured_ecef.2);
        let truth_clock_bias_s = 2.0e-4;
        let measured_clock_bias_s = 2.5e-4;
        let solution = NavSolutionEpoch {
            epoch: Epoch { index: 7 },
            t_rx_s: Seconds(100_000.0),
            source_time: ReceiverSampleTrace::from_sample_index(0, 1.0),
            ecef_x_m: Meters(measured_ecef.0),
            ecef_y_m: Meters(measured_ecef.1),
            ecef_z_m: Meters(measured_ecef.2),
            position_covariance_ecef_m2: None,
            latitude_deg: measured_geodetic.0,
            longitude_deg: measured_geodetic.1,
            altitude_m: Meters(measured_geodetic.2),
            clock_bias_s: Seconds(measured_clock_bias_s),
            clock_bias_m: Meters(measured_clock_bias_s * SPEED_OF_LIGHT_MPS),
            clock_drift_s_per_s: 0.0,
            pdop: 1.2,
            pre_fit_residual_rms_m: Some(Meters(3.5)),
            post_fit_residual_rms_m: Some(Meters(1.25)),
            rms_m: Meters(1.25),
            status: SolutionStatus::Converged,
            quality: NavQualityFlag::Float,
            validity: SolutionValidity::Stable,
            valid: true,
            processing_ms: None,
            residuals: Vec::new(),
            constellation_residual_rms: Vec::new(),
            health: Vec::new(),
            isb: Vec::new(),
            sigma_e_m: None,
            sigma_n_m: None,
            sigma_u_m: None,
            sigma_h_m: None,
            sigma_v_m: None,
            innovation_rms_m: None,
            normalized_innovation_rms: None,
            normalized_innovation_max: None,
            ekf_innovation_rms: None,
            ekf_condition_number: None,
            ekf_whiteness_ratio: None,
            ekf_predicted_variance: None,
            ekf_observed_variance: None,
            integrity_hpl_m: None,
            integrity_vpl_m: None,
            model_version: NAV_SOLUTION_MODEL_VERSION,
            lifecycle_state: NavLifecycleState::Converged,
            uncertainty_class: NavUncertaintyClass::Low,
            assumptions: None,
            refusal_class: None,
            artifact_id: "nav-epoch-0000000007-pvt-truth".to_string(),
            source_observation_epoch_id: "obs-epoch-0000000007-pvt-truth".to_string(),
            explain_decision: "accepted".to_string(),
            explain_reasons: vec!["navigation_solution_usable".to_string()],
            provenance: None,
            sat_count: 5,
            used_sat_count: 4,
            rejected_sat_count: 1,
            hdop: Some(0.9),
            vdop: Some(0.8),
            gdop: Some(1.3),
            tdop: Some(0.4),
            stability_signature: "navsig:v2:pvt-truth".to_string(),
            stability_signature_version: NAV_OUTPUT_STABILITY_SIGNATURE_VERSION,
        };
        let reference = SyntheticPvtTruthReferenceEpoch {
            position: ValidationReferenceEpoch {
                epoch_idx: 7,
                t_rx_s: Some(100_000.0),
                latitude_deg: 37.0,
                longitude_deg: -122.0,
                altitude_m: 10.0,
                ecef_x_m: Some(truth_ecef.0),
                ecef_y_m: Some(truth_ecef.1),
                ecef_z_m: Some(truth_ecef.2),
                vel_x_mps: None,
                vel_y_mps: None,
                vel_z_mps: None,
            },
            clock_bias_s: truth_clock_bias_s,
        };

        let report =
            validate_truth_guided_pvt_table("unit_test_pvt_truth", &[solution], &[reference]);
        let row = report.epochs.first().expect("pvt truth row");

        assert_eq!(report.solution_count, 1);
        assert_eq!(report.matched_epoch_count, 1);
        assert!(report.unmatched_solution_epochs.is_empty());
        assert!(report.unused_reference_epochs.is_empty());
        assert_eq!(row.truth_ecef_m.x_m, truth_ecef.0);
        assert_eq!(row.measured_ecef_m.y_m, measured_ecef.1);
        assert_eq!(row.ecef_error_m.x_m, 1.5);
        assert_eq!(row.ecef_error_m.y_m, -2.0);
        assert_eq!(row.ecef_error_m.z_m, 0.75);
        assert_eq!(row.truth_geodetic.latitude_deg, 37.0);
        assert_eq!(row.truth_geodetic.longitude_deg, -122.0);
        assert_eq!(row.truth_geodetic.altitude_m, 10.0);
        assert_eq!(row.clock_bias.truth_s, truth_clock_bias_s);
        assert_eq!(row.clock_bias.measured_s, measured_clock_bias_s);
        assert_eq!(row.clock_bias.error_s, measured_clock_bias_s - truth_clock_bias_s);
        assert!(
            (row.clock_bias.error_m
                - (measured_clock_bias_s - truth_clock_bias_s) * SPEED_OF_LIGHT_MPS)
                .abs()
                <= 1.0e-9
        );
        assert_eq!(row.residual_rms_m, 1.25);
        assert_eq!(row.pre_fit_residual_rms_m, Some(3.5));
        assert_eq!(row.post_fit_residual_rms_m, Some(1.25));
        assert_eq!(row.dop.pdop, 1.2);
        assert_eq!(row.solution_status, SolutionStatus::Converged);
        assert_eq!(row.solution_quality, NavQualityFlag::Float);
        assert_eq!(row.solution_validity, SolutionValidity::Stable);
        assert!(row.valid);
    }

    #[test]
    fn truth_guided_receiver_accuracy_budgets_are_hard_and_positive() {
        let budgets = truth_guided_receiver_accuracy_budgets();

        assert!(budgets.acquisition.max_doppler_error_hz > 0.0);
        assert!(budgets.acquisition.max_code_phase_error_samples > 0);
        assert!(budgets.tracking.max_carrier_error_hz > 0.0);
        assert!(budgets.tracking.max_doppler_error_hz > 0.0);
        assert!(budgets.tracking.max_code_phase_error_samples > 0.0);
        assert!(budgets.tracking.max_cn0_error_db_hz > 0.0);
        assert!(budgets.observation.max_pseudorange_error_m > 0.0);
        assert!(budgets.observation.max_carrier_phase_error_cycles > 0.0);
        assert!(budgets.observation.max_doppler_error_hz > 0.0);
        assert!(budgets.observation.max_cn0_error_db_hz > 0.0);
        assert!(budgets.pvt.max_position_error_3d_m > 0.0);
        assert!(budgets.pvt.max_clock_bias_error_m > 0.0);
        assert!(budgets.pvt.max_residual_rms_m > 0.0);
        assert!(budgets.pvt.max_pdop > 0.0);
    }

    #[test]
    fn acquisition_accuracy_budget_fails_when_truth_error_exceeds_threshold() {
        let report = SyntheticAcquisitionTruthTableReport {
            scenario_id: "acquisition_budget_failure".to_string(),
            doppler_tolerance_bins: 1,
            doppler_tolerance_hz: 500.0,
            code_phase_tolerance_samples: 2,
            sample_rate_hz: 1_023_000.0,
            period_samples: 1023,
            doppler_step_hz: 500,
            pass: true,
            satellites: vec![SyntheticAcquisitionTruthTableSatellite {
                sat: SatId { constellation: Constellation::Gps, prn: 3 },
                injected_doppler_hz: 750.0,
                expected_measured_doppler_hz: 750.0,
                measured_doppler_hz: 1_500.0,
                doppler_error_hz: 750.0,
                doppler_error_bins: 1.5,
                injected_code_phase_chips: 200.25,
                expected_code_phase_samples: 100,
                measured_code_phase_samples: 104,
                code_phase_error_samples: 4,
                peak_mean_ratio: 10.0,
                hypothesis: "accepted".to_string(),
                doppler_pass: false,
                code_phase_pass: false,
                pass: false,
            }],
        };

        let accuracy = validate_acquisition_accuracy_budget(
            &report,
            truth_guided_receiver_accuracy_budgets().acquisition,
        );
        let satellite = accuracy.satellites.first().expect("acquisition satellite");

        assert!(!accuracy.pass);
        assert_eq!(accuracy.passing_satellite_count, 0);
        assert!(!satellite.pass);
    }

    #[test]
    fn acquisition_accuracy_budget_requires_truth_satellites() {
        let report = SyntheticAcquisitionTruthTableReport {
            scenario_id: "acquisition_missing_truth".to_string(),
            doppler_tolerance_bins: 1,
            doppler_tolerance_hz: 500.0,
            code_phase_tolerance_samples: 2,
            sample_rate_hz: 1_023_000.0,
            period_samples: 1023,
            doppler_step_hz: 500,
            pass: false,
            satellites: Vec::new(),
        };

        let accuracy = validate_acquisition_accuracy_budget(
            &report,
            truth_guided_receiver_accuracy_budgets().acquisition,
        );

        assert!(!accuracy.truth_coverage_ready);
        assert_eq!(accuracy.truth_coverage_issues.len(), 1);
        assert_eq!(accuracy.truth_coverage_issues[0].code, "no_truth_satellites");
        assert!(!accuracy.pass);
    }

    #[test]
    fn tracking_accuracy_budget_requires_stable_truth_epochs() {
        let report = SyntheticTrackingTruthTableReport {
            scenario_id: "tracking_missing_truth".to_string(),
            carrier_tolerance_hz: 10.0,
            doppler_tolerance_hz: 10.0,
            code_phase_tolerance_samples: 1.0,
            cn0_tolerance_db_hz: 8.0,
            sample_rate_hz: 1_023_000.0,
            period_samples: 1023,
            output_scale_applied: 1.0,
            pass: false,
            satellites: vec![SyntheticTrackingTruthTableSatellite {
                sat: SatId { constellation: Constellation::Gps, prn: 7 },
                injected_doppler_hz: -250.0,
                expected_measured_doppler_hz: -250.0,
                injected_code_phase_chips: 100.0,
                injected_cn0_db_hz: 45.0,
                epoch_count: 1,
                stable_epoch_count: 0,
                first_stable_epoch_index: None,
                pass: false,
                epochs: vec![SyntheticTrackingTruthTableEpoch {
                    epoch_index: 0,
                    sample_index: 0,
                    expected_carrier_hz: -250.0,
                    measured_carrier_hz: -250.0,
                    carrier_error_hz: 0.0,
                    expected_doppler_hz: -250.0,
                    measured_doppler_hz: -250.0,
                    doppler_error_hz: 0.0,
                    expected_code_phase_samples: 100.0,
                    measured_code_phase_samples: 100.0,
                    code_phase_error_samples: 0.0,
                    expected_cn0_db_hz: 45.0,
                    measured_cn0_dbhz: 45.0,
                    cn0_error_db: 0.0,
                    lock: false,
                    pll_lock: false,
                    dll_lock: false,
                    fll_lock: false,
                    cycle_slip: false,
                    lock_state: "degraded".to_string(),
                    lock_state_reason: Some("no_stable_truth_window".to_string()),
                    stable_tracking_epoch: false,
                    pass: false,
                }],
            }],
        };

        let accuracy = super::validate_tracking_accuracy_budget(
            &report,
            truth_guided_receiver_accuracy_budgets().tracking,
        );

        assert!(!accuracy.truth_coverage_ready);
        assert_eq!(accuracy.truth_coverage_issues.len(), 1);
        assert_eq!(accuracy.truth_coverage_issues[0].sat, Some(report.satellites[0].sat));
        assert_eq!(accuracy.truth_coverage_issues[0].code, "no_stable_tracking_truth_epochs");
        assert!(!accuracy.pass);
    }

    #[test]
    fn pvt_accuracy_budget_fails_invalid_or_out_of_budget_epoch() {
        let report = SyntheticPvtTruthTableReport {
            scenario_id: "pvt_budget_failure".to_string(),
            solution_count: 1,
            matched_epoch_count: 1,
            unmatched_solution_epochs: Vec::new(),
            unused_reference_epochs: Vec::new(),
            epochs: vec![SyntheticPvtTruthTableEpoch {
                artifact_id: "nav-epoch-invalid".to_string(),
                source_observation_epoch_id: "obs-epoch-invalid".to_string(),
                epoch_index: 9,
                receive_time_s: 100_000.0,
                truth_ecef_m: SyntheticPvtTruthTableEcef { x_m: 0.0, y_m: 0.0, z_m: 0.0 },
                measured_ecef_m: SyntheticPvtTruthTableEcef { x_m: 0.0, y_m: 0.0, z_m: 6.0 },
                ecef_error_m: SyntheticPvtTruthTableEcef { x_m: 0.0, y_m: 0.0, z_m: 6.0 },
                truth_geodetic: SyntheticPvtTruthTableGeodetic {
                    latitude_deg: 0.0,
                    longitude_deg: 0.0,
                    altitude_m: 0.0,
                },
                measured_geodetic: SyntheticPvtTruthTableGeodetic {
                    latitude_deg: 0.0,
                    longitude_deg: 0.0,
                    altitude_m: 6.0,
                },
                enu_error_m: SyntheticPvtTruthTableEnuError {
                    east_m: 0.0,
                    north_m: 0.0,
                    up_m: 6.0,
                    horiz_m: 0.0,
                    vert_m: 6.0,
                    error_3d_m: 6.0,
                },
                clock_bias: SyntheticPvtTruthTableClockBias {
                    truth_s: 0.0,
                    measured_s: 1.0e-6,
                    error_s: 1.0e-6,
                    truth_m: 0.0,
                    measured_m: 10.0,
                    error_m: 10.0,
                },
                residual_rms_m: 2.0,
                pre_fit_residual_rms_m: Some(2.0),
                post_fit_residual_rms_m: Some(2.0),
                dop: SyntheticPvtTruthTableDop {
                    pdop: 4.0,
                    hdop: Some(2.0),
                    vdop: Some(2.0),
                    gdop: Some(4.5),
                    tdop: Some(1.0),
                },
                solution_status: SolutionStatus::Invalid,
                solution_quality: NavQualityFlag::NoFix,
                solution_validity: SolutionValidity::Invalid,
                valid: false,
                sat_count: 4,
                used_sat_count: 4,
                rejected_sat_count: 0,
            }],
        };

        let accuracy =
            validate_pvt_accuracy_budget(&report, truth_guided_receiver_accuracy_budgets().pvt);
        let epoch = accuracy.epochs.first().expect("pvt epoch");

        assert!(!accuracy.pass);
        assert_eq!(accuracy.passing_epoch_count, 0);
        assert!(!epoch.pass);
    }
