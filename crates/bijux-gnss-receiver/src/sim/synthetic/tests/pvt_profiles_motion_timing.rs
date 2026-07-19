    #[test]
    fn pvt_motion_profile_sorts_cases_by_truth_path_length() {
        let static_truth = SyntheticPvtTruthTableReport {
            scenario_id: "pvt_motion_profile_static".to_string(),
            solution_count: 2,
            matched_epoch_count: 2,
            unmatched_solution_epochs: Vec::new(),
            unused_reference_epochs: Vec::new(),
            epochs: vec![
                SyntheticPvtTruthTableEpoch {
                    artifact_id: "static-artifact-1".to_string(),
                    source_observation_epoch_id: "static-source-1".to_string(),
                    epoch_index: 1,
                    receive_time_s: 100.0,
                    truth_ecef_m: SyntheticPvtTruthTableEcef { x_m: 0.0, y_m: 0.0, z_m: 0.0 },
                    measured_ecef_m: SyntheticPvtTruthTableEcef { x_m: 0.2, y_m: 0.0, z_m: 0.0 },
                    position_covariance_ecef_m2: None,
                    ecef_error_m: SyntheticPvtTruthTableEcef { x_m: 0.2, y_m: 0.0, z_m: 0.0 },
                    truth_geodetic: SyntheticPvtTruthTableGeodetic {
                        latitude_deg: 0.0,
                        longitude_deg: 0.0,
                        altitude_m: 0.0,
                    },
                    measured_geodetic: SyntheticPvtTruthTableGeodetic {
                        latitude_deg: 0.0,
                        longitude_deg: 0.0,
                        altitude_m: 0.2,
                    },
                    enu_error_m: SyntheticPvtTruthTableEnuError {
                        east_m: 0.2,
                        north_m: 0.0,
                        up_m: 0.0,
                        horiz_m: 0.2,
                        vert_m: 0.0,
                        error_3d_m: 0.2,
                    },
                    clock_bias: SyntheticPvtTruthTableClockBias {
                        truth_s: 0.0,
                        measured_s: 0.0,
                        error_s: 0.0,
                        truth_m: 0.0,
                        measured_m: 0.0,
                        error_m: 0.0,
                    },
                    residual_rms_m: 0.4,
                    pre_fit_residual_rms_m: Some(0.4),
                    post_fit_residual_rms_m: Some(0.4),
                    dop: SyntheticPvtTruthTableDop {
                        pdop: 1.4,
                        hdop: Some(1.0),
                        vdop: Some(0.9),
                        gdop: Some(1.6),
                        tdop: Some(0.5),
                    },
                    solution_status: SolutionStatus::CodeOnly,
                    solution_quality: NavQualityFlag::Float,
                    solution_validity: SolutionValidity::Stable,
                    valid: true,
                    sat_count: 5,
                    used_sat_count: 5,
                    rejected_sat_count: 0,
                },
                SyntheticPvtTruthTableEpoch {
                    artifact_id: "static-artifact-2".to_string(),
                    source_observation_epoch_id: "static-source-2".to_string(),
                    epoch_index: 2,
                    receive_time_s: 101.0,
                    truth_ecef_m: SyntheticPvtTruthTableEcef { x_m: 0.0, y_m: 0.0, z_m: 0.0 },
                    measured_ecef_m: SyntheticPvtTruthTableEcef { x_m: 0.3, y_m: 0.0, z_m: 0.0 },
                    position_covariance_ecef_m2: None,
                    ecef_error_m: SyntheticPvtTruthTableEcef { x_m: 0.3, y_m: 0.0, z_m: 0.0 },
                    truth_geodetic: SyntheticPvtTruthTableGeodetic {
                        latitude_deg: 0.0,
                        longitude_deg: 0.0,
                        altitude_m: 0.0,
                    },
                    measured_geodetic: SyntheticPvtTruthTableGeodetic {
                        latitude_deg: 0.0,
                        longitude_deg: 0.0,
                        altitude_m: 0.3,
                    },
                    enu_error_m: SyntheticPvtTruthTableEnuError {
                        east_m: 0.3,
                        north_m: 0.0,
                        up_m: 0.0,
                        horiz_m: 0.3,
                        vert_m: 0.0,
                        error_3d_m: 0.3,
                    },
                    clock_bias: SyntheticPvtTruthTableClockBias {
                        truth_s: 0.0,
                        measured_s: 0.0,
                        error_s: 0.0,
                        truth_m: 0.0,
                        measured_m: 0.0,
                        error_m: 0.0,
                    },
                    residual_rms_m: 0.45,
                    pre_fit_residual_rms_m: Some(0.45),
                    post_fit_residual_rms_m: Some(0.45),
                    dop: SyntheticPvtTruthTableDop {
                        pdop: 1.4,
                        hdop: Some(1.0),
                        vdop: Some(0.9),
                        gdop: Some(1.6),
                        tdop: Some(0.5),
                    },
                    solution_status: SolutionStatus::CodeOnly,
                    solution_quality: NavQualityFlag::Float,
                    solution_validity: SolutionValidity::Stable,
                    valid: true,
                    sat_count: 5,
                    used_sat_count: 5,
                    rejected_sat_count: 0,
                },
            ],
        };
        let moving_truth = SyntheticPvtTruthTableReport {
            scenario_id: "pvt_motion_profile_linear_motion".to_string(),
            solution_count: 2,
            matched_epoch_count: 2,
            unmatched_solution_epochs: Vec::new(),
            unused_reference_epochs: Vec::new(),
            epochs: vec![
                SyntheticPvtTruthTableEpoch {
                    artifact_id: "moving-artifact-1".to_string(),
                    source_observation_epoch_id: "moving-source-1".to_string(),
                    epoch_index: 1,
                    receive_time_s: 100.0,
                    truth_ecef_m: SyntheticPvtTruthTableEcef { x_m: 0.0, y_m: 0.0, z_m: 0.0 },
                    measured_ecef_m: SyntheticPvtTruthTableEcef { x_m: 0.2, y_m: 0.0, z_m: 0.0 },
                    position_covariance_ecef_m2: None,
                    ecef_error_m: SyntheticPvtTruthTableEcef { x_m: 0.2, y_m: 0.0, z_m: 0.0 },
                    truth_geodetic: SyntheticPvtTruthTableGeodetic {
                        latitude_deg: 0.0,
                        longitude_deg: 0.0,
                        altitude_m: 0.0,
                    },
                    measured_geodetic: SyntheticPvtTruthTableGeodetic {
                        latitude_deg: 0.0,
                        longitude_deg: 0.0,
                        altitude_m: 0.2,
                    },
                    enu_error_m: SyntheticPvtTruthTableEnuError {
                        east_m: 0.2,
                        north_m: 0.0,
                        up_m: 0.0,
                        horiz_m: 0.2,
                        vert_m: 0.0,
                        error_3d_m: 0.2,
                    },
                    clock_bias: SyntheticPvtTruthTableClockBias {
                        truth_s: 0.0,
                        measured_s: 0.0,
                        error_s: 0.0,
                        truth_m: 0.0,
                        measured_m: 0.0,
                        error_m: 0.0,
                    },
                    residual_rms_m: 0.4,
                    pre_fit_residual_rms_m: Some(0.4),
                    post_fit_residual_rms_m: Some(0.4),
                    dop: SyntheticPvtTruthTableDop {
                        pdop: 1.5,
                        hdop: Some(1.1),
                        vdop: Some(0.9),
                        gdop: Some(1.7),
                        tdop: Some(0.5),
                    },
                    solution_status: SolutionStatus::CodeOnly,
                    solution_quality: NavQualityFlag::Float,
                    solution_validity: SolutionValidity::Stable,
                    valid: true,
                    sat_count: 5,
                    used_sat_count: 5,
                    rejected_sat_count: 0,
                },
                SyntheticPvtTruthTableEpoch {
                    artifact_id: "moving-artifact-2".to_string(),
                    source_observation_epoch_id: "moving-source-2".to_string(),
                    epoch_index: 2,
                    receive_time_s: 101.0,
                    truth_ecef_m: SyntheticPvtTruthTableEcef { x_m: 10.0, y_m: 0.0, z_m: 0.0 },
                    measured_ecef_m: SyntheticPvtTruthTableEcef { x_m: 10.5, y_m: 0.0, z_m: 0.0 },
                    position_covariance_ecef_m2: None,
                    ecef_error_m: SyntheticPvtTruthTableEcef { x_m: 0.5, y_m: 0.0, z_m: 0.0 },
                    truth_geodetic: SyntheticPvtTruthTableGeodetic {
                        latitude_deg: 0.0,
                        longitude_deg: 0.0,
                        altitude_m: 0.0,
                    },
                    measured_geodetic: SyntheticPvtTruthTableGeodetic {
                        latitude_deg: 0.0,
                        longitude_deg: 0.0,
                        altitude_m: 0.5,
                    },
                    enu_error_m: SyntheticPvtTruthTableEnuError {
                        east_m: 0.5,
                        north_m: 0.0,
                        up_m: 0.0,
                        horiz_m: 0.5,
                        vert_m: 0.0,
                        error_3d_m: 0.5,
                    },
                    clock_bias: SyntheticPvtTruthTableClockBias {
                        truth_s: 0.0,
                        measured_s: 0.0,
                        error_s: 0.0,
                        truth_m: 0.0,
                        measured_m: 0.0,
                        error_m: 0.0,
                    },
                    residual_rms_m: 0.5,
                    pre_fit_residual_rms_m: Some(0.5),
                    post_fit_residual_rms_m: Some(0.5),
                    dop: SyntheticPvtTruthTableDop {
                        pdop: 1.5,
                        hdop: Some(1.1),
                        vdop: Some(0.9),
                        gdop: Some(1.7),
                        tdop: Some(0.5),
                    },
                    solution_status: SolutionStatus::CodeOnly,
                    solution_quality: NavQualityFlag::Float,
                    solution_validity: SolutionValidity::Stable,
                    valid: true,
                    sat_count: 5,
                    used_sat_count: 5,
                    rejected_sat_count: 0,
                },
            ],
        };
        let static_accuracy = SyntheticPvtAccuracyReport {
            scenario_id: "pvt_motion_profile_static".to_string(),
            max_position_error_3d_m: 5.0,
            max_clock_bias_error_m: 10.0,
            max_residual_rms_m: 4.0,
            max_pdop: 6.0,
            epoch_count: 2,
            passing_epoch_count: 2,
            truth_coverage_ready: true,
            truth_coverage_issues: Vec::new(),
            pass: true,
            epochs: vec![
                SyntheticPvtAccuracyEpoch {
                    epoch_index: 1,
                    position_error_3d_m: 0.2,
                    clock_bias_error_m: 0.0,
                    residual_rms_m: 0.4,
                    pdop: 1.4,
                    pass: true,
                },
                SyntheticPvtAccuracyEpoch {
                    epoch_index: 2,
                    position_error_3d_m: 0.3,
                    clock_bias_error_m: 0.0,
                    residual_rms_m: 0.45,
                    pdop: 1.4,
                    pass: true,
                },
            ],
        };
        let moving_accuracy = SyntheticPvtAccuracyReport {
            scenario_id: "pvt_motion_profile_linear_motion".to_string(),
            max_position_error_3d_m: 5.0,
            max_clock_bias_error_m: 10.0,
            max_residual_rms_m: 4.0,
            max_pdop: 6.0,
            epoch_count: 2,
            passing_epoch_count: 2,
            truth_coverage_ready: true,
            truth_coverage_issues: Vec::new(),
            pass: true,
            epochs: vec![
                SyntheticPvtAccuracyEpoch {
                    epoch_index: 1,
                    position_error_3d_m: 0.2,
                    clock_bias_error_m: 0.0,
                    residual_rms_m: 0.4,
                    pdop: 1.5,
                    pass: true,
                },
                SyntheticPvtAccuracyEpoch {
                    epoch_index: 2,
                    position_error_3d_m: 0.5,
                    clock_bias_error_m: 0.0,
                    residual_rms_m: 0.5,
                    pdop: 1.5,
                    pass: true,
                },
            ],
        };

        let report: SyntheticPvtMotionProfileReport = summarize_truth_guided_pvt_motion_profile(
            &[
                SyntheticPvtMotionProfileCase {
                    scenario_id: "pvt_motion_profile_linear_motion",
                    truth_table: &moving_truth,
                    accuracy: &moving_accuracy,
                },
                SyntheticPvtMotionProfileCase {
                    scenario_id: "pvt_motion_profile_static",
                    truth_table: &static_truth,
                    accuracy: &static_accuracy,
                },
            ],
            "pvt_motion_profile",
        );

        assert_eq!(report.points.len(), 2);
        assert_eq!(report.points[0].scenario_id, "pvt_motion_profile_static");
        assert!(!report.points[0].moving);
        assert_eq!(report.points[0].path_length_m, 0.0);
        assert_eq!(report.points[1].scenario_id, "pvt_motion_profile_linear_motion");
        assert!(report.points[1].moving);
        assert_eq!(report.points[1].path_length_m, 10.0);
        assert_eq!(report.points[1].mean_speed_mps, 10.0);
        assert_eq!(report.points[1].stable_epoch_count, 2);
        assert!(report.points[1].ready);
    }

    #[test]
    fn pvt_clock_profile_sorts_cases_by_injected_drift_and_doppler_offset() {
        let stable_observations = vec![clock_profile_obs_epoch(1, &[100.0])];
        let drifting_observations = vec![clock_profile_obs_epoch(1, &[50.0])];
        let stable_accuracy = clock_profile_accuracy_report("pvt_clock_profile_stable", 1, 0.0);
        let drifting_accuracy =
            clock_profile_accuracy_report("pvt_clock_profile_drifting", 1, 15.0);
        let stable_solutions = vec![clock_profile_solution(1, 0.0)];
        let drifting_solutions = vec![clock_profile_solution(1, 5.0e-6)];

        let report = summarize_truth_guided_pvt_clock_profile(
            &[
                SyntheticPvtClockProfileCase {
                    scenario_id: "pvt_clock_profile_drifting",
                    injected_clock_drift_s_per_s: 5.0e-6,
                    expected_observation_doppler_offset_hz: -50.0,
                    observations: &drifting_observations,
                    reference_observations: Some(&stable_observations),
                    solutions: &drifting_solutions,
                    accuracy: &drifting_accuracy,
                },
                SyntheticPvtClockProfileCase {
                    scenario_id: "pvt_clock_profile_stable",
                    injected_clock_drift_s_per_s: 0.0,
                    expected_observation_doppler_offset_hz: 0.0,
                    observations: &stable_observations,
                    reference_observations: None,
                    solutions: &stable_solutions,
                    accuracy: &stable_accuracy,
                },
            ],
            "pvt_clock_profile",
        );

        assert_eq!(report.points.len(), 2);
        assert_eq!(report.points[0].scenario_id, "pvt_clock_profile_stable");
        assert_eq!(report.points[1].scenario_id, "pvt_clock_profile_drifting");
        assert_eq!(report.points[1].observed_mean_observation_doppler_offset_hz, Some(-50.0));
        assert_eq!(report.points[1].max_observation_doppler_offset_error_hz, Some(0.0));
        assert!(report.points[1].ready);
    }

    #[test]
    fn pvt_clock_profile_reports_reference_support_failures() {
        let drifting_observations = vec![clock_profile_obs_epoch(1, &[50.0])];
        let shifted_reference_observations = vec![clock_profile_obs_epoch(7, &[100.0])];
        let drifting_accuracy =
            clock_profile_accuracy_report("pvt_clock_profile_reference_support", 1, 15.0);
        let drifting_solutions = vec![clock_profile_solution(1, 5.0e-6)];

        let report = summarize_truth_guided_pvt_clock_profile(
            &[
                SyntheticPvtClockProfileCase {
                    scenario_id: "pvt_clock_profile_missing_reference",
                    injected_clock_drift_s_per_s: 5.0e-6,
                    expected_observation_doppler_offset_hz: -50.0,
                    observations: &drifting_observations,
                    reference_observations: None,
                    solutions: &drifting_solutions,
                    accuracy: &drifting_accuracy,
                },
                SyntheticPvtClockProfileCase {
                    scenario_id: "pvt_clock_profile_unmatched_reference",
                    injected_clock_drift_s_per_s: 5.0e-6,
                    expected_observation_doppler_offset_hz: -50.0,
                    observations: &drifting_observations,
                    reference_observations: Some(&shifted_reference_observations),
                    solutions: &drifting_solutions,
                    accuracy: &drifting_accuracy,
                },
            ],
            "pvt_clock_profile",
        );

        let missing_reference = report
            .points
            .iter()
            .find(|point| point.scenario_id == "pvt_clock_profile_missing_reference")
            .expect("missing reference point");
        let unmatched_reference = report
            .points
            .iter()
            .find(|point| point.scenario_id == "pvt_clock_profile_unmatched_reference")
            .expect("unmatched reference point");

        assert!(!missing_reference.truth_coverage_ready);
        assert!(missing_reference
            .truth_coverage_issues
            .iter()
            .any(|issue| issue.code == "missing_clock_profile_reference_observations"));
        assert!(!missing_reference.ready);
        assert!(!unmatched_reference.truth_coverage_ready);
        assert_eq!(unmatched_reference.observation_doppler_pair_count, 0);
        assert!(unmatched_reference
            .truth_coverage_issues
            .iter()
            .any(|issue| issue.code == "no_clock_profile_observation_doppler_pairs"));
        assert!(!unmatched_reference.ready);
    }

    fn time_profile_truth_and_accuracy(
        scenario_id: &str,
        position_error_3d_m: &[f64],
        residual_rms_m: &[f64],
        validities: &[SolutionValidity],
    ) -> (SyntheticPvtTruthTableReport, SyntheticPvtAccuracyReport) {
        assert_eq!(position_error_3d_m.len(), residual_rms_m.len());
        assert_eq!(position_error_3d_m.len(), validities.len());

        let truth = SyntheticPvtTruthTableReport {
            scenario_id: scenario_id.to_string(),
            solution_count: position_error_3d_m.len(),
            matched_epoch_count: position_error_3d_m.len(),
            unmatched_solution_epochs: Vec::new(),
            unused_reference_epochs: Vec::new(),
            epochs: position_error_3d_m
                .iter()
                .zip(residual_rms_m.iter())
                .zip(validities.iter())
                .enumerate()
                .map(|(index, ((position_error_3d_m, residual_rms_m), validity))| {
                    SyntheticPvtTruthTableEpoch {
                        artifact_id: format!("{scenario_id}-artifact-{index}"),
                        source_observation_epoch_id: format!("{scenario_id}-source-{index}"),
                        epoch_index: index as u64,
                        receive_time_s: 100.0 + index as f64,
                        truth_ecef_m: SyntheticPvtTruthTableEcef { x_m: 0.0, y_m: 0.0, z_m: 0.0 },
                        measured_ecef_m: SyntheticPvtTruthTableEcef {
                            x_m: *position_error_3d_m,
                            y_m: 0.0,
                            z_m: 0.0,
                        },
                        position_covariance_ecef_m2: None,
                        ecef_error_m: SyntheticPvtTruthTableEcef {
                            x_m: *position_error_3d_m,
                            y_m: 0.0,
                            z_m: 0.0,
                        },
                        truth_geodetic: SyntheticPvtTruthTableGeodetic {
                            latitude_deg: 0.0,
                            longitude_deg: 0.0,
                            altitude_m: 0.0,
                        },
                        measured_geodetic: SyntheticPvtTruthTableGeodetic {
                            latitude_deg: 0.0,
                            longitude_deg: 0.0,
                            altitude_m: *position_error_3d_m,
                        },
                        enu_error_m: SyntheticPvtTruthTableEnuError {
                            east_m: *position_error_3d_m,
                            north_m: 0.0,
                            up_m: 0.0,
                            horiz_m: *position_error_3d_m,
                            vert_m: 0.0,
                            error_3d_m: *position_error_3d_m,
                        },
                        clock_bias: SyntheticPvtTruthTableClockBias {
                            truth_s: 0.0,
                            measured_s: 0.0,
                            error_s: 0.0,
                            truth_m: 0.0,
                            measured_m: 0.0,
                            error_m: 0.0,
                        },
                        residual_rms_m: *residual_rms_m,
                        pre_fit_residual_rms_m: Some(*residual_rms_m),
                        post_fit_residual_rms_m: Some(*residual_rms_m),
                        dop: SyntheticPvtTruthTableDop {
                            pdop: 1.5,
                            hdop: Some(1.1),
                            vdop: Some(0.9),
                            gdop: Some(1.7),
                            tdop: Some(0.5),
                        },
                        solution_status: SolutionStatus::CodeOnly,
                        solution_quality: NavQualityFlag::Float,
                        solution_validity: *validity,
                        valid: *validity != SolutionValidity::Diverging,
                        sat_count: 5,
                        used_sat_count: 5,
                        rejected_sat_count: 0,
                    }
                })
                .collect(),
        };
        let accuracy = SyntheticPvtAccuracyReport {
            scenario_id: scenario_id.to_string(),
            max_position_error_3d_m: 50.0,
            max_clock_bias_error_m: 10.0,
            max_residual_rms_m: 50.0,
            max_pdop: 6.0,
            epoch_count: position_error_3d_m.len(),
            passing_epoch_count: position_error_3d_m.len(),
            truth_coverage_ready: true,
            truth_coverage_issues: Vec::new(),
            pass: true,
            epochs: position_error_3d_m
                .iter()
                .zip(residual_rms_m.iter())
                .enumerate()
                .map(|(index, (position_error_3d_m, residual_rms_m))| SyntheticPvtAccuracyEpoch {
                    epoch_index: index as u64,
                    position_error_3d_m: *position_error_3d_m,
                    clock_bias_error_m: 0.0,
                    residual_rms_m: *residual_rms_m,
                    pdop: 1.5,
                    pass: true,
                })
                .collect(),
        };

        (truth, accuracy)
    }

    #[test]
    fn pvt_time_profile_classifies_stabilizing_drifting_and_diverging_runs() {
        let (stabilizing_truth, stabilizing_accuracy) = time_profile_truth_and_accuracy(
            "pvt_time_profile_stabilizing",
            &[5.0, 3.5, 2.0, 1.2, 0.8],
            &[4.0, 2.8, 1.9, 1.1, 0.7],
            &[
                SolutionValidity::Coarse,
                SolutionValidity::Converging,
                SolutionValidity::Stable,
                SolutionValidity::Stable,
                SolutionValidity::Stable,
            ],
        );
        let (drifting_truth, drifting_accuracy) = time_profile_truth_and_accuracy(
            "pvt_time_profile_drifting",
            &[0.4, 0.8, 1.2, 1.8, 2.6],
            &[0.3, 0.6, 0.9, 1.3, 1.8],
            &[SolutionValidity::Stable; 5],
        );
        let (diverging_truth, diverging_accuracy) = time_profile_truth_and_accuracy(
            "pvt_time_profile_diverging",
            &[1.0, 4.0, 12.0, 40.0, 95.0],
            &[1.0, 4.5, 15.0, 45.0, 110.0],
            &[
                SolutionValidity::Stable,
                SolutionValidity::Converging,
                SolutionValidity::Coarse,
                SolutionValidity::Diverging,
                SolutionValidity::Diverging,
            ],
        );

        let report: SyntheticPvtTimeProfileReport = summarize_truth_guided_pvt_time_profile(
            &[
                SyntheticPvtTimeProfileCase {
                    scenario_id: "pvt_time_profile_diverging",
                    truth_table: &diverging_truth,
                    accuracy: &diverging_accuracy,
                },
                SyntheticPvtTimeProfileCase {
                    scenario_id: "pvt_time_profile_stabilizing",
                    truth_table: &stabilizing_truth,
                    accuracy: &stabilizing_accuracy,
                },
                SyntheticPvtTimeProfileCase {
                    scenario_id: "pvt_time_profile_drifting",
                    truth_table: &drifting_truth,
                    accuracy: &drifting_accuracy,
                },
            ],
            "pvt_time_profile",
        );

        assert_eq!(report.points.len(), 3);
        assert_eq!(report.points[0].duration_s, 4.0);
        let stabilizing = report
            .points
            .iter()
            .find(|point| point.scenario_id == "pvt_time_profile_stabilizing")
            .expect("stabilizing time profile point");
        let drifting = report
            .points
            .iter()
            .find(|point| point.scenario_id == "pvt_time_profile_drifting")
            .expect("drifting time profile point");
        let diverging = report
            .points
            .iter()
            .find(|point| point.scenario_id == "pvt_time_profile_diverging")
            .expect("diverging time profile point");

        assert_eq!(stabilizing.trend, SyntheticPvtTimeTrend::Stabilizing);
        assert_eq!(drifting.trend, SyntheticPvtTimeTrend::Drifting);
        assert_eq!(diverging.trend, SyntheticPvtTimeTrend::Diverging);
        assert!(stabilizing.ready && drifting.ready && diverging.ready, "{report:?}");
        assert!(stabilizing.position_error_drift_m_per_s.expect("stabilizing slope") < 0.0);
        assert!(drifting.position_error_drift_m_per_s.expect("drifting slope") > 0.0);
        assert!(diverging.diverging_epoch_count > 0);
    }

    #[test]
    fn pvt_time_profile_reports_growth_with_distinct_analysis_windows() {
        let (truth, accuracy) = time_profile_truth_and_accuracy(
            "pvt_time_profile_growth_windows",
            &[0.5, 0.8, 1.3, 2.1, 3.4],
            &[0.3, 0.5, 0.8, 1.2, 1.7],
            &[SolutionValidity::Stable; 5],
        );

        let report = summarize_truth_guided_pvt_time_profile(
            &[SyntheticPvtTimeProfileCase {
                scenario_id: "pvt_time_profile_growth_windows",
                truth_table: &truth,
                accuracy: &accuracy,
            }],
            "pvt_time_profile",
        );
        let point = report.points.first().expect("time profile point");

        assert_eq!(point.analysis_window_epoch_count, 2);
        assert_eq!(point.first_window_mean_position_error_3d_m, Some(0.65));
        assert_eq!(point.last_window_mean_position_error_3d_m, Some(2.75));
        assert_eq!(point.position_error_growth_m, Some(2.1));
        assert_eq!(point.first_window_mean_residual_rms_m, Some(0.4));
        assert_eq!(point.last_window_mean_residual_rms_m, Some(1.45));
        assert!(
            (point.residual_rms_growth_m.expect("residual growth") - 1.05).abs() <= 1.0e-12,
            "{report:?}"
        );
        assert!(point.ready, "{report:?}");
    }

    #[test]
    fn pvt_time_profile_reports_insufficient_long_run_evidence() {
        let (truth, accuracy) = time_profile_truth_and_accuracy(
            "pvt_time_profile_short_run",
            &[0.5, 0.7, 1.0, 1.4],
            &[0.3, 0.4, 0.6, 0.8],
            &[SolutionValidity::Stable; 4],
        );

        let report = summarize_truth_guided_pvt_time_profile(
            &[SyntheticPvtTimeProfileCase {
                scenario_id: "pvt_time_profile_short_run",
                truth_table: &truth,
                accuracy: &accuracy,
            }],
            "pvt_time_profile",
        );
        let point = report.points.first().expect("time profile point");

        assert!(!point.truth_coverage_ready, "{report:?}");
        assert!(point
            .truth_coverage_issues
            .iter()
            .any(|issue| issue.code == "insufficient_time_profile_truth_epochs"));
        assert!(point
            .truth_coverage_issues
            .iter()
            .any(|issue| issue.code == "insufficient_time_profile_accuracy_epochs"));
        assert!(!point.ready, "{report:?}");
    }
