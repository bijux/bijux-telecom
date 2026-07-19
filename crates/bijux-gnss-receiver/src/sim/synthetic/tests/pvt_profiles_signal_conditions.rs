    #[test]
    fn pvt_cn0_profile_summarizes_observation_levels_and_epoch_quality() {
        let weak_observations =
            vec![sample_obs_epoch(4, &[24.0, 26.0]), sample_obs_epoch(5, &[28.0, 30.0])];
        let strong_observations =
            vec![sample_obs_epoch(6, &[40.0, 42.0]), sample_obs_epoch(7, &[44.0, 46.0])];
        let weak_accuracy = SyntheticPvtAccuracyReport {
            scenario_id: "pvt_cn0_profile_weak".to_string(),
            max_position_error_3d_m: 5.0,
            max_clock_bias_error_m: 10.0,
            max_residual_rms_m: 4.0,
            max_pdop: 6.0,
            epoch_count: 2,
            passing_epoch_count: 1,
            truth_coverage_ready: true,
            truth_coverage_issues: Vec::new(),
            pass: false,
            epochs: vec![
                SyntheticPvtAccuracyEpoch {
                    epoch_index: 4,
                    position_error_3d_m: 2.0,
                    clock_bias_error_m: 1.0,
                    residual_rms_m: 0.5,
                    pdop: 2.0,
                    pass: true,
                },
                SyntheticPvtAccuracyEpoch {
                    epoch_index: 5,
                    position_error_3d_m: 4.0,
                    clock_bias_error_m: 3.0,
                    residual_rms_m: 1.5,
                    pdop: 3.0,
                    pass: false,
                },
            ],
        };
        let strong_accuracy = SyntheticPvtAccuracyReport {
            scenario_id: "pvt_cn0_profile_strong".to_string(),
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
                    epoch_index: 6,
                    position_error_3d_m: 1.0,
                    clock_bias_error_m: 0.5,
                    residual_rms_m: 0.25,
                    pdop: 1.5,
                    pass: true,
                },
                SyntheticPvtAccuracyEpoch {
                    epoch_index: 7,
                    position_error_3d_m: 1.5,
                    clock_bias_error_m: 0.75,
                    residual_rms_m: 0.5,
                    pdop: 1.75,
                    pass: true,
                },
            ],
        };

        let report = summarize_truth_guided_pvt_cn0_profile(
            &[
                SyntheticPvtCn0ProfileCase {
                    scenario_id: "pvt_cn0_profile_strong",
                    observations: &strong_observations,
                    accuracy: &strong_accuracy,
                },
                SyntheticPvtCn0ProfileCase {
                    scenario_id: "pvt_cn0_profile_weak",
                    observations: &weak_observations,
                    accuracy: &weak_accuracy,
                },
            ],
            "pvt_cn0_profile",
        );

        assert_eq!(report.points.len(), 2);
        assert_eq!(report.points[0].scenario_id, "pvt_cn0_profile_weak");
        assert!((report.points[0].mean_observation_cn0_dbhz - 27.0).abs() <= 1.0e-12);
        assert_eq!(report.points[0].min_observation_cn0_dbhz, 25.0);
        assert_eq!(report.points[0].max_observation_cn0_dbhz, 29.0);
        assert_eq!(report.points[0].passing_epoch_count, 1);
        assert!((report.points[0].pass_rate - 0.5).abs() <= 1.0e-12);
        assert_eq!(report.points[0].max_position_error_3d_m, Some(4.0));
        assert!(report.points[0].ready);
        assert_eq!(report.points[1].scenario_id, "pvt_cn0_profile_strong");
        assert!((report.points[1].mean_observation_cn0_dbhz - 43.0).abs() <= 1.0e-12);
        assert_eq!(report.points[1].passing_epoch_count, 2);
        assert!((report.points[1].pass_rate - 1.0).abs() <= 1.0e-12);
        assert_eq!(report.points[1].max_position_error_3d_m, Some(1.5));
    }

    #[test]
    fn accuracy_cn0_profile_merges_stage_metrics_by_signal_level() {
        let acquisition = SyntheticAcquisitionDetectionRateReport {
            scenario_id_prefix: "accuracy_cn0_profile".to_string(),
            code_phase_tolerance_samples: 2,
            doppler_tolerance_bins: 1,
            doppler_step_hz: 500,
            points: vec![
                SyntheticAcquisitionDetectionRatePoint {
                    sat: SatId { constellation: Constellation::Gps, prn: 7 },
                    cn0_db_hz: 27.0,
                    doppler_hz: 250.0,
                    coherent_ms: 1,
                    noncoherent: 1,
                    trial_count: 4,
                    accepted_count: 2,
                    detected_count: 1,
                    acceptance_probability: 0.5,
                    detection_probability: 0.25,
                    mean_peak_mean_ratio: 3.0,
                },
                SyntheticAcquisitionDetectionRatePoint {
                    sat: SatId { constellation: Constellation::Gps, prn: 7 },
                    cn0_db_hz: 43.0,
                    doppler_hz: 250.0,
                    coherent_ms: 1,
                    noncoherent: 1,
                    trial_count: 4,
                    accepted_count: 4,
                    detected_count: 4,
                    acceptance_probability: 1.0,
                    detection_probability: 1.0,
                    mean_peak_mean_ratio: 12.0,
                },
            ],
        };
        let tracking = SyntheticTrackingLockRateReport {
            scenario_id_prefix: "accuracy_cn0_profile".to_string(),
            points: vec![
                SyntheticTrackingLockRatePoint {
                    sat: SatId { constellation: Constellation::Gps, prn: 7 },
                    cn0_db_hz: 27.0,
                    duration_s: 0.08,
                    seeded_doppler_error_hz: 60.0,
                    seeded_code_phase_error_samples: 1,
                    min_locked_epochs: 4,
                    trial_count: 4,
                    stable_lock_count: 1,
                    refused_lock_count: 2,
                    lock_probability: 0.25,
                    mean_locked_epochs: 2.0,
                },
                SyntheticTrackingLockRatePoint {
                    sat: SatId { constellation: Constellation::Gps, prn: 7 },
                    cn0_db_hz: 43.0,
                    duration_s: 0.08,
                    seeded_doppler_error_hz: 60.0,
                    seeded_code_phase_error_samples: 1,
                    min_locked_epochs: 4,
                    trial_count: 4,
                    stable_lock_count: 4,
                    refused_lock_count: 0,
                    lock_probability: 1.0,
                    mean_locked_epochs: 8.0,
                },
            ],
        };
        let pvt = SyntheticPvtCn0ProfileReport {
            scenario_id_prefix: "accuracy_cn0_profile".to_string(),
            points: vec![
                SyntheticPvtCn0ProfilePoint {
                    scenario_id: "pvt_weak".to_string(),
                    observation_epoch_count: 2,
                    mean_observation_cn0_dbhz: 27.0,
                    min_observation_cn0_dbhz: 25.0,
                    max_observation_cn0_dbhz: 29.0,
                    epoch_count: 2,
                    passing_epoch_count: 1,
                    pass_rate: 0.5,
                    rms_position_error_3d_m: Some(3.0),
                    max_position_error_3d_m: Some(4.0),
                    rms_clock_bias_error_m: Some(2.0),
                    max_clock_bias_error_m: Some(3.0),
                    rms_residual_rms_m: Some(1.0),
                    max_residual_rms_m: Some(1.5),
                    max_pdop: Some(3.0),
                    truth_coverage_ready: true,
                    truth_coverage_issues: Vec::new(),
                    ready: true,
                },
                SyntheticPvtCn0ProfilePoint {
                    scenario_id: "pvt_strong".to_string(),
                    observation_epoch_count: 2,
                    mean_observation_cn0_dbhz: 43.0,
                    min_observation_cn0_dbhz: 41.0,
                    max_observation_cn0_dbhz: 45.0,
                    epoch_count: 2,
                    passing_epoch_count: 2,
                    pass_rate: 1.0,
                    rms_position_error_3d_m: Some(1.25),
                    max_position_error_3d_m: Some(1.5),
                    rms_clock_bias_error_m: Some(0.625),
                    max_clock_bias_error_m: Some(0.75),
                    rms_residual_rms_m: Some(0.375),
                    max_residual_rms_m: Some(0.5),
                    max_pdop: Some(1.75),
                    truth_coverage_ready: true,
                    truth_coverage_issues: Vec::new(),
                    ready: true,
                },
            ],
        };

        let report: SyntheticAccuracyCn0ProfileReport = summarize_truth_guided_accuracy_cn0_profile(
            "accuracy_cn0_profile",
            &acquisition,
            &tracking,
            &pvt,
        );

        assert_eq!(report.points.len(), 2);
        assert_eq!(report.points[0].cn0_db_hz, 27.0);
        assert_eq!(report.points[0].acquisition_case_count, 1);
        assert_eq!(report.points[0].tracking_case_count, 1);
        assert_eq!(report.points[0].pvt_case_count, 1);
        assert_eq!(report.points[0].acquisition_detection_probability_mean, Some(0.25));
        assert_eq!(report.points[0].tracking_lock_probability_mean, Some(0.25));
        assert_eq!(report.points[0].pvt_pass_rate_mean, Some(0.5));
        assert_eq!(report.points[1].cn0_db_hz, 43.0);
        assert_eq!(report.points[1].acquisition_detection_probability_mean, Some(1.0));
        assert_eq!(report.points[1].tracking_lock_probability_mean, Some(1.0));
        assert_eq!(report.points[1].pvt_pass_rate_mean, Some(1.0));
        assert_eq!(report.points[1].pvt_max_position_error_3d_m_max, Some(1.5));
    }

    #[test]
    fn pvt_geometry_profile_sorts_cases_by_mean_pdop() {
        let poor_accuracy = SyntheticPvtAccuracyReport {
            scenario_id: "pvt_geometry_profile_poor".to_string(),
            max_position_error_3d_m: 5.0,
            max_clock_bias_error_m: 10.0,
            max_residual_rms_m: 4.0,
            max_pdop: 6.0,
            epoch_count: 2,
            passing_epoch_count: 1,
            truth_coverage_ready: true,
            truth_coverage_issues: Vec::new(),
            pass: false,
            epochs: vec![
                SyntheticPvtAccuracyEpoch {
                    epoch_index: 1,
                    position_error_3d_m: 3.5,
                    clock_bias_error_m: 1.75,
                    residual_rms_m: 1.2,
                    pdop: 4.0,
                    pass: false,
                },
                SyntheticPvtAccuracyEpoch {
                    epoch_index: 2,
                    position_error_3d_m: 4.5,
                    clock_bias_error_m: 2.25,
                    residual_rms_m: 1.5,
                    pdop: 5.5,
                    pass: true,
                },
            ],
        };
        let good_accuracy = SyntheticPvtAccuracyReport {
            scenario_id: "pvt_geometry_profile_good".to_string(),
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
                    epoch_index: 3,
                    position_error_3d_m: 0.9,
                    clock_bias_error_m: 0.4,
                    residual_rms_m: 0.2,
                    pdop: 1.4,
                    pass: true,
                },
                SyntheticPvtAccuracyEpoch {
                    epoch_index: 4,
                    position_error_3d_m: 1.2,
                    clock_bias_error_m: 0.5,
                    residual_rms_m: 0.25,
                    pdop: 1.8,
                    pass: true,
                },
            ],
        };

        let report: SyntheticPvtGeometryProfileReport = summarize_truth_guided_pvt_geometry_profile(
            &[
                SyntheticPvtGeometryProfileCase {
                    scenario_id: "pvt_geometry_profile_poor",
                    accuracy: &poor_accuracy,
                },
                SyntheticPvtGeometryProfileCase {
                    scenario_id: "pvt_geometry_profile_good",
                    accuracy: &good_accuracy,
                },
            ],
            "pvt_geometry_profile",
        );

        assert_eq!(report.points.len(), 2);
        assert_eq!(report.points[0].scenario_id, "pvt_geometry_profile_good");
        assert_eq!(report.points[0].mean_pdop, Some(1.6));
        assert_eq!(report.points[0].max_pdop, Some(1.8));
        assert_eq!(report.points[0].passing_epoch_count, 2);
        assert_eq!(report.points[1].scenario_id, "pvt_geometry_profile_poor");
        assert_eq!(report.points[1].mean_pdop, Some(4.75));
        assert_eq!(report.points[1].max_pdop, Some(5.5));
        assert_eq!(report.points[1].pass_rate, 0.5);
        assert!(report.points[1].ready);
    }

    #[test]
    fn pvt_constellation_geometry_profile_prioritizes_availability_then_dop() {
        let gps_only_truth = SyntheticPvtTruthTableReport {
            scenario_id: "pvt_constellation_geometry_profile_gps_only".to_string(),
            solution_count: 1,
            matched_epoch_count: 1,
            unmatched_solution_epochs: Vec::new(),
            unused_reference_epochs: vec![2],
            epochs: vec![SyntheticPvtTruthTableEpoch {
                artifact_id: "gps-only-artifact".to_string(),
                source_observation_epoch_id: "gps-only-source".to_string(),
                epoch_index: 1,
                receive_time_s: 1.0,
                truth_ecef_m: SyntheticPvtTruthTableEcef { x_m: 0.0, y_m: 0.0, z_m: 0.0 },
                measured_ecef_m: SyntheticPvtTruthTableEcef { x_m: 1.0, y_m: 0.0, z_m: 0.0 },
                position_covariance_ecef_m2: None,
                ecef_error_m: SyntheticPvtTruthTableEcef { x_m: 1.0, y_m: 0.0, z_m: 0.0 },
                truth_geodetic: SyntheticPvtTruthTableGeodetic {
                    latitude_deg: 0.0,
                    longitude_deg: 0.0,
                    altitude_m: 0.0,
                },
                measured_geodetic: SyntheticPvtTruthTableGeodetic {
                    latitude_deg: 0.0,
                    longitude_deg: 0.0,
                    altitude_m: 1.0,
                },
                enu_error_m: SyntheticPvtTruthTableEnuError {
                    east_m: 1.0,
                    north_m: 0.0,
                    up_m: 0.0,
                    horiz_m: 1.0,
                    vert_m: 0.0,
                    error_3d_m: 1.0,
                },
                clock_bias: SyntheticPvtTruthTableClockBias {
                    truth_s: 0.0,
                    measured_s: 0.0,
                    error_s: 0.0,
                    truth_m: 0.0,
                    measured_m: 0.0,
                    error_m: 0.0,
                },
                residual_rms_m: 0.75,
                pre_fit_residual_rms_m: Some(0.75),
                post_fit_residual_rms_m: Some(0.75),
                dop: SyntheticPvtTruthTableDop {
                    pdop: 4.0,
                    hdop: Some(2.5),
                    vdop: Some(3.0),
                    gdop: Some(4.5),
                    tdop: Some(1.0),
                },
                solution_status: SolutionStatus::CodeOnly,
                solution_quality: NavQualityFlag::Float,
                solution_validity: SolutionValidity::Stable,
                valid: true,
                sat_count: 4,
                used_sat_count: 4,
                rejected_sat_count: 0,
            }],
        };
        let mixed_truth = SyntheticPvtTruthTableReport {
            scenario_id: "pvt_constellation_geometry_profile_mixed".to_string(),
            solution_count: 2,
            matched_epoch_count: 2,
            unmatched_solution_epochs: Vec::new(),
            unused_reference_epochs: Vec::new(),
            epochs: vec![
                SyntheticPvtTruthTableEpoch {
                    artifact_id: "mixed-artifact-1".to_string(),
                    source_observation_epoch_id: "mixed-source-1".to_string(),
                    epoch_index: 1,
                    receive_time_s: 1.0,
                    truth_ecef_m: SyntheticPvtTruthTableEcef { x_m: 0.0, y_m: 0.0, z_m: 0.0 },
                    measured_ecef_m: SyntheticPvtTruthTableEcef { x_m: 0.5, y_m: 0.0, z_m: 0.0 },
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
                    residual_rms_m: 0.25,
                    pre_fit_residual_rms_m: Some(0.25),
                    post_fit_residual_rms_m: Some(0.25),
                    dop: SyntheticPvtTruthTableDop {
                        pdop: 1.4,
                        hdop: Some(1.0),
                        vdop: Some(1.0),
                        gdop: Some(1.7),
                        tdop: Some(0.4),
                    },
                    solution_status: SolutionStatus::CodeOnly,
                    solution_quality: NavQualityFlag::Float,
                    solution_validity: SolutionValidity::Stable,
                    valid: true,
                    sat_count: 7,
                    used_sat_count: 7,
                    rejected_sat_count: 0,
                },
                SyntheticPvtTruthTableEpoch {
                    artifact_id: "mixed-artifact-2".to_string(),
                    source_observation_epoch_id: "mixed-source-2".to_string(),
                    epoch_index: 2,
                    receive_time_s: 2.0,
                    truth_ecef_m: SyntheticPvtTruthTableEcef { x_m: 0.0, y_m: 0.0, z_m: 0.0 },
                    measured_ecef_m: SyntheticPvtTruthTableEcef { x_m: 0.75, y_m: 0.0, z_m: 0.0 },
                    position_covariance_ecef_m2: None,
                    ecef_error_m: SyntheticPvtTruthTableEcef { x_m: 0.75, y_m: 0.0, z_m: 0.0 },
                    truth_geodetic: SyntheticPvtTruthTableGeodetic {
                        latitude_deg: 0.0,
                        longitude_deg: 0.0,
                        altitude_m: 0.0,
                    },
                    measured_geodetic: SyntheticPvtTruthTableGeodetic {
                        latitude_deg: 0.0,
                        longitude_deg: 0.0,
                        altitude_m: 0.75,
                    },
                    enu_error_m: SyntheticPvtTruthTableEnuError {
                        east_m: 0.75,
                        north_m: 0.0,
                        up_m: 0.0,
                        horiz_m: 0.75,
                        vert_m: 0.0,
                        error_3d_m: 0.75,
                    },
                    clock_bias: SyntheticPvtTruthTableClockBias {
                        truth_s: 0.0,
                        measured_s: 0.0,
                        error_s: 0.0,
                        truth_m: 0.0,
                        measured_m: 0.0,
                        error_m: 0.0,
                    },
                    residual_rms_m: 0.3,
                    pre_fit_residual_rms_m: Some(0.3),
                    post_fit_residual_rms_m: Some(0.3),
                    dop: SyntheticPvtTruthTableDop {
                        pdop: 1.8,
                        hdop: Some(1.2),
                        vdop: Some(1.1),
                        gdop: Some(2.0),
                        tdop: Some(0.45),
                    },
                    solution_status: SolutionStatus::CodeOnly,
                    solution_quality: NavQualityFlag::Float,
                    solution_validity: SolutionValidity::Stable,
                    valid: true,
                    sat_count: 7,
                    used_sat_count: 7,
                    rejected_sat_count: 0,
                },
            ],
        };
        let gps_only_accuracy = SyntheticPvtAccuracyReport {
            scenario_id: "pvt_constellation_geometry_profile_gps_only".to_string(),
            max_position_error_3d_m: 5.0,
            max_clock_bias_error_m: 10.0,
            max_residual_rms_m: 4.0,
            max_pdop: 6.0,
            epoch_count: 1,
            passing_epoch_count: 1,
            truth_coverage_ready: true,
            truth_coverage_issues: Vec::new(),
            pass: true,
            epochs: vec![SyntheticPvtAccuracyEpoch {
                epoch_index: 1,
                position_error_3d_m: 1.0,
                clock_bias_error_m: 0.5,
                residual_rms_m: 0.75,
                pdop: 4.0,
                pass: true,
            }],
        };
        let mixed_accuracy = SyntheticPvtAccuracyReport {
            scenario_id: "pvt_constellation_geometry_profile_mixed".to_string(),
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
                    position_error_3d_m: 0.5,
                    clock_bias_error_m: 0.2,
                    residual_rms_m: 0.25,
                    pdop: 1.4,
                    pass: true,
                },
                SyntheticPvtAccuracyEpoch {
                    epoch_index: 2,
                    position_error_3d_m: 0.75,
                    clock_bias_error_m: 0.25,
                    residual_rms_m: 0.3,
                    pdop: 1.8,
                    pass: true,
                },
            ],
        };

        let report: SyntheticPvtConstellationGeometryProfileReport =
            summarize_truth_guided_pvt_constellation_geometry_profile(
                &[
                    SyntheticPvtConstellationGeometryProfileCase {
                        scenario_id: "pvt_constellation_geometry_profile_gps_only",
                        constellations: &[Constellation::Gps],
                        visible_satellite_count: 4,
                        truth_table: &gps_only_truth,
                        accuracy: &gps_only_accuracy,
                    },
                    SyntheticPvtConstellationGeometryProfileCase {
                        scenario_id: "pvt_constellation_geometry_profile_mixed",
                        constellations: &[
                            Constellation::Gps,
                            Constellation::Galileo,
                            Constellation::Beidou,
                        ],
                        visible_satellite_count: 7,
                        truth_table: &mixed_truth,
                        accuracy: &mixed_accuracy,
                    },
                ],
                "pvt_constellation_geometry_profile",
            );

        assert_eq!(report.points.len(), 2);
        assert_eq!(report.points[0].scenario_id, "pvt_constellation_geometry_profile_mixed");
        assert_eq!(report.points[0].constellations.len(), 3);
        assert_eq!(report.points[0].visible_satellite_count, 7);
        assert_eq!(report.points[0].expected_epoch_count, 2);
        assert_eq!(report.points[0].solved_epoch_count, 2);
        assert_eq!(report.points[0].availability_rate, 1.0);
        assert_eq!(report.points[0].mean_pdop, Some(1.6));
        assert_eq!(report.points[0].mean_gdop, Some(1.85));
        assert_eq!(report.points[1].scenario_id, "pvt_constellation_geometry_profile_gps_only");
        assert_eq!(report.points[1].constellations, vec![Constellation::Gps]);
        assert_eq!(report.points[1].expected_epoch_count, 2);
        assert_eq!(report.points[1].solved_epoch_count, 1);
        assert_eq!(report.points[1].availability_rate, 0.5);
        assert_eq!(report.points[1].mean_pdop, Some(4.0));
        assert_eq!(report.points[1].mean_gdop, Some(4.5));
        assert!(report.points[1].ready);
    }

    #[test]
    fn pvt_multipath_profile_sorts_cases_by_injected_bias_and_validity() {
        let clean_truth = SyntheticPvtTruthTableReport {
            scenario_id: "pvt_multipath_profile_clean".to_string(),
            solution_count: 1,
            matched_epoch_count: 1,
            unmatched_solution_epochs: Vec::new(),
            unused_reference_epochs: Vec::new(),
            epochs: vec![SyntheticPvtTruthTableEpoch {
                artifact_id: "clean-artifact".to_string(),
                source_observation_epoch_id: "clean-source".to_string(),
                epoch_index: 1,
                receive_time_s: 1.0,
                truth_ecef_m: SyntheticPvtTruthTableEcef { x_m: 0.0, y_m: 0.0, z_m: 0.0 },
                measured_ecef_m: SyntheticPvtTruthTableEcef { x_m: 0.5, y_m: 0.0, z_m: 0.0 },
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
                residual_rms_m: 0.75,
                pre_fit_residual_rms_m: Some(0.75),
                post_fit_residual_rms_m: Some(0.75),
                dop: SyntheticPvtTruthTableDop {
                    pdop: 1.5,
                    hdop: Some(1.0),
                    vdop: Some(1.0),
                    gdop: Some(1.8),
                    tdop: Some(0.6),
                },
                solution_status: SolutionStatus::CodeOnly,
                solution_quality: NavQualityFlag::Float,
                solution_validity: SolutionValidity::Stable,
                valid: true,
                sat_count: 5,
                used_sat_count: 5,
                rejected_sat_count: 0,
            }],
        };
        let severe_truth = SyntheticPvtTruthTableReport {
            scenario_id: "pvt_multipath_profile_severe".to_string(),
            solution_count: 1,
            matched_epoch_count: 1,
            unmatched_solution_epochs: Vec::new(),
            unused_reference_epochs: Vec::new(),
            epochs: vec![SyntheticPvtTruthTableEpoch {
                artifact_id: "severe-artifact".to_string(),
                source_observation_epoch_id: "severe-source".to_string(),
                epoch_index: 2,
                receive_time_s: 2.0,
                truth_ecef_m: SyntheticPvtTruthTableEcef { x_m: 0.0, y_m: 0.0, z_m: 0.0 },
                measured_ecef_m: SyntheticPvtTruthTableEcef { x_m: 20.0, y_m: 0.0, z_m: 0.0 },
                position_covariance_ecef_m2: None,
                ecef_error_m: SyntheticPvtTruthTableEcef { x_m: 20.0, y_m: 0.0, z_m: 0.0 },
                truth_geodetic: SyntheticPvtTruthTableGeodetic {
                    latitude_deg: 0.0,
                    longitude_deg: 0.0,
                    altitude_m: 0.0,
                },
                measured_geodetic: SyntheticPvtTruthTableGeodetic {
                    latitude_deg: 0.0,
                    longitude_deg: 0.0,
                    altitude_m: 20.0,
                },
                enu_error_m: SyntheticPvtTruthTableEnuError {
                    east_m: 20.0,
                    north_m: 0.0,
                    up_m: 0.0,
                    horiz_m: 20.0,
                    vert_m: 0.0,
                    error_3d_m: 20.0,
                },
                clock_bias: SyntheticPvtTruthTableClockBias {
                    truth_s: 0.0,
                    measured_s: 0.0,
                    error_s: 0.0,
                    truth_m: 0.0,
                    measured_m: 0.0,
                    error_m: 0.0,
                },
                residual_rms_m: 60.0,
                pre_fit_residual_rms_m: Some(60.0),
                post_fit_residual_rms_m: Some(60.0),
                dop: SyntheticPvtTruthTableDop {
                    pdop: 1.5,
                    hdop: Some(1.0),
                    vdop: Some(1.0),
                    gdop: Some(1.8),
                    tdop: Some(0.6),
                },
                solution_status: SolutionStatus::Degraded,
                solution_quality: NavQualityFlag::Degraded,
                solution_validity: SolutionValidity::Diverging,
                valid: true,
                sat_count: 5,
                used_sat_count: 5,
                rejected_sat_count: 0,
            }],
        };
        let clean_accuracy = SyntheticPvtAccuracyReport {
            scenario_id: "pvt_multipath_profile_clean".to_string(),
            max_position_error_3d_m: 5.0,
            max_clock_bias_error_m: 10.0,
            max_residual_rms_m: 4.0,
            max_pdop: 6.0,
            epoch_count: 1,
            passing_epoch_count: 1,
            truth_coverage_ready: true,
            truth_coverage_issues: Vec::new(),
            pass: true,
            epochs: vec![SyntheticPvtAccuracyEpoch {
                epoch_index: 1,
                position_error_3d_m: 0.5,
                clock_bias_error_m: 0.0,
                residual_rms_m: 0.75,
                pdop: 1.5,
                pass: true,
            }],
        };
        let severe_accuracy = SyntheticPvtAccuracyReport {
            scenario_id: "pvt_multipath_profile_severe".to_string(),
            max_position_error_3d_m: 5.0,
            max_clock_bias_error_m: 10.0,
            max_residual_rms_m: 4.0,
            max_pdop: 6.0,
            epoch_count: 1,
            passing_epoch_count: 0,
            truth_coverage_ready: true,
            truth_coverage_issues: Vec::new(),
            pass: false,
            epochs: vec![SyntheticPvtAccuracyEpoch {
                epoch_index: 2,
                position_error_3d_m: 20.0,
                clock_bias_error_m: 0.0,
                residual_rms_m: 60.0,
                pdop: 1.5,
                pass: false,
            }],
        };

        let report: SyntheticPvtMultipathProfileReport =
            summarize_truth_guided_pvt_multipath_profile(
                &[
                    SyntheticPvtMultipathProfileCase {
                        scenario_id: "pvt_multipath_profile_severe",
                        affected_satellite_count: 4,
                        mean_abs_pseudorange_bias_m: 22.0,
                        max_abs_pseudorange_bias_m: 35.0,
                        truth_table: &severe_truth,
                        accuracy: &severe_accuracy,
                    },
                    SyntheticPvtMultipathProfileCase {
                        scenario_id: "pvt_multipath_profile_clean",
                        affected_satellite_count: 0,
                        mean_abs_pseudorange_bias_m: 0.0,
                        max_abs_pseudorange_bias_m: 0.0,
                        truth_table: &clean_truth,
                        accuracy: &clean_accuracy,
                    },
                ],
                "pvt_multipath_profile",
            );

        assert_eq!(report.points.len(), 2);
        assert_eq!(report.points[0].scenario_id, "pvt_multipath_profile_clean");
        assert_eq!(report.points[0].stable_epoch_count, 1);
        assert_eq!(report.points[0].diverging_epoch_count, 0);
        assert_eq!(report.points[0].pass_rate, 1.0);
        assert_eq!(report.points[1].scenario_id, "pvt_multipath_profile_severe");
        assert_eq!(report.points[1].max_abs_pseudorange_bias_m, 35.0);
        assert_eq!(report.points[1].stable_epoch_count, 0);
        assert_eq!(report.points[1].diverging_epoch_count, 1);
        assert_eq!(report.points[1].max_residual_rms_m, Some(60.0));
        assert!(report.points[1].ready);
    }
