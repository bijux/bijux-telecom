use super::*;

#[test]
fn golden_reference_validation() {
    let (ecef_x_m, ecef_y_m, ecef_z_m) = bijux_gnss_core::api::lla_to_ecef(0.0, 0.0, 0.0);
    let sol = NavSolutionEpoch {
        epoch: bijux_gnss_core::api::Epoch { index: 0 },
        t_rx_s: bijux_gnss_core::api::Seconds(0.0),
        source_time: bijux_gnss_core::api::ReceiverSampleTrace::from_sample_index(0, 1.0),
        ecef_x_m: bijux_gnss_core::api::Meters(ecef_x_m),
        ecef_y_m: bijux_gnss_core::api::Meters(ecef_y_m),
        ecef_z_m: bijux_gnss_core::api::Meters(ecef_z_m),
        position_covariance_ecef_m2: None,
        latitude_deg: 0.0,
        longitude_deg: 0.0,
        altitude_m: bijux_gnss_core::api::Meters(0.0),
        clock_bias_s: bijux_gnss_core::api::Seconds(0.0),
        clock_bias_m: bijux_gnss_core::api::Meters(0.0),
        clock_drift_s_per_s: 0.0,
        pdop: 1.0,
        pre_fit_residual_rms_m: Some(bijux_gnss_core::api::Meters(0.0)),
        post_fit_residual_rms_m: Some(bijux_gnss_core::api::Meters(0.0)),
        rms_m: bijux_gnss_core::api::Meters(0.0),
        status: SolutionStatus::CodeOnly,
        quality: SolutionStatus::CodeOnly.quality_flag(),
        validity: bijux_gnss_core::api::SolutionValidity::Stable,
        valid: true,
        processing_ms: None,
        residuals: Vec::new(),
        constellation_residual_rms: Vec::new(),
        health: Vec::new(),
        isb: Vec::new(),
        sigma_e_m: None,
        sigma_n_m: None,
        sigma_u_m: None,
        horizontal_error_ellipse_major_axis_m: None,
        horizontal_error_ellipse_minor_axis_m: None,
        horizontal_error_ellipse_azimuth_deg: None,
        sigma_h_m: None,
        sigma_v_m: None,
        innovation_rms_m: None,
        normalized_innovation_rms: None,
        normalized_innovation_max: None,
        ekf_innovation_rms: None,
        ekf_condition_number: None,
        wls_solver_rank: None,
        wls_condition_number: None,
        ekf_whiteness_ratio: None,
        ekf_predicted_variance: None,
        ekf_observed_variance: None,
        integrity_hpl_m: None,
        integrity_vpl_m: None,
        model_version: bijux_gnss_core::api::NAV_SOLUTION_MODEL_VERSION,
        lifecycle_state: bijux_gnss_core::api::NavLifecycleState::CodeOnly,
        uncertainty_class: bijux_gnss_core::api::NavUncertaintyClass::Low,
        assumptions: None,
        refusal_class: None,
        artifact_id: "nav-epoch-0000000000-golden".to_string(),
        source_observation_epoch_id: "obs-epoch-0000000000-golden".to_string(),
        explain_decision: "accepted".to_string(),
        explain_reasons: vec!["navigation_solution_usable".to_string()],
        provenance: None,
        sat_count: 4,
        used_sat_count: 4,
        rejected_sat_count: 0,
        hdop: Some(1.0),
        vdop: Some(1.0),
        gdop: Some(1.0),
        tdop: Some(0.5),
        stability_signature: "navsig:v2:golden".to_string(),
        stability_signature_version: bijux_gnss_core::api::NAV_OUTPUT_STABILITY_SIGNATURE_VERSION,
    };
    let reference = ValidationReferenceEpoch {
        epoch_idx: 0,
        t_rx_s: Some(0.0),
        latitude_deg: 0.0,
        longitude_deg: 0.0,
        altitude_m: 0.0,
        ecef_x_m: Some(ecef_x_m),
        ecef_y_m: Some(ecef_y_m),
        ecef_z_m: Some(ecef_z_m),
        vel_x_mps: None,
        vel_y_mps: None,
        vel_z_mps: None,
    };
    let report = build_validation_report(
        &[],
        &[],
        &[sol],
        &[reference],
        1.0,
        false,
        Vec::new(),
        ValidationSciencePolicy::default(),
    )
    .expect("validation report");
    assert!(report.horiz_error_m.rms <= 1e-6);
    assert!(report.vert_error_m.rms <= 1e-6);
    assert!(report.error_3d_m.rms <= 1e-6);
    assert_eq!(report.reference_position_errors.len(), 1);
    assert_eq!(report.integrity.len(), 1);
}

#[test]
fn validation_report_emits_enu_reference_components() {
    let (x_ref, y_ref, z_ref) = bijux_gnss_core::api::lla_to_ecef(0.0, 0.0, 0.0);
    let solution = NavSolutionEpoch {
        ecef_x_m: bijux_gnss_core::api::Meters(x_ref + 3.0),
        ecef_y_m: bijux_gnss_core::api::Meters(y_ref + 1.0),
        ecef_z_m: bijux_gnss_core::api::Meters(z_ref + 2.0),
        position_covariance_ecef_m2: Some([[9.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 4.0]]),
        sigma_e_m: Some(bijux_gnss_core::api::Meters(1.0)),
        sigma_n_m: Some(bijux_gnss_core::api::Meters(1.5)),
        sigma_u_m: Some(bijux_gnss_core::api::Meters(3.0)),
        horizontal_error_ellipse_major_axis_m: Some(bijux_gnss_core::api::Meters(2.0)),
        horizontal_error_ellipse_minor_axis_m: Some(bijux_gnss_core::api::Meters(1.0)),
        horizontal_error_ellipse_azimuth_deg: Some(42.0),
        sigma_h_m: Some(bijux_gnss_core::api::Meters(2.0)),
        sigma_v_m: Some(bijux_gnss_core::api::Meters(3.0)),
        integrity_hpl_m: Some(3.0),
        integrity_vpl_m: Some(4.0),
        ..fixture_solution(7, 1.0, 0.5, 4)
    };
    let reference = ValidationReferenceEpoch {
        epoch_idx: 7,
        t_rx_s: Some(7.0),
        latitude_deg: 0.0,
        longitude_deg: 0.0,
        altitude_m: 0.0,
        ecef_x_m: Some(x_ref),
        ecef_y_m: Some(y_ref),
        ecef_z_m: Some(z_ref),
        vel_x_mps: None,
        vel_y_mps: None,
        vel_z_mps: None,
    };

    let report = build_validation_report(
        &[],
        &[],
        &[solution],
        &[reference],
        1.0,
        false,
        Vec::new(),
        ValidationSciencePolicy::default(),
    )
    .expect("validation report");

    let error = report.reference_position_errors.first().expect("reference position error");
    assert_eq!(error.epoch_idx, 7);
    assert!((error.east_m - 1.0).abs() < 1.0e-12);
    assert!((error.north_m - 2.0).abs() < 1.0e-12);
    assert!((error.up_m - 3.0).abs() < 1.0e-12);
    assert!((error.horiz_m - 5.0_f64.sqrt()).abs() < 1.0e-12);
    assert!((error.vert_m - 3.0).abs() < 1.0e-12);
    assert!((error.error_3d_m - 14.0_f64.sqrt()).abs() < 1.0e-12);
    assert_eq!(error.hpl_m, Some(3.0));
    assert_eq!(error.vpl_m, Some(4.0));
    assert_eq!(error.horizontal_within_hpl, Some(true));
    assert_eq!(error.vertical_within_vpl, Some(true));
    assert!(
        (error.horizontal_margin_m.expect("horizontal margin") - (3.0 - 5.0_f64.sqrt())).abs()
            < 1.0e-12
    );
    assert!((error.vertical_margin_m.expect("vertical margin") - 1.0).abs() < 1.0e-12);
    assert!((report.east_error_m.rms - 1.0).abs() < 1.0e-12);
    assert!((report.north_error_m.rms - 2.0).abs() < 1.0e-12);
    assert!((report.up_error_m.rms - 3.0).abs() < 1.0e-12);
    assert!((report.horiz_error_m.rms - 5.0_f64.sqrt()).abs() < 1.0e-12);
    assert!((report.vert_error_m.rms - 3.0).abs() < 1.0e-12);
    assert!((report.error_3d_m.rms - 14.0_f64.sqrt()).abs() < 1.0e-12);
    assert!(report.nees_mean.is_some());
    assert_eq!(report.covariance_realism.total_epoch_count, 1);
    assert_eq!(report.covariance_realism.covariance_epoch_count, 1);
    assert_eq!(report.covariance_realism.horizontal_95.sample_count, 1);
    assert_eq!(report.covariance_realism.vertical_95.sample_count, 1);
    assert_eq!(report.covariance_realism.position_3d_95.sample_count, 1);
    let horizontal_nees = report.covariance_realism.horizontal_nees_mean.expect("horizontal nees");
    let vertical_normalized_error = report
        .covariance_realism
        .vertical_normalized_error_squared_mean
        .expect("vertical normalized error");
    let position_nees = report.covariance_realism.position_nees_mean.expect("position nees");
    assert!((horizontal_nees - 2.0).abs() < 1.0e-5, "{horizontal_nees}");
    assert!((vertical_normalized_error - 1.0).abs() < 1.0e-9, "{vertical_normalized_error}");
    assert!((position_nees - 3.0).abs() < 1.0e-5, "{position_nees}");
    assert_eq!(report.protection_levels.matched_epoch_count, 1);
    assert_eq!(report.protection_levels.horizontal_reported_epoch_count, 1);
    assert_eq!(report.protection_levels.vertical_reported_epoch_count, 1);
    assert_eq!(report.protection_levels.horizontal_contained_epoch_count, 1);
    assert_eq!(report.protection_levels.vertical_contained_epoch_count, 1);
    assert!(report.protection_levels.horizontal_breach_epochs.is_empty());
    assert!(report.protection_levels.vertical_breach_epochs.is_empty());
}

#[test]
fn validation_report_summarizes_protection_level_breaches() {
    let (x_ref, y_ref, z_ref) = bijux_gnss_core::api::lla_to_ecef(0.0, 0.0, 0.0);
    let solution = NavSolutionEpoch {
        ecef_x_m: bijux_gnss_core::api::Meters(x_ref + 3.0),
        ecef_y_m: bijux_gnss_core::api::Meters(y_ref + 1.0),
        ecef_z_m: bijux_gnss_core::api::Meters(z_ref + 4.0),
        integrity_hpl_m: Some(1.0),
        integrity_vpl_m: Some(2.0),
        ..fixture_solution(8, 1.0, 0.5, 4)
    };
    let reference = ValidationReferenceEpoch {
        epoch_idx: 8,
        t_rx_s: Some(8.0),
        latitude_deg: 0.0,
        longitude_deg: 0.0,
        altitude_m: 0.0,
        ecef_x_m: Some(x_ref),
        ecef_y_m: Some(y_ref),
        ecef_z_m: Some(z_ref),
        vel_x_mps: None,
        vel_y_mps: None,
        vel_z_mps: None,
    };

    let report = build_validation_report(
        &[],
        &[],
        &[solution],
        &[reference],
        1.0,
        false,
        Vec::new(),
        ValidationSciencePolicy::default(),
    )
    .expect("validation report");

    let error = report.reference_position_errors.first().expect("reference position error");
    assert_eq!(error.horizontal_within_hpl, Some(false));
    assert_eq!(error.vertical_within_vpl, Some(false));
    assert!(error.horizontal_margin_m.expect("horizontal margin") < 0.0);
    assert!(error.vertical_margin_m.expect("vertical margin") < 0.0);
    assert_eq!(report.protection_levels.horizontal_breach_epochs, vec![8]);
    assert_eq!(report.protection_levels.vertical_breach_epochs, vec![8]);
    assert_eq!(report.protection_levels.horizontal_contained_epoch_count, 0);
    assert_eq!(report.protection_levels.vertical_contained_epoch_count, 0);
    assert!(report.protection_levels.min_horizontal_margin_m.expect("horizontal min margin") < 0.0);
    assert!(report.protection_levels.min_vertical_margin_m.expect("vertical min margin") < 0.0);
}

#[test]
fn validation_report_preserves_pre_and_post_fit_residual_rms() {
    let mut solution = fixture_solution(9, 1.0, 0.5, 4);
    solution.pre_fit_residual_rms_m = Some(bijux_gnss_core::api::Meters(12.0));
    solution.post_fit_residual_rms_m = Some(bijux_gnss_core::api::Meters(3.0));
    solution.rms_m = bijux_gnss_core::api::Meters(3.0);
    solution.constellation_residual_rms = vec![
        bijux_gnss_core::api::NavConstellationResidualRms {
            constellation: Constellation::Gps,
            pre_fit_rms_m: Some(bijux_gnss_core::api::Meters(10.0)),
            post_fit_rms_m: Some(bijux_gnss_core::api::Meters(2.0)),
            pre_fit_sat_count: 4,
            post_fit_sat_count: 4,
        },
        bijux_gnss_core::api::NavConstellationResidualRms {
            constellation: Constellation::Galileo,
            pre_fit_rms_m: Some(bijux_gnss_core::api::Meters(16.0)),
            post_fit_rms_m: Some(bijux_gnss_core::api::Meters(5.0)),
            pre_fit_sat_count: 2,
            post_fit_sat_count: 2,
        },
    ];

    let report = build_validation_report(
        &[],
        &[],
        &[solution],
        &[],
        1.0,
        false,
        Vec::new(),
        ValidationSciencePolicy::default(),
    )
    .expect("validation report");

    let residual = report.residuals.first().expect("residual report");
    assert_eq!(residual.epoch_idx, 9);
    assert_eq!(residual.rms_m, 3.0);
    assert_eq!(residual.pre_fit_rms_m, Some(12.0));
    assert_eq!(residual.post_fit_rms_m, Some(3.0));
    assert_eq!(residual.constellation_residual_rms.len(), 2);
    assert_eq!(residual.constellation_residual_rms[0].constellation, "Gps");
    assert_eq!(residual.constellation_residual_rms[0].pre_fit_rms_m, Some(10.0));
    assert_eq!(residual.constellation_residual_rms[0].post_fit_rms_m, Some(2.0));
    assert_eq!(residual.constellation_residual_rms[0].pre_fit_sat_count, 4);
    assert_eq!(residual.constellation_residual_rms[0].post_fit_sat_count, 4);
    assert_eq!(residual.constellation_residual_rms[1].constellation, "Galileo");
}

#[test]
fn validation_report_accepts_reference_position_error_within_budget() {
    let (x_ref, y_ref, z_ref) = bijux_gnss_core::api::lla_to_ecef(0.0, 0.0, 0.0);
    let budgets = ValidationBudgets {
        nav_min_lock_epochs: 0,
        reference_position_error_3d_m_max: Some(5.0),
        ..ValidationBudgets::default()
    };
    let report = build_validation_report_with_budgets(
        &[],
        &[],
        &[NavSolutionEpoch {
            ecef_x_m: bijux_gnss_core::api::Meters(x_ref),
            ecef_y_m: bijux_gnss_core::api::Meters(y_ref),
            ecef_z_m: bijux_gnss_core::api::Meters(z_ref),
            ..fixture_solution(11, 1.0, 0.5, 4)
        }],
        &[ValidationReferenceEpoch {
            epoch_idx: 11,
            t_rx_s: Some(11.0),
            latitude_deg: 0.0,
            longitude_deg: 0.0,
            altitude_m: 0.0,
            ecef_x_m: Some(x_ref),
            ecef_y_m: Some(y_ref),
            ecef_z_m: Some(z_ref),
            vel_x_mps: None,
            vel_y_mps: None,
            vel_z_mps: None,
        }],
        1.0,
        false,
        Vec::new(),
        ValidationSciencePolicy::default(),
        budgets,
    )
    .expect("validation report");

    assert!(report.budget_violations.is_empty());
    assert!(report.reference_coordinate_coverage.required);
    assert!(report.reference_coordinate_coverage.ready);
    assert!(report.reference_coordinate_coverage.unmatched_solution_epochs.is_empty());
}

#[test]
fn validation_report_reports_reference_position_budget_violation() {
    let (x_ref, y_ref, z_ref) = bijux_gnss_core::api::lla_to_ecef(0.0, 0.0, 0.0);
    let budgets = ValidationBudgets {
        nav_min_lock_epochs: 0,
        reference_position_error_3d_m_max: Some(5.0),
        ..ValidationBudgets::default()
    };
    let report = build_validation_report_with_budgets(
        &[],
        &[],
        &[NavSolutionEpoch {
            ecef_x_m: bijux_gnss_core::api::Meters(x_ref + 2.0),
            ecef_y_m: bijux_gnss_core::api::Meters(y_ref + 3.0),
            ecef_z_m: bijux_gnss_core::api::Meters(z_ref + 4.0),
            ..fixture_solution(12, 1.0, 0.5, 4)
        }],
        &[ValidationReferenceEpoch {
            epoch_idx: 12,
            t_rx_s: Some(12.0),
            latitude_deg: 0.0,
            longitude_deg: 0.0,
            altitude_m: 0.0,
            ecef_x_m: Some(x_ref),
            ecef_y_m: Some(y_ref),
            ecef_z_m: Some(z_ref),
            vel_x_mps: None,
            vel_y_mps: None,
            vel_z_mps: None,
        }],
        1.0,
        false,
        Vec::new(),
        ValidationSciencePolicy::default(),
        budgets,
    )
    .expect("validation report");

    assert_eq!(
        report.budget_violations,
        vec!["reference position 3d error too high at epoch 12: 5.39 m"]
    );
    assert!(report.reference_coordinate_coverage.required);
    assert!(report.reference_coordinate_coverage.ready);
}

#[test]
fn validation_report_requires_reference_coordinates_when_budget_is_set() {
    let budgets = ValidationBudgets {
        nav_min_lock_epochs: 0,
        reference_position_error_3d_m_max: Some(5.0),
        ..ValidationBudgets::default()
    };
    let report = build_validation_report_with_budgets(
        &[],
        &[],
        &[fixture_solution(21, 1.0, 0.5, 4)],
        &[],
        1.0,
        false,
        Vec::new(),
        ValidationSciencePolicy::default(),
        budgets,
    )
    .expect("validation report");

    assert!(report.reference_coordinate_coverage.required);
    assert!(!report.reference_coordinate_coverage.ready);
    assert_eq!(report.reference_coordinate_coverage.matched_solution_epoch_count, 0);
    assert_eq!(report.reference_coordinate_coverage.unmatched_solution_epochs, vec![21]);
    assert_eq!(
        report.budget_violations,
        vec!["reference coordinates unavailable for solution epochs: 21"]
    );
}

#[test]
fn validation_report_requires_reference_coordinates_for_all_solution_epochs() {
    let budgets = ValidationBudgets {
        nav_min_lock_epochs: 0,
        reference_position_error_3d_m_max: Some(5.0),
        ..ValidationBudgets::default()
    };
    let (x_ref, y_ref, z_ref) = bijux_gnss_core::api::lla_to_ecef(0.0, 0.0, 0.0);
    let mut matched_solution = fixture_solution(30, 1.0, 0.5, 4);
    matched_solution.ecef_x_m = bijux_gnss_core::api::Meters(x_ref);
    matched_solution.ecef_y_m = bijux_gnss_core::api::Meters(y_ref);
    matched_solution.ecef_z_m = bijux_gnss_core::api::Meters(z_ref);
    let report = build_validation_report_with_budgets(
        &[],
        &[],
        &[matched_solution, fixture_solution(31, 1.0, 0.5, 4)],
        &[ValidationReferenceEpoch {
            epoch_idx: 30,
            t_rx_s: Some(30.0),
            latitude_deg: 0.0,
            longitude_deg: 0.0,
            altitude_m: 0.0,
            ecef_x_m: Some(x_ref),
            ecef_y_m: Some(y_ref),
            ecef_z_m: Some(z_ref),
            vel_x_mps: None,
            vel_y_mps: None,
            vel_z_mps: None,
        }],
        1.0,
        false,
        Vec::new(),
        ValidationSciencePolicy::default(),
        budgets,
    )
    .expect("validation report");

    assert!(report.reference_coordinate_coverage.required);
    assert!(!report.reference_coordinate_coverage.ready);
    assert_eq!(report.reference_coordinate_coverage.matched_solution_epoch_count, 1);
    assert_eq!(report.reference_coordinate_coverage.unmatched_solution_epochs, vec![31]);
    assert_eq!(
        report.budget_violations,
        vec!["reference coordinates unavailable for solution epochs: 31"]
    );
}
