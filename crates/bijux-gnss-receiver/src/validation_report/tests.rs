use super::*;
use crate::api::TrackingResult;
use bijux_gnss_core::api::{
    Chips, Constellation, Cycles, Epoch, Hertz, LockFlags, Meters, NavAssumptions, ObsMetadata,
    ObsSatellite, ObservationEpochDecision, ObservationStatus, ReceiverRole, ReceiverSampleTrace,
    SatId, SigId, SignalCode, TrackEpoch,
};
use bijux_gnss_signal::api::{carrier_wavelength_m, signal_registry};
use serde::Deserialize;

#[path = "tests/ppp_policy.rs"]
mod ppp_policy;

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
fn validation_report_surfaces_innovation_consistency_anomalies() {
    let mut solution = fixture_solution(9, 1.0, 0.5, 4);
    solution.health.push(bijux_gnss_core::api::NavHealthEvent::InnovationConsistencyAnomaly {
        normalized_innovation_squared: 8.0,
        lower_bound: 0.1,
        upper_bound: 6.6,
        measurement_dimension: 1,
    });
    let reference = ValidationReferenceEpoch {
        epoch_idx: 9,
        t_rx_s: Some(9.0),
        latitude_deg: 0.0,
        longitude_deg: 0.0,
        altitude_m: 0.0,
        ecef_x_m: Some(0.0),
        ecef_y_m: Some(0.0),
        ecef_z_m: Some(0.0),
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

    assert!(report.consistency_warnings.iter().any(|warning| {
        warning.contains("innovation consistency anomalies") && warning.contains("peak NIS 8.000")
    }));
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

#[test]
fn validation_report_marks_l1_l2_dual_frequency_pairs_ready_for_combinations() {
    let report = build_validation_report(
        &[],
        &[dual_frequency_epoch(
            40,
            vec![
                dual_frequency_satellite(SignalBand::L1, SignalCode::Ca, true, true),
                dual_frequency_satellite(SignalBand::L2, SignalCode::Py, true, true),
            ],
        )],
        &[fixture_solution(40, 1.0, 0.5, 4)],
        &[],
        1.0,
        true,
        Vec::new(),
        ValidationSciencePolicy::default(),
    )
    .expect("validation report");

    assert_eq!(report.dual_frequency_observations.complete_pairs, 1);
    assert_eq!(report.dual_frequency_observations.l1_l2_pairs, 1);
    assert!(report.ppp_readiness.multi_freq_present);
    assert!(report.ppp_readiness.combinations_valid);
    assert_eq!(report.ppp_readiness.maturity, AdvancedMaturity::NotReady);
    assert_eq!(report.ppp_readiness.status, "not_ready");
    assert_eq!(report.ppp_readiness.status_reason.as_deref(), Some("missing_reference_frame"));
    assert_eq!(report.ppp_readiness.claim, AdvancedSolutionClaim::NotReady);
}

#[test]
fn validation_report_marks_l1_l5_dual_frequency_pairs_ready_for_combinations() {
    let report = build_validation_report(
        &[],
        &[dual_frequency_epoch(
            41,
            vec![
                dual_frequency_satellite(SignalBand::L1, SignalCode::Ca, true, true),
                dual_frequency_satellite(SignalBand::L5, SignalCode::L5I, true, true),
            ],
        )],
        &[fixture_solution(41, 1.0, 0.5, 4)],
        &[],
        1.0,
        true,
        Vec::new(),
        ValidationSciencePolicy::default(),
    )
    .expect("validation report");

    assert_eq!(report.dual_frequency_observations.complete_pairs, 1);
    assert_eq!(report.dual_frequency_observations.l1_l5_pairs, 1);
    assert!(report.ppp_readiness.multi_freq_present);
    assert!(report.ppp_readiness.combinations_valid);
}

#[test]
fn validation_report_marks_e1_e5_dual_frequency_pairs_ready_for_combinations() {
    let report = build_validation_report(
        &[],
        &[dual_frequency_epoch(
            44,
            vec![
                dual_frequency_satellite_for_constellation(
                    Constellation::Galileo,
                    SignalBand::E1,
                    SignalCode::E1B,
                    true,
                    true,
                ),
                dual_frequency_satellite_for_constellation(
                    Constellation::Galileo,
                    SignalBand::E5,
                    SignalCode::E5a,
                    true,
                    true,
                ),
            ],
        )],
        &[fixture_solution(44, 1.0, 0.5, 4)],
        &[],
        1.0,
        true,
        Vec::new(),
        ValidationSciencePolicy::default(),
    )
    .expect("validation report");

    assert_eq!(report.dual_frequency_observations.complete_pairs, 1);
    assert_eq!(report.dual_frequency_observations.e1_e5_pairs, 1);
    assert!(report.ppp_readiness.multi_freq_present);
    assert!(report.ppp_readiness.combinations_valid);
}

#[test]
fn validation_report_marks_b1_b2_dual_frequency_pairs_ready_for_combinations() {
    let report = build_validation_report(
        &[],
        &[dual_frequency_epoch(
            45,
            vec![
                dual_frequency_satellite_for_constellation(
                    Constellation::Beidou,
                    SignalBand::B1,
                    SignalCode::B1I,
                    true,
                    true,
                ),
                dual_frequency_satellite_for_constellation(
                    Constellation::Beidou,
                    SignalBand::B2,
                    SignalCode::B2I,
                    true,
                    true,
                ),
            ],
        )],
        &[fixture_solution(45, 1.0, 0.5, 4)],
        &[],
        1.0,
        true,
        Vec::new(),
        ValidationSciencePolicy::default(),
    )
    .expect("validation report");

    assert_eq!(report.dual_frequency_observations.complete_pairs, 1);
    assert_eq!(report.dual_frequency_observations.b1_b2_pairs, 1);
    assert!(report.ppp_readiness.multi_freq_present);
    assert!(report.ppp_readiness.combinations_valid);
}

#[test]
fn validation_report_marks_incomplete_dual_frequency_pairs_not_ready_for_combinations() {
    let report = build_validation_report(
        &[],
        &[dual_frequency_epoch(
            42,
            vec![
                dual_frequency_satellite(SignalBand::L1, SignalCode::Ca, true, true),
                dual_frequency_satellite(SignalBand::L2, SignalCode::Py, false, true),
            ],
        )],
        &[fixture_solution(42, 1.0, 0.5, 4)],
        &[],
        1.0,
        true,
        Vec::new(),
        ValidationSciencePolicy::default(),
    )
    .expect("validation report");

    assert_eq!(report.dual_frequency_observations.complete_pairs, 0);
    assert_eq!(report.dual_frequency_observations.incomplete_pairs, 2);
    assert!(report.ppp_readiness.multi_freq_present);
    assert!(!report.ppp_readiness.combinations_valid);
    assert!(!report.ppp_readiness.prerequisites_met);
    assert_eq!(report.ppp_readiness.status, "not_ready");
    assert_eq!(report.ppp_readiness.status_reason.as_deref(), Some("missing_reference_frame"));
    assert_eq!(report.ppp_readiness.refusal_class, Some(AdvancedRefusalClass::UnsupportedModel));
    assert_eq!(report.ppp_readiness.claim, AdvancedSolutionClaim::NotReady);
}

#[test]
fn validation_report_accepts_code_ready_pairs_without_carrier_lock() {
    let report = build_validation_report(
        &[],
        &[dual_frequency_epoch(
            43,
            vec![
                dual_frequency_satellite(SignalBand::L1, SignalCode::Ca, true, false),
                dual_frequency_satellite(SignalBand::L2, SignalCode::Py, true, false),
            ],
        )],
        &[fixture_solution(43, 1.0, 0.5, 4)],
        &[],
        1.0,
        true,
        Vec::new(),
        ValidationSciencePolicy::default(),
    )
    .expect("validation report");

    assert_eq!(report.dual_frequency_observations.complete_pairs, 0);
    assert_eq!(report.dual_frequency_observations.incomplete_pairs, 2);
    assert!(report.ppp_readiness.multi_freq_present);
    assert!(report.ppp_readiness.combinations_valid);
    assert_eq!(report.ppp_readiness.status, "not_ready");
    assert_eq!(report.ppp_readiness.status_reason.as_deref(), Some("missing_reference_frame"));
    assert_eq!(report.ppp_readiness.claim, AdvancedSolutionClaim::NotReady);
}

#[test]
fn validation_report_surfaces_carrier_smoothed_code_summary_without_raw_residuals() {
    let report = build_validation_report(
        &[],
        &[
            dual_frequency_epoch(43, vec![carrier_smoothed_code_satellite(1, false)]),
            dual_frequency_epoch(44, vec![carrier_smoothed_code_satellite(6, false)]),
            dual_frequency_epoch(45, vec![carrier_smoothed_code_satellite(1, true)]),
        ],
        &[fixture_solution(43, 1.0, 0.5, 4)],
        &[],
        1.0,
        false,
        Vec::new(),
        ValidationSciencePolicy::default(),
    )
    .expect("validation report");

    assert_eq!(report.carrier_smoothed_code.observations, 3);
    assert_eq!(report.carrier_smoothed_code.accepted_observations, 3);
    assert_eq!(report.carrier_smoothed_code.smoothed_observations, 3);
    assert_eq!(report.carrier_smoothed_code.cycle_slip_observations, 1);
    assert_eq!(report.carrier_smoothed_code.slip_reset_observations, 1);
    assert_eq!(report.carrier_smoothed_code.hidden_slip_observations, 0);
    assert_eq!(report.carrier_smoothed_code.improvement_verified, None);
    assert_eq!(report.carrier_smoothed_code.slip_visibility_verified, Some(true));
}

#[test]
fn validation_report_summarizes_geometry_free_dynamics() {
    let report = build_validation_report(
        &[],
        &[
            dual_frequency_epoch(
                50,
                vec![
                    dual_frequency_satellite_with_phase(
                        SignalBand::L1,
                        SignalCode::Ca,
                        22_000_000.0,
                    ),
                    dual_frequency_satellite_with_phase(
                        SignalBand::L2,
                        SignalCode::Py,
                        21_999_999.50,
                    ),
                ],
            ),
            dual_frequency_epoch(
                51,
                vec![
                    dual_frequency_satellite_with_phase(
                        SignalBand::L1,
                        SignalCode::Ca,
                        22_000_000.0,
                    ),
                    dual_frequency_satellite_with_phase(
                        SignalBand::L2,
                        SignalCode::Py,
                        21_999_999.46,
                    ),
                ],
            ),
            dual_frequency_epoch(
                52,
                vec![
                    dual_frequency_satellite_with_phase(
                        SignalBand::L1,
                        SignalCode::Ca,
                        22_000_000.0,
                    ),
                    dual_frequency_satellite_with_phase(
                        SignalBand::L2,
                        SignalCode::Py,
                        21_999_999.20,
                    ),
                ],
            ),
        ],
        &[
            fixture_solution(50, 1.0, 0.5, 4),
            fixture_solution(51, 1.0, 0.5, 4),
            fixture_solution(52, 1.0, 0.5, 4),
        ],
        &[],
        1.0,
        true,
        Vec::new(),
        ValidationSciencePolicy::default(),
    )
    .expect("validation report");

    assert_eq!(report.geometry_free.observations, 3);
    assert_eq!(report.geometry_free.complete_pairs, 3);
    assert_eq!(report.geometry_free.unavailable, 0);
    assert_eq!(report.geometry_free.insufficient_history, 1);
    assert_eq!(report.geometry_free.ionosphere_drift, 1);
    assert_eq!(report.geometry_free.cycle_slip_suspects, 1);
    assert!(report.geometry_free.max_abs_delta_m.expect("max delta") > 0.2);
}

#[test]
fn validation_report_summarizes_melbourne_wubbena_dynamics() {
    let report = build_validation_report(
        &[],
        &[
            dual_frequency_epoch(
                60,
                vec![
                    dual_frequency_satellite_with_phase(
                        SignalBand::L1,
                        SignalCode::Ca,
                        21_999_999.0,
                    ),
                    dual_frequency_satellite_with_phase(
                        SignalBand::L2,
                        SignalCode::Py,
                        22_000_000.5,
                    ),
                ],
            ),
            dual_frequency_epoch(
                61,
                vec![
                    dual_frequency_satellite_with_phase(
                        SignalBand::L1,
                        SignalCode::Ca,
                        21_999_999.01,
                    ),
                    dual_frequency_satellite_with_phase(
                        SignalBand::L2,
                        SignalCode::Py,
                        22_000_000.49,
                    ),
                ],
            ),
            dual_frequency_epoch(
                62,
                vec![
                    dual_frequency_satellite_with_phase(
                        SignalBand::L1,
                        SignalCode::Ca,
                        22_000_005.5,
                    ),
                    dual_frequency_satellite_with_phase(
                        SignalBand::L2,
                        SignalCode::Py,
                        22_000_000.5,
                    ),
                ],
            ),
        ],
        &[
            fixture_solution(60, 1.0, 0.5, 4),
            fixture_solution(61, 1.0, 0.5, 4),
            fixture_solution(62, 1.0, 0.5, 4),
        ],
        &[],
        1.0,
        true,
        Vec::new(),
        ValidationSciencePolicy::default(),
    )
    .expect("validation report");

    assert_eq!(report.melbourne_wubbena.observations, 3);
    assert_eq!(report.melbourne_wubbena.complete_pairs, 3);
    assert_eq!(report.melbourne_wubbena.unavailable, 0);
    assert_eq!(report.melbourne_wubbena.insufficient_history, 1);
    assert_eq!(report.melbourne_wubbena.nominal, 1);
    assert_eq!(report.melbourne_wubbena.wide_lane_slip_suspects, 1);
    assert!(
        report.melbourne_wubbena.max_abs_delta_wide_lane_cycles.expect("max wide-lane delta")
            >= 0.5
    );
}

#[derive(Debug, Deserialize)]
struct ScienceFixtureCase {
    name: String,
    pdop: f64,
    rms_m: f64,
    used_sat_count: usize,
    lock_ratio: f64,
    expected_class: String,
}

fn fixture_solution(
    epoch_idx: u64,
    pdop: f64,
    rms_m: f64,
    used_sat_count: usize,
) -> NavSolutionEpoch {
    NavSolutionEpoch {
        epoch: bijux_gnss_core::api::Epoch { index: epoch_idx },
        t_rx_s: bijux_gnss_core::api::Seconds(epoch_idx as f64),
        source_time: bijux_gnss_core::api::ReceiverSampleTrace::from_sample_index(epoch_idx, 1.0),
        ecef_x_m: bijux_gnss_core::api::Meters(0.0),
        ecef_y_m: bijux_gnss_core::api::Meters(0.0),
        ecef_z_m: bijux_gnss_core::api::Meters(0.0),
        position_covariance_ecef_m2: None,
        latitude_deg: 0.0,
        longitude_deg: 0.0,
        altitude_m: bijux_gnss_core::api::Meters(0.0),
        clock_bias_s: bijux_gnss_core::api::Seconds(0.0),
        clock_bias_m: bijux_gnss_core::api::Meters(0.0),
        clock_drift_s_per_s: 0.0,
        pdop,
        pre_fit_residual_rms_m: Some(bijux_gnss_core::api::Meters(rms_m)),
        post_fit_residual_rms_m: Some(bijux_gnss_core::api::Meters(rms_m)),
        rms_m: bijux_gnss_core::api::Meters(rms_m),
        status: SolutionStatus::CodeOnly,
        quality: SolutionStatus::CodeOnly.quality_flag(),
        validity: bijux_gnss_core::api::SolutionValidity::Stable,
        valid: true,
        processing_ms: None,
        sigma_e_m: Some(bijux_gnss_core::api::Meters(1.0)),
        sigma_n_m: Some(bijux_gnss_core::api::Meters(1.0)),
        sigma_u_m: Some(bijux_gnss_core::api::Meters(1.0)),
        horizontal_error_ellipse_major_axis_m: Some(bijux_gnss_core::api::Meters(1.0)),
        horizontal_error_ellipse_minor_axis_m: Some(bijux_gnss_core::api::Meters(0.5)),
        horizontal_error_ellipse_azimuth_deg: Some(0.0),
        sigma_h_m: Some(bijux_gnss_core::api::Meters(1.0)),
        sigma_v_m: Some(bijux_gnss_core::api::Meters(1.0)),
        residuals: Vec::new(),
        constellation_residual_rms: Vec::new(),
        isb: Vec::new(),
        health: Vec::new(),
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
        uncertainty_class: bijux_gnss_core::api::NavUncertaintyClass::Medium,
        assumptions: Some(NavAssumptions {
            time_system: "gps".to_string(),
            reference_frame: "ecef_wgs84".to_string(),
            clock_model: "receiver_clock_bias_drift_linear".to_string(),
            ephemeris_source: "broadcast_lnav".to_string(),
            frame_decode_mode: "lnav".to_string(),
            ephemeris_completeness: "sufficient".to_string(),
            ephemeris_count: used_sat_count,
        }),
        refusal_class: None,
        artifact_id: format!("nav-epoch-{epoch_idx:010}"),
        source_observation_epoch_id: format!("obs-epoch-{epoch_idx:010}"),
        explain_decision: "accepted".to_string(),
        explain_reasons: vec!["fixture".to_string()],
        provenance: None,
        sat_count: used_sat_count,
        used_sat_count,
        rejected_sat_count: 0,
        hdop: Some(pdop),
        vdop: Some(pdop),
        gdop: Some(pdop),
        tdop: Some(pdop / 2.0),
        stability_signature: format!("navsig:v2:fixture:{epoch_idx}"),
        stability_signature_version: bijux_gnss_core::api::NAV_OUTPUT_STABILITY_SIGNATURE_VERSION,
    }
}

fn with_integrity_support(mut solution: NavSolutionEpoch) -> NavSolutionEpoch {
    solution.integrity_hpl_m = Some(10.0);
    solution.integrity_vpl_m = Some(12.0);
    solution
}

fn dual_frequency_epoch(epoch_idx: u64, sats: Vec<ObsSatellite>) -> ObsEpoch {
    ObsEpoch {
        t_rx_s: bijux_gnss_core::api::Seconds(epoch_idx as f64),
        source_time: ReceiverSampleTrace::from_sample_index(epoch_idx, 1.0),
        gps_week: None,
        tow_s: None,
        epoch_idx,
        discontinuity: false,
        valid: true,
        processing_ms: None,
        role: ReceiverRole::Rover,
        sats,
        decision: ObservationEpochDecision::Accepted,
        decision_reason: Some("accepted_observables_present".to_string()),
        manifest: None,
    }
}

fn dual_frequency_satellite(
    band: SignalBand,
    code: SignalCode,
    code_lock: bool,
    carrier_lock: bool,
) -> ObsSatellite {
    dual_frequency_satellite_for_constellation(
        Constellation::Gps,
        band,
        code,
        code_lock,
        carrier_lock,
    )
}

fn dual_frequency_satellite_for_constellation(
    constellation: Constellation,
    band: SignalBand,
    code: SignalCode,
    code_lock: bool,
    carrier_lock: bool,
) -> ObsSatellite {
    ObsSatellite {
        signal_id: SigId { sat: SatId { constellation, prn: 12 }, band, code },
        pseudorange_m: Meters(22_000_000.0),
        pseudorange_var_m2: 1.0,
        carrier_phase_cycles: Cycles(200.0),
        carrier_phase_var_cycles2: 0.1,
        doppler_hz: Hertz(15.0),
        doppler_var_hz2: 1.0,
        cn0_dbhz: 45.0,
        lock_flags: LockFlags { code_lock, carrier_lock, bit_lock: true, cycle_slip: false },
        multipath_suspect: false,
        observation_status: ObservationStatus::Accepted,
        observation_reject_reasons: Vec::new(),
        elevation_deg: Some(45.0),
        azimuth_deg: Some(120.0),
        weight: Some(1.0),
        timing: None,
        error_model: None,
        metadata: ObsMetadata {
            tracking_mode: "scalar".to_string(),
            integration_ms: 1,
            lock_quality: 1.0,
            smoothing_window: 0,
            smoothing_age: 0,
            smoothing_resets: 0,
            signal: signal_registry(constellation, band, code)
                .expect("dual-frequency signal must exist")
                .spec,
            doppler_model: "tracked_carrier_hz_minus_intermediate_freq".to_string(),
            observation_lock_state: "locked".to_string(),
            ..ObsMetadata::default()
        },
    }
}

fn dual_frequency_satellite_with_phase(
    band: SignalBand,
    code: SignalCode,
    phase_m: f64,
) -> ObsSatellite {
    let mut satellite = dual_frequency_satellite(band, code, true, true);
    let wavelength_m = carrier_wavelength_m(satellite.metadata.signal.carrier_hz).0;
    satellite.carrier_phase_cycles = Cycles(phase_m / wavelength_m);
    satellite
}

fn carrier_smoothed_code_satellite(smoothing_age: u32, cycle_slip: bool) -> ObsSatellite {
    let mut satellite = dual_frequency_satellite(SignalBand::L1, SignalCode::Ca, true, true);
    satellite.lock_flags.cycle_slip = cycle_slip;
    satellite.metadata.smoothing_window = 8;
    satellite.metadata.smoothing_age = smoothing_age;
    satellite
}

fn fixture_track(epoch_idx: u64, lock_ratio: f64) -> TrackingResult {
    let sat = SatId { constellation: Constellation::Gps, prn: 1 };
    let total = 8u64;
    let locked = ((total as f64) * lock_ratio).round() as u64;
    let mut epochs = Vec::new();
    for idx in 0..total {
        epochs.push(TrackEpoch {
            epoch: Epoch { index: epoch_idx },
            sample_index: idx,
            sat,
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            glonass_frequency_channel: None,
            prompt_i: 1.0,
            prompt_q: 0.0,
            carrier_hz: Hertz(0.0),
            code_rate_hz: Hertz(1_023_000.0),
            code_phase_samples: Chips(0.0),
            lock: idx < locked,
            cn0_dbhz: 35.0,
            pll_lock: idx < locked,
            dll_lock: idx < locked,
            fll_lock: idx < locked,
            cycle_slip: false,
            nav_bit_lock: false,
            dll_err: 0.0,
            pll_err: 0.0,
            fll_err: 0.0,
            anti_false_lock: false,
            cycle_slip_reason: None,
            lock_state: "tracking".to_string(),
            lock_state_reason: None,
            processing_ms: None,
            ..TrackEpoch::default()
        });
    }
    TrackingResult {
        sat,
        carrier_hz: 0.0,
        code_phase_samples: 0.0,
        acquisition_hypothesis: "accepted".to_string(),
        acquisition_score: 1.0,
        acquisition_code_phase_samples: 0,
        acquisition_carrier_hz: 0.0,
        acq_to_track_state: "tracking".to_string(),
        epochs,
        transitions: Vec::new(),
    }
}

#[test]
fn science_fixture_integrity_classes_are_deterministic() {
    let fixture_path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../bijux-gnss-receiver/tests/data/validation/science_integrity_classification.json");
    let cases: Vec<ScienceFixtureCase> =
        serde_json::from_str(&std::fs::read_to_string(fixture_path).expect("fixture"))
            .expect("cases");
    for (idx, case) in cases.iter().enumerate() {
        let solution = with_integrity_support(fixture_solution(
            idx as u64,
            case.pdop,
            case.rms_m,
            case.used_sat_count,
        ));
        let track = fixture_track(idx as u64, case.lock_ratio);
        let report = build_validation_report(
            &[track],
            &[],
            &[solution],
            &[],
            1.0,
            false,
            Vec::new(),
            ValidationSciencePolicy::default(),
        )
        .expect("report");
        let class = report.integrity.first().expect("integrity row");
        let expected = match case.expected_class.as_str() {
            "missing_integrity_evidence" => NavIntegrityClass::IntegrityEvidenceMissing,
            "weak_geometry" => NavIntegrityClass::WeakGeometry,
            "suspicious_residuals" => NavIntegrityClass::SuspiciousResiduals,
            "unstable_lock" => NavIntegrityClass::UnstableLock,
            other => panic!("unsupported expected class: {other}"),
        };
        assert_eq!(class.class, expected, "case {}", case.name);
    }
}

#[test]
fn validation_policy_marks_high_gdop_as_weak_geometry() {
    let solution = with_integrity_support(fixture_solution(0, 2.0, 1.0, 4));
    let report = build_validation_report(
        &[fixture_track(0, 1.0)],
        &[],
        &[NavSolutionEpoch { gdop: Some(20.0), ..solution }],
        &[],
        1.0,
        false,
        Vec::new(),
        ValidationSciencePolicy::default(),
    )
    .expect("report");

    assert_eq!(
        report.integrity.first().expect("integrity row").class,
        NavIntegrityClass::WeakGeometry
    );
}

#[test]
fn validation_policy_requires_integrity_evidence_for_nominal_classification() {
    let solution = fixture_solution(0, 2.0, 1.0, 4);
    let report = build_validation_report(
        &[fixture_track(0, 1.0)],
        &[],
        &[solution],
        &[],
        1.0,
        false,
        Vec::new(),
        ValidationSciencePolicy::default(),
    )
    .expect("report");

    let integrity = report.integrity.first().expect("integrity row");
    assert_eq!(integrity.class, NavIntegrityClass::IntegrityEvidenceMissing);
    assert_eq!(integrity.reasons, vec!["missing_integrity_evidence".to_string()]);
}

#[test]
fn validation_policy_allows_nominal_when_integrity_evidence_is_present() {
    let solution = with_integrity_support(fixture_solution(0, 2.0, 1.0, 4));
    let report = build_validation_report(
        &[fixture_track(0, 1.0)],
        &[],
        &[solution],
        &[],
        1.0,
        false,
        Vec::new(),
        ValidationSciencePolicy::default(),
    )
    .expect("report");

    let integrity = report.integrity.first().expect("integrity row");
    assert_eq!(integrity.class, NavIntegrityClass::Nominal);
    assert!(integrity.reasons.is_empty());
}
