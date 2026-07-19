use super::*;
use bijux_gnss_infra::api::core::{
    Epoch, Meters, NavLifecycleState, NavSolutionEpoch, NavUncertaintyClass, ReceiverSampleTrace,
    Seconds, SolutionStatus, SolutionValidity, NAV_OUTPUT_STABILITY_SIGNATURE_VERSION,
    NAV_SOLUTION_MODEL_VERSION,
};

fn sample_solution(ecef_x_m: f64, ecef_y_m: f64, ecef_z_m: f64) -> NavSolutionEpoch {
    NavSolutionEpoch {
        epoch: Epoch { index: 5 },
        t_rx_s: Seconds(5.0),
        source_time: ReceiverSampleTrace::from_sample_index(5, 1.0),
        ecef_x_m: Meters(ecef_x_m),
        ecef_y_m: Meters(ecef_y_m),
        ecef_z_m: Meters(ecef_z_m),
        position_covariance_ecef_m2: None,
        latitude_deg: 0.0,
        longitude_deg: 0.0,
        altitude_m: Meters(0.0),
        clock_bias_s: Seconds(0.0),
        clock_bias_m: Meters(0.0),
        clock_drift_s_per_s: 0.0,
        pdop: 1.0,
        pre_fit_residual_rms_m: None,
        post_fit_residual_rms_m: None,
        rms_m: Meters(0.5),
        status: SolutionStatus::CodeOnly,
        quality: SolutionStatus::CodeOnly.quality_flag(),
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
        model_version: NAV_SOLUTION_MODEL_VERSION,
        lifecycle_state: NavLifecycleState::CodeOnly,
        uncertainty_class: NavUncertaintyClass::Low,
        assumptions: None,
        refusal_class: None,
        artifact_id: "nav-epoch-0000000005-validate".to_string(),
        source_observation_epoch_id: "obs-epoch-0000000005-validate".to_string(),
        explain_decision: "accepted".to_string(),
        explain_reasons: vec!["validation_fixture".to_string()],
        provenance: None,
        sat_count: 4,
        used_sat_count: 4,
        rejected_sat_count: 0,
        hdop: Some(1.0),
        vdop: Some(1.0),
        gdop: Some(1.0),
        tdop: Some(0.5),
        stability_signature: "navsig:v2:validate".to_string(),
        stability_signature_version: NAV_OUTPUT_STABILITY_SIGNATURE_VERSION,
    }
}

fn with_integrity_support(mut solution: NavSolutionEpoch) -> NavSolutionEpoch {
    solution.integrity_hpl_m = Some(4.0);
    solution.integrity_vpl_m = Some(6.0);
    solution
}

#[test]
fn validation_evidence_bundle_includes_enu_reference_metrics() {
    let (x_ref, y_ref, z_ref) = bijux_gnss_infra::api::core::lla_to_ecef(0.0, 0.0, 0.0);
    let solution = sample_solution(x_ref + 3.0, y_ref + 1.0, z_ref + 2.0);
    let report = bijux_gnss_infra::api::receiver::build_validation_report(
        &[],
        &[],
        std::slice::from_ref(&solution),
        &[ValidationReferenceEpoch {
            epoch_idx: 5,
            t_rx_s: Some(5.0),
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
        bijux_gnss_infra::api::receiver::ValidationSciencePolicy::default(),
    )
    .expect("validation report");

    let evidence = validation_evidence_bundle(&[], &[solution], &report);

    assert_eq!(evidence["numerical"]["east_error_rms_m"], 1.0);
    assert_eq!(evidence["numerical"]["north_error_rms_m"], 2.0);
    assert_eq!(evidence["numerical"]["up_error_rms_m"], 3.0);
    assert_eq!(evidence["numerical"]["horiz_error_rms_m"], 5.0_f64.sqrt());
    assert_eq!(evidence["numerical"]["vert_error_rms_m"], 3.0);
    assert_eq!(evidence["numerical"]["error_3d_rms_m"], 14.0_f64.sqrt());
    assert_eq!(evidence["numerical"]["reference_match_count"], 1);
    assert_eq!(evidence["numerical"]["reference_error_3d_max_m"], 14.0_f64.sqrt());
    assert!(evidence["numerical"]["reference_error_3d_budget_m_max"].is_null());
    assert!(evidence["numerical"]["reference_error_3d_budget_pass"].is_null());
}

#[test]
fn validation_evidence_bundle_reports_reference_position_budget_status() {
    let (x_ref, y_ref, z_ref) = bijux_gnss_infra::api::core::lla_to_ecef(0.0, 0.0, 0.0);
    let solution = sample_solution(x_ref + 3.0, y_ref + 1.0, z_ref + 2.0);
    let report = bijux_gnss_infra::api::receiver::build_validation_report_with_budgets(
        &[],
        &[],
        std::slice::from_ref(&solution),
        &[ValidationReferenceEpoch {
            epoch_idx: 5,
            t_rx_s: Some(5.0),
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
        bijux_gnss_infra::api::receiver::ValidationSciencePolicy::default(),
        bijux_gnss_infra::api::receiver::ValidationBudgets {
            nav_min_lock_epochs: 0,
            reference_position_error_3d_m_max: Some(3.0),
            ..bijux_gnss_infra::api::receiver::ValidationBudgets::default()
        },
    )
    .expect("validation report");

    let evidence = validation_evidence_bundle(&[], &[solution], &report);

    assert_eq!(evidence["numerical"]["reference_error_3d_budget_m_max"], 3.0);
    assert_eq!(evidence["numerical"]["reference_error_3d_budget_pass"], false);
}

#[test]
fn validation_evidence_bundle_surfaces_missing_integrity_claim_support() {
    let solution = sample_solution(1.0, 2.0, 3.0);
    let report = bijux_gnss_infra::api::receiver::build_validation_report(
        &[],
        &[],
        std::slice::from_ref(&solution),
        &[],
        1.0,
        false,
        Vec::new(),
        bijux_gnss_infra::api::receiver::ValidationSciencePolicy::default(),
    )
    .expect("validation report");

    let evidence = validation_evidence_bundle(&[], &[solution], &report);

    assert_eq!(evidence["numerical"]["stable_integrity_evidence_missing_epochs"], 1);
    assert!(evidence["claim_evidence_guard"]["violations"]
        .as_array()
        .expect("violations array")
        .iter()
        .any(|value| value == "stable_solutions_missing_integrity_evidence:1/1"));
}

#[test]
fn validation_evidence_bundle_clears_integrity_gap_once_protection_levels_exist() {
    let solution = with_integrity_support(sample_solution(1.0, 2.0, 3.0));
    let report = bijux_gnss_infra::api::receiver::build_validation_report(
        &[],
        &[],
        std::slice::from_ref(&solution),
        &[],
        1.0,
        false,
        Vec::new(),
        bijux_gnss_infra::api::receiver::ValidationSciencePolicy::default(),
    )
    .expect("validation report");

    let evidence = validation_evidence_bundle(&[], &[solution], &report);

    assert_eq!(evidence["numerical"]["stable_integrity_evidence_missing_epochs"], 0);
    assert!(!evidence["claim_evidence_guard"]["violations"]
        .as_array()
        .expect("violations array")
        .iter()
        .any(|value| value == "stable_solutions_missing_integrity_evidence:1/1"));
}
