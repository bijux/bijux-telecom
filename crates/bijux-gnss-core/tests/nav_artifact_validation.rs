use bijux_gnss_core::api::{
    ArtifactPayloadValidate, Constellation, Epoch, Meters, NavConstellationResidualRms,
    NavLifecycleState, NavResidual, NavSolutionEpoch, NavUncertaintyClass, ReceiverSampleTrace,
    SatId, Seconds, SolutionStatus, SolutionValidity,
};

fn sample_solution() -> NavSolutionEpoch {
    NavSolutionEpoch {
        epoch: Epoch { index: 1 },
        t_rx_s: Seconds(100_000.0),
        source_time: ReceiverSampleTrace::from_sample_index(100_000, 1.0),
        ecef_x_m: Meters(1_000_000.0),
        ecef_y_m: Meters(2_000_000.0),
        ecef_z_m: Meters(3_000_000.0),
        position_covariance_ecef_m2: Some([[4.0, 0.5, 0.25], [0.5, 9.0, 0.75], [0.25, 0.75, 16.0]]),
        latitude_deg: 37.0,
        longitude_deg: -122.0,
        altitude_m: Meters(20.0),
        clock_bias_s: Seconds(0.0),
        clock_bias_m: Meters(0.0),
        clock_drift_s_per_s: 0.0,
        pdop: 1.5,
        pre_fit_residual_rms_m: Some(Meters(2.0)),
        post_fit_residual_rms_m: Some(Meters(2.0)),
        rms_m: Meters(2.0),
        status: SolutionStatus::CodeOnly,
        quality: SolutionStatus::CodeOnly.quality_flag(),
        validity: SolutionValidity::Stable,
        valid: true,
        processing_ms: None,
        residuals: vec![NavResidual {
            sat: SatId { constellation: Constellation::Gps, prn: 1 },
            residual_m: Meters(0.5),
            rejected: false,
            weight: Some(1.0),
            reject_reason: None,
        }],
        constellation_residual_rms: vec![NavConstellationResidualRms {
            constellation: Constellation::Gps,
            pre_fit_rms_m: Some(Meters(2.0)),
            post_fit_rms_m: Some(Meters(2.0)),
            pre_fit_sat_count: 1,
            post_fit_sat_count: 1,
        }],
        health: Vec::new(),
        isb: Vec::new(),
        sigma_e_m: Some(Meters(0.8)),
        sigma_n_m: Some(Meters(0.9)),
        sigma_u_m: Some(Meters(1.2)),
        horizontal_error_ellipse_major_axis_m: Some(Meters(1.1)),
        horizontal_error_ellipse_minor_axis_m: Some(Meters(0.7)),
        horizontal_error_ellipse_azimuth_deg: Some(32.0),
        sigma_h_m: Some(Meters(1.0)),
        sigma_v_m: Some(Meters(1.5)),
        innovation_rms_m: Some(0.5),
        normalized_innovation_rms: Some(0.5),
        normalized_innovation_max: Some(0.7),
        ekf_innovation_rms: Some(0.5),
        ekf_condition_number: Some(10.0),
        ekf_whiteness_ratio: Some(1.0),
        ekf_predicted_variance: Some(1.0),
        ekf_observed_variance: Some(1.1),
        integrity_hpl_m: Some(6.0),
        integrity_vpl_m: Some(9.0),
        model_version: bijux_gnss_core::api::NAV_SOLUTION_MODEL_VERSION,
        lifecycle_state: NavLifecycleState::CodeOnly,
        uncertainty_class: NavUncertaintyClass::Medium,
        assumptions: None,
        refusal_class: None,
        artifact_id: "nav-epoch-0000000001-sample".to_string(),
        source_observation_epoch_id: "obs-epoch-0000000001-sample".to_string(),
        explain_decision: "accepted".to_string(),
        explain_reasons: vec!["navigation_solution_usable".to_string()],
        provenance: None,
        sat_count: 1,
        used_sat_count: 1,
        rejected_sat_count: 0,
        hdop: Some(1.0),
        vdop: Some(1.2),
        gdop: Some(1.4),
        tdop: Some(0.8),
        stability_signature: "navsig:v2:sample".to_string(),
        stability_signature_version: bijux_gnss_core::api::NAV_OUTPUT_STABILITY_SIGNATURE_VERSION,
    }
}

#[test]
fn nav_artifact_validation_accepts_consistent_solution() {
    let solution = sample_solution();
    let diagnostics = solution.validate_payload();
    assert!(diagnostics.iter().all(|event| event.code != "GNSS_NAV_MODEL_VERSION_INVALID"
        && event.code != "GNSS_NAV_SAT_COUNTS_INVALID"));
}

#[test]
fn nav_artifact_validation_rejects_invalid_model_version_and_sat_counts() {
    let mut solution = sample_solution();
    solution.model_version = 0;
    solution.sat_count = 3;
    solution.used_sat_count = 1;
    solution.rejected_sat_count = 1;
    let diagnostics = solution.validate_payload();
    assert!(diagnostics.iter().any(|event| event.code == "GNSS_NAV_MODEL_VERSION_INVALID"));
    assert!(diagnostics.iter().any(|event| event.code == "GNSS_NAV_SAT_COUNTS_INVALID"));
}

#[test]
fn nav_artifact_validation_warns_when_non_usable_status_lacks_refusal_metadata() {
    let mut solution = sample_solution();
    solution.status = SolutionStatus::Refused;
    solution.valid = false;
    solution.quality = SolutionStatus::Refused.quality_flag();
    solution.lifecycle_state = NavLifecycleState::Refused;
    solution.refusal_class = None;
    solution.explain_decision.clear();
    let diagnostics = solution.validate_payload();
    assert!(diagnostics.iter().any(|event| event.code == "GNSS_NAV_REFUSAL_CLASS_MISSING"));
}

#[test]
fn nav_artifact_validation_warns_when_non_usable_status_claims_integrity_bounds() {
    let mut solution = sample_solution();
    solution.status = SolutionStatus::IntegrityFailed;
    solution.valid = false;
    solution.quality = SolutionStatus::IntegrityFailed.quality_flag();
    solution.lifecycle_state = NavLifecycleState::IntegrityFailed;
    solution.integrity_hpl_m = Some(6.0);
    solution.integrity_vpl_m = Some(9.0);
    let diagnostics = solution.validate_payload();
    assert!(diagnostics.iter().any(|event| event.code == "GNSS_NAV_INTEGRITY_CLAIMS_INVALID"));
}

#[test]
fn nav_artifact_validation_rejects_inconsistent_clock_bias_units() {
    let mut solution = sample_solution();
    solution.clock_bias_s = Seconds(1.0e-4);
    solution.clock_bias_m = Meters(1.0);
    let diagnostics = solution.validate_payload();
    assert!(diagnostics.iter().any(|event| event.code == "GNSS_NAV_CLOCK_BIAS_UNITS_INCONSISTENT"));
}

#[test]
fn nav_artifact_validation_rejects_non_finite_dops() {
    let mut solution = sample_solution();
    solution.tdop = Some(f64::NAN);
    let diagnostics = solution.validate_payload();
    assert!(diagnostics.iter().any(|event| event.code == "GNSS_NAV_DOPS_INVALID"));
}

#[test]
fn nav_artifact_validation_warns_on_post_fit_rms_mismatch() {
    let mut solution = sample_solution();
    solution.post_fit_residual_rms_m = Some(Meters(3.0));
    let diagnostics = solution.validate_payload();
    assert!(diagnostics.iter().any(|event| event.code == "GNSS_NAV_POST_FIT_RMS_MISMATCH"));
}

#[test]
fn nav_artifact_validation_warns_on_missing_position_covariance() {
    let mut solution = sample_solution();
    solution.position_covariance_ecef_m2 = None;
    let diagnostics = solution.validate_payload();
    assert!(diagnostics.iter().any(|event| event.code == "GNSS_NAV_POSITION_COVARIANCE_MISSING"));
}

#[test]
fn nav_artifact_validation_warns_on_missing_enu_position_sigmas() {
    let mut solution = sample_solution();
    solution.sigma_e_m = None;
    let diagnostics = solution.validate_payload();
    assert!(diagnostics.iter().any(|event| event.code == "GNSS_NAV_POSITION_SIGMA_ENU_MISSING"));
}

#[test]
fn nav_artifact_validation_warns_on_missing_horizontal_error_ellipse() {
    let mut solution = sample_solution();
    solution.horizontal_error_ellipse_major_axis_m = None;
    let diagnostics = solution.validate_payload();
    assert!(diagnostics
        .iter()
        .any(|event| event.code == "GNSS_NAV_HORIZONTAL_ERROR_ELLIPSE_MISSING"));
}

#[test]
fn nav_artifact_validation_rejects_constellation_residual_count_mismatch() {
    let mut solution = sample_solution();
    solution.constellation_residual_rms[0].post_fit_sat_count = 2;
    let diagnostics = solution.validate_payload();
    assert!(diagnostics
        .iter()
        .any(|event| event.code == "GNSS_NAV_CONSTELLATION_POST_FIT_COUNT_MISMATCH"));
}

#[test]
fn nav_artifact_validation_rejects_non_finite_position_covariance() {
    let mut solution = sample_solution();
    solution.position_covariance_ecef_m2 =
        Some([[1.0, 0.0, 0.0], [0.0, f64::NAN, 0.0], [0.0, 0.0, 1.0]]);
    let diagnostics = solution.validate_payload();
    assert!(diagnostics.iter().any(|event| event.code == "GNSS_NAV_POSITION_COVARIANCE_INVALID"));
}

#[test]
fn nav_artifact_validation_rejects_non_finite_enu_position_sigma() {
    let mut solution = sample_solution();
    solution.sigma_u_m = Some(Meters(f64::NAN));
    let diagnostics = solution.validate_payload();
    assert!(diagnostics.iter().any(|event| event.code == "GNSS_NAV_POSITION_SIGMA_ENU_INVALID"));
}

#[test]
fn nav_artifact_validation_rejects_non_finite_horizontal_error_ellipse() {
    let mut solution = sample_solution();
    solution.horizontal_error_ellipse_azimuth_deg = Some(f64::NAN);
    let diagnostics = solution.validate_payload();
    assert!(diagnostics
        .iter()
        .any(|event| event.code == "GNSS_NAV_HORIZONTAL_ERROR_ELLIPSE_INVALID"));
}
