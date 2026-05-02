use bijux_gnss_core::api::{
    ArtifactPayloadValidate, Constellation, Epoch, Meters, NavLifecycleState, NavResidual,
    NavSolutionEpoch, NavUncertaintyClass, SatId, Seconds, SolutionStatus, SolutionValidity,
};

fn sample_solution() -> NavSolutionEpoch {
    NavSolutionEpoch {
        epoch: Epoch { index: 1 },
        t_rx_s: Seconds(100_000.0),
        ecef_x_m: Meters(1_000_000.0),
        ecef_y_m: Meters(2_000_000.0),
        ecef_z_m: Meters(3_000_000.0),
        latitude_deg: 37.0,
        longitude_deg: -122.0,
        altitude_m: Meters(20.0),
        clock_bias_s: Seconds(0.0),
        clock_drift_s_per_s: 0.0,
        pdop: 1.5,
        rms_m: Meters(2.0),
        status: SolutionStatus::Converged,
        quality: SolutionStatus::Converged.quality_flag(),
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
        health: Vec::new(),
        isb: Vec::new(),
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
        lifecycle_state: NavLifecycleState::Converged,
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
        stability_signature: "navsig:v1:sample".to_string(),
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
fn nav_artifact_validation_warns_on_missing_refusal_metadata() {
    let mut solution = sample_solution();
    solution.status = SolutionStatus::Invalid;
    solution.valid = false;
    solution.quality = SolutionStatus::Invalid.quality_flag();
    solution.lifecycle_state = NavLifecycleState::Invalid;
    solution.refusal_class = None;
    solution.explain_decision.clear();
    let diagnostics = solution.validate_payload();
    assert!(diagnostics.iter().any(|event| event.code == "GNSS_NAV_REFUSAL_CLASS_MISSING"));
}
