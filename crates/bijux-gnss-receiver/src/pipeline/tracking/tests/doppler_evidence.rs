#[test]
fn doppler_estimator_consistency_accepts_aligned_residuals() {
    let consistency = super::doppler_estimator_consistency(0.0, 4.0, -3.0, 8.0);

    assert!(consistency.consistent, "{consistency:?}");
    assert_eq!(consistency.spread_hz, 7.0);
    assert_eq!(consistency.limit_hz, super::DOPPLER_ESTIMATOR_MIN_SPREAD_LIMIT_HZ);
}

#[test]
fn doppler_estimator_consistency_rejects_divergent_residuals() {
    let consistency = super::doppler_estimator_consistency(0.0, 12.0, 95.0, 8.0);

    assert!(!consistency.consistent, "{consistency:?}");
    assert_eq!(consistency.spread_hz, 95.0);
}

#[test]
fn doppler_estimator_consistency_rejects_non_finite_residuals() {
    let consistency = super::doppler_estimator_consistency(0.0, f64::NAN, 2.0, 8.0);

    assert!(!consistency.consistent, "{consistency:?}");
    assert!(consistency.spread_hz.is_infinite());
}

#[test]
fn doppler_estimator_uncertainty_sample_carries_estimator_spread() {
    let consistency = super::doppler_estimator_consistency(0.0, -10.0, 88.0, 8.0);
    let sample = super::doppler_estimator_uncertainty_sample_hz(consistency, 4.0);

    assert_eq!(sample, consistency.spread_hz);
}

#[test]
fn doppler_estimator_provenance_reports_divergence() {
    let consistency = super::doppler_estimator_consistency(0.0, 12.0, 95.0, 8.0);
    let provenance = super::doppler_estimator_provenance(consistency);

    assert!(provenance.contains("doppler_estimator_consistency=divergent"));
    assert!(provenance.contains("doppler_estimator_spread_hz=95.000"));
}

#[test]
fn tracking_provenance_segment_preserves_doppler_estimator_evidence() {
    let provenance = "tracking lock_detector_fll_hz=10.000 doppler_estimator_consistency=divergent doppler_estimator_spread_hz=95.000 doppler_estimator_limit_hz=25.000 doppler_loop_residual_hz=0.000 doppler_phase_rate_residual_hz=12.000 doppler_prompt_residual_hz=95.000 unrelated=true";
    let segment = super::tracking_provenance_segment(
        provenance,
        "doppler_estimator_consistency=",
        super::DOPPLER_ESTIMATOR_PROVENANCE_TOKEN_COUNT,
    )
    .expect("doppler estimator segment");

    assert_eq!(
            segment,
            "doppler_estimator_consistency=divergent doppler_estimator_spread_hz=95.000 doppler_estimator_limit_hz=25.000 doppler_loop_residual_hz=0.000 doppler_phase_rate_residual_hz=12.000 doppler_prompt_residual_hz=95.000"
        );
}
