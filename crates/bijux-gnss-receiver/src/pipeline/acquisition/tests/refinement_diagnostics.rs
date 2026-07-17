use super::*;

#[test]
fn parabolic_refinement_estimates_sub_bin_offset_from_neighbor_peaks() {
    let sat = SatId { constellation: Constellation::Gps, prn: 1 };
    let candidates = vec![
        candidate_for_search_window_test(sat, -250.0, 9.0),
        candidate_for_search_window_test(sat, 0.0, 16.0),
        candidate_for_search_window_test(sat, 250.0, 12.0),
    ];

    let refinement =
        estimate_acquisition_doppler_refinement(0.0, 0.0, &candidates, 250).expect("refinement");

    assert_eq!(refinement.method, "parabolic_peak");
    assert_eq!(refinement.coarse_carrier_hz.0, 0.0);
    assert!((refinement.offset_bins - 0.136_363_636_4).abs() < 1.0e-6);
    assert!((refinement.offset_hz - 34.090_909_1).abs() < 1.0e-6);
}

#[test]
fn parabolic_refinement_skips_search_edge_candidates() {
    let sat = SatId { constellation: Constellation::Gps, prn: 1 };
    let candidates = vec![
        candidate_for_search_window_test(sat, 0.0, 16.0),
        candidate_for_search_window_test(sat, 250.0, 12.0),
    ];

    let refinement = estimate_acquisition_doppler_refinement(0.0, 0.0, &candidates, 250);

    assert!(refinement.is_none());
}

#[test]
fn quadratic_surface_refinement_estimates_joint_peak_offsets() {
    let surface = LocalAcquisitionLikelihoodSurface {
        doppler_cross_section: [7.84, 10.0, 9.16],
        code_phase_cross_section: [9.38, 10.0, 8.62],
        values: [[7.62, 7.84, 6.06], [9.38, 10.0, 8.62], [8.14, 9.16, 8.18]],
    };

    let (doppler_offset_bins, code_phase_offset_samples) =
        estimate_quadratic_surface_peak_offsets(&surface).expect("joint refinement");

    assert!((doppler_offset_bins - 0.2).abs() < 1.0e-6);
    assert!((code_phase_offset_samples + 0.15).abs() < 1.0e-6);
}

#[test]
fn quadratic_surface_refinement_rejects_non_maximum_center() {
    let surface = LocalAcquisitionLikelihoodSurface {
        doppler_cross_section: [11.0, 10.0, 8.0],
        code_phase_cross_section: [8.0, 10.0, 7.0],
        values: [[7.0, 11.0, 6.5], [8.0, 10.0, 7.0], [6.0, 8.0, 5.5]],
    };

    assert!(estimate_quadratic_surface_peak_offsets(&surface).is_none());
}

#[test]
fn log_likelihood_covariance_tightens_for_sharper_surfaces() {
    let broad = estimate_log_likelihood_covariance_2x2(
        &LocalAcquisitionLikelihoodSurface {
            doppler_cross_section: [14.0, 16.0, 15.0],
            code_phase_cross_section: [14.0, 16.0, 15.0],
            values: [[13.0, 14.0, 13.5], [14.0, 16.0, 15.0], [13.5, 15.0, 14.0]],
        },
        1.0,
        250.0,
        1.0,
    )
    .expect("broad covariance");
    let sharp = estimate_log_likelihood_covariance_2x2(
        &LocalAcquisitionLikelihoodSurface {
            doppler_cross_section: [9.0, 16.0, 12.0],
            code_phase_cross_section: [9.0, 16.0, 12.0],
            values: [[8.0, 9.0, 8.5], [9.0, 16.0, 12.0], [8.5, 12.0, 10.0]],
        },
        1.0,
        250.0,
        1.0,
    )
    .expect("sharp covariance");

    assert!(sharp.doppler_variance_hz2 < broad.doppler_variance_hz2, "{sharp:?} {broad:?}");
    assert!(
        sharp.code_phase_variance_samples2 < broad.code_phase_variance_samples2,
        "{sharp:?} {broad:?}"
    );
}

#[test]
fn log_likelihood_covariance_reports_rate_variance_when_rate_axis_is_available() {
    let covariance = estimate_log_likelihood_covariance_3x3(
        &LocalAcquisitionLikelihoodVolume {
            values: [
                [[3.0, 4.0, 3.0], [5.0, 7.0, 5.0], [3.0, 4.0, 3.0]],
                [[4.0, 6.0, 4.0], [8.0, 16.0, 8.0], [4.0, 6.0, 4.0]],
                [[3.0, 4.0, 3.0], [5.0, 7.0, 5.0], [3.0, 4.0, 3.0]],
            ],
        },
        1.0,
        250.0,
        1.0,
        5_000.0,
    )
    .expect("rate-aware covariance");

    assert!(covariance.doppler_variance_hz2 > 0.0);
    assert!(covariance.code_phase_variance_samples2 > 0.0);
    assert!(
        covariance.doppler_rate_variance_hz2_per_s2.is_some_and(|variance| variance > 0.0),
        "{covariance:?}"
    );
}

#[test]
fn estimate_acquisition_uncertainty_skips_ambiguous_candidate() {
    let sat = SatId { constellation: Constellation::Gps, prn: 1 };
    let config = ReceiverPipelineConfig::default();
    let signal_model =
        AcquisitionSignalModel::for_sat_signal(sat, Some(SignalBand::L1), SignalCode::Ca, None)
            .expect("signal model lookup")
            .expect("gps l1ca signal model");
    let frame = noise_only_frame(
        config.sampling_freq_hz,
        signal_model.samples_per_code(config.sampling_freq_hz),
        0xA11CE,
    );
    let ambiguity_candidate = AcqResult {
        hypothesis: AcqHypothesis::Ambiguous,
        ..candidate_for_search_window_test(sat, 0.0, 4.0)
    };
    let uncertainty = estimate_acquisition_uncertainty(AcquisitionUncertaintyRequest {
        config: &config,
        frame: &frame,
        signal_model: &signal_model,
        candidate: &ambiguity_candidate,
        coherent_ms: 1,
        noncoherent: 1,
        doppler_step_hz: 250,
        doppler_rate_search_hz_per_s: 0,
        doppler_rate_step_hz_per_s: 250,
    });

    assert!(uncertainty.is_none());
}

#[test]
fn code_phase_refinement_estimates_sub_sample_offset_from_neighbor_bins() {
    let profile = [12.0, 16.0, 15.0];
    let (offset_samples, left, center, right) =
        estimate_parabolic_code_phase_offset_samples(&profile, 1).expect("refinement");

    assert!((offset_samples - 0.3).abs() < 1.0e-6);
    assert_eq!(left, 12.0);
    assert_eq!(center, 16.0);
    assert_eq!(right, 15.0);
}

#[test]
fn code_phase_refinement_wraps_negative_offsets_at_period_edges() {
    let offset_samples = -0.2;
    let refined_code_phase_samples = wrap_acquisition_code_phase_samples(0.0 + offset_samples, 4);

    assert!(offset_samples < 0.0);
    assert!((refined_code_phase_samples - 3.8).abs() < 1.0e-6);
}

#[test]
fn correlation_metrics_report_primary_and_secondary_peak_indices() {
    let metrics = correlation_metrics(&[1.0, 4.0, 2.0, 3.5, 1.5]);

    assert_eq!(metrics.peak_idx, 1);
    assert_eq!(metrics.second_idx, 3);
    assert_eq!(metrics.peak, 4.0);
    assert_eq!(metrics.second, 3.5);
    assert!((metrics.mean - 2.4).abs() < 1.0e-6);
}

#[test]
fn delayed_secondary_peak_diagnostic_ignores_main_lobe_neighbors() {
    let diagnostic =
        delayed_secondary_peak_diagnostic(&[1.0, 8.0, 7.6, 7.2, 1.4, 5.5, 1.2, 0.8], 1, 8, 4)
            .expect("delayed secondary peak");

    assert_eq!(diagnostic.secondary_code_phase_samples, 5);
    assert_eq!(diagnostic.delay_samples, 4);
}

#[test]
fn delayed_secondary_peak_diagnostic_skips_short_wraparound_alias() {
    let diagnostic =
        delayed_secondary_peak_diagnostic(&[6.0, 1.0, 0.8, 0.9, 1.1, 1.3, 1.2, 5.4], 7, 8, 4);

    assert!(diagnostic.is_none());
}

#[test]
fn multipath_candidate_reason_reports_delay_context() {
    let reason = multipath_candidate_reason(
        5.2,
        1.3,
        3.4,
        &DelayedSecondaryPeakDiagnostic { secondary_code_phase_samples: 1440, delay_samples: 240 },
        4092,
        1023,
        1.92,
    );

    assert!(reason.starts_with("multipath_suspect: delayed secondary peak"));
    assert!(reason.contains("+240 samples, 60.000 chips"));
    assert!(reason.contains("peak_second_ratio=1.300000"));
}
