    include!("validation_detection_reports/acquisition_measurements.rs");

    include!("validation_detection_reports/acquisition_refinement.rs");


    include!("validation_detection_reports/tracking_detection_rates.rs");


    include!("validation_detection_reports/acquisition_detection_rates.rs");

    include!("validation_detection_reports/acquisition_signal_profiles.rs");

    #[test]
    fn observation_error_summary_tracks_bias_and_absolute_magnitude() {
        let summary =
            summarize_observation_errors(&[-2.0, 1.0, 3.0]).expect("error summary must exist");

        assert_eq!(summary.count, 3);
        assert!((summary.mean_error - (2.0 / 3.0)).abs() <= 1.0e-12, "{summary:?}");
        assert!((summary.median_abs_error - 2.0).abs() <= 1.0e-12, "{summary:?}");
        assert!((summary.rms_error - (14.0_f64 / 3.0).sqrt()).abs() <= 1.0e-12, "{summary:?}");
        assert!((summary.p95_abs_error - 3.0).abs() <= 1.0e-12, "{summary:?}");
        assert!((summary.max_abs_error - 3.0).abs() <= 1.0e-12, "{summary:?}");
    }
