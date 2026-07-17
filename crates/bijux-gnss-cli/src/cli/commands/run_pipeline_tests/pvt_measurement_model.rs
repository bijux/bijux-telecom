#[test]
    fn pvt_command_uses_broadcast_satellite_clock_correction_from_rinex_navigation() {
        let (ephemerides, pvt_case) = broadcast_clock_pvt_case(2.75e-4);

        let corrected_solution = solve_pvt_case_with_rinex_nav(
            "broadcast_clock_rinex_corrected",
            &ephemerides,
            &pvt_case,
        );
        let zero_clock_solution = solve_pvt_case_with_rinex_nav(
            "broadcast_clock_rinex_zero_clock",
            &clear_broadcast_clock_parameters(&ephemerides),
            &pvt_case,
        );

        let corrected_error_m = position_error_3d_m(&corrected_solution, pvt_case.truth_ecef_m);
        let zero_clock_error_m = position_error_3d_m(&zero_clock_solution, pvt_case.truth_ecef_m);

        assert!(corrected_solution.valid);
        assert!(corrected_error_m < 5.0);
        assert!(zero_clock_error_m > corrected_error_m + 20.0);
        assert!(
            (corrected_solution.clock_bias_s.0 - pvt_case.receiver_clock_bias_s).abs() < 1.0e-9
        );
    }

    #[test]
    fn pvt_command_rinex_solution_prefers_earth_rotation_correction() {
        let ephemerides = sample_ephemerides();
        let pvt_case = sample_troposphere_free_pvt_case(&ephemerides, 2.75e-4);
        let solution = solve_pvt_case_with_rinex_nav_mode_and_troposphere(
            "earth_rotation_rinex",
            &ephemerides,
            &pvt_case,
            false,
            false,
        );

        let corrected_rms_m =
            pvt_residual_rms_with_earth_rotation_mode_m(&solution, &ephemerides, &pvt_case, true);
        let uncorrected_rms_m =
            pvt_residual_rms_with_earth_rotation_mode_m(&solution, &ephemerides, &pvt_case, false);

        assert!(solution.valid);
        assert!(position_error_3d_m(&solution, pvt_case.truth_ecef_m) < 5.0);
        assert!(corrected_rms_m < 1.0e-3);
        assert!(uncorrected_rms_m > corrected_rms_m + 1.0);
    }

    #[test]
    fn pvt_command_uses_broadcast_satellite_clock_correction_from_decoded_lnav() {
        let (ephemerides, pvt_case) = broadcast_clock_pvt_case(2.75e-4);

        let corrected_solution = solve_pvt_case_with_decoded_lnav_reports(
            "broadcast_clock_decoded_lnav_corrected",
            &ephemerides,
            &pvt_case,
        );
        let zero_clock_solution = solve_pvt_case_with_decoded_lnav_reports(
            "broadcast_clock_decoded_lnav_zero_clock",
            &clear_broadcast_clock_parameters(&ephemerides),
            &pvt_case,
        );

        let corrected_error_m = position_error_3d_m(&corrected_solution, pvt_case.truth_ecef_m);
        let zero_clock_error_m = position_error_3d_m(&zero_clock_solution, pvt_case.truth_ecef_m);

        assert!(corrected_solution.valid);
        assert!(corrected_error_m < 5.0);
        assert!(zero_clock_error_m > corrected_error_m + 20.0);
        assert!(
            (corrected_solution.clock_bias_s.0 - pvt_case.receiver_clock_bias_s).abs() < 1.0e-9
        );
    }

    #[test]
    fn pvt_command_decoded_lnav_solution_prefers_earth_rotation_correction() {
        let ephemerides = sample_ephemerides();
        let pvt_case = sample_troposphere_free_pvt_case(&ephemerides, 2.75e-4);
        let solution = solve_pvt_case_with_decoded_lnav_reports_and_troposphere(
            "earth_rotation_decoded_lnav",
            &ephemerides,
            &pvt_case,
            false,
        );

        let corrected_rms_m =
            pvt_residual_rms_with_earth_rotation_mode_m(&solution, &ephemerides, &pvt_case, true);
        let uncorrected_rms_m =
            pvt_residual_rms_with_earth_rotation_mode_m(&solution, &ephemerides, &pvt_case, false);

        assert!(solution.valid);
        assert!(position_error_3d_m(&solution, pvt_case.truth_ecef_m) < 5.0);
        assert!(corrected_rms_m < 1.0e-3);
        assert!(uncorrected_rms_m > corrected_rms_m + 1.0);
    }

    #[test]
    fn pvt_command_downweights_high_variance_satellite() {
        let ephs = sample_ephemerides();
        let low_variance_case = sample_pvt_case_with_adjustments(
            &ephs,
            2.75e-4,
            &[(
                5,
                SyntheticSatelliteAdjustment {
                    pseudorange_bias_m: 80.0,
                    pseudorange_sigma_m: SYNTHETIC_PVT_PSEUDORANGE_SIGMA_M,
                    ..SyntheticSatelliteAdjustment::default()
                },
            )],
        );
        let high_variance_case = sample_pvt_case_with_adjustments(
            &ephs,
            2.75e-4,
            &[(
                5,
                SyntheticSatelliteAdjustment {
                    pseudorange_bias_m: 80.0,
                    pseudorange_sigma_m: 200.0,
                    ..SyntheticSatelliteAdjustment::default()
                },
            )],
        );

        let low_variance_solution =
            solve_pvt_case_with_rinex_nav("low_variance_bias", &ephs, &low_variance_case);
        let high_variance_solution =
            solve_pvt_case_with_rinex_nav("high_variance_bias", &ephs, &high_variance_case);

        let low_variance_error_m =
            position_error_3d_m(&low_variance_solution, low_variance_case.truth_ecef_m);
        let high_variance_error_m =
            position_error_3d_m(&high_variance_solution, high_variance_case.truth_ecef_m);
        let provenance = high_variance_solution.provenance.as_ref().expect("nav provenance");

        assert!(high_variance_solution.valid);
        assert_eq!(provenance.solver_family, "wls_weighted");
        assert_eq!(provenance.weighting_mode, "elevation_sigma_weighted");
        assert!(high_variance_error_m < low_variance_error_m);
    }

    #[test]
    fn pvt_command_downweights_low_elevation_satellite() {
        let ephs = sample_ephemerides();
        let strong_signal_case = sample_pvt_case_with_adjustments(
            &ephs,
            2.75e-4,
            &[(
                5,
                SyntheticSatelliteAdjustment {
                    pseudorange_bias_m: 40.0,
                    pseudorange_sigma_m: SYNTHETIC_PVT_PSEUDORANGE_SIGMA_M,
                    elevation_deg: Some(70.0),
                    ..SyntheticSatelliteAdjustment::default()
                },
            )],
        );
        let weak_signal_case = sample_pvt_case_with_adjustments(
            &ephs,
            2.75e-4,
            &[(
                5,
                SyntheticSatelliteAdjustment {
                    pseudorange_bias_m: 40.0,
                    pseudorange_sigma_m: SYNTHETIC_PVT_PSEUDORANGE_SIGMA_M,
                    elevation_deg: Some(15.0),
                    ..SyntheticSatelliteAdjustment::default()
                },
            )],
        );

        let strong_signal_solution =
            solve_pvt_case_with_rinex_nav("strong_signal_bias", &ephs, &strong_signal_case);
        let weak_signal_solution =
            solve_pvt_case_with_rinex_nav("weak_signal_bias", &ephs, &weak_signal_case);

        let strong_signal_error_m =
            position_error_3d_m(&strong_signal_solution, strong_signal_case.truth_ecef_m);
        let weak_signal_error_m =
            position_error_3d_m(&weak_signal_solution, weak_signal_case.truth_ecef_m);
        let provenance = weak_signal_solution.provenance.as_ref().expect("nav provenance");

        assert!(weak_signal_solution.valid);
        assert_eq!(provenance.solver_family, "wls_weighted");
        assert_eq!(provenance.weighting_mode, "elevation_sigma_weighted");
        assert!(weak_signal_error_m < strong_signal_error_m);
    }

