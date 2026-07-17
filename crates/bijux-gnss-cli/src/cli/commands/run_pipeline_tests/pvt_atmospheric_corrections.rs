#[test]
    fn pvt_command_marks_wls_solution_ionosphere_uncorrected_without_broadcast_model() {
        let ephemerides = sample_ephemerides();
        let pvt_case = sample_troposphere_free_pvt_case(&ephemerides, 2.75e-4);
        let solution = solve_pvt_case_with_rinex_nav_mode_and_troposphere(
            "ionosphere_uncorrected_wls",
            &ephemerides,
            &pvt_case,
            false,
            false,
        );

        assert!(solution.explain_reasons.iter().any(|reason| reason == "ionosphere_uncorrected"));
    }

    #[test]
    fn pvt_command_applies_broadcast_ionosphere_correction_in_wls() {
        let ephemerides = sample_ephemerides();
        let klobuchar = sample_klobuchar_coefficients();
        let navigation = GpsBroadcastNavigationData {
            ephemerides: ephemerides.clone(),
            klobuchar: Some(klobuchar),
        };
        let clean_case = sample_troposphere_free_pvt_case(&ephemerides, 2.75e-4);
        let ionosphere_biased_case =
            add_klobuchar_delay_to_pvt_case(&clean_case, &ephemerides, klobuchar);

        let corrected_solution = solve_pvt_case_with_broadcast_navigation_data_and_troposphere(
            "broadcast_ionosphere_wls_corrected",
            &navigation,
            &ionosphere_biased_case,
            false,
            false,
        );
        let uncorrected_solution = solve_pvt_case_with_rinex_nav_mode_and_troposphere(
            "broadcast_ionosphere_wls_uncorrected",
            &ephemerides,
            &ionosphere_biased_case,
            false,
            false,
        );

        let corrected_error_m = position_error_3d_m(&corrected_solution, clean_case.truth_ecef_m);
        let uncorrected_error_m =
            position_error_3d_m(&uncorrected_solution, clean_case.truth_ecef_m);

        assert!(corrected_solution.valid);
        assert!(corrected_solution
            .explain_reasons
            .iter()
            .any(|reason| reason == "ionosphere_correction=klobuchar_broadcast"));
        assert!(corrected_error_m < 5.0);
        assert!(uncorrected_error_m > corrected_error_m + 3.0);
    }

    #[test]
    fn pvt_command_applies_rinex_broadcast_ionosphere_correction_in_wls() {
        let ephemerides = sample_ephemerides();
        let klobuchar = sample_klobuchar_coefficients();
        let navigation = GpsBroadcastNavigationData {
            ephemerides: ephemerides.clone(),
            klobuchar: Some(klobuchar),
        };
        let clean_case = sample_troposphere_free_pvt_case(&ephemerides, 2.75e-4);
        let ionosphere_biased_case =
            add_klobuchar_delay_to_pvt_case(&clean_case, &ephemerides, klobuchar);

        let corrected_solution =
            solve_pvt_case_with_rinex_broadcast_navigation_data_and_troposphere(
                "rinex_broadcast_ionosphere_wls_corrected",
                &navigation,
                &ionosphere_biased_case,
                false,
                false,
            );
        let uncorrected_solution = solve_pvt_case_with_rinex_nav_mode_and_troposphere(
            "rinex_broadcast_ionosphere_wls_uncorrected",
            &ephemerides,
            &ionosphere_biased_case,
            false,
            false,
        );

        let corrected_error_m = position_error_3d_m(&corrected_solution, clean_case.truth_ecef_m);
        let uncorrected_error_m =
            position_error_3d_m(&uncorrected_solution, clean_case.truth_ecef_m);

        assert!(corrected_solution.valid);
        assert!(corrected_solution
            .explain_reasons
            .iter()
            .any(|reason| reason == "ionosphere_correction=klobuchar_broadcast"));
        assert!(corrected_error_m < 5.0);
        assert!(uncorrected_error_m > corrected_error_m + 3.0);
    }

    #[test]
    fn pvt_command_marks_ekf_solution_ionosphere_uncorrected_without_broadcast_model() {
        let ephemerides = sample_ephemerides();
        let pvt_case = sample_troposphere_free_pvt_case(&ephemerides, 2.75e-4);
        let solution = solve_pvt_case_with_rinex_nav_mode_and_troposphere(
            "ionosphere_uncorrected_ekf",
            &ephemerides,
            &pvt_case,
            true,
            false,
        );

        assert!(solution.explain_reasons.iter().any(|reason| reason == "ionosphere_uncorrected"));
    }

    #[test]
    fn pvt_command_applies_broadcast_ionosphere_correction_in_ekf() {
        let ephemerides = sample_ephemerides();
        let klobuchar = sample_klobuchar_coefficients();
        let navigation = GpsBroadcastNavigationData {
            ephemerides: ephemerides.clone(),
            klobuchar: Some(klobuchar),
        };
        let clean_case = sample_troposphere_free_pvt_case(&ephemerides, 2.75e-4);
        let ionosphere_biased_case =
            add_klobuchar_delay_to_pvt_case(&clean_case, &ephemerides, klobuchar);

        let corrected_solution = solve_pvt_case_with_broadcast_navigation_data_and_troposphere(
            "broadcast_ionosphere_ekf_corrected",
            &navigation,
            &ionosphere_biased_case,
            true,
            false,
        );
        let uncorrected_solution = solve_pvt_case_with_rinex_nav_mode_and_troposphere(
            "broadcast_ionosphere_ekf_uncorrected",
            &ephemerides,
            &ionosphere_biased_case,
            true,
            false,
        );

        let corrected_error_m = position_error_3d_m(&corrected_solution, clean_case.truth_ecef_m);
        let uncorrected_error_m =
            position_error_3d_m(&uncorrected_solution, clean_case.truth_ecef_m);

        assert!(corrected_solution.valid);
        assert!(corrected_solution
            .explain_reasons
            .iter()
            .any(|reason| reason == "ionosphere_correction=klobuchar_broadcast"));
        assert!(corrected_error_m < uncorrected_error_m);
    }

    #[test]
    fn pvt_command_applies_rinex_broadcast_ionosphere_correction_in_ekf() {
        let ephemerides = sample_ephemerides();
        let klobuchar = sample_klobuchar_coefficients();
        let navigation = GpsBroadcastNavigationData {
            ephemerides: ephemerides.clone(),
            klobuchar: Some(klobuchar),
        };
        let clean_case = sample_troposphere_free_pvt_case(&ephemerides, 2.75e-4);
        let ionosphere_biased_case =
            add_klobuchar_delay_to_pvt_case(&clean_case, &ephemerides, klobuchar);

        let corrected_solution =
            solve_pvt_case_with_rinex_broadcast_navigation_data_and_troposphere(
                "rinex_broadcast_ionosphere_ekf_corrected",
                &navigation,
                &ionosphere_biased_case,
                true,
                false,
            );
        let uncorrected_solution = solve_pvt_case_with_rinex_nav_mode_and_troposphere(
            "rinex_broadcast_ionosphere_ekf_uncorrected",
            &ephemerides,
            &ionosphere_biased_case,
            true,
            false,
        );

        let corrected_error_m = position_error_3d_m(&corrected_solution, clean_case.truth_ecef_m);
        let uncorrected_error_m =
            position_error_3d_m(&uncorrected_solution, clean_case.truth_ecef_m);

        assert!(corrected_solution.valid);
        assert!(corrected_solution
            .explain_reasons
            .iter()
            .any(|reason| reason == "ionosphere_correction=klobuchar_broadcast"));
        assert!(corrected_error_m < uncorrected_error_m);
    }

    #[test]
    fn pvt_command_marks_wls_solution_troposphere_uncorrected_when_disabled() {
        let ephemerides = sample_ephemerides();
        let pvt_case = sample_troposphere_free_pvt_case(&ephemerides, 2.75e-4);
        let solution = solve_pvt_case_with_rinex_nav_mode_and_troposphere(
            "troposphere_uncorrected_wls",
            &ephemerides,
            &pvt_case,
            false,
            false,
        );

        assert!(solution.explain_reasons.iter().any(|reason| reason == "troposphere_uncorrected"));
    }

    #[test]
    fn pvt_command_applies_saastamoinen_troposphere_correction_in_wls() {
        let ephemerides = sample_ephemerides();
        let clean_case = sample_troposphere_free_pvt_case(&ephemerides, 2.75e-4);
        let troposphere_biased_case = add_saastamoinen_delay_to_pvt_case(&clean_case, &ephemerides);

        let corrected_solution = solve_pvt_case_with_rinex_nav(
            "saastamoinen_troposphere_wls_corrected",
            &ephemerides,
            &troposphere_biased_case,
        );
        let uncorrected_solution = solve_pvt_case_with_rinex_nav_mode_and_troposphere(
            "saastamoinen_troposphere_wls_uncorrected",
            &ephemerides,
            &troposphere_biased_case,
            false,
            false,
        );

        let corrected_error_m = position_error_3d_m(&corrected_solution, clean_case.truth_ecef_m);
        let uncorrected_error_m =
            position_error_3d_m(&uncorrected_solution, clean_case.truth_ecef_m);

        assert!(corrected_solution.valid);
        assert!(corrected_solution
            .explain_reasons
            .iter()
            .any(|reason| reason == "troposphere_correction=saastamoinen"));
        assert!(corrected_error_m < 5.0);
        assert!(uncorrected_error_m > corrected_error_m + 3.0);
    }

    #[test]
    fn pvt_command_applies_saastamoinen_troposphere_correction_in_ekf() {
        let ephemerides = sample_ephemerides();
        let clean_case = sample_troposphere_free_pvt_case(&ephemerides, 2.75e-4);
        let troposphere_biased_case = add_saastamoinen_delay_to_pvt_case(&clean_case, &ephemerides);

        let corrected_solution = solve_pvt_case_with_rinex_nav_mode(
            "saastamoinen_troposphere_ekf_corrected",
            &ephemerides,
            &troposphere_biased_case,
            true,
        );
        let uncorrected_solution = solve_pvt_case_with_rinex_nav_mode_and_troposphere(
            "saastamoinen_troposphere_ekf_uncorrected",
            &ephemerides,
            &troposphere_biased_case,
            true,
            false,
        );

        let corrected_error_m = position_error_3d_m(&corrected_solution, clean_case.truth_ecef_m);
        let uncorrected_error_m =
            position_error_3d_m(&uncorrected_solution, clean_case.truth_ecef_m);

        assert!(corrected_solution.valid);
        assert!(corrected_solution
            .explain_reasons
            .iter()
            .any(|reason| reason == "troposphere_correction=saastamoinen"));
        assert!(corrected_error_m < uncorrected_error_m);
    }