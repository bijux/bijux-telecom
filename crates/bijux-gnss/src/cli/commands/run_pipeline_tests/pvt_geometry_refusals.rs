#[test]
    fn pvt_command_refuses_sparse_followup_epoch_in_wls_mode() {
        let ephemerides = sample_ephemerides();
        let first_case = sample_pvt_case(&ephemerides, 2.75e-4);
        let second_obs = sparse_followup_obs_epoch(&first_case.obs_epoch);

        let solutions = solve_obs_epochs_with_rinex_nav_mode_and_troposphere(
            "sparse_followup_wls_refusal",
            &ephemerides,
            &[first_case.obs_epoch.clone(), second_obs],
            false,
            true,
        );

        assert_eq!(solutions.len(), 2);
        assert!(solutions[0].valid);
        assert_eq!(solutions[1].status, bijux_gnss_infra::api::core::SolutionStatus::Refused);
        assert_eq!(
            solutions[1].refusal_class,
            Some(bijux_gnss_infra::api::core::NavRefusalClass::InsufficientGeometry)
        );
        assert!(solutions[1].used_sat_count < 4);
        assert_eq!(
            solutions[1].sat_count,
            solutions[1].used_sat_count + solutions[1].rejected_sat_count
        );
        assert_eq!(solutions[1].ecef_x_m.0, 0.0);
        assert_eq!(solutions[1].ecef_y_m.0, 0.0);
        assert_eq!(solutions[1].ecef_z_m.0, 0.0);
        assert!(!solutions[1].valid);
    }

    #[test]
    fn pvt_command_refuses_sparse_followup_epoch_in_ekf_mode() {
        let ephemerides = sample_ephemerides();
        let first_case = sample_pvt_case(&ephemerides, 2.75e-4);
        let second_obs = sparse_followup_obs_epoch(&first_case.obs_epoch);

        let solutions = solve_obs_epochs_with_rinex_nav_mode_and_troposphere(
            "sparse_followup_ekf_refusal",
            &ephemerides,
            &[first_case.obs_epoch.clone(), second_obs],
            true,
            true,
        );

        assert_eq!(solutions.len(), 2);
        assert!(solutions[0].valid);
        assert_eq!(solutions[1].status, bijux_gnss_infra::api::core::SolutionStatus::Refused);
        assert_eq!(
            solutions[1].refusal_class,
            Some(bijux_gnss_infra::api::core::NavRefusalClass::InsufficientGeometry)
        );
        assert_eq!(solutions[1].sat_count, 3);
        assert_eq!(solutions[1].used_sat_count, 3);
        assert_eq!(solutions[1].ecef_x_m.0, 0.0);
        assert_eq!(solutions[1].ecef_y_m.0, 0.0);
        assert_eq!(solutions[1].ecef_z_m.0, 0.0);
        assert!(solutions[1]
            .explain_reasons
            .iter()
            .any(|reason| reason == "minimum_usable_satellites=4"));
        assert!(!solutions[1].valid);
    }

    #[test]
    fn pvt_command_refuses_wls_solution_when_gdop_exceeds_science_threshold() {
        let ephemerides = sample_ephemerides();
        let pvt_case = sample_pvt_case(&ephemerides, 2.75e-4);
        let baseline_solution = solve_pvt_case_with_rinex_nav_mode(
            "wls_geometry_threshold_baseline",
            &ephemerides,
            &pvt_case,
            false,
        );
        let baseline_gdop = baseline_solution.gdop.expect("baseline gdop");

        let refused_solution = solve_pvt_case_with_rinex_nav_profile_and_troposphere(
            "wls_geometry_threshold_refusal",
            &ephemerides,
            &pvt_case,
            false,
            true,
            |profile| {
                profile.navigation.science_thresholds.max_pdop = 100.0;
                profile.navigation.science_thresholds.max_gdop = baseline_gdop - 0.01;
            },
        );

        assert!(baseline_solution.valid);
        assert_eq!(refused_solution.status, bijux_gnss_infra::api::core::SolutionStatus::Refused);
        assert_eq!(
            refused_solution.refusal_class,
            Some(bijux_gnss_infra::api::core::NavRefusalClass::InsufficientGeometry)
        );
        assert!((refused_solution.gdop.expect("refused gdop") - baseline_gdop).abs() < 1.0e-9);
        assert!(refused_solution
            .explain_reasons
            .iter()
            .any(|reason| reason.starts_with("gdop_above_threshold:")));
        assert_eq!(refused_solution.ecef_x_m.0, 0.0);
        assert_eq!(refused_solution.ecef_y_m.0, 0.0);
        assert_eq!(refused_solution.ecef_z_m.0, 0.0);
        assert!(!refused_solution.valid);
    }

    #[test]
    fn pvt_command_refuses_ekf_solution_when_pdop_exceeds_science_threshold() {
        let ephemerides = sample_ephemerides();
        let pvt_case = sample_pvt_case(&ephemerides, 2.75e-4);
        let baseline_solution = solve_pvt_case_with_rinex_nav_mode(
            "ekf_geometry_threshold_baseline",
            &ephemerides,
            &pvt_case,
            true,
        );

        let refused_solution = solve_pvt_case_with_rinex_nav_profile_and_troposphere(
            "ekf_geometry_threshold_refusal",
            &ephemerides,
            &pvt_case,
            true,
            true,
            |profile| {
                profile.navigation.science_thresholds.max_pdop = baseline_solution.pdop - 0.01;
                profile.navigation.science_thresholds.max_gdop = 100.0;
            },
        );

        assert!(baseline_solution.valid);
        assert_eq!(refused_solution.status, bijux_gnss_infra::api::core::SolutionStatus::Refused);
        assert_eq!(
            refused_solution.refusal_class,
            Some(bijux_gnss_infra::api::core::NavRefusalClass::InsufficientGeometry)
        );
        assert!((refused_solution.pdop - baseline_solution.pdop).abs() < 1.0e-9);
        assert!(refused_solution
            .explain_reasons
            .iter()
            .any(|reason| reason.starts_with("pdop_above_threshold:")));
        assert_eq!(refused_solution.ecef_x_m.0, 0.0);
        assert_eq!(refused_solution.ecef_y_m.0, 0.0);
        assert_eq!(refused_solution.ecef_z_m.0, 0.0);
        assert!(!refused_solution.valid);
    }

