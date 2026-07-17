fn add_saastamoinen_delay_to_pvt_case(
        pvt_case: &SyntheticPvtCase,
        ephs: &[GpsEphemeris],
    ) -> SyntheticPvtCase {
        let (lat_deg, lon_deg, alt_m) = ecef_to_geodetic(
            pvt_case.truth_ecef_m.0,
            pvt_case.truth_ecef_m.1,
            pvt_case.truth_ecef_m.2,
        );
        let receiver = bijux_gnss_infra::api::core::Llh { lat_deg, lon_deg, alt_m };
        let model = SaastamoinenModel;
        let mut biased_epoch = pvt_case.obs_epoch.clone();
        for sat in &mut biased_epoch.sats {
            let ephemeris = ephs
                .iter()
                .find(|ephemeris| ephemeris.sat == sat.signal_id.sat)
                .expect("matching ephemeris");
            let timing = sat.timing.expect("timing");
            let state = sat_state_gps_l1ca(
                ephemeris,
                timing.transmit_gps_time.tow_s,
                timing.signal_travel_time_s.0,
            );
            let (_azimuth_deg, elevation_deg) = elevation_azimuth_deg(
                pvt_case.truth_ecef_m.0,
                pvt_case.truth_ecef_m.1,
                pvt_case.truth_ecef_m.2,
                state.x_m,
                state.y_m,
                state.z_m,
            );
            let delay_m =
                model.delay_m(receiver, elevation_deg, Seconds(pvt_case.obs_epoch.t_rx_s.0));
            let delay_s = delay_m / SPEED_OF_LIGHT_MPS;
            sat.pseudorange_m.0 += delay_m;
            sat.timing = Some(ObsSignalTiming {
                signal_travel_time_s: Seconds(timing.signal_travel_time_s.0 + delay_s),
                transmit_gps_time: timing.transmit_gps_time.offset_seconds(-delay_s),
            });
        }
        SyntheticPvtCase {
            obs_epoch: biased_epoch,
            truth_ecef_m: pvt_case.truth_ecef_m,
            receiver_clock_bias_s: pvt_case.receiver_clock_bias_s,
        }
    }

    fn sparse_followup_obs_epoch(base: &ObsEpoch) -> ObsEpoch {
        let mut obs = base.clone();
        obs.epoch_idx += 1;
        obs.source_time = ReceiverSampleTrace::from_sample_index(
            base.source_time.sample_index + 1_000,
            base.source_time.sample_rate_hz,
        );
        obs.sats.truncate(3);
        obs
    }

    fn position_error_3d_m(
        solution: &bijux_gnss_infra::api::core::NavSolutionEpoch,
        truth_ecef_m: (f64, f64, f64),
    ) -> f64 {
        let dx = solution.ecef_x_m.0 - truth_ecef_m.0;
        let dy = solution.ecef_y_m.0 - truth_ecef_m.1;
        let dz = solution.ecef_z_m.0 - truth_ecef_m.2;
        (dx * dx + dy * dy + dz * dz).sqrt()
    }

    fn iterative_pseudorange_residual_with_earth_rotation_mode_m(
        eph: &GpsEphemeris,
        sat: &ObsSatellite,
        receiver_ecef_m: (f64, f64, f64),
        receiver_clock_bias_s: f64,
        receive_tow_s: f64,
        apply_earth_rotation: bool,
    ) -> f64 {
        let mut tau = sat
            .timing
            .map(|timing| timing.signal_travel_time_s.0)
            .unwrap_or(sat.pseudorange_m.0 / SPEED_OF_LIGHT_MPS);
        let mut predicted_pseudorange_m = 0.0;
        for _ in 0..10 {
            let state = sat_state_gps_l1ca(
                eph,
                receive_tow_s - tau,
                if apply_earth_rotation { tau } else { 0.0 },
            );
            let dx = receiver_ecef_m.0 - state.x_m;
            let dy = receiver_ecef_m.1 - state.y_m;
            let dz = receiver_ecef_m.2 - state.z_m;
            let range_m = (dx * dx + dy * dy + dz * dz).sqrt();
            predicted_pseudorange_m = range_m + receiver_clock_bias_s * SPEED_OF_LIGHT_MPS
                - state.clock_correction.bias_s * SPEED_OF_LIGHT_MPS;
            let next_tau = predicted_pseudorange_m / SPEED_OF_LIGHT_MPS;
            if (next_tau - tau).abs() < 1.0e-12 {
                break;
            }
            tau = next_tau;
        }
        sat.pseudorange_m.0 - predicted_pseudorange_m
    }

    fn pvt_residual_rms_with_earth_rotation_mode_m(
        solution: &bijux_gnss_infra::api::core::NavSolutionEpoch,
        ephs: &[GpsEphemeris],
        pvt_case: &SyntheticPvtCase,
        apply_earth_rotation: bool,
    ) -> f64 {
        let receive_tow_s = pvt_case.obs_epoch.tow_s.expect("receive tow").0;
        let residual_sum_squares_m2 = pvt_case
            .obs_epoch
            .sats
            .iter()
            .map(|sat| {
                let ephemeris = ephs
                    .iter()
                    .find(|ephemeris| ephemeris.sat == sat.signal_id.sat)
                    .expect("matching ephemeris");
                let residual_m = iterative_pseudorange_residual_with_earth_rotation_mode_m(
                    ephemeris,
                    sat,
                    (solution.ecef_x_m.0, solution.ecef_y_m.0, solution.ecef_z_m.0),
                    solution.clock_bias_s.0,
                    receive_tow_s,
                    apply_earth_rotation,
                );
                residual_m * residual_m
            })
            .sum::<f64>();
        (residual_sum_squares_m2 / pvt_case.obs_epoch.sats.len() as f64).sqrt()
    }

    fn solve_pvt_case_with_rinex_nav(
        case_name: &str,
        ephs: &[GpsEphemeris],
        pvt_case: &SyntheticPvtCase,
    ) -> bijux_gnss_infra::api::core::NavSolutionEpoch {
        solve_pvt_case_with_rinex_nav_mode_and_troposphere(case_name, ephs, pvt_case, false, true)
    }

    fn solve_pvt_case_with_rinex_nav_mode(
        case_name: &str,
        ephs: &[GpsEphemeris],
        pvt_case: &SyntheticPvtCase,
        ekf: bool,
    ) -> bijux_gnss_infra::api::core::NavSolutionEpoch {
        solve_pvt_case_with_rinex_nav_mode_and_troposphere(case_name, ephs, pvt_case, ekf, true)
    }

    fn solve_pvt_case_with_rinex_nav_mode_and_troposphere(
        case_name: &str,
        ephs: &[GpsEphemeris],
        pvt_case: &SyntheticPvtCase,
        ekf: bool,
        tropo_enabled: bool,
    ) -> bijux_gnss_infra::api::core::NavSolutionEpoch {
        let mut solutions = solve_obs_epochs_with_rinex_nav_mode_and_troposphere(
            case_name,
            ephs,
            &[pvt_case.obs_epoch.clone()],
            ekf,
            tropo_enabled,
        );
        assert_eq!(solutions.len(), 1);
        solutions.remove(0)
    }

    fn solve_pvt_case_with_rinex_nav_profile_and_troposphere(
        case_name: &str,
        ephs: &[GpsEphemeris],
        pvt_case: &SyntheticPvtCase,
        ekf: bool,
        tropo_enabled: bool,
        configure: impl FnOnce(&mut ReceiverConfig),
    ) -> bijux_gnss_infra::api::core::NavSolutionEpoch {
        let mut solutions = solve_obs_epochs_with_rinex_nav_profile_and_troposphere(
            case_name,
            ephs,
            &[pvt_case.obs_epoch.clone()],
            ekf,
            tropo_enabled,
            configure,
        );
        assert_eq!(solutions.len(), 1);
        solutions.remove(0)
    }

    fn solve_obs_epochs_with_rinex_nav_mode_and_troposphere(
        case_name: &str,
        ephs: &[GpsEphemeris],
        obs_epochs: &[ObsEpoch],
        ekf: bool,
        tropo_enabled: bool,
    ) -> Vec<bijux_gnss_infra::api::core::NavSolutionEpoch> {
        let root = std::env::temp_dir().join(format!(
            "bijux_pvt_{}_{}_{}",
            case_name,
            std::process::id(),
            SystemTime::now().duration_since(UNIX_EPOCH).expect("unix epoch").as_nanos()
        ));
        fs::create_dir_all(&root).expect("create test root");
        let obs_path = root.join("obs.jsonl");
        let eph_path = root.join("nav.rnx");
        fs::write(
            &obs_path,
            obs_epochs
                .iter()
                .map(|obs_epoch| {
                    serde_json::to_string(obs_epoch).expect("serialize observation epoch")
                })
                .collect::<Vec<_>>()
                .join("\n")
                + "\n",
        )
        .expect("write obs");
        write_rinex_nav(&eph_path, ephs, true).expect("write rinex nav");

        let common = sample_common_args_with_troposphere(root.clone(), tropo_enabled);
        let out_dir = artifacts_dir(&common, "pvt", None).expect("artifacts dir");
        fs::create_dir_all(&out_dir).expect("create pvt artifacts dir");
        handle_pvt(GnssCommand::Pvt { common: common.clone(), obs: obs_path, eph: eph_path, ekf })
            .expect("pvt command");

        let nav_path = out_dir.join("pvt.jsonl");
        let solutions = read_nav_solutions(&nav_path).expect("read nav solutions");
        fs::remove_dir_all(root).expect("remove test root");
        solutions
    }

