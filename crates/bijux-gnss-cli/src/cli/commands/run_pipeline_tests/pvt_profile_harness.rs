fn solve_obs_epochs_with_rinex_nav_profile_and_troposphere(
        case_name: &str,
        ephs: &[GpsEphemeris],
        obs_epochs: &[ObsEpoch],
        ekf: bool,
        tropo_enabled: bool,
        configure: impl FnOnce(&mut ReceiverConfig),
    ) -> Vec<bijux_gnss_infra::api::core::NavSolutionEpoch> {
        let root = std::env::temp_dir().join(format!(
            "bijux_pvt_profile_{}_{}_{}",
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

        let common = sample_common_args_with_profile(root.clone(), |profile| {
            profile.navigation.tropo_enable = tropo_enabled;
            configure(profile);
        });
        let out_dir = artifacts_dir(&common, "pvt", None).expect("artifacts dir");
        fs::create_dir_all(&out_dir).expect("create pvt artifacts dir");
        handle_pvt(GnssCommand::Pvt { common: common.clone(), obs: obs_path, eph: eph_path, ekf })
            .expect("pvt command");

        let nav_path = out_dir.join("pvt.jsonl");
        let solutions = read_nav_solutions(&nav_path).expect("read nav solutions");
        fs::remove_dir_all(root).expect("remove test root");
        solutions
    }

