#[test]
    fn pvt_command_emits_all_dops_in_nav_solution_output() {
        let ephemerides = sample_ephemerides();
        let pvt_case = sample_pvt_case(&ephemerides, 2.75e-4);
        let root = std::env::temp_dir().join(format!(
            "bijux_pvt_output_dops_{}_{}",
            std::process::id(),
            SystemTime::now().duration_since(UNIX_EPOCH).expect("unix epoch").as_nanos()
        ));
        fs::create_dir_all(&root).expect("create test root");
        let obs_path = root.join("obs.jsonl");
        let eph_path = root.join("nav.rnx");
        fs::write(
            &obs_path,
            format!(
                "{}\n",
                serde_json::to_string(&pvt_case.obs_epoch).expect("serialize observation epoch")
            ),
        )
        .expect("write obs");
        write_rinex_nav(&eph_path, &ephemerides, true).expect("write rinex nav");

        let common = sample_common_args_with_troposphere(root.clone(), true);
        let out_dir = artifacts_dir(&common, "pvt", None).expect("artifacts dir");
        fs::create_dir_all(&out_dir).expect("create pvt artifacts dir");
        handle_pvt(GnssCommand::Pvt {
            common: common.clone(),
            obs: obs_path,
            eph: eph_path,
            ekf: false,
        })
        .expect("pvt command");

        let mut solutions =
            read_nav_solutions(&out_dir.join("pvt.jsonl")).expect("read nav solutions");
        assert_eq!(solutions.len(), 1);
        let solution = solutions.remove(0);
        let nav_output = fs::read_to_string(out_dir.join("nav_solution.jsonl"))
            .expect("read nav solution output");
        let payload: serde_json::Value =
            serde_json::from_str(nav_output.lines().next().expect("nav solution output line"))
                .expect("parse nav solution output");

        assert_eq!(payload["dops"]["pdop"].as_f64().expect("pdop"), solution.pdop);
        assert_eq!(
            payload["dops"]["hdop"].as_f64().expect("hdop"),
            solution.hdop.expect("solution hdop")
        );
        assert_eq!(
            payload["dops"]["vdop"].as_f64().expect("vdop"),
            solution.vdop.expect("solution vdop")
        );
        assert_eq!(
            payload["dops"]["gdop"].as_f64().expect("gdop"),
            solution.gdop.expect("solution gdop")
        );
        assert_eq!(
            payload["dops"]["tdop"].as_f64().expect("tdop"),
            solution.tdop.expect("solution tdop")
        );

        fs::remove_dir_all(root).expect("remove test root");
    }

