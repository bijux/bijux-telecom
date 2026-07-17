    #[test]
    fn experiment_command_writes_artifacts_from_canonical_receiver_run() {
        let root = std::env::temp_dir().join(format!(
            "bijux-experiment-{}",
            SystemTime::now().duration_since(UNIX_EPOCH).expect("unix epoch").as_nanos()
        ));
        fs::create_dir_all(&root).expect("create test root");
        let scenario_path = root.join("scenario.toml");
        fs::write(
            &scenario_path,
            toml::to_string_pretty(&sample_experiment_scenario()).expect("serialize scenario"),
        )
        .expect("write scenario");
        let common = sample_common_args_with_profile(root.clone(), |profile| {
            profile.sample_rate_hz = 1_023_000.0;
            profile.intermediate_freq_hz = 0.0;
            profile.tracking.max_channels = 2;
        });

        handle_experiment(GnssCommand::Experiment {
            common: common.clone(),
            scenario: scenario_path,
            sweep: Vec::new(),
        })
        .expect("experiment command");

        let out_dir = artifacts_dir(&common, "experiment", None).expect("experiment artifacts dir");
        let summary = serde_json::from_str::<serde_json::Value>(
            &fs::read_to_string(out_dir.join("experiment_summary.json"))
                .expect("read experiment summary"),
        )
        .expect("parse experiment summary");
        let runs = summary["runs"].as_array().expect("experiment summary runs");

        assert_eq!(runs.len(), 1, "experiment command must emit one summary row without a sweep");
        assert!(
            out_dir.join("run_000").join("result.json").exists(),
            "experiment command must write a per-run result artifact",
        );
        assert!(
            out_dir.join("run_000").join("cn0.csv").exists(),
            "experiment command must write the per-run CN0 artifact",
        );
    }
