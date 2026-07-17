#![allow(missing_docs)]

use serde_json::Value;
use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::time::{SystemTime, UNIX_EPOCH};

use bijux_gnss_receiver::api::{sim::SyntheticNavigationValidationScenario, ReceiverConfig};

fn repo_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(Path::parent)
        .expect("workspace root")
        .to_path_buf()
}

fn run_bijux(args: &[&str], cwd: &Path) -> std::process::Output {
    Command::new(env!("CARGO_BIN_EXE_bijux"))
        .args(args)
        .current_dir(cwd)
        .output()
        .expect("run bijux")
}

fn temp_dir_path(name: &str) -> PathBuf {
    let nanos = SystemTime::now().duration_since(UNIX_EPOCH).expect("unix epoch").as_nanos();
    std::env::temp_dir().join(format!("bijux_{}_{}_{}", name, std::process::id(), nanos))
}

fn write_bounded_navigation_validation_inputs(dir: &Path) -> (PathBuf, PathBuf) {
    let repo = repo_root();
    let mut config: ReceiverConfig = toml::from_str(
        &fs::read_to_string(repo.join("configs/receiver_low_rate.toml"))
            .expect("read receiver config"),
    )
    .expect("parse receiver config");
    let mut scenario: SyntheticNavigationValidationScenario = toml::from_str(
        &fs::read_to_string(repo.join("configs/scenarios/synthetic_navigation_accuracy.toml"))
            .expect("read synthetic navigation scenario"),
    )
    .expect("parse synthetic navigation scenario");

    scenario.duration_s = 0.08;

    config.sample_rate_hz = scenario.sample_rate_hz;
    config.intermediate_freq_hz = scenario.intermediate_freq_hz;
    config.code_freq_basis_hz = 1_023_000.0;
    config.code_length = 1023;
    config.acquisition.doppler_search_hz = 1_500;
    config.acquisition.doppler_step_hz = 250;
    config.tracking.max_channels = scenario.satellites.len();
    config.tracking.per_epoch_budget_ms = 100.0;
    config.navigation.hatch_window = 20;

    let scenario_path = dir.join("synthetic_navigation_validation.toml");
    let config_path = dir.join("receiver_navigation_validation.toml");
    fs::write(&scenario_path, toml::to_string(&scenario).expect("serialize bounded scenario"))
        .expect("write bounded scenario");
    fs::write(&config_path, toml::to_string(&config).expect("serialize bounded config"))
        .expect("write bounded config");
    (scenario_path, config_path)
}

#[test]
fn validate_synthetic_navigation_emits_truth_guided_accuracy_artifact() {
    let repo = repo_root();
    let out_dir = temp_dir_path("validate_synthetic_navigation");
    fs::create_dir_all(&out_dir).expect("create output dir");
    let (scenario_path, config_path) = write_bounded_navigation_validation_inputs(&out_dir);

    let output = run_bijux(
        &[
            "gnss",
            "validate-synthetic-navigation",
            "--scenario",
            scenario_path.to_str().expect("scenario path"),
            "--config",
            config_path.to_str().expect("config path"),
            "--report",
            "json",
            "--out",
            out_dir.to_str().expect("out dir"),
        ],
        &repo,
    );
    assert!(
        output.status.success(),
        "validate-synthetic-navigation failed: {}",
        String::from_utf8_lossy(&output.stderr)
    );

    let report: Value = serde_json::from_str(
        &fs::read_to_string(out_dir.join("validate_synthetic_navigation_report.json"))
            .expect("read synthetic navigation validation report"),
    )
    .expect("parse synthetic navigation validation report");
    let artifact: Value = serde_json::from_str(
        &fs::read_to_string(out_dir.join("artifacts/gnss_accuracy_artifact.json"))
            .expect("read gnss accuracy artifact"),
    )
    .expect("parse gnss accuracy artifact");

    assert_eq!(report["scenario_id"], "synthetic_navigation_accuracy");
    assert!(report["pass"].is_boolean());
    assert!(report["truth_coverage_ready"].is_boolean());
    assert_eq!(
        report["output_artifact"],
        out_dir.join("artifacts/gnss_accuracy_artifact.json").display().to_string()
    );

    assert_eq!(artifact["scenario_id"], "synthetic_navigation_accuracy");
    assert!(artifact["pass"].is_boolean());
    assert!(artifact["truth_coverage_ready"].is_boolean());
    assert_eq!(report["closure_ready"], true);
    assert_eq!(artifact["closure_ready"], true);
    assert_eq!(report["closure"]["pass"], true);
    assert_eq!(artifact["closure"]["pass"], true);
    assert_eq!(artifact["closure"]["applicable_stage_count"], 9);
    assert_eq!(artifact["closure"]["passed_stage_count"], 9);
    assert_eq!(artifact["closure"]["not_applicable_stage_count"], 1);
    assert_eq!(
        artifact["closure"]["stages"]
            .as_array()
            .expect("artifact closure stages")
            .iter()
            .map(|stage| stage["stage"].as_str().expect("closure stage name"))
            .collect::<Vec<_>>(),
        vec![
            "iq_input",
            "acquisition",
            "tracking",
            "navigation_data_timing",
            "observations",
            "satellite_states",
            "corrections",
            "estimator",
            "ambiguity_processing",
            "integrity_decision",
        ]
    );
    let ambiguity = artifact["closure"]["stages"]
        .as_array()
        .expect("artifact closure stages")
        .iter()
        .find(|stage| stage["stage"] == "ambiguity_processing")
        .expect("ambiguity closure stage");
    assert_eq!(ambiguity["status"], "not_applicable");
    assert_eq!(artifact["data_source"]["source_kind"], "synthetic_gps_l1_ca_navigation_validation");
    assert_eq!(artifact["reference_truth"]["truth_kind"], "synthetic_signal_and_position_truth");
    assert!(artifact["acquisition"]["summary"]["pass"].is_boolean());
    assert!(artifact["tracking"]["summary"]["pass"].is_boolean());
    assert!(artifact["observation"]["summary"]["pass"].is_boolean());
    assert!(artifact["pvt"]["summary"]["pass"].is_boolean());
    assert!(artifact["observation"]["report"]["truth_coverage_issues"].is_array());
    assert!(artifact["pvt"]["report"]["epochs"].is_array());

    fs::remove_dir_all(&out_dir).expect("remove output dir");
}
