#![allow(missing_docs)]

use serde_json::Value;
use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::time::{SystemTime, UNIX_EPOCH};

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

#[test]
fn validate_synthetic_navigation_emits_truth_guided_accuracy_artifact() {
    let repo = repo_root();
    let out_dir = temp_dir_path("validate_synthetic_navigation");
    fs::create_dir_all(&out_dir).expect("create output dir");

    let output = run_bijux(
        &[
            "gnss",
            "validate-synthetic-navigation",
            "--scenario",
            "configs/scenarios/synthetic_navigation_accuracy.toml",
            "--config",
            "configs/receiver_low_rate.toml",
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
