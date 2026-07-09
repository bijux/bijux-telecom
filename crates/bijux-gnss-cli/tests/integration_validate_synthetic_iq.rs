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

fn export_reference_bundle(repo: &Path, export_dir: &Path) {
    fs::create_dir_all(export_dir).expect("create export dir");
    let output = run_bijux(
        &[
            "gnss",
            "export-synthetic-iq",
            "--scenario",
            "configs/scenarios/synthetic_iq_cn0_reference.toml",
            "--report",
            "json",
            "--out",
            export_dir.to_str().expect("export dir"),
        ],
        repo,
    );
    assert!(
        output.status.success(),
        "export-synthetic-iq failed: {}",
        String::from_utf8_lossy(&output.stderr)
    );
}

#[test]
fn validate_synthetic_iq_accepts_reference_cn0_bundle() {
    let repo = repo_root();
    let export_dir = temp_dir_path("export_synthetic_iq_cn0_reference");
    export_reference_bundle(&repo, &export_dir);

    let validate_dir = temp_dir_path("validate_synthetic_iq_cn0_reference");
    fs::create_dir_all(&validate_dir).expect("create validate dir");
    let artifacts_dir = export_dir.join("artifacts");
    let iq_path = artifacts_dir.join("synthetic_iq_cn0_reference.iq16");
    let sidecar_path = artifacts_dir.join("synthetic_iq_cn0_reference.sidecar.toml");
    let truth_path = artifacts_dir.join("synthetic_iq_cn0_reference.truth.json");

    let output = run_bijux(
        &[
            "gnss",
            "validate-synthetic-iq",
            "--unregistered-dataset",
            "--file",
            iq_path.to_str().expect("iq path"),
            "--sidecar",
            sidecar_path.to_str().expect("sidecar path"),
            "--truth",
            truth_path.to_str().expect("truth path"),
            "--config",
            "configs/receiver_low_rate.toml",
            "--report",
            "json",
            "--out",
            validate_dir.to_str().expect("validate dir"),
        ],
        &repo,
    );
    assert!(
        output.status.success(),
        "validate-synthetic-iq failed: {}",
        String::from_utf8_lossy(&output.stderr)
    );

    let report: Value = serde_json::from_str(
        &fs::read_to_string(validate_dir.join("validate_synthetic_iq_report.json"))
            .expect("read validation report"),
    )
    .expect("parse validation report");
    assert_eq!(report["validation"]["scenario_id"], "synthetic_iq_cn0_reference");
    assert_eq!(report["validation"]["pass"], true);
    let rows = report["validation"]["satellites"].as_array().expect("satellite rows");
    assert_eq!(rows.len(), 1);
    assert_eq!(rows[0]["sat"]["prn"], 7);
    assert_eq!(rows[0]["pass"], true);
    let cn0_delta_db = rows[0]["cn0_delta_db"].as_f64().expect("cn0 delta");
    assert!(cn0_delta_db.abs() <= 3.0, "cn0 delta out of tolerance: {cn0_delta_db}");
    assert!(validate_dir.join("manifest.json").exists(), "missing validation manifest");

    fs::remove_dir_all(&export_dir).expect("remove export dir");
    fs::remove_dir_all(&validate_dir).expect("remove validate dir");
}

#[test]
fn validate_synthetic_iq_rejects_too_tight_cn0_tolerance() {
    let repo = repo_root();
    let export_dir = temp_dir_path("export_synthetic_iq_cn0_tight_tolerance");
    export_reference_bundle(&repo, &export_dir);

    let validate_dir = temp_dir_path("validate_synthetic_iq_cn0_tight_tolerance");
    fs::create_dir_all(&validate_dir).expect("create validate dir");
    let artifacts_dir = export_dir.join("artifacts");
    let iq_path = artifacts_dir.join("synthetic_iq_cn0_reference.iq16");
    let sidecar_path = artifacts_dir.join("synthetic_iq_cn0_reference.sidecar.toml");
    let truth_path = artifacts_dir.join("synthetic_iq_cn0_reference.truth.json");

    let output = run_bijux(
        &[
            "gnss",
            "validate-synthetic-iq",
            "--unregistered-dataset",
            "--file",
            iq_path.to_str().expect("iq path"),
            "--sidecar",
            sidecar_path.to_str().expect("sidecar path"),
            "--truth",
            truth_path.to_str().expect("truth path"),
            "--config",
            "configs/receiver_low_rate.toml",
            "--tolerance-db-hz",
            "0.1",
            "--report",
            "json",
            "--out",
            validate_dir.to_str().expect("validate dir"),
        ],
        &repo,
    );
    assert!(
        !output.status.success(),
        "validate-synthetic-iq unexpectedly succeeded with tight tolerance"
    );
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(
        stderr.contains("synthetic IQ validation failed"),
        "stderr did not explain failure: {stderr}"
    );
    assert!(
        validate_dir.join("validate_synthetic_iq_report.json").exists(),
        "missing validation report after failure"
    );

    fs::remove_dir_all(&export_dir).expect("remove export dir");
    fs::remove_dir_all(&validate_dir).expect("remove validate dir");
}
