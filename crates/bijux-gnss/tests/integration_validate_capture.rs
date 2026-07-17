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
fn validate_capture_reports_public_iq_tracking_and_position_attempts() {
    let repo = repo_root();
    let out_dir = temp_dir_path("validate_capture_public_excerpt");
    fs::create_dir_all(&out_dir).expect("create output dir");

    let output = run_bijux(
        &[
            "gnss",
            "validate-capture",
            "--dataset",
            "gps_l1_2022_03_27_excerpt",
            "--config",
            "configs/receiver_live_sky_gps_l1.toml",
            "--eph",
            "datasets/recorded/gps_l1_2022_03_27_broadcast_nav.22n",
            "--prn",
            "11,12,25,31,32",
            "--report",
            "json",
            "--out",
            out_dir.to_str().expect("out dir"),
        ],
        &repo,
    );

    assert!(
        output.status.success(),
        "validate-capture failed: {}",
        String::from_utf8_lossy(&output.stderr)
    );

    let report_path = out_dir.join("validate_capture_report.json");
    let report: Value =
        serde_json::from_str(&fs::read_to_string(&report_path).expect("read capture report"))
            .expect("parse capture report");

    let reported_prns =
        report["acquisition"]["reported_prns"].as_array().expect("reported_prns array");
    assert!(
        reported_prns.iter().any(|entry| {
            entry["sat"]["constellation"] == "Gps"
                && entry["sat"]["prn"] == 31
                && entry["classification"] == "accepted"
        }),
        "expected accepted G31 in acquisition report: {report}"
    );

    let tracked_prns = report["tracked_prns"].as_array().expect("tracked_prns array");
    assert!(
        tracked_prns
            .iter()
            .any(|entry| { entry["sat"]["constellation"] == "Gps" && entry["sat"]["prn"] == 31 }),
        "expected G31 in tracked PRNs: {report}"
    );

    let attempted_epochs =
        report["navigation_attempts"]["attempted_epochs"].as_u64().expect("attempted epochs");
    let position_attempts = report["position_attempts"].as_array().expect("position attempts");
    assert!(attempted_epochs > 0, "expected at least one navigation attempt: {report}");
    assert_eq!(attempted_epochs as usize, position_attempts.len());
    assert!(
        report["navigation_attempts"]["refusal_counts"]["InconsistentObservations"]
            .as_u64()
            .unwrap_or(0)
            > 0,
        "expected explicit refusal accounting for the public excerpt: {report}"
    );

    assert!(out_dir.join("manifest.json").exists(), "missing validate-capture manifest");
    assert!(
        out_dir.join("artifacts/validation_report.json").exists(),
        "missing validation report artifact"
    );
    let validation_report_path = out_dir.join("artifacts/validation_report.json");
    let validation_report: Value = serde_json::from_str(
        &fs::read_to_string(&validation_report_path).expect("read validation report artifact"),
    )
    .expect("parse validation report artifact");
    assert!(
        validation_report["dual_frequency_observations"]["observed_pairs"].as_u64().is_some(),
        "expected dual-frequency observation summary in validation report: {validation_report}"
    );
    assert!(
        validation_report["carrier_smoothed_code"]["observations"].as_u64().is_some(),
        "expected carrier-smoothed code validation summary in validation report: {validation_report}"
    );
    let carrier_smoothed_code_validation_path =
        out_dir.join("artifacts/carrier_smoothed_code_validation.json");
    assert!(
        carrier_smoothed_code_validation_path.exists(),
        "missing carrier-smoothed code validation artifact"
    );
    let carrier_smoothed_code_validation: Value = serde_json::from_str(
        &fs::read_to_string(&carrier_smoothed_code_validation_path)
            .expect("read carrier-smoothed code validation artifact"),
    )
    .expect("parse carrier-smoothed code validation artifact");
    assert_eq!(
        validation_report["carrier_smoothed_code"],
        carrier_smoothed_code_validation,
        "expected validate-capture report to surface the emitted carrier-smoothed code validation artifact"
    );
    assert!(
        out_dir.join("artifacts/validation_evidence_bundle.json").exists(),
        "missing validation evidence bundle"
    );
    assert!(
        out_dir.join("artifacts/nav_solution.jsonl").exists(),
        "missing navigation solution artifact"
    );

    fs::remove_dir_all(&out_dir).expect("remove output dir");
}
