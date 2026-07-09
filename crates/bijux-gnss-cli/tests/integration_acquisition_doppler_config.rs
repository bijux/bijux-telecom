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
fn validate_config_rejects_unaligned_doppler_grid() {
    let repo = repo_root();
    let temp_dir = temp_dir_path("validate_unaligned_doppler_grid");
    fs::create_dir_all(&temp_dir).expect("create temp dir");
    let config_path = write_receiver_config(&temp_dir, 1_250, 500);
    let out_dir = temp_dir.join("validate_out");
    fs::create_dir_all(&out_dir).expect("create output dir");

    let output = run_bijux(
        &[
            "gnss",
            "validate-config",
            "--config",
            config_path.to_str().expect("config path"),
            "--unregistered-dataset",
            "--out",
            out_dir.to_str().expect("out dir"),
        ],
        &repo,
    );

    assert!(!output.status.success(), "validate-config unexpectedly succeeded");
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(
        stderr.contains(
            "acquisition.doppler_search_hz must be an integer multiple of acquisition.doppler_step_hz"
        ),
        "stderr did not mention the Doppler-grid constraint: {stderr}"
    );

    fs::remove_dir_all(&temp_dir).expect("remove temp dir");
}

#[test]
fn acquire_uses_profile_doppler_search_settings_by_default() {
    let repo = repo_root();
    let temp_dir = temp_dir_path("acquire_profile_doppler_grid");
    fs::create_dir_all(&temp_dir).expect("create temp dir");
    let config_path = write_receiver_config(&temp_dir, 1_500, 250);
    let (iq_path, sidecar_path) = export_synthetic_iq_bundle(&repo, &temp_dir);
    let acquire_dir = temp_dir.join("acquire_profile");
    fs::create_dir_all(&acquire_dir).expect("create acquire dir");

    let output = run_bijux(
        &[
            "gnss",
            "acquire",
            "--unregistered-dataset",
            "--file",
            iq_path.to_str().expect("iq path"),
            "--sidecar",
            sidecar_path.to_str().expect("sidecar path"),
            "--config",
            config_path.to_str().expect("config path"),
            "--prn",
            "3,7",
            "--report",
            "json",
            "--out",
            acquire_dir.to_str().expect("acquire dir"),
        ],
        &repo,
    );

    assert!(
        output.status.success(),
        "acquire failed: {}",
        String::from_utf8_lossy(&output.stderr)
    );
    assert_doppler_search_report(&acquire_dir.join("acquire_report.json"), 1_500, 250, 13);
    assert_doppler_search_artifact(&acquire_dir.join("artifacts").join("acq.jsonl"), 1_500, 250);

    fs::remove_dir_all(&temp_dir).expect("remove temp dir");
}

#[test]
fn acquire_can_override_profile_doppler_search_settings() {
    let repo = repo_root();
    let temp_dir = temp_dir_path("acquire_override_doppler_grid");
    fs::create_dir_all(&temp_dir).expect("create temp dir");
    let config_path = write_receiver_config(&temp_dir, 1_500, 250);
    let (iq_path, sidecar_path) = export_synthetic_iq_bundle(&repo, &temp_dir);
    let acquire_dir = temp_dir.join("acquire_override");
    fs::create_dir_all(&acquire_dir).expect("create acquire dir");

    let output = run_bijux(
        &[
            "gnss",
            "acquire",
            "--unregistered-dataset",
            "--file",
            iq_path.to_str().expect("iq path"),
            "--sidecar",
            sidecar_path.to_str().expect("sidecar path"),
            "--config",
            config_path.to_str().expect("config path"),
            "--doppler-search-hz",
            "1000",
            "--doppler-step-hz",
            "500",
            "--prn",
            "3,7",
            "--report",
            "json",
            "--out",
            acquire_dir.to_str().expect("acquire dir"),
        ],
        &repo,
    );

    assert!(
        output.status.success(),
        "acquire failed: {}",
        String::from_utf8_lossy(&output.stderr)
    );
    assert_doppler_search_report(&acquire_dir.join("acquire_report.json"), 1_000, 500, 5);
    assert_doppler_search_artifact(&acquire_dir.join("artifacts").join("acq.jsonl"), 1_000, 500);

    fs::remove_dir_all(&temp_dir).expect("remove temp dir");
}

fn write_receiver_config(temp_dir: &Path, doppler_search_hz: i64, doppler_step_hz: i64) -> PathBuf {
    let base_path = repo_root().join("configs/receiver_low_rate.toml");
    let mut config: toml::Value =
        toml::from_str(&fs::read_to_string(&base_path).expect("read base config"))
            .expect("parse base config");
    config["acquisition"]["doppler_search_hz"] = toml::Value::Integer(doppler_search_hz);
    config["acquisition"]["doppler_step_hz"] = toml::Value::Integer(doppler_step_hz);
    let config_path = temp_dir.join("receiver.toml");
    fs::write(&config_path, toml::to_string(&config).expect("serialize config"))
        .expect("write config");
    config_path
}

fn export_synthetic_iq_bundle(repo: &Path, temp_dir: &Path) -> (PathBuf, PathBuf) {
    let export_dir = temp_dir.join("export");
    fs::create_dir_all(&export_dir).expect("create export dir");
    let output = run_bijux(
        &[
            "gnss",
            "export-synthetic-iq",
            "--scenario",
            "configs/scenarios/synthetic_iq_reference.toml",
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
    let artifacts_dir = export_dir.join("artifacts");
    (
        artifacts_dir.join("synthetic_iq_reference.iq16"),
        artifacts_dir.join("synthetic_iq_reference.sidecar.toml"),
    )
}

fn assert_doppler_search_report(
    report_path: &Path,
    expected_search_hz: i64,
    expected_step_hz: i64,
    expected_bin_count: i64,
) {
    let report: Value =
        serde_json::from_str(&fs::read_to_string(report_path).expect("read acquire report"))
            .expect("parse acquire report");
    assert_eq!(report["doppler_search"]["max_search_hz"], expected_search_hz);
    assert_eq!(report["doppler_search"]["bin_width_hz"], expected_step_hz);
    assert_eq!(report["doppler_search"]["bin_count"], expected_bin_count);
}

fn assert_doppler_search_artifact(
    artifact_path: &Path,
    expected_search_hz: i64,
    expected_step_hz: i64,
) {
    let contents = fs::read_to_string(artifact_path).expect("read acq artifact");
    assert!(!contents.is_empty(), "expected acquisition artifact rows");
    for line in contents.lines() {
        let row: Value = serde_json::from_str(line).expect("parse acq artifact row");
        assert_eq!(
            row["payload"]["assumptions"]["doppler_search_hz"],
            expected_search_hz,
            "unexpected doppler_search_hz row: {row}"
        );
        assert_eq!(
            row["payload"]["assumptions"]["doppler_step_hz"],
            expected_step_hz,
            "unexpected doppler_step_hz row: {row}"
        );
    }
}
