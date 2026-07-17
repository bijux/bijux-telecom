#![allow(missing_docs)]

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
fn validate_config_accepts_live_sky_receiver_profile() {
    let repo = repo_root();
    let out_dir = temp_dir_path("validate_live_sky_config");
    fs::create_dir_all(&out_dir).expect("create output dir");

    let output = run_bijux(
        &[
            "gnss",
            "validate-config",
            "--config",
            "configs/receiver_live_sky_gps_l1.toml",
            "--unregistered-dataset",
            "--out",
            out_dir.to_str().expect("out dir"),
        ],
        &repo,
    );

    assert!(
        output.status.success(),
        "validate-config failed: {}",
        String::from_utf8_lossy(&output.stderr)
    );
    let stdout = String::from_utf8_lossy(&output.stdout);
    assert!(
        stdout.contains("config valid: configs/receiver_live_sky_gps_l1.toml"),
        "stdout did not confirm the validated profile: {stdout}"
    );
    assert!(out_dir.join("manifest.json").exists(), "validate-config did not emit a manifest");

    fs::remove_dir_all(&out_dir).expect("remove output dir");
}

#[test]
fn validate_config_accepts_low_rate_receiver_profile() {
    let repo = repo_root();
    let out_dir = temp_dir_path("validate_low_rate_config");
    fs::create_dir_all(&out_dir).expect("create output dir");

    let output = run_bijux(
        &[
            "gnss",
            "validate-config",
            "--config",
            "configs/receiver_low_rate.toml",
            "--unregistered-dataset",
            "--out",
            out_dir.to_str().expect("out dir"),
        ],
        &repo,
    );

    assert!(
        output.status.success(),
        "validate-config failed: {}",
        String::from_utf8_lossy(&output.stderr)
    );
    let stdout = String::from_utf8_lossy(&output.stdout);
    assert!(
        stdout.contains("config valid: configs/receiver_low_rate.toml"),
        "stdout did not confirm the validated profile: {stdout}"
    );
    assert!(out_dir.join("manifest.json").exists(), "validate-config did not emit a manifest");

    fs::remove_dir_all(&out_dir).expect("remove output dir");
}

#[test]
fn validate_config_rejects_unsupported_acquisition_coherent_integration() {
    let repo = repo_root();
    let temp_dir = temp_dir_path("validate_unsupported_coherent_integration");
    fs::create_dir_all(&temp_dir).expect("create temp dir");
    let config_path = write_receiver_config_with_integration_ms(&temp_dir, 3);
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
        stderr.contains("acquisition.integration_ms must be one of [1, 2, 5, 10, 20]"),
        "stderr did not mention the coherent integration constraint: {stderr}"
    );

    fs::remove_dir_all(&temp_dir).expect("remove temp dir");
}

fn write_receiver_config_with_integration_ms(temp_dir: &Path, integration_ms: i64) -> PathBuf {
    let base_path = repo_root().join("configs/receiver_low_rate.toml");
    let mut config: toml::Value =
        toml::from_str(&fs::read_to_string(&base_path).expect("read base config"))
            .expect("parse base config");
    config["acquisition"]["integration_ms"] = toml::Value::Integer(integration_ms);
    let config_path = temp_dir.join("receiver.toml");
    fs::write(&config_path, toml::to_string(&config).expect("serialize config"))
        .expect("write config");
    config_path
}
