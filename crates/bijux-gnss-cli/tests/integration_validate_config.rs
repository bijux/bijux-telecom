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
    assert!(
        out_dir.join("manifest.json").exists(),
        "validate-config did not emit a manifest"
    );

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
    assert!(
        out_dir.join("manifest.json").exists(),
        "validate-config did not emit a manifest"
    );

    fs::remove_dir_all(&out_dir).expect("remove output dir");
}
