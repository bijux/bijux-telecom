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
fn acquire_reports_prns_from_live_sky_excerpt() {
    let repo = repo_root();
    let out_dir = temp_dir_path("acquire_live_sky_excerpt");
    fs::create_dir_all(&out_dir).expect("create output dir");

    let output = run_bijux(
        &[
            "gnss",
            "acquire",
            "--dataset",
            "gps_l1_2022_03_27_excerpt",
            "--config",
            "configs/receiver_live_sky_gps_l1.toml",
            "--prn",
            "11,12,25,31,32",
            "--report",
            "json",
            "--out",
            out_dir.to_str().expect("out dir"),
        ],
        &repo,
    );

    assert!(output.status.success(), "acquire failed: {}", String::from_utf8_lossy(&output.stderr));

    let report_path = out_dir.join("acquire_report.json");
    let report: Value =
        serde_json::from_str(&fs::read_to_string(&report_path).expect("read acquire report"))
            .expect("parse acquire report");
    let reported_prns = report["reported_prns"].as_array().expect("reported_prns array");
    assert!(
        !reported_prns.is_empty(),
        "reported_prns should not be empty for the live-sky excerpt"
    );
    assert!(
        reported_prns.iter().any(|entry| {
            entry["sat"]["constellation"] == "Gps"
                && entry["sat"]["prn"] == 31
                && entry["classification"] == "accepted"
        }),
        "reported_prns did not include accepted G31: {report}"
    );
    assert!(out_dir.join("manifest.json").exists(), "acquire did not emit a manifest");

    fs::remove_dir_all(&out_dir).expect("remove output dir");
}
