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

fn write_constant_iq8_capture(path: &Path, i: i8, q: i8, sample_count: usize) {
    let mut bytes = Vec::with_capacity(sample_count * 2);
    for _ in 0..sample_count {
        bytes.push(i as u8);
        bytes.push(q as u8);
    }
    fs::write(path, bytes).expect("write iq capture");
}

fn write_raw_iq_sidecar(path: &Path) {
    fs::write(
        path,
        r#"
format = "iq8"
sample_rate_hz = 5000000.0
intermediate_freq_hz = 0.0
capture_start_utc = "2026-07-09T00:00:00Z"
quantization_bits = 8
"#,
    )
    .expect("write sidecar");
}

fn load_json(path: &Path) -> Value {
    serde_json::from_str(&fs::read_to_string(path).expect("read json")).expect("parse json")
}

#[test]
fn acquire_reports_front_end_metrics_from_acquisition_window() {
    let temp = temp_dir_path("acquire_front_end_metrics");
    fs::create_dir_all(&temp).expect("create temp dir");

    let iq_path = temp.join("demo.iq8");
    write_constant_iq8_capture(&iq_path, 32, 0, 5_000);
    let sidecar_path = temp.join("demo.sidecar.toml");
    write_raw_iq_sidecar(&sidecar_path);

    let out_dir = temp.join("acquire-out");
    let output = run_bijux(
        &[
            "gnss",
            "acquire",
            "--unregistered-dataset",
            "--file",
            iq_path.to_str().expect("iq path"),
            "--sidecar",
            sidecar_path.to_str().expect("sidecar path"),
            "--prn",
            "1",
            "--report",
            "json",
            "--out",
            out_dir.to_str().expect("out dir"),
        ],
        &repo_root(),
    );

    assert!(output.status.success(), "acquire failed: {}", String::from_utf8_lossy(&output.stderr));
    let report = load_json(&out_dir.join("acquire_report.json"));
    let metrics = report
        .get("front_end_metrics")
        .expect("front_end_metrics present");
    assert_eq!(metrics.get("sample_count").and_then(Value::as_u64), Some(5_000));
    assert_eq!(metrics.get("i_mean").and_then(Value::as_f64), Some(0.25));
    assert_eq!(metrics.get("q_mean").and_then(Value::as_f64), Some(0.0));
    assert_eq!(metrics.get("rms").and_then(Value::as_f64), Some(0.25));
    assert_eq!(metrics.get("dc_imbalance").and_then(Value::as_f64), Some(1.0));

    fs::remove_dir_all(&temp).expect("remove temp dir");
}

#[test]
fn track_reports_front_end_metrics_from_acquisition_window() {
    let temp = temp_dir_path("track_front_end_metrics");
    fs::create_dir_all(&temp).expect("create temp dir");

    let iq_path = temp.join("demo.iq8");
    write_constant_iq8_capture(&iq_path, 32, 0, 5_000);
    let sidecar_path = temp.join("demo.sidecar.toml");
    write_raw_iq_sidecar(&sidecar_path);

    let out_dir = temp.join("track-out");
    let output = run_bijux(
        &[
            "gnss",
            "track",
            "--unregistered-dataset",
            "--file",
            iq_path.to_str().expect("iq path"),
            "--sidecar",
            sidecar_path.to_str().expect("sidecar path"),
            "--prn",
            "1",
            "--report",
            "json",
            "--out",
            out_dir.to_str().expect("out dir"),
        ],
        &repo_root(),
    );

    assert!(output.status.success(), "track failed: {}", String::from_utf8_lossy(&output.stderr));
    let report = load_json(&out_dir.join("track_report.json"));
    let metrics = report
        .get("front_end_metrics")
        .expect("front_end_metrics present");
    assert_eq!(metrics.get("sample_count").and_then(Value::as_u64), Some(5_000));
    assert_eq!(metrics.get("i_mean").and_then(Value::as_f64), Some(0.25));
    assert_eq!(metrics.get("q_mean").and_then(Value::as_f64), Some(0.0));
    assert_eq!(metrics.get("rms").and_then(Value::as_f64), Some(0.25));
    assert_eq!(metrics.get("dc_imbalance").and_then(Value::as_f64), Some(1.0));

    fs::remove_dir_all(&temp).expect("remove temp dir");
}
