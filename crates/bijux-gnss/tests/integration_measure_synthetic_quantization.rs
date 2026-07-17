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
fn measure_synthetic_quantization_emits_artifact_and_inserts_float_reference() {
    let repo = repo_root();
    let out_dir = temp_dir_path("measure_synthetic_quantization");
    fs::create_dir_all(&out_dir).expect("create output dir");

    let output = run_bijux(
        &[
            "gnss",
            "measure-synthetic-quantization",
            "--scenario",
            "configs/scenarios/synthetic_iq_reference.toml",
            "--config",
            "configs/receiver_low_rate.toml",
            "--quantization",
            "signed16-bit,signed8-bit,signed4-bit,signed2-bit,bipolar1-bit",
            "--report",
            "json",
            "--out",
            out_dir.to_str().expect("out dir"),
        ],
        &repo,
    );
    assert!(
        output.status.success(),
        "measure-synthetic-quantization failed: {}",
        String::from_utf8_lossy(&output.stderr)
    );

    let report: Value = serde_json::from_str(
        &fs::read_to_string(out_dir.join("measure_synthetic_quantization_report.json"))
            .expect("read quantization measurement report"),
    )
    .expect("parse quantization measurement report");
    let artifact: Value = serde_json::from_str(
        &fs::read_to_string(out_dir.join("artifacts/synthetic_quantization_loss_artifact.json"))
            .expect("read quantization measurement artifact"),
    )
    .expect("parse quantization measurement artifact");

    assert_eq!(report["scenario_id"], "synthetic_iq_reference");
    assert_eq!(
        report["output_artifact"],
        out_dir.join("artifacts/synthetic_quantization_loss_artifact.json").display().to_string()
    );
    let measured_quantizations =
        report["measured_quantizations"].as_array().expect("measured_quantizations");
    assert_eq!(measured_quantizations.len(), 6);
    assert_eq!(measured_quantizations[0], "float32");
    assert_eq!(measured_quantizations[5], "bipolar1_bit");

    assert_eq!(artifact["scenario_id"], "synthetic_iq_reference");
    assert_eq!(artifact["reference_quantization"], "float32");
    let points = artifact["points"].as_array().expect("artifact points");
    assert_eq!(points.len(), 6);
    assert_eq!(points[0]["quantization"], "float32");
    assert_eq!(points[0]["sample_format"], "cf32_le");
    assert_eq!(points[1]["quantization"], "signed16_bit");
    assert_eq!(points[5]["quantization"], "bipolar1_bit");
    assert_eq!(points[5]["sample_format"], "iq8");

    fs::remove_dir_all(&out_dir).expect("remove output dir");
}
