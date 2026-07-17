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
fn validate_sidecar_rejects_missing_sample_rate_metadata() {
    let temp = temp_dir_path("missing_sample_rate_sidecar");
    fs::create_dir_all(&temp).expect("create temp dir");
    let sidecar_path = temp.join("broken.sidecar.toml");
    fs::write(
        &sidecar_path,
        r#"
format = "iq16_le"
intermediate_freq_hz = 0.0
capture_start_utc = "2026-07-09T00:00:00Z"
"#,
    )
    .expect("write sidecar");

    let output = run_bijux(
        &[
            "gnss",
            "validate-sidecar",
            "--unregistered-dataset",
            "--sidecar-file",
            sidecar_path.to_str().expect("sidecar path"),
        ],
        &repo_root(),
    );

    assert!(!output.status.success(), "command unexpectedly succeeded");
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(stderr.contains("sample_rate_hz"), "stderr did not mention sample_rate_hz: {stderr}");

    fs::remove_dir_all(&temp).expect("remove temp dir");
}

#[test]
fn validate_sidecar_rejects_missing_intermediate_frequency_metadata() {
    let temp = temp_dir_path("missing_intermediate_frequency_sidecar");
    fs::create_dir_all(&temp).expect("create temp dir");
    let sidecar_path = temp.join("broken.sidecar.toml");
    fs::write(
        &sidecar_path,
        r#"
format = "iq16_le"
sample_rate_hz = 5000000.0
capture_start_utc = "2026-07-09T00:00:00Z"
"#,
    )
    .expect("write sidecar");

    let output = run_bijux(
        &[
            "gnss",
            "validate-sidecar",
            "--unregistered-dataset",
            "--sidecar-file",
            sidecar_path.to_str().expect("sidecar path"),
        ],
        &repo_root(),
    );

    assert!(!output.status.success(), "command unexpectedly succeeded");
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(
        stderr.contains("intermediate_freq_hz"),
        "stderr did not mention intermediate_freq_hz: {stderr}"
    );

    fs::remove_dir_all(&temp).expect("remove temp dir");
}

#[test]
fn inspect_uses_sidecar_sample_rate_when_registry_omits_it() {
    let temp = temp_dir_path("registry_missing_sample_rate");
    let datasets_dir = temp.join("datasets");
    fs::create_dir_all(&datasets_dir).expect("create datasets dir");

    let iq_path = datasets_dir.join("demo.iq16");
    fs::write(&iq_path, [0x00u8, 0x80u8, 0xffu8, 0x7fu8]).expect("write iq data");

    let sidecar_path = datasets_dir.join("demo.sidecar.toml");
    fs::write(
        &sidecar_path,
        r#"
format = "iq16_le"
sample_rate_hz = 5000000.0
intermediate_freq_hz = 0.0
capture_start_utc = "2026-07-09T00:00:00Z"
quantization_bits = 16
"#,
    )
    .expect("write sidecar");

    let registry_path = datasets_dir.join("registry.toml");
    fs::write(
        &registry_path,
        format!(
            r#"
version = 1

[[entries]]
id = "demo"
path = "{}"
format = "iq16_le"
intermediate_freq_hz = 0.0
capture_start_utc = "2026-07-09T00:00:00Z"
expected_sats = []
expected_region = "N/A"
expected_time_utc = "N/A"
sidecar = "{}"
"#,
            iq_path.display(),
            sidecar_path.display()
        ),
    )
    .expect("write registry");

    let out_dir = temp.join("inspect-out");
    let output = run_bijux(
        &[
            "gnss",
            "inspect",
            "--dataset",
            "demo",
            "--report",
            "json",
            "--out",
            out_dir.to_str().expect("out dir"),
        ],
        &temp,
    );

    assert!(output.status.success(), "inspect failed: {}", String::from_utf8_lossy(&output.stderr));
    let report = fs::read_to_string(out_dir.join("inspect_report.json")).expect("read report");
    assert!(
        report.contains("\"sample_rate_hz\": 5000000.0"),
        "report missing sidecar sample rate: {report}"
    );

    fs::remove_dir_all(&temp).expect("remove temp dir");
}

#[test]
fn inspect_uses_sidecar_intermediate_frequency_when_registry_omits_it() {
    let temp = temp_dir_path("registry_missing_intermediate_frequency");
    let datasets_dir = temp.join("datasets");
    fs::create_dir_all(&datasets_dir).expect("create datasets dir");

    let iq_path = datasets_dir.join("demo.iq16");
    fs::write(&iq_path, [0x00u8, 0x80u8, 0xffu8, 0x7fu8]).expect("write iq data");

    let sidecar_path = datasets_dir.join("demo.sidecar.toml");
    fs::write(
        &sidecar_path,
        r#"
format = "iq16_le"
sample_rate_hz = 5000000.0
intermediate_freq_hz = 250000.0
capture_start_utc = "2026-07-09T00:00:00Z"
quantization_bits = 16
"#,
    )
    .expect("write sidecar");

    let registry_path = datasets_dir.join("registry.toml");
    fs::write(
        &registry_path,
        format!(
            r#"
version = 1

[[entries]]
id = "demo"
path = "{}"
format = "iq16_le"
sample_rate_hz = 5000000.0
capture_start_utc = "2026-07-09T00:00:00Z"
expected_sats = []
expected_region = "N/A"
expected_time_utc = "N/A"
sidecar = "{}"
"#,
            iq_path.display(),
            sidecar_path.display()
        ),
    )
    .expect("write registry");

    let out_dir = temp.join("inspect-out");
    let output = run_bijux(
        &[
            "gnss",
            "inspect",
            "--dataset",
            "demo",
            "--report",
            "json",
            "--out",
            out_dir.to_str().expect("out dir"),
        ],
        &temp,
    );

    assert!(output.status.success(), "inspect failed: {}", String::from_utf8_lossy(&output.stderr));
    let report = fs::read_to_string(out_dir.join("inspect_report.json")).expect("read report");
    assert!(
        report.contains("\"intermediate_freq_hz\": 250000.0"),
        "report missing sidecar intermediate frequency: {report}"
    );

    fs::remove_dir_all(&temp).expect("remove temp dir");
}

#[test]
fn inspect_rejects_unregistered_raw_iq_without_intermediate_frequency_metadata() {
    let temp = temp_dir_path("inspect_missing_intermediate_frequency");
    fs::create_dir_all(&temp).expect("create temp dir");

    let iq_path = temp.join("demo.iq16");
    fs::write(&iq_path, [0x00u8, 0x80u8, 0xffu8, 0x7fu8]).expect("write iq data");

    let sidecar_path = temp.join("broken.sidecar.toml");
    fs::write(
        &sidecar_path,
        r#"
format = "iq16_le"
sample_rate_hz = 5000000.0
capture_start_utc = "2026-07-09T00:00:00Z"
"#,
    )
    .expect("write sidecar");

    let out_dir = temp.join("inspect-out");
    let output = run_bijux(
        &[
            "gnss",
            "inspect",
            "--unregistered-dataset",
            "--file",
            iq_path.to_str().expect("iq path"),
            "--sidecar",
            sidecar_path.to_str().expect("sidecar path"),
            "--report",
            "json",
            "--out",
            out_dir.to_str().expect("out dir"),
        ],
        &repo_root(),
    );

    assert!(!output.status.success(), "inspect unexpectedly succeeded");
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(
        stderr.contains("intermediate_freq_hz"),
        "stderr did not mention intermediate_freq_hz: {stderr}"
    );

    fs::remove_dir_all(&temp).expect("remove temp dir");
}

#[test]
fn inspect_rejects_unregistered_raw_iq_without_sample_rate_metadata() {
    let temp = temp_dir_path("inspect_missing_sample_rate");
    fs::create_dir_all(&temp).expect("create temp dir");

    let iq_path = temp.join("demo.iq16");
    fs::write(&iq_path, [0x00u8, 0x80u8, 0xffu8, 0x7fu8]).expect("write iq data");

    let sidecar_path = temp.join("broken.sidecar.toml");
    fs::write(
        &sidecar_path,
        r#"
format = "iq16_le"
intermediate_freq_hz = 0.0
capture_start_utc = "2026-07-09T00:00:00Z"
"#,
    )
    .expect("write sidecar");

    let out_dir = temp.join("inspect-out");
    let output = run_bijux(
        &[
            "gnss",
            "inspect",
            "--unregistered-dataset",
            "--file",
            iq_path.to_str().expect("iq path"),
            "--sidecar",
            sidecar_path.to_str().expect("sidecar path"),
            "--report",
            "json",
            "--out",
            out_dir.to_str().expect("out dir"),
        ],
        &repo_root(),
    );

    assert!(!output.status.success(), "inspect unexpectedly succeeded");
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(stderr.contains("sample_rate_hz"), "stderr did not mention sample_rate_hz: {stderr}");

    fs::remove_dir_all(&temp).expect("remove temp dir");
}
