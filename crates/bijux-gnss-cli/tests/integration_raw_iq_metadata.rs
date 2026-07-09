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
    assert!(
        stderr.contains("sample_rate_hz"),
        "stderr did not mention sample_rate_hz: {stderr}"
    );

    fs::remove_dir_all(&temp).expect("remove temp dir");
}
