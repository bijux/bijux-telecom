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
fn nav_decode_recovers_navigation_bit_signs_from_track_artifact() {
    let repo = repo_root();
    let temp = temp_dir_path("nav_decode_bits");
    fs::create_dir_all(&temp).expect("create temp dir");
    let scenario_path = temp.join("nav_decode_bits.toml");
    fs::write(
        &scenario_path,
        r#"id = "nav_decode_bits"
sample_rate_hz = 4092000.0
intermediate_freq_hz = 0.0
duration_s = 0.080
seed = 24072026

[[satellites]]
sat = { constellation = "Gps", prn = 12 }
doppler_hz = 120.0
code_phase_chips = 144.375
carrier_phase_rad = 0.3
cn0_db_hz = 52.0
data_bit_flip = true
"#,
    )
    .expect("write scenario");

    let export_dir = temp.join("export");
    fs::create_dir_all(&export_dir).expect("create export dir");
    let export_output = run_bijux(
        &[
            "gnss",
            "export-synthetic-iq",
            "--scenario",
            scenario_path.to_str().expect("scenario path"),
            "--report",
            "json",
            "--out",
            export_dir.to_str().expect("export dir"),
        ],
        &repo,
    );
    assert!(
        export_output.status.success(),
        "export-synthetic-iq failed: {}",
        String::from_utf8_lossy(&export_output.stderr)
    );

    let artifact_dir = export_dir.join("artifacts");
    let iq_path = artifact_dir.join("nav_decode_bits.iq16");
    let sidecar_path = artifact_dir.join("nav_decode_bits.sidecar.toml");

    let track_dir = temp.join("track");
    fs::create_dir_all(&track_dir).expect("create track dir");
    let track_output = run_bijux(
        &[
            "gnss",
            "track",
            "--unregistered-dataset",
            "--file",
            iq_path.to_str().expect("iq path"),
            "--sidecar",
            sidecar_path.to_str().expect("sidecar path"),
            "--config",
            "configs/receiver_low_rate.toml",
            "--prn",
            "12",
            "--report",
            "json",
            "--out",
            track_dir.to_str().expect("track dir"),
        ],
        &repo,
    );
    assert!(
        track_output.status.success(),
        "track failed: {}",
        String::from_utf8_lossy(&track_output.stderr)
    );

    let nav_dir = temp.join("nav");
    fs::create_dir_all(&nav_dir).expect("create nav dir");
    let track_artifact = track_dir.join("artifacts").join("track.jsonl");
    let nav_output = run_bijux(
        &[
            "gnss",
            "nav",
            "decode",
            "--unregistered-dataset",
            "--track",
            track_artifact.to_str().expect("track artifact"),
            "--prn",
            "12",
            "--config",
            "configs/receiver_low_rate.toml",
            "--report",
            "json",
            "--out",
            nav_dir.to_str().expect("nav dir"),
        ],
        &repo,
    );
    assert!(
        nav_output.status.success(),
        "nav decode failed: {}",
        String::from_utf8_lossy(&nav_output.stderr)
    );

    let report: Value = serde_json::from_str(
        &fs::read_to_string(nav_dir.join("nav_decode_report.json")).expect("read nav report"),
    )
    .expect("parse nav report");
    let bit_start_ms = report["bit_start_ms"].as_u64().expect("bit_start_ms");
    let bit_signs = report["bit_signs"].as_array().expect("bit_signs");

    assert_eq!(report["sat"]["prn"], 12);
    assert!(bit_start_ms < 20, "bit_start_ms={bit_start_ms}");
    assert!(bit_signs.len() >= 3, "report={report}");
    assert!(
        bit_signs.iter().any(|sign| sign.as_i64() == Some(1))
            && bit_signs.iter().any(|sign| sign.as_i64() == Some(-1)),
        "recovered navigation bits did not contain both polarities in the alternating-bit scenario: report={report}"
    );

    fs::remove_dir_all(&temp).expect("remove temp dir");
}
