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
fn export_synthetic_iq_emits_truth_bundle_and_ingestable_capture() {
    let repo = repo_root();
    let export_dir = temp_dir_path("export_synthetic_iq");
    fs::create_dir_all(&export_dir).expect("create export dir");
    let scenario = "configs/scenarios/synthetic_iq_reference.toml";

    let export_output = run_bijux(
        &[
            "gnss",
            "export-synthetic-iq",
            "--scenario",
            scenario,
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

    let artifacts_dir = export_dir.join("artifacts");
    let iq_path = artifacts_dir.join("synthetic_iq_reference.iq16");
    let sidecar_path = artifacts_dir.join("synthetic_iq_reference.sidecar.toml");
    let truth_path = artifacts_dir.join("synthetic_iq_reference.truth.json");
    let scenario_copy_path = artifacts_dir.join("synthetic_iq_reference.scenario.toml");
    assert!(iq_path.exists(), "missing exported iq bundle");
    assert!(sidecar_path.exists(), "missing exported sidecar");
    assert!(truth_path.exists(), "missing exported truth file");
    assert!(scenario_copy_path.exists(), "missing scenario copy");

    let truth: Value =
        serde_json::from_str(&fs::read_to_string(&truth_path).expect("read truth"))
            .expect("parse truth");
    assert_eq!(truth["scenario_id"], "synthetic_iq_reference");
    assert_eq!(truth["seed"], 24071985);
    assert_eq!(truth["sample_format"], "iq16_le");
    assert_eq!(truth["quantization_bits"], 16);
    let sats = truth["satellites"].as_array().expect("satellites");
    assert_eq!(sats.len(), 2);
    assert!(sats.iter().any(|sat| {
        sat["sat"]["prn"] == 3
            && sat["nav_bit_mode"] == "alternating_gps_lnav20ms"
            && sat["nav_bit_segments"].as_array().map(|segments| segments.len()) == Some(2)
    }));
    assert!(sats.iter().any(|sat| {
        sat["sat"]["prn"] == 7
            && sat["nav_bit_mode"] == "constant_positive"
            && sat["nav_bit_segments"].as_array().map(|segments| segments.len()) == Some(1)
    }));

    let inspect_dir = temp_dir_path("inspect_exported_synthetic_iq");
    fs::create_dir_all(&inspect_dir).expect("create inspect dir");
    let inspect_output = run_bijux(
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
            inspect_dir.to_str().expect("inspect dir"),
        ],
        &repo,
    );
    assert!(
        inspect_output.status.success(),
        "inspect failed: {}",
        String::from_utf8_lossy(&inspect_output.stderr)
    );

    let acquire_dir = temp_dir_path("acquire_exported_synthetic_iq");
    fs::create_dir_all(&acquire_dir).expect("create acquire dir");
    let acquire_output = run_bijux(
        &[
            "gnss",
            "acquire",
            "--unregistered-dataset",
            "--file",
            iq_path.to_str().expect("iq path"),
            "--sidecar",
            sidecar_path.to_str().expect("sidecar path"),
            "--config",
            "configs/receiver_low_rate.toml",
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
        acquire_output.status.success(),
        "acquire failed: {}",
        String::from_utf8_lossy(&acquire_output.stderr)
    );

    let acquire_report: Value = serde_json::from_str(
        &fs::read_to_string(acquire_dir.join("acquire_report.json")).expect("read acquire report"),
    )
    .expect("parse acquire report");
    let search_summary = &acquire_report["search_summary"];
    assert_eq!(search_summary["searched_satellites"], 2);
    assert_eq!(
        search_summary["accepted"].as_u64().unwrap_or(0)
            + search_summary["ambiguous"].as_u64().unwrap_or(0)
            + search_summary["rejected"].as_u64().unwrap_or(0)
            + search_summary["deferred"].as_u64().unwrap_or(0),
        2
    );
    let primary_results = acquire_report["primary_results"]
        .as_array()
        .expect("primary_results");
    assert_eq!(primary_results.len(), 2, "expected one selected acquisition row per searched PRN");
    let results = acquire_report["results"].as_array().expect("results");
    assert!(
        results.len() >= primary_results.len(),
        "candidate results should include the selected primary rows: {acquire_report}"
    );
    let reported_prns = acquire_report["reported_prns"].as_array().expect("reported_prns");
    assert!(
        reported_prns.iter().any(|entry| entry["sat"]["prn"] == 3),
        "reported_prns did not include PRN 3: {acquire_report}"
    );
    let acq_artifact = fs::read_to_string(acquire_dir.join("artifacts").join("acq.jsonl"))
        .expect("read acq artifact");
    assert_eq!(
        acq_artifact.lines().count(),
        results.len(),
        "acq artifact row count should match detailed candidate rows"
    );

    fs::remove_dir_all(&export_dir).expect("remove export dir");
    fs::remove_dir_all(&inspect_dir).expect("remove inspect dir");
    fs::remove_dir_all(&acquire_dir).expect("remove acquire dir");
}
