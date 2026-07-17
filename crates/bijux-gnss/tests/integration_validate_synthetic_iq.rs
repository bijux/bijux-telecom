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

fn export_bundle(repo: &Path, export_dir: &Path, scenario: &Path) {
    fs::create_dir_all(export_dir).expect("create export dir");
    let output = run_bijux(
        &[
            "gnss",
            "export-synthetic-iq",
            "--scenario",
            scenario.to_str().expect("scenario path"),
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
}

fn export_reference_bundle(repo: &Path, export_dir: &Path) {
    export_bundle(
        repo,
        export_dir,
        &repo.join("configs/scenarios/synthetic_iq_cn0_reference.toml"),
    );
}

fn validate_acquisition_reference_bundle(
    repo: &Path,
    scenario_relative_path: &str,
    scenario_id: &str,
    expected_sample_rate_hz: f64,
    expected_receiver_clock_frequency_bias_hz: Option<f64>,
) {
    let export_dir = temp_dir_path(scenario_id);
    export_bundle(repo, &export_dir, &repo.join(scenario_relative_path));

    let validate_dir = temp_dir_path(&format!("{scenario_id}_validation"));
    fs::create_dir_all(&validate_dir).expect("create validate dir");
    let artifacts_dir = export_dir.join("artifacts");
    let iq_path = artifacts_dir.join(format!("{scenario_id}.iq16"));
    let sidecar_path = artifacts_dir.join(format!("{scenario_id}.sidecar.toml"));
    let truth_path = artifacts_dir.join(format!("{scenario_id}.truth.json"));

    let output = run_bijux(
        &[
            "gnss",
            "validate-synthetic-iq",
            "--unregistered-dataset",
            "--file",
            iq_path.to_str().expect("iq path"),
            "--sidecar",
            sidecar_path.to_str().expect("sidecar path"),
            "--truth",
            truth_path.to_str().expect("truth path"),
            "--config",
            "configs/receiver_low_rate.toml",
            "--report",
            "json",
            "--out",
            validate_dir.to_str().expect("validate dir"),
        ],
        repo,
    );
    assert!(
        output.status.success(),
        "validate-synthetic-iq failed for {scenario_id}: {}",
        String::from_utf8_lossy(&output.stderr)
    );

    let report: Value = serde_json::from_str(
        &fs::read_to_string(validate_dir.join("validate_synthetic_iq_report.json"))
            .expect("read validation report"),
    )
    .expect("parse validation report");
    assert_eq!(report["validation"]["scenario_id"], scenario_id);
    assert_eq!(report["validation"]["sample_rate_hz"], expected_sample_rate_hz);
    assert!(report["validation"]["pass"].as_bool().expect("validation pass"));
    assert_eq!(
        report["acquisition_code_phase_validation"]["sample_rate_hz"],
        expected_sample_rate_hz
    );
    assert!(report["acquisition_code_phase_validation"]["pass"]
        .as_bool()
        .expect("acquisition code phase validation pass"));
    assert_eq!(
        report["acquisition_code_phase_refinement_validation"]["sample_rate_hz"],
        expected_sample_rate_hz
    );
    assert!(report["acquisition_code_phase_refinement_validation"]["pass"]
        .as_bool()
        .expect("acquisition code phase refinement validation pass"));
    assert_eq!(report["acquisition_doppler_validation"]["sample_rate_hz"], expected_sample_rate_hz);
    assert!(report["acquisition_doppler_validation"]["pass"]
        .as_bool()
        .expect("acquisition doppler validation pass"));
    assert_eq!(
        report["acquisition_receiver_clock_offset_validation"]["sample_rate_hz"],
        expected_sample_rate_hz
    );
    assert!(report["acquisition_receiver_clock_offset_validation"]["pass"]
        .as_bool()
        .expect("acquisition receiver clock offset validation pass"));
    assert_eq!(
        report["acquisition_code_phase_validation"]["satellites"]
            .as_array()
            .expect("acquisition code-phase rows")
            .len(),
        2
    );
    assert_eq!(
        report["acquisition_doppler_validation"]["satellites"]
            .as_array()
            .expect("acquisition doppler rows")
            .len(),
        2
    );
    assert_eq!(
        report["acquisition_receiver_clock_offset_validation"]["satellites"]
            .as_array()
            .expect("acquisition receiver clock-offset rows")
            .len(),
        2
    );
    if let Some(expected_receiver_clock_frequency_bias_hz) =
        expected_receiver_clock_frequency_bias_hz
    {
        assert_eq!(
            report["acquisition_receiver_clock_offset_validation"]
                ["injected_receiver_clock_frequency_bias_hz"],
            expected_receiver_clock_frequency_bias_hz
        );
        let mean_measured_receiver_clock_frequency_bias_hz = report
            ["acquisition_receiver_clock_offset_validation"]
            ["mean_measured_receiver_clock_frequency_bias_hz"]
            .as_f64()
            .expect("mean measured receiver clock frequency bias");
        assert!(
            (mean_measured_receiver_clock_frequency_bias_hz
                - expected_receiver_clock_frequency_bias_hz)
                .abs()
                <= 500.0 + f64::EPSILON,
            "receiver clock bias out of tolerance: {mean_measured_receiver_clock_frequency_bias_hz}"
        );
    }

    fs::remove_dir_all(&export_dir).expect("remove export dir");
    fs::remove_dir_all(&validate_dir).expect("remove validate dir");
}

#[test]
fn validate_synthetic_iq_accepts_reference_cn0_bundle() {
    let repo = repo_root();
    let export_dir = temp_dir_path("export_synthetic_iq_cn0_reference");
    export_reference_bundle(&repo, &export_dir);

    let validate_dir = temp_dir_path("validate_synthetic_iq_cn0_reference");
    fs::create_dir_all(&validate_dir).expect("create validate dir");
    let artifacts_dir = export_dir.join("artifacts");
    let iq_path = artifacts_dir.join("synthetic_iq_cn0_reference.iq16");
    let sidecar_path = artifacts_dir.join("synthetic_iq_cn0_reference.sidecar.toml");
    let truth_path = artifacts_dir.join("synthetic_iq_cn0_reference.truth.json");

    let output = run_bijux(
        &[
            "gnss",
            "validate-synthetic-iq",
            "--unregistered-dataset",
            "--file",
            iq_path.to_str().expect("iq path"),
            "--sidecar",
            sidecar_path.to_str().expect("sidecar path"),
            "--truth",
            truth_path.to_str().expect("truth path"),
            "--config",
            "configs/receiver_low_rate.toml",
            "--report",
            "json",
            "--out",
            validate_dir.to_str().expect("validate dir"),
        ],
        &repo,
    );
    assert!(
        output.status.success(),
        "validate-synthetic-iq failed: {}",
        String::from_utf8_lossy(&output.stderr)
    );

    let report: Value = serde_json::from_str(
        &fs::read_to_string(validate_dir.join("validate_synthetic_iq_report.json"))
            .expect("read validation report"),
    )
    .expect("parse validation report");
    assert_eq!(report["validation"]["scenario_id"], "synthetic_iq_cn0_reference");
    assert!(report["validation"]["pass"].as_bool().expect("validation pass"));
    assert!(report["acquisition_code_phase_validation"]["pass"]
        .as_bool()
        .expect("acquisition code phase validation pass"));
    assert!(report["acquisition_code_phase_refinement_validation"]["pass"]
        .as_bool()
        .expect("acquisition code phase refinement validation pass"));
    assert!(report["acquisition_doppler_validation"]["pass"]
        .as_bool()
        .expect("acquisition doppler validation pass"));
    let rows = report["validation"]["satellites"].as_array().expect("satellite rows");
    assert_eq!(rows.len(), 1);
    assert_eq!(rows[0]["sat"]["prn"], 7);
    assert!(rows[0]["pass"].as_bool().expect("satellite validation pass"));
    let epochs_measured = rows[0]["epochs_measured"].as_u64().expect("epochs measured");
    assert!(
        epochs_measured >= 5,
        "expected a stable tracking window in the CN0 validation report: {rows:?}"
    );
    let measured_mean_cn0_dbhz =
        rows[0]["measured_mean_cn0_dbhz"].as_f64().expect("measured mean cn0");
    let cn0_delta_db = rows[0]["cn0_delta_db"].as_f64().expect("cn0 delta");
    assert!(
        (48.0..=57.0).contains(&measured_mean_cn0_dbhz),
        "measured mean cn0 drifted outside the expected tracking estimate band: {measured_mean_cn0_dbhz}"
    );
    assert!(cn0_delta_db.abs() <= 5.0, "cn0 delta out of tolerance: {cn0_delta_db}");
    let acquisition_rows = report["acquisition_code_phase_validation"]["satellites"]
        .as_array()
        .expect("acquisition satellite rows");
    assert_eq!(acquisition_rows.len(), 1);
    assert_eq!(acquisition_rows[0]["sat"]["prn"], 7);
    assert!(acquisition_rows[0]["pass"].as_bool().expect("acquisition satellite validation pass"));
    assert_eq!(acquisition_rows[0]["code_phase_error_samples"], 0);
    let acquisition_refinement_rows = report["acquisition_code_phase_refinement_validation"]
        ["satellites"]
        .as_array()
        .expect("acquisition refinement rows");
    assert_eq!(acquisition_refinement_rows.len(), 1);
    assert_eq!(acquisition_refinement_rows[0]["sat"]["prn"], 7);
    assert!(acquisition_refinement_rows[0]["pass"]
        .as_bool()
        .expect("acquisition refinement satellite validation pass"));
    let coarse_error_samples = acquisition_refinement_rows[0]["coarse_error_samples"]
        .as_f64()
        .expect("coarse code phase error");
    let refined_error_samples = acquisition_refinement_rows[0]["refined_error_samples"]
        .as_f64()
        .expect("refined code phase error");
    assert!(
        refined_error_samples <= coarse_error_samples + f64::EPSILON,
        "refined code phase should not regress initialization: {acquisition_refinement_rows:?}"
    );
    let acquisition_doppler_rows = report["acquisition_doppler_validation"]["satellites"]
        .as_array()
        .expect("acquisition doppler rows");
    assert_eq!(acquisition_doppler_rows.len(), 1);
    assert_eq!(acquisition_doppler_rows[0]["sat"]["prn"], 7);
    assert!(acquisition_doppler_rows[0]["pass"]
        .as_bool()
        .expect("acquisition doppler satellite validation pass"));
    let doppler_error_hz =
        acquisition_doppler_rows[0]["doppler_error_hz"].as_f64().expect("doppler error");
    assert!(
        doppler_error_hz <= 500.0 + f64::EPSILON,
        "doppler error out of tolerance: {doppler_error_hz}"
    );
    assert!(validate_dir.join("manifest.json").exists(), "missing validation manifest");

    fs::remove_dir_all(&export_dir).expect("remove export dir");
    fs::remove_dir_all(&validate_dir).expect("remove validate dir");
}

#[test]
fn validate_synthetic_iq_accepts_low_rate_acquisition_reference_bundle() {
    let repo = repo_root();

    validate_acquisition_reference_bundle(
        &repo,
        "configs/scenarios/synthetic_iq_acquisition_reference_low_rate.toml",
        "synthetic_iq_acquisition_reference_low_rate",
        2_046_000.0,
        Some(0.0),
    );
}

#[test]
fn validate_synthetic_iq_accepts_high_rate_acquisition_reference_bundle() {
    let repo = repo_root();

    validate_acquisition_reference_bundle(
        &repo,
        "configs/scenarios/synthetic_iq_acquisition_reference_high_rate.toml",
        "synthetic_iq_acquisition_reference_high_rate",
        4_092_000.0,
        Some(0.0),
    );
}

#[test]
fn validate_synthetic_iq_accepts_acquisition_clock_offset_reference_bundle() {
    let repo = repo_root();

    validate_acquisition_reference_bundle(
        &repo,
        "configs/scenarios/synthetic_iq_acquisition_clock_offset_reference.toml",
        "synthetic_iq_acquisition_clock_offset_reference",
        4_092_000.0,
        Some(500.0),
    );
}

#[test]
fn validate_synthetic_iq_rejects_too_tight_acquisition_code_phase_tolerance() {
    let repo = repo_root();
    let export_dir = temp_dir_path("export_synthetic_iq_fractional_phase_accuracy");
    fs::create_dir_all(&export_dir).expect("create export dir");
    let scenario_path = export_dir.join("fractional_code_phase_accuracy.toml");
    let artifacts_dir = export_dir.join("artifacts");
    let probe_root = temp_dir_path("validate_synthetic_iq_fractional_phase_probe");
    fs::create_dir_all(&probe_root).expect("create probe root");
    let mut max_error = None;

    for (index, code_phase_chips) in
        [200.125, 200.25, 200.375, 200.5, 200.625, 200.75, 200.875].into_iter().enumerate()
    {
        fs::write(
            &scenario_path,
            format!(
                r#"
id = "fractional_code_phase_accuracy"
sample_rate_hz = 4000000.0
intermediate_freq_hz = 0.0
duration_s = 0.04
seed = 24071985

[[satellites]]
sat = {{ constellation = "Gps", prn = 9 }}
doppler_hz = 500.0
code_phase_chips = {code_phase_chips}
carrier_phase_rad = 0.0
cn0_db_hz = 58.0
navigation_data = "constant_positive"
"#
            ),
        )
        .expect("write scenario");
        export_bundle(&repo, &export_dir, &scenario_path);

        let probe_dir = probe_root.join(format!("candidate_{index}"));
        fs::create_dir_all(&probe_dir).expect("create probe dir");
        let probe_output = run_bijux(
            &[
                "gnss",
                "validate-synthetic-iq",
                "--unregistered-dataset",
                "--file",
                artifacts_dir
                    .join("fractional_code_phase_accuracy.iq16")
                    .to_str()
                    .expect("iq path"),
                "--sidecar",
                artifacts_dir
                    .join("fractional_code_phase_accuracy.sidecar.toml")
                    .to_str()
                    .expect("sidecar path"),
                "--truth",
                artifacts_dir
                    .join("fractional_code_phase_accuracy.truth.json")
                    .to_str()
                    .expect("truth path"),
                "--config",
                "configs/receiver_low_rate.toml",
                "--report",
                "json",
                "--out",
                probe_dir.to_str().expect("probe dir"),
            ],
            &repo,
        );
        assert!(
            probe_output.status.success(),
            "initial validate-synthetic-iq failed for code_phase_chips={code_phase_chips}: {}",
            String::from_utf8_lossy(&probe_output.stderr)
        );
        let probe_report: Value = serde_json::from_str(
            &fs::read_to_string(probe_dir.join("validate_synthetic_iq_report.json"))
                .expect("read probe validation report"),
        )
        .expect("parse probe validation report");
        let candidate_error = probe_report["acquisition_code_phase_validation"]["satellites"]
            .as_array()
            .expect("probe acquisition rows")
            .iter()
            .map(|row| row["code_phase_error_samples"].as_u64().expect("code phase error"))
            .max()
            .expect("at least one acquisition row");
        if candidate_error > 0 {
            max_error = Some(candidate_error);
            break;
        }
    }

    let max_error = max_error
        .expect("candidate scenarios did not produce a non-zero acquisition code-phase error");
    let strict_tolerance = (max_error - 1).to_string();

    let strict_dir = temp_dir_path("validate_synthetic_iq_fractional_phase_strict");
    fs::create_dir_all(&strict_dir).expect("create strict dir");
    let strict_output = run_bijux(
        &[
            "gnss",
            "validate-synthetic-iq",
            "--unregistered-dataset",
            "--file",
            artifacts_dir.join("fractional_code_phase_accuracy.iq16").to_str().expect("iq path"),
            "--sidecar",
            artifacts_dir
                .join("fractional_code_phase_accuracy.sidecar.toml")
                .to_str()
                .expect("sidecar path"),
            "--truth",
            artifacts_dir
                .join("fractional_code_phase_accuracy.truth.json")
                .to_str()
                .expect("truth path"),
            "--config",
            "configs/receiver_low_rate.toml",
            "--acquisition-code-phase-tolerance-samples",
            strict_tolerance.as_str(),
            "--report",
            "json",
            "--out",
            strict_dir.to_str().expect("strict dir"),
        ],
        &repo,
    );
    assert!(
        !strict_output.status.success(),
        "validate-synthetic-iq unexpectedly succeeded with strict acquisition code-phase tolerance"
    );
    let stderr = String::from_utf8_lossy(&strict_output.stderr);
    assert!(
        stderr.contains("acq_code_phase="),
        "stderr did not include acquisition code-phase failure details: {stderr}"
    );

    fs::remove_dir_all(&export_dir).expect("remove export dir");
    fs::remove_dir_all(&probe_root).expect("remove probe root");
    fs::remove_dir_all(&strict_dir).expect("remove strict dir");
}

#[test]
fn validate_synthetic_iq_rejects_too_tight_cn0_tolerance() {
    let repo = repo_root();
    let export_dir = temp_dir_path("export_synthetic_iq_cn0_tight_tolerance");
    export_reference_bundle(&repo, &export_dir);

    let validate_dir = temp_dir_path("validate_synthetic_iq_cn0_tight_tolerance");
    fs::create_dir_all(&validate_dir).expect("create validate dir");
    let artifacts_dir = export_dir.join("artifacts");
    let iq_path = artifacts_dir.join("synthetic_iq_cn0_reference.iq16");
    let sidecar_path = artifacts_dir.join("synthetic_iq_cn0_reference.sidecar.toml");
    let truth_path = artifacts_dir.join("synthetic_iq_cn0_reference.truth.json");

    let output = run_bijux(
        &[
            "gnss",
            "validate-synthetic-iq",
            "--unregistered-dataset",
            "--file",
            iq_path.to_str().expect("iq path"),
            "--sidecar",
            sidecar_path.to_str().expect("sidecar path"),
            "--truth",
            truth_path.to_str().expect("truth path"),
            "--config",
            "configs/receiver_low_rate.toml",
            "--tolerance-db-hz",
            "0.1",
            "--report",
            "json",
            "--out",
            validate_dir.to_str().expect("validate dir"),
        ],
        &repo,
    );
    assert!(
        !output.status.success(),
        "validate-synthetic-iq unexpectedly succeeded with tight tolerance"
    );
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(
        stderr.contains("synthetic IQ validation failed"),
        "stderr did not explain failure: {stderr}"
    );
    assert!(
        validate_dir.join("validate_synthetic_iq_report.json").exists(),
        "missing validation report after failure"
    );

    fs::remove_dir_all(&export_dir).expect("remove export dir");
    fs::remove_dir_all(&validate_dir).expect("remove validate dir");
}

#[test]
fn validate_synthetic_iq_rejects_too_tight_acquisition_doppler_tolerance() {
    let repo = repo_root();
    let export_dir = temp_dir_path("export_synthetic_iq_fractional_doppler_accuracy");
    fs::create_dir_all(&export_dir).expect("create export dir");
    let scenario_path = export_dir.join("fractional_doppler_accuracy.toml");
    fs::write(
        &scenario_path,
        r#"
id = "fractional_doppler_accuracy"
sample_rate_hz = 4092000.0
intermediate_freq_hz = 0.0
duration_s = 0.04
seed = 24071985

[[satellites]]
sat = { constellation = "Gps", prn = 7 }
doppler_hz = 875.0
code_phase_chips = 321.0
carrier_phase_rad = 0.2
cn0_db_hz = 58.0
navigation_data = "constant_positive"
"#,
    )
    .expect("write scenario");
    export_bundle(&repo, &export_dir, &scenario_path);

    let validate_dir = temp_dir_path("validate_synthetic_iq_fractional_doppler_default");
    fs::create_dir_all(&validate_dir).expect("create validate dir");
    let artifacts_dir = export_dir.join("artifacts");
    let iq_path = artifacts_dir.join("fractional_doppler_accuracy.iq16");
    let sidecar_path = artifacts_dir.join("fractional_doppler_accuracy.sidecar.toml");
    let truth_path = artifacts_dir.join("fractional_doppler_accuracy.truth.json");

    let default_output = run_bijux(
        &[
            "gnss",
            "validate-synthetic-iq",
            "--unregistered-dataset",
            "--file",
            iq_path.to_str().expect("iq path"),
            "--sidecar",
            sidecar_path.to_str().expect("sidecar path"),
            "--truth",
            truth_path.to_str().expect("truth path"),
            "--config",
            "configs/receiver_low_rate.toml",
            "--report",
            "json",
            "--out",
            validate_dir.to_str().expect("validate dir"),
        ],
        &repo,
    );
    assert!(
        default_output.status.success(),
        "validate-synthetic-iq failed with default doppler tolerance: {}",
        String::from_utf8_lossy(&default_output.stderr)
    );
    let report: Value = serde_json::from_str(
        &fs::read_to_string(validate_dir.join("validate_synthetic_iq_report.json"))
            .expect("read validation report"),
    )
    .expect("parse validation report");
    let acquisition_doppler_rows = report["acquisition_doppler_validation"]["satellites"]
        .as_array()
        .expect("acquisition doppler rows");
    assert_eq!(acquisition_doppler_rows.len(), 1);
    let doppler_error_hz =
        acquisition_doppler_rows[0]["doppler_error_hz"].as_f64().expect("doppler error");
    assert!(doppler_error_hz > 0.0, "scenario did not produce a non-zero doppler error");

    let strict_dir = temp_dir_path("validate_synthetic_iq_fractional_doppler_strict");
    fs::create_dir_all(&strict_dir).expect("create strict dir");
    let strict_output = run_bijux(
        &[
            "gnss",
            "validate-synthetic-iq",
            "--unregistered-dataset",
            "--file",
            iq_path.to_str().expect("iq path"),
            "--sidecar",
            sidecar_path.to_str().expect("sidecar path"),
            "--truth",
            truth_path.to_str().expect("truth path"),
            "--config",
            "configs/receiver_low_rate.toml",
            "--acquisition-doppler-tolerance-bins",
            "0",
            "--report",
            "json",
            "--out",
            strict_dir.to_str().expect("strict dir"),
        ],
        &repo,
    );
    assert!(
        !strict_output.status.success(),
        "validate-synthetic-iq unexpectedly succeeded with strict acquisition doppler tolerance"
    );
    let stderr = String::from_utf8_lossy(&strict_output.stderr);
    assert!(
        stderr.contains("acq_doppler="),
        "stderr did not include acquisition doppler failure details: {stderr}"
    );

    fs::remove_dir_all(&export_dir).expect("remove export dir");
    fs::remove_dir_all(&validate_dir).expect("remove validate dir");
    fs::remove_dir_all(&strict_dir).expect("remove strict dir");
}
