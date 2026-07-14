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

    let truth: Value = serde_json::from_str(&fs::read_to_string(&truth_path).expect("read truth"))
        .expect("parse truth");
    assert_eq!(truth["scenario_id"], "synthetic_iq_reference");
    assert_eq!(truth["seed"], 24071985);
    assert_eq!(truth["sample_format"], "iq16_le");
    assert_eq!(truth["quantization_bits"], 16);
    let sats = truth["satellites"].as_array().expect("satellites");
    assert_eq!(sats.len(), 2);
    assert!(sats.iter().any(|sat| {
        sat["sat"]["prn"] == 3
            && sat["navigation_data"] == "alternating_start_positive"
            && sat["nav_bit_mode"] == "alternating_gps_lnav20ms"
            && sat["nav_bit_segments"].as_array().map(|segments| segments.len()) == Some(2)
    }));
    assert!(sats.iter().any(|sat| {
        sat["sat"]["prn"] == 7
            && sat["navigation_data"] == "constant_positive"
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
    let primary_results = acquire_report["primary_results"].as_array().expect("primary_results");
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

#[test]
fn acquire_report_and_artifact_preserve_ranked_alternatives_per_prn() {
    let repo = repo_root();
    let temp = temp_dir_path("acquisition_ranked_alternatives");
    fs::create_dir_all(&temp).expect("create temp dir");
    let scenario_path = temp.join("ranked_alternatives.toml");
    fs::write(
        &scenario_path,
        r#"id = "ranked_alternatives"
sample_rate_hz = 4092000.0
intermediate_freq_hz = 0.0
duration_s = 0.001
seed = 24071996

[[satellites]]
sat = { constellation = "Gps", prn = 7 }
doppler_hz = 0.0
code_phase_chips = 300.0
carrier_phase_rad = 0.0
cn0_db_hz = 44.0
navigation_data = "constant_positive"

[[satellites]]
sat = { constellation = "Gps", prn = 7 }
doppler_hz = 250.0
code_phase_chips = 300.0
carrier_phase_rad = 0.0
cn0_db_hz = 44.0
navigation_data = "constant_positive"
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

    let iq_path = export_dir.join("artifacts").join("ranked_alternatives.iq16");
    let sidecar_path = export_dir.join("artifacts").join("ranked_alternatives.sidecar.toml");
    let acquire_dir = temp.join("acquire");
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
            "7",
            "--doppler-search-hz",
            "500",
            "--doppler-step-hz",
            "250",
            "--top",
            "4",
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
    assert_eq!(acquire_report["search_summary"]["ambiguous"], 1, "{acquire_report}");

    let results = acquire_report["results"].as_array().expect("results");
    assert!(results.len() >= 2, "{acquire_report}");
    assert_eq!(results[0]["candidate_rank"], 1, "{acquire_report}");
    assert_eq!(results[0]["is_primary_candidate"], true, "{acquire_report}");
    assert_eq!(results[0]["hypothesis"], "ambiguous", "{acquire_report}");
    assert_eq!(results[1]["candidate_rank"], 2, "{acquire_report}");
    assert_eq!(results[1]["is_primary_candidate"], false, "{acquire_report}");
    assert_eq!(results[1]["hypothesis"], "rejected", "{acquire_report}");
    assert!(
        results[1]["selection_reason"]
            .as_str()
            .is_some_and(|reason| reason.starts_with("ranked_alternative:")),
        "{acquire_report}"
    );

    let acq_artifact = fs::read_to_string(acquire_dir.join("artifacts").join("acq.jsonl"))
        .expect("read acq artifact");
    let artifact_rows: Vec<Value> = acq_artifact
        .lines()
        .map(|line| serde_json::from_str(line).expect("parse artifact row"))
        .collect();
    assert!(artifact_rows.len() >= 2);
    assert_eq!(artifact_rows[0]["payload"]["candidate_rank"], 1);
    assert_eq!(artifact_rows[0]["payload"]["is_primary_candidate"], true);
    assert_eq!(artifact_rows[1]["payload"]["candidate_rank"], 2);
    assert_eq!(artifact_rows[1]["payload"]["is_primary_candidate"], false);
    assert_eq!(artifact_rows[1]["payload"]["hypothesis"], "rejected");

    fs::remove_dir_all(&temp).expect("remove temp dir");
}

#[test]
fn acquire_report_exposes_refined_doppler_for_fractional_synthetic_truth() {
    let repo = repo_root();
    let temp = temp_dir_path("acquisition_refinement_report");
    fs::create_dir_all(&temp).expect("create temp dir");
    let scenario_path = temp.join("fractional_acquisition_refinement.toml");
    fs::write(
        &scenario_path,
r#"id = "fractional_acquisition_refinement"
sample_rate_hz = 4092000.0
intermediate_freq_hz = 0.0
duration_s = 0.08
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
    .expect("write refinement scenario");
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

    let iq_path = export_dir.join("artifacts").join("fractional_acquisition_refinement.iq16");
    let sidecar_path =
        export_dir.join("artifacts").join("fractional_acquisition_refinement.sidecar.toml");
    let truth_path =
        export_dir.join("artifacts").join("fractional_acquisition_refinement.truth.json");
    let _truth: Value = serde_json::from_str(&fs::read_to_string(&truth_path).expect("read truth"))
        .expect("parse truth");

    let acquire_config_path = temp.join("receiver_acquisition_refinement.toml");
    fs::write(
        &acquire_config_path,
r#"schema_version = 1
sample_rate_hz = 4_092_000.0
intermediate_freq_hz = 0.0
quantization_bits = 16
code_freq_basis_hz = 1_023_000.0
code_length = 1023
seed = 1

[acquisition]
doppler_search_hz = 10_000
doppler_step_hz = 500
integration_ms = 1
noncoherent_integration = 1
peak_mean_threshold = 3.0
peak_second_threshold = 1.8

[tracking]
early_late_spacing_chips = 0.5
dll_bw_hz = 1.5
pll_bw_hz = 12.0
fll_bw_hz = 8.0
max_channels = 8
per_epoch_budget_ms = 0.7

[navigation]
robust_solver = true
huber_k = 30.0
raim = true
hatch_window = 100
iono_mode = "broadcast"
tropo_enable = true
tropo_ztd_m = 2.3

[navigation.weighting]
enabled = true
mode = "elevation"
min_elev_deg = 5.0
elev_exponent = 2.0
cn0_ref_dbhz = 50.0
min_weight = 0.1
elev_mask_deg = 5.0
tracking_mode_scalar_weight = 1.0
tracking_mode_vector_weight = 1.2
"#,
    )
    .expect("write refinement config");

    let acquire_dir = temp.join("acquire");
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
            acquire_config_path.to_str().expect("acquire config"),
            "--prn",
            "7",
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
    let primary_row = acquire_report["primary_results"]
        .as_array()
        .and_then(|rows| rows.first())
        .expect("primary acquisition row");
    assert_eq!(primary_row["signal_band"], "L1", "{primary_row}");
    assert_eq!(primary_row["source_sample_index"], 0, "{primary_row}");
    let reported_doppler_hz = primary_row["doppler_hz"].as_f64().expect("reported doppler");
    let coarse_carrier_hz =
        primary_row["coarse_carrier_hz"].as_f64().expect("coarse carrier for primary row");
    let refined_carrier_hz = primary_row["carrier_hz"].as_f64().expect("refined carrier");
    let refinement_bins = primary_row["doppler_refinement_bins"].as_f64().expect("refinement bins");
    assert!(
        (refined_carrier_hz - coarse_carrier_hz).abs() > f64::EPSILON,
        "refined carrier should differ from the coarse bin when refinement is present: {primary_row}"
    );
    assert!(
        refinement_bins.abs() <= 0.5,
        "refinement should stay within one half-bin: {primary_row}"
    );

    let acq_artifact = fs::read_to_string(acquire_dir.join("artifacts").join("acq.jsonl"))
        .expect("read acq artifact");
    let artifact_row: Value =
        serde_json::from_str(acq_artifact.lines().next().expect("artifact row"))
            .expect("parse acquisition artifact row");
    let artifact_doppler_hz =
        artifact_row["payload"]["doppler_hz"].as_f64().expect("artifact doppler");
    assert_eq!(artifact_row["payload"]["signal_band"], "L1", "{artifact_row}");
    assert_eq!(artifact_row["payload"]["source_time"]["sample_index"], 0, "{artifact_row}");
    assert_eq!(artifact_doppler_hz, reported_doppler_hz);
    assert_eq!(
        artifact_row["payload"]["doppler_refinement"]["method"], "parabolic_peak",
        "acquisition artifact should expose doppler refinement provenance: {artifact_row}"
    );

    fs::remove_dir_all(&temp).expect("remove temp dir");
}

#[test]
fn acquire_report_exposes_refined_code_phase_for_fractional_synthetic_truth() {
    let repo = repo_root();
    let temp = temp_dir_path("acquisition_code_phase_refinement_report");
    fs::create_dir_all(&temp).expect("create temp dir");
    let scenario_path = temp.join("fractional_code_phase_refinement.toml");
    fs::write(
        &scenario_path,
        r#"id = "fractional_code_phase_refinement"
sample_rate_hz = 4000000.0
intermediate_freq_hz = 0.0
duration_s = 0.08
seed = 24071985

[[satellites]]
sat = { constellation = "Gps", prn = 3 }
doppler_hz = 0.0
code_phase_chips = 200.125
carrier_phase_rad = 0.0
cn0_db_hz = 65.0
navigation_data = "constant_positive"
"#,
    )
    .expect("write refinement scenario");
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

    let acquire_dir = temp.join("acquire");
    fs::create_dir_all(&acquire_dir).expect("create acquire dir");
    let acquire_output = run_bijux(
        &[
            "gnss",
            "acquire",
            "--unregistered-dataset",
            "--file",
            export_dir
                .join("artifacts")
                .join("fractional_code_phase_refinement.iq16")
                .to_str()
                .expect("iq path"),
            "--sidecar",
            export_dir
                .join("artifacts")
                .join("fractional_code_phase_refinement.sidecar.toml")
                .to_str()
                .expect("sidecar path"),
            "--config",
            "configs/receiver_low_rate.toml",
            "--prn",
            "3",
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
    let primary_row = acquire_report["primary_results"]
        .as_array()
        .and_then(|rows| rows.first())
        .expect("primary acquisition row");
    let coarse_code_phase_samples =
        primary_row["code_phase_samples"].as_u64().expect("coarse code phase samples") as f64;
    let refined_code_phase_samples =
        primary_row["refined_code_phase_samples"].as_f64().expect("refined code phase");
    let code_phase_refinement_samples =
        primary_row["code_phase_refinement_samples"].as_f64().expect("code phase refinement");
    assert!(
        (refined_code_phase_samples - coarse_code_phase_samples).abs() > f64::EPSILON,
        "refined code phase should differ from the coarse sample when refinement is present: {primary_row}"
    );
    assert!(
        code_phase_refinement_samples.abs() <= 0.5 + f64::EPSILON,
        "code phase refinement should stay within one half-sample: {primary_row}"
    );

    let acq_artifact = fs::read_to_string(acquire_dir.join("artifacts").join("acq.jsonl"))
        .expect("read acq artifact");
    let artifact_row: Value =
        serde_json::from_str(acq_artifact.lines().next().expect("artifact row"))
            .expect("parse acquisition artifact row");
    assert_eq!(
        artifact_row["payload"]["code_phase_refinement"]["method"], "quadratic_likelihood_surface",
        "acquisition artifact should expose code-phase refinement provenance: {artifact_row}"
    );

    fs::remove_dir_all(&temp).expect("remove temp dir");
}
